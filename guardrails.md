# Killswitch — Safety & Reliability Guardrails

**This document is binding.** Where it conflicts with convenience, performance, or a
demo deadline, this document wins.

The bench effector is an eye-safe LED. Every guardrail is sized for a hazardous effector
on purpose: the architecture claims this brain re-hosts onto a real one, and that claim
is only credible if the safety logic was built for it from the start.

`[HARD]` inviolable · `[STRICT]` deviation needs documented justification · `[GUIDE]` strong default

---

## 1. The ESP32 Is the Safety Authority

**The host proposes. The firmware decides.** This is the single most important structural
property of the system, and it is what makes the safety story credible to a defence
audience.

The host is Python: it can stall on the GIL, pause for garbage collection, be descheduled
by the OS, crash, or hang on a blocking call. None of that can extend a burn, because
none of the safety decisions live there.

### Enforced in firmware, independent of the host

| Guard | Value | Measured | Behaviour |
|---|---|---|---|
| Deadman timeout | 250 ms | **254 ms** | No valid command → effector LOW, disarm |
| Burn ceiling | 2000 ms | **2002 ms** | Cut regardless of what the host commands |
| Boot e-stop latch | — | — | Latched at boot; refuses to fire until deliberately armed |
| Two-key arming | — | — | `L1` rejected (E04) without a prior `M1` |
| Bounds clamp | pan 0–180, tilt 45–135 | — | Clamped again before PWM write |

- `[HARD]` `setup()` sets `PIN_EFFECTOR` OUTPUT and LOW as its **literal first statement**,
  before `Serial.begin()`, before servo attach, before anything that can block or fault.
- `[HARD]` **250 ms deadman.** A host crash, exception, GIL stall, GC pause or unplugged
  cable all resolve to effector-off without the host participating.
- `[HARD]` **Burn ceiling in firmware.** The host is not trusted to time its own burn.
- `[HARD]` **`Z` (e-stop) is processed ahead of everything**, including its own validation.
- `[HARD]` Any parse error, range violation or checksum failure → effector LOW + `ERR`.
  Never guess at a malformed command's intent.
- `[HARD]` RX overflow discards the line and resynchronises on the next newline. Never
  act on a truncated command.

### Soft latch vs. the human barrier

The boot/e-stop latch is a **soft** latch: a checksummed `M1` clears it. It exists so a
booted-or-faulted board refuses to fire until something deliberately and verifiably arms
it — not to make recovery require a power cycle.

`[HARD]` The barrier requiring human action is the **physical interlock in series with
the effector** — a key switch or removable link. No software path can clear it. On the
bench with an LED this is optional; with any hazardous effector it is mandatory.

`[HARD]` `MockActuator` mirrors these semantics exactly. It once raised on
arm-after-e-stop while the firmware cleared the latch — the mock was validating fiction.
If firmware behaviour changes, the mock changes in the same commit.

### Hardware layer

- `[HARD]` **10 kΩ pulldown from GPIO 7 to GND.** Between power-on and the first line of
  `setup()`, every ESP32 GPIO is a floating input. A floating gate is an undefined
  effector state during boot, reflash, brownout and crash. Software cannot fix this.
- `[HARD]` Hazardous effectors drive through a MOSFET, never GPIO directly. (An LED at
  <20 mA through 220 Ω is the documented exception.)
- `[STRICT]` Servos on a **separate 5 V rail with common ground.** Two SG90s stall-draw
  ~700 mA each and brown out a shared rail under load — i.e. during tracking, i.e. on stage.
- `[STRICT]` Effector gate is not a strapping pin (not GPIO 0/3/45/46).

### Host layer

- `[HARD]` `ENGAGE` is reachable **only** from `OPERATOR_AUTH`. One predecessor, enforced
  by the transition table, asserted by test.
- `[HARD]` Authorisation binds to `target_id` **and** consumes a single-use `nonce`.
  Auth for track 7 cannot fire on track 9; a replayed nonce is rejected.
- `[HARD]` Every exception handler that can run while armed **de-energises first, then
  handles.** Effector-off is the first statement, not the cleanup.
- `[HARD]` Exit from `ENGAGE` sends effector-off and **confirms via status frame**
  before entering IDLE. Fire-and-forget is not adequate for an effector command.
- `[STRICT]` Lock loss during `ENGAGE` cuts the beam immediately. Do not coast.
- `[STRICT]` Loss of serial, MQTT or camera while past IDLE → immediate IDLE.

---

## 2. Comms Integrity — Asymmetric Checksums

Commands that **increase** hazard require an XOR checksum. Commands that **decrease**
hazard are accepted unconditionally.

| Command | Checksum | Rationale |
|---|---|---|
| `M1` arm | **required** — `M1*7C` | A corrupted byte must not arm |
| `L1` fire | **required** — `L1*7D` | A corrupted byte must not fire |
| `M0` disarm | none | A corrupted byte must not block a disarm |
| `L0` cease | none | A corrupted byte must not block a shutdown |
| `Z` e-stop | none | Must never be rejected, for any reason |

- `[HARD]` Energising requires **two independently checksum-valid commands in order**.
- `[HARD]` The checksum is XOR of the body bytes, two uppercase hex digits.
  `helper/hardware/protocol.py::checksum` and the firmware's `computeChecksum` must agree;
  `test_checksum_matches_firmware_xor` pins both.
- `[STRICT]` **Residual risk, named:** a corrupted `L0` is not checksum-protected and may
  fail to parse, leaving the effector energised. The backstops are the 250 ms deadman and
  the 2.0 s burn ceiling, so worst case is ~250 ms of unintended emission. Do not "fix"
  this by checksumming shutdowns — that trades a bounded risk for an unbounded one.

---

## 3. MQTT Payload Validation

The legacy handler that could switch on the effector carried the comment *"Only valid
messages should be received, so no need to make checks."* Every inbound payload is
hostile until proven otherwise.

- `[HARD]` Size-cap **before** parsing (4096 bytes).
- `[HARD]` Schema-validate: reject missing keys **and unknown keys** (schema drift).
- `[HARD]` Range-check `azimuth` ∈ [0,360), `elevation` ∈ [-90,90], both **finite** —
  NaN and Infinity are legal JSON and must never reach a PWM calculation.
- `[HARD]` Reject `bool` where a number is expected. `isinstance(True, int)` is True in
  Python; a naive numeric check accepts `{"azimuth": true}` as 1.0.
- `[HARD]` `target_id` restricted to `[A-Za-z0-9_-]{1,64}` — these strings reach log
  lines, filenames and operator displays.
- `[HARD]` No inbound topic commands the effector directly. The only path is through the
  state machine. There is no remote `laser:1`.
- `[STRICT]` Never echo a supplied token into a rejection message or log.
- `[STRICT]` Count rejections. A spike is an attack indicator.
- `[STRICT]` Demo runs on an isolated network, documented as such. Production requires
  mTLS, per-client credentials and topic ACLs.

---

## 4. Actuator Bounds

| Axis | Min | Max | Rationale |
|---|---|---|---|
| Pan | 0° | 180° | SG90 mechanical range |
| Tilt | 45° | 135° | Gimbal geometry; prevents frame collision |

- `[HARD]` **Enforced twice** — host-side before transmit, firmware before PWM write.
  Host-side alone is one bug away from a stalled, overheating servo.
- `[HARD]` **Clamp, never wrap.** 190° becomes 180°, never 10°.
- `[HARD]` Reject non-finite values before they reach a PWM calculation.
- `[STRICT]` Slew-rate limiting in firmware (~250°/s) bounds inrush current and avoids
  the large steps that stall an SG90.
- `[STRICT]` Do not command below the ~1–2° deadband — it produces buzz and current draw
  with no motion.

### Sweep — the legacy bug, and why the fix is shaped this way

Legacy `scan_handle_x` called `turn_servo_x(+rate)` in **both** bounds-recovery branches
(`main_helper.py:51` and `:59`), so hitting the upper bound commanded further travel into
it. A stalled SG90 draws locked-rotor current and cooks.

- `[HARD]` Test whether the **next** step would exceed a bound **before** taking it.
  Never step out and correct. `SweepController.step()` implements this.
- `[HARD]` Reversal flips direction and steps the orthogonal axis. It never re-commands
  the same direction.
- `[HARD]` **Bounded search.** `max_cycles` complete traverses, then IDLE. No unbounded
  search loop exists anywhere in the system.
- Verified by a 2000-step fuzz asserting bounds are never exceeded, a test that reversal
  moves *away* from the bound, and a test that the search terminates.

---

## 5. Vision & Model Gates

- `[HARD]` **Resolution gate.** Below `min_box_px`, revert to centre-of-mass and record
  the downgrade in `AimPoint.reason`. An offset smaller than box jitter aims at noise.
- `[HARD]` Clamp offsets to ±0.5 — the aimpoint may never leave its detected box.
- `[HARD]` Track identity switch during `HOLD` resets the timer. A new `track_id` is a
  different object until proven otherwise.
- `[STRICT]` A detection below the confidence floor is not a detection and must not start
  a HOLD timer.
- `[STRICT]` **Report per-class recall, not aggregate mAP.** A model with strong overall
  mAP and zero recall on a confuser class is a model that will engage the confuser. If a
  negative class reads 0.0 recall, that is a stop-ship defect for an autonomous
  engagement claim — fix the class balance or disclose it explicitly.
- `[STRICT]` Cap the applied lead (`max_lead_px`). An unbounded lead from a noisy velocity
  estimate slews the gimbal off target.

---

## 6. Failure Modes

| Failure | Detection | Response |
|---|---|---|
| Host process dies | ESP32 deadman (254 ms measured) | Effector LOW in firmware |
| Serial cable pulled | Write exception / read fault | Effector LOW, → IDLE |
| Camera disconnect | 30 consecutive grab failures | → IDLE, log, `is_healthy` False |
| MQTT broker down | paho disconnect callback | → IDLE; no cue means no engagement |
| Track lost in `HOLD` | No target in snapshot | Hold timer reset, → TRACK |
| Track lost in `ENGAGE` | No target in snapshot | **Beam off immediately**, → IDLE |
| Identity switch in `HOLD` | `track_id` mismatch | Timer reset, → TRACK |
| Auth timeout | 10 s deadline | → TRACK, denial logged |
| Cue outside gimbal arc | `cue_to_gimbal` returns None | Cue rejected + audit event |
| Illegal transition | Transition table guard | Raise → log → force IDLE |
| Unhandled exception | Top-level tick guard | **De-energise first**, log, force IDLE |

`[HARD]` **IDLE is the universal safe harbour.** Every fault path terminates there with
the effector confirmed de-energised. No failure mode's response is "continue and hope" —
which is exactly what the legacy `except Exception: print(); continue` did.
