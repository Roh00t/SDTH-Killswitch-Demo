# Killswitch — Safety & Reliability Guardrails

**This document is binding.** Where it conflicts with convenience, performance, or a
demo deadline, this document wins.

Killswitch commands a physical effector. On the bench that is a low-power proxy laser;
the architecture is explicitly designed to re-host onto a high-energy effector. Every
habit built now is a habit that carries forward. Build them correctly.

**Severity:**
`[HARD]` — inviolable. A violation is a stop-work defect.
`[STRICT]` — deviation requires a documented, reviewed justification.
`[GUIDE]` — strong default; deviate with a comment explaining why.

---

## 1. Concurrency & Threading

The legacy codebase mutated one global dict from three threads with no synchronisation.
That is the single largest source of latent defects we are removing.

### Thread inventory

| Thread | Owns | May not |
|---|---|---|
| **Main / state machine** | `EngagementState`, orchestration | Block on I/O |
| **Grab thread** (daemon) | `cv2.VideoCapture`, latest frame | Touch state or actuator |
| **Vision worker** | Detector, tracker, predictor | Touch serial or state directly |
| **MQTT callback** (paho) | Inbound payload validation | Do work; must enqueue and return |
| **Serial writer** | `pyserial` handle, heartbeat | Touch state |

### Rules

- `[HARD]` **One lock owns state.** `EngagementState` changes only inside the state
  machine's transition method holding its `threading.RLock`. No exceptions, no
  convenience setters, no direct assignment from callbacks.
- `[HARD]` **MQTT callbacks do not act.** Paho dispatches on its own network thread. A
  callback validates, enqueues, and returns. It never transitions state, drives the
  actuator, or performs I/O inline. A blocked callback stalls the MQTT client.
- `[HARD]` **One writer on the serial port.** `SerialActuator` owns the handle and a
  `threading.Lock` around every write. Two threads interleaving writes corrupts framing.
- `[HARD]` **No thread-per-command.** The legacy code spawned a thread per servo
  movement, producing unsynchronised read-add-write races against the same servo object.
  Commands go to a single queue drained by the serial writer.
- `[STRICT]` **Frames are handed off immutably.** The grab thread publishes a reference;
  consumers treat it as read-only or copy. Never mutate a frame another thread may hold.
- `[STRICT]` **No `time.sleep()` in the state machine thread.** Timers are deadline
  comparisons against `time.monotonic()`, checked each tick. Sleeping blocks fail-safes.
- `[STRICT]` **`time.monotonic()` for all durations.** `time.time()` jumps with NTP and
  DST. A clock step during `HOLD` must not extend or truncate a burn.
- `[GUIDE]` Bound every queue. An unbounded queue turns backpressure into memory
  exhaustion; prefer dropping stale frames to queueing them.

---

## 2. Physical Safety — The Laser

**The default state of the effector is OFF. Everything else is an exception that must be
actively and continuously justified.**

### Hardware layer

- `[HARD]` **10 kΩ pulldown from the laser gate GPIO to GND.** Between power-on and the
  first line of `setup()`, every ESP32 GPIO is a floating input. Floating gate = undefined
  laser state during boot, reflash, brownout, and crash. Software cannot fix this. The
  resistor can.
- `[HARD]` **Laser driven through a MOSFET/transistor, never from GPIO directly.**
- `[HARD]` **A physical interlock in series** — key switch or removable link. The operator
  must be able to make firing impossible without touching software.
- `[STRICT]` Laser gate GPIO is not a strapping pin (not GPIO 0/3/45/46).
- `[STRICT]` Eye protection rated for the proxy laser's wavelength and class is worn by
  everyone present during any powered test. Beam path terminates in a beam dump or matte
  backstop — never an open room, a window, or anything reflective.

### Firmware layer (ESP32 — the authority)

- `[HARD]` `setup()` sets the laser pin `OUTPUT` and `LOW` **before** anything else.
- `[HARD]` **250 ms deadman timer.** No valid command within the window → laser LOW,
  unconditionally. A host crash, Python exception, GIL stall, GC pause, or unplugged
  cable all resolve to beam-off without the host participating.
- `[HARD]` **Two-key arming.** `L1` is rejected unless the subsystem was separately armed
  via `M1`. A single corrupted byte must not be able to fire.
- `[HARD]` **Hard burn ceiling in firmware.** The ESP32 independently cuts the beam at
  the maximum burn duration (2.0 s bench) even if the host commands longer. The host is
  not trusted to time the burn.
- `[HARD]` `Z` (e-stop) is processed ahead of any queued command. Laser LOW, latch
  disarmed until an explicit re-arm.
- `[STRICT]` Any parse error, range violation, or checksum failure → laser LOW + `ERR`.
  Never guess at a malformed command's intent.

### Host layer

- `[HARD]` `ENGAGE` is reachable **only** from `OPERATOR_AUTH`. One predecessor. The
  transition table forbids every other path and raises on attempt.
- `[HARD]` Authorisation is bound to `target_id`. Auth for track 7 cannot fire on track 9.
  Re-acquisition after loss produces a new ID and requires fresh authorisation.
- `[HARD]` **Every exception handler that can run while armed de-energises first, then
  handles.** Laser-off is the first statement, not the cleanup.
- `[HARD]` Exit from `ENGAGE` — normal, fault, or exception — sends laser-off and
  **confirms via status frame** before entering `IDLE`. Fire-and-forget is not adequate
  for an effector command.
- `[STRICT]` Loss of lock during `ENGAGE` cuts the beam immediately. Do not coast.
- `[STRICT]` Serial disconnect, MQTT disconnect, or camera loss while armed → immediate
  transition to `IDLE`.
- `[STRICT]` `MockActuator` and `SerialActuator` enforce identical safety logic. Tests
  must exercise the real rules.

---

## 3. Actuator Bounds

### Bounds

| Axis | Min | Max | Rationale |
|---|---|---|---|
| Pan | 0° | 180° | SG90 mechanical range |
| Tilt | 45° | 135° | Gimbal geometry; prevents frame collision |

- `[HARD]` **Enforced twice** — host-side before transmit, ESP32-side before PWM write.
  Host-side alone is one bug away from a stalled, overheating servo.
- `[HARD]` Clamp, never wrap. 190° becomes 180°, never 10°.
- `[HARD]` Reject non-finite values. `NaN` into a PWM calculation is undefined behaviour
  at the hardware.

### Sweep — the legacy bug

Legacy `scan_handle_x` called `turn_servo_x(+rate)` in **both** bounds-recovery branches
(`main_helper.py:51` and `:59`), so hitting the upper bound commanded further travel into
it. Required logic:

- `[HARD]` Check whether the **next** step would exceed a bound **before** taking it.
  Never step out and correct — a stalled SG90 draws locked-rotor current and cooks.
- `[HARD]` Reversal flips direction and steps the orthogonal axis. It never re-commands
  the same direction.
- `[HARD]` **Bounded sweep cycles.** Count complete cycles; N without detection → `IDLE`.
  No unbounded search loop exists anywhere in the system.

### Rate limiting

- `[STRICT]` Clamp commanded angular step per tick. SG90s stall and jitter under large
  step commands; the predictor is allowed to ask for more than the servo can deliver.
- `[STRICT]` Do not command below the deadband (~1–2°). Sub-deadband commands produce
  buzz and current draw with no motion.
- `[GUIDE]` Detach or idle PWM when stationary in `IDLE` for an extended period. SG90s
  hunt around setpoint and draw current indefinitely.

---

## 4. Vision & Model Gates

- `[HARD]` **Resolution gate on aimpoint offset.** Below the configured minimum box
  dimension, revert to centre-of-mass and log the downgrade. Applying a 0.35 offset to a
  box whose jitter exceeds it is aiming at noise.
- `[HARD]` Clamp offsets to ±0.5. The aimpoint may never leave the detected box.
- `[STRICT]` A detection below the confidence floor is not a detection. Do not let a
  low-confidence box start a `HOLD` timer.
- `[STRICT]` **Report per-class recall, not aggregate mAP**, in any document or slide. A
  model with strong overall mAP and zero recall on a confuser class is a model that will
  engage the confuser. If a negative class reads 0.0 recall, that is a stop-ship defect
  for an autonomous engagement claim — fix the class balance or disclose it explicitly.
- `[STRICT]` Track ID continuity is a precondition for `HOLD`. An ID switch mid-hold
  resets the timer — it is a different object until proven otherwise.
- `[GUIDE]` Log every frame that triggers a transition. Post-hoc review needs the images,
  not just the decisions.

---

## 5. Comms Security & Validation

The legacy handler that could switch on the laser carried the comment *"Only valid
messages should be received, so no need to make checks."* That comment is the template
for how this goes wrong.

### Validation

- `[HARD]` **Every inbound payload is schema-validated before it can influence state.**
  Reject unknown fields, wrong types, and missing required keys.
- `[HARD]` Range-check `slew_to_cue`: `azimuth` ∈ [0, 360), `elevation` ∈ [-90, 90], both
  finite. `target_id` non-empty, length-capped, charset-restricted.
- `[HARD]` Reject malformed JSON silently to the sender and loudly to the log. Never
  partially apply a payload.
- `[HARD]` Cap payload size before parsing.
- `[STRICT]` Validation failures are logged with source and raw payload, and counted. A
  spike in rejects is an attack indicator.

### Authorisation

- `[HARD]` `c2/operator/auth` requires `auth: true` **and** a `target_id` matching the
  currently tracked ID **and** a valid token. All three, or no engagement.
- `[HARD]` Auth is single-use and expires. An authorisation accepted at T is invalid at
  T+10 s. No replay.
- `[HARD]` Auth received in any state other than `OPERATOR_AUTH` is discarded and logged.
  It is never buffered for later application.
- `[STRICT]` No inbound topic can command the laser directly. The only path to the
  effector is through the state machine. There is no remote `laser:1`.

### Transport

- `[STRICT]` The demo runs on an isolated network, documented as such. Plaintext MQTT on
  a shared LAN with a live effector is not acceptable.
- `[STRICT]` Production requires mTLS, per-client credentials, and topic ACLs. State this
  in the proposal rather than letting a judge raise it.
- `[GUIDE]` Publish heartbeat telemetry so C2 can detect a dead node.

---

## 6. Failure Modes

| Failure | Detection | Response |
|---|---|---|
| Host process dies | ESP32 deadman (250 ms) | Laser LOW in firmware |
| Serial cable pulled | Write exception / timeout | Laser LOW, → `IDLE` |
| Camera disconnect | Grab returns false N times | → `IDLE`, log, attempt reopen |
| MQTT broker down | paho disconnect callback | → `IDLE`; no cue means no engagement |
| Track lost in `HOLD` | Tracker reports no ID | Reset hold timer, → `TRACK` |
| Track lost in `ENGAGE` | Tracker reports no ID | **Beam off immediately**, → `IDLE` |
| Auth timeout | 10 s deadline | → `TRACK`, log denial |
| Servo stall | Commanded bound repeatedly | Stop commanding axis, → `IDLE`, alert |
| Illegal transition | State machine guard | Raise, log, force `IDLE` |
| Unhandled exception | Top-level handler | **De-energise first**, log traceback, → `IDLE` |

`[HARD]` **`IDLE` is the universal safe harbour.** Every fault path terminates there with
the effector confirmed de-energised. There is no failure mode whose response is "continue
and hope" — which is precisely what the legacy `except Exception: print(); continue`
pattern did.
