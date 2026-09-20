# Killswitch — System Architecture

**Project:** Software-Defined Directed Energy (SDDE) Counter-UAS Node
**Event:** Singapore Defence Tech Hackathon (SDTH)
**Status:** Active refactor from legacy Raspberry Pi bird-deterrent codebase

---

## 1. System Overview

Killswitch is a **targeting brain, not a turret.**

The thesis: fielded C-UAS directed-energy systems cost upward of US$1.5M per node as
closed, vertically integrated hardware. At that price coverage does not scale, and a
saturating salvo routes around the few nodes a nation can afford. The binding constraint
is cost-per-node and vendor lock-in — not beam power.

So every hardware-specific concern sits behind a driver interface:

```
        ┌──────────────────────────────────────────┐
        │   TARGETING BRAIN  (portable, the IP)    │
        │   detect → track → predict → aim → gate  │
        └──────────────────────────────────────────┘
                  │              │              │
           SensorDriver    ActuatorDriver   EffectorDriver
                  │              │              │
        ┌─────────┴───┐   ┌──────┴─────┐  ┌─────┴──────┐
        │ USB camera  │   │ ESP32+SG90 │  │ proxy laser│   ← today, ~US$200
        │ MWIR imager │   │ beam dir.  │  │ HEL        │   ← operational, same brain
        └─────────────┘   └────────────┘  └────────────┘
```

**The decoupling is the product.** Identical targeting logic must run against the bench
rig and against a military beam director with no change to the control logic — only a
driver swap and a retune of loop constants.

### Design principles

1. **Hardware agnosticism at the interface.** Abstract base classes define the contract.
   Concrete drivers are swappable at construction. Minimum two live implementations of
   every ABC at all times, so the abstraction is exercised and not theoretical.
2. **Human authority in the control flow, not bolted on.** `OPERATOR_AUTH` is a state,
   not a callback. No path reaches `ENGAGE` without traversing it.
3. **Fail to safe, always.** Every error path, timeout, and lost-lock condition
   de-energises the effector before doing anything else.
4. **Measured, not assumed.** Latency, pointing error, and hold duration are logged
   quantities. No claim in the pitch that is not backed by a number in a log file.

---

## 2. Hardware Topology

Compute is split across a **Host** and an **Edge Actuator**, joined by a serial boundary.

```
┌───────────────────────────────────────────────┐
│ HOST COMPUTE  (laptop/PC — Python 3.9+)       │
│                                               │
│  HBVCam-3M2111 V22 ──USB──▶ FrameGrabber      │
│                              │ (grab-thread)  │
│                              ▼                │
│                        Vision Pipeline        │
│                        (YOLO → track → KF)    │
│                              │                │
│   MQTT ◀──── C2 cue / operator auth           │
│                              ▼                │
│                        State Machine          │
│                              │                │
└──────────────────────────────┼────────────────┘
                               │ USB-CDC / UART
                    ASCII line protocol, 250 ms deadman
                               ▼
┌───────────────────────────────────────────────┐
│ EDGE ACTUATOR  (ESP32-S3-WROOM-1, C++)        │
│                                               │
│  Parser ─▶ Bounds clamp ─▶ LEDC PWM ─▶ SG90×2 │
│         └─▶ Laser gate ──▶ MOSFET ──▶ laser   │
│         └─▶ Watchdog (kills laser on silence) │
└───────────────────────────────────────────────┘
```

### Why split at all

The serial boundary is not an implementation detail — it is a **deliberate rehearsal of
the real interface.** A military beam director is a separate subsystem behind a command
link. By forcing our aiming commands through a constrained, latency-bearing, bandwidth-
limited channel today, we prove the brain works without shared memory to the effector.
That is the portability claim, demonstrated rather than asserted.

It also puts the safety-critical laser gate on a microcontroller that cannot be blocked
by Python's GIL, a garbage-collection pause, or an OS scheduler decision.

### Pin map (ESP32-S3-WROOM-1)

| Function | GPIO | Notes |
|---|---|---|
| Servo PAN (azimuth) | 5 | LEDC channel 0, 50 Hz |
| Servo TILT (elevation) | 6 | LEDC channel 1, 50 Hz |
| Laser gate | 7 | **10 kΩ external pulldown to GND, mandatory** |
| Status LED | 48 | Onboard RGB on most S3 devkits; verify variant |

**Avoid:** GPIO 0, 3, 45, 46 (strapping pins — indeterminate at boot), GPIO 19/20 (native
USB D-/D+), GPIO 26–32 (SPI flash/PSRAM; also 33–37 on octal-PSRAM variants).

**The pulldown is a safety requirement, not a preference.** Between power-on and the
first line of `setup()`, every ESP32 GPIO is a floating input. A floating gate on a laser
driver is an undefined output state. The resistor guarantees LOW through reset, reflash,
brownout, and crash. See `guardrails.md` §2.

### Actuator characterisation — read this before tuning

The SG90 is a hobby servo and it is the **dominant pole of the control loop:**

- **No position feedback.** Three wires, no output. Commanded angle is the only angle we
  know. The actuator runs **open-loop**; the loop is closed optically, by the camera
  observing the result.
- ~100 ms per 60° of travel, plus startup deadband. Small corrections are disproportionately
  slow relative to their size.
- Deadband ≈ 5–10 µs pulse width → ~1–2° of commanded resolution that does nothing.
- Jitter and buzz under load; no torque holding guarantee.

Consequences, which propagate into every other document:
1. The Kalman predictor must lead by the **full glass-to-photon latency**, including
   mechanical travel — not just compute latency.
2. `HOLD` tolerance (±15 px) must be wider than servo jitter, or the state will chatter.
3. "Servo telemetry" means **commanded** position echoed by the ESP32. It is not a
   measurement. Never present it as one.

---

## 3. Serial Protocol

Newline-terminated ASCII. Human-readable so it can be driven from a serial monitor during
bring-up and debugged without tooling.

**Host → ESP32**

| Command | Meaning |
|---|---|
| `A<pan>,<tilt>\n` | Absolute angle, degrees, 1 decimal. e.g. `A090.0,045.5` |
| `L<0\|1>\n` | Laser gate. `L1` requires ARMED state + valid auth token |
| `M<0\|1>\n` | Arm / disarm the laser subsystem |
| `P\n` | Ping — refreshes the deadman timer |
| `S\n` | Status request |
| `Z\n` | **Emergency stop.** Laser LOW, servos hold, latch disarmed |

**ESP32 → Host**

| Response | Meaning |
|---|---|
| `OK <echo>\n` | Command accepted and applied |
| `ST <pan>,<tilt>,<laser>,<armed>,<uptime_ms>\n` | Status frame |
| `ERR <code> <detail>\n` | Rejected — see guardrails §5 for codes |

**Transport note:** the ESP32-S3-WROOM-1 exposes both a native USB-OTG port (USB CDC,
where the baud setting is nominal and ignored) and a UART bridge port on most devkits.
Prefer **native USB CDC** — higher throughput, no bridge chip latency. If using the UART
bridge, set 921600 baud. Confirm which port you are cabled into before tuning.

**Deadman timer:** the ESP32 de-energises the laser if no valid command arrives within
**250 ms**. The host sends `P` at minimum 10 Hz whenever armed. A host crash, a Python
exception, an unplugged cable, or a hung GIL therefore all resolve to laser-off within
a quarter second, without the host participating.

---

## 4. Software Components

### 4.1 Vision Module (`helper/vision/`)

| Component | Responsibility |
|---|---|
| `FrameSource` (ABC) | Frame acquisition contract |
| `UsbCameraSource` | `cv2.VideoCapture` + dedicated grab-thread (see below) |
| `Detector` (ABC) | `detect(frame) -> list[Detection]` |
| `UltralyticsDetector` | YOLO `.pt` via ultralytics — training/GPU path |
| `OnnxDetector` | ONNX Runtime — portable/CPU deployment path |
| `Tracker` | ByteTrack — identity persistence across frames and dropouts |
| `AimpointSolver` | Bounding box + normalised offset → pixel aimpoint |
| `TrajectoryPredictor` | Kalman filter — leads target by measured loop latency |

**`Detection` is the widened contract.** The legacy `YOLOv5` ABC returned bare `(x, y)`
centre tuples, discarding `w` and `h` at the point of computation. Weak-point offsetting
is arithmetically impossible against that interface. The new contract carries the full
box, confidence, class, and track ID.

**Aimpoint solving:**

```
aim_x = x_center + (offset_x * w)
aim_y = y_center + (offset_y * h)
```

`offset = (0.0, 0.0)` is centre-of-mass. `(-0.35, -0.35)` biases toward a forward rotor
hub; `(0.0, 0.4)` toward a slung payload. Offsets are clamped to ±0.5 so the aimpoint can
never leave the detected box.

**Resolution gate — mandatory.** The offset only carries information when it exceeds
frame-to-frame box jitter. Below a configured minimum box dimension the solver reverts to
centre-of-mass and reports the downgrade. Aiming at a sub-component of a 10-pixel box is
aiming at noise, and claiming otherwise is the fastest way to lose a technical panel.

**Zero-latency frame acquisition.** `cv2.CAP_PROP_BUFFERSIZE = 1` is honoured by the V4L2
and DSHOW backends and **ignored by AVFoundation on macOS.** It is set as a best-effort
hint, but the load-bearing mechanism is a daemon thread calling `cap.grab()` continuously
and `cap.retrieve()` only when the pipeline asks. Stale frames are discarded inside the
driver. This is platform-independent and is the only mechanism we rely on.

Also request MJPG explicitly (`CAP_PROP_FOURCC`). UVC cameras default to YUYV, which
saturates USB 2.0 bandwidth and collapses to ~5 fps at higher resolutions.

### 4.2 Comms Module (`helper/comms/`)

MQTT is the **C2 plane** — cueing and authorisation only. It is never in the inner
control loop.

| Topic | Dir | Payload | Effect |
|---|---|---|---|
| `c2/radar/slew_to_cue` | in | `{"azimuth": float, "elevation": float, "target_id": str}` | `IDLE` → `SCAN` |
| `c2/operator/auth` | in | `{"auth": bool, "target_id": str, "token": str}` | `OPERATOR_AUTH` → `ENGAGE` |
| `c2/node/telemetry` | out | state, track, pointing error, latency | Situational awareness |
| `c2/node/event` | out | State transitions, engagements, faults | Audit trail |

Every inbound payload is schema-validated and range-checked before it can influence
state. The legacy code's comment — *"Only valid messages should be received, so no need
to make checks"* — sat directly above the handler that could switch on a laser. See
`guardrails.md` §5.

### 4.3 Hardware Interface Module (`helper/hardware/`)

| Component | Responsibility |
|---|---|
| `ActuatorDriver` (ABC) | Gimbal + effector contract |
| `SerialActuator` | ESP32-S3 over pyserial; owns the write lock and heartbeat |
| `MockActuator` | In-memory; records command history for tests |

`MockActuator` is not a testing nicety — it is the **second implementation that proves the
abstraction is real.** Two backends means an architecture; one means a claim.

---

## 5. Data & Control Flow — Engagement Lifecycle

```
[1] C2 CUE          MQTT c2/radar/slew_to_cue → validate → IDLE transitions to SCAN
     │
[2] SLEW            Cue az/el → gimbal angles → SerialActuator → ESP32 → servos
     │
[3] ACQUIRE         Grab-thread → latest frame → Detector → Detection[]
     │              No detection before timeout → back to IDLE
     ▼
[4] TRACK           Tracker assigns stable ID across frames
     │              AimpointSolver applies offset → pixel aimpoint
     │              Predictor leads by measured glass-to-photon latency
     │              Error → control law → incremental angle → ESP32
     ▼
[5] HOLD            Error inside ±15 px → start hold timer
     │              Error exits band OR track lost → timer resets, back to TRACK
     │              3.0 s continuous → firing solution is valid
     ▼
[6] OPERATOR_AUTH   Publish solution, await c2/operator/auth
     │              Gimbal keeps tracking throughout — the target does not wait
     │              Lock lost or auth timeout → back to TRACK or SCAN, never ENGAGE
     ▼
[7] ENGAGE          Arm → L1 → 2.0 s burn, tracking continues → L0 → disarm
     │              Log time-on-target, mean/peak error, total latency
     ▼
[8] IDLE            Laser confirmed LOW, servos safed, audit record written
```

**Invariant:** the laser is energised only inside step 7, only after step 6 returned an
affirmative authorisation bound to the same `target_id` tracked since step 4.

---

## 6. The State Machine

Single authoritative `EngagementState` enum. All transitions go through one guarded
method holding a `threading.RLock`. No component mutates state directly.

| State | Entry action | Valid exits | Fail-safe |
|---|---|---|---|
| `IDLE` | Laser LOW, disarm, servos to stow | → `SCAN` on validated C2 cue | Terminal safe state; all faults land here |
| `SCAN` | Slew to cued az/el, begin search | → `TRACK` on detection<br>→ `IDLE` on timeout (30 s) | Bounded sweep; never reverses into a bound (see §7) |
| `TRACK` | Engage control law on aimpoint | → `HOLD` on error in band<br>→ `SCAN` on track loss > 1.0 s<br>→ `IDLE` on op abort | Laser stays LOW throughout |
| `HOLD` | Start hold timer | → `OPERATOR_AUTH` at 3.0 s continuous<br>→ `TRACK` on error excursion | **Timer resets on any excursion — never accumulates across breaks** |
| `OPERATOR_AUTH` | Publish solution, await auth | → `ENGAGE` on valid auth<br>→ `TRACK` on lock loss<br>→ `IDLE` on deny/timeout (10 s) | Continues tracking while waiting; auth is bound to `target_id` |
| `ENGAGE` | Arm, laser HIGH, burn timer | → `IDLE` at 2.0 s or on any fault | **Hard ceiling. Any exception, lock loss, or serial fault cuts the beam immediately** |

**Transition rules:**
- Undeclared transitions raise `IllegalTransitionError` and force `IDLE`.
- Every transition is timestamped and published to `c2/node/event`.
- `ENGAGE` is reachable from exactly one predecessor. Non-negotiable.

---

## 7. Sweep Logic (legacy bug, corrected)

The legacy `scan_handle_x` called `turn_servo_x(+rate)` in **both** bounds-recovery
branches (`main_helper.py:51` and `:59`), so hitting the upper bound pushed the servo
further into it. Corrected logic:

1. Step in the current direction.
2. If the **next** step would exceed a bound, do not take it.
3. Flip direction, step the orthogonal axis one row, resume.
4. Maintain a sweep-cycle counter; N complete cycles without detection → `IDLE`.

Bounds are enforced **twice** — host-side before transmission, and again on the ESP32
before PWM write. Host-side alone is one bug away from a stalled servo.

---

## 8. Latency Budget

Targets for the demo configuration. **Replace every figure with a measurement before it
goes near a slide.**

| Stage | Budget | Notes |
|---|---|---|
| Camera exposure + USB transfer | ~33 ms | 640×480 MJPG @ 30 fps |
| Grab + decode | ~5 ms | Grab-thread; not in critical path |
| YOLO inference | 10–80 ms | GPU ~10–15 ms; laptop CPU 40–80 ms |
| Tracker update | 1–3 ms | ByteTrack |
| Predict + aimpoint + control | < 1 ms | |
| Serial TX + ESP32 parse | 2–5 ms | Native USB CDC |
| **Subtotal — glass to serial write** | **~50–125 ms** | **The figure the software owns** |
| SG90 mechanical response | 20–150 ms | Travel-dependent; no feedback |
| **Total — glass to photon on target** | **~70–275 ms** | **The figure that matters operationally** |

Quote both, and say which is which. The software number is the portable one; the
mechanical number is an artefact of a US$3 servo and is exactly what a real beam director
replaces. That framing turns your worst measurement into evidence for the thesis.

---

## 9. Known Architectural Limits

Stated here so they are never discovered by a judge first.

1. **Open-loop actuator.** No true position feedback. The loop closes optically only.
2. **Monocular — no range.** Bearing-only. Cannot compute time-to-target, true size, or
   slant range. Aimpoint offsets are angular, not metric.
3. **Aimpoint offset is geometric, not semantic.** It biases within a box. It does not
   identify a rotor hub. A trained keypoint model behind the same interface would — that
   is future work, and must be described as such.
4. **Visible-spectrum only.** No night, no degraded visibility, no hit-spot verification.
5. **Single node.** No multi-node deconfliction or fire distribution.
6. **MQTT unauthenticated at transport in the demo config.** Payload validation and auth
   tokens are application-layer. Production requires mTLS. See `guardrails.md` §5.
