# Killswitch — System Architecture (As-Built)

**Software-Defined Directed Energy (SDDE) Counter-UAS Node**
Singapore Defence Tech Hackathon · status: implemented, 119 tests passing, hardware-free

---

## 1. System Overview

Killswitch is a **targeting brain, not a turret.**

Fielded C-UAS directed-energy systems arrive as closed, vertically integrated hardware
north of US$1.5M per node. At that price coverage does not scale, and a saturating salvo
routes around the few nodes a nation can afford. The binding constraint is cost-per-node
and vendor lock-in — not beam power.

Every hardware-specific concern therefore sits behind a driver interface:

```
        ┌──────────────────────────────────────────┐
        │   TARGETING BRAIN  (portable, the IP)    │
        │   detect → track → predict → aim → gate  │
        └──────────────────────────────────────────┘
                  │              │              │
            FrameSource    ActuatorDriver   (effector via actuator)
                  │              │
        ┌─────────┴───┐   ┌──────┴─────┐
        │ UsbCamera   │   │ Serial     │   ← today, ~US$200 of COTS
        │ MockFrame   │   │ Mock       │   ← the second implementation
        │ MWIR imager │   │ beam dir.  │   ← operational, same brain
        └─────────────┘   └────────────┘
```

**The decoupling is the product.** The entire test suite — all 119 tests — runs against
the mock implementations with no camera, no ESP32, no broker and no model weights. That
is the hardware-agnosticism claim, demonstrated rather than asserted.

### Design principles, and where they are enforced

| Principle | Enforced by |
|---|---|
| Every ABC has ≥2 live implementations | `FrameSource`, `Detector`, `ActuatorDriver`, C2 client |
| Human authority is a state, not a callback | `EngagementState.OPERATOR_AUTH` in the transition table |
| Fail to safe, always | `_enter_idle()`, firmware deadman, `force_idle()` |
| Measured, not assumed | `LatencyTracker` — lead time self-tunes at runtime |

---

## 2. Topology

```
┌─────────────────────────────────────────────────────┐
│ HOST COMPUTE  (Linux demo box / macOS dev)          │
│                                                     │
│  HBVCam-3M2111 V22 ──USB──▶ UsbCameraSource         │
│       1280x720 MJPG @59.8fps  │ (frame-grabber)     │
│       measured ~32ms capture  ▼                     │
│                        UltralyticsDetector          │
│                        YOLOv11 + ByteTrack          │
│                               │                     │
│                        AimpointSolver               │
│                        AimpointPredictor            │
│                               ▼                     │
│   MQTT ◀── C2 cue / operator auth ── StateMachine   │
│                               │                     │
└───────────────────────────────┼─────────────────────┘
                                │ UART bridge, 921600
                     ASCII lines, 250 ms deadman
                                ▼
┌─────────────────────────────────────────────────────┐
│ EDGE ACTUATOR  (ESP32-S3-WROOM-1, C++)              │
│                                                     │
│  Parser → checksum → bounds clamp → LEDC → SG90×2   │
│         → arming interlock → effector gate → LED    │
│         → deadman (254 ms measured)                 │
│         → burn ceiling (2002 ms measured)           │
└─────────────────────────────────────────────────────┘
```

### Why the split exists

The serial boundary is a **deliberate rehearsal of the real interface.** A military beam
director is a separate subsystem behind a command link. By forcing aiming commands through
a constrained, latency-bearing channel today, the brain is proven to work without shared
memory to the effector.

It also places the safety-critical gate on a microcontroller that **cannot be blocked by
Python's GIL, a garbage-collection pause, or an OS scheduling decision.** This is the
single most important structural property of the system: the host proposes, the firmware
decides.

### Port selection

Use the **UART bridge** (CP2102/CH340), not native USB-CDC. Native USB re-enumerates on
every ESP32 reset, so the host's serial handle dies and pyserial raises. The bridge chip
is independently powered and stays enumerated. At 921600 baud the link carries ~92 KB/s
against a protocol that needs under 1 KB/s.

### Pin map (locked)

| Function | GPIO | Notes |
|---|---|---|
| Servo PAN | 5 | LEDC ch 0, 50 Hz, 500–2400 µs |
| Servo TILT | 6 | LEDC ch 1, 50 Hz |
| Effector gate | 7 | 220 Ω → LED → GND, **10 kΩ pulldown to GND** |

Avoid GPIO 0/3/45/46 (strapping), 19/20 (native USB), 26–32 (flash/PSRAM).

**Power:** SG90s run from a separate 5 V supply with common ground to the ESP32. Two
SG90s stall-draw ~700 mA each and will brown out a shared rail under load — i.e. during
tracking, i.e. on stage.

### Actuator characterisation

The SG90 is the dominant pole of the control loop:

- **No position feedback.** Commanded angle is the only angle known. The actuator runs
  **open-loop**; the loop closes optically, through the camera.
- ~100 ms per 60° travel plus startup deadband.
- ~1–2° of commanded resolution does nothing (pulse-width deadband).

`ActuatorStatus.pan`/`.tilt` are **commanded**, never measured. Never present them as
telemetry in a document a judge will read.

---

## 3. Serial Protocol

Newline-terminated ASCII, readable from a serial monitor during bring-up.

**Host → ESP32**

| Command | Meaning | Checksum |
|---|---|---|
| `A<pan>,<tilt>` | Absolute angles, degrees | no |
| `M1` / `M0` | Arm / disarm | **`M1` yes** |
| `L1` / `L0` | Energise / de-energise | **`L1` yes** |
| `P` / `S` / `Z` | Ping / status / e-stop | no |

**ESP32 → Host**

| Response | Meaning |
|---|---|
| `OK <echo>` | Accepted |
| `ST <pan>,<tilt>,<laser>,<armed>,<uptime_ms>` | Status @ 10 Hz |
| `ERR <code> <detail>` | Rejected, E01–E08 |

### Asymmetric integrity

Commands that **increase** hazard carry an XOR checksum: `M1*7C`, `L1*7D`. Commands that
**decrease** hazard carry none and are accepted unconditionally.

A corrupted byte can never fire the effector, because the checksum will not validate. A
corrupted byte can never block a shutdown, because shutdowns are not validated at all.
Combined with two-key arming, energising requires **two independently checksum-valid
commands in order**.

---

## 4. Concurrency Model

Six threads. Three guarded handoffs. **No shared mutable dict** — the legacy
`global_data` pattern, mutated from three threads with zero synchronisation, is gone.

```
main ───────────── tick @100 Hz ── THE ONLY WRITER OF STATE
  │                                        ▲
  │  reads SnapshotHolder ────(Lock)───────┤
  │  polls C2 inbox ──────────(Queue)──────┤
  │  writes actuator ─────────(Lock)───────┘
  │
vision-worker ──── detect → track → solve → publish snapshot
frame-grabber ──── inside UsbCameraSource; newest-frame-wins
mqtt-network ───── paho; validate → enqueue → return
serial-reader ──── inside SerialActuator; parses ST/OK/ERR
serial-heartbeat ─ inside SerialActuator; refreshes firmware deadman @10 Hz
```

### Thread contracts

| Thread | Owns | Must not |
|---|---|---|
| **main** | `EngagementState`, engagement bookkeeping | Block on I/O; `time.sleep` inside a handler |
| **vision-worker** | Detector, tracker, solver | Touch state directly or command the actuator |
| **frame-grabber** | `cv2.VideoCapture`, latest frame | Touch state or actuator |
| **mqtt-network** | Inbound validation | Do work — enqueue and return |
| **serial-reader** | Inbound frame parsing | Touch state |
| **serial-heartbeat** | Deadman refresh | Touch state |

### The three guarded handoffs

**`StateMachine`** — `threading.RLock`. Every transition passes through one guarded
method. Reentrant so an `on_transition` callback can read state without deadlocking.
No component assigns state directly.

**`SnapshotHolder`** — `threading.Lock`, single slot, overwrite-not-queue. The vision
worker publishes `TrackSnapshot`; the main thread reads the latest. A queued result is a
stale result, and stale results drive the gimbal to where the target used to be. Each
snapshot is immutable and carries `captured_at` / `processed_at`, so the state machine
reasons about **age explicitly** rather than assuming freshness.

**C2 inbox** — bounded `queue.Queue(64)`. Backpressure surfaces as dropped stale cues,
never as memory growth or a blocked MQTT client.

### Why vision runs on its own thread

Inference at 40–80 ms would stall the tick loop, and the tick loop enforces every
fail-safe: HOLD timer, auth timeout, burn ceiling, liveness. Decoupling lets the state
machine run at 100 Hz regardless of model speed. The cost is that the state machine sees
results up to one tick (10 ms) late, which is inside the latency budget and is measured
by `LatencyTracker` anyway.

---

## 5. The State Machine

```python
class EngagementState(str, Enum):
    IDLE = "IDLE"; SCAN = "SCAN"; TRACK = "TRACK"
    HOLD = "HOLD"; OPERATOR_AUTH = "OPERATOR_AUTH"; ENGAGE = "ENGAGE"
```

### Transition table

| State | → Legal | Trigger | Fail-safe |
|---|---|---|---|
| `IDLE` | `SCAN` | Validated `slew_to_cue`, azimuth inside gimbal arc | Effector confirmed LOW, disarmed, stowed, inbox drained, tracker reset |
| `SCAN` | `TRACK`, `IDLE` | Detection acquired / 30 s timeout / search volume covered | Bounded sweep; never steps into a bound |
| `TRACK` | `HOLD`, `SCAN`, `IDLE` | Error ≤ 15 px / target lost > 1.0 s | Effector LOW throughout |
| `HOLD` | `OPERATOR_AUTH`, `TRACK`, `SCAN`, `IDLE` | 3.0 s continuous in band / excursion / identity switch | **Timer resets on any excursion — never accumulates** |
| `OPERATOR_AUTH` | `ENGAGE`, `TRACK`, `SCAN`, `IDLE` | Valid auth / denial / 10 s timeout / lock loss | Tracking continues while waiting; auth bound to `target_id`; nonce single-use |
| `ENGAGE` | `IDLE` | 2.0 s burn / lock loss / any fault | **Beam cut immediately on lock loss — no coasting** |

### Two encoded invariants

1. **`ENGAGE` has exactly one predecessor**, `OPERATOR_AUTH`. Enforced by the table,
   asserted by `test_engage_has_exactly_one_predecessor`.
2. **`IDLE` is reachable from everywhere** — the universal safe harbour. Asserted by
   `test_idle_is_reachable_from_every_state`.

Undeclared transitions raise `IllegalTransitionError`, which the tick loop catches and
answers by forcing `IDLE`. `force_idle()` is the only bypass in the system, and it only
ever moves toward safety.

### Engagement lifecycle

```
[1] CUE        slew_to_cue → validate → cue_to_gimbal → IDLE→SCAN
                 (azimuth outside the ±90° arc is REJECTED, not clamped)
[2] SLEW       boustrophedon sweep, bounded by max_cycles
[3] ACQUIRE    YOLOv11 → ByteTrack → select_priority_target → SCAN→TRACK
[4] TRACK      AimpointSolver → AimpointPredictor leads by measured latency
                 → compute_correction → SerialActuator → ESP32
[5] HOLD       error ≤15 px continuously for 3.0 s → publish firing solution
[6] AUTH       await c2/operator/auth bound to target_id, nonce unused
[7] ENGAGE     arm → L1 → burn, tracking throughout → L0 → disarm → confirm
[8] IDLE       metrics logged, audit event published
```

---

## 6. Latency Budget (measured where possible)

| Stage | Figure | Source |
|---|---|---|
| Camera exposure + USB transfer | ~16.7 ms | measured, 59.8 fps @720p MJPG |
| Driver latency | ~15 ms | measured |
| **Capture subtotal** | **~32 ms** | **measured** |
| YOLOv11s ONNX inference @1024 | **216 ms** | **measured, macOS CPU** — re-measure on the demo box |
| Track + solve + control | < 3 ms | |
| Serial TX + ESP32 parse | ~3 ms | |
| **Glass → serial write** | **measured at runtime** | `LatencyTracker.compute_latency_s` |
| SG90 mechanical | ~60 ms allowance | **estimated — not measurable, no feedback** |
| **Glass → photon** | **`total_lead_s`** | compute (measured) + mechanical (estimated) |

`LatencyTracker` maintains an EWMA of real capture-to-command intervals and feeds it
straight into the predictor's lead time. Quote the compute figure as measured and the
mechanical figure as estimated — the distinction is the credibility.

### The observation rate is the binding constraint

`model/best.onnx` is a **fixed-shape YOLO11s export at 1024x1024** (`dynamic: False`), so
Ultralytics silently ignores any other `imgsz`. On macOS CPU it delivers **4.6 fps**.

The tick loop runs at 100 Hz so fail-safes stay responsive, but **the control law may only
step on a NEW observation.** Re-running it on a stale snapshot re-applies the same error
~20 times per frame, each correction stacking on the last, winding the gimbal far past the
target. `_drive_to_target` gates on `TrackSnapshot.frame_id` for exactly this reason.

**Measured target-rate envelope at 4.6 fps** (closed-loop simulation, `Kp=0.6`):

| Target behaviour | Steady-state error | HOLD latches? |
|---|---|---|
| Static | 1.2 px | yes |
| Steady crossing, 5–20 deg/s | 3–6 px | yes |
| Weave +/-15 deg @ 0.1 Hz | 6.8 px | yes |
| Weave +/-15 deg @ 0.2 Hz | 21.7 px | **no** |
| Weave +/-15 deg @ 0.4 Hz | 170 px | **no** |

At 50 fps the 0.4 Hz weave holds at 1.5 px. The limit is **sampling rate, not tuning** —
a predictor-smoothing sweep from 0.2 to 1.0 never brought the 0.4 Hz weave inside the
band. You cannot track what you cannot observe. Faster inference (GPU, or a 640 re-export)
is the only fix.

---

## 7. Repo Layout

```
main.py                      entry point, wiring, tick loop, state handlers
config/bench.yaml            every tunable
helper/
  vision/  types.py          Detection, AimPoint
           frame_source.py   FrameSource ABC, UsbCameraSource, MockFrameSource
           detector.py       Detector ABC, UltralyticsDetector, ScriptedDetector
           aimpoint.py       AimpointSolver, target selection, pointing error
           predictor.py      AimpointPredictor, LatencyTracker
  comms/   schemas.py        payload validation
           mqtt_client.py    C2Client, MockC2Client
  hardware/protocol.py       wire format, bounds, checksum, ActuatorStatus
           actuator.py       ActuatorDriver ABC, SerialActuator, MockActuator
  state/   machine.py        EngagementState, StateMachine, SnapshotHolder
           sweep.py          SweepController, cue_to_gimbal
           control.py        compute_correction — shared by node and simulator
firmware/esp32_actuator/     C++ — the safety authority
tools/                       camera_probe, serial_probe, operator_console, simulator
tests/                       hardware-free
```

---

## 8. Known Architectural Limits

Stated here so a judge never discovers them first.

1. **Open-loop actuator.** No true position feedback. The loop closes optically only.
2. **Monocular — no range.** Bearing-only. No time-to-target, no slant range, no true
   target size. Aimpoint offsets are angular, not metric.
3. **Aimpoint offset is geometric, not semantic.** It biases within a box. It does not
   identify a rotor hub. A trained keypoint model behind the same interface would.
4. **180° pan arc.** Cues outside ±90° of boresight are rejected, not serviced.
5. **Visible spectrum only.** No night, no degraded visibility, no hit-spot verification.
6. **Single node.** No multi-node deconfliction or fire distribution.
7. **MQTT unauthenticated at transport in the demo config.** Payload validation and auth
   tokens are application-layer. Production requires mTLS.
