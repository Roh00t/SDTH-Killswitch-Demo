# CLAUDE.md

Guidance for Claude Code working in this repository.

> Read `architecture.md` and `guardrails.md` before structural changes.
> `guardrails.md` is safety-critical and overrides convenience in every case.

---

## Project Identity

**Killswitch** — a Software-Defined Directed Energy (SDDE) Counter-UAS node for the
Singapore Defence Tech Hackathon. It is the **targeting brain, not the turret**: an open,
hardware-agnostic layer that finds, tracks, predicts and holds a beam on a manoeuvring
drone, running on COTS hardware and gated by a human authorisation state.

Refactored from a legacy Raspberry Pi bird deterrent (`upstream` remote). Legacy code is
being deleted, not extended. `picamera2`, `gpiozero`, `pigpio`, audio triggering and the
`YoloV5_*` wrappers are all legacy — not patterns to follow.

**This system commands a physical effector.** Changes to the effector path, the state
machine, or MQTT input handling are safety-critical.

---

## Hardware (as built)

| Role | Part |
|---|---|
| Host | Laptop/PC — Linux demo box, macOS dev |
| Edge actuator | ESP32-S3-WROOM-1, UART bridge @921600 |
| Gimbal | 2× SG90, pan GPIO 5 / tilt GPIO 6, separate 5 V rail |
| Camera | HBVCam-3M2111 V22 — 1280×720 MJPG @59.8 fps measured |
| Effector | Proxy LED, GPIO 7, 10 kΩ pulldown |

Weights train on **Google Colab**, land as `.pt`/`.onnx`, and are **gitignored**.

---

## Tech Stack

Python 3.9+ · OpenCV · Ultralytics YOLOv11 + ByteTrack · `paho-mqtt<2.0` (2.x changed
callback signatures — do not unpin) · pyserial · PyYAML · pytest · C++/Arduino
(`ESP32Servo`, LEDC).

---

## Commands

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

python main.py --config config/bench.yaml           # full node
python main.py --config config/bench.yaml --mock    # no hardware at all
python -m tools.camera_probe                        # modes, real fps, BUFFERSIZE
python -m tools.serial_probe --port <dev>           # every firmware interlock
python -m tools.operator_console                    # C2 dashboard, SPACE to authorise
python -m tools.simulator                           # closed-loop convergence proof
pytest tests/ -q                                    # 119 tests, zero hardware
```

Run everything **from the repo root**.

---

## Architectural Rules

Not style preferences — each one has a real bug behind it.

1. **No global mutable state.** The legacy `global_data` dict was mutated from three
   threads unsynchronised. State lives in owning classes; anything crossing a thread
   boundary is lock-guarded.

2. **One writer of state.** `EngagementState` changes only via
   `StateMachine.transition_to()` holding an `RLock`. Undeclared transitions raise.
   `force_idle()` is the sole bypass and only moves toward safety.

3. **Three guarded handoffs, no more.** `StateMachine` (RLock), `SnapshotHolder` (Lock),
   C2 inbox (bounded Queue). Do not add a fourth without a documented reason.

4. **MQTT callbacks validate and enqueue. They never act.** Paho dispatches on its own
   thread; a blocked callback stalls the client.

5. **Every ABC has ≥2 live implementations.** One real, one mock. The whole test suite
   runs on mocks — that is the hardware-agnosticism claim, demonstrated.

6. **Mocks mirror firmware exactly.** `MockActuator` once raised on arm-after-e-stop
   while the firmware's checksummed `M1` cleared the latch. The mock was validating
   fiction. If firmware behaviour changes, change the mock in the same commit.

7. **Zero-latency capture is a grab thread, not a property.** `CAP_PROP_BUFFERSIZE = 1`
   is honoured by V4L2/DSHOW and **ignored by AVFoundation on macOS**. It is set as a
   hint; the load-bearing mechanism is the newest-frame-wins reader thread.

8. **The effector fails safe.** LOW is the default in every state, every error path,
   every exception handler, and on the firmware's 250 ms deadman.

9. **Bounds enforced twice** — host-side before transmit, firmware before PWM write.

10. **Control law lives in one place.** `helper/state/control.py::compute_correction` is
    shared by the node and the simulator. If they diverge, the simulator proves nothing.

---

## Coding Standards

- **Type hints on every signature**, including returns. `Detection`, `AimPoint`,
  `EngagementState`, `TrackSnapshot` are the shared vocabulary — never raw tuples.
- **Strict Enums for closed sets**, `@dataclass(frozen=True)` for data. No dict-as-struct.
- **No silent broad excepts.** `except Exception: print(); continue` masked four real
  defects in the legacy code. Catch narrow, log with context, and either handle
  meaningfully or transition to IDLE. The single permitted broad catch is the top-level
  tick guard, which **de-energises first, then logs, then forces IDLE**.
- **Docstrings** on every public class and method: purpose, args, returns, raises, and
  **which thread is expected to call it**.
- **`time.monotonic()` for all durations.** `time.time()` steps with NTP; a clock jump
  must not extend or truncate a burn.
- **Tunables live in `config/bench.yaml`.** No magic numbers inline.

---

## Context for Future Sessions

Two design decisions that look odd without their history:

**`AimPoint` carries provenance, not just coordinates.** Every solution records
`downgraded`, `offset_applied`, `reason` and `track_id`. The resolution gate silently
reverts weak-point biasing to centre-of-mass when the box is smaller than
`min_box_px` — because an offset smaller than box jitter aims at noise. Without
provenance the audit log could not distinguish "we aimed at the rotor hub" from "we
wanted to and couldn't." Preserve these fields; the firing-solution event publishes them.
`track_id` also binds authorisation: auth for track 7 must never fire on track 9.

**The velocity estimator is explicit, not ByteTrack's.** ByteTrack maintains a Kalman
filter per track, but reaching it means `model.predictor.trackers[0].tracked_stracks[i].mean`
— private API whose shape shifts between ultralytics releases. `AimpointPredictor` is an
EWMA-smoothed finite-difference estimator on the **aimpoint** (not the box centre, because
the aimpoint is what we drive to). It is version-independent and unit-tested. Do not
"simplify" it back to the tracker's internals.

**`LatencyTracker` self-tunes the lead time.** It samples every processed snapshot, not
only those that produce a command — a target sitting inside the deadband would otherwise
never update the estimate. `compute_latency_s` is measured; `mechanical_allowance_s` is
an estimate because SG90s have no position feedback. Keep that distinction in any output
a human reads.

---

## Testing

119 tests, all hardware-free, ~0.1 s.

| File | Covers |
|---|---|
| `test_aimpoint.py` | Offset math, clamping, resolution gate, target selection |
| `test_comms.py` | Payload validation, hostile inputs, token handling |
| `test_actuator.py` | Framing, checksums, bounds, arming interlock, e-stop |
| `test_state_machine.py` | Transition table, sweep bounds, cue geometry, prediction |
| `test_closed_loop.py` | Control-loop convergence against a simulated gimbal |

**Unit tests are necessary but not sufficient.** Five real bugs were found only by running
the whole node in mock mode — duration logging, mock free-running, wrong teardown verb,
nominal completion logged as FORCED, and latency never sampling. Run
`python main.py --mock` after touching the tick loop.
