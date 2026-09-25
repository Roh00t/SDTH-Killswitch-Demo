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

| Role | Part | Bus / pins |
|---|---|---|
| Host | Laptop/PC — Windows demo box, macOS dev | — |
| Edge actuator | ESP32-S3-N16R8 (16 MB flash, 8 MB PSRAM) | CH343 USB-UART @921600, `COM3` |
| Gimbal | 2× SG90, separate 5 V rail | pan GPIO 5 / tilt GPIO 6 |
| Effector | KY-008 650 nm laser module, low-side switched | GPIO 7 → 1 kΩ → 2N2222 base, 10 kΩ pulldown |
| Camera | USB UVC webcam, opened by the HOST via OpenCV | host USB, **not** the ESP32 |

**The camera is on the host, deliberately.** An OV5640 wired to the ESP32-S3 is
architecturally excluded, not merely unimplemented — see *Why the camera is not on the
ESP32* below. `helper/vision/frame_source.py` opens a UVC device by index; it has no
path that reaches a DVP/MIPI sensor.

**Use the CH343 UART port, never the native USB-CDC port.** The S3 exposes both. The
bridge stays enumerated across ESP32 resets; native USB re-enumerates and takes the
host's serial handle with it. `_BRIDGE_HINTS` in `helper/hardware/actuator.py` matches
the CH343 family and deliberately does *not* match `USB Serial Device`.

### Why the camera is not on the ESP32

Three independent blockers, any one of which is disqualifying:

1. **GPIO collision.** ESP32-S3 camera wiring occupies most of GPIO 4–18 for the DVP
   data bus. GPIO 5, 6 and 7 — pan, tilt and the effector gate — are inside that range
   on every common S3 camera pinout. The actuator pins would have to move, and the
   effector pin is the one carrying the 10 kΩ pulldown.
2. **The link cannot carry the pixels.** The UART bridge runs at 921600 baud ≈ 92 KB/s.
   One 1280×720 MJPEG frame is 50–100 KB. That is roughly 1 fps, against a control loop
   designed around ~5 fps of inference. Streaming over WiFi instead adds 100–200 ms to
   the glass-to-photon budget that `LatencyTracker` exists to minimise.
3. **It puts vision on the safety processor.** The ESP32 is the safety authority: it
   owns the 250 ms deadman, the 2000 ms burn ceiling and the 50 Hz servo update. Adding
   camera DMA and a WiFi stack to that core competes with exactly those deadlines.

Onboard inference is not a third option: YOLO11s does not run on an S3.

### HITL verification status — actuator path

**Verified on the Windows rig:**

- CH343 bridge enumerates as `COM3`; 921600 baud link stable.
- `ActuatorStatus` frames arrive clean — no drops, no checksum failures.
- Gimbal tracks absolute angle commands via `serial_probe --interactive` (`a <pan> <tilt>`).
- Centre (90, 90) and tilt to 48 deg with no binding, over-travel or brownout.

**NOT yet verified. The stack is not integration-ready until these pass:**

- **The travel corners.** `--servo-sweep` commands pan 5/175 and tilt 48/132, which sit
  5 deg and 3 deg *inside* the software bounds of pan 0/180 and tilt 45/135. But
  `SweepController.step()` and `cue_to_gimbal()` both command the true bounds, so SCAN
  will drive the gimbal past anything that has been physically tested. The labels in
  `servo_sweep()` read "(0 deg)" and "(45 deg)" while commanding 5 and 48 — believe the
  numbers, not the labels.
- **Every effector interlock**: arm/disarm, fire-while-disarmed refusal, e-stop, the
  2000 ms burn ceiling, the 250 ms deadman. Interactive positioning exercises none of them.
- The vision pipeline, and the C2 plane end to end on the real box.

**One command closes the first two gaps:**

```bash
python -m tools.serial_probe --port COM3
```

`run_checks()` issues `set_angles(999, -999)`, so the firmware clamps to pan 180 /
tilt 45 — the real corners — and then runs 13 interlock checks.

> **That command fires the effector.** It is now a KY-008 laser, not the LED its console
> prints still describe: twice for ~0.5 s, then a full 2.6 s burn-ceiling test, then a
> deadman test. Point it at a matte backstop before running it.

### Pan/tilt bounds are not configurable — on purpose

No YAML file defines them, and none should. The limits live in
`helper/hardware/protocol.py` (`PAN_MIN_DEG` 0, `PAN_MAX_DEG` 180, `TILT_MIN_DEG` 45,
`TILT_MAX_DEG` 135) and again in `esp32_actuator.ino`, which is architectural rule 9:
bounds enforced twice, host-side before transmit and firmware before the PWM write. A
config knob would be a third authority that the firmware does not honour. What the config
*does* hold is `scan.pan_step_deg` / `tilt_step_deg` (sweep granularity),
`scan.boresight_azimuth_deg` (cue geometry) and `actuator.stow_pan_deg` / `stow_tilt_deg`
(the safe-harbour pose) — none of which are limits.

Weights train on **Google Colab**, land as `.pt`/`.onnx`, and are **gitignored**.

---

## Tech Stack

Python 3.9+ for the node, **3.11+ for `tools/c2_bridge.py`** (`asyncio.TaskGroup` and
PEP 654 `except*`). Validated on 3.14 on Windows, where the bridge forces a selector
event loop — ProactorEventLoop has no `add_reader`, which aiomqtt requires.
OpenCV · Ultralytics YOLOv11 + ByteTrack · onnxruntime · `paho-mqtt<2.0`
(2.x changed callback signatures — do not unpin) · pyserial · PyYAML · pytest ·
C++/Arduino (`ESP32Servo`, LEDC).

**`lap>=0.5.12` is pinned deliberately.** Ultralytics does not declare it, and
auto-installs it on the first `model.track()` call — which fails on an offline demo box,
at runtime, mid-engagement.

**Model:** `model/best.onnx`, YOLO11s, classes `{0: drone, 1: bird, 2: airplane,
3: helicopter}`. Filtering is by NAME, so index drift cannot silently re-target the
system; an unknown name is rejected at construction.

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

11. **The control law steps only on a NEW observation.** `_drive_to_target` gates on
    `TrackSnapshot.frame_id`. The tick runs at 100 Hz; the model delivers ~4.6 fps.
    Without the gate the same stale error is re-applied ~20x per frame and the gimbal
    winds up. A feedback loop may only step when its feedback is new.

12. **`imgsz` in config must match the export.** `model/best.onnx` is fixed-shape at
    1024x1024 (`dynamic: False`). Ultralytics **silently overrides** any other value, so
    a wrong `imgsz` is a no-op that looks like a tuning knob. The detector reads the
    native size from ONNX metadata and warns loudly on mismatch.

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
