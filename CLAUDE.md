# CLAUDE.md

Guidance for Claude Code working in this repository.

> **Read `architecture.md` and `guardrails.md` before making structural changes.**
> `guardrails.md` is safety-critical and overrides convenience in every case.

---

## Project Identity

**Killswitch** — a Software-Defined Directed Energy (SDDE) Counter-UAS node for the
Singapore Defence Tech Hackathon. It is the **targeting brain, not the turret**: an
open, hardware-agnostic software layer that finds, tracks, predicts and holds a beam on a
manoeuvring drone, running on COTS hardware.

The repo is an in-progress refactor of a legacy Raspberry Pi bird-deterrent
(`dragonstonehafiz/inf2009-project`, see the `upstream` remote). Legacy code is being
replaced, not extended. If you find `picamera2`, `gpiozero`, `pigpio`, microphone/audio
triggering, or `helper/sound.py` — that is legacy and slated for deletion, not a pattern
to follow.

**This system points a laser.** Changes touching the effector path, the state machine, or
MQTT input handling are safety-critical.

---

## Hardware (actual, as built)

| Role | Part |
|---|---|
| Host compute | Laptop/PC (macOS dev, Python 3.9+) |
| Edge actuator | ESP32-S3-WROOM-1 (C++/Arduino) |
| Gimbal | 2× SG90 micro servo, pan + tilt |
| Camera | HBVCam-3M2111 V22 (USB UVC) |
| Effector | Low-power proxy laser via MOSFET |

Model training runs on **Google Colab**; weights land here as `.pt` / `.onnx` and are
**gitignored**. Never commit weights.

---

## Tech Stack

- **Python 3.9+** — host pipeline
- **OpenCV** — capture and frame ops
- **Ultralytics YOLO** (`.pt`, GPU/training) and **ONNX Runtime** (`.onnx`, portable CPU)
- **ByteTrack** — multi-object tracking, identity persistence
- **paho-mqtt < 2.0** — C2 plane (API differs in 2.x; pin it)
- **pyserial** — host ↔ ESP32
- **C++/Arduino** — ESP32-S3 firmware (`ESP32Servo`, LEDC PWM)

---

## Commands

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

python main.py --config config/bench.yaml          # full node
python main.py --config config/bench.yaml --mock   # no hardware, mock actuator
python -m tools.camera_probe                       # enumerate modes, measure fps
python -m tools.serial_probe                       # ESP32 link check
pytest tests/ -v                                   # all tests, hardware-free
pytest tests/test_state_machine.py -v              # state machine only
```

All commands run **from the repo root**. Package dirs have `__init__.py`; the legacy
`helper/` did not, which is why legacy scripts only worked from root by accident.

---

## Architectural Rules

These are not style preferences. Violating them has caused real bugs in this repo.

1. **No global mutable state.** The legacy `global_data` dict was mutated from the main
   loop, a model thread, and MQTT callbacks with zero synchronisation. Gone. State lives
   in owning classes; anything crossing a thread boundary is guarded by a lock.

2. **State transitions go through one guarded method.** `EngagementState` is a `StrEnum`.
   All changes route through the state machine's transition method holding a
   `threading.RLock`. No component assigns state directly. Undeclared transitions raise.

3. **Strict separation of concerns.** Vision does not touch serial. Comms does not touch
   the actuator. The state machine orchestrates; it does not implement.

4. **Every ABC has ≥2 live implementations.** One real, one mock. An abstraction with a
   single implementation is an unverified claim — and hardware agnosticism *is* the pitch.

5. **Zero-latency capture is a grab-thread, not a property.** Set
   `cv2.CAP_PROP_BUFFERSIZE = 1` as a hint, but never rely on it: **it is ignored by
   AVFoundation on macOS** (V4L2/DSHOW only). The load-bearing mechanism is a daemon
   thread spinning `cap.grab()` with `retrieve()` on demand. Also request MJPG via
   `CAP_PROP_FOURCC` — UVC defaults to YUYV and collapses to ~5 fps.

6. **The effector fails safe.** Laser LOW is the default in every state, every error path,
   every exception handler, and on the ESP32's 250 ms deadman timeout. See
   `guardrails.md` §2.

7. **Bounds are enforced twice.** Host-side before transmit, ESP32-side before PWM write.

8. **MQTT input is hostile until validated.** Schema, type, and range check every payload
   before it can influence state.

---

## Coding Standards

- **Type hints on every signature**, including returns. `Detection`, `AimPoint`,
  `EngagementState` are the shared vocabulary — use them, don't pass raw tuples.
- **No silent broad excepts.** `except Exception: print(...)` and continue is banned —
  it masked four real defects in the legacy code. Catch narrow, log with context, and
  either handle meaningfully or transition to `IDLE`. If you truly must catch broad,
  log the traceback and transition to a safe state.
- **Docstrings on every public class and method**: purpose, args, returns, raises, and
  thread-safety. Say explicitly which thread a method is expected to be called from.
- **`dataclass` for data, `Enum` for closed sets.** No dict-as-struct.
- **Constants at module top or in config.** No magic numbers inline. Tunables
  (thresholds, timeouts, offsets, pins) belong in YAML config, not in code.
- **Log state transitions and every effector command.** The audit trail is a deliverable.

---

## Platform Gotchas

- `CAP_PROP_BUFFERSIZE` is a no-op on macOS — see rule 5.
- ESP32-S3-WROOM-1 has **both** native USB-CDC and a UART bridge. Prefer native USB
  (baud is nominal). Know which port you are cabled into before debugging latency.
- GPIO 0, 3, 45, 46 are strapping pins; 19/20 are native USB; 26–32 are flash/PSRAM.
  Do not assign peripherals there.
- The laser GPIO needs a **10 kΩ external pulldown**. GPIOs float during boot and reflash.
- SG90s have **no position feedback**. Reported angle is *commanded*, never measured.
  Never describe it as telemetry in a document a judge will read.
- `paho-mqtt` 2.x changed the callback signature. Pin `<2.0`.

---

## Repo Layout (target)

```
main.py                    # entry point, wiring only
config/                    # YAML — thresholds, pins, topics, offsets
helper/
  vision/                  # FrameSource, Detector, Tracker, AimpointSolver, Predictor
  comms/                   # MQTT client, payload schemas + validation
  hardware/                # ActuatorDriver ABC, SerialActuator, MockActuator
  state/                   # EngagementState enum, guarded StateMachine
firmware/esp32_actuator/   # C++ — parser, bounds clamp, deadman, laser gate
tests/                     # hardware-free; mocks for camera, serial, MQTT
tools/                     # camera_probe, serial_probe, latency_bench
```

---

## Testing

The legacy repo had **zero automated tests**; several shipped defects would each have
been caught by one. Non-negotiable coverage:

- State machine transitions, including every illegal transition and fail-safe.
- `HOLD` timer reset on error excursion (must not accumulate across breaks).
- Aimpoint offset math, including clamping and the resolution-gate downgrade.
- Sweep bounds — specifically that hitting a bound reverses instead of pushing into it.
- MQTT payload validation, including malformed, out-of-range, and hostile inputs.
- Serial framing and the deadman timeout.

All tests run **without hardware attached** via `MockActuator` and synthetic frames.

---

## Legacy Status

Being removed: `helper/PiCameraInterface.py`, `helper/sound.py`, `helper/RaspberryPiZero2.py`,
`helper/Arduino.py`, `calibrate.py`, `sound/`, `model/bird_sound_model.onnx`,
`model/yolov5n_*.onnx`.

Known legacy defects — do not reintroduce these patterns:
- Init race: worker thread started before the board it calls was assigned.
- `UnboundLocalError` from a variable assigned only inside conditional branches.
- Sweep reversal pushing the servo further into its bound.
- `conf_thres` accepted as a parameter then ignored in favour of a hardcoded value.
- Centre/corner box-format mismatch fed to NMS.
