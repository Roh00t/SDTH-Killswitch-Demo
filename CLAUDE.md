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

### HITL verification status — actuator path COMPLETE

`python -m tools.serial_probe --port COM3` — **13/13 PASS** on the Windows rig.

**Verified:**

| Area | Covered |
|---|---|
| Link | CH343 bridge on `COM3`, 921600 baud, `ActuatorStatus` frames clean — no drops, no checksum failures |
| Boot state | Effector de-energised and disarmed at boot (e-stop latched) |
| Bounds | Out-of-range commands clamped, not wrapped: pan → 180, tilt → 45 |
| Arming interlock | Fire refused while disarmed; `M1` accepted; effector energises only once armed |
| De-energise | Effector off on command, confirmed by status frame |
| Burn ceiling | Firmware cut the burn at `MAX_BURN_MS` without host involvement |
| Deadman | Heartbeat suppressed 0.6 s → firmware cut the effector **and** dropped arming |
| Positioning | Absolute angle commands via `serial_probe --interactive` (`a <pan> <tilt>`); centre (90, 90) and tilt 48 deg with no binding or brownout |

**One caveat to preserve, because the repo's own discipline demands it.** The two bounds
checks read `pan`/`tilt` out of the status frame, and those are *commanded* angles — SG90s
have no position feedback. `run_checks()` prints this itself: the gimbal section "passes
even with no servos attached." So 13/13 proves the **firmware clamps correctly**; it does
not, on its own, prove the servos physically reached pan 180 / tilt 45 without binding.
That is confirmed by eye during the run, or not at all. Never quote the clamp result as
measured travel.

**Remaining before the stack is integration-ready:**

- The vision pipeline on the rig — YOLO lock against a real target through the host webcam.
- The C2 plane end to end: bridge, dashboard, and one clean authorisation through
  `SPACE` → `ENGAGE` → `ENGAGEMENT COMPLETE`.

> `serial_probe --port COM3` **fires the effector** — now a KY-008 laser, not the LED its
> console prints still describe. Two ~0.5 s pulses, a 2.6 s burn-ceiling test, then a
> deadman test. Matte backstop, area behind it clear, every time it is run.

### `--sim-target` never touches real hardware

`main.py --sim-target` replaces the camera and detector with
`tools/simulator.py::SceneDetector` and **forces** the mock actuator; no flag combination
lets a synthetic target reach `SerialActuator`, and `SceneDetector` raises `TypeError` if
it is handed one. The reason is the release chain: a synthetic target satisfies "visual
lock" with nothing real in the beam path. Keep that invariant if you extend the sim.
MQTT stays real so the bridge, console and dashboard run against it unchanged.

### The TAK map is output-only and never invents state

The bridge sends CoT; nothing listens for it. `parse_cot` is hardened but unwired, so a
marker dropped in ATAK does not cue the gimbal. Wiring it would make the LAN an
unauthenticated cue source for an effector-carrying gimbal, which is safety-critical work.

Node 1's marker mirrors the real node's reported state (`KILLSWITCH-01 [ENGAGE]`) and
reads `NO LINK` before the first report, on the MQTT last-will, or after
`NODE_LINK_TIMEOUT_S` of silence. Every other marker says `SIMULATED` in its remarks.
Never add a path that sets engagement state on the map by hand; a status a keypress can
fake is a status a judge cannot trust.

### The dashboard WebSocket is output-only too

`--ws-host 0.0.0.0` puts the dashboard feed on the LAN for a phone. `serve_client` reads
and discards everything a client sends. It used to log `authorise`/`abort` under a comment
claiming the veto went out over MQTT, which it never did. Keep it receive-nothing: a
control input here would be unauthenticated. `broadcast` gives each client a 0.5 s send
deadline and drops laggards, because `websockets` blocks `send()` once ~2 MB queue behind a
client that has stopped reading. Without the deadline, one locked phone freezes every
screen.

The dashboard obeys the map's `NO LINK` rule too: `FleetState.node1_view()` sends
`NO LINK` and an empty telemetry object, so Panel C reads `NO NODE TELEMETRY` rather than
a dead node's frozen numbers.

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
python main.py --config config/fallback.yaml --sim-target  # hardware-free demo, real MQTT
pytest tests/ -q                                    # 243 tests, zero hardware
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

243 tests, all hardware-free, ~2 s.

| File | Covers |
|---|---|
| `test_aimpoint.py` | Offset math, clamping, resolution gate, target selection |
| `test_comms.py` | Payload validation, hostile inputs, token handling |
| `test_actuator.py` | Framing, checksums, bounds, arming interlock, e-stop |
| `test_state_machine.py` | Transition table, sweep bounds, cue geometry, prediction, auth-window telemetry |
| `test_closed_loop.py` | Control-loop convergence against a simulated gimbal |
| `test_cot.py` | CoT wire format, hostile input, geodesy, bridge priority, unicast, stale pad, honest map labels, last-will, dashboard socket, dashboard NO LINK |
| `test_sim_scene.py` | `--sim-target` scene: refuses a real actuator, closes the loop, fresh ids on reset |

**Unit tests are necessary but not sufficient.** Five real bugs were found only by running
the whole node in mock mode — duration logging, mock free-running, wrong teardown verb,
nominal completion logged as FORCED, and latency never sampling. Run
`python main.py --mock` after touching the tick loop.
