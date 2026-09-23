# Killswitch

**A Software-Defined Directed Energy (SDDE) Counter-UAS node.**
Singapore Defence Tech Hackathon

> We are not building a cheaper laser. We are making the aiming intelligence
> portable — so the next thousand interceptor nodes cost what the hardware
> costs, not what the vendor charges.

---

## The Problem

A US$500 drone gets intercepted by a US$100,000 missile or a US$1.5M turret. That is an
exchange ratio no nation wins, and a salvo of fifty simply routes around the three nodes
you could afford to buy.

Directed energy fixes the economics — but today's systems ship as closed, vertically
integrated hardware. **The binding constraint is not beam power. It is cost-per-node and
vendor lock-in.**

Killswitch is the **targeting brain, not the turret**: an open, hardware-agnostic layer
that finds, tracks, predicts and holds a beam on a manoeuvring drone, running today on a
webcam, an ESP32 and a US$200 gimbal — and designed to re-host onto a military beam
director without a rewrite.

---

## Architecture

```
┌──────────────────────────────────────────┐
│   TARGETING BRAIN  (portable, the IP)    │
│   detect → track → predict → aim → gate  │
└──────────────────────────────────────────┘
        │              │              │
  FrameSource    ActuatorDriver   C2 Client
        │              │              │
  USB camera      ESP32 + SG90      MQTT
  (or MWIR)       (or beam dir.)   (radar cue + operator auth)
```

**Host (Python):** OpenCV capture with stale-frame rejection → YOLOv11 + ByteTrack →
aimpoint solver → velocity estimator → control law → serial.

**Edge actuator (ESP32-S3, C++):** command parser → checksum → bounds clamp → servo PWM →
effector gate, plus a deadman timer and burn ceiling **the host cannot override**.

### Engagement state machine

```
IDLE ──cue──▶ SCAN ──detect──▶ TRACK ──error≤15px──▶ HOLD
                                                      │ 3.0s continuous
                                                      ▼
  IDLE ◀──burn complete / fault── ENGAGE ◀──SPACE── OPERATOR_AUTH
```

`ENGAGE` has **exactly one predecessor**. There is no path to the effector that does not
pass through a human.

---

## Why This Architecture Wins

**1. The firmware is the safety authority, not the host.**
Python can stall on the GIL, pause for GC, be descheduled, or crash. None of that can
extend a burn, because none of the safety decisions live there. The ESP32 independently
enforces a 250 ms deadman (**measured 254 ms**) and a 2.0 s burn ceiling (**measured
2002 ms**). Pull the USB cable mid-burn and the effector dies in a quarter second without
the host participating.

**2. Asymmetric integrity on the wire.**
Commands that *increase* hazard carry an XOR checksum — `M1*7C` arm, `L1*7D` fire.
Commands that *decrease* hazard carry none and are accepted unconditionally. A corrupted
byte can never fire the effector; a corrupted byte can never block a shutdown. Combined
with two-key arming, energising requires **two independently checksum-valid commands in
order**.

**3. Hardware agnosticism, demonstrated rather than asserted.**
Every abstract interface has at least two live implementations, and **all 136 tests run
with no camera, no ESP32, no broker and no model weights.** Swapping the bench gimbal for
a beam director is a driver change.

**4. The human gate is structural, not cosmetic.**
`OPERATOR_AUTH` is a state in the transition table, not a callback someone can forget to
await. Authorisation binds to `target_id` and consumes a single-use nonce — auth for
track 7 cannot fire on track 9, and a replayed message is rejected.

**5. Everything is measured.**
`LatencyTracker` samples real glass-to-command intervals and feeds the lead time. The
closed-loop simulator proved the shipped gain (`Kp=0.6`) is stable and that `Kp≥0.8` goes
under-damped. Every number in the pitch traces to a log file.

---
## Target Effects & Thermal Kill Chain

The `Killswitch` software architecture enforces a strict 3.0-second continuous hold (`HOLD` state) before operator authorization is requested. This window is derived mathematically against standard commercial drone polycarbonates.

![Thermal Kill Chain](docs/thermal_kill_chain.png)

Based on a simulated 5 kW optical load targeting a 0.785 cm² rotor junction area, continuous track maintenance initiates material phase change (melting) at **~1.12 seconds**. Structural failure of the target frame is achieved well within the software's mandatory 3.0-second gate.

## Hardware

| Role | Part |
|---|---|
| Host | Laptop/PC (Linux demo box, macOS dev) |
| Actuator | ESP32-S3-WROOM-1 |
| Gimbal | 2× SG90 micro servo, pan/tilt |
| Camera | HBVCam-3M2111 V22 — **1280×720 MJPG @ 59.8 fps measured** |
| Effector | Proxy LED (eye-safe stand-in for a directed-energy effector) |

### Wiring

| Function | GPIO | Notes |
|---|---|---|
| Servo PAN | 5 | Separate 5 V rail |
| Servo TILT | 6 | Separate 5 V rail |
| Effector gate | 7 | 220 Ω → LED → GND, **10 kΩ pulldown to GND** |

**Two wiring rules that are not optional:**

**Servos get their own 5 V supply, with common ground to the ESP32.** Two SG90s stall-draw
~700 mA each. On a shared rail they brown out the board under load — i.e. during tracking,
i.e. on stage.

**The 10 kΩ pulldown on GPIO 7 is mandatory.** Between power-on and the first line of
`setup()`, every ESP32 GPIO is a floating input. A floating gate is an undefined effector
state through boot, reflash, brownout and crash. Software cannot fix this. The resistor can.

Use the **UART bridge** port, not native USB-CDC: the bridge stays enumerated across ESP32
resets, so your serial handle survives.

---

## Quickstart

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

**1. Bring up the camera** — measures real fps per mode and whether `CAP_PROP_BUFFERSIZE`
is honoured on your machine:

```bash
python -m tools.camera_probe
```

**2. Flash and verify the actuator.** Open `firmware/esp32_actuator/esp32_actuator.ino`
(board: *ESP32S3 Dev Module*, library: *ESP32Servo*), flash, then:

```bash
python -m tools.serial_probe --list
python -m tools.serial_probe --port /dev/ttyUSB0
```

This walks boot state → gimbal motion → bounds clamping → arming interlock → effector →
burn ceiling → deadman, printing pass/fail per check.

**3. Prove the control loop converges** — no hardware needed:

```bash
python -m tools.simulator
```

**4. Start the broker.** Platform-specific — `systemctl` does not exist on macOS:

```bash
brew install mosquitto && brew services start mosquitto
```

```bash
sudo apt install -y mosquitto && sudo systemctl start mosquitto
```

**5. Run the node.** Subsystems mock independently, because they fail
independently — a dev machine commonly has a camera and model but no ESP32 and no broker:

```bash
python main.py --config config/bench.yaml
```

| Flag | Use when |
|---|---|
| *(none)* | Everything attached |
| `--mock-actuator` | No ESP32 plugged in |
| `--mock-c2` | No MQTT broker running |
| `--mock-camera` | No camera, or permission not granted |
| `--mock-detector` | No weights present |
| `--mock` | All of the above — pure logic run |

Real camera and real model on a laptop with no hardware and no broker:

```bash
python main.py --config config/bench.yaml --mock-actuator --mock-c2
```

**6. Run the operator console** (separate terminal — this is the demo screen):

```bash
python -m tools.operator_console
```

`SPACE` authorise · `D` deny · `C` send a test cue · `Q` quit

**Tests:**

```bash
pytest tests/ -q        # 136 tests, zero hardware, ~0.1s
```

---

## Platform Notes (macOS)

**Camera permission.** macOS gates camera access per-application. If
`tools.camera_probe` prints `not authorized to capture video`, grant access to your
*terminal* app under **System Settings → Privacy & Security → Camera**, then **fully quit
and reopen the terminal** — the permission is read at process start, so a running shell
will not pick it up.

**Serial port naming.** macOS does not use `/dev/ttyUSB0`. With a CP2102 bridge the port
is `/dev/cu.usbserial-XXXX`; with a CH340 it is `/dev/cu.wchusbserial-XXXX`. Always
confirm before editing `config/bench.yaml`:

```bash
python -m tools.serial_probe --list
```

If nothing but `Bluetooth-Incoming-Port` and `debug-console` appears, the board is not
enumerating — check the cable carries data (not charge-only), that the board is powered,
and that the CP2102/CH340 driver is installed.

**`systemctl` does not exist on macOS.** Use `brew services` as above.

---

## Demo Runbook

1. Start `mosquitto`, then `main.py`, then `operator_console.py`.
2. Press `C` on the console to inject a radar cue — the node leaves `IDLE`.
3. Present the target. Watch `SCAN → TRACK → HOLD`, hold bar filling.
4. At 3.0 s continuous lock the banner turns amber: **AUTHORISATION REQUIRED**.
5. Press `SPACE`. The LED fires for 2.0 s, metrics are logged, node returns to `IDLE`.
6. **The fail-safe demo:** during a burn, pull the USB cable. The LED dies within 250 ms
   because the firmware, not the host, is holding it on.

Every state transition, firing solution and engagement is written to
`logs/<node>-<timestamp>.jsonl` as well as published over MQTT — so the audit trail
survives a broker outage.

---

## Documentation

| File | |
|---|---|
| [architecture.md](architecture.md) | Topology, concurrency model, state machine, latency budget, known limits |
| [guardrails.md](guardrails.md) | Binding safety constraints — read before touching the effector path |
| [CLAUDE.md](CLAUDE.md) | Conventions and context for AI-assisted development |

---

## Honest Limits

Stated here so nobody has to discover them.

- **Open-loop actuator.** SG90s have no position feedback. Reported angles are
  *commanded*, never measured. The loop closes optically, through the camera.
- **Monocular — no range.** Bearing-only. No time-to-target, no slant range.
- **Aimpoint offset is geometric, not semantic.** It biases within a detected box. It does
  not identify a rotor hub — a trained keypoint model behind the same interface would.
  Below `min_box_px` the solver reverts to centre-of-mass and says so in the audit log.
- **180° pan arc.** Cues outside ±90° of boresight are rejected, not serviced.
- **Visible spectrum only.** No night, no degraded visibility, no hit-spot verification.
- **The effector is an LED.** Every firmware guardrail is sized for a hazardous effector
  because the architecture claims this brain re-hosts onto one.
- **MQTT is unauthenticated at transport in the demo config.** Payload validation and auth
  tokens are application-layer. Production requires mTLS.
