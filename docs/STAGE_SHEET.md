# Stage Sheet: physical node

Windows laptop → CH343 `COM3` → ESP32-S3 → two SG90s and a KY-008 650 nm laser (a
low-power proxy for a high-energy laser). The camera is a USB webcam on the laptop.

**Roles**

| Role | Job | Touches |
|---|---|---|
| OPERATOR | Owns the **Operator Console** (OpenCV) window. Presses SPACE or D, nothing else | Keyboard only |
| HANDLER | Places the target, then stands **behind the gimbal** | Target stand |
| PRESENTER | Talks | Nothing |

## Safety rules: read aloud at T-60

- [ ] The laser can only be on in **ENGAGE**, after SPACE. The host burns 1.8 s, the
  firmware cuts at 2.0 s, and the firmware's deadman cuts 250 ms after the laptop goes
  silent.
- [ ] A matte backstop sits behind the target. The gimbal's pan arc (0–180°) faces the
  backstop, **never the audience**.
- [ ] The target moves **only while the STATE pill reads SCAN or TRACK**, pushed by its
  stand's base from the side.
- [ ] **Hands and faces out of the target area from the moment the pill turns amber**
  (OPERATOR_AUTH). SPACE can come at any second after that.
- [ ] Emergency stop: **pull the ESP32 USB cable**. The firmware cuts the laser within
  250 ms.

## T-60: hardware and host

| ☐ | Step | Command or action | Pass when |
|---|---|---|---|
| ☐ | Code is current | `git pull`, then `git log --oneline -1` | Shows the commit that added this sheet, or later |
| ☐ | Laptop never sleeps | On AC power. `powercfg /change standby-timeout-ac 0`. Power Options → Advanced → USB settings → USB selective suspend → **Disabled** | Suspend would drop the CH343 mid-demo, and the node falls back to IDLE with "serial link lost" |
| ☐ | Power order | 1. ESP32 USB into the **labelled** port. 2. Then the servo 5 V rail | Servos go to centre (90/90); laser off |
| ☐ | Serial port | `python -m tools.serial_probe --list` | A CH343 (`USB-Enhanced-SERIAL CH343`) on **COM3**. On any other COM, set `actuator.port` in `config/fallback.yaml` |
| ☐ | Mark the target spot | `python -m tools.serial_probe --port COM3 --interactive`, then `a 45 97`, tape where the camera points, then `a 90 90`, then **`q`** | Tape on the backstop. **Quit** before step B: COM3 is exclusive. Never run `serial_probe` without `--interactive` here, because the full probe **fires the laser** |
| ☐ | The target is detected | Target on its stand at the tape. `python -m tools.vision_probe --config config/fallback.yaml`, then **Q** to quit (the camera is exclusive) | A box labelled `bird`, `airplane`, `kite` or `frisbee` at **≥ 0.40**. COCO has no drone class, so a drone prop usually fails this |
| ☐ | Works offline | `dir yolo11s.pt` in the repo folder, then run the whole sheet once **with Wi-Fi off** | The file exists. It only auto-downloads when online |
| ☐ | Broker is local only | `Get-Service mosquitto`, then `netstat -an \| findstr :1883` | Running, listening on `127.0.0.1:1883` only. If it shows `0.0.0.0:1883`, anyone on the hotspot can send radar cues (cues carry no token): set `listener 1883 127.0.0.1` in `mosquitto.conf` and restart the service |

## T-15: tuning, only if the gimbal misbehaves

Edit `control:` in `config/fallback.yaml`. Change **one knob at a time**, restart window
B, and re-test. There is no PID to add: the controller is P plus velocity feed-forward,
in `helper/state/control.py`.

| Symptom (target still) | Knob | From → to | Never |
|---|---|---|---|
| Buzz or twitch at rest, error under 5 px | `deadband_px` | 3 → 5 (max 8) | Near the 15 px HOLD band |
| Overshoots and hunts around the target | `proportional_gain` | 0.6 → 0.45 | Above 1.0: SG90s go under-damped |
| Sluggish; HOLD never latches on a moving target | First check `camera.horizontal_fov_deg`, then `proportional_gain` | 0.6 → 0.75 | Raising P "for jitter": it makes jitter worse |
| Big jump on acquisition | `max_step_deg` | 6 → 4 | — |

## T-5: start order

One PowerShell window per command, all in the repo folder. If PowerShell shows `>>`,
two lines went into one window: press Ctrl+C and split them.

| Window | Command | Healthy when |
|---|---|---|
| 0 | `Get-Service mosquitto` | `Running` |
| A | `python -m http.server 8000 -d tools`, then open `http://localhost:8000/c2_dashboard.html` | The page loads and reads CONNECTING (no bridge yet; expected) |
| B | `python main.py --config config/fallback.yaml` | `C2 connected to localhost:1883 as killswitch-01`, then a resolved serial port |
| C | `python -m tools.operator_console --config config/fallback.yaml` | The **Operator Console** window opens and reads `LINK OK` |
| D, **at T-0** | `python -m tools.c2_bridge --config config/fallback.yaml --threat-start-m 420` | `Operator tasks: token checked…`, `MQTT connected`, `WebSocket serving` |

**The bridge goes last** because the 420 m cycle starts the moment it launches. BETA-01
breaches at about 1.7 s, and every threat has resolved by about 13.7 s. Started first, the
cycle burns away while the engine is still connecting.

After D starts, the dashboard reads CONNECTED with a **green `PHYSICAL GIMBAL`** badge,
and DEADMAN AGE shows a live value under 0.250 s. A cyan `LOOPBACK SIM` badge means the
engine is on a mock, not the rig.

## T-0: execution

Times are measured from launching window D.

| t | Pill | Who | Action | Say |
|---|---|---|---|---|
| 0 | IDLE | OPERATOR | Launch D. **Click the Operator Console window's title bar.** From now on, only SPACE or D | — |
| ≈1.7 s | SCAN | — | BETA-01 crosses 350 m; the gimbal slews to pan 45° | "The radar picture cues the gimbal." |
| ≈2–5 s | SCAN → TRACK | HANDLER | Target at the tape. While the pill reads **TRACK**, optionally slide the stand's base slowly sideways, then stop | "It's closing a real control loop on a real camera." |
| +3 s | HOLD | HANDLER | Hands off; step back behind the gimbal | "It has to hold three seconds inside fifteen pixels." |
| amber | OPERATOR_AUTH, `AUTH WINDOW 10.0 s left` | OPERATOR | Call and response (below), then SPACE, aiming for t ≈ 10.0–10.5 s | — |
| red | ENGAGE, `FIRING` | All | Watch | "The firmware, not the laptop, bounds this burn." |
| +1.8 s | IDLE (`ENGAGEMENT COMPLETE`) | PRESENTER | GAMMA-01 arrives from 180°, outside the arc, and is refused | "Out of arc, so it's refused, not chased." |

**Call and response before SPACE:**

> PRESENTER: "Firing solution on track BETA-01: held three seconds, inside fifteen pixels."
> OPERATOR (eyes on the console, which reads `AUTHORISATION REQUIRED`): "BETA-01 confirmed hostile. Beam path clear. Authorising." → **SPACE**
>
> Deny variant: "Beam path not clear. Denied." → **D**

## Recovery

| Symptom | Do this |
|---|---|
| Laser on when it shouldn't be | **Pull the ESP32 USB** (off within 250 ms) |
| SPACE did nothing | Keys only reach the **Operator Console** window: click its title bar. SPACE only counts in OPERATOR_AUTH. If the window expired, the node re-holds and asks again about 3 s later. A late SPACE is **discarded, not saved**, so press again in the new window |
| `Could not open COM3` | Something else holds the port (`serial_probe`, the Arduino serial monitor), or it moved: `serial_probe --list`. Otherwise use the fallback below |
| Pill stuck in SCAN | The target isn't detected: check it's at the tape, check the lighting, re-run `vision_probe` |
| Dashboard reads CONNECTING | Window D isn't running |
| STATE `NO LINK`, Panel C `NO NODE TELEMETRY` | Window B isn't running or isn't on the broker |

## Fallback: no hardware (`--sim-target`)

Relaunch **only window B**: `python main.py --config config/fallback.yaml --sim-target`.

The camera and detector become a synthetic target, and the mock actuator is forced, so
the real laser can never fire. The badge reads cyan **`LOOPBACK SIM`**. **Say so out
loud**: a judge who spots it unprompted will assume you hid it.

## Facts to say, and not to say

| Say | Don't say |
|---|---|
| Windows laptop + ESP32-S3; YOLO11s | Jetson, YOLOv8, GStreamer |
| KY-008, a milliwatt-class **proxy** for a 3–5 kW laser | "a simulated 3–5 kW laser" |
| P + velocity feed-forward + lead prediction | "PID" |
| IDLE → SCAN → TRACK → HOLD → OPERATOR_AUTH → ENGAGE | DETECT, BRANCH, EXECUTE |
| Authorisation is SPACE, bound to the track id, single use | "click Approve" |
