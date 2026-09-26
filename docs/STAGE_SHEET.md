# Stage Sheet: physical node

Windows laptop → CH343 `COM3` → ESP32-S3 → two SG90s and a KY-008 650 nm laser (a
low-power proxy for a high-energy laser). The camera is the OV5640 on the ESP32-S3, mounted
on the gimbal, streaming to the laptop over Wi-Fi (firmware v3).

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
- [ ] The actuator board runs `firmware/esp32_actuator` **v3** (its boot line says
  `killswitch-actuator v3`) and nothing else. **Never flash any other sketch onto it**,
  a stock CameraWebServer included: on this board's camera connector GPIO 7 is HREF, a
  camera line, and a sketch that doesn't know about the laser can drive it.

## One-time: rewire, flash, re-verify (before the first v3 run)

The camera's connector owns GPIO 4–18. Firmware v3 moves the actuator off 5/6/7 and will
not build if any actuator pin lands on a camera pin.

| ☐ | Step | Action | Pass when |
|---|---|---|---|
| ☐ | Power off | Unplug the ESP32 USB **and** the servo rail | Nothing lit |
| ☐ | Pan servo | Signal wire GPIO **5 → 14** | — |
| ☐ | Tilt servo | Signal wire GPIO **6 → 21** | — |
| ☐ | Laser gate | Move the 1 kΩ resistor's ESP32 end **and** the 10 kΩ pulldown's ESP32 end from GPIO **7 → 1** (the pulldown's other end stays on GND) | **Nothing** left on GPIO 5, 6 or 7 |
| ☐ | Camera | Ribbon seated; camera fixed to the **moving** part of the gimbal, looking where the laser points | It turns with the laser |
| ☐ | Wi-Fi details | In `firmware/esp32_actuator/`, copy `wifi_secrets.example.h` to `wifi_secrets.h`, then set your hotspot name and password | The ESP32-S3 is **2.4 GHz only**: on an iPhone hotspot turn on *Maximise Compatibility* |
| ☐ | Flash | Arduino IDE, Tools: Board **ESP32S3 Dev Module** (not AI Thinker), USB CDC On Boot **Disabled**, PSRAM **OPI PSRAM**, Flash Size **16MB**, Partition **16M Flash (3MB APP/9.9MB FATFS)**, Port **COM3**. Upload. Then **close the Serial Monitor** | Upload finishes. A `static assertion failed` error means a pin collides: fix the wiring constants, never delete the check |
| ☐ | Camera address | Laptop on the same hotspot, no browser tab on the stream. `python -m tools.camera_probe --config config/fallback.yaml --find --write` | `FOUND http://<ip>/stream` and `Wrote camera.stream_url`. If `NOT FOUND`: Arduino Serial Monitor at 921600, press RST, and read the lines: `OK BOOT killswitch-actuator v3.2 … pwm=ok`, `CAM camera ok, sensor 0x5640`, `CAM wifi "<name>" heard on channel …`, then `CAM http://<ip>/stream`. `CAM FAIL` means check the ribbon and the PSRAM setting. `NOT FOUND` means the hotspot is 5 GHz or the name differs; `CAM wifi not joined: …` repeats every 10 s and names the reason |
| ☐ | Direction check | Still in that shell, with `http://<ip>/stream` open in a browser: `a 60 90`, then `a 120 90` | The picture slides **left**. If it slides right, set `camera.flip_horizontal: true` |
| ☐ | | `a 90 80`, then `a 90 100` | The picture slides **down**. If it slides up, set `camera.flip_vertical: true`. Without this the gimbal turns **away** from the target |
| ☐ | Field of view | Put an object at the picture's **right** edge (`a <p1> 90`), then raise pan until it reaches the **left** edge (`a <p2> 90`) | Set `camera.horizontal_fov_deg` to `p2 − p1`. It sets the degrees-per-pixel gain; 65 was the old webcam's |
| ☐ | Re-verify 13/13 | Matte backstop in place. Keep the stream **open in the browser**, `q` the shell, then `python -m tools.serial_probe --port COM3` | **13/13 PASS** with the camera streaming. Anything less: stop and fix before any demo |
| ☐ | Free the stream | Close the browser tab | The stream serves **one viewer at a time**; `main.py` needs it |

## T-60: hardware and host

| ☐ | Step | Command or action | Pass when |
|---|---|---|---|
| ☐ | Code is current | `git pull`, then `git log --oneline -1` | Shows the commit that added this sheet, or later |
| ☐ | Laptop never sleeps | On AC power. `powercfg /change standby-timeout-ac 0`. Power Options → Advanced → USB settings → USB selective suspend → **Disabled** | Suspend would drop the CH343 mid-demo, and the node falls back to IDLE with "serial link lost" |
| ☐ | Power order | 1. ESP32 USB into the **labelled** port. 2. Then the servo 5 V rail | Servos go to centre (90/90); laser off |
| ☐ | Serial port | `python -m tools.serial_probe --list` | A CH343 (`USB-Enhanced-SERIAL CH343`) on **COM3**. On any other COM, set `actuator.port` in `config/fallback.yaml` |
| ☐ | Same network | Laptop and ESP32 both on the phone hotspot (2.4 GHz; iPhone: *Maximise Compatibility*) | — |
| ☐ | Camera stream | Close any browser tab showing the stream. `python -m tools.camera_probe --config config/fallback.yaml` | `camera.stream_url=… -> FOUND (640x480, N fps)`. `NOT FOUND`: the hotspot may have given it a new address, so run it again with `--find --write` |
| ☐ | Mark the target spot | `python -m tools.serial_probe --port COM3 --interactive`, then `a 45 97`, tape where the camera points, then `a 90 90`, then **`q`** | Tape on the backstop. **Quit** before step B: COM3 is exclusive. Never run `serial_probe` without `--interactive` here, because the full probe **fires the laser** |
| ☐ | The target is detected | Target on its stand at the tape. `python -m tools.vision_probe --config config/fallback.yaml`, then **Q** to quit (the camera is exclusive) | A box labelled `bird`, `airplane`, `kite` or `frisbee` at **≥ 0.40**. COCO has no drone class, so a drone prop usually fails this |
| ☐ | Works without internet | `dir yolo11s.pt` in the repo folder, then run the whole sheet once with **no internet** (hotspot mobile data off; the camera still needs the hotspot itself) | The file exists. It only auto-downloads when online |
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
| A | Nothing to start. Window D opens the dashboard in Chrome itself. Closed it? `start chrome "$PWD\tools\c2_dashboard.html"` from the repo folder. Never open `:8765` in a browser: that is the data feed, not the page | — |
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
| Node drops to IDLE with `camera lost` | The stream stalled: Wi-Fi, ESP32 power, or a browser tab took the one stream slot. **Close any tab on the stream.** Window B keeps reconnecting and logs `Camera stream … recovered` when frames return; the next cue then works. Still lost after 30 s: press RST on the ESP32 (safe: the laser pin boots low), then restart window B. Firmware v3.2 restarts a stalled sensor by itself and prints `CAM sensor stalled, camera restarted` |
| Gimbal turns **away** from the target | Wrong image direction: set `flip_horizontal` (pan) or `flip_vertical` (tilt) in `config/fallback.yaml`, restart window B |
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
| The ESP32's camera streams to the laptop over Wi-Fi; YOLO runs on the laptop | "AI on the ESP32" or "edge inference" |
| KY-008, a milliwatt-class **proxy** for a 3–5 kW laser | "a simulated 3–5 kW laser" |
| P + velocity feed-forward + lead prediction | "PID" |
| IDLE → SCAN → TRACK → HOLD → OPERATOR_AUTH → ENGAGE | DETECT, BRANCH, EXECUTE |
| Authorisation is SPACE, bound to the track id, single use | "click Approve" |
