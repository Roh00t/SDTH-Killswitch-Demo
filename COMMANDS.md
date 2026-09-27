# Bring the system live

PowerShell, from the repo folder, **one command at a time**. `>>` in the prompt means
several lines went in together. Every window's prompt must end in `SDTH-Killswitch-Demo>`.

## 1. Hardware

1. ESP32 USB into the **UART** port first (`COM3`, CH343), then the servo 5 V supply.
2. Laptop and ESP32 on the same 2.4 GHz Wi-Fi (`WIFI_SSID` in
   `firmware/esp32_actuator/wifi_secrets.h`).
3. Laser at a matte backstop, nobody downrange. Target at the tape mark.

Flashing? Arduino IDE, **ESP32S3 Dev Module**, USB CDC On Boot **Disabled**, PSRAM **OPI
PSRAM**, Flash Size **16MB**, Port `COM3`. The boot line must read
`OK BOOT killswitch-actuator v3.5 … pwm=ok`. Close the Serial Monitor afterwards.

## 2. Clean slate

```powershell
cd C:\Users\rohit\Documents\GitHub\SDTH-Killswitch-Demo
Get-Process python* | Stop-Process -Force
Get-Service mosquitto        # Start-Service mosquitto if it is not Running
```

## 3. Target, camera, preflight

```powershell
python -m tools.vision_probe --config config/bench.yaml      # drone photo; or config/fallback.yaml for a bird/frisbee
python -m tools.camera_probe --config config/bench.yaml --find --write
python -m tools.preflight --config config/bench.yaml
```

Keep the config that draws a box at 0.40 or more, and use it everywhere below. Wait for
`READY`.

## 4. Three windows, in this order

| Window | Command | Wait for |
|---|---|---|
| B, node | `python main.py --config config/bench.yaml` | `C2 connected` |
| C, operator console | `python -m tools.operator_console --config config/bench.yaml` | The camera in the console |
| D, bridge (last) | `python -m tools.c2_bridge --config config/bench.yaml --threat-start-m 420` | The dashboard opens and reads PHYSICAL GIMBAL |

TRACK → HOLD → amber, then click the console and press **SPACE**: a 1.8 s burn.

Stop with Ctrl+C in D, then C, then B. The node parks the gimbal and turns the laser off.

## Checks and tools

```powershell
python -m tools.serial_probe --port COM3 --servo-sweep    # listen at each end: a buzz is a stall
python -m tools.serial_probe --port COM3                  # 13 interlock checks; FIRES the laser
python -m tools.laser_test                                # node stopped: L toggles the laser
python main.py --config config/fallback.yaml --sim-target # hardware-free demo, real MQTT
pytest tests/ -q
```
