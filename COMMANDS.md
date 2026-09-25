Your Wi-Fi adapter is successfully locked to `Private`. Windows will now correctly route the incoming UDP multicast traffic for the WinTAK CoT XML stream instead of silently dropping it.

Gate 1 is fully cleared. You have confirmed a stable 5V external power rail with a common ground for the physical servos, and your network transport layer is properly configured.

For your first full hardware-in-the-loop dry run at the NUS Defence Venture Lab hackathon today, launch the stack in this exact sequence:

* **Terminal 1 (Broker):** `"C:\Program Files\mosquitto\mosquitto.exe" -v`
* **Terminal 2 (Engine):** `python main.py --config config/fallback.yaml`
* **Terminal 3 (Dashboard):** `start tools\c2_dashboard.html`
* **Terminal 4 (Auth Gate):** `python -m tools.operator_console --config config/fallback.yaml`
* **Terminal 5 (C2 Bridge):** `python -m tools.c2_bridge --threat-start-m 420`

Run Terminal 5 last to trigger the 14-second threat breach cycle. Let me know if the gimbal physically tracks the target once you hit `SPACE` in Terminal 4 to authorize the engagement.