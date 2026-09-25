# Scale Sheet: ATAK-CIV and the C2 dashboard, 1 operator : 3 assets

The Scale Sheet runs as its **own cycle, after the Stage Sheet**. Windows 0, A, B and C
stay up; only the bridge (window D) is relaunched. **Window B is still the real rig, so
every Stage Sheet safety rule still applies.** Node 1 is cued and asks for
authorisation in this cycle too.

**What it shows:**
- One real node, whose map label is whatever the node itself reports.
- Three simulated assets, labelled `SIMULATED` on every marker.
- One operator tasking each simulated asset by hand: keys `1`, `2`, `3`.
- Tasking labels only. It never engages, never moves a servo and never touches Node 1.

## T-10: phone and network

| ☐ | Step | Command or action | Pass when |
|---|---|---|---|
| ☐ | Phone ready | Android with ATAK-CIV, map around NUS cached, screen awake, battery optimisation **off** for ATAK | ATAK open on the map |
| ☐ | Same network | Laptop on the phone's hotspot, or both on the same Wi-Fi | — |
| ☐ | Phone IP, **re-read now** | On the phone's own hotspot, the phone is the laptop's **Default Gateway** in `ipconfig`; otherwise read it in the phone's Wi-Fi details | Written down. Hotspots reassign addresses |
| ☐ | Firewall: outbound | `Get-NetFirewallProfile \| Format-Table Name, Enabled, DefaultOutboundAction` | `Allow` or `NotConfigured` on the active profile. The bridge sends from a random local port to the phone's port **6969**. If it reads `Block`, run `tools\win_firewall_cot.ps1` as Administrator: it adds an outbound rule for remote port 6969 |
| ☐ | Firewall: profile | `Get-NetConnectionProfile` | `Private`, not `Public` |
| ☐ | ATAK input | ATAK → Settings → Network Preferences → Network Connection Preferences → Manage Inputs | A UDP input on `239.2.3.1:6969` is enabled. If unicast still shows nothing, add a UDP input on port **6969** |

## Start

Stop window D (Ctrl+C), then relaunch it:

```bat
python -m tools.c2_bridge --config config/fallback.yaml --threat-start-m 700 --sim-nodes 3 --cot-unicast <ANDROID_IP> --ws-host 0.0.0.0
```

| Flag | Why |
|---|---|
| `--config config/fallback.yaml` | Reads the token that keys 1–3 carry. It must be the **same file** windows B and C use, or every key is rejected |
| `--threat-start-m 700` | BETA breaches at about 8 s and the last threat resolves at about 23 s, leaving time for the map walk and the keys. At `420`, everything is gone by about 13.7 s |
| `--sim-nodes 3` | Exactly three simulated assets: `KILLSWITCH-02` SENTRY-UGV, `-03` LASER, `-04` INTERCEPTOR |
| `--cot-unicast <ANDROID_IP>` | A direct copy to the phone. Many hotspots drop multicast |
| `--ws-host 0.0.0.0` | The dashboard on the LAN, receive-only |

**Healthy when the bridge prints:**
- `CoT -> 239.2.3.1:6969 + unicast <ANDROID_IP>:6969  |  4 nodes (1 real)`
- `Operator tasks: token checked against c2.auth_token in config/fallback.yaml`
- `MQTT connected`
- `WebSocket serving`

## Execution

Times are measured from relaunching window D; each was measured on the full stack.

| t | ATAK map | Dashboard | Operator | Presenter says |
|---|---|---|---|---|
| 0–8 s | `KILLSWITCH-01 [<state>]` at the site. `KILLSWITCH-02/03/04` remarks `SIMULATED <KIND>`. Four red `SIMULATED THREAT` markers inbound | Four amber diamonds | — | "Node 1 is the real node. Tap it: REAL NODE, and its state is whatever the node reports. Tap any other asset: SIMULATED." |
| any time before ≈16 s | — | — | In the **Operator Console** window, press **1**, then **2**, then **3** | "Each keypress is a human tasking decision." |
| after 1 | `KILLSWITCH-02 [TASKED]`: `SIMULATED SENTRY-UGV \| TASKED BY OPERATOR -> SWARM-ALPHA-02` | ALPHA-02 row STATUS: `INBOUND · TASKED KILLSWITCH-02` | — | — |
| after 2 | `KILLSWITCH-03 [TASKED]` → `SWARM-ALPHA-01` | ALPHA-01 row shows TASKED | — | — |
| after 3 | `KILLSWITCH-04 [TASKED]` → `SWARM-GAMMA-01` | GAMMA-01 row shows TASKED | — | "GAMMA comes from behind the real node's arc. The node refuses it, so a human assigns cover." |
| ≈8–9 s | BETA crosses 350 m; Node 1 is cued onto it | BETA row turns red | — | "The real node takes BETA itself." |
| ≈15 s (measured with `--sim-target`; the rig depends on its lock) | `KILLSWITCH-01 [OPERATOR_AUTH]` | `AUTH WINDOW 10.0 s left` | SPACE with the Stage Sheet call and response, **or** let it time out | — |
| ≈18.7 / 20.2 / 23.7 s | Each asset returns to `SIMULATED <KIND>` as its threat resolves | TASKED clears | — | "Tasks end when the threat does." |
| ≈23.7 s | The scenario resets; threats reappear about 1.2 km out | — | — | — |

**Which threat each key picks.** Node 1 always owns BETA, the threat it is cued onto.
Each key gives its asset the highest-priority threat nobody owns yet. So it's
1 → ALPHA-02, 2 → ALPHA-01, 3 → GAMMA-01, every run.

**If a judge asks what the keys do:**
- They label a simulated asset as tasked.
- They never engage anything, never move a servo, and never change Node 1 or a threat.
- They carry the same token as authorisation.
- A rejected key is logged in window D as `Rejected operator task: …`.
- A repeat press is ignored.

## Verbal pivot

> "Node 1 is a real closed loop: a real camera, real servos and a real low-power laser,
> and every state on its map marker is reported by the node itself. The other three
> assets and the swarm are a labelled simulation, because we can't fly drones or fire a
> kilowatt-class laser indoors, and what they prove is the part that scales: one
> operator making every tasking and firing decision by hand."

## Recovery

| Symptom | Do this |
|---|---|
| No markers on the phone | Re-read the phone IP and relaunch D. Check the outbound firewall, the ATAK input on 6969, and that the phone is awake |
| Threats vanish the moment they appear | The phone's clock is ahead: add `--stale-pad-s 5` (up to 60) |
| Keys 1–3 do nothing | Click the **Operator Console** window's title bar. Then read window D: `token mismatch` means windows B, C and D aren't all on `--config config/fallback.yaml`; `already tasked` means it was pressed twice; `every inbound threat is already covered` means all four are owned |
| Tasks cleared early | Their threats resolved. Relaunch D for a fresh cycle |
