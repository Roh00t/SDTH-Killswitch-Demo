"""Camera bring-up probe — run this FIRST, before tuning anything.

Answers the questions the node depends on:
  1. Which device indices deliver frames, and is the one the config opens
     (`camera.device_index`) among them?
  2. Does it actually sustain 30 fps, and does MJPG change that?
  3. Is CAP_PROP_BUFFERSIZE honoured on this machine? (Expect 'no' on macOS.)

To tell which index is the external camera, unplug it and run this again: the
index that disappears is the external one.

The ESP32-S3 camera's address changes whenever the hotspot hands out a new
one. --find looks for it on this laptop's networks; --write puts it into the
config's camera.stream_url.

Usage:
    python -m tools.camera_probe --config config/fallback.yaml
    python -m tools.camera_probe --config config/fallback.yaml --find --write
    python -m tools.camera_probe --index 0 --seconds 5
"""
from __future__ import annotations

import argparse
import ipaddress
import platform
import re
import socket
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, List, Optional, Sequence, Tuple

import cv2
import yaml

TEST_MODES: List[Tuple[int, int]] = [(640, 480), (800, 600), (1280, 720), (1920, 1080)]


def backend_for_platform() -> Tuple[int, str]:
    system = platform.system()
    if system == "Darwin":
        return cv2.CAP_AVFOUNDATION, "AVFoundation"
    if system == "Linux":
        return cv2.CAP_V4L2, "V4L2"
    if system == "Windows":
        return cv2.CAP_DSHOW, "DirectShow"
    return cv2.CAP_ANY, "auto"


def enumerate_devices(max_index: int = 5) -> List[int]:
    backend, _ = backend_for_platform()
    found = []
    for index in range(max_index):
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                found.append(index)
        cap.release()
    return found


def no_camera_hint(system: str) -> List[str]:
    """What to check when no index delivers a frame, for this OS."""
    if system == "Windows":
        return [
            "  Windows: Settings > Privacy & security > Camera: turn ON both",
            "  'Camera access' and 'Let desktop apps access your camera'.",
            "  Close anything holding the camera (Teams, Zoom, the Camera app, browser",
            "  tabs, a running main.py or vision_probe). Plug it into the laptop, not a hub.",
        ]
    if system == "Darwin":
        return [
            "  Check the cable and macOS camera permissions",
            "  (System Settings > Privacy & Security > Camera > Terminal).",
        ]
    return ["  Check the cable, that /dev/video* exists, and that you are in the 'video' group."]


def load_configured_index(path: str) -> Optional[int]:
    """`camera.device_index` from a node config, or None if it cannot be read."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError) as exc:
        print(f"  Could not read {path}: {exc}")
        return None
    index = (config.get("camera") or {}).get("device_index")
    return index if isinstance(index, int) and not isinstance(index, bool) else None


def load_stream_url(path: str) -> Optional[str]:
    """`camera.stream_url` from a node config, or None when unset or unreadable."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return None
    url = (config.get("camera") or {}).get("stream_url")
    return url if isinstance(url, str) and url.strip() else None


STREAM_HINTS = [
    "  Open the URL in a browser on this laptop. If that shows nothing too:",
    "  - The address may have changed: run this again with --find --write.",
    "  - The ESP32 prints its address on COM3 after boot: run",
    "    'python -m tools.serial_probe --port COM3 --interactive' and read the",
    "    'Firmware: CAM http://.../stream' line (then q, before starting main.py).",
    "  - Laptop and ESP32 must be on the same network, and the ESP32-S3 is 2.4 GHz",
    "    only (iPhone hotspot: turn on Maximise Compatibility).",
    "  - Only ONE viewer at a time: close any browser tab showing the stream.",
]


def probe_stream(url: str, seconds: float = 3.0) -> Tuple[bool, str]:
    """Open the Wi-Fi camera stream, count frames for `seconds`, report.

    Returns:
        (found, a line to print).
    """
    from helper.vision.frame_source import HttpStreamSource

    src = HttpStreamSource(url, reconnect_window_s=seconds)
    try:
        src.start()
    except RuntimeError:
        return False, f"camera.stream_url={url} -> NOT FOUND (no frames)"
    try:
        _, first = src.read()
        time.sleep(seconds)
        _, last = src.read()
        width, height = src.frame_size
    finally:
        src.stop()
    return True, (f"camera.stream_url={url} -> FOUND "
                  f"({width}x{height}, {(last - first) / seconds:.1f} fps)")


# The firmware's own names: CAM_HOSTNAME and STREAM_BOUNDARY in
# firmware/esp32_actuator/esp32_actuator.ino.
CAMERA_MDNS_NAME = "killswitch-cam.local"
STREAM_BOUNDARY = "killswitchframe"

FIND_HINTS = [
    "  - The laptop and the ESP32 must be on the same hotspot. The ESP32-S3 hears",
    "    2.4 GHz only (Android: hotspot band 2.4 GHz; iPhone: Maximise Compatibility).",
    "  - Firmware v3.1 says why Wi-Fi failed: Arduino Serial Monitor at 921600,",
    "    press RST on the board, and read the 'CAM wifi ...' lines.",
]


def classify_stream_reply(head: bytes) -> Optional[str]:
    """Name what answered GET /stream, from the start of its reply.

    Args:
        head: The reply's first bytes, at least the status line and headers.

    Returns:
        "killswitch" for this repo's firmware, "mjpeg" for any other MJPEG
        server, None for anything else.
    """
    status, _, headers = head.decode("latin-1").partition("\r\n")
    if " 200" not in status:
        return None
    headers = headers.lower()
    if "multipart/x-mixed-replace" not in headers:
        return None
    return "killswitch" if f"boundary={STREAM_BOUNDARY}" in headers else "mjpeg"


def probe_host(host: str, port: int = 80, connect_timeout: float = 0.4,
               reply_timeout: float = 1.5) -> Optional[str]:
    """Ask one address for /stream, then hang up. Runs on the finder's workers.

    Hanging up after the headers frees the firmware's single viewer slot.

    Returns:
        classify_stream_reply()'s answer; "busy" when the port accepts but
        says nothing (the firmware's one viewer slot is taken); None when
        nothing listens.
    """
    try:
        sock = socket.create_connection((host, port), timeout=connect_timeout)
    except OSError:
        return None
    head = b""
    with sock:
        sock.settimeout(reply_timeout)
        try:
            sock.sendall(f"GET /stream HTTP/1.1\r\nHost: {host}\r\n"
                         "Connection: close\r\n\r\n".encode("ascii"))
            while b"\r\n\r\n" not in head and len(head) < 2048:
                chunk = sock.recv(512)
                if not chunk:
                    break
                head += chunk
        except socket.timeout:
            if not head:
                return "busy"
        except OSError:
            return None
    return classify_stream_reply(head)


def subnet_hosts(own_ip: str) -> List[str]:
    """Every other address in own_ip's /24. Phone hotspots hand out a /24 or less."""
    network = ipaddress.ip_network(f"{own_ip}/24", strict=False)
    return [str(host) for host in network.hosts() if str(host) != own_ip]


def local_ipv4s() -> List[str]:
    """This laptop's IPv4 addresses that could share a network with the ESP32.

    The address of the default route comes first: that is the network the
    laptop is actually using.
    """
    found: List[str] = []
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
            route.connect(("8.8.8.8", 53))   # UDP: picks a route, sends nothing
            found.append(route.getsockname()[0])
    except OSError:
        pass
    try:
        found += [info[4][0] for info in
                  socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)]
    except OSError:
        pass
    usable: List[str] = []
    for ip in found:
        address = ipaddress.ip_address(ip)
        if not (address.is_loopback or address.is_link_local) and ip not in usable:
            usable.append(ip)
    return usable


def resolve_with_timeout(name: str, timeout: float = 2.0) -> Optional[str]:
    """name's IPv4 address, or None. The OS resolver has no timeout of its own."""
    result: List[str] = []

    def lookup() -> None:
        try:
            result.append(socket.gethostbyname(name))
        except OSError:
            pass

    worker = threading.Thread(target=lookup, daemon=True)
    worker.start()
    worker.join(timeout)
    return result[0] if result else None


def find_camera(hosts: Sequence[str],
                probe: Callable[[str], Optional[str]] = probe_host,
                workers: int = 64) -> Tuple[Optional[str], List[str]]:
    """Probe every host in parallel for the firmware's stream.

    Returns:
        (the first address serving the Killswitch stream or None,
         every other address that answered, as "ip (kind)").
    """
    with ThreadPoolExecutor(max_workers=workers) as pool:
        kinds = list(pool.map(probe, hosts))
    found = next((h for h, k in zip(hosts, kinds) if k == "killswitch"), None)
    others = [f"{h} ({k})" for h, k in zip(hosts, kinds) if k and k != "killswitch"]
    return found, others


_STREAM_URL_LINE = re.compile(
    r"^(?P<lead>[ \t]+stream_url:[ \t]*)(?P<value>[^#\n]*?)(?P<tail>[ \t]*(?:#.*)?)$",
    re.MULTILINE)


def write_stream_url(path: str, url: str) -> bool:
    """Set camera.stream_url in a node config, keeping every comment.

    Returns:
        True once the file loads with camera.stream_url == url. False, with
        the file untouched, when there is not exactly one stream_url line.
    """
    with open(path, "r", encoding="utf-8") as handle:
        original = handle.read()
    updated, count = _STREAM_URL_LINE.subn(
        lambda m: f'{m.group("lead")}"{url}"{m.group("tail")}', original)
    if count != 1:
        return False
    camera = (yaml.safe_load(updated) or {}).get("camera") or {}
    if camera.get("stream_url") != url:
        return False
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(updated)
    return True


def run_find(config_path: str, write: bool) -> int:
    """Find the camera by name, else by scanning, and report or write its URL."""
    print("Looking for the Killswitch camera...")
    host: Optional[str] = None
    others: List[str] = []
    named = resolve_with_timeout(CAMERA_MDNS_NAME)
    if named is not None:
        print(f"  {CAMERA_MDNS_NAME} -> {named}")
        if probe_host(named) == "killswitch":
            host = named
    if host is None:
        own = local_ipv4s()
        if not own:
            print("  This laptop has no network address. Join the hotspot first.")
            return 1
        hosts: List[str] = []
        for ip in own[:4]:
            network = ipaddress.ip_network(f"{ip}/24", strict=False)
            print(f"  Scanning {network} (this laptop is {ip})")
            hosts += [h for h in subnet_hosts(ip) if h not in hosts]
        host, others = find_camera(hosts)
    if host is None:
        print("  NOT FOUND.")
        for line in others:
            print(f"  Answered, but not the Killswitch stream: {line}")
        if any(line.endswith("(busy)") for line in others):
            print("  - 'busy' means it accepts but won't talk: close any browser tab showing")
            print("    the stream, then run this again. The stream has one viewer slot.")
        print("\n".join(FIND_HINTS))
        return 1

    url = f"http://{host}/stream"
    print(f"  FOUND {url}")
    if write:
        if not write_stream_url(config_path, url):
            print(f"  Could not update {config_path}: set camera.stream_url by hand.")
            return 1
        print(f"  Wrote camera.stream_url in {config_path}.")
    else:
        print(f'  Put this in {config_path} under camera:   stream_url: "{url}"')
        print("  or run this again with --write to set it for you.")
    print(f"  Next: python -m tools.camera_probe --config {config_path}")
    return 0


def check_configured_index(
    working: Sequence[int], configured: Optional[int], config_path: str,
) -> Tuple[bool, str]:
    """Is the index the node will open one that actually delivers frames?

    Returns:
        (found, a line to print).
    """
    if configured is None:
        return False, f"Config {config_path}: no usable camera.device_index"
    if configured in working:
        return True, f"Config {config_path}: camera.device_index={configured} -> FOUND"
    return False, (
        f"Config {config_path}: camera.device_index={configured} -> NOT FOUND. "
        f"Working indices: {list(working)}. Set camera.device_index to the external one."
    )


def measure_mode(index: int, width: int, height: int, use_mjpg: bool, seconds: float):
    """Open at a mode and measure sustained fps. Returns a result dict or None."""
    backend, _ = backend_for_platform()
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        return None

    if use_mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    buffersize_ok = bool(cap.set(cv2.CAP_PROP_BUFFERSIZE, 1))

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None

    actual = (frame.shape[1], frame.shape[0])
    frames, latencies = 0, []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        t0 = time.monotonic()
        ok, _ = cap.read()
        if not ok:
            break
        latencies.append((time.monotonic() - t0) * 1000.0)
        frames += 1

    cap.release()
    if not latencies:
        return None

    latencies.sort()
    return {
        "requested": (width, height),
        "actual": actual,
        "mjpg": use_mjpg,
        "fps": frames / seconds,
        "read_ms_median": latencies[len(latencies) // 2],
        "read_ms_p95": latencies[int(len(latencies) * 0.95)],
        "buffersize_ok": buffersize_ok,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index", type=int, default=None, help="Device index (default: probe)")
    parser.add_argument("--seconds", type=float, default=3.0, help="Seconds per mode")
    parser.add_argument("--config", default="config/bench.yaml",
                        help="Node config whose camera.device_index is checked")
    parser.add_argument("--find", action="store_true",
                        help="Find the ESP32 camera on this laptop's networks")
    parser.add_argument("--write", action="store_true",
                        help="With --find: set camera.stream_url in --config")
    args = parser.parse_args()

    if args.find:
        return run_find(args.config, args.write)
    if args.write:
        parser.error("--write only works with --find")

    _, backend_name = backend_for_platform()
    print(f"Platform : {platform.system()}  |  OpenCV {cv2.__version__}  |  backend {backend_name}")

    if args.index is None and load_stream_url(args.config):
        url = load_stream_url(args.config)
        print(f"\nConfig {args.config} uses the Wi-Fi camera stream.")
        found, line = probe_stream(url, args.seconds)
        print(line)
        if not found:
            print("\n".join(STREAM_HINTS))
            return 1
        return 0

    if args.index is None:
        configured = load_configured_index(args.config)
        print("\nEnumerating devices...")
        devices = enumerate_devices()
        if not devices:
            print("  No cameras found.")
            print("\n".join(no_camera_hint(platform.system())))
            print(check_configured_index(devices, configured, args.config)[1])
            return 1
        print(f"  Working indices: {devices}")
        found, line = check_configured_index(devices, configured, args.config)
        print(line)
        print("  Which one is the external camera? Unplug it and run this again:")
        print("  the index that disappears is the external one.")
        if not found:
            return 1
        # Measure the camera the node will open, not whichever index came first
        # (usually the laptop's built-in one).
        index = configured
    else:
        index = args.index

    print(f"\nProbing index {index} ({args.seconds}s per mode)\n")
    header = f"{'mode':>11} {'fmt':>5} {'actual':>11} {'fps':>7} {'med ms':>8} {'p95 ms':>8}"
    print(header)
    print("-" * len(header))

    results = []
    for width, height in TEST_MODES:
        for use_mjpg in (True, False):
            r = measure_mode(index, width, height, use_mjpg, args.seconds)
            if r is None:
                continue
            results.append(r)
            print(
                f"{width:>5}x{height:<5} {'MJPG' if use_mjpg else 'YUYV':>5} "
                f"{r['actual'][0]:>5}x{r['actual'][1]:<5} {r['fps']:>7.1f} "
                f"{r['read_ms_median']:>8.1f} {r['read_ms_p95']:>8.1f}"
            )

    if not results:
        print("  No mode produced frames.")
        return 1

    print()
    if not results[0]["buffersize_ok"]:
        print("CAP_PROP_BUFFERSIZE : IGNORED by this backend (expected on macOS).")
        print("                      Stale-frame rejection relies on the grabber")
        print("                      thread in helper/vision/frame_source.py.")
    else:
        print("CAP_PROP_BUFFERSIZE : honoured (grabber thread still used).")

    best = max(results, key=lambda r: (round(r["fps"]), r["actual"][0] * r["actual"][1]))
    print(
        f"\nRecommended        : {best['actual'][0]}x{best['actual'][1]} "
        f"{'MJPG' if best['mjpg'] else 'YUYV'} @ {best['fps']:.1f} fps"
    )
    print(f"Capture budget     : ~{1000.0 / best['fps']:.0f} ms/frame")
    print("\nPut these numbers into architecture.md section 8, replacing the estimates.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
