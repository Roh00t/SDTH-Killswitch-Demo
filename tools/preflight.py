"""One-command preflight: is the rig ready for a live run? Never fires the laser.

Every failure the rig has shown so far was set-up, not logic: COM3 held by a
leftover python, the one-viewer stream held by a browser tab, a stale camera
address, the wrong model for the target, a broker that was not running. This
checks all of them in the order the stack needs them, prints one PASS / WARN /
FAIL row per check with the fix, and exits non-zero on any FAIL.

It sends NO command to the firmware. It resets the ESP32 the way the RST button
does (an RTS pulse on the CH343, as esptool's hard reset does), because this board
does not reset when the port opens, and the boot line is the only place the
firmware states its version and whether PWM attached. A reset is safe: setup()
drives the laser pin LOW as its first statement and boots with the e-stop latched.

Usage (from the repo folder, with nothing else running):
    python -m tools.preflight --config config/bench.yaml
    python -m tools.preflight --config config/fallback.yaml --seconds 10
    python -m tools.preflight --config config/bench.yaml --no-reset
"""
from __future__ import annotations

import argparse
import logging
import re
import socket
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable, Dict, List, Optional, Protocol, Sequence, Tuple

import numpy as np
import yaml

from helper.hardware.actuator import ActuatorError, port_open_error, resolve_port
from helper.vision.frame_source import FrameSource, normalize_stream_url
from helper.vision.types import Detection

# Hit ratio of frames carrying a target at or above the node's threshold. HOLD
# needs 3 s unbroken inside the band, and the track is dropped after
# track_loss_timeout_s without one, so a flickering detection costs the take.
PASS_HIT_RATIO = 0.8
WARN_HIT_RATIO = 0.3
PROBE_CONF_FLOOR = 0.05       # low on purpose: show what the model sees instead
BOOT_LISTEN_S = 25.0          # boot banner plus Wi-Fi join on a phone hotspot
RST_PROMPT_AFTER_S = 5.0      # no boot line by now: the RTS pulse did not reset it
NO_RESET_LISTEN_S = 3.0       # status frames arrive at 10 Hz


class Status(Enum):
    """Outcome of one check."""

    PASS = "PASS"
    WARN = "WARN"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass(frozen=True)
class Row:
    """One printed line of the preflight.

    Attributes:
        name: Short check name.
        status: PASS, WARN, FAIL or SKIP.
        detail: What was found.
        fix: What to do about it, or None when there is nothing to do.
    """

    name: str
    status: Status
    detail: str
    fix: Optional[str] = None


@dataclass
class BootReport:
    """What the firmware said on the serial line after the reset.

    Attributes:
        banner: The ``OK BOOT ...`` line, or None if it never arrived.
        attach_failed: The firmware reported a servo PWM attach failure.
        cam_lines: Every ``CAM ...`` line, in order.
        status_frames: Count of ``ST ...`` frames seen.
        first_uptime_ms: Firmware uptime from the first status frame.
        prompted: The operator was asked to press RST.
    """

    banner: Optional[str] = None
    attach_failed: bool = False
    cam_lines: List[str] = field(default_factory=list)
    status_frames: int = 0
    first_uptime_ms: Optional[int] = None
    prompted: bool = False


@dataclass
class DetectionTally:
    """Counts from running the detector over live frames.

    Attributes:
        frames: New frames the detector processed.
        hits: Frames with a target class at or above the node's threshold.
        peaks: Highest confidence seen per class, any class.
        infer_s: Total time spent inside ``detect``.
        elapsed_s: Wall time of the whole run.
    """

    frames: int = 0
    hits: int = 0
    peaks: Dict[str, float] = field(default_factory=dict)
    infer_s: float = 0.0
    elapsed_s: float = 0.0


class LinePort(Protocol):
    """The slice of ``serial.Serial`` the boot reader uses."""

    dtr: bool
    rts: bool

    def readline(self) -> bytes: ...


class DetectsFrames(Protocol):
    """The slice of ``UltralyticsDetector`` the detection check uses."""

    def detect(self, frame: np.ndarray) -> List[Detection]: ...


# --------------------------------------------------------------------------- #
# Pure checks
# --------------------------------------------------------------------------- #

def check_weights(weights: str) -> Row:
    """Is the model file on disk? An offline laptop cannot fetch it mid-take.

    Thread: main thread.
    """
    if Path(weights).is_file():
        return Row("weights", Status.PASS, f"{weights} on disk")
    fix = "Check detector.weights in the config, and run from the repo folder."
    if not Path(weights).parent.parts:  # a bare name such as yolo11s.pt
        fix = (f"Ultralytics downloads {weights} on first use, which fails offline. "
               f"Run this once with internet so the file lands in the repo folder.")
    return Row("weights", Status.FAIL, f"{weights} not found", fix)


def check_broker(host: str, port: int, timeout_s: float = 1.0) -> Row:
    """Does the MQTT broker accept a TCP connection?

    Thread: main thread.
    """
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return Row("broker", Status.PASS, f"{host}:{port} accepting connections")
    except OSError as exc:
        return Row("broker", Status.FAIL, f"{host}:{port} refused ({exc})",
                   "Start it: Start-Service mosquitto (then Get-Service mosquitto).")


_VERSION = re.compile(r"killswitch-actuator v(\d+)(?:\.(\d+))?")


def parse_boot_version(banner: str) -> Optional[Tuple[int, int]]:
    """``OK BOOT killswitch-actuator v3.2 ...`` -> (3, 2). ``v3`` -> (3, 0)."""
    match = _VERSION.search(banner)
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2) or 0)


def firmware_verdict(banner: Optional[str], attach_failed: bool,
                     pan_reversed: bool, reset_requested: bool) -> Row:
    """Judge the boot line against the laptop's config.

    v3.3 mirrored pan in firmware. With ``actuator.pan_reversed: true`` on the
    laptop as well, pan is reversed twice and the gimbal turns away from every
    cue, with nothing else in the system able to notice.

    Thread: main thread.
    """
    if banner is None:
        if not reset_requested:
            return Row("firmware", Status.WARN, "version not checked (--no-reset)",
                       "Run without --no-reset, or press RST with the Arduino Serial "
                       "Monitor open: the boot line must say v3.2.")
        return Row("firmware", Status.WARN, "no boot line: version and PWM unchecked",
                   "Press RST on the ESP32 while this runs, or run it again.")
    version = parse_boot_version(banner)
    if version is None:
        return Row("firmware", Status.WARN, f"unrecognised boot line: {banner}",
                   "Flash firmware/esp32_actuator from main (v3.2).")
    label = f"v{version[0]}.{version[1]}"
    if version < (3, 0):
        return Row("firmware", Status.FAIL, f"{label} drives the old pins 5/6/7",
                   "Flash firmware/esp32_actuator from main (v3.2).")
    if attach_failed or "pwm=ok" not in banner:
        return Row("firmware", Status.FAIL, f"{label}: servo PWM did not attach",
                   "Check the servo signal wires on GPIO 14/21, then press RST.")
    if version == (3, 3):
        if pan_reversed:
            return Row("firmware", Status.FAIL,
                       "v3.3 mirrors pan in firmware AND actuator.pan_reversed is true: "
                       "pan is reversed twice",
                       "Flash firmware/esp32_actuator from main (v3.2). Or set "
                       "actuator.pan_reversed: false and redo the tape mark.")
        return Row("firmware", Status.PASS, "v3.3 (pan mirrored in firmware), pwm=ok")
    if version < (3, 2):
        return Row("firmware", Status.WARN,
                   f"{label}, pwm=ok, but no self-restart on a stalled camera",
                   "Flash firmware/esp32_actuator from main (v3.2) when you can.")
    return Row("firmware", Status.PASS, f"{label}, pwm=ok")


_CAM_URL = re.compile(r"^CAM (http://\S+/stream)")


def parse_cam_url(line: str) -> Optional[str]:
    """``CAM http://10.0.0.5/stream`` -> the URL. None for any other line."""
    match = _CAM_URL.match(line.strip())
    return match.group(1) if match else None


def _cam_settled(cam_lines: Sequence[str]) -> bool:
    """True once the camera has either come up with an address or said why not."""
    for line in cam_lines:
        if parse_cam_url(line) and "not ready" not in line:
            return True
        if line.startswith("CAM FAIL") or "not joined" in line or "NOT FOUND" in line:
            return True
    return False


def camera_verdict(cam_lines: Sequence[str], configured_url: Optional[str],
                   config_path: str, banner_seen: bool) -> Row:
    """Judge the firmware's camera and Wi-Fi lines against camera.stream_url.

    Thread: main thread.
    """
    if not banner_seen:
        return Row("camera", Status.SKIP, "no boot line, so no camera report")
    for line in cam_lines:
        if line.startswith("CAM FAIL"):
            return Row("camera", Status.FAIL, line,
                       "Reseat the camera ribbon; PSRAM must be OPI in the Arduino IDE.")
    for line in cam_lines:
        if "not joined" in line or "NOT FOUND" in line:
            return Row("camera", Status.FAIL, line,
                       "Hotspot on 2.4 GHz (iPhone: Maximise Compatibility), name and "
                       "password as in wifi_secrets.h, laptop on the same hotspot.")
    reported = [u for u in (parse_cam_url(line) for line in cam_lines) if u]
    if not reported:
        return Row("camera", Status.WARN, "no camera address within the listen window",
                   "Wi-Fi may still be joining. Run the preflight again in 20 s.")
    url = reported[-1]
    if not configured_url:
        return Row("camera", Status.FAIL, f"firmware serves {url}, config has none",
                   f"python -m tools.camera_probe --config {config_path} --find --write")
    if normalize_stream_url(configured_url) != url:
        return Row("camera", Status.FAIL,
                   f"firmware serves {url}, config says {configured_url}",
                   f"python -m tools.camera_probe --config {config_path} --find --write")
    if any("not ready" in line for line in cam_lines if parse_cam_url(line) == url):
        return Row("camera", Status.WARN, f"{url} is up but the sensor is not ready")
    return Row("camera", Status.PASS, f"firmware serves {url}, matching the config")


def detection_verdict(tally: DetectionTally, targets: Sequence[str], conf: float,
                      config_path: str) -> Row:
    """Would the node hold a lock on what the camera sees right now?

    Thread: main thread.
    """
    if tally.frames == 0:
        return Row("detection", Status.FAIL, "no new frames reached the detector",
                   "The stream stalled mid-check. Close any browser tab on it; rerun.")
    ratio = tally.hits / tally.frames
    rate = tally.frames / tally.elapsed_s if tally.elapsed_s > 0 else 0.0
    ms = 1000.0 * tally.infer_s / tally.frames
    target_peak = max((tally.peaks.get(t, 0.0) for t in targets), default=0.0)
    detail = (f"{tally.hits}/{tally.frames} frames ({ratio:.0%}) show "
              f"{'/'.join(targets)} >= {conf:.2f}, peak {target_peak:.2f}; "
              f"{ms:.0f} ms/frame, {rate:.1f} fps")
    if ratio >= PASS_HIT_RATIO:
        return Row("detection", Status.PASS, detail)
    others = sorted(((c, p) for c, p in tally.peaks.items()
                     if c not in targets or p < conf), key=lambda cp: -cp[1])[:3]
    seen = ", ".join(f"{c} {p:.2f}" for c, p in others) or "nothing at all"
    other_cfg = ("config/fallback.yaml" if "drone" in targets
                 else "config/bench.yaml (best.onnx has a drone class)")
    fix = (f"The model saw instead: {seen}. Fill more of the frame, cut glare on "
           f"the target, and watch it live: python -m tools.vision_probe --config "
           f"{config_path}. Or compare --config {other_cfg}.")
    if ratio >= WARN_HIT_RATIO:
        return Row("detection", Status.WARN, detail + " (flickers: HOLD may reset)", fix)
    return Row("detection", Status.FAIL, detail, fix)


# --------------------------------------------------------------------------- #
# Hardware steps, each with the device injected so tests can fake it
# --------------------------------------------------------------------------- #

def pulse_reset(port: LinePort, sleep: Callable[[float], None] = time.sleep) -> None:
    """Reset the ESP32 through the CH343's RTS line, as esptool's hard reset does.

    DTR stays low so GPIO 0 stays high and the board boots the sketch, not the
    ROM loader. Re-writing DTR after each RTS change is esptool's workaround for
    Windows drivers that only send the line state when DTR is written.

    Thread: main thread.
    """
    port.dtr = False
    port.rts = True
    port.dtr = port.dtr
    sleep(0.1)
    port.rts = False
    port.dtr = port.dtr


def read_boot(port: LinePort, listen_s: float, expect_banner: bool,
              clock: Callable[[], float] = time.monotonic,
              say: Callable[[str], None] = print) -> BootReport:
    """Collect the boot banner, camera lines and status frames. Sends nothing.

    Stops early once the banner is in and the camera has an address (or has
    said why it has none). Without a banner after RST_PROMPT_AFTER_S, the RTS
    pulse did not reach the reset pin: ask for the RST button, once.

    Thread: main thread.
    """
    report = BootReport()
    started = clock()
    while clock() - started < listen_s:
        raw = port.readline()
        if (expect_banner and report.banner is None and not report.prompted
                and clock() - started > RST_PROMPT_AFTER_S):
            say("         no boot line yet: press RST on the ESP32 now "
                "(safe: the laser pin boots LOW)")
            report.prompted = True
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if "OK BOOT" in line:
            report.banner = line
        if "ATTACH FAILED" in line:
            report.attach_failed = True
        if line.startswith("CAM "):
            report.cam_lines.append(line)
        if line.startswith("ST "):
            report.status_frames += 1
            if report.first_uptime_ms is None:
                try:
                    report.first_uptime_ms = int(line.rsplit(",", 1)[1])
                except (IndexError, ValueError):
                    pass
        if not expect_banner and report.status_frames >= 3:
            break
        if report.banner and _cam_settled(report.cam_lines):
            break
    return report


def open_serial(port_name: str, baud: int):
    """Open the port with DTR and RTS low, so opening it is not itself a pulse.

    Thread: main thread.

    Raises:
        OSError, ValueError: from pyserial, if the port will not open.
    """
    import serial

    handle = serial.Serial()
    handle.port = port_name
    handle.baudrate = baud
    handle.timeout = 0.2
    handle.dtr = False
    handle.rts = False
    handle.open()
    return handle


def check_serial(cfg: dict, config_path: str, reset: bool,
                 opener: Callable[[str, int], LinePort] = open_serial,
                 say: Callable[[str], None] = print) -> List[Row]:
    """Serial link, firmware version and the camera address the firmware reports.

    Thread: main thread.
    """
    act, cam = cfg["actuator"], cfg["camera"]
    try:
        port_name = resolve_port(str(act["port"]))
    except ActuatorError as exc:
        return [Row("serial", Status.FAIL, str(exc))]
    try:
        port = opener(port_name, int(act["baud"]))
    except (OSError, ValueError) as exc:
        message = port_open_error(port_name, exc)
        fix = (None if "Another program" in message else
               "python -m tools.serial_probe --list, then set actuator.port to the "
               "CH343's port in the config.")
        return [Row("serial", Status.FAIL, message, fix)]
    try:
        if reset:
            pulse_reset(port)
        report = read_boot(port, BOOT_LISTEN_S if reset else NO_RESET_LISTEN_S,
                           expect_banner=reset, say=say)
    finally:
        close = getattr(port, "close", None)
        if close is not None:
            close()

    rows = []
    if report.status_frames:
        rows.append(Row("serial", Status.PASS,
                        f"{port_name}: {report.status_frames} status frames, laser port released"))
    else:
        rows.append(Row("serial", Status.FAIL, f"{port_name}: no status frames",
                        "Wrong port, or the board is not running the sketch: "
                        "python -m tools.serial_probe --list, then press RST."))
    rows.append(firmware_verdict(report.banner, report.attach_failed,
                                 bool(act.get("pan_reversed", False)), reset))
    if reset:
        rows.append(camera_verdict(report.cam_lines, cam.get("stream_url"),
                                   config_path, report.banner is not None))
    return rows


def check_stream(source: FrameSource, attempts: int = 3, retry_wait_s: float = 3.0,
                 sample_s: float = 2.0, sleep: Callable[[float], None] = time.sleep,
                 clock: Callable[[], float] = time.monotonic) -> Row:
    """Open the stream, retrying while Wi-Fi rejoins after the reset, then time it.

    Leaves the source running on PASS or WARN, for the detection check.

    Thread: main thread.
    """
    error = ""
    for attempt in range(attempts):
        try:
            source.start()
            break
        except RuntimeError as exc:
            error = str(exc)
            if attempt < attempts - 1:
                sleep(retry_wait_s)
    else:
        return Row("stream", Status.FAIL, error.split(". ")[0],
                   "It serves ONE viewer: close any browser tab, vision_probe or "
                   "main.py on it. Still nothing: press RST, wait 10 s, rerun.")
    started, last_id, frames = clock(), -1, 0
    while clock() - started < sample_s:
        _, frame_id = source.read()
        if frame_id != last_id:
            frames += 1
            last_id = frame_id
        sleep(0.005)
    width, height = source.frame_size
    fps = frames / sample_s
    if frames <= 1:
        return Row("stream", Status.WARN, f"{width}x{height} opened but frames stalled",
                   "Weak Wi-Fi: move the laptop and the ESP32 nearer the hotspot.")
    return Row("stream", Status.PASS, f"{width}x{height} at {fps:.1f} fps")


def run_detection(source: FrameSource, detector: DetectsFrames, targets: Sequence[str],
                  conf: float, seconds: float,
                  clock: Callable[[], float] = time.monotonic,
                  sleep: Callable[[float], None] = time.sleep) -> DetectionTally:
    """Run the detector on every new frame for ``seconds``.

    Thread: main thread.
    """
    tally = DetectionTally()
    wanted = set(targets)
    started, last_id = clock(), -1
    while clock() - started < seconds:
        frame, frame_id = source.read()
        if frame is None or frame_id == last_id:
            sleep(0.005)
            continue
        last_id = frame_id
        t0 = clock()
        detections = detector.detect(frame)
        tally.infer_s += clock() - t0
        tally.frames += 1
        hit = False
        for det in detections:
            tally.peaks[det.class_name] = max(tally.peaks.get(det.class_name, 0.0),
                                              det.confidence)
            if det.class_name in wanted and det.confidence >= conf:
                hit = True
        tally.hits += int(hit)
    tally.elapsed_s = clock() - started
    return tally


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def print_row(row: Row) -> None:
    """Print one row, ASCII only: the Windows console may not be UTF-8."""
    print(f"  [{row.status.value}] {row.name:<9} {row.detail}")
    if row.fix and row.status in (Status.FAIL, Status.WARN):
        print(f"         fix: {row.fix}")


def start_order(config_path: str) -> List[str]:
    """The stack's start order for this config, bridge last."""
    return [
        "One PowerShell window each, from the repo folder:",
        f"  B  python main.py --config {config_path}",
        f"  C  python -m tools.operator_console --config {config_path}",
        f"  D  python -m tools.c2_bridge --config {config_path} --threat-start-m 420"
        "   (last: the threat cycle starts when it launches)",
    ]


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run every check in the order the stack needs them. 0 when nothing FAILs."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="config/bench.yaml")
    parser.add_argument("--seconds", type=float, default=6.0,
                        help="How long to run the detector on the live target.")
    parser.add_argument("--no-reset", action="store_true",
                        help="Do not reset the ESP32; skips the version and camera "
                             "address checks.")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="         %(levelname)s %(message)s")

    try:
        with open(args.config, "r", encoding="utf-8") as handle:
            cfg = yaml.safe_load(handle)
    except OSError as exc:
        print(f"Cannot read {args.config}: {exc}. Run from the repo folder.")
        return 2

    det, cam, c2 = cfg["detector"], cfg["camera"], cfg["c2"]
    print(f"Killswitch preflight, {args.config}. Nothing here fires the laser.\n")
    rows: List[Row] = []

    def emit(row: Row) -> None:
        rows.append(row)
        print_row(row)

    emit(check_weights(det["weights"]))
    emit(check_broker(c2["broker_host"], int(c2["broker_port"])))
    if not args.no_reset:
        print("         resetting the ESP32 and listening to it boot (up to "
              f"{BOOT_LISTEN_S:.0f} s)...")
    for row in check_serial(cfg, args.config, reset=not args.no_reset):
        emit(row)

    source: Optional[FrameSource] = None
    if not cam.get("stream_url"):
        emit(Row("stream", Status.SKIP, "camera.stream_url is not set"))
    else:
        from helper.vision.frame_source import HttpStreamSource
        source = HttpStreamSource(
            cam["stream_url"],
            flip_horizontal=bool(cam.get("flip_horizontal", False)),
            flip_vertical=bool(cam.get("flip_vertical", False)),
        )
        emit(check_stream(source))

    try:
        if rows[-1].name != "stream" or rows[-1].status is Status.FAIL or source is None:
            emit(Row("detection", Status.SKIP, "no live stream to run the model on"))
        elif rows[0].status is Status.FAIL:
            emit(Row("detection", Status.SKIP, "no weights"))
        else:
            from helper.vision.detector import DetectorError, UltralyticsDetector
            print(f"         loading {det['weights']} and watching the target for "
                  f"{args.seconds:.0f} s...")
            try:
                # target_classes=None and a low floor: show every class the model
                # emits, so a miss says what the model saw instead.
                detector = UltralyticsDetector(
                    det["weights"], conf_threshold=PROBE_CONF_FLOOR,
                    iou_threshold=det["iou_threshold"], target_classes=None,
                    imgsz=det["imgsz"], device=det.get("device"),
                )
            except DetectorError as exc:
                emit(Row("detection", Status.FAIL, str(exc),
                         "pip install -r requirements.txt, and check detector.weights."))
            else:
                tally = run_detection(source, detector, det["target_classes"],
                                      float(det["conf_threshold"]), args.seconds)
                emit(detection_verdict(tally, det["target_classes"],
                                       float(det["conf_threshold"]), args.config))
    finally:
        if source is not None:
            source.stop()   # the stream serves one viewer: hand it to main.py

    failed = [r for r in rows if r.status is Status.FAIL]
    print()
    if failed:
        print(f"NOT READY: {len(failed)} FAIL ({', '.join(r.name for r in failed)}). "
              f"Fix those, then run the preflight again.")
        return 1
    warned = [r.name for r in rows if r.status is Status.WARN]
    print("READY" + (f", with warnings on {', '.join(warned)}." if warned else "."))
    for line in start_order(args.config):
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
