"""Preflight verdicts, reset pulse, boot reader and detection loop. No hardware."""
from __future__ import annotations

import socket
from typing import List, Optional, Sequence, Tuple

import numpy as np
import pytest

import tools.preflight as preflight
from helper.vision.types import Detection
from tools.preflight import (
    BootReport,
    DetectionTally,
    Status,
    camera_verdict,
    check_broker,
    check_serial,
    check_stream,
    check_weights,
    detection_verdict,
    firmware_verdict,
    parse_boot_version,
    parse_cam_url,
    pulse_reset,
    read_boot,
    run_detection,
)

V32 = "OK BOOT killswitch-actuator v3.2 pan_ch=1 tilt_ch=2 pwm=ok"
V33 = "OK BOOT killswitch-actuator v3.3 pan_ch=1 tilt_ch=2 pwm=ok"
URL = "http://10.244.153.41/stream"


class FakeClock:
    """Advances by `step` on every call, so loops bounded by time terminate."""

    def __init__(self, step: float = 0.1) -> None:
        self.now = 0.0
        self.step = step

    def __call__(self) -> float:
        self.now += self.step
        return self.now


class FakePort:
    """Scripted serial lines; records every DTR/RTS write in order."""

    def __init__(self, lines: Sequence[str] = ()) -> None:
        self._lines = [line.encode("ascii") + b"\r\n" for line in lines]
        self.writes: List[Tuple[str, bool]] = []
        self._dtr = True
        self._rts = True
        self.closed = False
        self.sent: List[bytes] = []

    @property
    def dtr(self) -> bool:
        return self._dtr

    @dtr.setter
    def dtr(self, value: bool) -> None:
        self._dtr = value
        self.writes.append(("dtr", value))

    @property
    def rts(self) -> bool:
        return self._rts

    @rts.setter
    def rts(self, value: bool) -> None:
        self._rts = value
        self.writes.append(("rts", value))

    def readline(self) -> bytes:
        return self._lines.pop(0) if self._lines else b""

    def write(self, data: bytes) -> int:  # must never be called
        self.sent.append(data)
        return len(data)

    def close(self) -> None:
        self.closed = True


def det(name: str, conf: float) -> Detection:
    return Detection(x=320, y=240, w=80, h=60, confidence=conf, class_id=0, class_name=name)


class TestVersionParsing:
    def test_minor_version(self):
        assert parse_boot_version(V32) == (3, 2)

    def test_bare_major_is_minor_zero(self):
        assert parse_boot_version("OK BOOT killswitch-actuator v3 pan_ch=1") == (3, 0)

    def test_garbage(self):
        assert parse_boot_version("OK BOOT something else") is None


class TestFirmwareVerdict:
    def test_v32_passes(self):
        assert firmware_verdict(V32, False, True, True).status is Status.PASS

    def test_v33_with_laptop_mirror_is_a_double_reversal(self):
        row = firmware_verdict(V33, False, pan_reversed=True, reset_requested=True)
        assert row.status is Status.FAIL
        assert "twice" in row.detail and "v3.2" in row.fix

    def test_v33_without_laptop_mirror_is_consistent(self):
        assert firmware_verdict(V33, False, False, True).status is Status.PASS

    def test_v2_is_the_old_pin_map(self):
        row = firmware_verdict("OK BOOT killswitch-actuator v2 pan_ch=1 tilt_ch=2 pwm=ok",
                               False, True, True)
        assert row.status is Status.FAIL and "5/6/7" in row.detail

    def test_attach_failure(self):
        banner = ("OK BOOT killswitch-actuator v3.2 pan_ch=0 tilt_ch=2 "
                  "SERVO ATTACH FAILED - NO PWM WILL BE GENERATED")
        assert firmware_verdict(banner, True, True, True).status is Status.FAIL

    def test_pre_stall_recovery_warns(self):
        row = firmware_verdict("OK BOOT killswitch-actuator v3.1 pan_ch=1 tilt_ch=2 pwm=ok",
                               False, True, True)
        assert row.status is Status.WARN

    def test_no_banner_after_reset_warns(self):
        assert firmware_verdict(None, False, True, True).status is Status.WARN

    def test_no_reset_says_unchecked(self):
        row = firmware_verdict(None, False, True, reset_requested=False)
        assert row.status is Status.WARN and "--no-reset" in row.detail


class TestCameraVerdict:
    def test_matching_address(self):
        row = camera_verdict([f"CAM {URL}"], URL, "config/bench.yaml", True)
        assert row.status is Status.PASS

    def test_bare_configured_address_still_matches(self):
        row = camera_verdict([f"CAM {URL}"], "10.244.153.41", "config/bench.yaml", True)
        assert row.status is Status.PASS

    def test_moved_address_names_the_fix(self):
        row = camera_verdict(["CAM http://10.244.153.99/stream"], URL,
                             "config/bench.yaml", True)
        assert row.status is Status.FAIL
        assert "--find --write" in row.fix and "config/bench.yaml" in row.fix

    def test_camera_fail(self):
        row = camera_verdict(["CAM FAIL camera init error 0x105 - check the ribbon"],
                             URL, "c.yaml", True)
        assert row.status is Status.FAIL and "ribbon" in row.fix

    def test_wifi_not_joined_carries_the_reason(self):
        line = "CAM wifi not joined: wrong password (reason 15)"
        row = camera_verdict([line], URL, "c.yaml", True)
        assert row.status is Status.FAIL and row.detail == line

    def test_no_address_yet(self):
        row = camera_verdict(["CAM camera ok, sensor 0x5640, 640x480"], URL, "c.yaml", True)
        assert row.status is Status.WARN

    def test_parse_cam_url(self):
        assert parse_cam_url(f"CAM {URL}") == URL
        assert parse_cam_url("CAM http://1.2.3.4/stream (camera not ready)") == \
            "http://1.2.3.4/stream"
        assert parse_cam_url("ST 90.0,90.0,0,0,1234") is None


class TestPulseReset:
    def test_rts_pulses_with_dtr_held_low(self):
        port = FakePort()
        sleeps: List[float] = []
        pulse_reset(port, sleep=sleeps.append)
        rts = [value for line, value in port.writes if line == "rts"]
        assert rts == [True, False], "one reset pulse: EN low, then released"
        assert all(value is False for line, value in port.writes if line == "dtr"), \
            "DTR high would pull GPIO 0 low and boot the ROM loader, not the sketch"
        assert sleeps == [0.1]


class TestReadBoot:
    def test_collects_banner_camera_and_status(self):
        port = FakePort(["garbage", V32, "ST 90.0,90.0,0,0,812",
                         'CAM wifi "hotspot" heard on channel 6 at -48 dBm, joining',
                         f"CAM {URL}", "ST 90.0,90.0,0,0,912"])
        report = read_boot(port, listen_s=30.0, expect_banner=True,
                           clock=FakeClock(), say=lambda _: None)
        assert report.banner == V32
        assert report.first_uptime_ms == 812
        assert report.cam_lines[-1] == f"CAM {URL}"
        assert not report.prompted
        assert port.sent == [], "the preflight never sends the firmware a command"

    def test_no_banner_asks_for_rst_once(self):
        said: List[str] = []
        report = read_boot(FakePort(["ST 90.0,90.0,0,0,999999"]), listen_s=20.0,
                           expect_banner=True, clock=FakeClock(0.5), say=said.append)
        assert report.banner is None and report.prompted
        assert len(said) == 1 and "press RST" in said[0]

    def test_no_reset_mode_stops_after_status_frames(self):
        port = FakePort(["ST 90.0,90.0,0,0,1"] * 5)
        report = read_boot(port, listen_s=10.0, expect_banner=False, clock=FakeClock())
        assert report.status_frames == 3 and not report.prompted


class TestCheckSerial:
    CFG = {"actuator": {"port": "COM3", "baud": 921600, "pan_reversed": True},
           "camera": {"stream_url": URL}}

    def test_healthy_board(self, monkeypatch):
        monkeypatch.setattr(preflight, "BOOT_LISTEN_S", 5.0)
        port = FakePort([V32, "ST 90.0,90.0,0,0,812", f"CAM {URL}"])
        rows = check_serial(self.CFG, "c.yaml", reset=True,
                            opener=lambda name, baud: port, say=lambda _: None)
        assert [r.status for r in rows] == [Status.PASS] * 3
        assert port.closed, "COM3 must be released for main.py"
        assert port.sent == []

    def test_held_port_uses_the_held_port_hint(self):
        def opener(name: str, baud: int):
            raise PermissionError("could not open port 'COM3': Access is denied.")
        rows = check_serial(self.CFG, "c.yaml", reset=True, opener=opener)
        assert len(rows) == 1 and rows[0].status is Status.FAIL
        assert "Another program has COM3 open" in rows[0].detail


class TestWeightsAndBroker:
    def test_bare_weights_name_warns_about_offline(self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)
        row = check_weights("yolo11s.pt")
        assert row.status is Status.FAIL and "offline" in row.fix

    def test_weights_present(self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)
        (tmp_path / "yolo11s.pt").write_bytes(b"x")
        assert check_weights("yolo11s.pt").status is Status.PASS

    def test_broker_listening(self):
        with socket.socket() as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            assert check_broker("127.0.0.1", server.getsockname()[1]).status is Status.PASS

    def test_broker_down(self):
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        row = check_broker("127.0.0.1", port, timeout_s=0.5)
        assert row.status is Status.FAIL and "mosquitto" in row.fix


class FakeSource:
    """A FrameSource stand-in: new frame id on every read, or failing starts."""

    def __init__(self, fail_starts: int = 0) -> None:
        self.fail_starts = fail_starts
        self.frame_id = 0
        self.started = False

    def start(self) -> None:
        if self.fail_starts:
            self.fail_starts -= 1
            raise RuntimeError("No frames from camera stream x. It serves ONE viewer: ...")
        self.started = True

    def read(self) -> Tuple[Optional[np.ndarray], int]:
        self.frame_id += 1
        return np.zeros((480, 640, 3), np.uint8), self.frame_id

    def stop(self) -> None:
        self.started = False

    @property
    def frame_size(self) -> Tuple[int, int]:
        return (640, 480)


class TestStream:
    def test_retries_while_wifi_rejoins(self):
        source = FakeSource(fail_starts=2)
        row = check_stream(source, sleep=lambda _: None, clock=FakeClock())
        assert row.status is Status.PASS and "640x480" in row.detail

    def test_gives_up_with_the_one_viewer_fix(self):
        row = check_stream(FakeSource(fail_starts=9), sleep=lambda _: None,
                           clock=FakeClock())
        assert row.status is Status.FAIL and "ONE viewer" in row.fix


class ScriptedDetector:
    def __init__(self, script: Sequence[List[Detection]]) -> None:
        self._script = list(script)

    def detect(self, frame: np.ndarray) -> List[Detection]:
        return self._script.pop(0) if self._script else []


class TestDetection:
    def test_counts_hits_at_the_node_threshold(self):
        script = [[det("drone", 0.7)], [det("drone", 0.3)], [det("bird", 0.9)],
                  [det("drone", 0.5)]]
        tally = run_detection(FakeSource(), ScriptedDetector(script), ["drone"], 0.4,
                              seconds=5.0, clock=FakeClock(0.1), sleep=lambda _: None)
        assert tally.frames >= 4
        assert tally.hits == 2
        assert tally.peaks["drone"] == 0.7 and tally.peaks["bird"] == 0.9

    def test_steady_lock_passes(self):
        tally = DetectionTally(frames=20, hits=19, peaks={"drone": 0.8},
                               infer_s=4.0, elapsed_s=6.0)
        assert detection_verdict(tally, ["drone"], 0.4, "c.yaml").status is Status.PASS

    def test_flicker_warns(self):
        tally = DetectionTally(frames=20, hits=10, peaks={"drone": 0.6},
                               infer_s=4.0, elapsed_s=6.0)
        assert detection_verdict(tally, ["drone"], 0.4, "c.yaml").status is Status.WARN

    def test_miss_says_what_the_model_saw_and_names_the_other_config(self):
        tally = DetectionTally(frames=20, hits=0, peaks={"airplane": 0.55, "drone": 0.2},
                               infer_s=4.0, elapsed_s=6.0)
        row = detection_verdict(tally, ["drone"], 0.4, "config/bench.yaml")
        assert row.status is Status.FAIL
        assert "airplane 0.55" in row.fix and "drone 0.20" in row.fix
        assert "config/fallback.yaml" in row.fix

    def test_coco_miss_points_at_the_drone_model(self):
        tally = DetectionTally(frames=10, hits=0, peaks={}, infer_s=1.0, elapsed_s=6.0)
        row = detection_verdict(tally, ["airplane", "bird"], 0.4, "config/fallback.yaml")
        assert "nothing at all" in row.fix and "config/bench.yaml" in row.fix

    def test_no_frames(self):
        row = detection_verdict(DetectionTally(), ["drone"], 0.4, "c.yaml")
        assert row.status is Status.FAIL


def test_no_hardware_run_fails_cleanly(tmp_path, monkeypatch, capsys):
    """The whole tool on a box with no broker, no board and no camera."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        dead_port = s.getsockname()[1]
    cfg = tmp_path / "c.yaml"
    cfg.write_text(
        "detector: {weights: missing.onnx, conf_threshold: 0.4, iou_threshold: 0.5,"
        " imgsz: 640, target_classes: [drone], device: null}\n"
        "camera: {stream_url: null}\n"
        f"c2: {{broker_host: 127.0.0.1, broker_port: {dead_port}}}\n"
        "actuator: {port: /dev/does-not-exist, baud: 921600, pan_reversed: true}\n",
        encoding="utf-8",
    )
    assert preflight.main(["--config", str(cfg), "--no-reset"]) == 1
    out = capsys.readouterr().out
    assert "NOT READY" in out
    assert "[FAIL] weights" in out and "[FAIL] broker" in out and "[FAIL] serial" in out
    assert "[SKIP] detection" in out
    assert "Traceback" not in out
