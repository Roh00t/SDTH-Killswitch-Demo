"""camera_probe's answers to 'is the configured camera reachable?'. No camera needed."""
from __future__ import annotations

import socket
from pathlib import Path

import pytest
import yaml

import tools.camera_probe as camera_probe
from tests.test_stream_source import mjpeg_server  # noqa: F401  (fixture)
from tools.camera_probe import (
    check_configured_index,
    classify_stream_reply,
    find_camera,
    load_configured_index,
    load_stream_url,
    no_camera_hint,
    probe_host,
    subnet_hosts,
    write_stream_url,
)

FALLBACK = Path(__file__).resolve().parents[1] / "config" / "fallback.yaml"


class TestConfiguredIndex:
    def test_found(self):
        found, line = check_configured_index([0, 1], 1, "config/fallback.yaml")
        assert found and line.endswith("camera.device_index=1 -> FOUND")

    def test_not_found_names_the_working_indices(self):
        found, line = check_configured_index([0], 1, "cfg.yaml")
        assert not found
        assert "NOT FOUND" in line and "Working indices: [0]" in line

    def test_no_cameras_at_all(self):
        found, line = check_configured_index([], 1, "cfg.yaml")
        assert not found and "NOT FOUND" in line

    def test_unusable_config(self):
        found, line = check_configured_index([0, 1], None, "cfg.yaml")
        assert not found and "no usable camera.device_index" in line


class TestLoadConfiguredIndex:
    def test_reads_the_index(self, tmp_path):
        cfg = tmp_path / "c.yaml"
        cfg.write_text("camera:\n  device_index: 3\n")
        assert load_configured_index(str(cfg)) == 3

    def test_missing_file_block_or_bad_type_is_none(self, tmp_path):
        assert load_configured_index(str(tmp_path / "nope.yaml")) is None
        (tmp_path / "a.yaml").write_text("detector: {}\n")
        assert load_configured_index(str(tmp_path / "a.yaml")) is None
        (tmp_path / "b.yaml").write_text("camera:\n  device_index: true\n")
        assert load_configured_index(str(tmp_path / "b.yaml")) is None


class TestHints:
    def test_windows_hint_names_the_privacy_switch(self):
        text = "\n".join(no_camera_hint("Windows"))
        assert "Let desktop apps access your camera" in text and "Teams" in text

    def test_macos_and_linux_hints(self):
        assert "Privacy & Security" in "\n".join(no_camera_hint("Darwin"))
        assert "/dev/video" in "\n".join(no_camera_hint("Linux"))


class TestLoadStreamUrl:
    def test_set_unset_and_blank(self, tmp_path):
        (tmp_path / "a.yaml").write_text('camera:\n  stream_url: "http://10.0.0.9/stream"\n')
        (tmp_path / "b.yaml").write_text("camera:\n  stream_url: null\n")
        (tmp_path / "c.yaml").write_text('camera:\n  stream_url: "  "\n')
        assert load_stream_url(str(tmp_path / "a.yaml")) == "http://10.0.0.9/stream"
        assert load_stream_url(str(tmp_path / "b.yaml")) is None
        assert load_stream_url(str(tmp_path / "c.yaml")) is None
        assert load_stream_url(str(tmp_path / "missing.yaml")) is None


class TestFindCamera:
    """--find: locate the firmware's stream without reading the Serial Monitor."""

    def test_classify_names_only_our_firmware(self):
        ours = (b"HTTP/1.1 200 OK\r\nContent-Type: "
                b"multipart/x-mixed-replace;boundary=killswitchframe\r\n\r\n")
        other = (b"HTTP/1.1 200 OK\r\nContent-Type: "
                 b"multipart/x-mixed-replace;boundary=123456789000000000000987654321\r\n\r\n")
        assert classify_stream_reply(ours) == "killswitch"
        assert classify_stream_reply(other) == "mjpeg"
        assert classify_stream_reply(b"HTTP/1.1 404 Not Found\r\n\r\n") is None
        assert classify_stream_reply(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n") is None

    def test_probe_recognises_the_firmware_stream_format(self, mjpeg_server):
        url, _ = mjpeg_server
        port = int(url.split(":")[2].split("/")[0])
        assert probe_host("127.0.0.1", port) == "killswitch"

    def test_probe_busy_then_nothing(self):
        # A listener that never answers is how the firmware looks while a
        # browser tab holds its one stream slot.
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            port = listener.getsockname()[1]
            assert probe_host("127.0.0.1", port, reply_timeout=0.3) == "busy"
        assert probe_host("127.0.0.1", port) is None

    def test_subnet_hosts_skip_this_laptop(self):
        hosts = subnet_hosts("192.168.43.12")
        assert len(hosts) == 253 and "192.168.43.12" not in hosts
        assert hosts[0] == "192.168.43.1" and hosts[-1] == "192.168.43.254"

    def test_find_picks_the_firmware_and_lists_the_rest(self):
        kinds = {"10.0.0.2": "busy", "10.0.0.5": "killswitch", "10.0.0.7": "mjpeg"}
        host, others = find_camera(list(kinds) + ["10.0.0.9"], probe=kinds.get)
        assert host == "10.0.0.5"
        assert others == ["10.0.0.2 (busy)", "10.0.0.7 (mjpeg)"]

    def test_write_keeps_every_comment_and_every_other_value(self, tmp_path):
        original = FALLBACK.read_text(encoding="utf-8")
        cfg = tmp_path / "fallback.yaml"
        cfg.write_text(original, encoding="utf-8")
        assert write_stream_url(str(cfg), "http://192.168.43.57/stream")
        before, after = yaml.safe_load(original), yaml.safe_load(cfg.read_text())
        assert after["camera"]["stream_url"] == "http://192.168.43.57/stream"
        before["camera"]["stream_url"] = after["camera"]["stream_url"]
        assert after == before
        assert cfg.read_text().count("#") == original.count("#")
        assert write_stream_url(str(cfg), "http://10.0.0.9/stream")   # a new hotspot IP
        assert load_stream_url(str(cfg)) == "http://10.0.0.9/stream"

    def test_write_refuses_without_exactly_one_stream_url_line(self, tmp_path):
        cfg = tmp_path / "c.yaml"
        cfg.write_text("camera:\n  device_index: 0\n")
        assert not write_stream_url(str(cfg), "http://10.0.0.9/stream")
        assert cfg.read_text() == "camera:\n  device_index: 0\n"

    def test_run_find_scans_when_the_name_fails_and_writes(self, tmp_path, monkeypatch, capsys):
        cfg = tmp_path / "c.yaml"
        cfg.write_text("camera:\n  stream_url: null  # set by --find\n")
        scanned = []
        monkeypatch.setattr(camera_probe, "resolve_with_timeout", lambda name: None)
        monkeypatch.setattr(camera_probe, "local_ipv4s", lambda: ["192.168.43.12"])
        monkeypatch.setattr(camera_probe, "find_camera",
                            lambda hosts: (scanned.extend(hosts), ("192.168.43.57", []))[1])
        assert camera_probe.run_find(str(cfg), write=True) == 0
        assert len(scanned) == 253
        assert load_stream_url(str(cfg)) == "http://192.168.43.57/stream"
        assert "# set by --find" in cfg.read_text()
        assert "FOUND http://192.168.43.57/stream" in capsys.readouterr().out

    def test_run_find_not_found_prints_why(self, tmp_path, monkeypatch, capsys):
        monkeypatch.setattr(camera_probe, "resolve_with_timeout", lambda name: None)
        monkeypatch.setattr(camera_probe, "local_ipv4s", lambda: ["172.20.10.2"])
        monkeypatch.setattr(camera_probe, "find_camera",
                            lambda hosts: (None, ["172.20.10.1 (busy)"]))
        assert camera_probe.run_find(str(tmp_path / "c.yaml"), write=True) == 1
        out = capsys.readouterr().out
        assert "NOT FOUND" in out and "172.20.10.1 (busy)" in out and "2.4 GHz" in out

    def test_run_find_confirms_a_bare_configured_address_without_scanning(
            self, tmp_path, monkeypatch):
        # The rig's config held "10.244.153.41": no scheme, no path.
        cfg = tmp_path / "c.yaml"
        cfg.write_text('camera:\n  stream_url: "10.244.153.41"\n')
        probed = []
        monkeypatch.setattr(camera_probe, "probe_host",
                            lambda host: probed.append(host) or "killswitch")
        monkeypatch.setattr(camera_probe, "find_camera",
                            lambda hosts: pytest.fail("scanned although the config was right"))
        assert camera_probe.run_find(str(cfg), write=True) == 0
        assert probed == ["10.244.153.41"]
        assert load_stream_url(str(cfg)) == "http://10.244.153.41/stream"
