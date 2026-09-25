"""camera_probe's answers to 'is the configured camera reachable?'. No camera needed."""
from __future__ import annotations

from tools.camera_probe import (
    check_configured_index,
    load_configured_index,
    load_stream_url,
    no_camera_hint,
)


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
