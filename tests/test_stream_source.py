"""HttpStreamSource: the ESP32-S3 Wi-Fi camera, newest frame wins.

The fakes pin down reconnect and health behaviour. The last class serves the
firmware's exact multipart format from a local HTTP server and decodes it with
OpenCV's FFmpeg backend, so "OpenCV can read our stream" is tested, not assumed.
"""
from __future__ import annotations

import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pytest

from helper.vision.frame_source import HttpStreamSource, normalize_stream_url

BOUNDARY = "killswitchframe"   # firmware/esp32_actuator STREAM_BOUNDARY


def marked_frame(value: int, width: int = 64, height: int = 48) -> np.ndarray:
    """A frame whose left column is bright and right column dark."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :4] = 255
    frame[0, 0] = value % 256
    return frame


class FakeCapture:
    """Yields `frames` good reads, then fails every read."""

    def __init__(self, frames: int, opened: bool = True) -> None:
        self._left = frames
        self._opened = opened
        self.released = False

    def isOpened(self) -> bool:
        return self._opened

    def read(self):
        if self._left <= 0:
            time.sleep(0.01)
            return False, None
        self._left -= 1
        return True, marked_frame(self._left)

    def release(self) -> None:
        self.released = True


def wait_for(predicate, timeout: float = 3.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class TestWithFakes:
    def test_start_learns_the_size_and_frames_keep_arriving(self):
        src = HttpStreamSource("http://cam/stream",
                               capture_factory=lambda url: FakeCapture(10_000))
        src.start()
        try:
            assert src.frame_size == (64, 48)
            _, first = src.read()
            assert wait_for(lambda: src.read()[1] > first + 5)
            assert src.is_healthy
        finally:
            src.stop()

    def test_no_stream_fails_start_with_a_useful_message(self):
        src = HttpStreamSource("http://cam/stream",
                               capture_factory=lambda url: FakeCapture(0, opened=False))
        with pytest.raises(RuntimeError, match="Open it in a browser"):
            src.start()

    def test_a_drop_reconnects_and_stays_healthy(self):
        opens = []

        def factory(url):
            opens.append(url)
            # First capture dies after 3 frames; the reconnect gets a good one.
            return FakeCapture(3 if len(opens) == 1 else 10_000)

        src = HttpStreamSource("http://cam/stream", reconnect_window_s=2.0,
                               capture_factory=factory)
        src.start()
        try:
            assert wait_for(lambda: len(opens) >= 2)
            _, after_reconnect = src.read()
            assert wait_for(lambda: src.read()[1] > after_reconnect)
            assert src.is_healthy
        finally:
            src.stop()

    def test_a_dead_stream_goes_unhealthy_after_the_window(self):
        opens = []

        def factory(url):
            # One frame to start on, then every reconnect finds nothing.
            opens.append(url)
            return FakeCapture(1 if len(opens) == 1 else 0)

        src = HttpStreamSource("http://cam/stream", reconnect_window_s=0.5,
                               capture_factory=factory)
        src.start()
        try:
            assert wait_for(lambda: not src.is_healthy, timeout=3.0), \
                "a dead camera must end in unhealthy, so the node drops to IDLE"
        finally:
            src.stop()

    def test_an_exception_on_reconnect_still_ends_unhealthy(self):
        opens = []

        def factory(url):
            opens.append(url)
            if len(opens) == 1:
                return FakeCapture(1)
            raise cv2.error("network stack said no")

        src = HttpStreamSource("http://cam/stream", reconnect_window_s=0.5,
                               capture_factory=factory)
        src.start()
        try:
            assert wait_for(lambda: not src.is_healthy, timeout=3.0), \
                "a raising reopen must not leave a dead reader reporting healthy"
        finally:
            src.stop()

    def test_a_stream_that_comes_back_is_healthy_again(self):
        # The rig on venue Wi-Fi: a stall long enough to declare the camera
        # lost, then the ESP32 answers again. It used to stay dead until the
        # node was restarted.
        back = threading.Event()
        opens = []

        def factory(url):
            opens.append(url)
            if len(opens) == 1:
                return FakeCapture(1)
            return FakeCapture(10_000) if back.is_set() else FakeCapture(0)

        src = HttpStreamSource("http://cam/stream", reconnect_window_s=0.5,
                               capture_factory=factory)
        src.start()
        try:
            assert wait_for(lambda: not src.is_healthy, timeout=3.0)
            _, during_outage = src.read()
            back.set()
            assert wait_for(lambda: src.is_healthy, timeout=5.0), \
                "frames returned, so the source must report healthy again"
            assert wait_for(lambda: src.read()[1] > during_outage + 5)
        finally:
            src.stop()

    def test_stop_is_prompt_while_reconnecting(self):
        opens = []

        def factory(url):
            opens.append(url)
            return FakeCapture(1 if len(opens) == 1 else 0)

        src = HttpStreamSource("http://cam/stream", reconnect_window_s=0.3,
                               capture_factory=factory)
        src.start()
        assert wait_for(lambda: not src.is_healthy, timeout=3.0)
        started = time.monotonic()
        src.stop()
        assert time.monotonic() - started < 2.0
        assert not src.is_healthy

    @pytest.mark.parametrize("h, v, bright_col", [(False, False, 0), (True, False, 63)])
    def test_flip_mirrors_the_frame(self, h, v, bright_col):
        src = HttpStreamSource("http://cam/stream", flip_horizontal=h, flip_vertical=v,
                               capture_factory=lambda url: FakeCapture(10_000))
        src.start()
        try:
            frame, _ = src.read()
            assert frame[10, bright_col].tolist() == [255, 255, 255]
        finally:
            src.stop()


class TestNormalizeStreamUrl:
    """A hand-typed address must not reach FFmpeg as a file path."""

    @pytest.mark.parametrize("typed, opened", [
        ("10.244.153.41", "http://10.244.153.41/stream"),
        (" 10.244.153.41 ", "http://10.244.153.41/stream"),
        ("http://10.244.153.41", "http://10.244.153.41/stream"),
        ("http://10.244.153.41/", "http://10.244.153.41/stream"),
        ("10.244.153.41/stream", "http://10.244.153.41/stream"),
        ("killswitch-cam.local", "http://killswitch-cam.local/stream"),
        ("http://10.244.153.41/stream", "http://10.244.153.41/stream"),
        ("http://10.244.153.41:81/stream", "http://10.244.153.41:81/stream"),
        ("rtsp://10.0.0.5/live", "rtsp://10.0.0.5/live"),
    ])
    def test_completes_only_what_is_missing(self, typed, opened):
        assert normalize_stream_url(typed) == opened

    def test_the_source_opens_the_completed_url(self):
        opened = []

        def factory(url):
            opened.append(url)
            return FakeCapture(10_000)

        src = HttpStreamSource("10.244.153.41", capture_factory=factory)
        src.start()
        try:
            assert opened[0] == "http://10.244.153.41/stream"
        finally:
            src.stop()


class _MjpegHandler(BaseHTTPRequestHandler):
    """Serves /stream exactly as streamHandler() in the firmware does."""

    fps = 20.0

    def log_message(self, *args) -> None:
        pass

    def do_GET(self) -> None:
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace;boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        n = 0
        try:
            while not self.server.stopping.is_set():
                ok, jpg = cv2.imencode(".jpg", marked_frame(n, 640, 480))
                body = jpg.tobytes()
                self.wfile.write(
                    f"\r\n--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                    f"Content-Length: {len(body)}\r\n\r\n".encode() + body)
                self.wfile.flush()
                n += 1
                time.sleep(1.0 / self.fps)
        except (BrokenPipeError, ConnectionResetError):
            pass


@pytest.fixture
def mjpeg_server():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = ThreadingHTTPServer(("127.0.0.1", port), _MjpegHandler)
    server.daemon_threads = True
    server.stopping = threading.Event()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{port}/stream", server
    server.stopping.set()
    server.shutdown()
    server.server_close()


@pytest.mark.skipif("FFMPEG:                      YES" not in cv2.getBuildInformation(),
                    reason="this OpenCV build has no FFmpeg backend")
class TestRealMjpegOverHttp:
    def test_opencv_decodes_the_firmware_stream_format(self, mjpeg_server):
        url, _ = mjpeg_server
        src = HttpStreamSource(url)
        src.start()
        try:
            assert src.frame_size == (640, 480)
            _, first = src.read()
            assert wait_for(lambda: src.read()[1] >= first + 10, timeout=5.0), \
                "frames must keep arriving at the stream's rate"
            frame, _ = src.read()
            assert frame.shape == (480, 640, 3)
            assert frame[100, 1].mean() > 200 and frame[100, 600].mean() < 50
        finally:
            src.stop()

    def test_server_gone_means_unhealthy(self, mjpeg_server):
        url, server = mjpeg_server
        src = HttpStreamSource(url, read_timeout_s=0.5, open_timeout_s=0.5,
                               reconnect_window_s=1.0)
        src.start()
        try:
            server.stopping.set()
            server.shutdown()
            server.server_close()
            assert wait_for(lambda: not src.is_healthy, timeout=8.0)
        finally:
            src.stop()

    def test_server_back_on_the_same_port_means_healthy_again(self, mjpeg_server):
        url, server = mjpeg_server
        port = server.server_address[1]
        src = HttpStreamSource(url, read_timeout_s=0.5, open_timeout_s=0.5,
                               reconnect_window_s=1.0)
        src.start()
        replacement = None
        try:
            server.stopping.set()
            server.shutdown()
            server.server_close()
            assert wait_for(lambda: not src.is_healthy, timeout=8.0)
            replacement = ThreadingHTTPServer(("127.0.0.1", port), _MjpegHandler)
            replacement.daemon_threads = True
            replacement.stopping = threading.Event()
            threading.Thread(target=replacement.serve_forever, daemon=True).start()
            assert wait_for(lambda: src.is_healthy, timeout=10.0), \
                "the ESP32 answering again must bring the camera back"
        finally:
            src.stop()
            if replacement is not None:
                replacement.stopping.set()
                replacement.shutdown()
                replacement.server_close()

    def test_camera_probe_reports_found_with_size_and_rate(self, mjpeg_server):
        from tools.camera_probe import probe_stream

        url, _ = mjpeg_server
        found, line = probe_stream(url, seconds=1.0)
        assert found and line.startswith(f"camera.stream_url={url} -> FOUND (640x480, ")

    def test_camera_probe_reports_not_found(self):
        from tools.camera_probe import probe_stream

        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]          # nothing listens here
        found, line = probe_stream(f"http://127.0.0.1:{port}/stream", seconds=0.5)
        assert not found and "NOT FOUND" in line
