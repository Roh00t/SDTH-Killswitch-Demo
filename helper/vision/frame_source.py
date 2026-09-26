"""Frame acquisition with stale-frame rejection.

Zero-latency capture is a *threading* problem, not a property setting.
``cv2.CAP_PROP_BUFFERSIZE`` is implemented by the V4L2 and DSHOW backends and
**ignored by AVFoundation on macOS**. We set it as a best-effort hint and report
whether it took, but the load-bearing mechanism is a daemon thread that reads
continuously and keeps only the most recent frame. Consumers therefore always
get the freshest frame the driver has produced, on every platform.
"""
from __future__ import annotations

import logging
import platform
import threading
import time
import urllib.parse
from abc import ABC, abstractmethod
from typing import Callable, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class FrameSource(ABC):
    """Contract for anything that produces frames.

    Two live implementations exist (UsbCameraSource, MockFrameSource) because an
    abstraction with one implementation is an unverified claim, and hardware
    agnosticism is the project thesis. See CLAUDE.md rule 4.
    """

    @abstractmethod
    def start(self) -> None:
        """Begin acquisition. Idempotent."""

    @abstractmethod
    def read(self) -> Tuple[Optional[np.ndarray], int]:
        """Return the most recent frame and its monotonic frame id.

        Returns:
            ``(frame, frame_id)``. Frame is None if no frame is available yet.
            The id increments per captured frame, letting a consumer detect
            whether it is re-processing a frame it has already seen.
        """

    @abstractmethod
    def stop(self) -> None:
        """Release resources. Idempotent and safe to call from any thread."""

    @property
    @abstractmethod
    def frame_size(self) -> Tuple[int, int]:
        """``(width, height)`` in pixels."""

    @property
    def frame_center(self) -> Tuple[float, float]:
        """``(cx, cy)`` — the boresight, and the origin of pointing error."""
        w, h = self.frame_size
        return (w / 2.0, h / 2.0)


class UsbCameraSource(FrameSource):
    """USB UVC camera via cv2.VideoCapture with a stale-frame-dropping reader.

    Thread-safety: ``read()`` is safe from any thread. ``start()``/``stop()``
    should be called from the owning thread only.
    """

    def __init__(
        self,
        device_index: int = 0,
        width: int = 640,
        height: int = 480,
        target_fps: int = 30,
        use_mjpg: bool = True,
    ) -> None:
        """Configure the camera.

        Args:
            device_index: OpenCV device index.
            width: Requested capture width.
            height: Requested capture height.
            target_fps: Requested frame rate.
            use_mjpg: Request MJPG. UVC cameras default to YUYV, which saturates
                USB 2.0 bandwidth and collapses to ~5 fps at higher resolutions.
                Leave this on unless you have a specific reason.
        """
        self._device_index = device_index
        self._requested_size = (width, height)
        self._target_fps = target_fps
        self._use_mjpg = use_mjpg

        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._running = threading.Event()

        self._latest: Optional[np.ndarray] = None
        self._frame_id: int = 0
        self._actual_size: Tuple[int, int] = (width, height)
        self._buffersize_honoured: bool = False
        self._consecutive_failures: int = 0

    @staticmethod
    def _preferred_backend() -> int:
        """Pick the platform's native capture backend explicitly.

        Letting OpenCV auto-select can land on a slow fallback (e.g. GStreamer)
        that silently ignores format requests.
        """
        system = platform.system()
        if system == "Darwin":
            return cv2.CAP_AVFOUNDATION
        if system == "Linux":
            return cv2.CAP_V4L2
        if system == "Windows":
            return cv2.CAP_DSHOW
        return cv2.CAP_ANY

    def start(self) -> None:
        """Open the device and start the reader thread.

        Raises:
            RuntimeError: If the device cannot be opened or produces no frames.
        """
        if self._running.is_set():
            return

        cap = cv2.VideoCapture(self._device_index, self._preferred_backend())
        if not cap.isOpened():
            raise RuntimeError(
                f"Could not open camera index {self._device_index}. "
                f"Run 'python -m tools.camera_probe' to enumerate devices."
            )

        # Order matters: FOURCC before resolution, or some UVC drivers ignore it.
        if self._use_mjpg:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._requested_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._requested_size[1])
        cap.set(cv2.CAP_PROP_FPS, self._target_fps)

        # Best-effort only. Ignored on AVFoundation; we do not depend on it.
        self._buffersize_honoured = bool(cap.set(cv2.CAP_PROP_BUFFERSIZE, 1))

        ok, probe = cap.read()
        if not ok or probe is None:
            cap.release()
            raise RuntimeError(
                f"Camera {self._device_index} opened but produced no frame. "
                f"Another process may hold the device."
            )

        self._actual_size = (probe.shape[1], probe.shape[0])
        if self._actual_size != self._requested_size:
            logger.warning(
                "Camera negotiated %dx%d, not the requested %dx%d",
                self._actual_size[0], self._actual_size[1],
                self._requested_size[0], self._requested_size[1],
            )
        logger.info(
            "Camera %d open at %dx%d, CAP_PROP_BUFFERSIZE %s",
            self._device_index, self._actual_size[0], self._actual_size[1],
            "honoured" if self._buffersize_honoured else "IGNORED (expected on macOS)",
        )

        self._cap = cap
        with self._lock:
            self._latest = probe
            self._frame_id = 1

        self._running.set()
        self._thread = threading.Thread(
            target=self._reader_loop, name="frame-grabber", daemon=True
        )
        self._thread.start()

    def _reader_loop(self) -> None:
        """Continuously read, keeping only the newest frame.

        Runs on the grabber thread. Overwriting rather than queueing is the
        point: a queued frame is a stale frame, and stale frames drive the
        gimbal to where the target used to be.
        """
        while self._running.is_set():
            cap = self._cap
            if cap is None:
                break

            ok, frame = cap.read()
            if not ok or frame is None:
                self._consecutive_failures += 1
                if self._consecutive_failures >= 30:
                    logger.error(
                        "Camera produced %d consecutive failures; treating as "
                        "disconnected", self._consecutive_failures,
                    )
                    self._running.clear()
                    break
                time.sleep(0.01)
                continue

            self._consecutive_failures = 0
            with self._lock:
                self._latest = frame
                self._frame_id += 1

    def read(self) -> Tuple[Optional[np.ndarray], int]:
        """Return the newest frame and its id. Never blocks on the device."""
        with self._lock:
            if self._latest is None:
                return (None, self._frame_id)
            return (self._latest, self._frame_id)

    def stop(self) -> None:
        """Stop the reader thread and release the device."""
        self._running.clear()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)
            if thread.is_alive():
                logger.warning("Grabber thread did not exit within 2s")
        self._thread = None

        if self._cap is not None:
            self._cap.release()
            self._cap = None
        logger.info("Camera %d released", self._device_index)

    @property
    def frame_size(self) -> Tuple[int, int]:
        return self._actual_size

    @property
    def is_healthy(self) -> bool:
        """False once the device has been declared disconnected."""
        return self._running.is_set()


def normalize_stream_url(value: str) -> str:
    """Complete a hand-typed camera address into the firmware's stream URL.

    The firmware serves one thing, ``http://<ip>/stream``. A bare
    ``10.244.153.41`` in the config otherwise reaches FFmpeg as a file path and
    fails as "no frames", which reads like a dead camera. Adds a missing
    ``http://`` and a missing path; leaves a URL with its own path, port or
    scheme alone.

    Args:
        value: What the config holds.

    Returns:
        The URL to open.
    """
    text = value.strip()
    if "://" not in text:
        text = "http://" + text
    parts = urllib.parse.urlsplit(text)
    if parts.scheme in ("http", "https") and parts.path in ("", "/"):
        text = urllib.parse.urlunsplit(parts._replace(path="/stream"))
    return text


def _open_http_capture(url: str, open_timeout_s: float, read_timeout_s: float) -> cv2.VideoCapture:
    """Open an MJPEG-over-HTTP stream with FFmpeg, with bounded open and read waits.

    Without the timeouts FFmpeg waits about 30 s on a dead link, so a Wi-Fi drop
    would freeze the reader instead of failing fast. Older OpenCV builds without
    the params overload fall back to the defaults.
    """
    params = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, int(open_timeout_s * 1000),
              cv2.CAP_PROP_READ_TIMEOUT_MSEC, int(read_timeout_s * 1000)]
    try:
        return cv2.VideoCapture(url, cv2.CAP_FFMPEG, params)
    except (TypeError, cv2.error):
        return cv2.VideoCapture(url, cv2.CAP_FFMPEG)


class HttpStreamSource(FrameSource):
    """MJPEG-over-HTTP camera, e.g. the ESP32-S3's `/stream`, newest frame wins.

    FFmpeg buffers network frames, and CAP_PROP_BUFFERSIZE does not bound that
    for HTTP, so reading "a frame" when the model is ready can return one that is
    seconds old. The load-bearing mechanism is the same as UsbCameraSource: a
    daemon thread reads the stream as fast as it arrives and keeps only the
    newest frame (CLAUDE.md rule 7).

    A Wi-Fi drop ends the stream. The reader reopens it and declares the source
    unhealthy after `reconnect_window_s` without a frame, at which point the
    node's liveness check drops to IDLE. It then keeps reconnecting, and turns
    healthy again on the first new frame. A venue network that stalls for a few
    seconds therefore costs one engagement, not the rest of the demo. Recovery
    re-arms nothing: leaving IDLE still takes a fresh cue, a HOLD and SPACE.

    Latency note: frames are timestamped when the host receives them, so the
    ESP32's JPEG encode and the Wi-Fi hop (~100-200 ms) are not in the measured
    compute latency. The lead predictor under-leads by that much.

    Thread-safety: ``read()`` is safe from any thread. ``start()``/``stop()``
    should be called from the owning thread only.
    """

    def __init__(
        self,
        url: str,
        open_timeout_s: float = 5.0,
        read_timeout_s: float = 2.0,
        reconnect_window_s: float = 3.0,
        flip_horizontal: bool = False,
        flip_vertical: bool = False,
        capture_factory: Optional[Callable[[str], "cv2.VideoCapture"]] = None,
    ) -> None:
        """Configure the stream.

        Args:
            url: Stream URL, e.g. ``http://192.168.43.50/stream``. A bare
                address is completed by ``normalize_stream_url``, with a warning.
            open_timeout_s: Bound on connecting.
            read_timeout_s: Bound on waiting for one frame.
            reconnect_window_s: How long without a frame, reconnecting, before
                the source reports unhealthy.
            flip_horizontal: Mirror left-right. Set it when the gimbal turns
                AWAY from a target on the pan axis: the control law assumes
                image x grows the way pan grows, and flipping the image is the
                one-line fix for a camera mounted the other way round.
            flip_vertical: The same for tilt.
            capture_factory: Opens a capture for a URL. Tests inject a fake;
                the default is FFmpeg with the two timeouts above.
        """
        self._url = normalize_stream_url(url)
        if self._url != url:
            logger.warning("camera.stream_url %r is not a full URL; using %s", url, self._url)
        self._reconnect_window_s = reconnect_window_s
        # cv2.flip codes: 1 horizontal, 0 vertical, -1 both; None = no flip.
        self._flip_code: Optional[int] = (
            -1 if flip_horizontal and flip_vertical
            else 1 if flip_horizontal
            else 0 if flip_vertical
            else None
        )
        self._open: Callable[[str], "cv2.VideoCapture"] = capture_factory or (
            lambda u: _open_http_capture(u, open_timeout_s, read_timeout_s)
        )
        self._cap: Optional["cv2.VideoCapture"] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._running = threading.Event()   # stop flag: cleared only by stop()
        self._healthy = threading.Event()   # frames are arriving
        self._latest: Optional[np.ndarray] = None
        self._frame_id: int = 0
        self._actual_size: Tuple[int, int] = (0, 0)
        self._reconnects: int = 0

    def start(self) -> None:
        """Connect, read one frame to learn the size, start the reader thread.

        Raises:
            RuntimeError: If the stream cannot be opened or yields no frame.
        """
        if self._running.is_set():
            return
        cap = self._safe_open()
        ok, probe = (cap.read() if cap is not None and cap.isOpened() else (False, None))
        if not ok or probe is None:
            if cap is not None:
                cap.release()
            raise RuntimeError(
                f"No frames from camera stream {self._url}. It serves ONE viewer: "
                f"close any browser tab or other program showing it. Still nothing: "
                f"press RST on the ESP32, wait 10 s and retry. If a browser on this "
                f"laptop can't show it either, run 'python -m tools.camera_probe "
                f"--config <your config> --find --write'."
            )
        self._actual_size = (probe.shape[1], probe.shape[0])
        probe = self._oriented(probe)
        logger.info("Camera stream %s open at %dx%d%s", self._url, *self._actual_size,
                    "" if self._flip_code is None else f", flip code {self._flip_code}")
        self._cap = cap
        with self._lock:
            self._latest = probe
            self._frame_id = 1
        self._healthy.set()
        self._running.set()
        self._thread = threading.Thread(
            target=self._reader_loop, name="stream-grabber", daemon=True
        )
        self._thread.start()

    def _reader_loop(self) -> None:
        """Read continuously, keep only the newest frame, reconnect on loss.

        Runs on the grabber thread until stop(). Past `reconnect_window_s`
        without a frame the source reports unhealthy, once, and keeps trying:
        every second, logging every tenth attempt, until frames return.
        """
        last_frame_at = time.monotonic()
        outage_reconnects = 0
        while self._running.is_set():
            cap = self._cap
            ok, frame = cap.read() if cap is not None else (False, None)
            if ok and frame is not None:
                last_frame_at = time.monotonic()
                frame = self._oriented(frame)
                with self._lock:
                    self._latest = frame
                    self._frame_id += 1
                if not self._healthy.is_set():
                    logger.info("Camera stream %s recovered after %d reconnects",
                                self._url, outage_reconnects)
                    self._healthy.set()
                outage_reconnects = 0
                continue

            if (self._healthy.is_set()
                    and time.monotonic() - last_frame_at > self._reconnect_window_s):
                logger.error(
                    "Camera stream %s gave no frame for %.1fs; treating as "
                    "disconnected and still reconnecting",
                    self._url, self._reconnect_window_s,
                )
                self._healthy.clear()
            if cap is not None:
                cap.release()
                self._cap = None
            time.sleep(0.2 if self._healthy.is_set() else 1.0)
            if not self._running.is_set():
                break
            self._reconnects += 1
            outage_reconnects += 1
            if self._healthy.is_set():
                logger.warning("Camera stream dropped; reconnecting (%d)", self._reconnects)
            elif outage_reconnects % 10 == 0:
                logger.warning("Camera stream %s still down; %d reconnects so far",
                               self._url, outage_reconnects)
            reopened = self._safe_open()
            if not self._running.is_set():   # stop() ran while we were connecting
                if reopened is not None:
                    reopened.release()
                break
            self._cap = reopened

    def _safe_open(self) -> Optional["cv2.VideoCapture"]:
        """Open the stream; None instead of an exception.

        An exception here would kill the reader thread while `is_healthy` still
        read True, freezing the node on its last frame instead of dropping it
        to IDLE. None goes through the normal no-frame path and times out.
        """
        try:
            return self._open(self._url)
        except (cv2.error, OSError, ValueError) as exc:
            logger.warning("Opening camera stream %s failed: %s", self._url, exc)
            return None

    def _oriented(self, frame: np.ndarray) -> np.ndarray:
        return frame if self._flip_code is None else cv2.flip(frame, self._flip_code)

    def read(self) -> Tuple[Optional[np.ndarray], int]:
        """Return the newest frame and its id. Never blocks on the network."""
        with self._lock:
            return (self._latest, self._frame_id)

    def stop(self) -> None:
        """Stop the reader thread and close the stream."""
        self._running.clear()
        self._healthy.clear()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
            if thread.is_alive():
                logger.warning("Stream grabber did not exit within 3s")
        self._thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        logger.info("Camera stream %s closed", self._url)

    @property
    def frame_size(self) -> Tuple[int, int]:
        return self._actual_size

    @property
    def is_healthy(self) -> bool:
        """False while no frame has arrived for `reconnect_window_s`, and after
        stop(). True again as soon as a reconnect delivers a frame."""
        return self._healthy.is_set()


class MockFrameSource(FrameSource):
    """Synthetic frames for hardware-free testing.

    The second implementation of FrameSource, and the one the test suite runs
    against. Emits all-black frames at a throttled rate: there is nothing in
    the pixels to find. For a target, pair it with a scripted detector, or with
    tools/simulator.py::SceneDetector for a closed loop (`main.py --sim-target`).
    """

    def __init__(self, width: int = 640, height: int = 480, fps: float = 60.0) -> None:
        """Args:
            fps: Synthetic frame rate. Rate-limiting matters: an unthrottled
                mock lets the vision worker free-run at thousands of iterations
                per second, so mock timings bear no relation to real ones and
                scripted fixtures are consumed almost instantly.
        """
        self._size = (width, height)
        self._period = 1.0 / fps if fps > 0 else 0.0
        self._frame_id = 0
        self._running = False
        self._last_emit = 0.0

    def start(self) -> None:
        self._running = True
        self._last_emit = time.monotonic()

    def read(self) -> Tuple[Optional[np.ndarray], int]:
        if not self._running:
            return (None, self._frame_id)
        now = time.monotonic()
        if now - self._last_emit < self._period:
            return (None, self._frame_id)
        self._last_emit = now
        self._frame_id += 1
        frame = np.zeros((self._size[1], self._size[0], 3), dtype=np.uint8)
        return (frame, self._frame_id)

    def stop(self) -> None:
        self._running = False

    @property
    def frame_size(self) -> Tuple[int, int]:
        return self._size
