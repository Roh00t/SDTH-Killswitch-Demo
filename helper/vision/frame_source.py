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
from abc import ABC, abstractmethod
from typing import Optional, Tuple

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
            thread.join(timeout=2.0)
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


class MockFrameSource(FrameSource):
    """Synthetic frames for hardware-free testing.

    The second implementation of FrameSource, and the one the test suite runs
    against. Emits a configurable blob on a dark field so a detector or a
    contrived stub has something to find.
    """

    def __init__(self, width: int = 640, height: int = 480) -> None:
        self._size = (width, height)
        self._frame_id = 0
        self._running = False

    def start(self) -> None:
        self._running = True

    def read(self) -> Tuple[Optional[np.ndarray], int]:
        if not self._running:
            return (None, self._frame_id)
        self._frame_id += 1
        frame = np.zeros((self._size[1], self._size[0], 3), dtype=np.uint8)
        return (frame, self._frame_id)

    def stop(self) -> None:
        self._running = False

    @property
    def frame_size(self) -> Tuple[int, int]:
        return self._size
