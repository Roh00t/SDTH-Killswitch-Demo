"""Detection and tracking — YOLOv11 via Ultralytics, with ByteTrack identity.

Ultralytics' ``model.track(persist=True)`` runs ByteTrack internally and returns
stable track ids across frames and short dropouts. We use that rather than
bolting on a second tracker: ByteTrack already maintains a Kalman filter per
track, so its predicted state is available without writing a parallel one.
"""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import List, Optional, Sequence

import numpy as np

from helper.vision.types import Detection

logger = logging.getLogger(__name__)


class DetectorError(RuntimeError):
    """Raised when inference fails in a way the caller must handle.

    Never swallowed. Per guardrails section 1, a detector failure transitions the
    state machine toward IDLE rather than being printed and ignored.
    """


class Detector(ABC):
    """Contract for object detection with optional identity tracking."""

    @abstractmethod
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Run inference on one frame.

        Args:
            frame: BGR image as produced by a FrameSource.

        Returns:
            Detections above the configured confidence floor, filtered to the
            configured target classes. Empty list means nothing found — which is
            a normal result, not an error.

        Raises:
            DetectorError: If inference fails.
        """

    @abstractmethod
    def reset(self) -> None:
        """Clear tracker state.

        Must be called on every transition to IDLE. Stale track ids surviving an
        engagement would let an old authorisation bind to a new object.
        """


class UltralyticsDetector(Detector):
    """YOLOv11 detection plus ByteTrack identity via the Ultralytics API.

    Thread-safety: NOT thread-safe. The underlying model holds tracker state
    between calls. Confine to a single vision worker thread.
    """

    def __init__(
        self,
        weights_path: str,
        conf_threshold: float = 0.4,
        iou_threshold: float = 0.5,
        target_classes: Optional[Sequence[str]] = None,
        imgsz: int = 640,
        device: Optional[str] = None,
        tracker_config: str = "bytetrack.yaml",
    ) -> None:
        """Load the model.

        Args:
            weights_path: Path to a ``.pt`` file (trained on Colab, gitignored).
            conf_threshold: Confidence floor. Detections below this are not
                detections and must not start a HOLD timer.
            iou_threshold: NMS IoU threshold.
            target_classes: Class names to keep, e.g. ``("drone",)``. None keeps
                everything the model emits.
            imgsz: Inference resolution.
            device: 'cpu', 'cuda', 'mps', or None to let Ultralytics choose.
            tracker_config: Ultralytics tracker config name.

        Raises:
            DetectorError: If the model cannot be loaded.
        """
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise DetectorError(
                "ultralytics is not installed. pip install ultralytics"
            ) from exc

        try:
            self._model = YOLO(weights_path)
        except (OSError, ValueError, RuntimeError) as exc:
            raise DetectorError(f"Could not load weights '{weights_path}': {exc}") from exc

        self._conf = conf_threshold
        self._iou = iou_threshold
        self._imgsz = imgsz
        self._device = device
        self._tracker_config = tracker_config
        self._target_classes = set(target_classes) if target_classes else None

        names = getattr(self._model, "names", {}) or {}
        self._class_names = {int(k): str(v) for k, v in names.items()}
        logger.info(
            "Loaded %s, classes=%s, conf=%.2f, targets=%s",
            weights_path, list(self._class_names.values()), conf_threshold,
            sorted(self._target_classes) if self._target_classes else "ALL",
        )

        if self._target_classes:
            unknown = self._target_classes - set(self._class_names.values())
            if unknown:
                raise DetectorError(
                    f"target_classes {sorted(unknown)} are not in the model's "
                    f"classes {sorted(self._class_names.values())}"
                )

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Run tracked inference. See Detector.detect."""
        try:
            results = self._model.track(
                frame,
                conf=self._conf,
                iou=self._iou,
                imgsz=self._imgsz,
                device=self._device,
                tracker=self._tracker_config,
                persist=True,
                verbose=False,
            )
        except (RuntimeError, ValueError) as exc:
            raise DetectorError(f"Inference failed: {exc}") from exc

        detections: List[Detection] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for box in boxes:
                class_id = int(box.cls[0].item())
                class_name = self._class_names.get(class_id, f"class_{class_id}")

                if self._target_classes and class_name not in self._target_classes:
                    continue

                # xywh is centre-format. Keep it that way: Detection.xyxy exists
                # for the corner-format boundaries, so the legacy centre/corner
                # mismatch cannot recur.
                cx, cy, w, h = (float(v) for v in box.xywh[0].tolist())

                track_id = None
                if box.id is not None:
                    track_id = int(box.id[0].item())

                detections.append(
                    Detection(
                        x=cx, y=cy, w=w, h=h,
                        confidence=float(box.conf[0].item()),
                        class_id=class_id,
                        class_name=class_name,
                        track_id=track_id,
                    )
                )

        return detections

    def reset(self) -> None:
        """Drop ByteTrack state so track ids restart."""
        try:
            predictor = getattr(self._model, "predictor", None)
            if predictor is not None and hasattr(predictor, "trackers"):
                for tracker in predictor.trackers:
                    tracker.reset()
                logger.debug("Tracker state reset")
        except (AttributeError, RuntimeError) as exc:
            # Non-fatal: a failed reset means stale ids, which the state machine
            # detects as a track-id mismatch and handles by refusing to fire.
            logger.warning("Tracker reset failed (%s); ids may persist", exc)


class ScriptedDetector(Detector):
    """Returns pre-programmed detections. The second Detector implementation.

    Lets the state machine, HOLD timer, and authorisation binding be tested
    deterministically with no model, no camera, and no GPU.
    """

    def __init__(self, script: Sequence[Sequence[Detection]]) -> None:
        """Args:
            script: One list of Detections per call to detect(). Exhausting the
                script yields empty lists thereafter (target lost).
        """
        self._script = list(script)
        self._index = 0

    def detect(self, frame: np.ndarray) -> List[Detection]:
        if self._index >= len(self._script):
            return []
        batch = list(self._script[self._index])
        self._index += 1
        return batch

    def reset(self) -> None:
        self._index = 0
