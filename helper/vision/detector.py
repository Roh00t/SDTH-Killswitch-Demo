"""Detection and tracking — YOLOv11 via Ultralytics, with ByteTrack identity.

Ultralytics' ``model.track(persist=True)`` runs ByteTrack internally and returns
stable track ids across frames and short dropouts. We use that rather than
bolting on a second tracker: ByteTrack already maintains a Kalman filter per
track, so its predicted state is available without writing a parallel one.
"""
from __future__ import annotations

import logging
import math
from abc import ABC, abstractmethod
from typing import List, Optional, Sequence

import numpy as np

from helper.vision.types import Detection

logger = logging.getLogger(__name__)

# Boxes thinner than this in either axis are clipped-at-edge artefacts, not
# targets. Measured against the live model: a noise frame produced a 44x0 box.
MIN_BOX_DIMENSION_PX: float = 2.0


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
        weights_path: str = "model/best.onnx",
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
        self._degenerate_rejected = 0
        self._track_id_churn = 0
        self._last_track_ids: set = set()

        # A fixed-shape ONNX export carries its own imgsz, and Ultralytics
        # SILENTLY overrides whatever the caller asks for. Requesting 640 from a
        # 1024 export is a no-op, which means anyone tuning imgsz for latency
        # sees no effect and no warning. Read the truth and say so.
        native = self._native_imgsz(weights_path)
        if native is not None and native != imgsz:
            logger.warning(
                "imgsz=%d was requested but %s is a fixed-shape export at %d. "
                "Ultralytics will silently use %d. Using the native size. To run "
                "at a smaller size you must re-export from Colab with "
                "imgsz=%d (and ideally dynamic=True).",
                imgsz, weights_path, native, native, imgsz,
            )
            self._imgsz = native

        names = getattr(self._model, "names", {}) or {}
        self._class_names = {int(k): str(v) for k, v in names.items()}
        logger.info(
            "Loaded %s, classes=%s, conf=%.2f, targets=%s",
            weights_path, list(self._class_names.values()), conf_threshold,
            sorted(self._target_classes) if self._target_classes else "ALL",
        )

        # Ultralytics builds its tracking predictor lazily, which loads the
        # model a SECOND time on the first track() call. Left alone that is a
        # multi-second stall on the first frame of an engagement — it looks like
        # a freeze and it drops the first observations. Pay it at startup.
        try:
            import numpy as _np

            warm = _np.zeros((self._imgsz, self._imgsz, 3), dtype=_np.uint8)
            self._model.track(
                warm, conf=self._conf, imgsz=self._imgsz, device=self._device,
                tracker=self._tracker_config, persist=False, verbose=False,
            )
            self.reset()
            logger.info("Detector warmed up; tracker predictor is resident")
        except Exception as exc:  # noqa: BLE001 - warmup is an optimisation
            logger.warning("Detector warmup failed (non-fatal): %s", exc)

        if self._target_classes:
            unknown = self._target_classes - set(self._class_names.values())
            if unknown:
                raise DetectorError(
                    f"target_classes {sorted(unknown)} are not in the model's "
                    f"classes {sorted(self._class_names.values())}"
                )

    @staticmethod
    def _native_imgsz(weights_path: str) -> Optional[int]:
        """Read the square input size baked into an ONNX export, if any.

        Returns None for non-ONNX weights or if the metadata cannot be read —
        a best-effort diagnostic must never block startup.
        """
        if not weights_path.lower().endswith(".onnx"):
            return None
        try:
            import onnxruntime as ort

            session = ort.InferenceSession(
                weights_path, providers=["CPUExecutionProvider"]
            )
            shape = session.get_inputs()[0].shape
            if len(shape) == 4 and isinstance(shape[2], int) and shape[2] == shape[3]:
                return int(shape[2])
        except Exception as exc:  # noqa: BLE001 - diagnostic only
            logger.debug("Could not read native imgsz from %s: %s", weights_path, exc)
        return None

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

                # Degenerate boxes are real: detections clipped at a frame edge
                # come back with zero or near-zero height/width. A zero-area box
                # produces a meaningless aimpoint, defeats the resolution gate's
                # intent, and is exactly the kind of thing ByteTrack drops a
                # track over. Reject at the source rather than propagating it.
                if w < MIN_BOX_DIMENSION_PX or h < MIN_BOX_DIMENSION_PX:
                    self._degenerate_rejected += 1
                    continue
                if not (math.isfinite(cx) and math.isfinite(cy)
                        and math.isfinite(w) and math.isfinite(h)):
                    self._degenerate_rejected += 1
                    continue

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

        # Track-id churn diagnostic. Constant identity changes starve the
        # velocity estimator (it resets on every switch), which silently
        # reinstates the standing tracking error that feed-forward removes.
        current_ids = {d.track_id for d in detections if d.track_id is not None}
        if current_ids and self._last_track_ids and current_ids != self._last_track_ids:
            self._track_id_churn += 1
        self._last_track_ids = current_ids

        return detections

    @property
    def diagnostics(self) -> dict:
        """Counters for demo-day forensics."""
        return {
            "degenerate_boxes_rejected": self._degenerate_rejected,
            "track_id_churn": self._track_id_churn,
            "imgsz": self._imgsz,
            "classes": sorted(self._class_names.values()),
        }

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
