"""
Train Detector
===============
Supports multiple model backends:
  - YOLOv8 / YOLO11 (ultralytics) — most accurate
  - YOLOv8n NCNN (ultralytics export) — fastest on Raspberry Pi CPU
  - MobileNetV2 SSD TFLite — lightest, works on Pi Zero
  - Mock/dummy detector — for testing without camera or model

Based on:
  - automaticdai/rpi-object-detection (YOLO on Pi pattern)
  - ultralytics/ultralytics (NCNN export pattern for ARM)

Place your trained model in:
  /models/best.pt          (PyTorch — for PC testing)
  /models/best_ncnn_model/ (NCNN folder — for Raspberry Pi)
  /models/model.tflite     (TFLite — for Pi Zero / MobileNet)
"""

import logging
import time
from typing import List, Dict, Optional, Tuple
import cv2
import numpy as np

logger = logging.getLogger(__name__)


class Detection:
    """Single object detection result."""
    def __init__(self, x1: int, y1: int, x2: int, y2: int,
                 confidence: float, label: str = "train"):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.confidence = confidence
        self.label = label

    def as_dict(self) -> dict:
        return {
            "x1": self.x1, "y1": self.y1,
            "x2": self.x2, "y2": self.y2,
            "confidence": self.confidence,
            "label": self.label,
        }

    def draw(self, frame, color=(0, 255, 0)):
        """Draw bounding box on an OpenCV frame."""
        cv2.rectangle(frame, (self.x1, self.y1), (self.x2, self.y2), color, 2)
        label = f"{self.label} {self.confidence:.0%}"
        cv2.putText(frame, label, (self.x1, self.y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        return frame


class BaseDetector:
    """Abstract base class for all detectors."""

    def load(self) -> None:
        raise NotImplementedError

    def detect(self, frame: np.ndarray) -> List[Detection]:
        raise NotImplementedError

    def close(self) -> None:
        pass


# ── YOLO Detector (ultralytics) ───────────────────────────────────────────────

class YOLODetector(BaseDetector):
    """
    YOLOv8 / YOLO11 detector using ultralytics.
    Supports .pt (PyTorch) and NCNN format (use for Raspberry Pi).

    Model path examples:
        models/best.pt           — PyTorch, use on PC / development
        models/best_ncnn_model   — NCNN folder, use on Raspberry Pi for speed
    """

    def __init__(self, model_path: str, confidence: float = 0.50,
                 img_size: int = 640, use_ncnn: bool = False):
        self.model_path  = model_path
        self.confidence  = confidence
        self.img_size    = img_size
        self.use_ncnn    = use_ncnn
        self._model      = None

    def load(self) -> None:
        from ultralytics import YOLO
        logger.info(f"Loading YOLO model: {self.model_path}")
        self._model = YOLO(self.model_path)
        logger.info("✅ YOLO model loaded")

    def detect(self, frame: np.ndarray) -> List[Detection]:
        if self._model is None:
            raise RuntimeError("Model not loaded. Call load() first.")

        results = self._model(frame, conf=self.confidence,
                              imgsz=self.img_size, verbose=False)
        detections = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = [int(c) for c in box.xyxy[0]]
            conf  = float(box.conf[0])
            cls   = int(box.cls[0])
            label = results[0].names.get(cls, "train")
            detections.append(Detection(x1, y1, x2, y2, conf, label))
        return detections

    def close(self) -> None:
        self._model = None


# ── MobileNetV2 SSD TFLite Detector ──────────────────────────────────────────

class TFLiteDetector(BaseDetector):
    """
    MobileNetV2 SSD TFLite detector — lightest option for Pi Zero.
    Model: models/model.tflite
    Labels: models/labels.txt  (one label per line)
    """

    def __init__(self, model_path: str, labels_path: str,
                 confidence: float = 0.50, input_size: Tuple[int, int] = (300, 300)):
        self.model_path  = model_path
        self.labels_path = labels_path
        self.confidence  = confidence
        self.input_size  = input_size
        self._interpreter= None
        self._labels     = []
        self._input_det  = None
        self._output_det = None

    def load(self) -> None:
        try:
            import tflite_runtime.interpreter as tflite
        except ImportError:
            import tensorflow.lite as tflite

        logger.info(f"Loading TFLite model: {self.model_path}")
        self._interpreter = tflite.Interpreter(model_path=self.model_path)
        self._interpreter.allocate_tensors()
        self._input_det  = self._interpreter.get_input_details()
        self._output_det = self._interpreter.get_output_details()

        with open(self.labels_path) as f:
            self._labels = [line.strip() for line in f.readlines()]
        logger.info("✅ TFLite model loaded")

    def detect(self, frame: np.ndarray) -> List[Detection]:
        h, w = frame.shape[:2]
        inp_h, inp_w = self.input_size

        # Preprocess
        resized = cv2.resize(frame, (inp_w, inp_h))
        rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        blob    = np.expand_dims(rgb, axis=0).astype(np.uint8)

        self._interpreter.set_tensor(self._input_det[0]["index"], blob)
        self._interpreter.invoke()

        boxes   = self._interpreter.get_tensor(self._output_det[1]["index"])[0]
        classes = self._interpreter.get_tensor(self._output_det[3]["index"])[0]
        scores  = self._interpreter.get_tensor(self._output_det[0]["index"])[0]

        detections = []
        for i in range(len(scores)):
            if scores[i] < self.confidence:
                continue
            ymin, xmin, ymax, xmax = boxes[i]
            x1 = int(xmin * w); y1 = int(ymin * h)
            x2 = int(xmax * w); y2 = int(ymax * h)
            cls_idx = int(classes[i])
            label   = self._labels[cls_idx] if cls_idx < len(self._labels) else "train"
            detections.append(Detection(x1, y1, x2, y2, float(scores[i]), label))
        return detections


# ── Mock Detector (for testing without camera or model) ───────────────────────

class MockDetector(BaseDetector):
    """
    Simulates detections on a timer — useful for testing collision logic
    without any camera or trained model.

    Sends a fake detection every `trigger_every_n` frames.
    """

    def __init__(self, trigger_every_n: int = 60,
                 zone_x1: int = 100, zone_y1: int = 100,
                 zone_x2: int = 400, zone_y2: int = 300):
        self.trigger_every_n = trigger_every_n
        self.zone = (zone_x1, zone_y1, zone_x2, zone_y2)
        self._frame_count = 0

    def load(self) -> None:
        logger.info("MockDetector loaded — will trigger every "
                    f"{self.trigger_every_n} frames")

    def detect(self, frame: np.ndarray) -> List[Detection]:
        self._frame_count += 1
        # Trigger a fake detection inside the zone periodically
        if (self._frame_count // self.trigger_every_n) % 2 == 1:
            x1, y1, x2, y2 = self.zone
            logger.debug(f"[MOCK] Fake train detection in zone (frame {self._frame_count})")
            return [Detection(x1 + 10, y1 + 10, x2 - 10, y2 - 10, 0.99, "train")]
        return []


# ── Factory ───────────────────────────────────────────────────────────────────

def create_detector(model_type: str, **kwargs) -> BaseDetector:
    """
    Factory function to create a detector by type string.

    Args:
        model_type: "yolo", "yolo_ncnn", "tflite", or "mock"
        **kwargs:   passed to the detector constructor

    Example:
        det = create_detector("yolo", model_path="models/best.pt")
        det = create_detector("yolo_ncnn", model_path="models/best_ncnn_model")
        det = create_detector("tflite",
                              model_path="models/model.tflite",
                              labels_path="models/labels.txt")
        det = create_detector("mock")
    """
    model_type = model_type.lower()
    if model_type == "yolo":
        return YOLODetector(**kwargs)
    elif model_type == "yolo_ncnn":
        return YOLODetector(use_ncnn=True, img_size=320, **kwargs)
    elif model_type == "tflite":
        return TFLiteDetector(**kwargs)
    elif model_type == "mock":
        return MockDetector(**kwargs)
    else:
        raise ValueError(f"Unknown model_type: {model_type!r}. "
                         "Choose from: yolo, yolo_ncnn, tflite, mock")
