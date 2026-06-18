# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  Color Blob Train Detector
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  Detects both trains in each camera frame by the colored stickers
#  placed on their roofs. Uses HSV color space which is more robust
#  to lighting changes than BGR.
#
#  SETUP REQUIRED:
#    Each train must have a bright solid-color sticker on its roof.
#    Train A (front):  one color   e.g. bright RED
#    Train B (rear):   other color e.g. bright YELLOW
#    Colors must be clearly different from the track and background.
#
#  Run calibrate.py to set the exact HSV ranges for your lighting.
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import json
import os
import logging
import time
from collections import deque

import config

logger = logging.getLogger("TrainDetector")


class TrainPosition:
    """Holds detected position and metadata for one train."""

    def __init__(self, x: int, y: int, area: float,
                 bbox: tuple = None, confidence: float = 1.0):
        self.x          = x         # centroid x (pixels)
        self.y          = y         # centroid y (pixels)
        self.area       = area      # blob area (pixels²)
        self.bbox       = bbox      # (x, y, w, h) bounding box or None
        self.confidence = confidence
        self.timestamp  = time.time()

    def as_tuple(self):
        return (self.x, self.y)

    def __repr__(self):
        return f"TrainPosition(x={self.x}, y={self.y}, area={self.area:.0f})"


class TrainDetector:
    """
    Detects both trains in camera frames using colored stickers.

    Detection pipeline per train per frame:
      1. Convert BGR → HSV
      2. GaussianBlur to reduce noise
      3. Color mask using inRange (dual-range for red)
      4. Morphological close + dilate to clean mask
      5. Find contours
      6. Filter by minimum area
      7. Return centroid of largest matching blob

    Position smoothing:
      A small rolling average smooths out frame-to-frame jitter
      without introducing significant lag.
    """

    def __init__(self):
        # Color ranges loaded from calibration file or config defaults
        self._a_lower1  = None
        self._a_upper1  = None
        self._a_lower2  = None   # Second range for red wrap-around (HSV)
        self._a_upper2  = None
        self._a_dual    = False  # True if Train A color needs two HSV ranges

        self._b_lower   = None
        self._b_upper   = None
        self._b_dual    = False

        # Rolling position buffers for smoothing
        _buf = config.POSITION_SMOOTH_FRAMES
        self._a_pos_buf = deque(maxlen=_buf)
        self._b_pos_buf = deque(maxlen=_buf)

        # Last seen timestamps (for missing-train detection)
        self._a_last_seen = 0.0
        self._b_last_seen = 0.0

        # Morphological cleanup kernel
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (7, 7))

        self._load_calibration()

    # ── Public API ────────────────────────────────────────────────

    def detect(self, frame) -> tuple:
        """
        Detect both trains in a camera frame.

        Args:
            frame: BGR camera frame (numpy array)

        Returns:
            (train_a, train_b)
            Each is TrainPosition or None if not found in this frame.
            None means the train's sticker color was not detected.
        """
        # Pre-process: blur to reduce color noise
        blurred = cv2.GaussianBlur(
            frame,
            (config.BLUR_KERNEL_SIZE, config.BLUR_KERNEL_SIZE), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Detect each train
        raw_a = self._detect_color(
            hsv,
            self._a_lower1, self._a_upper1,
            self._a_lower2 if self._a_dual else None,
            self._a_upper2 if self._a_dual else None,
        )
        raw_b = self._detect_color(
            hsv,
            self._b_lower, self._b_upper,
        )

        # Update seen timestamps
        now = time.time()
        if raw_a: self._a_last_seen = now
        if raw_b: self._b_last_seen = now

        # Smooth positions using rolling buffer
        smooth_a = self._smooth(raw_a, self._a_pos_buf)
        smooth_b = self._smooth(raw_b, self._b_pos_buf)

        return smooth_a, smooth_b

    def get_masks(self, frame) -> tuple:
        """
        Return binary masks for both trains — used by calibrate.py
        and main.py for debug display.

        Returns:
            (mask_a, mask_b) — each is a binary numpy array
        """
        blurred = cv2.GaussianBlur(
            frame,
            (config.BLUR_KERNEL_SIZE, config.BLUR_KERNEL_SIZE), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask_a = self._build_mask(
            hsv,
            self._a_lower1, self._a_upper1,
            self._a_lower2 if self._a_dual else None,
            self._a_upper2 if self._a_dual else None,
        )
        mask_b = self._build_mask(
            hsv, self._b_lower, self._b_upper)

        return mask_a, mask_b

    def is_train_a_missing(self) -> bool:
        """True if Train A has not been seen for MISSING_TIMEOUT_S."""
        return (time.time() - self._a_last_seen) > config.MISSING_TIMEOUT_S

    def is_train_b_missing(self) -> bool:
        """True if Train B has not been seen for MISSING_TIMEOUT_S."""
        return (time.time() - self._b_last_seen) > config.MISSING_TIMEOUT_S

    def reload_calibration(self):
        """Reload color ranges from calibration file (call after calibrate.py)."""
        self._a_pos_buf.clear()
        self._b_pos_buf.clear()
        self._a_last_seen = 0.0
        self._b_last_seen = 0.0
        self._load_calibration()
        logger.info("Calibration reloaded")

    # ── Private helpers ───────────────────────────────────────────

    def _load_calibration(self):
        """Load HSV color ranges from calibration.json or fall back to config defaults."""
        cal_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            config.CALIBRATION_FILE)

        if os.path.exists(cal_path):
            try:
                with open(cal_path) as f:
                    cal = json.load(f)

                a = cal.get("train_a", {})
                b = cal.get("train_b", {})

                self._a_lower1 = np.array(a.get("hsv_lower1", config.TRAIN_A_HSV_LOWER1))
                self._a_upper1 = np.array(a.get("hsv_upper1", config.TRAIN_A_HSV_UPPER1))
                self._a_dual   = a.get("dual_range", config.TRAIN_A_USES_DUAL)
                if self._a_dual:
                    self._a_lower2 = np.array(a.get("hsv_lower2", config.TRAIN_A_HSV_LOWER2))
                    self._a_upper2 = np.array(a.get("hsv_upper2", config.TRAIN_A_HSV_UPPER2))

                self._b_lower  = np.array(b.get("hsv_lower",  config.TRAIN_B_HSV_LOWER))
                self._b_upper  = np.array(b.get("hsv_upper",  config.TRAIN_B_HSV_UPPER))
                self._b_dual   = b.get("dual_range", config.TRAIN_B_USES_DUAL)

                logger.info(f"Calibration loaded from {cal_path}")
                return

            except Exception as e:
                logger.warning(f"Could not load calibration ({e}) — using config defaults")

        # Fall back to config defaults
        self._a_lower1 = np.array(config.TRAIN_A_HSV_LOWER1)
        self._a_upper1 = np.array(config.TRAIN_A_HSV_UPPER1)
        self._a_dual   = config.TRAIN_A_USES_DUAL
        if self._a_dual:
            self._a_lower2 = np.array(config.TRAIN_A_HSV_LOWER2)
            self._a_upper2 = np.array(config.TRAIN_A_HSV_UPPER2)
        self._b_lower  = np.array(config.TRAIN_B_HSV_LOWER)
        self._b_upper  = np.array(config.TRAIN_B_HSV_UPPER)
        self._b_dual   = config.TRAIN_B_USES_DUAL
        logger.info("Using default color ranges from config.py — run calibrate.py for best results")

    def _build_mask(self, hsv, lower1, upper1, lower2=None, upper2=None):
        """Build cleaned binary mask for a color range."""
        mask = cv2.inRange(hsv, lower1, upper1)
        if lower2 is not None and upper2 is not None:
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask  = cv2.bitwise_or(mask, mask2)
        # Close small gaps, then dilate to merge nearby pixels
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        mask = cv2.dilate(mask, self._kernel, iterations=1)
        return mask

    def _detect_color(self, hsv, lower1, upper1,
                      lower2=None, upper2=None) -> "TrainPosition | None":
        """Find largest blob of given color. Returns TrainPosition or None."""
        mask = self._build_mask(hsv, lower1, upper1, lower2, upper2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Pick largest blob by area
        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)

        if area < config.MIN_BLOB_AREA:
            return None

        # Centroid
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Bounding box
        x, y, w, h = cv2.boundingRect(largest)

        return TrainPosition(cx, cy, area, (x, y, w, h))

    def _smooth(self, pos: "TrainPosition | None",
                buf: deque) -> "TrainPosition | None":
        """Apply rolling average to train position."""
        if pos is not None:
            buf.append((pos.x, pos.y, pos.area))

        if not buf:
            return None

        # Average buffered positions
        xs    = [p[0] for p in buf]
        ys    = [p[1] for p in buf]
        areas = [p[2] for p in buf]
        sx    = int(round(sum(xs) / len(xs)))
        sy    = int(round(sum(ys) / len(ys)))
        sa    = sum(areas) / len(areas)

        # Use latest bbox if available
        bbox = pos.bbox if pos else None
        return TrainPosition(sx, sy, sa, bbox)


def pixel_distance(a: "TrainPosition | None",
                   b: "TrainPosition | None") -> "float | None":
    """
    Euclidean pixel distance between two train centroids.
    Returns None if either train is not detected.

    NOTE: This is straight-line distance, not along-track distance.
    For trains close together on the same straight section this is
    a good approximation. For wrap-around detection on a loop, a
    future improvement would measure distance along the track path.
    """
    if a is None or b is None:
        return None
    dx = a.x - b.x
    dy = a.y - b.y
    return float(np.sqrt(dx * dx + dy * dy))
