# ══════════════════════════════════════════════════════════════════
#  zone_detector.py  —  OpenCV Shared Zone Detection
#
#  Reads zone coordinates from shared_zone.json automatically.
#  Uses background subtraction to detect any train entering the zone.
#  No AI, no training data, no YOLO — works immediately.
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import logging
import json
import os
import sys

import config

logger = logging.getLogger("ZoneDetector")

ZONE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         config.ZONE_FILE)


def load_zone_from_file():
    """
    Load shared zone coordinates from shared_zone.json.
    Exits with a clear error message if the file is missing.
    """
    if not os.path.exists(ZONE_FILE):
        print("\n❌ ERROR: shared_zone.json not found!")
        print("   Run calibration first:")
        print("   python calibrate_zone.py")
        print("   Draw the shared zone, press S to save, then run main.py\n")
        sys.exit(1)

    with open(ZONE_FILE, "r") as f:
        data = json.load(f)

    x1, y1, x2, y2 = data["x1"], data["y1"], data["x2"], data["y2"]
    logger.info(f"Zone loaded: ({x1},{y1}) → ({x2},{y2})  "
                f"size: {x2-x1}x{y2-y1}px  "
                f"saved: {data.get('saved_at', 'unknown')}")
    return x1, y1, x2, y2


class ZoneDetector:
    """
    Detects trains entering the shared zone using OpenCV background subtraction.

    How it works:
      1. Reads zone coordinates from shared_zone.json
      2. Opens USB camera
      3. Background subtractor learns what the empty layout looks like
      4. Any moving foreground blob detected inside the zone = train present
      5. Returns zone_active = True/False each frame
    """

    def __init__(self):
        self._cap        = None
        self._bg_sub     = None
        self._zone       = None
        self._detect_buf = []
        self._frame_w    = 0
        self._frame_h    = 0

    def start(self):
        """Load zone coordinates, open camera, initialise background subtractor."""

        x1, y1, x2, y2 = load_zone_from_file()
        self._zone = (x1, y1, x2, y2)

        self._cap = cv2.VideoCapture(config.CAMERA_INDEX)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        self._cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera index {config.CAMERA_INDEX}. "
                "Try changing CAMERA_INDEX in config.py to 1 or 2.")

        ret, frame = self._cap.read()
        if not ret:
            raise RuntimeError("Camera opened but cannot read frames.")

        self._frame_h, self._frame_w = frame.shape[:2]
        logger.info(f"Camera opened: {self._frame_w}x{self._frame_h}")

        self._bg_sub = cv2.createBackgroundSubtractorMOG2(
            history=config.BG_HISTORY,
            varThreshold=config.BG_THRESHOLD,
            detectShadows=config.BG_DETECT_SHADOW
        )
        logger.info("Zone detector ready ✅")

    def reload_zone(self):
        """Reload zone from file — call if user recalibrates while running."""
        x1, y1, x2, y2 = load_zone_from_file()
        self._zone = (x1, y1, x2, y2)
        self._detect_buf = []
        logger.info(f"Zone reloaded: ({x1},{y1}) → ({x2},{y2})")

    def read_frame(self):
        """
        Read one camera frame and check for train in shared zone.

        Returns:
            frame       — annotated BGR image for display
            zone_active — True if a train is detected in the zone
            contours    — list of detected contours inside the zone
        """
        ret, frame = self._cap.read()
        if not ret:
            logger.warning("Failed to read camera frame")
            return None, False, []

        x1, y1, x2, y2 = self._zone

        # Apply background subtraction
        fg_mask = self._bg_sub.apply(frame)

        # Clean noise with morphological operations
        kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN,  kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
        fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)

        # Restrict detection to shared zone only
        zone_mask = np.zeros_like(fg_mask)
        zone_mask[y1:y2, x1:x2] = fg_mask[y1:y2, x1:x2]

        # Find contours and filter by minimum area
        contours, _ = cv2.findContours(
            zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        significant = [c for c in contours
                       if cv2.contourArea(c) >= config.MIN_DETECTION_AREA]

        detected_now = len(significant) > 0

        # Rolling buffer — require N consecutive frames before triggering
        self._detect_buf.append(detected_now)
        if len(self._detect_buf) > config.DETECTION_FRAMES_THRESHOLD:
            self._detect_buf.pop(0)

        zone_active = (all(self._detect_buf) and
                       len(self._detect_buf) == config.DETECTION_FRAMES_THRESHOLD)

        if config.SHOW_VIDEO:
            frame = self._annotate(frame, zone_active, significant)

        return frame, zone_active, significant

    def _annotate(self, frame, zone_active, contours):
        """Draw zone box, train bounding boxes, and status bar on the frame."""
        x1, y1, x2, y2 = self._zone

        # Zone rectangle
        col = config.ZONE_COLOR_ACTIVE if zone_active else config.ZONE_COLOR_SAFE
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 3)
        cv2.putText(frame,
                    "ZONE ACTIVE" if zone_active else "SHARED ZONE",
                    (x1, max(y1 - 10, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

        # Bounding boxes around detected trains
        for c in contours:
            bx, by, bw, bh = cv2.boundingRect(c)
            cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), (0, 165, 255), 2)
            cv2.putText(frame, f"Train ({int(cv2.contourArea(c))}px)",
                        (bx, by - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

        # Status bar at top
        bar_col = (0, 0, 160) if zone_active else (0, 100, 0)
        cv2.rectangle(frame, (0, 0), (self._frame_w, 36), bar_col, -1)
        status = "COLLISION PREVENTION — TRAIN STOPPED" if zone_active \
                 else "MONITORING SHARED ZONE..."
        cv2.putText(frame, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

        # Bottom info line
        cv2.putText(frame,
                    f"Zone: ({x1},{y1})->({x2},{y2})  MinArea:{config.MIN_DETECTION_AREA}",
                    (10, self._frame_h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)

        return frame

    def stop(self):
        """Release camera and close windows."""
        if self._cap:
            self._cap.release()
        cv2.destroyAllWindows()
        logger.info("Camera released")
