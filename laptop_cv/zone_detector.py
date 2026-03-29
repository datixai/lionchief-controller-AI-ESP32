# ══════════════════════════════════════════════════════════════════
#  zone_detector.py  —  OpenCV Shared Zone Detection
#  Reads zone coordinates from shared_zone.json automatically
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import logging
import json
import os
import sys

import config

logger = logging.getLogger("ZoneDetector")

# ── Zone file path ────────────────────────────────────────────────
ZONE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         config.ZONE_FILE)


def load_zone_from_file():
    """
    Load shared zone coordinates from shared_zone.json.
    Exits with clear error if file not found.
    """
    if not os.path.exists(ZONE_FILE):
        print("\n❌ ERROR: shared_zone.json not found!")
        print("   You must run calibration first:")
        print("   python calibrate_zone.py")
        print("   Draw the shared track zone, press S to save, then run main.py\n")
        sys.exit(1)

    with open(ZONE_FILE, "r") as f:
        data = json.load(f)

    x1 = data["x1"]
    y1 = data["y1"]
    x2 = data["x2"]
    y2 = data["y2"]

    logger.info(f"Loaded zone from {ZONE_FILE}")
    logger.info(f"Zone: ({x1},{y1}) → ({x2},{y2})  "
                f"size: {x2-x1}x{y2-y1}px  "
                f"saved: {data.get('saved_at', 'unknown')}")

    return x1, y1, x2, y2


class ZoneDetector:
    """
    Uses OpenCV background subtraction to detect trains in shared zone.

    How it works:
      1. Reads zone coordinates from shared_zone.json
      2. Opens USB camera
      3. Builds background model (learns what layout looks like empty)
      4. Detects foreground blobs (anything moving = a train)
      5. Checks if blob is inside the shared zone box
      6. Returns True/False — is_zone_occupied
    """

    def __init__(self):
        self._cap        = None
        self._bg_sub     = None
        self._zone       = None   # loaded from file in start()
        self._detect_buf = []
        self._frame_w    = 0
        self._frame_h    = 0

    def start(self):
        """Load zone, open camera, init background subtractor."""

        # Load zone from file
        x1, y1, x2, y2 = load_zone_from_file()
        self._zone = (x1, y1, x2, y2)

        # Open camera
        self._cap = cv2.VideoCapture(config.CAMERA_INDEX)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        self._cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera {config.CAMERA_INDEX}. "
                "Try changing CAMERA_INDEX in config.py (0, 1, 2...)")

        ret, frame = self._cap.read()
        if ret:
            self._frame_h, self._frame_w = frame.shape[:2]
            logger.info(f"Camera opened: {self._frame_w}x{self._frame_h}")
        else:
            raise RuntimeError("Camera opened but cannot read frames")

        # Background subtractor
        self._bg_sub = cv2.createBackgroundSubtractorMOG2(
            history=config.BG_HISTORY,
            varThreshold=config.BG_THRESHOLD,
            detectShadows=config.BG_DETECT_SHADOW
        )
        logger.info("Zone detector ready ✅")

    def reload_zone(self):
        """Reload zone from file — useful if user recalibrates while running."""
        x1, y1, x2, y2 = load_zone_from_file()
        self._zone = (x1, y1, x2, y2)
        self._detect_buf = []   # reset buffer
        logger.info(f"Zone reloaded: ({x1},{y1}) → ({x2},{y2})")

    def read_frame(self):
        """
        Read one frame and detect if a train is in the shared zone.

        Returns:
          frame        — annotated BGR image for display
          zone_active  — True if a train is detected in the zone
          contours     — list of detected contours inside zone
        """
        ret, frame = self._cap.read()
        if not ret:
            logger.warning("Failed to read camera frame")
            return None, False, []

        x1, y1, x2, y2 = self._zone

        # Background subtraction
        fg_mask = self._bg_sub.apply(frame)

        # Clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN,  kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
        fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)

        # Crop mask to zone only
        zone_mask = np.zeros_like(fg_mask)
        zone_mask[y1:y2, x1:x2] = fg_mask[y1:y2, x1:x2]

        # Find contours inside zone
        contours, _ = cv2.findContours(
            zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        significant = [c for c in contours
                       if cv2.contourArea(c) >= config.MIN_DETECTION_AREA]

        detected_this_frame = len(significant) > 0

        # Rolling buffer — N consecutive frames must detect train
        self._detect_buf.append(detected_this_frame)
        if len(self._detect_buf) > config.DETECTION_FRAMES_THRESHOLD:
            self._detect_buf.pop(0)

        zone_active = (all(self._detect_buf) and
                       len(self._detect_buf) == config.DETECTION_FRAMES_THRESHOLD)

        # Annotate frame
        if config.SHOW_VIDEO:
            frame = self._annotate(frame, zone_active, significant)

        return frame, zone_active, significant

    def _annotate(self, frame, zone_active, contours):
        x1, y1, x2, y2 = self._zone

        # Zone box
        col = config.ZONE_COLOR_ACTIVE if zone_active else config.ZONE_COLOR_SAFE
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 3)

        # Zone label
        lbl = "⚠ ZONE ACTIVE" if zone_active else "SHARED ZONE"
        cv2.putText(frame, lbl, (x1, max(y1 - 10, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

        # Detected train bounding boxes
        for c in contours:
            bx, by, bw, bh = cv2.boundingRect(c)
            cv2.rectangle(frame, (bx, by), (bx+bw, by+bh),
                          (0, 165, 255), 2)
            cv2.putText(frame, f"Train ({int(cv2.contourArea(c))}px)",
                        (bx, by - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

        # Status bar
        bg = (0, 0, 160) if zone_active else (0, 100, 0)
        cv2.rectangle(frame, (0, 0), (self._frame_w, 36), bg, -1)
        status = "⚠  COLLISION PREVENTION — TRAIN STOPPED" if zone_active \
                 else "✓  MONITORING SHARED ZONE..."
        cv2.putText(frame, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

        # Bottom info
        zone_file_name = os.path.basename(ZONE_FILE)
        cv2.putText(frame,
                    f"Zone from {zone_file_name}: ({x1},{y1})->({x2},{y2})  "
                    f"MinArea:{config.MIN_DETECTION_AREA}",
                    (10, self._frame_h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)

        return frame

    def stop(self):
        if self._cap:
            self._cap.release()
        cv2.destroyAllWindows()
        logger.info("Camera released")
