# ══════════════════════════════════════════════════════════════════
#  zone_detector.py  —  Three-Zone Detection Engine
#
#  DUAL DETECTION — solves the stopped-train problem:
#    Method 1: MOG2 background subtraction → catches MOVING trains
#    Method 2: Reference frame comparison  → catches STOPPED trains
#
#  HOW THE STOPPED-TRAIN PROBLEM IS SOLVED:
#    MOG2 learns the stopped train as "background" after a few seconds
#    and stops detecting it. We fix this by keeping a reference frame
#    taken when the zone was EMPTY. Any frame different from that
#    reference means something is there — moving or stopped.
#
#  ZONE LOCK:
#    Once a train is detected in a zone, the zone stays LOCKED
#    even if detection momentarily drops (train partially out of view,
#    lighting flicker). Only cleared when train exits via Zone C.
#
#  THREE ZONES:
#    Zone A → inner loop approach (before shared section)
#    Zone B → outer loop approach (before shared section)
#    Zone C → shared section / exit
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import logging
import json
import os
import sys
import time

import config

logger = logging.getLogger("ZoneDetector")

ZONES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          config.ZONES_FILE)


# ── Zone data class ───────────────────────────────────────────────

class Zone:
    """Represents a single detection zone with its own detection state."""

    def __init__(self, key, rect, label, color):
        self.key   = key
        self.x1    = rect["x1"]
        self.y1    = rect["y1"]
        self.x2    = rect["x2"]
        self.y2    = rect["y2"]
        self.label = label
        self.color = color  # BGR

        # Detection state
        self.detect_buf     = []   # rolling frame buffer
        self.locked         = False  # zone lock — stays True until train exits
        self.reference_frame = None  # empty-track reference for stopped train detection

    @property
    def rect(self):
        return (self.x1, self.y1, self.x2, self.y2)

    @property
    def area(self):
        return (self.x2 - self.x1) * (self.y2 - self.y1)


# ── Load zones ────────────────────────────────────────────────────

def load_zones():
    """
    Load all three zones from zones.json.
    Exits with clear error if file missing or zones incomplete.
    """
    if not os.path.exists(ZONES_FILE):
        print("\n❌ zones.json not found!")
        print("   Run calibration first:")
        print("   python calibrate_zone.py")
        print("   Draw all three zones and press A to save.\n")
        sys.exit(1)

    with open(ZONES_FILE, "r") as f:
        data = json.load(f)

    zones = {}
    colors = {
        "A": config.COLOR_ZONE_A_SAFE,
        "B": config.COLOR_ZONE_B_SAFE,
        "C": config.COLOR_ZONE_C_SAFE,
    }

    missing = []
    for key in ["A", "B", "C"]:
        if key not in data or not data[key]:
            missing.append(key)
            continue
        z = data[key]
        zones[key] = Zone(
            key   = key,
            rect  = z["rect"],
            label = z.get("label", f"Zone {key}"),
            color = colors[key],
        )
        logger.info(
            f"Zone {key} loaded: ({z['rect']['x1']},{z['rect']['y1']}) → "
            f"({z['rect']['x2']},{z['rect']['y2']})  "
            f"size: {z.get('zone_width','?')}x{z.get('zone_height','?')}px"
        )

    if missing:
        print(f"\n⚠️  Missing zones: {missing}")
        print("   Run  python calibrate_zone.py  and draw the missing zones.")
        if len(missing) == 3:
            sys.exit(1)
        print("   Continuing with available zones...\n")

    return zones


# ── Main detector class ───────────────────────────────────────────

class ZoneDetector:
    """
    Three-zone camera detector for collision prevention.

    Detection flow per zone:
      1. Apply MOG2 background subtraction → catches moving trains
      2. Compare to reference frame → catches stopped trains
      3. Combine results with OR
      4. Filter by minimum blob size
      5. Require N consecutive frames (anti-flicker)
      6. Apply zone lock — once active stays active until manually cleared
    """

    def __init__(self):
        self._cap         = None
        self._bg_sub      = None
        self._zones       = {}
        self._frame_w     = 0
        self._frame_h     = 0
        self._frame_count = 0

    # ── Public API ────────────────────────────────────────────────

    def start(self):
        """Load zones, open camera, init background subtractor."""
        self._zones = load_zones()

        self._cap = cv2.VideoCapture(config.CAMERA_INDEX)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        self._cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera {config.CAMERA_INDEX}. "
                "Try changing CAMERA_INDEX in config.py to 1 or 2.")

        ret, frame = self._cap.read()
        if not ret:
            raise RuntimeError("Camera opened but cannot read frames.")

        self._frame_h, self._frame_w = frame.shape[:2]
        logger.info(f"Camera: {self._frame_w}x{self._frame_h}")

        # MOG2 — tuned to be LESS sensitive to subtle changes
        self._bg_sub = cv2.createBackgroundSubtractorMOG2(
            history      = config.BG_HISTORY,
            varThreshold = config.BG_THRESHOLD,
            detectShadows= config.BG_DETECT_SHADOW
        )

        # Warm up background model with 30 frames
        logger.info("Warming up background model (30 frames)...")
        for _ in range(30):
            ret, f = self._cap.read()
            if ret:
                self._bg_sub.apply(f)

        # Take initial reference frames for each zone (empty track)
        ret, ref_frame = self._cap.read()
        if ret:
            gray_ref = cv2.cvtColor(ref_frame, cv2.COLOR_BGR2GRAY)
            gray_ref = cv2.GaussianBlur(gray_ref, (21, 21), 0)
            for zone in self._zones.values():
                zone.reference_frame = gray_ref.copy()
            logger.info("Reference frames captured for all zones ✅")

        logger.info("Zone detector ready ✅")

    def update_reference(self, zone_key):
        """
        Capture a fresh reference frame for a zone.
        Call this when zone is confirmed empty.
        """
        if zone_key not in self._zones:
            return
        ret, frame = self._cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)
            self._zones[zone_key].reference_frame = gray
            logger.info(f"Reference frame updated for Zone {zone_key}")

    def unlock_zone(self, zone_key):
        """Manually unlock a zone (called by main.py after train exits)."""
        if zone_key in self._zones:
            self._zones[zone_key].locked     = False
            self._zones[zone_key].detect_buf = []
            logger.info(f"Zone {zone_key} unlocked")

    def read_frame(self):
        """
        Read one frame and run detection on all three zones.

        Returns:
            frame        — annotated BGR image
            zone_states  — dict {"A": bool, "B": bool, "C": bool}
                           True = train detected OR zone is locked
        """
        ret, frame = self._cap.read()
        if not ret:
            logger.warning("Camera read failed")
            return None, {}

        self._frame_count += 1

        # Convert to grayscale + blur for stable detection
        gray        = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur   = cv2.GaussianBlur(gray, (21, 21), 0)

        # MOG2 foreground mask (moving objects)
        fg_mog2 = self._bg_sub.apply(frame)
        fg_mog2 = self._clean_mask(fg_mog2)

        zone_states = {}

        for key, zone in self._zones.items():
            x1, y1, x2, y2 = zone.rect

            # ── Method 1: MOG2 (moving train) ─────────────────────
            mog2_zone = fg_mog2[y1:y2, x1:x2]
            mog2_detected = self._has_significant_blob(mog2_zone)

            # ── Method 2: Reference frame diff (stopped train) ────
            ref_detected = False
            if zone.reference_frame is not None:
                diff = cv2.absdiff(
                    gray_blur[y1:y2, x1:x2],
                    zone.reference_frame[y1:y2, x1:x2]
                )
                _, diff_thresh = cv2.threshold(
                    diff, config.REFERENCE_DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)

                zone_pixels  = (x2-x1) * (y2-y1)
                changed_frac = np.sum(diff_thresh > 0) / max(zone_pixels, 1)
                ref_detected = changed_frac >= config.REFERENCE_MIN_CHANGED

            # ── Combine: train present if EITHER method fires ─────
            detected_now = mog2_detected or ref_detected

            # ── Rolling frame buffer (anti-flicker) ───────────────
            zone.detect_buf.append(detected_now)
            if len(zone.detect_buf) > config.DETECTION_FRAMES_THRESHOLD:
                zone.detect_buf.pop(0)

            # Confirmed detection = all N frames agree
            confirmed = (
                len(zone.detect_buf) == config.DETECTION_FRAMES_THRESHOLD
                and all(zone.detect_buf)
            )

            # ── Zone lock logic ────────────────────────────────────
            if config.ZONE_LOCK_ENABLED:
                if confirmed and not zone.locked:
                    zone.locked = True
                    logger.info(f"Zone {key} LOCKED — train detected")
                # NOTE: zone.locked is only cleared by main.py
                # calling unlock_zone() after exit is confirmed
                zone_states[key] = zone.locked
            else:
                zone_states[key] = confirmed

        # ── Annotate frame ─────────────────────────────────────────
        if config.SHOW_VIDEO:
            frame = self._annotate(frame, zone_states)

        return frame, zone_states

    def stop(self):
        if self._cap:
            self._cap.release()
        cv2.destroyAllWindows()

    # ── Private helpers ───────────────────────────────────────────

    def _clean_mask(self, mask):
        """
        Remove noise from foreground mask.
        Larger kernel = less sensitive to small changes (light, shadows).
        """
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def _has_significant_blob(self, zone_mask):
        """
        Check if the mask contains any blob large enough to be a train.
        Filters out: shadows, light flickers, insects, hands (if too small).
        """
        contours, _ = cv2.findContours(
            zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) >= config.MIN_DETECTION_AREA:
                return True
        return False

    def _annotate(self, frame, zone_states):
        """Draw zones, status bars, and detection info on the frame."""
        h, w = frame.shape[:2]

        for key, zone in self._zones.items():
            x1, y1, x2, y2 = zone.rect
            active = zone_states.get(key, False)

            # Color: red if active, zone's own safe color otherwise
            col = (0, 0, 255) if active else zone.color

            # Semi-transparent fill
            overlay = frame.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2),
                          (0, 0, 180) if active else col, -1)
            cv2.addWeighted(overlay, 0.20, frame, 0.80, 0, frame)

            # Border
            cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)

            # Zone label
            lbl = f"Zone {key} {'⚠ ACTIVE' if active else '— clear'}"
            cv2.putText(frame, lbl,
                        (x1 + 4, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

        # ── Top status bar ────────────────────────────────────────
        any_active = any(zone_states.values())
        bar_col = (0, 0, 140) if any_active else (0, 80, 0)
        cv2.rectangle(frame, (0, 0), (w, 40), bar_col, -1)

        if any_active:
            active_zones = [k for k, v in zone_states.items() if v]
            cv2.putText(frame,
                        f"COLLISION PREVENTION ACTIVE — Zone(s): "
                        f"{' '.join(active_zones)} — OUTER TRAIN STOPPED",
                        (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (255, 255, 255), 2)
        else:
            cv2.putText(frame,
                        "MONITORING — All zones clear",
                        (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (200, 255, 200), 1)

        # ── Zone status strip bottom ──────────────────────────────
        strip_y = h - 28
        cv2.rectangle(frame, (0, strip_y), (w, h), (20, 20, 20), -1)
        for i, (key, zone) in enumerate(self._zones.items()):
            active = zone_states.get(key, False)
            col    = (0, 0, 255) if active else zone.color
            txt    = f"Zone {key}: {'LOCKED' if zone.locked else 'clear'}"
            cv2.putText(frame, txt,
                        (10 + i * 220, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        return frame
