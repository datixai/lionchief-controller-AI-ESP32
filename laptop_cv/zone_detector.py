# ══════════════════════════════════════════════════════════════════
#  zone_detector.py  —  Polygon Zone Detector
#  Harry Locomotive Project  |  Datix AI  |  May 2026
#
#  DUAL DETECTION — solves the stopped-train problem:
#    Method 1: MOG2 background subtraction → catches MOVING trains
#    Method 2: Reference frame comparison  → catches STOPPED trains
#              (MOG2 learns stopped train as background after ~10s
#               and stops detecting it — reference frame never forgets)
#
#  POLYGON ZONES:
#    Zones are polygons drawn by calibrate_zone.py.
#    A polygon can follow any curved track — not just a rectangle.
#    Detection mask is created using cv2.fillPoly for exact coverage.
#
#  ZONE LOCK:
#    Once a train is detected in a zone, the zone stays LOCKED
#    even if detection momentarily drops (lighting change, partial
#    occlusion). Only cleared when main.py calls unlock_zone().
#
#  THREE ZONES:
#    Zone A → inner loop approach (inner train only)
#    Zone B → outer loop approach (outer train only)
#    Zone C → shared section / exit (any train)
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


# ── Zone class ────────────────────────────────────────────────────

class Zone:
    """
    A single detection zone defined by a polygon.
    Holds its own detection state and reference frame.
    """

    def __init__(self, key: str, points: list, label: str,
                 safe_color, active_color, frame_shape: tuple):
        self.key          = key
        self.points       = np.array(points, dtype=np.int32)
        self.label        = label
        self.safe_color   = safe_color    # BGR when clear
        self.active_color = active_color  # BGR when occupied

        # Pre-build mask — only recomputed if frame size changes
        h, w = frame_shape[:2]
        self.mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(self.mask, [self.points], 255)

        # Detection state
        self.detect_buf      = []    # rolling frame buffer
        self.locked          = False # stays True until manually unlocked
        self.reference_frame = None  # empty-track snapshot for stopped-train detection

    @property
    def active(self) -> bool:
        return self.locked


# ── Load zones from file ──────────────────────────────────────────

def load_zones(frame_shape: tuple) -> dict:
    """
    Load polygon zones from zones.json.
    Returns dict {key: Zone}.
    Exits with a clear message if zones are missing.
    """
    zones_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), config.ZONES_FILE)

    if not os.path.exists(zones_file):
        print("\n❌ zones.json not found!")
        print("   Run calibration first:")
        print("   python calibrate_zone.py")
        print("   Draw all three zones and press A to save.\n")
        sys.exit(1)

    with open(zones_file, "r") as f:
        data = json.load(f)

    color_map = {
        "A": (config.COLOR_ZONE_A_SAFE, config.COLOR_ZONE_A_ACTIVE),
        "B": (config.COLOR_ZONE_B_SAFE, config.COLOR_ZONE_B_ACTIVE),
        "C": (config.COLOR_ZONE_C_SAFE, config.COLOR_ZONE_C_ACTIVE),
    }

    zones   = {}
    missing = []

    for key in ["A", "B", "C"]:
        if key not in data or not data[key].get("points"):
            missing.append(key)
            continue

        pts = data[key]["points"]
        if len(pts) < 3:
            logger.warning(f"Zone {key} has fewer than 3 points — skipping")
            missing.append(key)
            continue

        safe_c, active_c = color_map[key]
        zones[key] = Zone(
            key          = key,
            points       = pts,
            label        = data[key].get("label", f"Zone {key}"),
            safe_color   = safe_c,
            active_color = active_c,
            frame_shape  = frame_shape,
        )
        logger.info(
            f"Zone {key} loaded — {len(pts)} polygon points, "
            f"area: {data[key].get('area_px','?')}px²"
        )

    if missing:
        print(f"\n⚠️  Missing zones: {missing}")
        print("   Run python calibrate_zone.py to draw them.")
        if len(missing) == 3:
            sys.exit(1)

    return zones


# ── Main detector class ───────────────────────────────────────────

class ZoneDetector:
    """
    Three-zone polygon camera detector.

    Per-frame detection pipeline for each zone:
      1. Apply polygon mask to frame
      2. MOG2 background subtraction → catches moving trains
      3. Reference frame diff        → catches stopped trains
      4. OR both methods
      5. Minimum blob size filter
      6. N-frame consistency buffer (prevents flicker triggers)
      7. Zone lock — once active, stays until unlock_zone() called
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
        """Open camera, load zones, warm up background model."""
        self._cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        self._cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera {config.CAMERA_INDEX}. "
                "Try changing CAMERA_INDEX in config.py to 0 or 2.")

        ret, frame = self._cap.read()
        if not ret:
            raise RuntimeError("Camera opened but cannot read frames.")

        self._frame_h, self._frame_w = frame.shape[:2]
        logger.info(f"Camera: {self._frame_w}×{self._frame_h}")

        # Load polygon zones using actual frame dimensions
        self._zones = load_zones(frame.shape)

        # MOG2 background subtractor — tuned for ceiling-mounted camera
        self._bg_sub = cv2.createBackgroundSubtractorMOG2(
            history       = config.BG_HISTORY,
            varThreshold  = config.BG_THRESHOLD,
            detectShadows = config.BG_DETECT_SHADOW,
        )

        # Warm up background model with empty-track frames
        logger.info("Warming up background model (40 frames)...")
        for _ in range(40):
            ret, f = self._cap.read()
            if ret:
                self._bg_sub.apply(f)

        # Take initial reference frames (empty track snapshot)
        ret, ref_frame = self._cap.read()
        if ret:
            gray_ref = self._to_gray_blur(ref_frame)
            for zone in self._zones.values():
                zone.reference_frame = gray_ref.copy()
            logger.info("Reference frames captured ✅")

        logger.info("Zone detector ready ✅")

    def update_reference(self, zone_key: str):
        """
        Refresh the reference frame for a zone.
        Call this after confirming the zone is empty
        (e.g., after inner train parks and outer resumes).
        """
        if zone_key not in self._zones:
            return
        ret, frame = self._cap.read()
        if ret:
            gray = self._to_gray_blur(frame)
            self._zones[zone_key].reference_frame = gray
            logger.debug(f"Reference frame updated for Zone {zone_key}")

    def unlock_zone(self, zone_key: str):
        """Unlock a zone — called by main.py after train exits."""
        if zone_key in self._zones:
            z = self._zones[zone_key]
            z.locked     = False
            z.detect_buf = []
            logger.info(f"Zone {zone_key} unlocked")

    def read_frame(self) -> tuple:
        """
        Read one camera frame and run detection on all three zones.

        Returns:
            (frame, zone_states)
            frame       — annotated BGR image for display
            zone_states — dict {"A": bool, "B": bool, "C": bool}
                          True = zone is locked (train present or was present)
        """
        ret, frame = self._cap.read()
        if not ret:
            logger.warning("Camera frame read failed")
            return None, {}

        self._frame_count += 1
        gray_blur = self._to_gray_blur(frame)

        # MOG2 on full frame, then mask per zone
        fg_mog2 = self._bg_sub.apply(frame)
        fg_mog2 = self._clean_mask(fg_mog2)

        zone_states = {}

        for key, zone in self._zones.items():
            # ── Method 1: MOG2 within polygon ──────────────────
            mog2_zone     = cv2.bitwise_and(fg_mog2, fg_mog2, mask=zone.mask)
            mog2_detected = self._has_significant_blob(mog2_zone)

            # ── Method 2: Reference frame diff within polygon ──
            ref_detected = False
            if zone.reference_frame is not None:
                diff = cv2.absdiff(gray_blur, zone.reference_frame)
                # Apply polygon mask to diff
                diff_masked = cv2.bitwise_and(diff, diff, mask=zone.mask)
                _, diff_thresh = cv2.threshold(
                    diff_masked,
                    config.REFERENCE_DIFF_THRESHOLD,
                    255,
                    cv2.THRESH_BINARY
                )
                zone_area     = int(np.sum(zone.mask > 0))
                changed_px    = int(np.sum(diff_thresh > 0))
                changed_frac  = changed_px / max(zone_area, 1)
                ref_detected  = changed_frac >= config.REFERENCE_MIN_CHANGED_FRAC

            # ── Combine: detected if EITHER method fires ────────
            detected_now = mog2_detected or ref_detected

            # ── Rolling frame buffer (anti-flicker) ─────────────
            zone.detect_buf.append(detected_now)
            if len(zone.detect_buf) > config.DETECTION_FRAMES_REQUIRED:
                zone.detect_buf.pop(0)

            confirmed = (
                len(zone.detect_buf) == config.DETECTION_FRAMES_REQUIRED
                and all(zone.detect_buf)
            )

            # ── Zone lock ────────────────────────────────────────
            if config.ZONE_LOCK_ENABLED:
                if confirmed and not zone.locked:
                    zone.locked = True
                    logger.info(f"Zone {key} LOCKED — train detected")
                zone_states[key] = zone.locked
            else:
                zone_states[key] = confirmed

        # Annotate frame
        if config.SHOW_VIDEO:
            frame = self._annotate(frame, zone_states)

        return frame, zone_states

    def stop(self):
        if self._cap:
            self._cap.release()
        cv2.destroyAllWindows()

    # ── Private helpers ───────────────────────────────────────────

    def _to_gray_blur(self, frame) -> np.ndarray:
        """Convert to grayscale and blur for stable comparison."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.GaussianBlur(gray, (21, 21), 0)

    def _clean_mask(self, mask) -> np.ndarray:
        """
        Remove noise from MOG2 mask.
        Larger kernel = less sensitive to shadows, micro-movements.
        Ceiling camera gets more ambient light changes — keep kernel large.
        """
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def _has_significant_blob(self, zone_mask) -> bool:
        """
        Check if mask has any blob large enough to be a train.
        Filters out: shadows, light flickers, small insects, hands
        (if MIN_DETECTION_AREA is set correctly for ceiling distance).
        """
        contours, _ = cv2.findContours(
            zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) >= config.MIN_DETECTION_AREA:
                return True
        return False

    def _annotate(self, frame, zone_states) -> np.ndarray:
        """Draw polygon zones and status overlay on the display frame."""
        h, w = frame.shape[:2]

        for key, zone in self._zones.items():
            active = zone_states.get(key, False)
            col    = zone.active_color if active else zone.safe_color

            # Semi-transparent polygon fill
            overlay = frame.copy()
            cv2.fillPoly(overlay, [zone.points], col)
            cv2.addWeighted(overlay, 0.20, frame, 0.80, 0, frame)

            # Polygon outline
            cv2.polylines(frame, [zone.points], True, col, 2)

            # Zone label
            cx = int(np.mean(zone.points[:, 0]))
            cy = int(np.mean(zone.points[:, 1]))
            status = "⚠ ACTIVE" if active else "clear"
            cv2.putText(frame, f"Zone {key} — {status}",
                        (cx - 40, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

        # Top status bar
        any_active = any(zone_states.values())
        bar_col = (0, 0, 130) if any_active else (0, 70, 0)
        cv2.rectangle(frame, (0, 0), (w, 38), bar_col, -1)

        if any_active:
            active_keys = [k for k, v in zone_states.items() if v]
            cv2.putText(frame,
                        f"COLLISION PREVENTION ACTIVE — Zone(s): "
                        f"{' '.join(active_keys)} — OUTER TRAIN STOPPED",
                        (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
        else:
            cv2.putText(frame, "MONITORING — All zones clear",
                        (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (180, 255, 180), 1)

        # Bottom strip
        cv2.rectangle(frame, (0, h - 24), (w, h), (20, 20, 20), -1)
        for i, (key, zone) in enumerate(self._zones.items()):
            col = zone.active_color if zone_states.get(key) else zone.safe_color
            cv2.putText(frame,
                        f"Zone {key}: {'LOCKED' if zone.locked else 'clear'}",
                        (10 + i * 220, h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, 1)

        return frame