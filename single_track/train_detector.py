# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  Optical Flow Train Tracker
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  WHY OPTICAL FLOW INSTEAD OF CSRT:
#    CSRT memorises how the train LOOKS inside the box.
#    On a turn the train rotates → looks different → CSRT loses it.
#
#    Lucas-Kanade optical flow tracks WHERE individual texture
#    points on the train MOVED between frames.
#    Rotation does not matter — points are still there, just moved.
#    Works through turns, curves, speed changes.
#
#  HOW IT WORKS:
#    1. User drags a box around a train
#    2. cv2.goodFeaturesToTrack finds corner points inside the box
#    3. Every frame: cv2.calcOpticalFlowPyrLK finds where each point moved
#    4. Centre of surviving points = train position
#    5. If points drop below OF_MIN_POINTS → re-sample nearby
#    6. Every OF_RESAMPLE_EVERY frames → refresh points to prevent drift
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import time
import logging
from collections import deque

import config

logger = logging.getLogger("Tracker")

WAIT_A   = "WAIT_A"
WAIT_B   = "WAIT_B"
TRACKING = "TRACKING"

# Lucas-Kanade parameters — built once, reused every frame
LK_PARAMS = dict(
    winSize  = (config.OF_WIN_SIZE, config.OF_WIN_SIZE),
    maxLevel = config.OF_PYRAMID_LEVELS,
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
)

# Feature detector parameters
FEAT_PARAMS = dict(
    maxCorners   = config.OF_MAX_CORNERS,
    qualityLevel = config.OF_QUALITY_LEVEL,
    minDistance  = config.OF_MIN_DISTANCE,
    blockSize    = 7,
)


class TrainPosition:
    def __init__(self, x: int, y: int, bbox: tuple = None):
        self.x         = x
        self.y         = y
        self.bbox      = bbox
        self.timestamp = time.time()

    def as_tuple(self):
        return (self.x, self.y)


# ── Single train optical flow tracker ─────────────────────────────

class _FlowTracker:
    """
    Tracks one train using Lucas-Kanade sparse optical flow.
    Works through turns — tracks motion of individual points,
    not overall appearance of the bounding box.
    """

    def __init__(self, label: str):
        self.label       = label
        self._points     = None     # current tracked points (N,1,2) float32
        self._gray_prev  = None     # previous frame (grayscale)
        self._active     = False
        self._last_seen  = 0.0
        self._frame_count = 0       # counts frames since last resample
        self._buf        = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self.last_pos    = None

    @property
    def active(self) -> bool:
        return self._active

    @property
    def initialized(self) -> bool:
        return self._gray_prev is not None

    def init(self, gray_frame: np.ndarray, bbox: tuple) -> bool:
        """
        Sample feature points inside bbox and begin tracking.
        bbox = (x, y, w, h) in display coordinates.
        """
        x, y, bw, bh = [int(v) for v in bbox]

        # Crop ROI from grayscale frame
        roi = gray_frame[y: y + bh, x: x + bw]
        if roi.size == 0:
            logger.warning(f"[{self.label}] Empty ROI — try a bigger box")
            return False

        pts = cv2.goodFeaturesToTrack(roi, **FEAT_PARAMS)

        if pts is None or len(pts) < config.OF_MIN_POINTS:
            logger.warning(
                f"[{self.label}] Only {0 if pts is None else len(pts)} "
                f"feature points found — try drawing a bigger box or on "
                f"a more textured part of the train")
            return False

        # Shift from ROI-local to full-frame coordinates
        pts[:, :, 0] += x
        pts[:, :, 1] += y

        self._points      = pts.astype(np.float32)
        self._gray_prev   = gray_frame.copy()
        self._active      = True
        self._last_seen   = time.time()
        self._frame_count = 0
        self._buf.clear()

        cx = int(np.mean(pts[:, 0, 0]))
        cy = int(np.mean(pts[:, 0, 1]))
        self._buf.append((cx, cy))
        self.last_pos = TrainPosition(cx, cy, bbox)

        logger.info(f"[{self.label}] Optical flow init — "
                    f"{len(pts)} points at ({cx},{cy})")
        return True

    def update(self, gray_frame: np.ndarray) -> "TrainPosition | None":
        """
        Compute optical flow from previous frame to current frame.
        Returns TrainPosition on success, None if train is lost.
        """
        if not self._active or self._points is None or self._gray_prev is None:
            self._gray_prev = gray_frame.copy()
            return None

        self._frame_count += 1

        # ── Periodic resample to prevent point drift ──────────────
        # Every OF_RESAMPLE_EVERY frames, refresh feature points
        # around the current position so they stay on the train.
        if self._frame_count % config.OF_RESAMPLE_EVERY == 0:
            if self.last_pos:
                self._resample_near(gray_frame,
                                    self.last_pos.x, self.last_pos.y)

        # ── Compute optical flow ───────────────────────────────────
        new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._gray_prev, gray_frame, self._points, None, **LK_PARAMS)

        # Keep only successfully tracked points
        if new_pts is not None and status is not None:
            mask  = status.ravel() == 1
            good  = new_pts[mask]
        else:
            good = np.array([]).reshape(0, 2)

        # ── Too few points — try immediate resample ────────────────
        if len(good) < config.OF_MIN_POINTS:
            if self.last_pos:
                resampled = self._resample_near(
                    gray_frame, self.last_pos.x, self.last_pos.y)
                if resampled:
                    # Return last position — train temporarily lost
                    # but points have been refreshed for next frame
                    self._gray_prev = gray_frame.copy()
                    logger.debug(f"[{self.label}] Re-sampled after point loss")
                    return self.last_pos
            # Truly lost
            self._active    = False
            self._gray_prev = gray_frame.copy()
            logger.debug(f"[{self.label}] Lost — only {len(good)} points survived")
            return None

        # ── Good tracking ─────────────────────────────────────────
        pts_2d = good.reshape(-1, 2)
        cx = int(np.mean(pts_2d[:, 0]))
        cy = int(np.mean(pts_2d[:, 1]))

        # Rolling average for smooth position
        self._buf.append((cx, cy))
        sx = int(round(sum(p[0] for p in self._buf) / len(self._buf)))
        sy = int(round(sum(p[1] for p in self._buf) / len(self._buf)))

        # Estimate bounding box from point spread
        xs   = pts_2d[:, 0]
        ys   = pts_2d[:, 1]
        pad  = 15
        bx   = max(0, int(np.min(xs)) - pad)
        by   = max(0, int(np.min(ys)) - pad)
        bw   = int(np.max(xs) - np.min(xs)) + pad * 2
        bh   = int(np.max(ys) - np.min(ys)) + pad * 2

        self._points    = good.reshape(-1, 1, 2).astype(np.float32)
        self._gray_prev = gray_frame.copy()
        self._active    = True
        self._last_seen = time.time()
        self.last_pos   = TrainPosition(sx, sy, (bx, by, bw, bh))

        return self.last_pos

    def _resample_near(self, gray_frame: np.ndarray,
                       cx: int, cy: int) -> bool:
        """
        Sample new feature points in a circle around (cx, cy).
        Called when surviving points drop below OF_MIN_POINTS,
        and periodically to keep points fresh on the train.
        """
        h, w  = gray_frame.shape[:2]
        r     = config.OF_SEARCH_RADIUS
        x1    = max(0, cx - r)
        y1    = max(0, cy - r)
        x2    = min(w, cx + r)
        y2    = min(h, cy + r)
        roi   = gray_frame[y1:y2, x1:x2]

        if roi.size == 0:
            return False

        pts = cv2.goodFeaturesToTrack(roi, **FEAT_PARAMS)
        if pts is None or len(pts) < config.OF_MIN_POINTS:
            return False

        pts[:, :, 0] += x1
        pts[:, :, 1] += y1
        self._points = pts.astype(np.float32)
        self._active = True
        return True

    def is_missing(self) -> bool:
        return (time.time() - self._last_seen) > config.MISSING_TIMEOUT_S

    def reset(self):
        self._points      = None
        self._gray_prev   = None
        self._active      = False
        self._frame_count = 0
        self._buf.clear()
        self.last_pos     = None

    def get_points(self) -> "np.ndarray | None":
        """Return current tracked points for debug display."""
        return self._points


# ══════════════════════════════════════════════════════════════════
#  DragTracker — public interface used by main.py and calibrate.py
# ══════════════════════════════════════════════════════════════════

class DragTracker:
    """
    Drag-to-select two-train optical flow tracker.
    Interface identical to the old CSRT version — no changes in main.py.

    User holds mouse and drags a box around each train.
    Optical flow then tracks each train through turns and curves.
    """

    def __init__(self):
        self._tkr_a = _FlowTracker("A")
        self._tkr_b = _FlowTracker("B")

        self.state       = WAIT_A
        self.drag_start  = None
        self.drag_end    = None
        self.is_dragging = False

        self._display_frame = None   # latest display-sized BGR frame
        self._gray_frame    = None   # latest display-sized gray frame

        self.flash_a_time = 0.0
        self.flash_b_time = 0.0

    # ── Properties ────────────────────────────────────────────────

    @property
    def ready(self) -> bool:
        return self._tkr_a.active and self._tkr_b.active

    @property
    def tracking_a(self) -> bool:
        return self._tkr_a.active

    @property
    def tracking_b(self) -> bool:
        return self._tkr_b.active

    # ── Frame supply ──────────────────────────────────────────────

    def set_display_frame(self, display_frame):
        self._display_frame = display_frame.copy()
        self._gray_frame    = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)

    # ── Mouse events ──────────────────────────────────────────────

    def on_mouse_down(self, x: int, y: int):
        self.drag_start  = (x, y)
        self.drag_end    = (x, y)
        self.is_dragging = True

    def on_mouse_move(self, x: int, y: int):
        if self.is_dragging:
            self.drag_end = (x, y)

    def on_mouse_up(self, x: int, y: int):
        if not self.is_dragging or self.drag_start is None:
            return
        self.drag_end    = (x, y)
        self.is_dragging = False

        x1 = min(self.drag_start[0], self.drag_end[0])
        y1 = min(self.drag_start[1], self.drag_end[1])
        bw = abs(self.drag_end[0] - self.drag_start[0])
        bh = abs(self.drag_end[1] - self.drag_start[1])

        if bw < config.MIN_BOX_SIZE or bh < config.MIN_BOX_SIZE:
            logger.debug(f"Drag too small ({bw}×{bh}) — ignored")
            return

        x1 = max(0, x1)
        y1 = max(0, y1)
        bw = min(bw, config.DISPLAY_W - x1)
        bh = min(bh, config.DISPLAY_H - y1)
        self._init_tracker((x1, y1, bw, bh))

    # ── Tracker init ──────────────────────────────────────────────

    def _init_tracker(self, bbox: tuple):
        if self._gray_frame is None:
            logger.warning("No frame yet — drag ignored")
            return

        if self.state == WAIT_A:
            ok = self._tkr_a.init(self._gray_frame, bbox)
            if ok:
                self.flash_a_time = time.time()
                self.state = WAIT_B
                logger.info("Train A locked via optical flow")
            else:
                logger.warning("Train A init failed — drag a bigger box")

        elif self.state in (WAIT_B, TRACKING):
            ok = self._tkr_b.init(self._gray_frame, bbox)
            if ok:
                self.flash_b_time = time.time()
                self.state = TRACKING
                logger.info("Train B locked via optical flow")
            else:
                logger.warning("Train B init failed — drag a bigger box")

    # ── Re-select ─────────────────────────────────────────────────

    def reselect_a(self):
        self._tkr_a.reset()
        self.state = WAIT_A
        logger.info("Waiting for Train A re-select")

    def reselect_b(self):
        self._tkr_b.reset()
        self.state = WAIT_B
        logger.info("Waiting for Train B re-select")

    # ── Update ────────────────────────────────────────────────────

    def update(self, display_frame) -> tuple:
        """
        Run optical flow on current display frame.
        Returns (pos_a, pos_b) — TrainPosition or None each.
        """
        gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
        self._display_frame = display_frame.copy()
        self._gray_frame    = gray

        pos_a = self._tkr_a.update(gray) if self._tkr_a.initialized else None
        pos_b = self._tkr_b.update(gray) if self._tkr_b.initialized else None
        return pos_a, pos_b

    # ── Safety ────────────────────────────────────────────────────

    def is_a_missing(self) -> bool:
        return not self._tkr_a.initialized or self._tkr_a.is_missing()

    def is_b_missing(self) -> bool:
        return not self._tkr_b.initialized or self._tkr_b.is_missing()

    def get_points_a(self):
        return self._tkr_a.get_points()

    def get_points_b(self):
        return self._tkr_b.get_points()

    # ── Status ────────────────────────────────────────────────────

    def instruction_text(self) -> str:
        if self.state == WAIT_A:
            return "STEP 1 — HOLD and DRAG a box around  TRAIN A  (front train)"
        elif self.state == WAIT_B:
            return "STEP 2 — HOLD and DRAG a box around  TRAIN B  (rear BLE train)"
        else:
            a = "OK" if self._tkr_a.active else "LOST — press A to re-select"
            b = "OK" if self._tkr_b.active else "LOST — press B to re-select"
            return f"Tracking   A: {a}     B: {b}"


# ── Distance helper ───────────────────────────────────────────────

def pixel_distance(a, b) -> "float | None":
    if a is None or b is None:
        return None
    return float(np.sqrt((a.x - b.x)**2 + (a.y - b.y)**2))