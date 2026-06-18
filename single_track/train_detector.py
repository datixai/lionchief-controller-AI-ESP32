# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  Drag-to-Select Train Tracker
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  USER DRAWS A BOX around each train by holding and dragging.
#  OpenCV CSRT tracker then follows each train frame by frame.
#
#  No stickers. No colors. No calibration files needed.
#
#  SELECTION STATES:
#    WAIT_A  → waiting for user to drag a box around Train A
#    WAIT_B  → waiting for user to drag a box around Train B
#    TRACKING → both selected, auto-control running
#
#  RE-SELECT anytime:
#    Press A → drag new box around Train A
#    Press B → drag new box around Train B
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import time
import logging
from collections import deque

import config

logger = logging.getLogger("Tracker")

# Selection state constants
WAIT_A    = "WAIT_A"
WAIT_B    = "WAIT_B"
TRACKING  = "TRACKING"


class TrainPosition:
    def __init__(self, x: int, y: int, bbox: tuple = None):
        self.x         = x
        self.y         = y
        self.bbox      = bbox
        self.timestamp = time.time()

    def as_tuple(self):
        return (self.x, self.y)


class DragTracker:
    """
    Two-train drag-to-select tracker.

    User holds mouse, drags a rectangle around a train, releases.
    CSRT tracker is initialized on the selected region.
    Works regardless of train color or surrounding objects.
    """

    def __init__(self):
        # CSRT trackers
        self._tracker_a = None
        self._tracker_b = None

        # Active tracking flags
        self._active_a  = False
        self._active_b  = False

        # Last seen timestamps (for missing-train safety)
        self._last_seen_a = 0.0
        self._last_seen_b = 0.0

        # Position smoothing buffers
        self._buf_a = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self._buf_b = deque(maxlen=config.POSITION_SMOOTH_FRAMES)

        # Last known positions
        self.last_pos_a = None
        self.last_pos_b = None

        # Selection state
        self.state = WAIT_A

        # Drag state (set by mouse callback in main.py)
        self.drag_start  = None   # (x, y) when mouse pressed
        self.drag_end    = None   # (x, y) current mouse position
        self.is_dragging = False  # True while holding mouse button

        # Latest frame — needed to init tracker after drag
        self._latest_frame = None

    # ── Properties ────────────────────────────────────────────────

    @property
    def ready(self) -> bool:
        """True when both trains are selected and trackers running."""
        return self._active_a and self._active_b

    @property
    def tracking_a(self) -> bool:
        return self._active_a

    @property
    def tracking_b(self) -> bool:
        return self._active_b

    # ── Frame supply ──────────────────────────────────────────────

    def set_frame(self, frame):
        """Store latest frame — needed when drag completes."""
        self._latest_frame = frame.copy()

    # ── Mouse events (called from main.py callback) ───────────────

    def on_mouse_down(self, x: int, y: int):
        """User pressed mouse button — start drag."""
        self.drag_start  = (x, y)
        self.drag_end    = (x, y)
        self.is_dragging = True

    def on_mouse_move(self, x: int, y: int):
        """User moving mouse while holding — update drag rectangle."""
        if self.is_dragging:
            self.drag_end = (x, y)

    def on_mouse_up(self, x: int, y: int):
        """User released mouse — finalize drag and initialize tracker."""
        if not self.is_dragging or self.drag_start is None:
            return
        self.drag_end    = (x, y)
        self.is_dragging = False

        # Build bounding box from drag
        x1 = min(self.drag_start[0], self.drag_end[0])
        y1 = min(self.drag_start[1], self.drag_end[1])
        x2 = max(self.drag_start[0], self.drag_end[0])
        y2 = max(self.drag_start[1], self.drag_end[1])
        bw = x2 - x1
        bh = y2 - y1

        # Reject tiny boxes (accidental click without real drag)
        if bw < config.MIN_BOX_SIZE or bh < config.MIN_BOX_SIZE:
            logger.debug(f"Drag too small ({bw}x{bh}) — ignored")
            return

        bbox = (x1, y1, bw, bh)
        self._init_tracker(bbox)

    def _init_tracker(self, bbox: tuple):
        """Initialize CSRT tracker for the current state (A or B)."""
        if self._latest_frame is None:
            logger.warning("No frame available yet — drag ignored")
            return

        cx = int(bbox[0] + bbox[2] / 2)
        cy = int(bbox[1] + bbox[3] / 2)

        if self.state == WAIT_A:
            self._tracker_a = cv2.TrackerCSRT_create()
            ok = self._tracker_a.init(self._latest_frame, bbox)
            if ok:
                self._active_a    = True
                self._last_seen_a = time.time()
                self._buf_a.clear()
                self.last_pos_a   = TrainPosition(cx, cy, bbox)
                self.state        = WAIT_B
                logger.info(f"Train A selected — bbox={bbox}")
            else:
                logger.error("Train A tracker init failed")

        elif self.state == WAIT_B:
            self._tracker_b = cv2.TrackerCSRT_create()
            ok = self._tracker_b.init(self._latest_frame, bbox)
            if ok:
                self._active_b    = True
                self._last_seen_b = time.time()
                self._buf_b.clear()
                self.last_pos_b   = TrainPosition(cx, cy, bbox)
                self.state        = TRACKING
                logger.info(f"Train B selected — bbox={bbox}")
            else:
                logger.error("Train B tracker init failed")

    # ── Manual re-select ──────────────────────────────────────────

    def reselect_a(self):
        """Press A key → next drag resets Train A tracker."""
        self.state      = WAIT_A
        self._active_a  = False
        self._tracker_a = None
        self._buf_a.clear()
        logger.info("Re-select: drag a box around Train A")

    def reselect_b(self):
        """Press B key → next drag resets Train B tracker."""
        self.state      = WAIT_B
        self._active_b  = False
        self._tracker_b = None
        self._buf_b.clear()
        logger.info("Re-select: drag a box around Train B")

    # ── Update trackers ───────────────────────────────────────────

    def update(self, frame) -> tuple:
        """
        Run both CSRT trackers on the current frame.

        Returns:
            (pos_a, pos_b) — each is TrainPosition or None if lost.
        """
        self._latest_frame = frame.copy()
        now = time.time()

        pos_a = self._update_one(
            self._tracker_a, self._buf_a,
            self._active_a, "A", now)

        pos_b = self._update_one(
            self._tracker_b, self._buf_b,
            self._active_b, "B", now)

        if pos_a:
            self.last_pos_a   = pos_a
            self._last_seen_a = now
        if pos_b:
            self.last_pos_b   = pos_b
            self._last_seen_b = now

        return pos_a, pos_b

    def _update_one(self, tracker, buf, active, label, now):
        if not active or tracker is None:
            return None

        ok, bbox = tracker.update(self._latest_frame)

        if ok:
            x, y, bw, bh = [int(v) for v in bbox]
            cx = x + bw // 2
            cy = y + bh // 2
            buf.append((cx, cy))

            # Smooth position
            sx = int(round(sum(p[0] for p in buf) / len(buf)))
            sy = int(round(sum(p[1] for p in buf) / len(buf)))

            if label == "A":
                self._active_a = True
            else:
                self._active_b = True

            return TrainPosition(sx, sy, (x, y, bw, bh))
        else:
            if label == "A":
                self._active_a = False
            else:
                self._active_b = False
            return None

    # ── Safety checks ─────────────────────────────────────────────

    def is_a_missing(self) -> bool:
        if not self._tracker_a:
            return True
        return (time.time() - self._last_seen_a) > config.MISSING_TIMEOUT_S

    def is_b_missing(self) -> bool:
        if not self._tracker_b:
            return True
        return (time.time() - self._last_seen_b) > config.MISSING_TIMEOUT_S

    # ── Status text for display ───────────────────────────────────

    def instruction_text(self) -> str:
        if self.state == WAIT_A:
            return "DRAG a box around  TRAIN A  (front train)"
        elif self.state == WAIT_B:
            return "DRAG a box around  TRAIN B  (rear BLE train)"
        else:
            a = "OK" if self._active_a else "LOST — press A to reselect"
            b = "OK" if self._active_b else "LOST — press B to reselect"
            return f"Tracking  A:{a}   B:{b}"


# ── Distance helper ───────────────────────────────────────────────

def pixel_distance(a, b) -> "float | None":
    """Euclidean pixel distance between two TrainPosition centroids."""
    if a is None or b is None:
        return None
    dx = a.x - b.x
    dy = a.y - b.y
    return float(np.sqrt(dx * dx + dy * dy))
