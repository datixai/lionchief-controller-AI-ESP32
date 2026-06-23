# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  Drag-to-Select Train Tracker
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  CRITICAL DESIGN DECISION:
#    The tracker is initialized and updated on the DISPLAY-SIZED frame
#    (DISPLAY_W × DISPLAY_H = 960×540), NOT the raw camera frame.
#    The window is opened at exactly the same size (WINDOW_AUTOSIZE).
#    Therefore mouse coordinates == pixel coordinates in the frame.
#    No DPI scaling, no coordinate translation, no mismatch possible.
#
#  HOW TO SELECT:
#    Hold left mouse button, drag a box around the train, release.
#    The tracker locks onto the visual texture inside the box.
#    No stickers, no colors, works with any train appearance.
#
#  RE-SELECT:
#    Press A key → next drag re-selects Train A
#    Press B key → next drag re-selects Train B
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import time
import logging
from collections import deque

import config

logger = logging.getLogger("Tracker")

# State constants
WAIT_A   = "WAIT_A"
WAIT_B   = "WAIT_B"
TRACKING = "TRACKING"


def _make_tracker():
    """
    Create best available CSRT tracker.
    Tries all known API locations across OpenCV versions.
    CSRT is the most accurate tracker for small objects.
    """
    for fn in [
        lambda: cv2.legacy.TrackerCSRT_create(),
        lambda: cv2.legacy.TrackerKCF_create(),
        lambda: cv2.legacy.TrackerMOSSE_create(),
        lambda: cv2.TrackerCSRT_create(),
        lambda: cv2.TrackerKCF_create(),
    ]:
        try:
            return fn()
        except AttributeError:
            continue
    raise RuntimeError(
        "No OpenCV tracker found. Run: pip install opencv-contrib-python")


class TrainPosition:
    """Position of one detected train."""
    def __init__(self, x: int, y: int, bbox: tuple = None):
        self.x    = x
        self.y    = y
        self.bbox = bbox  # (x, y, w, h) in display pixels
        self.timestamp = time.time()

    def as_tuple(self):
        return (self.x, self.y)


class DragTracker:
    """
    Drag-to-select tracker for two trains.

    Works entirely in DISPLAY coordinates (960×540).
    The display frame is passed in via set_display_frame() and update().
    Mouse events arrive in the same coordinate space — no scaling needed.
    """

    def __init__(self):
        self._tkr_a     = None
        self._tkr_b     = None
        self._active_a  = False
        self._active_b  = False
        self._seen_a    = 0.0
        self._seen_b    = 0.0
        self._buf_a     = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self._buf_b     = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self.last_pos_a = None
        self.last_pos_b = None

        # Current selection state
        self.state      = WAIT_A

        # Drag gesture state
        self.drag_start  = None   # (x, y) on mouse down
        self.drag_end    = None   # (x, y) current mouse position
        self.is_dragging = False

        # Latest display-sized frame for tracker init
        self._display_frame = None

        # Flash confirmation timestamps
        self.flash_a_time = 0.0
        self.flash_b_time = 0.0

    # ── Properties ────────────────────────────────────────────────

    @property
    def ready(self) -> bool:
        return self._active_a and self._active_b

    @property
    def tracking_a(self) -> bool:
        return self._active_a

    @property
    def tracking_b(self) -> bool:
        return self._active_b

    # ── Frame supply ──────────────────────────────────────────────

    def set_display_frame(self, display_frame):
        """Store latest display frame for use on next click."""
        self._display_frame = display_frame.copy()

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

        # Build bounding box from drag
        x1 = min(self.drag_start[0], self.drag_end[0])
        y1 = min(self.drag_start[1], self.drag_end[1])
        bw = abs(self.drag_end[0] - self.drag_start[0])
        bh = abs(self.drag_end[1] - self.drag_start[1])

        if bw < config.MIN_BOX_SIZE or bh < config.MIN_BOX_SIZE:
            logger.debug(f"Drag too small ({bw}×{bh}px) — ignored")
            return

        # Clamp to display frame bounds
        x1 = max(0, x1)
        y1 = max(0, y1)
        bw = min(bw, config.DISPLAY_W - x1)
        bh = min(bh, config.DISPLAY_H - y1)

        self._init_tracker((x1, y1, bw, bh))

    # ── Manual re-select ──────────────────────────────────────────

    def reselect_a(self):
        self.state     = WAIT_A
        self._active_a = False
        self._tkr_a    = None
        self._buf_a.clear()
        logger.info("Waiting for Train A re-select")

    def reselect_b(self):
        self.state     = WAIT_B
        self._active_b = False
        self._tkr_b    = None
        self._buf_b.clear()
        logger.info("Waiting for Train B re-select")

    # ── Tracker init ──────────────────────────────────────────────

    def _init_tracker(self, bbox: tuple):
        """
        Initialize CSRT tracker for the pending train (A or B).
        bbox is in display coordinates — same as mouse coordinates.
        Tracker is initialized on the display frame — same coordinate space.
        """
        if self._display_frame is None:
            logger.warning("No display frame available yet")
            return

        x, y, bw, bh = bbox
        cx = x + bw // 2
        cy = y + bh // 2

        if self.state == WAIT_A:
            self._tkr_a = _make_tracker()
            ok = self._tkr_a.init(self._display_frame, bbox)
            if ok:
                self._active_a   = True
                self._seen_a     = time.time()
                self.flash_a_time = time.time()
                self._buf_a.clear()
                self.last_pos_a  = TrainPosition(cx, cy, bbox)
                self.state       = WAIT_B
                logger.info(f"Train A locked — bbox={bbox}")
            else:
                logger.error("Train A tracker init failed — try dragging again")

        elif self.state in (WAIT_B, TRACKING):
            self._tkr_b = _make_tracker()
            ok = self._tkr_b.init(self._display_frame, bbox)
            if ok:
                self._active_b   = True
                self._seen_b     = time.time()
                self.flash_b_time = time.time()
                self._buf_b.clear()
                self.last_pos_b  = TrainPosition(cx, cy, bbox)
                self.state       = TRACKING
                logger.info(f"Train B locked — bbox={bbox}")
            else:
                logger.error("Train B tracker init failed — try dragging again")

    # ── Update trackers ───────────────────────────────────────────

    def update(self, display_frame) -> tuple:
        """
        Run both trackers on the current display frame.

        Args:
            display_frame: BGR frame already resized to DISPLAY_W×DISPLAY_H.

        Returns:
            (pos_a, pos_b) — TrainPosition or None for each train.
        """
        # Store for use on next click
        self._display_frame = display_frame.copy()

        now   = time.time()
        pos_a = self._update_one(self._tkr_a, self._buf_a, "A",
                                 display_frame, now)
        pos_b = self._update_one(self._tkr_b, self._buf_b, "B",
                                 display_frame, now)

        if pos_a:
            self.last_pos_a = pos_a
            self._seen_a    = now
            self._active_a  = True
        if pos_b:
            self.last_pos_b = pos_b
            self._seen_b    = now
            self._active_b  = True

        return pos_a, pos_b

    def _update_one(self, tracker, buf, label, frame, now):
        if tracker is None:
            return None

        ok, raw_bbox = tracker.update(frame)
        if not ok:
            if label == "A":
                self._active_a = False
            else:
                self._active_b = False
            return None

        x, y, bw, bh = [int(v) for v in raw_bbox]
        cx = x + bw // 2
        cy = y + bh // 2
        buf.append((cx, cy))

        # Smooth position with rolling average
        sx = int(round(sum(p[0] for p in buf) / len(buf)))
        sy = int(round(sum(p[1] for p in buf) / len(buf)))

        return TrainPosition(sx, sy, (x, y, bw, bh))

    # ── Safety ────────────────────────────────────────────────────

    def is_a_missing(self) -> bool:
        return (not self._tkr_a or
                (time.time() - self._seen_a) > config.MISSING_TIMEOUT_S)

    def is_b_missing(self) -> bool:
        return (not self._tkr_b or
                (time.time() - self._seen_b) > config.MISSING_TIMEOUT_S)

    # ── Status text ───────────────────────────────────────────────

    def instruction_text(self) -> str:
        if self.state == WAIT_A:
            return "STEP 1 — HOLD and DRAG a box around  TRAIN A  (front train)"
        elif self.state == WAIT_B:
            return "STEP 2 — HOLD and DRAG a box around  TRAIN B  (rear BLE train)"
        else:
            a = "OK" if self._active_a else "LOST — press A to re-select"
            b = "OK" if self._active_b else "LOST — press B to re-select"
            return f"Tracking   A: {a}     B: {b}"


# ── Distance helper ───────────────────────────────────────────────

def pixel_distance(a, b) -> "float | None":
    """Euclidean distance between two TrainPosition centroids."""
    if a is None or b is None:
        return None
    return float(np.sqrt((a.x - b.x)**2 + (a.y - b.y)**2))