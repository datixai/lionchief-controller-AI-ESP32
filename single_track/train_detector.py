# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  Click-to-Track Train Detector
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  No stickers. No colors. No calibration.
#  User clicks on each train in the live camera feed.
#  OpenCV CSRT tracker follows the texture/shape of each train.
#
#  CSRT (Channel and Spatial Reliability Tracker) is the most
#  accurate OpenCV built-in tracker for small objects. It works
#  on the visual pattern under the bounding box — completely
#  immune to other colored objects in the room.
#
#  USAGE:
#    detector = ClickTracker()
#    detector.click(frame, x, y)     # assign next train at (x,y)
#    pos_a, pos_b = detector.update(frame)  # call every frame
#    detector.reassign_a()           # next click → Train A
#    detector.reassign_b()           # next click → Train B
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import time
import logging
from collections import deque

import config

logger = logging.getLogger("ClickTracker")


class TrainPosition:
    """Holds one train's current detected position."""

    def __init__(self, x: int, y: int, bbox: tuple = None):
        self.x         = x
        self.y         = y
        self.bbox      = bbox       # (x, y, w, h) from tracker
        self.timestamp = time.time()

    def as_tuple(self):
        return (self.x, self.y)

    def __repr__(self):
        return f"TrainPos(x={self.x}, y={self.y})"


class _SingleTracker:
    """
    Wraps a CSRT tracker for one train.
    Handles init, update, loss detection and position smoothing.
    """

    def __init__(self, label: str):
        self.label      = label    # "A" or "B"
        self._tracker   = None
        self._active    = False    # True when tracker is running
        self._last_seen = 0.0
        self._pos_buf   = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self.last_pos   = None     # last known TrainPosition

    @property
    def active(self) -> bool:
        return self._active

    @property
    def initialized(self) -> bool:
        return self._tracker is not None

    def init(self, frame, click_x: int, click_y: int):
        """
        Start tracking at click position.
        Creates a bounding box centered on the click.
        """
        half  = config.CLICK_BOX_SIZE // 2
        h, w  = frame.shape[:2]
        bx    = max(0, click_x - half)
        by    = max(0, click_y - half)
        bw    = min(config.CLICK_BOX_SIZE, w - bx)
        bh    = min(config.CLICK_BOX_SIZE, h - by)
        bbox  = (bx, by, bw, bh)

        self._tracker = cv2.TrackerCSRT_create()
        ok = self._tracker.init(frame, bbox)

        if ok:
            self._active    = True
            self._last_seen = time.time()
            self._pos_buf.clear()
            cx = bx + bw // 2
            cy = by + bh // 2
            self.last_pos = TrainPosition(cx, cy, bbox)
            self._pos_buf.append((cx, cy))
            logger.info(
                f"[Train {self.label}] Tracker initialized at "
                f"({click_x},{click_y}) bbox={bbox}")
        else:
            logger.error(
                f"[Train {self.label}] Tracker init failed at ({click_x},{click_y})")
            self._active  = False
            self._tracker = None

        return ok

    def update(self, frame) -> "TrainPosition | None":
        """
        Update tracker on new frame.
        Returns TrainPosition on success, None if tracker lost.
        """
        if not self._tracker:
            return None

        ok, bbox = self._tracker.update(frame)

        if ok:
            x, y, bw, bh = [int(v) for v in bbox]
            cx = x + bw // 2
            cy = y + bh // 2

            self._pos_buf.append((cx, cy))
            self._last_seen = time.time()
            self._active    = True

            # Smooth position via rolling average
            sx = int(round(sum(p[0] for p in self._pos_buf) / len(self._pos_buf)))
            sy = int(round(sum(p[1] for p in self._pos_buf) / len(self._pos_buf)))

            self.last_pos = TrainPosition(sx, sy, (x, y, bw, bh))
            return self.last_pos
        else:
            # Tracker lost this frame
            self._active = False
            logger.debug(f"[Train {self.label}] Tracker lost frame")
            return None

    def is_missing(self) -> bool:
        """True if tracker has not succeeded for MISSING_TIMEOUT_S."""
        return (time.time() - self._last_seen) > config.MISSING_TIMEOUT_S

    def reset(self):
        self._tracker   = None
        self._active    = False
        self._pos_buf.clear()
        self.last_pos   = None


# ── Assign state ─────────────────────────────────────────────────

class AssignMode:
    A    = "A"     # next click → Train A
    B    = "B"     # next click → Train B
    DONE = "DONE"  # both assigned, tracking running


class ClickTracker:
    """
    Two-train click-to-track system.

    Workflow:
      1. System starts in ASSIGN_A mode → guides Peter to click Train A
      2. After Train A click → switches to ASSIGN_B mode
      3. After Train B click → switches to DONE / tracking mode
      4. Peter can press A or B at any time to re-assign a train
         (e.g. if tracker drifts or train was out of view)
    """

    def __init__(self):
        self._tracker_a  = _SingleTracker("A")
        self._tracker_b  = _SingleTracker("B")
        self._mode       = AssignMode.A   # starts waiting for Train A click
        self._last_frame = None           # stored to init tracker on click

    # ── Public API ────────────────────────────────────────────────

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def ready(self) -> bool:
        """True when both trains have been clicked and trackers running."""
        return (self._tracker_a.initialized and
                self._tracker_b.initialized)

    @property
    def tracking_a(self) -> bool:
        return self._tracker_a.active

    @property
    def tracking_b(self) -> bool:
        return self._tracker_b.active

    def set_frame(self, frame):
        """Store latest frame — needed to initialize tracker on click."""
        self._last_frame = frame.copy()

    def handle_click(self, x: int, y: int) -> str:
        """
        Handle a mouse click at (x, y).
        Assigns the click to Train A or B based on current mode.

        Returns:
            "A", "B" — which train was just assigned
            "ignored" — click happened in DONE mode (no reassign pending)
        """
        if self._last_frame is None:
            logger.warning("Click ignored — no frame available yet")
            return "ignored"

        if self._mode == AssignMode.A:
            ok = self._tracker_a.init(self._last_frame, x, y)
            if ok:
                logger.info(f"Train A assigned at ({x},{y})")
                # Auto-advance to B if B not yet set
                if not self._tracker_b.initialized:
                    self._mode = AssignMode.B
                else:
                    self._mode = AssignMode.DONE
            return "A"

        elif self._mode == AssignMode.B:
            ok = self._tracker_b.init(self._last_frame, x, y)
            if ok:
                logger.info(f"Train B assigned at ({x},{y})")
                self._mode = AssignMode.DONE
            return "B"

        else:
            # DONE mode — click ignored unless user pressed A/B key first
            return "ignored"

    def reassign_a(self):
        """Pressing A key → next click will re-initialize Train A tracker."""
        self._mode = AssignMode.A
        logger.info("Re-assign mode: Train A — click on front train")

    def reassign_b(self):
        """Pressing B key → next click will re-initialize Train B tracker."""
        self._mode = AssignMode.B
        logger.info("Re-assign mode: Train B — click on rear train")

    def update(self, frame) -> tuple:
        """
        Run both trackers on current frame.

        Returns:
            (pos_a, pos_b)
            Each is TrainPosition or None if tracker lost / not initialized.
        """
        self._last_frame = frame.copy()

        pos_a = self._tracker_a.update(frame) if self._tracker_a.initialized else None
        pos_b = self._tracker_b.update(frame) if self._tracker_b.initialized else None

        return pos_a, pos_b

    def is_train_a_missing(self) -> bool:
        if not self._tracker_a.initialized:
            return True
        return self._tracker_a.is_missing()

    def is_train_b_missing(self) -> bool:
        if not self._tracker_b.initialized:
            return True
        return self._tracker_b.is_missing()

    def get_last_a(self) -> "TrainPosition | None":
        return self._tracker_a.last_pos

    def get_last_b(self) -> "TrainPosition | None":
        return self._tracker_b.last_pos

    def status_text(self) -> str:
        """Human-readable status for display overlay."""
        if self._mode == AssignMode.A:
            return "Click on TRAIN A  (front train)"
        elif self._mode == AssignMode.B:
            return "Click on TRAIN B  (rear train)"
        else:
            a = "✅" if self._tracker_a.active else "❌ lost"
            b = "✅" if self._tracker_b.active else "❌ lost"
            return f"Tracking  A:{a}  B:{b}   Press A/B to re-click"


# ── Distance helper ───────────────────────────────────────────────

def pixel_distance(a: "TrainPosition | None",
                   b: "TrainPosition | None") -> "float | None":
    """Euclidean pixel distance between two train centroids."""
    if a is None or b is None:
        return None
    dx = a.x - b.x
    dy = a.y - b.y
    return float(np.sqrt(dx * dx + dy * dy))
