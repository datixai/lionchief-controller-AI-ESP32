# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  2-Box MOG2 Tracker with Tracking Circles
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  SELECTION (3 drags in order):
#    1. Drag rectangle around TABLE  → defines detection boundary
#    2. Drag box around TRAIN A      → starts tracking Train A
#    3. Drag box around TRAIN B      → starts tracking Train B
#
#  TRACKING:
#    Each train has one local search area (SEARCH_RADIUS).
#    MOG2 finds moving blobs only inside that radius.
#    A circle follows each train — size scales with blob size.
#    People outside the table boundary are invisible to detection.
#
#  DIRECTION:
#    Velocity dot-product determines if A is chasing B or vice versa.
# ══════════════════════════════════════════════════════════════════

import cv2
import json
import os
import numpy as np
import time
import logging
from collections import deque

import config

logger = logging.getLogger("Tracker")

# Selection states
WAIT_TABLE = "WAIT_TABLE"
WAIT_A     = "WAIT_A"
WAIT_B     = "WAIT_B"
TRACKING   = "TRACKING"


class TrainPosition:
    def __init__(self, x: int, y: int, radius: int = 25):
        self.x      = x
        self.y      = y
        self.radius = radius   # tracking circle radius (pixels)
        self.timestamp = time.time()

    def as_tuple(self):
        return (self.x, self.y)


# ── One-train local tracker ───────────────────────────────────────

class _TrainTracker:
    """
    Tracks one train with a local MOG2 blob search.
    Only looks for motion within SEARCH_RADIUS of last known position.
    Stores blob size to draw a correctly-sized tracking circle.
    """

    def __init__(self, label: str):
        self.label       = label
        self._px         = None   # float x position
        self._py         = None   # float y position
        self._vx         = 0.0   # velocity x
        self._vy         = 0.0   # velocity y
        self._active     = False
        self._seen       = 0.0
        self._no_blob    = 0
        self._buf        = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self._blob_area  = 400.0  # running estimate of blob size

    @property
    def active(self) -> bool:
        return self._active

    @property
    def initialized(self) -> bool:
        return self._px is not None

    @property
    def velocity(self):
        return (self._vx, self._vy)

    def init(self, cx: int, cy: int):
        self._px      = float(cx)
        self._py      = float(cy)
        self._vx      = 0.0
        self._vy      = 0.0
        self._active  = True
        self._seen    = time.time()
        self._no_blob = 0
        self._buf.clear()
        self._buf.append((cx, cy))
        logger.info(f"[{self.label}] init at ({cx},{cy})")

    def search_and_update(self, fg_masked: np.ndarray):
        """
        Find largest motion blob within SEARCH_RADIUS.
        fg_masked is the full-frame MOG2 foreground with table mask applied.
        """
        if not self.initialized:
            return

        pred_x, pred_y = self._predict()
        h, w = fg_masked.shape[:2]
        r    = config.SEARCH_RADIUS

        x1 = max(0, int(pred_x - r))
        y1 = max(0, int(pred_y - r))
        x2 = min(w, int(pred_x + r))
        y2 = min(h, int(pred_y + r))

        local = fg_masked[y1:y2, x1:x2]
        if local.size == 0:
            self._coast()
            return

        contours, _ = cv2.findContours(
            local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_area = config.MIN_BLOB_AREA
        best_cx   = None
        best_cy   = None

        for c in contours:
            area = cv2.contourArea(c)
            if area < best_area:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"]) + x1
            cy = int(M["m01"] / M["m00"]) + y1
            best_area = area
            best_cx   = cx
            best_cy   = cy

        if best_cx is not None:
            # Update blob size estimate (slow moving average)
            self._blob_area = 0.9 * self._blob_area + 0.1 * best_area
            self._update(best_cx, best_cy)
        else:
            self._coast()

    def _predict(self):
        return (self._px + self._vx, self._py + self._vy)

    def _update(self, cx: int, cy: int):
        if self._px is not None:
            dx = cx - self._px
            dy = cy - self._py
            a  = config.VELOCITY_ALPHA
            self._vx = a * dx + (1.0 - a) * self._vx
            self._vy = a * dy + (1.0 - a) * self._vy
        self._px      = float(cx)
        self._py      = float(cy)
        self._active  = True
        self._seen    = time.time()
        self._no_blob = 0
        self._buf.append((cx, cy))

    def _coast(self):
        """Advance by velocity when no blob found."""
        if self._px is None:
            return
        self._px      += self._vx
        self._py      += self._vy
        self._no_blob += 1
        if self._no_blob > 8:
            self._vx *= 0.92
            self._vy *= 0.92
        if self._no_blob > 25:
            self._active = False

    def get_position(self) -> "TrainPosition | None":
        if not self._active or not self._buf:
            return None
        sx = int(round(sum(p[0] for p in self._buf) / len(self._buf)))
        sy = int(round(sum(p[1] for p in self._buf) / len(self._buf)))
        # Circle radius scaled from blob area
        r = int(np.sqrt(self._blob_area / np.pi) * 1.6)
        r = max(config.CIRCLE_RADIUS_MIN, min(config.CIRCLE_RADIUS_MAX, r))
        return TrainPosition(sx, sy, r)

    def get_raw_pos(self):
        if self._px is None:
            return None
        return (int(round(self._px)), int(round(self._py)))

    def draw_search_area(self, frame, color):
        """Draw local search circle on frame (subtle, for debug)."""
        pos = self.get_raw_pos()
        if pos:
            px = int(self._px + self._vx)
            py = int(self._py + self._vy)
            cv2.circle(frame, (px, py),
                       config.SEARCH_RADIUS, color, 1)

    def is_missing(self) -> bool:
        return (time.time() - self._seen) > config.MISSING_TIMEOUT_S

    def reset(self):
        self._px = None; self._py = None
        self._vx = 0.0;  self._vy = 0.0
        self._active = False
        self._no_blob = 0
        self._buf.clear()


# ══════════════════════════════════════════════════════════════════
#  DragTracker — 3-step selection: table → Train A → Train B
# ══════════════════════════════════════════════════════════════════

class DragTracker:
    """
    Simplified 2-box tracker with table boundary.
    User drags 3 boxes in order: TABLE, TRAIN A, TRAIN B.
    Each train tracked by local MOG2 search with tracking circle display.
    """

    def __init__(self):
        self._bg = cv2.createBackgroundSubtractorMOG2(
            history       = config.MOG2_HISTORY,
            varThreshold  = config.MOG2_VAR_THRESHOLD,
            detectShadows = False,
        )
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5))

        self._tkr_a = _TrainTracker("A")
        self._tkr_b = _TrainTracker("B")

        # Table boundary mask
        self._table_mask = None
        self._table_rect = None   # (x1, y1, x2, y2) in display coords

        # Drag state
        self.state       = WAIT_TABLE
        self.drag_start  = None
        self.drag_end    = None
        self.is_dragging = False

        # Flash timestamps
        self.flash_table = 0.0
        self.flash_a     = 0.0
        self.flash_b     = 0.0

        # Try loading saved table mask
        self._load_table_mask()

    # ── Properties ────────────────────────────────────────────────

    @property
    def ready(self) -> bool:
        return self._tkr_a.initialized and self._tkr_b.initialized

    @property
    def tracking_a(self) -> bool:
        return self._tkr_a.active

    @property
    def tracking_b(self) -> bool:
        return self._tkr_b.active

    @property
    def has_table_mask(self) -> bool:
        return self._table_mask is not None

    # ── Frame supply ──────────────────────────────────────────────

    def set_display_frame(self, frame):
        self._bg.apply(frame, learningRate=config.MOG2_LEARNING_RATE)

    # ── Mouse events ──────────────────────────────────────────────

    def on_mouse_down(self, x, y):
        self.drag_start  = (x, y)
        self.drag_end    = (x, y)
        self.is_dragging = True

    def on_mouse_move(self, x, y):
        if self.is_dragging:
            self.drag_end = (x, y)

    def on_mouse_up(self, x, y):
        if not self.is_dragging or self.drag_start is None:
            return
        self.drag_end    = (x, y)
        self.is_dragging = False

        bw = abs(self.drag_end[0] - self.drag_start[0])
        bh = abs(self.drag_end[1] - self.drag_start[1])
        if bw < config.MIN_BOX_SIZE or bh < config.MIN_BOX_SIZE:
            return

        cx = (self.drag_start[0] + self.drag_end[0]) // 2
        cy = (self.drag_start[1] + self.drag_end[1]) // 2

        x1 = min(self.drag_start[0], self.drag_end[0])
        y1 = min(self.drag_start[1], self.drag_end[1])
        x2 = max(self.drag_start[0], self.drag_end[0])
        y2 = max(self.drag_start[1], self.drag_end[1])

        if self.state == WAIT_TABLE:
            self._set_table_rect(x1, y1, x2, y2)
            self.flash_table = time.time()
            self.state       = WAIT_A
            logger.info(f"Table boundary set: ({x1},{y1}) → ({x2},{y2})")

        elif self.state == WAIT_A:
            self._tkr_a.init(cx, cy)
            self.flash_a = time.time()
            self.state   = WAIT_B
            logger.info(f"Train A selected at ({cx},{cy})")

        elif self.state in (WAIT_B, TRACKING):
            self._tkr_b.init(cx, cy)
            self.flash_b = time.time()
            self.state   = TRACKING
            logger.info(f"Train B selected at ({cx},{cy})")

    # ── Table mask ────────────────────────────────────────────────

    def _set_table_rect(self, x1: int, y1: int, x2: int, y2: int):
        self._table_rect = (x1, y1, x2, y2)
        mask = np.zeros((config.DISPLAY_H, config.DISPLAY_W), dtype=np.uint8)
        cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
        self._table_mask = mask
        # Save to file
        mask_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            config.TABLE_MASK_FILE)
        with open(mask_file, "w") as f:
            json.dump({"rect": [x1, y1, x2, y2]}, f)
        logger.info("Table mask saved")

    def _load_table_mask(self):
        mask_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            config.TABLE_MASK_FILE)
        if not os.path.exists(mask_file):
            return
        try:
            with open(mask_file) as f:
                data = json.load(f)
            x1, y1, x2, y2 = data["rect"]
            self._table_rect = (x1, y1, x2, y2)
            mask = np.zeros(
                (config.DISPLAY_H, config.DISPLAY_W), dtype=np.uint8)
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
            self._table_mask = mask
            self.state = WAIT_A   # table already set — skip to train selection
            logger.info(f"Table mask loaded from file")
        except Exception as e:
            logger.warning(f"Table mask load failed: {e}")

    def redraw_table(self):
        """Press T to re-draw the table boundary."""
        self._table_mask = None
        self._table_rect = None
        self.state       = WAIT_TABLE

    # ── Re-select trains ──────────────────────────────────────────

    def reselect_a(self):
        self._tkr_a.reset()
        self.state = WAIT_A

    def reselect_b(self):
        self._tkr_b.reset()
        self.state = WAIT_B

    # ── Update ────────────────────────────────────────────────────

    def update(self, display_frame) -> tuple:
        """
        Build MOG2 fg with table mask, run local search per train.
        Returns (pos_a, pos_b).
        """
        fg = self._bg.apply(display_frame,
                            learningRate=config.MOG2_LEARNING_RATE)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, self._kernel)
        fg = cv2.dilate(fg, self._kernel, iterations=1)

        # Apply table mask — zero out everything outside the table
        if self._table_mask is not None:
            fg = cv2.bitwise_and(fg, self._table_mask)

        if self._tkr_a.initialized:
            self._tkr_a.search_and_update(fg)
        if self._tkr_b.initialized:
            self._tkr_b.search_and_update(fg)

        return self._tkr_a.get_position(), self._tkr_b.get_position()

    # ── Direction detection ───────────────────────────────────────

    def a_is_chasing_b(self, pos_a, pos_b) -> bool:
        """
        True when Train A is BEHIND Train B and heading toward it.
        Uses velocity dot product — no head/tail needed.
        """
        if pos_a is None or pos_b is None:
            return False
        avx, avy = self._tkr_a.velocity
        a_spd    = np.sqrt(avx**2 + avy**2)
        if a_spd < 0.3:
            return False
        # Vector from A toward B
        dx  = pos_b.x - pos_a.x
        dy  = pos_b.y - pos_a.y
        dot = avx * dx + avy * dy
        return dot > 0

    def facing_gap(self, pos_a, pos_b,
                   a_chasing: bool) -> "float | None":
        """
        Centre-to-centre gap minus both radii = gap between surfaces.
        """
        if pos_a is None or pos_b is None:
            return None
        dx   = pos_a.x - pos_b.x
        dy   = pos_a.y - pos_b.y
        dist = float(np.sqrt(dx*dx + dy*dy))
        # Subtract estimated train radii so gap is surface-to-surface
        gap  = dist - pos_a.radius - pos_b.radius
        return max(0.0, gap)

    # ── Safety ────────────────────────────────────────────────────

    def is_a_missing(self) -> bool:
        return not self._tkr_a.initialized or self._tkr_a.is_missing()

    def is_b_missing(self) -> bool:
        return not self._tkr_b.initialized or self._tkr_b.is_missing()

    # ── Status text ───────────────────────────────────────────────

    def instruction_text(self) -> str:
        msgs = {
            WAIT_TABLE: "STEP 1 — DRAG a box around the TABLE  (whole track area)",
            WAIT_A:     "STEP 2 — DRAG a box around  TRAIN A  (front train)",
            WAIT_B:     "STEP 3 — DRAG a box around  TRAIN B  (rear BLE train)",
            TRACKING:   (f"Tracking — A:{'OK' if self._tkr_a.active else 'LOST (press A)'}  "
                         f"B:{'OK' if self._tkr_b.active else 'LOST (press B)'}"),
        }
        return msgs.get(self.state, "")


# ── Distance helper ───────────────────────────────────────────────

def pixel_distance(a, b) -> "float | None":
    if a is None or b is None:
        return None
    return float(np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2))