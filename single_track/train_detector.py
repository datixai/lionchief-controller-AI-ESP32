# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  4-Box Local Search Tracker
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  USER SELECTS 4 BOXES:
#    A-HEAD → A-TAIL → B-HEAD → B-TAIL
#
#  EACH BOX TRACKED INDEPENDENTLY:
#    MOG2 background subtraction runs on full frame.
#    Each box only searches for blobs within SEARCH_RADIUS of
#    its last known position — people far away are ignored.
#
#  DIRECTION FROM GEOMETRY:
#    gap_b_chasing_a = distance(B_head, A_tail)
#    gap_a_chasing_b = distance(A_head, B_tail)
#    Whichever gap is smaller = that scenario is happening.
#    No velocity math — always geometrically correct.
#
#  IF ONE BOX OF A TRAIN IS LOST:
#    The other box + last known train length estimates the lost one.
#    Train keeps tracking with one box until it re-acquires both.
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import json
import os
import time
import logging
from collections import deque

import config

logger = logging.getLogger("Tracker")

# Selection states
WAIT_A_HEAD = "WAIT_A_HEAD"
WAIT_A_TAIL = "WAIT_A_TAIL"
WAIT_B_HEAD = "WAIT_B_HEAD"
WAIT_B_TAIL = "WAIT_B_TAIL"
TRACKING    = "TRACKING"


class TrainPosition:
    """Combined position of one train from its head and tail boxes."""
    def __init__(self, head: tuple, tail: tuple, center: tuple):
        self.head      = head    # (x, y) leading edge
        self.tail      = tail    # (x, y) trailing edge
        self.x         = center[0]
        self.y         = center[1]
        self.timestamp = time.time()

    def as_tuple(self):
        return (self.x, self.y)


# ── One box local tracker ─────────────────────────────────────────

class _BoxTracker:
    """
    Tracks a single drag-selected box using local MOG2 search.
    Only looks within SEARCH_RADIUS of its predicted position.
    Fast-moving people outside this radius are completely ignored.
    """

    def __init__(self, label: str):
        self.label     = label
        self._px       = None     # float x
        self._py       = None     # float y
        self._vx       = 0.0
        self._vy       = 0.0
        self._active   = False
        self._seen     = 0.0
        self._no_blob  = 0
        self._buf      = deque(maxlen=config.POSITION_SMOOTH_FRAMES)

    @property
    def active(self) -> bool:
        return self._active

    @property
    def initialized(self) -> bool:
        return self._px is not None

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

    def search_and_update(self, fg_full: np.ndarray):
        """
        Find the largest blob within SEARCH_RADIUS of predicted position.
        Updates position if found; uses velocity prediction if not.
        """
        if not self.initialized:
            return

        pred_x, pred_y = self._predict()
        h, w = fg_full.shape[:2]
        r    = config.SEARCH_RADIUS

        # Local bounding box clamped to frame
        x1 = max(0, int(pred_x - r))
        y1 = max(0, int(pred_y - r))
        x2 = min(w, int(pred_x + r))
        y2 = min(h, int(pred_y + r))

        local_fg = fg_full[y1:y2, x1:x2]
        if local_fg.size == 0:
            self._coast()
            return

        contours, _ = cv2.findContours(
            local_fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
            # Convert local → full frame coords
            cx = int(M["m10"] / M["m00"]) + x1
            cy = int(M["m01"] / M["m00"]) + y1
            best_area = area
            best_cx   = cx
            best_cy   = cy

        if best_cx is not None:
            self._update(best_cx, best_cy)
        else:
            self._coast()

    def _predict(self) -> tuple:
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
        # Gently decay velocity if coasting many frames
        if self._no_blob > 8:
            self._vx *= 0.92
            self._vy *= 0.92
        if self._no_blob > 20:
            self._active = False

    def get_pos(self) -> "tuple | None":
        """Return smoothed (x, y) or None."""
        if not self._active or not self._buf:
            return None
        sx = int(round(sum(p[0] for p in self._buf) / len(self._buf)))
        sy = int(round(sum(p[1] for p in self._buf) / len(self._buf)))
        return (sx, sy)

    def get_raw_pos(self) -> "tuple | None":
        if self._px is None:
            return None
        return (int(round(self._px)), int(round(self._py)))

    def is_missing(self) -> bool:
        return (time.time() - self._seen) > config.MISSING_TIMEOUT_S

    def reset(self):
        self._px = None; self._py = None
        self._vx = 0.0;  self._vy = 0.0
        self._active = False; self._no_blob = 0
        self._buf.clear()

    def draw_search_circle(self, frame, color):
        """Draw the local search area on the display frame."""
        pos = self.get_raw_pos()
        if pos:
            pred_x = int(self._px + self._vx)
            pred_y = int(self._py + self._vy)
            cv2.circle(frame, (pred_x, pred_y),
                       config.SEARCH_RADIUS, color, 1)


# ══════════════════════════════════════════════════════════════════
#  DragTracker — 4-box system
# ══════════════════════════════════════════════════════════════════

class DragTracker:
    """
    4-box drag-to-select tracker.
    A_HEAD, A_TAIL, B_HEAD, B_TAIL selected in sequence.
    Each box tracked independently in its local search radius.
    """

    def __init__(self):
        # MOG2 for full frame — used by all 4 boxes
        self._bg = cv2.createBackgroundSubtractorMOG2(
            history       = config.MOG2_HISTORY,
            varThreshold  = config.MOG2_VAR_THRESHOLD,
            detectShadows = False,
        )
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5))

        # Table boundary mask (blocks detection outside the table)
        self._table_mask = self._load_table_mask()

        # 4 independent boxes
        self._ah = _BoxTracker("A-HEAD")
        self._at = _BoxTracker("A-TAIL")
        self._bh = _BoxTracker("B-HEAD")
        self._bt = _BoxTracker("B-TAIL")

        # Selection state
        self.state       = WAIT_A_HEAD
        self.drag_start  = None
        self.drag_end    = None
        self.is_dragging = False
        self._latest_frame = None

        # Flash confirmation timestamps
        self.flash_ah = 0.0
        self.flash_at = 0.0
        self.flash_bh = 0.0
        self.flash_bt = 0.0

        # Last full-frame fg mask (for drawing)
        self._last_fg = None

    # ── Properties ────────────────────────────────────────────────

    @property
    def ready(self) -> bool:
        """True when all 4 boxes have been selected."""
        return (self._ah.initialized and self._at.initialized and
                self._bh.initialized and self._bt.initialized)

    @property
    def tracking_a(self) -> bool:
        return self._ah.active or self._at.active

    @property
    def tracking_b(self) -> bool:
        return self._bh.active or self._bt.active

    # ── Frame supply ──────────────────────────────────────────────

    def set_display_frame(self, frame):
        self._latest_frame = frame.copy()
        self._bg.apply(frame, learningRate=config.MOG2_LEARNING_RATE)

    # ── Mouse ─────────────────────────────────────────────────────

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

        bw = abs(self.drag_end[0] - self.drag_start[0])
        bh = abs(self.drag_end[1] - self.drag_start[1])
        if bw < config.MIN_BOX_SIZE or bh < config.MIN_BOX_SIZE:
            return

        cx = (self.drag_start[0] + self.drag_end[0]) // 2
        cy = (self.drag_start[1] + self.drag_end[1]) // 2
        self._assign(cx, cy)

    def _assign(self, cx: int, cy: int):
        if self.state == WAIT_A_HEAD:
            self._ah.init(cx, cy)
            self.flash_ah = time.time()
            self.state    = WAIT_A_TAIL
        elif self.state == WAIT_A_TAIL:
            self._at.init(cx, cy)
            self.flash_at = time.time()
            self.state    = WAIT_B_HEAD
        elif self.state == WAIT_B_HEAD:
            self._bh.init(cx, cy)
            self.flash_bh = time.time()
            self.state    = WAIT_B_TAIL
        elif self.state in (WAIT_B_TAIL, TRACKING):
            self._bt.init(cx, cy)
            self.flash_bt = time.time()
            self.state    = TRACKING

    # ── Re-select ─────────────────────────────────────────────────

    def reselect_a(self):
        """Press A to re-select both A boxes."""
        self._ah.reset()
        self._at.reset()
        self.state = WAIT_A_HEAD
        logger.info("Re-select Train A: drag HEAD then TAIL")

    def reselect_b(self):
        """Press B to re-select both B boxes."""
        self._bh.reset()
        self._bt.reset()
        self.state = WAIT_B_HEAD
        logger.info("Re-select Train B: drag HEAD then TAIL")

    # ── Table mask ────────────────────────────────────────────────

    def set_table_mask(self, mask):
        """Set table boundary mask (np.uint8 array, 255 inside table)."""
        self._table_mask = mask
        logger.info("Table mask applied to tracker")

    def has_table_mask(self) -> bool:
        return self._table_mask is not None

    def _load_table_mask(self):
        """Load saved table mask from file if it exists."""
        mask_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            config.TABLE_MASK_FILE)
        if not os.path.exists(mask_file):
            return None
        try:
            with open(mask_file) as f:
                data = json.load(f)
            pts  = np.array(data["points"], dtype=np.int32)
            mask = np.zeros(
                (config.DISPLAY_H, config.DISPLAY_W), dtype=np.uint8)
            cv2.fillPoly(mask, [pts], 255)
            logger.info(f"Table mask loaded — {len(pts)} points")
            return mask
        except Exception as e:
            logger.warning(f"Table mask load failed: {e}")
            return None

    # ── Update ────────────────────────────────────────────────────

    def update(self, display_frame) -> tuple:
        """
        Build MOG2 fg mask, run local search for each box.
        Returns (pos_a, pos_b) — TrainPosition or None.
        """
        self._latest_frame = display_frame.copy()

        # Full-frame MOG2
        fg = self._bg.apply(display_frame,
                            learningRate=config.MOG2_LEARNING_RATE)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN,  self._kernel)
        fg = cv2.dilate(fg,       self._kernel, iterations=1)

        # Apply table boundary mask — motion outside table is zeroed out
        if self._table_mask is not None:
            fg = cv2.bitwise_and(fg, self._table_mask)

        self._last_fg = fg

        # Update each box independently with local search
        for box in (self._ah, self._at, self._bh, self._bt):
            if box.initialized:
                box.search_and_update(fg)

        return self._make_pos_a(), self._make_pos_b()

    def _make_pos_a(self) -> "TrainPosition | None":
        head = self._ah.get_pos()
        tail = self._at.get_pos()
        return self._combine(head, tail, self._ah, self._at)

    def _make_pos_b(self) -> "TrainPosition | None":
        head = self._bh.get_pos()
        tail = self._bt.get_pos()
        return self._combine(head, tail, self._bh, self._bt)

    def _combine(self, head, tail, head_tkr, tail_tkr):
        """
        Combine head and tail into a TrainPosition.
        If one box is lost, estimate it from the other box + last known separation.
        """
        if head is None and tail is None:
            return None

        # Estimate missing point from the other
        if head is None and tail is not None:
            # Use raw (unsmoothed) head pos if available
            rh = head_tkr.get_raw_pos()
            head = rh if rh else tail   # fallback: head = tail
        if tail is None and head is not None:
            rt = tail_tkr.get_raw_pos()
            tail = rt if rt else head

        cx = (head[0] + tail[0]) // 2
        cy = (head[1] + tail[1]) // 2
        return TrainPosition(head, tail, (cx, cy))

    # ── Direction + gap ───────────────────────────────────────────

    def a_is_chasing_b(self, pos_a: "TrainPosition | None",
                        pos_b: "TrainPosition | None") -> bool:
        """
        Detect if Train A is BEHIND Train B using pure geometry.
        Compares gap_A_chasing_B vs gap_B_chasing_A.
        No velocity math — always correct from head/tail positions.
        """
        if pos_a is None or pos_b is None:
            return False
        # B chasing A: B_head → A_tail
        bh = pos_b.head
        at = pos_a.tail
        # A chasing B: A_head → B_tail
        ah = pos_a.head
        bt = pos_b.tail

        d_b_chasing = np.sqrt((bh[0]-at[0])**2 + (bh[1]-at[1])**2)
        d_a_chasing = np.sqrt((ah[0]-bt[0])**2 + (ah[1]-bt[1])**2)
        return d_a_chasing < d_b_chasing

    def facing_gap(self, pos_a: "TrainPosition | None",
                   pos_b: "TrainPosition | None",
                   a_chasing: bool) -> "float | None":
        """
        Gap between facing edges based on who is chasing whom.
        """
        if pos_a is None or pos_b is None:
            return None
        if a_chasing:
            # A behind B: gap = A_head to B_tail
            p1, p2 = pos_a.head, pos_b.tail
        else:
            # B behind A: gap = B_head to A_tail
            p1, p2 = pos_b.head, pos_a.tail
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return float(np.sqrt(dx*dx + dy*dy))

    # ── Debug ─────────────────────────────────────────────────────

    def get_boxes(self) -> dict:
        """Return all 4 box trackers for display."""
        return {"AH": self._ah, "AT": self._at,
                "BH": self._bh, "BT": self._bt}

    # ── Safety ────────────────────────────────────────────────────

    def is_a_missing(self) -> bool:
        return not (self._ah.initialized and self._at.initialized)

    def is_b_missing(self) -> bool:
        return not (self._bh.initialized and self._bt.initialized)

    # ── Status ────────────────────────────────────────────────────

    def instruction_text(self) -> str:
        instructions = {
            WAIT_A_HEAD: "STEP 1/4 — Drag box around  TRAIN A  HEAD  (front end of front train)",
            WAIT_A_TAIL: "STEP 2/4 — Drag box around  TRAIN A  TAIL  (rear end of front train)",
            WAIT_B_HEAD: "STEP 3/4 — Drag box around  TRAIN B  HEAD  (front end of rear BLE train)",
            WAIT_B_TAIL: "STEP 4/4 — Drag box around  TRAIN B  TAIL  (rear end of rear BLE train)",
            TRACKING:    f"Tracking — A:{'OK' if self.tracking_a else 'LOST (press A)'}  "
                         f"B:{'OK' if self.tracking_b else 'LOST (press B)'}",
        }
        return instructions.get(self.state, "")


# ── Distance helper ───────────────────────────────────────────────

def pixel_distance(a, b) -> "float | None":
    if a is None or b is None:
        return None
    return float(np.sqrt((a.x - b.x)**2 + (a.y - b.y)**2))