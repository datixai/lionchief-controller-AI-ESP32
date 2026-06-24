# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  MOG2 Motion Tracker with Head/Tail + Track Mask
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  IMPROVEMENTS IN THIS VERSION:
#    1. TRACK MASK — polygon drawn around track in calibrate.py.
#       MOG2 detection only runs INSIDE the mask.
#       People walking outside the track are completely ignored.
#
#    2. HEAD / TAIL BOXES — two boxes per train derived automatically
#       from blob bounding box + velocity direction each frame.
#       Gap calculation uses the FACING EDGES (tail-to-head distance)
#       not center-to-center — much more accurate stopping distance.
#
#    3. DIRECTION DETECTION — detects when Train A is BEHIND Train B
#       (wrap-around on the loop). Returns a flag so speed_controller
#       can tell Train B to speed up instead of slow down.
#
#    4. MOG2 BLOB DETECTION (unchanged — it was working well).
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

WAIT_A   = "WAIT_A"
WAIT_B   = "WAIT_B"
TRACKING = "TRACKING"

HEAD_BOX  = 18   # size of displayed head/tail indicator boxes (pixels)


class TrainPosition:
    def __init__(self, x: int, y: int, bbox: tuple = None,
                 head: tuple = None, tail: tuple = None,
                 vel: tuple = (0.0, 0.0)):
        self.x    = x
        self.y    = y
        self.bbox = bbox     # full bounding box  (x,y,w,h)
        self.head = head     # leading edge point (x,y)  — direction of travel
        self.tail = tail     # trailing edge point (x,y)
        self.vel  = vel      # (vx, vy) velocity in pixels/frame
        self.timestamp = time.time()

    def as_tuple(self):
        return (self.x, self.y)


# ── Track mask loader ─────────────────────────────────────────────

def _load_track_mask(display_w: int, display_h: int):
    """
    Load track boundary polygon from track_mask.json.
    Returns a binary mask (np.uint8, 255 inside track, 0 outside).
    Returns None if no mask file exists — detection covers full frame.
    """
    mask_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), config.TRACK_MASK_FILE)
    if not os.path.exists(mask_file):
        return None
    try:
        with open(mask_file) as f:
            data = json.load(f)
        pts = np.array(data["points"], dtype=np.int32)
        mask = np.zeros((display_h, display_w), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        logger.info(f"Track mask loaded — {len(pts)} polygon points")
        return mask
    except Exception as e:
        logger.warning(f"Track mask load failed: {e}")
        return None


# ── Per-train velocity tracker ────────────────────────────────────

class _VelocityTracker:

    def __init__(self, label: str):
        self.label      = label
        self._px        = None
        self._py        = None
        self._vx        = 0.0
        self._vy        = 0.0
        self._active    = False
        self._last_seen = 0.0
        self._last_bbox = None
        self._buf       = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self._no_blob   = 0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def initialized(self) -> bool:
        return self._px is not None

    @property
    def velocity(self):
        return (self._vx, self._vy)

    def set_initial_position(self, x: int, y: int):
        self._px        = float(x)
        self._py        = float(y)
        self._vx        = 0.0
        self._vy        = 0.0
        self._active    = True
        self._last_seen = time.time()
        self._no_blob   = 0
        self._buf.clear()
        self._buf.append((x, y))
        logger.info(f"[{self.label}] Initial position ({x},{y})")

    def predict_next(self):
        if self._px is None:
            return 0.0, 0.0
        return (self._px + self._vx, self._py + self._vy)

    def update_from_blob(self, cx: int, cy: int, bbox: tuple):
        if self._px is not None:
            dx = cx - self._px
            dy = cy - self._py
            a  = config.VELOCITY_ALPHA
            self._vx = a * dx + (1.0 - a) * self._vx
            self._vy = a * dy + (1.0 - a) * self._vy
        self._px        = float(cx)
        self._py        = float(cy)
        self._last_bbox = bbox
        self._last_seen = time.time()
        self._active    = True
        self._no_blob   = 0
        self._buf.append((cx, cy))

    def predict_update(self):
        if self._px is None:
            return
        self._px   += self._vx
        self._py   += self._vy
        self._no_blob += 1
        if self._no_blob > 10:
            self._vx *= 0.9
            self._vy *= 0.9

    def is_missing(self) -> bool:
        return (time.time() - self._last_seen) > config.MISSING_TIMEOUT_S

    def reset(self):
        self._px = None; self._py = None
        self._vx = 0.0;  self._vy = 0.0
        self._active = False
        self._last_bbox = None
        self._buf.clear()
        self._no_blob = 0

    def get_position(self) -> "TrainPosition | None":
        if not self._active or self._px is None:
            return None

        # Smoothed centre
        if self._buf:
            sx = int(round(sum(p[0] for p in self._buf) / len(self._buf)))
            sy = int(round(sum(p[1] for p in self._buf) / len(self._buf)))
        else:
            sx = int(round(self._px))
            sy = int(round(self._py))

        # Full bounding box
        if self._last_bbox:
            bx, by, bw, bh = self._last_bbox
            # Re-centre bbox on smoothed position
            bx = sx - bw // 2
            by = sy - bh // 2
            bbox = (bx, by, bw, bh)
        else:
            s    = 25
            bbox = (sx - s, sy - s, s * 2, s * 2)

        # Head / tail derived from velocity direction
        head, tail = self._head_tail(sx, sy)

        return TrainPosition(sx, sy, bbox, head, tail,
                             (self._vx, self._vy))

    def _head_tail(self, cx: int, cy: int):
        """
        Derive head (leading edge) and tail (trailing edge) positions
        from current velocity direction.
        Returns (head_pt, tail_pt) or (None, None) if not moving.
        """
        speed = np.sqrt(self._vx ** 2 + self._vy ** 2)
        if speed < 0.4:   # too slow to determine direction reliably
            return None, None

        nvx = self._vx / speed
        nvy = self._vy / speed

        # Use bbox extent or a fixed reach
        if self._last_bbox:
            reach = max(self._last_bbox[2], self._last_bbox[3]) // 2
        else:
            reach = 25

        reach = max(reach, 20)
        hx = int(cx + nvx * reach)
        hy = int(cy + nvy * reach)
        tx = int(cx - nvx * reach)
        ty = int(cy - nvy * reach)
        return (hx, hy), (tx, ty)


# ══════════════════════════════════════════════════════════════════
#  DragTracker — public interface
# ══════════════════════════════════════════════════════════════════

class DragTracker:
    """
    Drag-to-select two-train MOG2 tracker with:
      • Track mask  — ignores motion outside the track boundary
      • Head / tail — shows leading and trailing edge of each train
      • Direction   — detects when Train A is behind Train B
    """

    def __init__(self):
        self._bg = cv2.createBackgroundSubtractorMOG2(
            history       = config.MOG2_HISTORY,
            varThreshold  = config.MOG2_VAR_THRESHOLD,
            detectShadows = False,
        )
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        # Track mask (loaded once; None = no mask = full frame)
        self._mask = _load_track_mask(config.DISPLAY_W, config.DISPLAY_H)
        if self._mask is not None:
            logger.info("Track mask active — motion outside track ignored")
        else:
            logger.info("No track mask — draw one in calibrate.py (press M) "
                        "to ignore people moving around the layout")

        self._tkr_a = _VelocityTracker("A")
        self._tkr_b = _VelocityTracker("B")

        self.state       = WAIT_A
        self.drag_start  = None
        self.drag_end    = None
        self.is_dragging = False
        self._latest_frame = None

        self.flash_a_time = 0.0
        self.flash_b_time = 0.0
        self._last_blobs  = []

        # Direction flag — updated each frame
        self._a_is_chasing_b = False

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

    @property
    def a_is_chasing_b(self) -> bool:
        """
        True when Train A (manual, front) is BEHIND Train B (BLE, rear)
        and heading toward it. Speed controller uses this to speed B up
        instead of slowing it down.
        """
        return self._a_is_chasing_b

    # ── Frame supply ──────────────────────────────────────────────

    def set_display_frame(self, frame):
        self._latest_frame = frame.copy()
        self._bg.apply(frame, learningRate=config.MOG2_LEARNING_RATE)

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
            return

        cx = x1 + bw // 2
        cy = y1 + bh // 2
        self._init_position(cx, cy)

    def _init_position(self, cx: int, cy: int):
        if self.state == WAIT_A:
            self._tkr_a.set_initial_position(cx, cy)
            self.flash_a_time = time.time()
            self.state        = WAIT_B
        elif self.state in (WAIT_B, TRACKING):
            self._tkr_b.set_initial_position(cx, cy)
            self.flash_b_time = time.time()
            self.state        = TRACKING

    # ── Re-select ─────────────────────────────────────────────────

    def reselect_a(self):
        self._tkr_a.reset()
        self.state = WAIT_A

    def reselect_b(self):
        self._tkr_b.reset()
        self.state = WAIT_B

    # ── Main update ───────────────────────────────────────────────

    def update(self, display_frame) -> tuple:
        self._latest_frame = display_frame.copy()
        blobs = self._detect_blobs(display_frame)
        self._last_blobs = blobs
        self._match(blobs)
        self._update_direction()
        return self._tkr_a.get_position(), self._tkr_b.get_position()

    def _detect_blobs(self, frame) -> list:
        fg = self._bg.apply(frame, learningRate=config.MOG2_LEARNING_RATE)

        # Apply track mask — ignore motion outside track boundary
        if self._mask is not None:
            fg = cv2.bitwise_and(fg, self._mask)

        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN,  self._kernel)
        fg = cv2.dilate(fg,       self._kernel, iterations=2)

        contours, _ = cv2.findContours(
            fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blobs = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < config.MIN_BLOB_AREA:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx  = int(M["m10"] / M["m00"])
            cy  = int(M["m01"] / M["m00"])
            x, y, bw, bh = cv2.boundingRect(c)
            blobs.append({"cx": cx, "cy": cy,
                          "area": area, "bbox": (x, y, bw, bh),
                          "used": False})
        return blobs

    def _match(self, blobs: list):
        trains = []
        if self._tkr_a.initialized:
            px, py = self._tkr_a.predict_next()
            trains.append((self._tkr_a, px, py))
        if self._tkr_b.initialized:
            px, py = self._tkr_b.predict_next()
            trains.append((self._tkr_b, px, py))

        blobs_s = sorted(blobs, key=lambda b: -b["area"])

        for (tkr, pred_x, pred_y) in trains:
            best_d, best_b = float(config.MAX_MATCH_DIST), None
            for blob in blobs_s:
                if blob["used"]:
                    continue
                dx = blob["cx"] - pred_x
                dy = blob["cy"] - pred_y
                d  = np.sqrt(dx * dx + dy * dy)
                if d < best_d:
                    best_d = d
                    best_b = blob
            if best_b:
                best_b["used"] = True
                tkr.update_from_blob(best_b["cx"], best_b["cy"],
                                     best_b["bbox"])
            else:
                tkr.predict_update()

    def _update_direction(self):
        """
        Determine if Train A is behind Train B and heading toward it.
        Uses dot product of Train A's velocity with the vector from A to B.
        dot > 0 → A heading toward B → A is behind B → speed up B.
        dot < 0 → A heading away from B → B is behind A → normal slow-B logic.
        """
        if not (self._tkr_a.initialized and self._tkr_b.initialized):
            self._a_is_chasing_b = False
            return

        ax, ay = self._tkr_a._px or 0, self._tkr_a._py or 0
        bx, by = self._tkr_b._px or 0, self._tkr_b._py or 0
        avx, avy = self._tkr_a.velocity

        a_speed = np.sqrt(avx ** 2 + avy ** 2)
        if a_speed < 0.3:
            # Train A not really moving — no chase
            self._a_is_chasing_b = False
            return

        # Vector from A to B
        dx = bx - ax
        dy = by - ay
        dot = avx * dx + avy * dy

        # If dot > 0: A's velocity points toward B → A chasing B
        self._a_is_chasing_b = (dot > 0)

    # ── Gap using facing edges ─────────────────────────────────────

    def facing_gap(self, pos_a: TrainPosition,
                   pos_b: TrainPosition) -> "float | None":
        """
        Gap between the FACING edges of the two trains.
        Uses head/tail points when available — much more accurate
        than centre-to-centre distance.
        If either train's head/tail is not yet known (still building
        velocity), falls back to centre-to-centre.
        """
        if pos_a is None or pos_b is None:
            return None

        if self._a_is_chasing_b:
            # A is behind B → gap = B tail to A head
            a_edge = pos_a.head
            b_edge = pos_b.tail
        else:
            # B is behind A → gap = A tail to B head
            a_edge = pos_a.tail
            b_edge = pos_b.head

        # Fall back to centres if head/tail not yet determined
        if a_edge is None or b_edge is None:
            return pixel_distance(pos_a, pos_b)

        dx = a_edge[0] - b_edge[0]
        dy = a_edge[1] - b_edge[1]
        return float(np.sqrt(dx * dx + dy * dy))

    # ── Debug helpers ─────────────────────────────────────────────

    def get_blobs(self) -> list:
        return self._last_blobs

    def get_mask(self):
        return self._mask

    # ── Safety ────────────────────────────────────────────────────

    def is_a_missing(self) -> bool:
        return not self._tkr_a.initialized or self._tkr_a.is_missing()

    def is_b_missing(self) -> bool:
        return not self._tkr_b.initialized or self._tkr_b.is_missing()

    # ── Status ────────────────────────────────────────────────────

    def instruction_text(self) -> str:
        if self.state == WAIT_A:
            return "STEP 1 — HOLD and DRAG a box around  TRAIN A  (front train)"
        elif self.state == WAIT_B:
            return "STEP 2 — HOLD and DRAG a box around  TRAIN B  (rear BLE train)"
        else:
            a = "tracking" if self._tkr_a.active else "LOST — press A"
            b = "tracking" if self._tkr_b.active else "LOST — press B"
            chase = "  ⚠ A CHASING B" if self._a_is_chasing_b else ""
            return f"A: {a}   B: {b}{chase}"


# ── Distance helpers ──────────────────────────────────────────────

def pixel_distance(a, b) -> "float | None":
    """Centre-to-centre pixel distance between two TrainPositions."""
    if a is None or b is None:
        return None
    return float(np.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2))