# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  Robust MOG2 Tracker v9.0
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  THREE KEY IMPROVEMENTS:
#
#  1. REFERENCE PATCH — catches stopped trains.
#     When MOG2 finds no blob near a train, the tracker compares
#     the current camera image to a snapshot taken when the train was
#     last seen. If the image MATCHES → train is stopped there →
#     position is LOCKED (frozen). If it doesn't match → train moved.
#     This means a stopped train never loses its tracking circle.
#
#  2. BLOB MUTEX — prevents both trackers grabbing the same blob.
#     When trains are close and blobs merge, both trackers might
#     compete for the same blob. After matching, if both picked the
#     same blob, only the closer tracker keeps it. The other coasts
#     on velocity briefly or stays locked in its last position.
#
#  3. POSITION LOCK — frozen position when train is stopped.
#     Rather than coasting (drifting) when no blob is found,
#     a stopped-confirmed train holds its exact last position.
#     The tracker only moves again when actual motion is detected.
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
    def __init__(self, x: int, y: int, radius: int = 25,
                 locked: bool = False):
        self.x      = x
        self.y      = y
        self.radius = radius
        self.locked = locked    # True = train is stopped / position frozen
        self.timestamp = time.time()

    def as_tuple(self):
        return (self.x, self.y)


# ── Per-train tracker ─────────────────────────────────────────────

class _TrainTracker:
    """
    Tracks one train using:
      - Local MOG2 blob search (moving trains)
      - Reference patch comparison (stopped trains)
      - Position lock (don't drift when stopped)
    """

    def __init__(self, label: str):
        self.label         = label
        self._px           = None    # float x
        self._py           = None    # float y
        self._vx           = 0.0
        self._vy           = 0.0
        self._active       = False
        self._seen         = 0.0
        self._buf          = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self._blob_area    = 300.0

        # Stopped-train detection
        self._ref_gray     = None    # reference patch (grayscale)
        self._ref_x1       = 0       # patch origin in frame
        self._ref_y1       = 0
        self._no_blob_ct   = 0       # consecutive frames without blob
        self._locked       = False   # True = position frozen (stopped)

    @property
    def active(self) -> bool:
        return self._active

    @property
    def initialized(self) -> bool:
        return self._px is not None

    @property
    def is_locked(self) -> bool:
        return self._locked

    @property
    def velocity(self):
        return (self._vx, self._vy)

    def init(self, cx: int, cy: int):
        self._px         = float(cx)
        self._py         = float(cy)
        self._vx         = 0.0
        self._vy         = 0.0
        self._active     = True
        self._seen       = time.time()
        self._no_blob_ct = 0
        self._locked     = False
        self._ref_gray   = None
        self._buf.clear()
        self._buf.append((cx, cy))
        logger.info(f"[{self.label}] init at ({cx},{cy})")

    # ── Core update ───────────────────────────────────────────────

    def update_with_blob(self, blob: "dict | None",
                          gray_frame: np.ndarray):
        """
        Called by DragTracker after blob-mutex assignment.
        blob = {"cx": int, "cy": int, "area": float} or None.
        """
        if not self.initialized:
            return

        if blob is not None:
            # ── Motion detected — normal update ──────────────────
            cx, cy, area = blob["cx"], blob["cy"], blob["area"]
            self._update(cx, cy, area)
            self._locked     = False
            self._no_blob_ct = 0
            # Store fresh reference patch at this confirmed position
            self._store_reference(gray_frame, cx, cy)

        else:
            # ── No blob found in local search ─────────────────────
            self._no_blob_ct += 1

            if self._no_blob_ct >= config.STOP_CONFIRM_FRAMES:
                # Check if train is still there (stopped)
                if self._reference_matches(gray_frame):
                    # Train is stopped — LOCK position
                    self._locked = True
                    self._vx    *= 0.0   # zero velocity — not moving
                    self._vy    *= 0.0
                    # Append current position to buf (keeps display stable)
                    if self._px is not None:
                        self._buf.append((int(round(self._px)),
                                          int(round(self._py))))
                    # Still active — just stopped
                    self._active = True
                else:
                    # Patch doesn't match → train moved, we haven't found it
                    self._locked = False
                    if self._no_blob_ct < 40:
                        # Coast briefly on velocity
                        self._px += self._vx
                        self._py += self._vy
                        self._vx *= 0.88
                        self._vy *= 0.88
                    else:
                        # Truly lost after extended search
                        self._active = False
            else:
                # Too few frames without blob — coast briefly
                if not self._locked:
                    self._px += self._vx
                    self._py += self._vy
                    self._vx *= 0.92
                    self._vy *= 0.92

    def _update(self, cx: int, cy: int, area: float):
        if self._px is not None:
            dx = cx - self._px
            dy = cy - self._py
            a  = config.VELOCITY_ALPHA
            self._vx = a * dx + (1.0 - a) * self._vx
            self._vy = a * dy + (1.0 - a) * self._vy
        self._px         = float(cx)
        self._py         = float(cy)
        self._blob_area  = 0.85 * self._blob_area + 0.15 * area
        self._active     = True
        self._seen       = time.time()
        self._buf.append((cx, cy))

    # ── Reference patch ───────────────────────────────────────────

    def _store_reference(self, gray: np.ndarray, cx: int, cy: int):
        """Save a grayscale patch around the train's current position."""
        h, w  = gray.shape[:2]
        r     = config.REF_PATCH_HALF
        x1    = max(0, cx - r)
        y1    = max(0, cy - r)
        x2    = min(w, cx + r)
        y2    = min(h, cy + r)
        self._ref_gray = gray[y1:y2, x1:x2].copy()
        self._ref_x1   = x1
        self._ref_y1   = y1

    def _reference_matches(self, gray: np.ndarray) -> bool:
        """
        Compare current frame patch to stored reference.
        Returns True if the scene looks the same → train still there.
        """
        if self._ref_gray is None or self._px is None:
            return False

        h, w = gray.shape[:2]
        ph, pw = self._ref_gray.shape[:2]
        x1 = self._ref_x1
        y1 = self._ref_y1
        x2 = min(w, x1 + pw)
        y2 = min(h, y1 + ph)

        current = gray[y1:y2, x1:x2]

        # Size might differ at edges — use the smaller common region
        ch, cw = current.shape[:2]
        rh, rw = self._ref_gray.shape[:2]
        mh, mw = min(ch, rh), min(cw, rw)

        if mh < 10 or mw < 10:
            return False

        diff = cv2.absdiff(current[:mh, :mw],
                           self._ref_gray[:mh, :mw])
        mean_diff = float(np.mean(diff))

        # Low diff → same scene → train still there (stopped)
        return mean_diff < config.STOPPED_DIFF_THR

    # ── Local blob search ─────────────────────────────────────────

    def local_search(self, fg_masked: np.ndarray) -> "dict | None":
        """
        Find the largest blob within SEARCH_RADIUS of predicted position.
        Returns blob dict or None.
        """
        if not self.initialized:
            return None

        pred_x = self._px + self._vx
        pred_y = self._py + self._vy
        h, w   = fg_masked.shape[:2]
        r      = config.SEARCH_RADIUS

        x1 = max(0, int(pred_x - r))
        y1 = max(0, int(pred_y - r))
        x2 = min(w, int(pred_x + r))
        y2 = min(h, int(pred_y + r))

        local = fg_masked[y1:y2, x1:x2]
        if local.size == 0:
            return None

        contours, _ = cv2.findContours(
            local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_area = config.MIN_BLOB_AREA
        best      = None

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
            best      = {"cx": cx, "cy": cy, "area": area}

        return best

    def predicted_pos(self):
        if self._px is None:
            return (0.0, 0.0)
        return (self._px + self._vx, self._py + self._vy)

    # ── Position output ───────────────────────────────────────────

    def get_position(self) -> "TrainPosition | None":
        if not self._active or not self._buf:
            return None
        sx = int(round(sum(p[0] for p in self._buf) / len(self._buf)))
        sy = int(round(sum(p[1] for p in self._buf) / len(self._buf)))
        r  = int(np.sqrt(self._blob_area / np.pi) * 1.7)
        r  = max(config.CIRCLE_RADIUS_MIN, min(config.CIRCLE_RADIUS_MAX, r))
        return TrainPosition(sx, sy, r, self._locked)

    def draw_search_area(self, frame, color):
        if self._px is not None:
            px = int(self._px + self._vx)
            py = int(self._py + self._vy)
            cv2.circle(frame, (px, py), config.SEARCH_RADIUS, color, 1)

    def is_missing(self) -> bool:
        return (time.time() - self._seen) > config.MISSING_TIMEOUT_S

    def reset(self):
        self._px = None; self._py = None
        self._vx = 0.0;  self._vy = 0.0
        self._active = False
        self._locked = False
        self._ref_gray = None
        self._no_blob_ct = 0
        self._buf.clear()


# ══════════════════════════════════════════════════════════════════
#  DragTracker
# ══════════════════════════════════════════════════════════════════

class DragTracker:

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

        self._table_mask = None
        self._table_rect = None

        self.state       = WAIT_TABLE
        self.drag_start  = None
        self.drag_end    = None
        self.is_dragging = False

        self.flash_table = 0.0
        self.flash_a     = 0.0
        self.flash_b     = 0.0

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

    # ── Frame supply ──────────────────────────────────────────────

    def set_display_frame(self, frame):
        self._bg.apply(frame, learningRate=config.MOG2_LEARNING_RATE)

    # ── Mouse ─────────────────────────────────────────────────────

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

        elif self.state == WAIT_A:
            self._tkr_a.init(cx, cy)
            self.flash_a = time.time()
            self.state   = WAIT_B

        elif self.state in (WAIT_B, TRACKING):
            self._tkr_b.init(cx, cy)
            self.flash_b = time.time()
            self.state   = TRACKING

    # ── Table mask ────────────────────────────────────────────────

    def _set_table_rect(self, x1, y1, x2, y2):
        self._table_rect = (x1, y1, x2, y2)
        mask = np.zeros((config.DISPLAY_H, config.DISPLAY_W), dtype=np.uint8)
        cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
        self._table_mask = mask
        mf = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          config.TABLE_MASK_FILE)
        with open(mf, "w") as f:
            json.dump({"rect": [x1, y1, x2, y2]}, f)
        logger.info(f"Table mask saved ({x1},{y1})→({x2},{y2})")

    def _load_table_mask(self):
        mf = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          config.TABLE_MASK_FILE)
        if not os.path.exists(mf):
            return
        try:
            with open(mf) as f:
                d = json.load(f)
            x1, y1, x2, y2 = d["rect"]
            self._table_rect = (x1, y1, x2, y2)
            mask = np.zeros(
                (config.DISPLAY_H, config.DISPLAY_W), dtype=np.uint8)
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
            self._table_mask = mask
            self.state       = WAIT_A
            logger.info("Table mask loaded from file")
        except Exception as e:
            logger.warning(f"Table mask load failed: {e}")

    def redraw_table(self):
        self._table_mask = None
        self._table_rect = None
        self.state       = WAIT_TABLE

    def reselect_a(self):
        self._tkr_a.reset()
        self.state = WAIT_A

    def reselect_b(self):
        self._tkr_b.reset()
        self.state = WAIT_B

    # ── Main update with blob mutex ───────────────────────────────

    def update(self, display_frame) -> tuple:
        """
        Full pipeline:
          1. MOG2 foreground (with table mask)
          2. Local blob search for each train independently
          3. Blob MUTEX — if both trackers found the same blob,
             only the closer one keeps it
          4. Update each tracker with its assigned blob (or None)
          5. Return positions
        """
        # Build foreground mask
        fg = self._bg.apply(display_frame,
                            learningRate=config.MOG2_LEARNING_RATE)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, self._kernel)
        fg = cv2.dilate(fg, self._kernel, iterations=1)

        if self._table_mask is not None:
            fg = cv2.bitwise_and(fg, self._table_mask)

        # Convert to grayscale for reference patch comparison
        gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)

        # Local blob search per train
        blob_a = self._tkr_a.local_search(fg) if self._tkr_a.initialized else None
        blob_b = self._tkr_b.local_search(fg) if self._tkr_b.initialized else None

        # ── BLOB MUTEX ────────────────────────────────────────────
        # If both trackers found a blob at the same location → conflict.
        # Only the closer tracker keeps its blob; the other gets None.
        if blob_a is not None and blob_b is not None:
            same = self._blobs_overlap(blob_a, blob_b)
            if same:
                pred_a = self._tkr_a.predicted_pos()
                pred_b = self._tkr_b.predicted_pos()
                da = np.sqrt((blob_a["cx"]-pred_a[0])**2 +
                             (blob_a["cy"]-pred_a[1])**2)
                db = np.sqrt((blob_b["cx"]-pred_b[0])**2 +
                             (blob_b["cy"]-pred_b[1])**2)
                if da <= db:
                    blob_b = None   # A is closer — B coasts/locks
                else:
                    blob_a = None   # B is closer — A coasts/locks
                logger.debug(
                    f"Blob mutex triggered — "
                    f"{'A' if blob_a else 'B'} keeps blob")

        # Update trackers
        self._tkr_a.update_with_blob(blob_a, gray)
        self._tkr_b.update_with_blob(blob_b, gray)

        return self._tkr_a.get_position(), self._tkr_b.get_position()

    def _blobs_overlap(self, ba: dict, bb: dict) -> bool:
        """True if two blobs are close enough to be considered the same."""
        dx   = ba["cx"] - bb["cx"]
        dy   = ba["cy"] - bb["cy"]
        dist = np.sqrt(dx*dx + dy*dy)
        # Overlap if centres are within half a search radius of each other
        return dist < (config.SEARCH_RADIUS * 0.6)

    # ── Direction detection ───────────────────────────────────────

    def a_is_chasing_b(self, pos_a, pos_b) -> bool:
        if pos_a is None or pos_b is None:
            return False
        avx, avy = self._tkr_a.velocity
        spd = np.sqrt(avx**2 + avy**2)
        if spd < 0.3:
            return False
        dx  = pos_b.x - pos_a.x
        dy  = pos_b.y - pos_a.y
        return (avx * dx + avy * dy) > 0

    def facing_gap(self, pos_a, pos_b, a_chasing: bool) -> "float | None":
        if pos_a is None or pos_b is None:
            return None
        dx  = pos_a.x - pos_b.x
        dy  = pos_a.y - pos_b.y
        d   = float(np.sqrt(dx*dx + dy*dy))
        gap = d - pos_a.radius - pos_b.radius
        return max(0.0, gap)

    # ── Safety ────────────────────────────────────────────────────

    def is_a_missing(self) -> bool:
        return not self._tkr_a.initialized or self._tkr_a.is_missing()

    def is_b_missing(self) -> bool:
        return not self._tkr_b.initialized or self._tkr_b.is_missing()

    # ── Status ────────────────────────────────────────────────────

    def instruction_text(self) -> str:
        msgs = {
            WAIT_TABLE: "STEP 1 — DRAG a box around the TABLE  (whole track area)",
            WAIT_A:     "STEP 2 — DRAG a box around  TRAIN A  (front train)",
            WAIT_B:     "STEP 3 — DRAG a box around  TRAIN B  (rear BLE train)",
            TRACKING:   (
                f"A:{'LOCKED' if self._tkr_a.is_locked else 'tracking' if self._tkr_a.active else 'LOST'}  "
                f"B:{'LOCKED' if self._tkr_b.is_locked else 'tracking' if self._tkr_b.active else 'LOST'}"
            ),
        }
        return msgs.get(self.state, "")


# ── Distance helper ───────────────────────────────────────────────

def pixel_distance(a, b) -> "float | None":
    if a is None or b is None:
        return None
    return float(np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2))