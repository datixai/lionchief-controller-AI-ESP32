# ══════════════════════════════════════════════════════════════════
#  train_detector.py  —  MOG2 Motion-Based Train Tracker
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  WHY MOG2 INSTEAD OF OPTICAL FLOW / CSRT:
#    Optical flow and CSRT track APPEARANCE — how the train looks.
#    Small ceiling-camera trains have little texture, rotate on turns,
#    and move fast → appearance changes → box leaves the train.
#
#    MOG2 tracks MOTION — what is moving in the frame.
#    Trains are the only moving objects on the layout.
#    Every frame: find all moving blobs → match nearest blob to each
#    train's last known position → that blob IS the train.
#    Works through turns, speed changes, curves — always.
#
#  HOW IT WORKS:
#    1. User drags a box to set INITIAL position for each train
#    2. MOG2 builds background model (learns the static layout)
#    3. Every frame: background subtraction finds moving blobs
#    4. Each blob is matched to the nearest train by distance
#    5. Velocity prediction smooths position when no blob is near
#    6. Low learning rate prevents stopped trains from disappearing
#       from the background model for several minutes
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


class TrainPosition:
    def __init__(self, x: int, y: int, bbox: tuple = None):
        self.x         = x
        self.y         = y
        self.bbox      = bbox
        self.timestamp = time.time()

    def as_tuple(self):
        return (self.x, self.y)


# ── Per-train state tracker with velocity prediction ───────────────

class _VelocityTracker:
    """
    Tracks one train's position and velocity.
    Position is updated from detected motion blobs.
    When no blob is found, velocity prediction fills the gap.
    """

    def __init__(self, label: str):
        self.label      = label
        self._px        = None    # current x position (float)
        self._py        = None    # current y position (float)
        self._vx        = 0.0    # velocity x (pixels/frame)
        self._vy        = 0.0    # velocity y (pixels/frame)
        self._active    = False
        self._last_seen = 0.0    # time last matched to a real blob
        self._buf       = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self._last_bbox = None
        self._frames_without_blob = 0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def initialized(self) -> bool:
        return self._px is not None

    def set_initial_position(self, x: int, y: int):
        """Called once when user drags a box to select this train."""
        self._px        = float(x)
        self._py        = float(y)
        self._vx        = 0.0
        self._vy        = 0.0
        self._active    = True
        self._last_seen = time.time()
        self._buf.clear()
        self._buf.append((x, y))
        self._frames_without_blob = 0
        logger.info(f"[Train {self.label}] Initial position: ({x},{y})")

    def predict_next(self) -> tuple:
        """Predicted position for next frame based on current velocity."""
        if self._px is None:
            return 0.0, 0.0
        return (self._px + self._vx, self._py + self._vy)

    def update_from_blob(self, blob_cx: int, blob_cy: int,
                         blob_bbox: tuple):
        """
        Blob was matched to this train. Update position and velocity.
        Velocity is updated as exponential moving average so it
        responds to direction changes on turns without being jerky.
        """
        if self._px is not None:
            dx = blob_cx - self._px
            dy = blob_cy - self._py
            a  = config.VELOCITY_ALPHA
            self._vx = a * dx + (1.0 - a) * self._vx
            self._vy = a * dy + (1.0 - a) * self._vy

        self._px        = float(blob_cx)
        self._py        = float(blob_cy)
        self._last_seen = time.time()
        self._active    = True
        self._last_bbox = blob_bbox
        self._frames_without_blob = 0
        self._buf.append((blob_cx, blob_cy))

    def predict_update(self):
        """
        No blob was matched. Advance position by velocity.
        Used when train is between blobs (e.g. short gap in detection).
        """
        if self._px is None:
            return
        self._px += self._vx
        self._py += self._vy
        self._frames_without_blob += 1
        # Gradually decay velocity if no blob for a while (train may be stopping)
        if self._frames_without_blob > 10:
            self._vx *= 0.9
            self._vy *= 0.9

    def get_position(self) -> "TrainPosition | None":
        """Return smoothed current position."""
        if not self._active or self._px is None:
            return None

        # Smooth using rolling buffer
        if self._buf:
            sx = int(round(sum(p[0] for p in self._buf) / len(self._buf)))
            sy = int(round(sum(p[1] for p in self._buf) / len(self._buf)))
        else:
            sx = int(round(self._px))
            sy = int(round(self._py))

        # Estimate bbox: use last real blob bbox, or small default
        if self._last_bbox:
            bx, by, bw, bh = self._last_bbox
            # Shift bbox to current smoothed position
            bx = sx - bw // 2
            by = sy - bh // 2
            bbox = (bx, by, bw, bh)
        else:
            s    = 30
            bbox = (sx - s, sy - s, s * 2, s * 2)

        return TrainPosition(sx, sy, bbox)

    def is_missing(self) -> bool:
        return (time.time() - self._last_seen) > config.MISSING_TIMEOUT_S

    def reset(self):
        self._px        = None
        self._py        = None
        self._vx        = 0.0
        self._vy        = 0.0
        self._active    = False
        self._last_bbox = None
        self._buf.clear()
        self._frames_without_blob = 0

    @property
    def current_xy(self):
        return self._px, self._py


# ══════════════════════════════════════════════════════════════════
#  DragTracker — public interface (same as before, no changes in main.py)
# ══════════════════════════════════════════════════════════════════

class DragTracker:
    """
    Drag-to-select two-train MOG2 motion tracker.

    User drags a box to set INITIAL position only.
    After that MOG2 detects moving blobs every frame and matches them
    to each train by proximity. Completely robust to turns and speed.

    Interface unchanged from previous versions — main.py needs no edits.
    """

    def __init__(self):
        # MOG2 background subtractor
        self._bg = cv2.createBackgroundSubtractorMOG2(
            history       = config.MOG2_HISTORY,
            varThreshold  = config.MOG2_VAR_THRESHOLD,
            detectShadows = False,
        )
        # Morphological kernel for blob cleaning
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (7, 7))

        # Per-train velocity trackers
        self._tkr_a = _VelocityTracker("A")
        self._tkr_b = _VelocityTracker("B")

        # Drag selection state
        self.state       = WAIT_A
        self.drag_start  = None
        self.drag_end    = None
        self.is_dragging = False

        self._latest_frame = None

        # Flash confirmation timestamps
        self.flash_a_time = 0.0
        self.flash_b_time = 0.0

        # Last detected blobs (for display)
        self._last_blobs = []

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

    def set_display_frame(self, frame):
        """
        Store frame and feed it to MOG2 background model.
        Called every frame — including during warmup — so the
        background model is always up to date.
        """
        self._latest_frame = frame.copy()
        # Feed to background model with low learning rate
        # so stopped trains take minutes to be "learned" as background
        self._bg.apply(frame, learningRate=config.MOG2_LEARNING_RATE)

    # ── Mouse events (coordinates already in display space) ───────

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

        # Centre of the drawn box = initial train position
        cx = x1 + bw // 2
        cy = y1 + bh // 2
        self._init_position(cx, cy)

    def _init_position(self, cx: int, cy: int):
        """Set initial position for the pending train."""
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

    # ── Main update — MOG2 detection + matching ────────────────────

    def update(self, display_frame) -> tuple:
        """
        Detect all moving blobs in the frame.
        Match each blob to the nearest active train.
        Return updated (pos_a, pos_b).
        """
        self._latest_frame = display_frame.copy()

        # Step 1: detect all moving blobs
        blobs = self._detect_blobs(display_frame)
        self._last_blobs = blobs

        # Step 2: match blobs to trains
        self._match(blobs)

        # Step 3: return positions
        return self._tkr_a.get_position(), self._tkr_b.get_position()

    def _detect_blobs(self, frame) -> list:
        """
        Run MOG2 and find all moving foreground blobs.
        Returns list of dicts with cx, cy, area, bbox, used.
        """
        # Apply background subtraction
        fg = self._bg.apply(frame,
                            learningRate=config.MOG2_LEARNING_RATE)

        # Remove noise: open (erosion then dilation) removes tiny specks
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN,  self._kernel)
        # Dilate to join nearby blobs from same train
        fg = cv2.dilate(fg, self._kernel, iterations=2)

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
            blobs.append({
                "cx": cx, "cy": cy,
                "area": area, "bbox": (x, y, bw, bh),
                "used": False,
            })

        return blobs

    def _match(self, blobs: list):
        """
        Greedy blob-to-train matching.
        Each blob is assigned to the nearest active train
        whose predicted position is within MAX_MATCH_DIST.
        Trains with no matching blob are updated by velocity.
        """
        # Build list of active trains with their predictions
        trains = []
        if self._tkr_a.initialized:
            px, py = self._tkr_a.predict_next()
            trains.append((self._tkr_a, px, py))
        if self._tkr_b.initialized:
            px, py = self._tkr_b.predict_next()
            trains.append((self._tkr_b, px, py))

        if not trains:
            return

        # Sort blobs by area descending — prefer larger blobs
        blobs_sorted = sorted(blobs, key=lambda b: -b["area"])

        for (tkr, pred_x, pred_y) in trains:
            best_dist = float(config.MAX_MATCH_DIST)
            best_blob = None

            for blob in blobs_sorted:
                if blob["used"]:
                    continue
                dx = blob["cx"] - pred_x
                dy = blob["cy"] - pred_y
                d  = np.sqrt(dx * dx + dy * dy)
                if d < best_dist:
                    best_dist = d
                    best_blob = blob

            if best_blob:
                best_blob["used"] = True
                tkr.update_from_blob(
                    best_blob["cx"], best_blob["cy"],
                    best_blob["bbox"])
            else:
                # No blob nearby — coast on velocity
                tkr.predict_update()

    # ── Debug display ─────────────────────────────────────────────

    def get_blobs(self) -> list:
        """Return last detected blobs for debug display."""
        return self._last_blobs

    # ── Safety ────────────────────────────────────────────────────

    def is_a_missing(self) -> bool:
        return not self._tkr_a.initialized or self._tkr_a.is_missing()

    def is_b_missing(self) -> bool:
        return not self._tkr_b.initialized or self._tkr_b.is_missing()

    # ── Status text ───────────────────────────────────────────────

    def instruction_text(self) -> str:
        if self.state == WAIT_A:
            return "STEP 1 — HOLD and DRAG a box around  TRAIN A  (front train)"
        elif self.state == WAIT_B:
            return "STEP 2 — HOLD and DRAG a box around  TRAIN B  (rear BLE train)"
        else:
            a = "tracking" if self._tkr_a.active else "LOST — press A to re-select"
            b = "tracking" if self._tkr_b.active else "LOST — press B to re-select"
            return f"A: {a}     B: {b}"


# ── Distance helper ───────────────────────────────────────────────

def pixel_distance(a, b) -> "float | None":
    if a is None or b is None:
        return None
    return float(np.sqrt((a.x - b.x)**2 + (a.y - b.y)**2))