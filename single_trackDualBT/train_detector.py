# train_detector.py — v10.0 with confidence + track path learning
# Harry Locomotive Project 3 | Datix AI | June 2026

import cv2
import json
import os
import numpy as np
import time
import logging
from collections import deque

import config

logger = logging.getLogger("Tracker")

WAIT_TABLE = "WAIT_TABLE"
WAIT_A     = "WAIT_A"
WAIT_B     = "WAIT_B"
TRACKING   = "TRACKING"


class TrainPosition:
    def __init__(self, x, y, radius=25, locked=False, confidence=1.0):
        self.x          = x
        self.y          = y
        self.radius     = radius
        self.locked     = locked
        self.confidence = confidence   # 0.0-1.0 tracking quality
        self.timestamp  = time.time()

    def as_tuple(self):
        return (self.x, self.y)


# ── Track path recorder / snapper ────────────────────────────────

class TrackPath:
    """
    Records the loop path from Train A's movements.
    Snaps tracker positions to nearest path point to prevent drift.
    """

    def __init__(self):
        self._points   = []
        self.recording = False
        self._loaded   = False

    @property
    def has_path(self) -> bool:
        return len(self._points) >= 10

    @property
    def points(self):
        return self._points

    def start_recording(self):
        self._points   = []
        self.recording = True
        logger.info("Track path recording started")

    def stop_recording(self):
        self.recording = False
        if self.has_path:
            self._save()
        logger.info(f"Track path recording stopped — {len(self._points)} pts")

    def record(self, x: int, y: int):
        if not self.recording:
            return
        if not self._points:
            self._points.append((x, y))
            return
        lx, ly = self._points[-1]
        if np.sqrt((x-lx)**2 + (y-ly)**2) >= config.PATH_MIN_POINT_DIST:
            self._points.append((x, y))

    def snap(self, x: int, y: int) -> tuple:
        """Return nearest path point if within PATH_SNAP_RADIUS."""
        if not self.has_path:
            return (x, y)
        pts    = np.array(self._points, dtype=np.float32)
        dists  = np.sqrt((pts[:,0]-x)**2 + (pts[:,1]-y)**2)
        idx    = int(np.argmin(dists))
        d_min  = float(dists[idx])
        if d_min < config.PATH_SNAP_RADIUS:
            return (int(pts[idx,0]), int(pts[idx,1]))
        return (x, y)

    def _save(self):
        track_file = getattr(config, 'TRACK_PATH_FILE', 'track_path.json')
        pf = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            track_file)
        with open(pf, "w") as f:
            json.dump({"points": self._points}, f)
        logger.info(f"Track path saved → {pf}")

    def load(self):
        # getattr fallback in case config.py is not yet updated
        track_file = getattr(config, 'TRACK_PATH_FILE', 'track_path.json')
        pf = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            track_file)
        if not os.path.exists(pf):
            return
        try:
            with open(pf) as f:
                d = json.load(f)
            self._points = [tuple(p) for p in d["points"]]
            self._loaded = True
            logger.info(f"Track path loaded — {len(self._points)} pts")
        except Exception as e:
            logger.warning(f"Track path load failed: {e}")


# ── Per-train tracker ─────────────────────────────────────────────

class _TrainTracker:

    def __init__(self, label: str):
        self.label         = label
        self._px           = None
        self._py           = None
        self._vx           = 0.0
        self._vy           = 0.0
        self._active       = False
        self._seen         = 0.0
        self._buf          = deque(maxlen=config.POSITION_SMOOTH_FRAMES)
        self._blob_area    = 300.0
        self._ref_gray     = None
        self._ref_x1       = 0
        self._ref_y1       = 0
        self._no_blob_ct   = 0
        self._locked       = False
        self._fresh_blobs  = 0   # consecutive frames with real blob

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

    @property
    def confidence(self) -> float:
        """0.0-1.0 tracking quality indicator."""
        if not self._active:
            return 0.0
        if self._locked:
            return 0.65   # stopped & confirmed — medium confidence
        if self._no_blob_ct == 0:
            # Fresh blob — confidence based on consecutive fresh frames
            return min(1.0, 0.7 + self._fresh_blobs * 0.03)
        # Coasting: decays with frames without blob
        return max(0.1, 1.0 - self._no_blob_ct / 30.0)

    def init(self, cx: int, cy: int, gray_frame=None):
        self._px          = float(cx)
        self._py          = float(cy)
        self._vx          = 0.0
        self._vy          = 0.0
        self._active      = True
        self._seen        = time.time()
        self._no_blob_ct  = 0
        self._fresh_blobs = 0
        self._locked      = False
        self._ref_gray    = None
        self._buf.clear()
        self._buf.append((cx, cy))
        # Store reference patch immediately so stopped-train
        # detection works from the very first frame
        if gray_frame is not None:
            self._store_ref(gray_frame, cx, cy)
            logger.info(
                f"[{self.label}] init ({cx},{cy}) — ref patch stored")
        else:
            logger.info(f"[{self.label}] init ({cx},{cy})")

    # Frames to HOLD position after selection before coast/lost logic.
    # Gives train time to start moving after the user selects it.
    # MOG2 learns stationary trains as background, so we hold position
    # while the train is still and wait for motion to appear.
    _INITIAL_HOLD_FRAMES = 90   # 3 seconds at 30fps

    def update_with_blob(self, blob, gray_frame: np.ndarray,
                          track_path: "TrackPath | None" = None):
        if not self.initialized:
            return

        if blob is not None:
            # ── Blob found — train is MOVING ──────────────────────
            cx, cy, area = blob["cx"], blob["cy"], blob["area"]
            if track_path and track_path.has_path:
                cx, cy = track_path.snap(cx, cy)
            self._update(cx, cy, area)
            self._locked      = False
            self._no_blob_ct  = 0
            self._fresh_blobs = min(self._fresh_blobs + 1, 20)
            self._store_ref(gray_frame, cx, cy)

        else:
            # ── No blob found ──────────────────────────────────────
            self._no_blob_ct += 1
            self._fresh_blobs = 0

            if self._no_blob_ct <= self._INITIAL_HOLD_FRAMES:
                # HOLD PHASE: just selected or recently seen.
                # Train may be stationary (MOG2 background) — stay put.
                # This prevents losing the tracker in the first seconds.
                self._active = True
                self._locked = False
                if self._px is not None:
                    self._buf.append((int(round(self._px)),
                                      int(round(self._py))))

            elif self._ref_matches(gray_frame):
                # LOCK PHASE: ref patch confirms train is still there.
                # Train stopped and was confirmed in place.
                self._locked = True
                self._vx     = 0.0
                self._vy     = 0.0
                self._active = True
                if self._px is not None:
                    self._buf.append((int(round(self._px)),
                                      int(round(self._py))))

            else:
                # COAST / LOST: train may have moved without detection.
                self._locked = False
                coast_limit  = self._INITIAL_HOLD_FRAMES + 60
                if self._no_blob_ct < coast_limit:
                    # Coast on velocity briefly
                    self._px  = (self._px or 0) + self._vx
                    self._py  = (self._py or 0) + self._vy
                    self._vx *= 0.85
                    self._vy *= 0.85
                    self._active = True
                else:
                    # Truly lost — mark inactive
                    self._active = False

    def _update(self, cx, cy, area):
        if self._px is not None:
            dx = cx - self._px
            dy = cy - self._py
            a  = config.VELOCITY_ALPHA
            self._vx = a*dx + (1-a)*self._vx
            self._vy = a*dy + (1-a)*self._vy
        self._px        = float(cx)
        self._py        = float(cy)
        self._blob_area = 0.85*self._blob_area + 0.15*area
        self._active    = True
        self._seen      = time.time()
        self._buf.append((cx, cy))

    def _store_ref(self, gray, cx, cy):
        h, w = gray.shape[:2]
        r    = config.REF_PATCH_HALF
        x1   = max(0, cx - r)
        y1   = max(0, cy - r)
        x2   = min(w, cx + r)
        y2   = min(h, cy + r)
        self._ref_gray = gray[y1:y2, x1:x2].copy()
        self._ref_x1   = x1
        self._ref_y1   = y1

    def _ref_matches(self, gray) -> bool:
        if self._ref_gray is None or self._px is None:
            return False
        h, w = gray.shape[:2]
        ph, pw = self._ref_gray.shape[:2]
        x1, y1 = self._ref_x1, self._ref_y1
        x2, y2 = min(w, x1+pw), min(h, y1+ph)
        cur = gray[y1:y2, x1:x2]
        mh  = min(cur.shape[0], ph)
        mw  = min(cur.shape[1], pw)
        if mh < 10 or mw < 10:
            return False
        diff = cv2.absdiff(cur[:mh,:mw], self._ref_gray[:mh,:mw])
        return float(np.mean(diff)) < config.STOPPED_DIFF_THR

    def local_search(self, fg) -> "dict | None":
        if not self.initialized:
            return None
        pred_x = (self._px or 0) + self._vx
        pred_y = (self._py or 0) + self._vy
        h, w   = fg.shape[:2]
        r      = config.SEARCH_RADIUS
        x1 = max(0, int(pred_x-r)); y1 = max(0, int(pred_y-r))
        x2 = min(w, int(pred_x+r)); y2 = min(h, int(pred_y+r))
        local  = fg[y1:y2, x1:x2]
        if local.size == 0:
            return None
        contours, _ = cv2.findContours(
            local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_a, best = config.MIN_BLOB_AREA, None
        for c in contours:
            area = cv2.contourArea(c)
            if area < best_a: continue
            M = cv2.moments(c)
            if M["m00"] == 0: continue
            cx = int(M["m10"]/M["m00"])+x1
            cy = int(M["m01"]/M["m00"])+y1
            best_a = area
            best   = {"cx": cx, "cy": cy, "area": area}
        return best

    def predicted_pos(self):
        return ((self._px or 0)+self._vx,
                (self._py or 0)+self._vy)

    def get_position(self) -> "TrainPosition | None":
        if not self._active or not self._buf:
            return None
        sx = int(round(sum(p[0] for p in self._buf)/len(self._buf)))
        sy = int(round(sum(p[1] for p in self._buf)/len(self._buf)))
        r  = max(config.CIRCLE_RADIUS_MIN,
                 min(config.CIRCLE_RADIUS_MAX,
                     int(np.sqrt(self._blob_area/np.pi)*1.7)))
        return TrainPosition(sx, sy, r, self._locked, self.confidence)

    def draw_search_area(self, frame, color):
        if self._px is not None:
            cv2.circle(frame,
                       (int((self._px or 0)+self._vx),
                        int((self._py or 0)+self._vy)),
                       config.SEARCH_RADIUS, color, 1)

    def is_missing(self) -> bool:
        return (time.time()-self._seen) > config.MISSING_TIMEOUT_S

    def reset(self):
        self._px=None; self._py=None; self._vx=0.0; self._vy=0.0
        self._active=False; self._locked=False; self._ref_gray=None
        self._no_blob_ct=0; self._fresh_blobs=0; self._buf.clear()


# ══════════════════════════════════════════════════════════════════
#  DragTracker
# ══════════════════════════════════════════════════════════════════

class DragTracker:

    def __init__(self):
        self._bg = cv2.createBackgroundSubtractorMOG2(
            history=config.MOG2_HISTORY,
            varThreshold=config.MOG2_VAR_THRESHOLD,
            detectShadows=False)
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

        self._tkr_a = _TrainTracker("A")
        self._tkr_b = _TrainTracker("B")

        self.track_path = TrackPath()
        self.track_path.load()

        self._table_mask  = None
        self._table_rect  = None
        self._latest_gray = None   # used to pass gray to init()

        self.state       = WAIT_TABLE
        self.drag_start  = None
        self.drag_end    = None
        self.is_dragging = False

        self.flash_table = 0.0
        self.flash_a     = 0.0
        self.flash_b     = 0.0

        self._load_table_mask()

    @property
    def ready(self) -> bool:
        return self._tkr_a.initialized and self._tkr_b.initialized

    @property
    def tracking_a(self) -> bool:
        return self._tkr_a.active

    @property
    def tracking_b(self) -> bool:
        return self._tkr_b.active

    def confidence_a(self) -> float:
        return self._tkr_a.confidence

    def confidence_b(self) -> float:
        return self._tkr_b.confidence

    def set_display_frame(self, frame):
        self._bg.apply(frame, learningRate=config.MOG2_LEARNING_RATE)
        # Store gray frame so init() can use it for reference patch
        self._latest_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def on_mouse_down(self, x, y):
        self.drag_start=(x,y); self.drag_end=(x,y); self.is_dragging=True

    def on_mouse_move(self, x, y):
        if self.is_dragging: self.drag_end=(x,y)

    def on_mouse_up(self, x, y):
        if not self.is_dragging or self.drag_start is None: return
        self.drag_end=(x,y); self.is_dragging=False
        bw=abs(self.drag_end[0]-self.drag_start[0])
        bh=abs(self.drag_end[1]-self.drag_start[1])
        if bw<config.MIN_BOX_SIZE or bh<config.MIN_BOX_SIZE: return
        cx=(self.drag_start[0]+self.drag_end[0])//2
        cy=(self.drag_start[1]+self.drag_end[1])//2
        x1=min(self.drag_start[0],self.drag_end[0])
        y1=min(self.drag_start[1],self.drag_end[1])
        x2=max(self.drag_start[0],self.drag_end[0])
        y2=max(self.drag_start[1],self.drag_end[1])
        if self.state==WAIT_TABLE:
            self._set_table(x1,y1,x2,y2)
            self.flash_table=time.time(); self.state=WAIT_A
        elif self.state==WAIT_A:
            # Pass current gray frame so reference patch is stored now
            self._tkr_a.init(cx, cy, self._latest_gray)
            self.flash_a=time.time(); self.state=WAIT_B
        elif self.state in (WAIT_B,TRACKING):
            self._tkr_b.init(cx, cy, self._latest_gray)
            self.flash_b=time.time(); self.state=TRACKING

    def _set_table(self, x1, y1, x2, y2):
        self._table_rect = (x1, y1, x2, y2)
        mask = np.zeros((config.DISPLAY_H, config.DISPLAY_W), dtype=np.uint8)
        cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
        self._table_mask = mask

        # Save to file — always overwrite, always show path
        mf = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            config.TABLE_MASK_FILE)
        try:
            with open(mf, "w") as f:
                json.dump({"rect": [x1, y1, x2, y2]}, f, indent=2)
            print(f"  [OK] Table mask SAVED → {mf}")
            print(f"     Rect: ({x1},{y1}) → ({x2},{y2})")
            logger.info(f"Table mask saved: ({x1},{y1})→({x2},{y2})")
        except Exception as e:
            print(f"  [ERR] Table mask SAVE FAILED: {e}")
            print(f"     Tried to write: {mf}")
            logger.error(f"Table mask save failed: {e}")

    def _load_table_mask(self):
        mf = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            config.TABLE_MASK_FILE)
        if not os.path.exists(mf):
            print(f"  No table mask file found at: {mf}")
            print(f"  Draw the table boundary first (Step 1)")
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
            print(f"  [OK] Table mask LOADED from: {mf}")
            print(f"     Rect: ({x1},{y1}) → ({x2},{y2})")
            print(f"     Press T to redraw it")
            logger.info(f"Table mask loaded: ({x1},{y1})→({x2},{y2})")
        except Exception as e:
            print(f"  [ERR] Table mask LOAD FAILED: {e}")
            logger.warning(f"Table mask load failed: {e}")

    def redraw_table(self):
        self._table_mask=None; self._table_rect=None; self.state=WAIT_TABLE

    def reselect_a(self):
        self._tkr_a.reset(); self.state=WAIT_A

    def reselect_b(self):
        self._tkr_b.reset(); self.state=WAIT_B

    def update(self, display_frame) -> tuple:
        fg = self._bg.apply(display_frame,
                            learningRate=config.MOG2_LEARNING_RATE)
        fg = cv2.morphologyEx(fg,cv2.MORPH_OPEN,self._kernel)
        fg = cv2.dilate(fg,self._kernel,iterations=1)
        if self._table_mask is not None:
            fg = cv2.bitwise_and(fg,self._table_mask)
        gray = cv2.cvtColor(display_frame,cv2.COLOR_BGR2GRAY)

        blob_a = self._tkr_a.local_search(fg) if self._tkr_a.initialized else None
        blob_b = self._tkr_b.local_search(fg) if self._tkr_b.initialized else None

        # Blob mutex
        if blob_a and blob_b:
            dx=blob_a["cx"]-blob_b["cx"]; dy=blob_a["cy"]-blob_b["cy"]
            if np.sqrt(dx*dx+dy*dy) < config.SEARCH_RADIUS*0.6:
                pa=self._tkr_a.predicted_pos(); pb=self._tkr_b.predicted_pos()
                da=np.sqrt((blob_a["cx"]-pa[0])**2+(blob_a["cy"]-pa[1])**2)
                db=np.sqrt((blob_b["cx"]-pb[0])**2+(blob_b["cy"]-pb[1])**2)
                if da<=db: blob_b=None
                else:      blob_a=None

        # Record path from Train A when recording
        if blob_a and self.track_path.recording:
            self.track_path.record(blob_a["cx"], blob_a["cy"])

        self._tkr_a.update_with_blob(blob_a, gray, self.track_path)
        self._tkr_b.update_with_blob(blob_b, gray, self.track_path)

        return self._tkr_a.get_position(), self._tkr_b.get_position()

    def a_is_chasing_b(self, pos_a, pos_b) -> bool:
        if pos_a is None or pos_b is None: return False
        avx, avy = self._tkr_a.velocity
        spd = np.sqrt(avx**2+avy**2)
        if spd < 0.3: return False
        dx=pos_b.x-pos_a.x; dy=pos_b.y-pos_a.y
        return (avx*dx+avy*dy) > 0

    def facing_gap(self, pos_a, pos_b, a_chasing: bool):
        if pos_a is None or pos_b is None: return None
        dx=pos_a.x-pos_b.x; dy=pos_a.y-pos_b.y
        d=float(np.sqrt(dx*dx+dy*dy))
        return max(0.0, d-pos_a.radius-pos_b.radius)

    def is_a_missing(self) -> bool:
        return not self._tkr_a.initialized or self._tkr_a.is_missing()

    def is_b_missing(self) -> bool:
        return not self._tkr_b.initialized or self._tkr_b.is_missing()

    def instruction_text(self) -> str:
        if self.state==WAIT_TABLE:
            return "STEP 1 — DRAG a box around the TABLE (whole track area)"
        elif self.state==WAIT_A:
            return "STEP 2 — DRAG a box around TRAIN A (front train)"
        elif self.state==WAIT_B:
            return "STEP 3 — DRAG a box around TRAIN B (rear BLE train)"
        la=("LOCKED" if self._tkr_a.is_locked else
            f"OK {self._tkr_a.confidence:.0%}" if self._tkr_a.active else "LOST")
        lb=("LOCKED" if self._tkr_b.is_locked else
            f"OK {self._tkr_b.confidence:.0%}" if self._tkr_b.active else "LOST")
        path_s = " | PATH:REC" if self.track_path.recording else (
                 " | PATH:ON" if self.track_path.has_path else "")
        return f"A:{la}   B:{lb}{path_s}"


def pixel_distance(a, b):
    if a is None or b is None: return None
    return float(np.sqrt((a.x-b.x)**2+(a.y-b.y)**2))