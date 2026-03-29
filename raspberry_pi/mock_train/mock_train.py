"""
mock_train.py  —  LionChief Collision Prevention Simulation
============================================================
Peter's O-Scale Layout (15ft x 20ft)

COLLISION LOGIC (correct):
  Point A = entry  of shared section (left junction)
  Point B = exit   of shared section (right junction)

  Rule 1: If outer train enters shared (crosses A) → stop inner train
          AT its waiting spot just BEFORE point A on inner path.
          Inner waits there until outer exits from B, then resumes.

  Rule 2: If inner train enters shared (crosses A) → stop outer train
          AT its waiting spot just BEFORE point A on outer path.
          Outer waits there until inner exits from B, then resumes.

  Rule 3: If a train is ALREADY PAST A (inside shared or past B)
          when the other train arrives, the arriving train MUST WAIT
          before A. The in-shared train finishes its run to B first.

CONTROLS:
  On-screen buttons  (click with mouse)
  Keyboard: Q / ESC = quit

Run:  pip install pygame  →  python mock_train.py
"""

import pygame
import math
import sys
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass, field

# ── WINDOW ─────────────────────────────────────────────────────────────────
WIDTH, HEIGHT  = 1280, 760
LAYOUT_W       = 820     # left panel width
FPS            = 60
TITLE          = "LionChief Collision Prevention — Peter's Layout"

# ── LAYOUT CENTRE ──────────────────────────────────────────────────────────
CX, CY = 400, 368

# ── OUTER OVAL ─────────────────────────────────────────────────────────────
O_CX, O_CY = CX, CY
O_RX, O_RY = 318, 215

# ── SHARED SECTION ─────────────────────────────────────────────────────────
# Defined as angle range on the OUTER oval (left side of layout)
A_ANG = 148    # Point A  — entry junction (lower-left)
B_ANG = 212    # Point B  — exit  junction (upper-left)

# Wait positions: each train waits here when blocked (just before A)
# Expressed as fraction (0–1) along each train's full path
OUTER_WAIT_FRAC = 0.966   # outer path fraction ≈ just before A
INNER_WAIT_FRAC = 0.968   # inner path fraction ≈ just before A

# ── COLOURS ────────────────────────────────────────────────────────────────
C_BG          = (10,  15,  24)
C_FELT        = (32,  78,  44)
C_FELT_EDGE   = (22,  58,  32)
C_GRID        = (24,  58,  36)
C_BALLAST     = (88,  82,  72)    # gravel ballast colour
C_SLEEPER     = (95,  62,  30)
C_RAIL        = (192, 192, 202)
C_SHARED_OK   = (48,  215, 92)
C_SHARED_WARN = (255, 88,  28)
C_OUTER_T     = (42,  148, 255)
C_INNER_T     = (255, 180,   0)
C_STOPPED     = (255,  48,  48)
C_WAITING     = (255, 210,   0)
C_TEXT        = (225, 225, 225)
C_DIM         = (108, 112, 125)
C_BORDER      = (44,  54,  74)
C_GREEN       = (48,  205, 78)
C_RED         = (220,  44, 44)
C_ORANGE      = (255, 152,   0)
C_YELLOW      = (255, 222,   0)
C_WHITE       = (255, 255, 255)
C_JUNCTION    = (255, 228,   0)
C_BTN_NORM    = (38,  48,  68)
C_BTN_HOV     = (55,  68,  95)
C_BTN_PRESS   = (28,  38,  55)

# ── BUILDINGS ──────────────────────────────────────────────────────────────
BUILDINGS = [
    (CX-18,  CY-52,  64, 40, (138, 98,  78), "Station"),
    (CX+96,  CY-72,  50, 36, (78,  108, 88), "Depot"),
    (CX-114, CY-76,  44, 30, (98,  128, 98), "Tower"),
    (CX+46,  CY+26,  54, 34, (128, 88,  78), "Yard"),
    (CX-68,  CY+36,  44, 30, (88,  108, 78), "House"),
    (CX+136, CY-14,  40, 30, (108, 98,  88), "Office"),
    (CX-10,  CY+72,  50, 26, (98,  118, 92), "Barn"),
    (CX+68,  CY-146, 34, 50, (198, 165, 26), "Crane"),
]


# ─────────────────────────────────────────────────────────────────────────────
#  GEOMETRY
# ─────────────────────────────────────────────────────────────────────────────

def oval_pt(cx, cy, rx, ry, deg):
    a = math.radians(deg)
    return (cx + rx * math.cos(a), cy + ry * math.sin(a))


def arc_pts(cx, cy, rx, ry, s, e, n=120):
    return [oval_pt(cx, cy, rx, ry, s + (e - s) * i / n) for i in range(n + 1)]


def bezier_pt(ctrl, t):
    pts = list(ctrl)
    while len(pts) > 1:
        pts = [(pts[i][0]*(1-t)+pts[i+1][0]*t,
                pts[i][1]*(1-t)+pts[i+1][1]*t)
               for i in range(len(pts)-1)]
    return pts[0]


def sample_bezier(ctrl, n):
    return [bezier_pt(ctrl, i / n) for i in range(n + 1)]


def build_cum(pts):
    cum = [0.0]
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i-1][0]
        dy = pts[i][1] - pts[i-1][1]
        cum.append(cum[-1] + math.hypot(dx, dy))
    return cum


def pos_at_t(pts, cum, t):
    t = max(0.0, min(1.0, t))
    target = t * cum[-1]
    lo, hi = 0, len(pts) - 1
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if cum[mid] <= target:
            lo = mid
        else:
            hi = mid
    seg  = cum[hi] - cum[lo]
    frac = (target - cum[lo]) / seg if seg > 1e-9 else 0.0
    x = pts[lo][0] + frac * (pts[hi][0] - pts[lo][0])
    y = pts[lo][1] + frac * (pts[hi][1] - pts[lo][1])
    return (int(x), int(y))


def angle_at_t(pts, cum, t):
    """Direction angle at position t (for train orientation)."""
    t  = max(0.001, min(0.999, t))
    p1 = pos_at_t(pts, cum, t - 0.005)
    p2 = pos_at_t(pts, cum, t + 0.005)
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


# ─────────────────────────────────────────────────────────────────────────────
#  BUILD PATHS
# ─────────────────────────────────────────────────────────────────────────────

def build_paths():
    """
    OUTER path  = full outer oval (360°), starting & ending at point A.
    INNER path  = shared section (A→B)  +  private inner loop (B back to A).

    t = 0 on outer path  ≡  Point A entry
    t = sf_outer         ≡  Point B exit  on outer path

    t = 0 on inner path  ≡  Point A entry
    t = se_inner         ≡  Point B exit  on inner path
    """
    pt_A = oval_pt(O_CX, O_CY, O_RX, O_RY, A_ANG)   # entry junction
    pt_B = oval_pt(O_CX, O_CY, O_RX, O_RY, B_ANG)   # exit  junction

    # ── OUTER FULL PATH (starts at A_ANG, clockwise 360°) ──────────────────
    outer_pts = arc_pts(O_CX, O_CY, O_RX, O_RY, A_ANG, A_ANG + 360, 600)
    outer_cum = build_cum(outer_pts)
    # fraction where outer train reaches point B
    sf_outer  = (B_ANG - A_ANG) / 360.0   # ≈ 0.178

    # ── SHARED SECTION (A → B on outer oval) ───────────────────────────────
    shared_pts = arc_pts(O_CX, O_CY, O_RX, O_RY, A_ANG, B_ANG, 100)

    # ── INNER PRIVATE SECTION (B → A via bezier, going right/inside) ────────
    # Large smooth curve going to the right, matching photo layout
    ctrl = [
        pt_B,
        (pt_B[0] + 55,  pt_B[1] - 42),
        (O_CX - 35,     O_CY - 205),
        (O_CX + 210,    O_CY - 188),
        (O_CX + 252,    O_CY      ),
        (O_CX + 210,    O_CY + 192),
        (O_CX - 35,     O_CY + 202),
        (pt_A[0] + 55,  pt_A[1] + 42),
        pt_A,
    ]
    inner_priv = sample_bezier(ctrl, 320)

    # ── INNER FULL PATH  =  shared_pts  +  inner_priv (skip duplicate pt_B) ─
    inner_pts = shared_pts + inner_priv[1:]
    inner_cum = build_cum(inner_pts)
    # fraction where inner train exits shared section (reaches B)
    shared_len  = build_cum(shared_pts)[-1]
    se_inner    = shared_len / inner_cum[-1]

    return dict(
        outer_pts  = outer_pts,
        outer_cum  = outer_cum,
        sf_outer   = sf_outer,

        inner_pts  = inner_pts,
        inner_cum  = inner_cum,
        se_inner   = se_inner,

        shared_pts = shared_pts,
        inner_priv = inner_priv,
        pt_A       = pt_A,
        pt_B       = pt_B,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  TRAIN
# ─────────────────────────────────────────────────────────────────────────────

class TrainState:
    RUNNING = "running"
    WAITING = "waiting"    # stopped before point A
    BRAKING = "braking"    # slowing down toward wait pos

@dataclass
class Train:
    name:      str
    color:     tuple
    pts:       list
    cum:       list
    t:         float = 0.0
    speed:     float = 0.0015
    state:     str   = TrainState.RUNNING
    wait_t:    float = OUTER_WAIT_FRAC
    trail:     list  = field(default_factory=list)
    TRAIL:     int   = 150

    @property
    def pos(self):
        return pos_at_t(self.pts, self.cum, self.t)

    @property
    def heading(self):
        return angle_at_t(self.pts, self.cum, self.t)

    @property
    def stopped(self):
        return self.state in (TrainState.WAITING, TrainState.BRAKING)

    def update(self):
        if self.state == TrainState.RUNNING:
            self.t = (self.t + self.speed) % 1.0

        elif self.state == TrainState.BRAKING:
            # Crawl toward wait position
            dist = self.wait_t - self.t
            if dist < 0:
                dist += 1.0
            step = min(self.speed * 0.4, dist)
            if dist < 0.003:
                self.t     = self.wait_t
                self.state = TrainState.WAITING
            else:
                self.t = (self.t + step) % 1.0

        # elif WAITING: do nothing

        self.trail.append((self.t, self.state))
        if len(self.trail) > self.TRAIL:
            self.trail.pop(0)

    def stop_before_A(self):
        """Order train to stop at waiting position before point A."""
        if self.state == TrainState.RUNNING:
            self.state = TrainState.BRAKING

    def resume(self):
        """Allow train to proceed."""
        self.state = TrainState.RUNNING


# ─────────────────────────────────────────────────────────────────────────────
#  COLLISION CONTROLLER
# ─────────────────────────────────────────────────────────────────────────────

class CollisionController:
    """
    Manages the shared section lock.

    LOCK held by outer → inner must wait before A.
    LOCK held by inner → outer must wait before A.
    LOCK free          → both may enter.
    """
    NONE  = "none"
    OUTER = "outer"
    INNER = "inner"

    def __init__(self, paths):
        self.p          = paths
        self.lock       = self.NONE
        self.log: List[str] = []
        self.stop_ct    = 0
        self.resume_ct  = 0

    def _outer_in_shared(self, outer):
        return 0.0 <= outer.t <= self.p["sf_outer"]

    def _inner_in_shared(self, inner):
        return 0.0 <= inner.t <= self.p["se_inner"]

    def _outer_past_B(self, outer):
        return outer.t > self.p["sf_outer"]

    def _inner_past_B(self, inner):
        return inner.t > self.p["se_inner"]

    def tick(self, outer: Train, inner: Train):
        outer_in = self._outer_in_shared(outer)
        inner_in = self._inner_in_shared(inner)

        # ── Outer enters shared → lock for outer, halt inner ────────────────
        if outer_in and self.lock == self.NONE:
            self.lock = self.OUTER
            self._halt(inner, "inner", "outer entered shared section")

        # ── Inner enters shared → lock for inner, halt outer ────────────────
        elif inner_in and self.lock == self.NONE:
            self.lock = self.INNER
            self._halt(outer, "outer", "inner entered shared section")

        # ── Outer exits shared (past B) → check if lock can release ─────────
        if self.lock == self.OUTER and self._outer_past_B(outer):
            # outer fully exited → free lock
            self.lock = self.NONE
            self._resume(inner, "inner", "outer exited shared section")

        # ── Inner exits shared (past B) → check if lock can release ─────────
        if self.lock == self.INNER and self._inner_past_B(inner):
            self.lock = self.NONE
            self._resume(outer, "outer", "inner exited shared section")

    def emergency_stop(self, train: Train, name: str):
        train.state = TrainState.WAITING
        self.log.append(f"🚨 EMERGENCY STOP {name.upper()}")
        self.stop_ct += 1
        print(f"[EMERGENCY] {name} stopped")

    def manual_resume(self, train: Train, name: str):
        # Only resume if lock isn't held against this train
        train.resume()
        self.resume_ct += 1
        self.log.append(f"▶ Manual resume {name.upper()}")
        print(f"[RESUME] {name} resumed manually")

    def _halt(self, train: Train, name: str, reason: str):
        train.stop_before_A()
        self.stop_ct += 1
        msg = f"⚠ HOLD {name.upper()} → {reason[:28]}"
        self.log.append(msg)
        print(f"[CTRL] {msg}")
        print(f"       BLE → STOP  [0x00, 0x45, 0x00]")

    def _resume(self, train: Train, name: str, reason: str):
        train.resume()
        self.resume_ct += 1
        msg = f"✓ FREE {name.upper()} → {reason[:28]}"
        self.log.append(msg)
        print(f"[CTRL] {msg}")
        print(f"       BLE → RESUME [0x00, 0x45, 0x07]")

    @property
    def zone_active(self):
        return self.lock != self.NONE


# ─────────────────────────────────────────────────────────────────────────────
#  UI BUTTON
# ─────────────────────────────────────────────────────────────────────────────

class Button:
    def __init__(self, x, y, w, h, label, color=None, text_col=C_WHITE):
        self.rect      = pygame.Rect(x, y, w, h)
        self.label     = label
        self.color     = color or C_BTN_NORM
        self.text_col  = text_col
        self.hovered   = False
        self.pressed   = False
        self._font     = None

    def draw(self, surf):
        if self._font is None:
            self._font = pygame.font.SysFont("consolas", 12, bold=True)
        col = (C_BTN_PRESS if self.pressed else
               C_BTN_HOV  if self.hovered else self.color)
        pygame.draw.rect(surf, col,        self.rect, border_radius=5)
        pygame.draw.rect(surf, C_BORDER,   self.rect, 1, border_radius=5)
        t = self._font.render(self.label, True, self.text_col)
        surf.blit(t, (self.rect.centerx - t.get_width()  // 2,
                      self.rect.centery - t.get_height() // 2))

    def handle(self, event):
        """Returns True if button was clicked this event."""
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.pressed = True
                return True
        if event.type == pygame.MOUSEBUTTONUP:
            self.pressed = False
        return False


# ─────────────────────────────────────────────────────────────────────────────
#  DRAW HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def draw_felt(surf):
    # Shadow
    pygame.draw.rect(surf, (6, 10, 18),
                     (8, 8, LAYOUT_W - 2, HEIGHT - 16), border_radius=14)
    # Felt surface
    pygame.draw.rect(surf, C_FELT,
                     (10, 10, LAYOUT_W - 4, HEIGHT - 20), border_radius=12)
    # Grid
    for x in range(10, LAYOUT_W - 4, 32):
        pygame.draw.line(surf, C_GRID, (x, 10), (x, HEIGHT - 10))
    for y in range(10, HEIGHT - 10, 32):
        pygame.draw.line(surf, C_GRID, (10, y), (LAYOUT_W - 4, y))


def draw_track_ballast(surf, pts, w_ballast=14):
    """Draw gravel ballast under the track."""
    if len(pts) < 2:
        return
    for i in range(0, len(pts) - 1, 3):
        p0, p1 = pts[i], pts[i+1]
        dx, dy  = p1[0]-p0[0], p1[1]-p0[1]
        ln      = math.hypot(dx, dy)
        if ln < 1e-6:
            continue
        nx, ny = -dy/ln * w_ballast//2, dx/ln * w_ballast//2
        mx, my = (p0[0]+p1[0])/2, (p0[1]+p1[1])/2
        # Draw ballast as small polygon
        poly = [(int(mx+nx), int(my+ny)),
                (int(mx-nx), int(my-ny)),
                (int(p1[0]-nx), int(p1[1]-ny)),
                (int(p1[0]+nx), int(p1[1]+ny))]
        pygame.draw.polygon(surf, C_BALLAST, poly)


def draw_track(surf, pts, sleeper_gap=10):
    """Draw realistic track: ballast → sleepers → twin rails."""
    if len(pts) < 2:
        return
    ipts = [(int(p[0]), int(p[1])) for p in pts]

    # Ballast
    draw_track_ballast(surf, pts)

    # Sleepers (crossties)
    for i in range(0, len(pts)-1, sleeper_gap):
        p0, p1 = pts[i], pts[i+1]
        dx, dy  = p1[0]-p0[0], p1[1]-p0[1]
        ln      = math.hypot(dx, dy)
        if ln < 1e-6:
            continue
        nx, ny = -dy/ln * 10, dx/ln * 10
        mx, my = (p0[0]+p1[0])/2, (p0[1]+p1[1])/2
        pygame.draw.line(surf, C_SLEEPER,
                         (int(mx-nx), int(my-ny)),
                         (int(mx+nx), int(my+ny)), 4)

    # Twin rails
    for side in (-4, 4):
        rail = []
        for i in range(len(pts)-1):
            p0, p1 = pts[i], pts[i+1]
            dx, dy = p1[0]-p0[0], p1[1]-p0[1]
            ln     = math.hypot(dx, dy)
            if ln < 1e-6:
                continue
            nx, ny = -dy/ln*side, dx/ln*side
            rail.append((int(p0[0]+nx), int(p0[1]+ny)))
        if len(rail) > 1:
            pygame.draw.lines(surf, C_RAIL, False, rail, 2)


def draw_shared_highlight(surf, shared_pts, zone_active, flash):
    if len(shared_pts) < 2:
        return
    base  = C_SHARED_WARN if zone_active else C_SHARED_OK
    pulse = int(30 + 20 * math.sin(flash))
    col   = tuple(min(255, c + pulse) for c in base)
    ipts  = [(int(p[0]), int(p[1])) for p in shared_pts]

    # Glow
    for w in (14, 9, 5):
        gs = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        pygame.draw.lines(gs, (*col, 25), False, ipts, w)
        surf.blit(gs, (0, 0))
    pygame.draw.lines(surf, col, False, ipts, 4)

    # Shared track label
    mid  = shared_pts[len(shared_pts)//2]
    font = pygame.font.SysFont("consolas", 12, bold=True)
    lbl  = "⚠ SHARED IN USE" if zone_active else "── SHARED TRACK ──"
    t    = font.render(lbl, True, col)
    surf.blit(t, (int(mid[0]) - t.get_width()//2,
                  int(mid[1]) - 22))


def draw_junction(surf, pt, label="", size=9):
    x, y = int(pt[0]), int(pt[1])
    # Outer glow
    gs = pygame.Surface((size*4, size*4), pygame.SRCALPHA)
    pygame.draw.circle(gs, (*C_JUNCTION, 60), (size*2, size*2), size*2)
    surf.blit(gs, (x - size*2, y - size*2))
    # Diamond
    diamond = [(x, y-size), (x+size, y), (x, y+size), (x-size, y)]
    pygame.draw.polygon(surf, C_JUNCTION, diamond)
    pygame.draw.polygon(surf, C_BG,       diamond, 2)
    pygame.draw.circle(surf, C_WHITE, (x, y), 3)
    if label:
        font = pygame.font.SysFont("consolas", 13, bold=True)
        t    = font.render(label, True, C_YELLOW)
        surf.blit(t, (x + size + 4, y - 8))


def draw_train(surf, train: Train):
    """Draw realistic train: locomotive + 2 cars + trail."""
    if not train.trail:
        return

    # Trail
    total = len(train.trail)
    for i, (tt, state) in enumerate(train.trail):
        alpha = i / total
        size  = max(2, int(10 * alpha))
        col   = (C_STOPPED  if state == TrainState.WAITING else
                 C_WAITING  if state == TrainState.BRAKING else
                 train.color)
        faded = tuple(int(c * alpha * 0.65) for c in col)
        p = pos_at_t(train.pts, train.cum, tt)
        pygame.draw.circle(surf, faded, p, size)

    # Current head color
    hcol = (C_STOPPED  if train.state == TrainState.WAITING else
            C_WAITING  if train.state == TrainState.BRAKING else
            train.color)

    # Draw 2 trailing cars first (behind loco)
    heading = train.heading
    for i, (car_back, car_size) in enumerate([(0.022, 9), (0.044, 8)]):
        ct  = (train.t - car_back) % 1.0
        cp  = pos_at_t(train.pts, train.cum, ct)
        ch  = angle_at_t(train.pts, train.cum, ct)
        ccol = tuple(max(0, c - 40 - i*20) for c in hcol)
        # Car body
        _draw_car(surf, cp, ch, car_size, ccol)

    # Draw locomotive (head)
    head = train.pos
    _draw_loco(surf, head, heading, hcol, train.name)


def _draw_car(surf, pos, heading, size, col):
    """Draw a single train car as a rectangle aligned to heading."""
    x, y    = pos
    cos_h   = math.cos(heading)
    sin_h   = math.sin(heading)
    hw, hh  = size * 2.2, size * 0.9
    corners = []
    for dx, dy in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]:
        rx = dx * cos_h - dy * sin_h + x
        ry = dx * sin_h + dy * cos_h + y
        corners.append((int(rx), int(ry)))
    pygame.draw.polygon(surf, col, corners)
    pygame.draw.polygon(surf, tuple(min(255,c+30) for c in col), corners, 1)


def _draw_loco(surf, pos, heading, col, name):
    """Draw the locomotive with glow, body, and name tag."""
    x, y = pos
    # Glow
    for r in range(18, 7, -3):
        gs   = pygame.Surface((r*2, r*2), pygame.SRCALPHA)
        gcol = tuple(min(255, c+40) for c in col)
        pygame.draw.circle(gs, (*gcol, max(0,160-r*13)), (r,r), r)
        surf.blit(gs, (x-r, y-r))
    # Body rectangle (longer nose facing heading direction)
    cos_h = math.cos(heading); sin_h = math.sin(heading)
    hw, hh = 14, 8
    corners = []
    for dx, dy in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]:
        rx = dx*cos_h - dy*sin_h + x
        ry = dx*sin_h + dy*cos_h + y
        corners.append((int(rx), int(ry)))
    pygame.draw.polygon(surf, col, corners)
    pygame.draw.polygon(surf, C_WHITE, corners, 1)
    # Headlight
    hl_x = int(x + 13*cos_h); hl_y = int(y + 13*sin_h)
    pygame.draw.circle(surf, (255, 255, 200), (hl_x, hl_y), 3)
    # Cab window
    pygame.draw.circle(surf, C_WHITE, pos, 5)
    pygame.draw.circle(surf, col,     pos, 5)
    # Name tag
    font = pygame.font.SysFont("consolas", 11, bold=True)
    t    = font.render(name, True, col)
    surf.blit(t, (x + 16, y - 8))


def draw_buildings(surf):
    font = pygame.font.SysFont("consolas", 10)
    for bx, by, bw, bh, col, label in BUILDINGS:
        # Shadow
        pygame.draw.rect(surf, (5,14,6),  (bx+3,by+3,bw,bh), border_radius=3)
        # Body
        pygame.draw.rect(surf, col,        (bx,by,bw,bh), border_radius=3)
        # Roof
        roof = tuple(max(0, c-45) for c in col)
        pygame.draw.rect(surf, roof, (bx,by,bw,bh//3), border_radius=3)
        # Border
        pygame.draw.rect(surf, (190,190,190), (bx,by,bw,bh), 1, border_radius=3)
        # Label
        t = font.render(label, True, C_WHITE)
        surf.blit(t, (bx+bw//2-t.get_width()//2, by+bh-13))


def draw_legend(surf):
    fb = pygame.font.SysFont("consolas", 12, bold=True)
    fs = pygame.font.SysFont("consolas", 11)
    t  = fb.render("Peter's O-Scale Layout  (15ft × 20ft)", True, C_DIM)
    surf.blit(t, (16, 15))
    items = [("●", C_OUTER_T,     "Outer loop"),
             ("●", C_INNER_T,     "Inner loop"),
             ("━", C_SHARED_OK,   "Shared track — clear"),
             ("━", C_SHARED_WARN, "Shared track — occupied"),
             ("◆", C_JUNCTION,    "Junction A / B")]
    lx, ly = 16, HEIGHT - 10 - len(items)*16
    for sym, col, desc in items:
        surf.blit(fs.render(sym,         True, col),   (lx,    ly))
        surf.blit(fs.render(f"  {desc}", True, C_DIM), (lx+8,  ly))
        ly += 16


# ─────────────────────────────────────────────────────────────────────────────
#  RIGHT PANEL
# ─────────────────────────────────────────────────────────────────────────────

def draw_panel(surf, outer, inner, ctrl, log, elapsed,
               buttons, font_cache):
    PX, PY = LAYOUT_W + 2, 6
    PW, PH = WIDTH - LAYOUT_W - 8, HEIGHT - 12

    bg = pygame.Surface((PW, PH), pygame.SRCALPHA)
    bg.fill((14, 20, 34, 232))
    surf.blit(bg, (PX, PY))
    pygame.draw.rect(surf, C_BORDER, (PX, PY, PW, PH), 1, border_radius=5)

    fb = font_cache["b14"]
    fm = font_cache["m13"]
    fs = font_cache["s11"]

    y = PY + 10

    # Title
    t = fb.render("LIONCHIEF  MONITOR", True, C_YELLOW)
    surf.blit(t, (PX+PW//2 - t.get_width()//2, y)); y += 20
    t = fs.render(f"Runtime: {int(elapsed//60):02d}:{int(elapsed%60):02d}",
                  True, C_DIM)
    surf.blit(t, (PX+PW//2 - t.get_width()//2, y)); y += 18
    pygame.draw.line(surf, C_BORDER, (PX+5,y),(PX+PW-5,y)); y += 8

    # Zone status
    zc = C_RED if ctrl.zone_active else C_GREEN
    zt = (f"⚠ LOCKED — {ctrl.lock.upper()} IN SHARED"
          if ctrl.zone_active else "✓  SHARED ZONE CLEAR")
    t  = fm.render(zt, True, zc)
    surf.blit(t, (PX+PW//2-t.get_width()//2, y)); y += 22
    pygame.draw.line(surf, C_BORDER, (PX+5,y),(PX+PW-5,y)); y += 8

    # ── Train cards ──────────────────────────────────────────────────────────
    for train, role, in_shared in [
        (outer, "OUTER LOOP", 0.0<=outer.t<=ctrl.p["sf_outer"]),
        (inner, "INNER LOOP", 0.0<=inner.t<=ctrl.p["se_inner"]),
    ]:
        bx, by_, bw, bh = PX+6, y, PW-12, 72
        bc   = C_STOPPED if train.state == TrainState.WAITING  else \
               C_WAITING if train.state == TrainState.BRAKING  else C_GREEN
        bg2  = tuple(c//6 for c in bc)
        pygame.draw.rect(surf, bg2, (bx,by_,bw,bh), border_radius=4)
        pygame.draw.rect(surf, bc,  (bx,by_,bw,bh), 1, border_radius=4)

        t  = fm.render(role, True, train.color)
        surf.blit(t, (bx+8, by_+4))

        st_txt = ("🛑 STOPPED"     if train.state == TrainState.WAITING else
                  "🟡 BRAKING"    if train.state == TrainState.BRAKING  else
                  "✓  RUNNING")
        t2 = fm.render(st_txt, True, bc)
        surf.blit(t2, (bx+8, by_+22))

        sp = int(train.speed * 10000)
        t3 = fs.render(f"Speed:{sp:>3}   Pos:{train.t:.3f}", True, C_DIM)
        surf.blit(t3, (bx+8, by_+40))

        # Zone dot indicator
        dc = C_ORANGE if in_shared else C_DIM
        pygame.draw.circle(surf, dc, (bx+bw-16, by_+bh//2), 7)
        if in_shared:
            pygame.draw.circle(surf, C_WHITE, (bx+bw-16, by_+bh//2), 4)

        # Speed buttons (drawn inside card)
        buttons[f"{role[:5]}_plus"].rect  = pygame.Rect(bx+bw-84, by_+bh-26, 36, 20)
        buttons[f"{role[:5]}_minus"].rect = pygame.Rect(bx+bw-46, by_+bh-26, 36, 20)
        buttons[f"{role[:5]}_plus"].draw(surf)
        buttons[f"{role[:5]}_minus"].draw(surf)

        y += bh + 6

    pygame.draw.line(surf, C_BORDER, (PX+5,y),(PX+PW-5,y)); y += 8

    # ── Emergency stop buttons ───────────────────────────────────────────────
    bw2 = (PW - 18) // 2
    buttons["stop_outer"].rect  = pygame.Rect(PX+6,       y, bw2, 30)
    buttons["stop_inner"].rect  = pygame.Rect(PX+8+bw2,   y, bw2, 30)
    buttons["stop_outer"].draw(surf)
    buttons["stop_inner"].draw(surf)
    y += 36

    buttons["res_outer"].rect = pygame.Rect(PX+6,       y, bw2, 28)
    buttons["res_inner"].rect = pygame.Rect(PX+8+bw2,   y, bw2, 28)
    buttons["res_outer"].draw(surf)
    buttons["res_inner"].draw(surf)
    y += 34

    pygame.draw.line(surf, C_BORDER, (PX+5,y),(PX+PW-5,y)); y += 8

    # ── Stats ────────────────────────────────────────────────────────────────
    for lbl, val, col in [("Auto stops:",  str(ctrl.stop_ct),   C_RED),
                            ("Auto resumes:",str(ctrl.resume_ct), C_GREEN)]:
        t  = fm.render(lbl, True, C_DIM);  surf.blit(t,  (PX+8, y))
        tv = fm.render(val, True, col);    surf.blit(tv, (PX+PW-tv.get_width()-10, y))
        y += 17
    y += 4
    pygame.draw.line(surf, C_BORDER, (PX+5,y),(PX+PW-5,y)); y += 8

    # ── BLE log ───────────────────────────────────────────────────────────────
    t = fm.render("EVENT LOG", True, C_YELLOW)
    surf.blit(t, (PX+8, y)); y += 17
    for line in ctrl.log[-12:]:
        c = (C_RED    if "STOP"   in line or "HOLD" in line or "EMERGENCY" in line else
             C_GREEN  if "FREE"   in line or "RESUM" in line else
             C_ORANGE if "⚠"     in line else C_DIM)
        t = fs.render(line[:36], True, c)
        surf.blit(t, (PX+6, y)); y += 13


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    pygame.init()
    pygame.display.set_caption(TITLE)
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock  = pygame.time.Clock()

    # Font cache
    fonts = {
        "b14": pygame.font.SysFont("consolas", 14, bold=True),
        "m13": pygame.font.SysFont("consolas", 13),
        "s11": pygame.font.SysFont("consolas", 11),
    }

    print("Building track paths...", end=" ", flush=True)
    paths = build_paths()
    print("done ✅\n")

    outer = Train(
        name   = "OUTER",
        color  = C_OUTER_T,
        pts    = paths["outer_pts"],
        cum    = paths["outer_cum"],
        t      = 0.45,       # start well past shared section
        speed  = 0.00125,
        wait_t = OUTER_WAIT_FRAC,
    )
    inner = Train(
        name   = "INNER",
        color  = C_INNER_T,
        pts    = paths["inner_pts"],
        cum    = paths["inner_cum"],
        t      = paths["se_inner"] + 0.08,   # start past shared section
        speed  = 0.00170,
        wait_t = INNER_WAIT_FRAC,
    )

    ctrl = CollisionController(paths)

    # ── Buttons ───────────────────────────────────────────────────────────────
    # Positions filled in by draw_panel; initial rects are placeholders
    R = pygame.Rect
    buttons = {
        "OUTER_plus":  Button(0,0,36,20, " + ",  (28,68,28),  C_GREEN),
        "OUTER_minus": Button(0,0,36,20, " - ",  (68,28,28),  C_RED),
        "INNER_plus":  Button(0,0,36,20, " + ",  (28,68,28),  C_GREEN),
        "INNER_minus": Button(0,0,36,20, " - ",  (68,28,28),  C_RED),
        "stop_outer":  Button(0,0,100,30,"🛑 STOP OUTER", (80,20,20), C_WHITE),
        "stop_inner":  Button(0,0,100,30,"🛑 STOP INNER", (80,20,20), C_WHITE),
        "res_outer":   Button(0,0,100,28,"▶ RESUME OUTER",(20,60,28), C_WHITE),
        "res_inner":   Button(0,0,100,28,"▶ RESUME INNER",(20,60,28), C_WHITE),
    }

    t_start = time.time()
    flash_t = 0.0
    running = True

    print("╔══════════════════════════════════════════╗")
    print("║  LionChief Collision Prevention Sim      ║")
    print("╠══════════════════════════════════════════╣")
    print("║  COLLISION LOGIC:                        ║")
    print("║  A = entry of shared, B = exit           ║")
    print("║  Train in shared → other train waits     ║")
    print("║                    BEFORE point A        ║")
    print("╠══════════════════════════════════════════╣")
    print("║  Click on-screen buttons or press Q/ESC  ║")
    print("╚══════════════════════════════════════════╝\n")

    while running:
        clock.tick(FPS)
        flash_t += 0.07

        # ── Events ─────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            if ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

            # Button handling
            if buttons["OUTER_plus"].handle(ev):
                outer.speed = min(0.006, outer.speed + 0.00025)
                ctrl.log.append(f"Outer speed ↑ {int(outer.speed*10000)}")
            if buttons["OUTER_minus"].handle(ev):
                outer.speed = max(0.0003, outer.speed - 0.00025)
                ctrl.log.append(f"Outer speed ↓ {int(outer.speed*10000)}")
            if buttons["INNER_plus"].handle(ev):
                inner.speed = min(0.006, inner.speed + 0.00025)
                ctrl.log.append(f"Inner speed ↑ {int(inner.speed*10000)}")
            if buttons["INNER_minus"].handle(ev):
                inner.speed = max(0.0003, inner.speed - 0.00025)
                ctrl.log.append(f"Inner speed ↓ {int(inner.speed*10000)}")
            if buttons["stop_outer"].handle(ev):
                ctrl.emergency_stop(outer, "outer")
            if buttons["stop_inner"].handle(ev):
                ctrl.emergency_stop(inner, "inner")
            if buttons["res_outer"].handle(ev):
                ctrl.manual_resume(outer, "outer")
            if buttons["res_inner"].handle(ev):
                ctrl.manual_resume(inner, "inner")

            # Mouse hover for all buttons
            for btn in buttons.values():
                btn.handle(ev)

        # ── Collision control ───────────────────────────────────────────────
        ctrl.tick(outer, inner)

        # ── Update trains ───────────────────────────────────────────────────
        outer.update()
        inner.update()

        # ── Draw ────────────────────────────────────────────────────────────
        screen.fill(C_BG)
        draw_felt(screen)
        draw_buildings(screen)

        # Outer track (non-shared part: B_ANG → A_ANG going clockwise)
        outer_non_shared = arc_pts(O_CX, O_CY, O_RX, O_RY,
                                   B_ANG, A_ANG + 360, 380)
        draw_track(screen, outer_non_shared, sleeper_gap=11)

        # Inner private track
        draw_track(screen, paths["inner_priv"], sleeper_gap=10)

        # Shared section (ONE track, drawn on top)
        draw_track(screen, paths["shared_pts"], sleeper_gap=9)
        draw_shared_highlight(screen, paths["shared_pts"],
                              ctrl.zone_active, flash_t)

        # Junction A and B markers
        draw_junction(screen, paths["pt_A"], "A")
        draw_junction(screen, paths["pt_B"], "B")

        # Trains
        draw_train(screen, outer)
        draw_train(screen, inner)

        draw_legend(screen)

        # Right panel (buttons get positioned inside)
        draw_panel(screen, outer, inner, ctrl, ctrl.log,
                   time.time() - t_start, buttons, fonts)

        # Danger flash
        if ctrl.zone_active:
            a  = int(18 + 12 * math.sin(flash_t * 6))
            fs = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            fs.fill((255, 55, 0, a))
            screen.blit(fs, (0, 0))
            fw = pygame.font.SysFont("consolas", 22, bold=True)
            w  = fw.render(
                f"⚠  {ctrl.lock.upper()} TRAIN IN SHARED SECTION — OTHER TRAIN WAITING AT A  ⚠",
                True, C_RED)
            screen.blit(w, (LAYOUT_W//2 - w.get_width()//2, HEIGHT - 36))

        pygame.display.flip()

    pygame.quit()
    print("Simulation ended.")
    sys.exit()


if __name__ == "__main__":
    main()