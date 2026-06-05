# ══════════════════════════════════════════════════════════════════
#  calibrate_zone.py  —  Precision Polygon Zone Calibration
#  Harry Locomotive Project  |  Datix AI  |  May 2026
#
#  Draws THREE polygon zones matching ESP32 IR sensor positions.
#  Polygons follow curved track precisely — unlimited corner points.
#
#  ZONES:
#    Zone A — Inner loop approach (before shared section, inner track)
#    Zone B — Outer loop approach (before shared section, outer track)
#    Zone C — Shared section / exit area
#
#  HOW TO USE:
#    1.  python calibrate_zone.py
#    2.  Press TAB to switch between Zone A / B / C
#    3.  LEFT CLICK to place polygon corner points
#        (click around both sides of the track — as many points as needed)
#    4.  RIGHT CLICK to undo / remove last placed point
#    5.  LEFT CLICK near an existing point (green dot) to SELECT and drag it
#    6.  Press Z to toggle 4x ZOOM window for precise placement
#    7.  Press ENTER or F to FINISH / CLOSE the current polygon
#    8.  Press S to SAVE current zone
#    9.  Press A to SAVE ALL zones and quit
#   10.  Press R to RESET / redraw current zone from scratch
#   11.  Press Q / ESC to quit
#
#  TIPS FOR CURVED TRACK:
#    • Click many points closely spaced to follow the curve
#    • Click on BOTH SIDES of the track rail to enclose it
#    • The filled polygon shows exactly what the detector will watch
#    • Use Z zoom to place points precisely on thin rails
# ══════════════════════════════════════════════════════════════════

import cv2
import json
import os
import numpy as np
import datetime
import config

ZONES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          config.ZONES_FILE)

# Zone definitions
ZONE_DEFS = {
    "A": {
        "label":       "Zone A — Inner Loop Approach",
        "desc":        "Draw around INNER loop track, just BEFORE shared section entry",
        "color":       (0, 165, 255),   # orange
    },
    "B": {
        "label":       "Zone B — Outer Loop Approach",
        "desc":        "Draw around OUTER loop track, just BEFORE shared section entry",
        "color":       (255, 80, 0),    # blue
    },
    "C": {
        "label":       "Zone C — Shared Section / Exit",
        "desc":        "Draw over ENTIRE shared track section including exit",
        "color":       (0, 200, 0),     # green
    },
}
ZONE_ORDER = ["A", "B", "C"]

# ── State ─────────────────────────────────────────────────────────
saved_zones    = {}          # {zone_key: [[x,y], [x,y], ...]}
current_zone   = "A"
current_points = []          # points being drawn for current zone
polygon_closed = False       # True after ENTER/F pressed
selected_idx   = None        # index of point being dragged
drag_active    = False
show_zoom      = False
cursor_pos     = (0, 0)
frame_w        = 1280
frame_h        = 720

# Point selection radius in pixels
SELECT_RADIUS = 12

# ── Load existing zones ───────────────────────────────────────────

def load_existing():
    global saved_zones, current_points, polygon_closed
    if not os.path.exists(ZONES_FILE):
        print("\n  No existing zones.json — draw fresh zones.")
        return
    try:
        with open(ZONES_FILE, "r") as f:
            data = json.load(f)
        for z in ZONE_ORDER:
            if z in data and data[z].get("points"):
                pts = data[z]["points"]
                if len(pts) >= 3:
                    saved_zones[z] = pts
        if saved_zones:
            # Load active zone into drawing buffer
            if current_zone in saved_zones:
                current_points = [list(p) for p in saved_zones[current_zone]]
                polygon_closed = True
            print(f"  Loaded zones: {list(saved_zones.keys())} from {ZONES_FILE}")
    except Exception as e:
        print(f"  Warning: could not load zones.json — {e}")


def save_zone(zone_key, points, closed):
    if len(points) < 3 or not closed:
        print(f"  ❌ Zone {zone_key} needs at least 3 points and must be closed (press ENTER)")
        return False

    if os.path.exists(ZONES_FILE):
        try:
            with open(ZONES_FILE, "r") as f:
                data = json.load(f)
        except Exception:
            data = {}
    else:
        data = {}

    area = cv2.contourArea(np.array(points, dtype=np.float32))
    data[zone_key] = {
        "points":      points,
        "label":       ZONE_DEFS[zone_key]["label"],
        "desc":        ZONE_DEFS[zone_key]["desc"],
        "point_count": len(points),
        "area_px":     int(area),
        "frame_width": frame_w,
        "frame_height":frame_h,
        "saved_at":    datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }

    with open(ZONES_FILE, "w") as f:
        json.dump(data, f, indent=2)

    saved_zones[zone_key] = [list(p) for p in points]
    print(f"  ✅ Zone {zone_key} saved — {len(points)} points, area: {int(area)}px²")
    return True


def save_all(current_pts, closed):
    # Save current zone first if it has points
    if len(current_pts) >= 3 and closed:
        save_zone(current_zone, current_pts, closed)
    saved = list(saved_zones.keys())
    missing = [z for z in ZONE_ORDER if z not in saved_zones]
    print(f"\n  Saved zones: {saved}")
    if missing:
        print(f"  ⚠️  Missing zones: {missing}")
    else:
        print("  ✅ All 3 zones saved!")
    return saved


# ── Mouse callback ────────────────────────────────────────────────

def get_nearest_point_idx(pts, x, y, radius=SELECT_RADIUS):
    """Return index of nearest point within radius, or None."""
    best_idx  = None
    best_dist = radius * radius
    for i, (px, py) in enumerate(pts):
        d = (px - x)**2 + (py - y)**2
        if d < best_dist:
            best_dist = d
            best_idx  = i
    return best_idx


def mouse_callback(event, x, y, flags, param):
    global current_points, polygon_closed, selected_idx, drag_active, cursor_pos

    cursor_pos = (x, y)

    if event == cv2.EVENT_LBUTTONDOWN:
        if polygon_closed:
            # Try to select existing point for dragging
            idx = get_nearest_point_idx(current_points, x, y)
            if idx is not None:
                selected_idx = idx
                drag_active  = True
            # Click far from any point: do nothing (polygon already closed)
        else:
            # Add a new corner point to the polygon
            current_points.append([x, y])

    elif event == cv2.EVENT_MOUSEMOVE:
        if drag_active and selected_idx is not None:
            current_points[selected_idx] = [x, y]

    elif event == cv2.EVENT_LBUTTONUP:
        drag_active  = False
        selected_idx = None

    elif event == cv2.EVENT_RBUTTONDOWN:
        if polygon_closed:
            # Right click: deselect and allow editing again
            polygon_closed = False
            print("  Polygon opened for editing — click to move points, ENTER to close again")
        else:
            # Undo last added point
            if current_points:
                current_points.pop()
                print(f"  Removed last point — {len(current_points)} remaining")


# ── Draw helpers ──────────────────────────────────────────────────

def draw_zone_on_frame(frame, zone_key, points, closed, is_active):
    """Draw a polygon zone with fill, outline and point handles."""
    if not points:
        return frame

    pts_arr = np.array(points, dtype=np.int32)
    color   = ZONE_DEFS[zone_key]["color"]
    is_cur  = is_active

    # Semi-transparent fill if closed
    if closed and len(points) >= 3:
        overlay = frame.copy()
        cv2.fillPoly(overlay, [pts_arr], color)
        alpha = 0.35 if is_cur else 0.15
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    # Draw edges
    if len(points) >= 2:
        for i in range(len(points) - 1):
            cv2.line(frame,
                     tuple(points[i]),
                     tuple(points[i + 1]),
                     color, 2 if is_cur else 1)
        if closed and len(points) >= 3:
            cv2.line(frame,
                     tuple(points[-1]),
                     tuple(points[0]),
                     color, 2 if is_cur else 1)

    # Draw point handles
    for i, (px, py) in enumerate(points):
        # Outer circle
        cv2.circle(frame, (px, py), 6 if is_cur else 4, color, -1)
        # White inner dot
        cv2.circle(frame, (px, py), 3 if is_cur else 2, (255, 255, 255), -1)

    # Zone label
    if points:
        cx = int(np.mean([p[0] for p in points]))
        cy = int(np.mean([p[1] for p in points]))
        label = f"Zone {zone_key}"
        cv2.putText(frame, label, (cx - 25, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65 if is_cur else 0.45,
                    color, 2 if is_cur else 1)
        if is_cur:
            info = f"{len(points)}pts {'CLOSED' if closed else 'drawing...'}"
            cv2.putText(frame, info, (cx - 30, cy + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    return frame


def draw_zoom_panel(frame, cursor_x, cursor_y, zoom=4, size=160):
    """Draw a magnified view of the cursor area for precise placement."""
    h, w = frame.shape[:2]
    half = size // zoom // 2

    x1 = max(0, cursor_x - half)
    y1 = max(0, cursor_y - half)
    x2 = min(w, cursor_x + half)
    y2 = min(h, cursor_y + half)

    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return frame

    zoomed = cv2.resize(crop, None, fx=zoom, fy=zoom,
                        interpolation=cv2.INTER_NEAREST)
    zh, zw = zoomed.shape[:2]
    ph = min(zh, 200)
    pw = min(zw, 200)
    zoomed = zoomed[:ph, :pw]

    # Crosshair on zoom
    cv2.line(zoomed, (pw // 2, 0), (pw // 2, ph), (0, 255, 255), 1)
    cv2.line(zoomed, (0, ph // 2), (pw, ph // 2), (0, 255, 255), 1)

    # Paste to bottom-right
    fy_start = h - ph - 4
    fx_start = w - pw - 4
    frame[fy_start:fy_start + ph, fx_start:fx_start + pw] = zoomed
    cv2.rectangle(frame,
                  (fx_start - 1, fy_start - 1),
                  (fx_start + pw + 1, fy_start + ph + 1),
                  (0, 255, 255), 2)
    cv2.putText(frame, f"ZOOM {zoom}x",
                (fx_start, fy_start - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    return frame


# ── Main ──────────────────────────────────────────────────────────

def main():
    global current_zone, current_points, polygon_closed
    global show_zoom, frame_w, frame_h

    print("╔════════════════════════════════════════════════════════╗")
    print("║   LionChief — Precision Polygon Zone Calibration      ║")
    print("╠════════════════════════════════════════════════════════╣")
    print("║  LEFT CLICK   = Add polygon corner point              ║")
    print("║  RIGHT CLICK  = Undo last point / reopen closed zone  ║")
    print("║  DRAG point   = Move an existing corner (when closed) ║")
    print("║  ENTER / F    = Close / finish polygon                ║")
    print("║  TAB          = Switch zone  A → B → C               ║")
    print("║  S            = Save current zone                     ║")
    print("║  A            = Save ALL zones and quit               ║")
    print("║  R            = Reset / redraw current zone           ║")
    print("║  Z            = Toggle zoom window                    ║")
    print("║  Q / ESC      = Quit                                  ║")
    print("╠════════════════════════════════════════════════════════╣")
    print("║  TIP: Click many points to follow curved track        ║")
    print("║  TIP: Enclose BOTH sides of the rail for accuracy     ║")
    print("╚════════════════════════════════════════════════════════╝\n")

    load_existing()

    # Open camera
    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print(f"❌ Cannot open camera {config.CAMERA_INDEX}")
        return

    ret, test_frame = cap.read()
    if not ret:
        print("❌ Cannot read from camera")
        cap.release()
        return

    frame_h, frame_w = test_frame.shape[:2]
    print(f"  Camera: {frame_w}×{frame_h}  |  Active zone: Zone {current_zone}\n")

    WIN = "LionChief — Zone Calibration"
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, mouse_callback)

    save_flash = 0   # frames to show save confirmation

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        display = frame.copy()

        # Draw all SAVED zones (non-active) behind active zone
        for z_key in ZONE_ORDER:
            if z_key == current_zone:
                continue
            if z_key in saved_zones and len(saved_zones[z_key]) >= 3:
                draw_zone_on_frame(display, z_key,
                                   saved_zones[z_key], True, False)

        # Draw active zone being edited
        if current_points or polygon_closed:
            draw_zone_on_frame(display, current_zone,
                               current_points, polygon_closed, True)

        # Cursor crosshair (when not dragging)
        cx, cy = cursor_pos
        cv2.line(display, (cx - 15, cy), (cx + 15, cy), (200, 200, 200), 1)
        cv2.line(display, (cx, cy - 15), (cx, cy + 15), (200, 200, 200), 1)
        cv2.putText(display, f"({cx},{cy})",
                    (cx + 8, cy - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        # Zoom panel
        if show_zoom:
            draw_zoom_panel(display, cx, cy)

        # ── Top status bar ─────────────────────────────────────
        col   = ZONE_DEFS[current_zone]["color"]
        bar_c = (20, 100, 20) if save_flash > 0 else (25, 25, 25)
        cv2.rectangle(display, (0, 0), (frame_w, 52), bar_c, -1)

        if save_flash > 0:
            cv2.putText(display,
                        f"  ✅ Zone {current_zone} SAVED  —  {len(current_points)} points",
                        (10, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (100, 255, 100), 2)
            save_flash -= 1
        else:
            cv2.putText(display,
                        f"  Active: Zone {current_zone}  |  "
                        f"TAB=Switch  ENTER=Close  S=Save  A=SaveAll  R=Reset  Z=Zoom  Q=Quit",
                        (4, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            status = "CLOSED ✅" if polygon_closed else f"DRAWING — {len(current_points)} pts"
            cv2.putText(display,
                        f"  {ZONE_DEFS[current_zone]['desc']}  |  {status}",
                        (4, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

        # Zone status indicators (top-right)
        for i, z in enumerate(ZONE_ORDER):
            c    = ZONE_DEFS[z]["color"]
            tick = "✅" if z in saved_zones else "○"
            mark = "►" if z == current_zone else " "
            cv2.putText(display,
                        f"{mark} Zone {z}: {tick}",
                        (frame_w - 150, 16 + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43,
                        c if z == current_zone else (130, 130, 130), 1)

        # Bottom hint
        cv2.putText(display,
                    "Left=AddPoint  Right=Undo/Edit  Drag=MovePoint  ENTER=ClosePolygon",
                    (6, frame_h - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.37, (120, 120, 120), 1)

        cv2.imshow(WIN, display)

        key = cv2.waitKey(20) & 0xFF

        # ── Key handling ───────────────────────────────────────
        if key in (ord('q'), ord('Q'), 27):
            break

        elif key == 9:   # TAB — switch zone
            # Auto-save current to buffer before switching
            if len(current_points) >= 3 and polygon_closed:
                saved_zones[current_zone] = [list(p) for p in current_points]

            idx = ZONE_ORDER.index(current_zone)
            current_zone = ZONE_ORDER[(idx + 1) % len(ZONE_ORDER)]

            # Load saved data for new zone
            if current_zone in saved_zones:
                current_points = [list(p) for p in saved_zones[current_zone]]
                polygon_closed = True
            else:
                current_points = []
                polygon_closed = False

            print(f"  Switched to Zone {current_zone}: {ZONE_DEFS[current_zone]['desc']}")

        elif key in (13, ord('f'), ord('F')):   # ENTER or F — close polygon
            if len(current_points) >= 3:
                polygon_closed = True
                print(f"  Zone {current_zone} polygon CLOSED — {len(current_points)} points")
                print("  Press S to save, or drag points to fine-tune")
            else:
                print(f"  Need at least 3 points (have {len(current_points)})")

        elif key in (ord('s'), ord('S')):
            if save_zone(current_zone, current_points, polygon_closed):
                save_flash = 60

        elif key in (ord('a'), ord('A')):
            save_all(current_points, polygon_closed)
            break

        elif key in (ord('r'), ord('R')):
            current_points = []
            polygon_closed = False
            if current_zone in saved_zones:
                del saved_zones[current_zone]
            print(f"  Zone {current_zone} RESET — draw new polygon")

        elif key in (ord('z'), ord('Z')):
            show_zoom = not show_zoom
            print(f"  Zoom: {'ON' if show_zoom else 'OFF'}")

    cap.release()
    cv2.destroyAllWindows()
    print("\n  Calibration complete.")
    saved = list(saved_zones.keys())
    missing = [z for z in ZONE_ORDER if z not in saved_zones]
    if saved:
        print(f"  Saved zones: {saved}")
    if missing:
        print(f"  ⚠️  Missing: {missing} — run again to draw them")


if __name__ == "__main__":
    main()