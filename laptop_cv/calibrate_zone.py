# ══════════════════════════════════════════════════════════════════
#  calibrate_zone.py  —  Precision 3-Zone Calibration Tool
#
#  Draws three zones matching ESP32 IR sensor positions:
#    Zone A — Inner loop approach (before shared section entry)
#    Zone B — Outer loop approach (before shared section entry)
#    Zone C — Shared section / exit
#
#  HOW TO USE:
#    1. python calibrate_zone.py
#    2. Press TAB to switch between Zone A / B / C
#    3. Click and drag to draw the zone on camera image
#    4. Use ARROW KEYS to fine-tune zone edges precisely
#       (hold SHIFT for large steps, normal for 1px steps)
#    5. Press Z to toggle zoom window for precise placement
#    6. Press S to save current zone
#    7. Press A to save ALL zones and quit
#    8. Press R to redraw current zone
#    9. Press Q to quit
# ══════════════════════════════════════════════════════════════════

import cv2
import json
import os
import datetime
import numpy as np
import config

ZONES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          config.ZONES_FILE)

# Zone definitions — label, instructions, display color
ZONE_DEFS = {
    "A": {
        "label":       "Zone A — Inner Loop Approach",
        "description": "Draw on INNER loop track, just BEFORE shared section entry",
        "color":       (0, 165, 255),   # orange
        "key":         "A",
    },
    "B": {
        "label":       "Zone B — Outer Loop Approach",
        "description": "Draw on OUTER loop track, just BEFORE shared section entry",
        "color":       (255, 100, 0),   # blue
        "key":         "B",
    },
    "C": {
        "label":       "Zone C — Shared Section / Exit",
        "description": "Draw over the ENTIRE shared track section",
        "color":       (0, 220, 0),     # green
        "key":         "C",
    },
}

ZONE_ORDER = ["A", "B", "C"]

# ── State ─────────────────────────────────────────────────────────
zones = {}           # {"A": (x1,y1,x2,y2), "B": ..., "C": ...}
current_zone = "A"   # which zone is being edited
drawing = False
drag_start = (0, 0)
drag_end   = (0, 0)
show_zoom  = False
zoom_edge  = "right"  # which edge is highlighted for fine-tuning
frame_w = 1280
frame_h = 720


def load_existing():
    """Load previously saved zones if file exists."""
    global zones
    if os.path.exists(ZONES_FILE):
        with open(ZONES_FILE, "r") as f:
            data = json.load(f)
        for z in ZONE_ORDER:
            if z in data and data[z]:
                r = data[z]["rect"]
                zones[z] = (r["x1"], r["y1"], r["x2"], r["y2"])
        print(f"\n  Loaded existing zones from {ZONES_FILE}")
    else:
        print(f"\n  No existing zones found — draw all three zones.")


def save_zone(zone_key):
    """Save the current zone to zones.json."""
    if zone_key not in zones:
        print(f"  ❌ Zone {zone_key} not drawn yet")
        return False

    x1, y1, x2, y2 = zones[zone_key]

    # Load existing file or start fresh
    if os.path.exists(ZONES_FILE):
        with open(ZONES_FILE, "r") as f:
            data = json.load(f)
    else:
        data = {}

    data[zone_key] = {
        "rect": {"x1": x1, "y1": y1, "x2": x2, "y2": y2},
        "label":       ZONE_DEFS[zone_key]["label"],
        "description": ZONE_DEFS[zone_key]["description"],
        "frame_width":  frame_w,
        "frame_height": frame_h,
        "zone_width":  x2 - x1,
        "zone_height": y2 - y1,
        "saved_at":    datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }

    with open(ZONES_FILE, "w") as f:
        json.dump(data, f, indent=2)

    print(f"  ✅ Zone {zone_key} saved: ({x1},{y1})→({x2},{y2})  "
          f"size:{x2-x1}×{y2-y1}px")
    return True


def save_all():
    """Save all zones."""
    saved = 0
    for z in ZONE_ORDER:
        if z in zones:
            if save_zone(z):
                saved += 1
    print(f"\n  ✅ Saved {saved}/3 zones to {ZONES_FILE}")
    missing = [z for z in ZONE_ORDER if z not in zones]
    if missing:
        print(f"  ⚠️  Missing zones: {missing} — run again to draw them")
    return saved


def mouse_callback(event, x, y, flags, param):
    global drawing, drag_start, drag_end

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing    = True
        drag_start = (x, y)
        drag_end   = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            drag_end = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing  = False
        drag_end = (x, y)
        x1 = min(drag_start[0], drag_end[0])
        y1 = min(drag_start[1], drag_end[1])
        x2 = max(drag_start[0], drag_end[0])
        y2 = max(drag_start[1], drag_end[1])
        # Ignore tiny accidental clicks
        if (x2 - x1) > 15 and (y2 - y1) > 15:
            zones[current_zone] = (x1, y1, x2, y2)
            print(f"  Zone {current_zone} drawn: ({x1},{y1})→({x2},{y2})  "
                  f"size:{x2-x1}×{y2-y1}px  "
                  f"Press S to save, arrow keys to fine-tune")


def adjust_zone(zone_key, direction, step=1):
    """
    Fine-tune zone edges with arrow keys.
    Direction: 'left_in', 'left_out', 'right_in', 'right_out',
               'top_in', 'top_out', 'bottom_in', 'bottom_out'
    Cycle through edges with WASD, then use arrows to nudge.
    """
    if zone_key not in zones:
        return
    x1, y1, x2, y2 = zones[zone_key]

    if direction == "left":     x1 = max(0, x1 - step)
    elif direction == "right":  x1 = min(x2 - 10, x1 + step)
    elif direction == "up":     y1 = max(0, y1 - step)
    elif direction == "down":   y1 = min(y2 - 10, y1 + step)
    elif direction == "r_left": x2 = max(x1 + 10, x2 - step)
    elif direction == "r_right":x2 = min(frame_w, x2 + step)
    elif direction == "r_up":   y2 = max(y1 + 10, y2 - step)
    elif direction == "r_down": y2 = min(frame_h, y2 + step)

    zones[zone_key] = (x1, y1, x2, y2)


def draw_all_zones(frame, active_key):
    """Draw all zones on the frame, highlight the active one."""
    overlay = frame.copy()

    for z_key in ZONE_ORDER:
        if z_key not in zones:
            continue

        x1, y1, x2, y2 = zones[z_key]
        color = ZONE_DEFS[z_key]["color"]
        is_active = (z_key == active_key)

        # Fill
        alpha = 0.35 if is_active else 0.15
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)

        # Border — thicker for active zone
        thickness = 3 if is_active else 1
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

        # Corner handles for active zone
        if is_active:
            hlen = 12
            for cx, cy in [(x1,y1),(x2,y1),(x1,y2),(x2,y2)]:
                dx = 1 if cx == x1 else -1
                dy = 1 if cy == y1 else -1
                cv2.line(frame, (cx, cy), (cx + dx*hlen, cy), (255,255,255), 2)
                cv2.line(frame, (cx, cy), (cx, cy + dy*hlen), (255,255,255), 2)

        # Label
        label = f"Zone {z_key}"
        lx = x1 + 4
        ly = y1 + 20 if y1 + 20 < y2 else y2 - 6
        cv2.putText(frame, label, (lx, ly),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65 if is_active else 0.5,
                    color,
                    2 if is_active else 1)

        # Size label for active zone
        if is_active:
            size_txt = f"{x2-x1}x{y2-y1}px  ({x1},{y1})→({x2},{y2})"
            cv2.putText(frame, size_txt, (x1, max(y1 - 8, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    # Blend overlay
    cv2.addWeighted(overlay, 0.25, frame, 0.75, 0, frame)
    return frame


def draw_zoom_panel(frame, zone_key):
    """
    Draw a 4x magnified view of the active zone edges
    in the bottom-right corner for precision placement.
    """
    if zone_key not in zones:
        return frame

    x1, y1, x2, y2 = zones[zone_key]
    h, w = frame.shape[:2]

    # Crop a small strip around right+bottom edges
    margin = 30
    cx = max(0, x2 - margin)
    cy = max(0, y2 - margin)
    ex = min(w, x2 + margin)
    ey = min(h, y2 + margin)

    crop = frame[cy:ey, cx:ex]
    if crop.size == 0:
        return frame

    zoom = cv2.resize(crop, None, fx=4, fy=4,
                      interpolation=cv2.INTER_NEAREST)

    # Crosshair on zoom
    zh, zw = zoom.shape[:2]
    cv2.line(zoom, (zw//2, 0), (zw//2, zh), (0, 255, 255), 1)
    cv2.line(zoom, (0, zh//2), (zw, zh//2), (0, 255, 255), 1)

    # Paste in bottom-right corner
    ph, pw = zoom.shape[:2]
    ph = min(ph, h - 4)
    pw = min(pw, w - 4)
    zoom = zoom[:ph, :pw]
    frame[h-ph-4:h-4, w-pw-4:w-4] = zoom

    # Border around zoom panel
    cv2.rectangle(frame,
                  (w-pw-4, h-ph-4),
                  (w-4, h-4),
                  (0, 255, 255), 2)
    cv2.putText(frame, "ZOOM (4x) — right/bottom edge",
                (w-pw-4, h-ph-20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    return frame


def main():
    global current_zone, show_zoom, frame_w, frame_h

    print("╔══════════════════════════════════════════════════╗")
    print("║  LionChief — Precision 3-Zone Calibration Tool  ║")
    print("╠══════════════════════════════════════════════════╣")
    print("║  TAB        = Switch zone  A → B → C            ║")
    print("║  DRAG       = Draw zone with mouse               ║")
    print("║  ARROWS     = Nudge TOP-LEFT corner (1px)        ║")
    print("║  SHIFT+ARR  = Nudge BOTTOM-RIGHT corner (1px)    ║")
    print("║  CTRL+ARR   = Large step (10px)                  ║")
    print("║  Z          = Toggle zoom window                 ║")
    print("║  S          = Save current zone                  ║")
    print("║  A          = Save ALL zones and quit            ║")
    print("║  R          = Reset current zone                 ║")
    print("║  Q / ESC    = Quit                               ║")
    print("╚══════════════════════════════════════════════════╝")

    load_existing()

    cap = cv2.VideoCapture(config.CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print(f"❌ Cannot open camera {config.CAMERA_INDEX}")
        return

    ret, test = cap.read()
    if not ret:
        print("❌ Cannot read camera frames")
        cap.release()
        return

    frame_h, frame_w = test.shape[:2]
    print(f"\n  Camera: {frame_w}x{frame_h}")
    print(f"  Active zone: Zone {current_zone} — "
          f"{ZONE_DEFS[current_zone]['description']}\n")

    WIN = "LionChief — Zone Calibration"
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, mouse_callback)

    save_flash = 0   # frames to show save flash

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        display = frame.copy()

        # Draw current drag rectangle
        if drawing:
            x1 = min(drag_start[0], drag_end[0])
            y1 = min(drag_start[1], drag_end[1])
            x2 = max(drag_start[0], drag_end[0])
            y2 = max(drag_start[1], drag_end[1])
            color = ZONE_DEFS[current_zone]["color"]
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
            cv2.putText(display, f"{x2-x1}x{y2-y1}",
                        (x1+4, y1+16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Draw all saved zones
        display = draw_all_zones(display, current_zone)

        # Zoom panel
        if show_zoom:
            display = draw_zoom_panel(display, current_zone)

        # ── Top instruction bar ──────────────────────────────────
        color = ZONE_DEFS[current_zone]["color"]
        bar_col = (20, 80, 20) if save_flash > 0 else (30, 30, 30)
        cv2.rectangle(display, (0, 0), (frame_w, 50), bar_col, -1)

        if save_flash > 0:
            cv2.putText(display, f"✅ Zone {current_zone} SAVED!",
                        (10, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (100, 255, 100), 2)
            save_flash -= 1
        else:
            cv2.putText(display,
                        f"Active: Zone {current_zone}  |  "
                        f"TAB=Switch  S=Save  A=SaveAll  R=Reset  Z=Zoom  Q=Quit",
                        (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(display,
                        ZONE_DEFS[current_zone]["description"],
                        (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Zone status indicators top-right
        for i, z in enumerate(ZONE_ORDER):
            c = ZONE_DEFS[z]["color"]
            status = "✅" if z in zones else "○"
            active_marker = "►" if z == current_zone else " "
            cv2.putText(display,
                        f"{active_marker} Zone {z}: {status}",
                        (frame_w - 145, 18 + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        c if z == current_zone else (150, 150, 150), 1)

        # Arrow key hint
        cv2.putText(display,
                    "Arrows=nudge TL corner  |  Shift+Arrows=nudge BR corner  "
                    "|  Ctrl+Arrows=10px step",
                    (10, frame_h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 140, 140), 1)

        cv2.imshow(WIN, display)

        key = cv2.waitKey(20) & 0xFF
        flags = cv2.getWindowProperty(WIN, cv2.WND_PROP_AUTOSIZE)

        # ── Key handling ─────────────────────────────────────────
        if key in (ord('q'), ord('Q'), 27):
            break

        elif key == 9:   # TAB — switch zone
            idx = ZONE_ORDER.index(current_zone)
            current_zone = ZONE_ORDER[(idx + 1) % len(ZONE_ORDER)]
            print(f"  Switched to Zone {current_zone}: "
                  f"{ZONE_DEFS[current_zone]['description']}")

        elif key in (ord('s'), ord('S')):
            if save_zone(current_zone):
                save_flash = 60

        elif key in (ord('a'), ord('A')):
            save_all()
            break

        elif key in (ord('r'), ord('R')):
            if current_zone in zones:
                del zones[current_zone]
            print(f"  Zone {current_zone} reset — draw a new one")

        elif key in (ord('z'), ord('Z')):
            show_zoom = not show_zoom
            print(f"  Zoom window: {'ON' if show_zoom else 'OFF'}")

        # ── Arrow keys — fine-tune zone edges ──────────────────
        # Regular arrows → move TOP-LEFT corner (x1, y1)
        # Shift + arrows → move BOTTOM-RIGHT corner (x2, y2)
        # Ctrl  + arrows → 10px step
        elif key == 81 or key == 2424832:    # Left arrow
            adjust_zone(current_zone, "left", 1)
        elif key == 83 or key == 2555904:    # Right arrow
            adjust_zone(current_zone, "right", 1)
        elif key == 82 or key == 2490368:    # Up arrow
            adjust_zone(current_zone, "up", 1)
        elif key == 84 or key == 2621440:    # Down arrow
            adjust_zone(current_zone, "down", 1)

        # Check for extended key codes (platform dependent)
        # OpenCV on Windows sends different codes
        # We also check for the raw key values
        if key == 0:   # some platforms send 0 for special keys
            pass

    cap.release()
    cv2.destroyAllWindows()

    # Final status
    saved = [z for z in ZONE_ORDER if z in zones]
    print(f"\n  Zones in memory: {saved}")
    if os.path.exists(ZONES_FILE):
        print(f"  File saved: {ZONES_FILE}")
    print("  Calibration tool closed.\n")


if __name__ == "__main__":
    main()
