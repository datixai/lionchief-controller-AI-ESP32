# ══════════════════════════════════════════════════════════════════
#  calibrate_zone.py  —  Visual Zone Calibration Tool
#
#  Saves shared zone coordinates to: shared_zone.json
#  All other files (zone_detector.py, main.py) read from that file.
#
#  HOW TO USE:
#    1. Run:  python calibrate_zone.py
#    2. Camera window opens
#    3. Click and drag to draw the shared zone box
#    4. Press S → saved to shared_zone.json automatically
#    5. Press R to redraw
#    6. Press Q to quit
#    7. Run:  python main.py
# ══════════════════════════════════════════════════════════════════

import cv2
import json
import os
import datetime
import config

# ── Zone file path (same folder as this script) ───────────────────
ZONE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         config.ZONE_FILE)

# ── Mouse state ───────────────────────────────────────────────────
state = {
    "drawing":    False,
    "start_x":    0,
    "start_y":    0,
    "end_x":      0,
    "end_y":      0,
    "final_rect": None,
}


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        state["drawing"] = True
        state["start_x"] = x
        state["start_y"] = y
        state["end_x"]   = x
        state["end_y"]   = y

    elif event == cv2.EVENT_MOUSEMOVE:
        if state["drawing"]:
            state["end_x"] = x
            state["end_y"] = y

    elif event == cv2.EVENT_LBUTTONUP:
        state["drawing"] = False
        state["end_x"]   = x
        state["end_y"]   = y
        x1 = min(state["start_x"], state["end_x"])
        y1 = min(state["start_y"], state["end_y"])
        x2 = max(state["start_x"], state["end_x"])
        y2 = max(state["start_y"], state["end_y"])
        if x2 - x1 > 10 and y2 - y1 > 10:   # ignore tiny accidental clicks
            state["final_rect"] = (x1, y1, x2, y2)


def save_zone(x1, y1, x2, y2, frame_w, frame_h):
    """
    Save zone coordinates to shared_zone.json.
    If file already exists it is OVERWRITTEN with new values.
    """
    data = {
        "x1":          x1,
        "y1":          y1,
        "x2":          x2,
        "y2":          y2,
        "frame_width":  frame_w,
        "frame_height": frame_h,
        "zone_width":   x2 - x1,
        "zone_height":  y2 - y1,
        "saved_at":     datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "note":         "Shared track section — both trains use this zone"
    }
    with open(ZONE_FILE, "w") as f:
        json.dump(data, f, indent=2)
    return data


def load_zone():
    """Load existing zone from shared_zone.json if it exists."""
    if os.path.exists(ZONE_FILE):
        with open(ZONE_FILE, "r") as f:
            return json.load(f)
    return None


def main():
    print("═══════════════════════════════════════════════════")
    print("  LionChief — Zone Calibration Tool")
    print("═══════════════════════════════════════════════════")
    print(f"  Saves to: {ZONE_FILE}")
    print("═══════════════════════════════════════════════════")
    print("  MOUSE  = Click and drag to draw shared zone")
    print("  S      = Save zone to shared_zone.json")
    print("  R      = Reset and redraw")
    print("  Q/ESC  = Quit")
    print("═══════════════════════════════════════════════════\n")

    # Check if zone already exists
    existing = load_zone()
    if existing:
        print(f"⚠️  Existing zone found (saved {existing.get('saved_at', 'unknown')})")
        print(f"   x1={existing['x1']} y1={existing['y1']} "
              f"x2={existing['x2']} y2={existing['y2']}")
        print("   Drawing a new zone and pressing S will overwrite it.\n")

    # Open camera
    cap = cv2.VideoCapture(config.CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)

    if not cap.isOpened():
        print(f"❌ Cannot open camera index {config.CAMERA_INDEX}")
        print("   Try changing CAMERA_INDEX in config.py to 1 or 2")
        return

    ret, test_frame = cap.read()
    if not ret:
        print("❌ Camera opened but cannot read frames")
        cap.release()
        return

    frame_h, frame_w = test_frame.shape[:2]
    print(f"✅ Camera opened: {frame_w}x{frame_h}")
    print("   Draw a rectangle around the shared track section.\n")

    # If existing zone, pre-load into state
    if existing:
        state["start_x"]    = existing["x1"]
        state["start_y"]    = existing["y1"]
        state["end_x"]      = existing["x2"]
        state["end_y"]      = existing["y2"]
        state["final_rect"] = (existing["x1"], existing["y1"],
                               existing["x2"], existing["y2"])

    win = "Calibrate — Draw Shared Zone"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, mouse_callback)

    saved_flash = 0   # frames to show green "Saved!" flash

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        display = frame.copy()

        # Current rectangle
        x1 = min(state["start_x"], state["end_x"])
        y1 = min(state["start_y"], state["end_y"])
        x2 = max(state["start_x"], state["end_x"])
        y2 = max(state["start_y"], state["end_y"])

        # Draw zone if valid
        if x2 - x1 > 5 and y2 - y1 > 5:
            # Shaded fill
            overlay = display.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), -1)
            cv2.addWeighted(overlay, 0.2, display, 0.8, 0, display)
            # Border
            col = (0, 220, 0) if saved_flash == 0 else (0, 255, 150)
            cv2.rectangle(display, (x1, y1), (x2, y2), col, 3)
            # Size label inside zone
            cv2.putText(display,
                        f"SHARED ZONE  {x2-x1}x{y2-y1}px",
                        (x1 + 6, y1 + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            # Corner coordinates
            cv2.putText(display, f"A({x1},{y1})",
                        (x1, max(y1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            cv2.putText(display, f"B({x2},{y2})",
                        (x2 - 80, y2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        # Top instruction bar
        bar_col = (20, 100, 20) if saved_flash > 0 else (40, 40, 40)
        cv2.rectangle(display, (0, 0), (frame_w, 34), bar_col, -1)
        if saved_flash > 0:
            cv2.putText(display, "✅ SAVED to shared_zone.json!",
                        (10, 23), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (100, 255, 100), 2)
            saved_flash -= 1
        else:
            cv2.putText(display,
                        "Draw zone  |  S=Save to file   R=Reset   Q=Quit",
                        (10, 23), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 1)

        # Bottom: show current save file path
        cv2.putText(display,
                    f"File: {ZONE_FILE}",
                    (10, frame_h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        cv2.imshow(win, display)
        key = cv2.waitKey(1) & 0xFF

        # ── S — Save ──────────────────────────────────────────────
        if key in (ord('s'), ord('S')):
            if state["final_rect"]:
                rx1, ry1, rx2, ry2 = state["final_rect"]
                data = save_zone(rx1, ry1, rx2, ry2, frame_w, frame_h)
                saved_flash = 90  # show green flash for ~3 seconds

                print(f"\n✅ Zone saved to: {ZONE_FILE}")
                print("─────────────────────────────────────────")
                print(f"  x1 = {rx1}")
                print(f"  y1 = {ry1}")
                print(f"  x2 = {rx2}")
                print(f"  y2 = {ry2}")
                print(f"  Zone size  : {rx2-rx1} x {ry2-ry1} pixels")
                print(f"  Frame size : {frame_w} x {frame_h} pixels")
                print(f"  Saved at   : {data['saved_at']}")
                print("─────────────────────────────────────────")
                print("  ✅ zone_detector.py will read this automatically")
                print("  ✅ main.py will read this automatically")
                print("\n  Run:  python main.py\n")

                # Save screenshot
                cv2.imwrite("calibrated_zone.jpg", display)
                print("  Screenshot: calibrated_zone.jpg\n")
            else:
                print("❌ No zone drawn yet — click and drag on the image")

        # ── R — Reset ─────────────────────────────────────────────
        elif key in (ord('r'), ord('R')):
            state["start_x"] = state["start_y"] = 0
            state["end_x"]   = state["end_y"]   = 0
            state["final_rect"] = None
            print("Zone cleared — draw a new one")

        # ── Q — Quit ──────────────────────────────────────────────
        elif key in (ord('q'), ord('Q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()

    # Final status
    final = load_zone()
    if final:
        print(f"✅ Active zone: x1={final['x1']} y1={final['y1']} "
              f"x2={final['x2']} y2={final['y2']}")
    print("Calibration tool closed.\n")


if __name__ == "__main__":
    main()
