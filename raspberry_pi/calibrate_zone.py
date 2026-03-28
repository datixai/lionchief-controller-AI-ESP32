"""
calibrate_zone.py — Interactive Zone Calibration Tool
=======================================================
Run this ONCE to set the shared track zone coordinates.
Click and drag on the camera window to draw the zone.
Press S to save coordinates to config.py automatically.
Press R to reset and redraw.
Press Q to quit without saving.

Usage:
    python calibrate_zone.py
    python calibrate_zone.py --camera 1   # if camera index is not 0
"""

import cv2
import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config

# ── Mouse state ───────────────────────────────────────────────────────────────
drawing   = False
ix, iy    = -1, -1
fx, fy    = -1, -1
rect_done = False
frame_ref = None


def mouse_callback(event, x, y, flags, param):
    global drawing, ix, iy, fx, fy, rect_done, frame_ref

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing   = True
        rect_done = False
        ix, iy    = x, y
        fx, fy    = x, y

    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        fx, fy = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing   = False
        rect_done = True
        fx, fy    = x, y
        print(f"\n📐 Zone drawn: ({min(ix,fx)}, {min(iy,fy)}) → ({max(ix,fx)}, {max(iy,fy)})")
        print(f"   SHARED_ZONE = ({min(ix,fx)}, {min(iy,fy)}, {max(ix,fx)}, {max(iy,fy)})")


def save_zone_to_config(x1, y1, x2, y2):
    """Patch the SHARED_ZONE line in config.py."""
    config_path = os.path.join(os.path.dirname(__file__), "config.py")
    with open(config_path, "r") as f:
        lines = f.readlines()

    new_lines = []
    for line in lines:
        if line.strip().startswith("SHARED_ZONE"):
            new_lines.append(f"SHARED_ZONE = ({x1}, {y1}, {x2}, {y2})   "
                             f"# calibrated\n")
        else:
            new_lines.append(line)

    with open(config_path, "w") as f:
        f.writelines(new_lines)

    print(f"✅ Saved SHARED_ZONE = ({x1}, {y1}, {x2}, {y2}) to config.py")


def main():
    global ix, iy, fx, fy, rect_done

    parser = argparse.ArgumentParser(description="Zone calibration tool")
    parser.add_argument("--camera", type=int, default=config.CAMERA_INDEX)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)

    if not cap.isOpened():
        print("❌ Cannot open camera. Check --camera index.")
        sys.exit(1)

    cv2.namedWindow("Zone Calibration")
    cv2.setMouseCallback("Zone Calibration", mouse_callback)

    print("\n" + "="*55)
    print("  ZONE CALIBRATION TOOL")
    print("="*55)
    print("  1. Watch the live camera feed")
    print("  2. Click and drag to draw the SHARED TRACK section")
    print("  3. Press S  to SAVE zone to config.py")
    print("  4. Press R  to RESET and redraw")
    print("  5. Press Q  to quit without saving")
    print("="*55 + "\n")

    # Show current zone from config
    cx1, cy1, cx2, cy2 = config.SHARED_ZONE
    ix, iy = cx1, cy1
    fx, fy = cx2, cy2
    rect_done = True
    print(f"Current zone in config.py: {config.SHARED_ZONE}")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Draw current/in-progress rectangle
        if ix >= 0 and fx >= 0:
            x1, y1 = min(ix, fx), min(iy, fy)
            x2, y2 = max(ix, fx), max(iy, fy)
            color  = (0, 165, 255) if rect_done else (0, 255, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, "SHARED ZONE", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(frame, f"({x1},{y1}) → ({x2},{y2})",
                        (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Instructions overlay
        cv2.putText(frame, "Drag: draw zone | S: save | R: reset | Q: quit",
                    (8, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Zone Calibration", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("s") and rect_done:
            x1, y1 = min(ix, fx), min(iy, fy)
            x2, y2 = max(ix, fx), max(iy, fy)
            save_zone_to_config(x1, y1, x2, y2)
            print("Run main.py to start monitoring with the new zone.")
            break

        elif key == ord("r"):
            ix = iy = fx = fy = -1
            rect_done = False
            print("Zone reset — draw a new one.")

        elif key in (ord("q"), 27):
            print("Quit — zone NOT saved.")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
