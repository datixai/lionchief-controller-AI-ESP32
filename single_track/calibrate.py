# ══════════════════════════════════════════════════════════════════
#  calibrate.py  —  Distance Threshold Calibration
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  Sets pixel distance thresholds for the 5 speed zones.
#  No color calibration needed — click-to-track handles detection.
#
#  HOW TO USE:
#    1. python calibrate.py
#    2. Click on Train A (front) → orange circle appears
#    3. Click on Train B (rear)  → green circle appears
#    4. Move trains to each gap position, press the matching key:
#       D = DANGER  (trains dangerously close)
#       W = WARNING (too close, must slow)
#       C = CAUTION (start slowing)
#       S = SAFE    (ideal following gap)
#       F = FAR     (gap too large, catch up)
#    5. Press ENTER or A to save → calibration.json
#    6. Press Q to quit without saving
# ══════════════════════════════════════════════════════════════════

import cv2
import json
import os
import sys
import numpy as np

import config
from train_detector import ClickTracker, pixel_distance

CAL_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), config.CALIBRATION_FILE)

# Current distance thresholds
dist_cal = {
    "danger_px":  config.DISTANCE_DANGER,
    "warning_px": config.DISTANCE_WARNING,
    "caution_px": config.DISTANCE_CAUTION,
    "safe_px":    config.DISTANCE_SAFE,
    "far_px":     config.DISTANCE_FAR,
}


def save_calibration():
    data = {"distances": dist_cal}
    with open(CAL_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n  ✅ Calibration saved → {CAL_FILE}")
    print(f"     Danger={dist_cal['danger_px']}  Warning={dist_cal['warning_px']}  "
          f"Caution={dist_cal['caution_px']}  Safe={dist_cal['safe_px']}  "
          f"Far={dist_cal['far_px']}")


tracker = ClickTracker()


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        assigned = tracker.handle_click(x, y)
        if assigned in ("A", "B"):
            print(f"  Train {assigned} assigned at ({x},{y})")


def main():
    print("╔══════════════════════════════════════════════════════╗")
    print("║  Distance Calibration — Click to Track              ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  1. Click on Train A (front train)                  ║")
    print("║  2. Click on Train B (rear train)                   ║")
    print("║  3. Position trains at each gap → press key:        ║")
    print("║     D=Danger  W=Warning  C=Caution  S=Safe  F=Far  ║")
    print("║  4. Press ENTER or A to SAVE and quit               ║")
    print("║     Press A/B key then click to re-assign a train   ║")
    print("║     Press Q to quit without saving                  ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print(f"❌ Cannot open camera {config.CAMERA_INDEX}")
        sys.exit(1)

    WIN = "Calibration — Click on each train"
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, mouse_callback)

    saved_flash = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        tracker.set_frame(frame)
        pos_a, pos_b = tracker.update(frame)
        dist = pixel_distance(pos_a, pos_b)
        display = frame.copy()

        # Draw tracking boxes and circles
        if pos_a and pos_a.bbox:
            bx,by,bw,bh = pos_a.bbox
            cv2.rectangle(display, (bx,by),(bx+bw,by+bh), (0,165,255), 2)
            cv2.circle(display, (pos_a.x,pos_a.y), 5, (0,165,255), -1)
            cv2.putText(display, "A (front)",
                        (pos_a.x+10, pos_a.y-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)

        if pos_b and pos_b.bbox:
            bx,by,bw,bh = pos_b.bbox
            cv2.rectangle(display, (bx,by),(bx+bw,by+bh), (0,220,50), 2)
            cv2.circle(display, (pos_b.x,pos_b.y), 5, (0,220,50), -1)
            cv2.putText(display, "B (rear)",
                        (pos_b.x+10, pos_b.y-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,220,50), 2)

        # Distance line
        if pos_a and pos_b and dist:
            cv2.line(display, (pos_a.x,pos_a.y),(pos_b.x,pos_b.y),(255,255,0),1)
            mx = (pos_a.x+pos_b.x)//2
            my = (pos_a.y+pos_b.y)//2
            cv2.putText(display, f"{dist:.0f}px",
                        (mx+6,my-6),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,0),2)

        h,w = display.shape[:2]

        # Top bar
        bar_col = (20,100,20) if saved_flash>0 else (20,20,20)
        cv2.rectangle(display,(0,0),(w,48),bar_col,-1)
        if saved_flash > 0:
            cv2.putText(display,"  ✅ SAVED — run main.py to start",
                        (8,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(100,255,100),2)
            saved_flash -= 1
        else:
            cv2.putText(display, f"  {tracker.status_text()}",
                        (8,20),cv2.FONT_HERSHEY_SIMPLEX,0.52,(255,255,255),1)
            dist_str = f"{dist:.0f}px" if dist else "---"
            cv2.putText(display,
                        f"  Gap: {dist_str}  |  "
                        f"D={dist_cal['danger_px']}  W={dist_cal['warning_px']}  "
                        f"C={dist_cal['caution_px']}  S={dist_cal['safe_px']}  "
                        f"F={dist_cal['far_px']}",
                        (8,40),cv2.FONT_HERSHEY_SIMPLEX,0.4,(200,200,200),1)

        # Bottom bar
        cv2.rectangle(display,(0,h-24),(w,h),(20,20,20),-1)
        cv2.putText(display,
                    "D=Danger  W=Warning  C=Caution  S=Safe  F=Far  "
                    "A/B=Reassign train  ENTER=Save  Q=Quit",
                    (8,h-8),cv2.FONT_HERSHEY_SIMPLEX,0.36,(150,150,150),1)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(20) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            print("  Quit — not saved.")
            break

        elif key in (ord('a'), ord('A')):
            tracker.reassign_a()
            print("  Click on Train A (front train) to re-assign")

        elif key in (ord('b'), ord('B')):
            tracker.reassign_b()
            print("  Click on Train B (rear train) to re-assign")

        elif key in (13,):   # ENTER — save
            save_calibration()
            saved_flash = 90
            break

        elif key in (ord('d'), ord('D')) and dist:
            dist_cal["danger_px"] = int(dist)
            print(f"  DANGER  = {int(dist)}px")

        elif key in (ord('w'), ord('W')) and dist:
            dist_cal["warning_px"] = int(dist)
            print(f"  WARNING = {int(dist)}px")

        elif key in (ord('c'), ord('C')) and dist:
            dist_cal["caution_px"] = int(dist)
            print(f"  CAUTION = {int(dist)}px")

        elif key in (ord('s'), ord('S')) and dist:
            dist_cal["safe_px"] = int(dist)
            print(f"  SAFE    = {int(dist)}px")

        elif key in (ord('f'), ord('F')) and dist:
            dist_cal["far_px"] = int(dist)
            print(f"  FAR     = {int(dist)}px")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
