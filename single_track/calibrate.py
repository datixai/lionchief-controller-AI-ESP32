# ══════════════════════════════════════════════════════════════════
#  calibrate.py  —  Distance Threshold Calibration
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  HOW TO USE:
#    1. python calibrate.py
#    2. DRAG a box around Train A (front) → release
#    3. DRAG a box around Train B (rear)  → release
#    4. Position trains at each distance and press:
#         D = DANGER   W = WARNING   C = CAUTION
#         S = SAFE     F = FAR
#    5. Press ENTER to save → calibration.json
#    6. Press A or B to re-draw that train's box anytime
# ══════════════════════════════════════════════════════════════════

import cv2
import json
import os
import sys

import config
from train_detector import DragTracker, pixel_distance, WAIT_A, WAIT_B, TRACKING

CAL_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), config.CALIBRATION_FILE)

dist_cal = {
    "danger_px":  config.DISTANCE_DANGER,
    "warning_px": config.DISTANCE_WARNING,
    "caution_px": config.DISTANCE_CAUTION,
    "safe_px":    config.DISTANCE_SAFE,
    "far_px":     config.DISTANCE_FAR,
}

tracker = DragTracker()


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        tracker.on_mouse_down(x, y)
    elif event == cv2.EVENT_MOUSEMOVE:
        tracker.on_mouse_move(x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        tracker.on_mouse_up(x, y)


def save_cal():
    with open(CAL_FILE, "w") as f:
        json.dump({"distances": dist_cal}, f, indent=2)
    print(f"\n✅ Saved → {CAL_FILE}")
    print(f"   D={dist_cal['danger_px']}  W={dist_cal['warning_px']}  "
          f"C={dist_cal['caution_px']}  S={dist_cal['safe_px']}  "
          f"F={dist_cal['far_px']}")


def main():
    print("╔══════════════════════════════════════════════════════╗")
    print("║  Calibration — Drag to select each train            ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  DRAG box around Train A  then  Train B             ║")
    print("║  Position trains at each gap → press D/W/C/S/F      ║")
    print("║  ENTER = save   A/B = reselect train   Q = quit     ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print(f"Cannot open camera {config.CAMERA_INDEX}")
        sys.exit(1)

    WIN = "Calibration"
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
        h, w = frame.shape[:2]
        display = frame.copy()

        # Draw tracker boxes
        for pos, col, lbl in [
            (pos_a, (0,165,255), "A (front)"),
            (pos_b, (0,220,50),  "B (rear)"),
        ]:
            if pos and pos.bbox:
                bx,by,bw,bh = pos.bbox
                cv2.rectangle(display,(bx,by),(bx+bw,by+bh),col,2)
                cv2.circle(display,(pos.x,pos.y),4,col,-1)
                cv2.putText(display, lbl,
                            (pos.x+8,pos.y-8),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,col,2)

        # Draw live drag rectangle
        if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
            x1 = min(tracker.drag_start[0], tracker.drag_end[0])
            y1 = min(tracker.drag_start[1], tracker.drag_end[1])
            x2 = max(tracker.drag_start[0], tracker.drag_end[0])
            y2 = max(tracker.drag_start[1], tracker.drag_end[1])
            drag_col = (0,165,255) if tracker.state==WAIT_A else (0,220,50)
            cv2.rectangle(display,(x1,y1),(x2,y2),drag_col,2)

        # Distance line
        if pos_a and pos_b and dist:
            cv2.line(display,(pos_a.x,pos_a.y),(pos_b.x,pos_b.y),(255,255,0),1)
            mx=(pos_a.x+pos_b.x)//2; my=(pos_a.y+pos_b.y)//2
            cv2.putText(display,f"{dist:.0f}px",
                        (mx+6,my-6),cv2.FONT_HERSHEY_SIMPLEX,0.65,(255,255,0),2)

        # Top bar
        bar_col=(20,100,20) if saved_flash>0 else (20,20,20)
        cv2.rectangle(display,(0,0),(w,48),bar_col,-1)
        if saved_flash>0:
            cv2.putText(display,"✅ SAVED — run main.py to start",
                        (8,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(100,255,100),2)
            saved_flash-=1
        else:
            cv2.putText(display, tracker.instruction_text(),
                        (8,22),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)
            dist_str = f"{dist:.0f}px" if dist else "---"
            cv2.putText(display,
                        f"Gap: {dist_str}  |  "
                        f"D={dist_cal['danger_px']}  W={dist_cal['warning_px']}  "
                        f"C={dist_cal['caution_px']}  S={dist_cal['safe_px']}  "
                        f"F={dist_cal['far_px']}",
                        (8,42),cv2.FONT_HERSHEY_SIMPLEX,0.38,(200,200,200),1)

        # Bottom bar
        cv2.rectangle(display,(0,h-24),(w,h),(20,20,20),-1)
        cv2.putText(display,
                    "D=Danger  W=Warning  C=Caution  S=Safe  F=Far  "
                    "A/B=Reselect  ENTER=Save  Q=Quit",
                    (8,h-8),cv2.FONT_HERSHEY_SIMPLEX,0.36,(130,130,130),1)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(20) & 0xFF

        if key in (ord('q'),ord('Q'),27):
            break
        elif key == 13:  # ENTER
            save_cal(); saved_flash=90
        elif key in (ord('a'),ord('A')):
            tracker.reselect_a()
        elif key in (ord('b'),ord('B')):
            tracker.reselect_b()
        elif key in (ord('d'),ord('D')) and dist:
            dist_cal["danger_px"]=int(dist); print(f"DANGER={int(dist)}px")
        elif key in (ord('w'),ord('W')) and dist:
            dist_cal["warning_px"]=int(dist); print(f"WARNING={int(dist)}px")
        elif key in (ord('c'),ord('C')) and dist:
            dist_cal["caution_px"]=int(dist); print(f"CAUTION={int(dist)}px")
        elif key in (ord('s'),ord('S')) and dist:
            dist_cal["safe_px"]=int(dist); print(f"SAFE={int(dist)}px")
        elif key in (ord('f'),ord('F')) and dist:
            dist_cal["far_px"]=int(dist); print(f"FAR={int(dist)}px")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
