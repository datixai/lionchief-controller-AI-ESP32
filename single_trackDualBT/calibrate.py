# calibrate.py — Distance Calibration (4-box system)
# Harry Locomotive Project 3 | Datix AI | June 2026
# Drag 4 boxes → position trains → press D/W/C/S/F → ENTER to save

import cv2
import json
import os
import sys

import config
from train_detector import (DragTracker, pixel_distance,
                             WAIT_A_HEAD, WAIT_A_TAIL,
                             WAIT_B_HEAD, WAIT_B_TAIL, TRACKING)

CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        config.CALIBRATION_FILE)

dist_cal = {
    "danger_px":  config.DISTANCE_DANGER,
    "warning_px": config.DISTANCE_WARNING,
    "caution_px": config.DISTANCE_CAUTION,
    "safe_px":    config.DISTANCE_SAFE,
    "far_px":     config.DISTANCE_FAR,
}

tracker = DragTracker()

def mouse_cb(event, x, y, flags, param):
    if   event == cv2.EVENT_LBUTTONDOWN: tracker.on_mouse_down(x, y)
    elif event == cv2.EVENT_MOUSEMOVE:   tracker.on_mouse_move(x, y)
    elif event == cv2.EVENT_LBUTTONUP:   tracker.on_mouse_up(x, y)

def save():
    with open(CAL_FILE, "w") as f:
        json.dump({"distances": dist_cal}, f, indent=2)
    print(f"✅ Saved: D={dist_cal['danger_px']} W={dist_cal['warning_px']} "
          f"C={dist_cal['caution_px']} S={dist_cal['safe_px']} F={dist_cal['far_px']}")

def main():
    print("Calibration — drag 4 boxes, set distances, ENTER to save")
    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    if not cap.isOpened():
        print("Cannot open camera"); sys.exit(1)
    for _ in range(config.CAMERA_WARMUP_FRAMES):
        cap.read()

    WIN = "Calibration"
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WIN, mouse_cb)
    saved_flash = 0

    BOX_STYLE = {
        "AH": ("A-HEAD", (0,165,255)), "AT": ("A-TAIL", (0,100,180)),
        "BH": ("B-HEAD", (0,220, 50)), "BT": ("B-TAIL", (0,140, 30)),
    }

    while True:
        ret, raw = cap.read()
        if not ret: continue
        display = cv2.resize(raw, (config.DISPLAY_W, config.DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)
        tracker.set_display_frame(display)
        pos_a, pos_b = tracker.update(display)

        a_chasing = tracker.a_is_chasing_b(pos_a, pos_b)
        dist = tracker.facing_gap(pos_a, pos_b, a_chasing)
        h, w = display.shape[:2]

        # Draw boxes
        for key,(lbl,col) in BOX_STYLE.items():
            box = tracker.get_boxes()[key]
            if not box.initialized: continue
            pos = box.get_pos()
            if pos:
                cv2.circle(display, pos, 8, col, -1)
                cv2.putText(display,lbl,(pos[0]+10,pos[1]-6),
                            cv2.FONT_HERSHEY_SIMPLEX,0.42,col,1)

        # Train body lines
        if pos_a and pos_a.head and pos_a.tail:
            cv2.line(display,pos_a.head,pos_a.tail,(0,165,255),3)
        if pos_b and pos_b.head and pos_b.tail:
            cv2.line(display,pos_b.head,pos_b.tail,(0,220,50),3)

        # Gap line
        if pos_a and pos_b and dist:
            p1 = pos_a.head if a_chasing else pos_b.head
            p2 = pos_b.tail if a_chasing else pos_a.tail
            if p1 and p2:
                cv2.line(display,p1,p2,(255,255,0),1)
                mx=(p1[0]+p2[0])//2; my=(p1[1]+p2[1])//2
                cv2.putText(display,f"{dist:.0f}px",
                            (mx+6,my-6),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,0),2)

        # Live drag
        if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
            x1=min(tracker.drag_start[0],tracker.drag_end[0])
            y1=min(tracker.drag_start[1],tracker.drag_end[1])
            x2=max(tracker.drag_start[0],tracker.drag_end[0])
            y2=max(tracker.drag_start[1],tracker.drag_end[1])
            cv2.rectangle(display,(x1,y1),(x2,y2),(200,200,200),2)

        # Top bar
        bc=(20,100,20) if saved_flash>0 else (20,20,20)
        cv2.rectangle(display,(0,0),(w,44),bc,-1)
        if saved_flash>0:
            cv2.putText(display,"✅ SAVED — run main.py",
                        (8,28),cv2.FONT_HERSHEY_SIMPLEX,0.7,(100,255,100),2)
            saved_flash-=1
        else:
            cv2.putText(display, tracker.instruction_text(),
                        (8,22),cv2.FONT_HERSHEY_SIMPLEX,0.52,(255,255,255),1)
            ds=f"{dist:.0f}px" if dist else "---"
            cv2.putText(display,
                        f"Gap:{ds}  D={dist_cal['danger_px']}  "
                        f"W={dist_cal['warning_px']}  C={dist_cal['caution_px']}  "
                        f"S={dist_cal['safe_px']}  F={dist_cal['far_px']}",
                        (8,40),cv2.FONT_HERSHEY_SIMPLEX,0.34,(200,200,200),1)

        cv2.rectangle(display,(0,h-22),(w,h),(20,20,20),-1)
        cv2.putText(display,
                    "D=Danger W=Warning C=Caution S=Safe F=Far  "
                    "A/B=Reselect  ENTER=Save  Q=Quit",
                    (8,h-7),cv2.FONT_HERSHEY_SIMPLEX,0.34,(130,130,130),1)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(20) & 0xFF
        if key in (ord('q'),ord('Q'),27): break
        elif key==13: save(); saved_flash=90
        elif key in (ord('a'),ord('A')): tracker.reselect_a()
        elif key in (ord('b'),ord('B')): tracker.reselect_b()
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

    cap.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()