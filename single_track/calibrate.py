# ══════════════════════════════════════════════════════════════════
#  calibrate.py  —  Distance + Track Mask Calibration
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  TWO STEPS:
#    Step 1: Draw track boundary mask  (press M to enter/exit)
#    Step 2: Set distance thresholds   (position trains, press D/W/C/S/F)
#
#  TRACK MASK:
#    Press M → click polygon points around the track boundary
#    Press ENTER → close polygon and preview it
#    Press M again → save mask and exit mask mode
#    People/objects outside this polygon are completely ignored.
#
#  DISTANCE THRESHOLDS:
#    Drag box on Train A then Train B → measure gap → press key
#    D=Danger  W=Warning  C=Caution  S=Safe  F=Far
#    Press ENTER to save all → calibration.json + track_mask.json
# ══════════════════════════════════════════════════════════════════

import cv2
import json
import os
import sys
import numpy as np

import config
from train_detector import DragTracker, pixel_distance, WAIT_A, WAIT_B, TRACKING

CAL_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.CALIBRATION_FILE)
MASK_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.TRACK_MASK_FILE)

dist_cal = {
    "danger_px":  config.DISTANCE_DANGER,
    "warning_px": config.DISTANCE_WARNING,
    "caution_px": config.DISTANCE_CAUTION,
    "safe_px":    config.DISTANCE_SAFE,
    "far_px":     config.DISTANCE_FAR,
}

tracker      = DragTracker()
mask_mode    = False
mask_points  = []    # polygon points for track boundary
mask_closed  = False
cursor_pos   = (0, 0)


def mouse_callback(event, x, y, flags, param):
    global cursor_pos, mask_points, mask_closed
    cursor_pos = (x, y)

    if mask_mode:
        if event == cv2.EVENT_LBUTTONDOWN:
            mask_points.append([x, y])
        elif event == cv2.EVENT_RBUTTONDOWN and mask_points:
            mask_points.pop()
            mask_closed = False
    else:
        if event == cv2.EVENT_LBUTTONDOWN: tracker.on_mouse_down(x, y)
        elif event == cv2.EVENT_MOUSEMOVE: tracker.on_mouse_move(x, y)
        elif event == cv2.EVENT_LBUTTONUP: tracker.on_mouse_up(x, y)


def save_distances():
    data = {}
    if os.path.exists(CAL_FILE):
        try:
            with open(CAL_FILE) as f:
                data = json.load(f)
        except Exception:
            data = {}
    data["distances"] = dist_cal
    with open(CAL_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"✅ Distances saved: D={dist_cal['danger_px']} "
          f"W={dist_cal['warning_px']} C={dist_cal['caution_px']} "
          f"S={dist_cal['safe_px']} F={dist_cal['far_px']}")


def save_mask():
    if len(mask_points) < 3:
        print("  Need at least 3 points for mask")
        return False
    with open(MASK_FILE, "w") as f:
        json.dump({"points": mask_points,
                   "display_w": config.DISPLAY_W,
                   "display_h": config.DISPLAY_H}, f, indent=2)
    print(f"✅ Track mask saved — {len(mask_points)} polygon points")
    return True


def main():
    global mask_mode, mask_points, mask_closed

    print("╔══════════════════════════════════════════════════════╗")
    print("║  Calibration — Track Mask + Distances               ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  M = enter/exit track mask drawing                  ║")
    print("║  (In mask mode: click=add point, right=undo)        ║")
    print("║  Drag on trains → D/W/C/S/F = set distances         ║")
    print("║  ENTER = save all   A/B = reselect train   Q = quit ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)
    if not cap.isOpened():
        print(f"Cannot open camera {config.CAMERA_INDEX}")
        sys.exit(1)

    for _ in range(config.CAMERA_WARMUP_FRAMES):
        cap.read()

    WIN = "Calibration"
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WIN, mouse_callback)
    saved_flash = 0

    while True:
        ret, raw = cap.read()
        if not ret:
            continue

        display = cv2.resize(raw, (config.DISPLAY_W, config.DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)
        tracker.set_display_frame(display)
        pos_a, pos_b = tracker.update(display)
        dist = pixel_distance(pos_a, pos_b)
        h, w = display.shape[:2]

        # ── Draw track mask overlay ─────────────────────────────
        saved_mask = tracker.get_mask()
        if saved_mask is not None:
            overlay = display.copy()
            overlay[saved_mask == 0] = (0, 0, 50)
            cv2.addWeighted(overlay, 0.3, display, 0.7, 0, display)

        if mask_mode:
            # Show polygon being drawn
            for pt in mask_points:
                cv2.circle(display, tuple(pt), 5, (0, 255, 255), -1)
            if len(mask_points) >= 2:
                pts = np.array(mask_points, dtype=np.int32)
                cv2.polylines(display, [pts], mask_closed, (0, 255, 255), 2)
                if mask_closed:
                    overlay2 = display.copy()
                    cv2.fillPoly(overlay2, [pts], (0, 80, 80))
                    cv2.addWeighted(overlay2, 0.3, display, 0.7, 0, display)
            # Cursor
            cv2.circle(display, cursor_pos, 4, (0, 255, 255), -1)
        else:
            # Train tracking display
            for pos, col, lbl in [(pos_a, (0,165,255), "A"),
                                   (pos_b, (0,220,50),  "B")]:
                if pos and pos.bbox:
                    bx,by,bw,bh = pos.bbox
                    cv2.rectangle(display,(bx,by),(bx+bw,by+bh),col,2)
                if pos:
                    cv2.circle(display,(pos.x,pos.y),5,col,-1)
                    cv2.putText(display,lbl,(pos.x+8,pos.y-8),
                                cv2.FONT_HERSHEY_SIMPLEX,0.6,col,2)
                # Head/tail boxes
                if pos and pos.head:
                    cv2.circle(display, pos.head, 7, col, 2)
                    cv2.putText(display,"H",
                                (pos.head[0]+4,pos.head[1]-4),
                                cv2.FONT_HERSHEY_SIMPLEX,0.4,col,1)
                if pos and pos.tail:
                    cv2.circle(display, pos.tail, 7, (120,120,120), 2)

            if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
                x1=min(tracker.drag_start[0],tracker.drag_end[0])
                y1=min(tracker.drag_start[1],tracker.drag_end[1])
                x2=max(tracker.drag_start[0],tracker.drag_end[0])
                y2=max(tracker.drag_start[1],tracker.drag_end[1])
                dcol=(0,165,255) if tracker.state==WAIT_A else (0,220,50)
                cv2.rectangle(display,(x1,y1),(x2,y2),dcol,2)

            if pos_a and pos_b and dist:
                cv2.line(display,(pos_a.x,pos_a.y),(pos_b.x,pos_b.y),(255,255,0),1)
                mx=(pos_a.x+pos_b.x)//2; my=(pos_a.y+pos_b.y)//2
                cv2.putText(display,f"{dist:.0f}px",
                            (mx+6,my-6),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,0),2)

        # ── Top bar ────────────────────────────────────────────
        bar_col=(0,70,70) if mask_mode else ((20,100,20) if saved_flash>0 else (20,20,20))
        cv2.rectangle(display,(0,0),(w,44),bar_col,-1)
        if mask_mode:
            pts_str = f"{len(mask_points)} pts"
            cv2.putText(display,
                        f"MASK MODE — Click polygon around track ({pts_str}) "
                        f"| Right=undo  ENTER=close  M=save+exit",
                        (8,26),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),1)
        elif saved_flash>0:
            cv2.putText(display,"✅ SAVED",
                        (8,28),cv2.FONT_HERSHEY_SIMPLEX,0.7,(100,255,100),2)
            saved_flash-=1
        else:
            cv2.putText(display, tracker.instruction_text(),
                        (8,22),cv2.FONT_HERSHEY_SIMPLEX,0.52,(255,255,255),1)
            dist_str=f"{dist:.0f}px" if dist else "---"
            cv2.putText(display,
                        f"Gap:{dist_str}  D={dist_cal['danger_px']}  "
                        f"W={dist_cal['warning_px']}  C={dist_cal['caution_px']}  "
                        f"S={dist_cal['safe_px']}  F={dist_cal['far_px']}  "
                        f"| M=draw track mask",
                        (8,40),cv2.FONT_HERSHEY_SIMPLEX,0.34,(200,200,200),1)

        cv2.rectangle(display,(0,h-22),(w,h),(20,20,20),-1)
        cv2.putText(display,
                    "D/W/C/S/F=set dist   A/B=reselect   M=mask   ENTER=save   Q=quit",
                    (8,h-7),cv2.FONT_HERSHEY_SIMPLEX,0.35,(130,130,130),1)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(20) & 0xFF

        if key in (ord('q'),ord('Q'),27): break
        elif key in (ord('m'),ord('M')):
            if mask_mode:
                # Exit mask mode — save if polygon is closed
                if mask_closed and len(mask_points)>=3:
                    save_mask()
                    saved_flash=60
                mask_mode=False
                print("  Mask mode OFF")
            else:
                mask_mode=True
                mask_points=[]
                mask_closed=False
                print("  Mask mode ON — click polygon points around track")
        elif key==13:  # ENTER
            if mask_mode:
                if len(mask_points)>=3:
                    mask_closed=True
                    print(f"  Polygon closed — {len(mask_points)} points. Press M to save.")
            else:
                save_distances()
                saved_flash=90
        elif not mask_mode:
            if key in (ord('a'),ord('A')): tracker.reselect_a()
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

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()