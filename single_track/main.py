# ══════════════════════════════════════════════════════════════════
#  main.py  —  Single Track Safe Distance v8.0
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  SETUP (3 drags):
#    1. Drag box around TABLE  →  defines detection boundary
#    2. Drag box around TRAIN A  →  tracking circle appears
#    3. Drag box around TRAIN B  →  tracking circle appears
#    → Y = auto control   N = manual control
#
#  SPEED KEYS:
#    ↑ / +   Speed up Train B
#    ↓ / -   Speed down Train B
#    1-7     Set exact speed
#    M       Toggle Manual ↔ Auto
#    S       Stop    R = Resume    P = Pause
#    A / B   Re-select that train
#    T       Redraw table boundary
#    H / L   Horn / Lights
#    Q / ESC Quit
# ══════════════════════════════════════════════════════════════════

import cv2
import json
import os
import sys
import time
import logging
import numpy as np

import config
from train_detector   import (DragTracker, pixel_distance,
                               WAIT_TABLE, WAIT_A, WAIT_B, TRACKING)
from speed_controller import SpeedController, Zone
from ble_controller   import TrainBLEController

handlers = [logging.StreamHandler(sys.stdout)]
if config.LOG_TO_FILE:
    handlers.append(logging.FileHandler(config.LOG_FILE, mode="a"))
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(message)s",
                    datefmt="%H:%M:%S", handlers=handlers)
logger = logging.getLogger("Main")

SELECTION_STATES = (WAIT_TABLE, WAIT_A, WAIT_B)
CAL_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         config.CALIBRATION_FILE)


def load_calibration():
    if not os.path.exists(CAL_FILE):
        return
    try:
        with open(CAL_FILE) as f:
            d = json.load(f).get("distances", {})
        if d.get("danger_px"):  config.DISTANCE_DANGER  = d["danger_px"]
        if d.get("warning_px"): config.DISTANCE_WARNING = d["warning_px"]
        if d.get("caution_px"): config.DISTANCE_CAUTION = d["caution_px"]
        if d.get("safe_px"):    config.DISTANCE_SAFE    = d["safe_px"]
        if d.get("far_px"):     config.DISTANCE_FAR     = d["far_px"]
    except Exception:
        pass


# ── Global tracker ────────────────────────────────────────────────
_tracker = None

def mouse_callback(event, x, y, flags, param):
    if _tracker is None:
        return
    if   event == cv2.EVENT_LBUTTONDOWN: _tracker.on_mouse_down(x, y)
    elif event == cv2.EVENT_MOUSEMOVE:   _tracker.on_mouse_move(x, y)
    elif event == cv2.EVENT_LBUTTONUP:   _tracker.on_mouse_up(x, y)


# ── Drawing helpers ───────────────────────────────────────────────

def draw_tracking_circle(frame, pos, color, label: str):
    """
    Draw tracking circle around a train.
    Shows LOCKED state (dashed/different style) when train is stopped.
    """
    if pos is None:
        return
    cx, cy, r = pos.x, pos.y, pos.radius
    locked     = getattr(pos, "locked", False)

    if locked:
        # Stopped train: smaller solid inner circle + dashed outer ring
        # Draw dashed circle using line segments
        import math
        segs = 16
        for i in range(segs):
            if i % 2 == 0:   # draw every other segment = dashed effect
                a1 = 2 * math.pi * i / segs
                a2 = 2 * math.pi * (i + 1) / segs
                p1 = (int(cx + r * math.cos(a1)), int(cy + r * math.sin(a1)))
                p2 = (int(cx + r * math.cos(a2)), int(cy + r * math.sin(a2)))
                cv2.line(frame, p1, p2, color, 2)
        # Small filled centre
        cv2.circle(frame, (cx, cy), 5, color, -1)
        state_lbl = "STOPPED"
    else:
        # Moving train: solid circle with light fill and crosshair marks
        overlay = frame.copy()
        cv2.circle(overlay, (cx, cy), r, color, -1)
        cv2.addWeighted(overlay, 0.08, frame, 0.92, 0, frame)
        cv2.circle(frame, (cx, cy), r, color, 2)
        cv2.line(frame, (cx-r, cy),   (cx-r+8, cy),  color, 2)
        cv2.line(frame, (cx+r, cy),   (cx+r-8, cy),  color, 2)
        cv2.line(frame, (cx, cy-r),   (cx, cy-r+8),  color, 2)
        cv2.line(frame, (cx, cy+r),   (cx, cy+r-8),  color, 2)
        cv2.circle(frame, (cx, cy), 3, color, -1)
        state_lbl = ""

    # Label
    lbl_text = f"{label} {state_lbl}".strip()
    cv2.putText(frame, lbl_text,
                (cx + r + 6, cy - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2)


def draw_table_rect(frame, rect, color=(0, 200, 200)):
    """Draw the saved table boundary rectangle."""
    if rect is None:
        return
    x1, y1, x2, y2 = rect
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
    cv2.addWeighted(overlay, 0.06, frame, 0.94, 0, frame)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)


def draw_confirm_overlay(display):
    h, w = display.shape[:2]
    ov = display.copy()
    cv2.rectangle(ov, (0,0), (w,h), (0,0,0), -1)
    cv2.addWeighted(ov, 0.55, display, 0.45, 0, display)
    bx, by, bw, bh = w//2-290, h//2-75, 580, 150
    cv2.rectangle(display, (bx,by), (bx+bw,by+bh), (30,30,30), -1)
    cv2.rectangle(display, (bx,by), (bx+bw,by+bh), (0,220,50), 2)
    cv2.putText(display, "Both trains selected!  Start Train B?",
                (bx+16, by+38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255,255,255), 2)
    cv2.putText(display, "Y  =  Auto control  (gap detection)",
                (bx+60, by+78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0,220,50), 2)
    cv2.putText(display, "N  =  Manual control  (you set speed)",
                (bx+60, by+118),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0,200,255), 2)


# ── Overlay ───────────────────────────────────────────────────────

def draw_overlay(display, tracker, pos_a, pos_b, dist, zone,
                 speed, user_speed, ble_ok, manual_mode, a_chasing):
    h, w = display.shape[:2]
    now  = time.time()
    zcol = config.ZONE_COLORS.get(zone, (120,120,120))

    # Table boundary (subtle tint inside rectangle)
    draw_table_rect(display, tracker._table_rect)

    # Search area circles (very subtle dark gray)
    if pos_a:
        tracker._tkr_a.draw_search_area(display, (50,50,50))
    if pos_b:
        tracker._tkr_b.draw_search_area(display, (50,50,50))

    # Tracking circles — the key visual
    draw_tracking_circle(display, pos_a, (0,165,255), "Train A")
    draw_tracking_circle(display, pos_b, (0,220, 50), f"Train B  {speed}")

    # Gap line between trains
    if pos_a and pos_b and dist is not None:
        cv2.line(display, (pos_a.x,pos_a.y), (pos_b.x,pos_b.y), zcol, 1)
        mx = (pos_a.x + pos_b.x) // 2
        my = (pos_a.y + pos_b.y) // 2
        cv2.putText(display, f"{dist:.0f}px | {zone}",
                    (mx+6, my-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, zcol, 2)

    # Live drag rectangle
    if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
        x1 = min(tracker.drag_start[0], tracker.drag_end[0])
        y1 = min(tracker.drag_start[1], tracker.drag_end[1])
        x2 = max(tracker.drag_start[0], tracker.drag_end[0])
        y2 = max(tracker.drag_start[1], tracker.drag_end[1])
        drag_col = {
            WAIT_TABLE: (0, 200, 200),
            WAIT_A:     (0, 165, 255),
            WAIT_B:     (0, 220,  50),
        }.get(tracker.state, (200, 200, 200))
        cv2.rectangle(display, (x1,y1), (x2,y2), drag_col, 2)
        # Label inside top-left of drag rect
        lbl = {WAIT_TABLE:"TABLE",WAIT_A:"Train A",WAIT_B:"Train B"}
        cv2.putText(display,
                    lbl.get(tracker.state, ""),
                    (x1+4, y1+18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, drag_col, 2)

    # Top status bar
    if tracker.state in SELECTION_STATES:
        step_col = {WAIT_TABLE:(0,200,200),WAIT_A:(0,165,255),WAIT_B:(0,220,50)}
        col = step_col.get(tracker.state, (200,200,200))
        cv2.rectangle(display, (0,0), (w,52), (25,15,0), -1)
        cv2.putText(display, tracker.instruction_text(),
                    (8,28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
        cv2.putText(display,
                    "Hold left mouse + drag rectangle → release",
                    (8,46), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (160,160,160), 1)
    else:
        if zone == Zone.ESCAPE:       barc = (55,0,55)
        elif zone == Zone.DANGER:     barc = (0,0,85)
        elif zone == Zone.SAFE:       barc = (0,52,0)
        elif manual_mode:             barc = (45,28,0)
        else:                         barc = (22,22,22)
        cv2.rectangle(display, (0,0), (w,48), barc, -1)

        if manual_mode:
            msg  = f"MANUAL MODE — ↑↓ or +- to adjust speed"
            mcol = (0,200,255)
        elif zone == Zone.ESCAPE:
            msg  = f"A IS BEHIND B — Train B escaping (speed {speed})"
            mcol = (200,80,200)
        elif not tracker.tracking_a or not tracker.tracking_b:
            msg  = "Tracker lost — press A or B to re-select"
            mcol = (0,80,255)
        else:
            msgs = {
                Zone.DANGER:  f"STOP — gap too small",
                Zone.WARNING: f"WARNING — slowing Train B (speed {speed})",
                Zone.CAUTION: f"CAUTION — speed {speed}",
                Zone.SAFE:    f"SAFE — following at speed {speed}",
                Zone.FAR:     f"FAR — catching up (speed {speed})",
                Zone.UNKNOWN: f"Not visible — holding speed {speed}",
            }
            msg  = msgs.get(zone, zone)
            mcol = zcol

        cv2.putText(display, msg, (8,26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        mode_lbl = "MANUAL" if manual_mode else "AUTO  "
        mode_col = (0,200,255) if manual_mode else (100,220,100)
        cv2.putText(display,
                    f"[{mode_lbl}]  "
                    f"A:{'OK' if tracker.tracking_a else 'LOST'}  "
                    f"B:{'OK' if tracker.tracking_b else 'LOST'}  |  "
                    f"BLE:{'OK' if ble_ok else 'connecting'}  |  "
                    f"Speed:{speed}/7  Gap:{f'{dist:.0f}px' if dist is not None else '---'}  |  "
                    f"M=Auto/Manual  T=Table  A/B=reselect",
                    (8,44), cv2.FONT_HERSHEY_SIMPLEX, 0.36, mode_col, 1)

    # Flash confirmations
    flashes = [
        (tracker.flash_table, "Table boundary saved — now select Train A", (0,200,200)),
        (tracker.flash_a,     "Train A locked — now select Train B",       (0,165,255)),
        (tracker.flash_b,     "Train B locked — both trains tracking!",    (0,220, 50)),
    ]
    for t, msg, col in flashes:
        if now - t < 2.5:
            fy = h//2 - 28
            cv2.rectangle(display,(0,fy),(w,fy+50),(15,15,15),-1)
            cv2.rectangle(display,(0,fy),(w,fy+50),col,3)
            cv2.putText(display, f"  ✅  {msg}",
                        (20,fy+34), cv2.FONT_HERSHEY_SIMPLEX,0.82,col,2)
            break

    # Speed panel (bottom right)
    px = w - 130
    cv2.rectangle(display,(px,h-90),(w,h),(28,28,28),-1)
    cv2.putText(display,"SPEED",(px+28,h-70),
                cv2.FONT_HERSHEY_SIMPLEX,0.4,(120,120,120),1)
    spd_col = (0,60,200) if zone==Zone.DANGER else \
              (0,200,255) if manual_mode else (0,200,60)
    cv2.putText(display, str(speed),(px+28,h-30),
                cv2.FONT_HERSHEY_SIMPLEX,1.9,spd_col,3)
    cv2.putText(display,"/7",(px+88,h-30),
                cv2.FONT_HERSHEY_SIMPLEX,0.55,(100,100,100),1)
    mode_s = "MAN" if manual_mode else "AUTO"
    cv2.putText(display,mode_s,(px+30,h-10),
                cv2.FONT_HERSHEY_SIMPLEX,0.38,
                (0,200,255) if manual_mode else (100,220,100),1)

    # Bottom bar
    cv2.rectangle(display,(0,h-22),(w-130,h),(18,18,18),-1)
    cv2.putText(display,
                "DRAG=select  A/B=reselect  ↑↓/+-=Speed  M=Mode  "
                "S=Stop  R=Resume  T=Table  H=Horn  Q=Quit",
                (8,h-7),cv2.FONT_HERSHEY_SIMPLEX,0.32,(120,120,120),1)
    return display


# ── Main ──────────────────────────────────────────────────────────

def main():
    global _tracker

    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Safe Distance v8.0                     ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  Drag 1: TABLE boundary  (big box around table)     ║")
    print("║  Drag 2: TRAIN A         (front train)              ║")
    print("║  Drag 3: TRAIN B         (rear BLE train)           ║")
    print("║  Then: Y=auto / N=manual                            ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    load_calibration()
    ble = TrainBLEController()
    ble.start()

    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)
    if not cap.isOpened():
        logger.error("Cannot open camera")
        ble.shutdown(); sys.exit(1)

    logger.info("Warming up camera...")
    for _ in range(config.CAMERA_WARMUP_FRAMES):
        cap.read()
    logger.info("Camera ready ✅")

    _tracker = DragTracker()
    ctrl     = SpeedController()

    cv2.namedWindow(config.WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(config.WINDOW_TITLE, mouse_callback)

    manual_mode     = False
    waiting_confirm = False
    prev_state      = _tracker.state
    horn_on         = False
    lights_on       = False
    last_ka         = time.time()
    start_time      = time.time()
    frame_count     = 0
    stops_sent      = 0

    while True:
        ret, raw = cap.read()
        if not ret:
            time.sleep(0.02); continue

        frame_count += 1
        now = time.time()

        display = cv2.resize(raw, (config.DISPLAY_W, config.DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)

        _tracker.set_display_frame(display)
        pos_a, pos_b = _tracker.update(display)

        # Detect when Train B was just selected → show Y/N
        if (_tracker.state == TRACKING and
                prev_state == WAIT_B and
                not waiting_confirm):
            waiting_confirm = True
            logger.info("Both trains ready — awaiting Y/N confirmation")

        prev_state = _tracker.state

        a_chasing = _tracker.a_is_chasing_b(pos_a, pos_b)
        dist      = _tracker.facing_gap(pos_a, pos_b, a_chasing)

        # Speed control
        if not waiting_confirm and _tracker.ready:
            if manual_mode:
                speed = ble.current_speed
                zone  = Zone.UNKNOWN
            else:
                eff = None if (_tracker.is_a_missing() or
                               _tracker.is_b_missing()) else dist
                speed, zone = ctrl.update(eff, a_chasing)
                if ctrl.should_send_command(speed):
                    if zone == Zone.DANGER and ble.current_speed != 0:
                        stops_sent += 1
                    ble.set_speed(speed)
                    ctrl.command_sent(speed)
        else:
            speed = ble.current_speed
            zone  = Zone.UNKNOWN

        if now - last_ka >= config.KEEPALIVE_INTERVAL:
            last_ka = now
            ble.keepalive()

        draw_overlay(display, _tracker, pos_a, pos_b, dist, zone,
                     speed, ctrl.user_speed, ble.connected,
                     manual_mode, a_chasing)

        if waiting_confirm:
            draw_confirm_overlay(display)

        cv2.imshow(config.WINDOW_TITLE, display)

        # Keys
        raw_key = cv2.waitKey(1)
        if raw_key == -1:
            continue

        if waiting_confirm:
            k = raw_key & 0xFF
            if k in (ord('y'), ord('Y')):
                waiting_confirm = False
                manual_mode     = False
                logger.info("AUTO mode started")
            elif k in (ord('n'), ord('N')):
                waiting_confirm = False
                manual_mode     = True
                logger.info("MANUAL mode — use ↑↓ or 1-7")
            continue

        key = raw_key & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('a'), ord('A')):
            _tracker.reselect_a()
        elif key in (ord('b'), ord('B')):
            _tracker.reselect_b()
        elif key in (ord('t'), ord('T')):
            _tracker.redraw_table()
            logger.info("Draw table boundary again")
        elif key in (ord('m'), ord('M'), ord('p'), ord('P')):
            manual_mode = not manual_mode
            ctrl.reset()
            logger.info(f"{'MANUAL' if manual_mode else 'AUTO'} mode")
        elif key in (ord('s'), ord('S')):
            ble.send_stop(); ctrl.reset()
        elif key in (ord('r'), ord('R')):
            ble.set_speed(ctrl.user_speed); ctrl.reset()
        elif key in (ord('h'), ord('H')):
            horn_on = not horn_on
            ble.horn_on() if horn_on else ble.horn_off()
        elif key in (ord('l'), ord('L')):
            lights_on = not lights_on
            ble.lights_on() if lights_on else ble.lights_off()
        elif key in (ord('1'),ord('2'),ord('3'),ord('4'),
                     ord('5'),ord('6'),ord('7')):
            ctrl.set_user_speed(int(chr(key)))
            ble.set_speed(ctrl.user_speed)
        elif key in (ord('+'), ord('=')):
            ctrl.set_user_speed(ctrl.user_speed + 1)
            ble.set_speed(ctrl.user_speed)
        elif key in (ord('-'), ord('_')):
            ctrl.set_user_speed(max(1, ctrl.user_speed - 1))
            ble.set_speed(ctrl.user_speed)
        # Arrow keys (Windows raw values)
        elif raw_key in (2490368, 65362):   # Up
            ctrl.set_user_speed(ctrl.user_speed + 1)
            ble.set_speed(ctrl.user_speed)
            logger.info(f"Speed ↑ {ctrl.user_speed}")
        elif raw_key in (2621440, 65364):   # Down
            ctrl.set_user_speed(max(1, ctrl.user_speed - 1))
            ble.set_speed(ctrl.user_speed)
            logger.info(f"Speed ↓ {ctrl.user_speed}")

    ble.shutdown(); cap.release(); cv2.destroyAllWindows()
    elapsed = int(time.time() - start_time)
    print(f"\n  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Frames:{frame_count}  Stops:{stops_sent}\n")


if __name__ == "__main__":
    main()