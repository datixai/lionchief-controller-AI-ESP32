# ══════════════════════════════════════════════════════════════════
#  main.py  —  Single Track Dual BLE Safe Distance v1.0
#  Harry Locomotive Project 3 BT  |  Datix AI  |  June 2026
#
#  BOTH trains are Bluetooth. Each has its own speed controls.
#  Commands NEVER cross between trains.
#
#  TRAIN A (front) speed keys:
#    [   = slow down Train A
#    ]   = speed up Train A
#    F1  = stop Train A
#    F2  = resume Train A
#
#  TRAIN B (rear) speed keys — gap detection controls in auto mode:
#    1-7       = set Train B base speed
#    Arrow Up / +  = speed up Train B
#    Arrow Down / - = slow down Train B
#
#  SHARED KEYS:
#    E    = Emergency stop BOTH trains
#    M    = Toggle Manual / Auto mode
#    S    = Stop Train B (manual)
#    R    = Resume Train B (manual)
#    A    = Re-select Train A tracking box
#    B    = Re-select Train B tracking box
#    T    = Redraw table boundary
#    L    = Toggle track path learning
#    H    = Horn (both trains)
#    Q    = Quit
#
#  AUTO MODE:
#    Train A runs at its user-set speed (adjust with [ ])
#    Train B speed controlled by gap detection
#    DANGER zone: BOTH trains stopped
#    ESCAPE mode: Train B speeds up, Train A slows by 1
#
#  MANUAL MODE:
#    Train A: [ ] keys
#    Train B: arrows / 1-7 keys
#    No gap detection
# ══════════════════════════════════════════════════════════════════

import cv2
import json
import os
import sys
import time
import io
import logging
import numpy as np

import config
from train_detector   import (DragTracker, pixel_distance,
                               WAIT_TABLE, WAIT_A, WAIT_B, TRACKING)
from speed_controller import SpeedController, Zone
from ble_controller   import DualBLEController

import io as _io
_stdout_utf8 = _io.TextIOWrapper(
    sys.stdout.buffer, encoding='utf-8', errors='replace')
handlers = [logging.StreamHandler(_stdout_utf8)]
if config.LOG_TO_FILE:
    handlers.append(logging.FileHandler(
        config.LOG_FILE, mode="a", encoding='utf-8'))
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(message)s",
                    datefmt="%H:%M:%S", handlers=handlers)
logger = logging.getLogger("Main")

SELECTION_STATES = (WAIT_TABLE, WAIT_A, WAIT_B)
CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
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


_tracker = None

def mouse_callback(event, x, y, flags, param):
    if _tracker is None:
        return
    if   event == cv2.EVENT_LBUTTONDOWN: _tracker.on_mouse_down(x, y)
    elif event == cv2.EVENT_MOUSEMOVE:   _tracker.on_mouse_move(x, y)
    elif event == cv2.EVENT_LBUTTONUP:   _tracker.on_mouse_up(x, y)


# ── Drawing helpers ───────────────────────────────────────────────

def draw_tracking_circle(frame, pos, color, label: str):
    if pos is None:
        return
    cx, cy, r = pos.x, pos.y, pos.radius
    locked = getattr(pos, "locked", False)
    import math
    if locked:
        segs = 16
        for i in range(segs):
            if i % 2 == 0:
                a1=(2*math.pi*i/segs); a2=(2*math.pi*(i+1)/segs)
                p1=(int(cx+r*math.cos(a1)),int(cy+r*math.sin(a1)))
                p2=(int(cx+r*math.cos(a2)),int(cy+r*math.sin(a2)))
                cv2.line(frame,p1,p2,color,2)
        cv2.circle(frame,(cx,cy),5,color,-1)
        state_s = "STOPPED"
    else:
        ov=frame.copy()
        cv2.circle(ov,(cx,cy),r,color,-1)
        cv2.addWeighted(ov,0.08,frame,0.92,0,frame)
        cv2.circle(frame,(cx,cy),r,color,2)
        cv2.line(frame,(cx-r,cy),(cx-r+8,cy),color,2)
        cv2.line(frame,(cx+r,cy),(cx+r-8,cy),color,2)
        cv2.line(frame,(cx,cy-r),(cx,cy-r+8),color,2)
        cv2.line(frame,(cx,cy+r),(cx,cy+r-8),color,2)
        cv2.circle(frame,(cx,cy),3,color,-1)
        state_s = ""
    cv2.putText(frame,f"{label} {state_s}".strip(),
                (cx+r+6,cy-6),cv2.FONT_HERSHEY_SIMPLEX,0.50,color,2)


def draw_confidence_bar(frame, x, y, conf, color, label):
    bar_w, bar_h = 40, 6
    filled = int(bar_w * max(0.0, min(1.0, conf)))
    cv2.rectangle(frame,(x,y),(x+bar_w,y+bar_h),(50,50,50),-1)
    bc = (0,200,60) if conf>0.75 else ((0,180,220) if conf>0.4 else (0,60,220))
    cv2.rectangle(frame,(x,y),(x+filled,y+bar_h),bc,-1)
    cv2.rectangle(frame,(x,y),(x+bar_w,y+bar_h),(100,100,100),1)
    cv2.putText(frame,f"{conf:.0%}",(x+bar_w+4,y+bar_h),
                cv2.FONT_HERSHEY_SIMPLEX,0.32,bc,1)


def draw_table_rect(frame, rect):
    if rect is None: return
    x1,y1,x2,y2=rect
    ov=frame.copy()
    cv2.rectangle(ov,(x1,y1),(x2,y2),(0,200,200),-1)
    cv2.addWeighted(ov,0.06,frame,0.94,0,frame)
    cv2.rectangle(frame,(x1,y1),(x2,y2),(0,200,200),1)


def draw_confirm_overlay(display):
    h,w=display.shape[:2]
    ov=display.copy()
    cv2.rectangle(ov,(0,0),(w,h),(0,0,0),-1)
    cv2.addWeighted(ov,0.55,display,0.45,0,display)
    bx,by,bw,bh=w//2-300,h//2-80,600,160
    cv2.rectangle(display,(bx,by),(bx+bw,by+bh),(30,30,30),-1)
    cv2.rectangle(display,(bx,by),(bx+bw,by+bh),(0,220,50),2)
    cv2.putText(display,"Both trains selected! Start now?",
                (bx+16,by+38),cv2.FONT_HERSHEY_SIMPLEX,0.72,(255,255,255),2)
    cv2.putText(display,"Y = Auto mode (gap detection controls Train B)",
                (bx+40,by+78),cv2.FONT_HERSHEY_SIMPLEX,0.60,(0,220,50),2)
    cv2.putText(display,"N = Manual mode (you control both trains)",
                (bx+40,by+118),cv2.FONT_HERSHEY_SIMPLEX,0.60,(0,200,255),2)


# ── Main overlay ──────────────────────────────────────────────────

def draw_overlay(display, tracker, pos_a, pos_b, dist, zone,
                 spd_a, spd_b, ble, manual_mode, a_chasing,
                 conf_a=1.0, conf_b=1.0, ctrl=None):
    h,w = display.shape[:2]
    now = time.time()
    zcol = config.ZONE_COLORS.get(zone,(120,120,120))

    # Table rect
    draw_table_rect(display, tracker._table_rect)

    # Track path dots
    if tracker.track_path.has_path:
        for px,py in tracker.track_path.points[::3]:
            cv2.circle(display,(int(px),int(py)),2,
                       (0,200,200) if tracker.track_path.recording
                       else (60,50,0),-1)

    # Search area circles
    tracker._tkr_a.draw_search_area(display,(50,50,50))
    tracker._tkr_b.draw_search_area(display,(50,50,50))

    # Tracking circles
    draw_tracking_circle(display,pos_a,(0,165,255),"A (front)")
    draw_tracking_circle(display,pos_b,(0,220, 50),f"B (rear) {spd_b}")

    # Confidence bars
    if pos_a:
        draw_confidence_bar(display,pos_a.x+pos_a.radius+10,
                            pos_a.y-20,conf_a,(0,165,255),"A")
    if pos_b:
        draw_confidence_bar(display,pos_b.x+pos_b.radius+10,
                            pos_b.y-20,conf_b,(0,220,50),"B")

    # Gap line
    if pos_a and pos_b and dist is not None:
        cv2.line(display,(pos_a.x,pos_a.y),(pos_b.x,pos_b.y),zcol,1)
        mx=(pos_a.x+pos_b.x)//2; my=(pos_a.y+pos_b.y)//2
        rate_s=""
        if ctrl and abs(ctrl.gap_rate)>0.5:
            arr="v" if ctrl.gap_rate<0 else "^"
            rate_s=f" {arr}{abs(ctrl.gap_rate):.0f}px/f"
        cv2.putText(display,f"{dist:.0f}px | {zone}{rate_s}",
                    (mx+6,my-6),cv2.FONT_HERSHEY_SIMPLEX,0.52,zcol,2)

    # Live drag
    if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
        x1=min(tracker.drag_start[0],tracker.drag_end[0])
        y1=min(tracker.drag_start[1],tracker.drag_end[1])
        x2=max(tracker.drag_start[0],tracker.drag_end[0])
        y2=max(tracker.drag_start[1],tracker.drag_end[1])
        dc={WAIT_TABLE:(0,200,200),WAIT_A:(0,165,255),WAIT_B:(0,220,50)}
        dcol=dc.get(tracker.state,(200,200,200))
        cv2.rectangle(display,(x1,y1),(x2,y2),dcol,2)
        lbl={WAIT_TABLE:"TABLE",WAIT_A:"Train A",WAIT_B:"Train B"}
        cv2.putText(display,lbl.get(tracker.state,""),(x1+4,y1+18),
                    cv2.FONT_HERSHEY_SIMPLEX,0.55,dcol,2)

    # Top status bar
    if tracker.state in SELECTION_STATES:
        sc={WAIT_TABLE:(0,200,200),WAIT_A:(0,165,255),WAIT_B:(0,220,50)}
        col=sc.get(tracker.state,(200,200,200))
        cv2.rectangle(display,(0,0),(w,52),(25,15,0),-1)
        cv2.putText(display,tracker.instruction_text(),
                    (8,28),cv2.FONT_HERSHEY_SIMPLEX,0.65,col,2)
        cv2.putText(display,"Hold left mouse + drag a box → release",
                    (8,46),cv2.FONT_HERSHEY_SIMPLEX,0.36,(160,160,160),1)
    else:
        if zone==Zone.ESCAPE:        barc=(55,0,55)
        elif zone==Zone.DANGER:      barc=(0,0,85)
        elif zone==Zone.SAFE:        barc=(0,52,0)
        elif manual_mode:            barc=(45,28,0)
        else:                        barc=(22,22,22)
        cv2.rectangle(display,(0,0),(w,48),barc,-1)

        if manual_mode:
            msg  = f"MANUAL — A:[  ] keys spd:{spd_a}   B:arrows/1-7 spd:{spd_b}"
            mcol = (0,200,255)
        elif zone==Zone.ESCAPE:
            msg  = f"A BEHIND B — Train B escaping (B:{spd_b}) A slowing (A:{spd_a})"
            mcol = (200,80,200)
        elif zone==Zone.DANGER:
            msg  = f"DANGER — BOTH TRAINS STOPPED"
            mcol = (80,80,255)
        elif not tracker.tracking_a or not tracker.tracking_b:
            msg  = "Tracker lost — press A or B to re-select"
            mcol = (0,80,255)
        else:
            msgs={Zone.WARNING:f"WARNING — slowing B (spd:{spd_b})",
                  Zone.CAUTION:f"CAUTION — B spd:{spd_b}",
                  Zone.SAFE:   f"SAFE — A:{spd_a}  B:{spd_b}",
                  Zone.FAR:    f"FAR — B catching up (spd:{spd_b})",
                  Zone.UNKNOWN:f"Not visible — holding"}
            msg=msgs.get(zone,zone); mcol=zcol

        cv2.putText(display,msg,(8,26),
                    cv2.FONT_HERSHEY_SIMPLEX,0.62,(255,255,255),2)
        mode_s="MANUAL" if manual_mode else "AUTO"
        mode_c=(0,200,255) if manual_mode else (100,220,100)
        ble_s=(f"A:{'OK' if ble.connected_a else 'wait'} "
               f"B:{'OK' if ble.connected_b else 'wait'}")
        cv2.putText(display,
                    f"[{mode_s}] {ble_s} | "
                    f"Gap:{f'{dist:.0f}px' if dist is not None else '---'} | "
                    f"M=mode E=EmergStop T=table L=path A/B=reselect",
                    (8,44),cv2.FONT_HERSHEY_SIMPLEX,0.34,mode_c,1)

    # Flash
    flashes=[
        (tracker.flash_table,"Table saved — select Train A",(0,200,200)),
        (tracker.flash_a,    "Train A locked — select Train B",(0,165,255)),
        (tracker.flash_b,    "Train B locked — both tracking!",(0,220,50)),
    ]
    for t,msg,col in flashes:
        if now-t<2.5:
            fy=h//2-28
            cv2.rectangle(display,(0,fy),(w,fy+50),(15,15,15),-1)
            cv2.rectangle(display,(0,fy),(w,fy+50),col,3)
            cv2.putText(display,f"  [OK]  {msg}",
                        (20,fy+34),cv2.FONT_HERSHEY_SIMPLEX,0.82,col,2)
            break

    # ══════════════════════════════════════════════════════════
    #  DASHBOARD  —  bottom 100px
    # ══════════════════════════════════════════════════════════
    dash_y = h - 100
    cv2.rectangle(display, (0, dash_y), (w, h), (18, 18, 18), -1)
    cv2.line(display, (0, dash_y), (w, dash_y), (50, 50, 50), 1)

    # ── Train A speed block (left) ────────────────────────────
    col_a   = (0, 165, 255)
    ble_a_c = (0, 200, 60) if ble.connected_a else (60, 60, 200)
    cv2.rectangle(display, (0, dash_y), (170, h), (24, 24, 24), -1)
    cv2.line(display, (170, dash_y), (170, h), (50, 50, 50), 1)
    # BLE dot
    cv2.circle(display, (14, dash_y + 14), 5, ble_a_c, -1)
    cv2.putText(display, "TRAIN  A",
                (24, dash_y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, col_a, 1)
    # Large speed number
    spd_a_col = (0, 60, 200) if zone == Zone.DANGER else col_a
    cv2.putText(display, str(spd_a),
                (18, dash_y + 72),
                cv2.FONT_HERSHEY_SIMPLEX, 2.2, spd_a_col, 3)
    cv2.putText(display, "/ 7",
                (90, dash_y + 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 80), 1)
    cv2.putText(display, "[  faster    ]  slower",
                (8, dash_y + 92),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, (90, 90, 90), 1)

    # ── Train B speed block (right) ───────────────────────────
    col_b   = (0, 220, 50)
    ble_b_c = (0, 200, 60) if ble.connected_b else (60, 60, 200)
    cv2.rectangle(display, (w - 170, dash_y), (w, h), (24, 24, 24), -1)
    cv2.line(display, (w - 170, dash_y), (w - 170, h), (50, 50, 50), 1)
    cv2.circle(display, (w - 14, dash_y + 14), 5, ble_b_c, -1)
    cv2.putText(display, "TRAIN  B",
                (w - 150, dash_y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, col_b, 1)
    spd_b_col = (0, 60, 200) if zone == Zone.DANGER else col_b
    cv2.putText(display, str(spd_b),
                (w - 152, dash_y + 72),
                cv2.FONT_HERSHEY_SIMPLEX, 2.2, spd_b_col, 3)
    cv2.putText(display, "/ 7",
                (w - 80, dash_y + 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 80), 1)
    cv2.putText(display, "1-7 keys  /  Arrow Up  Down",
                (w - 168, dash_y + 92),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, (90, 90, 90), 1)

    # ── Mode badge (centre top) ───────────────────────────────
    mode_lbl = "MANUAL" if manual_mode else "AUTO"
    mode_col = (0, 200, 255) if manual_mode else (0, 180, 60)
    mx = w // 2
    cv2.rectangle(display, (mx - 42, dash_y + 4),
                  (mx + 42, dash_y + 24), mode_col, -1)
    cv2.putText(display, mode_lbl,
                (mx - 34, dash_y + 19),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 2)

    # ── Command buttons (centre panel) ────────────────────────
    # Helper: draw one key button, return next x
    def _btn(x, y, key, label, kc=(55, 55, 55), lc=(170, 170, 170)):
        kw = max(20, len(key) * 7 + 8)
        lw = len(label) * 6 + 6
        bh = 20
        # key box
        cv2.rectangle(display, (x, y), (x + kw, y + bh), kc, -1)
        cv2.rectangle(display, (x, y), (x + kw, y + bh), (80, 80, 80), 1)
        cv2.putText(display, key, (x + 4, y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (230, 230, 230), 1)
        # label box
        cv2.rectangle(display, (x + kw, y), (x + kw + lw, y + bh),
                      (30, 30, 30), -1)
        cv2.rectangle(display, (x + kw, y), (x + kw + lw, y + bh),
                      (55, 55, 55), 1)
        cv2.putText(display, label, (x + kw + 3, y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, lc, 1)
        return x + kw + lw + 5

    # Row 1 — top row of buttons
    r1y = dash_y + 32
    x = 178
    x = _btn(x, r1y, "M",   "Mode",     (0, 120, 160))
    x = _btn(x, r1y, "E",   "Stop Both",(140, 0, 0),  (255, 120, 120))
    x = _btn(x, r1y, "S",   "Stop B",   (120, 0, 0),  (200, 100, 100))
    x = _btn(x, r1y, "R",   "Resume B", (0, 100, 0),  (100, 220, 100))
    x = _btn(x, r1y, "A",   "Resel A",  (0, 80, 140))
    x = _btn(x, r1y, "B",   "Resel B",  (0, 100, 40))

    # Row 2 — bottom row of buttons
    r2y = dash_y + 58
    x = 178
    x = _btn(x, r2y, "[",   "Slow A",   (40, 80, 120))
    x = _btn(x, r2y, "]",   "Fast A",   (40, 80, 120))
    x = _btn(x, r2y, "1-7", "Set B Spd",(60, 60, 60))
    x = _btn(x, r2y, "^v",  "B Speed",  (60, 60, 60))
    x = _btn(x, r2y, "T",   "Table",    (60, 60, 60))
    x = _btn(x, r2y, "H",   "Horn",     (60, 60, 60))
    x = _btn(x, r2y, "K",   "Path",     (60, 60, 60))
    x = _btn(x, r2y, "Q",   "Quit",     (80, 30, 30), (200, 130, 130))


# ── Main ──────────────────────────────────────────────────────────

def main():
    global _tracker

    print("\n+------------------------------------------------------+")
    print("|  LionChief -- Dual BLE Safe Distance v1.0           |")
    print("|  Datix AI  |  June 2026                             |")
    print("+------------------------------------------------------+")
    print("|  Train A (front): [ = slower   ] = faster           |")
    print("|  Train B (rear) : 1-7 / arrows / + -                |")
    print("|  E = Emergency stop BOTH    M = Auto/Manual          |")
    print("+------------------------------------------------------+\n")

    load_calibration()

    ble = DualBLEController()
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
    logger.info("Camera ready")

    _tracker = DragTracker()
    ctrl     = SpeedController()

    cv2.namedWindow(config.WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(config.WINDOW_TITLE, mouse_callback)

    manual_mode     = False
    waiting_confirm = False
    auto_paused     = False
    last_danger_t   = 0.0    # cooldown — DANGER stop sent at most once/second
    resume_time     = 0.0    # grace period — DANGER suppressed 3s after resume
    DANGER_COOLDOWN = 1.0    # seconds between emergency stops
    RESUME_GRACE    = 3.0    # seconds to suppress DANGER after resume
    prev_state      = _tracker.state
    horn_on         = False
    last_ka         = time.time()
    start_time      = time.time()
    frame_count     = 0

    # Track speeds for display
    spd_a = config.DEFAULT_SPEED_A
    spd_b = config.DEFAULT_SPEED_B

    while True:
        ret, raw = cap.read()
        if not ret:
            time.sleep(0.02); continue

        frame_count += 1
        now = time.time()

        display = cv2.resize(raw,(config.DISPLAY_W,config.DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)
        _tracker.set_display_frame(display)
        pos_a, pos_b = _tracker.update(display)

        if (_tracker.state==TRACKING and
                prev_state==WAIT_B and not waiting_confirm):
            waiting_confirm = True
        prev_state = _tracker.state

        a_chasing = _tracker.a_is_chasing_b(pos_a,pos_b)
        dist      = _tracker.facing_gap(pos_a,pos_b,a_chasing)

        # Speed control
        if not waiting_confirm and _tracker.ready:
            a_miss = _tracker.is_a_missing()
            b_miss = _tracker.is_b_missing()

            # Auto-pause if tracker lost
            if a_miss or b_miss:
                if not auto_paused:
                    auto_paused = True
                    ble.train_b.send_stop_raw()
                    logger.warning("AUTO-PAUSED — tracker lost")
            elif auto_paused:
                auto_paused  = False
                resume_time  = now    # start grace period — suppress DANGER for 3s
                ctrl.reset()          # reset PD state so stale gap_rate doesn't trigger
                logger.info("AUTO-RESUMED — 3s DANGER grace period started")

            if manual_mode or auto_paused:
                spd_b = ble.train_b.current_speed
            else:
                eff = None if (a_miss or b_miss) else dist
                speed_b, zone = ctrl.update(eff, a_chasing)

                if zone == Zone.DANGER:
                    # BOTH trains stop — rate limited to 1x per second
                    # and suppressed for RESUME_GRACE seconds after resume
                    in_grace = (now - resume_time) < RESUME_GRACE
                    if not in_grace and (now - last_danger_t) >= DANGER_COOLDOWN:
                        ble.emergency_stop_both()
                        last_danger_t = now
                    spd_a = 0
                    spd_b = 0

                elif zone == Zone.ESCAPE:
                    # Speed up B, slow A by 1 step — without touching user_speed
                    if ctrl.should_send_command(speed_b):
                        ble.train_b.set_speed(speed_b)
                        ctrl.command_sent(speed_b)
                    # Use set_speed_no_save so user_speed is NOT changed
                    # (prevents runaway slowdown of Train A)
                    desired_a = max(1, ble.train_a.user_speed
                                    - config.ESCAPE_SLOW_A_BY)
                    if ble.train_a.current_speed != desired_a:
                        ble.train_a.set_speed_no_save(desired_a)
                    spd_a = desired_a
                    spd_b = speed_b

                else:
                    # Normal: Train A runs freely, Train B gap-controlled
                    if ctrl.should_send_command(speed_b):
                        ble.train_b.set_speed(speed_b)
                        ctrl.command_sent(speed_b)
                    if ble.train_a.current_speed != ble.train_a.user_speed:
                        ble.train_a.set_speed(ble.train_a.user_speed)
                    spd_a = ble.train_a.current_speed
                    spd_b = speed_b
        else:
            zone  = Zone.UNKNOWN
            spd_a = ble.train_a.current_speed
            spd_b = ble.train_b.current_speed

        # Keepalive
        if now - last_ka >= config.KEEPALIVE_INTERVAL:
            last_ka = now
            ble.keepalive()

        conf_a = _tracker.confidence_a()
        conf_b = _tracker.confidence_b()

        draw_overlay(display,_tracker,pos_a,pos_b,dist,zone,
                     spd_a,spd_b,ble,manual_mode or auto_paused,
                     a_chasing,conf_a,conf_b,ctrl)

        if waiting_confirm:
            draw_confirm_overlay(display)

        if auto_paused and not waiting_confirm:
            h2,w2=display.shape[:2]
            cv2.rectangle(display,(0,h2//2-22),(w2,h2//2+22),(0,0,120),-1)
            cv2.putText(display,"AUTO-PAUSED -- tracker lost, re-select A or B",
                        (20,h2//2+8),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,80,255),2)

        cv2.imshow(config.WINDOW_TITLE,display)

        raw_key = cv2.waitKey(1)
        if raw_key == -1:
            continue

        if waiting_confirm:
            k = raw_key & 0xFF
            if k in (ord('y'),ord('Y')):
                waiting_confirm = False; manual_mode = False
                ble.train_a.set_speed(config.DEFAULT_SPEED_A)
                logger.info("AUTO mode — Train A running, Train B gap-controlled")
            elif k in (ord('n'),ord('N')):
                waiting_confirm = False; manual_mode = True
                logger.info("MANUAL mode")
            continue

        key = raw_key & 0xFF

        if key in (ord('q'),ord('Q'),27):
            break

        # ── Train A speed controls ─────────────────────────────────
        elif key == ord('['):
            ble.train_a.set_speed(max(1, ble.train_a.user_speed - 1))
            logger.info(f"Train A speed down -> {ble.train_a.user_speed}")

        elif key == ord(']'):
            ble.train_a.set_speed(min(7, ble.train_a.user_speed + 1))
            logger.info(f"Train A speed up -> {ble.train_a.user_speed}")

        # F1 = stop Train A, F2 = resume Train A
        elif raw_key == 7340032:  # F1
            ble.train_a.send_stop()
            logger.info("Train A stopped (F1)")
        elif raw_key == 7405568:  # F2
            ble.train_a.resume()
            logger.info(f"Train A resumed at {ble.train_a.user_speed}")

        # ── Train B speed controls ─────────────────────────────────
        elif key in (ord('1'),ord('2'),ord('3'),ord('4'),
                     ord('5'),ord('6'),ord('7')):
            ctrl.set_user_speed(int(chr(key)))
            ble.train_b.set_speed(ctrl.user_speed)
            logger.info(f"Train B speed -> {ctrl.user_speed}")

        elif key in (ord('+'),ord('=')):
            ctrl.set_user_speed(ctrl.user_speed+1)
            ble.train_b.set_speed(ctrl.user_speed)

        elif key in (ord('-'),ord('_')):
            ctrl.set_user_speed(max(1,ctrl.user_speed-1))
            ble.train_b.set_speed(ctrl.user_speed)

        elif raw_key in (2490368,65362):  # Up arrow
            ctrl.set_user_speed(ctrl.user_speed+1)
            ble.train_b.set_speed(ctrl.user_speed)
            logger.info(f"Train B up -> {ctrl.user_speed}")

        elif raw_key in (2621440,65364):  # Down arrow
            ctrl.set_user_speed(max(1,ctrl.user_speed-1))
            ble.train_b.set_speed(ctrl.user_speed)
            logger.info(f"Train B down -> {ctrl.user_speed}")

        # ── Shared controls ────────────────────────────────────────
        elif key in (ord('e'),ord('E')):
            ble.emergency_stop_both()
            ctrl.reset()
            logger.warning("EMERGENCY STOP -- both trains")

        elif key in (ord('s'),ord('S')):
            ble.train_b.send_stop(); ctrl.reset()

        elif key in (ord('r'),ord('R')):
            ble.train_b.set_speed(ctrl.user_speed); ctrl.reset()

        elif key in (ord('m'),ord('M'),ord('p'),ord('P')):
            manual_mode = not manual_mode
            ctrl.reset()
            if not manual_mode:
                # Resuming auto — set Train A to its user speed
                ble.train_a.set_speed(ble.train_a.user_speed)
            logger.info(f"{'MANUAL' if manual_mode else 'AUTO'} mode")

        elif key in (ord('a'),ord('A')):
            _tracker.reselect_a()
        elif key in (ord('b'),ord('B')):
            _tracker.reselect_b()
        elif key in (ord('t'),ord('T')):
            _tracker.redraw_table()

        elif key in (ord('k'),ord('K')):  # K for path (L conflicts with lights)
            tp = _tracker.track_path
            if tp.recording:
                tp.stop_recording()
            else:
                tp.start_recording()

        elif key in (ord('h'),ord('H')):
            horn_on = not horn_on
            if horn_on:
                ble.train_a.horn_on(); ble.train_b.horn_on()
            else:
                ble.train_a.horn_off(); ble.train_b.horn_off()

        elif key in (ord('l'),ord('L')):
            ble.train_a.lights_on(); ble.train_b.lights_on()

    ble.shutdown(); cap.release(); cv2.destroyAllWindows()
    elapsed = int(time.time()-start_time)
    print(f"\n  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Frames:{frame_count}\n")


if __name__ == "__main__":
    main()