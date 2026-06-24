# ══════════════════════════════════════════════════════════════════
#  main.py  —  Single Track Safe Distance v7.0
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  NEW FEATURES:
#    1. TABLE BOUNDARY — draw polygon around table on first run.
#       Motion outside = completely ignored. T key to redraw.
#
#    2. Y / N CONFIRMATION — after all 4 boxes selected, asks:
#       "Start Train B now?  Y = auto control  N = manual"
#
#    3. MANUAL / AUTO MODE — M key toggles at any time.
#       Manual: full speed control with arrow keys or 1-7.
#       Auto:   gap detection controls speed automatically.
#
#  SPEED CONTROLS:
#    ↑ Arrow or +    Speed up Train B
#    ↓ Arrow or -    Speed down Train B
#    1-7             Set exact speed
#    M               Toggle Manual / Auto mode
#
#  OTHER KEYS:
#    A / B           Re-select that train (both boxes)
#    S               Stop Train B
#    R               Resume Train B at current user speed
#    P               Pause auto control (same as Manual)
#    H               Horn toggle
#    L               Lights toggle
#    T               Redraw table boundary
#    Q / ESC         Quit
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
                               WAIT_A_HEAD, WAIT_A_TAIL,
                               WAIT_B_HEAD, WAIT_B_TAIL, TRACKING)
from speed_controller import SpeedController, Zone
from ble_controller   import TrainBLEController

handlers = [logging.StreamHandler(sys.stdout)]
if config.LOG_TO_FILE:
    handlers.append(logging.FileHandler(config.LOG_FILE, mode="a"))
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(message)s",
                    datefmt="%H:%M:%S", handlers=handlers)
logger = logging.getLogger("Main")

SELECTION_STATES = (WAIT_A_HEAD, WAIT_A_TAIL, WAIT_B_HEAD, WAIT_B_TAIL)

BOX_STYLE = {
    "AH": ("A-HEAD", (0, 165, 255)),
    "AT": ("A-TAIL", (0, 100, 180)),
    "BH": ("B-HEAD", (0, 220,  50)),
    "BT": ("B-TAIL", (0, 140,  30)),
}

MASK_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         config.TABLE_MASK_FILE)
CAL_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         config.CALIBRATION_FILE)


# ── Calibration loader ────────────────────────────────────────────
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


# ══════════════════════════════════════════════════════════════════
#  TABLE BOUNDARY DRAWING
# ══════════════════════════════════════════════════════════════════

_mask_points  = []
_mask_cursor  = (0, 0)
_mask_closed  = False
_drawing_mask = False


def _mask_mouse(event, x, y, flags, param):
    global _mask_points, _mask_cursor, _mask_closed
    _mask_cursor = (x, y)
    if event == cv2.EVENT_LBUTTONDOWN:
        if not _mask_closed:
            _mask_points.append([x, y])
    elif event == cv2.EVENT_RBUTTONDOWN:
        if _mask_points:
            _mask_points.pop()
            _mask_closed = False


def draw_table_boundary(cap, win: str) -> "np.ndarray | None":
    """
    Interactive polygon drawing for table boundary.
    Returns binary mask (255 inside table) or None if skipped.
    Saves to table_mask.json automatically on save.
    """
    global _mask_points, _mask_cursor, _mask_closed
    _mask_points = []
    _mask_closed = False

    cv2.setMouseCallback(win, _mask_mouse)
    saved_flash = 0

    print("\n  Table boundary drawing:")
    print("  LEFT CLICK  = add polygon point")
    print("  RIGHT CLICK = remove last point")
    print("  ENTER       = close polygon")
    print("  S           = save and continue")
    print("  Q           = skip (no mask)\n")

    while True:
        ret, raw = cap.read()
        if not ret:
            continue
        display = cv2.resize(raw, (config.DISPLAY_W, config.DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)
        h, w = display.shape[:2]

        # Shade outside area if closed
        if _mask_closed and len(_mask_points) >= 3:
            pts = np.array(_mask_points, dtype=np.int32)
            overlay = display.copy()
            cv2.fillPoly(overlay, [pts], (0, 60, 60))
            cv2.addWeighted(overlay, 0.4, display, 0.6, 0, display)
            cv2.polylines(display, [pts], True, (0, 220, 220), 2)
        elif len(_mask_points) >= 2:
            pts_arr = np.array(_mask_points, dtype=np.int32)
            cv2.polylines(display, [pts_arr], False, (0, 220, 220), 2)
            cv2.line(display, tuple(_mask_points[-1]),
                     _mask_cursor, (0, 120, 120), 1)

        for p in _mask_points:
            cv2.circle(display, tuple(p), 5, (0, 220, 220), -1)

        cv2.circle(display, _mask_cursor, 4, (0, 220, 220), -1)

        # Top bar
        cv2.rectangle(display, (0, 0), (w, 52), (20, 20, 20), -1)
        if saved_flash > 0:
            cv2.putText(display, "  ✅ Table boundary SAVED — continuing",
                        (8, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (100, 255, 100), 2)
            saved_flash -= 1
            cv2.imshow(win, display)
            cv2.waitKey(30)
            if saved_flash == 0:
                break
            continue

        if not _mask_closed:
            cv2.putText(display,
                        f"DRAW TABLE BOUNDARY — click points ({len(_mask_points)} placed)"
                        f"  |  ENTER to close polygon",
                        (8, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.58, (0, 220, 220), 2)
            cv2.putText(display,
                        "Right-click = undo last point   Q = skip (no boundary)",
                        (8, 47), cv2.FONT_HERSHEY_SIMPLEX,
                        0.36, (150, 150, 150), 1)
        else:
            cv2.putText(display,
                        f"Polygon closed ({len(_mask_points)} points) — press S to SAVE and continue",
                        (8, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 120), 2)

        cv2.rectangle(display, (0, h-22), (w, h), (20, 20, 20), -1)
        cv2.putText(display,
                    "Click=add point  Right-click=undo  ENTER=close  S=save  Q=skip",
                    (8, h-7), cv2.FONT_HERSHEY_SIMPLEX,
                    0.34, (130, 130, 130), 1)

        cv2.imshow(win, display)
        key = cv2.waitKey(20) & 0xFF

        if key == 13:  # ENTER — close polygon
            if len(_mask_points) >= 3:
                _mask_closed = True
            else:
                print("  Need at least 3 points")

        elif key in (ord('s'), ord('S')) and _mask_closed:
            pts = np.array(_mask_points, dtype=np.int32)
            mask = np.zeros(
                (config.DISPLAY_H, config.DISPLAY_W), dtype=np.uint8)
            cv2.fillPoly(mask, [pts], 255)
            with open(MASK_FILE, 'w') as f:
                json.dump({"points": _mask_points}, f, indent=2)
            print(f"  ✅ Table mask saved — {len(_mask_points)} points")
            saved_flash = 25

        elif key in (ord('q'), ord('Q'), 27):
            print("  Table boundary skipped — full frame will be used")
            return None

    return mask if _mask_closed else None


# ══════════════════════════════════════════════════════════════════
#  OVERLAY DRAWING
# ══════════════════════════════════════════════════════════════════

def draw_overlay(display, tracker, pos_a, pos_b, dist, zone,
                 speed, user_speed, ble_ok, manual_mode, a_chasing):
    h, w = display.shape[:2]
    now  = time.time()
    zcol = config.ZONE_COLORS.get(zone, (120, 120, 120))

    # Table mask tint
    mask = tracker._table_mask
    if mask is not None:
        outside = display.copy()
        outside[mask == 0] = (0, 0, 40)
        cv2.addWeighted(outside, 0.22, display, 0.78, 0, display)

    # 4 box trackers + search circles
    for key, (lbl, col) in BOX_STYLE.items():
        box = tracker.get_boxes()[key]
        if not box.initialized:
            continue
        pos = box.get_pos()
        if pos is None:
            continue
        box.draw_search_circle(display, (50, 50, 50))
        cv2.circle(display, pos, 8, col, -1)
        cv2.putText(display, lbl, (pos[0]+10, pos[1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)

    # Train body lines
    if pos_a and pos_a.head and pos_a.tail:
        cv2.line(display, pos_a.head, pos_a.tail, (0, 165, 255), 3)
        cv2.putText(display, "TRAIN A",
                    (pos_a.x+8, pos_a.y-14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 165, 255), 2)
    if pos_b and pos_b.head and pos_b.tail:
        cv2.line(display, pos_b.head, pos_b.tail, (0, 220, 50), 3)
        cv2.putText(display, f"TRAIN B  spd:{speed}",
                    (pos_b.x+8, pos_b.y-14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 220, 50), 2)

    # Gap line
    if pos_a and pos_b and dist:
        p1 = pos_a.head if a_chasing else pos_b.head
        p2 = pos_b.tail if a_chasing else pos_a.tail
        if p1 and p2:
            cv2.line(display, p1, p2, zcol, 2)
            mx, my = (p1[0]+p2[0])//2, (p1[1]+p2[1])//2
            cv2.putText(display, f"{dist:.0f}px | {zone}",
                        (mx+6, my-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, zcol, 2)

    # Live drag rect
    if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
        x1 = min(tracker.drag_start[0], tracker.drag_end[0])
        y1 = min(tracker.drag_start[1], tracker.drag_end[1])
        x2 = max(tracker.drag_start[0], tracker.drag_end[0])
        y2 = max(tracker.drag_start[1], tracker.drag_end[1])
        step_cols = {WAIT_A_HEAD:(0,165,255), WAIT_A_TAIL:(0,100,180),
                     WAIT_B_HEAD:(0,220,50),  WAIT_B_TAIL:(0,140,30)}
        dcol = step_cols.get(tracker.state, (200, 200, 200))
        cv2.rectangle(display, (x1,y1), (x2,y2), dcol, 2)

    # Top status bar
    if tracker.state in SELECTION_STATES:
        step_cols = {WAIT_A_HEAD:(0,165,255), WAIT_A_TAIL:(0,100,180),
                     WAIT_B_HEAD:(0,220,50),  WAIT_B_TAIL:(0,140,30)}
        col = step_cols.get(tracker.state, (200,200,200))
        cv2.rectangle(display, (0,0), (w,52), (30,20,0), -1)
        cv2.putText(display, tracker.instruction_text(),
                    (8,28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, col, 2)
        cv2.putText(display,
                    "HEAD = front end of train (direction of travel)",
                    (8,46), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (170,170,170), 1)
    else:
        # Status bar color
        if zone == Zone.ESCAPE:       barc = (60, 0, 60)
        elif zone == Zone.DANGER:     barc = (0, 0, 90)
        elif zone == Zone.SAFE:       barc = (0, 55, 0)
        elif manual_mode:             barc = (50, 30, 0)
        else:                         barc = (25, 25, 25)
        cv2.rectangle(display, (0,0), (w,48), barc, -1)

        if manual_mode:
            msg  = f"MANUAL MODE — Train B speed: {speed}/7  (↑↓ or +- to adjust)"
            mcol = (0, 200, 255)
        elif zone == Zone.ESCAPE:
            msg  = f"A IS BEHIND B — Train B escaping (speed {speed})"
            mcol = (200, 80, 200)
        elif not tracker.tracking_a or not tracker.tracking_b:
            msg  = "Tracker lost — press A or B then drag new boxes"
            mcol = (0, 80, 255)
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

        mode_str = "MANUAL" if manual_mode else "AUTO"
        mode_col = (0,200,255) if manual_mode else (100,220,100)
        cv2.putText(display,
                    f"[{mode_str}]  "
                    f"A:{'OK' if tracker.tracking_a else 'LOST'}  "
                    f"B:{'OK' if tracker.tracking_b else 'LOST'}  |  "
                    f"BLE:{'OK' if ble_ok else 'connecting'}  |  "
                    f"Speed:{speed}/7 usr:{user_speed}  |  "
                    f"Gap:{f'{dist:.0f}px' if dist else '---'}  |  "
                    f"M=toggle  T=redraw mask",
                    (8,44), cv2.FONT_HERSHEY_SIMPLEX, 0.36, mode_col, 1)

    # Flash confirmations
    flash_items = [
        (tracker.flash_ah, "A-HEAD saved",                    (0,165,255)),
        (tracker.flash_at, "A-TAIL saved — now drag B-HEAD",  (0,100,180)),
        (tracker.flash_bh, "B-HEAD saved — now drag B-TAIL",  (0,220,50)),
        (tracker.flash_bt, "B-TAIL saved — all boxes done!",  (0,140,30)),
    ]
    for t, msg, col in flash_items:
        if now - t < 2.0:
            fy = h // 2 - 28
            cv2.rectangle(display, (0,fy), (w,fy+50), (15,15,15), -1)
            cv2.rectangle(display, (0,fy), (w,fy+50), col, 3)
            cv2.putText(display, f"  ✅  {msg}",
                        (20,fy+34), cv2.FONT_HERSHEY_SIMPLEX, 0.85, col, 2)
            break

    # Speed panel (right side)
    px = w - 160
    cv2.rectangle(display, (px,h-100), (w,h), (30,30,30), -1)
    cv2.putText(display, "SPEED", (px+50,h-82),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150,150,150), 1)
    spd_col = (0,80,220) if zone==Zone.DANGER else \
              (0,200,255) if manual_mode else (0,200,60)
    cv2.putText(display, str(speed), (px+55,h-38),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0, spd_col, 3)
    cv2.putText(display, "/ 7", (px+105,h-38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120,120,120), 1)
    mode_lbl = "MANUAL" if manual_mode else "AUTO  "
    cv2.putText(display, mode_lbl, (px+30,h-12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (0,200,255) if manual_mode else (100,220,100), 1)

    # Bottom bar
    cv2.rectangle(display, (0,h-22), (w-160,h), (20,20,20), -1)
    cv2.putText(display,
                "DRAG=boxes  A/B=reselect  ↑↓/+-=Speed  M=Auto/Manual  "
                "S=Stop  R=Resume  T=Table  Q=Quit",
                (8,h-7), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (130,130,130), 1)
    return display


# ══════════════════════════════════════════════════════════════════
#  Y/N CONFIRMATION OVERLAY
# ══════════════════════════════════════════════════════════════════

def draw_confirm_overlay(display):
    h, w = display.shape[:2]
    # Dark overlay
    overlay = display.copy()
    cv2.rectangle(overlay, (0,0), (w,h), (0,0,0), -1)
    cv2.addWeighted(overlay, 0.55, display, 0.45, 0, display)

    # Box
    bx, by, bw, bh = w//2-280, h//2-70, 560, 140
    cv2.rectangle(display, (bx,by), (bx+bw,by+bh), (30,30,30), -1)
    cv2.rectangle(display, (bx,by), (bx+bw,by+bh), (0,220,50), 2)

    cv2.putText(display,
                "All 4 boxes set!  Start Train B now?",
                (bx+20, by+38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255), 2)
    cv2.putText(display,
                "Y  =  Auto control (gap detection)",
                (bx+60, by+72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0,220,50), 2)
    cv2.putText(display,
                "N  =  Manual control (you set speed)",
                (bx+60, by+106),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0,200,255), 2)
    return display


# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════

_tracker = None

def mouse_callback(event, x, y, flags, param):
    if _tracker is None: return
    if   event == cv2.EVENT_LBUTTONDOWN: _tracker.on_mouse_down(x, y)
    elif event == cv2.EVENT_MOUSEMOVE:   _tracker.on_mouse_move(x, y)
    elif event == cv2.EVENT_LBUTTONUP:   _tracker.on_mouse_up(x, y)


def main():
    global _tracker

    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Safe Distance v7.0                     ║")
    print("║  Datix AI  |  June 2026                             ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  Step 1: Draw table boundary (or load saved one)    ║")
    print("║  Step 2: Draw 4 boxes on trains                     ║")
    print("║  Step 3: Y = auto  /  N = manual speed control      ║")
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

    # Open window first
    cv2.namedWindow(config.WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)

    # ── STEP 1: TABLE BOUNDARY ────────────────────────────────────
    table_mask = None
    if os.path.exists(MASK_FILE):
        # Load existing mask
        try:
            with open(MASK_FILE) as f:
                data = json.load(f)
            pts = np.array(data["points"], dtype=np.int32)
            table_mask = np.zeros(
                (config.DISPLAY_H, config.DISPLAY_W), dtype=np.uint8)
            cv2.fillPoly(table_mask, [pts], 255)
            logger.info(f"Table mask loaded from file ✅")
            print(f"  Table boundary loaded ({len(pts)} points).")
            print("  Press T at any time to redraw it.\n")
        except Exception as e:
            logger.warning(f"Could not load table mask: {e}")
            table_mask = None

    if table_mask is None:
        print("  No table boundary found — please draw one now.")
        table_mask = draw_table_boundary(cap, config.WINDOW_TITLE)

    # ── STEP 2: TRACKER + BOX SELECTION ──────────────────────────
    _tracker = DragTracker()
    if table_mask is not None:
        _tracker.set_table_mask(table_mask)

    ctrl = SpeedController()
    cv2.setMouseCallback(config.WINDOW_TITLE, mouse_callback)

    manual_mode  = False
    waiting_confirm = False   # True when showing Y/N prompt
    horn_on      = False
    lights_on    = False
    last_ka      = time.time()
    start_time   = time.time()
    frame_count  = 0
    stops_sent   = 0
    prev_state   = None

    logger.info("Draw 4 boxes on the trains to begin\n")

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

        # Detect when all 4 boxes just became ready → show Y/N
        if (_tracker.state == TRACKING and
                prev_state == WAIT_B_TAIL and
                not waiting_confirm):
            waiting_confirm = True
            logger.info("All boxes set — waiting for Y/N confirmation")

        prev_state = _tracker.state

        a_chasing = _tracker.a_is_chasing_b(pos_a, pos_b)
        dist      = _tracker.facing_gap(pos_a, pos_b, a_chasing)

        # Speed control
        if not waiting_confirm and _tracker.ready:
            if manual_mode:
                # Manual: just keepalive, no auto speed change
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

        # Keepalive
        if now - last_ka >= config.KEEPALIVE_INTERVAL:
            last_ka = now
            ble.keepalive()

        # Draw
        draw_overlay(display, _tracker, pos_a, pos_b, dist, zone,
                     speed, ctrl.user_speed, ble.connected,
                     manual_mode, a_chasing)

        # Y/N overlay
        if waiting_confirm:
            draw_confirm_overlay(display)

        cv2.imshow(config.WINDOW_TITLE, display)

        # ── Keyboard ──────────────────────────────────────────────
        key = cv2.waitKey(1)

        if key == -1:
            continue

        # Y/N confirmation handling
        if waiting_confirm:
            if key & 0xFF in (ord('y'), ord('Y')):
                waiting_confirm = False
                manual_mode     = False
                logger.info("AUTO mode started — gap detection controlling Train B")
            elif key & 0xFF in (ord('n'), ord('N')):
                waiting_confirm = False
                manual_mode     = True
                logger.info("MANUAL mode — use ↑↓ or 1-7 to control Train B speed")
            continue  # ignore other keys while confirming

        raw_key = key
        key     = key & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break

        elif key in (ord('t'), ord('T')):
            # Redraw table boundary
            logger.info("Redrawing table boundary...")
            new_mask = draw_table_boundary(cap, config.WINDOW_TITLE)
            if new_mask is not None:
                table_mask = new_mask
                _tracker.set_table_mask(table_mask)
            cv2.setMouseCallback(config.WINDOW_TITLE, mouse_callback)

        elif key in (ord('a'), ord('A')):
            _tracker.reselect_a()
        elif key in (ord('b'), ord('B')):
            _tracker.reselect_b()

        elif key in (ord('m'), ord('M')):
            manual_mode = not manual_mode
            ctrl.reset()
            logger.info(f"{'MANUAL' if manual_mode else 'AUTO'} mode")

        elif key in (ord('s'), ord('S')):
            ble.send_stop(); ctrl.reset()

        elif key in (ord('r'), ord('R')):
            ble.set_speed(ctrl.user_speed); ctrl.reset()

        elif key in (ord('p'), ord('P')):
            manual_mode = not manual_mode
            ctrl.reset()

        elif key in (ord('h'), ord('H')):
            horn_on = not horn_on
            ble.horn_on() if horn_on else ble.horn_off()

        elif key in (ord('l'), ord('L')):
            lights_on = not lights_on
            ble.lights_on() if lights_on else ble.lights_off()

        elif key in (ord('1'),ord('2'),ord('3'),ord('4'),
                     ord('5'),ord('6'),ord('7')):
            spd = int(chr(key))
            ctrl.set_user_speed(spd)
            ble.set_speed(spd)

        elif key in (ord('+'), ord('=')):
            ctrl.set_user_speed(ctrl.user_speed + 1)
            ble.set_speed(ctrl.user_speed)

        elif key in (ord('-'), ord('_')):
            ctrl.set_user_speed(max(1, ctrl.user_speed - 1))
            ble.set_speed(ctrl.user_speed)

        # Arrow keys — Windows: raw key value without & 0xFF
        elif raw_key == 2490368 or raw_key == 65362:  # Up arrow
            ctrl.set_user_speed(ctrl.user_speed + 1)
            ble.set_speed(ctrl.user_speed)
            logger.info(f"Speed ↑ → {ctrl.user_speed}")

        elif raw_key == 2621440 or raw_key == 65364:  # Down arrow
            ctrl.set_user_speed(max(1, ctrl.user_speed - 1))
            ble.set_speed(ctrl.user_speed)
            logger.info(f"Speed ↓ → {ctrl.user_speed}")

    ble.shutdown()
    cap.release()
    cv2.destroyAllWindows()
    elapsed = int(time.time() - start_time)
    print(f"\n  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Frames:{frame_count}  Stops:{stops_sent}\n")


if __name__ == "__main__":
    main()