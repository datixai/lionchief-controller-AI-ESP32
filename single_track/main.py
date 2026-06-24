# ══════════════════════════════════════════════════════════════════
#  main.py  —  Single Track Safe Distance v6.0
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  SELECTION: drag 4 boxes in order:
#    A-HEAD → A-TAIL → B-HEAD → B-TAIL
#  Then system tracks and controls automatically.
#
#  KEYS:
#    A        Re-select Train A (both boxes)
#    B        Re-select Train B (both boxes)
#    1-7      Set speed
#    S/R/P    Stop / Resume / Pause
#    H/L      Horn / Lights
#    Q        Quit
# ══════════════════════════════════════════════════════════════════

import cv2
import time
import logging
import sys
import json
import os

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

# Box display config  (label, color)
BOX_STYLE = {
    "AH": ("A-HEAD", (0, 165, 255)),
    "AT": ("A-TAIL", (0, 100, 180)),
    "BH": ("B-HEAD", (0, 220,  50)),
    "BT": ("B-TAIL", (0, 140,  30)),
}


def load_calibration():
    cal = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       config.CALIBRATION_FILE)
    if not os.path.exists(cal):
        return
    try:
        with open(cal) as f:
            data = json.load(f)
        d = data.get("distances", {})
        if d.get("danger_px"):  config.DISTANCE_DANGER  = d["danger_px"]
        if d.get("warning_px"): config.DISTANCE_WARNING = d["warning_px"]
        if d.get("caution_px"): config.DISTANCE_CAUTION = d["caution_px"]
        if d.get("safe_px"):    config.DISTANCE_SAFE    = d["safe_px"]
        if d.get("far_px"):     config.DISTANCE_FAR     = d["far_px"]
        logger.info("Calibration loaded")
    except Exception as e:
        logger.warning(f"Calibration load: {e}")


_tracker = None

def mouse_callback(event, x, y, flags, param):
    if _tracker is None: return
    if   event == cv2.EVENT_LBUTTONDOWN: _tracker.on_mouse_down(x, y)
    elif event == cv2.EVENT_MOUSEMOVE:   _tracker.on_mouse_move(x, y)
    elif event == cv2.EVENT_LBUTTONUP:   _tracker.on_mouse_up(x, y)


# ── Overlay ───────────────────────────────────────────────────────

def draw_overlay(display, tracker, pos_a, pos_b,
                 dist, zone, speed, user_speed, ble_ok, paused,
                 a_chasing):
    h, w = display.shape[:2]
    now  = time.time()
    zcol = config.ZONE_COLORS.get(zone, (120, 120, 120))

    # ── 4 box trackers with search circles ───────────────────────
    boxes = tracker.get_boxes()
    for key, (lbl, col) in BOX_STYLE.items():
        box = boxes[key]
        if not box.initialized:
            continue
        pos = box.get_pos()
        if pos is None:
            continue

        # Search circle (subtle)
        box.draw_search_circle(display, (60, 60, 60))

        # Box dot and label
        cv2.circle(display, pos, 8, col, -1)
        cv2.putText(display, lbl, (pos[0]+10, pos[1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)

    # ── Train A body line (head to tail) ─────────────────────────
    if pos_a and pos_a.head and pos_a.tail:
        cv2.line(display, pos_a.head, pos_a.tail, (0, 165, 255), 3)
        cv2.putText(display, "TRAIN A",
                    (pos_a.x+8, pos_a.y-14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 165, 255), 2)

    # ── Train B body line (head to tail) ─────────────────────────
    if pos_b and pos_b.head and pos_b.tail:
        cv2.line(display, pos_b.head, pos_b.tail, (0, 220, 50), 3)
        cv2.putText(display, f"TRAIN B  spd:{speed}",
                    (pos_b.x+8, pos_b.y-14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 220, 50), 2)

    # ── Facing-edge gap line ──────────────────────────────────────
    if pos_a and pos_b and dist:
        if a_chasing:
            p1, p2 = pos_a.head, pos_b.tail
        else:
            p1, p2 = pos_b.head, pos_a.tail
        if p1 and p2:
            cv2.line(display, p1, p2, zcol, 2)
            mx = (p1[0] + p2[0]) // 2
            my = (p1[1] + p2[1]) // 2
            cv2.putText(display, f"{dist:.0f}px | {zone}",
                        (mx+6, my-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, zcol, 2)

    # ── Live drag rect ────────────────────────────────────────────
    if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
        x1 = min(tracker.drag_start[0], tracker.drag_end[0])
        y1 = min(tracker.drag_start[1], tracker.drag_end[1])
        x2 = max(tracker.drag_start[0], tracker.drag_end[0])
        y2 = max(tracker.drag_start[1], tracker.drag_end[1])
        drag_colors = {
            WAIT_A_HEAD: (0, 165, 255), WAIT_A_TAIL: (0, 100, 180),
            WAIT_B_HEAD: (0, 220,  50), WAIT_B_TAIL: (0, 140,  30),
        }
        dcol = drag_colors.get(tracker.state, (200, 200, 200))
        cv2.rectangle(display, (x1, y1), (x2, y2), dcol, 2)

    # ── Top bar ───────────────────────────────────────────────────
    if tracker.state in SELECTION_STATES:
        step_colors = {
            WAIT_A_HEAD: (0, 165, 255), WAIT_A_TAIL: (0, 100, 180),
            WAIT_B_HEAD: (0, 220,  50), WAIT_B_TAIL: (0, 140,  30),
        }
        col = step_colors.get(tracker.state, (200, 200, 200))
        cv2.rectangle(display, (0, 0), (w, 52), (30, 20, 0), -1)
        cv2.putText(display, tracker.instruction_text(),
                    (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, col, 2)
        cv2.putText(display,
                    "Draw box around that end of the train — "
                    "HEAD = front end facing travel direction",
                    (8, 46), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (170, 170, 170), 1)
    else:
        if zone == Zone.ESCAPE:
            barc = (60, 0, 60)
        elif zone == Zone.DANGER:
            barc = (0, 0, 90)
        elif zone == Zone.SAFE:
            barc = (0, 55, 0)
        else:
            barc = (25, 25, 25)
        cv2.rectangle(display, (0, 0), (w, 48), barc, -1)

        if paused:
            msg = "PAUSED — press P to resume"
            mc  = (0, 200, 255)
        elif zone == Zone.ESCAPE:
            msg = f"A IS BEHIND B — Train B escaping (speed {speed})"
            mc  = (200, 80, 200)
        elif not tracker.tracking_a or not tracker.tracking_b:
            msg = "Tracker lost — press A or B then drag new boxes"
            mc  = (0, 80, 255)
        else:
            msgs = {
                Zone.DANGER:  f"STOP — gap too small ({dist:.0f}px)" if dist else "STOP",
                Zone.WARNING: f"WARNING — slowing Train B (speed {speed})",
                Zone.CAUTION: f"CAUTION — speed {speed}",
                Zone.SAFE:    f"SAFE — following at speed {speed}",
                Zone.FAR:     f"FAR — catching up (speed {speed})",
                Zone.UNKNOWN: f"Not visible — holding speed {speed}",
            }
            msg = msgs.get(zone, zone)
            mc  = zcol

        cv2.putText(display, msg, (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(display,
                    f"A:{'OK' if tracker.tracking_a else 'LOST'}  "
                    f"B:{'OK' if tracker.tracking_b else 'LOST'}  |  "
                    f"BLE:{'OK' if ble_ok else 'connecting'}  |  "
                    f"Speed:{speed}/7 usr:{user_speed}  |  "
                    f"Gap:{f'{dist:.0f}px' if dist else '---'}  |  "
                    f"A/B=reselect",
                    (8, 44), cv2.FONT_HERSHEY_SIMPLEX,
                    0.37, (200, 200, 200), 1)

    # ── Flash confirmation ────────────────────────────────────────
    flash_items = [
        (tracker.flash_ah, "A-HEAD saved", (0, 165, 255)),
        (tracker.flash_at, "A-TAIL saved — now drag B-HEAD", (0, 100, 180)),
        (tracker.flash_bh, "B-HEAD saved", (0, 220, 50)),
        (tracker.flash_bt, "B-TAIL saved — ALL SET, tracking started!", (0, 140, 30)),
    ]
    for i, (t, msg, col) in enumerate(flash_items):
        if now - t < 2.0:
            fy = h // 2 - 28
            cv2.rectangle(display, (0, fy), (w, fy+50), (15,15,15), -1)
            cv2.rectangle(display, (0, fy), (w, fy+50), col, 3)
            cv2.putText(display, f"  ✅  {msg}",
                        (20, fy+34), cv2.FONT_HERSHEY_SIMPLEX,
                        0.85, col, 2)
            break  # show only most recent

    # ── Bottom bar ────────────────────────────────────────────────
    cv2.rectangle(display, (0, h-22), (w, h), (20, 20, 20), -1)
    cv2.putText(display,
                "DRAG=select boxes  A/B=reselect train  "
                "1-7=Speed  S=Stop  R=Resume  P=Pause  Q=Quit",
                (8, h-7), cv2.FONT_HERSHEY_SIMPLEX,
                0.34, (130, 130, 130), 1)
    return display


# ── Main ──────────────────────────────────────────────────────────

def main():
    global _tracker

    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Safe Distance v6.0  |  4-Box Tracking  ║")
    print("║  Datix AI  |  June 2026                             ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  Draw 4 boxes in order:                             ║")
    print("║    1. Train A HEAD   2. Train A TAIL                ║")
    print("║    3. Train B HEAD   4. Train B TAIL                ║")
    print("║  HEAD = front end of train (direction of travel)    ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    load_calibration()

    ble = TrainBLEController()
    ble.start()

    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        logger.error(f"Cannot open camera {config.CAMERA_INDEX}")
        ble.shutdown(); sys.exit(1)

    logger.info("Warming up camera...")
    for _ in range(config.CAMERA_WARMUP_FRAMES):
        cap.read()
    logger.info("Camera ready ✅\n")

    _tracker = DragTracker()
    ctrl     = SpeedController()

    cv2.namedWindow(config.WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(config.WINDOW_TITLE, mouse_callback)

    paused     = False
    horn_on    = False
    lights_on  = False
    last_ka    = time.time()
    start_time = time.time()
    frame_count= 0
    stops_sent = 0

    while True:
        ret, raw = cap.read()
        if not ret:
            time.sleep(0.02); continue

        frame_count += 1
        now = time.time()

        display = cv2.resize(raw,
                             (config.DISPLAY_W, config.DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)

        _tracker.set_display_frame(display)
        pos_a, pos_b = _tracker.update(display)

        a_chasing = _tracker.a_is_chasing_b(pos_a, pos_b)
        dist      = _tracker.facing_gap(pos_a, pos_b, a_chasing)

        if not paused and _tracker.ready:
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
            last_ka = now; ble.keepalive()

        draw_overlay(display, _tracker, pos_a, pos_b,
                     dist, zone, speed, ctrl.user_speed,
                     ble.connected, paused, a_chasing)
        cv2.imshow(config.WINDOW_TITLE, display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27): break
        elif key in (ord('a'), ord('A')): _tracker.reselect_a()
        elif key in (ord('b'), ord('B')): _tracker.reselect_b()
        elif key in (ord('p'), ord('P')): paused = not paused; ctrl.reset()
        elif key in (ord('s'), ord('S')): ble.send_stop(); ctrl.reset()
        elif key in (ord('r'), ord('R')): ble.set_speed(ctrl.user_speed); ctrl.reset()
        elif key in (ord('1'),ord('2'),ord('3'),ord('4'),
                     ord('5'),ord('6'),ord('7')):
            ctrl.set_user_speed(int(chr(key)))
        elif key in (ord('+'),ord('=')): ctrl.set_user_speed(ctrl.user_speed+1)
        elif key in (ord('-'),ord('_')): ctrl.set_user_speed(max(1,ctrl.user_speed-1))
        elif key in (ord('h'),ord('H')):
            horn_on = not horn_on
            ble.horn_on() if horn_on else ble.horn_off()
        elif key in (ord('l'),ord('L')):
            lights_on = not lights_on
            ble.lights_on() if lights_on else ble.lights_off()

    ble.shutdown(); cap.release(); cv2.destroyAllWindows()
    elapsed = int(time.time()-start_time)
    print(f"\n  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Frames:{frame_count}  Stops:{stops_sent}\n")


if __name__ == "__main__":
    main()