# ══════════════════════════════════════════════════════════════════
#  main.py  —  Single Track Safe Distance
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  Train A = front (manual, no BLE)
#  Train B = rear  (BLE — speed controlled by camera gap)
#
#  HOW TO START:
#    python main.py
#    → Camera opens
#    → HOLD and DRAG a box around Train A (front) → release
#    → HOLD and DRAG a box around Train B (rear)  → release
#    → System controls Train B speed automatically
#
#  IF TRACKER DRIFTS:
#    Press A → drag new box around Train A
#    Press B → drag new box around Train B
#
#  KEYS:
#    DRAG    Select / re-select train
#    A / B   Next drag re-assigns that train
#    1-7     Set Train B cruising speed
#    + / -   Speed up / down
#    S       Stop Train B
#    R       Resume Train B
#    P       Pause / resume auto control
#    H       Horn    L = Lights
#    Q/ESC   Quit
# ══════════════════════════════════════════════════════════════════

import cv2
import time
import logging
import sys
import json
import os

import config
from train_detector   import DragTracker, pixel_distance, WAIT_A, WAIT_B, TRACKING
from speed_controller import SpeedController, Zone
from ble_controller   import TrainBLEController

# ── Logging ───────────────────────────────────────────────────────
handlers = [logging.StreamHandler(sys.stdout)]
if config.LOG_TO_FILE:
    handlers.append(logging.FileHandler(config.LOG_FILE, mode="a"))
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
    handlers=handlers,
)
logger = logging.getLogger("Main")


def load_calibration():
    cal = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), config.CALIBRATION_FILE)
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
        logger.info(f"Calibration loaded — "
                    f"D:{config.DISTANCE_DANGER} W:{config.DISTANCE_WARNING} "
                    f"S:{config.DISTANCE_SAFE}")
    except Exception as e:
        logger.warning(f"Calibration load failed: {e}")


# ── Globals shared with mouse callback ───────────────────────────
_tracker  = None
_frame_w  = config.CAMERA_WIDTH   # actual frame width in pixels
_frame_h  = config.CAMERA_HEIGHT  # actual frame height in pixels


def mouse_callback(event, x, y, flags, param):
    """
    Scale mouse window coordinates → actual frame coordinates.
    Required when window is displayed at different size than camera frame
    (e.g. Windows DPI scaling, manual window resize).
    Without this the tracking box initialises at the wrong position.
    """
    global _tracker, _frame_w, _frame_h
    if _tracker is None:
        return

    # Scale from window display size to actual frame resolution
    try:
        rect = cv2.getWindowImageRect(config.WINDOW_TITLE)
        win_w, win_h = rect[2], rect[3]
        if win_w > 0 and win_h > 0 and _frame_w > 0 and _frame_h > 0:
            x = int(x * _frame_w / win_w)
            y = int(y * _frame_h / win_h)
    except Exception:
        pass  # use raw coords if window rect unavailable

    if event == cv2.EVENT_LBUTTONDOWN:
        _tracker.on_mouse_down(x, y)
    elif event == cv2.EVENT_MOUSEMOVE:
        _tracker.on_mouse_move(x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        _tracker.on_mouse_up(x, y)


# ── Overlay ───────────────────────────────────────────────────────
def draw_overlay(frame, tracker, pos_a, pos_b,
                 dist, zone, speed, user_speed, ble_ok, paused):
    h, w = frame.shape[:2]

    # Train boxes
    for pos, col, lbl in [
        (pos_a, (0, 165, 255), "A  front"),
        (pos_b, (0, 220,  50), f"B  spd:{speed}"),
    ]:
        if pos and pos.bbox:
            bx, by, bw, bh = pos.bbox
            cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), col, 2)
            cv2.circle(frame, (pos.x, pos.y), 5, col, -1)
            cv2.putText(frame, lbl,
                        (pos.x+10, pos.y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, col, 2)

    # Live drag rectangle — coordinates are already in frame space (scaled by mouse_callback)
    if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
        x1 = min(tracker.drag_start[0], tracker.drag_end[0])
        y1 = min(tracker.drag_start[1], tracker.drag_end[1])
        x2 = max(tracker.drag_start[0], tracker.drag_end[0])
        y2 = max(tracker.drag_start[1], tracker.drag_end[1])
        # Clamp to frame bounds
        x1 = max(0, min(x1, w-1)); y1 = max(0, min(y1, h-1))
        x2 = max(0, min(x2, w-1)); y2 = max(0, min(y2, h-1))
        dcol = (0,165,255) if tracker.state==WAIT_A else (0,220,50)
        cv2.rectangle(frame,(x1,y1),(x2,y2), dcol, 2)
        label = "Train A" if tracker.state==WAIT_A else "Train B"
        cv2.putText(frame, label,
                    (x1+4, max(y1-6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, dcol, 2)

    # Distance line
    if pos_a and pos_b and dist:
        zcol = config.ZONE_COLORS.get(zone, (150,150,150))
        cv2.line(frame,(pos_a.x,pos_a.y),(pos_b.x,pos_b.y), zcol, 2)
        mx = (pos_a.x+pos_b.x)//2
        my = (pos_a.y+pos_b.y)//2
        cv2.putText(frame, f"{dist:.0f}px | {zone}",
                    (mx+6, my-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, zcol, 2)

    # ── Top bar ───────────────────────────────────────────────────
    if tracker.state in (WAIT_A, WAIT_B):
        # Selection mode — big clear instruction
        cv2.rectangle(frame,(0,0),(w,56),(40,25,0),-1)
        if tracker.state == WAIT_A:
            msg  = "STEP 1:  Hold mouse + DRAG a box around  TRAIN A  (front train)"
            mcol = (0, 165, 255)
        else:
            msg  = "STEP 2:  Hold mouse + DRAG a box around  TRAIN B  (rear BLE train)"
            mcol = (0, 220, 50)
        cv2.putText(frame, msg,
                    (8,30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, mcol, 2)
        cv2.putText(frame,
                    "Draw the box around the whole train body — it does not need to be perfect",
                    (8,50), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180,180,180), 1)
    else:
        # Tracking mode
        zcol  = config.ZONE_COLORS.get(zone,(80,80,80))
        barc  = (0,0,90)  if zone==Zone.DANGER else \
                (0,55,0)  if zone==Zone.SAFE   else (25,25,25)
        cv2.rectangle(frame,(0,0),(w,48), barc,-1)

        if paused:
            msg  = "PAUSED — press P to resume"
            mcol = (0,200,255)
        elif not tracker.tracking_a or not tracker.tracking_b:
            msg  = "Tracker lost — press A or B then drag new box around that train"
            mcol = (0,80,255)
        else:
            msgs = {
                Zone.DANGER:  f"STOP — gap too small",
                Zone.WARNING: f"WARNING — slowing Train B to speed {speed}",
                Zone.CAUTION: f"CAUTION — speed {speed}",
                Zone.SAFE:    f"SAFE — following at speed {speed}",
                Zone.FAR:     f"FAR — catching up, speed {speed}",
                Zone.UNKNOWN: f"Train not visible — holding speed {speed}",
            }
            msg  = msgs.get(zone, zone)
            mcol = zcol

        cv2.putText(frame, msg,
                    (8,26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        cv2.putText(frame,
                    f"A:{'OK' if tracker.tracking_a else 'LOST'}  "
                    f"B:{'OK' if tracker.tracking_b else 'LOST'}  |  "
                    f"BLE:{'OK' if ble_ok else 'connecting'}  |  "
                    f"Speed:{speed}/7 (user:{user_speed})  |  "
                    f"Gap:{f'{dist:.0f}px' if dist else '---'}  |  "
                    f"A/B = re-select train",
                    (8,44), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200,200,200), 1)

    # Bottom bar
    cv2.rectangle(frame,(0,h-24),(w,h),(20,20,20),-1)
    cv2.putText(frame,
                "DRAG=select   A/B=reselect   1-7=Speed   "
                "S=Stop   R=Resume   P=Pause   H=Horn   Q=Quit",
                (8,h-8), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (130,130,130), 1)

    # Flash confirmation — shown 2 seconds after each successful selection
    now = time.time()
    flash = []
    if now - tracker.flash_a_time < 2.0:
        flash.append(("Train A box SAVED  -  now drag a box around Train B", (0, 165, 255)))
    if now - tracker.flash_b_time < 2.0:
        flash.append(("Train B box SAVED  -  tracking started!", (0, 220, 50)))
    for i, (msg, col) in enumerate(flash):
        fy = h // 2 - 28 + i * 58
        cv2.rectangle(frame, (0, fy), (w, fy + 50), (15, 15, 15), -1)
        cv2.rectangle(frame, (0, fy), (w, fy + 50), col, 3)
        cv2.putText(frame, msg,
                    (30, fy + 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.95, col, 2)

    return frame


# ── Main ──────────────────────────────────────────────────────────
def main():
    global _tracker

    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Safe Distance Control  v3.0            ║")
    print("║  Datix AI  |  June 2026                             ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  Camera opens → DRAG box around Train A             ║")
    print("║                → DRAG box around Train B            ║")
    print("║  System controls Train B speed automatically        ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    load_calibration()

    # BLE
    ble = TrainBLEController()
    ble.start()

    # Camera
    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        logger.error(f"Cannot open camera {config.CAMERA_INDEX}")
        ble.shutdown()
        sys.exit(1)

    _tracker = DragTracker()
    ctrl     = SpeedController()

    # Read first frame to get actual camera resolution
    ret, first = cap.read()
    if ret:
        _frame_h, _frame_w = first.shape[:2]
        logger.info(f"Camera frame size: {_frame_w}x{_frame_h}")

    cv2.namedWindow(config.WINDOW_TITLE, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(config.WINDOW_TITLE, 1280, 720)
    cv2.setMouseCallback(config.WINDOW_TITLE, mouse_callback)

    paused      = False
    horn_on     = False
    lights_on   = False
    last_ka     = time.time()
    start_time  = time.time()
    frame_count = 0
    stops_sent  = 0

    logger.info("Ready — drag a box around Train A to begin\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        frame_count += 1
        now = time.time()

        _tracker.set_frame(frame)
        pos_a, pos_b = _tracker.update(frame)
        dist = pixel_distance(pos_a, pos_b)

        # Speed control — only when both trains selected
        if not paused and _tracker.ready:
            eff_dist = None if (
                _tracker.is_a_missing() or _tracker.is_b_missing()
            ) else dist

            speed, zone = ctrl.update(eff_dist)

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

        # Draw and show
        draw_overlay(frame, _tracker, pos_a, pos_b,
                     dist, zone, speed, ctrl.user_speed,
                     ble.connected, paused)
        cv2.imshow(config.WINDOW_TITLE, frame)

        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('a'), ord('A')):
            _tracker.reselect_a()
            logger.info("Re-select: drag box around Train A")
        elif key in (ord('b'), ord('B')):
            _tracker.reselect_b()
            logger.info("Re-select: drag box around Train B")
        elif key in (ord('p'), ord('P')):
            paused = not paused
            ctrl.reset()
        elif key in (ord('s'), ord('S')):
            ble.send_stop(); ctrl.reset()
        elif key in (ord('r'), ord('R')):
            ble.set_speed(ctrl.user_speed); ctrl.reset()
        elif key in (ord('1'),ord('2'),ord('3'),ord('4'),
                     ord('5'),ord('6'),ord('7')):
            ctrl.set_user_speed(int(chr(key)))
        elif key in (ord('+'), ord('=')):
            ctrl.set_user_speed(ctrl.user_speed + 1)
        elif key in (ord('-'), ord('_')):
            ctrl.set_user_speed(max(1, ctrl.user_speed - 1))
        elif key in (ord('h'), ord('H')):
            horn_on = not horn_on
            ble.horn_on() if horn_on else ble.horn_off()
        elif key in (ord('l'), ord('L')):
            lights_on = not lights_on
            ble.lights_on() if lights_on else ble.lights_off()

    # Shutdown
    ble.shutdown()
    cap.release()
    cv2.destroyAllWindows()

    elapsed = int(time.time() - start_time)
    print(f"\n  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Frames:{frame_count}  Stops:{stops_sent}\n")


if __name__ == "__main__":
    main()