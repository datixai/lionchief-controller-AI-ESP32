# ══════════════════════════════════════════════════════════════════
#  main.py  —  Single Track Click-to-Track Safe Distance
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  No stickers. No colors. Just click on each train.
#
#  HOW TO START EVERY SESSION:
#    1. python main.py
#    2. Camera opens — click on Train A (front)
#    3. Click on Train B (rear)
#    4. System tracks both and controls Train B speed automatically
#
#  IF TRACKER DRIFTS:
#    Press A → click on Train A again to re-lock
#    Press B → click on Train B again to re-lock
#
#  KEYBOARD:
#    LEFT CLICK   Assign train (guided — follows A then B)
#    A            Next click re-assigns Train A
#    B            Next click re-assigns Train B
#    1-7          Set Train B cruising speed
#    + / -        Speed up / down
#    S            Manual STOP Train B
#    R            Manual RESUME Train B
#    P            Pause / resume auto control
#    H            Horn    L=Lights
#    Q / ESC      Quit
# ══════════════════════════════════════════════════════════════════

import cv2
import time
import logging
import sys
import json
import os
import numpy as np

import config
from train_detector  import ClickTracker, pixel_distance, AssignMode
from speed_controller import SpeedController, Zone
from ble_controller  import TrainBLEController

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


# ── Load saved distance calibration if available ──────────────────
def load_calibration():
    cal_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), config.CALIBRATION_FILE)
    if not os.path.exists(cal_file):
        return
    try:
        with open(cal_file) as f:
            data = json.load(f)
        d = data.get("distances", {})
        if d.get("danger_px"):  config.DISTANCE_DANGER  = d["danger_px"]
        if d.get("warning_px"): config.DISTANCE_WARNING = d["warning_px"]
        if d.get("caution_px"): config.DISTANCE_CAUTION = d["caution_px"]
        if d.get("safe_px"):    config.DISTANCE_SAFE    = d["safe_px"]
        if d.get("far_px"):     config.DISTANCE_FAR     = d["far_px"]
        logger.info(
            f"Calibration loaded — Danger:{config.DISTANCE_DANGER}  "
            f"Warning:{config.DISTANCE_WARNING}  "
            f"Safe:{config.DISTANCE_SAFE}")
    except Exception as e:
        logger.warning(f"Could not load calibration: {e}")


# ── Globals shared with mouse callback ────────────────────────────
_tracker   = None
_cur_frame = None


def mouse_callback(event, x, y, flags, param):
    global _tracker, _cur_frame
    if event == cv2.EVENT_LBUTTONDOWN and _tracker and _cur_frame is not None:
        assigned = _tracker.handle_click(x, y)
        if assigned in ("A", "B"):
            logger.info(f"Train {assigned} clicked at ({x},{y})")


# ── Banner ────────────────────────────────────────────────────────
def print_banner():
    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Click-to-Track Safe Distance  v2.0     ║")
    print("║  Datix AI  |  Ahmed Ali  |  June 2026               ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  No stickers needed — just click on each train      ║")
    print("║  Train A = front (runs freely)                      ║")
    print(f"║  Train B = rear  (BLE: {config.TRAIN_B_MAC})   ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  CLICK = assign train   A/B = re-assign             ║")
    print("║  1-7=Speed  S=Stop  R=Resume  P=Pause  Q=Quit      ║")
    print("╚══════════════════════════════════════════════════════╝\n")


# ── Draw overlay ──────────────────────────────────────────────────
def annotate(frame, tracker, pos_a, pos_b, dist, zone,
             cmd_speed, user_speed, ble_ok, paused):
    h, w = frame.shape[:2]

    # ── Train boxes and labels ────────────────────────────────────
    if pos_a:
        if pos_a.bbox:
            bx,by,bw,bh = pos_a.bbox
            col = (0,165,255) if tracker.tracking_a else (0,0,200)
            cv2.rectangle(frame,(bx,by),(bx+bw,by+bh), col, 2)
        cv2.circle(frame,(pos_a.x,pos_a.y), 5, (0,165,255), -1)
        cv2.putText(frame,"A (front)",
                    (pos_a.x+12, pos_a.y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,165,255), 2)

    if pos_b:
        if pos_b.bbox:
            bx,by,bw,bh = pos_b.bbox
            col = (0,220,50) if tracker.tracking_b else (0,0,200)
            cv2.rectangle(frame,(bx,by),(bx+bw,by+bh), col, 2)
        cv2.circle(frame,(pos_b.x,pos_b.y), 5, (0,220,50), -1)
        cv2.putText(frame, f"B spd:{cmd_speed}",
                    (pos_b.x+12, pos_b.y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,220,50), 2)

    # ── Distance line ─────────────────────────────────────────────
    if pos_a and pos_b and dist:
        zcol = config.ZONE_COLORS.get(zone,(150,150,150))
        cv2.line(frame,(pos_a.x,pos_a.y),(pos_b.x,pos_b.y), zcol, 2)
        mx = (pos_a.x+pos_b.x)//2
        my = (pos_a.y+pos_b.y)//2
        cv2.putText(frame, f"{dist:.0f}px | {zone}",
                    (mx+6,my-6),cv2.FONT_HERSHEY_SIMPLEX, 0.55, zcol, 2)

    # ── Top status bar ────────────────────────────────────────────
    mode = tracker.mode

    if not tracker.ready:
        # Guided assignment mode
        bar_col = (60, 30, 0)
        cv2.rectangle(frame,(0,0),(w,48), bar_col,-1)
        if mode == AssignMode.A:
            msg = "STEP 1 — Click on TRAIN A  (the front train)"
            mcol= (0, 165, 255)
        else:
            msg = "STEP 2 — Click on TRAIN B  (the rear BLE train)"
            mcol= (0, 220, 50)
        cv2.putText(frame, msg, (8,28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, mcol, 2)
        cv2.putText(frame, "Click directly on the train body in the image",
                    (8,44),cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180,180,180), 1)
    else:
        # Tracking mode
        zcol   = config.ZONE_COLORS.get(zone,(80,80,80))
        bar_c  = (0,0,100) if zone == Zone.DANGER else \
                 (0,60,0) if zone == Zone.SAFE else (25,25,25)
        cv2.rectangle(frame,(0,0),(w,48), bar_c,-1)

        if paused:
            msg  = "⏸  PAUSED — press P to resume"
            mcol = (0,200,255)
        elif not tracker.tracking_a or not tracker.tracking_b:
            msg  = "⚠️  Tracker lost — press A or B then click on that train to re-lock"
            mcol = (0,80,255)
        else:
            msgs = {
                Zone.DANGER:  f"🛑 DANGER — Train B STOPPED",
                Zone.WARNING: f"⚠️  WARNING — slowing to speed {cmd_speed}",
                Zone.CAUTION: f"🔶 CAUTION — speed {cmd_speed}",
                Zone.SAFE:    f"✅  SAFE — following at speed {cmd_speed}",
                Zone.FAR:     f"📶  FAR — catching up speed {cmd_speed}",
                Zone.UNKNOWN: f"❓  UNKNOWN — holding speed {cmd_speed}",
            }
            msg  = msgs.get(zone, zone)
            mcol = zcol

        cv2.putText(frame, msg, (8,28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        cv2.putText(frame,
                    f"A:{'✅' if tracker.tracking_a else '❌'}  "
                    f"B:{'✅' if tracker.tracking_b else '❌'}  |  "
                    f"BLE:{'✅' if ble_ok else '⏳'}  |  "
                    f"Speed:{cmd_speed}/7 (usr:{user_speed})  |  "
                    f"Gap:{f'{dist:.0f}px' if dist else '---'}  |  "
                    f"Press A/B to re-click a train",
                    (8,44),cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200,200,200), 1)

    # ── Bottom bar ────────────────────────────────────────────────
    cv2.rectangle(frame,(0,h-24),(w,h),(20,20,20),-1)
    cv2.putText(frame,
                "CLICK=assign  A/B=re-assign  1-7=Speed  "
                "S=Stop  R=Resume  P=Pause  H=Horn  L=Lights  Q=Quit",
                (8,h-8),cv2.FONT_HERSHEY_SIMPLEX, 0.36, (130,130,130), 1)

    return frame


# ── Main ──────────────────────────────────────────────────────────
def main():
    global _tracker, _cur_frame

    print_banner()
    load_calibration()

    # BLE controller
    logger.info("Starting BLE controller...")
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

    # Click tracker and speed controller
    _tracker = ClickTracker()
    ctrl     = SpeedController()

    # Window + mouse callback
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

    logger.info("Camera open — click on Train A to begin\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        frame_count += 1
        now = time.time()
        _cur_frame = frame.copy()

        # Supply latest frame to tracker (needed for click init)
        _tracker.set_frame(frame)

        # Update trackers
        pos_a, pos_b = _tracker.update(frame)
        dist = pixel_distance(pos_a, pos_b)

        # ── Speed control (only when both trains assigned) ────────
        if not paused and _tracker.ready:
            eff_dist = None if (
                _tracker.is_train_a_missing() or
                _tracker.is_train_b_missing()
            ) else dist

            speed, zone = ctrl.update(eff_dist)

            if ctrl.should_send_command():
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

        # ── Display ───────────────────────────────────────────────
        annotate(frame, _tracker, pos_a, pos_b, dist, zone,
                 speed, ctrl.user_speed, ble.connected, paused)
        cv2.imshow(config.WINDOW_TITLE, frame)

        # ── Keyboard ──────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break

        elif key in (ord('a'), ord('A')):
            _tracker.reassign_a()
            logger.info("Re-assign: click on Train A (front)")

        elif key in (ord('b'), ord('B')):
            _tracker.reassign_b()
            logger.info("Re-assign: click on Train B (rear)")

        elif key in (ord('p'), ord('P')):
            paused = not paused
            ctrl.reset()
            logger.info(f"Auto control {'PAUSED' if paused else 'RESUMED'}")

        elif key in (ord('s'), ord('S')):
            ble.send_stop()
            ctrl.reset()
            logger.info("Manual STOP")

        elif key in (ord('r'), ord('R')):
            ble.set_speed(ctrl.user_speed)
            ctrl.reset()
            logger.info(f"Manual RESUME at speed {ctrl.user_speed}")

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

    # ── Shutdown ──────────────────────────────────────────────────
    logger.info("Shutting down...")
    ble.shutdown()
    cap.release()
    cv2.destroyAllWindows()

    elapsed = int(time.time() - start_time)
    print(f"\n{'═'*54}")
    print(f"  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Frames: {frame_count}  Stops: {stops_sent}")
    print(f"{'═'*54}\n")


if __name__ == "__main__":
    main()
