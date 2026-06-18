# ══════════════════════════════════════════════════════════════════
#  main.py  —  Dual BLE Single Track Safe Distance
#  Harry Locomotive Project 3 BT  |  Datix AI  |  June 2026
#
#  Both trains are Bluetooth-controlled on the outer loop.
#  Camera measures gap → both train speeds adjusted cooperatively.
#
#  HOW TO RUN:
#    Step 1: python calibrate.py  (set sticker colors + distances)
#    Step 2: python main.py
#
#  KEYBOARD (click camera window):
#    1-7    Set BOTH trains' cruising speed
#    A+1-7  Set Train A speed only  (hold A then number)
#    B+1-7  Set Train B speed only  (hold B then number)
#    S      STOP both trains
#    R      Resume both trains
#    P      Pause / resume auto control
#    E      Emergency stop both
#    H      Horn  B=Bell  L=Lights
#    Q/ESC  Quit
# ══════════════════════════════════════════════════════════════════

import cv2
import time
import logging
import sys
import os
import numpy as np

import config
from train_detector  import TrainDetector, pixel_distance
from speed_controller import DualSpeedController, Zone
from ble_controller  import DualTrainController

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


def print_banner():
    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Dual BLE Safe Distance  v1.0           ║")
    print("║  Datix AI  |  Ahmed Ali  |  June 2026               ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  Train A (front): {config.TRAIN_A_MAC:<34}║")
    print(f"║  Train B (rear) : {config.TRAIN_B_MAC:<34}║")
    print(f"║  Camera         : index {config.CAMERA_INDEX:<28}║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  1-7=Speed(both)  S=StopAll  R=ResumeAll  E=Emerg  ║")
    print("║  P=Pause  H=Horn  B=Bell  L=Lights  Q=Quit         ║")
    print("╚══════════════════════════════════════════════════════╝\n")


def annotate(frame, train_a, train_b, distance, zone,
             speed_a, speed_b, ctrl, paused):
    h, w = frame.shape[:2]

    # ── Train markers ─────────────────────────────────────────────
    if train_a:
        cv2.circle(frame, (train_a.x, train_a.y), 14, (0, 165, 255), 2)
        cv2.circle(frame, (train_a.x, train_a.y),  3, (0, 165, 255), -1)
        cv2.putText(frame, f"A spd:{speed_a}",
                    (train_a.x + 16, train_a.y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

    if train_b:
        cv2.circle(frame, (train_b.x, train_b.y), 14, (0, 220, 50), 2)
        cv2.circle(frame, (train_b.x, train_b.y),  3, (0, 220, 50), -1)
        cv2.putText(frame, f"B spd:{speed_b}",
                    (train_b.x + 16, train_b.y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 50), 2)

    # ── Distance line ─────────────────────────────────────────────
    if train_a and train_b and distance:
        zcol = config.ZONE_COLORS.get(zone, (150, 150, 150))
        cv2.line(frame, (train_a.x, train_a.y),
                 (train_b.x, train_b.y), zcol, 2)
        mid_x = (train_a.x + train_b.x) // 2
        mid_y = (train_a.y + train_b.y) // 2
        cv2.putText(frame, f"{distance:.0f}px | {zone}",
                    (mid_x + 6, mid_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, zcol, 2)

    # ── Top status bar ────────────────────────────────────────────
    zcol   = config.ZONE_COLORS.get(zone, (80, 80, 80))
    bar_c  = (0, 0, 100) if zone == Zone.DANGER else \
             (0, 60,  0) if zone == Zone.SAFE   else \
             (30, 30, 30)
    cv2.rectangle(frame, (0, 0), (w, 48), bar_c, -1)

    if paused:
        msg   = "⏸  PAUSED — auto control off"
        mcol  = (0, 200, 255)
    elif zone == Zone.DANGER:
        msg   = f"🛑 DANGER — BOTH TRAINS STOPPED"
        mcol  = (100, 100, 255)
    elif zone == Zone.WARNING:
        msg   = f"⚠️  WARNING — slowing B, nudging A forward"
        mcol  = zcol
    elif zone == Zone.CAUTION:
        msg   = f"🔶 CAUTION — reducing B speed"
        mcol  = zcol
    elif zone == Zone.SAFE:
        msg   = f"✅  SAFE — following at user speed"
        mcol  = zcol
    elif zone == Zone.FAR:
        msg   = f"📶  FAR — B catching up, A slowing slightly"
        mcol  = zcol
    else:
        msg   = f"❓  UNKNOWN — train not visible"
        mcol  = zcol

    cv2.putText(frame, msg, (8, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    # ── BLE status row ────────────────────────────────────────────
    a_col = (0, 200, 0) if ctrl.connected_a else (0, 0, 200)
    b_col = (0, 200, 0) if ctrl.connected_b else (0, 0, 200)
    cv2.putText(frame,
                f"A: {'✅' if ctrl.connected_a else '⏳'} spd={speed_a} usr={ctrl.train_a.user_speed}  |  "
                f"B: {'✅' if ctrl.connected_b else '⏳'} spd={speed_b} usr={ctrl.train_b.user_speed}  |  "
                f"Gap: {f'{distance:.0f}px' if distance else '---'}",
                (8, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (210, 210, 210), 1)

    # ── Bottom bar ────────────────────────────────────────────────
    cv2.rectangle(frame, (0, h - 24), (w, h), (20, 20, 20), -1)
    cv2.putText(frame,
                "1-7=Speed(both)  S=StopAll  R=Resume  E=Emergency  "
                "P=Pause  H=Horn  L=Lights  Q=Quit",
                (8, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (150, 150, 150), 1)

    return frame


def main():
    print_banner()

    # Start dual BLE
    logger.info("Starting dual BLE controller...")
    ble = DualTrainController()
    ble.start()

    # Start train detector
    logger.info("Opening camera and loading calibration...")
    detector = TrainDetector()

    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        logger.error(f"Cannot open camera {config.CAMERA_INDEX}")
        ble.shutdown()
        sys.exit(1)

    ret, _ = cap.read()
    if not ret:
        logger.error("Camera opened but cannot read frames")
        cap.release()
        ble.shutdown()
        sys.exit(1)

    logger.info("Camera ready ✅")

    # Speed controller
    ctrl = DualSpeedController()

    if config.SHOW_VIDEO:
        cv2.namedWindow(config.WINDOW_TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(config.WINDOW_TITLE, 1280, 720)

    # ── State ──────────────────────────────────────────────────
    paused      = False
    horn_on     = False
    lights_on   = False
    last_ka     = time.time()

    start_time  = time.time()
    frame_count = 0
    emergency_count = 0

    # Current displayed speeds
    disp_speed_a = config.DEFAULT_SPEED_A
    disp_speed_b = config.DEFAULT_SPEED_B

    logger.info("System running — monitoring both trains\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        frame_count += 1
        now = time.time()

        # ── Keepalive ─────────────────────────────────────────
        if now - last_ka >= config.KEEPALIVE_INTERVAL:
            last_ka = now
            ble.keepalive()

        # ── Detect trains ─────────────────────────────────────
        train_a, train_b = detector.detect(frame)
        dist = pixel_distance(train_a, train_b)

        # ── Speed control ─────────────────────────────────────
        if not paused:
            # Use None gap if trains not visible long enough
            eff_dist = None if (
                detector.is_train_a_missing() or
                detector.is_train_b_missing()
            ) else dist

            speed_a, speed_b, zone = ctrl.update(eff_dist)

            # Emergency — bypass rate limiting for both
            if ctrl.is_emergency:
                ble.emergency_stop_all()
                emergency_count += 1
            else:
                ble.set_speeds(speed_a, speed_b)

            disp_speed_a = speed_a
            disp_speed_b = speed_b
        else:
            zone = Zone.UNKNOWN

        # ── Display ───────────────────────────────────────────
        if config.SHOW_VIDEO:
            annotate(frame, train_a, train_b, dist, zone,
                     disp_speed_a, disp_speed_b, ble, paused)
            cv2.imshow(config.WINDOW_TITLE, frame)

        # ── Keyboard ──────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break

        elif key in (ord('e'), ord('E')):
            ble.emergency_stop_all()
            ctrl.reset()
            emergency_count += 1
            logger.warning("Manual EMERGENCY STOP — both trains")

        elif key in (ord('s'), ord('S')):
            ble.train_a.send_stop_raw()
            ble.train_b.send_stop_raw()
            ctrl.reset()
            logger.info("Manual STOP — both trains")

        elif key in (ord('r'), ord('R')):
            ble.train_a.set_speed(ctrl.user_speed_a)
            ble.train_b.set_speed(ctrl.user_speed_b)
            ctrl.reset()
            logger.info(f"Manual RESUME — A:{ctrl.user_speed_a} B:{ctrl.user_speed_b}")

        elif key in (ord('p'), ord('P')):
            paused = not paused
            ctrl.reset()
            logger.info(f"Auto control {'PAUSED' if paused else 'RESUMED'}")

        elif key in (ord('1'), ord('2'), ord('3'), ord('4'),
                     ord('5'), ord('6'), ord('7')):
            spd = int(chr(key))
            ctrl.set_user_speeds(spd)
            if paused:
                ble.set_speeds(spd, spd)
            logger.info(f"User speed (both) → {spd}")

        elif key in (ord('+'), ord('=')):
            ctrl.set_user_speeds(min(7, ctrl.user_speed_b + 1))

        elif key in (ord('-'), ord('_')):
            ctrl.set_user_speeds(max(1, ctrl.user_speed_b - 1))

        elif key in (ord('h'), ord('H')):
            horn_on = not horn_on
            ble.train_a.horn_on() if horn_on else ble.train_a.horn_off()
            ble.train_b.horn_on() if horn_on else ble.train_b.horn_off()

        elif key in (ord('l'), ord('L')):
            lights_on = not lights_on
            ble.train_a.lights_on() if lights_on else ble.train_a.lights_off()
            ble.train_b.lights_on() if lights_on else ble.train_b.lights_off()

    # ── Shutdown ──────────────────────────────────────────────
    logger.info("Shutting down...")
    ble.shutdown()
    cap.release()
    cv2.destroyAllWindows()

    elapsed = int(time.time() - start_time)
    fps     = frame_count / max(elapsed, 1)
    print(f"\n{'═'*54}")
    print(f"  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Frames: {frame_count}  FPS: {fps:.1f}  "
          f"Emergency stops: {emergency_count}")
    print(f"{'═'*54}\n")


if __name__ == "__main__":
    main()
