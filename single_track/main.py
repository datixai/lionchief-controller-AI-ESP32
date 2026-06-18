# ══════════════════════════════════════════════════════════════════
#  main.py  —  Single Track Safe Distance Control
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  Two trains on outer loop track 3:
#    Train A (front)  — runs freely, no BLE
#    Train B (rear)   — BLE controlled, speed adjusted by camera gap
#
#  Camera detects both trains by colored stickers on their roofs.
#  Speed controller adjusts Train B so it never catches Train A.
#
#  HOW TO RUN:
#    Step 1:  python calibrate.py  (set sticker colors + distances)
#    Step 2:  python main.py
#
#  KEYBOARD (click camera window first):
#    1-7    Set Train B cruising speed
#    + / -  Speed up / down
#    S      Manual STOP Train B
#    R      Manual RESUME Train B
#    P      Pause / resume auto control
#    H      Horn   B = Bell   L = Lights
#    Q/ESC  Quit cleanly
# ══════════════════════════════════════════════════════════════════

import cv2
import time
import logging
import sys
import os
import numpy as np

import config
from train_detector  import TrainDetector, pixel_distance
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


# ── Banner ─────────────────────────────────────────────────────────
def print_banner(ble_mac):
    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Single Track Safe Distance  v1.0       ║")
    print("║  Datix AI  |  Ahmed Ali  |  June 2026               ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  Train A (front) : runs freely — no BLE             ║")
    print(f"║  Train B (rear)  : {ble_mac:<33}║")
    print(f"║  Camera          : index {config.CAMERA_INDEX:<28}║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  1-7=Speed  +/-=Adjust  S=Stop  R=Resume  P=Pause   ║")
    print("║  H=Horn  B=Bell  L=Lights  Q=Quit                   ║")
    print("╚══════════════════════════════════════════════════════╝\n")


# ── Annotate frame ────────────────────────────────────────────────
def annotate(frame, train_a, train_b, distance, zone,
             cmd_speed, user_speed, ble_ok, paused):
    h, w = frame.shape[:2]

    # ── Draw train markers ────────────────────────────────────────
    if train_a:
        cv2.circle(frame, (train_a.x, train_a.y), 14,
                   (0, 165, 255), 2)
        cv2.circle(frame, (train_a.x, train_a.y), 3,
                   (0, 165, 255), -1)
        cv2.putText(frame, "A (front)",
                    (train_a.x + 16, train_a.y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

    if train_b:
        cv2.circle(frame, (train_b.x, train_b.y), 14,
                   (0, 220, 50), 2)
        cv2.circle(frame, (train_b.x, train_b.y), 3,
                   (0, 220, 50), -1)
        cv2.putText(frame, f"B (rear) spd:{cmd_speed}",
                    (train_b.x + 16, train_b.y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 50), 2)

    # ── Distance line ─────────────────────────────────────────────
    if train_a and train_b and distance:
        zone_col = config.ZONE_COLORS.get(zone, (150, 150, 150))
        cv2.line(frame, (train_a.x, train_a.y),
                 (train_b.x, train_b.y), zone_col, 2)
        mid_x = (train_a.x + train_b.x) // 2
        mid_y = (train_a.y + train_b.y) // 2
        cv2.putText(frame,
                    f"{distance:.0f}px | {zone}",
                    (mid_x + 6, mid_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, zone_col, 2)

    # ── Top status bar ────────────────────────────────────────────
    zone_col = config.ZONE_COLORS.get(zone, (80, 80, 80))
    bar_col  = (0, 60, 0) if zone == Zone.SAFE else \
               (0, 0, 100) if zone == Zone.DANGER else \
               (30, 30, 30)
    cv2.rectangle(frame, (0, 0), (w, 44), bar_col, -1)

    if paused:
        status = "⏸  PAUSED — auto control off"
        s_col  = (0, 200, 255)
    else:
        status = {
            Zone.DANGER:  f"🛑 DANGER — Train B STOPPED (gap too small)",
            Zone.WARNING: f"⚠️  WARNING — Train B slowing to speed {cmd_speed}",
            Zone.CAUTION: f"🔶 CAUTION — reducing speed to {cmd_speed}",
            Zone.SAFE:    f"✅  SAFE — Train B following at speed {cmd_speed}",
            Zone.FAR:     f"📶  FAR — catching up, speed {cmd_speed}",
            Zone.UNKNOWN: f"❓  UNKNOWN — train not visible, holding speed {cmd_speed}",
        }.get(zone, zone)
        s_col = zone_col

    cv2.putText(frame, status, (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)

    # ── Bottom strip ──────────────────────────────────────────────
    cv2.rectangle(frame, (0, h - 26), (w, h), (20, 20, 20), -1)
    ble_col = (0, 200, 0) if ble_ok else (0, 0, 200)
    cv2.putText(frame,
                f"BLE: {'✅' if ble_ok else '⏳ connecting'}  "
                f"| Speed: {cmd_speed}/7 (user:{user_speed})  "
                f"| Gap: {f'{distance:.0f}px' if distance else '---'}  "
                f"| 1-7=Speed  S=Stop  R=Resume  P=Pause  Q=Quit",
                (8, h - 9),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

    return frame


# ── Main ──────────────────────────────────────────────────────────
def main():
    print_banner(config.TRAIN_B_MAC)

    # Start BLE
    logger.info("Starting BLE controller for Train B...")
    ble = TrainBLEController()
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
    ctrl = SpeedController()

    # Display window
    if config.SHOW_VIDEO:
        cv2.namedWindow(config.WINDOW_TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(config.WINDOW_TITLE, 1280, 720)

    # ── State ──────────────────────────────────────────────────
    paused      = False
    horn_on     = False
    bell_on     = False
    lights_on   = False
    last_ka     = time.time()

    # Statistics
    start_time  = time.time()
    frame_count = 0
    stops_sent  = 0

    logger.info("System running — monitoring both trains\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.warning("Frame read failed — retrying")
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
            # Use MISSING speed if a train is not visible long enough
            if detector.is_train_a_missing() or detector.is_train_b_missing():
                effective_dist = None   # triggers UNKNOWN zone → safe speed
            else:
                effective_dist = dist

            speed, zone = ctrl.update(effective_dist)

            if ctrl.should_send_command():
                if zone == Zone.DANGER and ble.current_speed != 0:
                    stops_sent += 1
                ble.set_speed(speed)
                ctrl.command_sent(speed)
        else:
            speed = ble.current_speed
            zone  = Zone.UNKNOWN

        # ── Annotate and display ──────────────────────────────
        if config.SHOW_VIDEO:
            annotate(frame, train_a, train_b, dist, zone,
                     speed, ctrl.user_speed, ble.connected, paused)
            cv2.imshow(config.WINDOW_TITLE, frame)

        # ── Keyboard ──────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break

        elif key in (ord('p'), ord('P')):
            paused = not paused
            ctrl.reset()
            logger.info(f"Auto control {'PAUSED' if paused else 'RESUMED'}")

        elif key in (ord('s'), ord('S')):
            ble.emergency_stop()
            ctrl.reset()
            logger.info("Manual STOP")

        elif key in (ord('r'), ord('R')):
            ble.set_speed(ctrl.user_speed)
            ctrl.reset()
            logger.info(f"Manual RESUME at speed {ctrl.user_speed}")

        elif key in (ord('1'), ord('2'), ord('3'), ord('4'),
                     ord('5'), ord('6'), ord('7')):
            spd = int(chr(key))
            ctrl.set_user_speed(spd)
            if paused:
                ble.set_speed(spd)

        elif key in (ord('+'), ord('=')):
            ctrl.set_user_speed(ctrl.user_speed + 1)
            logger.info(f"User speed → {ctrl.user_speed}")

        elif key in (ord('-'), ord('_')):
            ctrl.set_user_speed(max(1, ctrl.user_speed - 1))
            logger.info(f"User speed → {ctrl.user_speed}")

        elif key in (ord('h'), ord('H')):
            horn_on = not horn_on
            ble.horn_on() if horn_on else ble.horn_off()

        elif key in (ord('b'), ord('B')):
            bell_on = not bell_on
            ble.bell_on() if bell_on else ble.bell_off()

        elif key in (ord('l'), ord('L')):
            lights_on = not lights_on
            ble.lights_on() if lights_on else ble.lights_off()

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
          f"Stops sent: {stops_sent}")
    print(f"{'═'*54}\n")


if __name__ == "__main__":
    main()
