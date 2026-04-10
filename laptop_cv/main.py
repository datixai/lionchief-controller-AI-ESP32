# ══════════════════════════════════════════════════════════════════
#  main.py  —  LionChief Laptop CV  —  Entry Point
#
#  Collision prevention using laptop + USB webcam only.
#  No ESP32. No Raspberry Pi. Laptop sends BLE directly to train.
#
#  HOW TO RUN:
#    Step 1:  pip install -r requirements.txt
#    Step 2:  python calibrate_zone.py    (once — draws shared zone)
#    Step 3:  python main.py
#
#  KEYBOARD CONTROLS (click camera window first):
#    Q / ESC  = Quit
#    P        = Pause / resume detection
#    S        = Manual STOP train
#    R        = Manual RESUME train
#    H        = Horn
#    B        = Bell toggle
#    L        = Lights toggle
#    A        = Announce / speech
#    + / =    = Speed up
#    -        = Speed down
#    F        = Forward
#    V        = Reverse
# ══════════════════════════════════════════════════════════════════

import cv2
import time
import logging
import sys
import os

import config
from zone_detector  import ZoneDetector
from ble_controller import TrainBLEController

# ── Logging ───────────────────────────────────────────────────────
handlers = [logging.StreamHandler(sys.stdout)]
if config.LOG_TO_FILE:
    handlers.append(logging.FileHandler(config.LOG_FILE_PATH))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
    handlers=handlers
)
logger = logging.getLogger("Main")


def print_banner():
    mac_mode = f"MAC: {config.TRAIN_MAC}" if config.TRAIN_MAC \
               else f"Auto-scan: name prefix '{config.TRAIN_NAME}'"
    print("\n╔══════════════════════════════════════════════════╗")
    print("║   LionChief Collision Prevention — Laptop CV    ║")
    print("╠══════════════════════════════════════════════════╣")
    print(f"║  BLE: {mac_mode:<44}║")
    print("╠══════════════════════════════════════════════════╣")
    print("║  KEYS  Q=Quit  P=Pause  S=Stop  R=Resume        ║")
    print("║        H=Horn  B=Bell   L=Lights A=Announce     ║")
    print("║        F=Fwd   V=Rev    +/-=Speed                ║")
    print("╚══════════════════════════════════════════════════╝\n")


def main():
    print_banner()

    # ── Start BLE controller (connects in background) ──────────────
    logger.info("Starting BLE controller...")
    ble = TrainBLEController()
    ble.start()

    # ── Start zone detector (opens camera) ─────────────────────────
    logger.info("Starting zone detector...")
    detector = ZoneDetector()
    try:
        detector.start()
    except RuntimeError as e:
        logger.error(f"Camera error: {e}")
        ble.shutdown()
        sys.exit(1)

    # ── State ───────────────────────────────────────────────────────
    zone_was_active = False
    paused          = False
    resume_timer    = None
    bell_state      = False
    lights_state    = False
    current_speed   = 4
    stop_count      = 0
    resume_count    = 0
    start_time      = time.time()

    logger.info("Monitoring started — watching shared zone...\n")

    # ── Main loop ───────────────────────────────────────────────────
    while True:
        frame, detected, contours = detector.read_frame()

        if frame is None:
            time.sleep(0.05)
            continue

        # ── Collision logic ─────────────────────────────────────────
        if not paused:

            # Train enters zone → STOP
            if detected and not zone_was_active:
                resume_timer = None
                ble.stop_train()
                stop_count += 1
                logger.info(f"TRAIN IN ZONE — STOP sent (#{stop_count})")

            # Train leaves zone → start safety timer
            if not detected and zone_was_active:
                resume_timer = time.time()
                logger.info(
                    f"Zone cleared — waiting {config.RESUME_DELAY_SECONDS}s...")

            # Safety timer expired → RESUME
            if resume_timer and \
                    (time.time() - resume_timer) >= config.RESUME_DELAY_SECONDS:
                resume_timer = None
                ble.resume_train()
                resume_count += 1
                logger.info(f"RESUME sent (#{resume_count})")

            zone_was_active = detected

        # ── Stats overlay ───────────────────────────────────────────
        if config.SHOW_VIDEO and frame is not None:
            h, w    = frame.shape[:2]
            elapsed = int(time.time() - start_time)
            mm, ss  = elapsed // 60, elapsed % 60

            stats = [
                f"BLE: {'Connected' if ble.connected else 'Connecting...'}",
                f"Stops:{stop_count}  Resumes:{resume_count}",
                f"Speed: {current_speed}/7",
                f"Time: {mm:02d}:{ss:02d}",
                "--- PAUSED ---" if paused else "",
            ]
            box_x = w - 265
            cv2.rectangle(frame, (box_x, h - 115), (w - 4, h - 4),
                          (30, 30, 30), -1)
            for i, s in enumerate(stats):
                if s:
                    col = (0, 80, 255) if "PAUSED" in s else (200, 200, 200)
                    cv2.putText(frame, s, (box_x + 6, h - 95 + i * 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.44, col, 1)

            cv2.imshow(config.WINDOW_TITLE, frame)

        # ── Key handling ────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('p'), ord('P')):
            paused = not paused
            logger.info("Detection " + ("PAUSED" if paused else "RESUMED"))
        elif key in (ord('s'), ord('S')):
            ble.stop_train()
            logger.info("Manual STOP")
        elif key in (ord('r'), ord('R')):
            ble.resume_train()
            zone_was_active = False
            resume_timer = None
            logger.info("Manual RESUME")
        elif key in (ord('h'), ord('H')):
            ble.horn()
        elif key in (ord('b'), ord('B')):
            bell_state = not bell_state
            ble.bell_on() if bell_state else ble.bell_off()
        elif key in (ord('l'), ord('L')):
            lights_state = not lights_state
            ble.lights_on() if lights_state else ble.lights_off()
        elif key in (ord('a'), ord('A')):
            ble.announce()
        elif key in (ord('f'), ord('F')):
            ble.forward()
        elif key in (ord('v'), ord('V')):
            ble.reverse()
        elif key in (ord('+'), ord('=')):
            current_speed = min(7, current_speed + 1)
            ble.set_speed(current_speed)
            logger.info(f"Speed → {current_speed}")
        elif key in (ord('-'), ord('_')):
            current_speed = max(0, current_speed - 1)
            ble.set_speed(current_speed)
            logger.info(f"Speed → {current_speed}")

    # ── Shutdown ────────────────────────────────────────────────────
    logger.info("Shutting down...")
    ble.stop_train()
    time.sleep(0.4)
    ble.shutdown()
    detector.stop()

    elapsed = int(time.time() - start_time)
    print(f"\n{'='*52}")
    print(f"  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Stops: {stop_count}  Resumes: {resume_count}")
    print(f"{'='*52}\n")


if __name__ == "__main__":
    main()
