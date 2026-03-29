# ══════════════════════════════════════════════════════════════════
#  main.py  —  LionChief Laptop CV — Entry Point
#
#  Collision prevention using laptop camera only.
#  No ESP32. No Raspberry Pi. Just laptop + USB camera + train.
#
#  HOW TO USE:
#    Step 1:  pip install -r requirements.txt
#    Step 2:  python calibrate_zone.py   (do this ONCE)
#    Step 3:  paste zone coords into config.py
#    Step 4:  python main.py
#
#  CONTROLS (press in camera window):
#    Q / ESC = quit
#    P       = pause / resume detection
#    S       = manually send STOP to train
#    R       = manually send RESUME to train
#    H       = horn
#    B       = bell toggle
#    +       = speed up
#    -       = speed down
#    C       = show calibration overlay
# ══════════════════════════════════════════════════════════════════

import cv2
import time
import logging
import sys
import os

import config
from zone_detector  import ZoneDetector
from ble_controller import TrainBLEController

# ── LOGGING SETUP ─────────────────────────────────────────────────
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
    print("\n╔══════════════════════════════════════════════════╗")
    print("║   LionChief Collision Prevention — Laptop CV    ║")
    print("╠══════════════════════════════════════════════════╣")
    print("║  Camera → OpenCV Zone Detection → BLE → Train   ║")
    print("║  No ESP32. No Raspberry Pi. Laptop only.         ║")
    print("╠══════════════════════════════════════════════════╣")
    print("║  KEYS (click camera window first):              ║")
    print("║   Q/ESC = Quit                                   ║")
    print("║   P     = Pause/Resume detection                 ║")
    print("║   S     = Manual STOP train                      ║")
    print("║   R     = Manual RESUME train                    ║")
    print("║   H     = Horn                                    ║")
    print("║   B     = Bell toggle                             ║")
    print("║   + / - = Speed up / down                        ║")
    print("╚══════════════════════════════════════════════════╝\n")


def main():
    print_banner()

    # ── INIT COMPONENTS ───────────────────────────────────────────
    logger.info("Starting BLE controller...")
    ble = TrainBLEController()
    ble.start()

    logger.info("Starting zone detector...")
    detector = ZoneDetector()
    try:
        detector.start()
    except RuntimeError as e:
        logger.error(f"Camera error: {e}")
        ble.shutdown()
        sys.exit(1)

    # ── STATE ─────────────────────────────────────────────────────
    zone_was_active  = False      # previous frame state
    zone_active      = False      # current frame state
    paused           = False      # P key toggles
    resume_timer     = None       # when zone cleared
    bell_on          = False
    current_speed    = 4          # default medium speed
    stop_count       = 0
    resume_count     = 0
    start_time       = time.time()

    logger.info("System running — monitoring shared zone...\n")

    # ── MAIN LOOP ─────────────────────────────────────────────────
    while True:
        frame, detected, contours = detector.read_frame()

        if frame is None:
            logger.warning("No frame — camera issue?")
            time.sleep(0.1)
            continue

        if not paused:
            # ── COLLISION LOGIC ───────────────────────────────────
            #
            # Train ENTERS zone → STOP immediately
            if detected and not zone_was_active:
                zone_active = True
                resume_timer = None
                ble.stop_train()
                stop_count += 1
                logger.info(f"⚠️  TRAIN IN SHARED ZONE — STOP sent "
                            f"(stop #{stop_count})")

            # Train CLEARS zone → start safety timer
            if not detected and zone_was_active:
                resume_timer = time.time()
                logger.info(f"✅ Zone cleared — {config.RESUME_DELAY_SECONDS}s "
                            f"safety delay...")

            # Safety timer expired → RESUME
            if (resume_timer and
                    (time.time() - resume_timer) >= config.RESUME_DELAY_SECONDS):
                zone_active = False
                resume_timer = None
                ble.resume_train()
                resume_count += 1
                logger.info(f"✅ RESUME sent (resume #{resume_count})")

            zone_was_active = detected

        # ── OVERLAY STATS ─────────────────────────────────────────
        if config.SHOW_VIDEO and frame is not None:
            h, w = frame.shape[:2]
            elapsed = int(time.time() - start_time)
            mm, ss  = elapsed // 60, elapsed % 60

            # Stats box bottom right
            stats = [
                f"BLE: {'Connected' if ble.connected else 'Searching...'}",
                f"Stops: {stop_count}   Resumes: {resume_count}",
                f"Speed: {current_speed}/7",
                f"Runtime: {mm:02d}:{ss:02d}",
                "PAUSED" if paused else "",
            ]
            box_x = w - 260
            cv2.rectangle(frame, (box_x, h - 110), (w - 5, h - 5),
                          (30, 30, 30), -1)
            for i, s in enumerate(stats):
                if s:
                    col = (0, 100, 255) if s == "PAUSED" else (200, 200, 200)
                    cv2.putText(frame, s, (box_x + 6, h - 90 + i * 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

            cv2.imshow(config.WINDOW_TITLE, frame)

        # ── KEY HANDLING ──────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):          # Q / ESC
            break

        elif key in (ord('p'), ord('P')):             # Pause
            paused = not paused
            logger.info(f"Detection {'PAUSED' if paused else 'RESUMED'}")

        elif key in (ord('s'), ord('S')):             # Manual stop
            ble.stop_train()
            logger.info("Manual STOP sent")

        elif key in (ord('r'), ord('R')):             # Manual resume
            ble.resume_train()
            zone_active = zone_was_active = False
            resume_timer = None
            logger.info("Manual RESUME sent")

        elif key in (ord('h'), ord('H')):             # Horn
            ble.horn()

        elif key in (ord('b'), ord('B')):             # Bell
            if bell_on:
                ble.bell_off()
                bell_on = False
            else:
                ble.bell_on()
                bell_on = True

        elif key in (ord('+'), ord('='),              # Speed up
                     0x57, 0x2B):
            current_speed = min(7, current_speed + 1)
            ble.set_speed(current_speed)
            logger.info(f"Speed → {current_speed}")

        elif key in (ord('-'), ord('_')):             # Speed down
            current_speed = max(0, current_speed - 1)
            ble.set_speed(current_speed)
            logger.info(f"Speed → {current_speed}")

    # ── CLEANUP ───────────────────────────────────────────────────
    logger.info("Shutting down...")
    ble.stop_train()
    time.sleep(0.5)
    ble.shutdown()
    detector.stop()

    print("\n╔══════════════════════════════════════════════════╗")
    print("║  Session Summary                                 ║")
    print(f"║  Total stops:   {stop_count:<32} ║")
    print(f"║  Total resumes: {resume_count:<32} ║")
    elapsed = int(time.time() - start_time)
    print(f"║  Runtime:       {elapsed // 60:02d}m {elapsed % 60:02d}s"
          f"{'':25} ║")
    print("╚══════════════════════════════════════════════════╝\n")


if __name__ == "__main__":
    main()
