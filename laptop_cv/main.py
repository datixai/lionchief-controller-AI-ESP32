# ══════════════════════════════════════════════════════════════════
#  main.py  —  LionChief Laptop CV — Collision Prevention
#
#  THREE-ZONE LOGIC matching ESP32 IR sensor behaviour:
#
#    Zone A (inner approach) → inner train entering shared section
#      Action: BLE STOP outer train → keep sending every 0.5s
#              Zone stays LOCKED until train exits via Zone C
#
#    Zone B (outer approach) → outer train entering shared section
#      Action: Nothing extra needed — outer train is BLE controlled,
#              we already know where it is. But if Zone B fires while
#              zone C is occupied = danger → BLE STOP outer train
#
#    Zone C (shared section / exit) → any train in shared section
#      Action: When newly OCCUPIED = zone is active
#              When CLEARED → 2.5s safety delay → BLE RESUME
#              Unlock Zone A + B after resume
#
#  REPEATED STOP:
#    While any zone is locked, STOP is sent every 0.5s.
#    Prevents outer train creeping if first BLE packet missed.
#
#  RESUME RAMP:
#    Resume sends SLOW speed first, then after 3s ramps to FULL.
#    Prevents outer train rushing into shared section.
#
#  HOW TO RUN:
#    Step 1: python calibrate_zone.py   (draw 3 zones, press A to save)
#    Step 2: python main.py
#
#  KEYBOARD CONTROLS (click camera window first):
#    Q / ESC  = Quit
#    P        = Pause detection
#    S        = Manual STOP outer train
#    R        = Manual RESUME outer train
#    U        = Force unlock all zones (emergency reset)
#    H        = Horn        B = Bell    L = Lights
#    A        = Announce    F = Forward V = Reverse
#    + / -    = Speed up / down
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
    handlers.append(logging.FileHandler(config.LOG_FILE_PATH, mode='a'))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
    handlers=handlers
)
logger = logging.getLogger("Main")


def print_banner():
    mode = f"MAC: {config.TRAIN_MAC}" if config.TRAIN_MAC \
           else f"Scan: '{config.TRAIN_NAME}'"
    print("\n╔══════════════════════════════════════════════════╗")
    print("║  LionChief — 3-Zone Collision Prevention (CV)   ║")
    print(f"║  BLE: {mode:<44}║")
    print("╠══════════════════════════════════════════════════╣")
    print("║  Zone A = Inner loop approach                   ║")
    print("║  Zone B = Outer loop approach                   ║")
    print("║  Zone C = Shared section / exit                 ║")
    print("╠══════════════════════════════════════════════════╣")
    print("║  Q=Quit  P=Pause  S=Stop  R=Resume  U=Unlock    ║")
    print("║  H=Horn  B=Bell   L=Lights  +/-=Speed           ║")
    print("╚══════════════════════════════════════════════════╝\n")


def main():
    print_banner()

    # ── Start BLE ─────────────────────────────────────────────────
    logger.info("Starting BLE controller...")
    ble = TrainBLEController()
    ble.start()

    # ── Start detector ────────────────────────────────────────────
    logger.info("Starting zone detector...")
    detector = ZoneDetector()
    try:
        detector.start()
    except RuntimeError as e:
        logger.error(f"Camera error: {e}")
        ble.shutdown()
        sys.exit(1)

    # ── State ─────────────────────────────────────────────────────
    paused        = False
    zone_locked   = False       # any zone active = outer train stopped
    resume_timer  = None        # safety delay timer
    last_stop_sent = 0          # timestamp of last repeated STOP
    ramp_timer    = None        # timer for speed ramp after resume

    # Previous zone states for edge detection
    prev_states = {"A": False, "B": False, "C": False}

    # Manual control state
    bell_on    = False
    lights_on  = False
    cur_speed  = 4

    # Stats
    stop_count   = 0
    resume_count = 0
    start_time   = time.time()

    logger.info("System running — monitoring all three zones\n")

    # ── Main loop ─────────────────────────────────────────────────
    while True:
        frame, states = detector.read_frame()

        if frame is None:
            time.sleep(0.05)
            continue

        now = time.time()

        if not paused:

            zone_a = states.get("A", False)
            zone_b = states.get("B", False)
            zone_c = states.get("C", False)

            # ── ZONE A: Inner train entering shared section ────────
            # Inner train approaching → STOP outer train immediately
            if zone_a and not prev_states["A"]:
                if not zone_locked:
                    zone_locked  = True
                    resume_timer = None
                    ramp_timer   = None
                    ble.stop_train()
                    last_stop_sent = now
                    stop_count += 1
                    logger.info(
                        f"Zone A — inner train entering! "
                        f"BLE STOP sent (#{stop_count})")

            # ── ZONE B: Outer train entering shared section ────────
            # If zone C is occupied when outer train tries to enter
            # → danger → STOP outer train
            if zone_b and not prev_states["B"]:
                if zone_c and not zone_locked:
                    zone_locked  = True
                    resume_timer = None
                    ramp_timer   = None
                    ble.stop_train()
                    last_stop_sent = now
                    stop_count += 1
                    logger.info(
                        f"Zone B — outer train entering while C occupied! "
                        f"BLE STOP sent (#{stop_count})")

            # ── ZONE C: Train in shared section ───────────────────
            # When zone C first becomes active = train entered shared section
            if zone_c and not prev_states["C"]:
                if not zone_locked:
                    zone_locked  = True
                    resume_timer = None
                    ramp_timer   = None
                    ble.stop_train()
                    last_stop_sent = now
                    stop_count += 1
                    logger.info(
                        f"Zone C — train in shared section! "
                        f"BLE STOP sent (#{stop_count})")

            # When Zone C CLEARS → start safety delay → then resume
            if not zone_c and prev_states["C"] and zone_locked:
                resume_timer = now
                logger.info(
                    f"Zone C cleared — {config.RESUME_DELAY_SECONDS}s "
                    f"safety delay starting...")

            # ── REPEATED STOP while zone locked ───────────────────
            # Keep sending STOP every 0.5s — prevents train creeping
            # in if first BLE packet was missed
            if zone_locked and resume_timer is None:
                if now - last_stop_sent >= config.STOP_REPEAT_INTERVAL:
                    ble.stop_train()
                    last_stop_sent = now

            # ── Safety delay expired → RESUME ─────────────────────
            if resume_timer and (now - resume_timer) >= config.RESUME_DELAY_SECONDS:
                resume_timer = None
                zone_locked  = False
                ramp_timer   = now

                # Unlock zones in detector
                detector.unlock_zone("A")
                detector.unlock_zone("B")
                detector.unlock_zone("C")

                # Update reference frames (zone is now empty)
                detector.update_reference("A")
                detector.update_reference("B")
                detector.update_reference("C")

                # Resume at slow speed first
                ble.resume_train()
                resume_count += 1
                logger.info(
                    f"RESUME slow speed (#{resume_count}) — "
                    f"will ramp to full in {config.SPEED_RAMP_DELAY}s")

            # ── Speed ramp: slow → full after delay ───────────────
            if ramp_timer and (now - ramp_timer) >= config.SPEED_RAMP_DELAY:
                ramp_timer = None
                ble.resume_full()
                logger.info("RESUME full speed")

            # Update previous states
            prev_states = {"A": zone_a, "B": zone_b, "C": zone_c}

        # ── Stats overlay ──────────────────────────────────────────
        if config.SHOW_VIDEO and frame is not None:
            h, w   = frame.shape[:2]
            elapsed = int(now - start_time)
            mm, ss  = elapsed // 60, elapsed % 60

            stats = [
                f"BLE: {'OK' if ble.connected else 'connecting...'}",
                f"Stops: {stop_count}  Resumes: {resume_count}",
                f"Speed: {cur_speed}/7",
                f"Time:  {mm:02d}:{ss:02d}",
                "PAUSED" if paused else
                ("ZONE LOCKED" if zone_locked else "monitoring"),
            ]
            bx = w - 200
            cv2.rectangle(frame, (bx, h - 108), (w - 4, h - 30),
                          (30, 30, 30), -1)
            for i, s in enumerate(stats):
                col = (0, 80, 255) if s in ("PAUSED", "ZONE LOCKED") \
                      else (200, 200, 200)
                cv2.putText(frame, s, (bx + 6, h - 90 + i * 17),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)

            cv2.imshow(config.WINDOW_TITLE, frame)

        # ── Key handling ───────────────────────────────────────────
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
            zone_locked  = False
            resume_timer = None
            ramp_timer   = None
            detector.unlock_zone("A")
            detector.unlock_zone("B")
            detector.unlock_zone("C")
            ble.resume_train()
            logger.info("Manual RESUME")

        elif key in (ord('u'), ord('U')):
            # Emergency unlock — force reset all zones
            zone_locked  = False
            resume_timer = None
            ramp_timer   = None
            detector.unlock_zone("A")
            detector.unlock_zone("B")
            detector.unlock_zone("C")
            detector.update_reference("A")
            detector.update_reference("B")
            detector.update_reference("C")
            logger.info("FORCE UNLOCK — all zones reset + reference refreshed")

        elif key in (ord('h'), ord('H')):
            ble.horn()

        elif key in (ord('b'), ord('B')):
            bell_on = not bell_on
            ble.bell_on() if bell_on else ble.bell_off()

        elif key in (ord('l'), ord('L')):
            lights_on = not lights_on
            ble.lights_on() if lights_on else ble.lights_off()

        elif key in (ord('a'), ord('A')):
            ble.announce()

        elif key in (ord('f'), ord('F')):
            ble.forward()

        elif key in (ord('v'), ord('V')):
            ble.reverse()

        elif key in (ord('+'), ord('=')):
            cur_speed = min(7, cur_speed + 1)
            ble.set_speed(cur_speed)
            logger.info(f"Speed → {cur_speed}")

        elif key in (ord('-'), ord('_')):
            cur_speed = max(0, cur_speed - 1)
            ble.set_speed(cur_speed)
            logger.info(f"Speed → {cur_speed}")

    # ── Shutdown ───────────────────────────────────────────────────
    logger.info("Shutting down...")
    ble.stop_train()
    time.sleep(0.4)
    ble.shutdown()
    detector.stop()

    elapsed = int(time.time() - start_time)
    print(f"\n{'═'*52}")
    print(f"  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Stops: {stop_count}  Resumes: {resume_count}")
    print(f"{'═'*52}\n")


if __name__ == "__main__":
    main()
