# ══════════════════════════════════════════════════════════════════
#  main.py  —  LionChief Camera Collision Prevention
#  Harry Locomotive Project  |  Datix AI  |  May 2026
#
#  Replicates ESP32 v5.1 state machine using camera zone detection
#  instead of IR sensors. Controls outer BLE train via Python/bleak.
#
#  STATE MACHINE (6 states — identical to ESP32 v5.1):
#
#    IDLE ──[Zone A]──► LOCKED(inner) ──[Zone C clears]──► INNER_PARKING
#                                                                │
#                                                       [PARKING_DELAY_S]
#                                                                ▼
#    IDLE ──[Zone B]──► LOCKED(outer) ──[Zone C clears]──►  DELAY
#                                                                │
#                                                       [outer: SWITCH]
#                                                       [inner: RAMP  ]
#                                                                ▼
#                                   IDLE ◄── RAMP ◄── SWITCH ──┘
#
#    IDLE           Outer running, inner parked — all clear
#    LOCKED         Zone occupied — outer stopped / collision prevention
#    INNER_PARKING  Inner exited zone — coasting to parking position
#    DELAY          Safety buffer before resuming outer train
#    SWITCH         Timing delay replacing ESP32 relay2 switch pulse
#    RAMP           Outer ramping back to user-set speed
#
#  ZONE → ACTION MAPPING:
#    Zone A detected → inner train approaching → STOP outer BLE
#    Zone B detected → outer train approaching → monitor only
#                      (inner train relay = hardware / ESP32 side)
#    Zone C clears   → train exited shared section → resume sequence
#
#  AUTO INNER TIMER:
#    Every AUTO_INNER_INTERVAL_S seconds, logs a reminder.
#    If ESP32 is connected via USB serial, sends 'X' command to
#    trigger one inner train loop via hardware relay.
#
#  HOW TO RUN:
#    Step 1: python calibrate_zone.py   (draw 3 zones, press A)
#    Step 2: python main.py
#
#  KEYBOARD CONTROLS (click camera window first):
#    Q / ESC  = Quit
#    P        = Pause / resume detection
#    S        = Manual STOP outer train
#    R        = Manual RESUME outer train (force unlock all zones)
#    1-7      = Set outer train speed (restored after collision stops)
#    + / -    = Speed up / down
#    H        = Horn    B = Bell    L = Lights
#    A        = Announce
#    U        = Force unlock all zones (emergency reset)
# ══════════════════════════════════════════════════════════════════

import cv2
import time
import logging
import sys
import os
import serial
import serial.tools.list_ports
from enum import Enum

import config
from zone_detector  import ZoneDetector
from ble_controller import TrainBLEController

# ── Logging setup ─────────────────────────────────────────────────
handlers = [logging.StreamHandler(sys.stdout)]
if config.LOG_TO_FILE:
    handlers.append(logging.FileHandler(config.LOG_FILE_PATH, mode='a'))

logging.basicConfig(
    level   = logging.INFO,
    format  = "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt = "%H:%M:%S",
    handlers= handlers,
)
logger = logging.getLogger("Main")


# ── State machine ─────────────────────────────────────────────────

class ZoneState(Enum):
    IDLE          = "IDLE"
    LOCKED        = "LOCKED"
    INNER_PARKING = "INNER_PARKING"
    DELAY         = "DELAY"
    SWITCH        = "SWITCH"   # timing delay replacing hardware switch pulse
    RAMP          = "RAMP"


# ── Optional ESP32 serial connection ──────────────────────────────

def find_esp32_serial():
    """Try to auto-detect ESP32 USB serial port."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if any(k in desc for k in ["silicon labs", "cp210", "ch340", "uart"]):
            return p.device
    return None


def connect_esp32_serial(port=None):
    """
    Open optional serial connection to ESP32.
    Returns serial.Serial if successful, None otherwise.
    Used to send 'X' command (start inner loop) to ESP32 relay.
    """
    target = port or find_esp32_serial()
    if not target:
        return None
    try:
        ser = serial.Serial(target, 115200, timeout=1)
        logger.info(f"ESP32 serial connected on {target}")
        return ser
    except Exception as e:
        logger.warning(f"ESP32 serial not available ({e}) — inner train is manual only")
        return None


def send_esp32(ser, cmd: str):
    """Send a single-character command to ESP32 via serial."""
    if ser:
        try:
            ser.write(cmd.encode())
            logger.info(f"ESP32 serial → '{cmd}'")
        except Exception as e:
            logger.warning(f"ESP32 serial send failed: {e}")


# ── Banner ─────────────────────────────────────────────────────────

def print_banner(ble_mac, esp32_port):
    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Camera Collision Prevention  v5.1      ║")
    print("║  Datix AI  |  Ahmed Ali  |  May 2026                ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  BLE train  : {ble_mac:<38}║")
    print(f"║  ESP32 port : {(esp32_port or 'not connected — inner train manual'):<38}║")
    print(f"║  Camera     : index {config.CAMERA_INDEX:<34}║")
    print(f"║  Inner auto : every {config.AUTO_INNER_INTERVAL_S:.0f}s                           ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  Zone A → Inner approach  |  Zone B → Outer approach║")
    print("║  Zone C → Shared section / exit                     ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  Q=Quit  P=Pause  S=Stop  R=Resume  U=ForceUnlock   ║")
    print("║  1-7=Speed  +/-=Adjust   H=Horn  B=Bell  L=Lights   ║")
    print("╚══════════════════════════════════════════════════════╝\n")


# ── Main ──────────────────────────────────────────────────────────

def main():
    # Try connecting to ESP32 for inner train relay control
    esp32 = connect_esp32_serial()

    print_banner(config.TRAIN_MAC, find_esp32_serial())

    # Start BLE controller
    logger.info("Starting BLE controller...")
    ble = TrainBLEController()
    ble.start()

    # Start camera zone detector
    logger.info("Starting zone detector...")
    detector = ZoneDetector()
    try:
        detector.start()
    except RuntimeError as e:
        logger.error(f"Camera error: {e}")
        ble.shutdown()
        sys.exit(1)

    # ── State ──────────────────────────────────────────────────
    state            = ZoneState.IDLE
    inner_caused     = False  # True = inner train triggered zone lock
                               # False = outer train triggered zone lock
    inner_was_cut    = False  # inner train was cut because outer entered

    state_enter_time = time.time()
    last_stop_sent   = 0.0
    last_keepalive   = time.time()
    last_inner_run   = time.time()  # for auto inner timer

    paused           = False
    horn_on          = False
    bell_on          = False
    lights_on        = False

    # Statistics
    stops_count      = 0
    resumes_count    = 0
    zone_timeouts    = 0
    start_time       = time.time()

    # Previous zone states for edge detection
    prev_states = {"A": False, "B": False, "C": False}

    logger.info("System running — monitoring all zones\n")

    def enter_state(new_state):
        """Transition to a new state and record entry time."""
        nonlocal state, state_enter_time
        logger.info(f"[STATE] {state.value} → {new_state.value}")
        state            = new_state
        state_enter_time = time.time()

    def time_in_state() -> float:
        return time.time() - state_enter_time

    # ── Main loop ─────────────────────────────────────────────
    while True:
        frame, zone_states = detector.read_frame()

        if frame is None:
            time.sleep(0.02)
            continue

        now = time.time()

        # ── BLE keepalive ────────────────────────────────────
        if now - last_keepalive >= config.KEEPALIVE_INTERVAL:
            last_keepalive = now
            ble.keepalive()

        # ── Auto inner train timer ────────────────────────────
        # Every AUTO_INNER_INTERVAL_S when IDLE, trigger inner train
        if (state == ZoneState.IDLE and
                now - last_inner_run >= config.AUTO_INNER_INTERVAL_S):
            last_inner_run = now
            if esp32:
                logger.info(
                    f"[INNER] Auto-loop triggered — sending X to ESP32")
                send_esp32(esp32, 'X')
            else:
                logger.info(
                    f"[INNER] ⏰ Auto-loop time — trigger inner train manually "
                    f"(press X on ESP32 serial monitor)")

        # ════════════════════════════════════════════════════════
        #  ZONE STATE MACHINE — mirrors ESP32 v5.1
        # ════════════════════════════════════════════════════════

        if not paused:
            zone_a = zone_states.get("A", False)
            zone_b = zone_states.get("B", False)
            zone_c = zone_states.get("C", False)

            # ── IDLE: watch for trains approaching ───────────
            if state == ZoneState.IDLE:

                # Zone A: inner train entering shared section
                if zone_a and not prev_states["A"]:
                    logger.info(
                        f"\n[ZONE A] ⚠️  Inner train detected!\n"
                        f"[ACTION]  BLE STOP — saving user speed "
                        f"{ble.user_speed} for restore on resume")
                    inner_caused  = True
                    inner_was_cut = False
                    stops_count  += 1
                    ble.send_stop()
                    last_stop_sent = now
                    enter_state(ZoneState.LOCKED)

                # Zone B: outer train entering shared section
                elif zone_b and not prev_states["B"]:
                    logger.info(
                        f"\n[ZONE B] ⚠️  Outer train detected entering!\n"
                        f"[ACTION]  Zone locked — sending relay cut to ESP32")
                    inner_caused = False

                    # Cut inner train power via ESP32 serial if connected
                    if esp32:
                        send_esp32(esp32, 'z')  # emergency cut inner power
                        inner_was_cut = True
                        logger.info("[RELAY] Inner train power cut via ESP32")
                    else:
                        inner_was_cut = False
                        logger.info("[RELAY] No ESP32 — inner train cut is manual")

                    enter_state(ZoneState.LOCKED)

            # ── LOCKED: zone occupied ────────────────────────
            elif state == ZoneState.LOCKED:

                # Repeat STOP every 0.5s while inner is in zone
                if inner_caused and (now - last_stop_sent >= config.STOP_REPEAT_S):
                    last_stop_sent = now
                    ble.send_stop_raw()

                # Zone C cleared → train has exited shared section
                if not zone_c and prev_states.get("C_active", False):
                    logger.info("[ZONE C] ✅ Shared section cleared")
                    if inner_caused:
                        logger.info(
                            f"[INNER]  Coasting to parking — "
                            f"{config.PARKING_DELAY_S}s then power cut")
                        enter_state(ZoneState.INNER_PARKING)
                    else:
                        logger.info(
                            f"[SYSTEM] Outer exited — "
                            f"safety delay {config.RESUME_DELAY_S}s")
                        enter_state(ZoneState.DELAY)

                # Track Zone C active state for edge detection
                prev_states["C_active"] = zone_c

                # Timeout: Zone C never cleared
                if time_in_state() >= config.ZONE_TIMEOUT_S:
                    zone_timeouts += 1
                    logger.warning(
                        f"[TIMEOUT] ⚠️  Zone locked {config.ZONE_TIMEOUT_S}s "
                        f"without clearing — force resuming\n"
                        f"          {'Inner' if inner_caused else 'Outer'} may be "
                        f"stuck or derailed in shared section")
                    # Reset inner timer so auto-loop doesn't fire immediately
                    last_inner_run = now
                    inner_was_cut  = False
                    enter_state(ZoneState.DELAY)

            # ── INNER_PARKING: inner coasting to parking ─────
            elif state == ZoneState.INNER_PARKING:

                # Keep outer stopped during parking coast
                if now - last_stop_sent >= config.STOP_REPEAT_S:
                    last_stop_sent = now
                    ble.send_stop_raw()

                if time_in_state() >= config.PARKING_DELAY_S:
                    # Cut inner train power at parking position
                    if esp32:
                        send_esp32(esp32, 'z')
                        logger.info("[INNER] ⛔ Stopped at parking (ESP32 relay cut)")
                    else:
                        logger.info("[INNER] ⛔ Parking time reached — cut manually")

                    # Reset auto timer
                    last_inner_run = now
                    logger.info(
                        f"[INNER] Next auto-loop in {config.AUTO_INNER_INTERVAL_S:.0f}s")
                    enter_state(ZoneState.DELAY)

            # ── DELAY: safety buffer ─────────────────────────
            elif state == ZoneState.DELAY:

                # Keep sending STOP during delay
                if now - last_stop_sent >= config.STOP_REPEAT_S:
                    last_stop_sent = now
                    ble.send_stop_raw()

                if time_in_state() >= config.RESUME_DELAY_S:
                    if not inner_caused:
                        # Outer exited → switch timing delay
                        logger.info(
                            f"[SWITCH] Track switch timing delay "
                            f"{config.STOP_REPEAT_S * 2:.1f}s")
                        enter_state(ZoneState.SWITCH)
                    else:
                        # Inner parked → resume outer at ramp speed
                        logger.info(
                            f"[RESUME] Inner parked → outer ramping to speed "
                            f"{min(config.RESUME_RAMP_SPEED, ble.user_speed)}")
                        ble.resume_ramp()
                        resumes_count += 1
                        enter_state(ZoneState.RAMP)

            # ── SWITCH: replaces hardware track switch pulse ──
            elif state == ZoneState.SWITCH:

                # Keep sending STOP
                if now - last_stop_sent >= config.STOP_REPEAT_S:
                    last_stop_sent = now
                    ble.send_stop_raw()

                # 1-second switch timing (matches hardware relay pulse)
                if time_in_state() >= 1.0:
                    # Restore inner train power if it was cut for outer
                    if inner_was_cut and esp32:
                        send_esp32(esp32, 'x')
                        inner_was_cut = False
                        logger.info("[INNER] Power restored — resuming loop")
                    else:
                        inner_was_cut = False

                    logger.info(
                        f"[RESUME] Outer train resuming → "
                        f"ramp speed {min(config.RESUME_RAMP_SPEED, ble.user_speed)}")
                    ble.resume_ramp()
                    resumes_count += 1
                    enter_state(ZoneState.RAMP)

            # ── RAMP: ramp up to user speed ──────────────────
            elif state == ZoneState.RAMP:

                if time_in_state() >= config.SPEED_RAMP_S:
                    logger.info(
                        f"[RESUME] Restoring user speed {ble.user_speed}/7")
                    ble.resume_user_speed()

                    # Unlock all zones and refresh reference frames
                    for z_key in ["A", "B", "C"]:
                        detector.unlock_zone(z_key)
                        detector.update_reference(z_key)

                    inner_caused  = False
                    inner_was_cut = False
                    prev_states   = {"A": False, "B": False, "C": False}
                    enter_state(ZoneState.IDLE)
                    logger.info("[ZONE] ✅ UNLOCKED — outer at user speed\n")

            # Update previous states
            prev_states.update({
                "A": zone_a,
                "B": zone_b,
                "C": zone_c,
            })

        # ── Stats overlay ────────────────────────────────────
        if config.SHOW_VIDEO and frame is not None:
            h, w    = frame.shape[:2]
            elapsed = int(now - start_time)
            mm, ss  = elapsed // 60, elapsed % 60

            info_lines = [
                f"BLE: {'OK ✅' if ble.connected else 'connecting...'}",
                f"State: {state.value}",
                f"Speed: {ble.current_speed}/7 (usr:{ble.user_speed})",
                f"Stops:{stops_count}  Rsm:{resumes_count}",
                f"Time: {mm:02d}:{ss:02d}",
                "PAUSED" if paused else ("LOCKED" if state != ZoneState.IDLE else ""),
            ]
            bx = w - 210
            cv2.rectangle(frame, (bx, h - 115), (w - 4, h - 28),
                          (30, 30, 30), -1)
            for i, s in enumerate(info_lines):
                if not s:
                    continue
                c = (0, 80, 255) if s in ("PAUSED", "LOCKED") else (200, 200, 200)
                cv2.putText(frame, s, (bx + 6, h - 100 + i * 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, c, 1)

            cv2.imshow(config.WINDOW_TITLE, frame)

        # ── Keyboard input ───────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break

        elif key in (ord('p'), ord('P')):
            paused = not paused
            logger.info(f"Detection {'PAUSED' if paused else 'RESUMED'}")

        elif key in (ord('s'), ord('S')):
            ble.send_stop()
            logger.info("Manual STOP")

        elif key in (ord('r'), ord('R')):
            # Force unlock all zones and resume
            for z_key in ["A", "B", "C"]:
                detector.unlock_zone(z_key)
                detector.update_reference(z_key)
            ble.resume_ramp()
            inner_caused  = False
            inner_was_cut = False
            prev_states   = {"A": False, "B": False, "C": False}
            enter_state(ZoneState.IDLE)
            logger.info("Force RESUME — all zones unlocked")

        elif key in (ord('u'), ord('U')):
            # Emergency unlock — full reset
            for z_key in ["A", "B", "C"]:
                detector.unlock_zone(z_key)
                detector.update_reference(z_key)
            inner_caused   = False
            inner_was_cut  = False
            last_inner_run = now
            prev_states    = {"A": False, "B": False, "C": False}
            enter_state(ZoneState.IDLE)
            logger.info("FORCE UNLOCK — all zones + references reset")

        elif key == ord('+') or key == ord('='):
            ble.set_speed(ble.current_speed + 1)

        elif key == ord('-') or key == ord('_'):
            ble.set_speed(max(1, ble.current_speed - 1))

        elif key in (ord('1'), ord('2'), ord('3'), ord('4'),
                     ord('5'), ord('6'), ord('7')):
            ble.set_speed(int(chr(key)))

        elif key in (ord('h'), ord('H')):
            horn_on = not horn_on
            ble.horn_on() if horn_on else ble.horn_off()

        elif key in (ord('b'), ord('B')):
            bell_on = not bell_on
            ble.bell_on() if bell_on else ble.bell_off()

        elif key in (ord('l'), ord('L')):
            lights_on = not lights_on
            ble.lights_on() if lights_on else ble.lights_off()

        elif key in (ord('a'), ord('A')):
            ble.announce()

    # ── Shutdown ─────────────────────────────────────────────
    logger.info("Shutting down...")
    ble.shutdown()
    detector.stop()
    if esp32:
        esp32.close()

    elapsed = int(time.time() - start_time)
    print(f"\n{'═'*54}")
    print(f"  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Stops: {stops_count}  Resumes: {resumes_count}  "
          f"Timeouts: {zone_timeouts}")
    print(f"{'═'*54}\n")


if __name__ == "__main__":
    main()