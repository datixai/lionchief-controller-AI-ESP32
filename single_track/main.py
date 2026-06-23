# ══════════════════════════════════════════════════════════════════
#  main.py  —  Single Track Safe Distance
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  HOW COORDINATE FIX WORKS:
#    Raw camera frame (1280×720) is resized to DISPLAY_W×DISPLAY_H
#    (960×540) before ANYTHING else happens — before showing,
#    before tracker init, before drawing.
#    Window is opened with WINDOW_AUTOSIZE so it is exactly
#    DISPLAY_W×DISPLAY_H — no padding, no scaling.
#    Mouse coordinates from OpenCV therefore ALWAYS equal pixel
#    positions in the display frame. Works on any DPI setting.
#
#  CAMERA BLINK FIX:
#    40 warm-up frames are read silently before opening the window.
#    Auto-exposure settles during warm-up — no visible flicker.
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
    format="%(asctime)s [%(name)s] %(message)s",
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
        logger.info(f"Calibration loaded — zones set")
    except Exception as e:
        logger.warning(f"Calibration load failed: {e}")


# ── Global tracker (shared with mouse callback) ───────────────────
_tracker = None


def mouse_callback(event, x, y, flags, param):
    """
    Mouse events arrive in DISPLAY coordinates (960×540).
    Window is WINDOW_AUTOSIZE at exactly DISPLAY_W×DISPLAY_H.
    Therefore (x, y) directly equals pixel position in display frame.
    No scaling needed — this is the whole point of the design.
    """
    if _tracker is None:
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        _tracker.on_mouse_down(x, y)
    elif event == cv2.EVENT_MOUSEMOVE:
        _tracker.on_mouse_move(x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        _tracker.on_mouse_up(x, y)


# ── Overlay drawing ───────────────────────────────────────────────

def draw_overlay(display, tracker, pos_a, pos_b,
                 dist, zone, speed, user_speed, ble_ok, paused):
    h, w = display.shape[:2]
    now  = time.time()

    # ── Train bounding boxes ──────────────────────────────────────
    for pos, col, lbl in [
        (pos_a, (0, 165, 255), "A  front"),
        (pos_b, (0, 220,  50), f"B  spd:{speed}"),
    ]:
        if pos and pos.bbox:
            bx, by, bw, bh = pos.bbox
            cv2.rectangle(display, (bx, by), (bx+bw, by+bh), col, 2)
        if pos:
            cv2.circle(display, (pos.x, pos.y), 5, col, -1)
            cv2.putText(display, lbl, (pos.x+10, pos.y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

    # ── Live drag rectangle ───────────────────────────────────────
    if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
        x1 = min(tracker.drag_start[0], tracker.drag_end[0])
        y1 = min(tracker.drag_start[1], tracker.drag_end[1])
        x2 = max(tracker.drag_start[0], tracker.drag_end[0])
        y2 = max(tracker.drag_start[1], tracker.drag_end[1])
        dcol = (0,165,255) if tracker.state == WAIT_A else (0,220,50)
        cv2.rectangle(display, (x1,y1), (x2,y2), dcol, 2)
        lbl = "Train A" if tracker.state == WAIT_A else "Train B"
        cv2.putText(display, lbl, (x1+4, max(y1-6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, dcol, 2)

    # ── Distance line ─────────────────────────────────────────────
    if pos_a and pos_b and dist:
        zcol = config.ZONE_COLORS.get(zone, (150,150,150))
        cv2.line(display, (pos_a.x,pos_a.y), (pos_b.x,pos_b.y), zcol, 2)
        mx = (pos_a.x + pos_b.x) // 2
        my = (pos_a.y + pos_b.y) // 2
        cv2.putText(display, f"{dist:.0f}px | {zone}",
                    (mx+6, my-6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, zcol, 2)

    # ── Top status bar ────────────────────────────────────────────
    if tracker.state in (WAIT_A, WAIT_B):
        # Guided selection mode
        cv2.rectangle(display, (0,0), (w,52), (40,25,0), -1)
        col  = (0,165,255) if tracker.state == WAIT_A else (0,220,50)
        msg  = ("STEP 1:  Hold + DRAG a box around  TRAIN A  (front train)"
                if tracker.state == WAIT_A else
                "STEP 2:  Hold + DRAG a box around  TRAIN B  (rear BLE train)")
        cv2.putText(display, msg, (8,28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, col, 2)
        cv2.putText(display,
                    "Draw the box around the whole train body — it does not need to be perfect",
                    (8,46), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180,180,180), 1)
    else:
        # Tracking mode status bar
        zcol = config.ZONE_COLORS.get(zone, (80,80,80))
        barc = (0,0,90)  if zone == Zone.DANGER else \
               (0,55,0)  if zone == Zone.SAFE   else (25,25,25)
        cv2.rectangle(display, (0,0), (w,48), barc, -1)

        if paused:
            msg  = "PAUSED — press P to resume"
            mcol = (0,200,255)
        elif not tracker.tracking_a or not tracker.tracking_b:
            msg  = "Tracker lost — press A or B then drag new box around that train"
            mcol = (0,80,255)
        else:
            msgs = {
                Zone.DANGER:  f"STOP — gap too small",
                Zone.WARNING: f"WARNING — slowing Train B (speed {speed})",
                Zone.CAUTION: f"CAUTION — speed {speed}",
                Zone.SAFE:    f"SAFE — following at speed {speed}",
                Zone.FAR:     f"FAR — catching up (speed {speed})",
                Zone.UNKNOWN: f"Train not visible — holding speed {speed}",
            }
            msg  = msgs.get(zone, zone)
            mcol = zcol

        cv2.putText(display, msg, (8,26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        cv2.putText(display,
                    f"A:{'OK' if tracker.tracking_a else 'LOST'}  "
                    f"B:{'OK' if tracker.tracking_b else 'LOST'}  |  "
                    f"BLE:{'OK' if ble_ok else 'connecting'}  |  "
                    f"Spd:{speed}/7 usr:{user_speed}  |  "
                    f"Gap:{f'{dist:.0f}px' if dist else '---'}  |  "
                    f"A/B = re-select",
                    (8,44), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200,200,200), 1)

    # ── Flash confirmation (2 seconds after each selection) ────────
    flash = []
    if now - tracker.flash_a_time < 2.0:
        flash.append(("  Train A box SAVED  —  now drag a box around Train B",
                       (0, 165, 255)))
    if now - tracker.flash_b_time < 2.0:
        flash.append(("  Train B box SAVED  —  tracking started!",
                       (0, 220, 50)))
    for i, (msg, col) in enumerate(flash):
        fy = h // 2 - 26 + i * 56
        cv2.rectangle(display, (0, fy), (w, fy+50), (15,15,15), -1)
        cv2.rectangle(display, (0, fy), (w, fy+50), col, 3)
        cv2.putText(display, msg, (20, fy+34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, col, 2)

    # ── Bottom bar ────────────────────────────────────────────────
    cv2.rectangle(display, (0,h-22), (w,h), (20,20,20), -1)
    cv2.putText(display,
                "DRAG=select   A/B=reselect   1-7=Speed   "
                "S=Stop   R=Resume   P=Pause   H=Horn   Q=Quit",
                (8,h-7), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (130,130,130), 1)

    return display


# ── Main ──────────────────────────────────────────────────────────

def main():
    global _tracker

    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Safe Distance Control  v4.0            ║")
    print("║  Datix AI  |  June 2026                             ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  Camera warms up → window opens → drag boxes        ║")
    print("║  Train A = front (manual)  Train B = rear (BLE)     ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    load_calibration()

    # BLE controller
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

    # ── CAMERA WARM-UP ────────────────────────────────────────────
    # Read frames silently so auto-exposure settles before window opens.
    # This prevents the camera blink/flicker on startup.
    logger.info(f"Camera warming up ({config.CAMERA_WARMUP_FRAMES} frames)...")
    for _ in range(config.CAMERA_WARMUP_FRAMES):
        cap.read()
    logger.info("Camera ready ✅ — opening window")

    # ── WINDOW ────────────────────────────────────────────────────
    # WINDOW_AUTOSIZE: window is exactly DISPLAY_W×DISPLAY_H — no more, no less.
    # Mouse coordinates therefore always equal pixel positions in the frame.
    cv2.namedWindow(config.WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)

    _tracker = DragTracker()
    ctrl     = SpeedController()

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
        ret, raw_frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        frame_count += 1
        now = time.time()

        # ── RESIZE TO FIXED DISPLAY SIZE ──────────────────────────
        # This is the key fix. Everything downstream (tracker, drawing,
        # mouse) works in this one consistent coordinate space.
        display = cv2.resize(raw_frame,
                             (config.DISPLAY_W, config.DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)

        # Supply display frame to tracker for drag init
        _tracker.set_display_frame(display)

        # Update trackers — pass display frame
        pos_a, pos_b = _tracker.update(display)
        dist = pixel_distance(pos_a, pos_b)

        # ── Speed control ─────────────────────────────────────────
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

        # ── Draw and show ─────────────────────────────────────────
        draw_overlay(display, _tracker, pos_a, pos_b,
                     dist, zone, speed, ctrl.user_speed,
                     ble.connected, paused)
        cv2.imshow(config.WINDOW_TITLE, display)

        # ── Keyboard ──────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('a'), ord('A')):
            _tracker.reselect_a()
            logger.info("Re-select: drag box on Train A")
        elif key in (ord('b'), ord('B')):
            _tracker.reselect_b()
            logger.info("Re-select: drag box on Train B")
        elif key in (ord('p'), ord('P')):
            paused = not paused
            ctrl.reset()
            logger.info(f"{'PAUSED' if paused else 'RESUMED'}")
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