# ══════════════════════════════════════════════════════════════════
#  main.py  —  Single Track Safe Distance v5.0
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
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
        logger.warning(f"Calibration load failed: {e}")


_tracker = None

def mouse_callback(event, x, y, flags, param):
    if _tracker is None:
        return
    if event == cv2.EVENT_LBUTTONDOWN: _tracker.on_mouse_down(x, y)
    elif event == cv2.EVENT_MOUSEMOVE: _tracker.on_mouse_move(x, y)
    elif event == cv2.EVENT_LBUTTONUP: _tracker.on_mouse_up(x, y)


def draw_overlay(display, tracker, pos_a, pos_b,
                 dist, zone, speed, user_speed, ble_ok, paused):
    h, w = display.shape[:2]
    now  = time.time()

    # ── Track mask tint ───────────────────────────────────────────
    mask = tracker.get_mask()
    if mask is not None:
        outside = display.copy()
        outside[mask == 0] = (0, 0, 40)
        cv2.addWeighted(outside, 0.25, display, 0.75, 0, display)

    # ── MOG2 detected blobs ───────────────────────────────────────
    for blob in tracker.get_blobs():
        cv2.circle(display, (blob["cx"], blob["cy"]),
                   5, (100, 100, 100), 1)

    # ── Train bounding box, head & tail ──────────────────────────
    for pos, main_col, lbl in [
        (pos_a, (0, 165, 255), "A  front"),
        (pos_b, (0, 220,  50), f"B  spd:{speed}"),
    ]:
        if pos is None:
            continue
        # Full bounding box
        if pos.bbox:
            bx, by, bw, bh = pos.bbox
            cv2.rectangle(display,(bx,by),(bx+bw,by+bh), main_col, 2)

        # Centre dot
        cv2.circle(display,(pos.x,pos.y), 5, main_col, -1)
        cv2.putText(display, lbl, (pos.x+10, pos.y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, main_col, 2)

        # Head box (direction of travel) — filled circle
        if pos.head:
            cv2.circle(display, pos.head, 9, main_col, -1)
            cv2.putText(display, "H",
                        (pos.head[0]-5, pos.head[1]+4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,0,0), 1)

        # Tail box — hollow circle
        if pos.tail:
            cv2.circle(display, pos.tail, 9, (120,120,120), 2)
            cv2.putText(display, "T",
                        (pos.tail[0]-4, pos.tail[1]+4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120,120,120), 1)

        # Velocity arrow
        if pos.vel and (abs(pos.vel[0]) > 0.5 or abs(pos.vel[1]) > 0.5):
            vx, vy = pos.vel
            scale = 8.0
            ex = int(pos.x + vx * scale)
            ey = int(pos.y + vy * scale)
            cv2.arrowedLine(display, (pos.x,pos.y), (ex,ey),
                            main_col, 1, tipLength=0.4)

    # ── Live drag rect ────────────────────────────────────────────
    if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
        x1=min(tracker.drag_start[0],tracker.drag_end[0])
        y1=min(tracker.drag_start[1],tracker.drag_end[1])
        x2=max(tracker.drag_start[0],tracker.drag_end[0])
        y2=max(tracker.drag_start[1],tracker.drag_end[1])
        dcol=(0,165,255) if tracker.state==WAIT_A else (0,220,50)
        cv2.rectangle(display,(x1,y1),(x2,y2),dcol,2)
        lbl="Train A" if tracker.state==WAIT_A else "Train B"
        cv2.putText(display,lbl,(x1+4,max(y1-6,14)),
                    cv2.FONT_HERSHEY_SIMPLEX,0.55,dcol,2)

    # ── Distance line ─────────────────────────────────────────────
    if pos_a and pos_b and dist:
        zcol = config.ZONE_COLORS.get(zone,(150,150,150))
        cv2.line(display,(pos_a.x,pos_a.y),(pos_b.x,pos_b.y),zcol,2)
        mx=(pos_a.x+pos_b.x)//2; my=(pos_a.y+pos_b.y)//2
        cv2.putText(display,f"{dist:.0f}px | {zone}",
                    (mx+6,my-6),cv2.FONT_HERSHEY_SIMPLEX,0.55,zcol,2)

    # ── Top status bar ────────────────────────────────────────────
    if tracker.state in (WAIT_A, WAIT_B):
        cv2.rectangle(display,(0,0),(w,52),(40,25,0),-1)
        col = (0,165,255) if tracker.state==WAIT_A else (0,220,50)
        msg = ("STEP 1:  Hold + DRAG a box around  TRAIN A  (front train)"
               if tracker.state==WAIT_A else
               "STEP 2:  Hold + DRAG a box around  TRAIN B  (rear BLE train)")
        cv2.putText(display,msg,(8,28),cv2.FONT_HERSHEY_SIMPLEX,0.62,col,2)
        cv2.putText(display,"Draw the box around the whole train body",
                    (8,46),cv2.FONT_HERSHEY_SIMPLEX,0.36,(180,180,180),1)
    else:
        zcol = config.ZONE_COLORS.get(zone,(80,80,80))
        if zone == Zone.ESCAPE:
            barc = (60,0,60)
        elif zone == Zone.DANGER:
            barc = (0,0,90)
        elif zone == Zone.SAFE:
            barc = (0,55,0)
        else:
            barc = (25,25,25)
        cv2.rectangle(display,(0,0),(w,48),barc,-1)

        if paused:
            msg  = "PAUSED — press P to resume"
            mcol = (0,200,255)
        elif zone == Zone.ESCAPE:
            msg  = f"A IS BEHIND B — Train B speeding up (speed {speed})"
            mcol = (255,100,200)
        elif not tracker.tracking_a or not tracker.tracking_b:
            msg  = "Tracker lost — press A or B then drag new box"
            mcol = (0,80,255)
        else:
            msgs = {
                Zone.DANGER:  f"STOP — gap too small",
                Zone.WARNING: f"WARNING — slowing (speed {speed})",
                Zone.CAUTION: f"CAUTION — speed {speed}",
                Zone.SAFE:    f"SAFE — following at speed {speed}",
                Zone.FAR:     f"FAR — catching up (speed {speed})",
                Zone.UNKNOWN: f"Train not visible — holding speed {speed}",
            }
            msg  = msgs.get(zone, zone)
            mcol = zcol

        cv2.putText(display,msg,(8,26),
                    cv2.FONT_HERSHEY_SIMPLEX,0.65,(255,255,255),2)
        chase_warn = "  ⚠ A CHASING B" if tracker.a_is_chasing_b else ""
        cv2.putText(display,
                    f"A:{'OK' if tracker.tracking_a else 'LOST'}  "
                    f"B:{'OK' if tracker.tracking_b else 'LOST'}  |  "
                    f"BLE:{'OK' if ble_ok else 'connecting'}  |  "
                    f"Spd:{speed}/7 usr:{user_speed}  |  "
                    f"Gap:{f'{dist:.0f}px' if dist else '---'}"
                    f"{chase_warn}",
                    (8,44),cv2.FONT_HERSHEY_SIMPLEX,0.38,(200,200,200),1)

    # ── Flash confirmation ────────────────────────────────────────
    flash = []
    if now - tracker.flash_a_time < 2.0:
        flash.append(("  Train A LOCKED  —  now drag a box around Train B",
                      (0,165,255)))
    if now - tracker.flash_b_time < 2.0:
        flash.append(("  Train B LOCKED  —  tracking started!",
                      (0,220,50)))
    for i,(msg,col) in enumerate(flash):
        fy = h//2 - 26 + i*56
        cv2.rectangle(display,(0,fy),(w,fy+50),(15,15,15),-1)
        cv2.rectangle(display,(0,fy),(w,fy+50),col,3)
        cv2.putText(display,msg,(20,fy+34),
                    cv2.FONT_HERSHEY_SIMPLEX,0.85,col,2)

    # ── Bottom bar ────────────────────────────────────────────────
    cv2.rectangle(display,(0,h-22),(w,h),(20,20,20),-1)
    cv2.putText(display,
                "DRAG=select  A/B=reselect  1-7=Speed  "
                "S=Stop  R=Resume  P=Pause  H=Horn  Q=Quit",
                (8,h-7),cv2.FONT_HERSHEY_SIMPLEX,0.35,(130,130,130),1)

    return display


def main():
    global _tracker

    print("\n╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Safe Distance Control  v5.0            ║")
    print("║  Datix AI  |  June 2026                             ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  Camera warms up → drag boxes → auto control runs   ║")
    print("║  Run calibrate.py first for track mask + distances  ║")
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

    logger.info(f"Warming up camera ({config.CAMERA_WARMUP_FRAMES} frames)...")
    for _ in range(config.CAMERA_WARMUP_FRAMES):
        cap.read()
    logger.info("Camera ready ✅")

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

    logger.info("Ready — drag a box around Train A to begin\n")

    while True:
        ret, raw = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        frame_count += 1
        now = time.time()

        display = cv2.resize(raw, (config.DISPLAY_W, config.DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)

        _tracker.set_display_frame(display)
        pos_a, pos_b = _tracker.update(display)

        # Use facing-edge gap for more accurate stopping distance
        dist = _tracker.facing_gap(pos_a, pos_b)
        a_chasing_b = _tracker.a_is_chasing_b

        if not paused and _tracker.ready:
            eff_dist = None if (
                _tracker.is_a_missing() or _tracker.is_b_missing()
            ) else dist

            speed, zone = ctrl.update(eff_dist, a_chasing_b)

            if ctrl.should_send_command(speed):
                if zone == Zone.DANGER and ble.current_speed != 0:
                    stops_sent += 1
                ble.set_speed(speed)
                ctrl.command_sent(speed)
        else:
            speed = ble.current_speed
            zone  = Zone.UNKNOWN

        if now - last_ka >= config.KEEPALIVE_INTERVAL:
            last_ka = now
            ble.keepalive()

        draw_overlay(display, _tracker, pos_a, pos_b,
                     dist, zone, speed, ctrl.user_speed,
                     ble.connected, paused)
        cv2.imshow(config.WINDOW_TITLE, display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'),ord('Q'),27): break
        elif key in (ord('a'),ord('A')): _tracker.reselect_a()
        elif key in (ord('b'),ord('B')): _tracker.reselect_b()
        elif key in (ord('p'),ord('P')): paused = not paused; ctrl.reset()
        elif key in (ord('s'),ord('S')): ble.send_stop(); ctrl.reset()
        elif key in (ord('r'),ord('R')): ble.set_speed(ctrl.user_speed); ctrl.reset()
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

    ble.shutdown()
    cap.release()
    cv2.destroyAllWindows()
    elapsed = int(time.time()-start_time)
    print(f"\n  Session: {elapsed//60:02d}m{elapsed%60:02d}s  "
          f"Frames:{frame_count}  Stops:{stops_sent}\n")

if __name__ == "__main__":
    main()