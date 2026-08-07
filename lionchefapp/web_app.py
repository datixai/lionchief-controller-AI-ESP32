# ══════════════════════════════════════════════════════════════════
#  web_app.py  —  LionChief Web Interface
#  Project 3  |  Datix AI  |  July 2026
#
#  Replaces main.py with a web-based interface.
#  All existing files (train_detector, ble_controller, etc.) unchanged.
#
#  Run:   python web_app.py
#  Open:  http://localhost:5000
# ══════════════════════════════════════════════════════════════════

import cv2
import time
import json
import logging
import sys
import io
import threading
import os

from flask import Flask, Response, render_template, request, jsonify
from flask_socketio import SocketIO

import config
from train_detector   import (DragTracker, pixel_distance,
                               WAIT_TABLE, WAIT_A, WAIT_B, TRACKING)
from speed_controller import SpeedController, Zone
from ble_controller   import DualBLEController

# ── Logging ───────────────────────────────────────────────────────
_out = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(message)s",
    datefmt="%H:%M:%S",
    handlers=[logging.StreamHandler(_out)]
)
logger = logging.getLogger("WebApp")

# ── Flask app ─────────────────────────────────────────────────────
import os as _os
_base = _os.path.dirname(_os.path.abspath(__file__))
app = Flask(__name__)
app.config['SECRET_KEY'] = 'lionchief2026'
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

# ══════════════════════════════════════════════════════════════════
#  SHARED STATE — accessed by camera thread and Flask routes
# ══════════════════════════════════════════════════════════════════
state = {
    'frame':          None,      # latest JPEG bytes
    'running':        False,     # camera loop active
    'manual_mode':    False,
    'auto_paused':    False,
    'waiting_confirm':False,
    'zone':           'UNKNOWN',
    'dist':           None,
    'spd_a':          0,
    'spd_b':          0,
    'conf_a':         0.0,
    'conf_b':         0.0,
    'ble_a':          False,
    'ble_b':          False,
    'instruction':    'Draw a box around the TABLE area first',
    'tracker':        None,
    'ctrl':           None,
    'ble':            None,
    'a_chasing':      False,
    'last_danger_t':  0.0,
    'resume_time':    0.0,
    'escape_frames':  0,
    'last_a_cmd_t':   0.0,
}
state_lock = threading.Lock()

SELECTION_STATES = (WAIT_TABLE, WAIT_A, WAIT_B)
DANGER_COOLDOWN  = 0.6
RESUME_GRACE     = 2.0
ESCAPE_CONFIRM   = 15

# ══════════════════════════════════════════════════════════════════
#  OVERLAY DRAWING — same logic as main.py draw_overlay
# ══════════════════════════════════════════════════════════════════

import math
import numpy as np

def draw_tracking_circle(frame, pos, color, label):
    if pos is None:
        return
    cx, cy, r = pos.x, pos.y, pos.radius
    locked = getattr(pos, 'locked', False)
    if locked:
        segs = 16
        for i in range(segs):
            if i % 2 == 0:
                a1 = 2*math.pi*i/segs
                a2 = 2*math.pi*(i+1)/segs
                p1 = (int(cx+r*math.cos(a1)), int(cy+r*math.sin(a1)))
                p2 = (int(cx+r*math.cos(a2)), int(cy+r*math.sin(a2)))
                cv2.line(frame, p1, p2, color, 2)
        cv2.circle(frame, (cx, cy), 5, color, -1)
    else:
        ov = frame.copy()
        cv2.circle(ov, (cx,cy), r, color, -1)
        cv2.addWeighted(ov, 0.08, frame, 0.92, 0, frame)
        cv2.circle(frame, (cx,cy), r, color, 2)
        cv2.line(frame,(cx-r,cy),(cx-r+8,cy),color,2)
        cv2.line(frame,(cx+r,cy),(cx+r-8,cy),color,2)
        cv2.line(frame,(cx,cy-r),(cx,cy-r+8),color,2)
        cv2.line(frame,(cx,cy+r),(cx,cy+r-8),color,2)
        cv2.circle(frame,(cx,cy),3,color,-1)
    lbl = f"{label} {'STOPPED' if locked else ''}".strip()
    cv2.putText(frame, lbl, (cx+r+6, cy-6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2)


def draw_frame_overlay(display, tracker, pos_a, pos_b,
                       dist, zone, spd_a, spd_b,
                       manual_mode, a_chasing, waiting_confirm,
                       auto_paused):
    h, w = display.shape[:2]
    zcol = config.ZONE_COLORS.get(zone, (120,120,120))

    # Table rect tint
    if tracker._table_rect:
        x1,y1,x2,y2 = tracker._table_rect
        ov = display.copy()
        ov[y1:y2, x1:x2] = display[y1:y2, x1:x2]
        cv2.rectangle(display,(x1,y1),(x2,y2),(0,200,200),1)

    # Tracking circles
    draw_tracking_circle(display, pos_a, (180,180,180), "BLUE")
    draw_tracking_circle(display, pos_b, (0,165,255),   f"ORANGE {spd_b}")

    # Gap line between train circles
    if pos_a and pos_b and dist is not None:
        cv2.line(display,(pos_a.x,pos_a.y),(pos_b.x,pos_b.y),zcol,1)
        mx=(pos_a.x+pos_b.x)//2; my=(pos_a.y+pos_b.y)//2
        cv2.putText(display,f"{dist:.0f}px | {zone}",
                    (mx+6,my-6),cv2.FONT_HERSHEY_SIMPLEX,0.52,zcol,2)

    # Live drag rect
    if tracker.is_dragging and tracker.drag_start and tracker.drag_end:
        x1=min(tracker.drag_start[0],tracker.drag_end[0])
        y1=min(tracker.drag_start[1],tracker.drag_end[1])
        x2=max(tracker.drag_start[0],tracker.drag_end[0])
        y2=max(tracker.drag_start[1],tracker.drag_end[1])
        dcol = {WAIT_TABLE:(0,200,200),WAIT_A:(180,180,180),WAIT_B:(0,165,255)}
        col = dcol.get(tracker.state,(200,200,200))
        cv2.rectangle(display,(x1,y1),(x2,y2),col,2)
        lbl_map={WAIT_TABLE:"TABLE",WAIT_A:"Train A",WAIT_B:"Train B"}
        cv2.putText(display,lbl_map.get(tracker.state,""),(x1+4,y1+18),
                    cv2.FONT_HERSHEY_SIMPLEX,0.55,col,2)

    # Top status bar
    if tracker.state in SELECTION_STATES:
        sc={WAIT_TABLE:(0,200,200),WAIT_A:(180,180,180),WAIT_B:(0,165,255)}
        col = sc.get(tracker.state,(200,200,200))
        cv2.rectangle(display,(0,0),(w,46),(25,15,0),-1)
        cv2.putText(display, tracker.instruction_text(),
                    (8,28),cv2.FONT_HERSHEY_SIMPLEX,0.62,col,2)
    else:
        if zone==Zone.ESCAPE:       barc=(55,0,55)
        elif zone==Zone.DANGER:     barc=(0,0,85)
        elif zone==Zone.SAFE:       barc=(0,52,0)
        elif manual_mode:           barc=(45,28,0)
        else:                       barc=(22,22,22)
        cv2.rectangle(display,(0,0),(w,46),barc,-1)

        if auto_paused:
            msg="AUTO-PAUSED -- tracker lost, re-select BLUE or ORANGE"
            mcol=(0,80,255)
        elif manual_mode:
            msg=f"MANUAL MODE -- BLUE:{spd_a}/7  ORANGE:{spd_b}/7"
            mcol=(0,200,255)
        elif zone==Zone.ESCAPE:
            msg=f"BLUE BEHIND ORANGE -- ORANGE escaping (spd:{spd_b})"
            mcol=(200,80,200)
        elif waiting_confirm:
            msg="Both trains selected -- click Start AUTO or Start MANUAL below"
            mcol=(0,220,50)
        else:
            msgs={
                Zone.DANGER: f"STOP -- gap too small",
                Zone.WARNING:f"WARNING -- slowing ORANGE (spd:{spd_b})",
                Zone.CAUTION:f"CAUTION -- ORANGE spd:{spd_b}",
                Zone.SAFE:   f"SAFE -- BLUE:{spd_a}  ORANGE:{spd_b}",
                Zone.FAR:    f"FAR -- ORANGE catching up (spd:{spd_b})",
                Zone.UNKNOWN:f"Tracking...",
            }
            msg=msgs.get(zone,zone); mcol=zcol
        cv2.putText(display,msg,(8,26),
                    cv2.FONT_HERSHEY_SIMPLEX,0.62,(255,255,255),2)

    # Flash confirmations
    now = time.time()
    flashes=[
        (tracker.flash_table,"Table saved -- select BLUE Train",(0,200,200)),
        (tracker.flash_a,    "BLUE Train locked -- select ORANGE Train",(180,180,180)),
        (tracker.flash_b,    "ORANGE Train locked -- both tracking!",(0,165,255)),
    ]
    for t,msg,col in flashes:
        if now-t<2.5:
            fy=h//2-28
            cv2.rectangle(display,(0,fy),(w,fy+50),(15,15,15),-1)
            cv2.rectangle(display,(0,fy),(w,fy+50),col,3)
            cv2.putText(display,f"  [OK]  {msg}",
                        (20,fy+34),cv2.FONT_HERSHEY_SIMPLEX,0.82,col,2)
            break

    return display


# ══════════════════════════════════════════════════════════════════
#  CAMERA LOOP — runs in background thread
# ══════════════════════════════════════════════════════════════════

def camera_loop():
    logger.info("Camera thread starting...")

    # Try every camera index and backend combination.
    # Also verify frames are not black - camera can open() successfully
    # but return black frames when another app holds it, or previous
    # session crashed without releasing it, or resolution not supported.
    cap = None
    for idx in [config.CAMERA_INDEX, 0, 1, 2]:
        for backend in [cv2.CAP_DSHOW, cv2.CAP_ANY]:
            try:
                _c = cv2.VideoCapture(idx, backend)
                if not _c.isOpened():
                    _c.release()
                    continue
                _c.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
                _c.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
                _c.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)
                # Read 8 test frames and check brightness
                live = False
                for _ in range(8):
                    ret, fr = _c.read()
                    if ret and fr is not None and fr.mean() > 2.0:
                        live = True
                        break
                if live:
                    cap = _c
                    logger.info(f"Camera OK: index={idx} backend={backend}")
                    break
                else:
                    logger.warning(
                        f"Camera idx={idx} backend={backend}: "
                        "opens but frames are BLACK -- trying next")
                    _c.release()
            except Exception as e:
                logger.warning(f"Camera idx={idx}: {e}")
        if cap:
            break

    if cap is None:
        msg = ("Camera black or unavailable. Close Camera app, Teams, "
               "Zoom, or any other app using the camera. "
               "Unplug and replug the USB camera. Then refresh this page.")
        logger.error(msg)
        socketio.emit('camera_error', {'msg': msg})
        return

    logger.info("Warming up camera...")
    for _ in range(config.CAMERA_WARMUP_FRAMES):
        cap.read()
    logger.info("Camera ready")

    tracker = DragTracker()
    ctrl    = SpeedController()

    with state_lock:
        state['tracker'] = tracker
        state['ctrl']    = ctrl
        state['running'] = True

    last_ka    = time.time()
    prev_tstate = tracker.state

    while state['running']:
        ret, raw = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        now = time.time()
        display = cv2.resize(raw,(config.DISPLAY_W,config.DISPLAY_H),
                             interpolation=cv2.INTER_LINEAR)
        tracker.set_display_frame(display)
        pos_a, pos_b = tracker.update(display)

        a_chasing = tracker.a_is_chasing_b(pos_a, pos_b)
        dist      = tracker.facing_gap(pos_a, pos_b, a_chasing)
        conf_a    = tracker.confidence_a()
        conf_b    = tracker.confidence_b()
        ble       = state['ble']

        # Always update BLE status every frame — not gated on tracking state
        if ble:
            with state_lock:
                state['ble_a'] = ble.connected_a
                state['ble_b'] = ble.connected_b

        # SAFETY NET: always stop trains if critically close, regardless of any other state
        if (ble and dist is not None and
                dist < config.DISTANCE_DANGER and
                now - state['last_danger_t'] >= 0.5):
            ble.train_b.send_stop_raw()
            ble.train_a.send_stop_raw()
            with state_lock: state['last_danger_t'] = now
            logger.warning(f"SAFETY NET -- gap={dist:.0f}px both stopped")

        # Detect all 4 boxes selected → ask Y/N via web
        if (tracker.state == TRACKING and
                prev_tstate == WAIT_B and
                not state['waiting_confirm']):
            with state_lock:
                state['waiting_confirm'] = True
            socketio.emit('trains_ready', {})
            logger.info("Waiting for Y/N from browser")

        prev_tstate = tracker.state

        # Speed control
        spd_a = state['spd_a']
        spd_b = state['spd_b']
        zone  = Zone.UNKNOWN

        if (tracker.ready and not state['auto_paused'] and ble
                and state.get('mode_chosen', False)):

            # Only pause if tracker is truly lost (not just locked/stopped)
            a_miss = tracker.is_a_missing()
            b_miss = tracker.is_b_missing()
            # Extra check: if tracker is active, don't pause even if is_missing fires
            a_ok = (pos_a is not None) or tracker._tkr_a.is_locked
            b_ok = (pos_b is not None) or tracker._tkr_b.is_locked
            truly_lost = (a_miss and not a_ok) or (b_miss and not b_ok)

            if truly_lost:
                if not state['auto_paused']:
                    with state_lock:
                        state['auto_paused'] = True
                    logger.warning("AUTO-PAUSED -- tracker truly lost")
                    socketio.emit('auto_paused', {})
            elif state['auto_paused']:
                with state_lock:
                    state['auto_paused'] = False
                    state['resume_time'] = now
                ctrl.reset()
                logger.info("AUTO-RESUMED")
                socketio.emit('auto_resumed', {})

            if not state['manual_mode'] and not state['auto_paused']:
                eff = None if (a_miss or b_miss) else dist
                speed_b, zone = ctrl.update(eff, a_chasing)

                if zone == Zone.DANGER:
                    in_grace = (now - state['resume_time']) < RESUME_GRACE
                    if not in_grace and (now - state['last_danger_t']) >= DANGER_COOLDOWN:
                        if ble: ble.train_b.send_stop_raw()
                        with state_lock:
                            state['last_danger_t'] = now
                        if ble and (ble.train_a.current_speed != ble.train_a.user_speed
                                    and now - state['last_a_cmd_t'] > 0.5):
                            ble.train_a.set_speed(ble.train_a.user_speed)
                            with state_lock: state['last_a_cmd_t'] = now
                    spd_a = ble.train_a.current_speed if ble else 0
                    spd_b = 0

                elif zone == Zone.ESCAPE:
                    with state_lock:
                        state['escape_frames'] = min(state['escape_frames']+1, ESCAPE_CONFIRM+1)
                    if ctrl.should_send_command(speed_b) and ble:
                        ble.train_b.set_speed(speed_b)
                        ctrl.command_sent(speed_b)
                    if state['escape_frames'] >= ESCAPE_CONFIRM and ble:
                        desired_a = max(1, ble.train_a.user_speed - config.ESCAPE_SLOW_A_BY)
                        if (ble.train_a.current_speed != desired_a
                                and now - state['last_a_cmd_t'] > 1.0):
                            ble.train_a.set_speed_no_save(desired_a)
                            with state_lock: state['last_a_cmd_t'] = now
                    spd_a = ble.train_a.current_speed if ble else 0
                    spd_b = speed_b

                else:
                    with state_lock: state['escape_frames'] = 0
                    if ctrl.should_send_command(speed_b) and ble:
                        ble.train_b.set_speed(speed_b)
                        ctrl.command_sent(speed_b)
                    if ble and (ble.train_a.current_speed != ble.train_a.user_speed
                                and now - state['last_a_cmd_t'] > 0.5):
                        ble.train_a.set_speed(ble.train_a.user_speed)
                        with state_lock: state['last_a_cmd_t'] = now
                    spd_a = ble.train_a.current_speed if ble else 0
                    spd_b = speed_b
            else:
                spd_a = ble.train_a.current_speed if ble else 0
                spd_b = ble.train_b.current_speed if ble else 0

        # Keepalive
        if now - last_ka >= config.KEEPALIVE_INTERVAL:
            last_ka = now
            if ble: ble.keepalive()

        # Draw overlay
        draw_frame_overlay(
            display, tracker, pos_a, pos_b,
            dist, zone, spd_a, spd_b,
            state['manual_mode'], a_chasing,
            state['waiting_confirm'], state['auto_paused'])

        # Encode JPEG
        _, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 80])

        # Update shared state
        with state_lock:
            state['frame']    = jpeg.tobytes()
            state['zone']     = zone
            state['dist']     = dist
            state['spd_a']    = spd_a
            state['spd_b']    = spd_b
            state['conf_a']   = conf_a
            state['conf_b']   = conf_b
            state['a_chasing']= a_chasing
            if ble:
                state['ble_a'] = ble.connected_a
                state['ble_b'] = ble.connected_b
            state['instruction'] = tracker.instruction_text()

        # Push status to browser
        socketio.emit('status', {
            'zone':     zone,
            'dist':     f"{dist:.0f}" if dist is not None else '---',
            'spd_a':    spd_a,
            'spd_b':    spd_b,
            'conf_a':   round(conf_a*100),
            'conf_b':   round(conf_b*100),
            'ble_a':    state['ble_a'],
            'ble_b':    state['ble_b'],
            'manual':   state['manual_mode'],
            'paused':   state['auto_paused'],
            'waiting':  state['waiting_confirm'],
            'mode_chosen': state.get('mode_chosen', False),
            'step':     tracker.state,
            'a_chasing':a_chasing,
        })

    cap.release()
    logger.info("Camera thread stopped")


# ══════════════════════════════════════════════════════════════════
#  FLASK ROUTES
# ══════════════════════════════════════════════════════════════════

@app.route('/')
def index():
    from flask import send_from_directory
    return send_from_directory(_base, 'index.html')

@app.route('/static/<path:filename>')
def static_files(filename):
    from flask import send_from_directory
    return send_from_directory(_base, filename)


@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            with state_lock:
                frame = state['frame']
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n'
                       + frame + b'\r\n')
            time.sleep(0.033)   # ~30fps
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/mouse', methods=['POST'])
def mouse_event():
    """Receive mouse drag events from browser for box selection."""
    data = request.json
    event = data.get('event')
    x     = int(data.get('x', 0))
    y     = int(data.get('y', 0))
    with state_lock:
        tracker = state['tracker']
    if tracker is None:
        return jsonify({'ok': False, 'msg': 'not ready'})
    if   event == 'down': tracker.on_mouse_down(x, y)
    elif event == 'move': tracker.on_mouse_move(x, y)
    elif event == 'up':   tracker.on_mouse_up(x, y)
    return jsonify({'ok': True})


@app.route('/api/command', methods=['POST'])
def command():
    """Handle all button commands from the browser."""
    data = request.json
    cmd  = data.get('cmd')
    val  = data.get('val', None)

    with state_lock:
        tracker = state['tracker']
        ctrl    = state['ctrl']
        ble     = state['ble']

    if tracker is None:
        return jsonify({'ok': False})

    # ── Confirmation ──────────────────────────────────────────────
    if cmd == 'start_auto':
        with state_lock:
            state['waiting_confirm'] = False
            state['manual_mode']     = False
            state['mode_chosen']     = True
        if ble: ble.train_a.set_speed(config.DEFAULT_SPEED_A)
        logger.info("AUTO mode started from browser")

    elif cmd == 'start_manual':
        with state_lock:
            state['waiting_confirm'] = False
            state['manual_mode']     = True
            state['mode_chosen']     = True
        logger.info("MANUAL mode started from browser")

    # ── Mode ──────────────────────────────────────────────────────
    elif cmd == 'toggle_manual':
        with state_lock:
            state['manual_mode'] = not state['manual_mode']
            state['mode_chosen'] = True
        if ctrl: ctrl.reset()
        if not state['manual_mode'] and ble:
            ble.train_a.set_speed(ble.train_a.user_speed)

    # ── Emergency ─────────────────────────────────────────────────
    elif cmd == 'stop_both':
        # Force stop both trains directly - belt and suspenders approach
        if ble:
            ble.train_a.send_stop_raw()
            ble.train_b.send_stop_raw()
            ble.train_a.send_stop_raw()   # send twice for reliability
            ble.train_b.send_stop_raw()
            logger.warning("STOP BOTH -- dashboard button")
        if ctrl: ctrl.reset()
        with state_lock:
            state['spd_a'] = 0
            state['spd_b'] = 0

    # ── Train A controls ──────────────────────────────────────────
    elif cmd == 'a_slower':
        if ble:
            ble.train_a.set_speed(max(1, ble.train_a.user_speed - 1))
            with state_lock: state['spd_a'] = ble.train_a.current_speed
    elif cmd == 'a_faster':
        if ble:
            ble.train_a.set_speed(min(7, ble.train_a.user_speed + 1))
            with state_lock: state['spd_a'] = ble.train_a.current_speed
    elif cmd == 'a_stop':
        if ble: ble.train_a.send_stop()
    elif cmd == 'a_resume':
        if ble: ble.train_a.resume()

    # ── Train B controls ──────────────────────────────────────────
    elif cmd == 'b_speed':
        spd = max(1, min(7, int(val or 1)))
        if ctrl: ctrl.set_user_speed(spd)
        if ble:  ble.train_b.set_speed(spd)
        with state_lock:
            state['spd_b'] = spd
    elif cmd == 'b_slower':
        if ctrl and ble:
            ctrl.set_user_speed(max(1, ctrl.user_speed - 1))
            ble.train_b.set_speed(ctrl.user_speed)
    elif cmd == 'b_faster':
        if ctrl and ble:
            ctrl.set_user_speed(min(7, ctrl.user_speed + 1))
            ble.train_b.set_speed(ctrl.user_speed)
    elif cmd == 'b_stop':
        if ble: ble.train_b.send_stop()
        if ctrl: ctrl.reset()
    elif cmd == 'b_resume':
        if ble and ctrl: ble.train_b.set_speed(ctrl.user_speed)

    # ── Tracker ───────────────────────────────────────────────────
    elif cmd == 'reselect_a':
        tracker.reselect_a()
    elif cmd == 'reselect_b':
        tracker.reselect_b()
    elif cmd == 'redraw_table':
        tracker.redraw_table()

    elif cmd == 'a_speed':
        spd = max(1, min(7, int(val or 1)))
        if ble: ble.train_a.set_speed(spd)
        with state_lock: state['spd_a'] = spd

    elif cmd == 'horn_a_on':
        if ble: ble.train_a.horn_on()
    elif cmd == 'horn_a_off':
        if ble: ble.train_a.horn_off()
    elif cmd == 'horn_b_on':
        if ble: ble.train_b.horn_on()
    elif cmd == 'horn_b_off':
        if ble: ble.train_b.horn_off()
    elif cmd == 'bell_a_on':
        if ble: ble.train_a.bell_on()
    elif cmd == 'bell_a_off':
        if ble: ble.train_a.bell_off()
    elif cmd == 'bell_b_on':
        if ble: ble.train_b.bell_on()
    elif cmd == 'bell_b_off':
        if ble: ble.train_b.bell_off()
    elif cmd == 'bell_both_on':
        if ble: ble.train_a.bell_on(); ble.train_b.bell_on()
    elif cmd == 'bell_both_off':
        if ble: ble.train_a.bell_off(); ble.train_b.bell_off()

    elif cmd == 'a_reverse':
        if ble: ble.train_a.set_reverse()
        logger.info("Train A REVERSE")
    elif cmd == 'a_forward':
        if ble: ble.train_a.set_forward()
        logger.info("Train A FORWARD")
    elif cmd == 'b_reverse':
        if ble: ble.train_b.set_reverse()
        logger.info("Train B REVERSE")
    elif cmd == 'b_forward':
        if ble: ble.train_b.set_forward()
        logger.info("Train B FORWARD")

    # ── Accessories ───────────────────────────────────────────────
    elif cmd == 'horn_both_on':
        if ble: ble.train_a.horn_on(); ble.train_b.horn_on()
    elif cmd == 'horn_both_off':
        if ble: ble.train_a.horn_off(); ble.train_b.horn_off()

    return jsonify({'ok': True})


@app.route('/api/restart', methods=['POST'])
def restart_app():
    """Stop trains, spawn a fresh process, then kill this one."""
    import subprocess, signal
    def _restart():
        time.sleep(0.4)
        ble = state.get('ble')
        if ble:
            try:
                ble.train_a.send_stop_raw()
                ble.train_b.send_stop_raw()
                time.sleep(0.3)
                ble.shutdown()
            except Exception:
                pass
        # Spawn a helper that waits 2s then relaunches us
        script = (
            f"import time, subprocess; "
            f"time.sleep(2); "
            f"subprocess.Popen([r'{sys.executable}', r'{_os.path.abspath(__file__)}'], "
            f"cwd=r'{_os.path.dirname(_os.path.abspath(__file__))}')"
        )
        subprocess.Popen([sys.executable, '-c', script])
        _os.kill(_os.getpid(), signal.SIGTERM)
    import threading as _t
    _t.Thread(target=_restart, daemon=True).start()
    return jsonify({'ok': True})


@app.route('/api/quit', methods=['POST'])
def quit_app():
    """Stop both trains and shut down the server."""
    def _shutdown():
        time.sleep(0.4)
        ble = state.get('ble')
        if ble:
            try:
                ble.train_a.send_stop_raw()
                ble.train_b.send_stop_raw()
                time.sleep(0.3)
                ble.shutdown()
            except Exception:
                pass
        import os, signal
        os.kill(os.getpid(), signal.SIGTERM)
    import threading as _t
    _t.Thread(target=_shutdown, daemon=True).start()
    return jsonify({'ok': True})


@app.route('/api/status')
def get_status():
    with state_lock:
        return jsonify({
            'zone':    state['zone'],
            'dist':    f"{state['dist']:.0f}" if state['dist'] else '---',
            'spd_a':   state['spd_a'],
            'spd_b':   state['spd_b'],
            'ble_a':   state['ble_a'],
            'ble_b':   state['ble_b'],
            'manual':  state['manual_mode'],
            'paused':  state['auto_paused'],
            'waiting': state['waiting_confirm'],
        })


# ══════════════════════════════════════════════════════════════════
#  STARTUP
# ══════════════════════════════════════════════════════════════════

def start_services():
    # BLE
    ble = DualBLEController()
    ble.start()
    with state_lock:
        state['ble'] = ble

    # Camera thread
    cam_thread = threading.Thread(target=camera_loop, daemon=True, name="Camera")
    cam_thread.start()
    logger.info("Services started")


if __name__ == '__main__':
    print("\n+--------------------------------------------------+")
    print("|  LionChief Web Interface  |  Datix AI  |  2026  |")
    print("+--------------------------------------------------+")
    print("|  Open browser:  http://localhost:5000            |")
    print("+--------------------------------------------------+\n")

    start_services()
    socketio.run(app, host='0.0.0.0', port=5000, debug=False,
                 allow_unsafe_werkzeug=True)