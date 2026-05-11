"""
cam.py — Live Track Camera Viewer
Run alongside pio device monitor to watch the layout.

Usage:
  python cam.py              (auto-detects COM port)
  python cam.py --port COM4  (specify port)
  python cam.py --camera 0   (change camera index)

Keys:
  Q / ESC = Quit
  S       = Save screenshot
  F       = Fullscreen
"""

import cv2
import serial
import serial.tools.list_ports
import threading
import time
import argparse
import os
import datetime
import sys

# ── CONFIG ─────────────────────────────────────────────────────────
CAMERA_INDEX  = 1       # change to 0 if wrong camera opens
SERIAL_BAUD   = 115200
MAX_LOG_LINES = 20

# ── STATE ──────────────────────────────────────────────────────────
log_lines = []
log_lock  = threading.Lock()


def find_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if any(k in desc for k in ["silicon labs", "cp210", "ch340", "uart"]):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


def serial_reader(port):
    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=1)
        print(f"[CAM] Serial connected: {port}")
    except Exception as e:
        print(f"[CAM] Serial failed: {e} — camera-only mode")
        return

    while True:
        try:
            raw  = ser.readline()
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            with log_lock:
                log_lines.append((time.time(), line))
                if len(log_lines) > MAX_LOG_LINES * 2:
                    log_lines.pop(0)
        except Exception:
            time.sleep(0.5)


def draw_overlay(frame):
    h, w = frame.shape[:2]

    # Top bar
    cv2.rectangle(frame, (0, 0), (w, 38), (15, 15, 15), -1)
    cv2.putText(frame, "LionChief — Live Track View  |  Datix AI",
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 180, 255), 1)
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    cv2.putText(frame, ts, (w - 90, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

    # Log panel bottom-left
    panel_w = min(620, w - 20)
    panel_h = MAX_LOG_LINES * 17 + 18
    panel_y = h - panel_h - 30

    overlay = frame.copy()
    cv2.rectangle(overlay, (6, panel_y), (panel_w + 6, h - 30),
                  (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)
    cv2.rectangle(frame, (6, panel_y), (panel_w + 6, h - 30), (60, 60, 60), 1)
    cv2.putText(frame, "ESP32 Log", (12, panel_y + 13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)

    with log_lock:
        recent = log_lines[-MAX_LOG_LINES:]

    for i, (_, text) in enumerate(recent):
        if any(k in text for k in ["ERROR", "❌"]):
            col = (60, 60, 255)
        elif any(k in text for k in ["✅", "RESUME", "UNLOCKED", "RUNNING"]):
            col = (60, 220, 60)
        elif any(k in text for k in ["STOP", "LOCKED", "power cut"]):
            col = (60, 60, 255)
        elif "SENSOR" in text:
            col = (0, 200, 255)
        elif "BLE" in text:
            col = (200, 155, 60)
        elif "RELAY" in text:
            col = (200, 80, 255)
        else:
            col = (185, 185, 185)

        display = text[:85] if len(text) > 85 else text
        cv2.putText(frame, display, (12, panel_y + 26 + i * 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, col, 1)

    # Bottom bar
    cv2.rectangle(frame, (0, h - 26), (w, h), (15, 15, 15), -1)
    cv2.putText(frame, "Q=Quit   S=Screenshot   F=Fullscreen",
                (12, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 100), 1)

    return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",   default=None)
    parser.add_argument("--camera", default=CAMERA_INDEX, type=int)
    args = parser.parse_args()

    port   = args.port or find_port()
    cam_id = args.camera

    print(f"[CAM] Camera:{cam_id}  Serial:{port or 'not found'}")

    if port:
        threading.Thread(target=serial_reader, args=(port,), daemon=True).start()

    cap = cv2.VideoCapture(cam_id, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print(f"❌ Cannot open camera {cam_id}")
        sys.exit(1)

    WIN = "LionChief — Track Camera"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 720)
    fullscreen     = False
    screenshot_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "screenshots")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        frame = draw_overlay(frame)
        cv2.imshow(WIN, frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('s'), ord('S')):
            os.makedirs(screenshot_dir, exist_ok=True)
            fname = os.path.join(screenshot_dir,
                f"track_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")
            cv2.imwrite(fname, frame)
            print(f"[CAM] Screenshot saved: {fname}")
        elif key in (ord('f'), ord('F')):
            fullscreen = not fullscreen
            cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()