"""
cam.py — Live Track Camera Viewer
Usage: python cam.py --camera 1
Q / ESC = Quit  |  S = Screenshot  |  F = Fullscreen
"""

import cv2
import argparse
import os
import datetime
import sys

CAMERA_INDEX = 1   # change if wrong camera

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default=CAMERA_INDEX, type=int)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print(f"Cannot open camera {args.camera}")
        sys.exit(1)

    WIN = "LionChief — Track Camera"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 720)
    fullscreen = False
    screenshot_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "screenshots")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        cv2.imshow(WIN, frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('s'), ord('S')):
            os.makedirs(screenshot_dir, exist_ok=True)
            fname = os.path.join(screenshot_dir,
                f"track_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")
            cv2.imwrite(fname, frame)
            print(f"Screenshot saved: {fname}")
        elif key in (ord('f'), ord('F')):
            fullscreen = not fullscreen
            cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()