"""
main.py — Raspberry Pi Collision Prevention System
====================================================
Combines:
  - AI camera detection (YOLO / MobileNet / TFLite)
  - BLE train control (bleak)
  - Zone-based collision logic

Usage:
    python main.py                         # uses settings in config.py
    python main.py --mock                  # mock detector, no camera
    python main.py --model yolo_ncnn       # override model type
    python main.py --mac AA:BB:CC:DD:EE:FF # override train MAC

Pipeline:
    Camera → Detector → ZoneManager → BLE stop/go → Train
"""

import asyncio
import argparse
import logging
import os
import sys
import time
import cv2

# ── Setup path so script runs from any directory ──────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config
from lionchief.controller import LionChiefController
from detection.detector    import create_detector
from detection.zone_manager import Zone, ZoneManager


# ── Logging setup ─────────────────────────────────────────────────────────────
os.makedirs("logs", exist_ok=True)
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(config.LOG_FILE),
    ],
)
logger = logging.getLogger("main")


# ── Argument parsing ──────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="LionChief Collision Prevention")
    p.add_argument("--mock",  action="store_true", help="Use mock detector (no camera)")
    p.add_argument("--model", default=None,
                   choices=["yolo", "yolo_ncnn", "tflite", "mock"],
                   help="Override model type from config")
    p.add_argument("--mac",   default=None,
                   help="Override train MAC address")
    p.add_argument("--no-display", action="store_true",
                   help="Disable live video window")
    return p.parse_args()


# ── Main async loop ───────────────────────────────────────────────────────────
async def run(args):
    # ── Config overrides from CLI ─────────────────────────────────────────────
    mac        = args.mac   or config.TRAIN_MAC_ADDRESS or None
    model_type = args.model or ("mock" if args.mock else config.MODEL_TYPE)
    show_disp  = config.SHOW_DISPLAY and not args.no_display

    logger.info("=" * 60)
    logger.info("  LionChief Collision Prevention — Raspberry Pi")
    logger.info("=" * 60)
    logger.info(f"  Model type  : {model_type}")
    logger.info(f"  Train MAC   : {mac or '(auto-discover)'}")
    logger.info(f"  Resume speed: {config.RESUME_SPEED}")
    logger.info(f"  Resume delay: {config.RESUME_DELAY}s")
    logger.info(f"  Shared zone : {config.SHARED_ZONE}")
    logger.info("=" * 60)

    # ── Build model path based on type ────────────────────────────────────────
    model_kwargs = {}
    if model_type == "yolo":
        model_kwargs = {"model_path": config.MODEL_PATH_PT,
                        "confidence": config.DETECTION_CONFIDENCE}
    elif model_type == "yolo_ncnn":
        model_kwargs = {"model_path": config.MODEL_PATH_NCNN,
                        "confidence": config.DETECTION_CONFIDENCE}
    elif model_type == "tflite":
        model_kwargs = {"model_path": config.MODEL_PATH_TFLITE,
                        "labels_path": config.LABELS_PATH,
                        "confidence": config.DETECTION_CONFIDENCE}
    elif model_type == "mock":
        model_kwargs = {
            "trigger_every_n": config.MOCK_TRIGGER_EVERY_N_FRAMES,
            "zone_x1": config.SHARED_ZONE[0],
            "zone_y1": config.SHARED_ZONE[1],
            "zone_x2": config.SHARED_ZONE[2],
            "zone_y2": config.SHARED_ZONE[3],
        }

    # ── Load detector ─────────────────────────────────────────────────────────
    detector = create_detector(model_type, **model_kwargs)
    detector.load()

    # ── Build zone manager ────────────────────────────────────────────────────
    x1, y1, x2, y2 = config.SHARED_ZONE
    zone_mgr = ZoneManager([
        Zone(x1, y1, x2, y2, name="shared_section")
    ])

    # ── Connect to train ──────────────────────────────────────────────────────
    train = LionChiefController(mac_address=mac, auto_connect=True)
    connected = await train.connect()
    if not connected:
        if model_type == "mock":
            logger.warning("Running in mock mode — BLE not connected, simulating commands")
        else:
            logger.error("Could not connect to train. Check MAC address and BLE.")
            logger.error("Tip: Run with --mock to test without train hardware.")
            # Continue anyway — might reconnect later

    # ── Open camera ───────────────────────────────────────────────────────────
    cap = None
    if model_type != "mock":
        cap = cv2.VideoCapture(config.CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)
        if not cap.isOpened():
            logger.error("Could not open camera. Check CAMERA_INDEX in config.py")
            sys.exit(1)
        logger.info(f"Camera opened (index={config.CAMERA_INDEX})")

    # ── State tracking ────────────────────────────────────────────────────────
    train_in_zone  = False
    frame_count    = 0
    fps_time       = time.time()

    logger.info("🚂 Monitoring started. Press Q in window or Ctrl+C to stop.")

    try:
        while True:
            # ── Get frame (real or dummy) ─────────────────────────────────────
            if cap is not None:
                ret, frame = cap.read()
                if not ret:
                    logger.error("Camera read failed")
                    break
            else:
                # Mock mode: create blank frame
                frame = cv2.UMat(cv2.Mat(
                    cv2.resize(
                        cv2.imencode('.jpg',
                            cv2.cvtColor(
                                cv2.UMat(
                                    (config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3),
                                    cv2.CV_8UC3
                                ),
                                cv2.COLOR_RGB2BGR
                            )
                        )[1], (config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
                    )
                )) if False else (
                    cv2.cvtColor(
                        cv2.resize(
                            cv2.imread("/dev/null") if False
                            else cv2.UMat(
                                (config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3),
                                cv2.CV_8UC3
                            ).get(),
                            (config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
                        ), cv2.COLOR_RGB2BGR
                    )
                )
                # Simpler: just use numpy
                import numpy as np
                frame = np.zeros(
                    (config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3), dtype=np.uint8
                )

            frame_count += 1

            # ── Run detection ─────────────────────────────────────────────────
            detections = detector.detect(frame)
            det_dicts  = [d.as_dict() for d in detections]

            # ── Check zones ───────────────────────────────────────────────────
            zone_triggered = zone_mgr.any_zone_triggered(det_dicts)

            # ── Collision logic ───────────────────────────────────────────────
            if zone_triggered and not train_in_zone:
                # Inner train just entered shared zone → stop outer train
                logger.warning("⚠️  INNER TRAIN IN ZONE — stopping outer train!")
                await train.stop()
                train_in_zone = True

            elif not zone_triggered and train_in_zone:
                # Zone just cleared → wait safety delay, then resume
                logger.info(f"✅ Zone clear — resuming in {config.RESUME_DELAY}s ...")
                await asyncio.sleep(config.RESUME_DELAY)
                await train.resume(speed=config.RESUME_SPEED)
                train_in_zone = False

            # ── Draw debug overlay ────────────────────────────────────────────
            if show_disp:
                # Draw zones
                frame = zone_mgr.draw_zones(frame)
                # Draw detections
                for det in detections:
                    color = (0, 0, 255) if zone_triggered else (0, 255, 0)
                    det.draw(frame, color=color)
                # Status text
                status = "🛑 STOPPED" if train_in_zone else "✅ RUNNING"
                fps = frame_count / max(1, time.time() - fps_time)
                cv2.putText(frame, f"{status}  |  FPS:{fps:.1f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255) if train_in_zone else (0, 255, 0), 2)
                cv2.imshow("Train Monitor — Press Q to quit", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

            # Yield to allow BLE keepalive task to run
            await asyncio.sleep(0)

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    finally:
        logger.info("Shutting down...")
        await train.stop()
        await asyncio.sleep(0.5)
        await train.disconnect()
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        detector.close()
        logger.info("Shutdown complete.")


def main():
    args = parse_args()
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
