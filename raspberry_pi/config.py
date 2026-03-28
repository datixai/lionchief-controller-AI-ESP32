"""
config.py — All project settings in one file
=============================================
Edit THIS file to configure your system.
"""

import os

# ── Train BLE Settings ────────────────────────────────────────────────────────
# Peter's confirmed Service UUID — already verified
TRAIN_SERVICE_UUID = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"

# MAC address of the OUTER LOOP train (the one we stop on collision)
# Peter needs to find this using nRF Connect app while train is powered on
# Format: "AA:BB:CC:DD:EE:FF"
TRAIN_MAC_ADDRESS = os.environ.get("TRAIN_MAC", "")   # set via env or hardcode below
# TRAIN_MAC_ADDRESS = "AA:BB:CC:DD:EE:FF"            # ← uncomment and paste MAC here

# Speed to resume at after inner train clears the zone
RESUME_SPEED = 7   # 0–31

# Safety delay (seconds) after zone clears before resuming outer train
RESUME_DELAY = 2.5

# ── Camera Settings ───────────────────────────────────────────────────────────
CAMERA_INDEX    = 0          # 0 = first USB camera, 1 = second, etc.
CAMERA_WIDTH    = 640
CAMERA_HEIGHT   = 480
CAMERA_FPS      = 30

# Show live debug window with zone overlay and detections
SHOW_DISPLAY    = True       # set False when running headless on Pi

# ── AI Model Settings ─────────────────────────────────────────────────────────
# Options: "yolo", "yolo_ncnn", "tflite", "mock"
#   yolo      — use on PC / development (needs GPU or fast CPU)
#   yolo_ncnn — use on Raspberry Pi (NCNN format, fastest on ARM CPU)
#   tflite    — use on Pi Zero (MobileNetV2, lightest)
#   mock      — no model, no camera — pure simulation for testing
MODEL_TYPE = "mock"   # ← change to "yolo_ncnn" when deploying on Pi

# Paths — drop your trained model files in the models/ folder
MODEL_PATH_PT    = "models/best.pt"              # PyTorch — for PC
MODEL_PATH_NCNN  = "models/best_ncnn_model"      # NCNN folder — for Pi
MODEL_PATH_TFLITE= "models/model.tflite"         # TFLite — for Pi Zero
LABELS_PATH      = "models/labels.txt"

# Detection confidence threshold (0.0–1.0)
DETECTION_CONFIDENCE = 0.50

# ── Shared Zone (Region of Interest) ─────────────────────────────────────────
# Pixel coordinates of the shared track section in the camera frame.
# Run python calibrate_zone.py to set these interactively with your mouse.
# Format: (x1, y1, x2, y2) — top-left and bottom-right corners
SHARED_ZONE = (150, 150, 490, 330)   # ← calibrate for your camera angle!

# ── Logging ───────────────────────────────────────────────────────────────────
LOG_LEVEL = "INFO"   # DEBUG / INFO / WARNING / ERROR
LOG_FILE  = "logs/train_monitor.log"

# ── Mock Detector Settings ────────────────────────────────────────────────────
# Only used when MODEL_TYPE = "mock"
MOCK_TRIGGER_EVERY_N_FRAMES = 90   # fake a detection every N frames
