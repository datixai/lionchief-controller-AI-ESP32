"""
config.py — All project settings in one place
==============================================
Edit THIS file to configure the system.

TWO WAYS TO CONNECT TO THE TRAIN:
  Mode A — Manual MAC (recommended):
    Set TRAIN_MAC_ADDRESS to the confirmed MAC below.
    Fastest — connects directly without scanning.

  Mode B — Auto-discover by name:
    Set TRAIN_MAC_ADDRESS = "" (empty string).
    System scans BLE devices and connects to the first device
    whose name starts with TRAIN_NAME_PREFIX = "LC0".
"""

import os

# ── TRAIN BLE CONNECTION ──────────────────────────────────────────────────────

# Mode A: Direct MAC connection (recommended)
# Peter's confirmed train — found via Android nRF Connect
TRAIN_MAC_ADDRESS = "CC:01:78:D0:F0:99"
# Set to "" to use Mode B (auto-scan by name):
# TRAIN_MAC_ADDRESS = ""

# Mode B: Auto-discover by name prefix (used when MAC is empty)
TRAIN_NAME_PREFIX = "LC0"    # matches "LC015556-99F0" and any LionChief

# LionChief BLE UUIDs — confirmed via nRF Connect
TRAIN_SERVICE_UUID = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"

# Speed to resume outer train at after inner train clears the zone (0–31)
RESUME_SPEED = 7

# Safety delay in seconds after zone clears before resuming
RESUME_DELAY = 2.5

# ── CAMERA ───────────────────────────────────────────────────────────────────
CAMERA_INDEX  = 0     # 0 = first USB camera, 1 = second, etc.
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 30

# Show live video window with zone overlay
SHOW_DISPLAY = True   # set False when running headless on Pi

# ── AI MODEL ─────────────────────────────────────────────────────────────────
# Options: "yolo", "yolo_ncnn", "tflite", "mock"
#   mock      — no camera, no model — pure simulation (start here)
#   yolo      — PC/dev with GPU or fast CPU
#   yolo_ncnn — Raspberry Pi 5 (NCNN format, fastest on ARM CPU)
#   tflite    — Pi Zero (MobileNetV2, lightest)
MODEL_TYPE = "mock"   # ← change to "yolo_ncnn" when deploying on Pi

MODEL_PATH_PT     = "models/best.pt"            # PyTorch — PC/dev
MODEL_PATH_NCNN   = "models/best_ncnn_model"    # NCNN folder — Pi 5
MODEL_PATH_TFLITE = "models/model.tflite"       # TFLite — Pi Zero
LABELS_PATH       = "models/labels.txt"

DETECTION_CONFIDENCE = 0.50  # minimum detection confidence (0.0–1.0)

# ── SHARED ZONE (pixel coordinates) ──────────────────────────────────────────
# Set by running:  python calibrate_zone.py
# Format: (x1, y1, x2, y2)
SHARED_ZONE = (150, 150, 490, 330)

# ── LOGGING ───────────────────────────────────────────────────────────────────
LOG_LEVEL = "INFO"
LOG_FILE  = "logs/train_monitor.log"

# ── MOCK DETECTOR ─────────────────────────────────────────────────────────────
MOCK_TRIGGER_EVERY_N_FRAMES = 90   # fake detection every N frames (mock mode)
