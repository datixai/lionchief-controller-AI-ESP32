# ══════════════════════════════════════════════════════════════════
#  config.py  —  LionChief Laptop CV  —  All Settings
#
#  Zone coordinates are stored in shared_zone.json (separate file)
#  Run calibrate_zone.py to create/update shared_zone.json
# ══════════════════════════════════════════════════════════════════

# ── TRAIN BLE ─────────────────────────────────────────────────────
# Peter's train MAC address — leave empty to auto-find by name
TRAIN_MAC  = ""
TRAIN_NAME = "LC0"

# LionChief BLE UUIDs (confirmed by Peter's nRF Connect scan)
SERVICE_UUID        = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
CHARACTERISTIC_UUID = "08590f7e-db05-467e-8757-72f6faeb13d4"

# BLE Commands
CMD_STOP   = bytes([0x00, 0x45, 0x00])
CMD_RESUME = bytes([0x00, 0x45, 0x07])

# ── CAMERA ────────────────────────────────────────────────────────
# 0 = built-in webcam, 1 = first USB camera, 2 = second USB camera
CAMERA_INDEX  = 0
CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30

# ── ZONE FILE ─────────────────────────────────────────────────────
# Shared zone coordinates are saved here by calibrate_zone.py
# All other files read from this file automatically
ZONE_FILE = "shared_zone.json"

# ── DETECTION SETTINGS ────────────────────────────────────────────
# Minimum pixel area to count as a train (filters out noise)
MIN_DETECTION_AREA = 800

# How many consecutive frames must detect movement before triggering
DETECTION_FRAMES_THRESHOLD = 3

# Background subtractor settings
BG_HISTORY       = 500
BG_THRESHOLD     = 25
BG_DETECT_SHADOW = False

# ── SAFETY TIMING ─────────────────────────────────────────────────
# Seconds to wait after zone clears before sending RESUME
RESUME_DELAY_SECONDS = 2.5

# Seconds between BLE reconnect attempts if connection drops
RECONNECT_INTERVAL = 5.0

# ── DISPLAY ───────────────────────────────────────────────────────
SHOW_VIDEO   = True
WINDOW_TITLE = "LionChief — Zone Detection"
SHOW_ZONE    = True
ZONE_COLOR_SAFE   = (0, 255, 0)    # Green when clear
ZONE_COLOR_ACTIVE = (0, 0, 255)    # Red when train detected

# ── LOGGING ───────────────────────────────────────────────────────
LOG_TO_FILE   = True
LOG_FILE_PATH = "detection_log.txt"
