# ══════════════════════════════════════════════════════════════════
#  config.py  —  LionChief Laptop CV  —  All Settings
#
#  Zone coordinates are stored in shared_zone.json (separate file).
#  Run calibrate_zone.py once to create shared_zone.json.
# ══════════════════════════════════════════════════════════════════

# ── TRAIN BLE CONNECTION ──────────────────────────────────────────
#
#  TWO WAYS TO CONNECT:
#
#  Mode A — Direct MAC (recommended, faster):
#    Set TRAIN_MAC to the train's MAC address.
#    Code connects directly without scanning.
#
#  Mode B — Auto-discover by name:
#    Set TRAIN_MAC = "" to leave empty.
#    Code scans BLE devices and connects to first device whose
#    name starts with TRAIN_NAME prefix "LC0".
#
# Peter's confirmed train MAC (from Android nRF Connect):
TRAIN_MAC  = "CC:01:78:D0:F0:99"   # ← Mode A (leave empty for Mode B)
TRAIN_NAME = "LC0"                  # ← name prefix used in Mode B only

# LionChief BLE UUIDs — confirmed via Peter's nRF Connect scan
SERVICE_UUID        = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
CHARACTERISTIC_UUID = "08590f7e-db05-467e-8757-72f6faeb13d4"

# ── BLE COMMANDS ──────────────────────────────────────────────────
CMD_STOP   = bytes([0x00, 0x45, 0x00])   # Speed 0 — immediate stop
CMD_RESUME = bytes([0x00, 0x45, 0x07])   # Speed 7 — resume at medium-fast

# ── CAMERA ────────────────────────────────────────────────────────
CAMERA_INDEX  = 0      # 0=built-in webcam, 1=first USB, 2=second USB
CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30

# ── ZONE FILE ─────────────────────────────────────────────────────
# Created by calibrate_zone.py — read automatically by all other files
ZONE_FILE = "shared_zone.json"

# ── DETECTION SETTINGS ────────────────────────────────────────────
MIN_DETECTION_AREA         = 800   # min pixel area of moving blob to count as train
DETECTION_FRAMES_THRESHOLD = 3     # consecutive frames needed before triggering
BG_HISTORY                 = 500   # frames used to build background model
BG_THRESHOLD               = 25    # lower = more sensitive to motion
BG_DETECT_SHADOW           = False

# ── SAFETY TIMING ─────────────────────────────────────────────────
RESUME_DELAY_SECONDS = 2.5   # wait after zone clears before resuming train
RECONNECT_INTERVAL   = 5.0   # seconds between BLE reconnect attempts

# ── DISPLAY ───────────────────────────────────────────────────────
SHOW_VIDEO        = True
WINDOW_TITLE      = "LionChief — Zone Detection"
ZONE_COLOR_SAFE   = (0, 255, 0)   # Green = zone clear
ZONE_COLOR_ACTIVE = (0, 0, 255)   # Red   = train detected

# ── LOGGING ───────────────────────────────────────────────────────
LOG_TO_FILE   = True
LOG_FILE_PATH = "detection_log.txt"
