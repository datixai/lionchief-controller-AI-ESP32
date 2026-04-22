# ══════════════════════════════════════════════════════════════════
#  config.py  —  LionChief Laptop CV  —  All Settings
#
#  Three-zone collision prevention matching ESP32 IR sensor logic:
#    Zone A → inner loop approach track (before shared section)
#    Zone B → outer loop approach track (before shared section)
#    Zone C → shared section / exit area
#
#  Run calibrate_zone.py once to draw all three zones.
#  Zones are saved to zones.json automatically.
# ══════════════════════════════════════════════════════════════════

# ── TRAIN BLE CONNECTION ──────────────────────────────────────────
#  Mode A — Direct MAC (recommended):
#    TRAIN_MAC set → connects directly, no scan
#  Mode B — Auto-discover:
#    TRAIN_MAC = "" → scans for device starting with TRAIN_NAME
TRAIN_MAC  = "CC:01:78:D0:F0:99"   # Peter's confirmed MAC
TRAIN_NAME = "LC0"                  # name prefix fallback

# LionChief BLE UUIDs — confirmed via nRF Connect
SERVICE_UUID        = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
CHARACTERISTIC_UUID = "08590f7e-db05-467e-8757-72f6faeb13d4"

# ── BLE COMMANDS ──────────────────────────────────────────────────
CMD_STOP     = bytes([0x00, 0x45, 0x00])   # immediate stop
CMD_SPEED_2  = bytes([0x00, 0x45, 0x02])   # slow resume speed
CMD_RESUME   = bytes([0x00, 0x45, 0x07])   # normal resume speed

# ── CAMERA ────────────────────────────────────────────────────────
CAMERA_INDEX  = 1      # 0=built-in, 1=first USB, 2=second USB
CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30

# ── ZONE FILE ─────────────────────────────────────────────────────
# Written by calibrate_zone.py, read by zone_detector.py + main.py
ZONES_FILE = "zones.json"

# ── DETECTION SETTINGS ────────────────────────────────────────────
# Minimum pixel area — a train is large, ignore anything smaller
# Increase if hands/shadows still trigger. Decrease only if train is tiny.
MIN_DETECTION_AREA = 2500

# How many consecutive frames must confirm detection before triggering
# Higher = slower response but far fewer false triggers
DETECTION_FRAMES_THRESHOLD = 5

# Background subtractor — learns the empty layout
BG_HISTORY       = 300    # frames to build model (lower = adapts faster)
BG_THRESHOLD     = 50     # higher = LESS sensitive to subtle changes
BG_DETECT_SHADOW = False

# Reference frame comparison — catches STOPPED trains
# MOG2 alone misses trains that stop (learns them as background)
# We also compare against a fixed reference frame taken when zone was empty
REFERENCE_DIFF_THRESHOLD = 35    # pixel brightness difference to count as changed
REFERENCE_MIN_CHANGED    = 0.04  # fraction of zone pixels that must change (4%)

# ── ZONE LOCK ─────────────────────────────────────────────────────
# Once a train enters a zone, lock stays active until train exits via Zone C
# This prevents false "clear" when train briefly disappears from detection
ZONE_LOCK_ENABLED = True

# Repeat STOP command every N seconds while zone is locked
# Prevents outer train from creeping in if first BLE message was missed
STOP_REPEAT_INTERVAL = 0.5

# ── SAFETY TIMING ─────────────────────────────────────────────────
RESUME_DELAY_SECONDS = 2.5    # wait after zone clears before RESUME
SPEED_RAMP_DELAY     = 3.0    # seconds at slow speed before full speed
RECONNECT_INTERVAL   = 5.0    # BLE reconnect attempt interval

# ── DISPLAY ───────────────────────────────────────────────────────
SHOW_VIDEO   = True
WINDOW_TITLE = "LionChief — Collision Prevention"

# Zone display colors (BGR)
COLOR_ZONE_A_SAFE   = (0, 165, 255)   # Orange = inner approach clear
COLOR_ZONE_A_ACTIVE = (0, 0, 255)     # Red    = inner approach occupied
COLOR_ZONE_B_SAFE   = (255, 100, 0)   # Blue   = outer approach clear
COLOR_ZONE_B_ACTIVE = (0, 0, 255)     # Red    = outer approach occupied
COLOR_ZONE_C_SAFE   = (0, 220, 0)     # Green  = shared section clear
COLOR_ZONE_C_ACTIVE = (0, 0, 255)     # Red    = shared section occupied

# ── LOGGING ───────────────────────────────────────────────────────
LOG_TO_FILE   = True
LOG_FILE_PATH = "detection_log.txt"
