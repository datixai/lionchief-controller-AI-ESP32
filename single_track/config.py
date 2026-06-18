# ══════════════════════════════════════════════════════════════════
#  config.py  —  Single Track Click-to-Track System
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  No stickers. No colors. No calibration for colors.
#  Peter clicks directly on each train in the live camera feed.
#  OpenCV CSRT tracker follows each train frame by frame.
#
#  ONLY THINGS TO TUNE:
#    CLICK_BOX_SIZE   — size of tracking box around click
#    DISTANCE_*       — pixel gap thresholds for speed zones
#    TRAIN_B_MAC      — rear BLE train Bluetooth address
# ══════════════════════════════════════════════════════════════════

# ── TRAIN B BLE (rear train — the one we control) ─────────────────
TRAIN_B_MAC  = "60:F9:FB:49:94:C9"
TRAIN_B_NAME = "LC-1-1-09F9-60F9"

SERVICE_UUID        = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
CHARACTERISTIC_UUID = "08590f7e-db05-467e-8757-72f6faeb13d4"

CMD_STOP    = bytes([0x00, 0x45, 0x00])
CMD_SPEED_1 = bytes([0x00, 0x45, 0x01])
CMD_SPEED_2 = bytes([0x00, 0x45, 0x02])
CMD_SPEED_3 = bytes([0x00, 0x45, 0x03])
CMD_SPEED_4 = bytes([0x00, 0x45, 0x04])
CMD_SPEED_5 = bytes([0x00, 0x45, 0x05])
CMD_SPEED_6 = bytes([0x00, 0x45, 0x06])
CMD_SPEED_7 = bytes([0x00, 0x45, 0x07])
CMD_FORWARD  = bytes([0x00, 0x46, 0x01])
CMD_REVERSE  = bytes([0x00, 0x46, 0x02])
CMD_HORN_ON  = bytes([0x00, 0x48, 0x01])
CMD_HORN_OFF = bytes([0x00, 0x48, 0x00])
CMD_BELL_ON  = bytes([0x00, 0x47, 0x01])
CMD_BELL_OFF = bytes([0x00, 0x47, 0x00])
CMD_LIGHT_ON = bytes([0x00, 0x51, 0x01])
CMD_LIGHT_OFF= bytes([0x00, 0x51, 0x00])
CMD_SOUND_ON = bytes([0x00, 0x4C, 0x07])
CMD_SOUND_OFF= bytes([0x00, 0x4C, 0x00])
CMD_ANNOUNCE = bytes([0x00, 0x4D, 0x00, 0x00])

SPEED_CMDS = [
    CMD_STOP, CMD_SPEED_1, CMD_SPEED_2, CMD_SPEED_3,
    CMD_SPEED_4, CMD_SPEED_5, CMD_SPEED_6, CMD_SPEED_7,
]

# ── CAMERA ────────────────────────────────────────────────────────
CAMERA_INDEX  = 1
CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30

# ── CLICK TRACKER ─────────────────────────────────────────────────
# Size of the tracking bounding box (pixels) placed around each click.
# ★ TUNE THIS: increase if tracker loses train too often,
#              decrease if it picks up surrounding objects instead.
CLICK_BOX_SIZE = 40       # width and height of tracking box in pixels

# Smoothing: rolling average of last N positions (reduces jitter)
POSITION_SMOOTH_FRAMES = 3

# ── DISTANCE ZONES (pixels) ───────────────────────────────────────
# Set these after seeing how big the gap looks on your camera.
# Use calibrate.py to measure and set them interactively.
DISTANCE_DANGER  = 80
DISTANCE_WARNING = 150
DISTANCE_CAUTION = 230
DISTANCE_SAFE    = 320
DISTANCE_FAR     = 450
HYSTERESIS_OFFSET = 20

# Calibration file — written by calibrate.py, read on startup
CALIBRATION_FILE = "calibration.json"

# ── SPEED SETTINGS ────────────────────────────────────────────────
DEFAULT_SPEED      = 5
MAX_CATCH_SPEED    = 7
FOLLOW_MIN_SPEED   = 2
CAUTION_SPEED      = 3

# ── SPEED CONTROL ─────────────────────────────────────────────────
ALPHA_SLOW_DOWN          = 0.7
ALPHA_SPEED_UP           = 0.25
MIN_COMMAND_INTERVAL_MS  = 300

# ── MISSING TRAIN SAFETY ──────────────────────────────────────────
# If tracker loses a train for this many seconds → slow Train B
MISSING_TIMEOUT_S  = 3.0
MISSING_SAFE_SPEED = 2

# ── BLE TIMING ────────────────────────────────────────────────────
RECONNECT_INTERVAL = 5.0
KEEPALIVE_INTERVAL = 20.0

# ── DISPLAY ───────────────────────────────────────────────────────
SHOW_VIDEO   = True
WINDOW_TITLE = "LionChief — Click to Track"

ZONE_COLORS = {
    "DANGER":  (0,   0,  200),
    "WARNING": (0,  80,  220),
    "CAUTION": (0, 140,  255),
    "SAFE":    (0, 180,   60),
    "FAR":     (200, 160,  0),
    "UNKNOWN": (120, 120, 120),
}

# ── LOGGING ───────────────────────────────────────────────────────
LOG_TO_FILE = True
LOG_FILE    = "safe_distance_log.txt"
