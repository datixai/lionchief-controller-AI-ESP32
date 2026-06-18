# ══════════════════════════════════════════════════════════════════
#  config.py  —  Single Track Safe Distance System
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  Two trains on one outer loop track.
#  Train A (front) runs freely — no BLE control.
#  Train B (rear)  is BLE-controlled — speed adjusted by camera.
#
#  Camera detects both trains by COLORED STICKERS on their roofs:
#    Train A → one solid color  (e.g. bright RED sticker)
#    Train B → different color  (e.g. bright YELLOW sticker)
#
#  Run calibrate.py FIRST to set correct color ranges and distances.
#  All values saved to calibration.json and loaded automatically.
# ══════════════════════════════════════════════════════════════════

# ── TRAIN B BLE (rear train — the one we control) ─────────────────
#  Train A runs freely — no BLE needed for it.
#  Change TRAIN_B_MAC when Peter gets the second LionChief.
TRAIN_B_MAC  = "60:F9:FB:49:94:C9"   # rear BLE train MAC
TRAIN_B_NAME = "LC-1-1-09F9-60F9"    # name prefix fallback

# LionChief BLE UUIDs — same across all LionChief models
SERVICE_UUID        = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
CHARACTERISTIC_UUID = "08590f7e-db05-467e-8757-72f6faeb13d4"

# ── ALL BLE SPEED COMMANDS ────────────────────────────────────────
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
CAMERA_INDEX  = 1      # USB ceiling camera (0=built-in, 1=USB)
CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30

# ── CALIBRATION FILE ──────────────────────────────────────────────
# Written by calibrate.py — read automatically by all other files.
# Contains color ranges for Train A and B, and pixel distance zones.
CALIBRATION_FILE = "calibration.json"

# ── DEFAULT COLOR RANGES (HSV) ────────────────────────────────────
# These are fallback defaults if calibration.json does not exist.
# Run calibrate.py to set correct values for your lighting.
#
# HSV ranges for RED (wraps around in HSV — needs two ranges):
TRAIN_A_HSV_LOWER1 = [0,   120, 80]    # red range 1
TRAIN_A_HSV_UPPER1 = [10,  255, 255]
TRAIN_A_HSV_LOWER2 = [170, 120, 80]    # red range 2 (wrap-around)
TRAIN_A_HSV_UPPER2 = [180, 255, 255]
TRAIN_A_USES_DUAL  = True              # True only for red sticker

# HSV ranges for YELLOW:
TRAIN_B_HSV_LOWER  = [20, 100, 100]
TRAIN_B_HSV_UPPER  = [35, 255, 255]
TRAIN_B_USES_DUAL  = False

# ── DETECTION SETTINGS ────────────────────────────────────────────
# Trains look SMALL from ceiling — keep area threshold low
MIN_BLOB_AREA    = 150   # Minimum pixel area to count as a train
                          # Increase if getting false detections
                          # Decrease if real train stickers not found

BLUR_KERNEL_SIZE = 5     # Pre-blur to reduce noise (odd number)

# How many frames to average position over (smooths jitter)
POSITION_SMOOTH_FRAMES = 3

# ── DISTANCE ZONES (pixels) ───────────────────────────────────────
# ★ TUNE THESE after calibration using calibrate.py
# These define how far apart the trains appear in your camera image.
# Actual pixel values depend on camera height and zoom.
#
#  DANGER  ← stop immediately
#  WARNING ← slow to min speed
#  CAUTION ← reduce speed
#  SAFE    ← run at user speed
#  FAR     ← speed up to catch (train fell behind)
#
DISTANCE_DANGER  = 80    # pixels — STOP Train B
DISTANCE_WARNING = 150   # pixels — slow to FOLLOW_MIN_SPEED
DISTANCE_CAUTION = 230   # pixels — slow to mid speed
DISTANCE_SAFE    = 320   # pixels — run at user speed
DISTANCE_FAR     = 450   # pixels — speed up to catch up

# Hysteresis offsets — prevents speed oscillation at zone boundaries
# Zone exit threshold is offset + zone entry threshold
HYSTERESIS_OFFSET = 20   # pixels

# ── SPEED SETTINGS ────────────────────────────────────────────────
DEFAULT_SPEED      = 5   # Starting speed for Train B
MAX_CATCH_SPEED    = 7   # Max speed when gap is FAR (catching up)
FOLLOW_MIN_SPEED   = 2   # Min speed in WARNING zone (not full stop)
CAUTION_SPEED      = 3   # Speed in CAUTION zone

# ── SPEED CONTROL TUNING ──────────────────────────────────────────
# Smoothing alpha: how fast commanded speed changes toward target
# Higher = more responsive but jerkier
# Lower = smoother but slower to react
ALPHA_SLOW_DOWN = 0.7    # Faster response when slowing (safety)
ALPHA_SPEED_UP  = 0.25   # Slower response when speeding up (smooth)

# Minimum ms between BLE speed command sends (prevents BLE flooding)
MIN_COMMAND_INTERVAL_MS = 300

# ── MISSING TRAIN SAFETY ──────────────────────────────────────────
# If a train is not detected for this many seconds → slow Train B
MISSING_TIMEOUT_S  = 2.0
MISSING_SAFE_SPEED = 2    # Speed while train is not visible

# ── BLE TIMING ────────────────────────────────────────────────────
RECONNECT_INTERVAL = 5.0   # seconds between reconnect attempts
KEEPALIVE_INTERVAL = 20.0  # seconds between keepalive pings

# ── DISPLAY ───────────────────────────────────────────────────────
SHOW_VIDEO   = True
WINDOW_TITLE = "LionChief — Safe Distance Control"

# Zone bar colors (BGR)
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
