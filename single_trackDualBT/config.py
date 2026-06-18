# ══════════════════════════════════════════════════════════════════
#  config.py  —  Single Track Dual BLE Safe Distance System
#  Harry Locomotive Project 3 (BT variant)  |  Datix AI  |  June 2026
#
#  Both trains are Bluetooth-controlled on the same outer loop.
#  The camera measures the gap between them every frame.
#  Both trains' speeds are adjusted cooperatively to maintain gap.
#
#  COOPERATIVE CONTROL:
#    Gap too small → slow Train B AND optionally speed up Train A
#    Gap too large → slow Train A AND speed up Train B
#    Emergency     → STOP BOTH trains immediately
#
#  ★ UPDATE BOTH MACs BELOW before running.
# ══════════════════════════════════════════════════════════════════

# ── TRAIN A — FRONT TRAIN (BLE) ───────────────────────────────────
#  Normally runs at user-set speed.
#  System may adjust its speed slightly for cooperative gap control.
TRAIN_A_MAC  = "CC:01:78:D0:F0:99"    # ← UPDATE: front train MAC
TRAIN_A_NAME = "LC015556-99F0"         # name prefix fallback

# ── TRAIN B — REAR TRAIN (BLE) ────────────────────────────────────
#  Primary follower — speed adjusted to maintain safe gap.
TRAIN_B_MAC  = "60:F9:FB:49:94:C9"    # ← UPDATE: rear train MAC
TRAIN_B_NAME = "LC-1-1-09F9-60F9"     # name prefix fallback

# LionChief BLE UUIDs — identical across all LionChief models
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
CAMERA_INDEX  = 1
CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30

# ── CALIBRATION FILE ──────────────────────────────────────────────
CALIBRATION_FILE = "calibration.json"

# ── DEFAULT COLOR RANGES (HSV) ────────────────────────────────────
# Set correctly by calibrate.py — these are fallback defaults.
TRAIN_A_HSV_LOWER1 = [0,   120, 80]
TRAIN_A_HSV_UPPER1 = [10,  255, 255]
TRAIN_A_HSV_LOWER2 = [170, 120, 80]
TRAIN_A_HSV_UPPER2 = [180, 255, 255]
TRAIN_A_USES_DUAL  = True

TRAIN_B_HSV_LOWER  = [20, 100, 100]
TRAIN_B_HSV_UPPER  = [35, 255, 255]
TRAIN_B_USES_DUAL  = False

# ── DETECTION SETTINGS ────────────────────────────────────────────
MIN_BLOB_AREA         = 150
BLUR_KERNEL_SIZE      = 5
POSITION_SMOOTH_FRAMES= 3

# ── DISTANCE ZONES (pixels) ───────────────────────────────────────
# Set by calibrate.py — tune to your camera height and layout.
DISTANCE_DANGER  = 80
DISTANCE_WARNING = 150
DISTANCE_CAUTION = 230
DISTANCE_SAFE    = 320
DISTANCE_FAR     = 450
HYSTERESIS_OFFSET = 20

# ── SPEED SETTINGS ────────────────────────────────────────────────
DEFAULT_SPEED_A    = 5   # Train A default cruising speed
DEFAULT_SPEED_B    = 5   # Train B default cruising speed
MAX_CATCH_SPEED    = 7   # Max speed for Train B catching up
MIN_FOLLOW_SPEED   = 2   # Min speed in WARNING zone
CAUTION_SPEED      = 3   # Speed in CAUTION zone

# ── COOPERATIVE CONTROL ───────────────────────────────────────────
# When gap is too small: slow Train B AND speed up Train A by this amount
# When gap is too large: slow Train A AND speed up Train B by this amount
# Set to 0 to disable cooperative adjustment of Train A
COOPERATIVE_ADJUST_A = 1   # Speed steps to adjust Train A (+/-)

# ── SPEED SMOOTHING ───────────────────────────────────────────────
ALPHA_SLOW_DOWN          = 0.7    # Faster braking response (safety)
ALPHA_SPEED_UP           = 0.25   # Slower acceleration (smooth)
MIN_COMMAND_INTERVAL_MS  = 300    # Max BLE command rate per train

# ── MISSING TRAIN SAFETY ──────────────────────────────────────────
MISSING_TIMEOUT_S  = 2.0
MISSING_SAFE_SPEED = 2

# ── BLE TIMING ────────────────────────────────────────────────────
RECONNECT_INTERVAL = 5.0
KEEPALIVE_INTERVAL = 20.0

# ── DISPLAY ───────────────────────────────────────────────────────
SHOW_VIDEO   = True
WINDOW_TITLE = "LionChief — Dual BLE Safe Distance"

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
LOG_FILE    = "dual_bt_log.txt"
