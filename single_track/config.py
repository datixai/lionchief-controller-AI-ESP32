# ══════════════════════════════════════════════════════════════════
#  config.py  —  Single Track Safe Distance
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  Train A = front train  (manual, no BLE)
#  Train B = rear train   (BLE controlled)
#
#  HOW TO SELECT TRAINS:
#    Camera opens → HOLD and DRAG a box around Train A → release
#    Then HOLD and DRAG a box around Train B → release
#    System starts tracking and controlling automatically
# ══════════════════════════════════════════════════════════════════

# ── TRAIN B BLE ───────────────────────────────────────────────────
TRAIN_B_MAC  = "60:F9:FB:49:94:C9"
TRAIN_B_NAME = "LC-1-1-09F9-60F9"

SERVICE_UUID        = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
CHARACTERISTIC_UUID = "08590f7e-db05-467e-8757-72f6faeb13d4"

CMD_STOP     = bytes([0x00, 0x45, 0x00])
CMD_SPEED_1  = bytes([0x00, 0x45, 0x01])
CMD_SPEED_2  = bytes([0x00, 0x45, 0x02])
CMD_SPEED_3  = bytes([0x00, 0x45, 0x03])
CMD_SPEED_4  = bytes([0x00, 0x45, 0x04])
CMD_SPEED_5  = bytes([0x00, 0x45, 0x05])
CMD_SPEED_6  = bytes([0x00, 0x45, 0x06])
CMD_SPEED_7  = bytes([0x00, 0x45, 0x07])
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
    CMD_STOP,    CMD_SPEED_1, CMD_SPEED_2, CMD_SPEED_3,
    CMD_SPEED_4, CMD_SPEED_5, CMD_SPEED_6, CMD_SPEED_7,
]

# ── CAMERA ────────────────────────────────────────────────────────
CAMERA_INDEX  = 1
CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30

# ── TRACKER ───────────────────────────────────────────────────────
MIN_BOX_SIZE           = 15    # minimum drag size in pixels to accept
POSITION_SMOOTH_FRAMES = 3     # rolling average frames for smooth position

# ── DISTANCE ZONES (pixels) ───────────────────────────────────────
# Adjust these after seeing your actual camera image.
# Run calibrate.py to set them interactively.
DISTANCE_DANGER  = 80
DISTANCE_WARNING = 150
DISTANCE_CAUTION = 230
DISTANCE_SAFE    = 320
DISTANCE_FAR     = 450
HYSTERESIS_OFFSET = 20

CALIBRATION_FILE = "calibration.json"

# ── SPEED ─────────────────────────────────────────────────────────
DEFAULT_SPEED      = 5
MAX_CATCH_SPEED    = 7
FOLLOW_MIN_SPEED   = 2
CAUTION_SPEED      = 3

# ── SPEED SMOOTHING ───────────────────────────────────────────────
ALPHA_SLOW_DOWN         = 0.7
ALPHA_SPEED_UP          = 0.25
MIN_COMMAND_INTERVAL_MS = 300

# ── SAFETY ────────────────────────────────────────────────────────
MISSING_TIMEOUT_S  = 3.0
MISSING_SAFE_SPEED = 2

# ── BLE ───────────────────────────────────────────────────────────
RECONNECT_INTERVAL = 5.0
KEEPALIVE_INTERVAL = 20.0

# ── DISPLAY ───────────────────────────────────────────────────────
SHOW_VIDEO   = True
WINDOW_TITLE = "LionChief — Safe Distance Control"

ZONE_COLORS = {
    "DANGER":  (0,   0, 200),
    "WARNING": (0,  80, 220),
    "CAUTION": (0, 140, 255),
    "SAFE":    (0, 180,  60),
    "FAR":     (200, 160, 0),
    "UNKNOWN": (120, 120, 120),
}

# ── LOGGING ───────────────────────────────────────────────────────
LOG_TO_FILE = True
LOG_FILE    = "safe_distance_log.txt"
