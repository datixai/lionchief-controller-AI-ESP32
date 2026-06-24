# ══════════════════════════════════════════════════════════════════
#  config.py  —  Single Track Safe Distance v6.0
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  4-BOX SYSTEM:
#    User manually drags 4 boxes:
#      Train A HEAD (leading edge of front train)
#      Train A TAIL (trailing edge of front train)
#      Train B HEAD (leading edge of rear BLE train)
#      Train B TAIL (trailing edge of rear BLE train)
#
#    Each box has its own LOCAL SEARCH RADIUS.
#    MOG2 only looks for motion near each box.
#    People walking far away are completely ignored.
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

DISPLAY_W = 960
DISPLAY_H = 540
CAMERA_WARMUP_FRAMES = 40

# ── FILES ─────────────────────────────────────────────────────────
CALIBRATION_FILE = "calibration.json"

# ── TRACKER ───────────────────────────────────────────────────────
MIN_BOX_SIZE = 15     # minimum drag size in pixels to accept a box

# LOCAL SEARCH RADIUS — each box only looks for motion within this
# many pixels of its last known position.
# ★ KEY: keeps people walking outside this radius from affecting tracking
# ★ TUNE UP if train moves fast and tracker loses it
# ★ TUNE DOWN if nearby people still interfere
SEARCH_RADIUS = 80    # pixels in display coordinates (960×540)

MIN_BLOB_AREA = 80    # minimum blob area inside local search to count

# Velocity smoothing for each box
VELOCITY_ALPHA = 0.4

# Rolling average frames for smooth position display
POSITION_SMOOTH_FRAMES = 4

# ── MOG2 ──────────────────────────────────────────────────────────
MOG2_HISTORY       = 300
MOG2_VAR_THRESHOLD = 45   # ★ TUNE UP if false detections remain
MOG2_LEARNING_RATE = 0.005

# ── DISTANCE ZONES (pixels in 960×540 display) ────────────────────
# Distance is measured HEAD-to-TAIL between facing edges.
DISTANCE_DANGER  = 80    # STOP immediately
DISTANCE_WARNING = 130   # slow to min speed
DISTANCE_CAUTION = 190   # reduce speed
DISTANCE_SAFE    = 270   # follow at user speed
DISTANCE_FAR     = 380   # gap too large — catch up
HYSTERESIS_OFFSET = 18

CALIBRATION_FILE = "calibration.json"

# ── SPEED ─────────────────────────────────────────────────────────
DEFAULT_SPEED      = 5
MAX_CATCH_SPEED    = 7
FOLLOW_MIN_SPEED   = 2
CAUTION_SPEED      = 3
ESCAPE_MIN_SPEED   = 5   # speed when Train A is behind Train B
ESCAPE_MAX_SPEED   = 7

# ── SMOOTHING ─────────────────────────────────────────────────────
ALPHA_SLOW_DOWN         = 0.75
ALPHA_SPEED_UP          = 0.25
MIN_COMMAND_INTERVAL_MS = 300

# ── SAFETY ────────────────────────────────────────────────────────
MISSING_TIMEOUT_S  = 3.0
MISSING_SAFE_SPEED = 2

# ── BLE ───────────────────────────────────────────────────────────
RECONNECT_INTERVAL = 5.0
KEEPALIVE_INTERVAL = 20.0

# ── DISPLAY ───────────────────────────────────────────────────────
WINDOW_TITLE = "LionChief — Safe Distance"
SHOW_VIDEO   = True

ZONE_COLORS = {
    "DANGER":  (0,   0, 200),
    "WARNING": (0,  80, 220),
    "CAUTION": (0, 140, 255),
    "SAFE":    (0, 180,  60),
    "FAR":     (200, 160, 0),
    "UNKNOWN": (120, 120, 120),
    "ESCAPE":  (200,  80, 200),
}

# ── LOGGING ───────────────────────────────────────────────────────
LOG_TO_FILE = True
LOG_FILE    = "safe_distance_log.txt"