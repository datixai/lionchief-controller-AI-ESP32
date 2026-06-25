# ══════════════════════════════════════════════════════════════════
#  config.py  —  Single Track Safe Distance v9.0
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
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
TABLE_MASK_FILE  = "table_mask.json"

# ── TRACKER ───────────────────────────────────────────────────────
MIN_BOX_SIZE = 15

# Local search radius — larger = more robust for fast trains
# ★ TUNE UP if losing tracking on turns
SEARCH_RADIUS = 120      # pixels in 960×540

MIN_BLOB_AREA = 60       # minimum blob area inside search

POSITION_SMOOTH_FRAMES = 3
VELOCITY_ALPHA          = 0.4

# Tracking circle display
CIRCLE_RADIUS_MIN = 18
CIRCLE_RADIUS_MAX = 55

# ── STOPPED TRAIN DETECTION ───────────────────────────────────────
# When MOG2 finds no blob, compare current frame to reference patch
# taken when the train was last seen. If patches match → train stopped.
REF_PATCH_HALF   = 45    # half-size of reference patch (pixels)
STOPPED_DIFF_THR = 18    # mean pixel diff below this = "same scene"
                          # ★ TUNE UP if false "stopped" detections
STOP_CONFIRM_FRAMES = 5  # frames of no motion before locking position

# ── MOG2 ──────────────────────────────────────────────────────────
MOG2_HISTORY       = 400
MOG2_VAR_THRESHOLD = 50   # ★ TUNE UP to ignore more noise
# Very low learning rate — stopped trains stay visible in fg for longer
MOG2_LEARNING_RATE = 0.001

# ── DISTANCE ZONES (pixels in 960×540) ───────────────────────────
# Large DANGER zone — Train B stops well before reaching Train A.
# These are surface-to-surface gaps (minus train radii).
DISTANCE_DANGER  = 120   # STOP immediately — plenty of room
DISTANCE_WARNING = 180   # slow to minimum
DISTANCE_CAUTION = 250   # reduce speed
DISTANCE_SAFE    = 340   # follow at user speed
DISTANCE_FAR     = 460   # gap too large — catch up
HYSTERESIS_OFFSET = 25

# ── SPEED ─────────────────────────────────────────────────────────
DEFAULT_SPEED      = 5
MAX_CATCH_SPEED    = 7
FOLLOW_MIN_SPEED   = 2
CAUTION_SPEED      = 3
ESCAPE_MIN_SPEED   = 5
ESCAPE_MAX_SPEED   = 7

# ── SMOOTHING ─────────────────────────────────────────────────────
ALPHA_SLOW_DOWN         = 0.8
ALPHA_SPEED_UP          = 0.20
MIN_COMMAND_INTERVAL_MS = 300

# ── SAFETY ────────────────────────────────────────────────────────
MISSING_TIMEOUT_S  = 8.0    # longer before declaring "missing"
MISSING_SAFE_SPEED = 2

# ── BLE ───────────────────────────────────────────────────────────
RECONNECT_INTERVAL = 5.0
KEEPALIVE_INTERVAL = 20.0

# ── DISPLAY ───────────────────────────────────────────────────────
WINDOW_TITLE = "LionChief — Safe Distance"

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