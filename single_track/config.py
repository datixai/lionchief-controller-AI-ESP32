# ══════════════════════════════════════════════════════════════════
#  config.py  —  Single Track Safe Distance v8.0
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

# Fixed display size — window = frame = mouse coords always match
DISPLAY_W = 960
DISPLAY_H = 540
CAMERA_WARMUP_FRAMES = 40

# ── FILES ─────────────────────────────────────────────────────────
CALIBRATION_FILE = "calibration.json"
TABLE_MASK_FILE  = "table_mask.json"

# ── TRACKER ───────────────────────────────────────────────────────
MIN_BOX_SIZE = 15          # minimum drag size to accept a selection

# Local search radius — each train only searches this far from
# its last known position. Keeps people walking far away undetected.
# ★ TUNE UP if train moves fast and tracker loses it
SEARCH_RADIUS = 90         # pixels in 960×540 display coords

MIN_BLOB_AREA = 80         # minimum blob area inside search circle

POSITION_SMOOTH_FRAMES = 4
VELOCITY_ALPHA         = 0.4

# Tracking circle display
# Circle is drawn around each tracked train — radius scales with blob size
CIRCLE_RADIUS_MIN = 18    # minimum circle radius (pixels)
CIRCLE_RADIUS_MAX = 50    # maximum circle radius (pixels)

# ── MOG2 ──────────────────────────────────────────────────────────
MOG2_HISTORY       = 300
MOG2_VAR_THRESHOLD = 45   # ★ TUNE UP if false detections remain
MOG2_LEARNING_RATE = 0.005

# ── DISTANCE ZONES (pixels in 960×540) ───────────────────────────
DISTANCE_DANGER  = 90
DISTANCE_WARNING = 140
DISTANCE_CAUTION = 200
DISTANCE_SAFE    = 280
DISTANCE_FAR     = 400
HYSTERESIS_OFFSET = 18

# ── SPEED ─────────────────────────────────────────────────────────
DEFAULT_SPEED      = 5
MAX_CATCH_SPEED    = 7
FOLLOW_MIN_SPEED   = 2
CAUTION_SPEED      = 3
ESCAPE_MIN_SPEED   = 5
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