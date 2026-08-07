# ══════════════════════════════════════════════════════════════════
#  config.py  —  Single Track DUAL BLE Safe Distance v1.0
#  Harry Locomotive Project 3 BT  |  Datix AI  |  June 2026
#
#  Both trains are Bluetooth.
#  Train A = front  (user-controlled speed via [ ] keys)
#  Train B = rear   (gap-detection controlled, same as single_track)
#
#  DANGER zone → BOTH trains stopped (emergency)
#  ESCAPE mode → Train B speeds up, Train A slows slightly
# ══════════════════════════════════════════════════════════════════

# ── TRAIN A — FRONT TRAIN (BLE) ───────────────────────────────────
# ★ UPDATE MAC to your actual front train address
TRAIN_A_MAC  = "60:FA:11:E3:94:C9"
TRAIN_A_NAME = "LC-1-1-09F8-60FA"

# ── TRAIN B — REAR TRAIN (BLE) ────────────────────────────────────
TRAIN_B_MAC  = "60:F9:FB:49:94:C9"
TRAIN_B_NAME = "LC-1-1-09F9-60F9"

# LionChief BLE UUIDs — same across all models
SERVICE_UUID        = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
CHARACTERISTIC_UUID = "08590f7e-db05-467e-8757-72f6faeb13d4"

CMD_STOP     = bytes([0x00, 0x45, 0x00])
CMD_SPEED_1  = bytes([0x00, 0x45, 0x04])   # 4  of 28 — slow creep
CMD_SPEED_2  = bytes([0x00, 0x45, 0x08])   # 8  of 28
CMD_SPEED_3  = bytes([0x00, 0x45, 0x0C])   # 12 of 28
CMD_SPEED_4  = bytes([0x00, 0x45, 0x10])   # 16 of 28 — medium
CMD_SPEED_5  = bytes([0x00, 0x45, 0x14])   # 20 of 28
CMD_SPEED_6  = bytes([0x00, 0x45, 0x18])   # 24 of 28
CMD_SPEED_7  = bytes([0x00, 0x45, 0x1C])   # 28 of 28 — full power
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
TRACK_PATH_FILE  = "track_path.json"

# ── TRACKER ───────────────────────────────────────────────────────
MIN_BOX_SIZE           = 15
SEARCH_RADIUS          = 120
MIN_BLOB_AREA          = 60
POSITION_SMOOTH_FRAMES = 3
VELOCITY_ALPHA         = 0.4

CIRCLE_RADIUS_MIN = 18
CIRCLE_RADIUS_MAX = 55

# ── STOPPED TRAIN DETECTION ───────────────────────────────────────
REF_PATCH_HALF      = 45
STOPPED_DIFF_THR    = 18
STOP_CONFIRM_FRAMES = 5

# ── TRACK PATH LEARNING ───────────────────────────────────────────
PATH_MIN_POINT_DIST = 12
PATH_SNAP_RADIUS    = 45

# ── MOG2 ──────────────────────────────────────────────────────────
MOG2_HISTORY       = 400
MOG2_VAR_THRESHOLD = 50
MOG2_LEARNING_RATE = 0.001

# ── BLE LATENCY COMPENSATION ──────────────────────────────────────
BLE_LATENCY_MS = 150

# ── CLOSING RATE (PD CONTROLLER) ──────────────────────────────────
CLOSING_RATE_WEIGHT = 0.8
CLOSING_RATE_ALPHA  = 0.5

# ── DISTANCE ZONES (surface-to-surface pixels) ────────────────────
DISTANCE_DANGER  = 160
DISTANCE_WARNING = 250
DISTANCE_CAUTION = 340
DISTANCE_SAFE    = 460
DISTANCE_FAR     = 600
HYSTERESIS_OFFSET = 20

# ── SPEED ─────────────────────────────────────────────────────────
DEFAULT_SPEED_A    = 5   # Train A default cruising speed
DEFAULT_SPEED_B    = 5   # Train B default cruising speed
MAX_CATCH_SPEED    = 7
FOLLOW_MIN_SPEED   = 2
CAUTION_SPEED      = 3
ESCAPE_MIN_SPEED   = 5
ESCAPE_MAX_SPEED   = 7

# Cooperative: when ESCAPE, slow Train A by this many steps
ESCAPE_SLOW_A_BY   = 1

# Aliases so speed_controller.py (copied from single_track) works unchanged
DEFAULT_SPEED      = DEFAULT_SPEED_B
MISSING_SAFE_SPEED = 2

# ── SMOOTH DECELERATION ───────────────────────────────────────────
USE_SMOOTH_DECEL = True

# ── SMOOTHING ─────────────────────────────────────────────────────
ALPHA_SLOW_DOWN         = 0.8
ALPHA_SPEED_UP          = 0.20
MIN_COMMAND_INTERVAL_MS = 150

# ── AUTO-PAUSE WHEN TRACKER LOST ──────────────────────────────────
AUTO_PAUSE_TIMEOUT_S = 1.5

# ── SAFETY ────────────────────────────────────────────────────────
MISSING_TIMEOUT_S = 8.0

# ── BLE ───────────────────────────────────────────────────────────
RECONNECT_INTERVAL = 5.0
KEEPALIVE_INTERVAL = 20.0

# ── DISPLAY ───────────────────────────────────────────────────────
WINDOW_TITLE = "LionChief — Dual BLE Safe Distance"

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
LOG_FILE    = "dual_bt_log.txt"