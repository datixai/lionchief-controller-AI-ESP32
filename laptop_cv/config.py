# ══════════════════════════════════════════════════════════════════
#  config.py  —  LionChief Laptop CV  v5.1
#  Harry Locomotive Project  |  Datix AI  |  May 2026
#
#  All tunable values live here.
#  Never edit zone_detector.py, main.py etc. — only edit this file.
#
#  THREE ZONES matching ESP32 IR sensor positions:
#    Zone A — Inner loop approach   (before shared section, inner track)
#    Zone B — Outer loop approach   (before shared section, outer track)
#    Zone C — Shared section / exit (any train passing here)
#
#  ZONES are POLYGONS — drawn by calibrate_zone.py
#  Polygon zones can follow curved track precisely.
# ══════════════════════════════════════════════════════════════════

# ── OUTER TRAIN BLE ───────────────────────────────────────────────
#  New train confirmed by Peter (May 2026)
TRAIN_MAC  = "60:F9:FB:49:94:C9"      # direct MAC — faster connect
TRAIN_NAME = "LC-1-1-09F9-60F9"       # name prefix fallback if MAC fails

# LionChief BLE UUIDs — standard across all LionChief models
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
    CMD_SPEED_4, CMD_SPEED_5, CMD_SPEED_6, CMD_SPEED_7
]

# ── SPEED SETTINGS ────────────────────────────────────────────────
DEFAULT_OUTER_SPEED  = 7   # Speed outer train starts at
RESUME_RAMP_SPEED    = 5   # Intermediate speed during ramp-up
                            # Capped at user-set speed automatically

# ── CAMERA ────────────────────────────────────────────────────────
CAMERA_INDEX  = 1      # USB camera index — change if wrong camera
CAMERA_WIDTH  = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS    = 30

# ── ZONE FILE ─────────────────────────────────────────────────────
# Written by calibrate_zone.py, read by zone_detector.py + main.py
ZONES_FILE = "zones.json"

# ── DETECTION SETTINGS ────────────────────────────────────────────
# Camera is ceiling-mounted — trains appear SMALL from above
# Keep MIN_DETECTION_AREA lower than ground-level camera setups

MIN_DETECTION_AREA = 800    # Minimum blob area to count as a train
                             # Increase if getting false triggers
                             # Decrease if real trains not being detected

DETECTION_FRAMES_REQUIRED = 4   # Consecutive frames needed before trigger
                                  # Prevents single-frame flickers

# MOG2 background subtractor — learns the static layout
BG_HISTORY       = 400     # Frames to build background model
BG_THRESHOLD     = 40      # Higher = less sensitive to small changes
BG_DETECT_SHADOW = False   # Shadows cause false detections

# Reference frame comparison — catches STOPPED trains
# MOG2 misses trains that stop (learns them as background over time)
REFERENCE_DIFF_THRESHOLD     = 28   # Pixel brightness diff to count as changed
REFERENCE_MIN_CHANGED_FRAC   = 0.04 # Fraction of zone that must change (4%)

# ── TIMING (seconds) ──────────────────────────────────────────────
# All match ESP32 v5.1 timing values

AUTO_INNER_INTERVAL_S = 30.0   # ★ TUNE: how often to alert for inner loop
                                 # (no relay in Python — logs a reminder)

PARKING_DELAY_S       = 2.0    # ★ TUNE: time inner train takes to reach
                                 # parking spot AFTER Zone C clears

RESUME_DELAY_S        = 1.5    # Safety buffer after zone clears
SPEED_RAMP_S          = 1.5    # Time at ramp speed before user speed
STOP_REPEAT_S         = 0.5    # Repeat STOP every 0.5s while zone locked
ZONE_TIMEOUT_S        = 30.0   # Force-resume if Zone C never clears
RECONNECT_INTERVAL    = 5.0    # BLE reconnect attempt interval
KEEPALIVE_INTERVAL    = 20.0   # BLE keepalive ping interval

# ── ZONE LOCK ─────────────────────────────────────────────────────
# Once train enters a zone, detection stays active until it physically
# exits via Zone C. Prevents brief detection gaps from unlocking early.
ZONE_LOCK_ENABLED = True

# ── DISPLAY ───────────────────────────────────────────────────────
SHOW_VIDEO   = True
WINDOW_TITLE = "LionChief — Camera Collision Prevention"

# Zone colors BGR — safe / active
COLOR_ZONE_A_SAFE   = (0,  165, 255)   # Orange — inner approach clear
COLOR_ZONE_A_ACTIVE = (0,   0,  220)   # Red    — inner approach occupied
COLOR_ZONE_B_SAFE   = (255,  80,  0)   # Blue   — outer approach clear
COLOR_ZONE_B_ACTIVE = (0,   0,  220)   # Red    — outer approach occupied
COLOR_ZONE_C_SAFE   = (0,  200,   0)   # Green  — shared section clear
COLOR_ZONE_C_ACTIVE = (0,   0,  220)   # Red    — shared section occupied

# ── LOGGING ───────────────────────────────────────────────────────
LOG_TO_FILE   = True
LOG_FILE_PATH = "detection_log.txt"