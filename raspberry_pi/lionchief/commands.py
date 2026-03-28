"""
LionChief BLE Command Constants
================================
Sources combined:
  - Property404/lionchief-controller  (original protocol reverse engineering)
  - idaband/lionchief-controller-raspberrypi  (extended command set)
  - chrcraven/LionchiefInteractiveDisplay  (production patterns)

Confirmed UUIDs for Peter's train:
  Service UUID   : e20a39f4-73f5-4bc4-a12f-17d1ad07a961
  Characteristic : 08590f7e-db05-467e-8757-72f6faeb13d4

All commands use handle 0x25. Format: [0x00, CMD, VALUE, ..., CHECKSUM]
Checksum = (256 - sum(values)) % 256  — but train does NOT verify it.
"""

# ── BLE UUIDs ──────────────────────────────────────────────────────────────────
SERVICE_UUID        = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
CHARACTERISTIC_UUID = "08590f7e-db05-467e-8757-72f6faeb13d4"

# ── Speed range ───────────────────────────────────────────────────────────────
SPEED_STOP   = 0x00   # Absolute stop
SPEED_SLOW   = 0x03   # Slow crawl
SPEED_MEDIUM = 0x07   # Medium — default resume speed
SPEED_FAST   = 0x0F   # Fast
SPEED_MAX    = 0x1F   # Maximum speed

# ── Command bytes (second byte in packet) ─────────────────────────────────────
CMD_SPEED     = 0x45   # [0x45, 0x00–0x1F]
CMD_DIRECTION = 0x46   # [0x46, 0x01=fwd | 0x02=rev]
CMD_BELL      = 0x47   # [0x47, 0x01=on  | 0x00=off]
CMD_HORN      = 0x48   # [0x48, 0x01=on  | 0x00=off]
CMD_SPEAK     = 0x4D   # [0x4D, phrase, 0] — 0=random
CMD_VOLUME    = 0x4C   # [0x4C, 0x00–0x07] master volume
CMD_LIGHTS    = 0x51   # [0x51, 0x01=on  | 0x00=off]
CMD_DISCONNECT= 0x4B   # [0x4B, 0x00, 0x00]

# Volume/pitch sub-commands (second byte after CMD_VOLUME_PITCH = 0x44)
CMD_VOLUME_PITCH = 0x44
SUBCMD_HORN_PITCH    = 0x01
SUBCMD_BELL_PITCH    = 0x02
SUBCMD_SPEECH_VOLUME = 0x03
SUBCMD_ENGINE_VOLUME = 0x04

# Pre-built ready-to-send byte arrays (no checksum needed — train ignores it)
STOP_CMD     = bytearray([0x00, CMD_SPEED,     SPEED_STOP])
SLOW_CMD     = bytearray([0x00, CMD_SPEED,     SPEED_SLOW])
MEDIUM_CMD   = bytearray([0x00, CMD_SPEED,     SPEED_MEDIUM])
FAST_CMD     = bytearray([0x00, CMD_SPEED,     SPEED_FAST])
MAX_CMD      = bytearray([0x00, CMD_SPEED,     SPEED_MAX])
FORWARD_CMD  = bytearray([0x00, CMD_DIRECTION, 0x01])
REVERSE_CMD  = bytearray([0x00, CMD_DIRECTION, 0x02])
BELL_ON_CMD  = bytearray([0x00, CMD_BELL,      0x01])
BELL_OFF_CMD = bytearray([0x00, CMD_BELL,      0x00])
HORN_ON_CMD  = bytearray([0x00, CMD_HORN,      0x01])
HORN_OFF_CMD = bytearray([0x00, CMD_HORN,      0x00])
LIGHT_ON_CMD = bytearray([0x00, CMD_LIGHTS,    0x01])
LIGHT_OFF_CMD= bytearray([0x00, CMD_LIGHTS,    0x00])
SPEAK_CMD    = bytearray([0x00, CMD_SPEAK,     0x00, 0x00])  # random phrase
DISCO_CMD    = bytearray([0x00, CMD_DISCONNECT,0x00, 0x00])


def build_speed_cmd(speed: int) -> bytearray:
    """Return a stop/go speed command. Speed must be 0–31 (0x00–0x1F)."""
    speed = max(0, min(31, int(speed)))
    return bytearray([0x00, CMD_SPEED, speed])


def build_volume_cmd(level: int) -> bytearray:
    """Master volume 0–7."""
    level = max(0, min(7, int(level)))
    return bytearray([0x00, CMD_VOLUME, level])
