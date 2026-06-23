# speed_controller.py — Gap to Speed Controller
# Harry Locomotive Project 3 | Datix AI | June 2026

import time
import logging
import config

logger = logging.getLogger("SpeedCtrl")


class Zone:
    DANGER  = "DANGER"
    WARNING = "WARNING"
    CAUTION = "CAUTION"
    SAFE    = "SAFE"
    FAR     = "FAR"
    UNKNOWN = "UNKNOWN"


class SpeedController:

    def __init__(self):
        self._user_speed    = config.DEFAULT_SPEED
        self._smooth        = float(config.DEFAULT_SPEED)
        self._current_zone  = Zone.UNKNOWN
        self._last_cmd_spd  = -1
        self._last_cmd_time = 0.0
        self._emergency     = False

    @property
    def user_speed(self) -> int:
        return self._user_speed

    @property
    def current_zone(self) -> str:
        return self._current_zone

    @property
    def is_emergency(self) -> bool:
        return self._emergency

    def set_user_speed(self, speed: int):
        self._user_speed = max(1, min(7, speed))
        logger.info(f"User speed → {self._user_speed}")

    def update(self, gap_px: "float | None") -> tuple:
        raw  = self._classify(gap_px)
        zone = self._hysteresis(raw, gap_px)
        self._current_zone = zone

        if zone == Zone.DANGER:
            self._emergency = True
            self._smooth    = 0.0
            return 0, zone

        self._emergency = False

        targets = {
            Zone.WARNING: config.FOLLOW_MIN_SPEED,
            Zone.CAUTION: config.CAUTION_SPEED,
            Zone.SAFE:    self._user_speed,
            Zone.FAR:     min(config.MAX_CATCH_SPEED, self._user_speed + 2),
            Zone.UNKNOWN: config.MISSING_SAFE_SPEED,
        }
        target = targets.get(zone, self._user_speed)

        alpha = config.ALPHA_SLOW_DOWN if target < self._smooth \
                else config.ALPHA_SPEED_UP
        self._smooth = alpha * target + (1.0 - alpha) * self._smooth
        speed = max(0, min(7, int(round(self._smooth))))
        return speed, zone

    def should_send_command(self, speed: int) -> bool:
        if self._emergency:
            return True
        now = time.time()
        return ((now - self._last_cmd_time) * 1000 >= config.MIN_COMMAND_INTERVAL_MS
                and speed != self._last_cmd_spd)

    def command_sent(self, speed: int):
        self._last_cmd_spd  = speed
        self._last_cmd_time = time.time()

    def reset(self):
        self._smooth        = float(self._user_speed)
        self._current_zone  = Zone.UNKNOWN
        self._last_cmd_spd  = -1
        self._emergency     = False

    def _classify(self, d: "float | None") -> str:
        if d is None:              return Zone.UNKNOWN
        if d < config.DISTANCE_DANGER:   return Zone.DANGER
        if d < config.DISTANCE_WARNING:  return Zone.WARNING
        if d < config.DISTANCE_CAUTION:  return Zone.CAUTION
        if d < config.DISTANCE_FAR:      return Zone.SAFE
        return Zone.FAR

    def _hysteresis(self, raw: str, d: "float | None") -> str:
        if d is None:
            return raw
        prev = self._current_zone
        h    = config.HYSTERESIS_OFFSET

        if raw == Zone.DANGER:
            return Zone.DANGER
        if raw == Zone.WARNING:
            if prev in (Zone.CAUTION, Zone.SAFE, Zone.FAR, Zone.UNKNOWN):
                return Zone.WARNING if d < config.DISTANCE_WARNING else prev
            return Zone.WARNING
        if raw == Zone.CAUTION:
            if prev == Zone.WARNING:
                return Zone.CAUTION if d > config.DISTANCE_WARNING + h else Zone.WARNING
            return Zone.CAUTION
        if raw == Zone.SAFE:
            if prev == Zone.CAUTION:
                return Zone.SAFE if d > config.DISTANCE_CAUTION + h else Zone.CAUTION
            if prev == Zone.WARNING:
                return Zone.SAFE if d > config.DISTANCE_WARNING + h else Zone.WARNING
            return Zone.SAFE
        if raw == Zone.FAR:
            if prev == Zone.SAFE:
                return Zone.FAR if d > config.DISTANCE_FAR + h else Zone.SAFE
            return Zone.FAR
        return raw