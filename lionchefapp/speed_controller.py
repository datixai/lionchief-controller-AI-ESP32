# speed_controller.py — PD Controller v10.0
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
    ESCAPE  = "ESCAPE"


class SpeedController:
    """
    PD-based speed controller with BLE latency compensation and smooth decel.

    Gap rate (D term): measures how fast gap is closing each frame.
    Effective gap: current gap projected forward by BLE latency.
    Smoothstep curve: smooth braking between DANGER and SAFE zones.
    """

    def __init__(self):
        self._user_speed   = config.DEFAULT_SPEED
        self._smooth       = float(config.DEFAULT_SPEED)
        self._current_zone = Zone.UNKNOWN
        self._last_cmd_spd = -1
        self._last_cmd_t   = 0.0
        self._emergency    = False
        self._prev_gap     = None
        self._gap_rate     = 0.0   # px/frame negative=closing
        self._latency_f    = (config.BLE_LATENCY_MS / 1000.0
                              * config.CAMERA_FPS)

    @property
    def user_speed(self) -> int:
        return self._user_speed

    @property
    def current_zone(self) -> str:
        return self._current_zone

    @property
    def is_emergency(self) -> bool:
        return self._emergency

    @property
    def gap_rate(self) -> float:
        return self._gap_rate

    def set_user_speed(self, speed: int):
        self._user_speed = max(1, min(7, speed))

    def update(self, gap_px, a_chasing_b: bool = False) -> tuple:
        # Update closing rate (D term)
        if gap_px is not None and self._prev_gap is not None:
            raw = gap_px - self._prev_gap
            a   = config.CLOSING_RATE_ALPHA
            self._gap_rate = a * raw + (1.0 - a) * self._gap_rate
        elif gap_px is None:
            self._gap_rate = 0.0
        self._prev_gap = gap_px

        # Escape mode
        if a_chasing_b:
            self._emergency    = False
            self._current_zone = Zone.ESCAPE
            target = (config.ESCAPE_MAX_SPEED if gap_px is None
                      or gap_px < config.DISTANCE_SAFE
                      else max(config.ESCAPE_MIN_SPEED, self._user_speed))
            self._smooth = 0.8 * target + 0.2 * self._smooth
            return max(0, min(7, int(round(self._smooth)))), Zone.ESCAPE

        # Effective gap — project forward by BLE latency
        if gap_px is not None:
            corr    = self._gap_rate * self._latency_f
            eff_gap = max(0.0, gap_px + min(0.0, corr))
        else:
            eff_gap = None

        # Zone classify
        raw_zone = self._classify(eff_gap)
        zone     = self._hyst(raw_zone, eff_gap)
        self._current_zone = zone

        if zone == Zone.DANGER:
            self._emergency = True
            self._smooth    = 0.0
            return 0, Zone.DANGER

        self._emergency = False
        target = self._target(zone, eff_gap)
        alpha  = config.ALPHA_SLOW_DOWN if target < self._smooth \
                 else config.ALPHA_SPEED_UP
        self._smooth = alpha * target + (1.0 - alpha) * self._smooth
        return max(0, min(7, int(round(self._smooth)))), zone

    def _target(self, zone: str, eff_gap) -> float:
        if eff_gap is None:
            return float(config.MISSING_SAFE_SPEED)
        if zone == Zone.FAR:
            return float(min(config.MAX_CATCH_SPEED, self._user_speed + 2))
        if zone == Zone.SAFE and eff_gap >= config.DISTANCE_SAFE:
            return float(self._user_speed)
        if zone == Zone.UNKNOWN:
            return float(config.MISSING_SAFE_SPEED)
        if not config.USE_SMOOTH_DECEL:
            return {Zone.WARNING: float(config.FOLLOW_MIN_SPEED),
                    Zone.CAUTION: float(config.CAUTION_SPEED),
                    Zone.SAFE:    float(self._user_speed)}.get(
                    zone, float(self._user_speed))
        # Smoothstep curve DANGER → SAFE
        span = float(config.DISTANCE_SAFE - config.DISTANCE_DANGER)
        if span <= 0:
            return float(self._user_speed)
        t        = max(0.0, min(1.0,
                       (eff_gap - config.DISTANCE_DANGER) / span))
        t_smooth = t * t * (3.0 - 2.0 * t)   # S-curve
        return max(0.0, t_smooth * self._user_speed)

    def should_send_command(self, speed: int) -> bool:
        if self._emergency: return True
        return ((time.time()-self._last_cmd_t)*1000
                >= config.MIN_COMMAND_INTERVAL_MS
                and speed != self._last_cmd_spd)

    def command_sent(self, speed: int):
        self._last_cmd_spd = speed
        self._last_cmd_t   = time.time()

    def reset(self):
        self._smooth       = float(self._user_speed)
        self._current_zone = Zone.UNKNOWN
        self._last_cmd_spd = -1
        self._emergency    = False
        self._prev_gap     = None
        self._gap_rate     = 0.0

    def _classify(self, d) -> str:
        if d is None:                    return Zone.UNKNOWN
        if d < config.DISTANCE_DANGER:  return Zone.DANGER
        if d < config.DISTANCE_WARNING: return Zone.WARNING
        if d < config.DISTANCE_CAUTION: return Zone.CAUTION
        if d < config.DISTANCE_FAR:     return Zone.SAFE
        return Zone.FAR

    def _hyst(self, raw, d) -> str:
        if d is None: return raw
        prev = self._current_zone
        h    = config.HYSTERESIS_OFFSET
        if raw == Zone.DANGER: return Zone.DANGER
        if raw == Zone.WARNING:
            return Zone.WARNING if (prev in (Zone.CAUTION,Zone.SAFE,
                Zone.FAR,Zone.UNKNOWN) and d<config.DISTANCE_WARNING
            ) else (Zone.WARNING if prev==Zone.WARNING else prev)
        if raw == Zone.CAUTION:
            return Zone.CAUTION if prev!=Zone.WARNING else (
                Zone.CAUTION if d>config.DISTANCE_WARNING+h else Zone.WARNING)
        if raw == Zone.SAFE:
            if prev==Zone.CAUTION: return Zone.SAFE if d>config.DISTANCE_CAUTION+h else Zone.CAUTION
            if prev==Zone.WARNING: return Zone.SAFE if d>config.DISTANCE_WARNING+h else Zone.WARNING
            return Zone.SAFE
        if raw == Zone.FAR:
            return Zone.FAR if (prev!=Zone.SAFE or d>config.DISTANCE_FAR+h) else Zone.SAFE
        return raw
