# ══════════════════════════════════════════════════════════════════
#  speed_controller.py  —  Gap-to-Speed Controller
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  Converts pixel gap between trains into a BLE speed command (0-7)
#  for Train B (rear train).
#
#  ZONE MAP:
#    DANGER  < DISTANCE_DANGER  → speed 0 (STOP immediately)
#    WARNING < DISTANCE_WARNING → speed FOLLOW_MIN_SPEED
#    CAUTION < DISTANCE_CAUTION → speed CAUTION_SPEED
#    SAFE    < DISTANCE_SAFE    → user-set speed
#    FAR     > DISTANCE_FAR     → MAX_CATCH_SPEED (catch up)
#
#  HYSTERESIS:
#    Zone boundary is not a single threshold — there is an offset
#    so you must travel HYSTERESIS_OFFSET pixels past the boundary
#    before the zone changes back. Prevents rapid speed oscillation
#    when the gap hovers around a threshold.
#
#  SMOOTHING:
#    Commanded speed is smoothed with an exponential moving average.
#    Slow-down uses a higher alpha (reacts faster — safety).
#    Speed-up uses a lower alpha (reacts slower — smooth).
# ══════════════════════════════════════════════════════════════════

import time
import logging

import config

logger = logging.getLogger("SpeedController")


class Zone:
    DANGER  = "DANGER"
    WARNING = "WARNING"
    CAUTION = "CAUTION"
    SAFE    = "SAFE"
    FAR     = "FAR"
    UNKNOWN = "UNKNOWN"   # one or both trains not visible


# Maps zone to a nominal integer speed
ZONE_SPEED = {
    Zone.DANGER:  0,
    Zone.WARNING: config.FOLLOW_MIN_SPEED,
    Zone.CAUTION: config.CAUTION_SPEED,
    Zone.SAFE:    config.DEFAULT_SPEED,    # replaced by user_speed at runtime
    Zone.FAR:     config.MAX_CATCH_SPEED,
    Zone.UNKNOWN: config.MISSING_SAFE_SPEED,
}


class SpeedController:
    """
    Zone-based speed controller with hysteresis and exponential smoothing.

    Usage:
        ctrl = SpeedController()
        ctrl.set_user_speed(5)       # user sets desired following speed
        speed, zone = ctrl.update(distance_px)  # call every frame
        if ctrl.should_send_command():
            ble.set_speed(speed)
    """

    def __init__(self):
        self._user_speed       = config.DEFAULT_SPEED
        self._current_zone     = Zone.UNKNOWN
        self._smooth_speed     = float(config.DEFAULT_SPEED)
        self._last_cmd_speed   = -1      # last speed actually sent via BLE
        self._last_cmd_time    = 0.0     # time of last BLE command
        self._pending_speed    = config.DEFAULT_SPEED
        self._emergency_stop   = False   # True when DANGER zone — bypass rate limit

    # ── Public API ────────────────────────────────────────────────

    def set_user_speed(self, speed: int):
        """Set the desired cruising speed (1-7). Used in SAFE zone."""
        self._user_speed = max(1, min(7, speed))
        logger.info(f"User speed set to {self._user_speed}")

    @property
    def user_speed(self) -> int:
        return self._user_speed

    @property
    def current_zone(self) -> str:
        return self._current_zone

    @property
    def commanded_speed(self) -> int:
        return self._pending_speed

    def update(self, distance_px: "float | None") -> tuple:
        """
        Calculate the target speed for Train B given current gap.

        Args:
            distance_px: pixel distance between trains, or None if
                         either train is not visible in the frame.

        Returns:
            (speed, zone) — speed is int 0-7, zone is a Zone string.
        """
        # ── Determine raw zone ─────────────────────────────────
        if distance_px is None:
            raw_zone = Zone.UNKNOWN
        else:
            raw_zone = self._classify_distance(distance_px)

        # ── Hysteresis — only change zone if moved far enough ──
        new_zone = self._apply_hysteresis(raw_zone, distance_px)
        self._current_zone = new_zone

        # ── Target speed for this zone ──────────────────────────
        if new_zone == Zone.SAFE:
            target = self._user_speed
        elif new_zone == Zone.FAR:
            # Catch up but do not exceed MAX_CATCH_SPEED
            target = min(config.MAX_CATCH_SPEED, self._user_speed + 2)
        else:
            target = ZONE_SPEED[new_zone]

        # ── Emergency stop — bypass all smoothing ──────────────
        if new_zone == Zone.DANGER:
            self._emergency_stop = True
            self._smooth_speed   = 0.0
            self._pending_speed  = 0
            logger.warning(f"[DANGER] Emergency STOP — gap {distance_px:.0f}px" if distance_px else "[DANGER] Train missing → STOP")
            return 0, new_zone

        self._emergency_stop = False

        # ── Exponential smoothing ───────────────────────────────
        # Slow down faster than speed up (asymmetric — safety)
        if target < self._smooth_speed:
            alpha = config.ALPHA_SLOW_DOWN
        else:
            alpha = config.ALPHA_SPEED_UP

        self._smooth_speed = (
            alpha * target + (1.0 - alpha) * self._smooth_speed
        )

        # Round to nearest integer and clamp
        speed = int(round(self._smooth_speed))
        speed = max(0, min(7, speed))
        self._pending_speed = speed

        return speed, new_zone

    def should_send_command(self) -> bool:
        """
        True if a new BLE command should be sent.

        Rules:
          1. Always send on emergency stop (DANGER zone).
          2. Otherwise rate-limit to MIN_COMMAND_INTERVAL_MS.
          3. Do not send if speed has not changed since last send.
        """
        if self._emergency_stop:
            return True

        now = time.time()
        time_ok  = (now - self._last_cmd_time) * 1000 >= config.MIN_COMMAND_INTERVAL_MS
        speed_ok = self._pending_speed != self._last_cmd_speed

        return time_ok and speed_ok

    def command_sent(self, speed: int):
        """Call after successfully sending a BLE command to update tracking."""
        self._last_cmd_speed = speed
        self._last_cmd_time  = time.time()

    def reset(self):
        """Reset controller state — call when resuming after a pause."""
        self._smooth_speed   = float(self._user_speed)
        self._current_zone   = Zone.UNKNOWN
        self._last_cmd_speed = -1
        self._emergency_stop = False

    # ── Private helpers ───────────────────────────────────────────

    def _classify_distance(self, d: float) -> str:
        """Raw zone classification — no hysteresis applied here."""
        if d < config.DISTANCE_DANGER:
            return Zone.DANGER
        elif d < config.DISTANCE_WARNING:
            return Zone.WARNING
        elif d < config.DISTANCE_CAUTION:
            return Zone.CAUTION
        elif d < config.DISTANCE_SAFE:
            return Zone.SAFE
        elif d < config.DISTANCE_FAR:
            return Zone.SAFE   # comfortable safe zone
        else:
            return Zone.FAR

    def _apply_hysteresis(self, raw_zone: str,
                          distance_px: "float | None") -> str:
        """
        Apply hysteresis: only change to a 'safer' (faster) zone if
        the gap has moved HYSTERESIS_OFFSET pixels past the boundary.

        This prevents the speed from oscillating when the gap
        hovers right on a threshold.
        """
        if distance_px is None:
            return raw_zone

        prev = self._current_zone
        h    = config.HYSTERESIS_OFFSET

        # Allow immediate move to a more restrictive zone (safety first)
        # Require extra distance to relax back to a more permissive zone

        if raw_zone == Zone.DANGER:
            return Zone.DANGER   # always react to danger immediately

        if raw_zone == Zone.WARNING:
            if prev in (Zone.CAUTION, Zone.SAFE, Zone.FAR, Zone.UNKNOWN):
                # Moving to more restrictive — only if clearly past boundary
                return Zone.WARNING if distance_px < config.DISTANCE_WARNING else prev
            return Zone.WARNING

        if raw_zone == Zone.CAUTION:
            if prev == Zone.WARNING:
                # Exiting WARNING — require extra margin
                return Zone.CAUTION if distance_px > config.DISTANCE_WARNING + h else Zone.WARNING
            return Zone.CAUTION

        if raw_zone == Zone.SAFE:
            if prev == Zone.CAUTION:
                return Zone.SAFE if distance_px > config.DISTANCE_CAUTION + h else Zone.CAUTION
            if prev == Zone.WARNING:
                return Zone.SAFE if distance_px > config.DISTANCE_WARNING + h else Zone.WARNING
            return Zone.SAFE

        if raw_zone == Zone.FAR:
            if prev == Zone.SAFE:
                return Zone.FAR if distance_px > config.DISTANCE_FAR + h else Zone.SAFE
            return Zone.FAR

        return raw_zone
