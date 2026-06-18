# ══════════════════════════════════════════════════════════════════
#  speed_controller.py  —  Cooperative Dual Train Speed Controller
#  Harry Locomotive Project 3 BT  |  Datix AI  |  June 2026
#
#  Both trains are BLE — the controller adjusts BOTH speeds.
#
#  COOPERATIVE CONTROL:
#
#    Gap closing (too small):
#      → Slow Train B (primary action)
#      → Optionally speed up Train A by COOPERATIVE_ADJUST_A steps
#        (this widens the gap faster than slowing B alone)
#
#    Gap opening (too large):
#      → Speed up Train B (primary action)
#      → Optionally slow Train A by COOPERATIVE_ADJUST_A steps
#        (this closes the gap faster than speeding B alone)
#
#    Emergency (DANGER zone):
#      → STOP BOTH immediately — both returns speed 0
#
#  Returns (speed_a, speed_b, zone) every call.
#  Smoothing and hysteresis apply to Train B.
#  Train A adjustments are integer steps — no smoothing needed
#  (small, brief adjustments that quickly return to user speed).
# ══════════════════════════════════════════════════════════════════

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


class DualSpeedController:
    """
    Cooperative speed controller for both BLE trains.

    update(gap_px) → (speed_a, speed_b, zone)

    Train A is the front train — normally runs at user_speed_a.
    Train B is the rear train  — adjusted primarily based on gap.
    Cooperative: Train A is nudged slightly to help close/widen gap faster.
    """

    def __init__(self):
        self._user_speed_a   = config.DEFAULT_SPEED_A
        self._user_speed_b   = config.DEFAULT_SPEED_B
        self._current_zone   = Zone.UNKNOWN
        self._smooth_b       = float(config.DEFAULT_SPEED_B)
        self._last_speed_a   = config.DEFAULT_SPEED_A
        self._last_speed_b   = config.DEFAULT_SPEED_B
        self._emergency      = False

    # ── Public API ────────────────────────────────────────────────

    def set_user_speed_a(self, speed: int):
        """Set Train A cruising speed (1-7)."""
        self._user_speed_a = max(1, min(7, speed))

    def set_user_speed_b(self, speed: int):
        """Set Train B cruising speed (1-7)."""
        self._user_speed_b = max(1, min(7, speed))

    def set_user_speeds(self, speed: int):
        """Set same cruising speed for both trains."""
        self.set_user_speed_a(speed)
        self.set_user_speed_b(speed)

    @property
    def user_speed_a(self) -> int:
        return self._user_speed_a

    @property
    def user_speed_b(self) -> int:
        return self._user_speed_b

    @property
    def current_zone(self) -> str:
        return self._current_zone

    @property
    def is_emergency(self) -> bool:
        return self._emergency

    def update(self, gap_px: "float | None") -> tuple:
        """
        Calculate target speeds for both trains.

        Args:
            gap_px: pixel distance between trains, or None if not visible.

        Returns:
            (speed_a, speed_b, zone)
              speed_a — target speed for Train A (0-7)
              speed_b — target speed for Train B (0-7)
              zone    — current zone string
        """
        # ── Classify zone ──────────────────────────────────────
        if gap_px is None:
            raw_zone = Zone.UNKNOWN
        else:
            raw_zone = self._classify(gap_px)

        new_zone = self._apply_hysteresis(raw_zone, gap_px)
        self._current_zone = new_zone

        # ── Emergency stop — both trains ───────────────────────
        if new_zone == Zone.DANGER:
            self._emergency  = True
            self._smooth_b   = 0.0
            self._last_speed_a = 0
            self._last_speed_b = 0
            logger.warning(
                f"[DANGER] STOP BOTH — gap={f'{gap_px:.0f}px' if gap_px else 'invisible'}")
            return 0, 0, Zone.DANGER

        self._emergency = False

        # ── Calculate target speeds ────────────────────────────
        adj = config.COOPERATIVE_ADJUST_A

        if new_zone == Zone.WARNING:
            # Gap dangerously small → slow B, nudge A forward
            target_b = config.MIN_FOLLOW_SPEED
            target_a = min(7, self._user_speed_a + adj)

        elif new_zone == Zone.CAUTION:
            # Gap reducing → slow B, keep A steady
            target_b = config.CAUTION_SPEED
            target_a = self._user_speed_a

        elif new_zone == Zone.SAFE:
            # Comfortable gap → both at user speed
            target_b = self._user_speed_b
            target_a = self._user_speed_a

        elif new_zone == Zone.FAR:
            # Gap too large → speed B up, nudge A back slightly
            target_b = min(config.MAX_CATCH_SPEED,
                           self._user_speed_b + 2)
            target_a = max(1, self._user_speed_a - adj)

        else:  # UNKNOWN — one train not visible
            target_b = config.MISSING_SAFE_SPEED
            target_a = self._user_speed_a

        # ── Smooth Train B (asymmetric) ────────────────────────
        if target_b < self._smooth_b:
            alpha = config.ALPHA_SLOW_DOWN   # brake fast
        else:
            alpha = config.ALPHA_SPEED_UP    # accelerate slowly

        self._smooth_b = alpha * target_b + (1.0 - alpha) * self._smooth_b
        speed_b = max(0, min(7, int(round(self._smooth_b))))

        # Train A: integer target, no smoothing needed
        speed_a = max(0, min(7, int(round(target_a))))

        self._last_speed_a = speed_a
        self._last_speed_b = speed_b

        return speed_a, speed_b, new_zone

    def reset(self):
        """Reset smoothing — call after pause/resume."""
        self._smooth_b     = float(self._user_speed_b)
        self._current_zone = Zone.UNKNOWN
        self._emergency    = False

    # ── Private ────────────────────────────────────────────────

    def _classify(self, d: float) -> str:
        if d < config.DISTANCE_DANGER:
            return Zone.DANGER
        elif d < config.DISTANCE_WARNING:
            return Zone.WARNING
        elif d < config.DISTANCE_CAUTION:
            return Zone.CAUTION
        elif d < config.DISTANCE_FAR:
            return Zone.SAFE
        else:
            return Zone.FAR

    def _apply_hysteresis(self, raw: str, d: "float | None") -> str:
        """
        Prevent zone oscillation at boundaries.
        Moving to a more restrictive zone is immediate (safety).
        Moving to a more permissive zone requires HYSTERESIS_OFFSET extra.
        """
        if d is None:
            return raw
        prev = self._current_zone
        h    = config.HYSTERESIS_OFFSET

        if raw == Zone.DANGER:
            return Zone.DANGER   # always immediate

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
