# ══════════════════════════════════════════════════════════════════
#  ble_controller.py  —  Dual LionChief BLE Controller
#  Harry Locomotive Project 3 BT  |  Datix AI  |  June 2026
#
#  Controls BOTH trains via Bluetooth independently.
#  Each train runs in its own asyncio loop on its own background thread.
#
#  DualTrainController wraps both and exposes a clean joint API:
#    - emergency_stop_all()  → stops both simultaneously
#    - set_speeds(a, b)      → sets both in one call
#    - connected_both        → True only when both are online
# ══════════════════════════════════════════════════════════════════

import asyncio
import threading
import logging
import time
from bleak import BleakClient, BleakScanner

import config

logger = logging.getLogger("BLE")


class SingleTrainBLE:
    """
    BLE controller for one LionChief train.
    Runs its own asyncio loop in a daemon thread.
    """

    def __init__(self, mac: str, name_prefix: str, label: str):
        self._mac          = mac.strip()
        self._name_prefix  = name_prefix
        self._label        = label        # "TrainA" or "TrainB" for logs
        self._client       = None
        self._char         = None
        self._connected    = False
        self._loop         = None
        self._thread       = None
        self._stop_event   = threading.Event()

        self.current_speed = 0
        self.user_speed    = config.DEFAULT_SPEED_A if label == "TrainA" \
                             else config.DEFAULT_SPEED_B
        self.connect_count = 0
        self.cmd_count     = 0

        # Rate limiting per train
        self._last_cmd_time  = 0.0
        self._last_cmd_speed = -1

    # ── Lifecycle ─────────────────────────────────────────────────

    def start(self):
        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name=f"BLE-{self._label}")
        self._thread.start()
        logger.info(f"[{self._label}] BLE started — target: {self._mac}")

    def shutdown(self):
        self.set_speed(0)
        time.sleep(0.3)
        self._stop_event.set()
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._disconnect(), self._loop)
        if self._thread:
            self._thread.join(timeout=5)

    @property
    def connected(self) -> bool:
        return self._connected

    # ── Commands ──────────────────────────────────────────────────

    def set_speed(self, speed: int):
        """Set speed (0-7). Saves user_speed if speed > 0."""
        speed = max(0, min(7, speed))
        self.current_speed = speed
        if speed > 0:
            self.user_speed = speed
        self._queue(config.SPEED_CMDS[speed], f"SPEED {speed}")
        self._last_cmd_speed = speed
        self._last_cmd_time  = time.time()

    def emergency_stop(self):
        """Immediate stop — bypasses rate limiting."""
        self.current_speed = 0
        self._queue(config.CMD_STOP, "EMERGENCY STOP")
        self._last_cmd_speed = 0
        self._last_cmd_time  = time.time()

    def send_stop_raw(self):
        """Send stop without touching user_speed tracking."""
        self._queue(config.CMD_STOP, "STOP")

    def keepalive(self):
        """Keepalive — only sends if speed > 0."""
        if self.current_speed > 0:
            self._queue(
                config.SPEED_CMDS[self.current_speed], "keepalive")

    def should_send(self, new_speed: int) -> bool:
        """True if enough time has passed and speed has changed."""
        now      = time.time()
        time_ok  = (now - self._last_cmd_time) * 1000 >= config.MIN_COMMAND_INTERVAL_MS
        speed_ok = new_speed != self._last_cmd_speed
        return time_ok and speed_ok

    def forward(self):    self._queue(config.CMD_FORWARD,  "FORWARD")
    def reverse(self):    self._queue(config.CMD_REVERSE,  "REVERSE")
    def horn_on(self):    self._queue(config.CMD_HORN_ON,  "HORN ON")
    def horn_off(self):   self._queue(config.CMD_HORN_OFF, "HORN OFF")
    def bell_on(self):    self._queue(config.CMD_BELL_ON,  "BELL ON")
    def bell_off(self):   self._queue(config.CMD_BELL_OFF, "BELL OFF")
    def lights_on(self):  self._queue(config.CMD_LIGHT_ON, "LIGHTS ON")
    def lights_off(self): self._queue(config.CMD_LIGHT_OFF,"LIGHTS OFF")
    def announce(self):   self._queue(config.CMD_ANNOUNCE, "ANNOUNCE")

    # ── Internal ──────────────────────────────────────────────────

    def _queue(self, cmd: bytes, label: str):
        if not self._loop:
            return
        if self._connected and self._char:
            asyncio.run_coroutine_threadsafe(
                self._send(cmd, label), self._loop)
        else:
            logger.debug(f"[{self._label}] '{label}' dropped — not connected")

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._main_loop())

    async def _main_loop(self):
        while not self._stop_event.is_set():
            if not self._connected:
                await self._connect()
            await asyncio.sleep(config.RECONNECT_INTERVAL)

    async def _connect(self):
        try:
            mac = self._mac
            if not mac:
                logger.info(f"[{self._label}] Scanning for '{self._name_prefix}'...")
                devices = await BleakScanner.discover(timeout=10.0)
                for d in devices:
                    name = (d.name or "").strip()
                    if name.upper().startswith(self._name_prefix.upper()):
                        mac = d.address
                        self._mac = mac
                        logger.info(f"[{self._label}] Found by name: {name} [{mac}]")
                        break
                if not mac:
                    logger.warning(f"[{self._label}] Not found — powered on?")
                    return

            logger.info(f"[{self._label}] Connecting to {mac}...")
            self._client = BleakClient(
                mac, disconnected_callback=self._on_disconnect)
            await self._client.connect(timeout=15.0)

            svc = self._client.services.get_service(config.SERVICE_UUID)
            if not svc:
                logger.error(f"[{self._label}] LionChief service not found")
                await self._client.disconnect()
                return

            self._char = svc.get_characteristic(config.CHARACTERISTIC_UUID)
            if not self._char:
                logger.error(f"[{self._label}] Characteristic not found")
                await self._client.disconnect()
                return

            self._connected   = True
            self.connect_count += 1
            logger.info(f"[{self._label}] ✅ Connected [{mac}] "
                        f"(#{self.connect_count})")

        except Exception as e:
            logger.error(f"[{self._label}] Connect failed: {e}")
            self._connected = False
            self._client    = None
            self._char      = None

    async def _send(self, cmd: bytes, label: str):
        try:
            if self._client and self._connected and self._char:
                await self._client.write_gatt_char(
                    self._char.uuid, cmd, response=False)
                self.cmd_count += 1
                logger.info(f"[{self._label}] ▶ {label:25s} "
                            f"[{' '.join(f'{b:02X}' for b in cmd)}]")
        except Exception as e:
            logger.error(f"[{self._label}] Send '{label}' failed: {e}")
            self._connected = False

    async def _disconnect(self):
        try:
            if self._client and self._client.is_connected:
                await self._client.disconnect()
        except Exception:
            pass
        self._connected = False
        self._char      = None

    def _on_disconnect(self, client):
        self._connected = False
        self._char      = None
        logger.warning(f"[{self._label}] Disconnected — reconnecting in 5s")


# ══════════════════════════════════════════════════════════════════
#  DUAL TRAIN CONTROLLER — wraps both trains with joint API
# ══════════════════════════════════════════════════════════════════

class DualTrainController:
    """
    Manages both BLE trains as a single unit.

    train_a → front train (controlled but normally runs freely)
    train_b → rear train  (primary follower — adjusted by gap)

    Cooperative control:
        When gap is closing → slow B, optionally speed up A
        When gap is opening → slow A, optionally speed up B
        Emergency          → STOP BOTH immediately
    """

    def __init__(self):
        self.train_a = SingleTrainBLE(
            config.TRAIN_A_MAC, config.TRAIN_A_NAME, "TrainA")
        self.train_b = SingleTrainBLE(
            config.TRAIN_B_MAC, config.TRAIN_B_NAME, "TrainB")

    def start(self):
        """Start both BLE threads."""
        self.train_a.start()
        self.train_b.start()
        logger.info("Dual BLE controller started")

    def shutdown(self):
        """Stop both trains cleanly."""
        logger.info("Shutting down dual BLE controller...")
        self.train_a.shutdown()
        self.train_b.shutdown()

    @property
    def connected_a(self) -> bool:
        return self.train_a.connected

    @property
    def connected_b(self) -> bool:
        return self.train_b.connected

    @property
    def connected_both(self) -> bool:
        return self.train_a.connected and self.train_b.connected

    def emergency_stop_all(self):
        """Stop BOTH trains immediately — bypasses rate limiting."""
        self.train_a.emergency_stop()
        self.train_b.emergency_stop()
        logger.warning("EMERGENCY STOP — both trains")

    def set_speeds(self, speed_a: int, speed_b: int):
        """
        Set both train speeds.
        Each train's rate-limiting is checked independently.
        """
        speed_a = max(0, min(7, speed_a))
        speed_b = max(0, min(7, speed_b))

        if self.train_a.should_send(speed_a):
            self.train_a.set_speed(speed_a)

        if self.train_b.should_send(speed_b):
            self.train_b.set_speed(speed_b)

    def keepalive(self):
        """Send keepalive to both trains."""
        self.train_a.keepalive()
        self.train_b.keepalive()

    def set_user_speed_a(self, speed: int):
        self.train_a.user_speed = max(1, min(7, speed))

    def set_user_speed_b(self, speed: int):
        self.train_b.user_speed = max(1, min(7, speed))

    def set_user_speeds(self, speed: int):
        """Set same cruising speed for both trains."""
        self.set_user_speed_a(speed)
        self.set_user_speed_b(speed)
