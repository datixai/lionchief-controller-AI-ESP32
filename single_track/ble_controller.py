# ══════════════════════════════════════════════════════════════════
#  ble_controller.py  —  LionChief BLE Controller (Train B only)
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  Controls Train B (rear BLE train) via Bluetooth.
#  Train A runs freely — no BLE needed for it.
#
#  Runs in a background asyncio thread — never blocks camera loop.
#
#  To change train: update TRAIN_B_MAC in config.py only.
#  All BLE protocol (UUIDs, commands) is identical across LionChief.
# ══════════════════════════════════════════════════════════════════

import asyncio
import threading
import logging
import time
from bleak import BleakClient, BleakScanner

import config

logger = logging.getLogger("BLE")


class TrainBLEController:
    """
    Async BLE controller for the rear LionChief train (Train B).

    Thread-safe: all public methods can be called from the camera
    thread while the BLE loop runs in its own background thread.
    """

    def __init__(self):
        self._client       = None
        self._char         = None
        self._connected    = False
        self._train_mac    = config.TRAIN_B_MAC.strip()
        self._loop         = None
        self._thread       = None
        self._stop_event   = threading.Event()

        # Speed tracking
        self.current_speed = 0
        self.user_speed    = config.DEFAULT_SPEED

        # Statistics
        self.connect_count = 0
        self.cmd_count     = 0

    # ── Lifecycle ─────────────────────────────────────────────────

    def start(self):
        """Start background BLE thread and begin connecting."""
        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name="BLE-TrainB")
        self._thread.start()
        logger.info(f"BLE started — target: {self._train_mac}")

    def shutdown(self):
        """Send stop, disconnect cleanly, join thread."""
        self.set_speed(0)
        time.sleep(0.4)
        self._stop_event.set()
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._disconnect(), self._loop)
        if self._thread:
            self._thread.join(timeout=5)
        logger.info("BLE shut down")

    @property
    def connected(self) -> bool:
        return self._connected

    # ── Speed commands ─────────────────────────────────────────────

    def set_speed(self, speed: int):
        """
        Send speed command to Train B.
        Updates current_speed and user_speed.
        user_speed is never set to 0 (prevents restore-to-zero bug).
        """
        speed = max(0, min(7, speed))
        self.current_speed = speed
        if speed > 0:
            self.user_speed = speed
        self._queue(config.SPEED_CMDS[speed], f"SPEED {speed}")

    def emergency_stop(self):
        """Send STOP immediately — bypasses normal rate limiting."""
        self.current_speed = 0
        self._queue(config.CMD_STOP, "EMERGENCY STOP")

    def keepalive(self):
        """Send keepalive — only if train is moving (never sends speed 0)."""
        if self.current_speed > 0:
            self._queue(
                config.SPEED_CMDS[self.current_speed], "keepalive")

    # ── Accessory commands ─────────────────────────────────────────

    def forward(self):       self._queue(config.CMD_FORWARD,   "FORWARD")
    def reverse(self):       self._queue(config.CMD_REVERSE,   "REVERSE")
    def horn_on(self):       self._queue(config.CMD_HORN_ON,   "HORN ON")
    def horn_off(self):      self._queue(config.CMD_HORN_OFF,  "HORN OFF")
    def bell_on(self):       self._queue(config.CMD_BELL_ON,   "BELL ON")
    def bell_off(self):      self._queue(config.CMD_BELL_OFF,  "BELL OFF")
    def lights_on(self):     self._queue(config.CMD_LIGHT_ON,  "LIGHTS ON")
    def lights_off(self):    self._queue(config.CMD_LIGHT_OFF, "LIGHTS OFF")
    def sound_on(self):      self._queue(config.CMD_SOUND_ON,  "SOUND ON")
    def sound_off(self):     self._queue(config.CMD_SOUND_OFF, "SOUND OFF")
    def announce(self):      self._queue(config.CMD_ANNOUNCE,  "ANNOUNCE")

    # ── Internal ──────────────────────────────────────────────────

    def _queue(self, cmd: bytes, label: str):
        """Thread-safe dispatch to background asyncio loop."""
        if not self._loop:
            logger.warning(f"Cannot send '{label}' — loop not ready")
            return
        if self._connected and self._char:
            asyncio.run_coroutine_threadsafe(
                self._send(cmd, label), self._loop)
        else:
            logger.warning(f"'{label}' dropped — not connected")

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
        """Connect to Train B. Tries direct MAC, then name scan."""
        try:
            mac = self._train_mac
            if not mac:
                logger.info(f"Scanning for '{config.TRAIN_B_NAME}'...")
                devices = await BleakScanner.discover(timeout=10.0)
                for d in devices:
                    name = (d.name or "").strip()
                    logger.debug(f"  Scan: {name} [{d.address}]")
                    if name.upper().startswith(
                            config.TRAIN_B_NAME.upper()):
                        mac = d.address
                        self._train_mac = mac
                        logger.info(f"Found by name: {name} [{mac}]")
                        break
                if not mac:
                    logger.warning("Train B not found — is it powered on?")
                    return

            logger.info(f"Connecting to Train B: {mac}")
            self._client = BleakClient(
                mac, disconnected_callback=self._on_disconnect)
            await self._client.connect(timeout=15.0)

            svc = self._client.services.get_service(config.SERVICE_UUID)
            if not svc:
                logger.error("LionChief service not found — wrong device?")
                await self._client.disconnect()
                return

            self._char = svc.get_characteristic(
                config.CHARACTERISTIC_UUID)
            if not self._char:
                logger.error("LionChief characteristic not found")
                await self._client.disconnect()
                return

            self._connected  = True
            self.connect_count += 1
            logger.info(f"✅ Train B connected [{mac}] "
                        f"(connect #{self.connect_count})")

        except Exception as e:
            logger.error(f"BLE connect failed: {e}")
            self._connected = False
            self._client    = None
            self._char      = None

    async def _send(self, cmd: bytes, label: str):
        """Write GATT characteristic. Marks disconnected on failure."""
        try:
            if self._client and self._connected and self._char:
                await self._client.write_gatt_char(
                    self._char.uuid, cmd, response=False)
                self.cmd_count += 1
                logger.info(
                    f"BLE ▶ {label:30s}  "
                    f"[{' '.join(f'{b:02X}' for b in cmd)}]")
        except Exception as e:
            logger.error(f"Send '{label}' failed: {e}")
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
        logger.warning("Train B disconnected — auto-reconnect in 5s")
