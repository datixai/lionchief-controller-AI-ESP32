# ══════════════════════════════════════════════════════════════════
#  ble_controller.py  —  LionChief BLE Controller
#
#  Connects directly from laptop to train via Bluetooth.
#  No ESP32 needed for the outer BLE train.
#
#  Connection modes:
#    Mode A: TRAIN_MAC set  → connect directly by MAC
#    Mode B: TRAIN_MAC empty → scan for device with TRAIN_NAME prefix
#
#  Runs in background thread — never blocks camera/CV loop.
# ══════════════════════════════════════════════════════════════════

import asyncio
import threading
import logging
from bleak import BleakClient, BleakScanner

import config

logger = logging.getLogger("BLE")


class TrainBLEController:
    """
    BLE controller for outer LionChief train.

    Usage:
        ble = TrainBLEController()
        ble.start()          # connects in background
        ble.stop_train()     # immediate stop
        ble.resume_train()   # slow then ramp to medium
        ble.shutdown()       # clean disconnect
    """

    def __init__(self):
        self._client     = None
        self._char       = None
        self._connected  = False
        self._train_mac  = config.TRAIN_MAC.strip()
        self._loop       = None
        self._thread     = None
        self._stop_event = threading.Event()

    # ── Public API ────────────────────────────────────────────────

    def start(self):
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("BLE controller started")

    def stop_train(self):
        """Send immediate STOP."""
        self._queue(config.CMD_STOP, "STOP")

    def resume_train(self):
        """Resume at slow speed (main.py handles the ramp to full speed)."""
        self._queue(config.CMD_SPEED_2, "RESUME slow")

    def resume_full(self):
        """Resume at normal running speed."""
        self._queue(config.CMD_RESUME, "RESUME full")

    def horn(self):
        self._queue(bytes([0x00, 0x48, 0x01]), "HORN ON")

    def horn_off(self):
        self._queue(bytes([0x00, 0x48, 0x00]), "HORN OFF")

    def bell_on(self):
        self._queue(bytes([0x00, 0x47, 0x01]), "BELL ON")

    def bell_off(self):
        self._queue(bytes([0x00, 0x47, 0x00]), "BELL OFF")

    def lights_on(self):
        self._queue(bytes([0x00, 0x51, 0x01]), "LIGHTS ON")

    def lights_off(self):
        self._queue(bytes([0x00, 0x51, 0x00]), "LIGHTS OFF")

    def announce(self):
        self._queue(bytes([0x00, 0x4D, 0x00, 0x00]), "ANNOUNCE")

    def set_speed(self, level: int):
        level = max(0, min(7, level))
        self._queue(bytes([0x00, 0x45, level]), f"SPEED {level}")

    def forward(self):
        self._queue(bytes([0x00, 0x46, 0x01]), "FORWARD")

    def reverse(self):
        self._queue(bytes([0x00, 0x46, 0x02]), "REVERSE")

    def shutdown(self):
        self._stop_event.set()
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)
        if self._thread:
            self._thread.join(timeout=5)
        logger.info("BLE controller shut down")

    @property
    def connected(self) -> bool:
        return self._connected

    # ── Internal ──────────────────────────────────────────────────

    def _queue(self, cmd: bytes, label: str):
        """Thread-safe command dispatch to background loop."""
        if not self._loop:
            logger.warning(f"Cannot send {label} — BLE loop not ready")
            return
        if self._connected and self._char:
            asyncio.run_coroutine_threadsafe(
                self._send(cmd, label), self._loop)
        else:
            logger.warning(f"{label} — not connected")

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
            mac = self._train_mac

            if mac:
                logger.info(f"Connecting to MAC: {mac}")
            else:
                logger.info(f"Scanning for '{config.TRAIN_NAME}'...")
                devices = await BleakScanner.discover(timeout=10.0)
                for d in devices:
                    name = (d.name or "").strip()
                    if name:
                        logger.debug(f"  Found: {name} [{d.address}]")
                    if name.upper().startswith(config.TRAIN_NAME.upper()):
                        mac = d.address
                        self._train_mac = mac
                        logger.info(f"Train found: {name} [{mac}]")
                        break
                if not mac:
                    logger.warning("Train not found. Is it powered on?")
                    return

            self._client = BleakClient(mac,
                                       disconnected_callback=self._on_disconnect)
            await self._client.connect(timeout=15.0)

            svc = self._client.services.get_service(config.SERVICE_UUID)
            if not svc:
                logger.error("LionChief service not found on device")
                await self._client.disconnect()
                return

            self._char = svc.get_characteristic(config.CHARACTERISTIC_UUID)
            if not self._char:
                logger.error("LionChief characteristic not found")
                await self._client.disconnect()
                return

            self._connected = True
            logger.info(f"✅ Connected to train [{mac}]")

        except Exception as e:
            logger.error(f"BLE connection failed: {e}")
            self._connected = False
            self._client = None
            self._char = None

    async def _send(self, cmd: bytes, label: str):
        try:
            if self._client and self._connected and self._char:
                await self._client.write_gatt_char(
                    self._char.uuid, cmd, response=False)
                logger.info(f"BLE ▶ {label}  {list(cmd)}")
            else:
                logger.warning(f"Cannot send {label} — not connected")
        except Exception as e:
            logger.error(f"Send failed ({label}): {e}")
            self._connected = False

    async def _disconnect(self):
        try:
            if self._client and self._client.is_connected:
                await self._client.disconnect()
        except Exception:
            pass
        self._connected = False
        self._char = None

    def _on_disconnect(self, client):
        self._connected = False
        self._char = None
        logger.warning("Train disconnected — reconnecting...")
