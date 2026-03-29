# ══════════════════════════════════════════════════════════════════
#  ble_controller.py  —  LionChief BLE Controller
#  Runs entirely on laptop — no ESP32 needed
# ══════════════════════════════════════════════════════════════════

import asyncio
import threading
import time
import logging
from bleak import BleakClient, BleakScanner

import config

logger = logging.getLogger("BLE")


class TrainBLEController:
    """
    Manages BLE connection to the LionChief train directly from laptop.
    Runs in a background thread so it never blocks the camera/CV code.

    Usage:
        controller = TrainBLEController()
        controller.start()           # connects in background
        controller.stop_train()      # send stop command
        controller.resume_train()    # send resume command
        controller.shutdown()        # clean disconnect
    """

    def __init__(self):
        self._client       = None
        self._char         = None
        self._connected    = False
        self._train_mac    = config.TRAIN_MAC
        self._loop         = None
        self._thread       = None
        self._stop_event   = threading.Event()
        self._command_queue = asyncio.Queue() if False else None  # set in start()

    # ── PUBLIC API ────────────────────────────────────────────────

    def start(self):
        """Start background BLE thread and connect to train."""
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("BLE controller thread started")

    def stop_train(self):
        """Send STOP command to train."""
        if self._connected and self._char:
            asyncio.run_coroutine_threadsafe(
                self._send(config.CMD_STOP, "STOP"), self._loop)
        else:
            logger.warning("STOP requested but not connected")

    def resume_train(self):
        """Send RESUME command to train."""
        if self._connected and self._char:
            asyncio.run_coroutine_threadsafe(
                self._send(config.CMD_RESUME, "RESUME"), self._loop)
        else:
            logger.warning("RESUME requested but not connected")

    def horn(self):
        asyncio.run_coroutine_threadsafe(
            self._send(bytes([0x00, 0x48, 0x01]), "HORN ON"), self._loop)

    def bell_on(self):
        asyncio.run_coroutine_threadsafe(
            self._send(bytes([0x00, 0x47, 0x01]), "BELL ON"), self._loop)

    def bell_off(self):
        asyncio.run_coroutine_threadsafe(
            self._send(bytes([0x00, 0x47, 0x00]), "BELL OFF"), self._loop)

    def set_speed(self, level: int):
        """Set speed 0-7. 0=stop, 7=max."""
        level = max(0, min(7, level))
        asyncio.run_coroutine_threadsafe(
            self._send(bytes([0x00, 0x45, level]), f"SPEED {level}"), self._loop)

    def shutdown(self):
        """Gracefully disconnect and stop thread."""
        self._stop_event.set()
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)
        if self._thread:
            self._thread.join(timeout=5)
        logger.info("BLE controller shut down")

    @property
    def connected(self) -> bool:
        return self._connected

    # ── INTERNAL ──────────────────────────────────────────────────

    def _run_loop(self):
        """Background thread: runs asyncio event loop."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._main_loop())

    async def _main_loop(self):
        """Keep trying to connect; reconnect if dropped."""
        while not self._stop_event.is_set():
            if not self._connected:
                await self._connect()
            await asyncio.sleep(config.RECONNECT_INTERVAL)

    async def _connect(self):
        """Scan for train by name or MAC and connect."""
        try:
            mac = self._train_mac

            # Auto-discover if no MAC set
            if not mac:
                logger.info(f"Scanning for train '{config.TRAIN_NAME}'...")
                devices = await BleakScanner.discover(timeout=10.0)
                for d in devices:
                    if d.name and config.TRAIN_NAME.lower() in d.name.lower():
                        mac = d.address
                        self._train_mac = mac
                        logger.info(f"Found train: {d.name} [{mac}]")
                        break
                if not mac:
                    logger.warning("Train not found in scan. Retrying...")
                    return

            logger.info(f"Connecting to {mac}...")
            self._client = BleakClient(mac, disconnected_callback=self._on_disconnect)
            await self._client.connect(timeout=15.0)

            # Get characteristic
            service = self._client.services.get_service(config.SERVICE_UUID)
            if not service:
                logger.error("LionChief service not found on device!")
                await self._client.disconnect()
                return

            self._char = service.get_characteristic(config.CHARACTERISTIC_UUID)
            if not self._char:
                logger.error("LionChief characteristic not found!")
                await self._client.disconnect()
                return

            self._connected = True
            logger.info(f"✅ Connected to train [{mac}] — ready to control!")

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            self._connected = False

    async def _send(self, cmd: bytes, label: str):
        """Write command bytes to train characteristic."""
        try:
            if self._client and self._connected and self._char:
                await self._client.write_gatt_char(
                    self._char.uuid, cmd, response=False)
                logger.info(f"BLE ▶ {label}  raw={list(cmd)}")
            else:
                logger.warning(f"Cannot send {label} — not connected")
        except Exception as e:
            logger.error(f"Send failed ({label}): {e}")
            self._connected = False

    async def _disconnect(self):
        if self._client and self._client.is_connected:
            await self._client.disconnect()
        self._connected = False

    def _on_disconnect(self, client):
        self._connected = False
        self._char = None
        logger.warning("Train disconnected — will reconnect automatically")
