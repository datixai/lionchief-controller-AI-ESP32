# ══════════════════════════════════════════════════════════════════
#  ble_controller.py  —  LionChief BLE Controller
#
#  Connects directly from laptop to train via Bluetooth.
#  No ESP32 needed — laptop IS the BLE controller.
#
#  Connection logic:
#    1. If TRAIN_MAC is set  → connect directly by MAC address
#    2. If TRAIN_MAC is empty → scan and find device by name prefix
#
#  Runs in a background thread so it never blocks the camera/CV loop.
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

    Usage:
        controller = TrainBLEController()
        controller.start()           # connects in background thread
        controller.stop_train()      # send STOP
        controller.resume_train()    # send RESUME
        controller.shutdown()        # clean disconnect
    """

    def __init__(self):
        self._client      = None
        self._char        = None
        self._connected   = False
        self._train_mac   = config.TRAIN_MAC.strip()   # confirmed MAC
        self._loop        = None
        self._thread      = None
        self._stop_event  = threading.Event()

    # ── PUBLIC API ────────────────────────────────────────────────

    def start(self):
        """Start background BLE thread and connect to train."""
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("BLE controller thread started")

    def stop_train(self):
        """Send STOP command — immediate stop."""
        self._queue_cmd(config.CMD_STOP, "STOP")

    def resume_train(self):
        """Send RESUME command — train moves again."""
        self._queue_cmd(config.CMD_RESUME, "RESUME")

    def horn(self):
        self._queue_cmd(bytes([0x00, 0x48, 0x01]), "HORN ON")

    def horn_off(self):
        self._queue_cmd(bytes([0x00, 0x48, 0x00]), "HORN OFF")

    def bell_on(self):
        self._queue_cmd(bytes([0x00, 0x47, 0x01]), "BELL ON")

    def bell_off(self):
        self._queue_cmd(bytes([0x00, 0x47, 0x00]), "BELL OFF")

    def announce(self):
        self._queue_cmd(bytes([0x00, 0x4D, 0x00, 0x00]), "ANNOUNCE")

    def lights_on(self):
        self._queue_cmd(bytes([0x00, 0x51, 0x01]), "LIGHTS ON")

    def lights_off(self):
        self._queue_cmd(bytes([0x00, 0x51, 0x00]), "LIGHTS OFF")

    def set_speed(self, level: int):
        """Set speed 0–7. 0=stop, 7=max."""
        level = max(0, min(7, level))
        self._queue_cmd(bytes([0x00, 0x45, level]), f"SPEED {level}")

    def forward(self):
        self._queue_cmd(bytes([0x00, 0x46, 0x01]), "FORWARD")

    def reverse(self):
        self._queue_cmd(bytes([0x00, 0x46, 0x02]), "REVERSE")

    def shutdown(self):
        """Gracefully disconnect and stop background thread."""
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

    def _queue_cmd(self, cmd: bytes, label: str):
        """Thread-safe way to send a command from outside the event loop."""
        if not self._loop:
            logger.warning(f"Cannot send {label} — BLE loop not started yet")
            return
        if self._connected and self._char:
            asyncio.run_coroutine_threadsafe(
                self._send(cmd, label), self._loop)
        else:
            logger.warning(f"{label} requested but train not connected")

    def _run_loop(self):
        """Background thread: owns the asyncio event loop."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._main_loop())

    async def _main_loop(self):
        """Keep connected — reconnect whenever connection drops."""
        while not self._stop_event.is_set():
            if not self._connected:
                await self._connect()
            await asyncio.sleep(config.RECONNECT_INTERVAL)

    async def _connect(self):
        """
        Connect to the train.

        Mode A: TRAIN_MAC is set → connect directly by MAC address.
        Mode B: TRAIN_MAC empty  → scan for device with TRAIN_NAME prefix.
        """
        try:
            mac = self._train_mac

            if mac:
                # ── Mode A: direct MAC connection ─────────────────
                logger.info(f"Connecting to train at MAC: {mac}")
            else:
                # ── Mode B: auto-discover by name ──────────────────
                logger.info(f"Scanning for train with name prefix '{config.TRAIN_NAME}'...")
                devices = await BleakScanner.discover(timeout=10.0)
                for d in devices:
                    name = (d.name or "").strip()
                    if name:
                        logger.debug(f"  Found: {name!r} [{d.address}]")
                    if name.upper().startswith(config.TRAIN_NAME.upper()):
                        mac = d.address
                        self._train_mac = mac
                        logger.info(f"Train found by name: {name!r} [{mac}]")
                        break
                if not mac:
                    logger.warning(
                        f"No train found with name prefix '{config.TRAIN_NAME}'. "
                        "Is the train powered on?")
                    return

            # ── Establish BLE connection ───────────────────────────
            self._client = BleakClient(
                mac,
                disconnected_callback=self._on_disconnect
            )
            await self._client.connect(timeout=15.0)

            # ── Get the LionChief service ──────────────────────────
            service = self._client.services.get_service(config.SERVICE_UUID)
            if not service:
                logger.error("LionChief service UUID not found on this device!")
                logger.error(f"Expected: {config.SERVICE_UUID}")
                await self._client.disconnect()
                return

            # ── Get the write characteristic ──────────────────────
            self._char = service.get_characteristic(config.CHARACTERISTIC_UUID)
            if not self._char:
                logger.error("LionChief characteristic not found!")
                await self._client.disconnect()
                return

            self._connected = True
            logger.info(f"✅ Connected to train at {mac} — all commands ready!")

        except Exception as e:
            logger.error(f"BLE connection failed: {e}")
            self._connected = False
            self._client = None
            self._char = None

    async def _send(self, cmd: bytes, label: str):
        """Write command bytes to the train characteristic."""
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
        if self._client:
            try:
                if self._client.is_connected:
                    await self._client.disconnect()
            except Exception:
                pass
        self._connected = False
        self._char = None

    def _on_disconnect(self, client):
        """Called automatically by bleak when BLE connection drops."""
        self._connected = False
        self._char = None
        logger.warning("Train disconnected — will reconnect automatically")
