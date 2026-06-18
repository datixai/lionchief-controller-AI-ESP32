# ══════════════════════════════════════════════════════════════════
#  ble_controller.py  —  Train B BLE Controller
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
# ══════════════════════════════════════════════════════════════════

import asyncio
import threading
import logging
import time
from bleak import BleakClient, BleakScanner

import config

logger = logging.getLogger("BLE")


class TrainBLEController:

    def __init__(self):
        self._client       = None
        self._char         = None
        self._connected    = False
        self._mac          = config.TRAIN_B_MAC.strip()
        self._loop         = None
        self._thread       = None
        self._stop_event   = threading.Event()
        self.current_speed = 0
        self.user_speed    = config.DEFAULT_SPEED
        self.connect_count = 0

    def start(self):
        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name="BLE")
        self._thread.start()

    def shutdown(self):
        self.send_stop()
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

    def set_speed(self, speed: int):
        speed = max(0, min(7, speed))
        self.current_speed = speed
        if speed > 0:
            self.user_speed = speed
        self._queue(config.SPEED_CMDS[speed], f"SPEED {speed}")

    def send_stop(self):
        if self.current_speed > 0:
            self.user_speed = self.current_speed
        self.current_speed = 0
        self._queue(config.CMD_STOP, "STOP")

    def send_stop_raw(self):
        self._queue(config.CMD_STOP, "STOP")

    def keepalive(self):
        if self.current_speed > 0:
            self._queue(config.SPEED_CMDS[self.current_speed], "keepalive")

    def horn_on(self):    self._queue(config.CMD_HORN_ON,  "HORN ON")
    def horn_off(self):   self._queue(config.CMD_HORN_OFF, "HORN OFF")
    def bell_on(self):    self._queue(config.CMD_BELL_ON,  "BELL ON")
    def bell_off(self):   self._queue(config.CMD_BELL_OFF, "BELL OFF")
    def lights_on(self):  self._queue(config.CMD_LIGHT_ON, "LIGHTS ON")
    def lights_off(self): self._queue(config.CMD_LIGHT_OFF,"LIGHTS OFF")
    def announce(self):   self._queue(config.CMD_ANNOUNCE, "ANNOUNCE")

    def _queue(self, cmd: bytes, label: str):
        if not self._loop:
            return
        if self._connected and self._char:
            asyncio.run_coroutine_threadsafe(
                self._send(cmd, label), self._loop)
        else:
            logger.debug(f"'{label}' dropped — not connected")

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
                devices = await BleakScanner.discover(timeout=10.0)
                for d in devices:
                    if (d.name or "").startswith(config.TRAIN_B_NAME):
                        mac = d.address
                        break
                if not mac:
                    logger.warning("Train B not found — is it on?")
                    return

            logger.info(f"Connecting to Train B: {mac}")
            self._client = BleakClient(
                mac, disconnected_callback=self._on_disconnect)
            await self._client.connect(timeout=15.0)

            svc = self._client.services.get_service(config.SERVICE_UUID)
            if not svc:
                logger.error("LionChief service not found")
                await self._client.disconnect()
                return

            self._char = svc.get_characteristic(config.CHARACTERISTIC_UUID)
            if not self._char:
                logger.error("Characteristic not found")
                await self._client.disconnect()
                return

            self._connected   = True
            self.connect_count += 1
            logger.info(f"Train B connected [{mac}]")

        except Exception as e:
            logger.error(f"BLE connect failed: {e}")
            self._connected = False
            self._client    = None
            self._char      = None

    async def _send(self, cmd: bytes, label: str):
        try:
            if self._client and self._connected and self._char:
                await self._client.write_gatt_char(
                    self._char.uuid, cmd, response=False)
                logger.info(f"BLE ▶ {label}  "
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
        logger.warning("Train B disconnected — reconnecting in 5s")
