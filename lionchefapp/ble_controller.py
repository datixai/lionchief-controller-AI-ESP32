# ══════════════════════════════════════════════════════════════════
#  ble_controller.py  —  Dual BLE Controller
#  Harry Locomotive Project 3 BT  |  Datix AI  |  June 2026
#
#  SingleTrainBLE  — controls one LionChief train independently.
#  DualBLEController — wraps both trains with combined API.
#
#  Each train has its OWN:
#    - BLE connection thread
#    - current_speed / user_speed tracking
#    - set_speed(), send_stop(), keepalive(), horn, lights etc.
#
#  Commands NEVER cross between trains.
#  Train A keys only send to Train A.
#  Train B keys only send to Train B.
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
    Completely independent — no shared state with the other train.
    """

    def __init__(self, mac: str, name: str, label: str,
                 default_speed: int):
        self._mac          = mac.strip()
        self._name         = name
        self._label        = label          # "A" or "B"
        self._client       = None
        self._char         = None
        self._connected    = False
        self._loop         = None
        self._thread       = None
        self._stop_event   = threading.Event()

        self.current_speed = 0
        self.user_speed    = default_speed
        self.connect_count = 0

    # ── Lifecycle ─────────────────────────────────────────────────

    def start(self):
        self._thread = threading.Thread(
            target=self._run_loop,
            daemon=True,
            name=f"BLE-Train{self._label}")
        self._thread.start()
        logger.info(f"[Train {self._label}] BLE started — {self._mac}")

    def shutdown(self):
        self.send_stop_raw()
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

    # ── Speed commands ─────────────────────────────────────────────

    def set_speed(self, speed: int):
        """Set speed 0-7. Saves user_speed if speed > 0."""
        speed = max(0, min(7, speed))
        self.current_speed = speed
        if speed > 0:
            self.user_speed = speed
        self._queue(config.SPEED_CMDS[speed],
                    f"Train{self._label} SPEED {speed}")

    def set_speed_no_save(self, speed: int):
        """Set speed WITHOUT updating user_speed (used for ESCAPE adjustments)."""
        speed = max(0, min(7, speed))
        self.current_speed = speed
        self._queue(config.SPEED_CMDS[speed],
                    f"Train{self._label} TEMP {speed}")

    def send_stop(self):
        """Collision stop — saves user_speed before stopping."""
        if self.current_speed > 0:
            self.user_speed = self.current_speed
        self.current_speed = 0
        self._queue(config.CMD_STOP, f"Train{self._label} STOP")

    def send_stop_raw(self):
        """Stop without touching user_speed (for repeated stops)."""
        self.current_speed = 0
        self._queue(config.CMD_STOP, f"Train{self._label} STOP-RAW")

    def resume(self):
        """Resume at last user_speed."""
        self.set_speed(self.user_speed)

    def keepalive(self):
        if self.current_speed > 0:
            self._queue(config.SPEED_CMDS[self.current_speed],
                        f"Train{self._label} keepalive")

    # ── Accessory commands ─────────────────────────────────────────

    def horn_on(self):    self._queue(config.CMD_HORN_ON,  f"Train{self._label} HORN ON")
    def horn_off(self):   self._queue(config.CMD_HORN_OFF, f"Train{self._label} HORN OFF")
    def bell_on(self):    self._queue(config.CMD_BELL_ON,  f"Train{self._label} BELL ON")
    def bell_off(self):   self._queue(config.CMD_BELL_OFF, f"Train{self._label} BELL OFF")
    def lights_on(self):  self._queue(config.CMD_LIGHT_ON, f"Train{self._label} LIGHTS ON")
    def lights_off(self): self._queue(config.CMD_LIGHT_OFF,f"Train{self._label} LIGHTS OFF")
    def announce(self):   self._queue(config.CMD_ANNOUNCE, f"Train{self._label} ANNOUNCE")

    # ── Internal BLE ──────────────────────────────────────────────

    def _queue(self, cmd: bytes, label: str):
        if not self._loop:
            return
        if self._connected and self._char:
            asyncio.run_coroutine_threadsafe(
                self._send(cmd, label), self._loop)
        else:
            logger.debug(f"[Train{self._label}] '{label}' dropped — not connected")

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
                logger.info(f"[Train{self._label}] Scanning for '{self._name}'...")
                devices = await BleakScanner.discover(timeout=10.0)
                for d in devices:
                    if (d.name or "").startswith(self._name):
                        mac = d.address
                        self._mac = mac
                        break
                if not mac:
                    logger.warning(
                        f"[Train{self._label}] Not found — powered on?")
                    return

            logger.info(f"[Train{self._label}] Connecting to {mac}...")
            self._client = BleakClient(
                mac, disconnected_callback=self._on_disconnect)
            await self._client.connect(timeout=15.0)

            svc = self._client.services.get_service(config.SERVICE_UUID)
            if not svc:
                logger.error(
                    f"[Train{self._label}] LionChief service not found")
                await self._client.disconnect()
                return

            self._char = svc.get_characteristic(
                config.CHARACTERISTIC_UUID)
            if not self._char:
                logger.error(
                    f"[Train{self._label}] Characteristic not found")
                await self._client.disconnect()
                return

            self._connected   = True
            self.connect_count += 1
            logger.info(
                f"[Train{self._label}] Connected [{mac}] "
                f"(#{self.connect_count})")

        except Exception as e:
            logger.error(f"[Train{self._label}] Connect failed: {e}")
            self._connected = False
            self._client    = None
            self._char      = None

    async def _send(self, cmd: bytes, label: str):
        try:
            if self._client and self._connected and self._char:
                await self._client.write_gatt_char(
                    self._char.uuid, cmd, response=False)
                logger.info(
                    f"[Train{self._label}] >> {label}  "
                    f"[{' '.join(f'{b:02X}' for b in cmd)}]")
        except Exception as e:
            logger.error(
                f"[Train{self._label}] Send '{label}' failed: {e}")
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
        logger.warning(
            f"[Train{self._label}] Disconnected — reconnecting...")


# ══════════════════════════════════════════════════════════════════
#  DualBLEController — wraps both trains with unified API
# ══════════════════════════════════════════════════════════════════

class DualBLEController:
    """
    Manages both BLE trains independently.
    Train A and Train B have completely separate BLE threads.
    Commands NEVER cross between trains.
    """

    def __init__(self):
        self.train_a = SingleTrainBLE(
            config.TRAIN_A_MAC, config.TRAIN_A_NAME,
            "A", config.DEFAULT_SPEED_A)
        self.train_b = SingleTrainBLE(
            config.TRAIN_B_MAC, config.TRAIN_B_NAME,
            "B", config.DEFAULT_SPEED_B)

    def start(self):
        self.train_a.start()
        self.train_b.start()
        logger.info("Dual BLE controller started")

    def shutdown(self):
        logger.info("Shutting down both BLE trains...")
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

    def emergency_stop_both(self):
        """Stop BOTH trains immediately — used in DANGER zone."""
        self.train_a.send_stop_raw()
        self.train_b.send_stop_raw()
        logger.warning("[DANGER] Emergency stop — BOTH trains")

    def keepalive(self):
        self.train_a.keepalive()
        self.train_b.keepalive()
