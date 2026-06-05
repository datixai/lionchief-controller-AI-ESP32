# ══════════════════════════════════════════════════════════════════
#  ble_controller.py  —  LionChief BLE Controller
#  Harry Locomotive Project  |  Datix AI  |  May 2026
#
#  Controls outer LionChief train via Bluetooth.
#  New train: 60:F9:FB:49:94:C9  (LC-1-1-09F9-60F9)
#
#  KEY FEATURE — User Speed Restoration:
#    Before any collision STOP, currentSpeed is saved as userSetSpeed.
#    After zone clears, train resumes at exactly userSetSpeed — not
#    a hardcoded slow speed. Same behaviour as ESP32 v5.1.
#
#  Runs in a background asyncio thread — never blocks camera loop.
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
    BLE controller for outer LionChief train.

    Speed tracking:
        current_speed — speed currently commanded
        user_speed    — speed user intentionally set (restored after stop)

    Usage:
        ble = TrainBLEController()
        ble.start()
        ble.set_speed(5)         # user sets speed — saves to user_speed
        ble.send_stop()          # collision stop — saves user_speed first
        ble.resume_ramp()        # ramp to intermediate speed
        ble.resume_user_speed()  # return to exact user_speed
        ble.shutdown()
    """

    def __init__(self):
        self._client       = None
        self._char         = None
        self._connected    = False
        self._train_mac    = config.TRAIN_MAC.strip()
        self._loop         = None
        self._thread       = None
        self._stop_event   = threading.Event()

        # Speed tracking — mirrors ESP32 v5.1 logic
        self.current_speed = config.DEFAULT_OUTER_SPEED
        self.user_speed    = config.DEFAULT_OUTER_SPEED

        # Stats
        self.stops_sent    = 0
        self.resumes_sent  = 0
        self.connect_count = 0

    # ── Public API ────────────────────────────────────────────────

    def start(self):
        """Start background BLE thread and connect."""
        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name="BLE-Thread")
        self._thread.start()
        logger.info(f"BLE controller started — target: {self._train_mac}")

    @property
    def connected(self) -> bool:
        return self._connected

    # ── Speed commands ─────────────────────────────────────────────

    def set_speed(self, speed: int):
        """
        Set outer train speed (0-7).
        Updates both current_speed and user_speed.
        user_speed is NOT updated if speed == 0 (prevents restoring to 0).
        """
        speed = max(0, min(7, speed))
        self.current_speed = speed
        if speed > 0:
            self.user_speed = speed  # only save non-zero speed as restore target
        self._queue(config.SPEED_CMDS[speed], f"SPEED {speed}")

    def send_stop(self):
        """
        Collision prevention STOP.
        Saves current_speed to user_speed BEFORE setting to 0,
        so resume can restore it exactly.
        """
        if self.current_speed > 0:
            self.user_speed = self.current_speed
        self.current_speed = 0
        self.stops_sent += 1
        self._queue(config.CMD_STOP, "STOP outer train")

    def send_stop_raw(self):
        """
        Send STOP without affecting user_speed tracking.
        Use for repeated stops during zone lock — does not corrupt
        the userSetSpeed that will be restored on resume.
        """
        self._queue(config.CMD_STOP, "STOP repeated")

    def resume_ramp(self):
        """
        Resume at intermediate ramp speed (capped at user_speed).
        Prevents train briefly overshooting if user had set a low speed.
        """
        ramp = min(config.RESUME_RAMP_SPEED, self.user_speed)
        if ramp == 0:
            ramp = config.RESUME_RAMP_SPEED
        self.current_speed = ramp
        self.resumes_sent += 1
        self._queue(config.SPEED_CMDS[ramp], f"RESUME ramp speed {ramp}")

    def resume_user_speed(self):
        """Resume at exactly the speed the user had before the stop."""
        speed = self.user_speed
        if speed == 0:
            speed = config.DEFAULT_OUTER_SPEED
        self.current_speed = speed
        self._queue(config.SPEED_CMDS[speed], f"RESUME user speed {speed}")

    def keepalive(self):
        """
        Send keepalive ping — only if train should be moving.
        Never sends speed 0 (would stop the train silently).
        """
        if self.current_speed > 0:
            self._queue(config.SPEED_CMDS[self.current_speed], "keepalive")

    # ── Accessory commands ─────────────────────────────────────────

    def forward(self):
        self._queue(config.CMD_FORWARD,  "FORWARD")

    def reverse(self):
        self._queue(config.CMD_REVERSE,  "REVERSE")

    def horn_on(self):
        self._queue(config.CMD_HORN_ON,  "HORN ON")

    def horn_off(self):
        self._queue(config.CMD_HORN_OFF, "HORN OFF")

    def bell_on(self):
        self._queue(config.CMD_BELL_ON,  "BELL ON")

    def bell_off(self):
        self._queue(config.CMD_BELL_OFF, "BELL OFF")

    def lights_on(self):
        self._queue(config.CMD_LIGHT_ON,  "LIGHTS ON")

    def lights_off(self):
        self._queue(config.CMD_LIGHT_OFF, "LIGHTS OFF")

    def sound_on(self):
        self._queue(config.CMD_SOUND_ON,  "SOUND ON")

    def sound_off(self):
        self._queue(config.CMD_SOUND_OFF, "SOUND OFF")

    def announce(self):
        self._queue(config.CMD_ANNOUNCE, "ANNOUNCE")

    def shutdown(self):
        """Clean shutdown — send stop, disconnect."""
        self.send_stop()
        time.sleep(0.3)
        self._stop_event.set()
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)
        if self._thread:
            self._thread.join(timeout=5)
        logger.info("BLE controller shut down")

    # ── Internal ──────────────────────────────────────────────────

    def _queue(self, cmd: bytes, label: str):
        """Thread-safe command dispatch to the BLE asyncio loop."""
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
        """
        Connect to outer train.
        Tries direct MAC first, then scans by name prefix.
        """
        try:
            mac = self._train_mac

            if not mac:
                logger.info(f"Scanning for '{config.TRAIN_NAME}'...")
                devices = await BleakScanner.discover(timeout=10.0)
                for d in devices:
                    name = (d.name or "").strip()
                    logger.debug(f"  Scan found: {name} [{d.address}]")
                    if name.upper().startswith(config.TRAIN_NAME.upper()):
                        mac = d.address
                        self._train_mac = mac
                        logger.info(f"Train found by name: {name} [{mac}]")
                        break
                if not mac:
                    logger.warning("Train not found — is it powered on?")
                    return

            logger.info(f"Connecting to {mac}...")
            self._client = BleakClient(
                mac, disconnected_callback=self._on_disconnect)
            await self._client.connect(timeout=15.0)

            # Locate LionChief service
            svc = self._client.services.get_service(config.SERVICE_UUID)
            if not svc:
                logger.error(
                    f"LionChief service not found on {mac} — "
                    "wrong device or wrong UUID?")
                await self._client.disconnect()
                return

            # Locate write characteristic
            self._char = svc.get_characteristic(config.CHARACTERISTIC_UUID)
            if not self._char:
                logger.error("LionChief characteristic not found")
                await self._client.disconnect()
                return

            self._connected  = True
            self.connect_count += 1
            logger.info(f"✅ Connected to outer train [{mac}] "
                        f"(connect #{self.connect_count})")

            # Send initial speed so train doesn't start from 0
            await self._send(
                config.SPEED_CMDS[self.current_speed],
                f"initial speed {self.current_speed}")

        except Exception as e:
            logger.error(f"BLE connection failed: {e}")
            self._connected = False
            self._client    = None
            self._char      = None

    async def _send(self, cmd: bytes, label: str):
        """Write a GATT command. Marks disconnected on failure."""
        try:
            if self._client and self._connected and self._char:
                await self._client.write_gatt_char(
                    self._char.uuid, cmd, response=False)
                logger.info(f"BLE ▶ {label:38s}  [{' '.join(f'{b:02X}' for b in cmd)}]")
            else:
                logger.warning(f"Cannot send '{label}' — client gone")
        except Exception as e:
            logger.error(f"Send failed ('{label}'): {e}")
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
        logger.warning("Train disconnected — auto-reconnect in 5s")