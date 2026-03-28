"""
LionChief BLE Controller
=========================
Combines the best patterns from:
  - Property404/lionchief-controller  — original protocol + checksum
  - idaband/lionchief-controller-raspberrypi  — threaded ramp, extended cmds
  - chrcraven/LionchiefInteractiveDisplay  — async reconnect loop, keepalive
  - hbldh/bleak  — modern cross-platform BLE (replaces pygatt / gatttool)

Usage:
    import asyncio
    from lionchief.controller import LionChiefController

    async def main():
        ctrl = LionChiefController("AA:BB:CC:DD:EE:FF")
        await ctrl.connect()
        await ctrl.stop()
        await ctrl.set_speed(7)
        await ctrl.disconnect()

    asyncio.run(main())
"""

import asyncio
import logging
import time
from typing import Optional

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

from .commands import (
    SERVICE_UUID, CHARACTERISTIC_UUID,
    STOP_CMD, MEDIUM_CMD, FORWARD_CMD,
    HORN_ON_CMD, HORN_OFF_CMD,
    BELL_ON_CMD, BELL_OFF_CMD,
    LIGHT_ON_CMD, LIGHT_OFF_CMD,
    SPEAK_CMD, DISCO_CMD,
    build_speed_cmd, build_volume_cmd,
)

logger = logging.getLogger(__name__)


class LionChiefController:
    """
    Async BLE controller for Lionel LionChief trains.

    Handles:
      - Auto-discovery (no MAC needed)
      - Auto-reconnect on connection drop
      - Keepalive pings to stop the train making noise when idle
      - Thread-safe speed ramping
      - All known LionChief commands
    """

    RECONNECT_DELAY   = 5.0   # seconds between reconnect attempts
    KEEPALIVE_INTERVAL= 25.0  # seconds — send current speed to keep BLE alive
    MAX_RETRIES       = 10    # max reconnect attempts before giving up

    def __init__(self, mac_address: Optional[str] = None, auto_connect: bool = True):
        """
        Args:
            mac_address:  BLE MAC of the train.  If None, auto-scans for first
                          LionChief device found.
            auto_connect: If True, start a background reconnect loop.
        """
        self.mac_address   = mac_address
        self.auto_connect  = auto_connect
        self._client:  Optional[BleakClient] = None
        self._connected    = False
        self._current_speed= 0
        self._direction    = "forward"
        self._lock         = asyncio.Lock()
        self._reconnect_task: Optional[asyncio.Task] = None
        self._should_run   = True
        self._last_keepalive = 0.0

    # ── Connection management ─────────────────────────────────────────────────

    async def connect(self) -> bool:
        """Connect to the train. Returns True on success."""
        try:
            if not self.mac_address:
                logger.info("No MAC supplied — scanning for LionChief train...")
                self.mac_address = await self._discover_train()
                if not self.mac_address:
                    logger.error("No LionChief train found during scan.")
                    return False

            logger.info(f"Connecting to train at {self.mac_address} ...")
            self._client = BleakClient(
                self.mac_address,
                disconnected_callback=self._on_disconnect,
            )
            await self._client.connect(timeout=15.0)
            self._connected = True
            self._last_keepalive = time.time()
            logger.info(f"✅ Connected to LionChief at {self.mac_address}")

            if self.auto_connect and self._reconnect_task is None:
                self._reconnect_task = asyncio.create_task(self._keepalive_loop())

            return True

        except (BleakError, asyncio.TimeoutError) as e:
            logger.error(f"Connection failed: {e}")
            self._connected = False
            return False

    async def disconnect(self) -> None:
        """Gracefully disconnect from the train."""
        self._should_run = False
        if self._reconnect_task:
            self._reconnect_task.cancel()
            self._reconnect_task = None
        if self._client and self._connected:
            try:
                await self._write(DISCO_CMD)
                await self._client.disconnect()
            except Exception:
                pass
        self._connected = False
        logger.info("Disconnected from train.")

    def _on_disconnect(self, client: BleakClient) -> None:
        """Called automatically by bleak when BLE drops."""
        logger.warning("⚠️  BLE connection lost — will attempt reconnect")
        self._connected = False

    async def _discover_train(self, timeout: float = 10.0) -> Optional[str]:
        """
        Scan for nearby BLE devices and return MAC of first LionChief found.
        LionChief trains advertise with the known service UUID.
        """
        logger.info(f"Scanning {timeout}s for LionChief service UUID...")
        devices = await BleakScanner.discover(timeout=timeout)
        for d in devices:
            name = d.name or ""
            logger.debug(f"  Found: {name!r}  [{d.address}]")
            if "lion" in name.lower() or "chief" in name.lower():
                logger.info(f"Found LionChief: {name!r} [{d.address}]")
                return d.address
            # Also match by service UUID advertisement
            if d.metadata and SERVICE_UUID.lower() in str(d.metadata).lower():
                logger.info(f"Found LionChief by UUID: {name!r} [{d.address}]")
                return d.address
        return None

    async def _keepalive_loop(self) -> None:
        """
        Background task: reconnects if connection drops,
        sends keepalive every KEEPALIVE_INTERVAL seconds.
        From: chrcraven/LionchiefInteractiveDisplay connection_loop pattern.
        """
        retries = 0
        while self._should_run:
            await asyncio.sleep(2)
            if not self._connected:
                if retries >= self.MAX_RETRIES:
                    logger.error("Max reconnect attempts reached. Giving up.")
                    break
                logger.info(f"Reconnecting... (attempt {retries + 1})")
                success = await self.connect()
                if success:
                    retries = 0
                else:
                    retries += 1
                    await asyncio.sleep(self.RECONNECT_DELAY)
            else:
                # Keepalive: resend current speed to prevent BLE timeout
                now = time.time()
                if now - self._last_keepalive > self.KEEPALIVE_INTERVAL:
                    try:
                        await self._write(build_speed_cmd(self._current_speed))
                        self._last_keepalive = now
                        logger.debug("Keepalive sent")
                    except Exception:
                        pass

    # ── Low-level write ───────────────────────────────────────────────────────

    async def _write(self, data: bytearray) -> bool:
        """
        Write bytes to the LionChief GATT characteristic.
        Uses write_without_response for speed (train doesn't ACK anyway).
        """
        if not self._connected or not self._client:
            logger.warning(f"Cannot send {list(data)} — not connected")
            return False
        try:
            async with self._lock:
                await self._client.write_gatt_char(
                    CHARACTERISTIC_UUID,
                    data,
                    response=False,
                )
            logger.debug(f"Sent: {[hex(b) for b in data]}")
            return True
        except BleakError as e:
            logger.error(f"BLE write failed: {e}")
            self._connected = False
            return False

    # ── Train commands ────────────────────────────────────────────────────────

    async def stop(self) -> bool:
        """Emergency / collision stop."""
        self._current_speed = 0
        logger.info("🛑 STOP command sent")
        return await self._write(STOP_CMD)

    async def set_speed(self, speed: int) -> bool:
        """Set train speed 0–31 (0=stop, 31=max)."""
        speed = max(0, min(31, int(speed)))
        self._current_speed = speed
        return await self._write(build_speed_cmd(speed))

    async def ramp(self, start_speed: int, end_speed: int, step_delay: float = 0.2) -> None:
        """
        Gradually change speed from start to end.
        From: idaband — threaded ramp, adapted to async.
        """
        speed = start_speed
        while speed != end_speed:
            await self.set_speed(speed)
            speed += 1 if speed < end_speed else -1
            await asyncio.sleep(step_delay)
        await self.set_speed(end_speed)

    async def resume(self, speed: int = 7) -> bool:
        """Resume train at given speed after a stop."""
        logger.info(f"✅ RESUME at speed {speed}")
        return await self.set_speed(speed)

    async def forward(self) -> bool:
        self._direction = "forward"
        return await self._write(FORWARD_CMD)

    async def set_horn(self, on: bool) -> bool:
        return await self._write(HORN_ON_CMD if on else HORN_OFF_CMD)

    async def set_bell(self, on: bool) -> bool:
        return await self._write(BELL_ON_CMD if on else BELL_OFF_CMD)

    async def set_lights(self, on: bool) -> bool:
        return await self._write(LIGHT_ON_CMD if on else LIGHT_OFF_CMD)

    async def speak(self, phrase: int = 0) -> bool:
        """Trigger conductor speech. phrase=0 picks random."""
        cmd = bytearray([0x00, 0x4D, phrase & 0xFF, 0x00])
        return await self._write(cmd)

    async def set_volume(self, level: int) -> bool:
        """Master volume 0–7."""
        return await self._write(build_volume_cmd(level))

    # ── Properties ───────────────────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def current_speed(self) -> int:
        return self._current_speed

    # ── Context manager ───────────────────────────────────────────────────────

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, *args):
        await self.disconnect()
