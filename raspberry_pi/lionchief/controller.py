"""
lionchief/controller.py — LionChief BLE Controller
====================================================

TWO WAYS TO CONNECT:

  Mode A — Direct MAC (recommended):
    Pass mac_address="CC:01:78:D0:F0:99" (or set in config.py).
    Connects directly without scanning. Fastest and most reliable.

  Mode B — Auto-discover by name prefix:
    Pass mac_address=None (or leave empty in config.py).
    Scans BLE devices and connects to the first device whose name
    starts with "LC0" (matches any LionChief train).

Usage:
    import asyncio
    from lionchief.controller import LionChiefController

    async def main():
        # Mode A — direct MAC
        ctrl = LionChiefController("CC:01:78:D0:F0:99")

        # Mode B — auto-discover
        # ctrl = LionChiefController()

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
    STOP_CMD, MEDIUM_CMD, FORWARD_CMD, REVERSE_CMD,
    HORN_ON_CMD, HORN_OFF_CMD,
    BELL_ON_CMD, BELL_OFF_CMD,
    LIGHT_ON_CMD, LIGHT_OFF_CMD,
    SPEAK_CMD, DISCO_CMD,
    build_speed_cmd, build_volume_cmd,
)

logger = logging.getLogger(__name__)

# Name prefix used for auto-discover (matches "LC015556-99F0" and any LionChief)
LIONCHIEF_NAME_PREFIX = "LC0"


class LionChiefController:
    """
    Async BLE controller for Lionel LionChief trains.

    Handles:
      - Mode A: direct connection by MAC address
      - Mode B: auto-discovery by device name prefix "LC0"
      - Auto-reconnect on connection drop
      - Keepalive pings to maintain BLE connection
      - All known LionChief BLE commands
    """

    RECONNECT_DELAY    = 5.0    # seconds between reconnect attempts
    KEEPALIVE_INTERVAL = 25.0   # seconds between keepalive speed pings
    MAX_RETRIES        = 10     # give up after this many failed reconnects

    def __init__(self, mac_address: Optional[str] = None,
                 auto_connect: bool = True):
        """
        Args:
            mac_address:  Train MAC address for Mode A.
                          Pass None or "" to use Mode B (auto-discover).
            auto_connect: If True, start background reconnect/keepalive loop.
        """
        self.mac_address     = (mac_address or "").strip()
        self.auto_connect    = auto_connect
        self._client: Optional[BleakClient] = None
        self._connected      = False
        self._current_speed  = 0
        self._lock           = asyncio.Lock()
        self._reconnect_task: Optional[asyncio.Task] = None
        self._should_run     = True
        self._last_keepalive = 0.0

    # ── Connection ────────────────────────────────────────────────────────────

    async def connect(self) -> bool:
        """
        Connect to the train. Returns True on success.

        Mode A: MAC address known → connect directly.
        Mode B: MAC address empty → scan and find by name prefix "LC0".
        """
        try:
            if not self.mac_address:
                # Mode B: auto-discover by name prefix
                logger.info(
                    f"No MAC address set — scanning for LionChief "
                    f"(name prefix '{LIONCHIEF_NAME_PREFIX}')...")
                self.mac_address = await self._discover_by_name()
                if not self.mac_address:
                    logger.error(
                        "No LionChief train found during scan. "
                        "Is the train powered on and within BLE range?")
                    return False
            else:
                # Mode A: direct MAC connection
                logger.info(f"Connecting directly to MAC: {self.mac_address}")

            self._client = BleakClient(
                self.mac_address,
                disconnected_callback=self._on_disconnect,
            )
            await self._client.connect(timeout=15.0)
            self._connected      = True
            self._last_keepalive = time.time()
            logger.info(f"✅ Connected to LionChief at {self.mac_address}")

            if self.auto_connect and self._reconnect_task is None:
                self._reconnect_task = asyncio.create_task(
                    self._keepalive_loop())

            return True

        except (BleakError, asyncio.TimeoutError) as e:
            logger.error(f"Connection failed: {e}")
            self._connected = False
            return False

    async def disconnect(self) -> None:
        """Gracefully disconnect."""
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
        logger.warning("BLE connection lost — will auto-reconnect")
        self._connected = False

    async def _discover_by_name(self, timeout: float = 10.0) -> Optional[str]:
        """
        Scan BLE devices and return MAC of first device whose name
        starts with LIONCHIEF_NAME_PREFIX ("LC0").
        Also falls back to matching the LionChief service UUID.
        """
        logger.info(f"Scanning {timeout}s for BLE devices...")
        devices = await BleakScanner.discover(timeout=timeout)

        for d in devices:
            name = (d.name or "").strip()
            if name:
                logger.debug(f"  Found: {name!r} [{d.address}]")

            # Primary: match by name prefix "LC0"
            if name.upper().startswith(LIONCHIEF_NAME_PREFIX.upper()):
                logger.info(f"Found LionChief by name: {name!r} [{d.address}]")
                return d.address

        # Secondary: match by service UUID in advertisement
        for d in devices:
            meta = str(d.metadata) if d.metadata else ""
            if SERVICE_UUID.lower() in meta.lower():
                logger.info(
                    f"Found LionChief by UUID: {d.name!r} [{d.address}]")
                return d.address

        return None

    async def _keepalive_loop(self) -> None:
        """Background task: reconnect if dropped, send keepalive pings."""
        retries = 0
        while self._should_run:
            await asyncio.sleep(2)
            if not self._connected:
                if retries >= self.MAX_RETRIES:
                    logger.error("Max reconnect attempts reached.")
                    break
                logger.info(f"Reconnecting... (attempt {retries + 1})")
                success = await self.connect()
                retries = 0 if success else retries + 1
                if not success:
                    await asyncio.sleep(self.RECONNECT_DELAY)
            else:
                now = time.time()
                if now - self._last_keepalive > self.KEEPALIVE_INTERVAL:
                    try:
                        await self._write(build_speed_cmd(self._current_speed))
                        self._last_keepalive = now
                        logger.debug("Keepalive ping sent")
                    except Exception:
                        pass

    # ── Low-level write ───────────────────────────────────────────────────────

    async def _write(self, data: bytearray) -> bool:
        if not self._connected or not self._client:
            logger.warning(f"Cannot send {list(data)} — not connected")
            return False
        try:
            async with self._lock:
                await self._client.write_gatt_char(
                    CHARACTERISTIC_UUID, data, response=False)
            logger.debug(f"Sent: {[hex(b) for b in data]}")
            return True
        except BleakError as e:
            logger.error(f"BLE write failed: {e}")
            self._connected = False
            return False

    # ── Commands ──────────────────────────────────────────────────────────────

    async def stop(self) -> bool:
        """Send immediate stop."""
        self._current_speed = 0
        logger.info("STOP sent")
        return await self._write(STOP_CMD)

    async def set_speed(self, speed: int) -> bool:
        """Set speed 0–31 (0=stop, 31=max)."""
        speed = max(0, min(31, int(speed)))
        self._current_speed = speed
        return await self._write(build_speed_cmd(speed))

    async def resume(self, speed: int = 7) -> bool:
        """Resume at given speed after a stop."""
        logger.info(f"RESUME at speed {speed}")
        return await self.set_speed(speed)

    async def ramp(self, start: int, end: int, step_delay: float = 0.2):
        """Gradually change speed from start to end."""
        speed = start
        while speed != end:
            await self.set_speed(speed)
            speed += 1 if speed < end else -1
            await asyncio.sleep(step_delay)
        await self.set_speed(end)

    async def forward(self) -> bool:
        return await self._write(FORWARD_CMD)

    async def reverse(self) -> bool:
        return await self._write(REVERSE_CMD)

    async def set_horn(self, on: bool) -> bool:
        return await self._write(HORN_ON_CMD if on else HORN_OFF_CMD)

    async def set_bell(self, on: bool) -> bool:
        return await self._write(BELL_ON_CMD if on else BELL_OFF_CMD)

    async def set_lights(self, on: bool) -> bool:
        return await self._write(LIGHT_ON_CMD if on else LIGHT_OFF_CMD)

    async def speak(self, phrase: int = 0) -> bool:
        """Conductor speech. phrase=0 = random."""
        return await self._write(bytearray([0x00, 0x4D, phrase & 0xFF, 0x00]))

    async def set_volume(self, level: int) -> bool:
        """Master volume 0–7."""
        return await self._write(build_volume_cmd(level))

    # ── Properties / context manager ──────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def current_speed(self) -> int:
        return self._current_speed

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, *args):
        await self.disconnect()
