"""
mock_train.py — Full Simulation Without Any Hardware
======================================================
Tests the COMPLETE collision prevention pipeline:
  - Fake BLE train (prints commands instead of sending)
  - Fake IR sensor triggers via keyboard
  - Fake camera zone detection on timer
  - All the same logic as main.py

Controls:
    T  — simulate inner train ENTERING shared zone
    C  — simulate inner train CLEARING the zone
    S  — emergency stop
    R  — resume train
    Q  — quit

Usage:
    python mock_train.py
"""

import asyncio
import sys
import os
import logging

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("mock")


# ── Fake BLE Client ───────────────────────────────────────────────────────────
class FakeLionChief:
    """
    Simulates the LionChief train over BLE.
    Prints every command instead of transmitting it.
    Based on the mock pattern from: fake_train.py (from prior session)
    """

    COMMANDS = {
        bytes([0x00, 0x45, 0x00]): "🛑  STOP",
        bytes([0x00, 0x45, 0x02]): "🐢  Speed SLOW  (2)",
        bytes([0x00, 0x45, 0x07]): "🚆  Speed MEDIUM (7)",
        bytes([0x00, 0x45, 0x0F]): "🚀  Speed FAST  (15)",
        bytes([0x00, 0x45, 0x1F]): "💨  Speed MAX   (31)",
        bytes([0x00, 0x46, 0x01]): "➡️   Direction FORWARD",
        bytes([0x00, 0x46, 0x02]): "⬅️   Direction REVERSE",
        bytes([0x00, 0x47, 0x01]): "🔔  Bell ON",
        bytes([0x00, 0x47, 0x00]): "🔕  Bell OFF",
        bytes([0x00, 0x48, 0x01]): "📯  Horn ON",
        bytes([0x00, 0x48, 0x00]): "📯  Horn OFF",
        bytes([0x00, 0x51, 0x01]): "💡  Lights ON",
        bytes([0x00, 0x51, 0x00]): "💡  Lights OFF",
    }

    def __init__(self, mac="AA:BB:CC:DD:EE:FF"):
        self.mac           = mac
        self._connected    = False
        self._speed        = 0

    async def connect(self) -> bool:
        await asyncio.sleep(0.3)   # simulate BLE connect delay
        self._connected = True
        print(f"\n[FAKE TRAIN] ✅ Connected to {self.mac}")
        return True

    async def disconnect(self):
        print(f"[FAKE TRAIN] ❌ Disconnected from {self.mac}")
        self._connected = False

    async def write_gatt_char(self, uuid: str, data: bytearray, response=False):
        cmd   = bytes(data[:3]) if len(data) >= 3 else bytes(data)
        label = self.COMMANDS.get(cmd, f"❓ Unknown: {list(data)}")
        speed_byte = data[2] if len(data) >= 3 and data[1] == 0x45 else None
        if speed_byte is not None:
            self._speed = speed_byte
        print(f"[FAKE TRAIN] 📨  {label}   raw={list(data)}")

    @property
    def is_connected(self) -> bool:
        return self._connected


# ── Fake Controller (wraps FakeLionChief like LionChiefController) ────────────
class FakeController:
    """Drop-in replacement for LionChiefController for testing."""

    def __init__(self):
        self._train   = FakeLionChief()
        self._speed   = 0
        self._running = True

    async def connect(self) -> bool:
        return await self._train.connect()

    async def disconnect(self):
        await self._train.disconnect()

    async def stop(self) -> bool:
        self._speed = 0
        await self._train.write_gatt_char("", bytearray([0x00, 0x45, 0x00]))
        return True

    async def resume(self, speed: int = 7) -> bool:
        return await self.set_speed(speed)

    async def set_speed(self, speed: int) -> bool:
        speed = max(0, min(31, int(speed)))
        self._speed = speed
        await self._train.write_gatt_char("", bytearray([0x00, 0x45, speed]))
        return True

    async def set_horn(self, on: bool) -> bool:
        cmd = bytearray([0x00, 0x48, 0x01 if on else 0x00])
        await self._train.write_gatt_char("", cmd)
        return True

    async def set_bell(self, on: bool) -> bool:
        cmd = bytearray([0x00, 0x47, 0x01 if on else 0x00])
        await self._train.write_gatt_char("", cmd)
        return True

    @property
    def is_connected(self) -> bool:
        return self._train.is_connected


# ── Collision Prevention Logic ────────────────────────────────────────────────
async def collision_loop(train: FakeController, event_queue: asyncio.Queue):
    """
    Core collision prevention loop.
    Listens for zone events from the input handler.
    Mirrors exactly what main.py does.
    """
    import config
    train_in_zone = False

    logger.info("🚂 Collision prevention loop started")

    while True:
        try:
            event = await asyncio.wait_for(event_queue.get(), timeout=0.5)
        except asyncio.TimeoutError:
            continue

        if event == "ENTER_ZONE":
            if not train_in_zone:
                logger.warning("⚠️  INNER TRAIN IN ZONE — stopping outer train!")
                await train.stop()
                train_in_zone = True
            else:
                logger.info("Already stopped (zone already active)")

        elif event == "CLEAR_ZONE":
            if train_in_zone:
                logger.info(f"✅ Zone clear — resuming in {config.RESUME_DELAY}s ...")
                await asyncio.sleep(config.RESUME_DELAY)
                await train.resume(speed=config.RESUME_SPEED)
                train_in_zone = False
            else:
                logger.info("Zone was not active — nothing to resume")

        elif event == "STOP":
            logger.warning("🚨 EMERGENCY STOP!")
            await train.stop()
            train_in_zone = True

        elif event == "RESUME":
            await train.resume()
            train_in_zone = False

        elif event == "QUIT":
            logger.info("Quit signal received")
            await train.stop()
            await train.disconnect()
            break


# ── Keyboard input handler ────────────────────────────────────────────────────
async def input_handler(event_queue: asyncio.Queue):
    """Read keyboard in non-blocking way using stdin."""
    import sys, tty, termios

    print("\n" + "="*55)
    print("  MOCK SIMULATION CONTROLS")
    print("="*55)
    print("  T — inner train ENTERS shared zone")
    print("  C — inner train CLEARS the zone")
    print("  S — emergency STOP")
    print("  R — manual RESUME")
    print("  Q — quit")
    print("="*55 + "\n")

    fd      = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1).lower()
            if ch == "t":
                print("\n[INPUT] → ENTER ZONE triggered")
                await event_queue.put("ENTER_ZONE")
            elif ch == "c":
                print("\n[INPUT] → CLEAR ZONE triggered")
                await event_queue.put("CLEAR_ZONE")
            elif ch == "s":
                print("\n[INPUT] → EMERGENCY STOP")
                await event_queue.put("STOP")
            elif ch == "r":
                print("\n[INPUT] → MANUAL RESUME")
                await event_queue.put("RESUME")
            elif ch in ("q", "\x03"):   # q or Ctrl+C
                await event_queue.put("QUIT")
                break
            await asyncio.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# ── Auto-scenario (runs if terminal doesn't support raw input) ────────────────
async def auto_scenario(event_queue: asyncio.Queue):
    """
    Automatic test scenario — runs the full collision sequence
    without any keyboard needed. Useful on Windows or headless.
    """
    import config
    print("\n[AUTO] Running automatic collision scenario...\n")
    await asyncio.sleep(2)

    print("[AUTO] Inner train approaching shared zone...")
    await asyncio.sleep(1)

    print("[AUTO] → ENTER ZONE")
    await event_queue.put("ENTER_ZONE")
    await asyncio.sleep(4)

    print("[AUTO] Inner train has passed through...")
    print("[AUTO] → CLEAR ZONE")
    await event_queue.put("CLEAR_ZONE")
    await asyncio.sleep(5)

    print("[AUTO] Second pass test...")
    await event_queue.put("ENTER_ZONE")
    await asyncio.sleep(3)
    await event_queue.put("CLEAR_ZONE")
    await asyncio.sleep(4)

    print("[AUTO] Emergency stop test...")
    await event_queue.put("STOP")
    await asyncio.sleep(2)
    await event_queue.put("RESUME")
    await asyncio.sleep(3)

    print("[AUTO] Scenario complete ✅")
    await event_queue.put("QUIT")


async def main():
    train       = FakeController()
    event_queue = asyncio.Queue()

    await train.connect()

    # Decide whether to use keyboard or auto mode
    try:
        import tty, termios
        keyboard_available = sys.stdin.isatty()
    except ImportError:
        keyboard_available = False

    if keyboard_available:
        await asyncio.gather(
            collision_loop(train, event_queue),
            input_handler(event_queue),
        )
    else:
        print("[INFO] No interactive terminal — running auto scenario")
        await asyncio.gather(
            collision_loop(train, event_queue),
            auto_scenario(event_queue),
        )


if __name__ == "__main__":
    asyncio.run(main())
