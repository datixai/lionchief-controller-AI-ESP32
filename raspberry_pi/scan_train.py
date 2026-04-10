"""
scan_train.py — BLE Scanner to Find Train MAC Address
======================================================
Run this while Peter's train is powered ON.

Scans all nearby BLE devices and highlights any LionChief train found.
Looks for devices with:
  - Name starting with "LC0"  (e.g. "LC015556-99F0")
  - Or matching the LionChief service UUID

Usage:
    python scan_train.py
    python scan_train.py --timeout 15     # scan longer

Output example:
    🚂 TRAIN FOUND → LC015556-99F0    CC:01:78:D0:F0:99   RSSI:-55
    Match reason: name prefix "LC0"

    PASTE THIS INTO config.py:
       TRAIN_MAC_ADDRESS = "CC:01:78:D0:F0:99"
"""

import asyncio
import argparse
import logging
from bleak import BleakScanner

logging.basicConfig(level=logging.WARNING)  # keep bleak quiet

LIONCHIEF_SERVICE_UUID = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
LIONCHIEF_NAME_PREFIX  = "LC0"


async def scan(timeout: float = 10.0):
    print("\n" + "=" * 58)
    print("  LIONCHIEF TRAIN BLE SCANNER")
    print("=" * 58)
    print(f"  Scanning for {timeout}s — make sure the train is ON")
    print(f"  Looking for: name prefix '{LIONCHIEF_NAME_PREFIX}'")
    print("=" * 58 + "\n")

    devices = await BleakScanner.discover(timeout=timeout)

    if not devices:
        print("No BLE devices found. Is Bluetooth enabled on this device?")
        return

    print(f"Found {len(devices)} BLE device(s):\n")
    trains_found = []

    for d in sorted(devices, key=lambda x: x.name or ""):
        name  = d.name or "(unnamed)"
        addr  = d.address
        rssi  = getattr(d, "rssi", "?")
        meta  = str(d.metadata) if d.metadata else ""

        is_train  = False
        reasons   = []

        # Check by name prefix "LC0"
        if name.upper().startswith(LIONCHIEF_NAME_PREFIX.upper()):
            is_train = True
            reasons.append(f'name prefix "{LIONCHIEF_NAME_PREFIX}"')

        # Check by service UUID in advertisement data
        if LIONCHIEF_SERVICE_UUID.lower() in meta.lower():
            is_train = True
            reasons.append("service UUID match")

        if is_train:
            print(f"  🚂 TRAIN FOUND  →  {name:<30} {addr}   RSSI:{rssi}")
            print(f"                     Match: {', '.join(reasons)}")
            trains_found.append(d)
        else:
            print(f"     {name:<30} {addr}   RSSI:{rssi}")

    print()

    if trains_found:
        print("=" * 58)
        print("✅ PASTE THIS INTO config.py (raspberry_pi or laptop_cv):")
        for t in trains_found:
            name = t.name or "(unnamed)"
            print(f"\n   # {name}")
            print(f'   TRAIN_MAC_ADDRESS = "{t.address}"')
        print("\n   OR for laptop_cv/config.py:")
        for t in trains_found:
            print(f'   TRAIN_MAC = "{t.address}"')
        print("=" * 58)
    else:
        print("=" * 58)
        print("⚠️  No LionChief train detected.")
        print(f"   Expected device name starting with: {LIONCHIEF_NAME_PREFIX!r}")
        print("   Make sure the train is powered on and within ~10m range.")
        print("   Try:  python scan_train.py --timeout 20")
        print("=" * 58)


def main():
    parser = argparse.ArgumentParser(
        description="Scan for LionChief train MAC address")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Scan duration in seconds (default: 10)")
    args = parser.parse_args()
    asyncio.run(scan(args.timeout))


if __name__ == "__main__":
    main()
