"""
scan_train.py — BLE Scanner to Find Train MAC Address
======================================================
Run this while Peter's train is powered ON.
It will print all nearby BLE devices and highlight
anything that looks like a LionChief train.

Usage:
    python scan_train.py
    python scan_train.py --timeout 15    # scan longer

This replaces needing the nRF Connect app — works directly from
Raspberry Pi or any Linux/Mac/Windows PC with Bluetooth.
"""

import asyncio
import argparse
import sys
import logging
from bleak import BleakScanner

logging.basicConfig(level=logging.WARNING)   # keep bleak quiet during scan

LIONCHIEF_SERVICE_UUID = "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"


async def scan(timeout: float = 10.0):
    print("\n" + "="*55)
    print("  LIONCHIEF TRAIN BLE SCANNER")
    print("="*55)
    print(f"  Scanning {timeout}s — make sure the train is ON")
    print("="*55 + "\n")

    devices = await BleakScanner.discover(timeout=timeout)

    if not devices:
        print("No BLE devices found. Check that Bluetooth is enabled.")
        return

    print(f"Found {len(devices)} device(s):\n")
    found_trains = []

    for d in devices:
        name = d.name or "(unnamed)"
        addr = d.address
        rssi = d.rssi if hasattr(d, "rssi") else "?"

        # Check for LionChief identifiers
        is_train = False
        reasons  = []

        if any(k in name.lower() for k in ["lion", "chief", "lionchief"]):
            is_train = True
            reasons.append("name match")

        meta = str(d.metadata) if d.metadata else ""
        if LIONCHIEF_SERVICE_UUID.lower() in meta.lower():
            is_train = True
            reasons.append("service UUID match")

        prefix = "  🚂 TRAIN FOUND  →" if is_train else "     "
        print(f"{prefix}  {name:<30}  {addr}   RSSI:{rssi}")

        if is_train:
            found_trains.append(d)
            print(f"             Match reason: {', '.join(reasons)}")

    print()
    if found_trains:
        print("="*55)
        print("✅ PASTE THIS INTO config.py or .env:")
        for t in found_trains:
            print(f'\n   TRAIN_MAC_ADDRESS = "{t.address}"')
        print("="*55)
    else:
        print("⚠️  No LionChief train detected.")
        print("   Make sure the train is powered on and within BLE range (~10m).")
        print("   Try increasing --timeout or moving closer.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Scan duration in seconds (default 10)")
    args = parser.parse_args()
    asyncio.run(scan(args.timeout))


if __name__ == "__main__":
    main()
