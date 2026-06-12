#!/usr/bin/env python3
"""
Pure-Python USB test for BJJCZ SEA-LASER controller.

Replicates test_lcs.py without liblcs2dll.so.

Run as:
    sudo .venv/bin/python test_sea_usb.py
"""

import sys
import time

sys.path.insert(0, ".")

from bjjcz.sea_usb import SEALaserUSB

print("Opening USB device (USB 04b4:1004)...")
dev = SEALaserUSB()

try:
    dev.connect()
    print("  Connected.")
except IOError as e:
    print(f"  FAILED: {e}")
    sys.exit(1)

print("Running init sequence...")
try:
    dev.init()
    print("  Init OK.")
except Exception as e:
    print(f"  FAILED: {e}")
    dev.disconnect()
    sys.exit(1)

CARD = 0

print("\n--- goto_xy: center (0, 0) ---")
print("  (watch red dot — should be at field center)")
dev.goto_xy(0.0, 0.0)
print("  OK")
time.sleep(1.0)

print("\n--- goto_xy: +20mm X ---")
dev.goto_xy(20.0, 0.0)
print("  OK")
time.sleep(1.0)

print("\n--- goto_xy: +20mm Y ---")
dev.goto_xy(0.0, 20.0)
print("  OK")
time.sleep(1.0)

print("\n--- goto_xy: back to center ---")
dev.goto_xy(0.0, 0.0)
print("  OK")
time.sleep(0.5)

print("\n--- get_status ---")
s = dev.get_status()
print(f"  status byte = {s:#04x}")

print("\nDisconnecting...")
dev.disconnect()
print("Done.")
