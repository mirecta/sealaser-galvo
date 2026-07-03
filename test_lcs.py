#!/usr/bin/env python3
"""
Direct test of liblcs2dll.so — the official BJJCZ control library shipped with LightBurn.

This bypasses galvoplotter entirely and calls the vendor library via ctypes.
If goto_xy works here, we know the coordinates and can reverse-engineer
what galvoplotter is missing.

Run as:
    sudo LD_LIBRARY_PATH=/home/miro/.local/share/LightBurn/lib \
        .venv/bin/python test_lcs.py
"""

import ctypes
import ctypes.util
import sys
import time
import os

LB_LIB = "/home/miro/.local/share/LightBurn/lib"

def load_lib(name):
    path = os.path.join(LB_LIB, name)
    return ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)

print("Loading dependent libraries...")
# Load in dependency order — each must be RTLD_GLOBAL so liblcs2dll sees them
for dep in ["libutils.so", "libparam.so", "libtransfer.so",
            "libexecutor.so", "libBslCal.so"]:
    try:
        load_lib(dep)
        print(f"  {dep} OK")
    except OSError as e:
        print(f"  {dep} FAILED: {e}")

print("Loading liblcs2dll.so...")
try:
    lcs = ctypes.CDLL(os.path.join(LB_LIB, "liblcs2dll.so"), mode=ctypes.RTLD_GLOBAL)
    print("  OK")
except OSError as e:
    print(f"  FAILED: {e}")
    sys.exit(1)

# --- Function signatures ---
# uint32_t init_dll(void)
lcs.init_dll.restype  = ctypes.c_uint32
lcs.init_dll.argtypes = []

# uint32_t free_dll(void)
lcs.free_dll.restype  = ctypes.c_uint32
lcs.free_dll.argtypes = []

# uint32_t goto_xy(uint32_t card, double x, double y)
lcs.goto_xy.restype  = ctypes.c_uint32
lcs.goto_xy.argtypes = [ctypes.c_uint32, ctypes.c_double, ctypes.c_double]

# uint32_t n_goto_xy(uint32_t card, double x, double y)  [non-blocking variant]
lcs.n_goto_xy.restype  = ctypes.c_uint32
lcs.n_goto_xy.argtypes = [ctypes.c_uint32, ctypes.c_double, ctypes.c_double]

# uint32_t enable_laser(uint32_t card)
lcs.enable_laser.restype  = ctypes.c_uint32
lcs.enable_laser.argtypes = [ctypes.c_uint32]

# uint32_t disable_laser(uint32_t card)
lcs.disable_laser.restype  = ctypes.c_uint32
lcs.disable_laser.argtypes = [ctypes.c_uint32]

# ---------------------------------------------------------------------------
print("\n--- init_dll() ---")
ret = lcs.init_dll()
print(f"  ret = {ret:#010x}  ({ret})")
if ret != 0:
    print("  init_dll FAILED — device not found or init error")
    sys.exit(1)

CARD = 0  # first card

print("\n--- goto_xy: center (0, 0) ---")
print("  (watch red dot — should be at field center)")
ret = lcs.goto_xy(CARD, 0.0, 0.0)
print(f"  ret = {ret:#010x}")
time.sleep(1.0)

print("\n--- goto_xy: +20mm X ---")
ret = lcs.goto_xy(CARD, 20.0, 0.0)
print(f"  ret = {ret:#010x}")
time.sleep(1.0)

print("\n--- goto_xy: +20mm Y ---")
ret = lcs.goto_xy(CARD, 0.0, 20.0)
print(f"  ret = {ret:#010x}")
time.sleep(1.0)

print("\n--- goto_xy: back to center ---")
ret = lcs.goto_xy(CARD, 0.0, 0.0)
print(f"  ret = {ret:#010x}")
time.sleep(0.5)

print("\n--- free_dll() ---")
lcs.free_dll()
print("  done")
