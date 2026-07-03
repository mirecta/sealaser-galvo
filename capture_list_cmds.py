#!/usr/bin/env python3
"""
Focused capture of list-building commands only.
No get_status / wait_finished — those block after goto mode.

Terminal 1:  sudo tcpdump -i usbmon1 -w /tmp/list_capture.pcap
Terminal 2:  sudo .venv/bin/python capture_list_cmds.py
Stop tcpdump when you see:  === DONE ===
"""

import ctypes
import sys
import time

sys.path.insert(0, ".")

from bjjcz import lcs_api

GAP = 3.0

def mark(label):
    print(f"\n[{time.strftime('%H:%M:%S')}] === {label} ===", flush=True)
    time.sleep(0.1)

def gap():
    print(f"  sleeping {GAP}s...", flush=True)
    time.sleep(GAP)

# ---------------------------------------------------------------------------
print("Loading liblcs2dll.so...", flush=True)
lcs_api.load()
lib = lcs_api._l()
CARD = lcs_api.CARD
print("  OK")

print(f"\n  sleeping {GAP}s — start tcpdump NOW...", flush=True)
time.sleep(GAP)

# 1. init
mark("init_dll")
lcs_api.init()
print("  OK")
gap()

# 2–3. speeds (immediate, before list)
mark("set_mark_speed(500)")
lib.set_mark_speed(CARD, ctypes.c_double(500.0))
print("  OK")
gap()

mark("set_jump_speed(2000)")
lib.set_jump_speed(CARD, ctypes.c_double(2000.0))
print("  OK")
gap()

# 4. start list
mark("n_set_start_list_1")
lib.n_set_start_list_1(CARD)
print("  OK")
gap()

# 5. jump to start point
mark("n_jump_abs  (0, 0)")
lib.n_jump_abs(CARD, ctypes.c_double(0.0), ctypes.c_double(0.0))
print("  OK")
gap()

# 6. mark to end point
mark("n_mark_abs  (+10, 0)")
lib.n_mark_abs(CARD, ctypes.c_double(10.0), ctypes.c_double(0.0))
print("  OK")
gap()

# 7. end list (does NOT execute yet)
mark("n_set_end_of_list")
lib.n_set_end_of_list(CARD)
print("  OK")
gap()

# 8. execute — this actually fires the laser
mark("execute_list_1")
lib.execute_list_1(CARD)
print("  OK — list sent to hardware")
gap()

# 9. free
mark("free_dll")
lcs_api.free()
print("  OK")

print("\n=== DONE — stop tcpdump now ===")
