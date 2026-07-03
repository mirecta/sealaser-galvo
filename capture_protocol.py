#!/usr/bin/env python3
"""
Protocol capture helper — calls each lcs_api function one at a time
with 3-second gaps and printed timestamps so tshark frames can be correlated.

Run in one terminal:
    sudo .venv/bin/python capture_protocol.py

Run in another terminal (start BEFORE this script):
    sudo tshark -i usbmon1 -w /tmp/proto_capture.pcap

After capture, decode with:
    python3 decode_capture.py /tmp/proto_capture.pcap
"""

import sys
import time

sys.path.insert(0, ".")

from bjjcz import lcs_api

GAP = 3.0  # seconds between function calls

def mark(label):
    ts = time.strftime("%H:%M:%S")
    print(f"\n[{ts}] === {label} ===", flush=True)
    time.sleep(0.1)   # tiny settle before the call

def gap(note=""):
    print(f"  sleeping {GAP}s{' — ' + note if note else ''}...", flush=True)
    time.sleep(GAP)

# ---------------------------------------------------------------------------
print("Loading liblcs2dll.so...")
lcs_api.load()
print("  OK\n")

gap("start tshark NOW if not already running")

# 1. init
mark("init_dll")
lcs_api.init()
print("  ret OK")
gap()

# 2. goto center
mark("goto_xy  (0, 0)  — center")
lcs_api.goto(0.0, 0.0)
print("  ret OK")
gap()

# 3. goto +20mm X
mark("goto_xy  (+20, 0)")
lcs_api.goto(20.0, 0.0)
print("  ret OK")
gap()

# 4. goto center again
mark("goto_xy  (0, 0)  — back to center")
lcs_api.goto(0.0, 0.0)
print("  ret OK")
gap()

# 5. get_status
mark("get_status")
s = lcs_api.get_status()
print(f"  status = {s:#010x}")
gap()

# 6. set speeds
mark("set_mark_speed(500)  +  set_jump_speed(2000)")
lcs_api.set_mark_speed(500.0)
lcs_api.set_jump_speed(2000.0)
print("  ret OK")
gap()

# 7. list: jump + mark (simple 1-segment line)
mark("n_set_start_list_1")
lcs_api.list_start()
print("  ret OK")
gap()

mark("n_jump_abs  (0, 0)")
lcs_api.list_jump(0.0, 0.0)
print("  ret OK")
gap()

mark("n_mark_abs  (+10, 0)")
lcs_api.list_mark(10.0, 0.0)
print("  ret OK")
gap()

mark("n_set_end_of_list")
from bjjcz import lcs_api as _lcs
_lcs._l().n_set_end_of_list(_lcs.CARD)   # without execute
print("  ret OK")
gap()

mark("execute_list_1  (fires laser list!)")
_lcs._l().execute_list_1(_lcs.CARD)
print("  ret OK — waiting for finish...")
lcs_api.wait_finished(timeout_s=10)
print("  finished")
gap()

# 8. free
mark("free_dll")
lcs_api.free()
print("  ret OK")

print("\n=== DONE — stop tshark now ===")
