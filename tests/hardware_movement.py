"""
Hardware movement test — requires SEA-LASER device connected via USB.

Verify galvo mirrors physically respond by jogging through a sequence of
positions.  Watch the red-dot pointer on the work surface while running:

    sudo python3 tests/hardware_movement.py

Expected visual: the red dot should trace:
    center → 10mm right → 10mm up → 10mm left → center

Pass --laser to also fire a brief mark between each jog (laser safety required).
"""

import argparse
import struct
import sys
import time

sys.path.insert(0, ".")

from bjjcz.coord import GALVO_CENTER, mm_to_galvo


VENDOR_ID = 0x04B4
PRODUCT_ID = 0x1004
GALVOS_PER_MM = 500
FLIP_Y = True


class _SEALaserMixin:
    """Override READY-bit waits: USB-Lite firmware never sets the READY bit."""

    def wait_ready(self):
        return

    def wait_finished(self):
        t_settle = time.monotonic() + 0.1
        while time.monotonic() < t_settle and not self.is_busy():
            time.sleep(0.005)
        t_end = time.monotonic() + 600.0
        while self.is_busy() and time.monotonic() < t_end:
            time.sleep(0.01)
            if not self._sending:
                return


def build_controller():
    from galvo.controller import GalvoController
    from bjjcz.usb import ConfigurableUSBConnection

    SEAController = type("SEAController", (_SEALaserMixin, GalvoController), {})
    ctrl = SEAController(galvos_per_mm=GALVOS_PER_MM)
    ctrl.source = "fiber"
    ctrl.connection = ConfigurableUSBConnection(
        vendor_id=VENDOR_ID,
        product_id=PRODUCT_ID,
        read_endpoint=0x84,
        channel=ctrl.usb_log,
    )
    return ctrl


def jog_to(ctrl, x_mm, y_mm, label=""):
    gx, gy = mm_to_galvo(x_mm, y_mm, GALVOS_PER_MM, FLIP_Y)
    t0 = time.monotonic()
    ctrl.jog(gx, gy)
    ms = (time.monotonic() - t0) * 1000
    print(f"  jog {label:15s}  ({x_mm:+.1f}, {y_mm:+.1f}) mm  →  gx={gx:#06x} gy={gy:#06x}  [{ms:.1f}ms]")


def mark_square(ctrl, size_mm=5.0):
    """Mark a small square at current position using the marking context."""
    half = size_mm / 2
    points = [
        (-half, -half),
        ( half, -half),
        ( half,  half),
        (-half,  half),
        (-half, -half),
    ]
    print(f"  Marking {size_mm}mm square...")
    with ctrl.marking() as ctx:
        ctx.set_power(20)
        ctx.set_mark_speed(500)
        ctx.set_frequency(20)
        ctx.set_pulse_width(200)
        for x, y in points:
            gx, gy = mm_to_galvo(x, y, GALVOS_PER_MM, FLIP_Y)
            ctx.mark(gx, gy)
    ctrl.wait_finished()
    print("  Done.")


def main():
    parser = argparse.ArgumentParser(description="Hardware galvo movement test")
    parser.add_argument("--laser", action="store_true", help="Enable laser marking")
    args = parser.parse_args()

    print("Building controller...")
    ctrl = build_controller()

    print("Connecting...")
    t0 = time.monotonic()
    ctrl.connect_if_needed()
    print(f"Connected in {(time.monotonic()-t0)*1000:.0f}ms")

    # Check status
    status = ctrl.get_status()
    b = struct.pack("<4H", *status) if isinstance(status, tuple) else b""
    print(f"Status: {status}")

    print("\n--- Jogging sequence (watch red dot) ---")
    jog_to(ctrl, 0.0,   0.0,   "center")
    time.sleep(0.5)

    jog_to(ctrl, 10.0,  0.0,   "+10mm X")
    time.sleep(0.5)

    jog_to(ctrl, 10.0,  10.0,  "+10mm Y")
    time.sleep(0.5)

    jog_to(ctrl, -10.0, 10.0,  "-10mm X")
    time.sleep(0.5)

    jog_to(ctrl,  0.0,   0.0,  "center")
    time.sleep(0.5)

    print("\nJog sequence done.")

    if args.laser:
        print("\n--- Laser marking test (5mm square) ---")
        print("WARNING: laser will fire. Ensure workpiece and safety glasses in place.")
        input("Press Enter to continue or Ctrl+C to abort...")
        mark_square(ctrl, size_mm=5.0)

    print("\nDisconnecting...")
    ctrl.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()
