#!/usr/bin/env python3
"""
gconsole — interactive debug console for BJJCZ galvo controllers.

Uses liblcs2dll.so (the same library as LightBurn and the Rayforge driver)
directly via ctypes through bjjcz.lcs_api.

Run as:
    sudo LD_LIBRARY_PATH=/home/miro/.local/share/LightBurn/lib \
        .venv/bin/python gconsole.py [-v]

    (or just: sudo .venv/bin/python gconsole.py)

Type 'help' at the prompt for a full command list.
"""

import argparse
import readline
import sys
import time

sys.path.insert(0, ".")

from bjjcz import lcs_api


# ---------------------------------------------------------------------------
# Console
# ---------------------------------------------------------------------------

class Console:
    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.connected = False
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0
        self._power: float = 20.0
        self._mark_speed: float = 500.0
        self._jump_speed: float = 2000.0

    # -----------------------------------------------------------------------
    # Connection
    # -----------------------------------------------------------------------

    def connect(self):
        if self.connected:
            print("Already connected.")
            return
        print("Loading liblcs2dll.so...")
        try:
            lcs_api.load()
        except FileNotFoundError as exc:
            print(f"ERROR: {exc}")
            return
        print("Connecting to BJJCZ controller (USB 04b4:1004)...")
        try:
            lcs_api.init()
            self.connected = True
            status = lcs_api.get_status()
            print(f"Connected.  Status: {status:#010x}  "
                  f"{'BUSY' if status & lcs_api.STATUS_BUSY else 'idle'}")
        except Exception as exc:
            print(f"ERROR: {exc}")

    def disconnect(self):
        if not self.connected:
            return
        try:
            lcs_api.free()
        except Exception:
            pass
        self.connected = False
        print("Disconnected.")

    def _require(self):
        if not self.connected:
            raise ConnectionError("Not connected — type 'connect' first")

    # -----------------------------------------------------------------------
    # Coordinate helpers
    # -----------------------------------------------------------------------

    def _parse_mm(self, tok: str) -> float:
        t = tok.strip().lower()
        if t.endswith("mm"):
            return float(t[:-2])
        return float(t)

    # -----------------------------------------------------------------------
    # Command handlers
    # -----------------------------------------------------------------------

    def cmd_status(self, _args):
        self._require()
        s = lcs_api.get_status()
        print(f"  status: {s:#010x}  {'BUSY' if s & lcs_api.STATUS_BUSY else 'idle'}")

    def cmd_goto(self, args):
        """goto X Y  — jog mirrors (no laser).  Units: mm or raw float."""
        self._require()
        if len(args) < 2:
            print("Usage: goto X Y  (e.g. 'goto 10mm -5mm' or 'goto 20 0')")
            return
        x = self._parse_mm(args[0])
        y = self._parse_mm(args[1])
        t0 = time.monotonic()
        lcs_api.goto(x, y)
        ms = (time.monotonic() - t0) * 1000
        self._pos_x, self._pos_y = x, y
        print(f"  → ({x:+.3f}, {y:+.3f}) mm  [{ms:.1f} ms]")

    def cmd_center(self, _args):
        self._require()
        lcs_api.goto(0.0, 0.0)
        self._pos_x, self._pos_y = 0.0, 0.0
        print("  → (0, 0) mm")

    def cmd_pos(self, _args):
        print(f"  last: ({self._pos_x:+.3f}, {self._pos_y:+.3f}) mm")

    def cmd_laser(self, args):
        """laser on|off  — enable or disable the laser directly."""
        self._require()
        if not args:
            print("Usage: laser on|off")
            return
        if args[0].lower() in ("on", "1"):
            lcs_api.enable_laser()
            print("  laser ON")
        else:
            lcs_api.disable_laser()
            print("  laser OFF")

    def cmd_power(self, args):
        """power PCT  — set laser power 0–100 %."""
        self._require()
        if not args:
            print(f"  current power: {self._power:.1f}%")
            return
        self._power = float(args[0])
        lcs_api.set_power(self._power)
        print(f"  power → {self._power:.1f}%")

    def cmd_speed(self, args):
        """speed MARK [JUMP]  — set mark speed (and optionally jump speed) in mm/s."""
        self._require()
        if not args:
            print(f"  mark: {self._mark_speed:.0f} mm/s  jump: {self._jump_speed:.0f} mm/s")
            return
        self._mark_speed = float(args[0])
        lcs_api.set_mark_speed(self._mark_speed)
        if len(args) > 1:
            self._jump_speed = float(args[1])
            lcs_api.set_jump_speed(self._jump_speed)
        print(f"  mark: {self._mark_speed:.0f} mm/s  jump: {self._jump_speed:.0f} mm/s")

    def cmd_dot(self, args):
        """dot [X Y [DUR_MS]]  — fire laser at one point (default: current pos, 200 ms)."""
        self._require()
        x = self._parse_mm(args[0]) if len(args) > 0 else self._pos_x
        y = self._parse_mm(args[1]) if len(args) > 1 else self._pos_y
        dur_ms = float(args[2]) if len(args) > 2 else 200.0

        print(f"  Dot at ({x:+.3f}, {y:+.3f}) mm  power={self._power:.0f}%  {dur_ms:.0f} ms...")
        lcs_api.set_power(self._power)
        lcs_api.set_mark_speed(self._mark_speed)
        lcs_api.set_jump_speed(self._jump_speed)

        t0 = time.monotonic()
        lcs_api.list_start()
        lcs_api.list_jump(x, y)
        lcs_api.list_laser_on(int(dur_ms * 1000))
        lcs_api.list_end()
        self._pos_x, self._pos_y = x, y
        print(f"  Done in {(time.monotonic()-t0)*1000:.0f} ms")

    def cmd_square(self, args):
        """square [SIZE [POWER [SPEED]]]  — mark a square (default 5mm, current power/speed)."""
        self._require()
        size = float(args[0]) if args else 5.0
        power = float(args[1]) if len(args) > 1 else self._power
        speed = float(args[2]) if len(args) > 2 else self._mark_speed
        half = size / 2
        corners = [
            (-half, -half), ( half, -half),
            ( half,  half), (-half,  half),
            (-half, -half),
        ]
        print(f"  Marking {size}mm square  power={power:.0f}%  speed={speed:.0f} mm/s...")

        lcs_api.set_power(power)
        lcs_api.set_mark_speed(speed)
        lcs_api.set_jump_speed(self._jump_speed)

        t0 = time.monotonic()
        lcs_api.list_start()
        first = True
        for cx, cy in corners:
            if first:
                lcs_api.list_jump(cx, cy)
                first = False
            else:
                lcs_api.list_mark(cx, cy)
        lcs_api.list_end()
        print(f"  Done in {(time.monotonic()-t0)*1000:.0f} ms")

    def cmd_sweep(self, args):
        """sweep [STEPS [RANGE_MM]]  — jog grid of positions (no laser)."""
        self._require()
        steps = int(args[0]) if args else 5
        rng = float(args[1]) if len(args) > 1 else 20.0
        half = rng / 2.0
        step = rng / max(steps - 1, 1)
        print(f"  Sweeping {steps}×{steps} grid ±{rng}mm  (watch red dot)...")
        for i in range(steps):
            for j in range(steps):
                x = -half + i * step
                y = -half + j * step
                lcs_api.goto(x, y)
                time.sleep(0.06)
        lcs_api.goto(0.0, 0.0)
        self._pos_x, self._pos_y = 0.0, 0.0
        print("  Done → returned to center")

    def cmd_poll(self, args):
        """poll [N [MS]]  — read status N times every MS ms."""
        self._require()
        n = int(args[0]) if args else 10
        ms = float(args[1]) if len(args) > 1 else 200.0
        for i in range(n):
            s = lcs_api.get_status()
            print(f"  [{i+1:3d}] {s:#010x}  {'BUSY' if s & lcs_api.STATUS_BUSY else 'idle'}")
            if i < n - 1:
                time.sleep(ms / 1000.0)

    def cmd_help(self, _args):
        print("""
Commands
────────
  connect                   Load library and open USB
  disconnect                Release USB

  status  / s               Read hardware status word
  pos                       Show last known position

  goto X Y                  Jog mirrors to position (no laser)
                            e.g.  goto 10 -5    or    goto 10mm -5.5mm
  center / home             Goto field centre (0, 0)
  sweep  [STEPS [RANGE]]    Jog STEPS×STEPS grid ±RANGE mm (default 5×5, ±20mm)

  power [PCT]               Get/set laser power 0–100 %
  speed [MARK [JUMP]]       Get/set speeds in mm/s

  dot  [X Y [DUR_MS]]       Fire laser at a point (default current pos, 200ms)
  square [SIZE [PWR [SPD]]] Mark a square (default 5mm)

  poll [N [MS]]             Read status N times every MS ms
  help / ?                  This message
  quit / q / exit           Exit
""")

    # -----------------------------------------------------------------------
    # REPL
    # -----------------------------------------------------------------------

    ALIASES = {
        "s": "status", "c": "center", "home": "center",
        "?": "help", "exit": "quit", "q": "quit",
    }

    HANDLERS = {
        "connect":    lambda self, a: self.connect(),
        "disconnect": lambda self, a: self.disconnect(),
        "status":     cmd_status,
        "pos":        cmd_pos,
        "goto":       cmd_goto,
        "center":     cmd_center,
        "laser":      cmd_laser,
        "power":      cmd_power,
        "speed":      cmd_speed,
        "dot":        cmd_dot,
        "square":     cmd_square,
        "sweep":      cmd_sweep,
        "poll":       cmd_poll,
        "help":       cmd_help,
        "quit":       None,
    }

    def run(self):
        self.connect()
        if not self.connected:
            print("Could not connect.  Type 'connect' to retry, 'help' for commands.")
        else:
            print("\nType 'help' for command list.\n")

        while True:
            try:
                line = input("galvo> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            parts = line.split()
            name = self.ALIASES.get(parts[0], parts[0])
            args = parts[1:]
            if name == "quit":
                break
            handler = self.HANDLERS.get(name)
            if handler is None and name not in self.HANDLERS:
                print(f"  Unknown: '{parts[0]}'  (type 'help')")
                continue
            try:
                handler(self, args)
            except ConnectionError as exc:
                print(f"  {exc}")
            except Exception as exc:
                print(f"  ERROR: {type(exc).__name__}: {exc}")

        self.disconnect()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    try:
        readline.read_history_file(".gconsole_history")
    except Exception:
        pass

    parser = argparse.ArgumentParser(description="BJJCZ galvo debug console (lcs_api)")
    parser.add_argument("-v", "--verbose", action="store_true")
    cli = parser.parse_args()

    con = Console(verbose=cli.verbose)
    try:
        con.run()
    finally:
        try:
            readline.write_history_file(".gconsole_history")
        except Exception:
            pass


if __name__ == "__main__":
    main()
