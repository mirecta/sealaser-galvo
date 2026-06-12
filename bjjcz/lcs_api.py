"""
Ctypes wrapper around liblcs2dll.so — the official BJJCZ/JCZ galvo control
library shipped with LightBurn.

Coordinates are in millimetres, origin at field centre:
  x > 0  →  right
  y > 0  →  up  (same as Rayforge with origin=center; apply flip_y before calling)

Naming convention (BJJCZ SDK):
  Functions without n_ prefix: execute immediately (blocking unless noted).
  Functions with n_ prefix:    add command to the current list buffer.
The list is built with n_set_start_list_1 … n_set_end_of_list, then fired
with execute_list_1.

Set LCS_LIB_PATH env-var to override the default LightBurn lib directory.
"""

from __future__ import annotations

import ctypes
import os
import time
from contextlib import contextmanager
from pathlib import Path

# ---------------------------------------------------------------------------
# Library location
# ---------------------------------------------------------------------------

_LB_LIB = Path("/home/miro/.local/share/LightBurn/lib")

_DEPS = [
    "libutils.so",
    "libparam.so",
    "libtransfer.so",
    "libexecutor.so",
    "libBslCal.so",
]

_LCS_SO = "liblcs2dll.so"


def _load() -> ctypes.CDLL:
    lib_dir = os.environ.get("LCS_LIB_PATH", str(_LB_LIB))
    for dep in _DEPS:
        p = os.path.join(lib_dir, dep)
        try:
            ctypes.CDLL(p, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass

    path = os.path.join(lib_dir, _LCS_SO)
    try:
        lib = ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
    except OSError as exc:
        raise FileNotFoundError(
            f"Cannot load {path}: {exc}\n"
            "Install LightBurn or set LCS_LIB_PATH to the directory "
            "containing liblcs2dll.so and its dependencies."
        ) from exc
    _bind(lib)
    return lib


def _fn(lib: ctypes.CDLL, name: str, restype, argtypes) -> None:
    try:
        f = getattr(lib, name)
        f.restype  = restype
        f.argtypes = argtypes
    except AttributeError:
        pass


def _bind(lib: ctypes.CDLL) -> None:
    u32 = ctypes.c_uint32
    dbl = ctypes.c_double
    boo = ctypes.c_bool

    # Init / cleanup
    _fn(lib, "init_dll",               u32, [])
    _fn(lib, "free_dll",               u32, [])

    # Direct (immediate) motion — blocking
    _fn(lib, "goto_xy",                u32, [u32, dbl, dbl])
    _fn(lib, "n_goto_xy",              u32, [u32, dbl, dbl])   # non-blocking

    # Direct laser on/off
    _fn(lib, "enable_laser",           u32, [u32])
    _fn(lib, "disable_laser",          u32, [u32])

    # Laser parameters (immediate)
    _fn(lib, "set_laser_mode",         u32, [u32, u32, boo])   # card, mode, pulsed
    _fn(lib, "set_laser_power",        u32, [u32, u32])         # card, power 0-100
    _fn(lib, "set_laser_pulses",       u32, [u32, u32, u32])   # card, freq_hz, pulse_us
    _fn(lib, "set_laser_delays",       u32, [u32, u32, u32])   # card, on_us, off_us
    _fn(lib, "set_scanner_delays",     u32, [u32, u32, u32, u32])  # card, jump, mark, poly

    # Speed (immediate, effective for the next list execution)
    _fn(lib, "set_mark_speed",         u32, [u32, dbl])
    _fn(lib, "set_jump_speed",         u32, [u32, dbl])

    # ---- List-building (n_ = "add to current list") ----
    _fn(lib, "n_set_start_list_1",     u32, [u32])
    _fn(lib, "n_set_start_list_2",     u32, [u32])
    _fn(lib, "n_set_end_of_list",      u32, [u32])

    # Movement in list
    _fn(lib, "n_jump_abs",             u32, [u32, dbl, dbl])
    _fn(lib, "n_mark_abs",             u32, [u32, dbl, dbl])
    _fn(lib, "jump_abs",               u32, [u32, dbl, dbl])   # may also be list
    _fn(lib, "mark_abs",               u32, [u32, dbl, dbl])

    # Laser timed pulse inside list
    _fn(lib, "n_laser_on_list",        u32, [u32, u32])         # card, time_us
    _fn(lib, "laser_on_list",          u32, [u32, u32])

    # In-list parameter overrides
    _fn(lib, "n_set_mark_speed",       u32, [u32, dbl])
    _fn(lib, "n_set_jump_speed",       u32, [u32, dbl])
    _fn(lib, "n_set_laser_power",      u32, [u32, u32])

    # List execution / control
    _fn(lib, "execute_list_1",         u32, [u32])
    _fn(lib, "execute_list_2",         u32, [u32])
    _fn(lib, "list_continue",          u32, [u32])
    _fn(lib, "restart_list",           u32, [u32])
    _fn(lib, "stop_execution",         u32, [u32])
    _fn(lib, "stop_list",              u32, [u32])

    # Status
    _fn(lib, "get_status",             u32, [u32])

    # Port I/O
    _fn(lib, "write_io_port",          u32, [u32, u32])
    _fn(lib, "read_io_port",           u32, [u32])


# ---------------------------------------------------------------------------
# Module-level state
# ---------------------------------------------------------------------------

_lib: ctypes.CDLL | None = None

CARD = 0  # First (and usually only) card

# BUSY bit in the status word returned by get_status()
STATUS_BUSY = 0x04


def load() -> None:
    """Load liblcs2dll.so and its dependencies.  Call once before any other function."""
    global _lib
    if _lib is None:
        _lib = _load()


def _l() -> ctypes.CDLL:
    if _lib is None:
        raise RuntimeError("lcs_api not loaded — call lcs_api.load() first")
    return _lib


def _check(ret: int, op: str) -> None:
    if ret != 0:
        raise IOError(f"lcs2dll: {op} returned {ret:#010x}")


# ---------------------------------------------------------------------------
# Public API — immediate (direct hardware commands)
# ---------------------------------------------------------------------------

def init() -> None:
    """Open USB and initialise the BJJCZ controller.  Returns when ready."""
    _check(_l().init_dll(), "init_dll")


def free() -> None:
    """Release USB device."""
    _l().free_dll()


def goto(x_mm: float, y_mm: float) -> None:
    """Jog mirrors to (x_mm, y_mm) — blocking, no laser."""
    _check(_l().goto_xy(CARD, float(x_mm), float(y_mm)), "goto_xy")


def n_goto(x_mm: float, y_mm: float) -> None:
    """Jog mirrors — non-blocking."""
    _check(_l().n_goto_xy(CARD, float(x_mm), float(y_mm)), "n_goto_xy")


def enable_laser() -> None:
    _check(_l().enable_laser(CARD), "enable_laser")


def disable_laser() -> None:
    _check(_l().disable_laser(CARD), "disable_laser")


def set_power(percent: float) -> None:
    """Set laser power 0–100 %."""
    _check(_l().set_laser_power(CARD, int(percent)), "set_laser_power")


def set_pulses(freq_hz: float, pulse_us: float) -> None:
    """Set pulsed-fibre parameters: frequency (Hz) and pulse width (µs)."""
    _check(_l().set_laser_pulses(CARD, int(freq_hz), int(pulse_us)),
           "set_laser_pulses")


def set_mark_speed(mm_s: float) -> None:
    _check(_l().set_mark_speed(CARD, float(mm_s)), "set_mark_speed")


def set_jump_speed(mm_s: float) -> None:
    _check(_l().set_jump_speed(CARD, float(mm_s)), "set_jump_speed")


def get_status() -> int:
    """Return the hardware status word.  Bit 0x04 = BUSY."""
    return int(_l().get_status(CARD))


def is_busy() -> bool:
    return bool(get_status() & STATUS_BUSY)


def wait_finished(timeout_s: float = 60.0) -> None:
    """Block until the controller is no longer busy."""
    deadline = time.monotonic() + timeout_s
    # Allow the hardware a short settle before checking
    time.sleep(0.05)
    while time.monotonic() < deadline:
        if not is_busy():
            return
        time.sleep(0.01)
    raise TimeoutError(f"lcs2dll: marking did not finish within {timeout_s:.0f}s")


def stop() -> None:
    """Abort the currently executing list."""
    _l().stop_execution(CARD)


# ---------------------------------------------------------------------------
# List-building API  (n_ = add-to-list)
# ---------------------------------------------------------------------------

def list_start() -> None:
    """Begin building list 1.  Call before any n_jump / n_mark calls."""
    _check(_l().n_set_start_list_1(CARD), "n_set_start_list_1")


def list_end() -> None:
    """Finalise and execute list 1, then block until marking completes."""
    _check(_l().n_set_end_of_list(CARD), "n_set_end_of_list")
    _check(_l().execute_list_1(CARD), "execute_list_1")
    wait_finished()


def list_jump(x_mm: float, y_mm: float) -> None:
    """Queue a travel move (no laser) in the current list."""
    _check(_l().n_jump_abs(CARD, float(x_mm), float(y_mm)), "n_jump_abs")


def list_mark(x_mm: float, y_mm: float) -> None:
    """Queue a laser-on move in the current list."""
    _check(_l().n_mark_abs(CARD, float(x_mm), float(y_mm)), "n_mark_abs")


def list_laser_on(time_us: int) -> None:
    """Queue a stationary laser pulse (time in microseconds)."""
    _check(_l().n_laser_on_list(CARD, int(time_us)), "n_laser_on_list")


# ---------------------------------------------------------------------------
# Convenience: marking context manager
# ---------------------------------------------------------------------------

@contextmanager
def marking(mark_speed_mm_s: float = 500.0, jump_speed_mm_s: float = 2000.0):
    """
    Context manager that builds and executes a marking list.

    Usage::

        with lcs_api.marking(mark_speed_mm_s=500) as ctx:
            ctx.jump(x0, y0)
            ctx.mark(x1, y1)
            ctx.mark(x2, y2)

    On exit the list is sent to hardware and we wait for completion.
    On exception the list is aborted.
    """
    set_mark_speed(mark_speed_mm_s)
    set_jump_speed(jump_speed_mm_s)
    list_start()

    class _Ctx:
        def jump(self, x, y):       list_jump(x, y)
        def mark(self, x, y):       list_mark(x, y)
        def laser_on(self, us):     list_laser_on(us)
        def speed(self, mm_s):      set_mark_speed(mm_s)
        def power(self, pct):       set_power(pct)

    ctx = _Ctx()
    try:
        yield ctx
    except BaseException:
        try:
            stop()
        except Exception:
            pass
        raise

    list_end()
