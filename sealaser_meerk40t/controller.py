"""
Controller adapter for MeerK40t's generic galvo driver logic.

Presents the primitive method surface a balor-style driver expects
(goto/mark/power/frequency/set_settings/get_last_xy/rapid_mode/
program_mode/set_xy/connect/disconnect), backed by our confirmed-working
bjjcz.sea_usb protocol driver — see project memory "sea-laser-protocol"
for the full reverse-engineered wire format this wraps.

Coordinates arrive from MeerK40t already resolved to native galvo units
(0-0xFFFF, 0x8000=center) by its view/geometry pipeline. sea_usb.py works
in mm, so every call here converts using GalvoConfig before delegating.

Marks are accumulated into a pending batch and only sent to hardware on
flush() (mode change, power/frequency change, or an intervening goto) —
mark_path() needs the whole point list at once to interpolate and batch
correctly (a single isolated mark record does not fire the laser; this was
confirmed against real hardware — see sea_usb.py's mark_path docstring).
"""

from __future__ import annotations

import threading
from typing import List, Optional, Tuple

from bjjcz.galvo_config import GalvoConfig
from bjjcz.sea_usb import GALVO_CENTER, SEALaserUSB


class SeaLaserController:
    def __init__(self, service, force_mock: bool = False):
        self.service = service
        self._force_mock = force_mock

        self.config = GalvoConfig(
            field_width_mm=float(getattr(service, "field_width_mm", 150.0)),
            field_height_mm=float(getattr(service, "field_height_mm", 150.0)),
            scale_x=float(getattr(service, "scale_x", 1.0)),
            scale_y=float(getattr(service, "scale_y", 1.0)),
            jump_speed_mm_s=float(getattr(service, "default_jump_speed", 4000.0)),
            mark_speed_mm_s=float(getattr(service, "default_mark_speed", 500.0)),
            power_pct=float(getattr(service, "default_power", 30.0)),
            freq_khz=float(getattr(service, "default_frequency", 20.0)),
        )
        self._usb = SEALaserUSB(config=self.config)
        self._lock = threading.RLock()

        self._connected = False
        self._pending_points: List[Tuple[float, float]] = []
        self._pending_power: Optional[float] = None
        self._pending_freq: Optional[float] = None
        self._last_x_native = GALVO_CENTER
        self._last_y_native = GALVO_CENTER

        self._power = self.config.power_pct
        self._frequency = self.config.freq_khz
        self._mark_speed = self.config.mark_speed_mm_s
        self._jump_speed = self.config.jump_speed_mm_s

        # Kept for parity with balormk's controller surface — driver.py
        # sets these before a light/travel-only job; unused by sea_usb.py
        # today but harmless to accept.
        self._light_speed = None
        self._dark_speed = None
        self._goto_speed = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._connected

    def connect_if_needed(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._usb.connect()
            self._usb.init()
            self._connected = True

    def disconnect(self) -> None:
        with self._lock:
            try:
                self.flush()
            finally:
                if self._connected:
                    self._usb.disconnect()
                self._connected = False

    def abort_connect(self) -> None:
        pass

    # ------------------------------------------------------------------
    # Coordinate helpers (native galvo units <-> mm)
    # ------------------------------------------------------------------

    def _native_to_mm(self, x: float, y: float) -> Tuple[float, float]:
        x_mm = (x - GALVO_CENTER) / self.config.galvos_per_mm_x
        y_mm = (y - GALVO_CENTER) / self.config.galvos_per_mm_y
        return x_mm, y_mm

    # ------------------------------------------------------------------
    # Primitive API expected by the generic balor-style driver
    # ------------------------------------------------------------------

    def get_last_xy(self) -> Tuple[float, float]:
        return self._last_x_native, self._last_y_native

    def goto(self, x: float, y: float, long=None, short=None, distance_limit=None) -> None:
        with self._lock:
            self.connect_if_needed()
            self.flush()
            x_mm, y_mm = self._native_to_mm(x, y)
            self._usb.goto_xy(x_mm, y_mm)
            self._last_x_native, self._last_y_native = x, y

    def mark(self, x: float, y: float) -> None:
        with self._lock:
            self._pending_points.append((x, y))
            self._last_x_native, self._last_y_native = x, y

    def set_xy(self, x: float, y: float) -> None:
        """Immediate absolute jog, used outside of a running job (e.g. manual positioning)."""
        self.goto(x, y)

    def power(self, power_pct: float) -> None:
        with self._lock:
            if (
                self._pending_points
                and self._pending_power is not None
                and power_pct != self._pending_power
            ):
                self.flush()
            self._power = power_pct
            self._pending_power = power_pct

    def frequency(self, freq_khz: float) -> None:
        with self._lock:
            if (
                self._pending_points
                and self._pending_freq is not None
                and freq_khz != self._pending_freq
            ):
                self.flush()
            self._frequency = freq_khz
            self._pending_freq = freq_khz

    def set_settings(self, settings: dict) -> None:
        """
        Applies a MeerK40t cut/op settings dict. Power scaling (settings["power"]
        is 0-1000 representing 0-100.0%, per MeerK40t's usual convention) is
        UNVERIFIED against actual MeerK40t behavior — confirm against a real
        job before trusting the exact numbers, only the mechanism is solid.
        """
        if settings.get("power") is not None:
            self.power(float(settings["power"]) / 10.0)
        if settings.get("frequency") is not None:
            self.frequency(float(settings["frequency"]))
        if settings.get("speed") is not None:
            self._mark_speed = float(settings["speed"])

    def rapid_mode(self) -> None:
        self.flush()

    def program_mode(self) -> None:
        self.flush()

    def raster_mode(self) -> None:
        pass

    def wait_finished(self) -> None:
        self.flush()

    def flush(self) -> None:
        """Send any accumulated mark points as one continuous batch."""
        with self._lock:
            if not self._pending_points:
                return
            self.connect_if_needed()
            points_mm = [self._native_to_mm(x, y) for x, y in self._pending_points]
            self._pending_points = []
            power = self._pending_power if self._pending_power is not None else self._power
            freq = self._pending_freq if self._pending_freq is not None else self._frequency
            self._usb.mark_path(
                points_mm,
                power_pct=power,
                freq_khz=freq,
                jump_speed_mm_s=self._jump_speed,
                mark_speed_mm_s=self._mark_speed,
            )
            self._pending_power = None
            self._pending_freq = None
