"""
Configurable calibration/settings for a BJJCZ/SEA-LASER galvo controller.

Values default to what was read out of a real device's LightBurn profile
("BSLFiber", Gweike G2, prefs.ini) on 2026-07-03. Designed to be constructed
from a Rayforge device.yaml `driver_args` block so each physical machine can
carry its own calibration.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GalvoConfig:
    # --- Field geometry ---
    field_width_mm: float = 150.0
    field_height_mm: float = 150.0

    # --- Per-axis linear scale correction ---
    # Confirmed: LightBurn applies this as a fine-trim multiplier on the
    # nominal (65536 / field_size_mm) conversion. CorScaleOverride=false on
    # the reference device, i.e. no binary .cor lookup table is used — this
    # scalar model IS the actual correction in effect.
    scale_x: float = 0.9027799963951111
    scale_y: float = 0.9027199745178223

    # --- Nonlinear correction coefficients (NOT YET APPLIED) ---
    # Present in the LightBurn profile but the exact formula combining them
    # is unverified — do not use until validated against a calibration-grid
    # capture (see project memory: sea-laser-protocol). Kept here so the
    # config shape is stable and Rayforge device.yaml can already carry them.
    bulge_x: float = 1.0360000133514404
    bulge_y: float = 0.9589999914169312
    skew_x: float = 0.9649999737739563
    skew_y: float = 1.0299999713897705
    trapezoid_x: float = 0.996999979019165
    trapezoid_y: float = 0.996999979019165

    # --- Motion defaults ---
    jump_speed_mm_s: float = 4000.0
    mark_speed_mm_s: float = 500.0

    # --- Laser defaults ---
    power_pct: float = 30.0
    freq_khz: float = 20.0
    freq_min_khz: float = 20.0
    freq_max_khz: float = 80.0
    laser_type: int = 1  # 1 = fiber (confirmed: Laser_Type=1 on reference device)

    @property
    def galvos_per_mm_x(self) -> float:
        return (65536.0 / self.field_width_mm) * self.scale_x

    @property
    def galvos_per_mm_y(self) -> float:
        return (65536.0 / self.field_height_mm) * self.scale_y
