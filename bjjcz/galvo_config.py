"""
Configurable calibration/settings for a BJJCZ/SEA-LASER galvo controller.

Values default to what was read out of a real device's LightBurn profile
("BSLFiber", Gweike G2, prefs.ini) on 2026-07-03. Designed to be constructed
from a Rayforge device.yaml `driver_args` block so each physical machine can
carry its own calibration.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


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

    # --- Nonlinear correction coefficients ---
    # Real values read from the reference device's LightBurn profile. This is
    # a known, standard BJJCZ/SEACad/EZCad calibration scheme (confirmed via
    # web research: LightBurn's own docs, support forum, and a Gweike-G2-
    # specific community setup guide all describe it and source it from the
    # vendor's own GLaser/SEACad config files — matching our own `markcfg0`)
    # but the exact mathematical formula combining these 4 terms is NOT
    # publicly documented anywhere, including by LightBurn itself. The
    # `correct()` method below implements a best-effort model derived from
    # LightBurn's plain-language description of each term's *effect* (bulge=
    # radial distortion strongest at center, skew=linear shear, trapezoid=
    # position-dependent keystone scaling) — this is an UNVALIDATED
    # HYPOTHESIS, not reverse-engineered from captured bytes like everything
    # else in this codebase. `apply_distortion_correction` defaults to False
    # so it doesn't silently change already-confirmed-working behavior;
    # enable and empirically tune against a real test burn before trusting it.
    bulge_x: float = 1.0360000133514404
    bulge_y: float = 0.9589999914169312
    skew_x: float = 0.9649999737739563
    skew_y: float = 1.0299999713897705
    trapezoid_x: float = 0.996999979019165
    trapezoid_y: float = 0.996999979019165
    apply_distortion_correction: bool = False

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

    def correct(self, x_mm: float, y_mm: float) -> Tuple[float, float]:
        """
        Apply Bulge/Skew/Trapezoid correction to a coordinate, BEFORE
        mm->galvo conversion. UNVALIDATED — see the coefficient comments
        above. No-ops (returns input unchanged) unless
        `apply_distortion_correction` is True.

        Model (each coefficient is 1.0 at "no correction", matching how
        LightBurn stores them):
          - Bulge:      radial term, strongest at field center, tapering to
                        zero at the edges — "distorts/stretches the center
                        of shapes".
          - Skew:       linear shear (X shifts with Y position and vice
                        versa) — "pulls the bottom of a design left/right
                        relative to the top".
          - Trapezoid:  position-dependent linear scale (keystone) — "X
                        width scales with Y position" — "stretches/pinches
                        the corners".
        All three are applied additively as small corrections on top of the
        input coordinate; order between them is also unverified.
        """
        if not self.apply_distortion_correction:
            return x_mm, y_mm

        half_w = self.field_width_mm / 2.0
        half_h = self.field_height_mm / 2.0

        bulge_dx = (self.bulge_x - 1.0) * x_mm * (1.0 - (y_mm / half_h) ** 2) if half_h else 0.0
        bulge_dy = (self.bulge_y - 1.0) * y_mm * (1.0 - (x_mm / half_w) ** 2) if half_w else 0.0

        skew_dx = (self.skew_x - 1.0) * y_mm
        skew_dy = (self.skew_y - 1.0) * x_mm

        trap_dx = (self.trapezoid_x - 1.0) * x_mm * (y_mm / half_h) if half_h else 0.0
        trap_dy = (self.trapezoid_y - 1.0) * y_mm * (x_mm / half_w) if half_w else 0.0

        return (
            x_mm + bulge_dx + skew_dx + trap_dx,
            y_mm + bulge_dy + skew_dy + trap_dy,
        )
