"""
Unit tests for coordinate conversion and encoder logic.

These tests import only from bjjcz.coord (no Rayforge/raygeo dependency)
so they run without a full Rayforge installation.
"""

import pytest

from bjjcz.coord import GALVO_CENTER, GALVO_MAX, GALVO_MIN, mm_to_galvo


class TestMmToGalvo:
    def test_center_maps_to_galvo_center(self):
        gx, gy = mm_to_galvo(0.0, 0.0, galvos_per_mm=500, flip_y=True)
        assert gx == GALVO_CENTER
        assert gy == GALVO_CENTER

    def test_positive_x_increases_galvo_x(self):
        gx, gy = mm_to_galvo(1.0, 0.0, galvos_per_mm=500, flip_y=True)
        assert gx == GALVO_CENTER + 500
        assert gy == GALVO_CENTER

    def test_negative_x_decreases_galvo_x(self):
        gx, gy = mm_to_galvo(-1.0, 0.0, galvos_per_mm=500, flip_y=True)
        assert gx == GALVO_CENTER - 500
        assert gy == GALVO_CENTER

    def test_positive_y_decreases_galvo_y_when_flipped(self):
        gx, gy = mm_to_galvo(0.0, 1.0, galvos_per_mm=500, flip_y=True)
        assert gx == GALVO_CENTER
        assert gy == GALVO_CENTER - 500

    def test_positive_y_increases_galvo_y_when_not_flipped(self):
        gx, gy = mm_to_galvo(0.0, 1.0, galvos_per_mm=500, flip_y=False)
        assert gx == GALVO_CENTER
        assert gy == GALVO_CENTER + 500

    def test_g2_full_field_corner_in_range(self):
        # G2: 110mm field, ±55mm from center
        gx, gy = mm_to_galvo(55.0, 55.0, galvos_per_mm=500, flip_y=True)
        assert GALVO_MIN <= gx <= GALVO_MAX
        assert GALVO_MIN <= gy <= GALVO_MAX

    def test_out_of_range_clamped_high(self):
        gx, gy = mm_to_galvo(200.0, 0.0, galvos_per_mm=500, flip_y=True)
        assert gx == GALVO_MAX

    def test_out_of_range_clamped_low(self):
        gx, gy = mm_to_galvo(-200.0, 0.0, galvos_per_mm=500, flip_y=True)
        assert gx == GALVO_MIN

    def test_positive_y_clamps_to_min_when_flipped(self):
        # large positive Y with flip → galvo Y goes very low → clamped to MIN
        gx, gy = mm_to_galvo(0.0, 200.0, galvos_per_mm=500, flip_y=True)
        assert gy == GALVO_MIN

    def test_scale_factor_respected(self):
        gx, gy = mm_to_galvo(2.0, 0.0, galvos_per_mm=1000, flip_y=True)
        assert gx == GALVO_CENTER + 2000

    def test_symmetry_x(self):
        gx_pos, _ = mm_to_galvo(10.0, 0.0, galvos_per_mm=500, flip_y=True)
        gx_neg, _ = mm_to_galvo(-10.0, 0.0, galvos_per_mm=500, flip_y=True)
        assert gx_pos - GALVO_CENTER == GALVO_CENTER - gx_neg

    def test_symmetry_y_flipped(self):
        _, gy_pos = mm_to_galvo(0.0, 10.0, galvos_per_mm=500, flip_y=True)
        _, gy_neg = mm_to_galvo(0.0, -10.0, galvos_per_mm=500, flip_y=True)
        # +10mm → center - 5000; -10mm → center + 5000: symmetric around center
        assert GALVO_CENTER - gy_pos == gy_neg - GALVO_CENTER
