"""
Unit tests for driver coordinate helpers and jog delta math.

These tests import only from bjjcz.coord so they run without Rayforge.
"""

import pytest

from bjjcz.coord import GALVO_CENTER, mm_to_galvo


class TestCoordHelper:
    """Verify the coordinate helper used by BJJCZDriver._to_galvo()."""

    def test_center(self):
        gx, gy = mm_to_galvo(0.0, 0.0, 500.0, True)
        assert gx == GALVO_CENTER
        assert gy == GALVO_CENTER

    def test_1mm_right(self):
        gx, gy = mm_to_galvo(1.0, 0.0, 500.0, True)
        assert gx == GALVO_CENTER + 500
        assert gy == GALVO_CENTER

    def test_1mm_up_flipped(self):
        gx, gy = mm_to_galvo(0.0, 1.0, 500.0, True)
        assert gx == GALVO_CENTER
        assert gy == GALVO_CENTER - 500


class TestJogDeltaMath:
    """Verify the jog delta calculation mirrors BJJCZDriver.jog()."""

    def _calc_jog(self, last_x, last_y, dx_mm, dy_mm, galvos_per_mm=500, flip_y=True):
        new_gx = int(last_x + dx_mm * galvos_per_mm)
        new_gy = int(last_y + (-dy_mm if flip_y else dy_mm) * galvos_per_mm)
        return new_gx, new_gy

    def test_jog_right_1mm(self):
        gx, gy = self._calc_jog(GALVO_CENTER, GALVO_CENTER, 1.0, 0.0)
        assert gx == GALVO_CENTER + 500
        assert gy == GALVO_CENTER

    def test_jog_up_1mm_flipped(self):
        gx, gy = self._calc_jog(GALVO_CENTER, GALVO_CENTER, 0.0, 1.0)
        assert gx == GALVO_CENTER
        assert gy == GALVO_CENTER - 500

    def test_jog_diagonal(self):
        gx, gy = self._calc_jog(GALVO_CENTER, GALVO_CENTER, 2.0, 3.0)
        assert gx == GALVO_CENTER + 1000
        assert gy == GALVO_CENTER - 1500

    def test_jog_from_non_center(self):
        start_x = GALVO_CENTER + 1000  # already 2mm right
        start_y = GALVO_CENTER
        gx, gy = self._calc_jog(start_x, start_y, 1.0, 0.0)
        assert gx == GALVO_CENTER + 1500
        assert gy == GALVO_CENTER
