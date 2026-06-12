"""
Unit tests for driver position tracking and coordinate helpers.

These tests use only bjjcz.coord (no Rayforge / lcs_api dependency) so they
run without hardware or LightBurn installed.

BJJCZDriver.jog() accumulates deltas in mm and applies flip_y before calling
lcs_api.goto(x, -y).  The math is verified here independently.
"""

import pytest

from bjjcz.coord import GALVO_CENTER, mm_to_galvo


class TestCoordHelper:
    """Verify the coordinate helper used for legacy galvo-unit callers."""

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
    """
    Verify the jog delta calculation that BJJCZDriver.jog() uses.

    Driver tracks (pos_x, pos_y) in Rayforge mm space (before flip_y).
    The call to lcs_api.goto receives (pos_x, -pos_y) when flip_y=True.
    """

    @staticmethod
    def _apply_jog(pos_x, pos_y, dx, dy):
        """Simulate one driver.jog() call: returns new (pos_x, pos_y) in Rayforge space."""
        return pos_x + dx, pos_y + dy

    @staticmethod
    def _to_lcs(pos_x, pos_y, flip_y=True):
        """Apply flip before sending to lcs_api.goto()."""
        return pos_x, (-pos_y if flip_y else pos_y)

    def test_jog_right_1mm(self):
        px, py = self._apply_jog(0.0, 0.0, 1.0, 0.0)
        assert px == 1.0
        assert py == 0.0
        lx, ly = self._to_lcs(px, py)
        assert lx == 1.0
        assert ly == 0.0

    def test_jog_up_1mm_flip_y(self):
        px, py = self._apply_jog(0.0, 0.0, 0.0, 1.0)
        assert px == 0.0
        assert py == 1.0
        lx, ly = self._to_lcs(px, py, flip_y=True)
        assert lx == 0.0
        assert ly == -1.0   # Y flipped when sent to hardware

    def test_jog_diagonal(self):
        px, py = self._apply_jog(0.0, 0.0, 2.0, 3.0)
        lx, ly = self._to_lcs(px, py)
        assert lx == 2.0
        assert ly == -3.0

    def test_jog_from_non_origin(self):
        px, py = self._apply_jog(5.0, -2.0, 1.0, 0.0)
        assert px == 6.0
        assert py == -2.0

    def test_no_flip_y(self):
        px, py = self._apply_jog(0.0, 0.0, 0.0, 4.0)
        lx, ly = self._to_lcs(px, py, flip_y=False)
        assert ly == 4.0    # no flip
