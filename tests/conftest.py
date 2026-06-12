"""Shared test fixtures for rayforge-addon-bjjcz."""
import pytest


@pytest.fixture
def galvos_per_mm():
    return 500


@pytest.fixture
def flip_y():
    return True
