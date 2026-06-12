"""
Galvo coordinate conversion — no external dependencies.

Hardware reference:
    The BJJCZ LMC controller uses 16-bit unsigned integer coordinates.
    The center of the scan field is at (0x8000, 0x8000) = (32768, 32768).
    galvos_per_mm converts physical millimetres to galvo units.

Rayforge uses mm with configurable origin. For galvo lasers the profile
sets origin=center so (0 mm, 0 mm) maps to the center of the scan field.
"""

GALVO_CENTER = 0x8000  # 32768
GALVO_MIN = 0x0000
GALVO_MAX = 0xFFFF


def mm_to_galvo(
    x_mm: float,
    y_mm: float,
    galvos_per_mm: float,
    flip_y: bool,
) -> tuple[int, int]:
    """
    Convert (x_mm, y_mm) from Rayforge machine space to galvo integer units.

    Args:
        x_mm: X position in mm, origin at field center.
        y_mm: Y position in mm, origin at field center.
        galvos_per_mm: Scale factor; typically 500 for BJJCZ systems.
        flip_y: When True, positive Y in Rayforge maps to decreasing galvo Y.
                Matches the typical physical orientation of galvo mirrors.

    Returns:
        (gx, gy) clamped to [0x0000, 0xFFFF].
    """
    gx = int(GALVO_CENTER + x_mm * galvos_per_mm)
    gy = int(GALVO_CENTER + (-y_mm if flip_y else y_mm) * galvos_per_mm)
    gx = max(GALVO_MIN, min(GALVO_MAX, gx))
    gy = max(GALVO_MIN, min(GALVO_MAX, gy))
    return gx, gy
