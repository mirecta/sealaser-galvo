"""
BJJCZEncoder — translates Rayforge Ops commands to galvoplotter commands.

Produces an EncodedOutput whose driver_data["commands"] is a list of
(method_name, *args) tuples that BJJCZDriver.run() replays through a
GalvoController.

Coordinate conversion (machine origin = center):
    galvo_x = int(GALVO_CENTER + x_mm * galvos_per_mm)
    galvo_y = int(GALVO_CENTER - y_mm * galvos_per_mm)   # Y axis flipped

The Y flip matches the typical physical orientation of galvo mirrors where
increasing Y in Rayforge (upward on screen) maps to decreasing galvo Y.
The flip_y setting in driver_args can disable this when not needed.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, List, Tuple

if TYPE_CHECKING:
    from raygeo.ops import Ops
    from rayforge.core.doc import Doc
    from rayforge.machine.models.machine import Machine

from .coord import GALVO_CENTER, mm_to_galvo

logger = logging.getLogger(__name__)

Command = Tuple[str, Any]


class BJJCZEncoder(OpsEncoder):
    """
    Converts Rayforge Ops to a list of galvoplotter command tuples.

    Stored in EncodedOutput.driver_data["commands"] as:
        [("goto", gx, gy), ("mark", gx, gy), ("power", 50.0), ...]

    The text field holds a human-readable log for the Rayforge UI.
    """

    def encode(self, ops: Ops, machine: "Machine", doc: "Doc") -> EncodedOutput:
        driver_args = (machine.driver_args or {}) if machine else {}
        galvos_per_mm: float = float(driver_args.get("galvos_per_mm", 500))
        flip_y: bool = bool(driver_args.get("flip_y", True))

        commands: List[Command] = []
        text_lines: List[str] = []
        op_map = MachineCodeOpMap()

        for i in range(ops.len()):
            start = len(commands)
            self._handle_command(ops, i, galvos_per_mm, flip_y, commands, text_lines)
            end = len(commands)
            op_map.op_to_machine_code[i] = list(range(start, end))
            for idx in range(start, end):
                op_map.machine_code_to_op[idx] = i

        return EncodedOutput(
            text="\n".join(text_lines),
            op_map=op_map,
            driver_data={"commands": commands},
        )

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    @staticmethod
    def mm_to_galvo(x_mm: float, y_mm: float, galvos_per_mm: float, flip_y: bool) -> Tuple[int, int]:
        return mm_to_galvo(x_mm, y_mm, galvos_per_mm, flip_y)

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------

    def _handle_command(
        self,
        ops: Ops,
        idx: int,
        galvos_per_mm: float,
        flip_y: bool,
        commands: List[Command],
        text: List[str],
    ) -> None:
        ct = ops.command_type(idx)

        if ct == CommandType.SET_POWER:
            power_pct = ops.power(idx) * 100.0
            commands.append(("power", power_pct))
            text.append(f"POWER {power_pct:.1f}%")

        elif ct == CommandType.SET_CUT_SPEED:
            speed = ops.speed(idx)
            commands.append(("mark_speed", speed))
            text.append(f"MARK_SPEED {speed:.1f} mm/s")

        elif ct == CommandType.SET_TRAVEL_SPEED:
            speed = ops.speed(idx)
            commands.append(("travel_speed", speed))
            text.append(f"TRAVEL_SPEED {speed:.1f} mm/s")

        elif ct == CommandType.SET_FREQUENCY:
            freq_hz = ops.frequency(idx)
            freq_khz = freq_hz / 1000.0
            commands.append(("frequency", freq_khz))
            text.append(f"FREQUENCY {freq_khz:.1f} kHz")

        elif ct == CommandType.SET_PULSE_WIDTH:
            pw_ns = ops.pulse_width(idx)
            commands.append(("pulse_width", pw_ns))
            text.append(f"PULSE_WIDTH {pw_ns:.1f} ns")

        elif ct == CommandType.MOVE_TO:
            end = ops.endpoint(idx)
            gx, gy = self.mm_to_galvo(end[0], end[1], galvos_per_mm, flip_y)
            commands.append(("goto", gx, gy))
            text.append(f"GOTO X:{end[0]:.3f} Y:{end[1]:.3f} ({gx:#06x},{gy:#06x})")

        elif ct == CommandType.LINE_TO:
            end = ops.endpoint(idx)
            gx, gy = self.mm_to_galvo(end[0], end[1], galvos_per_mm, flip_y)
            commands.append(("mark", gx, gy))
            text.append(f"MARK X:{end[0]:.3f} Y:{end[1]:.3f} ({gx:#06x},{gy:#06x})")

        elif ct == CommandType.ARC_TO:
            # Galvo controllers have no native arc support — linearize.
            cur_pos = ops.endpoint(idx - 1) if idx > 0 else (0.0, 0.0, 0.0)
            sub = ops.linearize(idx, cur_pos)
            for j in range(sub.len()):
                self._handle_command(sub, j, galvos_per_mm, flip_y, commands, text)

        elif ct == CommandType.SCAN_LINE:
            cur_pos = ops.endpoint(idx - 1) if idx > 0 else (0.0, 0.0, 0.0)
            sub = ops.linearize(idx, cur_pos)
            for j in range(sub.len()):
                self._handle_command(sub, j, galvos_per_mm, flip_y, commands, text)

        elif ct in (CommandType.JOB_START, CommandType.JOB_END,
                    CommandType.LAYER_START, CommandType.LAYER_END,
                    CommandType.WORKPIECE_START, CommandType.WORKPIECE_END):
            # No galvo-level equivalent; handled by the driver lifecycle.
            pass

        else:
            logger.debug("BJJCZEncoder: unhandled command type %s at index %d", ct, idx)
