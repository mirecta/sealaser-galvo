"""
BJJCZEncoder — translates Rayforge Ops commands to lcs_api commands.

Produces an EncodedOutput whose driver_data["commands"] is a list of
(cmd, *args) tuples.  BJJCZDriver.run() replays them through lcs_api.

Movement commands carry mm coordinates (origin = field centre).
Y-axis flip is applied here: when flip_y=True, y_out = -y_mm so that
Rayforge "up" maps to the correct physical beam direction.

Commands emitted:
  ("goto",        x_mm, y_mm)  — travel (no laser)
  ("mark",        x_mm, y_mm)  — mark (laser on)
  ("power",       pct)         — 0–100 %
  ("mark_speed",  mm_s)        — marking speed
  ("travel_speed",mm_s)        — travel speed
  ("frequency",   hz)          — pulse repetition rate
  ("pulse_width", ns)          — pulse width
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, List, Tuple

if TYPE_CHECKING:
    from raygeo.ops import Ops
    from rayforge.core.doc import Doc
    from rayforge.machine.models.machine import Machine

logger = logging.getLogger(__name__)

Command = Tuple[str, Any]


class BJJCZEncoder(OpsEncoder):
    """
    Converts Rayforge Ops to a list of lcs_api command tuples.

    Stored in EncodedOutput.driver_data["commands"] as:
        [("goto", x_mm, y_mm), ("mark", x_mm, y_mm), ("power", 50.0), ...]

    The text field holds a human-readable log for the Rayforge UI.
    """

    def encode(self, ops: Ops, machine: "Machine", doc: "Doc") -> EncodedOutput:
        driver_args = (machine.driver_args or {}) if machine else {}
        flip_y: bool = bool(driver_args.get("flip_y", True))

        commands: List[Command] = []
        text_lines: List[str] = []
        op_map = MachineCodeOpMap()

        for i in range(ops.len()):
            start = len(commands)
            self._handle_command(ops, i, flip_y, commands, text_lines)
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
    def to_lcs(x_mm: float, y_mm: float, flip_y: bool) -> Tuple[float, float]:
        """Convert Rayforge mm coords to lcs_api mm coords (apply Y flip)."""
        return x_mm, (-y_mm if flip_y else y_mm)

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------

    def _handle_command(
        self,
        ops: Ops,
        idx: int,
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
            commands.append(("frequency", freq_hz))
            text.append(f"FREQUENCY {freq_hz:.0f} Hz")

        elif ct == CommandType.SET_PULSE_WIDTH:
            pw_ns = ops.pulse_width(idx)
            commands.append(("pulse_width", pw_ns))
            text.append(f"PULSE_WIDTH {pw_ns:.1f} ns")

        elif ct == CommandType.MOVE_TO:
            end = ops.endpoint(idx)
            x, y = self.to_lcs(end[0], end[1], flip_y)
            commands.append(("goto", x, y))
            text.append(f"GOTO X:{end[0]:.3f} Y:{end[1]:.3f}")

        elif ct == CommandType.LINE_TO:
            end = ops.endpoint(idx)
            x, y = self.to_lcs(end[0], end[1], flip_y)
            commands.append(("mark", x, y))
            text.append(f"MARK X:{end[0]:.3f} Y:{end[1]:.3f}")

        elif ct == CommandType.ARC_TO:
            cur_pos = ops.endpoint(idx - 1) if idx > 0 else (0.0, 0.0, 0.0)
            sub = ops.linearize(idx, cur_pos)
            for j in range(sub.len()):
                self._handle_command(sub, j, flip_y, commands, text)

        elif ct == CommandType.SCAN_LINE:
            cur_pos = ops.endpoint(idx - 1) if idx > 0 else (0.0, 0.0, 0.0)
            sub = ops.linearize(idx, cur_pos)
            for j in range(sub.len()):
                self._handle_command(sub, j, flip_y, commands, text)

        elif ct in (CommandType.JOB_START, CommandType.JOB_END,
                    CommandType.LAYER_START, CommandType.LAYER_END,
                    CommandType.WORKPIECE_START, CommandType.WORKPIECE_END):
            pass

        else:
            logger.debug("BJJCZEncoder: unhandled command type %s at index %d", ct, idx)
