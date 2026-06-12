"""
BJJCZDriver — Rayforge driver for BJJCZ/JCZ LMC galvo controllers.

Uses liblcs2dll.so (shipped with LightBurn) via ctypes.  All blocking calls
are offloaded to a thread pool with asyncio.to_thread().

Coordinate convention: mm, origin at field centre.  flip_y negates the
Y axis so Rayforge's "up" matches the physical beam direction.
"""

from __future__ import annotations

import asyncio
import logging
from gettext import gettext as _
from typing import TYPE_CHECKING, Any, Awaitable, Callable, Dict, List, Optional, Tuple, Union

from rayforge.context import RayforgeContext
from rayforge.core.varset import IntVar, VarSet
from rayforge.machine.driver.driver import (
    DeviceError,
    DeviceState,
    DeviceStatus,
    Driver,
    DriverMaturity,
    DriverPrecheckError,
    DriverSetupError,
    Pos,
)
from rayforge.machine.transport import TransportStatus

if TYPE_CHECKING:
    from raygeo.ops import Ops
    from raygeo.ops.axis import Axis

    from rayforge.core.doc import Doc
    from rayforge.machine.models.laser import Laser
    from rayforge.machine.models.machine import Machine
    from rayforge.pipeline.encoder.base import EncodedOutput, OpsEncoder

from . import lcs_api

logger = logging.getLogger(__name__)


class BJJCZDriver(Driver):
    """
    Driver for BJJCZ/JCZ LMC galvo laser controllers.

    Requires liblcs2dll.so from LightBurn in the path defined by LCS_LIB_PATH
    (default: ~/.local/share/LightBurn/lib/).
    """

    label = _("BJJCZ Galvo (USB)")
    subtitle = _("Connect to a BJJCZ/JCZ LMC galvo controller over USB")
    uses_gcode = False
    maturity = DriverMaturity.EXPERIMENTAL
    supports_settings = False
    reports_granular_progress = False

    def __init__(self, context: RayforgeContext, machine: "Machine"):
        super().__init__(context, machine)
        self._flip_y: bool = True
        self._mark_speed: float = 500.0
        self._jump_speed: float = 2000.0
        self._power_pct: float = 20.0
        self._pos_x: float = 0.0   # current position in mm
        self._pos_y: float = 0.0
        self._ready: bool = False
        self._connection_task: Optional[asyncio.Task] = None
        self._keep_running: bool = False

    # ------------------------------------------------------------------
    # Abstract property implementations
    # ------------------------------------------------------------------

    @property
    def machine_space_wcs(self) -> str:
        return "MACHINE"

    @property
    def machine_space_wcs_display_name(self) -> str:
        return _("Machine Coordinates")

    @property
    def supported_wcs(self) -> List[str]:
        return ["MACHINE"]

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    @classmethod
    def precheck(cls, **kwargs: Any) -> None:
        pass

    @classmethod
    def get_setup_vars(cls) -> "VarSet":
        return VarSet(vars=[
            IntVar(
                key="mark_speed",
                label=_("Mark speed (mm/s)"),
                description=_("Default laser marking speed in mm/s."),
                default=500,
            ),
            IntVar(
                key="jump_speed",
                label=_("Jump speed (mm/s)"),
                description=_("Default mirror travel speed (no laser) in mm/s."),
                default=2000,
            ),
        ])

    def _setup_implementation(self, **kwargs: Any) -> None:
        driver_args = self._machine.driver_args or {}
        self._flip_y     = bool(driver_args.get("flip_y", True))
        self._mark_speed = float(driver_args.get("mark_speed",
                                 kwargs.get("mark_speed", 500)))
        self._jump_speed = float(driver_args.get("jump_speed",
                                 kwargs.get("jump_speed", 2000)))
        try:
            lcs_api.load()
        except FileNotFoundError as exc:
            raise DriverSetupError(str(exc)) from exc

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    async def _connect_implementation(self) -> None:
        if self._connection_task and not self._connection_task.done():
            return
        self._keep_running = True
        self._connection_task = asyncio.create_task(self._connection_loop())

    async def _connection_loop(self) -> None:
        while self._keep_running:
            self._emit_transport(TransportStatus.CONNECTING)
            try:
                await asyncio.to_thread(lcs_api.init)
                self._ready = True
                self._emit_transport(TransportStatus.CONNECTED)
                self.state.status = DeviceStatus.IDLE
                self.state_changed.send(self, state=self.state)
                logger.info("BJJCZ controller connected")

                while self._keep_running:
                    await asyncio.sleep(2.0)
                    # Light health check: get_status() returns 0 when idle
                    try:
                        lcs_api.get_status()
                    except Exception:
                        break

            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.error("BJJCZ connection error: %s", exc)
                self._ready = False
                self.state.error = DeviceError(-1, str(exc), _("Connection failed"))
                self._emit_transport(TransportStatus.ERROR, str(exc))
                if self._keep_running:
                    self._emit_transport(TransportStatus.SLEEPING)
                    await asyncio.sleep(5.0)

    async def cleanup(self) -> None:
        self._keep_running = False
        if self._connection_task:
            self._connection_task.cancel()
            try:
                await self._connection_task
            except asyncio.CancelledError:
                pass
            self._connection_task = None
        if self._ready:
            try:
                await asyncio.to_thread(lcs_api.free)
            except Exception:
                pass
            self._ready = False
        self._emit_transport(TransportStatus.DISCONNECTED)
        await super().cleanup()

    # ------------------------------------------------------------------
    # Job execution
    # ------------------------------------------------------------------

    @classmethod
    def create_encoder(cls, machine: "Machine") -> "OpsEncoder":
        from .encoder import BJJCZEncoder
        return BJJCZEncoder()

    async def run(
        self,
        encoded: "EncodedOutput",
        doc: "Doc",
        ops: "Ops",
        on_command_done: Optional[Callable[[int], Union[None, Awaitable[None]]]] = None,
    ) -> None:
        assert self._ready, "Controller not connected"
        commands = encoded.driver_data.get("commands", [])

        mark_speed = self._mark_speed
        jump_speed = self._jump_speed

        def _execute() -> None:
            lcs_api.set_mark_speed(mark_speed)
            lcs_api.set_jump_speed(jump_speed)
            lcs_api.list_start()

            for entry in commands:
                cmd, *args = entry
                if cmd == "goto":
                    lcs_api.list_jump(args[0], args[1])
                elif cmd == "mark":
                    lcs_api.list_mark(args[0], args[1])
                elif cmd == "mark_speed":
                    lcs_api.set_mark_speed(float(args[0]))
                elif cmd == "travel_speed":
                    lcs_api.set_jump_speed(float(args[0]))
                elif cmd == "power":
                    lcs_api.set_power(float(args[0]))
                elif cmd == "frequency":
                    pass  # TODO: n_set_laser_pulses inside list
                elif cmd == "pulse_width":
                    pass  # TODO: n_set_laser_pulses inside list
                else:
                    logger.debug("BJJCZ: unhandled command %s", cmd)

            lcs_api.list_end()  # sends + waits

        self.state.status = DeviceStatus.RUN
        self.state_changed.send(self, state=self.state)
        try:
            await asyncio.to_thread(_execute)
        finally:
            self.state.status = DeviceStatus.IDLE
            self.state_changed.send(self, state=self.state)
            self.job_finished.send(self)

    async def run_raw(self, machine_code: str) -> None:
        if machine_code and machine_code.strip():
            logger.warning("BJJCZ controllers do not support raw text commands.")
        self.job_finished.send(self)

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def _apply_flip(self, x_mm: float, y_mm: float) -> Tuple[float, float]:
        return x_mm, (-y_mm if self._flip_y else y_mm)

    async def move_to(self, pos_x: float, pos_y: float) -> None:
        assert self._ready
        x, y = self._apply_flip(pos_x, pos_y)
        await asyncio.to_thread(lcs_api.goto, x, y)
        self._pos_x, self._pos_y = pos_x, pos_y

    def can_home(self, axis=None) -> bool:
        return True

    async def home(self, axes=None) -> None:
        assert self._ready
        await asyncio.to_thread(lcs_api.goto, 0.0, 0.0)
        self._pos_x, self._pos_y = 0.0, 0.0

    def can_jog(self, axis=None) -> bool:
        return True

    async def jog(self, speed: int, **deltas: float) -> None:
        assert self._ready
        new_x = self._pos_x + deltas.get("x", 0.0)
        new_y = self._pos_y + deltas.get("y", 0.0)
        await self.move_to(new_x, new_y)

    # ------------------------------------------------------------------
    # Hold / cancel
    # ------------------------------------------------------------------

    async def set_hold(self, hold: bool = True) -> None:
        if not self._ready:
            return
        if hold:
            await asyncio.to_thread(lcs_api.stop)

    async def cancel(self) -> None:
        if not self._ready:
            return
        await asyncio.to_thread(lcs_api.stop)
        self.state.status = DeviceStatus.IDLE
        self.state_changed.send(self, state=self.state)

    # ------------------------------------------------------------------
    # Power
    # ------------------------------------------------------------------

    async def set_power(self, head: "Laser", percent: float) -> None:
        if not self._ready:
            return
        await asyncio.to_thread(lcs_api.set_power, percent * 100.0)

    async def set_focus_power(self, head: "Laser", percent: float) -> None:
        await self.set_power(head, percent)

    # ------------------------------------------------------------------
    # Settings (stubs)
    # ------------------------------------------------------------------

    def get_setting_vars(self) -> List["VarSet"]:
        return [VarSet(title=_("No configurable settings"))]

    async def read_settings(self) -> None:
        await asyncio.sleep(0)
        self.settings_read.send(self, settings=[])

    async def write_setting(self, key: str, value: Any) -> None:
        pass

    async def clear_alarm(self) -> None:
        pass

    # ------------------------------------------------------------------
    # Tool / WCS stubs
    # ------------------------------------------------------------------

    async def select_tool(self, tool_number: int) -> None:
        pass

    async def set_wcs_offset(self, wcs_slot: str, x: float, y: float, z: float) -> None:
        pass

    async def read_wcs_offsets(self) -> Dict[str, Pos]:
        offsets: Dict[str, Pos] = {"MACHINE": (0.0, 0.0, 0.0)}
        self.wcs_updated.send(self, offsets=offsets)
        return offsets

    async def run_probe_cycle(self, axis, max_travel: float, feed_rate: int) -> Optional[Pos]:
        return None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _emit_transport(self, status: "TransportStatus", message: str = "") -> None:
        self.connection_status_changed.send(self, status=status, message=message)
