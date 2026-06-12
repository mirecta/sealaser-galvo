"""
BJJCZDriver — Rayforge driver for BJJCZ/JCZ LMC galvo controllers.

Wraps galvoplotter.GalvoController (threading model) inside Rayforge's
async Driver interface using asyncio.to_thread() for all blocking calls.

Supports any LMC-protocol galvo controller reachable over USB.
Common USB IDs:
  - 0x9588:0x9899 — standard BJJCZ LMC controller
  - 0x04b4:0x1004 — SEA-LASER (Cypress FX2 USB bridge, same LMC protocol)
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

from .coord import GALVO_CENTER, mm_to_galvo

logger = logging.getLogger(__name__)


class _SEALaserController:
    """
    Mixin that replaces the two galvoplotter methods that poll the READY bit.

    USB-Lite firmware (e.g. SEA-LASER / SEATHINKING) only toggles the BUSY
    bit (0x04) during list execution.  The READY bit (0x20) is never set, so
    galvoplotter's wait_ready() and wait_finished() loop forever.

    LightBurn's WaitForCompletion exits when BUSY=0 — the same logic as the
    wait_finished() override here.  wait_ready() simply returns immediately
    because the USB-Lite device is always ready to receive list data.
    """

    def wait_ready(self):
        return  # READY bit never set; device is always ready

    def wait_finished(self):
        import time as _t
        t_settle = _t.monotonic() + 0.1
        while _t.monotonic() < t_settle and not self.is_busy():
            _t.sleep(0.005)
        t_end = _t.monotonic() + 600.0
        while self.is_busy() and _t.monotonic() < t_end:
            _t.sleep(0.01)
            if not self._sending:
                return


# Galvo command → GalvoController method mapping used in run()
_CMD_METHOD: Dict[str, str] = {
    "goto": "goto",
    "mark": "mark",
    "power": "set_power",
    "mark_speed": "set_mark_speed",
    "travel_speed": "set_travel_speed",
    "frequency": "set_frequency",
    "pulse_width": "set_pulse_width",
}


class BJJCZDriver(Driver):
    """
    Driver for BJJCZ/JCZ LMC galvo laser controllers.

    Uses the galvoplotter library for low-level USB communication.
    The controller runs in a background thread; asyncio.to_thread()
    bridges every blocking call into Rayforge's async event loop.
    """

    label = _("BJJCZ Galvo (USB)")
    subtitle = _("Connect to a BJJCZ/JCZ LMC galvo controller over USB")
    uses_gcode = False
    maturity = DriverMaturity.EXPERIMENTAL
    supports_settings = False
    reports_granular_progress = False

    def __init__(self, context: RayforgeContext, machine: "Machine"):
        super().__init__(context, machine)
        self._controller = None
        self._galvos_per_mm: float = 500.0
        self._flip_y: bool = True
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
        galvos = kwargs.get("galvos_per_mm", 500)
        if not isinstance(galvos, (int, float)) or galvos <= 0:
            raise DriverPrecheckError(_("galvos_per_mm must be a positive number."))

    @classmethod
    def get_setup_vars(cls) -> "VarSet":
        return VarSet(
            vars=[
                IntVar(
                    key="usb_vendor_id",
                    label=_("USB Vendor ID"),
                    description=_(
                        "USB vendor ID in decimal. "
                        "0x9588 (38280) = standard BJJCZ LMC; "
                        "0x04b4 (1204) = Cypress/SEA-LASER."
                    ),
                    default=1204,  # 0x04b4 — Cypress / SEA-LASER
                ),
                IntVar(
                    key="usb_product_id",
                    label=_("USB Product ID"),
                    description=_(
                        "USB product ID in decimal. "
                        "0x9899 (39065) = standard BJJCZ LMC; "
                        "0x1004 (4100) = SEA-LASER."
                    ),
                    default=4100,  # 0x1004 — SEA-LASER
                ),
                IntVar(
                    key="galvos_per_mm",
                    label=_("Galvos per mm"),
                    description=_(
                        "Scale factor: galvo units per millimetre. "
                        "Adjust to match your correction file calibration. "
                        "Default 500 suits most BJJCZ systems."
                    ),
                    default=500,
                ),
                IntVar(
                    key="machine_index",
                    label=_("USB device index"),
                    description=_(
                        "Index of the USB controller when multiple devices "
                        "are connected. Usually 0."
                    ),
                    default=0,
                ),
                IntVar(
                    key="read_endpoint",
                    label=_("USB read endpoint"),
                    description=_(
                        "USB bulk-in endpoint address. "
                        "0x84 (132) = SEA-LASER/Cypress FX2; "
                        "0x88 (136) = standard BJJCZ LMC."
                    ),
                    default=132,  # 0x84 — SEA-LASER
                ),
            ]
        )

    def _setup_implementation(self, **kwargs: Any) -> None:
        from galvo.controller import GalvoController

        from .usb import ConfigurableUSBConnection

        driver_args = self._machine.driver_args or {}
        source = driver_args.get("source", "fiber")
        self._galvos_per_mm = float(
            driver_args.get("galvos_per_mm", kwargs.get("galvos_per_mm", 500))
        )
        self._flip_y = bool(driver_args.get("flip_y", True))
        machine_index = int(kwargs.get("machine_index", driver_args.get("machine_index", 0)))

        # USB IDs: prefer setup-var value, fall back to driver_args, then defaults.
        # Stored as decimal in VarSet; device.yaml hex strings are parsed by int().
        vendor_id = int(
            kwargs.get("usb_vendor_id", driver_args.get("usb_vendor_id", 0x04B4))
        )
        product_id = int(
            kwargs.get("usb_product_id", driver_args.get("usb_product_id", 0x1004))
        )
        read_endpoint = int(
            kwargs.get("read_endpoint", driver_args.get("read_endpoint", 0x84))
        )

        try:
            # Build a one-off subclass that patches out the READY-bit waits.
            SEAController = type(
                "SEAController",
                (_SEALaserController, GalvoController),
                {},
            )
            self._controller = SEAController(
                galvos_per_mm=int(self._galvos_per_mm),
                machine_index=machine_index,
            )
            self._controller.source = source
            # Inject our configurable connection so galvoplotter doesn't
            # create its hardcoded USBConnection(0x9588:0x9899).
            self._controller.connection = ConfigurableUSBConnection(
                vendor_id=vendor_id,
                product_id=product_id,
                read_endpoint=read_endpoint,
                channel=self._controller.usb_log,
            )
        except Exception as exc:
            raise DriverSetupError(str(exc)) from exc

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    async def _connect_implementation(self) -> None:
        if not self._controller:
            self._emit_transport(TransportStatus.DISCONNECTED, "Not configured")
            return
        if self._connection_task and not self._connection_task.done():
            return
        self._keep_running = True
        self._connection_task = asyncio.create_task(self._connection_loop())

    async def _connection_loop(self) -> None:
        while self._keep_running:
            self._emit_transport(TransportStatus.CONNECTING)
            try:
                await asyncio.to_thread(self._controller.connect_if_needed)
                self._emit_transport(TransportStatus.CONNECTED)
                self.state.status = DeviceStatus.IDLE
                self.state_changed.send(self, state=self.state)
                logger.info("BJJCZ controller connected", extra=self._log_extra("MACHINE_EVENT"))

                # Poll connection health every 2 s
                while self._keep_running and self._controller.is_connected:
                    await asyncio.sleep(2.0)
                    self._sync_state()

                if self._keep_running:
                    logger.warning("BJJCZ controller lost connection, retrying")
                    self._emit_transport(TransportStatus.SLEEPING)
                    await asyncio.sleep(5.0)

            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.error("BJJCZ connection error: %s", exc)
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
        if self._controller and self._controller.is_connected:
            await asyncio.to_thread(self._controller.disconnect)
        self._controller = None
        self._emit_transport(TransportStatus.DISCONNECTED)
        await super().cleanup()

    # ------------------------------------------------------------------
    # Job execution
    # ------------------------------------------------------------------

    @classmethod
    def create_encoder(cls, machine: "Machine") -> "OpsEncoder":
        return BJJCZEncoder()

    async def run(
        self,
        encoded: "EncodedOutput",
        doc: "Doc",
        ops: "Ops",
        on_command_done: Optional[Callable[[int], Union[None, Awaitable[None]]]] = None,
    ) -> None:
        assert self._controller, "Controller not initialised"
        commands = encoded.driver_data.get("commands", [])

        def _job(c) -> bool:
            for entry in commands:
                cmd, *args = entry
                method_name = _CMD_METHOD.get(cmd)
                if method_name:
                    getattr(c, method_name)(*args)
                else:
                    logger.debug("BJJCZ: unknown command %s", cmd)
            return True

        self.state.status = DeviceStatus.RUN
        self.state_changed.send(self, state=self.state)
        try:
            with self._controller.marking() as ctx:
                await asyncio.to_thread(_job, ctx)
            await asyncio.to_thread(self._controller.wait_finished)
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

    async def move_to(self, pos_x: float, pos_y: float) -> None:
        assert self._controller
        gx, gy = self._to_galvo(pos_x, pos_y)
        await asyncio.to_thread(self._controller.jog, gx, gy)

    def can_home(self, axis: Optional[Axis] = None) -> bool:
        return True

    async def home(self, axes: Optional[Axis] = None) -> None:
        assert self._controller
        await asyncio.to_thread(self._controller.jog, GALVO_CENTER, GALVO_CENTER)

    def can_jog(self, axis: Optional[Axis] = None) -> bool:
        return True

    async def jog(self, speed: int, **deltas: float) -> None:
        assert self._controller
        last_x, last_y = self._controller.get_last_xy()
        dx_mm = deltas.get("x", 0.0)
        dy_mm = deltas.get("y", 0.0)
        new_gx = int(last_x + dx_mm * self._galvos_per_mm)
        new_gy = int(last_y + (-dy_mm if self._flip_y else dy_mm) * self._galvos_per_mm)
        await asyncio.to_thread(self._controller.jog, new_gx, new_gy)

    # ------------------------------------------------------------------
    # Hold / cancel
    # ------------------------------------------------------------------

    async def set_hold(self, hold: bool = True) -> None:
        assert self._controller
        if hold:
            await asyncio.to_thread(self._controller.pause)
        else:
            await asyncio.to_thread(self._controller.resume)

    async def cancel(self) -> None:
        assert self._controller
        await asyncio.to_thread(self._controller.abort)
        self.state.status = DeviceStatus.IDLE
        self.state_changed.send(self, state=self.state)

    # ------------------------------------------------------------------
    # Power
    # ------------------------------------------------------------------

    async def set_power(self, head: "Laser", percent: float) -> None:
        assert self._controller
        await asyncio.to_thread(self._controller.set_power, percent * 100.0)

    async def set_focus_power(self, head: "Laser", percent: float) -> None:
        await self.set_power(head, percent)

    # ------------------------------------------------------------------
    # Settings (stubs — LMC config is handled via correction files)
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
    # Tool / WCS stubs (not applicable to galvo)
    # ------------------------------------------------------------------

    async def select_tool(self, tool_number: int) -> None:
        pass

    async def set_wcs_offset(self, wcs_slot: str, x: float, y: float, z: float) -> None:
        pass

    async def read_wcs_offsets(self) -> Dict[str, Pos]:
        offsets: Dict[str, Pos] = {"MACHINE": (0.0, 0.0, 0.0)}
        self.wcs_updated.send(self, offsets=offsets)
        return offsets

    async def run_probe_cycle(
        self, axis: Axis, max_travel: float, feed_rate: int
    ) -> Optional[Pos]:
        return None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _to_galvo(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        return mm_to_galvo(x_mm, y_mm, self._galvos_per_mm, self._flip_y)

    def _sync_state(self) -> None:
        if not self._controller:
            return
        raw = self._controller.state
        if raw is None:
            return
        status_str, _ = raw
        mapping = {
            "idle": DeviceStatus.IDLE,
            "busy": DeviceStatus.RUN,
            "hold": DeviceStatus.HOLD,
        }
        new_status = mapping.get(status_str, DeviceStatus.UNKNOWN)
        if new_status != self.state.status:
            self.state.status = new_status
            self.state_changed.send(self, state=self.state)

    def _emit_transport(self, status: "TransportStatus", message: str = "") -> None:
        self.connection_status_changed.send(self, status=status, message=message)
