"""
Galvo driver for the SEA-LASER (BJJCZ/USB-Lite, 04b4:1004) backend.

Adapted from MeerK40t's own `balormk/driver.py` (MIT licensed) — that file
is almost entirely generic MeerK40t-facing plot/job logic, not specific to
the standard 0x9588 BJJCZ protocol it normally targets. This version keeps
that generic shape but delegates every hardware call to
`sealaser_meerk40t.controller.SeaLaserController`, which wraps our own
confirmed-working `bjjcz.sea_usb` protocol driver instead of MeerK40t's
built-in `balormk.controller.GalvoController`.

Deliberately NOT ported in this first pass (kept as honest no-ops/simple
stubs rather than silently missing): footpedal polling, rotary/cylinder
axis wrapping, GPIO input/output cut handling (OutputCut/InputCut — our
board's port-I/O protocol hasn't been reverse-engineered). Dwell/Wait cuts
get a simple time.sleep-based implementation rather than a real hardware
dwell command, since sea_usb.py doesn't have one yet.
"""

from __future__ import annotations

import time

from meerk40t.core.cutcode.cubiccut import CubicCut
from meerk40t.core.cutcode.dwellcut import DwellCut
from meerk40t.core.cutcode.gotocut import GotoCut
from meerk40t.core.cutcode.homecut import HomeCut
from meerk40t.core.cutcode.linecut import LineCut
from meerk40t.core.cutcode.plotcut import PlotCut
from meerk40t.core.cutcode.quadcut import QuadCut
from meerk40t.core.cutcode.waitcut import WaitCut
from meerk40t.core.geomstr import Geomstr
from meerk40t.core.plotplanner import PlotPlanner
from meerk40t.device.basedevice import PLOT_FINISH, PLOT_JOG, PLOT_RAPID, PLOT_SETTING

from .controller import SeaLaserController


class SeaLaserDriver:
    def __init__(self, service, force_mock: bool = False):
        self.service = service
        self.name = str(service)

        self.connection = SeaLaserController(service, force_mock=force_mock)
        self.service.add_service_delegate(self.connection)

        self.paused = False
        self.is_relative = False
        self.laser = False
        self._shutdown = False
        self._aborting = False

        self.queue = list()
        self._queue_current = 0
        self._queue_total = 0
        self.plot_planner = PlotPlanner(
            dict(),
            single=True,
            ppi=False,
            shift=False,
            group=True,
            require_uniform_movement=False,
        )
        self.value_penbox = None
        self.plot_planner.settings_then_jog = True

    def __repr__(self) -> str:
        return f"SeaLaserDriver({self.name})"

    @property
    def connected(self) -> bool:
        return self.connection is not None and self.connection.connected

    def service_attach(self) -> None:
        self._shutdown = False

    def service_detach(self) -> None:
        self._shutdown = True

    def connect(self) -> None:
        try:
            self.connection.connect_if_needed()
        except Exception:
            return

    def disconnect(self) -> None:
        self.connection.disconnect()

    def abort_retry(self) -> None:
        self.connection.abort_connect()

    # ------------------------------------------------------------------
    # Job lifecycle
    # ------------------------------------------------------------------

    def job_start(self, job) -> None:
        self._aborting = False

    def hold_work(self, priority) -> bool:
        return False

    def get_internal_queue_status(self):
        return self._queue_current, self._queue_total

    def _set_queue_status(self, current, total) -> None:
        self._queue_current = current
        self._queue_total = total

    def get(self, key, default=None):
        return getattr(self, key, default)

    def set(self, key, value) -> None:
        setattr(self, key, value)

    def status(self):
        return "idle" if not self.connected else "connected"

    def laser_off(self, *values) -> None:
        self.connection.flush()

    def laser_on(self, *values) -> None:
        pass

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------

    def plot(self, plot) -> None:
        self.queue.append(plot)

    def plot_start(self) -> None:
        """Called after all cutcode objects are queued for this job."""
        self.service.laser_status = "active"
        con = self.connection
        con.program_mode()

        queue = self.queue
        self.queue = list()
        total = len(queue)
        current = 0
        last_on = None

        for q in queue:
            current += 1
            self._set_queue_status(current, total)
            settings = q.settings
            con.set_settings(settings)

            if self._abort_mission():
                return
            while self.paused:
                time.sleep(0.05)

            if isinstance(q, LineCut):
                last_x, last_y = con.get_last_xy()
                x, y = q.start
                if last_x != x or last_y != y:
                    con.goto(x, y)
                con.mark(*q.end)
            elif isinstance(q, QuadCut):
                last_x, last_y = con.get_last_xy()
                x, y = q.start
                if last_x != x or last_y != y:
                    con.goto(x, y)
                g = Geomstr()
                g.quad(complex(*q.start), complex(*q.c()), complex(*q.end))
                for p in list(g.as_equal_interpolated_points(distance=self.service.interp))[1:]:
                    if self._abort_mission():
                        return
                    con.mark(p.real, p.imag)
            elif isinstance(q, CubicCut):
                last_x, last_y = con.get_last_xy()
                x, y = q.start
                if last_x != x or last_y != y:
                    con.goto(x, y)
                g = Geomstr()
                g.cubic(
                    complex(*q.start),
                    complex(*q.c1()),
                    complex(*q.c2()),
                    complex(*q.end),
                )
                for p in list(g.as_equal_interpolated_points(distance=self.service.interp))[1:]:
                    if self._abort_mission():
                        return
                    con.mark(p.real, p.imag)
            elif isinstance(q, PlotCut):
                last_x, last_y = con.get_last_xy()
                x, y = q.start
                if last_x != x or last_y != y:
                    con.goto(x, y)
                for ox, oy, on, x, y in q.plot:
                    if self._abort_mission():
                        return
                    if on == 0:
                        con.goto(x, y)
                    else:
                        if last_on is None or on != last_on:
                            last_on = on
                            max_power = float(q.settings.get("power", self.service.default_power))
                            percent_power = max_power / 10.0
                            con.power(percent_power * on)
                        con.mark(x, y)
            elif isinstance(q, DwellCut):
                start = q.start
                con.goto(*start)
                con.flush()
                time.sleep(q.dwell_time / 1000.0)
            elif isinstance(q, WaitCut):
                time.sleep(q.dwell_time / 1000.0)
            elif isinstance(q, (HomeCut, GotoCut)):
                con.goto(0x8000, 0x8000)
            else:
                # Rastercut and anything else routed through the plot planner
                self.plot_planner.push(q)
                for x, y, on in self.plot_planner.gen():
                    if self._abort_mission():
                        return
                    if on > 1:
                        if on & PLOT_FINISH:
                            break
                        elif on & PLOT_SETTING:
                            settings = self.plot_planner.settings
                            con.set_settings(settings)
                        elif on & (PLOT_RAPID | PLOT_JOG):
                            con.goto(x, y)
                        continue
                    if on == 0:
                        con.goto(x, y)
                    else:
                        if last_on is None or on != last_on:
                            last_on = on
                            settings = self.plot_planner.settings
                            percent_power = float(
                                settings.get("power", self.service.default_power)
                            ) / 10.0
                            con.power(percent_power * on)
                        con.mark(x, y)

        con.flush()
        con.rapid_mode()

    # ------------------------------------------------------------------
    # Direct motion
    # ------------------------------------------------------------------

    def move_abs(self, x, y) -> None:
        native_x, native_y = self.service.view.position(x, y)
        native_x = max(0, min(0xFFFF, native_x))
        native_y = max(0, min(0xFFFF, native_y))
        self.connection.set_xy(native_x, native_y)

    def move_rel(self, dx, dy, confined=False) -> None:
        last_x, last_y = self.connection.get_last_xy()
        unit_dx, unit_dy = self.service.view.position(dx, dy, vector=True)
        native_x = max(0, min(0xFFFF, last_x + unit_dx))
        native_y = max(0, min(0xFFFF, last_y + unit_dy))
        self.connection.set_xy(native_x, native_y)

    def home(self) -> None:
        self.move_abs("50%", "50%")

    def physical_home(self) -> None:
        self.home()

    def rapid_mode(self) -> None:
        self.connection.rapid_mode()

    def program_mode(self) -> None:
        self.connection.program_mode()

    def raster_mode(self, *args) -> None:
        self.connection.raster_mode()

    def wait_finished(self) -> None:
        self.connection.wait_finished()

    def function(self, function) -> None:
        function()

    def wait(self, time_in_ms) -> None:
        time.sleep(time_in_ms / 1000.0)

    def console(self, value) -> None:
        self.service(value)

    def beep(self) -> None:
        self.service("beep\n")

    def signal(self, signal, *args) -> None:
        self.service.signal(signal, *args)

    def pause(self) -> None:
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def reset(self) -> None:
        self.paused = False
        self._aborting = True
        self.queue = list()

    def dwell(self, time_in_ms, settings=None) -> None:
        if settings:
            self.connection.set_settings(settings)
        self.connection.flush()
        time.sleep(time_in_ms / 1000.0)

    def pulse(self, pulse_time, power=None) -> None:
        if power is not None:
            self.connection.power(power)
        x, y = self.connection.get_last_xy()
        self.connection.mark(x, y)
        self.connection.flush()

    def set_abort(self) -> None:
        self._aborting = True

    def _abort_mission(self) -> bool:
        return self._aborting
