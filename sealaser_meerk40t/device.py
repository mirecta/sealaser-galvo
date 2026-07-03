"""
SeaLaserDevice — MeerK40t Service for the SEA-LASER (BJJCZ/USB-Lite,
04b4:1004) galvo controller.

Adapted from MeerK40t's own `balormk/device.py` (MIT licensed), trimmed to
the settings we've actually confirmed against real hardware this far
(field size, per-axis Scale, laser defaults — see bjjcz/galvo_config.py and
project memory "sea-laser-protocol"). Bulge/Skew/Trapezoid correction,
red-dot offset, and other balormk-parity settings are deliberately left out
until validated — see galvo_config.py's docstring for why.
"""

from __future__ import annotations

from meerk40t.core.spoolers import Spooler
from meerk40t.core.units import Length
from meerk40t.core.view import View
from meerk40t.kernel import Service, signal_listener

from .driver import SeaLaserDriver


class SeaLaserDevice(Service):
    """
    MeerK40t device service for the Gweike G2 / SEA-LASER galvo controller.
    """

    def __init__(self, kernel, path, *args, choices=None, **kwargs):
        Service.__init__(self, kernel, path)
        self.name = "sealaser"
        self.extension = "lmc"
        self.job = None

        if choices is not None:
            for c in choices:
                attr = c.get("attr")
                default = c.get("default")
                if attr is not None and default is not None:
                    setattr(self, attr, default)

        _ = kernel.translation
        self.register(
            "format/op cut",
            "{danger}{defop}{enabled}{pass}{element_type} {speed}mm/s @{power} {frequency}kHz {colcode} {opstop}",
        )
        self.register(
            "format/op engrave",
            "{danger}{defop}{enabled}{pass}{element_type} {speed}mm/s @{power} {frequency}kHz {colcode} {opstop}",
        )
        self.register(
            "format/op raster",
            "{danger}{defop}{enabled}{pass}{element_type}{direction}{speed}mm/s @{power} {frequency}kHz {colcode} {opstop}",
        )
        self.register(
            "format/op image",
            "{danger}{defop}{enabled}{penvalue}{pass}{element_type}{direction}{speed}mm/s @{power} {frequency}kHz {colcode}",
        )
        self.register("format/util console", "{enabled}{command}")
        self.setting(bool, "use_percent_for_power_display", True)

        choices = [
            {
                "attr": "label",
                "object": self,
                "default": "sealaser-device",
                "type": str,
                "label": _("Label"),
                "tip": _("What is this device called."),
                "section": "_00_General",
                "priority": "10",
                "signals": "device;renamed",
            },
            {
                "attr": "lens_size",
                "object": self,
                "default": "150mm",
                "type": Length,
                "label": _("Field size"),
                "tip": _("Width/height of the square galvo field (confirmed 150mm on the reference Gweike G2)."),
                "section": "_00_General",
            },
            {
                "attr": "scale_x",
                "object": self,
                "default": 0.9027799963951111,
                "type": float,
                "label": _("X-Scale"),
                "tip": _("Per-axis linear scale correction, read from the device's LightBurn profile."),
                "section": "_10_Calibration",
            },
            {
                "attr": "scale_y",
                "object": self,
                "default": 0.9027199745178223,
                "type": float,
                "label": _("Y-Scale"),
                "tip": _("Per-axis linear scale correction, read from the device's LightBurn profile."),
                "section": "_10_Calibration",
            },
            {
                "attr": "apply_distortion_correction",
                "object": self,
                "default": False,
                "type": bool,
                "label": _("Enable Bulge/Skew/Trapezoid correction"),
                "tip": _(
                    "EXPERIMENTAL/UNVALIDATED: the correction formula is a best-effort "
                    "guess, not reverse-engineered from real protocol bytes like the rest "
                    "of this driver. Test against a real burn before trusting it."
                ),
                "section": "_11_Distortion (experimental)",
            },
            {
                "attr": "bulge_x",
                "object": self,
                "default": 1.0360000133514404,
                "type": float,
                "label": _("Bulge X"),
                "section": "_11_Distortion (experimental)",
                "conditional": (self, "apply_distortion_correction"),
            },
            {
                "attr": "bulge_y",
                "object": self,
                "default": 0.9589999914169312,
                "type": float,
                "label": _("Bulge Y"),
                "section": "_11_Distortion (experimental)",
                "conditional": (self, "apply_distortion_correction"),
            },
            {
                "attr": "skew_x",
                "object": self,
                "default": 0.9649999737739563,
                "type": float,
                "label": _("Skew X"),
                "section": "_11_Distortion (experimental)",
                "conditional": (self, "apply_distortion_correction"),
            },
            {
                "attr": "skew_y",
                "object": self,
                "default": 1.0299999713897705,
                "type": float,
                "label": _("Skew Y"),
                "section": "_11_Distortion (experimental)",
                "conditional": (self, "apply_distortion_correction"),
            },
            {
                "attr": "trapezoid_x",
                "object": self,
                "default": 0.996999979019165,
                "type": float,
                "label": _("Trapezoid X"),
                "section": "_11_Distortion (experimental)",
                "conditional": (self, "apply_distortion_correction"),
            },
            {
                "attr": "trapezoid_y",
                "object": self,
                "default": 0.996999979019165,
                "type": float,
                "label": _("Trapezoid Y"),
                "section": "_11_Distortion (experimental)",
                "conditional": (self, "apply_distortion_correction"),
            },
            {
                "attr": "flip_x",
                "object": self,
                "default": False,
                "type": bool,
                "label": _("Flip X"),
                "section": "_10_Calibration",
            },
            {
                "attr": "flip_y",
                "object": self,
                "default": False,
                "type": bool,
                "label": _("Flip Y"),
                "section": "_10_Calibration",
            },
            {
                "attr": "rotate",
                "object": self,
                "default": 0,
                "type": int,
                "style": "combo",
                "choices": [0, 90, 180, 270],
                "label": _("Rotate"),
                "section": "_10_Calibration",
            },
            {
                "attr": "swap_xy",
                "object": self,
                "default": False,
                "type": bool,
                "label": _("Swap X/Y"),
                "section": "_10_Calibration",
            },
            {
                "attr": "user_margin_x",
                "object": self,
                "default": "0mm",
                "type": Length,
                "label": _("Margin X"),
                "section": "_10_Calibration",
            },
            {
                "attr": "user_margin_y",
                "object": self,
                "default": "0mm",
                "type": Length,
                "label": _("Margin Y"),
                "section": "_10_Calibration",
            },
            {
                "attr": "default_power",
                "object": self,
                "default": 30.0,
                "type": float,
                "label": _("Default power (%)"),
                "section": "_20_Laser",
            },
            {
                "attr": "default_frequency",
                "object": self,
                "default": 20.0,
                "type": float,
                "label": _("Default frequency (kHz)"),
                "tip": _("Reference device: Laser_MinFreq=20, Laser_MaxFreq=80"),
                "section": "_20_Laser",
            },
            {
                "attr": "default_jump_speed",
                "object": self,
                "default": 4000.0,
                "type": float,
                "label": _("Default jump speed (mm/s)"),
                "section": "_20_Laser",
            },
            {
                "attr": "default_mark_speed",
                "object": self,
                "default": 500.0,
                "type": float,
                "label": _("Default mark speed (mm/s)"),
                "section": "_20_Laser",
            },
        ]
        self.register_choices("sealaser-device", choices)

        self.state = 0

        unit_size = float(Length(self.lens_size))
        galvo_range = 0xFFFF
        units_per_galvo = unit_size / galvo_range
        self.view = View(
            self.lens_size,
            self.lens_size,
            native_scale_x=units_per_galvo,
            native_scale_y=units_per_galvo,
        )
        self.realize()

        self.spooler = Spooler(self)
        self.driver = SeaLaserDriver(self)
        self.spooler.driver = self.driver
        self.add_service_delegate(self.spooler)

        self.laser_status = "idle"

    @property
    def safe_label(self):
        if not hasattr(self, "label"):
            return self.name
        name = self.label.replace(" ", "-")
        return name.replace("/", "-")

    @property
    def supports_pwm(self):
        return False

    def outline(self):
        """
        "Outline" toolbar button handler. Without this method, MeerK40t's
        laserpanel.py falls back to "element* trace hull" (convex hull),
        which hits an unrelated upstream bug in Geomstr.convex_hull's numpy
        code for some point sets. "trace quick" (a plain bounding-box
        outline, no convex hull math) avoids it entirely and is good enough
        for a positioning preview.

        NOTE: unlike balormk's outline() (which does a continuous red-dot
        "full-light" trace of the actual shape via a dedicated LightJob),
        this only traces the bounding box and doesn't move the laser at
        all yet — we haven't wired up a red-dot preview job. Good enough
        for now; revisit if a real light-trace is needed.
        """
        self("element* trace quick\n")

    def service_attach(self, *args, **kwargs):
        if hasattr(self.driver, "service_attach"):
            self.driver.service_attach()
        self.realize()

    def service_detach(self, *args, **kwargs):
        if hasattr(self.driver, "service_detach"):
            self.driver.service_detach()

    @signal_listener("lens_size")
    @signal_listener("rotate")
    @signal_listener("flip_x")
    @signal_listener("flip_y")
    @signal_listener("swap_xy")
    @signal_listener("user_margin_x")
    @signal_listener("user_margin_y")
    def realize(self, origin=None, *args):
        if origin is not None and origin != self.path:
            return
        try:
            unit_size = float(Length(self.lens_size))
        except ValueError:
            return
        if unit_size == 0:
            return
        galvo_range = 0xFFFF
        units_per_galvo = unit_size / galvo_range

        self.view.set_dims(self.lens_size, self.lens_size)
        self.view.set_margins(self.user_margin_x, self.user_margin_y)
        self.view.set_native_scale(units_per_galvo, units_per_galvo)
        self.view.transform(
            flip_x=self.flip_x,
            flip_y=self.flip_y,
            swap_xy=self.swap_xy,
        )
        if self.rotate >= 90:
            self.view.rotate_cw()
        if self.rotate >= 180:
            self.view.rotate_cw()
        if self.rotate >= 270:
            self.view.rotate_cw()
        self.signal("view;realized")

    @property
    def current(self):
        return self.view.iposition(self.driver.connection._last_x_native, self.driver.connection._last_y_native)
