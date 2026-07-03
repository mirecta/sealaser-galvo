"""
SEA-LASER Galvo Device Plugin for MeerK40t.

Registers this package as a MeerK40t device provider, discovered via the
"meerk40t.extension" entry-point group (see pyproject.toml). Structure
follows MeerK40t's own `balormk/plugin.py` (MIT licensed).
"""


def plugin(kernel, lifecycle):
    if lifecycle == "plugins":
        return []
    elif lifecycle == "invalidate":
        try:
            import usb.core  # noqa: F401
            import usb.util  # noqa: F401
        except ImportError:
            print("SEA-LASER plugin could not load because pyusb is not installed.")
            return True
    if lifecycle == "register":
        from .device import SeaLaserDevice

        kernel.register("provider/device/sealaser", SeaLaserDevice)
        kernel.register("provider/friendly/sealaser", ("Fibre-Laser", 3))
        _ = kernel.translation
        kernel.register(
            "dev_info/sealaser-fiber",
            {
                "provider": "provider/device/sealaser",
                "friendly_name": _("Gweike G2 / SEA-LASER Fibre-Laser (USB-Lite)"),
                "extended_info": _(
                    "Pure-USB driver for the BJJCZ/SEA-LASER USB-Lite galvo controller "
                    "(Cypress FX2, USB 04b4:1004) used in the Gweike G2 fiber laser. "
                    "No vendor SDK/DLL required."
                ),
                "priority": 9,
                "family": _("Generic Fibre-Laser"),
                "choices": [
                    {
                        "attr": "label",
                        "default": "Gweike-G2",
                    },
                ],
            },
        )
    elif lifecycle == "preboot":
        prefix = "sealaser"
        for d in kernel.settings.section_startswith(prefix):
            kernel.root(f"service device start -p {d} {prefix}\n")
