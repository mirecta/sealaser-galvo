"""
Backend entry point for the bjjcz addon.

Loaded in both worker and main processes. Registers the BJJCZDriver
with Rayforge's driver registry so it appears in the machine wizard.
"""

from rayforge.core.hooks import hookimpl


@hookimpl
def rayforge_init(context):
    from rayforge.machine.driver import register_driver

    from .driver import BJJCZDriver

    register_driver(BJJCZDriver)
