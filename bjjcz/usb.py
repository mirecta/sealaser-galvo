"""
ConfigurableUSBConnection — galvoplotter USBConnection with overrideable USB IDs.

galvoplotter hardcodes 0x9588:0x9899 (standard BJJCZ LMC controller) and
reads from endpoint 0x88. This subclass overrides find_device() and read()
so any vendor/product ID and read endpoint works, letting the driver support
controllers like the SEA-LASER (04b4:1004) that use a Cypress FX2 USB bridge
with the same LMC protocol but respond on endpoint 0x84.
"""

from __future__ import annotations

import usb.core
from usb.backend.libusb1 import LIBUSB_ERROR_ACCESS

from galvo.usb_connection import WRITE_ENDPOINT, USBConnection


def _(s):
    return s

# SEA-LASER sends 20-byte responses framed as:
#   [0]      0xFE  start marker
#   [1]      0xFF  response type / echo
#   [2:4]    packet length = 0x14 (20)
#   [4:12]   8 bytes of real response data (4×uint16 LE — same as BJJCZ)
#   [12:19]  zero padding
#   [19]     checksum (~sum_of_bytes_0_to_18 & 0xFF)
#
# galvoplotter.send() does struct.unpack("<4H", r) on the read result, so
# read() must return exactly 8 bytes.
_SEA_RESPONSE_SIZE = 32      # read buffer — big enough to capture full packet
_SEA_PAYLOAD_OFFSET = 4      # byte offset of the 8-byte LMC payload
_SEA_PAYLOAD_LENGTH = 8      # standard BJJCZ payload size expected by galvoplotter
_SEA_START_MARKER = 0xFE

# The SEA-LASER (Cypress FX2) responds on 0x84, not galvoplotter's default 0x88.
_SEA_READ_ENDPOINT = 0x84


class ConfigurableUSBConnection(USBConnection):
    """
    USBConnection with configurable USB vendor/product IDs, read endpoint,
    and auto-detected response framing.

    The SEA-LASER (04b4:1004) wraps the standard 8-byte LMC response in
    a 20-byte envelope and responds on endpoint 0x84 (not galvoplotter's
    default 0x88). read() strips the framing and returns the 8-byte
    payload so galvoplotter's struct.unpack("<4H", r) still works.
    """

    def __init__(
        self,
        vendor_id: int,
        product_id: int,
        read_endpoint: int = _SEA_READ_ENDPOINT,
        channel=None,
    ):
        super().__init__(channel=channel)
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.read_endpoint = read_endpoint

    # ------------------------------------------------------------------
    # Override: configurable USB IDs
    # ------------------------------------------------------------------

    def find_device(self, index: int = 0):
        self.channel(_(f"Searching for USB device {self.vendor_id:#06x}:{self.product_id:#06x}"))
        try:
            devices = list(
                usb.core.find(
                    idVendor=self.vendor_id,
                    idProduct=self.product_id,
                    find_all=True,
                )
            )
        except usb.core.USBError as exc:
            self.backend_error_code = exc.backend_error_code
            self.channel(str(exc))
            raise ConnectionRefusedError from exc

        if not devices:
            self.channel(_(f"No device found with {self.vendor_id:#06x}:{self.product_id:#06x}"))
            raise ConnectionRefusedError

        for dev in devices:
            self.channel(_("Galvo device detected:"))
            self.channel(str(dev).replace("\n", "\n\t"))

        try:
            return devices[index]
        except IndexError:
            if self.backend_error_code == LIBUSB_ERROR_ACCESS:
                self.channel(_("Permission denied — add a udev rule or run as root."))
                raise PermissionError
            raise ConnectionRefusedError

    # ------------------------------------------------------------------
    # Override: larger read buffer + framing strip
    # ------------------------------------------------------------------

    def read(self, index: int = 0, attempt: int = 0):
        try:
            raw = bytes(
                self.devices[index].read(
                    endpoint=self.read_endpoint,
                    size_or_buffer=_SEA_RESPONSE_SIZE,
                    timeout=self.timeout,
                )
            )
            return self._extract_payload(raw)
        except usb.core.USBError as exc:
            if attempt <= 3:
                try:
                    self.close(index)
                    self.open(index)
                except ConnectionError:
                    import time
                    time.sleep(1)
                return self.read(index, attempt + 1)
            self.backend_error_code = exc.backend_error_code
            self.channel(str(exc))
            raise ConnectionError from exc
        except KeyError:
            raise ConnectionError("Not Connected.")

    @staticmethod
    def _extract_payload(raw: bytes) -> bytes:
        """
        Extract the 8-byte LMC payload from a raw USB response.

        Standard BJJCZ: exactly 8 bytes — returned as-is.
        SEA-LASER framed: starts with 0xFE, payload at offset 4.
        Fallback: return the first 8 bytes.
        """
        if len(raw) == _SEA_PAYLOAD_LENGTH:
            return raw  # standard BJJCZ — no framing

        if len(raw) > _SEA_PAYLOAD_OFFSET + _SEA_PAYLOAD_LENGTH and raw[0] == _SEA_START_MARKER:
            payload = raw[_SEA_PAYLOAD_OFFSET: _SEA_PAYLOAD_OFFSET + _SEA_PAYLOAD_LENGTH]
            return payload

        # Fallback: truncate/pad to 8 bytes
        return (raw + b"\x00" * _SEA_PAYLOAD_LENGTH)[:_SEA_PAYLOAD_LENGTH]
