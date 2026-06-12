"""
Pure-Python USB driver for BJJCZ/SEA-LASER galvo controllers (USB 04b4:1004).

Protocol reverse-engineered from USB traffic captures.

Two endpoint pairs are used:
  EP 0x06 (OUT) / EP 0x88 (IN)  —  init sequence, status polling
  EP 0x02 (OUT) / EP 0x84 (IN)  —  direct position commands (goto_xy)

Frame format on EP 0x06:
  FE FF 00 LL  80 00  CMD1 CMD2  [data…]  CSUM
  where CSUM = (CMD1 + CMD2) & 0xFF

Frame format on EP 0x02 (goto):
  FE FF 00 22  SEQ_HI SEQ_LO  02 41  12 00
  VSPD_HI VSPD_LO  MSPD_HI MSPD_LO  00
  X_HI X_LO  00  Y_HI Y_LO
  00 00 00 00  0A 00 08 00  00 00 00 02  00 45
  (34 bytes, all multi-byte fields big-endian)

Coordinates: galvo = 0x8000 + int(mm * GALVOS_PER_MM)
  GALVOS_PER_MM ≈ 546.15  (empirically: 10923 units / 20 mm)
  Override with SEA_GALVOS_PER_MM environment variable.
"""

from __future__ import annotations

import math
import os
import struct
import time
from typing import Optional

import usb.core
import usb.util

# ---------------------------------------------------------------------------
# USB constants
# ---------------------------------------------------------------------------

VID = 0x04B4
PID = 0x1004

EP_CMD  = 0x06   # bulk OUT  — init / list commands
EP_RESP = 0x88   # bulk IN   — responses
EP_POS  = 0x02   # bulk OUT  — goto_xy commands
EP_ACK  = 0x84   # bulk IN   — goto ack

TIMEOUT_MS = 2000

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

GALVO_CENTER: int = 0x8000  # 32768 — center of the galvo range

_DEFAULT_GALVOS_PER_MM = 65536.0 / 120.0  # ≈ 546.133…  (120 mm field)

_DEFAULT_JUMP_SPEED  = 5000   # mm/s
_DEFAULT_MARK_SPEED  = 500    # mm/s

# ---------------------------------------------------------------------------
# Init sequence  (TX frames sent to EP 0x06, in order)
# ---------------------------------------------------------------------------
# Captured from liblcs2dll.so during init_dll().
# Responses are read-and-discarded; their content is device-specific calibration
# data that we currently do not need to parse.

_INIT_FRAMES: list[bytes] = [
    # 1. Firmware-version probe  (raw 12 bytes, no FE FF header)
    bytes.fromhex("012300000000000000000000"),
    # 2. Status query  (ff f1)
    bytes.fromhex("feff00148000fff10000000000000000000000f0"),
    # 3. HW info query  (f0 f0) — 60-byte response
    bytes.fromhex("feff00148000f0f00000000000000000000000e0"),
    # 4–15. Config / calibration reads  (ff fb sub-commands)
    bytes.fromhex("feff00188000fffb010900000000000000000000000000fa"),
    bytes.fromhex("feff00188000fffb011100000000000000000000000000fa"),
    bytes.fromhex("feff00188000fffb011000000000000000000000000000fa"),
    bytes.fromhex("feff00188000fffb011000000000000000000000000000fa"),  # repeated
    bytes.fromhex("feff00188000fffb010900000000000000000000000000fa"),  # repeated
    bytes.fromhex("feff00188000fffb010100000000000000000000000000fa"),
    bytes.fromhex("feff00188000fffb010200000000000000000000000000fa"),
    bytes.fromhex("feff00188000fffb010300000000000000000000000000fa"),
    bytes.fromhex("feff00188000fffb010400000000000000000000000000fa"),
    bytes.fromhex("feff00188000fffb010500000000000000000000000000fa"),
    bytes.fromhex("feff00188000fffb011100000000000000000000000000fa"),  # repeated
    bytes.fromhex("feff00188000fffb010600000000000000000000000000fa"),
    # 16. Enter ready state
    bytes.fromhex("feff00148000aa100001000000000000000000ba"),
    bytes.fromhex("feff00148000aa340000000000000000000000de"),
    bytes.fromhex("feff00148000aa050000000000000000000000af"),  # status → 01 (IDLE)
    bytes.fromhex("feff00148000aa100003000000000000000000ba"),  # activate goto mode
]

# Expected response lengths for each init frame (for reads)
_INIT_RESP_SIZES: list[int] = [
    20,   # 1  fw version
    20,   # 2  status
    60,   # 3  hw info
    20,   # 4  ff fb 09
    28,   # 5  ff fb 11
    20,   # 6  ff fb 10
    20,   # 7  ff fb 10 (repeat)
    20,   # 8  ff fb 09 (repeat)
    24,   # 9  ff fb 01
    24,   # 10 ff fb 02
    24,   # 11 ff fb 03
    24,   # 12 ff fb 04
    24,   # 13 ff fb 05
    28,   # 14 ff fb 11 (repeat)
    24,   # 15 ff fb 06
    20,   # 16 aa 10 sub=01
    20,   # 17 aa 34
    40,   # 18 aa 05 (status)
    20,   # 19 aa 10 sub=03
]


# ---------------------------------------------------------------------------
# SEALaserUSB
# ---------------------------------------------------------------------------

class SEALaserUSB:
    """
    Pure-Python USB driver for BJJCZ/SEA-LASER galvo controllers.

    Usage::

        dev = SEALaserUSB()
        dev.connect()
        dev.init()
        dev.goto_xy(20.0, 0.0)
        dev.goto_xy(0.0, 0.0)
        dev.disconnect()

    All coordinates are in mm, origin at field centre, +X right, +Y up.
    """

    def __init__(
        self,
        galvos_per_mm: Optional[float] = None,
        jump_speed: int = _DEFAULT_JUMP_SPEED,
        mark_speed: int = _DEFAULT_MARK_SPEED,
    ) -> None:
        self._galvos_per_mm: float = galvos_per_mm or float(
            os.environ.get("SEA_GALVOS_PER_MM", _DEFAULT_GALVOS_PER_MM)
        )
        self._jump_speed = jump_speed
        self._mark_speed = mark_speed
        self._seq: int = 0
        self._pos_x_galvo: int = GALVO_CENTER
        self._pos_y_galvo: int = GALVO_CENTER
        self._dev: Optional[usb.core.Device] = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open the USB device (must be run as root or with udev rule)."""
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise IOError(
                f"BJJCZ controller not found (USB {VID:04x}:{PID:04x}). "
                "Check USB connection and permissions."
            )
        # Detach kernel driver if active
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
        dev.set_configuration()
        self._dev = dev

    def disconnect(self) -> None:
        """Release the USB device."""
        if self._dev is not None:
            usb.util.dispose_resources(self._dev)
            self._dev = None

    def __enter__(self) -> "SEALaserUSB":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    # ------------------------------------------------------------------
    # Init
    # ------------------------------------------------------------------

    def init(self) -> None:
        """Run the firmware init sequence and activate goto mode."""
        dev = self._require()
        for frame, resp_size in zip(_INIT_FRAMES, _INIT_RESP_SIZES):
            dev.write(EP_CMD, frame, timeout=TIMEOUT_MS)
            dev.read(EP_RESP, resp_size, timeout=TIMEOUT_MS)

    # ------------------------------------------------------------------
    # Direct motion
    # ------------------------------------------------------------------

    def goto_xy(self, x_mm: float, y_mm: float) -> None:
        """Move mirrors to (x_mm, y_mm) with no laser.  Blocking."""
        dev = self._require()
        self._seq += 1

        x_g = self._mm_to_galvo(x_mm)
        y_g = self._mm_to_galvo(y_mm)

        vspeed = self._vector_speed(x_g, y_g)

        frame = self._build_goto(self._seq, vspeed, self._mark_speed, x_g, y_g)
        dev.write(EP_POS, frame, timeout=TIMEOUT_MS)
        dev.read(EP_ACK, 20, timeout=TIMEOUT_MS)

        self._pos_x_galvo = x_g
        self._pos_y_galvo = y_g

    def get_status(self) -> int:
        """
        Poll the controller status word (cmd aa 05).
        Returns the raw status byte (0x01 = active/idle, 0x03 = motion pending).
        """
        dev = self._require()
        CMD = bytes.fromhex("feff00148000aa050000000000000000000000af")
        dev.write(EP_CMD, CMD, timeout=TIMEOUT_MS)
        resp = bytes(dev.read(EP_RESP, 40, timeout=TIMEOUT_MS))
        # Status byte at offset 10 in the 40-byte response
        return resp[10] if len(resp) >= 11 else 0

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _require(self) -> usb.core.Device:
        if self._dev is None:
            raise RuntimeError("Not connected — call connect() first")
        return self._dev

    def _mm_to_galvo(self, mm: float) -> int:
        val = GALVO_CENTER + round(mm * self._galvos_per_mm)
        return max(0, min(0xFFFF, val))

    def _vector_speed(self, x_g: int, y_g: int) -> int:
        """
        Compute the vector speed for the jump.

        The firmware expects the Euclidean magnitude of the per-axis speeds:
          - axis-aligned move:  speed = jump_speed
          - diagonal move:      speed = jump_speed * √2
        """
        dx = abs(x_g - self._pos_x_galvo)
        dy = abs(y_g - self._pos_y_galvo)
        if dx > 0 and dy > 0:
            return round(self._jump_speed * math.sqrt(2))
        return self._jump_speed

    @staticmethod
    def _build_goto(seq: int, vspeed: int, mspeed: int, x_g: int, y_g: int) -> bytes:
        """Build a 34-byte goto frame for EP 0x02."""
        return (
            b"\xfe\xff\x00\x22"              # magic + length
            + bytes([seq & 0xFF, 0x00])      # sequence (single byte + pad)
            + b"\x02\x41"                    # command: GOTO_XY
            + b"\x12\x00"                # constant
            + struct.pack(">H", vspeed)  # vector speed (BE)
            + struct.pack(">H", mspeed)  # mark speed (BE)
            + b"\x00"
            + struct.pack(">H", x_g)     # X galvo (BE)
            + b"\x00"
            + struct.pack(">H", y_g)     # Y galvo (BE)
            + b"\x00\x00\x00\x00"
            + b"\x0a\x00\x08\x00"
            + b"\x00\x00\x00\x02"
            + b"\x00\x45"               # constant footer
        )
