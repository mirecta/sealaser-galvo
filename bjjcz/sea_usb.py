"""
Pure-Python USB driver for BJJCZ/SEA-LASER galvo controllers (USB 04b4:1004).

Protocol reverse-engineered from USB traffic captures and static analysis of
libexecutor.so (LightBurn).

Two endpoint pairs are used:
  EP 0x06 (OUT) / EP 0x88 (IN)  —  EP_CMD: init, status, laser config
  EP 0x02 (OUT) / EP 0x84 (IN)  —  EP_POS: direct position commands

EP_CMD frame format (20 bytes total):
  FE FF 00 14  80 00  CMD1 CMD2  [11 data bytes]  CSUM
  CSUM = (CMD1 + CMD2) & 0xFF
  0x14 = 20 = total frame length

  Known EP_CMD command codes (reverse-engineered from libexecutor.so):
    aa 05 — status poll (response is 40 bytes)
    aa 10 — controller mode select (sub 01 = read, sub 03 = motion enable)
    aa 34 — init handshake
    02 10 — fiber laser pulse parameters (Executor5::sendFiberPower)
    02 11 — laser mode/type select (Executor5::sendLenPara)
    02 12 — secondary laser params (Executor5::sendLenPara, second frame)
    02 18 — first-pulse suppression / FPK (Executor5::sendPreionFPS)
    02 41 — goto XY, no laser (Executor5::sendMoveToAbs)
    02 43 — mark XY with laser on (Executor5::sendLineToAbs, USB/Executor5 variant)

EP_POS frame format (34 bytes, all multi-byte fields big-endian):
  FE FF 00 22  SEQ 00  CMD1 CMD2  12 00
  VSPD_HI VSPD_LO  MSPD_HI MSPD_LO  00
  X_HI X_LO  00  Y_HI Y_LO
  00 00 00 00  0A 00 08 00  00 00 00 02  00 45
  CMD = 02 41 (goto) or 02 43 (mark with laser)

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

_DEFAULT_GALVOS_PER_MM = 65536.0 / 100.0  # 655.36 — 100 mm field (markcfg0: FIELDSIZE=100)

_DEFAULT_JUMP_SPEED  = 5000   # mm/s
_DEFAULT_MARK_SPEED  = 500    # mm/s

# ---------------------------------------------------------------------------
# Init sequence  (TX frames sent to EP 0x06, in order)
# ---------------------------------------------------------------------------
# Captured from liblcs2dll.so during init_dll().
# Responses are read-and-discarded; their content is device-specific calibration
# data that we currently do not need to parse.

_PROBE = bytes.fromhex("012300000000000000000000")  # 12-byte firmware probe

# Init frames sent AFTER the probe handshake (each expects one response).
_INIT_FRAMES: list[tuple[bytes, int]] = [
    # (TX frame,                                                               expected RX size)
    (bytes.fromhex("feff00148000fff10000000000000000000000f0"),  20),  # status query
    (bytes.fromhex("feff00148000f0f00000000000000000000000e0"),  60),  # hw info (60-byte resp)
    (bytes.fromhex("feff00188000fffb010900000000000000000000000000fa"), 20),  # ff fb 09
    (bytes.fromhex("feff00188000fffb011100000000000000000000000000fa"), 28),  # ff fb 11
    (bytes.fromhex("feff00188000fffb011000000000000000000000000000fa"), 20),  # ff fb 10
    (bytes.fromhex("feff00188000fffb011000000000000000000000000000fa"), 20),  # ff fb 10 (repeat)
    (bytes.fromhex("feff00188000fffb010900000000000000000000000000fa"), 20),  # ff fb 09 (repeat)
    (bytes.fromhex("feff00188000fffb010100000000000000000000000000fa"), 24),  # ff fb 01
    (bytes.fromhex("feff00188000fffb010200000000000000000000000000fa"), 24),  # ff fb 02
    (bytes.fromhex("feff00188000fffb010300000000000000000000000000fa"), 24),  # ff fb 03
    (bytes.fromhex("feff00188000fffb010400000000000000000000000000fa"), 24),  # ff fb 04
    (bytes.fromhex("feff00188000fffb010500000000000000000000000000fa"), 24),  # ff fb 05
    (bytes.fromhex("feff00188000fffb011100000000000000000000000000fa"), 28),  # ff fb 11 (repeat)
    (bytes.fromhex("feff00188000fffb010600000000000000000000000000fa"), 24),  # ff fb 06
    (bytes.fromhex("feff00148000aa100001000000000000000000ba"),           20),  # aa 10 sub=01
    (bytes.fromhex("feff00148000aa340000000000000000000000de"),           20),  # aa 34
    (bytes.fromhex("feff00148000aa050000000000000000000000af"),           40),  # aa 05 → status 01
]

# Sent once before the very first goto_xy call to activate motion mode.
_ACTIVATE_GOTO = bytes.fromhex("feff00148000aa100003000000000000000000ba")


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
        self._goto_activated: bool = False   # True after first aa10-sub03 sent
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
        """Run the firmware init sequence."""
        dev = self._require()

        # Probe: device ignores the first attempt; send it twice.
        # First TX has no response — use a short timeout and swallow the error.
        try:
            dev.write(EP_CMD, _PROBE, timeout=200)
            dev.read(EP_RESP, 20, timeout=200)
        except usb.core.USBError:
            pass
        time.sleep(0.1)
        dev.write(EP_CMD, _PROBE, timeout=TIMEOUT_MS)
        dev.read(EP_RESP, 20, timeout=TIMEOUT_MS)

        # Main init sequence
        for frame, resp_size in _INIT_FRAMES:
            dev.write(EP_CMD, frame, timeout=TIMEOUT_MS)
            dev.read(EP_RESP, resp_size, timeout=TIMEOUT_MS)

        self._goto_activated = False

    # ------------------------------------------------------------------
    # Direct motion
    # ------------------------------------------------------------------

    _STATUS_CMD = bytes.fromhex("feff00148000aa050000000000000000000000af")

    def goto_xy(self, x_mm: float, y_mm: float) -> None:
        """Move mirrors to (x_mm, y_mm) with no laser.  Blocking."""
        dev = self._require()

        # Activate motion mode on the first goto call
        if not self._goto_activated:
            dev.write(EP_CMD, _ACTIVATE_GOTO, timeout=TIMEOUT_MS)
            dev.read(EP_RESP, 20, timeout=TIMEOUT_MS)
            self._goto_activated = True

        # Status poll required before every EP 0x02 command
        dev.write(EP_CMD, self._STATUS_CMD, timeout=TIMEOUT_MS)
        dev.read(EP_RESP, 40, timeout=TIMEOUT_MS)

        self._seq += 1
        x_g = self._mm_to_galvo(x_mm)
        y_g = self._mm_to_galvo(y_mm)
        vspeed = self._vector_speed(x_g, y_g)

        frame = self._build_goto(self._seq, vspeed, self._mark_speed, x_g, y_g)
        dev.write(EP_POS, frame, timeout=TIMEOUT_MS)
        # EP 0x84 ack: device only responds if an IN token was pending before the write.
        # liblcs2dll uses async libusb so it pre-submits the read; we skip it here.
        try:
            dev.read(EP_ACK, 20, timeout=50)
        except usb.core.USBError:
            pass

        self._pos_x_galvo = x_g
        self._pos_y_galvo = y_g

    def mark_xy(self, x_mm: float, y_mm: float) -> None:
        """
        Move mirrors to (x_mm, y_mm) WITH laser on.  Blocking.

        Uses command byte 0x43 instead of 0x41 (hypothesis: bit 1 = laser enable).
        Laser parameters (power, frequency) must be set via set_laser_params()
        before calling this.
        """
        dev = self._require()

        if not self._goto_activated:
            dev.write(EP_CMD, _ACTIVATE_GOTO, timeout=TIMEOUT_MS)
            dev.read(EP_RESP, 20, timeout=TIMEOUT_MS)
            self._goto_activated = True

        dev.write(EP_CMD, self._STATUS_CMD, timeout=TIMEOUT_MS)
        dev.read(EP_RESP, 40, timeout=TIMEOUT_MS)

        self._seq += 1
        x_g = self._mm_to_galvo(x_mm)
        y_g = self._mm_to_galvo(y_mm)
        vspeed = self._mark_speed   # mark moves at mark_speed (not jump)

        frame = self._build_mark(self._seq, vspeed, self._mark_speed, x_g, y_g)
        dev.write(EP_POS, frame, timeout=TIMEOUT_MS)
        try:
            dev.read(EP_ACK, 20, timeout=50)
        except usb.core.USBError:
            pass

        self._pos_x_galvo = x_g
        self._pos_y_galvo = y_g

    def set_laser_params(
        self,
        power_pct: int = 30,
        freq_hz: int = 20000,
        pulse_us: int = 200,
    ) -> None:
        """
        Send EP_CMD frames to configure fiber laser parameters before marking.

        Sends two command frames reverse-engineered from libexecutor.so:

        ``02 10`` — fiber laser pulse parameters (Executor5::sendFiberPower)
          Reverse-engineered from libexecutor.so at 0x18c10 using a 48 MHz clock base:
            Bytes 8-9   (BE uint16): period_ticks = int(48_000_000 / freq_hz)
            Bytes 10-11: 00 00 (unused slot 2)
            Bytes 12-13 (BE uint16): pulse_ticks = int(pulse_us * 48)
            Bytes 14-15: 00 00 (unused slot 4)
            Byte 16     (uint8):    power_byte = int(power_pct * 255 / 100)
            Bytes 17-19: 00 00 00 (padding / slot 6 = 0)
            Byte 20     (CSUM):     (0x02 + 0x10) & 0xFF = 0x12

        ``02 11`` — laser mode / type frame (Executor5::sendLenPara)
          Needs verification via USB capture.  The frame below is a best-guess
          for a standard fiber laser (type 5, pulsed mode) based on static analysis
          of sendLenPara at 0x18760.  If the laser still doesn't fire, capture the
          USB traffic produced by lcs_api.set_pulses() and update this frame.

        Call before mark_xy(). Safe to call multiple times.
        """
        dev = self._require()

        # ------------------------------------------------------------------
        # 02 10 — fiber laser power / timing (Executor5::sendFiberPower)
        # ------------------------------------------------------------------
        # Clock base: 48 MHz  (confirmed from libexecutor.so constants 48.0 and 48000.0)
        # Period ticks = clock_hz / freq_hz  = 48_000_000 / freq_hz
        # Pulse ticks  = pulse_us * 48       (48 MHz / 1e6 = 48 ticks per µs)
        # Power byte   = power_pct * 255 // 100  (maps 0-100% → 0-255)
        period_ticks = min(0xFFFF, int(48_000_000 / max(1, freq_hz)))
        pulse_ticks  = min(0xFFFF, int(pulse_us * 48))
        power_byte   = min(0xFF, int(power_pct * 255 // 100))

        # Frame layout for 20-byte EP_CMD frames:
        # FE FF 00 14  80 00  CMD1 CMD2  [11 data bytes]  CSUM
        # Where 0x14 = 20 = total frame length.
        # 11 data bytes are organised as 5 slots:
        #   slot 1 (2B, BE): bytes 8-9
        #   slot 2 (2B, BE): bytes 10-11
        #   slot 3 (2B, BE): bytes 12-13
        #   slot 4 (2B, BE): bytes 14-15
        #   slot 5 (1B):     byte 16
        #   slot 6 (2B, BE): bytes 17-18
        #   CSUM:            byte 19

        frame_0210 = (
            b"\xfe\xff\x00\x14"
            b"\x80\x00"
            b"\x02\x10"
            + struct.pack(">H", period_ticks)  # slot 1: period ticks (2 B, BE)
            + b"\x00\x00"                      # slot 2: unused
            + struct.pack(">H", pulse_ticks)   # slot 3: pulse width ticks (2 B, BE)
            + b"\x00\x00"                      # slot 4: unused
            + bytes([power_byte])              # slot 5: power 0-255 (1 B)
            + b"\x00\x00"                      # slot 6: flags word (0 for standard fiber)
            + b"\x12"                          # CSUM = (0x02 + 0x10) & 0xFF
        )
        assert len(frame_0210) == 20
        dev.write(EP_CMD, frame_0210, timeout=TIMEOUT_MS)
        dev.read(EP_RESP, 20, timeout=TIMEOUT_MS)

        # ------------------------------------------------------------------
        # 02 11 — laser mode frame (Executor5::sendLenPara)
        # Best-guess for fiber laser (type 5, pulsed):
        #   byte 8:  0x55 (fiber laser mode byte, sendLenPara default path)
        #   byte 9:  0x22 (pulsed-mode secondary flag; 0x11 for CW)
        #   bytes 10-11: 0x0000 (no extra mode param for standard fiber / type 5)
        #   bytes 12-18: zeros
        # TODO: verify against a USB capture of lcs_api.set_pulses() + mark_abs().
        # ------------------------------------------------------------------
        frame_0211 = (
            b"\xfe\xff\x00\x14"
            b"\x80\x00"
            b"\x02\x11"
            + bytes([0x55, 0x22])  # mode bytes: fiber pulsed
            + b"\x00\x00"          # slot 2 word
            + b"\x00\x00"          # slot 3 word
            + b"\x00\x00"          # slot 4 word
            + b"\x00"              # slot 5 byte
            + b"\x00\x00"          # slot 6 word
            + b"\x13"              # CSUM = (0x02 + 0x11) & 0xFF
        )
        assert len(frame_0211) == 20
        dev.write(EP_CMD, frame_0211, timeout=TIMEOUT_MS)
        dev.read(EP_RESP, 20, timeout=TIMEOUT_MS)

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
            + b"\x02\x41"                    # command: GOTO_XY (laser off)
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

    @staticmethod
    def _build_mark(seq: int, vspeed: int, mspeed: int, x_g: int, y_g: int) -> bytes:
        """Build a 34-byte mark frame for EP 0x02 (laser on, bit 1 set: 0x43 vs 0x41)."""
        return (
            b"\xfe\xff\x00\x22"
            + bytes([seq & 0xFF, 0x00])
            + b"\x02\x43"                    # command: MARK_XY (laser on — bit 1 set)
            + b"\x12\x00"
            + struct.pack(">H", vspeed)
            + struct.pack(">H", mspeed)
            + b"\x00"
            + struct.pack(">H", x_g)
            + b"\x00"
            + struct.pack(">H", y_g)
            + b"\x00\x00\x00\x00"
            + b"\x0a\x00\x08\x00"
            + b"\x00\x00\x00\x02"
            + b"\x00\x45"
        )
