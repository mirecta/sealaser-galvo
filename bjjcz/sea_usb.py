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

from .galvo_config import GalvoConfig

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
        config: Optional[GalvoConfig] = None,
        galvos_per_mm: Optional[float] = None,
        jump_speed: Optional[int] = None,
        mark_speed: Optional[int] = None,
    ) -> None:
        """
        `config` carries per-machine calibration (field size, scale, laser
        defaults) and is the intended path for Rayforge device.yaml wiring.
        `galvos_per_mm`/`jump_speed`/`mark_speed` are legacy overrides kept
        for backwards compatibility with earlier standalone test scripts.
        """
        self.config: GalvoConfig = config or GalvoConfig()
        self._galvos_per_mm_x: float = galvos_per_mm or float(
            os.environ.get("SEA_GALVOS_PER_MM", self.config.galvos_per_mm_x)
        )
        self._galvos_per_mm_y: float = galvos_per_mm or float(
            os.environ.get("SEA_GALVOS_PER_MM", self.config.galvos_per_mm_y)
        )
        self._jump_speed = jump_speed or int(self.config.jump_speed_mm_s)
        self._mark_speed = mark_speed or int(self.config.mark_speed_mm_s)
        # Step size used to chain mark_xy() into multiple compact records,
        # matching the density of a real captured burn (single-record marks
        # did not fire — see mark_xy docstring).
        self._interp_step_mm: float = 0.5
        self._seq: int = 0
        self._pos_x_galvo: int = GALVO_CENTER
        self._pos_y_galvo: int = GALVO_CENTER
        self._goto_activated: bool = False   # True after first aa10-sub03 sent
        self._dev: Optional[usb.core.Device] = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self, reset: bool = True) -> None:
        """
        Open the USB device (must be run as root or with udev rule).

        A prior script run (especially one that crashed or was Ctrl+C'd
        mid-sequence) can leave the firmware's endpoint buffers holding
        stale queued data, causing the NEXT run's init() to read a leftover
        response instead of the one it expects and time out further down
        the sequence. `dev.reset()` issues a real USB port reset, clearing
        this — pass reset=False only if you know the device is already in
        a clean state (e.g. immediately after a previous connect()+reset()
        in the same process).
        """
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise IOError(
                f"BJJCZ controller not found (USB {VID:04x}:{PID:04x}). "
                "Check USB connection and permissions."
            )

        if reset:
            try:
                dev.reset()
            except usb.core.USBError:
                pass
            time.sleep(0.5)  # let the device re-enumerate/settle
            dev = usb.core.find(idVendor=VID, idProduct=PID)
            if dev is None:
                raise IOError(
                    f"BJJCZ controller disappeared after USB reset "
                    f"(USB {VID:04x}:{PID:04x}). Check USB connection."
                )

        # Detach kernel driver if active
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
        dev.set_configuration()
        self._dev = dev

        # Drain any stale queued IN data left over from a previous session
        # (defensive — reset() above should already have cleared this).
        for ep in (EP_RESP, EP_ACK):
            for _ in range(4):
                try:
                    dev.read(ep, 64, timeout=20)
                except usb.core.USBError:
                    break

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

        # Main init sequence. Retry each step a couple of times — even after
        # connect()'s reset, individual USB transactions can transiently
        # time out on this firmware.
        for frame, resp_size in _INIT_FRAMES:
            self._write_read_retry(EP_CMD, frame, EP_RESP, resp_size)

        self._goto_activated = False

    def _write_read_retry(
        self, out_ep: int, frame: bytes, in_ep: int, resp_size: int, attempts: int = 3,
    ) -> bytes:
        last_exc: Optional[Exception] = None
        for attempt in range(attempts):
            try:
                self._dev.write(out_ep, frame, timeout=TIMEOUT_MS)
                return bytes(self._dev.read(in_ep, resp_size, timeout=TIMEOUT_MS))
            except usb.core.USBError as exc:
                last_exc = exc
                time.sleep(0.1 * (attempt + 1))
        raise last_exc

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
        x_g = self._mm_to_galvo_x(x_mm)
        y_g = self._mm_to_galvo_y(y_mm)
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

    # 38-byte "prepare mark batch" frame, captured verbatim before a real
    # burn's parameter+mark batch. Purpose of its internal fields is not
    # understood (they did not correlate with power/freq/speed changes
    # across captures) but replaying it verbatim, at this exact position
    # in the sequence, was necessary for a confirmed real fire — a
    # configure_laser()+mark_xy() split with this frame omitted moved the
    # galvo but did NOT fire (only the red pointer showed). Treated as a
    # required, opaque mode-entry signal. See project memory: sea-laser-protocol.
    _PREP_BODY = bytes.fromhex(
        "02120600000002210c00ffffffff00000000021b0c0000000000000000000014"
    )

    # Max EP_POS frame body size before splitting into a continuation write.
    # 504 bytes was the largest single frame observed in a real capture
    # (LightBurn itself split a longer batch into a 504-byte + 168-byte
    # pair) — matching that ceiling here rather than pushing past what's
    # confirmed to work on this firmware.
    _MAX_FRAME_BODY = 498

    def mark_xy(
        self,
        x_mm: float,
        y_mm: float,
        power_pct: Optional[float] = None,
        freq_khz: Optional[float] = None,
        jump_speed_mm_s: Optional[float] = None,
        mark_speed_mm_s: Optional[float] = None,
    ) -> None:
        """Mark a single segment from the current position to (x_mm, y_mm). See mark_path()."""
        self.mark_path([(x_mm, y_mm)], power_pct, freq_khz, jump_speed_mm_s, mark_speed_mm_s)

    def mark_path(
        self,
        points: list[tuple[float, float]],
        power_pct: Optional[float] = None,
        freq_khz: Optional[float] = None,
        jump_speed_mm_s: Optional[float] = None,
        mark_speed_mm_s: Optional[float] = None,
    ) -> None:
        """
        Mark a continuous multi-segment path (e.g. all 4 sides of a square)
        with the laser staying on through corners, arming power/frequency
        once for the whole path. Blocking.

        IMPORTANT: calling mark_xy() once per segment in a loop sends a
        separate arm+batch sequence (02 12 prep frame + param block) for
        EACH segment — which re-triggers the laser's warm-up/fire cycle at
        every corner, visibly starting and stopping instead of staying on
        continuously like LightBurn's own output. Use mark_path() with the
        full list of corners instead for a single continuous burn.

        Confirmed-working sequence (reproduced from a real captured burn):
          aa10 (activate) -> status poll -> 0x02 12 prep frame -> aa10
          -> status poll -> ONE combined EP_POS write:
          [lead goto record][param block][chained mark records interpolating
          every segment of the path]. If the whole path doesn't fit in one
          frame, it's split at record boundaries into continuation writes
          (no prep frame or param block resent) — matching how LightBurn
          itself splits a long batch across multiple USB writes.
        """
        dev = self._require()
        cfg = self.config

        if not self._goto_activated:
            dev.write(EP_CMD, _ACTIVATE_GOTO, timeout=TIMEOUT_MS)
            dev.read(EP_RESP, 20, timeout=TIMEOUT_MS)
            self._goto_activated = True

        dev.write(EP_CMD, self._STATUS_CMD, timeout=TIMEOUT_MS)
        dev.read(EP_RESP, 40, timeout=TIMEOUT_MS)

        power_pct = cfg.power_pct if power_pct is None else power_pct
        freq_khz = cfg.freq_khz if freq_khz is None else freq_khz
        jump_speed_mm_s = self._jump_speed if jump_speed_mm_s is None else jump_speed_mm_s
        mark_speed_mm_s = self._mark_speed if mark_speed_mm_s is None else mark_speed_mm_s

        # 1) 02 12 prep frame
        self._seq += 1
        prep_frame = (
            b"\xfe\xff" + struct.pack(">H", 6 + len(self._PREP_BODY))
            + bytes([self._seq & 0xFF, 0x00])
            + self._PREP_BODY
        )
        dev.write(EP_POS, prep_frame, timeout=TIMEOUT_MS)
        try:
            dev.read(EP_ACK, 20, timeout=50)
        except usb.core.USBError:
            pass

        # 2) aa10 (re-activate) + status poll, matching captured sequence
        dev.write(EP_CMD, _ACTIVATE_GOTO, timeout=TIMEOUT_MS)
        dev.read(EP_RESP, 20, timeout=TIMEOUT_MS)
        dev.write(EP_CMD, self._STATUS_CMD, timeout=TIMEOUT_MS)
        dev.read(EP_RESP, 40, timeout=TIMEOUT_MS)

        # 3) build chained mark records for every segment of the path.
        # A real captured burn always had ~20+ chained mark records forming
        # a continuous path, never a single jump-to-target record — a
        # single record apparently completes too fast for the fiber laser
        # to actually turn on (LightBurn's "Laser_OpenMODelay" warm-up
        # setting suggests the MO needs sustained "on" time first).
        cur_x_mm = (self._pos_x_galvo - GALVO_CENTER) / self._galvos_per_mm_x
        cur_y_mm = (self._pos_y_galvo - GALVO_CENTER) / self._galvos_per_mm_y

        mark_records = b""
        for tx, ty in points:
            dist_mm = math.hypot(tx - cur_x_mm, ty - cur_y_mm)
            n_steps = max(1, round(dist_mm / self._interp_step_mm))
            for i in range(1, n_steps + 1):
                t = i / n_steps
                step_x_g = self._mm_to_galvo_x(cur_x_mm + (tx - cur_x_mm) * t)
                step_y_g = self._mm_to_galvo_y(cur_y_mm + (ty - cur_y_mm) * t)
                mark_records += self._build_compact_record(
                    b"\x02\x45", int(mark_speed_mm_s), 0, step_x_g, step_y_g,
                )
            cur_x_mm, cur_y_mm = tx, ty

        final_x_g = self._mm_to_galvo_x(points[-1][0])
        final_y_g = self._mm_to_galvo_y(points[-1][1])

        # 4) first write: lead goto record + param block + as many mark
        # records as fit under _MAX_FRAME_BODY (aligned to 18-byte records)
        lead_record = self._build_compact_record(
            b"\x02\x41", 1, int(jump_speed_mm_s),
            self._pos_x_galvo, self._pos_y_galvo,
        )
        param_block = self._build_param_block(jump_speed_mm_s, power_pct, freq_khz)

        first_capacity = self._MAX_FRAME_BODY - len(lead_record) - len(param_block)
        first_capacity -= first_capacity % 18
        first_records, remaining = mark_records[:first_capacity], mark_records[first_capacity:]

        self._send_pos_frame(lead_record + param_block + first_records)

        # 5) continuation writes: just more mark records, no prep/param resend
        chunk_capacity = self._MAX_FRAME_BODY - (self._MAX_FRAME_BODY % 18)
        while remaining:
            chunk, remaining = remaining[:chunk_capacity], remaining[chunk_capacity:]
            self._send_pos_frame(chunk)

        self._pos_x_galvo = final_x_g
        self._pos_y_galvo = final_y_g

    def _send_pos_frame(self, body: bytes) -> None:
        dev = self._require()
        self._seq += 1
        frame = (
            b"\xfe\xff" + struct.pack(">H", 6 + len(body))
            + bytes([self._seq & 0xFF, 0x00])
            + body
        )
        dev.write(EP_POS, frame, timeout=TIMEOUT_MS)
        try:
            dev.read(EP_ACK, 20, timeout=50)
        except usb.core.USBError:
            pass

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

    def _mm_to_galvo_x(self, mm: float) -> int:
        val = GALVO_CENTER + round(mm * self._galvos_per_mm_x)
        return max(0, min(0xFFFF, val))

    def _mm_to_galvo_y(self, mm: float) -> int:
        val = GALVO_CENTER + round(mm * self._galvos_per_mm_y)
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
        """
        Build a 34-byte mark frame for EP 0x02 (laser on).

        Confirmed via USB capture of licensed LightBurn: the mark command byte
        is 0x45 (bit 0x04 set relative to goto's 0x41), NOT 0x43. LightBurn
        itself batches multiple mark moves into a larger multi-record frame
        (18 bytes/record) rather than this single-move 34-byte layout; this
        single-frame version is a first test of whether the firmware accepts
        the same per-move layout goto uses, just with the mark opcode.
        """
        return (
            b"\xfe\xff\x00\x22"
            + bytes([seq & 0xFF, 0x00])
            + b"\x02\x45"                    # command: MARK_XY (laser on, confirmed via capture)
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

    # Template for the 46-byte laser-arming parameter block, captured
    # byte-for-byte from a licensed LightBurn session (40% power, 20kHz,
    # 5000mm/s jump speed). Confirmed-variable fields are overwritten by
    # _build_param_block(); every other byte's meaning is unknown but was
    # constant across two independent real captures with identical settings,
    # so it's treated as a fixed protocol constant. See project memory:
    # sea-laser-protocol for the full offset derivation.
    _PARAM_BLOCK_TEMPLATE = bytes.fromhex(
        "0210080001001388020808000000006402140a00"
        "66660100001402130c0009600009600000000a00"
        "08000003 0d40".replace(" ", "")
    )

    @staticmethod
    def _build_param_block(jump_speed_mm_s: float, power_pct: float, freq_khz: float) -> bytes:
        """
        Build the 46-byte laser-arming parameter block sent on EP_POS.

        Confirmed offsets (verified against two independent real captures,
        both matching exactly for freq=20kHz→2400 ticks and power=40%→102):
          local[6:8]   (BE u16): jump speed, mm/s
          local[20:22] (2x u8):  power byte = round(255 * power_pct/100), duplicated
          local[30:32] and [33:35] (BE u16, duplicated): period_ticks = 48_000_000 / freq_hz
        """
        block = bytearray(SEALaserUSB._PARAM_BLOCK_TEMPLATE)
        assert len(block) == 46, f"param block template is {len(block)} bytes, expected 46"

        block[6:8] = struct.pack(">H", min(0xFFFF, int(jump_speed_mm_s)))

        power_byte = max(0, min(0xFF, round(255 * power_pct / 100)))
        block[20] = power_byte
        block[21] = power_byte

        freq_hz = max(1, freq_khz * 1000.0)
        period_ticks = max(0, min(0xFFFF, round(48_000_000 / freq_hz)))
        block[30:32] = struct.pack(">H", period_ticks)
        block[33:35] = struct.pack(">H", period_ticks)

        return bytes(block)

    @staticmethod
    def _build_compact_record(cmd: bytes, field_a: int, field_b: int, x_g: int, y_g: int) -> bytes:
        """
        Build an 18-byte compact goto/mark record for use inside a batched
        EP_POS frame (as opposed to the standalone 34-byte single-move
        frames). Layout confirmed via capture:
          cmd(2) 1200(2) fieldA(2,BE) fieldB(2,BE) pad(1) X(2,BE) pad(1) Y(2,BE) trailer(4)
        fieldA/fieldB are speed-like for goto (vspeed, mspeed); for mark
        records fieldA is speed-like and fieldB's meaning is unconfirmed
        (usually 0). Not yet used by any public method — kept for a future
        full-batch marking implementation if the simpler configure_laser()
        + mark_xy() sequence proves insufficient for complex paths.
        """
        return (
            cmd
            + b"\x12\x00"
            + struct.pack(">H", field_a & 0xFFFF)
            + struct.pack(">H", field_b & 0xFFFF)
            + b"\x00"
            + struct.pack(">H", x_g)
            + b"\x00"
            + struct.pack(">H", y_g)
            + b"\x00\x00\x00\x00"
        )
