"""
Unit tests for ConfigurableUSBConnection.

Tests cover USB ID storage and the _extract_payload() framing logic.
No USB hardware is opened.
"""

import struct

import pytest

from bjjcz.usb import (
    ConfigurableUSBConnection,
    _SEA_PAYLOAD_LENGTH,
    _SEA_PAYLOAD_OFFSET,
    _SEA_READ_ENDPOINT,
    _SEA_RESPONSE_SIZE,
)


class TestConfigurableUSBConnectionIDs:
    def _make(self, vid, pid, read_ep=None):
        if read_ep is not None:
            return ConfigurableUSBConnection(vendor_id=vid, product_id=pid, read_endpoint=read_ep)
        return ConfigurableUSBConnection(vendor_id=vid, product_id=pid)

    def test_sea_laser_ids_stored(self):
        conn = self._make(0x04B4, 0x1004)
        assert conn.vendor_id == 0x04B4
        assert conn.product_id == 0x1004

    def test_sea_laser_default_read_endpoint(self):
        conn = self._make(0x04B4, 0x1004)
        assert conn.read_endpoint == 0x84

    def test_bjjcz_ids_stored(self):
        conn = self._make(0x9588, 0x9899)
        assert conn.vendor_id == 0x9588
        assert conn.product_id == 0x9899

    def test_bjjcz_custom_read_endpoint(self):
        conn = self._make(0x9588, 0x9899, read_ep=0x88)
        assert conn.read_endpoint == 0x88

    def test_distinct_instances_are_independent(self):
        a = self._make(0x04B4, 0x1004)
        b = self._make(0x9588, 0x9899)
        assert a.vendor_id != b.vendor_id
        assert a.product_id != b.product_id


class TestExtractPayload:
    """Verify _extract_payload() handles all response framing cases."""

    def test_standard_bjjcz_8_bytes_passthrough(self):
        raw = struct.pack("<4H", 3, 0, 0, 0x20)  # typical BJJCZ GetVersion response
        result = ConfigurableUSBConnection._extract_payload(raw)
        assert result == raw
        assert len(result) == 8

    def test_sea_laser_20_byte_response_extracted(self):
        # Actual response captured from SEA-LASER hardware:
        # feff0014fff3ffff0000000000000000000000fe
        raw = bytes.fromhex("feff0014fff3ffff0000000000000000000000fe")
        result = ConfigurableUSBConnection._extract_payload(raw)
        assert len(result) == 8
        # payload at offset 4: fff3 ffff 0000 0000
        assert result == bytes.fromhex("fff3ffff00000000")

    def test_sea_laser_payload_unpacks_as_4_uint16(self):
        raw = bytes.fromhex("feff0014fff3ffff0000000000000000000000fe")
        result = ConfigurableUSBConnection._extract_payload(raw)
        vals = struct.unpack("<4H", result)
        assert len(vals) == 4

    def test_fallback_truncates_non_framed_long_response(self):
        # No 0xFE start marker — fallback: take first 8 bytes
        raw = bytes(range(20))  # 0x00, 0x01, 0x02 ... not framed
        result = ConfigurableUSBConnection._extract_payload(raw)
        assert len(result) == 8
        assert result == bytes(range(8))

    def test_fallback_pads_short_response(self):
        raw = b"\x01\x02\x03\x04"
        result = ConfigurableUSBConnection._extract_payload(raw)
        assert len(result) == 8
