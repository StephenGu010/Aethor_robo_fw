"""Offline checks for the statically reconstructed DMTool transport envelope."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from analyze_6620_package_format import crc8, extract_frame, frame_payload


def reference_crc(payload):
    """Independent MSB-first polynomial 0x31 model with reflected input/output."""
    state = 0
    for value in payload:
        state ^= int(f"{value:08b}"[::-1], 2)
        for _ in range(8):
            state = ((state << 1) ^ (0x31 if state & 0x80 else 0)) & 0xFF
    return int(f"{state:08b}"[::-1], 2)


class PackageFormatTests(unittest.TestCase):
    """Validate CRC, framing boundaries, and corruption rejection without devices."""

    def test_crc_reference(self):
        """Cross-check independent bit orientation and the standard check vector."""
        self.assertEqual(crc8(b"123456789"), 0xA1)
        for payload in (b"", bytes(range(256)), bytes(range(256)) * 32):
            self.assertEqual(crc8(payload), reference_crc(payload))

    def test_boundary_roundtrip(self):
        """Exercise the partial final block and maximum block length."""
        for length in (1, 255, 256, 4196, 8192):
            payload = bytes(index % 256 for index in range(length))
            self.assertEqual(extract_frame(frame_payload(payload, 7)), payload)

    def test_corrupt_payload(self):
        """Reject a single flipped data bit in every bit position of a byte."""
        for bit in range(8):
            damaged = bytearray(frame_payload(b"0123456789", 0))
            damaged[7] ^= 1 << bit
            with self.assertRaises(ValueError):
                extract_frame(damaged)

    def test_invalid_lengths(self):
        """Reject empty/oversized blocks and truncated/misdeclared envelopes."""
        for payload in (b"", bytes(8193)):
            with self.assertRaises(ValueError):
                frame_payload(payload, 0)
        frame = frame_payload(b"123", 0)
        for damaged in (frame[:-1], frame[:3] + b"\x04\x00" + frame[5:]):
            with self.assertRaises(ValueError):
                extract_frame(damaged)

    def test_crc_excludes_remaining_count(self):
        """Document observed CRC scope: changing remaining-count keeps payload CRC."""
        first, last = frame_payload(b"123", 7), frame_payload(b"123", 0)
        self.assertEqual(first[-1], last[-1])
        self.assertNotEqual(first[1], last[1])


if __name__ == "__main__":
    unittest.main()
