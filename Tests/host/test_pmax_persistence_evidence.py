"""Independent host checks for fixed raw drive snapshots; no serial or probe imports."""
from __future__ import annotations
import importlib.util
from pathlib import Path
import struct
import unittest

REPOSITORY = Path(__file__).resolve().parents[2]
SPECIFICATION = importlib.util.spec_from_file_location("pmax_evidence", REPOSITORY / "tools/pmax_persistence_evidence.py")
assert SPECIFICATION and SPECIFICATION.loader
EVIDENCE = importlib.util.module_from_spec(SPECIFICATION)
SPECIFICATION.loader.exec_module(EVIDENCE)


def raw_float(value: float) -> int:
    """Encode synthetic native little-endian IEEE754 bits without decimal rounding."""
    return struct.unpack("<I", struct.pack("<f", value))[0]


def synthetic_words() -> list[dict[str, int | str]]:
    """Construct the published register surface with explicitly synthetic measurements."""
    registers = list(range(0x25)) + list(range(0x32, 0x38)) + [0x50, 0x51]
    integer_registers = {7, 8, 9, 10, 13, 14, 15, 16, 35, 36}
    words = [{"rid": rid, "kind": "uint32" if rid in integer_registers else "float32", "raw": 1 if rid in integer_registers else raw_float(1.0)} for rid in registers]
    for rid, raw in {7: 23, 8: 7, 10: 1, 14: 808597046, 36: 49, 21: raw_float(12.5), 22: raw_float(200.0), 23: raw_float(10.0)}.items():
        next(word for word in words if word["rid"] == rid)["raw"] = raw
    return words


class RawSnapshotGateTests(unittest.TestCase):
    """Reject unsafe collateral saves independently from firmware decision fields."""

    def test_target_and_raw_configuration_scope(self) -> None:
        """Allow only the two fixed target changes and leave dynamic words outside equality."""
        baseline = synthetic_words()
        target = synthetic_words()
        next(word for word in target if word["rid"] == 21)["raw"] = raw_float(512.0)
        next(word for word in target if word["rid"] == 10)["raw"] = 2
        self.assertEqual(EVIDENCE.compare_configuration(baseline, target, allow_targets=True), (10, 21))
        with self.assertRaises(ValueError):
            EVIDENCE.compare_configuration(baseline, target, allow_targets=False)
        for rid in [0x50, 0x51]:
            next(word for word in target if word["rid"] == rid)["raw"] = raw_float(-11.0)
        self.assertEqual(EVIDENCE.compare_configuration(baseline, target, allow_targets=True), (10, 21))

    def test_every_other_writable_bit_change_rejected(self) -> None:
        """Catch collateral protection, loop, identity, watchdog and communication changes."""
        for rid in list(range(11)) + list(range(21, 36)):
            if rid in (10, 21):
                continue
            with self.subTest(rid=rid):
                baseline, changed = synthetic_words(), synthetic_words()
                next(word for word in changed if word["rid"] == 21)["raw"] = raw_float(512.0)
                next(word for word in changed if word["rid"] == 10)["raw"] = 2
                next(word for word in changed if word["rid"] == rid)["raw"] ^= 1
                with self.assertRaises(ValueError):
                    EVIDENCE.compare_configuration(baseline, changed, allow_targets=True)
                # Removing only the unauthorized bit delta must make the exact same
                # target candidate valid; an absent target must not mask this case.
                next(word for word in changed if word["rid"] == rid)["raw"] ^= 1
                self.assertEqual(EVIDENCE.compare_configuration(baseline, changed, allow_targets=True), (10, 21))

    def test_surface_types_and_raw_width_are_exact(self) -> None:
        """Reject missing, repeated, unknown, wrongly typed or numerically coerced words."""
        cases = []
        words = synthetic_words(); words.pop(); cases.append(words)
        words = synthetic_words(); words[-1] = dict(words[0]); cases.append(words)
        words = synthetic_words(); words[-1]["rid"] = 0x52; cases.append(words)
        for value in [-1, 1 << 32, 1.0, True, "3f800000"]:
            words = synthetic_words(); words[0]["raw"] = value; cases.append(words)
        words = synthetic_words(); words[7]["kind"] = "float32"; cases.append(words)
        for words in cases:
            with self.subTest(words=words[-1]):
                with self.assertRaises(ValueError):
                    EVIDENCE.validate_snapshot(words)

    def test_finite_and_fixed_identity(self) -> None:
        """Reject NaN or infinity even in dynamic fields and reject a different motor family."""
        for rid in [0, 21, 29, 0x50, 0x51]:
            for raw in [0x7FC00000, 0x7F800000, 0xFF800000]:
                words = synthetic_words()
                next(word for word in words if word["rid"] == rid)["raw"] = raw
                with self.subTest(rid=rid, raw=raw), self.assertRaises(ValueError):
                    EVIDENCE.validate_snapshot(words)
        for rid, raw in {7: 24, 8: 6, 14: 0, 36: 0, 10: 3, 21: raw_float(1024.0), 22: raw_float(5.0), 23: raw_float(18.0)}.items():
            words = synthetic_words()
            next(word for word in words if word["rid"] == rid)["raw"] = raw
            with self.subTest(rid=rid), self.assertRaises(ValueError):
                EVIDENCE.validate_snapshot(words)

    def test_target_requirement_and_unknown_ov_range(self) -> None:
        """Require target words when requested without inventing an OV engineering range."""
        words = synthetic_words()
        self.assertEqual(len(EVIDENCE.validate_snapshot(words)), 45)
        with self.assertRaises(ValueError):
            EVIDENCE.validate_snapshot(words, require_target=True)
        words[21]["raw"] = raw_float(512.0); words[10]["raw"] = 2
        words[29]["raw"] = raw_float(-123.0)
        self.assertEqual(len(EVIDENCE.validate_snapshot(words, require_target=True)), 45)


if __name__ == "__main__":
    unittest.main()
