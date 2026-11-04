"""Offline synthetic checks of exported per-item evidence; no hardware or Save access.

The raw gate deliberately remains data-only. These tests require a separate strict
observation gate without treating timestamps as physical SOF or motor sample times.
"""
from __future__ import annotations

from copy import deepcopy
import importlib.util
from pathlib import Path
import struct
from typing import Any
import unittest

REPOSITORY = Path(__file__).resolve().parents[2]
SPECIFICATION = importlib.util.spec_from_file_location("pmax_provenance_evidence", REPOSITORY / "tools/pmax_persistence_evidence.py")
assert SPECIFICATION and SPECIFICATION.loader
EVIDENCE = importlib.util.module_from_spec(SPECIFICATION)
SPECIFICATION.loader.exec_module(EVIDENCE)

PUBLISHED_REGISTERS = tuple(range(0x25)) + tuple(range(0x32, 0x38)) + (0x50, 0x51)
INTEGER_REGISTERS = frozenset((7, 8, 9, 10, 13, 14, 15, 16, 35, 36))
EXPECTED_OWNER = {"boot_epoch": 11, "request_id": 22, "owner_generation": 33}
READ_PHASES = (1, 4, 6, 10)
TIME_FIELDS = ("created_us", "submitted_us", "boundary_us", "send_completed_observed_us", "rx_lower_us", "received_us")
UINT32_MAX = (1 << 32) - 1
UINT64_MAX = (1 << 64) - 1
NOW_US = 2_000_000


def raw_float(value: float) -> int:
    """Encode explicitly synthetic float32 measurements as raw little-endian bits."""
    return struct.unpack("<I", struct.pack("<f", value))[0]


def observation_records(*, phase: int = 1, target: bool = False, start_us: int = 1_000_000) -> list[dict[str, Any]]:
    """Expand the known default raw fixture with synthetic complete item evidence."""
    records = []
    raw_overrides = {7: 23, 8: 7, 10: 2 if target else 1, 14: 808597046, 36: 49,
                     21: raw_float(512.0 if target else 12.5), 22: raw_float(200.0), 23: raw_float(10.0)}
    for index, register_address in enumerate(PUBLISHED_REGISTERS):
        created_us = start_us + index * 1000
        records.append({
            "rid": register_address,
            "kind": "uint32" if register_address in INTEGER_REGISTERS else "float32",
            "raw": raw_overrides.get(register_address, 1 if register_address in INTEGER_REGISTERS else raw_float(1.0)),
            "valid": 1,
            "identity": {**EXPECTED_OWNER, "phase": phase, "sequence": 200 + index * 3},
            "created_us": created_us,
            "boundary_us": created_us + 100,
            "submitted_us": created_us + 200,
            "send_completed_observed_us": created_us + 300,
            "rx_lower_us": created_us + 250,
            "received_us": created_us + 400,
        })
    return records


class ObservationProvenanceGateTests(unittest.TestCase):
    """Check each invalid candidate after an independently accepted starting fixture."""

    def validate(self, records: list[dict[str, Any]], **overrides: Any) -> dict[int, int]:
        """Require the new callable explicitly, making a missing extension an honest RED."""
        validator = getattr(EVIDENCE, "validate_observation_records", None)
        self.assertTrue(callable(validator), "validate_observation_records is not implemented; this is an extension deficiency, not a hardware failure")
        arguments = {"expected_owner": EXPECTED_OWNER, "expected_phase": 1, "now_us": NOW_US}
        arguments.update(overrides)
        return validator(records, **arguments)

    def reject_record_field(self, records: list[dict[str, Any]], field: str, bad_value: Any, *, index: int = 20, **arguments: Any) -> None:
        """Reject a single-field mutation and prove that restoring only it is sufficient."""
        expected_words = self.validate(records, **arguments)
        candidate = deepcopy(records)
        candidate[index][field] = bad_value
        with self.assertRaises(ValueError):
            self.validate(candidate, **arguments)
        candidate[index][field] = deepcopy(records[index][field])
        self.assertEqual(self.validate(candidate, **arguments), expected_words)

    def reject_identity_field(self, records: list[dict[str, Any]], field: str, bad_value: Any, *, index: int = 20, **arguments: Any) -> None:
        """Keep all other item evidence valid while mutating one identity member."""
        expected_words = self.validate(records, **arguments)
        candidate = deepcopy(records)
        candidate[index]["identity"][field] = bad_value
        with self.assertRaises(ValueError):
            self.validate(candidate, **arguments)
        candidate[index]["identity"][field] = records[index]["identity"][field]
        self.assertEqual(self.validate(candidate, **arguments), expected_words)

    def test_valid_read_phases_and_sequence_gaps(self) -> None:
        """Accept all four published read phases and monotonic nonconsecutive sequences."""
        for phase in READ_PHASES:
            records = observation_records(phase=phase)
            with self.subTest(phase=phase):
                self.assertEqual(self.validate(records, expected_phase=phase), EVIDENCE.validate_snapshot(records))

    def test_deferred_tx_observation_can_follow_rx(self) -> None:
        """Accept later TX-completion polling without inventing physical TX ordering."""
        records = observation_records()
        for record in records:
            record["send_completed_observed_us"] = record["received_us"] + 100
        self.assertEqual(len(self.validate(records)), 45)

    def test_equal_timestamp_bounds_are_legal(self) -> None:
        """Preserve firmware's inclusive submission/RX bounds under 1 ms quantization."""
        records = observation_records()
        for record in records:
            for field in TIME_FIELDS:
                record[field] = record["created_us"]
        self.assertEqual(len(self.validate(records, now_us=records[-1]["created_us"])), 45)

    def test_owner_requires_exact_shape_and_nonzero_uint32(self) -> None:
        """Reject absent/extra owner keys, coercions and zero or overflowing identities."""
        records = observation_records()
        self.validate(records)
        bad_owners: list[Any] = [None, [], {}, {**EXPECTED_OWNER, "phase": 1}]
        for field in EXPECTED_OWNER:
            missing_owner = dict(EXPECTED_OWNER)
            missing_owner.pop(field)
            bad_owners.append(missing_owner)
            for value in (0, -1, 1 << 32, True, float(EXPECTED_OWNER[field]), str(EXPECTED_OWNER[field]), None):
                bad_owners.append({**EXPECTED_OWNER, field: value})
        for bad_owner in bad_owners:
            with self.subTest(owner=bad_owner), self.assertRaises(ValueError):
                self.validate(records, expected_owner=bad_owner)
            self.assertEqual(len(self.validate(records)), 45)

    def test_expected_phase_requires_a_read_phase_exact_int(self) -> None:
        """Reject write, waiting, terminal and coerced phase requests before evidence use."""
        records = observation_records()
        self.validate(records)
        for phase in (-1, 0, 2, 3, 5, 7, 8, 9, 11, 12, 256, True, 1.0, "1", None):
            with self.subTest(phase=phase), self.assertRaises(ValueError):
                self.validate(records, expected_phase=phase)
            self.assertEqual(len(self.validate(records)), 45)

    def test_now_requires_positive_exact_uint64(self) -> None:
        """Reject clock coercion or width errors and accept the exact uint64 upper limit."""
        records = observation_records()
        self.validate(records)
        for now_us in (-1, 0, 1 << 64, True, float(NOW_US), str(NOW_US), None):
            with self.subTest(now_us=now_us), self.assertRaises(ValueError):
                self.validate(records, now_us=now_us)
            self.assertEqual(len(self.validate(records)), 45)
        self.assertEqual(len(self.validate(records, now_us=UINT64_MAX)), 45)

    def test_identity_requires_complete_exact_shape(self) -> None:
        """Reject every omitted identity key and unexpected members without raw defects."""
        records = observation_records()
        for value in (None, [], {}, {**records[20]["identity"], "untrusted": 1}):
            with self.subTest(identity=value):
                self.reject_record_field(records, "identity", value)
        for field in records[20]["identity"]:
            identity = dict(records[20]["identity"])
            identity.pop(field)
            with self.subTest(missing=field):
                self.reject_record_field(records, "identity", identity)

    def test_identity_members_require_exact_uint32(self) -> None:
        """Reject negative, overflowing and coerced values for all five identity members."""
        records = observation_records()
        for field in records[20]["identity"]:
            for value in (-1, 1 << 32, True, float(records[20]["identity"][field]), str(records[20]["identity"][field]), None):
                with self.subTest(field=field, value=value):
                    self.reject_identity_field(records, field, value)

    def test_owner_and_phase_binding_are_exact(self) -> None:
        """Reject a different owner or phase even when all raw drive values are correct."""
        records = observation_records()
        for field in EXPECTED_OWNER:
            for value in (0, EXPECTED_OWNER[field] + 1):
                with self.subTest(field=field, value=value):
                    self.reject_identity_field(records, field, value)
        for phase in (0, 3, 4, 6, 10, 11, UINT32_MAX):
            with self.subTest(phase=phase):
                self.reject_identity_field(records, "phase", phase)

    def test_sequence_reuse_reverse_and_zero_are_rejected(self) -> None:
        """Require nonzero strictly increasing item sequence while permitting gaps."""
        records = observation_records()
        for sequence in (0, records[19]["identity"]["sequence"], records[19]["identity"]["sequence"] - 1,
                         records[21]["identity"]["sequence"] + 1):
            with self.subTest(sequence=sequence):
                self.reject_identity_field(records, "sequence", sequence)
        for index, record in enumerate(records):
            record["identity"]["sequence"] = UINT32_MAX - 44 + index
        self.assertEqual(len(self.validate(records)), 45)

    def test_rid_order_is_not_silently_sorted(self) -> None:
        """Swap only same-kind RID fields so no sequence or timestamp fault masks order."""
        records = observation_records()
        self.validate(records)
        records[1]["rid"], records[2]["rid"] = records[2]["rid"], records[1]["rid"]
        self.assertEqual(len(EVIDENCE.validate_snapshot(records)), 45)
        with self.assertRaises(ValueError):
            self.validate(records)
        records[1]["rid"], records[2]["rid"] = records[2]["rid"], records[1]["rid"]
        self.assertEqual(len(self.validate(records)), 45)

    def test_valid_flag_requires_exact_integer_one(self) -> None:
        """Treat a firmware success-like coercion as invalid per-item export data."""
        records = observation_records()
        for valid in (0, 2, -1, True, False, 1.0, "1", None):
            with self.subTest(valid=valid):
                self.reject_record_field(records, "valid", valid)

    def test_every_timestamp_requires_positive_exact_uint64(self) -> None:
        """Cover all six times independently, repairing one candidate field each time."""
        records = observation_records()
        for field in TIME_FIELDS:
            for value in (-1, 0, 1 << 64, True, float(records[20][field]), str(records[20][field]), None):
                with self.subTest(field=field, value=value):
                    self.reject_record_field(records, field, value)

    def test_boolean_equivalents_are_rejected_without_other_faults(self) -> None:
        """Use numeric-one controls so unrelated identity or time ordering cannot mask bool errors."""
        owner = {field: 1 for field in EXPECTED_OWNER}
        records = observation_records()
        for index, record in enumerate(records):
            record["identity"] = {**owner, "phase": 1, "sequence": index + 1}
            for field in TIME_FIELDS:
                record[field] = 1
        arguments = {"expected_owner": owner, "now_us": 1}
        self.validate(records, **arguments)
        for field in owner:
            with self.subTest(expected_owner=field), self.assertRaises(ValueError):
                self.validate(records, expected_owner={**owner, field: True}, now_us=1)
            self.validate(records, **arguments)
        for field in records[0]["identity"]:
            with self.subTest(identity=field):
                self.reject_identity_field(records, field, True, index=0, **arguments)
        for field in TIME_FIELDS:
            with self.subTest(time=field):
                self.reject_record_field(records, field, True, index=0, **arguments)
        for extra_arguments in ({"expected_phase": True}, {"now_us": True}):
            with self.subTest(arguments=extra_arguments), self.assertRaises(ValueError):
                self.validate(records, **{**arguments, **extra_arguments})
            self.validate(records, **arguments)

    def test_every_required_evidence_field_must_exist(self) -> None:
        """Reject omissions without relying on a false flag or another missing member."""
        records = observation_records()
        expected_words = self.validate(records)
        for field in ("valid", "identity", *TIME_FIELDS):
            candidate = deepcopy(records)
            candidate[20].pop(field)
            with self.subTest(missing=field), self.assertRaises(ValueError):
                self.validate(candidate)
            candidate[20][field] = deepcopy(records[20][field])
            self.assertEqual(self.validate(candidate), expected_words)

    def test_item_time_order_edges(self) -> None:
        """Exercise each causal inequality with an otherwise valid isolated mutation."""
        records = observation_records()
        last_record = records[-1]
        bad_times = {"created_us": last_record["boundary_us"] + 1,
                     "boundary_us": last_record["submitted_us"] + 1,
                     "send_completed_observed_us": last_record["submitted_us"] - 1,
                     "rx_lower_us": last_record["submitted_us"] - 1,
                     "received_us": last_record["rx_lower_us"] - 1}
        for field, value in bad_times.items():
            with self.subTest(field=field):
                self.reject_record_field(records, field, value, index=44)

    def test_future_terminal_observations_are_rejected_independently(self) -> None:
        """Check each independent terminal clock bound while keeping the frame budget valid."""
        records = observation_records()
        now_us = records[-1]["created_us"] + 500
        for field in ("received_us", "send_completed_observed_us"):
            with self.subTest(field=field):
                self.reject_record_field(records, field, now_us + 1, index=44, now_us=now_us)
        for record in records:
            for field in TIME_FIELDS:
                record[field] = record["created_us"]
        last_time = records[-1]["created_us"]
        self.validate(records, now_us=last_time)
        with self.assertRaises(ValueError):
            self.validate(records, now_us=last_time - 1)
        self.assertEqual(len(self.validate(records, now_us=last_time)), 45)

    def test_frame_deadline_is_exclusive_for_both_terminal_times(self) -> None:
        """Accept 99999 us and reject 100000/100001 us without extending any budget."""
        for field in ("received_us", "send_completed_observed_us"):
            records = observation_records()
            records[-1][field] = records[-1]["created_us"] + 99_999
            self.validate(records)
            for offset in (100_000, 100_001):
                with self.subTest(field=field, offset=offset):
                    self.reject_record_field(records, field, records[-1]["created_us"] + offset, index=44)

    def test_frame_deadline_cannot_overflow_uint64(self) -> None:
        """Accept a representable high-clock deadline and reject only created+budget overflow."""
        records = observation_records(start_us=UINT64_MAX - 100_000 - 44_000)
        self.validate(records, now_us=UINT64_MAX)
        self.reject_record_field(records, "created_us", records[-1]["created_us"] + 1, index=44, now_us=UINT64_MAX)

    def test_adjacent_records_wait_for_both_terminal_observations(self) -> None:
        """Prevent overlap using max(RX, TX observation) while permitting equal next creation."""
        for field in ("received_us", "send_completed_observed_us"):
            records = observation_records()
            records[20][field] = records[21]["created_us"]
            self.validate(records)
            with self.subTest(field=field):
                self.reject_record_field(records, field, records[21]["created_us"] + 1)

    def test_original_raw_gate_is_reused(self) -> None:
        """Preserve original surface, native type, finite, motor identity and target checks."""
        records = observation_records()
        for field, value in (("rid", 0x52), ("kind", "uint32"), ("raw", 0x7FC00000), ("raw", 0x7F800000),
                             ("raw", 0xFF800000), ("raw", True), ("raw", 1 << 32)):
            with self.subTest(field=field, value=value):
                self.reject_record_field(records, field, value, index=0)
        for register_address, raw in {7: 24, 8: 6, 14: 0, 36: 0, 10: 3, 21: raw_float(1024.0),
                                      22: raw_float(5.0), 23: raw_float(18.0)}.items():
            with self.subTest(rid=register_address):
                self.reject_record_field(records, "raw", raw, index=register_address)
        target_records = observation_records(target=True)
        for register_address, raw in ((10, 1), (21, raw_float(12.5))):
            with self.subTest(target_rid=register_address):
                self.reject_record_field(target_records, "raw", raw, index=register_address, require_target=True)

    def test_collateral_rw_gate_still_rejects_only_unauthorized_delta(self) -> None:
        """Apply provenance before the existing independent 26-RW raw comparison."""
        baseline = observation_records()
        for register_address in (*range(10), *range(22, 36)):
            target = observation_records(target=True)
            self.validate(target, require_target=True)
            target[register_address]["raw"] ^= 1
            with self.subTest(rid=register_address), self.assertRaises(ValueError):
                self.validate(target, require_target=True)
                EVIDENCE.compare_configuration(baseline, target, allow_targets=True)
            target[register_address]["raw"] ^= 1
            self.validate(target, require_target=True)
            self.assertEqual(EVIDENCE.compare_configuration(baseline, target, allow_targets=True), (10, 21))

    def test_legacy_raw_gate_keeps_its_data_only_boundary(self) -> None:
        """Show old raw acceptance is intentional scope, and the new gate catches bad timing."""
        records = observation_records()
        self.validate(records)
        records[20]["received_us"] = records[20]["submitted_us"] - 1
        self.assertEqual(len(EVIDENCE.validate_snapshot(records)), 45)
        with self.assertRaises(ValueError):
            self.validate(records)
        records[20]["received_us"] = observation_records()[20]["received_us"]
        self.assertEqual(len(self.validate(records)), 45)


if __name__ == "__main__":
    unittest.main()
