"""Validate raw public ID7 drive configuration on the host without hardware access.

This module is only an independent data gate. It neither accepts a firmware success
flag as proof nor sends commands, claims Flash persistence, or opens serial/probes.
Transport provenance, image binding, once tokens and cold-start evidence belong to
the explicit maintenance client and remain mandatory before any real Save.
"""
from __future__ import annotations
import math
import struct
from collections.abc import Mapping, Sequence
from typing import Any

PUBLIC_REGISTERS = tuple(range(0x25)) + tuple(range(0x32, 0x38)) + (0x50, 0x51)
INTEGER_REGISTERS = frozenset((7, 8, 9, 10, 13, 14, 15, 16, 35, 36))
WRITABLE_REGISTERS = tuple(range(11)) + tuple(range(21, 36))
TARGET_WORDS = ((10, 2), (21, 0x44000000))
TARGET_REGISTERS = frozenset((10, 21))


def _float32(raw: int) -> float:
    """Decode native bits while keeping raw uint32 comparison independent of float formatting."""
    return struct.unpack("<f", struct.pack("<I", raw))[0]


def validate_snapshot(records: Sequence[Mapping[str, Any]], *, require_target: bool = False) -> dict[int, int]:
    """Require every public RID exactly once with its declared type and fixed motor identity.

    Float fields, including dynamic positions, must be finite. OV is not assigned an
    invented range. This establishes data consistency, not fresh transmission or power state.
    """
    if not isinstance(records, (list, tuple)) or len(records) != len(PUBLIC_REGISTERS):
        raise ValueError("expected exactly 45 public register records")
    words: dict[int, int] = {}
    for record in records:
        if not isinstance(record, Mapping):
            raise ValueError("register record must be an object")
        rid, raw = record.get("rid"), record.get("raw")
        if type(rid) is not int or rid not in PUBLIC_REGISTERS or rid in words:
            raise ValueError("unknown, repeated or invalid register address")
        if type(raw) is not int or not 0 <= raw <= 0xFFFFFFFF:
            raise ValueError(f"RID {rid:02x}: raw value must be an exact uint32")
        expected_kind = "uint32" if rid in INTEGER_REGISTERS else "float32"
        if record.get("kind") != expected_kind:
            raise ValueError(f"RID {rid:02x}: wrong native representation")
        if expected_kind == "float32" and not math.isfinite(_float32(raw)):
            raise ValueError(f"RID {rid:02x}: nonfinite float")
        words[rid] = raw
    if set(words) != set(PUBLIC_REGISTERS):
        raise ValueError("incomplete public register surface")
    expected_identity = {7: 23, 8: 7, 14: 808597046, 36: 49}
    if any(words[rid] != expected for rid, expected in expected_identity.items()):
        raise ValueError("expected ID7/MST23 drive firmware 6620_1")
    if _float32(words[22]) != 200.0 or _float32(words[23]) != 10.0:
        raise ValueError("unexpected VMAX/TMAX baseline")
    if require_target:
        if any(words[rid] != expected for rid, expected in TARGET_WORDS):
            raise ValueError("expected PMAX512 and default POS mode2")
    elif words[10] not in (1, 2) or _float32(words[21]) not in (12.5, 512.0):
        raise ValueError("unsupported source mode or PMAX")
    return words


def compare_configuration(baseline: Sequence[Mapping[str, Any]], current: Sequence[Mapping[str, Any]], *, allow_targets: bool) -> tuple[int, ...]:
    """Reject any collateral RW change, allowing only exact authorized target words.

    Dynamic p_m/xout and read-only measurements are not configuration equality.
    Returning changed RIDs never grants Save authorization or proves a cold boot.
    """
    original = validate_snapshot(baseline)
    observed = validate_snapshot(current, require_target=allow_targets)
    changed = tuple(rid for rid in WRITABLE_REGISTERS if original[rid] != observed[rid])
    forbidden = tuple(rid for rid in changed if not allow_targets or rid not in TARGET_REGISTERS)
    if forbidden:
        raise ValueError("unauthorized raw configuration changes: " + ",".join(f"{rid:02x}" for rid in forbidden))
    return changed


def _observation_unsigned(value: Any, *, bits: int, nonzero: bool, label: str) -> int:
    """Require an exact unsigned export integer without bool, float or string coercion."""
    if type(value) is not int or not (1 if nonzero else 0) <= value <= (1 << bits) - 1:
        qualifier = "positive " if nonzero else ""
        raise ValueError(f"{label} must be an exact {qualifier}uint{bits}")
    return value


def validate_observation_records(records: Sequence[Mapping[str, Any]], *, expected_owner: Mapping[str, Any],
                                 expected_phase: int, now_us: int, require_target: bool = False) -> dict[int, int]:
    """Validate a complete synthetic/exported read set against fixed item evidence bounds.

    Raw validation runs first and remains unchanged. The four permitted read phases
    and six observation times mirror MotorPersistenceItem; the 100000 us deadline
    is exclusive and never renewed here. TX-completion observation can follow RX,
    and quantized RX lower bounds may equal submission. Empty FIFO and actual
    pending/TXBTO evidence remain firmware responsibilities: this data gate is not
    a physical probe, SOF or motor sample timestamp, 24 V check, or Save permission.
    """
    words = validate_snapshot(records, require_target=require_target)
    owner_fields = ("boot_epoch", "request_id", "owner_generation")
    identity_fields = (*owner_fields, "phase", "sequence")
    time_fields = ("created_us", "submitted_us", "boundary_us", "send_completed_observed_us", "rx_lower_us", "received_us")
    if not isinstance(expected_owner, Mapping) or set(expected_owner) != set(owner_fields):
        raise ValueError("expected_owner must contain exactly boot_epoch, request_id and owner_generation")
    owner = {field: _observation_unsigned(expected_owner[field], bits=32, nonzero=True, label=f"expected_owner.{field}")
             for field in owner_fields}
    if type(expected_phase) is not int or expected_phase not in (1, 4, 6, 10):
        raise ValueError("expected_phase must be an exact read phase: 1, 4, 6 or 10")
    observed_now = _observation_unsigned(now_us, bits=64, nonzero=True, label="now_us")
    previous_sequence = 0
    previous_finished_us = 0
    for expected_rid, record in zip(PUBLIC_REGISTERS, records):
        if record["rid"] != expected_rid:
            raise ValueError("observation records must follow the published RID order")
        label = f"RID {expected_rid:02x}"
        if type(record.get("valid")) is not int or record["valid"] != 1:
            raise ValueError(f"{label}: valid must be the exact integer 1")
        identity = record.get("identity")
        if not isinstance(identity, Mapping) or set(identity) != set(identity_fields):
            raise ValueError(f"{label}: identity must contain exactly the five transaction fields")
        identity_words = {field: _observation_unsigned(identity[field], bits=32, nonzero=False, label=f"{label}.identity.{field}")
                          for field in identity_fields}
        if any(identity_words[field] != owner[field] for field in owner_fields) or identity_words["phase"] != expected_phase:
            raise ValueError(f"{label}: transaction owner or phase mismatch")
        sequence = identity_words["sequence"]
        if sequence <= previous_sequence:
            raise ValueError(f"{label}: sequence must be nonzero and strictly increasing")
        times = {field: _observation_unsigned(record.get(field), bits=64, nonzero=True, label=f"{label}.{field}")
                 for field in time_fields}
        created_us = times["created_us"]
        submitted_us = times["submitted_us"]
        completed_us = times["send_completed_observed_us"]
        received_us = times["received_us"]
        if not created_us <= times["boundary_us"] <= submitted_us <= completed_us <= observed_now:
            raise ValueError(f"{label}: invalid creation/boundary/submission/TX-observation order")
        if not submitted_us <= times["rx_lower_us"] <= received_us <= observed_now:
            raise ValueError(f"{label}: invalid submission/RX-observation order")
        if created_us > ((1 << 64) - 1) - 100000:
            raise ValueError(f"{label}: frame deadline would overflow uint64")
        frame_deadline_us = created_us + 100000
        if completed_us >= frame_deadline_us or received_us >= frame_deadline_us:
            raise ValueError(f"{label}: terminal observation reached the fixed frame deadline")
        if created_us < previous_finished_us:
            raise ValueError(f"{label}: next item was created before both previous observations finished")
        previous_sequence = sequence
        previous_finished_us = max(completed_us, received_us)
    return words
