"""Offline tests for an irrevocable host Save claim; retain every filesystem artifact.

The fixtures are synthetic identity bindings, not evidence of a driver, USB power,
firmware deployment, or a physical Save. No serial, probe, or transport is opened.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import FrozenInstanceError
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

REPOSITORY = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY / "tools/pmax_persistence_once.py"
TOKEN_NAME = "id7_pmax512_pos2.once"
EVIDENCE_DIRECTORY = REPOSITORY / "output/repair_20261023/persistence_host_once"
ONCE = None
if MODULE_PATH.is_file():
    SPECIFICATION = importlib.util.spec_from_file_location("pmax_once", MODULE_PATH)
    assert SPECIFICATION and SPECIFICATION.loader
    ONCE = importlib.util.module_from_spec(SPECIFICATION)
    sys.modules[SPECIFICATION.name] = ONCE
    SPECIFICATION.loader.exec_module(ONCE)


def binding_values(**changes: object) -> dict[str, object]:
    """Return explicit synthetic, valid bindings with individual test overrides."""
    values: dict[str, object] = {
        "firmware_hex_sha256": "1" * 64,
        "source_manifest_sha256": "2" * 64,
        "cold_baseline_sha256": "3" * 64,
        "controller_usb_serial": "386B336F3134",
        "boot_epoch": 1,
        "request_id": 2,
        "owner_generation": 3,
        "cdc_link_state": 0x80000004,
    }
    values.update(changes)
    return values


def expected_token_bytes(values: dict[str, object]) -> bytes:
    """Specify the exact canonical JSON contract independently of helper constants."""
    document = {
        "schema_version": 1,
        "target": {
            "can_id": 7,
            "mst": 23,
            "firmware_main": 808597046,
            "firmware_sub": 49,
            "pmax_raw": 0x44000000,
            "mode": 2,
        },
        "binding": values,
    }
    return json.dumps(document, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")


class SaveOnceClaimTests(unittest.TestCase):
    """Exercise actual exclusive creation and durable-write failure boundaries."""

    @classmethod
    def setUpClass(cls) -> None:
        """Keep all run artifacts under the explicitly authorized output directory."""
        EVIDENCE_DIRECTORY.mkdir(parents=True, exist_ok=True)
        cls.run_directory = Path(tempfile.mkdtemp(prefix="test_run_", dir=EVIDENCE_DIRECTORY))
        print(f"Retained once-test artifacts: {cls.run_directory}")

    def setUp(self) -> None:
        """Fail explicitly during RED, then allocate a distinct retained test directory."""
        self.assertIsNotNone(ONCE, "persistent Save-once helper has not been implemented")
        self.directory = Path(tempfile.mkdtemp(prefix=self._testMethodName + "_", dir=self.run_directory))
        self.token_path = self.directory / TOKEN_NAME

    def make_binding(self, **changes: object) -> object:
        """Construct the immutable public binding from synthetic field values."""
        return ONCE.SaveOnceBinding(**binding_values(**changes))

    def assert_consumed(self, expected_bytes: bytes) -> None:
        """Require unchanged bytes and refusal of any second claim in that directory."""
        self.assertEqual(self.token_path.read_bytes(), expected_bytes)
        with self.assertRaises(OSError):
            ONCE.claim_save_once(self.directory, self.make_binding())
        self.assertEqual(self.token_path.read_bytes(), expected_bytes)
        self.assertEqual([entry.name for entry in self.directory.iterdir()], [TOKEN_NAME])

    def test_success_writes_exact_fixed_target_and_binding(self) -> None:
        """Return only after creating the fixed token with the complete canonical document."""
        result = ONCE.claim_save_once(self.directory, self.make_binding())
        self.assertEqual(result, self.token_path)
        self.assert_consumed(expected_token_bytes(binding_values()))

    def test_binding_is_frozen_and_has_no_mutable_attribute_dictionary(self) -> None:
        """Prevent ordinary assignment and mutable-record substitution in a claim."""
        binding = self.make_binding()
        self.assertFalse(hasattr(binding, "__dict__"))
        with self.assertRaises(FrozenInstanceError):
            setattr(binding, "request_id", 4)
        for invalid_binding in (None, binding_values(), object()):
            with self.subTest(binding_type=type(invalid_binding).__name__), self.assertRaises(ValueError):
                ONCE.claim_save_once(self.directory, invalid_binding)
            self.assertFalse(self.token_path.exists())

    def test_all_binding_fields_reject_invalid_values_before_creation(self) -> None:
        """Reject hashes, serials, coerced integers and every prohibited uint32 boundary."""
        bad_hashes = ("", "0" * 64, "1" * 63, "1" * 65, "A" * 64, "g" * 64, None, True, 1, b"1" * 64)
        invalid_values = {
            "firmware_hex_sha256": bad_hashes,
            "source_manifest_sha256": bad_hashes,
            "cold_baseline_sha256": bad_hashes,
            "controller_usb_serial": ("", "386b336f3134", "386B336F3135", "386B336F3134 ", None, True, 386),
            "boot_epoch": (0, -1, 1 << 32, True, False, 1.0, "1", None),
            "request_id": (0, -1, 1 << 32, True, False, 1.0, "1", None),
            "owner_generation": (0, -1, 1 << 32, True, False, 1.0, "1", None),
            "cdc_link_state": (0, 1, 0x7FFFFFFF, 0x80000000, -1, 1 << 32, True, False, float(0x80000001), "2147483649", None),
        }
        for field_name, field_values in invalid_values.items():
            for invalid_value in field_values:
                values = binding_values(**{field_name: invalid_value})
                with self.subTest(field=field_name, value=invalid_value):
                    with self.assertRaises(ValueError):
                        ONCE.SaveOnceBinding(**values)
                    # Deliberately bypass dataclass initialization to prove that the
                    # claim itself validates again before creating a token.
                    forged_binding = object.__new__(ONCE.SaveOnceBinding)
                    for attribute_name, attribute_value in values.items():
                        object.__setattr__(forged_binding, attribute_name, attribute_value)
                    with self.assertRaises(ValueError):
                        ONCE.claim_save_once(self.directory, forged_binding)
                    self.assertFalse(self.token_path.exists())
        self.assertEqual(list(self.directory.iterdir()), [])

    def test_nonzero_uint32_and_configured_link_endpoints_are_accepted(self) -> None:
        """Accept exact minimum/maximum counters and both configured-link endpoints."""
        for index, counter in enumerate((1, 0xFFFFFFFF)):
            directory = self.directory / f"endpoint_{index}"
            directory.mkdir()
            link_state = 0x80000001 if counter == 1 else 0xFFFFFFFF
            values = binding_values(boot_epoch=counter, request_id=counter, owner_generation=counter, cdc_link_state=link_state)
            ONCE.claim_save_once(directory, ONCE.SaveOnceBinding(**values))
            self.assertEqual((directory / TOKEN_NAME).read_bytes(), expected_token_bytes(values))

    def test_existing_empty_partial_different_and_complete_tokens_are_never_read_as_approval(self) -> None:
        """Reject all pre-existing byte sequences without modifying or replacing them."""
        for index, existing_bytes in enumerate((b"", b'{"binding":', b"unrelated content", expected_token_bytes(binding_values()))):
            directory = self.directory / f"existing_{index}"
            directory.mkdir()
            token_path = directory / TOKEN_NAME
            token_path.write_bytes(existing_bytes)
            with self.subTest(existing=existing_bytes), self.assertRaises(OSError):
                ONCE.claim_save_once(directory, self.make_binding())
            self.assertEqual(token_path.read_bytes(), existing_bytes)
            self.assertEqual([entry.name for entry in directory.iterdir()], [TOKEN_NAME])

    def test_existing_token_directory_is_preserved(self) -> None:
        """Treat a same-named directory as consumed and retain its contents."""
        self.token_path.mkdir()
        marker_path = self.token_path / "retained.txt"
        marker_path.write_bytes(b"keep directory")
        with self.assertRaises(OSError):
            ONCE.claim_save_once(self.directory, self.make_binding())
        self.assertTrue(self.token_path.is_dir())
        self.assertEqual(marker_path.read_bytes(), b"keep directory")

    def test_missing_directory_and_file_directory_are_rejected_without_mkdir(self) -> None:
        """Require a previously provisioned directory and never create missing parents."""
        missing_directory = self.directory / "not_provisioned"
        with self.assertRaises(OSError):
            ONCE.claim_save_once(missing_directory, self.make_binding())
        self.assertFalse(missing_directory.exists())
        file_directory = self.directory / "a_file"
        file_directory.write_bytes(b"unchanged")
        with self.assertRaises(OSError):
            ONCE.claim_save_once(file_directory, self.make_binding())
        self.assertEqual(file_directory.read_bytes(), b"unchanged")
        self.assertFalse(self.token_path.exists())

    def test_symlink_token_is_rejected_without_following_or_replacing(self) -> None:
        """Leave an existing link and its target unchanged when symlinks are available."""
        target_path = self.directory / "retained_target.txt"
        target_path.write_bytes(b"link target unchanged")
        try:
            self.token_path.symlink_to(target_path)
        except OSError as error:
            if getattr(error, "winerror", None) == 1314:
                self.skipTest("Windows symlink privilege unavailable; no alternate creation attempted")
            raise
        with self.assertRaises(OSError):
            ONCE.claim_save_once(self.directory, self.make_binding())
        self.assertTrue(self.token_path.is_symlink())
        self.assertEqual(target_path.read_bytes(), b"link target unchanged")

    def test_symlink_directory_is_rejected_without_creating_in_target(self) -> None:
        """Reject a final directory alias rather than consuming a token through it."""
        actual_directory = self.directory / "actual_directory"
        actual_directory.mkdir()
        linked_directory = self.directory / "linked_directory"
        try:
            linked_directory.symlink_to(actual_directory, target_is_directory=True)
        except OSError as error:
            if getattr(error, "winerror", None) == 1314:
                self.skipTest("Windows symlink privilege unavailable; no alternate creation attempted")
            raise
        with self.assertRaises(OSError):
            ONCE.claim_save_once(linked_directory, self.make_binding())
        self.assertTrue(linked_directory.is_symlink())
        self.assertEqual(list(actual_directory.iterdir()), [])

    def test_changed_request_boot_image_and_other_bindings_cannot_bypass_token(self) -> None:
        """Keep one fixed filename across requests, boots, images and binding changes."""
        ONCE.claim_save_once(self.directory, self.make_binding())
        original_bytes = self.token_path.read_bytes()
        changes = (
            {"request_id": 99}, {"boot_epoch": 99}, {"firmware_hex_sha256": "4" * 64},
            {"source_manifest_sha256": "4" * 64}, {"cold_baseline_sha256": "4" * 64},
            {"owner_generation": 99}, {"cdc_link_state": 0x80000099},
        )
        for changed_fields in changes:
            with self.subTest(changes=changed_fields), self.assertRaises(OSError):
                ONCE.claim_save_once(self.directory, self.make_binding(**changed_fields))
            self.assertEqual(self.token_path.read_bytes(), original_bytes)
        self.assertEqual([entry.name for entry in self.directory.iterdir()], [TOKEN_NAME])

    def test_short_writes_loop_until_complete_before_returning(self) -> None:
        """Use real OS writes of seven bytes to require a complete-write loop."""
        real_write = os.write

        def short_write(descriptor: int, payload: bytes) -> int:
            """Inject a successful short write while preserving actual filesystem data."""
            return real_write(descriptor, payload[:7])

        with patch.object(ONCE.os, "write", side_effect=short_write):
            ONCE.claim_save_once(self.directory, self.make_binding())
        self.assert_consumed(expected_token_bytes(binding_values()))

    def test_mid_write_failure_leaves_partial_token_consumed(self) -> None:
        """Propagate a write failure after actual bytes and refuse any later claim."""
        real_write = os.write
        write_calls = 0

        def fail_after_prefix(descriptor: int, payload: bytes) -> int:
            """Write an eleven-byte prefix once, then inject an I/O exception."""
            nonlocal write_calls
            write_calls += 1
            if write_calls == 1:
                return real_write(descriptor, payload[:11])
            raise OSError("injected mid-write failure")

        with patch.object(ONCE.os, "write", side_effect=fail_after_prefix), self.assertRaises(OSError):
            ONCE.claim_save_once(self.directory, self.make_binding())
        self.assert_consumed(expected_token_bytes(binding_values())[:11])

    def test_zero_write_fails_with_empty_token_consumed(self) -> None:
        """Avoid a stalled loop and leave an exclusive empty claim permanently consumed."""
        with patch.object(ONCE.os, "write", return_value=0), self.assertRaises(OSError):
            ONCE.claim_save_once(self.directory, self.make_binding())
        self.assert_consumed(b"")

    def test_fsync_failure_retains_complete_token_and_propagates(self) -> None:
        """Refuse approval if synchronization fails even when all bytes can be read."""
        with patch.object(ONCE.os, "fsync", side_effect=OSError("injected fsync failure")), self.assertRaises(OSError):
            ONCE.claim_save_once(self.directory, self.make_binding())
        self.assert_consumed(expected_token_bytes(binding_values()))

    def test_close_failure_retains_complete_token_and_propagates(self) -> None:
        """Refuse approval when close reports failure after the real descriptor is closed."""
        real_close = os.close

        def close_then_fail(descriptor: int) -> None:
            """Close the actual test descriptor, then inject uncertain close completion."""
            real_close(descriptor)
            raise OSError("injected close failure")

        with patch.object(ONCE.os, "close", side_effect=close_then_fail), self.assertRaises(OSError):
            ONCE.claim_save_once(self.directory, self.make_binding())
        self.assert_consumed(expected_token_bytes(binding_values()))

    def test_real_concurrent_claims_have_exactly_one_success(self) -> None:
        """Contend distinct requests on real O_EXCL creation without mocking the OS."""
        contender_count = 16
        barrier = threading.Barrier(contender_count)

        def contend(request_id: int) -> tuple[int, Path | OSError]:
            """Release all contenders together and retain their real filesystem outcome."""
            binding = self.make_binding(request_id=request_id)
            barrier.wait(timeout=10)
            try:
                return request_id, ONCE.claim_save_once(self.directory, binding)
            except OSError as error:
                return request_id, error

        with ThreadPoolExecutor(max_workers=contender_count) as executor:
            outcomes = list(executor.map(contend, range(1, contender_count + 1)))
        successful_requests = [request_id for request_id, outcome in outcomes if isinstance(outcome, Path)]
        self.assertEqual(len(successful_requests), 1, outcomes)
        self.assertEqual(sum(isinstance(outcome, OSError) for _, outcome in outcomes), contender_count - 1)
        self.assert_consumed(expected_token_bytes(binding_values(request_id=successful_requests[0])))


if __name__ == "__main__":
    unittest.main()
