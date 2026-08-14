"""File-level tests for the deterministic seven-motor protocol simulator."""

from __future__ import annotations

import json
import pathlib
import random
import sys
import unittest

PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from aethor_host_simulator import AethorHostSimulator, encode_frame  # noqa: E402


class AethorHostSimulatorTests(unittest.TestCase):
    """Exercises session, motion, telemetry, replay, and fault semantics."""

    def setUp(self) -> None:
        """Creates one fresh deterministic simulator for every test."""
        self.simulator = AethorHostSimulator(boot_id=1234)

    def request(self, body: str) -> list[str]:
        """Encodes and processes one host request body."""
        return self.simulator.process_line(encode_frame(body))

    def test_formal_lifecycle_and_replay(self) -> None:
        """Completes the formal seven-axis lifecycle with replay-safe MOVE."""
        hello = self.request("REQ 1 HELLO client=test protocol=1")
        self.assertIn("boot_id=1234", hello[0])
        self.assertIn("session=1", hello[0])
        self.assertIn("RSP 2 ok", self.request("REQ 2 GET_STATE")[0])
        self.assertIn("ACK 3 accepted", self.request(
            "REQ 3 ALIGN_REFERENCE q_ref_deg=0,0,0,0,0,0,0")[0])
        self.assertIn("DONE 3 COMPLETED", self.simulator.drain_outputs()[0])
        self.assertIn("ACK 4 accepted", self.request("REQ 4 SET_MODE mode=MIT")[0])
        self.assertIn("DONE 4 COMPLETED", self.simulator.drain_outputs()[0])
        self.assertIn("ACK 5 accepted", self.request("REQ 5 ENABLE")[0])
        self.assertIn("DONE 5 COMPLETED", self.simulator.drain_outputs()[0])

        move_body = (
            "REQ 6 MOVE_JOINTS q_deg=10,-5,2,0,1,0,-1 speed=0.5 mode=MIT"
        )
        accepted = self.request(move_body)
        replayed = self.request(move_body)
        self.assertEqual(accepted, replayed)
        self.assertIn("motion_id=6", accepted[0])
        self.simulator.advance(3000)
        outputs = self.simulator.drain_outputs()
        self.assertTrue(any("DONE 6 COMPLETED" in output for output in outputs))
        self.assertEqual(self.simulator.joint_position_deg,
                         [10.0, -5.0, 2.0, 0.0, 1.0, 0.0, -1.0])

    def test_fault_and_watchdog_disable_all_motors(self) -> None:
        """Latches injected faults and communication timeout as fail-safe disable."""
        self.request("REQ 1 HELLO client=test protocol=1")
        self.simulator.force_ready(mode="POS_VEL")
        self.simulator.inject_fault(joint=3, fault="overtemperature")
        self.assertEqual(self.simulator.arm_state, "FAULT")
        self.assertFalse(any(self.simulator.motor_enabled))
        self.assertTrue(any("DRIVER_FAULT" in item
                            for item in self.simulator.drain_outputs()))

        self.simulator.clear_injected_faults()
        self.simulator.force_ready(mode="POS_VEL")
        self.simulator.advance(1001)
        self.assertEqual(self.simulator.arm_state, "FAULT")
        self.assertFalse(any(self.simulator.motor_enabled))
        self.assertTrue(any("LINK_TIMEOUT" in item
                            for item in self.simulator.drain_outputs()))

    def test_fragmented_stream_and_bad_crc(self) -> None:
        """Accepts fragmented/concatenated lines and rejects a corrupt frame."""
        hello = encode_frame("REQ 1 HELLO client=test protocol=1")
        state = encode_frame("REQ 2 GET_STATE")
        outputs = []
        outputs.extend(self.simulator.feed_bytes(hello[:7]))
        self.assertEqual(outputs, [])
        outputs.extend(self.simulator.feed_bytes(hello[7:] + state))
        self.assertEqual(len(outputs), 2)
        bad = bytearray(encode_frame("REQ 3 GET_STATE"))
        bad[-3] = ord("0") if bad[-3] != ord("0") else ord("1")
        self.assertIn("ERR 3 BAD_CRC", self.simulator.feed_bytes(bytes(bad))[0])

    def test_session_telemetry_is_replaceable(self) -> None:
        """Generates bounded latest-value telemetry for the current session."""
        self.request("REQ 1 HELLO client=test protocol=1")
        self.request("REQ 2 SET_STREAM rate_hz=50 fields=jpos,jvel,state,motor")
        self.simulator.advance(100)
        outputs = self.simulator.drain_outputs()
        telemetry = [item for item in outputs if item.startswith("TEL ")]
        self.assertEqual(len(telemetry), 2)
        self.assertTrue(any("JOINT_STATE" in item and "q_deg=" in item
                            for item in telemetry))
        self.assertTrue(any("MOTOR_STATE" in item and "valid_mask=" in item
                            for item in telemetry))

    def test_replay_cache_is_bounded_and_expires(self) -> None:
        """Keeps replay state within 32 entries and expires it after 60 seconds."""
        self.request("REQ 1 HELLO client=test protocol=1")
        for request_id in range(2, 42):
            self.request(f"REQ {request_id} GET_STATE")
        self.assertLessEqual(len(self.simulator._replay), 32)
        self.simulator.advance(60_000)
        self.request("REQ 42 GET_STATE")
        self.assertLessEqual(len(self.simulator._replay), 2)

    def test_published_compatibility_vectors_have_valid_crc(self) -> None:
        """Validates every complete Golden Frame and byte-stream fixture CRC."""
        vector_path = (PROJECT_ROOT / "Tests" / "protocol" /
                       "aethor-arm-ascii-v1-compatibility-vectors.json")
        vectors = json.loads(vector_path.read_text(encoding="utf-8"))
        for vector in vectors["frames"]:
            line = vector["line"].encode("ascii")
            body = line.rstrip(b"\r\n").rsplit(b" *", 1)[0].decode("ascii")
            self.assertEqual(line, encode_frame(body), vector["type"])

        stream_simulator = AethorHostSimulator(boot_id=1234)
        fragmented_outputs: list[str] = []
        for chunk in vectors["byte_stream_cases"][0]["chunks_ascii"]:
            fragmented_outputs.extend(stream_simulator.feed_bytes(chunk.encode("ascii")))
        self.assertTrue(fragmented_outputs[0].startswith("RSP 1 ok"))

        concatenated_outputs: list[str] = []
        for chunk in vectors["byte_stream_cases"][1]["chunks_ascii"]:
            concatenated_outputs.extend(stream_simulator.feed_bytes(chunk.encode("ascii")))
        self.assertEqual(len(concatenated_outputs), 2)
        self.assertTrue(all(output.startswith("RSP ") for output in concatenated_outputs))

        bad_crc_outputs = stream_simulator.feed_bytes(
            vectors["byte_stream_cases"][2]["chunks_ascii"][0].encode("ascii"))
        self.assertEqual(bad_crc_outputs, ["ERR 42 BAD_CRC"])

    def test_fragmentation_bad_crc_and_reconnect_stress_is_bounded(self) -> None:
        """Feeds 2,000 variably fragmented requests across repeated sessions."""
        random_generator = random.Random(3519)
        simulator = AethorHostSimulator(boot_id=4321)
        bad_crc_count = 0
        response_count = 0

        for request_id in range(1, 2001):
            operation = (f"REQ {request_id} HELLO client=stress protocol=1"
                         if request_id % 200 == 1
                         else f"REQ {request_id} GET_STATE")
            frame = bytearray(encode_frame(operation))
            expect_bad_crc = request_id % 37 == 0
            if expect_bad_crc:
                frame[-3] = ord("0") if frame[-3] != ord("0") else ord("1")
            offset = 0
            outputs: list[str] = []
            while offset < len(frame):
                chunk_length = random_generator.randint(1, 17)
                outputs.extend(simulator.feed_bytes(
                    bytes(frame[offset:offset + chunk_length])))
                offset += chunk_length
            self.assertEqual(len(outputs), 1)
            if expect_bad_crc:
                self.assertIn("BAD_CRC", outputs[0])
                bad_crc_count += 1
            else:
                self.assertTrue(outputs[0].startswith("RSP "))
                response_count += 1

        self.assertEqual(bad_crc_count, 54)
        self.assertEqual(response_count, 1946)
        self.assertEqual(simulator.session_id, 10)
        self.assertEqual(len(simulator._rx_buffer), 0)
        self.assertLessEqual(len(simulator._replay), 32)


if __name__ == "__main__":
    unittest.main()
