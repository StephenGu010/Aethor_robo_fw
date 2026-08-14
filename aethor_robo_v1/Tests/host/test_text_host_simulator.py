"""Host tests for the deterministic aethor-text-v1 simulator."""

from __future__ import annotations

import json
import math
import pathlib
import sys
import unittest

PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "Tools"))

from aethor_text_simulator import AethorTextSimulator, encode_line  # noqa: E402
from aethor_reference_client import (  # noqa: E402
    AethorReferenceClient,
    build_one_shot_bench_move,
    parse_one_shot_move_terminal,
)


class ScriptedActionTransport:
    """Records one action write and returns a predefined ACK/DONE transcript."""

    def __init__(self, action_outputs: list[str]) -> None:
        """Stores the outputs that the one-shot action receive loop will observe."""
        self.action_outputs = action_outputs
        self.action_bodies: list[str] = []
        self.query_bodies: list[str] = []

    def transact(self, body: str) -> list[str]:
        """Records an ordinary query transaction for interface compatibility."""
        self.query_bodies.append(body)
        return []

    def transact_until_done(self, body: str, request_id: int) -> list[str]:
        """Records exactly one action write and returns its scripted lifecycle."""
        self.action_bodies.append(body)
        return list(self.action_outputs)


class AethorTextSimulatorTests(unittest.TestCase):
    """Exercises readable framing, replay, streaming, motion, and watchdog."""

    def setUp(self) -> None:
        """Creates one fresh bench-profile simulator."""
        self.simulator = AethorTextSimulator(boot_id=1234, profile="bench")

    def request(self, body: str) -> list[str]:
        """Sends one LF-terminated readable request."""
        return self.simulator.process_line(encode_line(body))

    def test_query_replay_and_conflict(self) -> None:
        """Returns readable identity and exact replay for normalized requests."""
        self.assertIn("protocol=aethor-text-v1", self.request("1 hello")[0])
        first = self.request("2 show state")
        self.assertEqual(first, self.request("2   SHOW   STATE"))
        self.assertEqual(self.request("2 ping"),
                         ["error 2 ping code=request_conflict"])

    def test_fragmented_plain_lines(self) -> None:
        """Accepts fragmented and concatenated LF-delimited plain text."""
        outputs = self.simulator.feed_bytes(b"1 hel")
        self.assertEqual(outputs, [])
        outputs = self.simulator.feed_bytes(b"lo\nshow state\r\nping\n")
        self.assertEqual(len(outputs), 3)
        self.assertTrue(outputs[0].startswith("ok 1 hello "))
        self.assertTrue(outputs[1].startswith("ok 0 show state "))
        self.assertTrue(outputs[2].startswith("ok 0 ping "))

    def test_bench_jog_done_and_watchdog(self) -> None:
        """Completes bounded jog motion and disables energized motors on timeout."""
        self.request("1 hello")
        self.request("2 bench init 1,3")
        self.simulator.drain_outputs()
        self.request("3 bench enable 1,3")
        self.simulator.drain_outputs()
        accepted = self.request("4 bench jog 1,3 delta=0.2 speed=1")
        self.assertEqual(accepted, ["ok 4 bench jog accepted=1"])
        self.simulator.advance(200)
        self.assertIn("done 4 bench jog result=completed elapsed_ms=200 arrived=05",
                      self.simulator.drain_outputs())
        self.simulator.advance(800)
        self.assertEqual(self.simulator.motor_enabled_mask, 0)
        self.assertTrue(any("link_timeout" in item
                            for item in self.simulator.drain_outputs()))

    def test_fixed_stream_format_and_rate(self) -> None:
        """Publishes fixed joint data without custom field negotiation."""
        self.request("1 hello")
        self.assertEqual(self.request("2 stream joints rate=20"),
                         ["ok 2 stream joints rate=20"])
        self.simulator.advance(1)
        outputs = self.simulator.drain_outputs()
        self.assertEqual(len(outputs), 1)
        self.assertTrue(outputs[0].startswith("data 501 joints "))
        self.assertIn(" q=0,0,0,0,0,0,0 ", outputs[0])

    def test_disabled_idle_does_not_timeout(self) -> None:
        """Leaves an idle, non-energized connection out of the watchdog scope."""
        self.request("1 hello")
        self.simulator.advance(10_000)
        self.assertEqual(self.simulator.arm_state, "unaligned")
        self.assertFalse(any("link_timeout" in item
                             for item in self.simulator.drain_outputs()))

    def test_shared_protocol_vectors(self) -> None:
        """Runs published line and fragmented-stream vectors unchanged."""
        vector_path = (PROJECT_ROOT / "Tests" / "protocol" /
                       "aethor-text-v1-vectors.json")
        vectors = json.loads(vector_path.read_text(encoding="utf-8"))

        for vector in vectors["lines"]:
            simulator = AethorTextSimulator(boot_id=1234, profile="bench")
            self.assertEqual(simulator.process_line(vector["request"]),
                             [vector["response"]], vector["name"])

        fragmented = vectors["byte_stream_cases"][0]
        simulator = AethorTextSimulator(boot_id=1234, profile="bench")
        outputs: list[str] = []
        for chunk in fragmented["chunks_ascii"]:
            outputs.extend(simulator.feed_bytes(chunk.encode("ascii")))
        self.assertEqual(outputs, fragmented["responses"])

        concatenated = vectors["byte_stream_cases"][1]
        simulator = AethorTextSimulator(boot_id=1234, profile="bench")
        outputs = simulator.feed_bytes(
            concatenated["chunks_ascii"][0].encode("ascii"))
        self.assertEqual(len(outputs), len(concatenated["response_prefixes"]))
        for output, prefix in zip(outputs, concatenated["response_prefixes"]):
            self.assertTrue(output.startswith(prefix))

    def test_one_shot_move_build_vectors(self) -> None:
        """Builds published single- and multi-motor one-to-one move requests."""
        vector_path = (PROJECT_ROOT / "Tests" / "protocol" /
                       "aethor-text-v1-vectors.json")
        vectors = json.loads(vector_path.read_text(encoding="utf-8"))

        for vector in vectors["bench_move_build_cases"]:
            self.assertEqual(
                build_one_shot_bench_move(
                    vector["request_id"],
                    vector["motors"],
                    vector["positions_deg"],
                    vector["speeds_deg_s"],
                ),
                vector["request"],
                vector["name"],
            )

        mismatch = vectors["bench_move_invalid_cases"][0]
        with self.assertRaisesRegex(ValueError, "equal non-empty lengths"):
            build_one_shot_bench_move(
                mismatch["request_id"],
                mismatch["motors"],
                mismatch["positions_deg"],
                mismatch["speeds_deg_s"],
            )
        self.assertEqual(
            mismatch["firmware_response"],
            "error 52 bench move code=count_mismatch field=position",
        )

    def test_one_shot_move_builder_rejects_unsafe_local_values(self) -> None:
        """Rejects duplicate IDs, invalid IDs, non-finite values, and nonpositive speed."""
        invalid_arguments = (
            ([1, 1], [1.0, 2.0], [1.0, 1.0]),
            ([0], [1.0], [1.0]),
            ([8], [1.0], [1.0]),
            ([1], [math.nan], [1.0]),
            ([1], [math.inf], [1.0]),
            ([1], [1.0], [math.inf]),
            ([1], [1.0], [0.0]),
            ([1], [1.0], [-1.0]),
        )
        for motors, positions, speeds in invalid_arguments:
            with self.subTest(motors=motors, positions=positions, speeds=speeds):
                with self.assertRaises(ValueError):
                    build_one_shot_bench_move(50, motors, positions, speeds)

    def test_one_shot_move_terminal_vectors(self) -> None:
        """Parses completed, failed, cancelled, and stopped terminal schemas."""
        vector_path = (PROJECT_ROOT / "Tests" / "protocol" /
                       "aethor-text-v1-vectors.json")
        vectors = json.loads(vector_path.read_text(encoding="utf-8"))

        for vector in vectors["bench_move_terminal_cases"]:
            result = parse_one_shot_move_terminal(
                vector["line"], vector["request_id"])
            self.assertEqual(result.result, vector["result"], vector["name"])
            self.assertEqual(result.elapsed_ms, vector.get("elapsed_ms"))
            self.assertEqual(result.motor_mask, vector.get("motor_mask"))
            self.assertEqual(result.stage, vector.get("stage"))
            self.assertEqual(result.code, vector.get("code"))
            self.assertEqual(result.motor, vector.get("motor"))

    def test_one_shot_move_sends_once_without_keepalive(self) -> None:
        """Writes one move line, waits for matching ACK/DONE, and emits no ping."""
        transport = ScriptedActionTransport([
            "ok 50 bench move accepted=1",
            "done 50 bench move result=failed stage=motion "
            "code=stale_feedback motor=3",
        ])
        client = AethorReferenceClient(transport)
        client.next_request_id = 50

        result = client.bench_move_once(
            [1, 3], [90.0, -45.0], [30.0, 20.0])

        self.assertEqual(
            transport.action_bodies,
            ["50 bench move 1,3 position=90,-45 speed=30,20"],
        )
        self.assertEqual(transport.query_bodies, [])
        self.assertFalse(any("ping" in body for body in transport.action_bodies))
        self.assertEqual(result.result, "failed")
        self.assertEqual(result.stage, "motion")
        self.assertEqual(result.code, "stale_feedback")
        self.assertEqual(result.motor, 3)

    def test_one_shot_move_requires_matching_ack_and_done(self) -> None:
        """Rejects missing ACKs and terminal lines for another request ID."""
        missing_ack = AethorReferenceClient(ScriptedActionTransport([
            "done 1 bench move result=cancelled",
        ]))
        with self.assertRaisesRegex(RuntimeError, "matching acceptance"):
            missing_ack.bench_move_once([1], [1.0], [1.0])

        wrong_done = AethorReferenceClient(ScriptedActionTransport([
            "ok 1 bench move accepted=1",
            "done 2 bench move result=completed elapsed_ms=1 motors=01",
        ]))
        with self.assertRaisesRegex(RuntimeError, "matching terminal"):
            wrong_done.bench_move_once([1], [1.0], [1.0])

    def test_published_replay_and_conflict_sequences_are_explicit(self) -> None:
        """Keeps exact replay and request-ID conflict outcomes in shared vectors."""
        vector_path = (PROJECT_ROOT / "Tests" / "protocol" /
                       "aethor-text-v1-vectors.json")
        vectors = json.loads(vector_path.read_text(encoding="utf-8"))
        replay, conflict = vectors["bench_move_request_sequences"]

        self.assertEqual(replay["requests"][0], replay["requests"][1])
        self.assertEqual(replay["responses"][0], replay["responses"][1])
        self.assertEqual(replay["responses"][0],
                         "ok 62 bench move accepted=1")
        self.assertNotEqual(conflict["requests"][0], conflict["requests"][1])
        self.assertEqual(conflict["responses"][1],
                         "error 62 bench move code=request_conflict")


if __name__ == "__main__":
    unittest.main()
