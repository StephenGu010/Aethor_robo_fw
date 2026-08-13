"""Host tests for the deterministic aethor-text-v1 simulator."""

from __future__ import annotations

import json
import pathlib
import sys
import unittest

PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "Tools"))

from aethor_text_simulator import AethorTextSimulator, encode_line  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
