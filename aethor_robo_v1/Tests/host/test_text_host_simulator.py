"""Host tests for the deterministic aethor-text-v1 simulator."""

from __future__ import annotations

import json
import math
import pathlib
import sys
import unittest

PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "Tools"))

import aethor_reference_client as reference_client  # noqa: E402
import aethor_text_simulator as simulator_module  # noqa: E402
from aethor_text_simulator import AethorTextSimulator, encode_line  # noqa: E402
from aethor_reference_client import (  # noqa: E402
    AethorReferenceClient,
    SerialTransport,
    SimulatorTransport,
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

    def transact_until_done(self, body: str, request_id: int,
                            action_timeout: float) -> list[str]:
        """Records exactly one action write and returns its scripted lifecycle."""
        del request_id, action_timeout
        self.action_bodies.append(body)
        return list(self.action_outputs)


class RecordingSerialPort:
    """Records serial writes without requiring a physical COM port."""

    def __init__(self) -> None:
        """Initializes an empty write log."""
        self.write_calls: list[bytes] = []

    def write(self, payload: bytes) -> int:
        """Records one attempted serial write."""
        self.write_calls.append(payload)
        return len(payload)

    def flush(self) -> None:
        """Provides the serial flush interface without side effects."""

    def readline(self) -> bytes:
        """Returns no input because invalid timeout tests must not read."""
        return b""


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
        self.simulator.advance(201)
        self.assertIn("done 4 bench jog result=completed elapsed_ms=201 arrived=05",
                      self.simulator.drain_outputs())
        self.simulator.advance(799)
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

    def test_invalid_requests_do_not_refresh_energized_watchdog(self) -> None:
        """Times out from the last valid request despite each rejected request class."""
        invalid_requests = (
            "50 bench move 1,3 position=1 speed=1,1",
            "50 bench move 1 position=nan speed=1",
            "50 nonsense",
            "show\tstate",
        )
        for request_body in invalid_requests:
            with self.subTest(request=request_body):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                simulator.process_line(encode_line("1 hello"))
                simulator.process_line(encode_line("2 bench enable 1"))
                simulator.drain_outputs()
                simulator.advance(900)
                simulator.process_line(request_body + "\n")
                self.assertEqual(simulator.last_request_ms, 0)
                simulator.advance(100)
                self.assertTrue(any("link_timeout" in output
                                    for output in simulator.drain_outputs()))

        conflict_simulator = AethorTextSimulator(boot_id=1234, profile="bench")
        conflict_simulator.process_line(encode_line("1 hello"))
        conflict_simulator.process_line(encode_line("50 show state"))
        conflict_simulator.process_line(encode_line("2 bench enable 1"))
        conflict_simulator.drain_outputs()
        conflict_simulator.advance(900)
        self.assertEqual(
            conflict_simulator.process_line(encode_line("50 ping")),
            ["error 50 ping code=request_conflict"],
        )
        self.assertEqual(conflict_simulator.last_request_ms, 0)
        conflict_simulator.advance(100)
        self.assertTrue(any("link_timeout" in output
                            for output in conflict_simulator.drain_outputs()))

        busy_simulator = AethorTextSimulator(boot_id=1234, profile="bench")
        busy_simulator.process_line(encode_line("1 hello"))
        busy_simulator.process_line(encode_line(
            "50 bench move 1 position=90 speed=30"))
        busy_simulator.advance(900)
        self.assertEqual(
            busy_simulator.process_line(encode_line(
                "51 bench move 3 position=1 speed=1")),
            ["error 51 bench move code=busy"],
        )
        self.assertEqual(busy_simulator.last_request_ms, 0)

    def test_invalid_request_shapes_match_c_and_do_not_refresh_watchdog(
            self) -> None:
        """Rejects extra protocol tokens without extending energized lifetime."""
        invalid_cases = (
            ("50 stream bogus rate=1",
             "error 50 stream bogus code=unknown_command"),
            ("50 stream off extra",
             "error 50 stream off code=bad_argument"),
            ("50 stream joints extra rate=1",
             "error 50 stream joints code=bad_argument"),
            ("50 stream motors rate=1 extra=1",
             "error 50 stream motors code=bad_argument"),
            ("50 stream joints rate=+1",
             "error 50 stream joints code=bad_argument"),
            ("50 stream joints rate=4294967296",
             "error 50 stream joints code=bad_argument"),
            ("50 show state extra",
             "error 50 show state code=bad_argument"),
            ("50 show config 1 extra",
             "error 50 show config code=bad_argument"),
            ("50 bench init 1 unexpected=1",
             "error 50 bench init code=bad_argument"),
            ("50 bench enable 1 unexpected=1",
             "error 50 bench enable code=bad_argument"),
            ("50 bench stop 1 unexpected=1",
             "error 50 bench stop code=bad_argument"),
            ("50 bench disable 1 unexpected=1",
             "error 50 bench disable code=bad_argument"),
            ("50 bench clear 1 unexpected=1",
             "error 50 bench clear code=bad_argument"),
            ("50 show config +1",
             "error 50 show config code=bad_argument"),
            ("50 show config abc",
             "error 50 show config code=bad_argument"),
            ("50 show config 0",
             "error 50 show config code=out_of_range field=joint"),
            ("50 show config 8",
             "error 50 show config code=out_of_range field=joint"),
            ("50 show config 4294967296",
             "error 50 show config code=bad_argument"),
            ("50 stream",
             "error 50 stream code=unknown_command"),
            ("50 bench",
             "error 50 bench code=unknown_command"),
            ("50 show",
             "error 50 show code=unknown_command"),
            ("50 help SHOW",
             "error 50 help code=bad_argument"),
            ("50 help STREAM",
             "error 50 help code=bad_argument"),
            ("50 help ARM",
             "error 50 help code=bad_argument"),
            ("50 help BENCH",
             "error 50 help code=bad_argument"),
            ("50 help show extra",
             "error 50 help code=bad_argument"),
            ("50 help topic=show",
             "error 50 help code=bad_argument"),
            ("50 show diag CAN",
             "error 50 show diag code=bad_argument"),
            ("50 show diag MOTION",
             "error 50 show diag code=bad_argument"),
            ("50 show diag USB",
             "error 50 show diag code=bad_argument"),
            ("50 show diag RTOS",
             "error 50 show diag code=bad_argument"),
            ("50 show diag unknown",
             "error 50 show diag code=bad_argument"),
            ("50 show diag can extra",
             "error 50 show diag code=bad_argument"),
            ("50 show diag can unexpected=1",
             "error 50 show diag code=bad_argument"),
            ("50 show info extra",
             "error 50 show info code=bad_argument"),
            ("50 show joints extra",
             "error 50 show joints code=bad_argument"),
            ("50 show motors extra",
             "error 50 show motors code=bad_argument"),
            ("50 show motor 1 extra",
             "error 50 show motor code=bad_argument"),
        )
        for request_body, expected_error in invalid_cases:
            with self.subTest(request=request_body):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                simulator.process_line(encode_line("1 hello"))
                simulator.process_line(encode_line("2 bench enable 1"))
                simulator.drain_outputs()
                simulator.advance(900)

                self.assertEqual(
                    simulator.process_line(encode_line(request_body)),
                    [expected_error],
                )
                self.assertEqual(simulator.last_request_ms, 0)
                simulator.advance(100)
                self.assertTrue(any("link_timeout" in output
                                    for output in simulator.drain_outputs()))

    def test_c_command_shape_table_accepts_and_refreshes_watchdog(self) -> None:
        """Accepts every public lower-case help, show, stream, and bench shape."""
        valid_cases = (
            ("80 help", "ok 80 help "),
            ("80 help show", "ok 80 help show "),
            ("80 help stream", "ok 80 help stream "),
            ("80 help arm", "ok 80 help arm "),
            ("80 help bench", "ok 80 help bench "),
            ("80 show info", "ok 80 show info "),
            ("80 show state", "ok 80 show state "),
            ("80 show joints", "ok 80 show joints "),
            ("80 show motors", "ok 80 show motors "),
            ("80 show motor 1", "ok 80 show motor joint=1 "),
            ("80 show config", "ok 80 show config "),
            ("80 show config 1", "ok 80 show config "),
            ("80 show diag", "ok 80 show diag "),
            ("80 show diag can", "ok 80 show diag can "),
            ("80 show diag motion", "ok 80 show diag motion "),
            ("80 show diag usb", "ok 80 show diag usb "),
            ("80 show diag rtos", "ok 80 show diag rtos "),
            ("80 stream off", "ok 80 stream off"),
            ("80 stream joints rate=1", "ok 80 stream joints rate=1"),
            ("80 stream motors rate=1", "ok 80 stream motors rate=1"),
            ("80 bench init 1", "ok 80 bench init accepted=1"),
            ("80 bench enable 1", "ok 80 bench enable accepted=1"),
            ("80 bench jog 1 delta=1 speed=1",
             "ok 80 bench jog accepted=1"),
            ("80 bench move 1 position=1 speed=1",
             "ok 80 bench move accepted=1"),
            ("80 bench stop 1", "ok 80 bench stop accepted=1"),
            ("80 bench disable 1", "ok 80 bench disable accepted=1"),
            ("80 bench clear 1", "ok 80 bench clear accepted=1"),
        )
        for request_body, expected_prefix in valid_cases:
            with self.subTest(request=request_body):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                simulator.process_line(encode_line("1 hello"))
                simulator.process_line(encode_line("2 bench enable 1"))
                simulator.drain_outputs()
                simulator.advance(900)

                responses = simulator.process_line(encode_line(request_body))
                self.assertTrue(responses[0].startswith(expected_prefix), responses)
                self.assertEqual(simulator.last_request_ms, 900)

    def test_valid_ping_show_and_replay_refresh_energized_watchdog(self) -> None:
        """Refreshes watchdog time only for valid commands and exact replay."""
        keepalive_requests = (
            ("3 ping", None),
            ("3 show state", None),
            ("3 show config 1", None),
            ("3 show config 7", None),
            ("3 show config 01", None),
            ("3 help show", None),
            ("3 help stream", None),
            ("3 help arm", None),
            ("3 help bench", None),
            ("3 show diag can", None),
            ("3 show diag motion", None),
            ("3 show diag usb", None),
            ("3 show diag rtos", None),
            ("50 show state", "50 show state"),
        )
        for request_body, replay_seed in keepalive_requests:
            with self.subTest(request=request_body, replay=bool(replay_seed)):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                simulator.process_line(encode_line("1 hello"))
                if replay_seed is not None:
                    simulator.process_line(encode_line(replay_seed))
                simulator.process_line(encode_line("2 bench enable 1"))
                simulator.drain_outputs()
                simulator.advance(900)
                simulator.process_line(encode_line(request_body))
                self.assertEqual(simulator.last_request_ms, 900)
                simulator.advance(999)
                self.assertFalse(any("link_timeout" in output
                                     for output in simulator.drain_outputs()))
                simulator.advance(1)
                self.assertTrue(any("link_timeout" in output
                                    for output in simulator.drain_outputs()))

    def test_real_show_motor_matches_firmware_limits_and_client_discovers_first(
            self) -> None:
        """Queries simulated discovered limits before a one-shot client move."""
        response = self.request("1 show motor 1")[0]
        expected_response = (
            "ok 1 show motor joint=1 esc=01 master=11 state=disabled "
            "pos_deg=0 speed_deg_s=0 torque_nm=0 mos_c=0 rotor_c=0 "
            "fault=0 age_ms=0 pmax_deg=180 vmax_deg_s=360 "
            "max_speed_deg_s=360 move_speed_limit_deg_s=360")
        self.assertEqual(response, expected_response)
        self.assertLessEqual(len(response) + 1, 256)

        invalid_cases = (
            ("2 show motor", "error 2 show motor code=bad_argument"),
            ("3 show motor abc", "error 3 show motor code=bad_argument"),
            ("4 show motor +1", "error 4 show motor code=bad_argument"),
            ("5 show motor 0", "error 5 show motor code=out_of_range field=joint"),
            ("6 show motor 8", "error 6 show motor code=out_of_range field=joint"),
            ("7 show motor 1 extra", "error 7 show motor code=bad_argument"),
        )
        for request_body, expected_error in invalid_cases:
            with self.subTest(request=request_body):
                self.assertEqual(self.request(request_body), [expected_error])

        transport = SimulatorTransport()
        client = AethorReferenceClient(transport)
        limits = client.request("show motor 1")
        result = client.bench_move_once([1], [90.0], [30.0])
        self.assertIn("pmax_deg=180", limits[0])
        self.assertIn("move_speed_limit_deg_s=360", limits[0])
        self.assertEqual(result.result, "completed")
        self.assertEqual(client.transcript[0], "> 1 show motor 1")
        self.assertEqual(client.transcript[2],
                         "> 2 bench move 1 position=90 speed=30")

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

    def test_one_shot_move_builder_enforces_firmware_float32_range(self) -> None:
        """Accepts normal float32 boundaries and rejects unparseable numeric inputs."""
        float32_min = 1.1754943508222875e-38
        float32_max = 3.4028234663852886e38
        boundary_body = build_one_shot_bench_move(
            57, [1], [float32_min], [float32_max])
        numeric_text = boundary_body.split("position=", 1)[1]
        position_text, speed_text = numeric_text.split(" speed=", 1)
        self.assertNotIn("e", position_text.lower())
        self.assertNotIn("e", speed_text.lower())

        invalid_values = (1e-40, 1e39, 10 ** 10000, object())
        for case_index, invalid_value in enumerate(invalid_values):
            with self.subTest(case_index=case_index,
                              value_type=type(invalid_value).__name__):
                with self.assertRaises(ValueError):
                    build_one_shot_bench_move(
                        58, [1], [invalid_value], [1.0])
                with self.assertRaises(ValueError):
                    build_one_shot_bench_move(
                        59, [1], [0.0], [invalid_value])

    def test_one_shot_move_builder_uses_plain_decimal_and_normalizes_zero(self) -> None:
        """Emits tiny finite values without exponents and canonicalizes negative zero."""
        self.assertEqual(
            build_one_shot_bench_move(53, [1], [1e-10], [1e-10]),
            "53 bench move 1 position=0.0000000001 speed=0.0000000001",
        )
        self.assertEqual(
            build_one_shot_bench_move(54, [1], [-0.0], [1.0]),
            "54 bench move 1 position=0 speed=1",
        )

    def test_one_shot_move_builder_enforces_final_ascii_line_length(self) -> None:
        """Checks the complete multi-axis body against the firmware line limit."""
        valid_body = build_one_shot_bench_move(
            55, [1, 3, 7], [1e-10, -1e-10, 0.0], [1e-10, 1e-10, 1e-10])
        self.assertLessEqual(len(valid_body.encode("ascii")), 160)

        with self.assertRaisesRegex(ValueError, "160 ASCII bytes"):
            build_one_shot_bench_move(
                56,
                [1, 2, 3, 4, 5, 6, 7],
                [1e-10] * 7,
                [1e-10] * 7,
            )

    def test_one_shot_move_length_failure_writes_nothing(self) -> None:
        """Rejects an overlong action before invoking its serial transport."""
        transport = ScriptedActionTransport([])
        client = AethorReferenceClient(transport)

        float32_max = 3.4028234663852886e38
        with self.assertRaisesRegex(ValueError, "160 ASCII bytes"):
            client.bench_move_once(
                [1, 2, 3, 4, 5, 6, 7],
                [float32_max] * 7,
                [float32_max] * 7,
            )

        self.assertEqual(transport.action_bodies, [])
        self.assertEqual(transport.query_bodies, [])

    def test_one_shot_move_requires_nonzero_uint32_request_id(self) -> None:
        """Rejects sentinel, signed, and overflowing action request identifiers."""
        for invalid_request_id in (0, -1, 0x100000000, True):
            with self.subTest(request_id=invalid_request_id):
                with self.assertRaises(ValueError):
                    build_one_shot_bench_move(
                        invalid_request_id, [1], [0.0], [1.0])

        invalid_terminal_cases = (
            ("done 0 bench move result=cancelled", 0),
            ("done -1 bench move result=cancelled", 1),
            ("done +1 bench move result=cancelled", 1),
            ("done 4294967296 bench move result=cancelled", 1),
        )
        for terminal_line, expected_request_id in invalid_terminal_cases:
            with self.subTest(line=terminal_line):
                with self.assertRaises(ValueError):
                    parse_one_shot_move_terminal(
                        terminal_line, expected_request_id)

        self.assertEqual(
            self.request(
                "4294967296 bench move 1 position=0 speed=1"),
            ["error 0 parse code=bad_line"],
        )

    def test_one_shot_move_terminal_rejects_unknown_public_tokens(self) -> None:
        """Rejects failure stages and codes absent from the firmware formatter."""
        invalid_lines = (
            "done 50 bench move result=failed stage=other "
            "code=timeout motor=1",
            "done 50 bench move result=failed stage=motion "
            "code=other motor=1",
        )
        for invalid_line in invalid_lines:
            with self.subTest(line=invalid_line):
                with self.assertRaises(ValueError):
                    parse_one_shot_move_terminal(invalid_line, 50)

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

    def test_one_shot_move_rejects_invalid_timeout_before_transport(self) -> None:
        """Rejects nonpositive or nonfinite action timeouts before any write."""
        maximum_timeout_seconds = 0xFFFFFFFF / 1000.0
        invalid_timeouts = (
            0.0,
            -1.0,
            math.nan,
            math.inf,
            -math.inf,
            math.nextafter(maximum_timeout_seconds, math.inf),
            sys.float_info.max,
        )
        for invalid_timeout in invalid_timeouts:
            with self.subTest(timeout=invalid_timeout):
                scripted_transport = ScriptedActionTransport([])
                scripted_client = AethorReferenceClient(scripted_transport)
                with self.assertRaises(ValueError):
                    scripted_client.bench_move_once(
                        [1], [0.0], [1.0], action_timeout=invalid_timeout)
                self.assertEqual(scripted_transport.action_bodies, [])

                simulator_transport = SimulatorTransport()
                simulator_client = AethorReferenceClient(simulator_transport)
                with self.assertRaises(ValueError):
                    simulator_client.bench_move_once(
                        [1], [0.0], [1.0], action_timeout=invalid_timeout)
                self.assertEqual(simulator_client.transcript, [])
                self.assertIsNone(simulator_transport.simulator.active_motion)

                recording_serial = RecordingSerialPort()
                serial_transport = SerialTransport.__new__(SerialTransport)
                serial_transport.serial = recording_serial
                serial_transport.action_timeout = 120.0
                with self.assertRaises(ValueError):
                    serial_transport.transact_until_done(
                        "1 bench move 1 position=0 speed=1",
                        1,
                        invalid_timeout,
                    )
                self.assertEqual(recording_serial.write_calls, [])

    def test_one_shot_move_accepts_maximum_representable_timeout(self) -> None:
        """Accepts the largest timeout whose milliseconds fit the transport contract."""
        maximum_timeout = reference_client.AETHOR_ACTION_TIMEOUT_MAX_SECONDS
        self.assertLessEqual(int(maximum_timeout * 1000.0), 0xFFFFFFFF)
        transport = ScriptedActionTransport([
            "ok 1 bench move accepted=1",
            "done 1 bench move result=completed elapsed_ms=4 motors=01",
        ])
        result = AethorReferenceClient(transport).bench_move_once(
            [1], [0.0], [1.0], action_timeout=maximum_timeout)
        self.assertEqual(result.result, "completed")
        self.assertEqual(len(transport.action_bodies), 1)

    def test_one_shot_move_validates_lifecycle_order_and_selected_mask(self) -> None:
        """Rejects reordered, duplicated, or out-of-scope lifecycle output."""
        output_cases = (
            [
                "done 1 bench move result=completed elapsed_ms=1 motors=01",
                "ok 1 bench move accepted=1",
            ],
            [
                "ok 1 bench move accepted=1",
                "ok 1 bench move accepted=1",
                "done 1 bench move result=completed elapsed_ms=1 motors=01",
            ],
            [
                "ok 1 bench move accepted=1",
                "done 1 bench move result=completed elapsed_ms=1 motors=02",
            ],
            [
                "ok 1 bench move accepted=1",
                "done 1 bench move result=failed stage=motion "
                "code=stale_feedback motor=3",
            ],
        )
        for outputs in output_cases:
            with self.subTest(outputs=outputs):
                client = AethorReferenceClient(ScriptedActionTransport(outputs))
                with self.assertRaises(RuntimeError):
                    client.bench_move_once([1], [0.0], [1.0])

    def test_one_shot_move_round_trips_through_real_simulator(self) -> None:
        """Completes single- and multi-motor moves through the production simulator."""
        transport = SimulatorTransport()
        client = AethorReferenceClient(transport)

        single_result = client.bench_move_once([1], [90.0], [30.0])
        multi_result = client.bench_move_once(
            [3, 1], [-45.0, 15.0], [20.0, 10.0])

        self.assertEqual(single_result.result, "completed")
        self.assertEqual(single_result.motor_mask, 0x01)
        self.assertEqual(multi_result.result, "completed")
        self.assertEqual(multi_result.motor_mask, 0x05)
        self.assertEqual(transport.simulator.motor_enabled_mask, 0)
        self.assertEqual(transport.simulator.joint_position_deg[0], 15.0)
        self.assertEqual(transport.simulator.joint_position_deg[2], -45.0)
        self.assertFalse(any("ping" in line for line in client.transcript))

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
        """Runs published replay and request-ID conflicts through the real simulator."""
        vector_path = (PROJECT_ROOT / "Tests" / "protocol" /
                       "aethor-text-v1-vectors.json")
        vectors = json.loads(vector_path.read_text(encoding="utf-8"))

        for vector in vectors["bench_move_request_sequences"]:
            simulator = AethorTextSimulator(boot_id=1234, profile="bench")
            responses = [simulator.process_line(encode_line(request))[0]
                         for request in vector["requests"]]
            self.assertEqual(responses, vector["responses"], vector["name"])

    def test_published_move_errors_are_consumed_by_real_simulator(self) -> None:
        """Runs malformed shared move vectors through the production simulator."""
        vector_path = (PROJECT_ROOT / "Tests" / "protocol" /
                       "aethor-text-v1-vectors.json")
        vectors = json.loads(vector_path.read_text(encoding="utf-8"))

        for vector in vectors["bench_move_invalid_cases"]:
            simulator = AethorTextSimulator(boot_id=1234, profile="bench")
            self.assertEqual(
                simulator.process_line(encode_line(vector["firmware_request"])),
                [vector["firmware_response"]],
                vector["name"],
            )

    def test_real_simulator_reports_busy_during_active_one_shot_move(self) -> None:
        """Keeps an accepted simulated move active until deterministic completion."""
        accepted = self.request(
            "50 bench move 1 position=90 speed=30")
        busy = self.request(
            "51 bench move 3 position=-45 speed=20")

        self.assertEqual(accepted, ["ok 50 bench move accepted=1"])
        self.assertEqual(busy, ["error 51 bench move code=busy"])

    def test_real_simulator_move_numbers_match_strict_firmware_decimal_syntax(
            self) -> None:
        """Mirrors firmware decimal grammar and its 63-character token limit."""
        sixty_three_character_number = ("0" * 62) + "1"
        sixty_four_character_number = ("0" * 63) + "1"
        self.assertEqual(len(sixty_three_character_number), 63)
        self.assertEqual(len(sixty_four_character_number), 64)

        accepted_positions = (
            sixty_three_character_number,
            "+1",
            "-1",
            ".5",
            "5.",
            "+.5",
            "-.5",
        )
        for case_index, position_token in enumerate(accepted_positions, 70):
            with self.subTest(position=position_token):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                self.assertEqual(
                    simulator.process_line(encode_line(
                        f"{case_index} bench move 1 "
                        f"position={position_token} speed=1")),
                    [f"ok {case_index} bench move accepted=1"],
                )

        invalid_fields = (
            ("position", "1e-10"),
            ("speed", "1E-10"),
            ("position", sixty_four_character_number),
            ("speed", sixty_four_character_number),
            ("position", "."),
            ("position", "+."),
            ("position", "--1"),
        )
        for case_index, (field_name, numeric_token) in enumerate(
                invalid_fields, 80):
            with self.subTest(field=field_name, token=numeric_token):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                position_token = numeric_token if field_name == "position" else "0"
                speed_token = numeric_token if field_name == "speed" else "1"
                self.assertEqual(
                    simulator.process_line(encode_line(
                        f"{case_index} bench move 1 "
                        f"position={position_token} speed={speed_token}")),
                    [f"error {case_index} bench move "
                     f"code=bad_argument field={field_name}"],
                )

    def test_real_simulator_uses_float32_values_for_motion_and_limits(self) -> None:
        """Rounds parsed decimals to float32 before move and jog validation."""
        accepted_moves = (
            ("145 bench move 1 position=180.000001 speed=360.000001", 180.0),
            ("146 bench move 1 position=-180.000001 speed=360.000001", -180.0),
        )
        for request_body, expected_position in accepted_moves:
            with self.subTest(request=request_body):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                request_id = request_body.split()[0]
                self.assertEqual(
                    simulator.process_line(encode_line(request_body)),
                    [f"ok {request_id} bench move accepted=1"],
                )
                self.assertEqual(simulator.active_motion.target_deg[0],
                                 expected_position)
                simulator.advance(500)
                terminal = simulator.drain_outputs()[0]
                self.assertEqual(
                    simulator.process_line(encode_line(request_body)),
                    [terminal],
                )

        jog_simulator = AethorTextSimulator(boot_id=1234, profile="bench")
        self.assertEqual(
            jog_simulator.process_line(encode_line(
                "147 bench jog 1 delta=180.000001 speed=360.000001")),
            ["ok 147 bench jog accepted=1"],
        )
        self.assertEqual(jog_simulator.active_motion.target_deg[0], 180.0)

        rejected_moves = (
            ("148 bench move 1 position=180.00001 speed=360",
             "position_out_of_range"),
            ("149 bench move 1 position=-180.00001 speed=360",
             "position_out_of_range"),
            ("153 bench move 1 position=180 speed=360.00002",
             "speed_out_of_range"),
        )
        for request_body, expected_code in rejected_moves:
            with self.subTest(request=request_body):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                request_id = request_body.split()[0]
                expected_terminal = (f"done {request_id} bench move result=failed "
                                     f"stage=validate code={expected_code} motor=1")
                self.assertEqual(
                    simulator.process_line(encode_line(request_body)),
                    [f"ok {request_id} bench move accepted=1"],
                )
                self.assertEqual(simulator.drain_outputs(), [expected_terminal])
                self.assertEqual(
                    simulator.process_line(encode_line(request_body)),
                    [expected_terminal],
                )

    def test_real_simulator_quantizes_before_float32_range_validation(self) -> None:
        """Accepts rounded normal endpoints and rejects inf, zero, or subnormal."""
        maximum_rounds_to_float32 = (
            "340282347000000000000000000000000000000")
        minimum_rounds_to_normal = (
            "0.000000000000000000000000000000000000011754943")
        overflow_rounding_threshold = (
            "340282356779733661637539395458142568448")
        subnormal_rounding_threshold = (
            "0.0000000000000000000000000000000000000117549428")
        rounds_to_zero = (
            "0.0000000000000000000000000000000000000000000001")

        accepted_tokens = (
            (maximum_rounds_to_float32, simulator_module.FLOAT32_MAX),
            (minimum_rounds_to_normal, simulator_module.FLOAT32_MIN_NORMAL),
        )
        for token, expected_value in accepted_tokens:
            with self.subTest(token=token):
                status, parsed_value = simulator_module.parse_strict_float32_token(
                    token)
                self.assertEqual(status, "ok")
                self.assertEqual(parsed_value, expected_value)

        for token in (overflow_rounding_threshold,
                      subnormal_rounding_threshold,
                      rounds_to_zero):
            with self.subTest(token=token):
                self.assertEqual(
                    simulator_module.parse_strict_float32_token(token),
                    ("bad_argument", 0.0),
                )

        completed_transport = SimulatorTransport()
        completed_body = (f"154 bench move 1 position={minimum_rounds_to_normal} "
                          "speed=1")
        completed_outputs = completed_transport.transact_until_done(
            completed_body, 154, 2.0)
        self.assertEqual(completed_outputs[0],
                         "ok 154 bench move accepted=1")
        self.assertTrue(completed_outputs[-1].startswith(
            "done 154 bench move result=completed "))

        failed_transport = SimulatorTransport()
        failed_body = (f"155 bench move 1 position=0 "
                       f"speed={maximum_rounds_to_float32}")
        failed_outputs = failed_transport.transact_until_done(
            failed_body, 155, 2.0)
        self.assertEqual(failed_outputs, [
            "ok 155 bench move accepted=1",
            "done 155 bench move result=failed stage=validate "
            "code=speed_out_of_range motor=1",
        ])

    def test_real_simulator_legacy_motor_lists_require_strict_ascending_order(
            self) -> None:
        """Rejects unordered legacy masks while preserving move caller order."""
        legacy_requests = (
            (90, "init", ""),
            (91, "enable", ""),
            (92, "jog", " delta=4 speed=4"),
            (93, "stop", ""),
            (94, "disable", ""),
            (95, "clear", ""),
        )
        for request_id, operation, fields in legacy_requests:
            with self.subTest(operation=operation):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                self.assertEqual(
                    simulator.process_line(encode_line(
                        f"{request_id} bench {operation} 3,1{fields}")),
                    [f"error {request_id} bench {operation} "
                     "code=bad_argument field=motors"],
                )

        self.assertEqual(
            self.request("96 bench move 3,1 position=-45,90 speed=20,30"),
            ["ok 96 bench move accepted=1"],
        )
        self.assertEqual(self.simulator.active_motion.target_deg[2], -45.0)
        self.assertEqual(self.simulator.active_motion.target_deg[0], 90.0)

    def test_real_simulator_motor_tokens_match_firmware_ascii_u32_rules(
            self) -> None:
        """Accepts leading zeroes but rejects non-ASCII or signed motor tokens."""
        valid_requests = (
            "107 bench enable 01,03",
            "108 bench move 03,01 position=-45,90 speed=20,30",
        )
        for request_body in valid_requests:
            with self.subTest(request=request_body):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                self.assertEqual(
                    simulator.process_line(encode_line(request_body)),
                    [f"ok {request_body.split()[0]} bench "
                     f"{request_body.split()[2]} accepted=1"],
                )

        operation_fields = {
            "init": "",
            "enable": "",
            "jog": " delta=1 speed=1",
            "move": " position=1 speed=1",
            "stop": "",
            "disable": "",
            "clear": "",
        }
        invalid_motor_lists = ("+1", "-1", "1,,3", "")
        request_id = 109
        for operation, fields in operation_fields.items():
            for motor_list in invalid_motor_lists:
                with self.subTest(operation=operation, motors=motor_list):
                    simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                    request_body = (f"{request_id} bench {operation} "
                                    f"{motor_list}{fields}")
                    self.assertEqual(
                        simulator.process_line(encode_line(request_body)),
                        [f"error {request_id} bench {operation} "
                         "code=bad_argument field=motors"],
                    )
                    request_id += 1

        with self.assertRaisesRegex(ValueError, "motors"):
            AethorTextSimulator._parse_motor_list("１", require_ascending=False)
        self.assertEqual(
            self.simulator.process_line("144 bench move １ position=1 speed=1"),
            ["error 0 parse code=bad_line"],
        )

    def test_real_simulator_legacy_jog_uses_firmware_numeric_grammar(self) -> None:
        """Accepts unrestricted valid jog values and rejects exponent syntax."""
        valid_jogs = (
            (100, "delta=4 speed=4"),
            (101, "delta=90 speed=30"),
            (102, "delta=0 speed=30"),
            (103, "delta=-90 speed=30"),
        )
        for request_id, fields in valid_jogs:
            with self.subTest(fields=fields):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                self.assertEqual(
                    simulator.process_line(encode_line(
                        f"{request_id} bench jog 1 {fields}")),
                    [f"ok {request_id} bench jog accepted=1"],
                )

        invalid_jogs = (
            (104, "delta=1e-2 speed=1", "delta"),
            (105, "delta=1 speed=1E2", "speed"),
            (106, "delta=1 speed=1 extra=1", "delta"),
        )
        for request_id, fields, field_name in invalid_jogs:
            with self.subTest(fields=fields):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                self.assertEqual(
                    simulator.process_line(encode_line(
                        f"{request_id} bench jog 1 {fields}")),
                    [f"error {request_id} bench jog "
                     f"code=bad_argument field={field_name}"],
                )

    def test_real_client_receives_move_limit_failures_as_terminals(self) -> None:
        """Reports simulated discovered-limit failures after ACK and replays DONE."""
        cases = (
            ([800.0, 0.0], [30.0, 20.0],
             "position_out_of_range", 3),
            ([0.0, 0.0], [30.0, 1200.0],
             "speed_out_of_range", 1),
        )
        for positions, speeds, expected_code, expected_motor in cases:
            with self.subTest(code=expected_code):
                transport = SimulatorTransport()
                client = AethorReferenceClient(transport)
                result = client.bench_move_once(
                    [3, 1], positions, speeds)
                self.assertEqual(result.result, "failed")
                self.assertEqual(result.stage, "validate")
                self.assertEqual(result.code, expected_code)
                self.assertEqual(result.motor, expected_motor)
                self.assertEqual(client.transcript[1],
                                 "< ok 1 bench move accepted=1")
                self.assertEqual(client.transcript[2], f"< {result.raw_line}")
                request_body = client.transcript[0][2:]
                self.assertEqual(
                    transport.simulator.process_line(encode_line(request_body)),
                    [result.raw_line],
                )
                self.assertIsNone(transport.simulator.active_motion)
                self.assertEqual(transport.simulator.motor_enabled_mask, 0)

    def test_real_simulator_move_validation_uses_joint_order_and_replays_terminal(
            self) -> None:
        """Reports the first per-joint position-then-speed validation failure."""
        cases = (
            ("150 bench move 3,1 position=800,0 speed=30,1200",
             "speed_out_of_range", 1),
            ("151 bench move 5,2 position=800,800 speed=1200,30",
             "position_out_of_range", 2),
            ("152 bench move 3 position=800 speed=1200",
             "position_out_of_range", 3),
        )
        for request_body, expected_code, expected_motor in cases:
            with self.subTest(request=request_body):
                simulator = AethorTextSimulator(boot_id=1234, profile="bench")
                request_id = request_body.split()[0]
                expected_accepted = f"ok {request_id} bench move accepted=1"
                expected_terminal = (f"done {request_id} bench move result=failed "
                                     f"stage=validate code={expected_code} "
                                     f"motor={expected_motor}")
                self.assertEqual(
                    simulator.process_line(encode_line(request_body)),
                    [expected_accepted],
                )
                self.assertEqual(simulator.drain_outputs(), [expected_terminal])
                self.assertEqual(
                    simulator.process_line(encode_line(request_body)),
                    [expected_terminal],
                )

    def test_real_simulator_stop_preempts_move_and_replays_terminals(self) -> None:
        """Cancels one move before completing STOP with the union motor mask."""
        self.assertEqual(
            self.request("50 bench move 1 position=90 speed=30"),
            ["ok 50 bench move accepted=1"],
        )
        self.assertEqual(
            self.request("51 bench stop 3"),
            ["ok 51 bench stop accepted=1"],
        )
        expected_terminals = [
            "done 50 bench move result=cancelled",
            "done 51 bench stop result=stopped stopped=05",
        ]
        self.assertEqual(self.simulator.drain_outputs(), expected_terminals)
        self.assertIsNone(self.simulator.active_motion)
        self.assertEqual(self.simulator.motor_enabled_mask, 0)

        self.assertEqual(
            self.request("50 bench move 1 position=90 speed=30"),
            [expected_terminals[0]],
        )
        self.assertEqual(self.request("51 bench stop 3"),
                         [expected_terminals[1]])
        self.assertEqual(
            self.request("51 bench stop 2"),
            ["error 51 bench stop code=request_conflict"],
        )
        self.assertEqual(self.simulator.drain_outputs(), [])

    def test_real_simulator_stop_without_move_has_one_replayable_terminal(
            self) -> None:
        """Completes and replays one standalone STOP without cancellation output."""
        self.assertEqual(self.request("60 bench stop 2"),
                         ["ok 60 bench stop accepted=1"])
        terminal = "done 60 bench stop result=stopped stopped=02"
        self.assertEqual(self.simulator.drain_outputs(), [terminal])
        self.assertEqual(self.request("60 bench stop 2"), [terminal])
        self.assertEqual(self.simulator.drain_outputs(), [])

    def test_real_simulator_stop_preserves_legacy_jog_terminal_schema(self) -> None:
        """Cancels an active legacy jog without mislabeling it as bench move."""
        self.assertEqual(
            self.request("61 bench jog 1 delta=0.2 speed=1"),
            ["ok 61 bench jog accepted=1"],
        )
        self.assertEqual(self.request("62 bench stop 3"),
                         ["ok 62 bench stop accepted=1"])
        self.assertEqual(
            self.simulator.drain_outputs(),
            [
                "done 61 bench jog result=cancelled elapsed_ms=0 arrived=00",
                "done 62 bench stop result=stopped stopped=05",
            ],
        )

    def test_manifest_vectors_client_and_simulator_share_public_tokens(self) -> None:
        """Cross-checks public command, size, token, and request-ID assets."""
        manifest_path = (PROJECT_ROOT / "docs" / "compatibility" /
                         "aethor-text-v1-manifest.json")
        vector_path = (PROJECT_ROOT / "Tests" / "protocol" /
                       "aethor-text-v1-vectors.json")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        vectors = json.loads(vector_path.read_text(encoding="utf-8"))

        self.assertIn("move", manifest["profiles"]["bench"]["commands"])
        help_response = self.request("help bench")[0]
        help_commands = help_response.split("commands=", 1)[1].split(",")
        self.assertEqual(help_commands,
                         manifest["profiles"]["bench"]["commands"])
        self.assertEqual(manifest["transport"]["maximum_request_bytes"],
                         reference_client.AETHOR_TEXT_MAX_REQUEST_BODY_BYTES)
        self.assertEqual(manifest["transport"]["maximum_request_bytes"],
                         simulator_module.AETHOR_TEXT_MAX_REQUEST_BODY_BYTES)
        self.assertTrue(
            manifest["lifecycle"]["valid_request_refreshes_watchdog"])
        self.assertTrue(
            manifest["lifecycle"]["exact_replay_refreshes_watchdog"])
        self.assertFalse(
            manifest["lifecycle"]["rejected_request_refreshes_watchdog"])
        self.assertEqual(set(manifest["bench_move_results"]["stages"]),
                         reference_client.ONE_SHOT_MOVE_STAGES)
        self.assertEqual(set(manifest["bench_move_results"]["codes"]),
                         reference_client.ONE_SHOT_MOVE_ERRORS)
        self.assertEqual(
            set(manifest["bench_move_results"]["result_tokens"]),
            reference_client.ONE_SHOT_MOVE_RESULTS,
        )
        self.assertEqual(manifest["profiles"]["bench"]["move"]["request_id_min"],
                         1)
        self.assertEqual(manifest["profiles"]["bench"]["move"]["request_id_max"],
                         0xFFFFFFFF)
        self.assertEqual(
            manifest["profiles"]["bench"]["move"]["float32_min_normal"],
            reference_client.FLOAT32_MIN_NORMAL,
        )
        self.assertEqual(
            manifest["profiles"]["bench"]["move"]["float32_max"],
            reference_client.FLOAT32_MAX,
        )
        self.assertEqual(
            manifest["profiles"]["bench"]["move"]
                    ["numeric_token_max_characters"],
            simulator_module.AETHOR_TEXT_MAX_FLOAT_TOKEN_CHARACTERS,
        )
        show_motor_response = self.request("show motor 1")[0]
        show_motor_fields = {
            token.split("=", 1)[0]
            for token in show_motor_response.split()
            if "=" in token
        }
        self.assertEqual(
            set(manifest["show_motor_move_fields"]) - {"format"},
            {"pmax_deg", "vmax_deg_s", "max_speed_deg_s",
             "move_speed_limit_deg_s"},
        )
        self.assertTrue(
            (set(manifest["show_motor_move_fields"]) - {"format"}) <=
            show_motor_fields)
        for vector in vectors["bench_move_build_cases"]:
            self.assertGreater(vector["request_id"], 0, vector["name"])
            self.assertLessEqual(vector["request_id"], 0xFFFFFFFF,
                                 vector["name"])
        for vector in vectors["bench_move_invalid_cases"]:
            self.assertGreaterEqual(vector["request_id"], 0, vector["name"])
            self.assertLessEqual(vector["request_id"], 0xFFFFFFFF,
                                 vector["name"])
        for sequence in vectors["bench_move_request_sequences"]:
            for request in sequence["requests"]:
                self.assertGreater(int(request.split()[0]), 0, sequence["name"])


if __name__ == "__main__":
    unittest.main()
