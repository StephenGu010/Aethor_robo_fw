"""File-level deterministic long-duration tests for the Aethor host simulator."""

from __future__ import annotations

import pathlib
import sys
import unittest

PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from aethor_host_simulator import AethorHostSimulator, encode_frame  # noqa: E402


class AethorSimulatorSoakTests(unittest.TestCase):
    """Exercises bounded state through the PRD logical 8-hour and 2-hour runs."""

    @staticmethod
    def request(simulator: AethorHostSimulator, body: str) -> list[str]:
        """Encodes and sends one request body to an explicit simulator."""
        return simulator.process_line(encode_frame(body))

    def test_eight_logical_hours_disabled_at_fifty_hertz(self) -> None:
        """Runs 1,440,000 telemetry periods without replay or queue growth."""
        simulator = AethorHostSimulator(boot_id=8001)
        self.request(simulator, "REQ 1 HELLO client=soak protocol=1")
        self.request(simulator,
                     "REQ 2 SET_STREAM rate_hz=50 fields=jpos,jvel,state,motor")
        next_request_id = 3
        telemetry_samples = 0
        maximum_pending_outputs = 0

        for telemetry_period in range(8 * 60 * 60 * 50):
            simulator.advance(20)
            pending_outputs = simulator.drain_outputs()
            telemetry_samples += sum(item.startswith("TEL ")
                                     for item in pending_outputs)
            maximum_pending_outputs = max(maximum_pending_outputs,
                                          len(pending_outputs))
            if telemetry_period % 10 == 9:
                heartbeat = self.request(
                    simulator,
                    f"REQ {next_request_id} HEARTBEAT")
                self.assertTrue(heartbeat[0].startswith("RSP "))
                next_request_id += 1

        self.assertEqual(simulator.now_ms, 8 * 60 * 60 * 1000)
        self.assertEqual(telemetry_samples,
                         8 * 60 * 60 * (50 + 10))
        self.assertLessEqual(maximum_pending_outputs, 2)
        self.assertLessEqual(len(simulator._replay), 32)
        self.assertEqual(simulator.arm_state, "UNALIGNED")
        self.assertFalse(any(simulator.motor_enabled))

    def test_two_logical_hours_move_stop_and_fault_recovery(self) -> None:
        """Cycles synchronized moves, controlled stops, and explicit recovery."""
        simulator = AethorHostSimulator(boot_id=2001)
        self.request(simulator, "REQ 1 HELLO client=soak protocol=1")
        simulator.force_ready(mode="POS_VEL")
        next_request_id = 2
        completed_moves = 0
        stopped_moves = 0
        recovered_faults = 0

        for cycle in range(7200):
            target = 1 if cycle % 2 == 0 else -1
            target_vector = ",".join([str(target)] * 7)
            move_id = next_request_id
            move_output = self.request(
                simulator,
                f"REQ {move_id} MOVE_JOINTS q_deg={target_vector} "
                "speed=1 mode=POS_VEL")
            self.assertTrue(move_output[0].startswith(f"ACK {move_id} "))
            next_request_id += 1

            if cycle % 3 == 0:
                simulator.advance(20)
                stop_id = next_request_id
                stop_output = self.request(simulator,
                                           f"REQ {stop_id} STOP behavior=controlled")
                self.assertTrue(stop_output[0].startswith(f"ACK {stop_id} "))
                next_request_id += 1
                stopped_moves += 1
            else:
                simulator.advance(100)
                self.assertTrue(any(f"DONE {move_id} COMPLETED" in item
                                    for item in simulator.drain_outputs()))
                completed_moves += 1

            if cycle % 300 == 299:
                simulator.inject_fault(3, "overtemperature")
                simulator.drain_outputs()
                simulator.clear_injected_faults()
                clear_id = next_request_id
                self.request(simulator, f"REQ {clear_id} CLEAR_FAULT")
                simulator.drain_outputs()
                next_request_id += 1
                enable_id = next_request_id
                self.request(simulator, f"REQ {enable_id} ENABLE")
                simulator.drain_outputs()
                next_request_id += 1
                recovered_faults += 1

            remaining_cycle_ms = 1000 - (20 if cycle % 3 == 0 else 100)
            while remaining_cycle_ms > 0:
                advance_ms = min(200, remaining_cycle_ms)
                simulator.advance(advance_ms)
                simulator.drain_outputs()
                heartbeat_id = next_request_id
                self.request(simulator, f"REQ {heartbeat_id} HEARTBEAT")
                next_request_id += 1
                remaining_cycle_ms -= advance_ms

        self.assertEqual(simulator.now_ms, 2 * 60 * 60 * 1000)
        self.assertEqual(completed_moves, 4800)
        self.assertEqual(stopped_moves, 2400)
        self.assertEqual(recovered_faults, 24)
        self.assertEqual(simulator.arm_state, "READY")
        self.assertTrue(all(simulator.motor_enabled))
        self.assertLessEqual(len(simulator._replay), 32)
        self.assertLessEqual(len(simulator._pending_outputs), 1)


if __name__ == "__main__":
    unittest.main()
