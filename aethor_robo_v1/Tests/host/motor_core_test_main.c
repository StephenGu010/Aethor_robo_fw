/**
 * @file motor_core_test_main.c
 * @brief Host-side tests for the seven-motor bank and bounded CAN scheduler.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "arm_config.h"
#include "can_frame.h"
#include "can_tx_scheduler.h"
#include "motor_bank.h"

/**
 * @brief Creates one deterministic eight-byte Classic CAN frame.
 * @param identifier Standard CAN identifier.
 * @param marker Payload marker used to distinguish frames.
 * @return Initialized frame value.
 */
static CanFrame make_test_frame(uint16_t identifier, uint8_t marker)
{
    uint8_t payload[CAN_CLASSIC_MAX_DATA_LENGTH] = {0U};
    CanFrame frame;

    payload[0] = marker;
    assert(can_frame_init(&frame,
                          identifier,
                          payload,
                          CAN_CLASSIC_MAX_DATA_LENGTH) == CAN_FRAME_STATUS_OK);
    return frame;
}

/**
 * @brief Verifies the production mapping creates exactly seven independent motors.
 */
static void test_motor_bank_uses_frozen_seven_axis_mapping(void)
{
    const ArmConfig *configuration = arm_config_get_production();
    MotorBank bank;
    uint8_t joint_index;

    assert(motor_bank_init(&bank, configuration) == MOTOR_BANK_STATUS_OK);
    assert(bank.initialized != 0U);
    assert(bank.valid_joint_mask == 0U);

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        assert(bank.motors[joint_index].joint_index == joint_index);
        assert(bank.motors[joint_index].esc_id == (uint16_t)(joint_index + 0x01U));
        assert(bank.motors[joint_index].master_id == (uint16_t)(joint_index + 0x11U));
        assert(bank.motors[joint_index].feedback_valid == 0U);
    }
}

/**
 * @brief Verifies feedback identity, monotonic timestamps, and stale snapshot masks.
 */
static void test_motor_bank_publishes_coherent_feedback_snapshots(void)
{
    const ArmConfig *configuration = arm_config_get_production();
    MotorBank bank;
    MotorJointFeedback feedback = {0};
    MotorFeedbackSnapshot snapshot;

    assert(motor_bank_init(&bank, configuration) == MOTOR_BANK_STATUS_OK);
    feedback.position_rad = 1.25F;
    feedback.velocity_rad_s = -0.50F;
    feedback.torque_nm = 0.75F;
    feedback.fault_flags = 0U;
    feedback.timestamp_us = 1000U;

    assert(motor_bank_update_feedback(&bank, 0x11U, 0x01U, &feedback) ==
           MOTOR_BANK_STATUS_OK);
    assert(motor_bank_update_feedback(&bank, 0x12U, 0x01U, &feedback) ==
           MOTOR_BANK_STATUS_ID_MISMATCH);
    assert(motor_bank_update_feedback(&bank, 0x11U, 0x01U, &feedback) ==
           MOTOR_BANK_STATUS_STALE_SAMPLE);

    assert(motor_bank_get_snapshot(&bank, 1200U, 500U, &snapshot) ==
           MOTOR_BANK_STATUS_OK);
    assert(snapshot.valid_joint_mask == 0x01U);
    assert(snapshot.generation == 1U);
    assert(snapshot.published_at_us == 1200U);
    assert(snapshot.joints[0].position_rad == 1.25F);

    assert(motor_bank_get_snapshot(&bank, 1600U, 500U, &snapshot) ==
           MOTOR_BANK_STATUS_OK);
    assert(snapshot.valid_joint_mask == 0U);
    assert(snapshot.generation == 1U);
}

/**
 * @brief Verifies emergency frames preempt normal traffic without overwriting it.
 */
static void test_can_scheduler_prioritizes_emergency_frames(void)
{
    CanTxScheduler scheduler;
    CanFrame normal_frame = make_test_frame(0x101U, 0x11U);
    CanFrame emergency_frame = make_test_frame(0x101U, 0xFDU);
    CanFrame popped_frame;
    CanTxPriority popped_priority;

    can_tx_scheduler_init(&scheduler);
    assert(can_tx_scheduler_submit_joint(&scheduler, 0U, &normal_frame) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(can_tx_scheduler_submit_emergency(&scheduler, &emergency_frame) ==
           CAN_TX_SCHEDULER_STATUS_OK);

    assert(can_tx_scheduler_pop(&scheduler, &popped_frame, &popped_priority) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(popped_priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(popped_frame.data[0] == 0xFDU);

    assert(can_tx_scheduler_pop(&scheduler, &popped_frame, &popped_priority) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(popped_priority == CAN_TX_PRIORITY_JOINT_CONTROL);
    assert(popped_frame.data[0] == 0x11U);
}

/**
 * @brief Verifies joint slots coalesce latest values and pop in round-robin order.
 */
static void test_can_scheduler_is_bounded_and_fair(void)
{
    CanTxScheduler scheduler;
    CanFrame joint_zero_old = make_test_frame(0x101U, 0x10U);
    CanFrame joint_zero_new = make_test_frame(0x101U, 0x20U);
    CanFrame joint_three = make_test_frame(0x104U, 0x43U);
    CanFrame popped_frame;
    CanTxPriority popped_priority;

    can_tx_scheduler_init(&scheduler);
    assert(can_tx_scheduler_submit_joint(&scheduler, 0U, &joint_zero_old) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(can_tx_scheduler_submit_joint(&scheduler, 3U, &joint_three) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(can_tx_scheduler_submit_joint(&scheduler, 0U, &joint_zero_new) ==
           CAN_TX_SCHEDULER_STATUS_REPLACED);
    assert(scheduler.coalesced_joint_frame_count == 1U);

    assert(can_tx_scheduler_pop(&scheduler, &popped_frame, &popped_priority) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(popped_frame.data[0] == 0x20U);
    assert(can_tx_scheduler_pop(&scheduler, &popped_frame, &popped_priority) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(popped_frame.data[0] == 0x43U);
    assert(can_tx_scheduler_pop(&scheduler, &popped_frame, &popped_priority) ==
           CAN_TX_SCHEDULER_STATUS_EMPTY);
}

/**
 * @brief Runs all seven-motor core tests.
 * @return Zero when every assertion passes.
 */
int main(void)
{
    test_motor_bank_uses_frozen_seven_axis_mapping();
    test_motor_bank_publishes_coherent_feedback_snapshots();
    test_can_scheduler_prioritizes_emergency_frames();
    test_can_scheduler_is_bounded_and_fair();
    puts("MOTOR_CORE_TESTS_PASSED");
    return 0;
}
