/**
 * @file motor_core_test_main.c
 * @brief Host-side tests for the seven-motor bank and bounded CAN scheduler.
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "arm_config.h"
#include "can_frame.h"
#include "can_tx_scheduler.h"
#include "motor_bank.h"
#include "motor_discovery.h"
#include "motor_runtime.h"
#include "s3519_codec.h"

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
 * @brief Initializes two discovered motors with deliberately different limits.
 * @param runtime Destination initialized runtime.
 */
static void prepare_runtime_with_two_discovered_motors(MotorRuntime *runtime)
{
    assert(motor_runtime_init(runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    runtime->discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    runtime->discovery.target_joint_mask = 0x05U;
    runtime->discovery.verified_joint_mask = 0x05U;

    runtime->discovery.results[0].ranges.position_max_rad = 2.0F;
    runtime->discovery.results[0].ranges.velocity_max_rad_s = 4.0F;
    runtime->discovery.results[0].ranges.torque_max_nm = 18.0F;
    runtime->discovery.results[0].maximum_speed_rad_s = 3.0F;
    runtime->discovery.results[0].verified_fields_mask =
        MOTOR_DISCOVERY_ALL_FIELDS_MASK;

    runtime->discovery.results[2].ranges.position_max_rad = 1.75F;
    runtime->discovery.results[2].ranges.velocity_max_rad_s = 2.5F;
    runtime->discovery.results[2].ranges.torque_max_nm = 16.0F;
    runtime->discovery.results[2].maximum_speed_rad_s = 5.0F;
    runtime->discovery.results[2].verified_fields_mask =
        MOTOR_DISCOVERY_ALL_FIELDS_MASK;
}

/**
 * @brief Creates a fault-free feedback snapshot for the two discovered motors.
 * @return Snapshot whose valid mask covers J1 and J3.
 */
static MotorFeedbackSnapshot make_two_motor_feedback_snapshot(void)
{
    MotorFeedbackSnapshot feedback_snapshot = {0};

    feedback_snapshot.valid_joint_mask = 0x05U;
    return feedback_snapshot;
}

/**
 * @brief Verifies every byte in an output batch has been cleared.
 * @param batch Batch expected to contain no partial frame.
 */
static void assert_motor_frame_batch_is_zeroed(
    const MotorEmergencyFrameBatch *batch)
{
    const uint8_t *batch_bytes = (const uint8_t *)batch;
    size_t byte_index;

    assert(batch != NULL);
    for (byte_index = 0U; byte_index < sizeof(*batch); ++byte_index)
    {
        assert(batch_bytes[byte_index] == 0U);
    }
}

/**
 * @brief Verifies dynamic POS_VEL limits come only from complete discovery data.
 */
static void test_motor_runtime_reports_discovered_position_velocity_limits(void)
{
    MotorRuntime runtime;
    MotorPositionVelocityLimits limits = {0};

    prepare_runtime_with_two_discovered_motors(&runtime);

    assert(motor_runtime_get_position_velocity_limits(&runtime, 0U, &limits) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(limits.position_max_rad == 2.0F);
    assert(limits.velocity_mapping_max_rad_s == 4.0F);
    assert(limits.maximum_speed_rad_s == 3.0F);
    assert(limits.move_speed_limit_rad_s == 3.0F);

    assert(motor_runtime_get_position_velocity_limits(&runtime, 2U, &limits) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(limits.position_max_rad == 1.75F);
    assert(limits.velocity_mapping_max_rad_s == 2.5F);
    assert(limits.maximum_speed_rad_s == 5.0F);
    assert(limits.move_speed_limit_rad_s == 2.5F);

    memset(&limits, 0xA5, sizeof(limits));
    assert(motor_runtime_get_position_velocity_limits(&runtime, 1U, &limits) ==
           MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE);

    runtime.discovery.results[0].ranges.position_max_rad = NAN;
    assert(motor_runtime_get_position_velocity_limits(&runtime, 0U, &limits) ==
           MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE);
    runtime.discovery.results[0].ranges.position_max_rad = 2.0F;
    runtime.discovery.results[0].ranges.velocity_max_rad_s = 0.0F;
    assert(motor_runtime_get_position_velocity_limits(&runtime, 0U, &limits) ==
           MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE);
    runtime.discovery.results[0].ranges.velocity_max_rad_s = 4.0F;
    runtime.discovery.results[0].maximum_speed_rad_s = -1.0F;
    assert(motor_runtime_get_position_velocity_limits(&runtime, 0U, &limits) ==
           MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE);
}

/**
 * @brief Verifies move validation accepts ninety degrees and exact boundaries.
 */
static void test_motor_runtime_accepts_valid_position_velocity_move_limits(void)
{
    MotorRuntime runtime;
    MotorFeedbackSnapshot feedback_snapshot = make_two_motor_feedback_snapshot();
    float motor_position_rad[ARM_JOINT_COUNT] = {0.0F};
    float motor_speed_rad_s[ARM_JOINT_COUNT] = {0.0F};
    uint8_t failed_joint_index = 0xFFU;

    prepare_runtime_with_two_discovered_motors(&runtime);
    motor_position_rad[0] = 1.57079632679F;
    motor_position_rad[2] = -1.57079632679F;
    motor_speed_rad_s[0] = 2.0F;
    motor_speed_rad_s[2] = 2.0F;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime,
               &feedback_snapshot,
               0x05U,
               motor_position_rad,
               motor_speed_rad_s,
               &failed_joint_index) == MOTOR_RUNTIME_STATUS_OK);
    assert(failed_joint_index == 0xFFU);

    motor_position_rad[0] = 2.0F;
    motor_position_rad[2] = -1.75F;
    motor_speed_rad_s[0] = 3.0F;
    motor_speed_rad_s[2] = 2.5F;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime,
               &feedback_snapshot,
               0x05U,
               motor_position_rad,
               motor_speed_rad_s,
               &failed_joint_index) == MOTOR_RUNTIME_STATUS_OK);
}

/**
 * @brief Verifies move validation reports the first selected motor failure.
 */
static void test_motor_runtime_rejects_invalid_position_velocity_moves(void)
{
    MotorRuntime runtime;
    MotorFeedbackSnapshot feedback_snapshot = make_two_motor_feedback_snapshot();
    float motor_position_rad[ARM_JOINT_COUNT] = {0.0F};
    float motor_speed_rad_s[ARM_JOINT_COUNT] = {0.0F};
    uint8_t failed_joint_index = 0xFFU;

    prepare_runtime_with_two_discovered_motors(&runtime);
    motor_position_rad[0] = 1.0F;
    motor_position_rad[2] = 1.0F;
    motor_speed_rad_s[0] = 1.0F;
    motor_speed_rad_s[2] = 1.0F;

    motor_position_rad[0] = 2.01F;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_POSITION_OUT_OF_RANGE);
    assert(failed_joint_index == 0U);
    motor_position_rad[0] = 1.0F;

    motor_position_rad[2] = -1.76F;
    failed_joint_index = 0xFFU;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_POSITION_OUT_OF_RANGE);
    assert(failed_joint_index == 2U);
    motor_position_rad[2] = 1.0F;

    motor_speed_rad_s[0] = 3.01F;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE);
    assert(failed_joint_index == 0U);
    motor_speed_rad_s[0] = 1.0F;

    motor_speed_rad_s[2] = 2.51F;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE);
    assert(failed_joint_index == 2U);
    motor_speed_rad_s[2] = 1.0F;

    motor_position_rad[0] = NAN;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_POSITION_OUT_OF_RANGE);
    assert(failed_joint_index == 0U);
    motor_position_rad[0] = 1.0F;

    motor_speed_rad_s[0] = INFINITY;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE);
    assert(failed_joint_index == 0U);
    motor_speed_rad_s[0] = 0.0F;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE);
    assert(failed_joint_index == 0U);
    motor_speed_rad_s[0] = -0.1F;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE);
    assert(failed_joint_index == 0U);
}

/**
 * @brief Verifies move validation requires fresh, fault-free selected feedback.
 */
static void test_motor_runtime_rejects_missing_or_faulted_move_feedback(void)
{
    MotorRuntime runtime;
    MotorFeedbackSnapshot feedback_snapshot = make_two_motor_feedback_snapshot();
    float motor_position_rad[ARM_JOINT_COUNT] = {0.0F};
    float motor_speed_rad_s[ARM_JOINT_COUNT] = {0.0F};
    uint8_t failed_joint_index = 0xFFU;

    prepare_runtime_with_two_discovered_motors(&runtime);
    motor_speed_rad_s[0] = 1.0F;
    motor_speed_rad_s[2] = 1.0F;

    feedback_snapshot.valid_joint_mask = 0x01U;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_STALE_FEEDBACK);
    assert(failed_joint_index == 2U);

    feedback_snapshot.valid_joint_mask = 0x05U;
    feedback_snapshot.joints[2].fault_flags = 0x08U;
    assert(motor_runtime_validate_position_velocity_move_subset(
               &runtime, &feedback_snapshot, 0x05U, motor_position_rad,
               motor_speed_rad_s, &failed_joint_index) ==
           MOTOR_RUNTIME_STATUS_FAULT_PRESENT);
    assert(failed_joint_index == 2U);
}

/**
 * @brief Verifies subset encoding allows HOLD and never exposes partial frames.
 */
static void test_motor_runtime_builds_position_velocity_subset_atomically(void)
{
    MotorRuntime runtime;
    MotorEmergencyFrameBatch batch;
    float motor_position_rad[ARM_JOINT_COUNT] = {0.0F};
    float motor_speed_rad_s[ARM_JOINT_COUNT] = {0.0F};

    prepare_runtime_with_two_discovered_motors(&runtime);
    motor_position_rad[0] = 1.0F;
    motor_position_rad[2] = -1.0F;
    assert(motor_runtime_build_position_velocity_subset(
               &runtime, 0x05U, motor_position_rad, motor_speed_rad_s,
               &batch) == MOTOR_RUNTIME_STATUS_OK);
    assert(batch.count == 2U);
    assert(batch.frames[0].identifier == 0x101U);
    assert(batch.frames[1].identifier == 0x103U);
    assert(memcmp(&batch.frames[0].data[4],
                  &(float){0.0F},
                  sizeof(float)) == 0);
    assert(memcmp(&batch.frames[1].data[4],
                  &(float){0.0F},
                  sizeof(float)) == 0);

    memset(&batch, 0xA5, sizeof(batch));
    motor_position_rad[2] = 1.76F;
    assert(motor_runtime_build_position_velocity_subset(
               &runtime, 0x05U, motor_position_rad, motor_speed_rad_s,
               &batch) == MOTOR_RUNTIME_STATUS_POSITION_OUT_OF_RANGE);
    assert_motor_frame_batch_is_zeroed(&batch);

    memset(&batch, 0xA5, sizeof(batch));
    motor_position_rad[2] = -1.0F;
    motor_speed_rad_s[2] = NAN;
    assert(motor_runtime_build_position_velocity_subset(
               &runtime, 0x05U, motor_position_rad, motor_speed_rad_s,
               &batch) == MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE);
    assert_motor_frame_batch_is_zeroed(&batch);

    memset(&batch, 0xA5, sizeof(batch));
    motor_speed_rad_s[2] = -0.1F;
    assert(motor_runtime_build_position_velocity_subset(
               &runtime, 0x05U, motor_position_rad, motor_speed_rad_s,
               &batch) == MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE);
    assert(batch.count == 0U);
    assert_motor_frame_batch_is_zeroed(&batch);
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
        assert(bank.motors[joint_index].state == MOTOR_LIFECYCLE_ABSENT);
        assert(bank.motors[joint_index].configuration_consistent == 0U);
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
 * @brief Verifies a reader never accepts a snapshot during an active publish.
 */
static void test_motor_bank_rejects_snapshot_during_publish(void)
{
    MotorBank bank;
    MotorFeedbackSnapshot snapshot;

    assert(motor_bank_init(&bank, arm_config_get_production()) ==
           MOTOR_BANK_STATUS_OK);
    bank.publication_sequence = 1U;

    assert(motor_bank_get_snapshot(&bank, 1000U, 500U, &snapshot) ==
           MOTOR_BANK_STATUS_SNAPSHOT_BUSY);
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
    assert(CAN_TX_SCHEDULER_CAPACITY == 32U);
    assert(can_tx_scheduler_submit(&scheduler,
                                   CAN_TX_PRIORITY_JOINT_CONTROL,
                                   &normal_frame) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(can_tx_scheduler_submit(&scheduler,
                                   CAN_TX_PRIORITY_EMERGENCY,
                                   &emergency_frame) ==
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
 * @brief Verifies emergency traffic evicts lower-priority work from a full ring.
 */
static void test_can_scheduler_reserves_progress_for_emergency_frames(void)
{
    CanTxScheduler scheduler;
    CanFrame parameter_frame = make_test_frame(0x7FFU, 0x33U);
    CanFrame emergency_frame = make_test_frame(0x101U, 0xFDU);
    CanFrame popped_frame;
    CanTxPriority popped_priority;
    uint8_t frame_index;

    can_tx_scheduler_init(&scheduler);
    for (frame_index = 0U; frame_index < CAN_TX_SCHEDULER_CAPACITY; ++frame_index)
    {
        assert(can_tx_scheduler_submit(&scheduler,
                                       CAN_TX_PRIORITY_PARAMETER,
                                       &parameter_frame) ==
               CAN_TX_SCHEDULER_STATUS_OK);
    }
    assert(can_tx_scheduler_submit(&scheduler,
                                   CAN_TX_PRIORITY_EMERGENCY,
                                   &emergency_frame) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(scheduler.count == CAN_TX_SCHEDULER_CAPACITY);
    assert(can_tx_scheduler_pop(&scheduler, &popped_frame, &popped_priority) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(popped_priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(popped_frame.data[0] == 0xFDU);
}

/**
 * @brief Verifies an unknown motor mode produces both safe disable identifiers.
 */
static void test_motor_runtime_builds_fail_safe_disable_batch(void)
{
    MotorRuntime runtime;
    MotorEmergencyFrameBatch batch;

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    assert(motor_runtime_build_emergency_disable(&runtime, &batch) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(batch.count == 14U);
    assert(batch.frames[0].identifier == 0x001U);
    assert(batch.frames[1].identifier == 0x101U);
    assert(batch.frames[0].data[7] == 0xFDU);
    assert(batch.frames[1].data[7] == 0xFDU);
}

/**
 * @brief Verifies selected unknown modes emit both identifiers in joint order.
 */
static void test_motor_runtime_builds_selected_unknown_mode_disable_batch(void)
{
    MotorRuntime runtime;
    MotorEmergencyFrameBatch batch;

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_build_emergency_disable_subset(
               &runtime, 0x05U, &batch) == MOTOR_RUNTIME_STATUS_OK);
    assert(batch.count == 4U);
    assert(batch.frames[0].identifier == 0x001U);
    assert(batch.frames[1].identifier == 0x101U);
    assert(batch.frames[2].identifier == 0x003U);
    assert(batch.frames[3].identifier == 0x103U);
    assert(batch.frames[0].data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(batch.frames[1].data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(batch.frames[2].data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(batch.frames[3].data[7] == S3519_MODE_COMMAND_DISABLE);
}

/**
 * @brief Verifies selected verified mixed modes use only their known identifiers.
 */
static void test_motor_runtime_builds_selected_known_mode_disable_batch(void)
{
    MotorRuntime runtime;
    MotorEmergencyFrameBatch batch;

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    runtime.discovery.results[0].verified_fields_mask =
        MOTOR_DISCOVERY_MODE_FIELDS_MASK;
    runtime.discovery.results[0].observed_control_mode = 1U;
    runtime.discovery.results[2].verified_fields_mask =
        MOTOR_DISCOVERY_MODE_FIELDS_MASK;
    runtime.discovery.results[2].observed_control_mode = 2U;

    assert(motor_runtime_build_emergency_disable_subset(
               &runtime, 0x05U, &batch) == MOTOR_RUNTIME_STATUS_OK);
    assert(batch.count == 2U);
    assert(batch.frames[0].identifier == 0x001U);
    assert(batch.frames[1].identifier == 0x103U);
    assert(batch.frames[0].data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(batch.frames[1].data[7] == S3519_MODE_COMMAND_DISABLE);
}

/**
 * @brief Verifies invalid subset requests never expose partial emergency frames.
 */
static void test_motor_runtime_rejects_invalid_emergency_disable_subset(void)
{
    MotorRuntime runtime;
    MotorEmergencyFrameBatch batch;

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    memset(&batch, 0xA5, sizeof(batch));
    assert(motor_runtime_build_emergency_disable_subset(
               &runtime, 0U, &batch) == MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT);
    assert_motor_frame_batch_is_zeroed(&batch);

    memset(&batch, 0xA5, sizeof(batch));
    assert(motor_runtime_build_emergency_disable_subset(
               &runtime, 0x80U, &batch) == MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT);
    assert_motor_frame_batch_is_zeroed(&batch);

    runtime.initialized = 0U;
    memset(&batch, 0xA5, sizeof(batch));
    assert(motor_runtime_build_emergency_disable_subset(
               &runtime, 0x01U, &batch) == MOTOR_RUNTIME_STATUS_NOT_INITIALIZED);
    assert_motor_frame_batch_is_zeroed(&batch);
}

/**
 * @brief Verifies seven mode writes are followed by exact register readbacks.
 */
static void test_motor_runtime_switches_mode_with_readback(void)
{
    MotorRuntime runtime;
    CanFrame frame;
    uint8_t joint_index;
    uint64_t timestamp_us = 1000U;

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    runtime.discovery.verified_joint_mask = 0x7FU;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        runtime.discovery.results[joint_index].verified_fields_mask =
            MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    }
    assert(motor_runtime_begin_control_mode_switch(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY) == MOTOR_RUNTIME_STATUS_OK);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        assert(motor_runtime_next_control_mode_frame(&runtime,
                                                     timestamp_us,
                                                     &frame) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.identifier == 0x7FFU);
        assert(frame.data[2] == 0x55U);
        assert(frame.data[3] == S3519_REGISTER_CONTROL_MODE);
        assert(frame.data[4] == 2U);
        timestamp_us += 1000U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t response_payload[8] = {0U};
        CanFrame response;

        assert(motor_runtime_next_control_mode_frame(&runtime,
                                                     timestamp_us,
                                                     &frame) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.data[2] == 0x33U);
        assert(frame.data[3] == S3519_REGISTER_CONTROL_MODE);
        response_payload[0] = (uint8_t)(joint_index + 1U);
        response_payload[2] = 0x33U;
        response_payload[3] = S3519_REGISTER_CONTROL_MODE;
        response_payload[4] = 2U;
        assert(can_frame_init(&response,
                              (uint16_t)(0x11U + joint_index),
                              response_payload,
                              sizeof(response_payload)) == CAN_FRAME_STATUS_OK);
        assert(motor_runtime_accept_frame(&runtime,
                                          &response,
                                          timestamp_us + 100U) ==
               ((joint_index == (ARM_JOINT_COUNT - 1U))
                    ? MOTOR_RUNTIME_STATUS_ACTION_COMPLETE
                    : MOTOR_RUNTIME_STATUS_OK));
        timestamp_us += 1000U;
    }
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 timestamp_us,
                                                 &frame) ==
           MOTOR_RUNTIME_STATUS_ACTION_COMPLETE);
}

/**
 * @brief Verifies a complete selected motor can switch mode while another discovery waits.
 */
static void test_motor_runtime_masked_mode_switch_serializes_unselected_discovery(void)
{
    MotorRuntime runtime;
    CanFrame request;
    CanFrame response;
    CanFrame unselected_discovery_response;
    uint8_t selected_response_payload[8] = {
        1U, 0U, 0x33U, S3519_REGISTER_CONTROL_MODE, 2U, 0U, 0U, 0U
    };
    uint8_t unselected_response_payload[8] = {
        2U, 0U, 0x33U, S3519_REGISTER_CONTROL_MODE, 2U, 0U, 0U, 0U
    };

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    runtime.discovery.verified_joint_mask = 0x01U;
    runtime.discovery.results[0].verified_fields_mask =
        MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    runtime.discovery.state = MOTOR_DISCOVERY_STATE_READY;
    runtime.discovery.target_joint_mask = 0x02U;
    runtime.discovery.current_joint_index = 1U;
    runtime.discovery.current_register_index = 2U;
    runtime.discovery_active = 1U;
    assert(motor_runtime_next_discovery_frame(&runtime, 500U, &request) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request.data[0] == 2U);
    assert(request.data[3] == S3519_REGISTER_CONTROL_MODE);

    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x01U) == MOTOR_RUNTIME_STATUS_OK);
    assert((runtime.discovery.verified_joint_mask & 0x01U) == 0U);
    assert((runtime.discovery.results[0].verified_fields_mask &
            MOTOR_DISCOVERY_MODE_FIELDS_MASK) == 0U);
    assert(motor_runtime_next_control_mode_frame(&runtime, 1000U, &request) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request.data[0] == 1U);
    assert(request.data[2] == 0x55U);
    assert(motor_runtime_next_control_mode_frame(&runtime, 2000U, &request) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request.data[0] == 1U);
    assert(request.data[2] == 0x33U);
    assert(can_frame_init(&unselected_discovery_response,
                          0x12U,
                          unselected_response_payload,
                          sizeof(unselected_response_payload)) ==
           CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime,
                                      &unselected_discovery_response,
                                      2050U) == MOTOR_RUNTIME_STATUS_OK);
    assert(runtime.mode_switch_state == MOTOR_MODE_SWITCH_READ_WAITING);
    assert(can_frame_init(&response,
                          0x11U,
                          selected_response_payload,
                          sizeof(selected_response_payload)) ==
           CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &response, 2100U) ==
           MOTOR_RUNTIME_STATUS_ACTION_COMPLETE);
    assert((runtime.discovery.verified_joint_mask & 0x01U) != 0U);
    assert((runtime.discovery.results[0].verified_fields_mask &
            MOTOR_DISCOVERY_MODE_FIELDS_MASK) != 0U);
    assert(runtime.discovery.state == MOTOR_DISCOVERY_STATE_READY);
    assert(runtime.discovery.current_joint_index == 1U);
}

/**
 * @brief Verifies masked mode admission ignores unselected failure but rejects selected gaps.
 */
static void test_motor_runtime_masked_mode_switch_requires_selected_readiness(void)
{
    MotorRuntime runtime;
    CanFrame request;

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    runtime.discovery.verified_joint_mask = 0x01U;
    runtime.discovery.results[0].verified_fields_mask =
        MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    runtime.discovery.state = MOTOR_DISCOVERY_STATE_FAILED;
    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x01U) == MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_control_mode_frame(&runtime, 1000U, &request) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request.data[0] == 1U);

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    runtime.discovery.verified_joint_mask = 0x01U;
    runtime.discovery.results[0].verified_fields_mask =
        (uint16_t)(MOTOR_DISCOVERY_ALL_FIELDS_MASK &
                   (uint16_t)~MOTOR_DISCOVERY_RANGE_FIELDS_MASK);
    runtime.discovery.state = MOTOR_DISCOVERY_STATE_FAILED;
    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x01U) == MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR);
}

/**
 * @brief Verifies aborting active discovery and mode sequences is idempotent,
 *        emits no old parameter frame, and preserves verified runtime data.
 */
static void test_motor_runtime_aborts_active_parameter_sequences(void)
{
    MotorRuntime runtime;
    CanFrame frame;
    CanFrame response;
    MotorDiscoveryResult preserved_result;
    uint32_t preserved_feedback_count;
    uint8_t response_payload[8] = {
        1U,
        0U,
        0x33U,
        S3519_REGISTER_CONTROL_MODE,
        2U,
        0U,
        0U,
        0U
    };

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    runtime.discovery.verified_joint_mask = 0x02U;
    runtime.discovery.results[1].verified_fields_mask =
        MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    runtime.discovery.results[1].observed_control_mode = 2U;
    runtime.discovery.results[1].maximum_speed_rad_s = 20.0F;
    runtime.accepted_feedback_count = 7U;
    preserved_result = runtime.discovery.results[1];
    preserved_feedback_count = runtime.accepted_feedback_count;
    assert(motor_runtime_begin_discovery(&runtime, 0x01U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_discovery_frame(&runtime, 1000U, &frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);

    assert(motor_runtime_abort_active_parameter_sequences(&runtime, 1100U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(runtime.discovery.state == MOTOR_DISCOVERY_STATE_COMPLETE);
    assert(motor_runtime_next_discovery_frame(&runtime, 2000U, &frame) ==
           MOTOR_RUNTIME_STATUS_DISCOVERY_COMPLETE);
    assert(runtime.discovery.verified_joint_mask == 0x02U);
    assert(memcmp(&runtime.discovery.results[1],
                  &preserved_result,
                  sizeof(preserved_result)) == 0);
    assert(runtime.accepted_feedback_count == preserved_feedback_count);
    assert(motor_runtime_abort_active_parameter_sequences(&runtime, 2100U) ==
           MOTOR_RUNTIME_STATUS_OK);

    runtime.discovery.verified_joint_mask = 0x03U;
    runtime.discovery.results[0].verified_fields_mask =
        MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    runtime.discovery.results[0].observed_control_mode = 1U;
    runtime.discovery.results[1].verified_fields_mask =
        MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    runtime.discovery.results[1].observed_control_mode = 2U;
    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x01U) == MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_control_mode_frame(
               &runtime,
               1100U + MOTOR_DISCOVERY_REQUEST_TIMEOUT_US + 1U,
               &frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(motor_runtime_abort_active_parameter_sequences(
               &runtime,
               1200U + MOTOR_DISCOVERY_REQUEST_TIMEOUT_US) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(runtime.mode_switch_state == MOTOR_MODE_SWITCH_IDLE);
    assert(runtime.mode_switch_joint_mask == 0U);
    assert(runtime.mode_switch_joint_index == 0U);
    assert(runtime.mode_switch_attempt_count == 0U);
    assert(runtime.mode_request_sent_at_us == 0U);
    assert((runtime.discovery.results[0].verified_fields_mask &
            MOTOR_DISCOVERY_MODE_FIELDS_MASK) == 0U);
    assert((runtime.discovery.verified_joint_mask & 0x01U) == 0U);
    assert((runtime.discovery.results[1].verified_fields_mask &
            MOTOR_DISCOVERY_MODE_FIELDS_MASK) != 0U);
    assert((runtime.discovery.verified_joint_mask & 0x02U) != 0U);
    assert(motor_runtime_next_control_mode_frame(
               &runtime,
               1300U + MOTOR_DISCOVERY_REQUEST_TIMEOUT_US,
               &frame) ==
           MOTOR_RUNTIME_STATUS_WAITING);

    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x01U) == MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_control_mode_frame(
               &runtime,
               1400U + MOTOR_DISCOVERY_REQUEST_TIMEOUT_US,
               &frame) == MOTOR_RUNTIME_STATUS_WAITING);
    assert(motor_runtime_next_control_mode_frame(
               &runtime,
               1201U + (2U * MOTOR_DISCOVERY_REQUEST_TIMEOUT_US),
               &frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[2] == 0x55U);
    assert(motor_runtime_next_control_mode_frame(
               &runtime,
               1202U + (2U * MOTOR_DISCOVERY_REQUEST_TIMEOUT_US),
               &frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[2] == 0x33U);
    assert(can_frame_init(&response,
                          0x11U,
                          response_payload,
                          sizeof(response_payload)) == CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(
               &runtime,
               &response,
               1203U + (2U * MOTOR_DISCOVERY_REQUEST_TIMEOUT_US)) ==
           MOTOR_RUNTIME_STATUS_ACTION_COMPLETE);
    assert((runtime.discovery.results[0].verified_fields_mask &
            MOTOR_DISCOVERY_MODE_FIELDS_MASK) != 0U);
    assert((runtime.discovery.verified_joint_mask & 0x01U) != 0U);
}

/**
 * @brief Verifies late discovery and mode responses are discarded after abort.
 */
static void test_motor_runtime_discards_late_parameter_responses_after_abort(void)
{
    static const uint8_t feedback_payload[8] = {
        0x01U,
        0x80U,
        0x00U,
        0x80U,
        0x08U,
        0x00U,
        42U,
        40U
    };
    MotorRuntime runtime;
    CanFrame feedback_frame;
    CanFrame request_frame;
    CanFrame response_frame;
    MotorDiscoveryResult preserved_discovery_result;
    MotorJointFeedback preserved_feedback;
    uint32_t preserved_accepted_feedback_count;
    uint32_t preserved_rejected_feedback_count;
    uint8_t discovery_response_payload[8] = {
        1U,
        0U,
        0x33U,
        S3519_REGISTER_MASTER_ID,
        0x11U,
        0U,
        0U,
        0U
    };
    uint8_t mode_response_payload[8] = {
        1U,
        0U,
        0x33U,
        S3519_REGISTER_CONTROL_MODE,
        2U,
        0U,
        0U,
        0U
    };

    prepare_runtime_with_two_discovered_motors(&runtime);
    runtime.discovery.results[0].observed_control_mode = 1U;
    assert(can_frame_init(&feedback_frame,
                          0x11U,
                          feedback_payload,
                          sizeof(feedback_payload)) == CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &feedback_frame, 1000U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_begin_discovery(&runtime, 0x01U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_discovery_frame(&runtime,
                                              2000U,
                                              &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request_frame.data[3] == S3519_REGISTER_MASTER_ID);
    assert(motor_runtime_abort_active_parameter_sequences(&runtime, 2050U) ==
           MOTOR_RUNTIME_STATUS_OK);
    preserved_discovery_result = runtime.discovery.results[0];
    preserved_feedback = runtime.bank.motors[0].feedback;
    preserved_accepted_feedback_count = runtime.accepted_feedback_count;
    preserved_rejected_feedback_count = runtime.rejected_feedback_count;
    assert(can_frame_init(&response_frame,
                          0x11U,
                          discovery_response_payload,
                          sizeof(discovery_response_payload)) ==
           CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &response_frame, 2100U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(memcmp(&runtime.discovery.results[0],
                  &preserved_discovery_result,
                  sizeof(preserved_discovery_result)) == 0);
    assert(memcmp(&runtime.bank.motors[0].feedback,
                  &preserved_feedback,
                  sizeof(preserved_feedback)) == 0);
    assert(runtime.accepted_feedback_count ==
           preserved_accepted_feedback_count);
    assert(runtime.rejected_feedback_count ==
           preserved_rejected_feedback_count);
    assert(motor_runtime_accept_frame(&runtime, &response_frame, 2101U) ==
           MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE);
    assert(runtime.rejected_feedback_count ==
           preserved_rejected_feedback_count + 1U);
    assert(motor_runtime_next_discovery_frame(&runtime,
                                              2200U,
                                              &request_frame) ==
           MOTOR_RUNTIME_STATUS_DISCOVERY_COMPLETE);

    prepare_runtime_with_two_discovered_motors(&runtime);
    runtime.discovery.results[0].observed_control_mode = 1U;
    assert(can_frame_init(&feedback_frame,
                          0x11U,
                          feedback_payload,
                          sizeof(feedback_payload)) == CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &feedback_frame, 3000U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x01U) == MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 3100U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 3101U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request_frame.data[3] == S3519_REGISTER_CONTROL_MODE);
    assert(motor_runtime_abort_active_parameter_sequences(&runtime, 3150U) ==
           MOTOR_RUNTIME_STATUS_OK);
    preserved_discovery_result = runtime.discovery.results[0];
    preserved_feedback = runtime.bank.motors[0].feedback;
    preserved_accepted_feedback_count = runtime.accepted_feedback_count;
    preserved_rejected_feedback_count = runtime.rejected_feedback_count;
    assert(can_frame_init(&response_frame,
                          0x11U,
                          mode_response_payload,
                          sizeof(mode_response_payload)) ==
           CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &response_frame, 3200U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(memcmp(&runtime.discovery.results[0],
                  &preserved_discovery_result,
                  sizeof(preserved_discovery_result)) == 0);
    assert(memcmp(&runtime.bank.motors[0].feedback,
                  &preserved_feedback,
                  sizeof(preserved_feedback)) == 0);
    assert(runtime.accepted_feedback_count ==
           preserved_accepted_feedback_count);
    assert(runtime.rejected_feedback_count ==
           preserved_rejected_feedback_count);
    assert(motor_runtime_accept_frame(&runtime, &response_frame, 3201U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(memcmp(&runtime.discovery.results[0],
                  &preserved_discovery_result,
                  sizeof(preserved_discovery_result)) == 0);
    assert(runtime.accepted_feedback_count ==
           preserved_accepted_feedback_count + 1U);
    assert(runtime.bank.motors[0].feedback.timestamp_us == 3201U);
    assert(runtime.rejected_feedback_count ==
           preserved_rejected_feedback_count);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 3300U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_WAITING);
}

/**
 * @brief Verifies quarantine gates a new request and expires after one timeout.
 */
static void test_motor_runtime_parameter_quarantine_is_bounded(void)
{
    MotorRuntime runtime;
    CanFrame request_frame;
    CanFrame late_frame;
    uint32_t accepted_feedback_count;
    uint8_t response_payload[8] = {
        1U,
        0U,
        0x33U,
        S3519_REGISTER_CONTROL_MODE,
        2U,
        0U,
        0U,
        0U
    };

    prepare_runtime_with_two_discovered_motors(&runtime);
    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x01U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 4000U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 4001U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(motor_runtime_abort_active_parameter_sequences(&runtime, 4050U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x01U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 4060U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_WAITING);
    assert(can_frame_init(&late_frame,
                          0x11U,
                          response_payload,
                          sizeof(response_payload)) == CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &late_frame, 4070U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(runtime.mode_switch_state == MOTOR_MODE_SWITCH_WRITING);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 4080U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_WAITING);
    response_payload[2] = 0x55U;
    assert(can_frame_init(&late_frame,
                          0x11U,
                          response_payload,
                          sizeof(response_payload)) == CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &late_frame, 4081U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 4082U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 4083U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);

    assert(motor_runtime_abort_active_parameter_sequences(&runtime, 4090U) ==
           MOTOR_RUNTIME_STATUS_OK);
    response_payload[2] = 0x33U;
    assert(can_frame_init(&late_frame,
                          0x11U,
                          response_payload,
                          sizeof(response_payload)) == CAN_FRAME_STATUS_OK);
    accepted_feedback_count = runtime.accepted_feedback_count;
    assert(motor_runtime_accept_frame(
               &runtime,
               &late_frame,
               4090U + MOTOR_DISCOVERY_REQUEST_TIMEOUT_US + 1U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(runtime.accepted_feedback_count == accepted_feedback_count + 1U);
}

/**
 * @brief Verifies early mode-write acknowledgements remain parameter traffic.
 */
static void test_motor_runtime_tracks_multiple_mode_parameter_expectations(void)
{
    MotorRuntime runtime;
    CanFrame request_frame;
    CanFrame response_frame;
    uint32_t accepted_feedback_count;
    uint32_t rejected_feedback_count;
    uint8_t response_payload[8] = {
        1U,
        0U,
        0x55U,
        S3519_REGISTER_CONTROL_MODE,
        2U,
        0U,
        0U,
        0U
    };

    prepare_runtime_with_two_discovered_motors(&runtime);
    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x05U) == MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 5000U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request_frame.data[0] == 1U);
    assert(request_frame.data[2] == 0x55U);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 5001U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request_frame.data[0] == 3U);
    assert(request_frame.data[2] == 0x55U);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 5002U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request_frame.data[0] == 1U);
    assert(request_frame.data[2] == 0x33U);

    accepted_feedback_count = runtime.accepted_feedback_count;
    rejected_feedback_count = runtime.rejected_feedback_count;
    assert(can_frame_init(&response_frame,
                          0x11U,
                          response_payload,
                          sizeof(response_payload)) == CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &response_frame, 5003U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(runtime.accepted_feedback_count == accepted_feedback_count);
    assert(runtime.rejected_feedback_count == rejected_feedback_count);
    assert(runtime.mode_switch_state == MOTOR_MODE_SWITCH_READ_WAITING);
    assert(runtime.parameter_expectations.count == 2U);
}

/**
 * @brief Verifies a bounded write acknowledgement stays parameter traffic
 *        after the corresponding readback has completed the mode sequence.
 */
static void test_motor_runtime_accepts_late_mode_write_ack_after_readback(void)
{
    MotorRuntime runtime;
    CanFrame request_frame;
    CanFrame response_frame;
    uint32_t accepted_feedback_count;
    uint32_t rejected_feedback_count;
    uint8_t response_payload[8] = {
        1U,
        0U,
        0x33U,
        S3519_REGISTER_CONTROL_MODE,
        2U,
        0U,
        0U,
        0U
    };

    prepare_runtime_with_two_discovered_motors(&runtime);
    assert(motor_runtime_begin_control_mode_switch_mask(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY,
               0x01U) == MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 5500U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 5501U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(can_frame_init(&response_frame,
                          0x11U,
                          response_payload,
                          sizeof(response_payload)) == CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &response_frame, 5502U) ==
           MOTOR_RUNTIME_STATUS_ACTION_COMPLETE);

    accepted_feedback_count = runtime.accepted_feedback_count;
    rejected_feedback_count = runtime.rejected_feedback_count;
    response_payload[2] = 0x55U;
    assert(can_frame_init(&response_frame,
                          0x11U,
                          response_payload,
                          sizeof(response_payload)) == CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &response_frame, 5503U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(runtime.accepted_feedback_count == accepted_feedback_count);
    assert(runtime.rejected_feedback_count == rejected_feedback_count);
    assert(runtime.parameter_expectations.count == 0U);
}

/**
 * @brief Verifies abort quarantines the maximum discovery-plus-mode burst.
 */
static void test_motor_runtime_quarantines_every_outstanding_parameter_response(void)
{
    MotorRuntime runtime;
    CanFrame request_frame;
    CanFrame response_frame;
    MotorDiscoveryResult preserved_discovery_result;
    MotorJointFeedback preserved_feedback[ARM_JOINT_COUNT];
    uint32_t accepted_feedback_count;
    uint32_t rejected_feedback_count;
    uint8_t joint_index;
    uint8_t discovery_response_payload[8] = {
        1U,
        0U,
        0x33U,
        S3519_REGISTER_MASTER_ID,
        0x11U,
        0U,
        0U,
        0U
    };
    uint8_t mode_response_payload[8] = {
        0U,
        0U,
        0x55U,
        S3519_REGISTER_CONTROL_MODE,
        2U,
        0U,
        0U,
        0U
    };

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    runtime.discovery.verified_joint_mask = 0x7FU;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        runtime.discovery.results[joint_index].verified_fields_mask =
            MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    }
    assert(motor_runtime_begin_control_mode_switch(
               &runtime,
               S3519_CONTROL_MODE_POSITION_VELOCITY) == MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_begin_discovery(&runtime, 0x01U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(motor_runtime_next_discovery_frame(&runtime,
                                              6000U,
                                              &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request_frame.data[3] == S3519_REGISTER_MASTER_ID);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        assert(motor_runtime_next_control_mode_frame(
                   &runtime,
                   (uint64_t)(6001U + joint_index),
                   &request_frame) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(request_frame.data[0] == (uint8_t)(joint_index + 1U));
        assert(request_frame.data[2] == 0x55U);
    }
    assert(motor_runtime_next_control_mode_frame(&runtime,
                                                 6010U,
                                                 &request_frame) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request_frame.data[0] == 1U);
    assert(request_frame.data[2] == 0x33U);
    assert(runtime.parameter_expectations.count ==
           MOTOR_RUNTIME_PARAMETER_EXPECTATION_CAPACITY);
    assert(motor_runtime_abort_active_parameter_sequences(&runtime, 6050U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(runtime.parameter_expectations.count == 0U);
    assert(runtime.parameter_quarantine.count ==
           MOTOR_RUNTIME_PARAMETER_EXPECTATION_CAPACITY);

    preserved_discovery_result = runtime.discovery.results[0];
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        preserved_feedback[joint_index] =
            runtime.bank.motors[joint_index].feedback;
    }
    accepted_feedback_count = runtime.accepted_feedback_count;
    rejected_feedback_count = runtime.rejected_feedback_count;

    assert(can_frame_init(&response_frame,
                          0x11U,
                          discovery_response_payload,
                          sizeof(discovery_response_payload)) ==
           CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &response_frame, 6060U) ==
           MOTOR_RUNTIME_STATUS_OK);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        mode_response_payload[0] = (uint8_t)(joint_index + 1U);
        assert(can_frame_init(&response_frame,
                              (uint16_t)(0x11U + joint_index),
                              mode_response_payload,
                              sizeof(mode_response_payload)) ==
               CAN_FRAME_STATUS_OK);
        assert(motor_runtime_accept_frame(
                   &runtime,
                   &response_frame,
                   (uint64_t)(6061U + joint_index)) == MOTOR_RUNTIME_STATUS_OK);
    }
    mode_response_payload[0] = 1U;
    mode_response_payload[2] = 0x33U;
    assert(can_frame_init(&response_frame,
                          0x11U,
                          mode_response_payload,
                          sizeof(mode_response_payload)) ==
           CAN_FRAME_STATUS_OK);
    assert(motor_runtime_accept_frame(&runtime, &response_frame, 6070U) ==
           MOTOR_RUNTIME_STATUS_OK);

    assert(runtime.accepted_feedback_count == accepted_feedback_count);
    assert(runtime.rejected_feedback_count == rejected_feedback_count);
    assert(runtime.parameter_quarantine.count == 0U);
    assert(memcmp(&runtime.discovery.results[0],
                  &preserved_discovery_result,
                  sizeof(preserved_discovery_result)) == 0);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        assert(memcmp(&runtime.bank.motors[joint_index].feedback,
                      &preserved_feedback[joint_index],
                      sizeof(preserved_feedback[joint_index])) == 0);
    }
}

/**
 * @brief Verifies seven control frames are accepted atomically or not at all.
 */
static void test_can_scheduler_accepts_atomic_seven_frame_groups(void)
{
    CanTxScheduler scheduler;
    CanFrame parameter_frame = make_test_frame(0x7FFU, 0x33U);
    CanFrame joint_frames[ARM_JOINT_COUNT];
    CanFrame popped_frame;
    CanTxPriority popped_priority;
    uint8_t frame_index;

    can_tx_scheduler_init(&scheduler);
    for (frame_index = 0U; frame_index < ARM_JOINT_COUNT; ++frame_index)
    {
        joint_frames[frame_index] = make_test_frame((uint16_t)(0x101U + frame_index),
                                                    (uint8_t)(0x10U + frame_index));
    }
    for (frame_index = 0U; frame_index < 26U; ++frame_index)
    {
        assert(can_tx_scheduler_submit(&scheduler,
                                       CAN_TX_PRIORITY_PARAMETER,
                                       &parameter_frame) == CAN_TX_SCHEDULER_STATUS_OK);
    }

    assert(can_tx_scheduler_submit_control_group(&scheduler,
                                                 joint_frames,
                                                 ARM_JOINT_COUNT) ==
           CAN_TX_SCHEDULER_STATUS_GROUP_REJECTED);
    assert(scheduler.count == 26U);
    assert(can_tx_scheduler_pop(&scheduler, &popped_frame, &popped_priority) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(scheduler.count == 25U);
    assert(can_tx_scheduler_submit_control_group(&scheduler,
                                                 joint_frames,
                                                 ARM_JOINT_COUNT) ==
           CAN_TX_SCHEDULER_STATUS_OK);
    assert(scheduler.count == CAN_TX_SCHEDULER_CAPACITY);

    for (frame_index = 0U; frame_index < ARM_JOINT_COUNT; ++frame_index)
    {
        assert(can_tx_scheduler_pop(&scheduler, &popped_frame, &popped_priority) ==
               CAN_TX_SCHEDULER_STATUS_OK);
        assert(popped_priority == CAN_TX_PRIORITY_JOINT_CONTROL);
        assert(popped_frame.data[0] == (uint8_t)(0x10U + frame_index));
    }
}

/**
 * @brief Verifies POS_VEL, mode, parameter-read, and MIT frames against vendor layout.
 */
static void test_s3519_command_encoding(void)
{
    static const uint8_t expected_midpoint_mit[8] = {
        0x7FU, 0xFFU, 0x7FU, 0xF7U, 0xFFU, 0x7FU, 0xF7U, 0xFFU
    };
    CanFrame frame;
    S3519Ranges ranges = {12.5F, 45.0F, 18.0F};

    assert(S3519_EXPLICIT_FEEDBACK_QUERY_VALIDATED == 0U);
    assert(s3519_pack_position_velocity(3U, 1.25F, 0.5F, &frame) ==
           S3519_CODEC_STATUS_OK);
    assert(frame.identifier == 0x103U);
    assert(frame.length == 8U);
    assert(memcmp(&frame.data[0], &(float){1.25F}, sizeof(float)) == 0);
    assert(memcmp(&frame.data[4], &(float){0.5F}, sizeof(float)) == 0);

    assert(s3519_pack_mode_command(3U,
                                    S3519_CONTROL_MODE_POSITION_VELOCITY,
                                    S3519_MODE_COMMAND_ENABLE,
                                    &frame) ==
           S3519_CODEC_STATUS_OK);
    assert(frame.identifier == 0x103U);
    assert(frame.data[7] == 0xFCU);

    assert(s3519_pack_parameter_read(3U, S3519_REGISTER_MASTER_ID, &frame) ==
           S3519_CODEC_STATUS_OK);
    assert(frame.identifier == 0x7FFU);
    assert(frame.length == 4U);
    assert(frame.data[0] == 3U);
    assert(frame.data[2] == 0x33U);
    assert(frame.data[3] == 0x07U);

    assert(s3519_pack_control_mode_write(3U, 2U, &frame) ==
           S3519_CODEC_STATUS_OK);
    assert(frame.identifier == 0x7FFU);
    assert(frame.length == 8U);
    assert(frame.data[2] == 0x55U);
    assert(frame.data[3] == S3519_REGISTER_CONTROL_MODE);
    assert(frame.data[4] == 2U);
    assert(frame.data[5] == 0U);

    assert(s3519_pack_feedback_query(3U, &frame) == S3519_CODEC_STATUS_OK);
    assert(frame.identifier == 0x7FFU);
    assert(frame.length == 4U);
    assert(frame.data[0] == 3U);
    assert(frame.data[2] == 0xCCU);
    assert(frame.data[3] == 0U);

    assert(s3519_pack_mit(3U,
                          &ranges,
                          0.0F,
                          0.0F,
                          250.0F,
                          2.5F,
                          0.0F,
                          &frame) == S3519_CODEC_STATUS_OK);
    assert(frame.identifier == 0x003U);
    assert(memcmp(frame.data, expected_midpoint_mit, sizeof(expected_midpoint_mit)) == 0);
}

/**
 * @brief Reinterprets one float as the raw little-endian register value.
 * @param value Floating-point register value.
 * @return Bit-identical uint32 value.
 */
static uint32_t float_to_raw_register(float value)
{
    uint32_t raw_value;

    memcpy(&raw_value, &value, sizeof(raw_value));
    return raw_value;
}

/**
 * @brief Supplies a deterministic valid raw value for one discovery register.
 * @param register_address Requested vendor register.
 * @param esc_id One-based motor identifier.
 * @return Raw uint32 payload value.
 */
static uint32_t make_discovery_raw_value(S3519Register register_address,
                                         uint8_t esc_id)
{
    switch (register_address)
    {
        case S3519_REGISTER_ACCELERATION:
            return float_to_raw_register(30.0F);
        case S3519_REGISTER_DECELERATION:
            return float_to_raw_register(25.0F);
        case S3519_REGISTER_MAXIMUM_SPEED:
            return float_to_raw_register(20.0F);
        case S3519_REGISTER_MASTER_ID:
            return (uint32_t)(esc_id + 0x10U);
        case S3519_REGISTER_ESC_ID:
            return esc_id;
        case S3519_REGISTER_CONTROL_MODE:
            return 2U;
        case S3519_REGISTER_HARDWARE_VERSION:
            return 0x00010002U;
        case S3519_REGISTER_SOFTWARE_VERSION:
            return 0x00030004U;
        case S3519_REGISTER_SUB_VERSION:
            return 0x00000005U;
        case S3519_REGISTER_POSITION_RANGE:
            return float_to_raw_register(12.5F);
        case S3519_REGISTER_VELOCITY_RANGE:
            return float_to_raw_register(45.0F);
        case S3519_REGISTER_TORQUE_RANGE:
            return float_to_raw_register(18.0F);
        default:
            assert(0);
            return 0U;
    }
}

/**
 * @brief Verifies discovery reads identity, tuning, ranges, and versions for seven motors.
 */
static void test_motor_discovery_verifies_every_joint(void)
{
    static const S3519Register expected_registers[MOTOR_DISCOVERY_REGISTER_COUNT] = {
        S3519_REGISTER_MASTER_ID,
        S3519_REGISTER_ESC_ID,
        S3519_REGISTER_CONTROL_MODE,
        S3519_REGISTER_ACCELERATION,
        S3519_REGISTER_DECELERATION,
        S3519_REGISTER_MAXIMUM_SPEED,
        S3519_REGISTER_HARDWARE_VERSION,
        S3519_REGISTER_SOFTWARE_VERSION,
        S3519_REGISTER_SUB_VERSION,
        S3519_REGISTER_POSITION_RANGE,
        S3519_REGISTER_VELOCITY_RANGE,
        S3519_REGISTER_TORQUE_RANGE
    };
    const ArmConfig *configuration = arm_config_get_production();
    MotorDiscovery discovery;
    uint64_t timestamp_us = 1000U;
    uint8_t joint_index;

    assert(motor_discovery_init(&discovery, configuration) == MOTOR_DISCOVERY_STATUS_OK);

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t register_index;

        for (register_index = 0U;
             register_index < MOTOR_DISCOVERY_REGISTER_COUNT;
             ++register_index)
        {
            CanFrame request_frame;
            S3519ParameterResponse response = {0};

            assert(motor_discovery_next_request(&discovery, timestamp_us, &request_frame) ==
                   MOTOR_DISCOVERY_STATUS_FRAME_READY);
            assert(request_frame.identifier == S3519_PARAMETER_COMMAND_IDENTIFIER);
            assert(request_frame.length == 4U);
            assert(request_frame.data[0] == (uint8_t)(joint_index + 1U));
            assert(request_frame.data[3] == (uint8_t)expected_registers[register_index]);

            response.esc_id = (uint16_t)(joint_index + 1U);
            response.register_address = (uint8_t)expected_registers[register_index];
            switch (expected_registers[register_index])
            {
                case S3519_REGISTER_ACCELERATION:
                    response.raw_value = float_to_raw_register(30.0F);
                    response.float_value = 30.0F;
                    break;
                case S3519_REGISTER_DECELERATION:
                    response.raw_value = float_to_raw_register(-25.0F);
                    response.float_value = -25.0F;
                    break;
                case S3519_REGISTER_MAXIMUM_SPEED:
                    response.raw_value = float_to_raw_register(20.0F);
                    response.float_value = 20.0F;
                    break;
                case S3519_REGISTER_MASTER_ID:
                    response.raw_value = (uint32_t)(joint_index + 0x11U);
                    break;
                case S3519_REGISTER_ESC_ID:
                    response.raw_value = (uint32_t)(joint_index + 1U);
                    break;
                case S3519_REGISTER_CONTROL_MODE:
                    response.raw_value = 2U;
                    break;
                case S3519_REGISTER_HARDWARE_VERSION:
                    response.raw_value = 0x00010002U;
                    break;
                case S3519_REGISTER_SOFTWARE_VERSION:
                    response.raw_value = 0x00030004U;
                    break;
                case S3519_REGISTER_SUB_VERSION:
                    response.raw_value = 0x00000005U;
                    break;
                case S3519_REGISTER_POSITION_RANGE:
                    response.raw_value = float_to_raw_register(12.5F);
                    response.float_value = 12.5F;
                    break;
                case S3519_REGISTER_VELOCITY_RANGE:
                    response.raw_value = float_to_raw_register(45.0F);
                    response.float_value = 45.0F;
                    break;
                case S3519_REGISTER_TORQUE_RANGE:
                    response.raw_value = float_to_raw_register(18.0F);
                    response.float_value = 18.0F;
                    break;
                default:
                    assert(0);
                    break;
            }

            assert(motor_discovery_accept_response(&discovery,
                                                    (uint16_t)(joint_index + 0x11U),
                                                    &response) == MOTOR_DISCOVERY_STATUS_OK);
            timestamp_us += 1000U;
        }

        assert(discovery.results[joint_index].acceleration_rad_s2 == 30.0F);
        assert(discovery.results[joint_index].deceleration_rad_s2 == 25.0F);
        assert(discovery.results[joint_index].maximum_speed_rad_s == 20.0F);
        assert(discovery.results[joint_index].hardware_version == 0x00010002U);
        assert(discovery.results[joint_index].software_version == 0x00030004U);
        assert(discovery.results[joint_index].sub_version == 0x00000005U);
    }

    assert(discovery.state == MOTOR_DISCOVERY_STATE_COMPLETE);
    assert(discovery.verified_joint_mask == 0x7FU);
    assert(motor_discovery_next_request(&discovery, timestamp_us, &(CanFrame){0}) ==
           MOTOR_DISCOVERY_STATUS_COMPLETE);
}

/**
 * @brief Verifies bench discovery skips every motor outside an explicit mask.
 */
static void test_motor_discovery_targets_selected_subset(void)
{
    MotorDiscovery discovery;
    uint64_t timestamp_us = 1000U;
    uint8_t response_count = 0U;

    assert(motor_discovery_init(&discovery, arm_config_get_production()) ==
           MOTOR_DISCOVERY_STATUS_OK);
    assert(motor_discovery_begin(&discovery, 0x05U) ==
           MOTOR_DISCOVERY_STATUS_OK);
    while (discovery.state != MOTOR_DISCOVERY_STATE_COMPLETE)
    {
        CanFrame request_frame;
        S3519ParameterResponse response = {0};
        uint8_t esc_id;
        S3519Register register_address;

        assert(motor_discovery_next_request(&discovery,
                                            timestamp_us,
                                            &request_frame) ==
               MOTOR_DISCOVERY_STATUS_FRAME_READY);
        esc_id = request_frame.data[0];
        register_address = (S3519Register)request_frame.data[3];
        assert((esc_id == 1U) || (esc_id == 3U));
        response.esc_id = esc_id;
        response.register_address = (uint8_t)register_address;
        response.raw_value = make_discovery_raw_value(register_address, esc_id);
        memcpy(&response.float_value,
               &response.raw_value,
               sizeof(response.float_value));
        assert(motor_discovery_accept_response(
                   &discovery,
                   (uint16_t)(esc_id + 0x10U),
                   &response) == MOTOR_DISCOVERY_STATUS_OK);
        ++response_count;
        timestamp_us += 1000U;
    }
    assert(response_count == (2U * MOTOR_DISCOVERY_REGISTER_COUNT));
    assert(discovery.target_joint_mask == 0x05U);
    assert(discovery.verified_joint_mask == 0x05U);
    assert(discovery.results[0].verified_fields_mask ==
           MOTOR_DISCOVERY_ALL_FIELDS_MASK);
    assert(discovery.results[1].verified_fields_mask == 0U);
    assert(discovery.results[2].verified_fields_mask ==
           MOTOR_DISCOVERY_ALL_FIELDS_MASK);
}

/**
 * @brief Verifies identity mismatches latch discovery failure before enable.
 */
static void test_motor_discovery_rejects_mapping_mismatch(void)
{
    MotorDiscovery discovery;
    CanFrame request_frame;
    S3519ParameterResponse response = {0};

    assert(motor_discovery_init(&discovery, arm_config_get_production()) ==
           MOTOR_DISCOVERY_STATUS_OK);
    assert(motor_discovery_next_request(&discovery, 1000U, &request_frame) ==
           MOTOR_DISCOVERY_STATUS_FRAME_READY);
    response.esc_id = 1U;
    response.register_address = S3519_REGISTER_MASTER_ID;
    response.raw_value = 0x21U;
    assert(motor_discovery_accept_response(&discovery, 0x11U, &response) ==
           MOTOR_DISCOVERY_STATUS_CONFIG_MISMATCH);
    assert(discovery.state == MOTOR_DISCOVERY_STATE_FAILED);
}

/**
 * @brief Verifies an absent motor exhausts a finite retry budget and fails closed.
 */
static void test_motor_discovery_times_out_without_response(void)
{
    MotorDiscovery discovery;
    CanFrame request_frame;
    uint8_t attempt_index;

    assert(motor_discovery_init(&discovery, arm_config_get_production()) ==
           MOTOR_DISCOVERY_STATUS_OK);
    for (attempt_index = 0U; attempt_index < MOTOR_DISCOVERY_MAX_ATTEMPTS; ++attempt_index)
    {
        uint64_t timestamp_us =
            (uint64_t)attempt_index * MOTOR_DISCOVERY_REQUEST_TIMEOUT_US;

        assert(motor_discovery_next_request(&discovery, timestamp_us, &request_frame) ==
               MOTOR_DISCOVERY_STATUS_FRAME_READY);
    }

    assert(motor_discovery_next_request(
               &discovery,
               (uint64_t)MOTOR_DISCOVERY_MAX_ATTEMPTS *
                   MOTOR_DISCOVERY_REQUEST_TIMEOUT_US,
               &request_frame) == MOTOR_DISCOVERY_STATUS_TIMEOUT);
    assert(discovery.state == MOTOR_DISCOVERY_STATE_FAILED);
}

/**
 * @brief Verifies task-facing runtime discovery and feedback decoding end to end.
 */
static void test_motor_runtime_routes_discovery_and_feedback(void)
{
    MotorRuntime runtime;
    MotorFeedbackSnapshot snapshot;
    uint64_t timestamp_us = 1000U;
    uint16_t response_index;

    assert(motor_runtime_init(&runtime, arm_config_get_production()) ==
           MOTOR_RUNTIME_STATUS_OK);
    for (response_index = 0U;
         response_index < (uint16_t)(ARM_JOINT_COUNT * MOTOR_DISCOVERY_REGISTER_COUNT);
         ++response_index)
    {
        CanFrame request_frame;
        CanFrame response_frame;
        uint8_t response_payload[8] = {0U};
        uint8_t esc_id;
        uint32_t raw_value;

        assert(motor_runtime_next_discovery_frame(&runtime,
                                                  timestamp_us,
                                                  &request_frame) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        esc_id = request_frame.data[0];
        raw_value = make_discovery_raw_value(
            (S3519Register)request_frame.data[3],
            esc_id);
        response_payload[0] = esc_id;
        response_payload[2] = 0x33U;
        response_payload[3] = request_frame.data[3];
        response_payload[4] = (uint8_t)(raw_value & 0xFFU);
        response_payload[5] = (uint8_t)((raw_value >> 8U) & 0xFFU);
        response_payload[6] = (uint8_t)((raw_value >> 16U) & 0xFFU);
        response_payload[7] = (uint8_t)((raw_value >> 24U) & 0xFFU);
        assert(can_frame_init(&response_frame,
                              (uint16_t)(esc_id + 0x10U),
                              response_payload,
                              sizeof(response_payload)) == CAN_FRAME_STATUS_OK);
        assert(motor_runtime_accept_frame(&runtime,
                                          &response_frame,
                                          timestamp_us + 100U) ==
               MOTOR_RUNTIME_STATUS_OK);
        timestamp_us += 1000U;
    }
    assert(motor_runtime_next_discovery_frame(&runtime,
                                              timestamp_us,
                                              &(CanFrame){0}) ==
           MOTOR_RUNTIME_STATUS_DISCOVERY_COMPLETE);
    for (response_index = 0U; response_index < ARM_JOINT_COUNT; ++response_index)
    {
        assert(runtime.bank.motors[response_index].state ==
               MOTOR_LIFECYCLE_DISABLED);
        assert(runtime.bank.motors[response_index].configuration_consistent != 0U);
        assert(runtime.bank.motors[response_index].parameter_valid_mask ==
               MOTOR_DISCOVERY_ALL_FIELDS_MASK);
    }

    {
        static const uint8_t feedback_payload[8] = {
            0x01U, 0x80U, 0x00U, 0x80U, 0x08U, 0x00U, 42U, 40U
        };
        CanFrame feedback_frame;

        assert(can_frame_init(&feedback_frame,
                              0x11U,
                              feedback_payload,
                              sizeof(feedback_payload)) == CAN_FRAME_STATUS_OK);
        assert(motor_runtime_accept_frame(&runtime,
                                          &feedback_frame,
                                          timestamp_us) ==
               MOTOR_RUNTIME_STATUS_OK);
    }
    assert(motor_runtime_get_snapshot(&runtime,
                                      timestamp_us,
                                      MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US,
                                      &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(snapshot.valid_joint_mask == 0x01U);
    assert(snapshot.joints[0].mos_temperature_c == 42.0F);
    assert(snapshot.joints[0].rotor_temperature_c == 40.0F);
    assert(snapshot.joints[0].fault_flags == 0U);
    assert(runtime.bank.motors[0].state == MOTOR_LIFECYCLE_DISABLED);

    {
        static const uint8_t enabled_payload[8] = {
            0x11U, 0x80U, 0x00U, 0x80U, 0x08U, 0x00U, 42U, 40U
        };
        CanFrame enabled_frame;

        assert(can_frame_init(&enabled_frame,
                              0x11U,
                              enabled_payload,
                              sizeof(enabled_payload)) == CAN_FRAME_STATUS_OK);
        assert(motor_runtime_accept_frame(&runtime,
                                          &enabled_frame,
                                          timestamp_us + 1U) ==
               MOTOR_RUNTIME_STATUS_OK);
        assert(runtime.bank.motors[0].state == MOTOR_LIFECYCLE_ENABLED);
        assert(runtime.bank.motors[0].feedback.fault_flags == 0U);
    }

    {
        static const uint8_t fault_payload[8] = {
            0x81U, 0x80U, 0x00U, 0x80U, 0x08U, 0x00U, 42U, 40U
        };
        CanFrame fault_frame;

        assert(can_frame_init(&fault_frame,
                              0x11U,
                              fault_payload,
                              sizeof(fault_payload)) == CAN_FRAME_STATUS_OK);
        assert(motor_runtime_accept_frame(&runtime,
                                          &fault_frame,
                                          timestamp_us + 2U) ==
               MOTOR_RUNTIME_STATUS_OK);
        assert(runtime.bank.motors[0].state == MOTOR_LIFECYCLE_FAULT);
        assert(runtime.bank.motors[0].feedback.fault_flags == 8U);
    }
    assert(runtime.accepted_feedback_count == 3U);
    assert(runtime.accepted_parameter_response_count ==
           (uint32_t)(ARM_JOINT_COUNT * MOTOR_DISCOVERY_REGISTER_COUNT));
}

/**
 * @brief Runs all seven-motor core tests.
 * @return Zero when every assertion passes.
 */
int main(void)
{
    test_motor_runtime_reports_discovered_position_velocity_limits();
    test_motor_runtime_accepts_valid_position_velocity_move_limits();
    test_motor_runtime_rejects_invalid_position_velocity_moves();
    test_motor_runtime_rejects_missing_or_faulted_move_feedback();
    test_motor_runtime_builds_position_velocity_subset_atomically();
    test_motor_bank_uses_frozen_seven_axis_mapping();
    test_motor_bank_publishes_coherent_feedback_snapshots();
    test_motor_bank_rejects_snapshot_during_publish();
    test_can_scheduler_prioritizes_emergency_frames();
    test_can_scheduler_reserves_progress_for_emergency_frames();
    test_motor_runtime_builds_fail_safe_disable_batch();
    test_motor_runtime_builds_selected_unknown_mode_disable_batch();
    test_motor_runtime_builds_selected_known_mode_disable_batch();
    test_motor_runtime_rejects_invalid_emergency_disable_subset();
    test_motor_runtime_switches_mode_with_readback();
    test_motor_runtime_masked_mode_switch_serializes_unselected_discovery();
    test_motor_runtime_masked_mode_switch_requires_selected_readiness();
    test_motor_runtime_aborts_active_parameter_sequences();
    test_motor_runtime_discards_late_parameter_responses_after_abort();
    test_motor_runtime_parameter_quarantine_is_bounded();
    test_motor_runtime_tracks_multiple_mode_parameter_expectations();
    test_motor_runtime_accepts_late_mode_write_ack_after_readback();
    test_motor_runtime_quarantines_every_outstanding_parameter_response();
    test_can_scheduler_accepts_atomic_seven_frame_groups();
    test_s3519_command_encoding();
    test_motor_discovery_verifies_every_joint();
    test_motor_discovery_targets_selected_subset();
    test_motor_discovery_rejects_mapping_mismatch();
    test_motor_discovery_times_out_without_response();
    test_motor_runtime_routes_discovery_and_feedback();
    puts("MOTOR_CORE_TESTS_PASSED");
    return 0;
}
