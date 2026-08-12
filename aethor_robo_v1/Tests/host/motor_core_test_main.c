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
    assert(frame.data[0] == 3U);
    assert(frame.data[2] == 0x33U);
    assert(frame.data[3] == 0x07U);

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
                    response.raw_value = float_to_raw_register(25.0F);
                    response.float_value = 25.0F;
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
    assert(runtime.accepted_feedback_count == 1U);
    assert(runtime.accepted_parameter_response_count ==
           (uint32_t)(ARM_JOINT_COUNT * MOTOR_DISCOVERY_REGISTER_COUNT));
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
    test_can_scheduler_accepts_atomic_seven_frame_groups();
    test_s3519_command_encoding();
    test_motor_discovery_verifies_every_joint();
    test_motor_discovery_rejects_mapping_mismatch();
    test_motor_discovery_times_out_without_response();
    test_motor_runtime_routes_discovery_and_feedback();
    puts("MOTOR_CORE_TESTS_PASSED");
    return 0;
}
