/**
 * @file text_protocol_arm_profile_test_main.c
 * @brief Verifies aethor-text-v1 formal seven-axis command admission.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol_engine.h"

/** @brief Builds a fully verified configuration for host-only admission tests. */
static ArmConfig make_verified_configuration(void)
{
    ArmConfig configuration = *arm_config_get_production();
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        JointConfig *joint = &configuration.joints[joint_index];

        joint->direction = 1;
        joint->soft_limit_min_rad = -3.0F;
        joint->soft_limit_max_rad = 3.0F;
        joint->max_velocity_rad_s = 1.0F;
        joint->max_acceleration_rad_s2 = 2.0F;
        joint->mit_kp = 10.0F;
        joint->mit_kd = 1.0F;
        joint->motor_pmax_rad = 12.5F;
        joint->motor_vmax_rad_s = 45.0F;
        joint->motor_tmax_nm = 18.0F;
        joint->gear_ratio = 1.0F;
        joint->position_tolerance_rad = 0.01F;
        joint->velocity_tolerance_rad_s = 0.02F;
        joint->verified_fields = ARM_JOINT_REQUIRED_ENABLE_FIELDS;
    }
    return configuration;
}

/** @brief Builds a fully ready arm snapshot for formal motion admission. */
static ProtocolQueryContext make_ready_context(void)
{
    ProtocolQueryContext query_context;

    memset(&query_context, 0, sizeof(query_context));
    query_context.arm.state = ARM_STATE_READY;
    query_context.arm.fault = ARM_FAULT_NONE;
    query_context.arm.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
    query_context.arm.aligned = 1U;
    query_context.arm.enabled = 1U;
    query_context.joints.aligned = 1U;
    query_context.joints.valid_joint_mask = 0x7FU;
    query_context.motors.valid_joint_mask = 0x7FU;
    query_context.motor_identity_verified_mask = 0x7FU;
    query_context.motor_mode_verified_mask = 0x7FU;
    query_context.motor_ranges_verified_mask = 0x7FU;
    query_context.motor_version_verified_mask = 0x7FU;
    return query_context;
}

/** @brief Processes one formal-profile request and returns its single output. */
static const char *process_arm_request(ProtocolEngine *engine,
                                       const char *request,
                                       uint64_t timestamp_us,
                                       ProtocolEngineStatus expected_status,
                                       ProtocolOutputBatch *output_batch)
{
    ProtocolEngineStatus status = protocol_engine_process_text_line(
        engine, request, strlen(request), timestamp_us, output_batch);

    assert(status == expected_status);
    assert(output_batch->count == 1U);
    return output_batch->messages[0].data;
}

/** @brief Verifies compile-time profile identity and cross-profile rejection. */
static void test_arm_profile_identity(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    const char *response;

    protocol_engine_init(&engine, 1001U);
    response = process_arm_request(&engine,
                                   "1 hello\n",
                                   1000U,
                                   PROTOCOL_ENGINE_STATUS_OK,
                                   &output_batch);
    assert(strstr(response, " profile=arm ") != NULL);

    response = process_arm_request(&engine,
                                   "2 bench enable 1\n",
                                   2000U,
                                   PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                   &output_batch);
    assert(strcmp(response,
                  "error 2 bench enable code=profile current=arm required=bench\n") == 0);
}

/** @brief Verifies align, enable, move, stop, disable, and clear queue mapping. */
static void test_arm_command_mapping(void)
{
    ArmConfig configuration = make_verified_configuration();
    ProtocolQueryContext query_context = make_ready_context();
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolCommand command;
    const char *response;

    protocol_engine_init(&engine, 1002U);
    protocol_engine_set_configuration(&engine, &configuration);

    query_context.arm.state = ARM_STATE_UNALIGNED;
    query_context.arm.aligned = 0U;
    query_context.arm.enabled = 0U;
    query_context.joints.aligned = 0U;
    protocol_engine_update_query_context(&engine, &query_context);
    response = process_arm_request(&engine,
                                   "10 arm align 0,0,90,0,0,0,0\n",
                                   1000U,
                                   PROTOCOL_ENGINE_STATUS_OK,
                                   &output_batch);
    assert(strcmp(response, "ok 10 arm align accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_ALIGN_REFERENCE);
    assert(command.values[2] == 90.0F);

    query_context = make_ready_context();
    query_context.arm.state = ARM_STATE_DISABLED;
    query_context.arm.enabled = 0U;
    protocol_engine_update_query_context(&engine, &query_context);
    response = process_arm_request(&engine,
                                   "11 arm enable\n",
                                   2000U,
                                   PROTOCOL_ENGINE_STATUS_OK,
                                   &output_batch);
    assert(strcmp(response, "ok 11 arm enable accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_ENABLE);
    assert(command.control_mode == ARM_CONTROL_MODE_POSITION_VELOCITY);

    query_context = make_ready_context();
    protocol_engine_update_query_context(&engine, &query_context);
    response = process_arm_request(&engine,
                                   "12 arm move 0,-15,30,0,20,0,5 speed=5\n",
                                   3000U,
                                   PROTOCOL_ENGINE_STATUS_OK,
                                   &output_batch);
    assert(strcmp(response, "ok 12 arm move accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_MOVE_JOINTS);
    assert(command.control_mode == ARM_CONTROL_MODE_POSITION_VELOCITY);
    assert(command.values[1] == -15.0F);
    assert(command.values[2] == 30.0F);
    assert(command.speeds[0] > 0.0F);
    assert(command.speeds[0] <= 1.0F);

    response = process_arm_request(&engine,
                                   "13 arm stop\n",
                                   4000U,
                                   PROTOCOL_ENGINE_STATUS_OK,
                                   &output_batch);
    assert(strcmp(response, "ok 13 arm stop accepted=1\n") == 0);
    assert(protocol_engine_pop_stop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_STOP);

    response = process_arm_request(&engine,
                                   "14 arm disable\n",
                                   5000U,
                                   PROTOCOL_ENGINE_STATUS_OK,
                                   &output_batch);
    assert(strcmp(response, "ok 14 arm disable accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_DISABLE);
}

/** @brief Verifies malformed motion and unavailable configuration are rejected. */
static void test_arm_command_rejections(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolQueryContext query_context = make_ready_context();
    const char *response;

    protocol_engine_init(&engine, 1003U);
    protocol_engine_update_query_context(&engine, &query_context);
    response = process_arm_request(&engine,
                                   "20 arm move 0,0,0 speed=5\n",
                                   1000U,
                                   PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                   &output_batch);
    assert(strcmp(response,
                  "error 20 arm move code=bad_argument field=q\n") == 0);

    query_context.arm.state = ARM_STATE_DISABLED;
    query_context.arm.enabled = 0U;
    protocol_engine_update_query_context(&engine, &query_context);
    response = process_arm_request(&engine,
                                   "21 arm enable\n",
                                   2000U,
                                   PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                   &output_batch);
    assert(strcmp(response,
                  "error 21 arm enable code=not_ready detail=config\n") == 0);
}

/** @brief Verifies formal motion completion replaces ACK with readable DONE. */
static void test_arm_done_output(void)
{
    ArmConfig configuration = make_verified_configuration();
    ProtocolQueryContext query_context = make_ready_context();
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolCommand command;
    ProtocolCommandResult result;

    protocol_engine_init(&engine, 1004U);
    protocol_engine_set_configuration(&engine, &configuration);
    protocol_engine_update_query_context(&engine, &query_context);
    (void)process_arm_request(&engine,
                              "30 arm move 0,-15,30,0,20,0,5 speed=5\n",
                              1000U,
                              PROTOCOL_ENGINE_STATUS_OK,
                              &output_batch);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    memset(&result, 0, sizeof(result));
    result.request_id = command.request_id;
    result.session_id = command.session_id;
    result.type = command.type;
    result.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
    result.accepted_at_us = command.accepted_at_us;
    result.completed_at_us = 2951000U;
    result.auxiliary_values[0] = 0.18F;
    assert(protocol_engine_submit_command_result(&engine, &result) != 0U);
    assert(protocol_engine_pop_result_output(&engine, &output_batch) != 0U);
    assert(strcmp(output_batch.messages[0].data,
                  "done 30 arm move result=completed elapsed_ms=2950 "
                  "max_error_deg=0.18\n") == 0);
}

/** @brief Runs formal arm-profile text protocol tests. */
int main(void)
{
    test_arm_profile_identity();
    test_arm_command_mapping();
    test_arm_command_rejections();
    test_arm_done_output();
    puts("TEXT_PROTOCOL_ARM_PROFILE_TESTS_PASSED");
    return 0;
}
