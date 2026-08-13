/**
 * @file aethor_app.c
 * @brief Implements the allocation-free Phase 0 application facade.
 */

#include "aethor_app.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "arm_config.h"
#include "joint_motion_can.h"

#define AETHOR_APP_ACTION_TIMEOUT_US (500000ULL)
#define AETHOR_APP_DISCOVERY_TIMEOUT_US (8000000ULL)
#define AETHOR_APP_MOTION_SETTLE_US (200000ULL)
#define AETHOR_APP_MOTION_TIMEOUT_MARGIN_US (2000000ULL)
#define AETHOR_APP_ALL_JOINTS_MASK ((uint8_t)0x7FU)
#define AETHOR_APP_DEG_TO_RAD (0.017453292519943295F)

/** @brief Describes one asynchronous motor-backed action owned by ArmControlTask. */
typedef enum
{
    AETHOR_APP_ACTION_IDLE = 0,
    AETHOR_APP_ACTION_MODE_SWITCH,
    AETHOR_APP_ACTION_DISCOVERY,
    AETHOR_APP_ACTION_BENCH_MODE_SWITCH,
    AETHOR_APP_ACTION_ENABLE_WAIT,
    AETHOR_APP_ACTION_BENCH_MOVE_WAIT,
    AETHOR_APP_ACTION_DISABLE_WAIT,
    AETHOR_APP_ACTION_CLEAR_FAULT_WAIT,
    AETHOR_APP_ACTION_MOTION,
    AETHOR_APP_ACTION_CONTROLLED_STOP
} AethorAppActionState;

/** @brief Owns the active protocol command and its bounded CAN frame batch. */
typedef struct
{
    ProtocolCommand command;
    MotorEmergencyFrameBatch frames;
    JointMotionPlan motion_plan;
    JointControlledStopPlan controlled_stop_plan;
    JointMotionCompletion motion_completion;
    CanFrame control_group[ARM_JOINT_COUNT];
    uint64_t deadline_us;
    float maximum_following_error_deg;
    AethorAppActionState state;
    CanTxPriority priority;
    uint8_t frame_read_index;
    uint8_t control_group_ready;
} AethorAppAction;

static Diagnostics application_diagnostics;
static ArmController application_controller;
static JointReference application_joint_reference;
static MotorRuntime application_motor_runtime;
static ProtocolEngine application_protocol_engine;
static MotorEmergencyFrameBatch application_emergency_disable_batch;
static uint8_t application_emergency_disable_read_index;
static AethorAppAction application_action;
static uint64_t application_last_service_timestamp_us;
static uint8_t application_initialized;

/** @brief Maps the public arm mode to the vendor identifier offset. */
static S3519ControlMode aethor_app_vendor_mode(ArmControlMode control_mode)
{
    return (control_mode == ARM_CONTROL_MODE_MIT)
               ? S3519_CONTROL_MODE_MIT
               : S3519_CONTROL_MODE_POSITION_VELOCITY;
}

/** @brief Completes the active command and clears its execution gate. */
static uint8_t aethor_app_complete_action(ProtocolCommandResultCode code,
                                          uint16_t detail,
                                          uint64_t timestamp_us)
{
    ProtocolCommandResult result;

    memset(&result, 0, sizeof(result));
    result.request_id = application_action.command.request_id;
    result.session_id = application_action.command.session_id;
    result.type = application_action.command.type;
    result.code = code;
    result.detail = detail;
    result.accepted_at_us = application_action.command.accepted_at_us;
    result.completed_at_us = timestamp_us;
    result.motor_mask = application_action.command.motor_mask;
    result.bench_relative_scope =
        application_action.command.bench_relative_scope;
    if (application_action.command.type == PROTOCOL_COMMAND_MOVE_JOINTS)
    {
        result.auxiliary_values[0] =
            application_action.maximum_following_error_deg;
    }
    memset(&application_action, 0, sizeof(application_action));
    application_last_service_timestamp_us = 0U;
    return protocol_engine_submit_command_result(&application_protocol_engine,
                                                 &result);
}

/** @brief Accumulates the maximum absolute trajectory-following error in degrees. */
static void aethor_app_update_following_error(
    const float feedback_position_rad[ARM_JOINT_COUNT],
    uint64_t timestamp_us)
{
    JointMotionSample desired_sample;
    uint8_t joint_index;

    if ((feedback_position_rad == NULL) ||
        (joint_motion_sample(&application_action.motion_plan,
                             timestamp_us,
                             &desired_sample) != JOINT_MOTION_STATUS_OK))
    {
        return;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        float error_deg = fabsf(desired_sample.position_rad[joint_index] -
                                feedback_position_rad[joint_index]) /
                          AETHOR_APP_DEG_TO_RAD;

        if (isfinite(error_deg) &&
            (error_deg > application_action.maximum_following_error_deg))
        {
            application_action.maximum_following_error_deg = error_deg;
        }
    }
}

/** @brief Converts one coherent public joint snapshot back to SI radians. */
static uint8_t aethor_app_joint_snapshot_to_radians(
    const JointStateSnapshot *snapshot,
    float position_rad[ARM_JOINT_COUNT],
    float velocity_rad_s[ARM_JOINT_COUNT])
{
    uint8_t joint_index;

    if ((snapshot == NULL) || (position_rad == NULL) ||
        (velocity_rad_s == NULL) || (snapshot->aligned == 0U) ||
        (snapshot->valid_joint_mask != AETHOR_APP_ALL_JOINTS_MASK))
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        position_rad[joint_index] =
            snapshot->position_deg[joint_index] * AETHOR_APP_DEG_TO_RAD;
        velocity_rad_s[joint_index] =
            snapshot->velocity_deg_s[joint_index] * AETHOR_APP_DEG_TO_RAD;
    }
    return 1U;
}

/** @brief Builds the next complete seven-frame motion control group. */
static JointMotionCanStatus aethor_app_prepare_motion_group(
    uint64_t timestamp_us)
{
    JointMotionSample sample;

    if (joint_motion_sample(&application_action.motion_plan,
                            timestamp_us,
                            &sample) != JOINT_MOTION_STATUS_OK)
    {
        return JOINT_MOTION_CAN_STATUS_INVALID_ARGUMENT;
    }
    if (joint_motion_can_pack_group(&application_action.motion_plan,
                                    &sample,
                                    &application_joint_reference,
                                    &application_motor_runtime,
                                    application_action.control_group) !=
        JOINT_MOTION_CAN_STATUS_OK)
    {
        return JOINT_MOTION_CAN_STATUS_CODEC_ERROR;
    }
    application_action.control_group_ready = 1U;
    return JOINT_MOTION_CAN_STATUS_OK;
}

/** @brief Builds the next controlled-stop group for the confirmed motor mode. */
static JointMotionCanStatus aethor_app_prepare_controlled_stop_group(
    uint64_t timestamp_us)
{
    JointMotionPlan encoding_plan;
    JointMotionSample sample;
    uint8_t joint_index;

    memset(&encoding_plan, 0, sizeof(encoding_plan));
    encoding_plan.mode =
        (application_action.command.control_mode == ARM_CONTROL_MODE_MIT)
            ? JOINT_MOTION_MODE_MIT
            : JOINT_MOTION_MODE_POSITION_VELOCITY;
    if (encoding_plan.mode == JOINT_MOTION_MODE_POSITION_VELOCITY)
    {
        memset(&sample, 0, sizeof(sample));
        sample.timestamp_us = timestamp_us;
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            sample.position_rad[joint_index] = application_action
                                                    .controlled_stop_plan
                                                    .hold_position_rad[joint_index];
            sample.velocity_rad_s[joint_index] = application_action
                                                     .controlled_stop_plan
                                                     .start_velocity_rad_s[joint_index];
        }
    }
    else if (joint_motion_sample_controlled_stop(
                 &application_action.controlled_stop_plan,
                 timestamp_us,
                 &sample) != JOINT_MOTION_STATUS_OK)
    {
        return JOINT_MOTION_CAN_STATUS_INVALID_ARGUMENT;
    }
    if (joint_motion_can_pack_group(&encoding_plan,
                                    &sample,
                                    &application_joint_reference,
                                    &application_motor_runtime,
                                    application_action.control_group) !=
        JOINT_MOTION_CAN_STATUS_OK)
    {
        return JOINT_MOTION_CAN_STATUS_CODEC_ERROR;
    }
    application_action.control_group_ready = 1U;
    return JOINT_MOTION_CAN_STATUS_OK;
}

/** @brief Starts fail-safe disable frames after a motion-control failure. */
static void aethor_app_latch_motion_failure(uint16_t detail,
                                            uint64_t timestamp_us)
{
    (void)arm_controller_latch_runtime_fault(&application_controller,
                                             ARM_FAULT_MOTION_CONTROL,
                                             detail,
                                             timestamp_us);
    application_emergency_disable_read_index = 0U;
    (void)motor_runtime_build_emergency_disable(
        &application_motor_runtime,
        &application_emergency_disable_batch);
}

/** @brief Returns the first formal-arm driver fault detail, or zero. */
static uint32_t aethor_app_first_driver_fault_detail(
    const MotorFeedbackSnapshot *snapshot)
{
    uint8_t joint_index;

    if (snapshot == NULL)
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (snapshot->joints[joint_index].fault_flags != 0U)
        {
            return ((uint32_t)(joint_index + 1U) << 24) |
                   (snapshot->joints[joint_index].fault_flags & 0x00FFFFFFUL);
        }
    }
    return 0U;
}

/** @brief Checks one selected feedback field against an exact driver state. */
static uint8_t aethor_app_selected_motors_have_state(
    const MotorFeedbackSnapshot *snapshot,
    uint8_t motor_mask,
    uint8_t driver_state)
{
    uint8_t joint_index;

    if ((snapshot->valid_joint_mask & motor_mask) != motor_mask)
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (((motor_mask & (uint8_t)(1U << joint_index)) != 0U) &&
            (snapshot->joints[joint_index].driver_state != driver_state))
        {
            return 0U;
        }
    }
    return 1U;
}

/** @brief Checks that selected fresh feedback no longer reports a driver fault. */
static uint8_t aethor_app_selected_motors_are_fault_free(
    const MotorFeedbackSnapshot *snapshot,
    uint8_t motor_mask)
{
    uint8_t joint_index;

    if ((snapshot->valid_joint_mask & motor_mask) != motor_mask)
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (((motor_mask & (uint8_t)(1U << joint_index)) != 0U) &&
            (snapshot->joints[joint_index].fault_flags != 0U))
        {
            return 0U;
        }
    }
    return 1U;
}

/** @brief Refreshes the protocol query context from coherent domain snapshots. */
static void aethor_app_update_protocol_context(uint64_t timestamp_us)
{
    ProtocolQueryContext query_context;
    uint8_t joint_index;

    memset(&query_context, 0, sizeof(query_context));
    (void)arm_controller_get_snapshot(&application_controller,
                                      &query_context.arm);
    (void)joint_reference_get_snapshot(&application_joint_reference,
                                       &query_context.joints);
    (void)motor_runtime_get_snapshot(&application_motor_runtime,
                                     timestamp_us,
                                     MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US,
                                     &query_context.motors);
    (void)diagnostics_get_counters(&application_diagnostics,
                                   &query_context.diagnostics);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);
        uint16_t verified_fields = application_motor_runtime.discovery
                                       .results[joint_index]
                                       .verified_fields_mask;

        if ((verified_fields & MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK) ==
            MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK)
        {
            query_context.motor_identity_verified_mask |= joint_bit;
        }
        if ((verified_fields & MOTOR_DISCOVERY_MODE_FIELDS_MASK) ==
            MOTOR_DISCOVERY_MODE_FIELDS_MASK)
        {
            query_context.motor_mode_verified_mask |= joint_bit;
        }
        if ((verified_fields & MOTOR_DISCOVERY_RANGE_FIELDS_MASK) ==
            MOTOR_DISCOVERY_RANGE_FIELDS_MASK)
        {
            query_context.motor_ranges_verified_mask |= joint_bit;
        }
        if ((verified_fields & MOTOR_DISCOVERY_VERSION_FIELDS_MASK) ==
            MOTOR_DISCOVERY_VERSION_FIELDS_MASK)
        {
            query_context.motor_version_verified_mask |= joint_bit;
        }
    }
    query_context.timestamp_us = timestamp_us;
    protocol_engine_update_query_context(&application_protocol_engine,
                                         &query_context);
}

/**
 * @brief Initializes all static Phase 0 application state.
 * @param timestamp_us Initialization timestamp in microseconds.
 * @param boot_id Nonzero boot identity generated by the STM32 entry point.
 */
void aethor_app_init(uint64_t timestamp_us, uint32_t boot_id)
{
    MotorRuntimeStatus motor_status;

    diagnostics_init(&application_diagnostics);
    arm_controller_init(&application_controller,
                        arm_config_get_production(),
                        &application_diagnostics,
                        timestamp_us);
    (void)joint_reference_init(&application_joint_reference,
                               arm_config_get_production());
    motor_status = motor_runtime_init(&application_motor_runtime,
                                      arm_config_get_production());
    protocol_engine_init(&application_protocol_engine, boot_id);
    memset(&application_emergency_disable_batch,
           0,
           sizeof(application_emergency_disable_batch));
    application_emergency_disable_read_index = 0U;
    memset(&application_action, 0, sizeof(application_action));
    application_initialized =
        (uint8_t)(motor_status == MOTOR_RUNTIME_STATUS_OK);
}

/**
 * @brief Processes one complete USB protocol line in task context.
 */
ProtocolEngineStatus aethor_app_process_protocol_line(
    const char *line,
    size_t length,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    if (application_initialized == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_INVALID_ARGUMENT;
    }
    aethor_app_update_protocol_context(timestamp_us);
    return protocol_engine_process_line(&application_protocol_engine,
                                        line,
                                        length,
                                        timestamp_us,
                                        output_batch);
}

/**
 * @brief Pops one pending fail-safe motor disable frame.
 */
uint8_t aethor_app_pop_emergency_can_frame(CanFrame *frame)
{
    if ((application_initialized == 0U) || (frame == NULL) ||
        (application_emergency_disable_read_index >=
         application_emergency_disable_batch.count))
    {
        return 0U;
    }
    *frame = application_emergency_disable_batch.frames[
        application_emergency_disable_read_index];
    ++application_emergency_disable_read_index;
    return 1U;
}

/**
 * @brief Pops one ordered atomic J1-J7 motion control group.
 */
uint8_t aethor_app_pop_control_group(
    CanFrame frames[ARM_JOINT_COUNT])
{
    if ((application_initialized == 0U) || (frames == NULL) ||
        (application_action.control_group_ready == 0U))
    {
        return 0U;
    }
    memcpy(frames,
           application_action.control_group,
           sizeof(application_action.control_group));
    application_action.control_group_ready = 0U;
    return 1U;
}

/**
 * @brief Latches an atomic control-group scheduler rejection and stops safely.
 */
uint8_t aethor_app_report_control_group_failure(
    CanTxSchedulerStatus scheduler_status,
    uint64_t timestamp_us)
{
    if ((application_initialized == 0U) ||
        ((application_action.state != AETHOR_APP_ACTION_MOTION) &&
         (application_action.state != AETHOR_APP_ACTION_CONTROLLED_STOP)))
    {
        return 0U;
    }
    aethor_app_latch_motion_failure((uint16_t)scheduler_status, timestamp_us);
    return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                                      (uint16_t)scheduler_status,
                                      timestamp_us);
}

/**
 * @brief Produces the next bounded CAN service frame for the platform scheduler.
 */
MotorRuntimeStatus aethor_app_next_can_frame(uint64_t timestamp_us,
                                             CanFrame *frame,
                                             CanTxPriority *priority)
{
    MotorRuntimeStatus runtime_status;

    if (application_initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    if ((frame == NULL) || (priority == NULL))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }

    if (application_action.frame_read_index < application_action.frames.count)
    {
        *frame = application_action.frames.frames[
            application_action.frame_read_index];
        ++application_action.frame_read_index;
        *priority = application_action.priority;
        return MOTOR_RUNTIME_STATUS_FRAME_READY;
    }
    if ((application_action.state == AETHOR_APP_ACTION_MODE_SWITCH) ||
        (application_action.state == AETHOR_APP_ACTION_BENCH_MODE_SWITCH))
    {
        runtime_status = motor_runtime_next_control_mode_frame(
            &application_motor_runtime,
            timestamp_us,
            frame);
        if (runtime_status == MOTOR_RUNTIME_STATUS_FRAME_READY)
        {
            *priority = CAN_TX_PRIORITY_PARAMETER;
        }
        return runtime_status;
    }

    runtime_status = motor_runtime_next_discovery_frame(
        &application_motor_runtime,
        timestamp_us,
        frame);
    if (runtime_status == MOTOR_RUNTIME_STATUS_FRAME_READY)
    {
        *priority = CAN_TX_PRIORITY_PARAMETER;
    }
    return runtime_status;
}

/**
 * @brief Routes one received CAN frame through discovery or feedback decode.
 */
MotorRuntimeStatus aethor_app_receive_can_frame(const CanFrame *frame,
                                                uint64_t timestamp_us)
{
    if (application_initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    return motor_runtime_accept_frame(&application_motor_runtime,
                                      frame,
                                      timestamp_us);
}

/**
 * @brief Copies the current coherent seven-motor feedback snapshot.
 */
bool aethor_app_get_motor_snapshot(uint64_t timestamp_us,
                                  MotorFeedbackSnapshot *snapshot)
{
    if (application_initialized == 0U)
    {
        return false;
    }
    return motor_runtime_get_snapshot(&application_motor_runtime,
                                      timestamp_us,
                                      MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US,
                                      snapshot) == MOTOR_RUNTIME_STATUS_OK;
}

/** @brief Starts one motor-backed lifecycle command after protocol admission. */
static uint8_t aethor_app_start_lifecycle_action(
    const ProtocolCommand *command,
    const MotorFeedbackSnapshot *motor_snapshot,
    MotorRuntimeStatus snapshot_status,
    uint64_t timestamp_us)
{
    ProtocolCommandResult result;
    ArmSnapshot arm_snapshot;
    S3519ControlMode vendor_mode;
    MotorRuntimeStatus runtime_status = MOTOR_RUNTIME_STATUS_OK;

    memset(&result, 0, sizeof(result));
    result.request_id = command->request_id;
    result.session_id = command->session_id;
    result.type = command->type;
    result.code = PROTOCOL_COMMAND_RESULT_FAILED;
    result.accepted_at_us = command->accepted_at_us;
    result.completed_at_us = timestamp_us;
    result.motor_mask = command->motor_mask;
    result.bench_relative_scope = command->bench_relative_scope;
    (void)arm_controller_get_snapshot(&application_controller, &arm_snapshot);
    vendor_mode = aethor_app_vendor_mode(arm_snapshot.control_mode);

    if (command->type == PROTOCOL_COMMAND_ALIGN_REFERENCE)
    {
        JointReferenceStatus reference_status =
            (snapshot_status == MOTOR_RUNTIME_STATUS_OK)
                ? joint_reference_align(&application_joint_reference,
                                        motor_snapshot,
                                        command->values,
                                        timestamp_us)
                : JOINT_REFERENCE_STATUS_FEEDBACK_INCOMPLETE;
        result.detail = (uint16_t)reference_status;
        if ((reference_status == JOINT_REFERENCE_STATUS_OK) &&
            (arm_controller_mark_reference_aligned(&application_controller,
                                                   timestamp_us) ==
             ARM_TRANSITION_STATUS_OK))
        {
            result.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
            result.detail = 0U;
            memcpy(result.values, command->values, sizeof(result.values));
            (void)joint_reference_get_bias_degrees(&application_joint_reference,
                                                   result.auxiliary_values);
        }
        return protocol_engine_submit_command_result(&application_protocol_engine,
                                                     &result);
    }
    if (command->type == PROTOCOL_COMMAND_INIT_MOTORS)
    {
        runtime_status = motor_runtime_begin_discovery(
            &application_motor_runtime,
            command->motor_mask);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            memset(&application_action, 0, sizeof(application_action));
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_DISCOVERY;
            application_action.deadline_us = timestamp_us +
                                             AETHOR_APP_DISCOVERY_TIMEOUT_US;
            return 0U;
        }
    }
    else if (command->type == PROTOCOL_COMMAND_SET_MODE)
    {
        vendor_mode = aethor_app_vendor_mode(command->control_mode);
        runtime_status = motor_runtime_begin_control_mode_switch(
            &application_motor_runtime,
            vendor_mode);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            memset(&application_action, 0, sizeof(application_action));
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_MODE_SWITCH;
            application_action.deadline_us = timestamp_us +
                                             AETHOR_APP_ACTION_TIMEOUT_US;
            return 0U;
        }
    }
    else if ((command->type == PROTOCOL_COMMAND_ENABLE) &&
             (command->bench_relative_scope != 0U))
    {
        runtime_status =
            ((application_motor_runtime.discovery.verified_joint_mask &
              command->motor_mask) == command->motor_mask)
                ? motor_runtime_build_mode_command_batch(
                      &application_motor_runtime,
                      S3519_CONTROL_MODE_POSITION_VELOCITY,
                      S3519_MODE_COMMAND_ENABLE,
                      command->motor_mask,
                      &application_action.frames)
                : MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR;
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_ENABLE_WAIT;
            application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
            application_action.frame_read_index = 0U;
            application_action.deadline_us = timestamp_us +
                                             AETHOR_APP_ACTION_TIMEOUT_US;
            return 0U;
        }
    }
    else if (command->type == PROTOCOL_COMMAND_ENABLE)
    {
        runtime_status = motor_runtime_build_mode_command_batch(
            &application_motor_runtime,
            vendor_mode,
            S3519_MODE_COMMAND_ENABLE,
            AETHOR_APP_ALL_JOINTS_MASK,
            &application_action.frames);
        if ((runtime_status == MOTOR_RUNTIME_STATUS_OK) &&
            (arm_controller_begin_enable(&application_controller,
                                         timestamp_us) ==
             ARM_TRANSITION_STATUS_OK))
        {
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_ENABLE_WAIT;
            application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
            application_action.frame_read_index = 0U;
            application_action.deadline_us = timestamp_us +
                                             AETHOR_APP_ACTION_TIMEOUT_US;
            return 0U;
        }
    }
    else if ((command->type == PROTOCOL_COMMAND_STOP) &&
             (command->bench_relative_scope != 0U))
    {
        float hold_position_rad[ARM_JOINT_COUNT] = {0};
        float zero_velocity_rad_s[ARM_JOINT_COUNT] = {0};
        uint8_t joint_index;
        uint8_t selected_feedback_valid = 1U;

        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            uint8_t joint_bit = (uint8_t)(1U << joint_index);

            if ((command->motor_mask & joint_bit) == 0U)
            {
                continue;
            }
            if ((motor_snapshot->valid_joint_mask & joint_bit) == 0U)
            {
                selected_feedback_valid = 0U;
                break;
            }
            hold_position_rad[joint_index] =
                motor_snapshot->joints[joint_index].position_rad;
        }
        if (selected_feedback_valid != 0U)
        {
            runtime_status = motor_runtime_build_position_velocity_subset(
                &application_motor_runtime,
                command->motor_mask,
                hold_position_rad,
                zero_velocity_rad_s,
                &application_action.frames);
            if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
            {
                application_action.command = *command;
                application_action.state = AETHOR_APP_ACTION_BENCH_MOVE_WAIT;
                application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
                application_action.frame_read_index = 0U;
                memcpy(application_action.motion_plan.target_position_rad,
                       hold_position_rad,
                       sizeof(hold_position_rad));
                application_action.deadline_us = timestamp_us +
                                                 AETHOR_APP_ACTION_TIMEOUT_US;
                return 0U;
            }
        }
        runtime_status = motor_runtime_build_mode_command_batch(
            &application_motor_runtime,
            S3519_CONTROL_MODE_POSITION_VELOCITY,
            S3519_MODE_COMMAND_DISABLE,
            command->motor_mask,
            &application_action.frames);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_DISABLE_WAIT;
            application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
            application_action.frame_read_index = 0U;
            application_action.deadline_us = timestamp_us +
                                             AETHOR_APP_ACTION_TIMEOUT_US;
            return 0U;
        }
    }
    else if (command->type == PROTOCOL_COMMAND_STOP)
    {
        JointStateSnapshot joint_snapshot;
        JointControlledStopPlan stop_plan;
        float position_rad[ARM_JOINT_COUNT];
        float velocity_rad_s[ARM_JOINT_COUNT];
        uint8_t joint_index;

        memset(&joint_snapshot, 0, sizeof(joint_snapshot));
        if ((snapshot_status == MOTOR_RUNTIME_STATUS_OK) &&
            (joint_reference_get_snapshot(&application_joint_reference,
                                          &joint_snapshot) ==
             JOINT_REFERENCE_STATUS_OK) &&
            (aethor_app_joint_snapshot_to_radians(&joint_snapshot,
                                                  position_rad,
                                                  velocity_rad_s) != 0U) &&
            (joint_motion_plan_controlled_stop(arm_config_get_production(),
                                               position_rad,
                                               velocity_rad_s,
                                               1.0F,
                                               timestamp_us,
                                               &stop_plan) ==
             JOINT_MOTION_STATUS_OK) &&
            (arm_controller_begin_controlled_stop(&application_controller,
                                                  timestamp_us) ==
             ARM_TRANSITION_STATUS_OK))
        {
            memset(&application_action, 0, sizeof(application_action));
            application_action.command = *command;
            application_action.controlled_stop_plan = stop_plan;
            application_action.motion_plan.start_time_us = timestamp_us;
            application_action.motion_plan.duration_us = stop_plan.duration_us;
            application_action.motion_plan.mode =
                (command->control_mode == ARM_CONTROL_MODE_MIT)
                    ? JOINT_MOTION_MODE_MIT
                    : JOINT_MOTION_MODE_POSITION_VELOCITY;
            for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
            {
                application_action.motion_plan.target_position_rad[joint_index] =
                    stop_plan.hold_position_rad[joint_index];
            }
            application_action.state = AETHOR_APP_ACTION_CONTROLLED_STOP;
            application_action.deadline_us =
                timestamp_us + stop_plan.duration_us +
                AETHOR_APP_MOTION_SETTLE_US +
                AETHOR_APP_MOTION_TIMEOUT_MARGIN_US;
            joint_motion_completion_init(&application_action.motion_completion);
            if (aethor_app_prepare_controlled_stop_group(timestamp_us) ==
                JOINT_MOTION_CAN_STATUS_OK)
            {
                return 0U;
            }
        }

        memset(&application_action, 0, sizeof(application_action));
        runtime_status = motor_runtime_build_emergency_disable(
            &application_motor_runtime,
            &application_action.frames);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_DISABLE_WAIT;
            application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
            application_action.frame_read_index = 0U;
            application_action.deadline_us = timestamp_us +
                                             AETHOR_APP_ACTION_TIMEOUT_US;
            return 0U;
        }
    }
    else if ((command->type == PROTOCOL_COMMAND_DISABLE) &&
             (command->bench_relative_scope != 0U))
    {
        runtime_status = motor_runtime_build_mode_command_batch(
            &application_motor_runtime,
            S3519_CONTROL_MODE_POSITION_VELOCITY,
            S3519_MODE_COMMAND_DISABLE,
            command->motor_mask,
            &application_action.frames);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_DISABLE_WAIT;
            application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
            application_action.frame_read_index = 0U;
            application_action.deadline_us = timestamp_us +
                                             AETHOR_APP_ACTION_TIMEOUT_US;
            return 0U;
        }
    }
    else if (command->type == PROTOCOL_COMMAND_DISABLE)
    {
        runtime_status = motor_runtime_build_emergency_disable(
            &application_motor_runtime,
            &application_action.frames);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_DISABLE_WAIT;
            application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
            application_action.frame_read_index = 0U;
            application_action.deadline_us = timestamp_us +
                                             AETHOR_APP_ACTION_TIMEOUT_US;
            return 0U;
        }
    }
    else if ((command->type == PROTOCOL_COMMAND_CLEAR_FAULT) &&
             (command->bench_relative_scope != 0U))
    {
        runtime_status = motor_runtime_build_mode_command_batch(
            &application_motor_runtime,
            S3519_CONTROL_MODE_POSITION_VELOCITY,
            S3519_MODE_COMMAND_CLEAR_ERROR,
            command->motor_mask,
            &application_action.frames);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_CLEAR_FAULT_WAIT;
            application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
            application_action.frame_read_index = 0U;
            application_action.deadline_us = timestamp_us +
                                             AETHOR_APP_ACTION_TIMEOUT_US;
            return 0U;
        }
    }
    else if (command->type == PROTOCOL_COMMAND_CLEAR_FAULT)
    {
        if (arm_snapshot.control_mode != ARM_CONTROL_MODE_UNKNOWN)
        {
            runtime_status = motor_runtime_build_mode_command_batch(
                &application_motor_runtime,
                vendor_mode,
                S3519_MODE_COMMAND_CLEAR_ERROR,
                command->motor_mask,
                &application_action.frames);
            if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
            {
                application_action.command = *command;
                application_action.state = AETHOR_APP_ACTION_CLEAR_FAULT_WAIT;
                application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
                application_action.frame_read_index = 0U;
                application_action.deadline_us = timestamp_us +
                                                 AETHOR_APP_ACTION_TIMEOUT_US;
                return 0U;
            }
        }
    }
    else if (command->type == PROTOCOL_COMMAND_MOVE_RELATIVE)
    {
        float motor_position_rad[ARM_JOINT_COUNT];
        float motor_velocity_rad_s[ARM_JOINT_COUNT];
        uint8_t joint_index;
        uint8_t selected_feedback_valid = 1U;
        float maximum_duration_seconds = 0.0F;

        memset(motor_position_rad, 0, sizeof(motor_position_rad));
        memset(motor_velocity_rad_s, 0, sizeof(motor_velocity_rad_s));
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            uint8_t joint_bit = (uint8_t)(1U << joint_index);

            if ((command->motor_mask & joint_bit) == 0U)
            {
                continue;
            }
            if (((motor_snapshot->valid_joint_mask & joint_bit) == 0U) ||
                (motor_snapshot->joints[joint_index].fault_flags != 0U) ||
                (motor_snapshot->joints[joint_index].driver_state !=
                 S3519_DRIVER_STATE_ENABLED))
            {
                selected_feedback_valid = 0U;
                break;
            }
            motor_position_rad[joint_index] =
                motor_snapshot->joints[joint_index].position_rad +
                (command->values[joint_index] * AETHOR_APP_DEG_TO_RAD);
            motor_velocity_rad_s[joint_index] =
                command->speeds[joint_index] * AETHOR_APP_DEG_TO_RAD;
            if ((command->values[joint_index] < 0.0F
                     ? -command->values[joint_index]
                     : command->values[joint_index]) /
                    command->speeds[joint_index] > maximum_duration_seconds)
            {
                maximum_duration_seconds =
                    (command->values[joint_index] < 0.0F
                         ? -command->values[joint_index]
                         : command->values[joint_index]) /
                    command->speeds[joint_index];
            }
        }
        if (selected_feedback_valid != 0U)
        {
            runtime_status = motor_runtime_build_position_velocity_subset(
                &application_motor_runtime,
                command->motor_mask,
                motor_position_rad,
                motor_velocity_rad_s,
                &application_action.frames);
            if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
            {
                application_action.command = *command;
                application_action.state = AETHOR_APP_ACTION_BENCH_MOVE_WAIT;
                application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
                application_action.frame_read_index = 0U;
                memcpy(application_action.motion_plan.target_position_rad,
                       motor_position_rad,
                       sizeof(motor_position_rad));
                application_action.deadline_us = timestamp_us +
                    (uint64_t)(maximum_duration_seconds * 1000000.0F) +
                    AETHOR_APP_ACTION_TIMEOUT_US;
                return 0U;
            }
        }
        runtime_status = MOTOR_RUNTIME_STATUS_STALE_FEEDBACK;
    }
    else if (command->type == PROTOCOL_COMMAND_MOVE_JOINTS)
    {
        JointStateSnapshot joint_snapshot;
        JointMotionPlan motion_plan;
        JointMotionMode motion_mode =
            (command->control_mode == ARM_CONTROL_MODE_MIT)
                ? JOINT_MOTION_MODE_MIT
                : JOINT_MOTION_MODE_POSITION_VELOCITY;
        float start_position_rad[ARM_JOINT_COUNT];
        float current_velocity_rad_s[ARM_JOINT_COUNT];
        float target_position_rad[ARM_JOINT_COUNT];
        uint8_t joint_index;

        memset(&joint_snapshot, 0, sizeof(joint_snapshot));
        if ((joint_reference_get_snapshot(&application_joint_reference,
                                          &joint_snapshot) ==
             JOINT_REFERENCE_STATUS_OK) &&
            (aethor_app_joint_snapshot_to_radians(&joint_snapshot,
                                                  start_position_rad,
                                                  current_velocity_rad_s) != 0U))
        {
            for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
            {
                target_position_rad[joint_index] =
                    command->values[joint_index] * AETHOR_APP_DEG_TO_RAD;
            }
            if ((joint_motion_plan(arm_config_get_production(),
                                   start_position_rad,
                                   target_position_rad,
                                   command->speeds[0],
                                   motion_mode,
                                   timestamp_us,
                                   &motion_plan) == JOINT_MOTION_STATUS_OK) &&
                (arm_controller_begin_motion(&application_controller,
                                             timestamp_us) ==
                 ARM_TRANSITION_STATUS_OK))
            {
                memset(&application_action, 0, sizeof(application_action));
                application_action.command = *command;
                application_action.motion_plan = motion_plan;
                application_action.state = AETHOR_APP_ACTION_MOTION;
                application_action.deadline_us =
                    timestamp_us + motion_plan.duration_us +
                    AETHOR_APP_MOTION_SETTLE_US +
                    AETHOR_APP_MOTION_TIMEOUT_MARGIN_US;
                joint_motion_completion_init(
                    &application_action.motion_completion);
                if (aethor_app_prepare_motion_group(timestamp_us) ==
                    JOINT_MOTION_CAN_STATUS_OK)
                {
                    return 0U;
                }
                aethor_app_latch_motion_failure(1U, timestamp_us);
                return aethor_app_complete_action(
                    PROTOCOL_COMMAND_RESULT_FAILED,
                    1U,
                    timestamp_us);
            }
        }
        runtime_status = MOTOR_RUNTIME_STATUS_ACTION_FAILED;
    }

    result.detail = (uint16_t)runtime_status;
    return protocol_engine_submit_command_result(&application_protocol_engine,
                                                 &result);
}

/** @brief Advances one active motor-backed lifecycle action. */
static uint8_t aethor_app_service_active_action(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    if (application_action.state == AETHOR_APP_ACTION_IDLE)
    {
        return 0U;
    }
    if (application_action.state == AETHOR_APP_ACTION_MODE_SWITCH)
    {
        if (application_motor_runtime.mode_switch_state ==
            MOTOR_MODE_SWITCH_COMPLETE)
        {
            if (arm_controller_confirm_control_mode(
                    &application_controller,
                    application_action.command.control_mode,
                    timestamp_us) == ARM_TRANSITION_STATUS_OK)
            {
                return aethor_app_complete_action(
                    PROTOCOL_COMMAND_RESULT_COMPLETED,
                    0U,
                    timestamp_us);
            }
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                                              1U,
                                              timestamp_us);
        }
        if (application_motor_runtime.mode_switch_state ==
            MOTOR_MODE_SWITCH_FAILED)
        {
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                                              2U,
                                              timestamp_us);
        }
    }
    else if (application_action.state == AETHOR_APP_ACTION_DISCOVERY)
    {
        if ((application_motor_runtime.discovery.state ==
             MOTOR_DISCOVERY_STATE_COMPLETE) &&
            ((application_motor_runtime.discovery.verified_joint_mask &
              application_action.command.motor_mask) ==
             application_action.command.motor_mask))
        {
            if (motor_runtime_begin_control_mode_switch_mask(
                    &application_motor_runtime,
                    S3519_CONTROL_MODE_POSITION_VELOCITY,
                    application_action.command.motor_mask) ==
                MOTOR_RUNTIME_STATUS_OK)
            {
                application_action.state =
                    AETHOR_APP_ACTION_BENCH_MODE_SWITCH;
                application_action.deadline_us = timestamp_us +
                                                 AETHOR_APP_ACTION_TIMEOUT_US;
                return 0U;
            }
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                                              2U,
                                              timestamp_us);
        }
        if (application_motor_runtime.discovery.state ==
            MOTOR_DISCOVERY_STATE_FAILED)
        {
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                                              1U,
                                              timestamp_us);
        }
    }
    else if (application_action.state == AETHOR_APP_ACTION_BENCH_MODE_SWITCH)
    {
        if (application_motor_runtime.mode_switch_state ==
            MOTOR_MODE_SWITCH_COMPLETE)
        {
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_COMPLETED,
                                              0U,
                                              timestamp_us);
        }
        if (application_motor_runtime.mode_switch_state ==
            MOTOR_MODE_SWITCH_FAILED)
        {
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                                              3U,
                                              timestamp_us);
        }
    }
    else if ((application_action.frame_read_index >=
              application_action.frames.count) &&
             (application_action.state == AETHOR_APP_ACTION_ENABLE_WAIT) &&
             (aethor_app_selected_motors_have_state(
                  motor_snapshot,
                  (application_action.command.bench_relative_scope != 0U)
                      ? application_action.command.motor_mask
                      : AETHOR_APP_ALL_JOINTS_MASK,
                  S3519_DRIVER_STATE_ENABLED) != 0U))
    {
        if (application_action.command.bench_relative_scope != 0U)
        {
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_COMPLETED,
                                              0U,
                                              timestamp_us);
        }
        if (arm_controller_confirm_enabled(&application_controller,
                                           timestamp_us) ==
            ARM_TRANSITION_STATUS_OK)
        {
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_COMPLETED,
                                              0U,
                                              timestamp_us);
        }
    }
    else if (application_action.state == AETHOR_APP_ACTION_MOTION)
    {
        JointStateSnapshot joint_snapshot;
        float feedback_position_rad[ARM_JOINT_COUNT];
        float feedback_velocity_rad_s[ARM_JOINT_COUNT];

        memset(&joint_snapshot, 0, sizeof(joint_snapshot));
        if ((joint_reference_get_snapshot(&application_joint_reference,
                                          &joint_snapshot) ==
             JOINT_REFERENCE_STATUS_OK) &&
            (aethor_app_joint_snapshot_to_radians(&joint_snapshot,
                                                  feedback_position_rad,
                                                  feedback_velocity_rad_s) != 0U))
        {
            aethor_app_update_following_error(feedback_position_rad,
                                              timestamp_us);
            (void)joint_motion_update_completion(
                &application_action.motion_plan,
                arm_config_get_production(),
                feedback_position_rad,
                feedback_velocity_rad_s,
                joint_snapshot.valid_joint_mask,
                timestamp_us,
                AETHOR_APP_MOTION_SETTLE_US,
                &application_action.motion_completion);
            if (application_action.motion_completion.completed != 0U)
            {
                if (arm_controller_complete_motion(&application_controller,
                                                   timestamp_us) ==
                    ARM_TRANSITION_STATUS_OK)
                {
                    return aethor_app_complete_action(
                        PROTOCOL_COMMAND_RESULT_COMPLETED,
                        0U,
                        timestamp_us);
                }
            }
        }
        if ((application_action.control_group_ready == 0U) &&
            ((application_action.motion_plan.mode == JOINT_MOTION_MODE_MIT) ||
             (timestamp_us == application_action.motion_plan.start_time_us)) &&
            (aethor_app_prepare_motion_group(timestamp_us) !=
             JOINT_MOTION_CAN_STATUS_OK))
        {
            aethor_app_latch_motion_failure(2U, timestamp_us);
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                                              2U,
                                              timestamp_us);
        }
    }
    else if (application_action.state == AETHOR_APP_ACTION_CONTROLLED_STOP)
    {
        JointStateSnapshot joint_snapshot;
        float feedback_position_rad[ARM_JOINT_COUNT];
        float feedback_velocity_rad_s[ARM_JOINT_COUNT];
        uint64_t trajectory_end_us =
            application_action.controlled_stop_plan.start_time_us +
            application_action.controlled_stop_plan.duration_us;

        memset(&joint_snapshot, 0, sizeof(joint_snapshot));
        if ((timestamp_us >= trajectory_end_us) &&
            (joint_reference_get_snapshot(&application_joint_reference,
                                          &joint_snapshot) ==
             JOINT_REFERENCE_STATUS_OK) &&
            (aethor_app_joint_snapshot_to_radians(&joint_snapshot,
                                                  feedback_position_rad,
                                                  feedback_velocity_rad_s) != 0U))
        {
            (void)joint_motion_update_completion(
                &application_action.motion_plan,
                arm_config_get_production(),
                feedback_position_rad,
                feedback_velocity_rad_s,
                joint_snapshot.valid_joint_mask,
                timestamp_us,
                AETHOR_APP_MOTION_SETTLE_US,
                &application_action.motion_completion);
            if (application_action.motion_completion.completed != 0U)
            {
                if (arm_controller_complete_motion(&application_controller,
                                                   timestamp_us) ==
                    ARM_TRANSITION_STATUS_OK)
                {
                    return aethor_app_complete_action(
                        PROTOCOL_COMMAND_RESULT_STOPPED,
                        0U,
                        timestamp_us);
                }
            }
        }
        if ((application_action.control_group_ready == 0U) &&
            (application_action.command.control_mode == ARM_CONTROL_MODE_MIT) &&
            (aethor_app_prepare_controlled_stop_group(timestamp_us) !=
             JOINT_MOTION_CAN_STATUS_OK))
        {
            aethor_app_latch_motion_failure(4U, timestamp_us);
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                                              4U,
                                              timestamp_us);
        }
    }
    else if ((application_action.frame_read_index >=
              application_action.frames.count) &&
             (application_action.state == AETHOR_APP_ACTION_BENCH_MOVE_WAIT))
    {
        uint8_t joint_index;
        uint8_t all_selected_at_target = 1U;

        if ((motor_snapshot->valid_joint_mask &
             application_action.command.motor_mask) !=
            application_action.command.motor_mask)
        {
            all_selected_at_target = 0U;
        }
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            uint8_t joint_bit = (uint8_t)(1U << joint_index);
            float position_error;

            if ((application_action.command.motor_mask & joint_bit) == 0U)
            {
                continue;
            }
            position_error = motor_snapshot->joints[joint_index].position_rad -
                             application_action.motion_plan
                                 .target_position_rad[joint_index];
            if (position_error < 0.0F)
            {
                position_error = -position_error;
            }
            if ((position_error > (0.5F * AETHOR_APP_DEG_TO_RAD)) ||
                (motor_snapshot->joints[joint_index].fault_flags != 0U))
            {
                all_selected_at_target = 0U;
            }
        }
        if (all_selected_at_target != 0U)
        {
            return aethor_app_complete_action(
                (application_action.command.type == PROTOCOL_COMMAND_STOP)
                    ? PROTOCOL_COMMAND_RESULT_STOPPED
                    : PROTOCOL_COMMAND_RESULT_COMPLETED,
                0U,
                timestamp_us);
        }
    }
    else if ((application_action.frame_read_index >=
              application_action.frames.count) &&
             (application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT) &&
             (aethor_app_selected_motors_have_state(
                  motor_snapshot,
                  (application_action.command.bench_relative_scope != 0U)
                      ? application_action.command.motor_mask
                      : AETHOR_APP_ALL_JOINTS_MASK,
                  S3519_DRIVER_STATE_DISABLED) != 0U))
    {
        if (application_action.command.bench_relative_scope == 0U)
        {
            (void)arm_controller_confirm_disabled(&application_controller,
                                                  timestamp_us);
        }
        return aethor_app_complete_action(
            (application_action.command.type == PROTOCOL_COMMAND_STOP)
                ? PROTOCOL_COMMAND_RESULT_STOPPED
                : PROTOCOL_COMMAND_RESULT_COMPLETED,
            0U,
            timestamp_us);
    }
    else if ((application_action.frame_read_index >=
              application_action.frames.count) &&
             (application_action.state == AETHOR_APP_ACTION_CLEAR_FAULT_WAIT) &&
             (aethor_app_selected_motors_are_fault_free(
                  motor_snapshot,
                  application_action.command.motor_mask) != 0U))
    {
        if (arm_controller_clear_fault(&application_controller,
                                       timestamp_us) ==
            ARM_TRANSITION_STATUS_OK)
        {
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_COMPLETED,
                                              0U,
                                              timestamp_us);
        }
        if (application_action.command.bench_relative_scope != 0U)
        {
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_COMPLETED,
                                              0U,
                                              timestamp_us);
        }
    }

    if (timestamp_us >= application_action.deadline_us)
    {
        if (application_action.state == AETHOR_APP_ACTION_ENABLE_WAIT)
        {
            (void)arm_controller_force_stop_disable(&application_controller,
                                                     timestamp_us);
            application_emergency_disable_read_index = 0U;
            (void)motor_runtime_build_emergency_disable(
                &application_motor_runtime,
                &application_emergency_disable_batch);
        }
        else if ((application_action.state == AETHOR_APP_ACTION_MOTION) ||
                 (application_action.state == AETHOR_APP_ACTION_CONTROLLED_STOP))
        {
            aethor_app_latch_motion_failure(3U, timestamp_us);
        }
        return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                                          3U,
                                          timestamp_us);
    }
    return 0U;
}

/**
 * @brief Executes one non-blocking Phase 0 application service cycle.
 * @param timestamp_us Current monotonic time in microseconds.
 */
uint8_t aethor_app_service(uint64_t timestamp_us)
{
    uint8_t result_generated = 0U;

    if (application_initialized != 0U)
    {
        MotorFeedbackSnapshot motor_snapshot;
        ProtocolCommand command;
        MotorRuntimeStatus snapshot_status;
        ArmSnapshot arm_snapshot;

        if ((application_last_service_timestamp_us != 0U) &&
            (timestamp_us >= application_last_service_timestamp_us))
        {
            uint64_t period_us = timestamp_us -
                                 application_last_service_timestamp_us;

            diagnostics_record_control_period(
                &application_diagnostics,
                (period_us > UINT32_MAX) ? UINT32_MAX : (uint32_t)period_us);
        }
        application_last_service_timestamp_us = timestamp_us;

        memset(&motor_snapshot, 0, sizeof(motor_snapshot));
        snapshot_status = motor_runtime_get_snapshot(
            &application_motor_runtime,
            timestamp_us,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US,
            &motor_snapshot);
        if (snapshot_status == MOTOR_RUNTIME_STATUS_OK)
        {
            (void)joint_reference_publish(&application_joint_reference,
                                          &motor_snapshot,
                                          timestamp_us);
        }
        arm_controller_step(&application_controller, timestamp_us);

        memset(&arm_snapshot, 0, sizeof(arm_snapshot));
        (void)arm_controller_get_snapshot(&application_controller,
                                          &arm_snapshot);
        if ((arm_snapshot.enabled != 0U) || (arm_snapshot.moving != 0U))
        {
            uint32_t driver_fault_detail =
                aethor_app_first_driver_fault_detail(&motor_snapshot);
            ArmFault runtime_fault = ARM_FAULT_NONE;
            uint32_t runtime_fault_detail = 0U;

            if (driver_fault_detail != 0U)
            {
                runtime_fault = ARM_FAULT_DRIVER;
                runtime_fault_detail = driver_fault_detail;
            }
            else if ((snapshot_status != MOTOR_RUNTIME_STATUS_OK) ||
                     (motor_snapshot.valid_joint_mask !=
                      AETHOR_APP_ALL_JOINTS_MASK))
            {
                runtime_fault = ARM_FAULT_FEEDBACK_STALE;
                runtime_fault_detail = motor_snapshot.valid_joint_mask;
            }
            else if (application_diagnostics.counters
                         .control_consecutive_miss_count >= 3U)
            {
                runtime_fault = ARM_FAULT_CONTROL_DEADLINE;
                runtime_fault_detail = application_diagnostics.counters
                                           .control_consecutive_miss_count;
            }
            if (runtime_fault != ARM_FAULT_NONE)
            {
                if (application_action.state != AETHOR_APP_ACTION_IDLE)
                {
                    result_generated = aethor_app_complete_action(
                        PROTOCOL_COMMAND_RESULT_FAILED,
                        (uint16_t)runtime_fault,
                        timestamp_us);
                }
                (void)arm_controller_latch_runtime_fault(
                    &application_controller,
                    runtime_fault,
                    runtime_fault_detail,
                    timestamp_us);
                application_emergency_disable_read_index = 0U;
                (void)motor_runtime_build_emergency_disable(
                    &application_motor_runtime,
                    &application_emergency_disable_batch);
                protocol_engine_cancel_pending_commands(
                    &application_protocol_engine);
                return result_generated;
            }
        }

        if (protocol_engine_watchdog_expired(&application_protocol_engine,
                                             timestamp_us) != 0U)
        {
            ProtocolCommandResult timeout_result;

            if (application_action.state != AETHOR_APP_ACTION_IDLE)
            {
                (void)aethor_app_complete_action(
                    PROTOCOL_COMMAND_RESULT_CANCELLED,
                    0U,
                    timestamp_us);
            }
            (void)arm_controller_force_stop_disable(&application_controller,
                                                     timestamp_us);
            protocol_engine_cancel_pending_commands(&application_protocol_engine);
            application_emergency_disable_read_index = 0U;
            (void)motor_runtime_build_emergency_disable(
                &application_motor_runtime,
                &application_emergency_disable_batch);
            memset(&timeout_result, 0, sizeof(timeout_result));
            timeout_result.session_id = application_protocol_engine.session_id;
            timeout_result.type = PROTOCOL_COMMAND_LINK_TIMEOUT;
            timeout_result.code = PROTOCOL_COMMAND_RESULT_STOPPED;
            timeout_result.completed_at_us = timestamp_us;
            result_generated = protocol_engine_submit_command_result(
                &application_protocol_engine,
                &timeout_result);
        }
        else if (application_action.state != AETHOR_APP_ACTION_IDLE)
        {
            ProtocolCommand stop_command;

            if (protocol_engine_pop_stop_command(&application_protocol_engine,
                                                 &stop_command) != 0U)
            {
                uint8_t cancel_generated = aethor_app_complete_action(
                    PROTOCOL_COMMAND_RESULT_CANCELLED,
                    0U,
                    timestamp_us);
                uint8_t stop_generated = aethor_app_start_lifecycle_action(
                    &stop_command,
                    &motor_snapshot,
                    snapshot_status,
                    timestamp_us);

                result_generated = (uint8_t)(cancel_generated | stop_generated);
            }
            else
            {
                result_generated = aethor_app_service_active_action(
                    &motor_snapshot,
                    timestamp_us);
            }
        }
        else if (protocol_engine_pop_command(&application_protocol_engine,
                                             &command) != 0U)
        {
            result_generated = aethor_app_start_lifecycle_action(
                &command,
                &motor_snapshot,
                snapshot_status,
                timestamp_us);
        }
    }
    return result_generated;
}

/**
 * @brief Formats one pending terminal command result for ProtocolTask.
 */
uint8_t aethor_app_pop_protocol_result_output(
    ProtocolOutputBatch *output_batch)
{
    if ((application_initialized == 0U) || (output_batch == NULL))
    {
        return 0U;
    }
    return protocol_engine_pop_result_output(&application_protocol_engine,
                                             output_batch);
}

/**
 * @brief Generates due session telemetry and state-change events.
 */
uint8_t aethor_app_generate_stream_output(
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    if ((application_initialized == 0U) || (output_batch == NULL))
    {
        return 0U;
    }
    aethor_app_update_protocol_context(timestamp_us);
    return protocol_engine_generate_stream_output(&application_protocol_engine,
                                                  timestamp_us,
                                                  output_batch);
}

/**
 * @brief Formats a high-priority error for one discarded overlong USB line.
 */
ProtocolEngineStatus aethor_app_format_line_too_long(
    ProtocolOutputBatch *output_batch)
{
    return protocol_engine_format_line_too_long(output_batch);
}

/**
 * @brief Copies the current arm state snapshot.
 * @param snapshot Output snapshot.
 * @return true after initialization when snapshot is non-null.
 */
bool aethor_app_get_snapshot(ArmSnapshot *snapshot)
{
    if (application_initialized == 0U)
    {
        return false;
    }

    return arm_controller_get_snapshot(&application_controller, snapshot);
}

/**
 * @brief Copies one retained diagnostic event.
 * @param logical_index Zero-based index from the oldest retained event.
 * @param event Output event copy.
 * @return true when initialized and the requested event exists.
 */
bool aethor_app_get_diagnostic(uint16_t logical_index,
                               DiagnosticEvent *event)
{
    if (application_initialized == 0U)
    {
        return false;
    }

    return diagnostics_get(&application_diagnostics, logical_index, event);
}

/**
 * @brief Copies the current diagnostic counters.
 * @param counters Output counter snapshot.
 * @return true after initialization when counters is non-null.
 */
bool aethor_app_get_diagnostic_counters(DiagnosticCounters *counters)
{
    if (application_initialized == 0U)
    {
        return false;
    }

    return diagnostics_get_counters(&application_diagnostics, counters);
}

/**
 * @brief Applies one platform-neutral transport/resource diagnostic sample.
 */
void aethor_app_update_runtime_diagnostics(
    const RuntimeDiagnosticSample *sample)
{
    if ((application_initialized != 0U) && (sample != NULL))
    {
        diagnostics_update_runtime_sample(&application_diagnostics, sample);
    }
}

/**
 * @brief Latches a severe platform transport fault and schedules all-axis disable.
 */
uint8_t aethor_app_report_transport_fault(uint32_t detail,
                                          uint64_t timestamp_us)
{
    uint8_t result_generated = 0U;

    if (application_initialized == 0U)
    {
        return 0U;
    }
    if (application_action.state != AETHOR_APP_ACTION_IDLE)
    {
        result_generated = aethor_app_complete_action(
            PROTOCOL_COMMAND_RESULT_FAILED,
            (uint16_t)ARM_FAULT_TRANSPORT,
            timestamp_us);
    }
    (void)arm_controller_latch_runtime_fault(&application_controller,
                                             ARM_FAULT_TRANSPORT,
                                             detail,
                                             timestamp_us);
    protocol_engine_cancel_pending_commands(&application_protocol_engine);
    application_emergency_disable_read_index = 0U;
    (void)motor_runtime_build_emergency_disable(
        &application_motor_runtime,
        &application_emergency_disable_batch);
    return result_generated;
}
