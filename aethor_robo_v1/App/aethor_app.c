/**
 * @file aethor_app.c
 * @brief Implements the allocation-free Phase 0 application facade.
 */

#include "aethor_app.h"

#include <stddef.h>
#include <string.h>

#include "arm_config.h"
#include "joint_motion_can.h"

#define AETHOR_APP_ACTION_TIMEOUT_US (500000ULL)
#define AETHOR_APP_MOTION_SETTLE_US (200000ULL)
#define AETHOR_APP_MOTION_TIMEOUT_MARGIN_US (2000000ULL)
#define AETHOR_APP_ALL_JOINTS_MASK ((uint8_t)0x7FU)
#define AETHOR_APP_DEG_TO_RAD (0.017453292519943295F)

/** @brief Describes one asynchronous motor-backed action owned by ArmControlTask. */
typedef enum
{
    AETHOR_APP_ACTION_IDLE = 0,
    AETHOR_APP_ACTION_MODE_SWITCH,
    AETHOR_APP_ACTION_ENABLE_WAIT,
    AETHOR_APP_ACTION_DISABLE_WAIT,
    AETHOR_APP_ACTION_CLEAR_FAULT_WAIT,
    AETHOR_APP_ACTION_MOTION
} AethorAppActionState;

/** @brief Owns the active protocol command and its bounded CAN frame batch. */
typedef struct
{
    ProtocolCommand command;
    MotorEmergencyFrameBatch frames;
    JointMotionPlan motion_plan;
    JointMotionCompletion motion_completion;
    CanFrame control_group[ARM_JOINT_COUNT];
    uint64_t deadline_us;
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
    result.completed_at_us = timestamp_us;
    memset(&application_action, 0, sizeof(application_action));
    return protocol_engine_submit_command_result(&application_protocol_engine,
                                                 &result);
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
        (application_action.state != AETHOR_APP_ACTION_MOTION))
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
    if (application_action.state == AETHOR_APP_ACTION_MODE_SWITCH)
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
    result.completed_at_us = timestamp_us;
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
    if (command->type == PROTOCOL_COMMAND_SET_MODE)
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
    else if ((command->type == PROTOCOL_COMMAND_DISABLE) ||
             (command->type == PROTOCOL_COMMAND_STOP))
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
    else if ((application_action.frame_read_index >=
              application_action.frames.count) &&
             (application_action.state == AETHOR_APP_ACTION_ENABLE_WAIT) &&
             (aethor_app_selected_motors_have_state(
                  motor_snapshot,
                  AETHOR_APP_ALL_JOINTS_MASK,
                  S3519_DRIVER_STATE_ENABLED) != 0U))
    {
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
    else if ((application_action.frame_read_index >=
              application_action.frames.count) &&
             (application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT) &&
             (aethor_app_selected_motors_have_state(
                  motor_snapshot,
                  AETHOR_APP_ALL_JOINTS_MASK,
                  S3519_DRIVER_STATE_DISABLED) != 0U))
    {
        (void)arm_controller_confirm_disabled(&application_controller,
                                              timestamp_us);
        return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_COMPLETED,
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
        else if (application_action.state == AETHOR_APP_ACTION_MOTION)
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
