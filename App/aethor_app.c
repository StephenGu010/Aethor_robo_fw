/**
 * @file aethor_app.c
 * @brief Implements the allocation-free Phase 0 application facade.
 */

#include "aethor_app.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "arm_config.h"
#include "joint_motion_can.h"
#include "DebugUi/debug_ui_mailbox.h"
#include "Config/motor_compatibility.h"
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
#include "Adrc/adrc_app_bridge.h"
#include "Adrc/adrc_generated_adapter.h"
#if AETHOR_ADRC_LCD_INTEGRATED
#include "Adrc/adrc_lcd_ownership.h"
#endif
/** @brief Static selected-axis bridge; legacy-only targets allocate no ADRC gateway. */
static AdrcAppBridge application_adrc_bridge;
#if AETHOR_ADRC_LCD_INTEGRATED
/** @brief Exclusive runtime owner and measured legacy-CAN drain state. */
static AdrcLcdOwnership application_adrc_lcd_ownership;
static uint8_t application_integrated_can_idle;
static uint64_t application_integrated_can_quiet_since_us;
/** @brief One read-only pre-handoff query must transmit before a newer disabled sample can count. */
static uint64_t application_integrated_probe_submitted_us;
static uint8_t application_integrated_probe_transmitted;
static uint8_t application_integrated_probe_failed;
/** @brief A separate LCD-owned diagnostic probe never participates in an ADRC handoff. */
static uint64_t application_integrated_diag_requested_us;
static uint64_t application_integrated_diag_transmitted_us;
static AdrcLcdOwnershipStatus application_integrated_handoff;
/** @brief Builds a read-only gate snapshot for an explicit axis without reserving ownership. */
static void aethor_app_integrated_build_evidence(uint64_t timestamp_us, uint8_t axis,
    AdrcLcdOwnershipEvidence *evidence);
#endif
#endif

#define AETHOR_APP_ACTION_TIMEOUT_US (500000ULL)
#define AETHOR_APP_DISCOVERY_TIMEOUT_US (8000000ULL)
#define AETHOR_APP_MOTION_SETTLE_US (200000ULL)
#define AETHOR_APP_MOTION_TIMEOUT_MARGIN_US (2000000ULL)
#define AETHOR_APP_ALL_JOINTS_MASK ((uint8_t)0x7FU)
#define AETHOR_APP_DEG_TO_RAD (0.017453292519943295F)
#define AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S (0.05F)
#define AETHOR_APP_MIT_QUINTIC_MAX_VELOCITY_FACTOR (1.875F)
#define AETHOR_APP_MIT_MINIMUM_TRAJECTORY_US (4000ULL)
/* DM-S3519-1EC V1.1 rated output torque, approved after bounded no-load trials.
 * This is a feedback-triggered stop threshold, not an instantaneous current clamp. */
#define AETHOR_APP_LOCAL_MIT_TORQUE_TRIP_NM (3.5F)
/* Bench POS stops after a sender stall rather than catching up in one position step. */
#define AETHOR_APP_POS_REFERENCE_MAX_GAP_US (50000ULL)
/* Local fixed-target commissioning guards; final position accuracy remains 0.5 degree. */
#define AETHOR_APP_LOCAL_POS_PROGRESS_TIMEOUT_US (2000000ULL)
#define AETHOR_APP_LOCAL_POS_PROGRESS_RAD (0.1F * AETHOR_APP_DEG_TO_RAD)
/* At least three terminals; divide 256 so uint8 sequence wrap preserves slot order. */
#define AETHOR_APP_DEFERRED_RESULT_CAPACITY (4U)
#define AETHOR_APP_TRANSPORT_FAULT_CAPACITY (2U)

/**
 * @brief Describes legacy and self-contained motor action phases owned by ArmControlTask.
 */
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
    AETHOR_APP_ACTION_CONTROLLED_STOP,
    AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY,
    AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT,
    AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH,
    AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT,
    AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT,
    AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT,
    AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT,
    AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT,
    AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT,
    AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT
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
    float target_position_rad[ARM_JOINT_COUNT];
    float target_speed_rad_s[ARM_JOINT_COUNT];
    uint64_t phase_feedback_baseline_us[ARM_JOINT_COUNT];
    uint64_t deadline_us;
    uint64_t last_pos_reference_timestamp_us;
    uint64_t local_pos_progress_timestamp_us;
    uint64_t local_pos_settle_start_us;
    uint64_t local_pos_last_feedback_us;
    float local_pos_progress_error_rad;
    float maximum_following_error_deg;
    AethorAppActionState state;
    ProtocolCommandStage failed_stage;
    ProtocolCommandError failure_error;
    DebugUiReason local_failure_reason; /* Preserve the first explicit local rejection through cleanup. */
    CanTxPriority priority;
    uint8_t frame_read_index;
    uint8_t control_group_ready;
    uint8_t failed_motor_number;
    uint8_t enabled_by_action_mask;
    uint8_t cleanup_disable_mask;
    uint8_t missing_discovery_mask;
} AethorAppAction;

/** @brief Stores one fully published cross-task transport fault event. */
typedef struct
{
    uint64_t timestamp_us;
    uint32_t detail;
} AethorAppTransportFaultEvent;

static Diagnostics application_diagnostics;
static ArmController application_controller;
static JointReference application_joint_reference;
static MotorRuntime application_motor_runtime;
static ProtocolEngine application_protocol_engine;
static MotorEmergencyFrameBatch application_emergency_disable_batch;
#if !AETHOR_ADRC_BENCH
static uint8_t application_emergency_disable_read_index;
#endif
static AethorAppAction application_action;
static ProtocolCommandResult
    application_deferred_results[AETHOR_APP_DEFERRED_RESULT_CAPACITY];
static uint8_t application_deferred_result_read_sequence;
static uint8_t application_deferred_result_write_sequence;
static AethorAppTransportFaultEvent
    application_transport_faults[AETHOR_APP_TRANSPORT_FAULT_CAPACITY];
static volatile uint8_t application_transport_fault_read_sequence;
static volatile uint8_t application_transport_fault_write_sequence;
#if !AETHOR_ADRC_BENCH
static uint64_t application_last_service_timestamp_us;
#endif
static AethorAppTaskCriticalHook application_enter_task_critical_hook;
static AethorAppTaskCriticalHook application_exit_task_critical_hook;
static uint8_t application_initialized;

/** @brief Shared local state; all cross-task reads/writes use the existing critical hooks. */
typedef struct
{
    DebugUiMailbox mailbox;
    DebugUiMotorProfile profiles[DEBUG_UI_MOTOR_COUNT];
    DebugUiDiagnostics diagnostics;
    DebugUiAuthority authority;
    uint64_t last_activity_us;
    uint32_t epoch;
    uint32_t generation;
    uint32_t reference_generation[DEBUG_UI_MOTOR_COUNT];
    uint32_t observed_mode[DEBUG_UI_MOTOR_COUNT];
    uint8_t target_motor_id;
    uint8_t health_stop_started;
    /** @brief Latched pre-revocation health evidence; subsequent good heartbeats cannot overwrite it. */
    DebugUiHealth health_failure_snapshot;
    uint64_t health_failure_timestamp_us;
    DebugUiAuthority health_failure_authority;
    uint32_t health_failure_action_state;
    /** @brief Original cleanup scope survives failed terminals and selection changes. */
    uint8_t unconfirmed_disable_mask;
    uint64_t disable_confirmation_after_us[DEBUG_UI_MOTOR_COUNT];
    /** @brief Preserve the exact MIT protection sample after cleanup overwrites live feedback. */
    MotorJointFeedback last_mit_guard_feedback;
    DebugUiReason last_mit_guard_reason;
    uint8_t last_mit_guard_motor_id;
    /** @brief Arm-owned id-free stop execution; no result slot or protocol producer needed. */
    uint8_t fallback_stop_mask;
    uint8_t fallback_frame_read_index;
    uint64_t fallback_deadline_us;
    MotorEmergencyFrameBatch fallback_frames;
} AethorAppDebugUiState;

static AethorAppDebugUiState application_debug_ui;

/** @brief Validates local authorization again at the ArmControlTask boundary. */
#if !AETHOR_ADRC_BENCH
static DebugUiReason aethor_app_debug_ui_validate_start(
    const ProtocolCommand *command, const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us,
    bool preflight_complete);
#endif
/** @brief Routes only LOCAL_UI result heads without formatting USB text. */
#if !AETHOR_ADRC_BENCH
static void aethor_app_debug_ui_route_results(void);
#endif
/** @brief ArmControlTask independently detects local UI or Protocol service failure. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_debug_ui_service_health(uint64_t timestamp_us);
#endif
/** @brief Copies local ownership under critical before the USB watchdog decision. */
#if !AETHOR_ADRC_BENCH
static bool aethor_app_debug_ui_owns_link_lifecycle(void);
#endif
/** @brief Checks command ownership before optional idle register monitoring. */
static bool aethor_app_debug_ui_executor_busy(void);
/** @brief Advances the local epoch and authority under the caller's critical boundary. */
static void aethor_app_debug_ui_revoke(DebugUiAuthority authority);
/** @brief Retains required post-cleanup feedback for every original target. Caller holds critical. */
static void aethor_app_debug_ui_lock_targets(uint8_t motor_mask, uint64_t timestamp_us);
/** @brief Arm independently consumes saturated STOP intents without inventing result identities. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_debug_ui_service_stop_intent(uint64_t timestamp_us, uint8_t *result_generated);
#endif
/** @brief Copies immutable command source and physical disable evidence to its terminal. */
static void aethor_app_debug_ui_result_metadata(
    ProtocolCommandResult *result, const ProtocolCommand *command, uint64_t timestamp_us);

static void aethor_app_update_protocol_context(uint64_t timestamp_us);
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_start_one_shot_hold(uint64_t timestamp_us);
#endif

/** @brief Enters the injected task boundary when a platform installed one. */
static void aethor_app_enter_task_critical(void)
{
    if (application_enter_task_critical_hook != NULL)
    {
        application_enter_task_critical_hook();
    }
}

/** @brief Exits the task boundary paired with the latest local entry. */
static void aethor_app_exit_task_critical(void)
{
    if (application_exit_task_critical_hook != NULL)
    {
        application_exit_task_critical_hook();
    }
}

/** @brief Installs or clears paired platform task-critical hooks. */
void aethor_app_set_task_critical_hooks(
    AethorAppTaskCriticalHook enter_hook,
    AethorAppTaskCriticalHook exit_hook)
{
    if ((enter_hook == NULL) || (exit_hook == NULL))
    {
        application_enter_task_critical_hook = NULL;
        application_exit_task_critical_hook = NULL;
        return;
    }
    application_enter_task_critical_hook = enter_hook;
    application_exit_task_critical_hook = exit_hook;
}

/** @brief Prevents compiler reordering across SPSC mailbox ownership edges. */
#if !AETHOR_ADRC_BENCH
static void aethor_app_compiler_barrier(void)
{
#if defined(__CC_ARM)
    __schedule_barrier();
#elif defined(__GNUC__) || defined(__clang__)
    __asm__ volatile ("" ::: "memory");
#else
    volatile uint32_t barrier_value = 0U;
    (void)barrier_value;
#endif
}
#endif

/**
 * @brief Applies one transport fault from the ArmControlTask ownership domain.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_apply_transport_fault(uint32_t detail,
                                                uint64_t timestamp_us);
#endif

/**
 * @brief Consumes and merges every fully published transport fault event.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_take_transport_fault(uint32_t *detail,
                                               uint64_t *timestamp_us)
{
    uint8_t event_count = 0U;

    if ((detail == NULL) || (timestamp_us == NULL))
    {
        return 0U;
    }
    *detail = 0U;
    *timestamp_us = 0U;
    while (application_transport_fault_read_sequence !=
           application_transport_fault_write_sequence)
    {
        uint8_t slot_index;
        AethorAppTransportFaultEvent event;

        aethor_app_compiler_barrier();
        slot_index = (uint8_t)(application_transport_fault_read_sequence %
                               AETHOR_APP_TRANSPORT_FAULT_CAPACITY);
        event = application_transport_faults[slot_index];
        aethor_app_compiler_barrier();
        ++application_transport_fault_read_sequence;
        *detail |= event.detail;
        if (event_count == 0U)
        {
            *timestamp_us = event.timestamp_us;
        }
        ++event_count;
    }
    return (uint8_t)(event_count != 0U);
}
#endif

/**
 * @brief Calculates a finite representable travel, settle, and safety timeout.
 */
#if !AETHOR_ADRC_BENCH
static bool aethor_app_calculate_one_shot_motion_timeout_us(
    float current_position_rad,
    float target_position_rad,
    float speed_rad_s,
    uint64_t *timeout_us)
{
    double travel_time_us;
    double total_timeout_us;

    if ((timeout_us == NULL) || !isfinite(current_position_rad) ||
        !isfinite(target_position_rad) || !isfinite(speed_rad_s) ||
        (speed_rad_s <= 0.0F))
    {
        return false;
    }

    travel_time_us =
        fabs((double)target_position_rad - (double)current_position_rad) /
        (double)speed_rad_s * 1000000.0;
    total_timeout_us = travel_time_us +
                       (double)AETHOR_APP_MOTION_SETTLE_US +
                       (double)AETHOR_APP_MOTION_TIMEOUT_MARGIN_US;
    if (!isfinite(total_timeout_us) ||
        (total_timeout_us >= (double)UINT64_MAX))
    {
        return false;
    }

    *timeout_us = (uint64_t)ceil(total_timeout_us);
    return true;
}
#endif

/** @brief Adds a timeout only when the absolute deadline is finite and representable. */
#if !AETHOR_ADRC_BENCH
static bool aethor_app_calculate_deadline_us(uint64_t timestamp_us,
                                             uint64_t interval_us,
                                             uint64_t *deadline_us)
{
    if ((deadline_us == NULL) ||
        (interval_us >= (UINT64_MAX - timestamp_us)))
    {
        return false;
    }
    *deadline_us = timestamp_us + interval_us;
    return true;
}
#endif

/** @brief Reports whether the accepted one-shot action owns its link lifecycle. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_active_action_owns_link_lifecycle(void)
{
    if ((application_action.command.type !=
         PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) &&
        (application_action.command.type != PROTOCOL_COMMAND_MIT_ACTION) &&
        !((application_action.command.type == PROTOCOL_COMMAND_SET_MODE) &&
          (application_action.command.bench_relative_scope != 0U)))
    {
        return 0U;
    }
    switch (application_action.state)
    {
        case AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY:
        case AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH:
        case AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT:
            return 1U;
        case AETHOR_APP_ACTION_IDLE:
        case AETHOR_APP_ACTION_MODE_SWITCH:
        case AETHOR_APP_ACTION_DISCOVERY:
        case AETHOR_APP_ACTION_BENCH_MODE_SWITCH:
        case AETHOR_APP_ACTION_ENABLE_WAIT:
        case AETHOR_APP_ACTION_BENCH_MOVE_WAIT:
        case AETHOR_APP_ACTION_DISABLE_WAIT:
        case AETHOR_APP_ACTION_CLEAR_FAULT_WAIT:
        case AETHOR_APP_ACTION_MOTION:
        case AETHOR_APP_ACTION_CONTROLLED_STOP:
        default:
            return 0U;
    }
}
#endif

/** @brief Maps the current one-shot state to stable terminal-result stage metadata. */
static ProtocolCommandStage aethor_app_current_one_shot_stage(void)
{
    switch (application_action.state)
    {
        case AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY:
            return PROTOCOL_COMMAND_STAGE_DISCOVERY;
        case AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT:
            return PROTOCOL_COMMAND_STAGE_VALIDATE;
        case AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH:
            return PROTOCOL_COMMAND_STAGE_MODE;
        case AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT:
            return PROTOCOL_COMMAND_STAGE_CLEAR;
        case AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT:
            return PROTOCOL_COMMAND_STAGE_ENABLE;
        case AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT:
            return PROTOCOL_COMMAND_STAGE_MOTION;
        case AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT:
            return PROTOCOL_COMMAND_STAGE_HOLD;
        case AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT:
            return PROTOCOL_COMMAND_STAGE_DISABLE;
        case AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT:
            return (application_action.failed_stage !=
                    PROTOCOL_COMMAND_STAGE_NONE)
                       ? application_action.failed_stage
                       : PROTOCOL_COMMAND_STAGE_DISABLE;
        case AETHOR_APP_ACTION_IDLE:
        case AETHOR_APP_ACTION_MODE_SWITCH:
        case AETHOR_APP_ACTION_DISCOVERY:
        case AETHOR_APP_ACTION_BENCH_MODE_SWITCH:
        case AETHOR_APP_ACTION_ENABLE_WAIT:
        case AETHOR_APP_ACTION_BENCH_MOVE_WAIT:
        case AETHOR_APP_ACTION_DISABLE_WAIT:
        case AETHOR_APP_ACTION_CLEAR_FAULT_WAIT:
        case AETHOR_APP_ACTION_MOTION:
        case AETHOR_APP_ACTION_CONTROLLED_STOP:
        default:
            return PROTOCOL_COMMAND_STAGE_NONE;
    }
}

/**
 * @brief Records only the first stable one-shot failure metadata.
 * @param failed_stage Stage associated with the original failure.
 * @param failure_error Stable public error associated with the failure.
 * @param failed_motor_number One-based failed motor number, or zero if unknown.
 */
#if !AETHOR_ADRC_BENCH
static void aethor_app_record_one_shot_failure_once(
    ProtocolCommandStage failed_stage,
    ProtocolCommandError failure_error,
    uint8_t failed_motor_number)
{
    if (application_action.failed_stage == PROTOCOL_COMMAND_STAGE_NONE)
    {
        application_action.failed_stage = failed_stage;
        application_action.failure_error = failure_error;
        application_action.failed_motor_number = failed_motor_number;
    }
}
#endif

/** @brief Maps the public arm mode to the vendor identifier offset. */
#if !AETHOR_ADRC_BENCH
static S3519ControlMode aethor_app_vendor_mode(ArmControlMode control_mode)
{
    return (control_mode == ARM_CONTROL_MODE_MIT)
               ? S3519_CONTROL_MODE_MIT
               : S3519_CONTROL_MODE_POSITION_VELOCITY;
}
#endif

/** @brief Returns the vendor mode explicitly owned by the active one-shot command. */
#if !AETHOR_ADRC_BENCH
static S3519ControlMode aethor_app_one_shot_vendor_mode(void)
{
    return aethor_app_vendor_mode(application_action.command.control_mode);
}
#endif

/** @brief Reports whether fresh selected motors are disabled and nearly stationary. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_selected_motors_are_safe_for_mode_switch(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint8_t motor_mask)
{
    uint8_t joint_index;

    if ((motor_snapshot == NULL) ||
        ((motor_snapshot->valid_joint_mask & motor_mask) != motor_mask))
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);

        if ((motor_mask & joint_bit) == 0U)
        {
            continue;
        }
        if ((motor_snapshot->joints[joint_index].driver_state !=
             S3519_DRIVER_STATE_DISABLED) ||
            (motor_snapshot->joints[joint_index].fault_flags != 0U) ||
            (fabsf(motor_snapshot->joints[joint_index].velocity_rad_s) >
             AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S))
        {
            return 0U;
        }
    }
    return 1U;
}
#endif

/** @brief Returns the number of terminal results retained by the app FIFO. */
static uint8_t aethor_app_deferred_result_count(void)
{
    return (uint8_t)(application_deferred_result_write_sequence -
                     application_deferred_result_read_sequence);
}

/** @brief Reports whether one more terminal can be retained without loss. */
static uint8_t aethor_app_deferred_result_has_capacity(void)
{
    return (uint8_t)(aethor_app_deferred_result_count() <
                     AETHOR_APP_DEFERRED_RESULT_CAPACITY);
}

/**
 * @brief Submits one result or appends it behind older deferred terminals.
 * @param result Immutable terminal result.
 * @return One only after fixed storage owns the result.
 */
static uint8_t aethor_app_submit_or_defer_result(
    const ProtocolCommandResult *result)
{
    uint8_t slot_index;

    if (result == NULL)
    {
        return 0U;
    }
    if ((aethor_app_deferred_result_count() == 0U) &&
        (protocol_engine_submit_command_result(&application_protocol_engine,
                                               result) != 0U))
    {
        return 1U;
    }
    if (aethor_app_deferred_result_has_capacity() == 0U)
    {
        return 0U;
    }
    slot_index = (uint8_t)(application_deferred_result_write_sequence %
                           AETHOR_APP_DEFERRED_RESULT_CAPACITY);
    application_deferred_results[slot_index] = *result;
    ++application_deferred_result_write_sequence;
    return 1U;
}

/** @brief Moves deferred terminals to ProtocolEngine without reordering them. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_flush_deferred_results(void)
{
    uint8_t flushed = 0U;

    while (aethor_app_deferred_result_count() != 0U)
    {
        uint8_t slot_index =
            (uint8_t)(application_deferred_result_read_sequence %
                      AETHOR_APP_DEFERRED_RESULT_CAPACITY);

        if (protocol_engine_submit_command_result(
                &application_protocol_engine,
                &application_deferred_results[slot_index]) == 0U)
        {
            break;
        }
        ++application_deferred_result_read_sequence;
        flushed = 1U;
    }
    return flushed;
}
#endif

/** @brief Retains one explicit terminal for a queued command. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_submit_queued_command_result(
    const ProtocolCommand *command,
    ProtocolCommandResultCode code,
    uint16_t detail,
    uint64_t timestamp_us)
{
    ProtocolCommandResult result;

    if (command == NULL)
    {
        return 0U;
    }
    memset(&result, 0, sizeof(result));
    result.request_id = command->request_id;
    result.session_id = command->session_id;
    aethor_app_debug_ui_result_metadata(&result, command, timestamp_us);
    result.type = command->type;
    result.code = code;
    result.detail = detail;
    result.accepted_at_us = command->accepted_at_us;
    result.completed_at_us = timestamp_us;
    result.motor_mask = command->motor_mask;
    result.bench_relative_scope = command->bench_relative_scope;
    if (code == PROTOCOL_COMMAND_RESULT_FAILED)
    {
        result.error = PROTOCOL_COMMAND_ERROR_ACTION_FAILED;
    }
    return aethor_app_submit_or_defer_result(&result);
}
#endif

/** @brief Retains one cancelled terminal for a queued STOP-preempted command. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_cancel_queued_command(
    const ProtocolCommand *command,
    uint64_t timestamp_us)
{
    return aethor_app_submit_queued_command_result(
        command,
        PROTOCOL_COMMAND_RESULT_CANCELLED,
        0U,
        timestamp_us);
}
#endif

/** @brief Retains a failed terminal for a globally cancelled queued motion. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_fail_queued_motion(uint16_t detail,
                                             uint64_t timestamp_us)
{
    ProtocolCommand queued_motion;
    ProtocolCommand pending_stop;

    if (protocol_engine_take_queued_active_motion(
            &application_protocol_engine,
            &queued_motion) == 0U)
    {
        return 0U;
    }
    (void)protocol_engine_widen_pending_stop_mask(
        &application_protocol_engine,
        queued_motion.motor_mask,
        &pending_stop);
    return aethor_app_submit_queued_command_result(
        &queued_motion,
        PROTOCOL_COMMAND_RESULT_FAILED,
        detail,
        timestamp_us);
}
#endif

/** @brief Preserves the active one-shot/STOP scope in a pending newer STOP. */
#if !AETHOR_ADRC_BENCH
static void aethor_app_widen_pending_stop_before_global_cancel(void)
{
    ProtocolCommand pending_stop;
    uint8_t inherited_motor_mask = application_debug_ui.unconfirmed_disable_mask;

    if (application_action.state != AETHOR_APP_ACTION_IDLE)
    {
        inherited_motor_mask |= application_action.command.motor_mask;
    }
    if (inherited_motor_mask != 0U)
    {
        (void)protocol_engine_widen_pending_stop_mask(
            &application_protocol_engine,
            inherited_motor_mask,
            &pending_stop);
    }
}
#endif

/** @brief Completes the active command and clears its execution gate. */
static uint8_t aethor_app_complete_action(ProtocolCommandResultCode code,
                                          uint16_t detail,
                                          uint64_t timestamp_us)
{
    ProtocolCommandResult result;

    memset(&result, 0, sizeof(result));
    result.request_id = application_action.command.request_id;
    result.session_id = application_action.command.session_id;
    aethor_app_debug_ui_result_metadata(&result, &application_action.command, timestamp_us);
    if (result.local_reason == DEBUG_UI_REASON_NONE && code == PROTOCOL_COMMAND_RESULT_FAILED &&
        application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI)
    { result.local_reason = application_action.local_failure_reason; }
    result.type = application_action.command.type;
    result.code = code;
    result.detail = detail;
    result.accepted_at_us = application_action.command.accepted_at_us;
    result.completed_at_us = timestamp_us;
    result.motor_mask = application_action.command.motor_mask;
    result.bench_relative_scope =
        application_action.command.bench_relative_scope;
    result.stage = application_action.failed_stage;
    result.error = application_action.failure_error;
    if ((code == PROTOCOL_COMMAND_RESULT_FAILED) &&
        ((application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) ||
         (application_action.command.cleanup_after_stop != 0U)) &&
        (result.stage == PROTOCOL_COMMAND_STAGE_NONE))
    {
        result.stage = (application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT)
            ? PROTOCOL_COMMAND_STAGE_DISABLE : PROTOCOL_COMMAND_STAGE_MOTION;
        result.error = PROTOCOL_COMMAND_ERROR_FEEDBACK_TIMEOUT;
    }
    result.failed_motor_number = application_action.failed_motor_number;
    if ((result.type == PROTOCOL_COMMAND_INIT_MOTORS) &&
        (result.code == PROTOCOL_COMMAND_RESULT_COMPLETED))
    {
        uint8_t joint_index;
        /* Capture immutable evidence before a later action can change discovery. */
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            uint8_t joint_bit = (uint8_t)(1U << joint_index);
            uint16_t fields = application_motor_runtime.discovery.results[joint_index].verified_fields_mask;
            if ((result.motor_mask & joint_bit) == 0U) { continue; }
            if ((fields & MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK) == MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK)
                result.discovery_identity_mask |= joint_bit;
            if ((fields & MOTOR_DISCOVERY_MODE_FIELDS_MASK) == MOTOR_DISCOVERY_MODE_FIELDS_MASK)
                result.discovery_mode_mask |= joint_bit;
            if ((fields & MOTOR_DISCOVERY_RANGE_FIELDS_MASK) == MOTOR_DISCOVERY_RANGE_FIELDS_MASK)
                result.discovery_ranges_mask |= joint_bit;
            if ((fields & MOTOR_DISCOVERY_VERSION_FIELDS_MASK) == MOTOR_DISCOVERY_VERSION_FIELDS_MASK)
                result.discovery_version_mask |= joint_bit;
        }
    }
    if (application_action.command.type == PROTOCOL_COMMAND_MOVE_JOINTS)
    {
        result.auxiliary_values[0] =
            application_action.maximum_following_error_deg;
    }
    if (aethor_app_submit_or_defer_result(&result) == 0U)
    {
        return 0U;
    }
    if ((code == PROTOCOL_COMMAND_RESULT_FAILED) &&
        ((application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) ||
         (application_action.command.cleanup_after_stop != 0U)))
    {
        aethor_app_enter_task_critical();
        if (result.disabled_confirmed == 0U)
        { aethor_app_debug_ui_lock_targets(result.motor_mask, timestamp_us); }
        if (application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_FAULT)
        { aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT); }
        aethor_app_exit_task_critical();
    }
    memset(&application_action, 0, sizeof(application_action));
#if !AETHOR_ADRC_BENCH
    application_last_service_timestamp_us = 0U;
#endif
    return 1U;
}

/** @brief Accumulates the maximum absolute trajectory-following error in degrees. */
#if !AETHOR_ADRC_BENCH
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
#endif

/** @brief Converts one coherent public joint snapshot back to SI radians. */
#if !AETHOR_ADRC_BENCH
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
#endif

/** @brief Builds the next complete seven-frame motion control group. */
#if !AETHOR_ADRC_BENCH
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
#endif

/** @brief Builds the next controlled-stop group for the confirmed motor mode. */
#if !AETHOR_ADRC_BENCH
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
#endif

/** @brief Starts fail-safe disable frames after a motion-control failure. */
static void aethor_app_latch_motion_failure(uint16_t detail,
                                            uint64_t timestamp_us)
{
    (void)arm_controller_latch_runtime_fault(&application_controller,
                                             ARM_FAULT_MOTION_CONTROL,
                                             detail,
                                             timestamp_us);
#if !AETHOR_ADRC_BENCH
    application_emergency_disable_read_index = 0U;
#endif
    (void)motor_runtime_build_emergency_disable(
        &application_motor_runtime,
        &application_emergency_disable_batch);
}

/** @brief Returns the first formal-arm driver fault detail, or zero. */
#if !AETHOR_ADRC_BENCH
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
#endif

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
#if !AETHOR_ADRC_BENCH
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
#endif

/**
 * @brief Checks that each selected feedback timestamp advanced from its baseline.
 * @param snapshot Current freshness-filtered motor snapshot.
 * @param motor_mask Selected J1-J7 mask.
 * @param baseline_us Per-joint timestamps captured before the current batch.
 * @return One only when every selected joint published a newer valid sample.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_selected_feedback_advanced(
    const MotorFeedbackSnapshot *snapshot,
    uint8_t motor_mask,
    const uint64_t baseline_us[ARM_JOINT_COUNT])
{
    uint8_t joint_index;

    if ((snapshot == NULL) || (baseline_us == NULL) ||
        ((snapshot->valid_joint_mask & motor_mask) != motor_mask))
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);

        if (((motor_mask & joint_bit) != 0U) &&
            (snapshot->joints[joint_index].timestamp_us <=
             baseline_us[joint_index]))
        {
            return 0U;
        }
    }
    return 1U;
}
#endif

/**
 * @brief Captures the latest per-joint feedback generation timestamps.
 * @param snapshot Coherent snapshot captured before a new command batch.
 */
#if !AETHOR_ADRC_BENCH
static void aethor_app_capture_feedback_baseline(
    const MotorFeedbackSnapshot *snapshot)
{
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        application_action.phase_feedback_baseline_us[joint_index] =
            (snapshot != NULL) ? snapshot->joints[joint_index].timestamp_us : 0U;
    }
}
#endif

/**
 * @brief Maps motor-runtime admission failures to the fixed public protocol error set.
 * @param runtime_status Motor runtime result to normalize.
 * @return Stable public command error.
 */
#if !AETHOR_ADRC_BENCH
static ProtocolCommandError aethor_app_map_runtime_error(
    MotorRuntimeStatus runtime_status)
{
    switch (runtime_status)
    {
        case MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE:
        case MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR:
        case MOTOR_RUNTIME_STATUS_NOT_INITIALIZED:
            return PROTOCOL_COMMAND_ERROR_NOT_READY;
        case MOTOR_RUNTIME_STATUS_POSITION_OUT_OF_RANGE:
            return PROTOCOL_COMMAND_ERROR_POSITION_OUT_OF_RANGE;
        case MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE:
            return PROTOCOL_COMMAND_ERROR_SPEED_OUT_OF_RANGE;
        case MOTOR_RUNTIME_STATUS_FAULT_PRESENT:
            return PROTOCOL_COMMAND_ERROR_FAULT_PRESENT;
        case MOTOR_RUNTIME_STATUS_STALE_FEEDBACK:
            return PROTOCOL_COMMAND_ERROR_STALE_FEEDBACK;
        case MOTOR_RUNTIME_STATUS_OK:
            return PROTOCOL_COMMAND_ERROR_NONE;
        case MOTOR_RUNTIME_STATUS_FRAME_READY:
        case MOTOR_RUNTIME_STATUS_WAITING:
        case MOTOR_RUNTIME_STATUS_DISCOVERY_COMPLETE:
        case MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT:
        case MOTOR_RUNTIME_STATUS_CODEC_ERROR:
        case MOTOR_RUNTIME_STATUS_ID_MISMATCH:
        case MOTOR_RUNTIME_STATUS_ACTION_COMPLETE:
        case MOTOR_RUNTIME_STATUS_ACTION_FAILED:
        default:
            return PROTOCOL_COMMAND_ERROR_ACTION_FAILED;
    }
}
#endif

/**
 * @brief Records a one-shot failure and starts mode-safe selected cleanup.
 * @param failed_stage Setup stage that failed.
 * @param failure_error Stable public failure reason.
 * @param failed_motor_number One-based failed motor number, or zero when unknown.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when a terminal result was queued, otherwise zero while cleanup waits.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_begin_one_shot_cleanup(
    ProtocolCommandStage failed_stage,
    ProtocolCommandError failure_error,
    uint8_t failed_motor_number,
    uint64_t timestamp_us)
{
    MotorRuntimeStatus runtime_status;
    MotorRuntimeStatus snapshot_status;
    MotorFeedbackSnapshot motor_snapshot;
    float hold_position_rad[ARM_JOINT_COUNT] = {0.0F};
    float hold_speed_rad_s[ARM_JOINT_COUNT] = {0.0F};
    uint8_t joint_index;
    uint8_t local_health_cleanup;

    aethor_app_record_one_shot_failure_once(failed_stage,
                                            failure_error,
                                            failed_motor_number);
    memset(&motor_snapshot, 0, sizeof(motor_snapshot));
    snapshot_status = motor_runtime_get_snapshot(
        &application_motor_runtime,
        timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US,
        &motor_snapshot);
    aethor_app_enter_task_critical();
    local_health_cleanup = (uint8_t)(
        (application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
        (application_debug_ui.health_stop_started != 0U));
    aethor_app_exit_task_critical();
    application_action.cleanup_disable_mask =
        ((application_action.failed_stage == PROTOCOL_COMMAND_STAGE_VALIDATE) &&
         (local_health_cleanup == 0U))
            ? 0U
            : application_action.command.motor_mask;
    if (application_action.cleanup_disable_mask == 0U)
    {
        (void)motor_runtime_abort_active_parameter_sequences(
            &application_motor_runtime,
            timestamp_us);
        return aethor_app_complete_action(
            PROTOCOL_COMMAND_RESULT_FAILED,
            (uint16_t)application_action.failure_error,
            timestamp_us);
    }

    if ((application_action.failed_stage ==
         PROTOCOL_COMMAND_STAGE_DISCOVERY) ||
        (application_action.failed_stage == PROTOCOL_COMMAND_STAGE_MODE))
    {
        runtime_status = motor_runtime_build_emergency_disable_subset(
            &application_motor_runtime,
            application_action.cleanup_disable_mask,
            &application_action.frames);
        (void)motor_runtime_abort_active_parameter_sequences(
            &application_motor_runtime,
            timestamp_us);
        if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
        {
            return aethor_app_complete_action(
                PROTOCOL_COMMAND_RESULT_FAILED,
                (uint16_t)application_action.failure_error,
                timestamp_us);
        }
        application_action.state =
            AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT;
        application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
        application_action.frame_read_index = 0U;
        aethor_app_capture_feedback_baseline(&motor_snapshot);
    }
    else if (((application_action.enabled_by_action_mask &
               application_action.cleanup_disable_mask) ==
              application_action.cleanup_disable_mask) &&
             (snapshot_status == MOTOR_RUNTIME_STATUS_OK) &&
             ((motor_snapshot.valid_joint_mask &
               application_action.cleanup_disable_mask) ==
              application_action.cleanup_disable_mask) &&
             (aethor_app_selected_motors_are_fault_free(
                  &motor_snapshot,
                  application_action.cleanup_disable_mask) != 0U) &&
             (aethor_app_selected_motors_have_state(
                  &motor_snapshot,
                  application_action.cleanup_disable_mask,
                  S3519_DRIVER_STATE_ENABLED) != 0U))
    {
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            hold_position_rad[joint_index] =
                motor_snapshot.joints[joint_index].position_rad;
        }
        runtime_status =
            (application_action.command.control_mode == ARM_CONTROL_MODE_MIT)
                ? motor_runtime_build_mit_subset(
                      &application_motor_runtime,
                      application_action.cleanup_disable_mask,
                      hold_position_rad,
                      hold_speed_rad_s,
                      application_action.command.mit_kp,
                      application_action.command.mit_kd,
                      application_action.command.mit_torque_ff_nm,
                      &application_action.frames)
                : motor_runtime_build_position_velocity_subset(
                      &application_motor_runtime,
                      application_action.cleanup_disable_mask,
                      hold_position_rad,
                      hold_speed_rad_s,
                      &application_action.frames);
        (void)motor_runtime_abort_active_parameter_sequences(
            &application_motor_runtime,
            timestamp_us);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            application_action.state =
                AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT;
            application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
            application_action.frame_read_index = 0U;
        }
        else
        {
            runtime_status = motor_runtime_build_mode_command_batch(
                &application_motor_runtime,
                aethor_app_one_shot_vendor_mode(),
                S3519_MODE_COMMAND_DISABLE,
                application_action.cleanup_disable_mask,
                &application_action.frames);
            application_action.state =
                AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT;
            application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
            application_action.frame_read_index = 0U;
            aethor_app_capture_feedback_baseline(&motor_snapshot);
        }
    }
    else
    {
        (void)motor_runtime_abort_active_parameter_sequences(
            &application_motor_runtime,
            timestamp_us);
        runtime_status = motor_runtime_build_mode_command_batch(
            &application_motor_runtime,
            aethor_app_one_shot_vendor_mode(),
            S3519_MODE_COMMAND_DISABLE,
            application_action.cleanup_disable_mask,
            &application_action.frames);
        application_action.state =
            AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT;
        application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
        application_action.frame_read_index = 0U;
        aethor_app_capture_feedback_baseline(&motor_snapshot);
    }
    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return aethor_app_complete_action(
            PROTOCOL_COMMAND_RESULT_FAILED,
            (uint16_t)application_action.failure_error,
            timestamp_us);
    }
    if (!aethor_app_calculate_deadline_us(timestamp_us,
                                          AETHOR_APP_ACTION_TIMEOUT_US,
                                          &application_action.deadline_us))
    {
        return aethor_app_complete_action(
            PROTOCOL_COMMAND_RESULT_FAILED,
            (uint16_t)application_action.failure_error,
            timestamp_us);
    }
    return 0U;
}
#endif

/**
 * @brief Replaces cleanup HOLD work with a bounded selected POS_VEL disable.
 * @param motor_snapshot Snapshot captured before the disable batch.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when a terminal result was queued, otherwise zero while cleanup waits.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_start_one_shot_cleanup_disable(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    MotorRuntimeStatus runtime_status = motor_runtime_build_mode_command_batch(
        &application_motor_runtime,
        aethor_app_one_shot_vendor_mode(),
        S3519_MODE_COMMAND_DISABLE,
        application_action.cleanup_disable_mask,
        &application_action.frames);

    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return aethor_app_complete_action(
            PROTOCOL_COMMAND_RESULT_FAILED,
            (uint16_t)application_action.failure_error,
            timestamp_us);
    }
    application_action.state =
        AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT;
    application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
    application_action.frame_read_index = 0U;
    aethor_app_capture_feedback_baseline(motor_snapshot);
    if (!aethor_app_calculate_deadline_us(timestamp_us,
                                          AETHOR_APP_ACTION_TIMEOUT_US,
                                          &application_action.deadline_us))
    {
        return aethor_app_complete_action(
            PROTOCOL_COMMAND_RESULT_FAILED,
            (uint16_t)application_action.failure_error,
            timestamp_us);
    }
    return 0U;
}
#endif

/** @brief Converts selected same-model bench motor positions to LCD output-shaft coordinates. */
static float aethor_app_lcd_position_ratio(uint8_t joint_index)
{
#if AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
    return joint_index < ARM_JOINT_COUNT &&
        (AETHOR_S3519_SAME_MODEL_MASK & (1U << joint_index)) != 0U ?
        AETHOR_S3519_MOTOR7_POSITION_RATIO : 1.0F;
#else
    (void)joint_index;
    return 1.0F;
#endif
}

/** @brief Applies mixed position/velocity units only to local LCD motion timing. */
#if !AETHOR_ADRC_BENCH
static float aethor_app_local_motion_position_ratio(uint8_t joint_index)
{
    return application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
        application_action.command.bench_relative_scope != 0U ?
        aethor_app_lcd_position_ratio(joint_index) : 1.0F;
}
#endif

/** @brief Selects fixed endpoints only for LCD single-motor POS commissioning commands. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_one_shot_uses_local_pos_endpoint(void)
{
    uint8_t motor_mask = application_action.command.motor_mask;
    return (uint8_t)(application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
        application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED &&
        application_action.command.bench_relative_scope != 0U && motor_mask != 0U &&
        (motor_mask & (motor_mask - 1U)) == 0U);
}
#endif

/** @brief Requires fresh stable endpoint feedback and bounds sustained lack of progress. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_local_pos_endpoint_arrived(
    const MotorFeedbackSnapshot *motor_snapshot, uint64_t timestamp_us, uint8_t *failed)
{
    uint8_t joint_index;
    *failed = 0U;
    if (motor_snapshot == NULL) { return 0U; }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const MotorJointFeedback *feedback = &motor_snapshot->joints[joint_index];
        float position_error_rad;
        uint64_t feedback_timestamp_us = feedback->timestamp_us;
        if ((application_action.command.motor_mask & (1U << joint_index)) == 0U) { continue; }
        if ((motor_snapshot->valid_joint_mask & (1U << joint_index)) == 0U ||
            !isfinite(feedback->position_rad) || feedback->fault_flags != 0U ||
            feedback_timestamp_us > timestamp_us ||
            timestamp_us - feedback_timestamp_us > MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US)
        {
            application_action.local_pos_settle_start_us = 0U;
            return 0U;
        }
        if (feedback_timestamp_us <= application_action.local_pos_last_feedback_us) { return 0U; }
        if (feedback_timestamp_us - application_action.local_pos_last_feedback_us >
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US)
        {
            application_action.local_pos_settle_start_us = 0U;
        }
        application_action.local_pos_last_feedback_us = feedback_timestamp_us;
        position_error_rad = fabsf(feedback->position_rad - application_action.target_position_rad[joint_index]) /
            aethor_app_local_motion_position_ratio(joint_index);
        if (position_error_rad <= 0.5F * AETHOR_APP_DEG_TO_RAD)
        {
            application_action.local_pos_progress_timestamp_us = timestamp_us;
            if (application_action.local_pos_settle_start_us == 0U)
            {
                application_action.local_pos_settle_start_us = feedback_timestamp_us;
            }
            return (uint8_t)(feedback_timestamp_us - application_action.local_pos_settle_start_us >=
                AETHOR_APP_MOTION_SETTLE_US);
        }
        application_action.local_pos_settle_start_us = 0U;
        if (application_action.local_pos_progress_error_rad - position_error_rad >=
            AETHOR_APP_LOCAL_POS_PROGRESS_RAD)
        {
            application_action.local_pos_progress_error_rad = position_error_rad;
            application_action.local_pos_progress_timestamp_us = timestamp_us;
        }
        else if (timestamp_us - application_action.local_pos_progress_timestamp_us >=
                 AETHOR_APP_LOCAL_POS_PROGRESS_TIMEOUT_US)
        {
            *failed = 1U;
        }
        return 0U;
    }
    return 0U;
}
#endif

/** @brief Validates LCD POS travel bounds or USB POS timestamped reference tracking.
 * The 0.5 degree envelope is a commissioning gate, not a calibrated mechanical limit.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_bench_pos_feedback_safe(
    const MotorFeedbackSnapshot *motor_snapshot, uint8_t joint_index, uint64_t timestamp_us)
{
    JointMotionSample reference_sample;
    DebugUiMotorProfile profile;
    float position_rad = motor_snapshot->joints[joint_index].position_rad;
    uint64_t reference_timestamp_us = motor_snapshot->joints[joint_index].timestamp_us;
    if (!isfinite(position_rad) || (reference_timestamp_us > timestamp_us))
    {
        return 0U;
    }
    if (aethor_app_one_shot_uses_local_pos_endpoint())
    {
        float start = application_action.motion_plan.start_position_rad[joint_index];
        float target = application_action.target_position_rad[joint_index];
        float margin = 0.5F * AETHOR_APP_DEG_TO_RAD *
            aethor_app_local_motion_position_ratio(joint_index);
        if ((position_rad < fminf(start, target) - margin) ||
            (position_rad > fmaxf(start, target) + margin))
        {
            application_action.local_failure_reason = DEBUG_UI_REASON_OUT_OF_RANGE;
            return 0U;
        }
    }
    else
    {
        if (reference_timestamp_us > application_action.last_pos_reference_timestamp_us)
        {
            reference_timestamp_us = application_action.last_pos_reference_timestamp_us;
        }
        if (joint_motion_sample(&application_action.motion_plan, reference_timestamp_us,
                                &reference_sample) != JOINT_MOTION_STATUS_OK)
        {
            return 0U;
        }
        if (fabsf(position_rad - reference_sample.position_rad[joint_index]) >
            (0.5F * AETHOR_APP_DEG_TO_RAD))
        {
            return 0U;
        }
    }
    if (application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI)
    {
        if (!aethor_app_debug_ui_get_motor_profile((uint8_t)(joint_index + 1U), &profile) ||
            !profile.pos_valid || !isfinite(profile.position_min_rad) ||
            !isfinite(profile.position_max_rad) ||
            (position_rad < profile.position_min_rad) ||
            (position_rad > profile.position_max_rad))
        {
            return 0U;
        }
    }
    return 1U;
}
#endif

/**
 * @brief Fails energized one-shot phases on the first selected stale or faulted motor.
 * @param motor_snapshot Current freshness-filtered motor feedback.
 * @param timestamp_us Current monotonic timestamp.
 * @param failure_detected Destination set when cleanup or a terminal result started.
 * @return One when failure handling queued a terminal result, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_check_one_shot_motor_safety(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us,
    uint8_t *failure_detected)
{
    ProtocolCommandStage failed_stage;
    uint8_t joint_index;

    if (failure_detected == NULL)
    {
        return 0U;
    }
    *failure_detected = 0U;
    switch (application_action.state)
    {
        case AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT:
            failed_stage = PROTOCOL_COMMAND_STAGE_MOTION;
            break;
        case AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT:
            failed_stage = PROTOCOL_COMMAND_STAGE_ENABLE;
            break;
        case AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT:
            failed_stage = PROTOCOL_COMMAND_STAGE_HOLD;
            break;
        case AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT:
            failed_stage = PROTOCOL_COMMAND_STAGE_DISABLE;
            break;
        case AETHOR_APP_ACTION_IDLE:
        case AETHOR_APP_ACTION_MODE_SWITCH:
        case AETHOR_APP_ACTION_DISCOVERY:
        case AETHOR_APP_ACTION_BENCH_MODE_SWITCH:
        case AETHOR_APP_ACTION_ENABLE_WAIT:
        case AETHOR_APP_ACTION_BENCH_MOVE_WAIT:
        case AETHOR_APP_ACTION_DISABLE_WAIT:
        case AETHOR_APP_ACTION_CLEAR_FAULT_WAIT:
        case AETHOR_APP_ACTION_MOTION:
        case AETHOR_APP_ACTION_CONTROLLED_STOP:
        case AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY:
        case AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH:
        case AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT:
        case AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT:
        default:
            return 0U;
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);

        if ((application_action.command.motor_mask & joint_bit) == 0U)
        {
            continue;
        }
        if ((motor_snapshot == NULL) ||
            ((motor_snapshot->valid_joint_mask & joint_bit) == 0U))
        {
            *failure_detected = 1U;
            return aethor_app_begin_one_shot_cleanup(
                failed_stage,
                PROTOCOL_COMMAND_ERROR_STALE_FEEDBACK,
                (uint8_t)(joint_index + 1U),
                timestamp_us);
        }
        if (motor_snapshot->joints[joint_index].fault_flags != 0U)
        {
            *failure_detected = 1U;
            return aethor_app_begin_one_shot_cleanup(
                failed_stage,
                PROTOCOL_COMMAND_ERROR_FAULT_PRESENT,
                (uint8_t)(joint_index + 1U),
                timestamp_us);
        }
        /* Enforce local MIT commissioning feedback guards independently of USB monitoring. */
        if ((application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
            (application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION) &&
            (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT))
        {
            const MotorJointFeedback *feedback = &motor_snapshot->joints[joint_index];
            const DebugUiMotorProfile *profile = &application_debug_ui.profiles[joint_index];
            /* One feedback LSB accommodates rounding at the discovered speed boundary. */
            float speed_margin = 2.0F * application_motor_runtime.discovery.results[joint_index].ranges.velocity_max_rad_s / 4095.0F;
            DebugUiReason guard_reason = DEBUG_UI_REASON_NONE;
            if (!isfinite(feedback->torque_nm) || !isfinite(feedback->velocity_rad_s) ||
                !isfinite(feedback->position_rad) || !isfinite(feedback->mos_temperature_c) ||
                !isfinite(feedback->rotor_temperature_c))
                guard_reason = DEBUG_UI_REASON_INVALID_FEEDBACK;
            else if (fabsf(feedback->torque_nm) > AETHOR_APP_LOCAL_MIT_TORQUE_TRIP_NM)
                guard_reason = DEBUG_UI_REASON_TORQUE_LIMIT;
            else if (fabsf(feedback->velocity_rad_s) > profile->mit_max_speed_rad_s + speed_margin)
                guard_reason = DEBUG_UI_REASON_SPEED_LIMIT;
            else if (feedback->position_rad < profile->mit_position_min_rad ||
                     feedback->position_rad > profile->mit_position_max_rad)
                guard_reason = DEBUG_UI_REASON_OUT_OF_RANGE;
            else if (feedback->mos_temperature_c > 55U || feedback->rotor_temperature_c > 55U)
                guard_reason = DEBUG_UI_REASON_TEMPERATURE_LIMIT;
            if (guard_reason != DEBUG_UI_REASON_NONE)
            {
                application_action.local_failure_reason = guard_reason;
                application_debug_ui.last_mit_guard_feedback = *feedback;
                application_debug_ui.last_mit_guard_reason = guard_reason;
                application_debug_ui.last_mit_guard_motor_id = (uint8_t)(joint_index + 1U);
                *failure_detected = 1U;
                return aethor_app_begin_one_shot_cleanup(failed_stage,
                    PROTOCOL_COMMAND_ERROR_ACTION_FAILED, (uint8_t)(joint_index + 1U), timestamp_us);
            }
        }
        if ((application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT) &&
            (application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) &&
            (application_action.command.bench_relative_scope != 0U) &&
            !aethor_app_bench_pos_feedback_safe(motor_snapshot, joint_index, timestamp_us))
        {
            *failure_detected = 1U;
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_MOTION, PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                (uint8_t)(joint_index + 1U), timestamp_us);
        }
    }
    return 0U;
}
#endif

/**
 * @brief Validates fixed one-shot targets and begins the selected POS_VEL mode cycle.
 * @param motor_snapshot Current freshness-filtered feedback snapshot.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when validation queued a terminal failure, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_validate_one_shot_targets(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    uint8_t failed_joint_index = 0U;
    uint8_t joint_index;
    MotorRuntimeStatus runtime_status = MOTOR_RUNTIME_STATUS_OK;

    if ((application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
        ((application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
         (application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION) ||
         (application_action.command.type == PROTOCOL_COMMAND_SET_MODE)))
    {
        /* The queued relative intent becomes an absolute target only after a new disabled ACK. */
        DebugUiReason reason = aethor_app_debug_ui_validate_start(
            &application_action.command, motor_snapshot, timestamp_us, true);
        if (reason != DEBUG_UI_REASON_NONE)
        {
            application_action.local_failure_reason = reason;
            return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_VALIDATE,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 0U, timestamp_us);
        }
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            if (((application_action.command.motor_mask & (1U << joint_index)) != 0U) &&
                (application_action.command.relative_target != 0U))
            {
                application_action.target_position_rad[joint_index] =
                    motor_snapshot->joints[joint_index].position_rad +
                    application_action.command.values[joint_index];
            }
        }
    }
    if ((application_action.command.type !=
         PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) &&
        (aethor_app_selected_motors_are_safe_for_mode_switch(
             motor_snapshot,
             application_action.command.motor_mask) == 0U))
    {
        runtime_status = MOTOR_RUNTIME_STATUS_ACTION_FAILED;
    }
    else if (application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION)
    {
        float validation_velocity_rad_s[ARM_JOINT_COUNT] = {0.0F};

        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            uint8_t joint_bit = (uint8_t)(1U << joint_index);
            MotorPositionVelocityLimits limits;

            if ((application_action.command.motor_mask & joint_bit) == 0U)
            {
                continue;
            }
            if (application_action.command.mit_action ==
                PROTOCOL_MIT_ACTION_HOLD)
            {
                application_action.target_position_rad[joint_index] =
                    motor_snapshot->joints[joint_index].position_rad;
                application_action.target_speed_rad_s[joint_index] = 0.0F;
            }
            else
            {
                runtime_status = motor_runtime_get_position_velocity_limits(
                    &application_motor_runtime,
                    joint_index,
                    &limits);
                if ((runtime_status != MOTOR_RUNTIME_STATUS_OK) ||
                    (application_action.target_speed_rad_s[joint_index] <=
                     0.0F) ||
                    (application_action.target_speed_rad_s[joint_index] >
                     limits.move_speed_limit_rad_s))
                {
                    failed_joint_index = joint_index;
                    runtime_status = MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE;
                    break;
                }
                validation_velocity_rad_s[joint_index] =
                    application_action.target_speed_rad_s[joint_index];
            }
        }
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            MotorEmergencyFrameBatch validation_batch;

            runtime_status = motor_runtime_build_mit_subset(
                &application_motor_runtime,
                application_action.command.motor_mask,
                application_action.target_position_rad,
                validation_velocity_rad_s,
                application_action.command.mit_kp,
                application_action.command.mit_kd,
                application_action.command.mit_torque_ff_nm,
                &validation_batch);
        }
    }
    else if (application_action.command.type ==
             PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED)
    {
        runtime_status = motor_runtime_validate_position_velocity_move_subset(
            &application_motor_runtime,
            motor_snapshot,
            application_action.command.motor_mask,
            application_action.target_position_rad,
            application_action.target_speed_rad_s,
            &failed_joint_index);
    }

    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_VALIDATE,
            aethor_app_map_runtime_error(runtime_status),
            (uint8_t)(failed_joint_index + 1U),
            timestamp_us);
    }
    runtime_status = motor_runtime_begin_control_mode_switch_mask(
        &application_motor_runtime,
        aethor_app_one_shot_vendor_mode(),
        application_action.command.motor_mask);
    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_MODE,
            aethor_app_map_runtime_error(runtime_status),
            0U,
            timestamp_us);
    }
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH;
    application_action.deadline_us = timestamp_us +
                                     AETHOR_APP_ACTION_TIMEOUT_US;
    return 0U;
}
#endif

/**
 * @brief Sends selected fail-safe disable frames before target validation.
 * @param motor_snapshot Snapshot whose per-joint timestamps form the baseline.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when batch setup queued a terminal failure, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_begin_one_shot_preflight_disable(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    uint8_t joint_index;
    MotorRuntimeStatus runtime_status =
        motor_runtime_build_emergency_disable_subset(
            &application_motor_runtime,
            application_action.command.motor_mask,
            &application_action.frames);

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        application_action.phase_feedback_baseline_us[joint_index] =
            (motor_snapshot != NULL)
                ? motor_snapshot->joints[joint_index].timestamp_us
                : 0U;
        if ((application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
            ((application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
             (application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION) ||
             (application_action.command.type == PROTOCOL_COMMAND_SET_MODE)) &&
            (application_action.phase_feedback_baseline_us[joint_index] < timestamp_us))
        {
            application_action.phase_feedback_baseline_us[joint_index] = timestamp_us;
        }
    }

    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_VALIDATE,
            aethor_app_map_runtime_error(runtime_status),
            0U,
            timestamp_us);
    }
    application_action.state =
        AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT;
    application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
    application_action.frame_read_index = 0U;
    if (!aethor_app_calculate_deadline_us(timestamp_us,
                                          AETHOR_APP_ACTION_TIMEOUT_US,
                                          &application_action.deadline_us))
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_VALIDATE,
            PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
            0U,
            timestamp_us);
    }
    return 0U;
}
#endif

/**
 * @brief Validates fresh selected feedback or starts bounded acquisition.
 * @param motor_snapshot Current freshness-filtered feedback snapshot.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when setup queued a terminal failure, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_prepare_one_shot_validation(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    uint8_t motor_mask = application_action.command.motor_mask;

    if ((application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
        ((application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
         (application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION) ||
         (application_action.command.type == PROTOCOL_COMMAND_SET_MODE)))
    {
        return aethor_app_begin_one_shot_preflight_disable(motor_snapshot, timestamp_us);
    }
    if ((motor_snapshot != NULL) &&
        ((motor_snapshot->valid_joint_mask & motor_mask) == motor_mask) &&
        ((application_action.command.type ==
          PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
         (aethor_app_selected_motors_are_safe_for_mode_switch(motor_snapshot,
                                                              motor_mask) !=
          0U)))
    {
        return aethor_app_validate_one_shot_targets(motor_snapshot,
                                                    timestamp_us);
    }
    return aethor_app_begin_one_shot_preflight_disable(motor_snapshot,
                                                       timestamp_us);
}
#endif

/** @brief Use an MCU reference for MIT moves and self-contained bench POS moves. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_one_shot_uses_reference_trajectory(void)
{
    return (uint8_t)((application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION) ||
        ((application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) &&
         (application_action.command.bench_relative_scope != 0U) &&
         !aethor_app_one_shot_uses_local_pos_endpoint()));
}
#endif

/** @brief Sample shared quintic mathematics while preserving the selected wire protocol. */
#if !AETHOR_ADRC_BENCH
static MotorRuntimeStatus aethor_app_build_one_shot_reference(uint64_t timestamp_us)
{
    JointMotionSample sample;
    MotorRuntimeStatus runtime_status;
    if (joint_motion_sample(&application_action.motion_plan, timestamp_us, &sample) != JOINT_MOTION_STATUS_OK)
    {
        return MOTOR_RUNTIME_STATUS_ACTION_FAILED;
    }
    if (application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION)
    {
        uint8_t joint_index;
        /* Quintic positions remain rotor radians; the derivative must match
         * the driver's output-side velocity field, without rescaling gains. */
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            sample.velocity_rad_s[joint_index] /=
                aethor_app_local_motion_position_ratio(joint_index);
        }
        return motor_runtime_build_mit_subset(&application_motor_runtime,
            application_action.command.motor_mask, sample.position_rad, sample.velocity_rad_s,
            application_action.command.mit_kp, application_action.command.mit_kd,
            application_action.command.mit_torque_ff_nm, &application_action.frames);
    }
    /* POS receives intermediate positions and a positive speed limit, never MIT gains. */
    runtime_status = motor_runtime_build_position_velocity_subset(&application_motor_runtime,
        application_action.command.motor_mask, sample.position_rad,
        application_action.target_speed_rad_s, &application_action.frames);
    if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
    {
        application_action.last_pos_reference_timestamp_us = timestamp_us;
    }
    return runtime_status;
}
#endif

/**
 * @brief Starts reference or fixed-target transmission with a bounded deadline.
 * @param motor_snapshot Fresh selected feedback captured after enable.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when a terminal result was queued, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_start_one_shot_motion(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    MotorRuntimeStatus runtime_status;
    uint64_t maximum_timeout_us = 0U;
    uint64_t motion_deadline_us;
    uint8_t joint_index;

    if ((motor_snapshot == NULL) ||
        ((motor_snapshot->valid_joint_mask &
          application_action.command.motor_mask) !=
         application_action.command.motor_mask))
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_MOTION,
            PROTOCOL_COMMAND_ERROR_STALE_FEEDBACK,
            0U,
            timestamp_us);
    }
    if ((application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION) &&
        (application_action.command.mit_action == PROTOCOL_MIT_ACTION_HOLD))
    {
        return aethor_app_start_one_shot_hold(timestamp_us);
    }
    if (aethor_app_one_shot_uses_reference_trajectory() != 0U)
    {
        double maximum_duration_seconds = 0.0;

        memset(&application_action.motion_plan, 0,
               sizeof(application_action.motion_plan));
        application_action.motion_plan.start_time_us = timestamp_us;
        /* This enum selects quintic mathematics; command.control_mode still selects POS/MIT on CAN. */
        application_action.motion_plan.mode = JOINT_MOTION_MODE_MIT;
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            uint8_t joint_bit = (uint8_t)(1U << joint_index);
            float start_position_rad =
                motor_snapshot->joints[joint_index].position_rad;

            application_action.motion_plan.start_position_rad[joint_index] =
                start_position_rad;
            application_action.motion_plan.target_position_rad[joint_index] =
                ((application_action.command.motor_mask & joint_bit) != 0U)
                    ? application_action.target_position_rad[joint_index]
                    : start_position_rad;
            if ((application_action.command.motor_mask & joint_bit) != 0U)
            {
                double duration_seconds =
                    (double)AETHOR_APP_MIT_QUINTIC_MAX_VELOCITY_FACTOR *
                    fabs((double)application_action.target_position_rad[joint_index] -
                         (double)start_position_rad) /
                    ((double)application_action.target_speed_rad_s[joint_index] *
                     (double)aethor_app_local_motion_position_ratio(joint_index));

                if (!isfinite(duration_seconds) || duration_seconds >
                    (double)(UINT64_MAX - AETHOR_APP_MOTION_SETTLE_US -
                             AETHOR_APP_MOTION_TIMEOUT_MARGIN_US) / 1000000.0)
                {
                    return aethor_app_begin_one_shot_cleanup(
                        PROTOCOL_COMMAND_STAGE_MOTION,
                        PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                        (uint8_t)(joint_index + 1U), timestamp_us);
                }
                if (duration_seconds > maximum_duration_seconds)
                {
                    maximum_duration_seconds = duration_seconds;
                }
            }
        }
        if (!isfinite(maximum_duration_seconds) || maximum_duration_seconds >
            (double)(UINT64_MAX - AETHOR_APP_MOTION_SETTLE_US - AETHOR_APP_MOTION_TIMEOUT_MARGIN_US) / 1000000.0)
        {
            return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_MOTION,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 0U, timestamp_us);
        }
        application_action.motion_plan.duration_us =
            (uint64_t)ceil(maximum_duration_seconds * 1000000.0);
        if (application_action.motion_plan.duration_us <
            AETHOR_APP_MIT_MINIMUM_TRAJECTORY_US)
        {
            application_action.motion_plan.duration_us =
                AETHOR_APP_MIT_MINIMUM_TRAJECTORY_US;
        }
        if ((aethor_app_build_one_shot_reference(timestamp_us) != MOTOR_RUNTIME_STATUS_OK) ||
            !aethor_app_calculate_deadline_us(
                timestamp_us,
                application_action.motion_plan.duration_us +
                    AETHOR_APP_MOTION_SETTLE_US +
                    AETHOR_APP_MOTION_TIMEOUT_MARGIN_US,
                &motion_deadline_us))
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_MOTION,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                0U,
                timestamp_us);
        }
        application_action.state = AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT;
        application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
        application_action.frame_read_index = 0U;
        application_action.deadline_us = motion_deadline_us;
        return 0U;
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);
        uint64_t joint_timeout_us;

        if ((application_action.command.motor_mask & joint_bit) == 0U)
        {
            continue;
        }
        if (!aethor_app_calculate_one_shot_motion_timeout_us(
                motor_snapshot->joints[joint_index].position_rad /
                    aethor_app_local_motion_position_ratio(joint_index),
                application_action.target_position_rad[joint_index] /
                    aethor_app_local_motion_position_ratio(joint_index),
                application_action.target_speed_rad_s[joint_index],
                &joint_timeout_us))
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_MOTION,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                (uint8_t)(joint_index + 1U),
                timestamp_us);
        }
        if (joint_timeout_us > maximum_timeout_us)
        {
            maximum_timeout_us = joint_timeout_us;
        }
    }

    if (!aethor_app_calculate_deadline_us(timestamp_us,
                                          maximum_timeout_us,
                                          &motion_deadline_us))
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_MOTION,
            PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
            0U,
            timestamp_us);
    }

    runtime_status = motor_runtime_build_position_velocity_subset(
        &application_motor_runtime,
        application_action.command.motor_mask,
        application_action.target_position_rad,
        application_action.target_speed_rad_s,
        &application_action.frames);
    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_MOTION,
            aethor_app_map_runtime_error(runtime_status),
            0U,
            timestamp_us);
    }

    if (aethor_app_one_shot_uses_local_pos_endpoint())
    {
        application_action.last_pos_reference_timestamp_us = timestamp_us;
        application_action.local_pos_progress_timestamp_us = timestamp_us;
        application_action.local_pos_settle_start_us = 0U;
        application_action.local_pos_last_feedback_us = 0U;
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            application_action.motion_plan.start_position_rad[joint_index] =
                motor_snapshot->joints[joint_index].position_rad;
            if ((application_action.command.motor_mask & (1U << joint_index)) != 0U)
            {
                application_action.local_pos_progress_error_rad = fabsf(
                    application_action.target_position_rad[joint_index] -
                    motor_snapshot->joints[joint_index].position_rad) /
                    aethor_app_local_motion_position_ratio(joint_index);
            }
        }
    }
    memcpy(application_action.motion_plan.target_position_rad,
           application_action.target_position_rad,
           sizeof(application_action.target_position_rad));
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT;
    application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
    application_action.frame_read_index = 0U;
    application_action.deadline_us = motion_deadline_us;
    return 0U;
}
#endif

/**
 * @brief Checks 0.5 output degree for LCD moves, retaining raw degrees for USB.
 * @param motor_snapshot Current freshness-filtered motor feedback.
 * @return One when all selected motors have arrived, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_one_shot_targets_arrived(
    const MotorFeedbackSnapshot *motor_snapshot)
{
    uint8_t joint_index;
    uint8_t motor_mask = application_action.command.motor_mask;

    if ((motor_snapshot == NULL) ||
        ((motor_snapshot->valid_joint_mask & motor_mask) != motor_mask))
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);
        float position_error_rad;

        if ((motor_mask & joint_bit) == 0U)
        {
            continue;
        }
        position_error_rad =
            fabsf(motor_snapshot->joints[joint_index].position_rad -
                  application_action.target_position_rad[joint_index]) /
            aethor_app_local_motion_position_ratio(joint_index);
        if ((position_error_rad > (0.5F * AETHOR_APP_DEG_TO_RAD)) ||
            (motor_snapshot->joints[joint_index].fault_flags != 0U))
        {
            return 0U;
        }
    }
    return 1U;
}
#endif

/**
 * @brief Replaces motion targets with final-position zero-speed HOLD frames.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when a terminal result was queued, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_start_one_shot_hold(uint64_t timestamp_us)
{
    float hold_speed_rad_s[ARM_JOINT_COUNT] = {0.0F};
    MotorRuntimeStatus runtime_status =
        (application_action.command.control_mode == ARM_CONTROL_MODE_MIT)
            ? motor_runtime_build_mit_subset(
                  &application_motor_runtime,
                  application_action.command.motor_mask,
                  application_action.target_position_rad,
                  hold_speed_rad_s,
                  application_action.command.mit_kp,
                  application_action.command.mit_kd,
                  application_action.command.mit_torque_ff_nm,
                  &application_action.frames)
            : motor_runtime_build_position_velocity_subset(
                  &application_motor_runtime,
                  application_action.command.motor_mask,
                  application_action.target_position_rad,
                  hold_speed_rad_s,
                  &application_action.frames);

    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_HOLD,
            aethor_app_map_runtime_error(runtime_status),
            0U,
            timestamp_us);
    }
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT;
    application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
    application_action.frame_read_index = 0U;
    if (!aethor_app_calculate_deadline_us(
            timestamp_us,
            (application_action.command.control_mode == ARM_CONTROL_MODE_MIT)
                ? application_action.command.hold_duration_us
                : AETHOR_APP_ACTION_TIMEOUT_US,
                                          &application_action.deadline_us))
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_HOLD,
            PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
            0U,
            timestamp_us);
    }
    return 0U;
}
#endif

/**
 * @brief Starts selected POS_VEL disable after every HOLD frame was emitted.
 * @param motor_snapshot Snapshot captured before the disable batch.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when a terminal result was queued, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_start_one_shot_disable(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    MotorRuntimeStatus runtime_status = motor_runtime_build_mode_command_batch(
        &application_motor_runtime,
        aethor_app_one_shot_vendor_mode(),
        S3519_MODE_COMMAND_DISABLE,
        application_action.command.motor_mask,
        &application_action.frames);

    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_DISABLE,
            aethor_app_map_runtime_error(runtime_status),
            0U,
            timestamp_us);
    }
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT;
    application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
    application_action.frame_read_index = 0U;
    aethor_app_capture_feedback_baseline(motor_snapshot);
    if (!aethor_app_calculate_deadline_us(timestamp_us,
                                          AETHOR_APP_ACTION_TIMEOUT_US,
                                          &application_action.deadline_us))
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_DISABLE,
            PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
            0U,
            timestamp_us);
    }
    return 0U;
}
#endif

/**
 * @brief Starts a fixed-capacity self-contained POS_VEL, MIT, or mode setup.
 * @param command Parsed joint-indexed degrees and degrees-per-second request.
 * @param motor_snapshot Current freshness-filtered feedback snapshot.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when admission queued a terminal failure, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_start_one_shot_move(
    const ProtocolCommand *command,
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    uint8_t joint_index;
    MotorRuntimeStatus runtime_status;

    memset(&application_action, 0, sizeof(application_action));
    application_action.command = *command;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);

        if ((command->motor_mask & joint_bit) == 0U)
        {
            continue;
        }
        if (command->type != PROTOCOL_COMMAND_SET_MODE)
        {
            application_action.target_position_rad[joint_index] =
                (command->values_in_radians != 0U)
                    ? command->values[joint_index]
                    : command->values[joint_index] * AETHOR_APP_DEG_TO_RAD;
            if (command->relative_target != 0U)
            {
                application_action.target_position_rad[joint_index] +=
                    motor_snapshot->joints[joint_index].position_rad;
            }
            application_action.target_speed_rad_s[joint_index] =
                (command->values_in_radians != 0U)
                    ? command->speeds[joint_index]
                    : command->speeds[joint_index] * AETHOR_APP_DEG_TO_RAD;
        }
        if (!isfinite(application_action.target_position_rad[joint_index]) ||
            !isfinite(application_action.target_speed_rad_s[joint_index]))
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_VALIDATE,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                (uint8_t)(joint_index + 1U),
                timestamp_us);
        }
    }

    application_action.missing_discovery_mask =
        command->motor_mask &
        (uint8_t)~application_motor_runtime.discovery.verified_joint_mask;
    if (application_action.missing_discovery_mask == 0U)
    {
        return aethor_app_prepare_one_shot_validation(motor_snapshot,
                                                      timestamp_us);
    }
    runtime_status = motor_runtime_begin_discovery(
        &application_motor_runtime,
        application_action.missing_discovery_mask);
    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return aethor_app_begin_one_shot_cleanup(
            PROTOCOL_COMMAND_STAGE_DISCOVERY,
            aethor_app_map_runtime_error(runtime_status),
            0U,
            timestamp_us);
    }
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY;
    application_action.deadline_us = timestamp_us +
                                     AETHOR_APP_DISCOVERY_TIMEOUT_US;
    return 0U;
}
#endif

/**
 * @brief Advances discovery, validation, mode, clear, and enable for one one-shot move.
 * @param motor_snapshot Current freshness-filtered feedback snapshot.
 * @param timestamp_us Current monotonic timestamp.
 * @return One when a terminal result was queued, otherwise zero.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_advance_one_shot_setup(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    MotorRuntimeStatus runtime_status;
    uint8_t motor_mask = application_action.command.motor_mask;

    if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY)
    {
        if ((application_motor_runtime.discovery.state ==
             MOTOR_DISCOVERY_STATE_COMPLETE) &&
            ((application_motor_runtime.discovery.verified_joint_mask &
              motor_mask) == motor_mask))
        {
            return aethor_app_prepare_one_shot_validation(motor_snapshot,
                                                          timestamp_us);
        }
        if (application_motor_runtime.discovery.state ==
            MOTOR_DISCOVERY_STATE_FAILED)
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_DISCOVERY,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                0U,
                timestamp_us);
        }
    }
    else if ((application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT) &&
             (application_action.frame_read_index >=
              application_action.frames.count) &&
             (aethor_app_selected_feedback_advanced(
                  motor_snapshot,
                  motor_mask,
                  application_action.phase_feedback_baseline_us) != 0U) &&
             ((aethor_app_selected_motors_have_state(
                   motor_snapshot,
                   motor_mask,
                   S3519_DRIVER_STATE_DISABLED) != 0U) ||
              (aethor_app_selected_motors_are_fault_free(motor_snapshot,
                                                         motor_mask) == 0U)))
    {
        return aethor_app_validate_one_shot_targets(motor_snapshot,
                                                    timestamp_us);
    }
    else if (application_action.state ==
             AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH)
    {
        if (application_motor_runtime.mode_switch_state ==
            MOTOR_MODE_SWITCH_COMPLETE)
        {
            if ((application_action.command.type == PROTOCOL_COMMAND_SET_MODE) &&
                (application_action.command.bench_relative_scope != 0U))
            {
                aethor_app_update_protocol_context(timestamp_us);
                return aethor_app_complete_action(
                    PROTOCOL_COMMAND_RESULT_COMPLETED,
                    0U,
                    timestamp_us);
            }
            runtime_status = motor_runtime_build_mode_command_batch(
                &application_motor_runtime,
                aethor_app_one_shot_vendor_mode(),
                S3519_MODE_COMMAND_CLEAR_ERROR,
                motor_mask,
                &application_action.frames);
            if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
            {
                return aethor_app_begin_one_shot_cleanup(
                    PROTOCOL_COMMAND_STAGE_CLEAR,
                    aethor_app_map_runtime_error(runtime_status),
                    0U,
                    timestamp_us);
            }
            application_action.state =
                AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT;
            application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
            application_action.frame_read_index = 0U;
            aethor_app_capture_feedback_baseline(motor_snapshot);
            if (!aethor_app_calculate_deadline_us(
                    timestamp_us,
                    AETHOR_APP_ACTION_TIMEOUT_US,
                    &application_action.deadline_us))
            {
                return aethor_app_begin_one_shot_cleanup(
                    PROTOCOL_COMMAND_STAGE_CLEAR,
                    PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                    0U,
                    timestamp_us);
            }
            return 0U;
        }
        if (application_motor_runtime.mode_switch_state ==
            MOTOR_MODE_SWITCH_FAILED)
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_MODE,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                0U,
                timestamp_us);
        }
    }
    else if ((application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT) &&
             (application_action.frame_read_index >=
              application_action.frames.count) &&
             (aethor_app_selected_feedback_advanced(
                  motor_snapshot,
                  motor_mask,
                  application_action.phase_feedback_baseline_us) != 0U) &&
             (aethor_app_selected_motors_are_fault_free(motor_snapshot,
                                                        motor_mask) != 0U))
    {
        runtime_status = motor_runtime_build_mode_command_batch(
            &application_motor_runtime,
            aethor_app_one_shot_vendor_mode(),
            S3519_MODE_COMMAND_ENABLE,
            motor_mask,
            &application_action.frames);
        if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_ENABLE,
                aethor_app_map_runtime_error(runtime_status),
                0U,
                timestamp_us);
        }
        application_action.state = AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT;
        application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
        application_action.frame_read_index = 0U;
        aethor_app_capture_feedback_baseline(motor_snapshot);
        if (!aethor_app_calculate_deadline_us(
                timestamp_us,
                AETHOR_APP_ACTION_TIMEOUT_US,
                &application_action.deadline_us))
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_ENABLE,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                0U,
                timestamp_us);
        }
        return 0U;
    }
    else if ((application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT) &&
             (application_action.frame_read_index >=
              application_action.frames.count) &&
             (aethor_app_selected_feedback_advanced(
                  motor_snapshot,
                  motor_mask,
                  application_action.phase_feedback_baseline_us) != 0U) &&
             (aethor_app_selected_motors_have_state(
                  motor_snapshot,
                  motor_mask,
                  S3519_DRIVER_STATE_ENABLED) != 0U))
    {
        application_action.enabled_by_action_mask = motor_mask;
        return aethor_app_start_one_shot_motion(motor_snapshot,
                                                timestamp_us);
    }
    else if (application_action.state ==
             AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT)
    {
        uint8_t reference_trajectory = aethor_app_one_shot_uses_reference_trajectory();

        /* Check before arrival too: fresh endpoint feedback cannot hide a stalled sender. */
        if (((reference_trajectory != 0U) || aethor_app_one_shot_uses_local_pos_endpoint()) &&
            (application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) &&
            ((timestamp_us < application_action.last_pos_reference_timestamp_us) ||
             ((timestamp_us - application_action.last_pos_reference_timestamp_us) >
              AETHOR_APP_POS_REFERENCE_MAX_GAP_US)))
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_MOTION,
                PROTOCOL_COMMAND_ERROR_TIMEOUT, 0U, timestamp_us);
        }

        if (aethor_app_one_shot_uses_local_pos_endpoint())
        {
            uint8_t failed = 0U;
            if (aethor_app_local_pos_endpoint_arrived(motor_snapshot, timestamp_us, &failed))
            {
                return aethor_app_start_one_shot_hold(timestamp_us);
            }
            if (failed)
            {
                return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_MOTION,
                    PROTOCOL_COMMAND_ERROR_TIMEOUT, 0U, timestamp_us);
            }
        }
        else if ((aethor_app_one_shot_targets_arrived(motor_snapshot) != 0U) &&
            ((reference_trajectory == 0U) ||
             ((timestamp_us - application_action.motion_plan.start_time_us) >=
              application_action.motion_plan.duration_us)))
        {
            return aethor_app_start_one_shot_hold(timestamp_us);
        }
        if ((application_action.deadline_us != 0U) &&
            (timestamp_us >= application_action.deadline_us))
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_MOTION,
                PROTOCOL_COMMAND_ERROR_TIMEOUT,
                0U,
                timestamp_us);
        }
        if (application_action.frame_read_index >=
            application_action.frames.count)
        {
            if (reference_trajectory != 0U)
            {
                if (aethor_app_build_one_shot_reference(timestamp_us) != MOTOR_RUNTIME_STATUS_OK)
                {
                    return aethor_app_begin_one_shot_cleanup(
                        PROTOCOL_COMMAND_STAGE_MOTION,
                        PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                        0U,
                        timestamp_us);
                }
            }
            if (aethor_app_one_shot_uses_local_pos_endpoint())
            {
                application_action.last_pos_reference_timestamp_us = timestamp_us;
            }
            application_action.frame_read_index = 0U;
        }
    }
    else if (application_action.state ==
             AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT)
    {
        if ((application_action.command.control_mode != ARM_CONTROL_MODE_MIT) &&
            (application_action.frame_read_index >=
             application_action.frames.count))
        {
            return aethor_app_start_one_shot_disable(motor_snapshot,
                                                     timestamp_us);
        }
        if ((application_action.command.control_mode == ARM_CONTROL_MODE_MIT) &&
            (application_action.deadline_us != 0U) &&
            (timestamp_us >= application_action.deadline_us))
        {
            return aethor_app_start_one_shot_disable(motor_snapshot,
                                                     timestamp_us);
        }
        if ((application_action.command.control_mode == ARM_CONTROL_MODE_MIT) &&
            (application_action.frame_read_index >=
             application_action.frames.count))
        {
            application_action.frame_read_index = 0U;
        }
    }
    else if ((application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT) &&
             (application_action.frame_read_index >=
              application_action.frames.count))
    {
        return aethor_app_start_one_shot_cleanup_disable(motor_snapshot,
                                                         timestamp_us);
    }
    else if ((application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT) &&
             (application_action.frame_read_index >=
              application_action.frames.count) &&
             (aethor_app_selected_feedback_advanced(
                  motor_snapshot,
                  motor_mask,
                  application_action.phase_feedback_baseline_us) != 0U) &&
             (aethor_app_selected_motors_have_state(
                  motor_snapshot,
                  motor_mask,
                  S3519_DRIVER_STATE_DISABLED) != 0U))
    {
        aethor_app_update_protocol_context(timestamp_us);
        return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_COMPLETED,
                                          0U,
                                          timestamp_us);
    }
    else if ((application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT) &&
             (application_action.frame_read_index >=
              application_action.frames.count) &&
             (aethor_app_selected_feedback_advanced(
                  motor_snapshot,
                  application_action.cleanup_disable_mask,
                  application_action.phase_feedback_baseline_us) != 0U) &&
             (aethor_app_selected_motors_have_state(
                  motor_snapshot,
                  application_action.cleanup_disable_mask,
                  S3519_DRIVER_STATE_DISABLED) != 0U))
    {
        aethor_app_update_protocol_context(timestamp_us);
        return aethor_app_complete_action(
            PROTOCOL_COMMAND_RESULT_FAILED,
            (uint16_t)application_action.failure_error,
            timestamp_us);
    }

    if ((application_action.deadline_us != 0U) &&
        (timestamp_us >= application_action.deadline_us))
    {
        if (application_action.state ==
            AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY)
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_DISCOVERY,
                PROTOCOL_COMMAND_ERROR_TIMEOUT,
                0U,
                timestamp_us);
        }
        if (application_action.state ==
            AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT)
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_VALIDATE,
                PROTOCOL_COMMAND_ERROR_FEEDBACK_TIMEOUT,
                0U,
                timestamp_us);
        }
        if (application_action.state ==
            AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH)
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_MODE,
                PROTOCOL_COMMAND_ERROR_TIMEOUT,
                0U,
                timestamp_us);
        }
        if (application_action.state ==
            AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT)
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_CLEAR,
                PROTOCOL_COMMAND_ERROR_FEEDBACK_TIMEOUT,
                0U,
                timestamp_us);
        }
        if (application_action.state ==
            AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT)
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_ENABLE,
                PROTOCOL_COMMAND_ERROR_FEEDBACK_TIMEOUT,
                0U,
                timestamp_us);
        }
        if ((application_action.state ==
             AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT) &&
            (application_action.command.control_mode != ARM_CONTROL_MODE_MIT))
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_HOLD,
                PROTOCOL_COMMAND_ERROR_TIMEOUT,
                0U,
                timestamp_us);
        }
        if (application_action.state ==
            AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT)
        {
            return aethor_app_begin_one_shot_cleanup(
                PROTOCOL_COMMAND_STAGE_DISABLE,
                PROTOCOL_COMMAND_ERROR_FEEDBACK_TIMEOUT,
                0U,
                timestamp_us);
        }
        if (application_action.state ==
            AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT)
        {
            return aethor_app_complete_action(
                PROTOCOL_COMMAND_RESULT_FAILED,
                (uint16_t)application_action.failure_error,
                timestamp_us);
        }
        if (application_action.state ==
            AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT)
        {
            return aethor_app_start_one_shot_cleanup_disable(motor_snapshot,
                                                             timestamp_us);
        }
    }
    return 0U;
}
#endif

/** @brief Refreshes the protocol query context from coherent domain snapshots. */
static void aethor_app_update_protocol_context(uint64_t timestamp_us)
{
    ProtocolQueryContext query_context;
    uint8_t joint_index;

    memset(&query_context, 0, sizeof(query_context));
    aethor_app_enter_task_critical();
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
        MotorPositionVelocityLimits motion_limits;
        uint8_t joint_bit = (uint8_t)(1U << joint_index);
        uint16_t verified_fields = application_motor_runtime.discovery
                                       .results[joint_index]
                                       .verified_fields_mask;

        if (motor_runtime_get_position_velocity_limits(
                &application_motor_runtime,
                joint_index,
                &motion_limits) == MOTOR_RUNTIME_STATUS_OK)
        {
            query_context.motor_position_max_rad[joint_index] =
                motion_limits.position_max_rad;
            query_context.motor_velocity_max_rad_s[joint_index] =
                motion_limits.velocity_mapping_max_rad_s;
            query_context.motor_maximum_speed_rad_s[joint_index] =
                motion_limits.maximum_speed_rad_s;
            query_context.motor_motion_limits_valid_mask |= joint_bit;
        }
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
    aethor_app_exit_task_critical();
}

/**
 * @brief Initializes all static Phase 0 application state.
 * @param timestamp_us Initialization timestamp in microseconds.
 * @param boot_id Nonzero boot identity generated by the STM32 entry point.
 */
void aethor_app_init(uint64_t timestamp_us, uint32_t boot_id)
{
    MotorRuntimeStatus motor_status;

    application_enter_task_critical_hook = NULL;
    application_exit_task_critical_hook = NULL;
    memset(&application_debug_ui, 0, sizeof(application_debug_ui));
    application_debug_ui.epoch = 1U;
    debug_ui_mailbox_init(&application_debug_ui.mailbox);
#if !AETHOR_ADRC_BENCH
    application_last_service_timestamp_us = 0U;
#endif
    diagnostics_init(&application_diagnostics);
    arm_controller_init(&application_controller,
                        arm_config_get_production(),
                        &application_diagnostics,
                        timestamp_us);
    (void)joint_reference_init(&application_joint_reference,
                               arm_config_get_production());
    motor_status = motor_runtime_init(&application_motor_runtime,
                                      arm_config_get_production());
#if AETHOR_DEBUG_UI_ENABLE && \
    AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
    if (motor_status == MOTOR_RUNTIME_STATUS_OK)
    { motor_status = motor_runtime_begin_discovery(&application_motor_runtime,
          (uint8_t)(1U << (DEBUG_UI_INITIAL_MOTOR_ID - 1U))); }
#endif
    protocol_engine_init(&application_protocol_engine, boot_id);
    memset(&application_emergency_disable_batch,
           0,
           sizeof(application_emergency_disable_batch));
#if !AETHOR_ADRC_BENCH
    application_emergency_disable_read_index = 0U;
#endif
    memset(&application_action, 0, sizeof(application_action));
    memset(application_deferred_results,
           0,
           sizeof(application_deferred_results));
    application_deferred_result_read_sequence = 0U;
    application_deferred_result_write_sequence = 0U;
    memset(application_transport_faults,
           0,
           sizeof(application_transport_faults));
    application_transport_fault_read_sequence = 0U;
    application_transport_fault_write_sequence = 0U;
    application_initialized =
        (uint8_t)(motor_status == MOTOR_RUNTIME_STATUS_OK);
#if AETHOR_ADRC_BENCH
    if (application_initialized != 0U && adrc_app_bridge_init(&application_adrc_bridge,
        &application_motor_runtime, &application_protocol_engine,
        adrc_generated_controller_step, NULL) != ADRC_RESULT_OK)
    { application_initialized = 0U; }
#elif AETHOR_ADRC_LCD_INTEGRATED
    adrc_lcd_ownership_init(&application_adrc_lcd_ownership);
    application_integrated_can_idle = 0U;
    application_integrated_can_quiet_since_us = 0ULL;
    application_integrated_probe_submitted_us = 0ULL;
    application_integrated_probe_transmitted = 0U;
    application_integrated_probe_failed = 0U;
    application_integrated_diag_requested_us = 0ULL;
    application_integrated_diag_transmitted_us = 0ULL;
    application_integrated_handoff = ADRC_LCD_OWNERSHIP_WAITING;
    if (application_initialized != 0U && adrc_app_bridge_init_deferred(&application_adrc_bridge,
        &application_motor_runtime, &application_protocol_engine,
        adrc_generated_controller_step, NULL) != ADRC_RESULT_OK)
    { application_initialized = 0U; }
#endif
}

#if AETHOR_ADRC_LCD_INTEGRATED
/** @brief Encodes one bounded response from the exclusive ownership gate. */
static ProtocolEngineStatus aethor_app_integrated_response(ProtocolOutputBatch *output,
    ProtocolEngineStatus status, uint32_t request_id, const char *detail)
{
    int length;
    if (output == NULL || detail == NULL) { return PROTOCOL_ENGINE_STATUS_INVALID_ARGUMENT; }
    memset(output, 0, sizeof(*output));
    length = snprintf(output->messages[0].data, sizeof(output->messages[0].data),
        "%s %lu adrc %s\r\n", status == PROTOCOL_ENGINE_STATUS_OK ? "ok" : "error",
        (unsigned long)request_id, detail);
    if (length < 0 || (size_t)length >= sizeof(output->messages[0].data))
    { return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL; }
    output->messages[0].length = (uint16_t)length;
    output->messages[0].priority = PROTOCOL_OUTPUT_HIGH_PRIORITY;
    output->count = 1U;
    return status;
}

/** @brief Gives the current owner a stable read-only protocol name. */
static const char *aethor_app_integrated_owner_name(AdrcLcdOwnerState state)
{
    switch (state)
    {
        case ADRC_LCD_OWNER_LCD: return "lcd";
        case ADRC_LCD_OWNER_ACQUIRING: return "acquiring";
        case ADRC_LCD_OWNER_ADRC: return "adrc";
        case ADRC_LCD_OWNER_RELEASING: return "releasing";
        default: return "unknown";
    }
}

/** @brief Exposes the latest handoff outcome without granting control authority. */
static const char *aethor_app_integrated_handoff_name(AdrcLcdOwnershipStatus status)
{
    switch (status)
    {
        case ADRC_LCD_OWNERSHIP_TRANSFERRED: return "transferred";
        case ADRC_LCD_OWNERSHIP_RELEASED: return "released";
        case ADRC_LCD_OWNERSHIP_UNSAFE: return "unsafe";
        case ADRC_LCD_OWNERSHIP_TIMEOUT: return "timeout";
        default: return "pending";
    }
}

/** @brief Matches the full ADRC namespace prefix without accepting partial words. */
static uint8_t aethor_app_integrated_is_adrc(const TextProtocolRequest *request)
{
    const TextProtocolSpan *word = &request->command_words[0];
    return (uint8_t)(word->length == 4U &&
        (word->data[0] == 'a' || word->data[0] == 'A') &&
        (word->data[1] == 'd' || word->data[1] == 'D') &&
        (word->data[2] == 'r' || word->data[2] == 'R') &&
        (word->data[3] == 'c' || word->data[3] == 'C'));
}

/** @brief Permits only legacy queries during ADRC ownership. */
static uint8_t aethor_app_integrated_legacy_read_only(const TextProtocolRequest *request)
{
    return (uint8_t)(text_protocol_request_path_equals(request, "hello", NULL) ||
        text_protocol_request_path_equals(request, "ping", NULL) ||
        text_protocol_request_path_equals(request, "help", NULL) ||
        text_protocol_request_path_equals(request, "show", "info"));
}

/** @brief Adds the measured ownership state to the existing ADRC status response. */
static ProtocolEngineStatus aethor_app_integrated_append_owner(ProtocolOutputBatch *output,
    AdrcLcdOwnerState state)
{
    ProtocolOutputMessage *message;
    const char *name = aethor_app_integrated_owner_name(state);
    const char *handoff = aethor_app_integrated_handoff_name(application_integrated_handoff);
    int added;
    if (output == NULL || output->count == 0U) { return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL; }
    message = &output->messages[0];
    if (message->length < 2U)
    { return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL; }
    message->length -= 2U;
    added = snprintf(&message->data[message->length],
        sizeof(message->data) - message->length,
        " owner=%s handoff=%s\r\n", name, handoff);
    if (added < 0 || (size_t)added >= sizeof(message->data) - message->length)
    { memset(output, 0, sizeof(*output)); return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL; }
    message->length = (uint16_t)(message->length + added);
    return PROTOCOL_ENGINE_STATUS_OK;
}

/** @brief Routes USB requests through the exclusive owner before either command ring. */
static ProtocolEngineStatus aethor_app_integrated_process_line(const char *line, size_t length,
    uint64_t timestamp_us, ProtocolOutputBatch *output)
{
    TextProtocolRequest request;
    TextProtocolSpan motor;
    ProtocolEngineStatus status;
    AdrcLcdOwnerState owner;
    if (text_protocol_parse_request(line, length, &request) != TEXT_PROTOCOL_STATUS_OK)
    { return protocol_engine_process_text_line(&application_protocol_engine,
        line, length, timestamp_us, output); }
    aethor_app_enter_task_critical();
    owner = application_adrc_lcd_ownership.state;
    if (text_protocol_request_path_equals(&request, "adrc", "acquire"))
    {
        AdrcLcdOwnershipStatus admission;
        if (request.has_request_id == 0U || request.positional_count != 0U ||
            request.field_count != 1U || !text_protocol_find_field(&request, "motor", &motor) ||
            motor.length != 1U || motor.data[0] < '1' || motor.data[0] > '7' ||
            (uint8_t)(motor.data[0] - '0') != DEBUG_UI_INITIAL_MOTOR_ID)
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, "code=bad_argument"); }
        else
        {
            admission = (application_integrated_diag_requested_us != 0ULL &&
                timestamp_us >= application_integrated_diag_requested_us &&
                timestamp_us - application_integrated_diag_requested_us <=
                    ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US) ?
                ADRC_LCD_OWNERSHIP_BUSY : request.request_id <=
                application_adrc_bridge.bench.gateway.highest_admitted_request_id ?
                ADRC_LCD_OWNERSHIP_STALE_ID :
                adrc_lcd_ownership_submit_acquire(&application_adrc_lcd_ownership,
                    (uint8_t)(motor.data[0] - '1'), request.request_id);
            if (admission == ADRC_LCD_OWNERSHIP_OK)
            {
                application_integrated_handoff = ADRC_LCD_OWNERSHIP_WAITING;
                application_adrc_bridge.bench.gateway.highest_admitted_request_id = request.request_id;
                application_integrated_probe_submitted_us = 0ULL;
                application_integrated_probe_transmitted = 0U;
                application_integrated_probe_failed = 0U;
                application_integrated_diag_requested_us = 0ULL;
                application_integrated_diag_transmitted_us = 0ULL;
            }
            status = aethor_app_integrated_response(output,
                admission == ADRC_LCD_OWNERSHIP_OK ? PROTOCOL_ENGINE_STATUS_OK :
                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request.request_id,
                admission == ADRC_LCD_OWNERSHIP_OK ? "acquire=accepted" : "code=owner_busy_or_stale");
        }
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "release"))
    {
        AdrcLcdOwnershipStatus admission = ADRC_LCD_OWNERSHIP_BAD_ARGUMENT;
        if (request.has_request_id != 0U && request.positional_count == 0U &&
            request.field_count == 0U && request.request_id >
                application_adrc_bridge.bench.gateway.highest_admitted_request_id)
        { admission = adrc_lcd_ownership_submit_release(&application_adrc_lcd_ownership,
            request.request_id, timestamp_us); }
        if (admission == ADRC_LCD_OWNERSHIP_OK)
        {
            application_integrated_handoff = ADRC_LCD_OWNERSHIP_WAITING;
            application_adrc_bridge.bench.gateway.highest_admitted_request_id = request.request_id;
            adrc_app_bridge_request_release_disable(&application_adrc_bridge);
        }
        status = aethor_app_integrated_response(output,
            admission == ADRC_LCD_OWNERSHIP_OK ? PROTOCOL_ENGINE_STATUS_OK :
                PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request.request_id,
            admission == ADRC_LCD_OWNERSHIP_OK ? "release=accepted" : "code=owner_busy_or_stale");
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "discover"))
    {
        /* An explicit ADRC discovery reads registers through the normal CAN
         * scheduler but never runs bench init's POS_VEL mode-write phase. */
        AdrcLcdOwnershipEvidence evidence;
        MotorRuntimeStatus discovery_status = MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
        if (request.has_request_id == 0U || request.positional_count != 0U ||
            request.field_count != 1U ||
            !text_protocol_find_field(&request, "motor", &motor) ||
            motor.length != 1U ||
            motor.data[0] != (char)('0' + DEBUG_UI_INITIAL_MOTOR_ID))
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, "code=bad_argument"); }
        else
        {
            aethor_app_integrated_build_evidence(timestamp_us,
                DEBUG_UI_INITIAL_MOTOR_ID - 1U, &evidence);
            if (owner == ADRC_LCD_OWNER_LCD && evidence.lcd_idle != 0U &&
                evidence.can_idle != 0U &&
                application_action.state == AETHOR_APP_ACTION_IDLE)
            { discovery_status = motor_runtime_begin_discovery(&application_motor_runtime,
                (uint8_t)(1U << (DEBUG_UI_INITIAL_MOTOR_ID - 1U))); }
            status = aethor_app_integrated_response(output,
                discovery_status == MOTOR_RUNTIME_STATUS_OK ? PROTOCOL_ENGINE_STATUS_OK :
                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request.request_id,
                discovery_status == MOTOR_RUNTIME_STATUS_OK ? "discover=accepted" :
                    "code=owner_or_bus_busy");
        }
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "probe"))
    {
        if (request.positional_count == 0U && request.field_count == 0U)
        {
            MotorFeedbackSnapshot snapshot;
            const MotorJointFeedback *feedback;
            const char *probe_state = "idle";
            uint64_t feedback_age_us = UINT64_MAX;
            uint8_t after_transmit;
            char detail[240];
            int written;
            memset(&snapshot, 0, sizeof(snapshot));
            (void)motor_runtime_get_snapshot(&application_motor_runtime,
                timestamp_us, UINT64_MAX, &snapshot);
            feedback = &snapshot.joints[DEBUG_UI_INITIAL_MOTOR_ID - 1U];
            after_transmit = (uint8_t)(application_integrated_diag_transmitted_us != 0ULL &&
                (snapshot.valid_joint_mask &
                    (uint8_t)(1U << (DEBUG_UI_INITIAL_MOTOR_ID - 1U))) != 0U &&
                feedback->timestamp_us > application_integrated_diag_transmitted_us);
            if (feedback->timestamp_us != 0ULL && timestamp_us >= feedback->timestamp_us)
            { feedback_age_us = timestamp_us - feedback->timestamp_us; }
            if (application_integrated_diag_requested_us != 0ULL)
            {
                if (application_integrated_probe_failed != 0U) { probe_state = "failed"; }
                else if (timestamp_us < application_integrated_diag_requested_us ||
                    timestamp_us - application_integrated_diag_requested_us >
                        ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US)
                { probe_state = "timeout"; }
                else if (after_transmit != 0U) { probe_state = "feedback"; }
                else if (application_integrated_diag_transmitted_us != 0ULL)
                { probe_state = "transmitted"; }
                else if (application_integrated_probe_submitted_us != 0ULL)
                { probe_state = "submitted"; }
                else { probe_state = "queued"; }
            }
            written = snprintf(detail, sizeof(detail),
                "probe motor=%u state=%s tx=%u sample_after_tx=%u feedback_state=%u fault=%lu age_us=%llu owner=%s",
                (unsigned int)DEBUG_UI_INITIAL_MOTOR_ID, probe_state,
                (unsigned int)application_integrated_probe_transmitted,
                (unsigned int)after_transmit, (unsigned int)feedback->driver_state,
                (unsigned long)feedback->fault_flags,
                (unsigned long long)feedback_age_us,
                aethor_app_integrated_owner_name(owner));
            status = written < 0 || (size_t)written >= sizeof(detail) ?
                PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL :
                aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK,
                    request.request_id, detail);
        }
        else if (request.has_request_id == 0U || request.positional_count != 0U ||
            request.field_count != 1U ||
            !text_protocol_find_field(&request, "motor", &motor) ||
            motor.length != 1U ||
            motor.data[0] != (char)('0' + DEBUG_UI_INITIAL_MOTOR_ID))
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, "code=bad_argument"); }
        else
        {
            AdrcLcdOwnershipEvidence evidence;
            uint8_t previous_probe_active = (uint8_t)(
                application_integrated_diag_requested_us != 0ULL &&
                timestamp_us >= application_integrated_diag_requested_us &&
                timestamp_us - application_integrated_diag_requested_us <=
                    ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US);
            aethor_app_integrated_build_evidence(timestamp_us,
                DEBUG_UI_INITIAL_MOTOR_ID - 1U, &evidence);
            if (owner != ADRC_LCD_OWNER_LCD || previous_probe_active != 0U ||
                evidence.lcd_idle == 0U || evidence.can_idle == 0U ||
                evidence.mit_discovered == 0U ||
                application_action.state != AETHOR_APP_ACTION_IDLE)
            { status = aethor_app_integrated_response(output,
                PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request.request_id,
                "code=owner_bus_or_mode_busy"); }
            else
            {
                application_integrated_diag_requested_us = timestamp_us;
                application_integrated_diag_transmitted_us = 0ULL;
                application_integrated_probe_submitted_us = 0ULL;
                application_integrated_probe_transmitted = 0U;
                application_integrated_probe_failed = 0U;
                status = aethor_app_integrated_response(output,
                    PROTOCOL_ENGINE_STATUS_OK, request.request_id, "probe=accepted");
            }
        }
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "gate"))
    {
        if (request.positional_count != 0U || request.field_count != 1U ||
            !text_protocol_find_field(&request, "motor", &motor) ||
            motor.length != 1U || motor.data[0] != (char)('0' + DEBUG_UI_INITIAL_MOTOR_ID))
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, "code=bad_argument"); }
        else
        {
            AdrcLcdOwnershipEvidence evidence;
            const MotorDiscoveryResult *discovery =
                &application_motor_runtime.discovery.results[DEBUG_UI_INITIAL_MOTOR_ID - 1U];
            char detail[320];
            int written;
            aethor_app_integrated_build_evidence(timestamp_us,
                DEBUG_UI_INITIAL_MOTOR_ID - 1U, &evidence);
            written = snprintf(detail, sizeof(detail),
                "gate motor=%u lcd_idle=%u can_idle=%u mit_ready=%u mode=%lu fields=%04x verified=%02x fb_fresh=%u disabled=%u no_fault=%u stationary=%u authority=%u action=%u ui_busy=%u ui_results=%u proto_results=%u discovery_active=%u",
                (unsigned int)DEBUG_UI_INITIAL_MOTOR_ID,
                (unsigned int)evidence.lcd_idle, (unsigned int)evidence.can_idle,
                (unsigned int)evidence.mit_discovered,
                (unsigned long)discovery->observed_control_mode,
                (unsigned int)discovery->verified_fields_mask,
                (unsigned int)application_motor_runtime.discovery.verified_joint_mask,
                (unsigned int)evidence.feedback_fresh, (unsigned int)evidence.disabled,
                (unsigned int)evidence.no_fault, (unsigned int)evidence.stationary,
                (unsigned int)application_debug_ui.authority,
                (unsigned int)application_action.state,
                (unsigned int)debug_ui_mailbox_busy(&application_debug_ui.mailbox, true),
                (unsigned int)debug_ui_mailbox_result_count(&application_debug_ui.mailbox),
                (unsigned int)(uint8_t)(application_protocol_engine.result_write_sequence -
                    application_protocol_engine.result_read_sequence),
                (unsigned int)application_motor_runtime.discovery_active);
            status = written < 0 || (size_t)written >= sizeof(detail) ?
                PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL :
                aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK,
                    request.request_id, detail);
        }
    }
    else if (aethor_app_integrated_is_adrc(&request))
    {
        uint8_t read_only = (uint8_t)(text_protocol_request_path_equals(&request, "adrc", NULL) ||
            text_protocol_request_path_equals(&request, "adrc", "status") ||
            text_protocol_request_path_equals(&request, "adrc", "hardware") ||
            text_protocol_request_path_equals(&request, "adrc", "limits") ||
            text_protocol_request_path_equals(&request, "adrc", "feedback"));
        if (!read_only && owner != ADRC_LCD_OWNER_ADRC)
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, "code=owner_required"); }
        else if (text_protocol_request_path_equals(&request, "adrc", "prepare") &&
            text_protocol_find_field(&request, "motor", &motor) &&
            (motor.length != 1U ||
             (uint8_t)(motor.data[0] - '1') != application_adrc_lcd_ownership.axis_index))
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, "code=owner_axis"); }
        else
        {
            status = adrc_app_bridge_process_line(&application_adrc_bridge,
                line, length, timestamp_us, output);
            if (status == PROTOCOL_ENGINE_STATUS_OK &&
                (text_protocol_request_path_equals(&request, "adrc", NULL) ||
                 text_protocol_request_path_equals(&request, "adrc", "status")))
            { status = aethor_app_integrated_append_owner(output, owner); }
        }
    }
    else if (owner == ADRC_LCD_OWNER_ACQUIRING &&
        (text_protocol_request_path_equals(&request, "stop", NULL) ||
         text_protocol_request_path_equals(&request, "disable", NULL)))
    {
        application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_LCD;
        status = protocol_engine_process_text_line(&application_protocol_engine,
            line, length, timestamp_us, output);
    }
    else if (owner != ADRC_LCD_OWNER_LCD &&
        (text_protocol_request_path_equals(&request, "stop", NULL) ||
         text_protocol_request_path_equals(&request, "disable", NULL)))
    {
        adrc_app_bridge_request_stop(&application_adrc_bridge);
        if (request.request_id > application_adrc_bridge.bench.gateway.highest_admitted_request_id)
        { application_adrc_bridge.bench.gateway.highest_admitted_request_id = request.request_id; }
        status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK,
            request.request_id, "stop=latched");
    }
    else if (owner != ADRC_LCD_OWNER_LCD &&
             !aethor_app_integrated_legacy_read_only(&request))
    { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        request.request_id, "code=owner_adrc"); }
    else
    { status = protocol_engine_process_text_line(&application_protocol_engine,
        line, length, timestamp_us, output); }
    aethor_app_exit_task_critical();
    return status;
}
#endif

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
#if !AETHOR_ADRC_BENCH
#if AETHOR_ADRC_LCD_INTEGRATED
    return aethor_app_integrated_process_line(line, length, timestamp_us, output_batch);
#else
    return protocol_engine_process_text_line(&application_protocol_engine,
                                             line,
                                             length,
                                             timestamp_us,
                                             output_batch);
#endif
#else
    {
        ProtocolEngineStatus status;
        aethor_app_enter_task_critical();
        status = adrc_app_bridge_process_line(&application_adrc_bridge, line, length, timestamp_us, output_batch);
        aethor_app_exit_task_critical();
        return status;
    }
#endif
}

/**
 * @brief Pops one pending fail-safe motor disable frame.
 */
uint8_t aethor_app_pop_emergency_can_frame(CanFrame *frame)
{
#if AETHOR_ADRC_BENCH
    (void)frame;
    return 0U;
#else
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD) { return 0U; }
#endif
    if ((application_initialized == 0U) || (frame == NULL))
    {
        return 0U;
    }
    if (application_emergency_disable_read_index >= application_emergency_disable_batch.count)
    {
        if (application_debug_ui.fallback_frame_read_index >= application_debug_ui.fallback_frames.count)
        { return 0U; }
        *frame = application_debug_ui.fallback_frames.frames[application_debug_ui.fallback_frame_read_index];
        ++application_debug_ui.fallback_frame_read_index;
        return 1U;
    }
    *frame = application_emergency_disable_batch.frames[
        application_emergency_disable_read_index];
    ++application_emergency_disable_read_index;
    return 1U;
#endif
}

/**
 * @brief Pops one ordered atomic J1-J7 motion control group.
 */
uint8_t aethor_app_pop_control_group(
    CanFrame frames[ARM_JOINT_COUNT])
{
#if AETHOR_ADRC_BENCH
    (void)frames;
    return 0U;
#else
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD) { return 0U; }
#endif
    if ((application_initialized == 0U) || (frames == NULL) ||
        (application_debug_ui.fallback_stop_mask != 0U) ||
        (application_action.control_group_ready == 0U))
    {
        return 0U;
    }
    memcpy(frames,
           application_action.control_group,
           sizeof(application_action.control_group));
    application_action.control_group_ready = 0U;
    return 1U;
#endif
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

#if AETHOR_ADRC_BENCH
    if (application_initialized == 0U) { return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED; }
    if (frame == NULL || priority == NULL) { return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT; }
    aethor_app_enter_task_critical();
    runtime_status = adrc_app_bridge_next_discovery(&application_adrc_bridge, timestamp_us, frame);
    if (runtime_status == MOTOR_RUNTIME_STATUS_FRAME_READY) { *priority = CAN_TX_PRIORITY_PARAMETER; }
    aethor_app_exit_task_critical();
    return runtime_status;
#else
    if (application_initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    if ((frame == NULL) || (priority == NULL))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING)
    { return MOTOR_RUNTIME_STATUS_WAITING; }
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
    {
        runtime_status = adrc_app_bridge_next_discovery(&application_adrc_bridge,
            timestamp_us, frame);
        if (runtime_status == MOTOR_RUNTIME_STATUS_FRAME_READY)
        { *priority = CAN_TX_PRIORITY_PARAMETER; }
        return runtime_status;
    }
#endif
#if AETHOR_DEBUG_UI_ENABLE && !AETHOR_DEBUG_UI_ALLOW_MOTION && \
    AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
    if (aethor_app_debug_ui_executor_busy() || application_motor_runtime.discovery_active ||
        debug_ui_mailbox_busy(&application_debug_ui.mailbox, true))
    { (void)motor_runtime_next_position_read(&application_motor_runtime,
          DEBUG_UI_INITIAL_MOTOR_ID - 1U, timestamp_us, 0U, frame); }
#endif
    if (application_debug_ui.fallback_stop_mask != 0U)
    { return MOTOR_RUNTIME_STATUS_WAITING; }

    /* HAL tick timestamps have millisecond granularity. Defer this local preflight
     * until the clock advances so a fast same-tick DISABLE reply is strictly newer
     * than the request baseline. Keep the original fresh-feedback comparison. */
    if ((application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT) &&
        (application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
        ((application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
         (application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION) ||
         (application_action.command.type == PROTOCOL_COMMAND_SET_MODE)))
    {
        uint8_t joint_index;
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            if (((application_action.command.motor_mask & (1U << joint_index)) != 0U) &&
                (timestamp_us <= application_action.phase_feedback_baseline_us[joint_index]))
            {
                return MOTOR_RUNTIME_STATUS_WAITING;
            }
        }
    }

    if (application_action.frame_read_index < application_action.frames.count)
    {
        *frame = application_action.frames.frames[
            application_action.frame_read_index];
        ++application_action.frame_read_index;
        if ((application_action.frame_read_index ==
             application_action.frames.count) &&
            ((application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT) ||
              (application_action.state ==
               AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT) ||
              (application_action.state ==
               AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT) ||
              ((application_action.state ==
                AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT) &&
               (application_action.command.control_mode !=
                ARM_CONTROL_MODE_MIT)) ||
              (application_action.state ==
               AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT) ||
              (application_action.state ==
               AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT) ||
              (application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT)))
        {
            if (!aethor_app_calculate_deadline_us(
                    timestamp_us,
                    AETHOR_APP_ACTION_TIMEOUT_US,
                    &application_action.deadline_us))
            {
                application_action.deadline_us = timestamp_us;
            }
        }
        *priority = application_action.priority;
        return MOTOR_RUNTIME_STATUS_FRAME_READY;
    }
    if ((application_action.state == AETHOR_APP_ACTION_MODE_SWITCH) ||
        (application_action.state == AETHOR_APP_ACTION_BENCH_MODE_SWITCH) ||
        (application_action.state ==
         AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH))
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
#if AETHOR_DEBUG_UI_ENABLE && !AETHOR_DEBUG_UI_ALLOW_MOTION && \
    AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
    else if (!aethor_app_debug_ui_executor_busy() && !application_motor_runtime.discovery_active &&
             !debug_ui_mailbox_busy(&application_debug_ui.mailbox, true) &&
             application_emergency_disable_read_index >= application_emergency_disable_batch.count &&
             (application_motor_runtime.discovery.results[DEBUG_UI_INITIAL_MOTOR_ID - 1U].verified_fields_mask &
              MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK) == MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK)
    {
        if (motor_runtime_next_position_read(&application_motor_runtime,
            DEBUG_UI_INITIAL_MOTOR_ID - 1U, timestamp_us, 1U, frame) == MOTOR_RUNTIME_STATUS_FRAME_READY)
        { *priority = CAN_TX_PRIORITY_PARAMETER; return MOTOR_RUNTIME_STATUS_FRAME_READY; }
    }
#endif
    return runtime_status;
#endif
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
#if AETHOR_ADRC_BENCH
    {
        MotorRuntimeStatus status;
        aethor_app_enter_task_critical();
        status = motor_runtime_accept_frame(&application_motor_runtime, frame, timestamp_us);
        aethor_app_exit_task_critical();
        return status;
    }
#else
    return motor_runtime_accept_frame(&application_motor_runtime,
                                      frame,
                                      timestamp_us);
#endif
}

/**
 * @brief Copies the current coherent seven-motor feedback snapshot.
 */
bool aethor_app_get_motor_snapshot(uint64_t timestamp_us,
                                  MotorFeedbackSnapshot *snapshot)
{
    MotorRuntimeStatus snapshot_status;

    if (application_initialized == 0U)
    {
        return false;
    }
    aethor_app_enter_task_critical();
    snapshot_status = motor_runtime_get_snapshot(
        &application_motor_runtime,
        timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US,
        snapshot);
    aethor_app_exit_task_critical();
    return (snapshot_status == MOTOR_RUNTIME_STATUS_OK);
}

/** @brief Starts one motor-backed lifecycle command after protocol admission. */
#if !AETHOR_ADRC_BENCH
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
    aethor_app_debug_ui_result_metadata(&result, command, timestamp_us);
    result.type = command->type;
    result.code = PROTOCOL_COMMAND_RESULT_FAILED;
    result.accepted_at_us = command->accepted_at_us;
    result.completed_at_us = timestamp_us;
    result.motor_mask = command->motor_mask;
    result.bench_relative_scope = command->bench_relative_scope;
    (void)arm_controller_get_snapshot(&application_controller, &arm_snapshot);
    vendor_mode = aethor_app_vendor_mode(arm_snapshot.control_mode);

    if ((command->origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
        (command->type != PROTOCOL_COMMAND_STOP))
    {
        result.local_reason = aethor_app_debug_ui_validate_start(command, motor_snapshot, timestamp_us, false);
        if (result.local_reason != DEBUG_UI_REASON_NONE)
        {
            result.stage = PROTOCOL_COMMAND_STAGE_VALIDATE;
            result.error = PROTOCOL_COMMAND_ERROR_NOT_READY;
            return aethor_app_submit_or_defer_result(&result);
        }
    }
    if (((command->cleanup_after_stop != 0U) && (command->type == PROTOCOL_COMMAND_STOP)) ||
        ((command->origin == DEBUG_UI_ORIGIN_LOCAL_UI) && (command->type == PROTOCOL_COMMAND_DISABLE)))
    {
        aethor_app_capture_feedback_baseline(motor_snapshot);
    }

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
        return aethor_app_submit_or_defer_result(&result);
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
    else if ((command->type == PROTOCOL_COMMAND_SET_MODE) &&
             (command->bench_relative_scope != 0U))
    {
        return aethor_app_start_one_shot_move(command,
                                              motor_snapshot,
                                              timestamp_us);
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
            if (motor_snapshot->joints[joint_index].driver_state !=
                S3519_DRIVER_STATE_ENABLED)
            {
                selected_feedback_valid = 0U;
                break;
            }
            hold_position_rad[joint_index] =
                motor_snapshot->joints[joint_index].position_rad;
        }
        if (selected_feedback_valid != 0U)
        {
            runtime_status =
                (command->control_mode == ARM_CONTROL_MODE_MIT)
                    ? motor_runtime_build_mit_subset(
                          &application_motor_runtime,
                          command->motor_mask,
                          hold_position_rad,
                          zero_velocity_rad_s,
                          command->mit_kp,
                          command->mit_kd,
                          command->mit_torque_ff_nm,
                          &application_action.frames)
                    : motor_runtime_build_position_velocity_subset(
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
        runtime_status = motor_runtime_build_emergency_disable_subset(
            &application_motor_runtime,
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
            (command->origin == DEBUG_UI_ORIGIN_LOCAL_UI)
                ? aethor_app_vendor_mode(command->control_mode)
                : S3519_CONTROL_MODE_POSITION_VELOCITY,
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
            (command->origin == DEBUG_UI_ORIGIN_LOCAL_UI)
                ? aethor_app_vendor_mode(command->control_mode)
                : S3519_CONTROL_MODE_POSITION_VELOCITY,
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
    else if ((command->type ==
              PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
             (command->type == PROTOCOL_COMMAND_MIT_ACTION))
    {
        return aethor_app_start_one_shot_move(command,
                                              motor_snapshot,
                                              timestamp_us);
    }
    else if (command->type == PROTOCOL_COMMAND_MOVE_RELATIVE)
    {
        float motor_position_rad[ARM_JOINT_COUNT];
        float motor_velocity_rad_s[ARM_JOINT_COUNT];
        uint8_t failed_joint_index = 0U;
        uint8_t joint_index;
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
        runtime_status = motor_runtime_validate_position_velocity_move_subset(
            &application_motor_runtime,
            motor_snapshot,
            command->motor_mask,
            motor_position_rad,
            motor_velocity_rad_s,
            &failed_joint_index);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
        {
            for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
            {
                uint8_t joint_bit = (uint8_t)(1U << joint_index);

                if (((command->motor_mask & joint_bit) != 0U) &&
                    (motor_snapshot->joints[joint_index].driver_state !=
                     S3519_DRIVER_STATE_ENABLED))
                {
                    failed_joint_index = joint_index;
                    runtime_status = MOTOR_RUNTIME_STATUS_ACTION_FAILED;
                    break;
                }
            }
        }
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK)
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
        result.stage = PROTOCOL_COMMAND_STAGE_VALIDATE;
        result.error = aethor_app_map_runtime_error(runtime_status);
        result.failed_motor_number = (uint8_t)(failed_joint_index + 1U);
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
    return aethor_app_submit_or_defer_result(&result);
}
#endif

/** @brief Advances one active motor-backed lifecycle action. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_service_active_action(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    if (application_action.state == AETHOR_APP_ACTION_IDLE)
    {
        return 0U;
    }
    if ((application_action.state >=
         AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY) &&
        (application_action.state <=
         AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT))
    {
        return aethor_app_advance_one_shot_setup(motor_snapshot,
                                                 timestamp_us);
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
        if ((application_action.command.type == PROTOCOL_COMMAND_STOP) &&
            (application_action.command.cleanup_after_stop != 0U))
        {
            MotorRuntimeStatus disable_status = motor_runtime_build_emergency_disable_subset(
                &application_motor_runtime, application_action.command.motor_mask,
                &application_action.frames);
            if (disable_status != MOTOR_RUNTIME_STATUS_OK)
            {
                return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                    (uint16_t)disable_status, timestamp_us);
            }
            application_action.state = AETHOR_APP_ACTION_DISABLE_WAIT;
            application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
            application_action.frame_read_index = 0U;
            application_action.deadline_us = timestamp_us + AETHOR_APP_ACTION_TIMEOUT_US;
            aethor_app_capture_feedback_baseline(motor_snapshot);
            return 0U;
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
        application_action.frame_read_index = 0U;
    }
    else if ((application_action.frame_read_index >=
              application_action.frames.count) &&
             (application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT) &&
             (((application_action.command.cleanup_after_stop == 0U) &&
               (application_action.command.origin != DEBUG_UI_ORIGIN_LOCAL_UI)) ||
              (aethor_app_selected_feedback_advanced(motor_snapshot,
                   application_action.command.motor_mask,
                   application_action.phase_feedback_baseline_us) != 0U)) &&
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
#if !AETHOR_ADRC_BENCH
            application_emergency_disable_read_index = 0U;
#endif
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
#endif

#if AETHOR_ADRC_LCD_INTEGRATED
/** @brief Captures only fresh physical feedback and drained legacy sources for handoff. */
static void aethor_app_integrated_build_evidence(uint64_t timestamp_us, uint8_t axis,
    AdrcLcdOwnershipEvidence *evidence)
{
    MotorFeedbackSnapshot snapshot;
    const MotorDiscoveryResult *discovery;
    const MotorJointFeedback *feedback;
    const JointConfig *joint;
    uint8_t bit;
    memset(evidence, 0, sizeof(*evidence));
    memset(&snapshot, 0, sizeof(snapshot));
    evidence->now_us = timestamp_us;
    evidence->can_idle = application_integrated_can_idle;
    evidence->probe_submitted_us = application_integrated_probe_submitted_us;
    evidence->probe_transmitted = application_integrated_probe_transmitted;
    evidence->probe_failed = application_integrated_probe_failed;
    evidence->lcd_idle = (uint8_t)(!aethor_app_debug_ui_executor_busy() &&
        !debug_ui_mailbox_busy(&application_debug_ui.mailbox, true) &&
        application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE &&
        application_debug_ui.unconfirmed_disable_mask == 0U &&
        debug_ui_mailbox_result_count(&application_debug_ui.mailbox) == 0U &&
        application_protocol_engine.result_read_sequence ==
            application_protocol_engine.result_write_sequence &&
        application_debug_ui.fallback_frame_read_index >= application_debug_ui.fallback_frames.count &&
        application_emergency_disable_read_index >= application_emergency_disable_batch.count &&
        application_action.control_group_ready == 0U &&
        application_action.frame_read_index >= application_action.frames.count &&
        application_motor_runtime.discovery_active == 0U);
    if (axis >= ARM_JOINT_COUNT) { return; }
    bit = (uint8_t)(1U << axis);
    joint = &arm_config_get_production()->joints[axis];
    discovery = &application_motor_runtime.discovery.results[axis];
    evidence->mit_discovered = (uint8_t)(
        (application_motor_runtime.discovery.verified_joint_mask & bit) != 0U &&
        (discovery->verified_fields_mask & MOTOR_DISCOVERY_ALL_FIELDS_MASK) ==
            MOTOR_DISCOVERY_ALL_FIELDS_MASK &&
        discovery->observed_esc_id == joint->esc_id &&
        discovery->observed_master_id == joint->master_id &&
        discovery->observed_control_mode == 1U);
    (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        ADRC_LCD_OWNER_FEEDBACK_MAX_AGE_US, &snapshot);
    if ((snapshot.valid_joint_mask & bit) == 0U) { return; }
    feedback = &snapshot.joints[axis];
    evidence->feedback_us = feedback->timestamp_us;
    evidence->feedback_fresh = 1U;
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING)
    { evidence->can_idle = (uint8_t)(application_integrated_can_idle != 0U &&
        feedback->timestamp_us > application_integrated_can_quiet_since_us); }
    evidence->disabled = (uint8_t)(feedback->driver_state == S3519_DRIVER_STATE_DISABLED);
    evidence->no_fault = (uint8_t)(feedback->fault_flags == 0U);
    evidence->stationary = (uint8_t)(isfinite(feedback->velocity_rad_s) &&
        fabsf(feedback->velocity_rad_s) <= AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S);
}

/** @brief Advances the exclusive owner and keeps failed transfers fail closed. */
static uint8_t aethor_app_integrated_service(uint64_t timestamp_us)
{
    AdrcLcdOwnershipEvidence evidence;
    AdrcLcdOwnershipStatus transition;
    uint8_t result_generated = 0U;
    aethor_app_enter_task_critical();
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ADRC ||
        application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING)
    { result_generated = adrc_app_bridge_service(&application_adrc_bridge, timestamp_us); }
    aethor_app_integrated_build_evidence(timestamp_us,
        application_adrc_lcd_ownership.axis_index, &evidence);
    transition = adrc_lcd_ownership_service(&application_adrc_lcd_ownership, &evidence);
    if (transition != ADRC_LCD_OWNERSHIP_WAITING)
    { application_integrated_handoff = transition; }
    if (transition == ADRC_LCD_OWNERSHIP_TRANSFERRED &&
        adrc_app_bridge_start_selected_discovery(&application_adrc_bridge,
            application_adrc_lcd_ownership.axis_index) != MOTOR_RUNTIME_STATUS_OK)
    {
        application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_LCD;
        application_integrated_handoff = ADRC_LCD_OWNERSHIP_UNSAFE;
    }
    if (transition == ADRC_LCD_OWNERSHIP_RELEASED)
    { adrc_app_bridge_complete_release(&application_adrc_bridge); }
    aethor_app_exit_task_critical();
    return result_generated;
}

/** @brief Requires a full 4 ms quiet interval after the last legacy CAN submission. */
void aethor_app_integrated_set_can_idle(uint8_t idle, uint64_t timestamp_us)
{
    aethor_app_enter_task_critical();
    if (idle == 0U || timestamp_us == 0ULL ||
        timestamp_us < application_integrated_can_quiet_since_us)
    {
        application_integrated_can_quiet_since_us = 0ULL;
        application_integrated_can_idle = 0U;
    }
    else
    {
        if (application_integrated_can_quiet_since_us == 0ULL)
        { application_integrated_can_quiet_since_us = timestamp_us; }
        application_integrated_can_idle = (uint8_t)(
            timestamp_us - application_integrated_can_quiet_since_us >= 4000ULL);
    }
    aethor_app_exit_task_critical();
}

/** @brief Invalidates an earlier idle sample when control code submits legacy CAN. */
void aethor_app_integrated_note_legacy_can_activity(void)
{
    aethor_app_enter_task_critical();
    application_integrated_can_quiet_since_us = 0ULL;
    application_integrated_can_idle = 0U;
    if (application_integrated_diag_requested_us != 0ULL)
    { application_integrated_probe_failed = 1U; }
    aethor_app_exit_task_critical();
}

/** @brief Issues at most one read-only disabled-feedback query after legacy CAN is quiet. */
uint8_t aethor_app_integrated_pop_probe_frame(CanFrame *frame, uint64_t timestamp_us)
{
    AdrcLcdOwnershipEvidence evidence;
    uint8_t available = 0U;
    uint8_t axis;
    if (!application_initialized || frame == NULL || timestamp_us == 0ULL) { return 0U; }
    aethor_app_enter_task_critical();
    if ((application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING ||
        (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD &&
         application_integrated_diag_requested_us != 0ULL &&
         timestamp_us >= application_integrated_diag_requested_us &&
         timestamp_us - application_integrated_diag_requested_us <=
            ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US)) &&
        application_integrated_probe_submitted_us == 0ULL &&
        application_integrated_probe_failed == 0U)
    {
        axis = application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING ?
            application_adrc_lcd_ownership.axis_index :
            (uint8_t)(DEBUG_UI_INITIAL_MOTOR_ID - 1U);
        aethor_app_integrated_build_evidence(timestamp_us,
            axis, &evidence);
        if (evidence.lcd_idle != 0U && evidence.can_idle != 0U &&
            evidence.mit_discovered != 0U)
        {
            const JointConfig *joint = &arm_config_get_production()->joints[axis];
            if (s3519_pack_feedback_query((uint8_t)joint->esc_id, frame) ==
                S3519_CODEC_STATUS_OK)
            {
                application_integrated_probe_submitted_us = timestamp_us;
                available = 1U;
            }
            else { application_integrated_probe_failed = 1U; }
        }
    }
    aethor_app_exit_task_critical();
    return available;
}

/** @brief Admits only a matching successful dedicated-buffer transmit as probe evidence. */
void aethor_app_integrated_report_probe_transmit(uint8_t succeeded, uint64_t timestamp_us)
{
    if (!application_initialized) { return; }
    aethor_app_enter_task_critical();
    if ((application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING ||
        (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD &&
         application_integrated_diag_requested_us != 0ULL)) &&
        application_integrated_probe_submitted_us != 0ULL)
    {
        if (succeeded != 0U && timestamp_us >= application_integrated_probe_submitted_us)
        {
            application_integrated_probe_transmitted = 1U;
            if (application_integrated_diag_requested_us != 0ULL)
            { application_integrated_diag_transmitted_us = timestamp_us; }
        }
        else { application_integrated_probe_failed = 1U; }
    }
    aethor_app_exit_task_critical();
}
#endif

/**
 * @brief Executes one non-blocking Phase 0 application service cycle.
 * @param timestamp_us Current monotonic time in microseconds.
 */
uint8_t aethor_app_service(uint64_t timestamp_us)
{
    uint8_t result_generated = 0U;
#if !AETHOR_ADRC_BENCH
    uint8_t deferred_result_flushed = 0U;
#endif

#if AETHOR_ADRC_BENCH
    if (application_initialized == 0U) { return 0U; }
    aethor_app_enter_task_critical();
    result_generated = adrc_app_bridge_service(&application_adrc_bridge, timestamp_us);
    aethor_app_exit_task_critical();
    return result_generated;
#else
    if (application_initialized != 0U)
    {
#if AETHOR_ADRC_LCD_INTEGRATED
        if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
        { return aethor_app_integrated_service(timestamp_us); }
#endif
        MotorFeedbackSnapshot motor_snapshot;
        ProtocolCommand command;
        MotorRuntimeStatus snapshot_status;
        ArmSnapshot arm_snapshot;
        uint8_t one_shot_motor_failure_detected;
        uint32_t transport_fault_detail;
        uint64_t transport_fault_timestamp_us;

        deferred_result_flushed = aethor_app_flush_deferred_results();
        result_generated |= aethor_app_debug_ui_service_health(timestamp_us);
        if (aethor_app_take_transport_fault(&transport_fault_detail,
                                            &transport_fault_timestamp_us) != 0U)
        {
            result_generated = aethor_app_apply_transport_fault(
                transport_fault_detail,
                transport_fault_timestamp_us);
            return (uint8_t)(result_generated | deferred_result_flushed);
        }
        if (aethor_app_debug_ui_service_stop_intent(timestamp_us, &result_generated) != 0U)
        { return (uint8_t)(result_generated | deferred_result_flushed); }

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
        if ((arm_snapshot.enabled != 0U) || (arm_snapshot.moving != 0U) ||
            (aethor_app_active_action_owns_link_lifecycle() != 0U))
        {
            uint32_t driver_fault_detail =
                aethor_app_first_driver_fault_detail(&motor_snapshot);
            ArmFault runtime_fault = ARM_FAULT_NONE;
            uint32_t runtime_fault_detail = 0U;

            if (((arm_snapshot.enabled != 0U) ||
                 (arm_snapshot.moving != 0U)) &&
                (driver_fault_detail != 0U))
            {
                runtime_fault = ARM_FAULT_DRIVER;
                runtime_fault_detail = driver_fault_detail;
            }
            else if (((arm_snapshot.enabled != 0U) ||
                      (arm_snapshot.moving != 0U)) &&
                     ((snapshot_status != MOTOR_RUNTIME_STATUS_OK) ||
                      (motor_snapshot.valid_joint_mask !=
                       AETHOR_APP_ALL_JOINTS_MASK)))
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
                aethor_app_widen_pending_stop_before_global_cancel();
                (void)motor_runtime_abort_active_parameter_sequences(
                    &application_motor_runtime,
                    timestamp_us);
                if (application_action.state != AETHOR_APP_ACTION_IDLE)
                {
                    if ((runtime_fault == ARM_FAULT_CONTROL_DEADLINE) &&
                        ((application_action.command.type ==
                          PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
                         (application_action.command.type ==
                          PROTOCOL_COMMAND_MIT_ACTION)))
                    {
                        aethor_app_record_one_shot_failure_once(
                            aethor_app_current_one_shot_stage(),
                            PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                            0U);
                    }
                    result_generated = aethor_app_complete_action(
                        PROTOCOL_COMMAND_RESULT_FAILED,
                        (uint16_t)runtime_fault,
                        timestamp_us);
                }
                result_generated |= aethor_app_fail_queued_motion(
                    (uint16_t)runtime_fault,
                    timestamp_us);
                (void)arm_controller_latch_runtime_fault(
                    &application_controller,
                    runtime_fault,
                    runtime_fault_detail,
                    timestamp_us);
#if !AETHOR_ADRC_BENCH
                application_emergency_disable_read_index = 0U;
#endif
                (void)motor_runtime_build_emergency_disable(
                    &application_motor_runtime,
                    &application_emergency_disable_batch);
                protocol_engine_cancel_pending_normal_commands(
                    &application_protocol_engine);
                return (uint8_t)(result_generated | deferred_result_flushed);
            }
        }
        result_generated = aethor_app_check_one_shot_motor_safety(
            &motor_snapshot,
            timestamp_us,
            &one_shot_motor_failure_detected);
        if (one_shot_motor_failure_detected != 0U)
        {
            return (uint8_t)(result_generated | deferred_result_flushed);
        }

        if (!aethor_app_debug_ui_owns_link_lifecycle() &&
            (aethor_app_active_action_owns_link_lifecycle() == 0U) &&
            (protocol_engine_watchdog_expired(&application_protocol_engine,
                                              timestamp_us) != 0U))
        {
            ProtocolCommandResult timeout_result;

            aethor_app_widen_pending_stop_before_global_cancel();
            (void)motor_runtime_abort_active_parameter_sequences(
                &application_motor_runtime,
                timestamp_us);
            if (application_action.state != AETHOR_APP_ACTION_IDLE)
            {
                (void)aethor_app_complete_action(
                    PROTOCOL_COMMAND_RESULT_CANCELLED,
                    0U,
                    timestamp_us);
            }
            result_generated |= aethor_app_fail_queued_motion(
                (uint16_t)ARM_FAULT_LINK_TIMEOUT,
                timestamp_us);
            (void)arm_controller_force_stop_disable(&application_controller,
                                                     timestamp_us);
            protocol_engine_cancel_pending_normal_commands(
                &application_protocol_engine);
#if !AETHOR_ADRC_BENCH
            application_emergency_disable_read_index = 0U;
#endif
            (void)motor_runtime_build_emergency_disable(
                &application_motor_runtime,
                &application_emergency_disable_batch);
            memset(&timeout_result, 0, sizeof(timeout_result));
            timeout_result.session_id = application_protocol_engine.session_id;
            timeout_result.type = PROTOCOL_COMMAND_LINK_TIMEOUT;
            timeout_result.code = PROTOCOL_COMMAND_RESULT_STOPPED;
            timeout_result.completed_at_us = timestamp_us;
            result_generated |=
                aethor_app_submit_or_defer_result(&timeout_result);
        }
        else if (application_action.state != AETHOR_APP_ACTION_IDLE)
        {
            ProtocolCommand stop_command;

            if ((aethor_app_deferred_result_has_capacity() != 0U) &&
                (protocol_engine_pop_stop_command(&application_protocol_engine,
                                                  &stop_command) != 0U))
            {
                uint8_t cancel_generated;
                uint8_t original_action_mask = application_action.command.motor_mask;
                uint8_t stop_generated;

                /* A new STOP cannot abandon an earlier unconfirmed local disable. */
                stop_command.motor_mask |= application_debug_ui.unconfirmed_disable_mask;
                stop_command.cleanup_after_stop |= (uint8_t)(application_debug_ui.unconfirmed_disable_mask != 0U);
                stop_command.cleanup_after_stop |= (uint8_t)(
                    (application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) ||
                    (application_action.command.cleanup_after_stop != 0U));
                stop_command.motor_mask |= original_action_mask;
                stop_command.control_mode =
                    application_action.command.control_mode;
                stop_command.mit_kp = application_action.command.mit_kp;
                stop_command.mit_kd = application_action.command.mit_kd;
                stop_command.mit_torque_ff_nm =
                    application_action.command.mit_torque_ff_nm;
                (void)motor_runtime_abort_active_parameter_sequences(
                    &application_motor_runtime,
                    timestamp_us);
                cancel_generated = aethor_app_complete_action(
                    PROTOCOL_COMMAND_RESULT_CANCELLED,
                    0U,
                    timestamp_us);
                stop_generated = aethor_app_start_lifecycle_action(
                    &stop_command,
                    &motor_snapshot,
                    snapshot_status,
                    timestamp_us);

                result_generated = (uint8_t)(cancel_generated | stop_generated);
            }
            else if ((application_action.command.type ==
                      PROTOCOL_COMMAND_STOP) &&
                     (aethor_app_deferred_result_has_capacity() == 0U))
            {
                ProtocolCommand pending_stop_command;

                if ((protocol_engine_widen_pending_stop_mask(
                         &application_protocol_engine,
                         application_action.command.motor_mask,
                         &pending_stop_command) != 0U) &&
                    (pending_stop_command.motor_mask !=
                     application_action.command.motor_mask))
                {
                    ProtocolCommand widened_active_stop =
                        application_action.command;

                    widened_active_stop.motor_mask =
                        pending_stop_command.motor_mask;
                    (void)motor_runtime_abort_active_parameter_sequences(
                        &application_motor_runtime,
                        timestamp_us);
                    result_generated = aethor_app_start_lifecycle_action(
                        &widened_active_stop,
                        &motor_snapshot,
                        snapshot_status,
                        timestamp_us);
                }
                else
                {
                    result_generated = aethor_app_service_active_action(
                        &motor_snapshot,
                        timestamp_us);
                }
            }
            else
            {
                result_generated = aethor_app_service_active_action(
                    &motor_snapshot,
                    timestamp_us);
            }
        }
        else
        {
            ProtocolCommand stop_command;

            if ((aethor_app_deferred_result_has_capacity() != 0U) &&
                (protocol_engine_pop_stop_command(&application_protocol_engine,
                                                  &stop_command) != 0U))
            {
                ProtocolCommand queued_motion;
                uint8_t cancel_generated = 0U;

                stop_command.motor_mask |= application_debug_ui.unconfirmed_disable_mask;
                stop_command.cleanup_after_stop |= (uint8_t)(application_debug_ui.unconfirmed_disable_mask != 0U);
                if (protocol_engine_take_queued_active_motion(
                        &application_protocol_engine,
                        &queued_motion) != 0U)
                {
                    stop_command.cleanup_after_stop |= (uint8_t)(
                        queued_motion.origin == DEBUG_UI_ORIGIN_LOCAL_UI);
                    stop_command.motor_mask |= queued_motion.motor_mask;
                    stop_command.control_mode = queued_motion.control_mode;
                    stop_command.mit_kp = queued_motion.mit_kp;
                    stop_command.mit_kd = queued_motion.mit_kd;
                    stop_command.mit_torque_ff_nm =
                        queued_motion.mit_torque_ff_nm;
                    cancel_generated = aethor_app_cancel_queued_command(
                        &queued_motion,
                        timestamp_us);
                }
                result_generated = (uint8_t)(
                    cancel_generated |
                    aethor_app_start_lifecycle_action(&stop_command,
                                                      &motor_snapshot,
                                                      snapshot_status,
                                                      timestamp_us));
            }
            else if ((aethor_app_deferred_result_count() == 0U) &&
#if AETHOR_ADRC_LCD_INTEGRATED
                     (application_motor_runtime.discovery_active == 0U) &&
#endif
                     (protocol_engine_pop_command(&application_protocol_engine,
                                                  &command) != 0U))
            {
                result_generated = aethor_app_start_lifecycle_action(
                    &command,
                    &motor_snapshot,
                    snapshot_status,
                    timestamp_us);
            }
        }
    }
    return (uint8_t)(result_generated | deferred_result_flushed);
#endif
}

/** @brief Advances epoch without making zero a valid authorization. Caller holds critical. */
static void aethor_app_debug_ui_revoke(DebugUiAuthority authority)
{
    ++application_debug_ui.epoch;
    if (application_debug_ui.epoch == 0U) { ++application_debug_ui.epoch; }
    application_debug_ui.authority = authority;
    application_protocol_engine.local_control_locked =
        (uint8_t)(authority != DEBUG_UI_AUTHORITY_REMOTE);
}

/** @brief Binds failed cleanup to the original mask and feedback newer than the attempt. */
static void aethor_app_debug_ui_lock_targets(uint8_t motor_mask, uint64_t timestamp_us)
{
    uint8_t joint_index;
    application_debug_ui.unconfirmed_disable_mask |= motor_mask;
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        if ((motor_mask & (1U << joint_index)) != 0U)
        { application_debug_ui.disable_confirmation_after_us[joint_index] = timestamp_us; }
    }
}

#if AETHOR_DEBUG_UI_ALLOW_MOTION
/** @brief Refuses recovery until every original target has fresh post-attempt safe feedback. */
static DebugUiReason aethor_app_debug_ui_recovery_reason(const MotorFeedbackSnapshot *snapshot)
{
    uint8_t joint_index;
    uint8_t motor_mask = application_debug_ui.unconfirmed_disable_mask;
    if ((snapshot->valid_joint_mask & motor_mask) != motor_mask)
    { return DEBUG_UI_REASON_STALE_FEEDBACK; }
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        const MotorJointFeedback *feedback = &snapshot->joints[joint_index];
        if ((motor_mask & (1U << joint_index)) == 0U) { continue; }
        if (feedback->timestamp_us <= application_debug_ui.disable_confirmation_after_us[joint_index])
        { return DEBUG_UI_REASON_STALE_FEEDBACK; }
        if ((feedback->driver_state != S3519_DRIVER_STATE_DISABLED) ||
            (feedback->fault_flags != 0U) || !isfinite(feedback->velocity_rad_s) ||
            (fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S))
        { return DEBUG_UI_REASON_NOT_DISABLED; }
    }
    return DEBUG_UI_REASON_NONE;
}
#endif

/** @brief Runs fail-safe disable despite result backpressure; accepted terminals are retained.
 * New saturated STOP submissions have only their synchronous STOP_LATCHED response. Their
 * id-free mask is consumed here, while old commands are cancelled only when storage exists.
 * The persistent recovery lock remains even after this bounded physical cleanup ends.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_debug_ui_service_stop_intent(uint64_t timestamp_us, uint8_t *result_generated)
{
#if !AETHOR_DEBUG_UI_ALLOW_MOTION
    (void)timestamp_us;
    (void)result_generated;
    return 0U;
#else
    uint8_t intent_mask;
    uint8_t waiting_stop_mask = 0U;
    uint8_t sequence;
    uint8_t cancel_count;
    MotorFeedbackSnapshot snapshot;
    ProtocolCommand cancelled;
    aethor_app_enter_task_critical();
    intent_mask = debug_ui_mailbox_take_stop_intent(&application_debug_ui.mailbox);
    if (application_protocol_engine.stop_read_sequence != application_protocol_engine.stop_write_sequence)
    { waiting_stop_mask |= application_protocol_engine.stop_command.motor_mask; }
    for (sequence = 0U; sequence < DEBUG_UI_RESULT_CAPACITY; ++sequence)
    {
        const DebugUiMailboxTransaction *transaction = &application_debug_ui.mailbox.transactions[sequence];
        if (transaction->used && !transaction->completion_ready &&
            (transaction->request.operation == DEBUG_UI_OPERATION_STOP))
        { waiting_stop_mask |= (uint8_t)(1U << (transaction->request.target_motor_id - 1U)); }
    }
    /* Accepted STOP identities stay in place while physical safety bypasses full FIFOs. */
    if ((application_debug_ui.fallback_stop_mask == 0U) && !aethor_app_deferred_result_has_capacity())
    { intent_mask |= waiting_stop_mask; }
    /* A new cross-target USB STOP during fallback must widen frames before cancellation. */
    if (application_debug_ui.fallback_stop_mask != 0U)
    { intent_mask |= (uint8_t)(waiting_stop_mask & ~application_debug_ui.fallback_stop_mask); }
    if (intent_mask != 0U)
    {
        intent_mask |= waiting_stop_mask;
        intent_mask |= application_debug_ui.unconfirmed_disable_mask;
        if (application_action.state != AETHOR_APP_ACTION_IDLE)
        { intent_mask |= application_action.command.motor_mask; }
        for (sequence = application_protocol_engine.command_read_sequence;
             sequence != application_protocol_engine.command_write_sequence; ++sequence)
        { intent_mask |= application_protocol_engine.commands[sequence % PROTOCOL_ENGINE_COMMAND_CAPACITY].motor_mask; }
        if (application_protocol_engine.stop_read_sequence != application_protocol_engine.stop_write_sequence)
        { intent_mask |= application_protocol_engine.stop_command.motor_mask; }
        application_debug_ui.fallback_stop_mask |= intent_mask;
        aethor_app_debug_ui_lock_targets(application_debug_ui.fallback_stop_mask, timestamp_us);
        if (application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_FAULT)
        { aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT); }
        application_debug_ui.fallback_deadline_us = timestamp_us + AETHOR_APP_ACTION_TIMEOUT_US;
    }
    aethor_app_exit_task_critical();
    if (application_debug_ui.fallback_stop_mask == 0U) { return 0U; }
    if (intent_mask != 0U)
    {
        (void)motor_runtime_abort_active_parameter_sequences(&application_motor_runtime, timestamp_us);
        application_action.control_group_ready = 0U;
        application_debug_ui.fallback_frame_read_index = 0U;
        if (motor_runtime_build_emergency_disable_subset(&application_motor_runtime,
            application_debug_ui.fallback_stop_mask, &application_debug_ui.fallback_frames) != MOTOR_RUNTIME_STATUS_OK)
        { application_debug_ui.fallback_frames.count = 0U; }
    }
    if ((application_action.state != AETHOR_APP_ACTION_IDLE) && aethor_app_deferred_result_has_capacity())
    { *result_generated |= aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_CANCELLED, 0U, timestamp_us); }
    for (cancel_count = 0U; cancel_count < PROTOCOL_ENGINE_COMMAND_CAPACITY + 1U; ++cancel_count)
    {
        if (!aethor_app_deferred_result_has_capacity() ||
            !protocol_engine_pop_command(&application_protocol_engine, &cancelled)) { break; }
        *result_generated |= aethor_app_cancel_queued_command(&cancelled, timestamp_us);
    }
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot);
    aethor_app_enter_task_critical();
    if ((application_action.state == AETHOR_APP_ACTION_IDLE) &&
        (application_protocol_engine.command_read_sequence == application_protocol_engine.command_write_sequence) &&
        (application_protocol_engine.stop_read_sequence == application_protocol_engine.stop_write_sequence) &&
        (application_debug_ui.fallback_frame_read_index >= application_debug_ui.fallback_frames.count) &&
        ((aethor_app_debug_ui_recovery_reason(&snapshot) == DEBUG_UI_REASON_NONE) ||
         (timestamp_us >= application_debug_ui.fallback_deadline_us)))
    { application_debug_ui.fallback_stop_mask = 0U; }
    aethor_app_exit_task_critical();
    return 1U;
#endif
}
#endif

/** @brief Reports actual executor and published queues, including source-neutral STOP. */
static bool aethor_app_debug_ui_executor_busy(void)
{
    return (application_action.state != AETHOR_APP_ACTION_IDLE) ||
#if AETHOR_ADRC_LCD_INTEGRATED
        (application_motor_runtime.discovery_active != 0U) ||
#endif
        (application_debug_ui.fallback_stop_mask != 0U) ||
        (application_debug_ui.mailbox.stop_intent_mask != 0U) ||
        (application_protocol_engine.command_write_sequence != application_protocol_engine.command_read_sequence) ||
        (application_protocol_engine.stop_write_sequence != application_protocol_engine.stop_read_sequence) ||
        (application_protocol_engine.active_motion_request_id != 0U) ||
        (application_protocol_engine.active_stop_request_id != 0U) ||
        (aethor_app_deferred_result_count() != 0U);
}

/** @brief Keeps USB disconnect from cancelling a local pending/active/cleanup lifecycle. */
#if !AETHOR_ADRC_BENCH
static bool aethor_app_debug_ui_owns_link_lifecycle(void)
{
    bool owns_lifecycle;
    aethor_app_enter_task_critical();
    owns_lifecycle = (application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE) ||
        debug_ui_mailbox_busy(&application_debug_ui.mailbox, true) ||
        (application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) ||
        (application_action.command.cleanup_after_stop != 0U);
    aethor_app_exit_task_critical();
    return owns_lifecycle;
}
#endif

/** @brief Validates explicit physical bounds without inventing commissioning defaults. */
static bool aethor_app_debug_ui_profile_valid(const DebugUiMotorProfile *profile)
{
    if ((profile == NULL) || (profile->pos_valid > 1U) || (profile->mit_valid > 1U)) { return false; }
    if ((profile->pos_valid == 0U) && (profile->mit_valid == 0U)) { return true; }
    if (!isfinite(profile->position_min_rad) || !isfinite(profile->position_max_rad) ||
        !isfinite(profile->max_delta_rad) || !isfinite(profile->max_speed_rad_s) ||
        (profile->position_min_rad >= profile->position_max_rad) ||
        (profile->max_delta_rad <= 0.0F) || (profile->max_speed_rad_s <= 0.0F)) { return false; }
    if (profile->mit_valid != 0U)
    {
        if (!isfinite(profile->mit_position_min_rad) || !isfinite(profile->mit_position_max_rad) ||
            !isfinite(profile->mit_max_delta_rad) || !isfinite(profile->mit_max_speed_rad_s) ||
            (profile->mit_position_min_rad >= profile->mit_position_max_rad) ||
            (profile->mit_max_delta_rad <= 0.0F) || (profile->mit_max_speed_rad_s <= 0.0F)) { return false; }
        if (!isfinite(profile->kp_min) || !isfinite(profile->kp_max) ||
            !isfinite(profile->kd_min) || !isfinite(profile->kd_max) ||
            !isfinite(profile->default_kp) || !isfinite(profile->default_kd) ||
            (profile->kp_min <= 0.0F) || (profile->kp_max > S3519_KP_MAX) ||
            (profile->kd_min <= 0.0F) || (profile->kd_max > S3519_KD_MAX) ||
            (profile->default_kp < profile->kp_min) || (profile->default_kp > profile->kp_max) ||
            (profile->default_kd < profile->kd_min) || (profile->default_kd > profile->kd_max) ||
            (profile->hold_min_ms < 100U) || (profile->hold_max_ms > 10000U) ||
            (profile->hold_min_ms > profile->hold_max_ms)) { return false; }
    }
    return true;
}

#if AETHOR_DEBUG_UI_S3519_PROFILE_MASK
/** @brief Derives each selected motor's bounds from its own discovered protocol limits.
 * Caller holds the task critical section. Any bound change invalidates old reviewed requests.
 * Same-model gains are a user-requested preset, not hardware acceptance for IDs1..6.
 */
static void aethor_app_debug_ui_refresh_motor_profiles(void)
{
    MotorPositionVelocityLimits limits;
    DebugUiMotorProfile profile;
    uint8_t joint_index;
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        DebugUiMotorProfile *current = &application_debug_ui.profiles[joint_index];
        if ((AETHOR_DEBUG_UI_S3519_PROFILE_MASK & (1U << joint_index)) == 0U) { continue; }
        memset(&profile, 0, sizeof(profile));
        if ((motor_runtime_get_position_velocity_limits(&application_motor_runtime, joint_index, &limits) ==
             MOTOR_RUNTIME_STATUS_OK) && isfinite(2.0F * limits.position_max_rad))
        {
            profile.pos_valid = 1U;
            profile.position_min_rad = -limits.position_max_rad;
            profile.position_max_rad = limits.position_max_rad;
            profile.max_delta_rad = 2.0F * limits.position_max_rad;
            profile.max_speed_rad_s = limits.move_speed_limit_rad_s;
#if AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE
            profile.mit_valid = 1U;
            profile.mit_position_min_rad = -limits.position_max_rad;
            profile.mit_position_max_rad = limits.position_max_rad;
            profile.mit_max_delta_rad = 2.0F * limits.position_max_rad;
            profile.mit_max_speed_rad_s = limits.move_speed_limit_rad_s;
            profile.kp_min = profile.kp_max = profile.default_kp =
                80.0F / aethor_app_lcd_position_ratio(joint_index);
            profile.kd_min = profile.kd_max = profile.default_kd = 0.2F;
            profile.hold_min_ms = 100U;
            profile.hold_max_ms = 1000U;
#endif
        }
        if (memcmp(current, &profile, sizeof(profile)) != 0)
        {
            *current = profile;
            ++application_debug_ui.reference_generation[joint_index];
        }
    }
}
#endif

/** @brief Updates coordinate generation on discovered-mode/verification or POS bound changes. */
static void aethor_app_debug_ui_refresh_reference(uint64_t timestamp_us)
{
    uint8_t joint_index;
    (void)timestamp_us; /* Freshness is an execution gate, not a coordinate/reference change. */
#if AETHOR_DEBUG_UI_S3519_PROFILE_MASK
    aethor_app_debug_ui_refresh_motor_profiles();
#endif
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        uint32_t signature = discovery->observed_control_mode |
            ((uint32_t)discovery->verified_fields_mask << 8U);
        if ((application_debug_ui.reference_generation[joint_index] == 0U) ||
            (application_debug_ui.observed_mode[joint_index] != signature))
        {
            application_debug_ui.observed_mode[joint_index] = signature;
            ++application_debug_ui.reference_generation[joint_index];
        }
    }
}

/** @brief Installs an explicit profile while no local authority or executor work exists. */
bool aethor_app_debug_ui_set_motor_profile(uint8_t motor_id, const DebugUiMotorProfile *profile)
{
    bool installed = false;
    if ((application_initialized == 0U) || (motor_id == 0U) ||
        (motor_id > DEBUG_UI_MOTOR_COUNT) || !aethor_app_debug_ui_profile_valid(profile)) { return false; }
    aethor_app_enter_task_critical();
    if ((application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE) &&
        !aethor_app_debug_ui_executor_busy() &&
        (debug_ui_mailbox_result_count(&application_debug_ui.mailbox) == 0U))
    {
        application_debug_ui.profiles[motor_id - 1U] = *profile;
        ++application_debug_ui.reference_generation[motor_id - 1U];
        installed = true;
    }
    aethor_app_exit_task_critical();
    return installed;
}

/** @brief Copies a profile under the same task critical hooks used for snapshots. */
bool aethor_app_debug_ui_get_motor_profile(uint8_t motor_id, DebugUiMotorProfile *profile)
{
    if ((application_initialized == 0U) || (profile == NULL) ||
        (motor_id == 0U) || (motor_id > DEBUG_UI_MOTOR_COUNT)) { return false; }
    aethor_app_enter_task_critical();
    *profile = application_debug_ui.profiles[motor_id - 1U];
    aethor_app_exit_task_critical();
    return true;
}

/** @brief Reserves one immutable local request without touching the protocol command ring. */
DebugUiReason aethor_app_debug_ui_submit(const DebugUiRequest *request)
{
#if !AETHOR_ADRC_BENCH
    DebugUiReason reason;
#endif
    if (application_initialized == 0U) { return DEBUG_UI_REASON_NOT_READY; }
#if AETHOR_ADRC_LCD_INTEGRATED
    aethor_app_enter_task_critical();
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING &&
        request != NULL && request->operation == DEBUG_UI_OPERATION_STOP)
    { application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_LCD; }
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
    {
        if (request == NULL) { aethor_app_exit_task_critical(); return DEBUG_UI_REASON_NOT_READY; }
        if (request->operation != DEBUG_UI_OPERATION_STOP)
        { aethor_app_exit_task_critical(); return DEBUG_UI_REASON_DISABLED; }
        adrc_app_bridge_request_stop(&application_adrc_bridge);
        aethor_app_exit_task_critical();
        return DEBUG_UI_REASON_STOP_LATCHED;
    }
    aethor_app_exit_task_critical();
#endif
#if AETHOR_ADRC_BENCH
    if (request == NULL) { return DEBUG_UI_REASON_NOT_READY; }
    if (request->operation != DEBUG_UI_OPERATION_STOP) { return DEBUG_UI_REASON_DISABLED; }
    aethor_app_enter_task_critical();
    adrc_app_bridge_request_stop(&application_adrc_bridge);
    aethor_app_exit_task_critical();
    return DEBUG_UI_REASON_STOP_LATCHED;
#else
    aethor_app_enter_task_critical();
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING &&
        request != NULL && request->operation == DEBUG_UI_OPERATION_STOP)
    { application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_LCD; }
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
    {
        if (request != NULL && request->operation == DEBUG_UI_OPERATION_STOP)
        {
            adrc_app_bridge_request_stop(&application_adrc_bridge);
            aethor_app_exit_task_critical();
            return DEBUG_UI_REASON_STOP_LATCHED;
        }
        aethor_app_exit_task_critical();
        return request == NULL ? DEBUG_UI_REASON_NOT_READY : DEBUG_UI_REASON_DISABLED;
    }
#endif
    reason = debug_ui_mailbox_submit(&application_debug_ui.mailbox, request);
    if ((reason == DEBUG_UI_REASON_STOP_LATCHED) && (AETHOR_DEBUG_UI_ALLOW_MOTION == 0U))
    {
        (void)debug_ui_mailbox_take_stop_intent(&application_debug_ui.mailbox);
        reason = DEBUG_UI_REASON_DISABLED;
    }
    if (reason == DEBUG_UI_REASON_RESULT_BACKPRESSURE)
    { ++application_debug_ui.diagnostics.result_backpressure_count; }
    aethor_app_exit_task_critical();
    return reason;
#endif
}

/** @brief Consumes exactly one retained admission acknowledgement. */
bool aethor_app_debug_ui_poll_admission(DebugUiAdmission *admission)
{
    bool available;
    aethor_app_enter_task_critical();
    available = debug_ui_mailbox_poll_admission(&application_debug_ui.mailbox, admission);
    aethor_app_exit_task_critical();
    return available;
}

/** @brief Consumes exactly one retained terminal acknowledgement. */
bool aethor_app_debug_ui_poll_completion(DebugUiCompletion *completion)
{
    bool available;
    aethor_app_enter_task_critical();
    available = debug_ui_mailbox_poll_completion(&application_debug_ui.mailbox, completion);
    aethor_app_exit_task_critical();
    return available;
}

/** @brief Publishes UI-owned health only; uint64 timestamps are copied while protected. */
void aethor_app_debug_ui_update_health(const DebugUiHealth *health)
{
    aethor_app_enter_task_critical();
    debug_ui_mailbox_update_health(&application_debug_ui.mailbox, health);
    aethor_app_exit_task_critical();
}

/** @brief Reports request/health work for the platform's task notification. */
bool aethor_app_debug_ui_needs_service(void)
{
    bool pending;
    aethor_app_enter_task_critical();
    pending = debug_ui_mailbox_needs_service(&application_debug_ui.mailbox);
    aethor_app_exit_task_critical();
    return pending;
}

/** @brief Copies all seven feedback rows and explicit validity under a bounded critical. */
bool aethor_app_debug_ui_get_snapshot(uint64_t timestamp_us, DebugUiSnapshot *snapshot)
{
    MotorFeedbackSnapshot motors;
    ArmSnapshot arm;
    uint8_t joint_index;
    if ((application_initialized == 0U) || (snapshot == NULL)) { return false; }
    aethor_app_enter_task_critical();
    memset(snapshot, 0, sizeof(*snapshot));
    memset(&motors, 0, sizeof(motors));
    memset(&arm, 0, sizeof(arm));
    (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &motors);
    (void)arm_controller_get_snapshot(&application_controller, &arm);
    aethor_app_debug_ui_refresh_reference(timestamp_us);
    snapshot->timestamp_us = timestamp_us;
    snapshot->generation = ++application_debug_ui.generation;
    snapshot->boot_id = application_protocol_engine.boot_id;
    snapshot->epoch = application_debug_ui.epoch;
    snapshot->authority = application_debug_ui.authority;
    snapshot->health = application_debug_ui.mailbox.health;
    snapshot->diagnostics = application_debug_ui.diagnostics;
    snapshot->diagnostics.control_deadline_miss_count = application_diagnostics.counters.control_deadline_miss_count;
    snapshot->diagnostics.can_rx_drop_count = application_diagnostics.counters.can_rx_overflow_count;
    snapshot->diagnostics.usb_rx_drop_count = application_diagnostics.counters.usb_rx_overflow_count;
    snapshot->diagnostics.can_tx_error_count = application_diagnostics.counters.can_tx_error_count;
    snapshot->diagnostics.usb_telemetry_drop_count = application_diagnostics.counters.usb_telemetry_drop_count;
    snapshot->diagnostics.minimum_stack_words = application_diagnostics.counters.minimum_stack_words;
    snapshot->diagnostics.minimum_heap_bytes = application_diagnostics.counters.minimum_heap_bytes;
    snapshot->bench_profile = (uint8_t)(AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE);
    snapshot->motion_enabled = AETHOR_DEBUG_UI_ALLOW_MOTION;
    snapshot->mit_enabled = AETHOR_DEBUG_UI_ALLOW_MIT;
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
    { snapshot->motion_enabled = 0U; snapshot->mit_enabled = 0U; }
#endif
    snapshot->target_motor_id = application_debug_ui.target_motor_id;
    snapshot->unconfirmed_disable_mask = application_debug_ui.unconfirmed_disable_mask;
    snapshot->retained_result_count = debug_ui_mailbox_result_count(&application_debug_ui.mailbox);
    snapshot->pending = (uint8_t)debug_ui_mailbox_busy(&application_debug_ui.mailbox, false);
    snapshot->stop_pending = (uint8_t)(debug_ui_mailbox_busy(&application_debug_ui.mailbox, true) ||
        (application_debug_ui.fallback_stop_mask != 0U));
    snapshot->usb_connected = (uint8_t)((application_protocol_engine.session_active != 0U) &&
        (timestamp_us >= application_protocol_engine.last_valid_request_at_us) &&
        (timestamp_us - application_protocol_engine.last_valid_request_at_us <= PROTOCOL_ENGINE_WATCHDOG_TIMEOUT_US));
    snapshot->arm_state = (uint8_t)arm.state;
    snapshot->arm_fault = (uint8_t)arm.fault;
    snapshot->active = (uint8_t)aethor_app_debug_ui_executor_busy();
    snapshot->active_identity.origin = application_protocol_engine.active_motion_origin;
    snapshot->active_identity.epoch = application_protocol_engine.active_motion_epoch;
    snapshot->active_identity.request_id = application_protocol_engine.active_motion_request_id;
    snapshot->stop_identity.origin = application_protocol_engine.active_stop_origin;
    snapshot->stop_identity.epoch = application_protocol_engine.active_stop_epoch;
    snapshot->stop_identity.request_id = application_protocol_engine.active_stop_request_id;
    if (application_action.state != AETHOR_APP_ACTION_IDLE)
    {
        snapshot->active_identity.origin = application_action.command.origin;
        snapshot->active_identity.epoch = application_action.command.session_id;
        snapshot->active_identity.request_id = application_action.command.request_id;
        snapshot->active_motor_mask = application_action.command.motor_mask;
        snapshot->active_stage = (DebugUiStage)aethor_app_current_one_shot_stage();
        /* The display shows current cleanup progress; terminal.stage retains its cause. */
        switch (application_action.state)
        {
            case AETHOR_APP_ACTION_MODE_SWITCH:
            case AETHOR_APP_ACTION_BENCH_MODE_SWITCH: snapshot->active_stage = DEBUG_UI_STAGE_MODE; break;
            case AETHOR_APP_ACTION_DISCOVERY: snapshot->active_stage = DEBUG_UI_STAGE_DISCOVERY; break;
            case AETHOR_APP_ACTION_ENABLE_WAIT: snapshot->active_stage = DEBUG_UI_STAGE_ENABLE; break;
            case AETHOR_APP_ACTION_CLEAR_FAULT_WAIT: snapshot->active_stage = DEBUG_UI_STAGE_CLEAR; break;
            case AETHOR_APP_ACTION_BENCH_MOVE_WAIT:
            case AETHOR_APP_ACTION_MOTION:
            case AETHOR_APP_ACTION_CONTROLLED_STOP: snapshot->active_stage = DEBUG_UI_STAGE_MOTION; break;
            case AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT: snapshot->active_stage = DEBUG_UI_STAGE_HOLD; break;
            case AETHOR_APP_ACTION_DISABLE_WAIT:
            case AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT: snapshot->active_stage = DEBUG_UI_STAGE_DISABLE; break;
            default: break;
        }
    }
    else if ((application_protocol_engine.stop_read_sequence != application_protocol_engine.stop_write_sequence) ||
             (application_protocol_engine.command_read_sequence != application_protocol_engine.command_write_sequence))
    {
        const ProtocolCommand *pending =
            (application_protocol_engine.stop_read_sequence != application_protocol_engine.stop_write_sequence)
                ? &application_protocol_engine.stop_command
                : &application_protocol_engine.commands[
                    application_protocol_engine.command_read_sequence % PROTOCOL_ENGINE_COMMAND_CAPACITY];
        snapshot->active_motor_mask = pending->motor_mask;
        if (pending->type == PROTOCOL_COMMAND_STOP)
        { snapshot->active_motor_mask |= application_debug_ui.unconfirmed_disable_mask; }
        snapshot->active_identity.origin = pending->origin;
        snapshot->active_identity.epoch = pending->session_id;
        snapshot->active_identity.request_id = pending->request_id;
        snapshot->active_stage = DEBUG_UI_STAGE_VALIDATE;
    }
    else
    {
        const DebugUiRequest *waiting_request = NULL;
        uint8_t slot_index;
        /* Mailbox work can precede ProtocolTask; its immutable target already has meaning. */
        for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
        {
            const DebugUiMailboxTransaction *transaction = &application_debug_ui.mailbox.transactions[slot_index];
            if (!transaction->used || transaction->completion_ready) { continue; }
            if ((waiting_request == NULL) || (transaction->request.operation == DEBUG_UI_OPERATION_STOP))
            { waiting_request = &transaction->request; }
            if (transaction->request.operation == DEBUG_UI_OPERATION_STOP) { break; }
        }
        if (waiting_request != NULL)
        {
            snapshot->active_identity = waiting_request->identity;
            snapshot->active_motor_mask = (uint8_t)(1U << (waiting_request->target_motor_id - 1U));
            snapshot->active_stage = DEBUG_UI_STAGE_VALIDATE;
            if (waiting_request->operation == DEBUG_UI_OPERATION_STOP)
            {
                snapshot->stop_identity = waiting_request->identity;
                snapshot->active_motor_mask |= application_debug_ui.unconfirmed_disable_mask;
            }
        }
    }
    if (application_debug_ui.mailbox.stop_intent_mask != 0U)
    {
        snapshot->active_motor_mask |= (uint8_t)(application_debug_ui.mailbox.stop_intent_mask |
            application_debug_ui.unconfirmed_disable_mask);
        snapshot->active_identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
        snapshot->active_identity.epoch = application_debug_ui.epoch;
        snapshot->active_identity.request_id = 0U;
    }
    if (application_debug_ui.fallback_stop_mask != 0U)
    {
        snapshot->active_motor_mask |= application_debug_ui.fallback_stop_mask;
        snapshot->active_stage = DEBUG_UI_STAGE_DISABLE;
        snapshot->active_identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
        snapshot->active_identity.epoch = application_debug_ui.epoch;
        snapshot->active_identity.request_id = 0U; /* Id-free safety cleanup, never a forged transaction. */
    }
    if (snapshot->active_motor_mask != 0U)
    {
        for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
        {
            if ((snapshot->active_motor_mask & (1U << joint_index)) != 0U)
            { snapshot->target_motor_id = (uint8_t)(joint_index + 1U); break; }
        }
    }
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        DebugUiMotorView *view = &snapshot->motors[joint_index];
        const MotorJointFeedback *feedback = &motors.joints[joint_index];
        const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        uint64_t age_us = timestamp_us >= feedback->timestamp_us
            ? timestamp_us - feedback->timestamp_us : UINT64_MAX;
        view->motor_id = (uint8_t)(joint_index + 1U);
        view->feedback_seen = application_motor_runtime.bank.motors[joint_index].feedback_valid;
        view->feedback_valid = (uint8_t)((motors.valid_joint_mask & (1U << joint_index)) != 0U);
        view->feedback_age_ms = age_us / 1000ULL > UINT32_MAX ? UINT32_MAX : (uint32_t)(age_us / 1000ULL);
        view->enabled = (uint8_t)(feedback->driver_state == S3519_DRIVER_STATE_ENABLED);
        view->driver_state = feedback->driver_state;
        view->position_rad = feedback->position_rad / aethor_app_lcd_position_ratio(joint_index);
        view->velocity_rad_s = feedback->velocity_rad_s;
        view->torque_nm = feedback->torque_nm;
        view->mos_temperature_c = feedback->mos_temperature_c;
        view->rotor_temperature_c = feedback->rotor_temperature_c;
        view->fault_flags = feedback->fault_flags;
        view->identity_verified = (uint8_t)((discovery->verified_fields_mask & MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK) == MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK);
        view->mode_verified = (uint8_t)((discovery->verified_fields_mask & MOTOR_DISCOVERY_MODE_FIELDS_MASK) == MOTOR_DISCOVERY_MODE_FIELDS_MASK);
        view->ranges_verified = (uint8_t)((discovery->verified_fields_mask & MOTOR_DISCOVERY_RANGE_FIELDS_MASK) == MOTOR_DISCOVERY_RANGE_FIELDS_MASK);
        if ((view->mode_verified != 0U) && ((discovery->observed_control_mode == 1U) || (discovery->observed_control_mode == 2U)))
        { view->actual_mode = (DebugUiMode)discovery->observed_control_mode; }
        view->reference_generation = application_debug_ui.reference_generation[joint_index];
        view->position_max_rad = discovery->ranges.position_max_rad / aethor_app_lcd_position_ratio(joint_index);
        view->velocity_max_rad_s = discovery->ranges.velocity_max_rad_s;
        view->torque_max_nm = discovery->ranges.torque_max_nm;
        view->maximum_speed_rad_s = discovery->maximum_speed_rad_s;
        view->profile = application_debug_ui.profiles[joint_index];
        /* Publish output-shaft bounds; App validation retains raw protocol bounds. */
        view->profile.position_min_rad /= aethor_app_lcd_position_ratio(joint_index);
        view->profile.position_max_rad /= aethor_app_lcd_position_ratio(joint_index);
        view->profile.max_delta_rad /= aethor_app_lcd_position_ratio(joint_index);
        view->profile.mit_position_min_rad /= aethor_app_lcd_position_ratio(joint_index);
        view->profile.mit_position_max_rad /= aethor_app_lcd_position_ratio(joint_index);
        view->profile.mit_max_delta_rad /= aethor_app_lcd_position_ratio(joint_index);
        /* Kp multiplies position error, so its coordinate conversion is inverse. */
        view->profile.kp_min *= aethor_app_lcd_position_ratio(joint_index);
        view->profile.kp_max *= aethor_app_lcd_position_ratio(joint_index);
        view->profile.default_kp *= aethor_app_lcd_position_ratio(joint_index);
        if (application_motor_runtime.position_read.joint_index == joint_index)
        {
            unsigned register_index;
            const MotorRegisterPosition *positions = &application_motor_runtime.position_read;
            view->register_seen_mask = positions->seen_mask;
            view->register_timeout_count = positions->timeout_count;
            for (register_index = 0U; register_index < 2U; ++register_index)
            {
                uint64_t register_age = timestamp_us >= positions->sample_us[register_index] ?
                    (timestamp_us - positions->sample_us[register_index]) / 1000ULL : UINT64_MAX;
                view->register_position[register_index] = positions->values[register_index];
                view->register_age_ms[register_index] = register_age > UINT32_MAX ? UINT32_MAX : (uint32_t)register_age;
            }
        }
    }
    aethor_app_exit_task_critical();
    return true;
}

/** @brief Checks admission twice; completed POS preflight additionally requires fresh safe feedback.
 * The queue TTL applies before Arm start. An in-flight bounded preflight retains ownership,
 * epoch, health and profile checks instead of expiring an already executing request.
 */
#if !AETHOR_ADRC_BENCH
static DebugUiReason aethor_app_debug_ui_validate_start(
    const ProtocolCommand *command, const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us,
    bool preflight_complete)
{
    DebugUiReason reason = DEBUG_UI_REASON_NONE;
    uint8_t joint_index;
    aethor_app_enter_task_critical();
    aethor_app_debug_ui_refresh_reference(timestamp_us);
    if ((AETHOR_DEBUG_UI_ALLOW_MOTION == 0U) ||
        (AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE)) { reason = DEBUG_UI_REASON_DISABLED; }
    else if (command->session_id != application_debug_ui.epoch) { reason = DEBUG_UI_REASON_OLD_EPOCH; }
    else if ((timestamp_us < command->created_at_us) || (!preflight_complete &&
             (timestamp_us - command->created_at_us > DEBUG_UI_REQUEST_MAX_AGE_US))) { reason = DEBUG_UI_REASON_STALE_REQUEST; }
    else if (!debug_ui_mailbox_healthy(&application_debug_ui.mailbox, timestamp_us)) { reason = DEBUG_UI_REASON_UNHEALTHY; }
    else if ((application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_ARMED) &&
             (application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_BUSY)) { reason = DEBUG_UI_REASON_NOT_ARMED; }
    for (joint_index = 0U; (reason == DEBUG_UI_REASON_NONE) && (joint_index < DEBUG_UI_MOTOR_COUNT); ++joint_index)
    {
        DebugUiMotorProfile mode_profile;
        const DebugUiMotorProfile *profile = &mode_profile;
        const MotorJointFeedback *feedback = &snapshot->joints[joint_index];
        const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        bool mit = (command->type == PROTOCOL_COMMAND_MIT_ACTION) ||
            ((command->type == PROTOCOL_COMMAND_SET_MODE) && (command->control_mode == ARM_CONTROL_MODE_MIT));
        bool move = (command->type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
            ((command->type == PROTOCOL_COMMAND_MIT_ACTION) && (command->mit_action == PROTOCOL_MIT_ACTION_MOVE));
        bool defer_feedback = !preflight_complete &&
            ((command->type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
             (command->type == PROTOCOL_COMMAND_MIT_ACTION) || (command->type == PROTOCOL_COMMAND_SET_MODE)) &&
            ((discovery->observed_control_mode == 1U) || (discovery->observed_control_mode == 2U));
        bool check_target = !defer_feedback || ((snapshot->valid_joint_mask & (1U << joint_index)) != 0U);
        float target = command->values[joint_index] + feedback->position_rad;
        mode_profile = debug_ui_profile_bounds(&application_debug_ui.profiles[joint_index], (uint8_t)mit);
        if ((command->motor_mask & (1U << joint_index)) == 0U) { continue; }
        if (((command->type == PROTOCOL_COMMAND_DISABLE) || (command->type == PROTOCOL_COMMAND_CLEAR_FAULT))
                ? ((profile->mit_valid == 0U) && (profile->pos_valid == 0U))
                : ((mit ? profile->mit_valid : profile->pos_valid) == 0U))
        { reason = DEBUG_UI_REASON_UNCALIBRATED; }
        else if (mit && (AETHOR_DEBUG_UI_ALLOW_MIT == 0U)) { reason = DEBUG_UI_REASON_DISABLED; }
        else if (command->reference_generation != application_debug_ui.reference_generation[joint_index]) { reason = DEBUG_UI_REASON_OLD_EPOCH; }
        else if (!defer_feedback && ((snapshot->valid_joint_mask & (1U << joint_index)) == 0U)) { reason = DEBUG_UI_REASON_STALE_FEEDBACK; }
        else if ((discovery->verified_fields_mask & (MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK | MOTOR_DISCOVERY_RANGE_FIELDS_MASK | MOTOR_DISCOVERY_MODE_FIELDS_MASK)) !=
                 (MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK | MOTOR_DISCOVERY_RANGE_FIELDS_MASK | MOTOR_DISCOVERY_MODE_FIELDS_MASK)) { reason = DEBUG_UI_REASON_NOT_READY; }
        else if ((command->type != PROTOCOL_COMMAND_DISABLE) &&
                 (command->type != PROTOCOL_COMMAND_CLEAR_FAULT) &&
                 ((feedback->driver_state != S3519_DRIVER_STATE_DISABLED) ||
                  !isfinite(feedback->velocity_rad_s) ||
                  (fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S) ||
                  (feedback->fault_flags != 0U))) { reason = DEBUG_UI_REASON_NOT_DISABLED; }
        else if (move && ((fabsf(command->values[joint_index]) > profile->max_delta_rad) ||
                 (command->speeds[joint_index] > profile->max_speed_rad_s) ||
                 (check_target && (!isfinite(target) || (target < profile->position_min_rad) ||
                  (target > profile->position_max_rad) || (fabsf(target) > discovery->ranges.position_max_rad))) ||
                 (command->speeds[joint_index] > discovery->maximum_speed_rad_s) ||
                 (command->speeds[joint_index] > discovery->ranges.velocity_max_rad_s))) { reason = DEBUG_UI_REASON_OUT_OF_RANGE; }
        else if ((command->type == PROTOCOL_COMMAND_MIT_ACTION) &&
                 ((command->mit_kp < profile->kp_min) || (command->mit_kp > profile->kp_max) ||
                  (command->mit_kd < profile->kd_min) || (command->mit_kd > profile->kd_max) ||
                  (command->mit_torque_ff_nm != 0.0F) ||
                  (command->hold_duration_us < (uint64_t)profile->hold_min_ms * 1000ULL) ||
                  (command->hold_duration_us > (uint64_t)profile->hold_max_ms * 1000ULL) ||
                  (check_target && (!isfinite(feedback->position_rad) ||
                   (feedback->position_rad < profile->position_min_rad) ||
                   (feedback->position_rad > profile->position_max_rad))))) { reason = DEBUG_UI_REASON_OUT_OF_RANGE; }
    }
    if (reason == DEBUG_UI_REASON_NONE) { reason = protocol_engine_validate_typed_command(command); }
    aethor_app_exit_task_critical();
    return reason;
}
#endif

/** @brief Captures source and fresh disable evidence at result creation, never later. */
static void aethor_app_debug_ui_result_metadata(
    ProtocolCommandResult *result, const ProtocolCommand *command, uint64_t timestamp_us)
{
    MotorFeedbackSnapshot snapshot;
    result->origin = command->origin;
    result->local_operation = command->local_operation;
    aethor_app_enter_task_critical();
    if ((command->origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
        (application_debug_ui.health_stop_started != 0U))
    { result->local_reason = DEBUG_UI_REASON_UNHEALTHY; }
    aethor_app_exit_task_critical();
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot);
    result->disabled_confirmed = aethor_app_selected_motors_have_state(
        &snapshot, command->motor_mask, S3519_DRIVER_STATE_DISABLED);
}

/** @brief Moves local result heads into their already reserved UI records. */
#if !AETHOR_ADRC_BENCH
static void aethor_app_debug_ui_route_results(void)
{
    ProtocolCommandResult result;
    uint8_t result_count;
    aethor_app_enter_task_critical();
    for (result_count = 0U; result_count < PROTOCOL_ENGINE_RESULT_CAPACITY; ++result_count)
    {
        DebugUiCompletion completion;
        if ((protocol_engine_peek_command_result(&application_protocol_engine, &result) == 0U) ||
            (result.origin != DEBUG_UI_ORIGIN_LOCAL_UI)) { break; }
        memset(&completion, 0, sizeof(completion));
        completion.identity.origin = result.origin;
        completion.identity.epoch = result.session_id;
        completion.identity.request_id = result.request_id;
        completion.operation = result.local_operation;
        completion.code = (DebugUiCompletionCode)result.code;
        completion.stage = (DebugUiStage)result.stage;
        completion.reason = result.local_reason;
        completion.timestamp_us = result.completed_at_us;
        completion.detail = result.detail;
        completion.error = (uint8_t)result.error;
        completion.motor_mask = result.motor_mask;
        completion.failed_motor_id = result.failed_motor_number;
        completion.disabled_confirmed = result.disabled_confirmed;
        if (!debug_ui_mailbox_complete(&application_debug_ui.mailbox, &completion)) { break; }
        (void)protocol_engine_pop_local_result(&application_protocol_engine, &result);
        application_debug_ui.last_activity_us = result.completed_at_us;
        if ((result.code == PROTOCOL_COMMAND_RESULT_FAILED) &&
            (application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_FAULT))
        { aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT); }
    }
    aethor_app_exit_task_critical();
}
#endif

#if AETHOR_DEBUG_UI_ALLOW_MOTION
/** @brief Builds a typed command; LCD never serializes text or impersonates a session. */
static void aethor_app_debug_ui_build_command(
    const DebugUiRequest *request, uint64_t timestamp_us, ProtocolCommand *command)
{
    uint8_t joint_index = (uint8_t)(request->target_motor_id - 1U);
    memset(command, 0, sizeof(*command));
    command->origin = request->identity.origin;
    command->session_id = request->identity.epoch;
    command->request_id = request->identity.request_id;
    command->created_at_us = request->created_at_us;
    command->accepted_at_us = timestamp_us;
    command->reference_generation = request->reference_generation;
    command->local_operation = request->operation;
    command->motor_mask = (uint8_t)(1U << joint_index);
    command->bench_relative_scope = 1U;
    command->values_in_radians = 1U;
    command->relative_target = 1U;
    command->values[joint_index] = request->delta_rad * aethor_app_lcd_position_ratio(joint_index);
    command->speeds[joint_index] = request->speed_rad_s;
    command->mit_kp = request->kp / aethor_app_lcd_position_ratio(joint_index);
    command->mit_kd = request->kd;
    command->mit_torque_ff_nm = request->torque_ff_nm;
    command->hold_duration_us = (uint64_t)request->hold_duration_ms * 1000ULL;
    command->control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
    switch (request->operation)
    {
        case DEBUG_UI_OPERATION_POS_MOVE: command->type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED; break;
        case DEBUG_UI_OPERATION_MIT_HOLD:
        case DEBUG_UI_OPERATION_MIT_MOVE:
            command->type = PROTOCOL_COMMAND_MIT_ACTION;
            command->control_mode = ARM_CONTROL_MODE_MIT;
            command->mit_action = request->operation == DEBUG_UI_OPERATION_MIT_HOLD
                ? PROTOCOL_MIT_ACTION_HOLD : PROTOCOL_MIT_ACTION_MOVE;
            break;
        case DEBUG_UI_OPERATION_SET_MODE:
            command->type = PROTOCOL_COMMAND_SET_MODE;
            command->control_mode = request->requested_mode == DEBUG_UI_MODE_MIT
                ? ARM_CONTROL_MODE_MIT : ARM_CONTROL_MODE_POSITION_VELOCITY;
            break;
        case DEBUG_UI_OPERATION_CLEAR_FAULT:
        case DEBUG_UI_OPERATION_DISABLE:
            command->type = request->operation == DEBUG_UI_OPERATION_DISABLE
                ? PROTOCOL_COMMAND_DISABLE : PROTOCOL_COMMAND_CLEAR_FAULT;
            command->control_mode = application_motor_runtime.discovery.results[joint_index].observed_control_mode == 1U
                ? ARM_CONTROL_MODE_MIT : ARM_CONTROL_MODE_POSITION_VELOCITY;
            break;
        case DEBUG_UI_OPERATION_STOP:
            command->type = PROTOCOL_COMMAND_STOP;
            command->cleanup_after_stop = 1U;
            if (application_motor_runtime.discovery.results[joint_index].observed_control_mode == 1U)
            {
                command->control_mode = ARM_CONTROL_MODE_MIT;
                command->mit_kp = application_debug_ui.profiles[joint_index].default_kp;
                command->mit_kd = application_debug_ui.profiles[joint_index].default_kd;
            }
            break;
        default: break;
    }
}
#endif

/** @brief Validates and grants/revokes authority only after all prior cleanup/results finish. */
#if !AETHOR_ADRC_BENCH
static DebugUiReason aethor_app_debug_ui_admit_request(
    const DebugUiRequest *request, uint64_t timestamp_us, ProtocolCommand *command)
{
#if !AETHOR_DEBUG_UI_ALLOW_MOTION
    (void)request;
    (void)timestamp_us;
    (void)command;
    return DEBUG_UI_REASON_DISABLED;
#else
    MotorFeedbackSnapshot snapshot;
    uint8_t joint_index = (uint8_t)(request->target_motor_id - 1U);
    const DebugUiMotorProfile *profile = &application_debug_ui.profiles[joint_index];
    DebugUiReason reason;
    aethor_app_debug_ui_build_command(request, timestamp_us, command);
    if (request->operation == DEBUG_UI_OPERATION_STOP)
    { return protocol_engine_submit_local_command(&application_protocol_engine, command); }
    if (!isfinite(request->delta_rad) || !isfinite(request->speed_rad_s) ||
        !isfinite(request->kp) || !isfinite(request->kd) || !isfinite(request->torque_ff_nm))
    { return DEBUG_UI_REASON_INVALID_ARGUMENT; }
    if (request->identity.epoch != application_debug_ui.epoch) { return DEBUG_UI_REASON_OLD_EPOCH; }
    if ((timestamp_us < request->created_at_us) ||
        (timestamp_us - request->created_at_us > DEBUG_UI_REQUEST_MAX_AGE_US)) { return DEBUG_UI_REASON_STALE_REQUEST; }
    if ((profile->pos_valid == 0U) && (profile->mit_valid == 0U)) { return DEBUG_UI_REASON_UNCALIBRATED; }
    if (!debug_ui_mailbox_healthy(&application_debug_ui.mailbox, timestamp_us)) { return DEBUG_UI_REASON_UNHEALTHY; }
    if (aethor_app_debug_ui_executor_busy()) { return DEBUG_UI_REASON_BUSY; }
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot);
    if ((request->operation == DEBUG_UI_OPERATION_ACQUIRE) || (request->operation == DEBUG_UI_OPERATION_RELEASE))
    {
        const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        /* Ownership does not actuate. A recovered idle fault can reuse confirmed disabled state;
         * unresolved cleanup still requires new post-attempt feedback for every original target. */
        bool defer_feedback = (application_debug_ui.unconfirmed_disable_mask == 0U) &&
            (((profile->pos_valid != 0U) && (discovery->observed_control_mode == 2U)) ||
             ((profile->mit_valid != 0U) && AETHOR_DEBUG_UI_ALLOW_MIT &&
              (discovery->observed_control_mode == 1U)));
        reason = aethor_app_debug_ui_recovery_reason(&snapshot);
        if (reason != DEBUG_UI_REASON_NONE) { return reason; }
        if ((request->operation == DEBUG_UI_OPERATION_RELEASE) &&
            (application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE) &&
            (application_debug_ui.target_motor_id != 0U) &&
            (request->target_motor_id != application_debug_ui.target_motor_id))
        { return DEBUG_UI_REASON_NOT_ARMED; }
        if (!defer_feedback && ((snapshot.valid_joint_mask & command->motor_mask) != command->motor_mask))
        { return DEBUG_UI_REASON_STALE_FEEDBACK; }
        if (defer_feedback && ((discovery->verified_fields_mask &
            (MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK | MOTOR_DISCOVERY_RANGE_FIELDS_MASK | MOTOR_DISCOVERY_MODE_FIELDS_MASK)) !=
            (MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK | MOTOR_DISCOVERY_RANGE_FIELDS_MASK | MOTOR_DISCOVERY_MODE_FIELDS_MASK)))
        { return DEBUG_UI_REASON_NOT_READY; }
        if ((snapshot.joints[joint_index].driver_state != S3519_DRIVER_STATE_DISABLED) ||
            !isfinite(snapshot.joints[joint_index].velocity_rad_s) ||
            (fabsf(snapshot.joints[joint_index].velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S) ||
            (snapshot.joints[joint_index].fault_flags != 0U)) { return DEBUG_UI_REASON_NOT_DISABLED; }
        if (debug_ui_mailbox_result_count(&application_debug_ui.mailbox) != 1U) { return DEBUG_UI_REASON_RESULT_BACKPRESSURE; }
        aethor_app_debug_ui_revoke(request->operation == DEBUG_UI_OPERATION_ACQUIRE
            ? DEBUG_UI_AUTHORITY_LOCAL_ARMED : DEBUG_UI_AUTHORITY_REMOTE);
        application_debug_ui.target_motor_id = request->target_motor_id;
        application_debug_ui.last_activity_us = timestamp_us;
        application_debug_ui.health_stop_started = 0U;
        application_debug_ui.unconfirmed_disable_mask = 0U;
        return DEBUG_UI_REASON_NONE;
    }
    if ((application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_ARMED) ||
        (request->target_motor_id != application_debug_ui.target_motor_id)) { return DEBUG_UI_REASON_NOT_ARMED; }
    if ((request->operation == DEBUG_UI_OPERATION_SET_MODE) &&
        (request->requested_mode != DEBUG_UI_MODE_MIT) &&
        (request->requested_mode != DEBUG_UI_MODE_POS_VEL)) { return DEBUG_UI_REASON_INVALID_ARGUMENT; }
    reason = aethor_app_debug_ui_validate_start(command, &snapshot, timestamp_us, false);
    if (reason != DEBUG_UI_REASON_NONE) { return reason; }
    reason = protocol_engine_submit_local_command(&application_protocol_engine, command);
    if (reason == DEBUG_UI_REASON_NONE)
    {
        application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_BUSY;
        application_debug_ui.last_activity_us = timestamp_us;
    }
    return reason;
#endif
}
#endif

/** @brief Runs at most one STOP and one ordinary admission in the sole ProtocolTask. */
void aethor_app_debug_ui_process(uint64_t timestamp_us)
{
#if !AETHOR_ADRC_BENCH
    uint8_t pass;
#endif
    if (application_initialized == 0U) { return; }
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD) { return; }
#endif
#if AETHOR_ADRC_BENCH
    (void)timestamp_us;
    return;
#else
    aethor_app_debug_ui_route_results();
    aethor_app_enter_task_critical();
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
    { aethor_app_exit_task_critical(); return; }
#endif
    aethor_app_debug_ui_refresh_reference(timestamp_us);
    debug_ui_mailbox_service_health(&application_debug_ui.mailbox, timestamp_us);
    if ((application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_BUSY) &&
        !aethor_app_debug_ui_executor_busy() &&
        (debug_ui_mailbox_result_count(&application_debug_ui.mailbox) == 0U))
    { application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; }
    for (pass = 0U; pass < 2U; ++pass)
    {
        DebugUiRequest request;
        DebugUiAdmission admission;
        ProtocolCommand command;
        if ((pass == 0U) &&
            (application_protocol_engine.stop_write_sequence != application_protocol_engine.stop_read_sequence))
        { continue; }
        if (!debug_ui_mailbox_take(&application_debug_ui.mailbox, pass == 0U, &request)) { continue; }
        memset(&admission, 0, sizeof(admission));
        admission.identity = request.identity;
        admission.operation = request.operation;
        admission.timestamp_us = timestamp_us;
        admission.reason = aethor_app_debug_ui_admit_request(&request, timestamp_us, &command);
        admission.accepted = (uint8_t)(admission.reason == DEBUG_UI_REASON_NONE);
        (void)debug_ui_mailbox_admit(&application_debug_ui.mailbox, &admission);
        if ((admission.accepted == 0U) || (request.operation == DEBUG_UI_OPERATION_ACQUIRE) ||
            (request.operation == DEBUG_UI_OPERATION_RELEASE))
        {
            DebugUiCompletion completion;
            memset(&completion, 0, sizeof(completion));
            completion.identity = request.identity;
            completion.operation = request.operation;
            completion.reason = admission.reason;
            completion.timestamp_us = timestamp_us;
            completion.code = admission.accepted != 0U ? DEBUG_UI_COMPLETED : DEBUG_UI_FAILED;
            completion.stage = DEBUG_UI_STAGE_VALIDATE;
            completion.disabled_confirmed = admission.accepted;
            (void)debug_ui_mailbox_complete(&application_debug_ui.mailbox, &completion);
            if (admission.accepted == 0U) { ++application_debug_ui.diagnostics.rejected_request_count; }
        }
    }
    aethor_app_exit_task_critical();
#endif
}

/** @brief ArmControlTask revokes unhealthy authority; healthy ownership lasts until manual release. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_debug_ui_service_health(uint64_t timestamp_us)
{
    bool unhealthy = false;
    aethor_app_enter_task_critical();
#if AETHOR_DEBUG_UI_ALLOW_MOTION
    if ((application_debug_ui.unconfirmed_disable_mask != 0U) &&
        !aethor_app_debug_ui_executor_busy())
    {
        MotorFeedbackSnapshot recovery_snapshot;
        memset(&recovery_snapshot, 0, sizeof(recovery_snapshot));
        (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &recovery_snapshot);
        /* Retain a completed acknowledgement across the user's later 800 ms review.
         * Clearing this evidence debt never grants ownership or enables a motor. */
        if (aethor_app_debug_ui_recovery_reason(&recovery_snapshot) == DEBUG_UI_REASON_NONE)
        { application_debug_ui.unconfirmed_disable_mask = 0U; }
    }
#endif
    if ((application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE) &&
        (application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_FAULT) &&
        !debug_ui_mailbox_healthy(&application_debug_ui.mailbox, timestamp_us))
    {
        /* Capture the decision inputs under the same critical section as the guard. */
        application_debug_ui.health_failure_snapshot = application_debug_ui.mailbox.health;
        application_debug_ui.health_failure_timestamp_us = timestamp_us;
        application_debug_ui.health_failure_authority = application_debug_ui.authority;
        application_debug_ui.health_failure_action_state = (uint32_t)application_action.state;
        aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT);
        application_debug_ui.health_stop_started = 1U;
        unhealthy = true;
        ++application_debug_ui.diagnostics.health_stop_count;
    }
    aethor_app_exit_task_critical();
    if (unhealthy && (application_action.state != AETHOR_APP_ACTION_IDLE) &&
        (application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
        (application_action.command.type != PROTOCOL_COMMAND_STOP))
    {
        ProtocolCommandStage stage = aethor_app_current_one_shot_stage();
        if (stage == PROTOCOL_COMMAND_STAGE_NONE) { stage = PROTOCOL_COMMAND_STAGE_CLEAR; }
        return aethor_app_begin_one_shot_cleanup(stage, PROTOCOL_COMMAND_ERROR_TIMEOUT,
            application_debug_ui.target_motor_id, timestamp_us);
    }
    return 0U;
}
#endif

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
#if AETHOR_ADRC_BENCH
    {
        uint8_t available;
        aethor_app_enter_task_critical();
        available = adrc_protocol_pop_result_output(&application_adrc_bridge.bench.gateway,
            &application_protocol_engine, output_batch);
        aethor_app_exit_task_critical();
        return available;
    }
#else
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
    {
        uint8_t available;
        aethor_app_enter_task_critical();
        available = adrc_protocol_pop_result_output(&application_adrc_bridge.bench.gateway,
            &application_protocol_engine, output_batch);
        aethor_app_exit_task_critical();
        return available;
    }
#endif
    aethor_app_debug_ui_route_results();
    return protocol_engine_pop_result_output(&application_protocol_engine,
                                             output_batch);
#endif
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
#if AETHOR_ADRC_BENCH
    (void)timestamp_us;
    memset(output_batch, 0, sizeof(*output_batch));
    return 0U;
#else
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
    { memset(output_batch, 0, sizeof(*output_batch)); return 0U; }
#endif
    aethor_app_update_protocol_context(timestamp_us);
    return protocol_engine_generate_stream_output(&application_protocol_engine,
                                                  timestamp_us,
                                                  output_batch);
#endif
}

/**
 * @brief Formats a high-priority error for one discarded overlong USB line.
 */
ProtocolEngineStatus aethor_app_format_line_too_long(
    ProtocolOutputBatch *output_batch)
{
    return protocol_engine_format_text_line_too_long(output_batch);
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
    bool copied;
    if (application_initialized == 0U)
    {
        return false;
    }

    aethor_app_enter_task_critical();
    copied = diagnostics_get_counters(&application_diagnostics, counters);
    aethor_app_exit_task_critical();
    return copied;
}

/**
 * @brief Applies one platform-neutral transport/resource diagnostic sample.
 */
void aethor_app_update_runtime_diagnostics(
    const RuntimeDiagnosticSample *sample)
{
    if ((application_initialized != 0U) && (sample != NULL))
    {
        aethor_app_enter_task_critical();
        diagnostics_update_runtime_sample(&application_diagnostics, sample);
        aethor_app_exit_task_critical();
    }
}

/**
 * @brief Publishes a severe transport fault for ArmControlTask ownership.
 */
uint8_t aethor_app_report_transport_fault(uint32_t detail,
                                          uint64_t timestamp_us)
{
#if !AETHOR_ADRC_BENCH
    uint8_t used_count;
    uint8_t slot_index;
#endif

    if ((application_initialized == 0U) || (detail == 0U))
    {
        return 0U;
    }
#if AETHOR_ADRC_LCD_INTEGRATED
    aethor_app_enter_task_critical();
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ADRC ||
        application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING)
    {
        adrc_app_bridge_report_fault(&application_adrc_bridge);
        aethor_app_exit_task_critical();
        return 1U;
    }
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING)
    { application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_LCD; }
    aethor_app_exit_task_critical();
#endif
#if AETHOR_ADRC_BENCH
    (void)timestamp_us;
    aethor_app_enter_task_critical();
    adrc_app_bridge_report_fault(&application_adrc_bridge);
    aethor_app_exit_task_critical();
    return 1U;
#else
    used_count = (uint8_t)(application_transport_fault_write_sequence -
                           application_transport_fault_read_sequence);
    if (used_count >= AETHOR_APP_TRANSPORT_FAULT_CAPACITY)
    {
        return 1U;
    }
    slot_index = (uint8_t)(application_transport_fault_write_sequence %
                           AETHOR_APP_TRANSPORT_FAULT_CAPACITY);
    application_transport_faults[slot_index].detail = detail;
    application_transport_faults[slot_index].timestamp_us = timestamp_us;
    aethor_app_compiler_barrier();
    ++application_transport_fault_write_sequence;
    return 1U;
#endif
}

/**
 * @brief Latches one consumed transport fault and schedules all-axis disable.
 */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_apply_transport_fault(uint32_t detail,
                                                uint64_t timestamp_us)
{
    uint8_t result_generated = 0U;

    aethor_app_widen_pending_stop_before_global_cancel();
    if (application_action.state != AETHOR_APP_ACTION_IDLE)
    {
        (void)motor_runtime_abort_active_parameter_sequences(
            &application_motor_runtime,
            timestamp_us);
        if ((application_action.command.type ==
             PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
            (application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION))
        {
            aethor_app_record_one_shot_failure_once(
                aethor_app_current_one_shot_stage(),
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED,
                0U);
        }
        result_generated = aethor_app_complete_action(
            PROTOCOL_COMMAND_RESULT_FAILED,
            (uint16_t)ARM_FAULT_TRANSPORT,
            timestamp_us);
    }
    result_generated |= aethor_app_fail_queued_motion(
        (uint16_t)ARM_FAULT_TRANSPORT,
        timestamp_us);
    (void)arm_controller_latch_runtime_fault(&application_controller,
                                             ARM_FAULT_TRANSPORT,
                                             detail,
                                             timestamp_us);
    protocol_engine_cancel_pending_normal_commands(
        &application_protocol_engine);
#if !AETHOR_ADRC_BENCH
    application_emergency_disable_read_index = 0U;
#endif
    (void)motor_runtime_build_emergency_disable(
        &application_motor_runtime,
        &application_emergency_disable_batch);
    return result_generated;
}
#endif

#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
/** @brief Delivers the sole selected-axis output under the application task critical hooks. */
uint8_t aethor_app_adrc_pop_frame(CanFrame *frame, uint8_t *kind, float *decoded_torque_nm)
{
    uint8_t available;
    if (!application_initialized) { return 0U; }
    aethor_app_enter_task_critical();
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_ADRC &&
        application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_RELEASING)
    { aethor_app_exit_task_critical(); return 0U; }
#endif
    available = adrc_app_bridge_pop_frame(&application_adrc_bridge, frame, kind, decoded_torque_nm);
    aethor_app_exit_task_critical();
    return available;
}

/** @brief Matches a hardware TX receipt to the selected-axis owner without claiming motor execution. */
void aethor_app_adrc_report_transmit(uint8_t kind, float decoded_torque_nm, uint8_t succeeded)
{
    if (!application_initialized) { return; }
    aethor_app_enter_task_critical();
    adrc_app_bridge_report_transmit(&application_adrc_bridge, kind, decoded_torque_nm, succeeded);
    aethor_app_exit_task_critical();
}

#if AETHOR_ADRC_LCD_INTEGRATED
/** @brief Couples a real selected-axis disable receipt to the LCD handback evidence gate. */
void aethor_app_adrc_report_transmit_at(uint8_t kind, float decoded_torque_nm,
    uint8_t succeeded, uint64_t timestamp_us)
{
    uint8_t matched_disable;
    if (!application_initialized) { return; }
    aethor_app_enter_task_critical();
    matched_disable = (uint8_t)(succeeded != 0U && kind == 2U &&
        application_adrc_bridge.awaiting_receipt != 0U &&
        application_adrc_bridge.frame_kind == kind &&
        application_adrc_bridge.decoded_torque_nm == decoded_torque_nm);
    adrc_app_bridge_report_transmit(&application_adrc_bridge, kind, decoded_torque_nm, succeeded);
    if (matched_disable != 0U)
    { adrc_lcd_ownership_report_disable_transmit(&application_adrc_lcd_ownership,
        timestamp_us, 1U); }
    aethor_app_exit_task_critical();
}
#endif

/** @brief Installs locally measured commissioning evidence, retaining the supervisor's provenance checks. */
AdrcExperimentResult aethor_app_adrc_set_evidence(const AdrcExperimentConfig *config,
    const AdrcExperimentQualification *qualification, uint64_t timestamp_us)
{
    AdrcExperimentResult result;
    if (!application_initialized) { return ADRC_RESULT_INVALID_STATE; }
    aethor_app_enter_task_critical();
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_ADRC)
    { aethor_app_exit_task_critical(); return ADRC_RESULT_INVALID_STATE; }
#endif
    result = adrc_app_bridge_set_evidence(&application_adrc_bridge, config, qualification, timestamp_us);
    aethor_app_exit_task_critical();
    return result;
}

/** @brief Copies the supervisor state coherently without exposing mutable bridge storage. */
uint8_t aethor_app_adrc_get_status(AdrcExperimentStatus *status)
{
    AdrcExperimentResult result;
    if (!application_initialized || status == NULL) { return 0U; }
    aethor_app_enter_task_critical();
    result = adrc_experiment_get_status(&application_adrc_bridge.bench.experiment, status);
    aethor_app_exit_task_critical();
    return (uint8_t)(result == ADRC_RESULT_OK);
}
#endif
