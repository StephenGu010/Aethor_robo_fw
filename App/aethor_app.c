/**
 * @file aethor_app.c
 * @brief Implements the allocation-free Phase 0 application facade.
 */

#include "aethor_app.h"
#include "motor_adrc_command.h"
#include "Motor/motor_idle_feedback.h"
#include "Motor/motor_standby_recovery.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "arm_config.h"
#include "joint_motion_can.h"
#include "DebugUi/debug_ui_mailbox.h"
#include "DebugUi/debug_ui_single_static_bounds.h"
#include "DebugUi/debug_ui_torque_envelope.h"
#include "Config/motor_compatibility.h"
#include "Motor/motor_persistence_app_port.h"
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
/** @brief Private read-only maintenance entry; intentionally absent from every production command parser. */
static uint8_t aethor_app_persistence_begin_readonly(uint32_t request_id, uint64_t now_us);
/** @brief Boot-only initialization and periodic irrevocable owner checks. */
static void aethor_app_persistence_init(void);
static void aethor_app_persistence_service(uint64_t now_us);
/** @brief Preserve existing STOP execution while revoking the separate maintenance permission. */
static void aethor_app_persistence_preempt(uint64_t now_us);
static void aethor_app_persistence_disable_intent(uint8_t motor_mask);
#endif
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
/** @brief Reads the actual integrated motor-7 owner under the bridge caller's task critical boundary. */
static uint8_t aethor_app_capability_owner_active(void)
{
    return (uint8_t)(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ADRC &&
        application_adrc_lcd_ownership.axis_index == 6U);
}
static uint8_t application_integrated_can_idle;
/** @brief One boot-local, LCD-owned raw pair; never grants model or motion qualification. */
typedef struct
{
    uint64_t started_us;
    uint64_t native_feedback_us;
    uint32_t request_id;
    uint32_t initial_timeouts;
    uint32_t initial_rejections;
    float native_position_rad;
    float native_quantum_rad;
    uint8_t state; /* 0 unused, 1 collecting, 2 complete, 3 cancelled/failed; never rearmed by USB. */
    uint8_t reason; /* 1 STOP, 2 timeout, 3 activity/binding, 4 invalid/rejected observation. */
} AethorCoordinatePair;
static AethorCoordinatePair application_coordinate_pair;
/** @brief Fixed volatile PMAX/VMAX maintenance states; failures retain a motion lock. */
typedef enum
{
    AETHOR_MAPPING_IDLE = 0, AETHOR_MAPPING_QUIET, AETHOR_MAPPING_SOURCE_WAIT,
    AETHOR_MAPPING_WRITE, AETHOR_MAPPING_SETTLE, AETHOR_MAPPING_TARGET_WAIT,
    AETHOR_MAPPING_DRAIN, AETHOR_MAPPING_DISCOVERY, AETHOR_MAPPING_DISABLE,
    AETHOR_MAPPING_BASELINE, AETHOR_MAPPING_DONE, AETHOR_MAPPING_FAILED
} AethorMappingState;
/** @brief Separates fixed range pairs sharing a register so failed transactions cannot cross-restore. */
typedef enum
{
    AETHOR_MAPPING_FAMILY_VELOCITY = 0,
    AETHOR_MAPPING_FAMILY_WIDE_POSITION,
    AETHOR_MAPPING_FAMILY_RATED70_POSITION
} AethorMappingFamily;
/** @brief Owns one serialized fixed-range transaction, never physical TX or model qualification. */
typedef struct
{
    uint64_t started_us;
    uint64_t phase_us;
    uint64_t disabled_observed_us;
    uint32_t request_id;
    float target_range;
    float source_range;
    AethorMappingState state;
    AethorMappingFamily family;
    uint8_t reason; /* 1 STOP, 2 timeout, 3 competing activity, 4 reply, 5 rediscovery, 6 baseline. */
    uint8_t writes;
    uint8_t recovery;
    uint8_t disable_pending;
    uint8_t register_address; /* Fixed 0x15 PMAX or 0x16 VMAX; recovery also matches the fixed family. */
} AethorLowSpeedMapping;
static AethorLowSpeedMapping application_low_speed_mapping;
/** @brief Real FIFO-empty/TX-drained observation from the platform; never a wire timestamp. */
static uint64_t application_mapping_rx_boundary_us;
static uint64_t application_integrated_can_quiet_since_us;
/** @brief One dedicated-channel DISABLE must transmit before newer disabled feedback can count. */
static uint64_t application_integrated_probe_submitted_us;
static uint8_t application_integrated_probe_transmitted;
static uint8_t application_integrated_probe_failed;
/** @brief A zero-request MIT control frame may follow only a confirmed acquisition DISABLE transmit. */
static uint8_t application_integrated_acquire_disable_transmitted;
static uint8_t application_integrated_acquire_neutral_submitted;
/** @brief At most three extra DISABLE frames, no faster than one control period. */
static uint8_t application_integrated_acquire_disable_retries;
#define AETHOR_ADRC_ACQUIRE_DISABLE_RETRY_LIMIT (3U)
#define AETHOR_ADRC_ACQUIRE_DISABLE_RETRY_US (4000ULL)
/** @brief Up to three supplemental feedback requests; existing failed-stop retries remain active. */
static uint64_t application_integrated_release_last_submit_us;
static uint8_t application_integrated_release_disable_retries;
#define AETHOR_ADRC_RELEASE_DISABLE_RETRY_LIMIT (3U)
#define AETHOR_ADRC_RELEASE_DISABLE_RETRY_US (4000ULL)
/** @brief Raw receive evidence retained after one bounded acquisition challenge. */
static uint32_t application_integrated_acquire_rx_count;
/** @brief First challenge timestamp is never overwritten by the neutral fallback. */
static uint64_t application_integrated_acquire_first_submit_us;
/** @brief Total includes all observed IDs; valid/rejected are selected-target decode outcomes. */
static uint32_t application_integrated_acquire_other_count;
static uint32_t application_integrated_acquire_unknown_time_count;
static uint32_t application_integrated_acquire_old_count;
/** @brief A phase counts time-attributed frames, never claims causality to a command. */
typedef struct
{
    uint64_t submitted_us;
    uint32_t raw_count;
    uint32_t target_valid_count;
    uint32_t other_count;
} AethorAcquireRxPhase;
static AethorAcquireRxPhase application_integrated_acquire_phases[2];
static uint32_t application_integrated_acquire_unassigned_count;
static uint32_t application_integrated_acquire_valid_count;
static uint32_t application_integrated_acquire_rejected_count;
static uint16_t application_integrated_acquire_last_rx_id;
static uint8_t application_integrated_acquire_last_rx_length;
/** @brief A separate LCD-owned diagnostic probe never participates in an ADRC handoff. */
static uint64_t application_integrated_diag_requested_us;
static uint64_t application_integrated_diag_transmitted_us;
static uint8_t application_integrated_diag_query_length;
static uint32_t application_integrated_diag_rx_after_tx;
static uint32_t application_integrated_diag_rx_valid;
static uint32_t application_integrated_diag_rx_rejected;
static uint16_t application_integrated_diag_last_rx_id;
static uint8_t application_integrated_diag_last_rx_length;
static uint8_t application_integrated_diag_last_rx_data0;
static uint8_t application_integrated_diag_last_rx_data2;
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
    DebugUiStaticTxIdentity static_identity;
    uint64_t static_takeover_started_us; /**< Fixed one-second window; never extended by retry or RX. */
    uint64_t static_takeover_last_poll_us;
    uint64_t static_takeover_last_refresh_us; /**< Last prepared refresh; no catch-up queue or deadline extension. */
    uint64_t static_takeover_proven_us; /**< First final-step ordered RX before the fixed deadline. */
    uint8_t static_takeover_step; /**< Last scheduled 50 ms increment, 0..19. */
    CanFrame static_frozen_frames[ARM_JOINT_COUNT];
    uint64_t static_setup_deadline_us;
    uint64_t static_submit_us;
    uint8_t static_preloaded_mask;
    uint8_t static_first_hold_mask;
    uint32_t single_static_rx_generation;
    uint32_t single_static_ordered_generation; /**< Only an accepted sample with hardware RX SOF after this TX event may qualify. */
    uint8_t single_static_tx_submitted;
    uint8_t single_static_tx_confirmed;
    uint8_t single_static_tx_failed;
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
#if AETHOR_IDLE_FEEDBACK_ENABLE
/** @brief Independent idle monitoring state; it owns no motor command or motion permission. */
static MotorIdleFeedback application_idle_feedback;
/** @brief A background-only ID7 identity recovery lease, never an actuation owner. */
static MotorStandbyRecovery application_standby_recovery;
#endif
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
    DebugUiSingleStaticProfile single_static_profiles[DEBUG_UI_MOTOR_COUNT];
    DebugUiStaticGains single_static_gains[DEBUG_UI_MOTOR_COUNT]; /**< Reference-bound output-shaft ID7 gains. */
    uint32_t single_static_reference[DEBUG_UI_MOTOR_COUNT];
    MotorDiscoveryResult single_static_discovery[DEBUG_UI_MOTOR_COUNT];
    uint32_t single_static_evidence_generation[DEBUG_UI_MOTOR_COUNT];
    uint32_t single_static_observed_reference[DEBUG_UI_MOTOR_COUNT];
    MotorDiscoveryResult single_static_observed_discovery[DEBUG_UI_MOTOR_COUNT];
    DebugUiMotorProfile single_static_observed_profile[DEBUG_UI_MOTOR_COUNT];
    uint8_t single_static_lcd_owned[DEBUG_UI_MOTOR_COUNT];
    DebugUiIdentity single_static_transport_identity;
    DebugUiStaticTxIdentity static_transport_identity;
    CanFrame static_transport_frame;
    uint64_t static_transport_submit_us;
    uint8_t single_static_transport_mask; /**< Queued/pending HAL TX survives action timeout and blocks future control. */
    DebugUiDiagnostics diagnostics;
    DebugUiStaticLoadReport static_load;
    DebugUiPosLoadReport pos_load;
    uint64_t pos_load_sample_us;
    uint64_t pos_load_observation_us;
    uint64_t pos_load_hold_started_us;
    uint64_t pos_load_recovery_started_us;
    uint64_t pos_load_stable_started_us;
    MotorDiscoveryResult pos_load_discovery;
    DebugUiMotorProfile pos_load_profile;
    float pos_load_start_position_rad;
    uint64_t pos_load_fallback_after_us;
    uint64_t pos_load_disable_after_us;
    DebugUiStaticTxIdentity pos_disable_identity;
    CanFrame pos_disable_frame;
    uint32_t pos_disable_next_sequence;
    uint32_t pos_disable_rx_generation;
    uint32_t pos_disable_ordered_generation;
    uint8_t pos_disable_submitted;
    uint8_t pos_disable_confirmed;
    uint8_t pos_disable_failed;
    uint8_t pos_disable_pending;
    uint8_t pos_disable_attempt_count; /**< Finite terminal DISABLE probes; never extends the action deadline. */
    uint8_t pos_disable_restart_requested; /**< New explicit STOP must not reuse pending original TX evidence. */
    uint8_t pos_load_cleanup_required;
    uint64_t static_sample_us[DEBUG_UI_MOTOR_COUNT];
    uint64_t static_last_batch_us;
    uint64_t static_fallback_after_us;
    DebugUiIdentity static_fallback_identity;
    uint8_t static_fallback_bound;
    DebugUiAuthority authority;
    uint64_t last_activity_us;
    uint32_t epoch;
    uint32_t generation;
    uint32_t reference_generation[DEBUG_UI_MOTOR_COUNT];
    uint32_t observed_mode[DEBUG_UI_MOTOR_COUNT];
    uint32_t observed_coordinate_identity[DEBUG_UI_MOTOR_COUNT]; /**< Exact discovered ESC/master pair binds coordinate continuity. */
    /** @brief Raw ID7 POS encoding range also invalidates reviews when ordinary bounds stay clamped. */
    uint32_t pos_load_observed_pmax_bits;
    uint8_t target_motor_id;
    uint8_t health_stop_started;
    /** @brief Latched pre-revocation health evidence; subsequent good heartbeats cannot overwrite it. */
    DebugUiHealth health_failure_snapshot;
    uint64_t health_failure_timestamp_us;
    DebugUiAuthority health_failure_authority;
    uint32_t health_failure_action_state;
    /** @brief Health attribution belongs only to the exact action first interrupted by this failure. */
    DebugUiIdentity health_failure_identity;
    DebugUiOperation health_failure_operation;
    ProtocolCommandType health_failure_command_type;
    uint8_t health_failure_motor_mask;
    uint8_t health_failure_action_valid;
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
/** @brief Experimental ID7 exposure credit is independent of UI ownership and transaction reports. */
static TorqueEnvelope application_torque_envelope;

/** @brief Validates local authorization again at the ArmControlTask boundary. */
#if !AETHOR_ADRC_BENCH
static DebugUiReason aethor_app_debug_ui_validate_start(
    const ProtocolCommand *command, const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us,
    bool preflight_complete);
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION
/** @brief Strict selected-only local preflight; defined beside frame-free configuration. */
static DebugUiReason aethor_app_single_static_preflight_validate(const ProtocolCommand *command,
    const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us, bool require_feedback);
#endif
#endif
/** @brief Routes only LOCAL_UI result heads without formatting USB text. */
#if !AETHOR_ADRC_BENCH
static void aethor_app_debug_ui_route_results(void);
#endif
/** @brief ArmControlTask independently detects local UI or Protocol service failure. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_debug_ui_service_health(uint64_t timestamp_us);
#endif
#if !AETHOR_ADRC_BENCH
/** @brief Passive trace hooks preserve first failure and cleanup without authorizing control. */
static void aethor_app_static_trace_begin(uint64_t timestamp_us);
static void aethor_app_static_trace_record(uint8_t event, uint8_t motor_id, const CanFrame *frame,
    uint64_t lower_us, uint64_t observed_us, uint16_t sof, uint8_t sof_valid, uint32_t fault);
static void aethor_app_static_trace_terminal(const ProtocolCommandResult *result);
static void aethor_app_static_trace_emergency_begin(uint64_t timestamp_us);
static void aethor_app_static_trace_emergency_progress(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us);
/** @brief Latch the original static trial's first exit before cleanup changes its takeover/hold phase. */
static void aethor_app_static_load_latch_exit(DebugUiReason reason, DebugUiStage stage,
    uint8_t error, DebugUiCompletionCode code);
#endif
/** @brief Copies local ownership under critical before the USB watchdog decision. */
#if !AETHOR_ADRC_BENCH
static bool aethor_app_debug_ui_owns_link_lifecycle(void);
#endif
/** @brief Checks command ownership before optional idle register monitoring. */
static bool aethor_app_debug_ui_executor_busy(void);
static uint8_t (*application_remote_release_can_idle)(void);
static uint32_t (*application_remote_release_link_word)(void);
#if !AETHOR_ADRC_BENCH
static void aethor_app_torque_envelope_finish(const ProtocolCommandResult *result);
static void aethor_app_torque_envelope_fault(void);
#endif
/** @brief Advances the local epoch and authority under the caller's critical boundary. */
static void aethor_app_debug_ui_revoke(DebugUiAuthority authority);
#if !AETHOR_ADRC_BENCH
/** @brief Reset the USB UI proxy at the same boundary as the local mailbox. */
static void aethor_app_remote_reset(void);
#endif
/** @brief Retains required post-cleanup feedback for every original target. Caller holds critical. */
static void aethor_app_debug_ui_lock_targets(uint8_t motor_mask, uint64_t timestamp_us);
/** @brief Arm independently consumes saturated STOP intents without inventing result identities. */
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_debug_ui_service_stop_intent(uint64_t timestamp_us, uint8_t *result_generated);
#endif
/** @brief Copies immutable command source and physical disable evidence to its terminal. */
static void aethor_app_debug_ui_result_metadata(
    ProtocolCommandResult *result, const ProtocolCommand *command, uint64_t timestamp_us);
/** @brief Matches latched health causality under the caller's task critical boundary. */
static uint8_t aethor_app_debug_ui_health_failure_matches(const ProtocolCommand *command);

static void aethor_app_update_protocol_context(uint64_t timestamp_us);
#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_start_one_shot_hold(uint64_t timestamp_us);
static uint8_t aethor_app_start_one_shot_motion(const MotorFeedbackSnapshot *motor_snapshot, uint64_t timestamp_us);
static void aethor_app_pos_load_freeze_elapsed(uint64_t timestamp_us);
/** @brief Publish the same coordinate generation for passive telemetry and HOME requests. */
static void aethor_app_debug_ui_refresh_reference(uint64_t timestamp_us);
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
        (application_action.command.type != PROTOCOL_COMMAND_HOME_PREPARE) &&
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

#if !AETHOR_ADRC_BENCH
    if (debug_ui_is_pos_load_operation(application_action.command.local_operation) &&
        code != PROTOCOL_COMMAND_RESULT_COMPLETED) aethor_app_pos_load_freeze_elapsed(timestamp_us);
#endif
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
    if (application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT)
    { result.stage = PROTOCOL_COMMAND_STAGE_DISABLE; }
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
#if !AETHOR_ADRC_BENCH
    aethor_app_static_trace_terminal(&result);
#endif
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
#if !AETHOR_ADRC_BENCH
    aethor_app_torque_envelope_finish(&result);
#endif
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
#if !AETHOR_ADRC_BENCH
    aethor_app_static_trace_emergency_begin(timestamp_us);
#endif
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

    aethor_app_pos_load_freeze_elapsed(timestamp_us);
    aethor_app_record_one_shot_failure_once(failed_stage,
                                            failure_error,
                                            failed_motor_number);
    aethor_app_static_load_latch_exit(application_action.local_failure_reason,
        (DebugUiStage)application_action.failed_stage, (uint8_t)application_action.failure_error, DEBUG_UI_FAILED);
    aethor_app_torque_envelope_fault();
    memset(&motor_snapshot, 0, sizeof(motor_snapshot));
    snapshot_status = motor_runtime_get_snapshot(
        &application_motor_runtime,
        timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US,
        &motor_snapshot);
    aethor_app_enter_task_critical();
    local_health_cleanup = aethor_app_debug_ui_health_failure_matches(&application_action.command);
    aethor_app_exit_task_critical();
    application_action.cleanup_disable_mask =
        ((application_action.failed_stage == PROTOCOL_COMMAND_STAGE_VALIDATE) &&
         (local_health_cleanup == 0U))
            ? 0U
            : application_action.command.motor_mask;
    aethor_app_static_trace_record(STATIC_TRACE_FIRST_FAULT, failed_motor_number, NULL,
        timestamp_us, timestamp_us, 0U, 0U,
        ((uint32_t)application_action.local_failure_reason << 16U) | (uint32_t)failure_error);
    application_action.static_identity.phase = DEBUG_UI_STATIC_TX_NONE;
    application_debug_ui.static_load.takeover_active = 0U;
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

    application_action.control_group_ready = 0U;
    /* Static protection is a control withdrawal, never a new powered target.
     * Preserve the original fault and deadline while issuing selected DISABLE. */
    if (debug_ui_is_pos_load_operation(application_action.command.local_operation) ||
        (application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
         debug_ui_is_static_operation(application_action.command.local_operation)))
    {
        (void)motor_runtime_abort_active_parameter_sequences(&application_motor_runtime, timestamp_us);
        runtime_status = motor_runtime_build_emergency_disable_subset(&application_motor_runtime,
            application_action.cleanup_disable_mask, &application_action.frames);
        application_action.state = AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT;
        application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
        application_action.frame_read_index = 0U;
        aethor_app_capture_feedback_baseline(&motor_snapshot);
    }
    else if ((application_action.failed_stage ==
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

#if !AETHOR_ADRC_BENCH
/** @brief Keep single static native pose bounds separate from ordinary MIT and POS authority. */
static DebugUiMotorProfile aethor_app_single_static_bounds(uint8_t joint_index)
{
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
    return debug_ui_single_static_bounds(&application_debug_ui.profiles[joint_index],
        (uint8_t)(joint_index + 1U),
        discovery->verified_fields_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK ? discovery->ranges.position_max_rad : 0.0F,
        1.0F);
}
#endif

/** @brief Compare every existing long-window discovery binding field without struct padding. */
static uint8_t aethor_app_single_static_discovery_equal(
    const MotorDiscoveryResult *left, const MotorDiscoveryResult *right)
{
    return (uint8_t)(left->observed_master_id == right->observed_master_id &&
        left->observed_esc_id == right->observed_esc_id &&
        left->observed_control_mode == right->observed_control_mode &&
        left->verified_fields_mask == right->verified_fields_mask &&
        left->hardware_version == right->hardware_version &&
        left->software_version == right->software_version && left->sub_version == right->sub_version &&
        left->communication_timeout_raw == right->communication_timeout_raw &&
        left->ranges.position_max_rad == right->ranges.position_max_rad &&
        left->ranges.velocity_max_rad_s == right->ranges.velocity_max_rad_s &&
        left->ranges.torque_max_nm == right->ranges.torque_max_nm &&
        left->maximum_speed_rad_s == right->maximum_speed_rad_s &&
        left->acceleration_rad_s2 == right->acceleration_rad_s2 &&
        left->deceleration_rad_s2 == right->deceleration_rad_s2);
}

/** @brief Compare original MIT parameters without changing ordinary POS/MIT reference semantics. */
static uint8_t aethor_app_single_static_mit_equal(
    const DebugUiMotorProfile *left, const DebugUiMotorProfile *right)
{
    return (uint8_t)(left->mit_valid == right->mit_valid &&
        left->mit_position_min_rad == right->mit_position_min_rad &&
        left->mit_position_max_rad == right->mit_position_max_rad &&
        left->mit_max_delta_rad == right->mit_max_delta_rad &&
        left->mit_max_speed_rad_s == right->mit_max_speed_rad_s &&
        left->kp_min == right->kp_min && left->kp_max == right->kp_max &&
        left->kd_min == right->kd_min && left->kd_max == right->kd_max &&
        left->default_kp == right->default_kp && left->default_kd == right->default_kd &&
        left->hold_min_ms == right->hold_min_ms && left->hold_max_ms == right->hold_max_ms);
}

/** @brief Track discovery/MIT/reference changes even before configuration; caller holds critical. */
static void aethor_app_single_static_refresh_evidence(uint8_t joint_index)
{
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
    const DebugUiMotorProfile *profile = &application_debug_ui.profiles[joint_index];
    if (application_debug_ui.single_static_evidence_generation[joint_index] == 0U ||
        application_debug_ui.single_static_observed_reference[joint_index] !=
            application_debug_ui.reference_generation[joint_index] ||
        !aethor_app_single_static_discovery_equal(discovery,
            &application_debug_ui.single_static_observed_discovery[joint_index]) ||
        !aethor_app_single_static_mit_equal(profile,
            &application_debug_ui.single_static_observed_profile[joint_index]))
    {
        ++application_debug_ui.single_static_evidence_generation[joint_index];
        if (application_debug_ui.single_static_evidence_generation[joint_index] == 0U)
        { ++application_debug_ui.single_static_evidence_generation[joint_index]; }
        application_debug_ui.single_static_observed_discovery[joint_index] = *discovery;
        application_debug_ui.single_static_observed_profile[joint_index] = *profile;
        application_debug_ui.single_static_observed_reference[joint_index] =
            application_debug_ui.reference_generation[joint_index];
        application_debug_ui.single_static_profiles[joint_index].configured = 0U;
        application_debug_ui.single_static_profiles[joint_index].torque_ff_nm = 0.0F;
    }
}

/** @brief Observe each received discovery transition before another frame can restore old values. */
static void aethor_app_single_static_refresh_received_evidence(void)
{
    uint8_t joint_index;
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    { aethor_app_single_static_refresh_evidence(joint_index); }
}

/** @brief Latch loss of exact identity/ranges/version or reference binding until explicit reconfiguration. */
static uint8_t aethor_app_single_static_profile_current(uint8_t joint_index)
{
    const MotorDiscoveryResult *bound;
    const MotorDiscoveryResult *current;
    uint8_t binding_current = 0U;
    if (joint_index >= DEBUG_UI_MOTOR_COUNT) { return 0U; }
    aethor_app_enter_task_critical();
    aethor_app_single_static_refresh_evidence(joint_index);
    bound = &application_debug_ui.single_static_discovery[joint_index];
    current = &application_motor_runtime.discovery.results[joint_index];
    if (application_debug_ui.single_static_profiles[joint_index].configured == 1U)
    {
        binding_current = (uint8_t)(application_debug_ui.single_static_reference[joint_index] ==
        application_debug_ui.reference_generation[joint_index] &&
        aethor_app_single_static_discovery_equal(bound, current));
        if (!binding_current)
        { application_debug_ui.single_static_profiles[joint_index].configured = 0U;
          application_debug_ui.single_static_profiles[joint_index].torque_ff_nm = 0.0F; }
    }
    aethor_app_exit_task_critical();
    return binding_current;
}

#include "DebugUi/debug_ui_static_trace_app.inc"
#include "DebugUi/debug_ui_torque_envelope_app.inc"
#include "DebugUi/debug_ui_static_load_app.inc"
#include "DebugUi/debug_ui_pos_load_app.inc"

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
        if (position_error_rad <= (aethor_app_is_pos_load() ? 0.125F : 0.5F) * AETHOR_APP_DEG_TO_RAD &&
            (!aethor_app_is_pos_load() || fabsf(feedback->velocity_rad_s) <=
             2.0F * application_motor_runtime.discovery.results[joint_index].ranges.velocity_max_rad_s / 4095.0F))
        {
            application_action.local_pos_progress_timestamp_us = timestamp_us;
            if (application_action.local_pos_settle_start_us == 0U)
            {
                application_action.local_pos_settle_start_us = feedback_timestamp_us;
            }
            return (uint8_t)(feedback_timestamp_us - application_action.local_pos_settle_start_us >=
                (aethor_app_is_pos_load() ? 250000ULL : AETHOR_APP_MOTION_SETTLE_US));
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
        if (application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI)
            application_action.local_failure_reason = DEBUG_UI_REASON_INVALID_FEEDBACK;
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
        if (aethor_app_is_pos_load()) profile = aethor_app_pos_load_bounds(joint_index);
        else if (!aethor_app_debug_ui_get_motor_profile((uint8_t)(joint_index + 1U), &profile)) {
            application_action.local_failure_reason = DEBUG_UI_REASON_UNCALIBRATED;
            return 0U;
        }
        if (!profile.pos_valid || !isfinite(profile.position_min_rad) ||
            !isfinite(profile.position_max_rad) ||
            (position_rad < profile.position_min_rad) ||
            (position_rad > profile.position_max_rad))
        {
            application_action.local_failure_reason = DEBUG_UI_REASON_OUT_OF_RANGE;
            return 0U;
        }
    }
    return 1U;
}
#endif

#if !AETHOR_ADRC_BENCH
/** @brief Offline candidate: widen only proven ID7 single HOLD tracking; retain startup and all other scopes. */
static float aethor_app_static_follow_error_limit_rad(uint8_t joint_index)
{
    if (joint_index == 6U &&
        application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
        application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION &&
        application_action.command.mit_action == PROTOCOL_MIT_ACTION_HOLD &&
        application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        application_action.command.motor_mask == 0x40U &&
        application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT &&
        (application_action.static_first_hold_mask & 0x40U) != 0U)
    { return 1.0F * AETHOR_APP_DEG_TO_RAD; }
    return 0.5F * AETHOR_APP_DEG_TO_RAD;
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
    /* Static holding cannot silently consume its window while its selected sender stalls. */
    if (debug_ui_is_static_operation(application_action.command.local_operation) &&
        application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT &&
        timestamp_us >= application_debug_ui.static_last_batch_us &&
        timestamp_us - application_debug_ui.static_last_batch_us > AETHOR_APP_MIT_MINIMUM_TRAJECTORY_US)
    {
        *failure_detected = 1U;
        application_action.local_failure_reason = DEBUG_UI_REASON_UNHEALTHY;
        return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_HOLD,
            PROTOCOL_COMMAND_ERROR_TIMEOUT, 0U, timestamp_us);
    }
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
        if (aethor_app_is_pos_load()) aethor_app_pos_load_observe(motor_snapshot, joint_index, timestamp_us);
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
        if (aethor_app_is_pos_load() && application_action.state != AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT)
        {
            DebugUiReason pos_reason = aethor_app_pos_load_guard(motor_snapshot, joint_index, timestamp_us);
            if (pos_reason != DEBUG_UI_REASON_NONE) {
                application_action.local_failure_reason = pos_reason;
                *failure_detected = 1U;
                return aethor_app_begin_one_shot_cleanup(failed_stage,
                    PROTOCOL_COMMAND_ERROR_ACTION_FAILED, (uint8_t)(joint_index + 1U), timestamp_us);
            }
        }
        /* Enforce local MIT commissioning feedback guards independently of USB monitoring. */
        if ((application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
            (application_action.command.type == PROTOCOL_COMMAND_MIT_ACTION) &&
            (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT))
        {
            const MotorJointFeedback *feedback = &motor_snapshot->joints[joint_index];
            DebugUiMotorProfile scoped_profile = application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ?
                aethor_app_single_static_bounds(joint_index) : application_debug_ui.profiles[joint_index];
            const DebugUiMotorProfile *profile = &scoped_profile;
            /* One feedback LSB accommodates rounding at the discovered speed boundary. */
            float speed_margin = 2.0F * application_motor_runtime.discovery.results[joint_index].ranges.velocity_max_rad_s / 4095.0F;
            DebugUiReason guard_reason = DEBUG_UI_REASON_NONE;
            if (!isfinite(feedback->torque_nm) || !isfinite(feedback->velocity_rad_s) ||
                !isfinite(feedback->position_rad) || !isfinite(feedback->mos_temperature_c) ||
                !isfinite(feedback->rotor_temperature_c))
                guard_reason = DEBUG_UI_REASON_INVALID_FEEDBACK;
            else if ((guard_reason = aethor_app_torque_envelope_guard(feedback, joint_index, timestamp_us)) != DEBUG_UI_REASON_NONE)
            { /* Preserve the precise policy failure while retaining every following original guard. */ }
            else if (application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
                !aethor_app_single_static_profile_current(joint_index))
                guard_reason = DEBUG_UI_REASON_UNCALIBRATED;
            else if (debug_ui_is_static_operation(application_action.command.local_operation) &&
                (application_action.command.static_reference_generations[joint_index] !=
                 application_debug_ui.reference_generation[joint_index]))
                guard_reason = DEBUG_UI_REASON_OLD_EPOCH;
            else if (debug_ui_is_static_operation(application_action.command.local_operation) &&
                ((application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT) ||
                 (application_action.static_first_hold_mask & joint_bit)) &&
                (feedback->driver_state != S3519_DRIVER_STATE_ENABLED))
                guard_reason = DEBUG_UI_REASON_NOT_READY;
            else if (debug_ui_is_static_operation(application_action.command.local_operation) &&
                ((application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT) ||
                 (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT)) &&
                 (fabsf(feedback->position_rad - application_action.target_position_rad[joint_index]) /
                  aethor_app_lcd_position_ratio(joint_index) > aethor_app_static_follow_error_limit_rad(joint_index)))
                guard_reason = DEBUG_UI_REASON_OUT_OF_RANGE;
            else if (debug_ui_is_static_operation(application_action.command.local_operation) &&
                application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT &&
                application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD &&
                feedback->driver_state != S3519_DRIVER_STATE_DISABLED)
                guard_reason = DEBUG_UI_REASON_NOT_DISABLED;
            else if (debug_ui_is_static_operation(application_action.command.local_operation) &&
                application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT &&
                application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD &&
                fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S)
                guard_reason = DEBUG_UI_REASON_SPEED_LIMIT;
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
static uint8_t aethor_app_start_one_shot_hold(uint64_t timestamp_us);
#include "DebugUi/debug_ui_static_startup_app.inc"
#endif

#if !AETHOR_ADRC_BENCH
static uint8_t aethor_app_validate_one_shot_targets(
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    uint8_t failed_joint_index = 0U;
    uint8_t joint_index;
    MotorRuntimeStatus runtime_status = MOTOR_RUNTIME_STATUS_OK;

    if (application_action.command.type == PROTOCOL_COMMAND_HOME_PREPARE)
    {
        /* This non-motion intent has emitted DISABLE only. Qualification is impossible before its newer standard ACK. */
        ProtocolCommand qualified = application_action.command;
        DebugUiReason reason;
        if (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT ||
            application_action.frame_read_index < application_action.frames.count || !motor_snapshot ||
            qualified.home_target_position_rad != application_action.target_position_rad[6] ||
            !aethor_app_selected_feedback_advanced(motor_snapshot, qualified.motor_mask,
                application_action.phase_feedback_baseline_us) ||
            !aethor_app_selected_motors_have_state(motor_snapshot, qualified.motor_mask, S3519_DRIVER_STATE_DISABLED))
        { reason = DEBUG_UI_REASON_STALE_FEEDBACK; }
        else
        {
            reason = aethor_app_debug_ui_validate_start(&qualified, motor_snapshot, timestamp_us, true);
            if (reason == DEBUG_UI_REASON_NONE)
            {
                qualified.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
                qualified.relative_target = 1U;
                qualified.values[6] = qualified.home_target_position_rad - motor_snapshot->joints[6].position_rad;
                qualified.home_prepared = 1U;
                /* No HOME exception: this relative candidate must pass the original complete motion guard. */
                reason = aethor_app_debug_ui_validate_start(&qualified, motor_snapshot, timestamp_us, true);
            }
        }
        if (reason != DEBUG_UI_REASON_NONE)
        {
            application_action.local_failure_reason = reason;
            return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_VALIDATE,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 7U, timestamp_us);
        }
        application_action.command = qualified;
    }
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
                    application_action.command.local_operation == DEBUG_UI_OPERATION_HOME ?
                        application_action.command.home_target_position_rad :
                        motor_snapshot->joints[joint_index].position_rad + application_action.command.values[joint_index];
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
                    debug_ui_is_static_operation(application_action.command.local_operation) ?
                        application_action.command.values[joint_index] : motor_snapshot->joints[joint_index].position_rad;
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
    if (aethor_app_is_pos_load())
    {
        MotorEmergencyFrameBatch frozen;
        float preload_position[ARM_JOINT_COUNT] = {0.0F};
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
            preload_position[joint_index] = motor_snapshot->joints[joint_index].position_rad;
        runtime_status = motor_runtime_build_position_velocity_subset(&application_motor_runtime,
            application_action.command.motor_mask, preload_position, application_action.target_speed_rad_s, &frozen);
        if (runtime_status != MOTOR_RUNTIME_STATUS_OK || frozen.count != 1U)
            return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_ENABLE,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 0U, timestamp_us);
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
            if (application_action.command.motor_mask & (1U << joint_index)) break;
        application_action.static_frozen_frames[joint_index] = frozen.frames[0];
        if (!application_action.static_setup_deadline_us)
            application_action.static_setup_deadline_us = timestamp_us + AETHOR_APP_ACTION_TIMEOUT_US;
        application_action.deadline_us = application_action.static_setup_deadline_us;
        application_action.state = AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT;
        aethor_app_enter_task_critical();
        application_debug_ui.pos_load.stage = DEBUG_UI_STAGE_ENABLE;
        application_debug_ui.pos_load.target_position_rad = application_action.target_position_rad[joint_index] /
            aethor_app_lcd_position_ratio(joint_index);
        application_debug_ui.pos_load_start_position_rad = preload_position[joint_index];
        aethor_app_exit_task_critical();
        aethor_app_static_prepare_phase(DEBUG_UI_STATIC_TX_PRELOAD, joint_index, timestamp_us);
        return 0U;
    }
    if (debug_ui_is_static_operation(application_action.command.local_operation))
    {
        MotorEmergencyFrameBatch frozen;
        float zero_speed[ARM_JOINT_COUNT] = {0.0F};
        uint8_t frame_index = 0U;
        runtime_status = motor_runtime_build_mit_subset(&application_motor_runtime,
            application_action.command.motor_mask, application_action.target_position_rad, zero_speed,
            application_action.command.mit_kp, application_action.command.mit_kd,
            0.0F, &frozen);
        if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
            return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_ENABLE,
                aethor_app_map_runtime_error(runtime_status), 0U, timestamp_us);
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
            if (application_action.command.motor_mask & (1U << joint_index))
                application_action.static_frozen_frames[joint_index] = frozen.frames[frame_index++];
        /* One original setup budget is shared by every PRELOAD, ENABLE and first-HOLD barrier. */
        if (!application_action.static_setup_deadline_us &&
            !aethor_app_calculate_deadline_us(timestamp_us, AETHOR_APP_ACTION_TIMEOUT_US,
            &application_action.static_setup_deadline_us))
            return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_ENABLE,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 0U, timestamp_us);
        application_action.deadline_us = application_action.static_setup_deadline_us;
        application_action.state = AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT;
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
            if (application_action.command.motor_mask & (1U << joint_index)) break;
        aethor_app_static_prepare_phase(DEBUG_UI_STATIC_TX_PRELOAD, joint_index, timestamp_us);
        return 0U;
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
            ((application_action.command.type == PROTOCOL_COMMAND_HOME_PREPARE) ||
             (application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
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
    if (debug_ui_is_static_operation(application_action.command.local_operation) || aethor_app_is_pos_load()) {
        if (!application_action.static_setup_deadline_us)
            application_action.static_setup_deadline_us = application_action.deadline_us;
        application_action.deadline_us = application_action.static_setup_deadline_us;
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
        ((application_action.command.type == PROTOCOL_COMMAND_HOME_PREPARE) ||
         (application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
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
    if (aethor_app_is_pos_load())
        memcpy(hold_speed_rad_s, application_action.target_speed_rad_s, sizeof(hold_speed_rad_s));
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
    aethor_app_static_load_begin(timestamp_us);
    if (aethor_app_is_pos_load()) {
        aethor_app_enter_task_critical();
        application_debug_ui.pos_load.stage = DEBUG_UI_STAGE_HOLD;
        application_debug_ui.pos_load_hold_started_us = timestamp_us;
        application_debug_ui.pos_load_sample_us = timestamp_us;
        application_action.last_pos_reference_timestamp_us = timestamp_us;
        aethor_app_exit_task_critical();
    }
    application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
    application_action.frame_read_index = 0U;
    if (!aethor_app_calculate_deadline_us(
            timestamp_us,
            (application_action.command.control_mode == ARM_CONTROL_MODE_MIT || aethor_app_is_pos_load())
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
    aethor_app_pos_load_freeze_elapsed(timestamp_us);
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
    if (command->local_operation == DEBUG_UI_OPERATION_HOME && command->type != PROTOCOL_COMMAND_HOME_PREPARE)
    {
        application_action.local_failure_reason = DEBUG_UI_REASON_INVALID_ARGUMENT;
        return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_VALIDATE,
            PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 7U, timestamp_us);
    }
    aethor_app_pos_load_begin(timestamp_us);
    aethor_app_static_trace_begin(timestamp_us);
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
                command->type == PROTOCOL_COMMAND_HOME_PREPARE ? command->home_target_position_rad :
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

    if ((debug_ui_is_static_operation(application_action.command.local_operation) || aethor_app_is_pos_load()) &&
        application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT && application_action.static_identity.phase)
        return aethor_app_static_advance_startup(motor_snapshot, timestamp_us);
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
        if (aethor_app_is_pos_load()) {
            uint8_t recovery_timeout = aethor_app_pos_load_sample(motor_snapshot, timestamp_us);
            if (recovery_timeout || (timestamp_us >= application_action.deadline_us && application_debug_ui.pos_load.recovering))
                return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_HOLD,
                    PROTOCOL_COMMAND_ERROR_TIMEOUT, 0U, timestamp_us);
            if (timestamp_us >= application_action.deadline_us) {
                aethor_app_enter_task_critical();
                application_debug_ui.pos_load.stage = DEBUG_UI_STAGE_DISABLE;
                aethor_app_exit_task_critical();
                return aethor_app_start_one_shot_disable(motor_snapshot, timestamp_us);
            }
            if (application_action.frame_read_index >= application_action.frames.count) {
                application_action.frame_read_index = 0U;
                application_action.last_pos_reference_timestamp_us = timestamp_us;
            }
            return 0U;
        }
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
             /* POS hardware SOF proves ordering even when HAL millisecond bounds are equal. */
             (aethor_app_is_pos_load() ? aethor_app_pos_disable_proven(motor_snapshot) :
              aethor_app_selected_feedback_advanced(
                  motor_snapshot,
                  motor_mask,
                  application_action.phase_feedback_baseline_us) != 0U) &&
             (aethor_app_selected_motors_have_state(
                  motor_snapshot,
                  motor_mask,
                  S3519_DRIVER_STATE_DISABLED) != 0U))
    {
        aethor_app_update_protocol_context(timestamp_us);
        if (aethor_app_torque_envelope_scope(&application_action.command, 6U) &&
            application_action.local_failure_reason != DEBUG_UI_REASON_NONE)
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED,
                (uint16_t)application_action.failure_error, timestamp_us);
        return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_COMPLETED, 0U, timestamp_us);
    }
    else if ((application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT) &&
             (application_action.frame_read_index >=
              application_action.frames.count) &&
             (aethor_app_is_pos_load() ? aethor_app_pos_disable_proven(motor_snapshot) :
              aethor_app_selected_feedback_advanced(
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
    torque_envelope_init(&application_torque_envelope);

#if !AETHOR_ADRC_BENCH
    aethor_app_remote_reset();
    memset(&application_torque_fault, 0, sizeof(application_torque_fault));
    application_remote_release_can_idle = NULL;
    application_remote_release_link_word = NULL;
#endif
#if AETHOR_IDLE_FEEDBACK_ENABLE
    memset(&application_idle_feedback, 0, sizeof(application_idle_feedback));
    memset(&application_standby_recovery, 0, sizeof(application_standby_recovery));
    application_standby_recovery.last_service_us = timestamp_us;
    application_standby_recovery.next_attempt_us = timestamp_us > UINT64_MAX - MOTOR_STANDBY_RECOVERY_COOLDOWN_US ?
        UINT64_MAX : timestamp_us + MOTOR_STANDBY_RECOVERY_COOLDOWN_US;
#endif
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
    memset(&application_coordinate_pair, 0, sizeof(application_coordinate_pair));
    memset(&application_low_speed_mapping, 0, sizeof(application_low_speed_mapping));
    application_mapping_rx_boundary_us = 0ULL;
    application_integrated_can_quiet_since_us = 0ULL;
    application_integrated_probe_submitted_us = 0ULL;
    application_integrated_probe_transmitted = 0U;
    application_integrated_probe_failed = 0U;
    application_integrated_acquire_disable_transmitted = 0U;
    application_integrated_acquire_neutral_submitted = 0U;
    application_integrated_acquire_disable_retries = 0U;
    application_integrated_release_last_submit_us = 0ULL;
    application_integrated_release_disable_retries = 0U;
    application_integrated_acquire_rx_count = 0U;
    application_integrated_acquire_first_submit_us = 0ULL;
    application_integrated_acquire_other_count = 0U;
    application_integrated_acquire_unknown_time_count = 0U;
    application_integrated_acquire_old_count = 0U;
    memset(application_integrated_acquire_phases, 0, sizeof(application_integrated_acquire_phases));
    application_integrated_acquire_unassigned_count = 0U;
    application_integrated_acquire_valid_count = 0U;
    application_integrated_acquire_rejected_count = 0U;
    application_integrated_acquire_last_rx_id = 0U;
    application_integrated_acquire_last_rx_length = 0U;
    application_integrated_diag_requested_us = 0ULL;
    application_integrated_diag_transmitted_us = 0ULL;
    application_integrated_diag_query_length = 4U;
    application_integrated_diag_rx_after_tx = 0U;
    application_integrated_diag_rx_valid = 0U;
    application_integrated_diag_rx_rejected = 0U;
    application_integrated_diag_last_rx_id = 0U;
    application_integrated_diag_last_rx_length = 0U;
    application_integrated_diag_last_rx_data0 = 0U;
    application_integrated_diag_last_rx_data2 = 0U;
    application_integrated_handoff = ADRC_LCD_OWNERSHIP_WAITING;
    if (application_initialized != 0U && adrc_app_bridge_init_deferred(&application_adrc_bridge,
        &application_motor_runtime, &application_protocol_engine,
        adrc_generated_controller_step, NULL) != ADRC_RESULT_OK)
    { application_initialized = 0U; }
    application_adrc_bridge.capability_owner_active = aethor_app_capability_owner_active;
#endif
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    aethor_app_persistence_init();
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

/** @brief A mapping transition or uncertain failure forbids every motion producer. */
static uint8_t aethor_app_mapping_locked(void)
{
    return (uint8_t)(application_low_speed_mapping.state != AETHOR_MAPPING_IDLE &&
        application_low_speed_mapping.state != AETHOR_MAPPING_DONE);
}

/** @brief Clear old feedback and all model trust before any range write or after uncertain failure. */
static void aethor_app_mapping_invalidate(void)
{
    MotorObject *motor = &application_motor_runtime.bank.motors[6];
    motor->feedback_valid = 0U;
    motor->target_valid = 0U;
    motor->parameter_valid_mask = 0U;
    motor->configuration_consistent = 0U;
    application_motor_runtime.bank.valid_joint_mask &= (uint8_t)~0x40U;
    application_motor_runtime.discovery.verified_joint_mask &= (uint8_t)~0x40U;
    application_motor_runtime.discovery.results[6].verified_fields_mask = 0U;
    application_adrc_bridge.bench.trusted_qualification.verified_flags = 0U;
    application_adrc_bridge.bench.experiment.qualification.verified_flags = 0U;
    application_adrc_bridge.bench.experiment.configured = 0U;
    application_adrc_bridge.evidence_bound = 0U;
    adrc_feedback_adapter_reset(&application_adrc_bridge.feedback_adapter);
    memset(&application_motor_runtime.feedback_timing[6], 0,
        sizeof(application_motor_runtime.feedback_timing[6]));
}

/** @brief Preserve failure and the lock; STOP/DISABLE remain independently routable. */
static void aethor_app_mapping_fail(uint8_t reason, uint64_t timestamp_us)
{
    if (!aethor_app_mapping_locked() || application_low_speed_mapping.state == AETHOR_MAPPING_FAILED) { return; }
    (void)motor_runtime_abort_active_parameter_sequences(&application_motor_runtime, timestamp_us);
    aethor_app_mapping_invalidate();
    application_low_speed_mapping.state = AETHOR_MAPPING_FAILED;
    application_low_speed_mapping.reason = reason;
    application_low_speed_mapping.disable_pending = 1U;
    if (timestamp_us > application_low_speed_mapping.phase_us)
    { application_low_speed_mapping.phase_us = timestamp_us; }
    application_low_speed_mapping.disabled_observed_us = 0ULL;
}

/** @brief Match only the audited motor/firmware and the complete fixed position/velocity/torque contract. */
static uint8_t aethor_app_mapping_identity(const MotorDiscoveryResult *discovery,
    float position_range, float velocity_range)
{
    return (uint8_t)(discovery->observed_esc_id == 7U && discovery->observed_master_id == 23U &&
        discovery->observed_control_mode == 2U && discovery->software_version == 808597046U &&
        discovery->sub_version == 49U && discovery->ranges.position_max_rad == position_range &&
        discovery->ranges.torque_max_nm == 10.0F && discovery->ranges.velocity_max_rad_s == velocity_range &&
        discovery->verified_fields_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK);
}

/** @brief Require locally authorized REMOTE ownership and empty producers/results, allowing only our discovery. */
static uint8_t aethor_app_mapping_sources_idle(uint8_t own_discovery)
{
    return (uint8_t)(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE &&
        !application_protocol_engine.local_control_locked &&
        application_action.state == AETHOR_APP_ACTION_IDLE &&
        !application_debug_ui.fallback_stop_mask && !application_debug_ui.mailbox.stop_intent_mask &&
        !debug_ui_mailbox_busy(&application_debug_ui.mailbox, false) &&
        !debug_ui_mailbox_busy(&application_debug_ui.mailbox, true) &&
        !debug_ui_mailbox_result_count(&application_debug_ui.mailbox) &&
        !application_debug_ui.unconfirmed_disable_mask &&
        application_protocol_engine.command_read_sequence == application_protocol_engine.command_write_sequence &&
        application_protocol_engine.stop_read_sequence == application_protocol_engine.stop_write_sequence &&
        application_protocol_engine.result_read_sequence == application_protocol_engine.result_write_sequence &&
        !application_protocol_engine.active_motion_request_id && !application_protocol_engine.active_stop_request_id &&
        !aethor_app_deferred_result_count() &&
        application_debug_ui.fallback_frame_read_index >= application_debug_ui.fallback_frames.count &&
        application_emergency_disable_read_index >= application_emergency_disable_batch.count &&
        !application_action.control_group_ready && application_action.frame_read_index >= application_action.frames.count &&
        application_coordinate_pair.state != 1U && !application_motor_runtime.position_read.pending &&
        !application_motor_runtime.idle_position_read.pending &&
        (own_discovery || !application_motor_runtime.discovery_active) &&
        application_motor_runtime.mode_switch_state != MOTOR_MODE_SWITCH_WRITING &&
        application_motor_runtime.mode_switch_state != MOTOR_MODE_SWITCH_READ_READY &&
        application_motor_runtime.mode_switch_state != MOTOR_MODE_SWITCH_READ_WAITING);
}

#include "Motor/motor_persistence_app.inc"
#include "Motor/motor_persistence_wire.inc"

/** @brief Admit one fixed PMAX/VMAX change or explicit same-register/family restore with disabled evidence. */
static uint8_t aethor_app_mapping_start(uint32_t request_id, uint8_t restore,
    AethorMappingFamily family, uint64_t timestamp_us)
{
    const MotorObject *motor = &application_motor_runtime.bank.motors[6];
    const MotorJointFeedback *feedback = &motor->feedback;
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[6];
    AethorLowSpeedMapping *mapping = &application_low_speed_mapping;
    uint8_t recovery = (uint8_t)(restore && mapping->state == AETHOR_MAPPING_FAILED);
    uint8_t position_mapping = (uint8_t)(family != AETHOR_MAPPING_FAMILY_VELOCITY);
    S3519Register register_address = position_mapping ? S3519_REGISTER_POSITION_RANGE : S3519_REGISTER_VELOCITY_RANGE;
    float position_source = family == AETHOR_MAPPING_FAMILY_RATED70_POSITION ? 512.0F : 12.5F;
    float position_target = family == AETHOR_MAPPING_FAMILY_RATED70_POSITION ? 1024.0F : 512.0F;
    float source_range = position_mapping ? (restore ? position_target : position_source) : (restore ? 5.0F : 200.0F);
    float target_range = position_mapping ? (restore ? position_source : position_target) : (restore ? 200.0F : 5.0F);
    if (!request_id || application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD ||
        (family != AETHOR_MAPPING_FAMILY_VELOCITY && family != AETHOR_MAPPING_FAMILY_WIDE_POSITION &&
         family != AETHOR_MAPPING_FAMILY_RATED70_POSITION) ||
        !aethor_app_mapping_sources_idle(0U) ||
        timestamp_us < application_motor_runtime.idle_position_read.not_before_us ||
        !application_integrated_can_idle ||
        timestamp_us < application_integrated_can_quiet_since_us ||
        timestamp_us - application_integrated_can_quiet_since_us < 4000ULL ||
        mapping->disable_pending ||
        (recovery && (mapping->register_address != (uint8_t)register_address || mapping->family != family)) ||
        (recovery && timestamp_us < mapping->started_us) ||
        (recovery && position_mapping &&
          (discovery->observed_esc_id != 7U || discovery->observed_master_id != 23U ||
           discovery->observed_control_mode != 2U || discovery->software_version != 808597046U ||
           discovery->sub_version != 49U || discovery->ranges.velocity_max_rad_s != 200.0F ||
           discovery->ranges.torque_max_nm != 10.0F ||
           (discovery->ranges.position_max_rad != position_source && discovery->ranges.position_max_rad != position_target))) ||
        (!recovery && mapping->state != AETHOR_MAPPING_IDLE && mapping->state != AETHOR_MAPPING_DONE) ||
        (!restore && mapping->state != AETHOR_MAPPING_IDLE) ||
        (!recovery && (!aethor_app_mapping_identity(discovery,
          position_mapping ? source_range : 12.5F, position_mapping ? 200.0F : source_range) ||
          !motor->feedback_valid || timestamp_us < feedback->timestamp_us ||
          timestamp_us - feedback->timestamp_us > 100000ULL ||
          feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags ||
          !isfinite(feedback->position_rad) || !isfinite(feedback->velocity_rad_s) || fabsf(feedback->velocity_rad_s) > 0.05F ||
          !isfinite(feedback->torque_nm) || fabsf(feedback->torque_nm) > 0.15F ||
          !isfinite(feedback->mos_temperature_c) || !isfinite(feedback->rotor_temperature_c) ||
          feedback->mos_temperature_c >= 50.0F || feedback->rotor_temperature_c >= 50.0F)) ||
        (recovery && (!mapping->disabled_observed_us || timestamp_us < mapping->disabled_observed_us ||
          timestamp_us - mapping->disabled_observed_us > 100000ULL))) { return 0U; }
    memset(mapping, 0, sizeof(*mapping));
    mapping->started_us = mapping->phase_us = timestamp_us;
    mapping->request_id = request_id;
    mapping->target_range = target_range;
    mapping->source_range = source_range;
    mapping->register_address = (uint8_t)register_address;
    mapping->family = family;
    mapping->recovery = recovery;
    mapping->state = AETHOR_MAPPING_QUIET;
    aethor_app_mapping_invalidate();
    return 1U;
}

/** @brief Consume maintenance replies and quarantine uncertain normal feedback before the new baseline. */
static uint8_t aethor_app_mapping_receive(const CanFrame *frame, uint64_t timestamp_us)
{
    AethorLowSpeedMapping *mapping = &application_low_speed_mapping;
    S3519ParameterResponse response;
    if (frame == NULL || mapping->state == AETHOR_MAPPING_IDLE) { return 0U; }
    if (frame->length >= 4U && frame->data[0] == 7U && frame->data[1] == 0U &&
        (frame->data[2] == 0x33U || frame->data[2] == 0x55U) && frame->data[3] != mapping->register_address &&
        aethor_app_mapping_locked())
    {
        uint8_t index;
        if (mapping->state == AETHOR_MAPPING_DISCOVERY && frame->data[2] == 0x33U)
        {
            for (index = 0U; index < application_motor_runtime.parameter_expectations.count; ++index)
            {
                const MotorParameterResponseSignature *expected =
                    &application_motor_runtime.parameter_expectations.entries[index];
                if (expected->valid && expected->register_address == frame->data[3] &&
                    expected->identifier == frame->identifier) { return 0U; }
            }
        }
        return 1U;
    }
    if (frame->length >= 4U && frame->data[0] == 7U && frame->data[1] == 0U &&
        (frame->data[2] == 0x33U || frame->data[2] == 0x55U) && frame->data[3] == mapping->register_address)
    {
        /* Full rediscovery owns its read expectation; write ACKs never prove readback. */
        if ((mapping->state == AETHOR_MAPPING_DISCOVERY || mapping->state == AETHOR_MAPPING_DONE) &&
            application_motor_runtime.discovery_active && frame->data[2] == 0x33U) { return 0U; }
        if (mapping->state == AETHOR_MAPPING_SOURCE_WAIT || mapping->state == AETHOR_MAPPING_TARGET_WAIT)
        {
            if (frame->data[2] == 0x55U) { return 1U; }
            if (frame->identifier != 23U || frame->length != 8U ||
                timestamp_us < mapping->phase_us || timestamp_us - mapping->phase_us >= 100000ULL ||
                s3519_decode_parameter_response(frame, &response) != S3519_CODEC_STATUS_OK ||
                !isfinite(response.float_value) ||
                (mapping->state == AETHOR_MAPPING_TARGET_WAIT && response.float_value != mapping->target_range) ||
                (mapping->state == AETHOR_MAPPING_SOURCE_WAIT &&
                  response.float_value != mapping->source_range && !(mapping->recovery && response.float_value == mapping->target_range)))
            { aethor_app_mapping_fail(4U, timestamp_us); return 1U; }
            if (mapping->state == AETHOR_MAPPING_SOURCE_WAIT)
            { mapping->source_range = response.float_value; mapping->state = AETHOR_MAPPING_WRITE; }
            else { mapping->state = AETHOR_MAPPING_DRAIN; }
            mapping->phase_us = timestamp_us;
        }
        return 1U;
    }
    if (aethor_app_mapping_locked() && frame->identifier == 23U && frame->length == 8U &&
        (frame->data[0] & 0x0FU) == 7U)
    {
        if (mapping->state != AETHOR_MAPPING_FAILED &&
            ((frame->data[0] >> 4U) != S3519_DRIVER_STATE_DISABLED ||
             frame->data[6] >= 50U || frame->data[7] >= 50U))
        { aethor_app_mapping_fail(6U, timestamp_us); return 1U; }
        if (mapping->state == AETHOR_MAPPING_BASELINE && timestamp_us >= mapping->phase_us) { return 0U; }
        if (mapping->state == AETHOR_MAPPING_FAILED)
        {
            const S3519Ranges conservative_ranges = {12.5F, 200.0F, 10.0F};
            S3519Feedback feedback;
            mapping->disabled_observed_us = 0ULL;
            if (timestamp_us && timestamp_us >= mapping->started_us && timestamp_us >= mapping->phase_us &&
                s3519_decode_feedback(frame, &conservative_ranges, &feedback) == S3519_CODEC_STATUS_OK &&
                feedback.state == S3519_DRIVER_STATE_DISABLED && fabsf(feedback.velocity_rad_s) <= 0.05F &&
                fabsf(feedback.torque_nm) <= 0.15F && feedback.mos_temperature_c < 50.0F &&
                feedback.rotor_temperature_c < 50.0F)
            { mapping->disabled_observed_us = timestamp_us; }
        }
        return 1U;
    }
    return 0U;
}

/** @brief Emit bounded fixed maintenance frames, then rediscover and obtain a freshly decoded DISABLE reply. */
static MotorRuntimeStatus aethor_app_mapping_next(uint64_t timestamp_us, CanFrame *frame, CanTxPriority *priority)
{
    AethorLowSpeedMapping *mapping = &application_low_speed_mapping;
    MotorRuntimeStatus status;
    const MotorJointFeedback *feedback = &application_motor_runtime.bank.motors[6].feedback;
    uint64_t age;
    uint8_t bytes[8] = {7U, 0U, 0x55U, 0U, 0U, 0U, 0U, 0U};
    if (!aethor_app_mapping_locked() || mapping->state == AETHOR_MAPPING_FAILED) { return MOTOR_RUNTIME_STATUS_WAITING; }
    if (timestamp_us < mapping->started_us || timestamp_us - mapping->started_us >= 3000000ULL)
    { aethor_app_mapping_fail(2U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD ||
        !aethor_app_mapping_sources_idle((uint8_t)(mapping->state == AETHOR_MAPPING_DISCOVERY)))
    { aethor_app_mapping_fail(3U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
    age = timestamp_us >= mapping->phase_us ? timestamp_us - mapping->phase_us : UINT64_MAX;
    *priority = CAN_TX_PRIORITY_PARAMETER;
    if ((mapping->state == AETHOR_MAPPING_QUIET && age >= 500000ULL) ||
        (mapping->state == AETHOR_MAPPING_SETTLE && age >= 100000ULL))
    {
        if (!timestamp_us || application_mapping_rx_boundary_us != timestamp_us) { return MOTOR_RUNTIME_STATUS_WAITING; }
        if (s3519_pack_parameter_read(7U, (S3519Register)mapping->register_address, frame) != S3519_CODEC_STATUS_OK)
        { aethor_app_mapping_fail(4U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
        mapping->state = mapping->state == AETHOR_MAPPING_QUIET ? AETHOR_MAPPING_SOURCE_WAIT : AETHOR_MAPPING_TARGET_WAIT;
        mapping->phase_us = timestamp_us;
        return MOTOR_RUNTIME_STATUS_FRAME_READY;
    }
    if ((mapping->state == AETHOR_MAPPING_SOURCE_WAIT || mapping->state == AETHOR_MAPPING_TARGET_WAIT ||
         mapping->state == AETHOR_MAPPING_BASELINE) && age >= 100000ULL)
    { aethor_app_mapping_fail(2U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
    if (mapping->state == AETHOR_MAPPING_WRITE)
    {
        if (!timestamp_us || application_mapping_rx_boundary_us != timestamp_us) { return MOTOR_RUNTIME_STATUS_WAITING; }
        bytes[3] = mapping->register_address;
        memcpy(bytes + 4U, &mapping->target_range, sizeof(float));
        if (can_frame_init(frame, 0x7FFU, bytes, 8U) != CAN_FRAME_STATUS_OK)
        { aethor_app_mapping_fail(4U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
        mapping->writes = 1U;
        mapping->state = AETHOR_MAPPING_SETTLE;
        mapping->phase_us = timestamp_us;
        return MOTOR_RUNTIME_STATUS_FRAME_READY;
    }
    if (mapping->state == AETHOR_MAPPING_DRAIN && age >= 500000ULL)
    {
        if (motor_runtime_begin_discovery(&application_motor_runtime, 0x40U) != MOTOR_RUNTIME_STATUS_OK)
        { aethor_app_mapping_fail(5U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
        mapping->state = AETHOR_MAPPING_DISCOVERY;
    }
    if (mapping->state == AETHOR_MAPPING_DISCOVERY)
    {
        status = motor_runtime_next_discovery_frame(&application_motor_runtime, timestamp_us, frame);
        if (status == MOTOR_RUNTIME_STATUS_DISCOVERY_COMPLETE)
        {
            if (!aethor_app_mapping_identity(&application_motor_runtime.discovery.results[6],
                mapping->register_address == S3519_REGISTER_POSITION_RANGE ? mapping->target_range : 12.5F,
                mapping->register_address == S3519_REGISTER_VELOCITY_RANGE ? mapping->target_range : 200.0F))
            { aethor_app_mapping_fail(5U, timestamp_us); }
            else { mapping->state = AETHOR_MAPPING_DISABLE; }
        }
        else if (status != MOTOR_RUNTIME_STATUS_FRAME_READY && status != MOTOR_RUNTIME_STATUS_WAITING)
        { aethor_app_mapping_fail(5U, timestamp_us); }
        return status;
    }
    if (mapping->state == AETHOR_MAPPING_DISABLE)
    {
        if (!timestamp_us || application_mapping_rx_boundary_us != timestamp_us) { return MOTOR_RUNTIME_STATUS_WAITING; }
        if (s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY, S3519_MODE_COMMAND_DISABLE, frame) != S3519_CODEC_STATUS_OK)
        { aethor_app_mapping_fail(6U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
        *priority = CAN_TX_PRIORITY_EMERGENCY;
        mapping->phase_us = timestamp_us;
        mapping->state = AETHOR_MAPPING_BASELINE;
        return MOTOR_RUNTIME_STATUS_FRAME_READY;
    }
    if (mapping->state == AETHOR_MAPPING_BASELINE && application_motor_runtime.bank.motors[6].feedback_valid &&
        feedback->timestamp_us >= mapping->phase_us && feedback->timestamp_us <= timestamp_us)
    {
        if (feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags ||
            !isfinite(feedback->position_rad) || !isfinite(feedback->velocity_rad_s) ||
            !isfinite(feedback->torque_nm) || !isfinite(feedback->mos_temperature_c) ||
            !isfinite(feedback->rotor_temperature_c) ||
            fabsf(feedback->velocity_rad_s) >
                (mapping->register_address == S3519_REGISTER_VELOCITY_RANGE && mapping->target_range == 5.0F ? 0.01F : 0.05F) ||
            fabsf(feedback->torque_nm) > 0.15F ||
            feedback->mos_temperature_c >= 50.0F || feedback->rotor_temperature_c >= 50.0F)
        { aethor_app_mapping_fail(6U, timestamp_us); }
        else { mapping->state = AETHOR_MAPPING_DONE; }
    }
    return MOTOR_RUNTIME_STATUS_WAITING;
}

/** @brief End only the passive reader, retaining the boot-local used state and late-reply quarantine. */
static void aethor_app_coordinate_cancel(uint8_t reason, uint64_t timestamp_us)
{
    CanFrame unused_frame;
    if (application_coordinate_pair.state != 1U) { return; }
    (void)motor_runtime_next_position_read(&application_motor_runtime, 6U,
        timestamp_us, 0U, &unused_frame);
    application_coordinate_pair.state = 3U;
    application_motor_runtime.position_read_reserved = 0U;
    application_coordinate_pair.reason = reason;
}

/** @brief Schedule only fixed 0x50/0x51 reads; service and RX timestamps are not physical TX proof. */
static MotorRuntimeStatus aethor_app_coordinate_next(uint64_t timestamp_us, CanFrame *frame)
{
    MotorRegisterPosition *reader = &application_motor_runtime.position_read;
    const MotorJointFeedback *feedback = &application_motor_runtime.bank.motors[6].feedback;
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[6];
    MotorRuntimeStatus result;
    if (application_coordinate_pair.state != 1U) { return MOTOR_RUNTIME_STATUS_WAITING; }
    if (timestamp_us < application_coordinate_pair.started_us ||
        timestamp_us - application_coordinate_pair.started_us >= 400000ULL ||
        reader->timeout_count != application_coordinate_pair.initial_timeouts)
    { aethor_app_coordinate_cancel(2U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD ||
        aethor_app_debug_ui_executor_busy() || application_motor_runtime.discovery_active ||
        discovery->observed_control_mode != 2U || discovery->observed_esc_id != 7U ||
        discovery->observed_master_id != 23U ||
        (discovery->verified_fields_mask & MOTOR_DISCOVERY_ALL_FIELDS_MASK) != MOTOR_DISCOVERY_ALL_FIELDS_MASK ||
        feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags != 0U)
    { aethor_app_coordinate_cancel(3U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
    if (reader->rejected_count != application_coordinate_pair.initial_rejections)
    { aethor_app_coordinate_cancel(4U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
    if (reader->seen_mask == 3U && !reader->pending)
    {
        if (reader->sample_us[0] <= application_coordinate_pair.started_us ||
            reader->sample_us[1] <= reader->sample_us[0] ||
            !isfinite(reader->values[0]) || !isfinite(reader->values[1]))
        { aethor_app_coordinate_cancel(4U, timestamp_us); }
        else { application_coordinate_pair.state = 2U; application_motor_runtime.position_read_reserved = 0U; }
        return MOTOR_RUNTIME_STATUS_WAITING;
    }
    /* HAL时钟为毫秒量化；首发须晚于准入时刻，使同发送tick的快速回执仍严格晚于started。
     * 期限、拒绝计数和上方严格RX时间门保持，不能把相等接收时间直接判为新鲜。 */
    if (timestamp_us == application_coordinate_pair.started_us)
    { return MOTOR_RUNTIME_STATUS_WAITING; }
    result = motor_runtime_next_position_read(&application_motor_runtime, 6U, timestamp_us, 1U, frame);
    if (reader->timeout_count != application_coordinate_pair.initial_timeouts)
    { aethor_app_coordinate_cancel(2U, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
    return result;
}

/** @brief Admit one raw pair after current disabled feedback, without ownership transfer or any motor command. */
static uint8_t aethor_app_coordinate_start(uint32_t request_id, uint64_t timestamp_us)
{
    AdrcLcdOwnershipEvidence evidence;
    MotorRegisterPosition *reader = &application_motor_runtime.position_read;
    const MotorObject *motor = &application_motor_runtime.bank.motors[6];
    const MotorJointFeedback *feedback = &motor->feedback;
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[6];
    aethor_app_integrated_build_evidence(timestamp_us, 6U, &evidence);
    if (request_id == 0U || application_coordinate_pair.state != 0U ||
        application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD || !evidence.lcd_idle || !evidence.can_idle ||
        application_motor_runtime.discovery_active || reader->pending || timestamp_us < reader->not_before_us ||
#if AETHOR_IDLE_FEEDBACK_ENABLE
        application_motor_runtime.idle_position_read.pending ||
        timestamp_us < application_motor_runtime.idle_position_read.not_before_us ||
#endif
        discovery->observed_control_mode != 2U || discovery->observed_esc_id != 7U ||
        discovery->observed_master_id != 23U ||
        (discovery->verified_fields_mask & MOTOR_DISCOVERY_ALL_FIELDS_MASK) != MOTOR_DISCOVERY_ALL_FIELDS_MASK ||
        !motor->feedback_valid || timestamp_us < feedback->timestamp_us ||
        timestamp_us - feedback->timestamp_us > 100000ULL ||
        feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags != 0U ||
        !isfinite(feedback->position_rad) || !isfinite(feedback->velocity_rad_s) ||
        !isfinite(feedback->torque_nm) || fabsf(feedback->velocity_rad_s) > 0.05F ||
        fabsf(feedback->torque_nm) > 0.15F || !isfinite(feedback->mos_temperature_c) ||
        !isfinite(feedback->rotor_temperature_c) || feedback->mos_temperature_c >= 50.0F ||
        feedback->rotor_temperature_c >= 50.0F || !isfinite(discovery->ranges.position_max_rad) ||
        discovery->ranges.position_max_rad <= 0.0F)
    { return 0U; }
    application_coordinate_pair.request_id = request_id;
    application_coordinate_pair.started_us = timestamp_us;
    application_coordinate_pair.native_feedback_us = feedback->timestamp_us;
    application_coordinate_pair.native_position_rad = feedback->position_rad;
    application_coordinate_pair.native_quantum_rad = 2.0F * discovery->ranges.position_max_rad / 65535.0F;
    application_coordinate_pair.initial_timeouts = reader->timeout_count;
    application_coordinate_pair.initial_rejections = reader->rejected_count;
    reader->joint_index = 6U;
    reader->next_register = 0U;
    reader->seen_mask = 0U;
    reader->sample_us[0] = reader->sample_us[1] = 0ULL;
    reader->values[0] = reader->values[1] = 0.0F;
    application_coordinate_pair.state = 1U;
    application_motor_runtime.position_read_reserved = 1U;
    return 1U;
}

/** @brief Routes USB requests through the exclusive owner before either command ring. */
static ProtocolEngineStatus aethor_app_integrated_process_line(const char *line, size_t length,
    uint64_t timestamp_us, ProtocolOutputBatch *output)
{
    TextProtocolRequest request;
    TextProtocolSpan motor;
    ProtocolEngineStatus status;
    AdrcLcdOwnerState owner;
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    aethor_app_enter_task_critical();
    aethor_app_persistence_service(timestamp_us);
    if (text_protocol_parse_request(line, length, &request) == TEXT_PROTOCOL_STATUS_OK &&
        aethor_app_persistence_wire_route(&request, timestamp_us, output, &status)) {
        aethor_app_exit_task_critical(); return status;
    }
    if (aethor_app_persistence_locked()) {
        /* Guard the parse-failure fallback too; malformed text cannot borrow the old parser. */
        if (text_protocol_parse_request(line, length, &request) != TEXT_PROTOCOL_STATUS_OK ||
            !(aethor_app_persistence_wire_readonly(&request) ||
              text_protocol_request_path_equals(&request, "stop", NULL) ||
              text_protocol_request_path_equals(&request, "arm", "stop") ||
              text_protocol_request_path_equals(&request, "bench", "stop") ||
              text_protocol_request_path_equals(&request, "adrc", "stop") ||
              text_protocol_request_path_equals(&request, "disable", NULL) ||
              text_protocol_request_path_equals(&request, "bench", "disable"))) {
            status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST, 0U,
                "code=persistence_locked");
            aethor_app_exit_task_critical(); return status;
        }
        if (!aethor_app_persistence_wire_readonly(&request)) {
            aethor_app_persistence_preempt(timestamp_us);
            aethor_app_persistence_disable_intent(aethor_app_persistence_text_stop_mask(&request));
        }
    }
    aethor_app_exit_task_critical();
#endif
    if (text_protocol_parse_request(line, length, &request) != TEXT_PROTOCOL_STATUS_OK)
    { return protocol_engine_process_text_line(&application_protocol_engine,
        line, length, timestamp_us, output); }
    aethor_app_enter_task_critical();
    owner = application_adrc_lcd_ownership.state;
    if (aethor_app_mapping_locked())
    {
        if (text_protocol_request_path_equals(&request, "stop", NULL) ||
            text_protocol_request_path_equals(&request, "arm", "stop") ||
            text_protocol_request_path_equals(&request, "bench", "stop") ||
            text_protocol_request_path_equals(&request, "adrc", "stop") ||
            text_protocol_request_path_equals(&request, "disable", NULL) ||
            text_protocol_request_path_equals(&request, "bench", "disable"))
        { aethor_app_mapping_fail(1U, timestamp_us); }
        else if (!text_protocol_request_path_equals(&request, "adrc", "low_speed_status") &&
                 !text_protocol_request_path_equals(&request, "adrc", "restore_speed") &&
                 !text_protocol_request_path_equals(&request, "adrc", "position_range_status") &&
                 !text_protocol_request_path_equals(&request, "adrc", "restore_position") &&
                 !text_protocol_request_path_equals(&request, "adrc", "restore_rated70_position") &&
                 !text_protocol_request_path_equals(&request, "adrc", "gate") &&
                 !text_protocol_request_path_equals(&request, "adrc", "status") &&
                 !text_protocol_request_path_equals(&request, "adrc", "shaft_status") &&
                 !text_protocol_request_path_equals(&request, "ping", NULL))
        {
            status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                request.request_id, "code=mapping_locked");
            aethor_app_exit_task_critical();
            return status;
        }
    }
    if (application_coordinate_pair.state == 1U)
    {
        if (text_protocol_request_path_equals(&request, "stop", NULL) ||
            text_protocol_request_path_equals(&request, "arm", "stop") ||
            text_protocol_request_path_equals(&request, "bench", "stop") ||
            text_protocol_request_path_equals(&request, "adrc", "stop") ||
            text_protocol_request_path_equals(&request, "disable", NULL) ||
            text_protocol_request_path_equals(&request, "bench", "disable"))
        { aethor_app_coordinate_cancel(1U, timestamp_us); }
        else if (!text_protocol_request_path_equals(&request, "adrc", "coordinates_status") &&
                 !text_protocol_request_path_equals(&request, "adrc", "gate") &&
                 !text_protocol_request_path_equals(&request, "adrc", "status") &&
                 !text_protocol_request_path_equals(&request, "adrc", "shaft_status") &&
                 !text_protocol_request_path_equals(&request, "ping", NULL))
        {
            status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                request.request_id, "code=coordinates_busy");
            aethor_app_exit_task_critical();
            return status;
        }
    }
    if (text_protocol_request_path_equals(&request, "adrc", "low_speed") ||
        text_protocol_request_path_equals(&request, "adrc", "restore_speed") ||
        text_protocol_request_path_equals(&request, "adrc", "wide_position") ||
        text_protocol_request_path_equals(&request, "adrc", "restore_position") ||
        text_protocol_request_path_equals(&request, "adrc", "rated70_position") ||
        text_protocol_request_path_equals(&request, "adrc", "restore_rated70_position"))
    {
        uint8_t rated70_mapping = (uint8_t)(text_protocol_request_path_equals(&request, "adrc", "rated70_position") ||
            text_protocol_request_path_equals(&request, "adrc", "restore_rated70_position"));
        uint8_t position_mapping = (uint8_t)(rated70_mapping ||
            text_protocol_request_path_equals(&request, "adrc", "wide_position") ||
            text_protocol_request_path_equals(&request, "adrc", "restore_position"));
        uint8_t restore = (uint8_t)(text_protocol_request_path_equals(&request, "adrc", "restore_speed") ||
            text_protocol_request_path_equals(&request, "adrc", "restore_position") ||
            text_protocol_request_path_equals(&request, "adrc", "restore_rated70_position"));
        AethorMappingFamily family = rated70_mapping ? AETHOR_MAPPING_FAMILY_RATED70_POSITION :
            position_mapping ? AETHOR_MAPPING_FAMILY_WIDE_POSITION : AETHOR_MAPPING_FAMILY_VELOCITY;
        const char *accepted = rated70_mapping ? (restore ? "restore_rated70_position accepted=1 qualified=0 control=0" :
            "rated70_position accepted=1 qualified=0 control=0") :
            position_mapping ? (restore ? "restore_position accepted=1 qualified=0 control=0" :
            "wide_position accepted=1 qualified=0 control=0") : "low_speed accepted=1 qualified=0 control=0";
        if (!request.has_request_id || request.positional_count || request.field_count != 1U ||
            !text_protocol_find_field(&request, "motor", &motor) || motor.length != 1U || motor.data[0] != '7' ||
            !aethor_app_mapping_start(request.request_id, restore, family, timestamp_us))
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, position_mapping ? "position_range code=not_ready_or_argument" : "low_speed code=not_ready_or_argument"); }
        else { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK,
            request.request_id, accepted); }
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "position_range_status"))
    {
        char detail[256];
        const S3519Ranges *ranges = &application_motor_runtime.discovery.results[6].ranges;
        /* Preserve the shared transaction identity so PMAX consumers reject VMAX state, including failures. */
        unsigned register_address = application_low_speed_mapping.state == AETHOR_MAPPING_IDLE ?
            (unsigned)S3519_REGISTER_POSITION_RANGE : (unsigned)application_low_speed_mapping.register_address;
        int written = snprintf(detail, sizeof(detail),
            "position_range_status id=%lu state=%u reason=%u locked=%u target=%.9g source=%.9g writes=%u pmax=%.9g vmax=%.9g tmax=%.9g register=%u qualified=0 control=0 tx_proven=0",
            (unsigned long)application_low_speed_mapping.request_id, (unsigned)application_low_speed_mapping.state,
            (unsigned)application_low_speed_mapping.reason, (unsigned)aethor_app_mapping_locked(),
            (double)application_low_speed_mapping.target_range, (double)application_low_speed_mapping.source_range,
            (unsigned)application_low_speed_mapping.writes, (double)ranges->position_max_rad,
            (double)ranges->velocity_max_rad_s, (double)ranges->torque_max_nm, register_address);
        status = request.positional_count || request.field_count ? PROTOCOL_ENGINE_STATUS_BAD_REQUEST :
            written < 0 || (size_t)written >= sizeof(detail) ? PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL :
            aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK, request.request_id, detail);
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "low_speed_status"))
    {
        char detail[220];
        int written = snprintf(detail, sizeof(detail),
            "low_speed_status id=%lu state=%u reason=%u locked=%u target=%.9g source=%.9g writes=%u vmax=%.9g qualified=0 control=0 tx_proven=0",
            (unsigned long)application_low_speed_mapping.request_id, (unsigned)application_low_speed_mapping.state,
            (unsigned)application_low_speed_mapping.reason, (unsigned)aethor_app_mapping_locked(),
            (double)application_low_speed_mapping.target_range, (double)application_low_speed_mapping.source_range,
            (unsigned)application_low_speed_mapping.writes,
            (double)application_motor_runtime.discovery.results[6].ranges.velocity_max_rad_s);
        status = request.positional_count || request.field_count ? PROTOCOL_ENGINE_STATUS_BAD_REQUEST :
            written < 0 || (size_t)written >= sizeof(detail) ? PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL :
            aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK, request.request_id, detail);
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "coordinates"))
    {
        if (!request.has_request_id || request.positional_count != 0U || request.field_count != 1U ||
            !text_protocol_find_field(&request, "motor", &motor) || motor.length != 1U || motor.data[0] != '7' ||
            !aethor_app_coordinate_start(request.request_id, timestamp_us))
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, "coordinates code=not_ready_or_argument"); }
        else { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK,
            request.request_id, "coordinates accepted=1 motor=7 qualified=0 control=0"); }
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "coordinates_status"))
    {
        char detail[440];
        const MotorRegisterPosition *reader = &application_motor_runtime.position_read;
        int written;
        if (request.positional_count != 0U || request.field_count != 0U)
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, "coordinates_status code=bad_argument"); }
        else
        {
            written = snprintf(detail, sizeof(detail),
                "coordinates_status id=%lu state=%u reason=%u seen=%u native_p=%.9g native_rx=%llu p_m=%.9g pm_rx=%llu xout=%.9g xout_rx=%llu qnative=%.9g qualified=0 control=0 sampling_unknown=1 tx_proven=0",
                (unsigned long)application_coordinate_pair.request_id, (unsigned)application_coordinate_pair.state,
                (unsigned)application_coordinate_pair.reason, (unsigned)reader->seen_mask,
                (double)application_coordinate_pair.native_position_rad,
                (unsigned long long)application_coordinate_pair.native_feedback_us,
                (double)reader->values[0], (unsigned long long)reader->sample_us[0],
                (double)reader->values[1], (unsigned long long)reader->sample_us[1],
                (double)application_coordinate_pair.native_quantum_rad);
            status = written < 0 || (size_t)written >= sizeof(detail) ? PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL :
                aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK, request.request_id, detail);
        }
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "acquire"))
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
                application_integrated_acquire_disable_transmitted = 0U;
                application_integrated_acquire_neutral_submitted = 0U;
                application_integrated_acquire_disable_retries = 0U;
                application_integrated_acquire_rx_count = 0U;
                application_integrated_acquire_first_submit_us = 0ULL;
                application_integrated_acquire_other_count = 0U;
                application_integrated_acquire_unknown_time_count = 0U;
                application_integrated_acquire_old_count = 0U;
                memset(application_integrated_acquire_phases, 0, sizeof(application_integrated_acquire_phases));
                application_integrated_acquire_unassigned_count = 0U;
                application_integrated_acquire_valid_count = 0U;
                application_integrated_acquire_rejected_count = 0U;
                application_integrated_acquire_last_rx_id = 0U;
                application_integrated_acquire_last_rx_length = 0U;
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
                application_adrc_bridge.bench.gateway.highest_admitted_request_id &&
            (!application_adrc_bridge.simple_request_id ||
             (application_adrc_bridge.simple.status.safe_release && application_adrc_bridge.simple_guard_ack)) &&
            adrc_shaft_adapter_release_ready(&application_adrc_bridge) &&
            (application_adrc_bridge.guard_test.state == 0U ||
             application_adrc_bridge.guard_test.state == 5U) &&
            (application_adrc_bridge.watchdog.state == 0U ||
             application_adrc_bridge.watchdog.state == 15U ||
             application_adrc_bridge.watchdog.state == 17U))
        { admission = adrc_lcd_ownership_submit_release(&application_adrc_lcd_ownership,
            request.request_id, timestamp_us); }
        if (admission == ADRC_LCD_OWNERSHIP_OK)
        {
            application_integrated_handoff = ADRC_LCD_OWNERSHIP_WAITING;
            application_adrc_bridge.bench.gateway.highest_admitted_request_id = request.request_id;
            application_integrated_release_last_submit_us = 0ULL;
            application_integrated_release_disable_retries = 0U;
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
        TextProtocolSpan query_length_span;
        if (request.positional_count == 0U && request.field_count == 0U)
        {
            MotorFeedbackSnapshot snapshot;
            const MotorJointFeedback *feedback;
            const char *probe_state = "idle";
            uint64_t feedback_age_us = UINT64_MAX;
            uint8_t after_transmit;
            char detail[320];
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
                "probe motor=%u state=%s tx=%u sample_after_tx=%u feedback_state=%u fault=%lu age_us=%llu owner=%s query_len=%u rx_after_tx=%lu rx_valid=%lu rx_rejected=%lu last_rx_id=%u last_rx_len=%u last_rx_b0=%02x last_rx_b2=%02x",
                (unsigned int)DEBUG_UI_INITIAL_MOTOR_ID, probe_state,
                (unsigned int)application_integrated_probe_transmitted,
                (unsigned int)after_transmit, (unsigned int)feedback->driver_state,
                (unsigned long)feedback->fault_flags,
                (unsigned long long)feedback_age_us,
                aethor_app_integrated_owner_name(owner),
                (unsigned int)application_integrated_diag_query_length,
                (unsigned long)application_integrated_diag_rx_after_tx,
                (unsigned long)application_integrated_diag_rx_valid,
                (unsigned long)application_integrated_diag_rx_rejected,
                (unsigned int)application_integrated_diag_last_rx_id,
                (unsigned int)application_integrated_diag_last_rx_length,
                (unsigned int)application_integrated_diag_last_rx_data0,
                (unsigned int)application_integrated_diag_last_rx_data2);
            status = written < 0 || (size_t)written >= sizeof(detail) ?
                PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL :
                aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK,
                    request.request_id, detail);
        }
        else if (request.has_request_id == 0U || request.positional_count != 0U ||
            (request.field_count != 1U && request.field_count != 2U) ||
            !text_protocol_find_field(&request, "motor", &motor) ||
            motor.length != 1U ||
            motor.data[0] != (char)('0' + DEBUG_UI_INITIAL_MOTOR_ID) ||
            (request.field_count == 2U &&
             (!text_protocol_find_field(&request, "len", &query_length_span) ||
              query_length_span.length != 1U ||
              (query_length_span.data[0] != '4' && query_length_span.data[0] != '8'))))
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
                application_integrated_diag_query_length = request.field_count == 2U ?
                    (uint8_t)(query_length_span.data[0] - '0') : 4U;
                application_integrated_diag_rx_after_tx = 0U;
                application_integrated_diag_rx_valid = 0U;
                application_integrated_diag_rx_rejected = 0U;
                application_integrated_diag_last_rx_id = 0U;
                application_integrated_diag_last_rx_length = 0U;
                application_integrated_diag_last_rx_data0 = 0U;
                application_integrated_diag_last_rx_data2 = 0U;
                application_integrated_probe_submitted_us = 0ULL;
                application_integrated_probe_transmitted = 0U;
                application_integrated_probe_failed = 0U;
                application_integrated_acquire_disable_transmitted = 0U;
                application_integrated_acquire_neutral_submitted = 0U;
                status = aethor_app_integrated_response(output,
                    PROTOCOL_ENGINE_STATUS_OK, request.request_id, "probe=accepted");
            }
        }
    }
    else if (text_protocol_request_path_equals(&request, "adrc", "acquisition"))
    {
        if (request.positional_count != 0U || request.field_count != 1U ||
            !text_protocol_find_field(&request, "motor", &motor) ||
            motor.length != 1U || motor.data[0] != (char)('0' + DEBUG_UI_INITIAL_MOTOR_ID))
        { status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            request.request_id, "code=bad_argument"); }
        else
        {
            const AethorAcquireRxPhase *disable_phase = &application_integrated_acquire_phases[0];
            const AethorAcquireRxPhase *neutral_phase = &application_integrated_acquire_phases[1];
            char detail[480];
            int written = snprintf(detail, sizeof(detail),
                "acquisition motor=%u basis=rx_lower_bound disable_us=%llu neutral_us=%llu total=%lu valid=%lu rejected=%lu other=%lu unknown=%lu old=%lu unassigned=%lu d_rx=%lu d_valid=%lu d_other=%lu n_rx=%lu n_valid=%lu n_other=%lu",
                (unsigned int)DEBUG_UI_INITIAL_MOTOR_ID,
                (unsigned long long)disable_phase->submitted_us,
                (unsigned long long)neutral_phase->submitted_us,
                (unsigned long)application_integrated_acquire_rx_count,
                (unsigned long)application_integrated_acquire_valid_count,
                (unsigned long)application_integrated_acquire_rejected_count,
                (unsigned long)application_integrated_acquire_other_count,
                (unsigned long)application_integrated_acquire_unknown_time_count,
                (unsigned long)application_integrated_acquire_old_count,
                (unsigned long)application_integrated_acquire_unassigned_count,
                (unsigned long)disable_phase->raw_count,
                (unsigned long)disable_phase->target_valid_count,
                (unsigned long)disable_phase->other_count,
                (unsigned long)neutral_phase->raw_count,
                (unsigned long)neutral_phase->target_valid_count,
                (unsigned long)neutral_phase->other_count);
            status = written < 0 || (size_t)written >= sizeof(detail) ?
                PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL :
                aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK,
                    request.request_id, detail);
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
            const MotorFeedbackTiming *feedback_timing =
                &application_motor_runtime.feedback_timing[DEBUG_UI_INITIAL_MOTOR_ID - 1U];
            char detail[500];
            int written;
            aethor_app_integrated_build_evidence(timestamp_us,
                DEBUG_UI_INITIAL_MOTOR_ID - 1U, &evidence);
            written = snprintf(detail, sizeof(detail),
                "gate motor=%u lcd_idle=%u can_idle=%u mit_ready=%u mode=%lu fields=%04x verified=%02x fb_fresh=%u disabled=%u no_fault=%u stationary=%u authority=%u action=%u ui_busy=%u ui_results=%u proto_results=%u discovery_active=%u active_samples=%lu active_intervals=%lu active_min_us=%lu active_max_us=%lu acq_submit=%u acq_tx=%u acq_failed=%u acq_disable_tx=%u acq_neutral=%u acq_rx=%lu acq_valid=%lu acq_rejected=%lu acq_last_id=%u acq_last_len=%u",
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
                (unsigned int)application_motor_runtime.discovery_active,
                (unsigned long)feedback_timing->sample_count,
                (unsigned long)feedback_timing->interval_count,
                (unsigned long)feedback_timing->minimum_interval_us,
                (unsigned long)feedback_timing->maximum_interval_us,
                (unsigned int)(application_integrated_probe_submitted_us != 0ULL),
                (unsigned int)application_integrated_probe_transmitted,
                (unsigned int)application_integrated_probe_failed,
                (unsigned int)application_integrated_acquire_disable_transmitted,
                (unsigned int)application_integrated_acquire_neutral_submitted,
                (unsigned long)application_integrated_acquire_rx_count,
                (unsigned long)application_integrated_acquire_valid_count,
                (unsigned long)application_integrated_acquire_rejected_count,
                (unsigned int)application_integrated_acquire_last_rx_id,
                (unsigned int)application_integrated_acquire_last_rx_length);
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
            text_protocol_request_path_equals(&request, "adrc", "evidence") ||
            text_protocol_request_path_equals(&request, "adrc", "capture") ||
            text_protocol_request_path_equals(&request, "adrc", "guard_status") ||
            text_protocol_request_path_equals(&request, "adrc", "watchdog_status") ||
            text_protocol_request_path_equals(&request, "adrc", "watchdog_fault") ||
            text_protocol_request_path_equals(&request, "adrc", "capabilities_status") ||
            text_protocol_request_path_equals(&request, "adrc", "capabilities_get") ||
            text_protocol_request_path_equals(&request, "adrc", "simple_status") ||
            text_protocol_request_path_equals(&request, "adrc", "simple_trace") ||
            text_protocol_request_path_equals(&request, "adrc", "shaft_status") ||
            text_protocol_request_path_equals(&request, "adrc", "shaft_binding") ||
            text_protocol_request_path_equals(&request, "adrc", "shaft_trace") ||
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
    { if (application_adrc_bridge.shaft.used && !application_adrc_bridge.shaft.released)
      { adrc_app_bridge_request_stop(&application_adrc_bridge); }
      status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        request.request_id, "code=owner_adrc"); }
    else
    { status = protocol_engine_process_text_line(&application_protocol_engine,
        line, length, timestamp_us, output); }
    aethor_app_exit_task_critical();
    return status;
}
#endif

#include "Motor/motor_idle_position_app.inc"
#include "DebugUi/debug_ui_remote_app.inc"
#include "Motor/motor_standby_recovery_app.inc"

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
#if !AETHOR_ADRC_BENCH
    {
        uint8_t matched;
        aethor_app_enter_task_critical();
        matched = aethor_app_remote_route(line, length, timestamp_us, output_batch);
        aethor_app_exit_task_critical();
        if (matched) return PROTOCOL_ENGINE_STATUS_OK;
    }
    if (aethor_app_static_trace_query(line, length, output_batch)) return PROTOCOL_ENGINE_STATUS_OK;
#endif
    aethor_app_update_protocol_context(timestamp_us);
#if !AETHOR_ADRC_BENCH
#if AETHOR_ADRC_LCD_INTEGRATED
#if AETHOR_IDLE_FEEDBACK_ENABLE
    {
        ProtocolEngineStatus status = aethor_app_integrated_process_line(line, length, timestamp_us, output_batch);
        aethor_app_enter_task_critical();
        status = aethor_app_append_idle_position(line, length, timestamp_us, status, output_batch);
        aethor_app_exit_task_critical();
        return status;
    }
#else
    return aethor_app_integrated_process_line(line, length, timestamp_us, output_batch);
#endif
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
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    /* An ordinary maintenance lock must not strand an already accepted physical STOP/DISABLE. */
    if (application_initialized && frame && aethor_app_persistence_locked() &&
        application_action.priority == CAN_TX_PRIORITY_EMERGENCY &&
        (application_action.command.type == PROTOCOL_COMMAND_STOP ||
         application_action.command.type == PROTOCOL_COMMAND_DISABLE) &&
        application_action.frame_read_index < application_action.frames.count) {
        const CanFrame *pending = &application_action.frames.frames[application_action.frame_read_index];
        if (pending->length == 8U && pending->data[7] == S3519_MODE_COMMAND_DISABLE) {
            *frame = *pending; ++application_action.frame_read_index; return 1U;
        }
    }
#endif
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_initialized && frame != NULL && application_low_speed_mapping.disable_pending)
    {
        if (s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY,
            S3519_MODE_COMMAND_DISABLE, frame) != S3519_CODEC_STATUS_OK) { return 0U; }
        application_low_speed_mapping.disable_pending = 0U;
        return 1U;
    }
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
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (aethor_app_persistence_locked()) { return 0U; }
#endif
#if AETHOR_ADRC_LCD_INTEGRATED
    if (aethor_app_mapping_locked()) { return 0U; }
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
#if AETHOR_IDLE_FEEDBACK_ENABLE
    uint8_t idle_feedback_allowed;
#endif
#if AETHOR_IDLE_FEEDBACK_ENABLE
    aethor_app_enter_task_critical();
    aethor_app_standby_service(timestamp_us);
    aethor_app_exit_task_critical();
#endif
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
#if AETHOR_IDLE_FEEDBACK_ENABLE
    aethor_app_idle_position_preempt(timestamp_us);
#endif
    aethor_app_persistence_service(timestamp_us);
    if (aethor_app_persistence_locked()) { return MOTOR_RUNTIME_STATUS_WAITING; }
#endif

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
#if AETHOR_IDLE_FEEDBACK_ENABLE
    aethor_app_idle_position_preempt(timestamp_us);
#endif
    if (aethor_app_mapping_locked() && application_low_speed_mapping.state != AETHOR_MAPPING_FAILED)
    {
        aethor_app_enter_task_critical();
        runtime_status = aethor_app_mapping_next(timestamp_us, frame, priority);
        aethor_app_exit_task_critical();
        if (runtime_status == MOTOR_RUNTIME_STATUS_FRAME_READY) { return runtime_status; }
        if (application_low_speed_mapping.state != AETHOR_MAPPING_FAILED) { return MOTOR_RUNTIME_STATUS_WAITING; }
    }
    if (application_coordinate_pair.state == 1U &&
        application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
    { aethor_app_coordinate_cancel(3U, timestamp_us); }
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
#if AETHOR_IDLE_FEEDBACK_ENABLE
    /* Background reads yield to every control/STOP/maintenance owner. They never
     * extend the 100 ms control freshness gate or generate a DISABLE for telemetry. */
    idle_feedback_allowed = (uint8_t)(!aethor_app_debug_ui_executor_busy() &&
        !application_motor_runtime.discovery_active &&
        !debug_ui_mailbox_busy(&application_debug_ui.mailbox, true) &&
        !application_controller.enabled && !application_controller.moving &&
        application_controller.fault == ARM_FAULT_NONE &&
        !application_debug_ui.unconfirmed_disable_mask &&
        !application_action.control_group_ready &&
        application_emergency_disable_read_index >= application_emergency_disable_batch.count &&
        application_debug_ui.fallback_frame_read_index >= application_debug_ui.fallback_frames.count &&
        !application_motor_runtime.position_read.pending &&
        application_transport_fault_read_sequence == application_transport_fault_write_sequence
#if AETHOR_ADRC_LCD_INTEGRATED
        && (application_integrated_diag_requested_us == 0ULL ||
            (timestamp_us >= application_integrated_diag_requested_us &&
             timestamp_us - application_integrated_diag_requested_us > ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US))
        && application_coordinate_pair.state != 1U
#endif
        );
    if (!idle_feedback_allowed) { motor_idle_feedback_pause(&application_idle_feedback, timestamp_us); }
#endif
#if AETHOR_DEBUG_UI_ENABLE && !AETHOR_DEBUG_UI_ALLOW_MOTION && \
    AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
    if (aethor_app_debug_ui_executor_busy() || application_motor_runtime.discovery_active ||
        debug_ui_mailbox_busy(&application_debug_ui.mailbox, true))
    { (void)motor_runtime_next_position_read(&application_motor_runtime,
          DEBUG_UI_INITIAL_MOTOR_ID - 1U, timestamp_us, 0U, frame); }
#endif
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_coordinate_pair.state == 1U)
    {
        aethor_app_enter_task_critical();
        runtime_status = aethor_app_coordinate_next(timestamp_us, frame);
        aethor_app_exit_task_critical();
        if (runtime_status == MOTOR_RUNTIME_STATUS_FRAME_READY)
        { *priority = CAN_TX_PRIORITY_PARAMETER; return runtime_status; }
        if (application_coordinate_pair.state == 1U) { return MOTOR_RUNTIME_STATUS_WAITING; }
    }
#endif
    if (application_debug_ui.fallback_stop_mask != 0U)
    { return MOTOR_RUNTIME_STATUS_WAITING; }

    /* HAL tick timestamps have millisecond granularity. Defer this local preflight
     * until the clock advances so a fast same-tick DISABLE reply is strictly newer
     * than the request baseline. Keep the original fresh-feedback comparison. */
    if ((application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT) &&
        (application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI) &&
        ((application_action.command.type == PROTOCOL_COMMAND_HOME_PREPARE) ||
         (application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
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
        if (application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT &&
            timestamp_us <= application_action.command.created_at_us) return MOTOR_RUNTIME_STATUS_WAITING;
        *frame = application_action.frames.frames[
            application_action.frame_read_index];
        ++application_action.frame_read_index;
        aethor_app_static_trace_record(STATIC_TRACE_DEQUEUED, 0U, frame, timestamp_us, timestamp_us, 0U, 0U, 0U);
        if (debug_ui_is_static_operation(application_action.command.local_operation) &&
            application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT)
        {
            uint8_t static_joint_index;
            for (static_joint_index = 0U; static_joint_index < ARM_JOINT_COUNT; ++static_joint_index)
            {
                if ((application_action.command.motor_mask & (1U << static_joint_index)) &&
                    frame->identifier == application_motor_runtime.configuration->joints[static_joint_index].esc_id)
                { application_debug_ui.static_load.frames_issued_mask |= (uint8_t)(1U << static_joint_index); break; }
            }
            if (application_action.frame_read_index == application_action.frames.count)
            { application_debug_ui.static_last_batch_us = timestamp_us; }
        }
        if (!((debug_ui_is_static_operation(application_action.command.local_operation) || aethor_app_is_pos_load()) &&
              application_action.static_setup_deadline_us &&
              (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT ||
               application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT)) &&
            (application_action.frame_read_index == application_action.frames.count) &&
            ((application_action.state ==
              AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT) ||
              (application_action.state ==
               AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT) ||
              (application_action.state ==
               AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT) ||
              ((application_action.state ==
                AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT) &&
               (application_action.command.control_mode !=
                ARM_CONTROL_MODE_MIT) && !aethor_app_is_pos_load()) ||
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
    if (aethor_app_pos_load_next_disable_probe(timestamp_us, frame)) {
        *priority = CAN_TX_PRIORITY_EMERGENCY;
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

#if AETHOR_IDLE_FEEDBACK_ENABLE
    aethor_app_enter_task_critical();
    if (application_standby_recovery.owned || !application_motor_runtime.discovery_active)
        runtime_status = aethor_app_standby_next(timestamp_us, frame);
    else
#endif
    runtime_status = motor_runtime_next_discovery_frame(
        &application_motor_runtime,
        timestamp_us,
        frame);
#if AETHOR_IDLE_FEEDBACK_ENABLE
    aethor_app_exit_task_critical();
#endif
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
#if AETHOR_IDLE_FEEDBACK_ENABLE
    if (idle_feedback_allowed && (runtime_status == MOTOR_RUNTIME_STATUS_WAITING ||
                                  runtime_status == MOTOR_RUNTIME_STATUS_DISCOVERY_COMPLETE ||
                                  (runtime_status == MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR &&
                                   application_motor_runtime.discovery.state == MOTOR_DISCOVERY_STATE_FAILED)))
    {
        if (aethor_app_idle_position_allowed(timestamp_us) &&
            motor_runtime_next_idle_position_read(&application_motor_runtime, 6U,
                timestamp_us, 1U, frame) == MOTOR_RUNTIME_STATUS_FRAME_READY)
        { *priority = CAN_TX_PRIORITY_FEEDBACK_QUERY; return MOTOR_RUNTIME_STATUS_FRAME_READY; }
        MotorRuntimeStatus monitor_status = motor_idle_feedback_next(&application_idle_feedback,
            &application_motor_runtime, timestamp_us, frame);
        if (monitor_status == MOTOR_RUNTIME_STATUS_FRAME_READY)
        { *priority = CAN_TX_PRIORITY_FEEDBACK_QUERY; return monitor_status; }
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
    return aethor_app_receive_can_frame_observed(frame, timestamp_us, timestamp_us);
}

/** @brief Preserves the old RX ABI while adding a distinct capability receive observation bound. */
MotorRuntimeStatus aethor_app_receive_can_frame_observed(const CanFrame *frame,
    uint64_t timestamp_us, uint64_t received_observed_us)
{
    (void)received_observed_us;
    if (application_initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    aethor_app_enter_task_critical();
    if (aethor_app_persistence_receive(frame, timestamp_us, received_observed_us, NULL, 0U, 0U))
    { aethor_app_exit_task_critical(); return MOTOR_RUNTIME_STATUS_OK; }
    aethor_app_exit_task_critical();
#endif
#if AETHOR_ADRC_BENCH
    {
        MotorRuntimeStatus status;
        uint32_t accepted_before;
        aethor_app_enter_task_critical();
        accepted_before = application_motor_runtime.accepted_feedback_count;
        status = (adrc_app_bridge_accept_capability_parameter(&application_adrc_bridge, frame, timestamp_us, received_observed_us) ||
            adrc_app_bridge_accept_watchdog_parameter(&application_adrc_bridge, frame, timestamp_us)) ?
            MOTOR_RUNTIME_STATUS_OK : motor_runtime_accept_frame(&application_motor_runtime, frame, timestamp_us);
        adrc_shaft_adapter_note_feedback(&application_adrc_bridge, accepted_before, timestamp_us, received_observed_us);
        adrc_app_bridge_check_capability_health(&application_adrc_bridge, received_observed_us);
        aethor_app_single_static_refresh_received_evidence();
        aethor_app_exit_task_critical();
        return status;
    }
#elif AETHOR_ADRC_LCD_INTEGRATED
    {
        uint32_t accepted_before = application_motor_runtime.accepted_feedback_count;
        uint32_t rejected_before = application_motor_runtime.rejected_feedback_count;
        MotorRuntimeStatus status;
        aethor_app_enter_task_critical();
        if (aethor_app_mapping_receive(frame, timestamp_us))
        { aethor_app_exit_task_critical(); return MOTOR_RUNTIME_STATUS_OK; }
        status = (
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
            /* Preserve normal feedback hooks while bypassing only the old collector's AA coincidence. */
            !(frame && frame->length == 8U && frame->data[2] == 0xaaU) &&
#endif
            (adrc_app_bridge_accept_capability_parameter(&application_adrc_bridge, frame, timestamp_us, received_observed_us) ||
             adrc_app_bridge_accept_watchdog_parameter(&application_adrc_bridge, frame, timestamp_us))) ?
            MOTOR_RUNTIME_STATUS_OK : motor_runtime_accept_frame(&application_motor_runtime, frame, timestamp_us);
        adrc_shaft_adapter_note_feedback(&application_adrc_bridge, accepted_before, timestamp_us, received_observed_us);
        adrc_app_bridge_check_capability_health(&application_adrc_bridge, received_observed_us);
        aethor_app_single_static_refresh_received_evidence();
        aethor_app_exit_task_critical();
        if (frame != NULL && application_adrc_lcd_ownership.state ==
                ADRC_LCD_OWNER_ACQUIRING &&
            application_integrated_acquire_first_submit_us != 0ULL)
        {
            aethor_app_enter_task_critical();
            ++application_integrated_acquire_rx_count;
            if (timestamp_us == 0ULL)
            { ++application_integrated_acquire_unknown_time_count; }
            else if (timestamp_us <= application_integrated_acquire_first_submit_us)
            { ++application_integrated_acquire_old_count; }
            if (frame->identifier != arm_config_get_production()->joints[
                    application_adrc_lcd_ownership.axis_index].master_id)
            { ++application_integrated_acquire_other_count; }
            else
            {
                if (application_motor_runtime.accepted_feedback_count > accepted_before)
                { ++application_integrated_acquire_valid_count; }
                if (application_motor_runtime.rejected_feedback_count > rejected_before)
                { ++application_integrated_acquire_rejected_count; }
            }
            /* A lower bound before neutral cannot prove the frame arrived before neutral.
             * Only a frame processed while neutral is not yet submitted belongs to DISABLE;
             * delayed or untimed evidence remains explicitly unassigned. */
            {
                AethorAcquireRxPhase *phase = NULL;
                if (timestamp_us > application_integrated_acquire_first_submit_us)
                {
                    if (application_integrated_acquire_phases[1].submitted_us == 0ULL)
                    { phase = &application_integrated_acquire_phases[0]; }
                    else if (timestamp_us > application_integrated_acquire_phases[1].submitted_us)
                    { phase = &application_integrated_acquire_phases[1]; }
                }
                if (phase == NULL) { ++application_integrated_acquire_unassigned_count; }
                else
                {
                    ++phase->raw_count;
                    if (frame->identifier != arm_config_get_production()->joints[
                            application_adrc_lcd_ownership.axis_index].master_id)
                    { ++phase->other_count; }
                    else if (application_motor_runtime.accepted_feedback_count > accepted_before)
                    { ++phase->target_valid_count; }
                }
            }
            application_integrated_acquire_last_rx_id = frame->identifier;
            application_integrated_acquire_last_rx_length = frame->length;
            aethor_app_exit_task_critical();
        }
        if (frame != NULL && application_integrated_diag_transmitted_us != 0ULL &&
            timestamp_us > application_integrated_diag_transmitted_us &&
            timestamp_us >= application_integrated_diag_requested_us &&
            timestamp_us - application_integrated_diag_requested_us <=
                ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US)
        {
            aethor_app_enter_task_critical();
            ++application_integrated_diag_rx_after_tx;
            if (application_motor_runtime.accepted_feedback_count > accepted_before)
            { ++application_integrated_diag_rx_valid; }
            if (application_motor_runtime.rejected_feedback_count > rejected_before)
            { ++application_integrated_diag_rx_rejected; }
            application_integrated_diag_last_rx_id = frame->identifier;
            application_integrated_diag_last_rx_length = frame->length;
            application_integrated_diag_last_rx_data0 = frame->length > 0U ?
                frame->data[0] : 0U;
            application_integrated_diag_last_rx_data2 = frame->length > 2U ?
                frame->data[2] : 0U;
            aethor_app_exit_task_critical();
        }
        return status;
    }
#else
    {
        MotorRuntimeStatus status;
        aethor_app_enter_task_critical();
        status = motor_runtime_accept_frame(&application_motor_runtime, frame, timestamp_us);
        aethor_app_single_static_refresh_received_evidence();
        aethor_app_exit_task_critical();
        return status;
    }
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

#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    aethor_app_persistence_service(timestamp_us);
    if (aethor_app_persistence_locked() && (command->type == PROTOCOL_COMMAND_STOP ||
        command->type == PROTOCOL_COMMAND_DISABLE)) {
        aethor_app_persistence_preempt(timestamp_us);
        aethor_app_persistence_disable_intent((uint8_t)(0x40U |
            (command->bench_relative_scope ? command->motor_mask : 0x7FU)));
    }
#endif

    if (command->type == PROTOCOL_COMMAND_STOP)
        aethor_app_pos_load_bind_stop(command, timestamp_us);
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
#if AETHOR_ADRC_LCD_INTEGRATED
    if ((aethor_app_mapping_locked()
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
         || aethor_app_persistence_locked()
#endif
        ) && command->type != PROTOCOL_COMMAND_STOP &&
        command->type != PROTOCOL_COMMAND_DISABLE)
    {
        result.stage = PROTOCOL_COMMAND_STAGE_VALIDATE;
        result.error = PROTOCOL_COMMAND_ERROR_NOT_READY;
        return aethor_app_submit_or_defer_result(&result);
    }
#endif
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
    else if (command->type == PROTOCOL_COMMAND_STOP &&
             application_debug_ui.pos_load.stop_identity.request_id == command->request_id &&
             application_debug_ui.pos_load.stop_identity.epoch == command->session_id &&
             application_debug_ui.pos_load.stop_identity.origin == command->origin)
    {
        application_action.control_group_ready = 0U;
        runtime_status = motor_runtime_build_mode_command_batch(&application_motor_runtime,
            S3519_CONTROL_MODE_POSITION_VELOCITY, S3519_MODE_COMMAND_DISABLE,
            command->motor_mask, &application_action.frames);
        if (runtime_status == MOTOR_RUNTIME_STATUS_OK) {
            application_action.command = *command;
            application_action.state = AETHOR_APP_ACTION_DISABLE_WAIT;
            application_action.priority = CAN_TX_PRIORITY_EMERGENCY;
            application_action.frame_read_index = 0U;
            application_action.deadline_us = timestamp_us + AETHOR_APP_ACTION_TIMEOUT_US;
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
        if (
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
            !aethor_app_persistence_locked() &&
#endif
            (snapshot_status == MOTOR_RUNTIME_STATUS_OK) &&
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
             (command->type == PROTOCOL_COMMAND_HOME_PREPARE) ||
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
#if AETHOR_DEBUG_UI_ALLOW_MOTION
    if (application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) {
        uint8_t sent = (uint8_t)(application_action.frame_read_index >= application_action.frames.count);
        DebugUiReason preflight_reason;
        if (timestamp_us >= application_action.deadline_us)
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED, 3U, timestamp_us);
        if (application_action.single_static_tx_failed) {
            application_action.local_failure_reason = DEBUG_UI_REASON_NOT_READY;
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED, 0U, timestamp_us);
        }
        preflight_reason = aethor_app_debug_ui_validate_start(&application_action.command,
            motor_snapshot, timestamp_us, sent != 0U);
        if (preflight_reason != DEBUG_UI_REASON_NONE &&
            (!sent || preflight_reason != DEBUG_UI_REASON_STALE_FEEDBACK)) {
            application_action.local_failure_reason = preflight_reason;
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED, 0U, timestamp_us);
        }
        if (!application_action.single_static_tx_confirmed || preflight_reason == DEBUG_UI_REASON_STALE_FEEDBACK) {
            if (timestamp_us >= application_action.deadline_us)
                return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED, 3U, timestamp_us);
            return 0U;
        }
    }
#endif
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
             (aethor_app_is_pos_load_stop() ? aethor_app_pos_disable_proven(motor_snapshot) :
              (((application_action.command.cleanup_after_stop == 0U) &&
               (application_action.command.origin != DEBUG_UI_ORIGIN_LOCAL_UI)) ||
              (aethor_app_selected_feedback_advanced(motor_snapshot,
                   application_action.command.motor_mask,
                   application_action.phase_feedback_baseline_us) != 0U))) &&
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
    /* A later retry must not erase the first successfully transmitted DISABLE.
     * RX still needs a known lower bound strictly after this original challenge. */
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING &&
        application_integrated_acquire_disable_transmitted != 0U)
    { evidence->probe_submitted_us = application_integrated_acquire_first_submit_us; }
    evidence->probe_transmitted = application_integrated_probe_transmitted;
    evidence->probe_failed = application_integrated_probe_failed;
    evidence->lcd_idle = (uint8_t)(!aethor_app_mapping_locked() && application_coordinate_pair.state != 1U && !aethor_app_debug_ui_executor_busy() &&
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
    /* A receipt proves transmission, not fresh driver state. Prompt RX may have
     * a conservative bound equal to the first submission; request another reply
     * without changing the causal anchor, freshness limit, or release deadline.
     * These limits bound supplemental requests after a successful TX. A failed
     * or missing receipt still follows the bridge's existing safety DISABLE
     * path; it must not stop attempting DISABLE merely to cap this counter. */
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING &&
        application_adrc_lcd_ownership.disable_transmitted_us != 0ULL &&
        application_integrated_release_last_submit_us != 0ULL &&
        timestamp_us >= application_integrated_release_last_submit_us &&
        timestamp_us - application_integrated_release_last_submit_us >= AETHOR_ADRC_RELEASE_DISABLE_RETRY_US &&
        timestamp_us >= application_adrc_lcd_ownership.release_started_us &&
        timestamp_us - application_adrc_lcd_ownership.release_started_us < ADRC_LCD_OWNER_RELEASE_TIMEOUT_US &&
        application_integrated_release_disable_retries < AETHOR_ADRC_RELEASE_DISABLE_RETRY_LIMIT &&
        application_adrc_bridge.release_disable_pending == 0U &&
        application_adrc_bridge.frame_pending == 0U && application_adrc_bridge.awaiting_receipt == 0U)
    {
        ++application_integrated_release_disable_retries;
        adrc_app_bridge_request_release_disable(&application_adrc_bridge);
    }
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
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    /* Keep the trusted Platform observation age; an old true cache cannot reserve maintenance. */
    application_persistence.can_idle_observed_us = idle ? timestamp_us : 0ULL;
#endif
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

/** @brief Monitor traffic resets CAN quiet evidence without rewriting a completed diagnostic receipt.
 * Its scheduler already excludes the active diagnostic window; ordinary control traffic
 * retains the stronger legacy invalidation path above.
 */
void aethor_app_integrated_note_idle_feedback_activity(void)
{
    aethor_app_enter_task_critical();
    application_integrated_can_quiet_since_us = 0ULL;
    application_integrated_can_idle = 0U;
    aethor_app_exit_task_critical();
}

/** @brief Issues acquisition DISABLE then zero-request MIT control, or an LCD-owned query. */
uint8_t aethor_app_integrated_pop_probe_frame(CanFrame *frame, uint64_t timestamp_us)
{
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (aethor_app_persistence_locked()) { return 0U; }
#endif
    AdrcLcdOwnershipEvidence evidence;
    uint8_t available = 0U;
    uint8_t axis;
    uint8_t acquiring;
    uint8_t neutral_due;
    uint8_t disable_retry_due;
    if (!application_initialized || frame == NULL || timestamp_us == 0ULL) { return 0U; }
    aethor_app_enter_task_critical();
    acquiring = (uint8_t)(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
    neutral_due = (uint8_t)(acquiring != 0U &&
        application_integrated_acquire_disable_transmitted != 0U &&
        application_integrated_acquire_neutral_submitted == 0U);
    disable_retry_due = (uint8_t)(acquiring != 0U &&
        application_integrated_acquire_disable_transmitted != 0U &&
        application_integrated_acquire_neutral_submitted != 0U &&
        application_integrated_probe_transmitted != 0U &&
        application_integrated_acquire_disable_retries < AETHOR_ADRC_ACQUIRE_DISABLE_RETRY_LIMIT &&
        timestamp_us >= application_integrated_probe_submitted_us &&
        timestamp_us - application_integrated_probe_submitted_us >= AETHOR_ADRC_ACQUIRE_DISABLE_RETRY_US);
    /* Keep the original half-open acquisition deadline even if service is delayed. */
    if (acquiring != 0U && (application_adrc_lcd_ownership.acquire_started_us == 0ULL ||
        timestamp_us < application_adrc_lcd_ownership.acquire_started_us ||
        timestamp_us - application_adrc_lcd_ownership.acquire_started_us >= ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US))
    { aethor_app_exit_task_critical(); return 0U; }
    if ((acquiring != 0U ||
        (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD &&
         application_integrated_diag_requested_us != 0ULL &&
         timestamp_us >= application_integrated_diag_requested_us &&
         timestamp_us - application_integrated_diag_requested_us <=
            ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US)) &&
        (application_integrated_probe_submitted_us == 0ULL || neutral_due != 0U || disable_retry_due != 0U) &&
        application_integrated_probe_failed == 0U)
    {
        axis = acquiring != 0U ?
            application_adrc_lcd_ownership.axis_index :
            (uint8_t)(DEBUG_UI_INITIAL_MOTOR_ID - 1U);
        aethor_app_integrated_build_evidence(timestamp_us,
            axis, &evidence);
        if (evidence.lcd_idle != 0U && evidence.can_idle != 0U &&
            evidence.mit_discovered != 0U)
        {
            uint8_t packed = 0U;
            if (acquiring != 0U && neutral_due == 0U)
            {
                MotorEmergencyFrameBatch disable_batch;
                if (motor_runtime_build_mode_command_batch(&application_motor_runtime,
                        S3519_CONTROL_MODE_MIT, S3519_MODE_COMMAND_DISABLE,
                        (uint8_t)(1U << axis), &disable_batch) == MOTOR_RUNTIME_STATUS_OK &&
                    disable_batch.count == 1U)
                {
                    *frame = disable_batch.frames[0];
                    packed = 1U;
                }
            }
            else if (neutral_due != 0U)
            {
                if (motor_adrc_build_neutral_probe(&application_motor_runtime,
                        axis, frame) == MOTOR_RUNTIME_STATUS_OK)
                { packed = 1U; }
            }
            else if (acquiring == 0U &&
                motor_adrc_build_feedback_query(&application_motor_runtime, axis,
                    application_integrated_diag_query_length, frame) == MOTOR_RUNTIME_STATUS_OK)
            { packed = 1U; }
            if (packed != 0U)
            {
                application_integrated_probe_submitted_us = timestamp_us;
                if (acquiring != 0U && disable_retry_due == 0U)
                { application_integrated_acquire_phases[neutral_due != 0U ? 1U : 0U].submitted_us = timestamp_us; }
                if (acquiring != 0U && application_integrated_acquire_first_submit_us == 0ULL)
                { application_integrated_acquire_first_submit_us = timestamp_us; }
                if (neutral_due != 0U)
                {
                    application_integrated_probe_transmitted = 0U;
                    application_integrated_acquire_neutral_submitted = 1U;
                }
                if (disable_retry_due != 0U)
                {
                    ++application_integrated_acquire_disable_retries;
                    application_integrated_probe_transmitted = 0U;
                }
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
            if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING &&
                application_integrated_acquire_neutral_submitted == 0U)
            { application_integrated_acquire_disable_transmitted = 1U; }
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
#if AETHOR_IDLE_FEEDBACK_ENABLE
    /* Retire the background lease before a queued foreground lifecycle can replace discovery. */
    aethor_app_enter_task_critical();
    aethor_app_standby_service(timestamp_us);
    aethor_app_exit_task_critical();
#endif
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    aethor_app_enter_task_critical();
    aethor_app_persistence_service(timestamp_us);
    aethor_app_exit_task_critical();
#endif
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
        aethor_app_torque_envelope_background(&motor_snapshot, timestamp_us);
        if (snapshot_status == MOTOR_RUNTIME_STATUS_OK)
        {
            (void)joint_reference_publish(&application_joint_reference,
                                          &motor_snapshot,
                                          timestamp_us);
        }
        aethor_app_static_trace_emergency_progress(&motor_snapshot, timestamp_us);
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
                aethor_app_static_trace_emergency_begin(timestamp_us);
                aethor_app_widen_pending_stop_before_global_cancel();
                (void)motor_runtime_abort_active_parameter_sequences(
                    &application_motor_runtime,
                    timestamp_us);
                if (application_action.state != AETHOR_APP_ACTION_IDLE)
                {
                    if ((runtime_fault == ARM_FAULT_CONTROL_DEADLINE) &&
                        ((application_action.command.type == PROTOCOL_COMMAND_HOME_PREPARE) ||
                         (application_action.command.type ==
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
        aethor_app_static_load_sample(&motor_snapshot, timestamp_us);
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
                aethor_app_static_load_bind_stop(&stop_command);
                aethor_app_pos_load_bind_stop(&stop_command, timestamp_us);
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

#include "DebugUi/debug_ui_static_transport_app.inc"

/** @brief Match exact frame bytes and the active preflight identity before using the dedicated receipt path. */
uint8_t aethor_app_debug_ui_preflight_frame_identity(const CanFrame *frame, DebugUiIdentity *identity)
{
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    if (frame == NULL || identity == NULL || application_action.state != AETHOR_APP_ACTION_DISABLE_WAIT ||
        application_action.command.local_operation != DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT ||
        application_action.frames.count != 1U || frame->identifier != application_action.frames.frames[0].identifier ||
        frame->length != 8U || memcmp(frame->data, application_action.frames.frames[0].data, 8U)) return 0U;
    identity->origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    identity->epoch = application_action.command.session_id;
    identity->request_id = application_action.command.request_id;
    application_debug_ui.single_static_transport_identity = *identity;
    application_debug_ui.static_transport_identity.epoch = identity->epoch;
    application_debug_ui.static_transport_identity.request_id = identity->request_id;
    application_debug_ui.static_transport_identity.sequence = 1U;
    application_debug_ui.static_transport_identity.motor_id = (uint8_t)frame->identifier;
    application_debug_ui.static_transport_identity.phase = DEBUG_UI_STATIC_TX_PREFLIGHT;
    application_debug_ui.single_static_transport_mask = application_action.command.motor_mask;
    return 1U;
#else
    (void)frame; (void)identity;
    return 0U;
#endif
}

/** @brief Capture per-ID sequence before HAL submission; equal old sample timestamps cannot qualify. */
uint8_t aethor_app_debug_ui_preflight_submitted(uint32_t epoch, uint32_t request_id, uint64_t rx_boundary_us)
{
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    uint8_t joint_index;
    if (application_action.command.local_operation != DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT ||
        application_action.command.session_id != epoch || application_action.command.request_id != request_id ||
        application_action.single_static_tx_submitted || rx_boundary_us <= application_action.command.created_at_us ||
        aethor_app_single_static_preflight_validate(&application_action.command, NULL, rx_boundary_us, false) != DEBUG_UI_REASON_NONE)
        return 0U;
    application_action.single_static_tx_submitted = 1U;
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
        if (application_action.command.motor_mask & (1U << joint_index)) {
            application_action.phase_feedback_baseline_us[joint_index] = rx_boundary_us - 1ULL;
            application_action.single_static_rx_generation = application_motor_runtime.bank.motors[joint_index].feedback_generation;
        }
    return 1U;
#else
    (void)epoch; (void)request_id; (void)rx_boundary_us;
    return 0U;
#endif
}

/** @brief Require the same submitted epoch/request and a real hardware completion, never mailbox admission. */
void aethor_app_debug_ui_preflight_transmitted(uint32_t epoch, uint32_t request_id, uint8_t succeeded)
{
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    if (application_debug_ui.single_static_transport_identity.epoch == epoch &&
        application_debug_ui.single_static_transport_identity.request_id == request_id)
        application_debug_ui.single_static_transport_mask = 0U;
    if (application_action.command.local_operation != DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT ||
        application_action.command.session_id != epoch || application_action.command.request_id != request_id ||
        application_action.single_static_tx_confirmed || application_action.single_static_tx_failed) return;
    if (!succeeded || !application_action.single_static_tx_submitted) application_action.single_static_tx_failed = 1U;
    else application_action.single_static_tx_confirmed = 1U;
#else
    (void)epoch; (void)request_id; (void)succeeded;
#endif
}

/** @brief Expose the original constant to the timestamp-ambiguity gate; no second timeout is introduced. */
uint64_t aethor_app_debug_ui_preflight_timeout_us(void)
{
    return AETHOR_APP_ACTION_TIMEOUT_US;
}

/** @brief A matching hardware order tag cannot qualify a rejected, different-ID or previously cached feedback sample. */
MotorRuntimeStatus aethor_app_debug_ui_receive_preflight_frame(const CanFrame *frame,
    uint64_t arrival_lower_us, uint64_t observed_us, const DebugUiIdentity *ordered_identity)
{
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    uint8_t joint_index, selected_index = 0U;
    uint32_t generation_before;
    MotorRuntimeStatus status;
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
        if (application_action.command.motor_mask & (1U << joint_index)) selected_index = joint_index;
    generation_before = application_motor_runtime.bank.motors[selected_index].feedback_generation;
    status = aethor_app_receive_can_frame_observed(frame, arrival_lower_us, observed_us);
    if (status == MOTOR_RUNTIME_STATUS_OK && ordered_identity &&
        ordered_identity->origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
        application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT &&
        application_action.command.session_id == ordered_identity->epoch &&
        application_action.command.request_id == ordered_identity->request_id &&
        application_action.single_static_tx_submitted && !application_action.single_static_tx_failed &&
        application_motor_runtime.bank.motors[selected_index].feedback_generation != generation_before)
        application_action.single_static_ordered_generation = application_motor_runtime.bank.motors[selected_index].feedback_generation;
    return status;
#else
    (void)ordered_identity;
    return aethor_app_receive_can_frame_observed(frame, arrival_lower_us, observed_us);
#endif
}

/** @brief Advances epoch without making zero a valid authorization. Caller holds critical. */
/** @brief Register immutable platform evidence providers; unknown/unbound evidence keeps fault recovery closed. */
void aethor_app_bind_remote_release_probes(uint8_t (*can_idle)(void), uint32_t (*usb_link_word)(void))
{
    aethor_app_enter_task_critical();
    if (!application_remote_release_can_idle && !application_remote_release_link_word && can_idle && usb_link_word) {
        application_remote_release_can_idle = can_idle;
        application_remote_release_link_word = usb_link_word;
        protocol_engine_bind_ui_link_probe(&application_protocol_engine, usb_link_word);
    }
    aethor_app_exit_task_critical();
}

static void aethor_app_debug_ui_revoke(DebugUiAuthority authority)
{
#if AETHOR_IDLE_FEEDBACK_ENABLE
    /* A new control epoch cannot reuse the previous display derivative baseline. */
    application_motor_runtime.idle_position_read.seen_mask = 0U;
    application_motor_runtime.idle_position_read.estimated_velocity_valid = 0U;
#endif
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_CONTEXT, application_persistence.last_us);
#endif
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
        aethor_app_static_load_bind_fallback(timestamp_us);
        aethor_app_pos_load_bind_fallback(timestamp_us);
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
    aethor_app_static_load_confirm_fallback(&snapshot, timestamp_us);
    aethor_app_pos_load_confirm_fallback(&snapshot, timestamp_us);
    if ((application_action.state == AETHOR_APP_ACTION_IDLE) &&
        (application_protocol_engine.command_read_sequence == application_protocol_engine.command_write_sequence) &&
        (application_protocol_engine.stop_read_sequence == application_protocol_engine.stop_write_sequence) &&
        (application_debug_ui.fallback_frame_read_index >= application_debug_ui.fallback_frames.count) &&
        ((aethor_app_debug_ui_recovery_reason(&snapshot) == DEBUG_UI_REASON_NONE) ||
         (timestamp_us >= application_debug_ui.fallback_deadline_us)))
    { application_debug_ui.fallback_stop_mask = 0U; application_debug_ui.static_fallback_bound = 0U; }
    aethor_app_exit_task_critical();
    return 1U;
#endif
}
#endif

/** @brief Reports actual executor and published queues, including source-neutral STOP. */
static bool aethor_app_debug_ui_executor_busy(void)
{
    return
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
        aethor_app_persistence_locked() ||
#endif
        (application_action.state != AETHOR_APP_ACTION_IDLE) ||
        (application_debug_ui.pos_disable_pending != 0U) ||
        (application_debug_ui.single_static_transport_mask != 0U) ||
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
#if AETHOR_IDLE_FEEDBACK_ENABLE
        if (joint_index == 6U && (application_standby_recovery.owned ||
            application_standby_recovery.lost_reference)) continue;
#endif
        DebugUiMotorProfile *current = &application_debug_ui.profiles[joint_index];
        if ((AETHOR_DEBUG_UI_S3519_PROFILE_MASK & (1U << joint_index)) == 0U) { continue; }
        memset(&profile, 0, sizeof(profile));
        if ((motor_runtime_get_position_velocity_limits(&application_motor_runtime, joint_index, &limits) ==
             MOTOR_RUNTIME_STATUS_OK) && isfinite(2.0F * limits.position_max_rad))
        {
            float ordinary_position_max = limits.position_max_rad;
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
            /* Enlarging driver encoding must not enlarge ordinary ID7 POS/MIT travel. */
            if (joint_index == 6U) ordinary_position_max = fminf(ordinary_position_max, 12.5F);
#endif
            profile.pos_valid = 1U;
            profile.position_min_rad = -ordinary_position_max;
            profile.position_max_rad = ordinary_position_max;
            profile.max_delta_rad = 2.0F * ordinary_position_max;
            profile.max_speed_rad_s = limits.move_speed_limit_rad_s;
#if AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE
            profile.mit_valid = 1U;
            profile.mit_position_min_rad = -ordinary_position_max;
            profile.mit_position_max_rad = ordinary_position_max;
            profile.mit_max_delta_rad = 2.0F * ordinary_position_max;
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
        aethor_app_single_static_refresh_evidence(joint_index);
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
#if AETHOR_IDLE_FEEDBACK_ENABLE
        if (joint_index == 6U && (application_standby_recovery.owned ||
            application_standby_recovery.lost_reference)) continue;
#endif
        const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        uint32_t signature = discovery->observed_control_mode |
            ((uint32_t)discovery->verified_fields_mask << 8U);
        uint32_t coordinate_identity = (uint32_t)discovery->observed_esc_id |
            ((uint32_t)discovery->observed_master_id << 16U);
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
        if (joint_index == 6U) {
            uint32_t position_max_bits;
            memcpy(&position_max_bits, &discovery->ranges.position_max_rad, sizeof(position_max_bits));
            if (position_max_bits != application_debug_ui.pos_load_observed_pmax_bits) {
                application_debug_ui.pos_load_observed_pmax_bits = position_max_bits;
                ++application_debug_ui.reference_generation[joint_index];
            }
        }
#endif
        if ((application_debug_ui.reference_generation[joint_index] == 0U) ||
            (application_debug_ui.observed_mode[joint_index] != signature) ||
            (application_debug_ui.observed_coordinate_identity[joint_index] != coordinate_identity))
        {
            application_debug_ui.observed_mode[joint_index] = signature;
            application_debug_ui.observed_coordinate_identity[joint_index] = coordinate_identity;
            ++application_debug_ui.reference_generation[joint_index];
        }
        aethor_app_single_static_refresh_evidence(joint_index);
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
        aethor_app_single_static_refresh_evidence((uint8_t)(motor_id - 1U));
        installed = true;
    }
    aethor_app_exit_task_critical();
    return installed;
}

/** @brief Install one RAM profile after the caller has enforced its own result reservation rules. */
static void aethor_app_single_static_install(uint8_t joint_index,
    const DebugUiSingleStaticProfile *profile, uint8_t lcd_owned)
{
    ++application_debug_ui.reference_generation[joint_index];
    aethor_app_single_static_refresh_evidence(joint_index);
    application_debug_ui.single_static_profiles[joint_index] = *profile;
    memset(&application_debug_ui.single_static_gains[joint_index], 0, sizeof(DebugUiStaticGains));
    if (!profile->configured) application_debug_ui.single_static_profiles[joint_index].torque_ff_nm = 0.0F;
    application_debug_ui.single_static_reference[joint_index] =
        application_debug_ui.reference_generation[joint_index];
    application_debug_ui.single_static_discovery[joint_index] =
        application_motor_runtime.discovery.results[joint_index];
    application_debug_ui.single_static_lcd_owned[joint_index] = lcd_owned;
}

/** @brief Install a separate bounded long-HOLD configuration against current reference evidence. */
bool aethor_app_debug_ui_set_single_static_profile(uint8_t motor_id,
    const DebugUiSingleStaticProfile *profile)
{
    bool installed = false;
    uint8_t joint_index;
    if (!application_initialized || profile == NULL || motor_id == 0U ||
        motor_id > DEBUG_UI_MOTOR_COUNT || profile->configured > 1U ||
        !isfinite(profile->torque_ff_nm) || fabsf(profile->torque_ff_nm) > 3.5F ||
        (profile->configured && (profile->minimum_ms < DEBUG_UI_SINGLE_STATIC_MIN_MS ||
         profile->maximum_ms > DEBUG_UI_SINGLE_STATIC_MAX_MS || profile->maximum_ms < profile->minimum_ms)))
    { return false; }
    joint_index = (uint8_t)(motor_id - 1U);
    aethor_app_enter_task_critical();
    aethor_app_debug_ui_refresh_reference(0ULL);
    if (application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE &&
        !aethor_app_debug_ui_executor_busy() &&
        debug_ui_mailbox_result_count(&application_debug_ui.mailbox) == 0U &&
        (!profile->configured || application_debug_ui.profiles[joint_index].mit_valid))
    {
        aethor_app_single_static_install(joint_index, profile, 0U);
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
#if !AETHOR_ADRC_BENCH
    aethor_app_enter_task_critical();
    reason = aethor_app_remote_local_gate(request);
    aethor_app_exit_task_critical();
    if (reason != DEBUG_UI_REASON_NONE) return reason;
#endif
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    aethor_app_enter_task_critical();
    if (aethor_app_persistence_locked()) {
        if (!request || request->operation != DEBUG_UI_OPERATION_STOP) {
            aethor_app_exit_task_critical(); return DEBUG_UI_REASON_DISABLED;
        }
        aethor_app_persistence_preempt(application_persistence.last_us);
        aethor_app_persistence_disable_intent(aethor_app_persistence_local_stop_mask(request->target_motor_id));
    }
    aethor_app_exit_task_critical();
#endif
#if AETHOR_ADRC_LCD_INTEGRATED
    aethor_app_enter_task_critical();
    if (aethor_app_mapping_locked())
    {
        if (request != NULL && request->operation == DEBUG_UI_OPERATION_STOP)
        { aethor_app_mapping_fail(1U, application_low_speed_mapping.phase_us); }
        else { aethor_app_exit_task_critical(); return DEBUG_UI_REASON_DISABLED; }
    }
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
    if (request != NULL && request->operation == DEBUG_UI_OPERATION_STOP &&
        (reason == DEBUG_UI_REASON_NONE || reason == DEBUG_UI_REASON_STOP_LATCHED))
    {
        uint8_t joint_index;
        /* STOP invalidates every already-reviewed installation even after its queues drain. */
        for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
        {
            ++application_debug_ui.single_static_evidence_generation[joint_index];
            if (application_debug_ui.single_static_evidence_generation[joint_index] == 0U)
            { ++application_debug_ui.single_static_evidence_generation[joint_index]; }
        }
    }
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
    snapshot->static_load_enabled = AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD;
    snapshot->single_static_load_enabled = AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD;
    snapshot->pos_load_enabled = AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST;
    snapshot->pos_load = application_debug_ui.pos_load;
    snapshot->static_load = application_debug_ui.static_load;
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD)
    { snapshot->motion_enabled = 0U; snapshot->mit_enabled = 0U; snapshot->static_load_enabled = 0U;
      snapshot->single_static_load_enabled = 0U; snapshot->pos_load_enabled = 0U; }
#endif
    snapshot->target_motor_id = application_debug_ui.target_motor_id;
    snapshot->unconfirmed_disable_mask = application_debug_ui.unconfirmed_disable_mask;
    snapshot->unconfirmed_disable_mask |= application_debug_ui.single_static_transport_mask;
    if (application_debug_ui.pos_disable_pending && application_debug_ui.pos_load.motor_id)
        snapshot->unconfirmed_disable_mask |= (uint8_t)(1U << (application_debug_ui.pos_load.motor_id - 1U));
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
        view->single_static_native_per_output = aethor_app_lcd_position_ratio(joint_index);
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
        if (joint_index == 6U) view->pos_load_output_per_flange =
            AETHOR_S3519_MOTOR7_FLANGE_TOTAL_RATIO / aethor_app_lcd_position_ratio(joint_index);
#endif
        view->feedback_seen = application_motor_runtime.bank.motors[joint_index].feedback_valid;
        view->feedback_valid = (uint8_t)((motors.valid_joint_mask & (1U << joint_index)) != 0U);
        view->feedback_age_ms = age_us / 1000ULL > UINT32_MAX ? UINT32_MAX : (uint32_t)(age_us / 1000ULL);
        view->feedback_timestamp_us = feedback->timestamp_us;
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
        view->single_static_profile = application_debug_ui.single_static_profiles[joint_index];
        view->single_static_gains = application_debug_ui.single_static_gains[joint_index];
        if (!aethor_app_single_static_profile_current(joint_index))
        { view->single_static_profile.configured = 0U; }
        view->single_static_evidence_generation = application_debug_ui.single_static_evidence_generation[joint_index];
        view->single_static_lcd_owned = application_debug_ui.single_static_lcd_owned[joint_index];
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
        if (application_motor_runtime.position_read.joint_index == joint_index
#if AETHOR_IDLE_FEEDBACK_ENABLE
            || application_motor_runtime.idle_position_read.joint_index == joint_index
#endif
            )
        {
            unsigned register_index;
            const MotorRegisterPosition *positions = &application_motor_runtime.position_read;
#if AETHOR_IDLE_FEEDBACK_ENABLE
            if (aethor_app_idle_position_allowed(timestamp_us) &&
                application_motor_runtime.idle_position_read.joint_index == joint_index &&
                application_motor_runtime.idle_position_read.seen_mask)
                positions = &application_motor_runtime.idle_position_read;
            if (joint_index == 6U && positions->joint_index == 6U &&
                positions == &application_motor_runtime.idle_position_read &&
                aethor_app_idle_position_allowed(timestamp_us))
            {
                view->register_position_contract = 1U;
                view->register_estimated_native_rad_s = positions->estimated_native_rad_s;
                view->register_estimated_velocity_valid = positions->estimated_velocity_valid;
            }
#endif
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
/** @brief Check immutable HOME bounds for DISABLE-only preparation and again after standard-feedback qualification. */
static DebugUiReason aethor_app_home_intent_reason(const ProtocolCommand *command,
    uint8_t joint_index, const DebugUiMotorProfile *profile, const MotorDiscoveryResult *discovery)
{
    float target = command->home_target_position_rad;
    if (joint_index != 6U || command->motor_mask != 0x40U || !isfinite(target) ||
        !isfinite(command->speeds[joint_index])) return DEBUG_UI_REASON_INVALID_ARGUMENT;
    if (target < profile->position_min_rad || target > profile->position_max_rad ||
        fabsf(target) > discovery->ranges.position_max_rad || command->speeds[joint_index] <= 0.0F ||
        command->speeds[joint_index] > profile->max_speed_rad_s ||
        command->speeds[joint_index] > discovery->maximum_speed_rad_s ||
        command->speeds[joint_index] > discovery->ranges.velocity_max_rad_s) return DEBUG_UI_REASON_OUT_OF_RANGE;
    return DEBUG_UI_REASON_NONE;
}

static DebugUiReason aethor_app_debug_ui_validate_start(
    const ProtocolCommand *command, const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us,
    bool preflight_complete)
{
    DebugUiReason reason = DEBUG_UI_REASON_NONE;
    uint8_t joint_index;
    aethor_app_enter_task_critical();
    aethor_app_debug_ui_refresh_reference(timestamp_us);
#if AETHOR_DEBUG_UI_ALLOW_MOTION
    if (command->local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) {
        reason = aethor_app_single_static_preflight_validate(command, snapshot, timestamp_us, preflight_complete);
        aethor_app_exit_task_critical();
        return reason;
    }
#endif
    if ((AETHOR_DEBUG_UI_ALLOW_MOTION == 0U) ||
        (AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE)) { reason = DEBUG_UI_REASON_DISABLED; }
    else if (command->session_id != application_debug_ui.epoch) { reason = DEBUG_UI_REASON_OLD_EPOCH; }
    else if ((timestamp_us < command->created_at_us) || (!preflight_complete &&
             (timestamp_us - command->created_at_us > DEBUG_UI_REQUEST_MAX_AGE_US))) { reason = DEBUG_UI_REASON_STALE_REQUEST; }
    else if (!debug_ui_mailbox_healthy(&application_debug_ui.mailbox, timestamp_us)) { reason = DEBUG_UI_REASON_UNHEALTHY; }
    else if ((application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_ARMED) &&
             (application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_BUSY)) { reason = DEBUG_UI_REASON_NOT_ARMED; }
    if (reason == DEBUG_UI_REASON_NONE && command->local_operation == DEBUG_UI_OPERATION_HOME &&
        !preflight_complete && command->type != PROTOCOL_COMMAND_HOME_PREPARE) reason = DEBUG_UI_REASON_INVALID_ARGUMENT;
    if (reason == DEBUG_UI_REASON_NONE && command->local_operation == DEBUG_UI_OPERATION_HOME &&
        command->home_boot_id != application_protocol_engine.boot_id) reason = DEBUG_UI_REASON_OLD_EPOCH;
    if ((reason == DEBUG_UI_REASON_NONE) && debug_ui_is_static_operation(command->local_operation))
    { reason = aethor_app_static_load_validate(command, snapshot); }
    if (reason == DEBUG_UI_REASON_NONE && debug_ui_is_pos_load_operation(command->local_operation) &&
        !AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST) reason = DEBUG_UI_REASON_DISABLED;
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
            ((command->type == PROTOCOL_COMMAND_HOME_PREPARE) ||
             (command->type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
             (command->type == PROTOCOL_COMMAND_MIT_ACTION) || (command->type == PROTOCOL_COMMAND_SET_MODE)) &&
            ((discovery->observed_control_mode == 1U) || (discovery->observed_control_mode == 2U));
        bool check_target = !defer_feedback || ((snapshot->valid_joint_mask & (1U << joint_index)) != 0U);
        float target = command->values[joint_index] + feedback->position_rad;
        DebugUiReason home_reason = DEBUG_UI_REASON_NONE;
        mode_profile = debug_ui_profile_bounds(&application_debug_ui.profiles[joint_index], (uint8_t)mit);
        if (command->local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
            DebugUiMotorProfile single_bounds = aethor_app_single_static_bounds(joint_index);
            mode_profile = debug_ui_profile_bounds(&single_bounds, 1U);
        }
        /* The preceding static gate proves exact reference-bound equality. */
        if (command->local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && joint_index == 6U &&
            application_debug_ui.single_static_lcd_owned[joint_index] &&
            debug_ui_static_gains_valid(application_debug_ui.single_static_gains[joint_index].kp,
                application_debug_ui.single_static_gains[joint_index].kd)) {
            mode_profile.kp_min = mode_profile.kp_max = command->mit_kp;
            mode_profile.kd_min = mode_profile.kd_max = command->mit_kd;
        }
        if (debug_ui_is_pos_load_operation(command->local_operation))
            mode_profile = aethor_app_pos_load_bounds(joint_index);
        if ((command->motor_mask & (1U << joint_index)) == 0U) { continue; }
        if (command->local_operation == DEBUG_UI_OPERATION_HOME)
            home_reason = aethor_app_home_intent_reason(command, joint_index, profile, discovery);
        if (debug_ui_is_pos_load_operation(command->local_operation) && discovery->observed_control_mode != 2U)
        { reason = DEBUG_UI_REASON_NOT_READY; break; }
        if (((command->type == PROTOCOL_COMMAND_DISABLE) || (command->type == PROTOCOL_COMMAND_CLEAR_FAULT))
                ? ((profile->mit_valid == 0U) && (profile->pos_valid == 0U))
                : ((mit ? profile->mit_valid : profile->pos_valid) == 0U))
        { reason = DEBUG_UI_REASON_UNCALIBRATED; }
        else if (mit && (AETHOR_DEBUG_UI_ALLOW_MIT == 0U)) { reason = DEBUG_UI_REASON_DISABLED; }
        else if ((debug_ui_is_static_operation(command->local_operation) ?
                  command->static_reference_generations[joint_index] : command->reference_generation) !=
                 application_debug_ui.reference_generation[joint_index]) { reason = DEBUG_UI_REASON_OLD_EPOCH; }
        else if (!defer_feedback && ((snapshot->valid_joint_mask & (1U << joint_index)) == 0U)) { reason = DEBUG_UI_REASON_STALE_FEEDBACK; }
        else if ((discovery->verified_fields_mask & (MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK | MOTOR_DISCOVERY_RANGE_FIELDS_MASK | MOTOR_DISCOVERY_MODE_FIELDS_MASK)) !=
                 (MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK | MOTOR_DISCOVERY_RANGE_FIELDS_MASK | MOTOR_DISCOVERY_MODE_FIELDS_MASK)) { reason = DEBUG_UI_REASON_NOT_READY; }
        else if ((command->type != PROTOCOL_COMMAND_DISABLE) &&
                 (command->type != PROTOCOL_COMMAND_CLEAR_FAULT) &&
                 ((feedback->driver_state != S3519_DRIVER_STATE_DISABLED) ||
                  !isfinite(feedback->velocity_rad_s) ||
                  (fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S) ||
                  (feedback->fault_flags != 0U))) { reason = DEBUG_UI_REASON_NOT_DISABLED; }
        else if (home_reason != DEBUG_UI_REASON_NONE) { reason = home_reason; }
        else if (move && ((fabsf(command->values[joint_index]) > profile->max_delta_rad) ||
                 (command->speeds[joint_index] > profile->max_speed_rad_s) ||
                 (check_target && (!isfinite(target) || (target < profile->position_min_rad) ||
                  (target > profile->position_max_rad) || (fabsf(target) > discovery->ranges.position_max_rad))) ||
                 (command->speeds[joint_index] > discovery->maximum_speed_rad_s) ||
                 (command->speeds[joint_index] > discovery->ranges.velocity_max_rad_s))) { reason = DEBUG_UI_REASON_OUT_OF_RANGE; }
        else if ((command->type == PROTOCOL_COMMAND_MIT_ACTION) &&
                 ((command->mit_kp < profile->kp_min) || (command->mit_kp > profile->kp_max) ||
                  (command->mit_kd < profile->kd_min) || (command->mit_kd > profile->kd_max) ||
                  (command->local_operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && command->mit_torque_ff_nm != 0.0F) ||
                  ((command->local_operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) &&
                   ((command->hold_duration_us < (uint64_t)profile->hold_min_ms * 1000ULL) ||
                    (command->hold_duration_us > (uint64_t)profile->hold_max_ms * 1000ULL))) ||
                  (check_target && (!isfinite(feedback->position_rad) ||
                   (feedback->position_rad < profile->position_min_rad) ||
                   (feedback->position_rad > profile->position_max_rad))))) { reason = DEBUG_UI_REASON_OUT_OF_RANGE; }
    }
    if (reason == DEBUG_UI_REASON_NONE) { reason = protocol_engine_validate_typed_command(command); }
    aethor_app_exit_task_critical();
    return reason;
}
#endif

/** @brief Keep historical health evidence scoped to its original action, including its pre-revocation epoch. */
static uint8_t aethor_app_debug_ui_health_failure_matches(const ProtocolCommand *command)
{
    return (uint8_t)(application_debug_ui.health_stop_started &&
        application_debug_ui.health_failure_action_valid &&
        command->origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
        command->origin == application_debug_ui.health_failure_identity.origin &&
        command->session_id == application_debug_ui.health_failure_identity.epoch &&
        command->request_id == application_debug_ui.health_failure_identity.request_id &&
        command->local_operation == application_debug_ui.health_failure_operation &&
        command->type == application_debug_ui.health_failure_command_type &&
        command->motor_mask == application_debug_ui.health_failure_motor_mask);
}

/** @brief Captures source and fresh disable evidence at result creation, never later. */
static void aethor_app_debug_ui_result_metadata(
    ProtocolCommandResult *result, const ProtocolCommand *command, uint64_t timestamp_us)
{
    MotorFeedbackSnapshot snapshot;
    result->origin = command->origin;
    result->local_operation = command->local_operation;
    aethor_app_enter_task_critical();
    if (aethor_app_debug_ui_health_failure_matches(command))
    { result->local_reason = DEBUG_UI_REASON_UNHEALTHY; }
    aethor_app_exit_task_critical();
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot);
    result->disabled_confirmed = aethor_app_selected_motors_have_state(
        &snapshot, command->motor_mask, S3519_DRIVER_STATE_DISABLED);
#if !AETHOR_ADRC_BENCH
    if (debug_ui_is_pos_load_operation(command->local_operation) ||
        (command->local_operation == DEBUG_UI_OPERATION_STOP &&
         command->request_id == application_debug_ui.pos_load.stop_identity.request_id &&
         command->session_id == application_debug_ui.pos_load.stop_identity.epoch &&
         command->origin == application_debug_ui.pos_load.stop_identity.origin))
        result->disabled_confirmed &= aethor_app_pos_disable_proven(&snapshot);
#endif
    if (command->local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) {
        uint8_t preflight_index;
        result->disabled_confirmed &= application_action.single_static_tx_confirmed;
        for (preflight_index = 0U; preflight_index < DEBUG_UI_MOTOR_COUNT; ++preflight_index)
            if (command->motor_mask & (1U << preflight_index)) {
                result->feedback_timestamp_us = snapshot.joints[preflight_index].timestamp_us;
                if (application_motor_runtime.bank.motors[preflight_index].feedback_generation == application_action.single_static_rx_generation ||
                    application_motor_runtime.bank.motors[preflight_index].feedback_generation != application_action.single_static_ordered_generation ||
                    result->feedback_timestamp_us <= application_action.phase_feedback_baseline_us[preflight_index])
                    result->disabled_confirmed = 0U;
            }
    }
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
        aethor_app_static_load_finish(&result);
        aethor_app_pos_load_finish(&result);
        aethor_app_static_trace_terminal(&result);
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
        completion.feedback_timestamp_us = result.feedback_timestamp_us;
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
    command->home_boot_id = request->home_boot_id;
    command->local_operation = request->operation;
    command->single_static_preflight_purpose = request->single_static_preflight_purpose;
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
        case DEBUG_UI_OPERATION_STATIC_HOLD:
        case DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD:
            command->type = PROTOCOL_COMMAND_MIT_ACTION;
            command->control_mode = ARM_CONTROL_MODE_MIT;
            command->mit_action = PROTOCOL_MIT_ACTION_HOLD;
            command->motor_mask = request->operation == DEBUG_UI_OPERATION_STATIC_HOLD ?
                0x7FU : (uint8_t)(1U << joint_index);
            command->relative_target = 0U;
            /* Explicit ID7 gains survive conversion; legacy zero pairs retain defaults. */
            if (request->operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ||
                (request->kp == 0.0F && request->kd == 0.0F)) {
            command->mit_kp = application_debug_ui.profiles[
                request->operation == DEBUG_UI_OPERATION_STATIC_HOLD ? 0U : joint_index].default_kp;
            command->mit_kd = application_debug_ui.profiles[
                request->operation == DEBUG_UI_OPERATION_STATIC_HOLD ? 0U : joint_index].default_kd;
            }
            command->planned_duration_us = command->hold_duration_us;
            for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
            {
                if ((command->motor_mask & (1U << joint_index)) == 0U) { continue; }
                command->values[joint_index] = request->static_positions_rad[joint_index] * aethor_app_lcd_position_ratio(joint_index);
                command->speeds[joint_index] = 0.0F;
                command->static_reference_generations[joint_index] = request->static_references[joint_index];
            }
            break;
        case DEBUG_UI_OPERATION_POS_MOVE:
        case DEBUG_UI_OPERATION_HOME:
        case DEBUG_UI_OPERATION_POS_LOAD_TEST:
            command->type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
            if (request->operation == DEBUG_UI_OPERATION_HOME) {
                command->type = PROTOCOL_COMMAND_HOME_PREPARE;
                command->relative_target = 0U;
                command->home_target_position_rad = command->values[joint_index];
                command->values[joint_index] = 0.0F;
            }
            if (debug_ui_is_pos_load_operation(request->operation)) command->planned_duration_us = command->hold_duration_us;
            break;
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
        case DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT:
            command->single_static_evidence_generation = request->single_static_evidence_generation;
            if (request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) command->relative_target = 0U;
            command->type = request->operation != DEBUG_UI_OPERATION_CLEAR_FAULT
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

#include "DebugUi/debug_ui_single_static_config_app.inc"

/** @brief Validates and grants/revokes authority only after all prior cleanup/results finish. */
#if !AETHOR_ADRC_BENCH
static DebugUiReason aethor_app_debug_ui_admit_request(
    const DebugUiRequest *request, uint64_t timestamp_us, ProtocolCommand *command)
{
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    aethor_app_persistence_service(timestamp_us);
    if (aethor_app_persistence_locked()) {
        if (!request || request->operation != DEBUG_UI_OPERATION_STOP) { return DEBUG_UI_REASON_DISABLED; }
        aethor_app_persistence_preempt(timestamp_us);
        aethor_app_persistence_disable_intent(aethor_app_persistence_local_stop_mask(request->target_motor_id));
    }
#endif
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
    if (request->operation == DEBUG_UI_OPERATION_STATIC_HOLD && !AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD)
    { return DEBUG_UI_REASON_DISABLED; }
    if (debug_ui_is_pos_load_operation(request->operation) && !AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    { return DEBUG_UI_REASON_DISABLED; }
    if (request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && !AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD)
    { return DEBUG_UI_REASON_DISABLED; }
    if (request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE ||
        request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK)
    { return aethor_app_single_static_config_request(request, timestamp_us); }
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
    if (request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) {
        if (aethor_app_debug_ui_executor_busy()) return DEBUG_UI_REASON_BUSY;
        if (debug_ui_mailbox_result_count(&application_debug_ui.mailbox) != 1U)
            return DEBUG_UI_REASON_RESULT_BACKPRESSURE;
        reason = aethor_app_single_static_preflight_validate(command, &snapshot, timestamp_us, false);
        return reason == DEBUG_UI_REASON_NONE ? protocol_engine_submit_local_command(&application_protocol_engine, command) : reason;
    }
    if ((request->operation == DEBUG_UI_OPERATION_ACQUIRE) || (request->operation == DEBUG_UI_OPERATION_RELEASE))
    {
        const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        if (request->operation == DEBUG_UI_OPERATION_RELEASE && request->remote_proxy &&
            application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT &&
            !aethor_remote_fault_release_admission(request)) return DEBUG_UI_REASON_NOT_ARMED;
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
            (request.operation == DEBUG_UI_OPERATION_RELEASE) ||
            (request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE) ||
            (request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK))
        {
            DebugUiCompletion completion;
            memset(&completion, 0, sizeof(completion));
            completion.identity = request.identity;
            completion.operation = request.operation;
            /* A rejected static request still has a known UI scope; preserve it for exact matching. */
            if (request.operation == DEBUG_UI_OPERATION_STATIC_HOLD)
            { completion.motor_mask = 0x7FU; }
            else if ((request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ||
                request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE ||
                request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK ||
                request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) &&
                request.target_motor_id >= 1U && request.target_motor_id <= DEBUG_UI_MOTOR_COUNT)
            { completion.motor_mask = (uint8_t)(1U << (request.target_motor_id - 1U)); }
            completion.reason = admission.reason;
            completion.timestamp_us = timestamp_us;
            completion.code = admission.accepted != 0U ? DEBUG_UI_COMPLETED : DEBUG_UI_FAILED;
            completion.stage = DEBUG_UI_STAGE_VALIDATE;
            completion.disabled_confirmed = (uint8_t)(admission.accepted &&
                request.operation != DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE &&
                request.operation != DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
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
    aethor_app_remote_service(timestamp_us);
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
        /* An idle fault owns no command. A cleanup already has its first cause; preserve it. */
        application_debug_ui.health_failure_action_valid = (uint8_t)(
            application_action.state != AETHOR_APP_ACTION_IDLE &&
            application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
            application_action.local_failure_reason == DEBUG_UI_REASON_NONE &&
            application_action.failed_stage == PROTOCOL_COMMAND_STAGE_NONE);
        if (application_debug_ui.health_failure_action_valid) {
            application_debug_ui.health_failure_identity.origin = application_action.command.origin;
            application_debug_ui.health_failure_identity.epoch = application_action.command.session_id;
            application_debug_ui.health_failure_identity.request_id = application_action.command.request_id;
            application_debug_ui.health_failure_operation = application_action.command.local_operation;
            application_debug_ui.health_failure_command_type = application_action.command.type;
            application_debug_ui.health_failure_motor_mask = application_action.command.motor_mask;
        }
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
        if (application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) {
            application_action.local_failure_reason = DEBUG_UI_REASON_UNHEALTHY;
            return aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED, 0U, timestamp_us);
        }
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

    aethor_app_static_trace_emergency_begin(timestamp_us);
    aethor_app_widen_pending_stop_before_global_cancel();
    if (application_action.state != AETHOR_APP_ACTION_IDLE)
    {
        (void)motor_runtime_abort_active_parameter_sequences(
            &application_motor_runtime,
            timestamp_us);
        if ((application_action.command.type ==
             PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED) ||
            (application_action.command.type == PROTOCOL_COMMAND_HOME_PREPARE) ||
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
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (aethor_app_persistence_locked()) { return 0U; }
#endif
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

/** @brief Binds platform callbacks before any selftest; USB cannot replace the hooks. */
void aethor_app_adrc_set_guard_hooks(const AdrcStopGuardHooks *hooks)
{
    if (!application_initialized || hooks == NULL) { return; }
    aethor_app_enter_task_critical();
    if (application_adrc_bridge.guard_test.state == 0U &&
        application_adrc_bridge.guard_test.hooks.arm == NULL)
    { application_adrc_bridge.guard_test.hooks = *hooks; }
    aethor_app_exit_task_critical();
}

/** @brief Installs the platform-only finite trial guard under existing application serialization. */
void aethor_app_adrc_set_simple_guard_hooks(const AdrcSimpleGuardHooks *hooks)
{
    if (!application_initialized || hooks == NULL) { return; }
    aethor_app_enter_task_critical();
    adrc_app_bridge_set_simple_guard_hooks(&application_adrc_bridge, hooks);
    aethor_app_exit_task_critical();
}

/** @brief Returns the deliberate control-task stall once, after the hardware deadline is armed. */
uint8_t aethor_app_adrc_take_guard_pause(void)
{
    uint8_t pause_ms;
    if (!application_initialized) { return 0U; }
    aethor_app_enter_task_critical();
    pause_ms = application_adrc_bridge.guard_test.pause_ms;
    application_adrc_bridge.guard_test.pause_ms = 0U;
    aethor_app_exit_task_critical();
    return pause_ms;
}

/** @brief Matches a hardware TX receipt to the selected-axis owner without claiming motor execution. */
void aethor_app_adrc_report_transmit(uint8_t kind, float decoded_torque_nm, uint8_t succeeded)
{
    if (!application_initialized) { return; }
    aethor_app_enter_task_critical();
    adrc_app_bridge_report_transmit(&application_adrc_bridge, kind, decoded_torque_nm, succeeded);
    aethor_app_exit_task_critical();
}

/** @brief Forwards fixed-read completion under task critical; late receipts cannot advance terminal state. */
void aethor_app_adrc_report_capability_transmit(const CanFrame *frame,
    uint8_t succeeded, uint64_t completed_observed_us)
{
    aethor_app_enter_task_critical();
    adrc_app_bridge_report_capability_transmit(&application_adrc_bridge,
        frame, succeeded, completed_observed_us);
    aethor_app_exit_task_critical();
}

/** @brief Supplies only board-published FIFO timing under the established owner critical hooks. */
void aethor_app_adrc_observe_capability_rx_boundary(uint64_t boundary_us)
{
    aethor_app_enter_task_critical();
#if AETHOR_ADRC_LCD_INTEGRATED
    application_mapping_rx_boundary_us = boundary_us;
#endif
    motor_capability_observe_rx_boundary(&application_adrc_bridge.capabilities, boundary_us);
    aethor_app_exit_task_critical();
}

/** @brief Marks a release DISABLE only after the dedicated hardware buffer accepts it. */
void aethor_app_adrc_report_disable_submit_at(uint8_t kind, uint64_t timestamp_us)
{
    if (!application_initialized || kind != 2U) { return; }
    aethor_app_enter_task_critical();
    adrc_app_bridge_note_capability_disable_submit(&application_adrc_bridge, timestamp_us);
#if AETHOR_ADRC_LCD_INTEGRATED
    adrc_lcd_ownership_report_disable_submit(&application_adrc_lcd_ownership, timestamp_us);
    if (application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING &&
        timestamp_us != 0ULL && timestamp_us >= application_adrc_lcd_ownership.release_started_us)
    { application_integrated_release_last_submit_us = timestamp_us; }
#endif
    aethor_app_exit_task_critical();
}

/** @brief Couples a real selected-axis disable receipt to the LCD handback evidence gate. */
void aethor_app_adrc_report_transmit_at(uint8_t kind, float decoded_torque_nm,
    uint8_t succeeded, uint64_t timestamp_us)
{
#if AETHOR_ADRC_LCD_INTEGRATED
    uint8_t matched_disable;
#endif
    if (!application_initialized) { return; }
    aethor_app_enter_task_critical();
#if AETHOR_ADRC_LCD_INTEGRATED
    matched_disable = (uint8_t)(succeeded != 0U && kind == 2U &&
        application_adrc_bridge.awaiting_receipt != 0U &&
        application_adrc_bridge.frame_kind == kind &&
        application_adrc_bridge.decoded_torque_nm == decoded_torque_nm);
#endif
    adrc_app_bridge_report_transmit_at(&application_adrc_bridge, kind, decoded_torque_nm, succeeded, timestamp_us);
#if AETHOR_ADRC_LCD_INTEGRATED
    if (matched_disable != 0U)
    { adrc_lcd_ownership_report_disable_transmit(&application_adrc_lcd_ownership,
        timestamp_us, 1U); }
#endif
    aethor_app_exit_task_critical();
}

/** @brief Installs locally measured commissioning evidence, retaining the supervisor's provenance checks. */
AdrcExperimentResult aethor_app_adrc_set_evidence(const AdrcExperimentConfig *config,
    const AdrcExperimentQualification *qualification, uint64_t timestamp_us)
{
    AdrcExperimentResult result;
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (aethor_app_persistence_locked()) { return ADRC_RESULT_INVALID_STATE; }
#endif
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
