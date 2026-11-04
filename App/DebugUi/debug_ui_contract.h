/** @file debug_ui_contract.h
 * @brief Value-only UI/application contract; no HAL, RTOS or graphics dependency.
 * All coordinates are S3519 output-shaft rad, rad/s and Nm, not joint calibration.
 */
#ifndef APP_DEBUG_UI_DEBUG_UI_CONTRACT_H
#define APP_DEBUG_UI_DEBUG_UI_CONTRACT_H

#include <stdint.h>
#include "debug_ui_single_static_gains.h"
#include "../Config/debug_ui_config.h"

/** @brief Namespace of a request; USB remains zero for existing initialized commands. */
typedef enum { DEBUG_UI_ORIGIN_USB = 0, DEBUG_UI_ORIGIN_LOCAL_UI = 1 } DebugUiOrigin;
/** @brief Immutable identity; epoch is USB session or local authorization generation. */
typedef struct { DebugUiOrigin origin; uint32_t epoch; uint32_t request_id; } DebugUiIdentity;
/** @brief Runtime authority; FAULT remains locked until results are read and re-acquired. */
typedef enum { DEBUG_UI_AUTHORITY_REMOTE = 0, DEBUG_UI_AUTHORITY_LOCAL_ARMED,
    DEBUG_UI_AUTHORITY_LOCAL_BUSY, DEBUG_UI_AUTHORITY_LOCAL_FAULT } DebugUiAuthority;
/** @brief Explicit operations; POS and MIT MOVE use a relative displacement. */
typedef enum { DEBUG_UI_OPERATION_ACQUIRE = 0, DEBUG_UI_OPERATION_RELEASE,
    DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_OPERATION_MIT_HOLD,
    DEBUG_UI_OPERATION_MIT_MOVE, DEBUG_UI_OPERATION_SET_MODE,
    DEBUG_UI_OPERATION_CLEAR_FAULT, DEBUG_UI_OPERATION_DISABLE,
    DEBUG_UI_OPERATION_STOP,
    DEBUG_UI_OPERATION_STATIC_HOLD,
    DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD,
    DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE,
    DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK,
    DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT,
    DEBUG_UI_OPERATION_POS_LOAD_TEST,
    DEBUG_UI_OPERATION_HOME } DebugUiOperation;
/** @brief HOME inherits every loaded-POS protection while keeping its own absolute-target identity. */
static inline uint8_t debug_ui_is_pos_load_operation(DebugUiOperation operation)
{
    return (uint8_t)(operation == DEBUG_UI_OPERATION_POS_LOAD_TEST || operation == DEBUG_UI_OPERATION_HOME);
}
/** @brief Only an explicit selected DISABLE for withdrawing this LCD profile may recover fault ownership. */
typedef enum { DEBUG_UI_SINGLE_STATIC_PREFLIGHT_NONE = 0,
    DEBUG_UI_SINGLE_STATIC_PREFLIGHT_LOCK } DebugUiSingleStaticPreflightPurpose;
/** @brief Classify the two explicit static operations without granting their compile gates. */
static inline uint8_t debug_ui_is_static_operation(DebugUiOperation operation)
{
    return (uint8_t)(operation == DEBUG_UI_OPERATION_STATIC_HOLD ||
        operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
}
/** @brief Actual/requested mode; UNKNOWN is not interpreted as POS. */
typedef enum { DEBUG_UI_MODE_UNKNOWN = 0, DEBUG_UI_MODE_MIT = 1,
    DEBUG_UI_MODE_POS_VEL = 2 } DebugUiMode;
/** @brief Bounded rejection reasons; submit success only reserves a mailbox slot. */
typedef enum { DEBUG_UI_REASON_NONE = 0, DEBUG_UI_REASON_BUSY,
    DEBUG_UI_REASON_DISABLED, DEBUG_UI_REASON_INVALID_ARGUMENT,
    DEBUG_UI_REASON_UNCALIBRATED, DEBUG_UI_REASON_STALE_REQUEST,
    DEBUG_UI_REASON_OLD_EPOCH, DEBUG_UI_REASON_NOT_ARMED,
    DEBUG_UI_REASON_STALE_FEEDBACK, DEBUG_UI_REASON_NOT_DISABLED,
    DEBUG_UI_REASON_UNHEALTHY, DEBUG_UI_REASON_OUT_OF_RANGE,
    DEBUG_UI_REASON_RESULT_BACKPRESSURE, DEBUG_UI_REASON_DUPLICATE,
    DEBUG_UI_REASON_NOT_READY,
    /** @brief Value 15: STOP intent latched synchronously; no admission/completion for this ID.
     * Existing accepted identities/results remain intact. Observe fresh stop_pending/feedback;
     * this is neither acceptance of a result-bearing transaction nor disable confirmation.
     */
    DEBUG_UI_REASON_STOP_LATCHED,
    /* Append-only runtime causes preserve existing reason values on the wire. */
    DEBUG_UI_REASON_TORQUE_LIMIT, DEBUG_UI_REASON_SPEED_LIMIT,
    DEBUG_UI_REASON_TEMPERATURE_LIMIT, DEBUG_UI_REASON_INVALID_FEEDBACK } DebugUiReason;
/** @brief Terminal state only; acceptance is reported separately. */
typedef enum { DEBUG_UI_COMPLETED = 0, DEBUG_UI_STOPPED,
    DEBUG_UI_FAILED, DEBUG_UI_CANCELLED } DebugUiCompletionCode;
/** @brief Stable action phase shared with the protocol's existing stage numbers. */
typedef enum { DEBUG_UI_STAGE_NONE = 0, DEBUG_UI_STAGE_VALIDATE,
    DEBUG_UI_STAGE_DISCOVERY, DEBUG_UI_STAGE_MODE, DEBUG_UI_STAGE_CLEAR,
    DEBUG_UI_STAGE_ENABLE, DEBUG_UI_STAGE_MOTION, DEBUG_UI_STAGE_HOLD,
    DEBUG_UI_STAGE_DISABLE } DebugUiStage;

/** @brief UI-owned immutable request; use monotonically increasing nonzero IDs per boot.
 * Copy epoch from the latest snapshot (including ACQUIRE); STOP retains its own identity
 * but ignores authorization/age gates. target_motor_id is one-based 1..7.
 * Submit NONE reserves exactly one admission and terminal. STOP_LATCHED consumes only
 * the safety intent/ID and promises no asynchronous result; never retry that ID as motion.
 */
typedef struct
{
    DebugUiIdentity identity;
    DebugUiOperation operation;
    uint64_t created_at_us;
    uint8_t remote_proxy; /**< Private adapter reservation; public LCD submit rejects this flag. */
    uint32_t reference_generation;
    uint32_t home_boot_id; /**< HOME-only exact boot binding; never passes through float gains. */
    uint8_t target_motor_id;
    DebugUiMode requested_mode;
    float delta_rad;
    float speed_rad_s;
    float kp;
    float kd;
    float torque_ff_nm;
    uint32_t hold_duration_ms;
    /** @brief Frozen existing output-axis coordinates and reference epochs for all seven axes. */
    float static_positions_rad[DEBUG_UI_MOTOR_COUNT];
    uint32_t static_references[DEBUG_UI_MOTOR_COUNT];
    /** @brief Frozen independent discovery/profile evidence; does not alter ordinary POS/MIT references. */
    uint32_t single_static_evidence_generation;
    /** @brief Local-only preflight purpose; every non-preflight request must leave this NONE. */
    DebugUiSingleStaticPreflightPurpose single_static_preflight_purpose;
} DebugUiRequest;

/** @brief ProtocolTask admission; rejected requests also receive a FAILED completion. */
typedef struct
{
    DebugUiIdentity identity;
    DebugUiOperation operation;
    DebugUiReason reason;
    uint64_t timestamp_us;
    uint8_t accepted;
} DebugUiAdmission;

/** @brief Retained until explicitly polled; disabled_confirmed requires fresh feedback. */
typedef struct
{
    DebugUiIdentity identity;
    DebugUiOperation operation;
    DebugUiCompletionCode code;
    DebugUiStage stage;
    DebugUiReason reason;
    uint64_t timestamp_us;
    uint16_t detail;
    uint8_t error;
    uint8_t motor_mask;
    uint8_t failed_motor_id;
    uint8_t disabled_confirmed;
    uint64_t feedback_timestamp_us; /**< Selected runtime feedback arrival lower bound at preflight completion; zero otherwise. */
} DebugUiCompletion;

/** @brief Explicit bench evidence profile; zero initialization leaves both modes invalid.
 * Bounds and defaults must come from a recorded bench test. Injection does not mark
 * motor identity/ranges/mode discovery as verified and cannot enable compile-time gates.
 */
typedef struct
{
    uint8_t pos_valid;
    uint8_t mit_valid;
    float position_min_rad;
    float position_max_rad;
    float max_delta_rad;
    float max_speed_rad_s;
    /* MIT limits are independent: a full POS envelope never authorizes MIT motion. */
    float mit_position_min_rad;
    float mit_position_max_rad;
    float mit_max_delta_rad;
    float mit_max_speed_rad_s;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float default_kp;
    float default_kd;
    uint32_t hold_min_ms;
    uint32_t hold_max_ms;
} DebugUiMotorProfile;

#define DEBUG_UI_SINGLE_STATIC_MIN_MS (1000U)
#define DEBUG_UI_SINGLE_STATIC_MAX_MS (60000U)
/** @brief Explicit one-axis long-window configuration; zero initialization never grants it.
 * Application binding to the current reference generation is independent of ordinary MIT gains.
 */
typedef struct
{
    uint8_t configured;
    uint32_t minimum_ms;
    uint32_t maximum_ms;
    float torque_ff_nm; /**< Signed protocol output-shaft feedforward frozen with this profile reference. */
} DebugUiSingleStaticProfile;

/** @brief Copy mode-specific bounds without changing the published profile or provenance. */
static inline DebugUiMotorProfile debug_ui_profile_bounds(const DebugUiMotorProfile *profile, uint8_t mit)
{
    DebugUiMotorProfile bounds = *profile;
    if (mit)
    {
        bounds.position_min_rad = profile->mit_position_min_rad;
        bounds.position_max_rad = profile->mit_position_max_rad;
        bounds.max_delta_rad = profile->mit_max_delta_rad;
        bounds.max_speed_rad_s = profile->mit_max_speed_rad_s;
    }
    return bounds;
}

/** @brief UiTask health publication; advance sequence only after checking input/display.
 * ProtocolTask owns protocol_* acknowledgement fields; UI-provided values are ignored.
 */
typedef struct
{
    uint64_t ui_timestamp_us;
    uint64_t protocol_timestamp_us;
    uint32_t ui_sequence;
    uint32_t protocol_sequence;
    uint32_t display_error;
    uint8_t input_valid;
    uint8_t display_valid;
    uint8_t ui_valid;
    uint8_t protocol_valid;
    /** @brief Optional UI-owned diagnostics; ignore these values until this bit is set. */
    uint8_t ui_diagnostics_valid;
    uint16_t adc_raw;
    uint8_t input_key;
    uint32_t input_age_ms;
    uint32_t display_flush_last_us;
    uint32_t display_dma_error_count;
    uint32_t ui_stack_min_words;
} DebugUiHealth;

/** @brief One motor display row; validity bits must be checked before showing a value. */
typedef struct
{
    uint8_t motor_id;
    uint8_t feedback_valid;
    uint8_t feedback_seen;
    uint8_t enabled;
    uint8_t driver_state;
    uint8_t identity_verified;
    uint8_t mode_verified;
    uint8_t ranges_verified;
    DebugUiMode actual_mode;
    uint32_t reference_generation;
    uint32_t feedback_age_ms;
    uint64_t feedback_timestamp_us; /**< Runtime arrival lower bound; display age never manufactures freshness or physical sample time. */
    uint32_t fault_flags;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float mos_temperature_c;
    float rotor_temperature_c;
    float position_max_rad;
    float velocity_max_rad_s;
    float torque_max_nm;
    float maximum_speed_rad_s;
    DebugUiMotorProfile profile;
    /** @brief Raw RID 0x50/0x51 values; independent from all standard feedback gates. */
    float register_position[2];
    uint32_t register_age_ms[2];
    uint32_t register_timeout_count;
    uint8_t register_seen_mask;
    float register_estimated_native_rad_s; /**< Display-only 0x50 derivative in rotor rad/s. */
    uint8_t register_estimated_velocity_valid;
    uint8_t register_position_contract; /**< 1: verified ID7 p_m rotor radians; never a safety gate. */
    DebugUiSingleStaticProfile single_static_profile;
    /** @brief Monotonic discovery/profile review generation, including changes while unconfigured. */
    uint32_t single_static_evidence_generation;
    /** @brief Only this LCD workflow may revoke its RAM profile; public provisioning remains separate. */
    uint8_t single_static_lcd_owned;
    /** @brief ID7 POS load output-shaft radians per final-flange radian; zero means unavailable. */
    float pos_load_output_per_flange;
    float single_static_native_per_output; /**< Verified display mapping for dedicated absolute-pose bounds. */
    DebugUiStaticGains single_static_gains; /**< Output-shaft pair bound to the configured reference; zero means legacy. */
} DebugUiMotorView;

/** @brief Platform-neutral UI diagnostic counters; producers retain individual ownership. */
typedef struct
{
    uint32_t control_deadline_miss_count;
    uint32_t can_rx_drop_count;
    uint32_t usb_rx_drop_count;
    uint32_t rejected_request_count;
    uint32_t health_stop_count;
    uint32_t result_backpressure_count;
    uint32_t can_tx_error_count;
    uint32_t usb_telemetry_drop_count;
    uint32_t minimum_stack_words;
    uint32_t minimum_heap_bytes;
} DebugUiDiagnostics;

/** @brief Observation phase, distinct from the HOLD duration and transport protocol stage. */
typedef enum { DEBUG_UI_STATIC_LOAD_PHASE_NONE = 0, DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER,
    DEBUG_UI_STATIC_LOAD_PHASE_HOLD } DebugUiStaticLoadPhase;

/** @brief Observed takeover/HOLD measurements; peaks are not calibrated flange/load ratings. */
typedef struct
{
    DebugUiIdentity identity;
    uint64_t started_at_us;
    DebugUiIdentity stop_identity; /**< Explicit STOP cleanup bound to this trial, distinct from its cancelled identity. */
    uint32_t elapsed_ms;
    uint32_t duration_ms;
    float target_torque_ff_nm; /**< Confirmed requested target, not measured support torque. */
    float requested_torque_ff_nm; /**< Current prepared request; transport proof is separate. */
    float transmitted_torque_ff_nm; /**< Last actual-TX-confirmed takeover frame's decoded 12-bit torque request; not measured load. */
    uint32_t takeover_elapsed_ms;
    uint8_t takeover_active;
    uint8_t takeover_complete;
    DebugUiStaticLoadPhase phase; /**< Last observation phase; cleanup never relabels takeover as HOLD. */
    DebugUiStaticLoadPhase first_exit_phase;
    DebugUiReason first_exit_reason;
    DebugUiStage first_exit_stage;
    DebugUiCompletionCode first_exit_code;
    uint8_t first_exit_error; /**< Stable protocol error when no more specific local reason exists. */
    uint8_t first_exit_latched; /**< First causal exit survives later normal/disabled RX and independent STOP. */
    uint32_t sample_generation[DEBUG_UI_MOTOR_COUNT]; /**< Last accepted per-axis generation, not UI poll time. */
    uint32_t samples[DEBUG_UI_MOTOR_COUNT];
    float target_position_rad[DEBUG_UI_MOTOR_COUNT];
    float peak_error_rad[DEBUG_UI_MOTOR_COUNT];
    float peak_torque_nm[DEBUG_UI_MOTOR_COUNT];
    float peak_mos_c[DEBUG_UI_MOTOR_COUNT];
    float peak_rotor_c[DEBUG_UI_MOTOR_COUNT];
    uint8_t valid_mask;
    uint8_t frames_issued_mask; /**< Facade output evidence, not driver execution or CAN TX receipts. */
    uint8_t active;
    uint8_t complete;
    uint8_t stopped;
    uint8_t disabled_confirmed;
    DebugUiOperation operation;
    uint8_t motor_mask;
} DebugUiStaticLoadReport;

/** @brief Identity-bound output-shaft POS observation; no disturbance means no recovery qualification. */
typedef struct
{
    DebugUiIdentity identity;
    DebugUiIdentity stop_identity;
    DebugUiStage stage;
    uint64_t started_at_us;
    uint32_t elapsed_ms;
    uint32_t duration_ms;
    uint32_t samples;
    uint32_t disturbances;
    uint32_t recoveries;
    uint32_t last_recovery_ms;
    uint32_t max_recovery_ms;
    float target_position_rad;
    float error_rad;
    float peak_error_rad;
    float peak_torque_nm;
    float peak_mos_c;
    float peak_rotor_c;
    uint8_t motor_id;
    uint8_t active;
    uint8_t complete;
    uint8_t stopped;
    uint8_t disabled_confirmed;
    uint8_t recovering;
    uint8_t recovery_observed;
} DebugUiPosLoadReport;

/** @brief Coherent bounded copy for formatting outside the critical section. */
typedef struct
{
    uint64_t timestamp_us;
    uint32_t generation;
    uint32_t boot_id;
    uint32_t epoch;
    DebugUiAuthority authority;
    DebugUiIdentity active_identity;
    DebugUiIdentity stop_identity;
    DebugUiStage active_stage;
    DebugUiHealth health;
    DebugUiDiagnostics diagnostics;
    DebugUiMotorView motors[DEBUG_UI_MOTOR_COUNT];
    uint8_t active_motor_mask;
    uint8_t active;
    uint8_t pending;
    uint8_t stop_pending;
    uint8_t target_motor_id;
    uint8_t usb_connected;
    /* Original cleanup targets that still lack a post-attempt disabled acknowledgement. */
    uint8_t unconfirmed_disable_mask;
    uint8_t bench_profile;
    uint8_t motion_enabled;
    uint8_t mit_enabled;
    uint8_t retained_result_count;
    uint8_t arm_state;
    uint8_t arm_fault;
    uint8_t static_load_enabled;
    DebugUiStaticLoadReport static_load;
    uint8_t single_static_load_enabled;
    uint8_t pos_load_enabled;
    DebugUiPosLoadReport pos_load;
} DebugUiSnapshot;

#endif
