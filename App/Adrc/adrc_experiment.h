/** @file adrc_experiment.h
 * @brief HAL/RTOS-independent single-axis experiment supervisor; all units use output-shaft SI.
 */
#ifndef APP_ADRC_EXPERIMENT_H
#define APP_ADRC_EXPERIMENT_H

#include <stdint.h>

#define ADRC_EXPERIMENT_TRACE_CAPACITY (1024U)
#define ADRC_EXPERIMENT_PERIOD_US (4000ULL)
#define ADRC_QUAL_IDENTITY (1UL << 0)
#define ADRC_QUAL_COORDINATES (1UL << 1)
#define ADRC_QUAL_RANGES (1UL << 2)
#define ADRC_QUAL_FEEDBACK_TIMING (1UL << 3)
#define ADRC_QUAL_WATCHDOG (1UL << 4)
#define ADRC_QUAL_BOUNDED_BENCH (1UL << 5)
#define ADRC_QUAL_IDENTIFIED_MODEL (1UL << 6)
#define ADRC_QUAL_GENERATED_ALGORITHM (1UL << 7)
#define ADRC_QUAL_IDENTIFY_REQUIRED (0x3FUL)
#define ADRC_QUAL_CLOSED_LOOP_REQUIRED (0xFFUL)

/** @brief Identifies execution provenance; hardware never accepts offline qualification. */
typedef enum { ADRC_ENV_HARDWARE = 0, ADRC_ENV_OFFLINE_FIXTURE } AdrcExperimentEnvironment;
/** @brief Identifies the externally visible supervisor state. */
typedef enum { ADRC_STATE_DISABLED = 0, ADRC_STATE_PREPARED, ADRC_STATE_RUNNING,
    ADRC_STATE_STOPPING, ADRC_STATE_FAULT } AdrcExperimentState;
/** @brief Selects finite identification excitation or injected generated control. */
typedef enum { ADRC_MODE_IDENTIFY = 0, ADRC_MODE_PI, ADRC_MODE_LADRC } AdrcExperimentMode;
/** @brief Reports ordinary API errors without assertions on external data. */
typedef enum { ADRC_RESULT_OK = 0, ADRC_RESULT_INVALID_ARGUMENT, ADRC_RESULT_INVALID_STATE,
    ADRC_RESULT_UNQUALIFIED, ADRC_RESULT_UNSAFE_FEEDBACK, ADRC_RESULT_NO_CONTROLLER,
    ADRC_RESULT_TRACE_UNAVAILABLE } AdrcExperimentResult;
/** @brief Preserves the first fault while subsequent feedback can confirm disable. */
typedef enum { ADRC_FAULT_NONE = 0, ADRC_FAULT_TIME, ADRC_FAULT_FEEDBACK,
    ADRC_FAULT_PERIOD, ADRC_FAULT_DRIVER, ADRC_FAULT_TEMPERATURE, ADRC_FAULT_SPEED,
    ADRC_FAULT_TRAVEL, ADRC_FAULT_SEND, ADRC_FAULT_CONTROLLER,
    ADRC_FAULT_STOP_TIMEOUT, ADRC_FAULT_ENABLE_TIMEOUT } AdrcExperimentFault;

/** @brief Frozen run parameters; configuration is only accepted with fresh disabled feedback. */
typedef struct
{
    AdrcExperimentMode mode;
    uint32_t duration_us;
    uint32_t identify_pulse_us;
    float identify_torque_nm;
    float reference_rad_s;
    float reference_accel_rad_s2;
    float stop_accel_rad_s2;
    float torque_limit_nm;
    float torque_slew_nm_s;
    float velocity_quantum_rad_s;
    float standstill_noise_rad_s;
    float near_zero_rad_s;
    float b0;
    float wc_rad_s;
    float wo_rad_s;
    uint8_t axis_index;
} AdrcExperimentConfig;

/** @brief Trusted local evidence, never a USB assertion or a substitute for measurements. */
typedef struct
{
    uint32_t verified_flags;
    AdrcExperimentEnvironment provenance;
    float position_max_rad;
    float velocity_max_rad_s;
    float torque_max_nm;
    uint8_t axis_index;
} AdrcExperimentQualification;

/** @brief Coherent selected-axis feedback and actual previous-transmit receipt. */
typedef struct
{
    uint64_t now_us;
    uint64_t feedback_us;
    float position_rad;
    float velocity_rad_s;
    float mos_temperature_c;
    float rotor_temperature_c;
    float prev_sent_nm;
    uint8_t axis_index;
    uint8_t feedback_valid;
    uint8_t enabled;
    uint8_t driver_fault;
    uint8_t prev_sent_valid;
    uint8_t prev_send_failed;
} AdrcExperimentInput;

/** @brief Pure controller callback input; no actuator or platform access is granted. */
typedef struct
{
    AdrcExperimentMode mode;
    float reference_rad_s;
    float velocity_rad_s;
    float prev_sent_nm;
    float b0;
    float wc_rad_s;
    float wo_rad_s;
    uint8_t reset;
    uint8_t new_sample;
} AdrcControllerInput;

/** @brief Controller diagnostics accompanying raw, not yet limited, nominal torque. */
typedef struct { float torque_raw_nm; float z1; float z2; } AdrcControllerOutput;
/** @brief Returns nonzero only when the generated controller produced finite valid output. */
typedef int (*AdrcControllerCallback)(void *context, const AdrcControllerInput *input,
    AdrcControllerOutput *output);

/** @brief Requests platform actions; a request never means a frame was actually sent. */
typedef struct
{
    float torque_nm;
    uint8_t axis_index;
    uint8_t send_torque;
    uint8_t enable_requested;
    uint8_t disable_requested;
} AdrcExperimentOutput;

/** @brief Fixed 64-byte-or-smaller record; previous torque is an actual send receipt. */
typedef struct
{
    uint64_t timestamp_us;
    uint64_t feedback_us;
    float reference_rad_s;
    float position_rad;
    float velocity_rad_s;
    float torque_raw_nm;
    float torque_requested_nm;
    float torque_sent_previous_nm;
    float z1;
    float z2;
    uint32_t sequence;
    uint8_t state;
    uint8_t fault;
    uint8_t flags;
    uint8_t axis_index;
} AdrcExperimentTrace;

/** @brief Separates requested disable from fresh feedback proving actual disable. */
typedef struct
{
    AdrcExperimentState state;
    AdrcExperimentFault fault;
    uint32_t trace_count;
    uint32_t trace_overflow_count;
    uint8_t axis_index;
    uint8_t disable_requested;
    uint8_t disabled_confirmed;
    uint8_t awaiting_enable;
    uint8_t trace_frozen;
} AdrcExperimentStatus;

/** @brief Sole-owner static state; caller serializes all APIs in its existing control task. */
typedef struct
{
    AdrcExperimentConfig config;
    AdrcExperimentQualification qualification;
    AdrcExperimentStatus status;
    AdrcExperimentTrace trace[ADRC_EXPERIMENT_TRACE_CAPACITY];
    AdrcControllerCallback controller;
    void *controller_context;
    AdrcExperimentEnvironment environment;
    uint64_t last_tick_us;
    uint64_t last_feedback_us;
    uint64_t run_started_us;
    uint64_t heartbeat_us;
    uint64_t stop_started_us;
    uint64_t low_speed_started_us;
    uint64_t disable_requested_us;
    float initial_position_rad;
    float reference_rad_s;
    float previous_sent_nm;
    uint32_t next_sequence;
    uint8_t initialized;
    uint8_t configured;
    uint8_t tick_seen;
    uint8_t low_speed_tracking;
    uint8_t controller_reset;
    uint8_t previous_send_expected;
} AdrcExperiment;

/** @brief Initializes a disabled, unqualified context; caller supplies memory. */
AdrcExperimentResult adrc_experiment_init(AdrcExperiment *experiment,
    AdrcExperimentEnvironment environment, AdrcControllerCallback controller, void *context);
/** @brief Installs limits and trusted evidence only with fresh actual disabled feedback. */
AdrcExperimentResult adrc_experiment_configure(AdrcExperiment *experiment,
    const AdrcExperimentConfig *config, const AdrcExperimentQualification *qualification,
    const AdrcExperimentInput *input);
/** @brief Qualifies an explicit single axis and latches PREPARED without enabling hardware. */
AdrcExperimentResult adrc_experiment_prepare(AdrcExperiment *experiment,
    const AdrcExperimentInput *input);
/** @brief Starts one finite run from fresh near-zero disabled feedback and requests enable. */
AdrcExperimentResult adrc_experiment_run(AdrcExperiment *experiment,
    const AdrcExperimentInput *input, AdrcExperimentOutput *output);
/** @brief Refreshes the host lease; timestamps must be monotonic. */
AdrcExperimentResult adrc_experiment_heartbeat(AdrcExperiment *experiment, uint64_t now_us);
/** @brief Starts a bounded zero-reference stop; repeated stop commands are idempotent. */
AdrcExperimentResult adrc_experiment_stop(AdrcExperiment *experiment, uint64_t now_us);
/** @brief Clears a fault only using fresh selected-axis disabled feedback. */
AdrcExperimentResult adrc_experiment_clear(AdrcExperiment *experiment,
    const AdrcExperimentInput *input);
/** @brief Advances supervision at 4 ms; active delays over 6 ms cause immediate fault. */
AdrcExperimentResult adrc_experiment_tick(AdrcExperiment *experiment,
    const AdrcExperimentInput *input, AdrcExperimentOutput *output);
/** @brief Copies a consistent status; caller provides task-level serialization. */
AdrcExperimentResult adrc_experiment_get_status(const AdrcExperiment *experiment,
    AdrcExperimentStatus *status);
/** @brief Reads only frozen, disabled-confirmed run records; full logs never wrap. */
AdrcExperimentResult adrc_experiment_trace_get(const AdrcExperiment *experiment,
    uint32_t index, AdrcExperimentTrace *record);

#endif
