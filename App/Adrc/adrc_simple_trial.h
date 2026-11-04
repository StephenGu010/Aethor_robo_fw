/** @file adrc_simple_trial.h
 * @brief Finite native-coordinate commissioning; never grants full ADRC qualification.
 */
#ifndef APP_ADRC_SIMPLE_TRIAL_H
#define APP_ADRC_SIMPLE_TRIAL_H
#include "adrc_experiment.h"
#define ADRC_SIMPLE_TRACE_CAPACITY 400U
/** @brief One-shot phases; STOPPING/LOCKED always request DISABLE. */
typedef enum { ADRC_SIMPLE_IDLE=0, ADRC_SIMPLE_ENABLE, ADRC_SIMPLE_NEUTRAL_1,
 ADRC_SIMPLE_POSITIVE, ADRC_SIMPLE_NEUTRAL_2, ADRC_SIMPLE_NEGATIVE,
 ADRC_SIMPLE_NEUTRAL_3, ADRC_SIMPLE_CONTROL, ADRC_SIMPLE_STOPPING,
 ADRC_SIMPLE_DONE, ADRC_SIMPLE_LOCKED } AdrcSimplePhase;
/** @brief Physical disable evidence and trial outcome are independent. */
typedef enum { ADRC_SIMPLE_PENDING=0, ADRC_SIMPLE_SUCCESS,
 ADRC_SIMPLE_COMPLETED_NO_RESPONSE, ADRC_SIMPLE_FAILED, ADRC_SIMPLE_ABORTED, ADRC_SIMPLE_PROBE_COMPLETED } AdrcSimpleResult;
/** @brief Input carries native feedback, actual transmit receipt, and measured torque. */
typedef struct { AdrcExperimentInput feedback; float feedback_torque_nm;
 uint8_t tx_receipt_valid; uint8_t tx_kind; } AdrcSimpleInput;
/** @brief Read-only public status; safe_release requires new healthy disabled dwell. */
typedef struct { AdrcSimplePhase phase; AdrcSimpleResult result;
 AdrcExperimentFault fault; float b0; uint32_t trace_count;
 uint8_t safe_release; } AdrcSimpleStatus;
/** @brief Raw bounded log, including original CAN velocity and actual previous send. */
typedef struct { AdrcExperimentTrace sample; float feedback_torque_nm;
 uint8_t enabled; } AdrcSimpleTrace;
/** @brief Caller-owned state, serialized by the control task, with no HAL access. */
typedef struct {
 AdrcSimpleStatus status; AdrcControllerCallback controller; void *controller_context;
 AdrcSimpleTrace trace[ADRC_SIMPLE_TRACE_CAPACITY];
 uint64_t started_us, phase_us, last_tick_us, last_feedback_us, stop_us, dwell_us;
 uint64_t position_times[16]; float positions[16];
 uint64_t estimate_midpoint_us, pulse_midpoint_start_us, pulse_midpoint_end_us, pulse_integral_us;
 float position_quantum_rad, initial_position_rad, velocity_rad_s;
 float requested_nm, positive_gain, pulse_velocity_start, pulse_velocity_end, pulse_impulse;
 float controller_raw_nm, controller_z1, controller_z2;
 uint8_t history_count, history_next, estimate_ready, expected_receipt;
 uint8_t pulse_start_seen, pulse_end_seen, dwell_tracking, controller_reset;
 uint8_t disable_sent;
 /* Append configuration to preserve all existing ARM fixture member offsets. */
 uint8_t profile;
 float torque_limit_nm;
 uint32_t pulse_duration_us;
 /* Profile 4 requires a continuous actual-send plateau over all estimator support. */
 uint8_t pulse_platform_seen, pulse_platform_valid;
 float pulse_platform_reference_nm;
 /* Profile 4 returns toward the starting position; legacy profiles remain positive. */
 float control_reference_rad_s;
 /* Append running envelope; initial health and disabled dwell remain at 0.15 Nm. */
 float torque_trip_nm, torque_slew_nm_s, torque_step_nm;
 /* Profile 8 observes actual torque receipts only; reaching the cap never extends the pulse. */
 float probe_peak_nm;
 uint8_t probe_reached;
} AdrcSimpleTrial;
/** @brief Initializes an idle one-shot trial; quantum must be actual native protocol resolution. */
int adrc_simple_init(AdrcSimpleTrial *trial, AdrcControllerCallback controller,
 void *context, float position_quantum_rad);
/** @brief Select bounded profile 0..8 only while idle; initialization selects profile 0. */
int adrc_simple_set_profile(AdrcSimpleTrial *trial, uint8_t profile);
/** @brief Checks disabled input and requests ENABLE; bridge must arm its independent guard first. */
int adrc_simple_start(AdrcSimpleTrial *trial, const AdrcSimpleInput *input, AdrcExperimentOutput *output);
/** @brief Advances nominally at 4 ms: control and active profile 8 permit 3..5 ms jitter; legacy pulses permit 3..8 ms. */
void adrc_simple_tick(AdrcSimpleTrial *trial, const AdrcSimpleInput *input, AdrcExperimentOutput *output);
/** @brief Observe an active matched actual torque receipt only; never advances phases, stops, or controls. */
void adrc_simple_observe_receipt(AdrcSimpleTrial *trial, const AdrcSimpleInput *input);
/** @brief Latches user STOP; the next tick requests DISABLE even if feedback is invalid. */
void adrc_simple_abort(AdrcSimpleTrial *trial);
/** @brief Copies an existing raw trace record without changing state or qualification. */
int adrc_simple_trace_get(const AdrcSimpleTrial *trial, uint32_t index, AdrcSimpleTrace *record);
#endif
