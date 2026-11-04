/** @file shaft_pilot.h
 * @brief HAL-free finite output-shaft pilot; no control, ESO, model token, USB or qualification.
 * @note All identity/baseline/guard/receipt inputs must come from a serialized trusted local bridge.
 *       This plain C API is not an authentication boundary and must never accept USB claims directly.
 */
#ifndef OUTPUT_SHAFT_PILOT_H
#define OUTPUT_SHAFT_PILOT_H
#include <stdint.h>
#define SHAFT_PILOT_TRACE_CAPACITY 512U
#define SHAFT_PILOT_HISTORY_CAPACITY 16U
/* Approved fixed observe values; output speed is 395 rpm / 2 converted to rad/s.
 * Native angle limits are trigger/observation budgets, not physical stopping guarantees. */
#define SHAFT_OBSERVE_SPEED_RAD_S 20.6821516F
#define SHAFT_OBSERVE_ANGLE_NATIVE_RAD 0.523598776F
#define SHAFT_OBSERVE_REVERSE_NATIVE_RAD 0.0872664626F
#define SHAFT_OBSERVE_ACTIVE_NATIVE_RAD 1.57079633F
#define SHAFT_OBSERVE_STOP_NATIVE_RAD 3.14159265F
#define SHAFT_OBSERVE_RAMP_US 900000U
#define SHAFT_OBSERVE_PLATEAU_US 200000U
/* Platform range reserve is a conditional observation budget, not a stopping-distance guarantee. */
#define SHAFT_PLATFORM_RESERVE_NATIVE_RAD 3.14159265F
#define SHAFT_PLATFORM_LOOKAHEAD_US 25000U
/* 宽档最低预留两转子圈；1058ms已含ENABLE64+NEUTRAL69+RAMP900+条件运动25ms。 */
#define SHAFT_PLATFORM_WIDE_RESERVE_NATIVE_RAD 12.5663706F
#define SHAFT_PLATFORM_WIDE_ENTRY_US 1058000U
/* 70%额定速度档独立冻结；四转子圈与1058ms仍是观察预算，不是物理停车保证。 */
#define SHAFT_PLATFORM70_SPEED_RAD_S 28.9550123F
#define SHAFT_PLATFORM70_RESERVE_NATIVE_RAD 25.1327412F
#define SHAFT_PLATFORM70_ENTRY_US 1058000U
/* 100%额定速度仅配合1Nm上限；九转子圈预留仍是条件观察预算。 */
#define SHAFT_PLATFORM100_SPEED_RAD_S 41.3643036F
#define SHAFT_PLATFORM100_RESERVE_NATIVE_RAD 56.5486679F
#define SHAFT_PLATFORM100_ENTRY_US 1058000U

/** @brief Finite one-direction transaction; LOCKED never resumes activity. */
typedef enum { SHAFT_PHASE_IDLE=0, SHAFT_PHASE_ENABLE, SHAFT_PHASE_NEUTRAL,
 SHAFT_PHASE_RAMP, SHAFT_PHASE_STOPPING, SHAFT_PHASE_DONE, SHAFT_PHASE_LOCKED } ShaftPilotPhase;
/** @brief Exactly one caller action; a torque request still requires Motor's true integer-grid selection. */
typedef enum { SHAFT_ACTION_NONE=0, SHAFT_ACTION_ENABLE, SHAFT_ACTION_TORQUE,
 SHAFT_ACTION_DISABLE } ShaftPilotAction;
/** @brief Endpoint is a trial observation, independent of disabled release evidence. */
typedef enum { SHAFT_END_PENDING=0, SHAFT_END_TARGET, SHAFT_END_TIME,
 SHAFT_END_FAULT, SHAFT_END_ABORT } ShaftPilotOutcome;
/** @brief First diagnostic is immutable even if healthy stopped evidence later returns. */
typedef enum { SHAFT_FAULT_NONE=0, SHAFT_FAULT_BINDING, SHAFT_FAULT_GUARD,
 SHAFT_FAULT_PERIOD, SHAFT_FAULT_FEEDBACK, SHAFT_FAULT_DRIVER, SHAFT_FAULT_TEMPERATURE,
 SHAFT_FAULT_SPEED, SHAFT_FAULT_POSITION, SHAFT_FAULT_TRAVEL, SHAFT_FAULT_RECEIPT,
 SHAFT_FAULT_ENABLE_TIMEOUT, SHAFT_FAULT_NEUTRAL_TIMEOUT, SHAFT_FAULT_ACTIVE_TIMEOUT,
 SHAFT_FAULT_STOP_TIMEOUT, SHAFT_FAULT_TRACE } ShaftPilotFault;
/** @brief Explicit driver-state decoding; unknown states never satisfy health. */
typedef enum { SHAFT_DRIVER_UNKNOWN=0, SHAFT_DRIVER_DISABLED, SHAFT_DRIVER_ENABLED } ShaftDriverState;

/** @brief Captured current-session disabled baseline; caller supplies all values, no historical defaults. */
typedef struct {
 uint64_t binding_id, session_id;
 uint32_t point_count;
 float span64_shaft_rad, velocity32_abs_peak_rad_s;
 float raw_zero_center_rad_s, torque_abs_peak_nm;
} ShaftPilotBaseline;
/** @brief Frozen readback ranges; native rotor position is converted, raw speed/torque retain protocol units.
 * @note Output-side raw-speed integration is supported by observations, not full timing/physical calibration.
 */
typedef struct {
 uint8_t motor_id;
 uint64_t binding_id, session_id;
 float gear_ratio, position_max_native_rad, velocity_max_rad_s, torque_max_nm;
 ShaftPilotBaseline baseline;
} ShaftPilotConfig;
/** @brief One finite direction with bounded requested cap, slew and step; no automatic escalation. */
typedef struct { int direction; float cap_nm, slew_nm_s, step_nm; } ShaftPilotPlan;
/** @brief Native feedback with receive lower bound and observation upper bound, not sample time. */
typedef struct {
 uint8_t valid;
 uint64_t rx_us, observed_us;
 float native_position_rad, raw_velocity_rad_s, torque_nm;
 float mos_temperature_c, rotor_temperature_c;
 ShaftDriverState driver_state;
 uint32_t driver_fault;
} ShaftPilotFeedback;
/** @brief Successful real send completion matched to action id; time is an observation upper bound.
 * @note Board clock has 1ms resolution; this is not an exact physical bus transmit timestamp.
 *       Synthetic fixtures are not physical sends. Bridge must preserve its final receipt for STOP tick.
 */
typedef struct {
 uint8_t valid, success;
 ShaftPilotAction kind;
 uint64_t command_id, completed_observed_us;
 float decoded_torque_nm;
} ShaftPilotReceipt;
/** @brief Trusted single-scheduler snapshot; guard-fired/acked are reported only from the actual guard. */
typedef struct {
 uint64_t now_us, binding_id, session_id;
 uint8_t motor_id, guard_armed, guard_live, guard_fired, guard_acked, stop_requested;
 ShaftPilotFeedback feedback;
 ShaftPilotReceipt receipt;
} ShaftPilotInput;
/** @brief One output; Motor must apply these amplitude/step bounds and return its actual decoded receipt. */
typedef struct {
 ShaftPilotAction action;
 uint8_t motor_id, previous_sent_valid;
 uint64_t binding_id, session_id, command_id, issued_us;
 /* First neutral uses nominal zero as the grid selection reference only; valid becomes true on receipt. */
 float torque_nm, previous_sent_nm, maximum_step_nm, cap_nm;
} ShaftPilotOutput;
/** @brief Public observations; caller_next_transaction_ready is a caller condition, never qualification. */
typedef struct {
 ShaftPilotPhase phase;
 ShaftPilotOutcome outcome;
 ShaftPilotFault first_fault;
 uint32_t trace_count;
 uint8_t safe_release, caller_next_transaction_ready, velocity_ready;
 uint8_t arrival_interval_conditional, internal_sampling_unknown;
 float quantum_native_rad, quantum_shaft_rad, quantum_velocity_rad_s, quantum_torque_nm;
 float velocity_rx32_rad_s, initial_stop_torque_gate_nm, running_torque_gate_nm;
 float last_successful_decoded_torque_nm, peak_successful_decoded_torque_nm;
 uint64_t window_rx_old_us, window_observed_old_us, window_rx_new_us, window_observed_new_us;
 uint64_t window_tmin_us, window_tmax_us;
} ShaftPilotStatus;
/** @brief Full raw snapshot, action and causal velocity for offline reconstruction; never overwritten. */
typedef struct {
 ShaftPilotInput input;
 ShaftPilotOutput output;
 ShaftPilotPhase phase;
 ShaftPilotOutcome outcome;
 ShaftPilotFault first_fault;
 float shaft_position_rad, velocity_rx32_rad_s;
 uint8_t velocity_ready, arrival_interval_conditional, internal_sampling_unknown;
 uint64_t window_rx_old_us, window_observed_old_us, window_rx_new_us, window_observed_new_us;
 uint64_t window_tmin_us, window_tmax_us;
} ShaftPilotTrace;
/** @brief Fixed causal history entry; arrival bounds are retained independently of nominal RX difference. */
typedef struct { uint64_t rx_us, observed_us; float shaft_position_rad; } ShaftPilotSample;
/** @brief Caller-owned core; serialize all calls, retain until evidence exported, no heap/HAL dependencies.
 * @note Low-profile state is trusted local configuration, never a USB amplitude or qualification claim.
 */
typedef struct {
 ShaftPilotConfig config;
 ShaftPilotPlan plan;
 ShaftPilotStatus status;
 ShaftPilotTrace trace[SHAFT_PILOT_TRACE_CAPACITY];
 ShaftPilotSample history[SHAFT_PILOT_HISTORY_CAPACITY];
 ShaftPilotFeedback previous_feedback;
 ShaftPilotOutput pending;
 uint64_t started_us, phase_us, last_tick_us, stop_us, dwell_us, command_sequence;
 uint64_t plateau_observed_us;
 float origin_shaft_rad, origin_native_rad, last_sent_nm, dwell_min_rad, dwell_max_rad;
 uint8_t initialized, history_count, history_next, feedback_seen, torque_seen;
 uint8_t disable_confirmed, dwell_tracking, trace_frozen;
 uint8_t low_profile, plateau_seen, visible_profile, observe_profile, platform_profile;
 uint8_t platform_wide_profile, platform70_profile, platform100_profile;
} ShaftPilot;

/** @brief Validate and freeze caller configuration/plan; no real Gr or baseline is defaulted. */
int shaft_pilot_init(ShaftPilot *pilot, const ShaftPilotConfig *config, const ShaftPilotPlan *plan);
/** @brief Freeze fixed low diagnostic cap .35Nm, slew 4Nm/s and step .016Nm; direction only.
 * @note Original init always selects default behavior. Low TIME requires a matching decoded platform
 *       receipt and 80ms observation; ABORT never grants readiness or model qualification.
 */
int shaft_pilot_init_low(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief Fixed VMAX=5-only .20Nm/4Nm/s/.016Nm diagnostic with the unchanged 80ms low platform.
 * @note Lower excitation precedes the observed .29Nm abort; it does not qualify velocity units or a model.
 */
int shaft_pilot_init_fine(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief Freeze visible diagnostic .50Nm/2Nm/s/.008Nm; native .15rad target, actual platform <=200ms.
 * @note This observation-only angle has no independent physical calibration; qualification remains external.
 */
int shaft_pilot_init_visible(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief Fixed 1.75Nm/4Nm/s/.016Nm observe with VMAX200, rated-half speed and 30-degree native trigger.
 * @note Real decoded platform holds 200ms within a 900ms ramp deadline. No automatic escalation or qualification.
 */
int shaft_pilot_init_observe(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief Fixed rated-half platform with no relative angle target, range reserve and 200ms true plateau.
 * @note Range/velocity lookahead may ABORT before the plateau; no PMAX change or physical stopping guarantee.
 */
int shaft_pilot_init_platform(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief 冻结PMAX512/VMAX200/TMAX10宽平台；两圈停止余量和1058ms本方向入口预算。
 * @note 与observe/platform共享既有速度、力矩、平台和全部保护；不宣称物理停止距离或模型资格。
 */
int shaft_pilot_init_platform_wide(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief 冻结PMAX1024/VMAX200/TMAX10、70%额定速度和四圈最低预留的新独立平台。
 * @note 力矩/周期/真实平台/反向/反馈/独立STOP保护沿用；不授予物理停车保证或模型资格。
 */
int shaft_pilot_init_platform70(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief 冻结PMAX1024/VMAX200/TMAX10、395rpm速度门、1Nm上限和九圈最低预留。
 * @note 真实200ms平台与所有旧保护保持；只允许方向输入，不授予物理停车保证或资格。
 */
int shaft_pilot_init_platform100(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief Start IDLE once from bound fresh disabled health with headroom and armed live guard. */
int shaft_pilot_start(ShaftPilot *pilot, const ShaftPilotInput *input, ShaftPilotOutput *output);
/** @brief Consume final receipt before STOP/guard/health protection, then produce one action.
 * @note Serialize input snapshot before clearing bridge outstanding slot; stopped recovery keeps faults.
 */
void shaft_pilot_tick(ShaftPilot *pilot, const ShaftPilotInput *input, ShaftPilotOutput *output);
/** @brief Copy existing raw evidence by index without wrapping, changing state or granting qualification. */
int shaft_pilot_trace_get(const ShaftPilot *pilot, uint32_t index, ShaftPilotTrace *record);
/** @brief Copy status without exposing a mutable status pointer to the caller. */
int shaft_pilot_status_get(const ShaftPilot *pilot, ShaftPilotStatus *status);
#endif
