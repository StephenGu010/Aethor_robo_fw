/** @file adrc_app_bridge.h
 * @brief Single-owner ADRC application adapter; callers serialize every API using task critical hooks.
 * @note Qualification is rejected when decoded torque steps cannot satisfy one 4 ms slew increment,
 *       when even half a torque LSB exceeds the amplitude cap, or when supplied velocity resolution
 *       understates the protocol grid. An unencodable command disables; limits are never expanded.
 *       The grid generally has no exact zero. CAN receipts are nominal commands, not measured torque.
 */
#ifndef APP_ADRC_APP_BRIDGE_H
#define APP_ADRC_APP_BRIDGE_H
#include "adrc_bench.h"
#include "adrc_feedback_adapter.h"
#include "adrc_simple_trial.h"
#include "motor_runtime.h"
#include "motor_capability_collector.h"
#include "adrc_shaft_pilot_adapter.h"

#define ADRC_DISABLED_CAPTURE_CAPACITY (64U)
/** @brief Raw receive-side observation; lower bound is not a physical sampling timestamp. */
typedef struct
{
    uint64_t observed_us;
    uint64_t rx_lower_us;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
} AdrcDisabledCaptureSample;

/** @brief Bounded DISABLE-only diagnostic, never a producer of motion qualification. */
typedef struct
{
    AdrcDisabledCaptureSample samples[ADRC_DISABLED_CAPTURE_CAPACITY];
    MotorDiscoveryResult discovery;
    uint64_t started_us;
    uint64_t first_request_us;
    uint64_t last_request_us;
    uint32_t request_id;
    uint32_t owner_epoch; /* Captured local lifetime; no USB binding claims. */
    uint16_t count;
    uint16_t observed_proven_count; /* Genuine accepted motor-7 RX upper bounds, never repeated service time. */
    uint16_t sent;
    uint8_t axis;
    uint8_t state; /* 0 idle, 1 collecting, 2 complete, 3 aborted. */
    uint8_t reason; /* 1 transport, 2 deadline, 3 feedback, 4 binding, 5 stop. */
} AdrcDisabledCapture;

/** @brief Trusted platform hooks; callers cannot supply evidence over USB. */
typedef struct
{
    uint8_t (*arm)(const CanFrame *, uint32_t, uint32_t);
    uint8_t (*observe)(uint32_t *, uint32_t *);
    uint8_t (*ack_disabled)(void);
} AdrcStopGuardHooks;

/** @brief Trusted platform lease hooks, separate from the nonrenewable diagnostic guard. */
typedef struct
{
    uint8_t (*arm_trial)(const CanFrame *, uint32_t);
    uint8_t (*renew_trial)(uint32_t);
} AdrcSimpleGuardHooks;

/** @brief DISABLE-only task-stall selftest; state 5 is diagnostic proof, never qualification. */
typedef struct
{
    AdrcStopGuardHooks hooks;
    uint64_t started_us;
    uint64_t request_us;
    uint64_t disabled_rx_us;
    uint64_t last_request_us;
    uint32_t request_id;
    uint32_t deadline_ms;
    uint32_t owner_epoch; /* Local selftest lifetime, required by shaft pilot admission. */
    uint32_t fired_ms;
    uint8_t state; /* 0 idle, 1 preflight, 2 wait, 3 stalled, 4 confirm, 5 pass, 6 fail. */
    uint8_t reason; /* 1 transport, 2 time, 3 feedback, 4 binding, 5 stop, 6 timer. */
    uint8_t pause_ms;
    uint8_t receipt;
    uint8_t requests;
} AdrcStopGuardTest;

/** @brief Bounded commissioning evidence; no field is a motion qualification flag. */
typedef struct
{
    MotorDiscoveryResult discovery;
    uint64_t started_us;
    uint64_t phase_started_us;
    uint64_t first_request_us;
    uint64_t last_request_us;
    uint64_t parameter_rx_us;
    uint64_t enable_request_us;
    uint64_t enabled_rx_us;
    uint64_t last_command_us;
    uint64_t last_command_observed_us;
    uint64_t watchdog_rx_us;
    uint64_t watchdog_observed_us;
    uint64_t disabled_rx_us;
    uint64_t finished_us;
    MotorJointFeedback failure_feedback; /* Immutable first-rejection snapshot, not live proof. */
    uint64_t failure_observed_us; /* Zero means STOP arrived without a service timestamp. */
    uint32_t failure_guard_deadline_ms;
    uint32_t failure_guard_fired_ms;
    uint8_t failure_phase;
    uint8_t failure_guard_state;
    uint8_t failure_receipt;
    uint32_t request_id;
    uint32_t parameter_value;
    uint8_t state; /* 1..14 active; 15 pass; 16 locked failure; 17 safely restored failure. */
    uint8_t reason; /* First failure: 1 TX, 2 deadline, 3 feedback, 4 binding, 5 STOP, 6 guard, 7 readback. */
    uint8_t requests;
    uint8_t receipt;
    uint8_t guard_armed;
    uint8_t enabled_seen;
    uint8_t watchdog_seen;
    uint8_t timeout_restored;
} AdrcWatchdogProbe;

/** @brief Fixed bridge storage with a single expiring output and matched transmit receipt. */
typedef struct AdrcAppBridge
{
    AdrcBench bench;
    AdrcShaftPilotAdapter shaft;
    MotorCapabilityCollector capabilities;
    MotorDiscoveryResult capability_discovery;
    uint32_t capability_epoch; /* Local owner/discovery lifetime, never supplied over USB. */
    uint64_t capability_prepare_started_us;
    uint64_t capability_prepare_submit_us; /* First dedicated hardware-buffer acceptance, not physical TX time. */
    uint8_t capability_prepare_state; /* 0 idle, 1 DISABLE preparation, 2 reads started, 3 preparation failed. */
    uint8_t capability_prepare_receipt;
    uint8_t capability_terminal_closed; /* Normal STOP/release closes this diagnostic lifetime, retaining COMPLETE. */
    uint8_t capability_standalone_owner;
    uint8_t (*capability_owner_active)(void); /* Trusted dynamic owner query, called under task critical. */
    AdrcDisabledCapture capture;
    AdrcStopGuardTest guard_test;
    AdrcWatchdogProbe watchdog;
    AdrcSimpleTrial simple;
    AdrcSimpleGuardHooks simple_hooks;
    MotorDiscoveryResult simple_discovery;
    uint64_t simple_lease_feedback_us;
    uint64_t simple_prepare_us; /* Bounded disabled-feedback refresh before any ENABLE. */
    uint32_t simple_request_id;
    uint8_t simple_pending;
    uint8_t simple_receipt;
    uint8_t simple_receipt_kind;
    uint8_t simple_guard_armed;
    uint8_t simple_guard_ack;
    uint8_t simple_bridge_fault; /* 1 binding, 2 guard, 3 encoding/admission. */
    AdrcFeedbackAdapter feedback_adapter;
    MotorDiscoveryResult observed_discovery;
    float observed_mapping[3];
    AdrcExperimentConfig evidence_config;
    AdrcInterfaceProfile observed_profile;
    uint8_t observed_axis;
    uint8_t observation_valid;
    uint8_t evidence_bound;
    MotorRuntime *runtime;
    ProtocolEngine *engine;
    CanFrame frame;
    float decoded_torque_nm;
    float receipt_torque_nm;
    uint64_t next_feedback_query_us;
    uint64_t last_feedback_us;
    uint64_t watchdog_quarantine_until_us;
    uint32_t feedback_query_count;
    uint32_t feedback_sample_count;
    uint32_t feedback_interval_min_us;
    uint32_t feedback_interval_max_us;
    uint8_t frame_kind;
    uint8_t frame_pending;
    uint8_t awaiting_receipt;
    uint8_t receipt_valid;
    uint8_t send_failed;
    uint8_t stop_pending;
    uint8_t release_disable_pending;
    uint8_t discovery_axis;
    uint8_t initialized;
} AdrcAppBridge;
/** @brief Binds trusted board timer operations before requests may enter the simple trial. */
void adrc_app_bridge_set_simple_guard_hooks(AdrcAppBridge *bridge, const AdrcSimpleGuardHooks *hooks);
/** @brief Initializes hardware provenance with no qualification and registers the protocol gateway. */
AdrcExperimentResult adrc_app_bridge_init(AdrcAppBridge *bridge, MotorRuntime *runtime,
    ProtocolEngine *engine, AdrcControllerCallback controller, void *controller_context);
/** @brief Initializes ADRC without replacing the LCD-owned motor discovery sequence. */
AdrcExperimentResult adrc_app_bridge_init_deferred(AdrcAppBridge *bridge, MotorRuntime *runtime,
    ProtocolEngine *engine, AdrcControllerCallback controller, void *controller_context);
/** @brief Starts selected-axis read-only discovery after exclusive ownership is granted. */
MotorRuntimeStatus adrc_app_bridge_start_selected_discovery(AdrcAppBridge *bridge, uint8_t axis_index);
/** @brief Runs only the selected-axis 4 ms owner and replaces the expiring output slot. */
uint8_t adrc_app_bridge_service(AdrcAppBridge *bridge, uint64_t timestamp_us);
/** @brief Rejects all legacy mutation paths before they can reach their command queues. */
ProtocolEngineStatus adrc_app_bridge_process_line(AdrcAppBridge *bridge, const char *line,
    size_t length, uint64_t timestamp_us, ProtocolOutputBatch *output);
/** @brief Pops the sole current frame; kind is enable=0, nominal torque=1, disable=2. */
uint8_t adrc_app_bridge_pop_frame(AdrcAppBridge *bridge, CanFrame *frame,
    uint8_t *kind, float *decoded_torque_nm);
/** @brief Matches a bus transmission receipt; nominal decoded torque is not measured motor torque. */
void adrc_app_bridge_report_transmit(AdrcAppBridge *bridge, uint8_t kind,
    float decoded_torque_nm, uint8_t succeeded);
/** @brief Records a matched real completion upper bound; old untimed API cannot prove pilot sends. */
void adrc_app_bridge_report_transmit_at(AdrcAppBridge *bridge, uint8_t kind,
    float decoded_torque_nm, uint8_t succeeded, uint64_t completed_observed_us);
/** @brief Latches a local safety stop for the control owner and revokes unsent actuation. */
void adrc_app_bridge_request_stop(AdrcAppBridge *bridge);
/** @brief Forces an explicit disable frame for LCD handback even if ADRC was already disabled. */
void adrc_app_bridge_request_release_disable(AdrcAppBridge *bridge);
/** @brief Discards only stale selected-axis output after measured handback to LCD. */
void adrc_app_bridge_complete_release(AdrcAppBridge *bridge);
/** @brief Latches a transport failure and selected-axis disable request without touching other motors. */
void adrc_app_bridge_report_fault(AdrcAppBridge *bridge);
/** @brief Returns only read-only discovery while no experiment or frame is active. */
MotorRuntimeStatus adrc_app_bridge_next_discovery(AdrcAppBridge *bridge, uint64_t timestamp_us,
    CanFrame *frame);
/** @brief Installs local measured evidence only after matching discovered MIT identity/ranges and feedback. */
AdrcExperimentResult adrc_app_bridge_set_evidence(AdrcAppBridge *bridge,
    const AdrcExperimentConfig *config, const AdrcExperimentQualification *qualification,
    uint64_t timestamp_us);
/** @brief Consumes only the dedicated TIMEOUT transaction replies, never granting qualification. */
uint8_t adrc_app_bridge_accept_watchdog_parameter(AdrcAppBridge *bridge,
    const CanFrame *frame, uint64_t rx_lower_us);
/** @brief Routes fixed capability responses before ordinary MotorBank decode, including late replies. */
uint8_t adrc_app_bridge_accept_capability_parameter(AdrcAppBridge *bridge,
    const CanFrame *frame, uint64_t rx_lower_us, uint64_t received_observed_us);
/** @brief Records only exact capability request completion under existing owner critical hooks. */
void adrc_app_bridge_report_capability_transmit(AdrcAppBridge *bridge,
    const CanFrame *frame, uint8_t succeeded, uint64_t completed_observed_us);
/** @brief Records only a successfully submitted dedicated motor-7 preparation DISABLE causal anchor. */
void adrc_app_bridge_note_capability_disable_submit(AdrcAppBridge *bridge, uint64_t submitted_us);
/** @brief Immediately revokes preparation/reads after newly decoded unhealthy feedback or owner loss. */
void adrc_app_bridge_check_capability_health(AdrcAppBridge *bridge, uint64_t observed_us);
#endif
