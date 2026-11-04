/** @file adrc_app_bridge.c
 * @brief Selected-axis protocol, feedback mapping and actual CAN-transmission receipt adapter.
 */
#include "adrc_app_bridge.h"
#include "motor_adrc_command.h"
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define ADRC_APP_BRIDGE_FEEDBACK_QUERY_PERIOD_US (4000ULL)

/** @brief Rejects nonfinite mapping and receipt values without relying on a platform math library. */
static uint8_t finite_value(float value)
{
    return (uint8_t)(value == value && value <= FLT_MAX && value >= -FLT_MAX);
}

/** @brief Formats one bounded read-only bridge response with a mandatory line terminator. */
static ProtocolEngineStatus format_query(ProtocolOutputBatch *output,
    ProtocolEngineStatus status, const char *format, ...)
{
    int length;
    va_list arguments;
    memset(output, 0, sizeof(*output));
    va_start(arguments, format);
    length = vsnprintf(output->messages[0].data, PROTOCOL_ENGINE_MESSAGE_CAPACITY - 2U,
        format, arguments);
    va_end(arguments);
    if (length < 0 || (unsigned int)length >= PROTOCOL_ENGINE_MESSAGE_CAPACITY - 2U)
    { return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL; }
    output->messages[0].data[length++] = '\r';
    output->messages[0].data[length++] = '\n';
    output->messages[0].data[length] = '\0';
    output->messages[0].length = (uint16_t)length;
    output->messages[0].priority = PROTOCOL_OUTPUT_HIGH_PRIORITY;
    output->count = 1U;
    return status;
}

/** @brief Revokes local evidence without erasing the first fault or an active disable obligation. */
static void revoke_evidence(AdrcAppBridge *bridge)
{
    bridge->bench.trusted_qualification.verified_flags = 0U;
    bridge->bench.experiment.qualification.verified_flags = 0U;
    bridge->bench.experiment.configured = 0U;
    bridge->evidence_bound = 0U;
    if (bridge->bench.experiment.status.state == ADRC_STATE_PREPARED)
    { bridge->bench.experiment.status.state = ADRC_STATE_DISABLED; }
}

/** @brief Compares discovery facts explicitly, ignoring struct padding and unrelated sequence state. */
static uint8_t same_discovery(const MotorDiscoveryResult *left, const MotorDiscoveryResult *right)
{
    return (uint8_t)(left->verified_fields_mask == right->verified_fields_mask &&
        left->observed_master_id == right->observed_master_id &&
        left->observed_esc_id == right->observed_esc_id &&
        left->observed_control_mode == right->observed_control_mode &&
        left->hardware_version == right->hardware_version &&
        left->software_version == right->software_version && left->sub_version == right->sub_version &&
        left->communication_timeout_raw == right->communication_timeout_raw &&
        left->ranges.position_max_rad == right->ranges.position_max_rad &&
        left->ranges.velocity_max_rad_s == right->ranges.velocity_max_rad_s &&
        left->ranges.torque_max_nm == right->ranges.torque_max_nm);
}

/** @brief Observes bindings even while disabled; any changed coordinates invalidate history and trust. */
static void observe_binding(AdrcAppBridge *bridge)
{
    uint8_t axis = bridge->bench.draft_config.axis_index;
    const MotorDiscoveryResult *discovery;
    if (axis >= ARM_JOINT_COUNT)
    { revoke_evidence(bridge); adrc_feedback_adapter_reset(&bridge->feedback_adapter); return; }
    discovery = &bridge->runtime->discovery.results[axis];
    if (bridge->observation_valid && (axis != bridge->observed_axis ||
        bridge->observed_profile != bridge->bench.draft_config.interface_profile ||
        memcmp(bridge->observed_mapping, bridge->bench.mapping, sizeof(bridge->observed_mapping)) != 0 ||
        !same_discovery(&bridge->observed_discovery, discovery)))
    {
        revoke_evidence(bridge);
        adrc_feedback_adapter_reset(&bridge->feedback_adapter);
    }
    if (bridge->evidence_bound && !adrc_experiment_config_equal(&bridge->evidence_config, &bridge->bench.draft_config)) { revoke_evidence(bridge); }
    bridge->observed_axis = axis;
    bridge->observed_profile = bridge->bench.draft_config.interface_profile;
    bridge->observed_discovery = *discovery;
    memcpy(bridge->observed_mapping, bridge->bench.mapping, sizeof(bridge->observed_mapping));
    bridge->observation_valid = 1U;
}

/** @brief Builds fresh output-shaft feedback; no draft mapping is promoted to measured evidence. */
static void make_input(AdrcAppBridge *bridge, uint64_t timestamp_us, AdrcInterfaceProfile profile,
    AdrcExperimentInput *input)
{
    MotorFeedbackSnapshot snapshot;
    const MotorJointFeedback *feedback;
    uint8_t axis = bridge->bench.draft_config.axis_index;
    observe_binding(bridge);
    memset(input, 0, sizeof(*input));
    memset(&snapshot, 0, sizeof(snapshot));
    input->axis_index = axis;
    input->now_us = timestamp_us;
    input->prev_sent_nm = bridge->receipt_torque_nm;
    input->prev_sent_valid = bridge->receipt_valid;
    input->prev_send_failed = bridge->send_failed;
    if (axis >= ARM_JOINT_COUNT) { return; }
    if (motor_runtime_get_snapshot(bridge->runtime, timestamp_us, 8000ULL, &snapshot) != MOTOR_RUNTIME_STATUS_OK)
    { return; }
    feedback = &snapshot.joints[axis];
    input->feedback_valid = (uint8_t)((snapshot.valid_joint_mask & (1U << axis)) != 0U);
    input->feedback_us = feedback->timestamp_us;
    input->position_rad = feedback->position_rad * bridge->bench.mapping[0];
    input->velocity_rad_s = feedback->velocity_rad_s * bridge->bench.mapping[1];
    input->raw_velocity_rad_s = input->velocity_rad_s;
    if (input->feedback_valid)
    {
        float position_quantum = bridge->runtime->discovery.results[axis].ranges.position_max_rad *
            fabsf(bridge->bench.mapping[0]) * (2.0F / 65535.0F);
        input->velocity_estimate_ready = adrc_feedback_adapter_update(&bridge->feedback_adapter,
            timestamp_us, input->feedback_us, input->position_rad, input->raw_velocity_rad_s, position_quantum);
    }
    else { adrc_feedback_adapter_reset(&bridge->feedback_adapter); }
    if (profile == ADRC_INTERFACE_POSITION32_QUANTIZED)
    { input->velocity_rad_s = bridge->feedback_adapter.velocity_rad_s; }
    input->mos_temperature_c = feedback->mos_temperature_c;
    input->rotor_temperature_c = feedback->rotor_temperature_c;
    input->enabled = (uint8_t)(feedback->driver_state != S3519_DRIVER_STATE_DISABLED);
    input->driver_fault = (uint8_t)(feedback->fault_flags != 0U ||
        (feedback->driver_state != S3519_DRIVER_STATE_DISABLED &&
         feedback->driver_state != S3519_DRIVER_STATE_ENABLED));
}

/** @brief Resets selected-axis feedback timing evidence without altering motor or supervisor state. */
static void reset_feedback_statistics(AdrcAppBridge *bridge)
{
    bridge->next_feedback_query_us = 0ULL;
    bridge->last_feedback_us = 0ULL;
    bridge->feedback_query_count = 0U;
    bridge->feedback_sample_count = 0U;
    bridge->feedback_interval_min_us = UINT32_MAX;
    bridge->feedback_interval_max_us = 0U;
}

/** @brief Records each new selected-axis sample once for commissioning interval evidence. */
static void observe_feedback(AdrcAppBridge *bridge, const AdrcExperimentInput *input)
{
    uint64_t interval_us;
    if (input->feedback_valid == 0U || input->feedback_us == 0ULL ||
        input->feedback_us == bridge->last_feedback_us) { return; }
    if (bridge->last_feedback_us != 0ULL && input->feedback_us > bridge->last_feedback_us)
    {
        interval_us = input->feedback_us - bridge->last_feedback_us;
        if (interval_us <= UINT32_MAX)
        {
            uint32_t bounded_interval_us = (uint32_t)interval_us;
            if (bounded_interval_us < bridge->feedback_interval_min_us)
            { bridge->feedback_interval_min_us = bounded_interval_us; }
            if (bounded_interval_us > bridge->feedback_interval_max_us)
            { bridge->feedback_interval_max_us = bounded_interval_us; }
        }
    }
    bridge->last_feedback_us = input->feedback_us;
    if (bridge->feedback_sample_count != UINT32_MAX) { ++bridge->feedback_sample_count; }
}

/** @brief Requires discovered identity, MIT mode and all protocol ranges for one selected motor. */
static uint8_t mit_discovered(const AdrcAppBridge *bridge, uint8_t axis)
{
    const MotorDiscoveryResult *discovery;
    uint16_t required = MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK | MOTOR_DISCOVERY_MODE_FIELDS_MASK |
        MOTOR_DISCOVERY_RANGE_FIELDS_MASK;
    if (axis >= ARM_JOINT_COUNT) { return 0U; }
    discovery = &bridge->runtime->discovery.results[axis];
    return (uint8_t)((discovery->verified_fields_mask & required) == required &&
        discovery->observed_control_mode == 1U);
}

/** @brief Packs exactly one selected special command; unknown mode cannot authorize an enable. */
static uint8_t build_special(AdrcAppBridge *bridge, uint8_t axis, uint8_t enable)
{
    MotorEmergencyFrameBatch batch;
    S3519ControlMode mode = S3519_CONTROL_MODE_MIT;
    if (axis >= ARM_JOINT_COUNT || (enable && !mit_discovered(bridge, axis))) { return 0U; }
    if (!enable && bridge->runtime->discovery.results[axis].observed_control_mode == 2U)
    { mode = S3519_CONTROL_MODE_POSITION_VELOCITY; }
    if (motor_runtime_build_mode_command_batch(bridge->runtime, mode,
        enable ? S3519_MODE_COMMAND_ENABLE : S3519_MODE_COMMAND_DISABLE,
        (uint8_t)(1U << axis), &batch) != MOTOR_RUNTIME_STATUS_OK || batch.count != 1U)
    { return 0U; }
    bridge->frame = batch.frames[0];
    bridge->frame_kind = enable ? 0U : 2U;
    bridge->decoded_torque_nm = 0.0F;
    bridge->frame_pending = 1U;
    return 1U;
}

/** @brief Checks owner, binding and all observed health fields; freshness is required only before reads start. */
static uint8_t capability_health(const AdrcAppBridge *bridge, uint64_t timestamp_us,
    uint8_t require_binding, uint8_t require_fresh)
{
    const MotorJointFeedback *feedback = &bridge->runtime->bank.motors[6].feedback;
    uint8_t owner = bridge->capability_owner_active != NULL ? bridge->capability_owner_active() :
        bridge->capability_standalone_owner;
    if (!owner || bridge->bench.draft_config.axis_index != 6U || !mit_discovered(bridge, 6U) ||
        bridge->runtime->discovery.results[6].observed_esc_id != 7U ||
        bridge->runtime->discovery.results[6].observed_master_id != 23U ||
        bridge->stop_pending || bridge->release_disable_pending || bridge->bench.gateway.stop_pending ||
        bridge->bench.experiment.status.state != ADRC_STATE_DISABLED ||
        !bridge->runtime->bank.motors[6].feedback_valid || !feedback->timestamp_us ||
        feedback->timestamp_us > timestamp_us || feedback->driver_state != S3519_DRIVER_STATE_DISABLED ||
        feedback->fault_flags || !finite_value(feedback->position_rad) || !finite_value(feedback->velocity_rad_s) ||
        !finite_value(feedback->torque_nm) || !finite_value(feedback->mos_temperature_c) ||
        !finite_value(feedback->rotor_temperature_c) || feedback->mos_temperature_c >= 50.0F ||
        feedback->rotor_temperature_c >= 50.0F) { return 0U; }
    if (require_binding && (bridge->capability_terminal_closed || bridge->capabilities.completed_revoked ||
        bridge->capabilities.epoch != bridge->capability_epoch ||
        !same_discovery(&bridge->capability_discovery, &bridge->runtime->discovery.results[6]))) { return 0U; }
    return (uint8_t)(!require_fresh || timestamp_us - feedback->timestamp_us <= 8000ULL);
}

/** @brief Fails preparation or reads and creates the existing immediate STOP/DISABLE obligation. */
static void fail_capability(AdrcAppBridge *bridge, MotorCapabilityReason reason)
{
    if (bridge->capability_prepare_state == 1U) { bridge->capability_prepare_state = 3U; }
    motor_capability_abort(&bridge->capabilities, reason);
    adrc_app_bridge_request_stop(bridge);
    bridge->release_disable_pending = 1U;
}

/** @brief Revalidates same-batch RX even after a prior completion poll; abnormal terminal validity never revives. */
void adrc_app_bridge_check_capability_health(AdrcAppBridge *bridge, uint64_t observed_us)
{
    if (bridge != NULL && bridge->initialized && (bridge->capability_prepare_state == 1U ||
        bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING ||
        (bridge->capabilities.state == MOTOR_CAPABILITY_COMPLETE &&
         !bridge->capability_terminal_closed && !bridge->capabilities.completed_revoked)) &&
        !capability_health(bridge, observed_us, 1U, 0U))
    {
        motor_capability_revoke_completed(&bridge->capabilities, MOTOR_CAPABILITY_REASON_BINDING);
        fail_capability(bridge, MOTOR_CAPABILITY_REASON_BINDING);
    }
}

/** @brief Performs a finite 64ms DISABLE challenge before any parameter read budget begins. */
static uint8_t service_capability_prepare(AdrcAppBridge *bridge, uint64_t timestamp_us)
{
    const MotorJointFeedback *feedback = &bridge->runtime->bank.motors[6].feedback;
    if (bridge->capability_prepare_state != 1U) { return 0U; }
    if (bridge->send_failed)
    { fail_capability(bridge, MOTOR_CAPABILITY_REASON_TRANSPORT); return 0U; }
    if (!capability_health(bridge, timestamp_us, 1U, 0U))
    { fail_capability(bridge, MOTOR_CAPABILITY_REASON_BINDING); return 0U; }
    if (timestamp_us < bridge->capability_prepare_started_us ||
        timestamp_us - bridge->capability_prepare_started_us > 64000ULL)
    { fail_capability(bridge, MOTOR_CAPABILITY_REASON_TIMEOUT); return 0U; }
    if (bridge->capability_prepare_receipt && bridge->capability_prepare_submit_us &&
        feedback->timestamp_us > bridge->capability_prepare_submit_us &&
        capability_health(bridge, timestamp_us, 1U, 1U))
    {
        if (!motor_capability_start_reserved(&bridge->capabilities, timestamp_us))
        { fail_capability(bridge, MOTOR_CAPABILITY_REASON_BINDING); return 0U; }
        bridge->capability_prepare_state = 2U;
        return 1U;
    }
    if (!build_special(bridge, 6U, 0U))
    { fail_capability(bridge, MOTOR_CAPABILITY_REASON_TRANSPORT); return 0U; }
    return 1U;
}

/** @brief Records the immutable first actual hardware submission, independent of repeated challenges. */
void adrc_app_bridge_note_capability_disable_submit(AdrcAppBridge *bridge, uint64_t submitted_us)
{
    if (bridge != NULL && bridge->initialized) { adrc_shaft_adapter_note_submit(bridge, submitted_us); }
    if (bridge != NULL && bridge->initialized && bridge->capability_prepare_state == 1U &&
        !bridge->capability_prepare_submit_us && submitted_us >= bridge->capability_prepare_started_us &&
        bridge->awaiting_receipt && bridge->frame_kind == 2U && bridge->frame.identifier == 7U &&
        bridge->frame.length == 8U && bridge->frame.data[7] == S3519_MODE_COMMAND_DISABLE)
    { bridge->capability_prepare_submit_us = submitted_us; }
}

/** @brief Aborts diagnostics while retaining an explicit safety-disable obligation. */
static void abort_capture(AdrcAppBridge *bridge, uint8_t reason)
{
    if (bridge->capture.state != 1U) { return; }
    bridge->capture.state = 3U;
    bridge->capture.reason = reason;
    bridge->release_disable_pending = 1U;
    bridge->stop_pending = 1U;
    bridge->frame_pending = 0U;
}

/** @brief Collects bounded disabled feedback only; never sends ENABLE or modifies qualification. */
static uint8_t service_capture(AdrcAppBridge *bridge, uint64_t timestamp_us)
{
    AdrcDisabledCapture *capture = &bridge->capture;
    MotorFeedbackSnapshot snapshot;
    const MotorJointFeedback *feedback;
    uint64_t last_rx_us;
    if (capture->state != 1U) { return 0U; }
    if (bridge->send_failed) { abort_capture(bridge, 1U); return 0U; }
    if (bridge->stop_pending || bridge->bench.gateway.stop_pending || bridge->release_disable_pending)
    { abort_capture(bridge, 5U); return 0U; }
    if (capture->axis != bridge->bench.draft_config.axis_index ||
        !same_discovery(&capture->discovery, &bridge->runtime->discovery.results[capture->axis]) ||
        bridge->bench.experiment.status.state != ADRC_STATE_DISABLED)
    { abort_capture(bridge, 4U); return 0U; }
    if (timestamp_us < capture->started_us || timestamp_us < capture->last_request_us ||
        timestamp_us - capture->started_us >= 400000ULL)
    { abort_capture(bridge, 2U); return 0U; }
    if (!capture->count && timestamp_us - capture->started_us > 16000ULL)
    { abort_capture(bridge, 3U); return 0U; }
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(bridge->runtime, timestamp_us, 8000ULL, &snapshot);
    feedback = &snapshot.joints[capture->axis];
    last_rx_us = capture->count ? capture->samples[capture->count - 1U].rx_lower_us : 0ULL;
    if (feedback->timestamp_us > timestamp_us ||
        (last_rx_us && feedback->timestamp_us < last_rx_us))
    { abort_capture(bridge, 3U); return 0U; }
    if ((snapshot.valid_joint_mask & (1U << capture->axis)) != 0U)
    {
        if (feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags ||
            !finite_value(feedback->position_rad) || !finite_value(feedback->velocity_rad_s) ||
            !finite_value(feedback->torque_nm) || fabsf(feedback->velocity_rad_s) > 0.05F ||
            (capture->count && fabsf(feedback->position_rad - capture->samples[0].position_rad) > 0.005F))
        { abort_capture(bridge, 3U); return 0U; }
        if (capture->sent && feedback->timestamp_us > capture->first_request_us &&
            feedback->timestamp_us > last_rx_us)
        {
            AdrcDisabledCaptureSample *sample;
            if (last_rx_us && feedback->timestamp_us - last_rx_us > 8000ULL)
            { abort_capture(bridge, 3U); return 0U; }
            sample = &capture->samples[capture->count++];
            sample->observed_us = (capture->axis == 6U && bridge->shaft.observed_rx_us == feedback->timestamp_us) ?
                bridge->shaft.observed_us : timestamp_us;
            if (capture->axis == 6U && bridge->shaft.observed_rx_us == feedback->timestamp_us)
            { ++capture->observed_proven_count; }
            sample->rx_lower_us = feedback->timestamp_us;
            sample->position_rad = feedback->position_rad;
            sample->velocity_rad_s = feedback->velocity_rad_s;
            sample->torque_nm = feedback->torque_nm;
            last_rx_us = feedback->timestamp_us;
        }
    }
    if ((last_rx_us && timestamp_us - last_rx_us > 8000ULL) ||
        (!capture->count && timestamp_us - capture->started_us > 16000ULL))
    { abort_capture(bridge, 3U); return 0U; }
    if (capture->count == ADRC_DISABLED_CAPTURE_CAPACITY)
    { capture->state = 2U; return 1U; }
    if (capture->sent && timestamp_us - capture->last_request_us < 4000ULL) { return 1U; }
    if (capture->sent >= 68U) { abort_capture(bridge, 2U); return 0U; }
    if (!build_special(bridge, capture->axis, 0U))
    { abort_capture(bridge, 1U); return 0U; }
    if (capture->sent == 0U) { capture->first_request_us = timestamp_us; }
    capture->last_request_us = timestamp_us;
    ++capture->sent;
    return 1U;
}

/** @brief Validates a bounded decimal sample index without truncation or overflow. */
static uint8_t capture_index(const TextProtocolRequest *request, uint16_t *index)
{
    TextProtocolSpan field;
    size_t character;
    unsigned int value = 0U;
    if (request->field_count != 1U || !text_protocol_find_field(request, "index", &field) ||
        field.length == 0U || field.length > 3U) { return 0U; }
    for (character = 0U; character < field.length; ++character)
    {
        if (field.data[character] < '0' || field.data[character] > '9') { return 0U; }
        value = value * 10U + (unsigned int)(field.data[character] - '0');
    }
    if (value >= ADRC_SIMPLE_TRACE_CAPACITY) { return 0U; }
    *index = (uint16_t)value;
    return 1U;
}

/** @brief Retains the timer lock after any failed selftest and requests ordinary DISABLE too. */
static void abort_guard_test(AdrcAppBridge *bridge, uint8_t reason)
{
    if (bridge->guard_test.state == 0U || bridge->guard_test.state >= 5U) { return; }
    bridge->guard_test.state = 6U;
    bridge->guard_test.reason = reason;
    bridge->guard_test.pause_ms = 0U;
    bridge->stop_pending = 1U;
    bridge->release_disable_pending = 1U;
    bridge->frame_pending = 0U;
}

/** @brief Verifies independent stop while already disabled; only the control task is paused. */
static uint8_t service_guard_test(AdrcAppBridge *bridge, uint64_t timestamp_us)
{
    AdrcStopGuardTest *test = &bridge->guard_test;
    MotorFeedbackSnapshot snapshot;
    const MotorJointFeedback *feedback;
    uint8_t timer_state;
    if (test->state == 0U || test->state >= 5U) { return 0U; }
    if (bridge->send_failed) { abort_guard_test(bridge, 1U); return 0U; }
    if (bridge->stop_pending || bridge->release_disable_pending || bridge->bench.gateway.stop_pending)
    { abort_guard_test(bridge, 5U); return 0U; }
    if (bridge->bench.draft_config.axis_index != 6U || bridge->capture.axis != 6U ||
        !same_discovery(&bridge->capture.discovery, &bridge->runtime->discovery.results[6]) ||
        bridge->bench.experiment.status.state != ADRC_STATE_DISABLED)
    { abort_guard_test(bridge, 4U); return 0U; }
    if (timestamp_us < test->started_us || timestamp_us < test->request_us ||
        timestamp_us - test->started_us > 60000ULL)
    { abort_guard_test(bridge, 2U); return 0U; }
    if (test->state == 1U)
    {
        if (!build_special(bridge, 6U, 0U)) { abort_guard_test(bridge, 1U); return 0U; }
        test->request_us = timestamp_us; test->receipt = 0U; test->state = 2U;
        test->last_request_us = timestamp_us; test->requests = 1U;
        return 1U;
    }
    if (test->state == 3U)
    {
        timer_state = test->hooks.observe(&test->deadline_ms, &test->fired_ms);
        if (timer_state != 3U || test->fired_ms != test->deadline_ms)
        { abort_guard_test(bridge, 6U); return 0U; }
        /* RX received during the deliberate pause has an old lower bound. Obtain
         * a new causal disabled response after resuming; never relabel that RX. */
        if (!build_special(bridge, 6U, 0U)) { abort_guard_test(bridge, 1U); return 0U; }
        test->request_us = timestamp_us; test->receipt = 0U; test->state = 4U;
        test->last_request_us = timestamp_us; test->requests = 1U;
        return 1U;
    }
    if (timestamp_us - test->request_us > 16000ULL)
    { abort_guard_test(bridge, 2U); return 0U; }
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(bridge->runtime, timestamp_us, 8000ULL, &snapshot);
    feedback = &snapshot.joints[6];
    if (feedback->timestamp_us > timestamp_us || feedback->fault_flags ||
        feedback->driver_state != S3519_DRIVER_STATE_DISABLED ||
        !finite_value(feedback->position_rad) || !finite_value(feedback->velocity_rad_s) ||
        !finite_value(feedback->torque_nm) || fabsf(feedback->velocity_rad_s) > 0.05F ||
        fabsf(feedback->position_rad - bridge->capture.samples[0].position_rad) > 0.005F)
    { abort_guard_test(bridge, 3U); return 0U; }
    if (!(snapshot.valid_joint_mask & (1U << 6)) || !test->receipt ||
        feedback->timestamp_us <= test->request_us)
    {
        if (test->requests < 3U && timestamp_us - test->last_request_us >= 4000ULL)
        {
            if (!build_special(bridge, 6U, 0U)) { abort_guard_test(bridge, 1U); return 0U; }
            test->last_request_us = timestamp_us;
            test->receipt = 0U;
            ++test->requests;
        }
        return 1U;
    }
    if (test->state == 2U)
    {
        if (!build_special(bridge, 6U, 0U)) { abort_guard_test(bridge, 1U); return 0U; }
        bridge->frame_pending = 0U; /* This frame is preloaded into the independent buffer only. */
        if (!test->hooks.arm(&bridge->frame, (uint32_t)(timestamp_us / 1000ULL), 20U))
        { abort_guard_test(bridge, 6U); return 0U; }
        test->state = 3U; test->pause_ms = 24U;
    }
    else
    {
        if (!test->hooks.ack_disabled()) { abort_guard_test(bridge, 6U); return 0U; }
        test->disabled_rx_us = feedback->timestamp_us;
        test->state = 5U;
    }
    return 1U;
}

/** @brief Freezes first-rejection inputs; timestamp zero is explicitly unknown, never inferred. */
static void record_watchdog_failure(AdrcAppBridge *bridge, uint8_t reason, uint64_t timestamp_us)
{
    AdrcWatchdogProbe *probe = &bridge->watchdog;
    MotorFeedbackSnapshot snapshot;
    if (probe->reason) { return; }
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(bridge->runtime, timestamp_us, 8000ULL, &snapshot);
    probe->failure_feedback = snapshot.joints[6];
    probe->failure_observed_us = timestamp_us;
    probe->failure_phase = probe->state;
    probe->failure_receipt = probe->receipt;
    if (probe->guard_armed && bridge->guard_test.hooks.observe != NULL)
    {
        probe->failure_guard_state = bridge->guard_test.hooks.observe(
            &probe->failure_guard_deadline_ms, &probe->failure_guard_fired_ms);
    }
    probe->reason = reason;
}

/** @brief Retains the first probe failure and enters an independent, bounded stop path. */
static void abort_watchdog(AdrcAppBridge *bridge, uint8_t reason, uint64_t timestamp_us)
{
    AdrcWatchdogProbe *probe = &bridge->watchdog;
    if (!probe->state || probe->state >= 15U) { return; }
    record_watchdog_failure(bridge, reason, timestamp_us);
    if (probe->state < 10U)
    { probe->state = 10U; probe->phase_started_us = timestamp_us; probe->requests = 0U; }
    else { probe->state = 16U; }
    bridge->frame_pending = 0U;
    if (probe->state == 16U)
    { bridge->release_disable_pending = 1U; (void)build_special(bridge, 6U, 0U); }
}

/** @brief Sends only the fixed volatile TIMEOUT candidate or its verified original zero value. */
static void watchdog_parameter_frame(AdrcAppBridge *bridge, uint8_t write, uint8_t value)
{
    memset(&bridge->frame, 0, sizeof(bridge->frame));
    bridge->frame.identifier = 0x7FFU;
    bridge->frame.length = write ? 8U : 4U;
    bridge->frame.data[0] = 7U;
    bridge->frame.data[2] = write ? 0x55U : 0x33U;
    bridge->frame.data[3] = 9U;
    bridge->frame.data[4] = write ? value : 0U;
    bridge->frame_kind = 4U; bridge->decoded_torque_nm = 0.0F;
    bridge->frame_pending = 1U; bridge->watchdog.receipt = 0U;
}

/** @brief Quarantines this transaction's parameter replies instead of decoding them as motor feedback. */
uint8_t adrc_app_bridge_accept_watchdog_parameter(AdrcAppBridge *bridge,
    const CanFrame *frame, uint64_t rx_lower_us)
{
    AdrcWatchdogProbe *probe;
    if (bridge == NULL || frame == NULL || !bridge->initialized ||
        (!bridge->watchdog.state && (!bridge->watchdog_quarantine_until_us ||
            rx_lower_us > bridge->watchdog_quarantine_until_us)) ||
        frame->identifier != 23U || frame->length != 8U || frame->data[0] != 7U ||
        frame->data[1] != 0U || frame->data[3] != 9U ||
        (frame->data[2] != 0x33U && frame->data[2] != 0x55U)) { return 0U; }
    probe = &bridge->watchdog;
    if ((probe->state == 3U || probe->state == 14U) && frame->data[2] == 0x33U &&
        rx_lower_us > probe->first_request_us)
    {
        probe->parameter_value = (uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8U) |
            ((uint32_t)frame->data[6] << 16U) | ((uint32_t)frame->data[7] << 24U);
        probe->parameter_rx_us = rx_lower_us;
    }
    return 1U;
}

/** @brief Preserves the original causal anchor and fixed timeout across at most three requests. */
static void watchdog_start_disable(AdrcAppBridge *bridge, uint64_t timestamp_us, uint8_t next_state)
{
    AdrcWatchdogProbe *probe = &bridge->watchdog;
    if (!build_special(bridge, 6U, 0U))
    { record_watchdog_failure(bridge, 1U, timestamp_us); probe->state = 16U; return; }
    probe->first_request_us = timestamp_us; probe->last_request_us = timestamp_us;
    probe->requests = 1U; probe->receipt = 0U; probe->state = next_state;
}

/** @brief Bounds disabled and enabled diagnostics without relaxing the normal supervisor. */
static uint8_t watchdog_feedback_values_safe(const AdrcAppBridge *bridge, const MotorJointFeedback *feedback)
{
    return (uint8_t)(finite_value(feedback->position_rad) && finite_value(feedback->velocity_rad_s) &&
        finite_value(feedback->torque_nm) && finite_value(feedback->mos_temperature_c) &&
        finite_value(feedback->rotor_temperature_c) && feedback->mos_temperature_c < 50.0F &&
        feedback->rotor_temperature_c < 50.0F && fabsf(feedback->velocity_rad_s) <= 0.05F &&
        fabsf(feedback->torque_nm) <= 0.05F &&
        fabsf(feedback->position_rad - bridge->capture.samples[0].position_rad) <= 0.005F);
}

/** @brief Keeps the established 8 ms maximum age separate from physical-value rejection. */
static uint8_t watchdog_feedback_safe(const AdrcAppBridge *bridge, const MotorJointFeedback *feedback,
    uint64_t timestamp_us)
{
    return (uint8_t)(feedback->timestamp_us != 0ULL && feedback->timestamp_us <= timestamp_us &&
        timestamp_us - feedback->timestamp_us <= 8000ULL && watchdog_feedback_values_safe(bridge, feedback));
}

/** @brief Makes a zero-gain nominal request, retaining its actual nonzero quantized torque. */
static uint8_t watchdog_neutral(AdrcAppBridge *bridge)
{
    float torque_max = bridge->watchdog.discovery.ranges.torque_max_nm;
    float decoded = 2047.0F * (2.0F * torque_max / 4095.0F) - torque_max;
    if (!finite_value(decoded) || fabsf(decoded) > 0.005F ||
        motor_adrc_build_neutral_probe(bridge->runtime, 6U, &bridge->frame) != MOTOR_RUNTIME_STATUS_OK)
    { return 0U; }
    bridge->frame_kind = 1U; bridge->decoded_torque_nm = decoded;
    bridge->frame_pending = 1U; bridge->watchdog.receipt = 0U;
    return 1U;
}

/** @brief Runs only one finite commissioning trial; all outcomes leave qualification untouched. */
static uint8_t service_watchdog(AdrcAppBridge *bridge, uint64_t timestamp_us)
{
    AdrcWatchdogProbe *probe = &bridge->watchdog;
    MotorFeedbackSnapshot snapshot;
    MotorDiscoveryResult current;
    const MotorJointFeedback *feedback;
    uint32_t deadline_ms = 0U, fired_ms = 0U;
    uint8_t timer_state = 0U;
    uint8_t safe;
    if (!probe->state || probe->state >= 15U) { return 0U; }
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(bridge->runtime, timestamp_us, 8000ULL, &snapshot);
    feedback = &snapshot.joints[6];
    safe = watchdog_feedback_safe(bridge, feedback, timestamp_us);
    current = bridge->runtime->discovery.results[6];
    if (probe->state >= 5U && probe->state <= 9U && current.communication_timeout_raw != 120U)
    { abort_watchdog(bridge, 4U, timestamp_us); }
    current.communication_timeout_raw = probe->discovery.communication_timeout_raw;
    if (bridge->bench.draft_config.axis_index != 6U || !same_discovery(&current, &probe->discovery))
    { abort_watchdog(bridge, 4U, timestamp_us); }
    if (bridge->send_failed) { abort_watchdog(bridge, 1U, timestamp_us); }
    if (bridge->stop_pending || bridge->bench.gateway.stop_pending || bridge->release_disable_pending)
    { abort_watchdog(bridge, 5U, timestamp_us); }
    bridge->send_failed = 0U;
    bridge->stop_pending = 0U;
    if (probe->state == 16U) { bridge->release_disable_pending = 1U; return 1U; }
    if (timestamp_us < probe->started_us || timestamp_us - probe->started_us > 180000ULL)
    {
        record_watchdog_failure(bridge, 2U, timestamp_us);
        probe->state = 16U; bridge->release_disable_pending = 1U;
        (void)build_special(bridge, 6U, 0U); return 1U;
    }
    if (probe->guard_armed)
    { timer_state = bridge->guard_test.hooks.observe(&deadline_ms, &fired_ms); }
    if (probe->state >= 7U && probe->state <= 9U && timer_state != 1U)
    { abort_watchdog(bridge, 6U, timestamp_us); }
    switch (probe->state)
    {
        case 1U:
            watchdog_parameter_frame(bridge, 1U, 120U); probe->state = 2U;
            break;
        case 2U:
        case 13U:
            if (!probe->receipt) { abort_watchdog(bridge, 1U, timestamp_us); break; }
            watchdog_parameter_frame(bridge, 0U, 0U);
            probe->first_request_us = timestamp_us; probe->last_request_us = timestamp_us;
            probe->parameter_rx_us = 0ULL; probe->requests = 1U;
            probe->state = probe->state == 2U ? 3U : 14U;
            break;
        case 3U:
        case 14U:
            if (timestamp_us < probe->first_request_us || timestamp_us - probe->first_request_us > 16000ULL ||
                probe->parameter_rx_us > timestamp_us)
            { abort_watchdog(bridge, 7U, timestamp_us); break; }
            if (probe->parameter_rx_us && probe->receipt && probe->parameter_rx_us <= timestamp_us)
            {
                if (probe->parameter_value != (probe->state == 3U ? 120U : 0U))
                {
                    abort_watchdog(bridge, 7U, timestamp_us);
                    break;
                }
                bridge->runtime->discovery.results[6].communication_timeout_raw = probe->parameter_value;
                if (probe->state == 3U) { watchdog_start_disable(bridge, timestamp_us, 5U); }
                else
                {
                    probe->timeout_restored = 1U; probe->finished_us = timestamp_us;
                    probe->state = (!probe->reason && probe->enabled_seen && probe->watchdog_seen) ? 15U : 17U;
                }
            }
            else if (probe->requests < 3U && timestamp_us - probe->last_request_us >= 4000ULL)
            {
                watchdog_parameter_frame(bridge, 0U, 0U);
                probe->last_request_us = timestamp_us; ++probe->requests;
            }
            break;
        case 5U:
        case 11U:
            if (timestamp_us < probe->first_request_us || timestamp_us - probe->first_request_us > 16000ULL)
            { abort_watchdog(bridge, 3U, timestamp_us); break; }
            if (!watchdog_feedback_values_safe(bridge, feedback) || feedback->fault_flags ||
                feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->timestamp_us > timestamp_us)
            { abort_watchdog(bridge, 3U, timestamp_us); break; }
            if (safe && feedback->driver_state == S3519_DRIVER_STATE_DISABLED && !feedback->fault_flags &&
                probe->receipt && feedback->timestamp_us > probe->first_request_us)
            {
                if (probe->state == 5U)
                {
                    if (!watchdog_neutral(bridge)) { abort_watchdog(bridge, 3U, timestamp_us); break; }
                    probe->state = 6U;
                }
                else
                {
                    if (probe->guard_armed && !bridge->guard_test.hooks.ack_disabled())
                    { record_watchdog_failure(bridge, 6U, timestamp_us); probe->state = 16U; break; }
                    probe->guard_armed = 0U; probe->disabled_rx_us = feedback->timestamp_us;
                    watchdog_parameter_frame(bridge, 1U, 0U); probe->state = 13U;
                }
            }
            else if (probe->requests < 3U && timestamp_us - probe->last_request_us >= 4000ULL)
            {
                if (!build_special(bridge, 6U, 0U))
                { record_watchdog_failure(bridge, 1U, timestamp_us); probe->state = 16U; break; }
                probe->last_request_us = timestamp_us; ++probe->requests; probe->receipt = 0U;
            }
            break;
        case 6U:
            if (!probe->receipt || !safe || feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags)
            { abort_watchdog(bridge, 3U, timestamp_us); break; }
            if (!build_special(bridge, 6U, 0U) ||
                !bridge->guard_test.hooks.arm(&bridge->frame, (uint32_t)(timestamp_us / 1000ULL), 20U))
            { abort_watchdog(bridge, 6U, timestamp_us); break; }
            probe->guard_armed = 1U;
            if (!build_special(bridge, 6U, 1U)) { abort_watchdog(bridge, 1U, timestamp_us); break; }
            probe->enable_request_us = timestamp_us; probe->receipt = 0U; probe->state = 7U;
            break;
        case 7U:
            if (!probe->receipt || !safe || feedback->fault_flags || !watchdog_neutral(bridge))
            { abort_watchdog(bridge, 3U, timestamp_us); break; }
            probe->last_command_us = timestamp_us; probe->state = 8U;
            break;
        case 8U:
            if (!probe->receipt || !safe || feedback->driver_state != S3519_DRIVER_STATE_ENABLED ||
                feedback->fault_flags || feedback->timestamp_us <= probe->enable_request_us)
            { abort_watchdog(bridge, 3U, timestamp_us); break; }
            probe->enabled_seen = 1U; probe->enabled_rx_us = feedback->timestamp_us;
            probe->last_command_observed_us = timestamp_us; probe->state = 9U;
            break;
        case 9U:
            if (safe && feedback->driver_state == 13U && feedback->timestamp_us > probe->last_command_us &&
                timestamp_us - probe->last_command_us >= 6000ULL &&
                feedback->timestamp_us <= probe->last_command_observed_us + 6000ULL)
            {
                probe->watchdog_seen = 1U; probe->watchdog_rx_us = feedback->timestamp_us;
                probe->watchdog_observed_us = timestamp_us;
                probe->state = 10U; probe->phase_started_us = timestamp_us; probe->requests = 0U;
            }
            else if (!safe || feedback->fault_flags || feedback->driver_state != S3519_DRIVER_STATE_ENABLED ||
                timestamp_us - probe->last_command_us > 16000ULL)
            { abort_watchdog(bridge, 3U, timestamp_us); }
            break;
        default: break;
    }
    if (probe->state == 10U)
    {
        if (!probe->phase_started_us) { probe->phase_started_us = timestamp_us; }
        if (!probe->guard_armed || (timer_state == 3U && fired_ms == deadline_ms))
        { watchdog_start_disable(bridge, timestamp_us, 11U); }
        else if (timestamp_us - probe->phase_started_us > 32000ULL)
        { record_watchdog_failure(bridge, 6U, timestamp_us); probe->state = 16U; }
        else if (!probe->requests ||
            (probe->requests < 8U && timestamp_us - probe->last_request_us >= 4000ULL))
        {
            (void)build_special(bridge, 6U, 0U);
            probe->last_request_us = timestamp_us; ++probe->requests; probe->receipt = 0U;
        }
    }
    if (probe->state == 16U)
    { bridge->release_disable_pending = 1U; (void)build_special(bridge, 6U, 0U); }
    if (bridge->frame_pending && bridge->frame_kind == 4U)
    { bridge->watchdog_quarantine_until_us = timestamp_us + 100000ULL; }
    return 1U;
}

/** @brief Requests one bounded Motor-layer torque frame and retains its decoded nominal value. */
static uint8_t build_torque(AdrcAppBridge *bridge, uint8_t axis, float torque_nm)
{
    MotorAdrcTorqueRequest request;
    request.axis_index = axis;
    request.torque_nm = torque_nm;
    request.previous_sent_nm = bridge->receipt_torque_nm;
    request.torque_scale = bridge->bench.mapping[2];
    request.torque_limit_nm = bridge->bench.experiment.config.torque_limit_nm;
    request.torque_step_nm = bridge->bench.experiment.config.torque_slew_nm_s * 0.004F;
    if (motor_adrc_build_torque_frame(bridge->runtime, &request, &bridge->frame,
        &bridge->decoded_torque_nm) != MOTOR_RUNTIME_STATUS_OK) { return 0U; }
    bridge->frame_kind = 1U;
    bridge->frame_pending = 1U;
    return 1U;
}

/** @brief Initializes shared bridge storage while optionally preserving LCD-owned discovery. */
static AdrcExperimentResult init_bridge(AdrcAppBridge *bridge, MotorRuntime *runtime,
    ProtocolEngine *engine, AdrcControllerCallback controller, void *controller_context,
    uint8_t start_discovery)
{
    AdrcExperimentResult result;
    if (bridge == NULL || runtime == NULL || engine == NULL || !runtime->initialized)
    { return ADRC_RESULT_INVALID_ARGUMENT; }
    memset(bridge, 0, sizeof(*bridge));
    bridge->runtime = runtime;
    bridge->engine = engine;
    result = adrc_bench_init(&bridge->bench, ADRC_ENV_HARDWARE, controller, controller_context);
    if (result != ADRC_RESULT_OK) { return result; }
    protocol_engine_set_adrc_handler(engine, adrc_protocol_handle_request, adrc_bench_gateway(&bridge->bench));
    bridge->discovery_axis = bridge->bench.draft_config.axis_index;
    reset_feedback_statistics(bridge);
    if (start_discovery != 0U)
    { (void)motor_runtime_begin_discovery(runtime, (uint8_t)(1U << bridge->discovery_axis)); }
    bridge->initialized = 1U;
    adrc_shaft_adapter_init(bridge);
    bridge->capability_epoch = 1U;
    bridge->capability_standalone_owner = start_discovery;
    return ADRC_RESULT_OK;
}

/** @brief Initializes a fail-closed isolated ADRC bridge with selected-axis discovery. */
AdrcExperimentResult adrc_app_bridge_init(AdrcAppBridge *bridge, MotorRuntime *runtime,
    ProtocolEngine *engine, AdrcControllerCallback controller, void *controller_context)
{
    return init_bridge(bridge, runtime, engine, controller, controller_context, 1U);
}

/** @brief Initializes ADRC without replacing the LCD-owned motor discovery sequence. */
AdrcExperimentResult adrc_app_bridge_init_deferred(AdrcAppBridge *bridge, MotorRuntime *runtime,
    ProtocolEngine *engine, AdrcControllerCallback controller, void *controller_context)
{
    return init_bridge(bridge, runtime, engine, controller, controller_context, 0U);
}

/** @brief Starts selected-axis read-only discovery after exclusive ownership is granted. */
MotorRuntimeStatus adrc_app_bridge_start_selected_discovery(AdrcAppBridge *bridge, uint8_t axis_index)
{
    MotorRuntimeStatus status;
    if (bridge == NULL || bridge->initialized == 0U || axis_index >= ARM_JOINT_COUNT)
    { return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT; }
    if (bridge->capability_prepare_state == 1U || bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING || (bridge->shaft.used && !bridge->shaft.released) || bridge->simple_request_id ||
        bridge->frame_pending || bridge->awaiting_receipt || bridge->stop_pending ||
        bridge->bench.experiment.status.state == ADRC_STATE_RUNNING ||
        bridge->bench.experiment.status.state == ADRC_STATE_STOPPING)
    { return MOTOR_RUNTIME_STATUS_WAITING; }
    status = motor_runtime_begin_discovery(bridge->runtime, (uint8_t)(1U << axis_index));
    if (status == MOTOR_RUNTIME_STATUS_OK)
    {
        bridge->discovery_axis = axis_index;
        bridge->bench.draft_config.axis_index = axis_index;
        reset_feedback_statistics(bridge);
    }
    return status;
}

/** @brief Binds the finite trial timer once, before any request can own it. */
void adrc_app_bridge_set_simple_guard_hooks(AdrcAppBridge *bridge, const AdrcSimpleGuardHooks *hooks)
{
    if (bridge && bridge->initialized && hooks && !bridge->simple_request_id && !bridge->simple_hooks.arm_trial)
    { bridge->simple_hooks = *hooks; }
}

/** @brief Copies unscaled protocol feedback; this trial does not claim joint-coordinate calibration. */
static void simple_input(AdrcAppBridge *bridge, uint64_t timestamp_us, AdrcSimpleInput *input)
{
    MotorFeedbackSnapshot snapshot;
    const MotorJointFeedback *feedback;
    memset(input, 0, sizeof(*input)); memset(&snapshot, 0, sizeof(snapshot));
    input->feedback.now_us = timestamp_us; input->feedback.axis_index = 6U;
    input->feedback.prev_sent_nm = bridge->receipt_torque_nm;
    input->feedback.prev_sent_valid = bridge->receipt_valid;
    input->feedback.prev_send_failed = bridge->send_failed;
    input->tx_receipt_valid = bridge->simple_receipt;
    input->tx_kind = bridge->simple_receipt_kind;
    if (motor_runtime_get_snapshot(bridge->runtime, timestamp_us, 8000ULL, &snapshot) != MOTOR_RUNTIME_STATUS_OK)
    { return; }
    feedback = &snapshot.joints[6];
    input->feedback.feedback_valid = (uint8_t)((snapshot.valid_joint_mask & 64U) != 0U);
    input->feedback.feedback_us = feedback->timestamp_us;
    input->feedback.position_rad = feedback->position_rad;
    input->feedback.velocity_rad_s = feedback->velocity_rad_s;
    input->feedback.raw_velocity_rad_s = feedback->velocity_rad_s;
    input->feedback.enabled = (uint8_t)(feedback->driver_state != S3519_DRIVER_STATE_DISABLED);
    input->feedback.driver_fault = (uint8_t)(feedback->fault_flags ||
        (feedback->driver_state != S3519_DRIVER_STATE_DISABLED && feedback->driver_state != S3519_DRIVER_STATE_ENABLED));
    input->feedback.mos_temperature_c = feedback->mos_temperature_c;
    input->feedback.rotor_temperature_c = feedback->rotor_temperature_c;
    input->feedback_torque_nm = feedback->torque_nm;
}

/** @brief Uses the actual integer grid and receipt for every zero-gain trial command. */
static uint8_t simple_output(AdrcAppBridge *bridge, const AdrcExperimentOutput *output)
{
    MotorAdrcTorqueRequest request;
    if (output->disable_requested) { return build_special(bridge, 6U, 0U); }
    if (output->enable_requested) { return build_special(bridge, 6U, 1U); }
    if (!output->send_torque) { return 1U; }
    memset(&request, 0, sizeof(request));
    request.axis_index = 6U; request.torque_nm = output->torque_nm;
    request.previous_sent_nm = bridge->receipt_torque_nm; request.torque_scale = 1.0F;
    request.torque_limit_nm = bridge->simple.torque_limit_nm; request.torque_step_nm = bridge->simple.torque_step_nm;
    if (motor_adrc_build_torque_frame(bridge->runtime, &request, &bridge->frame,
            &bridge->decoded_torque_nm) != MOTOR_RUNTIME_STATUS_OK) { return 0U; }
    bridge->frame_kind = 1U; bridge->frame_pending = 1U;
    return 1U;
}

/** @brief Executes one bounded trial without granting legacy RUN qualification or clearing faults. */
static uint8_t service_simple(AdrcAppBridge *bridge, uint64_t timestamp_us)
{
    AdrcSimpleInput input;
    AdrcExperimentOutput output;
    uint32_t deadline_ms = 0U, fired_ms = 0U;
    uint8_t guard_state = 0U;
    uint8_t starting;
    uint8_t stop_requested;
    if (!bridge->simple_request_id) { return 0U; }
    simple_input(bridge, timestamp_us, &input);
    memset(&output, 0, sizeof(output));
    starting = bridge->simple_pending;
    stop_requested = (uint8_t)(bridge->stop_pending || bridge->bench.gateway.stop_pending || bridge->release_disable_pending);
    if (!same_discovery(&bridge->simple_discovery, &bridge->runtime->discovery.results[6]) ||
        bridge->bench.draft_config.axis_index != 6U)
    { bridge->simple_bridge_fault = 1U; adrc_simple_abort(&bridge->simple); }
    if (bridge->stop_pending || bridge->bench.gateway.stop_pending || bridge->release_disable_pending)
    { adrc_simple_abort(&bridge->simple); }
    if (starting)
    {
        /* Selftest completion may precede the host request by more than 8 ms.
         * Refresh through DISABLE only; never reinterpret stale feedback as fresh. */
        if (starting == 1U) { bridge->simple_prepare_us = timestamp_us; }
        if (!bridge->simple_bridge_fault && !stop_requested && !bridge->send_failed &&
            timestamp_us - bridge->simple_prepare_us < 64000ULL &&
            (!input.feedback.feedback_valid || timestamp_us - input.feedback.feedback_us > 8000ULL ||
             (starting == 2U && (!input.tx_receipt_valid || input.tx_kind != 2U ||
              input.feedback.feedback_us <= bridge->simple_prepare_us))))
        {
            bridge->simple_pending = 2U;
            bridge->simple_receipt = 0U; bridge->receipt_valid = 0U;
            if (!build_special(bridge, 6U, 0U)) { bridge->simple_bridge_fault = 3U; }
            return 1U;
        }
        bridge->simple_pending = 0U;
        if (!bridge->simple_bridge_fault && !bridge->stop_pending && !bridge->bench.gateway.stop_pending &&
            !bridge->release_disable_pending && !bridge->send_failed &&
            (starting != 2U || (timestamp_us - bridge->simple_prepare_us < 64000ULL &&
             input.tx_receipt_valid && input.tx_kind == 2U && input.feedback.feedback_us > bridge->simple_prepare_us)) &&
            adrc_simple_start(&bridge->simple, &input, &output) &&
            build_special(bridge, 6U, 0U) && bridge->simple_hooks.arm_trial(&bridge->frame, (uint32_t)(timestamp_us / 1000ULL)))
        {
            bridge->simple_guard_armed = 1U;
            bridge->simple_lease_feedback_us = input.feedback.feedback_us;
        }
        else
        {
            bridge->simple_bridge_fault = 2U;
            adrc_simple_abort(&bridge->simple);
            if (bridge->simple.status.phase == ADRC_SIMPLE_IDLE)
            {
                /* Rejected admission still needs a real stop clock and fresh dwell.
                 * Arm a stop-only lease; no ENABLE was published by this branch. */
                bridge->simple.started_us = timestamp_us;
                bridge->simple.last_tick_us = timestamp_us;
                bridge->simple.stop_us = timestamp_us;
                bridge->simple.last_feedback_us = input.feedback.feedback_us;
                bridge->simple.initial_position_rad = input.feedback.position_rad;
                bridge->simple.status.phase = ADRC_SIMPLE_STOPPING;
                bridge->simple.status.result = stop_requested ? ADRC_SIMPLE_ABORTED : ADRC_SIMPLE_FAILED;
                bridge->simple.status.fault = stop_requested ? ADRC_FAULT_NONE : ADRC_FAULT_FEEDBACK;
                if (build_special(bridge, 6U, 0U) &&
                    bridge->simple_hooks.arm_trial(&bridge->frame, (uint32_t)(timestamp_us / 1000ULL)))
                { bridge->simple_guard_armed = 1U; }
            }
            memset(&output, 0, sizeof(output)); output.disable_requested = 1U;
        }
        bridge->frame_pending = 0U; /* Preloaded stop is not a queued ordinary frame. */
    }
    else
    {
        if (bridge->simple_guard_armed && !bridge->simple_guard_ack)
        { guard_state = bridge->guard_test.hooks.observe(&deadline_ms, &fired_ms); }
        if (bridge->simple.status.phase < ADRC_SIMPLE_STOPPING && guard_state != 1U)
        { bridge->simple_bridge_fault = 2U; adrc_simple_abort(&bridge->simple); }
        adrc_simple_tick(&bridge->simple, &input, &output);
        if ((output.enable_requested || output.send_torque) &&
            (!input.tx_receipt_valid || !input.feedback.feedback_valid || input.feedback.driver_fault ||
             (input.feedback.feedback_us > bridge->simple_lease_feedback_us &&
              !bridge->simple_hooks.renew_trial((uint32_t)(timestamp_us / 1000ULL)))))
        {
            bridge->simple_bridge_fault = 2U; adrc_simple_abort(&bridge->simple);
            memset(&output, 0, sizeof(output)); output.disable_requested = 1U;
        }
        else if ((output.send_torque || output.enable_requested) &&
            input.feedback.feedback_us > bridge->simple_lease_feedback_us)
        { bridge->simple_lease_feedback_us = input.feedback.feedback_us; }
        if (bridge->simple.status.safe_release && !bridge->simple_guard_ack)
        {
            if (guard_state == 3U && fired_ms == deadline_ms &&
                input.feedback.feedback_valid && !input.feedback.enabled && !input.feedback.driver_fault &&
                (uint32_t)((uint32_t)(input.feedback.feedback_us / 1000ULL) - fired_ms) != 0U &&
                (uint32_t)((uint32_t)(input.feedback.feedback_us / 1000ULL) - fired_ms) < 0x80000000UL &&
                input.tx_receipt_valid && input.tx_kind == 2U && bridge->guard_test.hooks.ack_disabled())
            { bridge->simple_guard_ack = 1U; }
            else { output.disable_requested = 1U; }
        }
    }
    bridge->simple_receipt = 0U; bridge->receipt_valid = 0U;
    bridge->send_failed = 0U; bridge->stop_pending = 0U;
    if (stop_requested) { memset(&output, 0, sizeof(output)); output.disable_requested = 1U; }
    if (!simple_output(bridge, &output))
    { bridge->simple_bridge_fault = 3U; adrc_simple_abort(&bridge->simple); (void)build_special(bridge, 6U, 0U); }
    if (output.disable_requested) { bridge->release_disable_pending = 1U; }
    return 1U;
}

/** @brief Expires the old slot before owner execution and selects disable over enable over torque. */
uint8_t adrc_app_bridge_service(AdrcAppBridge *bridge, uint64_t timestamp_us)
{
    AdrcExperimentInput input;
    AdrcExperimentOutput output;
    uint8_t force_disable;
    uint8_t disable_axis;
    uint8_t ready = 0U;
    if (bridge == NULL || !bridge->initialized) { return 0U; }
    adrc_app_bridge_check_capability_health(bridge, timestamp_us);
    if (bridge->frame_pending || bridge->awaiting_receipt) { bridge->send_failed = 1U; }
    bridge->frame_pending = 0U;
    bridge->awaiting_receipt = 0U;
    if (adrc_shaft_adapter_service(bridge, timestamp_us)) { return 1U; }
    if (service_capability_prepare(bridge, timestamp_us)) { return 1U; }
    if (service_simple(bridge, timestamp_us)) { return 1U; }
    if (service_watchdog(bridge, timestamp_us)) { return 1U; }
    if (service_guard_test(bridge, timestamp_us)) { return 1U; }
    if (service_capture(bridge, timestamp_us)) { return 1U; }
    if (bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING) { return 1U; }
    force_disable = (uint8_t)(bridge->release_disable_pending || ((bridge->stop_pending ||
        bridge->bench.gateway.stop_pending) &&
        bridge->bench.experiment.status.disabled_confirmed == 0U &&
        bridge->bench.experiment.status.state != ADRC_STATE_RUNNING &&
        bridge->bench.experiment.status.state != ADRC_STATE_STOPPING));
    if (bridge->stop_pending || bridge->send_failed)
    {
        (void)adrc_experiment_stop(&bridge->bench.experiment, timestamp_us);
    }
    bridge->stop_pending = 0U;
    make_input(bridge, timestamp_us, bridge->bench.draft_config.interface_profile, &input);
    observe_feedback(bridge, &input);
    bridge->receipt_valid = 0U;
    bridge->send_failed = 0U;
    (void)adrc_bench_service(&bridge->bench, &input, &output);
    /* A changed draft must never redirect an active run's emergency disable to another motor. */
    disable_axis = (bridge->bench.experiment.status.state == ADRC_STATE_RUNNING ||
        bridge->bench.experiment.status.state == ADRC_STATE_STOPPING ||
        bridge->bench.experiment.status.state == ADRC_STATE_FAULT) ?
        bridge->bench.experiment.config.axis_index : input.axis_index;
    if (force_disable || output.disable_requested) { ready = build_special(bridge, disable_axis, 0U); }
    else if (output.enable_requested) { ready = build_special(bridge, input.axis_index, 1U); }
    else if (output.send_torque) { ready = build_torque(bridge, input.axis_index, output.torque_nm); }
    if ((force_disable || output.disable_requested || output.enable_requested || output.send_torque) && !ready)
    {
        bridge->send_failed = 1U;
        ready = build_special(bridge, disable_axis, 0U);
    }
    return (uint8_t)(ready || bridge->bench.gateway.channel_state[0] == 3U ||
        bridge->bench.gateway.channel_state[1] == 3U || bridge->bench.gateway.channel_state[2] == 3U);
}

/** @brief Admits only ADRC and existing read-only namespaces in the opt-in firmware. */
ProtocolEngineStatus adrc_app_bridge_process_line(AdrcAppBridge *bridge, const char *line,
    size_t length, uint64_t timestamp_us, ProtocolOutputBatch *output)
{
    TextProtocolRequest request;
    TextProtocolStatus parsed;
    uint8_t allowed;
    int written;
    if (bridge == NULL || !bridge->initialized || output == NULL || line == NULL)
    { return PROTOCOL_ENGINE_STATUS_INVALID_ARGUMENT; }
    parsed = text_protocol_parse_request(line, length, &request);
    if (parsed != TEXT_PROTOCOL_STATUS_OK)
    { return protocol_engine_process_text_line(bridge->engine, line, length, timestamp_us, output); }
    adrc_app_bridge_check_capability_health(bridge, timestamp_us);
    { ProtocolEngineStatus shaft_status;
      if (adrc_shaft_adapter_process(bridge, &request, timestamp_us, output, &shaft_status)) { return shaft_status; } }
    if (text_protocol_request_path_equals(&request, "adrc", "capabilities_status"))
    {
        const MotorCapabilityCollector *collector = &bridge->capabilities;
        if (request.positional_count || request.field_count)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc capabilities_status code=bad_argument", (unsigned long)request.request_id); }
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc capabilities_status id=%lu epoch=%lu current=%u motor=7 state=%u prepare=%u reason=%u count=%u rejected=%lu qualified=0 model_known=0 actual_send_us=0",
            (unsigned long)request.request_id, (unsigned long)collector->request_id,
            (unsigned long)collector->epoch, (unsigned int)(collector->used &&
            capability_health(bridge, timestamp_us, 1U, 0U)),
            (unsigned int)(bridge->capability_prepare_state == 1U ? MOTOR_CAPABILITY_COLLECTING : collector->state),
            (unsigned int)bridge->capability_prepare_state, (unsigned int)collector->reason, (unsigned int)collector->count,
            (unsigned long)collector->rejected_count);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "capabilities_get"))
    {
        uint16_t index;
        const MotorCapabilityItem *item;
        static const char *names[6] = {"KT", "OT", "OC", "Damp", "Inertia", "Gr"};
        if (request.positional_count || !capture_index(&request, &index) || index >= MOTOR_CAPABILITY_COUNT)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc capabilities_get code=bad_argument", (unsigned long)request.request_id); }
        item = &bridge->capabilities.items[index];
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc capabilities_get id=%lu epoch=%lu current=%u index=%u rid=%u name=%s valid=%u raw_bits=%lu value=%.9g positive=%u request_created_us=%llu queue_submitted_us=%llu pre_submit_boundary_us=%llu send_completed_observed_us=%llu rx_lower_us=%llu received_observed_us=%llu actual_send_us=0 model_known=0 qualified=0",
            (unsigned long)request.request_id, (unsigned long)bridge->capabilities.request_id,
            (unsigned long)bridge->capabilities.epoch, (unsigned int)(bridge->capabilities.used &&
            capability_health(bridge, timestamp_us, 1U, 0U)),
            (unsigned int)index, (unsigned int)motor_capability_rid((uint8_t)index), names[index],
            (unsigned int)item->valid, (unsigned long)item->raw_bits, (double)item->value,
            (unsigned int)item->positive, (unsigned long long)item->request_created_us,
            (unsigned long long)item->queue_submitted_us, (unsigned long long)item->pre_submit_boundary_us,
            (unsigned long long)item->send_completed_observed_us, (unsigned long long)item->rx_lower_us,
            (unsigned long long)item->received_observed_us);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "capabilities"))
    {
        if (!request.has_request_id || !request.request_id || request.positional_count || request.field_count ||
            request.request_id <= bridge->bench.gateway.highest_admitted_request_id ||
            bridge->bench.draft_config.axis_index != 6U || !mit_discovered(bridge, 6U) ||
            bridge->runtime->discovery.results[6].observed_esc_id != 7U ||
            bridge->runtime->discovery.results[6].observed_master_id != 23U ||
            bridge->runtime->discovery_active || bridge->runtime->parameter_expectations.count ||
            bridge->runtime->position_read.pending ||
            (bridge->runtime->mode_switch_state != MOTOR_MODE_SWITCH_IDLE &&
             bridge->runtime->mode_switch_state != MOTOR_MODE_SWITCH_COMPLETE) ||
            bridge->simple_request_id || bridge->watchdog.state || bridge->capture.state == 1U ||
            (bridge->guard_test.state && bridge->guard_test.state != 5U) ||
            bridge->frame_pending || bridge->awaiting_receipt || bridge->stop_pending ||
            bridge->release_disable_pending || bridge->send_failed || bridge->bench.gateway.stop_pending ||
            bridge->bench.gateway.active_run_request_id ||
            bridge->bench.gateway.command_pending || bridge->bench.gateway.heartbeat_pending ||
            bridge->bench.trusted_qualification.verified_flags ||
            bridge->bench.experiment.status.state != ADRC_STATE_DISABLED ||
            !capability_health(bridge, timestamp_us, 0U, 0U) ||
            !motor_capability_reserve(&bridge->capabilities, request.request_id, bridge->capability_epoch))
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc capabilities code=not_ready", (unsigned long)request.request_id); }
        bridge->capability_discovery = bridge->runtime->discovery.results[6];
        bridge->capability_prepare_state = 1U;
        bridge->capability_prepare_started_us = timestamp_us;
        bridge->bench.gateway.highest_admitted_request_id = request.request_id;
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc capabilities id=%lu state=1 count=6 qualified=0",
            (unsigned long)request.request_id, (unsigned long)request.request_id);
    }
    if (bridge->capability_prepare_state == 1U || bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING)
    {
        if (text_protocol_request_path_equals(&request, "adrc", "stop"))
        {
            if (!request.has_request_id || !request.request_id || request.positional_count || request.field_count)
            { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                "error %lu adrc stop code=bad_argument", (unsigned long)request.request_id); }
            adrc_app_bridge_request_stop(bridge);
            return format_query(output, PROTOCOL_ENGINE_STATUS_OK, "ok %lu adrc stop=latched",
                (unsigned long)request.request_id);
        }
        if (!text_protocol_request_path_equals(&request, "adrc", "status") &&
            !text_protocol_request_path_equals(&request, "adrc", "feedback") &&
            !text_protocol_request_path_equals(&request, "adrc", "evidence"))
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc code=capabilities_active", (unsigned long)request.request_id); }
    }
    if (bridge->simple_request_id && text_protocol_request_path_equals(&request, "adrc", "stop"))
    {
        if (!request.has_request_id || !request.request_id || request.positional_count || request.field_count)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc simple_stop code=bad_argument", (unsigned long)request.request_id); }
        /* STOP never waits for the unused legacy experiment mailbox to be drained. */
        adrc_app_bridge_request_stop(bridge);
        if (request.request_id > bridge->bench.gateway.highest_admitted_request_id)
        { bridge->bench.gateway.highest_admitted_request_id = request.request_id; }
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc simple_stop=accepted", (unsigned long)request.request_id);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "simple_status"))
    {
        if (request.positional_count || request.field_count)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc simple_status code=bad_argument", (unsigned long)request.request_id); }
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc simple_status id=%lu phase=%u result=%u fault=%u safe_release=%u guard_ack=%u b0=%.9g count=%lu bridge_fault=%u profile=%u pulse_us=%lu limit=%.9g positive_us=%lu negative_us=%lu torque_trip=%.9g step=%.9g slew=%.9g probe=%u probe_peak=%.9g probe_reached=%u",
            (unsigned long)request.request_id, (unsigned long)bridge->simple_request_id,
            (unsigned int)bridge->simple.status.phase, (unsigned int)bridge->simple.status.result,
            (unsigned int)bridge->simple.status.fault, (unsigned int)bridge->simple.status.safe_release,
            (unsigned int)bridge->simple_guard_ack, (double)bridge->simple.status.b0,
            (unsigned long)bridge->simple.status.trace_count, (unsigned int)bridge->simple_bridge_fault,
            (unsigned int)bridge->simple.profile, (unsigned long)bridge->simple.pulse_duration_us,
            (double)bridge->simple.torque_limit_nm,
            (unsigned long)(bridge->simple.profile == 8U ? 640000U : (bridge->simple.profile >= 4U ? 312000U : bridge->simple.pulse_duration_us)),
            (unsigned long)(bridge->simple.profile == 8U ? 0U : bridge->simple.pulse_duration_us),
            (double)bridge->simple.torque_trip_nm, (double)bridge->simple.torque_step_nm,
            (double)bridge->simple.torque_slew_nm_s,
            (unsigned int)(bridge->simple.profile == 8U), (double)bridge->simple.probe_peak_nm,
            (unsigned int)bridge->simple.probe_reached);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "simple_trace"))
    {
        uint16_t index;
        AdrcSimpleTrace record;
        if (request.positional_count || !capture_index(&request, &index) ||
            bridge->simple.status.phase < ADRC_SIMPLE_DONE ||
            !adrc_simple_trace_get(&bridge->simple, index, &record))
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc simple_trace code=bad_argument", (unsigned long)request.request_id); }
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc simple_trace id=%lu index=%u t=%llu rx=%llu phase=%u p=%.9g v=%.9g raw_v=%.9g ref=%.9g requested=%.9g sent=%.9g torque_fb=%.9g enabled=%u z1=%.9g z2=%.9g",
            (unsigned long)request.request_id, (unsigned long)bridge->simple_request_id, (unsigned int)index,
            (unsigned long long)record.sample.timestamp_us, (unsigned long long)record.sample.feedback_us,
            (unsigned int)record.sample.state, (double)record.sample.position_rad,
            (double)record.sample.velocity_rad_s, (double)record.sample.raw_velocity_rad_s,
            (double)record.sample.reference_rad_s, (double)record.sample.torque_requested_nm,
            (double)record.sample.torque_sent_previous_nm, (double)record.feedback_torque_nm,
            (unsigned int)record.enabled, (double)record.sample.z1, (double)record.sample.z2);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "simple"))
    {
        const MotorDiscoveryResult *discovery = &bridge->runtime->discovery.results[6];
        TextProtocolSpan profile_field;
        uint8_t profile = 0U;
        /* Only fixed reviewed profiles are accepted; no arbitrary torque/time fields. */
        if (request.field_count)
        {
            if (request.field_count != 1U || !text_protocol_find_field(&request, "profile", &profile_field) ||
                profile_field.length != 1U || profile_field.data[0] < '0' || profile_field.data[0] > '8')
            { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                "error %lu adrc simple code=bad_profile", (unsigned long)request.request_id); }
            profile = (uint8_t)(profile_field.data[0] - '0');
        }
        if (!request.has_request_id || !request.request_id || request.positional_count ||
            request.request_id <= bridge->bench.gateway.highest_admitted_request_id ||
            bridge->simple_request_id || bridge->watchdog.state || bridge->bench.draft_config.axis_index != 6U ||
            bridge->guard_test.state != 5U || bridge->capture.state != 2U || bridge->capture.axis != 6U ||
            bridge->capture.count != ADRC_DISABLED_CAPTURE_CAPACITY ||
            !bridge->simple_hooks.arm_trial || !bridge->simple_hooks.renew_trial ||
            !bridge->guard_test.hooks.observe || !bridge->guard_test.hooks.ack_disabled ||
            !same_discovery(&bridge->capture.discovery, discovery) || !mit_discovered(bridge, 6U) ||
            discovery->communication_timeout_raw != 0U || bridge->frame_pending || bridge->awaiting_receipt ||
            bridge->stop_pending || bridge->release_disable_pending || bridge->send_failed ||
            bridge->bench.gateway.stop_pending || bridge->bench.gateway.active_run_request_id ||
            bridge->bench.experiment.status.state != ADRC_STATE_DISABLED ||
            !adrc_simple_init(&bridge->simple, bridge->bench.experiment.controller,
                bridge->bench.experiment.controller_context, discovery->ranges.position_max_rad * (2.0F / 65535.0F)) ||
            !adrc_simple_set_profile(&bridge->simple, profile))
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc simple code=not_ready", (unsigned long)request.request_id); }
        bridge->simple_discovery = *discovery; bridge->simple_request_id = request.request_id;
        bridge->simple_pending = 1U; bridge->simple_guard_ack = 0U; bridge->simple_guard_armed = 0U;
        bridge->simple_receipt = 0U; bridge->simple_bridge_fault = 0U; bridge->simple_lease_feedback_us = 0ULL;
        bridge->bench.gateway.highest_admitted_request_id = request.request_id;
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc simple=accepted", (unsigned long)request.request_id);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "watchdog_fault"))
    {
        const AdrcWatchdogProbe *probe = &bridge->watchdog;
        const MotorJointFeedback *feedback = &probe->failure_feedback;
        if (request.positional_count || request.field_count)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc watchdog_fault code=bad_argument", (unsigned long)request.request_id); }
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc watchdog_fault id=%lu reason=%u phase=%u observed_us=%llu rx_lower_us=%llu driver=%u fault=%u receipt=%u guard=%u deadline_ms=%lu fired_ms=%lu pos=%.7g vel=%.7g torque=%.7g mos=%.7g rotor=%.7g",
            (unsigned long)request.request_id, (unsigned long)probe->request_id,
            (unsigned int)probe->reason, (unsigned int)probe->failure_phase,
            (unsigned long long)probe->failure_observed_us, (unsigned long long)feedback->timestamp_us,
            (unsigned int)feedback->driver_state, (unsigned int)feedback->fault_flags,
            (unsigned int)probe->failure_receipt, (unsigned int)probe->failure_guard_state,
            (unsigned long)probe->failure_guard_deadline_ms, (unsigned long)probe->failure_guard_fired_ms,
            (double)feedback->position_rad, (double)feedback->velocity_rad_s, (double)feedback->torque_nm,
            (double)feedback->mos_temperature_c, (double)feedback->rotor_temperature_c);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "watchdog_status"))
    {
        const AdrcWatchdogProbe *probe = &bridge->watchdog;
        if (request.positional_count || request.field_count)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc watchdog_status code=bad_argument", (unsigned long)request.request_id); }
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc watchdog_status id=%lu state=%u reason=%u enabled=%u watchdog=%u restored=%u enable_us=%llu enabled_rx=%llu last_cmd=%llu tx_observed=%llu watchdog_rx=%llu watchdog_observed=%llu disabled_rx=%llu",
            (unsigned long)request.request_id, (unsigned long)probe->request_id,
            (unsigned int)probe->state, (unsigned int)probe->reason,
            (unsigned int)probe->enabled_seen, (unsigned int)probe->watchdog_seen,
            (unsigned int)probe->timeout_restored, (unsigned long long)probe->enable_request_us,
            (unsigned long long)probe->enabled_rx_us, (unsigned long long)probe->last_command_us,
            (unsigned long long)probe->last_command_observed_us, (unsigned long long)probe->watchdog_rx_us,
            (unsigned long long)probe->watchdog_observed_us, (unsigned long long)probe->disabled_rx_us);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "watchdog"))
    {
        AdrcWatchdogProbe *probe = &bridge->watchdog;
        if (!request.has_request_id || !request.request_id || request.positional_count || request.field_count ||
            request.request_id <= bridge->bench.gateway.highest_admitted_request_id || probe->state || bridge->simple_request_id ||
            bridge->guard_test.state != 5U || bridge->capture.state != 2U || bridge->capture.axis != 6U ||
            bridge->capture.count != ADRC_DISABLED_CAPTURE_CAPACITY ||
            !bridge->guard_test.hooks.arm || !bridge->guard_test.hooks.observe || !bridge->guard_test.hooks.ack_disabled ||
            bridge->bench.draft_config.axis_index != 6U || !mit_discovered(bridge, 6U) ||
            !same_discovery(&bridge->capture.discovery, &bridge->runtime->discovery.results[6]) ||
            bridge->runtime->discovery.results[6].communication_timeout_raw != 0U ||
            bridge->frame_pending || bridge->awaiting_receipt || bridge->send_failed || bridge->stop_pending ||
            bridge->release_disable_pending || bridge->bench.gateway.stop_pending ||
            bridge->bench.gateway.active_run_request_id || bridge->bench.experiment.status.state != ADRC_STATE_DISABLED)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc watchdog code=not_ready", (unsigned long)request.request_id); }
        probe->discovery = bridge->runtime->discovery.results[6];
        probe->started_us = timestamp_us; probe->request_id = request.request_id; probe->state = 1U;
        bridge->bench.gateway.highest_admitted_request_id = request.request_id;
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc watchdog state=1 timeout_raw=120 guard_ms=20", (unsigned long)request.request_id);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "guard_status"))
    {
        const AdrcStopGuardTest *test = &bridge->guard_test;
        if (request.positional_count || request.field_count)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc guard_status code=bad_argument", (unsigned long)request.request_id); }
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc guard_status id=%lu state=%u reason=%u deadline_ms=%lu fired_ms=%lu disabled_rx_us=%llu",
            (unsigned long)request.request_id, (unsigned long)test->request_id,
            (unsigned int)test->state, (unsigned int)test->reason,
            (unsigned long)test->deadline_ms, (unsigned long)test->fired_ms,
            (unsigned long long)test->disabled_rx_us);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "guard"))
    {
        AdrcStopGuardTest *test = &bridge->guard_test;
        if (!request.has_request_id || !request.request_id || request.positional_count || request.field_count ||
            request.request_id <= bridge->bench.gateway.highest_admitted_request_id || test->state || bridge->simple_request_id ||
            !test->hooks.arm || !test->hooks.observe || !test->hooks.ack_disabled ||
            bridge->bench.draft_config.axis_index != 6U || bridge->capture.axis != 6U ||
            bridge->capture.state != 2U || bridge->capture.count != ADRC_DISABLED_CAPTURE_CAPACITY ||
            !same_discovery(&bridge->capture.discovery, &bridge->runtime->discovery.results[6]) ||
            !mit_discovered(bridge, 6U) || bridge->frame_pending || bridge->awaiting_receipt ||
            bridge->stop_pending || bridge->release_disable_pending || bridge->send_failed ||
            bridge->bench.gateway.stop_pending || bridge->bench.gateway.active_run_request_id ||
            bridge->bench.experiment.status.state != ADRC_STATE_DISABLED)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc guard code=not_ready", (unsigned long)request.request_id); }
        test->started_us = timestamp_us; test->request_id = request.request_id; test->state = 1U;
        test->owner_epoch = bridge->capability_epoch;
        bridge->bench.gateway.highest_admitted_request_id = request.request_id;
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc guard state=1 disable_only=1", (unsigned long)request.request_id);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "sample"))
    {
        uint8_t axis = bridge->bench.draft_config.axis_index;
        if (!request.has_request_id || !request.request_id || request.positional_count || request.field_count ||
            request.request_id <= bridge->bench.gateway.highest_admitted_request_id ||
            bridge->simple_request_id || bridge->guard_test.state || bridge->capture.state == 1U || bridge->frame_pending || bridge->awaiting_receipt ||
            bridge->stop_pending || bridge->release_disable_pending || bridge->send_failed ||
            bridge->bench.gateway.stop_pending || bridge->bench.gateway.active_run_request_id ||
            bridge->bench.experiment.status.state != ADRC_STATE_DISABLED || !mit_discovered(bridge, axis))
        { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc sample code=not_ready", (unsigned long)request.request_id); }
        memset(&bridge->capture, 0, sizeof(bridge->capture));
        bridge->capture.axis = axis;
        bridge->capture.started_us = timestamp_us;
        bridge->capture.request_id = request.request_id;
        bridge->capture.owner_epoch = bridge->capability_epoch;
        bridge->capture.discovery = bridge->runtime->discovery.results[axis];
        bridge->capture.state = 1U;
        bridge->bench.gateway.highest_admitted_request_id = request.request_id;
        return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc sample state=1 capacity=64", (unsigned long)request.request_id);
    }
    if (text_protocol_request_path_equals(&request, "adrc", "capture"))
    {
        uint16_t index;
        if (!request.positional_count && !request.field_count)
        { return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc capture id=%lu motor=%u state=%u reason=%u count=%u requests=%u start_us=%llu first_request_us=%llu",
            (unsigned long)request.request_id, (unsigned long)bridge->capture.request_id,
            (unsigned int)bridge->capture.axis + 1U, (unsigned int)bridge->capture.state,
            (unsigned int)bridge->capture.reason, (unsigned int)bridge->capture.count,
            (unsigned int)bridge->capture.sent, (unsigned long long)bridge->capture.started_us,
            (unsigned long long)bridge->capture.first_request_us); }
        if (!request.positional_count && capture_index(&request, &index) &&
            bridge->capture.state >= 2U && index < bridge->capture.count)
        {
            const AdrcDisabledCaptureSample *sample = &bridge->capture.samples[index];
            return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
                "ok %lu adrc capture id=%lu index=%u observed_us=%llu rx_lower_us=%llu pos=%.9g vel=%.9g torque=%.9g",
                (unsigned long)request.request_id, (unsigned long)bridge->capture.request_id,
                (unsigned int)index, (unsigned long long)sample->observed_us,
                (unsigned long long)sample->rx_lower_us, (double)sample->position_rad,
                (double)sample->velocity_rad_s, (double)sample->torque_nm);
        }
        return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
            "error %lu adrc capture code=bad_argument", (unsigned long)request.request_id);
    }
    if ((text_protocol_request_path_equals(&request, "adrc", "hardware") != 0U) ||
        (text_protocol_request_path_equals(&request, "adrc", "limits") != 0U) ||
        (text_protocol_request_path_equals(&request, "adrc", "feedback") != 0U) ||
        (text_protocol_request_path_equals(&request, "adrc", "evidence") != 0U))
    {
        uint8_t axis = bridge->bench.draft_config.axis_index;
        const MotorDiscoveryResult *discovery;
        if (request.positional_count != 0U || request.field_count != 0U || axis >= ARM_JOINT_COUNT)
        {
            return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                "error %lu adrc code=bad_argument", (unsigned long)request.request_id);
        }
        discovery = &bridge->runtime->discovery.results[axis];
        if (text_protocol_request_path_equals(&request, "adrc", "evidence") != 0U)
        {
            uint32_t observed = 0U;
            uint32_t required = bridge->bench.draft_config.mode == ADRC_MODE_IDENTIFY ?
                ADRC_QUAL_IDENTIFY_REQUIRED : ADRC_QUAL_CLOSED_LOOP_REQUIRED;
            uint32_t verified;
            /* Observed readbacks are exported separately; neither arrival statistics nor
             * a watchdog register value establishes physical timing or watchdog action. */
            observe_binding(bridge);
            verified = bridge->bench.trusted_qualification.verified_flags;
            if (bridge->bench.draft_config.interface_profile == ADRC_INTERFACE_POSITION32_QUANTIZED)
            { required |= ADRC_QUAL_POSITION32_REQUIRED; }
            if ((discovery->verified_fields_mask & MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK) ==
                MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK) { observed |= ADRC_QUAL_IDENTITY; }
            if ((discovery->verified_fields_mask & MOTOR_DISCOVERY_RANGE_FIELDS_MASK) ==
                MOTOR_DISCOVERY_RANGE_FIELDS_MASK) { observed |= ADRC_QUAL_RANGES; }
            return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
                "ok %lu adrc evidence motor=%u observed=%lu verified=%lu required=%lu missing=%lu samples=%lu min_us=%lu max_us=%lu estimator=%u stop_bound=0",
                (unsigned long)request.request_id, (unsigned int)axis + 1U,
                (unsigned long)observed, (unsigned long)verified, (unsigned long)required,
                (unsigned long)(required & ~verified), (unsigned long)bridge->feedback_sample_count,
                (unsigned long)(bridge->feedback_interval_min_us == UINT32_MAX ? 0U : bridge->feedback_interval_min_us),
                (unsigned long)bridge->feedback_interval_max_us, (unsigned int)bridge->feedback_adapter.ready);
        }
        if (text_protocol_request_path_equals(&request, "adrc", "hardware") != 0U)
        {
            return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
                "ok %lu adrc hardware motor=%u discovery=%u fields=%04x accepted=%lu rejected=%lu esc=%lu master=%lu mode=%lu timeout_raw=%lu hw=%lu sw=%lu sub=%lu",
                (unsigned long)request.request_id, (unsigned int)axis + 1U,
                (unsigned int)bridge->runtime->discovery.state,
                (unsigned int)discovery->verified_fields_mask,
                (unsigned long)bridge->runtime->accepted_parameter_response_count,
                (unsigned long)bridge->runtime->rejected_parameter_response_count,
                (unsigned long)discovery->observed_esc_id,
                (unsigned long)discovery->observed_master_id,
                (unsigned long)discovery->observed_control_mode,
                (unsigned long)discovery->communication_timeout_raw,
                (unsigned long)discovery->hardware_version,
                (unsigned long)discovery->software_version,
                (unsigned long)discovery->sub_version);
        }
        if (text_protocol_request_path_equals(&request, "adrc", "limits") != 0U)
        {
            return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
                "ok %lu adrc limits motor=%u pmax=%.7g vmax=%.7g tmax=%.7g acc=%.7g dec=%.7g maxspd=%.7g",
                (unsigned long)request.request_id, (unsigned int)axis + 1U,
                (double)discovery->ranges.position_max_rad,
                (double)discovery->ranges.velocity_max_rad_s,
                (double)discovery->ranges.torque_max_nm,
                (double)discovery->acceleration_rad_s2,
                (double)discovery->deceleration_rad_s2,
                (double)discovery->maximum_speed_rad_s);
        }
        else
        {
            MotorFeedbackSnapshot snapshot;
            const MotorJointFeedback *feedback;
            uint64_t age_us = UINT64_MAX;
            uint8_t fresh;
            memset(&snapshot, 0, sizeof(snapshot));
            (void)motor_runtime_get_snapshot(bridge->runtime, timestamp_us, 8000ULL, &snapshot);
            feedback = &snapshot.joints[axis];
            fresh = (uint8_t)((snapshot.valid_joint_mask & (1U << axis)) != 0U);
            if (feedback->timestamp_us != 0ULL && timestamp_us >= feedback->timestamp_us)
            { age_us = timestamp_us - feedback->timestamp_us; }
            return format_query(output, PROTOCOL_ENGINE_STATUS_OK,
                "ok %lu adrc feedback motor=%u seen=%u fresh=%u state=%u fault=%lu age_us=%llu samples=%lu min_us=%lu max_us=%lu pos=%.7g vel=%.7g torque=%.7g mos=%.7g rotor=%.7g",
                (unsigned long)request.request_id, (unsigned int)axis + 1U,
                (unsigned int)bridge->runtime->bank.motors[axis].feedback_valid,
                (unsigned int)fresh, (unsigned int)feedback->driver_state,
                (unsigned long)feedback->fault_flags, (unsigned long long)age_us,
                (unsigned long)bridge->feedback_sample_count,
                (unsigned long)(bridge->feedback_interval_min_us == UINT32_MAX ? 0U :
                    bridge->feedback_interval_min_us),
                (unsigned long)bridge->feedback_interval_max_us,
                (double)feedback->position_rad, (double)feedback->velocity_rad_s,
                (double)feedback->torque_nm, (double)feedback->mos_temperature_c,
                (double)feedback->rotor_temperature_c);
        }
    }
    if ((bridge->simple_request_id || bridge->watchdog.state || bridge->capture.state == 1U || (bridge->guard_test.state && bridge->guard_test.state != 5U)) &&
        !text_protocol_request_path_equals(&request, "adrc", "status") &&
        !text_protocol_request_path_equals(&request, "adrc", "stop"))
    { return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        "error %lu adrc code=capture_active", (unsigned long)request.request_id); }
    allowed = (uint8_t)((request.command_words[0].length == 4U &&
        strncmp(request.command_words[0].data, "adrc", 4U) == 0) ||
        (request.command_words[0].length == 4U && strncmp(request.command_words[0].data, "show", 4U) == 0) ||
        text_protocol_request_path_equals(&request, "hello", NULL) ||
        text_protocol_request_path_equals(&request, "ping", NULL) ||
        text_protocol_request_path_equals(&request, "help", NULL));
    if (allowed)
    { return protocol_engine_process_text_line(bridge->engine, line, length, timestamp_us, output); }
    memset(output, 0, sizeof(*output));
    written = snprintf(output->messages[0].data, sizeof(output->messages[0].data),
        "error %lu command code=adrc_only\r\n", (unsigned long)request.request_id);
    if (written < 0 || (unsigned)written >= sizeof(output->messages[0].data))
    { return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL; }
    output->count = 1U; output->messages[0].length = (uint16_t)written;
    output->messages[0].priority = PROTOCOL_OUTPUT_HIGH_PRIORITY;
    return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
}

/** @brief Transfers the sole frame once and reserves its receipt until the next owner pass. */
uint8_t adrc_app_bridge_pop_frame(AdrcAppBridge *bridge, CanFrame *frame,
    uint8_t *kind, float *decoded_torque_nm)
{
    if (bridge == NULL || !bridge->initialized || frame == NULL || kind == NULL ||
        decoded_torque_nm == NULL || !bridge->frame_pending) { return 0U; }
    *frame = bridge->frame; *kind = bridge->frame_kind; *decoded_torque_nm = bridge->decoded_torque_nm;
    bridge->frame_pending = 0U; bridge->awaiting_receipt = 1U;
    return 1U;
}

/** @brief Accepts only the current frame's matched hardware receipt, never queue acceptance. */
void adrc_app_bridge_report_transmit(AdrcAppBridge *bridge, uint8_t kind,
    float decoded_torque_nm, uint8_t succeeded)
{
    if (bridge == NULL || !bridge->initialized) { return; }
    if (!bridge->awaiting_receipt || kind != bridge->frame_kind || !succeeded ||
        !finite_value(decoded_torque_nm) || decoded_torque_nm != bridge->decoded_torque_nm)
    { bridge->send_failed = 1U; bridge->receipt_valid = 0U; }
    else if (kind == 1U)
    { bridge->receipt_torque_nm = decoded_torque_nm; bridge->receipt_valid = 1U; }
    else if (kind == 0U)
    { bridge->receipt_torque_nm = 0.0F; bridge->receipt_valid = 0U; }
    if (bridge->watchdog.state && bridge->watchdog.state < 15U && bridge->send_failed == 0U)
    { bridge->watchdog.receipt = 1U; }
    if (bridge->simple_request_id && bridge->send_failed == 0U)
    {
        bridge->simple_receipt = 1U; bridge->simple_receipt_kind = kind;
        if (kind == 1U)
        {
            AdrcSimpleInput receipt;
            /* Ownership, kind, decoded value and hardware success matched above.
             * Observe now: a later STOP/guard/discovery abort must not hide this actual send. */
            memset(&receipt, 0, sizeof(receipt));
            receipt.tx_receipt_valid = bridge->simple_receipt; receipt.tx_kind = kind;
            receipt.feedback.prev_sent_valid = bridge->receipt_valid;
            receipt.feedback.prev_sent_nm = bridge->receipt_torque_nm;
            adrc_simple_observe_receipt(&bridge->simple, &receipt);
        }
    }
    if (bridge->send_failed == 0U && kind == 2U)
    { bridge->release_disable_pending = 0U; bridge->guard_test.receipt = 1U;
      if (bridge->capability_prepare_state == 1U && bridge->capability_prepare_submit_us)
      { bridge->capability_prepare_receipt = 1U; } }
    bridge->awaiting_receipt = 0U;
}

/** @brief Preserves actual completed observation before compatibility processing clears the transmit slot. */
void adrc_app_bridge_report_transmit_at(AdrcAppBridge *bridge, uint8_t kind,
    float decoded_torque_nm, uint8_t succeeded, uint64_t completed_observed_us)
{
    if (!bridge || !bridge->initialized) { return; }
    adrc_shaft_adapter_receipt(bridge, kind, decoded_torque_nm, succeeded, completed_observed_us);
    adrc_app_bridge_report_transmit(bridge, kind, decoded_torque_nm, succeeded);
}

/** @brief Revokes an unsent frame immediately and defers supervisor mutation to the owner. */
void adrc_app_bridge_request_stop(AdrcAppBridge *bridge)
{
    if (bridge != NULL && bridge->initialized)
    { if (bridge->shaft.used && !bridge->shaft.released) { bridge->shaft.stop_latched = 1U; }
      if (bridge->capabilities.used && !bridge->capability_terminal_closed)
      {
          bridge->capability_terminal_closed = 1U;
          if (bridge->capability_epoch != UINT32_MAX) { ++bridge->capability_epoch; }
      }
      if (bridge->capability_prepare_state == 1U) { bridge->capability_prepare_state = 3U; }
      motor_capability_abort(&bridge->capabilities, MOTOR_CAPABILITY_REASON_STOP);
      abort_capture(bridge, 5U); abort_guard_test(bridge, 5U); adrc_simple_abort(&bridge->simple);
      abort_watchdog(bridge, 5U, 0ULL);
      bridge->stop_pending = 1U; bridge->frame_pending = 0U; }
}

/** @brief Requires one matched disable receipt before the integrated owner can return to LCD. */
void adrc_app_bridge_request_release_disable(AdrcAppBridge *bridge)
{
    if (bridge != NULL && bridge->initialized)
    {
        /* Release intentionally revokes owner/capability binding. Close the
         * historical pilot only after actual stop proofs, before that revocation.
         * Ownership still requires its independent new DISABLE TX/RX evidence. */
        if (bridge->shaft.used && adrc_shaft_adapter_release_ready(bridge))
        { bridge->shaft.handoff_started = 1U; }
        bridge->release_disable_pending = 1U; adrc_app_bridge_request_stop(bridge);
    }
}

/** @brief Prevents a leftover ADRC slot from crossing a completed LCD handback. */
void adrc_app_bridge_complete_release(AdrcAppBridge *bridge)
{
    AdrcStopGuardHooks hooks;
    if (bridge == NULL || !bridge->initialized) { return; }
    if (bridge->capabilities.used) { bridge->capability_terminal_closed = 1U; }
    motor_capability_abort(&bridge->capabilities, MOTOR_CAPABILITY_REASON_STOP);
    if (bridge->capability_epoch != UINT32_MAX) { ++bridge->capability_epoch; }
    bridge->shaft.released = 1U; /* Retain frozen pilot trace and boot-used latch for read-only export. */
    hooks = bridge->guard_test.hooks;
    memset(&bridge->guard_test, 0, sizeof(bridge->guard_test));
    bridge->guard_test.hooks = hooks;
    memset(&bridge->watchdog, 0, sizeof(bridge->watchdog));
    memset(&bridge->simple, 0, sizeof(bridge->simple));
    bridge->simple_request_id = 0U; bridge->simple_pending = 0U;
    bridge->simple_guard_armed = 0U; bridge->simple_guard_ack = 0U; bridge->simple_receipt = 0U;
    bridge->capture.state = 0U;
    bridge->frame_pending = 0U;
    bridge->awaiting_receipt = 0U;
    bridge->stop_pending = 0U;
    bridge->release_disable_pending = 0U;
}

/** @brief Preserves transport failure until the next owner input and requests selected-axis disable. */
void adrc_app_bridge_report_fault(AdrcAppBridge *bridge)
{
    if (bridge != NULL && bridge->initialized)
    { bridge->send_failed = 1U; adrc_app_bridge_request_stop(bridge); }
}

/** @brief Consumes capability packets before the generic feedback decoder, including after release. */
uint8_t adrc_app_bridge_accept_capability_parameter(AdrcAppBridge *bridge,
    const CanFrame *frame, uint64_t rx_lower_us, uint64_t received_observed_us)
{
    uint8_t was_collecting;
    if (bridge == NULL || !bridge->initialized) { return 0U; }
    was_collecting = (uint8_t)(bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING);
    if ((bridge->capability_prepare_state == 1U || bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING) &&
        !capability_health(bridge, received_observed_us, 1U, 0U))
    { fail_capability(bridge, MOTOR_CAPABILITY_REASON_BINDING); }
    {
        uint8_t consumed = motor_capability_accept(&bridge->capabilities, frame, rx_lower_us, received_observed_us);
        if (was_collecting && bridge->capabilities.state == MOTOR_CAPABILITY_ABORTED && !bridge->stop_pending &&
            bridge->capabilities.reason != MOTOR_CAPABILITY_REASON_STOP)
        { fail_capability(bridge, bridge->capabilities.reason); }
        return consumed;
    }
}

/** @brief Matches only this owner's fixed capability request to a real completion observation. */
void adrc_app_bridge_report_capability_transmit(AdrcAppBridge *bridge,
    const CanFrame *frame, uint8_t succeeded, uint64_t completed_observed_us)
{
    uint8_t was_collecting;
    if (bridge == NULL || !bridge->initialized) { return; }
    was_collecting = (uint8_t)(bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING);
    if (bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING &&
        !capability_health(bridge, completed_observed_us, 1U, 0U))
    { fail_capability(bridge, MOTOR_CAPABILITY_REASON_BINDING); }
    (void)motor_capability_report_transmit(&bridge->capabilities, frame, succeeded,
        completed_observed_us, bridge->capability_epoch);
    if (was_collecting && bridge->capabilities.state == MOTOR_CAPABILITY_ABORTED && !bridge->stop_pending &&
        bridge->capabilities.reason != MOTOR_CAPABILITY_REASON_STOP)
    { fail_capability(bridge, bridge->capabilities.reason); }
}

/** @brief Runs discovery only outside experiments and never emits mode writes or actuation. */
MotorRuntimeStatus adrc_app_bridge_next_discovery(AdrcAppBridge *bridge, uint64_t timestamp_us,
    CanFrame *frame)
{
    uint8_t axis;
    MotorRuntimeStatus runtime_status;
    if (bridge == NULL || !bridge->initialized || frame == NULL) { return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT; }
    adrc_app_bridge_check_capability_health(bridge, timestamp_us);
    if (bridge->capability_prepare_state == 1U)
    { adrc_app_bridge_check_capability_health(bridge, timestamp_us); return MOTOR_RUNTIME_STATUS_WAITING; }
    if (bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING)
    {
        if (!capability_health(bridge, timestamp_us, 1U, 0U))
        { fail_capability(bridge, MOTOR_CAPABILITY_REASON_BINDING); return MOTOR_RUNTIME_STATUS_WAITING; }
        if (motor_capability_next(&bridge->capabilities, bridge->capability_epoch, timestamp_us, frame))
        { return MOTOR_RUNTIME_STATUS_FRAME_READY; }
        if (bridge->capabilities.state == MOTOR_CAPABILITY_ABORTED)
        { fail_capability(bridge, bridge->capabilities.reason); }
        return MOTOR_RUNTIME_STATUS_WAITING;
    }
    if (bridge->simple_request_id || bridge->watchdog.state || (bridge->guard_test.state && bridge->guard_test.state != 5U) ||
        bridge->capture.state == 1U || bridge->frame_pending || bridge->awaiting_receipt || bridge->stop_pending ||
        bridge->bench.gateway.active_run_request_id ||
        bridge->bench.experiment.status.state == ADRC_STATE_RUNNING ||
        bridge->bench.experiment.status.state == ADRC_STATE_STOPPING ||
        bridge->bench.experiment.status.state == ADRC_STATE_FAULT) { return MOTOR_RUNTIME_STATUS_WAITING; }
    axis = bridge->bench.draft_config.axis_index;
    if (axis != bridge->discovery_axis)
    {
        (void)motor_runtime_abort_active_parameter_sequences(bridge->runtime, timestamp_us);
        if (motor_runtime_begin_discovery(bridge->runtime, (uint8_t)(1U << axis)) != MOTOR_RUNTIME_STATUS_OK)
        { return MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR; }
        bridge->discovery_axis = axis;
        reset_feedback_statistics(bridge);
    }
    runtime_status = motor_runtime_next_discovery_frame(bridge->runtime, timestamp_us, frame);
    if (runtime_status != MOTOR_RUNTIME_STATUS_DISCOVERY_COMPLETE)
    { return runtime_status; }
    if (S3519_EXPLICIT_FEEDBACK_QUERY_VALIDATED == 0U ||
        timestamp_us < bridge->next_feedback_query_us)
    { return MOTOR_RUNTIME_STATUS_WAITING; }
    if (motor_adrc_build_feedback_query(bridge->runtime, axis, 4U,
            frame) != MOTOR_RUNTIME_STATUS_OK)
    { return MOTOR_RUNTIME_STATUS_CODEC_ERROR; }
    bridge->next_feedback_query_us = timestamp_us + ADRC_APP_BRIDGE_FEEDBACK_QUERY_PERIOD_US;
    if (bridge->feedback_query_count != UINT32_MAX) { ++bridge->feedback_query_count; }
    return MOTOR_RUNTIME_STATUS_FRAME_READY;
}

/** @brief Installs only locally supplied hardware evidence after actual disabled selected feedback. */
AdrcExperimentResult adrc_app_bridge_set_evidence(AdrcAppBridge *bridge,
    const AdrcExperimentConfig *config, const AdrcExperimentQualification *qualification,
    uint64_t timestamp_us)
{
    AdrcExperimentInput input;
    if (bridge == NULL || !bridge->initialized || config == NULL || qualification == NULL)
    { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (bridge->capability_prepare_state == 1U || bridge->capabilities.state == MOTOR_CAPABILITY_COLLECTING ||
        bridge->simple_request_id || bridge->watchdog.state ||
        (bridge->guard_test.state && bridge->guard_test.state != 5U) ||
        bridge->capture.state == 1U || qualification->provenance != ADRC_ENV_HARDWARE || config->axis_index != bridge->bench.draft_config.axis_index ||
        !mit_discovered(bridge, config->axis_index) || bridge->frame_pending || bridge->awaiting_receipt || bridge->stop_pending)
    { return ADRC_RESULT_UNQUALIFIED; }
    {
        const S3519Ranges *ranges = &bridge->runtime->discovery.results[config->axis_index].ranges;
        float position_range = ranges->position_max_rad * fabsf(bridge->bench.mapping[0]);
        float velocity_range = ranges->velocity_max_rad_s * fabsf(bridge->bench.mapping[1]);
        float torque_range = ranges->torque_max_nm * fabsf(bridge->bench.mapping[2]);
        float torque_quantum = torque_range * (2.0F / 4095.0F);
        float velocity_quantum = config->interface_profile == ADRC_INTERFACE_POSITION32_QUANTIZED ?
            position_range * (2.0F / 65535.0F) / 0.032F : velocity_range * (2.0F / 4095.0F);
        if (!finite_value(position_range) || !finite_value(velocity_range) || !finite_value(torque_range) ||
            position_range <= 0.0F || velocity_range <= 0.0F || torque_range <= 0.0F ||
            qualification->position_max_rad > position_range || qualification->velocity_max_rad_s > velocity_range ||
            qualification->torque_max_nm > torque_range ||
            !finite_value(config->velocity_quantum_rad_s) || config->velocity_quantum_rad_s < velocity_quantum ||
            !finite_value(config->torque_slew_nm_s) || torque_quantum > config->torque_slew_nm_s * 0.004F ||
            torque_quantum * 0.5F > config->torque_limit_nm)
        { return ADRC_RESULT_UNQUALIFIED; }
    }
    /* Installation evaluates the requested profile, never the legacy raw near-zero gate. */
    make_input(bridge, timestamp_us, config->interface_profile, &input);
    {
        AdrcExperimentResult result = adrc_bench_set_evidence(&bridge->bench, config, qualification, &input);
        if (result == ADRC_RESULT_OK)
        {
            bridge->evidence_config = *config;
            bridge->evidence_bound = 1U;
            bridge->observed_profile = config->interface_profile;
        }
        return result;
    }
}
