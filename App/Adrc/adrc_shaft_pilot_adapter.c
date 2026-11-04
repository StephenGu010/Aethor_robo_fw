/** @file adrc_shaft_pilot_adapter.c
 * @brief Trusted motor-7 finite pilot binding and real observation bridge; no model or qualification. */
#include "adrc_app_bridge.h"
#include "motor_adrc_command.h"
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/** @brief Dedicated ordinary ZI object avoids growing the application's existing large BSS section. */
static ShaftPilot pilot_storage;
/** @brief Immutable terminal export snapshot; all callers serialize through application task critical hooks.
 * @note Both firmware profiles hold that boundary across parser, copy and formatter. No ISR calls this exporter. */
static ShaftPilotTrace trace_readback;

/** @brief Rejects NaN and infinite scalar evidence. */
static uint8_t finite_scalar(float value)
{ return (uint8_t)(value == value && value <= FLT_MAX && value >= -FLT_MAX); }
/** @brief Formats one bounded packet, including CRLF, without silently truncating any evidence. */
static ProtocolEngineStatus response(ProtocolOutputBatch *output, ProtocolEngineStatus status,
    uint32_t rid, const char *command, const char *format, ...)
{
    va_list arguments;
    int prefix, length;
    memset(output, 0, sizeof(*output));
    prefix = snprintf(output->messages[0].data, PROTOCOL_ENGINE_MESSAGE_CAPACITY,
        "%s %lu adrc %s ", status == PROTOCOL_ENGINE_STATUS_OK ? "ok" : "error",
        (unsigned long)rid, command);
    if (prefix < 0 || prefix >= (int)PROTOCOL_ENGINE_MESSAGE_CAPACITY - 3)
    { return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL; }
    va_start(arguments, format);
    length = vsnprintf(output->messages[0].data + prefix,
        PROTOCOL_ENGINE_MESSAGE_CAPACITY - (unsigned)prefix - 2U, format, arguments);
    va_end(arguments);
    if (length < 0 || (unsigned)length >= PROTOCOL_ENGINE_MESSAGE_CAPACITY - (unsigned)prefix - 2U)
    { return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL; }
    length += prefix;
    output->messages[0].data[length++] = '\r'; output->messages[0].data[length++] = '\n';
    output->messages[0].data[length] = '\0';
    output->messages[0].length = (uint16_t)length;
    output->messages[0].priority = PROTOCOL_OUTPUT_HIGH_PRIORITY; output->count = 1U;
    return status;
}
/** @brief Checks complete discovered identity and ranges explicitly, independent of struct padding. */
static uint8_t same_discovery(const MotorDiscoveryResult *left, const MotorDiscoveryResult *right)
{
    return (uint8_t)(left->verified_fields_mask == right->verified_fields_mask &&
        left->observed_master_id == right->observed_master_id && left->observed_esc_id == right->observed_esc_id &&
        left->observed_control_mode == right->observed_control_mode &&
        left->communication_timeout_raw == right->communication_timeout_raw &&
        left->hardware_version == right->hardware_version && left->software_version == right->software_version &&
        left->sub_version == right->sub_version && left->ranges.position_max_rad == right->ranges.position_max_rad &&
        left->ranges.velocity_max_rad_s == right->ranges.velocity_max_rad_s && left->ranges.torque_max_nm == right->ranges.torque_max_nm &&
        left->acceleration_rad_s2 == right->acceleration_rad_s2 && left->deceleration_rad_s2 == right->deceleration_rad_s2 &&
        left->maximum_speed_rad_s == right->maximum_speed_rad_s);
}
/** @brief Requires the actual owner and unchanged boot/epoch/identity/recorded six raw capabilities. */
static uint8_t binding_current(const AdrcAppBridge *bridge)
{
    const AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    unsigned index;
    uint8_t owner = bridge->capability_owner_active ? bridge->capability_owner_active() : bridge->capability_standalone_owner;
    if (!owner || adapter->boot_id != bridge->engine->boot_id || bridge->bench.draft_config.axis_index != 6U ||
        adapter->owner_epoch != bridge->capability_epoch || bridge->capabilities.epoch != adapter->owner_epoch ||
        bridge->capabilities.state != MOTOR_CAPABILITY_COMPLETE || bridge->capabilities.completed_revoked ||
        !same_discovery(&adapter->discovery, &bridge->runtime->discovery.results[6])) { return 0U; }
    for (index = 0U; index < MOTOR_CAPABILITY_COUNT; ++index)
    { if (!bridge->capabilities.items[index].valid || bridge->capabilities.items[index].raw_bits != adapter->capability_raw_bits[index]) { return 0U; } }
    return 1U;
}
/** @brief Encodes exactly one selected action; Motor owns the inward amplitude/step lattice selection. */
static uint8_t publish(AdrcAppBridge *bridge, const ShaftPilotOutput *output)
{
    MotorEmergencyFrameBatch batch;
    if (output->action == SHAFT_ACTION_NONE) { return 1U; }
    if (output->action == SHAFT_ACTION_TORQUE)
    {
        MotorAdrcTorqueRequest request;
        memset(&request, 0, sizeof(request)); request.axis_index = 6U; request.torque_scale = 1.0F;
        request.torque_nm = output->torque_nm; request.previous_sent_nm = output->previous_sent_nm;
        request.torque_limit_nm = output->cap_nm; request.torque_step_nm = output->maximum_step_nm;
        if (motor_adrc_build_torque_frame(bridge->runtime, &request, &bridge->frame,
            &bridge->decoded_torque_nm) != MOTOR_RUNTIME_STATUS_OK) { return 0U; }
        bridge->frame_kind = 1U;
    }
    else
    {
        if (motor_runtime_build_mode_command_batch(bridge->runtime, S3519_CONTROL_MODE_MIT,
            output->action == SHAFT_ACTION_ENABLE ? S3519_MODE_COMMAND_ENABLE : S3519_MODE_COMMAND_DISABLE,
            0x40U, &batch) != MOTOR_RUNTIME_STATUS_OK || batch.count != 1U) { return 0U; }
        bridge->frame = batch.frames[0]; bridge->decoded_torque_nm = 0.0F;
        bridge->frame_kind = output->action == SHAFT_ACTION_ENABLE ? 0U : 2U;
    }
    bridge->shaft.transmit_command_id = output->command_id ? output->command_id :
        (0x8000000000000000ULL | ++bridge->shaft.external_command_sequence);
    bridge->shaft.transmit_issued_us = output->issued_us ? output->issued_us : bridge->shaft.service_us;
    bridge->frame_pending = 1U; return 1U;
}
/** @brief Generates only an independent stop command, including before core start can be admitted. */
static uint8_t disable(AdrcAppBridge *bridge)
{
    ShaftPilotOutput output;
    memset(&output, 0, sizeof(output)); output.action = SHAFT_ACTION_DISABLE;
    return publish(bridge, &output);
}
/** @brief Builds immutable receive observation and retained completion before any guard/stop decision. */
static void make_input(AdrcAppBridge *bridge, uint64_t now_us, ShaftPilotInput *input)
{
    const MotorJointFeedback *feedback = &bridge->runtime->bank.motors[6].feedback;
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    uint32_t deadline = 0U, fired = 0U;
    uint8_t guard = 0U;
    memset(input, 0, sizeof(*input)); input->now_us = now_us; input->motor_id = 7U;
    input->binding_id = binding_current(bridge) ? adapter->binding_id : 0ULL;
    input->session_id = adapter->session_id;
    input->stop_requested = (uint8_t)(adapter->stop_latched || bridge->stop_pending ||
        bridge->bench.gateway.stop_pending || bridge->release_disable_pending || bridge->send_failed);
    input->receipt = adapter->receipt;
    input->guard_armed = adapter->guard_armed;
    if (adapter->guard_armed && !adapter->guard_ack)
    { guard = bridge->guard_test.hooks.observe(&deadline, &fired); }
    input->guard_live = (uint8_t)(guard == 1U);
    input->guard_fired = (uint8_t)(adapter->guard_ack || (guard == 3U && deadline == fired));
    input->guard_acked = adapter->guard_ack;
    input->feedback.valid = (uint8_t)(bridge->runtime->bank.motors[6].feedback_valid &&
        adapter->observed_rx_us == feedback->timestamp_us && adapter->observed_us >= feedback->timestamp_us);
    input->feedback.rx_us = feedback->timestamp_us; input->feedback.observed_us = adapter->observed_us;
    input->feedback.native_position_rad = feedback->position_rad; input->feedback.raw_velocity_rad_s = feedback->velocity_rad_s;
    input->feedback.torque_nm = feedback->torque_nm; input->feedback.mos_temperature_c = feedback->mos_temperature_c;
    input->feedback.rotor_temperature_c = feedback->rotor_temperature_c; input->feedback.driver_fault = feedback->fault_flags;
    input->feedback.driver_state = feedback->driver_state == S3519_DRIVER_STATE_DISABLED ? SHAFT_DRIVER_DISABLED :
        (feedback->driver_state == S3519_DRIVER_STATE_ENABLED ? SHAFT_DRIVER_ENABLED : SHAFT_DRIVER_UNKNOWN);
}
/** @brief Requires finite current health without using USB or service time as receive evidence. */
static uint8_t disabled_health(const AdrcAppBridge *bridge, const ShaftPilotInput *input)
{
    const ShaftPilotFeedback *feedback = &input->feedback;
    const ShaftPilot *pilot = bridge->shaft.core;
    return (uint8_t)(input->binding_id && feedback->valid && feedback->rx_us && feedback->rx_us <= feedback->observed_us &&
        feedback->observed_us <= input->now_us && input->now_us >= feedback->rx_us && input->now_us - feedback->rx_us <= 8000ULL &&
        feedback->driver_state == SHAFT_DRIVER_DISABLED && !feedback->driver_fault &&
        finite_scalar(feedback->native_position_rad) && finite_scalar(feedback->raw_velocity_rad_s) && finite_scalar(feedback->torque_nm) &&
        finite_scalar(feedback->mos_temperature_c) && finite_scalar(feedback->rotor_temperature_c) &&
        feedback->mos_temperature_c < 50.0F && feedback->rotor_temperature_c < 50.0F &&
        fabsf(feedback->native_position_rad) <= pilot->config.position_max_native_rad &&
        fabsf(feedback->raw_velocity_rad_s) <= fabsf(pilot->config.baseline.raw_zero_center_rad_s) * (1.0F + 4.0F * FLT_EPSILON) &&
        fabsf(feedback->raw_velocity_rad_s) <= 0.05F && fabsf(feedback->torque_nm) <= pilot->status.initial_stop_torque_gate_nm);
}
/** @brief Derives all baseline values from retained genuine 64-point capture, rejecting invalid causal windows. */
static uint8_t baseline(AdrcAppBridge *bridge, ShaftPilotConfig *config)
{
    unsigned index, previous;
    float minimum = FLT_MAX, maximum = -FLT_MAX, raw_center;
    uint8_t window_seen = 0U;
    const AdrcDisabledCapture *capture = &bridge->capture;
    const MotorDiscoveryResult *discovery = &bridge->runtime->discovery.results[6];
    float gear = bridge->capabilities.items[5].value;
    memset(config, 0, sizeof(*config));
    if (!finite_scalar(gear) || gear <= 0.0F || capture->state != 2U || capture->axis != 6U || capture->count != 64U ||
        capture->observed_proven_count != 64U || capture->owner_epoch != bridge->capability_epoch || !same_discovery(&capture->discovery, discovery)) { return 0U; }
    config->motor_id = 7U; config->binding_id = bridge->shaft.binding_id; config->session_id = bridge->shaft.session_id;
    config->gear_ratio = gear; config->position_max_native_rad = discovery->ranges.position_max_rad;
    config->velocity_max_rad_s = discovery->ranges.velocity_max_rad_s; config->torque_max_nm = discovery->ranges.torque_max_nm;
    config->baseline.binding_id = config->binding_id; config->baseline.session_id = config->session_id; config->baseline.point_count = 64U;
    raw_center = capture->samples[0].velocity_rad_s;
    config->baseline.raw_zero_center_rad_s = raw_center;
    for (index = 0U; index < 64U; ++index)
    {
        const AdrcDisabledCaptureSample *sample = &capture->samples[index];
        float position = sample->position_rad / gear;
        if (!sample->rx_lower_us || sample->rx_lower_us > sample->observed_us || !finite_scalar(position) ||
            !finite_scalar(sample->velocity_rad_s) || !finite_scalar(sample->torque_nm) ||
            fabsf(sample->velocity_rad_s) > 0.05F || fabsf(sample->velocity_rad_s) > fabsf(raw_center) * (1.0F + 4.0F * FLT_EPSILON) ||
            (index && (sample->rx_lower_us <= capture->samples[index - 1U].rx_lower_us ||
             sample->rx_lower_us - capture->samples[index - 1U].rx_lower_us > 8000ULL ||
             sample->observed_us < capture->samples[index - 1U].observed_us))) { return 0U; }
        minimum = fminf(minimum, position); maximum = fmaxf(maximum, position);
        config->baseline.torque_abs_peak_nm = fmaxf(config->baseline.torque_abs_peak_nm, fabsf(sample->torque_nm));
        for (previous = 0U; previous < index; ++previous)
        {
            const AdrcDisabledCaptureSample *old = &capture->samples[previous];
            uint64_t interval = sample->rx_lower_us - old->rx_lower_us;
            if (interval >= 32000ULL && interval <= 40000ULL)
            {
                float velocity = fabsf((position - old->position_rad / gear) / ((float)interval * 1.0e-6F));
                if (!finite_scalar(velocity) || sample->rx_lower_us <= old->observed_us + 2000ULL) { return 0U; }
                config->baseline.velocity32_abs_peak_rad_s = fmaxf(config->baseline.velocity32_abs_peak_rad_s, velocity);
                window_seen = 1U;
            }
        }
    }
    config->baseline.span64_shaft_rad = maximum - minimum;
    return window_seen;
}
/** @brief Binds the single fixed storage object on application init; no active reinitialization API exists. */
void adrc_shaft_adapter_init(AdrcAppBridge *bridge)
{
    memset(&pilot_storage, 0, sizeof(pilot_storage)); memset(&bridge->shaft, 0, sizeof(bridge->shaft));
    bridge->shaft.core = &pilot_storage;
}
/** @brief Accepts only newly decoded selected feedback; repeated service and parameter packets cannot refresh it. */
void adrc_shaft_adapter_note_feedback(AdrcAppBridge *bridge, uint32_t accepted_before,
    uint64_t rx_lower_us, uint64_t observed_us)
{
    if (bridge && bridge->initialized && bridge->runtime->accepted_feedback_count != accepted_before &&
        bridge->runtime->bank.motors[6].feedback.timestamp_us == rx_lower_us && rx_lower_us && observed_us >= rx_lower_us &&
        rx_lower_us > bridge->shaft.observed_rx_us)
    { bridge->shaft.observed_rx_us = rx_lower_us; bridge->shaft.observed_us = observed_us; }
}
/** @brief Records immutable first preparation submit only for an outstanding DISABLE. */
void adrc_shaft_adapter_note_submit(AdrcAppBridge *bridge, uint64_t timestamp_us)
{
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    if (adapter->used && adapter->prepare == 1U && !adapter->prepare_submit_us && bridge->awaiting_receipt &&
        bridge->frame_kind == 2U && timestamp_us >= adapter->prepare_started_us)
    { adapter->prepare_submit_us = timestamp_us; }
}
/** @brief Matches one hardware completion and saves local nonce before legacy receipt handling clears the slot. */
void adrc_shaft_adapter_receipt(AdrcAppBridge *bridge, uint8_t kind, float decoded, uint8_t success, uint64_t observed_us)
{
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    if (!adapter->used || adapter->released) { return; }
    if (!bridge->awaiting_receipt || kind != bridge->frame_kind || decoded != bridge->decoded_torque_nm ||
        !finite_scalar(decoded) || !observed_us || !success ||
        observed_us < adapter->transmit_issued_us)
    { adapter->bridge_fault = adapter->bridge_fault ? adapter->bridge_fault : 3U; adapter->stop_latched = 1U; return; }
    if (adapter->prepare == 1U)
    {
        if (kind == 2U && adapter->prepare_submit_us && observed_us >= adapter->prepare_submit_us)
        { adapter->prepare_receipt = 1U; adapter->prepare_completed_us = observed_us; }
        return;
    }
    if (adapter->prepare == 3U)
    { if (kind == 2U && observed_us >= adapter->prepare_completed_us)
      { adapter->cleanup_receipt = 1U; adapter->cleanup_completed_us = observed_us; } return; }
    adapter->receipt.valid = 1U; adapter->receipt.success = success;
    adapter->receipt.kind = kind == 0U ? SHAFT_ACTION_ENABLE : (kind == 1U ? SHAFT_ACTION_TORQUE : SHAFT_ACTION_DISABLE);
    adapter->receipt.command_id = adapter->transmit_command_id;
    adapter->receipt.completed_observed_us = observed_us; adapter->receipt.decoded_torque_nm = decoded;
}
/** @brief Acknowledges only exact actual guard fire followed by fresh healthy DISABLED and actual DISABLE receipt. */
static void acknowledge(AdrcAppBridge *bridge, const ShaftPilotInput *input, uint8_t safe, uint8_t receipt)
{
    uint32_t deadline = 0U, fired = 0U, elapsed;
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    if (!safe || !receipt || !adapter->guard_armed || adapter->guard_ack || !disabled_health(bridge, input)) { return; }
    if (bridge->guard_test.hooks.observe(&deadline, &fired) != 3U || deadline != fired) { return; }
    elapsed = (uint32_t)((uint32_t)(input->feedback.rx_us / 1000ULL) - fired);
    { uint64_t completed = adapter->prepare == 3U ? adapter->cleanup_completed_us : input->receipt.completed_observed_us;
      uint32_t completion_delta = (uint32_t)((uint32_t)(completed / 1000ULL) - fired);
      if (completion_delta >= 0x80000000UL) { return; } }
    if (elapsed && elapsed < 0x80000000UL && bridge->guard_test.hooks.ack_disabled()) { adapter->guard_ack = 1U; }
}
/** @brief Adds a stop-only causal history sample; no new baseline or motion permission is produced. */
static uint8_t cleanup_velocity(AdrcAppBridge *bridge, const ShaftPilotInput *input)
{
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    unsigned index;
    uint8_t ready = 0U;
    float current = input->feedback.native_position_rad / adapter->core->config.gear_ratio;
    float gate = adapter->core->config.baseline.velocity32_abs_peak_rad_s +
        2.0F * adapter->core->status.quantum_shaft_rad / 0.032F;
    if (!adapter->cleanup_history_count || (adapter->cleanup_rx_us &&
        input->feedback.rx_us - adapter->cleanup_rx_us > 8000ULL)) { adapter->cleanup_history_count = 0U; }
    if (!adapter->cleanup_history_count || input->feedback.rx_us != adapter->cleanup_rx_us)
    {
        if (adapter->cleanup_history_count == SHAFT_PILOT_HISTORY_CAPACITY)
        { memmove(adapter->cleanup_history, adapter->cleanup_history + 1,
            (SHAFT_PILOT_HISTORY_CAPACITY - 1U) * sizeof(adapter->cleanup_history[0])); --adapter->cleanup_history_count; }
        adapter->cleanup_history[adapter->cleanup_history_count].rx_us = input->feedback.rx_us;
        adapter->cleanup_history[adapter->cleanup_history_count].observed_us = input->feedback.observed_us;
        adapter->cleanup_history[adapter->cleanup_history_count++].shaft_position_rad = current;
    }
    for (index = 0U; index < adapter->cleanup_history_count; ++index)
    {
        const ShaftPilotSample *old = &adapter->cleanup_history[index];
        uint64_t interval = input->feedback.rx_us - old->rx_us;
        if (interval >= 32000ULL && interval <= 40000ULL)
        { float velocity = fabsf((current - old->shaft_position_rad) / ((float)interval * 1.0e-6F));
          if (!finite_scalar(velocity) || velocity > gate) { return 0U; } ready = 1U; }
    }
    return ready;
}
/** @brief Preparation failure continues independent disable and builds bounded healthy dwell without ENABLE. */
static void cleanup(AdrcAppBridge *bridge, const ShaftPilotInput *input)
{
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    float position = input->feedback.native_position_rad / adapter->core->config.gear_ratio;
    adapter->cleanup_safe_release = 0U;
    if (disabled_health(bridge, input) && adapter->cleanup_receipt && input->feedback.rx_us > adapter->prepare_completed_us &&
        cleanup_velocity(bridge, input))
    {
        if (input->feedback.rx_us != adapter->cleanup_rx_us)
        {
            if (!adapter->cleanup_dwell_us || !adapter->cleanup_rx_us || input->feedback.rx_us - adapter->cleanup_rx_us > 8000ULL)
            { adapter->cleanup_dwell_us = input->feedback.rx_us; adapter->cleanup_min_rad = position; adapter->cleanup_max_rad = position; }
            adapter->cleanup_min_rad = fminf(adapter->cleanup_min_rad, position); adapter->cleanup_max_rad = fmaxf(adapter->cleanup_max_rad, position);
            adapter->cleanup_rx_us = input->feedback.rx_us;
            if (adapter->cleanup_max_rad - adapter->cleanup_min_rad > adapter->core->config.baseline.span64_shaft_rad + 2.0F * adapter->core->status.quantum_shaft_rad)
            { adapter->cleanup_dwell_us = 0ULL; }
        }
        adapter->cleanup_safe_release = (uint8_t)(adapter->cleanup_dwell_us && input->feedback.rx_us - adapter->cleanup_dwell_us >= 64000ULL);
    }
    else { adapter->cleanup_dwell_us = 0ULL;
      if (!disabled_health(bridge, input)) { adapter->cleanup_history_count = 0U; }
      adapter->cleanup_rx_us = input->feedback.rx_us; }
    acknowledge(bridge, input, adapter->cleanup_safe_release, adapter->cleanup_receipt);
    (void)disable(bridge);
}
/** @brief Services finite prepare and core; guard renewal requires validated receipt plus genuinely new healthy feedback. */
uint8_t adrc_shaft_adapter_service(AdrcAppBridge *bridge, uint64_t now_us)
{
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    ShaftPilotInput input;
    ShaftPilotOutput output;
    uint8_t was_active;
    if (!adapter->used || adapter->released) { return 0U; }
    adapter->service_us = now_us;
    if (adapter->handoff_started)
    {
        /* The accepted handback revokes the old binding. Retain the completed
         * pilot outcome/trace and continue stop-only recovery independently.
         * A failed actual transmit remains a live bridge fault; no LCD release
         * or new trial is inferred from this historical terminal snapshot. */
        if (bridge->send_failed && !adapter->bridge_fault) { adapter->bridge_fault = 3U; }
        if (!disable(bridge)) { adapter->bridge_fault = 3U; }
        return 1U;
    }
    make_input(bridge, now_us, &input);
    if (bridge->send_failed && !adapter->bridge_fault) { adapter->bridge_fault = 3U; }
    if (!input.binding_id || input.stop_requested)
    { adapter->stop_latched = 1U; if (!input.binding_id && !adapter->bridge_fault) { adapter->bridge_fault = 1U; } }
    if (adapter->prepare == 1U)
    {
        if (adapter->stop_latched || now_us < adapter->prepare_started_us || now_us - adapter->prepare_started_us >= 64000ULL ||
            (input.feedback.valid && input.now_us >= input.feedback.rx_us && input.now_us - input.feedback.rx_us <= 8000ULL && !disabled_health(bridge, &input)))
        { adapter->prepare = 3U; adapter->prepare_completed_us = now_us; adapter->bridge_fault = adapter->bridge_fault ? adapter->bridge_fault : 4U; }
        else if (adapter->prepare_receipt && input.feedback.rx_us > adapter->prepare_submit_us && disabled_health(bridge, &input))
        {
            if (!disable(bridge) || !bridge->simple_hooks.arm_trial(&bridge->frame, (uint32_t)(now_us / 1000ULL)))
            { adapter->prepare = 3U; adapter->prepare_completed_us = now_us; adapter->bridge_fault = 2U; }
            else
            {
                adapter->guard_armed = 1U; input.guard_armed = 1U; input.guard_live = 1U;
                if (input.feedback.rx_us > adapter->lease_feedback_us) { adapter->lease_feedback_us = input.feedback.rx_us; } adapter->prepare = 2U;
                memset(&input.receipt, 0, sizeof(input.receipt));
                if (!shaft_pilot_start(adapter->core, &input, &output))
                { adapter->prepare = 3U; adapter->prepare_completed_us = now_us; adapter->bridge_fault = 4U; }
                else if (!publish(bridge, &output)) { adapter->bridge_fault = 3U; adapter->stop_latched = 1U; }
                else { return 1U; }
            }
        }
        else { (void)disable(bridge); return 1U; }
    }
    if (adapter->prepare == 3U)
    { if (!adapter->guard_armed && adapter->bridge_fault != 2U && disable(bridge))
      { adapter->guard_armed = bridge->simple_hooks.arm_trial(&bridge->frame, (uint32_t)(now_us / 1000ULL));
        if (!adapter->guard_armed) { adapter->bridge_fault = 2U; } }
      cleanup(bridge, &input); return 1U; }
    was_active = (uint8_t)(adapter->core->status.phase >= SHAFT_PHASE_ENABLE && adapter->core->status.phase <= SHAFT_PHASE_RAMP);
    shaft_pilot_tick(adapter->core, &input, &output);
    if (was_active && (output.action == SHAFT_ACTION_ENABLE || output.action == SHAFT_ACTION_TORQUE))
    {
        if (!input.receipt.valid || !input.receipt.success ||
            (input.feedback.rx_us > adapter->lease_feedback_us && !bridge->simple_hooks.renew_trial((uint32_t)(now_us / 1000ULL))))
        {
            adapter->bridge_fault = 2U; adapter->stop_latched = 1U;
            /* Output is never published after failed renew; the pending action receives explicit failed receipt next tick. */
            adapter->receipt.valid = 1U; adapter->receipt.success = 0U; adapter->receipt.kind = output.action;
            adapter->receipt.command_id = output.command_id; adapter->receipt.completed_observed_us = now_us;
            (void)disable(bridge); return 1U;
        }
        adapter->lease_feedback_us = input.feedback.rx_us;
    }
    acknowledge(bridge, &input, adapter->core->status.safe_release,
        (uint8_t)(input.receipt.valid && input.receipt.success && input.receipt.kind == SHAFT_ACTION_DISABLE));
    memset(&adapter->receipt, 0, sizeof(adapter->receipt));
    if (!publish(bridge, &output)) { adapter->bridge_fault = 3U; adapter->stop_latched = 1U; (void)disable(bridge); }
    return 1U;
}
/** @brief Returns only actual safe-disabled and guard-ack release readiness, never trial/model qualification. */
uint8_t adrc_shaft_adapter_release_ready(const AdrcAppBridge *bridge)
{
    const AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    return (uint8_t)(!adapter->used || (adapter->guard_ack &&
        (adapter->prepare == 3U ? adapter->cleanup_safe_release : adapter->core->status.safe_release)));
}
/** @brief Parses bounded decimal fields with strict overflow and no signs/floats. */
static uint8_t index_field(const TextProtocolRequest *request, const char *name, unsigned maximum, unsigned *value)
{
    TextProtocolSpan field;
    unsigned index, number = 0U;
    if (!text_protocol_find_field(request, name, &field) || !field.length) { return 0U; }
    for (index = 0U; index < field.length; ++index)
    { if (field.data[index] < '0' || field.data[index] > '9' || number > maximum / 10U) { return 0U; }
      number = number * 10U + (unsigned)(field.data[index] - '0'); if (number > maximum) { return 0U; } }
    *value = number; return 1U;
}
/** @brief Freezes readback and a fixed diagnostic while disabled; USB cannot select numeric plan values. */
static uint8_t admit(AdrcAppBridge *bridge, const TextProtocolRequest *request, int direction,
    uint8_t low_profile, uint64_t now_us)
{
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    const MotorDiscoveryResult *discovery = &bridge->runtime->discovery.results[6];
    ShaftPilotConfig config;
    ShaftPilotPlan plan;
    unsigned index;
    uint8_t owner = bridge->capability_owner_active ? bridge->capability_owner_active() : bridge->capability_standalone_owner;
    if (!request->has_request_id || !request->request_id || request->request_id <= bridge->bench.gateway.highest_admitted_request_id ||
        adapter->used || !owner || !bridge->engine->boot_id || bridge->simple_request_id || bridge->watchdog.state ||
        bridge->bench.draft_config.axis_index != 6U || discovery->verified_fields_mask != MOTOR_DISCOVERY_ALL_FIELDS_MASK ||
        discovery->observed_control_mode != 1U || discovery->observed_esc_id != 7U || discovery->observed_master_id != 23U ||
        discovery->communication_timeout_raw || bridge->capabilities.state != MOTOR_CAPABILITY_COMPLETE ||
        bridge->capabilities.count != 6U || bridge->capabilities.completed_revoked || bridge->capability_terminal_closed ||
        bridge->capabilities.epoch != bridge->capability_epoch || !same_discovery(&bridge->capability_discovery, discovery) ||
        bridge->guard_test.state != 5U || bridge->guard_test.owner_epoch != bridge->capability_epoch || !bridge->guard_test.request_id || !bridge->capture.request_id ||
        bridge->frame_pending || bridge->awaiting_receipt || bridge->stop_pending || bridge->release_disable_pending || bridge->send_failed ||
        bridge->bench.gateway.stop_pending || bridge->bench.gateway.active_run_request_id || bridge->bench.experiment.status.state != ADRC_STATE_DISABLED ||
        !bridge->simple_hooks.arm_trial || !bridge->simple_hooks.renew_trial || !bridge->guard_test.hooks.observe || !bridge->guard_test.hooks.ack_disabled)
    { return 0U; }
    for (index = 0U; index < 6U; ++index)
    { if (!bridge->capabilities.items[index].valid || !finite_scalar(bridge->capabilities.items[index].value)) { return 0U; } }
    adapter->boot_id = bridge->engine->boot_id; adapter->owner_epoch = bridge->capability_epoch;
    adapter->session_id = ((uint64_t)adapter->boot_id << 32U) | adapter->owner_epoch;
    adapter->binding_id = ((uint64_t)adapter->owner_epoch << 32U) | request->request_id;
    if (!baseline(bridge, &config)) { return 0U; }
    plan.direction = direction; plan.cap_nm = 1.75F; plan.slew_nm_s = 4.0F; plan.step_nm = 0.016F;
    if (low_profile == 8U ? !shaft_pilot_init_platform100(adapter->core, &config, direction) :
        low_profile == 7U ? !shaft_pilot_init_platform70(adapter->core, &config, direction) :
        low_profile == 6U ? !shaft_pilot_init_platform_wide(adapter->core, &config, direction) :
        low_profile == 5U ? !shaft_pilot_init_platform(adapter->core, &config, direction) :
        low_profile == 4U ? !shaft_pilot_init_observe(adapter->core, &config, direction) :
        low_profile == 3U ? !shaft_pilot_init_fine(adapter->core, &config, direction) :
        low_profile == 2U ? !shaft_pilot_init_visible(adapter->core, &config, direction) : low_profile ? !shaft_pilot_init_low(adapter->core, &config, direction) :
        !shaft_pilot_init(adapter->core, &config, &plan)) { return 0U; }
    adapter->discovery = *discovery; adapter->request_id = request->request_id;
    adapter->capture_id = bridge->capture.request_id; adapter->guard_id = bridge->guard_test.request_id;
    adapter->collector_id = bridge->capabilities.request_id;
    for (index = 0U; index < 6U; ++index) { adapter->capability_raw_bits[index] = bridge->capabilities.items[index].raw_bits; }
    /* An expired last sample is refreshed through the bounded DISABLE preparation only. */
    if (bridge->runtime->bank.motors[6].feedback.driver_state != S3519_DRIVER_STATE_DISABLED ||
        bridge->runtime->bank.motors[6].feedback.fault_flags || !bridge->runtime->bank.motors[6].feedback_valid ||
        !finite_scalar(bridge->runtime->bank.motors[6].feedback.position_rad) ||
        !finite_scalar(bridge->runtime->bank.motors[6].feedback.velocity_rad_s) ||
        !finite_scalar(bridge->runtime->bank.motors[6].feedback.torque_nm) ||
        !finite_scalar(bridge->runtime->bank.motors[6].feedback.mos_temperature_c) ||
        !finite_scalar(bridge->runtime->bank.motors[6].feedback.rotor_temperature_c) ||
        bridge->runtime->bank.motors[6].feedback.mos_temperature_c >= 50.0F ||
        bridge->runtime->bank.motors[6].feedback.rotor_temperature_c >= 50.0F ||
        fabsf(bridge->runtime->bank.motors[6].feedback.velocity_rad_s) > fabsf(config.baseline.raw_zero_center_rad_s)*(1.0F+4.0F*FLT_EPSILON) ||
        fabsf(bridge->runtime->bank.motors[6].feedback.torque_nm) > adapter->core->status.initial_stop_torque_gate_nm)
    { return 0U; }
    adapter->released = 0U; adapter->used = 1U; adapter->prepare = 1U; adapter->prepare_started_us = now_us;
    bridge->bench.gateway.highest_admitted_request_id = request->request_id;
    bridge->capability_terminal_closed = 1U;
    /* The control task publishes preparation; protocol admission never installs an expiring frame. */
    return 1U;
}
/** @brief Produces fixed immutable binding shards, with all time/session integers retained at full width. */
static ProtocolEngineStatus binding_response(AdrcAppBridge *bridge, const TextProtocolRequest *request, ProtocolOutputBatch *output)
{
    unsigned index;
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    const ShaftPilotConfig *config = &adapter->core->config;
    if (request->positional_count || request->field_count != 1U ||
        !index_field(request, "index", adapter->core->observe_profile ? 4U : 3U, &index) || !adapter->used)
    { return response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request->request_id, "shaft_binding", "code=bad_argument"); }
    /* 100%且1Nm档第五分片独立冻结；旧0-7身份及其格式保持原样。 */
    if (index == 4U && adapter->core->platform100_profile)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_binding",
        "id=%lu index=4 profile=8 speed=%.9g reverse=%.9g reserve=%.9g lookahead_us=%lu entry_us=%lu ramp_us=%lu plateau_us=%lu rated_rpm=%.9g",
        (unsigned long)adapter->request_id, (double)SHAFT_PLATFORM100_SPEED_RAD_S,
        (double)SHAFT_OBSERVE_REVERSE_NATIVE_RAD, (double)SHAFT_PLATFORM100_RESERVE_NATIVE_RAD,
        (unsigned long)SHAFT_PLATFORM_LOOKAHEAD_US, (unsigned long)SHAFT_PLATFORM100_ENTRY_US,
        (unsigned long)SHAFT_OBSERVE_RAMP_US, (unsigned long)SHAFT_OBSERVE_PLATEAU_US, (double)395.0F); }
    /* 70%档第五分片独立冻结；旧0-6身份及其格式保持原样。 */
    if (index == 4U && adapter->core->platform70_profile)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_binding",
        "id=%lu index=4 profile=7 speed=%.9g reverse=%.9g reserve=%.9g lookahead_us=%lu entry_us=%lu ramp_us=%lu plateau_us=%lu rated_rpm=%.9g",
        (unsigned long)adapter->request_id, (double)SHAFT_PLATFORM70_SPEED_RAD_S,
        (double)SHAFT_OBSERVE_REVERSE_NATIVE_RAD, (double)SHAFT_PLATFORM70_RESERVE_NATIVE_RAD,
        (unsigned long)SHAFT_PLATFORM_LOOKAHEAD_US, (unsigned long)SHAFT_PLATFORM70_ENTRY_US,
        (unsigned long)SHAFT_OBSERVE_RAMP_US, (unsigned long)SHAFT_OBSERVE_PLATEAU_US, (double)395.0F); }
    /* 新宽档身份独立导出入口预算；旧platform/observe分片与历史身份保持原样。 */
    if (index == 4U && adapter->core->platform_wide_profile)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_binding",
        "id=%lu index=4 profile=6 speed=%.9g reverse=%.9g reserve=%.9g lookahead_us=%lu entry_us=%lu ramp_us=%lu plateau_us=%lu rated_rpm=%.9g",
        (unsigned long)adapter->request_id, (double)SHAFT_OBSERVE_SPEED_RAD_S,
        (double)SHAFT_OBSERVE_REVERSE_NATIVE_RAD, (double)SHAFT_PLATFORM_WIDE_RESERVE_NATIVE_RAD,
        (unsigned long)SHAFT_PLATFORM_LOOKAHEAD_US, (unsigned long)SHAFT_PLATFORM_WIDE_ENTRY_US,
        (unsigned long)SHAFT_OBSERVE_RAMP_US, (unsigned long)SHAFT_OBSERVE_PLATEAU_US, (double)395.0F); }
    if (index == 4U && adapter->core->platform_profile)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_binding",
        "id=%lu index=4 profile=5 speed=%.9g reverse=%.9g reserve=%.9g lookahead_us=%lu ramp_us=%lu plateau_us=%lu rated_rpm=%.9g",
        (unsigned long)adapter->request_id, (double)SHAFT_OBSERVE_SPEED_RAD_S,
        (double)SHAFT_OBSERVE_REVERSE_NATIVE_RAD, (double)SHAFT_PLATFORM_RESERVE_NATIVE_RAD,
        (unsigned long)SHAFT_PLATFORM_LOOKAHEAD_US, (unsigned long)SHAFT_OBSERVE_RAMP_US,
        (unsigned long)SHAFT_OBSERVE_PLATEAU_US, (double)395.0F); }
    if (index == 4U)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_binding",
        "id=%lu index=4 profile=4 speed=%.9g angle=%.9g reverse=%.9g active=%.9g stopped=%.9g ramp_us=%lu plateau_us=%lu rated_rpm=%.9g",
        (unsigned long)adapter->request_id, (double)SHAFT_OBSERVE_SPEED_RAD_S, (double)SHAFT_OBSERVE_ANGLE_NATIVE_RAD,
        (double)SHAFT_OBSERVE_REVERSE_NATIVE_RAD, (double)SHAFT_OBSERVE_ACTIVE_NATIVE_RAD,
        (double)SHAFT_OBSERVE_STOP_NATIVE_RAD, (unsigned long)SHAFT_OBSERVE_RAMP_US,
        (unsigned long)SHAFT_OBSERVE_PLATEAU_US, (double)395.0F); }
    if (index == 0U)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_binding",
        "id=%lu index=0 boot=%lu session=%llu binding=%llu epoch=%lu capture=%lu guard=%lu collector=%lu direction=%d cap=%.9g slew=%.9g step=%.9g qualified=0 control=0",
        (unsigned long)adapter->request_id, (unsigned long)adapter->boot_id, (unsigned long long)adapter->session_id,
        (unsigned long long)adapter->binding_id, (unsigned long)adapter->owner_epoch, (unsigned long)adapter->capture_id,
        (unsigned long)adapter->guard_id, (unsigned long)adapter->collector_id, adapter->core->plan.direction,
        (double)adapter->core->plan.cap_nm, (double)adapter->core->plan.slew_nm_s, (double)adapter->core->plan.step_nm); }
    if (index == 1U)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_binding",
        "id=%lu index=1 gr=%.9g pmax=%.9g vmax=%.9g tmax=%.9g points=%lu span=%.9g velocity_peak=%.9g raw_center=%.9g torque_peak=%.9g qnative=%.9g qshaft=%.9g qvelocity=%.9g qtorque=%.9g",
        (unsigned long)adapter->request_id, (double)config->gear_ratio, (double)config->position_max_native_rad,
        (double)config->velocity_max_rad_s, (double)config->torque_max_nm, (unsigned long)config->baseline.point_count,
        (double)config->baseline.span64_shaft_rad, (double)config->baseline.velocity32_abs_peak_rad_s,
        (double)config->baseline.raw_zero_center_rad_s, (double)config->baseline.torque_abs_peak_nm,
        (double)adapter->core->status.quantum_native_rad, (double)adapter->core->status.quantum_shaft_rad,
        (double)adapter->core->status.quantum_velocity_rad_s, (double)adapter->core->status.quantum_torque_nm); }
    if (index == 2U)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_binding",
        "id=%lu index=2 motor=%lu master=%lu mode=%lu timeout=%lu fields=%u hw=%lu sw=%lu sub=%lu accel=%.9g decel=%.9g maxspeed=%.9g",
        (unsigned long)adapter->request_id, (unsigned long)adapter->discovery.observed_esc_id,
        (unsigned long)adapter->discovery.observed_master_id, (unsigned long)adapter->discovery.observed_control_mode,
        (unsigned long)adapter->discovery.communication_timeout_raw, (unsigned)adapter->discovery.verified_fields_mask,
        (unsigned long)adapter->discovery.hardware_version, (unsigned long)adapter->discovery.software_version,
        (unsigned long)adapter->discovery.sub_version, (double)adapter->discovery.acceleration_rad_s2,
        (double)adapter->discovery.deceleration_rad_s2, (double)adapter->discovery.maximum_speed_rad_s); }
    return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_binding",
        "id=%lu index=3 kt=%lu ot=%lu oc=%lu damp=%lu inertia=%lu gr=%lu valid_mask=63",
        (unsigned long)adapter->request_id, (unsigned long)adapter->capability_raw_bits[0],
        (unsigned long)adapter->capability_raw_bits[1], (unsigned long)adapter->capability_raw_bits[2],
        (unsigned long)adapter->capability_raw_bits[3], (unsigned long)adapter->capability_raw_bits[4],
        (unsigned long)adapter->capability_raw_bits[5]);
}
/** @brief Exports every raw field in four fixed shards only after terminal trace freeze. */
static ProtocolEngineStatus trace_response(AdrcAppBridge *bridge, const TextProtocolRequest *request, ProtocolOutputBatch *output)
{
    unsigned index, part;
    ShaftPilotTrace *record = &trace_readback;
    const AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    if (request->positional_count || request->field_count != 2U || !index_field(request, "index", 511U, &index) ||
        !index_field(request, "part", 3U, &part) || !adapter->core->trace_frozen ||
        !shaft_pilot_trace_get(adapter->core, index, record))
    { return response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request->request_id, "shaft_trace", "code=bad_argument"); }
    if (part == 0U)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_trace",
        "id=%lu index=%u part=0 now=%llu binding=%llu session=%llu motor=%u ga=%u gl=%u gf=%u gack=%u stop=%u valid=%u rx=%llu observed=%llu",
        (unsigned long)adapter->request_id, index, (unsigned long long)record->input.now_us,
        (unsigned long long)record->input.binding_id, (unsigned long long)record->input.session_id, (unsigned)record->input.motor_id,
        (unsigned)record->input.guard_armed, (unsigned)record->input.guard_live, (unsigned)record->input.guard_fired,
        (unsigned)record->input.guard_acked, (unsigned)record->input.stop_requested, (unsigned)record->input.feedback.valid,
        (unsigned long long)record->input.feedback.rx_us, (unsigned long long)record->input.feedback.observed_us); }
    if (part == 1U)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_trace",
        "id=%lu index=%u part=1 native_p=%.9g raw_v=%.9g torque=%.9g mos=%.9g rotor=%.9g driver=%u driver_fault=%lu",
        (unsigned long)adapter->request_id, index, (double)record->input.feedback.native_position_rad,
        (double)record->input.feedback.raw_velocity_rad_s, (double)record->input.feedback.torque_nm,
        (double)record->input.feedback.mos_temperature_c, (double)record->input.feedback.rotor_temperature_c,
        (unsigned)record->input.feedback.driver_state, (unsigned long)record->input.feedback.driver_fault); }
    if (part == 2U)
    { return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_trace",
        "id=%lu index=%u part=2 rvalid=%u rsuccess=%u rkind=%u rcmd=%llu rcompleted=%llu rdecoded=%.9g action=%u omotor=%u obinding=%llu osession=%llu ocmd=%llu issued=%llu prev_valid=%u requested=%.9g previous=%.9g step=%.9g cap=%.9g",
        (unsigned long)adapter->request_id, index, (unsigned)record->input.receipt.valid, (unsigned)record->input.receipt.success,
        (unsigned)record->input.receipt.kind, (unsigned long long)record->input.receipt.command_id,
        (unsigned long long)record->input.receipt.completed_observed_us, (double)record->input.receipt.decoded_torque_nm,
        (unsigned)record->output.action, (unsigned)record->output.motor_id, (unsigned long long)record->output.binding_id,
        (unsigned long long)record->output.session_id, (unsigned long long)record->output.command_id,
        (unsigned long long)record->output.issued_us, (unsigned)record->output.previous_sent_valid,
        (double)record->output.torque_nm, (double)record->output.previous_sent_nm,
        (double)record->output.maximum_step_nm, (double)record->output.cap_nm); }
    return response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_trace",
        "id=%lu index=%u part=3 phase=%u outcome=%u fault=%u shaft_p=%.9g v_rx32=%.9g vready=%u conditional=%u sampling_unknown=%u rxold=%llu obsold=%llu rxnew=%llu obsnew=%llu tmin=%llu tmax=%llu",
        (unsigned long)adapter->request_id, index, (unsigned)record->phase, (unsigned)record->outcome, (unsigned)record->first_fault,
        (double)record->shaft_position_rad, (double)record->velocity_rx32_rad_s, (unsigned)record->velocity_ready,
        (unsigned)record->arrival_interval_conditional, (unsigned)record->internal_sampling_unknown,
        (unsigned long long)record->window_rx_old_us, (unsigned long long)record->window_observed_old_us,
        (unsigned long long)record->window_rx_new_us, (unsigned long long)record->window_observed_new_us,
        (unsigned long long)record->window_tmin_us, (unsigned long long)record->window_tmax_us);
}
/** @brief Handles only the fixed finite entry and immutable exports; conflicting motion immediately latches stop. */
uint8_t adrc_shaft_adapter_process(AdrcAppBridge *bridge, const TextProtocolRequest *request,
    uint64_t now_us, ProtocolOutputBatch *output, ProtocolEngineStatus *status)
{
    AdrcShaftPilotAdapter *adapter = &bridge->shaft;
    if (text_protocol_request_path_equals(request, "adrc", "shaft_pilot"))
    {
        TextProtocolSpan direction, profile;
        int sign = 0;
        uint8_t low_profile = 0U;
        uint8_t valid_fields = (uint8_t)(request->field_count == 1U);
        if (request->field_count == 2U && text_protocol_find_field(request, "profile", &profile) &&
            profile.length == 3U && !memcmp(profile.data, "low", 3U))
        { low_profile = 1U; valid_fields = 1U; }
        if (request->field_count == 2U && text_protocol_find_field(request, "profile", &profile) &&
            profile.length == 7U && !memcmp(profile.data, "visible", 7U))
        { low_profile = 2U; valid_fields = 1U; }
        if (request->field_count == 2U && text_protocol_find_field(request, "profile", &profile) &&
            profile.length == 4U && !memcmp(profile.data, "fine", 4U))
        { low_profile = 3U; valid_fields = 1U; }
        if (request->field_count == 2U && text_protocol_find_field(request, "profile", &profile) &&
            profile.length == 7U && !memcmp(profile.data, "observe", 7U))
        { low_profile = 4U; valid_fields = 1U; }
        if (request->field_count == 2U && text_protocol_find_field(request, "profile", &profile) &&
            profile.length == 8U && !memcmp(profile.data, "platform", 8U))
        { low_profile = 5U; valid_fields = 1U; }
        if (request->field_count == 2U && text_protocol_find_field(request, "profile", &profile) &&
            profile.length == 13U && !memcmp(profile.data, "platform_wide", 13U))
        { low_profile = 6U; valid_fields = 1U; }
        if (request->field_count == 2U && text_protocol_find_field(request, "profile", &profile) &&
            profile.length == 10U && !memcmp(profile.data, "platform70", 10U))
        { low_profile = 7U; valid_fields = 1U; }
        if (request->field_count == 2U && text_protocol_find_field(request, "profile", &profile) &&
            profile.length == 11U && !memcmp(profile.data, "platform100", 11U))
        { low_profile = 8U; valid_fields = 1U; }
        if (!request->positional_count && valid_fields && text_protocol_find_field(request, "direction", &direction))
        { if (direction.length == 8U && !memcmp(direction.data, "positive", 8U)) { sign = 1; }
          if (direction.length == 8U && !memcmp(direction.data, "negative", 8U)) { sign = -1; } }
        if (!sign || !admit(bridge, request, sign, low_profile, now_us))
        { *status = response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request->request_id, "shaft_pilot", "code=not_ready_or_argument"); }
        else { *status = response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_pilot", "accepted=1 motor=7 qualified=0 control=0"); }
        return 1U;
    }
    if (text_protocol_request_path_equals(request, "adrc", "shaft_status"))
    {
        const ShaftPilotStatus *state = &adapter->core->status;
        uint8_t safe = adapter->prepare == 3U ? adapter->cleanup_safe_release : state->safe_release;
        if (request->field_count || request->positional_count)
        { *status = response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request->request_id, "shaft_status", "code=bad_argument"); }
        else
        { *status = response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "shaft_status",
            "id=%lu phase=%u outcome=%u fault=%u safe_release=%u guard_ack=%u ready=%u count=%lu bridge_fault=%u prepare=%u binding=%llu boot=%lu last=%.9g peak=%.9g qualified=0 control=0",
            (unsigned long)adapter->request_id, (unsigned)(adapter->prepare == 3U ? SHAFT_PHASE_LOCKED : state->phase),
            (unsigned)(adapter->prepare == 3U ? SHAFT_END_FAULT : state->outcome), (unsigned)state->first_fault,
            (unsigned)safe, (unsigned)adapter->guard_ack, (unsigned)(state->phase == SHAFT_PHASE_DONE && safe && adapter->guard_ack &&
            !adapter->bridge_fault && state->first_fault == SHAFT_FAULT_NONE &&
            (state->outcome == SHAFT_END_TARGET || state->outcome == SHAFT_END_TIME)), (unsigned long)state->trace_count,
            (unsigned)adapter->bridge_fault, (unsigned)adapter->prepare, (unsigned long long)adapter->binding_id,
            (unsigned long)adapter->boot_id, (double)state->last_successful_decoded_torque_nm,
            (double)state->peak_successful_decoded_torque_nm); }
        return 1U;
    }
    if (text_protocol_request_path_equals(request, "adrc", "shaft_binding"))
    { *status = binding_response(bridge, request, output); return 1U; }
    if (text_protocol_request_path_equals(request, "adrc", "shaft_trace"))
    { *status = trace_response(bridge, request, output); return 1U; }
    if (adapter->used && !adapter->released)
    {
        if (text_protocol_request_path_equals(request, "adrc", "stop"))
        {
            if (!request->has_request_id || request->positional_count || request->field_count)
            { *status = response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request->request_id, "stop", "code=bad_argument"); }
            else
            { adrc_app_bridge_request_stop(bridge); adapter->stop_latched = 1U;
              if (request->request_id > bridge->bench.gateway.highest_admitted_request_id) { bridge->bench.gateway.highest_admitted_request_id = request->request_id; }
              *status = response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, "stop", "shaft_stop=accepted"); }
            return 1U;
        }
        /* Existing bridge/gateway observation namespaces remain read-only during pilot ownership.
         * config/sample/prepare/run/heartbeat/clear and diagnostic start commands are mutations. */
        if (!text_protocol_request_path_equals(request, "adrc", NULL) &&
            !text_protocol_request_path_equals(request, "adrc", "status") &&
            !text_protocol_request_path_equals(request, "adrc", "evidence") &&
            !text_protocol_request_path_equals(request, "adrc", "hardware") &&
            !text_protocol_request_path_equals(request, "adrc", "limits") &&
            !text_protocol_request_path_equals(request, "adrc", "trace") &&
            !text_protocol_request_path_equals(request, "adrc", "simple_status") &&
            !text_protocol_request_path_equals(request, "adrc", "simple_trace") &&
            !text_protocol_request_path_equals(request, "adrc", "watchdog_status") &&
            !text_protocol_request_path_equals(request, "adrc", "watchdog_fault") &&
            !text_protocol_request_path_equals(request, "adrc", "feedback") &&
            !text_protocol_request_path_equals(request, "adrc", "capabilities_status") &&
            !text_protocol_request_path_equals(request, "adrc", "capabilities_get") &&
            !text_protocol_request_path_equals(request, "adrc", "capture") &&
            !text_protocol_request_path_equals(request, "adrc", "guard_status") &&
            !(request->command_words[0].length == 4U && !memcmp(request->command_words[0].data, "show", 4U)) &&
            !text_protocol_request_path_equals(request, "hello", NULL) && !text_protocol_request_path_equals(request, "ping", NULL) &&
            !text_protocol_request_path_equals(request, "help", NULL))
        { adrc_app_bridge_request_stop(bridge); adapter->stop_latched = 1U;
          *status = response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request->request_id, "shaft_pilot", "code=mutation_conflict"); return 1U; }
    }
    return 0U;
}
