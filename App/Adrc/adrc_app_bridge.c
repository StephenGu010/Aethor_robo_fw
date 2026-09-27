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

/** @brief Builds fresh output-shaft feedback; no draft mapping is promoted to measured evidence. */
static void make_input(AdrcAppBridge *bridge, uint64_t timestamp_us, AdrcExperimentInput *input)
{
    MotorFeedbackSnapshot snapshot;
    const MotorJointFeedback *feedback;
    uint8_t axis = bridge->bench.draft_config.axis_index;
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
    if (bridge->frame_pending || bridge->awaiting_receipt || bridge->stop_pending ||
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

/** @brief Expires the old slot before owner execution and selects disable over enable over torque. */
uint8_t adrc_app_bridge_service(AdrcAppBridge *bridge, uint64_t timestamp_us)
{
    AdrcExperimentInput input;
    AdrcExperimentOutput output;
    uint8_t force_disable;
    uint8_t ready = 0U;
    if (bridge == NULL || !bridge->initialized) { return 0U; }
    if (bridge->frame_pending || bridge->awaiting_receipt) { bridge->send_failed = 1U; }
    bridge->frame_pending = 0U;
    bridge->awaiting_receipt = 0U;
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
    make_input(bridge, timestamp_us, &input);
    observe_feedback(bridge, &input);
    bridge->receipt_valid = 0U;
    bridge->send_failed = 0U;
    (void)adrc_bench_service(&bridge->bench, &input, &output);
    if (force_disable || output.disable_requested) { ready = build_special(bridge, input.axis_index, 0U); }
    else if (output.enable_requested) { ready = build_special(bridge, input.axis_index, 1U); }
    else if (output.send_torque) { ready = build_torque(bridge, input.axis_index, output.torque_nm); }
    if ((force_disable || output.disable_requested || output.enable_requested || output.send_torque) && !ready)
    {
        bridge->send_failed = 1U;
        ready = build_special(bridge, input.axis_index, 0U);
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
    if ((text_protocol_request_path_equals(&request, "adrc", "hardware") != 0U) ||
        (text_protocol_request_path_equals(&request, "adrc", "limits") != 0U) ||
        (text_protocol_request_path_equals(&request, "adrc", "feedback") != 0U))
    {
        uint8_t axis = bridge->bench.draft_config.axis_index;
        const MotorDiscoveryResult *discovery;
        if (request.positional_count != 0U || request.field_count != 0U || axis >= ARM_JOINT_COUNT)
        {
            return format_query(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                "error %lu adrc code=bad_argument", (unsigned long)request.request_id);
        }
        discovery = &bridge->runtime->discovery.results[axis];
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
    if (bridge->send_failed == 0U && kind == 2U)
    { bridge->release_disable_pending = 0U; }
    bridge->awaiting_receipt = 0U;
}

/** @brief Revokes an unsent frame immediately and defers supervisor mutation to the owner. */
void adrc_app_bridge_request_stop(AdrcAppBridge *bridge)
{
    if (bridge != NULL && bridge->initialized)
    { bridge->stop_pending = 1U; bridge->frame_pending = 0U; }
}

/** @brief Requires one matched disable receipt before the integrated owner can return to LCD. */
void adrc_app_bridge_request_release_disable(AdrcAppBridge *bridge)
{
    if (bridge != NULL && bridge->initialized)
    { bridge->release_disable_pending = 1U; adrc_app_bridge_request_stop(bridge); }
}

/** @brief Prevents a leftover ADRC slot from crossing a completed LCD handback. */
void adrc_app_bridge_complete_release(AdrcAppBridge *bridge)
{
    if (bridge == NULL || !bridge->initialized) { return; }
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

/** @brief Runs discovery only outside experiments and never emits mode writes or actuation. */
MotorRuntimeStatus adrc_app_bridge_next_discovery(AdrcAppBridge *bridge, uint64_t timestamp_us,
    CanFrame *frame)
{
    uint8_t axis;
    MotorRuntimeStatus runtime_status;
    if (bridge == NULL || !bridge->initialized || frame == NULL) { return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT; }
    if (bridge->frame_pending || bridge->awaiting_receipt || bridge->stop_pending ||
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
    if (s3519_pack_feedback_query(
            (uint8_t)bridge->runtime->configuration->joints[axis].esc_id,
            frame) != S3519_CODEC_STATUS_OK)
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
    if (qualification->provenance != ADRC_ENV_HARDWARE || config->axis_index != bridge->bench.draft_config.axis_index ||
        !mit_discovered(bridge, config->axis_index) || bridge->frame_pending || bridge->awaiting_receipt || bridge->stop_pending)
    { return ADRC_RESULT_UNQUALIFIED; }
    {
        const S3519Ranges *ranges = &bridge->runtime->discovery.results[config->axis_index].ranges;
        float position_range = ranges->position_max_rad * fabsf(bridge->bench.mapping[0]);
        float velocity_range = ranges->velocity_max_rad_s * fabsf(bridge->bench.mapping[1]);
        float torque_range = ranges->torque_max_nm * fabsf(bridge->bench.mapping[2]);
        float torque_quantum = torque_range * (2.0F / 4095.0F);
        float velocity_quantum = velocity_range * (2.0F / 4095.0F);
        if (!finite_value(position_range) || !finite_value(velocity_range) || !finite_value(torque_range) ||
            position_range <= 0.0F || velocity_range <= 0.0F || torque_range <= 0.0F ||
            qualification->position_max_rad > position_range || qualification->velocity_max_rad_s > velocity_range ||
            qualification->torque_max_nm > torque_range ||
            !finite_value(config->velocity_quantum_rad_s) || config->velocity_quantum_rad_s < velocity_quantum ||
            !finite_value(config->torque_slew_nm_s) || torque_quantum > config->torque_slew_nm_s * 0.004F ||
            torque_quantum * 0.5F > config->torque_limit_nm)
        { return ADRC_RESULT_UNQUALIFIED; }
    }
    make_input(bridge, timestamp_us, &input);
    return adrc_bench_set_evidence(&bridge->bench, config, qualification, &input);
}
