/** @file adrc_bench.c
 * @brief Bounded owner-task glue; no allocation, HAL, RTOS, transport, or fabricated hardware evidence.
 */
#include "adrc_bench.h"
#include <float.h>
#include <stddef.h>
#include <string.h>

/** @brief Checks ordinary context arguments without asserting on external input. */
static int valid_bench(const AdrcBench *bench)
{
    return bench != NULL && bench->initialized == 1U;
}

/** @brief Rejects nonfinite feedback before evaluating edit-time physical conditions. */
static int finite_value(float value)
{
    return value == value && value >= -FLT_MAX && value <= FLT_MAX;
}

/** @brief Requires actual fresh stationary disable before altering any draft or selected coordinate mapping. */
static int editable(const AdrcBench *bench, const AdrcExperimentInput *input)
{
    return (bench->experiment.status.state == ADRC_STATE_DISABLED ||
        bench->experiment.status.state == ADRC_STATE_PREPARED) && input != NULL &&
        input->feedback_valid == 1U && input->enabled == 0U && input->driver_fault == 0U &&
        input->axis_index == bench->draft_config.axis_index &&
        input->feedback_us <= input->now_us && input->now_us - input->feedback_us <= 8000ULL &&
        input->now_us >= bench->experiment.last_tick_us &&
        input->feedback_us >= bench->experiment.last_feedback_us &&
        finite_value(input->position_rad) && finite_value(input->velocity_rad_s) &&
        finite_value(input->mos_temperature_c) && finite_value(input->rotor_temperature_c) &&
        input->mos_temperature_c < 50.0F && input->rotor_temperature_c < 50.0F &&
        input->velocity_rad_s >= -bench->draft_config.near_zero_rad_s &&
        input->velocity_rad_s <= bench->draft_config.near_zero_rad_s;
}

/** @brief Computes published qualification solely from local evidence and the current draft's axis/mode. */
static uint8_t qualified(const AdrcBench *bench)
{
    uint32_t required = bench->draft_config.mode == ADRC_MODE_IDENTIFY ?
        ADRC_QUAL_IDENTIFY_REQUIRED : ADRC_QUAL_CLOSED_LOOP_REQUIRED;
    return (uint8_t)(bench->trusted_qualification.provenance == bench->experiment.environment &&
        bench->trusted_qualification.axis_index == bench->draft_config.axis_index &&
        (bench->trusted_qualification.verified_flags & required) == required);
}

/** @brief Publishes one coherent view while the caller holds the shared application critical hooks. */
static void publish(AdrcBench *bench)
{
    AdrcProtocolSnapshot snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    (void)adrc_experiment_get_status(&bench->experiment, &snapshot.status);
    snapshot.status.axis_index = bench->draft_config.axis_index;
    snapshot.qualified = qualified(bench);
    snapshot.trace_frozen = snapshot.status.trace_frozen;
    adrc_protocol_publish_snapshot(&bench->gateway, &snapshot);
}

/** @brief Exports only supervisor-approved frozen and physically disabled trace records. */
static uint8_t read_trace(void *context, uint32_t index, AdrcExperimentTrace *record)
{
    AdrcBench *bench = (AdrcBench *)context;
    return (uint8_t)(valid_bench(bench) &&
        adrc_experiment_trace_get(&bench->experiment, index, record) == ADRC_RESULT_OK);
}

/** @brief Maps an executed command to the gateway's permanently reserved completion channel. */
static unsigned terminal_channel(AdrcProtocolCommandType type)
{
    if (type == ADRC_PROTOCOL_STOP) { return 1U; }
    if (type == ADRC_PROTOCOL_HEARTBEAT) { return 2U; }
    return 0U;
}

/** @brief Retains an execution result independently until the protocol gateway accepts the terminal. */
static void retain_terminal(AdrcBench *bench, unsigned channel, uint32_t request_id,
    AdrcExperimentResult result, uint64_t now_us)
{
    bench->pending_terminal[channel].request_id = request_id;
    bench->pending_terminal[channel].result = result;
    bench->pending_terminal[channel].timestamp_us = now_us;
}

/** @brief Retries each of the three fixed completion slots once without blocking or discarding a result. */
static void flush_terminals(AdrcBench *bench)
{
    unsigned channel;
    for (channel = 0U; channel < 3U; channel++)
    {
        AdrcBenchTerminal *terminal = &bench->pending_terminal[channel];
        if (terminal->request_id != 0U && adrc_protocol_complete_command(&bench->gateway,
            terminal->request_id, terminal->result, terminal->timestamp_us))
        { terminal->request_id = 0U; }
    }
}

/** @brief Applies parsed draft fields only while disabled; never obtains trusted evidence from USB. */
static AdrcExperimentResult apply_config(AdrcBench *bench, const AdrcProtocolCommand *command,
    const AdrcExperimentInput *input)
{
    AdrcExperimentConfig config = bench->draft_config;
    uint32_t fields = command->field_mask;
    unsigned index;
    if (!editable(bench, input)) { return ADRC_RESULT_INVALID_STATE; }
    if (command->group == ADRC_CONFIG_MAPPING)
    {
        if (bench->experiment.status.state != ADRC_STATE_DISABLED) { return ADRC_RESULT_INVALID_STATE; }
        for (index = 0U; index < 3U; index++)
        {
            if ((fields & (1UL << index)) != 0U && bench->mapping[index] != command->values[index])
            {
                bench->mapping[index] = command->values[index];
                bench->trusted_qualification.verified_flags = 0U;
                bench->experiment.configured = 0U;
            }
        }
        return ADRC_RESULT_OK;
    }
    if (command->group == ADRC_CONFIG_CONTROL)
    {
        if (fields & 1U) { config.b0 = command->values[0]; }
        if (fields & 2U) { config.wc_rad_s = command->values[1]; }
        if (fields & 4U) { config.wo_rad_s = command->values[2]; }
        if (fields & 8U) { config.reference_rad_s = command->values[3]; }
        if (fields & 16U) { config.mode = (AdrcExperimentMode)(unsigned)command->values[4]; }
        if (fields & 32U) { config.duration_us = (uint32_t)command->values[5] * 1000U; }
    }
    else if (command->group == ADRC_CONFIG_LIMITS)
    {
        if (fields & 1U) { config.torque_limit_nm = command->values[0]; }
        if (fields & 2U) { config.torque_slew_nm_s = command->values[1]; }
        if (fields & 4U) { config.reference_accel_rad_s2 = command->values[2]; }
        if (fields & 8U) { config.near_zero_rad_s = command->values[3]; }
        if (config.reference_accel_rad_s2 > 0.3F) { return ADRC_RESULT_INVALID_ARGUMENT; }
    }
    else if (command->group == ADRC_CONFIG_IDENTIFY)
    {
        if (fields & 1U) { config.identify_torque_nm = command->values[0]; }
        if (fields & 2U) { config.identify_pulse_us = (uint32_t)command->values[1] * 1000U; }
    }
    else { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (config.b0 != bench->draft_config.b0)
    { bench->trusted_qualification.verified_flags &= ~ADRC_QUAL_IDENTIFIED_MODEL; }
    (void)adrc_experiment_stop(&bench->experiment, input->now_us);
    bench->draft_config = config;
    return ADRC_RESULT_OK;
}

/** @brief Selects one disabled axis and freezes a freshly validated draft before PREPARED. */
static AdrcExperimentResult prepare(AdrcBench *bench, const AdrcProtocolCommand *command,
    const AdrcExperimentInput *input)
{
    AdrcExperimentResult result;
    if (bench->experiment.status.state != ADRC_STATE_DISABLED &&
        bench->experiment.status.state != ADRC_STATE_PREPARED) { return ADRC_RESULT_INVALID_STATE; }
    if (command->axis_index >= 7U) { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (command->axis_index != bench->draft_config.axis_index)
    {
        int diagnostic_only = bench->experiment.status.state == ADRC_STATE_DISABLED &&
            !bench->evidence_installed && !bench->experiment.configured &&
            bench->trusted_qualification.verified_flags == 0U && bench->run_request_id == 0U &&
            !(input->feedback_valid == 1U && input->axis_index == bench->draft_config.axis_index &&
              input->enabled != 0U);
        if (!diagnostic_only && !editable(bench, input)) { return ADRC_RESULT_UNSAFE_FEEDBACK; }
        (void)adrc_experiment_stop(&bench->experiment, input->now_us);
        bench->trusted_qualification.verified_flags = 0U;
        bench->draft_config.axis_index = command->axis_index;
        bench->experiment.config.axis_index = command->axis_index;
        bench->experiment.status.axis_index = command->axis_index;
        bench->experiment.status.disabled_confirmed = 0U;
        bench->experiment.configured = 0U;
        return ADRC_RESULT_UNQUALIFIED;
    }
    if (!qualified(bench)) { return ADRC_RESULT_UNQUALIFIED; }
    result = adrc_experiment_configure(&bench->experiment, &bench->draft_config,
        &bench->trusted_qualification, input);
    if (result == ADRC_RESULT_OK) { result = adrc_experiment_prepare(&bench->experiment, input); }
    return result;
}

/** @brief Initializes conservative draft limits and connects the immutable trace reader. */
AdrcExperimentResult adrc_bench_init(AdrcBench *bench, AdrcExperimentEnvironment environment,
    AdrcControllerCallback controller, void *controller_context)
{
    AdrcExperimentResult result;
    if (bench == NULL) { return ADRC_RESULT_INVALID_ARGUMENT; }
    memset(bench, 0, sizeof(*bench));
    result = adrc_experiment_init(&bench->experiment, environment, controller, controller_context);
    if (result != ADRC_RESULT_OK) { return result; }
    adrc_protocol_init(&bench->gateway);
    bench->draft_config.mode = ADRC_MODE_IDENTIFY;
    bench->draft_config.duration_us = 3000000U;
    bench->draft_config.identify_pulse_us = 20000U;
    bench->draft_config.identify_torque_nm = 0.01F;
    bench->draft_config.reference_rad_s = 0.2F;
    bench->draft_config.reference_accel_rad_s2 = 0.3F;
    bench->draft_config.stop_accel_rad_s2 = 0.5F;
    bench->draft_config.torque_limit_nm = 0.1F;
    bench->draft_config.torque_slew_nm_s = 0.5F;
    bench->draft_config.velocity_quantum_rad_s = 0.001F;
    bench->draft_config.standstill_noise_rad_s = 0.0005F;
    bench->draft_config.near_zero_rad_s = 0.002F;
    bench->draft_config.wc_rad_s = 5.0F; bench->draft_config.wo_rad_s = 20.0F;
    bench->mapping[0] = 1.0F; bench->mapping[1] = 1.0F; bench->mapping[2] = 1.0F;
    bench->initialized = 1U;
    adrc_protocol_set_trace_reader(&bench->gateway, read_trace, bench);
    publish(bench);
    return ADRC_RESULT_OK;
}

/** @brief Returns the gateway only for a fully initialized caller-owned bench. */
AdrcProtocolGateway *adrc_bench_gateway(AdrcBench *bench)
{
    return valid_bench(bench) ? &bench->gateway : NULL;
}

/** @brief Validates locally supplied evidence with the supervisor and preserves its hardware provenance. */
AdrcExperimentResult adrc_bench_set_evidence(AdrcBench *bench, const AdrcExperimentConfig *config,
    const AdrcExperimentQualification *qualification, const AdrcExperimentInput *input)
{
    AdrcExperimentResult result;
    if (!valid_bench(bench) || config == NULL || qualification == NULL || input == NULL)
    { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (bench->run_request_id != 0U) { return ADRC_RESULT_INVALID_STATE; }
    result = adrc_experiment_configure(&bench->experiment, config, qualification, input);
    if (result == ADRC_RESULT_OK)
    {
        bench->draft_config = *config;
        bench->trusted_qualification = *qualification;
        bench->evidence_installed = 1U;
        publish(bench);
    }
    return result;
}

/** @brief Executes at most three priority commands and preserves accepted RUN until actual disabled terminal. */
AdrcExperimentResult adrc_bench_service(AdrcBench *bench, const AdrcExperimentInput *input,
    AdrcExperimentOutput *output)
{
    AdrcExperimentResult tick_result = ADRC_RESULT_OK;
    unsigned event_index;
    uint8_t started_run = 0U;
    if (output != NULL) { memset(output, 0, sizeof(*output)); }
    if (!valid_bench(bench) || input == NULL || output == NULL) { return ADRC_RESULT_INVALID_ARGUMENT; }
    flush_terminals(bench);
    for (event_index = 0U; event_index < 3U; event_index++)
    {
        AdrcProtocolCommand command;
        AdrcExperimentResult result;
        if (!adrc_protocol_pop_command(&bench->gateway, &command)) { break; }
        switch (command.type)
        {
        case ADRC_PROTOCOL_STOP:
            result = adrc_experiment_stop(&bench->experiment, input->now_us); break;
        case ADRC_PROTOCOL_HEARTBEAT:
            result = adrc_experiment_heartbeat(&bench->experiment, command.timestamp_us); break;
        case ADRC_PROTOCOL_CONFIG:
            result = apply_config(bench, &command, input); break;
        case ADRC_PROTOCOL_PREPARE:
            result = prepare(bench, &command, input); break;
        case ADRC_PROTOCOL_CLEAR:
            result = adrc_experiment_clear(&bench->experiment, input); break;
        case ADRC_PROTOCOL_RUN:
            result = adrc_experiment_run(&bench->experiment, input, output);
            if (result == ADRC_RESULT_OK)
            { bench->run_request_id = command.request_id; started_run = 1U; }
            break;
        default: result = ADRC_RESULT_INVALID_ARGUMENT; break;
        }
        if (command.type != ADRC_PROTOCOL_RUN || result != ADRC_RESULT_OK)
        { retain_terminal(bench, terminal_channel(command.type), command.request_id, result, input->now_us); }
    }
    if (!started_run) { tick_result = adrc_experiment_tick(&bench->experiment, input, output); }
    if (bench->run_request_id != 0U && bench->experiment.status.disabled_confirmed &&
        (bench->experiment.status.state == ADRC_STATE_DISABLED || bench->experiment.status.state == ADRC_STATE_FAULT))
    {
        retain_terminal(bench, 0U, bench->run_request_id,
            bench->experiment.status.fault == ADRC_FAULT_NONE ? ADRC_RESULT_OK : ADRC_RESULT_UNSAFE_FEEDBACK,
            input->now_us);
        bench->run_request_id = 0U;
    }
    publish(bench);
    flush_terminals(bench);
    return tick_result;
}
