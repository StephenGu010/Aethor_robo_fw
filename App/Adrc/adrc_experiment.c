/** @file adrc_experiment.c
 * @brief Fixed-memory single-axis experiment supervision; never accesses HAL, RTOS, CAN or USB.
 */
#include "adrc_experiment.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#define ADRC_MAX_FEEDBACK_AGE_US (8000ULL)
#define ADRC_MAX_PERIOD_US (6000ULL)
#define ADRC_HEARTBEAT_TIMEOUT_US (300000ULL)
#define ADRC_ENABLE_TIMEOUT_US (100000ULL)
#define ADRC_STOP_TIMEOUT_US (500000ULL)
#define ADRC_LOW_SPEED_DWELL_US (100000ULL)
#define ADRC_STEP_SECONDS (0.004F)

typedef char AdrcTraceRecordSizeCheck[(sizeof(AdrcExperimentTrace) <= 64U) ? 1 : -1];

/** @brief Tests float finiteness without depending on a particular C library isfinite macro. */
static int finite_value(float value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

/** @brief Returns magnitude for already finite values. */
static float magnitude(float value)
{
    return value < 0.0F ? -value : value;
}

/** @brief Moves toward a finite target by one fixed-period bounded increment. */
static float approach(float current, float target, float increment)
{
    if (target > current + increment) { return current + increment; }
    if (target < current - increment) { return current - increment; }
    return target;
}

/** @brief Validates context initialization for all ordinary APIs. */
static int valid_context(const AdrcExperiment *experiment)
{
    return experiment != NULL && experiment->initialized == 1U;
}

/** @brief Identifies states that own active experiment control. */
static int active_state(const AdrcExperiment *experiment)
{
    return experiment->status.state == ADRC_STATE_RUNNING ||
        experiment->status.state == ADRC_STATE_STOPPING;
}

/** @brief Resets every request before validation so failed APIs never leak stale commands. */
static void reset_output(const AdrcExperiment *experiment, AdrcExperimentOutput *output)
{
    if (output != NULL)
    {
        memset(output, 0, sizeof(*output));
        if (valid_context(experiment)) { output->axis_index = experiment->config.axis_index; }
    }
}

/** @brief Rejects malformed, future, stale, wrong-axis, or nonfinite feedback before any use. */
static int fresh_feedback(const AdrcExperimentInput *input, uint8_t axis_index)
{
    return input != NULL && input->feedback_valid == 1U && input->axis_index == axis_index &&
        input->enabled <= 1U && input->driver_fault <= 1U &&
        input->feedback_us <= input->now_us &&
        input->now_us - input->feedback_us <= ADRC_MAX_FEEDBACK_AGE_US &&
        finite_value(input->position_rad) && finite_value(input->velocity_rad_s) &&
        finite_value(input->mos_temperature_c) && finite_value(input->rotor_temperature_c);
}

/** @brief Checks fresh actual disable and near-zero speed before configuration, preparation or run. */
static int safe_disabled(const AdrcExperimentConfig *config,
    const AdrcExperimentQualification *qualification, const AdrcExperimentInput *input)
{
    return fresh_feedback(input, config->axis_index) && input->enabled == 0U &&
        input->driver_fault == 0U && input->mos_temperature_c < 50.0F &&
        input->rotor_temperature_c < 50.0F &&
        magnitude(input->velocity_rad_s) <= config->near_zero_rad_s &&
        magnitude(input->position_rad) <= qualification->position_max_rad;
}

/** @brief Validates the finite parameter set and discovered ranges without weakening fixed ceilings. */
static int valid_configuration(const AdrcExperimentConfig *config,
    const AdrcExperimentQualification *qualification)
{
    float resolution_threshold;
    if (config == NULL || qualification == NULL || config->axis_index >= 7U ||
        qualification->axis_index != config->axis_index ||
        (unsigned)config->mode > (unsigned)ADRC_MODE_LADRC ||
        config->duration_us == 0U || config->duration_us > 3000000U ||
        (config->identify_pulse_us != 20000U && config->identify_pulse_us != 40000U &&
         config->identify_pulse_us != 80000U) ||
        !finite_value(config->identify_torque_nm) || !finite_value(config->reference_rad_s) ||
        !finite_value(config->reference_accel_rad_s2) || !finite_value(config->stop_accel_rad_s2) ||
        !finite_value(config->torque_limit_nm) || !finite_value(config->torque_slew_nm_s) ||
        !finite_value(config->velocity_quantum_rad_s) || !finite_value(config->standstill_noise_rad_s) ||
        !finite_value(config->near_zero_rad_s) || !finite_value(config->b0) ||
        !finite_value(config->wc_rad_s) || !finite_value(config->wo_rad_s) ||
        !finite_value(qualification->position_max_rad) ||
        !finite_value(qualification->velocity_max_rad_s) ||
        !finite_value(qualification->torque_max_nm)) { return 0; }
    if (qualification->position_max_rad <= 0.0F || qualification->velocity_max_rad_s <= 0.0F ||
        qualification->torque_max_nm <= 0.0F || config->reference_accel_rad_s2 <= 0.0F ||
        config->reference_accel_rad_s2 > 0.3F || config->stop_accel_rad_s2 <= 0.0F ||
        config->stop_accel_rad_s2 > 0.5F || config->torque_limit_nm <= 0.0F ||
        config->torque_limit_nm > 0.1F || config->torque_limit_nm > qualification->torque_max_nm ||
        config->torque_slew_nm_s <= 0.0F || config->torque_slew_nm_s > 0.5F ||
        magnitude(config->reference_rad_s) > 0.2F ||
        magnitude(config->reference_rad_s) > qualification->velocity_max_rad_s ||
        magnitude(config->identify_torque_nm) > 0.05F || config->identify_torque_nm == 0.0F ||
        magnitude(config->identify_torque_nm) > config->torque_limit_nm ||
        config->velocity_quantum_rad_s <= 0.0F || config->velocity_quantum_rad_s > 0.015F ||
        config->standstill_noise_rad_s < 0.0F || config->standstill_noise_rad_s > 0.01F ||
        config->near_zero_rad_s <= 0.0F || config->near_zero_rad_s > 0.03F)
    { return 0; }
    resolution_threshold = 2.0F * config->velocity_quantum_rad_s;
    if (3.0F * config->standstill_noise_rad_s > resolution_threshold)
    { resolution_threshold = 3.0F * config->standstill_noise_rad_s; }
    if (config->near_zero_rad_s < resolution_threshold) { return 0; }
    if (config->mode == ADRC_MODE_IDENTIFY)
    { return config->identify_pulse_us <= config->duration_us; }
    return config->b0 > 0.0F && config->wc_rad_s > 0.0F &&
        config->wo_rad_s >= config->wc_rad_s && config->wo_rad_s <= 250.0F &&
        magnitude(config->reference_rad_s) > config->near_zero_rad_s;
}

/** @brief Requires trusted provenance and mode-specific evidence without circular identification gates. */
static int qualified(const AdrcExperiment *experiment, const AdrcExperimentConfig *config,
    const AdrcExperimentQualification *qualification)
{
    uint32_t required = config->mode == ADRC_MODE_IDENTIFY ? ADRC_QUAL_IDENTIFY_REQUIRED :
        ADRC_QUAL_CLOSED_LOOP_REQUIRED;
    return qualification->provenance == experiment->environment &&
        (qualification->verified_flags & required) == required;
}

/** @brief Latches a non-regressing disable watermark without claiming a frame reached the motor. */
static void request_disable(AdrcExperiment *experiment, uint64_t now_us)
{
    if (experiment->status.disable_requested == 0U)
    {
        experiment->disable_requested_us = now_us;
        if (experiment->last_tick_us > experiment->disable_requested_us)
        { experiment->disable_requested_us = experiment->last_tick_us; }
        if (experiment->last_feedback_us > experiment->disable_requested_us)
        { experiment->disable_requested_us = experiment->last_feedback_us; }
        experiment->status.disable_requested = 1U;
        experiment->status.disabled_confirmed = 0U;
    }
    experiment->status.awaiting_enable = 0U;
}

/** @brief Preserves the first fault and requests immediate zero/disable independently of slew limits. */
static void fault(AdrcExperiment *experiment, AdrcExperimentFault reason, uint64_t now_us,
    AdrcExperimentOutput *output)
{
    if (experiment->status.fault == ADRC_FAULT_NONE) { experiment->status.fault = reason; }
    experiment->status.state = ADRC_STATE_FAULT;
    request_disable(experiment, now_us);
    output->torque_nm = 0.0F;
    output->send_torque = 1U;
    output->enable_requested = 0U;
    output->disable_requested = 1U;
    experiment->previous_send_expected = 0U;
}

/** @brief Records one bounded immutable row; full logs retain earliest evidence and count loss. */
static void append_trace(AdrcExperiment *experiment, const AdrcExperimentInput *input,
    const AdrcExperimentOutput *output, const AdrcControllerOutput *controller, uint8_t new_sample)
{
    AdrcExperimentTrace *record;
    if (experiment->status.trace_frozen) { return; }
    if (experiment->status.trace_count >= ADRC_EXPERIMENT_TRACE_CAPACITY)
    {
        if (experiment->status.trace_overflow_count < UINT32_MAX)
        { experiment->status.trace_overflow_count++; }
        return;
    }
    record = &experiment->trace[experiment->status.trace_count++];
    memset(record, 0, sizeof(*record));
    record->timestamp_us = input->now_us;
    record->feedback_us = input->feedback_us;
    record->reference_rad_s = experiment->reference_rad_s;
    record->position_rad = input->position_rad;
    record->velocity_rad_s = input->velocity_rad_s;
    record->torque_raw_nm = controller->torque_raw_nm;
    record->torque_requested_nm = output->torque_nm;
    record->torque_sent_previous_nm = experiment->previous_sent_nm;
    record->z1 = controller->z1; record->z2 = controller->z2;
    record->sequence = experiment->next_sequence++;
    record->state = (uint8_t)experiment->status.state;
    record->fault = (uint8_t)experiment->status.fault;
    record->axis_index = experiment->config.axis_index;
    record->flags = (uint8_t)((new_sample ? 1U : 0U) |
        (experiment->status.disable_requested ? 2U : 0U) |
        (experiment->status.disabled_confirmed ? 4U : 0U) |
        (input->prev_sent_valid ? 8U : 0U) | (output->send_torque ? 16U : 0U));
}

/** @brief Accepts only fresh, non-regressing feedback after the disable request as disable evidence. */
static void update_disable_evidence(AdrcExperiment *experiment, const AdrcExperimentInput *input)
{
    experiment->status.disabled_confirmed = (uint8_t)(
        fresh_feedback(input, experiment->config.axis_index) && input->enabled == 0U &&
        input->now_us >= experiment->last_tick_us && input->feedback_us >= experiment->last_feedback_us &&
        (!experiment->status.disable_requested || input->feedback_us > experiment->disable_requested_us));
}

/** @brief Initializes fixed caller-owned storage without granting any evidence or actuator action. */
AdrcExperimentResult adrc_experiment_init(AdrcExperiment *experiment,
    AdrcExperimentEnvironment environment, AdrcControllerCallback controller, void *context)
{
    if (experiment == NULL || (environment != ADRC_ENV_HARDWARE && environment != ADRC_ENV_OFFLINE_FIXTURE))
    { return ADRC_RESULT_INVALID_ARGUMENT; }
    memset(experiment, 0, sizeof(*experiment));
    experiment->environment = environment;
    experiment->controller = controller;
    experiment->controller_context = context;
    experiment->initialized = 1U;
    return ADRC_RESULT_OK;
}

/** @brief Replaces frozen parameters only after checking current disabled evidence and qualification. */
AdrcExperimentResult adrc_experiment_configure(AdrcExperiment *experiment,
    const AdrcExperimentConfig *config, const AdrcExperimentQualification *qualification,
    const AdrcExperimentInput *input)
{
    if (!valid_context(experiment) || input == NULL || config == NULL || qualification == NULL)
    { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (active_state(experiment) || experiment->status.state == ADRC_STATE_FAULT)
    { return ADRC_RESULT_INVALID_STATE; }
    if (!valid_configuration(config, qualification)) { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (!qualified(experiment, config, qualification)) { return ADRC_RESULT_UNQUALIFIED; }
    if (config->mode != ADRC_MODE_IDENTIFY && experiment->controller == NULL)
    { return ADRC_RESULT_NO_CONTROLLER; }
    if (!safe_disabled(config, qualification, input) ||
        (experiment->tick_seen && input->now_us < experiment->last_tick_us))
    { return ADRC_RESULT_UNSAFE_FEEDBACK; }
    experiment->config = *config;
    experiment->qualification = *qualification;
    experiment->configured = 1U;
    experiment->status.axis_index = config->axis_index;
    experiment->status.state = ADRC_STATE_DISABLED;
    experiment->status.disable_requested = 0U;
    experiment->status.disabled_confirmed = 1U;
    experiment->status.awaiting_enable = 0U;
    experiment->last_tick_us = input->now_us;
    experiment->last_feedback_us = input->feedback_us;
    experiment->tick_seen = 1U;
    return ADRC_RESULT_OK;
}

/** @brief Prepares only a qualified selected axis from fresh stationary disabled feedback. */
AdrcExperimentResult adrc_experiment_prepare(AdrcExperiment *experiment,
    const AdrcExperimentInput *input)
{
    if (!valid_context(experiment) || input == NULL) { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (!experiment->configured) { return ADRC_RESULT_UNQUALIFIED; }
    if (experiment->status.state != ADRC_STATE_DISABLED) { return ADRC_RESULT_INVALID_STATE; }
    if (!qualified(experiment, &experiment->config, &experiment->qualification))
    { return ADRC_RESULT_UNQUALIFIED; }
    if (!safe_disabled(&experiment->config, &experiment->qualification, input) ||
        input->now_us < experiment->last_tick_us || input->feedback_us < experiment->last_feedback_us)
    { return ADRC_RESULT_UNSAFE_FEEDBACK; }
    experiment->status.state = ADRC_STATE_PREPARED;
    experiment->status.disabled_confirmed = 1U;
    experiment->last_tick_us = input->now_us;
    experiment->last_feedback_us = input->feedback_us;
    return ADRC_RESULT_OK;
}

/** @brief Starts a new finite experiment once; replayed RUN cannot restart timers or enable again. */
AdrcExperimentResult adrc_experiment_run(AdrcExperiment *experiment,
    const AdrcExperimentInput *input, AdrcExperimentOutput *output)
{
    reset_output(experiment, output);
    if (!valid_context(experiment) || input == NULL || output == NULL)
    { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (!experiment->configured) { return ADRC_RESULT_UNQUALIFIED; }
    if (experiment->status.state != ADRC_STATE_PREPARED) { return ADRC_RESULT_INVALID_STATE; }
    if (!safe_disabled(&experiment->config, &experiment->qualification, input) ||
        input->now_us < experiment->last_tick_us || input->feedback_us < experiment->last_feedback_us)
    { return ADRC_RESULT_UNSAFE_FEEDBACK; }
    memset(&experiment->status, 0, sizeof(experiment->status));
    experiment->status.state = ADRC_STATE_RUNNING;
    experiment->status.axis_index = experiment->config.axis_index;
    experiment->status.awaiting_enable = 1U;
    experiment->run_started_us = input->now_us;
    experiment->heartbeat_us = input->now_us;
    experiment->last_tick_us = input->now_us;
    experiment->last_feedback_us = input->feedback_us;
    experiment->initial_position_rad = input->position_rad;
    experiment->reference_rad_s = 0.0F;
    experiment->previous_sent_nm = 0.0F;
    experiment->previous_send_expected = 0U;
    experiment->low_speed_tracking = 0U;
    experiment->controller_reset = 1U;
    experiment->next_sequence = 0U;
    experiment->tick_seen = 1U;
    output->enable_requested = 1U;
    return ADRC_RESULT_OK;
}

/** @brief Refreshes the lease only for monotonic timestamps and never changes experiment state. */
AdrcExperimentResult adrc_experiment_heartbeat(AdrcExperiment *experiment, uint64_t now_us)
{
    if (!valid_context(experiment)) { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (now_us < experiment->last_tick_us || now_us < experiment->heartbeat_us)
    { return ADRC_RESULT_INVALID_ARGUMENT; }
    experiment->heartbeat_us = now_us;
    return ADRC_RESULT_OK;
}

/** @brief Enters STOPPING once; repeated stop calls cannot extend the bounded stop deadline. */
AdrcExperimentResult adrc_experiment_stop(AdrcExperiment *experiment, uint64_t now_us)
{
    if (!valid_context(experiment) || now_us < experiment->last_tick_us)
    { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (experiment->status.state == ADRC_STATE_RUNNING)
    {
        experiment->status.state = ADRC_STATE_STOPPING;
        experiment->stop_started_us = now_us;
        experiment->low_speed_tracking = 0U;
    }
    else if (experiment->status.state == ADRC_STATE_PREPARED)
    { experiment->status.state = ADRC_STATE_DISABLED; }
    return ADRC_RESULT_OK;
}

/** @brief Clears the software latch only after fresh safe disabled feedback; never clears drive faults. */
AdrcExperimentResult adrc_experiment_clear(AdrcExperiment *experiment,
    const AdrcExperimentInput *input)
{
    if (!valid_context(experiment) || input == NULL) { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (experiment->status.state != ADRC_STATE_FAULT) { return ADRC_RESULT_INVALID_STATE; }
    if (!safe_disabled(&experiment->config, &experiment->qualification, input) ||
        input->now_us < experiment->last_tick_us || input->feedback_us < experiment->last_feedback_us ||
        input->feedback_us <= experiment->disable_requested_us)
    { return ADRC_RESULT_UNSAFE_FEEDBACK; }
    experiment->status.state = ADRC_STATE_DISABLED;
    experiment->status.fault = ADRC_FAULT_NONE;
    experiment->status.disabled_confirmed = 1U;
    experiment->status.trace_frozen = 1U;
    experiment->status.awaiting_enable = 0U;
    experiment->previous_send_expected = 0U;
    experiment->last_tick_us = input->now_us;
    experiment->last_feedback_us = input->feedback_us;
    return ADRC_RESULT_OK;
}

/** @brief Determines immediate active safety faults before controller or actuator requests are generated. */
static AdrcExperimentFault active_fault(const AdrcExperiment *experiment,
    const AdrcExperimentInput *input)
{
    if (input->now_us < experiment->last_tick_us || input->now_us < experiment->heartbeat_us ||
        input->now_us < experiment->run_started_us ||
        (experiment->status.state == ADRC_STATE_STOPPING && input->now_us < experiment->stop_started_us))
    { return ADRC_FAULT_TIME; }
    if (input->now_us - experiment->last_tick_us > ADRC_MAX_PERIOD_US) { return ADRC_FAULT_PERIOD; }
    if (!fresh_feedback(input, experiment->config.axis_index) ||
        input->feedback_us < experiment->last_feedback_us) { return ADRC_FAULT_FEEDBACK; }
    if (input->driver_fault) { return ADRC_FAULT_DRIVER; }
    if (input->mos_temperature_c >= 50.0F || input->rotor_temperature_c >= 50.0F)
    { return ADRC_FAULT_TEMPERATURE; }
    if (magnitude(input->velocity_rad_s) > 0.3F ||
        magnitude(input->velocity_rad_s) > experiment->qualification.velocity_max_rad_s)
    { return ADRC_FAULT_SPEED; }
    if (magnitude(input->position_rad) > experiment->qualification.position_max_rad ||
        magnitude(input->position_rad - experiment->initial_position_rad) > 0.35F)
    { return ADRC_FAULT_TRAVEL; }
    if (input->prev_send_failed || (experiment->previous_send_expected &&
        (!input->prev_sent_valid || !finite_value(input->prev_sent_nm) ||
         magnitude(input->prev_sent_nm) > experiment->qualification.torque_max_nm)))
    { return ADRC_FAULT_SEND; }
    return ADRC_FAULT_NONE;
}

/** @brief Produces one limited torque using only an injected controller or finite identification pulse. */
static int produce_torque(AdrcExperiment *experiment, const AdrcExperimentInput *input,
    AdrcExperimentOutput *output, AdrcControllerOutput *controller_output, uint8_t new_sample)
{
    AdrcControllerInput controller_input;
    float limited_torque;
    if (experiment->config.mode == ADRC_MODE_IDENTIFY)
    {
        controller_output->torque_raw_nm = experiment->status.state == ADRC_STATE_RUNNING ?
            experiment->config.identify_torque_nm : 0.0F;
    }
    else
    {
        memset(&controller_input, 0, sizeof(controller_input));
        controller_input.mode = experiment->config.mode;
        controller_input.reference_rad_s = experiment->reference_rad_s;
        controller_input.velocity_rad_s = input->velocity_rad_s;
        controller_input.prev_sent_nm = experiment->previous_sent_nm;
        controller_input.b0 = experiment->config.b0;
        controller_input.wc_rad_s = experiment->config.wc_rad_s;
        controller_input.wo_rad_s = experiment->config.wo_rad_s;
        controller_input.reset = experiment->controller_reset;
        controller_input.new_sample = new_sample;
        if (experiment->controller == NULL ||
            !experiment->controller(experiment->controller_context, &controller_input, controller_output))
        { return 0; }
    }
    if (!finite_value(controller_output->torque_raw_nm) || !finite_value(controller_output->z1) ||
        !finite_value(controller_output->z2)) { return 0; }
    experiment->controller_reset = 0U;
    limited_torque = approach(experiment->previous_sent_nm, controller_output->torque_raw_nm,
        experiment->config.torque_slew_nm_s * ADRC_STEP_SECONDS);
    if (limited_torque > experiment->config.torque_limit_nm)
    { limited_torque = experiment->config.torque_limit_nm; }
    if (limited_torque < -experiment->config.torque_limit_nm)
    { limited_torque = -experiment->config.torque_limit_nm; }
    /* Identification excitation must end at its pulse deadline, without a torque tail. */
    if (experiment->config.mode == ADRC_MODE_IDENTIFY && experiment->status.state != ADRC_STATE_RUNNING)
    { limited_torque = 0.0F; }
    output->torque_nm = limited_torque;
    output->send_torque = 1U;
    experiment->previous_send_expected = 1U;
    return 1;
}

/** @brief Advances the bounded supervisor and separates disable requests from actual feedback evidence. */
AdrcExperimentResult adrc_experiment_tick(AdrcExperiment *experiment,
    const AdrcExperimentInput *input, AdrcExperimentOutput *output)
{
    AdrcControllerOutput controller_output;
    AdrcExperimentFault reason;
    uint8_t new_sample;
    uint8_t was_active;
    reset_output(experiment, output);
    if (!valid_context(experiment) || input == NULL || output == NULL)
    { return ADRC_RESULT_INVALID_ARGUMENT; }
    memset(&controller_output, 0, sizeof(controller_output));
    was_active = (uint8_t)active_state(experiment);
    new_sample = (uint8_t)(input->feedback_us > experiment->last_feedback_us);
    update_disable_evidence(experiment, input);
    /* Preserve valid actual transmit evidence even when this same tick trips another safety gate. */
    if (input->prev_sent_valid == 1U && !input->prev_send_failed &&
        finite_value(input->prev_sent_nm) &&
        magnitude(input->prev_sent_nm) <= experiment->qualification.torque_max_nm)
    { experiment->previous_sent_nm = input->prev_sent_nm; }
    if (was_active)
    {
        reason = active_fault(experiment, input);
        if (reason != ADRC_FAULT_NONE)
        { fault(experiment, reason, input->now_us, output); }
        else if (input->now_us - experiment->last_tick_us < ADRC_EXPERIMENT_PERIOD_US)
        { return ADRC_RESULT_INVALID_ARGUMENT; }
        else
        {
            if (experiment->previous_send_expected) { experiment->previous_sent_nm = input->prev_sent_nm; }
            experiment->previous_send_expected = 0U;
            if (experiment->status.state == ADRC_STATE_RUNNING &&
                (input->now_us - experiment->heartbeat_us >= ADRC_HEARTBEAT_TIMEOUT_US ||
                 input->now_us - experiment->run_started_us >= experiment->config.duration_us ||
                 (experiment->config.mode == ADRC_MODE_IDENTIFY &&
                  input->now_us - experiment->run_started_us >= experiment->config.identify_pulse_us)))
            { (void)adrc_experiment_stop(experiment, input->now_us); }
            if (experiment->status.state == ADRC_STATE_STOPPING && experiment->status.awaiting_enable)
            { request_disable(experiment, input->now_us); }
            if (experiment->status.disable_requested)
            {
                output->disable_requested = 1U;
                output->send_torque = 1U;
                if (experiment->status.disabled_confirmed)
                {
                    /* A late confirmation proves disable but cannot erase a missed stop deadline. */
                    if (input->feedback_us < experiment->stop_started_us ||
                        input->feedback_us - experiment->stop_started_us > ADRC_STOP_TIMEOUT_US)
                    { fault(experiment, ADRC_FAULT_STOP_TIMEOUT, input->now_us, output); }
                    else { experiment->status.state = ADRC_STATE_DISABLED; }
                }
            }
            else if (experiment->status.awaiting_enable)
            {
                if (input->now_us - experiment->run_started_us >= ADRC_ENABLE_TIMEOUT_US)
                { fault(experiment, ADRC_FAULT_ENABLE_TIMEOUT, input->now_us, output); }
                else if (input->enabled) { experiment->status.awaiting_enable = 0U; }
            }
            else if (!input->enabled)
            { fault(experiment, ADRC_FAULT_DRIVER, input->now_us, output); }
            if (active_state(experiment) && !experiment->status.awaiting_enable &&
                !experiment->status.disable_requested)
            {
                float target_reference = experiment->status.state == ADRC_STATE_RUNNING ?
                    experiment->config.reference_rad_s : 0.0F;
                float reference_increment = experiment->status.state == ADRC_STATE_RUNNING ?
                    experiment->config.reference_accel_rad_s2 : experiment->config.stop_accel_rad_s2;
                experiment->reference_rad_s = approach(experiment->reference_rad_s, target_reference,
                    reference_increment * ADRC_STEP_SECONDS);
                if (experiment->status.state == ADRC_STATE_STOPPING)
                {
                    if (magnitude(input->velocity_rad_s) <= experiment->config.near_zero_rad_s &&
                        experiment->reference_rad_s == 0.0F && new_sample)
                    {
                        if (!experiment->low_speed_tracking)
                        {
                            experiment->low_speed_started_us = input->now_us;
                            experiment->low_speed_tracking = 1U;
                        }
                        if (input->now_us - experiment->low_speed_started_us >= ADRC_LOW_SPEED_DWELL_US)
                        { request_disable(experiment, input->now_us); }
                    }
                    else if (magnitude(input->velocity_rad_s) > experiment->config.near_zero_rad_s ||
                        experiment->reference_rad_s != 0.0F)
                    { experiment->low_speed_tracking = 0U; }
                }
                if (experiment->status.disable_requested)
                { output->disable_requested = 1U; output->send_torque = 1U; }
                else if (!produce_torque(experiment, input, output, &controller_output, new_sample))
                { fault(experiment, ADRC_FAULT_CONTROLLER, input->now_us, output); }
            }
            if (experiment->status.state == ADRC_STATE_STOPPING &&
                input->now_us - experiment->stop_started_us >= ADRC_STOP_TIMEOUT_US)
            { fault(experiment, ADRC_FAULT_STOP_TIMEOUT, input->now_us, output); }
        }
    }
    if (experiment->status.state == ADRC_STATE_FAULT)
    {
        output->disable_requested = 1U;
        output->send_torque = 1U;
        output->torque_nm = 0.0F;
        output->enable_requested = 0U;
    }
    if (was_active || experiment->status.state == ADRC_STATE_FAULT)
    {
        append_trace(experiment, input, output, &controller_output, new_sample);
        if (experiment->status.disable_requested && experiment->status.disabled_confirmed)
        { experiment->status.trace_frozen = 1U; }
    }
    if (input->now_us >= experiment->last_tick_us) { experiment->last_tick_us = input->now_us; }
    if (fresh_feedback(input, experiment->config.axis_index) &&
        input->feedback_us >= experiment->last_feedback_us)
    { experiment->last_feedback_us = input->feedback_us; }
    experiment->tick_seen = 1U;
    return ADRC_RESULT_OK;
}

/** @brief Copies status without granting control or changing the experiment. */
AdrcExperimentResult adrc_experiment_get_status(const AdrcExperiment *experiment,
    AdrcExperimentStatus *status)
{
    if (!valid_context(experiment) || status == NULL) { return ADRC_RESULT_INVALID_ARGUMENT; }
    *status = experiment->status;
    return ADRC_RESULT_OK;
}

/** @brief Reads only a frozen log with disabled evidence, preventing concurrent mutation during export. */
AdrcExperimentResult adrc_experiment_trace_get(const AdrcExperiment *experiment,
    uint32_t index, AdrcExperimentTrace *record)
{
    if (!valid_context(experiment) || record == NULL) { return ADRC_RESULT_INVALID_ARGUMENT; }
    if (!experiment->status.trace_frozen || !experiment->status.disabled_confirmed ||
        active_state(experiment)) { return ADRC_RESULT_INVALID_STATE; }
    if (index >= experiment->status.trace_count) { return ADRC_RESULT_TRACE_UNAVAILABLE; }
    *record = experiment->trace[index];
    return ADRC_RESULT_OK;
}
