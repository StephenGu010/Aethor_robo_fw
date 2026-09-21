/** @file adrc_experiment_test_main.c
 * @brief Offline fault-injection contract tests; fixtures never constitute hardware evidence.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "adrc_experiment.h"

static AdrcExperiment experiment;
static AdrcExperimentConfig configuration;
static AdrcExperimentQualification qualification;
static AdrcExperimentInput feedback;
static AdrcExperimentOutput action;
static unsigned failures;
static unsigned checks;
static unsigned callback_calls;
static unsigned callback_new_samples;
static float callback_previous;
static float callback_reference;
static float callback_torque;

/** @brief Checks one behavioral condition without hiding later failures. */
static void expect(int condition, const char *description)
{
    checks++;
    if (!condition) { failures++; printf("FAIL: %s\n", description); }
}

#ifdef ADRC_TEST_RED_BASELINE
/** @brief Deliberately empty baseline used only to demonstrate the boot safety assertion failing. */
AdrcExperimentResult adrc_experiment_init(AdrcExperiment *value,
    AdrcExperimentEnvironment environment, AdrcControllerCallback controller, void *context)
{
    (void)environment; (void)controller; (void)context;
    memset(value, 0, sizeof(*value)); return ADRC_RESULT_OK;
}
/** @brief Missing supervisor behavior intentionally accepts an unsafe run in the red phase. */
AdrcExperimentResult adrc_experiment_run(AdrcExperiment *value,
    const AdrcExperimentInput *input, AdrcExperimentOutput *output)
{
    (void)value; (void)input; (void)output; return ADRC_RESULT_OK;
}
#endif

/** @brief Verifies reset can never grant hardware motion without measured qualification. */
static void test_boot_rejects_run(void)
{
    memset(&feedback, 0, sizeof(feedback));
    expect(adrc_experiment_init(&experiment, ADRC_ENV_HARDWARE, NULL, NULL) == ADRC_RESULT_OK,
        "initialize default hardware supervisor");
    expect(adrc_experiment_run(&experiment, &feedback, &action) != ADRC_RESULT_OK,
        "unqualified power-on RUN must be rejected");
}

#ifndef ADRC_TEST_RED_BASELINE
/** @brief Injects observable controller behavior without pretending it is generated control code. */
static int test_controller(void *context, const AdrcControllerInput *input,
    AdrcControllerOutput *output)
{
    (void)context;
    callback_calls++;
    callback_new_samples += input->new_sample;
    callback_previous = input->prev_sent_nm;
    callback_reference = input->reference_rad_s;
    output->torque_raw_nm = callback_torque;
    output->z1 = input->velocity_rad_s;
    output->z2 = 0.0F;
    return 1;
}

/** @brief Builds an explicit simulated fixture; its provenance is incompatible with hardware mode. */
static void fixture(void)
{
    memset(&configuration, 0, sizeof(configuration));
    memset(&qualification, 0, sizeof(qualification));
    memset(&feedback, 0, sizeof(feedback));
    memset(&action, 0, sizeof(action));
    callback_calls = 0U; callback_new_samples = 0U; callback_torque = 0.1F;
    configuration.mode = ADRC_MODE_LADRC;
    configuration.duration_us = 3000000U;
    configuration.identify_pulse_us = 20000U;
    configuration.identify_torque_nm = 0.01F;
    configuration.reference_rad_s = 0.2F;
    configuration.reference_accel_rad_s2 = 0.3F;
    configuration.stop_accel_rad_s2 = 0.5F;
    configuration.torque_limit_nm = 0.1F;
    configuration.torque_slew_nm_s = 0.5F;
    configuration.velocity_quantum_rad_s = 0.001F;
    configuration.standstill_noise_rad_s = 0.0005F;
    configuration.near_zero_rad_s = 0.002F;
    configuration.b0 = 1.0F; configuration.wc_rad_s = 5.0F; configuration.wo_rad_s = 20.0F;
    configuration.axis_index = 2U;
    qualification.verified_flags = ADRC_QUAL_CLOSED_LOOP_REQUIRED;
    qualification.provenance = ADRC_ENV_OFFLINE_FIXTURE;
    qualification.position_max_rad = 1.0F;
    qualification.velocity_max_rad_s = 1.0F;
    qualification.torque_max_nm = 1.0F;
    qualification.axis_index = 2U;
    feedback.now_us = 1000000ULL; feedback.feedback_us = feedback.now_us;
    feedback.feedback_valid = 1U; feedback.axis_index = 2U;
    feedback.mos_temperature_c = 25.0F; feedback.rotor_temperature_c = 25.0F;
    expect(adrc_experiment_init(&experiment, ADRC_ENV_OFFLINE_FIXTURE,
        test_controller, NULL) == ADRC_RESULT_OK, "fixture init");
}

/** @brief Configures and starts the selected simulated axis from actual disabled feedback. */
static void start(void)
{
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_OK, "fixture configure");
    expect(adrc_experiment_prepare(&experiment, &feedback) == ADRC_RESULT_OK, "fixture prepare");
    expect(adrc_experiment_run(&experiment, &feedback, &action) == ADRC_RESULT_OK, "fixture run");
    expect(action.enable_requested != 0U && action.axis_index == 2U, "enable selected axis only");
}

/** @brief Advances simulated coherent feedback and provides receipt only for requested sends. */
static void advance(uint64_t delta_us, int new_sample)
{
    feedback.prev_sent_valid = action.send_torque;
    feedback.prev_sent_nm = action.torque_nm;
    feedback.now_us += delta_us;
    if (new_sample) { feedback.feedback_us = feedback.now_us; }
    (void)adrc_experiment_tick(&experiment, &feedback, &action);
}

/** @brief Verifies trusted-evidence provenance, identification bootstrap, and immutable running config. */
static void test_configuration_gates(void)
{
    fixture();
    expect(adrc_experiment_init(&experiment, ADRC_ENV_HARDWARE, test_controller, NULL) == ADRC_RESULT_OK,
        "hardware init");
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_UNQUALIFIED, "offline evidence cannot unlock hardware");
    fixture(); qualification.verified_flags &= ~ADRC_QUAL_WATCHDOG;
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_UNQUALIFIED, "watchdog evidence required");
    fixture(); configuration.mode = ADRC_MODE_IDENTIFY; configuration.b0 = 0.0F;
    qualification.verified_flags = ADRC_QUAL_IDENTIFY_REQUIRED;
    expect(adrc_experiment_init(&experiment, ADRC_ENV_OFFLINE_FIXTURE, NULL, NULL) == ADRC_RESULT_OK,
        "identify without generated callback");
    start(); feedback.enabled = 1U; advance(4000ULL, 1);
    expect(action.send_torque && action.torque_nm > 0.0F, "bounded identify can bootstrap b0");
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_INVALID_STATE, "running configuration frozen");
    fixture(); configuration.near_zero_rad_s = 0.0001F;
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_INVALID_ARGUMENT, "near-zero respects quantization and noise");
    fixture(); configuration.reference_rad_s = NAN;
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_INVALID_ARGUMENT, "NaN config rejected");
    fixture(); configuration.mode = (AdrcExperimentMode)-1;
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_INVALID_ARGUMENT, "negative mode rejected regardless of enum signedness");
    fixture(); configuration.duration_us = 3000001U;
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_INVALID_ARGUMENT, "duration over three seconds rejected");
    fixture(); qualification.torque_max_nm = 0.05F;
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_INVALID_ARGUMENT, "discovered torque range restricts config");
}

/** @brief Tests timestamp freshness, per-axis identity, successful send feedback, and slew limiting. */
static void test_controller_contract(void)
{
    fixture(); start();
    advance(4000ULL, 1);
    expect(experiment.status.state == ADRC_STATE_RUNNING && experiment.status.awaiting_enable,
        "disabled feedback during bounded enable wait is allowed");
    feedback.enabled = 1U; advance(4000ULL, 1);
    expect(callback_calls == 1U && callback_new_samples == 1U, "first enabled sample resets controller");
    expect(fabsf(action.torque_nm - 0.002F) < 0.00001F, "torque slew uses four millisecond step");
    expect(fabsf(callback_reference - 0.0012F) < 0.00001F, "reference acceleration bounded");
    feedback.prev_sent_valid = 1U; feedback.prev_sent_nm = 0.0017F;
    feedback.now_us += 4000ULL;
    expect(adrc_experiment_tick(&experiment, &feedback, &action) == ADRC_RESULT_OK,
        "duplicate timestamp allowed while fresh");
    expect(callback_calls == 2U && callback_new_samples == 1U, "duplicate feedback never corrects observer twice");
    expect(fabsf(callback_previous - 0.0017F) < 0.000001F, "observer receives actual decoded send receipt");
    feedback.prev_sent_valid = 0U; feedback.now_us += 4000ULL; feedback.feedback_us = feedback.now_us;
    (void)adrc_experiment_tick(&experiment, &feedback, &action);
    expect(experiment.status.fault == ADRC_FAULT_SEND && action.disable_requested && action.torque_nm == 0.0F,
        "missing required send receipt faults immediately");
    fixture(); start(); feedback.enabled = 1U; feedback.axis_index = 1U; advance(4000ULL, 1);
    expect(experiment.status.fault == ADRC_FAULT_FEEDBACK, "wrong axis feedback cannot control selected axis");
}

/** @brief Applies independent safety violations and requires zero plus disable, never a success claim. */
static void test_immediate_faults(void)
{
    unsigned scenario;
    const AdrcExperimentFault expected[] = { ADRC_FAULT_PERIOD, ADRC_FAULT_FEEDBACK,
        ADRC_FAULT_TEMPERATURE, ADRC_FAULT_SPEED, ADRC_FAULT_TRAVEL,
        ADRC_FAULT_DRIVER, ADRC_FAULT_CONTROLLER, ADRC_FAULT_FEEDBACK, ADRC_FAULT_TIME };
    for (scenario = 0U; scenario < sizeof(expected) / sizeof(expected[0]); scenario++)
    {
        fixture(); start(); feedback.enabled = 1U;
        if (scenario == 0U) { feedback.now_us += 7000ULL; feedback.feedback_us = feedback.now_us; }
        else { feedback.now_us += 4000ULL; feedback.feedback_us = feedback.now_us; }
        if (scenario == 1U) { feedback.feedback_us = feedback.now_us - 8001ULL; }
        if (scenario == 2U) { feedback.rotor_temperature_c = 50.0F; }
        if (scenario == 3U) { feedback.velocity_rad_s = 0.301F; }
        if (scenario == 4U) { feedback.position_rad = 0.351F; }
        if (scenario == 5U) { feedback.driver_fault = 1U; }
        if (scenario == 6U) { callback_torque = INFINITY; }
        if (scenario == 7U) { feedback.velocity_rad_s = NAN; }
        if (scenario == 8U) { feedback.now_us = 1ULL; feedback.feedback_us = 1ULL; }
        (void)adrc_experiment_tick(&experiment, &feedback, &action);
        expect(experiment.status.fault == expected[scenario], "specific safety fault preserved");
        expect(action.disable_requested && action.send_torque && action.torque_nm == 0.0F,
            "fault requests immediate zero and disable");
        expect(!experiment.status.disabled_confirmed, "request does not prove disable");
    }
}

/** @brief Confirms orderly stop needs continuous fresh low-speed samples and post-request disabled feedback. */
static void test_stop_and_disable_evidence(void)
{
    unsigned index;
    fixture(); start(); feedback.enabled = 1U; advance(4000ULL, 1);
    expect(adrc_experiment_stop(&experiment, feedback.now_us) == ADRC_RESULT_OK, "stop accepted");
    for (index = 0U; index < 26U; index++) { advance(4000ULL, 1); }
    expect(action.disable_requested && !experiment.status.disabled_confirmed,
        "low speed dwell requests disable but does not claim success");
    feedback.enabled = 0U; advance(4000ULL, 1);
    expect(experiment.status.state == ADRC_STATE_DISABLED && experiment.status.disabled_confirmed,
        "fresh actual disabled completes normal stop");
    fixture(); start(); feedback.enabled = 1U; advance(4000ULL, 1);
    (void)adrc_experiment_stop(&experiment, feedback.now_us);
    feedback.velocity_rad_s = 0.1F;
    for (index = 0U; index < 126U; index++) { advance(4000ULL, 1); }
    expect(experiment.status.fault == ADRC_FAULT_STOP_TIMEOUT && action.disable_requested,
        "stop exceeding 500ms faults and disables");
    feedback.enabled = 0U; feedback.velocity_rad_s = 0.0F; advance(4000ULL, 1);
    expect(experiment.status.state == ADRC_STATE_FAULT && experiment.status.disabled_confirmed,
        "FAULT can confirm disable while preserving cause");
    expect(adrc_experiment_clear(&experiment, &feedback) == ADRC_RESULT_OK,
        "fresh disabled feedback allows explicit fault clear");
    expect(experiment.status.state == ADRC_STATE_DISABLED, "clear never restarts run");
}

/** @brief Exercises heartbeat expiry, enable timeout, finite pulse, overflow timestamps, and bounded logs. */
static void test_deadlines_and_trace(void)
{
    unsigned index;
    AdrcExperimentTrace record;
    fixture(); start(); feedback.enabled = 1U;
    for (index = 0U; index < 75U; index++) { advance(4000ULL, 1); }
    expect(experiment.status.state == ADRC_STATE_STOPPING, "300ms lease expiration begins stop");
    fixture(); start();
    for (index = 0U; index < 25U; index++) { advance(4000ULL, 1); }
    expect(experiment.status.fault == ADRC_FAULT_ENABLE_TIMEOUT, "enable confirmation limited to 100ms");
    fixture(); configuration.mode = ADRC_MODE_IDENTIFY; start(); feedback.enabled = 1U;
    for (index = 0U; index < 6U; index++) { advance(4000ULL, 1); }
    expect(action.torque_nm == 0.0F && experiment.status.state == ADRC_STATE_STOPPING,
        "identify pulse ends at configured finite deadline");
    fixture(); feedback.now_us = UINT64_MAX - 12000ULL; feedback.feedback_us = feedback.now_us;
    start(); feedback.enabled = 1U; advance(4000ULL, 1);
    expect(experiment.status.state == ADRC_STATE_RUNNING, "large timestamps do not overflow deadline arithmetic");
    feedback.now_us = 5ULL; feedback.feedback_us = 5ULL;
    (void)adrc_experiment_tick(&experiment, &feedback, &action);
    expect(experiment.status.fault == ADRC_FAULT_TIME, "wrapped timestamp fails safely");
    fixture(); start(); feedback.enabled = 1U; feedback.driver_fault = 1U; advance(4000ULL, 1);
    for (index = 0U; index < 1100U; index++) { advance(4000ULL, 1); }
    expect(experiment.status.trace_count == ADRC_EXPERIMENT_TRACE_CAPACITY,
        "trace never exceeds 1024 records");
    expect(experiment.status.trace_overflow_count > 0U, "full trace reports dropped records");
    expect(sizeof(AdrcExperimentTrace) <= 64U, "trace records fit 64 bytes");
    expect(adrc_experiment_trace_get(&experiment, 0U, &record) == ADRC_RESULT_INVALID_STATE,
        "fault trace cannot be read before disabled evidence and freeze");
    feedback.enabled = 0U; feedback.driver_fault = 0U; advance(4000ULL, 1);
    expect(experiment.status.disabled_confirmed && experiment.status.trace_frozen,
        "fault trace freezes only after observed disable");
    expect(adrc_experiment_trace_get(&experiment, 0U, &record) == ADRC_RESULT_OK && record.sequence == 0U,
        "trace retains first fault evidence rather than overwriting");
    expect(adrc_experiment_trace_get(&experiment, ADRC_EXPERIMENT_TRACE_CAPACITY, &record)
        == ADRC_RESULT_TRACE_UNAVAILABLE, "trace bounds are checked");
    expect(adrc_experiment_tick(NULL, &feedback, &action) == ADRC_RESULT_INVALID_ARGUMENT,
        "null input returns ordinary API error");
}

/** @brief Covers replay, inactive actions, exact freshness limits, frozen records and send evidence. */
static void test_additional_safety_boundaries(void)
{
    unsigned index;
    uint32_t frozen_count;
    uint64_t run_started;
    AdrcExperimentTrace record;
    AdrcExperimentStatus status;
    fixture();
    expect(adrc_experiment_tick(&experiment, &feedback, &action) == ADRC_RESULT_OK &&
        !action.enable_requested && !action.send_torque, "unconfigured tick never enables or drives");
    start(); run_started = experiment.run_started_us;
    expect(adrc_experiment_run(&experiment, &feedback, &action) == ADRC_RESULT_INVALID_STATE &&
        !action.enable_requested && experiment.run_started_us == run_started,
        "replayed RUN cannot restart or repeat enable");
    feedback.enabled = 1U; advance(6000ULL, 1);
    expect(experiment.status.state == ADRC_STATE_RUNNING, "six millisecond period remains admissible");
    advance(4000ULL, 0); advance(4000ULL, 0);
    expect(experiment.status.state == ADRC_STATE_RUNNING, "eight millisecond feedback age remains admissible");
    advance(4000ULL, 0);
    expect(experiment.status.fault == ADRC_FAULT_FEEDBACK, "feedback age beyond eight milliseconds faults");
    fixture(); start(); feedback.enabled = 1U; advance(4000ULL, 1);
    feedback.now_us += 4000ULL; feedback.feedback_us = feedback.now_us;
    feedback.prev_sent_valid = 1U; feedback.prev_sent_nm = 0.0017F;
    feedback.velocity_rad_s = 0.301F;
    (void)adrc_experiment_tick(&experiment, &feedback, &action);
    expect(fabsf(experiment.trace[experiment.status.trace_count - 1U].torque_sent_previous_nm - 0.0017F)
        < 0.000001F, "fault-triggering row retains actual prior transmit receipt");
    feedback.enabled = 0U; feedback.velocity_rad_s = 0.0F; advance(4000ULL, 1);
    frozen_count = experiment.status.trace_count;
    expect(adrc_experiment_trace_get(&experiment, 0U, &record) == ADRC_RESULT_OK,
        "disabled fault log is readable after freezing");
    for (index = 0U; index < 4U; index++) { advance(4000ULL, 1); }
    expect(experiment.status.trace_count == frozen_count, "frozen fault log does not change on later ticks");
    expect(adrc_experiment_get_status(&experiment, &status) == ADRC_RESULT_OK &&
        status.disabled_confirmed && status.trace_frozen, "status exposes final disable and freeze evidence");
    fixture(); start();
    feedback.enabled = 1U; advance(4000ULL, 1);
    expect(adrc_experiment_heartbeat(&experiment, feedback.now_us - 1ULL) == ADRC_RESULT_INVALID_ARGUMENT,
        "backward heartbeat is rejected");
    expect(adrc_experiment_stop(&experiment, feedback.now_us) == ADRC_RESULT_OK, "bounded stop starts");
    run_started = experiment.stop_started_us;
    expect(adrc_experiment_stop(&experiment, feedback.now_us + 1000ULL) == ADRC_RESULT_OK &&
        experiment.stop_started_us == run_started, "repeated STOP cannot extend deadline");
    fixture();
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_OK, "prepare near-zero rejection fixture");
    expect(adrc_experiment_prepare(&experiment, &feedback) == ADRC_RESULT_OK, "prepare before drift");
    feedback.velocity_rad_s = configuration.near_zero_rad_s + 0.0001F;
    expect(adrc_experiment_run(&experiment, &feedback, &action) == ADRC_RESULT_UNSAFE_FEEDBACK &&
        !action.enable_requested, "RUN rechecks near-zero after preparation");
    fixture(); configuration.torque_slew_nm_s = INFINITY;
    expect(adrc_experiment_configure(&experiment, &configuration, &qualification, &feedback)
        == ADRC_RESULT_INVALID_ARGUMENT, "infinite parameter rejected");
}
/** @brief Rejects pre-request disable evidence after a clock fault and accepts later genuine feedback. */
static void test_clock_fault_disable_watermark(void)
{
    uint64_t old_disabled_timestamp;
    fixture(); start();
    old_disabled_timestamp = feedback.feedback_us;
    feedback.now_us = 1ULL; feedback.feedback_us = 1ULL; feedback.enabled = 1U;
    (void)adrc_experiment_tick(&experiment, &feedback, &action);
    expect(experiment.status.fault == ADRC_FAULT_TIME && action.disable_requested,
        "clock rollback faults and requests disable");
    feedback.now_us = old_disabled_timestamp + 4000ULL;
    feedback.feedback_us = old_disabled_timestamp; feedback.enabled = 0U;
    (void)adrc_experiment_tick(&experiment, &feedback, &action);
    expect(!experiment.status.disabled_confirmed && !experiment.status.trace_frozen,
        "pre-RUN disabled sample cannot confirm disable after clock rollback");
    expect(adrc_experiment_clear(&experiment, &feedback) == ADRC_RESULT_UNSAFE_FEEDBACK,
        "pre-RUN disabled sample cannot clear clock fault");
    advance(4000ULL, 1);
    expect(experiment.status.state == ADRC_STATE_FAULT && experiment.status.disabled_confirmed &&
        experiment.status.trace_frozen, "new disabled feedback confirms without clearing clock fault");
    expect(adrc_experiment_clear(&experiment, &feedback) == ADRC_RESULT_OK,
        "new disabled feedback allows explicit clock fault clear");
}

/** @brief Classifies stop completion using the genuine feedback time at and beyond the deadline. */
static void test_stop_confirmation_deadline(void)
{
    unsigned scenario;
    unsigned step;
    for (scenario = 0U; scenario < 3U; scenario++)
    {
        fixture(); start(); feedback.enabled = 1U; advance(4000ULL, 1);
        (void)adrc_experiment_stop(&experiment, feedback.now_us);
        for (step = 0U; step < 124U; step++) { advance(4000ULL, 1); }
        expect(experiment.status.state == ADRC_STATE_STOPPING &&
            experiment.status.disable_requested && !experiment.status.disabled_confirmed,
            "stop remains pending without disabled feedback at 496ms");
        feedback.enabled = 0U;
        feedback.prev_sent_valid = action.send_torque; feedback.prev_sent_nm = action.torque_nm;
        feedback.now_us += scenario == 0U ? 4000ULL : 6000ULL;
        feedback.feedback_us = feedback.now_us - (scenario == 2U ? 2000ULL : 0ULL);
        (void)adrc_experiment_tick(&experiment, &feedback, &action);
        if (scenario == 1U)
        {
            expect(experiment.status.state == ADRC_STATE_FAULT &&
                experiment.status.fault == ADRC_FAULT_STOP_TIMEOUT,
                "first disabled feedback at 502ms preserves stop timeout");
            expect(action.disable_requested && action.send_torque && action.torque_nm == 0.0F,
                "late stop confirmation retains zero and disable requests");
        }
        else
        {
            expect(experiment.status.state == ADRC_STATE_DISABLED &&
                experiment.status.fault == ADRC_FAULT_NONE,
                "disabled feedback at 500ms completes within deadline even if read at 502ms");
        }
        expect(experiment.status.disabled_confirmed && experiment.status.trace_frozen,
            "stop outcome retains genuine disabled evidence and frozen trace");
    }
}
#endif

/** @brief Runs contract tests and returns nonzero for any observable safety regression. */
int main(void)
{
    test_boot_rejects_run();
#ifndef ADRC_TEST_RED_BASELINE
    test_configuration_gates(); test_controller_contract(); test_immediate_faults();
    test_stop_and_disable_evidence(); test_deadlines_and_trace();
    test_additional_safety_boundaries();
    test_clock_fault_disable_watermark(); test_stop_confirmation_deadline();
#endif
    printf("ADRC supervisor: %u checks, %u failures; record=%u bytes\n",
        checks, failures, (unsigned)sizeof(AdrcExperimentTrace));
    return failures ? 1 : 0;
}
