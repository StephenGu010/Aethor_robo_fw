/** @file adrc_bench_test_main.c
 * @brief Offline integration contracts for trusted evidence, bounded mailboxes and real disable completion.
 */
#include <stdio.h>
#include <string.h>
#include "adrc_bench.h"

static AdrcBench bench;
static ProtocolEngine engine;
static ProtocolOutputBatch messages;
static AdrcExperimentInput feedback;
static AdrcExperimentOutput action;
static unsigned checks;
static unsigned failures;

/** @brief Records independent behavioral assertions and preserves later diagnostics. */
static void expect(int condition, const char *description)
{
    checks++;
    if (!condition) { failures++; printf("FAIL: %s\n", description); }
}

/** @brief Supplies finite offline controller output without asserting generated algorithm provenance. */
static int controller(void *context, const AdrcControllerInput *input, AdrcControllerOutput *output)
{
    (void)context;
    output->torque_raw_nm = input->reference_rad_s > 0.0F ? 0.01F : 0.0F;
    output->z1 = input->velocity_rad_s; output->z2 = 0.0F;
    return 1;
}

/** @brief Initializes an entirely offline bridge with fresh disabled selected-axis feedback. */
static void fixture(void)
{
    memset(&feedback, 0, sizeof(feedback)); memset(&action, 0, sizeof(action));
    expect(adrc_bench_init(&bench, ADRC_ENV_OFFLINE_FIXTURE, controller, NULL) == ADRC_RESULT_OK,
        "initialize offline bench");
    protocol_engine_init(&engine, 1U);
    protocol_engine_set_adrc_handler(&engine, adrc_protocol_handle_request, adrc_bench_gateway(&bench));
    feedback.now_us = 1000000ULL; feedback.feedback_us = feedback.now_us;
    feedback.feedback_valid = 1U;
    feedback.mos_temperature_c = 25.0F; feedback.rotor_temperature_c = 25.0F;
    expect(adrc_bench_service(&bench, &feedback, &action) == ADRC_RESULT_OK,
        "publish initial coherent disabled snapshot");
}

/** @brief Runs one four-millisecond owner-task cycle with truthful previous-send evidence. */
static void advance(void)
{
    feedback.prev_sent_valid = action.send_torque; feedback.prev_sent_nm = action.torque_nm;
    feedback.now_us += 4000ULL; feedback.feedback_us = feedback.now_us;
    expect(adrc_bench_service(&bench, &feedback, &action) == ADRC_RESULT_OK, "service control cycle");
}

/** @brief Sends a literal through the production shared replay and ADRC parser. */
static ProtocolEngineStatus request(const char *line)
{
    return protocol_engine_process_text_line(&engine, line, strlen(line), feedback.now_us, &messages);
}

/** @brief Consumes one retained terminal through the real protocol engine cache. */
static void terminal(const char *expected)
{
    expect(adrc_protocol_pop_result_output(&bench.gateway, &engine, &messages) != 0U,
        "terminal remains available for protocol publication");
    expect(messages.count != 0U && strstr(messages.messages[0].data, expected) != NULL,
        "terminal identifies expected request and result");
}

/** @brief Installs explicit offline measurements through the trusted local-only API. */
static void evidence(void)
{
    AdrcExperimentConfig config = bench.draft_config;
    AdrcExperimentQualification qualification;
    memset(&qualification, 0, sizeof(qualification));
    config.mode = ADRC_MODE_LADRC; config.b0 = 1.0F;
    qualification.verified_flags = ADRC_QUAL_CLOSED_LOOP_REQUIRED;
    qualification.provenance = ADRC_ENV_OFFLINE_FIXTURE;
    qualification.position_max_rad = 1.0F; qualification.velocity_max_rad_s = 1.0F;
    qualification.torque_max_nm = 1.0F; qualification.axis_index = config.axis_index;
    expect(adrc_bench_set_evidence(&bench, &config, &qualification, &feedback) == ADRC_RESULT_OK,
        "local offline evidence qualifies its own environment only");
}

/** @brief Prepares a locally qualified selected axis and publishes the completed preparation. */
static void prepare(void)
{
    evidence();
    expect(request("1 adrc prepare motor=1") == PROTOCOL_ENGINE_STATUS_OK, "prepare accepted");
    advance(); terminal("done 1 adrc result=0");
    expect(bench.gateway.snapshot.qualified && bench.gateway.snapshot.status.state == ADRC_STATE_PREPARED,
        "prepared snapshot is derived from qualified supervisor");
}

/** @brief Verifies boot defaults, untrusted configuration and environment separation never unlock RUN. */
static void test_default_and_provenance(void)
{
    AdrcExperimentConfig config;
    AdrcExperimentQualification qualification;
    fixture();
    expect(bench.mapping[0] == 1.0F && bench.mapping[1] == 1.0F && bench.mapping[2] == 1.0F,
        "default draft coordinate scales are unity");
    expect(bench.trusted_qualification.verified_flags == 0U && !bench.gateway.snapshot.qualified,
        "default bridge has no fabricated qualification");
    expect(request("1 adrc run") == PROTOCOL_ENGINE_STATUS_BAD_REQUEST, "default RUN rejected");
    expect(request("2 adrc prepare motor=1") == PROTOCOL_ENGINE_STATUS_OK, "unqualified prepare admitted for execution");
    advance(); terminal("done 2 adrc result=3");
    expect(!action.enable_requested && !action.send_torque, "unqualified service cannot drive");
    config = bench.draft_config; memset(&qualification, 0, sizeof(qualification));
    qualification.provenance = ADRC_ENV_OFFLINE_FIXTURE; qualification.verified_flags = 255U;
    qualification.position_max_rad = 1.0F; qualification.velocity_max_rad_s = 1.0F;
    qualification.torque_max_nm = 1.0F;
    expect(adrc_bench_init(&bench, ADRC_ENV_HARDWARE, controller, NULL) == ADRC_RESULT_OK,
        "initialize real-hardware environment");
    expect(adrc_bench_set_evidence(&bench, &config, &qualification, &feedback) == ADRC_RESULT_UNQUALIFIED,
        "offline fixture evidence never qualifies hardware bridge");
}

/** @brief Completes a real state-machine run only after observed disable and exposes frozen trace. */
static void test_run_stop_and_trace(void)
{
    unsigned step;
    fixture(); prepare();
    expect(request("2 adrc run") == PROTOCOL_ENGINE_STATUS_OK, "qualified run accepted");
    advance(); expect(action.enable_requested && bench.run_request_id == 2U, "run requests selected-axis enable once");
    expect(!adrc_protocol_pop_result_output(&bench.gateway, &engine, &messages), "run has no premature terminal");
    feedback.enabled = 1U; advance();
    expect(request("adrc trace index=0") == PROTOCOL_ENGINE_STATUS_BAD_REQUEST, "running trace cannot export");
    expect(request("3 adrc stop") == PROTOCOL_ENGINE_STATUS_OK, "STOP uses independent channel during run");
    advance(); terminal("done 3 adrc result=0");
    for (step = 0U; step < 26U; step++) { advance(); }
    expect(action.disable_requested && !bench.experiment.status.disabled_confirmed,
        "disable request alone cannot complete run");
    expect(!adrc_protocol_pop_result_output(&bench.gateway, &engine, &messages), "run terminal waits for actual disable");
    feedback.enabled = 0U; advance(); terminal("done 2 adrc result=0");
    expect(request("adrc trace index=0") == PROTOCOL_ENGINE_STATUS_OK &&
        strstr(messages.messages[0].data, "adrc trace") != NULL, "frozen disabled trace reader is wired");
}

/** @brief Exercises draft field masks, preparation cancellation, model revocation and axis isolation. */
static void test_configuration_invalidation(void)
{
    fixture(); prepare();
    expect(request("2 adrc config group=mapping vel_scale=2") == PROTOCOL_ENGINE_STATUS_OK,
        "mapping patch reaches owner for state validation");
    advance(); terminal("done 2 adrc result=2");
    expect(bench.mapping[1] == 1.0F && bench.experiment.status.state == ADRC_STATE_PREPARED,
        "mapping cannot change while prepared");
    expect(request("3 adrc config group=control target=0.15 wc=4") == PROTOCOL_ENGINE_STATUS_OK, "control patch admitted");
    advance(); terminal("done 3 adrc result=0");
    expect(bench.experiment.status.state == ADRC_STATE_DISABLED && bench.draft_config.reference_rad_s == 0.15F &&
        bench.draft_config.wc_rad_s == 4.0F && bench.draft_config.wo_rad_s == 20.0F &&
        bench.trusted_qualification.verified_flags == 255U, "control patch cancels prepare without clearing identity");
    expect(request("4 adrc config group=control b0=2") == PROTOCOL_ENGINE_STATUS_OK, "b0 patch admitted");
    advance(); terminal("done 4 adrc result=0");
    expect((bench.trusted_qualification.verified_flags & ADRC_QUAL_IDENTIFIED_MODEL) == 0U &&
        (bench.trusted_qualification.verified_flags & ADRC_QUAL_IDENTITY) != 0U,
        "USB b0 edit revokes identified model evidence only");
    expect(request("5 adrc config group=identify torque=-0.02 pulse_ms=40") == PROTOCOL_ENGINE_STATUS_OK,
        "identification patch admitted");
    advance(); terminal("done 5 adrc result=0");
    expect(bench.draft_config.identify_torque_nm == -0.02F && bench.draft_config.identify_pulse_us == 40000U,
        "identification field mapping converts milliseconds");
    expect(request("6 adrc config group=mapping vel_scale=2") == PROTOCOL_ENGINE_STATUS_OK, "disabled mapping patch admitted");
    advance(); terminal("done 6 adrc result=0");
    expect(bench.mapping[1] == 2.0F && bench.trusted_qualification.verified_flags == 0U,
        "mapping edit invalidates all measurement evidence");
    evidence();
    expect(request("7 adrc prepare motor=2") == PROTOCOL_ENGINE_STATUS_OK, "new selected axis admitted");
    advance(); terminal("done 7 adrc result=3");
    expect(bench.draft_config.axis_index == 1U && bench.trusted_qualification.verified_flags == 0U &&
        !bench.gateway.snapshot.qualified, "axis selection cannot inherit another motor's evidence");
}

/** @brief Preserves three retained terminals and prevents STOP-cancelled queued RUN from reexecution. */
static void test_priority_and_terminal_retention(void)
{
    ProtocolEngine empty_engine;
    fixture(); prepare();
    expect(request("2 adrc run") == PROTOCOL_ENGINE_STATUS_OK, "queue run before stop");
    expect(request("3 adrc heartbeat") == PROTOCOL_ENGINE_STATUS_OK, "queue independent heartbeat");
    expect(request("4 adrc stop") == PROTOCOL_ENGINE_STATUS_OK, "queue priority stop");
    advance();
    expect(!action.enable_requested && !bench.run_request_id && bench.experiment.status.state == ADRC_STATE_DISABLED,
        "STOP cancellation cannot restart queued RUN");
    expect(bench.gateway.channel_state[0] == 3U && bench.gateway.channel_state[1] == 3U &&
        bench.gateway.channel_state[2] == 3U, "all three terminal slots survive one bounded service cycle");
    protocol_engine_init(&empty_engine, 2U);
    expect(!adrc_protocol_pop_result_output(&bench.gateway, &empty_engine, &messages),
        "failed terminal publication retains completed channel");
    advance();
    expect(bench.gateway.channel_state[0] == 3U, "later service does not overwrite retained terminal");
    terminal("done 2 adrc result=2"); terminal("done 4 adrc result=0"); terminal("done 3 adrc result=0");
    expect(request("2 adrc run") == PROTOCOL_ENGINE_STATUS_REPLAYED &&
        strstr(messages.messages[0].data, "cancelled=1") != NULL, "cancelled run terminal remains replayable");
}

/** @brief Finite and faulted runs both retain their terminal until fresh feedback proves actual disable. */
static void test_finite_and_fault_terminals(void)
{
    unsigned scenario;
    unsigned step;
    for (scenario = 0U; scenario < 2U; scenario++)
    {
        fixture(); bench.draft_config.duration_us = 40000U; prepare();
        expect(request("2 adrc run") == PROTOCOL_ENGINE_STATUS_OK, "bounded run admitted");
        advance(); feedback.enabled = 1U; advance();
        if (scenario == 1U) { feedback.driver_fault = 1U; }
        for (step = 0U; step < 45U && !action.disable_requested; step++) { advance(); }
        expect(action.disable_requested && !bench.experiment.status.disabled_confirmed &&
            bench.run_request_id == 2U, "finite/fault disable request keeps run pending");
        expect(!adrc_protocol_pop_result_output(&bench.gateway, &engine, &messages),
            "no finite/fault DONE before actual disable");
        feedback.enabled = 0U; feedback.driver_fault = 0U; advance();
        terminal(scenario == 0U ? "done 2 adrc result=0" : "done 2 adrc result=4");
        expect(bench.experiment.status.disabled_confirmed && bench.experiment.status.trace_frozen,
            "finite/fault terminal includes fresh disable and frozen trace");
    }
}

/** @brief Allows first diagnostic axis selection without inventing disabled feedback or motion authority. */
static void test_initial_diagnostic_axis_selection(void)
{
    fixture(); feedback.feedback_valid = 0U;
    expect(request("1 adrc prepare motor=7") == PROTOCOL_ENGINE_STATUS_OK,
        "first diagnostic target can be requested without current-axis feedback");
    advance(); terminal("done 1 adrc result=3");
    expect(bench.draft_config.axis_index == 6U && bench.gateway.snapshot.status.axis_index == 6U,
        "unconfigured bridge selects connected diagnostic axis without default-axis feedback");
    expect(!action.enable_requested && !action.send_torque &&
        bench.trusted_qualification.verified_flags == 0U && !bench.gateway.snapshot.qualified &&
        !bench.experiment.configured && !bench.experiment.status.disabled_confirmed,
        "diagnostic selection grants neither configuration nor actuation nor disabled evidence");
    fixture(); feedback.enabled = 1U;
    expect(request("1 adrc prepare motor=7") == PROTOCOL_ENGINE_STATUS_OK,
        "initial selection with known enabled old axis reaches safety validation");
    advance(); terminal("done 1 adrc result=4");
    expect(bench.draft_config.axis_index == 0U,
        "known enabled old axis cannot use diagnostic selection exception");
    fixture(); prepare(); feedback.feedback_valid = 0U;
    expect(request("2 adrc prepare motor=7") == PROTOCOL_ENGINE_STATUS_OK,
        "configured axis-switch request reaches owner validation");
    advance(); terminal("done 2 adrc result=4");
    expect(bench.draft_config.axis_index == 0U && !action.enable_requested,
        "previously qualified axis cannot be abandoned without fresh old-axis disable evidence");
    fixture(); evidence();
    expect(request("1 adrc config group=mapping pos_scale=2") == PROTOCOL_ENGINE_STATUS_OK,
        "mapping revocation accepted after local qualification");
    advance(); terminal("done 1 adrc result=0"); feedback.feedback_valid = 0U;
    expect(request("2 adrc prepare motor=7") == PROTOCOL_ENGINE_STATUS_OK,
        "revoked configuration still requires safe old-axis exit");
    advance(); terminal("done 2 adrc result=4");
    expect(bench.draft_config.axis_index == 0U,
        "revoking evidence cannot recreate first-boot diagnostic bypass");
}

/** @brief Runs all integration contracts without platform, HAL, RTOS or hardware access. */
int main(void)
{
    test_default_and_provenance(); test_run_stop_and_trace(); test_configuration_invalidation();
    test_priority_and_terminal_retention();
    test_finite_and_fault_terminals();
    test_initial_diagnostic_axis_selection();
    printf("ADRC bench: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
