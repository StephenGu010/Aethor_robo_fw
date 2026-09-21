/** @file adrc_app_test_main.c
 * @brief Offline ADRC facade tests; synthetic feedback and evidence never enter firmware profiles.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../App/aethor_app.c"

#if defined(ADRC_APP_HOST_CONTROLLER_STUB) && ADRC_APP_HOST_CONTROLLER_STUB
/** @brief Host-only generated callback stub refuses output; this is not generated control code. */
int adrc_generated_controller_step(void *context, const AdrcControllerInput *input,
    AdrcControllerOutput *output)
{
    (void)context; (void)input; (void)output;
    return 0;
}
#endif

static unsigned int lock_depth;
static unsigned int lock_count;

/** @brief Asserts that facade calls never nest task critical boundaries. */
static void enter_critical(void)
{
    assert(lock_depth == 0U);
    ++lock_depth; ++lock_count;
}

/** @brief Asserts that every task critical boundary is paired. */
static void exit_critical(void)
{
    assert(lock_depth == 1U);
    --lock_depth;
}

/** @brief Injects explicit offline motor discovery and fresh feedback into the receive-side runtime. */
static void fixture_feedback(uint8_t axis, uint64_t timestamp_us, uint8_t enabled)
{
    MotorJointFeedback feedback;
    MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[axis];
    const JointConfig *joint = &arm_config_get_production()->joints[axis];
    memset(discovery, 0, sizeof(*discovery));
    discovery->verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    discovery->observed_control_mode = 1U;
    discovery->observed_esc_id = joint->esc_id;
    discovery->observed_master_id = joint->master_id;
    discovery->ranges.position_max_rad = 2.0F;
    discovery->ranges.velocity_max_rad_s = 1.0F;
    discovery->ranges.torque_max_nm = 1.0F;
    application_motor_runtime.discovery.verified_joint_mask |= (uint8_t)(1U << axis);
    memset(&feedback, 0, sizeof(feedback));
    feedback.timestamp_us = timestamp_us;
    feedback.driver_state = enabled ? S3519_DRIVER_STATE_ENABLED : S3519_DRIVER_STATE_DISABLED;
    feedback.mos_temperature_c = 25.0F; feedback.rotor_temperature_c = 25.0F;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank, joint->master_id,
        joint->esc_id, &feedback) == MOTOR_BANK_STATUS_OK);
}

/** @brief Sends one literal through the real facade and checks shared response status. */
static void send_request(const char *line, uint64_t timestamp_us, ProtocolEngineStatus expected)
{
    ProtocolOutputBatch output;
    assert(aethor_app_process_protocol_line(line, strlen(line), timestamp_us, &output) == expected);
}

/** @brief Exercises selected-axis prepare, actual bus receipt decoding, stale-slot failure and LCD stop. */
static void test_selected_axis_and_receipts(void)
{
    ProtocolOutputBatch output;
    AdrcExperimentConfig config;
    AdrcExperimentQualification evidence;
    AdrcExperimentStatus status;
    DebugUiRequest stop;
    CanFrame frame;
    CanTxPriority priority;
    uint8_t kind;
    float decoded;
    uint16_t torque_code;
    aethor_app_init(1000U, 123U);
    lock_count = 0U; lock_depth = 0U;
    aethor_app_set_task_critical_hooks(enter_critical, exit_critical);
    fixture_feedback(0U, 4000U, 0U);
    (void)aethor_app_service(4000U);
    send_request("1 adrc prepare motor=3", 4001U, PROTOCOL_ENGINE_STATUS_OK);
    fixture_feedback(0U, 8000U, 0U);
    (void)aethor_app_service(8000U);
    assert(aethor_app_pop_protocol_result_output(&output) == 1U);
    assert(application_adrc_bridge.bench.draft_config.axis_index == 2U);
    fixture_feedback(2U, 12000U, 0U);
    (void)aethor_app_service(12000U);
    config = application_adrc_bridge.bench.draft_config;
    memset(&evidence, 0, sizeof(evidence));
    evidence.axis_index = 2U; evidence.provenance = ADRC_ENV_OFFLINE_FIXTURE;
    evidence.verified_flags = ADRC_QUAL_IDENTIFY_REQUIRED;
    evidence.position_max_rad = 2.0F; evidence.velocity_max_rad_s = 1.0F; evidence.torque_max_nm = 1.0F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 12000U) == ADRC_RESULT_UNQUALIFIED);
    /* Synthetic hardware-provenance objects exist only in this host fixture to test the local hook. */
    evidence.provenance = ADRC_ENV_HARDWARE;
    evidence.position_max_rad = 2.01F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 12000U) == ADRC_RESULT_UNQUALIFIED);
    evidence.position_max_rad = 2.0F;
    evidence.velocity_max_rad_s = 1.01F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 12000U) == ADRC_RESULT_UNQUALIFIED);
    evidence.velocity_max_rad_s = 1.0F;
    evidence.torque_max_nm = 1.01F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 12000U) == ADRC_RESULT_UNQUALIFIED);
    evidence.torque_max_nm = 1.0F;
    application_adrc_bridge.bench.mapping[0] = 0.0F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 12000U) == ADRC_RESULT_UNQUALIFIED);
    application_adrc_bridge.bench.mapping[0] = 1.0F;
    config.velocity_quantum_rad_s = 0.0001F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 12000U) == ADRC_RESULT_UNQUALIFIED);
    config.velocity_quantum_rad_s = 0.001F;
    application_motor_runtime.discovery.results[2].ranges.torque_max_nm = 10.0F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 12000U) == ADRC_RESULT_UNQUALIFIED);
    application_motor_runtime.discovery.results[2].ranges.torque_max_nm = 1.0F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 12000U) == ADRC_RESULT_OK);
    send_request("2 adrc prepare motor=3", 12001U, PROTOCOL_ENGINE_STATUS_OK);
    fixture_feedback(2U, 16000U, 0U);
    (void)aethor_app_service(16000U);
    assert(aethor_app_pop_protocol_result_output(&output) == 1U);
    send_request("3 adrc run", 16001U, PROTOCOL_ENGINE_STATUS_OK);
    fixture_feedback(2U, 20000U, 0U);
    (void)aethor_app_service(20000U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 0U);
    assert(frame.identifier == arm_config_get_production()->joints[2].esc_id);
    assert(aethor_app_next_can_frame(20000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_WAITING);
    aethor_app_adrc_report_transmit(kind, decoded, 1U);
    fixture_feedback(2U, 24000U, 1U);
    (void)aethor_app_service(24000U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 1U);
    assert(frame.identifier == arm_config_get_production()->joints[2].esc_id);
    assert((frame.data[3] & 0x0FU) == 0U && frame.data[4] == 0U);
    assert(frame.data[5] == 0U && (frame.data[6] & 0xF0U) == 0U);
    torque_code = (uint16_t)(((uint16_t)(frame.data[6] & 0x0FU) << 8U) | frame.data[7]);
    assert(decoded == (float)torque_code * 2.0F / 4095.0F - 1.0F);
    assert(decoded <= config.torque_slew_nm_s * 0.004F && decoded >= -config.torque_slew_nm_s * 0.004F);
    aethor_app_adrc_report_transmit(kind, decoded, 1U);
    fixture_feedback(2U, 28000U, 1U);
    (void)aethor_app_service(28000U);
    assert(application_adrc_bridge.bench.experiment.previous_sent_nm == decoded);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 1U);
    /* Deliberately omit the hardware receipt; the next tick must not replay this torque. */
    fixture_feedback(2U, 32000U, 1U);
    (void)aethor_app_service(32000U);
    assert(aethor_app_adrc_get_status(&status) == 1U);
    assert(status.state == ADRC_STATE_FAULT && status.fault == ADRC_FAULT_SEND);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 2U);
    aethor_app_adrc_report_transmit(kind, decoded, 1U);
    assert(status.disabled_confirmed == 0U);
    fixture_feedback(2U, 36000U, 0U);
    (void)aethor_app_service(36000U);
    assert(aethor_app_adrc_get_status(&status) == 1U && status.disabled_confirmed == 1U);
    memset(&stop, 0, sizeof(stop)); stop.operation = DEBUG_UI_OPERATION_STOP;
    assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_STOP_LATCHED);
    fixture_feedback(2U, 40000U, 0U);
    (void)aethor_app_service(40000U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 2U);
    assert(frame.identifier == arm_config_get_production()->joints[2].esc_id);
    assert(aethor_app_pop_emergency_can_frame(&frame) == 0U);
    assert(lock_count > 15U && lock_depth == 0U);
    aethor_app_set_task_critical_hooks(NULL, NULL);
}

/** @brief Tests actual negative-grid amplitude/slew bounds and no-code fail-closed behavior. */
static void test_negative_grid_and_coarse_resolution(void)
{
    ProtocolOutputBatch output;
    AdrcExperimentConfig config;
    AdrcExperimentQualification evidence;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    float previous;
    AdrcExperimentStatus status;
    aethor_app_init(1000U, 456U);
    fixture_feedback(0U, 4000U, 0U);
    (void)aethor_app_service(4000U);
    send_request("1 adrc config group=mapping pos_scale=0.5 vel_scale=0.25 torque_scale=2", 4001U, PROTOCOL_ENGINE_STATUS_OK);
    fixture_feedback(0U, 8000U, 0U);
    (void)aethor_app_service(8000U);
    assert(aethor_app_pop_protocol_result_output(&output) == 1U);
    config = application_adrc_bridge.bench.draft_config;
    config.identify_torque_nm = -0.0019F;
    config.torque_limit_nm = 0.0019F;
    memset(&evidence, 0, sizeof(evidence));
    evidence.provenance = ADRC_ENV_HARDWARE;
    evidence.verified_flags = ADRC_QUAL_IDENTIFY_REQUIRED;
    evidence.position_max_rad = 1.0F; evidence.velocity_max_rad_s = 0.25F; evidence.torque_max_nm = 1.0F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 8000U) == ADRC_RESULT_OK);
    send_request("2 adrc prepare", 8001U, PROTOCOL_ENGINE_STATUS_OK);
    fixture_feedback(0U, 12000U, 0U);
    (void)aethor_app_service(12000U);
    assert(aethor_app_pop_protocol_result_output(&output) == 1U);
    send_request("3 adrc run", 12001U, PROTOCOL_ENGINE_STATUS_OK);
    fixture_feedback(0U, 16000U, 0U);
    application_motor_runtime.bank.motors[0].feedback.position_rad = 0.2F;
    application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = 0.004F;
    (void)aethor_app_service(16000U);
    assert(application_adrc_bridge.bench.experiment.initial_position_rad == 0.1F);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 0U);
    aethor_app_adrc_report_transmit(kind, decoded, 1U);
    fixture_feedback(0U, 20000U, 1U);
    application_motor_runtime.bank.motors[0].feedback.position_rad = 0.2F;
    application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = 0.004F;
    (void)aethor_app_service(20000U);
    assert(application_adrc_bridge.bench.experiment.trace[0].position_rad == 0.1F);
    assert(application_adrc_bridge.bench.experiment.trace[0].velocity_rad_s == 0.001F);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 1U);
    assert(decoded < 0.0F && decoded >= -config.torque_limit_nm);
    assert(fabsf(decoded) <= config.torque_slew_nm_s * 0.004F);
    previous = decoded;
    aethor_app_adrc_report_transmit(kind, decoded, 1U);
    fixture_feedback(0U, 24000U, 1U);
    (void)aethor_app_service(24000U);
    assert(aethor_app_adrc_get_status(&status) == 1U);
    assert(status.state == ADRC_STATE_RUNNING && status.fault == ADRC_FAULT_NONE);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 1U);
    assert(decoded >= -config.torque_limit_nm && fabsf(decoded - previous) <= config.torque_slew_nm_s * 0.004F);
    aethor_app_adrc_report_transmit(kind, decoded, 1U);
    /* Deliberate host-only corruption models a now-unencodable range; it must disable, not widen caps. */
    fixture_feedback(0U, 28000U, 1U);
    application_motor_runtime.discovery.results[0].ranges.torque_max_nm = 100.0F;
    (void)aethor_app_service(28000U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 2U);
    assert(application_adrc_bridge.send_failed == 1U);
}

/** @brief Proves each finite identification deadline emits disable instead of a residual MIT torque ramp. */
static void test_identify_hard_deadlines(void)
{
    static const uint32_t pulse_durations_us[] = { 20000U, 40000U, 80000U, 80000U };
    unsigned int scenario_index;
    for (scenario_index = 0U; scenario_index < 4U; ++scenario_index)
    {
        AdrcExperimentConfig config;
        AdrcExperimentQualification evidence;
        AdrcExperimentStatus status;
        ProtocolOutputBatch result_output;
        DebugUiRequest stop;
        CanFrame frame;
        uint8_t kind;
        float decoded;
        uint32_t elapsed_us;
        uint32_t cutoff_us = scenario_index == 3U ? 8000U : pulse_durations_us[scenario_index];
        const uint64_t run_started_us = 12000ULL;
        aethor_app_init(1000U, 900U + scenario_index);
        fixture_feedback(0U, 4000U, 0U);
        (void)aethor_app_service(4000U);
        config = application_adrc_bridge.bench.draft_config;
        config.identify_pulse_us = pulse_durations_us[scenario_index];
        config.identify_torque_nm = scenario_index == 1U ? -0.01F : 0.01F;
        memset(&evidence, 0, sizeof(evidence));
        evidence.provenance = ADRC_ENV_HARDWARE;
        evidence.verified_flags = ADRC_QUAL_IDENTIFY_REQUIRED;
        evidence.position_max_rad = 2.0F;
        evidence.velocity_max_rad_s = 1.0F;
        evidence.torque_max_nm = 1.0F;
        assert(aethor_app_adrc_set_evidence(&config, &evidence, 4000U) == ADRC_RESULT_OK);
        send_request("1 adrc prepare", 4001U, PROTOCOL_ENGINE_STATUS_OK);
        fixture_feedback(0U, 8000U, 0U);
        (void)aethor_app_service(8000U);
        assert(aethor_app_pop_protocol_result_output(&result_output) == 1U);
        send_request("2 adrc run", 8001U, PROTOCOL_ENGINE_STATUS_OK);
        fixture_feedback(0U, run_started_us, 0U);
        (void)aethor_app_service(run_started_us);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 0U);
        aethor_app_adrc_report_transmit(kind, decoded, 1U);
        for (elapsed_us = 4000U; elapsed_us <= cutoff_us; elapsed_us += 4000U)
        {
            if (scenario_index == 3U && elapsed_us == cutoff_us)
            {
                memset(&stop, 0, sizeof(stop));
                stop.operation = DEBUG_UI_OPERATION_STOP;
                assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_STOP_LATCHED);
            }
            fixture_feedback(0U, run_started_us + elapsed_us, 1U);
            (void)aethor_app_service(run_started_us + elapsed_us);
            assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
            assert(kind == (elapsed_us < cutoff_us ? 1U : 2U));
            if (elapsed_us == cutoff_us)
            {
                assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE);
                assert(aethor_app_adrc_get_status(&status) == 1U);
                assert(status.state == ADRC_STATE_STOPPING && status.fault == ADRC_FAULT_NONE);
                assert(status.disable_requested == 1U && status.disabled_confirmed == 0U);
            }
            aethor_app_adrc_report_transmit(kind, decoded, 1U);
        }
        assert(aethor_app_pop_protocol_result_output(&result_output) == 0U);
        /* A successful CAN disable receipt alone is not proof that the drive became disabled. */
        fixture_feedback(0U, run_started_us + cutoff_us + 4000U, 1U);
        (void)aethor_app_service(run_started_us + cutoff_us + 4000U);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U && kind == 2U);
        aethor_app_adrc_report_transmit(kind, decoded, 1U);
        assert(aethor_app_adrc_get_status(&status) == 1U && status.disabled_confirmed == 0U);
        assert(aethor_app_pop_protocol_result_output(&result_output) == 0U);
        fixture_feedback(0U, run_started_us + cutoff_us + 8000U, 0U);
        (void)aethor_app_service(run_started_us + cutoff_us + 8000U);
        assert(aethor_app_adrc_get_status(&status) == 1U);
        assert(status.state == ADRC_STATE_DISABLED && status.disabled_confirmed == 1U);
        if (aethor_app_adrc_pop_frame(&frame, &kind, &decoded)) { assert(kind == 2U); }
        assert(aethor_app_pop_protocol_result_output(&result_output) == 1U);
        assert(strstr(result_output.messages[0].data, "done 2 adrc result=0 disabled=1") != NULL);
    }
}

/** @brief Requires an idle USB STOP to emit one selected-axis disable without motion authority. */
static void test_idle_protocol_stop_emits_disable(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;

    aethor_app_init(1000U, 456U);
    assert(aethor_app_process_protocol_line("1 adrc stop", strlen("1 adrc stop"),
        2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "ok 1 adrc accepted=1") != NULL);
    (void)aethor_app_service(4000U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
    assert(kind == 2U);
    assert(frame.identifier == arm_config_get_production()->joints[0].esc_id);
    assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(decoded == 0.0F);
}

/** @brief Checks idle discovery is read-only and actual received disable feedback remains available to LCD. */
static void test_readonly_discovery_and_receive(void)
{
    CanFrame frame;
    CanTxPriority priority;
    MotorFeedbackSnapshot snapshot;
    aethor_app_init(1000U, 789U);
    assert(aethor_app_next_can_frame(1000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == S3519_PARAMETER_COMMAND_IDENTIFIER && frame.data[2] == 0x33U);
    fixture_feedback(0U, 2000U, 0U);
    memset(&frame, 0, sizeof(frame));
    frame.identifier = arm_config_get_production()->joints[0].master_id;
    frame.length = 8U;
    frame.data[0] = (uint8_t)arm_config_get_production()->joints[0].esc_id;
    frame.data[1] = 0x80U; frame.data[2] = 0U;
    frame.data[3] = 0x80U; frame.data[4] = 0x08U; frame.data[5] = 0U;
    frame.data[6] = 25U; frame.data[7] = 25U;
    assert(aethor_app_receive_can_frame(&frame, 4000U) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_get_motor_snapshot(4000U, &snapshot));
    assert(snapshot.joints[0].driver_state == S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(4000U);
    assert(application_adrc_bridge.bench.gateway.snapshot.status.disabled_confirmed == 1U);
    (void)aethor_app_service(16001U);
    assert(application_adrc_bridge.bench.gateway.snapshot.status.disabled_confirmed == 0U);
}

/** @brief Ensures opt-in ADRC builds reject legacy actuation and lack startup qualification. */
int main(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    DebugUiRequest stop;
    AdrcExperimentStatus status;
    aethor_app_init(1000U, 123U);
    assert(aethor_app_process_protocol_line("1 adrc run", 10U, 1000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "not_ready") != NULL);
    assert(aethor_app_process_protocol_line("2 bench enable", 14U, 1000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "adrc_only") != NULL);
    assert(application_protocol_engine.command_write_sequence == 0U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
    memset(&stop, 0, sizeof(stop));
    stop.operation = DEBUG_UI_OPERATION_STOP;
    assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_STOP_LATCHED);
    (void)aethor_app_service(4000U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
    assert(kind == 2U);
    assert(frame.identifier == arm_config_get_production()->joints[0].esc_id);
    assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    aethor_app_adrc_report_transmit(kind, decoded, 1U);
    assert(aethor_app_adrc_get_status(&status) == 1U);
    assert(status.disabled_confirmed == 0U);
    test_selected_axis_and_receipts();
    test_negative_grid_and_coarse_resolution();
    test_identify_hard_deadlines();
    test_idle_protocol_stop_emits_disable();
    test_readonly_discovery_and_receive();
    puts("ADRC_APP_TESTS_PASSED");
    return 0;
}
