/** @file adrc_app_test_main.c
 * @brief Offline ADRC facade tests; synthetic feedback and evidence never enter firmware profiles.
 */
#include <assert.h>
#include <float.h>
#include <math.h>
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

/** @brief Injects nominal historical grids exclusively into a synthetic host fixture. */
static void candidate_feedback(uint64_t timestamp_us, uint8_t enabled)
{
    fixture_feedback(0U, timestamp_us, enabled);
    application_motor_runtime.discovery.results[0].ranges.position_max_rad = 12.5F;
    application_motor_runtime.discovery.results[0].ranges.velocity_max_rad_s = 200.0F;
    application_motor_runtime.discovery.results[0].ranges.torque_max_nm = 10.0F;
    application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = 200.0F / 4095.0F;
}

/** @brief Candidate proves passive warmup, explicit evidence, true codec slew and raw fault protection. */
static void test_candidate_interface(void)
{
    AdrcExperimentConfig config;
    AdrcExperimentQualification evidence;
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    unsigned sample;
    unsigned mutation;
    for (mutation = 0U; mutation < 7U; ++mutation)
    {
        aethor_app_init(1000U, 123U);
        memset(&evidence, 0, sizeof(evidence));
        evidence.verified_flags = 0xFFU;
        evidence.provenance = ADRC_ENV_HARDWARE; /* Synthetic host-only hook exercise. */
        evidence.position_max_rad = 12.5F;
        evidence.velocity_max_rad_s = 200.0F;
        evidence.torque_max_nm = 10.0F;
        config = application_adrc_bridge.bench.draft_config;
        config.interface_profile = ADRC_INTERFACE_POSITION32_QUANTIZED;
        config.torque_slew_nm_s = 1.25F;
        config.velocity_quantum_rad_s = 0.012F;
        config.near_zero_rad_s = 0.024F;
        candidate_feedback(4000U, 0U);
        (void)aethor_app_service(4000U);
        assert(aethor_app_adrc_set_evidence(&config, &evidence, 4000U) == ADRC_RESULT_UNQUALIFIED);
        evidence.verified_flags |= ADRC_QUAL_POSITION32_REQUIRED;
        assert(aethor_app_adrc_set_evidence(&config, &evidence, 4000U) == ADRC_RESULT_UNSAFE_FEEDBACK);
        for (sample = 2U; sample <= 9U; ++sample)
        { candidate_feedback(sample * 4000U, 0U); (void)aethor_app_service(sample * 4000U); }
        assert(aethor_app_adrc_set_evidence(&config, &evidence, 36000U) == ADRC_RESULT_OK);
        send_request("501 adrc prepare motor=1", 36001U, PROTOCOL_ENGINE_STATUS_OK);
        candidate_feedback(40000U, 0U); (void)aethor_app_service(40000U);
        assert(aethor_app_pop_protocol_result_output(&output));
        assert(application_adrc_bridge.bench.experiment.status.state == ADRC_STATE_PREPARED);
        if (mutation != 0U && mutation < 4U)
        {
            candidate_feedback(44000U, 0U);
            if (mutation == 1U) { application_motor_runtime.discovery.results[0].ranges.torque_max_nm = 9.0F; }
            if (mutation == 2U) { application_adrc_bridge.bench.mapping[0] = 0.5F; }
            if (mutation == 3U) { application_adrc_bridge.bench.draft_config.axis_index = 1U; }
            (void)aethor_app_service(44000U);
            assert(!application_adrc_bridge.bench.experiment.configured);
            assert(!application_adrc_bridge.bench.trusted_qualification.verified_flags);
            assert(application_adrc_bridge.bench.experiment.status.state != ADRC_STATE_PREPARED);
            send_request("502 adrc run", 44001U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
            continue;
        }
        send_request("502 adrc run", 40001U, PROTOCOL_ENGINE_STATUS_OK);
        candidate_feedback(44000U, 0U); (void)aethor_app_service(44000U);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) && kind == 0U);
        aethor_app_adrc_report_transmit(kind, decoded, 1U);
        candidate_feedback(48000U, 1U); (void)aethor_app_service(48000U);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) && kind == 1U);
        assert(decoded == (float)(((frame.data[6] & 15U) << 8U) | frame.data[7]) * 20.0F / 4095.0F - 10.0F);
        assert(fabsf(decoded) <= 0.005F && fabsf(decoded) <= 0.1F);
        aethor_app_adrc_report_transmit(kind, decoded, 1U);
        candidate_feedback(52000U, 1U);
        if (mutation == 0U) { application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = 0.31F; }
        if (mutation == 4U) { application_adrc_bridge.bench.draft_config.axis_index = 1U; }
        if (mutation == 5U) { application_adrc_bridge.bench.mapping[1] = 0.5F; }
        if (mutation == 6U) { application_motor_runtime.discovery.results[0].ranges.position_max_rad = 12.0F; }
        (void)aethor_app_service(52000U);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) && kind == 2U);
        assert(frame.identifier == arm_config_get_production()->joints[0].esc_id);
        assert(application_adrc_bridge.bench.experiment.status.fault ==
            (mutation == 0U ? ADRC_FAULT_SPEED : ADRC_FAULT_FEEDBACK));
    }
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
    assert(application_adrc_bridge.bench.experiment.status.fault == ADRC_FAULT_FEEDBACK);
    assert(application_adrc_bridge.bench.experiment.configured == 0U);
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

/** @brief Verifies completed discovery emits only the validated 0xCC feedback query at 4 ms intervals. */
static void test_readonly_feedback_query_pacing(void)
{
    CanFrame frame;
    CanTxPriority priority;
    const JointConfig *joint;

    aethor_app_init(1000U, 790U);
    joint = &arm_config_get_production()->joints[0];
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    application_motor_runtime.discovery.results[0].verified_fields_mask =
        MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    application_motor_runtime.discovery.verified_joint_mask = 0x01U;

    assert(aethor_app_next_can_frame(1000U, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_PARAMETER);
    assert(frame.identifier == S3519_PARAMETER_COMMAND_IDENTIFIER);
    assert(frame.length == 4U);
    assert(frame.data[0] == (uint8_t)joint->esc_id);
    assert(frame.data[1] == 0U && frame.data[2] == 0xCCU && frame.data[3] == 0U);
    assert(application_adrc_bridge.feedback_query_count == 1U);
    assert(aethor_app_next_can_frame(4999U, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_WAITING);
    assert(aethor_app_next_can_frame(5000U, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[2] == 0xCCU);
    assert(application_adrc_bridge.feedback_query_count == 2U);
}

/** @brief Verifies hardware diagnostics expose measured fields without creating an ADRC actuation frame. */
static void test_readonly_hardware_diagnostics(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    MotorDiscoveryResult *discovery;

    aethor_app_init(1000U, 791U);
    fixture_feedback(0U, 2000U, 0U);
    (void)aethor_app_service(2000U);
    fixture_feedback(0U, 6000U, 0U);
    (void)aethor_app_service(6000U);
    discovery = &application_motor_runtime.discovery.results[0];
    discovery->communication_timeout_raw = 100U;
    discovery->hardware_version = 0x00010002U;
    discovery->software_version = 0x00030004U;
    discovery->sub_version = 5U;
    discovery->acceleration_rad_s2 = 30.0F;
    discovery->deceleration_rad_s2 = 25.0F;
    discovery->maximum_speed_rad_s = 20.0F;
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;

    assert(aethor_app_process_protocol_line("1 adrc hardware", strlen("1 adrc hardware"),
        6001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "motor=1") != NULL);
    assert(strstr(output.messages[0].data, "fields=1fff") != NULL);
    assert(strstr(output.messages[0].data, "timeout_raw=100") != NULL);
    assert(strstr(output.messages[0].data, "hw=65538") != NULL);

    assert(aethor_app_process_protocol_line("2 adrc limits", strlen("2 adrc limits"),
        6001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "pmax=2") != NULL);
    assert(strstr(output.messages[0].data, "vmax=1") != NULL);
    assert(strstr(output.messages[0].data, "tmax=1") != NULL);
    assert(strstr(output.messages[0].data, "maxspd=20") != NULL);

    assert(aethor_app_process_protocol_line("3 adrc feedback", strlen("3 adrc feedback"),
        6001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "seen=1 fresh=1 state=0") != NULL);
    assert(strstr(output.messages[0].data, "samples=2 min_us=4000 max_us=4000") != NULL);
    assert(aethor_app_process_protocol_line("4 adrc evidence", strlen("4 adrc evidence"),
        6001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "observed=5") != NULL);
    assert(strstr(output.messages[0].data, "verified=0") != NULL);
    assert(strstr(output.messages[0].data, "stop_bound=0") != NULL);
    assert(application_adrc_bridge.bench.trusted_qualification.verified_flags == 0U);
    assert(application_protocol_engine.command_write_sequence == 0U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
}

/** @brief Deferring ADRC startup must preserve LCD discovery until exclusive acquisition. */
static void test_deferred_bridge_discovery(void)
{
    aethor_app_init(1000U, 792U);
    assert(motor_runtime_begin_discovery(&application_motor_runtime, 0x01U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.discovery.target_joint_mask == 0x01U);
    assert(adrc_app_bridge_init_deferred(&application_adrc_bridge,
        &application_motor_runtime, &application_protocol_engine,
        adrc_generated_controller_step, NULL) == ADRC_RESULT_OK);
    assert(application_motor_runtime.discovery.target_joint_mask == 0x01U);
    assert(adrc_app_bridge_start_selected_discovery(&application_adrc_bridge, 6U) ==
           MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.discovery.target_joint_mask == 0x40U);
}

/** @brief Characterizes measured motor-7 mapping gates without creating hardware evidence. */
static void test_motor7_mapping_quantization_characterization(void)
{
    AdrcExperimentConfig config;
    AdrcExperimentQualification evidence;
    MotorDiscoveryResult *discovery;

    aethor_app_init(1000U, 777U);
    fixture_feedback(0U, 4000U, 0U);
    discovery = &application_motor_runtime.discovery.results[0];
    discovery->ranges.position_max_rad = 12.5F;
    discovery->ranges.velocity_max_rad_s = 200.0F;
    discovery->ranges.torque_max_nm = 10.0F;
    config = application_adrc_bridge.bench.draft_config;
    config.velocity_quantum_rad_s = 0.0098F;
    config.near_zero_rad_s = 0.02F;
    memset(&evidence, 0, sizeof(evidence));
    evidence.axis_index = 0U;
    evidence.provenance = ADRC_ENV_HARDWARE;
    evidence.verified_flags = ADRC_QUAL_IDENTIFY_REQUIRED;
    evidence.position_max_rad = 12.5F;
    evidence.velocity_max_rad_s = 20.0F;
    evidence.torque_max_nm = 4.0F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 4000U) == ADRC_RESULT_UNQUALIFIED);
    discovery->ranges.velocity_max_rad_s = 20.0F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 4000U) == ADRC_RESULT_UNQUALIFIED);
    discovery->ranges.torque_max_nm = 4.0F;
    assert(aethor_app_adrc_set_evidence(&config, &evidence, 4000U) == ADRC_RESULT_OK);
}

/** @brief Disabled capture collects distinct causal samples without enabling or qualifying a motor. */
static void test_disabled_capture(void)
{
    CanFrame frame;
    uint8_t kind;
    float decoded;
    unsigned int sample;
    ProtocolOutputBatch output;
    aethor_app_init(1000U, 800U);
    candidate_feedback(4000U, 0U);
    send_request("100 adrc sample", 4000U, PROTOCOL_ENGINE_STATUS_OK);
    send_request("101 adrc sample", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    send_request("102 adrc prepare", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    (void)adrc_app_bridge_service(&application_adrc_bridge, 8000U);
    assert(adrc_app_bridge_pop_frame(&application_adrc_bridge, &frame, &kind, &decoded));
    assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    adrc_app_bridge_report_transmit(&application_adrc_bridge, kind, decoded, 1U);
    /* Equal request/receive lower bounds cannot establish a causal response. */
    candidate_feedback(8000U, 0U);
    (void)adrc_app_bridge_service(&application_adrc_bridge, 12000U);
    assert(application_adrc_bridge.capture.count == 0U);
    assert(adrc_app_bridge_pop_frame(&application_adrc_bridge, &frame, &kind, &decoded));
    adrc_app_bridge_report_transmit(&application_adrc_bridge, kind, decoded, 1U);
    for (sample = 0U; sample < 64U; ++sample)
    {
        uint64_t now_us = 16000ULL + sample * 4000ULL;
        candidate_feedback(now_us - 4000ULL, 0U);
        (void)adrc_app_bridge_service(&application_adrc_bridge, now_us);
        if (adrc_app_bridge_pop_frame(&application_adrc_bridge, &frame, &kind, &decoded))
        {
            assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
            adrc_app_bridge_report_transmit(&application_adrc_bridge, kind, decoded, 1U);
        }
    }
    assert(application_adrc_bridge.capture.state == 2U);
    assert(application_adrc_bridge.capture.count == 64U);
    assert(application_adrc_bridge.bench.trusted_qualification.verified_flags == 0U);
    assert(application_adrc_bridge.bench.experiment.configured == 0U);
    assert(aethor_app_process_protocol_line("103 adrc capture index=63",
        strlen("103 adrc capture index=63"), 280000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "rx_lower_us=") != NULL);
    send_request("100 adrc sample", 280000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    send_request("104 adrc capture index=64", 280000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
}

/** @brief Missing feedback, failed receipts and release abort capture without authorizing motion. */
static void test_disabled_capture_failure(void)
{
    unsigned int scenario;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    for (scenario = 0U; scenario < 11U; ++scenario)
    {
        aethor_app_init(1000U, 801U);
        candidate_feedback(4000U, 0U);
        send_request("100 adrc sample", 4000U, PROTOCOL_ENGINE_STATUS_OK);
        (void)adrc_app_bridge_service(&application_adrc_bridge, 8000U);
        assert(adrc_app_bridge_pop_frame(&application_adrc_bridge, &frame, &kind, &decoded));
        if (scenario != 1U)
        { adrc_app_bridge_report_transmit(&application_adrc_bridge, kind, decoded, (uint8_t)(scenario != 0U)); }
        if (scenario == 2U) { adrc_app_bridge_request_release_disable(&application_adrc_bridge); }
        if (scenario == 3U) { candidate_feedback(12000U, 1U); }
        if (scenario >= 5U) { candidate_feedback(12000U, 0U); }
        if (scenario == 5U) { application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = 0.051F; }
        if (scenario == 6U) { application_motor_runtime.bank.motors[0].feedback.fault_flags = 1U; }
        if (scenario == 7U) { application_motor_runtime.discovery.results[0].software_version += 1U; }
        if (scenario == 8U) { application_motor_runtime.bank.motors[0].feedback.timestamp_us = 17000U; }
        if (scenario == 10U) { candidate_feedback(24000U, 0U); }
        (void)adrc_app_bridge_service(&application_adrc_bridge,
            scenario == 4U ? 28000U : scenario == 9U ? 404000U : scenario == 10U ? 24000U : 16000U);
        assert(application_adrc_bridge.capture.state == 3U);
        assert(application_adrc_bridge.bench.trusted_qualification.verified_flags == 0U);
        if (adrc_app_bridge_pop_frame(&application_adrc_bridge, &frame, &kind, &decoded))
        { assert(kind == 2U); }
    }
}

/** @brief Duplicate samples never fill the buffer; a missing interval fails without stretching 8 ms. */
static void test_disabled_capture_gap(void)
{
    CanFrame frame;
    uint8_t kind;
    float decoded;
    unsigned int tick;
    aethor_app_init(1000U, 802U);
    candidate_feedback(4000U, 0U);
    send_request("100 adrc sample", 4000U, PROTOCOL_ENGINE_STATUS_OK);
    for (tick = 0U; tick < 4U; ++tick)
    {
        uint64_t now_us = 8000ULL + tick * 4000ULL;
        if (tick == 1U) { candidate_feedback(12000U, 0U); }
        (void)adrc_app_bridge_service(&application_adrc_bridge, now_us);
        assert(application_adrc_bridge.capture.count <= 1U);
        assert(adrc_app_bridge_next_discovery(&application_adrc_bridge, now_us, &frame) == MOTOR_RUNTIME_STATUS_WAITING);
        assert(adrc_app_bridge_pop_frame(&application_adrc_bridge, &frame, &kind, &decoded));
        assert(kind == 2U);
        adrc_app_bridge_report_transmit(&application_adrc_bridge, kind, decoded, 1U);
    }
    (void)adrc_app_bridge_service(&application_adrc_bridge, 24000U);
    assert(application_adrc_bridge.capture.state == 3U);
    assert(application_adrc_bridge.capture.count == 1U);
}

/** @brief Invalid payloads, backward time and the request budget cannot produce a successful capture. */
static void test_disabled_capture_bounds(void)
{
    unsigned int scenario;
    unsigned int tick;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    for (scenario = 0U; scenario < 5U; ++scenario)
    {
        aethor_app_init(1000U, 803U);
        candidate_feedback(4000U, 0U);
        send_request("100 adrc sample", 4000U, PROTOCOL_ENGINE_STATUS_OK);
        for (tick = 0U; tick < 72U; ++tick)
        {
            uint64_t now_us = 8000ULL + tick * 4000ULL;
            if (scenario != 4U || (tick & 1U)) { candidate_feedback(now_us, 0U); }
            if (tick == 2U)
            {
                if (scenario == 0U) { application_motor_runtime.bank.motors[0].feedback.position_rad = NAN; }
                if (scenario == 1U) { application_motor_runtime.bank.motors[0].feedback.position_rad = 0.006F; }
                if (scenario == 2U) { application_motor_runtime.bank.motors[0].feedback.timestamp_us = 11000U; }
                if (scenario == 3U) { now_us = 3000U; }
            }
            (void)adrc_app_bridge_service(&application_adrc_bridge, now_us);
            if (application_adrc_bridge.capture.state == 3U) { break; }
            assert(adrc_app_bridge_pop_frame(&application_adrc_bridge, &frame, &kind, &decoded));
            assert(kind == 2U);
            adrc_app_bridge_report_transmit(&application_adrc_bridge, kind, decoded, 1U);
        }
        assert(application_adrc_bridge.capture.state == 3U);
        assert(application_adrc_bridge.capture.sent <= 68U);
        if (scenario == 4U) { assert(application_adrc_bridge.capture.sent == 68U); }
        assert(application_adrc_bridge.bench.trusted_qualification.verified_flags == 0U);
    }
}

/** @brief Ensures opt-in ADRC builds reject legacy actuation and lack startup qualification. */
static uint8_t fake_guard_state;
static uint32_t fake_guard_deadline;
/** @brief Host-only timer fixture accepts only a fixed motor-7 DISABLE. */
static uint8_t fake_guard_arm(const CanFrame *frame, uint32_t now_ms, uint32_t duration_ms)
{
    assert(frame->identifier == 7U && frame->data[7] == 0xFDU && duration_ms == 20U);
    if (fake_guard_state) { return 0U; }
    fake_guard_deadline = now_ms + duration_ms;
    fake_guard_state = 1U;
    return 1U;
}
/** @brief Reports synthetic timer evidence only; never linked into a firmware target. */
static uint8_t fake_guard_observe(uint32_t *deadline_ms, uint32_t *fired_ms)
{
    *deadline_ms = fake_guard_deadline;
    *fired_ms = fake_guard_state == 3U ? fake_guard_deadline : 0U;
    return fake_guard_state;
}
/** @brief Checks that the trusted caller does not unlock before transmission proof. */
static uint8_t fake_guard_ack(void)
{
    assert(fake_guard_state == 3U);
    fake_guard_state = 0U;
    return 1U;
}
/** @brief Exercises the disabled-only stall test and rejects absent or unsafe proof. */
static void test_stop_guard(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 8U; ++scenario)
    {
        CanFrame frame;
        uint8_t kind;
        float decoded;
        AdrcAppBridge *bridge = &application_adrc_bridge;
        aethor_app_init(1000U, 901U);
        fixture_feedback(6U, 4000U, 0U);
        bridge->bench.draft_config.axis_index = 6U;
        bridge->capture.axis = 6U;
        bridge->capture.state = 2U;
        bridge->capture.count = 64U;
        bridge->capture.discovery = application_motor_runtime.discovery.results[6];
        bridge->guard_test.hooks.arm = fake_guard_arm;
        bridge->guard_test.hooks.observe = fake_guard_observe;
        bridge->guard_test.hooks.ack_disabled = fake_guard_ack;
        fake_guard_state = 0U;
        send_request("100 adrc guard", 4000U, PROTOCOL_ENGINE_STATUS_OK);
        send_request("101 adrc sample", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        send_request("102 adrc prepare", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        (void)adrc_app_bridge_service(bridge, 8000U);
        assert(adrc_app_bridge_pop_frame(bridge, &frame, &kind, &decoded));
        assert(kind == 2U && frame.identifier == 7U && frame.data[7] == 0xFDU);
        adrc_app_bridge_report_transmit(bridge, kind, decoded, 1U);
        fixture_feedback(6U, (scenario == 1U || scenario == 7U) ? 8000U : 12000U, 0U);
        (void)adrc_app_bridge_service(bridge, 12000U);
        if (scenario == 1U)
        {
            assert(adrc_app_bridge_pop_frame(bridge, &frame, &kind, &decoded));
            adrc_app_bridge_report_transmit(bridge, kind, decoded, 1U);
            (void)adrc_app_bridge_service(bridge, 28000U);
            assert(bridge->guard_test.state == 6U && fake_guard_state == 0U);
            continue;
        }
        if (scenario == 7U)
        {
            assert(bridge->guard_test.state == 2U && bridge->guard_test.request_us == 8000U);
            assert(adrc_app_bridge_pop_frame(bridge, &frame, &kind, &decoded) && kind == 2U);
            adrc_app_bridge_report_transmit(bridge, kind, decoded, 1U);
            fixture_feedback(6U, 12000U, 0U);
            (void)adrc_app_bridge_service(bridge, 16000U);
            assert(bridge->guard_test.requests == 2U);
        }
        assert(bridge->guard_test.state == 3U && bridge->guard_test.pause_ms == 24U);
        fake_guard_state = scenario == 2U ? 2U : 3U;
        if (scenario == 3U) { adrc_app_bridge_request_stop(bridge); }
        if (scenario == 4U) { application_motor_runtime.discovery.results[6].observed_master_id++; }
        (void)adrc_app_bridge_service(bridge, scenario == 7U ? 40000U : 36000U);
        if (scenario >= 2U && scenario <= 4U)
        { assert(bridge->guard_test.state == 6U); continue; }
        assert(bridge->guard_test.state == 4U);
        assert(adrc_app_bridge_pop_frame(bridge, &frame, &kind, &decoded));
        assert(kind == 2U);
        adrc_app_bridge_report_transmit(bridge, kind, decoded, scenario != 5U);
        fixture_feedback(6U, 40000U, scenario == 6U);
        (void)adrc_app_bridge_service(bridge, scenario == 7U ? 44000U : 40000U);
        if (scenario == 7U)
        {
            assert(bridge->guard_test.state == 4U && bridge->guard_test.request_us == 40000U);
            assert(adrc_app_bridge_pop_frame(bridge, &frame, &kind, &decoded) && kind == 2U);
            adrc_app_bridge_report_transmit(bridge, kind, decoded, 1U);
            fixture_feedback(6U, 44000U, 0U);
            (void)adrc_app_bridge_service(bridge, 48000U);
        }
        assert(bridge->guard_test.state == ((scenario == 0U || scenario == 7U) ? 5U : 6U));
        assert(bridge->bench.trusted_qualification.verified_flags == 0U);
        assert(fake_guard_state == ((scenario == 0U || scenario == 7U) ? 0U : 3U));
    }
}

/** @brief Synthetic commissioning failures must never install evidence or fall through to RUN. */
static void test_watchdog_probe(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 26U; ++scenario)
    {
        AdrcAppBridge *bridge = &application_adrc_bridge;
        CanFrame frame;
        uint8_t kind;
        float decoded;
        uint64_t timestamp_us;
        uint64_t last_command_us = 0ULL;
        uint8_t motor_state = 0U;
        uint32_t timeout_raw = 0U;
        uint8_t injected = 0U;
        uint8_t expect_immediate_disable = 0U;
        uint8_t request_only_age_rejection_seen = 0U;
        aethor_app_init(1000U, 902U);
        fixture_feedback(6U, 4000U, 0U);
        bridge->bench.draft_config.axis_index = 6U;
        bridge->capture.axis = 6U; bridge->capture.state = 2U; bridge->capture.count = 64U;
        bridge->capture.discovery = application_motor_runtime.discovery.results[6];
        bridge->guard_test.hooks.arm = fake_guard_arm;
        bridge->guard_test.hooks.observe = fake_guard_observe;
        bridge->guard_test.hooks.ack_disabled = fake_guard_ack;
        bridge->guard_test.state = 5U; /* Explicit host-only proof fixture. */
        fake_guard_state = 0U;
        send_request("100 adrc watchdog", 4000U, PROTOCOL_ENGINE_STATUS_OK);
        send_request("101 adrc prepare", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        for (timestamp_us = 8000ULL; timestamp_us < 300000ULL; timestamp_us += 4000ULL)
        {
            MotorJointFeedback *feedback = &application_motor_runtime.bank.motors[6].feedback;
            if (!injected && ((scenario == 22U && bridge->watchdog.state == 11U) ||
                (scenario == 23U && bridge->watchdog.state == 13U) ||
                (scenario == 24U && bridge->watchdog.state == 14U)))
            {
                bridge->bench.experiment.status.disabled_confirmed = 1U;
                adrc_app_bridge_request_stop(bridge);
                expect_immediate_disable = 1U;
                injected = 1U;
            }
            if (!injected && ((scenario == 7U && bridge->watchdog.state == 3U) ||
                (scenario == 8U && bridge->watchdog.state == 11U)))
            {
                timestamp_us = bridge->watchdog.first_request_us + 20000ULL;
                feedback->timestamp_us = timestamp_us;
                if (scenario == 7U) { bridge->watchdog.parameter_rx_us = timestamp_us; }
                injected = 1U;
            }
            if (fake_guard_state == 1U && timestamp_us / 1000ULL >= fake_guard_deadline && scenario != 4U)
            {
                fake_guard_state = 3U;
                if ((scenario != 5U && scenario != 25U) || motor_state != 13U) { motor_state = 0U; }
                feedback->timestamp_us = timestamp_us;
                feedback->driver_state = motor_state;
                feedback->fault_flags = motor_state == 13U ? 13U : 0U;
            }
            if (motor_state == 1U && timeout_raw == 120U &&
                timestamp_us - last_command_us >= 8000ULL && scenario != 2U)
            {
                motor_state = 13U;
                /* Request-only drivers latch D without publishing an unsolicited frame.
                 * Scenario 25 preserves the last RX lower bound until an actual request. */
                if (scenario == 25U) { injected = 1U; }
                else
                {
                    feedback->driver_state = 13U; feedback->fault_flags = 13U;
                    feedback->timestamp_us = timestamp_us;
                }
            }
            if (!injected && bridge->watchdog.state == 9U && scenario >= 14U && scenario <= 19U)
            {
                if (scenario == 14U) { adrc_app_bridge_request_stop(bridge); }
                if (scenario == 15U) { feedback->driver_state = 8U; feedback->fault_flags = 8U; }
                if (scenario == 16U) { feedback->velocity_rad_s = 0.051F; }
                if (scenario == 17U) { feedback->position_rad = 0.0051F; }
                if (scenario == 18U) { feedback->mos_temperature_c = 50.0F; }
                if (scenario == 19U) { feedback->torque_nm = 0.051F; }
                injected = 1U;
            }
            if (scenario == 25U && bridge->watchdog.state == 9U &&
                timestamp_us - feedback->timestamp_us > 8000ULL)
            {
                assert(feedback->driver_state == 1U && feedback->fault_flags == 0U);
                assert(timestamp_us - feedback->timestamp_us == 12000ULL);
                assert(bridge->watchdog.reason == 0U && !bridge->watchdog.watchdog_seen);
                request_only_age_rejection_seen = 1U;
            }
            (void)adrc_app_bridge_service(bridge, timestamp_us);
            if (expect_immediate_disable)
            {
                assert(bridge->watchdog.state == 16U && bridge->frame_pending && bridge->frame_kind == 2U);
                expect_immediate_disable = 0U;
            }
            if (adrc_app_bridge_pop_frame(bridge, &frame, &kind, &decoded))
            {
                uint8_t fail_tx = (uint8_t)(!injected &&
                    ((scenario == 9U && kind == 0U) ||
                     (scenario == 10U && kind == 1U && bridge->watchdog.state == 8U) ||
                     (scenario == 11U && kind == 4U && bridge->watchdog.state == 2U) ||
                     (scenario == 12U && kind == 4U && bridge->watchdog.state == 13U) ||
                     (scenario == 13U && kind == 4U && bridge->watchdog.state == 14U)));
                adrc_app_bridge_report_transmit(bridge, kind, decoded, (uint8_t)!fail_tx);
                if (fail_tx)
                {
                    injected = 1U;
                    expect_immediate_disable = (uint8_t)(scenario == 12U || scenario == 13U);
                    continue;
                }
                if (kind == 4U)
                {
                    CanFrame response = frame;
                    if (frame.data[2] == 0x55U) { timeout_raw = frame.data[4]; }
                    response.identifier = 23U; response.length = 8U;
                    response.data[4] = (uint8_t)((scenario == 3U && timeout_raw == 120U) ? 119U : timeout_raw);
                    if (scenario == 20U && bridge->watchdog.state == 14U)
                    { response.data[4] = 120U; injected = 1U; }
                    if (scenario == 21U && bridge->watchdog.state == 14U)
                    { injected = 1U; continue; }
                    assert(adrc_app_bridge_accept_watchdog_parameter(bridge, &response,
                        scenario == 6U ? bridge->watchdog.first_request_us : timestamp_us + 4000ULL));
                }
                else
                {
                    assert(frame.identifier == 7U);
                    if (kind == 0U) { motor_state = scenario == 1U ? 0U : 1U; }
                    if (kind == 0U || kind == 1U) { last_command_us = timestamp_us; }
                    if (kind == 2U && ((scenario != 5U && scenario != 25U) || motor_state != 13U)) { motor_state = 0U; }
                    feedback->timestamp_us = scenario == 25U ? timestamp_us : timestamp_us + 4000ULL;
                    feedback->driver_state = motor_state;
                    feedback->fault_flags = motor_state == 13U ? 13U : 0U;
                }
            }
            if (bridge->watchdog.state >= 15U) { break; }
        }
        assert(bridge->watchdog.state >= 15U);
        if (scenario >= 7U) { assert(injected); }
        if (scenario == 7U) { assert(bridge->watchdog.reason == 7U && !bridge->watchdog.enabled_seen); }
        if (scenario == 8U) { assert(bridge->watchdog.reason == 3U && bridge->watchdog.state == 16U); }
        if (scenario == 12U || scenario == 13U)
        { assert(bridge->watchdog.reason == 1U && bridge->watchdog.state == 16U); }
        if (scenario == 25U)
        {
            assert(request_only_age_rejection_seen);
            assert(bridge->watchdog.failure_phase == 9U);
            assert(bridge->watchdog.failure_observed_us - bridge->watchdog.failure_feedback.timestamp_us == 12000ULL);
            assert(bridge->watchdog.failure_feedback.driver_state == 1U);
            assert(bridge->watchdog.failure_feedback.fault_flags == 0U);
            assert(bridge->watchdog.failure_guard_state == 1U);
            send_request("105 adrc watchdog_fault", timestamp_us, PROTOCOL_ENGINE_STATUS_OK);
            send_request("106 adrc watchdog_fault extra=1", timestamp_us, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
            assert(bridge->watchdog.enabled_seen && !bridge->watchdog.watchdog_seen);
            assert(bridge->watchdog.reason == 3U && bridge->watchdog.state == 16U);
            assert(!bridge->watchdog.timeout_restored && timeout_raw == 120U);
            assert(motor_state == 13U && fake_guard_state == 3U);
        }
        if (scenario == 0U)
        {
            assert(bridge->watchdog.state == 15U);
            assert(bridge->watchdog.enabled_seen && bridge->watchdog.watchdog_seen);
            assert(bridge->watchdog.timeout_restored && timeout_raw == 0U);
        }
        else
        {
            if (bridge->watchdog.state == 15U) { fprintf(stderr, "unexpected watchdog pass: %u\n", scenario); }
            assert(bridge->watchdog.state != 15U);
        }
        assert(bridge->bench.trusted_qualification.verified_flags == 0U);
        assert(!bridge->bench.experiment.configured);
    }
}

/** @brief Models the finite lease independently of bridge service progress. */
static uint8_t fake_simple_arm(const CanFrame *frame, uint32_t now_ms)
{
    assert(frame->identifier == 7U && frame->data[7] == 0xFDU);
    if (fake_guard_state) { return 0U; }
    fake_guard_state = 1U; fake_guard_deadline = now_ms + 12U; return 1U;
}
/** @brief Rejects expired synthetic leases, never reviving a requested stop. */
static uint8_t fake_simple_renew(uint32_t now_ms)
{
    if (fake_guard_state != 1U || now_ms >= fake_guard_deadline) { return 0U; }
    fake_guard_deadline = now_ms + 12U; return 1U;
}
/** @brief Exercises actual encoding, receipts, stop debt and immutable binding without a responding plant. */
static void test_simple_bridge(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 19U; ++scenario)
    {
        AdrcAppBridge *bridge = &application_adrc_bridge;
        MotorJointFeedback *feedback;
        AdrcSimpleGuardHooks hooks;
        uint64_t now_us;
        CanFrame frame;
        uint8_t kind, enabled = 0U, saw_enable = 0U;
        float decoded, maximum_decoded = 0.0F, previous_decoded = 0.0F;
        uint64_t positive_platform_us = 0U, negative_platform_us = 0U;
        const uint8_t profile = scenario == 18U ? 8U : (scenario >= 15U ? (uint8_t)(scenario - 10U) : (scenario >= 9U ? (uint8_t)(scenario - 8U) : 0U));
        aethor_app_init(1000U, 905U);
        fixture_feedback(6U, 4000U, 0U);
        feedback = &application_motor_runtime.bank.motors[6].feedback;
        feedback->velocity_rad_s = -200.0F / 4095.0F;
        application_motor_runtime.discovery.results[6].ranges.position_max_rad = 12.5F;
        application_motor_runtime.discovery.results[6].ranges.velocity_max_rad_s = 200.0F;
        application_motor_runtime.discovery.results[6].ranges.torque_max_nm = 10.0F;
        bridge->bench.draft_config.axis_index = 6U;
        bridge->capture.axis = 6U; bridge->capture.state = 2U; bridge->capture.count = 64U;
        bridge->capture.discovery = application_motor_runtime.discovery.results[6];
        bridge->guard_test.state = 5U; bridge->guard_test.hooks.observe = fake_guard_observe;
        bridge->guard_test.hooks.ack_disabled = fake_guard_ack;
        hooks.arm_trial = fake_simple_arm; hooks.renew_trial = fake_simple_renew;
        adrc_app_bridge_set_simple_guard_hooks(bridge, &hooks);
        fake_guard_state = 0U;
        send_request("90 adrc simple profile=9", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        send_request("91 adrc simple profile=-1", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        send_request("92 adrc simple profile=1 torque=1", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        if (profile == 1U) { send_request("100 adrc simple profile=1", 4000U, PROTOCOL_ENGINE_STATUS_OK); }
        else if (profile == 2U) { send_request("100 adrc simple profile=2", 4000U, PROTOCOL_ENGINE_STATUS_OK); }
        else if (profile == 3U) { send_request("100 adrc simple profile=3", 4000U, PROTOCOL_ENGINE_STATUS_OK); }
        else if (profile == 4U) { send_request("100 adrc simple profile=4", 4000U, PROTOCOL_ENGINE_STATUS_OK); }
        else if (profile == 5U) { send_request("100 adrc simple profile=5", 4000U, PROTOCOL_ENGINE_STATUS_OK); }
        else if (profile == 6U) { send_request("100 adrc simple profile=6", 4000U, PROTOCOL_ENGINE_STATUS_OK); }
        else if (profile == 7U) { send_request("100 adrc simple profile=7", 4000U, PROTOCOL_ENGINE_STATUS_OK); }
        else if (profile == 8U) { send_request("100 adrc simple profile=8", 4000U, PROTOCOL_ENGINE_STATUS_OK); }
        else { send_request("100 adrc simple", 4000U, PROTOCOL_ENGINE_STATUS_OK); }
        send_request("101 adrc simple", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        send_request("102 adrc watchdog", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        send_request("103 adrc prepare", 4000U, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        if (scenario == 1U) { adrc_app_bridge_request_stop(bridge); }
        if (scenario == 5U) { fake_guard_state = 2U; }
        /* Hardware selftest-to-host-start gap leaves the initial disabled sample stale. */
        for (now_us = scenario >= 7U ? 40000ULL : 8000ULL; now_us < 1800000ULL; now_us += 4000ULL)
        {
            if (fake_guard_state == 1U && now_us / 1000ULL >= fake_guard_deadline)
            { fake_guard_state = 3U; enabled = 0U; }
            if ((scenario != 3U || now_us < 28000ULL) && !(scenario == 7U && now_us == 40000ULL) &&
                !(scenario == 8U && now_us < 160000ULL))
            { feedback->timestamp_us = now_us - 4000ULL; }
            feedback->driver_state = enabled;
            if (scenario == 4U && now_us == 28000ULL)
            { application_motor_runtime.discovery.results[6].software_version++; }
            if (scenario == 6U && now_us == 28000ULL)
            {
                send_request("107 adrc stop", now_us, PROTOCOL_ENGINE_STATUS_OK);
                assert(!bridge->bench.gateway.stop_pending && bridge->bench.gateway.channel_state[1] == 0U);
            }
            /* Arithmetic-only fit injection reaches the negative pulse to inspect the real codec.
             * It is not plant evidence and cannot qualify hardware. */
            if (scenario >= 15U && profile != 8U && bridge->simple.status.phase == ADRC_SIMPLE_POSITIVE &&
                now_us - bridge->simple.phase_us == 312000ULL)
            {
                bridge->simple.pulse_start_seen = 1U; bridge->simple.pulse_end_seen = 1U;
                bridge->simple.pulse_velocity_start = 0.0F; bridge->simple.pulse_velocity_end = 0.1F;
                bridge->simple.pulse_midpoint_start_us = bridge->simple.phase_us + 152000ULL;
                bridge->simple.pulse_midpoint_end_us = bridge->simple.phase_us + 280000ULL;
            }
            (void)adrc_app_bridge_service(bridge, now_us);
            if (profile == 8U) {
                assert(bridge->simple.status.phase != ADRC_SIMPLE_NEUTRAL_2 && bridge->simple.status.phase != ADRC_SIMPLE_NEGATIVE &&
                    bridge->simple.status.phase != ADRC_SIMPLE_NEUTRAL_3 && bridge->simple.status.phase != ADRC_SIMPLE_CONTROL);
                assert(bridge->simple.status.b0 == 0.0F);
            }
            if (adrc_app_bridge_pop_frame(bridge, &frame, &kind, &decoded))
            {
                assert(frame.identifier == 7U && kind <= 2U);
                assert(fabsf(decoded) <= bridge->simple.torque_limit_nm);
                if (kind == 1U && profile >= 5U)
                {
                    uint64_t pulse_age = now_us - bridge->simple.phase_us;
                    float inward_plateau = profile == 8U ? 1.746032F : (profile >= 7U ? 0.34920635F : (profile == 5U ? 0.1978022F : 0.2466422F));
                    float coded_step = (profile >= 7U ? 60.0F : 40.0F) / 4095.0F;
                    assert(fabsf(decoded - previous_decoded) <= (profile >= 7U ? 0.016001F : 0.012001F));
                    if (fabsf(decoded - previous_decoded) > (profile >= 7U ? 0.012F : 0.0075F))
                    { assert(fabsf(fabsf(decoded - previous_decoded) - coded_step) < 0.000002F); }
                    if (bridge->simple.status.phase == ADRC_SIMPLE_POSITIVE)
                    {
                        if (!positive_platform_us && fabsf(decoded - inward_plateau) < 0.000002F) positive_platform_us = pulse_age;
                        if (pulse_age >= (profile == 8U ? 480000ULL : 120000ULL)) assert(fabsf(decoded - inward_plateau) < 0.000002F);
                    }
                    if (bridge->simple.status.phase == ADRC_SIMPLE_NEGATIVE)
                    {
                        if (!negative_platform_us && fabsf(decoded + inward_plateau) < 0.000002F) negative_platform_us = pulse_age;
                        if (pulse_age >= 168000ULL) assert(fabsf(decoded + inward_plateau) < 0.000002F);
                    }
                    previous_decoded = decoded;
                }
                if (kind == 1U && decoded > maximum_decoded) { maximum_decoded = decoded; }
                if (kind == 0U) { assert(fake_guard_state == 1U); enabled = 1U; saw_enable = 1U; }
                if (kind == 2U) { enabled = 0U; }
                adrc_app_bridge_report_transmit(bridge, kind, decoded,
                    (uint8_t)!(scenario == 2U && now_us == 12000ULL));
            }
            if (bridge->simple.status.phase == ADRC_SIMPLE_DONE && bridge->simple_guard_ack) { break; }
            if (bridge->simple.status.phase == ADRC_SIMPLE_LOCKED) { break; }
        }
        assert(bridge->simple.status.phase >= ADRC_SIMPLE_DONE);
        assert(bridge->bench.trusted_qualification.verified_flags == 0U);
        if (profile)
        {
            if (profile >= 5U)
            {
                assert(positive_platform_us && positive_platform_us <= (profile == 8U ? 480000ULL : 120000ULL));
                if (scenario >= 15U && profile != 8U) assert(negative_platform_us && negative_platform_us <= 168000ULL);
                printf("bridge profile=%u positive_platform_us=%llu negative_platform_us=%llu actual_slope=%.9f Nm/s\n",
                    profile, (unsigned long long)positive_platform_us, (unsigned long long)negative_platform_us, (profile >= 7U ? 60.0 : 40.0) / 4095.0 / 0.004);
            }
            assert(bridge->simple.profile == profile);
            assert(maximum_decoded > bridge->simple.torque_limit_nm - 0.005F);
            assert(bridge->simple.status.result == (profile == 8U ? ADRC_SIMPLE_PROBE_COMPLETED : ADRC_SIMPLE_COMPLETED_NO_RESPONSE));
            assert(bridge->simple.status.safe_release && bridge->simple_guard_ack);
            if (profile >= 4U)
            {
                ProtocolOutputBatch status_output;
                const char *status_request = "4294967295 adrc simple_status";
                /* Largest request/trial identifiers and float rendering must retain both duration fields. */
                bridge->simple_request_id = UINT32_MAX;
                bridge->simple.status.b0 = 3.402823466e+38F;
                assert(aethor_app_process_protocol_line(status_request, (uint16_t)strlen(status_request),
                    now_us, &status_output) == PROTOCOL_ENGINE_STATUS_OK);
                assert(strstr(status_output.messages[0].data, profile == 8U ? "pulse_us=640000" : "pulse_us=360000") != NULL);
                if (profile >= 5U) {
                    assert(strstr(status_output.messages[0].data, "torque_trip=") != NULL);
                    assert(strstr(status_output.messages[0].data, (profile >= 7U ? "step=0.016" : "step=0.012")) != NULL);
                    assert(strstr(status_output.messages[0].data, (profile >= 7U ? "slew=4" : "slew=3")) != NULL);
                }
                assert(strstr(status_output.messages[0].data, profile == 8U ? "positive_us=640000 negative_us=0" : "positive_us=312000 negative_us=360000") != NULL);
                if (profile == 8U) {
                    assert(bridge->simple.probe_reached && fabsf(bridge->simple.probe_peak_nm - maximum_decoded) < 0.000001F);
                    assert(strstr(status_output.messages[0].data, "probe=1 probe_peak=") != NULL);
                    assert(strstr(status_output.messages[0].data, "probe_reached=1") != NULL);
                    assert(bridge->simple.pulse_integral_us == 0U && negative_platform_us == 0U);
                } else assert(strstr(status_output.messages[0].data, "probe=0 probe_peak=0 probe_reached=0") != NULL);
                assert(strlen(status_output.messages[0].data) < PROTOCOL_ENGINE_MESSAGE_CAPACITY - 2U);
                if (profile == 8U) printf("startup probe status capacity: bytes=%u capacity=%u actual_peak=%.9f Nm\n",
                    (unsigned)strlen(status_output.messages[0].data), (unsigned)PROTOCOL_ENGINE_MESSAGE_CAPACITY,
                    (double)maximum_decoded);
            }
        }
        if (scenario == 1U) { assert(!saw_enable && bridge->simple_guard_ack && bridge->simple.status.safe_release); }
        if (scenario == 5U) { assert(!saw_enable && !bridge->simple_guard_ack); }
        if (scenario == 8U)
        { assert(!saw_enable && bridge->simple_guard_ack && bridge->simple.status.safe_release);
          assert(bridge->simple.status.result == ADRC_SIMPLE_FAILED && bridge->simple.status.fault == ADRC_FAULT_FEEDBACK); }
        if (scenario == 6U)
        { assert(bridge->simple.status.result == ADRC_SIMPLE_ABORTED && bridge->simple_guard_ack); }
        if (scenario == 0U || scenario == 7U)
        {
            assert(bridge->simple.status.result == ADRC_SIMPLE_COMPLETED_NO_RESPONSE);
            assert(bridge->simple.status.safe_release && bridge->simple_guard_ack);
            adrc_app_bridge_request_release_disable(bridge);
            (void)adrc_app_bridge_service(bridge, now_us + 4000ULL);
            assert(adrc_app_bridge_pop_frame(bridge, &frame, &kind, &decoded) && kind == 2U);
            assert(bridge->simple.status.result == ADRC_SIMPLE_COMPLETED_NO_RESPONSE);
            send_request("104 adrc simple_status", now_us + 4000ULL, PROTOCOL_ENGINE_STATUS_OK);
            send_request("105 adrc simple_trace index=0", now_us + 4000ULL, PROTOCOL_ENGINE_STATUS_OK);
            send_request("106 adrc simple_trace index=400", now_us + 4000ULL, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        }
    }
}

/** @brief A matched peak receipt survives immediate STOP, guard loss or discovery abort, never enabling completion. */
static void test_simple_probe_receipt_abort(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<3U;++scenario) {
  AdrcAppBridge *bridge=&application_adrc_bridge; MotorJointFeedback *feedback;
  AdrcSimpleGuardHooks hooks; CanFrame frame; uint8_t kind, enabled=0U, injected=0U;
  uint64_t now_us; float decoded=0.0F, actual_peak=0.0F;
  aethor_app_init(1000U,905U); fixture_feedback(6U,4000U,0U);
  feedback=&application_motor_runtime.bank.motors[6].feedback;
  feedback->velocity_rad_s=-200.0F/4095.0F;
  application_motor_runtime.discovery.results[6].ranges.position_max_rad=12.5F;
  application_motor_runtime.discovery.results[6].ranges.velocity_max_rad_s=200.0F;
  application_motor_runtime.discovery.results[6].ranges.torque_max_nm=10.0F;
  bridge->bench.draft_config.axis_index=6U;
  bridge->capture.axis=6U; bridge->capture.state=2U; bridge->capture.count=64U;
  bridge->capture.discovery=application_motor_runtime.discovery.results[6];
  bridge->guard_test.state=5U; bridge->guard_test.hooks.observe=fake_guard_observe;
  bridge->guard_test.hooks.ack_disabled=fake_guard_ack;
  hooks.arm_trial=fake_simple_arm; hooks.renew_trial=fake_simple_renew;
  adrc_app_bridge_set_simple_guard_hooks(bridge,&hooks); fake_guard_state=0U;
  send_request("100 adrc simple profile=8",4000U,PROTOCOL_ENGINE_STATUS_OK);
  for(now_us=40000ULL;now_us<1200000ULL;now_us+=4000ULL) {
   feedback->timestamp_us=now_us-4000ULL; feedback->driver_state=enabled;
   (void)adrc_app_bridge_service(bridge,now_us);
   if(adrc_app_bridge_pop_frame(bridge,&frame,&kind,&decoded)) {
    if(kind==0U) enabled=1U;
    if(kind==2U) enabled=0U;
    if(!injected && kind==1U && decoded>=1.74F) {
     /* This receipt crosses the 99% cap boundary while the preceding peak is still below it. */
     assert(bridge->simple.probe_peak_nm<0.99F*bridge->simple.torque_limit_nm && !bridge->simple.probe_reached);
     actual_peak=decoded;
     adrc_app_bridge_report_transmit(bridge,kind,decoded,1U);
     if(scenario==0U) adrc_app_bridge_request_stop(bridge);
     if(scenario==1U) fake_guard_state=2U;
     if(scenario==2U) application_motor_runtime.discovery.results[6].software_version++;
     injected=1U;
    } else adrc_app_bridge_report_transmit(bridge,kind,decoded,1U);
   }
   if(injected && bridge->simple.status.phase>=ADRC_SIMPLE_STOPPING) {
    assert(kind==2U || (scenario==0U && bridge->simple.status.phase==ADRC_SIMPLE_STOPPING));
    assert(fabsf(bridge->simple.probe_peak_nm-actual_peak)<0.000001F && bridge->simple.probe_reached);
    assert(bridge->simple.status.result!=ADRC_SIMPLE_PROBE_COMPLETED && bridge->simple.status.b0==0.0F);
    assert(bridge->simple.pulse_integral_us==0U && bridge->simple.controller_reset);
    if(scenario==0U && kind!=2U) continue;
    break;
   }
  }
  assert(injected && actual_peak>1.74F);
  printf("probe last receipt before abort scenario=%u peak=%.9f reached=%u PASS\n",scenario,
   (double)bridge->simple.probe_peak_nm,(unsigned)bridge->simple.probe_reached);
 }
}
/** @brief Bridge rejects unmatched, failed, duplicate, stopped and DISABLE receipts even with stale high sent values. */
static void test_simple_probe_bridge_receipt_boundaries(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<9U;++scenario) {
  AdrcAppBridge *bridge=&application_adrc_bridge; uint8_t kind=1U, succeeded=1U;
  float actual_torque=1.74F;
  aethor_app_init(1000U,905U);
  bridge->simple_request_id=100U; bridge->simple.profile=8U;
  bridge->simple.status.phase=ADRC_SIMPLE_POSITIVE; bridge->simple.expected_receipt=2U;
  bridge->simple.torque_limit_nm=1.75F; bridge->simple.probe_peak_nm=1.0F;
  bridge->awaiting_receipt=1U; bridge->frame_kind=1U; bridge->decoded_torque_nm=1.74F;
  bridge->receipt_torque_nm=1.75F; bridge->receipt_valid=1U;
  if(scenario==1U) actual_torque=1.741F;
  if(scenario==2U) succeeded=0U;
  if(scenario==3U) kind=2U;
  if(scenario==4U) bridge->awaiting_receipt=0U;
  if(scenario==6U) bridge->simple.status.phase=ADRC_SIMPLE_STOPPING;
  if(scenario==7U) { kind=2U; bridge->frame_kind=2U; actual_torque=0.0F; bridge->decoded_torque_nm=0.0F; }
  if(scenario==8U) bridge->simple.expected_receipt=1U;
  adrc_app_bridge_report_transmit(bridge,kind,actual_torque,succeeded);
  if(scenario==0U || scenario==5U) {
   assert(bridge->simple.probe_peak_nm==1.74F && bridge->simple.probe_reached);
   if(scenario==5U) {
    /* A duplicate has no current frame ownership, even when the reported torque is larger. */
    bridge->decoded_torque_nm=1.75F;
    adrc_app_bridge_report_transmit(bridge,1U,1.75F,1U);
    assert(bridge->send_failed && bridge->simple.probe_peak_nm==1.74F);
   }
  } else assert(bridge->simple.probe_peak_nm==1.0F && !bridge->simple.probe_reached);
  assert(bridge->simple.status.b0==0.0F && !bridge->simple.status.safe_release);
 }
 puts("probe bridge current-frame receipt ownership/stopped/DISABLE boundaries: 9 cases PASS");
}
/** @brief Runs the synthetic facade regression without hardware access. */
/** @brief Completes a real preparation handshake using synthetic hardware-submit, receipt and later RX. */
static void complete_capability_prepare(uint64_t timestamp_us)
{
    CanFrame frame;
    uint8_t kind;
    float decoded;
    (void)aethor_app_service(timestamp_us);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) && kind == 2U && frame.identifier == 7U);
    aethor_app_adrc_report_disable_submit_at(kind, timestamp_us + 1ULL);
    aethor_app_adrc_report_transmit(kind, decoded, 1U);
    fixture_feedback(6U, timestamp_us + 4000ULL, 0U);
    (void)aethor_app_service(timestamp_us + 4001ULL);
    assert(application_adrc_bridge.capability_prepare_state == 2U);
    assert(application_adrc_bridge.capabilities.started_us == timestamp_us + 4001ULL);
}

/** @brief Capability collection is fixed, exclusive, read-only and preserves feedback/qualification. */
static void test_capability_bridge(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanFrame reply;
    CanTxPriority priority;
    uint32_t feedback_count;
    unsigned int item_index;
    static const float values[6] = {0.1F, 100.0F, 0.5F, 0.0F, -1.0F, 19.2032F};
    aethor_app_init(1000U, 123U);
    fixture_feedback(6U, 4000ULL, 0U);
    application_motor_runtime.discovery_active = 0U;
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    application_adrc_bridge.bench.draft_config.axis_index = 6U;
    application_adrc_bridge.discovery_axis = 6U;
    feedback_count = application_motor_runtime.accepted_feedback_count;
    send_request("201 adrc capabilities rid=1", 4001ULL, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    send_request("202 adrc capabilities", 4001ULL, PROTOCOL_ENGINE_STATUS_OK);
    send_request("203 adrc sample", 4002ULL, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    send_request("204 adrc run", 4002ULL, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    complete_capability_prepare(4002ULL);
    for (item_index = 0U; item_index < 6U; ++item_index)
    {
        uint64_t timestamp_us = 12004ULL + item_index * 8000ULL;
        uint32_t bits;
        assert(aethor_app_next_can_frame(timestamp_us - 4000ULL, &frame, &priority) == MOTOR_RUNTIME_STATUS_WAITING);
        aethor_app_adrc_observe_capability_rx_boundary(timestamp_us);
        assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_PARAMETER && frame.identifier == 0x7FFU && frame.length == 4U);
        assert(frame.data[2] == 0x33U && frame.data[3] == motor_capability_rid((uint8_t)item_index));
        aethor_app_adrc_report_capability_transmit(&frame, 1U, timestamp_us + 1ULL);
        reply = frame; reply.identifier = 23U; reply.length = 8U;
        memcpy(&bits, &values[item_index], sizeof(bits));
        memcpy(&reply.data[4], &bits, sizeof(bits));
        assert(aethor_app_receive_can_frame_observed(&reply, timestamp_us + 2ULL, timestamp_us + 3ULL) == MOTOR_RUNTIME_STATUS_OK);
    }
    assert(application_adrc_bridge.capabilities.count == 6U);
    assert(application_motor_runtime.accepted_feedback_count == feedback_count);
    assert(!application_adrc_bridge.bench.trusted_qualification.verified_flags);
    assert(aethor_app_process_protocol_line("205 adrc capabilities_get index=5", 33U, 52000ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "qualified=0") && output.messages[0].length < 520U);
    application_adrc_bridge.capabilities.epoch = UINT32_MAX;
    application_adrc_bridge.capabilities.request_id = UINT32_MAX;
    application_adrc_bridge.capabilities.items[5].request_created_us = UINT64_MAX - 20000ULL;
    application_adrc_bridge.capabilities.items[5].queue_submitted_us = UINT64_MAX;
    application_adrc_bridge.capabilities.items[5].pre_submit_boundary_us = UINT64_MAX;
    application_adrc_bridge.capabilities.items[5].send_completed_observed_us = UINT64_MAX;
    application_adrc_bridge.capabilities.items[5].rx_lower_us = UINT64_MAX;
    application_adrc_bridge.capabilities.items[5].received_observed_us = UINT64_MAX;
    application_adrc_bridge.capabilities.items[5].raw_bits = UINT32_MAX;
    application_adrc_bridge.capabilities.items[5].value = FLT_MAX;
    assert(aethor_app_process_protocol_line("4294967295 adrc capabilities_get index=5",
        strlen("4294967295 adrc capabilities_get index=5"), 52000ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(output.messages[0].length < 520U);
    printf("capability_get maximum ABI bytes=%u capacity=520\n", (unsigned int)output.messages[0].length);
    send_request("206 adrc capabilities_get index=6", 28000ULL, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    adrc_app_bridge_complete_release(&application_adrc_bridge);
    assert(aethor_app_receive_can_frame_observed(&reply, 28001ULL, 28002ULL) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.accepted_feedback_count == feedback_count);
    fixture_feedback(6U, 28003ULL, 0U);
    send_request("207 adrc capabilities", 28004ULL, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    aethor_app_init(1000U, 123U);
    fixture_feedback(6U, 4000ULL, 0U);
    application_motor_runtime.discovery_active = 0U;
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    application_adrc_bridge.bench.draft_config.axis_index = 6U;
    application_adrc_bridge.discovery_axis = 6U;
    send_request("208 adrc capabilities", 4001ULL, PROTOCOL_ENGINE_STATUS_OK);
    send_request("209 adrc stop", 4002ULL, PROTOCOL_ENGINE_STATUS_OK);
    assert(application_adrc_bridge.capabilities.state == MOTOR_CAPABILITY_ABORTED);
    assert(aethor_app_next_can_frame(4003ULL, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
}

/** @brief Rejects competing producers, stale/unhealthy feedback and pending legacy commands. */
static void test_capability_admission_boundaries(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 8U; ++scenario)
    {
        aethor_app_init(1000U, 123U);
        fixture_feedback(6U, 4000ULL, 0U);
        application_motor_runtime.discovery_active = 0U;
        application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
        application_adrc_bridge.bench.draft_config.axis_index = 6U;
        application_adrc_bridge.discovery_axis = 6U;
        if (scenario == 0U) { application_motor_runtime.position_read.pending = 1U; }
        if (scenario == 1U) { application_motor_runtime.parameter_expectations.count = 1U; }
        if (scenario == 2U) { application_motor_runtime.mode_switch_state = MOTOR_MODE_SWITCH_WRITING; }
        if (scenario == 3U) { application_adrc_bridge.bench.gateway.command_pending = 1U; }
        if (scenario == 4U) { application_motor_runtime.bank.motors[6].feedback.driver_state = 1U; }
        if (scenario == 5U) { application_motor_runtime.bank.motors[6].feedback.fault_flags = 1U; }
        if (scenario == 6U) { application_motor_runtime.bank.motors[6].feedback.rotor_temperature_c = 50.0F; }
        if (scenario == 7U) { application_motor_runtime.discovery.results[6].observed_master_id = 22U; }
        send_request("201 adrc capabilities", 4001ULL,
            PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!application_adrc_bridge.capabilities.used);
    }
}

/** @brief Stale cached feedback starts only bounded DISABLE preparation; missing causal RX never starts reads. */
static void test_capability_prepare_boundaries(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 5U; ++scenario)
    {
        CanFrame frame;
        uint8_t kind;
        float decoded;
        uint8_t cleanup_disable_seen = 0U;
        aethor_app_init(1000U, 123U); fixture_feedback(6U, 4000ULL, 0U);
        application_motor_runtime.discovery_active = 0U;
        application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
        application_adrc_bridge.bench.draft_config.axis_index = 6U;
        application_adrc_bridge.discovery_axis = 6U;
        send_request("201 adrc capabilities", 20000ULL, PROTOCOL_ENGINE_STATUS_OK);
        assert(!application_adrc_bridge.capabilities.started_us);
        (void)aethor_app_service(20001ULL);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) && kind == 2U && frame.identifier == 7U);
        if (scenario != 0U) { aethor_app_adrc_report_disable_submit_at(kind, 20002ULL); }
        aethor_app_adrc_report_transmit(kind, decoded, (uint8_t)(scenario != 1U));
        if (scenario == 2U) { fixture_feedback(6U, 20002ULL, 0U); } /* Equal submit bound is not causally late. */
        if (scenario == 3U) { fixture_feedback(6U, 24000ULL, 0U); } /* Fresh RX still lacks a hardware receipt until matched. */
        (void)aethor_app_service(24001ULL);
        if (scenario == 3U) { assert(application_adrc_bridge.capability_prepare_state == 2U); }
        else {
            assert(!application_adrc_bridge.capabilities.started_us);
            if (aethor_app_adrc_pop_frame(&frame, &kind, &decoded)) {
                if (application_adrc_bridge.capabilities.state == MOTOR_CAPABILITY_ABORTED && kind == 2U)
                { cleanup_disable_seen = 1U; }
                aethor_app_adrc_report_disable_submit_at(kind, 24002ULL);
                aethor_app_adrc_report_transmit(kind, decoded, 1U);
            }
            (void)aethor_app_service(84001ULL);
            assert(application_adrc_bridge.capabilities.state == MOTOR_CAPABILITY_ABORTED);
            if (aethor_app_adrc_pop_frame(&frame, &kind, &decoded)) {
                assert(kind == 2U && frame.identifier == 7U); cleanup_disable_seen = 1U;
            }
            assert(cleanup_disable_seen);
            send_request("202 adrc capabilities", 84002ULL, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        }
    }
}

/** @brief Final reply and deferred reply cannot commit after a later observed unhealthy motor frame. */
static void test_capability_commit_health(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 8U; ++scenario)
    {
        CanFrame frame;
        CanFrame reply;
        uint32_t raw_bits;
        float gear_ratio = 19.2032F;
        AdrcAppBridge *bridge = &application_adrc_bridge;
        aethor_app_init(1000U, 123U);
        fixture_feedback(6U, 4000ULL, 0U);
        application_motor_runtime.discovery_active = 0U;
        application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
        bridge->bench.draft_config.axis_index = 6U;
        bridge->discovery_axis = 6U;
        bridge->capability_discovery = application_motor_runtime.discovery.results[6];
        assert(motor_capability_start(&bridge->capabilities, 201U, bridge->capability_epoch, 4001ULL));
        bridge->capabilities.count = 5U; /* Synthetic previous five valid items; exercise final commit boundary. */
        assert(!motor_capability_next(&bridge->capabilities, bridge->capability_epoch, 4002ULL, &frame));
        motor_capability_observe_rx_boundary(&bridge->capabilities, 8000ULL);
        assert(motor_capability_next(&bridge->capabilities, bridge->capability_epoch, 8000ULL, &frame));
        reply = frame; reply.identifier = 23U; reply.length = 8U;
        memcpy(&raw_bits, &gear_ratio, sizeof(raw_bits)); memcpy(&reply.data[4], &raw_bits, sizeof(raw_bits));
        if (scenario < 4U) { aethor_app_adrc_report_capability_transmit(&frame, 1U, 8000ULL); }
        else {
            assert(aethor_app_receive_can_frame_observed(&reply, 8000ULL, 8001ULL) == MOTOR_RUNTIME_STATUS_OK);
            assert(bridge->capabilities.deferred && bridge->capabilities.count == 5U);
        }
        if (scenario % 4U < 2U) {
            CanFrame unhealthy_frame;
            uint8_t payload[8] = {0x17U, 0x7fU, 0xffU, 0x7fU, 0xf7U, 0xffU, 25U, 25U};
            if (scenario % 4U == 1U) { payload[0] = 0x87U; }
            assert(can_frame_init(&unhealthy_frame, 23U, payload, 8U) == CAN_FRAME_STATUS_OK);
            assert(aethor_app_receive_can_frame_observed(&unhealthy_frame, 8001ULL, 8002ULL) == MOTOR_RUNTIME_STATUS_OK);
            assert(bridge->capabilities.state == MOTOR_CAPABILITY_ABORTED); /* Same RX batch, before final reply/receipt. */
        }
        if (scenario % 4U == 2U) { application_motor_runtime.bank.motors[6].feedback.velocity_rad_s = NAN; }
        if (scenario % 4U == 3U) { application_motor_runtime.bank.motors[6].feedback.rotor_temperature_c = 50.0F; }
        if (scenario < 4U) { (void)aethor_app_receive_can_frame_observed(&reply, 8000ULL, 8002ULL); }
        else { aethor_app_adrc_report_capability_transmit(&frame, 1U, 8002ULL); }
        assert(bridge->capabilities.state == MOTOR_CAPABILITY_ABORTED && bridge->capabilities.count == 5U);
        assert(bridge->stop_pending && bridge->release_disable_pending);
        {
            ProtocolOutputBatch output;
            const char *status_request = "299 adrc capabilities_status";
            assert(aethor_app_process_protocol_line(status_request, strlen(status_request), 8003ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
            assert(strstr(output.messages[0].data, "current=0"));
        }
    }
}

/** @brief Core polls completion before queued control RX; completed diagnostics must never regain current validity. */
static void test_capability_completed_rx_order(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 4U; ++scenario)
    {
        AdrcAppBridge *bridge = &application_adrc_bridge;
        CanFrame frame;
        CanFrame reply;
        CanFrame feedback_frame;
        ProtocolOutputBatch output;
        uint8_t kind;
        float decoded;
        float gear_ratio = 19.2032F;
        uint32_t raw_bits;
        uint8_t feedback_payload[8] = {0x17U, 0x7fU, 0xffU, 0x7fU, 0xf7U, 0xffU, 25U, 25U};
        aethor_app_init(1000U, 123U); fixture_feedback(6U, 4000ULL, 0U);
        application_motor_runtime.discovery_active = 0U;
        application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
        bridge->bench.draft_config.axis_index = 6U; bridge->discovery_axis = 6U;
        bridge->capability_discovery = application_motor_runtime.discovery.results[6];
        assert(motor_capability_start(&bridge->capabilities, 201U, bridge->capability_epoch, 4001ULL));
        bridge->capabilities.count = 5U;
        assert(!motor_capability_next(&bridge->capabilities, bridge->capability_epoch, 4002ULL, &frame));
        motor_capability_observe_rx_boundary(&bridge->capabilities, 8000ULL);
        assert(motor_capability_next(&bridge->capabilities, bridge->capability_epoch, 8000ULL, &frame));
        reply = frame; reply.identifier = 23U; reply.length = 8U;
        memcpy(&raw_bits, &gear_ratio, sizeof(raw_bits)); memcpy(&reply.data[4], &raw_bits, sizeof(raw_bits));
        assert(aethor_app_receive_can_frame_observed(&reply, 8000ULL, 8000ULL) == MOTOR_RUNTIME_STATUS_OK);
        assert(bridge->capabilities.deferred);
        aethor_app_adrc_report_capability_transmit(&frame, 1U, 8001ULL);
        assert(bridge->capabilities.state == MOTOR_CAPABILITY_COMPLETE && bridge->capabilities.count == 6U);
        if (scenario < 2U) {
            if (scenario == 1U) { feedback_payload[0] = 0x87U; }
            assert(can_frame_init(&feedback_frame, 23U, feedback_payload, 8U) == CAN_FRAME_STATUS_OK);
            assert(aethor_app_receive_can_frame_observed(&feedback_frame, 8001ULL, 8002ULL) == MOTOR_RUNTIME_STATUS_OK);
            assert(bridge->capabilities.reason == MOTOR_CAPABILITY_REASON_BINDING);
            assert(bridge->capabilities.completed_revoked);
            assert(bridge->stop_pending && bridge->release_disable_pending);
        }
        else if (scenario == 2U) { adrc_app_bridge_request_stop(bridge); }
        else { adrc_app_bridge_request_release_disable(bridge); }
        feedback_payload[0] = 7U;
        assert(can_frame_init(&feedback_frame, 23U, feedback_payload, 8U) == CAN_FRAME_STATUS_OK);
        assert(aethor_app_receive_can_frame_observed(&feedback_frame, 12000ULL, 12001ULL) == MOTOR_RUNTIME_STATUS_OK);
        assert(bridge->capabilities.state == MOTOR_CAPABILITY_COMPLETE && bridge->capabilities.count == 6U);
        assert(bridge->capabilities.items[5].raw_bits == raw_bits && bridge->capabilities.items[5].value == gear_ratio);
        if (scenario < 2U) { assert(bridge->stop_pending && bridge->release_disable_pending); }
        else {
            assert(bridge->capabilities.reason == MOTOR_CAPABILITY_REASON_NONE);
            assert(!bridge->capabilities.completed_revoked && bridge->capability_terminal_closed);
        }
        (void)aethor_app_service(12002ULL);
        if (aethor_app_adrc_pop_frame(&frame, &kind, &decoded)) {
            assert(kind == 2U); aethor_app_adrc_report_transmit(kind, decoded, 1U);
        }
        assert(aethor_app_process_protocol_line("301 adrc capabilities_status",
            strlen("301 adrc capabilities_status"), 12003ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "current=0"));
        assert(aethor_app_process_protocol_line("302 adrc capabilities_get index=5",
            strlen("302 adrc capabilities_get index=5"), 12003ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "current=0"));
        adrc_app_bridge_complete_release(bridge);
        assert(bridge->capabilities.state == MOTOR_CAPABILITY_COMPLETE);
    }
}

/** @brief Runs the synthetic application facade regressions. */
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
    test_candidate_interface();
    test_selected_axis_and_receipts();
    test_negative_grid_and_coarse_resolution();
    test_identify_hard_deadlines();
    test_idle_protocol_stop_emits_disable();
    test_readonly_discovery_and_receive();
    test_readonly_feedback_query_pacing();
    test_readonly_hardware_diagnostics();
    test_deferred_bridge_discovery();
    test_motor7_mapping_quantization_characterization();
    test_disabled_capture();
    test_disabled_capture_failure();
    test_disabled_capture_gap();
    test_disabled_capture_bounds();
    test_stop_guard();
    test_watchdog_probe();
    test_simple_bridge(); test_simple_probe_receipt_abort(); test_simple_probe_bridge_receipt_boundaries();
    test_capability_bridge();
    test_capability_admission_boundaries();
    test_capability_commit_health();
    test_capability_prepare_boundaries();
    test_capability_completed_rx_order();
    puts("ADRC_APP_TESTS_PASSED");
    return 0;
}
