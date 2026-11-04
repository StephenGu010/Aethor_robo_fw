/** @file static_peak_reason_test.c
 * @brief Offline real-facade observations for takeover peaks and retained first exits.
 * Synthetic feedback never opens a board or changes physical safety limits.
 */
#define main existing_debug_ui_peak_fixture_main
#include "debug_ui_app_test_main.c"
#undef main
#include "../../App/DebugUi/debug_ui_load_trials.inc"

/** @brief Bind a selected-axis observation report to a synthetic accepted static command. */
static void peak_fixture(uint8_t mask, uint8_t takeover)
{
    uint8_t joint_index;
    test_fixture();
    if (mask != 0x7FU) {
        DebugUiSingleStaticProfile profile = { 1U, 1000U, 60000U, takeover ? -1.0F : 0.0F };
        application_motor_runtime.discovery.results[0].observed_control_mode = 1U;
        assert(aethor_app_debug_ui_set_single_static_profile(1U, &profile));
    }
    memset(&application_action, 0, sizeof(application_action));
    application_action.command.type = PROTOCOL_COMMAND_MIT_ACTION;
    application_action.command.mit_action = PROTOCOL_MIT_ACTION_HOLD;
    application_action.command.control_mode = ARM_CONTROL_MODE_MIT;
    application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    application_action.command.local_operation = mask == 0x7FU ?
        DEBUG_UI_OPERATION_STATIC_HOLD : DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    application_action.command.request_id = 501U;
    application_action.command.session_id = application_debug_ui.epoch;
    application_action.command.motor_mask = mask;
    application_action.command.hold_duration_us = 20000000ULL;
    application_action.command.mit_torque_ff_nm = takeover ? -1.0F : 0.0F;
    application_action.state = takeover ? AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT : AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index) {
        application_action.target_position_rad[joint_index] = 0.2F;
        application_action.command.static_reference_generations[joint_index] = application_debug_ui.reference_generation[joint_index];
    }
    aethor_app_static_load_begin(1000U);
    application_debug_ui.static_load.frames_issued_mask = mask;
    application_debug_ui.static_load.takeover_active = takeover;
}

/** @brief Publish one finite, identity-accepted synthetic sample through the actual motor bank. */
static MotorFeedbackSnapshot peak_feedback(uint8_t joint_index, uint64_t timestamp_us, float torque, float output_error)
{
    MotorJointFeedback feedback;
    MotorFeedbackSnapshot snapshot;
    memset(&feedback, 0, sizeof(feedback));
    feedback.timestamp_us = timestamp_us;
    feedback.position_rad = 0.2F + output_error * aethor_app_lcd_position_ratio(joint_index);
    feedback.torque_nm = torque;
    feedback.mos_temperature_c = 30.0F;
    feedback.rotor_temperature_c = 31.0F;
    feedback.driver_state = S3519_DRIVER_STATE_ENABLED;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        application_motor_runtime.configuration->joints[joint_index].master_id,
        application_motor_runtime.configuration->joints[joint_index].esc_id, &feedback) == MOTOR_BANK_STATUS_OK);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    return snapshot;
}

/** @brief A takeover failure must retain the violating sample although HOLD never began. */
static void peak_takeover_failure(void)
{
    MotorFeedbackSnapshot snapshot;
    peak_fixture(1U, 1U);
    snapshot = peak_feedback(0U, 672000U, 0.4469F, 0.6994F * AETHOR_APP_DEG_TO_RAD);
    aethor_app_static_load_sample(&snapshot, 672000U);
    if (application_debug_ui.static_load.samples[0] != 1U) {
        fputs("STATIC_PEAK_TAKEOVER_RED expected_samples=1 actual_samples=0\n", stderr);
        exit(1);
    }
    assert(fabsf(application_debug_ui.static_load.peak_torque_nm[0] - 0.4469F) < 0.00001F);
    assert(fabsf(application_debug_ui.static_load.peak_error_rad[0] - 0.6994F * AETHOR_APP_DEG_TO_RAD) < 0.00001F);
    assert(application_debug_ui.static_load.elapsed_ms == 0U);
    puts("STATIC_PEAK_TAKEOVER_PASS violating_sample=1 hold_not_started=1");
}

/** @brief Distinct accepted generations count even in one timestamp bin; polling and wrong scope do not. */
static void peak_generation_and_hold_clock(void)
{
    MotorFeedbackSnapshot snapshot;
    DebugUiStaticLoadReport *report;
    uint32_t generation;
    peak_fixture(1U, 1U);
    report = &application_debug_ui.static_load;
    snapshot = peak_feedback(0U, 4000U, 0.2F, 0.001F);
    aethor_app_static_load_sample(&snapshot, 4000U);
    aethor_app_static_load_sample(&snapshot, 4001U);
    assert(report->samples[0] == 1U && report->elapsed_ms == 0U);
    generation = application_motor_runtime.bank.motors[0].feedback_generation;
    /* Simulate two accepted publications with identical clock resolution. The real bank
     * currently rejects equal source timestamps; the report must still use generation. */
    ++application_motor_runtime.bank.motors[0].feedback_generation;
    application_motor_runtime.bank.motors[0].feedback.torque_nm = snapshot.joints[0].torque_nm = 0.3F;
    aethor_app_static_load_sample(&snapshot, 4001U);
    assert(report->sample_generation[0] != generation && report->samples[0] == 2U);
    snapshot = peak_feedback(1U, 5000U, 0.9F, 0.001F);
    aethor_app_static_load_sample(&snapshot, 5000U);
    assert(report->samples[0] == 2U && report->samples[1] == 0U);
    report->takeover_active = 0U; report->takeover_complete = 1U;
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT;
    aethor_app_static_load_begin(1001000U);
    assert(report->phase == DEBUG_UI_STATIC_LOAD_PHASE_HOLD && report->elapsed_ms == 0U);
    assert(report->samples[0] == 2U && fabsf(report->peak_torque_nm[0] - 0.3F) < 0.00001F);
    snapshot = peak_feedback(0U, 1005000U, 0.1F, 0.0001F);
    aethor_app_static_load_sample(&snapshot, 1005000U);
    assert(report->samples[0] == 3U && report->elapsed_ms == 4U);
    puts("STATIC_PEAK_GENERATION_PASS duplicate_poll=1 same_time_new_generation=1 unrelated_axis=1 hold_clock=1 takeover_peaks_preserved=1");
}

/** @brief An accepted generation cannot be attached to a report after the active command scope changes. */
static void peak_changed_scope_rejected(void)
{
    MotorFeedbackSnapshot snapshot;
    peak_fixture(1U, 1U);
    snapshot = peak_feedback(0U, 4000U, 0.2F, 0.001F);
    application_action.command.motor_mask = 2U;
    aethor_app_static_load_sample(&snapshot, 4000U);
    if (application_debug_ui.static_load.samples[0] != 0U) {
        fputs("STATIC_PEAK_SCOPE_RED changed_motor_mask_attaches_sample=1\n", stderr); exit(1);
    }
    application_action.command.motor_mask = 1U;
    aethor_app_static_load_sample(&snapshot, 4000U);
    assert(application_debug_ui.static_load.samples[0] == 1U);
    puts("STATIC_PEAK_SCOPE_PASS exact_motor_mask=1 unconsumed_valid_generation=1");
}

/** @brief Encode actual raw feedback accepted by the facade receive path, without fabricating a TX receipt. */
static void peak_receive_at(uint64_t lower_us, uint64_t observed_us, float torque)
{
    CanFrame encoded, received;
    const S3519Ranges *ranges = &application_motor_runtime.discovery.results[0].ranges;
    assert(s3519_pack_mit(1U, ranges, 0.2F, 0.0F, 0.0F, 0.0F, torque, &encoded) == S3519_CODEC_STATUS_OK);
    memset(&received, 0, sizeof(received));
    received.identifier = application_motor_runtime.configuration->joints[0].master_id;
    received.length = 8U;
    received.data[0] = (uint8_t)((S3519_DRIVER_STATE_ENABLED << 4U) | 1U);
    received.data[1] = encoded.data[0]; received.data[2] = encoded.data[1]; received.data[3] = encoded.data[2];
    received.data[4] = (uint8_t)((encoded.data[3] & 0xF0U) | (encoded.data[6] & 0x0FU));
    received.data[5] = encoded.data[7]; received.data[6] = 30U; received.data[7] = 31U;
    assert(aethor_app_receive_static_frame(&received, lower_us, observed_us, NULL, 0U, 0U) == MOTOR_RUNTIME_STATUS_OK);
}

/** @brief Keep ordinary fixture calls in one timestamp domain while retaining the raw receive path. */
static void peak_receive(uint64_t timestamp_us, float torque)
{
    peak_receive_at(timestamp_us, timestamp_us, torque);
}

/** @brief A violating raw RX is sampled before safety; later safe RX, disabled ACK and independent STOP retain it. */
static void peak_violating_receive_and_stop(void)
{
    DebugUiStaticLoadReport saved;
    ProtocolCommand stop;
    ProtocolCommandResult terminal;
    MotorFeedbackSnapshot snapshot;
    peak_fixture(1U, 1U);
    peak_receive(4000U, 4.0F);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
    assert(application_debug_ui.static_load.samples[0] == 1U);
    assert(application_debug_ui.static_load.peak_torque_nm[0] > 3.99F);
    assert(application_debug_ui.static_load.first_exit_latched);
    assert(application_debug_ui.static_load.first_exit_reason == DEBUG_UI_REASON_TORQUE_LIMIT);
    assert(application_debug_ui.static_load.first_exit_phase == DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER);
    saved = application_debug_ui.static_load;
    peak_receive(5000U, 0.1F);
    assert(application_debug_ui.static_load.samples[0] == saved.samples[0]);
    snapshot = peak_feedback(0U, 6000U, 0.0F, 0.0F);
    snapshot.joints[0].driver_state = S3519_DRIVER_STATE_DISABLED;
    aethor_app_static_load_sample(&snapshot, 6000U);
    memset(&stop, 0, sizeof(stop));
    stop.type = PROTOCOL_COMMAND_STOP; stop.local_operation = DEBUG_UI_OPERATION_STOP;
    stop.origin = DEBUG_UI_ORIGIN_LOCAL_UI; stop.session_id = saved.identity.epoch;
    stop.request_id = 502U; stop.motor_mask = 1U;
    aethor_app_static_load_bind_stop(&stop);
    memset(&terminal, 0, sizeof(terminal));
    terminal.origin = saved.identity.origin; terminal.session_id = saved.identity.epoch;
    terminal.request_id = saved.identity.request_id; terminal.local_operation = saved.operation;
    terminal.motor_mask = saved.motor_mask; terminal.code = PROTOCOL_COMMAND_RESULT_FAILED;
    terminal.local_reason = DEBUG_UI_REASON_UNHEALTHY;
    terminal.disabled_confirmed = 1U;
    aethor_app_static_load_finish(&terminal);
    application_action.command = stop;
    terminal.request_id = stop.request_id; terminal.local_operation = stop.local_operation;
    terminal.code = PROTOCOL_COMMAND_RESULT_STOPPED; terminal.local_reason = DEBUG_UI_REASON_NONE;
    aethor_app_static_load_finish(&terminal);
    assert(application_debug_ui.static_load.stopped && application_debug_ui.static_load.disabled_confirmed);
    assert(application_debug_ui.static_load.first_exit_reason == DEBUG_UI_REASON_TORQUE_LIMIT);
    assert(application_debug_ui.static_load.first_exit_phase == DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER);
    assert(application_debug_ui.static_load.samples[0] == saved.samples[0]);
    assert(application_debug_ui.static_load.peak_torque_nm[0] == saved.peak_torque_nm[0]);
    puts("STATIC_PEAK_RX_FIRST_EXIT_PASS violating_raw_rx=1 before_safety=1 later_normal_disabled=1 cleanup_health=1 independent_stop=1");
}

/** @brief Every accepted HOLD RX is observed even when multiple frames arrive before one UI/control poll. */
static void peak_hold_received_generations(void)
{
    peak_fixture(1U, 0U);
    peak_receive(4000U, 0.7F);
    peak_receive(5000U, 0.2F);
    assert(application_debug_ui.static_load.samples[0] == 2U);
    assert(application_debug_ui.static_load.peak_torque_nm[0] > 0.69F);
    assert(application_debug_ui.static_load.elapsed_ms == 4U);
    peak_fixture(1U, 0U);
    peak_receive_at(4000U, 5000U, 0.2F);
    peak_receive_at(4500U, 5000U, 0.7F);
    assert(application_debug_ui.static_load.samples[0] == 2U);
    assert(application_debug_ui.static_load.peak_torque_nm[0] > 0.69F);
    assert(!application_debug_ui.static_load.first_exit_latched);
    puts("STATIC_PEAK_HOLD_RX_PASS multiple_before_poll=1 peaks=1 same_observed_time_distinct_accepted_generations=1");
}

/** @brief HOLD must retain a violating accepted RX and its first cause before a later normal frame arrives. */
static void peak_hold_violating_receive(void)
{
    peak_fixture(1U, 0U);
    peak_receive(4000U, 4.0F);
    peak_receive(5000U, 0.1F);
    if (!application_debug_ui.static_load.first_exit_latched) {
        fputs("STATIC_PEAK_HOLD_RX_RED violating_sample_peak_without_first_cause=1\n", stderr); exit(1);
    }
    assert(application_debug_ui.static_load.first_exit_reason == DEBUG_UI_REASON_TORQUE_LIMIT);
    assert(application_debug_ui.static_load.first_exit_phase == DEBUG_UI_STATIC_LOAD_PHASE_HOLD);
    assert(application_debug_ui.static_load.samples[0] == 1U);
    assert(application_debug_ui.static_load.peak_torque_nm[0] > 3.99F);
    puts("STATIC_PEAK_HOLD_RX_GUARD_PASS violating_raw_rx=1 later_normal_cannot_hide=1 first_hold_cause=1");
}

/** @brief The existing seven-axis operation retains all axes, distinct counts, scope and HOLD time. */
static void peak_seven_axis_regression(void)
{
    MotorFeedbackSnapshot snapshot;
    uint8_t joint_index;
    peak_fixture(0x7FU, 0U);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index) {
        snapshot = peak_feedback(joint_index, 4000U, (float)(joint_index + 1U) * 0.1F, 0.001F);
        aethor_app_static_load_sample(&snapshot, 4000U);
    }
    aethor_app_static_load_sample(&snapshot, 4001U);
    assert(application_debug_ui.static_load.valid_mask == 0x7FU);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        assert(application_debug_ui.static_load.samples[joint_index] == 1U);
    assert(application_debug_ui.static_load.elapsed_ms == 3U);
    assert(application_debug_ui.static_load.phase == DEBUG_UI_STATIC_LOAD_PHASE_HOLD);
    puts("STATIC_PEAK_SEVEN_AXIS_PASS axes=7 generation_counting=1 hold_clock=1");
}

/** @brief A saturated emergency STOP latches its first cause without nesting the caller's critical section. */
static void peak_saturated_stop_first_cause(void)
{
    peak_fixture(1U, 1U);
    aethor_app_enter_task_critical();
    aethor_app_static_load_bind_fallback(5000U);
    aethor_app_exit_task_critical();
    if (!application_debug_ui.static_load.first_exit_latched) {
        fputs("STATIC_PEAK_FALLBACK_RED saturated_stop_has_no_first_cause=1\n", stderr); exit(1);
    }
    assert(application_debug_ui.static_load.first_exit_reason == DEBUG_UI_REASON_STOP_LATCHED);
    assert(application_debug_ui.static_load.first_exit_phase == DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER);
    assert(application_debug_ui.static_load.first_exit_code == DEBUG_UI_STOPPED);
    puts("STATIC_PEAK_FALLBACK_PASS saturated_stop_cause=1 no_nested_critical=1");
}

/** @brief A later independent STOP cannot reclassify an already completed and disabled historical trial. */
static void peak_completed_history_survives_stop(void)
{
    MotorFeedbackSnapshot measured;
    ProtocolCommandResult terminal;
    ProtocolCommand stop;
    DebugUiStaticLoadReport saved;
    peak_fixture(1U, 0U);
    measured = peak_feedback(0U, 4000U, 0.4F, 0.001F);
    aethor_app_static_load_sample(&measured, 4000U);
    memset(&terminal, 0, sizeof(terminal));
    terminal.origin = application_action.command.origin;
    terminal.session_id = application_action.command.session_id;
    terminal.request_id = application_action.command.request_id;
    terminal.local_operation = application_action.command.local_operation;
    terminal.motor_mask = 1U; terminal.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
    terminal.disabled_confirmed = 1U;
    aethor_app_static_load_finish(&terminal);
    assert(application_debug_ui.static_load.complete);
    saved = application_debug_ui.static_load;
    memset(&stop, 0, sizeof(stop));
    stop.type = PROTOCOL_COMMAND_STOP; stop.local_operation = DEBUG_UI_OPERATION_STOP;
    stop.origin = saved.identity.origin; stop.session_id = saved.identity.epoch;
    stop.request_id = 503U; stop.motor_mask = 1U;
    aethor_app_static_load_bind_stop(&stop);
    terminal.local_operation = DEBUG_UI_OPERATION_STOP; terminal.request_id = stop.request_id;
    terminal.code = PROTOCOL_COMMAND_RESULT_STOPPED;
    aethor_app_static_load_finish(&terminal);
    if (memcmp(&saved, &application_debug_ui.static_load, sizeof(saved)) != 0) {
        fputs("STATIC_PEAK_HISTORY_STOP_RED completed_trial_reclassified_by_independent_stop=1\n", stderr); exit(1);
    }
    puts("STATIC_PEAK_HISTORY_STOP_PASS completed_report_immutable=1 independent_stop=1");
}

/** @brief A single exact takeover-to-HOLD transition binds the new HOLD clock without weakening later ordering. */
static void peak_ledger_hold_transition(void)
{
    DebugUiLoadTrials ledger;
    DebugUiSnapshot snapshot;
    DebugUiRequest request;
    const DebugUiLoadTrialRecord *record;
    MotorFeedbackSnapshot measured;
    peak_fixture(1U, 1U);
    debug_ui_load_trials_init(&ledger);
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.timestamp_us = 500U; snapshot.boot_id = 77U; snapshot.epoch = application_debug_ui.epoch;
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    memset(&request, 0, sizeof(request));
    request.identity = application_debug_ui.static_load.identity;
    request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    request.target_motor_id = 1U; request.created_at_us = 500U; request.hold_duration_ms = 20000U;
    assert(debug_ui_load_trials_freeze(&ledger, 1U));
    assert(debug_ui_load_trials_submit(&ledger, &request));
    snapshot.static_load = application_debug_ui.static_load;
    snapshot.timestamp_us = 2000U;
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    record = debug_ui_load_trials_newest(&ledger, 0U);
    assert(record->report_seen && record->elapsed_ms == 0U);
    application_debug_ui.static_load.takeover_active = 0U;
    application_debug_ui.static_load.takeover_complete = 1U;
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT;
    aethor_app_static_load_begin(1001000U);
    measured = peak_feedback(0U, 1005000U, 0.2F, 0.001F);
    aethor_app_static_load_sample(&measured, 1005000U);
    snapshot.static_load = application_debug_ui.static_load;
    snapshot.timestamp_us = 1005000U;
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    if (record->elapsed_ms != 4U) {
        fputs("STATIC_PEAK_LEDGER_CLOCK_RED takeover_bound_start_rejects_hold_transition=1\n", stderr);
        exit(1);
    }
    assert(record->report_started_at_us == 1001000U && record->measurements_valid);
    snapshot.static_load.started_at_us += 1000U; snapshot.static_load.elapsed_ms = 9U;
    snapshot.timestamp_us += 10000U;
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    assert(record->elapsed_ms == 4U && record->report_started_at_us == 1001000U);
    snapshot.static_load = application_debug_ui.static_load;
    snapshot.static_load.phase = DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER;
    snapshot.static_load.started_at_us = 1000U; snapshot.static_load.elapsed_ms = 0U;
    snapshot.timestamp_us += 10000U;
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    assert(record->elapsed_ms == 4U);
    puts("STATIC_PEAK_LEDGER_CLOCK_PASS takeover_zero_elapsed=1 one_forward_transition=1 changed_hold_start_rejected=1 phase_regression_rejected=1");
}

/** @brief Run only hardware-independent peak observations in this executable. */
int main(void)
{
    peak_takeover_failure();
    peak_generation_and_hold_clock();
    peak_changed_scope_rejected();
    peak_violating_receive_and_stop();
    peak_hold_received_generations();
    peak_seven_axis_regression();
    peak_ledger_hold_transition();
    peak_saturated_stop_first_cause();
    peak_hold_violating_receive();
    peak_completed_history_survives_stop();
    return 0;
}
