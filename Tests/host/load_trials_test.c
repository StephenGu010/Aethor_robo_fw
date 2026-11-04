/** @file load_trials_test.c
 * @brief Strict C99 synthetic tests for display-only load metadata and trial history.
 * Application integration uses synthetic fixtures; no serial, probe or physical hardware is accessed.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "debug_ui_load_trials.h"
#ifdef LOAD_TRIAL_APP_ORDER_TEST
#define main existing_application_fixture_main
#include "debug_ui_app_test_main.c"
#undef main
#include "debug_ui_model.h"
#elif defined(LOAD_TRIAL_MODEL_HOOKS_TEST)
#define main existing_model_fixture_main
#include "debug_ui_model_test_main.c"
#undef main
#else
#include "../../App/DebugUi/debug_ui_load_trials.inc"
#endif

#ifdef LOAD_TRIAL_APP_ORDER_TEST
/** @brief Reserve a real mailbox request and start a production report from synthetic HOLD feedback. */
static DebugUiRequest trial_app_fixture(DebugUiModel *model)
{
    DebugUiRequest request;
    DebugUiSnapshot snapshot;
    MotorFeedbackSnapshot measured;
    test_fixture();
    test_acquire();
    request = test_request(DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, 2000ULL);
    request.hold_duration_ms = 1000U;
    debug_ui_model_init(model);
    assert(aethor_app_debug_ui_get_snapshot(2000ULL, &snapshot));
    debug_ui_model_update(model, &snapshot, 2000ULL, 1U, 1U);
    assert(debug_ui_load_trials_freeze(&model->load_trials, 1U));
    assert(debug_ui_load_trials_submit(&model->load_trials, &request));
    model->submitted = request;
    model->awaiting_result = 1U;
    assert(debug_ui_mailbox_submit(&application_debug_ui.mailbox, &request) == DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_build_command(&request, 2000ULL, &application_action.command);
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT;
    application_action.target_position_rad[0] = 0.2F;
    aethor_app_static_load_begin(3000ULL);
    application_debug_ui.static_load.frames_issued_mask = 1U;
    memset(&measured, 0, sizeof(measured));
    measured.valid_joint_mask = 1U;
    measured.joints[0].timestamp_us = 11000ULL;
    measured.joints[0].position_rad = 0.21F;
    measured.joints[0].torque_nm = 0.75F;
    measured.joints[0].mos_temperature_c = 31.0F;
    measured.joints[0].rotor_temperature_c = 32.0F;
    measured.joints[0].driver_state = S3519_DRIVER_STATE_ENABLED;
    /* Observation counts accepted motor-bank generations, never an unaccepted hand-built snapshot. */
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        application_motor_runtime.configuration->joints[0].master_id,
        application_motor_runtime.configuration->joints[0].esc_id, &measured.joints[0]) == MOTOR_BANK_STATUS_OK);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 11000ULL,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &measured) == MOTOR_RUNTIME_STATUS_OK);
    aethor_app_static_load_sample(&measured, 11000ULL);
    return request;
}

/** @brief Route one real local result through the production mailbox and authority revocation path. */
static void trial_app_route_terminal(const DebugUiRequest *request, ProtocolCommandResultCode code,
                                      DebugUiReason reason, uint64_t timestamp_us, uint8_t disabled)
{
    ProtocolCommandResult result;
    memset(&result, 0, sizeof(result));
    result.type = request->operation == DEBUG_UI_OPERATION_STOP ? PROTOCOL_COMMAND_STOP : PROTOCOL_COMMAND_MIT_ACTION;
    result.origin = request->identity.origin;
    result.session_id = request->identity.epoch;
    result.request_id = request->identity.request_id;
    result.local_operation = request->operation;
    result.motor_mask = 1U;
    result.code = code;
    result.local_reason = reason;
    result.stage = PROTOCOL_COMMAND_STAGE_DISABLE;
    result.completed_at_us = timestamp_us;
    result.disabled_confirmed = disabled;
    assert(protocol_engine_submit_command_result(&application_protocol_engine, &result));
    aethor_app_debug_ui_route_results();
}

/** @brief UiTask publishes the newer authority snapshot before polling the failed HOLD completion. */
static void test_app_snapshot_before_failed_completion(void)
{
    DebugUiModel model;
    DebugUiRequest request = trial_app_fixture(&model);
    DebugUiSnapshot snapshot;
    DebugUiCompletion completion;
    const DebugUiLoadTrialRecord *record;
    trial_app_route_terminal(&request, PROTOCOL_COMMAND_RESULT_FAILED, DEBUG_UI_REASON_TORQUE_LIMIT, 12000ULL, 0U);
    assert(aethor_app_debug_ui_get_snapshot(13000ULL, &snapshot));
    assert(snapshot.epoch > request.identity.epoch && snapshot.static_load.identity.epoch == request.identity.epoch);
    debug_ui_model_update(&model, &snapshot, 13000ULL, 1U, 1U);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    debug_ui_model_completion(&model, &completion);
    record = debug_ui_load_trials_newest(&model.load_trials, 0U);
    printf("LOAD_TRIAL_APP_EPOCH_OBSERVED request_epoch=%lu snapshot_epoch=%lu received_reason=%u model_reason=%u ledger_seen=%u ledger_reason=%u report_seen=%u\n",
        (unsigned long)request.identity.epoch, (unsigned long)snapshot.epoch, (unsigned)completion.reason,
        (unsigned)model.reason, record->hold.seen, record->hold.reason, record->report_seen);
    fflush(stdout);
    assert(record->hold.seen && record->hold.reason == DEBUG_UI_REASON_TORQUE_LIMIT);
    assert(record->report_seen && record->measurements_valid && record->samples == 1U && record->elapsed_ms == 8U);
    assert(record->peak_torque_nm == 0.75F && !record->disabled_confirmed);
    assert(debug_ui_load_trials_freeze(&model.load_trials, 1U));
    assert(!debug_ui_load_trials_submit(&model.load_trials, &request) && model.load_trials.count == 1U);
    puts("LOAD_TRIAL_APP_SNAPSHOT_FIRST_PASS real_route=1 revoke_before_poll=1 final_report=1 stale_submit_denied=1");
}

/** @brief A new-epoch STOP explicitly binds an older active HOLD; either delivery order retains cleanup. */
static void test_app_cross_epoch_stop_cleanup(void)
{
    unsigned stop_first;
    for (stop_first = 0U; stop_first < 2U; ++stop_first) {
        DebugUiModel model;
        DebugUiRequest request = trial_app_fixture(&model), stop;
        DebugUiSnapshot snapshot;
        DebugUiCompletion completion;
        ProtocolCommand stop_command;
        const DebugUiLoadTrialRecord *record;
        aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT);
        assert(aethor_app_debug_ui_get_snapshot(12000ULL, &snapshot));
        debug_ui_model_update(&model, &snapshot, 12000ULL, 1U, 1U);
        stop = test_request(DEBUG_UI_OPERATION_STOP, 13000ULL);
        assert(stop.identity.epoch > request.identity.epoch);
        model.outgoing_stop = stop;
        model.stop_waiting = model.stop_requested = 1U;
        assert(debug_ui_mailbox_submit(&application_debug_ui.mailbox, &stop) == DEBUG_UI_REASON_NONE);
        aethor_app_debug_ui_build_command(&stop, 13000ULL, &stop_command);
        aethor_app_static_load_bind_stop(&stop_command);
        assert(application_debug_ui.static_load.stop_identity.epoch == stop.identity.epoch);
        trial_app_route_terminal(&stop, PROTOCOL_COMMAND_RESULT_STOPPED, DEBUG_UI_REASON_NONE, 14000ULL, 1U);
        if (stop_first) {
            assert(aethor_app_debug_ui_poll_completion(&completion));
            debug_ui_model_completion(&model, &completion);
            assert(model.load_trials.pending_stop_seen);
            aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT);
        }
        assert(aethor_app_debug_ui_get_snapshot(15000ULL, &snapshot));
        debug_ui_model_update(&model, &snapshot, 15000ULL, 1U, 1U);
        if (!stop_first) {
            assert(aethor_app_debug_ui_poll_completion(&completion));
            debug_ui_model_completion(&model, &completion);
        }
        record = debug_ui_load_trials_newest(&model.load_trials, 0U);
        assert(record->stop.seen && record->stop.code == DEBUG_UI_STOPPED && record->disabled_confirmed);
        assert(record->stop_identity.epoch == stop.identity.epoch && !model.load_trials.pending_stop_seen);
        assert(record->identity.epoch == request.identity.epoch && record->metadata.lever_mm == 250U);
        assert(model.load_trials.count == 1U);
    }
    puts("LOAD_TRIAL_APP_CROSS_EPOCH_STOP_PASS report_explicit_identity=1 delivery_orders=2 pending_survives_revoke=1");
}
#endif

/** @brief Create one immutable, single-axis request with a unique local identity. */
static DebugUiRequest trial_request(uint32_t request_id)
{
    DebugUiRequest request;
    memset(&request, 0, sizeof(request));
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    request.identity.epoch = 4U;
    request.identity.request_id = request_id;
    request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    request.target_motor_id = 2U;
    request.created_at_us = 1000000ULL + request_id * 1000ULL;
    request.hold_duration_ms = 60000U;
    return request;
}

/** @brief Prepare a fresh report that exactly names one submitted trial. */
static DebugUiSnapshot trial_snapshot(const DebugUiRequest *request)
{
    DebugUiSnapshot snapshot;
    DebugUiStaticLoadReport *report;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.boot_id = 9U;
    snapshot.epoch = request->identity.epoch;
    snapshot.timestamp_us = request->created_at_us + 100000ULL;
    snapshot.generation = request->identity.request_id;
    report = &snapshot.static_load;
    report->identity = request->identity;
    report->operation = request->operation;
    report->motor_mask = 2U;
    report->started_at_us = request->created_at_us + 1000ULL;
    report->duration_ms = request->hold_duration_ms;
    report->elapsed_ms = 99U;
    report->valid_mask = 2U;
    report->samples[1] = 5U;
    report->peak_error_rad[1] = 0.02F;
    report->peak_torque_nm[1] = 0.3F;
    report->peak_mos_c[1] = 31.0F;
    report->peak_rotor_c[1] = 32.0F;
    return snapshot;
}

/** @brief Initialize application epoch evidence, freeze metadata, then bind one actual request. */
static void trial_submit(DebugUiLoadTrials *ledger, const DebugUiRequest *request)
{
    DebugUiSnapshot initial = trial_snapshot(request);
    initial.timestamp_us = request->created_at_us - 1ULL;
    memset(&initial.static_load, 0, sizeof(initial.static_load));
    debug_ui_load_trials_observe(ledger, &initial, initial.timestamp_us);
    assert(debug_ui_load_trials_freeze(ledger, request->target_motor_id));
    assert(debug_ui_load_trials_submit(ledger, request));
}

/** @brief Build a terminal with exact identity, operation and motor execution scope. */
static DebugUiCompletion trial_completion(const DebugUiRequest *request)
{
    DebugUiCompletion completion;
    memset(&completion, 0, sizeof(completion));
    completion.identity = request->identity;
    completion.operation = request->operation;
    completion.motor_mask = 2U;
    completion.timestamp_us = request->created_at_us + 110000ULL;
    completion.code = DEBUG_UI_FAILED;
    completion.reason = DEBUG_UI_REASON_TORQUE_LIMIT;
    completion.stage = DEBUG_UI_STAGE_DISABLE;
    completion.detail = 25U;
    completion.error = 3U;
    completion.failed_motor_id = 2U;
    return completion;
}

/** @brief Defaults, bounds and confirmation are independent for each one-based motor ID. */
static void test_metadata_and_calculation(void)
{
    DebugUiLoadTrials ledger;
    const DebugUiLoadMetadata *metadata;
    unsigned motor_id;
    debug_ui_load_trials_init(&ledger);
    assert(sizeof(ledger) <= DEBUG_UI_LOAD_TRIALS_MAX_BYTES);
    for (motor_id = 1U; motor_id <= DEBUG_UI_MOTOR_COUNT; ++motor_id) {
        metadata = debug_ui_load_trials_candidate(&ledger, (uint8_t)motor_id);
        assert(metadata != NULL && metadata->lever_mm == 250U && metadata->mass_g == 0U);
        assert(!metadata->confirmed && debug_ui_load_trials_gravity_nm(metadata) == 0.0F);
    }
    assert(!debug_ui_load_trials_candidate(&ledger, 0U));
    assert(!debug_ui_load_trials_candidate(&ledger, 8U));
    assert(!debug_ui_load_trials_set(&ledger, 0U, 250U, 100U));
    assert(!debug_ui_load_trials_set(&ledger, 8U, 250U, 100U));
    assert(!debug_ui_load_trials_set(&ledger, 2U, 0U, 0U));
    assert(!debug_ui_load_trials_set(&ledger, 2U, 2001U, 0U));
    assert(!debug_ui_load_trials_set(&ledger, 2U, 250U, 10001U));
    assert(!debug_ui_load_trials_confirm(&ledger, 0U));
    assert(!debug_ui_load_trials_confirm(&ledger, 8U));
    assert(debug_ui_load_trials_set(&ledger, 2U, 250U, 100U));
    metadata = debug_ui_load_trials_candidate(&ledger, 2U);
    assert(fabsf(debug_ui_load_trials_gravity_nm(metadata) - 0.24525F) < 0.00001F);
    assert(debug_ui_load_trials_confirm(&ledger, 2U) && metadata->confirmed);
    assert(debug_ui_load_trials_set(&ledger, 2U, 251U, 100U) && !metadata->confirmed);
    assert(debug_ui_load_trials_candidate(&ledger, 1U)->mass_g == 0U);
    assert(debug_ui_load_trials_set(&ledger, 2U, 1U, 0U));
    assert(debug_ui_load_trials_set(&ledger, 2U, 2000U, 10000U));
    assert(fabsf(debug_ui_load_trials_gravity_nm(metadata) - 196.2F) < 0.001F);
    assert(ledger.count == 0U);
    puts("LOAD_TRIAL_METADATA_PASS ids=7 bounds=1 zero=1 gravity=0.24525,196.2");
}

/** @brief Review metadata is frozen before identity assignment and never alters requests. */
static void test_immutable_freeze_and_submitted_identity(void)
{
    DebugUiLoadTrials ledger;
    DebugUiRequest request = trial_request(1U), unchanged = request;
    const DebugUiLoadTrialRecord *record;
    debug_ui_load_trials_init(&ledger);
    assert(!debug_ui_load_trials_freeze(&ledger, 0U));
    assert(!debug_ui_load_trials_submit(&ledger, &request));
    assert(debug_ui_load_trials_set(&ledger, 2U, 250U, 100U));
    assert(debug_ui_load_trials_confirm(&ledger, 2U));
    assert(debug_ui_load_trials_freeze(&ledger, 2U));
    assert(debug_ui_load_trials_set(&ledger, 2U, 300U, 200U));
    assert(debug_ui_load_trials_submit(&ledger, &request));
    assert(memcmp(&unchanged, &request, sizeof(request)) == 0);
    record = debug_ui_load_trials_newest(&ledger, 0U);
    assert(record && record->metadata.lever_mm == 250U && record->metadata.mass_g == 100U);
    assert(record->metadata.confirmed && record->target_motor_id == 2U);
    assert(record->identity.request_id == request.identity.request_id && record->duration_ms == 60000U);
    assert(record->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
    assert(!record->report_seen && !record->hold.seen && !record->disabled_known);
    assert(!debug_ui_load_trials_submit(&ledger, &request) && ledger.count == 1U);
    assert(debug_ui_load_trials_set(&ledger, 2U, 400U, 500U));
    assert(record->metadata.mass_g == 100U && record->metadata.confirmed);
    assert(!debug_ui_load_trials_newest(&ledger, 1U));
    request = trial_request(2U);
    assert(debug_ui_load_trials_freeze(&ledger, 1U));
    assert(!debug_ui_load_trials_submit(&ledger, &request));
    assert(debug_ui_load_trials_freeze(&ledger, 2U));
    request.operation = DEBUG_UI_OPERATION_STATIC_HOLD;
    assert(!debug_ui_load_trials_submit(&ledger, &request));
    puts("LOAD_TRIAL_FREEZE_PASS immutable=1 typed_request_unchanged=1 single_axis_only=1");
}

/** @brief Every independent scope/freshness mismatch keeps all report data unknown. */
static void test_report_scope_and_freshness(void)
{
    unsigned mismatch;
    for (mismatch = 0U; mismatch < 12U; ++mismatch) {
        DebugUiLoadTrials ledger;
        DebugUiRequest request = trial_request(3U);
        DebugUiSnapshot snapshot = trial_snapshot(&request);
        uint64_t now_us = snapshot.timestamp_us;
        debug_ui_load_trials_init(&ledger);
        trial_submit(&ledger, &request);
        switch (mismatch) {
        case 0U: snapshot.static_load.identity.origin = DEBUG_UI_ORIGIN_USB; break;
        case 1U: ++snapshot.static_load.identity.epoch; break;
        case 2U: ++snapshot.static_load.identity.request_id; break;
        case 3U: snapshot.static_load.operation = DEBUG_UI_OPERATION_STATIC_HOLD; break;
        case 4U: snapshot.static_load.motor_mask = 1U; break;
        case 5U: ++snapshot.static_load.duration_ms; break;
        case 6U: snapshot.timestamp_us = request.created_at_us - 1ULL; break;
        case 7U: now_us += DEBUG_UI_HEALTH_TIMEOUT_US; break;
        case 8U: --snapshot.epoch; break;
        case 9U: snapshot.static_load.started_at_us = request.created_at_us - 1ULL; break;
        case 10U: snapshot.static_load.started_at_us = snapshot.timestamp_us + 1ULL; break;
        default: now_us = snapshot.timestamp_us - 1ULL; break;
        }
        debug_ui_load_trials_observe(&ledger, &snapshot, now_us);
        assert(!debug_ui_load_trials_newest(&ledger, 0U)->report_seen);
    }
    puts("LOAD_TRIAL_SCOPE_PASS mismatches=12 fresh_snapshot_required=1");
}

/** @brief Invalid sample evidence cannot make peak values valid, while timing stays explicit. */
static void test_measurement_evidence_and_monotonic_snapshots(void)
{
    unsigned invalid;
    for (invalid = 0U; invalid < 6U; ++invalid) {
        DebugUiLoadTrials ledger;
        DebugUiRequest request = trial_request(4U);
        DebugUiSnapshot snapshot = trial_snapshot(&request);
        const DebugUiLoadTrialRecord *record;
        debug_ui_load_trials_init(&ledger);
        trial_submit(&ledger, &request);
        if (invalid == 0U) snapshot.static_load.valid_mask = 1U;
        else if (invalid == 1U) snapshot.static_load.samples[1] = 0U;
        else if (invalid == 2U) snapshot.static_load.peak_error_rad[1] = NAN;
        else if (invalid == 3U) snapshot.static_load.peak_torque_nm[1] = INFINITY;
        else if (invalid == 4U) snapshot.static_load.peak_mos_c[1] = -INFINITY;
        else snapshot.static_load.peak_rotor_c[1] = NAN;
        debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
        record = debug_ui_load_trials_newest(&ledger, 0U);
        assert(record->report_seen && !record->measurements_valid && record->elapsed_ms == 99U);
    }
    {
        DebugUiLoadTrials ledger;
        DebugUiRequest request = trial_request(5U);
        DebugUiSnapshot snapshot = trial_snapshot(&request);
        const DebugUiLoadTrialRecord *record;
        debug_ui_load_trials_init(&ledger);
        trial_submit(&ledger, &request);
        debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
        record = debug_ui_load_trials_newest(&ledger, 0U);
        assert(record->measurements_valid && record->samples == 5U && record->elapsed_ms == 99U);
        assert(record->peak_error_rad == 0.02F && record->peak_torque_nm == 0.3F);
        snapshot.static_load.elapsed_ms = 1U;
        snapshot.static_load.peak_torque_nm[1] = 99.0F;
        debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
        assert(record->elapsed_ms == 99U && record->peak_torque_nm == 0.3F);
        ++snapshot.timestamp_us; ++snapshot.generation;
        debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
        assert(record->elapsed_ms == 99U && record->peak_torque_nm == 0.3F);
    }
    puts("LOAD_TRIAL_MEASUREMENTS_PASS finite=1 sampled=1 monotonic=1 timing_is_application_only=1");
}

/** @brief Refusals and actual terminals preserve their real causes without synthetic success. */
static void test_rejection_and_hold_terminals(void)
{
    DebugUiLoadTrials ledger;
    DebugUiRequest request = trial_request(6U);
    DebugUiCompletion completion = trial_completion(&request), mismatch;
    const DebugUiLoadTrialRecord *record;
    unsigned dimension;
    debug_ui_load_trials_init(&ledger);
    trial_submit(&ledger, &request);
    record = debug_ui_load_trials_newest(&ledger, 0U);
    for (dimension = 0U; dimension < 6U; ++dimension) {
        mismatch = completion;
        if (dimension == 0U) mismatch.identity.origin = DEBUG_UI_ORIGIN_USB;
        else if (dimension == 1U) ++mismatch.identity.epoch;
        else if (dimension == 2U) ++mismatch.identity.request_id;
        else if (dimension == 3U) mismatch.operation = DEBUG_UI_OPERATION_MIT_HOLD;
        else if (dimension == 4U) mismatch.motor_mask = 1U;
        else mismatch.timestamp_us = request.created_at_us - 1ULL;
        debug_ui_load_trials_completion(&ledger, &mismatch, NULL);
        assert(!record->hold.seen);
    }
    debug_ui_load_trials_refused(&ledger, &request, DEBUG_UI_REASON_BUSY);
    assert(ledger.count == 1U && record->hold.seen && record->submit_refused);
    assert(record->hold.code == DEBUG_UI_FAILED && record->hold.reason == DEBUG_UI_REASON_BUSY);
    assert(!record->report_seen && !record->disabled_known);
    debug_ui_load_trials_completion(&ledger, &completion, NULL);
    assert(record->hold.reason == DEBUG_UI_REASON_BUSY);
    request = trial_request(7U);
    trial_submit(&ledger, &request);
    completion = trial_completion(&request);
    debug_ui_load_trials_completion(&ledger, &completion, NULL);
    record = debug_ui_load_trials_newest(&ledger, 0U);
    assert(record->hold.reason == DEBUG_UI_REASON_TORQUE_LIMIT && record->hold.detail == 25U);
    assert(record->hold.error == 3U && record->hold.failed_motor_id == 2U);
    assert(record->disabled_known && !record->disabled_confirmed && !record->report_seen);
    completion.reason = DEBUG_UI_REASON_NONE; completion.disabled_confirmed = 1U;
    debug_ui_load_trials_completion(&ledger, &completion, NULL);
    assert(record->hold.reason == DEBUG_UI_REASON_TORQUE_LIMIT && !record->disabled_confirmed);
    puts("LOAD_TRIAL_TERMINALS_PASS refused=1 exact_hold_scope=1 causes_preserved=1 duplicates_ignored=1");
}

/** @brief Explicit report STOP provenance works before or after HOLD terminals in either order. */
static void test_stop_binding_and_delivery_orders(void)
{
    unsigned order, mismatch;
    for (order = 0U; order < 4U; ++order) {
        DebugUiLoadTrials ledger;
        DebugUiRequest request = trial_request(8U), stop = trial_request(9U);
        DebugUiSnapshot snapshot = trial_snapshot(&request);
        DebugUiCompletion hold = trial_completion(&request), stopped;
        const DebugUiLoadTrialRecord *record;
        debug_ui_load_trials_init(&ledger);
        trial_submit(&ledger, &request);
        stop.operation = DEBUG_UI_OPERATION_STOP;
        stopped = trial_completion(&stop);
        stopped.code = DEBUG_UI_STOPPED;
        stopped.reason = order < 2U ? DEBUG_UI_REASON_NONE : DEBUG_UI_REASON_NOT_DISABLED;
        stopped.disabled_confirmed = (uint8_t)(order < 2U);
        snapshot.static_load.stop_identity = stop.identity;
        snapshot.static_load.complete = snapshot.static_load.stopped = 1U;
        record = debug_ui_load_trials_newest(&ledger, 0U);
        for (mismatch = 0U; mismatch < 4U; ++mismatch) {
            DebugUiCompletion wrong = stopped;
            if (mismatch == 0U) wrong.identity.origin = DEBUG_UI_ORIGIN_USB;
            else if (mismatch == 1U) ++wrong.identity.epoch;
            else if (mismatch == 2U) wrong.motor_mask = 1U;
            else ++wrong.identity.request_id;
            debug_ui_load_trials_completion(&ledger, &wrong, &stop);
            assert(!ledger.pending_stop_seen && !record->stop.seen);
        }
        if ((order & 1U) == 0U) {
            debug_ui_load_trials_completion(&ledger, &stopped, &stop);
            assert(ledger.pending_stop_seen && !record->stop.seen);
            debug_ui_load_trials_completion(&ledger, &hold, NULL);
            debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
        } else {
            debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
            debug_ui_load_trials_completion(&ledger, &stopped, &stop);
            debug_ui_load_trials_completion(&ledger, &hold, NULL);
        }
        assert(record->stop.seen && record->hold.seen && record->disabled_known);
        assert(record->disabled_confirmed == (uint8_t)(order < 2U));
        assert(record->hold.reason == DEBUG_UI_REASON_TORQUE_LIMIT && record->stop.code == DEBUG_UI_STOPPED);
        assert(record->stop.reason == (order < 2U ? DEBUG_UI_REASON_NONE : DEBUG_UI_REASON_NOT_DISABLED));
        assert(!ledger.pending_stop_seen);
        stopped.disabled_confirmed = (uint8_t)!stopped.disabled_confirmed;
        stopped.reason = DEBUG_UI_REASON_BUSY;
        debug_ui_load_trials_completion(&ledger, &stopped, &stop);
        assert(record->disabled_confirmed == (uint8_t)(order < 2U));
        assert(record->stop.reason == (order < 2U ? DEBUG_UI_REASON_NONE : DEBUG_UI_REASON_NOT_DISABLED));
    }
    puts("LOAD_TRIAL_STOP_PASS early_late=1 report_bound=1 wrong_scope_rejected=1 hold_cause_preserved=1 unconfirmed=1");
}

/** @brief An unrelated or wrong-origin STOP report cannot attach an earlier accepted pending receipt. */
static void test_stop_report_identity_is_complete(void)
{
    unsigned mismatch;
    for (mismatch = 0U; mismatch < 4U; ++mismatch) {
        DebugUiLoadTrials ledger;
        DebugUiRequest request = trial_request(8U), stop = trial_request(9U);
        DebugUiSnapshot snapshot = trial_snapshot(&request);
        DebugUiCompletion stopped;
        const DebugUiLoadTrialRecord *record;
        debug_ui_load_trials_init(&ledger);
        trial_submit(&ledger, &request);
        stop.operation = DEBUG_UI_OPERATION_STOP;
        stopped = trial_completion(&stop);
        stopped.disabled_confirmed = 1U;
        debug_ui_load_trials_completion(&ledger, &stopped, &stop);
        assert(ledger.pending_stop_seen);
        snapshot.static_load.stop_identity = stop.identity;
        if (mismatch == 0U) snapshot.static_load.stop_identity.origin = DEBUG_UI_ORIGIN_USB;
        else if (mismatch == 1U) ++snapshot.static_load.stop_identity.epoch;
        else if (mismatch == 2U) ++snapshot.static_load.stop_identity.request_id;
        else ++snapshot.static_load.identity.request_id;
        debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
        record = debug_ui_load_trials_newest(&ledger, 0U);
        assert(!record->stop.seen && !record->disabled_confirmed);
        snapshot.static_load.identity = request.identity;
        snapshot.static_load.stop_identity = stop.identity;
        ++snapshot.timestamp_us; ++snapshot.generation;
        debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
        assert(record->stop.seen && record->disabled_confirmed && !ledger.pending_stop_seen);
    }
    puts("LOAD_TRIAL_STOP_REPORT_SCOPE_PASS dimensions=4 no_cross_binding=1 valid_later_binding=1");
}

/** @brief Starting another actual HOLD discards an unresolved STOP slot from the prior trial. */
static void test_pending_stop_does_not_cross_new_submission(void)
{
    DebugUiLoadTrials ledger;
    DebugUiRequest request = trial_request(8U), stop = trial_request(9U), next = trial_request(10U);
    DebugUiCompletion stopped;
    DebugUiSnapshot snapshot;
    const DebugUiLoadTrialRecord *record;
    debug_ui_load_trials_init(&ledger);
    trial_submit(&ledger, &request);
    stop.operation = DEBUG_UI_OPERATION_STOP;
    stopped = trial_completion(&stop);
    stopped.disabled_confirmed = 1U;
    debug_ui_load_trials_completion(&ledger, &stopped, &stop);
    assert(ledger.pending_stop_seen);
    trial_submit(&ledger, &next);
    assert(!ledger.pending_stop_seen && ledger.count == 2U);
    debug_ui_load_trials_completion(&ledger, &stopped, &stop);
    assert(!ledger.pending_stop_seen);
    snapshot = trial_snapshot(&next);
    snapshot.static_load.stop_identity = stop.identity;
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    record = debug_ui_load_trials_newest(&ledger, 0U);
    assert(record->report_seen && !record->stop.seen && !record->disabled_confirmed);
    assert(!debug_ui_load_trials_newest(&ledger, 1U)->stop.seen);
    puts("LOAD_TRIAL_STOP_NEXT_SCOPE_PASS pending_cleared=1 old_receipt_ignored=1");
}

/** @brief Old submitted identities retain their terminal; new metadata freezes separately and reset clears RAM. */
static void test_epoch_history_ring_and_boot_reset(void)
{
    DebugUiLoadTrials ledger;
    DebugUiRequest request = trial_request(10U), old = request;
    DebugUiSnapshot snapshot;
    DebugUiCompletion completion;
    const DebugUiLoadTrialRecord *record;
    unsigned index;
    debug_ui_load_trials_init(&ledger);
    trial_submit(&ledger, &request);
    snapshot = trial_snapshot(&request); ++snapshot.epoch;
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    completion = trial_completion(&old);
    debug_ui_load_trials_completion(&ledger, &completion, NULL);
    assert(ledger.count == 1U && debug_ui_load_trials_newest(&ledger, 0U)->hold.seen);
    assert(debug_ui_load_trials_set(&ledger, 2U, 500U, 200U));
    request = trial_request(11U); ++request.identity.epoch;
    trial_submit(&ledger, &request);
    record = debug_ui_load_trials_newest(&ledger, 0U);
    assert(record->metadata.lever_mm == 500U && record->metadata.mass_g == 200U && !record->metadata.confirmed);
    snapshot = trial_snapshot(&old);
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    assert(debug_ui_load_trials_newest(&ledger, 1U)->report_seen);
    request = trial_request(12U); ++request.identity.epoch;
    assert(debug_ui_load_trials_freeze(&ledger, 2U));
    snapshot = trial_snapshot(&request); ++snapshot.epoch; ++snapshot.timestamp_us;
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    assert(!ledger.frozen_valid && !debug_ui_load_trials_submit(&ledger, &request));
    debug_ui_load_trials_init(&ledger);
    for (index = 1U; index <= 17U; ++index) {
        request = trial_request(index);
        trial_submit(&ledger, &request);
    }
    assert(ledger.count == 16U);
    assert(debug_ui_load_trials_newest(&ledger, 0U)->identity.request_id == 17U);
    assert(debug_ui_load_trials_newest(&ledger, 15U)->identity.request_id == 2U);
    assert(!debug_ui_load_trials_newest(&ledger, 16U));
    old = trial_request(1U);
    assert(debug_ui_load_trials_freeze(&ledger, 2U));
    assert(!debug_ui_load_trials_submit(&ledger, &old) && ledger.count == 16U);
    completion = trial_completion(&old);
    debug_ui_load_trials_completion(&ledger, &completion, NULL);
    assert(!debug_ui_load_trials_newest(&ledger, 0U)->hold.seen);
    snapshot = trial_snapshot(&request); ++snapshot.boot_id;
    debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
    assert(ledger.count == 0U && !ledger.frozen_valid);
    assert(debug_ui_load_trials_candidate(&ledger, 2U)->lever_mm == 250U);
    debug_ui_load_trials_init(&ledger);
    assert(ledger.count == 0U && !ledger.epoch_seen);
    puts("LOAD_TRIAL_HISTORY_PASS old_scope_retained=1 epoch_submit_locked=1 new_freeze=1 ring=16 boot_reset=1");
}

/** @brief The final matched report survives an authority change in either order, then its measurements seal. */
static void test_retained_epoch_final_report_is_immutable(void)
{
    unsigned terminal_first;
    for (terminal_first = 0U; terminal_first < 2U; ++terminal_first) {
        DebugUiLoadTrials ledger;
        DebugUiRequest request = trial_request(15U);
        DebugUiCompletion completion = trial_completion(&request);
        DebugUiSnapshot snapshot = trial_snapshot(&request);
        const DebugUiLoadTrialRecord *record;
        debug_ui_load_trials_init(&ledger);
        trial_submit(&ledger, &request);
        ++snapshot.epoch;
        if (terminal_first) debug_ui_load_trials_completion(&ledger, &completion, NULL);
        debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
        if (!terminal_first) debug_ui_load_trials_completion(&ledger, &completion, NULL);
        record = debug_ui_load_trials_newest(&ledger, 0U);
        assert(record->hold.seen && record->report_seen && record->report_finalized);
        assert(record->hold.reason == DEBUG_UI_REASON_TORQUE_LIMIT && record->peak_torque_nm == 0.3F);
        snapshot.static_load.peak_torque_nm[1] = 99.0F;
        snapshot.static_load.elapsed_ms = 1000U;
        snapshot.static_load.samples[1] = 50U;
        ++snapshot.timestamp_us; ++snapshot.generation;
        debug_ui_load_trials_observe(&ledger, &snapshot, snapshot.timestamp_us);
        assert(record->peak_torque_nm == 0.3F && record->elapsed_ms == 99U && record->samples == 5U);
        completion.reason = DEBUG_UI_REASON_NONE;
        debug_ui_load_trials_completion(&ledger, &completion, NULL);
        assert(record->hold.reason == DEBUG_UI_REASON_TORQUE_LIMIT && ledger.count == 1U);
    }
    puts("LOAD_TRIAL_EPOCH_FINAL_PASS orders=2 complete_scope=1 final_measurements_sealed=1 duplicate_cause_ignored=1");
}

#ifdef LOAD_TRIAL_MODEL_HOOKS_TEST
/** @brief Real review/submission/update/completion paths own the ledger while requests stay unchanged. */
static void test_model_ledger_hooks(void)
{
    unsigned refused;
    for (refused = 0U; refused < 2U; ++refused) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = fixture();
        DebugUiRequest draft_before_edit, submitted;
        DebugUiCompletion completion;
        const DebugUiLoadTrialRecord *record;
        snapshot.boot_id = 9U;
        snapshot.target_motor_id = 2U;
        snapshot.single_static_load_enabled = 1U;
        snapshot.motors[1].actual_mode = DEBUG_UI_MODE_MIT;
        snapshot.motors[1].single_static_profile.configured = 1U;
        snapshot.motors[1].single_static_profile.minimum_ms = 1000U;
        snapshot.motors[1].single_static_profile.maximum_ms = 60000U;
        snapshot.motors[1].single_static_evidence_generation = 41U;
        debug_ui_model_init(&model);
        model.selected_motor = 1U;
        assert(model.load_trials.count == 0U);
        assert(debug_ui_load_trials_candidate(&model.load_trials, 2U)->lever_mm == 250U);
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
        draft_before_edit = model.draft;
        assert(debug_ui_load_trials_set(&model.load_trials, 2U, 250U, 100U));
        assert(debug_ui_load_trials_confirm(&model.load_trials, 2U));
        assert(memcmp(&draft_before_edit, &model.draft, sizeof(model.draft)) == 0);
        model.focus = 1U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_REVIEW && model.load_trials.frozen_valid);
        assert(debug_ui_load_trials_set(&model.load_trials, 2U, 500U, 200U));
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1001U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1002U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 1802U, 800U);
        assert(debug_ui_model_take_request(&model, &submitted));
        assert(model.load_trials.count == 1U);
        record = debug_ui_load_trials_newest(&model.load_trials, 0U);
        assert(record->identity.request_id == submitted.identity.request_id && record->identity.epoch == snapshot.epoch);
        assert(record->metadata.mass_g == 100U && record->metadata.confirmed);
        assert(submitted.torque_ff_nm == 0.0F && submitted.kp == 0.0F && submitted.kd == 0.0F);
        if (refused) {
            debug_ui_model_submit_failed(&model, &submitted, DEBUG_UI_REASON_BUSY);
            assert(record->submit_refused && record->hold.reason == DEBUG_UI_REASON_BUSY && !record->report_seen);
        } else {
            DebugUiSnapshot measured = trial_snapshot(&submitted), report_snapshot = snapshot;
            DebugUiRequest stop;
            DebugUiCompletion stopped;
            report_snapshot.static_load = measured.static_load;
            report_snapshot.timestamp_us = measured.timestamp_us;
            debug_ui_model_update(&model, &report_snapshot, report_snapshot.timestamp_us, 1U, 1U);
            assert(record->report_seen && record->measurements_valid);
            key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2000U, 0U);
            key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2001U, 0U);
            assert(debug_ui_model_take_request(&model, &stop) && stop.operation == DEBUG_UI_OPERATION_STOP);
            stopped = trial_completion(&stop);
            stopped.code = DEBUG_UI_STOPPED; stopped.reason = DEBUG_UI_REASON_NONE; stopped.disabled_confirmed = 1U;
            debug_ui_model_completion(&model, &stopped);
            assert(model.load_trials.pending_stop_seen && !record->stop.seen);
            report_snapshot.static_load.stop_identity = stop.identity;
            ++report_snapshot.timestamp_us; ++report_snapshot.generation;
            debug_ui_model_update(&model, &report_snapshot, report_snapshot.timestamp_us, 1U, 1U);
            assert(record->stop.seen && record->disabled_confirmed);
            completion = trial_completion(&submitted);
            debug_ui_model_completion(&model, &completion);
            assert(record->hold.seen && record->hold.reason == DEBUG_UI_REASON_TORQUE_LIMIT && record->disabled_confirmed);
        }
    }
    puts("LOAD_TRIAL_MODEL_HOOKS_PASS review_freeze=1 identified_binding=1 snapshots=1 refusal=1 completion=1 request_unchanged=1");
}
#endif

/** @brief Run every deterministic metadata and record contract assertion. */
int main(void)
{
#ifdef LOAD_TRIAL_APP_ORDER_TEST
    test_app_snapshot_before_failed_completion();
    test_app_cross_epoch_stop_cleanup();
#endif
    test_metadata_and_calculation();
    test_immutable_freeze_and_submitted_identity();
    test_report_scope_and_freshness();
    test_measurement_evidence_and_monotonic_snapshots();
    test_rejection_and_hold_terminals();
    test_stop_binding_and_delivery_orders();
    test_stop_report_identity_is_complete();
    test_pending_stop_does_not_cross_new_submission();
    test_epoch_history_ring_and_boot_reset();
    test_retained_epoch_final_report_is_immutable();
#ifdef LOAD_TRIAL_MODEL_HOOKS_TEST
    test_model_ledger_hooks();
#endif
    printf("LOAD_TRIALS_PASS ledger_bytes=%u limit=%u\n", (unsigned)sizeof(DebugUiLoadTrials),
           (unsigned)DEBUG_UI_LOAD_TRIALS_MAX_BYTES);
    return 0;
}
