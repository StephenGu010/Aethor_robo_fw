/** @file pos_load_model_test.c
 * @brief Real production model all-POS key-flow tests using synthetic evidence only.
 */
#define main original_model_suite_main
#include "debug_ui_model_test_main.c"
#undef main

/** @brief Open only the appended third hub entry without publishing motor commands. */
static void pos_setup(DebugUiModel *model, DebugUiSnapshot *snapshot)
{
    debug_ui_model_init(model);
    snapshot->pos_load_enabled = 1U;
    snapshot->target_motor_id = 1U;
    debug_ui_model_update(model, snapshot, snapshot->timestamp_us, 1U, 1U);
    model->page = DEBUG_UI_PAGE_STATIC_HUB; model->focus = 1U;
    key(model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 1000U, 0U);
    assert(model->focus == 2U);
    key(model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1001U, 0U);
    assert(model->page == DEBUG_UI_PAGE_POS_LOAD_SETUP && model->pos_load_flow);
    assert(!model->outgoing_ready && !model->stop_ready);
    key(model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1002U, 0U);
}

/** @brief Draft edits remain bounded output-axis values, signed and isolated from old flows. */
static void test_pos_load_edit(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    unsigned index;
    pos_setup(&model, &snapshot);
    assert(model.draft.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST);
    model.focus = 1U;
    for (index = 0U; index < 100U; ++index)
        key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_LEFT, 1010U + index, 0U);
    assert(model.draft.delta_rad < 0.0f && model.draft.delta_rad >= -snapshot.motors[0].profile.max_delta_rad);
    model.focus = 3U;
    for (index = 0U; index < 65U; ++index)
        key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_RIGHT, 1200U + index, 0U);
    assert(model.draft.hold_duration_ms == 60000U);
    model.focus = 0U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1300U, 0U);
    assert(model.selected_motor == 1U && model.draft.target_motor_id == 2U);
    assert(!model.outgoing_ready && !model.single_static_flow);
}

/** @brief One release and 800ms review emits exactly one POS request; subsequent CENTER is STOP. */
static void test_pos_load_confirm_and_stop(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request, frozen;
    pos_setup(&model, &snapshot);
    model.focus = 5U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1010U, 0U);
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 2010U, 1000U);
    assert(!model.outgoing_ready);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2020U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2030U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 2829U, 799U);
    assert(!model.outgoing_ready);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 2830U, 800U);
    assert(model.page == DEBUG_UI_PAGE_RUNNING && debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST && request.requested_mode == DEBUG_UI_MODE_POS_VEL);
    assert(request.kp == 0.0f && request.kd == 0.0f && request.hold_duration_ms == 1000U);
    frozen = model.submitted;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 2840U, 0U);
    assert(!memcmp(&frozen, &model.submitted, sizeof(frozen)));
    assert(!debug_ui_model_take_request(&model, &request));
    model.input_valid = 0U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2841U, 0U);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP);
    assert(!debug_ui_model_take_request(&model, &request));
#else
    (void)request; (void)frozen;
    assert(model.page == DEBUG_UI_PAGE_NOTICE && model.reason == DEBUG_UI_REASON_DISABLED);
    assert(!model.outgoing_ready);
#endif
}

/** @brief Actual mode2, finite parameters and current identity are required before confirmed start. */
static void test_pos_load_gates(void)
{
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    pos_setup(&model, &snapshot);
    model.snapshot.motors[0].actual_mode = DEBUG_UI_MODE_MIT;
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_POS_LOAD_TEST) == DEBUG_UI_REASON_NOT_READY);
    model.focus = 5U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1010U, 0U);
    assert(model.page == DEBUG_UI_PAGE_NOTICE && !model.outgoing_ready);
    pos_setup(&model, &snapshot);
    model.focus = 5U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1010U, 0U);
    ++snapshot.motors[0].reference_generation;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(model.page == DEBUG_UI_PAGE_NOTICE && model.reason == DEBUG_UI_REASON_OLD_EPOCH);
    assert(!model.outgoing_ready);
#endif
}

/** @brief A real 800ms review keeps health live while standard disabled feedback ages; backend must reacquire it. */
static void test_pos_load_review_feedback_ages(void)
{
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    pos_setup(&model, &snapshot);
    model.focus = 5U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1010U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1020U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1030U, 0U);
    snapshot.timestamp_us += 830000ULL;
    snapshot.health.ui_timestamp_us = snapshot.health.protocol_timestamp_us = snapshot.timestamp_us;
    snapshot.motors[0].feedback_age_ms = 830U;
    snapshot.motors[0].feedback_valid = 0U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 1830U, 800U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST);
    assert(!model.snapshot.motors[0].feedback_valid && model.snapshot.motors[0].feedback_age_ms == 830U);
#endif
}

/** @brief Drive the existing release/800ms ownership confirmation without any motor command. */
static DebugUiRequest pos_control_check_start(DebugUiModel *model, DebugUiSnapshot *snapshot)
{
    DebugUiRequest request;
    snapshot->authority = DEBUG_UI_AUTHORITY_REMOTE;
    pos_setup(model, snapshot);
    model->draft.delta_rad = debug_ui_degrees_to_radians(-2.0f);
    model->draft.speed_rad_s = debug_ui_degrees_to_radians(0.5f);
    model->draft.hold_duration_ms = 9000U;
    model->focus = 6U;
    key(model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1010U, 0U);
    assert(model->page == DEBUG_UI_PAGE_PREPARE);
    key(model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1020U, 0U);
    assert(model->page == DEBUG_UI_PAGE_REVIEW && model->reviewed.operation == DEBUG_UI_OPERATION_ACQUIRE);
    key(model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1030U, 0U);
    key(model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1040U, 0U);
    key(model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 1840U, 800U);
    assert(debug_ui_model_take_request(model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_ACQUIRE && !model->outgoing_ready);
    return request;
}

/** @brief Build only a synthetic ownership receipt; it cannot install authority or feedback. */
static DebugUiCompletion pos_control_completion(const DebugUiRequest *request)
{
    DebugUiCompletion completion;
    memset(&completion, 0, sizeof(completion));
    completion.identity = request->identity;
    completion.operation = DEBUG_UI_OPERATION_ACQUIRE;
    completion.code = DEBUG_UI_COMPLETED;
    completion.reason = DEBUG_UI_REASON_NONE;
    return completion;
}

/** @brief Matching ownership completion returns directly with parameters, never inventing authorization. */
static void test_pos_control_check_return(void)
{
    DebugUiModel model, before;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request = pos_control_check_start(&model, &snapshot);
    DebugUiCompletion completion = pos_control_completion(&request);
    debug_ui_model_completion(&model, &completion);
    assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP && !model.awaiting_result);
    assert(model.draft.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST && model.draft.target_motor_id == 1U);
    assert(model.draft.delta_rad == debug_ui_degrees_to_radians(-2.0f));
    assert(model.draft.speed_rad_s == debug_ui_degrees_to_radians(0.5f) && model.draft.hold_duration_ms == 9000U);
    assert(model.snapshot.authority == DEBUG_UI_AUTHORITY_REMOTE && !model.outgoing_ready && !model.stop_ready);
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_POS_LOAD_TEST) == DEBUG_UI_REASON_NOT_ARMED);
#endif
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1850U, 0U);
    model.focus = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1860U, 0U);
    assert(model.page == DEBUG_UI_PAGE_POS_LOAD_NUMBER);
    before = model;
    debug_ui_model_completion(&model, &completion);
    assert(!memcmp(&before, &model, sizeof(model)));
    puts("POS_CONTROL_RETURN_PASS draft_preserved=1 no_forged_authority=1 duplicate_ignored=1");
}

/** @brief An already owned POS check returns instead of releasing; ordinary checks retain RELEASE. */
static void test_pos_control_already_owned(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest draft;
    pos_setup(&model, &snapshot);
    draft = model.draft;
    model.focus = 6U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1010U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1020U, 0U);
    assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP && !model.outgoing_ready && !model.awaiting_result);
    assert(!memcmp(&draft, &model.draft, sizeof(draft)));
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1030U, 0U);
    model.pos_load_flow = 0U; model.page = DEBUG_UI_PAGE_PREPARE;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1040U, 0U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW && model.reviewed.operation == DEBUG_UI_OPERATION_RELEASE);
    puts("POS_CONTROL_OWNED_PASS no_release=1 legacy_release_preserved=1");
}

/** @brief Wrong identity/type/scope receipts are ignored, and failed or obsolete checks never return as success. */
static void test_pos_control_receipt_guards(void)
{
    unsigned variant;
    for (variant = 0U; variant < 11U; ++variant) {
        DebugUiModel model, before;
        DebugUiSnapshot snapshot = fixture();
        DebugUiRequest request = pos_control_check_start(&model, &snapshot);
        DebugUiCompletion completion = pos_control_completion(&request);
        if (variant == 0U) completion.identity.origin = DEBUG_UI_ORIGIN_USB;
        if (variant == 1U) ++completion.identity.epoch;
        if (variant == 2U) ++completion.identity.request_id;
        if (variant == 3U) completion.operation = DEBUG_UI_OPERATION_RELEASE;
        if (variant == 4U) completion.motor_mask = 1U;
        if (variant == 5U) { completion.code = DEBUG_UI_FAILED; completion.reason = DEBUG_UI_REASON_NOT_READY; }
        if (variant == 6U) ++model.snapshot.epoch;
        if (variant == 7U) ++model.snapshot.motors[0].reference_generation;
        if (variant == 8U) model.snapshot.motors[0].actual_mode = DEBUG_UI_MODE_MIT;
        if (variant == 9U) completion.reason = DEBUG_UI_REASON_NOT_READY;
        if (variant == 10U) model.selected_motor = 1U;
        before = model;
        debug_ui_model_completion(&model, &completion);
        if (variant < 5U) assert(!memcmp(&before, &model, sizeof(model)));
        else assert(model.page != DEBUG_UI_PAGE_POS_LOAD_SETUP);
        assert(!model.outgoing_ready && !model.stop_ready);
    }
    puts("POS_CONTROL_RECEIPTS_PASS wrong_identity_type_scope_failed_epoch_reference_mode_axis=11");
}

/** @brief CENTER consistently opens checks; cancelling review preserves values, while wrong-axis and STOP remain gated. */
static void test_pos_control_cancel_and_scope(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest draft, request;
    snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    pos_setup(&model, &snapshot);
    model.draft.hold_duration_ms = 7000U;
    draft = model.draft;
    model.focus = 6U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1010U, 0U);
    assert(model.page == DEBUG_UI_PAGE_PREPARE);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1020U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1030U, 0U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1040U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1050U, 0U);
    assert(model.page == DEBUG_UI_PAGE_PREPARE && !memcmp(&draft, &model.draft, sizeof(draft)));
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1060U, 0U);
    assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP && !model.awaiting_result && !model.outgoing_ready);
    snapshot = fixture();
    pos_setup(&model, &snapshot);
    model.snapshot.target_motor_id = 2U;
    model.focus = 6U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1010U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1020U, 0U);
    assert(model.page == DEBUG_UI_PAGE_NOTICE && model.reason == DEBUG_UI_REASON_NOT_ARMED);
    assert(!model.outgoing_ready);
    snapshot = fixture();
    pos_setup(&model, &snapshot);
    model.focus = 6U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1010U, 0U);
    assert(model.page == DEBUG_UI_PAGE_PREPARE);
    model.snapshot.unconfirmed_disable_mask = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1020U, 0U);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP);
    assert(model.page != DEBUG_UI_PAGE_POS_LOAD_SETUP);
    puts("POS_CONTROL_SHALLOW_KEYS_PASS center_entry=1 cancel_preserves=1 wrong_axis_rejected=1 stop_first=1");
}

/** @brief Only the same boot's exact one-step ownership handoff can return from a new-epoch snapshot. */
static void test_pos_control_handoff_scope(void)
{
    unsigned variant;
    for (variant = 0U; variant < 9U; ++variant) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = fixture();
        DebugUiRequest request = pos_control_check_start(&model, &snapshot);
        DebugUiCompletion completion = pos_control_completion(&request);
        model.snapshot.epoch = request.identity.epoch + 1U;
        model.snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
        model.snapshot.target_motor_id = request.target_motor_id;
        if (variant == 1U) ++model.snapshot.boot_id;
        if (variant == 2U) ++model.snapshot.epoch;
        if (variant == 3U) model.snapshot.target_motor_id = 2U;
        if (variant == 4U) model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
        if (variant == 5U) model.snapshot.active = 1U;
        if (variant == 6U) model.snapshot.pending = 1U;
        if (variant == 7U) model.snapshot.unconfirmed_disable_mask = 1U;
        if (variant == 8U) model.snapshot.arm_fault = 1U;
        debug_ui_model_completion(&model, &completion);
        assert((model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP) == (variant == 0U));
        assert(!model.outgoing_ready && !model.stop_ready);
    }
    puts("POS_CONTROL_HANDOFF_SCOPE_PASS one_step_only=1 wrong_boot_epoch_axis_owner_activity_fault=8");
}

/** @brief An invalidated ownership review must not leave ACQUIRE values in the returned POS editor. */
static void test_pos_control_invalidated_review(void)
{
    unsigned reference_changed;
    for (reference_changed = 0U; reference_changed < 2U; ++reference_changed) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = fixture();
        DebugUiRequest draft;
        snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
        pos_setup(&model, &snapshot);
        model.draft.hold_duration_ms = 7000U;
        draft = model.draft;
        model.focus = 6U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1010U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1020U, 0U);
        assert(model.page == DEBUG_UI_PAGE_REVIEW);
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1030U, 0U);
        if (reference_changed) ++snapshot.motors[0].reference_generation;
        else ++snapshot.epoch;
        snapshot.timestamp_us += 5000ULL;
        snapshot.health.ui_timestamp_us = snapshot.health.protocol_timestamp_us = snapshot.timestamp_us;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(model.page == DEBUG_UI_PAGE_NOTICE && model.reason == DEBUG_UI_REASON_OLD_EPOCH);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1040U, 0U);
        assert(model.page == DEBUG_UI_PAGE_PREPARE);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1050U, 0U);
        assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP);
        assert(!memcmp(&draft, &model.draft, sizeof(draft)) && !model.awaiting_result && !model.outgoing_ready);
    }
    puts("POS_CONTROL_INVALIDATED_REVIEW_PASS epoch_reference=2 original_pos_draft_restored=1 no_auto_success=1");
}

/** @brief STOP remains dominant even if ownership success arrives before or after the STOP completion. */
static void test_pos_control_stop_order(void)
{
    unsigned stop_first;
    for (stop_first = 0U; stop_first < 2U; ++stop_first) {
        DebugUiModel model, before;
        DebugUiSnapshot snapshot = fixture();
        DebugUiRequest request = pos_control_check_start(&model, &snapshot), stop_request;
        DebugUiCompletion completion = pos_control_completion(&request), stopped;
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1850U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1860U, 0U);
        assert(debug_ui_model_take_request(&model, &stop_request) && stop_request.operation == DEBUG_UI_OPERATION_STOP);
        memset(&stopped, 0, sizeof(stopped));
        stopped.identity = stop_request.identity; stopped.operation = DEBUG_UI_OPERATION_STOP;
        stopped.code = DEBUG_UI_STOPPED; stopped.disabled_confirmed = 1U;
        stopped.motor_mask = 1U; stopped.stage = DEBUG_UI_STAGE_DISABLE;
        if (stop_first) debug_ui_model_completion(&model, &stopped);
        debug_ui_model_completion(&model, &completion);
        assert(model.page != DEBUG_UI_PAGE_POS_LOAD_SETUP && !model.awaiting_result);
        if (!stop_first) debug_ui_model_completion(&model, &stopped);
        before = model;
        debug_ui_model_completion(&model, &completion);
        assert(!memcmp(&before, &model, sizeof(model)) && !model.outgoing_ready && !model.stop_ready);
    }
    puts("POS_CONTROL_STOP_PASS completion_orders=2 no_success_navigation=1");
}

/** @brief Run both default-locked and enabled builds against the production model. */
int main(void)
{
    test_pos_control_invalidated_review();
    test_pos_control_handoff_scope();
    test_pos_control_check_return();
    test_pos_control_already_owned();
    test_pos_control_receipt_guards();
    test_pos_control_cancel_and_scope();
    test_pos_control_stop_order();
    test_pos_load_edit();
    test_pos_load_confirm_and_stop();
    test_pos_load_gates();
    test_pos_load_review_feedback_ages();
    puts("POS_LOAD_MODEL_OK");
    return 0;
}
