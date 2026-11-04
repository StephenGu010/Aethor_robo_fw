/** @file single_static_model_test.c
 * @brief Offline single-axis static UI contract tests using synthetic snapshots.
 */
#define main original_single_static_model_suite_main
#include "debug_ui_model_test_main.c"
#undef main

/** @brief The fourth home item owns static workflows and diagnostics has only three entries. */
static void test_single_static_menu_entry(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.page = DEBUG_UI_PAGE_OVERVIEW; model.focus = 3U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
    assert(model.focus == 0U);
    model.page = DEBUG_UI_PAGE_DIAGNOSTIC_MENU; model.focus = 2U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 1001U, 0U);
    assert(model.focus == 2U);
}

/** @brief Publish evidence only for the selected axis and its independent long profile. */
static DebugUiSnapshot single_fixture(unsigned selected_index)
{
    DebugUiSnapshot snapshot = fixture();
    DebugUiMotorView motor = snapshot.motors[selected_index];
    memset(snapshot.motors, 0, sizeof(snapshot.motors));
    motor.actual_mode = DEBUG_UI_MODE_MIT;
    motor.torque_max_nm = 10.0f;
    motor.single_static_profile.configured = 1U;
    motor.single_static_profile.minimum_ms = 1000U;
    motor.single_static_profile.maximum_ms = 60000U;
    snapshot.motors[selected_index] = motor;
    snapshot.single_static_load_enabled = 1U;
    return snapshot;
}

#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
/** @brief Each ID freezes only itself; STOP retains that submitted ID under browsing changes. */
static void test_single_static_ids_duration_and_stop(void)
{
    unsigned selected_index, index;
    for (selected_index = 0U; selected_index < DEBUG_UI_MOTOR_COUNT; ++selected_index) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = single_fixture(selected_index);
        DebugUiRequest request;
        uint32_t minimum, maximum;
        debug_ui_model_init(&model);
        model.selected_motor = (uint8_t)selected_index;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) == DEBUG_UI_REASON_NONE);
        assert(debug_ui_model_single_static_duration_bounds(&model, &minimum, &maximum));
        assert(minimum == 1000U && maximum == 60000U);
        assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
        assert(model.draft.target_motor_id == selected_index + 1U && model.draft.hold_duration_ms == 1000U);
        assert(model.static_locked && debug_ui_model_parameter_count(&model) == 1U);
        assert(debug_ui_model_parameter_field(&model, 0U) == 4U);
        for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
            assert(model.draft.static_references[index] == (index == selected_index ? 3U : 0U));
            assert(model.draft.static_positions_rad[index] == (index == selected_index ? 0.25f : 0.0f));
        }
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_NUMBER);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1001U, 0U);
        assert(model.draft.hold_duration_ms == 2000U);
        for (index = 0U; index < 65U; ++index)
            key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_UP, 1010U + index, 0U);
        assert(model.draft.hold_duration_ms == 60000U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1100U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1101U, 0U);
        model.focus = 1U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1102U, 0U);
        assert(model.page == DEBUG_UI_PAGE_REVIEW);
        key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 2000U, 900U);
        assert(!debug_ui_model_take_request(&model, &request));
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2001U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2010U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 2809U, 799U);
        assert(!debug_ui_model_take_request(&model, &request));
        key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 2810U, 800U);
        if ((selected_index & 1U) != 0U) assert(debug_ui_model_take_request(&model, &request));
        else { assert(model.outgoing_ready); request = model.outgoing; }
        assert(request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && request.hold_duration_ms == 60000U);
        assert(request.target_motor_id == selected_index + 1U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 2811U, 0U);
        assert(model.selected_motor == selected_index);
        model.selected_motor = (uint8_t)((selected_index + 1U) % DEBUG_UI_MOTOR_COUNT);
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2812U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2820U, 0U);
        assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP);
        assert(request.target_motor_id == selected_index + 1U && model.stop_motor_mask == (1U << selected_index));
        assert(!debug_ui_model_take_request(&model, &request));
        request = model.outgoing_stop;
        debug_ui_model_submit_failed(&model, &request, DEBUG_UI_REASON_STOP_LATCHED);
        snapshot.timestamp_us += 1000U;
        snapshot.health.ui_timestamp_us = snapshot.health.protocol_timestamp_us = snapshot.timestamp_us;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(model.stop_latch_state == DEBUG_UI_STOP_LATCH_CONFIRMED);
    }
    puts("SINGLE_STATIC_UI_IDS_OK ids=7 selected_only=1 step_ms=1000 max_ms=60000 stop_binding=1");
}

/** @brief Ordinary browsing also revokes single-axis drafts, and terminals must match scope. */
static void test_single_static_browse_and_terminal_scope(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = single_fixture(2U);
    DebugUiCompletion completion;
    debug_ui_model_init(&model);
    model.selected_motor = 2U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    model.page = DEBUG_UI_PAGE_MOTORS; model.focus = 2U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 1000U, 0U);
    assert(model.selected_motor == 3U && !model.static_locked && model.draft.static_references[2] == 0U);
    model.selected_motor = 2U;
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    model.submitted = model.draft;
    model.submitted.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    model.submitted.identity.epoch = snapshot.epoch;
    model.submitted.identity.request_id = 22U;
    model.awaiting_result = 1U;
    memset(&completion, 0, sizeof(completion));
    completion.identity = model.submitted.identity;
    completion.operation = DEBUG_UI_OPERATION_STATIC_HOLD;
    completion.motor_mask = 0x7FU;
    completion.disabled_confirmed = 1U;
    debug_ui_model_completion(&model, &completion);
    assert(model.awaiting_result && !model.completion_seen);
    completion.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    completion.motor_mask = 8U;
    debug_ui_model_completion(&model, &completion);
    assert(model.awaiting_result && !model.completion_seen);
    completion.motor_mask = 4U;
    debug_ui_model_completion(&model, &completion);
    assert(!model.awaiting_result && model.completion_seen && model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT);
    puts("SINGLE_STATIC_UI_BROWSE_TERMINAL_SCOPE_OK");
}

/** @brief Missing/illegal long profiles and selected feedback revoke all start paths. */
static void test_single_static_gates_and_selection(void)
{
    unsigned failure;
    for (failure = 0U; failure < 10U; ++failure) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = single_fixture(2U);
        DebugUiMotorView *motor = &snapshot.motors[2];
        if (failure == 0U) snapshot.single_static_load_enabled = 0U;
        else if (failure == 1U) motor->single_static_profile.configured = 0U;
        else if (failure == 2U) motor->single_static_profile.minimum_ms = 999U;
        else if (failure == 3U) motor->single_static_profile.maximum_ms = 61000U;
        else if (failure == 4U) motor->single_static_profile.maximum_ms = 999U;
        else if (failure == 5U) motor->profile.mit_valid = 0U;
        else if (failure == 6U) motor->feedback_valid = 0U;
        else if (failure == 7U) motor->reference_generation = 0U;
        else if (failure == 8U) motor->enabled = 1U;
        else motor->fault_flags = 1U;
        debug_ui_model_init(&model);
        model.selected_motor = 2U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) != DEBUG_UI_REASON_NONE);
        assert(!debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    }
    {
        DebugUiModel model;
        DebugUiSnapshot snapshot = single_fixture(2U);
        debug_ui_model_init(&model);
        model.selected_motor = 2U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
        model.page = DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS;
        model.focus = 0U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_IDS && model.focus == 2U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 1001U, 0U);
        assert(model.selected_motor == 3U && !model.static_locked);
        assert(model.draft.static_references[2] == 0U && model.reviewed.target_motor_id == 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1002U, 0U);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS);
        model.selected_motor = 2U;
        assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
        ++snapshot.motors[2].reference_generation;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(!model.static_locked && model.reason == DEBUG_UI_REASON_OLD_EPOCH);
        assert(model.notice_return_page == DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS);
    }
    puts("SINGLE_STATIC_UI_GATES_OK invalid=10 selection_revokes=1 reference_revokes=1");
}

/** @brief A lock created in one static mode cannot submit through the other mode's menu. */
static void test_single_static_lock_mode_isolation(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    unsigned index;
    snapshot.static_load_enabled = snapshot.single_static_load_enabled = 1U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        snapshot.motors[index].actual_mode = DEBUG_UI_MODE_MIT;
        snapshot.motors[index].single_static_profile.configured = 1U;
        snapshot.motors[index].single_static_profile.minimum_ms = 1000U;
        snapshot.motors[index].single_static_profile.maximum_ms = 60000U;
    }
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    model.page = DEBUG_UI_PAGE_STATIC_ACTIONS; model.focus = 3U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_NOTICE && !model.outgoing_ready);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS; model.focus = 4U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1001U, 0U);
    assert(model.page == DEBUG_UI_PAGE_NOTICE && !model.outgoing_ready);
    puts("SINGLE_STATIC_UI_LOCK_MODE_ISOLATION_OK");
}
#endif

#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
#include "static_lcd_model_cases.inc"
#endif

/** @brief The compile gate stays independent of the existing seven-axis enable bit. */
static void test_single_static_default_gate(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = single_fixture(0U);
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
#if !AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) == DEBUG_UI_REASON_DISABLED);
    assert(!debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    {
        DebugUiRequest request;
        debug_ui_model_init(&model);
        snapshot.target_motor_id = 1U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        model.page = DEBUG_UI_PAGE_STATIC_HUB;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
        model.focus = 2U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1001U, 0U);
        assert(model.page == DEBUG_UI_PAGE_NOTICE && model.reason == DEBUG_UI_REASON_DISABLED);
        assert(!debug_ui_model_take_request(&model, &request));
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1002U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1003U, 0U);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1004U, 0U);
        assert(model.page == DEBUG_UI_PAGE_STATIC_HUB && !model.single_static_flow);
        assert(model.snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED && !debug_ui_model_take_request(&model, &request));
        puts("STATIC_LCD_DEFAULT_LOCAL_BROWSE_OK configure_denied=1 safe_exit=1 ordinary_authority_preserved=1");
    }
#else
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) == DEBUG_UI_REASON_NONE);
#endif
}

/** @brief Execute the isolated offline UI cases; no application is linked. */
int main(void)
{
    test_single_static_menu_entry();
    test_single_static_default_gate();
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    test_lcd_configuration_and_relock();
    test_lcd_stale_review_and_stop();
    test_lcd_failure_exit_and_deferred_edit();
    test_lcd_idempotent_lock_preserves_external_profile();
    test_lcd_acquire_terminal_scope();
    test_lcd_revoked_review_cannot_escape();
    test_lcd_late_lock_after_stop();
    test_lcd_late_acquire_after_stop();
    test_lcd_cross_target_handoff();
    test_single_static_ids_duration_and_stop();
    test_single_static_gates_and_selection();
    test_single_static_lock_mode_isolation();
    test_single_static_browse_and_terminal_scope();
#endif
    puts("SINGLE_STATIC_UI_MODEL_OK");
    return 0;
}
