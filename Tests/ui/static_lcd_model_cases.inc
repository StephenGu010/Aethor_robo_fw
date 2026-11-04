/** @file static_lcd_model_cases.inc
 * @brief Test-first guided LCD lifecycle cases using production model events.
 */
/** @brief Release and hold independently; an entering center edge never confirms. */
static void lcd_confirm(DebugUiModel *model, uint32_t start_ms)
{
    key(model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, start_ms, 0U);
    key(model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, start_ms + 1U, 0U);
    key(model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, start_ms + 800U, 799U);
    assert(!model->outgoing_ready);
    key(model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, start_ms + 801U, 800U);
}
/** @brief Refresh the coherent synthetic snapshot without inventing motor output. */
static void lcd_publish(DebugUiModel *model, DebugUiSnapshot *snapshot)
{
    snapshot->timestamp_us += 1000ULL; ++snapshot->generation;
    snapshot->health.ui_timestamp_us = snapshot->health.protocol_timestamp_us = snapshot->timestamp_us;
    debug_ui_model_update(model, snapshot, snapshot->timestamp_us, 1U, 1U);
}
/** @brief Generate one exact-scope terminal for the currently submitted identity. */
static DebugUiCompletion lcd_completion(const DebugUiModel *model)
{
    DebugUiCompletion completion;
    memset(&completion, 0, sizeof(completion));
    completion.identity = model->submitted.identity;
    completion.operation = model->submitted.operation;
    completion.motor_mask = (uint8_t)(1U << (model->submitted.target_motor_id - 1U));
    completion.timestamp_us = model->now_us;
    return completion;
}
/** @brief Emulate exactly one acknowledged preflight and a newer coherent selected sample for lifecycle controls. */
static void lcd_preflight(DebugUiModel *model, DebugUiSnapshot *snapshot)
{
    DebugUiRequest preflight;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    if (!model->single_static_preflight) return;
    assert(debug_ui_model_take_request(model, &preflight));
    assert(preflight.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT);
    memset(&admission, 0, sizeof(admission));
    admission.identity = preflight.identity; admission.operation = preflight.operation; admission.accepted = 1U;
    debug_ui_model_admission(model, &admission);
    completion = lcd_completion(model);
    completion.stage = DEBUG_UI_STAGE_DISABLE; completion.disabled_confirmed = 1U;
    completion.timestamp_us = completion.feedback_timestamp_us = model->now_us + 1000ULL;
    debug_ui_model_completion(model, &completion);
    snapshot->motors[preflight.target_motor_id - 1U].feedback_timestamp_us = completion.feedback_timestamp_us;
    lcd_publish(model, snapshot);
}
/** @brief Every ID and selected duration requires config acknowledgement plus a new snapshot. */
static void test_lcd_configuration_and_relock(void)
{
    unsigned motor_index;
    for (motor_index = 0U; motor_index < DEBUG_UI_MOTOR_COUNT; ++motor_index) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = single_fixture(motor_index);
        DebugUiRequest request;
        DebugUiCompletion completion;
        DebugUiMotorView *motor = &snapshot.motors[motor_index];
        snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
        motor->single_static_profile.configured = 0U;
        motor->single_static_evidence_generation = 41U;
        debug_ui_model_init(&model);
        model.selected_motor = (uint8_t)motor_index;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        model.page = DEBUG_UI_PAGE_STATIC_HUB; model.focus = 0U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP && model.single_static_flow);
        model.single_static_seconds = motor_index == 0U ? 1U : 60U;
        model.focus = 2U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1001U, 0U);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW);
        key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 1901U, 900U);
        assert(!debug_ui_model_take_request(&model, &request));
        lcd_confirm(&model, 2000U);
        lcd_preflight(&model, &snapshot);
        assert(debug_ui_model_take_request(&model, &request));
        assert(request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE);
        assert(request.target_motor_id == motor_index + 1U);
        assert(request.hold_duration_ms == (uint32_t)model.single_static_seconds * 1000U);
        assert(request.reference_generation == 3U && request.single_static_evidence_generation == 41U);
        completion = lcd_completion(&model);
        completion.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK;
        debug_ui_model_completion(&model, &completion);
        assert(model.awaiting_result && !model.single_static_configured);
        completion.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE;
        completion.motor_mask ^= 0x7FU;
        debug_ui_model_completion(&model, &completion);
        assert(model.awaiting_result && !model.single_static_configured);
        completion.motor_mask = (uint8_t)(1U << motor_index);
        debug_ui_model_completion(&model, &completion);
        assert(!model.awaiting_result && !model.single_static_configured && model.single_static_wait_snapshot == 1U);
        lcd_publish(&model, &snapshot);
        assert(!model.single_static_configured);
        ++motor->reference_generation;
        ++motor->single_static_evidence_generation;
        motor->single_static_profile.configured = motor->single_static_lcd_owned = 1U;
        motor->single_static_profile.minimum_ms = 1000U;
        motor->single_static_profile.maximum_ms = request.hold_duration_ms;
        lcd_publish(&model, &snapshot);
        assert(model.single_static_configured && model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY);
        snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
        snapshot.target_motor_id = motor->motor_id;
        lcd_publish(&model, &snapshot);
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 3000U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 3001U, 0U);
        lcd_preflight(&model, &snapshot);
        assert(model.static_locked && model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY);
        assert(model.draft.hold_duration_ms == request.hold_duration_ms);
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 3002U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 3003U, 0U);
        assert(model.page == DEBUG_UI_PAGE_REVIEW);
        lcd_confirm(&model, 3100U);
        lcd_preflight(&model, &snapshot);
        assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
        assert(request.hold_duration_ms == (uint32_t)model.single_static_seconds * 1000U);
        completion = lcd_completion(&model); completion.disabled_confirmed = 1U;
        debug_ui_model_completion(&model, &completion);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT);
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 4000U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 4001U, 0U);
        lcd_preflight(&model, &snapshot);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
        assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
        completion = lcd_completion(&model); debug_ui_model_completion(&model, &completion);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
        motor->single_static_lcd_owned = motor->single_static_profile.configured = 0U;
        snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
        lcd_publish(&model, &snapshot);
        assert(model.page == DEBUG_UI_PAGE_STATIC_HUB && !model.single_static_configured && !model.static_locked);
    }
    puts("STATIC_LCD_CONFIG_RELOCK_OK ids=7 duration_1s_60s=1 exact_scope=1 completion_and_snapshot=1 independent_holds=1");
}
/** @brief Changed evidence during review and STOP versus late config completion cannot unlock. */
static void test_lcd_stale_review_and_stop(void)
{
    unsigned condition;
    for (condition = 0U; condition < 2U; ++condition) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = single_fixture(0U);
        DebugUiCompletion completion;
        DebugUiRequest request;
        snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
        snapshot.motors[0].single_static_profile.configured = 0U;
        snapshot.motors[0].single_static_evidence_generation = 1U;
        debug_ui_model_init(&model);
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        model.page = DEBUG_UI_PAGE_STATIC_HUB;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
        model.focus = 2U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1001U, 0U);
        if (condition == 0U) {
            ++snapshot.motors[0].single_static_evidence_generation;
            lcd_publish(&model, &snapshot);
            assert(model.page == DEBUG_UI_PAGE_NOTICE && !model.single_static_configured);
        } else {
            lcd_confirm(&model, 1100U);
            assert(debug_ui_model_take_request(&model, &request));
            completion = lcd_completion(&model);
            key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2000U, 0U);
            key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2001U, 0U);
            assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP);
            debug_ui_model_completion(&model, &completion);
            snapshot.motors[0].single_static_lcd_owned = snapshot.motors[0].single_static_profile.configured = 1U;
            ++snapshot.motors[0].reference_generation;
            lcd_publish(&model, &snapshot);
            assert(!model.single_static_configured && model.page != DEBUG_UI_PAGE_SINGLE_STATIC_READY);
        }
    }
    puts("STATIC_LCD_STALE_STOP_OK changed_evidence_revokes=1 late_config_cannot_unlock=1");
}

/** @brief Failure/STOP result pages cannot escape a still installed LCD profile through ordinary menus. */
static void test_lcd_failure_exit_and_deferred_edit(void)
{
    unsigned failure_page;
    for (failure_page = 0U; failure_page < 3U; ++failure_page) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = single_fixture(2U);
        DebugUiRequest request;
        DebugUiCompletion completion;
        DebugUiMotorView *motor = &snapshot.motors[2];
        motor->single_static_lcd_owned = 1U;
        motor->single_static_evidence_generation = 42U;
        snapshot.target_motor_id = 3U;
        debug_ui_model_init(&model); model.selected_motor = 2U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        model.single_static_flow = model.single_static_configured = model.static_locked = 1U;
        model.single_static_owned_reference = motor->reference_generation;
        model.single_static_owned_evidence = motor->single_static_evidence_generation;
        model.single_static_pending_motor = 2U;
        model.single_static_pending_seconds = model.single_static_seconds = 60U;
        model.page = failure_page == 0U ? DEBUG_UI_PAGE_FAULT :
            (failure_page == 1U ? DEBUG_UI_PAGE_RESULT : DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        if (failure_page == 2U) {
            model.focus = 0U;
            key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1001U, 0U);
            assert(model.selected_motor == 2U && !model.static_locked);
        } else key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1001U, 0U);
        lcd_preflight(&model, &snapshot);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK && model.single_static_configured);
        assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
        motor->single_static_lcd_owned = motor->single_static_profile.configured = 0U;
        snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
        lcd_publish(&model, &snapshot); /* New snapshot alone is never a LOCK acknowledgement. */
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
        assert(model.selected_motor == 2U && model.single_static_configured);
        completion = lcd_completion(&model);
        completion.identity.epoch ^= 1U;
        debug_ui_model_completion(&model, &completion);
        assert(model.awaiting_result && model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
        completion.identity = request.identity;
        debug_ui_model_completion(&model, &completion);
        lcd_publish(&model, &snapshot);
        assert(!model.single_static_configured && !model.static_locked);
        assert(model.page == (failure_page == 2U ? DEBUG_UI_PAGE_SINGLE_STATIC_SETUP : DEBUG_UI_PAGE_STATIC_HUB));
        assert(model.selected_motor == (failure_page == 2U ? 3U : 2U));
        assert(model.single_static_seconds == 60U);
        debug_ui_model_completion(&model, &completion);
        assert(model.page == (failure_page == 2U ? DEBUG_UI_PAGE_SINGLE_STATIC_SETUP : DEBUG_UI_PAGE_STATIC_HUB));
    }
    puts("STATIC_LCD_FAILURE_EDIT_LOCK_OK fault_exit_locked=1 lost_ack_blocks=1 deferred_id=1 duplicate_terminal_ignored=1");
}

/** @brief Idempotent exit must preserve an independently installed non-LCD profile. */
static void test_lcd_idempotent_lock_preserves_external_profile(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = single_fixture(0U);
    DebugUiRequest request;
    snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    snapshot.motors[0].single_static_evidence_generation = 55U;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.single_static_flow = 1U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP; model.focus = 2U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1000U, 0U);
    assert(!debug_ui_model_take_request(&model, &request));
    lcd_publish(&model, &snapshot);
    assert(model.page == DEBUG_UI_PAGE_STATIC_HUB && snapshot.motors[0].single_static_profile.configured);
    puts("STATIC_LCD_IDEMPOTENT_LOCK_OK non_lcd_profile_preserved=1");
}

/** @brief ACQUIRE uses its existing zero-mask terminal while retaining exact operation and identity. */
static void test_lcd_acquire_terminal_scope(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = single_fixture(0U);
    DebugUiCompletion completion;
    snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    snapshot.motors[0].single_static_lcd_owned = 1U;
    snapshot.motors[0].single_static_evidence_generation = 7U;
    debug_ui_model_init(&model); debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.single_static_flow = model.single_static_configured = 1U;
    model.single_static_owned_reference = snapshot.motors[0].reference_generation;
    model.single_static_owned_evidence = snapshot.motors[0].single_static_evidence_generation;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_READY;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    lcd_confirm(&model, 1100U);
    completion = lcd_completion(&model);
    completion.motor_mask = 0U; completion.disabled_confirmed = 1U;
    debug_ui_model_completion(&model, &completion);
    assert(!model.awaiting_result && model.single_static_wait_snapshot == 3U);
    {
        DebugUiModel stopping_model = model;
        DebugUiSnapshot stopping_snapshot = snapshot;
        key(&stopping_model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2000U, 0U);
        key(&stopping_model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2001U, 0U);
        assert(stopping_model.stop_requested && stopping_model.stop_ready);
        stopping_snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
        stopping_snapshot.target_motor_id = 1U;
        ++stopping_snapshot.epoch;
        lcd_publish(&stopping_model, &stopping_snapshot);
        assert(stopping_model.page == DEBUG_UI_PAGE_RUNNING && stopping_model.single_static_wait_snapshot == 0U);
    }
    snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; snapshot.target_motor_id = 1U; ++snapshot.epoch;
    lcd_publish(&model, &snapshot);
    assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY && model.single_static_wait_snapshot == 0U);
    puts("STATIC_LCD_ACQUIRE_SCOPE_OK original_zero_mask=1 new_epoch_snapshot=1");
}

/** @brief Revoked guided reviews and legacy compatibility pages cannot bypass relocking. */
static void test_lcd_revoked_review_cannot_escape(void)
{
    unsigned legacy_index;
    DebugUiPage legacy_pages[3] = {DEBUG_UI_PAGE_SINGLE_STATIC_LOAD, DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS,
        DEBUG_UI_PAGE_SINGLE_STATIC_IDS};
    for (legacy_index = 0U; legacy_index < 4U; ++legacy_index) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = single_fixture(0U);
        DebugUiRequest request;
        snapshot.motors[0].single_static_lcd_owned = 1U;
        snapshot.motors[0].single_static_evidence_generation = 7U;
        snapshot.target_motor_id = 1U;
        debug_ui_model_init(&model); debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        model.single_static_flow = model.single_static_configured = 1U;
        model.single_static_owned_reference = snapshot.motors[0].reference_generation;
        model.single_static_owned_evidence = snapshot.motors[0].single_static_evidence_generation;
        model.single_static_pending_seconds = 1U;
        if (legacy_index == 0U) {
            assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
            model.focus = 1U;
            key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
            assert(model.page == DEBUG_UI_PAGE_REVIEW);
            ++snapshot.motors[0].reference_generation;
            lcd_publish(&model, &snapshot);
            assert(model.page == DEBUG_UI_PAGE_NOTICE && !model.static_locked);
            snapshot.motors[0].feedback_valid = 1U;
            lcd_publish(&model, &snapshot);
            key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1001U, 0U);
            key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1002U, 0U);
            assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY);
        } else {
            model.page = legacy_pages[legacy_index - 1U]; model.focus = 0U;
            key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 1000U, 0U);
            assert(model.selected_motor == 0U && model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY);
        }
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1003U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1004U, 0U);
        lcd_preflight(&model, &snapshot);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK && snapshot.motors[0].single_static_lcd_owned);
        assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
    }
    puts("STATIC_LCD_REVOKED_REVIEW_LOCK_OK feedback_recovery_stays_guided=1 legacy_pages_contained=3 lock_before_exit=1");
}

/** @brief A late LOCK terminal after STOP cannot leave an unconsumable snapshot wait. */
static void test_lcd_late_lock_after_stop(void)
{
    unsigned terminal_order;
    for (terminal_order = 0U; terminal_order < 2U; ++terminal_order) {
    DebugUiModel model;
    DebugUiSnapshot snapshot = single_fixture(0U);
    DebugUiCompletion lock_completion, stop_completion;
    DebugUiRequest request;
    snapshot.target_motor_id = 1U;
    snapshot.motors[0].single_static_lcd_owned = 1U;
    snapshot.motors[0].single_static_evidence_generation = 8U;
    debug_ui_model_init(&model); debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.single_static_flow = model.single_static_configured = 1U;
    model.single_static_owned_reference = snapshot.motors[0].reference_generation;
    model.single_static_owned_evidence = snapshot.motors[0].single_static_evidence_generation;
    model.page = terminal_order == 0U ? DEBUG_UI_PAGE_SINGLE_STATIC_READY : DEBUG_UI_PAGE_SINGLE_STATIC_SETUP;
    model.focus = 2U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, terminal_order == 0U ? DEBUG_UI_KEY_LEFT : DEBUG_UI_KEY_CENTER, 1000U, 0U);
    lcd_preflight(&model, &snapshot);
    if (terminal_order != 0U) assert(model.single_static_lock_then_configure);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
    lock_completion = lcd_completion(&model);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1001U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1002U, 0U);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP);
    if (terminal_order == 0U) debug_ui_model_completion(&model, &lock_completion);
    memset(&stop_completion, 0, sizeof(stop_completion));
    stop_completion.identity = request.identity; stop_completion.operation = DEBUG_UI_OPERATION_STOP;
    stop_completion.code = DEBUG_UI_STOPPED; stop_completion.motor_mask = 1U; stop_completion.disabled_confirmed = 1U;
    debug_ui_model_completion(&model, &stop_completion);
    if (terminal_order != 0U) debug_ui_model_completion(&model, &lock_completion);
    snapshot.motors[0].single_static_lcd_owned = snapshot.motors[0].single_static_profile.configured = 0U;
    snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    lcd_publish(&model, &snapshot);
    assert(model.single_static_wait_snapshot == 0U && !model.awaiting_result && !model.stop_waiting);
    assert(model.completion.operation == DEBUG_UI_OPERATION_STOP);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1003U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1004U, 0U);
    assert(!debug_ui_model_take_request(&model, &request));
    lcd_publish(&model, &snapshot);
    assert(model.page == DEBUG_UI_PAGE_STATIC_HUB && !model.single_static_flow && !model.single_static_lock_then_configure);
    }
    puts("STATIC_LCD_LATE_LOCK_STOP_OK terminal_orders=2 input_before_poll_order=1 no_stuck_wait=1 retry_lock_exit=1");
}

/** @brief Cancelled ACQUIRE remains cancelled regardless of terminal delivery order. */
static void test_lcd_late_acquire_after_stop(void)
{
    unsigned terminal_order;
    for (terminal_order = 0U; terminal_order < 2U; ++terminal_order) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = single_fixture(0U);
        DebugUiCompletion acquire_completion, stop_completion;
        DebugUiRequest request;
        snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
        snapshot.motors[0].single_static_lcd_owned = 1U;
        snapshot.motors[0].single_static_evidence_generation = 8U;
        debug_ui_model_init(&model); debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        model.single_static_flow = model.single_static_configured = 1U;
        model.single_static_owned_reference = snapshot.motors[0].reference_generation;
        model.single_static_owned_evidence = snapshot.motors[0].single_static_evidence_generation;
        model.page = DEBUG_UI_PAGE_SINGLE_STATIC_READY;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
        lcd_confirm(&model, 1100U);
        assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_ACQUIRE);
        acquire_completion = lcd_completion(&model); acquire_completion.motor_mask = 0U;
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2000U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2001U, 0U);
        assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP);
        memset(&stop_completion, 0, sizeof(stop_completion));
        stop_completion.identity = request.identity; stop_completion.operation = DEBUG_UI_OPERATION_STOP;
        stop_completion.code = DEBUG_UI_STOPPED; stop_completion.motor_mask = 1U; stop_completion.disabled_confirmed = 1U;
        if (terminal_order == 0U) debug_ui_model_completion(&model, &acquire_completion);
        debug_ui_model_completion(&model, &stop_completion);
        if (terminal_order != 0U) debug_ui_model_completion(&model, &acquire_completion);
        snapshot.motors[0].single_static_lcd_owned = snapshot.motors[0].single_static_profile.configured = 0U;
        lcd_publish(&model, &snapshot);
        assert(model.single_static_wait_snapshot == 0U && !model.awaiting_result && !model.stop_waiting);
        assert(model.completion.operation == DEBUG_UI_OPERATION_STOP);
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2002U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 2003U, 0U);
        assert(!debug_ui_model_take_request(&model, &request) && model.page == DEBUG_UI_PAGE_STATIC_HUB);
    }
    puts("STATIC_LCD_LATE_ACQUIRE_STOP_OK terminal_orders=2 cancelled_identity_persists=1 stop_receipt_preserved=1");
}

/** @brief Hand back the old local target before configuring a different selected axis. */
static void test_lcd_cross_target_handoff(void)
{
    unsigned stale_old_target;
    for (stale_old_target = 0U; stale_old_target < 2U; ++stale_old_target) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = fixture();
        DebugUiRequest request;
        DebugUiCompletion completion;
        unsigned index;
        snapshot.single_static_load_enabled = 1U;
        snapshot.target_motor_id = 1U;
        for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
            snapshot.motors[index].actual_mode = DEBUG_UI_MODE_MIT;
            snapshot.motors[index].single_static_evidence_generation = 20U + index;
        }
        snapshot.motors[0].single_static_profile.configured = 1U;
        snapshot.motors[0].single_static_profile.minimum_ms = 1000U;
        snapshot.motors[0].single_static_profile.maximum_ms = 5000U;
        if (stale_old_target) snapshot.motors[0].feedback_valid = 0U;
        debug_ui_model_init(&model); model.selected_motor = 2U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        model.page = DEBUG_UI_PAGE_STATIC_HUB;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
        model.single_static_seconds = model.single_static_pending_seconds = 60U;
        model.focus = 2U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1001U, 0U);
        assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK && model.selected_motor == 2U);
        if (stale_old_target) {
            assert(model.reason == DEBUG_UI_REASON_STALE_FEEDBACK && !debug_ui_model_take_request(&model, &request));
            key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1002U, 0U);
            assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP && request.target_motor_id == 1U);
        } else {
            assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
            assert(request.target_motor_id == 1U && request.reference_generation == snapshot.motors[0].reference_generation &&
                request.single_static_evidence_generation == 20U);
            completion = lcd_completion(&model); debug_ui_model_completion(&model, &completion);
            {
                DebugUiModel stopping_model = model;
                key(&stopping_model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2000U, 0U);
                key(&stopping_model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2001U, 0U);
                assert(debug_ui_model_take_request(&stopping_model, &request) && request.operation == DEBUG_UI_OPERATION_STOP);
                assert(request.target_motor_id == 1U && stopping_model.stop_motor_mask == 1U);
            }
            snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE; ++snapshot.epoch;
            lcd_publish(&model, &snapshot);
            assert(snapshot.motors[0].single_static_profile.configured && !snapshot.motors[0].single_static_lcd_owned);
            assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW && model.single_static_flow);
            assert(model.selected_motor == 2U && model.single_static_seconds == 60U && model.reviewed.target_motor_id == 3U);
            assert(model.reviewed.hold_duration_ms == 60000U && !debug_ui_model_take_request(&model, &request));
        }
    }
    puts("STATIC_LCD_CROSS_TARGET_HANDOFF_OK old_id1_selected_id3=1 selected_seconds_preserved=1 non_lcd_profile_retained=1 unsafe_old_target_stop=1");
}
