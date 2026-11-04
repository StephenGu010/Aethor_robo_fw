/** @file load_trial_review_render_cases.inc
 * @brief Real-LVGL regressions for failed-HOLD metadata and fully scoped application timing.
 */

/** @brief Inspect actual visible labels, so semantic buffers cannot hide lost operator evidence. */
static uint8_t load_review_visible_contains(const char *needle)
{
    unsigned index;
    if (strstr(lv_label_get_text(view.page.footer), needle) != NULL) return 1U;
    for (index = 0U; index < DEBUG_UI_VIEW_ROWS; ++index)
        if (!lv_obj_has_flag(view.page.rows[index], LV_OBJ_FLAG_HIDDEN) &&
            strstr(lv_label_get_text(view.page.rows[index]), needle) != NULL) return 1U;
    return 0U;
}

/** @brief Create one real ledger submission and a fresh, sampled-or-unsampled matching snapshot. */
static DebugUiRequest load_review_fixture(uint8_t motor_id, uint32_t duration_ms, uint8_t confirmed)
{
    DebugUiRequest request;
    DebugUiStaticLoadReport *report;
    simulated_data();
    model.single_static_flow = 1U;
    model.selected_motor = (uint8_t)(motor_id - 1U);
    model.page = DEBUG_UI_PAGE_RUNNING;
    model.snapshot.motors[motor_id - 1U].actual_mode = DEBUG_UI_MODE_MIT;
    model.snapshot.motors[motor_id - 1U].fault_flags = 0U;
    if (confirmed) {
        assert(debug_ui_load_trials_set(&model.load_trials, motor_id, 250U, 100U));
        assert(debug_ui_load_trials_confirm(&model.load_trials, motor_id));
    }
    memset(&request, 0, sizeof(request));
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    request.identity.epoch = model.snapshot.epoch; request.identity.request_id = 100U;
    request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    request.target_motor_id = motor_id; request.hold_duration_ms = duration_ms;
    request.static_positions_rad[motor_id - 1U] = model.snapshot.motors[motor_id - 1U].position_rad;
    request.created_at_us = model.snapshot.timestamp_us;
    assert(debug_ui_load_trials_freeze(&model.load_trials, motor_id));
    assert(debug_ui_load_trials_submit(&model.load_trials, &request));
    model.submitted = request; model.awaiting_result = 1U;
    model.static_locked = 1U; model.draft = request;
    model.draft.static_positions_rad[motor_id - 1U] = model.snapshot.motors[motor_id - 1U].position_rad;
    model.snapshot.active = 1U; model.snapshot.target_motor_id = motor_id;
    model.snapshot.active_motor_mask = (uint8_t)(1U << (motor_id - 1U));
    model.snapshot.active_identity = request.identity;
    report = &model.snapshot.static_load;
    memset(report, 0, sizeof(*report));
    report->identity = request.identity; report->operation = request.operation;
    report->motor_mask = model.snapshot.active_motor_mask; report->active = 1U;
    report->duration_ms = duration_ms; report->elapsed_ms = duration_ms / 2U;
    report->target_position_rad[motor_id - 1U] = request.static_positions_rad[motor_id - 1U];
    report->started_at_us = request.created_at_us + 1000U;
    model.snapshot.timestamp_us += 10000U; model.now_us = model.snapshot.timestamp_us;
    debug_ui_load_trials_observe(&model.load_trials, &model.snapshot, model.now_us);
    return request;
}

/** @brief Deliver the same failed terminal used by production model completion, retaining its reason. */
static void load_review_fail(const DebugUiRequest *request)
{
    DebugUiCompletion completion;
    memset(&completion, 0, sizeof(completion));
    completion.identity = request->identity; completion.operation = request->operation;
    completion.motor_mask = (uint8_t)(1U << (request->target_motor_id - 1U));
    completion.code = DEBUG_UI_FAILED; completion.reason = DEBUG_UI_REASON_TORQUE_LIMIT;
    completion.stage = DEBUG_UI_STAGE_HOLD; completion.timestamp_us = model.snapshot.timestamp_us;
    model.snapshot.active = model.snapshot.static_load.active = 0U;
    debug_ui_model_completion(&model, &completion);
    assert(model.reason == DEBUG_UI_REASON_TORQUE_LIMIT && model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT);
}

/** @brief Record independent regressions before one final assertion so both review findings have RED evidence. */
static void render_load_trial_review_regressions(const char *directory)
{
    DebugUiRequest request;
    unsigned failures = 0U, index;
    static const char *const timer_names[] = {
        "185_load_timer_wrong_axis", "186_load_timer_wrong_duration", "187_load_timer_old_identity",
        "188_load_timer_old_start", "189_load_timer_stale_snapshot", "190_load_timer_cross_operation",
        "191_load_timer_other_boot", "192_load_timer_future_snapshot"
    };
    for (index = 0U; index < 4U; ++index) {
        char name[80];
        uint8_t confirmed = (uint8_t)((index & 1U) == 0U);
        request = load_review_fixture((uint8_t)(index + 3U), 1000U, confirmed);
        load_review_fail(&request);
        if (index == 2U) model.page = DEBUG_UI_PAGE_RESULT;
        if (index == 3U) model.page = DEBUG_UI_PAGE_FAULT;
        (void)snprintf(name, sizeof(name), "%u_load_failed_%s_%s", 180U + index,
            index < 2U ? "single" : (index == 2U ? "result" : "fault"), confirmed ? "confirmed" : "unfilled");
        save_page(directory, name);
        if (!(strstr(view.page.footer_text, confirmed ? "100g" : "载荷未填写") != NULL &&
              (!confirmed || strstr(view.page.footer_text, "250mm") != NULL) &&
              strstr(view.page.footer_text, "中准备") != NULL && strstr(view.page.footer_text, "左退出") != NULL &&
              load_review_visible_contains(debug_ui_reason_text(DEBUG_UI_REASON_TORQUE_LIMIT)))) {
            printf("FAILED_RESULT_META_RED case=%u footer=%s reason_visible=%u\n", index,
                view.page.footer_text, load_review_visible_contains(debug_ui_reason_text(DEBUG_UI_REASON_TORQUE_LIMIT)));
            ++failures;
        }
    }
    request = load_review_fixture(2U, 1000U, 1U);
    model.page = DEBUG_UI_PAGE_RESULT; model.awaiting_result = model.snapshot.active = 0U;
    model.submitted.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE;
    model.submitted.identity.request_id += 1U;
    save_page(directory, "184_load_lifecycle_without_hold");
    assert(strstr(view.page.footer_text, "100g") == NULL && strstr(view.page.footer_text, "载荷未填写") == NULL);
    for (index = 0U; index < 8U; ++index) {
        char expected[32];
        uint8_t motor_id = index == 0U ? 4U : (uint8_t)(index % 7U + 1U);
        uint32_t duration_ms = (index + 1U) * 1000U;
        request = load_review_fixture(motor_id, duration_ms, 1U);
        if (index == 0U) { model.snapshot.static_load.motor_mask = 1U; model.snapshot.static_load.duration_ms = 9000U; model.snapshot.static_load.elapsed_ms = 4500U; }
        else if (index == 1U) model.snapshot.static_load.duration_ms = 9000U;
        else if (index == 2U) --model.snapshot.static_load.identity.request_id;
        else if (index == 3U) model.snapshot.static_load.started_at_us = request.created_at_us - 1U;
        else if (index == 4U) model.now_us += DEBUG_UI_HEALTH_TIMEOUT_US;
        else if (index == 5U) { model.snapshot.static_load.operation = DEBUG_UI_OPERATION_STATIC_HOLD; model.snapshot.static_load.motor_mask = 0x7FU; }
        else if (index == 6U) ++model.snapshot.boot_id;
        else model.now_us = model.snapshot.timestamp_us - 1U;
        save_page(directory, timer_names[index]);
        (void)snprintf(expected, sizeof(expected), "-- / %lu s", (unsigned long)(duration_ms / 1000U));
        if (strcmp(lv_label_get_text(view.page.rows[1]), expected) != 0 || view.page.static_art.progress_steps != 0U) {
            printf("TIMER_SCOPE_RED case=%u expected=%s actual=%s steps=%u\n", index, expected,
                lv_label_get_text(view.page.rows[1]), view.page.static_art.progress_steps);
            ++failures;
        }
        (void)snprintf(expected, sizeof(expected), "ID%u", request.target_motor_id);
        if (strstr(view.page.footer_text, expected) == NULL || strstr(lv_label_get_text(view.page.rows[0]), expected) == NULL ||
            strstr(lv_label_get_text(view.page.rows[2]), "--") == NULL) {
            printf("TIMER_METADATA_SCOPE_RED case=%u expected=%s footer=%s title=%s error=%s\n", index, expected,
                view.page.footer_text, lv_label_get_text(view.page.rows[0]), lv_label_get_text(view.page.rows[2]));
            ++failures;
        }
    }
    request = load_review_fixture(4U, 1000U, 0U);
    save_page(directory, "193_load_timer_matched_zero_samples");
    assert(!model.snapshot.static_load.samples[3] && strcmp(lv_label_get_text(view.page.rows[1]), "0.5 / 1 s") == 0);
    assert(view.page.static_art.progress_steps == 12U);
    printf("LOAD_TRIAL_REVIEW_REGRESSIONS failures=%u metadata_cases=4 timer_scope_cases=8 zero_samples_clock=1\n", failures);
    assert(failures == 0U);
}

/** @brief Exercise accepted/rejected report delivery through production model_update before real raster checks. */
static void render_load_trial_timer_order(const char *directory)
{
    unsigned index, failures = 0U;
    static const char *const names[] = {
        "194_load_timer_regressive_elapsed", "195_load_timer_duplicate_changed_elapsed",
        "196_load_timer_finalized_reactivation", "197_load_timer_duplicate_accepted"
    };
    for (index = 0U; index < 4U; ++index) {
        DebugUiSnapshot snapshot;
        const DebugUiLoadTrialRecord *record;
        char expected[32];
        uint8_t motor_id = index == 0U ? 4U : (index == 1U ? 2U : (index == 2U ? 6U : 1U));
        uint32_t duration_ms = (index + 1U) * 1000U;
        (void)load_review_fixture(motor_id, duration_ms, (uint8_t)(index != 1U));
        snapshot = model.snapshot;
        snapshot.timestamp_us += 10000U;
        if (index == 0U) {
            snapshot.motors[3].mos_temperature_c = 31.0f;
            snapshot.motors[3].rotor_temperature_c = 33.0f;
        }
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        record = debug_ui_load_trials_newest(&model.load_trials, 0U);
        assert(record != NULL && record->elapsed_ms == duration_ms / 2U && record->report_active);
        if (index == 0U) {
            snapshot.timestamp_us += 10000U;
            snapshot.static_load.elapsed_ms = 100U;
        } else if (index == 1U) snapshot.static_load.elapsed_ms += 200U;
        else if (index == 2U) {
            snapshot.timestamp_us += 10000U;
            snapshot.active = snapshot.static_load.active = 0U;
            snapshot.static_load.complete = 1U;
            snapshot.static_load.elapsed_ms = duration_ms;
            debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
            assert(record->report_finalized && !record->report_active && record->elapsed_ms == duration_ms);
            snapshot.timestamp_us += 10000U;
            snapshot.active = snapshot.static_load.active = 1U;
            snapshot.static_load.complete = 0U;
        }
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(model.page == DEBUG_UI_PAGE_RUNNING);
        if (index < 2U) assert(record->elapsed_ms == duration_ms / 2U);
        save_page(directory, names[index]);
        if (index == 3U) (void)snprintf(expected, sizeof(expected), "2.0 / 4 s");
        else (void)snprintf(expected, sizeof(expected), "-- / %lu s", (unsigned long)(duration_ms / 1000U));
        if (strcmp(lv_label_get_text(view.page.rows[1]), expected) != 0 ||
            view.page.static_art.progress_steps != (index == 3U ? 12U : 0U)) {
            printf("ACCEPTED_TIME_ORDER_RED case=%u ledger_elapsed=%lu raw_elapsed=%lu finalized=%u timer=%s expected=%s\n",
                index, (unsigned long)record->elapsed_ms, (unsigned long)model.snapshot.static_load.elapsed_ms,
                record->report_finalized, lv_label_get_text(view.page.rows[1]), expected);
            ++failures;
        }
    }
    printf("LOAD_TRIAL_TIMER_ORDER failures=%u real_model_update=1 regression=1 duplicate_changed=1 finalized=1 duplicate_accepted=1\n", failures);
    assert(failures == 0U);
}
