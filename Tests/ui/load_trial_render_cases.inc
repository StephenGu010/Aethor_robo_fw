/** @file load_trial_render_cases.inc
 * @brief Actual LVGL renders using synthetic metadata and real public RAM-ledger APIs only.
 */

/** @brief Submit one synthetic identified HOLD with frozen per-ID metadata. */
static DebugUiRequest load_render_submit(uint8_t motor_id, uint32_t request_id)
{
    DebugUiRequest request;
    memset(&request, 0, sizeof(request));
    request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    request.target_motor_id = motor_id;
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    request.identity.epoch = model.snapshot.epoch;
    request.identity.request_id = request_id;
    request.created_at_us = model.snapshot.timestamp_us;
    request.hold_duration_ms = 1000U;
    assert(debug_ui_load_trials_freeze(&model.load_trials, motor_id));
    assert(debug_ui_load_trials_submit(&model.load_trials, &request));
    return request;
}

/** @brief Publish a matched measured report plus HOLD and independent STOP terminals. */
static void load_render_report(const DebugUiRequest *request, uint8_t failed, uint8_t disabled)
{
    DebugUiSnapshot snapshot = model.snapshot;
    DebugUiCompletion completion;
    DebugUiRequest stop = *request;
    DebugUiStaticLoadReport *report = &snapshot.static_load;
    unsigned target = request->target_motor_id - 1U;
    memset(report, 0, sizeof(*report));
    report->identity = request->identity;
    report->operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    report->motor_mask = report->valid_mask = (uint8_t)(1U << target);
    report->duration_ms = 1000U; report->elapsed_ms = failed ? 450U : 1000U;
    report->started_at_us = request->created_at_us;
    report->samples[target] = 60U;
    report->peak_error_rad[target] = 0.001f; report->peak_torque_nm[target] = 0.32f;
    report->peak_mos_c[target] = 31.0f; report->peak_rotor_c[target] = 33.0f;
    report->complete = (uint8_t)!failed; report->disabled_confirmed = disabled;
    snapshot.timestamp_us += 10000U;
    stop.operation = DEBUG_UI_OPERATION_STOP; stop.identity.request_id += 1000U;
    if (failed) report->stop_identity = stop.identity;
    debug_ui_load_trials_observe(&model.load_trials, &snapshot, snapshot.timestamp_us);
    memset(&completion, 0, sizeof(completion));
    completion.identity = request->identity; completion.operation = request->operation;
    completion.timestamp_us = snapshot.timestamp_us; completion.motor_mask = report->motor_mask;
    completion.code = failed ? DEBUG_UI_FAILED : DEBUG_UI_COMPLETED;
    completion.stage = failed ? DEBUG_UI_STAGE_HOLD : DEBUG_UI_STAGE_DISABLE;
    completion.reason = failed ? DEBUG_UI_REASON_TORQUE_LIMIT : DEBUG_UI_REASON_NONE;
    completion.disabled_confirmed = disabled;
    debug_ui_load_trials_completion(&model.load_trials, &completion, NULL);
    if (failed) {
        completion.identity = stop.identity; completion.operation = DEBUG_UI_OPERATION_STOP;
        completion.stage = DEBUG_UI_STAGE_DISABLE; completion.reason = DEBUG_UI_REASON_NOT_DISABLED;
        debug_ui_load_trials_completion(&model.load_trials, &completion, &stop);
    }
    model.snapshot = snapshot;
}

/** @brief Validate metadata/history dirty updates, frozen summaries and evidence placeholders. */
static void render_load_trial_pages(const char *directory)
{
    DebugUiRequest request;
    unsigned index;
    uint32_t changed;
    simulated_data();
    model.selected_motor = 2U; model.single_static_flow = 1U;
    model.page = DEBUG_UI_PAGE_LOAD_METADATA;
    save_page(directory, "163_load_default_250mm_unconfirmed");
    assert(strstr(view.page.row_text[2], "未确认") != NULL);
    assert(render_update_pixels() == 0U);
    assert(debug_ui_load_trials_set(&model.load_trials, 3U, 250U, 100U));
    assert(debug_ui_load_trials_confirm(&model.load_trials, 3U));
    changed = render_update_pixels();
    assert(changed > 0U && changed < 280U * 240U);
    model.focus = 3U;
    save_page(directory, "164_load_250mm_100g_confirmed");
    assert(strstr(view.page.row_text[2], "0.24525") != NULL);
    assert(debug_ui_load_trials_set(&model.load_trials, 3U, 1U, 0U)); model.focus = 0U;
    save_page(directory, "165_load_minimum_unconfirmed");
    assert(debug_ui_load_trials_set(&model.load_trials, 3U, 2000U, 10000U)); model.focus = 1U;
    save_page(directory, "166_load_maximum_unconfirmed");
    model.page = DEBUG_UI_PAGE_LOAD_NUMBER; model.load_edit_field = 0U;
    save_page(directory, "167_load_fine_lever_boundary");
    model.load_edit_field = 1U;
    save_page(directory, "168_load_fine_mass_boundary");
    model.page = DEBUG_UI_PAGE_LOAD_HISTORY; model.focus = 0U; model.list_first = 0U;
    save_page(directory, "169_load_history_empty");
    assert(debug_ui_load_trials_set(&model.load_trials, 4U, 250U, 100U));
    assert(debug_ui_load_trials_confirm(&model.load_trials, 4U));
    for (index = 1U; index <= 18U; ++index) request = load_render_submit(4U, index);
    load_render_report(&request, 1U, 0U);
    save_page(directory, "170_load_history_latest_16");
    assert(render_update_pixels() == 0U);
    model.list_first = 11U; model.focus = 15U;
    save_page(directory, "171_load_history_last_page");
    model.page = DEBUG_UI_PAGE_LOAD_DETAIL; model.load_history_offset = 0U; model.focus = 0U;
    save_page(directory, "172_load_hold_failed_stop_separate");
    assert(strstr(view.page.row_text[3], "失败") != NULL && strstr(view.page.row_text[5], "STOP") != NULL);
    assert(strstr(view.page.row_text[4], "转矩") != NULL);
    model.focus = 1U;
    save_page(directory, "173_load_measurements_disable_unconfirmed");
    assert(strstr(view.page.row_text[4], "保持计时") != NULL);
    assert(strstr(view.page.row_text[5], "未确认") != NULL);
    model.load_history_offset = 15U; model.focus = 0U;
    save_page(directory, "174_load_old_id4_without_report");
    model.focus = 1U;
    save_page(directory, "175_load_without_report_unknown");
    assert(strstr(view.page.row_text[1], "--") != NULL && strstr(view.page.row_text[5], "未知") != NULL);
    request = load_render_submit(3U, 19U);
    model.submitted = request; model.page = DEBUG_UI_PAGE_RUNNING;
    model.awaiting_result = 1U;
    model.snapshot.active = 1U;
    model.snapshot.active_identity = request.identity; model.snapshot.target_motor_id = 3U;
    model.snapshot.active_motor_mask = 4U; memset(&model.snapshot.static_load, 0, sizeof(model.snapshot.static_load));
    model.snapshot.motors[2].feedback_valid = 0U;
    save_page(directory, "176_load_running_unknown_metadata");
    assert(strstr(view.page.row_text[2], "--") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[1]), "--") != NULL);
    model.snapshot.active = 0U;
    model.awaiting_result = 0U;
    assert(debug_ui_load_trials_set(&model.load_trials, 3U, 250U, 100U));
    assert(debug_ui_load_trials_confirm(&model.load_trials, 3U));
    assert(debug_ui_load_trials_freeze(&model.load_trials, 3U));
    assert(debug_ui_load_trials_set(&model.load_trials, 3U, 260U, 200U));
    model.page = DEBUG_UI_PAGE_REVIEW; model.reviewed = request;
    save_page(directory, "177_load_review_frozen_100g");
    assert(strstr(view.page.row_text[4], "100g") != NULL && strstr(view.page.row_text[4], "260mm") == NULL);
    request = load_render_submit(4U, 20U); load_render_report(&request, 0U, 1U);
    model.submitted = request; model.page = DEBUG_UI_PAGE_LOAD_DETAIL;
    model.load_history_offset = 0U; model.focus = 1U;
    save_page(directory, "178_load_complete_measurements");
    assert(strstr(view.page.row_text[5], "已确认") != NULL);
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_RESULT; model.reason = DEBUG_UI_REASON_NONE;
    save_page(directory, "179_load_result_frozen_summary");
    assert(strstr(view.page.footer_text, "100g") != NULL);
    assert(strstr(view.page.footer_text, "中准备") != NULL && strstr(view.page.footer_text, "左退出") != NULL);
    printf("LOAD_TRIAL_RENDER_OK pages=17 metadata_dirty=%lu unchanged=0 immutable_summary=1 history=16\n", (unsigned long)changed);
}
