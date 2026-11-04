/** @file debug_ui_load_trial_layout.inc
 * @brief Fixed-slot monochrome metadata editor and sixteen-entry RAM history for LVGL 8.
 * Hanging-mass gravity torque is display-only and remains separate from driver feedback.
 * Raw terminal stage/code remain in RAM records; operator pages show outcome and actual cause.
 */

/** @brief Identify appended read/edit pages without affecting existing page IDs. */
static uint8_t load_view_page(DebugUiPage kind)
{
    return (uint8_t)(kind >= DEBUG_UI_PAGE_LOAD_METADATA && kind <= DEBUG_UI_PAGE_LOAD_DETAIL);
}

/** @brief Explain editing availability using copied UI state only, without calling model internals. */
static uint8_t load_view_editable(const DebugUiModel *model)
{
    return (uint8_t)(model->selected_motor < DEBUG_UI_MOTOR_COUNT && !model->snapshot.active &&
        !model->snapshot.pending && !model->snapshot.stop_pending && !model->snapshot.unconfirmed_disable_mask &&
        !model->awaiting_result && !model->outgoing_ready && !model->stop_ready && !model->stop_waiting &&
        !model->stop_requested && !model->single_static_wait_snapshot &&
        model->stop_latch_state != DEBUG_UI_STOP_LATCH_WAITING &&
        model->stop_latch_state != DEBUG_UI_STOP_LATCH_UNCONFIRMED);
}

/** @brief Find only the record bound to the submitted identity, never a later candidate. */
static const DebugUiLoadTrialRecord *load_view_bound_record(const DebugUiModel *model)
{
    unsigned offset;
    if (model->submitted.operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) return NULL;
    for (offset = 0U; offset < model->load_trials.count; ++offset) {
        const DebugUiLoadTrialRecord *record = debug_ui_load_trials_newest(&model->load_trials, (uint8_t)offset);
        if (record != NULL && view_same_identity(&record->identity, &model->submitted.identity) &&
            record->operation == model->submitted.operation && record->target_motor_id == model->submitted.target_motor_id &&
            record->duration_ms == model->submitted.hold_duration_ms) return record;
    }
    return NULL;
}

/** @brief Trust application timing only after this boot's ledger observed the complete submitted scope.
 * Sampling validity is deliberately independent: a matched zero-sample report still owns its HOLD clock.
 */
static uint8_t load_view_timed_report(const DebugUiModel *model)
{
    const DebugUiLoadTrialRecord *record = load_view_bound_record(model);
    const DebugUiStaticLoadReport *report = &model->snapshot.static_load;
    uint8_t target = model->submitted.target_motor_id;
    return (uint8_t)(record != NULL && record->report_seen && record->report_active && !record->report_finalized &&
        model->load_trials.boot_seen && model->load_trials.boot_id == model->snapshot.boot_id &&
        model->load_trials.last_snapshot_at_us == model->snapshot.timestamp_us &&
        model->snapshot_seen && model->now_us >= model->snapshot.timestamp_us &&
        model->now_us - model->snapshot.timestamp_us < DEBUG_UI_HEALTH_TIMEOUT_US &&
        model->snapshot.timestamp_us > model->submitted.created_at_us &&
        report->active && report->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        target > 0U && target <= DEBUG_UI_MOTOR_COUNT && report->motor_mask == (uint8_t)(1U << (target - 1U)) &&
        report->duration_ms == model->submitted.hold_duration_ms &&
        report->elapsed_ms == record->elapsed_ms &&
        view_same_identity(&report->identity, &model->submitted.identity) &&
        report->started_at_us == record->report_started_at_us &&
        report->started_at_us >= model->submitted.created_at_us && report->started_at_us <= model->snapshot.timestamp_us);
}

/** @brief Label the frozen submission ID explicitly so a mismatched report cannot relabel its load. */
static void load_view_running_footer(char *text, size_t capacity, const DebugUiModel *model,
                                     const DebugUiLoadMetadata *metadata)
{
    if (metadata == NULL || !metadata->confirmed)
        (void)snprintf(text, capacity, "ID%u 载荷未填写 / 反馈沿原轴口径", model->submitted.target_motor_id);
    else (void)snprintf(text, capacity, "ID%u %umm/%ug 挂重%.5fNm", model->submitted.target_motor_id,
        metadata->lever_mm, metadata->mass_g, (double)debug_ui_load_trials_gravity_nm(metadata));
}

/** @brief Summarize frozen metadata; no reduction or automatic feedback comparison is used. */
static void load_view_summary(char *text, size_t capacity, const DebugUiLoadMetadata *metadata)
{
    if (metadata == NULL || !metadata->confirmed) (void)snprintf(text, capacity, "载荷未填写 / 反馈沿原轴口径");
    else (void)snprintf(text, capacity, "%umm / %ug / 挂重 %.5fNm", metadata->lever_mm, metadata->mass_g,
        (double)debug_ui_load_trials_gravity_nm(metadata));
}

/** @brief Keep result acknowledgement and relock navigation visible beside a compact frozen summary. */
static void load_view_result_footer(char *text, size_t capacity, const DebugUiLoadMetadata *metadata)
{
    if (metadata == NULL || !metadata->confirmed)
        (void)snprintf(text, capacity, "载荷未填写  中准备 左退出");
    else (void)snprintf(text, capacity, "挂重%ug/%umm=%.3gNm 中准备 左退出", metadata->mass_g,
        metadata->lever_mm, (double)debug_ui_load_trials_gravity_nm(metadata));
}

/** @brief Name an evidenced terminal without implying a physical qualification. */
static const char *load_view_outcome(const DebugUiLoadTrialOutcome *outcome)
{
    if (!outcome->seen) return "未知";
    if (outcome->code == DEBUG_UI_COMPLETED) return "完成";
    if (outcome->code == DEBUG_UI_STOPPED) return "已停止";
    if (outcome->code == DEBUG_UI_CANCELLED) return "已取消";
    return "失败";
}

/** @brief Compose semantic strings from read-only ledger evidence and selected display state. */
static void load_view_rows(DebugUiViewPage *page, const DebugUiModel *model)
{
    char text[DEBUG_UI_VIEW_TEXT_BYTES];
    const DebugUiLoadMetadata *metadata = debug_ui_load_trials_candidate(&model->load_trials, (uint8_t)(model->selected_motor + 1U));
    const DebugUiLoadTrialRecord *record;
    unsigned index, first;
    if (model->page == DEBUG_UI_PAGE_LOAD_METADATA || model->page == DEBUG_UI_PAGE_LOAD_NUMBER) {
        if (metadata == NULL) { row(page, 0U, "电机ID未知"); return; }
        (void)snprintf(text, sizeof(text), "水平力臂  %u mm", metadata->lever_mm); row(page, 0U, text);
        (void)snprintf(text, sizeof(text), "额外挂重  %u g", metadata->mass_g); row(page, 1U, text);
        if (metadata->confirmed) (void)snprintf(text, sizeof(text), "挂重计算  %.5f Nm", (double)debug_ui_load_trials_gravity_nm(metadata));
        else (void)snprintf(text, sizeof(text), "挂重计算  -- / 未确认");
        row(page, 2U, text);
        row(page, 3U, metadata->confirmed ? "确认参数  已确认" : "确认参数  待填写/未确认");
        (void)snprintf(text, sizeof(text), "查看记录  %u/16", model->load_trials.count); row(page, 4U, text);
        row(page, 5U, "返回准备配置");
        row(page, 6U, load_view_editable(model) ? "仅记录 / 输入范围非允许载重" : "处理中 / 禁止编辑");
        return;
    }
    if (model->page == DEBUG_UI_PAGE_LOAD_HISTORY) {
        if (model->load_trials.count == 0U) {
            row(page, 0U, "暂无记录"); row(page, 1U, "单轴保持提交后记录");
            row(page, 2U, "最近16条 / 新到旧");
        } else {
            first = model->list_first;
            if (model->load_trials.count <= 5U) first = 0U;
            else if (first > model->load_trials.count - 5U) first = model->load_trials.count - 5U;
            for (index = 0U; index < 5U && first + index < model->load_trials.count; ++index) {
                record = debug_ui_load_trials_newest(&model->load_trials, (uint8_t)(first + index));
                (void)snprintf(text, sizeof(text), "%02u  ID%u  #%lu  %s", first + index + 1U, record->target_motor_id,
                    (unsigned long)record->identity.request_id, record->submit_refused ? "提交拒绝" : load_view_outcome(&record->hold));
                row(page, index, text);
            }
            (void)snprintf(text, sizeof(text), "行 %u-%u / %u  新到旧", first + 1U,
                first + 5U < model->load_trials.count ? first + 5U : model->load_trials.count, model->load_trials.count);
            row(page, 5U, text);
        }
        row(page, 6U, "RAM记录 / 满16条覆盖 / 重启清空");
        return;
    }
    record = debug_ui_load_trials_newest(&model->load_trials, model->load_history_offset);
    if (record == NULL) { row(page, 0U, "记录未知 / 返回列表"); return; }
    (void)snprintf(text, sizeof(text), "ID%u / #%lu / %s %u/2", record->target_motor_id,
        (unsigned long)record->identity.request_id, model->focus == 0U ? "实际结果" : "实测数据", model->focus + 1U);
    row(page, 0U, text);
    if (model->focus == 0U) {
        if (record->metadata.confirmed) (void)snprintf(text, sizeof(text), "力臂 %umm / 挂重 %ug", record->metadata.lever_mm, record->metadata.mass_g);
        else (void)snprintf(text, sizeof(text), "载荷未填写 / 参数未确认");
        row(page, 1U, text);
        if (record->metadata.confirmed) (void)snprintf(text, sizeof(text), "挂重计算 %.5f Nm", (double)debug_ui_load_trials_gravity_nm(&record->metadata));
        else (void)snprintf(text, sizeof(text), "挂重计算 -- / 未确认");
        row(page, 2U, text);
        if (record->hold.seen) (void)snprintf(text, sizeof(text), "保持 %s", record->submit_refused ? "提交拒绝" : load_view_outcome(&record->hold));
        else (void)snprintf(text, sizeof(text), "保持 / 未收到终态");
        row(page, 3U, text);
        row(page, 4U, record->hold.seen ? debug_ui_reason_text((DebugUiReason)record->hold.reason) : "保持原因 / 未知");
        if (record->stop.seen) (void)snprintf(text, sizeof(text), "STOP %s", load_view_outcome(&record->stop));
        else (void)snprintf(text, sizeof(text), "STOP / 未收到终态");
        row(page, 5U, text);
        row(page, 6U, record->stop.seen ? debug_ui_reason_text((DebugUiReason)record->stop.reason) : "STOP原因 / 未知");
    } else {
        uint8_t measured = (uint8_t)(record->report_seen && record->measurements_valid && record->samples != 0U);
        if (measured && isfinite(debug_ui_radians_to_degrees(record->peak_error_rad)))
            (void)snprintf(text, sizeof(text), "偏差峰值 %.3g°", (double)debug_ui_radians_to_degrees(record->peak_error_rad));
        else (void)snprintf(text, sizeof(text), "偏差峰值 --");
        row(page, 1U, text);
        if (measured) (void)snprintf(text, sizeof(text), "驱动反馈峰值 %.3g Nm", (double)record->peak_torque_nm);
        else (void)snprintf(text, sizeof(text), "驱动反馈峰值 --");
        row(page, 2U, text);
        if (measured) (void)snprintf(text, sizeof(text), "MOS %.3g / 转子 %.3g°C", (double)record->peak_mos_c, (double)record->peak_rotor_c);
        else (void)snprintf(text, sizeof(text), "MOS -- / 转子 --");
        row(page, 3U, text);
        if (record->report_seen) (void)snprintf(text, sizeof(text), "保持计时 %.1f / %.1f s", (double)record->elapsed_ms / 1000.0, (double)record->duration_ms / 1000.0);
        else (void)snprintf(text, sizeof(text), "保持计时 -- / %.1f s", (double)record->duration_ms / 1000.0);
        row(page, 4U, text);
        if (measured) (void)snprintf(text, sizeof(text), "样本 %lu / 失能 %s", (unsigned long)record->samples,
            record->disabled_known ? (record->disabled_confirmed ? "已确认" : "未确认") : "未知");
        else (void)snprintf(text, sizeof(text), "样本 -- / 失能 %s", record->disabled_known ? (record->disabled_confirmed ? "已确认" : "未确认") : "未知");
        row(page, 5U, text);
        row(page, 6U, "反馈沿原轴口径 / 非挂重计算");
    }
}

/** @brief Reuse seven label slots and bounded line-art panels, with no canvas or new objects. */
static void load_view_layout(DebugUiViewPage *page, const DebugUiModel *model)
{
    unsigned index;
    page->decoration = 4U;
    if (model->page == DEBUG_UI_PAGE_LOAD_NUMBER) {
        const DebugUiLoadMetadata *metadata = debug_ui_load_trials_candidate(&model->load_trials, (uint8_t)(model->selected_motor + 1U));
        char text[80];
        if (metadata == NULL) return;
        static_panel_add(page,12,48,268,174,0U);
        static_icon_add(page,model->load_edit_field == 0U ? STATIC_ICON_ANGLE : STATIC_ICON_TORQUE,24,61,1U,0U);
        instrument_slot(page,0U,61,62,193,23,model->load_edit_field == 0U ? "水平力臂精调" : "额外挂重精调",&ui_font_16,0xFFFFFFU,0U,0U);
        (void)snprintf(text,sizeof(text),"%u",model->load_edit_field == 0U ? metadata->lever_mm : metadata->mass_g);
        instrument_slot(page,1U,37,99,215,38,text,&ui_font_popup_30,0xFFFFFFU,0U,0U);
        instrument_slot(page,2U,24,145,232,20,model->load_edit_field == 0U ? "输入 1~2000 mm" : "输入 0~10000 g",&ui_font_popup_12,0xAAAAAAU,0U,0U);
        instrument_slot(page,3U,16,186,248,19,load_view_editable(model) ? "仅记录 / 非允许载重" : "处理中 / 禁止编辑",&ui_font_popup_12,0xAAAAAAU,0U,0U);
        return;
    }
    for (index = 0U; index < 7U; ++index) {
        uint8_t selected = (uint8_t)((model->page == DEBUG_UI_PAGE_LOAD_METADATA && model->focus == index && index < 6U) ||
            (model->page == DEBUG_UI_PAGE_LOAD_HISTORY && index < 5U && model->load_trials.count != 0U && model->focus == model->list_first + index));
        int top = 36 + (int)index * 25;
        if (index < 6U && (model->page == DEBUG_UI_PAGE_LOAD_METADATA || model->page == DEBUG_UI_PAGE_LOAD_HISTORY))
            static_panel_add(page,12,top,268,top+23,selected);
        if (model->page == DEBUG_UI_PAGE_LOAD_METADATA && index < 6U)
            static_icon_add(page,index == 0U ? STATIC_ICON_ANGLE : (index < 3U ? STATIC_ICON_TORQUE :
                (index == 3U ? STATIC_ICON_CHECK : (index == 4U ? STATIC_ICON_TIMER : STATIC_ICON_LEFT))),18,top,1U,selected);
        instrument_slot(page,index,model->page == DEBUG_UI_PAGE_LOAD_METADATA && index < 6U ? 48 : 20,top+3,
            model->page == DEBUG_UI_PAGE_LOAD_METADATA && index < 6U ? 208 : 240,21,page->row_text[index],&ui_font_popup_12,
            index == 6U ? 0xAAAAAAU : 0xFFFFFFU,selected,0U);
    }
}
