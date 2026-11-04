/** @file debug_ui_pos_load_layout.inc
 * @brief Fixed-slot real LVGL POS load pages with identity-bound measured evidence.
 */

/** @brief Select only independent POS pages and the currently submitted POS review/run/result. */
static uint8_t pos_load_view_page(const DebugUiModel *model)
{
    if (model->page == DEBUG_UI_PAGE_POS_LOAD_SETUP || model->page == DEBUG_UI_PAGE_POS_LOAD_NUMBER ||
        model->page == DEBUG_UI_PAGE_POS_LOAD_RESULT) return 1U;
    return (uint8_t)(model->pos_load_flow &&
        ((model->page == DEBUG_UI_PAGE_REVIEW && model->reviewed.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST) ||
         ((model->page == DEBUG_UI_PAGE_RUNNING || model->page == DEBUG_UI_PAGE_FAULT || model->page == DEBUG_UI_PAGE_RESULT) &&
          model->submitted.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST)));
}

/** @brief Match only the parameter-page control check, leaving result recovery navigation unchanged. */
static uint8_t pos_load_prepare_return(const DebugUiModel *model)
{
    return (uint8_t)(model->pos_load_flow && model->page == DEBUG_UI_PAGE_PREPARE &&
        model->prepare_return_page == DEBUG_UI_PAGE_POS_LOAD_SETUP);
}

/** @brief Match the complete trial identity and target before exposing any report values. */
static uint8_t pos_load_report_bound(const DebugUiModel *model)
{
    const DebugUiPosLoadReport *report = &model->snapshot.pos_load;
    const DebugUiIdentity *identity = &model->submitted.identity;
    return (uint8_t)(identity->request_id != 0U && report->identity.request_id == identity->request_id &&
        report->identity.origin == identity->origin && report->identity.epoch == identity->epoch &&
        report->motor_id == model->submitted.target_motor_id && report->duration_ms == model->submitted.hold_duration_ms &&
        report->motor_id > 0U && report->motor_id <= DEBUG_UI_MOTOR_COUNT &&
        report->started_at_us >= model->submitted.created_at_us && report->started_at_us <= model->snapshot.timestamp_us &&
        report->elapsed_ms <= report->duration_ms && isfinite(report->target_position_rad) &&
        isfinite(report->error_rad) && isfinite(report->peak_error_rad) && isfinite(report->peak_torque_nm) &&
        isfinite(report->peak_mos_c) && isfinite(report->peak_rotor_c));
}

/** @brief Bind display coordinates to the edited/reviewed/submitted target, never an unrelated browsed axis. */
static const DebugUiMotorView *pos_load_view_motor(const DebugUiModel *model)
{
    uint8_t target = model->page == DEBUG_UI_PAGE_REVIEW ? model->reviewed.target_motor_id :
        ((model->page == DEBUG_UI_PAGE_POS_LOAD_SETUP || model->page == DEBUG_UI_PAGE_POS_LOAD_NUMBER) ?
         model->draft.target_motor_id : model->submitted.target_motor_id);
    if (target < 1U || target > DEBUG_UI_MOTOR_COUNT) target = (uint8_t)(model->selected_motor + 1U);
    return &model->snapshot.motors[target <= DEBUG_UI_MOTOR_COUNT ? target - 1U : 0U];
}

/** @brief Detect known driver-coordinate overflow without manufacturing fresh start-pose evidence. */
static uint8_t pos_load_driver_range_blocked(const DebugUiMotorView *motor, const DebugUiRequest *request)
{
    DebugUiMotorProfile bounds = debug_ui_pos_load_bounds(motor);
    float target = motor->position_rad + request->delta_rad;
    return (uint8_t)(debug_ui_pos_load_flange(motor) &&
        (fabsf(request->delta_rad) > bounds.max_delta_rad ||
         (motor->feedback_valid && motor->feedback_seen && motor->feedback_age_ms < 150U &&
          (!isfinite(target) || target < bounds.position_min_rad || target > bounds.position_max_rad))));
}

/** @brief Describe measured deviation/recovery without asserting an external force or an unobserved pass. */
static const char *pos_load_observation(const DebugUiPosLoadReport *report)
{
    if (!report->disturbances) return report->complete ? "未观察到有效扰动" : "尚未观察到偏离";
    if (report->recovering) return report->complete ? "回位未完成 / 已结束" : "受扰回位观察中";
    return report->recoveries == report->disturbances && report->recovery_observed ? "已观察到偏离后回位" : "回位未完成";
}

/** @brief Compose seven semantic rows; ID7 uses confirmed flange degrees; stored control values remain output radians. */
static void pos_load_view_rows(DebugUiViewPage *page, const DebugUiModel *model)
{
    const DebugUiRequest *request = model->page == DEBUG_UI_PAGE_REVIEW ? &model->reviewed : &model->draft;
    const DebugUiPosLoadReport *report = &model->snapshot.pos_load;
    const DebugUiLoadMetadata *metadata = debug_ui_load_trials_candidate(&model->load_trials, (uint8_t)(model->selected_motor + 1U));
    const DebugUiMotorView *motor = pos_load_view_motor(model);
    char text[DEBUG_UI_VIEW_TEXT_BYTES];
    uint8_t setup = (uint8_t)(model->page == DEBUG_UI_PAGE_POS_LOAD_SETUP || model->page == DEBUG_UI_PAGE_POS_LOAD_NUMBER);
    uint8_t result = (uint8_t)(model->page == DEBUG_UI_PAGE_POS_LOAD_RESULT || model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT);
    if (setup || model->page == DEBUG_UI_PAGE_REVIEW) {
        (void)snprintf(text, sizeof(text), "ID%u / %s%s", model->selected_motor + 1U,
            debug_ui_pos_load_flange(motor) ? "法兰 / " : "",
            motor->mode_verified && motor->actual_mode == DEBUG_UI_MODE_POS_VEL ? "实际模式2" : "电机页先切模式2"); row(page, 0U, text);
        (void)snprintf(text, sizeof(text), "%s %+.1f°", debug_ui_pos_load_flange(motor) ? "法兰相对角" : "相对角度", (double)debug_ui_radians_to_degrees(debug_ui_pos_load_display(motor, request->delta_rad))); row(page, 1U, text);
        (void)snprintf(text, sizeof(text), "%s %.1f°/s", debug_ui_pos_load_flange(motor) ? "法兰速度" : "速度", (double)debug_ui_radians_to_degrees(debug_ui_pos_load_display(motor, request->speed_rad_s))); row(page, 2U, text);
        (void)snprintf(text, sizeof(text), "保持 %lu 秒 / 1~60秒", (unsigned long)(request->hold_duration_ms / 1000U)); row(page, 3U, text);
        (void)snprintf(text, sizeof(text), "记录 %ug / %umm%s", metadata != NULL ? metadata->mass_g : 0U,
            metadata != NULL ? metadata->lever_mm : 0U, metadata != NULL && metadata->confirmed ? " 已确认" : " 未确认"); row(page, 4U, text);
        row(page, 5U, setup ? (AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST && model->snapshot.pos_load_enabled ?
            (pos_load_driver_range_blocked(motor, request) ? "驱动范围不符 / 不可开始" : "预览并开始 / 一次确认") : "默认锁止 / 固件未开放") :
            (model->review_released ? "长按中键800ms开始" : "先松开中键再长按"));
        row(page, 6U, setup ? "控制检查 / 中进入" : "后端失能预检 / 需承托");
        if (model->page == DEBUG_UI_PAGE_POS_LOAD_NUMBER) {
            (void)snprintf(text, sizeof(text), "编辑 %s", model->edit_field == 1U ? "相对角度" : (model->edit_field == 2U ? "速度" : "保持秒数")); row(page, 0U, text);
            row(page, 5U, "左右选位 / 上下0-9");
            row(page, 6U, "中键保存 / 长按左键取消");
        }
        return;
    }
    if (!pos_load_report_bound(model)) {
        row(page, 0U, "本次报告尚未绑定"); row(page, 1U, "阶段 / 目标 --");
        row(page, 2U, "误差 / 转矩 --"); row(page, 3U, "温度 / 样本 --");
        row(page, 4U, "偏离 / 回位 --"); row(page, 5U, "不会使用旧试验报告");
        row(page, 6U, model->stop_requested ? "STOP已请求 / 等待失能" : debug_ui_reason_text(model->reason));
        return;
    }
    (void)snprintf(text, sizeof(text), "ID%u %s", report->motor_id, stage_text(report->stage)); row(page, 0U, text);
    if (result && model->pos_load_result_detail) {
        (void)snprintf(text, sizeof(text), "峰值误差 %.3f°", (double)debug_ui_radians_to_degrees(debug_ui_pos_load_display(motor, report->peak_error_rad))); row(page, 1U, text);
        (void)snprintf(text, sizeof(text), "峰值转矩 %.3f Nm", (double)report->peak_torque_nm); row(page, 2U, text);
        (void)snprintf(text, sizeof(text), "温度峰值 %.1f / %.1f C", (double)report->peak_mos_c, (double)report->peak_rotor_c); row(page, 3U, text);
        (void)snprintf(text, sizeof(text), "新样本 %lu", (unsigned long)report->samples); row(page, 4U, text);
        (void)snprintf(text, sizeof(text), "回位最近/最长 %lu/%lums", (unsigned long)report->last_recovery_ms, (unsigned long)report->max_recovery_ms); row(page, 5U, text);
    } else {
        (void)snprintf(text, sizeof(text), "%s %.3f°", debug_ui_pos_load_flange(motor) ? "法兰相对目标" : "目标", (double)debug_ui_radians_to_degrees(debug_ui_pos_load_display(motor, debug_ui_pos_load_flange(motor) ? model->submitted.delta_rad : report->target_position_rad))); row(page, 1U, text);
        (void)snprintf(text, sizeof(text), "误差 %.3f° / %.1fs", (double)debug_ui_radians_to_degrees(debug_ui_pos_load_display(motor, report->error_rad)), (double)report->elapsed_ms / 1000.0); row(page, 2U, text);
        row(page, 3U, pos_load_observation(report));
        (void)snprintf(text, sizeof(text), "偏离 %lu / 回位 %lu", (unsigned long)report->disturbances, (unsigned long)report->recoveries); row(page, 4U, text);
        if (result) {
            (void)snprintf(text, sizeof(text), "%s / %s", report->stopped ? "过程已停止" :
                (!model->completion_seen ? "终态待确认" :
                (model->completion.code == DEBUG_UI_COMPLETED ?
                (model->completion.operation == DEBUG_UI_OPERATION_STOP ? "过程已停止" : "过程完成") :
                (model->completion.code == DEBUG_UI_STOPPED || model->completion.code == DEBUG_UI_CANCELLED ? "过程已停止" : "过程失败"))),
                debug_ui_reason_text(model->reason));
        } else (void)snprintf(text, sizeof(text), "新样本 %lu", (unsigned long)report->samples);
        row(page, 5U, text);
    }
    if (result) row(page, 6U, report->disabled_confirmed ? "已确认失能 / 不自动重试" : "失能未确认 / 保持锁定");
    else row(page, 6U, model->stop_requested || model->snapshot.stop_pending ? "STOP已请求 / 等待失能" : "中键 STOP / 请求失能");
}

/** @brief Fit finite measurements into a bounded visual field without changing semantic report rows. */
static void pos_load_number(char *text, size_t capacity, float value, unsigned decimals)
{
    if (!isfinite(value)) (void)snprintf(text, capacity, "--");
    else if (fabsf(value) >= 1000.0f) (void)snprintf(text, capacity, "%.3g", (double)value);
    else (void)snprintf(text, capacity, "%.*f", (int)decimals, (double)value);
}

/** @brief Place a transparent existing row with ink-aligned typography and bounded numeric fallback. */
static void pos_load_label(DebugUiViewPage *page, unsigned index, int x, int y, int width,
                            const char *text, const lv_font_t *font, uint8_t selected)
{
    lv_point_t measured;
    lv_txt_get_size(&measured, text, font, 0, 0, 32767, LV_TEXT_FLAG_NONE);
    if (measured.x > width && font == &ui_font_numeric_28) {
        font = &ui_font_16;
        lv_txt_get_size(&measured, text, font, 0, 0, 32767, LV_TEXT_FLAG_NONE);
    }
    if (measured.x > width) font = &ui_font_popup_12;
    instrument_slot(page, index, x, y, width, (int)font->line_height, text, font, 0xFFFFFFU, selected, 0U);
    astra_popup_label(page->rows[index], font, page->display_text[index], x, y, width, LV_TEXT_ALIGN_LEFT);
}

/** @brief Draw fixed decimal places and underline the selected sign or digit using retained art. */
static void pos_load_digit_label(DebugUiViewPage *page, const DebugUiModel *model)
{
    char text[12], prefix[12], glyph[2];
    unsigned cursor = model->pos_digit_cursor;
    lv_point_t prefix_size, glyph_size;
    if (model->edit_field == 3U) {
        (void)snprintf(text, sizeof(text), "%02u", (unsigned)model->pos_digit_value);
        cursor = cursor == 2U ? 0U : 1U;
    } else {
        (void)snprintf(text, sizeof(text), "%c%03u.%u",
            model->edit_field == 1U ? (model->pos_digit_negative ? '-' : '+') : ' ',
            (unsigned)model->pos_digit_value / 10U, (unsigned)model->pos_digit_value % 10U);
        if (cursor == 4U) cursor = 5U;
    }
    assert(cursor < strlen(text));
    memcpy(prefix, text, cursor); prefix[cursor] = '\0';
    glyph[0] = text[cursor]; glyph[1] = '\0';
    lv_txt_get_size(&prefix_size, prefix, &ui_font_numeric_28, 0, 0, 32767, LV_TEXT_FLAG_NONE);
    lv_txt_get_size(&glyph_size, glyph, &ui_font_numeric_28, 0, 0, 32767, LV_TEXT_FLAG_NONE);
    pos_load_label(page, 1U, 91, 109, 165, text, &ui_font_numeric_28, 0U);
    static_panel_add(page, 91 + prefix_size.x, 143, 91 + prefix_size.x + glyph_size.x, 145, 1U);
}

/** @brief Reuse a persistent icon-label as a static caption without allocating or borrowing report storage. */
static void pos_load_caption(DebugUiViewPage *page, unsigned index, int x, int y, int width,
                              const char *text, uint8_t selected)
{
    lv_obj_t *label = page->icons[index];
    page->layout_icon_mask |= (uint8_t)(1U << index);
    style_set_color(label, LV_STYLE_TEXT_COLOR, lv_color_hex(selected ? 0U : 0xFFFFFFU));
    style_set_number(label, LV_STYLE_BG_OPA, LV_OPA_TRANSP);
    style_set_number(label, LV_STYLE_BORDER_WIDTH, 0);
    style_set_number(label, LV_STYLE_RADIUS, 0);
    if (lv_label_get_text(label) != text) lv_label_set_text_static(label, text);
    astra_popup_label(label, &ui_font_popup_12, text, x, y, width, LV_TEXT_ALIGN_LEFT);
}

/** @brief Draw three editable parameter cards; focus IDs are the model's existing angle/speed/time IDs. */
static void pos_load_parameter_cards(DebugUiViewPage *page, const DebugUiModel *model,
                                      const DebugUiRequest *request, uint8_t editable)
{
    const DebugUiMotorView *motor = pos_load_view_motor(model);
    static const int positions[] = {12, 108, 198};
    static const int widths[] = {90, 84, 70};
    static const unsigned kinds[] = {STATIC_ICON_ANGLE, STATIC_ICON_SPEED, STATIC_ICON_TIMER};
    static const char *const captions[] = {"角度 °", "速度 °/s", "保持 s"};
    unsigned index;
    char number[48];
    for (index = 0U; index < 3U; ++index) {
        int left = positions[index], width = widths[index];
        uint8_t selected = (uint8_t)(editable && model->focus == index + 1U);
        static_panel_add(page, left, 62, left + width, 130, selected);
        static_icon_add(page, kinds[index], left + 5, 65, 1U, selected);
        pos_load_caption(page, index + 1U, left + 34, 70, width - 37, debug_ui_pos_load_flange(motor) && index < 2U ?
            (index == 0U ? "法兰 °" : "法兰 °/s") : captions[index], selected);
        if (index == 2U) (void)snprintf(number, sizeof(number), "%lu", (unsigned long)(request->hold_duration_ms / 1000U));
        else pos_load_number(number, sizeof(number), debug_ui_radians_to_degrees(debug_ui_pos_load_display(motor, index == 0U ? request->delta_rad : request->speed_rad_s)), 1U);
        pos_load_label(page, index + 1U, left + 5, 98, width - 10, number, &ui_font_numeric_28, selected);
    }
}

/** @brief Classify evidenced result priority, ensuring STOP and failed/unknown completion never appear as a pass. */
static unsigned pos_load_outcome_icon(const DebugUiModel *model, const char **title)
{
    const DebugUiPosLoadReport *report = &model->snapshot.pos_load;
    if (report->stopped || (model->completion_seen &&
        (model->completion.code == DEBUG_UI_STOPPED || model->completion.code == DEBUG_UI_CANCELLED ||
         (model->completion.operation == DEBUG_UI_OPERATION_STOP && model->completion.code == DEBUG_UI_COMPLETED)))) {
        *title = "已停止"; return STATIC_ICON_STOP;
    }
    if (!model->completion_seen) { *title = "终态待确认"; return STATIC_ICON_WARNING; }
    if (model->completion.code != DEBUG_UI_COMPLETED) { *title = "未完成"; return STATIC_ICON_WARNING; }
    if (!report->disturbances) { *title = "未观察到有效扰动"; return STATIC_ICON_EMPTY; }
    if (report->recovery_observed && !report->recovering && report->recoveries == report->disturbances) {
        *title = "已观察到回位"; return STATIC_ICON_CHECK;
    }
    *title = "回位未完成"; return STATIC_ICON_WARNING;
}

/** @brief Render POS pages with the existing seven row/icon pairs and fixed line-art pools only. */
static void pos_load_view_layout(DebugUiViewPage *page, const DebugUiModel *model)
{
    const DebugUiRequest *request = model->page == DEBUG_UI_PAGE_REVIEW ? &model->reviewed : &model->draft;
    const DebugUiPosLoadReport *report = &model->snapshot.pos_load;
    uint8_t setup = (uint8_t)(model->page == DEBUG_UI_PAGE_POS_LOAD_SETUP);
    uint8_t result = (uint8_t)(model->page == DEBUG_UI_PAGE_POS_LOAD_RESULT || model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT);
    unsigned index;
    char text[80];
    const DebugUiMotorView *motor = pos_load_view_motor(model);
    page->decoration = 4U;
    if (setup || model->page == DEBUG_UI_PAGE_REVIEW) {
        uint8_t selected = (uint8_t)(setup && model->focus == 0U);
        slot(page, 0U, 12, 35, 256, 23, page->row_text[0], NULL, selected, 0xFFFFFFU, &ui_font_16);
        pos_load_parameter_cards(page, model, request, setup);
        for (index = 4U; index < 7U; ++index) {
            int top = 132 + (int)(index - 4U) * 26;
            unsigned kind = index == 4U ? STATIC_ICON_ARM : (index == 5U ?
                (AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST && model->snapshot.pos_load_enabled ? STATIC_ICON_PLAY : STATIC_ICON_LOCK) : STATIC_ICON_CONTROL);
            selected = (uint8_t)(setup ? model->focus == index : index == 5U);
            static_panel_add(page, 12, top, 268, top + 26, selected);
            static_icon_add(page, kind, 17, top, 1U, selected);
            pos_load_label(page, index, 49, top + 7, 211, page->row_text[index], &ui_font_popup_12, selected);
        }
        return;
    }
    if (model->page == DEBUG_UI_PAGE_POS_LOAD_NUMBER) {
        unsigned kind = model->edit_field == 1U ? STATIC_ICON_ANGLE : (model->edit_field == 2U ? STATIC_ICON_SPEED : STATIC_ICON_TIMER);
        static_panel_add(page, 12, 54, 268, 162, 0U);
        static_icon_add(page, kind, 28, 72, 2U, 0U);
        pos_load_label(page, 0U, 91, 75, 165, model->edit_field == 1U ? (debug_ui_pos_load_flange(motor) ? "法兰相对角 °" : "相对角度 °") :
            (model->edit_field == 2U ? (debug_ui_pos_load_flange(motor) ? "法兰速度 °/s" : "速度 °/s") : "保持时间 s"), &ui_font_16, 0U);
        pos_load_digit_label(page, model);
        pos_load_label(page, 5U, 16, 174, 248, page->row_text[5], &ui_font_popup_12, 0U);
        pos_load_label(page, 6U, 16, 195, 248, page->row_text[6], &ui_font_popup_12, 0U);
        return;
    }
    if (!pos_load_report_bound(model)) {
        static_icon_add(page, STATIC_ICON_LOCK, 18, 41, 2U, 0U);
        pos_load_label(page, 0U, 82, 48, 182, page->row_text[0], &ui_font_16, 0U);
        for (index = 1U; index < 6U; ++index)
            pos_load_label(page, index, 16, 89 + (int)(index - 1U)*20, 248, page->row_text[index], &ui_font_popup_12, 0U);
        static_panel_add(page, 12, 188, 268, 210, 1U);
        pos_load_label(page, 6U, 20, 193, 240, page->row_text[6], &ui_font_popup_12, 1U);
        return;
    }
    if (result && !model->pos_load_result_detail) {
        const char *outcome;
        unsigned kind = pos_load_outcome_icon(model, &outcome);
        static_icon_add(page, kind, 18, 46, 2U, 0U);
        pos_load_label(page, 0U, 82, 42, 182, page->row_text[0], &ui_font_popup_12, 0U);
        pos_load_label(page, 3U, 82, 65, 182, outcome, &ui_font_16, 0U);
        pos_load_label(page, 5U, 16, 106, 248, page->row_text[5], &ui_font_popup_12, 0U);
        pos_load_label(page, 4U, 16, 130, 248, page->row_text[4], &ui_font_16, 0U);
        pos_load_label(page, 1U, 16, 155, 248, page->row_text[1], &ui_font_popup_12, 0U);
        static_panel_add(page, 12, 183, 268, 210, report->disabled_confirmed);
        static_icon_add(page, report->disabled_confirmed ? STATIC_ICON_CHECK : STATIC_ICON_LOCK, 18, 184, 1U, report->disabled_confirmed);
        pos_load_label(page, 6U, 51, 191, 209, page->row_text[6], &ui_font_popup_12, report->disabled_confirmed);
        return;
    }
    if (result) {
        for (index = 0U; index < 7U; ++index) {
            const char *detail = page->row_text[index];
            if (index == 5U) {
                (void)snprintf(text, sizeof(text), "回位ms %lu/%lu", (unsigned long)report->last_recovery_ms, (unsigned long)report->max_recovery_ms);
                detail = text;
            }
            pos_load_label(page, index, 16, 38 + (int)index*24, 248, detail, &ui_font_popup_12, 0U);
        }
        return;
    }
    static_icon_add(page, STATIC_ICON_MOTOR, 16, 35, 1U, 0U);
    pos_load_label(page, 0U, 50, 40, 214, page->row_text[0], &ui_font_16, 0U);
    pos_load_label(page, 1U, 16, 67, 248, page->row_text[1], &ui_font_16, 0U);
    pos_load_caption(page, 2U, 16, 101, 62, "误差 °", 0U);
    pos_load_number(text, sizeof(text), debug_ui_radians_to_degrees(debug_ui_pos_load_display(motor, report->error_rad)), 3U);
    pos_load_label(page, 2U, 82, 93, 182, text, &ui_font_numeric_28, 0U);
    pos_load_label(page, 3U, 16, 132, 248, page->row_text[3], &ui_font_16, 0U);
    pos_load_label(page, 4U, 16, 157, 248, page->row_text[4], &ui_font_16, 0U);
    (void)snprintf(text, sizeof(text), "%.1f / %.1f s", (double)report->elapsed_ms / 1000.0, (double)report->duration_ms / 1000.0);
    pos_load_label(page, 5U, 16, 178, 248, text, &ui_font_popup_12, 0U);
    static_panel_add(page, 12, 192, 268, 210, 1U);
    pos_load_label(page, 6U, 20, 196, 240, page->row_text[6], &ui_font_popup_12, 1U);
}
