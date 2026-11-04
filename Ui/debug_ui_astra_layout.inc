/** @file debug_ui_astra_layout.inc
 * @brief Original Astra-inspired C/LVGL templates; no imported GPL code or assets.
 * Included by debug_ui_view.c to reuse its fixed text/icon slots.
 */
static const char *const astra_actions[] = { "状态详情", "POS 相对移动", "MIT 相对移动", "MIT 有限保持", "工作模式", "恢复与失能" };
static const char *const astra_recovery[] = { "清除故障", "请求失能", "位置寄存器", "控制检查" };
static const char *const astra_diagnostics[] = { "按键与显示", "调度与通信", "系统资源" };
static const char *const astra_fields[] = { "相对角度", "速度", "Kp", "Kd", "保持时间" };

/** @brief Center 10 deg/s and magnify nearby edits without narrowing the allowed speed range.
 * Each side uses a normalized distance/(distance+9) curve; bounds remain endpoints.
 * Profiles capped at or below 10 deg/s retain their linear, possibly fixed range.
 */
static float astra_speed_fraction(float current, float minimum, float maximum)
{
    const float center = 10.0f;
    float distance, span, offset;
    if (maximum <= center || minimum >= center)
        return (current-minimum)/(maximum-minimum);
    if (current <= minimum) return 0.0f;
    if (current >= maximum) return 1.0f;
    distance = current >= center ? current-center : center-current;
    span = current >= center ? maximum-center : center-minimum;
    offset = 0.5f*(distance/(distance+9.0f))/(span/(span+9.0f));
    return current >= center ? 0.5f+offset : 0.5f-offset;
}

/** @brief Draw one original line segment with a bounded coordinate transform. */
static void astra_line(lv_draw_ctx_t *context, int x, int y, int scale,
                       int x1, int y1, int x2, int y2, uint32_t color)
{
    lv_draw_line_dsc_t descriptor;
    lv_point_t first = { (lv_coord_t)(x + x1 * scale), (lv_coord_t)(y + y1 * scale) };
    lv_point_t last = { (lv_coord_t)(x + x2 * scale), (lv_coord_t)(y + y2 * scale) };
    lv_draw_line_dsc_init(&descriptor);
    descriptor.color = lv_color_hex(color);
    descriptor.width = (lv_coord_t)(scale > 1 ? 3 : 2);
    lv_draw_line(context, &descriptor, &first, &last);
}
/** @brief Original motor, chart, lock and static-load icons in a 24-unit square. */
static void astra_icon(lv_draw_ctx_t *context, unsigned icon, int x, int y, int scale, uint32_t color)
{
    static const int8_t motor[][4] = { {3,6,21,6},{21,6,21,18},{21,18,3,18},{3,18,3,6},{0,10,3,10},{0,14,3,14},{21,12,24,12},{7,2,17,2},{7,2,7,6},{17,2,17,6},{8,10,16,14},{16,10,8,14} };
    static const int8_t chart[][4] = { {2,2,2,22},{2,22,23,22},{5,16,10,10},{10,10,15,14},{15,14,22,5},{6,19,6,21},{12,17,12,21},{19,11,19,21} };
    static const int8_t lock[][4] = { {5,10,20,10},{20,10,20,23},{20,23,5,23},{5,23,5,10},{8,10,8,4},{8,4,11,1},{11,1,15,1},{15,1,18,4},{18,4,18,10},{12,14,12,19} };
    static const int8_t static_load[][4] = { {2,22,22,22},{6,22,6,16},{18,22,18,16},{4,16,20,16},{4,16,4,10},{20,16,20,10},{4,10,20,10},{8,10,8,4},{16,10,16,4},{8,4,16,4},{12,0,12,4},{10,2,12,4},{14,2,12,4} };
    const int8_t (*segments)[4] = icon == 0U ? motor : (icon == 1U ? chart : (icon == 2U ? lock : static_load));
    unsigned count = icon == 0U ? sizeof(motor)/sizeof(motor[0]) : (icon == 1U ? sizeof(chart)/sizeof(chart[0]) :
        (icon == 2U ? sizeof(lock)/sizeof(lock[0]) : sizeof(static_load)/sizeof(static_load[0])));
    unsigned index;
    for (index = 0U; index < count; ++index)
        astra_line(context, x, y, scale, segments[index][0], segments[index][1], segments[index][2], segments[index][3], color);
}
/** @brief Fill a clipped rectangle in LVGL's currently available draw buffer. */
static void astra_fill(lv_draw_ctx_t *context, int left, int top, int right, int bottom, uint32_t color)
{
    lv_draw_rect_dsc_t descriptor;
    lv_area_t area = {(lv_coord_t)left, (lv_coord_t)top, (lv_coord_t)right, (lv_coord_t)bottom};
    lv_draw_rect_dsc_init(&descriptor);
    descriptor.bg_color = lv_color_hex(color);
    descriptor.bg_opa = LV_OPA_COVER;
    lv_draw_rect(context, &descriptor, &area);
}
/** @brief Find the actual ink top so LVGL matches the reference's top-anchored text. */
static int astra_ink_top(const lv_font_t *font, const char *text)
{
    uint32_t offset = 0U;
    int top = font->line_height;
    while (text[offset] != '\0') {
        uint32_t codepoint = _lv_txt_encoded_next(text, &offset);
        lv_font_glyph_dsc_t glyph;
        if (lv_font_get_glyph_dsc(font, &glyph, codepoint, 0U) && glyph.box_h != 0U) {
            int glyph_top = font->line_height - font->base_line - glyph.box_h - glyph.ofs_y;
            if (glyph_top < top) top = glyph_top;
        }
    }
    return top == font->line_height ? 0 : top;
}
/** @brief Draw one parent-list string with the design's left/right text alignment. */
static void astra_parent_text(lv_draw_ctx_t *context, const lv_font_t *font,
                              const char *text, int x, int top, uint8_t right, uint32_t color)
{
    lv_draw_label_dsc_t descriptor;
    lv_point_t size;
    lv_area_t area;
    lv_draw_label_dsc_init(&descriptor);
    descriptor.font = font; descriptor.color = lv_color_hex(color);
    lv_txt_get_size(&size, text, font, 0, 0, 32767, LV_TEXT_FLAG_NONE);
    area.x1 = (lv_coord_t)(right ? x-size.x : x);
    area.y1 = (lv_coord_t)(top-astra_ink_top(font, text));
    area.x2 = (lv_coord_t)(area.x1+size.x); area.y2 = (lv_coord_t)(area.y1+font->line_height-1);
    lv_draw_label(context, &descriptor, &area, text, NULL);
}
/** @brief Apply the reference's checkerboard directly, bounded by both draw and clip areas. */
static void astra_parent_dither(lv_draw_ctx_t *context)
{
    lv_area_t area, content = {0,35,279,209};
    int x, y, stride = lv_area_get_width(context->buf_area);
    lv_color_t *pixels = (lv_color_t *)context->buf;
    if (!_lv_area_intersect(&area, &content, context->clip_area) ||
        !_lv_area_intersect(&area, &area, context->buf_area)) return;
    for (y = area.y1; y <= area.y2; ++y)
        for (x = area.x1; x <= area.x2; ++x)
            if (((x+y)&1) == 0)
                pixels[(y-context->buf_area->y1)*stride+x-context->buf_area->x1] = lv_color_black();
}
/** @brief Match the 247x157 radius-eight popup without enabling LVGL complex drawing. */
static void astra_number_panel(lv_draw_ctx_t *context, const DebugUiViewPage *page)
{
    static const uint8_t outer[8] = {6,4,3,2,1,1,0,0};
    static const uint8_t inner[8] = {0,0,6,4,3,3,2,2};
    static const uint8_t selection_corner[5] = {3,2,1,0,0};
    unsigned index;
    for (index = 0U; index < 5U; ++index) {
        int top = 40+(int)index*32;
        uint8_t selected = (uint8_t)(index == page->popup_focus);
        if (selected) {
            unsigned corner;
            astra_fill(context, 10, top+5, 257, top+23, 0xFFFFFFU);
            for (corner = 0U; corner < 5U; ++corner) {
                astra_fill(context, 10+selection_corner[corner], top+(int)corner, 257-selection_corner[corner], top+(int)corner, 0xFFFFFFU);
                astra_fill(context, 10+selection_corner[corner], top+28-(int)corner, 257-selection_corner[corner], top+28-(int)corner, 0xFFFFFFU);
            }
        }
        astra_parent_text(context, &ui_font_16, page->background_text[index], 18, top+5, 0U, selected ? 0U : 0xFFFFFFU);
        astra_parent_text(context, &ui_font_popup_14, page->background_text[index]+40, 249, top+6, 1U, selected ? 0U : 0xFFFFFFU);
    }
    astra_parent_dither(context);
    astra_fill(context, 17,56,263,196,0xFFFFFFU);
    astra_fill(context, 19,56,261,196,0U);
    for (index = 0U; index < 8U; ++index) {
        astra_fill(context, 17+outer[index],48+(int)index,263-outer[index],48+(int)index,0xFFFFFFU);
        astra_fill(context, 17+outer[index],204-(int)index,263-outer[index],204-(int)index,0xFFFFFFU);
        if (index >= 2U) {
            astra_fill(context,17+inner[index],48+(int)index,263-inner[index],48+(int)index,0U);
            astra_fill(context,17+inner[index],204-(int)index,263-inner[index],204-(int)index,0U);
        }
    }
    astra_fill(context,39,143,241,144,0xFFFFFFU);
    if (page->popup_marker >= 39 && page->popup_marker <= 241)
        astra_fill(context,page->popup_marker-3,138,page->popup_marker+3,148,0xFFFFFFU);
}
#include "debug_ui_static_icons.inc"

/** @brief Draw parent context, Tile corners and scrollbar without additional objects. */
static void astra_draw(lv_event_t *event)
{
    DebugUiViewPage *page = (DebugUiViewPage *)lv_event_get_user_data(event);
    lv_draw_ctx_t *context = lv_event_get_draw_ctx(event);
    unsigned index;
    if (page == NULL || context == NULL) return;
    if (page->decoration == 3U) {
        astra_fill(context,12,32,268,32,0xFFFFFFU);
        astra_fill(context,12,211,268,211,0xFFFFFFU);
        astra_number_panel(context, page);
    } else {
        uint32_t line_color = 0x777777U;
        astra_line(context, 0, 0, 1, 12, 29, 268, 29, line_color);
        astra_line(context, 0, 0, 1, 12, 211, 268, 211, line_color);
    }
    if (page->decoration >= 4U) static_art_draw(context, page);
    if (page->decoration == 1U) {
        int offset = page->tile_offset;
        astra_icon(context, (page->tile_focus + 3U) % 4U, 26 + offset, 96, 1, 0x777777U);
        astra_icon(context, page->tile_focus, 116 + offset, 78, 2, 0xFFFFFFU);
        astra_icon(context, (page->tile_focus + 1U) % 4U, 230 + offset, 96, 1, 0x777777U);
        for (index = 0U; index < 4U; ++index) {
            int x = index % 2U ? 180 : 100, y = index / 2U ? 142 : 64;
            int dx = index % 2U ? -12 : 12, dy = index / 2U ? -12 : 12;
            astra_line(context, 0, 0, 1, x, y, x + dx, y, 0xFFFFFFU);
            astra_line(context, 0, 0, 1, x, y, x, y + dy, 0xFFFFFFU);
        }
    } else if (page->decoration == 2U) {
        lv_draw_label_dsc_t descriptor;
        lv_draw_label_dsc_init(&descriptor);
        descriptor.font = &ui_font_16;
        descriptor.color = lv_color_hex(0x666666U);
        for (index = 0U; index < 5U; ++index) {
            lv_area_t area = { 22, (lv_coord_t)(35 + index * 34U), 258, (lv_coord_t)(55 + index * 34U) };
            lv_draw_label(context, &descriptor, &area, page->background_text[index], NULL);
        }
    }
    if (page->scrollbar_count > 5U) {
        int height = 160 * 5 / page->scrollbar_count;
        int start = 38 + (160-height) * page->scrollbar_first / (page->scrollbar_count-5U);
        astra_line(context, 0, 0, 1, 272, 38, 272, 198, 0x555555U);
        astra_line(context, 0, 0, 1, 272, start, 272, start+height, 0xFFFFFFU);
    }
}
/** @brief Advance a 140 ms idle Tile transition with a maximum 20 ms update rate. */
void debug_ui_view_animate(DebugUiView *view)
{
    static const lv_area_t home_animation_area = {12, 58, 268, 180};
    DebugUiViewPage *page;
    uint32_t now, elapsed;
    if (view == NULL || !view->initialized || !debug_ui_view_healthy()) return;
    page = &view->page;
    if (!page->animation_active) return;
    now = lv_tick_get();
    if (now - page->animation_tick < 20U) return;
    page->animation_tick = now;
    elapsed = now - page->animation_start;
    if (elapsed >= 140U) { page->animation_active = 0U; page->tile_offset = 0; }
    else page->tile_offset = (int16_t)(page->animation_from * (int32_t)(140U-elapsed) / 140);
    lv_obj_invalidate_area(page->root, &home_animation_area);
}
/** @brief Format one actual draft field in display units with no hard-coded bounds. */
static void astra_number(char *value, size_t capacity, const DebugUiRequest *action, unsigned field)
{
    if (field == 0U) (void)snprintf(value, capacity, "%+.1f°", (double)debug_ui_radians_to_degrees(action->delta_rad));
    else if (field == 1U) (void)snprintf(value, capacity, "%.1f°/s", (double)debug_ui_radians_to_degrees(action->speed_rad_s));
    else if (field == 2U) (void)snprintf(value, capacity, "%.2f", (double)action->kp);
    else if (field == 3U) (void)snprintf(value, capacity, "%.2f", (double)action->kd);
    else (void)snprintf(value, capacity, "%lu ms", (unsigned long)action->hold_duration_ms);
}
/** @brief Compose one list entry from immutable names and current snapshot evidence. */
static void astra_entry(char *value, size_t capacity, const DebugUiModel *model, DebugUiPage kind, unsigned index)
{
    const DebugUiMotorView *motor = &model->snapshot.motors[model->selected_motor < 7U ? model->selected_motor : 0U];
    if (kind == DEBUG_UI_PAGE_ACTIONS) (void)snprintf(value, capacity, "%s", astra_actions[index % 6U]);
    else if (kind == DEBUG_UI_PAGE_RECOVERY) (void)snprintf(value, capacity, "%s", astra_recovery[index % 4U]);
    else if (kind == DEBUG_UI_PAGE_DIAGNOSTIC_MENU) (void)snprintf(value, capacity, "%s", astra_diagnostics[index % 3U]);
    else if (kind == DEBUG_UI_PAGE_SINGLE_STATIC_IDS) {
        motor = &model->snapshot.motors[index < DEBUG_UI_MOTOR_COUNT ? index : 0U];
        (void)snprintf(value, capacity, "ID%u / %s", index + 1U,
            motor->single_static_profile.configured ? "长时已配置" : "长时未配置");
    }
    else if (kind == DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS) {
        uint8_t locked = (uint8_t)(model->static_locked && model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
            model->draft.target_motor_id == model->selected_motor + 1U);
        if (index == 0U) (void)snprintf(value, capacity, "选择电机 / ID%u", model->selected_motor + 1U);
        else if (index == 1U) (void)snprintf(value, capacity, "控制检查 / 先承托");
        else if (index == 2U) (void)snprintf(value, capacity, "锁定当前姿态 / %s", locked ? "已锁定" : "未锁定");
        else if (index == 3U) (void)snprintf(value, capacity, "保持时间 %lu ms", (unsigned long)(locked ? model->draft.hold_duration_ms : 0U));
        else if (index == 4U) {
            DebugUiReason reason = debug_ui_model_gate(model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
            (void)snprintf(value, capacity, "开始 / %s", reason != DEBUG_UI_REASON_NONE ? debug_ui_reason_text(reason) :
                (locked ? "长按确认" : "请先锁定姿态"));
        } else (void)snprintf(value, capacity, "%s", index == 5U ? "结束 / STOP" : "查看保留结果");
    }
    else if (kind == DEBUG_UI_PAGE_STATIC_ACTIONS) {
        uint8_t locked = (uint8_t)(model->static_locked && model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD);
        if (index == 0U) (void)snprintf(value, capacity, "控制检查 / 先承托");
        else if (index == 1U) (void)snprintf(value, capacity, "锁定七轴姿态 / %s", locked ? "已锁定" : "未锁定");
        else if (index == 2U) (void)snprintf(value, capacity, "保持时间 %lu ms",
            (unsigned long)(model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD ? model->draft.hold_duration_ms : 0U));
        else if (index == 3U) {
            DebugUiReason reason = debug_ui_model_gate(model, DEBUG_UI_OPERATION_STATIC_HOLD);
            (void)snprintf(value, capacity, "开始 / %s", reason != DEBUG_UI_REASON_NONE ? debug_ui_reason_text(reason) :
                (locked ? "长按确认" : "请先锁定姿态"));
        } else (void)snprintf(value, capacity, "%s", index == 4U ? "结束 / STOP" : "查看保留结果");
    }
    else if (kind == DEBUG_UI_PAGE_MODES) (void)snprintf(value, capacity, "%s  %s", index == 0U ? "POS" : "MIT",
        motor->mode_verified && motor->actual_mode == (index == 0U ? DEBUG_UI_MODE_POS_VEL : DEBUG_UI_MODE_MIT) ? "已读回" : (motor->mode_verified ? "待选择" : "模式未校验"));
    else if (kind == DEBUG_UI_PAGE_EDIT) {
        unsigned count = debug_ui_model_parameter_count(model);
        if (index >= count) (void)snprintf(value, capacity, "预览并执行");
        else { char number[48]; unsigned field = debug_ui_model_parameter_field(model, (uint8_t)index);
            if (field > 4U) field = 0U;
            astra_number(number, sizeof(number), &model->draft, field);
            (void)snprintf(value, capacity, "%s  %s", astra_fields[field], number); }
    } else if (kind == DEBUG_UI_PAGE_MOTORS) {
        uint8_t snapshot_current=(uint8_t)(model->snapshot_seen && model->now_us>=model->snapshot.timestamp_us &&
            model->now_us-model->snapshot.timestamp_us<DEBUG_UI_HEALTH_TIMEOUT_US);
        motor = &model->snapshot.motors[index < 7U ? index : 0U];
        (void)snprintf(value, capacity, "M%02u  %s  %s", index + 1U, motor->mode_verified?mode_text(motor->actual_mode):"模式未校验",
            motor->feedback_valid && snapshot_current ? (motor->fault_flags ? "故障" : (motor->enabled ? "使能" : "失能")) : (motor->feedback_seen ? "上次反馈过期" : "未知"));
    } else (void)snprintf(value, capacity, "%s", index < 4U ? (index == 0U ? "电机" : (index == 1U ? "诊断" : (index == 2U ? "本地控制" : "静载测试"))) : "");
}
/** @brief Count bounded menu rows; leaf pages have no selection. */
static unsigned astra_count(const DebugUiModel *model, DebugUiPage kind)
{
    if (kind == DEBUG_UI_PAGE_MOTORS || kind == DEBUG_UI_PAGE_SINGLE_STATIC_IDS) return 7U;
    if (kind == DEBUG_UI_PAGE_ACTIONS) return 6U;
    if (kind == DEBUG_UI_PAGE_EDIT) return debug_ui_model_parameter_count(model) + 1U;
    if (kind == DEBUG_UI_PAGE_MODES) return 2U;
    if (kind == DEBUG_UI_PAGE_RECOVERY) return 4U;
    if (kind == DEBUG_UI_PAGE_DIAGNOSTIC_MENU) return 3U;
    if (kind == DEBUG_UI_PAGE_STATIC_ACTIONS) return 6U;
    if (kind == DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS) return 7U;
    return 0U;
}

/** @brief Replace one retained root-drawn background row only when bytes changed. */
static void astra_background_set(DebugUiViewPage *page, unsigned index, const char background[80])
{
    assert(index < 5U);
    if (memcmp(page->background_text[index], background, sizeof(page->background_text[index])) == 0) return;
    memcpy(page->background_text[index], background, sizeof(page->background_text[index]));
    page->layout_custom_changed = 1U;
}

/** @brief Keep a dim, reconstructed parent menu behind the shared popup. */
static void astra_parent(DebugUiViewPage *page, const DebugUiModel *model, DebugUiPage parent)
{
    unsigned index, count = astra_count(model, parent);
    unsigned first = parent < DEBUG_UI_PAGE_COUNT ? model->page_first[parent] : 0U;
    page->decoration = 2U;
    for (index = 0U; index < 5U; ++index) {
        char background[80] = {0};
        if (index + first < count) astra_entry(background, sizeof(background), model, parent, index + first);
        astra_background_set(page, index, background);
    }
    page->layout_overlay_visible = 1U;
}
/** @brief Align a stable popup label to the reference ink box without another object. */
static void astra_popup_label(lv_obj_t *label, const lv_font_t *font, const char *text,
                              int x, int top, int width, lv_text_align_t align)
{
    object_set_pos(label, x, top-astra_ink_top(font,text));
    object_set_size(label, width, font->line_height);
    style_set_pointer(label,LV_STYLE_TEXT_FONT,font);
    style_set_number(label,LV_STYLE_PAD_LEFT,0); style_set_number(label,LV_STYLE_PAD_TOP,0);
    style_set_number(label,LV_STYLE_TEXT_ALIGN,align);
}
/** @brief Place one transparent text slot above retained monochrome line art. */
static void instrument_slot(DebugUiViewPage *page, unsigned index, int x, int y, int width, int height,
                            const char *text, const lv_font_t *font, uint32_t ink, uint8_t selected, uint8_t panel)
{
    lv_obj_t *label;
    (void)panel;
    assert(index < DEBUG_UI_VIEW_ROWS);
    label = page->rows[index];
    page->layout_row_mask |= (uint8_t)(1U << index);
    object_set_pos(label, x, y); object_set_size(label, width, height);
    style_set_pointer(label, LV_STYLE_TEXT_FONT, font);
    style_set_number(label, LV_STYLE_TEXT_ALIGN, LV_TEXT_ALIGN_LEFT);
    style_set_color(label, LV_STYLE_TEXT_COLOR, lv_color_hex(selected ? 0U : ink));
    style_set_number(label, LV_STYLE_BG_OPA, LV_OPA_TRANSP);
    style_set_number(label, LV_STYLE_RADIUS, 0);
    style_set_number(label, LV_STYLE_BORDER_WIDTH, 0);
    style_set_number(label, LV_STYLE_PAD_LEFT, 0);
    style_set_number(label, LV_STYLE_PAD_TOP, 0);
    text_set(label, page->display_text[index], sizeof(page->display_text[index]), text);
}

#include "debug_ui_load_trial_layout.inc"
#include "debug_ui_pos_load_layout.inc"

/** @brief Return the numeric part of a semantic measurement without changing its evidence. */
static const char *static_value_part(const char *text)
{
    const char *space = strchr(text, ' ');
    return space != NULL ? space + 1 : text;
}

/** @brief Format live output-axis error with explicit millidegrees only for coherent fresh evidence. */
static void static_error_value(char *text,size_t capacity,const DebugUiModel *model,const char *fallback)
{
    const DebugUiStaticLoadReport *report=&model->snapshot.static_load;
    unsigned target;
    (void)snprintf(text,capacity,"%s",fallback);
    if (model->single_static_flow && !load_view_timed_report(model)) return;
    if (!report->active || !static_report_scope_valid(report) || report->operation!=DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) return;
    for (target=0U;target<DEBUG_UI_MOTOR_COUNT;++target)
        if ((report->motor_mask&(1U<<target))!=0U) break;
    if (target<DEBUG_UI_MOTOR_COUNT && single_static_feedback_fresh(model,target)) {
        float error=debug_ui_radians_to_degrees(model->snapshot.motors[target].position_rad-report->target_position_rad[target]);
        if (isfinite(error) && fabsf(error)<1000.0f)
            (void)snprintf(text,capacity,"%+.3f°",(double)error);
    }
}

/** @brief Render the guided flow with fixed labels, inverse controls and original line icons. */
static void instrument_layout(DebugUiViewPage *page, const DebugUiModel *model)
{
    unsigned index;
    char value[DEBUG_UI_VIEW_TEXT_BYTES];
    const uint32_t primary = 0xFFFFFFU, muted = 0xAAAAAAU;
    page->decoration = 4U;
    page->popup_marker = 0;
    if (model->page == DEBUG_UI_PAGE_STATIC_HUB) {
        instrument_slot(page,0U,16,37,248,18,"选择测试范围",&ui_font_popup_12,muted,0U,0U);
        for (index=0U; index<3U; ++index) {
            uint8_t selected=(uint8_t)(model->focus==index);
            int top=58+(int)index*50;
            static_panel_add(page,12,top,268,top+44,selected);
            static_icon_add(page,index==0U?STATIC_ICON_MOTOR:STATIC_ICON_ARM,24,top,2U,selected);
            instrument_slot(page,1U+index*2U,83,top+3,171,23,index==0U?"单轴静载":(index==1U?"七轴静载":"POS带载自动测试"),
                &ui_font_16,primary,selected,0U);
            instrument_slot(page,2U+index*2U,83,top+26,171,18,
                index==0U?"ID1~7 / 1~60秒":(index==1U?"七轴姿态 / 原有保护":"模式2 / 移动 > 保持"),
                &ui_font_popup_12,muted,selected,0U);
            static_icon_add(page,STATIC_ICON_RIGHT,239,top+17,1U,selected);
        }
        return;
    }
    if (model->page == DEBUG_UI_PAGE_RUNNING) {
        const DebugUiLoadTrialRecord *record=load_view_bound_record(model);
        uint8_t timed=load_view_timed_report(model);
        uint32_t duration=model->submitted.hold_duration_ms;
        uint32_t elapsed=timed?record->elapsed_ms:0U;
        uint8_t stopped=(uint8_t)(model->stop_requested || model->snapshot.stop_pending);
        page->decoration=5U;
        if (model->snapshot.static_load.takeover_active) {
            const DebugUiStaticLoadReport *report=&model->snapshot.static_load;
            unsigned target;
            /* The ordinary HOLD decoration paints fixed measurement captions under these slots. */
            page->decoration=4U;
            instrument_slot(page,0U,16,35,248,22,"前馈接入 / 保持尚未计时",&ui_font_16,primary,0U,0U);
            (void)snprintf(value,sizeof(value),"%.2f / 1.00 s",(double)report->takeover_elapsed_ms/1000.0);
            instrument_slot(page,1U,24,61,232,39,value,&ui_font_popup_30,primary,0U,0U);
            (void)snprintf(value,sizeof(value),"目标 %+.2f / 请求 %+.2f Nm",(double)report->target_torque_ff_nm,
                (double)report->requested_torque_ff_nm);
            instrument_slot(page,2U,16,107,248,20,value,&ui_font_popup_12,primary,0U,0U);
            (void)snprintf(value,sizeof(value),"已发 %+.2f Nm / 非承重确认",(double)report->transmitted_torque_ff_nm);
            instrument_slot(page,3U,16,130,248,20,value,&ui_font_popup_12,primary,0U,0U);
            for (target=0U;target<DEBUG_UI_MOTOR_COUNT;++target)
                if ((report->motor_mask&(1U<<target))!=0U) break;
            if (target<DEBUG_UI_MOTOR_COUNT && single_static_feedback_fresh(model,target)) {
                const DebugUiMotorView *motor=&model->snapshot.motors[target];
                float error=debug_ui_radians_to_degrees(motor->position_rad-report->target_position_rad[target]);
                if (isfinite(error)) (void)snprintf(value,sizeof(value),"当前 %.3gNm / %+.3g°",
                    (double)motor->torque_nm,(double)error);
                else (void)snprintf(value,sizeof(value),"当前 -- / 反馈异常");
            } else (void)snprintf(value,sizeof(value),"当前 -- / 反馈过期");
            instrument_slot(page,4U,16,153,248,18,value,&ui_font_popup_12,primary,0U,0U);
            instrument_slot(page,6U,16,173,248,16,page->row_text[6],&ui_font_popup_12,muted,0U,0U);
            static_panel_add(page,12,191,268,210,1U);
            instrument_slot(page,5U,24,192,232,18,"中键 STOP / 请求失能",&ui_font_popup_12,primary,1U,0U);
            return;
        }

        if (timed && duration!=0U) {
            page->static_art.progress_steps=(uint8_t)((uint64_t)(elapsed>duration?duration:elapsed)*24U/duration);
            page->popup_marker=page->static_art.progress_steps;
        }
        (void)snprintf(value,sizeof(value),"%.100s / 保持计时",page->row_text[0]);
        instrument_slot(page,0U,16,35,248,21,value,&ui_font_16,primary,0U,0U);
        static_icon_add(page,stopped?STATIC_ICON_STOP:STATIC_ICON_TIMER,24,65,1U,0U);
        if (timed) (void)snprintf(value,sizeof(value),"%.1f / %.0f s",(double)elapsed/1000.0,(double)duration/1000.0);
        else (void)snprintf(value,sizeof(value),"-- / %.0f s",(double)duration/1000.0);
        instrument_slot(page,1U,72,58,194,39,value,&ui_font_popup_30,primary,0U,0U);
        static_icon_add(page,STATIC_ICON_ANGLE,14,117,1U,0U);
        static_icon_add(page,STATIC_ICON_TORQUE,145,117,1U,0U);
        static_error_value(value,sizeof(value),model,static_value_part(page->row_text[1]));
        instrument_slot(page,2U,42,129,96,23,value,&ui_font_16,primary,0U,0U);
        instrument_slot(page,3U,173,129,94,23,static_value_part(page->row_text[2]),&ui_font_16,primary,0U,0U);
        static_icon_add(page,STATIC_ICON_TEMP,15,144,1U,0U);
        instrument_slot(page,4U,48,154,218,19,page->row_text[3],&ui_font_popup_12,primary,0U,0U);
        instrument_slot(page,6U,16,172,248,16,page->row_text[6],&ui_font_popup_12,muted,0U,0U);
        static_panel_add(page,12,188,268,210,1U);
        instrument_slot(page,5U,24,190,232,19,stopped?"STOP已请求 / 失能待确认":"中键 STOP / 请求失能",
            &ui_font_popup_12,primary,1U,0U);
        return;
    }
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP) {
        static const uint8_t character_positions[4] = {0U, 1U, 3U, 4U};
        char number[16];
        (void)snprintf(value, sizeof(value), "参数 Kp %u / Kd %u.%02u / 中进入",
            model->single_static_kp, model->single_static_kd_centi / 100U, model->single_static_kd_centi % 100U);
        static_panel_add(page,12,33,268,53,(uint8_t)(model->focus == 4U));
        instrument_slot(page,0U,16,35,248,18,model->selected_motor == 6U ? value : "参数 / 仅 ID7 可调",
            &ui_font_popup_12,primary,(uint8_t)(model->focus == 4U),0U);
        for (index=0U; index<2U; ++index) {
            uint8_t selected=(uint8_t)(model->focus==index);
            int top=55+(int)index*31;
            static_panel_add(page,12,top,268,top+27,selected);
            if (index==0U) (void)snprintf(value,sizeof(value),"电机 < ID%u >",model->selected_motor+1U);
            else (void)snprintf(value,sizeof(value),"保持 < %u 秒 >",model->single_static_seconds);
            instrument_slot(page,index+1U,24,top+3,232,23,value,&ui_font_16,primary,selected,0U);
        }
        static_panel_add(page,12,117,268,144,(uint8_t)(model->focus==2U));
        instrument_slot(page,5U,24,120,232,23,page->row_text[5],&ui_font_16,primary,(uint8_t)(model->focus==2U),0U);
        (void)snprintf(number,sizeof(number),"%c%u.%02u",model->single_static_ff_negative?'-':'+',
            model->single_static_ff_centi/100U,model->single_static_ff_centi%100U);
        if (model->single_static_ff_editing && model->single_static_ff_digit<4U) {
            unsigned character=character_positions[model->single_static_ff_digit];
            (void)snprintf(value,sizeof(value),"前馈 %.*s[%c]%s Nm",(int)character,number,number[character],number+character+1U);
        } else (void)snprintf(value,sizeof(value),"前馈 %s Nm / 中编辑",number);
        static_panel_add(page,12,148,268,176,(uint8_t)(model->focus==3U));
        instrument_slot(page,3U,24,151,232,23,value,&ui_font_16,primary,(uint8_t)(model->focus==3U),0U);
        instrument_slot(page,4U,16,176,248,17,model->single_static_ff_editing?"左右选位 上下调值 中保存 长左取消":
            "输出轴 / 1kg 25cm 参考1.05Nm",&ui_font_popup_12,muted,0U,0U);
        instrument_slot(page,6U,16,194,248,17,model->reason==DEBUG_UI_REASON_NONE?"前馈 +/- 需核对 / 不自动输出":page->row_text[6],
            &ui_font_popup_12,muted,0U,0U);
        return;
    }
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW || model->page == DEBUG_UI_PAGE_REVIEW) {
        uint8_t configuration=(uint8_t)(model->page==DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW);
        uint8_t acquire=(uint8_t)(!configuration && model->reviewed.operation==DEBUG_UI_OPERATION_ACQUIRE);
        uint8_t pending=(uint8_t)(model->awaiting_result || model->single_static_wait_snapshot);
        static_icon_add(page,configuration?STATIC_ICON_LOCK:(acquire?STATIC_ICON_CONTROL:STATIC_ICON_PLAY),20,43,2U,0U);
        instrument_slot(page,0U,85,38,181,24,page->row_text[0],&ui_font_16,primary,0U,0U);
        instrument_slot(page,1U,85,68,181,25,page->row_text[1],&ui_font_16,primary,0U,0U);
        for (index=2U; index<6U; ++index)
            instrument_slot(page,index,16,102+(int)(index-2U)*21,248,20,page->row_text[index],&ui_font_popup_12,muted,0U,0U);
        static_panel_add(page,12,187,268,210,1U);
        instrument_slot(page,6U,24,190,232,19,pending?"等待确认 / 中键 STOP":
            (!model->review_released?"请先松开中键":"长按中键确认"),&ui_font_popup_12,primary,1U,0U);
        if (!pending && model->review_released && model->confirm_elapsed_ms!=0U) {
            uint32_t elapsed=model->confirm_elapsed_ms>800U?800U:model->confirm_elapsed_ms;
            object_set_pos(page->confirm_bar,16,207); object_set_size(page->confirm_bar,(int)(248U*elapsed/800U),3);
            style_set_color(page->confirm_bar,LV_STYLE_BG_COLOR,lv_color_hex(0x777777U));
            page->layout_confirm_visible=1U;
        }
        return;
    }
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_READY) {
        unsigned target=model->selected_motor<DEBUG_UI_MOTOR_COUNT?model->selected_motor:0U;
        uint8_t configured=(uint8_t)(model->single_static_configured && model->snapshot.motors[target].single_static_profile.configured);
        uint8_t local=(uint8_t)(model->snapshot.authority==DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
            model->snapshot.target_motor_id==target+1U && !model->single_static_wait_snapshot &&
            single_static_feedback_fresh(model,target) &&
            debug_ui_model_gate(model,DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)==DEBUG_UI_REASON_NONE);
        uint8_t locked=(uint8_t)(model->static_locked && model->draft.operation==DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
            model->draft.target_motor_id==target+1U && isfinite(model->draft.static_positions_rad[target]));
        instrument_slot(page,0U,16,35,248,21,page->row_text[0],&ui_font_16,primary,0U,0U);
        for (index=0U; index<3U; ++index) {
            uint8_t complete=index==0U?configured:(index==1U?local:(uint8_t)(locked && local));
            int top=62+(int)index*29;
            static_icon_add(page,complete?STATIC_ICON_CHECK:STATIC_ICON_EMPTY,16,top,1U,0U);
            instrument_slot(page,index+1U,53,top+4,212,22,page->row_text[index+1U],&ui_font_popup_12,complete?primary:muted,0U,0U);
        }
        instrument_slot(page,4U,16,153,248,18,page->row_text[4],&ui_font_popup_12,muted,0U,0U);
        instrument_slot(page,6U,16,172,248,16,page->row_text[6],&ui_font_popup_12,muted,0U,0U);
        static_panel_add(page,12,190,268,210,1U);
        instrument_slot(page,5U,24,191,232,18,page->row_text[5],&ui_font_popup_12,primary,1U,0U);
        return;
    }
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT) {
        uint8_t visible=static_report_visible(model);
        uint8_t disabled=(uint8_t)(visible && model->snapshot.static_load.disabled_confirmed);
        static_icon_add(page,disabled?STATIC_ICON_CHECK:STATIC_ICON_WARNING,16,35,1U,0U);
        instrument_slot(page,0U,54,37,212,22,page->row_text[0],&ui_font_16,primary,0U,0U);
        for (index=1U; index<7U; ++index) {
            static_icon_add(page,index==1U?STATIC_ICON_ANGLE:(index==2U?STATIC_ICON_TORQUE:
                (index==3U?STATIC_ICON_TEMP:(index==4U?STATIC_ICON_TIMER:(index==5U?STATIC_ICON_STOP:STATIC_ICON_CONTROL)))),
                16,61+(int)(index-1U)*24,1U,0U);
            instrument_slot(page,index,54,64+(int)(index-1U)*24,212,20,page->row_text[index],&ui_font_popup_12,
                index==6U?muted:primary,0U,0U);
        }
        if (visible) {
            const DebugUiStaticLoadReport *report=&model->snapshot.static_load;
            unsigned target=0U;
            for (index=0U;index<DEBUG_UI_MOTOR_COUNT;++index)
                if ((report->motor_mask&(1U<<index))!=0U) { target=index; break; }
            if ((report->valid_mask&(1U<<target))!=0U && report->samples[target]!=0U)
                (void)snprintf(value,sizeof(value),"保持计时 %.1f/%.0fs 样本%lu",
                    (double)report->elapsed_ms/1000.0,(double)report->duration_ms/1000.0,(unsigned long)report->samples[target]);
            else (void)snprintf(value,sizeof(value),"保持计时 --/%.0fs 样本--",(double)report->duration_ms/1000.0);
            instrument_slot(page,4U,54,136,212,20,value,&ui_font_popup_12,primary,0U,0U);
        }
        return;
    }
    /* Relock and cleanup failures retain every semantic line while leading with the true state icon. */
    static_icon_add(page,model->page==DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK && model->reason==DEBUG_UI_REASON_NONE?STATIC_ICON_LOCK:STATIC_ICON_WARNING,16,37,1U,0U);
    instrument_slot(page,0U,54,39,212,22,page->row_text[0],&ui_font_16,primary,0U,0U);
    for (index=1U;index<7U;++index)
        instrument_slot(page,index,16,65+(int)(index-1U)*24,248,21,page->row_text[index],&ui_font_popup_12,
            index==5U?primary:muted,0U,0U);
}

#include "debug_ui_mono_layout.inc"

/** @brief Apply shared Tile/List/Overlay templates without model mutation or allocations. */
static void layout(DebugUiViewPage *page, const DebugUiModel *model, int focus_row, uint8_t page_changed)
{
    static const lv_area_t home_area = {12,58,268,180};
    static const lv_area_t custom_area = {0,32,279,211};
    static const lv_area_t scrollbar_area = {270,38,274,198};
    unsigned index, count = astra_count(model, model->page);
    uint8_t old_decoration = page->decoration;
    uint8_t old_tile_focus = page->tile_focus;
    int16_t old_tile_offset = page->tile_offset;
    uint8_t old_scrollbar_count = page->scrollbar_count;
    uint8_t old_scrollbar_first = page->scrollbar_first;
    uint8_t old_popup_focus = page->popup_focus;
    int16_t old_popup_marker = page->popup_marker;
    DebugUiStaticArt old_static_art = page->static_art;
    char value[160];
    const DebugUiMotorView *motor = &model->snapshot.motors[model->selected_motor < 7U ? model->selected_motor : 0U];
    (void)focus_row;
    page->layout_row_mask = 0U; page->layout_icon_mask = 0U;
    page->layout_overlay_visible = 0U; page->layout_confirm_visible = 0U;
    page->layout_custom_changed = 0U;
    page->decoration = 0U; page->scrollbar_count = 0U;
    memset(&page->static_art, 0, sizeof(page->static_art));
    style_set_number(page->overlay, LV_STYLE_BG_OPA, LV_OPA_COVER);
    style_set_number(page->overlay, LV_STYLE_BORDER_WIDTH, 2);
    style_set_color(page->root, LV_STYLE_BG_COLOR, lv_color_hex(0x000000U));
    style_set_color(page->confirm_bar, LV_STYLE_BG_COLOR, lv_color_hex(0xFFFFFFU));
    if (model->page != DEBUG_UI_PAGE_OVERVIEW || model->snapshot.active || model->snapshot.pending || model->stop_requested || model->snapshot.arm_fault || !model->input_valid || !model->display_valid) {
        page->animation_active = 0U; page->tile_offset = 0;
    }
    if (pos_load_view_page(model)) pos_load_view_layout(page, model);
    else if (load_view_page(model->page)) load_view_layout(page, model);
    else if (single_static_instrument_page(model)) instrument_layout(page, model);
    else if (mono_layout(page, model, count)) { /* Reuse the fixed monochrome family slots. */ }
    else if (count != 0U) {
        unsigned first = model->list_first;
        if (count <= 5U) first = 0U; else if (first > count - 5U) first = count - 5U;
        page->scrollbar_count = (uint8_t)count; page->scrollbar_first = (uint8_t)first;
        for (index = 0U; index < 5U && first + index < count; ++index) {
            astra_entry(value, sizeof(value), model, model->page, first + index);
            slot(page, index, 12, 38+(int)index*32, 252, 30, value, NULL, (uint8_t)(first+index == model->focus), 0xFFFFFFU, &ui_font_16);
            style_set_number(page->rows[index], LV_STYLE_PAD_LEFT, 10); style_set_number(page->rows[index], LV_STYLE_PAD_TOP, 7);
        }
    } else switch (model->page) {
    case DEBUG_UI_PAGE_STATIC_LOAD:
    case DEBUG_UI_PAGE_STATIC_RESULT:
        if (model->page == DEBUG_UI_PAGE_STATIC_RESULT && model->static_result_retained &&
            model->snapshot.static_load.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
            for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
                slot(page, index, 10, 37+(int)index*24, 260, 23, page->row_text[index], NULL, 0U, 0xFFFFFFU, &ui_font_16);
            break;
        }
        for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
            slot(page, index, 10, 37+(int)index*24, 260, 23, page->row_text[index], NULL, 0U, 0xFFFFFFU, &ui_font_popup_12);
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_LOAD:
    case DEBUG_UI_PAGE_SINGLE_STATIC_RESULT:
        for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
            slot(page, index, 10, 37+(int)index*24, 260, 23, page->row_text[index], NULL, 0U, 0xFFFFFFU,
                model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT && model->static_result_retained &&
                model->snapshot.static_load.operation == DEBUG_UI_OPERATION_STATIC_HOLD ? &ui_font_popup_12 : &ui_font_16);
        break;
    case DEBUG_UI_PAGE_OVERVIEW:
        page->decoration = 1U;
        if (old_decoration == 1U && page->tile_focus != model->focus && !model->snapshot.active && !model->snapshot.pending && !model->stop_requested && !model->snapshot.arm_fault && model->input_valid && model->display_valid) {
            page->animation_from = (int16_t)(((page->tile_focus + 1U) % 4U == model->focus) ? 28 : -28);
            page->tile_offset = page->animation_from; page->animation_start = lv_tick_get(); page->animation_tick = page->animation_start; page->animation_active = 1U;
        }
        page->tile_focus = model->focus % 4U;
        slot(page, 0U, 12, 35, 256, 22, page->row_text[0], NULL, 0U, 0xAAAAAAU, &ui_font_16);
        slot(page, 1U, 80, 154, 190, 26, model->focus == 0U ? "电机" : (model->focus == 1U ? "诊断" : (model->focus == 2U ? "本地控制" : "静载测试")), NULL, 0U, 0xFFFFFFU, &ui_font_16);
        slot(page, 2U, 12, 186, 256, 22, page->row_text[2], NULL, 0U, 0xAAAAAAU, &ui_font_16);
        break;
    case DEBUG_UI_PAGE_NUMBER: {
        unsigned field = model->edit_field < 5U ? model->edit_field : 0U;
        unsigned first = model->page_first[DEBUG_UI_PAGE_EDIT];
        unsigned fields = debug_ui_model_parameter_count(model);
        static const char *const steps[] = {"每步 1.0°", "每步 1.0°/s", "每步 0.10", "每步 0.01", "每步 100 ms"};
        page->decoration = 3U;
        page->popup_focus = model->page_focus[DEBUG_UI_PAGE_EDIT] >= first ?
            (uint8_t)(model->page_focus[DEBUG_UI_PAGE_EDIT]-first) : 255U;
        page->popup_marker = -1;
        for (index=0U; index<5U; ++index) {
            char background[80] = {0};
            if (first+index < fields) {
                unsigned parent_field = debug_ui_model_parameter_field(model,(uint8_t)(first+index));
                (void)snprintf(background,40,"%s",astra_fields[parent_field]);
                astra_number(background+40,40,&model->draft,parent_field);
            } else if (first+index == fields) {
                (void)snprintf(background,40,"预览并执行");
                (void)snprintf(background+40,40,">");
            }
            astra_background_set(page,index,background);
        }
        slot(page, 0U, 28, 59, 224, 24, field == 1U ? "输出轴速度" : astra_fields[field], NULL, 0U, 0xFFFFFFU, &ui_font_popup_17);
        astra_popup_label(page->rows[0],&ui_font_popup_17,page->display_text[0],28,59,224,LV_TEXT_ALIGN_CENTER);
        astra_number(value, sizeof(value), &model->draft, field);
        slot(page, 1U, 30, 96, 220, 38, value, NULL, 0U, 0xFFFFFFU, &ui_font_popup_30);
        astra_popup_label(page->rows[1],&ui_font_popup_30,page->display_text[1],30,96,220,LV_TEXT_ALIGN_CENTER);
        {
            uint8_t mit = (uint8_t)(model->draft.operation == DEBUG_UI_OPERATION_MIT_MOVE || model->draft.operation == DEBUG_UI_OPERATION_MIT_HOLD);
            DebugUiMotorProfile bounds = debug_ui_profile_bounds(&motor->profile, mit);
            float minimum, maximum, current;
            if (field == 0U) { maximum = debug_ui_radians_to_degrees(bounds.max_delta_rad); minimum = -maximum; current = debug_ui_radians_to_degrees(model->draft.delta_rad); }
            else if (field == 1U) { maximum = debug_ui_radians_to_degrees(bounds.max_speed_rad_s); minimum = maximum < 1.0f ? maximum : 1.0f; current = debug_ui_radians_to_degrees(model->draft.speed_rad_s); }
            else if (field == 2U) { minimum = bounds.kp_min; maximum = bounds.kp_max; current = model->draft.kp; }
            else if (field == 3U) { minimum = bounds.kd_min; maximum = bounds.kd_max; current = model->draft.kd; }
            else {
                uint32_t minimum_ms = bounds.hold_min_ms, maximum_ms = bounds.hold_max_ms;
                if (model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD)
                    (void)debug_ui_model_static_duration_bounds(model, &minimum_ms, &maximum_ms);
                else if (model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)
                    (void)debug_ui_model_single_static_duration_bounds(model, &minimum_ms, &maximum_ms);
                minimum = (float)minimum_ms; maximum = (float)maximum_ms; current = (float)model->draft.hold_duration_ms;
            }
            if (isfinite(current) && isfinite(minimum) && isfinite(maximum) && maximum > minimum) {
                float fraction = field == 1U ? astra_speed_fraction(current,minimum,maximum) :
                    (current-minimum)/(maximum-minimum);
                if (fraction < 0.0f) fraction = 0.0f; else if (fraction > 1.0f) fraction = 1.0f;
                page->popup_marker = (int16_t)(39.0f+202.0f*fraction+(field == 1U ? 0.5f : 0.0f));
            } else if (isfinite(current) && isfinite(minimum) && maximum == minimum && current == minimum) page->popup_marker=140;
        }
        slot(page,2U,28,158,224,20,field == 4U && model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ?
            "每步 1000 ms" : steps[field],NULL,0U,0xFFFFFFU,&ui_font_popup_12);
        astra_popup_label(page->rows[2],&ui_font_popup_12,page->display_text[2],28,158,224,LV_TEXT_ALIGN_CENTER);
        slot(page,3U,28,184,224,20,"仅修改草稿",NULL,0U,0xFFFFFFU,&ui_font_popup_12);
        astra_popup_label(page->rows[3],&ui_font_popup_12,page->display_text[3],28,184,224,LV_TEXT_ALIGN_CENTER);
        break;
    }
    case DEBUG_UI_PAGE_REVIEW:
        astra_parent(page, model, model->review_return_page);
        object_set_pos(page->overlay, 16, 38); object_set_size(page->overlay, 248, 170);
        for (index = 0U; index < 6U; ++index)
            slot(page, index, 24, 44+(int)index*22, 232, 22, page->row_text[index], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        slot(page, 6U, 24, 179, 232, 24, page->row_text[6], NULL, 1U, 0xFFFFFFU, &ui_font_16);
        if (model->review_released && model->confirm_elapsed_ms > 0U) {
            uint32_t elapsed = model->confirm_elapsed_ms > 800U ? 800U : model->confirm_elapsed_ms;
            object_set_pos(page->confirm_bar, 24, 204);
            object_set_size(page->confirm_bar, (int)(232U*elapsed/800U), lv_obj_get_height(page->confirm_bar));
            page->layout_confirm_visible = 1U;
        }
        break;
    case DEBUG_UI_PAGE_NOTICE:
        astra_parent(page, model, model->notice_return_page);
        slot(page, 0U, 28, 66, 224, 30, "操作未提交", LV_SYMBOL_WARNING, 0U, 0xFFFFFFU, &ui_font_16);
        slot(page, 1U, 28, 111, 224, 44, debug_ui_reason_text(model->reason), NULL, 0U, 0xFFFFFFU, &ui_font_16);
        slot(page, 2U, 28, 174, 224, 25, "返回", LV_SYMBOL_LEFT, 1U, 0xFFFFFFU, &ui_font_16);
        break;
    case DEBUG_UI_PAGE_RESULT:
    case DEBUG_UI_PAGE_FAULT:
        astra_parent(page, model, DEBUG_UI_PAGE_ACTIONS);
        slot(page, 0U, 26, 57, 228, 24, page->row_text[0], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        slot(page, 1U, 26, 87, 228, 23, page->row_text[1], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        slot(page, 2U, 26, 116, 228, 23, page->row_text[3], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        slot(page, 3U, 26, 145, 228, 23, page->row_text[4], NULL, 0U, 0xAAAAAAU, &ui_font_16);
        slot(page, 4U, 26, 175, 228, 24, debug_ui_model_result_requires_check(model) ? "进入控制检查" : "已读返回", NULL, 1U, 0xFFFFFFU, &ui_font_16);
        break;
    case DEBUG_UI_PAGE_DETAIL:
        slot(page, 0U, 12, 35, 256, 23, page->row_text[0], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        if (!motor->feedback_valid && debug_ui_raw_position_current(model,motor)) {
            slot(page, 1U, 12, 64, 256, 34, page->row_text[1], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        } else if (motor->feedback_seen && !motor->feedback_valid) {
            float last_degrees = debug_ui_radians_to_degrees(motor->position_rad);
            if (isfinite(last_degrees)) (void)snprintf(value, sizeof(value), "上次 %.1f° / %lums", (double)last_degrees, (unsigned long)motor->feedback_age_ms);
            else (void)snprintf(value, sizeof(value), "位置 -- / 反馈无效");
            slot(page, 1U, 12, 64, 256, 34, value, NULL, 0U, 0xAAAAAAU, &ui_font_16);
        } else {
            debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(motor->position_rad), motor->feedback_valid, motor->feedback_valid, "°");
            slot(page, 1U, 12, 64, 256, 36, value, NULL, 0U, 0xFFFFFFU, &ui_font_numeric_28);
        }
        for (index = 2U; index < 6U; ++index) slot(page, index, 12, 107+(int)(index-2U)*24, 256, 23, page->row_text[index], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        break;
    case DEBUG_UI_PAGE_RUNNING: {
        if (static_load_running(model)) {
            for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
                slot(page, index, 10, 37+(int)index*24, 260, 23, page->row_text[index], NULL, 0U, 0xFFFFFFU,
                    single_static_running(model) ? &ui_font_16 : &ui_font_popup_12);
            break;
        }
        uint8_t target = debug_ui_model_active_target(model);
        const DebugUiMotorView *active_motor = &model->snapshot.motors[target > 0U && target <= 7U ? target-1U : 0U];
        uint8_t seen = (uint8_t)(target != 0U && active_motor->feedback_seen && isfinite(debug_ui_radians_to_degrees(active_motor->position_rad)));
        slot(page, 0U, 12, 35, 256, 21, page->row_text[0], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        slot(page, 1U, 12, 58, 256, 23, page->row_text[1], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        slot(page, 2U, 12, 82, 256, 23, page->row_text[2], NULL, 0U, 0xAAAAAAU, &ui_font_16);
        /* Last samples remain numeric only beside an explicit age/status label. */
        debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(active_motor->position_rad), seen, seen, "°");
        slot(page, 3U, 12, 106, 256, 34, value, NULL, 0U, 0xFFFFFFU, &ui_font_numeric_28);
        if (seen) (void)snprintf(value, sizeof(value), "%s / %lums", active_motor->feedback_valid ? "输出轴当前位置" : "上次输出轴位置", (unsigned long)active_motor->feedback_age_ms);
        else (void)snprintf(value, sizeof(value), "输出轴位置未知");
        slot(page, 6U, 12, 140, 256, 21, value, NULL, 0U, 0xAAAAAAU, &ui_font_16);
        slot(page, 4U, 12, 163, 256, 26, page->row_text[4], NULL, 1U, 0xFFFFFFU, &ui_font_16);
        slot(page, 5U, 12, 190, 256, 21, page->row_text[6], NULL, 0U, 0xAAAAAAU, &ui_font_16);
        break;
    }
    default:
        for (index = 0U; index < 7U; ++index) slot(page, index, 12, 36+(int)index*24, 256, 23, page->row_text[index], NULL, 0U, 0xFFFFFFU, &ui_font_16);
        break;
    }
    if (memcmp(&old_static_art, &page->static_art, sizeof(old_static_art)) != 0) page->layout_custom_changed = 1U;
    if (model->page != DEBUG_UI_PAGE_REVIEW) { object_set_pos(page->overlay, 16, 48); object_set_size(page->overlay, 248, 160); }
    if (page_changed || old_decoration != page->decoration) {
        lv_obj_invalidate(page->root);
    } else {
        if (old_tile_focus != page->tile_focus || old_tile_offset != page->tile_offset)
            lv_obj_invalidate_area(page->root, &home_area);
        if (old_scrollbar_count != page->scrollbar_count || old_scrollbar_first != page->scrollbar_first)
            lv_obj_invalidate_area(page->root, &scrollbar_area);
        if (page->layout_custom_changed || old_popup_focus != page->popup_focus || old_popup_marker != page->popup_marker)
            lv_obj_invalidate_area(page->root, &custom_area);
    }
    page->layout_initialized = 1U;
}
