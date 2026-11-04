/** @file debug_ui_view.c
 * @brief Shared LVGL v8 Astra-style view; no page allocations after initialization.
 * Semantic rows retain evidence, while bounded display slots implement templates.
 */
#include "debug_ui_view.h"
#include "../App/DebugUi/debug_ui_pos_load_units.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(__CC_ARM)
__attribute__((section(".lvgl_pool"), zero_init, aligned(32)))
#else
__attribute__((section(".lvgl_pool"), aligned(32)))
#endif
uint8_t debug_ui_lvgl_pool[LV_MEM_SIZE];
static volatile uint8_t graphics_fault;
/** @brief Draw allocation-free Astra decorations behind shared objects. */
static void astra_draw(lv_event_t *event);
/** @brief Check guided timing/report provenance before its later fixed-slot layout is included. */
static uint8_t load_view_timed_report(const DebugUiModel *model);

/** @brief Record local graphics failure; guarded entrypoints stop UI execution. */
void debug_ui_lvgl_assert_failed(void)
{
    graphics_fault = 1U;
    /* Task installs a local escape hook before calling LVGL. */
    extern void debug_ui_graphics_abort(void);
    debug_ui_graphics_abort();
}

/** @brief Read the latched local graphics fault without touching any LVGL object. */
uint8_t debug_ui_view_healthy(void) { return (uint8_t)!graphics_fault; }

/** @brief Set a fixed string only when it changes; never hand LVGL a stack buffer. */
static void text_set(lv_obj_t *label, char *storage, size_t capacity, const char *text)
{
    if (strcmp(storage, text) == 0) return;
    (void)snprintf(storage, capacity, "%s", text);
    lv_label_set_text_static(label, storage);
}

/** @brief Move an object only when its requested coordinates changed. */
static void object_set_pos(lv_obj_t *object, int x, int y)
{
    /* LVGL compares the local X/Y style values before refreshing coordinates. */
    lv_obj_set_pos(object, (lv_coord_t)x, (lv_coord_t)y);
}

/** @brief Resize an object only when its requested dimensions changed. */
static void object_set_size(lv_obj_t *object, int width, int height)
{
    /* LVGL compares the local width/height style values before invalidating. */
    lv_obj_set_size(object, (lv_coord_t)width, (lv_coord_t)height);
}

/** @brief Change one numeric local style property only when its value changed. */
static void style_set_number(lv_obj_t *object, lv_style_prop_t property, int32_t number)
{
    lv_style_value_t current, requested;
    requested.num = number;
    if (lv_obj_get_local_style_prop(object, property, &current, 0) == LV_STYLE_RES_FOUND &&
        current.num == requested.num) return;
    lv_obj_set_local_style_prop(object, property, requested, 0);
}

/** @brief Change one pointer local style property only when its value changed. */
static void style_set_pointer(lv_obj_t *object, lv_style_prop_t property, const void *pointer)
{
    lv_style_value_t current, requested;
    requested.ptr = pointer;
    if (lv_obj_get_local_style_prop(object, property, &current, 0) == LV_STYLE_RES_FOUND &&
        current.ptr == requested.ptr) return;
    lv_obj_set_local_style_prop(object, property, requested, 0);
}

/** @brief Change one color local style property only when its RGB565 value changed. */
static void style_set_color(lv_obj_t *object, lv_style_prop_t property, lv_color_t color)
{
    lv_style_value_t current, requested;
    requested.color = color;
    if (lv_obj_get_local_style_prop(object, property, &current, 0) == LV_STYLE_RES_FOUND &&
        lv_color_to16(current.color) == lv_color_to16(requested.color)) return;
    lv_obj_set_local_style_prop(object, property, requested, 0);
}
/** @brief Create one bounded, non-scrolling label. */
static lv_obj_t *label_create(lv_obj_t *parent, int x, int y, int width, int height)
{
    lv_obj_t *label = lv_label_create(parent);
    if (label == NULL) return NULL;
    lv_obj_set_pos(label, (lv_coord_t)x, (lv_coord_t)y);
    lv_obj_set_size(label, (lv_coord_t)width, (lv_coord_t)height);
    lv_obj_set_style_text_font(label, &ui_font_16, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_pad_left(label, 4, 0);
    lv_obj_set_style_pad_top(label, height < 23 ? 0 : 2, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return label;
}

/** @brief Allocate one screen, seven text/icon pairs and a confirmation indicator. */
uint8_t debug_ui_view_init(DebugUiView *view)
{
    unsigned row_index;
    DebugUiViewPage *page;
    if (view == NULL) return 0U;
    memset(view, 0, sizeof(*view));
    view->group = lv_group_create();
    if (view->group == NULL) return 0U;
    lv_group_set_wrap(view->group, false);
    page = &view->page;
    {
        page->root = lv_obj_create(NULL);
        if (page->root == NULL) return 0U;
        lv_obj_remove_style_all(page->root);
        lv_obj_set_size(page->root, 280, 240);
        lv_obj_set_style_bg_opa(page->root, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(page->root, lv_color_black(), 0);
        lv_obj_clear_flag(page->root, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(page->root, astra_draw, LV_EVENT_DRAW_MAIN_END, page);
        page->overlay = lv_obj_create(page->root);
        if (page->overlay == NULL) return 0U;
        lv_obj_remove_style_all(page->overlay);
        lv_obj_set_pos(page->overlay, 16, 48);
        lv_obj_set_size(page->overlay, 248, 160);
        lv_obj_set_style_bg_color(page->overlay, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(page->overlay, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(page->overlay, lv_color_white(), 0);
        lv_obj_set_style_border_width(page->overlay, 2, 0);
        lv_obj_set_style_radius(page->overlay, 8, 0);
        lv_obj_add_flag(page->overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(page->overlay, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        page->title = label_create(page->root, 8, 2, 264, 26);
        page->footer = label_create(page->root, 8, 212, 264, 26);
        if (page->title == NULL || page->footer == NULL) return 0U;
        lv_obj_set_style_bg_opa(page->title, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(page->title, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(page->footer, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(page->footer, lv_color_black(), 0);
        lv_obj_set_style_text_color(page->footer, lv_color_white(), 0);
        for (row_index = 0U; row_index < DEBUG_UI_VIEW_ROWS; ++row_index) {
            page->rows[row_index] = label_create(page->root, 8, 34 + (int)row_index * 24, 264, 24);
            page->icons[row_index] = label_create(page->root, 16, 34, 28, 30);
            if (page->rows[row_index] == NULL || page->icons[row_index] == NULL) return 0U;
            lv_obj_set_style_text_font(page->icons[row_index], &lv_font_montserrat_20, 0);
            lv_obj_set_style_text_color(page->icons[row_index], lv_color_white(), 0);
            lv_label_set_text_static(page->rows[row_index], page->display_text[row_index]);
            lv_label_set_text_static(page->icons[row_index], "");
        }
        page->confirm_bar = lv_obj_create(page->root);
        if (page->confirm_bar == NULL) return 0U;
        lv_obj_remove_style_all(page->confirm_bar);
        lv_obj_set_pos(page->confirm_bar, 10, 207);
        lv_obj_set_size(page->confirm_bar, 1, 3);
        lv_obj_set_style_bg_opa(page->confirm_bar, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(page->confirm_bar, lv_color_white(), 0);
        lv_obj_add_flag(page->confirm_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text_static(page->title, page->title_text);
        lv_label_set_text_static(page->footer, page->footer_text);
    }
    view->visible = DEBUG_UI_PAGE_OVERVIEW;
    lv_scr_load(page->root);
    view->initialized = debug_ui_view_healthy();
    return view->initialized;
}

/** @brief Bound display-only register age including time since the shared snapshot. */
static uint8_t debug_ui_raw_position_current(const DebugUiModel *model, const DebugUiMotorView *motor)
{
    return (uint8_t)(model->snapshot_seen && model->now_us >= model->snapshot.timestamp_us &&
        model->now_us - model->snapshot.timestamp_us < DEBUG_UI_HEALTH_TIMEOUT_US &&
        motor->register_position_contract == 1U &&
        isfinite(motor->single_static_native_per_output) && motor->single_static_native_per_output > 0.0F &&
        (motor->register_seen_mask & 1U) && isfinite(motor->register_position[0]) &&
        (uint64_t)motor->register_age_ms[0] + (model->now_us - model->snapshot.timestamp_us) / 1000ULL < 500ULL);
}

/** @brief Convert verified rotor p_m radians to the LCD's existing output-shaft degrees. */
static float debug_ui_register_output_degrees(const DebugUiMotorView *motor)
{
    return debug_ui_radians_to_degrees(motor->register_position[0] / motor->single_static_native_per_output);
}

/** @brief Expose a finite display estimate only while its source position remains current. */
static uint8_t debug_ui_estimated_speed_current(const DebugUiModel *model, const DebugUiMotorView *motor)
{
    return (uint8_t)(debug_ui_raw_position_current(model, motor) && motor->register_estimated_velocity_valid &&
        isfinite(motor->register_estimated_native_rad_s));
}

/** @brief Translate verified actual modes; zero is explicitly unknown. */
static const char *mode_text(DebugUiMode mode)
{
    return mode == DEBUG_UI_MODE_MIT ? "MIT" : (mode == DEBUG_UI_MODE_POS_VEL ? "POS" : "未知");
}
/** @brief Name bounded operations without implying they were executed. */
static const char *operation_text(DebugUiOperation operation)
{
    static const char *const names[] = { "申请本地控制", "释放本地控制", "POS 相对移动",
        "MIT 有限保持", "MIT 相对移动", "切换模式", "清除故障", "请求失能", "STOP 停止", "七轴静载保持", "单轴静载保持" };
    return (unsigned)operation < sizeof(names) / sizeof(names[0]) ? names[operation] : "未知操作";
}
/** @brief Display the application phase, separately from mailbox admission. */
static const char *stage_text(DebugUiStage stage)
{
    static const char *const names[] = { "等待进度", "校验参数", "发现电机", "切换模式", "清除故障",
        "正在使能", "正在运动", "有限保持", "失能清理" };
    return (unsigned)stage < sizeof(names) / sizeof(names[0]) ? names[stage] : "未知阶段";
}
/** @brief Count verified identities independently of current feedback freshness. */
static unsigned discovered_count(const DebugUiSnapshot *snapshot)
{
    unsigned index, count = 0U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        if (snapshot->motors[index].identity_verified) ++count;
    return count;
}
/** @brief Count fresh feedback only; unknown and stale motors remain in the list. */
static unsigned online_count(const DebugUiSnapshot *snapshot)
{
    unsigned index, count = 0U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        if (snapshot->motors[index].feedback_valid) ++count;
    return count;
}
/** @brief Count actual activity and outstanding cleanup targets without using focus. */
static unsigned active_target_count(const DebugUiModel *model)
{
    unsigned index, count = 0U;
    uint8_t mask = model->snapshot.active_motor_mask;
    if (model->stop_requested) mask |= model->stop_motor_mask;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        if ((mask & (1U << index)) != 0U) ++count;
    return count;
}
/** @brief Set one row from transient text through stable per-page storage. */
static void row(DebugUiViewPage *page, unsigned index, const char *text)
{
    (void)snprintf(page->row_text[index], sizeof(page->row_text[index]), "%s", text);
}
/** @brief Change visibility only when needed, avoiding needless full-screen refresh. */
static void show(lv_obj_t *object, uint8_t visible)
{
    uint8_t hidden = (uint8_t)lv_obj_has_flag(object, LV_OBJ_FLAG_HIDDEN);
    if (visible && hidden) lv_obj_clear_flag(object, LV_OBJ_FLAG_HIDDEN);
    else if (!visible && !hidden) lv_obj_add_flag(object, LV_OBJ_FLAG_HIDDEN);
}

/** @brief Render one bounded slot; symbols use LVGL's bundled font, not the CJK font. */
static void slot(DebugUiViewPage *page, unsigned index, int x, int y, int width, int height,
                 const char *value, const char *symbol, uint8_t selected, uint32_t color,
                 const lv_font_t *font)
{
    lv_obj_t *label = page->rows[index];
    lv_obj_t *glyph = page->icons[index];
    assert(index < DEBUG_UI_VIEW_ROWS);
    page->layout_row_mask |= (uint8_t)(1U << index);
    if (symbol != NULL) page->layout_icon_mask |= (uint8_t)(1U << index);
    object_set_pos(label, x, y);
    object_set_size(label, width, height);
    style_set_pointer(label, LV_STYLE_TEXT_FONT, font);
    style_set_number(label, LV_STYLE_TEXT_ALIGN, LV_TEXT_ALIGN_LEFT);
    style_set_number(label, LV_STYLE_PAD_LEFT, symbol != NULL ? 34 : 4);
    style_set_number(label, LV_STYLE_PAD_TOP, height < 23 ? 0 : 2);
    style_set_color(label, LV_STYLE_TEXT_COLOR, lv_color_hex(selected ? 0x000000U : color));
    style_set_color(label, LV_STYLE_BG_COLOR, lv_color_hex(selected ? 0xFFFFFFU : 0x000000U));
    style_set_number(label, LV_STYLE_BG_OPA, LV_OPA_COVER);
    style_set_color(label, LV_STYLE_BORDER_COLOR, lv_color_white());
    style_set_number(label, LV_STYLE_BORDER_WIDTH, 0);
    style_set_number(label, LV_STYLE_RADIUS, selected ? 5 : 0);
    text_set(label, page->display_text[index], sizeof(page->display_text[index]), value);
    if (symbol != NULL) {
        object_set_pos(glyph, x + 5, y + 1);
        object_set_size(glyph, lv_obj_get_width(glyph), height < 30 ? height - 1 : 29);
        style_set_pointer(glyph, LV_STYLE_TEXT_FONT, height < 26 ? &lv_font_montserrat_16 : &lv_font_montserrat_20);
        style_set_number(glyph, LV_STYLE_PAD_TOP, 0);
        style_set_color(glyph, LV_STYLE_TEXT_COLOR, lv_color_hex(selected ? 0x000000U : color));
        if (strcmp(lv_label_get_text(glyph), symbol) != 0) lv_label_set_text_static(glyph, symbol);
    }
}

/** @brief Compare the complete nonzero provenance key before displaying retained measurements. */
static uint8_t view_same_identity(const DebugUiIdentity *left, const DebugUiIdentity *right)
{
    return (uint8_t)(left->request_id != 0U && left->request_id == right->request_id &&
        left->origin == right->origin && left->epoch == right->epoch);
}

/** @brief Require a valid operation-specific selection before trusting a retained report. */
static uint8_t static_report_scope_valid(const DebugUiStaticLoadReport *report)
{
    uint8_t mask = report->motor_mask;
    if (report->operation == DEBUG_UI_OPERATION_STATIC_HOLD) return (uint8_t)(mask == 0x7FU);
    return (uint8_t)(report->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && mask != 0U &&
        (mask & 0x80U) == 0U && (mask & (mask - 1U)) == 0U);
}

/** @brief Permit explicit scoped history; current reports require identity, operation and ID. */
static uint8_t static_report_visible(const DebugUiModel *model)
{
    const DebugUiStaticLoadReport *report = &model->snapshot.static_load;
    uint8_t expected_mask = model->submitted.operation == DEBUG_UI_OPERATION_STATIC_HOLD ? 0x7FU :
        (model->submitted.target_motor_id > 0U && model->submitted.target_motor_id <= DEBUG_UI_MOTOR_COUNT ?
         (uint8_t)(1U << (model->submitted.target_motor_id - 1U)) : 0U);
    return (uint8_t)(static_report_scope_valid(report) && (model->static_result_retained ||
        (debug_ui_is_static_operation(model->submitted.operation) && report->operation == model->submitted.operation &&
         report->motor_mask == expected_mask && view_same_identity(&model->submitted.identity, &report->identity))));
}

/** @brief Explain the retained trial's first exit without borrowing a later STOP completion. */
static uint8_t static_report_exit_text(char *text, size_t capacity, const DebugUiModel *model)
{
    const DebugUiStaticLoadReport *report = &model->snapshot.static_load;
    const char *cause;
    const char *phase;
    if (!static_report_visible(model) || !report->first_exit_latched) return 0U;
    phase = report->first_exit_phase == DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER ? "前馈接入" :
        (report->first_exit_phase == DEBUG_UI_STATIC_LOAD_PHASE_HOLD ? "保持" : "准备");
    if (report->first_exit_reason != DEBUG_UI_REASON_NONE) cause = debug_ui_reason_text(report->first_exit_reason);
    else if (report->first_exit_code == DEBUG_UI_COMPLETED) cause = "到期";
    else if (report->first_exit_code == DEBUG_UI_STOPPED) cause = "停止";
    else if (report->first_exit_code == DEBUG_UI_CANCELLED) cause = "已取消";
    else {
        /* Stable protocol error values are retained verbatim in the display-only contract. */
        switch (report->first_exit_error) {
        case 1U: cause = "尚未就绪"; break;
        case 2U: cause = "超出范围"; break;
        case 3U: cause = "速度超限"; break;
        case 4U: cause = "驱动故障"; break;
        case 5U: case 7U: cause = "反馈过期"; break;
        case 6U: cause = "超时"; break;
        default: cause = "动作失败"; break;
        }
    }
    (void)snprintf(text, capacity, "首因 %s / %s", phase, cause);
    return 1U;
}

/** @brief Format bounded seven-row live/retained evidence; stale numbers never appear valid. */
static void static_load_rows(DebugUiViewPage *page, const DebugUiModel *model, uint8_t result)
{
    unsigned index;
    const DebugUiStaticLoadReport *report = &model->snapshot.static_load;
    uint8_t coherent = (uint8_t)(model->snapshot_seen && model->now_us >= model->snapshot.timestamp_us &&
        model->now_us - model->snapshot.timestamp_us < DEBUG_UI_HEALTH_TIMEOUT_US);
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        char text[DEBUG_UI_VIEW_TEXT_BYTES];
        const DebugUiMotorView *motor = &model->snapshot.motors[index];
        uint8_t fresh = (uint8_t)(coherent && motor->motor_id == index + 1U && motor->feedback_seen &&
            motor->feedback_valid && motor->feedback_age_ms < 150U && isfinite(motor->position_rad) &&
            isfinite(motor->torque_nm) && isfinite(motor->mos_temperature_c) && isfinite(motor->rotor_temperature_c));
        if (result) {
            uint8_t observed = (uint8_t)(static_report_visible(model) && report->identity.request_id != 0U &&
                (report->valid_mask & (1U << index)) && report->samples[index] != 0U);
            if (!observed) (void)snprintf(text, sizeof(text), "M%02u -- / 未观测", index + 1U);
            else if (model->static_result_detail == 2U)
                (void)snprintf(text, sizeof(text), "M%02u N%lu", index + 1U, (unsigned long)report->samples[index]);
            else if (model->static_result_detail == 1U && isfinite(report->peak_mos_c[index]) && isfinite(report->peak_rotor_c[index]))
                (void)snprintf(text, sizeof(text), "M%02u MOS%.3g°C ROT%.3g°C", index + 1U,
                    (double)report->peak_mos_c[index], (double)report->peak_rotor_c[index]);
            else if (model->static_result_detail == 0U && isfinite(debug_ui_radians_to_degrees(report->peak_error_rad[index])) && isfinite(report->peak_torque_nm[index]))
                (void)snprintf(text, sizeof(text), "M%02u e%.3g° T%.3gNm", index + 1U,
                    (double)debug_ui_radians_to_degrees(report->peak_error_rad[index]), (double)report->peak_torque_nm[index]);
            else (void)snprintf(text, sizeof(text), "M%02u -- / 数据无效", index + 1U);
        } else if (!fresh) {
            (void)snprintf(text, sizeof(text), "M%02u %s e-- T-- --/--", index + 1U, motor->feedback_seen ? "过期" : "未知");
        } else {
            uint8_t active_group = (uint8_t)(report->active && report->operation == DEBUG_UI_OPERATION_STATIC_HOLD && report->motor_mask == 0x7FU);
            uint8_t locked_group = (uint8_t)(model->static_locked && model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD);
            float target = active_group ? report->target_position_rad[index] : model->draft.static_positions_rad[index];
            float error = debug_ui_radians_to_degrees(motor->position_rad - target);
            char error_text[24];
            if ((active_group || locked_group) && isfinite(error))
                (void)snprintf(error_text, sizeof(error_text), "%+.3g", (double)error);
            else (void)snprintf(error_text, sizeof(error_text), "--");
            (void)snprintf(text, sizeof(text), "M%02u %s e%s° T%.3gNm %.3g/%.3g°C", index + 1U,
                motor->fault_flags ? "故障" : (motor->enabled ? "使能" : (motor->driver_state == 0U ? "失能" : "未知")),
                error_text, (double)motor->torque_nm, (double)motor->mos_temperature_c, (double)motor->rotor_temperature_c);
        }
        row(page, index, text);
    }
}

/** @brief Recognize the static runtime without mistaking retained reports for active work. */
static uint8_t static_load_running(const DebugUiModel *model)
{
    return (uint8_t)((model->snapshot.static_load.active && static_report_scope_valid(&model->snapshot.static_load)) ||
        (debug_ui_is_static_operation(model->submitted.operation) && (model->awaiting_result || model->stop_waiting)));
}

/** @brief Resolve the actual active static mode ahead of older submitted work. */
static uint8_t single_static_running(const DebugUiModel *model)
{
    if (model->single_static_flow && model->submitted.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        (model->awaiting_result || model->stop_waiting) &&
        (view_same_identity(&model->snapshot.active_identity, &model->submitted.identity) ||
         view_same_identity(&model->snapshot.static_load.identity, &model->submitted.identity))) return 1U;
    if (model->snapshot.static_load.active && static_report_scope_valid(&model->snapshot.static_load))
        return (uint8_t)(model->snapshot.static_load.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
    return (uint8_t)(model->submitted.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        (model->awaiting_result || model->stop_waiting));
}

/** @brief Render selected-axis live values or causal report evidence in seven bounded rows. */
static void single_static_rows(DebugUiViewPage *page, const DebugUiModel *model, uint8_t result)
{
    const DebugUiSnapshot *snapshot = &model->snapshot;
    const DebugUiStaticLoadReport *report = &snapshot->static_load;
    uint8_t visible = static_report_visible(model);
    uint8_t runtime = single_static_running(model);
    unsigned index = model->selected_motor < DEBUG_UI_MOTOR_COUNT ? model->selected_motor : 0U;
    const DebugUiMotorView *motor;
    char text[DEBUG_UI_VIEW_TEXT_BYTES];
    if ((result && visible) || (runtime && report->active && static_report_scope_valid(report) &&
        (!model->single_static_flow || load_view_timed_report(model)))) {
        for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
            if ((report->motor_mask & (1U << index)) != 0U) break;
    } else if (model->submitted.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        (result || runtime) && model->submitted.target_motor_id > 0U && model->submitted.target_motor_id <= DEBUG_UI_MOTOR_COUNT)
        index = model->submitted.target_motor_id - 1U;
    if (index >= DEBUG_UI_MOTOR_COUNT) index = 0U;
    motor = &snapshot->motors[index];
    if (result) {
        uint8_t observed = (uint8_t)(visible && report->identity.request_id != 0U &&
            (report->valid_mask & (1U << index)) != 0U && report->samples[index] != 0U);
        (void)snprintf(text, sizeof(text), visible ? "单轴 ID%u / RQ%lu" : "ID%u / 等待本次结果", index + 1U,
            (unsigned long)report->identity.request_id); row(page, 0U, text);
        if (!visible && report->identity.request_id != 0U &&
            view_same_identity(&model->submitted.identity, &report->identity))
            row(page, 0U, report->operation != model->submitted.operation ? "结果模式不符 / 已隐藏" : "结果ID不符 / 已隐藏");
        if (observed && isfinite(debug_ui_radians_to_degrees(report->peak_error_rad[index]))) {
            /* Report observed precision independently of trial completion and the ID7 HOLD trip. */
            const char *format = "偏差峰值 %.3g°";
            if (index == 6U && report->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)
                format = report->peak_error_rad[index] > debug_ui_degrees_to_radians(0.5f) ?
                    "偏差峰值 %.3g° >0.5°" : "偏差峰值 %.3g° <=0.5°";
            (void)snprintf(text, sizeof(text), format, (double)debug_ui_radians_to_degrees(report->peak_error_rad[index]));
        }
        else (void)snprintf(text, sizeof(text), "偏差峰值 --");
        row(page, 1U, text);
        if (observed && isfinite(report->peak_torque_nm[index]))
            (void)snprintf(text, sizeof(text), "转矩峰值 %.3g Nm", (double)report->peak_torque_nm[index]);
        else (void)snprintf(text, sizeof(text), "转矩峰值 --");
        row(page, 2U, text);
        if (observed && isfinite(report->peak_mos_c[index]) && isfinite(report->peak_rotor_c[index]))
            (void)snprintf(text, sizeof(text), "MOS %.3g / 转子 %.3g°C", (double)report->peak_mos_c[index], (double)report->peak_rotor_c[index]);
        else (void)snprintf(text, sizeof(text), "MOS -- / 转子 --");
        row(page, 3U, text);
        if (observed) (void)snprintf(text, sizeof(text), "有效采样 %lu", (unsigned long)report->samples[index]);
        else (void)snprintf(text, sizeof(text), "有效采样 -- / 未观测");
        row(page, 4U, text);
        row(page, 5U, !visible ? "停止确认 / --" : (report->disabled_confirmed ? "停止确认 / 已失能" : "停止确认 / 待确认"));
    } else {
        uint8_t fresh = (uint8_t)(model->snapshot_seen && model->now_us >= snapshot->timestamp_us &&
            model->now_us - snapshot->timestamp_us < DEBUG_UI_HEALTH_TIMEOUT_US && motor->motor_id == index + 1U &&
            motor->feedback_seen && motor->feedback_valid && motor->feedback_age_ms < 150U &&
            isfinite(motor->position_rad) && isfinite(motor->torque_nm) &&
            isfinite(motor->mos_temperature_c) && isfinite(motor->rotor_temperature_c));
        uint8_t locked = (uint8_t)(model->static_locked && model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
            model->draft.target_motor_id == index + 1U);
        DebugUiReason reason = debug_ui_model_gate(model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
        const char *state = runtime ? (model->stop_requested ? "停止待确认" : (report->takeover_active ? "前馈接入" : (report->active ? "运行中" : "等待应用"))) :
            (reason == DEBUG_UI_REASON_UNCALIBRATED && !motor->single_static_profile.configured ? "长时未配置" :
             (reason == DEBUG_UI_REASON_NONE ? (locked ? "已锁定" : "未锁定") : debug_ui_reason_text(reason)));
        (void)snprintf(text, sizeof(text), "ID%u %s", index + 1U, state); row(page, 0U, text);
        if (fresh && ((runtime && report->active && (!model->single_static_flow || load_view_timed_report(model))) ||
            (locked && (!runtime || !model->single_static_flow)))) {
            float target = runtime && report->active ? report->target_position_rad[index] : model->draft.static_positions_rad[index];
            float error = debug_ui_radians_to_degrees(motor->position_rad - target);
            if (isfinite(error)) (void)snprintf(text, sizeof(text), "输出轴偏差 %+.3g°", (double)error);
            else (void)snprintf(text, sizeof(text), "输出轴偏差 --");
        } else (void)snprintf(text, sizeof(text), "输出轴偏差 --");
        row(page, 1U, text);
        if (fresh) (void)snprintf(text, sizeof(text), "反馈转矩 %.3g Nm", (double)motor->torque_nm);
        else (void)snprintf(text, sizeof(text), "反馈转矩 -- / 过期");
        row(page, 2U, text);
        if (fresh) (void)snprintf(text, sizeof(text), "MOS %.3g / 转子 %.3g°C", (double)motor->mos_temperature_c, (double)motor->rotor_temperature_c);
        else (void)snprintf(text, sizeof(text), "MOS -- / 转子 --");
        row(page, 3U, text);
        (void)snprintf(text, sizeof(text), "计时 %lu/%lu ms", (unsigned long)(runtime && report->active ? report->elapsed_ms : 0U),
            (unsigned long)(runtime ? model->submitted.hold_duration_ms :
             (model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ? model->draft.hold_duration_ms : 0U))); row(page, 4U, text);
        row(page, 5U, runtime ? "中键 STOP / 请求失能" : "先承托 / 到期仍失能");
    }
    row(page, 6U, snapshot->unconfirmed_disable_mask != 0U || model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED ?
        "当前控制 / 仍锁定" : (snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? "当前控制 / 本地" : "当前控制 / 未获本地"));
    if (!result && runtime && visible && (report->valid_mask & (1U << index)) != 0U && report->samples[index] &&
        isfinite(debug_ui_radians_to_degrees(report->peak_error_rad[index])) && isfinite(report->peak_torque_nm[index])) {
        const char *control = snapshot->unconfirmed_disable_mask != 0U || model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED ?
            "锁定" : (snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? "本地" : "未获");
        (void)snprintf(text, sizeof(text), "峰值 %.2g°/%.2gNm 控制%s",
            (double)debug_ui_radians_to_degrees(report->peak_error_rad[index]),
            (double)report->peak_torque_nm[index], control);
        row(page, 6U, text);
    }
}

/** @brief A STOP terminal explains this report only when the application bound both identities. */
static uint8_t static_completion_matches_report(const DebugUiModel *model)
{
    const DebugUiStaticLoadReport *report = &model->snapshot.static_load;
    if (model->completion.operation == DEBUG_UI_OPERATION_STOP &&
        view_same_identity(&model->completion.identity, &report->stop_identity)) return 1U;
    return (uint8_t)(!report->stopped && model->completion.operation == report->operation &&
        view_same_identity(&model->completion.identity, &report->identity));
}

/** @brief Bind terminal wording to the retained report identity and distinguish cleanup evidence. */
static const char *static_result_heading(const DebugUiModel *model)
{
    const DebugUiStaticLoadReport *report = &model->snapshot.static_load;
    uint8_t disabled = report->disabled_confirmed; /* Historical causal acknowledgement is separate from today's control lock. */
    uint8_t matching_terminal = (uint8_t)(model->completion_seen && static_completion_matches_report(model));
    if (report->identity.request_id != 0U && !static_report_scope_valid(report)) return "静载 / 无效范围";
    if (!static_report_visible(model) && debug_ui_is_static_operation(model->submitted.operation) &&
        model->submitted.identity.request_id != 0U)
        return model->completion_seen && view_same_identity(&model->completion.identity, &model->submitted.identity) &&
            model->completion.code == DEBUG_UI_FAILED ? "静载未开始 / 已拒绝" : "静载结果 / 等待本次";
    if (report->identity.request_id == 0U) return "静载结果 / 未运行";
    if (model->static_result_retained) {
        if (report->first_exit_latched && report->first_exit_code == DEBUG_UI_FAILED)
            return disabled ? "保留失败 / 已失能" : "保留失败 / 待确认";
        if (report->stopped) return disabled ? "保留已停止 / 已失能" : "保留已停止 / 待确认";
        return disabled ? "保留结果 / 已失能" : "保留结果 / 待确认";
    }
    if (report->first_exit_latched && report->first_exit_code == DEBUG_UI_FAILED)
        return disabled ? "静载失败 / 已失能" : "静载失败 / 待确认";
    if (report->stopped) return disabled ? "静载已停止 / 已失能" : "静载已停止 / 待确认";
    if (matching_terminal && model->completion.code == DEBUG_UI_FAILED)
        return disabled ? "静载失败 / 已失能" : "静载失败 / 待确认";
    if (matching_terminal && model->completion.code == DEBUG_UI_STOPPED)
        return disabled ? "静载已停止 / 已失能" : "静载已停止 / 待确认";
    if (!report->complete) return disabled ? "静载未完成 / 已失能" : "静载未完成 / 待确认";
    return disabled ? "静载结果 / 已失能" : "静载结果 / 待确认";
}

/** @brief Identify the allocation-free monochrome templates owned by the guided workflow. */
static uint8_t single_static_instrument_page(const DebugUiModel *model)
{
    return (uint8_t)(model->page == DEBUG_UI_PAGE_STATIC_HUB ||
        model->page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP ||
        model->page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW ||
        model->page == DEBUG_UI_PAGE_SINGLE_STATIC_READY ||
        model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK ||
        model->page == DEBUG_UI_PAGE_SINGLE_STATIC_PREFLIGHT ||
        (model->single_static_flow && (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT ||
         model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT ||
         (model->page == DEBUG_UI_PAGE_REVIEW && (model->reviewed.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ||
          (model->reviewed.operation == DEBUG_UI_OPERATION_ACQUIRE && model->review_return_page == DEBUG_UI_PAGE_SINGLE_STATIC_READY))) ||
         (model->page == DEBUG_UI_PAGE_RUNNING && single_static_running(model)))));
}

/** @brief Require coherent selected-axis evidence before showing a current measurement. */
static uint8_t single_static_feedback_fresh(const DebugUiModel *model, unsigned index)
{
    const DebugUiMotorView *motor = &model->snapshot.motors[index];
    return (uint8_t)(model->snapshot_seen && model->now_us >= model->snapshot.timestamp_us &&
        model->now_us - model->snapshot.timestamp_us < DEBUG_UI_HEALTH_TIMEOUT_US &&
        motor->motor_id == index + 1U && motor->feedback_seen && motor->feedback_valid &&
        motor->feedback_age_ms < 150U && isfinite(motor->position_rad) &&
        isfinite(motor->torque_nm) && isfinite(motor->mos_temperature_c) && isfinite(motor->rotor_temperature_c));
}

/** @brief Compose setup, configuration, readiness and relock from published facts only. */
static void single_static_flow_rows(DebugUiViewPage *page, const DebugUiModel *model)
{
    unsigned index = model->selected_motor < DEBUG_UI_MOTOR_COUNT ? model->selected_motor : 0U;
    const DebugUiMotorView *motor = &model->snapshot.motors[index];
    uint8_t fresh = single_static_feedback_fresh(model, index);
    uint8_t unlocked = (uint8_t)(AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD && model->snapshot.single_static_load_enabled);
    uint8_t configured = (uint8_t)(model->single_static_configured && motor->single_static_profile.configured);
    uint8_t cancel_handoff = debug_ui_model_single_static_can_cancel_handoff(model);
    char text[DEBUG_UI_VIEW_TEXT_BYTES];
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_PREFLIGHT) {
        (void)snprintf(text, sizeof(text), "ID%u / 失能预检", model->submitted.target_motor_id); row(page, 0U, text);
        row(page, 1U, "仅输出一次 DISABLE");
        row(page, 2U, "等待输出后的失能反馈");
        row(page, 3U, "不会立即开始保持");
        row(page, 4U, "参数与姿态保持冻结");
        row(page, 5U, model->awaiting_result ? "等待有效新反馈" : "预检完成 / 检查新快照");
        row(page, 6U, "左 取消 / 中 STOP");
        return;
    }
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW) {
        (void)snprintf(text, sizeof(text), "ID%u / 配置确认", model->reviewed.target_motor_id); row(page, 0U, text);
        (void)snprintf(text, sizeof(text), "允许保持 %lu 秒", (unsigned long)(model->reviewed.hold_duration_ms / 1000U)); row(page, 1U, text);
        (void)snprintf(text, sizeof(text), "前馈 %+.2f Nm / 输出轴", (double)model->reviewed.torque_ff_nm); row(page, 2U, text);
        row(page, 3U, "确认后单次失能预检");
        if (model->reviewed.target_motor_id == 7U && model->reviewed.kp > 0.0f) {
            (void)snprintf(text, sizeof(text), "Kp %.0f / Kd %.2f / 输出轴",
                (double)model->reviewed.kp, (double)model->reviewed.kd); row(page, 4U, text);
        } else row(page, 4U, "开始保持还需独立确认");
        row(page, 5U, model->single_static_wait_snapshot ? "配置完成 / 等待新快照" :
            (model->awaiting_result ? "正在提交 / 等待应用确认" : "核对ID与时长 / 反馈待预检"));
        if (model->awaiting_result || model->single_static_wait_snapshot) row(page, 6U, "等待确认 / 中键 STOP");
        else if (!model->review_released) row(page, 6U, "请先松开中键");
        else { (void)snprintf(text, sizeof(text), "长按中键 %lu / 800ms", (unsigned long)model->confirm_elapsed_ms); row(page, 6U, text); }
        return;
    }
    (void)snprintf(text, sizeof(text), "ID%u / %s", index + 1U,
        model->page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP ? "准备配置" :
        (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK ? "锁止退出" : "检查并锁姿态")); row(page, 0U, text);
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK) {
        if (model->single_static_relock_motor_id > 0U && model->single_static_relock_motor_id <= DEBUG_UI_MOTOR_COUNT) {
            (void)snprintf(text, sizeof(text), "ID%u / 锁止退出", model->single_static_relock_motor_id);
            row(page, 0U, text);
        }
        if (model->single_static_relock_motor_id > 0U && model->single_static_relock_motor_id != index + 1U) {
            (void)snprintf(text, sizeof(text), "下一步 ID%u / %u 秒", index + 1U, model->single_static_seconds);
            row(page, 1U, text);
        } else row(page, 1U, model->awaiting_result || model->single_static_wait_snapshot ?
            "正在重新锁止" : "锁止失败 / 等待重试");
        row(page, 2U, cancel_handoff ? "本流程RAM配置 / 未配置" : "撤销本流程RAM配置");
        row(page, 3U, cancel_handoff ? "保留已有控制权" : "释放本地控制权");
        row(page, 4U, cancel_handoff ? "左键取消本次准备" : "收到应用确认后才离开");
        row(page, 5U, model->single_static_wait_snapshot ? "等待锁止后的新快照" :
            (model->awaiting_result ? "等待应用锁止确认" : debug_ui_reason_text(model->reason)));
        row(page, 6U, cancel_handoff ? "左 返回 / 中 重试 / 右 STOP" :
            (model->awaiting_result || model->single_static_wait_snapshot ? "处理中 / 中键 STOP" : "中键 重试锁止 / 右 STOP"));
        return;
    }
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP) {
        (void)snprintf(text, sizeof(text), "电机   < ID%u >", index + 1U); row(page, 1U, text);
        (void)snprintf(text, sizeof(text), "保持   < %u 秒 >", model->single_static_seconds); row(page, 2U, text);
        row(page, 3U, !unlocked ? "固件锁止 / 不可解锁" : (configured ? "RAM配置 / 已确认" : "RAM配置 / 未配置"));
        if (motor->mode_verified && motor->actual_mode != DEBUG_UI_MODE_MIT)
            (void)snprintf(text, sizeof(text), "先返回电机页切换 MIT 模式");
        else if (!fresh) (void)snprintf(text, sizeof(text), "反馈未知 / 确认后失能预检");
        else (void)snprintf(text, sizeof(text), "%s / 新反馈 / %s", motor->mode_verified ? mode_text(motor->actual_mode) : "模式未知",
            motor->enabled || motor->driver_state != 0U ? "失能待确认" : "已失能");
        row(page, 4U, text);
        row(page, 5U, !unlocked ? "固件锁止" : "配置并解锁");
        row(page, 6U, motor->mode_verified && motor->actual_mode != DEBUG_UI_MODE_MIT ? "先返回电机页切换 MIT 模式" :
            (model->reason == DEBUG_UI_REASON_NONE ? "仅配置 / 不产生保持输出" : debug_ui_reason_text(model->reason)));
        return;
    }
    (void)snprintf(text, sizeof(text), "RAM %s / %u 秒", configured ? "已确认" : "未配置", model->single_static_seconds); row(page, 1U, text);
    {
        char authority_text[32];
        if (model->snapshot.authority==DEBUG_UI_AUTHORITY_LOCAL_ARMED && model->snapshot.target_motor_id!=index+1U)
            (void)snprintf(authority_text,sizeof(authority_text),"本地ID%u",model->snapshot.target_motor_id);
        else (void)snprintf(authority_text,sizeof(authority_text),"%s",
            model->snapshot.authority==DEBUG_UI_AUTHORITY_LOCAL_ARMED?"本地":
            (model->snapshot.authority==DEBUG_UI_AUTHORITY_REMOTE?"远端":"控制锁定"));
        (void)snprintf(text,sizeof(text),"%s / %s / %s / %s",authority_text,
            motor->mode_verified?mode_text(motor->actual_mode):"模式未知",fresh?"新反馈":"反馈未知",
            fresh && !motor->enabled && motor->driver_state==0U?"已失能":"待确认");
        row(page,2U,text);
    }
    if (model->static_locked && model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        model->draft.target_motor_id == index + 1U && isfinite(model->draft.static_positions_rad[index]))
        (void)snprintf(text, sizeof(text), "冻结输出轴 %.2f°", (double)debug_ui_radians_to_degrees(model->draft.static_positions_rad[index]));
    else if (fresh) (void)snprintf(text, sizeof(text), "当前输出轴 %.2f°", (double)debug_ui_radians_to_degrees(motor->position_rad));
    else if (debug_ui_raw_position_current(model, motor))
        (void)snprintf(text, sizeof(text), "位置 %.2f°", (double)debug_ui_register_output_degrees(motor));
    else (void)snprintf(text, sizeof(text), "当前输出轴 --");
    row(page, 3U, text);
    row(page, 4U, model->static_locked ? "姿态已捕获 / 开始前再确认" :
        (model->snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? "本地控制 / 等待捕获姿态" : "远端控制 / 先申请本地"));
    row(page, 5U, model->single_static_wait_snapshot ? "等待控制权新快照" :
        (model->snapshot.authority != DEBUG_UI_AUTHORITY_LOCAL_ARMED ? "检查并申请本地控制" :
        (model->static_locked ? "查看开始确认" : "捕获当前姿态")));
    row(page, 6U, model->reason == DEBUG_UI_REASON_NONE ? "先承托 / 到期将失能" : debug_ui_reason_text(model->reason));
}

#include "debug_ui_astra_layout.inc"

/** @brief Map a copied model to persistent LVGL labels without runtime object creation. */
void debug_ui_view_update(DebugUiView *view, const DebugUiModel *model,
                          const DebugUiViewDiagnostics *diagnostics)
{
    DebugUiViewPage *page;
    const DebugUiSnapshot *snapshot;
    const DebugUiMotorView *motor;
    const DebugUiRequest *action;
    char text[DEBUG_UI_VIEW_TEXT_BYTES], value[80], title[80];
    const char *footer = "左 返回  上下 选择  中 进入";
    const char *heading = "总览";
    unsigned index;
    int focus_row = -1;
    uint8_t page_changed;
    uint8_t load_result;
    uint8_t pos_cleanup_locked;
    const DebugUiLoadTrialRecord *load_record;
    if (view == NULL || model == NULL || diagnostics == NULL || !view->initialized || !debug_ui_view_healthy()) return;
    if ((unsigned)model->page >= DEBUG_UI_PAGE_COUNT) return;
    page = &view->page;
    snapshot = &model->snapshot;
    pos_cleanup_locked = (uint8_t)(model->pos_load_flow && debug_ui_model_result_requires_check(model));
    motor = &snapshot->motors[model->selected_motor < DEBUG_UI_MOTOR_COUNT ? model->selected_motor : 0U];
    page_changed = (uint8_t)(!page->layout_initialized || view->visible != model->page);
    load_record = load_view_bound_record(model);
    load_result = (uint8_t)(model->single_static_flow && model->submitted.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT ||
         (load_record != NULL && (model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT))));
    view->visible = model->page;
    for (index = 0U; index < DEBUG_UI_VIEW_ROWS; ++index) page->row_text[index][0] = '\0';
    /* Compose into a bounded local row matrix, then update only changed strings. */
    switch (model->page) {
    case DEBUG_UI_PAGE_LOAD_METADATA:
    case DEBUG_UI_PAGE_LOAD_NUMBER:
    case DEBUG_UI_PAGE_LOAD_HISTORY:
    case DEBUG_UI_PAGE_LOAD_DETAIL:
        heading = model->page == DEBUG_UI_PAGE_LOAD_HISTORY ? "最近载荷记录" :
            (model->page == DEBUG_UI_PAGE_LOAD_DETAIL ? "载荷记录详情" : "载荷/记录");
        load_view_rows(page, model);
        footer = model->page == DEBUG_UI_PAGE_LOAD_NUMBER ? "上下 ±1  左右 ±10  中 返回" :
            (model->page == DEBUG_UI_PAGE_LOAD_HISTORY ? "上下 选择  中 详情  左 返回" :
            (model->page == DEBUG_UI_PAGE_LOAD_DETAIL ? "上下/右 翻面  中/左 列表" : "上下 选择  左右 ±10  中 进入"));
        break;
    case DEBUG_UI_PAGE_OVERVIEW:
        heading = "系统总览";
        (void)snprintf(text, sizeof(text), "%s / 已发现 %u/7", snapshot->bench_profile ? "台架" : "生产只读", discovered_count(snapshot)); row(page, 0U, text);
        (void)snprintf(text, sizeof(text), "USB %s  反馈有效 %u/7", snapshot->usb_connected ? "已连接" : "未连接", online_count(snapshot)); row(page, 1U, text);
        row(page, 2U, snapshot->arm_fault ? "系统故障 / 查看诊断" :
            (snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? "控制权: 本地已授权" :
            (snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT ? "控制权: 故障锁定" : "控制权: 远端 / 只读")));
        row(page, 3U, "电机列表  7台 / 每页5行");
        row(page, 4U, "诊断信息  按键 / 显示");
        row(page, 5U, snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? "释放本地控制" : "申请本地控制");
        row(page, 6U, debug_ui_reason_text(debug_ui_model_gate(model,
            snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? DEBUG_UI_OPERATION_RELEASE : DEBUG_UI_OPERATION_ACQUIRE)));
        focus_row = 3 + model->focus % 4U;
        footer = "左右 选择  中/下 进入";
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_GAINS:
        heading = "单轴静载 / 参数";
        row(page, 0U, "ID7 / 输出轴等效参数");
        (void)snprintf(text, sizeof(text), "%sKp  %u", model->focus == 0U ? "> " : "", model->single_static_kp); row(page, 1U, text);
        (void)snprintf(text, sizeof(text), "%sKd  %u.%02u", model->focus == 1U ? "> " : "", model->single_static_kd_centi / 100U,
            model->single_static_kd_centi % 100U); row(page, 2U, text);
        row(page, 3U, model->focus == 2U ? "> 恢复默认 120 / 0.30" : "恢复默认 120 / 0.30");
        row(page, 4U, model->focus == 3U ? "> 返回" : "返回");
        row(page, 5U, "Kp 80-200 / Kd 0.20-0.40");
        row(page, 6U, "停止后设置 / 下次生效");
        if (model->gain_editing) {
            char digits[8]; unsigned character;
            if (model->focus == 0U) (void)snprintf(digits, sizeof(digits), "%03u", model->gain_edit_value);
            else (void)snprintf(digits, sizeof(digits), "%u.%02u", model->gain_edit_value / 100U, model->gain_edit_value % 100U);
            character = model->gain_digit + (model->focus == 1U && model->gain_digit > 0U ? 1U : 0U);
            (void)snprintf(text, sizeof(text), "%s %.*s[%c]%s", model->focus == 0U ? "Kp" : "Kd",
                (int)character, digits, digits[character], digits + character + 1U);
            row(page, model->focus + 1U, text);
        }
        focus_row = model->focus + 1U;
        footer = model->gain_editing ? "左右选位 上下调值 中保存 长左取消" : "上下选择 中编辑 左返回";
        break;
    case DEBUG_UI_PAGE_STATIC_HUB:
        heading = "静载测试";
        footer = "上下 选择  中 进入  左 首页";
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_SETUP:
    case DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW:
    case DEBUG_UI_PAGE_SINGLE_STATIC_READY:
    case DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK:
    case DEBUG_UI_PAGE_SINGLE_STATIC_PREFLIGHT:
        heading = model->page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW ? "确认配置与解锁" :
            (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK ? "安全锁止" : "单轴静载");
        single_static_flow_rows(page, model);
        footer = model->page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP ? "中 配置  右 载荷/记录  左 锁止" :
            (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW ? "左 取消  松开后长按确认" :
            (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK ? "先确认失能 / 锁止后退出" : "左 锁止退出  中 下一步"));
        if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_PREFLIGHT) { heading = "失能预检"; footer = "单次 DISABLE / 左取消 中STOP"; }
        if (debug_ui_model_single_static_can_cancel_handoff(model)) footer = "左 取消返回 / 中 重试";
        break;
    case DEBUG_UI_PAGE_PREPARE:
        heading = "本地控制检查";
        (void)snprintf(text, sizeof(text), "ID%u %s / %s", model->selected_motor + 1U,
            mode_text(motor->actual_mode), motor->mode_verified ? "模式已校验" : "模式未校验"); row(page, 0U, text);
        row(page, 1U, snapshot->motion_enabled ? "本地控制: 已开放" : "本地控制: 固件只读");
        (void)snprintf(text, sizeof(text), "发现 %s / 范围 %s", motor->identity_verified ? "有效" : "未知",
            motor->ranges_verified ? "有效" : "未知"); row(page, 2U, text);
        if (motor->feedback_seen) (void)snprintf(text, sizeof(text), "反馈 %s %lums",
            motor->feedback_valid ? "有效" : "过期", (unsigned long)motor->feedback_age_ms);
        else (void)snprintf(text, sizeof(text), "反馈: 未知");
        row(page, 3U, text);
        row(page, 4U, motor->profile.pos_valid ? "POS标定: 已配置" : "POS标定: 尚未配置");
        row(page, 5U, pos_load_prepare_return(model) && !snapshot->unconfirmed_disable_mask && snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
            snapshot->target_motor_id == model->selected_motor + 1U ? "本轴已授权 / 保留参数草稿" :
            debug_ui_reason_text(debug_ui_model_gate(model,
            snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? DEBUG_UI_OPERATION_RELEASE : DEBUG_UI_OPERATION_ACQUIRE)));
        row(page, 6U, !AETHOR_DEBUG_UI_ALLOW_MOTION || !snapshot->motion_enabled ? "只读 / 本屏不可控制" :
            (snapshot->unconfirmed_disable_mask != 0U ? "中键 停止" :
            (pos_load_prepare_return(model) && snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
             snapshot->target_motor_id == model->selected_motor + 1U ? "中键 返回参数" : "中键 重新检查"))); focus_row = 6;
        if (pos_load_prepare_return(model) && snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
            snapshot->target_motor_id != model->selected_motor + 1U) {
            row(page, 5U, "本地控制非当前轴"); row(page, 6U, "中键 检查当前轴");
        }
        if (pos_load_prepare_return(model) && (model->stop_requested || snapshot->stop_pending)) {
            row(page, 5U, "STOP处理中 / 等待失能"); row(page, 6U, "中键 STOP / 等待失能");
        }
        if (pos_cleanup_locked && AETHOR_DEBUG_UI_ALLOW_MOTION && snapshot->motion_enabled &&
            !model->stop_requested && !snapshot->stop_pending) {
            row(page, 5U, "失能未确认 / 保持锁定"); row(page, 6U, "中键 STOP");
        }
        footer = "左 返回  右 电机详情";
        break;
    case DEBUG_UI_PAGE_ACTIONS:
        heading = "电机操作";
        footer = "左 列表  上下 选择  中 进入";
        break;
    case DEBUG_UI_PAGE_MOTORS:
        heading = "电机列表";
        for (index = 0U; index < DEBUG_UI_VISIBLE_MOTORS; ++index) {
            unsigned motor_index = model->list_first + index;
            const DebugUiMotorView *item = &snapshot->motors[motor_index];
            debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(item->position_rad),
                                   item->feedback_seen, item->feedback_valid, "°");
            if (!item->feedback_valid && debug_ui_raw_position_current(model, item))
                (void)snprintf(value, sizeof(value), "%.2f°", (double)debug_ui_register_output_degrees(item));
            if (item->fault_flags != 0U)
                (void)snprintf(text, sizeof(text), "%s%u %s %lu %s", motor_index == model->selected_motor ? ">" : " ",
                               motor_index + 1U, item->feedback_valid ? "故障" : "故障过期",
                               (unsigned long)item->fault_flags, mode_text(item->actual_mode));
            else (void)snprintf(text, sizeof(text), "%s%u %s %s %s", motor_index == model->selected_motor ? ">" : " ", motor_index + 1U,
                item->feedback_seen ? (item->feedback_valid ? (item->enabled ? "使能" : "失能") : "过期") :
                    (item->identity_verified ? "已发现" : "未知"), mode_text(item->actual_mode), value);
            row(page, index, text);
            if (motor_index == model->selected_motor) focus_row = (int)index;
        }
        row(page, 5U, "° 输出轴 / R 原始读值");
        (void)snprintf(text, sizeof(text), "行 %u-%u / 7  本次上电零点", model->list_first + 1U, model->list_first + 5U); row(page, 6U, text);
        break;
    case DEBUG_UI_PAGE_DETAIL: {
        static const char *const choices[8] = { "POS移动", "MIT保持", "MIT移动", "切换POS", "切换MIT", "清除故障", "请求失能", "位置读值" };
        heading = "电机详情";
        (void)snprintf(text, sizeof(text), "ID%u %s / %s", model->selected_motor + 1U, motor->mode_verified?mode_text(motor->actual_mode):"模式未校验",
            motor->feedback_seen ? (motor->feedback_valid ? (motor->enabled ? "已使能" : "已失能") : "状态过期") : "状态未知"); row(page, 0U, text);
        debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(motor->position_rad), motor->feedback_seen, motor->feedback_valid, "°");
        if (!motor->feedback_valid && debug_ui_raw_position_current(model, motor))
            (void)snprintf(text, sizeof(text), "位置 %.2f°", (double)debug_ui_register_output_degrees(motor));
        else (void)snprintf(text, sizeof(text), "位置 %s", value);
        row(page, 1U, text);
        debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(motor->velocity_rad_s), motor->feedback_seen, motor->feedback_valid, "°/s");
        if (!motor->feedback_valid && debug_ui_estimated_speed_current(model, motor))
            (void)snprintf(text, sizeof(text), "估算速度 %.2f°/s", (double)debug_ui_radians_to_degrees(
                motor->register_estimated_native_rad_s / motor->single_static_native_per_output));
        else (void)snprintf(text, sizeof(text), "速度 %s", value);
        row(page, 2U, text);
        debug_ui_format_value(value, sizeof(value), motor->torque_nm, motor->feedback_seen, motor->feedback_valid, "Nm");
        (void)snprintf(text, sizeof(text), "转矩 %s", value); row(page, 3U, text);
        if (motor->feedback_seen && motor->feedback_valid)
            (void)snprintf(text, sizeof(text), "温度 MOS %.0f / 转子 %.0f°C", (double)motor->mos_temperature_c, (double)motor->rotor_temperature_c);
        else (void)snprintf(text, sizeof(text), "温度 -- / %s", motor->feedback_seen ? "反馈过期" : "未知");
        row(page, 4U, text);
        if (motor->feedback_seen && motor->feedback_valid) (void)snprintf(text, sizeof(text), "反馈 %lums  故障 %lu", (unsigned long)motor->feedback_age_ms, (unsigned long)motor->fault_flags);
        else if (motor->feedback_seen) (void)snprintf(text, sizeof(text), "过期 %lums  故障 --", (unsigned long)motor->feedback_age_ms);
        else (void)snprintf(text, sizeof(text), "反馈 --  故障 --");
        row(page, 5U, text);
        (void)snprintf(text, sizeof(text), ">%s | %s", choices[model->focus % 8U],
                       model->focus == 7U ? "中/右 进入" :
                       (model->reason == DEBUG_UI_REASON_NONE ? "中键编辑" : debug_ui_reason_text(model->reason)));
        row(page, 6U, text); focus_row = 6;
        footer = "输出轴 / 左 返回操作";
        break;
    }
    case DEBUG_UI_PAGE_REGISTERS:
        heading = "位置读值";
        (void)snprintf(text, sizeof(text), "ID%u 原始值 / 未校验", model->selected_motor + 1U); row(page, 0U, text);
        for (index = 0U; index < 2U; ++index) {
            uint8_t seen = (uint8_t)((motor->register_seen_mask & (1U << index)) != 0U);
            if (seen && isfinite(motor->register_position[index]))
                (void)snprintf(value, sizeof(value), "%s%.6g", motor->register_age_ms[index] < 500U ? "" : "过期 ",
                    (double)motor->register_position[index]);
            else (void)snprintf(value, sizeof(value), "--");
            (void)snprintf(text, sizeof(text), "0x%02X %s", 0x50U + index, value); row(page, 1U + 2U * index, text);
            if (seen) (void)snprintf(text, sizeof(text), "样本 %lums", (unsigned long)motor->register_age_ms[index]);
            else (void)snprintf(text, sizeof(text), "样本 未知");
            row(page, 2U + 2U * index, text);
        }
        (void)snprintf(text, sizeof(text), "读取超时 %lu", (unsigned long)motor->register_timeout_count); row(page, 5U, text);
        row(page, 6U, motor->feedback_valid ? "状态反馈 有效" : "状态反馈 未知/过期");
        footer = "只读 / 左 返回";
        break;
    case DEBUG_UI_PAGE_DIAGNOSTICS:
        heading = "诊断信息";
        if (model->diagnostic_page == 0U) {
            if (diagnostics->adc_seen) (void)snprintf(text, sizeof(text), "ADC %u  样本 %lums", diagnostics->adc_raw, (unsigned long)diagnostics->adc_age_ms);
            else (void)snprintf(text, sizeof(text), "ADC --  样本未知");
            row(page, 0U, text);
            (void)snprintf(text, sizeof(text), "按键 %u  %s", (unsigned)diagnostics->key, diagnostics->input_valid ? "有效" : "输入未就绪"); row(page, 1U, text);
            (void)snprintf(text, sizeof(text), "输入故障 %lu", (unsigned long)diagnostics->input_fault); row(page, 2U, text);
            (void)snprintf(text, sizeof(text), "DMA错 %lu  块 %lums", (unsigned long)diagnostics->dma_errors, (unsigned long)diagnostics->flush_ms); row(page, 3U, text);
            (void)snprintf(text, sizeof(text), "REFRESH %luB  %lums", (unsigned long)diagnostics->refresh_bytes, (unsigned long)diagnostics->refresh_ms); row(page, 4U, text);
            (void)snprintf(text, sizeof(text), "有效 %lu.%lu FPS  栈 %lu", (unsigned long)(diagnostics->refresh_fps_tenths / 10U), (unsigned long)(diagnostics->refresh_fps_tenths % 10U), (unsigned long)diagnostics->stack_free_words); row(page, 5U, text);
            (void)snprintf(text, sizeof(text), "控制 %lu/%lu us", (unsigned long)diagnostics->control_execution_max_us, (unsigned long)diagnostics->control_period_max_us); row(page, 6U, text);
        } else if (model->diagnostic_page == 1U) {
            (void)snprintf(text, sizeof(text), "控制超期 %lu", (unsigned long)snapshot->diagnostics.control_deadline_miss_count); row(page, 0U, text);
            (void)snprintf(text, sizeof(text), "CAN丢弃 %lu", (unsigned long)snapshot->diagnostics.can_rx_drop_count); row(page, 1U, text);
            (void)snprintf(text, sizeof(text), "USB丢弃 %lu", (unsigned long)snapshot->diagnostics.usb_rx_drop_count); row(page, 2U, text);
            (void)snprintf(text, sizeof(text), "拒绝请求 %lu", (unsigned long)snapshot->diagnostics.rejected_request_count); row(page, 3U, text);
            (void)snprintf(text, sizeof(text), "健康停止 %lu", (unsigned long)snapshot->diagnostics.health_stop_count); row(page, 4U, text);
            (void)snprintf(text, sizeof(text), "结果积压 %lu", (unsigned long)snapshot->diagnostics.result_backpressure_count); row(page, 5U, text);
            row(page, 6U, "软件数据 / 未做实物验证");
        } else {
            (void)snprintf(text, sizeof(text), "固件 %s", diagnostics->firmware_version != NULL ? diagnostics->firmware_version : "--"); row(page, 0U, text);
            (void)snprintf(text, sizeof(text), "CAN TX错误 %lu", (unsigned long)snapshot->diagnostics.can_tx_error_count); row(page, 1U, text);
            (void)snprintf(text, sizeof(text), "遥测丢弃 %lu", (unsigned long)snapshot->diagnostics.usb_telemetry_drop_count); row(page, 2U, text);
            (void)snprintf(text, sizeof(text), "最小栈余量 %lu word", (unsigned long)snapshot->diagnostics.minimum_stack_words); row(page, 3U, text);
            (void)snprintf(text, sizeof(text), "最小堆余量 %lu B", (unsigned long)snapshot->diagnostics.minimum_heap_bytes); row(page, 4U, text);
            (void)snprintf(text, sizeof(text), "显示SPI累计错误 %lu", (unsigned long)diagnostics->spi_errors); row(page, 5U, text);
            (void)snprintf(text, sizeof(text), "显示初始化错误 %lu", (unsigned long)diagnostics->init_errors); row(page, 6U, text);
        }
        footer = "只读 / 上下/右 切换 / 左 返回";
        break;
    case DEBUG_UI_PAGE_STATIC_LOAD:
        heading = !AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || !snapshot->static_load_enabled ?
            "七轴静载 / 只读" : "七轴静载 / 中 操作";
        static_load_rows(page, model, 0U);
        footer = "输出轴e° T Nm MOS/转子°C";
        break;
    case DEBUG_UI_PAGE_STATIC_ACTIONS:
        heading = "静载操作 / 先承托";
        footer = "结束失能前先卸载 / 左 返回";
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_LOAD:
        heading = "单轴静载";
        single_static_rows(page, model, 0U);
        footer = "左 返回 / 中 操作";
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS:
        heading = "单轴操作";
        footer = "到期失能 / 请先承托卸载";
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_IDS:
        heading = "选择静载电机";
        footer = "更换ID需重新锁定姿态";
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_RESULT:
        heading = static_result_heading(model);
        if (model->static_result_retained && static_report_visible(model) &&
            snapshot->static_load.operation == DEBUG_UI_OPERATION_STATIC_HOLD) {
            heading = "七轴保留结果";
            static_load_rows(page, model, 1U);
        } else single_static_rows(page, model, 1U);
        footer = model->single_static_flow ? "中 再次准备  左 锁止退出" : "承托卸载 / 中 返回";
        if (model->completion_seen && static_report_visible(model) && static_completion_matches_report(model) &&
            model->completion.reason != DEBUG_UI_REASON_NONE) footer = debug_ui_reason_text(model->completion.reason);
        break;
    case DEBUG_UI_PAGE_STATIC_RESULT:
        heading = static_result_heading(model);
        if (model->static_result_retained && static_report_visible(model) &&
            snapshot->static_load.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
            heading = "单轴保留结果";
            single_static_rows(page, model, 1U);
        } else static_load_rows(page, model, 1U);
        footer = model->static_result_detail == 0U ? "输出轴峰值°/Nm 上下 翻页" :
            (model->static_result_detail == 1U ? "MOS/转子峰值°C 上下 翻页" : "有效采样数 / 中 返回");
        if (model->completion_seen && model->completion.reason != DEBUG_UI_REASON_NONE &&
            (static_completion_matches_report(model) ||
             (!static_report_visible(model) && !model->static_result_retained &&
              view_same_identity(&model->completion.identity, &model->submitted.identity)))) {
            (void)snprintf(value, sizeof(value), "%s / 上下 翻页", debug_ui_reason_text(model->completion.reason));
            footer = value;
        }
        if (snapshot->unconfirmed_disable_mask != 0U) footer = "控制仍锁定 / 需重新检查";
        break;
    case DEBUG_UI_PAGE_NUMBER:
    case DEBUG_UI_PAGE_EDIT:
    case DEBUG_UI_PAGE_REVIEW:
        action = model->page == DEBUG_UI_PAGE_REVIEW ? &model->reviewed : &model->draft;
        if (action->target_motor_id > 0U && action->target_motor_id <= DEBUG_UI_MOTOR_COUNT)
            motor = &snapshot->motors[action->target_motor_id - 1U];
        heading = model->page == DEBUG_UI_PAGE_EDIT ? "动作草稿" : "二次确认";
        (void)snprintf(text, sizeof(text), "ID%u %s", action->target_motor_id, operation_text(action->operation)); row(page, 0U, text);
        if (debug_ui_is_static_operation(action->operation)) {
            if (action->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
                (void)snprintf(text, sizeof(text), "ID%u 单轴静载 / 已锁定", action->target_motor_id); row(page, 0U, text);
            } else row(page, 0U, "七轴静载 / 已锁定姿态");
            (void)snprintf(text, sizeof(text), "限时 %lums / 前馈 0Nm", (unsigned long)action->hold_duration_ms); row(page, 1U, text);
            if (model->single_static_flow && action->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
                (void)snprintf(text, sizeof(text), "保持 %lu秒 FF%+.2fNm", (unsigned long)(action->hold_duration_ms / 1000U), (double)action->torque_ff_nm);
                row(page, 1U, text);
            }
            row(page, 2U, action->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ? "所选轴 MIT 失能静止" : "七轴均须 MIT 失能静止");
            if (model->single_static_flow && action->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
                action->target_motor_id > 0U && action->target_motor_id <= DEBUG_UI_MOTOR_COUNT) {
                float target_degrees = debug_ui_radians_to_degrees(action->static_positions_rad[action->target_motor_id - 1U]);
                if (isfinite(target_degrees)) (void)snprintf(text, sizeof(text), "冻结输出轴 %.2f°", (double)target_degrees);
                else (void)snprintf(text, sizeof(text), "冻结输出轴 --");
                row(page, 2U, text);
            }
            row(page, 3U, "先承托 / 失能前先卸载");
            row(page, 4U, "输出轴 / 非法兰标定");
        } else if (action->operation == DEBUG_UI_OPERATION_SET_MODE) {
            (void)snprintf(text, sizeof(text), "模式 %s -> %s", mode_text(motor->actual_mode), mode_text(action->requested_mode)); row(page, 1U, text);
            row(page, 2U, "模式读回后才确认成功");
            row(page, 3U, "切模前: 失能且速度近零");
        } else if (action->operation == DEBUG_UI_OPERATION_ACQUIRE || action->operation == DEBUG_UI_OPERATION_RELEASE ||
                   action->operation == DEBUG_UI_OPERATION_CLEAR_FAULT || action->operation == DEBUG_UI_OPERATION_DISABLE) {
            row(page, 1U, action->operation == DEBUG_UI_OPERATION_ACQUIRE ? "申请本地操作权限" :
                (action->operation == DEBUG_UI_OPERATION_RELEASE ? "交还远端操作权限" :
                (action->operation == DEBUG_UI_OPERATION_CLEAR_FAULT ? "清除当前电机故障" : "停止并请求电机失能")));
            row(page, 2U, "结果以应用反馈为准");
            row(page, 3U, debug_ui_reason_text(model->reason));
        } else {
            (void)snprintf(text, sizeof(text), "相对 %+.1f°  速度 %.1f°/s", (double)debug_ui_radians_to_degrees(action->delta_rad),
                           (double)debug_ui_radians_to_degrees(action->speed_rad_s)); row(page, 1U, text);
            if (action->operation == DEBUG_UI_OPERATION_MIT_HOLD || action->operation == DEBUG_UI_OPERATION_MIT_MOVE) {
            (void)snprintf(text, sizeof(text), "Kp %.2f  Kd %.2f", (double)action->kp, (double)action->kd); row(page, 2U, text);
            (void)snprintf(text, sizeof(text), "保持 %lums  前馈 0Nm", (unsigned long)action->hold_duration_ms); row(page, 3U, text);
            } else if (!motor->feedback_seen || !motor->feedback_valid || motor->feedback_age_ms >= 150U) {
            row(page, 2U, "动作前 --");
            row(page, 3U, "目标未知 / 等待反馈");
            } else {
            (void)snprintf(text, sizeof(text), "动作前 %.1f°", (double)debug_ui_radians_to_degrees(motor->position_rad)); row(page, 2U, text);
            (void)snprintf(text, sizeof(text), "预计目标 %.1f°", (double)debug_ui_radians_to_degrees(motor->position_rad + action->delta_rad)); row(page, 3U, text);
            }
        }
        if (debug_ui_is_static_operation(action->operation)) row(page, 4U, "输出轴 / 非法兰标定");
        else if (action->operation == DEBUG_UI_OPERATION_MIT_HOLD || action->operation == DEBUG_UI_OPERATION_MIT_MOVE) {
            (void)snprintf(text, sizeof(text), "MIT范围 %.1f~%.1f°",
                (double)debug_ui_radians_to_degrees(motor->profile.mit_position_min_rad),
                (double)debug_ui_radians_to_degrees(motor->profile.mit_position_max_rad)); row(page, 4U, text);
        } else row(page, 4U, action->operation == DEBUG_UI_OPERATION_SET_MODE ? "切换后仍保持失能" : "输出轴 / 本次上电零点");
        row(page, 5U, action->operation == DEBUG_UI_OPERATION_POS_MOVE || action->operation == DEBUG_UI_OPERATION_MIT_MOVE ||
            action->operation == DEBUG_UI_OPERATION_MIT_HOLD || debug_ui_is_static_operation(action->operation) || action->operation == DEBUG_UI_OPERATION_DISABLE ?
            "结束: 失能并检查反馈" : "结果: 等待应用确认");
        if (model->page == DEBUG_UI_PAGE_REVIEW) {
            if (!model->review_released) row(page, 6U, "请先松开中键");
            else { (void)snprintf(text, sizeof(text), "再按住中键 %lu / 800ms", (unsigned long)model->confirm_elapsed_ms); row(page, 6U, text); }
            footer = "左 取消  松开后长按确认";
        } else if (action->operation != DEBUG_UI_OPERATION_POS_MOVE && action->operation != DEBUG_UI_OPERATION_MIT_MOVE &&
                   action->operation != DEBUG_UI_OPERATION_MIT_HOLD) {
            row(page, 6U, action->operation == DEBUG_UI_OPERATION_SET_MODE ? "中键查看切模确认" : "中键查看操作确认");
            footer = "左 取消  中 查看确认";
        } else {
            static const char *const fields[] = { "相对角度", "速度", "Kp", "Kd", "保持时间" };
            (void)snprintf(text, sizeof(text), "编辑: %s  %s", fields[model->edit_field % 5U], model->reason == DEBUG_UI_REASON_NONE ? "" : debug_ui_reason_text(model->reason)); row(page, 6U, text);
            footer = model->page == DEBUG_UI_PAGE_NUMBER ? "上下 调值  中 保存  左 取消" : "上下 选择  中 编辑  左 返回";
        }
        focus_row = 6;
        break;
    case DEBUG_UI_PAGE_RUNNING: {
        if (static_load_running(model)) {
            heading = single_static_running(model) ? "单轴静载 / 运行" : "七轴静载 / 输出轴";
            if (single_static_running(model)) single_static_rows(page, model, 0U);
            else static_load_rows(page, model, 0U);
            if (model->stop_requested || snapshot->stop_pending) footer = "STOP已请求 / 失能待确认";
            else {
                (void)snprintf(value, sizeof(value), "%lu/%lums 中STOP", (unsigned long)snapshot->static_load.elapsed_ms,
                    (unsigned long)(snapshot->static_load.duration_ms ? snapshot->static_load.duration_ms : model->submitted.hold_duration_ms));
                footer = value;
            }
            if (model->single_static_flow && single_static_running(model))
                footer = model->stop_requested || snapshot->stop_pending ? "停止后等待新失能反馈" : "输出轴 / 到期自动请求失能";
            break;
        }
        uint8_t active_target = debug_ui_model_active_target(model);
        unsigned target_count = active_target_count(model);
        const char *origin = snapshot->active_identity.origin == DEBUG_UI_ORIGIN_USB &&
            (snapshot->active || snapshot->pending) ? "远端动作" : "本地请求";
        heading = "执行状态";
        if (active_target != 0U && target_count > 1U)
            (void)snprintf(text, sizeof(text), "ID%u等%u台 %s", active_target, target_count, origin);
        else if (active_target != 0U)
            (void)snprintf(text, sizeof(text), "ID%u %s", active_target, origin);
        else (void)snprintf(text, sizeof(text), "目标未知 / 等待快照");
        row(page, 0U, text);
        row(page, 1U, model->stop_latch_state == DEBUG_UI_STOP_LATCH_WAITING ? "停止已锁存 / 等待反馈" :
            (model->stop_requested ? "STOP已请求 / 等待清理" : (model->admission_seen ? "已受理 / 等待终态" : "请求中 / 尚未确认")));
        row(page, 2U, stage_text(snapshot->active_stage));
        {
            const DebugUiMotorView *active_motor = &snapshot->motors[active_target != 0U ? active_target - 1U : 0U];
            debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(active_motor->position_rad),
                (uint8_t)(active_target != 0U && active_motor->feedback_seen), active_motor->feedback_valid, "°");
            if (active_target != 0U) (void)snprintf(text, sizeof(text), "ID%u输出轴位置 %s", active_target, value);
            else (void)snprintf(text, sizeof(text), "输出轴位置 %s", value);
            row(page, 3U, text);
        }
        row(page, 4U, AETHOR_DEBUG_UI_ALLOW_MOTION && snapshot->motion_enabled ? "中键立即请求 STOP" : "只读模式 / 本屏不可停止");
        row(page, 5U, "运行中不可编辑或换电机");
        row(page, 6U, debug_ui_reason_text(model->reason));
        focus_row = 4;
        footer = AETHOR_DEBUG_UI_ALLOW_MOTION && snapshot->motion_enabled ?
            "中键 STOP / 不依赖当前焦点" : "只读监视 / 请使用远端控制";
        break;
    }
    case DEBUG_UI_PAGE_RESULT:
    case DEBUG_UI_PAGE_FAULT:
        heading = model->page == DEBUG_UI_PAGE_FAULT ? "故障 / 清理结果" : "动作结果";
        row(page, 0U, model->stop_latch_state == DEBUG_UI_STOP_LATCH_CONFIRMED || model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED ?
            "停止锁存 / 清理已结束" : (!model->completion_seen ? "请求未完成" : (model->completion.code == DEBUG_UI_COMPLETED ? "动作完成" :
                         (model->completion.code == DEBUG_UI_STOPPED ? "动作已停止" : (model->completion.code == DEBUG_UI_CANCELLED ? "动作已取消" : "动作失败")))));
        row(page, 1U, debug_ui_reason_text(model->reason));
        row(page, 2U, model->stop_latch_state != DEBUG_UI_STOP_LATCH_NONE ? "根据最新电机反馈" : stage_text(model->completion.stage));
        row(page, 3U, model->stop_latch_state == DEBUG_UI_STOP_LATCH_CONFIRMED ||
            (model->stop_latch_state == DEBUG_UI_STOP_LATCH_NONE && model->completion_seen && model->completion.disabled_confirmed) ?
            "反馈已确认失能" : "失能未确认 / 保持锁定");
        if (model->stop_latch_state != DEBUG_UI_STOP_LATCH_NONE) row(page, 4U, "结果以电机反馈为准");
        else { (void)snprintf(text, sizeof(text), "错误 %u  详情 %u", model->completion.error, model->completion.detail); row(page, 4U, text); }
        row(page, 5U, "不会自动重发动作");
        row(page, 6U, "再次执行需要重新确认");
        footer = "中/左 已读返回  右 诊断";
        break;
    default: break;
    }
    if (pos_load_view_page(model)) {
        pos_load_view_rows(page, model);
        heading = model->page == DEBUG_UI_PAGE_POS_LOAD_SETUP ?
            (debug_ui_pos_load_flange(pos_load_view_motor(model)) ? "POS带载 / 法兰相对角" : "POS带载 / 输出轴") :
            (model->page == DEBUG_UI_PAGE_POS_LOAD_NUMBER ? "POS参数编辑" :
            (model->page == DEBUG_UI_PAGE_REVIEW ? "POS带载 / 确认开始" :
            (model->page == DEBUG_UI_PAGE_RUNNING ? "POS带载 / 自动过程" : "POS带载 / 试验结果")));
        footer = model->page == DEBUG_UI_PAGE_POS_LOAD_SETUP ? "上下选择 左右调值 中微调/进入" :
            (model->page == DEBUG_UI_PAGE_POS_LOAD_NUMBER ? "左右选位 中保存 长按左取消" :
            (model->page == DEBUG_UI_PAGE_REVIEW ? "左取消 / 松开后长按确认" :
            (model->page == DEBUG_UI_PAGE_RUNNING ? "需独立承托 / 中键STOP" : "上下翻面 中返回重新设置")));
    }
    if (model->single_static_flow && model->page == DEBUG_UI_PAGE_REVIEW &&
        model->reviewed.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
        load_view_summary(value, sizeof(value), model->load_trials.frozen_valid ? &model->load_trials.frozen : NULL);
        row(page, 4U, value);
    }
    if (load_result) {
        load_view_result_footer(value, sizeof(value), load_record != NULL ? &load_record->metadata : NULL);
        footer = value;
        if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT && model->reason != DEBUG_UI_REASON_NONE &&
            !(static_report_visible(model) && snapshot->static_load.first_exit_latched)) {
            (void)snprintf(text, sizeof(text), "ID%u / %s", model->submitted.target_motor_id, debug_ui_reason_text(model->reason));
            row(page, 0U, text);
        }
        if (model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT)
            row(page, 6U, "准备/退出先锁止 / 保持需再确认");
    } else if (model->single_static_flow && model->page == DEBUG_UI_PAGE_RUNNING && single_static_running(model) &&
        model->submitted.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
        load_view_running_footer(value, sizeof(value), model, load_record != NULL ? &load_record->metadata : NULL);
        footer = value;
    }
    layout(page, model, focus_row, page_changed);
    if (model->page == DEBUG_UI_PAGE_NUMBER) heading = "数值编辑";
    else if (model->page == DEBUG_UI_PAGE_MODES) { heading = "工作模式"; footer = "左 返回  中/右 确认切模"; }
    else if (model->page == DEBUG_UI_PAGE_RECOVERY) heading = "恢复与失能";
    else if (model->page == DEBUG_UI_PAGE_DIAGNOSTIC_MENU) heading = "诊断分类";
    else if (model->page == DEBUG_UI_PAGE_DIAGNOSTICS) {
        static const char *const tabs[]={"按键与显示 1/3","调度与通信 2/3","系统资源 3/3"};
        heading=tabs[model->diagnostic_page<3U?model->diagnostic_page:2U];
    }
    else if (model->page == DEBUG_UI_PAGE_NOTICE) { heading = "操作未提交"; footer = "中/左 返回修改"; }
    if (model->page == DEBUG_UI_PAGE_PREPARE) footer = !AETHOR_DEBUG_UI_ALLOW_MOTION || !snapshot->motion_enabled ? "只读检查 / 左 返回" : (snapshot->unconfirmed_disable_mask != 0U || pos_cleanup_locked) ? "左 返回  中 STOP" :
        (pos_load_prepare_return(model) && snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
         snapshot->target_motor_id == model->selected_motor + 1U ? "中 返回参数 / 保留控制权" :
        (snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? "左 返回  中 释放确认" : "左 返回  中 检查并申请"));
    if (pos_load_prepare_return(model)) {
        if (model->stop_requested || snapshot->stop_pending) footer = "STOP处理中 / 等待失能";
        else if (!snapshot->unconfirmed_disable_mask && !pos_cleanup_locked && snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
                 snapshot->target_motor_id != model->selected_motor + 1U) footer = "左 返回 / 中 检查当前轴";
    }
    if (!load_result && (model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT) &&
        debug_ui_model_result_requires_check(model)) footer = "中 检查  左 返回  右 诊断";
    if (model->single_static_flow && !load_result && (model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT))
        footer = "中 锁止并准备  左 锁止退出";
    if ((model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT || model->page == DEBUG_UI_PAGE_STATIC_RESULT ||
         (load_result && (model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT))) &&
        static_report_exit_text(value, sizeof(value), model)) footer = value;
    if (model->page == DEBUG_UI_PAGE_NUMBER || model->page == DEBUG_UI_PAGE_EDIT) {
        const char *action_heading = model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ? "单轴静载" :
            (model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD ? "七轴静载" :
            (model->draft.operation == DEBUG_UI_OPERATION_POS_MOVE ? "POS移动" :
            (model->draft.operation == DEBUG_UI_OPERATION_MIT_MOVE ? "MIT移动" : "MIT保持")));
        if (model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD)
            (void)snprintf(title, sizeof(title), "七轴静载 / 限时%s", diagnostics->simulated ? " [模拟]" : "");
        else (void)snprintf(title, sizeof(title), "M%02u %s%s%s", model->draft.target_motor_id, action_heading,
            model->page == DEBUG_UI_PAGE_NUMBER ? " / 数值" : " / 参数", diagnostics->simulated ? " [模拟]" : "");
    } else if (model->page == DEBUG_UI_PAGE_LOAD_METADATA || model->page == DEBUG_UI_PAGE_LOAD_NUMBER) {
        (void)snprintf(title, sizeof(title), "ID%u 载荷/记录%s", model->selected_motor + 1U, diagnostics->simulated ? " [模拟]" : "");
    } else if (model->page == DEBUG_UI_PAGE_ACTIONS || model->page == DEBUG_UI_PAGE_MODES ||
               model->page == DEBUG_UI_PAGE_RECOVERY || model->page == DEBUG_UI_PAGE_DETAIL ||
               model->page == DEBUG_UI_PAGE_SINGLE_STATIC_LOAD || model->page == DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS) {
        (void)snprintf(title, sizeof(title), "M%02u %s%s", model->selected_motor+1U, heading, diagnostics->simulated ? " [模拟]" : "");
    } else (void)snprintf(title, sizeof(title), "%s%s", heading, diagnostics->simulated ? " [模拟]" : "");
    text_set(page->title, page->title_text, sizeof(page->title_text), title);
    text_set(page->footer, page->footer_text, sizeof(page->footer_text), footer);
    if (model->page == DEBUG_UI_PAGE_NUMBER) {
        const char *operation = model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD ? "MIT 保持" :
            (model->draft.operation == DEBUG_UI_OPERATION_POS_MOVE ? "POS 移动" :
            (model->draft.operation == DEBUG_UI_OPERATION_MIT_MOVE ? "MIT 移动" : "MIT 保持"));
        if (model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD)
            (void)snprintf(title, sizeof(title), "7 / MIT 保持");
        else if (model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)
            (void)snprintf(title, sizeof(title), "ID%02u / 单轴静载", model->draft.target_motor_id);
        else (void)snprintf(title,sizeof(title),"%02u / %s",model->draft.target_motor_id,operation);
        text_set(page->title,page->title_text,sizeof(page->title_text),title);
        text_set(page->footer,page->footer_text,sizeof(page->footer_text),"上下 调值   中 保存   左 取消");
        astra_popup_label(page->title,model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ?
            &ui_font_16 : &ui_font_popup_17,page->title_text,12,8,218,LV_TEXT_ALIGN_LEFT);
        astra_popup_label(page->footer,&ui_font_popup_12,page->footer_text,0,219,280,LV_TEXT_ALIGN_CENTER);
        if (diagnostics->simulated) {
            if (strcmp(lv_label_get_text(page->icons[6]), "SIM") != 0)
                lv_label_set_text_static(page->icons[6],"SIM");
            astra_popup_label(page->icons[6],&ui_font_popup_11,"SIM",240,11,28,LV_TEXT_ALIGN_RIGHT);
            style_set_color(page->icons[6],LV_STYLE_TEXT_COLOR,lv_color_white());
            page->layout_icon_mask |= (uint8_t)(1U << 6U);
            show(page->icons[6],1U);
        }
    } else if (pos_load_view_page(model) || single_static_instrument_page(model) || load_view_page(model->page) || page->decoration>=4U) {
        style_set_pointer(page->title, LV_STYLE_TEXT_FONT, &ui_font_16);
        style_set_pointer(page->footer, LV_STYLE_TEXT_FONT, pos_load_view_page(model) ? &ui_font_16 : &ui_font_popup_12);
        style_set_color(page->title, LV_STYLE_TEXT_COLOR, lv_color_hex(0xFFFFFFU));
        style_set_color(page->footer, LV_STYLE_TEXT_COLOR, lv_color_hex(0xAAAAAAU));
        object_set_pos(page->title, 8, 3); object_set_size(page->title, 264, 25);
        object_set_pos(page->footer, 8, 216); object_set_size(page->footer, 264, 20);
        style_set_number(page->footer, LV_STYLE_TEXT_ALIGN, LV_TEXT_ALIGN_LEFT);
        style_set_number(page->title, LV_STYLE_PAD_LEFT, 4); style_set_number(page->title, LV_STYLE_PAD_TOP, 2);
        style_set_number(page->footer, LV_STYLE_PAD_LEFT, 4);
        style_set_number(page->footer, LV_STYLE_PAD_TOP, 0);
    } else {
        object_set_pos(page->title,8,2); object_set_size(page->title,264,26);
        object_set_pos(page->footer,8,212); object_set_size(page->footer,264,26);
        style_set_pointer(page->title,LV_STYLE_TEXT_FONT,&ui_font_16);
        style_set_pointer(page->footer,LV_STYLE_TEXT_FONT,&ui_font_16);
        style_set_number(page->footer,LV_STYLE_TEXT_ALIGN,LV_TEXT_ALIGN_LEFT);
        style_set_number(page->title,LV_STYLE_PAD_LEFT,4); style_set_number(page->title,LV_STYLE_PAD_TOP,2);
        style_set_number(page->footer,LV_STYLE_PAD_LEFT,4); style_set_number(page->footer,LV_STYLE_PAD_TOP,2);
        style_set_color(page->title, LV_STYLE_TEXT_COLOR, lv_color_white());
        style_set_color(page->footer, LV_STYLE_TEXT_COLOR, lv_color_white());
    }
    for (index = 0U; index < DEBUG_UI_VIEW_ROWS; ++index) {
        show(page->rows[index], (uint8_t)((page->layout_row_mask & (1U << index)) != 0U));
        show(page->icons[index], (uint8_t)((page->layout_icon_mask & (1U << index)) != 0U));
    }
    show(page->overlay, page->layout_overlay_visible);
    show(page->confirm_bar, page->layout_confirm_visible);
}

/** @brief Count missing bitmap descriptors across every decoded UTF-8 codepoint. */
uint32_t debug_ui_view_missing_glyphs(const char *utf8)
{
    uint32_t offset = 0U, missing = 0U;
    if (utf8 == NULL) return 0U;
    while (utf8[offset] != '\0') {
        uint32_t codepoint = _lv_txt_encoded_next(utf8, &offset);
        lv_font_glyph_dsc_t descriptor;
        if (codepoint <= 32U) continue;
        if (!lv_font_get_glyph_dsc(&ui_font_16, &descriptor, codepoint, 0U) ||
            descriptor.is_placeholder || lv_font_get_glyph_bitmap(&ui_font_16, codepoint) == NULL)
            ++missing;
    }
    return missing;
}
