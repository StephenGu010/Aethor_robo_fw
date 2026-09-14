/** @file debug_ui_view.c
 * @brief Shared LVGL v8 icon view; no page allocations after initialization.
 * Semantic rows retain evidence, while bounded display slots implement templates.
 */
#include "debug_ui_view.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#if defined(__CC_ARM)
__attribute__((section(".lvgl_pool"), zero_init, aligned(32)))
#else
__attribute__((section(".lvgl_pool"), aligned(32)))
#endif
uint8_t debug_ui_lvgl_pool[LV_MEM_SIZE];
static volatile uint8_t graphics_fault;

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
/** @brief Create one bounded, non-scrolling label. */
static lv_obj_t *label_create(lv_obj_t *parent, int x, int y, int width, int height)
{
    lv_obj_t *label = lv_label_create(parent);
    if (label == NULL) return NULL;
    lv_obj_set_pos(label, (lv_coord_t)x, (lv_coord_t)y);
    lv_obj_set_size(label, (lv_coord_t)width, (lv_coord_t)height);
    lv_obj_set_style_text_font(label, &ui_font_16, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xDCE8F2), 0);
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
        lv_obj_set_style_bg_color(page->root, lv_color_hex(0x101820), 0);
        lv_obj_clear_flag(page->root, LV_OBJ_FLAG_SCROLLABLE);
        page->title = label_create(page->root, 8, 2, 264, 26);
        page->footer = label_create(page->root, 8, 212, 264, 26);
        if (page->title == NULL || page->footer == NULL) return 0U;
        lv_obj_set_style_bg_opa(page->title, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(page->title, lv_color_hex(0x17232C), 0);
        lv_obj_set_style_bg_opa(page->footer, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(page->footer, lv_color_hex(0x17232C), 0);
        lv_obj_set_style_text_color(page->footer, lv_color_hex(0x8EB1C8), 0);
        for (row_index = 0U; row_index < DEBUG_UI_VIEW_ROWS; ++row_index) {
            page->rows[row_index] = label_create(page->root, 8, 34 + (int)row_index * 24, 264, 24);
            page->icons[row_index] = label_create(page->root, 16, 34, 28, 30);
            if (page->rows[row_index] == NULL || page->icons[row_index] == NULL) return 0U;
            lv_obj_set_style_text_font(page->icons[row_index], &lv_font_montserrat_20, 0);
            lv_obj_set_style_text_color(page->icons[row_index], lv_color_hex(0x56D7E6), 0);
            lv_label_set_text_static(page->rows[row_index], page->display_text[row_index]);
            lv_label_set_text_static(page->icons[row_index], "");
        }
        page->confirm_bar = lv_obj_create(page->root);
        if (page->confirm_bar == NULL) return 0U;
        lv_obj_remove_style_all(page->confirm_bar);
        lv_obj_set_pos(page->confirm_bar, 10, 207);
        lv_obj_set_size(page->confirm_bar, 1, 3);
        lv_obj_set_style_bg_opa(page->confirm_bar, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(page->confirm_bar, lv_color_hex(0x56D7E6), 0);
        lv_obj_add_flag(page->confirm_bar, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text_static(page->title, page->title_text);
        lv_label_set_text_static(page->footer, page->footer_text);
    }
    view->visible = DEBUG_UI_PAGE_OVERVIEW;
    lv_scr_load(page->root);
    view->initialized = debug_ui_view_healthy();
    return view->initialized;
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
        "MIT 有限保持", "MIT 相对移动", "切换模式", "清除故障", "请求失能", "STOP 停止" };
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
    if (visible) lv_obj_clear_flag(object, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(object, LV_OBJ_FLAG_HIDDEN);
}

/** @brief Render one bounded slot; symbols use LVGL's bundled font, not the CJK font. */
static void slot(DebugUiViewPage *page, unsigned index, int x, int y, int width, int height,
                 const char *value, const char *symbol, uint8_t selected, uint32_t color,
                 const lv_font_t *font)
{
    lv_obj_t *label = page->rows[index];
    lv_obj_t *glyph = page->icons[index];
    lv_obj_set_pos(label, (lv_coord_t)x, (lv_coord_t)y);
    lv_obj_set_size(label, (lv_coord_t)width, (lv_coord_t)height);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_pad_left(label, symbol != NULL ? 34 : 4, 0);
    lv_obj_set_style_pad_top(label, height < 23 ? 0 : 2, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    lv_obj_set_style_bg_color(label, lv_color_hex(selected ? 0x234351 : 0x101820), 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(label, lv_color_hex(0x56D7E6), 0);
    lv_obj_set_style_border_width(label, selected ? 1 : 0, 0);
    text_set(label, page->display_text[index], sizeof(page->display_text[index]), value);
    show(label, 1U);
    show(glyph, (uint8_t)(symbol != NULL));
    if (symbol != NULL) {
        lv_obj_set_pos(glyph, (lv_coord_t)(x + 5), (lv_coord_t)(y + 1));
        lv_obj_set_height(glyph, (lv_coord_t)(height < 30 ? height - 1 : 29));
        lv_obj_set_style_text_font(glyph, height < 26 ? &lv_font_montserrat_16 : &lv_font_montserrat_20, 0);
        lv_obj_set_style_pad_top(glyph, 0, 0);
        lv_obj_set_style_text_color(glyph, lv_color_hex(color), 0);
        if (strcmp(lv_label_get_text(glyph), symbol) != 0) lv_label_set_text_static(glyph, symbol);
    }
}

/** @brief Apply the approved templates to current evidence without changing model state. */
static void layout(DebugUiViewPage *page, const DebugUiModel *model, int focus_row)
{
    unsigned index;
    uint8_t used = 0x7FU;
    uint8_t defaults = 0x7FU;
    char value[160];
    const DebugUiMotorView *motor = &model->snapshot.motors[model->selected_motor < 7U ? model->selected_motor : 0U];
    const DebugUiRequest *action = model->page == DEBUG_UI_PAGE_REVIEW ? &model->reviewed : &model->draft;
    const uint32_t normal = 0xEDF4F7U, muted = 0xA2B3BFU, cyan = 0x56D7E6U;
    if (model->page == DEBUG_UI_PAGE_OVERVIEW || model->page == DEBUG_UI_PAGE_ACTIONS ||
        model->page == DEBUG_UI_PAGE_DETAIL || model->page == DEBUG_UI_PAGE_RUNNING ||
        model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT) defaults = 0U;
    else if (model->page == DEBUG_UI_PAGE_REVIEW) defaults = 0x3FU;
    else if (model->page == DEBUG_UI_PAGE_EDIT && (action->operation == DEBUG_UI_OPERATION_POS_MOVE ||
             action->operation == DEBUG_UI_OPERATION_MIT_MOVE || action->operation == DEBUG_UI_OPERATION_MIT_HOLD)) defaults = 1U;
    for (index = 0U; index < DEBUG_UI_VIEW_ROWS; ++index) if ((defaults & (1U << index)) != 0U)
        slot(page, index, 8, 34 + (int)index * 24, 264, 23, page->row_text[index], NULL,
             (uint8_t)((int)index == focus_row), normal, &ui_font_16);
    show(page->confirm_bar, 0U);
    switch (model->page) {
    case DEBUG_UI_PAGE_OVERVIEW:
        for (index = 0U; index < 3U; ++index)
            slot(page, index, 8, 32 + (int)index * 21, 264, 21, page->row_text[index], NULL, 0U, muted, &ui_font_16);
        for (index = 3U; index < 6U; ++index) {
            static const char *const symbols[] = { LV_SYMBOL_LIST, LV_SYMBOL_SETTINGS, LV_SYMBOL_POWER };
            const char *label = index == 3U ? "电机" : (index == 4U ? "诊断" :
                (model->snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? "释放本地控制" :
                (model->snapshot.motion_enabled ? "申请本地控制" : "控制检查 / 只读")));
            slot(page, index, 8, 98 + (int)(index - 3U) * 36, 264, 32, label, symbols[index-3U],
                 (uint8_t)((int)index == focus_row), normal, &ui_font_16);
        }
        used = 0x3FU;
        break;
    case DEBUG_UI_PAGE_ACTIONS: {
        static const char *const labels[] = { "POS移动", "MIT保持", "MIT移动", "切换POS", "切换MIT", "清除故障", "请求失能", "位置读值" };
        static const char *const symbols[] = { LV_SYMBOL_PLAY, LV_SYMBOL_PAUSE, LV_SYMBOL_PLAY, LV_SYMBOL_REFRESH,
                                              LV_SYMBOL_REFRESH, LV_SYMBOL_WARNING, LV_SYMBOL_STOP, LV_SYMBOL_LIST };
        unsigned first = (model->focus / 4U) * 4U;
        (void)snprintf(value, sizeof(value), "ID%u 操作 %u/2", model->selected_motor+1U, first/4U+1U);
        slot(page, 0U, 8, 34, 264, 23, value, NULL, 0U, muted, &ui_font_16);
        for (index = 0U; index < 4U; ++index) {
            slot(page, index+1U, 8+(int)(index%2U)*134, 60+(int)(index/2U)*62, 130, 60,
                 labels[first+index], symbols[first+index], (uint8_t)(first+index == model->focus), normal, &ui_font_16);
            lv_obj_set_style_pad_top(page->rows[index+1U], 33, 0);
            lv_obj_set_style_pad_left(page->rows[index+1U], 8, 0);
        }
        used = 0x1FU;
        if (model->reason != DEBUG_UI_REASON_NONE) {
            slot(page, 5U, 8, 188, 264, 23, debug_ui_reason_text(model->reason), NULL, 0U, 0xFFC66DU, &ui_font_16);
            used |= 0x20U;
        }
        break;
    }
    case DEBUG_UI_PAGE_DETAIL:
        debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(motor->position_rad),
                              motor->feedback_valid, motor->feedback_valid, "°");
        slot(page, 0U, 8, 33, 264, 23, page->row_text[0], NULL, 0U, muted, &ui_font_16);
        slot(page, 1U, 8, 61, 264, 36, value, LV_SYMBOL_SETTINGS, 0U, normal, &lv_font_montserrat_28);
        slot(page, 2U, 8, 103, 264, 23, page->row_text[2], NULL, 0U, normal, &ui_font_16);
        slot(page, 3U, 8, 125, 264, 23, page->row_text[3], NULL, 0U, normal, &ui_font_16);
        slot(page, 4U, 8, 147, 264, 23, page->row_text[4], NULL, 0U, muted, &ui_font_16);
        slot(page, 5U, 8, 169, 264, 23, page->row_text[5], NULL, 0U, muted, &ui_font_16);
        slot(page, 6U, 8, 191, 264, 21, "打开操作菜单", LV_SYMBOL_LIST, 1U, cyan, &ui_font_16);
        break;
    case DEBUG_UI_PAGE_EDIT:
        if (action->operation == DEBUG_UI_OPERATION_POS_MOVE || action->operation == DEBUG_UI_OPERATION_MIT_MOVE ||
            action->operation == DEBUG_UI_OPERATION_MIT_HOLD) {
            static const char *const fields[] = { "相对角度", "速度", "Kp", "Kd", "保持时间" };
            unsigned field = model->edit_field % 5U;
            if (field == 0U) (void)snprintf(value, sizeof(value), "%+.1f°", (double)debug_ui_radians_to_degrees(action->delta_rad));
            else if (field == 1U) (void)snprintf(value, sizeof(value), "%.1f°/s", (double)debug_ui_radians_to_degrees(action->speed_rad_s));
            else if (field == 2U) (void)snprintf(value, sizeof(value), "%.2f", (double)action->kp);
            else if (field == 3U) (void)snprintf(value, sizeof(value), "%.2f", (double)action->kd);
            else (void)snprintf(value, sizeof(value), "%lu ms", (unsigned long)action->hold_duration_ms);
            slot(page, 1U, 8, 61, 264, 23, fields[field], NULL, 0U, cyan, &ui_font_16);
            slot(page, 2U, 8, 87, 264, 36, value, NULL, 1U, normal, &lv_font_montserrat_28);
            slot(page, 3U, 8, 127, 264, 23, page->row_text[1], NULL, 0U, muted, &ui_font_16);
            slot(page, 4U, 8, 150, 264, 21, page->row_text[3], NULL, 0U, muted, &ui_font_16);
            slot(page, 5U, 8, 171, 264, 20, debug_ui_reason_text(model->reason), NULL, 0U, 0xFFC66DU, &ui_font_16);
            slot(page, 6U, 8, 191, 264, 21, "中键 查看确认", LV_SYMBOL_PLAY, 0U, cyan, &ui_font_16);
        }
        break;
    case DEBUG_UI_PAGE_REVIEW:
        slot(page, 6U, 8, 179, 264, 27, page->row_text[6], LV_SYMBOL_OK, 1U, cyan, &ui_font_16);
        if (model->review_released && model->confirm_elapsed_ms > 0U) {
            uint32_t elapsed = model->confirm_elapsed_ms > 800U ? 800U : model->confirm_elapsed_ms;
            lv_obj_set_width(page->confirm_bar, (lv_coord_t)(260U * elapsed / 800U));
            show(page->confirm_bar, 1U);
        }
        break;
    case DEBUG_UI_PAGE_RUNNING: {
        uint8_t target = debug_ui_model_active_target(model);
        const DebugUiMotorView *active_motor = &model->snapshot.motors[target != 0U ? target - 1U : 0U];
        slot(page, 0U, 8, 32, 264, 21, page->row_text[0], NULL, 0U, muted, &ui_font_16);
        slot(page, 1U, 8, 56, 264, 23, page->row_text[1], NULL, 0U, normal, &ui_font_16);
        slot(page, 2U, 8, 82, 264, 23, page->row_text[2], NULL, 0U, cyan, &ui_font_16);
        debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(active_motor->position_rad),
            (uint8_t)(target != 0U && active_motor->feedback_valid), active_motor->feedback_valid, "°");
        slot(page, 3U, 8, 110, 264, 38, value, NULL, 0U, normal, &lv_font_montserrat_28);
        slot(page, 4U, 8, 156, 264, 34, page->row_text[4],
             model->snapshot.motion_enabled ? LV_SYMBOL_STOP : LV_SYMBOL_EYE_OPEN, 1U,
             model->snapshot.motion_enabled ? 0xFF7D80U : muted, &ui_font_16);
        slot(page, 5U, 8, 190, 264, 22, page->row_text[6], NULL, 0U, muted, &ui_font_16);
        used = 0x3FU;
        break;
    }
    case DEBUG_UI_PAGE_RESULT:
    case DEBUG_UI_PAGE_FAULT: {
        uint8_t confirmed = (uint8_t)(model->stop_latch_state == DEBUG_UI_STOP_LATCH_CONFIRMED ||
            (model->stop_latch_state == DEBUG_UI_STOP_LATCH_NONE && model->completion_seen && model->completion.disabled_confirmed));
        uint32_t color = confirmed ? 0x75D9A2U : 0xFF7D80U;
        if (model->completion_seen && model->completion.code != DEBUG_UI_COMPLETED) color = 0xFFC66DU;
        slot(page, 0U, 8, 36, 264, 30, page->row_text[0],
             confirmed && model->completion_seen && model->completion.code == DEBUG_UI_COMPLETED ? LV_SYMBOL_OK : LV_SYMBOL_WARNING,
             0U, color, &ui_font_16);
        slot(page, 1U, 8, 71, 264, 23, page->row_text[1], NULL, 0U, normal, &ui_font_16);
        slot(page, 2U, 8, 99, 264, 23, page->row_text[3], NULL, 0U, confirmed ? normal : 0xFF7D80U, &ui_font_16);
        slot(page, 3U, 8, 127, 264, 23, page->row_text[2], NULL, 0U, muted, &ui_font_16);
        slot(page, 4U, 8, 152, 264, 23, page->row_text[4], NULL, 0U, muted, &ui_font_16);
        slot(page, 5U, 8, 179, 264, 28, "已读返回", LV_SYMBOL_LEFT, 1U, cyan, &ui_font_16);
        used = 0x3FU;
        break;
    }
    default: break;
    }
    for (index = 0U; index < 7U; ++index) if ((used & (1U << index)) == 0U) {
        show(page->rows[index], 0U); show(page->icons[index], 0U);
    }
}

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
    if (view == NULL || model == NULL || diagnostics == NULL || !view->initialized || !debug_ui_view_healthy()) return;
    if ((unsigned)model->page >= DEBUG_UI_PAGE_COUNT) return;
    page = &view->page;
    snapshot = &model->snapshot;
    motor = &snapshot->motors[model->selected_motor < DEBUG_UI_MOTOR_COUNT ? model->selected_motor : 0U];
    if (view->visible != model->page) {
        view->visible = model->page;
    }
    for (index = 0U; index < DEBUG_UI_VIEW_ROWS; ++index) page->row_text[index][0] = '\0';
    /* Compose into a bounded local row matrix, then update only changed strings. */
    switch (model->page) {
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
        focus_row = 3 + model->focus % 3U;
        footer = "上下 选择  中/右 进入";
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
        row(page, 5U, debug_ui_reason_text(debug_ui_model_gate(model,
            snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ? DEBUG_UI_OPERATION_RELEASE : DEBUG_UI_OPERATION_ACQUIRE)));
        row(page, 6U, snapshot->unconfirmed_disable_mask != 0U ? "中键 停止" : "中键 重新检查"); focus_row = 6;
        footer = "左 返回  右 电机详情";
        break;
    case DEBUG_UI_PAGE_ACTIONS:
        heading = "电机操作";
        footer = "左 详情  上下 选择  中 进入";
        break;
    case DEBUG_UI_PAGE_MOTORS:
        heading = "电机列表";
        for (index = 0U; index < DEBUG_UI_VISIBLE_MOTORS; ++index) {
            unsigned motor_index = model->list_first + index;
            const DebugUiMotorView *item = &snapshot->motors[motor_index];
            debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(item->position_rad),
                                   item->feedback_seen, item->feedback_valid, "°");
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
        row(page, 5U, "位置: 电机输出轴 °");
        (void)snprintf(text, sizeof(text), "行 %u-%u / 7  本次上电零点", model->list_first + 1U, model->list_first + 5U); row(page, 6U, text);
        break;
    case DEBUG_UI_PAGE_DETAIL: {
        static const char *const choices[8] = { "POS移动", "MIT保持", "MIT移动", "切换POS", "切换MIT", "清除故障", "请求失能", "位置读值" };
        heading = "电机详情";
        (void)snprintf(text, sizeof(text), "ID%u %s / %s", model->selected_motor + 1U, mode_text(motor->actual_mode),
            motor->feedback_seen ? (motor->feedback_valid ? (motor->enabled ? "已使能" : "已失能") : "状态过期") : "状态未知"); row(page, 0U, text);
        debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(motor->position_rad), motor->feedback_seen, motor->feedback_valid, "°");
        (void)snprintf(text, sizeof(text), "位置 %s", value); row(page, 1U, text);
        debug_ui_format_value(value, sizeof(value), debug_ui_radians_to_degrees(motor->velocity_rad_s), motor->feedback_seen, motor->feedback_valid, "°/s");
        (void)snprintf(text, sizeof(text), "速度 %s", value); row(page, 2U, text);
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
        footer = "输出轴  左 列表  中 操作";
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
            (void)snprintf(text, sizeof(text), "RID%02X %s", 0x50U + index, value); row(page, 1U + 2U * index, text);
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
            (void)snprintf(text, sizeof(text), "DMA错误 %lu  块 %lums", (unsigned long)diagnostics->dma_errors, (unsigned long)diagnostics->flush_ms); row(page, 3U, text);
            (void)snprintf(text, sizeof(text), "UI栈余量 %lu word", (unsigned long)diagnostics->stack_free_words); row(page, 4U, text);
            (void)snprintf(text, sizeof(text), "控制耗时最大 %lu us", (unsigned long)diagnostics->control_execution_max_us); row(page, 5U, text);
            (void)snprintf(text, sizeof(text), "控制周期最大 %lu us", (unsigned long)diagnostics->control_period_max_us); row(page, 6U, text);
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
        footer = "左 返回  中/右 诊断翻页";
        break;
    case DEBUG_UI_PAGE_EDIT:
    case DEBUG_UI_PAGE_REVIEW:
        action = model->page == DEBUG_UI_PAGE_EDIT ? &model->draft : &model->reviewed;
        if (action->target_motor_id > 0U && action->target_motor_id <= DEBUG_UI_MOTOR_COUNT)
            motor = &snapshot->motors[action->target_motor_id - 1U];
        heading = model->page == DEBUG_UI_PAGE_EDIT ? "动作草稿" : "二次确认";
        (void)snprintf(text, sizeof(text), "ID%u %s", action->target_motor_id, operation_text(action->operation)); row(page, 0U, text);
        if (action->operation == DEBUG_UI_OPERATION_SET_MODE) {
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
        if (action->operation == DEBUG_UI_OPERATION_MIT_HOLD || action->operation == DEBUG_UI_OPERATION_MIT_MOVE) {
            (void)snprintf(text, sizeof(text), "MIT范围 %.1f~%.1f°",
                (double)debug_ui_radians_to_degrees(motor->profile.mit_position_min_rad),
                (double)debug_ui_radians_to_degrees(motor->profile.mit_position_max_rad)); row(page, 4U, text);
        } else row(page, 4U, action->operation == DEBUG_UI_OPERATION_SET_MODE ? "切换后仍保持失能" : "输出轴 / 本次上电零点");
        row(page, 5U, action->operation == DEBUG_UI_OPERATION_POS_MOVE || action->operation == DEBUG_UI_OPERATION_MIT_MOVE ||
            action->operation == DEBUG_UI_OPERATION_MIT_HOLD || action->operation == DEBUG_UI_OPERATION_DISABLE ?
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
            footer = "上下 调值  右 字段  中 预览";
        }
        focus_row = 6;
        break;
    case DEBUG_UI_PAGE_RUNNING: {
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
    layout(page, model, focus_row);
    (void)snprintf(title, sizeof(title), "%s%s", heading, diagnostics->simulated ? " [模拟]" : "");
    text_set(page->title, page->title_text, sizeof(page->title_text), title);
    text_set(page->footer, page->footer_text, sizeof(page->footer_text), footer);
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
