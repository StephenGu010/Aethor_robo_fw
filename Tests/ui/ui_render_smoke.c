/** @file ui_render_smoke.c
 * @brief Real LVGL 8.3.11 software raster/flush smoke test, no HTML or hardware.
 * Fake SPI keeps a DMA buffer until its simulated completion service; this
 * exercises the production double-buffer port and wait callback as well.
 */
#include "debug_ui_view.h"
#include "debug_ui_graphics_guard.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static lv_color_t framebuffer[280U * 240U];
static DebugUiView view;
static DebugUiModel model;
static DebugUiViewDiagnostics diagnostics;
static LcdTransferDone complete_hook;
static void *complete_context;
static const lv_color_t *owned_pixels;
static LcdTransferWindow owned_window;
static uint32_t fake_ms, transaction, wait_count, flush_count;
static uint32_t framebuffer_writes;
static size_t sampled_pool_used_max;

/** @brief Match the HAL-millisecond domain in the fake platform. */
uint32_t DebugUiNowMs(void) { return fake_ms; }
/** @brief Simulate one bounded scheduler yield during production LVGL wait. */
void DebugUiWaitService(void) { ++fake_ms; ++wait_count; }
/** @brief Initialize the fake transport without allocating any GUI objects. */
int lcd_st7789_init(LcdTransferDone done, void *context)
{
    complete_hook = done; complete_context = context; return 1;
}
/** @brief A host display is ready immediately, unlike the real reset sequence. */
int lcd_st7789_start(uint32_t now_ms) { (void)now_ms; return 1; }
/** @brief Hold the exact production draw-buffer pointer until simulated EOT. */
int lcd_st7789_begin_flush(const LcdTransferWindow *window, const void *buffer,
                           uint32_t bytes, uint32_t now_ms)
{
    (void)now_ms;
    assert(owned_pixels == NULL && window != NULL && buffer != NULL);
    assert(bytes <= 13440U && bytes == (uint32_t)(window->x2 - window->x1 + 1U) *
           (uint32_t)(window->y2 - window->y1 + 1U) * sizeof(lv_color_t));
    assert(((uintptr_t)buffer & 31U) == 0U);
    owned_window = *window;
    owned_pixels = buffer;
    ++transaction;
    ++flush_count;
    return 1;
}
/** @brief Copy the owned pixels before releasing them, then notify LVGL once. */
void lcd_st7789_service(uint32_t now_ms)
{
    unsigned x, y, offset = 0U;
    (void)now_ms;
    if (owned_pixels == NULL) return;
    for (y = owned_window.y1; y <= owned_window.y2; ++y)
        for (x = owned_window.x1; x <= owned_window.x2; ++x) {
            framebuffer[y * 280U + x] = owned_pixels[offset++];
            ++framebuffer_writes;
        }
    owned_pixels = NULL;
    complete_hook(complete_context, transaction, LCD_TRANSFER_OK);
}
/** @brief Publish explicit simulated health to the production display port. */
void lcd_st7789_get_status(LcdSt7789Status *status)
{
    memset(status, 0, sizeof(*status));
    status->state = LCD_ST7789_READY;
    status->buffer_owned = (uint8_t)(owned_pixels != NULL);
}

/** @brief Guard all allocation and object creation in the production fault boundary. */
static void initialize_graphics(void *context)
{
    (void)context;
    lv_init();
    assert(lv_port_disp_init());
    assert(debug_ui_view_init(&view));
    assert(lv_port_indev_init(view.group));
}

/** @brief Fill all seven rows with clearly synthetic motor states and profiles. */
static void simulated_data(void)
{
    unsigned index;
    DebugUiSnapshot *snapshot;
    debug_ui_model_init(&model);
    snapshot = &model.snapshot;
    snapshot->timestamp_us = 1000000ULL;
    snapshot->epoch = 3U;
    snapshot->authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    snapshot->bench_profile = snapshot->motion_enabled = snapshot->mit_enabled = 1U;
    snapshot->usb_connected = 1U;
    snapshot->health.ui_valid = snapshot->health.protocol_valid = 1U;
    snapshot->health.input_valid = snapshot->health.display_valid = 1U;
    snapshot->health.ui_timestamp_us = snapshot->health.protocol_timestamp_us = snapshot->timestamp_us;
    for (index = 0U; index < 7U; ++index) {
        DebugUiMotorView *motor = &snapshot->motors[index];
        motor->motor_id = (uint8_t)(index + 1U);
        motor->feedback_seen = motor->feedback_valid = (uint8_t)(index < 5U);
        motor->identity_verified = motor->mode_verified = motor->ranges_verified = 1U;
        motor->actual_mode = index % 2U ? DEBUG_UI_MODE_MIT : DEBUG_UI_MODE_POS_VEL;
        motor->position_rad = debug_ui_degrees_to_radians(12.5f + (float)index * 3.0f);
        motor->velocity_rad_s = 0.0f;
        motor->torque_nm = 0.3f;
        motor->feedback_age_ms = 28U;
        motor->profile.pos_valid = motor->profile.mit_valid = 1U;
        motor->profile.position_min_rad = -1.0f;
        motor->profile.position_max_rad = 1.0f;
        motor->profile.max_delta_rad = 0.08f;
        motor->profile.mit_position_min_rad = -1.0f;
        motor->profile.mit_position_max_rad = 1.0f;
        motor->profile.mit_max_delta_rad = 0.08f;
        motor->profile.mit_max_speed_rad_s = 0.05f;
        motor->profile.max_speed_rad_s = 0.05f;
        motor->profile.kp_min = 1.0f; motor->profile.kp_max = 5.0f;
        motor->profile.kd_min = 0.1f; motor->profile.kd_max = 1.0f;
        motor->profile.default_kp = 2.0f; motor->profile.default_kd = 0.4f;
        motor->profile.hold_min_ms = 100U; motor->profile.hold_max_ms = 1000U;
    }
    snapshot->motors[3].fault_flags = 6U;
    debug_ui_model_update(&model, snapshot, snapshot->timestamp_us, 1U, 1U);
    memset(&diagnostics, 0, sizeof(diagnostics));
    diagnostics.simulated = 1U;
    diagnostics.firmware_version = "HOST-SIM";
    diagnostics.adc_seen = diagnostics.input_valid = 1U;
    diagnostics.adc_raw = 4092U;
    diagnostics.adc_age_ms = 3U;
    diagnostics.stack_free_words = 1200U;
    diagnostics.flush_ms = 12U;
    diagnostics.control_execution_max_us = 210U;
    diagnostics.control_period_max_us = 4020U;
}

/** @brief Reject absent glyphs and clipping for every displayed title/body/footer. */
static void validate_text(lv_obj_t *label, const char *text)
{
    lv_point_t size;
    const lv_font_t *font = lv_obj_get_style_text_font(label, 0);
    int available = lv_obj_get_width(label) - lv_obj_get_style_pad_left(label, 0) - lv_obj_get_style_pad_right(label, 0);
    uint32_t offset = 0U;
    if (lv_obj_has_flag(label, LV_OBJ_FLAG_HIDDEN)) return;
    while (text[offset] != '\0') {
        uint32_t codepoint = _lv_txt_encoded_next(text, &offset);
        lv_font_glyph_dsc_t descriptor;
        if (codepoint <= 32U) continue;
        assert(lv_font_get_glyph_dsc(font, &descriptor, codepoint, 0U) && !descriptor.is_placeholder);
    }
    lv_txt_get_size(&size, text, font, 0, 0, 32767, LV_TEXT_FLAG_NONE);
    if (size.x > available) {
        fprintf(stderr, "TEXT_OVERFLOW width=%d available=%d: %s\n", size.x, lv_obj_get_width(label) - 4, text);
        exit(2);
    }
    if (size.y > lv_obj_get_height(label) - lv_obj_get_style_pad_top(label, 0)) {
        fprintf(stderr, "TEXT_HEIGHT_OVERFLOW: %s\n", text); exit(2);
    }
}

/** @brief Draw the current model through the actual software raster and flush path. */
static void render_graphics(void *context)
{
    unsigned index;
    lv_mem_monitor_t memory_sample;
    DebugUiViewPage *page;
    (void)context;
    debug_ui_view_update(&view, &model, &diagnostics);
    lv_tick_inc(100U);
    fake_ms += 100U;
    lv_refr_now(NULL);
    lcd_st7789_service(fake_ms);
    lv_mem_monitor(&memory_sample);
    if (LV_MEM_SIZE - memory_sample.free_size > sampled_pool_used_max)
        sampled_pool_used_max = LV_MEM_SIZE - memory_sample.free_size;
    page = &view.page;
    validate_text(page->title, page->title_text);
    validate_text(page->footer, page->footer_text);
    for (index = 0U; index < DEBUG_UI_VIEW_ROWS; ++index) {
        validate_text(page->rows[index], lv_label_get_text(page->rows[index]));
        validate_text(page->icons[index], lv_label_get_text(page->icons[index]));
    }
}

/** @brief Save framebuffer pixels as a binary PPM with a visible simulation label. */
static void save_page(const char *directory, const char *name)
{
    char path[1024];
    FILE *output;
    unsigned index;
    uint32_t before = framebuffer_writes;
    assert(debug_ui_graphics_run(render_graphics, NULL));
    assert(framebuffer_writes > before && debug_ui_view_healthy());
    (void)snprintf(path, sizeof(path), "%s/%s.ppm", directory, name);
    output = fopen(path, "wb");
    assert(output != NULL);
    (void)fprintf(output, "P6\n# Real LVGL v8.3.11 / SIMULATED DATA\n280 240\n255\n");
    for (index = 0U; index < 280U * 240U; ++index) {
        lv_color32_t color;
        unsigned char pixel[3];
        color.full = lv_color_to32(framebuffer[index]);
        pixel[0] = color.ch.red; pixel[1] = color.ch.green; pixel[2] = color.ch.blue;
        assert(fwrite(pixel, 1U, sizeof(pixel), output) == sizeof(pixel));
    }
    assert(fclose(output) == 0);
    puts(path);
}

/** @brief Verify every versioned codepoint, including text on branches not pictured. */
static void verify_font_inventory(const char *path)
{
    FILE *input = fopen(path, "rb");
    char inventory[8192];
    size_t count;
    assert(input != NULL);
    count = fread(inventory, 1U, sizeof(inventory) - 1U, input);
    assert(count < sizeof(inventory) - 1U);
    inventory[count] = '\0';
    assert(fclose(input) == 0);
    assert(debug_ui_view_missing_glyphs(inventory) == 0U);
    puts("ALL_UI_GLYPHS_OK");
}

/** @brief Force real pool exhaustion to verify a local escape instead of fatal. */
static void exhaust_graphics_pool(void *context)
{
    unsigned index;
    (void)context;
    for (index = 0U; index < 2048U; ++index)
        (void)lv_obj_create(view.page.root);
}

/** @brief Render distinct requested modes using actual mode evidence and frozen requests. */
static void render_mode_reviews(const char *directory)
{
    unsigned index;
    for (index = 0U; index < 2U; ++index) {
        DebugUiMode requested = index == 0U ? DEBUG_UI_MODE_POS_VEL : DEBUG_UI_MODE_MIT;
        simulated_data();
        model.snapshot.motors[0].actual_mode = index == 0U ? DEBUG_UI_MODE_MIT : DEBUG_UI_MODE_POS_VEL;
        assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SET_MODE, requested));
        model.reviewed = model.draft; model.page = DEBUG_UI_PAGE_REVIEW;
        save_page(directory, index == 0U ? "17_review_set_pos" : "18_review_set_mit");
        assert(strstr(view.page.row_text[1], index == 0U ? "MIT -> POS" : "POS -> MIT") != NULL);
        assert(strstr(view.page.row_text[3], "预计") == NULL);
    }
}

/** @brief Render actual target resolution and both result-less STOP cleanup outcomes. */
static void render_stop_latch(const char *directory, uint8_t confirmed)
{
    DebugUiRequest request;
    DebugUiInputEvent event;
    event.type = DEBUG_UI_INPUT_EVENT_PRESS; event.key = DEBUG_UI_KEY_CENTER;
    event.timestamp_ms = 1000U; event.held_ms = 0U;
    simulated_data();
    model.submitted.target_motor_id = 1U; /* Intentionally stale local action. */
    model.snapshot.active = 1U; model.snapshot.active_motor_mask = 4U;
    model.snapshot.target_motor_id = 0U; model.snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    model.snapshot.motors[2].enabled = 1U;
    debug_ui_model_update(&model, &model.snapshot, model.snapshot.timestamp_us, 1U, 1U);
    if (confirmed) {
        save_page(directory, "19_remote_id3");
        assert(strstr(view.page.row_text[0], "ID3") != NULL);
        assert(strstr(view.page.row_text[3], "18.5") != NULL);
    }
    debug_ui_model_event(&model, &event);
    assert(debug_ui_model_take_request(&model, &request) && request.target_motor_id == 3U);
    debug_ui_model_submit_failed(&model, &request, DEBUG_UI_REASON_STOP_LATCHED);
    model.snapshot.timestamp_us += 20000ULL;
    model.snapshot.active = 0U; model.snapshot.stop_pending = 1U;
    model.snapshot.active_stage = DEBUG_UI_STAGE_DISABLE;
    debug_ui_model_update(&model, &model.snapshot, model.snapshot.timestamp_us, 1U, 1U);
    if (confirmed) {
        save_page(directory, "20_stop_latched");
        assert(strstr(view.page.row_text[1], "停止已锁存") != NULL);
    }
    model.snapshot.timestamp_us += 20000ULL;
    model.snapshot.stop_pending = 0U; model.snapshot.active_motor_mask = 0U;
    model.snapshot.motors[2].enabled = (uint8_t)!confirmed;
    model.snapshot.motors[2].feedback_age_ms = 1U;
    debug_ui_model_update(&model, &model.snapshot, model.snapshot.timestamp_us, 1U, 1U);
    assert(!model.stop_waiting && !model.completion_seen);
    assert(model.page == (confirmed ? DEBUG_UI_PAGE_RESULT : DEBUG_UI_PAGE_FAULT));
    save_page(directory, confirmed ? "21_stop_latch_confirmed" : "22_stop_latch_unconfirmed");
    assert(strstr(view.page.row_text[3], confirmed ? "反馈已确认失能" : "失能未确认") != NULL);
}

/** @brief Make a multi-motor STOP scope visible and identify the position sample's ID. */
static void render_multiple_targets(const char *directory)
{
    simulated_data();
    model.snapshot.active = 1U; model.snapshot.active_motor_mask = 0x14U;
    model.snapshot.target_motor_id = 0U; model.snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    debug_ui_model_update(&model, &model.snapshot, model.snapshot.timestamp_us, 1U, 1U);
    save_page(directory, "23_remote_multiple_targets");
    if (strstr(view.page.row_text[0], "ID3等2台") == NULL ||
        strstr(view.page.row_text[3], "ID3") == NULL) {
        fputs("MULTI_TARGET_REGRESSION_FAIL scope_count_or_position_ID_missing\n", stderr);
        exit(1);
    }
}

/** @brief Render six required page families plus edit/review/result variants. */
#include "icon_ui_scenarios.inc"

/** @brief Exercise repeated actual LVGL layouts and assert no retained page allocations. */
static void verify_shared_view_reuse(void)
{
    unsigned iteration;
    lv_mem_monitor_t warmed, final_memory;
    simulated_data();
    for (iteration = 0U; iteration < 1212U; ++iteration) {
        model.page = (DebugUiPage)(iteration % DEBUG_UI_PAGE_COUNT);
        assert(debug_ui_graphics_run(render_graphics, NULL));
        assert(lv_obj_get_child_cnt(view.page.root) == 17U);
        if (iteration == 11U) lv_mem_monitor(&warmed);
    }
    lv_mem_monitor(&final_memory);
    assert(final_memory.free_size == warmed.free_size);
    printf("LVGL_SHARED_VIEW_REUSE_OK updates=1212 children=17 view_bytes=%lu sampled_pool_used_max=%lu\n",
        (unsigned long)sizeof(DebugUiView), (unsigned long)sampled_pool_used_max);
}

/** @brief Render all operating states, validate shared allocations and exercise OOM escape. */
int main(int argc, char **argv)
{
    lv_mem_monitor_t memory;
    unsigned objects_before;
    (void)setvbuf(stdout, NULL, _IONBF, 0U);
    assert(argc == 3);
    /* One shared view must fit without retaining every page's text and objects. */
    assert(sizeof(DebugUiView) < 4096U);
    assert(LVGL_VERSION_MAJOR == 8 && LVGL_VERSION_MINOR == 3 && LVGL_VERSION_PATCH == 11);
    assert(debug_ui_graphics_run(initialize_graphics, NULL));
    verify_font_inventory(argv[2]);
    simulated_data();
    objects_before = lv_obj_get_child_cnt(view.page.root);
    save_page(argv[1], "01_overview");
    model.page = DEBUG_UI_PAGE_MOTORS; model.selected_motor = 6U; model.list_first = 2U;
    save_page(argv[1], "02_motors_scrolled");
    model.page = DEBUG_UI_PAGE_DETAIL; model.selected_motor = 0U;
    save_page(argv[1], "03_motor_detail");
    model.snapshot.motors[0].feedback_valid = 0U; model.snapshot.motors[0].feedback_age_ms = 920U;
    save_page(argv[1], "04_feedback_stale");
    model.snapshot.motors[0].feedback_valid = 1U; model.snapshot.motors[0].feedback_age_ms = 20U;
    model.page = DEBUG_UI_PAGE_DIAGNOSTICS;
    save_page(argv[1], "05_diagnostics");
    model.diagnostic_page = 1U;
    save_page(argv[1], "06_control_diagnostics");
    model.diagnostic_page = 2U;
    save_page(argv[1], "16_runtime_diagnostics");
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    save_page(argv[1], "07_pos_draft");
    /* Expired standard feedback must not appear as a usable absolute POS preview. */
    model.snapshot.motors[model.selected_motor].feedback_valid = 0U;
    debug_ui_view_update(&view, &model, &diagnostics);
    assert(strcmp(view.page.row_text[2], "动作前 --") == 0);
    assert(strcmp(view.page.row_text[3], "目标未知 / 等待反馈") == 0);
    model.snapshot.motors[model.selected_motor].feedback_valid = 1U;
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_MIT_MOVE, DEBUG_UI_MODE_MIT));
    save_page(argv[1], "08_mit_draft");
    model.reviewed = model.draft; model.page = DEBUG_UI_PAGE_REVIEW;
    save_page(argv[1], "09_review_release");
    model.review_released = 1U; model.confirm_elapsed_ms = 650U;
    save_page(argv[1], "10_review_hold");
    model.page = DEBUG_UI_PAGE_RUNNING; model.admission_seen = 1U;
    model.snapshot.active = 1U; model.snapshot.active_motor_mask = 1U;
    model.snapshot.active_identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    model.snapshot.active_stage = DEBUG_UI_STAGE_MOTION; model.snapshot.target_motor_id = 1U;
    save_page(argv[1], "11_running");
    model.stop_requested = 1U; model.snapshot.active_stage = DEBUG_UI_STAGE_DISABLE;
    save_page(argv[1], "12_stop_cleanup");
    model.snapshot.motion_enabled = 0U; model.stop_requested = 0U;
    save_page(argv[1], "15_readonly_running");
    model.snapshot.motion_enabled = 1U;
    model.snapshot.active = 0U; model.snapshot.active_motor_mask = 0U;
    model.page = DEBUG_UI_PAGE_RESULT; model.completion_seen = 1U;
    model.completion.code = DEBUG_UI_COMPLETED; model.completion.disabled_confirmed = 1U;
    save_page(argv[1], "13_result");
    model.page = DEBUG_UI_PAGE_FAULT; model.completion.code = DEBUG_UI_FAILED;
    model.completion.disabled_confirmed = 0U; model.completion.error = 6U;
    model.completion.stage = DEBUG_UI_STAGE_DISABLE; model.reason = DEBUG_UI_REASON_STALE_FEEDBACK;
    save_page(argv[1], "14_fault");
    render_mode_reviews(argv[1]);
    render_stop_latch(argv[1], 1U);
    render_stop_latch(argv[1], 0U);
    render_multiple_targets(argv[1]);
    simulated_data();
    model.page = DEBUG_UI_PAGE_PREPARE;
    model.selected_motor = 6U;
    model.snapshot.motion_enabled = 0U;
    model.snapshot.motors[6].profile.pos_valid = 0U;
    save_page(argv[1], "24_control_prepare");
    model.page = DEBUG_UI_PAGE_REGISTERS;
    model.snapshot.motors[6].register_seen_mask = 3U;
    model.snapshot.motors[6].register_position[0] = 0.000219124f;
    model.snapshot.motors[6].register_position[1] = 0.4f;
    model.snapshot.motors[6].register_age_ms[0] = 20U;
    model.snapshot.motors[6].register_age_ms[1] = 650U;
    save_page(argv[1], "25_position_registers");
    assert(strstr(view.page.row_text[1], "0.000219124") != NULL);
    render_icon_scenarios(argv[1]);
    verify_shared_view_reuse();
    assert(lv_obj_get_child_cnt(view.page.root) == objects_before);
    assert(wait_count > 0U && flush_count > 100U && owned_pixels == NULL);
    lv_mem_monitor(&memory);
    assert(memory.free_size > 4096U);
    printf("LVGL_RENDER_SMOKE_OK version=8.3.11 flush=%lu wait=%lu pool_used=%lu pool_free=%lu\n",
           (unsigned long)flush_count, (unsigned long)wait_count, (unsigned long)(LV_MEM_SIZE - memory.free_size),
           (unsigned long)memory.free_size);
    assert(!debug_ui_graphics_run(exhaust_graphics_pool, NULL));
    assert(!debug_ui_view_healthy() && owned_pixels == NULL);
    puts("LVGL_OOM_LOCAL_ESCAPE_OK (no global fatal, no premature buffer release)");
    return 0;
}
