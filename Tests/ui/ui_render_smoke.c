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
#include <math.h>
#include <float.h>

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
static unsigned static_status_regression_failures; /**< Accumulate independent presentation regressions before one terminal assert. */

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
        {
            uint8_t glyph_valid=(uint8_t)(lv_font_get_glyph_dsc(font,&descriptor,codepoint,0U) && !descriptor.is_placeholder);
            if (!glyph_valid)
                fprintf(stderr,"MISSING_DISPLAY_GLYPH page=%u codepoint=%lu font_height=%u font=%p text=%s\n",
                    (unsigned)model.page,(unsigned long)codepoint,(unsigned)font->line_height,(const void *)font,text);
            assert(glyph_valid);
        }
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

/** @brief Assert retained visual geometry and unchanged fixed-object/32 KiB resource bounds. */
static void verify_static_art_geometry(void)
{
    unsigned index;
    const DebugUiStaticArt *art=&view.page.static_art;
    assert(art->icon_count<=12U && art->panel_count<=6U && art->progress_steps<=24U);
    assert(lv_obj_get_child_cnt(view.page.root)==18U);
    assert(LV_MEM_SIZE==32768U && LV_DRAW_COMPLEX==0);
    for (index=0U;index<art->icon_count;++index) {
        const DebugUiStaticIcon *icon=&art->icons[index];
        assert(icon->x>=12 && icon->y>=33 && icon->scale>=1U && icon->scale<=2U);
        assert(icon->x+24*icon->scale+2<=269 && icon->y+24*icon->scale+2<=211);
    }
}

/** @brief Count exact raster ink in a bounded region to prove diagonal/selection output. */
static unsigned raster_ink_count(int left,int top,int right,int bottom,uint8_t black)
{
    unsigned count=0U;
    int x,y;
    uint16_t ink=lv_color_to16(black?lv_color_black():lv_color_white());
    assert(left>=0 && top>=0 && right<280 && bottom<240);
    for (y=top;y<=bottom;++y)
        for (x=left;x<=right;++x)
            if (lv_color_to16(framebuffer[y*280+x])==ink) ++count;
    return count;
}

/** @brief Draw the current model through the actual software raster and flush path. */
static void render_graphics(void *context)
{
    unsigned index;
    lv_mem_monitor_t memory_sample;
    DebugUiViewPage *page;
    (void)context;
    debug_ui_view_update(&view, &model, &diagnostics);
    /* Static page exports show the settled 140 ms Tile; timing is tested separately. */
    lv_tick_inc(160U);
    fake_ms += 160U;
    debug_ui_view_animate(&view);
    lv_refr_now(NULL);
    lcd_st7789_service(fake_ms);
    lv_mem_monitor(&memory_sample);
    if (LV_MEM_SIZE - memory_sample.free_size > sampled_pool_used_max)
        sampled_pool_used_max = LV_MEM_SIZE - memory_sample.free_size;
    page = &view.page;
    verify_static_art_geometry();
    validate_text(page->title, page->title_text);
    validate_text(page->footer, page->footer_text);
    for (index = 0U; index < DEBUG_UI_VIEW_ROWS; ++index) {
        validate_text(page->rows[index], lv_label_get_text(page->rows[index]));
        validate_text(page->icons[index], lv_label_get_text(page->icons[index]));
    }
    if (page->decoration == 2U || page->decoration == 3U) {
        for (index = 0U; index < 5U; ++index) {
            lv_point_t size;
            assert(debug_ui_view_missing_glyphs(page->background_text[index]) == 0U);
            lv_txt_get_size(&size, page->background_text[index], &ui_font_16,
                            0, 0, 32767, LV_TEXT_FLAG_NONE);
            assert(size.x <= 237 && size.y <= 21);
            if (page->decoration == 3U) {
                const char *value = page->background_text[index]+40;
                uint32_t offset=0U;
                int caption_width=size.x;
                while (value[offset] != '\0') {
                    lv_font_glyph_dsc_t glyph;
                    uint32_t codepoint=_lv_txt_encoded_next(value,&offset);
                    assert(lv_font_get_glyph_dsc(&ui_font_popup_14,&glyph,codepoint,0U) && !glyph.is_placeholder);
                }
                lv_txt_get_size(&size,value,&ui_font_popup_14,0,0,32767,LV_TEXT_FLAG_NONE);
                assert(caption_width+size.x+12 <= 231 && size.y <= 18);
            }
        }
    }
}

/** @brief Measure pixels flushed by one model-to-view update after pending DMA completes. */
static uint32_t render_update_pixels(void)
{
    uint32_t writes_before = framebuffer_writes;
    debug_ui_view_update(&view, &model, &diagnostics);
    lv_refr_now(NULL);
    lcd_st7789_service(fake_ms);
    assert(owned_pixels == NULL);
    return framebuffer_writes - writes_before;
}

/** @brief Require stable views to stay idle and one-row changes to remain sub-screen. */
static void verify_dirty_update_budget(void)
{
    uint32_t unchanged_pixels = render_update_pixels();
    uint32_t changed_pixels;
    assert(unchanged_pixels == 0U);
    model.snapshot.bench_profile = (uint8_t)!model.snapshot.bench_profile;
    changed_pixels = render_update_pixels();
    assert(changed_pixels > 0U && changed_pixels < 280U * 240U);
    model.snapshot.bench_profile = (uint8_t)!model.snapshot.bench_profile;
    printf("LVGL_DIRTY_UPDATE_OK unchanged=%lu changed=%lu full=%lu\n",
           (unsigned long)unchanged_pixels, (unsigned long)changed_pixels,
           (unsigned long)(280U * 240U));
}

/** @brief Reject chromatic static-page pixels while allowing RGB565 gray quantization. */
static void verify_static_neutral_palette(void)
{
    unsigned index;
    if (view.page.decoration < 4U) return;
    for (index = 0U; index < 280U * 240U; ++index) {
        lv_color32_t color;
        unsigned maximum, minimum;
        color.full = lv_color_to32(framebuffer[index]);
        maximum = color.ch.red > color.ch.green ? color.ch.red : color.ch.green;
        if (color.ch.blue > maximum) maximum = color.ch.blue;
        minimum = color.ch.red < color.ch.green ? color.ch.red : color.ch.green;
        if (color.ch.blue < minimum) minimum = color.ch.blue;
        if (maximum - minimum > 8U) {
            fprintf(stderr, "STATIC_PALETTE_CHROMATIC page=%u pixel=%u rgb=%u/%u/%u\n",
                (unsigned)model.page, index, color.ch.red, color.ch.green, color.ch.blue);
            exit(3);
        }
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
    verify_static_neutral_palette();
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
#include "astra_ui_scenarios.inc"

/** @brief Verify reference geometry, centered typography, marker bounds and draft-only hint. */
static void verify_parameter_popup_design(void)
{
    unsigned field, x;
    astra_scenario_draft(DEBUG_UI_OPERATION_MIT_MOVE, DEBUG_UI_PAGE_NUMBER);
    for (field=0U; field<5U; ++field) {
        unsigned white_pixels=0U;
        model.edit_field=(uint8_t)field;
        assert(debug_ui_graphics_run(render_graphics,NULL));
        assert(view.page.decoration == 3U);
        assert(lv_obj_get_style_text_align(view.page.rows[0],0)==LV_TEXT_ALIGN_CENTER);
        assert(lv_obj_get_style_text_font(view.page.rows[0],0)==&ui_font_popup_17);
        assert(lv_obj_get_style_text_font(view.page.rows[1],0)==&ui_font_popup_30);
        assert(lv_obj_get_style_text_font(view.page.rows[2],0)==&ui_font_popup_12);
        assert(strncmp(lv_label_get_text(view.page.rows[2]),"每步",6)==0);
        assert(strcmp(lv_label_get_text(view.page.rows[3]),"仅修改草稿")==0);
        assert(view.page.popup_marker>=39 && view.page.popup_marker<=241);
        assert(framebuffer[48U*280U+23U].full==lv_color_white().full);
        assert(framebuffer[48U*280U+18U].full==lv_color_black().full); /* Dithered parent remains outside the rounded corner. */
        assert(framebuffer[204U*280U+140U].full==lv_color_white().full);
        for (x=39U; x<=241U; ++x) assert(framebuffer[143U*280U+x].full==lv_color_white().full);
        for (x=30U; x<=250U; ++x) if (framebuffer[190U*280U+x].full==lv_color_white().full) ++white_pixels;
        assert(white_pixels<100U); /* The design has a small hint, never a full inverse button. */
    }
    model.edit_field=0U;
    model.draft.delta_rad=-model.snapshot.motors[6].profile.mit_max_delta_rad;
    assert(debug_ui_graphics_run(render_graphics,NULL)); assert(view.page.popup_marker==39);
    model.draft.delta_rad=-model.draft.delta_rad;
    assert(debug_ui_graphics_run(render_graphics,NULL)); assert(view.page.popup_marker==241);
    puts("PARAMETER_POPUP_DESIGN_OK rounded_247x157=1 centered_Noto=1 slider_endpoints=1 draft_hint=1");
}

/** @brief Reproduce real-profile speed edits through model keys and rendered labels/markers. */
static void verify_speed_popup_steps(void)
{
    static const DebugUiOperation operations[] = { DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_OPERATION_MIT_MOVE };
    static const float limits[] = { 0.5f, 1.0f, 5.0f, 10.0f, 11.0f, 11459.156f };
    unsigned operation_index, limit_index;
    DebugUiInputEvent event = {0};
    for (operation_index=0U; operation_index<2U; ++operation_index) {
        int previous_marker;
        char filename[48];
        astra_scenario_draft(operations[operation_index], DEBUG_UI_PAGE_NUMBER);
        model.input_valid=1U; model.edit_field=1U;
        model.snapshot.motors[6].profile.max_speed_rad_s=200.0f;
        model.snapshot.motors[6].profile.mit_max_speed_rad_s=200.0f;
        model.draft.speed_rad_s=debug_ui_degrees_to_radians(10.0f);
        assert(debug_ui_graphics_run(render_graphics,NULL));
        assert(strcmp(lv_label_get_text(view.page.rows[1]),"10.0°/s")==0);
        assert(view.page.popup_marker==140);
        (void)snprintf(filename,sizeof(filename),"speed_%s_10",operation_index==0U ? "pos" : "mit");
        save_page("Tests/ui/build/render",filename);
        event.type=DEBUG_UI_INPUT_EVENT_PRESS; event.key=DEBUG_UI_KEY_UP;
        debug_ui_model_event(&model,&event);
        assert(debug_ui_graphics_run(render_graphics,NULL));
        assert(strcmp(lv_label_get_text(view.page.rows[1]),"11.0°/s")==0);
        assert(view.page.popup_marker>=145);
        (void)snprintf(filename,sizeof(filename),"speed_%s_11",operation_index==0U ? "pos" : "mit");
        save_page("Tests/ui/build/render",filename);
        event.key=DEBUG_UI_KEY_DOWN;
        debug_ui_model_event(&model,&event);
        assert(debug_ui_graphics_run(render_graphics,NULL));
        assert(view.page.popup_marker==140);
        debug_ui_model_event(&model,&event);
        assert(debug_ui_graphics_run(render_graphics,NULL));
        assert(strcmp(lv_label_get_text(view.page.rows[1]),"9.0°/s")==0);
        assert(view.page.popup_marker<=135);
        (void)snprintf(filename,sizeof(filename),"speed_%s_09",operation_index==0U ? "pos" : "mit");
        save_page("Tests/ui/build/render",filename);
        assert(!model.outgoing_ready && !model.stop_ready);
        for (limit_index=0U; limit_index<sizeof(limits)/sizeof(limits[0]); ++limit_index) {
            unsigned sample;
            float maximum=limits[limit_index], minimum=maximum<1.0f ? maximum : 1.0f;
            model.snapshot.motors[6].profile.max_speed_rad_s=debug_ui_degrees_to_radians(maximum);
            model.snapshot.motors[6].profile.mit_max_speed_rad_s=debug_ui_degrees_to_radians(maximum);
            previous_marker=38;
            for (sample=0U; sample<=100U; ++sample) {
                model.draft.speed_rad_s=debug_ui_degrees_to_radians(minimum+(maximum-minimum)*(float)sample/100.0f);
                assert(debug_ui_graphics_run(render_graphics,NULL));
                assert(view.page.popup_marker>=previous_marker && view.page.popup_marker<=241);
                if (sample==0U) assert(view.page.popup_marker==(maximum==minimum ? 140 : 39));
                if (sample==100U) assert(view.page.popup_marker==(maximum==minimum ? 140 : 241));
                previous_marker=view.page.popup_marker;
            }
        }
    }
    puts("SPEED_POPUP_STEPS_OK pos_mit=1 default_center=1 keys_change_numbers=1 visible_step=1 bounded_monotonic=1 draft_only=1");
}

/** @brief Check visible unknown values and result destinations against actual state. */
static void verify_view_edge_states(void)
{
    unsigned index;
    static const float invalid_positions[] = { NAN, INFINITY, -INFINITY, FLT_MAX };
    simulated_data();
    model.page = DEBUG_UI_PAGE_DETAIL;
    model.snapshot.motors[0].feedback_valid = 0U;
    for (index = 0U; index < sizeof(invalid_positions)/sizeof(invalid_positions[0]); ++index) {
        model.snapshot.motors[0].position_rad = invalid_positions[index];
        assert(debug_ui_graphics_run(render_graphics, NULL));
        assert(strstr(lv_label_get_text(view.page.rows[1]), "--") != NULL);
    }
    simulated_data();
    model.page = DEBUG_UI_PAGE_FAULT; model.completion_seen = 0U;
    model.reason = DEBUG_UI_REASON_NOT_READY;
    assert(debug_ui_graphics_run(render_graphics, NULL));
    assert(strcmp(lv_label_get_text(view.page.rows[4]), "已读返回") == 0);
    model.completion_seen = 1U; model.completion.disabled_confirmed = 1U;
    model.snapshot.unconfirmed_disable_mask = 1U;
    assert(debug_ui_graphics_run(render_graphics, NULL));
    assert(strcmp(lv_label_get_text(view.page.rows[4]), "进入控制检查") == 0);
    model.page = DEBUG_UI_PAGE_PREPARE; model.snapshot.motion_enabled = 0U;
    assert(debug_ui_graphics_run(render_graphics, NULL));
    assert(strstr(view.page.footer_text, "STOP") == NULL);
    assert(strstr(lv_label_get_text(view.page.rows[6]), "停止") == NULL);
    puts("ASTRA_VIEW_EDGE_STATES_OK nonfinite_unknown=1 result_destination=1 readonly_no_stop_hint=1");
}

/** @brief Verify idle animation timing and immediate activity/fault cancellation. */
static void verify_idle_animation(void)
{
    uint32_t started;
    simulated_data();
    model.page = DEBUG_UI_PAGE_OVERVIEW; model.focus = 0U;
    debug_ui_view_update(&view, &model, &diagnostics);
    lv_tick_inc(160U); debug_ui_view_animate(&view);
    model.focus = 1U;
    debug_ui_view_update(&view, &model, &diagnostics);
    assert(view.page.animation_active);
    started = view.page.animation_start;
    lv_tick_inc(20U); debug_ui_view_animate(&view);
    debug_ui_view_update(&view, &model, &diagnostics);
    assert(view.page.animation_start == started);
    lv_tick_inc(120U); debug_ui_view_animate(&view);
    assert(!view.page.animation_active && view.page.tile_offset == 0);
    model.focus = 2U;
    debug_ui_view_update(&view, &model, &diagnostics);
    assert(view.page.animation_active);
    model.snapshot.active = 1U;
    debug_ui_view_update(&view, &model, &diagnostics);
    assert(!view.page.animation_active && view.page.tile_offset == 0);
    model.snapshot.active = 0U; model.input_valid = 0U; model.focus = 0U;
    debug_ui_view_update(&view, &model, &diagnostics);
    assert(!view.page.animation_active);
    model.input_valid = 1U; model.focus = 3U;
    debug_ui_view_update(&view, &model, &diagnostics);
    assert(view.page.tile_focus == 3U && view.page.animation_from == -28);
    lv_tick_inc(160U); debug_ui_view_animate(&view);
    model.focus = 0U;
    debug_ui_view_update(&view, &model, &diagnostics);
    assert(view.page.tile_focus == 0U && view.page.animation_from == 28);
    lv_tick_inc(160U); debug_ui_view_animate(&view);
    puts("ASTRA_IDLE_ANIMATION_OK duration=140ms cadence=20ms activity_and_input_cancel=1");
}

/** @brief Exercise repeated actual LVGL layouts and assert no retained page allocations. */
static void verify_shared_view_reuse(void)
{
    unsigned iteration;
    lv_mem_monitor_t warmed, final_memory;
    simulated_data();
    for (iteration = 0U; iteration < 1212U; ++iteration) {
        model.page = (DebugUiPage)(iteration % DEBUG_UI_PAGE_COUNT);
        assert(debug_ui_graphics_run(render_graphics, NULL));
        assert(lv_obj_get_child_cnt(view.page.root) == 18U);
        if (iteration == 2U * DEBUG_UI_PAGE_COUNT - 1U) lv_mem_monitor(&warmed);
    }
    lv_mem_monitor(&final_memory);
    assert(final_memory.free_size == warmed.free_size);
    printf("LVGL_SHARED_VIEW_REUSE_OK updates=1212 children=18 view_bytes=%lu sampled_pool_used_max=%lu\n",
        (unsigned long)sizeof(DebugUiView), (unsigned long)sampled_pool_used_max);
}

/** @brief Hash the first six static rows so a seventh-row update cannot erase them. */
static uint32_t static_rows_digest(void)
{
    unsigned x, y;
    uint32_t digest = 2166136261U;
    for (y = 37U; y < 180U; ++y)
        for (x = 10U; x < 270U; ++x) digest = (digest ^ framebuffer[y * 280U + x].full) * 16777619U;
    return digest;
}

/** @brief Render seven-axis static pages using only synthetic published evidence. */
static void render_static_load_scenarios(const char *directory)
{
    unsigned index;
    uint32_t stable_rows_digest;
    simulated_data();
    model.page = DEBUG_UI_PAGE_STATIC_LOAD;
    model.snapshot.static_load_enabled = 0U;
    save_page(directory, "90_static_default_locked");
    assert(strstr(view.page.title_text, "静载") != NULL);
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        char motor_name[8];
        (void)snprintf(motor_name, sizeof(motor_name), "M%02u", index + 1U);
        assert(strstr(lv_label_get_text(view.page.rows[index]), motor_name) != NULL);
        model.snapshot.motors[index].actual_mode = DEBUG_UI_MODE_MIT;
        model.snapshot.motors[index].feedback_valid = model.snapshot.motors[index].feedback_seen = 1U;
        model.snapshot.motors[index].fault_flags = 0U;
        model.snapshot.motors[index].reference_generation = index + 1U;
        model.snapshot.motors[index].mos_temperature_c = 25.0f + (float)index;
        model.snapshot.motors[index].rotor_temperature_c = 27.0f + (float)index;
    }
    model.snapshot.static_load_enabled = 1U;
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    model.page = DEBUG_UI_PAGE_STATIC_LOAD;
    save_page(directory, "91_static_seven_fresh");
    assert(strstr(view.page.row_text[6], "--") == NULL);
    model.snapshot.motors[4].feedback_age_ms = 150U;
    save_page(directory, "92_static_one_stale");
    assert(strstr(view.page.row_text[4], "--") != NULL);
    model.snapshot.motors[4].feedback_age_ms = 20U;
    model.page = DEBUG_UI_PAGE_STATIC_ACTIONS;
    model.focus = 3U;
    save_page(directory, "93_static_actions");
    model.page = DEBUG_UI_PAGE_EDIT; model.focus = 0U;
    save_page(directory, "94_static_duration");
    model.page = DEBUG_UI_PAGE_NUMBER; model.edit_field = 4U;
    save_page(directory, "95_static_duration_number");
    model.page = DEBUG_UI_PAGE_REVIEW; model.reviewed = model.draft;
    model.review_return_page = DEBUG_UI_PAGE_STATIC_ACTIONS;
    model.review_released = 1U; model.confirm_elapsed_ms = 650U;
    save_page(directory, "96_static_review");
    assert(strstr(view.page.row_text[0], "七轴") != NULL);
    model.page = DEBUG_UI_PAGE_RUNNING; model.submitted = model.draft;
    model.snapshot.active = model.snapshot.static_load.active = 1U;
    model.snapshot.active_motor_mask = 0x7FU;
    model.snapshot.static_load.elapsed_ms = 240U;
    model.snapshot.static_load.duration_ms = 500U;
    model.snapshot.static_load.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    model.snapshot.static_load.identity.epoch = model.snapshot.epoch;
    model.snapshot.static_load.identity.request_id = 12U;
    model.snapshot.static_load.operation = DEBUG_UI_OPERATION_STATIC_HOLD;
    model.snapshot.static_load.motor_mask = 0x7FU;
    model.submitted.identity = model.snapshot.static_load.identity;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        DebugUiStaticLoadReport *report = &model.snapshot.static_load;
        report->target_position_rad[index] = model.snapshot.motors[index].position_rad;
        report->peak_error_rad[index] = debug_ui_degrees_to_radians(0.1f + 0.02f * (float)index);
        report->peak_torque_nm[index] = 0.5f + (float)index * 0.1f;
        report->peak_mos_c[index] = 31.0f + (float)index;
        report->peak_rotor_c[index] = 32.0f + (float)index;
        report->samples[index] = 10U + index;
    }
    model.snapshot.static_load.valid_mask = 0x7FU;
    save_page(directory, "97_static_running");
    assert(strstr(view.page.footer_text, "STOP") != NULL);
    model.snapshot.active = model.snapshot.static_load.active = 0U;
    model.snapshot.static_load.complete = 1U;
    model.page = DEBUG_UI_PAGE_STATIC_RESULT;
    save_page(directory, "98_static_result_unconfirmed");
    assert(strstr(view.page.title_text, "待确认") != NULL);
    model.snapshot.static_load.disabled_confirmed = 1U;
    save_page(directory, "99_static_result_confirmed");
    assert(strstr(view.page.title_text, "已失能") != NULL);
    model.static_result_detail = 1U;
    save_page(directory, "100_static_result_temperatures");
    model.static_result_detail = 2U;
    save_page(directory, "101_static_result_samples");
    stable_rows_digest = static_rows_digest();
    model.snapshot.static_load.valid_mask = 0x3FU;
    model.snapshot.static_load.samples[6] = 0U;
    save_page(directory, "102_static_result_missing_axis");
    assert(static_rows_digest() == stable_rows_digest);
    assert(strstr(view.page.row_text[6], "--") != NULL);
    for (index = 0U; index < 20U; ++index) {
        model.snapshot.static_load.valid_mask = index % 2U ? 0x3FU : 0x7FU;
        model.snapshot.static_load.samples[6] = index % 2U ? 0U : 16U;
        assert(debug_ui_graphics_run(render_graphics, NULL));
        assert(static_rows_digest() == stable_rows_digest);
    }
    model.static_result_detail = 0U;
    model.snapshot.static_load.complete = 0U;
    model.snapshot.static_load.disabled_confirmed = 0U;
    model.completion_seen = 1U;
    model.completion.identity = model.snapshot.static_load.identity;
    model.completion.operation = DEBUG_UI_OPERATION_STATIC_HOLD;
    model.completion.motor_mask = 0x7FU;
    model.completion.code = DEBUG_UI_FAILED;
    model.completion.reason = DEBUG_UI_REASON_TORQUE_LIMIT;
    save_page(directory, "103_static_failed_partial");
    assert(strstr(view.page.title_text, "失败") != NULL && strstr(view.page.title_text, "待确认") != NULL);
    assert(strstr(view.page.row_text[0], "--") == NULL && strstr(view.page.row_text[6], "--") != NULL);
    assert(strstr(view.page.footer_text, "转矩超限") != NULL);
    model.completion.code = DEBUG_UI_STOPPED;
    model.completion.reason = DEBUG_UI_REASON_NONE;
    model.snapshot.static_load.disabled_confirmed = 1U;
    save_page(directory, "104_static_stopped_confirmed");
    assert(strstr(view.page.title_text, "已停止") != NULL && strstr(view.page.title_text, "已失能") != NULL);
    model.snapshot.static_load.stopped = 1U;
    model.snapshot.static_load.stop_identity = model.snapshot.static_load.identity;
    model.snapshot.static_load.stop_identity.request_id = 70U;
    model.completion.identity = model.snapshot.static_load.stop_identity;
    model.completion.operation = DEBUG_UI_OPERATION_STOP;
    model.completion.reason = DEBUG_UI_REASON_TEMPERATURE_LIMIT;
    save_page(directory, "107_static_bound_stop_identity");
    assert(strstr(view.page.title_text, "已停止") != NULL && strstr(view.page.title_text, "已失能") != NULL);
    assert(strstr(view.page.footer_text, "温度超限") != NULL);
    ++model.completion.identity.request_id;
    model.completion.reason = DEBUG_UI_REASON_SPEED_LIMIT;
    model.static_result_detail = 1U;
    save_page(directory, "108_static_unrelated_stop_reason_hidden");
    assert(strstr(view.page.footer_text, "速度超限") == NULL);
    model.snapshot.static_load.stopped = 0U;
    model.snapshot.static_load.disabled_confirmed = 0U;
    model.completion.identity = model.snapshot.static_load.stop_identity;
    model.completion.code = DEBUG_UI_FAILED;
    model.completion.reason = DEBUG_UI_REASON_STALE_FEEDBACK;
    save_page(directory, "111_static_bound_stop_failed");
    assert(strstr(view.page.title_text, "失败") != NULL && strstr(view.page.title_text, "待确认") != NULL);
    assert(strstr(view.page.footer_text, "反馈过期") != NULL);
    model.snapshot.static_load.stopped = 1U;
    model.snapshot.static_load.disabled_confirmed = 1U;
    model.snapshot.static_load.stop_identity.request_id = 0U;
    model.completion.identity = model.snapshot.static_load.identity;
    model.completion.operation = DEBUG_UI_OPERATION_STATIC_HOLD;
    model.completion.code = DEBUG_UI_CANCELLED;
    model.completion.reason = DEBUG_UI_REASON_NOT_READY;
    model.static_result_detail = 2U;
    save_page(directory, "109_static_fallback_stop_confirmed");
    assert(strstr(view.page.title_text, "已停止") != NULL && strstr(view.page.title_text, "已失能") != NULL);
    assert(strstr(view.page.footer_text, "尚未就绪") == NULL);
    model.snapshot.unconfirmed_disable_mask = 0x7FU;
    save_page(directory, "110_static_confirmed_control_locked");
    assert(strstr(view.page.title_text, "已停止") != NULL && strstr(view.page.title_text, "已失能") != NULL);
    assert(strstr(view.page.footer_text, "控制仍锁定") != NULL);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_STATIC_HOLD) == DEBUG_UI_REASON_NOT_DISABLED);
    model.snapshot.unconfirmed_disable_mask = 0U;
    model.static_result_detail = 0U;
    ++model.submitted.identity.request_id;
    model.completion.identity = model.submitted.identity;
    model.completion.code = DEBUG_UI_FAILED;
    model.completion.reason = DEBUG_UI_REASON_OLD_EPOCH;
    save_page(directory, "105_static_old_report_hidden");
    assert(strstr(view.page.title_text, "未开始") != NULL);
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        assert(strstr(view.page.row_text[index], "--") != NULL);
    model.static_result_retained = 1U;
    save_page(directory, "106_static_explicit_history");
    assert(strstr(view.page.row_text[0], "--") == NULL);
    puts("STATIC_LVGL_RENDER_OK seven_rows=7 default_locked=1 stale_hidden=1 results_peaks_temperature_samples=1 bound_stop_identity=1 stable_rows_20_updates=1");
}

/** @brief Render selected-only long static evidence and reject cross-mode or cross-ID history. */
static void render_single_static_load_scenarios(const char *directory)
{
    DebugUiStaticLoadReport *report;
    unsigned index;
    simulated_data();
    model.selected_motor = 2U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_LOAD;
    save_page(directory, "112_single_default_locked");
    assert(strstr(view.page.title_text, "单轴") != NULL);
    model.snapshot.motors[2].actual_mode = DEBUG_UI_MODE_MIT;
    model.snapshot.motors[2].reference_generation = 3U;
    model.snapshot.single_static_load_enabled = 1U;
    save_page(directory, "113_single_unconfigured");
    assert(strstr(view.page.row_text[0], "长时未配置") != NULL);
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        DebugUiMotorView *motor = &model.snapshot.motors[index];
        motor->actual_mode = DEBUG_UI_MODE_MIT;
        motor->feedback_valid = motor->feedback_seen = (uint8_t)(index == 2U);
        motor->fault_flags = 0U;
        motor->reference_generation = index + 1U;
        motor->mos_temperature_c = 27.0f;
        motor->rotor_temperature_c = 29.0f;
        motor->single_static_profile.configured = 1U;
        motor->single_static_profile.minimum_ms = 1000U;
        motor->single_static_profile.maximum_ms = 60000U;
    }
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_LOAD;
    save_page(directory, "114_single_monitor_id3");
    assert(strstr(view.page.row_text[1], "--") == NULL);
    model.page = DEBUG_UI_PAGE_STATIC_LOAD;
    save_page(directory, "128_single_lock_not_seven_preview");
    assert(strstr(view.page.row_text[2], "e--") != NULL);
    model.page = DEBUG_UI_PAGE_STATIC_ACTIONS; model.focus = 3U; model.list_first = 0U;
    model.snapshot.static_load_enabled = 1U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        model.snapshot.motors[index].feedback_valid = model.snapshot.motors[index].feedback_seen = 1U;
    save_page(directory, "130_single_lock_not_seven_actions");
    assert(strstr(lv_label_get_text(view.page.rows[1]), "未锁定") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[2]), " 0 ms") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[3]), "请先锁定姿态") != NULL);
    model.snapshot.static_load_enabled = 0U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        model.snapshot.motors[index].feedback_valid = model.snapshot.motors[index].feedback_seen = (uint8_t)(index == 2U);
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_LOAD;
    model.snapshot.motors[2].feedback_age_ms = 150U;
    save_page(directory, "115_single_stale_id3");
    assert(strstr(view.page.row_text[1], "--") != NULL);
    model.snapshot.motors[2].feedback_age_ms = 28U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS;
    model.focus = 4U; model.list_first = 2U;
    save_page(directory, "116_single_actions");
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_IDS;
    model.focus = 6U; model.list_first = 2U;
    save_page(directory, "117_single_select_id7");
    model.page = DEBUG_UI_PAGE_NUMBER; model.edit_field = 4U;
    model.draft.hold_duration_ms = 60000U;
    save_page(directory, "118_single_duration_60s");
    assert(strstr(view.page.title_text, "单轴静载") != NULL);
    assert(strstr(view.page.title_text, "03") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[2]), "1000") != NULL);
    model.page = DEBUG_UI_PAGE_REVIEW; model.reviewed = model.draft;
    model.review_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS;
    model.review_released = 1U; model.confirm_elapsed_ms = 650U;
    save_page(directory, "119_single_review");
    assert(strstr(view.page.row_text[0], "ID3") != NULL);
    model.page = DEBUG_UI_PAGE_RUNNING; model.submitted = model.draft;
    model.awaiting_result = 1U;
    model.snapshot.active = 1U; model.snapshot.active_motor_mask = 4U;
    report = &model.snapshot.static_load;
    report->operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    report->motor_mask = report->valid_mask = 4U;
    report->active = 1U; report->elapsed_ms = 24500U; report->duration_ms = 60000U;
    report->identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    report->identity.epoch = model.snapshot.epoch; report->identity.request_id = 15U;
    model.submitted.identity = report->identity;
    report->target_position_rad[2] = model.draft.static_positions_rad[2];
    report->peak_error_rad[2] = debug_ui_degrees_to_radians(0.12f);
    report->peak_torque_nm[2] = 0.8f;
    report->peak_mos_c[2] = 31.0f; report->peak_rotor_c[2] = 32.0f;
    report->samples[2] = 6130U;
    save_page(directory, "120_single_running");
    assert(strstr(view.page.footer_text, "STOP") != NULL);
    assert(strstr(view.page.row_text[4], "24500/60000") != NULL);
    model.stop_requested = 1U;
    save_page(directory, "121_single_stop_pending");
    model.stop_requested = model.awaiting_result = model.snapshot.active = report->active = 0U;
    report->complete = 1U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_RESULT;
    save_page(directory, "122_single_result_unconfirmed");
    report->disabled_confirmed = 1U;
    save_page(directory, "123_single_result_confirmed");
    assert(strstr(view.page.row_text[4], "6130") != NULL);
    report->operation = DEBUG_UI_OPERATION_STATIC_HOLD; report->motor_mask = 0x7FU;
    save_page(directory, "124_single_wrong_mode_hidden");
    assert(strstr(view.page.row_text[4], "6130") == NULL);
    report->operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD; report->motor_mask = 8U;
    report->valid_mask = 8U; report->samples[3] = 111U;
    save_page(directory, "125_single_wrong_id_hidden");
    assert(strstr(view.page.row_text[4], "111") == NULL);
    model.static_result_retained = 1U;
    save_page(directory, "126_single_explicit_id4_history");
    assert(strstr(view.page.row_text[0], "ID4") != NULL);
    assert(strstr(view.page.row_text[4], "111") != NULL);
    report->motor_mask = 3U;
    save_page(directory, "129_single_invalid_history_mask");
    assert(strstr(view.page.title_text, "无效范围") != NULL);
    assert(strstr(view.page.row_text[4], "111") == NULL);
    model.static_result_retained = 0U;
    report->motor_mask = report->valid_mask = 4U;
    report->samples[2] = 0U;
    model.snapshot.unconfirmed_disable_mask = 4U;
    save_page(directory, "127_single_missing_sample_control_locked");
    assert(strstr(view.page.row_text[4], "未观测") != NULL);
    assert(strstr(view.page.row_text[6], "锁定") != NULL);
    puts("SINGLE_STATIC_LVGL_RENDER_OK selected_only=1 duration_60s=1 result_scope_identity=1 explicit_history_id=1");
}

/** @brief Publish monotonic application clocks through the real model before dirty-render checks. */
static void accept_render_clock(uint32_t elapsed_ms)
{
    DebugUiSnapshot snapshot = model.snapshot;
    const DebugUiLoadTrialRecord *record;
    snapshot.timestamp_us += 1000U;
    snapshot.static_load.elapsed_ms = elapsed_ms;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    record = debug_ui_load_trials_newest(&model.load_trials, 0U);
    assert(record != NULL && record->report_active && record->elapsed_ms == elapsed_ms);
}

/** @brief Render the guided LCD lifecycle while preserving all historic scope assertions. */
static void render_static_launcher_scenarios(const char *directory)
{
    DebugUiStaticLoadReport *report;
    unsigned index;
    simulated_data();
    model.page = DEBUG_UI_PAGE_STATIC_HUB;
    model.focus = 0U;
    save_page(directory, "132_static_hub_single");
    assert(view.page.static_art.icons[0].scale==2U);
    assert(lv_color_to16(framebuffer[58*280+12])==lv_color_to16(lv_color_white()));
    assert(raster_ink_count(247,80,257,95,1U)>10U);
    assert(strstr(lv_label_get_text(view.page.rows[1]), "单轴") != NULL);
    model.focus = 1U;
    save_page(directory, "133_static_hub_seven");
    assert(lv_color_to16(framebuffer[108*280+12])==lv_color_to16(lv_color_white()));
    assert(raster_ink_count(247,130,257,145,1U)>10U);
    model.single_static_flow = 1U;
    model.single_static_seconds = 1U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP;
    model.focus = 2U;
    save_page(directory, "134_lcd_default_locked");
    assert(strstr(view.page.row_text[3], "锁止") != NULL);
    model.snapshot.single_static_load_enabled = 1U;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        DebugUiMotorView *motor = &model.snapshot.motors[index];
        motor->actual_mode = DEBUG_UI_MODE_MIT;
        motor->feedback_seen = motor->feedback_valid = 1U;
        motor->fault_flags = 0U;
        motor->mos_temperature_c = 27.0f;
        motor->rotor_temperature_c = 29.0f;
        model.selected_motor = (uint8_t)index;
        model.focus = 0U;
        {
            char name[64];
            (void)snprintf(name, sizeof(name), "135_lcd_setup_id%u", index + 1U);
            save_page(directory, name);
        }
        assert(strstr(view.page.row_text[3], "未配置") != NULL);
    }
    model.single_static_seconds = 60U; model.focus = 1U;
    save_page(directory, "136_lcd_setup_id7_60s");
    /* Four independent controls include the explicitly editable protocol feedforward. */
    assert(view.page.static_art.panel_count==4U);
    assert(strstr(lv_label_get_text(view.page.rows[3]), "+0.00") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[2]), "60") != NULL);
    model.focus = 3U; model.single_static_ff_centi = 105U;
    save_page(directory, "280_single_ff_positive");
    model.single_static_ff_negative = model.single_static_ff_editing = 1U;
    model.single_static_ff_digit = 3U;
    save_page(directory, "281_single_ff_negative_digit");
    assert(strstr(lv_label_get_text(view.page.rows[3]), "-1.0[5]") != NULL);
    model.single_static_ff_negative = model.single_static_ff_editing = 0U;
    model.single_static_ff_centi = 0U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW;
    memset(&model.reviewed, 0, sizeof(model.reviewed));
    model.reviewed.target_motor_id = 7U; model.reviewed.hold_duration_ms = 60000U;
    model.review_released = 1U; model.confirm_elapsed_ms = 650U;
    /* Render from the frozen request even when an unrelated draft value changes. */
    model.selected_motor = 0U; model.single_static_seconds = 1U;
    save_page(directory, "137_lcd_config_review");
    assert(view.page.static_art.icons[0].kind==2U);
    assert(view.page.layout_confirm_visible && lv_obj_get_width(view.page.confirm_bar)>0);
    assert(strstr(view.page.row_text[0], "ID7") != NULL);
    assert(strstr(view.page.row_text[1], "60") != NULL);
    assert(strstr(view.page.row_text[3], "单次失能预检") != NULL);
    model.awaiting_result = 1U; model.confirm_elapsed_ms = 0U;
    save_page(directory, "138_lcd_config_pending");
    model.awaiting_result = 0U; model.single_static_wait_snapshot = 1U;
    save_page(directory, "139_lcd_config_snapshot_pending");
    assert(strstr(view.page.row_text[5], "快照") != NULL);
    model.single_static_wait_snapshot = 0U;
    model.selected_motor = 6U; model.single_static_seconds = 60U;
    model.reason = DEBUG_UI_REASON_STALE_FEEDBACK;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP;
    model.focus = 2U;
    save_page(directory, "140_lcd_config_failure");
    model.reason = DEBUG_UI_REASON_NONE;
    model.single_static_configured = 1U;
    model.snapshot.motors[6].single_static_profile.configured = 1U;
    model.snapshot.motors[6].single_static_profile.minimum_ms = 1000U;
    model.snapshot.motors[6].single_static_profile.maximum_ms = 60000U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_READY;
    save_page(directory, "141_lcd_ready_remote");
    model.page = DEBUG_UI_PAGE_REVIEW;
    model.reviewed.operation = DEBUG_UI_OPERATION_ACQUIRE;
    model.review_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_READY;
    model.confirm_elapsed_ms = 300U;
    save_page(directory, "152_lcd_acquire_review");
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_READY;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    model.snapshot.target_motor_id=7U;
    model.snapshot.motors[6].reference_generation=1U;
    save_page(directory, "142_lcd_ready_capture");
    model.static_locked = 1U;
    model.draft.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    model.draft.target_motor_id = 7U;
    model.draft.hold_duration_ms = 60000U;
    model.draft.static_positions_rad[6] = model.snapshot.motors[6].position_rad;
    save_page(directory, "143_lcd_ready_locked");
    model.page = DEBUG_UI_PAGE_REVIEW;
    model.reviewed = model.draft;
    model.review_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_READY;
    model.confirm_elapsed_ms = 400U;
    save_page(directory, "144_lcd_hold_review");
    assert(view.page.static_art.icons[0].kind==4U);
    assert(raster_ink_count(31,47,68,87,0U)>50U);
    model.page = DEBUG_UI_PAGE_RUNNING;
    model.submitted = model.draft;
    model.submitted.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    model.submitted.identity.epoch = 3U; model.submitted.identity.request_id = 99U;
    model.submitted.created_at_us = model.now_us;
    assert(debug_ui_load_trials_freeze(&model.load_trials, model.submitted.target_motor_id));
    assert(debug_ui_load_trials_submit(&model.load_trials, &model.submitted));
    model.awaiting_result = model.snapshot.active = 1U;
    model.snapshot.active_motor_mask = 64U;
    report = &model.snapshot.static_load;
    report->operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    report->identity = model.submitted.identity;
    report->active = 1U; report->motor_mask = report->valid_mask = 64U;
    report->elapsed_ms = 24500U; report->duration_ms = 60000U;
    report->started_at_us = model.submitted.created_at_us;
    report->target_position_rad[6] = model.draft.static_positions_rad[6];
    report->peak_error_rad[6] = debug_ui_degrees_to_radians(0.12f);
    report->peak_torque_nm[6] = 0.8f;
    report->peak_mos_c[6] = 31.0f; report->peak_rotor_c[6] = 32.0f;
    report->samples[6] = 6130U;
    /* Equal tenths/segments remain idle at the obsolete 248-pixel marker boundary. */
    accept_render_clock(24193U);
    assert(debug_ui_graphics_run(render_graphics,NULL));
    accept_render_clock(24194U);
    if (render_update_pixels()!=0U) {
        puts("STATIC_PROGRESS_UNUSED_MARKER_RED"); ++static_status_regression_failures;
    }
    /* Accepted clock ticks retain partial redraws without ever sending time backwards. */
    assert(render_update_pixels() == 0U);
    accept_render_clock(24195U);
    assert(render_update_pixels() == 0U);
    accept_render_clock(24295U);
    {
        uint32_t changed_pixels = render_update_pixels();
        assert(changed_pixels > 0U && changed_pixels < 280U * 240U);
    }
    accept_render_clock(24500U);
    save_page(directory, "145_lcd_running");
    assert(view.page.static_art.progress_steps==9U);
    {
        float old_position=model.snapshot.motors[6].position_rad;
        model.snapshot.motors[6].position_rad=report->target_position_rad[6]+debug_ui_degrees_to_radians(0.12f);
        save_page(directory,"159_lcd_running_small_error");
        assert(strstr(lv_label_get_text(view.page.rows[2]),"+0.120")!=NULL);
        model.snapshot.motors[6].position_rad=old_position;
        assert(debug_ui_graphics_run(render_graphics,NULL));
    }
    assert(lv_obj_get_x(view.page.rows[2])<lv_obj_get_x(view.page.rows[3]));
    assert(strstr(lv_label_get_text(view.page.rows[6]),"控制")!=NULL);
    assert(lv_obj_get_style_text_font(view.page.rows[1], 0) == &ui_font_popup_30);
    assert(view.page.popup_marker > 0 && view.page.popup_marker < 248);
    report->takeover_active = 1U; report->takeover_elapsed_ms = 500U;
    report->target_torque_ff_nm = -1.05f; report->requested_torque_ff_nm = -0.55f;
    report->transmitted_torque_ff_nm = -0.50f;
    save_page(directory, "282_single_ff_takeover");
    assert(view.page.decoration == 4U); /* Ordinary HOLD captions must not overpaint takeover labels. */
    assert(strstr(lv_label_get_text(view.page.rows[0]), "前馈接入") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[2]), "-1.05") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[3]), "-0.50") != NULL);
    report->takeover_active = 0U;
    model.snapshot.motors[6].feedback_valid = 0U;
    save_page(directory, "146_lcd_running_unknown");
    assert(strstr(lv_label_get_text(view.page.rows[2]), "--") != NULL);
    model.snapshot.motors[6].feedback_valid = 1U;
    model.stop_requested = model.snapshot.stop_pending = 1U;
    save_page(directory, "147_lcd_stop_pending");
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_RESULT;
    model.stop_requested = model.snapshot.stop_pending = 0U;
    model.awaiting_result = model.snapshot.active = report->active = 0U;
    report->complete = 1U;
    save_page(directory, "148_lcd_result_unconfirmed");
    report->disabled_confirmed = 1U;
    save_page(directory, "149_lcd_result_confirmed");
    assert(view.page.static_art.icons[0].kind==5U);
    assert(strstr(lv_label_get_text(view.page.rows[4]),"6130")!=NULL);
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK;
    model.awaiting_result = 1U;
    save_page(directory, "150_lcd_relock_pending");
    model.awaiting_result = 0U; model.reason = DEBUG_UI_REASON_NOT_DISABLED;
    save_page(directory, "151_lcd_relock_failure");
    assert(view.page.static_art.icons[0].kind==11U);
    model.page = DEBUG_UI_PAGE_FAULT;
    model.completion_seen = 1U; model.completion.code = DEBUG_UI_FAILED;
    model.completion.stage = DEBUG_UI_STAGE_DISABLE;
    save_page(directory, "153_lcd_cleanup_failure");
    assert(strstr(lv_label_get_text(view.page.rows[6]), "锁止") != NULL);
    assert(strstr(view.page.footer_text, "中准备") != NULL && strstr(view.page.footer_text, "左退出") != NULL);
    assert(view.page.decoration == 4U);
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK;
    model.selected_motor = 6U; model.single_static_seconds = 60U;
    model.single_static_relock_motor_id = 1U;
    model.awaiting_result = 1U; model.reason = DEBUG_UI_REASON_NONE;
    save_page(directory, "154_lcd_old_target_handoff");
    assert(strstr(view.page.row_text[0], "ID1") != NULL);
    assert(strstr(view.page.row_text[1], "ID7") != NULL);
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_PREFLIGHT;
    model.single_static_seconds = 1U;
    model.submitted.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT;
    model.submitted.target_motor_id = 7U;
    model.single_static_preflight = 1U; model.awaiting_result = 1U;
    save_page(directory, "178_lcd_preflight_pending");
    assert(strstr(view.page.row_text[1], "一次 DISABLE") != NULL);
    assert(strstr(view.page.row_text[2], "输出后") != NULL);
    assert(strstr(view.page.footer_text, "取消") != NULL);
    model.awaiting_result = 0U;
    save_page(directory, "179_lcd_preflight_snapshot");
    assert(strstr(view.page.row_text[5], "快照") != NULL);
    model.single_static_preflight = 0U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK; model.reason = DEBUG_UI_REASON_STALE_FEEDBACK;
    save_page(directory, "180_lcd_preflight_failed");
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP; model.reason = DEBUG_UI_REASON_NOT_READY;
    model.snapshot.motors[6].actual_mode = DEBUG_UI_MODE_POS_VEL;
    model.snapshot.motors[6].mode_verified = 1U;
    save_page(directory, "181_lcd_mode2_next_step");
    assert(strstr(view.page.row_text[6], "切换 MIT") != NULL);
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK; model.reason = DEBUG_UI_REASON_STALE_FEEDBACK;
    model.single_static_flow = model.single_static_lock_then_configure = 1U;
    model.single_static_configured = model.single_static_preflight = model.single_static_wait_snapshot = 0U;
    model.awaiting_result = model.outgoing_ready = model.stop_ready = model.stop_waiting = model.stop_requested = 0U;
    model.stop_latch_state = DEBUG_UI_STOP_LATCH_NONE;
    model.snapshot.active = model.snapshot.pending = model.snapshot.stop_pending = model.snapshot.unconfirmed_disable_mask = 0U;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; model.snapshot.target_motor_id = 7U;
    model.single_static_relock_motor_id = 7U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) model.snapshot.motors[index].single_static_lcd_owned = 0U;
    save_page(directory, "182_lcd_handoff_cancel_available");
    assert(strstr(view.page.row_text[6], "左 返回") != NULL && strstr(view.page.footer_text, "取消返回") != NULL);
    model.snapshot.unconfirmed_disable_mask = 0x40U;
    save_page(directory, "183_lcd_handoff_cleanup_locked");
    assert(strstr(view.page.row_text[6], "左 返回") == NULL && strstr(view.page.footer_text, "取消返回") == NULL);
    puts("STATIC_MONO_ICON_RENDER_OK neutral_565_spread8=1 inverse_focus=1 icon_diagonals=1 bounded_paths=1 timer_segments=1 fixed_objects18=1");
    puts("STATIC_LCD_LVGL_RENDER_OK home_four=1 hub=1 id1_to_7=1 seconds60=1 independent_reviews=1 snapshot_wait=1 unknown_values=1 relock=1 idle_redraw=0 progress_partial=1");
}

/** @brief Reject misleading completed check marks and active wording after a rejected relock. */
static void verify_static_status_truth(const char *directory)
{
    DebugUiMotorView *motor;
    unsigned failures=static_status_regression_failures;
    simulated_data();
    model.page=DEBUG_UI_PAGE_SINGLE_STATIC_READY;
    model.single_static_flow=model.single_static_configured=1U;
    model.single_static_seconds=60U;
    model.selected_motor=0U;
    model.snapshot.single_static_load_enabled=1U;
    model.snapshot.target_motor_id=1U;
    motor=&model.snapshot.motors[0];
    motor->actual_mode=DEBUG_UI_MODE_MIT;
    motor->reference_generation=1U;
    motor->single_static_profile.configured=1U;
    motor->single_static_profile.minimum_ms=1000U;
    motor->single_static_profile.maximum_ms=60000U;
    motor->feedback_valid=0U;
    save_page(directory,"155_lcd_ready_stale_feedback");
    if (view.page.static_art.icons[1].kind!=6U) {
        puts("STATIC_READY_STALE_FALSE_CHECK_RED"); ++failures;
    }
    motor->feedback_valid=1U; motor->enabled=1U; motor->driver_state=1U;
    save_page(directory,"156_lcd_ready_enabled");
    if (view.page.static_art.icons[1].kind!=6U) {
        puts("STATIC_READY_ENABLED_FALSE_CHECK_RED"); ++failures;
    }
    motor->enabled=0U; motor->driver_state=0U; motor->actual_mode=DEBUG_UI_MODE_POS_VEL;
    save_page(directory,"157_lcd_ready_wrong_mode");
    if (view.page.static_art.icons[1].kind!=6U) {
        puts("STATIC_READY_MODE_FALSE_CHECK_RED"); ++failures;
    }
    motor->actual_mode=DEBUG_UI_MODE_MIT; motor->mode_verified=0U;
    save_page(directory,"160_lcd_ready_unverified_mode");
    if (view.page.static_art.icons[1].kind!=6U) {
        puts("STATIC_READY_UNVERIFIED_MODE_FALSE_CHECK_RED"); ++failures;
    }
    motor->mode_verified=1U;
    model.selected_motor=6U;
    motor=&model.snapshot.motors[6];
    motor->actual_mode=DEBUG_UI_MODE_MIT; motor->mode_verified=1U;
    motor->feedback_seen=motor->feedback_valid=1U;
    motor->reference_generation=1U;
    motor->single_static_profile.configured=1U;
    motor->single_static_profile.minimum_ms=1000U; motor->single_static_profile.maximum_ms=60000U;
    save_page(directory,"161_lcd_ready_other_target");
    if (view.page.static_art.icons[1].kind!=6U) {
        puts("STATIC_READY_OTHER_TARGET_FALSE_CHECK_RED"); ++failures;
    }
    assert(strstr(view.page.row_text[2],"本地ID1")!=NULL);
    model.selected_motor=0U;
    save_page(directory,"162_lcd_ready_checks_complete");
    assert(view.page.static_art.icons[1].kind==5U);
    model.page=DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK;
    model.reason=DEBUG_UI_REASON_NOT_DISABLED;
    save_page(directory,"158_lcd_relock_retry");
    if (strstr(view.page.row_text[1],"等待重试")==NULL) {
        puts("STATIC_RELOCK_RETRY_ACTIVE_WORDING_RED"); ++failures;
    }
    assert(failures==0U);
    puts("STATIC_STATUS_TRUTH_OK stale_enabled_mode_checks_empty=1 rejected_relock_retry=1");
}

#include "load_trial_render_cases.inc"
#include "load_trial_review_render_cases.inc"
#include "pos_load_render_cases.inc"
#include "mono_render_cases.inc"

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
    render_mono_page_families(argv[1]);
    simulated_data();
    objects_before = lv_obj_get_child_cnt(view.page.root);
    save_page(argv[1], "01_overview");
    verify_dirty_update_budget();
    /* A dedicated fourth launcher must remain distinguishable after settling. */
    model.focus = 3U;
    save_page(argv[1], "131_home_static_launcher");
    assert(view.page.tile_focus == 3U);
    assert(strcmp(lv_label_get_text(view.page.rows[1]), "静载测试") == 0);
    model.focus = 0U;
    model.page = DEBUG_UI_PAGE_MOTORS; model.selected_motor = model.focus = 6U; model.list_first = 2U;
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
    render_astra_scenarios(argv[1]);
    render_static_load_scenarios(argv[1]);
    render_single_static_load_scenarios(argv[1]);
    render_static_launcher_scenarios(argv[1]);
    verify_static_status_truth(argv[1]);
    render_load_trial_pages(argv[1]);
    render_load_trial_review_regressions(argv[1]);
    render_load_trial_timer_order(argv[1]);
    render_pos_load_scenarios(argv[1]);
    verify_parameter_popup_design();
    verify_speed_popup_steps();
    verify_view_edge_states();
    verify_idle_animation();
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
