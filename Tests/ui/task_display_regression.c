/** @file task_display_probe.c
 * @brief Persistent regressions derived from the independent UI review task/display probe.
 * HAL/RTOS/ADC/App endpoints are deterministic host fakes; no physical IO exists.
 */
#define main baseline_render_main
#define DebugUiNowMs smoke_now_ms
#define DebugUiWaitService smoke_wait_service
#define lcd_st7789_begin_flush smoke_begin_flush
#define lcd_st7789_service smoke_display_service
#define lcd_st7789_get_status smoke_display_status
#include "../../Tests/ui/ui_render_smoke.c"
#undef main
#undef DebugUiNowMs
#undef DebugUiWaitService
#undef lcd_st7789_begin_flush
#undef lcd_st7789_service
#undef lcd_st7789_get_status
#include "../../Ui/debug_ui_task.c"
#include "debug_ui_mailbox.h"

/** @brief Fail deterministically in the console instead of opening a CRT assertion dialog. */
static void regression_check(int condition, const char *expression, unsigned line)
{
    if (!condition) { fprintf(stderr, "REGRESSION_FAIL line=%u %s\n", line, expression); exit(1); }
}
#undef assert
#define assert(condition) regression_check((condition) != 0, #condition, __LINE__)

static uint8_t retained_copy[13440U];
static uint32_t retained_bytes, release_at_ms, stop_submissions, notification_count;
static uint32_t immutable_checks, stop_while_owned, health_publications;
static uint16_t adc_value = 4095U;
static const void *seen_buffers[2];
static DebugUiSnapshot app_snapshot;
static DebugUiHealth published_health;
static uint32_t historical_spi_errors;
static DebugUiReason submission_reason;

/** @brief Advance host scheduler time without sleeping or calling hardware. */
int osDelay(uint32_t milliseconds) { fake_ms += milliseconds; return 0; }
/** @brief Match the production HAL tick domain with the deterministic host clock. */
uint32_t HAL_GetTick(void) { return fake_ms; }
/** @brief Return synthetic stack space; excluded from real stack validation. */
uint32_t uxTaskGetStackHighWaterMark(void *task) { (void)task; return 1200U; }
/** @brief Record that UI wakes the protocol owner. */
void AethorNotifyProtocolTask(void) { ++notification_count; }
/** @brief Convert the host clock into the App timestamp domain. */
uint64_t AethorUiTimestampUs(void) { return (uint64_t)fake_ms * 1000ULL; }
/** @brief No real control timing is available from this fixture. */
void AethorGetControlTiming(uint32_t *execution, uint32_t *period) { *execution = 0U; *period = 0U; }
/** @brief Accept fake ADC initialization. */
int lcd_key_adc_init(void) { return 1; }
/** @brief A fake conversion is always available without HAL work. */
void lcd_key_adc_service(uint32_t now_ms) { (void)now_ms; }
/** @brief Supply ordered raw ADC values to the unmodified production input parser. */
void lcd_key_adc_latest(uint32_t now_ms, LcdKeyAdcSample *sample)
{
    sample->raw = adc_value; sample->seq = now_ms + 1U; sample->ms = now_ms; sample->valid = 1U;
}
/** @brief Publish a fresh value-copy snapshot; all motor values are simulated. */
bool aethor_app_debug_ui_get_snapshot(uint64_t now_us, DebugUiSnapshot *snapshot)
{
    *snapshot = app_snapshot;
    snapshot->timestamp_us = now_us;
    snapshot->health.ui_timestamp_us = snapshot->health.protocol_timestamp_us = now_us;
    return true;
}
/** @brief Observe actual UiTask dispatch and buffer ownership at STOP submission. */
DebugUiReason aethor_app_debug_ui_submit(const DebugUiRequest *request)
{
    assert(request->operation == DEBUG_UI_OPERATION_STOP);
    ++stop_submissions;
    if (owned_pixels != NULL) ++stop_while_owned;
    return submission_reason;
}
/** @brief Keep admission unavailable while testing asynchronous display progress. */
bool aethor_app_debug_ui_poll_admission(DebugUiAdmission *admission) { (void)admission; return false; }
/** @brief Keep terminal unavailable so a repeated hold cannot cause duplicate STOP. */
bool aethor_app_debug_ui_poll_completion(DebugUiCompletion *completion) { (void)completion; return false; }
/** @brief Capture production UI health during DMA wait and graphics failure. */
void aethor_app_debug_ui_update_health(const DebugUiHealth *health)
{
    published_health = *health; ++health_publications;
}
/** @brief Retain and checksum the exact live production DMA buffer for 60 simulated ms. */
int lcd_st7789_begin_flush(const LcdTransferWindow *window, const void *buffer,
                          uint32_t bytes, uint32_t now_ms)
{
    int accepted = smoke_begin_flush(window, buffer, bytes, now_ms);
    assert(bytes <= sizeof(retained_copy));
    retained_bytes = bytes;
    memcpy(retained_copy, buffer, bytes);
    release_at_ms = now_ms + 60U;
    if (seen_buffers[0] == NULL) seen_buffers[0] = buffer;
    else if (seen_buffers[0] != buffer) {
        if (seen_buffers[1] == NULL) seen_buffers[1] = buffer;
        assert(seen_buffers[1] == buffer);
    }
    return accepted;
}
/** @brief Assert no retained pixels change before completion; release only at fake EOT. */
void lcd_st7789_service(uint32_t now_ms)
{
    if (owned_pixels == NULL) return;
    assert(memcmp(retained_copy, owned_pixels, retained_bytes) == 0);
    ++immutable_checks;
    if (now_ms >= release_at_ms) smoke_display_service(now_ms);
}
/** @brief Model the documented preservation of historical platform counters after recovery. */
void lcd_st7789_get_status(LcdSt7789Status *status)
{
    smoke_display_status(status);
    status->spi_errors = historical_spi_errors;
}
/** @brief Render with continuous host time so the input fixture has no artificial 100ms gap. */
static void render_continuously(void *context)
{
    (void)context;
    debug_ui_view_update(&view, &model, &diagnostics);
    lv_tick_inc(100U);
    lv_refr_now(NULL);
}
/** @brief Show whether recovered READY plus historical errors can regain App health. */
static void probe_recovered_health(void)
{
    DebugUiMailbox mailbox;
    uint32_t until_ms;
    app_snapshot.active = 0U;
    debug_ui_model_init(&task_model);
    adc_value = 4095U;
    until_ms = fake_ms + 300U;
    while (fake_ms < until_ms) DebugUiWaitService();
    historical_spi_errors = 1U;
    until_ms = fake_ms + 60U;
    while (fake_ms < until_ms) DebugUiWaitService();
    assert(published_health.input_valid && published_health.display_valid && published_health.ui_valid);
    debug_ui_mailbox_init(&mailbox);
    debug_ui_mailbox_update_health(&mailbox, &published_health);
    debug_ui_mailbox_service_health(&mailbox, AethorUiTimestampUs());
    printf("RECOVERED_DISPLAY_HEALTH ready=%u input=%u ui=%u display_error=%lu App_healthy=%u protocol_valid=%u\n",
        published_health.display_valid, published_health.input_valid, published_health.ui_valid,
        (unsigned long)published_health.display_error,
        (unsigned)debug_ui_mailbox_healthy(&mailbox, AethorUiTimestampUs()), mailbox.health.protocol_valid);
    assert(published_health.display_error == 0U && debug_ui_mailbox_healthy(&mailbox, AethorUiTimestampUs()));
    assert(task_diagnostics.spi_errors == 1U); /* Recovery must not erase diagnostics. */
    historical_spi_errors = 0U;
}
/** @brief Trigger the production graphics assertion with a live owned DMA transfer. */
static void assert_with_owned_buffer(void *context)
{
    lv_disp_drv_t *driver = lv_disp_get_default()->driver;
    lv_area_t area = { 0, 0, 9, 0 };
    (void)context;
    driver->draw_buf->flushing = 1U;
    driver->flush_cb(driver, &area, driver->draw_buf->buf1);
    assert(owned_pixels != NULL);
    debug_ui_lvgl_assert_failed();
    assert(0 && "graphics assertion must leave the local boundary");
}
/** @brief Render mode confirmations and prove whether their requested modes are visible. */
static void probe_mode_confirmation(const char *directory)
{
    char pos_rows[DEBUG_UI_VIEW_ROWS][DEBUG_UI_VIEW_TEXT_BYTES];
    simulated_data();
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SET_MODE, DEBUG_UI_MODE_POS_VEL));
    model.reviewed = model.draft; model.page = DEBUG_UI_PAGE_REVIEW;
    save_page(directory, "review_set_pos");
    memcpy(pos_rows, view.page.row_text, sizeof(pos_rows));
    model.page = DEBUG_UI_PAGE_DETAIL;
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SET_MODE, DEBUG_UI_MODE_MIT));
    model.reviewed = model.draft; model.page = DEBUG_UI_PAGE_REVIEW;
    lv_obj_invalidate(view.page.root);
    save_page(directory, "review_set_mit");
    assert(memcmp(pos_rows, view.page.row_text, sizeof(pos_rows)) != 0);
    assert(strstr(pos_rows[1], "POS") != NULL);
    assert(strstr(view.page.row_text[1], "MIT") != NULL);
    assert(strstr(view.page.row_text[1], "POS -> MIT") != NULL);
    assert(strstr(view.page.row_text[3], "预计") == NULL);
    puts("SET_MODE_CONFIRMATION_DIFFERENT_PASS");
}
/** @brief Follow synchronous latch return through production dispatch and snapshot service. */
static void test_task_stop_latched(void)
{
    DebugUiInputEvent event;
    debug_ui_model_init(&task_model);
    app_snapshot.active = 1U; app_snapshot.active_motor_mask = 4U;
    app_snapshot.target_motor_id = 0U; app_snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    app_snapshot.motors[2].enabled = 1U;
    stop_submissions = 0U;
    submission_reason = DEBUG_UI_REASON_STOP_LATCHED;
    adc_value = 4095U;
    service_control_io();
    event.type = DEBUG_UI_INPUT_EVENT_PRESS; event.key = DEBUG_UI_KEY_CENTER;
    event.timestamp_ms = fake_ms; event.held_ms = 0U;
    debug_ui_model_event(&task_model, &event);
    dispatch_requests();
    assert(stop_submissions == 1U && !task_model.stop_waiting && !task_model.awaiting_result);
    assert(task_model.stop_latch_state == DEBUG_UI_STOP_LATCH_WAITING);
    fake_ms += 60U;
    app_snapshot.active = 0U; app_snapshot.active_motor_mask = 0U;
    app_snapshot.motors[2].enabled = 0U; app_snapshot.motors[2].feedback_age_ms = 1U;
    service_control_io();
    assert(task_model.page == DEBUG_UI_PAGE_RESULT && !task_model.completion_seen);
    assert(task_model.stop_latch_state == DEBUG_UI_STOP_LATCH_CONFIRMED && stop_submissions == 1U);
    submission_reason = DEBUG_UI_REASON_NONE;
    app_snapshot.target_motor_id = 3U;
    puts("TASK_STOP_LATCHED_PASS dispatch_snapshot_no_async_result_no_duplicate");
}

/** @brief Keep discovered parameters distinct from live and historical feedback. */
static void test_feedback_status_labels(void)
{
    DebugUiModel status_model;
    DebugUiMotorView *motor;
    debug_ui_model_init(&status_model);
    status_model.snapshot.bench_profile = 1U;
    status_model.snapshot.usb_connected = 1U;
    motor = &status_model.snapshot.motors[6];
    motor->identity_verified = 1U;
    motor->feedback_seen = 1U;
    motor->enabled = 1U; /* Old enabled sample must not imply current activity. */
    motor->actual_mode = DEBUG_UI_MODE_POS_VEL;
    motor->feedback_age_ms = 143865U;
    debug_ui_view_update(&view, &status_model, &diagnostics);
    assert(strstr(view.page.row_text[0], "已发现 1/7") != NULL);
    assert(strstr(view.page.row_text[1], "反馈有效 0/7") != NULL);
    status_model.page = DEBUG_UI_PAGE_MOTORS;
    status_model.list_first = 2U;
    status_model.selected_motor = 6U;
    debug_ui_view_update(&view, &status_model, &diagnostics);
    assert(strstr(view.page.row_text[4], "过期") != NULL);
    assert(strstr(view.page.row_text[4], "使能") == NULL);
    motor->fault_flags = 6U;
    debug_ui_view_update(&view, &status_model, &diagnostics);
    assert(strstr(view.page.row_text[4], "故障过期") != NULL);
    status_model.page = DEBUG_UI_PAGE_DETAIL;
    debug_ui_view_update(&view, &status_model, &diagnostics);
    assert(strstr(view.page.row_text[5], "故障 --") != NULL);
    motor->feedback_valid = 1U;
    status_model.page = DEBUG_UI_PAGE_OVERVIEW;
    status_model.snapshot.arm_fault = 1U;
    debug_ui_view_update(&view, &status_model, &diagnostics);
    assert(strstr(view.page.row_text[1], "反馈有效 1/7") != NULL);
    assert(strstr(view.page.row_text[2], "系统故障") != NULL);
    puts("FEEDBACK_STATUS_LABELS_PASS discovered_fresh_stale_fault");
}

/** @brief Reproduce observed physical directions relative to the landscape text. */
static void test_landscape_key_navigation(void)
{
    DebugUiModel navigation;
    DebugUiInputEvent event;
    debug_ui_model_init(&navigation);
    navigation.input_valid = 1U; /* Model normally receives ADC health from UiTask. */
    memset(&event, 0, sizeof(event));
    event.type = DEBUG_UI_INPUT_EVENT_PRESS;
    /* Physical down was decoded as LEFT; it must select diagnostics. */
    event.key = DEBUG_UI_KEY_LEFT;
    lv_port_indev_event(&navigation, &event);
    assert(navigation.focus == 1U);
    /* Physical right was decoded as DOWN; it must enter diagnostics. */
    event.key = DEBUG_UI_KEY_DOWN;
    lv_port_indev_event(&navigation, &event);
    assert(navigation.page == DEBUG_UI_PAGE_DIAGNOSTIC_MENU);
    event.key = DEBUG_UI_KEY_UP;
    lv_port_indev_event(&navigation, &event);
    assert(navigation.page == DEBUG_UI_PAGE_OVERVIEW);
    event.key = DEBUG_UI_KEY_RIGHT;
    lv_port_indev_event(&navigation, &event);
    assert(navigation.focus == 0U);
    event.key = DEBUG_UI_KEY_CENTER;
    lv_port_indev_event(&navigation, &event);
    assert(navigation.page == DEBUG_UI_PAGE_MOTORS);
    puts("LANDSCAPE_KEY_NAVIGATION_PASS down_select_right_enter_left_back_up_select_center_enter");
}

/** @brief Verify real wait->UiTask->input->model->submit and post-assert cleanup paths. */
int main(int argc, char **argv)
{
    uint32_t prime_ms, previous_health_count;
    assert(argc == 3);
    assert(debug_ui_graphics_run(initialize_graphics, NULL));
    test_landscape_key_navigation();
    test_feedback_status_labels();
    simulated_data();
    app_snapshot = model.snapshot;
    app_snapshot.active = 1U; app_snapshot.target_motor_id = 3U;
    app_snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    debug_ui_model_init(&task_model);
    debug_ui_input_init(&task_input, 0U);
    for (prime_ms = 0U; prime_ms <= 300U; prime_ms += 5U)
        (void)debug_ui_input_feed(&task_input, 4095U, prime_ms + 1U, prime_ms, prime_ms);
    assert(task_input.valid);
    fake_ms = 300U; graphics_initialized = graphics_running = 1U;
    adc_value = 0U;
    assert(debug_ui_graphics_run(render_continuously, NULL));
    while (owned_pixels != NULL) { ++fake_ms; lv_port_disp_service(fake_ms); }
    assert(stop_submissions == 1U && stop_while_owned == 1U);
    assert(seen_buffers[0] != NULL && seen_buffers[1] != NULL && immutable_checks > 100U);
    assert(notification_count > 0U && health_publications > 0U);
    printf("TASK_DMA_WAIT_STOP_PASS requests=%lu while_owned=%lu buffers=2 immutable_checks=%lu health=%lu\n",
        (unsigned long)stop_submissions, (unsigned long)stop_while_owned,
        (unsigned long)immutable_checks, (unsigned long)health_publications);
    if (strcmp(argv[2], "mode") == 0 || strcmp(argv[2], "all") == 0) probe_mode_confirmation(argv[1]);
    while (owned_pixels != NULL) { ++fake_ms; lv_port_disp_service(fake_ms); }
    if (strcmp(argv[2], "health") == 0 || strcmp(argv[2], "all") == 0) probe_recovered_health();
    if (strcmp(argv[2], "all") == 0) test_task_stop_latched();
    debug_ui_model_init(&task_model);
    app_snapshot.active = 1U;
    app_snapshot.active_identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    stop_submissions = stop_while_owned = 0U;
    previous_health_count = health_publications;
    graphics_running = debug_ui_graphics_run(assert_with_owned_buffer, NULL);
    assert(!graphics_running && !debug_ui_view_healthy() && owned_pixels != NULL);
    DebugUiWaitService();
    assert(stop_submissions == 1U && stop_while_owned == 1U);
    while (owned_pixels != NULL) { DebugUiWaitService(); lv_port_disp_service(fake_ms); }
    assert(health_publications > previous_health_count && !published_health.display_valid && !published_health.ui_valid);
    puts("TASK_ASSERT_LIVE_DMA_PASS local_escape=1 STOP_before_release=1 health_withdrawn=1 cleanup_completed=1");
    return 0;
}
