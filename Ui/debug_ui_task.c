/** @file debug_ui_task.c
 * @brief Five-ms low-priority UI owner; ProtocolTask alone admits app requests.
 * No motor drivers or protocol text are called here. A local graphics failure
 * withdraws UI health while input/STOP/platform cleanup continue to be serviced.
 */
#include "debug_ui_task.h"
#include "../App/Config/debug_ui_config.h"
#include "cmsis_os.h"
#if AETHOR_DEBUG_UI_ENABLE
#include "debug_ui_view.h"
#include "debug_ui_graphics_guard.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "../App/aethor_app.h"
#include "../App/Platform/lcd_key_adc.h"
#include "../App/Config/build_info.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static DebugUiModel task_model;
static DebugUiInput task_input;
static DebugUiView task_view;
static DebugUiViewDiagnostics task_diagnostics;
static DebugUiSnapshot task_snapshot;
static uint32_t last_health_ms, last_snapshot_ms, last_view_ms, last_tick_ms;
static uint32_t health_sequence;
static uint8_t graphics_initialized, graphics_running;
static uint8_t redraw_after_recovery, view_dirty;

/** @brief Use the same HAL millisecond domain as platform ADC acquisition stamps. */
uint32_t DebugUiNowMs(void) { return HAL_GetTick(); }

/** @brief Submit at most two slots (STOP first); no automatic ordinary retries. */
static void dispatch_requests(void)
{
    unsigned index;
    DebugUiRequest request;
    for (index = 0U; index < 2U && debug_ui_model_take_request(&task_model, &request); ++index) {
        DebugUiReason reason = aethor_app_debug_ui_submit(&request);
        if (reason != DEBUG_UI_REASON_NONE) {
            debug_ui_model_submit_failed(&task_model, &request, reason);
            view_dirty = 1U;
        }
        AethorNotifyProtocolTask();
    }
}

/** @brief Service input, snapshots, results and independent health without LVGL objects. */
static void service_control_io(void)
{
    uint32_t now_ms = DebugUiNowMs();
    uint64_t now_us = AethorUiTimestampUs();
    LcdKeyAdcSample sample;
    DebugUiInputEvent event;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    uint8_t display_valid;
    unsigned result_index;
    lcd_key_adc_service(now_ms);
    lcd_key_adc_latest(now_ms, &sample);
    if (sample.valid) (void)debug_ui_input_feed(&task_input, sample.raw, sample.seq, sample.ms, now_ms);
    debug_ui_input_poll(&task_input, now_ms);
    display_valid = (uint8_t)(graphics_initialized && graphics_running && lv_port_disp_healthy());
    if ((uint32_t)(now_ms - last_snapshot_ms) >= 5U || !task_model.snapshot_seen) {
        last_snapshot_ms = now_ms;
        if (aethor_app_debug_ui_get_snapshot(now_us, &task_snapshot))
            debug_ui_model_update(&task_model, &task_snapshot, now_us, task_input.valid, display_valid);
        else debug_ui_model_update(&task_model, NULL, now_us, task_input.valid, display_valid);
    } else debug_ui_model_update(&task_model, NULL, now_us, task_input.valid, display_valid);
    if (debug_ui_input_emergency_event(&task_input, &event)) {
        lv_port_indev_event(&task_model, &event);
        view_dirty = 1U;
    }
    while (debug_ui_input_event(&task_input, &event)) {
        lv_port_indev_event(&task_model, &event);
        view_dirty = 1U;
    }
    dispatch_requests();
    if (aethor_app_debug_ui_poll_admission(&admission)) {
        debug_ui_model_admission(&task_model, &admission);
        view_dirty = 1U;
    }
    for (result_index = 0U; result_index < DEBUG_UI_RESULT_CAPACITY; ++result_index) {
        if (!aethor_app_debug_ui_poll_completion(&completion)) break;
        debug_ui_model_completion(&task_model, &completion);
        view_dirty = 1U;
    }
    task_diagnostics.adc_raw = sample.raw;
    task_diagnostics.adc_seen = (uint8_t)(sample.seq != 0U || sample.valid);
    task_diagnostics.adc_age_ms = now_ms - sample.ms;
    task_diagnostics.key = lv_port_indev_screen_key(task_input.key);
    task_diagnostics.input_valid = task_input.valid;
    task_diagnostics.input_fault = (uint32_t)task_input.fault;
    if ((uint32_t)(now_ms - last_health_ms) >= 50U) {
        DebugUiHealth health;
        LcdSt7789Status display_status;
        LvPortDispRefreshStatus refresh_status;
        memset(&health, 0, sizeof(health));
        lv_port_disp_status(&display_status);
        lv_port_disp_refresh_status(&refresh_status);
        health.ui_timestamp_us = now_us;
        health.ui_sequence = ++health_sequence;
        health.input_valid = task_input.valid;
        health.display_valid = display_valid;
        health.ui_valid = (uint8_t)(task_input.valid && display_valid);
        /* Historical counters survive recovery; only current faults revoke health. */
        health.display_error = (uint32_t)!display_valid;
        health.ui_diagnostics_valid = 1U;
        health.adc_raw = sample.raw;
        health.input_key = (uint8_t)lv_port_indev_screen_key(task_input.key);
        health.input_age_ms = now_ms - sample.ms;
        health.display_flush_last_us = display_status.transfer.last_duration_ms * 1000U;
        health.display_dma_error_count = display_status.transfer.errors + display_status.transfer.timeouts;
        health.ui_stack_min_words = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
        aethor_app_debug_ui_update_health(&health);
        AethorNotifyProtocolTask();
        task_diagnostics.dma_errors = display_status.transfer.errors + display_status.transfer.timeouts;
        task_diagnostics.spi_errors = display_status.spi_errors;
        task_diagnostics.init_errors = display_status.init_errors;
        task_diagnostics.flush_ms = display_status.transfer.last_duration_ms;
        task_diagnostics.refresh_bytes = refresh_status.last_bytes;
        task_diagnostics.refresh_ms = refresh_status.last_duration_ms;
        task_diagnostics.refresh_fps_tenths = refresh_status.fps_tenths;
        task_diagnostics.refresh_completed = refresh_status.completed;
        task_diagnostics.refresh_failed = refresh_status.failed;
        task_diagnostics.stack_free_words = health.ui_stack_min_words;
        AethorGetControlTiming(&task_diagnostics.control_execution_max_us, &task_diagnostics.control_period_max_us);
        last_health_ms = now_ms;
    }
}

/** @brief Service control liveness during display waits without recursing into LVGL. */
void DebugUiWaitService(void)
{
    service_control_io();
    (void)osDelay(1U);
}

/** @brief Initialize LVGL and every page inside the assertion escape boundary. */
static void graphics_initialize(void *context)
{
    (void)context;
    lv_init();
    if (!lv_port_disp_init()) return;
    if (!debug_ui_view_init(&task_view)) return;
    if (!lv_port_indev_init(task_view.group)) return;
    graphics_initialized = 1U;
    view_dirty = 1U;
}

/** @brief Render model events immediately, refresh telemetry at 100 ms, and animate. */
static void graphics_service(void *context)
{
    uint32_t now_ms = DebugUiNowMs();
    (void)context;
    lv_tick_inc(now_ms - last_tick_ms);
    last_tick_ms = now_ms;
    if (redraw_after_recovery) {
        lv_obj_invalidate(task_view.page.root);
        redraw_after_recovery = 0U;
    }
    if (view_dirty || task_view.visible != task_model.page || (uint32_t)(now_ms - last_view_ms) >= 100U) {
        debug_ui_view_update(&task_view, &task_model, &task_diagnostics);
        last_view_ms = now_ms;
        view_dirty = 0U;
    }
    debug_ui_view_animate(&task_view);
    (void)lv_timer_handler();
}
#endif

/** @brief Run the low-priority static UI task, or remain dormant in a disabled build. */
void StartDebugUiTask(void const *argument)
{
    (void)argument;
#if AETHOR_DEBUG_UI_ENABLE
    debug_ui_model_init(&task_model);
    task_model.selected_motor = (uint8_t)(DEBUG_UI_INITIAL_MOTOR_ID - 1U);
    if (DEBUG_UI_INITIAL_MOTOR_ID > DEBUG_UI_VISIBLE_MOTORS)
        task_model.list_first = (uint8_t)(DEBUG_UI_INITIAL_MOTOR_ID - DEBUG_UI_VISIBLE_MOTORS);
    task_diagnostics.firmware_version = build_info_get()->firmware_version;
    debug_ui_input_init(&task_input, DebugUiNowMs());
    (void)lcd_key_adc_init();
    graphics_running = debug_ui_graphics_run(graphics_initialize, NULL);
    last_tick_ms = DebugUiNowMs();
    for (;;) {
        lv_port_disp_service(DebugUiNowMs());
        service_control_io();
        if (!task_model.snapshot.active && !task_model.snapshot.pending && !task_model.snapshot.stop_pending &&
            !task_model.awaiting_result && !task_model.stop_waiting &&
            task_model.stop_latch_state != DEBUG_UI_STOP_LATCH_WAITING &&
            lv_port_disp_try_recover(DebugUiNowMs())) {
            redraw_after_recovery = 1U;
            view_dirty = 1U;
        }
        if (graphics_initialized && graphics_running && lv_port_disp_healthy())
            graphics_running = debug_ui_graphics_run(graphics_service, NULL);
        (void)osDelay(5U);
    }
#else
    /* This entry is deliberately never registered by a disabled target. */
    for (;;) (void)osDelay(1000U);
#endif
}
