/** @file static_peak_reason_render.c
 * @brief Synthetic peak/first-exit fixtures rendered by production LVGL 8.3.11 and its real flush port.
 * Includes maintained render helpers; no generic drawing or physical feedback is used.
 */
#define main existing_full_lvgl_render_main
#include "ui_render_smoke.c"
#undef main

/** @brief Bind a clearly simulated selected-axis report to a real RAM-ledger submission. */
static void peak_render_fixture(uint8_t motor_id)
{
    DebugUiStaticLoadReport *report;
    DebugUiMotorView *motor;
    uint8_t joint_index = (uint8_t)(motor_id - 1U);
    simulated_data();
    model.single_static_flow = model.static_locked = 1U;
    model.selected_motor = joint_index;
    memset(&model.submitted, 0, sizeof(model.submitted));
    model.submitted.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    model.submitted.identity.epoch = model.snapshot.epoch;
    model.submitted.identity.request_id = 501U;
    model.submitted.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    model.submitted.target_motor_id = motor_id;
    model.submitted.hold_duration_ms = 20000U;
    model.submitted.created_at_us = model.snapshot.timestamp_us;
    assert(debug_ui_load_trials_freeze(&model.load_trials, motor_id));
    assert(debug_ui_load_trials_submit(&model.load_trials, &model.submitted));
    report = &model.snapshot.static_load;
    memset(report, 0, sizeof(*report));
    report->identity = model.submitted.identity;
    report->operation = model.submitted.operation;
    report->motor_mask = report->valid_mask = (uint8_t)(1U << joint_index);
    report->active = report->takeover_active = 1U;
    report->phase = DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER;
    report->started_at_us = model.submitted.created_at_us + 1U;
    report->duration_ms = model.submitted.hold_duration_ms;
    report->takeover_elapsed_ms = 672U;
    report->target_torque_ff_nm = -1.0f; report->requested_torque_ff_nm = -0.65f;
    report->transmitted_torque_ff_nm = -0.65f;
    report->target_position_rad[joint_index] = 0.2f;
    report->samples[joint_index] = 125U;
    report->peak_error_rad[joint_index] = debug_ui_degrees_to_radians(0.12f);
    report->peak_torque_nm[joint_index] = 0.4469f;
    report->peak_mos_c[joint_index] = 30.0f; report->peak_rotor_c[joint_index] = 31.0f;
    motor = &model.snapshot.motors[joint_index];
    motor->motor_id = motor_id; motor->feedback_seen = motor->feedback_valid = motor->enabled = 1U;
    motor->driver_state = 1U;
    motor->feedback_age_ms = 4U; motor->actual_mode = DEBUG_UI_MODE_MIT;
    motor->position_rad = 0.2f + debug_ui_degrees_to_radians(0.12f);
    motor->torque_nm = -0.0317f; motor->mos_temperature_c = 30.0f; motor->rotor_temperature_c = 31.0f;
    model.snapshot.target_motor_id = motor_id;
    model.snapshot.active = model.awaiting_result = 1U;
    model.snapshot.active_motor_mask = report->motor_mask;
    model.snapshot.active_identity = report->identity;
    model.snapshot.timestamp_us += 672000U;
    debug_ui_model_update(&model, &model.snapshot, model.snapshot.timestamp_us, 1U, 1U);
    model.page = DEBUG_UI_PAGE_RUNNING;
}

/** @brief Active fixture observations remain inside the unchanged 0.5-degree position guard. */
static void peak_render_active_guard(DebugUiStaticLoadPhase phase)
{
    const DebugUiStaticLoadReport *report = &model.snapshot.static_load;
    uint8_t joint_index = (uint8_t)(model.submitted.target_motor_id - 1U);
    const DebugUiMotorView *motor = &model.snapshot.motors[joint_index];
    float error_deg = fabsf(debug_ui_radians_to_degrees(motor->position_rad - report->target_position_rad[joint_index]));
    float peak_deg = debug_ui_radians_to_degrees(report->peak_error_rad[joint_index]);
    if (error_deg > 0.5f || peak_deg > 0.5f) {
        fprintf(stderr, "STATIC_PEAK_LVGL_ACTIVE_GUARD_RED phase=%u current_deg=%.4f peak_deg=%.4f limit_deg=0.5\n",
            (unsigned)phase, (double)error_deg, (double)peak_deg);
        exit(1);
    }
    assert(report->active && report->phase == phase && !report->first_exit_latched);
    assert(motor->enabled && motor->driver_state == 1U && fabsf(motor->torque_nm) <= 3.5f && report->peak_torque_nm[joint_index] <= 3.5f);
    assert(report->identity.origin == model.submitted.identity.origin && report->identity.epoch == model.submitted.identity.epoch &&
        report->identity.request_id == model.submitted.identity.request_id);
    assert(report->motor_mask == (uint8_t)(1U << joint_index) && report->valid_mask == report->motor_mask);
    if (phase == DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER)
        assert(report->takeover_active && !report->takeover_complete && report->elapsed_ms == 0U);
    else
        assert(!report->takeover_active && report->takeover_complete && report->takeover_elapsed_ms == 1000U && report->elapsed_ms == 4U);
}

/** @brief Match each independent failed report's identity, clock, phase, first cause and violating peak. */
static void peak_render_failure_cause(DebugUiStaticLoadPhase phase, DebugUiReason reason, DebugUiStage stage)
{
    const DebugUiStaticLoadReport *report = &model.snapshot.static_load;
    uint8_t joint_index = (uint8_t)(model.submitted.target_motor_id - 1U);
    assert(!report->active && !report->takeover_active && report->disabled_confirmed && report->stopped);
    assert(report->phase == phase && report->first_exit_latched && report->first_exit_phase == phase);
    assert(report->first_exit_reason == reason && report->first_exit_stage == stage && report->first_exit_code == DEBUG_UI_FAILED);
    assert(report->identity.origin == model.submitted.identity.origin && report->identity.epoch == model.submitted.identity.epoch &&
        report->identity.request_id == model.submitted.identity.request_id);
    assert(report->motor_mask == (uint8_t)(1U << joint_index) && report->valid_mask == report->motor_mask && report->samples[joint_index]);
    assert(report->started_at_us >= model.submitted.created_at_us && report->started_at_us <= model.snapshot.timestamp_us);
    if (phase == DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER) {
        assert(!report->takeover_complete && report->elapsed_ms == 0U && report->takeover_elapsed_ms == 672U);
        assert(report->peak_error_rad[joint_index] > debug_ui_degrees_to_radians(0.5f) && report->peak_torque_nm[joint_index] <= 3.5f);
    } else {
        assert(report->takeover_complete && report->takeover_elapsed_ms == 1000U && report->elapsed_ms == 200U);
        assert(report->peak_torque_nm[joint_index] > 3.5f && report->peak_error_rad[joint_index] <= debug_ui_degrees_to_radians(0.5f));
    }
}

/** @brief Deliver a later STOP result through the real model, then render without changing retained report facts. */
static unsigned peak_render_cleanup_reason(DebugUiReason reason)
{
    DebugUiCompletion completion = model.completion;
    DebugUiStaticLoadReport retained = model.snapshot.static_load;
    char expected_identity[DEBUG_UI_VIEW_TEXT_BYTES];
    model.outgoing_stop.identity = model.completion.identity;
    ++model.outgoing_stop.identity.request_id;
    model.outgoing_stop.operation = DEBUG_UI_OPERATION_STOP;
    model.outgoing_stop.target_motor_id = model.submitted.target_motor_id;
    completion.reason = reason;
    completion.identity = model.outgoing_stop.identity;
    completion.motor_mask = retained.motor_mask;
    completion.disabled_confirmed = 1U;
    completion.timestamp_us = model.now_us;
    debug_ui_model_completion(&model, &completion);
    assert(model.reason == reason && model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT);
    assert(memcmp(&retained, &model.snapshot.static_load, sizeof(retained)) == 0);
    assert(debug_ui_graphics_run(render_graphics, NULL));
    (void)snprintf(expected_identity, sizeof(expected_identity), "单轴 ID%u / RQ%lu", model.submitted.target_motor_id,
        (unsigned long)retained.identity.request_id);
    if (strcmp(view.page.row_text[0], expected_identity) != 0) {
        fprintf(stderr, "STATIC_PEAK_LVGL_REVIEW_ID_RED cleanup_reason=%u row=%s\n", (unsigned)reason, view.page.row_text[0]);
        return 1U;
    }
    assert(strstr(view.page.footer_text, debug_ui_reason_text(retained.first_exit_reason)) != NULL);
    return 0U;
}

/** @brief Validate current/peak/stale output, HOLD clock transition and retained first cause after STOP. */
static void peak_render_cases(const char *directory)
{
    DebugUiStaticLoadReport *report;
    DebugUiMotorView *motor;
    const DebugUiLoadTrialRecord *record;
    unsigned review_failures = 0U;
    peak_render_fixture(7U);
    save_page(directory, "310_takeover_current_and_peak");
    peak_render_active_guard(DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER);
    if (strstr(lv_label_get_text(view.page.rows[6]), "峰值") == NULL) {
        fputs("STATIC_PEAK_LVGL_RED takeover_has_no_peak_label=1\n", stderr); exit(1);
    }
    assert(strstr(lv_label_get_text(view.page.rows[4]), "当前") != NULL);
    report = &model.snapshot.static_load; motor = &model.snapshot.motors[6];
    motor->feedback_valid = 0U; motor->feedback_age_ms = 200U;
    save_page(directory, "311_takeover_feedback_expired");
    peak_render_active_guard(DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER);
    assert(strstr(lv_label_get_text(view.page.rows[4]), "过期") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[6]), "0.45") != NULL);
    motor->feedback_valid = 1U; motor->feedback_age_ms = 4U;
    report->takeover_active = 0U; report->takeover_complete = 1U;
    report->takeover_elapsed_ms = 1000U;
    report->requested_torque_ff_nm = report->transmitted_torque_ff_nm = report->target_torque_ff_nm;
    report->phase = DEBUG_UI_STATIC_LOAD_PHASE_HOLD;
    report->started_at_us += 1000000U;
    report->elapsed_ms = 4U;
    model.snapshot.timestamp_us = report->started_at_us + 4000U;
    debug_ui_model_update(&model, &model.snapshot, model.snapshot.timestamp_us, 1U, 1U);
    model.page = DEBUG_UI_PAGE_RUNNING;
    record = debug_ui_load_trials_newest(&model.load_trials, 0U);
    assert(record != NULL && record->elapsed_ms == 4U);
    save_page(directory, "312_hold_current_and_peak");
    peak_render_active_guard(DEBUG_UI_STATIC_LOAD_PHASE_HOLD);
    assert(strstr(lv_label_get_text(view.page.rows[6]), "峰值") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[1]), "0.0 / 20") != NULL);
    motor->feedback_valid = 0U; motor->feedback_age_ms = 200U;
    save_page(directory, "313_hold_feedback_expired");
    peak_render_active_guard(DEBUG_UI_STATIC_LOAD_PHASE_HOLD);
    assert(strstr(lv_label_get_text(view.page.rows[3]), "过期") != NULL);
    /* Start a new TAKEOVER trial; this position-limit exit never entered HOLD. */
    peak_render_fixture(1U);
    report = &model.snapshot.static_load;
    motor = &model.snapshot.motors[0];
    assert(report->phase == DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER && !report->takeover_complete && report->elapsed_ms == 0U);
    report->peak_error_rad[0] = debug_ui_degrees_to_radians(0.6994f);
    motor->position_rad = report->target_position_rad[0] + report->peak_error_rad[0];
    motor->torque_nm = 0.0f; motor->enabled = 0U; motor->driver_state = 0U;
    report->first_exit_latched = 1U;
    report->first_exit_phase = DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER;
    report->first_exit_reason = DEBUG_UI_REASON_OUT_OF_RANGE;
    report->first_exit_code = DEBUG_UI_FAILED;
    report->first_exit_stage = DEBUG_UI_STAGE_ENABLE;
    report->active = report->takeover_active = 0U;
    report->disabled_confirmed = report->stopped = 1U;
    report->stop_identity = report->identity; report->stop_identity.request_id = 502U;
    model.awaiting_result = model.snapshot.active = 0U;
    model.snapshot.timestamp_us += 1000U;
    model.now_us = model.snapshot.timestamp_us;
    model.completion_seen = 1U; model.completion.identity = report->stop_identity;
    model.completion.operation = DEBUG_UI_OPERATION_STOP; model.completion.code = DEBUG_UI_STOPPED;
    model.completion.reason = DEBUG_UI_REASON_NONE;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_RESULT;
    save_page(directory, "314_takeover_first_cause_after_stop");
    peak_render_failure_cause(DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER, DEBUG_UI_REASON_OUT_OF_RANGE, DEBUG_UI_STAGE_ENABLE);
    assert(strstr(view.page.footer_text, "首因") != NULL);
    assert(strstr(view.page.footer_text, "前馈接入") != NULL && strstr(view.page.footer_text, "超出范围") != NULL);
    assert(strstr(view.page.title_text, "失败") != NULL);
    assert(strstr(view.page.row_text[2], "0.447") != NULL);
    assert(strstr(view.page.row_text[1], "0.699") != NULL);
    assert(report->elapsed_ms == 0U);
    review_failures += peak_render_cleanup_reason(DEBUG_UI_REASON_NOT_READY);
    review_failures += peak_render_cleanup_reason(DEBUG_UI_REASON_UNHEALTHY);
    /* Build a separate trial: a small position error and a violating HOLD torque
     * are retained before a later zero-torque disabled acknowledgement and STOP. */
    peak_render_fixture(1U);
    report = &model.snapshot.static_load;
    motor = &model.snapshot.motors[0];
    report->phase = DEBUG_UI_STATIC_LOAD_PHASE_HOLD;
    report->takeover_active = 0U; report->takeover_complete = 1U;
    report->takeover_elapsed_ms = 1000U;
    report->started_at_us = model.submitted.created_at_us + 1000000U;
    report->elapsed_ms = 200U;
    report->requested_torque_ff_nm = report->transmitted_torque_ff_nm = report->target_torque_ff_nm;
    report->samples[0] = 51U;
    report->peak_error_rad[0] = debug_ui_degrees_to_radians(0.12f);
    report->peak_torque_nm[0] = 3.75f;
    report->first_exit_latched = 1U;
    report->first_exit_phase = DEBUG_UI_STATIC_LOAD_PHASE_HOLD;
    report->first_exit_reason = DEBUG_UI_REASON_TORQUE_LIMIT;
    report->first_exit_stage = DEBUG_UI_STAGE_HOLD;
    report->first_exit_code = DEBUG_UI_FAILED;
    report->active = 0U; report->disabled_confirmed = report->stopped = 1U;
    report->stop_identity = report->identity; report->stop_identity.request_id = 502U;
    motor->position_rad = report->target_position_rad[0] + report->peak_error_rad[0];
    motor->torque_nm = 0.0f; motor->enabled = 0U; motor->driver_state = 0U;
    model.awaiting_result = model.snapshot.active = 0U;
    model.snapshot.timestamp_us = report->started_at_us + 200000U;
    debug_ui_model_update(&model, &model.snapshot, model.snapshot.timestamp_us, 1U, 1U);
    model.completion_seen = 1U; model.completion.identity = report->stop_identity;
    model.completion.operation = DEBUG_UI_OPERATION_STOP; model.completion.code = DEBUG_UI_STOPPED;
    model.completion.reason = DEBUG_UI_REASON_NONE;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_RESULT;
    save_page(directory, "315_hold_first_cause_after_stop");
    peak_render_failure_cause(DEBUG_UI_STATIC_LOAD_PHASE_HOLD, DEBUG_UI_REASON_TORQUE_LIMIT, DEBUG_UI_STAGE_HOLD);
    assert(strstr(view.page.footer_text, "保持") != NULL && strstr(view.page.footer_text, "转矩超限") != NULL);
    if (report->phase != DEBUG_UI_STATIC_LOAD_PHASE_HOLD || report->first_exit_stage != DEBUG_UI_STAGE_HOLD ||
        report->peak_torque_nm[0] <= 3.5f || report->peak_error_rad[0] >= debug_ui_degrees_to_radians(0.5f)) {
        fputs("STATIC_PEAK_LVGL_REVIEW_HOLD_RED torque_failure_fixture_has_no_violating_torque_or_hold_stage=1\n", stderr);
        ++review_failures;
    }
    assert(strstr(view.page.row_text[1], "0.12") != NULL && strstr(view.page.row_text[2], "3.75") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[4]), "0.2/20s") != NULL);
    assert(report->first_exit_phase == DEBUG_UI_STATIC_LOAD_PHASE_HOLD && report->first_exit_stage == DEBUG_UI_STAGE_HOLD);
    review_failures += peak_render_cleanup_reason(DEBUG_UI_REASON_NOT_READY);
    review_failures += peak_render_cleanup_reason(DEBUG_UI_REASON_UNHEALTHY);
    if (review_failures) exit(1);
    ++report->identity.request_id;
    model.reason = DEBUG_UI_REASON_NOT_READY;
    assert(debug_ui_graphics_run(render_graphics, NULL));
    assert(strcmp(view.page.row_text[0], "ID1 / 尚未就绪") == 0);
    model.reason = DEBUG_UI_REASON_UNHEALTHY;
    assert(debug_ui_graphics_run(render_graphics, NULL));
    assert(strcmp(view.page.row_text[0], "ID1 / 输入或显示异常") == 0);
    puts("STATIC_PEAK_LVGL_REVIEW_PASS later_stop_reasons=2 both_failure_phases=1 original_report_id=1 unbound_rejection_preserved=1 hold_torque=3.75 hold_error_deg=0.12 hold_stage=HOLD");
    puts("STATIC_PEAK_LVGL_CAUSAL_PASS active_pages=4 active_error_deg=0.12 active_peak_deg=0.12 position_failure_deg=0.6994 position_phase=TAKEOVER position_stage=ENABLE torque_failure_nm=3.75 torque_phase=HOLD torque_stage=HOLD identities_and_clocks=1");
    printf("STATIC_PEAK_LVGL_PASS current_peak=1 expired=1 first_reason_after_stop=1 phase=1 hold_clock=1 view_bytes=%lu ledger_bytes=%lu report_bytes=%lu\n",
        (unsigned long)sizeof(DebugUiView), (unsigned long)sizeof(DebugUiLoadTrials), (unsigned long)sizeof(DebugUiStaticLoadReport));
}

/** @brief Exercise actual LVGL fonts, bounds, raster, buffer ownership and free-pool gates. */
int main(int argc, char **argv)
{
    lv_mem_monitor_t memory;
    assert(argc == 3);
    assert(sizeof(DebugUiView) < 4096U);
    assert(debug_ui_graphics_run(initialize_graphics, NULL));
    verify_font_inventory(argv[2]);
    peak_render_cases(argv[1]);
    lv_mem_monitor(&memory);
    assert(memory.free_size > 4096U && owned_pixels == NULL);
    printf("STATIC_PEAK_LVGL_MEMORY_PASS free_bytes=%lu max_pool_used=%lu\n",
        (unsigned long)memory.free_size, (unsigned long)sampled_pool_used_max);
    return 0;
}
