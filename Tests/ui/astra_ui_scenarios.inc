/** @file astra_ui_scenarios.inc
 * @brief Astra B real-LVGL page fixtures with explicitly synthetic motor data.
 * Included by ui_render_smoke.c after its shared raster and fixture helpers.
 * These fixtures exercise presentation only; no mailbox or hardware is used.
 */

/** @brief Reset a synthetic ID7 operation while retaining the profile's units. */
static void astra_scenario_draft(DebugUiOperation operation, DebugUiPage page)
{
    simulated_data();
    model.selected_motor = 6U;
    model.snapshot.motors[6].feedback_seen = 1U;
    model.snapshot.motors[6].feedback_valid = 1U;
    model.snapshot.motors[6].position_rad = debug_ui_degrees_to_radians(24.0f);
    model.snapshot.motors[6].actual_mode = operation == DEBUG_UI_OPERATION_POS_MOVE ?
        DEBUG_UI_MODE_POS_VEL : DEBUG_UI_MODE_MIT;
    model.draft.operation = operation;
    model.draft.target_motor_id = 7U;
    model.draft.requested_mode = model.snapshot.motors[6].actual_mode;
    model.draft.delta_rad = operation == DEBUG_UI_OPERATION_MIT_HOLD ? 0.0f :
        debug_ui_degrees_to_radians(-2.0f);
    model.draft.speed_rad_s = debug_ui_degrees_to_radians(1.5f);
    model.draft.kp = model.snapshot.motors[6].profile.default_kp;
    model.draft.kd = model.snapshot.motors[6].profile.default_kd;
    model.draft.hold_duration_ms = 700U;
    model.number_backup = model.draft;
    model.reviewed = model.draft;
    model.review_return_page = DEBUG_UI_PAGE_EDIT;
    model.notice_return_page = DEBUG_UI_PAGE_EDIT;
    model.prepare_return_page = DEBUG_UI_PAGE_OVERVIEW;
    model.diagnostic_return_page = DEBUG_UI_PAGE_OVERVIEW;
    model.page = page;
    model.reason = DEBUG_UI_REASON_NONE;
    assert(diagnostics.simulated && !model.outgoing_ready && !model.stop_ready);
}

/** @brief Write a sequential image and enforce absence of submitted operations. */
static void astra_scenario_save(const char *directory, unsigned *number,
                                const char *name)
{
    char filename[96];
    assert(directory != NULL && number != NULL && name != NULL);
    assert(diagnostics.simulated && !model.outgoing_ready && !model.stop_ready);
    (void)snprintf(filename, sizeof(filename), "%02u_%s", *number, name);
    save_page(directory, filename);
    ++*number;
}

/** @brief Cover all B menu families, popup fields, confirmations and outcomes. */
static void render_astra_scenarios(const char *directory)
{
    unsigned number = 26U;
    unsigned scenario_index;
    char name[72];
    static const DebugUiOperation motion_operations[] = {
        DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_OPERATION_MIT_MOVE,
        DEBUG_UI_OPERATION_MIT_HOLD
    };
    static const char *const motion_names[] = { "pos", "mit_move", "mit_hold" };
    static const DebugUiOperation simple_operations[] = {
        DEBUG_UI_OPERATION_ACQUIRE, DEBUG_UI_OPERATION_RELEASE,
        DEBUG_UI_OPERATION_CLEAR_FAULT, DEBUG_UI_OPERATION_DISABLE
    };
    static const char *const simple_names[] = {
        "acquire", "release", "clear_fault", "disable"
    };
    static const DebugUiReason notice_reasons[] = {
        DEBUG_UI_REASON_OUT_OF_RANGE, DEBUG_UI_REASON_STALE_FEEDBACK,
        DEBUG_UI_REASON_UNCALIBRATED, DEBUG_UI_REASON_DISABLED
    };
    static const char *const notice_names[] = {
        "notice_out_of_range", "notice_stale_feedback",
        "notice_uncalibrated", "readonly_notice"
    };
    assert(directory != NULL);

    /* Tile zero is already covered by 01_overview in the parent harness. */
    for (scenario_index = 1U; scenario_index < 3U; ++scenario_index) {
        simulated_data();
        model.focus = (uint8_t)scenario_index;
        (void)snprintf(name, sizeof(name), "home_tile_%u", scenario_index);
        astra_scenario_save(directory, &number, name);
    }
    simulated_data();
    model.focus = 2U;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    astra_scenario_save(directory, &number, "home_remote_control");
    model.snapshot.motion_enabled = model.snapshot.bench_profile = 0U;
    astra_scenario_save(directory, &number, "readonly_home");
    simulated_data();
    model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    model.snapshot.usb_connected = 0U;
    memset(model.snapshot.motors, 0, sizeof(model.snapshot.motors));
    astra_scenario_save(directory, &number, "home_unknown");

    /* B uses one bounded vertical motor list and a six-entry action list. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_MOTORS;
    model.focus = model.selected_motor = 0U;
    model.list_first = 0U;
    astra_scenario_save(directory, &number, "motor_list_first");
    for (scenario_index = 0U; scenario_index < 6U; ++scenario_index) {
        simulated_data();
        model.selected_motor = 6U;
        model.page = DEBUG_UI_PAGE_ACTIONS;
        model.focus = (uint8_t)scenario_index;
        model.list_first = scenario_index == 5U ? 1U : 0U;
        (void)snprintf(name, sizeof(name), "actions_row_%u", scenario_index);
        astra_scenario_save(directory, &number, name);
    }

    astra_scenario_draft(DEBUG_UI_OPERATION_MIT_MOVE, DEBUG_UI_PAGE_DETAIL);
    astra_scenario_save(directory, &number, "detail_mit_fresh");
    model.snapshot.motors[6].feedback_valid = 0U;
    model.snapshot.motors[6].feedback_age_ms = 920U;
    astra_scenario_save(directory, &number, "detail_mit_stale");
    model.snapshot.motors[6].feedback_seen = 0U;
    model.snapshot.motors[6].actual_mode = DEBUG_UI_MODE_UNKNOWN;
    model.snapshot.motors[6].mode_verified = 0U;
    astra_scenario_save(directory, &number, "detail_unknown");

    /* Each parameter has its own NUMBER popup, including HOLD's sparse map.
     * Stored EDIT focus is the parent row; NUMBER focus is not a field index. */
    for (scenario_index = 0U; scenario_index < 3U; ++scenario_index) {
        unsigned parameter_index;
        uint8_t parameter_count;
        astra_scenario_draft(motion_operations[scenario_index], DEBUG_UI_PAGE_EDIT);
        parameter_count = debug_ui_model_parameter_count(&model);
        (void)snprintf(name, sizeof(name), "%s_parameters", motion_names[scenario_index]);
        astra_scenario_save(directory, &number, name);
        for (parameter_index = 0U; parameter_index < parameter_count; ++parameter_index) {
            astra_scenario_draft(motion_operations[scenario_index], DEBUG_UI_PAGE_NUMBER);
            model.edit_field = debug_ui_model_parameter_field(&model, (uint8_t)parameter_index);
            assert(model.edit_field < 5U);
            model.page_focus[DEBUG_UI_PAGE_EDIT] = (uint8_t)parameter_index;
            model.page_first[DEBUG_UI_PAGE_EDIT] = 0U;
            (void)snprintf(name, sizeof(name), "%s_number_field_%u",
                motion_names[scenario_index], (unsigned)model.edit_field);
            astra_scenario_save(directory, &number, name);
        }
        astra_scenario_draft(motion_operations[scenario_index], DEBUG_UI_PAGE_EDIT);
        model.focus = parameter_count;
        model.list_first = parameter_count >= 5U ? 1U : 0U;
        model.page_focus[DEBUG_UI_PAGE_EDIT] = model.focus;
        model.page_first[DEBUG_UI_PAGE_EDIT] = model.list_first;
        (void)snprintf(name, sizeof(name), "%s_preview_row", motion_names[scenario_index]);
        astra_scenario_save(directory, &number, name);
        model.page = DEBUG_UI_PAGE_REVIEW;
        (void)snprintf(name, sizeof(name), "%s_review_release", motion_names[scenario_index]);
        astra_scenario_save(directory, &number, name);
        model.review_released = 1U;
        model.confirm_elapsed_ms = 450U;
        (void)snprintf(name, sizeof(name), "%s_review_hold", motion_names[scenario_index]);
        astra_scenario_save(directory, &number, name);
    }

    /* A selected mode remains distinct from a verified current mode. */
    for (scenario_index = 0U; scenario_index < 2U; ++scenario_index) {
        astra_scenario_draft(DEBUG_UI_OPERATION_SET_MODE, DEBUG_UI_PAGE_MODES);
        model.focus = (uint8_t)scenario_index;
        model.snapshot.motors[6].actual_mode = DEBUG_UI_MODE_POS_VEL;
        astra_scenario_save(directory, &number,
            scenario_index ? "modes_select_mit" : "modes_select_pos");
        model.draft.requested_mode = scenario_index ? DEBUG_UI_MODE_MIT : DEBUG_UI_MODE_POS_VEL;
        model.snapshot.motors[6].actual_mode = scenario_index ? DEBUG_UI_MODE_POS_VEL : DEBUG_UI_MODE_MIT;
        model.draft.delta_rad = model.draft.speed_rad_s = 0.0f;
        model.draft.kp = model.draft.kd = 0.0f;
        model.draft.hold_duration_ms = 0U;
        model.reviewed = model.draft;
        model.review_return_page = DEBUG_UI_PAGE_MODES;
        model.page_focus[DEBUG_UI_PAGE_MODES] = model.focus;
        model.page = DEBUG_UI_PAGE_REVIEW;
        astra_scenario_save(directory, &number,
            scenario_index ? "mode_mit_confirmation" : "mode_pos_confirmation");
    }
    for (scenario_index = 0U; scenario_index < 4U; ++scenario_index) {
        simulated_data();
        model.selected_motor = 6U;
        model.page = DEBUG_UI_PAGE_RECOVERY;
        model.focus = (uint8_t)scenario_index;
        (void)snprintf(name, sizeof(name), "recovery_row_%u", scenario_index);
        astra_scenario_save(directory, &number, name);
    }
    for (scenario_index = 0U; scenario_index < 4U; ++scenario_index) {
        astra_scenario_draft(simple_operations[scenario_index], DEBUG_UI_PAGE_REVIEW);
        model.draft.delta_rad = model.draft.speed_rad_s = 0.0f;
        model.draft.kp = model.draft.kd = 0.0f;
        model.draft.hold_duration_ms = 0U;
        model.draft.requested_mode = DEBUG_UI_MODE_UNKNOWN;
        model.reviewed = model.draft;
        model.review_return_page = scenario_index < 2U ? DEBUG_UI_PAGE_PREPARE : DEBUG_UI_PAGE_RECOVERY;
        model.page_focus[DEBUG_UI_PAGE_RECOVERY] = scenario_index == 3U ? 1U : 0U;
        if (scenario_index == 0U) model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
        if (scenario_index == 2U) model.snapshot.motors[6].fault_flags = 6U;
        (void)snprintf(name, sizeof(name), "%s_confirmation", simple_names[scenario_index]);
        astra_scenario_save(directory, &number, name);
    }

    astra_scenario_draft(DEBUG_UI_OPERATION_ACQUIRE, DEBUG_UI_PAGE_PREPARE);
    model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    model.snapshot.motors[3].fault_flags = 0U;
    astra_scenario_save(directory, &number, "prepare_acquire_ready");
    model.prepare_return_page = DEBUG_UI_PAGE_RECOVERY;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_FAULT;
    model.snapshot.unconfirmed_disable_mask = 0x40U;
    model.snapshot.motors[6].feedback_valid = 0U;
    model.snapshot.motors[6].feedback_age_ms = 920U;
    astra_scenario_save(directory, &number, "prepare_disable_unconfirmed");
    model.snapshot.motion_enabled = model.snapshot.bench_profile = 0U;
    astra_scenario_save(directory, &number, "readonly_prepare_locked");

    /* Rejection popups preserve an explicit parent instead of replacing drafts. */
    for (scenario_index = 0U; scenario_index < 4U; ++scenario_index) {
        astra_scenario_draft(DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_PAGE_NOTICE);
        model.reason = notice_reasons[scenario_index];
        model.page_focus[DEBUG_UI_PAGE_EDIT] = 2U;
        if (scenario_index == 1U) {
            model.snapshot.motors[6].feedback_valid = 0U;
            model.snapshot.motors[6].feedback_age_ms = 920U;
        }
        if (scenario_index == 2U) model.snapshot.motors[6].identity_verified = 0U;
        if (scenario_index == 3U) {
            model.snapshot.motion_enabled = model.snapshot.bench_profile = 0U;
            model.notice_return_page = DEBUG_UI_PAGE_ACTIONS;
        }
        astra_scenario_save(directory, &number, notice_names[scenario_index]);
    }

    /* The parent harness already renders all three diagnostic leaves. */
    for (scenario_index = 0U; scenario_index < 3U; ++scenario_index) {
        simulated_data();
        model.page = DEBUG_UI_PAGE_DIAGNOSTIC_MENU;
        model.focus = (uint8_t)scenario_index;
        model.diagnostic_return_page = DEBUG_UI_PAGE_OVERVIEW;
        (void)snprintf(name, sizeof(name), "diagnostic_menu_%u", scenario_index);
        astra_scenario_save(directory, &number, name);
    }
    simulated_data();
    model.selected_motor = 6U;
    model.page = DEBUG_UI_PAGE_REGISTERS;
    model.snapshot.motors[6].register_seen_mask = 0U;
    astra_scenario_save(directory, &number, "registers_unknown");

    /* Fresh MIT activity resolves the target from actual snapshot evidence. */
    simulated_data();
    model.snapshot.active = 1U;
    model.snapshot.active_motor_mask = 2U;
    model.snapshot.target_motor_id = 2U;
    model.snapshot.active_identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    model.snapshot.active_stage = DEBUG_UI_STAGE_MOTION;
    model.snapshot.motors[1].enabled = 1U;
    model.snapshot.motors[1].velocity_rad_s = debug_ui_degrees_to_radians(1.5f);
    model.submitted.operation = DEBUG_UI_OPERATION_MIT_MOVE;
    model.submitted.target_motor_id = 2U;
    model.submitted.requested_mode = DEBUG_UI_MODE_MIT;
    model.admission_seen = 1U;
    debug_ui_model_update(&model, &model.snapshot, model.snapshot.timestamp_us, 1U, 1U);
    assert(model.page == DEBUG_UI_PAGE_RUNNING);
    astra_scenario_save(directory, &number, "running_mit_actual_target");

    simulated_data();
    model.page = DEBUG_UI_PAGE_RESULT;
    model.completion_seen = 1U;
    model.completion.operation = DEBUG_UI_OPERATION_STOP;
    model.completion.code = DEBUG_UI_STOPPED;
    model.completion.stage = DEBUG_UI_STAGE_DISABLE;
    model.completion.disabled_confirmed = 1U;
    astra_scenario_save(directory, &number, "result_stopped_confirmed");
    model.completion.code = DEBUG_UI_CANCELLED;
    model.reason = DEBUG_UI_REASON_OLD_EPOCH;
    astra_scenario_save(directory, &number, "result_cancelled_epoch");
    model.page = DEBUG_UI_PAGE_FAULT;
    model.completion.code = DEBUG_UI_FAILED;
    model.reason = DEBUG_UI_REASON_TORQUE_LIMIT;
    model.completion.reason = model.reason;
    astra_scenario_save(directory, &number, "result_torque_protection");
    assert(number >= 56U);
    printf("ASTRA_SCENARIOS_OK additional=%u last_number=%u\n", number - 26U, number - 1U);
}
