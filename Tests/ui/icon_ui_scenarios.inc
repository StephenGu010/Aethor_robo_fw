/** @file icon_ui_scenarios.inc
 * @brief Additional real-LVGL visual scenarios using synthetic, SIM-marked data.
 * Included after the smoke harness helpers; these snapshots exercise presentation
 * only. No request is submitted and no motor-control gate is bypassed at runtime.
 */

/** @brief Reset each fixture and construct an explicitly synthetic ID7 draft. */
static void icon_scenario_draft(DebugUiOperation operation, DebugUiPage page)
{
    simulated_data();
    model.selected_motor = 6U;
    model.snapshot.motors[6].feedback_seen = 1U;
    model.snapshot.motors[6].feedback_valid = 1U;
    model.snapshot.motors[6].position_rad = debug_ui_degrees_to_radians(24.0f);
    model.draft.operation = operation;
    model.draft.target_motor_id = 7U;
    model.draft.requested_mode = operation == DEBUG_UI_OPERATION_POS_MOVE ?
        DEBUG_UI_MODE_POS_VEL : DEBUG_UI_MODE_MIT;
    model.draft.delta_rad = operation == DEBUG_UI_OPERATION_MIT_HOLD ? 0.0f :
        debug_ui_degrees_to_radians(-2.0f);
    model.draft.speed_rad_s = debug_ui_degrees_to_radians(1.5f);
    model.draft.kp = 3.0f;
    model.draft.kd = 0.65f;
    model.draft.hold_duration_ms = 700U;
    model.reviewed = model.draft;
    model.page = page;
    model.reason = DEBUG_UI_REASON_NONE;
    assert(diagnostics.simulated && !model.outgoing_ready && !model.stop_ready);
}

/** @brief Cover icon focus, authority, all editor fields and protection outcomes. */
static void render_icon_scenarios(const char *directory)
{
    unsigned scenario_index;
    char filename[80];
    static const DebugUiOperation simple_operations[] = {
        DEBUG_UI_OPERATION_ACQUIRE, DEBUG_UI_OPERATION_RELEASE,
        DEBUG_UI_OPERATION_CLEAR_FAULT, DEBUG_UI_OPERATION_DISABLE
    };
    static const char *const simple_names[] = {
        "acquire", "release", "clear_fault", "disable"
    };
    static const DebugUiReason protection_reasons[] = {
        DEBUG_UI_REASON_TORQUE_LIMIT, DEBUG_UI_REASON_SPEED_LIMIT,
        DEBUG_UI_REASON_TEMPERATURE_LIMIT
    };
    assert(directory != NULL);

    /* 26-33: Every tile receives focus, including the second action-grid page. */
    for (scenario_index = 0U; scenario_index < 8U; ++scenario_index) {
        simulated_data();
        model.selected_motor = 6U;
        model.page = DEBUG_UI_PAGE_ACTIONS;
        model.focus = (uint8_t)scenario_index;
        (void)snprintf(filename, sizeof(filename), "%02u_actions_focus_%u",
                       26U + scenario_index, scenario_index);
        save_page(directory, filename);
    }

    /* 34: Remote ownership presents the local-control acquisition affordance. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_OVERVIEW;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    model.focus = 2U;
    save_page(directory, "34_home_remote");

    /* 35: Local release is highlighted instead of duplicating baseline home. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_OVERVIEW;
    model.focus = 2U;
    save_page(directory, "35_home_local_release");

    /* 36: Runtime read-only state must offer control checks with no motion. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_OVERVIEW;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    model.snapshot.motion_enabled = 0U;
    model.snapshot.bench_profile = 0U;
    model.focus = 2U;
    save_page(directory, "36_home_readonly");

    /* 37: No identity or feedback evidence must visibly remain unknown. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_OVERVIEW;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    model.snapshot.usb_connected = 0U;
    memset(model.snapshot.motors, 0, sizeof(model.snapshot.motors));
    save_page(directory, "37_home_no_data");

    /* 38: POS speed uses the enlarged numeric editor and explicit angle units. */
    icon_scenario_draft(DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_PAGE_EDIT);
    model.edit_field = 1U;
    save_page(directory, "38_pos_speed_edit");

    /* 39: POS review freezes ID7's target with a distinct partial hold bar. */
    icon_scenario_draft(DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_PAGE_REVIEW);
    model.review_released = 1U;
    model.confirm_elapsed_ms = 300U;
    save_page(directory, "39_pos_confirm");

    /* 40-42: Kp, Kd and duration each render their own label/value/unit. */
    for (scenario_index = 2U; scenario_index <= 4U; ++scenario_index) {
        icon_scenario_draft(DEBUG_UI_OPERATION_MIT_MOVE, DEBUG_UI_PAGE_EDIT);
        model.edit_field = (uint8_t)scenario_index;
        (void)snprintf(filename, sizeof(filename), "%02u_mit_field_%u",
                       38U + scenario_index, scenario_index);
        save_page(directory, filename);
    }

    /* 43: MIT HOLD exposes its gain editor without a displacement request. */
    icon_scenario_draft(DEBUG_UI_OPERATION_MIT_HOLD, DEBUG_UI_PAGE_EDIT);
    model.edit_field = 2U;
    save_page(directory, "43_mit_hold_edit");

    /* 44: MIT HOLD review displays duration, gains and release-before-confirm. */
    icon_scenario_draft(DEBUG_UI_OPERATION_MIT_HOLD, DEBUG_UI_PAGE_REVIEW);
    save_page(directory, "44_mit_hold_review");

    /* 45-52: Each non-motion operation has separate draft/review wording.
     * Synthetic requests intentionally avoid model_begin authorization gates. */
    for (scenario_index = 0U; scenario_index < 4U; ++scenario_index) {
        unsigned review_index;
        for (review_index = 0U; review_index < 2U; ++review_index) {
            icon_scenario_draft(simple_operations[scenario_index],
                review_index ? DEBUG_UI_PAGE_REVIEW : DEBUG_UI_PAGE_EDIT);
            model.draft.delta_rad = model.draft.speed_rad_s = 0.0f;
            model.draft.kp = model.draft.kd = 0.0f;
            model.draft.hold_duration_ms = 0U;
            model.reviewed = model.draft;
            if (simple_operations[scenario_index] == DEBUG_UI_OPERATION_ACQUIRE)
                model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
            if (simple_operations[scenario_index] == DEBUG_UI_OPERATION_CLEAR_FAULT)
                model.snapshot.motors[6].fault_flags = 6U;
            model.review_released = (uint8_t)review_index;
            model.confirm_elapsed_ms = review_index ? 450U : 0U;
            (void)snprintf(filename, sizeof(filename), "%02u_%s_%s",
                           45U + scenario_index * 2U + review_index,
                           simple_names[scenario_index], review_index ? "review" : "edit");
            save_page(directory, filename);
        }
    }

    /* 53: Cancellation remains distinct from success after an epoch change. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_RESULT;
    model.completion_seen = 1U;
    model.completion.code = DEBUG_UI_CANCELLED;
    model.completion.stage = DEBUG_UI_STAGE_VALIDATE;
    model.completion.disabled_confirmed = 1U;
    model.reason = DEBUG_UI_REASON_OLD_EPOCH;
    save_page(directory, "53_result_cancelled_epoch");

    /* 54-56: Torque, speed and temperature protection retain specific causes. */
    for (scenario_index = 0U; scenario_index < 3U; ++scenario_index) {
        simulated_data();
        model.page = DEBUG_UI_PAGE_FAULT;
        model.completion_seen = 1U;
        model.completion.code = DEBUG_UI_FAILED;
        model.completion.stage = DEBUG_UI_STAGE_DISABLE;
        model.completion.disabled_confirmed = (uint8_t)(scenario_index != 2U);
        model.completion.error = (uint8_t)(20U + scenario_index);
        model.completion.detail = (uint16_t)(101U + scenario_index);
        model.reason = protection_reasons[scenario_index];
        model.completion.reason = model.reason;
        (void)snprintf(filename, sizeof(filename), "%02u_result_protection_%u",
                       54U + scenario_index, scenario_index);
        save_page(directory, filename);
    }

    /* 57: Unconfirmed disable offers recovery STOP rather than acquisition. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_PREPARE;
    model.selected_motor = 6U;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_FAULT;
    model.snapshot.unconfirmed_disable_mask = 0x40U;
    model.snapshot.motors[6].feedback_seen = 1U;
    model.snapshot.motors[6].feedback_valid = 0U;
    model.snapshot.motors[6].feedback_age_ms = 850U;
    save_page(directory, "57_prepare_blocked_disable");

    /* 58: Fresh disabled feedback and cleared latch expose recheck recovery. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_PREPARE;
    model.selected_motor = 6U;
    model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    model.snapshot.motors[6].feedback_seen = 1U;
    model.snapshot.motors[6].feedback_valid = 1U;
    model.snapshot.motors[6].feedback_age_ms = 8U;
    model.snapshot.motors[3].fault_flags = 0U;
    save_page(directory, "58_prepare_recovery_ready");

    /* 59: The first five-motor window complements the existing scrolled window. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_MOTORS;
    model.selected_motor = 0U;
    model.list_first = 0U;
    save_page(directory, "59_motors_first_window");

    /* 60: ID2 has verified MIT mode and fresh feedback in the base fixture. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_DETAIL;
    model.selected_motor = 1U;
    assert(model.snapshot.motors[1].actual_mode == DEBUG_UI_MODE_MIT);
    assert(model.snapshot.motors[1].feedback_valid && model.snapshot.motors[1].mode_verified);
    save_page(directory, "60_mit_motor_detail");

    /* 61: Actual review validation rejects a relative target outside calibrated bounds. */
    {
        DebugUiInputEvent review_event;
        DebugUiRequest unexpected_request;
        simulated_data();
        assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
        model.snapshot.motors[0].position_rad = model.snapshot.motors[0].profile.position_max_rad -
            debug_ui_degrees_to_radians(0.5f);
        memset(&review_event, 0, sizeof(review_event));
        review_event.type = DEBUG_UI_INPUT_EVENT_PRESS;
        review_event.key = DEBUG_UI_KEY_CENTER;
        review_event.timestamp_ms = 1000U;
        debug_ui_model_event(&model, &review_event);
        assert(model.page == DEBUG_UI_PAGE_EDIT && model.reason == DEBUG_UI_REASON_OUT_OF_RANGE);
        assert(!debug_ui_model_take_request(&model, &unexpected_request));
        assert(!model.awaiting_result && !model.outgoing_ready && !model.stop_ready);
        save_page(directory, "61_pos_target_out_of_range");
    }

    /* 62: Running evidence belongs to an actual MIT-mode target, not merely a MIT draft. */
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
    assert(model.snapshot.motors[1].actual_mode == DEBUG_UI_MODE_MIT);
    assert(!model.outgoing_ready && !model.stop_ready);
    save_page(directory, "62_mit_running");

    /* 63: A STOPPED terminal with confirmed disable is distinct from a result-less latch. */
    simulated_data();
    model.page = DEBUG_UI_PAGE_RESULT;
    model.completion_seen = 1U;
    model.completion.operation = DEBUG_UI_OPERATION_STOP;
    model.completion.code = DEBUG_UI_STOPPED;
    model.completion.stage = DEBUG_UI_STAGE_DISABLE;
    model.completion.disabled_confirmed = 1U;
    model.reason = DEBUG_UI_REASON_NONE;
    assert(model.stop_latch_state == DEBUG_UI_STOP_LATCH_NONE);
    save_page(directory, "63_result_stopped");
    assert(strstr(view.page.row_text[0], "动作已停止") != NULL);
    assert(strstr(view.page.row_text[3], "反馈已确认失能") != NULL);
}
