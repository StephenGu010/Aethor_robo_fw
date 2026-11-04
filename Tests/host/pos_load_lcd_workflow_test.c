/** @file pos_load_lcd_workflow_test.c
 * @brief Exercise actual LCD confirmation through mailbox, typed protocol and continuous POS control.
 * All feedback, CAN completions and clock values are synthetic; no hardware is accessed.
 */
#define POS_LOAD_FIXTURE_ONLY (1)
#include "pos_load_app_test.c"
#include "../../App/DebugUi/debug_ui_model.h"
#include "../../App/DebugUi/debug_ui_pos_load_units.h"

/** @brief Advance UI health while preserving the single, subsequently stale disabled motor sample. */
static void workflow_key(DebugUiModel *model, DebugUiInputEventType type, DebugUiKey key,
    uint64_t timestamp_us, uint32_t held_ms)
{
    DebugUiSnapshot snapshot;
    DebugUiInputEvent event;
    test_health(timestamp_us, true);
    assert(aethor_app_debug_ui_get_snapshot(timestamp_us, &snapshot));
    debug_ui_model_update(model, &snapshot, timestamp_us, 1U, 1U);
    memset(&event, 0, sizeof(event));
    event.type = type;
    event.key = key;
    event.timestamp_ms = (uint32_t)(timestamp_us / 1000ULL);
    event.held_ms = held_ms;
    debug_ui_model_event(model, &event);
}

/** @brief One real 800ms confirmation must reach production preflight despite expired display feedback. */
static void workflow_confirm_to_hold(void)
{
    DebugUiModel model;
    DebugUiSnapshot ui_snapshot;
    DebugUiRequest request;
    DebugUiAdmission admission;
    ProtocolCommand command;
    MotorFeedbackSnapshot motor_snapshot;
    uint64_t timestamp_us;
    unsigned phase;
    test_fixture();
    test_acquire();
    debug_ui_model_init(&model);
    model.selected_motor = 0U;
    model.next_request_id = 100U;
    model.page = DEBUG_UI_PAGE_STATIC_HUB;
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 2000U, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 3000U, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 4000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 5000U, 0U);
    model.focus = 5U;
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 10000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 20000U, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 30000U, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 829000U, 799U);
    assert(!model.outgoing_ready);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 830000U, 800U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST && request.requested_mode == DEBUG_UI_MODE_POS_VEL);
    assert(!model.snapshot.motors[0].feedback_valid && model.snapshot.motors[0].feedback_age_ms >= 800U);
    admission = test_admission(request, 830000U);
    assert(admission.accepted);
    debug_ui_model_admission(&model, &admission);
    assert(protocol_engine_pop_command(&application_protocol_engine, &command));
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 830000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &motor_snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_start_one_shot_move(&command, &motor_snapshot, 830001U));
    pos_load_drain(830002U, 1U);
    test_feedback(831000U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    test_health(831000U, true);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 831000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &motor_snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(&motor_snapshot, 831001U));
    assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD);
    for (phase = DEBUG_UI_STATIC_TX_PRELOAD; phase <= DEBUG_UI_STATIC_TX_FIRST_HOLD; ++phase) {
        DebugUiStaticTxIdentity identity;
        CanFrame frame;
        CanTxPriority priority;
        timestamp_us = 832000ULL + phase * 100ULL;
        test_health(timestamp_us, true);
        assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.identifier == 0x101U && aethor_app_static_frame_identity(&frame, &identity));
        assert(identity.phase == phase && aethor_app_static_submitted(&identity, timestamp_us + 1ULL));
        aethor_app_static_transmitted(&identity, 1U, 100U, timestamp_us + 1ULL);
        if (phase == DEBUG_UI_STATIC_TX_ENABLE) continue;
        startup_test_feedback(1U, phase == DEBUG_UI_STATIC_TX_PRELOAD ?
            S3519_DRIVER_STATE_DISABLED : S3519_DRIVER_STATE_ENABLED, timestamp_us + 2ULL, &identity);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us + 3ULL,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &motor_snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_advance_one_shot_setup(&motor_snapshot, timestamp_us + 3ULL));
    }
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
    pos_load_drain(833000U, 0U);
    for (timestamp_us = 840000U; timestamp_us <= 1110000U; timestamp_us += 10000U)
        pos_load_step(&motor_snapshot, timestamp_us, application_action.target_position_rad[0]);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
    assert(aethor_app_debug_ui_get_snapshot(timestamp_us - 10000U, &ui_snapshot));
    debug_ui_model_update(&model, &ui_snapshot, timestamp_us - 10000U, 1U, 1U);
    assert(model.snapshot.pos_load.identity.request_id == request.identity.request_id);
    assert(model.snapshot.pos_load.stage == DEBUG_UI_STAGE_HOLD);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, timestamp_us, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, timestamp_us + 1000U, 0U);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP);
    assert(!debug_ui_model_take_request(&model, &request));
    puts("POS_LOAD_LCD_TO_FACADE_PASS stale830ms=1 one_confirmation=1 real_mailbox_protocol=1 mode2=1 continuous_hold=1 stop=1");
}

/** @brief Use actual ownership epoch rotation with both snapshot/receipt delivery orders. */
static void workflow_control_check_epoch(void)
{
    unsigned delivery_case;
    for (delivery_case = 0U; delivery_case < 4U; ++delivery_case) {
        DebugUiModel model;
        DebugUiSnapshot snapshot;
        DebugUiRequest request, parameters;
        DebugUiCompletion completion;
        DebugUiAdmission admission;
        test_fixture();
        if (delivery_case >= 2U) application_debug_ui.epoch = UINT32_MAX;
        debug_ui_model_init(&model);
        model.next_request_id = 100U;
        model.page = DEBUG_UI_PAGE_STATIC_HUB;
        model.focus = 2U;
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 4000U, 0U);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 5000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP);
        model.draft.hold_duration_ms = 7000U;
        parameters = model.draft;
        model.focus = 6U;
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 6000U, 0U);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 10000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_REVIEW && model.reviewed.operation == DEBUG_UI_OPERATION_ACQUIRE);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 20000U, 0U);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 30000U, 0U);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 830000U, 800U);
        assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_ACQUIRE);
        admission = test_admission(request, 830000U);
        assert(admission.accepted);
        debug_ui_model_admission(&model, &admission);
        assert(aethor_app_debug_ui_poll_completion(&completion) && completion.code == DEBUG_UI_COMPLETED);
        assert(aethor_app_debug_ui_get_snapshot(830000U, &snapshot));
        assert(snapshot.epoch != request.identity.epoch && snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
        if (delivery_case & 1U) debug_ui_model_update(&model, &snapshot, 830000U, 1U, 1U);
        debug_ui_model_completion(&model, &completion);
        assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP && !memcmp(&parameters, &model.draft, sizeof(parameters)));
        if (!(delivery_case & 1U)) debug_ui_model_update(&model, &snapshot, 830000U, 1U, 1U);
        assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP && model.snapshot.epoch == snapshot.epoch);
        assert(!model.awaiting_result && !model.outgoing_ready && !model.stop_ready);
        assert(application_action.state == AETHOR_APP_ACTION_IDLE);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 840000U, 0U);
        model.focus = 5U;
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 850000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_REVIEW && model.reviewed.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 860000U, 0U);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 870000U, 0U);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 1670000U, 800U);
        assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST);
        assert(request.identity.epoch == snapshot.epoch && request.hold_duration_ms == parameters.hold_duration_ms);
        assert(application_action.state == AETHOR_APP_ACTION_IDLE);
    }
    puts("POS_CONTROL_REAL_EPOCH_PASS actual_facade_acquire=1 snapshot_receipt_orders=2 uint32_wrap=1 parameters_preserved=1 subsequent_pos_request=1 no_auto_motion=1");
}

/** @brief Failed cleanup must lead to a usable recovery page without discarding the lock or starting motion. */
static void workflow_cleanup_lock_navigation(void)
{
    unsigned lock_case;
    for (lock_case = 0U; lock_case < 3U; ++lock_case) {
        DebugUiModel model;
        DebugUiRequest request;
        test_fixture();
        application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_FAULT;
        application_debug_ui.target_motor_id = 1U;
        application_debug_ui.unconfirmed_disable_mask = lock_case == 0U ? 1U : 0U;
        debug_ui_model_init(&model);
        model.selected_motor = 6U; /* Browsing another motor must never replace the retained cleanup target. */
        model.draft.delta_rad = 0.0123F;
        if (lock_case == 1U) {
            model.stop_latch_state = DEBUG_UI_STOP_LATCH_UNCONFIRMED;
            model.stop_motor_mask = 4U;
            model.completion.motor_mask = 1U; /* An older result must not replace this unresolved STOP target. */
        }
        if (lock_case == 2U) {
            model.completion_seen = 1U;
            model.completion.operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
            model.completion.code = DEBUG_UI_FAILED;
            model.completion.disabled_confirmed = 0U;
            model.completion.motor_mask = 4U;
            model.stop_motor_mask = 1U; /* An earlier confirmed STOP must not replace this failed trial target. */
        }
        model.page = DEBUG_UI_PAGE_STATIC_HUB;
        model.focus = 2U;
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 4000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_PREPARE);
        assert(model.selected_motor == (lock_case ? 2U : 0U));
        assert(debug_ui_model_result_requires_check(&model) && model.draft.delta_rad == 0.0123F);
        assert(!debug_ui_model_take_request(&model, &request));
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 5000U, 0U);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 6000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_STATIC_HUB);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 7000U, 0U);
        assert(model.page == DEBUG_UI_PAGE_PREPARE && !model.outgoing_ready && !model.stop_ready);
        model.page = DEBUG_UI_PAGE_POS_LOAD_NUMBER;
        model.edit_field = 1U;
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 7500U, 0U);
        assert(model.page == DEBUG_UI_PAGE_PREPARE && model.draft.delta_rad == 0.0123F);
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 8000U, 0U);
        assert(debug_ui_model_take_request(&model, &request));
        assert(request.operation == DEBUG_UI_OPERATION_STOP && request.target_motor_id == (lock_case ? 3U : 1U));
        assert(debug_ui_model_result_requires_check(&model));
        assert(!debug_ui_model_take_request(&model, &request));
    }
    puts("POS_CLEANUP_LOCK_NAVIGATION_PASS lock_sources=3 mid_edit_lock=1 failed_entry_to_check=1 left_returns=1 explicit_original_axis_stop_only=1 lock_preserved=1");
}

/** @brief Install only synthetic ID7 driver evidence; production never receives these values. */
static void flange_fixture(float maximum, float start)
{
    MotorJointFeedback feedback;
    test_fixture();
    application_motor_runtime.discovery.results[6] = application_motor_runtime.discovery.results[0];
    application_motor_runtime.discovery.results[6].observed_esc_id = 7U;
    application_motor_runtime.discovery.results[6].observed_master_id =
        application_motor_runtime.configuration->joints[6].master_id;
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = maximum;
    application_motor_runtime.discovery.verified_joint_mask |= 0x40U;
    feedback = application_motor_runtime.bank.motors[0].feedback;
    feedback.position_rad = start;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        application_motor_runtime.configuration->joints[6].master_id, 7U, &feedback) == MOTOR_BANK_STATUS_OK);
}

/** @brief Acquire the actual selected ID7 through production admission rather than reusing ID1 authority. */
static void flange_acquire(void)
{
    DebugUiSnapshot snapshot;
    DebugUiCompletion completion;
    DebugUiRequest request = test_request(DEBUG_UI_OPERATION_ACQUIRE, 1000U);
    DebugUiAdmission admission;
    assert(aethor_app_debug_ui_get_snapshot(1000U, &snapshot));
    request.target_motor_id = 7U;
    request.reference_generation = snapshot.motors[6].reference_generation;
    admission = test_admission(request, 1000U);
    assert(admission.accepted);
    assert(aethor_app_debug_ui_poll_completion(&completion) && completion.code == DEBUG_UI_COMPLETED);
}

/** @brief Verify proposed wide encoding quantization using the production feedback decoder and POS packer. */
static void flange_wide_codec(void)
{
    S3519Ranges ranges = {256.0F, 30.0F, 10.0F};
    unsigned direction;
    for (direction = 0U; direction < 2U; ++direction) {
        float target = 10.0F + (direction ? -1.0F : 1.0F) * 3.14159265358979323846F * 44.8F;
        uint32_t position_code = (uint32_t)lroundf((target + 256.0F) * 65535.0F / 512.0F);
        CanFrame frame = {0};
        S3519Feedback decoded;
        float wire_position, wire_speed;
        assert(s3519_pack_position_velocity(7U, target,
            debug_ui_degrees_to_radians(1.0F) * (44.8F / 19.2032F), &frame) == S3519_CODEC_STATUS_OK);
        assert(frame.identifier == 0x107U);
        memcpy(&wire_position, frame.data, sizeof(wire_position));
        memcpy(&wire_speed, frame.data + 4U, sizeof(wire_speed));
        assert(wire_position == target && wire_speed > 0.0407F && wire_speed < 0.0408F);
        frame.data[0] = 7U;
        frame.data[1] = (uint8_t)(position_code >> 8U); frame.data[2] = (uint8_t)position_code;
        frame.data[3] = 0x7FU; frame.data[4] = 0xF7U; frame.data[5] = 0xFFU;
        frame.data[6] = frame.data[7] = 30U;
        assert(s3519_decode_feedback(&frame, &ranges, &decoded) == S3519_CODEC_STATUS_OK);
        assert(fabsf(decoded.position_rad - target) <= 512.0F / 65535.0F);
        assert(debug_ui_radians_to_degrees(fabsf(decoded.position_rad - target) / 44.8F) < 0.0101F);
    }
    puts("POS_FLANGE_WIDE_CODEC_PASS synthetic_pmax256=1 signed_native_targets=2 flange_quantum_lt_0_0101deg=1 no_register_writes=1");
}

/** @brief Actual App snapshots drive signed flange edits, explicit review and distinct wire units. */
static void flange_conversion(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot;
    DebugUiRequest request;
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    float factor = 44.8F / 19.2032F;
    float expected = debug_ui_degrees_to_radians(1.0F) * factor;
    unsigned index;
    flange_fixture(256.0F, 10.0F);
    flange_acquire();
    debug_ui_model_init(&model);
    model.selected_motor = 6U; model.page = DEBUG_UI_PAGE_STATIC_HUB; model.focus = 2U;
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 4000ULL, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 4001ULL, 0U);
    assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP);
    assert(fabsf(model.draft.delta_rad - expected) < 0.000001F);
    assert(fabsf(model.draft.speed_rad_s - expected) < 0.000001F);
    assert(fabsf(model.snapshot.motors[6].profile.position_max_rad - 12.5F / 19.2032F) < 0.000001F);
    assert(model.snapshot.motors[6].profile.mit_position_max_rad == model.snapshot.motors[6].profile.position_max_rad);
    assert(model.snapshot.motors[6].profile.mit_max_delta_rad == model.snapshot.motors[6].profile.max_delta_rad);
    assert(model.snapshot.motors[0].pos_load_output_per_flange == 0.0F);
    assert(model.snapshot.motors[0].profile.max_delta_rad == 0.1F);
    model.focus = 1U;
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 5000ULL, 0U);
    assert(fabsf(model.draft.delta_rad) < 0.000001F);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 6000ULL, 0U);
    assert(fabsf(model.draft.delta_rad + expected) < 0.000001F);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 7000ULL, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 7001ULL, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 8000ULL, 0U);
    assert(fabsf(model.draft.delta_rad + 2.0F * expected) < 0.000001F);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 9000ULL, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_LEFT, 809000ULL, 800U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_LEFT, 810000ULL, 801U);
    assert(fabsf(model.draft.delta_rad + expected) < 0.000001F);
    for (index = 0U; index < 400U; ++index)
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_LEFT, 10000ULL + index, 0U);
    assert(fabsf(debug_ui_pos_load_display(&model.snapshot.motors[6], model.draft.delta_rad) +
        3.14159265358979323846F) < 0.000002F);
    for (index = 0U; index < 400U; ++index)
        workflow_key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_RIGHT, 11000ULL + index, 0U);
    assert(fabsf(debug_ui_pos_load_display(&model.snapshot.motors[6], model.draft.delta_rad) -
        3.14159265358979323846F) < 0.000002F);
    model.focus = 5U;
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 12000ULL, 0U);
    if (model.page != DEBUG_UI_PAGE_REVIEW) fprintf(stderr, "FLANGE_REVIEW page=%u reason=%u authority=%u owner=%u center=%u delta=%.9g max=%.9g\n",
        model.page, model.reason, model.snapshot.authority, model.snapshot.target_motor_id, model.center_down,
        (double)model.draft.delta_rad, (double)debug_ui_pos_load_bounds(&model.snapshot.motors[6]).max_delta_rad);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    request = model.reviewed;
    request.identity = test_request(DEBUG_UI_OPERATION_POS_LOAD_TEST, 12000ULL).identity;
    request.created_at_us = 12000ULL;
    aethor_app_debug_ui_build_command(&request, 12000ULL, &command);
    assert(fabsf(command.values[6] - 3.14159265358979323846F * 44.8F) < 0.0001F);
    assert(fabsf(command.speeds[6] - expected) < 0.000001F);
    application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    application_debug_ui.target_motor_id = 7U;
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 12000ULL,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback) == MOTOR_RUNTIME_STATUS_OK);
    {
        DebugUiReason reason = aethor_app_debug_ui_validate_start(&command, &feedback, 12000ULL, true);
        if (reason != DEBUG_UI_REASON_NONE) fprintf(stderr, "FLANGE_BACKEND reason=%u native=%.9g speed=%.9g\n", reason, (double)command.values[6], (double)command.speeds[6]);
        assert(reason == DEBUG_UI_REASON_NONE);
    }
    /* PMAX changes above ordinary-page clamps must still invalidate a reviewed request. */
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = 300.0F;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 12000ULL, true) == DEBUG_UI_REASON_OLD_EPOCH);
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = 256.0F;
    assert(aethor_app_debug_ui_get_snapshot(12000ULL, &snapshot));
    command.reference_generation = snapshot.motors[6].reference_generation;
    command.values[6] = -command.values[6];
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 12000ULL, true) == DEBUG_UI_REASON_NONE);
    command.values[6] = debug_ui_degrees_to_radians(180.01F) * 44.8F;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 12000ULL, true) == DEBUG_UI_REASON_OUT_OF_RANGE);
    command.values[6] = debug_ui_degrees_to_radians(180.0F) * 44.8F;
    feedback.joints[6].position_rad = 200.0F;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 12000ULL, true) == DEBUG_UI_REASON_OUT_OF_RANGE);
    /* Actual wide trial guard must use its separately frozen profile, while physical thresholds stay unchanged. */
    application_action.command = command;
    application_action.target_position_rad[6] = command.values[6];
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT;
    aethor_app_pos_load_begin(12000ULL);
    feedback.joints[6].driver_state = S3519_DRIVER_STATE_ENABLED;
    feedback.joints[6].position_rad = 150.0F;
    assert(aethor_app_pos_load_guard(&feedback, 6U, 12000ULL) == DEBUG_UI_REASON_NONE);
    feedback.joints[6].position_rad = 256.01F;
    assert(aethor_app_pos_load_guard(&feedback, 6U, 12000ULL) == DEBUG_UI_REASON_OUT_OF_RANGE);
    feedback.joints[6].position_rad = 0.0F;
    feedback.joints[6].driver_state = S3519_DRIVER_STATE_DISABLED;
    memset(&application_action, 0, sizeof(application_action));
    /* The same driver encoding must not enlarge ordinary motor-page motion. */
    command.local_operation = DEBUG_UI_OPERATION_POS_MOVE;
    command.hold_duration_us = 0U;
    feedback.joints[6].position_rad = 0.0F;
    command.values[6] = 26.0F;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 12000ULL, true) == DEBUG_UI_REASON_OUT_OF_RANGE);
    /* Large flange requests stay editable but cannot reach review on the original driver range. */
    flange_fixture(12.5F, 0.0F);
    flange_acquire();
    debug_ui_model_init(&model);
    model.selected_motor = 6U; model.page = DEBUG_UI_PAGE_STATIC_HUB; model.focus = 2U;
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 4000ULL, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 4001ULL, 0U);
    model.draft.delta_rad = debug_ui_pos_load_edit_limit(&model.snapshot.motors[6]);
    model.focus = 5U;
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 5000ULL, 0U);
    assert(model.page == DEBUG_UI_PAGE_NOTICE && model.reason == DEBUG_UI_REASON_OUT_OF_RANGE);
    assert(!model.outgoing_ready);
    assert(aethor_app_debug_ui_get_snapshot(5000ULL, &snapshot));
    command = (ProtocolCommand){0};
    request = test_request(DEBUG_UI_OPERATION_POS_LOAD_TEST, 5000ULL);
    request.target_motor_id = 7U; request.reference_generation = snapshot.motors[6].reference_generation;
    request.delta_rad = 3.14159265358979323846F * factor;
    request.speed_rad_s = expected; request.kp = request.kd = 0.0F; request.hold_duration_ms = 1000U;
    aethor_app_debug_ui_build_command(&request, 5000ULL, &command);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 5000ULL,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 5000ULL, true) == DEBUG_UI_REASON_OUT_OF_RANGE);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 5001ULL, 0U);
    model.page = DEBUG_UI_PAGE_POS_LOAD_SETUP;
    model.snapshot.motors[6].pos_load_output_per_flange = 0.0F;
    model.focus = 5U;
    {
        DebugUiInputEvent event = {0};
        event.type = DEBUG_UI_INPUT_EVENT_PRESS; event.key = DEBUG_UI_KEY_CENTER;
        debug_ui_model_event(&model, &event);
    }
    assert(model.page == DEBUG_UI_PAGE_NOTICE && model.reason == DEBUG_UI_REASON_UNCALIBRATED);
    puts("POS_FLANGE_BOUNDARIES_PASS default1deg=1 speed_distinct=1 signed180=1 fine_cancel=1 wide_driver_only=1 forged_rejected=1 ordinary_preserved=1 missing_ratio_locked=1");
}

/** @brief Drain real ID7 POS frames and produce a strict receipt only for terminal DISABLE. */
static void flange_lifecycle_drain(uint64_t now, uint8_t disabled)
{
    CanFrame frame;
    CanTxPriority priority;
    while (application_action.frame_read_index < application_action.frames.count) {
        DebugUiStaticTxIdentity identity;
        assert(aethor_app_next_can_frame(now, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.identifier == 7U || frame.identifier == 0x107U);
        if (disabled) {
            assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE);
            if (aethor_app_static_frame_identity(&frame, &identity)) {
                aethor_app_static_can_queued(&frame, 1U, 1U, now);
                assert(aethor_app_static_submitted(&identity, now));
                aethor_app_static_transmitted(&identity, 1U, 100U, now);
            }
        } else {
            float speed;
            memcpy(&speed, frame.data + 4U, sizeof(speed));
            assert(isfinite(speed) && speed > 0.0F);
        }
    }
}

/** @brief Drive wide ID7 requests through actual mailbox, preflight, startup, travel, hold and terminal proof. */
static void flange_wide_lifecycle(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 5U; ++scenario) {
        DebugUiRequest request;
        DebugUiSnapshot view;
        ProtocolCommand command;
        MotorFeedbackSnapshot snapshot;
        MotorJointFeedback feedback;
        DebugUiCompletion completion;
        uint64_t now, move_start, duration;
        float initial, target;
        unsigned phase;
        flange_fixture(512.0F, scenario & 1U ? -12.726745605F : 12.726745605F);
        flange_acquire();
        assert(aethor_app_debug_ui_get_snapshot(1001U, &view));
        request = test_request(DEBUG_UI_OPERATION_POS_LOAD_TEST, 1001U);
        request.target_motor_id = 7U; request.reference_generation = view.motors[6].reference_generation;
        request.delta_rad = debug_ui_degrees_to_radians(scenario == 4U ? -51.0F :
            (scenario & 2U ? -180.0F : 180.0F)) * (44.8F / 19.2032F) * (1.0F - 4.0F * FLT_EPSILON);
        request.speed_rad_s = debug_ui_degrees_to_radians(15.0F) * (44.8F / 19.2032F);
        request.kp = request.kd = 0.0F; request.hold_duration_ms = scenario == 4U ? 20000U : 1000U;
        assert(test_admission(request, 1001U).accepted);
        assert(protocol_engine_pop_command(&application_protocol_engine, &command));
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_start_one_shot_move(&command, &snapshot, 1002U));
        flange_lifecycle_drain(1003U, 1U);
        startup_test_feedback(7U, S3519_DRIVER_STATE_DISABLED, 2000U, NULL);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 2000U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_advance_one_shot_setup(&snapshot, 2001U));
        for (phase = DEBUG_UI_STATIC_TX_PRELOAD; phase <= DEBUG_UI_STATIC_TX_FIRST_HOLD; ++phase) {
            DebugUiStaticTxIdentity identity;
            CanFrame frame;
            CanTxPriority priority;
            now = 2100ULL + phase * 100ULL;
            test_health(now, true);
            assert(aethor_app_next_can_frame(now, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
            assert(aethor_app_static_frame_identity(&frame, &identity) && identity.motor_id == 7U);
            assert(aethor_app_static_submitted(&identity, now + 1ULL));
            aethor_app_static_transmitted(&identity, 1U, 100U, now + 1ULL);
            if (phase == DEBUG_UI_STATIC_TX_ENABLE) continue;
            startup_test_feedback(7U, phase == DEBUG_UI_STATIC_TX_PRELOAD ?
                S3519_DRIVER_STATE_DISABLED : S3519_DRIVER_STATE_ENABLED, now + 2ULL, &identity);
            assert(motor_runtime_get_snapshot(&application_motor_runtime, now + 3ULL,
                MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
            assert(!aethor_app_advance_one_shot_setup(&snapshot, now + 3ULL));
        }
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
        initial = snapshot.joints[6].position_rad; target = application_action.target_position_rad[6];
        move_start = now + 10ULL;
        duration = (uint64_t)ceil(fabs((double)target - initial) / (double)request.speed_rad_s / 19.2032 * 1000000.0);
        flange_lifecycle_drain(move_start, 0U);
        for (now = move_start + 10000ULL; now < move_start + duration + 300000ULL; now += 10000ULL) {
            float progress = fminf(1.0F, (float)(now - move_start) / (float)duration);
            uint8_t failed;
            feedback = snapshot.joints[6]; feedback.timestamp_us = now;
            feedback.position_rad = initial + (target - initial) * progress;
            feedback.velocity_rad_s = progress < 1.0F ? request.speed_rad_s : 0.0F;
            assert(motor_bank_update_feedback(&application_motor_runtime.bank, 23U, 7U, &feedback) == MOTOR_BANK_STATUS_OK);
            test_health(now, true);
            assert(motor_runtime_get_snapshot(&application_motor_runtime, now,
                MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
            (void)aethor_app_check_one_shot_motor_safety(&snapshot, now, &failed);
            assert(!failed);
            (void)aethor_app_advance_one_shot_setup(&snapshot, now);
            flange_lifecycle_drain(now, 0U);
            if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT) break;
        }
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
        for (now += 10000ULL; now < application_action.deadline_us; now += 10000ULL) {
            uint8_t failed;
            feedback.timestamp_us = now; feedback.position_rad = target; feedback.velocity_rad_s = 0.0F;
            assert(motor_bank_update_feedback(&application_motor_runtime.bank, 23U, 7U, &feedback) == MOTOR_BANK_STATUS_OK);
            test_health(now, true);
            assert(motor_runtime_get_snapshot(&application_motor_runtime, now,
                MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
            (void)aethor_app_check_one_shot_motor_safety(&snapshot, now, &failed); assert(!failed);
            assert(!aethor_app_advance_one_shot_setup(&snapshot, now));
            flange_lifecycle_drain(now, 0U);
        }
        now = application_action.deadline_us;
        feedback.timestamp_us = now; feedback.position_rad = target; feedback.velocity_rad_s = 0.0F;
        assert(motor_bank_update_feedback(&application_motor_runtime.bank, 23U, 7U, &feedback) == MOTOR_BANK_STATUS_OK);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, now,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_advance_one_shot_setup(&snapshot, now));
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT);
        flange_lifecycle_drain(now + 1ULL, 1U);
        startup_test_feedback(7U, S3519_DRIVER_STATE_DISABLED, now + 2ULL, &application_debug_ui.pos_disable_identity);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, now + 2ULL,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(aethor_app_advance_one_shot_setup(&snapshot, now + 3ULL));
        aethor_app_debug_ui_route_results();
        assert(aethor_app_debug_ui_poll_completion(&completion));
        assert(completion.code == DEBUG_UI_COMPLETED && completion.disabled_confirmed);
        assert(application_debug_ui.pos_load.complete && !application_debug_ui.pos_load_cleanup_required);
    }
    puts("POS_FLANGE_FULL_LIFECYCLE_PASS scenarios=5 signed180_start_extremes=4 captured_minus51=1 real_mailbox_startup_move_hold_disable=1");
}

#include "pos_load_digit_cases.inc"

/** @brief Include backend regressions and add the real model-to-facade lifecycle boundary. */
int main(void)
{
    pos_load_digit_cases();
    flange_wide_lifecycle();
    flange_wide_codec();
    flange_conversion();
    workflow_cleanup_lock_navigation();
    workflow_control_check_epoch();
    (void)pos_load_run_suite();
    workflow_confirm_to_hold();
    return 0;
}
