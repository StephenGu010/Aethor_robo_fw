/** @file static_lcd_recovery_cases.inc
 * @brief Offline stale-feedback, exact-TX and cancellation regressions using the production workflow fixture.
 */
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
#include "../../App/DebugUi/debug_ui_single_static_tx.h"
/** @brief Freeze a real model request after a human wait and independent 800ms confirmation. */
static void recovery_prepare(StaticLcdWorkflowFixture *fixture, uint8_t collect)
{
    workflow_fixture(fixture, 7U);
    fixture->feedback_paused = 1U;
    workflow_tick(fixture, 900000U);
    workflow_key(fixture, DEBUG_UI_KEY_DOWN); workflow_key(fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(fixture);
    assert(fixture->model.single_static_preflight == 1U && fixture->model.awaiting_result);
    (void)aethor_app_service(fixture->timestamp_us);
    if (collect) workflow_collect_frames(fixture);
}
/** @brief Finish a failed transaction without retrying a DISABLE or manufacturing a feedback sample. */
static void recovery_wait(StaticLcdWorkflowFixture *fixture)
{
    unsigned tick;
    for (tick = 0U; tick < 150U; ++tick) {
        workflow_tick(fixture, 4000U);
        (void)aethor_app_service(fixture->timestamp_us);
        workflow_collect_frames(fixture);
    }
    workflow_tick(fixture, 1000U);
}
/** @brief Old completion bits, cancellation, pending-only and invalid masks cannot prove a new hardware TX. */
static void recovery_receipt_proof(void)
{
    DebugUiSingleStaticTx proof = {0};
    uint8_t succeeded = 99U;
    assert(!debug_ui_single_static_tx_arm(&proof, 3U, 3U, 0U));
    assert(debug_ui_single_static_tx_arm(&proof, 4U, 0U, 4U));
    assert(!debug_ui_single_static_tx_collect(&proof, 0U, 4U, 0U, &succeeded) && proof.armed && !proof.pending_seen);
    assert(!debug_ui_single_static_tx_collect(&proof, 4U, 4U, 0U, &succeeded) && proof.armed && proof.pending_seen);
    assert(debug_ui_single_static_tx_collect(&proof, 0U, 4U, 0U, &succeeded) && !succeeded);
    assert(debug_ui_single_static_tx_arm(&proof, 4U, 4U, 0U));
    assert(!debug_ui_single_static_tx_collect(&proof, 4U, 0U, 0U, &succeeded));
    assert(!debug_ui_single_static_tx_collect(&proof, 4U, 0U, 4U, &succeeded) && proof.armed);
    assert(debug_ui_single_static_tx_collect(&proof, 0U, 4U, 4U, &succeeded) && !succeeded);
    assert(debug_ui_single_static_tx_arm(&proof, 4U, 4U, 0U));
    assert(debug_ui_single_static_tx_collect(&proof, 0U, 4U, 0U, &succeeded) && succeeded);
    assert(!debug_ui_single_static_tx_collect(&proof, 0U, 4U, 0U, &succeeded));
    puts("STATIC_LCD_TX_RECEIPT_PROOF_PASS new_pending=1 old_txbto_rejected=1 cancellation_rejected=1 cancel_still_pending_retains_lock=1 exactly_once=1");
}
/** @brief Selected accepted generations advance independently; rejected, old and other-ID samples never advance this ID. */
static void recovery_sample_generation(void)
{
    StaticLcdWorkflowFixture fixture;
    MotorJointFeedback feedback;
    MotorObject *motor;
    uint32_t generation;
    workflow_fixture(&fixture, 7U);
    motor = &application_motor_runtime.bank.motors[6];
    generation = motor->feedback_generation;
    feedback = motor->feedback;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank, motor->master_id,
        motor->esc_id, &feedback) == MOTOR_BANK_STATUS_STALE_SAMPLE);
    assert(motor->feedback_generation == generation);
    feedback.timestamp_us++; feedback.position_rad = NAN;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank, motor->master_id,
        motor->esc_id, &feedback) == MOTOR_BANK_STATUS_INVALID_SAMPLE);
    assert(motor->feedback_generation == generation);
    feedback.position_rad = 0.2F;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        application_motor_runtime.bank.motors[0].master_id, application_motor_runtime.bank.motors[0].esc_id,
        &feedback) == MOTOR_BANK_STATUS_OK);
    assert(motor->feedback_generation == generation);
    motor->feedback_generation = UINT32_MAX;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank, motor->master_id,
        motor->esc_id, &feedback) == MOTOR_BANK_STATUS_OK);
    assert(motor->feedback_generation == 1U);
    puts("STATIC_LCD_SAMPLE_GENERATION_PASS selected_only=1 rejected_unchanged=1 other_id_unchanged=1 wrap_nonzero=1 timestamps_unchanged=1");
}
/** @brief A fast decoded reply may precede receipt polling, but old bank data and wrong receipt identities cannot authorize. */
static void recovery_fast_reply_before_receipt(void)
{
    StaticLcdWorkflowFixture fixture;
    DebugUiIdentity identity;
    recovery_prepare(&fixture, 0U);
    fixture.receipt_paused = 1U;
    workflow_collect_frames(&fixture);
    identity = fixture.model.submitted.identity;
    fixture.feedback_paused = 0U;
    workflow_tick(&fixture, 1000U);
    (void)aethor_app_service(fixture.timestamp_us);
    workflow_tick(&fixture, 1000U);
    assert(fixture.model.awaiting_result && !application_debug_ui.single_static_lcd_owned[6]);
    aethor_app_debug_ui_preflight_transmitted(identity.epoch, identity.request_id + 1U, 1U);
    (void)aethor_app_service(fixture.timestamp_us);
    assert(application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT);
    aethor_app_debug_ui_preflight_transmitted(identity.epoch, identity.request_id, 1U);
    workflow_finish_preflight(&fixture);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY);
    assert(fixture.emitted_frames == 1U && !fixture.enable_frames && !fixture.control_frames);
    recovery_prepare(&fixture, 1U);
    /* Even a timestamp equal to the actual boundary is old without a new accepted sample generation. */
    application_motor_runtime.bank.motors[6].feedback.timestamp_us =
        application_action.phase_feedback_baseline_us[6] + 1ULL;
    recovery_wait(&fixture);
    assert(!application_debug_ui.single_static_lcd_owned[6] && fixture.emitted_frames == 1U);
    puts("STATIC_LCD_FAST_REPLY_PASS reply_before_receipt=1 wrong_receipt_rejected=1 old_bank_equal_boundary_rejected=1 frames=1");
}
/** @brief A selected sample accepted while the DISABLE is still pending cannot later borrow a successful TX receipt. */
static void recovery_pending_rx_cannot_qualify(void)
{
    StaticLcdWorkflowFixture fixture;
    DebugUiIdentity identity;
    recovery_prepare(&fixture, 0U);
    fixture.receipt_paused = 1U;
    fixture.wire_tx_pending = 1U;
    workflow_collect_frames(&fixture);
    identity = fixture.model.submitted.identity;
    fixture.feedback_paused = 0U;
    workflow_tick(&fixture, 1000U);
    assert(application_motor_runtime.bank.motors[6].feedback_generation != application_action.single_static_rx_generation);
    fixture.feedback_paused = 1U;
    aethor_app_debug_ui_preflight_transmitted(identity.epoch, identity.request_id, 1U);
    recovery_wait(&fixture);
    assert(!application_debug_ui.single_static_lcd_owned[6] && !fixture.model.single_static_configured);
    assert(fixture.emitted_frames == 1U && !fixture.enable_frames && !fixture.control_frames);
    puts("STATIC_LCD_PENDING_RX_REJECT_PASS accepted_before_wire_tx_cannot_borrow_later_receipt=1 frames=1");
}
/** @brief Every unsafe or changed post-send condition fails closed with no ENABLE/HOLD and no implicit retry. */
static void recovery_rejected_feedback(void)
{
    unsigned condition;
    for (condition = 0U; condition < 15U; ++condition) {
        StaticLcdWorkflowFixture fixture;
        MotorObject *motor;
        recovery_prepare(&fixture, 1U);
        motor = &application_motor_runtime.bank.motors[6];
        if (condition >= 4U && condition <= 9U) {
            motor->feedback.timestamp_us = fixture.timestamp_us + 1000ULL;
            ++motor->feedback_generation;
            if (condition == 4U) motor->feedback.driver_state = 1U;
            if (condition == 5U) motor->feedback.fault_flags = 1U;
            if (condition == 6U) motor->feedback.velocity_rad_s = 0.051F;
            if (condition == 7U) motor->feedback.torque_nm = AETHOR_APP_LOCAL_MIT_TORQUE_TRIP_NM + 0.01F;
            if (condition == 8U) motor->feedback.mos_temperature_c = 55.1F;
            if (condition == 9U) motor->feedback.position_rad = NAN;
        }
        if (condition == 1U) motor->feedback.timestamp_us = fixture.timestamp_us + 1000000ULL;
        if (condition == 2U) ++application_motor_runtime.bank.motors[0].feedback_generation;
        if (condition == 3U) motor->feedback.timestamp_us = application_action.phase_feedback_baseline_us[6];
        if (condition == 10U) application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
        if (condition == 11U) ++application_debug_ui.reference_generation[6];
        if (condition == 12U) ++application_debug_ui.epoch;
        if (condition == 13U) application_debug_ui.profiles[6].mit_valid = 0U;
        if (condition == 14U) application_motor_runtime.discovery.results[6].observed_esc_id = 6U;
        recovery_wait(&fixture);
        assert(!application_debug_ui.single_static_lcd_owned[6] && !fixture.model.single_static_configured);
        assert(fixture.emitted_frames == 1U && !fixture.enable_frames && !fixture.control_frames);
        assert(!fixture.model.single_static_preflight && !fixture.model.outgoing_ready);
    }
    puts("STATIC_LCD_PREFLIGHT_REJECT_PASS cases=15 no_old_future_wrong_id_invalid_fault_speed_torque_temp_changed_binding=1 frames_each=1 enable_hold=0");
}
/** @brief STOP or cancellation wins both before send and after successful hardware receipt but before continuation. */
static void recovery_stop_and_cancel(void)
{
    unsigned after_send, cancel;
    for (after_send = 0U; after_send < 2U; ++after_send)
        for (cancel = 0U; cancel < 2U; ++cancel) {
            StaticLcdWorkflowFixture fixture;
            recovery_prepare(&fixture, (uint8_t)after_send);
            workflow_key(&fixture, cancel ? DEBUG_UI_KEY_LEFT : DEBUG_UI_KEY_CENTER);
            fixture.feedback_paused = 0U;
            recovery_wait(&fixture);
            assert(!application_debug_ui.single_static_lcd_owned[6] && !fixture.model.single_static_preflight);
            assert(!fixture.enable_frames && !fixture.control_frames);
        }
    puts("STATIC_LCD_PREFLIGHT_CANCEL_PASS before_send=1 after_send=1 left_cancel=1 center_stop=1 enable_hold=0");
}
/** @brief Mailbox rejection cannot strand the visible preflight page or publish a continuation. */
static void recovery_rejected_submit(void)
{
    StaticLcdWorkflowFixture fixture;
    recovery_prepare(&fixture, 0U);
    debug_ui_model_submit_failed(&fixture.model, &fixture.model.submitted, DEBUG_UI_REASON_RESULT_BACKPRESSURE);
    assert(!fixture.model.single_static_preflight && !fixture.model.awaiting_result && !fixture.model.outgoing_ready);
    assert(fixture.model.page == DEBUG_UI_PAGE_NOTICE);
}
/** @brief Application admission rejection retains exact selected scope and retires the real mailbox/model transaction. */
static void recovery_rejected_admission(void)
{
    DebugUiReason reasons[4] = { DEBUG_UI_REASON_BUSY, DEBUG_UI_REASON_UNHEALTHY,
        DEBUG_UI_REASON_OLD_EPOCH, DEBUG_UI_REASON_RESULT_BACKPRESSURE };
    unsigned condition;
    for (condition = 0U; condition < 4U; ++condition) {
        StaticLcdWorkflowFixture fixture;
        DebugUiRequest request;
        DebugUiAdmission admission;
        DebugUiCompletion completion;
        DebugUiInputEvent event = {0};
        uint8_t terminal_seen = 0U;
        workflow_fixture(&fixture, 7U);
        fixture.feedback_paused = 1U;
        workflow_tick(&fixture, 900000U);
        workflow_key(&fixture, DEBUG_UI_KEY_DOWN); workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
        workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
        assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW);
        if (condition == 3U) {
            /* A different completed result arrives after the UI snapshot, before this confirmation. */
            DebugUiRequest retained = {0};
            DebugUiRequest taken;
            retained.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
            retained.identity.epoch = application_debug_ui.epoch;
            retained.identity.request_id = fixture.model.next_request_id++;
            retained.target_motor_id = 1U;
            retained.operation = DEBUG_UI_OPERATION_ACQUIRE;
            assert(debug_ui_mailbox_submit(&application_debug_ui.mailbox, &retained) == DEBUG_UI_REASON_NONE);
            assert(debug_ui_mailbox_take(&application_debug_ui.mailbox, false, &taken));
            memset(&admission, 0, sizeof(admission)); admission.identity = retained.identity;
            admission.operation = retained.operation; admission.accepted = 1U;
            assert(debug_ui_mailbox_admit(&application_debug_ui.mailbox, &admission));
            memset(&completion, 0, sizeof(completion)); completion.identity = retained.identity;
            completion.operation = retained.operation;
            assert(debug_ui_mailbox_complete(&application_debug_ui.mailbox, &completion));
        }
        event.key = DEBUG_UI_KEY_CENTER; event.type = DEBUG_UI_INPUT_EVENT_PRESS;
        event.timestamp_ms = (uint32_t)(fixture.timestamp_us / 1000ULL) + 1U;
        debug_ui_model_event(&fixture.model, &event);
        event.type = DEBUG_UI_INPUT_EVENT_HOLD; event.timestamp_ms += 800U; event.held_ms = 800U;
        debug_ui_model_event(&fixture.model, &event);
        assert(debug_ui_model_take_request(&fixture.model, &request));
        assert(request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT);
        assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
        if (condition == 0U) application_action.state = AETHOR_APP_ACTION_DISABLE_WAIT;
        if (condition == 1U) application_debug_ui.mailbox.health.input_valid = 0U;
        if (condition == 2U) ++application_debug_ui.epoch;
        aethor_app_debug_ui_process(fixture.timestamp_us);
        while (aethor_app_debug_ui_poll_admission(&admission)) {
            if (admission.identity.request_id == request.identity.request_id)
                assert(!admission.accepted && admission.reason == reasons[condition]);
            debug_ui_model_admission(&fixture.model, &admission);
        }
        while (aethor_app_debug_ui_poll_completion(&completion)) {
            if (completion.identity.request_id == request.identity.request_id) {
                terminal_seen = 1U;
                assert(completion.operation == request.operation && completion.reason == reasons[condition]);
                assert(completion.motor_mask == 0x40U && completion.code == DEBUG_UI_FAILED);
            }
            debug_ui_model_completion(&fixture.model, &completion);
        }
        assert(terminal_seen && !fixture.model.awaiting_result && !fixture.model.single_static_preflight && !fixture.model.outgoing_ready);
        assert(debug_ui_mailbox_result_count(&application_debug_ui.mailbox) == 0U);
        assert(!application_debug_ui.single_static_lcd_owned[6]);
        workflow_collect_frames(&fixture);
        assert(fixture.emitted_frames == 0U && !fixture.enable_frames && !fixture.control_frames);
    }
    puts("STATIC_LCD_ADMISSION_REJECTION_PASS reasons=4 exact_selected_terminal=1 real_mailbox_retired=1 model_settled=1 actuator_frames=0");
}
/** @brief Same-millisecond requests wait before output; a missing TX receipt remains locked beyond the original action timeout. */
static void recovery_send_clock_and_unresolved_tx(void)
{
    StaticLcdWorkflowFixture fixture;
    CanFrame frame;
    CanTxPriority priority;
    recovery_prepare(&fixture, 0U);
    application_action.command.created_at_us = fixture.timestamp_us;
    assert(aethor_app_next_can_frame(fixture.timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_WAITING);
    assert(application_action.frame_read_index == 0U);
    fixture.timestamp_us += 1000ULL;
    fixture.receipt_paused = 1U;
    workflow_collect_frames(&fixture);
    assert(fixture.emitted_frames == 1U);
    fixture.feedback_paused = 0U;
    recovery_wait(&fixture);
    assert(application_debug_ui.single_static_transport_mask == 0x40U);
    assert(fixture.model.snapshot.active && fixture.model.snapshot.unconfirmed_disable_mask == 0x40U);
    assert(!application_debug_ui.single_static_lcd_owned[6]);
    workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
    assert(fixture.model.page != DEBUG_UI_PAGE_STATIC_HUB);
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture.model.stop_requested || fixture.model.stop_waiting || application_debug_ui.mailbox.stop_intent_mask ||
        application_protocol_engine.active_stop_request_id || application_protocol_engine.stop_write_sequence != application_protocol_engine.stop_read_sequence);
    assert(!fixture.enable_frames && !fixture.control_frames);
    puts("STATIC_LCD_CLOCK_UNRESOLVED_PASS same_tick_waits_before_frame=1 missing_tx_receipt_remains_locked=1 browsing_cannot_bypass=1");
}
/** @brief Capture and independent start confirmation tolerate human delay, then recheck the frozen 0.5-degree envelope. */
static void recovery_capture_and_frozen_start(void)
{
    unsigned drift;
    for (drift = 0U; drift < 2U; ++drift) {
        StaticLcdWorkflowFixture fixture;
        float frozen_position;
        workflow_fixture(&fixture, 7U);
        workflow_key(&fixture, DEBUG_UI_KEY_DOWN); workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
        workflow_key(&fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(&fixture);
        workflow_key(&fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(&fixture);
        assert(fixture.model.snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
        fixture.feedback_paused = 1U;
        workflow_tick(&fixture, 900000U);
        workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
        assert(fixture.model.single_static_preflight == 3U && !fixture.model.static_locked);
        (void)aethor_app_service(fixture.timestamp_us); workflow_collect_frames(&fixture);
        fixture.feedback_paused = 0U; workflow_finish_preflight(&fixture);
        assert(fixture.model.static_locked && fixture.emitted_frames == 2U);
        frozen_position = fixture.model.draft.static_positions_rad[6];
        fixture.feedback_paused = 1U; workflow_tick(&fixture, 900000U);
        assert(fixture.model.static_locked);
        workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
        assert(fixture.model.page == DEBUG_UI_PAGE_REVIEW);
        workflow_confirm(&fixture);
        assert(fixture.model.single_static_preflight == 4U);
        assert(fixture.model.single_static_continuation.static_positions_rad[6] == frozen_position);
        (void)aethor_app_service(fixture.timestamp_us); workflow_collect_frames(&fixture);
        if (drift) {
            fixture.position_offset_rad = 0.01F;
            fixture.feedback_paused = 0U;
            workflow_tick(&fixture, 1000U); (void)aethor_app_service(fixture.timestamp_us); workflow_tick(&fixture, 1000U);
            assert(fabsf(fixture.model.snapshot.motors[6].position_rad - frozen_position) > debug_ui_degrees_to_radians(0.5F));
            assert(fixture.model.page == DEBUG_UI_PAGE_NOTICE && !fixture.model.outgoing_ready);
            assert(fixture.model.reason == DEBUG_UI_REASON_OUT_OF_RANGE);
        } else {
            fixture.feedback_paused = 0U; workflow_finish_preflight(&fixture);
            assert(fixture.model.submitted.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
            assert(fixture.model.submitted.static_positions_rad[6] == frozen_position);
        }
        /* Successful continuation retains the original HOLD executor's own independent DISABLE stage. */
        assert(fixture.emitted_frames == (drift ? 3U : 4U) && !fixture.enable_frames && !fixture.control_frames);
    }
    puts("STATIC_LCD_HUMAN_START_PASS capture_preflight=1 independent_800ms_start=1 old_ack_not_reused=1 target_frozen=1 half_degree_drift_rejected=1");
}
/** @brief Produce a real model preflight without refreshing the preceding idle snapshot after request creation. */
static void recovery_begin_completion_gap(StaticLcdWorkflowFixture *fixture, uint8_t phase)
{
    DebugUiInputEvent event = {0};
    workflow_fixture(fixture, 7U);
    workflow_key(fixture, DEBUG_UI_KEY_DOWN); workflow_key(fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER);
    if (phase != 1U) {
        workflow_confirm(fixture);
        if (phase != 2U) {
            workflow_key(fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(fixture);
            if (phase == 4U) {
                workflow_key(fixture, DEBUG_UI_KEY_CENTER);
                assert(fixture->model.static_locked);
                workflow_key(fixture, DEBUG_UI_KEY_CENTER);
                assert(fixture->model.page == DEBUG_UI_PAGE_REVIEW);
            }
        }
    }
    fixture->feedback_paused = 1U;
    workflow_tick(fixture, 900000U);
    event.key = phase == 2U ? DEBUG_UI_KEY_LEFT : DEBUG_UI_KEY_CENTER;
    event.type = DEBUG_UI_INPUT_EVENT_PRESS;
    event.timestamp_ms = (uint32_t)(fixture->timestamp_us / 1000ULL) + 1U;
    debug_ui_model_event(&fixture->model, &event);
    if (phase == 1U || phase == 4U) {
        event.type = DEBUG_UI_INPUT_EVENT_HOLD; event.timestamp_ms += 800U; event.held_ms = 800U;
        debug_ui_model_event(&fixture->model, &event);
    }
    assert(fixture->model.single_static_preflight == phase && fixture->model.awaiting_result);
    assert(!fixture->model.snapshot.active && !fixture->model.snapshot.pending && !fixture->model.snapshot.stop_pending);
}
/** @brief Deliver actual decoded selected feedback without publishing a new UI snapshot. */
static void recovery_receive_gap_feedback(StaticLcdWorkflowFixture *fixture)
{
    CanFrame encoded, reply = {0};
    const S3519Ranges *ranges = &application_motor_runtime.discovery.results[6].ranges;
    assert(s3519_pack_mit(7U, ranges, 0.26F, 0.0F, 0.0F, 0.0F, 0.0F, &encoded) == S3519_CODEC_STATUS_OK);
    reply.identifier = application_motor_runtime.configuration->joints[6].master_id;
    reply.length = 8U; reply.data[0] = 7U;
    reply.data[1] = encoded.data[0]; reply.data[2] = encoded.data[1]; reply.data[3] = encoded.data[2];
    reply.data[4] = (uint8_t)((encoded.data[3] & 0xF0U) | (encoded.data[6] & 0x0FU));
    reply.data[5] = encoded.data[7]; reply.data[6] = 30U; reply.data[7] = 31U;
    fixture->timestamp_us += 1000ULL;
    {
        DebugUiIdentity ordered = {DEBUG_UI_ORIGIN_LOCAL_UI, application_action.command.session_id,
            application_action.command.request_id};
        assert(aethor_app_debug_ui_receive_preflight_frame(&reply, fixture->timestamp_us,
            fixture->timestamp_us, &ordered) == MOTOR_RUNTIME_STATUS_OK);
    }
    test_health(fixture->timestamp_us, true);
}
/** @brief CENTER, emergency CENTER and LEFT cancel every phase after terminal delivery but before the next coherent snapshot. */
static void recovery_stop_after_terminal_before_snapshot(void)
{
    uint8_t phase, cancellation;
    for (phase = 1U; phase <= 4U; ++phase)
        for (cancellation = 0U; cancellation < 3U; ++cancellation) {
            StaticLcdWorkflowFixture fixture;
            DebugUiRequest request, stop;
            DebugUiAdmission admission;
            DebugUiCompletion completion;
            DebugUiSnapshot snapshot;
            DebugUiInputEvent event = {0};
            recovery_begin_completion_gap(&fixture, phase);
            assert(debug_ui_model_take_request(&fixture.model, &request));
            assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
            aethor_app_debug_ui_process(fixture.timestamp_us);
            while (aethor_app_debug_ui_poll_admission(&admission)) debug_ui_model_admission(&fixture.model, &admission);
            (void)aethor_app_service(fixture.timestamp_us); workflow_collect_frames(&fixture);
            recovery_receive_gap_feedback(&fixture);
            (void)aethor_app_service(fixture.timestamp_us);
            aethor_app_debug_ui_process(fixture.timestamp_us);
            while (aethor_app_debug_ui_poll_completion(&completion)) {
                assert(completion.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT &&
                    completion.code == DEBUG_UI_COMPLETED && completion.disabled_confirmed);
                debug_ui_model_completion(&fixture.model, &completion);
            }
            assert(!fixture.model.awaiting_result && fixture.model.single_static_preflight == phase);
            assert(fixture.model.snapshot.generation == fixture.model.single_static_snapshot_generation);
            assert(!fixture.model.snapshot.active && !fixture.model.snapshot.pending);
            event.key = cancellation == 2U ? DEBUG_UI_KEY_LEFT : DEBUG_UI_KEY_CENTER;
            event.type = cancellation == 1U ? DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP : DEBUG_UI_INPUT_EVENT_PRESS;
            event.timestamp_ms = (uint32_t)(fixture.timestamp_us / 1000ULL) + 1U;
            debug_ui_model_event(&fixture.model, &event);
            assert(debug_ui_model_take_request(&fixture.model, &stop) && stop.operation == DEBUG_UI_OPERATION_STOP);
            assert(!fixture.model.single_static_preflight);
            assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_NONE);
            aethor_app_debug_ui_process(fixture.timestamp_us);
            assert(aethor_app_debug_ui_get_snapshot(fixture.timestamp_us, &snapshot));
            debug_ui_model_update(&fixture.model, &snapshot, fixture.timestamp_us, 1U, 1U);
            assert(!fixture.model.single_static_preflight && !fixture.model.outgoing_ready);
            assert(application_debug_ui.single_static_lcd_owned[6] == (phase == 1U ? 0U : 1U));
            assert(application_protocol_engine.command_write_sequence == application_protocol_engine.command_read_sequence);
            assert(!fixture.enable_frames && !fixture.control_frames);
        }
    puts("STATIC_LCD_TERMINAL_SNAPSHOT_CANCEL_PASS phases=4 input_types=3 real_app_terminal_before_snapshot=1 all_continuations_cancelled=1 enable_hold=0");
}
/** @brief Reproduce SETUP CENTER's real stale ordinary-handoff intent, then cancel only that uncommitted intent with LEFT. */
static void recovery_cancel_stale_ordinary_handoff(void)
{
    unsigned guard, same_target;
    for (same_target = 0U; same_target < 2U; ++same_target)
        for (guard = 0U; guard < 11U; ++guard) {
            StaticLcdWorkflowFixture fixture;
            DebugUiSingleStaticProfile public_profile;
            DebugUiInputEvent event = {0};
            uint32_t epoch;
            uint8_t prior_target = (uint8_t)(same_target ? 7U : 1U);
            workflow_fixture(&fixture, 7U);
            memset(&public_profile, 0, sizeof(public_profile));
            public_profile.configured = 1U; public_profile.minimum_ms = 1000U; public_profile.maximum_ms = 60000U;
            assert(aethor_app_debug_ui_set_single_static_profile(1U, &public_profile));
            aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_ARMED);
            application_debug_ui.target_motor_id = prior_target;
            epoch = application_debug_ui.epoch;
            fixture.feedback_paused = 1U; workflow_tick(&fixture, 900000U);
            workflow_key(&fixture, DEBUG_UI_KEY_DOWN); workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
            workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
            assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK && fixture.model.single_static_lock_then_configure);
            assert(fixture.model.reason == DEBUG_UI_REASON_STALE_FEEDBACK && !fixture.model.outgoing_ready);
            if (guard == 1U) fixture.model.single_static_configured = 1U;
            if (guard == 2U) fixture.model.snapshot.motors[0].single_static_lcd_owned = 1U;
            if (guard == 3U) fixture.model.snapshot.active = 1U;
            if (guard == 4U) fixture.model.snapshot.unconfirmed_disable_mask = 0x40U;
            if (guard == 5U) fixture.model.snapshot.pending = 1U;
            if (guard == 6U) fixture.model.stop_latch_state = DEBUG_UI_STOP_LATCH_UNCONFIRMED;
            if (guard == 7U) fixture.model.awaiting_result = 1U;
            if (guard == 8U) fixture.model.single_static_wait_snapshot = 1U;
            if (guard == 9U) fixture.model.stop_requested = 1U;
            if (guard == 10U) fixture.model.single_static_preflight = 1U;
            event.type = DEBUG_UI_INPUT_EVENT_PRESS; event.key = DEBUG_UI_KEY_LEFT;
            event.timestamp_ms = (uint32_t)(fixture.timestamp_us / 1000ULL);
            debug_ui_model_event(&fixture.model, &event);
            if (!guard) {
                assert(fixture.model.page == DEBUG_UI_PAGE_STATIC_HUB && !fixture.model.single_static_lock_then_configure);
                assert(!fixture.model.outgoing_ready && !fixture.model.awaiting_result && !fixture.model.stop_ready);
                assert(application_debug_ui.epoch == epoch && application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
                    application_debug_ui.target_motor_id == prior_target);
                assert(!memcmp(&application_debug_ui.single_static_profiles[0], &public_profile, sizeof(public_profile)));
                assert(application_debug_ui.mailbox.last_request_id == 0U &&
                    debug_ui_mailbox_result_count(&application_debug_ui.mailbox) == 0U);
            } else assert(fixture.model.page != DEBUG_UI_PAGE_STATIC_HUB);
            workflow_collect_frames(&fixture);
            assert(!fixture.emitted_frames && !fixture.enable_frames && !fixture.control_frames);
        }
    puts("STATIC_LCD_STALE_HANDOFF_CANCEL_PASS real_setup_center_flag=1 left_cancels_intent=1 old_targets=2 authority_epoch_public_profile_preserved=1 busy_owned_cleanup_guards=10 requests_and_frames=0");
}
/** @brief Fail a real guided HOLD at the unchanged torque guard and settle its selected disabled cleanup. */
static void recovery_failed_hold(StaticLcdWorkflowFixture *fixture)
{
    unsigned budget = 0U;
    workflow_fixture(fixture, 7U);
    workflow_key(fixture, DEBUG_UI_KEY_DOWN); workflow_key(fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(fixture);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(fixture);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(fixture);
    do {
        workflow_tick(fixture, 4000U);
        (void)aethor_app_service(fixture->timestamp_us); workflow_collect_frames(fixture);
        assert(++budget < 30U);
    } while (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
    fixture->torque_nm = 4.0F;
    workflow_tick(fixture, 4000U);
    (void)aethor_app_service(fixture->timestamp_us); workflow_collect_frames(fixture);
    assert(application_action.local_failure_reason == DEBUG_UI_REASON_TORQUE_LIMIT);
    fixture->torque_nm = 0.0F;
    do {
        workflow_tick(fixture, 4000U);
        (void)aethor_app_service(fixture->timestamp_us); workflow_collect_frames(fixture);
        assert(++budget < 60U);
    } while (fixture->model.awaiting_result || application_action.state != AETHOR_APP_ACTION_IDLE);
    workflow_tick(fixture, 1000U);
    assert(fixture->model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT);
    assert(fixture->model.completion.code == DEBUG_UI_FAILED && fixture->model.completion.disabled_confirmed);
    assert(application_debug_ui.static_load.disabled_confirmed && !application_debug_ui.static_load.complete);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    assert(application_debug_ui.single_static_lcd_owned[6] && fixture->model.single_static_configured);
    assert(!fixture->model.snapshot.active && !fixture->model.snapshot.pending && !fixture->model.snapshot.stop_pending);
    assert(!fixture->model.snapshot.unconfirmed_disable_mask && !aethor_app_debug_ui_executor_busy());
}
/** @brief Confirmed failed HOLD exits only after an explicit selected DISABLE obtains new ordered evidence. */
static void recovery_fault_owned_exit(void)
{
    unsigned stop_first;
    for (stop_first = 0U; stop_first < 2U; ++stop_first) {
    StaticLcdWorkflowFixture fixture;
    DebugUiLoadTrialRecord failed_trial;
    uint32_t frames, enable_frames, control_frames;
    recovery_failed_hold(&fixture);
    assert(debug_ui_load_trials_newest(&fixture.model.load_trials, 0U));
    failed_trial = *debug_ui_load_trials_newest(&fixture.model.load_trials, 0U);
    assert(failed_trial.hold.seen && failed_trial.hold.code == DEBUG_UI_FAILED && !failed_trial.report_complete);
    if (stop_first) {
        unsigned budget = 0U;
        workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
        assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
        workflow_key(&fixture, DEBUG_UI_KEY_RIGHT);
        do {
            workflow_tick(&fixture, 4000U);
            (void)aethor_app_service(fixture.timestamp_us); workflow_collect_frames(&fixture);
            assert(++budget < 30U);
        } while (fixture.model.stop_waiting || fixture.model.stop_requested || application_action.state != AETHOR_APP_ACTION_IDLE);
        workflow_tick(&fixture, 1000U);
        assert(fixture.model.completion.operation == DEBUG_UI_OPERATION_STOP && fixture.model.completion.disabled_confirmed);
        assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT && application_debug_ui.single_static_lcd_owned[6]);
    }
    frames = fixture.emitted_frames; enable_frames = fixture.enable_frames; control_frames = fixture.control_frames;
    fixture.feedback_paused = 1U; workflow_tick(&fixture, 900000U);
    assert(fixture.model.snapshot.motors[6].feedback_age_ms >= 150U);
    workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK && !fixture.model.single_static_preflight);
    assert(fixture.emitted_frames == frames && !fixture.model.outgoing_ready);
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    fprintf(stderr, "FAULT_OWNED_EXIT_GATE authority=%u page=%u reason=%u preflight=%u owned=%u disabled=%u\n",
        fixture.model.snapshot.authority, fixture.model.page, fixture.model.reason, fixture.model.single_static_preflight,
        fixture.model.snapshot.motors[6].single_static_lcd_owned, fixture.model.completion.disabled_confirmed);
    assert(fixture.model.single_static_preflight == 2U && fixture.model.awaiting_result);
    (void)aethor_app_service(fixture.timestamp_us); workflow_collect_frames(&fixture);
    assert(fixture.emitted_frames == frames + 1U && application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    assert(application_debug_ui.single_static_lcd_owned[6]);
    fixture.feedback_paused = 0U; workflow_finish_preflight(&fixture);
    assert(fixture.model.page == DEBUG_UI_PAGE_STATIC_HUB && !fixture.model.single_static_flow);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE && !application_debug_ui.single_static_lcd_owned[6]);
    assert(!application_debug_ui.single_static_profiles[6].configured && !fixture.model.single_static_configured);
    assert(fixture.emitted_frames == frames + 1U && fixture.enable_frames == enable_frames && fixture.control_frames == control_frames);
    assert(!application_debug_ui.static_load.complete && application_debug_ui.static_load.identity.request_id == failed_trial.identity.request_id);
    assert(!memcmp(&debug_ui_load_trials_newest(&fixture.model.load_trials, 0U)->hold, &failed_trial.hold, sizeof(failed_trial.hold)));
    assert(!debug_ui_load_trials_newest(&fixture.model.load_trials, 0U)->report_complete);
    }
    puts("STATIC_LCD_FAULT_OWNED_EXIT_PASS real_failed_hold=1 confirmed_disable=1 direct_and_right_stop=2 stale_wait=900ms explicit_relock=1 selected_disable=1 ordered_fresh_rx=1 lock_frames=0 remote_hub=1 failed_trial_preserved=1");
}
/** @brief The fault exception cannot authorize another purpose, owner, generation or unresolved executor state. */
static void recovery_fault_admission_guards(void)
{
    unsigned condition;
    for (condition = 0U; condition < 24U; ++condition) {
        StaticLcdWorkflowFixture fixture;
        DebugUiRequest request = {0};
        DebugUiAdmission admission;
        DebugUiCompletion completion;
        DebugUiReason submit_reason;
        uint32_t frames, enable_frames, control_frames;
        recovery_failed_hold(&fixture);
        frames = fixture.emitted_frames; enable_frames = fixture.enable_frames; control_frames = fixture.control_frames;
        request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
        request.identity.epoch = application_debug_ui.epoch;
        request.identity.request_id = application_debug_ui.mailbox.last_request_id + 1U;
        request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT;
        request.single_static_preflight_purpose = DEBUG_UI_SINGLE_STATIC_PREFLIGHT_LOCK;
        request.target_motor_id = 7U; request.requested_mode = DEBUG_UI_MODE_MIT;
        request.created_at_us = fixture.timestamp_us;
        request.reference_generation = application_debug_ui.reference_generation[6];
        request.single_static_evidence_generation = application_debug_ui.single_static_evidence_generation[6];
        if (condition == 0U) request.single_static_preflight_purpose = DEBUG_UI_SINGLE_STATIC_PREFLIGHT_NONE;
        if (condition == 1U) request.single_static_preflight_purpose = (DebugUiSingleStaticPreflightPurpose)2U;
        if (condition == 2U) request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
        if (condition == 3U) application_debug_ui.target_motor_id = 1U;
        if (condition == 4U) application_debug_ui.single_static_lcd_owned[6] = 0U;
        if (condition == 5U) application_debug_ui.single_static_profiles[6].configured = 0U;
        if (condition == 6U) application_debug_ui.single_static_lcd_owned[0] = 1U;
        if (condition == 7U) ++request.reference_generation;
        if (condition == 8U) ++request.single_static_evidence_generation;
        if (condition == 9U) application_debug_ui.mailbox.health.input_valid = 0U;
        if (condition == 10U) application_controller.fault = 1U;
        if (condition == 11U) application_motor_runtime.bank.motors[6].feedback.fault_flags = 1U;
        if (condition == 12U) application_debug_ui.unconfirmed_disable_mask = 0x40U;
        if (condition == 13U) application_action.state = AETHOR_APP_ACTION_DISABLE_WAIT;
        if (condition == 14U) application_debug_ui.single_static_transport_mask = 0x40U;
        if (condition == 15U) application_debug_ui.mailbox.stop_intent_mask = 0x40U;
        if (condition == 16U) ++application_protocol_engine.stop_write_sequence;
        if (condition == 17U) application_debug_ui.fallback_stop_mask = 0x40U;
        if (condition == 18U) application_protocol_engine.active_stop_request_id = 900U;
        if (condition == 19U) ++request.identity.epoch;
        if (condition == 20U) ++application_protocol_engine.command_write_sequence;
        if (condition == 21U) {
            application_debug_ui.mailbox.transactions[1].used = 1U;
            application_debug_ui.mailbox.transactions[1].completion_ready = 1U;
        }
        if (condition >= 22U) {
            request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK;
            request.single_static_preflight_purpose = DEBUG_UI_SINGLE_STATIC_PREFLIGHT_NONE;
            if (condition == 22U) application_debug_ui.single_static_lcd_owned[6] = 0U;
            if (condition == 23U) application_controller.fault = 1U;
        }
        submit_reason = aethor_app_debug_ui_submit(&request);
        if (condition == 1U || condition == 2U) assert(submit_reason == DEBUG_UI_REASON_INVALID_ARGUMENT);
        else {
            assert(submit_reason == DEBUG_UI_REASON_NONE);
            aethor_app_debug_ui_process(fixture.timestamp_us);
            assert(aethor_app_debug_ui_poll_admission(&admission));
            if (admission.accepted) fprintf(stderr, "FAULT_GUARD_UNEXPECTED_ADMISSION condition=%u\n", condition);
            assert(!admission.accepted);
            assert(aethor_app_debug_ui_poll_completion(&completion) && completion.code == DEBUG_UI_FAILED);
        }
        workflow_collect_frames(&fixture);
        assert(fixture.emitted_frames == frames && fixture.enable_frames == enable_frames && fixture.control_frames == control_frames);
        assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
        assert(!application_debug_ui.static_load.complete);
    }
    puts("STATIC_LCD_FAULT_ADMISSION_GUARDS_PASS cases=24 wrong_purpose_owner_id_generation_health_fault_busy_stop_transport_result=1 frame_free_lock_scoped=1 recovery_frames=0");
}
/** @brief Model withdrawal never borrows another flow, target, health state or unresolved cleanup. */
static void recovery_fault_model_guards(void)
{
    unsigned condition;
    for (condition = 0U; condition < 18U; ++condition) {
        StaticLcdWorkflowFixture fixture;
        DebugUiInputEvent event = {0};
        recovery_failed_hold(&fixture);
        fixture.feedback_paused = 1U; workflow_tick(&fixture, 900000U);
        workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
        assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
        if (condition == 0U) fixture.model.single_static_owned_reference = 0U;
        if (condition == 1U) fixture.model.single_static_owned_evidence = 0U;
        if (condition == 2U) fixture.model.snapshot.motors[6].single_static_lcd_owned = 0U;
        if (condition == 3U) fixture.model.snapshot.motors[6].single_static_profile.configured = 0U;
        if (condition == 4U) fixture.model.snapshot.target_motor_id = 1U;
        if (condition == 5U) fixture.model.selected_motor = 0U;
        if (condition == 6U) fixture.model.snapshot.motors[0].single_static_lcd_owned = 1U;
        if (condition == 7U) fixture.model.snapshot.active = 1U;
        if (condition == 8U) fixture.model.snapshot.pending = 1U;
        if (condition == 9U) fixture.model.snapshot.stop_pending = 1U;
        if (condition == 10U) fixture.model.snapshot.unconfirmed_disable_mask = 0x40U;
        if (condition == 11U) fixture.model.stop_latch_state = DEBUG_UI_STOP_LATCH_UNCONFIRMED;
        if (condition == 12U) fixture.model.snapshot.health.input_valid = 0U;
        if (condition == 13U) fixture.model.snapshot.arm_fault = 1U;
        if (condition == 14U) fixture.model.snapshot.motors[6].fault_flags = 1U;
        if (condition == 15U) fixture.model.awaiting_result = 1U;
        if (condition == 16U) ++fixture.model.snapshot.motors[6].reference_generation;
        if (condition == 17U) fixture.model.snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_BUSY;
        event.key = DEBUG_UI_KEY_CENTER; event.type = DEBUG_UI_INPUT_EVENT_PRESS;
        event.timestamp_ms = (uint32_t)(fixture.timestamp_us / 1000ULL) + 1U;
        debug_ui_model_event(&fixture.model, &event);
        assert(!fixture.model.single_static_preflight && fixture.model.page != DEBUG_UI_PAGE_STATIC_HUB);
        assert(fixture.model.submitted.operation != DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT);
        assert(debug_ui_model_gate(&fixture.model, DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE) != DEBUG_UI_REASON_NONE);
        assert(debug_ui_model_gate(&fixture.model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) != DEBUG_UI_REASON_NONE);
    }
    puts("STATIC_LCD_FAULT_MODEL_GUARDS_PASS cases=18 same_installation_reference_required=1 stale_hold_never_rearmed=1 other_scope_busy_cleanup_fault_health_refused=1");
}
/** @brief Purpose validation is typed and local-only; tags on motion/configuration cannot impersonate recovery. */
static void recovery_fault_purpose_contract(void)
{
    ProtocolCommand command = {0};
    unsigned operation;
    command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    command.type = PROTOCOL_COMMAND_DISABLE; command.control_mode = ARM_CONTROL_MODE_MIT;
    command.local_operation = DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT;
    command.bench_relative_scope = command.values_in_radians = 1U;
    command.motor_mask = 0x40U; command.reference_generation = command.single_static_evidence_generation = 1U;
    command.single_static_preflight_purpose = DEBUG_UI_SINGLE_STATIC_PREFLIGHT_LOCK;
    assert(protocol_engine_validate_typed_command(&command) == DEBUG_UI_REASON_NONE);
    command.single_static_preflight_purpose = (DebugUiSingleStaticPreflightPurpose)2U;
    assert(protocol_engine_validate_typed_command(&command) == DEBUG_UI_REASON_INVALID_ARGUMENT);
    command.single_static_preflight_purpose = DEBUG_UI_SINGLE_STATIC_PREFLIGHT_LOCK;
    command.origin = DEBUG_UI_ORIGIN_USB;
    assert(protocol_engine_validate_typed_command(&command) == DEBUG_UI_REASON_INVALID_ARGUMENT);
    command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    for (operation = DEBUG_UI_OPERATION_ACQUIRE; operation < DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT; ++operation) {
        command.local_operation = (DebugUiOperation)operation;
        assert(protocol_engine_validate_typed_command(&command) == DEBUG_UI_REASON_INVALID_ARGUMENT);
    }
    puts("STATIC_LCD_FAULT_PURPOSE_CONTRACT_PASS local_only=1 unknown_tag_rejected=1 non_preflight_operations_rejected=13");
}
/** @brief Fault withdrawal still refuses old/missing/unsafe feedback, absent receipts and explicit cancellation. */
static void recovery_fault_evidence_guards(void)
{
    unsigned condition;
    for (condition = 0U; condition < 8U; ++condition) {
        StaticLcdWorkflowFixture fixture;
        uint32_t enable_frames, control_frames;
        recovery_failed_hold(&fixture);
        enable_frames = fixture.enable_frames; control_frames = fixture.control_frames;
        fixture.feedback_paused = 1U; workflow_tick(&fixture, 900000U);
        workflow_key(&fixture, DEBUG_UI_KEY_LEFT); workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
        assert(fixture.model.single_static_preflight == 2U);
        if (condition == 4U) fixture.receipt_paused = 1U;
        (void)aethor_app_service(fixture.timestamp_us); workflow_collect_frames(&fixture);
        if (condition == 1U) ++application_motor_runtime.bank.motors[0].feedback_generation;
        if (condition == 2U || condition == 3U || condition == 6U) {
            recovery_receive_gap_feedback(&fixture);
            if (condition == 2U) application_motor_runtime.bank.motors[6].feedback.timestamp_us = application_action.phase_feedback_baseline_us[6];
            if (condition == 3U) application_motor_runtime.bank.motors[6].feedback.fault_flags = 1U;
            if (condition == 6U) application_motor_runtime.bank.motors[6].feedback.torque_nm = NAN;
        }
        if (condition == 5U || condition == 7U)
            workflow_key(&fixture, condition == 5U ? DEBUG_UI_KEY_LEFT : DEBUG_UI_KEY_CENTER);
        recovery_wait(&fixture);
        assert(application_debug_ui.single_static_lcd_owned[6]);
        assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT && fixture.model.page != DEBUG_UI_PAGE_STATIC_HUB);
        assert(fixture.enable_frames == enable_frames && fixture.control_frames == control_frames);
        assert(!application_debug_ui.static_load.complete);
    }
    puts("STATIC_LCD_FAULT_EVIDENCE_GUARDS_PASS cases=8 missing_old_wrong_id_fault_nan_receipt_cancellation=1 no_enable_hold=1 lcd_profile_retained=1");
}
/** @brief A newer unsafe sample between ordered preflight success and frame-free LOCK admission must retain fault ownership. */
static void recovery_fault_lock_admission_gap(void)
{
    unsigned condition;
    uint32_t unsafe_app_admissions = 0U, unsafe_model_admissions = 0U;
    const DebugUiReason expected_reasons[9] = {DEBUG_UI_REASON_TORQUE_LIMIT, DEBUG_UI_REASON_TORQUE_LIMIT,
        DEBUG_UI_REASON_TEMPERATURE_LIMIT, DEBUG_UI_REASON_TEMPERATURE_LIMIT,
        DEBUG_UI_REASON_OUT_OF_RANGE, DEBUG_UI_REASON_OUT_OF_RANGE,
        DEBUG_UI_REASON_INVALID_FEEDBACK, DEBUG_UI_REASON_OLD_EPOCH, DEBUG_UI_REASON_OLD_EPOCH};
    for (condition = 0U; condition < 9U; ++condition) {
        StaticLcdWorkflowFixture fixture;
        DebugUiModel execution_model;
        DebugUiSnapshot snapshot;
        MotorObject *motor;
        MotorJointFeedback changed;
        DebugUiReason execution_reason;
        uint32_t frames, enable_frames, control_frames;
        recovery_failed_hold(&fixture);
        fixture.feedback_paused = 1U; workflow_tick(&fixture, 900000U);
        workflow_key(&fixture, DEBUG_UI_KEY_LEFT); workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
        (void)aethor_app_service(fixture.timestamp_us); workflow_collect_frames(&fixture);
        fixture.feedback_paused = 0U; workflow_tick(&fixture, 1000U);
        (void)aethor_app_service(fixture.timestamp_us);
        fixture.feedback_paused = 1U; workflow_tick(&fixture, 1000U);
        assert(fixture.model.outgoing_ready && fixture.model.outgoing.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
        assert(fixture.model.completion.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT &&
            fixture.model.completion.code == DEBUG_UI_COMPLETED && fixture.model.completion.disabled_confirmed);
        frames = fixture.emitted_frames; enable_frames = fixture.enable_frames; control_frames = fixture.control_frames;
        motor = &application_motor_runtime.bank.motors[6]; changed = motor->feedback;
        changed.timestamp_us = fixture.timestamp_us + 1ULL;
        if (condition == 0U) changed.torque_nm = 4.0F;
        if (condition == 1U) changed.torque_nm = -4.0F;
        if (condition == 2U) changed.mos_temperature_c = 56.0F;
        if (condition == 3U) changed.rotor_temperature_c = 56.0F;
        if (condition == 4U) changed.position_rad = aethor_app_single_static_bounds(6U).mit_position_max_rad + 0.1F;
        if (condition == 5U) changed.position_rad = aethor_app_single_static_bounds(6U).mit_position_min_rad - 0.1F;
        assert(motor_bank_update_feedback(&application_motor_runtime.bank, motor->master_id, motor->esc_id, &changed) == MOTOR_BANK_STATUS_OK);
        if (condition == 6U) motor->feedback.torque_nm = NAN;
        if (condition == 7U) ++application_debug_ui.reference_generation[6];
        if (condition == 8U) ++application_debug_ui.single_static_evidence_generation[6];
        assert(aethor_app_debug_ui_get_snapshot(fixture.timestamp_us + 1000ULL, &snapshot));
        execution_model = fixture.model;
        execution_model.snapshot = snapshot; execution_model.now_us = snapshot.timestamp_us;
        execution_model.awaiting_result = execution_model.outgoing_ready = 0U;
        execution_reason = debug_ui_model_gate(&execution_model, DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
        if (condition < 6U && execution_reason != expected_reasons[condition]) unsafe_model_admissions |= 1U << condition;
        workflow_tick(&fixture, 1000U);
        workflow_collect_frames(&fixture);
        if (fixture.model.completion.code != DEBUG_UI_FAILED || application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_FAULT ||
            !application_debug_ui.single_static_lcd_owned[6] || fixture.model.page == DEBUG_UI_PAGE_STATIC_HUB)
            unsafe_app_admissions |= 1U << condition;
        else assert(fixture.model.completion.reason == expected_reasons[condition]);
        fprintf(stderr, "FAULT_LOCK_ADMISSION_GAP case=%u model_reason=%u app_reason=%u authority=%u owned=%u\n",
            condition, execution_reason, fixture.model.completion.reason, application_debug_ui.authority,
            application_debug_ui.single_static_lcd_owned[6]);
        assert(fixture.emitted_frames == frames && fixture.enable_frames == enable_frames && fixture.control_frames == control_frames);
        assert(!application_debug_ui.static_load.complete);
    }
    fprintf(stderr, "FAULT_LOCK_ADMISSION_GAP_SUMMARY unsafe_app_mask=%lu unsafe_model_mask=%lu\n",
        (unsigned long)unsafe_app_admissions, (unsigned long)unsafe_model_admissions);
    assert(!unsafe_app_admissions && !unsafe_model_admissions);
    puts("STATIC_LCD_FAULT_LOCK_ADMISSION_GAP_PASS cases=9 torque_both_signs=1 mos_rotor=1 position_both_limits=1 nan_reference_evidence=1 no_frames=1 profile_fault_retained=1");
}
/** @brief Publish an actual invalid input heartbeat while retaining independent protocol progress. */
static void recovery_health_input_failure(StaticLcdWorkflowFixture *fixture)
{
    DebugUiHealth health = application_debug_ui.mailbox.health;
    health.ui_timestamp_us = fixture->timestamp_us;
    health.ui_sequence = ++test_health_sequence;
    health.input_valid = 0U;
    aethor_app_debug_ui_update_health(&health);
    (void)aethor_app_service(fixture->timestamp_us);
}

/** @brief Install the real LCD profile and acquire it through both independent human confirmations. */
static void recovery_health_owned_fixture(StaticLcdWorkflowFixture *fixture)
{
    workflow_fixture(fixture, 7U);
    workflow_key(fixture, DEBUG_UI_KEY_DOWN); workflow_key(fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(fixture);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(fixture);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    assert(application_action.state == AETHOR_APP_ACTION_IDLE && application_debug_ui.single_static_lcd_owned[6]);
}

/** @brief Recovering heartbeats keep the failed owner/profile locked until its explicit DISABLE and LOCK. */
static void recovery_health_idle_relock(void)
{
    unsigned stop_first;
    for (stop_first = 0U; stop_first < 2U; ++stop_first) {
        StaticLcdWorkflowFixture fixture;
        DebugUiCompletion old_completion;
        uint32_t epoch, frames;
        recovery_health_owned_fixture(&fixture);
        old_completion = fixture.model.completion;
        epoch = application_debug_ui.epoch;
        frames = fixture.emitted_frames;
        recovery_health_input_failure(&fixture);
        assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT && application_debug_ui.health_stop_started);
        assert(application_debug_ui.health_failure_snapshot.input_valid == 0U);
        assert(application_debug_ui.health_failure_action_state == AETHOR_APP_ACTION_IDLE);
        assert(application_debug_ui.epoch != epoch);
        workflow_tick(&fixture, 1000U); (void)aethor_app_service(fixture.timestamp_us);
        assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT && application_debug_ui.single_static_lcd_owned[6]);
        assert(application_debug_ui.single_static_profiles[6].configured && application_debug_ui.health_stop_started);
        assert(fixture.emitted_frames == frames && !fixture.enable_frames && !fixture.control_frames);
        workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
        assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK && !fixture.model.single_static_preflight);
        if (stop_first) {
            unsigned budget = 0U;
            workflow_key(&fixture, DEBUG_UI_KEY_RIGHT);
            do {
                workflow_tick(&fixture, 1000U); (void)aethor_app_service(fixture.timestamp_us);
                workflow_collect_frames(&fixture);
                assert(++budget < 30U);
            } while (fixture.model.stop_waiting || fixture.model.stop_requested || application_action.state != AETHOR_APP_ACTION_IDLE);
            workflow_tick(&fixture, 1000U);
            assert(fixture.model.completion.operation == DEBUG_UI_OPERATION_STOP && fixture.model.completion.disabled_confirmed);
            assert(fixture.model.completion.reason == DEBUG_UI_REASON_NONE);
            assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT && application_debug_ui.single_static_lcd_owned[6]);
        }
        frames = fixture.emitted_frames;
        fixture.feedback_paused = 1U; workflow_tick(&fixture, 900000U);
        if (stop_first && fixture.model.page != DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK)
            workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
        workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
        if (fixture.model.single_static_preflight != 2U || !fixture.model.awaiting_result)
            fprintf(stderr, "HEALTH_LOCK_START_FAIL stop_first=%u page=%u reason=%u phase=%u waiting=%u stop=%u/%u\n",
                stop_first, fixture.model.page, fixture.model.reason, fixture.model.single_static_preflight,
                fixture.model.awaiting_result, fixture.model.stop_requested, fixture.model.stop_waiting);
        assert(fixture.model.single_static_preflight == 2U && fixture.model.awaiting_result);
        debug_ui_model_completion(&fixture.model, &old_completion);
        assert(fixture.model.single_static_preflight == 2U && fixture.model.awaiting_result);
        assert(fixture.model.submitted.identity.epoch != old_completion.identity.epoch);
        (void)aethor_app_service(fixture.timestamp_us); workflow_collect_frames(&fixture);
        assert(fixture.emitted_frames == frames + 1U && application_debug_ui.single_static_lcd_owned[6]);
        fixture.feedback_paused = 0U; workflow_finish_preflight(&fixture);
        if (application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE)
            fprintf(stderr, "HEALTH_IDLE_RELOCK_FAIL stop_first=%u reason=%u operation=%u authority=%u owned=%u preflight=%u\n",
                stop_first, fixture.model.completion.reason, fixture.model.completion.operation,
                application_debug_ui.authority, application_debug_ui.single_static_lcd_owned[6], fixture.model.single_static_preflight);
        assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE && !application_debug_ui.single_static_lcd_owned[6]);
        assert(!application_debug_ui.single_static_profiles[6].configured && !fixture.model.single_static_configured);
        assert(fixture.emitted_frames == frames + 1U && !fixture.enable_frames && !fixture.control_frames);
        assert(application_debug_ui.health_failure_snapshot.input_valid == 0U);
    }
    puts("STATIC_LCD_HEALTH_IDLE_RELOCK_PASS real_idle_health_failure=1 no_auto_recovery=1 explicit_ordered_disable_lock=1 healthy_stop_reason_none=1 old_epoch_result_ignored=1 enable_control_save_zero=0");
}

/** @brief Start the real HOLD so health attribution is tested against its original immutable command. */
static void recovery_health_start_hold(StaticLcdWorkflowFixture *fixture)
{
    unsigned budget = 0U;
    recovery_health_owned_fixture(fixture);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER); workflow_confirm(fixture);
    do {
        workflow_tick(fixture, 4000U); (void)aethor_app_service(fixture->timestamp_us);
        workflow_collect_frames(fixture);
        assert(++budget < 30U);
    } while (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
}

/** @brief Original health failure survives recovery, while every other command identity retains its own reason. */
static void recovery_health_action_identity(void)
{
    StaticLcdWorkflowFixture fixture;
    ProtocolCommand original, unrelated;
    ProtocolCommandResult retained, result;
    unsigned condition, budget = 0U;
    recovery_health_start_hold(&fixture);
    original = application_action.command;
    recovery_health_input_failure(&fixture);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    workflow_tick(&fixture, 1000U);
    memset(&retained, 0, sizeof(retained));
    aethor_app_debug_ui_result_metadata(&retained, &original, fixture.timestamp_us);
    assert(retained.local_reason == DEBUG_UI_REASON_UNHEALTHY);
    for (condition = 0U; condition < 6U; ++condition) {
        unrelated = original;
        if (condition == 0U) unrelated.origin = DEBUG_UI_ORIGIN_USB;
        if (condition == 1U) ++unrelated.request_id;
        if (condition == 2U) unrelated.session_id = application_debug_ui.epoch;
        if (condition == 3U) unrelated.local_operation = DEBUG_UI_OPERATION_STOP;
        if (condition == 4U) unrelated.motor_mask = 1U;
        if (condition == 5U) unrelated.type = PROTOCOL_COMMAND_DISABLE;
        memset(&result, 0, sizeof(result));
        aethor_app_debug_ui_result_metadata(&result, &unrelated, fixture.timestamp_us);
        assert(result.local_reason == DEBUG_UI_REASON_NONE);
    }
    do {
        workflow_tick(&fixture, 1000U); (void)aethor_app_service(fixture.timestamp_us);
        workflow_collect_frames(&fixture);
        assert(++budget < 50U);
    } while (fixture.model.awaiting_result || application_action.state != AETHOR_APP_ACTION_IDLE);
    workflow_tick(&fixture, 1000U);
    assert(fixture.model.completion.identity.request_id == original.request_id);
    assert(fixture.model.completion.reason == DEBUG_UI_REASON_UNHEALTHY && fixture.model.completion.disabled_confirmed);
    assert(retained.local_reason == DEBUG_UI_REASON_UNHEALTHY);
    assert(application_debug_ui.health_failure_snapshot.input_valid == 0U && application_debug_ui.single_static_lcd_owned[6]);
    puts("STATIC_LCD_HEALTH_ACTION_IDENTITY_PASS real_hold_aborted_unhealthy=1 recovered_heartbeat_keeps_cause=1 wrong_origin_request_epoch_operation_axis_type_rejected=6 retained_result_immutable=1");
}

/** @brief A later health fault during cleanup cannot replace the selected position guard's first failure. */
static void recovery_health_cleanup_first_cause(void)
{
    StaticLcdWorkflowFixture fixture;
    unsigned budget = 0U;
    recovery_health_start_hold(&fixture);
    /* Exceed the candidate one-degree HOLD trip, rather than its separate half-degree precision band. */
    fixture.position_offset_rad = 0.02F;
    workflow_tick(&fixture, 4000U); (void)aethor_app_service(fixture.timestamp_us);
    assert(application_action.local_failure_reason == DEBUG_UI_REASON_OUT_OF_RANGE);
    assert(application_action.failed_stage != PROTOCOL_COMMAND_STAGE_NONE);
    recovery_health_input_failure(&fixture);
    fixture.position_offset_rad = 0.0F;
    do {
        workflow_tick(&fixture, 1000U); (void)aethor_app_service(fixture.timestamp_us);
        workflow_collect_frames(&fixture);
        assert(++budget < 50U);
    } while (fixture.model.awaiting_result || application_action.state != AETHOR_APP_ACTION_IDLE);
    workflow_tick(&fixture, 1000U);
    assert(fixture.model.completion.code == DEBUG_UI_FAILED);
    assert(fixture.model.completion.reason == DEBUG_UI_REASON_OUT_OF_RANGE && fixture.model.completion.disabled_confirmed);
    assert(application_debug_ui.health_failure_snapshot.input_valid == 0U);
    puts("STATIC_LCD_HEALTH_CLEANUP_FIRST_CAUSE_PASS real_position_guard_then_health_failure=1 original_out_of_range_preserved=1 diagnostic_snapshot_retained=1");
}

/** @brief Unhealthy admission and a second failure during recovery keep LOCK closed while emergency STOP remains executable. */
static void recovery_health_reflight_failure(void)
{
    StaticLcdWorkflowFixture fixture;
    DebugUiRequest request;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    uint32_t frames;
    unsigned budget = 0U;
    recovery_health_owned_fixture(&fixture);
    recovery_health_input_failure(&fixture);
    workflow_tick(&fixture, 1000U);
    workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
    request = test_request(DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT, fixture.timestamp_us);
    request.identity.request_id = fixture.model.next_request_id++;
    request.target_motor_id = 7U;
    request.reference_generation = fixture.model.snapshot.motors[6].reference_generation;
    request.single_static_evidence_generation = fixture.model.snapshot.motors[6].single_static_evidence_generation;
    request.single_static_preflight_purpose = DEBUG_UI_SINGLE_STATIC_PREFLIGHT_LOCK;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    recovery_health_input_failure(&fixture);
    aethor_app_debug_ui_process(fixture.timestamp_us);
    assert(aethor_app_debug_ui_poll_admission(&admission) && !admission.accepted && admission.reason == DEBUG_UI_REASON_UNHEALTHY);
    assert(aethor_app_debug_ui_poll_completion(&completion) && completion.reason == DEBUG_UI_REASON_UNHEALTHY);
    request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK;
    request.single_static_preflight_purpose = DEBUG_UI_SINGLE_STATIC_PREFLIGHT_NONE;
    request.identity.request_id = fixture.model.next_request_id++;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_process(fixture.timestamp_us);
    assert(aethor_app_debug_ui_poll_admission(&admission) && !admission.accepted && admission.reason == DEBUG_UI_REASON_UNHEALTHY);
    assert(aethor_app_debug_ui_poll_completion(&completion) && completion.reason == DEBUG_UI_REASON_UNHEALTHY);
    assert(application_debug_ui.single_static_lcd_owned[6] && application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    workflow_tick(&fixture, 1000U);
    fixture.feedback_paused = 1U;
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture.model.single_static_preflight == 2U);
    (void)aethor_app_service(fixture.timestamp_us); workflow_collect_frames(&fixture);
    frames = fixture.emitted_frames;
    recovery_health_input_failure(&fixture);
    workflow_tick(&fixture, 1000U);
    assert(fixture.model.completion.reason == DEBUG_UI_REASON_UNHEALTHY && !fixture.model.single_static_preflight);
    assert(application_debug_ui.single_static_lcd_owned[6] && application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    assert(fixture.emitted_frames == frames && !fixture.enable_frames && !fixture.control_frames);
    workflow_key(&fixture, DEBUG_UI_KEY_RIGHT);
    fixture.feedback_paused = 0U;
    do {
        workflow_tick(&fixture, 1000U); recovery_health_input_failure(&fixture);
        workflow_collect_frames(&fixture);
        assert(++budget < 40U);
    } while (fixture.model.stop_waiting || fixture.model.stop_requested || application_action.state != AETHOR_APP_ACTION_IDLE);
    workflow_tick(&fixture, 1000U);
    assert(fixture.model.completion.operation == DEBUG_UI_OPERATION_STOP && fixture.model.completion.disabled_confirmed);
    assert(application_debug_ui.single_static_lcd_owned[6] && application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    assert(!fixture.enable_frames && !fixture.control_frames);
    puts("STATIC_LCD_HEALTH_RECOVERY_FAILCLOSED_PASS unhealthy_preflight_and_lock_rejected=1 new_failure_during_recovery_aborts=1 explicit_stop_while_unhealthy_executes=1 profile_fault_retained=1 enable_control_save_zero=0");
}

/** @brief Run exact hardware-proof logic, realistic timing and fail-closed cross-layer cases. */
static void workflow_recovery_cases(void)
{
    recovery_health_idle_relock();
    recovery_health_action_identity();
    recovery_health_cleanup_first_cause();
    recovery_health_reflight_failure();
    recovery_fault_lock_admission_gap();
    recovery_fault_owned_exit();
    recovery_fault_admission_guards();
    recovery_fault_model_guards();
    recovery_fault_purpose_contract();
    recovery_fault_evidence_guards();
    recovery_cancel_stale_ordinary_handoff();
    recovery_receipt_proof();
    recovery_sample_generation();
    recovery_fast_reply_before_receipt();
    recovery_pending_rx_cannot_qualify();
    recovery_rejected_feedback();
    recovery_stop_and_cancel();
    recovery_rejected_submit();
    recovery_rejected_admission();
    recovery_stop_after_terminal_before_snapshot();
    recovery_send_clock_and_unresolved_tx();
    recovery_capture_and_frozen_start();
}
#endif
