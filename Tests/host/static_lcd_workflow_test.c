/** @file static_lcd_workflow_test.c
 * @brief Production UI-model/mailbox/facade workflow with synthetic feedback only.
 * No HAL, serial port, probe, firmware download or physical motor is accessed.
 */
#define main existing_application_fixture_main
#include "debug_ui_app_test_main.c"
#undef main
#include "../../App/DebugUi/debug_ui_model.h"

/** @brief Own one UI and a monotonic software clock for an explicitly selected motor. */
typedef struct
{
    DebugUiModel model;
    uint64_t timestamp_us;
    uint8_t motor_id;
    uint8_t driver_state;
    uint32_t emitted_frames;
    uint32_t enable_frames;
    uint32_t control_frames;
    uint8_t feedback_paused;
    uint8_t receipt_paused;
    uint8_t wire_tx_pending; /**< Offline hardware-order injection, independent from delayed receipt polling. */
    float position_offset_rad; /**< Explicit LCD output-axis offset converted through the existing internal mapping. */
    float torque_nm; /**< Synthetic decoded torque used only to exercise the unchanged production protection. */
} StaticLcdWorkflowFixture;
/** @brief Advance an explicit preflight only in the continuous-feedback control fixture. */
static void workflow_finish_preflight(StaticLcdWorkflowFixture *fixture);

/** @brief Configuration-only operation tags can never enter the typed motor command ring. */
static void workflow_reject_typed_configuration(void)
{
    ProtocolCommand command;
    DebugUiOperation operations[2] = {DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE,
        DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK};
    uint8_t operation_index, origin;
    for (operation_index = 0U; operation_index < 2U; ++operation_index)
    {
        for (origin = 0U; origin < 2U; ++origin)
        {
            memset(&command, 0, sizeof(command));
            command.type = PROTOCOL_COMMAND_MIT_ACTION;
            command.mit_action = PROTOCOL_MIT_ACTION_HOLD;
            command.origin = (DebugUiOrigin)origin;
            command.local_operation = operations[operation_index];
            command.control_mode = ARM_CONTROL_MODE_MIT;
            command.motor_mask = 1U;
            command.values_in_radians = command.bench_relative_scope = 1U;
            command.mit_kp = command.mit_kd = 1.0F;
            command.hold_duration_us = 1000000ULL;
            assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
        }
    }
    puts("STATIC_LCD_TYPED_CONFIGURATION_REJECT_PASS operations=2 origins=2");
}

/** @brief Publish selected feedback and exchange real immutable model/application messages. */
static void workflow_tick(StaticLcdWorkflowFixture *fixture, uint32_t elapsed_us)
{
    DebugUiSnapshot snapshot;
    DebugUiRequest request;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    MotorJointFeedback feedback;
    uint8_t joint_index = (uint8_t)(fixture->motor_id - 1U);
    fixture->timestamp_us += elapsed_us;
    memset(&feedback, 0, sizeof(feedback));
    feedback.timestamp_us = fixture->timestamp_us;
    feedback.position_rad = 0.2F + 0.01F * joint_index +
        fixture->position_offset_rad * aethor_app_lcd_position_ratio(joint_index);
    feedback.driver_state = fixture->driver_state;
    feedback.torque_nm = fixture->torque_nm;
    feedback.mos_temperature_c = 30.0F;
    feedback.rotor_temperature_c = 31.0F;
    if (!fixture->feedback_paused) {
        CanFrame command, reply;
        assert(s3519_pack_mit(fixture->motor_id, &application_motor_runtime.discovery.results[joint_index].ranges,
            feedback.position_rad, feedback.velocity_rad_s, 0.0F, 0.0F, feedback.torque_nm, &command) == S3519_CODEC_STATUS_OK);
        memset(&reply, 0, sizeof(reply));
        reply.identifier = application_motor_runtime.configuration->joints[joint_index].master_id;
        reply.length = 8U;
        reply.data[0] = (uint8_t)((fixture->driver_state << 4U) | fixture->motor_id);
        reply.data[1] = command.data[0]; reply.data[2] = command.data[1]; reply.data[3] = command.data[2];
        reply.data[4] = (uint8_t)((command.data[3] & 0xF0U) | (command.data[6] & 0x0FU));
        reply.data[5] = command.data[7]; reply.data[6] = 30U; reply.data[7] = 31U;
        {
            DebugUiStaticTxIdentity ordered = {0};
            const DebugUiStaticTxIdentity *proof = NULL;
            if (!fixture->wire_tx_pending && application_action.single_static_tx_submitted) {
                if (application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) {
                    ordered.epoch = application_action.command.session_id;
                    ordered.request_id = application_action.command.request_id;
                    ordered.phase = DEBUG_UI_STATIC_TX_PREFLIGHT;
                    ordered.sequence = 1U; ordered.motor_id = fixture->motor_id;
                    proof = &ordered;
                } else if (application_action.static_identity.phase) {
                    ordered = application_action.static_identity;
                    proof = &ordered;
                }
            }
            assert(aethor_app_receive_static_frame(&reply, fixture->timestamp_us,
                fixture->timestamp_us, proof, proof ? 101U : 0U, 1U) == MOTOR_RUNTIME_STATUS_OK);
        }
    }
    test_health(fixture->timestamp_us, false);
    while (debug_ui_model_take_request(&fixture->model, &request))
    {
        DebugUiReason submit_reason = aethor_app_debug_ui_submit(&request);
        if (submit_reason != DEBUG_UI_REASON_NONE)
        { debug_ui_model_submit_failed(&fixture->model, &request, submit_reason); }
    }
    aethor_app_debug_ui_process(fixture->timestamp_us);
    while (aethor_app_debug_ui_poll_admission(&admission))
    { debug_ui_model_admission(&fixture->model, &admission); }
    while (aethor_app_debug_ui_poll_completion(&completion))
    { debug_ui_model_completion(&fixture->model, &completion); }
    assert(aethor_app_debug_ui_get_snapshot(fixture->timestamp_us, &snapshot));
    debug_ui_model_update(&fixture->model, &snapshot, fixture->timestamp_us, 1U, 1U);
}

/** @brief Deliver one complete press/release in screen-relative key coordinates. */
static void workflow_key(StaticLcdWorkflowFixture *fixture, DebugUiKey key)
{
    DebugUiInputEvent event;
    workflow_tick(fixture, 1000U);
    memset(&event, 0, sizeof(event));
    event.key = key;
    event.type = DEBUG_UI_INPUT_EVENT_PRESS;
    event.timestamp_ms = (uint32_t)(fixture->timestamp_us / 1000ULL);
    debug_ui_model_event(&fixture->model, &event);
    workflow_tick(fixture, 1000U);
    event.type = DEBUG_UI_INPUT_EVENT_RELEASE;
    event.timestamp_ms = (uint32_t)(fixture->timestamp_us / 1000ULL);
    debug_ui_model_event(&fixture->model, &event);
    workflow_tick(fixture, 1000U);
    if (!fixture->feedback_paused) workflow_finish_preflight(fixture);
}

/** @brief Hold a fresh CENTER edge for 800ms while independent feedback and health advance. */
static void workflow_confirm(StaticLcdWorkflowFixture *fixture)
{
    DebugUiInputEvent event;
    unsigned interval;
    workflow_tick(fixture, 1000U);
    memset(&event, 0, sizeof(event));
    event.key = DEBUG_UI_KEY_CENTER;
    event.type = DEBUG_UI_INPUT_EVENT_PRESS;
    event.timestamp_ms = (uint32_t)(fixture->timestamp_us / 1000ULL);
    debug_ui_model_event(&fixture->model, &event);
    for (interval = 0U; interval < 200U; ++interval) { workflow_tick(fixture, 4000U); }
    event.type = DEBUG_UI_INPUT_EVENT_HOLD;
    event.held_ms = 800U;
    event.timestamp_ms = (uint32_t)(fixture->timestamp_us / 1000ULL);
    debug_ui_model_event(&fixture->model, &event);
    workflow_tick(fixture, 1000U);
    event.type = DEBUG_UI_INPUT_EVENT_RELEASE;
    event.timestamp_ms = (uint32_t)(fixture->timestamp_us / 1000ULL);
    debug_ui_model_event(&fixture->model, &event);
    workflow_tick(fixture, 1000U);
    if (!fixture->feedback_paused) workflow_finish_preflight(fixture);
}

/** @brief Select one independently discovered ID, leaving unrelated motors without feedback. */
static void workflow_fixture(StaticLcdWorkflowFixture *fixture, uint8_t motor_id)
{
    DebugUiMotorProfile profile;
    MotorDiscoveryResult discovery;
    uint8_t joint_index;
    test_fixture();
    profile = application_debug_ui.profiles[0];
    discovery = application_motor_runtime.discovery.results[0];
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        memset(&application_motor_runtime.discovery.results[joint_index], 0, sizeof(discovery));
        application_motor_runtime.bank.motors[joint_index].feedback_valid = 0U;
    }
    joint_index = (uint8_t)(motor_id - 1U);
    discovery.observed_control_mode = 1U;
    discovery.observed_master_id = application_motor_runtime.configuration->joints[joint_index].master_id;
    discovery.observed_esc_id = application_motor_runtime.configuration->joints[joint_index].esc_id;
    application_motor_runtime.discovery.results[joint_index] = discovery;
    application_motor_runtime.discovery.verified_joint_mask = (uint8_t)(1U << joint_index);
    /* This fixture has already supplied discovery results; no background readout is in flight. */
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    application_motor_runtime.discovery_active = 0U;
    assert(aethor_app_debug_ui_set_motor_profile(motor_id, &profile));
    memset(fixture, 0, sizeof(*fixture));
    fixture->motor_id = motor_id;
    fixture->driver_state = S3519_DRIVER_STATE_DISABLED;
    fixture->timestamp_us = 1000U;
    debug_ui_model_init(&fixture->model);
    workflow_tick(fixture, 1000U);
    workflow_key(fixture, DEBUG_UI_KEY_LEFT);
    assert(fixture->model.focus == 3U);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture->model.page == DEBUG_UI_PAGE_STATIC_HUB);
    workflow_key(fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture->model.page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
    for (joint_index = 1U; joint_index < motor_id; ++joint_index)
    { workflow_key(fixture, DEBUG_UI_KEY_RIGHT); }
    assert(fixture->model.selected_motor == motor_id - 1U);
}

/** @brief Count actual facade frames and acknowledge only emitted selected mode commands later. */
static void workflow_collect_frames(StaticLcdWorkflowFixture *fixture)
{
    CanFrame frame;
    CanTxPriority priority;
    unsigned frame_count = 0U;
    while (aethor_app_next_can_frame(fixture->timestamp_us + 1ULL, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY)
    {
        uint8_t prefix_index, mode_prefix = 1U;
        assert(++frame_count <= 7U);
        if (frame.identifier != application_motor_runtime.configuration->joints[fixture->motor_id - 1U].esc_id)
        {
            fprintf(stderr, "UNEXPECTED_WORKFLOW_FRAME id=%lu selected=%u byte0=%u byte7=%u action=%u page=%u pending=%u stop=%u\n",
                (unsigned long)frame.identifier, fixture->motor_id, frame.data[0], frame.data[7],
                (unsigned)application_action.state, (unsigned)fixture->model.page,
                fixture->model.snapshot.pending, fixture->model.stop_requested);
        }
        assert(frame.identifier == application_motor_runtime.configuration->joints[fixture->motor_id - 1U].esc_id);
        ++fixture->emitted_frames;
        {
            DebugUiStaticTxIdentity identity;
            if (aethor_app_static_frame_identity(&frame, &identity)) {
                aethor_app_static_submitted(&identity, fixture->timestamp_us + 1ULL);
                if (!fixture->receipt_paused) aethor_app_static_transmitted(&identity, 1U, 100U, fixture->timestamp_us + 1ULL);
            }
        }
        for (prefix_index = 0U; prefix_index < 7U; ++prefix_index)
        { if (frame.data[prefix_index] != 0xFFU) { mode_prefix = 0U; } }
        if (mode_prefix && frame.data[7] == S3519_MODE_COMMAND_ENABLE)
        { fixture->driver_state = S3519_DRIVER_STATE_ENABLED; ++fixture->enable_frames; }
        if (mode_prefix && frame.data[7] == S3519_MODE_COMMAND_DISABLE)
        { fixture->driver_state = S3519_DRIVER_STATE_DISABLED; }
        assert(!(mode_prefix && frame.data[7] == S3519_MODE_COMMAND_SAVE_ZERO));
        if (!mode_prefix) {
            ++fixture->control_frames;
            if (fixture->motor_id == 7U && application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
                application_debug_ui.single_static_lcd_owned[6] && application_debug_ui.single_static_gains[6].kp > 0.0F) {
                unsigned kp_code = ((unsigned)(frame.data[3] & 15U) << 8) | frame.data[4];
                unsigned kd_code = ((unsigned)frame.data[5] << 4) | (frame.data[6] >> 4);
                float expected_kp = application_debug_ui.single_static_gains[6].kp / aethor_app_lcd_position_ratio(6U);
                float expected_kd = application_debug_ui.single_static_gains[6].kd;
                assert(fabsf((float)kp_code * 500.0F / 4095.0F - expected_kp) <= 500.0F / 4095.0F);
                assert(fabsf((float)kd_code * 5.0F / 4095.0F - expected_kd) <= 5.0F / 4095.0F);
            }
        }
    }
}

/** @brief Execute only a visible pending preflight and its frame-free continuation. */
static void workflow_finish_preflight(StaticLcdWorkflowFixture *fixture)
{
    unsigned budget = 0U;
    if (!fixture->model.single_static_preflight) return;
    do {
        workflow_tick(fixture, 1000U);
        (void)aethor_app_service(fixture->timestamp_us);
        workflow_collect_frames(fixture);
        workflow_tick(fixture, 1000U);
        assert(++budget < 20U);
    } while (fixture->model.single_static_preflight || fixture->model.outgoing_ready || fixture->model.single_static_wait_snapshot);
}

/** @brief Stale browsing and already stranded pages release no unrelated authority or frames. */
static void workflow_stale_browse_exit(void)
{
    StaticLcdWorkflowFixture fixture;
    DebugUiPage pages[3] = { DEBUG_UI_PAGE_SINGLE_STATIC_SETUP,
        DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK, DEBUG_UI_PAGE_RESULT };
    uint8_t page_index, local;
    for (local = 0U; local < 2U; ++local)
        for (page_index = 0U; page_index < 3U; ++page_index)
        {
            DebugUiSnapshot snapshot;
            uint32_t epoch;
            workflow_fixture(&fixture, 3U);
            if (local) {
                aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_ARMED);
                application_debug_ui.target_motor_id = 1U;
            }
            epoch = application_debug_ui.epoch;
            fixture.timestamp_us += 900000ULL;
            test_health(fixture.timestamp_us, false);
            assert(aethor_app_debug_ui_get_snapshot(fixture.timestamp_us, &snapshot));
            debug_ui_model_update(&fixture.model, &snapshot, fixture.timestamp_us, 1U, 1U);
            fixture.model.page = pages[page_index];
            fixture.model.focus = 2U;
            fixture.model.single_static_relock_destination = DEBUG_UI_PAGE_STATIC_HUB;
            fixture.model.single_static_relock_motor_id = local ? 1U : 3U;
            {
                DebugUiInputEvent event = {0};
                event.key = page_index == 1U ? DEBUG_UI_KEY_CENTER : DEBUG_UI_KEY_LEFT;
                event.type = DEBUG_UI_INPUT_EVENT_PRESS;
                event.timestamp_ms = (uint32_t)(fixture.timestamp_us / 1000ULL);
                debug_ui_model_event(&fixture.model, &event);
            }
            assert(fixture.model.page == DEBUG_UI_PAGE_STATIC_HUB);
            assert(!fixture.model.outgoing_ready && !fixture.model.stop_ready);
            assert(application_debug_ui.epoch == epoch);
            assert(application_debug_ui.authority == (local ? DEBUG_UI_AUTHORITY_LOCAL_ARMED : DEBUG_UI_AUTHORITY_REMOTE));
            workflow_collect_frames(&fixture);
            assert(fixture.emitted_frames == 0U);
        }
    puts("STATIC_LCD_STALE_BROWSE_EXIT_PASS setup=1 stranded_relock=1 stop_result=1 authority_preserved=1 frames=0");
}

#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
/** @brief Signed decimal editing cycles one digit without carry and cancels without configuration or frames. */
static void workflow_feedforward_digits(void)
{
    StaticLcdWorkflowFixture fixture;
    DebugUiInputEvent event;
    workflow_fixture(&fixture, 7U);
    fixture.model.focus = 3U;
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture.model.single_static_ff_editing);
    fixture.model.single_static_ff_centi = 109U; fixture.model.single_static_ff_digit = 3U;
    workflow_key(&fixture, DEBUG_UI_KEY_UP);
    assert(fixture.model.single_static_ff_centi == 100U);
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    assert(fixture.model.single_static_ff_centi == 109U);
    fixture.model.single_static_ff_centi = 350U; fixture.model.single_static_ff_digit = 2U;
    workflow_key(&fixture, DEBUG_UI_KEY_UP);
    assert(fixture.model.single_static_ff_centi == 300U);
    fixture.model.single_static_ff_digit = 0U;
    workflow_key(&fixture, DEBUG_UI_KEY_UP);
    assert(debug_ui_model_single_static_feedforward(&fixture.model) == -3.0F);
    memset(&event, 0, sizeof(event)); event.key = DEBUG_UI_KEY_LEFT; event.type = DEBUG_UI_INPUT_EVENT_PRESS;
    debug_ui_model_event(&fixture.model, &event);
    event.type = DEBUG_UI_INPUT_EVENT_HOLD; event.held_ms = 800U;
    debug_ui_model_event(&fixture.model, &event);
    assert(!fixture.model.single_static_ff_editing && !fixture.model.single_static_ff_centi && !fixture.model.single_static_ff_negative);
    event.type = DEBUG_UI_INPUT_EVENT_RELEASE; debug_ui_model_event(&fixture.model, &event);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
    fixture.model.focus = 3U; workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    fixture.model.single_static_ff_centi = 105U; fixture.model.single_static_ff_negative = 1U;
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(!fixture.model.single_static_ff_editing && fixture.model.single_static_ff_centi == 105U);
    fixture.model.focus = 2U; workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW);
    assert(fabsf(fixture.model.reviewed.torque_ff_nm + 1.05F) < 0.000001F);
    workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
    fixture.model.focus = 0U; workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
    assert(!fixture.model.single_static_ff_centi && !fixture.model.single_static_ff_negative);
    assert(!fixture.emitted_frames);
    puts("STATIC_LCD_FF_DIGITS_PASS no_carry=1 illegal_candidates_skipped=1 sign=1 save_cancel=1 axis_change_zero=1");
}

/** @brief Human timing with no periodic feedback must enter review and request a separate preflight. */
static void workflow_stale_configuration_review(void)
{
    StaticLcdWorkflowFixture fixture;
    workflow_fixture(&fixture, 7U);
    fixture.feedback_paused = 1U;
    workflow_tick(&fixture, 900000U);
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW);
    workflow_confirm(&fixture);
    assert(!application_debug_ui.single_static_lcd_owned[6]);
    assert(fixture.model.awaiting_result);
    assert(fixture.model.submitted.operation != DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE);
    assert(fixture.model.submitted.target_motor_id == 7U);
    (void)aethor_app_service(fixture.timestamp_us);
    workflow_collect_frames(&fixture);
    assert(fixture.emitted_frames == 1U);
    workflow_tick(&fixture, 10000U);
    (void)aethor_app_service(fixture.timestamp_us);
    workflow_tick(&fixture, 1000U);
    assert(fixture.model.awaiting_result && !application_debug_ui.single_static_lcd_owned[6]);
    fixture.feedback_paused = 0U;
    workflow_finish_preflight(&fixture);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY);
    assert(application_debug_ui.single_static_lcd_owned[6] && fixture.emitted_frames == 1U);
    puts("STATIC_LCD_STALE_CONFIG_REVIEW_PASS human_wait=900ms hold=800ms separate_preflight=1 config_not_granted=1");
}

/** @brief Release a different idle local target without changing the selected test or its old public profile. */
static void workflow_handoff_existing_local_target(void)
{
    StaticLcdWorkflowFixture fixture;
    DebugUiSingleStaticProfile public_profile;
    uint8_t increment;
    workflow_fixture(&fixture, 7U);
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    for (increment = 1U; increment < 60U; ++increment)
    { workflow_key(&fixture, DEBUG_UI_KEY_RIGHT); }
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    memset(&public_profile, 0, sizeof(public_profile));
    public_profile.configured = 1U;
    public_profile.minimum_ms = 1000U;
    public_profile.maximum_ms = 60000U;
    assert(aethor_app_debug_ui_set_single_static_profile(1U, &public_profile));
    /* Represent a previously granted, idle local session; only the existing axis has new disabled evidence. */
    aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    application_debug_ui.target_motor_id = 1U;
    test_feedback(fixture.timestamp_us, 0.2F, S3519_DRIVER_STATE_DISABLED);
    workflow_tick(&fixture, 1000U);
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW);
    assert(fixture.model.selected_motor == 6U && fixture.model.reviewed.target_motor_id == 7U);
    assert(fixture.model.single_static_seconds == 60U);
    assert(application_debug_ui.single_static_profiles[0].configured &&
        !application_debug_ui.single_static_lcd_owned[0]);
    workflow_confirm(&fixture);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY);
    assert(application_debug_ui.single_static_lcd_owned[6]);
    assert(application_debug_ui.single_static_profiles[0].configured);
    workflow_collect_frames(&fixture);
    assert(fixture.emitted_frames == 1U);
    workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
    assert(!application_debug_ui.single_static_profiles[6].configured);
    assert(application_debug_ui.single_static_profiles[0].configured);
    assert(fixture.emitted_frames == 2U);
    puts("STATIC_LCD_E2E_LOCAL_HANDOFF_PASS old_id=1 selected_id=7 seconds=60 old_public_profile_preserved=1 selected_preflights=2 other_frames=0");
}

/** @brief Execute both independent confirmations, real HOLD lifecycle, report and locked exit. */
static void workflow_trial(uint8_t motor_id, uint8_t duration_seconds, float torque_ff_nm, uint8_t wide_pose)
{
    StaticLcdWorkflowFixture fixture;
    uint64_t budget_end_us;
    uint8_t increment;
    workflow_fixture(&fixture, motor_id);
    if (motor_id == 7U) {
        fixture.model.single_static_kp = wide_pose ? 80U : (duration_seconds == 60U ? 200U : 120U);
        fixture.model.single_static_kd_centi = wide_pose ? 20U : (duration_seconds == 60U ? 40U : 30U);
    }
    if (wide_pose) {
        assert(motor_id == 7U);
        application_motor_runtime.discovery.results[6].ranges.position_max_rad = 512.0F;
        fixture.position_offset_rad = (-1185.772F * AETHOR_APP_DEG_TO_RAD - 0.26F) / aethor_app_lcd_position_ratio(6U);
        workflow_tick(&fixture, 1000U);
        assert(fixture.model.snapshot.motors[6].position_rad < fixture.model.snapshot.motors[6].profile.mit_position_min_rad);
    }
    fixture.model.single_static_ff_centi = (uint16_t)(fabsf(torque_ff_nm) * 100.0F + 0.5F);
    fixture.model.single_static_ff_negative = (uint8_t)(torque_ff_nm < 0.0F);
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    for (increment = 1U; increment < duration_seconds; ++increment)
    { workflow_key(&fixture, DEBUG_UI_KEY_RIGHT); }
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW);
    workflow_confirm(&fixture);
    if (fixture.model.page != DEBUG_UI_PAGE_SINGLE_STATIC_READY)
    {
        DebugUiMotorView *motor = &fixture.model.snapshot.motors[motor_id - 1U];
        fprintf(stderr, "CONFIG_SYNC_FAILED id=%u page=%u reason=%u completed=%u wait=%u ref=%lu/%lu evidence=%lu/%lu configured=%u owned=%u\n",
            motor_id, (unsigned)fixture.model.page, (unsigned)fixture.model.reason,
            fixture.model.completion_seen, fixture.model.single_static_wait_snapshot,
            (unsigned long)motor->reference_generation, (unsigned long)fixture.model.submitted.reference_generation,
            (unsigned long)motor->single_static_evidence_generation, (unsigned long)fixture.model.submitted.single_static_evidence_generation,
            motor->single_static_profile.configured, motor->single_static_lcd_owned);
    }
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY);
    assert(application_debug_ui.single_static_profiles[motor_id - 1U].configured);
    assert(application_debug_ui.single_static_profiles[motor_id - 1U].maximum_ms == (uint32_t)duration_seconds * 1000U);
    assert(fixture.model.snapshot.motors[motor_id - 1U].single_static_lcd_owned);
    workflow_collect_frames(&fixture);
    assert(fixture.emitted_frames == 1U && application_action.state == AETHOR_APP_ACTION_IDLE);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE);
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    if (fixture.model.page != DEBUG_UI_PAGE_REVIEW) fprintf(stderr,
        "ACQUIRE_REVIEW_FAIL page=%u reason=%u consumed=%u down=%u wait=%u preflight=%u authority=%u\n",
        (unsigned)fixture.model.page, (unsigned)fixture.model.reason, fixture.model.center_consumed, fixture.model.center_down,
        fixture.model.single_static_wait_snapshot, fixture.model.single_static_preflight, fixture.model.snapshot.authority);
    assert(fixture.model.page == DEBUG_UI_PAGE_REVIEW);
    workflow_confirm(&fixture);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    if (fixture.model.page != DEBUG_UI_PAGE_SINGLE_STATIC_READY)
    {
        fprintf(stderr, "ACQUIRE_SYNC_FAILED id=%u page=%u reason=%u complete=%u wait=%u configured=%u ref=%lu/%lu evidence=%lu/%lu mask=%u\n",
            motor_id, (unsigned)fixture.model.page, (unsigned)fixture.model.reason,
            fixture.model.completion_seen, fixture.model.single_static_wait_snapshot,
            fixture.model.single_static_configured,
            (unsigned long)fixture.model.snapshot.motors[motor_id - 1U].reference_generation,
            (unsigned long)fixture.model.single_static_owned_reference,
            (unsigned long)fixture.model.snapshot.motors[motor_id - 1U].single_static_evidence_generation,
            (unsigned long)fixture.model.single_static_owned_evidence, fixture.model.completion.motor_mask);
    }
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY);
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture.model.static_locked);
    assert(fixture.model.draft.hold_duration_ms == (uint32_t)duration_seconds * 1000U);
    workflow_collect_frames(&fixture);
    assert(fixture.emitted_frames == 2U);
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    assert(fixture.model.page == DEBUG_UI_PAGE_REVIEW);
    workflow_confirm(&fixture);
    assert(fixture.model.submitted.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
    assert(fixture.model.submitted.hold_duration_ms == (uint32_t)duration_seconds * 1000U);
    budget_end_us = fixture.timestamp_us + (uint64_t)duration_seconds * 1000000ULL +
        (torque_ff_nm != 0.0F ? 1200000ULL : 200000ULL);
    do
    {
        workflow_tick(&fixture, 4000U);
        (void)aethor_app_service(fixture.timestamp_us);
        workflow_collect_frames(&fixture);
    } while ((fixture.model.awaiting_result || application_debug_ui.static_load.active) && fixture.timestamp_us < budget_end_us);
    workflow_tick(&fixture, 4000U);
    if (!application_debug_ui.static_load.complete)
    {
        fprintf(stderr, "WORKFLOW_FAILED id=%u seconds=%u app_state=%u reason=%u page=%u samples=%lu\n",
            motor_id, duration_seconds, (unsigned)application_action.state, (unsigned)fixture.model.reason,
            (unsigned)fixture.model.page, (unsigned long)application_debug_ui.static_load.samples[motor_id - 1U]);
    }
    assert(application_debug_ui.static_load.complete && application_debug_ui.static_load.disabled_confirmed);
    assert(application_debug_ui.static_load.elapsed_ms == (uint32_t)duration_seconds * 1000U);
    assert(fixture.model.page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT);
    workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
    assert(!application_debug_ui.single_static_profiles[motor_id - 1U].configured);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE);
    assert(fixture.model.page == DEBUG_UI_PAGE_STATIC_HUB);
    assert(test_critical_depth == 0U);
}
#endif

#include "static_lcd_recovery_cases.inc"

/** @brief Run actual production cross-layer flows with a default-lock control configuration. */
int main(void)
{
    workflow_reject_typed_configuration();
    workflow_stale_browse_exit();
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    uint8_t motor_id;
    workflow_feedforward_digits();
    workflow_stale_configuration_review();
    workflow_recovery_cases();
    workflow_handoff_existing_local_target();
    for (motor_id = 1U; motor_id <= DEBUG_UI_MOTOR_COUNT; ++motor_id)
    { workflow_trial(motor_id, 1U, 0.0F, 0U); workflow_trial(motor_id, 60U, 0.0F, 0U); }
    workflow_trial(7U, 1U, 1.05F, 0U); workflow_trial(7U, 1U, -1.05F, 0U);
    workflow_trial(7U, 1U, 0.0F, 1U); workflow_trial(7U, 1U, 1.05F, 1U);
    puts("STATIC_LCD_CACHED_POSE_E2E_PASS native_minus1185_772deg=1 configure_acquire_capture_start_hold=1 zero_and_positive_ff=1");
    puts("STATIC_LCD_FF_E2E_PASS signs=2 configure_capture_review_takeover_hold_relock=1");
    puts("STATIC_LCD_E2E_PASS ids=7 windows=1s_and_60s config_no_frames=1 acquire_no_frames=1 real_hold_and_relock=1");
#else
    StaticLcdWorkflowFixture fixture;
    workflow_fixture(&fixture, 3U);
    assert(debug_ui_model_gate(&fixture.model, DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) == DEBUG_UI_REASON_DISABLED);
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(&fixture, DEBUG_UI_KEY_CENTER);
    workflow_confirm(&fixture);
    workflow_collect_frames(&fixture);
    assert(!application_debug_ui.single_static_profiles[2].configured);
    assert(fixture.emitted_frames == 0U && application_action.state == AETHOR_APP_ACTION_IDLE);
    puts("STATIC_LCD_E2E_DEFAULT_LOCK_PASS browse_only=1 frames=0");
    workflow_fixture(&fixture, 3U);
    aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    application_debug_ui.target_motor_id = 1U;
    workflow_tick(&fixture, 1000U);
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(&fixture, DEBUG_UI_KEY_DOWN);
    workflow_key(&fixture, DEBUG_UI_KEY_LEFT);
    assert(fixture.model.page == DEBUG_UI_PAGE_STATIC_HUB);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
        application_debug_ui.target_motor_id == 1U);
    workflow_collect_frames(&fixture);
    assert(fixture.emitted_frames == 0U);
    puts("STATIC_LCD_E2E_LOCKED_BROWSE_EXIT_PASS original_local_authority_preserved=1 frames=0");
#endif
    return 0;
}
