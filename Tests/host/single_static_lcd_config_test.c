/** @file single_static_lcd_config_test.c
 * @brief Offline LCD configuration ownership, evidence and zero-actuation regression.
 */
#define main existing_debug_ui_test_main
#include "debug_ui_app_test_main.c"
#undef main

/** @brief Prepare one selected MIT motor under remote authority without a long profile. */
static DebugUiRequest lcd_config_fixture(uint8_t motor_id, uint32_t duration_ms)
{
    DebugUiSnapshot snapshot;
    DebugUiRequest request;
    DebugUiMotorProfile profile;
    MotorDiscoveryResult discovery;
    uint8_t joint_index = (uint8_t)(motor_id - 1U);
    test_fixture();
    profile = application_debug_ui.profiles[0];
    discovery = application_motor_runtime.discovery.results[0];
    discovery.observed_control_mode = 1U;
    discovery.observed_master_id = application_motor_runtime.configuration->joints[joint_index].master_id;
    discovery.observed_esc_id = application_motor_runtime.configuration->joints[joint_index].esc_id;
    application_motor_runtime.discovery.results[joint_index] = discovery;
    application_motor_runtime.discovery.verified_joint_mask |= (uint8_t)(1U << joint_index);
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    application_motor_runtime.discovery_active = 0U;
    assert(aethor_app_debug_ui_set_motor_profile(motor_id, &profile));
    application_motor_runtime.bank.motors[joint_index].feedback = application_motor_runtime.bank.motors[0].feedback;
    application_motor_runtime.bank.motors[joint_index].feedback_valid = 1U;
    assert(aethor_app_debug_ui_get_snapshot(1000U, &snapshot));
    request = test_request(DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE, 1000U);
    request.target_motor_id = motor_id;
    request.reference_generation = snapshot.motors[joint_index].reference_generation;
    request.single_static_evidence_generation = snapshot.motors[joint_index].single_static_evidence_generation;
    request.hold_duration_ms = duration_ms;
    request.kp = request.kd = 0.0F; /* Legacy configuration carries no gain override. */
    return request;
}

/** @brief Require immediate exact-identity results with no manufactured disable confirmation. */
static DebugUiAdmission lcd_config_execute(DebugUiRequest request, uint64_t timestamp_us)
{
    DebugUiAdmission admission = test_admission(request, timestamp_us);
    DebugUiCompletion completion;
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.identity.origin == request.identity.origin);
    assert(completion.identity.epoch == request.identity.epoch);
    assert(completion.identity.request_id == request.identity.request_id);
    assert(completion.operation == request.operation);
    assert(completion.motor_mask == (uint8_t)(1U << (request.target_motor_id - 1U)));
    assert(completion.disabled_confirmed == 0U);
    assert(completion.code == (admission.accepted ? DEBUG_UI_COMPLETED : DEBUG_UI_FAILED));
    return admission;
}

/** @brief Configuration and locking must leave every actuator queue and output batch empty. */
static void lcd_config_no_actuation(void)
{
    CanFrame frame;
    CanTxPriority priority;
    assert(application_action.state == AETHOR_APP_ACTION_IDLE);
    assert(application_action.frames.count == 0U);
    assert(application_protocol_engine.command_write_sequence == application_protocol_engine.command_read_sequence);
    assert(application_protocol_engine.stop_write_sequence == application_protocol_engine.stop_read_sequence);
    assert(!application_protocol_engine.active_motion_request_id);
    assert(!aethor_app_pop_emergency_can_frame(&frame));
    assert(aethor_app_next_can_frame(1001U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
}

/** @brief Default-closed compile gates reject the new request without motor side effects. */
static void lcd_config_compile_gate(void)
{
    DebugUiRequest request = lcd_config_fixture(1U, 60000U);
    DebugUiAdmission admission = lcd_config_execute(request, 1000U);
    assert(admission.accepted == (AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD != 0U));
    if (!AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD) { assert(admission.reason == DEBUG_UI_REASON_DISABLED); }
    lcd_config_no_actuation();
}

#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
/** @brief Create a fresh lock against the current selected snapshot. */
static DebugUiRequest lcd_lock_request(uint8_t motor_id)
{
    DebugUiSnapshot snapshot;
    DebugUiRequest request = test_request(DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK, 1001U);
    request.kp = request.kd = 0.0F; /* LOCK does not carry a gain command. */
    assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
    request.target_motor_id = motor_id;
    request.reference_generation = snapshot.motors[motor_id - 1U].reference_generation;
    request.single_static_evidence_generation = snapshot.motors[motor_id - 1U].single_static_evidence_generation;
    return request;
}

/** @brief RAM installation freezes valid ID7 gains and rejects every other scope or invalid pair. */
static void lcd_config_gain_cases(void)
{
    unsigned index;
    static const float invalid[][2] = {{79.0F,.3F},{201.0F,.3F},{100.0F,.19F},{100.0F,.41F},{NAN,.3F},{100.0F,NAN}};
    DebugUiRequest request;
    DebugUiSnapshot snapshot;
    for (index = 0U; index < sizeof(invalid)/sizeof(invalid[0]); ++index) {
        request = lcd_config_fixture(7U, 1000U);
        request.kp = invalid[index][0]; request.kd = invalid[index][1];
        assert(!lcd_config_execute(request, 1000U).accepted);
        lcd_config_no_actuation();
    }
    request = lcd_config_fixture(6U, 1000U); request.kp = 100.0F; request.kd = .30F;
    assert(!lcd_config_execute(request, 1000U).accepted);
    request = lcd_config_fixture(7U, 1000U); request.kp = 100.0F; request.kd = .30F;
    assert(lcd_config_execute(request, 1000U).accepted);
    assert(aethor_app_debug_ui_get_snapshot(1000U, &snapshot));
    assert(snapshot.motors[6].single_static_gains.kp == 100.0F);
    assert(snapshot.motors[6].single_static_gains.kd == .30F);
    lcd_config_no_actuation();
    puts("STATIC_GAIN_CONFIG_PASS invalid=6 scoped=1 snapshot=1 no_actuation=1");
}

/** @brief Reject nonfinite and out-of-bound signed feedforward before installing RAM configuration. */
static void lcd_config_feedforward_rejections(void)
{
    const float invalid[] = {NAN, INFINITY, -INFINITY, 3.51F, -3.51F};
    unsigned index;
    for (index = 0U; index < sizeof(invalid) / sizeof(invalid[0]); ++index) {
        DebugUiRequest request = lcd_config_fixture(7U, 1000U);
        request.torque_ff_nm = invalid[index];
        assert(!lcd_config_execute(request, 1000U).accepted);
        lcd_config_no_actuation();
    }
    puts("SINGLE_STATIC_FF_REJECT_PASS nan=1 infinity=1 signed_ceiling=1");
}

/** @brief All IDs and both endpoint windows install in RAM and lock without changing MIT bounds. */
static void lcd_config_install_and_lock(void)
{
    uint8_t motor_id;
    uint8_t window;
    for (motor_id = 1U; motor_id <= DEBUG_UI_MOTOR_COUNT; ++motor_id)
    {
        for (window = 0U; window < 2U; ++window)
        {
            DebugUiSnapshot snapshot;
            uint8_t joint_index = (uint8_t)(motor_id - 1U);
            uint32_t duration_ms = window ? 60000U : 1000U;
            DebugUiRequest request = lcd_config_fixture(motor_id, duration_ms);
            DebugUiMotorProfile original_profile = application_debug_ui.profiles[joint_index];
            assert(lcd_config_execute(request, 1000U).accepted);
            assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
            assert(snapshot.authority == DEBUG_UI_AUTHORITY_REMOTE);
            assert(snapshot.motors[joint_index].single_static_lcd_owned);
            assert(snapshot.motors[joint_index].single_static_profile.configured);
            assert(snapshot.motors[joint_index].single_static_profile.minimum_ms == 1000U);
            assert(snapshot.motors[joint_index].single_static_profile.maximum_ms == duration_ms);
            assert(snapshot.motors[joint_index].reference_generation != request.reference_generation);
            assert(memcmp(&original_profile, &application_debug_ui.profiles[joint_index], sizeof(original_profile)) == 0);
            assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_DUPLICATE);
            lcd_config_no_actuation();
            request = lcd_lock_request(motor_id);
            assert(lcd_config_execute(request, 1001U).accepted);
            assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
            assert(!snapshot.motors[joint_index].single_static_profile.configured);
            assert(!snapshot.motors[joint_index].single_static_lcd_owned);
            assert(snapshot.authority == DEBUG_UI_AUTHORITY_REMOTE);
            assert(memcmp(&original_profile, &application_debug_ui.profiles[joint_index], sizeof(original_profile)) == 0);
            lcd_config_no_actuation();
        }
    }
    puts("SINGLE_STATIC_LCD_INSTALL_LOCK_PASS ids=7 windows=1s_and_60s actuator_frames=0 original_mit_preserved=1");
}

/** @brief Reject each unsafe prerequisite and preserve remote authority and locked RAM. */
static void lcd_config_rejections(void)
{
    uint8_t scenario;
    for (scenario = 0U; scenario < 25U; ++scenario)
    {
        DebugUiRequest request = lcd_config_fixture(3U, 60000U);
        DebugUiAdmission admission;
        if (scenario == 0U) { request.hold_duration_ms = 999U; }
        if (scenario == 1U) { request.hold_duration_ms = 60001U; }
        if (scenario == 2U) { ++request.identity.epoch; }
        if (scenario == 3U) { ++request.reference_generation; }
        if (scenario == 4U) { ++request.single_static_evidence_generation; }
        if (scenario == 5U) { request.created_at_us = 1001U; }
        if (scenario == 6U) { application_debug_ui.mailbox.health.display_valid = 0U; }
        if (scenario == 7U) { application_motor_runtime.bank.motors[2].feedback_valid = 0U; }
        if (scenario == 8U) { application_motor_runtime.bank.motors[2].feedback.driver_state = S3519_DRIVER_STATE_ENABLED; }
        if (scenario == 9U) { application_motor_runtime.bank.motors[2].feedback.fault_flags = 1U; }
        if (scenario == 10U) { application_motor_runtime.bank.motors[2].feedback.velocity_rad_s = 1.0F; }
        if (scenario == 11U) { application_motor_runtime.discovery.results[2].verified_fields_mask = 0U; }
        if (scenario == 12U) { application_motor_runtime.discovery.results[2].observed_control_mode = 2U; }
        if (scenario == 13U) { application_debug_ui.profiles[2].mit_valid = 0U; }
        if (scenario == 14U) { application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; }
        if (scenario == 15U) { application_debug_ui.unconfirmed_disable_mask = 4U; }
        if (scenario == 16U) { application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT; }
        if (scenario == 17U) { application_motor_runtime.bank.motors[2].feedback.torque_nm = NAN; }
        if (scenario == 18U) { application_motor_runtime.bank.motors[2].feedback.mos_temperature_c = 56.0F; }
        if (scenario == 19U) { application_motor_runtime.bank.motors[2].feedback.rotor_temperature_c = 56.0F; }
        if (scenario == 20U) { application_motor_runtime.bank.motors[2].feedback.torque_nm = 3.6F; }
        if (scenario == 21U) { application_motor_runtime.bank.motors[2].feedback.timestamp_us = 0U; }
        if (scenario == 22U) { application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_BUSY; }
        if (scenario == 23U) { ++application_protocol_engine.command_write_sequence; }
        if (scenario == 24U) { application_debug_ui.mailbox.stop_intent_mask = 4U; }
        if (scenario >= 11U && scenario <= 13U)
        {
            DebugUiSnapshot snapshot;
            assert(aethor_app_debug_ui_get_snapshot(1000U, &snapshot));
            request.reference_generation = snapshot.motors[2].reference_generation;
            request.single_static_evidence_generation = snapshot.motors[2].single_static_evidence_generation;
        }
        admission = lcd_config_execute(request, 1000U);
        if (admission.accepted) { fprintf(stderr, "LCD_CONFIG_UNSAFE_ACCEPT scenario=%u\n", scenario); }
        assert(!admission.accepted);
        assert(!application_debug_ui.single_static_profiles[2].configured);
        assert(!application_action.frames.count);
    }
    {
        DebugUiRequest request = lcd_config_fixture(1U, 1000U);
        request.identity.origin = DEBUG_UI_ORIGIN_USB;
        assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_INVALID_ARGUMENT);
        lcd_config_no_actuation();
    }
    {
        DebugUiRequest request = lcd_config_fixture(3U, 60000U);
        uint64_t timestamp_us = DEBUG_UI_REQUEST_MAX_AGE_US + 1001U;
        assert(lcd_config_execute(request, timestamp_us).reason == DEBUG_UI_REASON_STALE_REQUEST);
        request = lcd_config_fixture(3U, 60000U);
        timestamp_us = MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US + 1001U;
        request.created_at_us = timestamp_us;
        test_health(timestamp_us, true);
        assert(lcd_config_execute(request, timestamp_us).reason == DEBUG_UI_REASON_STALE_FEEDBACK);
    }
    puts("SINGLE_STATIC_LCD_REJECT_PASS scenarios=28 no_unsafe_install=1");
}

/** @brief Evidence changes while locked invalidate reviewed requests even when values revert later. */
static void lcd_config_evidence_reversal(void)
{
    uint8_t field;
    for (field = 0U; field < 16U; ++field)
    {
        DebugUiRequest request = lcd_config_fixture(5U, 60000U);
        DebugUiSnapshot snapshot;
        MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[4];
        MotorDiscoveryResult original = *discovery;
        DebugUiMotorProfile original_profile = application_debug_ui.profiles[4];
        uint32_t old_reference = request.reference_generation;
        if (field == 0U) { ++discovery->observed_master_id; }
        if (field == 1U) { ++discovery->observed_esc_id; }
        if (field == 2U) { ++discovery->hardware_version; }
        if (field == 3U) { ++discovery->software_version; }
        if (field == 4U) { ++discovery->sub_version; }
        if (field == 5U) { ++discovery->communication_timeout_raw; }
        if (field == 6U) { discovery->ranges.position_max_rad += 1.0F; }
        if (field == 7U) { discovery->ranges.velocity_max_rad_s += 1.0F; }
        if (field == 8U) { discovery->ranges.torque_max_nm += 1.0F; }
        if (field == 9U) { discovery->maximum_speed_rad_s += 1.0F; }
        if (field == 10U) { discovery->acceleration_rad_s2 += 1.0F; }
        if (field == 11U) { discovery->deceleration_rad_s2 += 1.0F; }
        if (field == 12U) { discovery->observed_control_mode = 2U; }
        if (field == 13U) { discovery->verified_fields_mask ^= MOTOR_DISCOVERY_VERSION_FIELDS_MASK; }
        if (field == 14U) { ++application_debug_ui.reference_generation[4]; }
        if (field == 15U) { application_debug_ui.profiles[4].default_kp += 0.1F; }
        assert(aethor_app_debug_ui_get_snapshot(1000U, &snapshot));
        assert(snapshot.motors[4].single_static_evidence_generation != request.single_static_evidence_generation);
        /* Existing coordinate identity protection invalidates references on either CAN ID change. */
        if (field < 2U) { assert(snapshot.motors[4].reference_generation != old_reference); }
        else if (field < 12U) { assert(snapshot.motors[4].reference_generation == old_reference); }
        *discovery = original;
        if (field == 14U) { application_debug_ui.reference_generation[4] = old_reference; }
        if (field == 15U) { application_debug_ui.profiles[4] = original_profile; }
        assert(!lcd_config_execute(request, 1000U).accepted);
        lcd_config_no_actuation();
    }
    puts("SINGLE_STATIC_LCD_EVIDENCE_PASS reversible_changes=16 unconfigured_tracking=1 ordinary_reference_preserved=1");
}

/** @brief Unrelated reservations, stale locks, public provisioning and restart retain their scope. */
static void lcd_config_scope_and_pressure(void)
{
    DebugUiSnapshot snapshot;
    DebugUiSingleStaticProfile profile = {1U, 1000U, 60000U, 0.0F};
    DebugUiRequest request = lcd_config_fixture(7U, 60000U);
    DebugUiRequest stale_lock;
    DebugUiRequest unrelated = test_request(DEBUG_UI_OPERATION_ACQUIRE, 1000U);
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    assert(aethor_app_debug_ui_submit(&unrelated) == DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_process(1000U);
    assert(aethor_app_debug_ui_poll_admission(&admission));
    assert(aethor_app_debug_ui_poll_completion(&completion));
    application_debug_ui.authority = DEBUG_UI_AUTHORITY_REMOTE;
    request = lcd_lock_request(7U); request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE;
    request.hold_duration_ms = 60000U;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    assert(!aethor_app_debug_ui_set_single_static_profile(7U, &profile));
    application_debug_ui.mailbox.transactions[1].used = 1U;
    application_debug_ui.mailbox.transactions[1].taken = 1U;
    application_debug_ui.mailbox.transactions[1].completion_ready = 1U;
    aethor_app_debug_ui_process(1001U);
    assert(aethor_app_debug_ui_poll_admission(&admission) && !admission.accepted);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    request = lcd_config_fixture(7U, 60000U);
    assert(lcd_config_execute(request, 1000U).accepted);
    stale_lock = lcd_lock_request(7U);
    request = lcd_lock_request(7U); request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE;
    request.hold_duration_ms = 1000U;
    assert(lcd_config_execute(request, 1001U).accepted);
    stale_lock.identity.request_id = ++test_request_id;
    assert(!lcd_config_execute(stale_lock, 1001U).accepted);
    assert(application_debug_ui.single_static_profiles[6].configured);
    request = lcd_lock_request(1U);
    assert(!lcd_config_execute(request, 1001U).accepted);
    assert(application_debug_ui.single_static_profiles[6].configured);
    request = lcd_lock_request(7U);
    application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    application_debug_ui.target_motor_id = 7U;
    assert(lcd_config_execute(request, 1001U).accepted);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE);
    request = lcd_lock_request(7U);
    assert(lcd_config_execute(request, 1001U).accepted);
    assert(aethor_app_debug_ui_set_single_static_profile(7U, &profile));
    request = lcd_lock_request(7U);
    assert(lcd_config_execute(request, 1001U).accepted);
    assert(application_debug_ui.single_static_profiles[6].configured);
    assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
    assert(!snapshot.motors[6].single_static_lcd_owned);
    aethor_app_init(1U, 77U);
    assert(aethor_app_debug_ui_get_snapshot(1U, &snapshot));
    assert(!snapshot.motors[6].single_static_profile.configured && !snapshot.motors[6].single_static_lcd_owned);
    puts("SINGLE_STATIC_LCD_SCOPE_PASS unrelated_slot_rejected=1 stale_lock_rejected=1 cross_id_blocked=1 public_zero_results=1 restart_clears=1");
}

/** @brief A STOP queued with a reviewed configuration always prevents its later installation. */
static void lcd_config_stop_race(void)
{
    DebugUiRequest request = lcd_config_fixture(2U, 60000U);
    DebugUiRequest stop = test_request(DEBUG_UI_OPERATION_STOP, 1000U);
    DebugUiAdmission admission;
    uint8_t rejected_configuration = 0U;
    stop.target_motor_id = 2U;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_process(1000U);
    while (aethor_app_debug_ui_poll_admission(&admission))
    {
        if (admission.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE)
        { assert(!admission.accepted); rejected_configuration = 1U; }
    }
    assert(rejected_configuration && !application_debug_ui.single_static_profiles[1].configured);
    {
        DebugUiRequest reviewed = lcd_config_fixture(2U, 60000U);
        DebugUiCompletion completion;
        stop = test_request(DEBUG_UI_OPERATION_STOP, 1000U);
        stop.target_motor_id = 2U;
        assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_NONE);
        aethor_app_debug_ui_process(1000U);
        assert(aethor_app_debug_ui_poll_admission(&admission) && admission.accepted);
        /* Simulate the unrelated STOP terminal being fully drained, retaining the review barrier. */
        memset(&completion, 0, sizeof(completion));
        completion.identity = stop.identity;
        completion.operation = stop.operation;
        assert(debug_ui_mailbox_complete(&application_debug_ui.mailbox, &completion));
        assert(aethor_app_debug_ui_poll_completion(&completion));
        application_protocol_engine.stop_read_sequence = application_protocol_engine.stop_write_sequence;
        application_protocol_engine.active_stop_request_id = 0U;
        reviewed.identity.request_id = ++test_request_id;
        assert(lcd_config_execute(reviewed, 1001U).reason == DEBUG_UI_REASON_OLD_EPOCH);
        assert(!application_debug_ui.single_static_profiles[1].configured);
    }
    {
        DebugUiRequest reviewed = lcd_config_fixture(2U, 60000U);
        DebugUiSnapshot snapshot;
        uint8_t slot_index;
        for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
        {
            application_debug_ui.mailbox.transactions[slot_index].used = 1U;
            application_debug_ui.mailbox.transactions[slot_index].taken = 1U;
            application_debug_ui.mailbox.transactions[slot_index].completion_ready = 1U;
        }
        stop = test_request(DEBUG_UI_OPERATION_STOP, 1000U);
        stop.target_motor_id = 2U;
        assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_STOP_LATCHED);
        assert(aethor_app_debug_ui_get_snapshot(1000U, &snapshot));
        assert(snapshot.motors[1].single_static_evidence_generation != reviewed.single_static_evidence_generation);
        /* After an independently exercised STOP drain, old review evidence must still be obsolete. */
        for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
        { application_debug_ui.mailbox.transactions[slot_index].used = 0U; }
        application_debug_ui.mailbox.stop_intent_mask = 0U;
        reviewed.identity.request_id = ++test_request_id;
        assert(lcd_config_execute(reviewed, 1001U).reason == DEBUG_UI_REASON_OLD_EPOCH);
    }
    puts("SINGLE_STATIC_LCD_STOP_RACE_PASS queued_stop_prevents_unlock=1 drained_and_saturated_reviews_rejected=1");
}

/** @brief Relock never releases an executing/unsafe workflow, including an empty configuration. */
static void lcd_lock_safety(void)
{
    uint8_t scenario;
    for (scenario = 0U; scenario < 6U; ++scenario)
    {
        DebugUiRequest request = lcd_config_fixture(4U, 60000U);
        assert(lcd_config_execute(request, 1000U).accepted);
        application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
        application_debug_ui.target_motor_id = 4U;
        request = lcd_lock_request(4U);
        if (scenario == 0U) { application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT; }
        if (scenario == 1U) { application_debug_ui.unconfirmed_disable_mask = 8U; }
        if (scenario == 2U) { application_motor_runtime.bank.motors[3].feedback.driver_state = S3519_DRIVER_STATE_ENABLED; }
        if (scenario == 3U) { application_motor_runtime.bank.motors[3].feedback_valid = 0U; }
        if (scenario == 4U) { application_debug_ui.mailbox.health.input_valid = 0U; }
        if (scenario == 5U)
        {
            application_debug_ui.single_static_profiles[3].configured = 0U;
            application_debug_ui.single_static_lcd_owned[3] = 0U;
            application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT;
        }
        assert(!lcd_config_execute(request, 1001U).accepted);
        assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    }
    puts("SINGLE_STATIC_LCD_LOCK_SAFETY_PASS busy_and_unsafe_cases=6 empty_busy_not_released=1");
}
#endif

/** @brief Execute the isolated LCD configuration slice with production app and mailbox. */
int main(void)
{
    lcd_config_compile_gate();
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    lcd_config_gain_cases();
    lcd_config_feedforward_rejections();
    lcd_config_install_and_lock();
    lcd_config_rejections();
    lcd_config_evidence_reversal();
    lcd_config_scope_and_pressure();
    lcd_config_stop_race();
    lcd_lock_safety();
#endif
    puts("SINGLE_STATIC_LCD_CONFIG_PASS");
    return 0;
}
