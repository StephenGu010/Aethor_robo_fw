/** @file debug_ui_soak_test_main.c
 * @brief Thirty-minute LOGICAL-TIME readonly App/input/model integration regression.
 * Production App is included unchanged only to initialize synthetic discovery data
 * and inspect bounded queues. Feedback enters the real CAN decoder. No motor evidence
 * profile is installed. Each run is serial deterministic dispatch, NOT hardware,
 * wallclock load, RTOS concurrency, graphics rendering, or 4 ms real-time acceptance.
 */
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "debug_ui_input.h"
#include "debug_ui_model.h"
#include "../../App/aethor_app.c"

#if !AETHOR_DEBUG_UI_ENABLE || AETHOR_DEBUG_UI_ALLOW_MOTION || AETHOR_DEBUG_UI_ALLOW_MIT
#error "The soak test must compile the actual readonly configuration."
#endif
#ifdef NDEBUG
#error "Soak assertions must remain enabled."
#endif

#define SOAK_DURATION_MS (1800000U)
#define SOAK_START_US (1000000ULL)

/** @brief Counts actual serial test dispatches; none are wallclock timing measurements. */
typedef struct
{
    uint32_t arm_ticks;
    uint32_t ui_ticks;
    uint32_t health_reports;
    uint32_t protocol_passes;
    uint32_t usb_queries;
    uint32_t usb_hellos;
    uint32_t feedback_frames;
    uint32_t input_events;
    uint32_t page_transitions;
    uint32_t pages_mask;
    uint32_t action_focus_mask;
    uint32_t denied_actions_mask;
    uint32_t motors_mask;
    uint32_t local_requests;
    uint32_t stale_probe_sets;
    uint32_t identity_probe_sets;
    uint32_t critical_entries;
    uint32_t input_queue_high_watermark;
    uint32_t motor_value_changes[DEBUG_UI_MOTOR_COUNT];
    uint64_t arm_state_digest;
} DebugUiSoakStatistics;

static uint32_t soak_critical_depth;
static uint32_t soak_critical_entries;

/** @brief Accounts for critical-hook use without pretending to simulate preemption. */
static void soak_enter_critical(void)
{
    ++soak_critical_depth;
    ++soak_critical_entries;
}

/** @brief Checks that nested bounded critical copies are paired correctly. */
static void soak_exit_critical(void)
{
    assert(soak_critical_depth != 0U);
    --soak_critical_depth;
}

/** @brief Seeds explicitly synthetic discovery results; local motor profiles stay invalid. */
static void soak_initialize(void)
{
    uint8_t joint_index;
    aethor_app_init(SOAK_START_US, 0x50A8U);
    soak_critical_depth = soak_critical_entries = 0U;
    aethor_app_set_task_critical_hooks(soak_enter_critical, soak_exit_critical);
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        MotorDiscoveryResult *result = &application_motor_runtime.discovery.results[joint_index];
        result->observed_esc_id = application_motor_runtime.configuration->joints[joint_index].esc_id;
        result->observed_master_id = application_motor_runtime.configuration->joints[joint_index].master_id;
        result->observed_control_mode = 2U;
        result->verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
        result->ranges.position_max_rad = 12.5F;
        result->ranges.velocity_max_rad_s = 45.0F;
        result->ranges.torque_max_nm = 18.0F;
        result->maximum_speed_rad_s = 20.0F;
    }
    application_motor_runtime.discovery.verified_joint_mask = 0x7FU;
}

/** @brief Publishes seven varying, disabled feedback frames through production decode. */
static void soak_feedback(uint32_t cycle, uint64_t timestamp_us, DebugUiSoakStatistics *statistics)
{
    uint8_t joint_index;
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        uint16_t encoded_position = (uint16_t)(30000U + (cycle * 37U + joint_index * 701U) % 6000U);
        uint16_t encoded_velocity = (uint16_t)(2030U + (cycle + joint_index) % 35U);
        uint16_t encoded_torque = (uint16_t)(2010U + (cycle + 3U * joint_index) % 70U);
        uint8_t payload[8];
        CanFrame frame;
        payload[0] = (uint8_t)application_motor_runtime.configuration->joints[joint_index].esc_id;
        payload[1] = (uint8_t)(encoded_position >> 8U);
        payload[2] = (uint8_t)encoded_position;
        payload[3] = (uint8_t)(encoded_velocity >> 4U);
        payload[4] = (uint8_t)(((encoded_velocity & 15U) << 4U) | (encoded_torque >> 8U));
        payload[5] = (uint8_t)encoded_torque;
        payload[6] = (uint8_t)(25U + (cycle + joint_index) % 15U);
        payload[7] = (uint8_t)(24U + (cycle + joint_index) % 12U);
        assert(can_frame_init(&frame,
            application_motor_runtime.configuration->joints[joint_index].master_id,
            payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
        assert(aethor_app_receive_can_frame(&frame, timestamp_us) == MOTOR_RUNTIME_STATUS_OK);
        ++statistics->feedback_frames;
    }
}

/** @brief Exercises USB session/query identity; unsolicited LOCAL results are forbidden. */
static void soak_usb(const char *verb, uint32_t request_id, uint64_t timestamp_us)
{
    char line[96];
    ProtocolOutputBatch output;
    uint8_t message_index;
    int length = snprintf(line, sizeof(line), "%" PRIu32 " %s", request_id, verb);
    assert(length > 0 && (size_t)length < sizeof(line));
    assert(aethor_app_process_protocol_line(line, (size_t)length, timestamp_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(output.count > 0U && output.count <= PROTOCOL_ENGINE_MAX_OUTPUT_COUNT);
    for (message_index = 0U; message_index < output.count; ++message_index)
    {
        char response_kind[16];
        unsigned long response_id;
        assert(output.messages[message_index].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
        assert(sscanf(output.messages[message_index].data, "%15s %lu", response_kind, &response_id) == 2);
        assert(strcmp(response_kind, "ok") == 0);
        assert(response_id == request_id);
    }
}

/** @brief Converts a repeatable navigation script into 100 ms ADC presses plus releases. */
static uint16_t soak_adc(uint32_t elapsed_ms)
{
    static const DebugUiKey keys[] = {
        DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_DOWN,
        DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT,
        DEBUG_UI_KEY_UP, DEBUG_UI_KEY_UP, DEBUG_UI_KEY_UP, DEBUG_UI_KEY_UP,
        DEBUG_UI_KEY_UP, DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN,
        DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT,
        DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT,
        DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_UP, DEBUG_UI_KEY_RIGHT,
        DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT,
        DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_UP,
        DEBUG_UI_KEY_UP, DEBUG_UI_KEY_UP, DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT,
        DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN,
        DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT,
        DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_UP,
        DEBUG_UI_KEY_UP, DEBUG_UI_KEY_UP, DEBUG_UI_KEY_UP, DEBUG_UI_KEY_UP,
        DEBUG_UI_KEY_UP, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT,
        DEBUG_UI_KEY_UP, DEBUG_UI_KEY_UP, DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT,
        DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN,
        DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN,
        DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_LEFT, DEBUG_UI_KEY_DOWN
    };
    DebugUiKey key;
    uint32_t script_ms;
    if (elapsed_ms < 500U) { return 4095U; }
    script_ms = elapsed_ms - 500U;
    if (script_ms % 250U >= 100U) { return 4095U; }
    key = keys[(script_ms / 250U) % (sizeof(keys) / sizeof(keys[0]))];
    switch (key)
    {
        case DEBUG_UI_KEY_CENTER: return 0U;
        case DEBUG_UI_KEY_LEFT: return 816U;
        case DEBUG_UI_KEY_RIGHT: return 1636U;
        case DEBUG_UI_KEY_UP: return 2457U;
        case DEBUG_UI_KEY_DOWN: return 3279U;
        default: return 4095U;
    }
}

/** @brief Checks expiry and source rejection on isolated copies, without live requests. */
static void soak_expiry_and_identity_probes(const DebugUiInput *input, const DebugUiModel *model,
    uint32_t now_ms, uint64_t timestamp_us, uint32_t request_id, DebugUiSoakStatistics *statistics)
{
    DebugUiInput expired_input = *input;
    DebugUiMailbox copied_mailbox;
    DebugUiModel identity_model = *model;
    DebugUiModel before;
    DebugUiCompletion completion;
    DebugUiAdmission admission;
    MotorFeedbackSnapshot stale_feedback;
    uint8_t joint_index;
    aethor_app_enter_task_critical();
    copied_mailbox = application_debug_ui.mailbox;
    aethor_app_exit_task_critical();
    assert(debug_ui_mailbox_healthy(&copied_mailbox, timestamp_us));
    assert(!debug_ui_mailbox_healthy(&copied_mailbox, timestamp_us + DEBUG_UI_HEALTH_TIMEOUT_US + 1ULL));
    copied_mailbox.health.ui_timestamp_us = timestamp_us + DEBUG_UI_HEALTH_TIMEOUT_US + 1ULL;
    assert(!debug_ui_mailbox_healthy(&copied_mailbox, copied_mailbox.health.ui_timestamp_us));
    copied_mailbox.health.protocol_timestamp_us = copied_mailbox.health.ui_timestamp_us;
    copied_mailbox.health.ui_timestamp_us = timestamp_us;
    assert(!debug_ui_mailbox_healthy(&copied_mailbox, copied_mailbox.health.protocol_timestamp_us));
    debug_ui_input_poll(&expired_input, now_ms + DEBUG_UI_INPUT_INVALID_MS + 1U);
    assert(!expired_input.valid);
    assert(motor_bank_get_snapshot(&application_motor_runtime.bank,
        timestamp_us + MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US + 1ULL,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &stale_feedback) == MOTOR_BANK_STATUS_OK);
    assert(stale_feedback.valid_joint_mask == 0U);
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        /* Expiry retains the last displayed value, in LCD output-shaft units. */
        float displayed_position = stale_feedback.joints[joint_index].position_rad;
#if AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
        if (joint_index == 6U) { displayed_position /= 19.2032F; }
#endif
        assert(displayed_position == model->snapshot.motors[joint_index].position_rad);
    }
    ++statistics->stale_probe_sets;

    /* A hypothetical pending identity exists only in this isolated model copy. */
    identity_model.submitted.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    identity_model.submitted.identity.epoch = model->snapshot.epoch;
    identity_model.submitted.identity.request_id = request_id;
    identity_model.awaiting_result = 1U;
    before = identity_model;
    memset(&completion, 0, sizeof(completion));
    memset(&admission, 0, sizeof(admission));
    completion.identity = identity_model.submitted.identity;
    completion.identity.origin = DEBUG_UI_ORIGIN_USB;
    admission.identity = completion.identity;
    admission.accepted = 1U;
    debug_ui_model_admission(&identity_model, &admission);
    debug_ui_model_completion(&identity_model, &completion);
    assert(memcmp(&before, &identity_model, sizeof(before)) == 0);
    completion.identity = identity_model.submitted.identity;
    ++completion.identity.epoch;
    debug_ui_model_completion(&identity_model, &completion);
    assert(memcmp(&before, &identity_model, sizeof(before)) == 0);
    ++statistics->identity_probe_sets;
}

/** @brief Enforces bounded empty control/result channels on every 4 ms Arm tick. */
static void soak_check_control(DebugUiSoakStatistics *statistics)
{
    ArmSnapshot arm;
    assert(aethor_app_get_snapshot(&arm));
    assert(application_action.state == AETHOR_APP_ACTION_IDLE);
    assert(application_protocol_engine.command_write_sequence == application_protocol_engine.command_read_sequence);
    assert(application_protocol_engine.stop_write_sequence == application_protocol_engine.stop_read_sequence);
    assert(application_protocol_engine.result_write_sequence == application_protocol_engine.result_read_sequence);
    assert(application_protocol_engine.active_motion_request_id == 0U);
    assert(application_protocol_engine.active_stop_request_id == 0U);
    assert(aethor_app_deferred_result_count() == 0U);
    assert(debug_ui_mailbox_result_count(&application_debug_ui.mailbox) == 0U);
    assert(!application_protocol_engine.watchdog_timeout_reported);
    assert(!arm.enabled && !arm.moving);
    assert(application_emergency_disable_batch.count == 0U);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE);
    assert(application_debug_ui.epoch == 1U);
    statistics->arm_state_digest ^= (uint64_t)arm.state | ((uint64_t)arm.fault << 8U) |
        ((uint64_t)arm.control_mode << 16U) | ((uint64_t)arm.aligned << 24U);
    statistics->arm_state_digest *= UINT64_C(1099511628211);
}

/** @brief Runs the same 30 minute schedule with or without production input/model dispatch. */
static DebugUiSoakStatistics soak_run(uint8_t with_ui)
{
    DebugUiSoakStatistics statistics;
    DebugUiInput input;
    DebugUiModel model;
    float previous_position[DEBUG_UI_MOTOR_COUNT] = {0.0F};
    uint32_t elapsed_ms;
    uint32_t request_id = 1U;
    uint32_t sample_sequence = 0U;
    uint32_t health_sequence = 0U;
    memset(&statistics, 0, sizeof(statistics));
    statistics.arm_state_digest = UINT64_C(1469598103934665603);
    soak_initialize();
    debug_ui_input_init(&input, (uint32_t)(SOAK_START_US / 1000ULL));
    debug_ui_model_init(&model);
    soak_usb("hello", request_id++, SOAK_START_US);
    ++statistics.usb_hellos;
    for (elapsed_ms = 0U; elapsed_ms <= SOAK_DURATION_MS; ++elapsed_ms)
    {
        uint64_t timestamp_us = SOAK_START_US + (uint64_t)elapsed_ms * 1000ULL;
        uint32_t now_ms = (uint32_t)(timestamp_us / 1000ULL);
        if (elapsed_ms % 100U == 0U)
        {
            char query[48];
            uint32_t cycle = elapsed_ms / 100U;
            soak_feedback(cycle, timestamp_us, &statistics);
            if (cycle % 4U == 0U) { strcpy(query, "show state"); }
            else if (cycle % 4U == 1U) { strcpy(query, "show motors"); }
            else if (cycle % 4U == 2U) { strcpy(query, "show diag"); }
            else { (void)snprintf(query, sizeof(query), "show motor %u", (unsigned)(cycle % 7U + 1U)); }
            soak_usb(query, request_id++, timestamp_us);
            ++statistics.usb_queries;
            if ((elapsed_ms != 0U) && (elapsed_ms % 60000U == 0U))
            { soak_usb("hello", request_id++, timestamp_us); ++statistics.usb_hellos; }
        }
        if (elapsed_ms % 4U == 0U)
        {
            assert(aethor_app_service(timestamp_us) == 0U);
            soak_check_control(&statistics);
            ++statistics.arm_ticks;
        }
        if (with_ui && elapsed_ms % 5U == 0U)
        {
            DebugUiSnapshot snapshot;
            DebugUiInputEvent event;
            DebugUiRequest request;
            DebugUiPage previous_page = model.page;
            uint8_t joint_index;
            assert(debug_ui_input_feed(&input, soak_adc(elapsed_ms), ++sample_sequence, now_ms, now_ms));
            debug_ui_input_poll(&input, now_ms);
            if (input.event_count > statistics.input_queue_high_watermark)
            { statistics.input_queue_high_watermark = input.event_count; }
            assert(input.event_count <= DEBUG_UI_INPUT_EVENT_CAPACITY);
            assert(aethor_app_debug_ui_get_snapshot(timestamp_us, &snapshot));
            assert(!snapshot.motion_enabled && !snapshot.mit_enabled);
            assert(!snapshot.active && !snapshot.pending && !snapshot.stop_pending);
            for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
            {
                const DebugUiMotorView *motor = &snapshot.motors[joint_index];
                assert(motor->feedback_seen && motor->feedback_valid && !motor->enabled);
                assert(motor->motor_id == joint_index + 1U);
                assert(!motor->profile.pos_valid && !motor->profile.mit_valid);
                if (previous_position[joint_index] != motor->position_rad)
                { ++statistics.motor_value_changes[joint_index]; previous_position[joint_index] = motor->position_rad; }
            }
            debug_ui_model_update(&model, &snapshot, timestamp_us, input.valid, 1U);
            while (debug_ui_input_event(&input, &event))
            {
                uint8_t denied_action_focus = 255U;
                if (model.page == DEBUG_UI_PAGE_ACTIONS && model.focus >= 1U && model.focus <= 3U)
                    denied_action_focus = (uint8_t)(model.focus - 1U);
                else if (model.page == DEBUG_UI_PAGE_MODES) denied_action_focus = (uint8_t)(3U + model.focus);
                else if (model.page == DEBUG_UI_PAGE_RECOVERY && model.focus < 2U)
                    denied_action_focus = (uint8_t)(5U + model.focus);
                {
                    uint8_t attempts_control = (uint8_t)(denied_action_focus < 7U && event.type == DEBUG_UI_INPUT_EVENT_PRESS &&
                        (event.key == DEBUG_UI_KEY_CENTER || event.key == DEBUG_UI_KEY_RIGHT));
                    debug_ui_model_event(&model, &event);
                    ++statistics.input_events;
                    if (attempts_control) {
                        assert(model.page == DEBUG_UI_PAGE_NOTICE);
                        assert(model.reason == DEBUG_UI_REASON_DISABLED);
                        statistics.denied_actions_mask |= (uint32_t)(1U << denied_action_focus);
                    }
                }
            }
            assert(!debug_ui_input_emergency_event(&input, &event));
            while (debug_ui_model_take_request(&model, &request)) { ++statistics.local_requests; }
            assert(statistics.local_requests == 0U);
            assert(model.page < DEBUG_UI_PAGE_COUNT && model.selected_motor < DEBUG_UI_MOTOR_COUNT);
            statistics.pages_mask |= (uint32_t)(1U << model.page);
            if (model.page == DEBUG_UI_PAGE_ACTIONS)
            {
                assert(model.focus < 6U);
                statistics.action_focus_mask |= (uint32_t)(1U << model.focus);
            }
            statistics.motors_mask |= (uint32_t)(1U << model.selected_motor);
            if (previous_page != model.page) { ++statistics.page_transitions; }
            ++statistics.ui_ticks;
            if (elapsed_ms % 50U == 0U)
            {
                DebugUiHealth health;
                memset(&health, 0, sizeof(health));
                health.ui_timestamp_us = timestamp_us;
                health.ui_sequence = ++health_sequence;
                health.input_valid = input.valid;
                health.display_valid = health.ui_valid = 1U;
                aethor_app_debug_ui_update_health(&health);
                assert(aethor_app_debug_ui_needs_service());
                ++statistics.health_reports;
            }
            aethor_app_debug_ui_process(timestamp_us);
            ++statistics.protocol_passes;
            if (elapsed_ms >= 500U)
            {
                assert(debug_ui_mailbox_healthy(&application_debug_ui.mailbox, timestamp_us));
                assert(!aethor_app_debug_ui_needs_service());
            }
            if ((elapsed_ms != 0U) && (elapsed_ms % 60000U == 0U))
            { soak_expiry_and_identity_probes(&input, &model, now_ms, timestamp_us, request_id, &statistics); }
        }
        assert(soak_critical_depth == 0U);
    }
    assert(application_diagnostics.counters.control_deadline_miss_count == 0U);
    assert(application_motor_runtime.rejected_feedback_count == 0U);
    statistics.critical_entries = soak_critical_entries;
    return statistics;
}

/** @brief Requires page coverage, all seven changing rows, and identical Arm baseline digest. */
int main(void)
{
    DebugUiSoakStatistics baseline = soak_run(0U);
    DebugUiSoakStatistics with_ui = soak_run(1U);
    uint8_t joint_index;
    assert(baseline.arm_state_digest == with_ui.arm_state_digest);
    assert(with_ui.arm_ticks == SOAK_DURATION_MS / 4U + 1U);
    assert(with_ui.ui_ticks == SOAK_DURATION_MS / 5U + 1U);
    fprintf(stderr, "SOAK_PAGE_COVERAGE mask=%" PRIu32 " transitions=%" PRIu32 "\n",
        with_ui.pages_mask, with_ui.page_transitions);
    assert(with_ui.pages_mask == ((1U << DEBUG_UI_PAGE_OVERVIEW) | (1U << DEBUG_UI_PAGE_MOTORS) |
        (1U << DEBUG_UI_PAGE_DETAIL) | (1U << DEBUG_UI_PAGE_DIAGNOSTICS) |
        (1U << DEBUG_UI_PAGE_PREPARE) | (1U << DEBUG_UI_PAGE_REGISTERS) |
        (1U << DEBUG_UI_PAGE_ACTIONS) | (1U << DEBUG_UI_PAGE_MODES) |
        (1U << DEBUG_UI_PAGE_RECOVERY) | (1U << DEBUG_UI_PAGE_NOTICE) |
        (1U << DEBUG_UI_PAGE_DIAGNOSTIC_MENU)));
    assert(with_ui.action_focus_mask == 0x3FU && with_ui.denied_actions_mask == 0x7FU);
    assert(with_ui.motors_mask == 0x7FU && with_ui.page_transitions > 1000U);
    assert(with_ui.stale_probe_sets == 30U && with_ui.identity_probe_sets == 30U);
    printf("DEBUG_UI_SOAK_PASS logical_duration_ms=%u logical_iterations=%u readonly=1 baseline_equal=1\n",
        SOAK_DURATION_MS, SOAK_DURATION_MS + 1U);
    printf("arm_ticks=%" PRIu32 " ui_ticks=%" PRIu32 " protocol_passes=%" PRIu32 " health_reports=%" PRIu32 "\n",
        with_ui.arm_ticks, with_ui.ui_ticks, with_ui.protocol_passes, with_ui.health_reports);
    printf("usb_queries=%" PRIu32 " usb_hellos=%" PRIu32 " feedback_frames=%" PRIu32 " local_requests=%" PRIu32 "\n",
        with_ui.usb_queries, with_ui.usb_hellos, with_ui.feedback_frames, with_ui.local_requests);
    printf("input_events=%" PRIu32 " pages_mask=%" PRIu32 " page_transitions=%" PRIu32 " motors_mask=%" PRIu32 " input_queue_high_watermark=%" PRIu32 "\n",
        with_ui.input_events, with_ui.pages_mask, with_ui.page_transitions, with_ui.motors_mask, with_ui.input_queue_high_watermark);
    printf("action_focus_mask=%" PRIu32 " denied_actions_mask=%" PRIu32 "\n",
        with_ui.action_focus_mask, with_ui.denied_actions_mask);
    puts("command_queue_nonempty_ticks=0 stop_queue_nonempty_ticks=0 result_queue_nonempty_ticks=0 local_authority_transitions=0");
    printf("expiry_probe_sets=%" PRIu32 " identity_probe_sets=%" PRIu32 " critical_entries=%" PRIu32 " arm_digest=%" PRIu64 "\n",
        with_ui.stale_probe_sets, with_ui.identity_probe_sets, with_ui.critical_entries, with_ui.arm_state_digest);
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        assert(with_ui.motor_value_changes[joint_index] >= 18000U);
        printf("motor_%u_value_changes=%" PRIu32 "\n", (unsigned)(joint_index + 1U), with_ui.motor_value_changes[joint_index]);
    }
    puts("Evidence: deterministic serial logical-time software regression only; no hardware, RTOS timing or wallclock-load acceptance.");
    return 0;
}
