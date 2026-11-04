/** @file adrc_shaft_pilot_test_main.c
 * @brief Synthetic trusted bridge fixtures; no hardware, physical send, model or qualification evidence. */
#include "adrc_app_bridge.h"
#include "arm_config.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static AdrcAppBridge bridge;
static MotorRuntime runtime;
static ProtocolEngine engine;
static uint64_t clock_us;
static uint32_t guard_deadline, guard_fired, guard_hard;
static unsigned renew_count, ack_count, groups;
static uint8_t guard_state, owner_active;
static uint8_t real_feedback_codec;
static float codec_native_position, codec_raw_velocity;
/** @brief 用实际MIT编码和反馈解码构造离线报文，量程来自同一发现身份。 */
static S3519Feedback decode_codec_feedback(float native_position, float raw_velocity, uint8_t enabled)
{
    CanFrame command, frame;
    S3519Feedback decoded;
    const S3519Ranges *ranges = &runtime.discovery.results[6].ranges;
    assert(s3519_pack_mit(7U, ranges, native_position, raw_velocity, 0.0F, 0.0F, 0.0F, &command) == S3519_CODEC_STATUS_OK);
    memset(&frame, 0, sizeof(frame)); frame.length = 8U;
    frame.data[0] = (uint8_t)((enabled ? 0x10U : 0U) | 7U);
    frame.data[1] = command.data[0]; frame.data[2] = command.data[1]; frame.data[3] = command.data[2];
    frame.data[4] = (uint8_t)((command.data[3] & 0xF0U) | (command.data[6] & 0x0FU));
    frame.data[5] = command.data[7]; frame.data[6] = 30U; frame.data[7] = 28U;
    assert(s3519_decode_feedback(&frame, ranges, &decoded) == S3519_CODEC_STATUS_OK);
    return decoded;
}
/** @brief Simulates the actual owner query only inside this offline fixture. */
static uint8_t owner(void) { return owner_active; }
/** @brief Simulates finite arm; captures exact 12ms lease and nonrenewable 2s hard cap. */
static uint8_t arm(const CanFrame *frame, uint32_t now)
{ assert(frame->identifier == 7U && frame->data[7] == S3519_MODE_COMMAND_DISABLE);
  guard_state = 1U; guard_deadline = now + 12U; guard_hard = now + 2000U; guard_fired = 0U; return 1U; }
/** @brief Simulates real board lease renewal without extending the hard cap. */
static uint8_t renew(uint32_t now)
{ assert(guard_state == 1U); ++renew_count; guard_deadline = now + 12U < guard_hard ? now + 12U : guard_hard; return 1U; }
/** @brief Simulates exact scheduled guard firing and supports deliberate early/late fault fixtures. */
static uint8_t observe(uint32_t *deadline, uint32_t *fired)
{ if (guard_state == 1U && clock_us / 1000ULL >= guard_deadline) { guard_state = 3U; guard_fired = guard_deadline; }
  *deadline = guard_deadline; *fired = guard_fired; return guard_state; }
/** @brief Records actual acknowledgements in the offline timer double. */
static uint8_t acknowledge(void) { assert(guard_state == 3U); ++ack_count; guard_state = 0U; return 1U; }
/** @brief Injects new selected normal feedback and separately its immutable decode observation upper bound. */
static void feedback(uint64_t now, uint8_t enabled)
{
    uint32_t before = runtime.accepted_feedback_count;
    MotorJointFeedback *value = &runtime.bank.motors[6].feedback;
    clock_us = now; value->timestamp_us = now; value->position_rad = 0.0F;
    value->velocity_rad_s = -0.0488433838F; value->torque_nm = 0.0024420024F;
    value->mos_temperature_c = 30.0F; value->rotor_temperature_c = 28.0F;
    value->driver_state = enabled ? S3519_DRIVER_STATE_ENABLED : S3519_DRIVER_STATE_DISABLED;
    if (real_feedback_codec)
    {
        S3519Feedback decoded = decode_codec_feedback(codec_native_position, codec_raw_velocity, enabled);
        value->position_rad = decoded.position_rad; value->velocity_rad_s = decoded.velocity_rad_s;
        value->torque_nm = decoded.torque_nm; value->mos_temperature_c = decoded.mos_temperature_c;
        value->rotor_temperature_c = decoded.rotor_temperature_c; value->driver_state = decoded.state;
    }
    value->fault_flags = 0U; runtime.bank.motors[6].feedback_valid = 1U; ++runtime.accepted_feedback_count;
    adrc_shaft_adapter_note_feedback(&bridge, before, now, now + 1000ULL);
}
/** @brief Constructs offline-only same-session 64 samples and six readbacks with recorded scalar arithmetic. */
static void fixture(void)
{
    unsigned index;
    AdrcSimpleGuardHooks hooks;
    MotorDiscoveryResult *discovery;
    float cap_values[6] = {0.0F,100.0F,0.8F,0.000169623876F,0.0000181681371F,19.20319938659668F};
    real_feedback_codec = 0U; codec_native_position = codec_raw_velocity = 0.0F;
    protocol_engine_init(&engine, 123U);
    assert(motor_runtime_init(&runtime, arm_config_get_production()) == MOTOR_RUNTIME_STATUS_OK);
    assert(adrc_app_bridge_init_deferred(&bridge, &runtime, &engine, NULL, NULL) == ADRC_RESULT_OK);
    owner_active = 1U; bridge.capability_owner_active = owner; bridge.bench.draft_config.axis_index = 6U;
    discovery = &runtime.discovery.results[6]; memset(discovery, 0, sizeof(*discovery));
    discovery->verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    discovery->observed_master_id = 23U; discovery->observed_esc_id = 7U; discovery->observed_control_mode = 1U;
    discovery->ranges.position_max_rad = 12.5F; discovery->ranges.velocity_max_rad_s = 200.0F; discovery->ranges.torque_max_nm = 10.0F;
    runtime.discovery.verified_joint_mask = 0x40U; bridge.capability_discovery = *discovery;
    bridge.capture.state = 2U; bridge.capture.axis = 6U; bridge.capture.count = 64U;
    bridge.capture.observed_proven_count = 64U; bridge.capture.request_id = 1U;
    bridge.capture.owner_epoch = 1U; bridge.capture.discovery = *discovery;
    for (index = 0U; index < 64U; ++index)
    { bridge.capture.samples[index].rx_lower_us = 100000ULL + index * 4000ULL;
      bridge.capture.samples[index].observed_us = bridge.capture.samples[index].rx_lower_us + 1000ULL;
      bridge.capture.samples[index].position_rad = index == 8U ? 0.00038146973F : 0.0F;
      bridge.capture.samples[index].velocity_rad_s = -0.0488433838F;
      bridge.capture.samples[index].torque_nm = 0.0122098923F; }
    bridge.guard_test.state = 5U; bridge.guard_test.request_id = 2U; bridge.guard_test.owner_epoch = 1U;
    bridge.guard_test.hooks.observe = observe; bridge.guard_test.hooks.ack_disabled = acknowledge;
    hooks.arm_trial = arm; hooks.renew_trial = renew; adrc_app_bridge_set_simple_guard_hooks(&bridge, &hooks);
    bridge.capabilities.state = MOTOR_CAPABILITY_COMPLETE; bridge.capabilities.count = 6U;
    bridge.capabilities.used = 1U; bridge.capabilities.request_id = 3U; bridge.capabilities.epoch = 1U;
    for (index = 0U; index < 6U; ++index)
    { bridge.capabilities.items[index].valid = 1U; bridge.capabilities.items[index].value = cap_values[index];
      memcpy(&bridge.capabilities.items[index].raw_bits, &cap_values[index], sizeof(float)); }
    bridge.bench.gateway.highest_admitted_request_id = 3U;
    guard_state = 0U; renew_count = ack_count = 0U;
    feedback(400000ULL, 0U); clock_us = 401000ULL;
}
/** @brief Sends a public line and validates its bounded real formatter response. */
static ProtocolOutputBatch query(const char *line, ProtocolEngineStatus expected)
{
    ProtocolOutputBatch output;
    ProtocolEngineStatus actual = adrc_app_bridge_process_line(&bridge, line, strlen(line), clock_us, &output);
    if (actual != expected)
    { printf("query_mismatch line=%s actual=%u expected=%u reply=%s\n", line, (unsigned)actual, (unsigned)expected,
        output.count ? output.messages[0].data : "empty"); fflush(stdout); }
    assert(actual == expected);
    assert(output.count == 1U && output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
    return output;
}
/** @brief Transfers the actual bridge slot and records a timestamped matching successful completion. */
static uint8_t receipt(void)
{
    CanFrame frame; uint8_t kind; float decoded;
    assert(adrc_app_bridge_pop_frame(&bridge, &frame, &kind, &decoded));
    if (kind == 2U) { adrc_app_bridge_note_capability_disable_submit(&bridge, clock_us); }
    adrc_app_bridge_report_transmit_at(&bridge, kind, decoded, 1U, clock_us);
    return kind;
}
/** @brief Advances one legal tick with new selected feedback and returns one actual action kind. */
static uint8_t advance(uint64_t now, uint8_t enabled)
{ feedback(now, enabled); clock_us = now + 1000ULL; assert(adrc_app_bridge_service(&bridge, clock_us)); return receipt(); }
/** @brief Enters core only after a dedicated real disable completion and strictly newer receive sample. */
static void start(void)
{
    (void)query("4 adrc shaft_pilot direction=positive", PROTOCOL_ENGINE_STATUS_OK);
    assert(bridge.capability_terminal_closed && bridge.capabilities.state == MOTOR_CAPABILITY_COMPLETE);
    assert(adrc_app_bridge_service(&bridge, clock_us));
    assert(receipt() == 2U); assert(!bridge.shaft.guard_armed);
    assert(advance(404000ULL, 0U) == 0U); assert(bridge.shaft.core->status.phase == SHAFT_PHASE_ENABLE);
}
/** @brief Rejects missing/extra/variable plan fields, missing IDs and another pilot in the same boot. */
static void test_fixed_entry(void)
{
    const char *invalid[] = {"4 adrc shaft_pilot", "4 adrc shaft_pilot direction=positive cap=2", "4 adrc shaft_pilot direction=0",
        "4 adrc shaft_pilot direction=negative model=1", "adrc shaft_pilot direction=positive", "3 adrc shaft_pilot direction=positive"};
    unsigned index;
    for (index = 0U; index < sizeof(invalid)/sizeof(invalid[0]); ++index)
    { fixture(); (void)query(invalid[index], PROTOCOL_ENGINE_STATUS_BAD_REQUEST); assert(!bridge.shaft.used); }
    fixture(); bridge.shaft.released = 1U; start(); assert(!bridge.shaft.released);
    (void)query("5 adrc shaft_pilot direction=negative", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    ++groups;
}
/** @brief Checks baseline arithmetic in output coordinates and immutable capability handoff. */
static void test_baseline(void)
{
    fixture(); start();
    assert(fabsf(bridge.shaft.core->config.baseline.span64_shaft_rad - 1.9864904921325534e-5F) < 1e-10F);
    assert(fabsf(bridge.shaft.core->config.baseline.velocity32_abs_peak_rad_s - 0.00062077827879F) < 1e-8F);
    assert(fabsf(bridge.shaft.core->config.baseline.torque_abs_peak_nm - 0.0122098923F) < 1e-8F);
    assert(advance(408000ULL, 1U) == 1U);
    assert(!bridge.capabilities.completed_revoked && bridge.capabilities.state == MOTOR_CAPABILITY_COMPLETE);
    assert(bridge.shaft.core->status.phase == SHAFT_PHASE_NEUTRAL); ++groups;
}
/** @brief Repeated service and parameter decode counts never manufacture a new feedback observed upper bound. */
static void test_immutable_observation(void)
{
    uint64_t observation;
    fixture(); start(); assert(advance(408000ULL, 1U) == 1U); observation = bridge.shaft.observed_us;
    clock_us = 413000ULL; assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 1U);
    assert(bridge.shaft.observed_us == observation && renew_count == 1U);
    assert(advance(416000ULL, 1U) == 1U); assert(renew_count == 2U);
    adrc_shaft_adapter_note_feedback(&bridge, runtime.accepted_feedback_count, 420000ULL, 421000ULL);
    assert(bridge.shaft.observed_us == 417000ULL); ++groups;
}
/** @brief Every binding provenance change cancels motion without relaxing disabled collector monitoring. */
static void test_binding_changes(void)
{
    unsigned mutation;
    for (mutation = 0U; mutation < 7U; ++mutation)
    {
        fixture(); start(); feedback(408000ULL, 1U); clock_us = 409000ULL;
        if (mutation == 0U) { ++bridge.capability_epoch; }
        if (mutation == 1U) { ++engine.boot_id; }
        if (mutation == 2U) { owner_active = 0U; }
        if (mutation == 3U) { runtime.discovery.results[6].ranges.torque_max_nm = 9.0F; }
        if (mutation == 4U) { ++bridge.capabilities.items[5].raw_bits; }
        if (mutation == 5U) { bridge.capabilities.completed_revoked = 1U; }
        if (mutation == 6U) { bridge.bench.draft_config.axis_index = 0U; }
        assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
        assert(bridge.shaft.core->status.first_fault == SHAFT_FAULT_BINDING);
    }
    ++groups;
}
/** @brief Untimed, wrong-kind and late receipts never prove the pilot's previous actual motion. */
static void test_receipt_rejection(void)
{
    unsigned mutation;
    for (mutation = 0U; mutation < 4U; ++mutation)
    {
        CanFrame frame; uint8_t kind; float decoded;
        fixture(); (void)query("4 adrc shaft_pilot direction=positive", PROTOCOL_ENGINE_STATUS_OK);
        assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
        feedback(404000ULL, 0U); clock_us = 405000ULL; assert(adrc_app_bridge_service(&bridge, clock_us));
        assert(adrc_app_bridge_pop_frame(&bridge, &frame, &kind, &decoded) && kind == 0U);
        if (mutation == 0U) { adrc_app_bridge_report_transmit(&bridge, kind, decoded, 1U); }
        if (mutation == 1U) { adrc_app_bridge_report_transmit_at(&bridge, 1U, decoded, 1U, clock_us); }
        if (mutation == 2U) { adrc_app_bridge_report_transmit_at(&bridge, kind, decoded, 1U, clock_us - 1ULL); }
        if (mutation == 3U) { adrc_app_bridge_report_transmit_at(&bridge, kind, decoded, 1U, clock_us + 100000ULL); }
        assert(advance(408000ULL, 1U) == 2U); assert(bridge.shaft.core->status.first_fault == SHAFT_FAULT_RECEIPT);
    }
    ++groups;
}
/** @brief Retains the final real decoded receipt when STOP, transport failure or mutation arrives in the same beat. */
static void test_last_receipt_stop(void)
{
    unsigned mutation;
    for (mutation = 0U; mutation < 3U; ++mutation)
    {
        ShaftPilotTrace record;
        fixture(); start(); assert(advance(408000ULL, 1U) == 1U);
        assert(bridge.shaft.receipt.valid && bridge.shaft.receipt.kind == SHAFT_ACTION_TORQUE);
        if (mutation == 0U) { adrc_app_bridge_request_stop(&bridge); }
        if (mutation == 1U) { adrc_app_bridge_report_fault(&bridge); }
        if (mutation == 2U) { (void)query("5 adrc simple profile=8", PROTOCOL_ENGINE_STATUS_BAD_REQUEST); }
        assert(advance(412000ULL, 1U) == 2U);
        assert(shaft_pilot_trace_get(bridge.shaft.core, bridge.shaft.core->status.trace_count - 1U, &record));
        assert(record.input.receipt.valid && record.input.receipt.kind == SHAFT_ACTION_TORQUE);
        assert(bridge.shaft.core->status.peak_successful_decoded_torque_nm > 0.0F);
    }
    ++groups;
}
/** @brief Finite preparation always sends DISABLE and requires first submit, matching completion and newer RX. */
static void test_preparation(void)
{
    uint64_t now;
    fixture(); (void)query("4 adrc shaft_pilot direction=positive", PROTOCOL_ENGINE_STATUS_OK);
    for (now = 405000ULL; now <= 469000ULL; now += 4000ULL)
    { clock_us = now; bridge.frame_pending = 0U; bridge.awaiting_receipt = 0U;
      assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U); }
    assert(bridge.shaft.prepare == 3U && bridge.shaft.core->status.trace_count == 0U);
    assert(!bridge.shaft.cleanup_safe_release); ++groups;
}
/** @brief Natural end still requires exact guard fire and acknowledgement after real healthy disabled stop evidence. */
static void test_natural_and_guard(void)
{
    uint64_t now;
    unsigned before;
    fixture(); start();
    for (now = 408000ULL; now <= 1400000ULL; now += 4000ULL)
    { uint8_t enabled = bridge.shaft.core->status.phase < SHAFT_PHASE_STOPPING;
      uint8_t kind = advance(now, enabled); assert(kind == (enabled ? (bridge.shaft.core->status.phase < SHAFT_PHASE_STOPPING ? 1U : 2U) : 2U)); }
    assert(bridge.shaft.core->status.outcome == SHAFT_END_TIME);
    printf("natural phase=%u fault=%u safe=%u ack=%u dwell=%llu stop=%llu disabled=%u trace=%lu bridge=%u\n", (unsigned)bridge.shaft.core->status.phase, (unsigned)bridge.shaft.core->status.first_fault, bridge.shaft.core->status.safe_release,bridge.shaft.guard_ack,(unsigned long long)bridge.shaft.core->dwell_us,(unsigned long long)bridge.shaft.core->stop_us,bridge.shaft.core->disable_confirmed,(unsigned long)bridge.shaft.core->status.trace_count,bridge.shaft.bridge_fault); fflush(stdout);
    assert(bridge.shaft.core->status.phase == SHAFT_PHASE_DONE && bridge.shaft.core->status.safe_release);
    assert(bridge.shaft.guard_ack && ack_count == 1U && adrc_shaft_adapter_release_ready(&bridge));
    before = bridge.shaft.core->status.trace_count; assert(advance(1404000ULL, 0U) == 2U);
    assert(bridge.shaft.core->status.trace_count == before);
    ++groups;
}
/** @brief Wrong actual guard fire and rejected acknowledgements cannot grant release or another transaction. */
static void test_guard_mismatch(void)
{
    unsigned mutation, index;
    for (mutation = 0U; mutation < 2U; ++mutation)
    {
        fixture(); start(); assert(advance(408000ULL, 1U) == 1U);
        guard_state = 3U; guard_fired = mutation ? guard_deadline + 1U : guard_deadline - 1U;
        assert(advance(412000ULL, 1U) == 2U);
        for (index = 0U; index < 40U; ++index) { assert(advance(416000ULL + index * 4000ULL, 0U) == 2U); }
        assert(bridge.shaft.core->status.first_fault == SHAFT_FAULT_GUARD && bridge.shaft.core->status.safe_release);
        assert(!bridge.shaft.guard_ack && !ack_count && !adrc_shaft_adapter_release_ready(&bridge));
        guard_fired = guard_deadline; assert(advance(580000ULL, 0U) == 2U);
        assert(bridge.shaft.guard_ack && adrc_shaft_adapter_release_ready(&bridge));
        assert(!bridge.shaft.core->status.caller_next_transaction_ready);
    }
    ++groups;
}
/** @brief Missing genuine capture clocks, previous owner epoch and incomplete capabilities reject before consumption. */
static void test_admission_evidence(void)
{
    unsigned mutation;
    for (mutation = 0U; mutation < 8U; ++mutation)
    {
        fixture();
        if (mutation == 0U) { bridge.capture.observed_proven_count = 63U; }
        if (mutation == 1U) { bridge.capture.owner_epoch = 2U; }
        if (mutation == 2U) { bridge.guard_test.owner_epoch = 2U; }
        if (mutation == 3U) { bridge.capabilities.items[5].valid = 0U; }
        if (mutation == 4U) { bridge.capabilities.items[5].value = NAN; }
        if (mutation == 5U) { bridge.capture.samples[63].observed_us = 1ULL; }
        if (mutation == 6U) { bridge.capture.samples[63].torque_nm = 0.16F; }
        if (mutation == 7U) { runtime.discovery.results[6].communication_timeout_raw = 1U; }
        (void)query("4 adrc shaft_pilot direction=positive", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!bridge.shaft.used && !bridge.frame_pending);
        if (mutation < 7U) { assert(!bridge.capability_terminal_closed); }
        else { assert(bridge.capabilities.completed_revoked && bridge.capability_terminal_closed); }
    }
    ++groups;
}
/** @brief Failure preparation stops independently, recovers healthy dwell and ack, retaining no motion readiness. */
static void test_prepare_cleanup(void)
{
    unsigned index;
    fixture(); (void)query("4 adrc shaft_pilot direction=positive", PROTOCOL_ENGINE_STATUS_OK);
    clock_us = 465000ULL; assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
    assert(bridge.shaft.prepare == 3U && bridge.shaft.guard_armed && !bridge.shaft.core->status.trace_count);
    for (index = 0U; index < 48U; ++index) { assert(advance(468000ULL + index * 4000ULL, 0U) == 2U); }
    assert(bridge.shaft.cleanup_safe_release && bridge.shaft.guard_ack && adrc_shaft_adapter_release_ready(&bridge));
    (void)query("99 adrc shaft_status", PROTOCOL_ENGINE_STATUS_OK);
    assert(!bridge.shaft.core->status.caller_next_transaction_ready);
    ++groups;
}
/** @brief Exercises all exports at full-width timestamps and finite formatter extremes under the 520-byte contract. */
static void test_export_capacity(void)
{
    unsigned index, part; char command[100]; ShaftPilotTrace *trace;
    fixture(); start(); assert(advance(408000ULL, 1U) == 1U); adrc_app_bridge_request_stop(&bridge);
    for (index = 0U; index < 40U; ++index) { assert(advance(412000ULL + index * 4000ULL, 0U) == 2U); }
    for (index = 0U; index < 4U; ++index)
    { snprintf(command, sizeof(command), "99 adrc shaft_binding index=%u", index); (void)query(command, PROTOCOL_ENGINE_STATUS_OK); }
    trace = &bridge.shaft.core->trace[0]; trace->input.now_us = UINT64_MAX; trace->input.binding_id = UINT64_MAX;
    trace->input.session_id = UINT64_MAX; trace->input.feedback.rx_us = UINT64_MAX; trace->input.feedback.observed_us = UINT64_MAX;
    trace->input.receipt.command_id = UINT64_MAX; trace->input.receipt.completed_observed_us = UINT64_MAX;
    trace->output.binding_id = trace->output.session_id = trace->output.command_id = trace->output.issued_us = UINT64_MAX;
    trace->output.torque_nm = trace->output.previous_sent_nm = trace->output.maximum_step_nm = trace->output.cap_nm = -FLT_MAX;
    trace->input.receipt.decoded_torque_nm = -FLT_MAX;
    for (part = 0U; part < 4U; ++part)
    { ProtocolOutputBatch output; snprintf(command, sizeof(command), "99 adrc shaft_trace index=0 part=%u", part);
      output = query(command, PROTOCOL_ENGINE_STATUS_OK); printf("trace part=%u maximum bytes=%u\n", part, output.messages[0].length); }
    ++groups;
}
/** @brief Runs the real host export/evidence order on normal and protection-failed stopped pilots without STOP effects. */
static void test_host_export_evidence_sequence(void)
{
    unsigned scenario, index, part;
    uint32_t wire_id;
    char command[128];
    ProtocolOutputBatch output;
    for (scenario = 0U; scenario < 2U; ++scenario)
    {
        ShaftPilotStatus frozen_status;
        ShaftPilotReceipt retained_receipt;
        uint64_t now;
        fixture(); start();
        if (!scenario)
        { for (now = 408000ULL; now <= 1400000ULL; now += 4000ULL)
          { (void)advance(now, (uint8_t)(bridge.shaft.core->status.phase < SHAFT_PHASE_STOPPING)); } }
        else
        {
            assert(advance(408000ULL, 1U) == 1U);
            feedback(412000ULL, 1U); clock_us = 413000ULL;
            runtime.bank.motors[6].feedback.position_rad = 12.6F; /* Explicit synthetic absolute-PMAX fault. */
            assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
            for (index = 0U; index < 40U; ++index) { assert(advance(416000ULL + index * 4000ULL, 0U) == 2U); }
            assert(bridge.shaft.core->status.first_fault == SHAFT_FAULT_POSITION);
        }
        assert(bridge.shaft.core->trace_frozen && bridge.shaft.core->status.safe_release && bridge.shaft.guard_ack);
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.release_disable_pending);
        frozen_status = bridge.shaft.core->status; retained_receipt = bridge.shaft.receipt;
        wire_id = 81000709U;
        (void)query("81000709 ping", PROTOCOL_ENGINE_STATUS_OK);
        (void)query("81000710 adrc shaft_status", PROTOCOL_ENGINE_STATUS_OK);
        for (part = 0U; part < 4U; ++part)
        { snprintf(command, sizeof(command), "%lu adrc shaft_binding index=%u", (unsigned long)++wire_id + 10UL, part);
          (void)query(command, PROTOCOL_ENGINE_STATUS_OK); }
        for (index = 0U; index < frozen_status.trace_count; ++index)
        { for (part = 0U; part < 4U; ++part)
          { snprintf(command, sizeof(command), "%lu adrc shaft_trace index=%u part=%u", (unsigned long)++wire_id + 10UL, index, part);
            (void)query(command, PROTOCOL_ENGINE_STATUS_OK); } }
        snprintf(command, sizeof(command), "%lu adrc shaft_status", (unsigned long)++wire_id + 10UL);
        (void)query(command, PROTOCOL_ENGINE_STATUS_OK);
        snprintf(command, sizeof(command), "%lu ping", (unsigned long)++wire_id + 10UL);
        (void)query(command, PROTOCOL_ENGINE_STATUS_OK);
        snprintf(command, sizeof(command), "%lu adrc evidence", (unsigned long)++wire_id + 10UL);
        output = query(command, PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, " verified=0 ") != NULL);
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.release_disable_pending && !bridge.bench.gateway.stop_pending);
        assert(!memcmp(&frozen_status, &bridge.shaft.core->status, sizeof(frozen_status)));
        assert(!memcmp(&retained_receipt, &bridge.shaft.receipt, sizeof(retained_receipt)));
        assert(!bridge.bench.trusted_qualification.verified_flags && !bridge.bench.experiment.qualification.verified_flags);
        assert(!bridge.frame_pending && !bridge.awaiting_receipt);
        printf("host export scenario=%u rows=%lu evidence_verified=0 no_stop_side_effects=1\n", scenario, (unsigned long)frozen_status.trace_count);
    }
    ++groups;
}
/** @brief Covers all existing read-only ADRC namespaces and retains STOP rejection for every mutation family. */
static void test_read_only_namespace_classification(void)
{
    static const char *read_only[] = {"adrc", "adrc status", "adrc evidence", "adrc hardware", "adrc limits", "adrc feedback",
        "adrc capabilities_status", "adrc capabilities_get index=0", "adrc capture", "adrc capture index=0", "adrc guard_status",
        "adrc simple_status", "adrc watchdog_status", "adrc watchdog_fault", "adrc trace index=0", "adrc simple_trace index=0"};
    static const char *mutating[] = {"adrc config", "adrc config group=limits torque_limit=0.1", "adrc sample", "adrc prepare motor=7",
        "adrc run", "adrc heartbeat", "adrc clear", "adrc capabilities", "adrc guard", "adrc simple profile=8", "adrc watchdog"};
    static const char *invalid_read_only[] = {"adrc evidence verified=1", "adrc hardware extra=1", "adrc limits torque_limit=9",
        "adrc status extra=1", "adrc trace", "adrc simple_status profile=8", "adrc watchdog_fault extra=1"};
    unsigned index;
    char command[128];
    for (index = 0U; index < sizeof(read_only)/sizeof(read_only[0]); ++index)
    {
        ProtocolEngineStatus status;
        ProtocolOutputBatch output;
        fixture(); start();
        snprintf(command, sizeof(command), "%lu %s", (unsigned long)(900U + index), read_only[index]);
        status = adrc_app_bridge_process_line(&bridge, command, strlen(command), clock_us, &output);
        assert(status == (index >= 14U ? PROTOCOL_ENGINE_STATUS_BAD_REQUEST : PROTOCOL_ENGINE_STATUS_OK));
        assert(strstr(output.messages[0].data, "mutation_conflict") == NULL);
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.release_disable_pending && !bridge.bench.gateway.stop_pending);
        assert(bridge.shaft.core->status.phase == SHAFT_PHASE_ENABLE && !bridge.bench.trusted_qualification.verified_flags);
    }
    for (index = 0U; index < sizeof(invalid_read_only)/sizeof(invalid_read_only[0]); ++index)
    {
        ProtocolOutputBatch output;
        fixture(); start();
        snprintf(command, sizeof(command), "%lu %s", (unsigned long)(950U + index), invalid_read_only[index]);
        output = query(command, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(strstr(output.messages[0].data, "mutation_conflict") == NULL);
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.bench.gateway.stop_pending);
        assert(!bridge.bench.trusted_qualification.verified_flags && !bridge.bench.gateway.command_pending);
    }
    for (index = 0U; index < sizeof(mutating)/sizeof(mutating[0]); ++index)
    {
        ProtocolOutputBatch output;
        fixture(); start();
        snprintf(command, sizeof(command), "%lu %s", (unsigned long)(1000U + index), mutating[index]);
        output = query(command, PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(strstr(output.messages[0].data, "mutation_conflict") != NULL);
        assert(bridge.shaft.stop_latched && bridge.stop_pending && !bridge.frame_pending);
    }
    ++groups;
}
/** @brief Fixed low profiles bind both directions, real integer torque caps and reject variable USB plans. */
static void test_low_profile_entry(void)
{
    static const char *valid[] = {"4 adrc shaft_pilot direction=positive profile=low",
        "4 adrc shaft_pilot direction=negative profile=low"};
    static const char *invalid[] = {"4 adrc shaft_pilot profile=low",
        "4 adrc shaft_pilot direction=positive profile=default",
        "4 adrc shaft_pilot direction=positive profile=LOW",
        "4 adrc shaft_pilot direction=positive profile=low cap=0.35",
        "4 adrc shaft_pilot direction=positive profile=low model=1"};
    unsigned index, tick_index;
    for (index = 0U; index < sizeof(valid)/sizeof(valid[0]); ++index)
    {
        ProtocolOutputBatch output;
        fixture(); (void)query(valid[index], PROTOCOL_ENGINE_STATUS_OK);
        assert(bridge.shaft.core->low_profile && bridge.shaft.core->plan.cap_nm == 0.35F);
        assert(bridge.shaft.core->plan.direction == (index ? -1 : 1));
        output = query("5 adrc shaft_binding index=0", PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "cap=0.349999994") != NULL);
        assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
        assert(advance(404000ULL, 0U) == 0U);
        for (tick_index = 0U; tick_index < 120U && bridge.shaft.core->status.phase != SHAFT_PHASE_DONE; ++tick_index)
        {
            uint8_t enabled = (uint8_t)(bridge.shaft.core->status.phase <= SHAFT_PHASE_RAMP);
            (void)advance(408000ULL + tick_index * 4000ULL, enabled);
            assert(bridge.shaft.core->status.peak_successful_decoded_torque_nm <= 0.35F);
        }
        assert(bridge.shaft.core->status.phase == SHAFT_PHASE_DONE);
        assert(bridge.shaft.core->status.outcome == SHAFT_END_TIME && bridge.shaft.core->status.first_fault == SHAFT_FAULT_NONE);
        assert(!bridge.bench.trusted_qualification.verified_flags);
        (void)query("6 adrc shaft_pilot direction=negative profile=low", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    }
    for (index = 0U; index < sizeof(invalid)/sizeof(invalid[0]); ++index)
    { fixture(); (void)query(invalid[index], PROTOCOL_ENGINE_STATUS_BAD_REQUEST); assert(!bridge.shaft.used); }
    fixture();
    (void)query("4 adrc shaft_pilot direction=positive profile=fine", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(!bridge.shaft.used);
    fixture();
    runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F;
    bridge.capture.discovery = bridge.capability_discovery = runtime.discovery.results[6];
    for (index = 0U; index < 64U; ++index) bridge.capture.samples[index].velocity_rad_s = -0.00122117996F;
    runtime.bank.motors[6].feedback.velocity_rad_s = -0.00122117996F;
    (void)query("4 adrc shaft_pilot direction=positive profile=fine cap=0.2", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    (void)query("5 adrc shaft_pilot direction=positive profile=fine", PROTOCOL_ENGINE_STATUS_OK);
    assert(bridge.shaft.core->plan.cap_nm == 0.20F && bridge.shaft.core->low_profile);
    {
        ProtocolOutputBatch output = query("6 adrc shaft_binding index=0", PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "cap=0.200000003") != NULL);
    }
    fixture(); start(); assert(!bridge.shaft.core->low_profile && bridge.shaft.core->plan.cap_nm == 1.75F);
    ++groups;
}
/** @brief Fixed visible entry binds actual cap/slew, completes finitely and never accepts numeric override or replay. */
static void test_visible_profile_entry(void)
{
    unsigned index, tick_index;
    static const char *valid[] = {"4 adrc shaft_pilot direction=positive profile=visible",
        "4 adrc shaft_pilot direction=negative profile=visible"};
    static const char *invalid[] = {"4 adrc shaft_pilot direction=positive profile=VISIBLE",
        "4 adrc shaft_pilot direction=positive profile=visible cap=0.5",
        "4 adrc shaft_pilot direction=positive profile=visible angle=0.15",
        "4 adrc shaft_pilot direction=positive profile=visible b0=2"};
    for (index = 0U; index < 2U; ++index)
    {
        ProtocolOutputBatch output;
        fixture(); (void)query(valid[index], PROTOCOL_ENGINE_STATUS_OK);
        assert(bridge.shaft.core->plan.cap_nm == 0.5F);
        assert(bridge.shaft.core->plan.slew_nm_s == 2.0F && bridge.shaft.core->plan.step_nm == 0.008F);
        output = query("5 adrc shaft_binding index=0", PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "cap=0.5 slew=2 step=0.008") != NULL);
        assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
        assert(advance(404000ULL, 0U) == 0U);
        for (tick_index = 0U; tick_index < 300U && bridge.shaft.core->status.phase != SHAFT_PHASE_DONE; ++tick_index)
        {
            uint8_t enabled = (uint8_t)(bridge.shaft.core->status.phase <= SHAFT_PHASE_RAMP);
            (void)advance(408000ULL + tick_index * 4000ULL, enabled);
            assert(bridge.shaft.core->status.peak_successful_decoded_torque_nm <= 0.5F);
        }
        assert(bridge.shaft.core->status.phase == SHAFT_PHASE_DONE);
        assert(bridge.shaft.core->status.outcome == SHAFT_END_TIME);
        assert(bridge.shaft.core->status.first_fault == SHAFT_FAULT_NONE);
        assert(!bridge.bench.trusted_qualification.verified_flags);
        (void)query("6 adrc shaft_pilot direction=positive profile=visible", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    }
    for (index = 0U; index < sizeof(invalid)/sizeof(invalid[0]); ++index)
    { fixture(); (void)query(invalid[index], PROTOCOL_ENGINE_STATUS_BAD_REQUEST); assert(!bridge.shaft.used); }
    ++groups;
}
/** @brief Observe has explicit identity despite sharing default torque fields; USB numeric plans never enter. */
static void test_observe_profile_entry(void)
{
    static const char *invalid[] = {"4 adrc shaft_pilot profile=observe",
        "4 adrc shaft_pilot direction=positive profile=OBSERVE",
        "4 adrc shaft_pilot direction=positive profile=observe cap=1.75",
        "4 adrc shaft_pilot direction=positive profile=observe slew=4",
        "4 adrc shaft_pilot direction=positive profile=observe step=0.016",
        "4 adrc shaft_pilot direction=positive profile=observe speed=20.6821516",
        "4 adrc shaft_pilot direction=positive profile=observe angle=0.523598776",
        "4 adrc shaft_pilot direction=positive profile=observe reverse=0.0872664626",
        "4 adrc shaft_pilot direction=positive profile=observe active=1.57079633",
        "4 adrc shaft_pilot direction=positive profile=observe stopped=3.14159265",
        "4 adrc shaft_pilot direction=positive profile=observe ramp_us=900000",
        "4 adrc shaft_pilot direction=positive profile=observe plateau_us=200000",
        "4 adrc shaft_pilot direction=positive profile=observe rated_rpm=395",
        "4 adrc shaft_pilot direction=positive profile=observe b0=2"};
    unsigned index;
    for (index = 0U; index < sizeof(invalid)/sizeof(invalid[0]); ++index)
    { fixture(); (void)query(invalid[index], PROTOCOL_ENGINE_STATUS_BAD_REQUEST); assert(!bridge.shaft.used); }
    for (index = 0U; index < 3U; ++index)
    {
        fixture();
        if (index == 0U) { runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F; }
        if (index == 1U) { runtime.discovery.results[6].ranges.position_max_rad = 25.0F; }
        if (index == 2U) { runtime.discovery.results[6].ranges.torque_max_nm = 5.0F; }
        bridge.capture.discovery = bridge.capability_discovery = runtime.discovery.results[6];
        (void)query("4 adrc shaft_pilot direction=positive profile=observe", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!bridge.shaft.used && !bridge.frame_pending && !bridge.capability_terminal_closed);
    }
    for (index = 0U; index < 2U; ++index)
    {
        ProtocolOutputBatch output;
        fixture(); output = query(index ? "4 adrc shaft_pilot direction=negative profile=observe" :
            "4 adrc shaft_pilot direction=positive profile=observe", PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "qualified=0 control=0") != NULL);
        assert(bridge.shaft.core->observe_profile && !bridge.shaft.core->platform_profile);
        assert(bridge.shaft.core->plan.direction == (index ? -1 : 1));
        assert(bridge.shaft.core->plan.cap_nm == 1.75F && bridge.shaft.core->plan.slew_nm_s == 4.0F &&
            bridge.shaft.core->plan.step_nm == 0.016F);
        output = query("4294967295 adrc shaft_binding index=4", PROTOCOL_ENGINE_STATUS_OK);
        assert(!strcmp(output.messages[0].data,
            "ok 4294967295 adrc shaft_binding id=4 index=4 profile=4 speed=20.6821518 angle=0.52359879 reverse=0.0872664601 active=1.57079637 stopped=3.14159274 ramp_us=900000 plateau_us=200000 rated_rpm=395\r\n"));
        assert(output.messages[0].length == strlen(output.messages[0].data));
        assert(output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
        assert(!bridge.bench.trusted_qualification.verified_flags);
    }
    ++groups;
}
/** @brief Old profiles retain four binding shards even when their torque plan equals observe. */
static void test_observe_binding_isolation(void)
{
    static const char *entry[] = {"4 adrc shaft_pilot direction=positive",
        "4 adrc shaft_pilot direction=positive profile=low",
        "4 adrc shaft_pilot direction=positive profile=visible",
        "4 adrc shaft_pilot direction=positive profile=fine"};
    unsigned index;
    for (index = 0U; index < 4U; ++index)
    {
        fixture();
        if (index == 3U)
        {
            unsigned sample;
            runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F;
            bridge.capture.discovery = bridge.capability_discovery = runtime.discovery.results[6];
            for (sample = 0U; sample < 64U; ++sample) { bridge.capture.samples[sample].velocity_rad_s = -0.00122117996F; }
            runtime.bank.motors[6].feedback.velocity_rad_s = -0.00122117996F;
        }
        (void)query(entry[index], PROTOCOL_ENGINE_STATUS_OK);
        assert(!bridge.shaft.core->platform_profile);
        (void)query("5 adrc shaft_binding index=4", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending);
    }
    ++groups;
}
/** @brief Real Motor integer encoding and matching receipts finish the 200ms platform within the 900ms ramp. */
static void test_observe_real_codec_timing(void)
{
    unsigned period_ms, direction_index;
    for (period_ms = 3U; period_ms <= 5U; ++period_ms)
    { for (direction_index = 0U; direction_index < 2U; ++direction_index)
      {
        uint64_t now, ramp_started = 0ULL, plateau_received = 0ULL, stop_observed = 0ULL;
        unsigned tick;
        fixture(); (void)query(direction_index ? "4 adrc shaft_pilot direction=negative profile=observe" :
            "4 adrc shaft_pilot direction=positive profile=observe", PROTOCOL_ENGINE_STATUS_OK);
        assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
        assert(advance(404000ULL, 0U) == 0U);
        for (tick = 1U; tick < 460U && bridge.shaft.core->status.phase != SHAFT_PHASE_DONE; ++tick)
        {
            ShaftPilotPhase before = bridge.shaft.core->status.phase;
            uint8_t enabled = (uint8_t)(before < SHAFT_PHASE_STOPPING);
            now = 404000ULL + (uint64_t)tick * period_ms * 1000ULL;
            (void)advance(now, enabled);
            if (bridge.shaft.receipt.kind == SHAFT_ACTION_TORQUE)
            {
                uint16_t torque_code = (uint16_t)(((uint16_t)(bridge.frame.data[6] & 0x0FU) << 8U) | bridge.frame.data[7]);
                float decoded_from_frame = (float)torque_code * 20.0F / 4095.0F - 10.0F;
                assert(bridge.shaft.receipt.decoded_torque_nm == decoded_from_frame);
                assert((bridge.frame.data[3] & 0x0FU) == 0U && bridge.frame.data[4] == 0U &&
                    bridge.frame.data[5] == 0U && (bridge.frame.data[6] & 0xF0U) == 0U);
            }
            if (!ramp_started && bridge.shaft.core->status.phase == SHAFT_PHASE_RAMP) { ramp_started = clock_us; }
            if (!plateau_received && ramp_started && bridge.shaft.receipt.kind == SHAFT_ACTION_TORQUE &&
                1.75F - fabsf(bridge.shaft.receipt.decoded_torque_nm) < bridge.shaft.core->status.quantum_torque_nm)
            { plateau_received = clock_us; }
            if (!stop_observed && before == SHAFT_PHASE_RAMP && bridge.shaft.core->status.phase == SHAFT_PHASE_STOPPING)
            { stop_observed = clock_us; }
            assert(bridge.shaft.core->status.peak_successful_decoded_torque_nm <= 1.75F);
        }
        assert(ramp_started && plateau_received && stop_observed);
        assert(stop_observed - plateau_received >= 200000ULL);
        assert(stop_observed - ramp_started < 900000ULL);
        assert(bridge.shaft.core->status.phase == SHAFT_PHASE_DONE && bridge.shaft.core->status.safe_release);
        assert(bridge.shaft.core->status.outcome == SHAFT_END_TIME && bridge.shaft.core->status.first_fault == SHAFT_FAULT_NONE);
        assert(bridge.shaft.guard_ack && !bridge.bench.trusted_qualification.verified_flags);
        printf("observe real_codec period_ms=%u direction=%d ramp_to_stop_us=%llu plateau_dwell_us=%llu peak=%.9g\n",
            period_ms, direction_index ? -1 : 1, (unsigned long long)(stop_observed-ramp_started),
            (unsigned long long)(stop_observed-plateau_received), (double)bridge.shaft.core->status.peak_successful_decoded_torque_nm);
      } }
    ++groups;
}
/** @brief Platform freezes a separate fifth schema, actual VMAX200 ranges and direction-only USB admission. */
static void test_platform_profile_entry(void)
{
    static const char *invalid[] = {"4 adrc shaft_pilot profile=platform",
        "4 adrc shaft_pilot direction=positive profile=PLATFORM",
        "4 adrc shaft_pilot direction=positive profile=platform cap=1.75",
        "4 adrc shaft_pilot direction=positive profile=platform slew=4",
        "4 adrc shaft_pilot direction=positive profile=platform step=0.016",
        "4 adrc shaft_pilot direction=positive profile=platform speed=20.6821516",
        "4 adrc shaft_pilot direction=positive profile=platform angle=0.523598776",
        "4 adrc shaft_pilot direction=positive profile=platform reverse=0.0872664626",
        "4 adrc shaft_pilot direction=positive profile=platform active=1.57079633",
        "4 adrc shaft_pilot direction=positive profile=platform stopped=3.14159265",
        "4 adrc shaft_pilot direction=positive profile=platform reserve=3.14159265",
        "4 adrc shaft_pilot direction=positive profile=platform lookahead_us=25000",
        "4 adrc shaft_pilot direction=positive profile=platform ramp_us=900000",
        "4 adrc shaft_pilot direction=positive profile=platform plateau_us=200000",
        "4 adrc shaft_pilot direction=positive profile=platform rated_rpm=395",
        "4 adrc shaft_pilot direction=positive profile=platform b0=2"};
    unsigned index;
    for (index = 0U; index < sizeof(invalid)/sizeof(invalid[0]); ++index)
    { fixture(); (void)query(invalid[index], PROTOCOL_ENGINE_STATUS_BAD_REQUEST); assert(!bridge.shaft.used); }
    for (index = 0U; index < 3U; ++index)
    {
        fixture();
        if (index == 0U) { runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F; }
        if (index == 1U) { runtime.discovery.results[6].ranges.position_max_rad = 25.0F; }
        if (index == 2U) { runtime.discovery.results[6].ranges.torque_max_nm = 5.0F; }
        bridge.capture.discovery = bridge.capability_discovery = runtime.discovery.results[6];
        (void)query("4 adrc shaft_pilot direction=positive profile=platform", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!bridge.shaft.used && !bridge.frame_pending && !bridge.capability_terminal_closed);
    }
    for (index = 0U; index < 2U; ++index)
    {
        ProtocolOutputBatch output;
        fixture(); output = query(index ? "4 adrc shaft_pilot direction=negative profile=platform" :
            "4 adrc shaft_pilot direction=positive profile=platform", PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "qualified=0 control=0") != NULL);
        assert(bridge.shaft.core->observe_profile && bridge.shaft.core->platform_profile);
        assert(bridge.shaft.core->plan.direction == (index ? -1 : 1));
        assert(bridge.shaft.core->plan.cap_nm == 1.75F && bridge.shaft.core->plan.slew_nm_s == 4.0F &&
            bridge.shaft.core->plan.step_nm == 0.016F);
        assert(bridge.shaft.core->config.position_max_native_rad == 12.5F &&
            bridge.shaft.core->config.velocity_max_rad_s == 200.0F && bridge.shaft.core->config.torque_max_nm == 10.0F);
        output = query("4294967295 adrc shaft_binding index=4", PROTOCOL_ENGINE_STATUS_OK);
        assert(!strcmp(output.messages[0].data,
            "ok 4294967295 adrc shaft_binding id=4 index=4 profile=5 speed=20.6821518 reverse=0.0872664601 reserve=3.14159274 lookahead_us=25000 ramp_us=900000 plateau_us=200000 rated_rpm=395\r\n"));
        assert(output.messages[0].length == strlen(output.messages[0].data));
        assert(output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
        (void)query("5 adrc shaft_binding index=5", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.bench.trusted_qualification.verified_flags);
        (void)query("6 adrc shaft_pilot direction=positive profile=platform", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    }
    ++groups;
}
/** @brief Real Motor grid receipts complete both platform directions at 3/4/5ms without granting qualification. */
static void test_platform_real_codec_timing(void)
{
    unsigned period_ms, direction_index;
    for (period_ms = 3U; period_ms <= 5U; ++period_ms)
    { for (direction_index = 0U; direction_index < 2U; ++direction_index)
      {
        uint64_t now, ramp_started = 0ULL, plateau_received = 0ULL, stop_observed = 0ULL;
        unsigned tick, part;
        char command[100];
        fixture(); (void)query(direction_index ? "4 adrc shaft_pilot direction=negative profile=platform" :
            "4 adrc shaft_pilot direction=positive profile=platform", PROTOCOL_ENGINE_STATUS_OK);
        assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
        assert(advance(404000ULL, 0U) == 0U);
        for (tick = 1U; tick < 460U && bridge.shaft.core->status.phase != SHAFT_PHASE_DONE; ++tick)
        {
            ShaftPilotPhase before = bridge.shaft.core->status.phase;
            uint8_t enabled = (uint8_t)(before < SHAFT_PHASE_STOPPING);
            now = 404000ULL + (uint64_t)tick * period_ms * 1000ULL;
            (void)advance(now, enabled);
            if (bridge.shaft.receipt.kind == SHAFT_ACTION_TORQUE)
            {
                uint16_t torque_code = (uint16_t)(((uint16_t)(bridge.frame.data[6] & 0x0FU) << 8U) | bridge.frame.data[7]);
                float decoded_from_frame = (float)torque_code * 20.0F / 4095.0F - 10.0F;
                assert(bridge.shaft.receipt.decoded_torque_nm == decoded_from_frame);
                assert((bridge.frame.data[3] & 0x0FU) == 0U && bridge.frame.data[4] == 0U &&
                    bridge.frame.data[5] == 0U && (bridge.frame.data[6] & 0xF0U) == 0U);
            }
            if (!ramp_started && bridge.shaft.core->status.phase == SHAFT_PHASE_RAMP) { ramp_started = clock_us; }
            if (!plateau_received && ramp_started && bridge.shaft.receipt.kind == SHAFT_ACTION_TORQUE &&
                1.75F - fabsf(bridge.shaft.receipt.decoded_torque_nm) < bridge.shaft.core->status.quantum_torque_nm)
            { plateau_received = clock_us; }
            if (!stop_observed && before == SHAFT_PHASE_RAMP && bridge.shaft.core->status.phase == SHAFT_PHASE_STOPPING)
            { stop_observed = clock_us; }
            assert(bridge.shaft.core->status.peak_successful_decoded_torque_nm <= 1.75F);
        }
        assert(ramp_started && plateau_received && stop_observed);
        assert(stop_observed - plateau_received >= 200000ULL && stop_observed - ramp_started < 900000ULL);
        assert(stop_observed - bridge.shaft.core->started_us < 1400000ULL);
        assert(bridge.shaft.core->status.phase == SHAFT_PHASE_DONE && bridge.shaft.core->status.safe_release);
        assert(clock_us - bridge.shaft.core->started_us < 1500000ULL);
        assert(bridge.shaft.core->status.outcome == SHAFT_END_TIME && bridge.shaft.core->status.first_fault == SHAFT_FAULT_NONE);
        assert(bridge.shaft.guard_ack && bridge.shaft.core->trace_frozen && !bridge.bench.trusted_qualification.verified_flags);
        for (part = 0U; part < 5U; ++part)
        { snprintf(command, sizeof(command), "99 adrc shaft_binding index=%u", part); (void)query(command, PROTOCOL_ENGINE_STATUS_OK); }
        for (part = 0U; part < 4U; ++part)
        { snprintf(command, sizeof(command), "99 adrc shaft_trace index=0 part=%u", part); (void)query(command, PROTOCOL_ENGINE_STATUS_OK); }
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.frame_pending && !bridge.awaiting_receipt);
        printf("platform real_codec period_ms=%u direction=%d ramp_to_stop_us=%llu plateau_dwell_us=%llu peak=%.9g\n",
            period_ms, direction_index ? -1 : 1, (unsigned long long)(stop_observed-ramp_started),
            (unsigned long long)(stop_observed-plateau_received), (double)bridge.shaft.core->status.peak_successful_decoded_torque_nm);
      } }
    ++groups;
}
/** @brief 新量程发现、64点基线和新反馈全部经真实codec；不混用旧12.5解码。 */
static void platform_wide_fixture(void)
{
    unsigned index;
    fixture(); runtime.discovery.results[6].ranges.position_max_rad = 512.0F;
    bridge.capture.discovery = bridge.capability_discovery = runtime.discovery.results[6];
    for (index = 0U; index < 64U; ++index)
    {
        S3519Feedback decoded = decode_codec_feedback(index == 8U ? 0.016F : 0.0F, 0.0F, 0U);
        bridge.capture.samples[index].position_rad = decoded.position_rad;
        bridge.capture.samples[index].velocity_rad_s = decoded.velocity_rad_s;
        bridge.capture.samples[index].torque_nm = decoded.torque_nm;
    }
    real_feedback_codec = 1U; feedback(400000ULL, 0U); clock_us = 401000ULL;
}
/** @brief 宽档只有方向可变；身份6第五分片独立冻结，旧档绝不自动改为512。 */
static void test_platform_wide_profile_entry(void)
{
    static const char *invalid[] = {"4 adrc shaft_pilot profile=platform_wide",
        "4 adrc shaft_pilot direction=positive profile=PLATFORM_WIDE",
        "4 adrc shaft_pilot direction=positive profile=platform_wide cap=1.75",
        "4 adrc shaft_pilot direction=positive profile=platform_wide slew=4",
        "4 adrc shaft_pilot direction=positive profile=platform_wide step=0.016",
        "4 adrc shaft_pilot direction=positive profile=platform_wide speed=20.6821516",
        "4 adrc shaft_pilot direction=positive profile=platform_wide angle=0.523598776",
        "4 adrc shaft_pilot direction=positive profile=platform_wide reverse=0.0872664626",
        "4 adrc shaft_pilot direction=positive profile=platform_wide active=1.57079633",
        "4 adrc shaft_pilot direction=positive profile=platform_wide stopped=3.14159265",
        "4 adrc shaft_pilot direction=positive profile=platform_wide reserve=12.5663706",
        "4 adrc shaft_pilot direction=positive profile=platform_wide lookahead_us=25000",
        "4 adrc shaft_pilot direction=positive profile=platform_wide entry_us=1058000",
        "4 adrc shaft_pilot direction=positive profile=platform_wide ramp_us=900000",
        "4 adrc shaft_pilot direction=positive profile=platform_wide plateau_us=200000",
        "4 adrc shaft_pilot direction=positive profile=platform_wide rated_rpm=395",
        "4 adrc shaft_pilot direction=positive profile=platform_wide pmax=512",
        "4 adrc shaft_pilot direction=positive profile=platform_wide b0=2"};
    unsigned index;
    for (index = 0U; index < sizeof(invalid)/sizeof(invalid[0]); ++index)
    { platform_wide_fixture(); (void)query(invalid[index], PROTOCOL_ENGINE_STATUS_BAD_REQUEST); assert(!bridge.shaft.used); }
    for (index = 0U; index < 4U; ++index)
    {
        platform_wide_fixture();
        if (index == 0U) { runtime.discovery.results[6].ranges.position_max_rad = 12.5F; }
        if (index == 1U) { runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F; }
        if (index == 2U) { runtime.discovery.results[6].ranges.torque_max_nm = 5.0F; }
        if (index == 3U) { runtime.discovery.results[6].ranges.position_max_rad = 511.0F; }
        bridge.capture.discovery = bridge.capability_discovery = runtime.discovery.results[6];
        (void)query("4 adrc shaft_pilot direction=positive profile=platform_wide", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!bridge.shaft.used && !bridge.frame_pending && !bridge.capability_terminal_closed);
    }
    for (index = 0U; index < 2U; ++index)
    {
        ProtocolOutputBatch output;
        platform_wide_fixture(); output = query(index ? "4 adrc shaft_pilot direction=negative profile=platform_wide" :
            "4 adrc shaft_pilot direction=positive profile=platform_wide", PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "qualified=0 control=0") != NULL);
        assert(bridge.shaft.core->observe_profile && bridge.shaft.core->platform_profile);
        assert(bridge.shaft.core->plan.direction == (index ? -1 : 1));
        assert(bridge.shaft.core->config.position_max_native_rad == 512.0F &&
            bridge.shaft.core->config.velocity_max_rad_s == 200.0F && bridge.shaft.core->config.torque_max_nm == 10.0F);
        output = query("4294967295 adrc shaft_binding index=4", PROTOCOL_ENGINE_STATUS_OK);
        assert(!strcmp(output.messages[0].data,
            "ok 4294967295 adrc shaft_binding id=4 index=4 profile=6 speed=20.6821518 reverse=0.0872664601 reserve=12.566371 lookahead_us=25000 entry_us=1058000 ramp_us=900000 plateau_us=200000 rated_rpm=395\r\n"));
        output = query("5 adrc shaft_binding index=1", PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "pmax=512 vmax=200 tmax=10") != NULL);
        assert(strstr(output.messages[0].data, "qnative=0.0156252384") != NULL);
        assert(fabsf(bridge.shaft.core->status.quantum_shaft_rad - 1024.0F/65535.0F/19.2031994F) < 1.0e-9F);
        (void)query("6 adrc shaft_binding index=5", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.bench.trusted_qualification.verified_flags);
    }
    ++groups;
}
/** @brief 真实codec双方向3/4/5ms轨迹穿过旧PMAX，实际内缩平台保持200ms并失能收尾。 */
static void test_platform_wide_real_codec_timing(void)
{
    unsigned period_ms, direction_index;
    for (period_ms = 3U; period_ms <= 5U; ++period_ms)
    { for (direction_index = 0U; direction_index < 2U; ++direction_index)
      {
        uint64_t now, ramp_started = 0ULL, plateau_received = 0ULL, stop_observed = 0ULL;
        unsigned tick, part;
        int direction = direction_index ? -1 : 1;
        char command[100];
        platform_wide_fixture();
        (void)query(direction_index ? "4 adrc shaft_pilot direction=negative profile=platform_wide" :
            "4 adrc shaft_pilot direction=positive profile=platform_wide", PROTOCOL_ENGINE_STATUS_OK);
        assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
        assert(advance(404000ULL, 0U) == 0U);
        for (tick = 1U; tick < 460U && bridge.shaft.core->status.phase != SHAFT_PHASE_DONE; ++tick)
        {
            ShaftPilotPhase before = bridge.shaft.core->status.phase;
            uint8_t enabled = (uint8_t)(before < SHAFT_PHASE_STOPPING);
            codec_raw_velocity = before == SHAFT_PHASE_RAMP ? (float)direction * 2.0F : 0.0F;
            if (before == SHAFT_PHASE_RAMP)
            { codec_native_position += (float)direction * 2.0F * 19.2031994F * (float)period_ms * 0.001F; }
            now = 404000ULL + (uint64_t)tick * period_ms * 1000ULL; (void)advance(now, enabled);
            if (bridge.shaft.receipt.kind == SHAFT_ACTION_TORQUE)
            {
                uint16_t torque_code = (uint16_t)(((uint16_t)(bridge.frame.data[6] & 0x0FU) << 8U) | bridge.frame.data[7]);
                float decoded_from_frame = (float)torque_code * 20.0F / 4095.0F - 10.0F;
                assert(bridge.shaft.receipt.decoded_torque_nm == decoded_from_frame);
                assert((bridge.frame.data[3] & 0x0FU) == 0U && bridge.frame.data[4] == 0U &&
                    bridge.frame.data[5] == 0U && (bridge.frame.data[6] & 0xF0U) == 0U);
            }
            if (!ramp_started && bridge.shaft.core->status.phase == SHAFT_PHASE_RAMP) { ramp_started = clock_us; }
            if (!plateau_received && ramp_started && bridge.shaft.receipt.kind == SHAFT_ACTION_TORQUE &&
                1.75F - fabsf(bridge.shaft.receipt.decoded_torque_nm) < bridge.shaft.core->status.quantum_torque_nm)
            { plateau_received = clock_us; }
            if (!stop_observed && before == SHAFT_PHASE_RAMP && bridge.shaft.core->status.phase == SHAFT_PHASE_STOPPING)
            { stop_observed = clock_us; }
            assert(bridge.shaft.core->status.peak_successful_decoded_torque_nm <= 1.75F);
            assert(bridge.shaft.core->status.first_fault == SHAFT_FAULT_NONE);
        }
        assert(ramp_started && plateau_received && stop_observed && fabsf(codec_native_position) > 12.5F);
        assert(stop_observed - plateau_received >= 200000ULL && stop_observed - ramp_started < 900000ULL);
        assert(stop_observed - bridge.shaft.core->started_us < 1400000ULL);
        assert(bridge.shaft.core->status.phase == SHAFT_PHASE_DONE && bridge.shaft.core->status.safe_release);
        assert(clock_us - bridge.shaft.core->started_us < 1500000ULL);
        assert(bridge.shaft.core->status.outcome == SHAFT_END_TIME && bridge.shaft.guard_ack);
        assert(bridge.shaft.core->trace_frozen && !bridge.bench.trusted_qualification.verified_flags);
        for (part = 0U; part < 5U; ++part)
        { snprintf(command, sizeof(command), "99 adrc shaft_binding index=%u", part); (void)query(command, PROTOCOL_ENGINE_STATUS_OK); }
        for (part = 0U; part < 4U; ++part)
        { snprintf(command, sizeof(command), "99 adrc shaft_trace index=0 part=%u", part); (void)query(command, PROTOCOL_ENGINE_STATUS_OK); }
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.frame_pending && !bridge.awaiting_receipt);
        printf("platform_wide real_codec period_ms=%u direction=%d ramp_to_stop_us=%llu plateau_dwell_us=%llu native=%.9g peak=%.9g\n",
            period_ms, direction, (unsigned long long)(stop_observed-ramp_started),
            (unsigned long long)(stop_observed-plateau_received), (double)codec_native_position,
            (double)bridge.shaft.core->status.peak_successful_decoded_torque_nm);
      } }
    ++groups;
}
/** @brief 量程转换前后同一码解码不同；陈旧发现、旧档身份或未同步基线均不能借新档进入。 */
static void test_platform_wide_range_identity(void)
{
    S3519Feedback narrow, wide;
    ProtocolOutputBatch output;
    fixture(); narrow = decode_codec_feedback(1.0F, 0.0F, 0U);
    runtime.discovery.results[6].ranges.position_max_rad = 512.0F;
    wide = decode_codec_feedback(40.96F, 0.0F, 0U);
    assert(fabsf(wide.position_rad - narrow.position_rad * 40.96F) < 0.0001F);
    (void)query("4 adrc shaft_pilot direction=positive profile=platform_wide", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(!bridge.shaft.used);
    platform_wide_fixture(); (void)query("4 adrc shaft_pilot direction=positive profile=platform", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    platform_wide_fixture(); (void)query("4 adrc shaft_pilot direction=positive profile=observe", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    fixture(); (void)query("4 adrc shaft_pilot direction=positive profile=platform", PROTOCOL_ENGINE_STATUS_OK);
    output = query("5 adrc shaft_binding index=4", PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "profile=5") && strstr(output.messages[0].data, "reserve=3.14159274"));
    ++groups;
}
/** @brief 新量程发现、64点基线和新反馈全部经真实codec；不混用旧12.5解码。 */
static void platform70_fixture(void)
{
    unsigned index;
    fixture(); runtime.discovery.results[6].ranges.position_max_rad = 1024.0F;
    bridge.capture.discovery = bridge.capability_discovery = runtime.discovery.results[6];
    for (index = 0U; index < 64U; ++index)
    {
        S3519Feedback decoded = decode_codec_feedback(index == 8U ? 0.016F : 0.0F, 0.0F, 0U);
        bridge.capture.samples[index].position_rad = decoded.position_rad;
        bridge.capture.samples[index].velocity_rad_s = decoded.velocity_rad_s;
        bridge.capture.samples[index].torque_nm = decoded.torque_nm;
    }
    real_feedback_codec = 1U; feedback(400000ULL, 0U); clock_us = 401000ULL;
}
/** @brief 宽档只有方向可变；身份7第五分片独立冻结，旧档绝不自动改为1024。 */
static void test_platform70_profile_entry(void)
{
    static const char *invalid[] = {"4 adrc shaft_pilot profile=platform70",
        "4 adrc shaft_pilot direction=positive profile=PLATFORM70",
        "4 adrc shaft_pilot direction=positive profile=platform70 cap=1.75",
        "4 adrc shaft_pilot direction=positive profile=platform70 slew=4",
        "4 adrc shaft_pilot direction=positive profile=platform70 step=0.016",
        "4 adrc shaft_pilot direction=positive profile=platform70 speed=28.9550123",
        "4 adrc shaft_pilot direction=positive profile=platform70 angle=0.523598776",
        "4 adrc shaft_pilot direction=positive profile=platform70 reverse=0.0872664626",
        "4 adrc shaft_pilot direction=positive profile=platform70 active=1.57079633",
        "4 adrc shaft_pilot direction=positive profile=platform70 stopped=3.14159265",
        "4 adrc shaft_pilot direction=positive profile=platform70 reserve=25.1327412",
        "4 adrc shaft_pilot direction=positive profile=platform70 lookahead_us=25000",
        "4 adrc shaft_pilot direction=positive profile=platform70 entry_us=1058000",
        "4 adrc shaft_pilot direction=positive profile=platform70 ramp_us=900000",
        "4 adrc shaft_pilot direction=positive profile=platform70 plateau_us=200000",
        "4 adrc shaft_pilot direction=positive profile=platform70 rated_rpm=395",
        "4 adrc shaft_pilot direction=positive profile=platform70 pmax=1024",
        "4 adrc shaft_pilot direction=positive profile=platform70 b0=2",
        "4 adrc shaft_pilot direction=positive profile=platform70 entry_us=1057999"};
    unsigned index;
    for (index = 0U; index < sizeof(invalid)/sizeof(invalid[0]); ++index)
    { platform70_fixture(); (void)query(invalid[index], PROTOCOL_ENGINE_STATUS_BAD_REQUEST); assert(!bridge.shaft.used); }
    for (index = 0U; index < 4U; ++index)
    {
        platform70_fixture();
        if (index == 0U) { runtime.discovery.results[6].ranges.position_max_rad = 12.5F; }
        if (index == 1U) { runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F; }
        if (index == 2U) { runtime.discovery.results[6].ranges.torque_max_nm = 5.0F; }
        if (index == 3U) { runtime.discovery.results[6].ranges.position_max_rad = 1023.0F; }
        bridge.capture.discovery = bridge.capability_discovery = runtime.discovery.results[6];
        (void)query("4 adrc shaft_pilot direction=positive profile=platform70", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!bridge.shaft.used && !bridge.frame_pending && !bridge.capability_terminal_closed);
    }
    for (index = 0U; index < 2U; ++index)
    {
        ProtocolOutputBatch output;
        platform70_fixture(); output = query(index ? "4 adrc shaft_pilot direction=negative profile=platform70" :
            "4 adrc shaft_pilot direction=positive profile=platform70", PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "qualified=0 control=0") != NULL);
        assert(bridge.shaft.core->observe_profile && bridge.shaft.core->platform_profile);
        assert(bridge.shaft.core->plan.direction == (index ? -1 : 1));
        assert(bridge.shaft.core->config.position_max_native_rad == 1024.0F &&
            bridge.shaft.core->config.velocity_max_rad_s == 200.0F && bridge.shaft.core->config.torque_max_nm == 10.0F);
        output = query("4294967295 adrc shaft_binding index=4", PROTOCOL_ENGINE_STATUS_OK);
        assert(!strcmp(output.messages[0].data,
            "ok 4294967295 adrc shaft_binding id=4 index=4 profile=7 speed=28.9550114 reverse=0.0872664601 reserve=25.1327419 lookahead_us=25000 entry_us=1058000 ramp_us=900000 plateau_us=200000 rated_rpm=395\r\n"));
        output = query("5 adrc shaft_binding index=1", PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "pmax=1024 vmax=200 tmax=10") != NULL);
        assert(strstr(output.messages[0].data, "qnative=0.0312504768") != NULL);
        assert(fabsf(bridge.shaft.core->status.quantum_shaft_rad - 2048.0F/65535.0F/19.2031994F) < 1.0e-9F);
        (void)query("6 adrc shaft_binding index=5", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.bench.trusted_qualification.verified_flags);
    }
    ++groups;
}
/** @brief 真实codec双方向3/4/5ms轨迹穿过旧PMAX，实际内缩平台保持200ms并失能收尾。 */
static void test_platform70_real_codec_timing(void)
{
    unsigned period_ms, direction_index;
    for (period_ms = 3U; period_ms <= 5U; ++period_ms)
    { for (direction_index = 0U; direction_index < 2U; ++direction_index)
      {
        uint64_t now, ramp_started = 0ULL, plateau_received = 0ULL, stop_observed = 0ULL;
        unsigned tick, part;
        int direction = direction_index ? -1 : 1;
        char command[100];
        platform70_fixture();
        (void)query(direction_index ? "4 adrc shaft_pilot direction=negative profile=platform70" :
            "4 adrc shaft_pilot direction=positive profile=platform70", PROTOCOL_ENGINE_STATUS_OK);
        assert(adrc_app_bridge_service(&bridge, clock_us)); assert(receipt() == 2U);
        assert(advance(404000ULL, 0U) == 0U);
        for (tick = 1U; tick < 460U && bridge.shaft.core->status.phase != SHAFT_PHASE_DONE; ++tick)
        {
            ShaftPilotPhase before = bridge.shaft.core->status.phase;
            uint8_t enabled = (uint8_t)(before < SHAFT_PHASE_STOPPING);
            codec_raw_velocity = before == SHAFT_PHASE_RAMP ? (float)direction * 26.0F : 0.0F;
            if (before == SHAFT_PHASE_RAMP)
            { codec_native_position += (float)direction * 26.0F * 19.2031994F * (float)period_ms * 0.001F; }
            now = 404000ULL + (uint64_t)tick * period_ms * 1000ULL; (void)advance(now, enabled);
            if (bridge.shaft.receipt.kind == SHAFT_ACTION_TORQUE)
            {
                uint16_t torque_code = (uint16_t)(((uint16_t)(bridge.frame.data[6] & 0x0FU) << 8U) | bridge.frame.data[7]);
                float decoded_from_frame = (float)torque_code * 20.0F / 4095.0F - 10.0F;
                assert(bridge.shaft.receipt.decoded_torque_nm == decoded_from_frame);
                assert((bridge.frame.data[3] & 0x0FU) == 0U && bridge.frame.data[4] == 0U &&
                    bridge.frame.data[5] == 0U && (bridge.frame.data[6] & 0xF0U) == 0U);
            }
            if (!ramp_started && bridge.shaft.core->status.phase == SHAFT_PHASE_RAMP) { ramp_started = clock_us; }
            if (!plateau_received && ramp_started && bridge.shaft.receipt.kind == SHAFT_ACTION_TORQUE &&
                1.75F - fabsf(bridge.shaft.receipt.decoded_torque_nm) < bridge.shaft.core->status.quantum_torque_nm)
            { plateau_received = clock_us; }
            if (!stop_observed && before == SHAFT_PHASE_RAMP && bridge.shaft.core->status.phase == SHAFT_PHASE_STOPPING)
            { stop_observed = clock_us; }
            assert(bridge.shaft.core->status.peak_successful_decoded_torque_nm <= 1.75F);
            assert(bridge.shaft.core->status.first_fault == SHAFT_FAULT_NONE);
        }
        assert(ramp_started && plateau_received && stop_observed && fabsf(codec_native_position) > 12.5F);
        assert(stop_observed - plateau_received >= 200000ULL && stop_observed - ramp_started < 900000ULL);
        assert(stop_observed - bridge.shaft.core->started_us < 1400000ULL);
        assert(bridge.shaft.core->status.phase == SHAFT_PHASE_DONE && bridge.shaft.core->status.safe_release);
        assert(clock_us - bridge.shaft.core->started_us < 1500000ULL);
        assert(bridge.shaft.core->status.outcome == SHAFT_END_TIME && bridge.shaft.guard_ack);
        assert(bridge.shaft.core->trace_frozen && !bridge.bench.trusted_qualification.verified_flags);
        for (part = 0U; part < 5U; ++part)
        { snprintf(command, sizeof(command), "99 adrc shaft_binding index=%u", part); (void)query(command, PROTOCOL_ENGINE_STATUS_OK); }
        for (part = 0U; part < 4U; ++part)
        { snprintf(command, sizeof(command), "99 adrc shaft_trace index=0 part=%u", part); (void)query(command, PROTOCOL_ENGINE_STATUS_OK); }
        assert(!bridge.shaft.stop_latched && !bridge.stop_pending && !bridge.frame_pending && !bridge.awaiting_receipt);
        printf("platform70 real_codec period_ms=%u direction=%d ramp_to_stop_us=%llu plateau_dwell_us=%llu native=%.9g peak=%.9g\n",
            period_ms, direction, (unsigned long long)(stop_observed-ramp_started),
            (unsigned long long)(stop_observed-plateau_received), (double)codec_native_position,
            (double)bridge.shaft.core->status.peak_successful_decoded_torque_nm);
      } }
    ++groups;
}
/** @brief 量程转换前后同一码解码不同；陈旧发现、旧档身份或未同步基线均不能借新档进入。 */
static void test_platform70_range_identity(void)
{
    S3519Feedback narrow, wide;
    ProtocolOutputBatch output;
    fixture(); narrow = decode_codec_feedback(1.0F, 0.0F, 0U);
    runtime.discovery.results[6].ranges.position_max_rad = 1024.0F;
    wide = decode_codec_feedback(81.92F, 0.0F, 0U);
    assert(fabsf(wide.position_rad - narrow.position_rad * 81.92F) < 0.0002F);
    (void)query("4 adrc shaft_pilot direction=positive profile=platform70", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(!bridge.shaft.used);
    platform70_fixture(); (void)query("4 adrc shaft_pilot direction=positive profile=platform", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    platform70_fixture(); (void)query("4 adrc shaft_pilot direction=positive profile=observe", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    platform70_fixture(); (void)query("4 adrc shaft_pilot direction=positive profile=platform_wide", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    fixture(); (void)query("4 adrc shaft_pilot direction=positive profile=platform", PROTOCOL_ENGINE_STATUS_OK);
    output = query("5 adrc shaft_binding index=4", PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "profile=5") && strstr(output.messages[0].data, "reserve=3.14159274"));
    ++groups;
}
/** @brief Preserve the integrated platform100 command and frozen binding without granting qualification. */
static void test_platform100_integrated_command(void)
{
    ProtocolOutputBatch output;
    platform70_fixture();
    (void)query("4 adrc shaft_pilot direction=positive profile=platform100 cap=1", PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(!bridge.shaft.used);
    platform70_fixture();
    (void)query("4 adrc shaft_pilot direction=positive profile=platform100", PROTOCOL_ENGINE_STATUS_OK);
    assert(bridge.shaft.core->platform100_profile && bridge.shaft.core->observe_profile);
    assert(fabsf(bridge.shaft.core->plan.cap_nm - 1.0F) < 0.000001F);
    output = query("5 adrc shaft_binding index=4", PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "profile=8") && strstr(output.messages[0].data, "plateau_us=200000"));
    assert(strstr(output.messages[0].data, "rated_rpm=395"));
    assert(!bridge.bench.trusted_qualification.verified_flags);
    ++groups;
}
/** @brief Runs independent behavioral groups; all fabricated trust objects remain in this host executable. */
int main(void)
{
    test_platform100_integrated_command();
    test_platform70_profile_entry(); test_platform70_real_codec_timing(); test_platform70_range_identity();
    test_platform_wide_profile_entry(); test_platform_wide_real_codec_timing(); test_platform_wide_range_identity();
    test_platform_profile_entry(); test_platform_real_codec_timing();
    test_observe_profile_entry(); test_observe_binding_isolation(); test_observe_real_codec_timing();
    test_visible_profile_entry();
    test_low_profile_entry(); test_fixed_entry(); test_baseline(); test_immutable_observation(); test_binding_changes(); test_receipt_rejection();
    test_last_receipt_stop(); test_preparation(); test_natural_and_guard(); test_guard_mismatch();
    test_admission_evidence(); test_prepare_cleanup(); test_export_capacity();
    test_host_export_evidence_sequence(); test_read_only_namespace_classification();
    printf("SHAFT_BRIDGE_TESTS_PASS groups=%u sizeof_core=%lu sizeof_bridge=%lu\n", groups,
        (unsigned long)sizeof(ShaftPilot), (unsigned long)sizeof(AdrcAppBridge)); return 0;
}
