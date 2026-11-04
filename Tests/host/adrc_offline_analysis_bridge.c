/** @file adrc_offline_analysis_bridge.c
 * @brief Host-only adapters to production code; synthetic state never reaches hardware.
 * The process has no HAL, serial, CAN adapter or debugger dependency.
 */
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "s3519_codec.h"
#include "motor_adrc_command.h"
#include "adrc_lcd_ownership.h"
#include "adrc_generated_adapter.h"
#include "adrc_controller.h"
#include "can_rx_inbox.h"
#include "adrc_feedback_adapter.h"

static S3519Ranges analysis_ranges;
static ArmConfig analysis_configuration;
static MotorRuntime analysis_runtime;
static AdrcLcdOwnership analysis_owner;
static AdrcFeedbackAdapter analysis_estimator;

/** @brief Initialize an in-memory motor-7 fixture and reset the real generated controller. */
int analysis_initialize(float position_range, float velocity_range, float torque_range)
{
    if (!isfinite(position_range) || !isfinite(velocity_range) || !isfinite(torque_range) ||
        position_range <= 0.0F || velocity_range <= 0.0F || torque_range <= 0.0F) { return 0; }
    analysis_ranges.position_max_rad = position_range;
    analysis_ranges.velocity_max_rad_s = velocity_range;
    analysis_ranges.torque_max_nm = torque_range;
    memset(&analysis_configuration, 0, sizeof(analysis_configuration));
    memset(&analysis_runtime, 0, sizeof(analysis_runtime));
    analysis_configuration.joint_count = ARM_JOINT_COUNT;
    analysis_configuration.joints[6].esc_id = 7U;
    analysis_runtime.configuration = &analysis_configuration;
    analysis_runtime.initialized = 1U;
    analysis_runtime.discovery.results[6].ranges = analysis_ranges;
    analysis_runtime.discovery.results[6].observed_control_mode = 1U;
    analysis_runtime.discovery.results[6].verified_fields_mask =
        MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK | MOTOR_DISCOVERY_MODE_FIELDS_MASK |
        MOTOR_DISCOVERY_RANGE_FIELDS_MASK;
    adrc_controller_initialize();
    adrc_feedback_adapter_reset(&analysis_estimator);
    return 1;
}

/** @brief Apply the production 32 ms estimator to simulated decoded feedback and real-time bounds. */
int analysis_estimate(uint64_t now_us, uint64_t feedback_us, float position_rad,
    float raw_velocity_rad_s, float *velocity_rad_s)
{
    uint8_t ready;
    if (velocity_rad_s == NULL) { return 0; }
    ready = adrc_feedback_adapter_update(&analysis_estimator, now_us, feedback_us,
        position_rad, raw_velocity_rad_s, 2.0F * analysis_ranges.position_max_rad / 65535.0F);
    *velocity_rad_s = ready ? analysis_estimator.velocity_rad_s : 0.0F;
    return (int)ready;
}

/** @brief Decode explicit simulated feedback codes with the production float mapping. */
int analysis_decode(uint32_t position_code, uint32_t velocity_code, uint32_t torque_code,
    float *values)
{
    CanFrame frame;
    S3519Feedback feedback;
    if (values == NULL || position_code > 65535U || velocity_code > 4095U || torque_code > 4095U)
    { return 0; }
    memset(&frame, 0, sizeof(frame));
    frame.identifier = 0x17U;
    frame.length = 8U;
    frame.data[0] = 0x17U;
    frame.data[1] = (uint8_t)(position_code >> 8U);
    frame.data[2] = (uint8_t)position_code;
    frame.data[3] = (uint8_t)(velocity_code >> 4U);
    frame.data[4] = (uint8_t)(((velocity_code & 15U) << 4U) | (torque_code >> 8U));
    frame.data[5] = (uint8_t)torque_code;
    if (s3519_decode_feedback(&frame, &analysis_ranges, &feedback) != S3519_CODEC_STATUS_OK)
    { return 0; }
    values[0] = feedback.position_rad;
    values[1] = feedback.velocity_rad_s;
    values[2] = feedback.torque_nm;
    return 1;
}

/** @brief Return production MIT zero-request bytes, including its nonzero nominal torque code. */
int analysis_neutral(uint8_t *bytes)
{
    CanFrame frame;
    if (bytes == NULL || s3519_pack_mit(7U, &analysis_ranges, 0, 0, 0, 0, 0, &frame) !=
        S3519_CODEC_STATUS_OK) { return 0; }
    memcpy(bytes, frame.data, 8U);
    return 1;
}

/** @brief Apply the production bounded lattice selector; return status and decoded nominal torque. */
int analysis_torque(float target, float previous, float limit, float step, float *sent)
{
    MotorAdrcTorqueRequest request;
    CanFrame frame;
    memset(&request, 0, sizeof(request));
    request.axis_index = 6U;
    request.torque_scale = 1.0F;
    request.torque_nm = target;
    request.previous_sent_nm = previous;
    request.torque_limit_nm = limit;
    request.torque_step_nm = step;
    return (int)motor_adrc_build_torque_frame(&analysis_runtime, &request, &frame, sent);
}

/** @brief Step the real generated PI/LADRC code using a synthetic observation and previous command. */
int analysis_control(uint8_t mode, float reference, float measurement, float previous,
    float b0, float wc, float wo, uint8_t reset, uint8_t new_sample, float *outputs)
{
    AdrcControllerInput input;
    AdrcControllerOutput output;
    if (outputs == NULL) { return 0; }
    memset(&input, 0, sizeof(input));
    input.mode = (AdrcExperimentMode)mode;
    input.reference_rad_s = reference;
    input.velocity_rad_s = measurement;
    input.prev_sent_nm = previous;
    input.b0 = b0;
    input.wc_rad_s = wc;
    input.wo_rad_s = wo;
    input.reset = reset;
    input.new_sample = new_sample;
    if (!adrc_generated_controller_step(NULL, &input, &output)) { return 0; }
    outputs[0] = output.torque_raw_nm;
    outputs[1] = output.z1;
    outputs[2] = output.z2;
    return 1;
}

/** @brief Queue a real ownership request in a fresh synthetic state machine. */
int analysis_acquire(void)
{
    adrc_lcd_ownership_init(&analysis_owner);
    return (int)adrc_lcd_ownership_submit_acquire(&analysis_owner, 6U, 1U);
}

/** @brief Expose deterministic handoff snapshots; flags encode the nine explicit evidence fields. */
int analysis_owner_step(uint64_t now, uint64_t feedback, uint64_t submitted, uint32_t flags)
{
    AdrcLcdOwnershipEvidence evidence;
    AdrcLcdOwnershipStatus status;
    memset(&evidence, 0, sizeof(evidence));
    evidence.now_us = now;
    evidence.feedback_us = feedback;
    evidence.probe_submitted_us = submitted;
    evidence.lcd_idle = (uint8_t)((flags >> 0) & 1U);
    evidence.can_idle = (uint8_t)((flags >> 1) & 1U);
    evidence.probe_transmitted = (uint8_t)((flags >> 2) & 1U);
    evidence.probe_failed = (uint8_t)((flags >> 3) & 1U);
    evidence.feedback_fresh = (uint8_t)((flags >> 4) & 1U);
    evidence.disabled = (uint8_t)((flags >> 5) & 1U);
    evidence.no_fault = (uint8_t)((flags >> 6) & 1U);
    evidence.stationary = (uint8_t)((flags >> 7) & 1U);
    evidence.mit_discovered = (uint8_t)((flags >> 8) & 1U);
    status = adrc_lcd_ownership_service(&analysis_owner, &evidence);
    return (int)analysis_owner.state * 16 + (int)status;
}

/** @brief Replay delayed dequeue through the production inbox without inventing a newer arrival. */
int analysis_queued_owner_step(uint64_t now, uint64_t arrival_lower_bound,
    uint64_t submitted, uint32_t flags)
{
    CanRxInbox inbox;
    CanFrame frame;
    uint64_t retained_bound = 0ULL;
    memset(&frame, 0, sizeof(frame));
    can_rx_inbox_init(&inbox);
    if (can_rx_inbox_push_isr_at(&inbox, &frame, arrival_lower_bound) != CAN_RX_INBOX_STATUS_OK ||
        can_rx_inbox_pop_at(&inbox, &frame, &retained_bound) != CAN_RX_INBOX_STATUS_OK)
    { return -1; }
    return analysis_owner_step(now, retained_bound, submitted, flags);
}

/** @brief Queue release and independently report synthetic submit/receipt times. */
int analysis_release(uint64_t now, uint64_t submitted, uint64_t transmitted, uint8_t success)
{
    AdrcLcdOwnershipStatus status = adrc_lcd_ownership_submit_release(&analysis_owner, 2U, now);
    adrc_lcd_ownership_report_disable_submit(&analysis_owner, submitted);
    adrc_lcd_ownership_report_disable_transmit(&analysis_owner, transmitted, success);
    return (int)status;
}
