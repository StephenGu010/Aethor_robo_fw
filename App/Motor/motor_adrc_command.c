/** @file motor_adrc_command.c
 * @brief Keeps safe integer-grid torque selection, MIT packing and decode verification inside Motor.
 */
#include "motor_adrc_command.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

/** @brief Checks an external scalar before interval arithmetic or integer conversion. */
static uint8_t finite_value(float value)
{
    return (uint8_t)(value == value && value <= FLT_MAX && value >= -FLT_MAX);
}

/** @brief Requires selected-axis identity, MIT mode and complete dynamic mapping ranges. */
static uint8_t mit_discovered(const MotorRuntime *runtime, uint8_t axis_index)
{
    const MotorDiscoveryResult *discovery = &runtime->discovery.results[axis_index];
    uint16_t required = MOTOR_DISCOVERY_IDENTITY_FIELDS_MASK | MOTOR_DISCOVERY_MODE_FIELDS_MASK |
        MOTOR_DISCOVERY_RANGE_FIELDS_MASK;
    return (uint8_t)((discovery->verified_fields_mask & required) == required &&
        discovery->observed_control_mode == 1U);
}

/** @brief Commits a frame only after its actual integer code satisfies amplitude, step and zero-gain checks. */
MotorRuntimeStatus motor_adrc_build_torque_frame(const MotorRuntime *runtime,
    const MotorAdrcTorqueRequest *request, CanFrame *frame, float *decoded_torque_nm)
{
    CanFrame encoded_frame;
    float scale;
    float range;
    float limit;
    float step;
    float previous;
    float lower;
    float upper;
    float target;
    float best_error = FLT_MAX;
    float selected_torque = 0.0F;
    float decoded_frame_torque;
    int center_code;
    int candidate_code;
    uint16_t selected_code = 0U;
    uint8_t found = 0U;
    if (runtime == NULL || request == NULL || frame == NULL || decoded_torque_nm == NULL ||
        request->axis_index >= ARM_JOINT_COUNT) { return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT; }
    if (!runtime->initialized || runtime->configuration == NULL) { return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED; }
    if (!mit_discovered(runtime, request->axis_index)) { return MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE; }
    scale = request->torque_scale;
    limit = request->torque_limit_nm;
    step = request->torque_step_nm;
    previous = request->previous_sent_nm;
    target = request->torque_nm;
    if (!finite_value(scale) || scale == 0.0F || !finite_value(target) || !finite_value(previous) ||
        !finite_value(limit) || !finite_value(step) || limit <= 0.0F || step <= 0.0F)
    { return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT; }
    range = runtime->discovery.results[request->axis_index].ranges.torque_max_nm;
    if (!finite_value(range) || range <= 0.0F) { return MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE; }
    lower = previous - step;
    upper = previous + step;
    if (lower < -limit) { lower = -limit; }
    if (upper > limit) { upper = limit; }
    if (lower > upper) { return MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE; }
    if (target < lower) { target = lower; }
    if (target > upper) { target = upper; }
    /* The grid has no guaranteed true zero; never widen a physical bound to fit an integer code. */
    {
        double encoded = (((double)target / (double)scale + (double)range) /
            (2.0 * (double)range)) * 4095.0;
        if (encoded < 0.0) { encoded = 0.0; }
        if (encoded > 4095.0) { encoded = 4095.0; }
        center_code = (int)encoded;
    }
    for (candidate_code = center_code - 2; candidate_code <= center_code + 2; ++candidate_code)
    {
        float decoded;
        float error;
        if (candidate_code < 0 || candidate_code > 4095) { continue; }
        decoded = (((float)candidate_code * (2.0F * range) / 4095.0F) - range) * scale;
        if (!finite_value(decoded) || decoded < lower || decoded > upper ||
            fabsf(decoded - previous) > step || fabsf(decoded) > limit) { continue; }
        error = fabsf(decoded - request->torque_nm);
        if (!found || error < best_error)
        {
            selected_code = (uint16_t)candidate_code;
            selected_torque = decoded;
            best_error = error;
            found = 1U;
        }
    }
    if (!found) { return MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE; }
    if (s3519_pack_mit((uint8_t)runtime->configuration->joints[request->axis_index].esc_id,
        &runtime->discovery.results[request->axis_index].ranges, 0.0F, 0.0F, 0.0F, 0.0F,
        selected_torque / scale, &encoded_frame) != S3519_CODEC_STATUS_OK)
    { return MOTOR_RUNTIME_STATUS_CODEC_ERROR; }
    encoded_frame.data[6] = (uint8_t)((encoded_frame.data[6] & 0xF0U) | (selected_code >> 8U));
    encoded_frame.data[7] = (uint8_t)selected_code;
    decoded_frame_torque = (((float)(((uint16_t)(encoded_frame.data[6] & 0x0FU) << 8U) |
        encoded_frame.data[7]) * (2.0F * range) / 4095.0F) - range) * scale;
    if (decoded_frame_torque < lower || decoded_frame_torque > upper ||
        fabsf(decoded_frame_torque - previous) > step || fabsf(decoded_frame_torque) > limit ||
        (encoded_frame.data[3] & 0x0FU) != 0U || encoded_frame.data[4] != 0U ||
        encoded_frame.data[5] != 0U || (encoded_frame.data[6] & 0xF0U) != 0U)
    { return MOTOR_RUNTIME_STATUS_CODEC_ERROR; }
    *frame = encoded_frame;
    *decoded_torque_nm = decoded_frame_torque;
    return MOTOR_RUNTIME_STATUS_OK;
}
