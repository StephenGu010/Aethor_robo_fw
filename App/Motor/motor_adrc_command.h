/** @file motor_adrc_command.h
 * @brief Motor-layer bounded nominal torque encoding using discovered MIT ranges.
 */
#ifndef APP_MOTOR_ADRC_COMMAND_H
#define APP_MOTOR_ADRC_COMMAND_H
#include "motor_runtime.h"

/** @brief Immutable output-shaft request; the caller supplies its actual previous transmit value. */
typedef struct
{
    float torque_nm;
    float previous_sent_nm;
    float torque_scale;
    float torque_limit_nm;
    float torque_step_nm;
    uint8_t axis_index;
} MotorAdrcTorqueRequest;

/** @brief Selects a safe 12-bit torque code, packs zero-gain MIT and returns its nominal decoded value.
 * Returns RANGE_UNAVAILABLE without changing outputs if no safe grid point exists.
 * The caller supplies the permitted per-cycle step; this module owns no controller or sample clock.
 * Successful encoding does not prove transmission or measured physical torque.
 */
MotorRuntimeStatus motor_adrc_build_torque_frame(const MotorRuntime *runtime,
    const MotorAdrcTorqueRequest *request, CanFrame *frame, float *decoded_torque_nm);
#endif
