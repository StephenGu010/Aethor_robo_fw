/**
 * @file joint_motion_can.c
 * @brief Implements atomic seven-axis joint-sample to S3519 CAN encoding.
 */

#include "joint_motion_can.h"

#include <stddef.h>

/**
 * @brief Encodes one sample into an ordered all-or-nothing J1-J7 CAN group.
 */
JointMotionCanStatus joint_motion_can_pack_group(
    const JointMotionPlan *plan,
    const JointMotionSample *sample,
    const JointReference *reference,
    const MotorRuntime *motor_runtime,
    CanFrame frames[ARM_JOINT_COUNT])
{
    float motor_position_rad[ARM_JOINT_COUNT];
    float motor_velocity_rad_s[ARM_JOINT_COUNT];
    S3519ControlMode control_mode;

    if ((plan == NULL) || (sample == NULL) || (reference == NULL) ||
        (motor_runtime == NULL) || (frames == NULL) ||
        ((plan->mode != JOINT_MOTION_MODE_POSITION_VELOCITY) &&
         (plan->mode != JOINT_MOTION_MODE_MIT)))
    {
        return JOINT_MOTION_CAN_STATUS_INVALID_ARGUMENT;
    }
    if (joint_reference_joint_to_motor(reference,
                                       sample->position_rad,
                                       sample->velocity_rad_s,
                                       motor_position_rad,
                                       motor_velocity_rad_s) !=
        JOINT_REFERENCE_STATUS_OK)
    {
        return JOINT_MOTION_CAN_STATUS_REFERENCE_ERROR;
    }
    control_mode = (plan->mode == JOINT_MOTION_MODE_MIT)
                       ? S3519_CONTROL_MODE_MIT
                       : S3519_CONTROL_MODE_POSITION_VELOCITY;
    return (motor_runtime_build_control_group(motor_runtime,
                                              control_mode,
                                              motor_position_rad,
                                              motor_velocity_rad_s,
                                              frames) ==
            MOTOR_RUNTIME_STATUS_OK)
               ? JOINT_MOTION_CAN_STATUS_OK
               : JOINT_MOTION_CAN_STATUS_CODEC_ERROR;
}
