/**
 * @file joint_motion_can.h
 * @brief Defines atomic S3519 frame encoding for seven-axis motion samples.
 */

#ifndef APP_MOTION_JOINT_MOTION_CAN_H
#define APP_MOTION_JOINT_MOTION_CAN_H

#include "joint_motion.h"
#include "joint_reference.h"
#include "motor_runtime.h"

/** @brief Reports whole-group conversion and S3519 encoding outcomes. */
typedef enum
{
    JOINT_MOTION_CAN_STATUS_OK = 0,
    JOINT_MOTION_CAN_STATUS_INVALID_ARGUMENT,
    JOINT_MOTION_CAN_STATUS_REFERENCE_ERROR,
    JOINT_MOTION_CAN_STATUS_CODEC_ERROR
} JointMotionCanStatus;

/**
 * @brief Encodes one sample into an ordered all-or-nothing J1-J7 CAN group.
 * @param plan Active immutable motion plan selecting POS_VEL or MIT.
 * @param sample Same-cycle seven-axis joint-space sample.
 * @param reference Aligned motor/joint reference conversion domain.
 * @param motor_runtime Runtime owning motor identities, gains, and ranges.
 * @param frames Destination ordered seven-frame group, unchanged on failure.
 * @return OK or a precise validation, mapping, or codec failure.
 */
JointMotionCanStatus joint_motion_can_pack_group(
    const JointMotionPlan *plan,
    const JointMotionSample *sample,
    const JointReference *reference,
    const MotorRuntime *motor_runtime,
    CanFrame frames[ARM_JOINT_COUNT]);

#endif
