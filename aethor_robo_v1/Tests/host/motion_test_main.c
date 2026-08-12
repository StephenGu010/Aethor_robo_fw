/**
 * @file motion_test_main.c
 * @brief Host-side tests for seven-axis POS_VEL and MIT motion mathematics.
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "arm_config.h"
#include "joint_motion.h"
#include "joint_motion_can.h"

/** @brief Builds deterministic commissioned limits for pure motion tests. */
static ArmConfig make_motion_test_configuration(void)
{
    ArmConfig configuration = *arm_config_get_production();
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        JointConfig *joint = &configuration.joints[joint_index];

        joint->direction = 1;
        joint->soft_limit_min_rad = -3.0F;
        joint->soft_limit_max_rad = 3.0F;
        joint->max_velocity_rad_s = 1.0F + (float)joint_index;
        joint->max_acceleration_rad_s2 = 2.0F + (float)joint_index;
        joint->mit_kp = 20.0F;
        joint->mit_kd = 1.0F;
        joint->motor_pmax_rad = 12.5F;
        joint->motor_vmax_rad_s = 45.0F;
        joint->motor_tmax_nm = 18.0F;
        joint->gear_ratio = 1.0F;
        joint->position_tolerance_rad = 0.01F;
        joint->velocity_tolerance_rad_s = 0.02F;
        joint->verified_fields = ARM_JOINT_REQUIRED_ENABLE_FIELDS;
    }
    return configuration;
}

/** @brief Verifies POS_VEL axes share one duration and remain within limits. */
static void test_pos_vel_plan_uses_common_duration(void)
{
    ArmConfig configuration = make_motion_test_configuration();
    float start[ARM_JOINT_COUNT] = {0.0F};
    float target[ARM_JOINT_COUNT] = {1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    JointMotionPlan plan;

    assert(joint_motion_plan(&configuration,
                             start,
                             target,
                             0.5F,
                             JOINT_MOTION_MODE_POSITION_VELOCITY,
                             1000U,
                             &plan) == JOINT_MOTION_STATUS_OK);
    assert(plan.duration_us == 2000000U);
    assert(fabsf(plan.command_velocity_rad_s[0] - 0.5F) < 0.0001F);
    assert(fabsf(plan.command_velocity_rad_s[1] - 0.5F) < 0.0001F);
    assert(plan.command_velocity_rad_s[2] == 0.0F);
}

/** @brief Verifies invalid, non-finite, and out-of-limit plans fail atomically. */
static void test_motion_plan_rejects_bad_inputs(void)
{
    ArmConfig configuration = make_motion_test_configuration();
    float start[ARM_JOINT_COUNT] = {0.0F};
    float target[ARM_JOINT_COUNT] = {0.0F};
    JointMotionPlan plan;

    target[3] = 3.1F;
    assert(joint_motion_plan(&configuration, start, target, 0.5F,
                             JOINT_MOTION_MODE_POSITION_VELOCITY, 0U, &plan) ==
           JOINT_MOTION_STATUS_TARGET_OUT_OF_RANGE);
    target[3] = NAN;
    assert(joint_motion_plan(&configuration, start, target, 0.5F,
                             JOINT_MOTION_MODE_POSITION_VELOCITY, 0U, &plan) ==
           JOINT_MOTION_STATUS_NONFINITE_VALUE);
    target[3] = 0.0F;
    assert(joint_motion_plan(&configuration, start, target, 0.009F,
                             JOINT_MOTION_MODE_POSITION_VELOCITY, 0U, &plan) ==
           JOINT_MOTION_STATUS_SPEED_RATIO_OUT_OF_RANGE);
}

/** @brief Verifies quintic MIT sampling has zero boundary velocity/acceleration. */
static void test_mit_quintic_boundaries_and_limits(void)
{
    ArmConfig configuration = make_motion_test_configuration();
    float start[ARM_JOINT_COUNT] = {0.0F};
    float target[ARM_JOINT_COUNT] = {1.0F, -0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    JointMotionPlan plan;
    JointMotionSample sample;
    uint8_t joint_index;

    assert(joint_motion_plan(&configuration, start, target, 1.0F,
                             JOINT_MOTION_MODE_MIT, 100000U, &plan) ==
           JOINT_MOTION_STATUS_OK);
    assert(joint_motion_sample(&plan, 100000U, &sample) == JOINT_MOTION_STATUS_OK);
    assert(sample.position_rad[0] == 0.0F);
    assert(sample.velocity_rad_s[0] == 0.0F);
    assert(sample.acceleration_rad_s2[0] == 0.0F);

    assert(joint_motion_sample(&plan,
                               plan.start_time_us + plan.duration_us,
                               &sample) == JOINT_MOTION_STATUS_OK);
    assert(fabsf(sample.position_rad[0] - 1.0F) < 0.0001F);
    assert(sample.velocity_rad_s[0] == 0.0F);
    assert(sample.acceleration_rad_s2[0] == 0.0F);

    for (joint_index = 0U; joint_index <= 100U; ++joint_index)
    {
        uint64_t sample_time = plan.start_time_us +
                               (plan.duration_us * joint_index) / 100U;
        assert(joint_motion_sample(&plan, sample_time, &sample) ==
               JOINT_MOTION_STATUS_OK);
        assert(fabsf(sample.velocity_rad_s[0]) <=
               configuration.joints[0].max_velocity_rad_s + 0.0001F);
        assert(fabsf(sample.acceleration_rad_s2[0]) <=
               configuration.joints[0].max_acceleration_rad_s2 + 0.0001F);
    }
}

/** @brief Verifies completion requires continuous valid low-error feedback. */
static void test_motion_completion_requires_settle_window(void)
{
    ArmConfig configuration = make_motion_test_configuration();
    float start[ARM_JOINT_COUNT] = {0.0F};
    float target[ARM_JOINT_COUNT] = {0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    float feedback_position[ARM_JOINT_COUNT] = {0.5F, 0.0F, 0.0F, 0.0F,
                                                0.0F, 0.0F, 0.0F};
    float feedback_velocity[ARM_JOINT_COUNT] = {0.0F};
    JointMotionPlan plan;
    JointMotionCompletion completion;

    assert(joint_motion_plan(&configuration, start, target, 1.0F,
                             JOINT_MOTION_MODE_POSITION_VELOCITY, 0U, &plan) ==
           JOINT_MOTION_STATUS_OK);
    joint_motion_completion_init(&completion);
    assert(joint_motion_update_completion(&plan, &configuration,
                                          feedback_position, feedback_velocity,
                                          0x7FU, 1000000U, 200000U,
                                          &completion) == JOINT_MOTION_STATUS_OK);
    assert(completion.completed == 0U);
    assert(joint_motion_update_completion(&plan, &configuration,
                                          feedback_position, feedback_velocity,
                                          0x7FU, 1199999U, 200000U,
                                          &completion) == JOINT_MOTION_STATUS_OK);
    assert(completion.completed == 0U);
    assert(joint_motion_update_completion(&plan, &configuration,
                                          feedback_position, feedback_velocity,
                                          0x7FU, 1200000U, 200000U,
                                          &completion) == JOINT_MOTION_STATUS_OK);
    assert(completion.completed == 1U);

    joint_motion_completion_init(&completion);
    feedback_position[0] = 0.6F;
    assert(joint_motion_update_completion(&plan, &configuration,
                                          feedback_position, feedback_velocity,
                                          0x7FU, 1300000U, 200000U,
                                          &completion) == JOINT_MOTION_STATUS_OK);
    assert(completion.settle_started == 0U);
    assert(joint_motion_update_completion(&plan, &configuration,
                                          feedback_position, feedback_velocity,
                                          0x3FU, 1400000U, 200000U,
                                          &completion) ==
           JOINT_MOTION_STATUS_FEEDBACK_INCOMPLETE);
}

/**
 * @brief Verifies seven control frames are encoded as one validated J1-J7 group.
 */
static void test_motion_can_group_encoding(void)
{
    ArmConfig configuration = make_motion_test_configuration();
    JointReference reference;
    MotorFeedbackSnapshot motor_snapshot = {0};
    MotorRuntime motor_runtime = {0};
    JointMotionPlan plan;
    JointMotionSample sample;
    CanFrame frames[ARM_JOINT_COUNT];
    float alignment_degrees[ARM_JOINT_COUNT] = {0};
    float start_position_rad[ARM_JOINT_COUNT] = {0};
    float target_position_rad[ARM_JOINT_COUNT] = {0.5F, 0.4F, 0.3F, 0.2F,
                                                  0.1F, -0.1F, -0.2F};
    uint8_t joint_index;

    motor_snapshot.valid_joint_mask = 0x7FU;
    motor_snapshot.generation = 1U;
    assert(joint_reference_init(&reference, &configuration) ==
           JOINT_REFERENCE_STATUS_OK);
    assert(joint_reference_align(&reference,
                                 &motor_snapshot,
                                 alignment_degrees,
                                 1000U) == JOINT_REFERENCE_STATUS_OK);
    motor_runtime.configuration = &configuration;
    motor_runtime.initialized = 1U;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        motor_runtime.discovery.results[joint_index].ranges.position_max_rad = 12.5F;
        motor_runtime.discovery.results[joint_index].ranges.velocity_max_rad_s = 45.0F;
        motor_runtime.discovery.results[joint_index].ranges.torque_max_nm = 18.0F;
    }

    assert(joint_motion_plan(&configuration,
                             start_position_rad,
                             target_position_rad,
                             0.5F,
                             JOINT_MOTION_MODE_POSITION_VELOCITY,
                             2000U,
                             &plan) == JOINT_MOTION_STATUS_OK);
    assert(joint_motion_sample(&plan, 2000U, &sample) ==
           JOINT_MOTION_STATUS_OK);
    assert(joint_motion_can_pack_group(&plan,
                                       &sample,
                                       &reference,
                                       &motor_runtime,
                                       frames) == JOINT_MOTION_CAN_STATUS_OK);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        assert(frames[joint_index].identifier ==
               (uint16_t)(0x101U + joint_index));
        assert(frames[joint_index].length == 8U);
    }
    {
        float encoded_velocity_rad_s = 0.0F;

        memcpy(&encoded_velocity_rad_s, &frames[0].data[4], sizeof(float));
        assert(fabsf(encoded_velocity_rad_s - 0.5F) < 0.0001F);
    }

    assert(joint_motion_plan(&configuration,
                             start_position_rad,
                             target_position_rad,
                             0.5F,
                             JOINT_MOTION_MODE_MIT,
                             3000U,
                             &plan) == JOINT_MOTION_STATUS_OK);
    assert(joint_motion_sample(&plan, 3000U, &sample) ==
           JOINT_MOTION_STATUS_OK);
    assert(joint_motion_can_pack_group(&plan,
                                       &sample,
                                       &reference,
                                       &motor_runtime,
                                       frames) == JOINT_MOTION_CAN_STATUS_OK);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        assert(frames[joint_index].identifier ==
               (uint16_t)(1U + joint_index));
        assert(frames[joint_index].length == 8U);
    }
}

/** @brief Runs all motion algorithm tests. */
int main(void)
{
    test_pos_vel_plan_uses_common_duration();
    test_motion_plan_rejects_bad_inputs();
    test_mit_quintic_boundaries_and_limits();
    test_motion_completion_requires_settle_window();
    test_motion_can_group_encoding();
    puts("MOTION_TESTS_PASSED");
    return 0;
}
