/**
 * @file joint_controller.h
 * @brief Static seven-joint S3519 controller, commissioning lock, and safety state machine.
 */

#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <stddef.h>
#include <stdint.h>

#include "dm_motor_protocol.h"
#include "robot_config.h"
#include "sync_trajectory.h"
#include "usb_command.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JOINT_COUNT ROBOT_JOINT_COUNT
#define JOINT_CONTROLLER_PERIOD_MS 5U
#define JOINT_CONTROLLER_FEEDBACK_TIMEOUT_MS 50U
#define JOINT_CONTROLLER_COMMAND_TIMEOUT_MS 30000U
#define JOINT_CONTROLLER_COMMISSIONING_POSITION_LIMIT_DEG 3.0F
#define JOINT_CONTROLLER_COMMISSIONING_VELOCITY_DEG_S 3.0F
#define JOINT_CONTROLLER_COMMISSIONING_ACCELERATION_DEG_S2 6.0F
#define JOINT_CONTROLLER_ARRIVAL_POSITION_DEG 0.5F
#define JOINT_CONTROLLER_ARRIVAL_VELOCITY_DEG_S 1.0F
#define JOINT_CONTROLLER_ARRIVAL_CYCLES 3U
#define JOINT_CONTROLLER_BOOT_DELAY_MS 2000U
#define JOINT_CONTROLLER_STARTUP_TIMEOUT_MS 100U
#define JOINT_CONTROLLER_STARTUP_RETRY_LIMIT 3U
#define JOINT_CONTROLLER_READY_POLL_INTERVAL_MS 100U

/** @brief Joint-controller operation results. */
typedef enum
{
    JOINT_CONTROLLER_STATUS_OK = 0,
    JOINT_CONTROLLER_STATUS_INVALID_ARGUMENT,
    JOINT_CONTROLLER_STATUS_INVALID_SELECTION,
    JOINT_CONTROLLER_STATUS_RANGES_NOT_READY,
    JOINT_CONTROLLER_STATUS_NOT_ENABLED,
    JOINT_CONTROLLER_STATUS_COMMISSIONING_LIMIT,
    JOINT_CONTROLLER_STATUS_FULL_ROBOT_LOCKED,
    JOINT_CONTROLLER_STATUS_CAN_ERROR,
    JOINT_CONTROLLER_STATUS_SAFETY_LATCHED,
    JOINT_CONTROLLER_STATUS_COMMAND_BUSY,
    JOINT_CONTROLLER_STATUS_RESPONSE_TOO_SMALL
} JointControllerStatus;

/** @brief Latched reason for the most recent safety shutdown. */
typedef enum
{
    JOINT_CONTROLLER_SAFETY_NONE = 0,
    JOINT_CONTROLLER_SAFETY_COMMAND_TIMEOUT,
    JOINT_CONTROLLER_SAFETY_FEEDBACK_TIMEOUT,
    JOINT_CONTROLLER_SAFETY_INVALID_COMMAND,
    JOINT_CONTROLLER_SAFETY_MOTOR_FAULT,
    JOINT_CONTROLLER_SAFETY_BUS_OFF,
    JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT,
    JOINT_CONTROLLER_SAFETY_STARTUP_TIMEOUT,
    JOINT_CONTROLLER_SAFETY_MODE_MISMATCH
} JointControllerSafetyReason;

/** @brief Observable seven-axis control stages used by USB and firmware probes. */
typedef enum
{
    JOINT_CONTROLLER_STAGE_CONFIG_VALIDATE = 0,
    JOINT_CONTROLLER_STAGE_BOOT_DELAY,
    JOINT_CONTROLLER_STAGE_MODE_SETUP,
    JOINT_CONTROLLER_STAGE_RANGE_DISCOVERY,
    JOINT_CONTROLLER_STAGE_READY,
    JOINT_CONTROLLER_STAGE_ENABLING,
    JOINT_CONTROLLER_STAGE_MOVING,
    JOINT_CONTROLLER_STAGE_HOLDING,
    JOINT_CONTROLLER_STAGE_FAULT
} JointControllerStage;

/** @brief Public controller state used by USB diagnostics and tests. */
typedef struct
{
    uint8_t selected_joint;
    uint8_t active_mask;
    uint8_t mode_ready_mask;
    uint8_t enable_pending_mask;
    uint8_t enabled_mask;
    uint8_t ranges_ready_mask;
    uint8_t feedback_ready_mask;
    uint8_t arrived_mask;
    uint8_t motor_fault_mask;
    uint8_t commissioning_lock_active;
    uint8_t trajectory_active;
    JointControllerStage stage;
    JointControllerSafetyReason safety_reason;
    uint32_t last_command_time_ms;
    uint32_t last_feedback_time_ms[JOINT_COUNT];
    uint32_t first_arrival_time_ms[JOINT_COUNT];
    uint8_t motor_state[JOINT_COUNT];
    float joint_position_degrees[JOINT_COUNT];
    float joint_velocity_degrees_s[JOINT_COUNT];
    float motor_position_degrees[JOINT_COUNT];
    float motor_velocity_degrees_s[JOINT_COUNT];
    float commanded_position_degrees[JOINT_COUNT];
} JointControllerState;

typedef int (*JointControllerSendFunction)(const FdcanClassicFrame *frame);

void joint_controller_init(JointControllerSendFunction send_function,
                           const RobotConfiguration *configuration,
                           uint32_t time_ms);
JointControllerStatus joint_controller_submit(const UsbCommand *command,
                                              uint32_t time_ms,
                                              char *response,
                                              size_t response_capacity);
void joint_controller_step(uint32_t time_ms);
void joint_controller_on_can_frame(const FdcanClassicFrame *frame, uint32_t time_ms);
void joint_controller_on_invalid_command(void);
void joint_controller_on_can_start_failure(void);
void joint_controller_on_bus_off(void);
const JointControllerState *joint_controller_get_state(void);
const RobotConfiguration *joint_controller_get_configuration(void);

#ifdef __cplusplus
}
#endif

#endif /* JOINT_CONTROLLER_H */
