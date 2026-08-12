/**
 * @file dual_motor_controller.h
 * @brief Hardware-independent KEY1 controller for two S3519 position-speed motors.
 */

#ifndef DUAL_MOTOR_CONTROLLER_H
#define DUAL_MOTOR_CONTROLLER_H

#include <stdint.h>

#include "dm_motor_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DUAL_MOTOR_COUNT 2U
#define DUAL_MOTOR_ALL_MASK 0x03U
#define DUAL_MOTOR_CONTROL_PERIOD_MS 5U
#define DUAL_MOTOR_KEY_DEBOUNCE_MS 20U
#define DUAL_MOTOR_FEEDBACK_TIMEOUT_MS 50U
#define DUAL_MOTOR_MOVEMENT_TIMEOUT_MS 30000U
#define DUAL_MOTOR_TARGET_VELOCITY_RAD_S 0.5F

/** @brief Public high-level stages of the dual-motor controller. */
typedef enum
{
    DUAL_MOTOR_STAGE_BOOT_DELAY = 0,
    DUAL_MOTOR_STAGE_CONFIGURING,
    DUAL_MOTOR_STAGE_REQUESTING_FEEDBACK,
    DUAL_MOTOR_STAGE_READY,
    DUAL_MOTOR_STAGE_ENABLING,
    DUAL_MOTOR_STAGE_MOVING,
    DUAL_MOTOR_STAGE_HOLDING,
    DUAL_MOTOR_STAGE_FAULT
} DualMotorControllerStage;

/** @brief Latched reasons that stop both motors until the controller is reset. */
typedef enum
{
    DUAL_MOTOR_FAULT_NONE = 0,
    DUAL_MOTOR_FAULT_CONFIGURATION_TIMEOUT,
    DUAL_MOTOR_FAULT_INVALID_PARAMETER,
    DUAL_MOTOR_FAULT_TARGET_RANGE,
    DUAL_MOTOR_FAULT_FEEDBACK_TIMEOUT,
    DUAL_MOTOR_FAULT_MOTOR,
    DUAL_MOTOR_FAULT_MOVEMENT_TIMEOUT,
    DUAL_MOTOR_FAULT_CAN_TRANSMIT,
    DUAL_MOTOR_FAULT_BUS_OFF
} DualMotorControllerFault;

/** @brief Public state used by USB diagnostics and host tests. */
typedef struct
{
    DualMotorControllerStage stage;
    DualMotorControllerFault fault_reason;
    uint8_t stable_key_pressed;
    uint8_t move_accepted;
    int8_t next_direction;
    uint8_t mode_ready_mask;
    uint8_t ranges_ready_mask;
    uint8_t fresh_feedback_mask;
    uint8_t enabled_mask;
    uint8_t arrived_mask;
    uint8_t motor_state[DUAL_MOTOR_COUNT];
    uint32_t accepted_move_count;
    uint32_t last_feedback_time_ms[DUAL_MOTOR_COUNT];
    float position_max_rad[DUAL_MOTOR_COUNT];
    float velocity_max_rad_s[DUAL_MOTOR_COUNT];
    float torque_max_nm[DUAL_MOTOR_COUNT];
    float initial_position_rad[DUAL_MOTOR_COUNT];
    float target_position_rad[DUAL_MOTOR_COUNT];
    float measured_position_rad[DUAL_MOTOR_COUNT];
    float measured_velocity_rad_s[DUAL_MOTOR_COUNT];
} DualMotorControllerState;

typedef int (*DualMotorSendFunction)(const FdcanClassicFrame *frame);

void dual_motor_controller_init(DualMotorSendFunction send_function,
                                uint32_t current_time_ms);
void dual_motor_controller_step(uint32_t current_time_ms,
                                uint8_t user_key_is_pressed);
void dual_motor_controller_on_can_frame(const FdcanClassicFrame *frame,
                                        uint32_t current_time_ms);
void dual_motor_controller_on_can_start_failure(void);
void dual_motor_controller_on_bus_off(void);
const DualMotorControllerState *dual_motor_controller_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* DUAL_MOTOR_CONTROLLER_H */
