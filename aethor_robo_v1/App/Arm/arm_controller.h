/**
 * @file arm_controller.h
 * @brief Defines the seven-axis arm safety state machine and latched faults.
 */

#ifndef APP_ARM_ARM_CONTROLLER_H
#define APP_ARM_ARM_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_config.h"
#include "diagnostics.h"

/**
 * @brief Defines every formal first-arm lifecycle state from the PRD.
 */
typedef enum
{
    ARM_STATE_BOOT = 0,
    ARM_STATE_SELF_TEST,
    ARM_STATE_UNALIGNED,
    ARM_STATE_DISABLED,
    ARM_STATE_ENABLING,
    ARM_STATE_READY,
    ARM_STATE_MOVING,
    ARM_STATE_STOPPING,
    ARM_STATE_FAULT
} ArmState;

/**
 * @brief Distinguishes malformed configuration from incomplete commissioning.
 */
typedef enum
{
    ARM_FAULT_NONE = 0,
    ARM_FAULT_CONFIG_INVALID,
    ARM_FAULT_CONFIG_INCOMPLETE,
    ARM_FAULT_LINK_TIMEOUT,
    ARM_FAULT_MOTION_CONTROL
} ArmFault;

/** @brief Identifies the currently confirmed seven-motor control mode. */
typedef enum
{
    ARM_CONTROL_MODE_UNKNOWN = 0,
    ARM_CONTROL_MODE_POSITION_VELOCITY,
    ARM_CONTROL_MODE_MIT
} ArmControlMode;

/** @brief Reports whether a requested arm-domain transition was accepted. */
typedef enum
{
    ARM_TRANSITION_STATUS_OK = 0,
    ARM_TRANSITION_STATUS_INVALID_ARGUMENT,
    ARM_TRANSITION_STATUS_INVALID_STATE
} ArmTransitionStatus;

/**
 * @brief Owns the Phase 0 arm state without hardware or motor dependencies.
 */
typedef struct
{
    const ArmConfig *configuration;
    Diagnostics *diagnostics;
    ArmState state;
    ArmFault fault;
    ArmControlMode control_mode;
    uint64_t state_entered_at_us;
    uint32_t fault_detail;
    uint8_t aligned;
    uint8_t enabled;
    uint8_t moving;
    uint8_t initialized;
} ArmController;

/**
 * @brief Provides an immutable value snapshot for protocol and telemetry users.
 */
typedef struct
{
    ArmState state;
    ArmFault fault;
    ArmControlMode control_mode;
    uint64_t state_entered_at_us;
    uint32_t fault_detail;
    uint8_t joint_count;
    uint8_t aligned;
    uint8_t enabled;
    uint8_t moving;
} ArmSnapshot;

/**
 * @brief Initializes one controller in the BOOT state.
 * @param controller Controller storage to initialize; null is ignored.
 * @param configuration Immutable configuration used by self-test.
 * @param diagnostics Diagnostic store receiving state events.
 * @param timestamp_us Initialization timestamp in microseconds.
 */
void arm_controller_init(ArmController *controller,
                         const ArmConfig *configuration,
                         Diagnostics *diagnostics,
                         uint64_t timestamp_us);

/**
 * @brief Executes one non-blocking Phase 0 state-machine step.
 * @param controller Initialized controller to service.
 * @param timestamp_us Current monotonic time in microseconds.
 */
void arm_controller_step(ArmController *controller, uint64_t timestamp_us);

/**
 * @brief Commits successful RAM reference alignment into the arm state machine.
 * @param controller Initialized controller.
 * @param timestamp_us Transition timestamp.
 * @return OK only from UNALIGNED or already aligned DISABLED.
 */
ArmTransitionStatus arm_controller_mark_reference_aligned(
    ArmController *controller,
    uint64_t timestamp_us);

/**
 * @brief Latches communication loss and clears all logical enable/motion flags.
 * @param controller Initialized controller.
 * @param timestamp_us Fault timestamp.
 * @return OK or INVALID_ARGUMENT.
 */
ArmTransitionStatus arm_controller_force_stop_disable(
    ArmController *controller,
    uint64_t timestamp_us);

/**
 * @brief Latches a non-configuration runtime fault and clears enable/motion flags.
 * @param controller Initialized controller.
 * @param fault Runtime fault other than NONE or a configuration fault.
 * @param detail Stable subsystem-specific detail code.
 * @param timestamp_us Fault timestamp.
 * @return OK or an argument error.
 */
ArmTransitionStatus arm_controller_latch_runtime_fault(
    ArmController *controller,
    ArmFault fault,
    uint32_t detail,
    uint64_t timestamp_us);

/**
 * @brief Records a seven-motor mode after register readback confirmation.
 * @param controller Initialized disabled controller.
 * @param control_mode Confirmed POS_VEL or MIT mode.
 * @param timestamp_us Transition timestamp.
 * @return OK or an argument/state error.
 */
ArmTransitionStatus arm_controller_confirm_control_mode(
    ArmController *controller,
    ArmControlMode control_mode,
    uint64_t timestamp_us);

/**
 * @brief Enters ENABLING after all software safety gates pass.
 * @param controller Initialized aligned disabled controller.
 * @param timestamp_us Transition timestamp.
 * @return OK or an argument/state error.
 */
ArmTransitionStatus arm_controller_begin_enable(ArmController *controller,
                                                uint64_t timestamp_us);

/**
 * @brief Enters READY only after all seven drivers report enabled.
 * @param controller Controller in ENABLING.
 * @param timestamp_us Confirmation timestamp.
 * @return OK or an argument/state error.
 */
ArmTransitionStatus arm_controller_confirm_enabled(ArmController *controller,
                                                   uint64_t timestamp_us);

/**
 * @brief Enters MOVING after a complete seven-axis plan has been validated.
 * @param controller Enabled controller in READY.
 * @param timestamp_us Motion start timestamp.
 * @return OK or an argument/state error.
 */
ArmTransitionStatus arm_controller_begin_motion(ArmController *controller,
                                                uint64_t timestamp_us);

/**
 * @brief Enters STOPPING while retaining logical motor enable.
 * @param controller Controller in READY, MOVING, or STOPPING.
 * @param timestamp_us Controlled-stop start timestamp.
 * @return OK or an argument/state error.
 */
ArmTransitionStatus arm_controller_begin_controlled_stop(
    ArmController *controller,
    uint64_t timestamp_us);

/**
 * @brief Returns a completed motion or controlled stop to enabled READY.
 * @param controller Controller in MOVING or STOPPING.
 * @param timestamp_us Completion timestamp.
 * @return OK or an argument/state error.
 */
ArmTransitionStatus arm_controller_complete_motion(ArmController *controller,
                                                   uint64_t timestamp_us);

/**
 * @brief Records confirmed all-axis disable while retaining RAM alignment.
 * @param controller Initialized controller.
 * @param timestamp_us Confirmation timestamp.
 * @return OK or INVALID_ARGUMENT.
 */
ArmTransitionStatus arm_controller_confirm_disabled(ArmController *controller,
                                                    uint64_t timestamp_us);

/**
 * @brief Clears a latched runtime fault after its external source is gone.
 * @param controller Initialized disabled controller.
 * @param timestamp_us Recovery timestamp.
 * @return OK or an argument/state error.
 */
ArmTransitionStatus arm_controller_clear_fault(ArmController *controller,
                                               uint64_t timestamp_us);

/**
 * @brief Copies the current controller state.
 * @param controller Initialized controller to inspect.
 * @param snapshot Output snapshot.
 * @return true when both pointers are valid and the controller is initialized.
 */
bool arm_controller_get_snapshot(const ArmController *controller,
                                 ArmSnapshot *snapshot);

#endif
