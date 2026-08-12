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
    ARM_FAULT_CONFIG_INCOMPLETE
} ArmFault;

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
 * @brief Copies the current controller state.
 * @param controller Initialized controller to inspect.
 * @param snapshot Output snapshot.
 * @return true when both pointers are valid and the controller is initialized.
 */
bool arm_controller_get_snapshot(const ArmController *controller,
                                 ArmSnapshot *snapshot);

#endif
