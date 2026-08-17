/**
 * @file arm_controller.c
 * @brief Implements boot self-test and the safe entry into arm lifecycle states.
 */

#include "arm_controller.h"

#include <stddef.h>

#include "app_profile.h"

/**
 * @brief Transitions the controller to self-test and records the event.
 * @param controller Controller to update.
 * @param timestamp_us State-entry timestamp in microseconds.
 */
static void arm_controller_enter_self_test(ArmController *controller,
                                           uint64_t timestamp_us)
{
    controller->state = ARM_STATE_SELF_TEST;
    controller->state_entered_at_us = timestamp_us;
    (void)diagnostics_push(controller->diagnostics,
                           timestamp_us,
                           DIAGNOSTIC_CODE_SELF_TEST_STARTED,
                           DIAGNOSTIC_SEVERITY_INFO,
                           0U);
}

/**
 * @brief Latches a configuration fault and records structured diagnostics.
 * @param controller Controller to update.
 * @param fault Fault category to latch.
 * @param detail Validation bitmask associated with the fault.
 * @param timestamp_us Fault timestamp in microseconds.
 */
static void arm_controller_latch_fault(ArmController *controller,
                                       ArmFault fault,
                                       uint32_t detail,
                                       uint64_t timestamp_us)
{
    DiagnosticCode diagnostic_code = DIAGNOSTIC_CODE_CONFIG_INVALID;

    if (fault == ARM_FAULT_CONFIG_INCOMPLETE)
    {
        diagnostic_code = DIAGNOSTIC_CODE_CONFIG_INCOMPLETE;
    }

    controller->state = ARM_STATE_FAULT;
    controller->fault = fault;
    controller->fault_detail = detail;
    controller->state_entered_at_us = timestamp_us;
    diagnostics_record_config_validation_failure(controller->diagnostics);
    (void)diagnostics_push(controller->diagnostics,
                           timestamp_us,
                           diagnostic_code,
                           DIAGNOSTIC_SEVERITY_ERROR,
                           detail);
}

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
                         uint64_t timestamp_us)
{
    if (controller == NULL)
    {
        return;
    }

    controller->configuration = configuration;
    controller->diagnostics = diagnostics;
    controller->state = ARM_STATE_BOOT;
    controller->fault = ARM_FAULT_NONE;
    controller->control_mode = ARM_CONTROL_MODE_UNKNOWN;
    controller->state_entered_at_us = timestamp_us;
    controller->fault_detail = 0U;
    controller->aligned = 0U;
    controller->enabled = 0U;
    controller->moving = 0U;
    controller->initialized = 1U;

    (void)diagnostics_push(diagnostics,
                           timestamp_us,
                           DIAGNOSTIC_CODE_BOOT,
                           DIAGNOSTIC_SEVERITY_INFO,
                           0U);
}

/**
 * @brief Executes one non-blocking Phase 0 state-machine step.
 * @param controller Initialized controller to service.
 * @param timestamp_us Current monotonic time in microseconds.
 */
void arm_controller_step(ArmController *controller, uint64_t timestamp_us)
{
    ArmConfigValidation validation;

    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return;
    }

    diagnostics_record_service_cycle(controller->diagnostics);

    switch (controller->state)
    {
        case ARM_STATE_BOOT:
            arm_controller_enter_self_test(controller, timestamp_us);
            break;

        case ARM_STATE_SELF_TEST:
            if (!arm_config_validate_schema(controller->configuration, &validation))
            {
                arm_controller_latch_fault(controller,
                                           ARM_FAULT_CONFIG_INVALID,
                                           validation.schema_errors,
                                           timestamp_us);
            }
            else if (!arm_config_is_enable_ready(controller->configuration, &validation))
            {
#if (AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE)
                controller->state = ARM_STATE_UNALIGNED;
                controller->fault = ARM_FAULT_NONE;
                controller->fault_detail = validation.missing_verified_fields;
                controller->state_entered_at_us = timestamp_us;
#else
                arm_controller_latch_fault(controller,
                                           ARM_FAULT_CONFIG_INCOMPLETE,
                                           validation.missing_verified_fields,
                                           timestamp_us);
#endif
            }
            else
            {
                controller->state = ARM_STATE_UNALIGNED;
                controller->fault = ARM_FAULT_NONE;
                controller->fault_detail = 0U;
                controller->state_entered_at_us = timestamp_us;
            }
            break;

        case ARM_STATE_UNALIGNED:
        case ARM_STATE_DISABLED:
        case ARM_STATE_ENABLING:
        case ARM_STATE_READY:
        case ARM_STATE_MOVING:
        case ARM_STATE_STOPPING:
        case ARM_STATE_FAULT:
        default:
            break;
    }
}

/**
 * @brief Commits successful RAM reference alignment into the arm state machine.
 */
ArmTransitionStatus arm_controller_mark_reference_aligned(
    ArmController *controller,
    uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    if ((controller->state != ARM_STATE_UNALIGNED) &&
        !((controller->state == ARM_STATE_DISABLED) &&
          (controller->enabled == 0U) && (controller->moving == 0U)))
    {
        return ARM_TRANSITION_STATUS_INVALID_STATE;
    }

    controller->state = ARM_STATE_DISABLED;
    controller->state_entered_at_us = timestamp_us;
    controller->aligned = 1U;
    controller->enabled = 0U;
    controller->moving = 0U;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Latches communication loss and clears all logical enable/motion flags.
 */
ArmTransitionStatus arm_controller_force_stop_disable(
    ArmController *controller,
    uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    controller->state = ARM_STATE_FAULT;
    controller->fault = ARM_FAULT_LINK_TIMEOUT;
    controller->fault_detail = 0U;
    controller->state_entered_at_us = timestamp_us;
    controller->enabled = 0U;
    controller->moving = 0U;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Latches a non-configuration runtime fault and clears enable/motion flags.
 */
ArmTransitionStatus arm_controller_latch_runtime_fault(
    ArmController *controller,
    ArmFault fault,
    uint32_t detail,
    uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U) ||
        (fault == ARM_FAULT_NONE) || (fault == ARM_FAULT_CONFIG_INVALID) ||
        (fault == ARM_FAULT_CONFIG_INCOMPLETE))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    controller->state = ARM_STATE_FAULT;
    controller->fault = fault;
    controller->fault_detail = detail;
    controller->state_entered_at_us = timestamp_us;
    controller->enabled = 0U;
    controller->moving = 0U;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Records a seven-motor mode after register readback confirmation.
 */
ArmTransitionStatus arm_controller_confirm_control_mode(
    ArmController *controller,
    ArmControlMode control_mode,
    uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U) ||
        ((control_mode != ARM_CONTROL_MODE_POSITION_VELOCITY) &&
         (control_mode != ARM_CONTROL_MODE_MIT)))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    if ((controller->state != ARM_STATE_DISABLED) ||
        (controller->enabled != 0U) || (controller->moving != 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_STATE;
    }
    controller->control_mode = control_mode;
    controller->state_entered_at_us = timestamp_us;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Enters ENABLING after all software safety gates pass.
 */
ArmTransitionStatus arm_controller_begin_enable(ArmController *controller,
                                                uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    if ((controller->state != ARM_STATE_DISABLED) ||
        (controller->aligned == 0U) ||
        (controller->control_mode == ARM_CONTROL_MODE_UNKNOWN) ||
        (controller->fault != ARM_FAULT_NONE))
    {
        return ARM_TRANSITION_STATUS_INVALID_STATE;
    }
    controller->state = ARM_STATE_ENABLING;
    controller->state_entered_at_us = timestamp_us;
    controller->enabled = 0U;
    controller->moving = 0U;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Enters READY only after all seven drivers report enabled.
 */
ArmTransitionStatus arm_controller_confirm_enabled(ArmController *controller,
                                                   uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    if (controller->state != ARM_STATE_ENABLING)
    {
        return ARM_TRANSITION_STATUS_INVALID_STATE;
    }
    controller->state = ARM_STATE_READY;
    controller->state_entered_at_us = timestamp_us;
    controller->enabled = 1U;
    controller->moving = 0U;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Enters MOVING after a complete seven-axis plan has been validated.
 */
ArmTransitionStatus arm_controller_begin_motion(ArmController *controller,
                                                uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    if ((controller->state != ARM_STATE_READY) ||
        (controller->aligned == 0U) || (controller->enabled == 0U) ||
        (controller->moving != 0U) || (controller->fault != ARM_FAULT_NONE) ||
        (controller->control_mode == ARM_CONTROL_MODE_UNKNOWN))
    {
        return ARM_TRANSITION_STATUS_INVALID_STATE;
    }
    controller->state = ARM_STATE_MOVING;
    controller->state_entered_at_us = timestamp_us;
    controller->moving = 1U;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Enters STOPPING while retaining logical motor enable.
 */
ArmTransitionStatus arm_controller_begin_controlled_stop(
    ArmController *controller,
    uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    if (((controller->state != ARM_STATE_READY) &&
         (controller->state != ARM_STATE_MOVING) &&
         (controller->state != ARM_STATE_STOPPING)) ||
        (controller->enabled == 0U) || (controller->fault != ARM_FAULT_NONE))
    {
        return ARM_TRANSITION_STATUS_INVALID_STATE;
    }
    controller->state = ARM_STATE_STOPPING;
    controller->state_entered_at_us = timestamp_us;
    controller->moving = 1U;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Returns a completed motion or controlled stop to enabled READY.
 */
ArmTransitionStatus arm_controller_complete_motion(ArmController *controller,
                                                   uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    if (((controller->state != ARM_STATE_MOVING) &&
         (controller->state != ARM_STATE_STOPPING)) ||
        (controller->enabled == 0U) || (controller->fault != ARM_FAULT_NONE))
    {
        return ARM_TRANSITION_STATUS_INVALID_STATE;
    }
    controller->state = ARM_STATE_READY;
    controller->state_entered_at_us = timestamp_us;
    controller->moving = 0U;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Records confirmed all-axis disable while retaining RAM alignment.
 */
ArmTransitionStatus arm_controller_confirm_disabled(ArmController *controller,
                                                    uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    controller->state = (controller->aligned != 0U) ? ARM_STATE_DISABLED
                                                    : ARM_STATE_UNALIGNED;
    controller->state_entered_at_us = timestamp_us;
    controller->enabled = 0U;
    controller->moving = 0U;
    if (controller->fault == ARM_FAULT_LINK_TIMEOUT)
    {
        controller->fault = ARM_FAULT_NONE;
    }
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Clears a latched runtime fault after its external source is gone.
 */
ArmTransitionStatus arm_controller_clear_fault(ArmController *controller,
                                               uint64_t timestamp_us)
{
    if ((controller == NULL) || (controller->initialized == 0U))
    {
        return ARM_TRANSITION_STATUS_INVALID_ARGUMENT;
    }
    if ((controller->state != ARM_STATE_FAULT) ||
        (controller->enabled != 0U) || (controller->moving != 0U) ||
        (controller->fault == ARM_FAULT_CONFIG_INVALID) ||
        (controller->fault == ARM_FAULT_CONFIG_INCOMPLETE))
    {
        return ARM_TRANSITION_STATUS_INVALID_STATE;
    }
    controller->fault = ARM_FAULT_NONE;
    controller->fault_detail = 0U;
    controller->state = (controller->aligned != 0U) ? ARM_STATE_DISABLED
                                                    : ARM_STATE_UNALIGNED;
    controller->state_entered_at_us = timestamp_us;
    return ARM_TRANSITION_STATUS_OK;
}

/**
 * @brief Copies the current controller state.
 * @param controller Initialized controller to inspect.
 * @param snapshot Output snapshot.
 * @return true when both pointers are valid and the controller is initialized.
 */
bool arm_controller_get_snapshot(const ArmController *controller,
                                 ArmSnapshot *snapshot)
{
    if ((controller == NULL) || (snapshot == NULL) ||
        (controller->initialized == 0U))
    {
        return false;
    }

    snapshot->state = controller->state;
    snapshot->fault = controller->fault;
    snapshot->control_mode = controller->control_mode;
    snapshot->state_entered_at_us = controller->state_entered_at_us;
    snapshot->fault_detail = controller->fault_detail;
    snapshot->joint_count = (controller->configuration != NULL)
                                ? controller->configuration->joint_count
                                : 0U;
    snapshot->aligned = controller->aligned;
    snapshot->enabled = controller->enabled;
    snapshot->moving = controller->moving;
    return true;
}
