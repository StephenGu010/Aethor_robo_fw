/**
 * @file dual_motor_controller.c
 * @brief Implements non-blocking startup, KEY1 debounce, one-turn motion, and safety for two S3519 motors.
 */

#include "dual_motor_controller.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define DUAL_MOTOR_BOOT_DELAY_MS 2000U
#define DUAL_MOTOR_CONFIGURATION_TIMEOUT_MS 100U
#define DUAL_MOTOR_CONFIGURATION_RETRY_LIMIT 3U
#define DUAL_MOTOR_TWO_PI_RAD 6.2831853071795864769F
#define DUAL_MOTOR_ARRIVAL_POSITION_RAD 0.0087266462599716479F
#define DUAL_MOTOR_ARRIVAL_VELOCITY_RAD_S 0.0174532925199432958F
#define DUAL_MOTOR_VELOCITY_FEEDBACK_MAX_CODE 4095.0F
#define DUAL_MOTOR_VELOCITY_QUANTIZATION_MARGIN_RAD_S 0.001F
#define DUAL_MOTOR_ARRIVAL_CYCLES 3U
#define DUAL_MOTOR_CONFIGURATION_ITEM_COUNT 10U
#define DUAL_MOTOR_CONTROL_MODE_VALUE 2U

/** @brief Immutable CAN and Master identifiers for the two controlled motors. */
typedef struct
{
    uint8_t motor_id;
    uint16_t master_id;
} DualMotorConfiguration;

/** @brief One startup request selected from the fixed configuration sequence. */
typedef struct
{
    uint8_t motor_index;
    DmMotorRegister register_address;
    uint8_t is_write;
} DualMotorConfigurationRequest;

static const DualMotorConfiguration dual_motor_configuration[DUAL_MOTOR_COUNT] = {
    {1U, 0x11U},
    {2U, 0x12U}
};

static const DualMotorConfigurationRequest dual_motor_configuration_requests[
    DUAL_MOTOR_CONFIGURATION_ITEM_COUNT] = {
    {0U, DM_MOTOR_REGISTER_CONTROL_MODE, 1U},
    {0U, DM_MOTOR_REGISTER_CONTROL_MODE, 0U},
    {0U, DM_MOTOR_REGISTER_POSITION_RANGE, 0U},
    {0U, DM_MOTOR_REGISTER_VELOCITY_RANGE, 0U},
    {0U, DM_MOTOR_REGISTER_TORQUE_RANGE, 0U},
    {1U, DM_MOTOR_REGISTER_CONTROL_MODE, 1U},
    {1U, DM_MOTOR_REGISTER_CONTROL_MODE, 0U},
    {1U, DM_MOTOR_REGISTER_POSITION_RANGE, 0U},
    {1U, DM_MOTOR_REGISTER_VELOCITY_RANGE, 0U},
    {1U, DM_MOTOR_REGISTER_TORQUE_RANGE, 0U}
};

static DualMotorControllerState dual_motor_state;
static DualMotorSendFunction dual_motor_send_function;
static uint32_t dual_motor_initialization_time_ms;
static uint32_t dual_motor_request_time_ms;
static uint32_t dual_motor_movement_start_time_ms;
static uint8_t dual_motor_configuration_index;
static uint8_t dual_motor_configuration_retry_count;
static uint8_t dual_motor_request_waiting;
static uint8_t dual_motor_disable_request_mask;
static uint8_t dual_motor_arrival_cycle_count[DUAL_MOTOR_COUNT];
static uint8_t dual_motor_key_candidate_pressed;
static uint8_t dual_motor_key_press_armed;
static uint32_t dual_motor_key_candidate_since_ms;

static uint8_t dual_motor_feedback_is_recent(uint32_t current_time_ms);

/**
 * @brief Calculates a stopped-speed threshold that includes 12-bit feedback zero quantization.
 * @param motor_index Zero-based motor index with a confirmed VMAX parameter.
 * @return Positive velocity threshold in radians per second.
 */
static float dual_motor_arrival_velocity_limit(uint8_t motor_index)
{
    float quantized_zero_velocity_rad_s;

    assert(motor_index < DUAL_MOTOR_COUNT);
    assert(isfinite(dual_motor_state.velocity_max_rad_s[motor_index]));
    assert(dual_motor_state.velocity_max_rad_s[motor_index] > 0.0F);

    quantized_zero_velocity_rad_s =
        (dual_motor_state.velocity_max_rad_s[motor_index] /
         DUAL_MOTOR_VELOCITY_FEEDBACK_MAX_CODE) +
        DUAL_MOTOR_VELOCITY_QUANTIZATION_MARGIN_RAD_S;
    return fmaxf(DUAL_MOTOR_ARRIVAL_VELOCITY_RAD_S,
                 quantized_zero_velocity_rad_s);
}

/**
 * @brief Sends one already validated CAN frame through the injected transport.
 * @param frame Frame selected for transmission.
 * @return Zero on success; otherwise non-zero.
 */
static int dual_motor_send_frame(const FdcanClassicFrame *frame)
{
    assert(frame != NULL);
    if ((dual_motor_send_function == NULL) || (dual_motor_send_function(frame) != 0))
    {
        return -1;
    }
    return 0;
}

/**
 * @brief Sends one mode command to a configured motor.
 * @param motor_index Zero-based motor index.
 * @param command S3519 special mode command.
 * @return Zero on success; otherwise non-zero.
 */
static int dual_motor_send_mode(uint8_t motor_index, DmMotorModeCommand command)
{
    FdcanClassicFrame frame;

    assert(motor_index < DUAL_MOTOR_COUNT);
    if (dm_motor_pack_mode_command(dual_motor_configuration[motor_index].motor_id,
                                   command,
                                   &frame) != DM_MOTOR_STATUS_OK)
    {
        return -1;
    }
    return dual_motor_send_frame(&frame);
}

/**
 * @brief Enters a latched fault and schedules best-effort disable commands.
 * @param fault_reason Explicit reason for the shutdown.
 * @param request_disable Non-zero when CAN is still usable for disable frames.
 */
static void dual_motor_latch_fault(DualMotorControllerFault fault_reason,
                                   uint8_t request_disable)
{
    if (dual_motor_state.stage == DUAL_MOTOR_STAGE_FAULT)
    {
        return;
    }

    dual_motor_state.stage = DUAL_MOTOR_STAGE_FAULT;
    dual_motor_state.fault_reason = fault_reason;
    if (request_disable != 0U)
    {
        dual_motor_disable_request_mask = DUAL_MOTOR_ALL_MASK;
    }
    dual_motor_state.enabled_mask = 0U;
}

/**
 * @brief Sends pending best-effort disable frames outside interrupt context.
 */
static void dual_motor_service_pending_disable(void)
{
    uint8_t motor_index;

    for (motor_index = 0U; motor_index < DUAL_MOTOR_COUNT; ++motor_index)
    {
        uint8_t motor_bit = (uint8_t)(1U << motor_index);

        if ((dual_motor_disable_request_mask & motor_bit) != 0U)
        {
            if (dual_motor_send_mode(motor_index, DM_MOTOR_MODE_COMMAND_DISABLE) == 0)
            {
                dual_motor_disable_request_mask &= (uint8_t)~motor_bit;
            }
        }
    }
}

/**
 * @brief Requests fresh disabled feedback from both motors while waiting for KEY1.
 * @return Zero when both disable frames were submitted; otherwise non-zero.
 */
static int dual_motor_refresh_disabled_feedback(void)
{
    uint8_t motor_index;

    for (motor_index = 0U; motor_index < DUAL_MOTOR_COUNT; ++motor_index)
    {
        if (dual_motor_send_mode(motor_index, DM_MOTOR_MODE_COMMAND_DISABLE) != 0)
        {
            dual_motor_latch_fault(DUAL_MOTOR_FAULT_CAN_TRANSMIT, 1U);
            return -1;
        }
    }
    return 0;
}

/**
 * @brief Sends the current startup configuration request.
 * @param current_time_ms Current monotonic time in milliseconds.
 * @return Zero when the request was transmitted; otherwise non-zero.
 */
static int dual_motor_send_configuration_request(uint32_t current_time_ms)
{
    const DualMotorConfigurationRequest *request =
        &dual_motor_configuration_requests[dual_motor_configuration_index];
    FdcanClassicFrame frame;
    DmMotorStatus protocol_status;

    if (request->is_write != 0U)
    {
        protocol_status = dm_motor_pack_parameter_write_u32(
            dual_motor_configuration[request->motor_index].motor_id,
            request->register_address,
            DUAL_MOTOR_CONTROL_MODE_VALUE,
            &frame);
    }
    else
    {
        protocol_status = dm_motor_pack_parameter_read(
            dual_motor_configuration[request->motor_index].motor_id,
            request->register_address,
            &frame);
    }
    if ((protocol_status != DM_MOTOR_STATUS_OK) || (dual_motor_send_frame(&frame) != 0))
    {
        dual_motor_latch_fault(DUAL_MOTOR_FAULT_CAN_TRANSMIT, 1U);
        return -1;
    }

    dual_motor_request_time_ms = current_time_ms;
    dual_motor_request_waiting = 1U;
    return 0;
}

/**
 * @brief Completes the current configuration request and advances the sequence.
 */
static void dual_motor_complete_configuration_request(void)
{
    dual_motor_request_waiting = 0U;
    dual_motor_configuration_retry_count = 0U;
    dual_motor_configuration_index++;
}

/**
 * @brief Services the sequential mode and parameter startup exchange.
 * @param current_time_ms Current monotonic time in milliseconds.
 */
static void dual_motor_service_configuration(uint32_t current_time_ms)
{
    if (dual_motor_configuration_index >= DUAL_MOTOR_CONFIGURATION_ITEM_COUNT)
    {
        dual_motor_state.stage = DUAL_MOTOR_STAGE_REQUESTING_FEEDBACK;
        dual_motor_disable_request_mask = DUAL_MOTOR_ALL_MASK;
        dual_motor_state.fresh_feedback_mask = 0U;
        dual_motor_request_time_ms = current_time_ms;
        return;
    }

    if (dual_motor_request_waiting == 0U)
    {
        (void)dual_motor_send_configuration_request(current_time_ms);
        return;
    }
    if ((current_time_ms - dual_motor_request_time_ms) <
        DUAL_MOTOR_CONFIGURATION_TIMEOUT_MS)
    {
        return;
    }

    dual_motor_request_waiting = 0U;
    dual_motor_configuration_retry_count++;
    if (dual_motor_configuration_retry_count > DUAL_MOTOR_CONFIGURATION_RETRY_LIMIT)
    {
        dual_motor_latch_fault(DUAL_MOTOR_FAULT_CONFIGURATION_TIMEOUT, 1U);
        return;
    }
    (void)dual_motor_send_configuration_request(current_time_ms);
}

/**
 * @brief Updates the debounced key and reports a new stable press edge.
 * @param current_time_ms Current monotonic time in milliseconds.
 * @param raw_key_pressed Raw active-high logical key state.
 * @return One for one newly debounced press edge; otherwise zero.
 */
static uint8_t dual_motor_update_key(uint32_t current_time_ms, uint8_t raw_key_pressed)
{
    uint8_t normalized_pressed = (raw_key_pressed != 0U) ? 1U : 0U;

    if (normalized_pressed != dual_motor_key_candidate_pressed)
    {
        dual_motor_key_candidate_pressed = normalized_pressed;
        dual_motor_key_candidate_since_ms = current_time_ms;
        return 0U;
    }
    if ((dual_motor_state.stable_key_pressed != normalized_pressed) &&
        ((current_time_ms - dual_motor_key_candidate_since_ms) >=
         DUAL_MOTOR_KEY_DEBOUNCE_MS))
    {
        dual_motor_state.stable_key_pressed = normalized_pressed;
        return normalized_pressed;
    }
    return 0U;
}

/**
 * @brief Checks whether both motors can complete one turn in the requested direction.
 * @param direction_multiplier Positive one for forward or negative one for reverse.
 * @return One only when both current positions and resulting targets are finite and in range.
 */
static uint8_t dual_motor_direction_fits(float direction_multiplier)
{
    uint8_t motor_index;

    assert((direction_multiplier == 1.0F) || (direction_multiplier == -1.0F));
    for (motor_index = 0U; motor_index < DUAL_MOTOR_COUNT; ++motor_index)
    {
        float current_position_rad = dual_motor_state.measured_position_rad[motor_index];
        float target_position_rad =
            current_position_rad + direction_multiplier * DUAL_MOTOR_TWO_PI_RAD;

        if (!isfinite(current_position_rad) || !isfinite(target_position_rad) ||
            !isfinite(dual_motor_state.position_max_rad[motor_index]) ||
            (fabsf(target_position_rad) > dual_motor_state.position_max_rad[motor_index]))
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief Accepts one armed press and sends both enable frames for one revolution.
 * @param current_time_ms Current monotonic time in milliseconds.
 */
static void dual_motor_accept_move(uint32_t current_time_ms)
{
    float direction_multiplier = (dual_motor_state.next_direction >= 0) ? 1.0F : -1.0F;
    uint8_t motor_index;

    if (dual_motor_direction_fits(direction_multiplier) == 0U)
    {
        direction_multiplier = -direction_multiplier;
        if (dual_motor_direction_fits(direction_multiplier) == 0U)
        {
            dual_motor_latch_fault(DUAL_MOTOR_FAULT_TARGET_RANGE, 1U);
            return;
        }
    }

    for (motor_index = 0U; motor_index < DUAL_MOTOR_COUNT; ++motor_index)
    {
        float current_position_rad = dual_motor_state.measured_position_rad[motor_index];
        float target_position_rad =
            current_position_rad + direction_multiplier * DUAL_MOTOR_TWO_PI_RAD;

        dual_motor_state.initial_position_rad[motor_index] = current_position_rad;
        dual_motor_state.target_position_rad[motor_index] = target_position_rad;
        dual_motor_arrival_cycle_count[motor_index] = 0U;
    }

    dual_motor_state.move_accepted = 1U;
    dual_motor_state.arrived_mask = 0U;
    dual_motor_state.enabled_mask = 0U;
    dual_motor_movement_start_time_ms = current_time_ms;
    for (motor_index = 0U; motor_index < DUAL_MOTOR_COUNT; ++motor_index)
    {
        if (dual_motor_send_mode(motor_index, DM_MOTOR_MODE_COMMAND_ENABLE) != 0)
        {
            dual_motor_latch_fault(DUAL_MOTOR_FAULT_CAN_TRANSMIT, 1U);
            return;
        }
    }
    dual_motor_state.accepted_move_count++;
    dual_motor_state.next_direction =
        (direction_multiplier > 0.0F) ? -1 : 1;
    dual_motor_state.stage = DUAL_MOTOR_STAGE_ENABLING;
}

/**
 * @brief Sends both current absolute targets in one controller service cycle.
 */
static void dual_motor_send_targets(void)
{
    uint8_t motor_index;

    for (motor_index = 0U; motor_index < DUAL_MOTOR_COUNT; ++motor_index)
    {
        FdcanClassicFrame frame;

        if ((dm_motor_pack_position_velocity(
                 dual_motor_configuration[motor_index].motor_id,
                 dual_motor_state.target_position_rad[motor_index],
                 DUAL_MOTOR_TARGET_VELOCITY_RAD_S,
                 &frame) != DM_MOTOR_STATUS_OK) ||
            (dual_motor_send_frame(&frame) != 0))
        {
            dual_motor_latch_fault(DUAL_MOTOR_FAULT_CAN_TRANSMIT, 1U);
            return;
        }
    }
}

/**
 * @brief Enforces recent feedback for both motors while enabled or holding.
 * @param current_time_ms Current monotonic time in milliseconds.
 * @return One when both feedback timestamps remain valid; otherwise zero.
 */
static uint8_t dual_motor_feedback_is_recent(uint32_t current_time_ms)
{
    uint8_t motor_index;

    for (motor_index = 0U; motor_index < DUAL_MOTOR_COUNT; ++motor_index)
    {
        if ((current_time_ms - dual_motor_state.last_feedback_time_ms[motor_index]) >
            DUAL_MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            dual_motor_latch_fault(DUAL_MOTOR_FAULT_FEEDBACK_TIMEOUT, 1U);
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief Initializes all static dual-motor controller state.
 * @param send_function Injected classic-CAN transport callback.
 * @param current_time_ms Current monotonic time in milliseconds.
 */
void dual_motor_controller_init(DualMotorSendFunction send_function,
                                uint32_t current_time_ms)
{
    assert(send_function != NULL);
    memset(&dual_motor_state, 0, sizeof(dual_motor_state));
    memset(dual_motor_arrival_cycle_count, 0, sizeof(dual_motor_arrival_cycle_count));
    dual_motor_send_function = send_function;
    dual_motor_initialization_time_ms = current_time_ms;
    dual_motor_request_time_ms = current_time_ms;
    dual_motor_movement_start_time_ms = current_time_ms;
    dual_motor_configuration_index = 0U;
    dual_motor_configuration_retry_count = 0U;
    dual_motor_request_waiting = 0U;
    dual_motor_disable_request_mask = 0U;
    dual_motor_key_candidate_pressed = 0U;
    dual_motor_key_press_armed = 0U;
    dual_motor_key_candidate_since_ms = current_time_ms;
    dual_motor_state.stage = DUAL_MOTOR_STAGE_BOOT_DELAY;
    dual_motor_state.fault_reason = DUAL_MOTOR_FAULT_NONE;
    dual_motor_state.next_direction = 1;
}

/**
 * @brief Advances startup, key debounce, motion, holding, and safety state.
 * @param current_time_ms Current monotonic time in milliseconds.
 * @param user_key_is_pressed Non-zero while the active-low hardware key is pressed.
 */
void dual_motor_controller_step(uint32_t current_time_ms,
                                uint8_t user_key_is_pressed)
{
    uint8_t press_edge = dual_motor_update_key(current_time_ms, user_key_is_pressed);

    dual_motor_service_pending_disable();
    if (dual_motor_state.stage == DUAL_MOTOR_STAGE_FAULT)
    {
        return;
    }

    if (dual_motor_state.stage == DUAL_MOTOR_STAGE_BOOT_DELAY)
    {
        if ((current_time_ms - dual_motor_initialization_time_ms) >=
            DUAL_MOTOR_BOOT_DELAY_MS)
        {
            dual_motor_state.stage = DUAL_MOTOR_STAGE_CONFIGURING;
            dual_motor_service_configuration(current_time_ms);
        }
        return;
    }
    if (dual_motor_state.stage == DUAL_MOTOR_STAGE_CONFIGURING)
    {
        dual_motor_service_configuration(current_time_ms);
        return;
    }
    if (dual_motor_state.stage == DUAL_MOTOR_STAGE_REQUESTING_FEEDBACK)
    {
        if ((dual_motor_disable_request_mask == 0U) &&
            (dual_motor_state.fresh_feedback_mask == DUAL_MOTOR_ALL_MASK))
        {
            dual_motor_state.stage = DUAL_MOTOR_STAGE_READY;
        }
        else if ((current_time_ms - dual_motor_request_time_ms) >
                 DUAL_MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            dual_motor_latch_fault(DUAL_MOTOR_FAULT_FEEDBACK_TIMEOUT, 1U);
        }
        return;
    }
    if (dual_motor_state.stage == DUAL_MOTOR_STAGE_READY)
    {
        if (dual_motor_feedback_is_recent(current_time_ms) == 0U)
        {
            return;
        }
        if (dual_motor_state.stable_key_pressed == 0U)
        {
            dual_motor_key_press_armed = 1U;
        }
        if ((press_edge != 0U) && (dual_motor_key_press_armed != 0U))
        {
            dual_motor_key_press_armed = 0U;
            dual_motor_accept_move(current_time_ms);
        }
        else
        {
            (void)dual_motor_refresh_disabled_feedback();
        }
        return;
    }
    if (dual_motor_state.stage == DUAL_MOTOR_STAGE_ENABLING)
    {
        if ((current_time_ms - dual_motor_movement_start_time_ms) >
            DUAL_MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            dual_motor_latch_fault(DUAL_MOTOR_FAULT_FEEDBACK_TIMEOUT, 1U);
            return;
        }
        if (dual_motor_state.enabled_mask == DUAL_MOTOR_ALL_MASK)
        {
            dual_motor_state.stage = DUAL_MOTOR_STAGE_MOVING;
            dual_motor_movement_start_time_ms = current_time_ms;
            dual_motor_send_targets();
        }
        return;
    }
    if ((dual_motor_state.stage == DUAL_MOTOR_STAGE_MOVING) ||
        (dual_motor_state.stage == DUAL_MOTOR_STAGE_HOLDING))
    {
        if (dual_motor_feedback_is_recent(current_time_ms) == 0U)
        {
            return;
        }
        if ((dual_motor_state.stage == DUAL_MOTOR_STAGE_HOLDING) &&
            (dual_motor_state.stable_key_pressed == 0U))
        {
            dual_motor_key_press_armed = 1U;
        }
        if ((dual_motor_state.stage == DUAL_MOTOR_STAGE_HOLDING) &&
            (press_edge != 0U) &&
            (dual_motor_key_press_armed != 0U))
        {
            dual_motor_key_press_armed = 0U;
            dual_motor_accept_move(current_time_ms);
            return;
        }
        if ((dual_motor_state.stage == DUAL_MOTOR_STAGE_MOVING) &&
            ((current_time_ms - dual_motor_movement_start_time_ms) >
             DUAL_MOTOR_MOVEMENT_TIMEOUT_MS))
        {
            dual_motor_latch_fault(DUAL_MOTOR_FAULT_MOVEMENT_TIMEOUT, 1U);
            return;
        }
        dual_motor_send_targets();
    }
}

/**
 * @brief Consumes one parameter or status frame accepted by the FDCAN filter.
 * @param frame Accepted classic CAN frame.
 * @param current_time_ms Current monotonic receive time in milliseconds.
 */
void dual_motor_controller_on_can_frame(const FdcanClassicFrame *frame,
                                        uint32_t current_time_ms)
{
    uint8_t motor_index;

    if ((frame == NULL) || (frame->length != 8U))
    {
        return;
    }
    for (motor_index = 0U; motor_index < DUAL_MOTOR_COUNT; ++motor_index)
    {
        if (frame->identifier == dual_motor_configuration[motor_index].master_id)
        {
            break;
        }
    }
    if (motor_index >= DUAL_MOTOR_COUNT)
    {
        return;
    }

    if (((frame->data[2] == 0x33U) || (frame->data[2] == 0x55U)) &&
        (dual_motor_state.stage == DUAL_MOTOR_STAGE_CONFIGURING) &&
        (dual_motor_request_waiting != 0U))
    {
        const DualMotorConfigurationRequest *request =
            &dual_motor_configuration_requests[dual_motor_configuration_index];
        DmMotorParameterResponse response;

        if ((request->motor_index != motor_index) ||
            (dm_motor_decode_parameter_response(frame, &response) != DM_MOTOR_STATUS_OK) ||
            (response.motor_id != dual_motor_configuration[motor_index].motor_id) ||
            (response.register_address != request->register_address))
        {
            return;
        }

        if (response.register_address == DM_MOTOR_REGISTER_CONTROL_MODE)
        {
            if (response.raw_value != DUAL_MOTOR_CONTROL_MODE_VALUE)
            {
                dual_motor_latch_fault(DUAL_MOTOR_FAULT_INVALID_PARAMETER, 1U);
                return;
            }
            if (request->is_write == 0U)
            {
                dual_motor_state.mode_ready_mask |= (uint8_t)(1U << motor_index);
            }
        }
        else
        {
            float value = response.float_value;

            if (!isfinite(value) || (value <= 0.0F))
            {
                dual_motor_latch_fault(DUAL_MOTOR_FAULT_INVALID_PARAMETER, 1U);
                return;
            }
            if (response.register_address == DM_MOTOR_REGISTER_POSITION_RANGE)
            {
                dual_motor_state.position_max_rad[motor_index] = value;
            }
            else if (response.register_address == DM_MOTOR_REGISTER_VELOCITY_RANGE)
            {
                dual_motor_state.velocity_max_rad_s[motor_index] = value;
            }
            else if (response.register_address == DM_MOTOR_REGISTER_TORQUE_RANGE)
            {
                dual_motor_state.torque_max_nm[motor_index] = value;
                dual_motor_state.ranges_ready_mask |= (uint8_t)(1U << motor_index);
            }
        }
        dual_motor_complete_configuration_request();
        return;
    }

    if ((frame->data[0] & 0x0FU) == dual_motor_configuration[motor_index].motor_id &&
        (dual_motor_state.ranges_ready_mask & (uint8_t)(1U << motor_index)) != 0U)
    {
        DmMotorRanges ranges = {
            dual_motor_state.position_max_rad[motor_index],
            dual_motor_state.velocity_max_rad_s[motor_index],
            dual_motor_state.torque_max_nm[motor_index]
        };
        DmMotorFeedback feedback;
        uint8_t motor_bit = (uint8_t)(1U << motor_index);

        if (dm_motor_decode_feedback(frame, &ranges, &feedback) != DM_MOTOR_STATUS_OK)
        {
            return;
        }
        dual_motor_state.motor_state[motor_index] = feedback.state;
        dual_motor_state.measured_position_rad[motor_index] = feedback.position_rad;
        dual_motor_state.measured_velocity_rad_s[motor_index] = feedback.velocity_rad_s;
        dual_motor_state.last_feedback_time_ms[motor_index] = current_time_ms;
        dual_motor_state.fresh_feedback_mask |= motor_bit;
        if (feedback.state >= 8U)
        {
            dual_motor_latch_fault(DUAL_MOTOR_FAULT_MOTOR, 1U);
            return;
        }
        if (feedback.state == 1U)
        {
            dual_motor_state.enabled_mask |= motor_bit;
        }
        else
        {
            dual_motor_state.enabled_mask &= (uint8_t)~motor_bit;
        }

        if ((dual_motor_state.stage == DUAL_MOTOR_STAGE_MOVING) ||
            (dual_motor_state.stage == DUAL_MOTOR_STAGE_HOLDING))
        {
            if ((fabsf(feedback.position_rad - dual_motor_state.target_position_rad[motor_index]) <=
                 DUAL_MOTOR_ARRIVAL_POSITION_RAD) &&
                (fabsf(feedback.velocity_rad_s) <=
                 dual_motor_arrival_velocity_limit(motor_index)))
            {
                if (dual_motor_arrival_cycle_count[motor_index] < DUAL_MOTOR_ARRIVAL_CYCLES)
                {
                    dual_motor_arrival_cycle_count[motor_index]++;
                }
                if (dual_motor_arrival_cycle_count[motor_index] >= DUAL_MOTOR_ARRIVAL_CYCLES)
                {
                    dual_motor_state.arrived_mask |= motor_bit;
                }
            }
            else
            {
                dual_motor_arrival_cycle_count[motor_index] = 0U;
                dual_motor_state.arrived_mask &= (uint8_t)~motor_bit;
            }
            if (dual_motor_state.arrived_mask == DUAL_MOTOR_ALL_MASK)
            {
                dual_motor_state.stage = DUAL_MOTOR_STAGE_HOLDING;
            }
        }
    }
}

/**
 * @brief Latches Bus-Off without attempting CAN recovery in interrupt context.
 */
void dual_motor_controller_on_bus_off(void)
{
    dual_motor_latch_fault(DUAL_MOTOR_FAULT_BUS_OFF, 0U);
}

/**
 * @brief Latches an FDCAN initialization failure while leaving USB diagnostics active.
 */
void dual_motor_controller_on_can_start_failure(void)
{
    dual_motor_latch_fault(DUAL_MOTOR_FAULT_CAN_TRANSMIT, 0U);
}

/**
 * @brief Returns the read-only public dual-motor controller state.
 * @return Address of the static state structure.
 */
const DualMotorControllerState *dual_motor_controller_get_state(void)
{
    return &dual_motor_state;
}
