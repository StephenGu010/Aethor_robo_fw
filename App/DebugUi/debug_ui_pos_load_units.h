/** @file debug_ui_pos_load_units.h
 * @brief Pure POS-load display mapping; stored requests remain motor-output radians.
 * Only ID7 has a confirmed final-flange transmission. No driver setting is written here.
 */
#ifndef DEBUG_UI_POS_LOAD_UNITS_H
#define DEBUG_UI_POS_LOAD_UNITS_H
#include "debug_ui_contract.h"
#include <float.h>

/** @brief Accept only the explicitly published positive finite ID7 mapping. */
static inline uint8_t debug_ui_pos_load_flange(const DebugUiMotorView *motor)
{
    return (uint8_t)(motor && motor->motor_id == 7U &&
        motor->pos_load_output_per_flange > 0.0f && motor->pos_load_output_per_flange <= FLT_MAX);
}

/** @brief Return output radians per displayed radian without changing other axes' units. */
static inline float debug_ui_pos_load_scale(const DebugUiMotorView *motor)
{
    return debug_ui_pos_load_flange(motor) ? motor->pos_load_output_per_flange : 1.0f;
}

/** @brief Convert stored output angle or speed into the current POS display coordinate. */
static inline float debug_ui_pos_load_display(const DebugUiMotorView *motor, float output_value)
{
    return output_value / debug_ui_pos_load_scale(motor);
}

/** @brief Bound editable ID7 relative motion to displayed +/-180 degrees with inward float margin. */
static inline float debug_ui_pos_load_edit_limit(const DebugUiMotorView *motor)
{
    return debug_ui_pos_load_flange(motor) ?
        3.14159265358979323846f * debug_ui_pos_load_scale(motor) * (1.0f - 4.0f * FLT_EPSILON) :
        motor->profile.max_delta_rad;
}

/** @brief Build separate POS-load admission bounds from verified driver limits, never ordinary UI limits. */
static inline DebugUiMotorProfile debug_ui_pos_load_bounds(const DebugUiMotorView *motor)
{
    DebugUiMotorProfile bounds = debug_ui_profile_bounds(&motor->profile, 0U);
    if (debug_ui_pos_load_flange(motor) && motor->ranges_verified &&
        motor->position_max_rad > 0.0f && motor->position_max_rad <= FLT_MAX / 2.0f) {
        float span = 2.0f * motor->position_max_rad;
        float cap = debug_ui_pos_load_edit_limit(motor);
        bounds.position_min_rad = -motor->position_max_rad;
        bounds.position_max_rad = motor->position_max_rad;
        bounds.max_delta_rad = span < cap ? span : cap;
    }
    return bounds;
}
#endif
