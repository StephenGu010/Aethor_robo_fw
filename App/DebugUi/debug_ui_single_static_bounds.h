/** @file debug_ui_single_static_bounds.h
 * @brief Pure absolute-pose admission for single static HOLD; never changes driver zero or limits.
 */
#ifndef DEBUG_UI_SINGLE_STATIC_BOUNDS_H
#define DEBUG_UI_SINGLE_STATIC_BOUNDS_H
#include "debug_ui_contract.h"
#include "../Config/motor_compatibility.h"
#include <float.h>
/** @brief Intersect ID7 absolute flange +/-180 degrees with verified PMAX in the caller's coordinate. */
static inline DebugUiMotorProfile debug_ui_single_static_bounds(const DebugUiMotorProfile *profile,
    uint8_t motor_id, float verified_position_max, float native_per_unit)
{
    DebugUiMotorProfile bounds = *profile;
    if (motor_id == 7U && native_per_unit > 0.0f && native_per_unit <= FLT_MAX &&
        verified_position_max > 0.0f && verified_position_max <= FLT_MAX) {
        float limit = 3.14159265358979323846f * AETHOR_S3519_MOTOR7_FLANGE_TOTAL_RATIO / native_per_unit;
        if (verified_position_max < limit) limit = verified_position_max;
        bounds.mit_position_min_rad = -limit;
        bounds.mit_position_max_rad = limit;
    }
    return bounds;
}
/** @brief Validate a signed protocol request against the verified driver and independent 3.5 Nm ceiling. */
static inline uint8_t debug_ui_single_static_feedforward_valid(float request_nm, float verified_tmax)
{
    float magnitude = request_nm < 0.0f ? -request_nm : request_nm;
    return (uint8_t)(verified_tmax > 0.0f && verified_tmax <= FLT_MAX &&
        magnitude >= 0.0f && magnitude <= 3.5f && magnitude <= verified_tmax);
}
#endif
