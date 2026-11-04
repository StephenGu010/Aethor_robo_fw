/** @file debug_ui_single_static_gains.h
 * @brief Bounded ID7 test gains in output-shaft units; not a plant stability certificate.
 */
#ifndef DEBUG_UI_SINGLE_STATIC_GAINS_H
#define DEBUG_UI_SINGLE_STATIC_GAINS_H
#include <stdint.h>
#define DEBUG_UI_STATIC_KP_DEFAULT (120U)
#define DEBUG_UI_STATIC_KP_MAX (200U)
#define DEBUG_UI_STATIC_KD_CENTI_DEFAULT (30U)
/** @brief One immutable pair, separate from the legacy long-duration profile ABI. */
typedef struct { float kp; float kd; } DebugUiStaticGains;
/** @brief Reject non-finite and out-of-window values using ordered comparisons. */
static inline uint8_t debug_ui_static_gains_valid(float kp, float kd)
{
    return (uint8_t)(kp >= 80.0f && kp <= (float)DEBUG_UI_STATIC_KP_MAX && kd >= 0.20f && kd <= 0.40f);
}
#endif
