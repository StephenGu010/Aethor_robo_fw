/** @file adrc_feedback_adapter.h
 * @brief Fixed-memory causal position velocity adapter; timestamps are receive-time lower bounds.
 */
#ifndef APP_ADRC_FEEDBACK_ADAPTER_H
#define APP_ADRC_FEEDBACK_ADAPTER_H
#include <stdint.h>
#define ADRC_FEEDBACK_WINDOW_US (32000ULL)
#define ADRC_FEEDBACK_HISTORY_CAPACITY (33U)
/** @brief Caller-owned history; readiness is evidence of a window, not timing calibration. */
typedef struct
{
    uint64_t timestamps[ADRC_FEEDBACK_HISTORY_CAPACITY];
    float positions[ADRC_FEEDBACK_HISTORY_CAPACITY];
    float velocity_rad_s;
    float last_raw_velocity_rad_s;
    float position_quantum_rad;
    uint8_t count;
    uint8_t next;
    uint8_t ready;
} AdrcFeedbackAdapter;
/** @brief Revokes readiness and all history after coordinate or sample integrity changes. */
void adrc_feedback_adapter_reset(AdrcFeedbackAdapter *adapter);
/** @brief Updates once per distinct valid timestamp; rejects gaps >8ms and implausible position jumps.
 * @note Returned velocity uses a >=32ms causal difference. Lower-bound timing uncertainty requires
 *       independent measured qualification; this function does not assert exact bus sample times.
 */
uint8_t adrc_feedback_adapter_update(AdrcFeedbackAdapter *adapter, uint64_t now_us,
    uint64_t feedback_us, float position_rad, float raw_velocity_rad_s, float position_quantum_rad);
#endif
