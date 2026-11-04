/** @file adrc_feedback_adapter.c
 * @brief Bounded causal differencing with conservative feedback-integrity rejection.
 */
#include "adrc_feedback_adapter.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
/** @brief Checks finite sample values without platform-specific classification macros. */
static int finite_value(float value)
{ return value == value && value >= -FLT_MAX && value <= FLT_MAX; }
/** @brief Clears fixed storage and any previously valid velocity. */
void adrc_feedback_adapter_reset(AdrcFeedbackAdapter *adapter)
{ if (adapter != NULL) { memset(adapter, 0, sizeof(*adapter)); } }
/** @brief Accepts coherent distinct samples and forms the nearest causal window of at least 32ms. */
uint8_t adrc_feedback_adapter_update(AdrcFeedbackAdapter *adapter, uint64_t now_us,
    uint64_t feedback_us, float position_rad, float raw_velocity_rad_s, float position_quantum_rad)
{
    unsigned index;
    unsigned offset;
    uint64_t interval_us;
    if (adapter == NULL) { return 0U; }
    if (feedback_us == 0ULL || feedback_us > now_us || now_us - feedback_us > 8000ULL ||
        !finite_value(position_rad) || !finite_value(raw_velocity_rad_s) ||
        !finite_value(position_quantum_rad) || position_quantum_rad <= 0.0F)
    { adrc_feedback_adapter_reset(adapter); return 0U; }
    if (adapter->count != 0U)
    {
        index = (adapter->next + ADRC_FEEDBACK_HISTORY_CAPACITY - 1U) % ADRC_FEEDBACK_HISTORY_CAPACITY;
        if (feedback_us < adapter->timestamps[index] || position_quantum_rad != adapter->position_quantum_rad)
        { adrc_feedback_adapter_reset(adapter); return 0U; }
        interval_us = feedback_us - adapter->timestamps[index];
        if (interval_us == 0ULL)
        {
            if (position_rad != adapter->positions[index] || raw_velocity_rad_s != adapter->last_raw_velocity_rad_s)
            { adrc_feedback_adapter_reset(adapter); }
            return adapter->ready;
        }
        if (interval_us > 8000ULL || fabsf(position_rad - adapter->positions[index]) >
            0.3F * ((float)interval_us * 0.000001F) + 2.0F * position_quantum_rad)
        { adrc_feedback_adapter_reset(adapter); return 0U; }
    }
    index = adapter->next;
    adapter->timestamps[index] = feedback_us;
    adapter->positions[index] = position_rad;
    adapter->last_raw_velocity_rad_s = raw_velocity_rad_s;
    adapter->position_quantum_rad = position_quantum_rad;
    adapter->next = (uint8_t)((index + 1U) % ADRC_FEEDBACK_HISTORY_CAPACITY);
    if (adapter->count < ADRC_FEEDBACK_HISTORY_CAPACITY) { ++adapter->count; }
    adapter->ready = 0U;
    for (offset = 1U; offset < adapter->count; ++offset)
    {
        unsigned historical = (index + ADRC_FEEDBACK_HISTORY_CAPACITY - offset) % ADRC_FEEDBACK_HISTORY_CAPACITY;
        interval_us = feedback_us - adapter->timestamps[historical];
        if (interval_us >= ADRC_FEEDBACK_WINDOW_US)
        {
            adapter->velocity_rad_s = (position_rad - adapter->positions[historical]) /
                ((float)interval_us * 0.000001F);
            adapter->ready = (uint8_t)finite_value(adapter->velocity_rad_s);
            break;
        }
    }
    return adapter->ready;
}
