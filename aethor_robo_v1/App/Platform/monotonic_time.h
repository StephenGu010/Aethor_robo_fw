/**
 * @file monotonic_time.h
 * @brief Extends a wrapping 32-bit millisecond tick into monotonic microseconds.
 */

#ifndef APP_PLATFORM_MONOTONIC_TIME_H
#define APP_PLATFORM_MONOTONIC_TIME_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Owns fixed-capacity state for one wrapping millisecond tick source.
 */
typedef struct
{
    uint32_t last_tick_ms;
    uint64_t elapsed_ms;
    uint8_t initialized;
} AethorMonotonicTimeState;

/**
 * @brief Extends one consecutive unsigned tick sample into monotonic microseconds.
 * @param state Persistent extender state owned by the caller.
 * @param current_tick_ms Current wrapping 32-bit millisecond tick.
 * @return Extended monotonic timestamp in microseconds, or zero for null state.
 */
static inline uint64_t aethor_monotonic_time_update(
    AethorMonotonicTimeState *state,
    uint32_t current_tick_ms)
{
    uint32_t elapsed_tick_ms;

    if (state == NULL)
    {
        return 0ULL;
    }
    if (state->initialized == 0U)
    {
        state->last_tick_ms = current_tick_ms;
        state->elapsed_ms = current_tick_ms;
        state->initialized = 1U;
        return state->elapsed_ms * 1000ULL;
    }

    elapsed_tick_ms = current_tick_ms - state->last_tick_ms;
    state->last_tick_ms = current_tick_ms;
    state->elapsed_ms += elapsed_tick_ms;
    return state->elapsed_ms * 1000ULL;
}

#endif
