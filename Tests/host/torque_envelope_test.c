/** @file Synthetic ID7 overload exposure regression; never contacts hardware. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "DebugUi/debug_ui_torque_envelope.h"

/** @brief Feed a clocked series including repeated CAN samples between new observations. */
static TorqueEnvelopeReason advance(TorqueEnvelope *state, uint64_t first_us,
    uint64_t last_us, float torque_nm)
{
    uint64_t now_us;
    TorqueEnvelopeReason reason = TORQUE_ENVELOPE_OK;
    for (now_us = first_us; now_us <= last_us; now_us += 4000ULL) {
        reason = torque_envelope_observe(state, now_us, now_us, torque_nm);
        if (reason != TORQUE_ENVELOPE_OK) return reason;
    }
    return reason;
}

/** @brief Exercise limits, cumulative exposure, stale data, reset and clock failures. */
static void selected_limits(void)
{
    TorqueEnvelope state, original;
    unsigned index;
    static const uint32_t selections[] = { 3500U, 3501U, 5460U, 6239U, 6240U };
    torque_envelope_init(&state);
    original = state;
    assert(!torque_envelope_reset_cold_selected(&state, 3499U));
    assert(!memcmp(&original, &state, sizeof(state)));
    assert(!torque_envelope_reset_cold_selected(&state, 6241U));
    assert(!memcmp(&original, &state, sizeof(state)));
    for (index = 0U; index < sizeof(selections) / sizeof(selections[0]); ++index) {
        float selected_nm = (float)selections[index] / 1000.0F;
        assert(torque_envelope_reset_cold_selected(&state, selections[index]));
        assert(torque_envelope_observe(&state, 1000U, 1000U, selected_nm) == TORQUE_ENVELOPE_OK);
        assert(torque_envelope_observe(&state, 2000U, 2000U, -selected_nm) == TORQUE_ENVELOPE_OK);
        assert(torque_envelope_observe(&state, 3000U, 3000U, selected_nm + 0.001F) == TORQUE_ENVELOPE_PEAK);
        assert(torque_envelope_effective_limit(&state) == 3.5F);
        assert(torque_envelope_reset_cold_selected(&state, selections[index]));
        assert(torque_envelope_observe(&state, 1000U, 1000U, -selected_nm - 0.001F) == TORQUE_ENVELOPE_PEAK);
    }
    assert(torque_envelope_reset_cold_selected(&state, 6240U));
    assert(advance(&state, 1000U, 6581000U, 6.24F) == TORQUE_ENVELOPE_OK);
    assert(torque_envelope_observe(&state, 6585000U, 6585000U, 6.24F) == TORQUE_ENVELOPE_BUDGET);
    assert(!state.authorized);
    assert(torque_envelope_reset_cold_selected(&state, 6240U));
    assert(advance(&state, 1000U, 9997000U, 3.6F) == TORQUE_ENVELOPE_OK);
    assert(torque_envelope_observe(&state, 10001000U, 10001000U, 3.6F) == TORQUE_ENVELOPE_DURATION);
    puts("ADJUSTABLE_POLICY_PASS exact_signed_boundaries_6584ms_budget_10s_duration_invalid_atomic");
}

/** @brief Run fixed legacy and selected policy regressions with no hardware dependency. */
int main(void)
{
    TorqueEnvelope state;
    selected_limits();
    torque_envelope_init(&state);
    assert(!state.authorized);
    assert(torque_envelope_observe(&state, 1000U, 1000U, 3.5042734F) == TORQUE_ENVELOPE_NOMINAL);
    torque_envelope_reset_cold(&state);
    assert(state.authorized && state.exposure_nm2_s == 0.0F);
    assert(torque_envelope_observe(&state, 1000U, 1000U, 5.461F) == TORQUE_ENVELOPE_PEAK);
    assert(!state.authorized);
    torque_envelope_reset_cold(&state);
    assert(torque_envelope_observe(&state, 1000U, 1000U, -5.46F) == TORQUE_ENVELOPE_OK);
    assert(advance(&state, 5000U, 9997000U, -5.46F) == TORQUE_ENVELOPE_OK);
    assert(torque_envelope_observe(&state, 10001000U, 10001000U, -5.46F) != TORQUE_ENVELOPE_OK);
    assert(!state.authorized);
    torque_envelope_reset_cold(&state);
    assert(torque_envelope_observe(&state, 1000U, 1000U, 4.0F) == TORQUE_ENVELOPE_OK);
    assert(torque_envelope_observe(&state, 5000U, 1000U, 4.0F) == TORQUE_ENVELOPE_OK);
    assert(state.exposure_nm2_s > 0.014F);
    assert(torque_envelope_observe(&state, 25000U, 1000U, 4.0F) == TORQUE_ENVELOPE_FEEDBACK);
    assert(!state.authorized);
    torque_envelope_reset_cold(&state);
    assert(torque_envelope_observe(&state, 1000U, 1000U, 5.0F) == TORQUE_ENVELOPE_OK);
    assert(advance(&state, 5000U, 6001000U, 5.0F) == TORQUE_ENVELOPE_OK);
    torque_envelope_end(&state, 1U);
    assert(state.authorized && state.exposure_nm2_s > 76.0F);
    assert(torque_envelope_observe(&state, 9000000U, 9000000U, 5.0F) == TORQUE_ENVELOPE_OK);
    assert(advance(&state, 9004000U, 15000000U, 5.0F) == TORQUE_ENVELOPE_OK);
    torque_envelope_end(&state, 1U);
    assert(state.exposure_nm2_s > 152.0F);
    assert(torque_envelope_observe(&state, 20000000U, 20000000U, 5.0F) == TORQUE_ENVELOPE_OK);
    assert(advance(&state, 20004000U, 23000000U, 5.0F) == TORQUE_ENVELOPE_BUDGET);
    torque_envelope_reset_cold(&state);
    assert(torque_envelope_observe(&state, 5000U, 5000U, 1.0F) == TORQUE_ENVELOPE_OK);
    assert(torque_envelope_observe(&state, 4999U, 4999U, 1.0F) == TORQUE_ENVELOPE_CLOCK);
    torque_envelope_reset_cold(&state);
    assert(torque_envelope_observe(&state, 5000U, 5000U, 1.0F) == TORQUE_ENVELOPE_OK);
    assert(torque_envelope_observe(&state, 25001U, 25001U, 1.0F) == TORQUE_ENVELOPE_FEEDBACK);
    torque_envelope_reset_cold(&state);
    assert(torque_envelope_observe(&state, 5000U, 5001U, 1.0F) == TORQUE_ENVELOPE_FEEDBACK);
    torque_envelope_reset_cold(&state);
    assert(torque_envelope_observe(&state, 5000U, 5000U, NAN) == TORQUE_ENVELOPE_INVALID);
    torque_envelope_reset_cold(&state);
    assert(torque_envelope_observe(&state, 5000U, 5000U, 5.0F) == TORQUE_ENVELOPE_OK);
    torque_envelope_end(&state, 0U);
    assert(!state.authorized);
    torque_envelope_reset_cold(&state);
    assert(torque_envelope_observe(&state, 5000U, 5000U, 5.0F) == TORQUE_ENVELOPE_OK);
    torque_envelope_init(&state);
    assert(!state.authorized);
    assert(torque_envelope_observe(&state, 5000U, 5000U, 5.0F) == TORQUE_ENVELOPE_NOMINAL);
    puts("TORQUE_ENVELOPE_PASS peak_duration_repeated_samples_cumulative_pulses_clock_stale_boot_closed");
    return 0;
}
