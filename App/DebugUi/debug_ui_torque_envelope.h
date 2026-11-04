/** @file ID7 experimental output-shaft torque exposure limits; not a thermal model or current clamp. */
#ifndef DEBUG_UI_TORQUE_ENVELOPE_H
#define DEBUG_UI_TORQUE_ENVELOPE_H
#include <math.h>
#include <stdint.h>
#include <string.h>

#define TORQUE_ENVELOPE_CONTINUOUS_NM (3.5F)
#define TORQUE_ENVELOPE_PEAK_NM (5.46F)
#define TORQUE_ENVELOPE_MIN_MNM (3500U)
#define TORQUE_ENVELOPE_MAX_MNM (6240U)
#define TORQUE_ENVELOPE_LEGACY_MNM (5460U)
#define TORQUE_ENVELOPE_MAX_EXCESS_US (10000000ULL)
#define TORQUE_ENVELOPE_MAX_GAP_US (20000ULL)
#define TORQUE_ENVELOPE_CAPACITY_NM2_S (175.616)

/** @brief Separate software exit evidence; no value claims manufacturer overload qualification. */
typedef enum {
    TORQUE_ENVELOPE_OK = 0,
    TORQUE_ENVELOPE_NOMINAL,
    TORQUE_ENVELOPE_PEAK,
    TORQUE_ENVELOPE_DURATION,
    TORQUE_ENVELOPE_BUDGET,
    TORQUE_ENVELOPE_FEEDBACK,
    TORQUE_ENVELOPE_CLOCK,
    TORQUE_ENVELOPE_INVALID
} TorqueEnvelopeReason;

/** @brief Session-wide non-recharging exposure ledger, explicitly locked after MCU initialization. */
typedef struct {
    double exposure_nm2_s;
    uint64_t last_tick_us, last_feedback_us, excess_started_us;
    float last_abs_nm;
    uint32_t peak_mnm;
    TorqueEnvelopeReason last_reason;
    uint8_t authorized, tracking, excessive;
} TorqueEnvelope;

/** @brief Revoke all experimental headroom; nominal protection remains available. */
static inline TorqueEnvelopeReason torque_envelope_revoke(TorqueEnvelope *state, TorqueEnvelopeReason reason)
{
    state->authorized = 0U;
    state->exposure_nm2_s = TORQUE_ENVELOPE_CAPACITY_NM2_S;
    state->last_reason = reason;
    return reason;
}

/** @brief Every MCU/application initialization starts without experimental exposure credit. */
static inline void torque_envelope_init(TorqueEnvelope *state)
{
    memset(state, 0, sizeof(*state));
    state->exposure_nm2_s = TORQUE_ENVELOPE_CAPACITY_NM2_S;
    state->peak_mnm = TORQUE_ENVELOPE_LEGACY_MNM;
}

/** @brief Atomically install a validated integer exit line and cold credit after external maintenance gates pass. */
static inline uint8_t torque_envelope_reset_cold_selected(TorqueEnvelope *state, uint32_t peak_mnm)
{
    if (peak_mnm < TORQUE_ENVELOPE_MIN_MNM || peak_mnm > TORQUE_ENVELOPE_MAX_MNM) return 0U;
    memset(state, 0, sizeof(*state));
    state->peak_mnm = peak_mnm;
    state->authorized = 1U;
    return 1U;
}

/** @brief Preserve the original fixed 5.46 Nm cold-reset API for legacy callers. */
static inline void torque_envelope_reset_cold(TorqueEnvelope *state)
{
    (void)torque_envelope_reset_cold_selected(state, TORQUE_ENVELOPE_LEGACY_MNM);
}

/** @brief Return the actual current protection boundary; revoked authorization always uses the rated baseline. */
static inline float torque_envelope_effective_limit(const TorqueEnvelope *state)
{
    return state->authorized ? (float)state->peak_mnm / 1000.0F : TORQUE_ENVELOPE_CONTINUOUS_NM;
}

/** @brief End observation only on safe disable proof; actions and reconnects never recharge the ledger. */
static inline void torque_envelope_end(TorqueEnvelope *state, uint8_t safe_disabled)
{
    if (!safe_disabled) (void)torque_envelope_revoke(state, TORQUE_ENVELOPE_FEEDBACK);
    state->tracking = state->excessive = 0U;
    state->last_tick_us = state->last_feedback_us = 0U;
    state->last_abs_nm = 0.0F;
}

/** @brief Charge wall-clock exposure even for repeated samples; all invalid observations fail closed. */
static inline TorqueEnvelopeReason torque_envelope_observe(TorqueEnvelope *state,
    uint64_t now_us, uint64_t feedback_us, float torque_nm)
{
    float absolute_nm;
    uint64_t elapsed_us;
    double interval_nm;
    if (!isfinite(torque_nm)) return torque_envelope_revoke(state, TORQUE_ENVELOPE_INVALID);
    absolute_nm = fabsf(torque_nm);
    if (!state->authorized) return absolute_nm > TORQUE_ENVELOPE_CONTINUOUS_NM ?
        TORQUE_ENVELOPE_NOMINAL : TORQUE_ENVELOPE_OK;
    if (!feedback_us || feedback_us > now_us || now_us - feedback_us > TORQUE_ENVELOPE_MAX_GAP_US)
        return torque_envelope_revoke(state, TORQUE_ENVELOPE_FEEDBACK);
    if (state->tracking && (now_us < state->last_tick_us || feedback_us < state->last_feedback_us))
        return torque_envelope_revoke(state, TORQUE_ENVELOPE_CLOCK);
    if (state->tracking && now_us - state->last_tick_us > TORQUE_ENVELOPE_MAX_GAP_US)
        return torque_envelope_revoke(state, TORQUE_ENVELOPE_FEEDBACK);
    if (state->peak_mnm < TORQUE_ENVELOPE_MIN_MNM || state->peak_mnm > TORQUE_ENVELOPE_MAX_MNM)
        return torque_envelope_revoke(state, TORQUE_ENVELOPE_INVALID);
    if (absolute_nm > torque_envelope_effective_limit(state))
        return torque_envelope_revoke(state, TORQUE_ENVELOPE_PEAK);
    elapsed_us = state->tracking ? now_us - state->last_tick_us : now_us - feedback_us;
    interval_nm = state->tracking ? fmaxf(state->last_abs_nm, absolute_nm) : absolute_nm;
    if (interval_nm > TORQUE_ENVELOPE_CONTINUOUS_NM) {
        state->exposure_nm2_s += (interval_nm * interval_nm -
            (double)TORQUE_ENVELOPE_CONTINUOUS_NM * TORQUE_ENVELOPE_CONTINUOUS_NM) *
            ((double)elapsed_us / 1000000.0);
        if (!isfinite(state->exposure_nm2_s) || state->exposure_nm2_s >= TORQUE_ENVELOPE_CAPACITY_NM2_S)
            return torque_envelope_revoke(state, TORQUE_ENVELOPE_BUDGET);
    }
    if (state->excessive && now_us - state->excess_started_us >= TORQUE_ENVELOPE_MAX_EXCESS_US)
        return torque_envelope_revoke(state, TORQUE_ENVELOPE_DURATION);
    if (absolute_nm > TORQUE_ENVELOPE_CONTINUOUS_NM) {
        if (!state->excessive) state->excess_started_us = feedback_us;
        state->excessive = 1U;
        if (now_us - state->excess_started_us >= TORQUE_ENVELOPE_MAX_EXCESS_US)
            return torque_envelope_revoke(state, TORQUE_ENVELOPE_DURATION);
    } else state->excessive = 0U;
    state->last_tick_us = now_us;
    state->last_feedback_us = feedback_us;
    state->last_abs_nm = absolute_nm;
    state->tracking = 1U;
    state->last_reason = TORQUE_ENVELOPE_OK;
    return TORQUE_ENVELOPE_OK;
}
#endif
