/** @file adrc_feedback_adapter_test_main.c
 * @brief Deterministic lower-bound-time estimator tests; no hardware or qualification claims.
 */
#include "adrc_feedback_adapter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
/** @brief Rebuilds a known stationary window for independent corruption scenarios. */
static void warm_stationary(AdrcFeedbackAdapter *adapter)
{
    unsigned sample;
    adrc_feedback_adapter_reset(adapter);
    for (sample = 0U; sample <= 8U; ++sample)
        (void)adrc_feedback_adapter_update(adapter, 1000ULL + sample * 4000ULL,
            1000ULL + sample * 4000ULL, 0.0F, 0.04884F, 0.000382F);
    assert(adapter->ready);
}
/** @brief Verifies warmup, causal differencing, duplicate rejection and safety boundaries. */
int main(void)
{
    AdrcFeedbackAdapter adapter = {0};
    unsigned sample;
    for (sample = 0; sample < 8; ++sample)
        assert(!adrc_feedback_adapter_update(&adapter, 1000ULL + sample * 4000ULL,
            1000ULL + sample * 4000ULL, sample * 0.0004F, 0.04884F, 0.000382F));
    assert(adrc_feedback_adapter_update(&adapter, 33000, 33000, 0.0032F, 0.04884F, 0.000382F));
    assert(fabsf(adapter.velocity_rad_s - 0.1F) < 0.00001F);
    assert(adrc_feedback_adapter_update(&adapter, 34000, 33000, 0.0032F, 0.04884F, 0.000382F));
    assert(!adrc_feedback_adapter_update(&adapter, 34000, 33000, 0.0033F, 0.04884F, 0.000382F));
    assert(!adrc_feedback_adapter_update(&adapter, 35000, 0, 0, 0, 0.000382F));
    assert(!adrc_feedback_adapter_update(&adapter, 35000, 36000, 0, 0, 0.000382F));
    for (sample = 0; sample <= 4; ++sample)
        (void)adrc_feedback_adapter_update(&adapter, 41000ULL + sample * 8000ULL,
            41000ULL + sample * 8000ULL, 0, 0, 0.000382F);
    assert(adapter.ready);
    assert(!adrc_feedback_adapter_update(&adapter, 81001, 81001, 0, 0, 0.000382F));
    assert(!adrc_feedback_adapter_update(&adapter, 82000, 80000, 0, 0, 0.000382F));
    assert(!adrc_feedback_adapter_update(&adapter, 86000, 86000, 1, 0, 0.000382F));
    assert(!adrc_feedback_adapter_update(&adapter, 90000, 90000, NAN, 0, 0.000382F));
    warm_stationary(&adapter);
    assert(!adrc_feedback_adapter_update(&adapter, 34000, 32000, 0, 0.04884F, 0.000382F));
    assert(adapter.count == 0U);
    warm_stationary(&adapter);
    assert(!adrc_feedback_adapter_update(&adapter, 34000, 33000, 0, -0.04884F, 0.000382F));
    warm_stationary(&adapter);
    assert(!adrc_feedback_adapter_update(&adapter, 37000, 37000, 0, 0.04884F, 0.000764F));
    warm_stationary(&adapter);
    assert(!adrc_feedback_adapter_update(&adapter, 37000, 37000, 0.2F, 0.04884F, 0.000382F));
    warm_stationary(&adapter);
    assert(!adrc_feedback_adapter_update(&adapter, 41001, 33000, 0, 0.04884F, 0.000382F));
    assert(!adrc_feedback_adapter_update(&adapter, 0, 0, 0, 0, 0.000382F));
    puts("ADRC feedback adapter: boundary checks passed");
    return 0;
}
