/** @file pos_load_core_route_test.c
 * @brief Execute verbatim maintained Core frame-routing blocks against deterministic API boundaries.
 * Actual HAL/TX/SOF ordering is covered separately by the POS platform regression.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../App/Motor/can_tx_scheduler.h"
#include "../../App/Motor/motor_runtime.h"
#include "../../App/DebugUi/debug_ui_single_static_tx.h"

static uint8_t route_available, route_matches, route_phase, strict_accepts, generic_accepts;
static unsigned strict_calls, generic_calls, failed_receipts, queued_calls, critical_depth;
static uint8_t queued_strict, queued_accepted;
static unsigned event_sequence, failure_sequence, generic_sequence;

/** @brief Require routing and strict reservation to remain inside the existing task critical boundary. */
static void route_enter(void) { assert(!critical_depth); critical_depth = 1U; }
/** @brief Require paired task critical exit. */
static void route_exit(void) { assert(critical_depth == 1U); critical_depth = 0U; }
#define taskENTER_CRITICAL() route_enter()
#define taskEXIT_CRITICAL() route_exit()

/** @brief Supply only one selected emergency frame to the verbatim Core loop. */
static uint8_t aethor_app_pop_emergency_can_frame(CanFrame *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->identifier = 7U; frame->length = 8U;
    if (!route_available) return 0U;
    route_available = 0U;
    return 1U;
}
/** @brief Supply one ordinary-path frame with emergency priority, as terminal actions do. */
static MotorRuntimeStatus aethor_app_next_can_frame(uint64_t timestamp_us, CanFrame *frame, CanTxPriority *priority)
{
    (void)timestamp_us;
    *priority = CAN_TX_PRIORITY_EMERGENCY;
    return aethor_app_pop_emergency_can_frame(frame) ? MOTOR_RUNTIME_STATUS_FRAME_READY : MOTOR_RUNTIME_STATUS_WAITING;
}
/** @brief Publish only the explicitly selected fixture identity at the real matcher boundary. */
static uint8_t aethor_app_static_frame_identity(const CanFrame *frame, DebugUiStaticTxIdentity *identity)
{
    assert(critical_depth && frame->identifier == 7U);
    memset(identity, 0, sizeof(*identity));
    identity->epoch = 1U; identity->request_id = 2U; identity->sequence = 3U;
    identity->motor_id = 7U; identity->phase = route_phase;
    return route_matches;
}
/** @brief Limit emergency classification to the live POS terminal identity. */
static uint8_t aethor_app_pos_load_terminal_frame_identity(const CanFrame *frame, DebugUiStaticTxIdentity *identity)
{
    if (route_phase != DEBUG_UI_STATIC_TX_POS_DISABLE) return 0U;
    return aethor_app_static_frame_identity(frame, identity);
}
/** @brief Expose a fixed bounded fixture transport deadline without hardware. */
static uint64_t aethor_app_debug_ui_preflight_timeout_us(void) { return 500000ULL; }
/** @brief Preserve the production callback signature; actual callback behavior is tested in the facade suite. */
static uint8_t aethor_app_static_submitted(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us)
{ (void)identity; (void)boundary_us; return 1U; }
/** @brief Simulate reservation success/failure only; never manufacture a TX completion. */
static uint8_t stm32_platform_can_submit_static(const CanFrame *frame, const DebugUiStaticTxIdentity *identity,
    uint64_t timeout_us, uint8_t (*submitted)(const DebugUiStaticTxIdentity *, uint64_t))
{
    assert(critical_depth && frame->identifier == 7U && identity->motor_id == 7U);
    assert(timeout_us && submitted == aethor_app_static_submitted);
    ++strict_calls; ++event_sequence;
    return strict_accepts;
}
/** @brief Retain a physical emergency attempt when strict proof reservation fails. */
static CanTxSchedulerStatus stm32_platform_can_submit(CanTxPriority priority, const CanFrame *frame)
{
    assert(critical_depth && priority == CAN_TX_PRIORITY_EMERGENCY && frame->identifier == 7U);
    ++generic_calls; generic_sequence = ++event_sequence;
    return generic_accepts ? CAN_TX_SCHEDULER_STATUS_OK : CAN_TX_SCHEDULER_STATUS_FULL;
}
/** @brief Record the explicit failed proof before any unproven physical fallback is attempted. */
static void aethor_app_static_transmitted(const DebugUiStaticTxIdentity *identity, uint8_t succeeded,
    uint16_t sof, uint64_t observed_us)
{
    assert(critical_depth && identity->motor_id == 7U && !succeeded && !sof && observed_us);
    ++failed_receipts; failure_sequence = ++event_sequence;
}
/** @brief Keep queue acceptance separate from actual transmission evidence. */
static void aethor_app_static_can_queued(const CanFrame *frame, uint8_t accepted, uint8_t strict, uint64_t timestamp_us)
{
    assert(!critical_depth && frame->identifier == 7U && timestamp_us);
    ++queued_calls; queued_accepted = accepted; queued_strict = strict;
}
/** @brief Supply a monotonic synthetic timestamp to the exact Core blocks. */
static uint64_t AethorMonotonicTimestampUs(void) { return 1000U; }

#include "core_pos_route.inc"

/** @brief Reset all fixture boundaries before exercising a real routing branch. */
static void route_fixture(uint8_t phase, uint8_t strict_success, uint8_t generic_success)
{
    route_available = route_matches = 1U; route_phase = phase;
    strict_accepts = strict_success; generic_accepts = generic_success;
    strict_calls = generic_calls = failed_receipts = queued_calls = critical_depth = 0U;
    queued_strict = queued_accepted = 0U;
    event_sequence = failure_sequence = generic_sequence = 0U;
}

/** @brief Prove both Core paths preserve strict terminal proof and an unconfirmed emergency fallback. */
int main(void)
{
    unsigned ordinary;
    for (ordinary = 0U; ordinary < 2U; ++ordinary) {
        route_fixture(DEBUG_UI_STATIC_TX_POS_DISABLE, 1U, 1U);
        if (ordinary) route_ordinary(); else route_emergency();
        assert(strict_calls == 1U && !generic_calls && !failed_receipts);
        assert(queued_calls == 1U && queued_accepted && queued_strict && !critical_depth);
        route_fixture(DEBUG_UI_STATIC_TX_POS_DISABLE, 0U, 1U);
        if (ordinary) route_ordinary(); else route_emergency();
        assert(strict_calls == 1U && generic_calls == 1U && failed_receipts == 1U);
        assert(failure_sequence < generic_sequence && queued_accepted && !queued_strict);
        route_fixture(DEBUG_UI_STATIC_TX_POS_DISABLE, 0U, 0U);
        if (ordinary) route_ordinary(); else route_emergency();
        assert(failed_receipts == 1U && !queued_accepted && !queued_strict);
        route_fixture(DEBUG_UI_STATIC_TX_ENABLE, 1U, 1U);
        if (ordinary) { route_ordinary(); assert(strict_calls == 1U && !generic_calls); }
        else { route_emergency(); assert(!strict_calls && generic_calls == 1U); }
    }
    puts("POS_LOAD_CORE_ROUTING_PASS verbatim_core=1 paths=2 strict_terminal=1 fallback_unconfirmed=1 legacy_preserved=1");
    return 0;
}
