/** @file debug_ui_single_static_tx.h
 * @brief Isolated one-slot HAL FIFO and shared hardware SOF proof for preflight and frozen static startup.
 * RX SOF ordering proves bus order, not physical sample time or a causal command acknowledgement.
 */
#ifndef DEBUG_UI_SINGLE_STATIC_TX_H
#define DEBUG_UI_SINGLE_STATIC_TX_H
#include <stdint.h>
/** @brief Separate startup/preflight transactions; zero never identifies a strict request. */
typedef enum {
    DEBUG_UI_STATIC_TX_NONE = 0,
    DEBUG_UI_STATIC_TX_PREFLIGHT = 1,
    DEBUG_UI_STATIC_TX_PRELOAD = 2,
    DEBUG_UI_STATIC_TX_ENABLE = 3,
    DEBUG_UI_STATIC_TX_FIRST_HOLD = 4,
    DEBUG_UI_STATIC_TX_POS_DISABLE = 5,
    DEBUG_UI_STATIC_TX_TAKEOVER = 6
} DebugUiStaticTxPhase;
/** @brief Exact receipt identity includes phase, axis and a nonzero action-local sequence. */
typedef struct {
    uint32_t epoch;
    uint32_t request_id;
    uint32_t sequence;
    uint8_t motor_id;
    uint8_t phase;
} DebugUiStaticTxIdentity;
/** @brief Compare every identity component; receipts cannot be borrowed by another axis or phase. */
static inline uint8_t debug_ui_static_tx_identity_equal(const DebugUiStaticTxIdentity *left,
    const DebugUiStaticTxIdentity *right)
{
    return (uint8_t)(left && right && left->epoch && left->request_id && left->sequence && left->motor_id &&
        left->phase && left->epoch == right->epoch && left->request_id == right->request_id &&
        left->sequence == right->sequence && left->motor_id == right->motor_id && left->phase == right->phase);
}
/** @brief Pending-to-completed proof for one pinned FIFO buffer, separate from ADRC receipts. */
typedef struct {
    uint32_t buffer_mask;
    uint8_t armed;
    uint8_t pending_proven;
    uint8_t pending_seen;
} DebugUiSingleStaticTx;
/** @brief One 16-bit hardware SOF domain, anchored before this exact marked strict-frame submission. */
typedef struct {
    uint16_t anchor;
    uint16_t transmitted;
    uint8_t marker;
    uint8_t event_seen;
} DebugUiSingleStaticWireOrder;
/** @brief Prove the unchanged action window is strictly shorter than half a hardware timestamp wrap. */
static inline uint8_t debug_ui_single_static_wire_window(uint32_t kernel_hz,
    uint32_t nominal_bit_cycles, uint64_t action_timeout_us)
{
    return (uint8_t)(kernel_hz && nominal_bit_cycles && nominal_bit_cycles <= 35184372U && action_timeout_us &&
        action_timeout_us <= 0xFFFFFFFFULL &&
        (uint64_t)kernel_hz * action_timeout_us < 32768ULL * 16ULL * nominal_bit_cycles * 1000000ULL);
}
/** @brief Reject same-bin, pre-TX, old-wrap and ambiguous half-wrap RX timestamps. */
static inline uint8_t debug_ui_single_static_wire_after(const DebugUiSingleStaticWireOrder *order, uint16_t received)
{
    uint16_t transmit_delta, receive_delta;
    if (!order || !order->event_seen || !order->marker) return 0U;
    transmit_delta = (uint16_t)(order->transmitted - order->anchor);
    receive_delta = (uint16_t)(received - order->anchor);
    return (uint8_t)(transmit_delta < 32768U && receive_delta < 32768U && receive_delta > transmit_delta);
}
/** @brief Pin only one hardware buffer and a newly observed pending request with its completion bit clear. */
static inline uint8_t debug_ui_single_static_tx_arm(DebugUiSingleStaticTx *proof,
    uint32_t buffer_mask, uint32_t pending_mask, uint32_t transmitted_mask)
{
    if (!proof || proof->armed || !buffer_mask || (buffer_mask & (buffer_mask - 1U))) return 0U;
    proof->buffer_mask = buffer_mask;
    proof->armed = 1U;
    proof->pending_seen = (uint8_t)((pending_mask & buffer_mask) != 0U);
    proof->pending_proven = (uint8_t)((pending_mask & buffer_mask) && !(transmitted_mask & buffer_mask));
    return 1U;
}
/** @brief A terminal receipt requires observed new pending, then transmitted with no cancellation; old TXBTO fails. */
static inline uint8_t debug_ui_single_static_tx_collect(DebugUiSingleStaticTx *proof,
    uint32_t pending_mask, uint32_t transmitted_mask, uint32_t cancelled_mask, uint8_t *succeeded)
{
    if (!proof || !succeeded || !proof->armed) return 0U;
    if (pending_mask & proof->buffer_mask) {
        proof->pending_seen = 1U;
        if (!((transmitted_mask | cancelled_mask) & proof->buffer_mask)) proof->pending_proven = 1U;
        return 0U;
    }
    /* TXBAR can precede visible TXBRP. No observation of pending means no proof of clearance,
     * even when an old transmitted/cancelled bit is set. A separately matched fresh event is a platform concern. */
    if (!proof->pending_seen) return 0U;
    *succeeded = (uint8_t)(proof->pending_proven && (transmitted_mask & proof->buffer_mask) &&
        !(cancelled_mask & proof->buffer_mask));
    proof->armed = 0U;
    return 1U;
}
#endif
