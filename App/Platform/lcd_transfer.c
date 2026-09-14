/** @file lcd_transfer.c
 * @brief Bounded TX protocol; no wait loops, dynamic storage, HAL or LVGL.
 * The platform serializes service/begin against its local IRQs. All event tokens
 * identify the original transaction, never a newly read replacement transaction.
 */
#include "lcd_transfer.h"
#include <string.h>

/** Attempt one hardware stop step; transfer identity is retired before notification. */
static void try_finish(LcdTransfer *transfer, uint32_t now_ms)
{
    if (!transfer->ops.stop(transfer->hardware_context))
    {
        transfer->counters.stop_pending++;
        return;
    }
    transfer->ops.release(transfer->hardware_context);
    transfer->buffer = NULL;
    transfer->bytes = 0U;
    transfer->state = LCD_TRANSFER_IDLE;
    transfer->counters.last_duration_ms = now_ms - transfer->started_ms;
    transfer->counters.completed++;
    if (transfer->done != NULL)
    {
        transfer->done(transfer->done_context, transfer->token, transfer->result);
    }
}

/** Latch exactly one terminal reason, then start bounded hardware shutdown. */
static void request_finish(LcdTransfer *transfer, LcdTransferResult result, uint32_t now_ms)
{
    if (transfer->state != LCD_TRANSFER_ACTIVE) { return; }
    transfer->result = result;
    transfer->state = LCD_TRANSFER_STOPPING;
    if (result == LCD_TRANSFER_TIMEOUT) { transfer->counters.timeouts++; }
    else if (result == LCD_TRANSFER_IO_ERROR) { transfer->counters.errors++; }
    else if (result == LCD_TRANSFER_ABORTED) { transfer->counters.aborted++; }
    try_finish(transfer, now_ms);
}

/** Initialize fresh storage and copy operations; invalid operation tables reject begin. */
void lcd_transfer_init(LcdTransfer *transfer, const LcdTransferOps *ops,
                       void *hardware_context, LcdTransferDone done, void *done_context)
{
    if (transfer == NULL) { return; }
    memset(transfer, 0, sizeof(*transfer));
    if (ops != NULL) { transfer->ops = *ops; }
    transfer->hardware_context = hardware_context;
    transfer->done = done;
    transfer->done_context = done_context;
}

/** Validate before multiplication; maximum area is then bounded by the physical panel. */
int lcd_transfer_validate(const LcdTransferWindow *window, const uint8_t *buffer, uint32_t bytes)
{
    uint32_t expected;
    if (window == NULL || buffer == NULL || ((uintptr_t)buffer & 31U) != 0U ||
        bytes == 0U || bytes > LCD_TRANSFER_MAX_BYTES ||
        window->x1 > window->x2 || window->y1 > window->y2 ||
        window->x2 >= LCD_TRANSFER_WIDTH || window->y2 >= LCD_TRANSFER_HEIGHT)
    { return 0; }
    expected = ((uint32_t)window->x2 - window->x1 + 1U) *
               ((uint32_t)window->y2 - window->y1 + 1U) * 2U;
    return bytes == expected;
}

/** Acquire before start, including partially armed start failures; caller owes no hook. */
int lcd_transfer_begin(LcdTransfer *transfer, const LcdTransferWindow *window,
                       const uint8_t *buffer, uint32_t bytes, uint32_t now_ms)
{
    if (transfer == NULL) { return 0; }
    if (transfer->state != LCD_TRANSFER_IDLE || transfer->ops.start == NULL ||
        transfer->ops.stop == NULL || transfer->ops.release == NULL ||
        !lcd_transfer_validate(window, buffer, bytes))
    {
        transfer->counters.rejected++;
        return 0;
    }
    transfer->token++;
    if (transfer->token == 0U) { transfer->token = 1U; }
    transfer->started_ms = now_ms;
    transfer->buffer = buffer;
    transfer->bytes = bytes;
    transfer->result = LCD_TRANSFER_OK;
    transfer->state = LCD_TRANSFER_ACTIVE;
    transfer->counters.accepted++;
    if (!transfer->ops.start(transfer->hardware_context, window, buffer, bytes, transfer->token))
    { request_finish(transfer, LCD_TRANSFER_IO_ERROR, now_ms); }
    return 1;
}

/** EOT must match the active identity and deadline; duplicate/stale events do nothing. */
void lcd_transfer_eot(LcdTransfer *transfer, uint32_t token, uint32_t now_ms)
{
    if (transfer == NULL) { return; }
    if (transfer->state != LCD_TRANSFER_ACTIVE || token != transfer->token)
    { transfer->counters.late_events++; return; }
    request_finish(transfer, (now_ms - transfer->started_ms >= LCD_TRANSFER_TIMEOUT_MS) ?
                   LCD_TRANSFER_TIMEOUT : LCD_TRANSFER_OK, now_ms);
}

/** Errors are tied to the original identity; an abort reason cannot be overwritten. */
void lcd_transfer_error(LcdTransfer *transfer, uint32_t token, uint32_t now_ms)
{
    if (transfer == NULL) { return; }
    if (transfer->state != LCD_TRANSFER_ACTIVE || token != transfer->token)
    { transfer->counters.late_events++; return; }
    request_finish(transfer, LCD_TRANSFER_IO_ERROR, now_ms);
}

/** Explicit stop retires only an active transfer; repeated requests are idempotent. */
void lcd_transfer_abort(LcdTransfer *transfer, uint32_t now_ms)
{
    if (transfer != NULL) { request_finish(transfer, LCD_TRANSFER_ABORTED, now_ms); }
}

/** Perform at most one stop attempt; a stuck peripheral never frees its buffer. */
void lcd_transfer_service(LcdTransfer *transfer, uint32_t now_ms)
{
    if (transfer == NULL) { return; }
    if (transfer->state == LCD_TRANSFER_STOPPING)
    {
        if (transfer->result == LCD_TRANSFER_OK &&
            now_ms - transfer->started_ms >= LCD_TRANSFER_TIMEOUT_MS)
        { transfer->result = LCD_TRANSFER_TIMEOUT; transfer->counters.timeouts++; }
        try_finish(transfer, now_ms);
    }
    else if (transfer->state == LCD_TRANSFER_ACTIVE &&
             now_ms - transfer->started_ms >= LCD_TRANSFER_TIMEOUT_MS)
    { request_finish(transfer, LCD_TRANSFER_TIMEOUT, now_ms); }
}
