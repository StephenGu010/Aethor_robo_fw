/** @file lcd_transfer.h
 * @brief HAL-free bounded LCD TX ownership protocol; caller serializes all entrypoints.
 * Hardware stop must disable DMA/SPI and drain old IRQs before returning true.
 */
#ifndef LCD_TRANSFER_H
#define LCD_TRANSFER_H
#include <stdint.h>
#include <stddef.h>
#define LCD_TRANSFER_WIDTH 280U
#define LCD_TRANSFER_HEIGHT 240U
#define LCD_TRANSFER_MAX_BYTES 13440U
#define LCD_TRANSFER_TIMEOUT_MS 50U

/** Inclusive logical pixel window, before the controller offset. */
typedef struct { uint16_t x1, y1, x2, y2; } LcdTransferWindow;
/** Buffer is owned in ACTIVE/STOPPING and released only in IDLE. */
typedef enum { LCD_TRANSFER_IDLE, LCD_TRANSFER_ACTIVE, LCD_TRANSFER_STOPPING } LcdTransferState;
/** Terminal reason; rejected requests never acquire ownership or call the hook. */
typedef enum { LCD_TRANSFER_OK, LCD_TRANSFER_ABORTED, LCD_TRANSFER_TIMEOUT,
    LCD_TRANSFER_IO_ERROR } LcdTransferResult;
/** Completion may run in ISR; do not reenter the driver or start another transfer. */
typedef void (*LcdTransferDone)(void *context, uint32_t token, LcdTransferResult result);
/** Hardware operations have bounded execution. start may fail after enabling DMA.
 * stop returns 1 only when no reader holds the buffer and stale events are drained.
 * release is invoked after stop confirms quiescence, to deassert CS.
 */
typedef struct {
    int (*start)(void *context, const LcdTransferWindow *window,
                 const uint8_t *buffer, uint32_t bytes, uint32_t token);
    int (*stop)(void *context);
    void (*release)(void *context);
} LcdTransferOps;
/** Monotonic wrapping counters; elapsed time is unsigned milliseconds. */
typedef struct {
    uint32_t accepted, completed, rejected, timeouts, errors, aborted, late_events;
    uint32_t stop_pending, last_duration_ms;
} LcdTransferCounters;
/** Single-owner context, initialized once; no allocator, HAL, or LVGL dependency. */
typedef struct {
    LcdTransferOps ops;
    void *hardware_context;
    LcdTransferDone done;
    void *done_context;
    LcdTransferState state;
    LcdTransferResult result;
    LcdTransferCounters counters;
    const uint8_t *buffer;
    uint32_t token, started_ms, bytes;
} LcdTransfer;
/** Initialize a fresh context only, never overwrite an owned context. */
void lcd_transfer_init(LcdTransfer *transfer, const LcdTransferOps *ops,
                       void *hardware_context, LcdTransferDone done, void *done_context);
/** Validate inclusive coordinates, exact byte count and 32-byte pointer alignment. */
int lcd_transfer_validate(const LcdTransferWindow *window, const uint8_t *buffer, uint32_t bytes);
/** Return 1 if ownership accepted; even synchronous start failure completes via hook. */
int lcd_transfer_begin(LcdTransfer *transfer, const LcdTransferWindow *window,
                       const uint8_t *buffer, uint32_t bytes, uint32_t now_ms);
/** Hardware verified EOT only (DMA TC alone is insufficient), with original token. */
void lcd_transfer_eot(LcdTransfer *transfer, uint32_t token, uint32_t now_ms);
/** Report a hardware error for this transaction. */
void lcd_transfer_error(LcdTransfer *transfer, uint32_t token, uint32_t now_ms);
/** Stop without releasing memory until quiescence is positively confirmed. */
void lcd_transfer_abort(LcdTransfer *transfer, uint32_t now_ms);
/** Check 50-ms deadline/retry one bounded quiescence step, safe over tick wrap. */
void lcd_transfer_service(LcdTransfer *transfer, uint32_t now_ms);
#endif
