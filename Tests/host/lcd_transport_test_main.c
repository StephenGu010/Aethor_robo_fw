/** @file lcd_transport_test_main.c
 * @brief Behavioral tests for LCD DMA ownership with fake clock and stop hardware.
 * No physical SPI, cache, display or ADC acceptance is implied by these tests.
 */
#include "lcd_transfer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

/** Fake hardware records observable ownership/CS, with independently delayed stop. */
typedef struct {
    int start_ok, stop_ready, reader_active, chip_selected;
    unsigned starts, stops, releases, completions;
    uint32_t token;
    LcdTransferResult result;
} FakeTransport;
static uint8_t pixel_buffer[LCD_TRANSFER_MAX_BYTES] __attribute__((aligned(32)));
static const LcdTransferWindow tile = {0U, 0U, 279U, 23U};

/** Simulate a start failure that can occur after the DMA reader has been armed. */
static int fake_start(void *context, const LcdTransferWindow *window,
                      const uint8_t *buffer, uint32_t bytes, uint32_t token)
{
    FakeTransport *fake = context;
    CHECK(window != NULL && buffer != NULL && bytes <= sizeof(pixel_buffer));
    fake->starts++;
    fake->reader_active = 1;
    fake->chip_selected = 1;
    fake->token = token;
    return fake->start_ok;
}
/** A stop request is not a stop acknowledgement; memory remains owned until ready. */
static int fake_stop(void *context)
{
    FakeTransport *fake = context;
    fake->stops++;
    if (!fake->stop_ready) { return 0; }
    fake->reader_active = 0;
    return 1;
}
/** Assert chip select cannot be released while a DMA reader is active. */
static void fake_release(void *context)
{
    FakeTransport *fake = context;
    CHECK(!fake->reader_active);
    fake->chip_selected = 0;
    fake->releases++;
}
/** Assert buffer ownership and CS were returned before reporting flush completion. */
static void fake_done(void *context, uint32_t token, LcdTransferResult result)
{
    FakeTransport *fake = context;
    CHECK(!fake->reader_active && !fake->chip_selected);
    CHECK(token == fake->token);
    fake->completions++;
    fake->result = result;
}
/** Initialize an independent fixture; one transaction at a time. */
static void fixture(LcdTransfer *transfer, FakeTransport *fake)
{
    const LcdTransferOps ops = { fake_start, fake_stop, fake_release };
    memset(fake, 0, sizeof(*fake));
    fake->start_ok = 1;
    fake->stop_ready = 1;
    lcd_transfer_init(transfer, &ops, fake, fake_done, fake);
}
/** Reject malformed regions/lengths/alignment before any DMA or callback. */
static void test_bounds(void)
{
    LcdTransfer transfer;
    FakeTransport fake;
    LcdTransferWindow bad = tile;
    fixture(&transfer, &fake);
    CHECK(lcd_transfer_validate(&tile, pixel_buffer, sizeof(pixel_buffer)));
    CHECK(!lcd_transfer_begin(&transfer, NULL, pixel_buffer, 2U, 0U));
    CHECK(!lcd_transfer_begin(&transfer, &tile, NULL, sizeof(pixel_buffer), 0U));
    CHECK(!lcd_transfer_begin(&transfer, &tile, pixel_buffer + 1, sizeof(pixel_buffer), 0U));
    CHECK(!lcd_transfer_begin(&transfer, &tile, pixel_buffer, 0U, 0U));
    CHECK(!lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer) - 2U, 0U));
    bad.x2 = 280U;
    CHECK(!lcd_transfer_begin(&transfer, &bad, pixel_buffer, sizeof(pixel_buffer), 0U));
    bad = tile; bad.y2 = 240U;
    CHECK(!lcd_transfer_begin(&transfer, &bad, pixel_buffer, sizeof(pixel_buffer), 0U));
    bad = tile; bad.x1 = 280U;
    CHECK(!lcd_transfer_begin(&transfer, &bad, pixel_buffer, sizeof(pixel_buffer), 0U));
    bad = tile; bad.y1 = 24U;
    CHECK(!lcd_transfer_begin(&transfer, &bad, pixel_buffer, sizeof(pixel_buffer), 0U));
    bad = tile; bad.y2 = 24U;
    CHECK(!lcd_transfer_begin(&transfer, &bad, pixel_buffer, 14000U, 0U));
    CHECK(fake.starts == 0U && fake.completions == 0U);
    bad.x1 = bad.x2 = 279U; bad.y1 = bad.y2 = 239U;
    CHECK(lcd_transfer_begin(&transfer, &bad, pixel_buffer, 2U, 1U));
    lcd_transfer_eot(&transfer, fake.token, 2U);
    CHECK(fake.completions == 1U);
}
/** Only true EOT releases the buffer, duplicate and previous-token EOT are ignored. */
static void test_eot_once(void)
{
    LcdTransfer transfer;
    FakeTransport fake;
    uint32_t old_token;
    fixture(&transfer, &fake);
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 100U));
    old_token = fake.token;
    CHECK(!lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 101U));
    lcd_transfer_service(&transfer, 120U);
    CHECK(fake.completions == 0U && fake.reader_active && fake.chip_selected);
    lcd_transfer_eot(&transfer, old_token, 122U);
    lcd_transfer_eot(&transfer, old_token, 123U);
    CHECK(fake.completions == 1U && fake.result == LCD_TRANSFER_OK);
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 124U));
    lcd_transfer_eot(&transfer, old_token, 125U);
    lcd_transfer_error(&transfer, old_token, 125U);
    CHECK(fake.completions == 1U && fake.reader_active);
    lcd_transfer_eot(&transfer, fake.token, 146U);
    CHECK(fake.completions == 2U && transfer.counters.completed == 2U);
}
/** Timeout at exactly 50 ms; delayed abort never permits early buffer reuse. */
static void test_timeout_and_late_irq(void)
{
    LcdTransfer transfer;
    FakeTransport fake;
    uint32_t old_token;
    fixture(&transfer, &fake);
    fake.stop_ready = 0;
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 10U));
    old_token = fake.token;
    lcd_transfer_service(&transfer, 59U);
    CHECK(fake.stops == 0U);
    lcd_transfer_service(&transfer, 60U);
    CHECK(transfer.state == LCD_TRANSFER_STOPPING && fake.completions == 0U);
    lcd_transfer_eot(&transfer, old_token, 61U);
    lcd_transfer_abort(&transfer, 62U);
    CHECK(fake.completions == 0U && fake.reader_active);
    CHECK(!lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 63U));
    lcd_transfer_service(&transfer, 1000U);
    CHECK(fake.completions == 0U && transfer.counters.timeouts == 1U);
    fake.stop_ready = 1;
    lcd_transfer_service(&transfer, 1001U);
    CHECK(fake.completions == 1U && fake.result == LCD_TRANSFER_TIMEOUT);
    CHECK(transfer.buffer == NULL && transfer.state == LCD_TRANSFER_IDLE);
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 1002U));
    lcd_transfer_eot(&transfer, old_token, 1003U);
    CHECK(fake.completions == 1U);
    lcd_transfer_eot(&transfer, fake.token, 1024U);
    CHECK(fake.completions == 2U);
}
/** Abort/failure release exactly once; failed DMA start is still an accepted flush. */
static void test_abort_and_start_error(void)
{
    LcdTransfer transfer;
    FakeTransport fake;
    fixture(&transfer, &fake);
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 0U));
    lcd_transfer_abort(&transfer, 1U);
    lcd_transfer_abort(&transfer, 2U);
    lcd_transfer_eot(&transfer, fake.token, 3U);
    CHECK(fake.completions == 1U && fake.result == LCD_TRANSFER_ABORTED);
    fake.start_ok = 0; fake.stop_ready = 0;
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 4U));
    CHECK(fake.completions == 1U && transfer.state == LCD_TRANSFER_STOPPING);
    fake.stop_ready = 1;
    lcd_transfer_service(&transfer, 5U);
    CHECK(fake.completions == 2U && fake.result == LCD_TRANSFER_IO_ERROR);
    fake.start_ok = 1;
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 6U));
    lcd_transfer_error(&transfer, fake.token, 7U);
    CHECK(fake.completions == 3U && transfer.counters.errors == 2U);
}
/** Unsigned tick subtraction is valid across wrap; stale EOT cannot defeat deadline. */
static void test_wrap_and_eot_deadline(void)
{
    LcdTransfer transfer;
    FakeTransport fake;
    fixture(&transfer, &fake);
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), UINT32_MAX - 20U));
    lcd_transfer_service(&transfer, 28U);
    CHECK(fake.completions == 0U);
    lcd_transfer_service(&transfer, 29U);
    CHECK(fake.result == LCD_TRANSFER_TIMEOUT && fake.completions == 1U);
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 30U));
    lcd_transfer_eot(&transfer, fake.token, 80U);
    CHECK(fake.result == LCD_TRANSFER_TIMEOUT && fake.completions == 2U);
}
/** An EOT does not hide a stuck DMA-disable acknowledgement beyond the same deadline. */
static void test_eot_stop_deadline(void)
{
    LcdTransfer transfer;
    FakeTransport fake;
    fixture(&transfer, &fake);
    fake.stop_ready = 0;
    CHECK(lcd_transfer_begin(&transfer, &tile, pixel_buffer, sizeof(pixel_buffer), 0U));
    lcd_transfer_eot(&transfer, fake.token, 20U);
    CHECK(fake.completions == 0U);
    lcd_transfer_service(&transfer, 50U);
    CHECK(transfer.counters.timeouts == 1U && fake.completions == 0U);
    fake.stop_ready = 1;
    lcd_transfer_service(&transfer, 51U);
    CHECK(fake.completions == 1U && fake.result == LCD_TRANSFER_TIMEOUT);
}
/** Run all transport behavior groups. */
int main(void)
{
    test_bounds(); test_eot_once(); test_timeout_and_late_irq();
    test_abort_and_start_error(); test_wrap_and_eot_deadline();
    test_eot_stop_deadline();
    puts("PASS: LCD transport (6 behavior groups, no hardware claims)");
    return 0;
}
