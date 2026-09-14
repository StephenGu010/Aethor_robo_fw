/** @file lcd_st7789.h
 * @brief Private SPI1/DMA1_Stream0 ST7789V2 platform API, without LVGL/RTOS.
 * Software implementation only: panel revision, timing, orientation and color
 * must still be verified on hardware. All task entrypoints belong to UiTask.
 * IRQ8: forward DMA1_Stream0_IRQHandler and SPI1_IRQHandler to entries below.
 * Do not generate another SPI1 handle/MSP or call MX_SPI1_Init for this owner.
 */
#ifndef LCD_ST7789_H
#define LCD_ST7789_H
#include "lcd_transfer.h"
#define LCD_ST7789_WIDTH LCD_TRANSFER_WIDTH
#define LCD_ST7789_HEIGHT LCD_TRANSFER_HEIGHT
#define LCD_ST7789_MAX_BYTES LCD_TRANSFER_MAX_BYTES
#define LCD_ST7789_DMA_START 0x24000000UL
#define LCD_ST7789_DMA_END   0x24008000UL

/** Panel lifecycle; FAULT requires explicit init/start recovery by UiTask. */
typedef enum { LCD_ST7789_OFF, LCD_ST7789_INITIALIZING,
    LCD_ST7789_READY, LCD_ST7789_FAULT } LcdSt7789State;
/** Consistent diagnostic copy; buffer_owned remains true during failed abort. */
typedef struct {
    LcdSt7789State state;
    LcdTransferState transfer_state;
    LcdTransferResult last_result;
    LcdTransferCounters transfer;
    uint32_t init_errors, spi_errors, last_hal_error, transaction;
    uint8_t buffer_owned, backlight_on;
} LcdSt7789Status;
/** Initialize private HAL resources from UiTask, with backlight off. No fatal handler.
 * Reject reinitialization while a buffer is owned. Hook may run synchronously or
 * from IRQ8 and may call lv_disp_flush_ready; no other LVGL calls or reentry.
 * HAL TIM23 tick must already run. Counters survive explicit recovery.
 */
int lcd_st7789_init(LcdTransferDone done, void *done_context);
/** Start nonblocking reset/sleep-out sequence at HAL_GetTick-compatible time. */
int lcd_st7789_start(uint32_t now_ms);
/** Call every <=5 ms, also in LVGL wait_cb; no recursive LVGL calls. */
void lcd_st7789_service(uint32_t now_ms);
/** Return 1 when accepted (hook then occurs exactly once, possibly before return).
 * Return 0 on rejection (no hook: caller must release its own LVGL flush).
 * Buffer: MSB-first RGB565 bytes, 32-byte aligned, wholly within reserved
 * [0x24000000,0x24008000); reserve entire padded cache lines in the scatter file.
 * LVGL v8 uses LV_COLOR_16_SWAP=1. Never modify/reuse until completion hook.
 */
int lcd_st7789_begin_flush(const LcdTransferWindow *window, const void *buffer,
                           uint32_t bytes, uint32_t now_ms);
/** Copy diagnostics using only local LCD IRQ masking. */
void lcd_st7789_get_status(LcdSt7789Status *status);
/** Request a bounded stop. Return 1 only after buffer ownership is released.
 * If 0, keep calling service; retain the buffer and keep UI motion locked.
 */
int lcd_st7789_abort(uint32_t now_ms);
/** Forward DMA1_Stream0 IRQ to the private handle. */
void lcd_st7789_dma_irq(void);
/** Forward SPI1 IRQ; validates EOT before the HAL clears its status. */
void lcd_st7789_spi_irq(void);
#endif
