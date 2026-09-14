/** @file lv_port_disp.c
 * @brief Async SPI display bridge using two 280x24 RGB565 DMA buffers.
 * A completion hook is the only ISR LVGL call. Rejections own no DMA buffer;
 * accepted transfers release only after platform-verified quiescence/EOT.
 */
#include "lv_port_disp.h"
#include "debug_ui_view.h"
#include <stddef.h>

#if defined(__CC_ARM)
#define DEBUG_UI_DMA_MEMORY __attribute__((section(".lcd_dma"), zero_init, aligned(32)))
#else
#define DEBUG_UI_DMA_MEMORY __attribute__((section(".lcd_dma"), aligned(32)))
#endif
DEBUG_UI_DMA_MEMORY
static lv_color_t draw_buffer_first[280U * 24U];
DEBUG_UI_DMA_MEMORY
static lv_color_t draw_buffer_second[280U * 24U];
typedef char DebugUiDmaBudgetCheck[(sizeof(draw_buffer_first) + sizeof(draw_buffer_second) == 26880U) ? 1 : -1];
static lv_disp_draw_buf_t draw_buffers;
static lv_disp_drv_t display_driver;
static lv_disp_drv_t *volatile owned_driver;
static volatile uint8_t display_fault;
static uint8_t recovery_attempted;

/** @brief Called exactly once by the owner after it releases DMA; may run in ISR. */
static void transfer_done(void *context, uint32_t token, LcdTransferResult result)
{
    lv_disp_drv_t *driver = owned_driver;
    (void)context;
    (void)token;
    if (result != LCD_TRANSFER_OK) display_fault = 1U;
    owned_driver = NULL;
    if (driver != NULL) lv_disp_flush_ready(driver);
}

/** @brief Submit one inclusive LVGL dirty rectangle; do not copy or reuse pixels. */
static void display_flush(lv_disp_drv_t *driver, const lv_area_t *area, lv_color_t *pixels)
{
    LcdTransferWindow window;
    uint32_t bytes;
    if (driver == NULL || area == NULL || pixels == NULL) return;
    if (area->x1 < 0 || area->y1 < 0 || area->x2 >= 280 || area->y2 >= 240 ||
        area->x1 > area->x2 || area->y1 > area->y2 || display_fault || !debug_ui_view_healthy()) {
        display_fault = 1U;
        lv_disp_flush_ready(driver);
        return;
    }
    window.x1 = (uint16_t)area->x1; window.y1 = (uint16_t)area->y1;
    window.x2 = (uint16_t)area->x2; window.y2 = (uint16_t)area->y2;
    bytes = (uint32_t)lv_area_get_width(area) * (uint32_t)lv_area_get_height(area) * sizeof(lv_color_t);
    owned_driver = driver; /* Must precede start: completion may be synchronous. */
    if (!lcd_st7789_begin_flush(&window, pixels, bytes, DebugUiNowMs())) {
        owned_driver = NULL;
        display_fault = 1U;
        lv_disp_flush_ready(driver); /* Rejected: ownership was never acquired. */
    }
}

/** @brief One bounded wait step. No recursive timer_handler or widget changes. */
static void display_wait(lv_disp_drv_t *driver)
{
    (void)driver;
    lv_port_disp_service(DebugUiNowMs());
    DebugUiWaitService();
}

/** @brief Initialize the platform lifecycle and register persistent LVGL buffers. */
uint8_t lv_port_disp_init(void)
{
    if (!lcd_st7789_init(transfer_done, NULL) || !lcd_st7789_start(DebugUiNowMs())) {
        display_fault = 1U;
        return 0U;
    }
    lv_disp_draw_buf_init(&draw_buffers, draw_buffer_first, draw_buffer_second, 280U * 24U);
    lv_disp_drv_init(&display_driver);
    display_driver.hor_res = 280;
    display_driver.ver_res = 240;
    display_driver.draw_buf = &draw_buffers;
    display_driver.flush_cb = display_flush;
    display_driver.wait_cb = display_wait;
    display_driver.full_refresh = 0U;
    display_driver.direct_mode = 0U;
    if (lv_disp_drv_register(&display_driver) == NULL) { display_fault = 1U; return 0U; }
    return 1U;
}

/** @brief Advance bounded platform initialization, transfer and timeout handling. */
void lv_port_disp_service(uint32_t now_ms) { lcd_st7789_service(now_ms); }

/** @brief Require both local graphics health and an initialized physical transport. */
uint8_t lv_port_disp_healthy(void)
{
    LcdSt7789Status status;
    lcd_st7789_get_status(&status);
    return (uint8_t)(!display_fault && status.state == LCD_ST7789_READY && debug_ui_view_healthy());
}

/** @brief Delegate the coherent diagnostic copy to the platform owner. */
void lv_port_disp_status(LcdSt7789Status *status) { lcd_st7789_get_status(status); }

/** @brief Attempt one idle recovery only after platform and LVGL ownership return. */
uint8_t lv_port_disp_try_recover(uint32_t now_ms)
{
    LcdSt7789Status status;
    lcd_st7789_get_status(&status);
    if (recovery_attempted || status.buffer_owned || owned_driver != NULL ||
        status.state != LCD_ST7789_FAULT || !debug_ui_view_healthy()) return 0U;
    recovery_attempted = 1U;
    if (!lcd_st7789_init(transfer_done, NULL) || !lcd_st7789_start(now_ms)) return 0U;
    display_fault = 0U;
    return 1U;
}
