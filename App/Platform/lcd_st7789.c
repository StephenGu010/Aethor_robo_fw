/** @file lcd_st7789.c
 * @brief H723 private SPI1/DMA1_Stream0 ST7789V2 transport, software-only validated.
 * Evidence: dm-mc02/2D图纸/LCD模块/ST7789V2_SPEC_V1.0.pdf pp44-45,56:
 * SDA sampled on rising SCL, CS falling permits either idle SCL level. Mode 0
 * (LOW/1EDGE) therefore samples correctly; vendor HIGH/1EDGE samples falling.
 * p226 COLMOD note2 specifies 0x55 for 16-bit pixel writes. Vendor lcd.c supplies
 * MADCTL=0x70, X+20,Y+0 and panel power/gamma values, still pending real-panel QA.
 * HAL SPI NORMAL DMA TC only enables EOT IRQ; SPI IRQ closes transfer before
 * HAL_SPI_TxCpltCallback. DMA TC alone never releases CS or the draw buffer.
 * Resources, clocks, pins and IRQ8 are owned here, not duplicated in CubeMX MSP.
 */
#include "debug_ui_config.h"
#if AETHOR_DEBUG_UI_ENABLE
#include "lcd_st7789.h"
#if defined(LCD_PLATFORM_HOST_TEST)
#include "lcd_fake_hal.h"
#else
#include "stm32h7xx_hal.h"
#endif
#include <string.h>

#define LCD_COMMAND_TIMEOUT_MS 2U
#define LCD_DMA_STREAM0_FLAGS 0x0000003DUL
#define LCD_SPI_CLEAR_FLAGS 0x00000FF8UL
#define LCD_SPI_ERRORS (SPI_FLAG_UDR | SPI_FLAG_OVR | SPI_FLAG_FRE | SPI_FLAG_MODF)

/** Single panel command (maximum gamma payload 14 bytes), immutable vendor values. */
typedef struct { uint8_t command, count, parameters[14]; } LcdInitCommand;
static const LcdInitCommand init_commands[] = {
    {0x36,1,{0x70}}, {0x3A,1,{0x55}}, {0xB2,5,{0x0C,0x0C,0x00,0x33,0x33}},
    {0xB7,1,{0x35}}, {0xBB,1,{0x32}}, {0xC2,1,{0x01}}, {0xC3,1,{0x15}},
    {0xC4,1,{0x20}}, {0xC6,1,{0x0F}}, {0xD0,2,{0xA4,0xA1}},
    {0xE0,14,{0xD0,0x08,0x0E,0x09,0x09,0x05,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34}},
    {0xE1,14,{0xD0,0x08,0x0E,0x09,0x09,0x15,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34}},
    {0x21,0,{0}}, {0x29,0,{0}}
};
static SPI_HandleTypeDef lcd_spi;
static DMA_HandleTypeDef lcd_dma;
static LcdTransfer lcd_tx;
static LcdSt7789Status lcd_status;
static LcdTransferDone completion_hook;
static void *completion_context;
static uint32_t stage_started_ms, irq_token;
static uint8_t resources_ready, transfer_initialized, init_stage, init_index, eot_authorized;
static uint8_t error_callback_authorized;

/** Mask only LCD IRQs; CAN, timebase, scheduler and all higher-priority IRQs keep running. */
static uint32_t lock_lcd(void)
{
    uint32_t enabled = NVIC_GetEnableIRQ(DMA1_Stream0_IRQn) |
                       (NVIC_GetEnableIRQ(SPI1_IRQn) << 1U);
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    __DSB(); __ISB();
    return enabled;
}

/** Restore only the enable bits held by the caller. */
static void unlock_lcd(uint32_t enabled)
{
    __DMB();
    if ((enabled & 1U) != 0U) { HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn); }
    if ((enabled & 2U) != 0U) { HAL_NVIC_EnableIRQ(SPI1_IRQn); }
}

/** Set backlight only after a successful pixel flush, or clear it on fault. */
static void set_backlight(uint8_t enabled)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    lcd_status.backlight_on = enabled;
}

/** Drain only Stream0/DMAMUX channel0 and SPI1, never reset the shared DMA1 controller. */
static void drain_events(void)
{
    DMA1->LIFCR = LCD_DMA_STREAM0_FLAGS;
    DMAMUX1_ChannelStatus->CFR = 1U;
    SPI1->IFCR = LCD_SPI_CLEAR_FLAGS;
    __DSB();
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
    __DSB();
}

/** One bounded stop step. DMA EN readback confirms outstanding bus access has stopped.
 * RM0468 Rev3 sections 15.3.19/55.4.14 require DMA stop before disabling SPI/requests.
 * If EN stays set, preserve SPE/TXDMAEN, CS and buffer ownership for later service.
 */
static int stop_hardware(void *context)
{
    (void)context;
    eot_authorized = 0U;
    SPI1->IER = 0U;
    CLEAR_BIT(DMA1_Stream0->CR, DMA_SxCR_TCIE | DMA_SxCR_HTIE |
              DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_EN);
    CLEAR_BIT(DMA1_Stream0->FCR, DMA_SxFCR_FEIE);
    __DSB();
    if ((DMA1_Stream0->CR & DMA_SxCR_EN) != 0U) { return 0; }
    __HAL_SPI_DISABLE(&lcd_spi);
    __DSB();
    if ((SPI1->CR1 & SPI_CR1_SPE) != 0U) { return 0; }
    CLEAR_BIT(SPI1->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
    drain_events();
    lcd_dma.XferCpltCallback = NULL;
    lcd_dma.XferHalfCpltCallback = NULL;
    lcd_dma.XferErrorCallback = NULL;
    lcd_dma.XferAbortCallback = NULL;
    lcd_dma.State = HAL_DMA_STATE_READY;
    lcd_dma.Lock = HAL_UNLOCKED;
    lcd_spi.State = HAL_SPI_STATE_READY;
    lcd_spi.Lock = HAL_UNLOCKED;
    lcd_spi.TxXferCount = 0U;
    lcd_spi.RxXferCount = 0U;
    return 1;
}

/** Deassert CS after positive stop acknowledgement or actual EOT. */
static void release_bus(void *context)
{
    (void)context;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
}

/** Send one short command at bounded task-context cost, keeping caller-owned CS low. */
static int send_command(uint8_t command, const uint8_t *parameters, uint16_t count)
{
    HAL_StatusTypeDef result;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(&lcd_spi, &command, 1U, LCD_COMMAND_TIMEOUT_MS);
    if (result == HAL_OK && count != 0U)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
        result = HAL_SPI_Transmit(&lcd_spi, (uint8_t *)parameters, count, LCD_COMMAND_TIMEOUT_MS);
    }
    if (result != HAL_OK)
    {
        lcd_status.spi_errors++;
        lcd_status.last_hal_error = lcd_spi.ErrorCode;
        return 0;
    }
    return 1;
}

/** Program inclusive window then start MSB-first byte DMA after a TX-only cache clean. */
static int start_pixels(void *context, const LcdTransferWindow *window,
                        const uint8_t *buffer, uint32_t bytes, uint32_t token)
{
    uint16_t start_column = (uint16_t)(window->x1 + 20U);
    uint16_t end_column = (uint16_t)(window->x2 + 20U);
    uint8_t columns[4], rows[4];
    uint32_t cache_bytes = (bytes + 31U) & ~31UL;
    (void)context; (void)token;
    drain_events();
    columns[0] = (uint8_t)(start_column >> 8); columns[1] = (uint8_t)start_column;
    columns[2] = (uint8_t)(end_column >> 8); columns[3] = (uint8_t)end_column;
    rows[0] = (uint8_t)(window->y1 >> 8); rows[1] = (uint8_t)window->y1;
    rows[2] = (uint8_t)(window->y2 >> 8); rows[3] = (uint8_t)window->y2;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
    if (!send_command(0x2AU, columns, 4U) || !send_command(0x2BU, rows, 4U) ||
        !send_command(0x2CU, NULL, 0U)) { return 0; }
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
    SCB_CleanDCache_by_Addr((uint32_t *)buffer, (int32_t)cache_bytes);
    __DSB();
    if (HAL_SPI_Transmit_DMA(&lcd_spi, (uint8_t *)buffer, (uint16_t)bytes) != HAL_OK)
    { lcd_status.spi_errors++; lcd_status.last_hal_error = lcd_spi.ErrorCode; return 0; }
    /* Half-complete notifications have no ownership meaning and are unnecessary. */
    CLEAR_BIT(DMA1_Stream0->CR, DMA_SxCR_HTIE);
    return 1;
}

/** Publish terminal state before notifying LVGL; recovery remains an explicit UI decision. */
static void transfer_done(void *context, uint32_t token, LcdTransferResult result)
{
    (void)context;
    lcd_status.last_result = result;
    if (result != LCD_TRANSFER_OK) { lcd_status.state = LCD_ST7789_FAULT; set_backlight(0U); }
    else { set_backlight(1U); }
    if (completion_hook != NULL) { completion_hook(completion_context, token, result); }
}

/** Configure pins from the verified H723 mapping with safe initial GPIO levels. */
static void init_gpio(void)
{
    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE(); __HAL_RCC_GPIOE_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
    gpio.Mode = GPIO_MODE_OUTPUT_PP; gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_15; HAL_GPIO_Init(GPIOE, &gpio);
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11; HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_10; HAL_GPIO_Init(GPIOD, &gpio);
    gpio.Mode = GPIO_MODE_AF_PP; gpio.Speed = GPIO_SPEED_FREQ_HIGH; gpio.Alternate = GPIO_AF5_SPI1;
    gpio.Pin = GPIO_PIN_3; HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_7; HAL_GPIO_Init(GPIOD, &gpio);
}

/** Configure private HAL resources. MSP weak defaults intentionally do no extra work. */
static int init_resources(void)
{
    RCC_PeriphCLKInitTypeDef clock_config;
    memset(&clock_config, 0, sizeof(clock_config));
    clock_config.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    clock_config.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&clock_config) != HAL_OK) { return 0; }
    __HAL_RCC_SPI1_CLK_ENABLE(); __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI1_FORCE_RESET(); __DSB(); __HAL_RCC_SPI1_RELEASE_RESET();
    memset(&lcd_spi, 0, sizeof(lcd_spi)); memset(&lcd_dma, 0, sizeof(lcd_dma));
    lcd_dma.Instance = DMA1_Stream0;
    lcd_dma.Init.Request = DMA_REQUEST_SPI1_TX;
    lcd_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    lcd_dma.Init.PeriphInc = DMA_PINC_DISABLE; lcd_dma.Init.MemInc = DMA_MINC_ENABLE;
    lcd_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    lcd_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    lcd_dma.Init.Mode = DMA_NORMAL; lcd_dma.Init.Priority = DMA_PRIORITY_LOW;
    lcd_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    lcd_dma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    lcd_dma.Init.MemBurst = DMA_MBURST_SINGLE; lcd_dma.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&lcd_dma) != HAL_OK) { return 0; }
    lcd_spi.Instance = SPI1;
    lcd_spi.Init.Mode = SPI_MODE_MASTER; lcd_spi.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    lcd_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    lcd_spi.Init.CLKPolarity = SPI_POLARITY_LOW; lcd_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    lcd_spi.Init.NSS = SPI_NSS_SOFT; lcd_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    lcd_spi.Init.FirstBit = SPI_FIRSTBIT_MSB; lcd_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    lcd_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; lcd_spi.Init.CRCPolynomial = 7U;
    lcd_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE; lcd_spi.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    lcd_spi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    lcd_spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    lcd_spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    lcd_spi.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    lcd_spi.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    lcd_spi.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    lcd_spi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    lcd_spi.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    __HAL_LINKDMA(&lcd_spi, hdmatx, lcd_dma);
    if (HAL_SPI_Init(&lcd_spi) != HAL_OK) { return 0; }
    drain_events();
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 8U, 0U);
    HAL_NVIC_SetPriority(SPI1_IRQn, 8U, 0U);
    return 1;
}

/** Initialize from UiTask only; explicit recovery preserves counters and transaction epoch. */
int lcd_st7789_init(LcdTransferDone done, void *done_context)
{
    const LcdTransferOps operations = {start_pixels, stop_hardware, release_bus};
    uint32_t enabled = lock_lcd();
    if (lcd_tx.state != LCD_TRANSFER_IDLE)
    { unlock_lcd(enabled); return 0; }
    if (resources_ready && !stop_hardware(NULL)) { unlock_lcd(enabled); return 0; }
    resources_ready = 0U;
    init_gpio(); set_backlight(0U);
    if (!init_resources())
    {
        lcd_status.init_errors++; lcd_status.state = LCD_ST7789_FAULT;
        return 0;
    }
    if (!transfer_initialized)
    {
        lcd_transfer_init(&lcd_tx, &operations, NULL, transfer_done, NULL);
        transfer_initialized = 1U;
    }
    completion_hook = done; completion_context = done_context;
    resources_ready = 1U; eot_authorized = 0U;
    lcd_status.state = LCD_ST7789_OFF;
    unlock_lcd(3U);
    return 1;
}

/** Start hardware reset; 100-ms low/high and 120-ms sleep-out waits are task states. */
int lcd_st7789_start(uint32_t now_ms)
{
    uint32_t enabled = lock_lcd();
    if (!resources_ready || lcd_status.state != LCD_ST7789_OFF || lcd_tx.state != LCD_TRANSFER_IDLE)
    { unlock_lcd(enabled); return 0; }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    set_backlight(0U); init_stage = 0U; init_index = 0U; stage_started_ms = now_ms;
    lcd_status.state = LCD_ST7789_INITIALIZING;
    unlock_lcd(enabled);
    return 1;
}

/** Execute at most one panel command per call; every synchronous HAL call has 2-ms limit. */
static void service_init(uint32_t now_ms)
{
    int success = 1;
    if (init_stage == 0U && now_ms - stage_started_ms >= 100U)
    { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); stage_started_ms = now_ms; init_stage = 1U; }
    else if (init_stage == 1U && now_ms - stage_started_ms >= 100U)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        success = send_command(0x11U, NULL, 0U);
        release_bus(NULL); stage_started_ms = HAL_GetTick(); init_stage = 2U;
    }
    else if (init_stage == 2U && now_ms - stage_started_ms >= 120U) { init_stage = 3U; }
    else if (init_stage == 3U)
    {
        const LcdInitCommand *entry = &init_commands[init_index];
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        success = send_command(entry->command, entry->parameters, entry->count);
        release_bus(NULL);
        init_index++;
        if (init_index == sizeof(init_commands) / sizeof(init_commands[0]))
        { lcd_status.state = LCD_ST7789_READY; }
    }
    if (!success)
    {
        (void)stop_hardware(NULL); lcd_status.init_errors++;
        lcd_status.state = LCD_ST7789_FAULT; set_backlight(0U);
    }
}

/** Advance reset/initialization or one transport timeout/abort confirmation step. */
void lcd_st7789_service(uint32_t now_ms)
{
    uint32_t enabled = lock_lcd();
    if (resources_ready)
    {
        if (lcd_status.state == LCD_ST7789_INITIALIZING) { service_init(now_ms); }
        lcd_transfer_service(&lcd_tx, now_ms);
        if (lcd_tx.state == LCD_TRANSFER_STOPPING)
        { lcd_status.state = LCD_ST7789_FAULT; set_backlight(0U); }
    }
    unlock_lcd(enabled);
}

/** Enforce the dedicated AXI range including cache-line padding, never permitting DTCM. */
int lcd_st7789_begin_flush(const LcdTransferWindow *window, const void *buffer,
                           uint32_t bytes, uint32_t now_ms)
{
    uintptr_t address = (uintptr_t)buffer;
    uint32_t enabled = lock_lcd();
    int accepted = 0;
    if (lcd_status.state == LCD_ST7789_READY && resources_ready &&
        lcd_transfer_validate(window, (const uint8_t *)buffer, bytes) &&
        address >= LCD_ST7789_DMA_START && address < LCD_ST7789_DMA_END &&
        ((bytes + 31U) & ~31UL) <= LCD_ST7789_DMA_END - address)
    { accepted = lcd_transfer_begin(&lcd_tx, window, (const uint8_t *)buffer, bytes, now_ms); }
    else { lcd_tx.counters.rejected++; }
    unlock_lcd(enabled);
    return accepted;
}

/** Copy one consistent diagnostic record without masking control/timebase interrupts. */
void lcd_st7789_get_status(LcdSt7789Status *status)
{
    uint32_t enabled;
    if (status == NULL) { return; }
    enabled = lock_lcd();
    *status = lcd_status; status->transfer = lcd_tx.counters;
    status->transfer_state = lcd_tx.state; status->transaction = lcd_tx.token;
    status->buffer_owned = lcd_tx.state != LCD_TRANSFER_IDLE;
    unlock_lcd(enabled);
}

/** Explicitly stop the display; keep ownership if peripheral disable is not acknowledged. */
int lcd_st7789_abort(uint32_t now_ms)
{
    uint32_t enabled = lock_lcd();
    int released;
    if (resources_ready)
    {
        lcd_transfer_abort(&lcd_tx, now_ms);
        lcd_status.state = LCD_ST7789_FAULT; set_backlight(0U);
    }
    released = lcd_tx.state == LCD_TRANSFER_IDLE;
    unlock_lcd(enabled);
    return released;
}

/** Process DMA TC through HAL; errors use our bounded stop path instead of HAL abort waits. */
void lcd_st7789_dma_irq(void)
{
    uint32_t flags, control, fifo_control;
    if (!resources_ready) { HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn); return; }
    flags = DMA1->LISR;
    control = DMA1_Stream0->CR;
    fifo_control = DMA1_Stream0->FCR;
    irq_token = lcd_tx.token;
    error_callback_authorized = 1U;
    /* Match HAL 1.11.3 event enables: direct-mode FEIF with FEIE=0 is not an error IRQ. */
    if (((flags & DMA_LISR_TEIF0) != 0U && (control & DMA_SxCR_TEIE) != 0U) ||
        ((flags & DMA_LISR_DMEIF0) != 0U && (control & DMA_SxCR_DMEIE) != 0U) ||
        ((flags & DMA_LISR_FEIF0) != 0U && (fifo_control & DMA_SxFCR_FEIE) != 0U))
    {
        lcd_status.spi_errors++; lcd_status.last_hal_error = HAL_SPI_ERROR_DMA;
        lcd_transfer_error(&lcd_tx, lcd_tx.token, HAL_GetTick());
    }
    else { HAL_DMA_IRQHandler(&lcd_dma); }
    error_callback_authorized = 0U;
}

/** Snapshot EOT before HAL clears it; callbacks outside this IRQ cannot claim completion. */
void lcd_st7789_spi_irq(void)
{
    uint32_t flags;
    if (!resources_ready) { HAL_NVIC_ClearPendingIRQ(SPI1_IRQn); return; }
    flags = SPI1->SR;
    if ((flags & LCD_SPI_ERRORS) != 0U)
    {
        lcd_status.spi_errors++; lcd_status.last_hal_error = flags & LCD_SPI_ERRORS;
        lcd_transfer_error(&lcd_tx, lcd_tx.token, HAL_GetTick());
        return;
    }
    irq_token = lcd_tx.token;
    eot_authorized = (uint8_t)(lcd_tx.state == LCD_TRANSFER_ACTIVE &&
                      (flags & SPI1->IER & SPI_FLAG_EOT) != 0U);
    error_callback_authorized = 1U;
    HAL_SPI_IRQHandler(&lcd_spi);
    eot_authorized = 0U;
    error_callback_authorized = 0U;
}

/** HAL normal-DMA completion is accepted only from a wrapper-verified SPI EOT IRQ. */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle)
{
    if (handle != &lcd_spi) { return; }
    if (!eot_authorized) { lcd_tx.counters.late_events++; return; }
    eot_authorized = 0U;
    lcd_transfer_eot(&lcd_tx, irq_token, HAL_GetTick());
}

/** Handle HAL close-time errors using the same bounded buffer retention protocol. */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle)
{
    if (handle != &lcd_spi) { return; }
    if (!error_callback_authorized) { lcd_tx.counters.late_events++; return; }
    error_callback_authorized = 0U;
    lcd_status.spi_errors++; lcd_status.last_hal_error = handle->ErrorCode;
    lcd_transfer_error(&lcd_tx, irq_token, HAL_GetTick());
}
#endif
