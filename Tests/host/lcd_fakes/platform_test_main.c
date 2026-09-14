/** @file platform_test_main.c @brief Fake HAL tests of real LCD/ADC platform entrypoints. */
#include "lcd_st7789.h"
#include "lcd_key_adc.h"
#include "lcd_fake_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL platform line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
static unsigned completion_count;
static LcdTransferResult last_result;
static const LcdTransferWindow tile = { 0U, 0U, 279U, 23U };
static const void *pixels = (const void *)0x24000000UL;

/** Observe exactly-once completion after hardware relinquishes ownership. */
static void completed(void *context, uint32_t token, LcdTransferResult result)
{
    (void)context; (void)token;
    CHECK((DMA1_Stream0->CR & DMA_SxCR_EN) == 0U);
    CHECK(fake_hal.cs_high);
    completion_count++;
    last_result = result;
}
/** Drive the nonblocking initialization without wall-clock waits. */
static void panel_ready(void)
{
    LcdSt7789Status status;
    uint32_t tick;
    CHECK(lcd_st7789_init(completed, NULL));
    CHECK(lcd_st7789_start(fake_hal.tick));
    for (tick = 0U; tick < 500U; tick += 5U)
    {
        fake_hal.tick += 5U;
        lcd_st7789_service(fake_hal.tick);
    }
    lcd_st7789_get_status(&status);
    CHECK(status.state == LCD_ST7789_READY && !status.backlight_on);
    CHECK(fake_hal.spi->Init.CLKPolarity == SPI_POLARITY_LOW);
    CHECK(fake_hal.spi->Init.CLKPhase == SPI_PHASE_1EDGE);
    CHECK(fake_hal.spi->Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16);
    CHECK(fake_hal.colmod == 0x55U && fake_hal.madctl == 0x70U);
}
/** Exercise the real IRQ/EOT guard, AXI range, cache clean and stop ownership. */
static void test_panel(void)
{
    LcdSt7789Status status;
    uint32_t start;
    fake_hal_reset();
    panel_ready();
    CHECK(!lcd_st7789_begin_flush(&tile, (void *)0x20000000UL, 13440U, fake_hal.tick));
    CHECK(!lcd_st7789_begin_flush(&tile, (void *)0x24006000UL, 13440U, fake_hal.tick));
    CHECK(!lcd_st7789_begin_flush(&tile, (void *)0x24000001UL, 13440U, fake_hal.tick));
    CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
    CHECK(fake_hal.clean_address == (uintptr_t)pixels && fake_hal.clean_bytes == 13440U);
    CHECK(fake_hal.column_start == 20U && fake_hal.column_end == 299U);
    CHECK(!lcd_st7789_init(completed, NULL));
    fake_dma_tc();
    CHECK(completion_count == 0U && !fake_hal.cs_high);
    HAL_SPI_TxCpltCallback(fake_hal.spi); /* No wrapper-observed EOT: must be ignored. */
    CHECK(completion_count == 0U);
    fake_spi_eot();
    CHECK(completion_count == 1U && last_result == LCD_TRANSFER_OK);
    lcd_st7789_get_status(&status);
    CHECK(status.backlight_on && !status.buffer_owned);
    start = fake_hal.tick;
    CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, start));
    HAL_SPI_TxCpltCallback(fake_hal.spi); /* Old direct callback cannot finish new DMA. */
    CHECK(completion_count == 1U);
    HAL_SPI_ErrorCallback(fake_hal.spi); /* A retired callback cannot fault the new transaction. */
    CHECK(completion_count == 1U);
    fake_hal.dma_stuck = 1;
    fake_hal.tick = start + 50U;
    lcd_st7789_service(fake_hal.tick);
    lcd_st7789_get_status(&status);
    CHECK(status.buffer_owned && status.transfer_state == LCD_TRANSFER_STOPPING);
    CHECK(completion_count == 1U && !fake_hal.cs_high);
    CHECK(!lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
    CHECK(!lcd_st7789_init(completed, NULL));
    fake_queue_old_events();
    fake_hal.dma_stuck = 0;
    fake_hal.tick++;
    lcd_st7789_service(fake_hal.tick);
    CHECK(completion_count == 2U && last_result == LCD_TRANSFER_TIMEOUT);
    CHECK(!NVIC_GetPendingIRQ(SPI1_IRQn) && !NVIC_GetPendingIRQ(DMA1_Stream0_IRQn));
    CHECK((SPI1->SR & SPI_FLAG_EOT) == 0U && (DMA1->LISR & DMA_LISR_TCIF0) == 0U);
    panel_ready();
    CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
    HAL_SPI_TxCpltCallback(fake_hal.spi);
    CHECK(completion_count == 2U);
    fake_dma_tc(); fake_spi_eot();
    CHECK(completion_count == 3U);
    fake_hal.tx_fail = 1;
    CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
    CHECK(completion_count == 4U && last_result == LCD_TRANSFER_IO_ERROR);
    fake_hal.tx_fail = 0;
    panel_ready();
    CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
    DMA1->LISR |= DMA_LISR_TEIF0;
    lcd_st7789_dma_irq();
    CHECK(completion_count == 5U && last_result == LCD_TRANSFER_IO_ERROR);
    panel_ready();
    CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
    SPI1->SR |= SPI_FLAG_UDR;
    lcd_st7789_spi_irq();
    CHECK(completion_count == 6U && last_result == LCD_TRANSFER_IO_ERROR);
    lcd_st7789_get_status(&status);
    CHECK(status.spi_errors >= 3U && status.state == LCD_ST7789_FAULT && !status.buffer_owned);
}
/** Publish only completed IRQ samples; timeouts and tick wrap invalidate acquisition. */
static void test_adc(void)
{
    LcdKeyAdcSample sample;
    LcdKeyAdcStatus status;
    uint32_t starts;
    fake_hal.tick = UINT32_MAX - 10U;
    CHECK(lcd_key_adc_init());
    CHECK(fake_hal.adc->Init.Resolution == ADC_RESOLUTION_12B);
    CHECK(fake_hal.adc_channel.Channel == ADC_CHANNEL_19);
    CHECK(fake_hal.calibration == ADC_CALIB_OFFSET);
    lcd_key_adc_latest(fake_hal.tick, &sample);
    CHECK(!sample.valid && sample.seq == 0U);
    lcd_key_adc_service(fake_hal.tick);
    starts = fake_hal.adc_starts;
    lcd_key_adc_service(fake_hal.tick + 1U);
    CHECK(fake_hal.adc_starts == starts);
    fake_adc_complete(1636U);
    lcd_key_adc_latest(fake_hal.tick, &sample);
    CHECK(sample.valid && sample.raw == 1636U && sample.seq == 1U && sample.ms == fake_hal.tick);
    lcd_key_adc_latest(fake_hal.tick - 1U, &sample); /* Caller latched time before the IRQ. */
    CHECK(sample.valid);
    HAL_ADC_ConvCpltCallback(fake_hal.adc);
    lcd_key_adc_latest(fake_hal.tick, &sample);
    CHECK(sample.seq == 1U);
    fake_hal.tick += 5U;
    lcd_key_adc_service(fake_hal.tick);
    CHECK(fake_hal.adc_starts == starts + 1U);
    fake_hal.tick += 50U;
    lcd_key_adc_service(fake_hal.tick);
    lcd_key_adc_latest(fake_hal.tick, &sample);
    CHECK(!sample.valid);
    lcd_key_adc_get_status(&status);
    CHECK(status.timeouts == 1U && !status.initialized);
    CHECK(lcd_key_adc_init());
    fake_hal.adc_start_fail = 1;
    lcd_key_adc_service(fake_hal.tick);
    lcd_key_adc_latest(fake_hal.tick, &sample);
    CHECK(!sample.valid);
    lcd_key_adc_get_status(&status);
    CHECK(status.start_errors >= 1U);
    fake_hal.adc_start_fail = 0;
    fake_hal.calibration_stuck = 1;
    starts = fake_hal.tick;
    CHECK(!lcd_key_adc_init());
    CHECK(fake_hal.tick - starts <= 11U);
    lcd_key_adc_latest(fake_hal.tick, &sample);
    CHECK(!sample.valid);
    fake_hal.calibration_stuck = 0;
    CHECK(lcd_key_adc_init());
    lcd_key_adc_service(fake_hal.tick);
    ADC1->ISR = ADC_FLAG_EOC | ADC_FLAG_OVR;
    lcd_key_adc_irq();
    lcd_key_adc_latest(fake_hal.tick, &sample);
    CHECK(!sample.valid);
    lcd_key_adc_get_status(&status);
    CHECK(status.irq_errors >= 1U);
}
/** Require RM0468 DMA EN acknowledgement before either SPI enable/request is cleared. */
static void test_dma_stop_order(void)
{
    const LcdTransferResult reasons[] = {
        LCD_TRANSFER_ABORTED, LCD_TRANSFER_TIMEOUT, LCD_TRANSFER_IO_ERROR, LCD_TRANSFER_IO_ERROR
    };
    LcdSt7789Status status;
    unsigned scenario, retry, completed_before;
    fake_hal_reset();
    for (scenario = 0U; scenario < sizeof(reasons) / sizeof(reasons[0]); scenario++)
    {
        panel_ready();
        completed_before = completion_count;
        CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
        fake_hal.dma_disable_deferred = 1;
        if (scenario == 0U) { CHECK(!lcd_st7789_abort(fake_hal.tick)); }
        else if (scenario == 1U) { fake_hal.tick += 50U; lcd_st7789_service(fake_hal.tick); }
        else if (scenario == 2U) { DMA1->LISR |= DMA_LISR_TEIF0; lcd_st7789_dma_irq(); }
        else { SPI1->SR |= SPI_FLAG_UDR; lcd_st7789_spi_irq(); }
        CHECK(fake_hal.dma_disable_pending && (DMA1_Stream0->CR & DMA_SxCR_EN) != 0U);
        CHECK((SPI1->CR1 & SPI_CR1_SPE) != 0U);
        CHECK((SPI1->CFG1 & SPI_CFG1_TXDMAEN) != 0U);
        CHECK(fake_hal.dma_stop_order_errors == 0U);
        CHECK(SPI1->IER == 0U);
        CHECK((DMA1_Stream0->CR & (DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE)) == 0U);
        for (retry = 0U; retry < 3U; retry++)
        {
            fake_hal.tick += 60000U; /* Elapsed time can never substitute for bus acknowledgement. */
            lcd_st7789_service(fake_hal.tick);
            lcd_st7789_get_status(&status);
            CHECK(status.buffer_owned && status.transfer_state == LCD_TRANSFER_STOPPING);
            CHECK(!fake_hal.cs_high && completion_count == completed_before);
            CHECK((DMA1_Stream0->CR & DMA_SxCR_EN) != 0U);
            CHECK((SPI1->CR1 & SPI_CR1_SPE) != 0U && (SPI1->CFG1 & SPI_CFG1_TXDMAEN) != 0U);
            CHECK(fake_hal.spi->hdmatx->State == HAL_DMA_STATE_BUSY);
            CHECK(fake_hal.spi->hdmatx->Lock == HAL_LOCKED && fake_hal.spi->State == HAL_SPI_STATE_BUSY_TX);
            CHECK(!lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
            CHECK(!lcd_st7789_init(completed, NULL));
        }
        fake_queue_old_events();
        fake_dma_acknowledge_stop();
        CHECK(!fake_hal.cs_high && completion_count == completed_before);
        CHECK((SPI1->CR1 & SPI_CR1_SPE) != 0U && (SPI1->CFG1 & SPI_CFG1_TXDMAEN) != 0U);
        lcd_st7789_service(fake_hal.tick);
        CHECK(completion_count == completed_before + 1U && last_result == reasons[scenario]);
        CHECK((SPI1->CR1 & SPI_CR1_SPE) == 0U && (SPI1->CFG1 & SPI_CFG1_TXDMAEN) == 0U);
        CHECK(fake_hal.dma_stop_order_errors == 0U);
        CHECK(!NVIC_GetPendingIRQ(SPI1_IRQn) && !NVIC_GetPendingIRQ(DMA1_Stream0_IRQn));
        CHECK(SPI1->SR == 0U && DMA1->LISR == 0U);
        CHECK(fake_hal.spi->hdmatx->State == HAL_DMA_STATE_READY && fake_hal.spi->hdmatx->Lock == HAL_UNLOCKED);
        CHECK(fake_hal.spi->State == HAL_SPI_STATE_READY && fake_hal.spi->Lock == HAL_UNLOCKED);
        lcd_st7789_service(fake_hal.tick);
        CHECK(lcd_st7789_abort(fake_hal.tick));
        CHECK(completion_count == completed_before + 1U);
        fake_hal.dma_disable_deferred = 0;
        panel_ready();
        CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
        lcd_st7789_dma_irq(); lcd_st7789_spi_irq(); /* Retired NVIC entries carry no new event. */
        CHECK(completion_count == completed_before + 1U);
        fake_dma_tc();
        CHECK(completion_count == completed_before + 1U && !fake_hal.cs_high);
        fake_spi_eot();
        CHECK(completion_count == completed_before + 2U && last_result == LCD_TRANSFER_OK);
    }
    puts("PASS regression stop-order: abort/timeout/DMA/SPI error retain SPE, TXDMAEN and buffer until EN=0");
}

/** Ignore disabled error flags while preserving TC -> EOT and subsequent HAL reuse. */
static void test_dma_error_enables(void)
{
    const uint32_t flags[] = { DMA_LISR_FEIF0, DMA_LISR_TEIF0, DMA_LISR_DMEIF0 };
    const uint32_t enables[] = { DMA_SxFCR_FEIE, DMA_SxCR_TEIE, DMA_SxCR_DMEIE };
    LcdSt7789Status before, after;
    unsigned event, completed_before;
    fake_hal_reset();
    for (event = 0U; event < sizeof(flags) / sizeof(flags[0]); event++)
    {
        panel_ready();
        completed_before = completion_count;
        lcd_st7789_get_status(&before);
        CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
        if (event == 0U) { CHECK((DMA1_Stream0->FCR & DMA_SxFCR_FEIE) == 0U); }
        else { DMA1_Stream0->CR &= ~enables[event]; }
        DMA1->LISR |= flags[event];
        fake_dma_tc();
        CHECK(completion_count == completed_before && !fake_hal.cs_high);
        lcd_st7789_get_status(&after);
        CHECK(after.spi_errors == before.spi_errors && after.transfer_state == LCD_TRANSFER_ACTIVE);
        CHECK(after.buffer_owned && (SPI1->IER & SPI_IT_EOT) != 0U);
        CHECK((DMA1->LISR & flags[event]) != 0U); /* Disabled flag is not an enabled error event. */
        fake_spi_eot();
        CHECK(completion_count == completed_before + 1U && last_result == LCD_TRANSFER_OK);
        lcd_st7789_get_status(&after);
        CHECK(after.backlight_on && !after.buffer_owned && after.spi_errors == before.spi_errors);
        CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
        fake_dma_tc(); fake_spi_eot();
        CHECK(completion_count == completed_before + 2U && last_result == LCD_TRANSFER_OK);
        CHECK(lcd_st7789_begin_flush(&tile, pixels, 13440U, fake_hal.tick));
        if (event == 0U) { DMA1_Stream0->FCR |= enables[event]; }
        else { DMA1_Stream0->CR |= enables[event]; }
        DMA1->LISR |= flags[event];
        lcd_st7789_dma_irq();
        CHECK(completion_count == completed_before + 3U && last_result == LCD_TRANSFER_IO_ERROR);
        lcd_st7789_get_status(&after);
        CHECK(after.state == LCD_ST7789_FAULT && after.spi_errors == before.spi_errors + 1U);
    }
    puts("PASS regression dma-enables: FEIF/FEIE=0 continues TC/EOT; TE/DME/FE obey interrupt enables");
}

/** Re-read ADCAL after a deadline/poll bound reached during preemption; real hangs still fail. */
static void test_calibration_preemption(void)
{
    LcdKeyAdcStatus before, after;
    LcdKeyAdcSample sample;
    unsigned scenario;
    fake_hal_reset();
    for (scenario = 0U; scenario < 3U; scenario++)
    {
        fake_hal.tick = scenario == 1U ? UINT32_MAX - 5U : 100U;
        fake_hal.calibration_preempt_read = scenario == 2U ? 100000U : 1U;
        fake_hal.calibration_preempt_ms = scenario == 0U ? 10U : (scenario == 1U ? 12U : 0U);
        lcd_key_adc_get_status(&before);
        CHECK(lcd_key_adc_init());
        CHECK((ADC1->CR & ADC_CR_ADCAL) == 0U);
        CHECK(fake_hal.calibration_reads == fake_hal.calibration_preempt_read + 1U);
        lcd_key_adc_get_status(&after);
        CHECK(after.initialized && after.init_errors == before.init_errors);
        CHECK(NVIC_GetEnableIRQ(ADC_IRQn));
        lcd_key_adc_latest(fake_hal.tick, &sample);
        CHECK(!sample.valid);
        lcd_key_adc_service(fake_hal.tick); fake_adc_complete(1636U);
        lcd_key_adc_latest(fake_hal.tick, &sample);
        CHECK(sample.valid && sample.raw == 1636U);
    }
    fake_hal.calibration_preempt_read = 0U; fake_hal.calibration_preempt_ms = 0U;
    fake_hal.calibration_stuck = 1;
    for (scenario = 0U; scenario < 2U; scenario++)
    {
        fake_hal.tick = 100U; fake_hal.calibration_tick_frozen = (int)scenario;
        lcd_key_adc_get_status(&before);
        CHECK(!lcd_key_adc_init());
        CHECK((ADC1->CR & ADC_CR_ADCAL) != 0U);
        CHECK(fake_hal.calibration_reads == (scenario == 0U ? 11U : 100001U));
        CHECK(fake_hal.tick == (scenario == 0U ? 111U : 100U));
        lcd_key_adc_get_status(&after);
        CHECK(!after.initialized && after.init_errors == before.init_errors + 1U);
        CHECK(!NVIC_GetEnableIRQ(ADC_IRQn));
        lcd_key_adc_latest(fake_hal.tick, &sample); CHECK(!sample.valid);
    }
    fake_hal.calibration_stuck = 0; fake_hal.calibration_tick_frozen = 0;
    CHECK(lcd_key_adc_init());
    puts("PASS regression cal-preempt: deadline/wrap/poll-bound re-read; stuck ADCAL and frozen tick bounded");
}

/** Run all software tests by default, or one named regression to capture independent RED evidence. */
int main(int argc, char **argv)
{
    if (argc == 2)
    {
        if (strcmp(argv[1], "stop-order") == 0) { test_dma_stop_order(); }
        else if (strcmp(argv[1], "dma-enables") == 0) { test_dma_error_enables(); }
        else if (strcmp(argv[1], "cal-preempt") == 0) { test_calibration_preemption(); }
        else { fprintf(stderr, "Unknown regression: %s\n", argv[1]); return 2; }
        return 0;
    }
    CHECK(argc == 1);
    test_panel(); test_adc(); test_dma_stop_order(); test_dma_error_enables(); test_calibration_preemption();
    puts("PASS: LCD/ADC platform fake HAL (EOT, cache, abort, recovery, ADC IRQ/timeouts)");
    return 0;
}
