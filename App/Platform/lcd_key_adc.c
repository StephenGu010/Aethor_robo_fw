/** @file lcd_key_adc.c
 * @brief H723 private ADC1 PA5/INP19 single-channel 12-bit interrupt acquisition.
 * Vendor evidence: dm-mc02/例程/CtrBoard-H7_KEY/Core/Src/adc.c uses PA5/channel19,
 * PLL2 M2,N16,P2 / ADC async DIV16; main.c calibrates ADC_CALIB_OFFSET single-ended.
 * Current HSE=24 MHz gives PLL2P=96 MHz, /16=6 MHz, then H723 ADC12 fixed /2
 * yields 3 MHz conversion clock (HAL ADC_ConfigureBoostMode accounts for /2).
 * No PLL1/system clock edit.
 * Sampling 810.5 cycles gives extra settling time for the resistive key source;
 * this is a candidate setting, not a measured ADC distribution or key threshold.
 * ADC12 and its kernel clock are exclusively owned here; ADC2 is unused in the
 * inspected firmware. Revisit that ownership before introducing an ADC2 user.
 */
#include "debug_ui_config.h"
#if AETHOR_DEBUG_UI_ENABLE
#include "lcd_key_adc.h"
#if defined(LCD_PLATFORM_HOST_TEST)
#include "lcd_fake_hal.h"
#else
#include "stm32h7xx_hal.h"
#endif
#include <string.h>
#define KEY_REQUEST_PERIOD_MS 5U
#define KEY_SAMPLE_TIMEOUT_MS 50U
#define KEY_CALIBRATION_TIMEOUT_MS 10U
#define KEY_CALIBRATION_MAX_POLLS 100000U
static ADC_HandleTypeDef key_adc;
static LcdKeyAdcSample key_sample;
static LcdKeyAdcStatus key_status;
static uint32_t requested_ms;
static uint8_t request_started, adc_irq_authorized;

/** Serialize the bounded sample copy against ADC only; never mask motor or timebase IRQs. */
static uint32_t lock_adc(void)
{
    uint32_t enabled = NVIC_GetEnableIRQ(ADC_IRQn);
    HAL_NVIC_DisableIRQ(ADC_IRQn);
    __DSB(); __ISB();
    return enabled;
}

/** Restore the original ADC IRQ enable bit after a publication/read operation. */
static void unlock_adc(uint32_t enabled)
{
    __DMB();
    if (enabled) { HAL_NVIC_EnableIRQ(ADC_IRQn); }
}

/** Stop accepting IRQ samples on failure. Hardware stop itself is deferred to UiTask. */
static void invalidate_adc(void)
{
    key_sample.valid = 0U;
    key_status.initialized = 0U;
    key_status.conversion_pending = 0U;
    ADC1->IER = 0U;
    ADC1->ISR = ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR;
    adc_irq_authorized = 0U;
    __DSB();
    HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
}

/** Use the same LL offset calibration as HAL, with a 10-ms plus finite-poll bound.
 * ADC has just been reset/initialized and is disabled, with no concurrent owner.
 * HAL 1.11.3's generic 633600000-poll failure limit is deliberately not used.
 */
static int calibrate_offset(void)
{
    uint32_t started_ms = HAL_GetTick();
    uint32_t polls = 0U;
    volatile uint32_t settle_cycles;
    LL_ADC_StartCalibration(ADC1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0U)
    {
        polls++;
        if (HAL_GetTick() - started_ms >= KEY_CALIBRATION_TIMEOUT_MS ||
            polls >= KEY_CALIBRATION_MAX_POLLS)
        {
            /* A higher-priority IRQ may span completion after the loop's ADCAL read.
             * Recheck hardware at the bound, as HAL does for ADC ready/stop waits.
             */
            if (LL_ADC_IsCalibrationOnGoing(ADC1) != 0U) { return 0; }
            break;
        }
    }
    /* At 480 MHz even 1024 single-cycle iterations exceed four 3-MHz ADC cycles. */
    for (settle_cycles = 0U; settle_cycles < 1024U; settle_cycles++) { __DMB(); }
    return 1;
}

/** Initialize private ADC and calibrate only in UiTask; every vendor HAL wait is bounded. */
int lcd_key_adc_init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_PeriphCLKInitTypeDef clock_config;
    ADC_MultiModeTypeDef multimode;
    ADC_ChannelConfTypeDef channel;
    uint32_t enabled = lock_adc();
    if (key_status.conversion_pending) { unlock_adc(enabled); return 0; }
    key_status.initialized = 0U;
    key_sample.valid = 0U;
    request_started = 0U;
    adc_irq_authorized = 0U;
    memset(&gpio, 0, sizeof(gpio)); memset(&clock_config, 0, sizeof(clock_config));
    memset(&multimode, 0, sizeof(multimode)); memset(&channel, 0, sizeof(channel));
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_5; gpio.Mode = GPIO_MODE_ANALOG; gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    clock_config.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    clock_config.PLL2.PLL2M = 2U; clock_config.PLL2.PLL2N = 16U;
    clock_config.PLL2.PLL2P = 2U; clock_config.PLL2.PLL2Q = 2U; clock_config.PLL2.PLL2R = 2U;
    clock_config.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    clock_config.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE; clock_config.PLL2.PLL2FRACN = 0U;
    clock_config.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&clock_config) != HAL_OK)
    { key_status.init_errors++; return 0; }
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC12_FORCE_RESET(); __DSB(); __HAL_RCC_ADC12_RELEASE_RESET();
    memset(&key_adc, 0, sizeof(key_adc));
    key_adc.Instance = ADC1;
    key_adc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
    key_adc.Init.Resolution = ADC_RESOLUTION_12B; key_adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    key_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV; key_adc.Init.LowPowerAutoWait = DISABLE;
    key_adc.Init.ContinuousConvMode = DISABLE; key_adc.Init.NbrOfConversion = 1U;
    key_adc.Init.DiscontinuousConvMode = DISABLE; key_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    key_adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    key_adc.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    key_adc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    key_adc.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE; key_adc.Init.OversamplingMode = DISABLE;
    multimode.Mode = ADC_MODE_INDEPENDENT;
    channel.Channel = ADC_CHANNEL_19; channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_810CYCLES_5; channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE; channel.Offset = 0U; channel.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_Init(&key_adc) != HAL_OK ||
        HAL_ADCEx_MultiModeConfigChannel(&key_adc, &multimode) != HAL_OK ||
        HAL_ADC_ConfigChannel(&key_adc, &channel) != HAL_OK || !calibrate_offset())
    { key_status.init_errors++; invalidate_adc(); return 0; }
    ADC1->ISR = ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR;
    HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
    HAL_NVIC_SetPriority(ADC_IRQn, 8U, 0U);
    key_status.initialized = 1U;
    unlock_adc(1U);
    return 1;
}

/** Schedule one conversion per 5 ms and fail closed at a 50-ms acquisition timeout. */
void lcd_key_adc_service(uint32_t now_ms)
{
    uint32_t enabled = lock_adc();
    if (!key_status.initialized) { unlock_adc(enabled); return; }
    if (key_status.conversion_pending)
    {
        if (now_ms - requested_ms >= KEY_SAMPLE_TIMEOUT_MS)
        {
            key_status.timeouts++; invalidate_adc();
            /* HAL stop may wait for ADSTP; this task path leaves TIM23/RTOS IRQs enabled. */
            (void)HAL_ADC_Stop_IT(&key_adc);
        }
    }
    else if (!request_started || now_ms - requested_ms >= KEY_REQUEST_PERIOD_MS)
    {
        requested_ms = now_ms; request_started = 1U;
        key_status.conversion_pending = 1U;
        HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
        if (HAL_ADC_Start_IT(&key_adc) != HAL_OK)
        {
            key_status.start_errors++; invalidate_adc();
            (void)HAL_ADC_Stop_IT(&key_adc);
        }
    }
    unlock_adc(enabled);
}

/** Return an atomic copy; freshness is computed on a wrapping 32-bit HAL millisecond clock. */
void lcd_key_adc_latest(uint32_t now_ms, LcdKeyAdcSample *sample)
{
    uint32_t enabled;
    if (sample == NULL) { return; }
    enabled = lock_adc();
    *sample = key_sample;
    /* IRQ may publish after the caller captured now_ms. A slightly newer sample
     * is fresh, not UINT32_MAX milliseconds old; intervals must remain <2^31 ms.
     */
    if (!key_status.initialized ||
        ((int32_t)(now_ms - sample->ms) >= (int32_t)KEY_SAMPLE_TIMEOUT_MS))
    { sample->valid = 0U; }
    unlock_adc(enabled);
}

/** Copy error counters and readiness consistently without global interrupt masking. */
void lcd_key_adc_get_status(LcdKeyAdcStatus *status)
{
    uint32_t enabled;
    if (status == NULL) { return; }
    enabled = lock_adc(); *status = key_status; unlock_adc(enabled);
}

/** Gate callbacks on a pending, timely conversion and actual EOC; reject late IRQs. */
void lcd_key_adc_irq(void)
{
    uint32_t flags;
    if (key_adc.Instance != ADC1) { HAL_NVIC_ClearPendingIRQ(ADC_IRQn); return; }
    flags = ADC1->ISR;
    if (!key_status.initialized || !key_status.conversion_pending)
    {
        key_status.late_irqs++;
        ADC1->ISR = ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR;
        HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
        return;
    }
    if ((flags & ADC_FLAG_OVR) != 0U || HAL_GetTick() - requested_ms >= KEY_SAMPLE_TIMEOUT_MS)
    {
        if ((flags & ADC_FLAG_OVR) != 0U) { key_status.irq_errors++; }
        else { key_status.timeouts++; }
        invalidate_adc();
        return;
    }
    adc_irq_authorized = (uint8_t)((flags & ADC1->IER & ADC_FLAG_EOC) != 0U);
    HAL_ADC_IRQHandler(&key_adc);
    adc_irq_authorized = 0U;
}

/** ISR publishes only completed raw/sequence/time; no filtering, allocation or UI calls. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *handle)
{
    if (handle != &key_adc) { return; }
    if (!adc_irq_authorized || !key_status.conversion_pending)
    { key_status.late_irqs++; return; }
    adc_irq_authorized = 0U;
    key_sample.raw = (uint16_t)(HAL_ADC_GetValue(handle) & 0x0FFFU);
    key_sample.ms = HAL_GetTick();
    key_sample.seq++;
    key_sample.valid = 1U;
    __DMB();
    key_status.conversion_pending = 0U;
}

/** ADC error callback only invalidates publication and records the acquisition fault. */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *handle)
{
    if (handle != &key_adc) { return; }
    key_status.irq_errors++; invalidate_adc();
}
#endif
