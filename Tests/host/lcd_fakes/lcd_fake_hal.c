/** @file lcd_fake_hal.c @brief Deterministic fake registers/HAL, never linked into firmware. */
#include "lcd_fake_hal.h"
#include "lcd_st7789.h"
#include "lcd_key_adc.h"
#include <string.h>
#include <assert.h>
SPI_TypeDef fake_spi1;
DMA_Stream_TypeDef fake_dma_stream0;
DMA_TypeDef fake_dma1;
DMAMUX_ChannelStatus_TypeDef fake_dmamux_status;
ADC_TypeDef fake_adc1;
GPIO_TypeDef fake_gpioa, fake_gpiob, fake_gpiod, fake_gpioe;
FakeHalState fake_hal;
static uint32_t irq_enabled[3], irq_pending[3];
/** Reset the independent test fixture. */
void fake_hal_reset(void)
{
    memset(&fake_hal, 0, sizeof(fake_hal));
    memset(irq_enabled, 0, sizeof(irq_enabled));
    memset(irq_pending, 0, sizeof(irq_pending));
    fake_spi_reset(); fake_adc_reset();
    memset(&fake_dma_stream0, 0, sizeof(fake_dma_stream0));
    memset(&fake_dma1, 0, sizeof(fake_dma1));
}
/** Emulate only SPI1 peripheral reset, leaving all other peripherals alone. */
void fake_spi_reset(void) { memset(&fake_spi1, 0, sizeof(fake_spi1)); }
/** Emulate the exclusive ADC12 block reset. */
void fake_adc_reset(void) { memset(&fake_adc1, 0, sizeof(fake_adc1)); }
/** Apply hardware write-one-clear register semantics at peripheral barriers. */
void fake_barrier(void)
{
    fake_spi1.SR &= ~fake_spi1.IFCR; fake_spi1.IFCR = 0U;
    fake_dma1.LISR &= ~fake_dma1.LIFCR; fake_dma1.LIFCR = 0U;
}
/** Accept DMA disable writes while retaining EN until outstanding bus access completes. */
void fake_clear_bits(uint32_t *reg, uint32_t bits)
{
    if ((fake_dma_stream0.CR & DMA_SxCR_EN) != 0U)
    {
        if ((reg == &fake_spi1.CR1 && (bits & SPI_CR1_SPE) != 0U) ||
            (reg == &fake_spi1.CFG1 && (bits & SPI_CFG1_TXDMAEN) != 0U))
        { fake_hal.dma_stop_order_errors++; }
        if (reg == &fake_dma_stream0.CR && (bits & DMA_SxCR_EN) != 0U)
        {
            if (fake_hal.dma_stuck || fake_hal.dma_disable_deferred)
            { fake_hal.dma_disable_pending = 1; bits &= ~DMA_SxCR_EN; }
            else { fake_hal.dma_disable_pending = 0; }
        }
    }
    *reg &= ~bits;
}
/** Emulate hardware EN readback becoming zero independently of task time or retries. */
void fake_dma_acknowledge_stop(void)
{
    assert(fake_hal.dma_disable_pending);
    fake_dma_stream0.CR &= ~DMA_SxCR_EN;
    fake_hal.dma_disable_pending = 0;
    fake_dma1.LISR |= DMA_LISR_TCIF0;
}
/** Deliver DMA TC without SPI EOT, demonstrating the two distinct events. */
void fake_dma_tc(void)
{
    fake_dma_stream0.CR &= ~DMA_SxCR_EN;
    fake_dma1.LISR |= DMA_LISR_TCIF0;
    irq_pending[DMA1_Stream0_IRQn] = 1U;
    lcd_st7789_dma_irq();
    irq_pending[DMA1_Stream0_IRQn] = 0U;
}
/** Deliver the actual SPI EOT after the last bit leaves the shifter. */
void fake_spi_eot(void)
{
    fake_spi1.SR |= SPI_FLAG_EOT;
    irq_pending[SPI1_IRQn] = 1U;
    lcd_st7789_spi_irq();
    irq_pending[SPI1_IRQn] = 0U;
}
/** Leave old peripheral flags and pending NVIC entries for the abort drain to retire. */
void fake_queue_old_events(void)
{
    fake_spi1.SR |= SPI_FLAG_EOT;
    fake_dma1.LISR |= DMA_LISR_TCIF0;
    irq_pending[SPI1_IRQn] = 1U;
    irq_pending[DMA1_Stream0_IRQn] = 1U;
}
/** Deliver exactly one ADC conversion value. */
void fake_adc_complete(uint16_t raw)
{
    fake_adc1.DR = raw; fake_adc1.CR &= ~ADC_CR_ADSTART;
    fake_adc1.ISR = ADC_FLAG_EOC | ADC_FLAG_EOS;
    lcd_key_adc_irq();
}
/** Return the manually advanced fake clock. */
uint32_t HAL_GetTick(void) { return fake_hal.tick; }
/** Accept GPIO mode setup; electrical characteristics are outside host scope. */
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config) { (void)port; (void)config; }
/** Record signal levels including active-low CS. */
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) { port->output |= pin; } else { port->output &= ~(uint32_t)pin; }
    if (port == GPIOE && pin == GPIO_PIN_15) { fake_hal.cs_high = state == GPIO_PIN_SET; }
}
/** Require IRQ8, compatible with the firmware syscall threshold of 5. */
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub)
{ (void)irq; assert(preempt == 8U && sub == 0U); }
/** Track local NVIC enable state. */
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { irq_enabled[irq] = 1U; }
/** Mask only a device-local IRQ. */
void HAL_NVIC_DisableIRQ(IRQn_Type irq) { irq_enabled[irq] = 0U; }
/** Retire a pending interrupt before the next transaction. */
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq) { irq_pending[irq] = 0U; }
/** Read NVIC state used to restore caller masking. */
uint32_t NVIC_GetEnableIRQ(IRQn_Type irq) { return irq_enabled[irq]; }
/** Inspect stale pending IRQ state in tests. */
uint32_t NVIC_GetPendingIRQ(IRQn_Type irq) { return irq_pending[irq]; }
/** Accept the isolated kernel clock setup. */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config) { (void)config; return HAL_OK; }
/** Initialize the dedicated DMA handle. */
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *handle) { handle->State = HAL_DMA_STATE_READY; return HAL_OK; }
/** Save the actual private SPI handle for IRQ fault injection. */
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle)
{ fake_hal.spi = handle; handle->State = HAL_SPI_STATE_READY; return HAL_OK; }
/** Record short command data and require a bounded timeout. */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *bytes, uint16_t size, uint32_t timeout)
{
    assert(timeout <= 2U && size <= 14U);
    if (handle->State != HAL_SPI_STATE_READY || handle->Lock != HAL_UNLOCKED) { return HAL_BUSY; }
    if (fake_hal.tx_fail) { return HAL_TIMEOUT; }
    if ((GPIOD->output & GPIO_PIN_10) == 0U) { fake_hal.command = bytes[0]; }
    else if (fake_hal.command == 0x3AU) { fake_hal.colmod = bytes[0]; }
    else if (fake_hal.command == 0x36U) { fake_hal.madctl = bytes[0]; }
    else if (fake_hal.command == 0x2AU && size == 4U)
    {
        fake_hal.column_start = (uint16_t)((uint16_t)bytes[0] * 256U + bytes[1]);
        fake_hal.column_end = (uint16_t)((uint16_t)bytes[2] * 256U + bytes[3]);
    }
    return HAL_OK;
}
/** HAL normal-mode DMA completion arms SPI EOT without publishing pixel completion. */
static void fake_spi_dma_complete(DMA_HandleTypeDef *handle)
{
    SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)handle->Parent;
    spi->Instance->IER |= SPI_IT_EOT;
}
/** Forward enabled DMA errors through the HAL SPI callback contract. */
static void fake_spi_dma_error(DMA_HandleTypeDef *handle)
{
    SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)handle->Parent;
    spi->ErrorCode = HAL_SPI_ERROR_DMA;
    HAL_SPI_ErrorCallback(spi);
}
/** Model HAL 1.11.3 normal/direct DMA event enables; never dereference synthetic AXI. */
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *handle, uint8_t *bytes, uint16_t size)
{
    (void)bytes; assert(size <= 13440U);
    if (handle->State != HAL_SPI_STATE_READY || handle->Lock != HAL_UNLOCKED ||
        handle->hdmatx->State != HAL_DMA_STATE_READY || handle->hdmatx->Lock != HAL_UNLOCKED)
    { return HAL_BUSY; }
    handle->State = HAL_SPI_STATE_BUSY_TX;
    handle->hdmatx->State = HAL_DMA_STATE_BUSY;
    handle->hdmatx->Lock = HAL_LOCKED;
    handle->hdmatx->XferCpltCallback = fake_spi_dma_complete;
    handle->hdmatx->XferErrorCallback = fake_spi_dma_error;
    handle->Instance->CR1 |= SPI_CR1_SPE;
    handle->Instance->CFG1 |= SPI_CFG1_TXDMAEN;
    handle->hdmatx->Instance->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    return HAL_OK;
}
/** HAL handles flags only with the corresponding CR/FCR interrupt source enabled. */
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *handle)
{
    uint32_t flags = fake_dma1.LISR;
    uint32_t enabled_errors = 0U;
    if ((handle->Instance->CR & DMA_SxCR_TEIE) != 0U) { enabled_errors |= DMA_LISR_TEIF0; }
    if ((handle->Instance->CR & DMA_SxCR_DMEIE) != 0U) { enabled_errors |= DMA_LISR_DMEIF0; }
    if ((handle->Instance->FCR & DMA_SxFCR_FEIE) != 0U) { enabled_errors |= DMA_LISR_FEIF0; }
    if ((flags & enabled_errors) != 0U)
    {
        fake_dma1.LISR &= ~(flags & enabled_errors);
        if (handle->XferErrorCallback != NULL) { handle->XferErrorCallback(handle); }
        return;
    }
    if ((flags & DMA_LISR_TCIF0) != 0U && (handle->Instance->CR & DMA_SxCR_TCIE) != 0U)
    {
        fake_dma1.LISR &= ~DMA_LISR_TCIF0;
        handle->Instance->CR &= ~DMA_SxCR_TCIE;
        handle->State = HAL_DMA_STATE_READY; handle->Lock = HAL_UNLOCKED;
        if (handle->XferCpltCallback != NULL) { handle->XferCpltCallback(handle); }
    }
}
/** Model the relevant HAL EOT close sequence and completion callback ordering. */
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *handle)
{
    if ((handle->Instance->SR & handle->Instance->IER & SPI_FLAG_EOT) != 0U)
    {
        handle->Instance->SR &= ~SPI_FLAG_EOT;
        handle->Instance->IER = 0U; handle->Instance->CR1 &= ~SPI_CR1_SPE;
        handle->Instance->CFG1 &= ~(SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
        handle->State = HAL_SPI_STATE_READY;
        HAL_SPI_TxCpltCallback(handle);
    }
}
/** Record exact TX cache clean bounds. */
void SCB_CleanDCache_by_Addr(uint32_t *address, int32_t size)
{ fake_hal.clean_address = (uintptr_t)address; fake_hal.clean_bytes = (uint32_t)size; }
/** Save the private ADC handle and initialize it. */
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *handle) { fake_hal.adc = handle; return HAL_OK; }
/** Enforce independent ADC acquisition. */
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *handle, ADC_MultiModeTypeDef *mode)
{ (void)handle; assert(mode->Mode == ADC_MODE_INDEPENDENT); return HAL_OK; }
/** Record PA5's configured ADC channel. */
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *handle, ADC_ChannelConfTypeDef *channel)
{ (void)handle; fake_hal.adc_channel = *channel; return HAL_OK; }
/** Record the offset calibration mode used by the vendor example. */
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *handle, uint32_t mode, uint32_t single)
{ (void)handle; (void)single; fake_hal.calibration = mode; return HAL_OK; }
/** Record LL offset calibration, matching the corresponding HAL internal call. */
void LL_ADC_StartCalibration(ADC_TypeDef *instance, uint32_t mode, uint32_t single)
{
    (void)single; fake_hal.calibration = mode; fake_hal.calibration_reads = 0U;
    if (fake_hal.calibration_stuck || fake_hal.calibration_preempt_read != 0U)
    { instance->CR |= ADC_CR_ADCAL; }
    else { instance->CR &= ~ADC_CR_ADCAL; }
}
/** Inject preemption after the MMIO busy read: time advances and hardware clears ADCAL. */
uint32_t LL_ADC_IsCalibrationOnGoing(ADC_TypeDef *instance)
{
    uint32_t busy = instance->CR & ADC_CR_ADCAL;
    fake_hal.calibration_reads++;
    if (fake_hal.calibration_reads == fake_hal.calibration_preempt_read)
    {
        fake_hal.tick += fake_hal.calibration_preempt_ms;
        instance->CR &= ~ADC_CR_ADCAL;
    }
    if (busy != 0U && fake_hal.calibration_stuck && !fake_hal.calibration_tick_frozen)
    { fake_hal.tick++; }
    return busy;
}
/** Arm one conversion, or inject a HAL start failure. */
HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef *handle)
{
    fake_hal.adc_starts++;
    if (fake_hal.adc_start_fail) { return HAL_ERROR; }
    handle->Instance->ISR = 0U; handle->Instance->IER = ADC_FLAG_EOC | ADC_FLAG_OVR;
    handle->Instance->CR |= ADC_CR_ADSTART;
    return HAL_OK;
}
/** Simulate bounded HAL task-context ADC stop. */
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef *handle)
{ handle->Instance->IER = 0U; handle->Instance->CR &= ~ADC_CR_ADSTART; return HAL_OK; }
/** Read only the published ADC data register. */
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *handle) { return handle->Instance->DR; }
/** Dispatch conversion completion and then clear the event. */
void HAL_ADC_IRQHandler(ADC_HandleTypeDef *handle)
{
    if ((handle->Instance->ISR & handle->Instance->IER & ADC_FLAG_EOC) != 0U)
    { HAL_ADC_ConvCpltCallback(handle); handle->Instance->ISR = 0U; }
}
