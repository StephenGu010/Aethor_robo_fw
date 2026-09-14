/** @file lcd_fake_hal.h @brief Minimal test-only HAL/register surface; target uses real ST headers. */
#ifndef LCD_FAKE_HAL_H
#define LCD_FAKE_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { DMA1_Stream0_IRQn, SPI1_IRQn, ADC_IRQn } IRQn_Type;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
/** Test register banks retain only fields touched by the platform. */
typedef struct { uint32_t CR1, CFG1, IER, IFCR, SR; } SPI_TypeDef;
typedef struct { uint32_t CR, FCR; } DMA_Stream_TypeDef;
typedef struct { uint32_t LISR, LIFCR; } DMA_TypeDef;
typedef struct { uint32_t CFR; } DMAMUX_ChannelStatus_TypeDef;
typedef struct { uint32_t ISR, IER, CR, DR; } ADC_TypeDef;
typedef struct { uint32_t output; } GPIO_TypeDef;
extern SPI_TypeDef fake_spi1;
extern DMA_Stream_TypeDef fake_dma_stream0;
extern DMA_TypeDef fake_dma1;
extern DMAMUX_ChannelStatus_TypeDef fake_dmamux_status;
extern ADC_TypeDef fake_adc1;
extern GPIO_TypeDef fake_gpioa, fake_gpiob, fake_gpiod, fake_gpioe;
#define SPI1 (&fake_spi1)
#define DMA1_Stream0 (&fake_dma_stream0)
#define DMA1 (&fake_dma1)
#define DMAMUX1_ChannelStatus (&fake_dmamux_status)
#define ADC1 (&fake_adc1)
#define GPIOA (&fake_gpioa)
#define GPIOB (&fake_gpiob)
#define GPIOD (&fake_gpiod)
#define GPIOE (&fake_gpioe)
/** Test SPI initialization layout mirrors all configured real HAL fields. */
typedef struct {
    uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS, BaudRatePrescaler;
    uint32_t FirstBit, TIMode, CRCCalculation, CRCPolynomial, NSSPMode, NSSPolarity;
    uint32_t FifoThreshold, TxCRCInitializationPattern, RxCRCInitializationPattern;
    uint32_t MasterSSIdleness, MasterInterDataIdleness, MasterReceiverAutoSusp;
    uint32_t MasterKeepIOState, IOSwap;
} SPI_InitTypeDef;
/** DMA configuration and callback state needed by the fake IRQ dispatcher. */
typedef struct {
    uint32_t Request, Direction, PeriphInc, MemInc, PeriphDataAlignment;
    uint32_t MemDataAlignment, Mode, Priority, FIFOMode, FIFOThreshold, MemBurst, PeriphBurst;
} DMA_InitTypeDef;
typedef struct DMA_HandleTypeDef {
    DMA_Stream_TypeDef *Instance; DMA_InitTypeDef Init; void *Parent;
    uint32_t State, Lock, ErrorCode;
    void (*XferCpltCallback)(struct DMA_HandleTypeDef *);
    void (*XferHalfCpltCallback)(struct DMA_HandleTypeDef *);
    void (*XferErrorCallback)(struct DMA_HandleTypeDef *);
    void (*XferAbortCallback)(struct DMA_HandleTypeDef *);
} DMA_HandleTypeDef;
/** SPI handle is private to the driver; fake HAL records the observed pointer. */
typedef struct {
    SPI_TypeDef *Instance; SPI_InitTypeDef Init; DMA_HandleTypeDef *hdmatx;
    uint32_t State, Lock, ErrorCode; uint16_t TxXferCount, RxXferCount;
} SPI_HandleTypeDef;
/** ADC initialization/channel structures mirror the fields used for 12-bit IT. */
typedef struct {
    uint32_t ClockPrescaler, Resolution, ScanConvMode, EOCSelection, LowPowerAutoWait;
    uint32_t ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode;
    uint32_t ExternalTrigConv, ExternalTrigConvEdge, ConversionDataManagement;
    uint32_t Overrun, LeftBitShift, OversamplingMode;
} ADC_InitTypeDef;
typedef struct { ADC_TypeDef *Instance; ADC_InitTypeDef Init; uint32_t State, Lock, ErrorCode; } ADC_HandleTypeDef;
typedef struct { uint32_t Mode; } ADC_MultiModeTypeDef;
typedef struct {
    uint32_t Channel, Rank, SamplingTime, SingleDiff, OffsetNumber, Offset, OffsetSignedSaturation;
} ADC_ChannelConfTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct {
    uint32_t PLL2M, PLL2N, PLL2P, PLL2Q, PLL2R, PLL2RGE, PLL2VCOSEL, PLL2FRACN;
} FakePll2;
typedef struct {
    uint32_t PeriphClockSelection, Spi123ClockSelection, AdcClockSelection;
    FakePll2 PLL2;
} RCC_PeriphCLKInitTypeDef;
/** External observations and fault injection; no production conditionals depend on it. */
typedef struct {
    uint32_t tick; int cs_high, dma_stuck, tx_fail, adc_start_fail, calibration_stuck;
    int dma_disable_deferred, dma_disable_pending, calibration_tick_frozen;
    uint32_t dma_stop_order_errors, calibration_reads, calibration_preempt_read, calibration_preempt_ms;
    SPI_HandleTypeDef *spi; ADC_HandleTypeDef *adc; ADC_ChannelConfTypeDef adc_channel;
    uintptr_t clean_address; uint32_t clean_bytes, adc_starts, calibration;
    uint16_t column_start, column_end; uint8_t madctl, colmod, command;
} FakeHalState;
extern FakeHalState fake_hal;

#define DISABLE 0U
#define ENABLE 1U
#define HAL_UNLOCKED 0U
#define HAL_LOCKED 1U
#define HAL_SPI_STATE_RESET 0U
#define HAL_SPI_STATE_READY 1U
#define HAL_SPI_STATE_BUSY_TX 2U
#define HAL_DMA_STATE_READY 1U
#define HAL_DMA_STATE_BUSY 2U
#define HAL_DMA_ERROR_NONE 0U
#define HAL_SPI_ERROR_NONE 0U
#define HAL_SPI_ERROR_DMA 1U
#define HAL_SPI_ERROR_FLAG 2U
#define SPI_MODE_MASTER 1U
#define SPI_DIRECTION_2LINES_TXONLY 1U
#define SPI_DATASIZE_8BIT 7U
#define SPI_POLARITY_LOW 0U
#define SPI_PHASE_1EDGE 0U
#define SPI_NSS_SOFT 1U
#define SPI_BAUDRATEPRESCALER_16 3U
#define SPI_FIRSTBIT_MSB 0U
#define SPI_TIMODE_DISABLE 0U
#define SPI_CRCCALCULATION_DISABLE 0U
#define SPI_NSS_PULSE_DISABLE 0U
#define SPI_NSS_POLARITY_LOW 0U
#define SPI_FIFO_THRESHOLD_01DATA 0U
#define SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN 0U
#define SPI_MASTER_SS_IDLENESS_00CYCLE 0U
#define SPI_MASTER_INTERDATA_IDLENESS_00CYCLE 0U
#define SPI_MASTER_RX_AUTOSUSP_DISABLE 0U
#define SPI_MASTER_KEEP_IO_STATE_ENABLE 1U
#define SPI_IO_SWAP_DISABLE 0U
#define SPI_CR1_SPE 1U
#define SPI_CFG1_TXDMAEN 2U
#define SPI_CFG1_RXDMAEN 4U
#define SPI_FLAG_EOT 8U
#define SPI_IT_EOT SPI_FLAG_EOT
#define SPI_FLAG_UDR 16U
#define SPI_FLAG_OVR 32U
#define SPI_FLAG_FRE 64U
#define SPI_FLAG_MODF 128U
#define DMA_REQUEST_SPI1_TX 38U
#define DMA_MEMORY_TO_PERIPH 1U
#define DMA_PINC_DISABLE 0U
#define DMA_MINC_ENABLE 1U
#define DMA_PDATAALIGN_BYTE 0U
#define DMA_MDATAALIGN_BYTE 0U
#define DMA_NORMAL 0U
#define DMA_PRIORITY_LOW 0U
#define DMA_FIFOMODE_DISABLE 0U
#define DMA_FIFO_THRESHOLD_FULL 0U
#define DMA_MBURST_SINGLE 0U
#define DMA_PBURST_SINGLE 0U
#define DMA_SxCR_EN 1U
#define DMA_SxCR_TCIE 16U
#define DMA_SxCR_HTIE 8U
#define DMA_SxCR_TEIE 4U
#define DMA_SxCR_DMEIE 2U
#define DMA_SxFCR_FEIE 128U
#define DMA_LISR_FEIF0 1U
#define DMA_LISR_DMEIF0 4U
#define DMA_LISR_TEIF0 8U
#define DMA_LISR_TCIF0 32U
#define ADC_CLOCK_ASYNC_DIV16 16U
#define ADC_RESOLUTION_12B 12U
#define ADC_SCAN_DISABLE 0U
#define ADC_EOC_SINGLE_CONV 1U
#define ADC_SOFTWARE_START 0U
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0U
#define ADC_CONVERSIONDATA_DR 0U
#define ADC_OVR_DATA_PRESERVED 0U
#define ADC_LEFTBITSHIFT_NONE 0U
#define ADC_MODE_INDEPENDENT 0U
#define ADC_CHANNEL_19 19U
#define ADC_REGULAR_RANK_1 1U
#define ADC_SAMPLETIME_810CYCLES_5 810U
#define ADC_SINGLE_ENDED 0U
#define ADC_OFFSET_NONE 0U
#define ADC_CALIB_OFFSET 1U
#define ADC_FLAG_EOC 4U
#define ADC_FLAG_EOS 8U
#define ADC_FLAG_OVR 16U
#define ADC_CR_ADSTART 4U
#define ADC_CR_ADSTP 16U
#define ADC_CR_ADCAL (1UL << 31)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_15 (1U << 15)
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_MODE_ANALOG 3U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_SPEED_FREQ_HIGH 2U
#define GPIO_AF5_SPI1 5U
#define RCC_PERIPHCLK_SPI1 1U
#define RCC_SPI123CLKSOURCE_PLL 0U
#define RCC_PERIPHCLK_ADC 2U
#define RCC_ADCCLKSOURCE_PLL2 1U
#define RCC_PLL2VCIRANGE_3 3U
#define RCC_PLL2VCOWIDE 0U
#define SET_BIT(reg, bits) ((reg) |= (bits))
#define CLEAR_BIT(reg, bits) fake_clear_bits(&(reg), (bits))
#define __HAL_LINKDMA(handle, field, dma) do { (handle)->field = &(dma); (dma).Parent = (handle); } while (0)
#define __HAL_SPI_DISABLE(handle) CLEAR_BIT((handle)->Instance->CR1, SPI_CR1_SPE)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_ADC12_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI1_FORCE_RESET() fake_spi_reset()
#define __HAL_RCC_SPI1_RELEASE_RESET() ((void)0)
#define __HAL_RCC_ADC12_FORCE_RESET() fake_adc_reset()
#define __HAL_RCC_ADC12_RELEASE_RESET() ((void)0)
#define __DSB() fake_barrier()
#define __DMB() fake_barrier()
#define __ISB() ((void)0)
/** Fake hardware lifecycle and event injectors. */
void fake_hal_reset(void);
void fake_spi_reset(void);
void fake_adc_reset(void);
void fake_barrier(void);
void fake_clear_bits(uint32_t *reg, uint32_t bits);
/** Finish outstanding DMA bus accesses after a previously accepted disable request. */
void fake_dma_acknowledge_stop(void);
void fake_dma_tc(void);
void fake_spi_eot(void);
void fake_adc_complete(uint16_t raw);
/** Queue old DMA/SPI events without dispatching them, to test abort draining. */
void fake_queue_old_events(void);
/** Fake HAL functions preserve the signatures consumed by target platform code. */
uint32_t HAL_GetTick(void);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
void HAL_NVIC_DisableIRQ(IRQn_Type irq);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq);
uint32_t NVIC_GetEnableIRQ(IRQn_Type irq);
uint32_t NVIC_GetPendingIRQ(IRQn_Type irq);
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *bytes, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *handle, uint8_t *bytes, uint16_t size);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *handle);
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *handle);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle);
void SCB_CleanDCache_by_Addr(uint32_t *address, int32_t size);
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *handle, ADC_MultiModeTypeDef *mode);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *handle, ADC_ChannelConfTypeDef *channel);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *handle, uint32_t mode, uint32_t single);
/** Test LL calibration primitive, matching the primitive invoked by HAL itself. */
void LL_ADC_StartCalibration(ADC_TypeDef *instance, uint32_t mode, uint32_t single);
uint32_t LL_ADC_IsCalibrationOnGoing(ADC_TypeDef *instance);
HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef *handle);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *handle);
void HAL_ADC_IRQHandler(ADC_HandleTypeDef *handle);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *handle);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *handle);
#endif
