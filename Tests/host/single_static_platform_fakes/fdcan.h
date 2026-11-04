/** @file fdcan.h @brief Deterministic H723 FDCAN metadata/register/HAL fake for the actual single-static platform adapter. */
#ifndef SINGLE_STATIC_PLATFORM_FAKE_FDCAN_H
#define SINGLE_STATIC_PLATFORM_FAKE_FDCAN_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
/** @brief Hardware-owned status fields are individually controlled by each transport ordering test. */
typedef struct { uint32_t TXBRP, TXBTO, TXBCF, TXBCR, IR, TXEFS, TXEFC; } FDCAN_GlobalTypeDef;
/** @brief Only current production-platform configuration fields are represented. */
typedef struct { FDCAN_GlobalTypeDef *Instance; struct { uint32_t StdFiltersNbr, TxFifoQueueElmtsNbr,
    TxEventsNbr, NominalPrescaler, NominalTimeSeg1, NominalTimeSeg2; } Init; } FDCAN_HandleTypeDef;
/** @brief Classic HAL TX header including the dedicated eight-bit event marker. */
typedef struct { uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator,
    BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker; } FDCAN_TxHeaderTypeDef;
/** @brief Raw hardware RX metadata preserves the original SOF counter. */
typedef struct { uint32_t Identifier, IdType, RxFrameType, DataLength, BitRateSwitch, FDFormat, RxTimestamp; } FDCAN_RxHeaderTypeDef;
/** @brief TX event DLC follows the HAL's decoded unshifted representation. */
typedef struct { uint32_t Identifier, IdType, TxFrameType, DataLength, BitRateSwitch,
    FDFormat, TxTimestamp, MessageMarker, EventType; } FDCAN_TxEventFifoTypeDef;
/** @brief Preserve the production exact-ID filter configuration contract. */
typedef struct { uint32_t IdType, FilterType, FilterConfig, FilterID2, FilterIndex, FilterID1; } FDCAN_FilterTypeDef;
extern FDCAN_HandleTypeDef hfdcan1;
extern uint32_t single_fake_irq_mask;
/** @brief Verify metadata has been captured before the production adapter restores the previous IRQ mask. */
void single_fake_restore_irq(uint32_t value);
#define __get_PRIMASK() (single_fake_irq_mask)
#define __disable_irq() ((void)(single_fake_irq_mask = 1U))
#define __set_PRIMASK(value) single_fake_restore_irq(value)
#define __DMB() __asm__ volatile("" ::: "memory")
#define FDCAN_DLC_BYTES_0 0U
#define FDCAN_DLC_BYTES_1 0x10000U
#define FDCAN_DLC_BYTES_2 0x20000U
#define FDCAN_DLC_BYTES_3 0x30000U
#define FDCAN_DLC_BYTES_4 0x40000U
#define FDCAN_DLC_BYTES_5 0x50000U
#define FDCAN_DLC_BYTES_6 0x60000U
#define FDCAN_DLC_BYTES_7 0x70000U
#define FDCAN_DLC_BYTES_8 0x80000U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_FILTER_MASK 2U
#define FDCAN_FILTER_TO_RXFIFO0 1U
#define FDCAN_REJECT 2U
#define FDCAN_REJECT_REMOTE 1U
#define FDCAN_CFG_RX_FIFO0 0U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_RX_FIFO1 1U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_ERROR_WARNING 2U
#define FDCAN_IT_ERROR_PASSIVE 4U
#define FDCAN_IT_BUS_OFF 8U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_STORE_TX_EVENTS 0x800000U
#define FDCAN_TIMESTAMP_PRESC_16 0xF0000U
#define FDCAN_TIMESTAMP_INTERNAL 1U
#define FDCAN_TX_EVENT 0x400000U
#define FDCAN_TXEFS_EFFL 0x3FU
#define FDCAN_TXEFS_TEFL 0x2000000U
#define FDCAN_IR_TEFL 0x8000U
#define RCC_PERIPHCLK_FDCAN 1U
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, const FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigFifoWatermark(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, const FDCAN_TxHeaderTypeDef *, const uint8_t *);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_FDCAN_ConfigTimestampCounter(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_EnableTimestampCounter(FDCAN_HandleTypeDef *, uint32_t);
uint16_t HAL_FDCAN_GetTimestampCounter(const FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *, FDCAN_TxEventFifoTypeDef *);
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t);
uint32_t HAL_GetTick(void);
#endif
