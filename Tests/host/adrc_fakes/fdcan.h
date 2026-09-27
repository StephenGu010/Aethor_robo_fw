/** @file fdcan.h
 * @brief Minimal deterministic HAL fake for the dedicated ADRC transmit channel only.
 */
#ifndef ADRC_TEST_FDCAN_H
#define ADRC_TEST_FDCAN_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
/** @brief Fake hardware status registers; values are controlled by the test. */
typedef struct { uint32_t TXBRP, TXBTO, TXBCR, TXBCF, PSR; } FDCAN_GlobalTypeDef;
/** @brief Only members referenced by the channel under test. */
typedef struct { FDCAN_GlobalTypeDef *Instance; struct { uint32_t TxBuffersNbr; } Init; } FDCAN_HandleTypeDef;
/** @brief Classic transmit metadata copied by the fake HAL. */
typedef struct { uint32_t Identifier, IdType, TxFrameType, DataLength,
    ErrorStateIndicator, BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker; } FDCAN_TxHeaderTypeDef;
#define FDCAN_TX_BUFFER0 1U
#define FDCAN_PSR_BO 128U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
extern FDCAN_HandleTypeDef hfdcan1;
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxBuffer(FDCAN_HandleTypeDef *, const FDCAN_TxHeaderTypeDef *, const uint8_t *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_EnableTxBufferRequest(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *, uint32_t);
#endif
