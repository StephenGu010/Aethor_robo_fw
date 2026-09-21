/** @file stm32_adrc_channel.c
 * @brief Uses dedicated FDCAN buffer 0, with next-tick expiry and hardware transmission evidence.
 * ArmControlTask is the sole caller. TXBTO proves bus transmission, not motor torque application.
 */
#include "stm32_adrc_channel.h"
#include "fdcan.h"
#include <float.h>
#include <stddef.h>
#include <string.h>

static AdrcCanReceipt pending_receipt;

/** @brief Initializes bookkeeping only before control service begins. */
void stm32_adrc_channel_init(void)
{
    memset(&pending_receipt, 0, sizeof(pending_receipt));
}

/** @brief Consumes one receipt and aborts a command that failed to transmit within its control interval. */
AdrcCanReceipt stm32_adrc_channel_collect(void)
{
    AdrcCanReceipt receipt = pending_receipt;
    if (receipt.available)
    {
        receipt.transmitted = (uint8_t)(hfdcan1.Instance != NULL &&
            (hfdcan1.Instance->TXBTO & FDCAN_TX_BUFFER0) != 0U &&
            (hfdcan1.Instance->TXBRP & FDCAN_TX_BUFFER0) == 0U &&
            (hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U);
        if (!receipt.transmitted && hfdcan1.Instance != NULL)
        { (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER0); }
        pending_receipt.available = 0U;
    }
    return receipt;
}

/** @brief Never overwrites pending hardware storage, including during asynchronous cancellation. */
uint8_t stm32_adrc_channel_submit(const CanFrame *frame, AdrcCanKind kind, float decoded_torque_nm)
{
    FDCAN_TxHeaderTypeDef header;
    if (frame == NULL || frame->identifier > CAN_STANDARD_MAX_IDENTIFIER || frame->length != 8U ||
        (unsigned)kind > (unsigned)ADRC_CAN_DISABLE || decoded_torque_nm != decoded_torque_nm ||
        decoded_torque_nm > FLT_MAX || decoded_torque_nm < -FLT_MAX || pending_receipt.available ||
        hfdcan1.Instance == NULL || hfdcan1.Init.TxBuffersNbr == 0U ||
        (hfdcan1.Instance->TXBRP & FDCAN_TX_BUFFER0) != 0U ||
        (hfdcan1.Instance->TXBCR & FDCAN_TX_BUFFER0) != 0U ||
        (hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U)
    { return 0U; }
    memset(&header, 0, sizeof(header));
    header.Identifier = frame->identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    if (HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &header, frame->data, FDCAN_TX_BUFFER0) != HAL_OK ||
        HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER0) != HAL_OK)
    { return 0U; }
    pending_receipt.kind = kind;
    pending_receipt.decoded_torque_nm = decoded_torque_nm;
    pending_receipt.transmitted = 0U;
    pending_receipt.available = 1U;
    return 1U;
}
