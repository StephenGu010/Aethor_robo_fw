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

/** @brief Allows only the diagnosed four/eight-byte vendor feedback query. */
static uint8_t valid_feedback_query(const CanFrame *frame)
{
    return (uint8_t)(frame->identifier == 0x7FFU &&
        (frame->length == 4U || frame->length == 8U) &&
        frame->data[0] != 0U && frame->data[1] == 0U &&
        frame->data[2] == 0xCCU && frame->data[3] == 0U &&
        (frame->length == 4U ||
         (frame->data[4] == 0U && frame->data[5] == 0U &&
          frame->data[6] == 0U && frame->data[7] == 0U)));
}

/** @brief Allows one exact motor-7 DISABLE challenge, never enable or torque. */
static uint8_t valid_acquire_disable(const CanFrame *frame)
{
    uint8_t byte_index;
    if (frame->identifier != 7U || frame->length != 8U || frame->data[7] != 0xFDU)
    { return 0U; }
    for (byte_index = 0U; byte_index < 7U; ++byte_index)
    {
        if (frame->data[byte_index] != 0xFFU) { return 0U; }
    }
    return 1U;
}

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
    if (frame == NULL || frame->identifier > CAN_STANDARD_MAX_IDENTIFIER ||
        (unsigned)kind > (unsigned)ADRC_CAN_PROBE ||
        (kind == ADRC_CAN_PROBE &&
         ((valid_feedback_query(frame) == 0U && valid_acquire_disable(frame) == 0U) ||
          decoded_torque_nm != 0.0F)) ||
        (kind != ADRC_CAN_PROBE && frame->length != 8U) ||
        decoded_torque_nm != decoded_torque_nm ||
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
    header.DataLength = frame->length == 4U ? FDCAN_DLC_BYTES_4 : FDCAN_DLC_BYTES_8;
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
