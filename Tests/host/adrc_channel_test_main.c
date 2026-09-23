/** @file adrc_channel_test_main.c
 * @brief Verifies actual-transmit receipts, cancellation and no stale queue replay using a HAL fake.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "fdcan.h"
#include "stm32_adrc_channel.h"
static FDCAN_GlobalTypeDef registers;
FDCAN_HandleTypeDef hfdcan1;
static unsigned writes, aborts;
static unsigned reject_write, reject_enable, reject_abort;
static FDCAN_TxHeaderTypeDef last_header;

/** @brief Copies metadata without inventing a successful bus transmission. */
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxBuffer(FDCAN_HandleTypeDef *handle,
    const FDCAN_TxHeaderTypeDef *header, const uint8_t *data, uint32_t index)
{
    (void)handle; (void)data; assert(index == FDCAN_TX_BUFFER0);
    writes++; last_header = *header; return reject_write ? HAL_ERROR : HAL_OK;
}
/** @brief A new request clears the previous hardware completion bit, as on the controller. */
HAL_StatusTypeDef HAL_FDCAN_EnableTxBufferRequest(FDCAN_HandleTypeDef *handle, uint32_t index)
{
    if (reject_enable) { return HAL_ERROR; }
    handle->Instance->TXBRP |= index; handle->Instance->TXBTO &= ~index; return HAL_OK;
}
/** @brief Cancellation is asynchronous; the test decides when hardware clears pending. */
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *handle, uint32_t index)
{
    assert(index == FDCAN_TX_BUFFER0); aborts++;
    if (reject_abort) { return HAL_ERROR; }
    handle->Instance->TXBCR |= index;
    return HAL_OK;
}
/** @brief Runs transport lifecycle cases without any actual hardware access. */
int main(void)
{
    CanFrame frame;
    AdrcCanReceipt receipt;
    memset(&frame, 0, sizeof(frame)); frame.identifier = 7U; frame.length = 8U;
    hfdcan1.Instance = &registers; hfdcan1.Init.TxBuffersNbr = 8U;
    stm32_adrc_channel_init();
    assert(!stm32_adrc_channel_collect().available);
    frame.identifier = 0x7FFU; frame.length = 4U;
    frame.data[0] = 7U; frame.data[2] = 0xCCU;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    assert(last_header.Identifier == 0x7FFU && last_header.DataLength == FDCAN_DLC_BYTES_4);
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    receipt = stm32_adrc_channel_collect();
    assert(receipt.available && receipt.transmitted && receipt.kind == ADRC_CAN_PROBE);
    frame.identifier = 7U; frame.length = 8U;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_TORQUE, 0.012F));
    assert(last_header.Identifier == 7U && last_header.DataLength == 8U);
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_TORQUE, 0.02F));
    assert(writes == 2U);
    receipt = stm32_adrc_channel_collect();
    assert(receipt.available && !receipt.transmitted && aborts == 1U);
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_DISABLE, 0.0F));
    registers.TXBRP = 0U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_DISABLE, 0.0F));
    registers.TXBCR = 0U; registers.TXBCF = FDCAN_TX_BUFFER0;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_DISABLE, 0.0F));
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    receipt = stm32_adrc_channel_collect();
    assert(receipt.available && receipt.transmitted && receipt.kind == ADRC_CAN_DISABLE);
    assert(!stm32_adrc_channel_collect().available);
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_TORQUE, -0.013F));
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    receipt = stm32_adrc_channel_collect();
    assert(receipt.transmitted && receipt.decoded_torque_nm == -0.013F);
    /* A failed abort still cannot permit overwriting a request owned by hardware. */
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_TORQUE, 0.01F));
    reject_abort = 1U;
    receipt = stm32_adrc_channel_collect();
    assert(receipt.available && !receipt.transmitted);
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_DISABLE, 0.0F));
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    assert(!stm32_adrc_channel_collect().available);
    reject_abort = 0U;
    /* A late successful transmit during cancellation remains an expired receipt.
     * Reuse waits for the cancellation request to clear, then resets TXBTO. */
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_TORQUE, 0.02F));
    receipt = stm32_adrc_channel_collect();
    assert(receipt.available && !receipt.transmitted);
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_DISABLE, 0.0F));
    registers.TXBCR = 0U;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_DISABLE, 0.0F));
    assert(registers.TXBTO == 0U);
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    assert(stm32_adrc_channel_collect().transmitted);
    assert(!stm32_adrc_channel_submit(NULL, ADRC_CAN_TORQUE, 0.0F));
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_TORQUE, NAN));
    assert(!stm32_adrc_channel_submit(&frame, (AdrcCanKind)-1, 0.0F));
    frame.length = 4U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    frame.identifier = 0x7FFU;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.01F));
    frame.data[2] = 0U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    frame.data[2] = 0xCCU; frame.data[0] = 0U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    frame.data[0] = 7U;
    frame.length = 8U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    frame.identifier = 7U;
    frame.length = 7U; assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_ENABLE, 0.0F));
    frame.length = 8U; frame.identifier = 0x800U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_ENABLE, 0.0F));
    frame.identifier = 7U; registers.PSR = FDCAN_PSR_BO;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_ENABLE, 0.0F));
    registers.PSR = 0U; reject_write = 1U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_ENABLE, 0.0F));
    reject_write = 0U; reject_enable = 1U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_ENABLE, 0.0F));
    assert(!stm32_adrc_channel_collect().available);
    puts("ADRC dedicated CAN channel tests passed");
    return 0;
}
