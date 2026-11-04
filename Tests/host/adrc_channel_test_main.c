/** @file adrc_channel_test_main.c
 * @brief Verifies actual-transmit receipts, cancellation and no stale queue replay using a HAL fake.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "fdcan.h"
#include "s3519_codec.h"
#include "stm32_adrc_channel.h"
static FDCAN_GlobalTypeDef registers;
uint32_t adrc_test_irq_mask;
FDCAN_HandleTypeDef hfdcan1;
static unsigned writes, aborts;
static unsigned reject_write, reject_enable, reject_abort;
static FDCAN_TxHeaderTypeDef last_header;
static uint32_t fake_hal_now_ms;

/** @brief Supplies an independent clock so forged caller timestamps cannot extend a lease. */
uint32_t HAL_GetTick(void) { return fake_hal_now_ms; }

/** @brief Copies metadata without inventing a successful bus transmission. */
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxBuffer(FDCAN_HandleTypeDef *handle,
    const FDCAN_TxHeaderTypeDef *header, const uint8_t *data, uint32_t index)
{
    (void)handle; (void)data; assert(index == FDCAN_TX_BUFFER0 || index == FDCAN_TX_BUFFER1);
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
/** @brief Timer deadline requests a preloaded DISABLE even when no control collect call runs. */
static void test_independent_disable_deadline(void)
{
    CanFrame frame;
    memset(&registers, 0, sizeof(registers));
    reject_write = reject_enable = reject_abort = 0U;
    stm32_adrc_channel_init();
    memset(&frame, 0, sizeof(frame));
    frame.identifier = 7U; frame.length = 8U;
    memset(frame.data, 0xFF, 7U); frame.data[7] = 0xFDU;
    hfdcan1.State = HAL_FDCAN_STATE_READY;
    assert(!stm32_adrc_guard_arm(&frame, 100U, 20U));
    hfdcan1.State = HAL_FDCAN_STATE_BUSY;
    registers.CCCR = FDCAN_CCCR_INIT;
    assert(!stm32_adrc_guard_arm(&frame, 100U, 20U));
    registers.CCCR = 0U;
    adrc_test_irq_mask = 1U;
    reject_write = 1U;
    assert(!stm32_adrc_guard_arm(&frame, 100U, 20U));
    assert(adrc_test_irq_mask == 1U);
    reject_write = 0U;
    adrc_test_irq_mask = 0U;
    assert(stm32_adrc_guard_arm(&frame, 100U, 20U));
    assert(!stm32_adrc_guard_arm(&frame, 101U, 20U)); /* Cannot renew the deadline. */
    stm32_adrc_guard_tick_isr(119U);
    assert(registers.TXBAR == 0U);
    registers.TXBRP = FDCAN_TX_BUFFER0;
    stm32_adrc_guard_tick_isr(120U);
    assert(registers.TXBCR == FDCAN_TX_BUFFER0);
    assert(registers.TXBAR == FDCAN_TX_BUFFER1);
    assert(stm32_adrc_guard_state() == 2U); /* Requested, not proof of transmission. */
    assert(!stm32_adrc_guard_ack_disabled());
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_ENABLE, 0.0F));
    registers.TXBRP = 0U; registers.TXBCR = 0U;
    registers.TXBTO = FDCAN_TX_BUFFER1;
    registers.CCCR = FDCAN_CCCR_INIT;
    stm32_adrc_guard_tick_isr(121U);
    assert(stm32_adrc_guard_state() == 2U);
    registers.CCCR = 0U;
    stm32_adrc_guard_tick_isr(121U);
    assert(stm32_adrc_guard_state() == 3U);
    assert(stm32_adrc_guard_ack_disabled());
    assert(stm32_adrc_guard_state() == 0U);
    assert(stm32_adrc_guard_arm(&frame, UINT32_MAX - 9U, 20U));
    registers.TXBAR = 0U; registers.TXBTO = 0U;
    stm32_adrc_guard_tick_isr(9U);
    assert(registers.TXBAR == 0U);
    stm32_adrc_guard_tick_isr(10U);
    assert(registers.TXBAR == FDCAN_TX_BUFFER1);
    registers.PSR = FDCAN_PSR_BO;
    assert(stm32_adrc_guard_state() == 2U && !stm32_adrc_guard_ack_disabled());
    registers.PSR = 0U; registers.TXBTO = FDCAN_TX_BUFFER1; registers.TXBCR = 0U;
    stm32_adrc_guard_tick_isr(11U);
    assert(stm32_adrc_guard_ack_disabled());
    frame.data[7] = 0xFCU;
    assert(!stm32_adrc_guard_arm(&frame, 100U, 20U));
    frame.data[7] = 0xFDU; frame.identifier = 8U;
    assert(!stm32_adrc_guard_arm(&frame, 100U, 20U));
    frame.identifier = 7U;
    assert(!stm32_adrc_guard_arm(&frame, 100U, 0U));
    assert(!stm32_adrc_guard_arm(&frame, 100U, 21U));
}

/** @brief Restores idle fake hardware and returns the exact allowed motor-7 stop payload. */
static CanFrame reset_trial_fixture(uint32_t now_ms)
{
    CanFrame frame;
    memset(&registers, 0, sizeof(registers));
    memset(&frame, 0, sizeof(frame));
    frame.identifier = 7U; frame.length = 8U;
    memset(frame.data, 0xFF, 7U); frame.data[7] = 0xFDU;
    hfdcan1.Instance = &registers; hfdcan1.Init.TxBuffersNbr = 8U;
    hfdcan1.State = HAL_FDCAN_STATE_BUSY;
    reject_write = reject_enable = reject_abort = 0U;
    fake_hal_now_ms = now_ms;
    adrc_test_irq_mask = 0U;
    stm32_adrc_channel_init();
    return frame;
}

/** @brief Exercises lease expiry, immutable cap, rollover, transport admission and actual stop proof. */
static void test_trial_guard(void)
{
    CanFrame frame = reset_trial_fixture(100U);
    uint32_t deadline_ms;
    uint32_t elapsed_ms;
    assert(!stm32_adrc_guard_arm_trial(NULL, 100U));
    frame.data[7] = 0xFCU;
    assert(!stm32_adrc_guard_arm_trial(&frame, 100U));
    frame.data[7] = 0xFDU;
    assert(!stm32_adrc_guard_arm_trial(&frame, 101U));
    assert(!stm32_adrc_guard_arm_trial(&frame, 99U));
    assert(stm32_adrc_guard_arm_trial(&frame, 100U));
    assert(stm32_adrc_guard_observe(&deadline_ms, NULL) == 1U && deadline_ms == 112U);
    assert(!stm32_adrc_guard_arm(&frame, 100U, 20U));
    assert(!stm32_adrc_guard_arm_trial(&frame, 100U));
    fake_hal_now_ms = 111U;
    assert(!stm32_adrc_guard_renew_trial(110U));
    assert(!stm32_adrc_guard_renew_trial(112U));
    assert(stm32_adrc_guard_renew_trial(111U));
    assert(stm32_adrc_guard_observe(&deadline_ms, NULL) == 1U && deadline_ms == 123U);
    fake_hal_now_ms = 110U;
    assert(!stm32_adrc_guard_renew_trial(110U));
    fake_hal_now_ms = 123U;
    assert(!stm32_adrc_guard_renew_trial(123U)); /* ISR has not run: still expired. */
    assert(!stm32_adrc_guard_renew_trial(111U));
    stm32_adrc_guard_tick_isr(123U);
    assert(stm32_adrc_guard_state() == 2U);
    assert(registers.TXBAR == FDCAN_TX_BUFFER1 && registers.TXBCR == FDCAN_TX_BUFFER0);
    assert(!stm32_adrc_guard_renew_trial(123U));
    registers.TXBCR = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    stm32_adrc_guard_tick_isr(124U);
    assert(stm32_adrc_guard_state() == 2U && !stm32_adrc_guard_ack_disabled());
    registers.TXBTO |= FDCAN_TX_BUFFER1;
    stm32_adrc_guard_tick_isr(123U);
    assert(stm32_adrc_guard_state() == 2U); /* Same tick cannot confirm. */
    stm32_adrc_guard_tick_isr(124U);
    assert(stm32_adrc_guard_state() == 3U);
    assert(!stm32_adrc_guard_renew_trial(123U));
    assert(stm32_adrc_guard_ack_disabled());
    assert(!stm32_adrc_guard_renew_trial(123U));
    assert(stm32_adrc_guard_arm(&frame, 123U, 20U));
    assert(!stm32_adrc_guard_renew_trial(123U));
    assert(stm32_adrc_guard_observe(&deadline_ms, NULL) == 1U && deadline_ms == 143U);

    frame = reset_trial_fixture(0U);
    assert(stm32_adrc_guard_arm_trial(&frame, 0U));
    fake_hal_now_ms = 1U;
    hfdcan1.State = HAL_FDCAN_STATE_READY;
    assert(!stm32_adrc_guard_renew_trial(1U));
    hfdcan1.State = HAL_FDCAN_STATE_BUSY;
    registers.CCCR = FDCAN_CCCR_INIT;
    assert(!stm32_adrc_guard_renew_trial(1U));
    registers.CCCR = 0U; registers.PSR = FDCAN_PSR_BO;
    assert(!stm32_adrc_guard_renew_trial(1U));
    registers.PSR = 0U;
    registers.TXBRP = FDCAN_TX_BUFFER1;
    assert(!stm32_adrc_guard_renew_trial(1U));
    registers.TXBRP = 0U; registers.TXBCR = FDCAN_TX_BUFFER1;
    assert(!stm32_adrc_guard_renew_trial(1U));
    registers.TXBCR = 0U;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_DISABLE, 0.0F));
    assert(!stm32_adrc_guard_renew_trial(1U));
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    assert(!stm32_adrc_guard_renew_trial(1U)); /* Must collect even a completed request. */
    assert(stm32_adrc_channel_collect().transmitted);
    assert(stm32_adrc_guard_renew_trial(1U));
    assert(!stm32_adrc_guard_ack_disabled()); /* Ordinary FD is not independent FD proof. */
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_TORQUE, 0.0F));
    assert(!stm32_adrc_channel_collect().transmitted);
    registers.TXBRP = 0U;
    assert(!stm32_adrc_guard_renew_trial(1U));
    registers.TXBCR = 0U;
    adrc_test_irq_mask = 1U;
    assert(stm32_adrc_guard_renew_trial(1U));
    assert(adrc_test_irq_mask == 1U);

    frame = reset_trial_fixture(UINT32_MAX - 5U);
    assert(stm32_adrc_guard_arm_trial(&frame, fake_hal_now_ms));
    for (elapsed_ms = 10U; elapsed_ms < 2000U; elapsed_ms += 10U)
    {
        fake_hal_now_ms = (UINT32_MAX - 5U) + elapsed_ms;
        assert(stm32_adrc_guard_renew_trial(fake_hal_now_ms));
        stm32_adrc_guard_tick_isr(fake_hal_now_ms);
        assert(stm32_adrc_guard_state() == 1U);
    }
    assert(stm32_adrc_guard_observe(&deadline_ms, NULL) == 1U && deadline_ms == 1994U);
    fake_hal_now_ms = 1993U;
    assert(stm32_adrc_guard_renew_trial(fake_hal_now_ms));
    assert(stm32_adrc_guard_observe(&deadline_ms, NULL) == 1U && deadline_ms == 1994U);
    fake_hal_now_ms = 1994U;
    assert(!stm32_adrc_guard_renew_trial(fake_hal_now_ms));
    stm32_adrc_guard_tick_isr(fake_hal_now_ms);
    assert(stm32_adrc_guard_state() == 2U && registers.TXBAR == FDCAN_TX_BUFFER1);

    frame = reset_trial_fixture(3000U);
    assert(stm32_adrc_guard_arm_trial(&frame, 3000U));
    stm32_adrc_guard_tick_isr(3011U);
    assert(stm32_adrc_guard_state() == 1U);
    stm32_adrc_guard_tick_isr(3012U);
    assert(stm32_adrc_guard_state() == 2U);
}

/** @brief Runs transport lifecycle cases without any actual hardware access. */
int main(void)
{
    CanFrame frame;
    AdrcCanReceipt receipt;
    const uint8_t neutral_mit_payload[8] =
        {0x7FU, 0xFFU, 0x7FU, 0xF0U, 0x00U, 0x00U, 0x07U, 0xFFU};
    const S3519Ranges discovered_ranges = {12.5F, 200.0F, 10.0F};
    uint8_t neutral_byte_index;
    uint8_t saved_byte;
    memset(&frame, 0, sizeof(frame)); frame.identifier = 7U; frame.length = 8U;
    hfdcan1.Instance = &registers; hfdcan1.Init.TxBuffersNbr = 8U;
    hfdcan1.State = HAL_FDCAN_STATE_BUSY;
    stm32_adrc_channel_init();
    assert(!stm32_adrc_channel_collect().available);
    frame.identifier = 0x7FFU; frame.length = 4U;
    frame.data[0] = 7U; frame.data[2] = 0xCCU;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    assert(last_header.Identifier == 0x7FFU && last_header.DataLength == FDCAN_DLC_BYTES_4);
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    receipt = stm32_adrc_channel_collect();
    assert(receipt.available && receipt.transmitted && receipt.kind == ADRC_CAN_PROBE);
    frame.length = 8U;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    assert(last_header.Identifier == 0x7FFU && last_header.DataLength == FDCAN_DLC_BYTES_8);
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    receipt = stm32_adrc_channel_collect();
    assert(receipt.available && receipt.transmitted && receipt.kind == ADRC_CAN_PROBE);
    frame.identifier = 7U; frame.length = 8U;
    memset(frame.data, 0xFF, 7U);
    frame.data[7] = 0xFDU;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    assert(last_header.Identifier == 7U && last_header.DataLength == FDCAN_DLC_BYTES_8);
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    receipt = stm32_adrc_channel_collect();
    assert(receipt.available && receipt.transmitted && receipt.kind == ADRC_CAN_PROBE);
    assert(s3519_pack_mit(7U, &discovered_ranges,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, &frame) == S3519_CODEC_STATUS_OK);
    assert(memcmp(frame.data, neutral_mit_payload, sizeof(neutral_mit_payload)) == 0);
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    assert(last_header.Identifier == 7U && last_header.DataLength == FDCAN_DLC_BYTES_8);
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    receipt = stm32_adrc_channel_collect();
    assert(receipt.available && receipt.transmitted && receipt.kind == ADRC_CAN_PROBE);
    for (neutral_byte_index = 0U; neutral_byte_index < 8U; ++neutral_byte_index)
    {
        saved_byte = frame.data[neutral_byte_index];
        frame.data[neutral_byte_index] = (uint8_t)(saved_byte + 1U);
        assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
        frame.data[neutral_byte_index] = saved_byte;
    }
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.001F));
    frame.identifier = 8U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    frame.identifier = 7U;
    frame.length = 7U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    frame.length = 8U;
    memset(frame.data, 0xFF, 7U);
    frame.data[7] = 0xFCU;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    frame.data[7] = 0xFDU; frame.data[0] = 0U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    memset(frame.data, 0, sizeof(frame.data));
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_TORQUE, 0.012F));
    assert(last_header.Identifier == 7U && last_header.DataLength == 8U);
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_TORQUE, 0.02F));
    assert(writes == 5U);
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
    frame.data[4] = 1U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_PROBE, 0.0F));
    frame.data[4] = 0U;
    frame.length = 5U;
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
    reject_enable = 0U;
    memset(&registers, 0, sizeof(registers));
    memset(&frame, 0, sizeof(frame));
    frame.identifier = 0x7FFU; frame.length = 4U;
    frame.data[0] = 7U; frame.data[2] = 0x33U; frame.data[3] = 9U;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_WATCHDOG_PARAMETER, 0.0F));
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    assert(stm32_adrc_channel_collect().transmitted);
    frame.length = 8U; frame.data[2] = 0x55U; frame.data[4] = 120U;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_WATCHDOG_PARAMETER, 0.0F));
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    assert(stm32_adrc_channel_collect().transmitted);
    frame.data[4] = 121U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_WATCHDOG_PARAMETER, 0.0F));
    frame.data[4] = 0U;
    assert(stm32_adrc_channel_submit(&frame, ADRC_CAN_WATCHDOG_PARAMETER, 0.0F));
    registers.TXBRP = 0U; registers.TXBTO = FDCAN_TX_BUFFER0;
    assert(stm32_adrc_channel_collect().transmitted);
    frame.data[2] = 0xAAU;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_WATCHDOG_PARAMETER, 0.0F));
    frame.data[2] = 0x55U; frame.data[3] = 10U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_WATCHDOG_PARAMETER, 0.0F));
    frame.data[3] = 9U; frame.data[0] = 8U;
    assert(!stm32_adrc_channel_submit(&frame, ADRC_CAN_WATCHDOG_PARAMETER, 0.0F));
    test_independent_disable_deadline();
    test_trial_guard();
    puts("ADRC dedicated CAN channel tests passed");
    return 0;
}
