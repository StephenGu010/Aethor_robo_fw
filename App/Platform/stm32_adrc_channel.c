/** @file stm32_adrc_channel.c
 * @brief Uses dedicated FDCAN buffer 0, with exact probe allowlisting and transmission evidence.
 * ArmControlTask owns buffer 0; TIM23 exclusively requests preloaded DISABLE in buffer 1.
 * TXBTO proves bus transmission, not motor torque application.
 */
#include "stm32_adrc_channel.h"
#include "fdcan.h"
#include <float.h>
#include <stddef.h>
#include <string.h>

static AdrcCanReceipt pending_receipt;
static volatile uint32_t guard_deadline_ms;
static volatile uint32_t guard_fired_ms;
static volatile uint8_t guard_state;
static volatile uint8_t guard_trial_mode;
static volatile uint32_t guard_trial_hard_deadline_ms;
static volatile uint32_t guard_trial_last_now_ms;
/* Also declared by the target HAL; explicit declaration keeps the minimal host HAL usable. */
extern uint32_t HAL_GetTick(void);

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

/** @brief Allows only motor 7's zero-request MIT encoding after acquisition DISABLE. */
static uint8_t valid_acquire_neutral_mit(const CanFrame *frame)
{
    static const uint8_t neutral_mit_payload[8] =
        {0x7FU, 0xFFU, 0x7FU, 0xF0U, 0x00U, 0x00U, 0x07U, 0xFFU};
    return (uint8_t)(frame->identifier == 7U && frame->length == 8U &&
        memcmp(frame->data, neutral_mit_payload, sizeof(neutral_mit_payload)) == 0);
}

/** @brief Allows only motor-7 TIMEOUT read and fixed volatile 120/0 writes; never Flash save. */
static uint8_t valid_watchdog_parameter(const CanFrame *frame)
{
    if (frame->identifier != 0x7FFU || frame->data[0] != 7U ||
        frame->data[1] != 0U || frame->data[3] != 9U) { return 0U; }
    if (frame->length == 4U && frame->data[2] == 0x33U) { return 1U; }
    return (uint8_t)(frame->length == 8U && frame->data[2] == 0x55U &&
        (frame->data[4] == 0U || frame->data[4] == 120U) &&
        frame->data[5] == 0U && frame->data[6] == 0U && frame->data[7] == 0U);
}

/** @brief Initializes bookkeeping only before control service begins. */
void stm32_adrc_channel_init(void)
{
    memset(&pending_receipt, 0, sizeof(pending_receipt));
    guard_state = 0U;
    guard_deadline_ms = 0U;
    guard_fired_ms = 0U;
    guard_trial_mode = 0U;
    guard_trial_hard_deadline_ms = 0U;
    guard_trial_last_now_ms = 0U;
}

/** @brief Arms only a fixed disabled motor payload; an existing deadline cannot be extended. */
static uint8_t arm_guard(const CanFrame *disable_frame, uint32_t now_ms,
    uint32_t duration_ms, uint8_t trial_mode)
{
    FDCAN_TxHeaderTypeDef header;
    uint32_t interrupt_mask;
    uint8_t accepted = 0U;
    if (disable_frame == NULL || !valid_acquire_disable(disable_frame) ||
        duration_ms == 0U || duration_ms > 20U) { return 0U; }
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    if ((!trial_mode || now_ms == HAL_GetTick()) &&
        !guard_state && !pending_receipt.available && hfdcan1.Instance != NULL &&
        hfdcan1.State == HAL_FDCAN_STATE_BUSY &&
        (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U &&
        hfdcan1.Init.TxBuffersNbr >= 2U &&
        (hfdcan1.Instance->TXBRP & (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1)) == 0U &&
        (hfdcan1.Instance->TXBCR & (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1)) == 0U &&
        (hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U)
    {
        memset(&header, 0, sizeof(header));
        header.Identifier = 7U;
        header.IdType = FDCAN_STANDARD_ID;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_8;
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        if (HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &header, disable_frame->data,
                FDCAN_TX_BUFFER1) == HAL_OK)
        {
            guard_deadline_ms = now_ms + duration_ms;
            guard_fired_ms = 0U;
            guard_trial_mode = trial_mode;
            guard_trial_hard_deadline_ms = trial_mode ? now_ms + 2000U : 0U;
            guard_trial_last_now_ms = now_ms;
            __DMB();
            guard_state = 1U;
            accepted = 1U;
        }
    }
    __set_PRIMASK(interrupt_mask);
    return accepted;
}

/** @brief Preserves the ordinary nonrenewable 1..20 ms guard contract. */
uint8_t stm32_adrc_guard_arm(const CanFrame *disable_frame, uint32_t now_ms, uint32_t duration_ms)
{
    return arm_guard(disable_frame, now_ms, duration_ms, 0U);
}

/** @brief Starts the trial lease and absolute cap atomically with the preloaded DISABLE. */
uint8_t stm32_adrc_guard_arm_trial(const CanFrame *disable_frame, uint32_t now_ms)
{
    return arm_guard(disable_frame, now_ms, 12U, 1U);
}

/** @brief Renews only a live lease against current HAL time and idle, healthy dedicated buffers. */
uint8_t stm32_adrc_guard_renew_trial(uint32_t now_ms)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    uint32_t remaining_ms;
    uint8_t accepted = 0U;
    __disable_irq();
    if (guard_state == 1U && guard_trial_mode && now_ms == HAL_GetTick() &&
        (uint32_t)(now_ms - guard_trial_last_now_ms) < 0x80000000UL &&
        (uint32_t)(guard_deadline_ms - now_ms) > 0U &&
        (uint32_t)(guard_deadline_ms - now_ms) < 0x80000000UL &&
        (uint32_t)(guard_trial_hard_deadline_ms - now_ms) > 0U &&
        (uint32_t)(guard_trial_hard_deadline_ms - now_ms) < 0x80000000UL &&
        !pending_receipt.available && hfdcan1.Instance != NULL &&
        hfdcan1.State == HAL_FDCAN_STATE_BUSY && hfdcan1.Init.TxBuffersNbr >= 2U &&
        (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U &&
        (hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U &&
        (hfdcan1.Instance->TXBRP & (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1)) == 0U &&
        (hfdcan1.Instance->TXBCR & (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1)) == 0U)
    {
        remaining_ms = guard_trial_hard_deadline_ms - now_ms;
        guard_deadline_ms = now_ms + (remaining_ms < 12U ? remaining_ms : 12U);
        guard_trial_last_now_ms = now_ms;
        __DMB();
        accepted = 1U;
    }
    __set_PRIMASK(interrupt_mask);
    return accepted;
}

/** @brief Requests the preloaded stop from the hardware tick, independently of control-task execution. */
void stm32_adrc_guard_tick_isr(uint32_t now_ms)
{
    if (guard_state == 1U && ((uint32_t)(now_ms - guard_deadline_ms) < 0x80000000UL ||
        (guard_trial_mode &&
         (uint32_t)(now_ms - guard_trial_hard_deadline_ms) < 0x80000000UL)))
    {
        guard_state = 2U;
        guard_fired_ms = now_ms;
        if (hfdcan1.Instance != NULL)
        {
            hfdcan1.Instance->TXBCR = FDCAN_TX_BUFFER0;
            hfdcan1.Instance->TXBAR = FDCAN_TX_BUFFER1;
            __DMB();
        }
    }
    else if (guard_state == 2U && now_ms != guard_fired_ms && hfdcan1.Instance != NULL &&
        hfdcan1.State == HAL_FDCAN_STATE_BUSY &&
        (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U &&
        (hfdcan1.Instance->TXBTO & FDCAN_TX_BUFFER1) != 0U &&
        (hfdcan1.Instance->TXBRP & (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1)) == 0U &&
        (hfdcan1.Instance->TXBCR & FDCAN_TX_BUFFER0) == 0U &&
        (hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U)
    { guard_state = 3U; }
}

/** @brief Exports bus-request evidence without equating it to motor execution or power cutoff. */
uint8_t stm32_adrc_guard_state(void)
{
    return guard_state;
}

/** @brief Snapshots the ISR publication with its actual request time. */
uint8_t stm32_adrc_guard_observe(uint32_t *deadline_ms, uint32_t *fired_ms)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    uint8_t state;
    __disable_irq();
    state = guard_state;
    if (deadline_ms != NULL) { *deadline_ms = guard_deadline_ms; }
    if (fired_ms != NULL) { *fired_ms = guard_fired_ms; }
    __set_PRIMASK(interrupt_mask);
    return state;
}

/** @brief Releases the lock only after a matched guard transmission and caller-owned disabled proof. */
uint8_t stm32_adrc_guard_ack_disabled(void)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    uint8_t accepted = 0U;
    __disable_irq();
    if (guard_state == 3U && hfdcan1.Instance != NULL &&
        hfdcan1.State == HAL_FDCAN_STATE_BUSY &&
        (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U &&
        (hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U &&
        (hfdcan1.Instance->TXBRP & (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1)) == 0U)
    {
        guard_trial_mode = 0U;
        guard_trial_hard_deadline_ms = 0U;
        guard_trial_last_now_ms = 0U;
        guard_state = 0U;
        accepted = 1U;
    }
    __set_PRIMASK(interrupt_mask);
    return accepted;
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
static uint8_t submit_frame(const CanFrame *frame, AdrcCanKind kind, float decoded_torque_nm)
{
    FDCAN_TxHeaderTypeDef header;
    if (frame == NULL || frame->identifier > CAN_STANDARD_MAX_IDENTIFIER ||
        (guard_state >= 2U && (kind != ADRC_CAN_DISABLE || !valid_acquire_disable(frame))) ||
        (unsigned)kind > (unsigned)ADRC_CAN_WATCHDOG_PARAMETER ||
        (kind == ADRC_CAN_WATCHDOG_PARAMETER &&
         (!valid_watchdog_parameter(frame) || decoded_torque_nm != 0.0F)) ||
        (kind == ADRC_CAN_PROBE &&
         ((valid_feedback_query(frame) == 0U && valid_acquire_disable(frame) == 0U &&
           valid_acquire_neutral_mit(frame) == 0U) ||
          decoded_torque_nm != 0.0F)) ||
        (kind != ADRC_CAN_PROBE && kind != ADRC_CAN_WATCHDOG_PARAMETER && frame->length != 8U) ||
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

/** @brief Prevents a deadline interrupt between the admission check and a later ENABLE request. */
uint8_t stm32_adrc_channel_submit(const CanFrame *frame, AdrcCanKind kind, float decoded_torque_nm)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    uint8_t accepted;
    __disable_irq();
    accepted = submit_frame(frame, kind, decoded_torque_nm);
    __set_PRIMASK(interrupt_mask);
    return accepted;
}
