/** @file persistence_usb_link_test.c
 * @brief Exercises maintained CDC callbacks with synthetic USB buffers; no device access. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define CDC_PERSISTENCE_LINK_CONFIGURED (0x80000000UL)
#define CDC_PERSISTENCE_LINK_GENERATION_MASK (0x7fffffffUL)
#define AETHOR_MOTOR_PERSISTENCE_TRANSPORT (1)
#define USBD_OK (0)
#define USBD_STATE_CONFIGURED (3U)
#define USBD_STATE_SUSPENDED (4U)
#define SCB_SCR_SLEEPDEEP_Msk (4U)
#define SCB_SCR_SLEEPONEXIT_Msk (2U)
/** @brief Minimal callback argument, not a USB stack or transport simulation. */
typedef struct { uint8_t dev_state, dev_old_state; } USBD_HandleTypeDef;
/** @brief Offline PCD callback inputs preserve the existing low-power branch. */
typedef struct { void *pData; struct { uint8_t low_power_enable; } Init; } PCD_HandleTypeDef;
/** @brief Minimal SCB register used only by the maintained suspend callback. */
static struct { uint32_t SCR; } fake_scb;
#define SCB (&fake_scb)
typedef int USBD_StatusTypeDef;
static USBD_HandleTypeDef hUsbDeviceHS;
static uint8_t UserTxBufferHS[1], UserRxBufferHS[1];
static unsigned transmit_buffer_calls, receive_buffer_calls;
static unsigned phy_clock_gate_calls;
#define __HAL_PCD_GATE_PHYCLOCK(device) ((void)(device), ++phy_clock_gate_calls)
/** @brief Retains the unchanged CDC buffer-registration side effect for assertions. */
static void USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *device, uint8_t *buffer, uint32_t length)
{ assert(device == &hUsbDeviceHS && buffer == UserTxBufferHS && !length); ++transmit_buffer_calls; }
/** @brief Retains the unchanged CDC receive-buffer registration for assertions. */
static void USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *device, uint8_t *buffer)
{ assert(device == &hUsbDeviceHS && buffer == UserRxBufferHS); ++receive_buffer_calls; }
#include "persistence_usb_callbacks.inc"
/** @brief Proves reset/deinit/reinit cannot reuse an earlier configured maintenance binding. */
int main(void)
{
    uint32_t first, disconnected, reconnected, reinitialized;
    PCD_HandleTypeDef pcd = { &hUsbDeviceHS, {0U} };
    assert(!CDC_PersistenceLinkState_HS());
    assert(CDC_Init_HS() == USBD_OK);
    first = CDC_PersistenceLinkState_HS();
    assert(first & CDC_PERSISTENCE_LINK_CONFIGURED);
    assert(first & CDC_PERSISTENCE_LINK_GENERATION_MASK);
    assert(CDC_PersistenceLinkState_HS() == first);
    assert(transmit_buffer_calls == 1U && receive_buffer_calls == 1U);
    assert(CDC_DeInit_HS() == USBD_OK);
    disconnected = CDC_PersistenceLinkState_HS();
    assert(!(disconnected & CDC_PERSISTENCE_LINK_CONFIGURED));
    assert(disconnected && disconnected != first);
    assert(CDC_Init_HS() == USBD_OK);
    reconnected = CDC_PersistenceLinkState_HS();
    assert(reconnected & CDC_PERSISTENCE_LINK_CONFIGURED);
    assert(reconnected != first && reconnected != disconnected);
    assert(CDC_Init_HS() == USBD_OK);
    reinitialized = CDC_PersistenceLinkState_HS();
    assert(reinitialized & CDC_PERSISTENCE_LINK_CONFIGURED);
    assert(reinitialized != reconnected);
    assert(CDC_DeInit_HS() == USBD_OK && CDC_DeInit_HS() == USBD_OK);
    assert(!(CDC_PersistenceLinkState_HS() & CDC_PERSISTENCE_LINK_CONFIGURED));
    assert(transmit_buffer_calls == 3U && receive_buffer_calls == 3U);
    assert(CDC_Init_HS() == USBD_OK);
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    first = CDC_PersistenceLinkState_HS();
#if USE_HAL_PCD_REGISTER_CALLBACKS == 1
    PCD_SuspendCallback(&pcd);
#else
    HAL_PCD_SuspendCallback(&pcd);
#endif
    disconnected = CDC_PersistenceLinkState_HS();
    assert(disconnected != first && !(disconnected & CDC_PERSISTENCE_LINK_CONFIGURED));
    assert(hUsbDeviceHS.dev_state == USBD_STATE_SUSPENDED && phy_clock_gate_calls == 1U);
#if USE_HAL_PCD_REGISTER_CALLBACKS == 1
    PCD_ResumeCallback(&pcd);
#else
    HAL_PCD_ResumeCallback(&pcd);
#endif
    reconnected = CDC_PersistenceLinkState_HS();
    assert(reconnected != first && reconnected != disconnected);
    assert(reconnected & CDC_PERSISTENCE_LINK_CONFIGURED);
    assert(hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED);
    /* Resume of an unconfigured device cannot manufacture a configured link. */
    hUsbDeviceHS.dev_state = 0U;
    pcd.Init.low_power_enable = 1U;
#if USE_HAL_PCD_REGISTER_CALLBACKS == 1
    PCD_SuspendCallback(&pcd);
    PCD_ResumeCallback(&pcd);
#else
    HAL_PCD_SuspendCallback(&pcd);
    HAL_PCD_ResumeCallback(&pcd);
#endif
    assert(!(CDC_PersistenceLinkState_HS() & CDC_PERSISTENCE_LINK_CONFIGURED));
    assert(fake_scb.SCR == (SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    assert(phy_clock_gate_calls == 2U);
    puts("PERSISTENCE_USB_CDC_PCD_CALLBACK_PASS init_deinit_suspend_resume_reconnect_binding buffer_and_power_path_preserved=1 synthetic_only=1");
    return 0;
}
