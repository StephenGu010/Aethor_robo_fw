/**
 * @file bsp_fdcan.h
 * @brief STM32H7 FDCAN1 classic-CAN adapter for S3519 motor communication.
 */

#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include <stdint.h>

#include "fdcan.h"
#include "fdcan_classic_codec.h"
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define hcan_t FDCAN_HandleTypeDef

/** @brief Explicit results returned by the HAL-backed classic CAN adapter. */
typedef enum
{
    FDCAN_DRIVER_STATUS_OK = 0,
    FDCAN_DRIVER_STATUS_INVALID_ARGUMENT,
    FDCAN_DRIVER_STATUS_INVALID_FRAME,
    FDCAN_DRIVER_STATUS_NOT_READY,
    FDCAN_DRIVER_STATUS_TX_QUEUE_FULL,
    FDCAN_DRIVER_STATUS_HAL_ERROR,
    FDCAN_DRIVER_STATUS_NO_MESSAGE,
    FDCAN_DRIVER_STATUS_BUS_OFF
} FdcanDriverStatus;

/** @brief Latched CAN health and error-event counters. */
typedef struct
{
    uint8_t started;
    uint8_t warning;
    uint8_t error_passive;
    uint8_t bus_off_latched;
    uint32_t warning_event_count;
    uint32_t error_passive_event_count;
    uint32_t bus_off_event_count;
    uint32_t receive_error_count;
    uint32_t transmit_error_count;
} FdcanDriverState;

FdcanDriverStatus bsp_can_init(const RobotConfiguration *configuration);
FdcanDriverStatus can_filter_init(const RobotConfiguration *configuration);
FdcanDriverStatus fdcan_classic_send(FDCAN_HandleTypeDef *fdcan_handle,
                                     const FdcanClassicFrame *frame);
FdcanDriverStatus fdcan_classic_receive(FDCAN_HandleTypeDef *fdcan_handle,
                                        FdcanClassicFrame *frame);
const FdcanDriverState *fdcan_classic_get_state(void);
void fdcan_classic_frame_received(const FdcanClassicFrame *frame);
void fdcan_classic_bus_off_received(void);

uint8_t fdcanx_send_data(hcan_t *fdcan_handle, uint16_t identifier,
                         uint8_t *data, uint32_t length);
uint8_t fdcanx_receive(hcan_t *fdcan_handle, uint16_t *received_identifier,
                       uint8_t *buffer);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FDCAN_H */
