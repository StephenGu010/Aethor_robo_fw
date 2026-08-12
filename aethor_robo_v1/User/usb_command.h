/**
 * @file usb_command.h
 * @brief Strict parser definitions for the USB CDC text control protocol.
 */

#ifndef USB_COMMAND_H
#define USB_COMMAND_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_COMMAND_JOINT_COUNT 7U
#define USB_COMMAND_ECHO_CAPACITY 96U

/** @brief USB command parser result codes. */
typedef enum
{
    USB_COMMAND_STATUS_OK = 0,
    USB_COMMAND_STATUS_INVALID_ARGUMENT,
    USB_COMMAND_STATUS_UNKNOWN_COMMAND,
    USB_COMMAND_STATUS_INVALID_ARGUMENT_COUNT,
    USB_COMMAND_STATUS_INVALID_NUMBER,
    USB_COMMAND_STATUS_OUT_OF_RANGE,
    USB_COMMAND_STATUS_UNSUPPORTED_COMMAND,
    USB_COMMAND_STATUS_RESPONSE_TOO_SMALL
} UsbCommandStatus;

/** @brief Commands accepted from the USB CDC text channel. */
typedef enum
{
    USB_COMMAND_TYPE_NONE = 0,
    USB_COMMAND_TYPE_PING,
    USB_COMMAND_TYPE_ECHO,
    USB_COMMAND_TYPE_SELECT,
    USB_COMMAND_TYPE_START,
    USB_COMMAND_TYPE_STOP,
    USB_COMMAND_TYPE_DISABLE,
    USB_COMMAND_TYPE_HOME,
    USB_COMMAND_TYPE_MOVE_JOINTS,
    USB_COMMAND_TYPE_GET_JOINT_POSITIONS,
    USB_COMMAND_TYPE_GET_MOTOR_POSITIONS,
    USB_COMMAND_TYPE_GET_STATE,
    USB_COMMAND_TYPE_GET_ENABLE,
    USB_COMMAND_TYPE_GET_CAPABILITIES,
    USB_COMMAND_TYPE_GET_DH,
    USB_COMMAND_TYPE_GET_CONFIG
} UsbCommandType;

/** @brief Compatibility behavior selected by the joint-move prefix. */
typedef enum
{
    USB_MOVE_BEHAVIOR_SEQUENTIAL = 0,
    USB_MOVE_BEHAVIOR_INTERRUPTABLE
} UsbMoveBehavior;

/** @brief Parsed command and its validated parameters. */
typedef struct
{
    UsbCommandType type;
    uint8_t selected_joint;
    uint8_t query_joint;
    UsbMoveBehavior move_behavior;
    float joint_degrees[USB_COMMAND_JOINT_COUNT];
    float speed_percent;
    char echo_text[USB_COMMAND_ECHO_CAPACITY];
} UsbCommand;

UsbCommandStatus usb_command_process_line(const char *line,
                                          UsbCommand *command,
                                          char *response,
                                          size_t response_capacity);
uint8_t usb_command_is_allowed_in_key_control(UsbCommandType command_type);

#ifdef __cplusplus
}
#endif

#endif /* USB_COMMAND_H */
