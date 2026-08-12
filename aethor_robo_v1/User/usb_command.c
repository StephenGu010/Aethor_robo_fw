/**
 * @file usb_command.c
 * @brief Strict, allocation-free parser for the USB CDC text control protocol.
 */

#include "usb_command.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief Write a bounded protocol response.
 * @param response Destination response buffer.
 * @param response_capacity Destination capacity in bytes.
 * @param text Null-terminated response text.
 * @return USB command status describing whether the response fitted.
 */
static UsbCommandStatus usb_command_write_response(char *response,
                                                   size_t response_capacity,
                                                   const char *text)
{
    size_t text_length;

    if ((response == NULL) || (text == NULL) || (response_capacity == 0U))
    {
        return USB_COMMAND_STATUS_INVALID_ARGUMENT;
    }

    text_length = strlen(text);
    if (text_length >= response_capacity)
    {
        response[0] = '\0';
        return USB_COMMAND_STATUS_RESPONSE_TOO_SMALL;
    }

    memcpy(response, text, text_length + 1U);
    return USB_COMMAND_STATUS_OK;
}

/**
 * @brief Parse one finite floating-point token and advance to its delimiter.
 * @param cursor Address of the current token cursor.
 * @param value Receives the parsed value.
 * @return USB command status for the token.
 */
static UsbCommandStatus usb_command_parse_float_token(const char **cursor, float *value)
{
    char *end_pointer;
    float parsed_value;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL))
    {
        return USB_COMMAND_STATUS_INVALID_ARGUMENT;
    }

    errno = 0;
    end_pointer = NULL;
    parsed_value = strtof(*cursor, &end_pointer);
    if ((end_pointer == *cursor) || (errno == ERANGE) || (!isfinite(parsed_value)))
    {
        return USB_COMMAND_STATUS_INVALID_NUMBER;
    }

    while ((*end_pointer == ' ') || (*end_pointer == '\t'))
    {
        ++end_pointer;
    }

    *value = parsed_value;
    *cursor = end_pointer;
    return USB_COMMAND_STATUS_OK;
}

/**
 * @brief Parse the seven joint targets and global speed percentage.
 * @param arguments Comma-separated numeric argument text.
 * @param command Command structure to populate.
 * @return USB command status for the complete argument list.
 */
static UsbCommandStatus usb_command_parse_move(const char *arguments,
                                               UsbMoveBehavior move_behavior,
                                               UsbCommand *command)
{
    const char *cursor = arguments;
    float parsed_values[USB_COMMAND_JOINT_COUNT];
    uint32_t value_index;

    for (value_index = 0U; value_index < USB_COMMAND_JOINT_COUNT; ++value_index)
    {
        UsbCommandStatus parse_status =
            usb_command_parse_float_token(&cursor, &parsed_values[value_index]);

        if (parse_status != USB_COMMAND_STATUS_OK)
        {
            return parse_status;
        }

        if (value_index < (USB_COMMAND_JOINT_COUNT - 1U))
        {
            if (*cursor != ',')
            {
                return USB_COMMAND_STATUS_INVALID_ARGUMENT_COUNT;
            }
            ++cursor;
        }
    }

    command->speed_percent = 100.0F;
    if (*cursor == ',')
    {
        UsbCommandStatus speed_status;

        ++cursor;
        speed_status = usb_command_parse_float_token(&cursor, &command->speed_percent);
        if (speed_status != USB_COMMAND_STATUS_OK)
        {
            return speed_status;
        }
    }
    if (*cursor != '\0')
    {
        return USB_COMMAND_STATUS_INVALID_ARGUMENT_COUNT;
    }

    if ((command->speed_percent < 1.0F) || (command->speed_percent > 100.0F))
    {
        return USB_COMMAND_STATUS_OUT_OF_RANGE;
    }

    memcpy(command->joint_degrees,
           parsed_values,
           sizeof(command->joint_degrees));
    command->move_behavior = move_behavior;
    command->type = USB_COMMAND_TYPE_MOVE_JOINTS;
    return USB_COMMAND_STATUS_OK;
}

/**
 * @brief Parses an optional joint number following a fixed query command.
 * @param arguments Empty text or one joint number in the range 1 through 7.
 * @param command_type Query command type.
 * @param command Command structure to populate.
 * @return USB command status for the optional selector.
 */
static UsbCommandStatus usb_command_parse_optional_joint_query(
    const char *arguments,
    UsbCommandType command_type,
    UsbCommand *command)
{
    char *end_pointer;
    unsigned long query_joint;

    while ((*arguments == ' ') || (*arguments == '\t'))
    {
        ++arguments;
    }
    if (*arguments == '\0')
    {
        command->type = command_type;
        command->query_joint = 0U;
        return USB_COMMAND_STATUS_OK;
    }

    errno = 0;
    end_pointer = NULL;
    query_joint = strtoul(arguments, &end_pointer, 10);
    while ((*end_pointer == ' ') || (*end_pointer == '\t'))
    {
        ++end_pointer;
    }
    if ((end_pointer == arguments) || (errno == ERANGE) || (*end_pointer != '\0'))
    {
        return USB_COMMAND_STATUS_INVALID_NUMBER;
    }
    if ((query_joint < 1UL) || (query_joint > USB_COMMAND_JOINT_COUNT))
    {
        return USB_COMMAND_STATUS_OUT_OF_RANGE;
    }

    command->type = command_type;
    command->query_joint = (uint8_t)query_joint;
    return USB_COMMAND_STATUS_OK;
}

/**
 * @brief Parse a joint selector in the inclusive range 1 through 7.
 * @param arguments Selector argument text.
 * @param command Command structure to populate.
 * @return USB command status for the selector.
 */
static UsbCommandStatus usb_command_parse_select(const char *arguments, UsbCommand *command)
{
    char *end_pointer;
    unsigned long selected_joint;

    if (*arguments == '\0')
    {
        return USB_COMMAND_STATUS_INVALID_ARGUMENT_COUNT;
    }

    errno = 0;
    end_pointer = NULL;
    selected_joint = strtoul(arguments, &end_pointer, 10);
    if ((end_pointer == arguments) || (errno == ERANGE))
    {
        return USB_COMMAND_STATUS_INVALID_NUMBER;
    }

    while ((*end_pointer == ' ') || (*end_pointer == '\t'))
    {
        ++end_pointer;
    }
    if (*end_pointer != '\0')
    {
        return USB_COMMAND_STATUS_INVALID_ARGUMENT_COUNT;
    }

    if ((selected_joint < 1UL) || (selected_joint > USB_COMMAND_JOINT_COUNT))
    {
        return USB_COMMAND_STATUS_OUT_OF_RANGE;
    }

    command->selected_joint = (uint8_t)selected_joint;
    command->type = USB_COMMAND_TYPE_SELECT;
    return USB_COMMAND_STATUS_OK;
}

/**
 * @brief Parse and validate one complete USB CDC protocol line.
 * @param line Null-terminated input without a required CR or LF terminator.
 * @param command Receives the parsed command.
 * @param response Receives an immediate parser response when applicable.
 * @param response_capacity Response buffer capacity in bytes.
 * @return Explicit USB command parser status.
 */
UsbCommandStatus usb_command_process_line(const char *line,
                                          UsbCommand *command,
                                          char *response,
                                          size_t response_capacity)
{
    size_t line_length;

    if ((line == NULL) || (command == NULL) || (response == NULL) ||
        (response_capacity == 0U))
    {
        return USB_COMMAND_STATUS_INVALID_ARGUMENT;
    }

    memset(command, 0, sizeof(*command));
    response[0] = '\0';
    line_length = strlen(line);
    while ((line_length > 0U) &&
           ((line[line_length - 1U] == '\r') || (line[line_length - 1U] == '\n')))
    {
        --line_length;
    }

    if ((line_length == 5U) && (strncmp(line, "#PING", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_PING;
        return usb_command_write_response(response, response_capacity, "ok PONG");
    }

    if ((line_length >= 6U) && (strncmp(line, "#ECHO ", 6U) == 0))
    {
        size_t echo_length = line_length - 6U;
        int written_length;

        if (echo_length >= sizeof(command->echo_text))
        {
            return USB_COMMAND_STATUS_OUT_OF_RANGE;
        }
        memcpy(command->echo_text, &line[6], echo_length);
        command->echo_text[echo_length] = '\0';
        command->type = USB_COMMAND_TYPE_ECHO;
        written_length = snprintf(response, response_capacity, "ok %s", command->echo_text);
        if ((written_length < 0) || ((size_t)written_length >= response_capacity))
        {
            response[0] = '\0';
            return USB_COMMAND_STATUS_RESPONSE_TOO_SMALL;
        }
        return USB_COMMAND_STATUS_OK;
    }

    if ((line_length >= 8U) && (strncmp(line, "#SELECT ", 8U) == 0))
    {
        return usb_command_parse_select(&line[8], command);
    }

    if ((line_length == 6U) && (strncmp(line, "!START", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_START;
        return USB_COMMAND_STATUS_OK;
    }
    if ((line_length == 5U) && (strncmp(line, "!STOP", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_STOP;
        return USB_COMMAND_STATUS_OK;
    }
    if ((line_length == 8U) && (strncmp(line, "!DISABLE", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_DISABLE;
        return USB_COMMAND_STATUS_OK;
    }
    if ((line_length == 5U) && (strncmp(line, "!HOME", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_HOME;
        return USB_COMMAND_STATUS_OK;
    }
    if ((line_length == 8U) && (strncmp(line, "#GETJPOS", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_GET_JOINT_POSITIONS;
        return USB_COMMAND_STATUS_OK;
    }
    if ((line_length == 8U) && (strncmp(line, "#GETMPOS", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_GET_MOTOR_POSITIONS;
        return USB_COMMAND_STATUS_OK;
    }
    if ((line_length == 9U) && (strncmp(line, "#GETSTATE", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_GET_STATE;
        return USB_COMMAND_STATUS_OK;
    }
    if ((line_length == 10U) && (strncmp(line, "#GETENABLE", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_GET_ENABLE;
        return USB_COMMAND_STATUS_OK;
    }
    if ((line_length == 8U) && (strncmp(line, "#GETCAPS", line_length) == 0))
    {
        command->type = USB_COMMAND_TYPE_GET_CAPABILITIES;
        return USB_COMMAND_STATUS_OK;
    }
    if ((line_length >= 6U) && (strncmp(line, "#GETDH", 6U) == 0))
    {
        return usb_command_parse_optional_joint_query(&line[6],
                                                      USB_COMMAND_TYPE_GET_DH,
                                                      command);
    }
    if ((line_length >= 10U) && (strncmp(line, "#GETCONFIG", 10U) == 0))
    {
        return usb_command_parse_optional_joint_query(&line[10],
                                                      USB_COMMAND_TYPE_GET_CONFIG,
                                                      command);
    }
    if ((line_length > 1U) && ((line[0] == '>') || (line[0] == '&')))
    {
        return usb_command_parse_move(&line[1],
                                      (line[0] == '>')
                                          ? USB_MOVE_BEHAVIOR_SEQUENTIAL
                                          : USB_MOVE_BEHAVIOR_INTERRUPTABLE,
                                      command);
    }
    if ((line_length > 0U) &&
        ((line[0] == '@') ||
         (strncmp(line, "!RGB", 4U) == 0) ||
         (strncmp(line, "!LED", 4U) == 0) ||
         (strncmp(line, "#RGB", 4U) == 0) ||
         (strncmp(line, "#GETLPOS", 8U) == 0)))
    {
        return USB_COMMAND_STATUS_UNSUPPORTED_COMMAND;
    }

    return USB_COMMAND_STATUS_UNKNOWN_COMMAND;
}
