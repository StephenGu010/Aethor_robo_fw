/**
 * @file platform_io_test_main.c
 * @brief Host-side tests for ISR-to-task CAN and USB platform queues.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can_rx_inbox.h"
#include "monotonic_time.h"
#include "usb_cdc_stream.h"

static UsbCdcStreamTransmitResult simulated_usb_result;
static uint32_t simulated_usb_call_count;
static uint8_t simulated_usb_last_data[USB_CDC_STREAM_MESSAGE_CAPACITY];
static uint16_t simulated_usb_last_length;

/**
 * @brief Simulates the non-blocking STM32 USB CDC transmit entry point.
 */
static UsbCdcStreamTransmitResult simulated_usb_transmit(const uint8_t *data,
                                                         uint16_t length)
{
    assert(data != NULL);
    assert(length <= sizeof(simulated_usb_last_data));
    memcpy(simulated_usb_last_data, data, length);
    simulated_usb_last_length = length;
    ++simulated_usb_call_count;
    return simulated_usb_result;
}

/**
 * @brief Verifies the CAN inbox is bounded, FIFO ordered, and latches Bus-Off.
 */
static void test_can_rx_inbox(void)
{
    CanRxInbox inbox;
    CanFrame frame;
    CanFrame popped_frame;
    uint8_t payload[8] = {0x42U};
    uint8_t frame_index;

    can_rx_inbox_init(&inbox);
    assert(CAN_RX_INBOX_CAPACITY == 32U);
    assert(can_frame_init(&frame, 0x11U, payload, sizeof(payload)) ==
           CAN_FRAME_STATUS_OK);

    for (frame_index = 0U; frame_index < CAN_RX_INBOX_CAPACITY; ++frame_index)
    {
        frame.data[0] = frame_index;
        assert(can_rx_inbox_push_isr(&inbox, &frame) == CAN_RX_INBOX_STATUS_OK);
    }
    assert(can_rx_inbox_push_isr(&inbox, &frame) == CAN_RX_INBOX_STATUS_FULL);
    assert(inbox.dropped_frame_count == 1U);

    for (frame_index = 0U; frame_index < CAN_RX_INBOX_CAPACITY; ++frame_index)
    {
        assert(can_rx_inbox_pop(&inbox, &popped_frame) == CAN_RX_INBOX_STATUS_OK);
        assert(popped_frame.data[0] == frame_index);
    }
    assert(can_rx_inbox_pop(&inbox, &popped_frame) == CAN_RX_INBOX_STATUS_EMPTY);

    can_rx_inbox_latch_bus_off_isr(&inbox);
    assert(inbox.bus_off_latched != 0U);
    assert(inbox.bus_off_event_count == 1U);
}

/**
 * @brief Verifies USB packets assemble into strict task-context protocol lines.
 */
static void test_usb_receive_line_assembly(void)
{
    static const uint8_t first_packet[] = "REQ 1 GET_";
    static const uint8_t second_packet[] = "INFO *1234\r\nREQ 2 GET_STATE *5678\n";
    UsbCdcStream stream;
    char line[USB_CDC_STREAM_LINE_CAPACITY];
    uint16_t line_length;

    usb_cdc_stream_init(&stream, simulated_usb_transmit);
    assert(usb_cdc_stream_receive_isr(&stream,
                                      first_packet,
                                      sizeof(first_packet) - 1U) ==
           USB_CDC_STREAM_STATUS_OK);
    assert(usb_cdc_stream_receive_isr(&stream,
                                      second_packet,
                                      sizeof(second_packet) - 1U) ==
           USB_CDC_STREAM_STATUS_OK);
    assert(usb_cdc_stream_next_line(&stream, line, sizeof(line), &line_length) ==
           USB_CDC_STREAM_STATUS_OK);
    assert(strcmp(line, "REQ 1 GET_INFO *1234") == 0);
    assert(line_length == strlen(line));
    assert(usb_cdc_stream_next_line(&stream, line, sizeof(line), &line_length) ==
           USB_CDC_STREAM_STATUS_OK);
    assert(strcmp(line, "REQ 2 GET_STATE *5678") == 0);
    assert(usb_cdc_stream_next_line(&stream, line, sizeof(line), &line_length) ==
           USB_CDC_STREAM_STATUS_EMPTY);
}

/**
 * @brief Verifies in-flight response storage remains stable until completion.
 */
static void test_usb_transmit_lifetime_and_priority(void)
{
    static const uint8_t response_one[] = "RSP 1 ok *0000\n";
    static const uint8_t response_two[] = "RSP 2 ok *0000\n";
    static const uint8_t telemetry_one[] = "TEL 1 JOINT_STATE *0000\n";
    static const uint8_t telemetry_two[] = "TEL 2 JOINT_STATE *0000\n";
    static const uint8_t telemetry_three[] = "TEL 3 JOINT_STATE *0000\n";
    static const uint8_t telemetry_four[] = "TEL 4 JOINT_STATE *0000\n";
    static const uint8_t telemetry_five[] = "TEL 5 JOINT_STATE *0000\n";
    UsbCdcStream stream;

    simulated_usb_result = USB_CDC_STREAM_TRANSMIT_OK;
    simulated_usb_call_count = 0U;
    simulated_usb_last_length = 0U;
    usb_cdc_stream_init(&stream, simulated_usb_transmit);
    assert(USB_CDC_STREAM_HIGH_PRIORITY_CAPACITY == 16U);
    assert(USB_CDC_STREAM_QUERY_CAPACITY == 16U);
    assert(USB_CDC_STREAM_TELEMETRY_CAPACITY == 4U);

    assert(usb_cdc_stream_queue_telemetry(&stream,
                                          telemetry_one,
                                          sizeof(telemetry_one) - 1U) ==
           USB_CDC_STREAM_STATUS_OK);
    assert(usb_cdc_stream_queue_telemetry(&stream,
                                          telemetry_two,
                                          sizeof(telemetry_two) - 1U) ==
           USB_CDC_STREAM_STATUS_OK);
    assert(usb_cdc_stream_queue_telemetry(&stream,
                                          telemetry_three,
                                          sizeof(telemetry_three) - 1U) ==
           USB_CDC_STREAM_STATUS_OK);
    assert(usb_cdc_stream_queue_telemetry(&stream,
                                          telemetry_four,
                                          sizeof(telemetry_four) - 1U) ==
           USB_CDC_STREAM_STATUS_OK);
    assert(usb_cdc_stream_queue_telemetry(&stream,
                                          telemetry_five,
                                          sizeof(telemetry_five) - 1U) ==
           USB_CDC_STREAM_STATUS_REPLACED);
    assert(usb_cdc_stream_queue_query(&stream,
                                      response_two,
                                      sizeof(response_two) - 1U) ==
           USB_CDC_STREAM_STATUS_OK);
    assert(usb_cdc_stream_queue_high_priority(&stream,
                                              response_one,
                                              sizeof(response_one) - 1U) ==
           USB_CDC_STREAM_STATUS_OK);

    usb_cdc_stream_service_tx(&stream);
    assert(simulated_usb_call_count == 1U);
    assert(simulated_usb_last_length == sizeof(response_one) - 1U);
    assert(memcmp(simulated_usb_last_data,
                  response_one,
                  sizeof(response_one) - 1U) == 0);
    assert(stream.tx_in_flight != 0U);

    usb_cdc_stream_service_tx(&stream);
    assert(simulated_usb_call_count == 1U);
    usb_cdc_stream_on_tx_complete_isr(&stream);
    usb_cdc_stream_service_tx(&stream);
    assert(simulated_usb_call_count == 2U);
    assert(memcmp(simulated_usb_last_data,
                  response_two,
                  sizeof(response_two) - 1U) == 0);

    usb_cdc_stream_on_tx_complete_isr(&stream);
    usb_cdc_stream_service_tx(&stream);
    assert(simulated_usb_call_count == 3U);
    assert(memcmp(simulated_usb_last_data,
                  telemetry_two,
                  sizeof(telemetry_two) - 1U) == 0);
    assert(stream.telemetry_replaced_count == 1U);
}

/**
 * @brief Verifies BUSY retries do not dequeue or mutate the response.
 */
static void test_usb_busy_retry(void)
{
    static const uint8_t response[] = "ERR 1 BUSY *0000\n";
    UsbCdcStream stream;

    simulated_usb_result = USB_CDC_STREAM_TRANSMIT_BUSY;
    simulated_usb_call_count = 0U;
    usb_cdc_stream_init(&stream, simulated_usb_transmit);
    assert(usb_cdc_stream_queue_high_priority(&stream,
                                              response,
                                              sizeof(response) - 1U) ==
           USB_CDC_STREAM_STATUS_OK);
    usb_cdc_stream_service_tx(&stream);
    assert(stream.tx_in_flight == 0U);
    assert(stream.transmit_busy_count == 1U);

    simulated_usb_result = USB_CDC_STREAM_TRANSMIT_OK;
    usb_cdc_stream_service_tx(&stream);
    assert(stream.tx_in_flight != 0U);
    assert(memcmp(stream.active_message.data, response, sizeof(response) - 1U) == 0);
}

/**
 * @brief Verifies a 32-bit millisecond tick extends monotonically across wraparound.
 */
static void test_monotonic_time_extends_hal_tick_wraparound(void)
{
    AethorMonotonicTimeState time_state = {0};
    uint64_t timestamp_us;

    timestamp_us = aethor_monotonic_time_update(&time_state, 0xFFFFFFFEUL);
    assert(timestamp_us == 0xFFFFFFFEULL * 1000ULL);
    timestamp_us = aethor_monotonic_time_update(&time_state, 0xFFFFFFFFUL);
    assert(timestamp_us == (0xFFFFFFFEULL * 1000ULL) + 1000ULL);
    timestamp_us = aethor_monotonic_time_update(&time_state, 0U);
    assert(timestamp_us == (0xFFFFFFFEULL * 1000ULL) + 2000ULL);
    timestamp_us = aethor_monotonic_time_update(&time_state, 1U);
    assert(timestamp_us == (0xFFFFFFFEULL * 1000ULL) + 3000ULL);
}

/**
 * @brief Verifies ordinary and repeated ticks never move extended time backward.
 */
static void test_monotonic_time_handles_increment_and_repeated_tick(void)
{
    AethorMonotonicTimeState time_state = {0};

    assert(aethor_monotonic_time_update(&time_state, 100U) == 100000ULL);
    assert(aethor_monotonic_time_update(&time_state, 125U) == 125000ULL);
    assert(aethor_monotonic_time_update(&time_state, 125U) == 125000ULL);
    assert(aethor_monotonic_time_update(&time_state, 126U) == 126000ULL);
}

/**
 * @brief Runs all ISR-to-task platform queue tests.
 */
int main(void)
{
    test_can_rx_inbox();
    test_usb_receive_line_assembly();
    test_usb_transmit_lifetime_and_priority();
    test_usb_busy_retry();
    test_monotonic_time_extends_hal_tick_wraparound();
    test_monotonic_time_handles_increment_and_repeated_tick();
    puts("PLATFORM_IO_TESTS_PASSED");
    return 0;
}
