/** @file idle_polling_budget_test.c
 * @brief Measure the real idle register scheduler with synthetic 1ms responses; no hardware traffic.
 */
#define main existing_integrated_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Count 4-byte requests and 8-byte responses while proving each register remains younger than 500ms. */
int main(void)
{
    uint64_t now_us, previous[2] = {0U}, maximum_gap = 0U;
    unsigned requests = 0U, responses = 0U; CanFrame frame; CanTxPriority priority;
    memset(&application_persistence, 0, sizeof(application_persistence));
    aethor_app_init(1U, 1234U); fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    for (now_us = 1000000U; now_us < 11000000U; now_us += 4000U) {
        if (aethor_app_next_can_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY) {
            uint8_t payload[8] = {7U, 0U, 0x33U, 0U, 0U, 0U, 0U, 0U};
            unsigned register_index; float position = 1.0F;
            assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[2] == 0x33U);
            assert(priority == CAN_TX_PRIORITY_FEEDBACK_QUERY);
            register_index = frame.data[3] - 0x50U; assert(register_index < 2U);
            if (previous[register_index]) {
                uint64_t gap = now_us - previous[register_index];
                if (gap > maximum_gap) maximum_gap = gap;
                assert(gap < 500000ULL);
            }
            previous[register_index] = now_us; ++requests;
            payload[3] = frame.data[3]; memcpy(payload + 4U, &position, sizeof(position));
            assert(can_frame_init(&frame, 23U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
            assert(aethor_app_receive_can_frame(&frame, now_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
            ++responses;
        }
    }
    assert(requests == 87U && responses == requests && maximum_gap == 232000ULL);
    assert(application_motor_runtime.accepted_feedback_count == 0U);
    printf("IDLE_POLLING_BUDGET_PASS duration_s=10 request_frames=%u response_frames=%u request_payload=4 response_payload=8 max_register_gap_us=%llu normal_control_feedback=0\n",
        requests, responses, (unsigned long long)maximum_gap);
    return 0;
}
