/** @file motor_capability_collector_test_main.c
 * @brief Deterministic fixed-read collector contract; fixtures are not motor evidence. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_capability_collector.h"
#include "can_rx_inbox.h"
#include "monotonic_time.h"

/** @brief Creates the full two-byte ESC response with raw IEEE754 value. */
static CanFrame response(uint8_t rid, float value)
{
    CanFrame frame;
    uint32_t bits;
    unsigned int byte_index;
    memset(&frame, 0, sizeof(frame));
    frame.identifier = 23U; frame.length = 8U;
    frame.data[0] = 7U; frame.data[2] = 0x33U; frame.data[3] = rid;
    memcpy(&bits, &value, sizeof(bits));
    for (byte_index = 0U; byte_index < 4U; ++byte_index)
    { frame.data[4U + byte_index] = (uint8_t)(bits >> (8U * byte_index)); }
    return frame;
}

/** @brief Starts a fresh synthetic one-shot collection and produces its first read. */
static CanFrame start(MotorCapabilityCollector *collector)
{
    CanFrame request;
    motor_capability_init(collector);
    assert(motor_capability_start(collector, 99U, 1U, 1000ULL));
    assert(!motor_capability_next(collector, 1U, 1000ULL, &request));
    assert(!motor_capability_next(collector, 1U, 1000ULL, &request));
    motor_capability_observe_rx_boundary(collector, 1001ULL);
    assert(motor_capability_next(collector, 1U, 1001ULL, &request));
    assert(request.identifier == 0x7FFU && request.length == 4U);
    assert(request.data[0] == 7U && request.data[1] == 0U && request.data[2] == 0x33U);
    return request;
}

/** @brief Six fixed reads preserve provenance without deriving a model or qualification. */
static void test_complete(void)
{
    MotorCapabilityCollector collector;
    CanFrame request = start(&collector);
    static const float values[6] = {0.1F, 100.0F, 0.5F, -1.0F, 0.0F, 19.2032F};
    unsigned int item_index;
    for (item_index = 0U; item_index < 6U; ++item_index)
    {
        uint64_t now_us = 1001ULL + item_index * 4000ULL;
        CanFrame reply = response(motor_capability_rid((uint8_t)item_index), values[item_index]);
        if (item_index)
        {
            assert(!motor_capability_next(&collector, 1U, now_us - 1ULL, &request));
            motor_capability_observe_rx_boundary(&collector, now_us);
            assert(motor_capability_next(&collector, 1U, now_us, &request));
        }
        assert(motor_capability_report_transmit(&collector, &request, 1U, now_us + 1ULL, 1U));
        assert(motor_capability_accept(&collector, &reply, now_us + 2ULL, now_us + 3ULL));
        assert(collector.items[item_index].valid && collector.items[item_index].value == values[item_index]);
        assert(collector.items[item_index].queue_submitted_us == now_us);
        assert(collector.items[item_index].send_completed_observed_us == now_us + 1ULL);
        assert(collector.items[item_index].rx_lower_us == now_us + 2ULL);
    }
    assert(collector.state == MOTOR_CAPABILITY_COMPLETE && collector.count == 6U);
    assert(!motor_capability_start(&collector, 100U, 2U, 50000ULL));
    motor_capability_abort(&collector, MOTOR_CAPABILITY_REASON_STOP);
    assert(collector.state == MOTOR_CAPABILITY_COMPLETE && collector.reason == MOTOR_CAPABILITY_REASON_NONE);
    motor_capability_revoke_completed(&collector, MOTOR_CAPABILITY_REASON_BINDING);
    assert(collector.state == MOTOR_CAPABILITY_COMPLETE && collector.completed_revoked && collector.count == 6U);
    assert(collector.items[5].valid && collector.items[5].value == values[5]);
    motor_capability_revoke_completed(&collector, MOTOR_CAPABILITY_REASON_STOP);
    assert(collector.reason == MOTOR_CAPABILITY_REASON_BINDING);
}

/** @brief Invalid signatures, bounds and floats never become accepted measurements. */
static void test_rejections(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 12U; ++scenario)
    {
        MotorCapabilityCollector collector;
        CanFrame request = start(&collector);
        CanFrame reply = response(1U, 0.1F);
        assert(motor_capability_report_transmit(&collector, &request, 1U, 1002ULL, 1U));
        if (scenario == 0U) { reply.identifier = 22U; }
        if (scenario == 1U) { reply.data[0] = 6U; }
        if (scenario == 2U) { reply.data[1] = 8U; }
        if (scenario == 3U) { reply.data[2] = 0x55U; }
        if (scenario == 4U) { reply.data[3] = 2U; }
        if (scenario == 5U) { reply.length = 7U; }
        if (scenario == 6U) { reply = response(1U, NAN); }
        if (scenario == 7U) { reply = response(1U, INFINITY); }
        if (scenario == 8U) { reply = response(1U, -0.1F); }
        assert(motor_capability_accept(&collector, &reply,
            scenario == 9U ? 1000ULL : 1003ULL,
            scenario == 10U ? 22000ULL : (scenario == 11U ? 1002ULL : 1004ULL)));
        assert(!collector.items[0].valid && !collector.count);
    }
}

/** @brief A reply may precede completion polling but cannot commit without matched transmission. */
static void test_receipt_and_abort(void)
{
    MotorCapabilityCollector collector;
    CanFrame request = start(&collector);
    CanFrame reply = response(1U, 0.1F);
    CanFrame wrong_request = request;
    assert(motor_capability_accept(&collector, &reply, 1003ULL, 1004ULL));
    assert(!collector.count);
    wrong_request.data[3] = 2U;
    assert(!motor_capability_report_transmit(&collector, &wrong_request, 1U, 1005ULL, 1U));
    assert(!collector.count);
    assert(motor_capability_report_transmit(&collector, &request, 1U, 1005ULL, 1U));
    assert(collector.count == 1U);
    assert(motor_capability_accept(&collector, &reply, 1006ULL, 1007ULL));
    assert(collector.count == 1U);
    motor_capability_abort(&collector, MOTOR_CAPABILITY_REASON_STOP);
    assert(collector.state == MOTOR_CAPABILITY_ABORTED);
    assert(motor_capability_accept(&collector, &reply, 2000ULL, 2001ULL));
    assert(!motor_capability_start(&collector, 100U, 2U, 300000ULL));
    request = start(&collector);
    assert(!motor_capability_next(&collector, 2U, 1002ULL, &reply));
    assert(collector.reason == MOTOR_CAPABILITY_REASON_BINDING);
    request = start(&collector);
    assert(!motor_capability_next(&collector, 1U, 21002ULL, &reply));
    assert(collector.reason == MOTOR_CAPABILITY_REASON_TIMEOUT);
    request = start(&collector);
    assert(motor_capability_report_transmit(&collector, &request, 0U, 1002ULL, 1U));
    assert(collector.reason == MOTOR_CAPABILITY_REASON_TRANSPORT);
    request = start(&collector);
    assert(motor_capability_report_transmit(&collector, &request, 1U, 1002ULL, 1U));
    reply = response(1U, 0.1F);
    assert(motor_capability_accept(&collector, &reply, 1001ULL, 1003ULL));
    assert(collector.count == 1U); /* Same quantized pre-submit boundary is a proven empty-FIFO epoch. */
}

/** @brief Runs collector contracts and confirms the old discovery read whitelist remains closed. */
/** @brief Old TXBTO bits, missing pending proof, cancellation and slot reuse cannot fake completion. */
static void test_fifo_tracker(void)
{
    MotorCapabilityFifoTracker tracker;
    CanFrame frame;
    CanFrame receipt;
    uint8_t succeeded;
    assert(s3519_pack_capability_read(7U, 0U, &frame) == S3519_CODEC_STATUS_OK);
    memset(&tracker, 0, sizeof(tracker));
    assert(motor_capability_fifo_arm(&tracker, &frame, 4U, 4U, 0U));
    assert(!motor_capability_fifo_arm(&tracker, &frame, 8U, 8U, 0U));
    assert(!motor_capability_fifo_collect(&tracker, 4U, 4U, 0U, &receipt, &succeeded));
    assert(!motor_capability_fifo_collect(&tracker, 4U, 0U, 0U, &receipt, &succeeded));
    assert(motor_capability_fifo_collect(&tracker, 0U, 4U, 0U, &receipt, &succeeded));
    assert(succeeded && receipt.data[3] == 1U && !tracker.active);
    assert(!motor_capability_fifo_collect(&tracker, 0U, 4U, 0U, &receipt, &succeeded));
    assert(motor_capability_fifo_arm(&tracker, &frame, 4U, 4U, 4U));
    assert(motor_capability_fifo_collect(&tracker, 0U, 4U, 0U, &receipt, &succeeded));
    assert(!succeeded); /* Stale TXBTO at arm never grants this request a completion. */
    assert(motor_capability_fifo_arm(&tracker, &frame, 4U, 0U, 4U));
    assert(motor_capability_fifo_collect(&tracker, 0U, 4U, 0U, &receipt, &succeeded));
    assert(!succeeded); /* No pending proof is fail-closed even for very fast transmission. */
    assert(motor_capability_fifo_arm(&tracker, &frame, 4U, 4U, 0U));
    assert(motor_capability_fifo_collect(&tracker, 0U, 0U, 4U, &receipt, &succeeded));
    assert(!succeeded);
    frame.data[2] = 0x55U;
    assert(!motor_capability_fifo_arm(&tracker, &frame, 4U, 4U, 0U));
}

/** @brief Exercises Core's real 1ms clock and FIFO-bound policy across two control-task ticks. */
static void test_quantized_clock_and_fifo_boundary(void)
{
    MotorCapabilityCollector collector;
    AethorMonotonicTimeState clock_state;
    CanFrame request;
    CanFrame reply;
    uint64_t boundary_us;
    uint64_t now_us;
    memset(&clock_state, 0, sizeof(clock_state)); motor_capability_init(&collector);
    now_us = aethor_monotonic_time_update(&clock_state, 1U);
    boundary_us = can_rx_inbox_observe_lower_bound(0ULL, now_us, 0U, 0U, 0U);
    motor_capability_observe_rx_boundary(&collector, boundary_us);
    assert(motor_capability_start(&collector, 1U, 1U, now_us));
    assert(!motor_capability_next(&collector, 1U, now_us, &request));
    now_us = aethor_monotonic_time_update(&clock_state, 1U);
    assert(!motor_capability_next(&collector, 1U, now_us, &request));
    now_us = aethor_monotonic_time_update(&clock_state, 5U);
    boundary_us = can_rx_inbox_observe_lower_bound(boundary_us, now_us, 1U, 0U, 0U);
    motor_capability_observe_rx_boundary(&collector, boundary_us);
    assert(!motor_capability_next(&collector, 1U, now_us, &request)); /* Queued RX preserves old boundary. */
    boundary_us = can_rx_inbox_observe_lower_bound(boundary_us, now_us, 0U, 0U, 0U);
    motor_capability_observe_rx_boundary(&collector, boundary_us);
    assert(motor_capability_next(&collector, 1U, now_us, &request));
    reply = response(1U, 0.1F);
    assert(motor_capability_accept(&collector, &reply, 1000ULL, now_us));
    assert(!collector.count && !collector.deferred); /* Truly older bound cannot be relabelled. */
    assert(motor_capability_accept(&collector, &reply, boundary_us, now_us));
    assert(!collector.count && collector.deferred); /* Equal millisecond boundary is new, but TX unproven. */
    assert(motor_capability_report_transmit(&collector, &request, 1U, now_us, 1U));
    assert(collector.count == 1U && collector.items[0].pre_submit_boundary_us == 5000ULL);
    now_us = aethor_monotonic_time_update(&clock_state, 9U);
    assert(!motor_capability_next(&collector, 1U, now_us, &request));
    now_us = aethor_monotonic_time_update(&clock_state, 13U);
    boundary_us = can_rx_inbox_observe_lower_bound(boundary_us, now_us, 0U, 0U, 1U);
    motor_capability_observe_rx_boundary(&collector, boundary_us);
    assert(!motor_capability_next(&collector, 1U, now_us, &request)); /* Pending TX also preserves old bound. */
    now_us = aethor_monotonic_time_update(&clock_state, 30U);
    assert(!motor_capability_next(&collector, 1U, now_us, &request));
    assert(collector.state == MOTOR_CAPABILITY_ABORTED && collector.reason == MOTOR_CAPABILITY_REASON_TIMEOUT);
}

/** @brief Executes all fixed-read and hardware-independent FIFO receipt contracts. */
int main(void)
{
    CanFrame frame;
    test_complete(); test_rejections(); test_receipt_and_abort(); test_fifo_tracker();
    test_quantized_clock_and_fifo_boundary();
    assert(s3519_pack_parameter_read(7U, (S3519Register)1U, &frame) != S3519_CODEC_STATUS_OK);
    assert(s3519_pack_capability_read(7U, 6U, &frame) != S3519_CODEC_STATUS_OK);
    assert(s3519_pack_capability_read(6U, 0U, &frame) != S3519_CODEC_STATUS_OK);
    puts("MOTOR_CAPABILITY_COLLECTOR_TESTS_PASSED groups=5 signature_rejections=12");
    return 0;
}
