/** @file adrc_protocol_test_main.c
 * @brief Tests shared replay, trusted admission, and bounded ADRC command publication.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "adrc_protocol.h"

/** @brief Processes one literal request through the shared protocol engine. */
static ProtocolEngineStatus request(ProtocolEngine *engine, const char *line,
    uint64_t now_us, ProtocolOutputBatch *output)
{
    return protocol_engine_process_text_line(engine, line, strlen(line), now_us, output);
}

/** @brief Verifies immediate DONE replay, STOP cancellation, and permanent boot-local stale ID rejection. */
static void test_terminal_and_stop_contract(void)
{
    ProtocolEngine engine;
    AdrcProtocolGateway gateway;
    AdrcProtocolSnapshot snapshot;
    AdrcProtocolCommand command;
    ProtocolOutputBatch output;
    unsigned int index;
    char line[80];
    protocol_engine_init(&engine, 1U);
    adrc_protocol_init(&gateway);
    protocol_engine_set_adrc_handler(&engine, adrc_protocol_handle_request, &gateway);
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.status.state = ADRC_STATE_PREPARED;
    snapshot.status.disabled_confirmed = 1U;
    snapshot.qualified = 1U;
    snapshot.trace_frozen = 1U;
    adrc_protocol_publish_snapshot(&gateway, &snapshot);
    assert(request(&engine, "1 adrc config group=identify torque=-0.01 pulse_ms=40", 1U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(adrc_protocol_pop_command(&gateway, &command) == 1U);
    assert(command.group == ADRC_CONFIG_IDENTIFY);
    assert(command.values[0] == -0.01F);
    assert(adrc_protocol_complete_command(&gateway, 1U, ADRC_RESULT_OK, 2U) == 1U);
    assert(adrc_protocol_pop_result_output(&gateway, &engine, &output) == 1U);
    assert(request(&engine, "1 adrc config group=identify torque=-0.01 pulse_ms=40", 3U, &output) == PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(strstr(output.messages[0].data, "done 1 adrc") != NULL);
    assert(request(&engine, "2 adrc run", 4U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(request(&engine, "3 adrc stop", 5U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(adrc_protocol_pop_command(&gateway, &command) == 1U);
    assert(command.type == ADRC_PROTOCOL_STOP);
    assert(adrc_protocol_pop_command(&gateway, &command) == 0U);
    assert(adrc_protocol_pop_result_output(&gateway, &engine, &output) == 1U);
    assert(strstr(output.messages[0].data, "done 2 adrc") != NULL);
    for (index = 0U; index < 40U; ++index)
    {
        (void)snprintf(line, sizeof(line), "%u adrc status", index + 100U);
        assert(request(&engine, line, 100U + index, &output) == PROTOCOL_ENGINE_STATUS_OK);
    }
    assert(request(&engine, "2 adrc run", 70000000ULL, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "stale_request_id") != NULL);
    assert(adrc_protocol_complete_command(&gateway, 3U, ADRC_RESULT_OK, 70000001ULL) == 1U);
    assert(adrc_protocol_pop_result_output(&gateway, &engine, &output) == 1U);
    assert(strstr(output.messages[0].data, "done 3 adrc") != NULL);
    assert(protocol_engine_complete_adrc_request(&engine, 999U, 70000002ULL, &output) == 0U);
    assert(request(&engine, "4 adrc config group=identify torque=0.06 pulse_ms=40", 70000003ULL, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request(&engine, "5 adrc config group=identify torque=0.01 pulse_ms=41", 70000004ULL, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
}

/** @brief Returns a deterministic frozen trace record for read-gating tests. */
static uint8_t read_trace(void *context, uint32_t index, AdrcExperimentTrace *record)
{
    unsigned int *calls = (unsigned int *)context;
    ++*calls;
    if (index != 0U) { return 0U; }
    memset(record, 0, sizeof(*record));
    record->timestamp_us = 123U;
    return 1U;
}

/** @brief Verifies all pending channels survive hello/cache churn and frozen trace is gated. */
static void test_channels_and_trace(void)
{
    ProtocolEngine engine;
    ProtocolEngine empty_engine;
    AdrcProtocolGateway gateway;
    AdrcProtocolSnapshot snapshot;
    AdrcProtocolCommand command;
    ProtocolOutputBatch output;
    unsigned int calls = 0U;
    unsigned int index;
    char line[80];
    protocol_engine_init(&engine, 1U);
    protocol_engine_init(&empty_engine, 2U);
    adrc_protocol_init(&gateway);
    protocol_engine_set_adrc_handler(&engine, adrc_protocol_handle_request, &gateway);
    adrc_protocol_set_trace_reader(&gateway, read_trace, &calls);
    assert(request(&engine, "adrc trace index=0", 1U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(calls == 0U);
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.status.disabled_confirmed = 1U;
    snapshot.trace_frozen = 1U;
    adrc_protocol_publish_snapshot(&gateway, &snapshot);
    assert(request(&engine, "adrc trace index=0", 2U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(calls == 1U);
    assert(request(&engine, "adrc", 3U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(request(&engine, "10 adrc prepare motor=1", 4U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(request(&engine, "11 adrc heartbeat", 5U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(request(&engine, "12 adrc stop", 6U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(request(&engine, "13 adrc stop", 7U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "busy") != NULL);
    assert(request(&engine, "hello", 8U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(request(&engine, "hello", 9U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    for (index = 0U; index < 40U; ++index)
    {
        (void)snprintf(line, sizeof(line), "%u adrc status", index + 100U);
        assert(request(&engine, line, 100U + index, &output) == PROTOCOL_ENGINE_STATUS_OK);
    }
    assert(request(&engine, "10 adrc prepare motor=1", 70000000ULL, &output) == PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(request(&engine, "10 adrc prepare motor=2", 70000001ULL, &output) == PROTOCOL_ENGINE_STATUS_REQUEST_ID_CONFLICT);
    assert(adrc_protocol_pop_command(&gateway, &command) == 1U);
    assert(command.type == ADRC_PROTOCOL_STOP);
    assert(adrc_protocol_complete_command(&gateway, command.request_id, ADRC_RESULT_OK, 70000002ULL) == 1U);
    assert(adrc_protocol_pop_command(&gateway, &command) == 1U);
    assert(command.type == ADRC_PROTOCOL_HEARTBEAT);
    assert(adrc_protocol_complete_command(&gateway, command.request_id, ADRC_RESULT_OK, 70000003ULL) == 1U);
    assert(adrc_protocol_pop_command(&gateway, &command) == 1U);
    assert(command.type == ADRC_PROTOCOL_PREPARE);
    assert(adrc_protocol_complete_command(&gateway, command.request_id, ADRC_RESULT_UNQUALIFIED, 70000004ULL) == 1U);
    assert(adrc_protocol_pop_result_output(&gateway, &empty_engine, &output) == 0U);
    assert(gateway.channel_state[0] == 3U);
    assert(adrc_protocol_pop_result_output(&gateway, &engine, &output) == 1U);
    assert(adrc_protocol_pop_result_output(&gateway, &engine, &output) == 1U);
    assert(adrc_protocol_pop_result_output(&gateway, &engine, &output) == 1U);
    assert(adrc_protocol_pop_result_output(&gateway, &engine, &output) == 0U);
    assert(request(&engine, "20 adrc config group=limits reference_accel=0.4", 70000005ULL, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request(&engine, "21 adrc config group=control wo=251", 70000006ULL, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
}

/** @brief Exercises fail-closed admission and a pinned request surviving cache churn. */
int main(void)
{
    ProtocolEngine engine;
    AdrcProtocolGateway gateway;
    AdrcProtocolSnapshot snapshot;
    AdrcProtocolCommand command;
    ProtocolOutputBatch output;
    unsigned int index;
    char line[80];
    protocol_engine_init(&engine, 1U);
    adrc_protocol_init(&gateway);
    protocol_engine_set_adrc_handler(&engine, adrc_protocol_handle_request, &gateway);
    assert(request(&engine, "1 adrc run", 1U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "not_ready") != NULL);
    assert(request(&engine, "2 adrc config group=evidence verified=1", 2U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.status.state = ADRC_STATE_PREPARED;
    snapshot.status.disabled_confirmed = 1U;
    snapshot.qualified = 1U;
    snapshot.trace_frozen = 1U;
    adrc_protocol_publish_snapshot(&gateway, &snapshot);
    assert(request(&engine, "adrc run", 3U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request(&engine, "3 adrc config group=limits torque_limit=0.2", 4U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request(&engine, "10 adrc run", 5U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(adrc_protocol_pop_command(&gateway, &command) == 1U);
    assert(command.type == ADRC_PROTOCOL_RUN);
    assert(request(&engine, "10  adrc   run", 6U, &output) == PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(adrc_protocol_pop_command(&gateway, &command) == 0U);
    for (index = 0U; index < 40U; ++index)
    {
        (void)snprintf(line, sizeof(line), "%u adrc status", index + 100U);
        assert(request(&engine, line, 100U + index, &output) == PROTOCOL_ENGINE_STATUS_OK);
    }
    assert(request(&engine, "10 adrc run", 70000000ULL, &output) == PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(request(&engine, "10 adrc stop", 70000001ULL, &output) == PROTOCOL_ENGINE_STATUS_REQUEST_ID_CONFLICT);
    assert(request(&engine, "11 adrc stop", 70000002ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(adrc_protocol_pop_command(&gateway, &command) == 1U);
    assert(command.type == ADRC_PROTOCOL_STOP);
    assert(adrc_protocol_complete_command(&gateway, 10U, ADRC_RESULT_OK, 70000003ULL) == 1U);
    assert(adrc_protocol_pop_result_output(&gateway, &engine, &output) == 1U);
    assert(strstr(output.messages[0].data, "done 10 adrc") != NULL);
    assert(request(&engine, "10 adrc run", 70000004ULL, &output) == PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(strstr(output.messages[0].data, "done 10 adrc") != NULL);
    test_terminal_and_stop_contract();
    test_channels_and_trace();
    puts("ADRC protocol tests passed");
    return 0;
}
