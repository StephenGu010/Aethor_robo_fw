/**
 * @file text_protocol_engine_test_main.c
 * @brief Verifies the public aethor-text-v1 lifecycle and replay contract.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol_engine.h"

/**
 * @brief Processes one literal request and returns the single response message.
 * @param engine Initialized protocol engine.
 * @param request Null-terminated aethor-text-v1 request.
 * @param timestamp_us Deterministic request timestamp.
 * @param expected_status Expected engine outcome.
 * @param output_batch Destination output batch.
 * @return Pointer to the single null-terminated output message.
 */
static const char *process_text_request(ProtocolEngine *engine,
                                        const char *request,
                                        uint64_t timestamp_us,
                                        ProtocolEngineStatus expected_status,
                                        ProtocolOutputBatch *output_batch)
{
    ProtocolEngineStatus status = protocol_engine_process_text_line(
        engine,
        request,
        strlen(request),
        timestamp_us,
        output_batch);

    assert(status == expected_status);
    assert(output_batch->count == 1U);
    assert(output_batch->messages[0].length ==
           strlen(output_batch->messages[0].data));
    assert(output_batch->messages[0].data[
               output_batch->messages[0].length - 1U] == '\n');
    return output_batch->messages[0].data;
}

/** @brief Verifies HELLO and PING expose only the short public identity fields. */
static void test_text_lifecycle(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolQueryContext query_context;
    const char *response;

    protocol_engine_init(&engine, 1234U);
    memset(&query_context, 0, sizeof(query_context));
    query_context.arm.state = ARM_STATE_DISABLED;
    protocol_engine_update_query_context(&engine, &query_context);

    response = process_text_request(&engine,
                                    "1 hello\n",
                                    1000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 1 hello protocol=aethor-text-v1 fw=0.1.0-phase0 "
                  "profile=bench dof=7 boot=1234 watchdog_ms=1000\n") == 0);
    assert(engine.session_active != 0U);

    response = process_text_request(&engine,
                                    "ping\r\n",
                                    2000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 0 ping state=disabled enabled=00 boot=1234\n") == 0);
}

/** @brief Verifies nonzero IDs replay exactly and conflicting bodies are rejected. */
static void test_text_request_replay(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    char first_response[PROTOCOL_ENGINE_MESSAGE_CAPACITY];
    const char *response;

    protocol_engine_init(&engine, 4321U);
    response = process_text_request(&engine,
                                    "7 ping\n",
                                    1000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    (void)strcpy(first_response, response);

    response = process_text_request(&engine,
                                    "7   PING\n",
                                    2000U,
                                    PROTOCOL_ENGINE_STATUS_REPLAYED,
                                    &output_batch);
    assert(strcmp(response, first_response) == 0);

    response = process_text_request(&engine,
                                    "7 hello\n",
                                    3000U,
                                    PROTOCOL_ENGINE_STATUS_REQUEST_ID_CONFLICT,
                                    &output_batch);
    assert(strcmp(response,
                  "error 7 hello code=request_conflict\n") == 0);
}

/** @brief Verifies manual request ID zero is never retained for replay. */
static void test_manual_request_is_not_cached(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;

    protocol_engine_init(&engine, 5678U);
    (void)process_text_request(&engine,
                               "hello\n",
                               1000U,
                               PROTOCOL_ENGINE_STATUS_OK,
                               &output_batch);
    (void)process_text_request(&engine,
                               "0 ping\n",
                               2000U,
                               PROTOCOL_ENGINE_STATUS_OK,
                               &output_batch);
    assert(strcmp(output_batch.messages[0].data,
                  "ok 0 ping state=boot enabled=00 boot=5678\n") == 0);
}

/** @brief Verifies parser and dispatch failures use short stable error codes. */
static void test_text_errors(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    char overlong_request[TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH + 2U];
    const char *response;

    protocol_engine_init(&engine, 6789U);
    response = process_text_request(&engine,
                                    "nonsense\n",
                                    1000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 0 nonsense code=unknown_command\n") == 0);

    response = process_text_request(&engine,
                                    "show\tstate\n",
                                    2000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response, "error 0 parse code=bad_line\n") == 0);

    memset(overlong_request, 'x', sizeof(overlong_request));
    overlong_request[sizeof(overlong_request) - 1U] = '\0';
    response = process_text_request(&engine,
                                    overlong_request,
                                    3000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 0 parse code=line_too_long\n") == 0);
}

/** @brief Runs the aethor-text-v1 engine lifecycle tests. */
int main(void)
{
    test_text_lifecycle();
    test_text_request_replay();
    test_manual_request_is_not_cached();
    test_text_errors();
    puts("TEXT_PROTOCOL_ENGINE_TESTS_PASSED");
    return 0;
}
