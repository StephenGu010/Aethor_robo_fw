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
 * @brief Builds a coherent query snapshot with distinctive seven-axis values.
 * @return Deterministic snapshot suitable for readable output assertions.
 */
static ProtocolQueryContext make_query_context(void)
{
    ProtocolQueryContext query_context;
    static const float positions[ARM_JOINT_COUNT] = {
        0.0F, -15.012F, 29.998F, 0.004F, 20.001F, -0.003F, 4.995F
    };
    uint8_t joint_index;

    memset(&query_context, 0, sizeof(query_context));
    query_context.timestamp_us = 183924000ULL;
    query_context.arm.state = ARM_STATE_READY;
    query_context.arm.fault = ARM_FAULT_NONE;
    query_context.arm.aligned = 1U;
    query_context.arm.enabled = 1U;
    query_context.joints.published_at_us = 183920000ULL;
    query_context.joints.valid_joint_mask = 0x7FU;
    query_context.joints.aligned = 1U;
    query_context.motors.valid_joint_mask = 0x03U;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        query_context.joints.position_deg[joint_index] = positions[joint_index];
        query_context.motors.joints[joint_index].timestamp_us = 183920000ULL;
    }
    query_context.motors.joints[0].driver_state = 1U;
    query_context.motors.joints[1].driver_state = 1U;
    query_context.motors.joints[0].position_rad = 0.05235988F;
    query_context.motors.joints[0].torque_nm = 0.12F;
    query_context.motors.joints[0].mos_temperature_c = 32.0F;
    query_context.motors.joints[0].rotor_temperature_c = 30.0F;
    query_context.diagnostics.control_period_max_us = 4102U;
    query_context.diagnostics.can_tx_error_count = 2U;
    query_context.diagnostics.usb_telemetry_drop_count = 3U;
    return query_context;
}

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

/** @brief Verifies the concise state, joint, and motor snapshot queries. */
static void test_text_show_queries(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolQueryContext query_context = make_query_context();
    const char *response;

    protocol_engine_init(&engine, 7890U);
    protocol_engine_update_query_context(&engine, &query_context);

    response = process_text_request(&engine,
                                    "show state\n",
                                    183925000ULL,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 0 show state state=ready aligned=1 enabled=7f "
                  "moving=0 active=0 fault=none\n") == 0);

    response = process_text_request(&engine,
                                    "show joints\n",
                                    183926000ULL,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 0 show joints t_ms=183920 q=0,-15.012,29.998,0.004,"
                  "20.001,-0.003,4.995 qd=0,0,0,0,0,0,0 valid=7f\n") == 0);

    response = process_text_request(&engine,
                                    "show motors\n",
                                    183927000ULL,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 0 show motors present=03 enabled=03 moving=00 "
                  "holding=03 stale=00 fault=00\n") == 0);
}

/** @brief Verifies stream admission, due output, frequency limits, and stop. */
static void test_text_streaming(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolQueryContext query_context = make_query_context();
    const char *response;

    protocol_engine_init(&engine, 8901U);
    protocol_engine_update_query_context(&engine, &query_context);

    response = process_text_request(&engine,
                                    "3 stream joints rate=20\n",
                                    1000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response, "ok 3 stream joints rate=20\n") == 0);
    assert(protocol_engine_generate_stream_output(&engine,
                                                  1000U,
                                                  &output_batch) == 1U);
    assert(strcmp(output_batch.messages[0].data,
                  "data 501 joints t_ms=183920 q=0,-15.012,29.998,0.004,"
                  "20.001,-0.003,4.995 qd=0,0,0,0,0,0,0 valid=7f "
                  "state=ready\n") == 0);
    assert(protocol_engine_generate_stream_output(&engine,
                                                  50000U,
                                                  &output_batch) == 0U);
    assert(protocol_engine_generate_stream_output(&engine,
                                                  51000U,
                                                  &output_batch) == 1U);

    response = process_text_request(&engine,
                                    "4 stream motors rate=11\n",
                                    60000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 4 stream motors code=out_of_range field=rate\n") == 0);

    response = process_text_request(&engine,
                                    "stream off\n",
                                    70000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response, "ok 0 stream off\n") == 0);
    assert(protocol_engine_generate_stream_output(&engine,
                                                  1000000U,
                                                  &output_batch) == 0U);
}

/** @brief Verifies device, single-motor, configuration, and diagnostic queries. */
static void test_text_detailed_show_queries(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolQueryContext query_context = make_query_context();
    const char *response;

    protocol_engine_init(&engine, 9012U);
    protocol_engine_update_query_context(&engine, &query_context);

    response = process_text_request(&engine,
                                    "show info\n",
                                    183925000ULL,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 0 show info mcu=STM32H723VGT6 transport=usb_cdc "
                  "can=classic_1m motor=s3519 driver=dm3520 build=unknown\n") == 0);

    response = process_text_request(&engine,
                                    "show motor 1\n",
                                    183925000ULL,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 0 show motor joint=1 esc=01 master=11 state=holding "
                  "pos_deg=3 speed_deg_s=0 torque_nm=0.12 mos_c=32 "
                  "rotor_c=30 fault=0 age_ms=5\n") == 0);

    response = process_text_request(&engine,
                                    "show motor 8\n",
                                    183925000ULL,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 0 show motor code=out_of_range field=joint\n") == 0);

    response = process_text_request(&engine,
                                    "show config 1\n",
                                    183925000ULL,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 0 show config joint=1 esc=01 master=11 dir=? gear=? "
                  "min_deg=? max_deg=? vmax_deg_s=? amax_deg_s2=? verified=00\n") == 0);

    response = process_text_request(&engine,
                                    "show config\n",
                                    183925000ULL,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strncmp(response, "ok 0 show config map=", 21U) == 0);
    assert(strstr(response, " required=ff verified=00 enable_ready=0\n") != NULL);

    response = process_text_request(&engine,
                                    "show diag\n",
                                    183925000ULL,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 0 show diag loop_max_us=4102 deadline_miss=0 "
                  "can_error=2 usb_drop=3 fault=none\n") == 0);
}

/** @brief Runs the aethor-text-v1 engine lifecycle tests. */
int main(void)
{
    test_text_lifecycle();
    test_text_request_replay();
    test_manual_request_is_not_cached();
    test_text_errors();
    test_text_show_queries();
    test_text_streaming();
    test_text_detailed_show_queries();
    puts("TEXT_PROTOCOL_ENGINE_TESTS_PASSED");
    return 0;
}
