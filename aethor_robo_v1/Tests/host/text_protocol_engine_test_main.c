/**
 * @file text_protocol_engine_test_main.c
 * @brief Verifies the public aethor-text-v1 lifecycle and replay contract.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol_engine.h"
#include "s3519_codec.h"

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
    query_context.motor_position_max_rad[0] = 12.5F;
    query_context.motor_velocity_max_rad_s[0] = 45.0F;
    query_context.motor_maximum_speed_rad_s[0] = 20.0F;
    query_context.motor_motion_limits_valid_mask = 0x01U;
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

    query_context.arm.enabled = 0U;
    query_context.motors.valid_joint_mask = 0x05U;
    query_context.motors.joints[0].driver_state = S3519_DRIVER_STATE_ENABLED;
    query_context.motors.joints[2].driver_state = S3519_DRIVER_STATE_ENABLED;
    protocol_engine_update_query_context(&engine, &query_context);
    response = process_text_request(&engine,
                                    "2 ping\n",
                                    3000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response,
                  "ok 2 ping state=disabled enabled=05 boot=1234\n") == 0);
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
                                    "hello extra\n",
                                    1500U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 0 hello code=bad_argument\n") == 0);

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
    assert(strstr(response,
                  "pmax_deg=716.197 vmax_deg_s=2578.31 "
                  "max_speed_deg_s=1145.916 "
                  "move_speed_limit_deg_s=1145.916") != NULL);
    assert(strlen(response) < TEXT_PROTOCOL_MAX_RESPONSE_LINE_LENGTH);
    assert(response[strlen(response) - 1U] == '\n');

    response = process_text_request(&engine,
                                    "show motor 2\n",
                                    183925000ULL,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strstr(response,
                  "pmax_deg=? vmax_deg_s=? max_speed_deg_s=? "
                  "move_speed_limit_deg_s=?") != NULL);

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

/** @brief Verifies bench commands map to the existing bounded business queue. */
static void test_text_bench_commands(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolCommand command;
    const char *response;

    protocol_engine_init(&engine, 9123U);
    response = process_text_request(&engine,
                                    "20 bench init 1,3\n",
                                    1000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response, "ok 20 bench init accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_INIT_MOTORS);
    assert(command.motor_mask == 0x05U);
    assert(command.control_mode == ARM_CONTROL_MODE_POSITION_VELOCITY);
    assert(command.bench_relative_scope != 0U);

    response = process_text_request(&engine,
                                    "21 bench enable 1,3\n",
                                    2000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response, "ok 21 bench enable accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_ENABLE);
    assert(command.motor_mask == 0x05U);

    response = process_text_request(&engine,
                                    "22 bench jog 1,3 delta=0.2 speed=1\n",
                                    3000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response, "ok 22 bench jog accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_MOVE_RELATIVE);
    assert(command.motor_mask == 0x05U);
    assert(command.values[0] == 0.2F);
    assert(command.values[2] == 0.2F);
    assert(command.speeds[0] == 1.0F);
    assert(command.speeds[2] == 1.0F);

    response = process_text_request(&engine,
                                    "23 bench stop 1,3\n",
                                    4000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response, "ok 23 bench stop accepted=1\n") == 0);
    assert(protocol_engine_pop_stop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_STOP);
    assert(command.motor_mask == 0x05U);
}

/** @brief Verifies one-shot bench moves preserve caller motor/value ordering. */
static void test_text_bench_move_ordered_values(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolCommand command;
    const char *response;
    uint8_t joint_index;

    protocol_engine_init(&engine, 9178U);
    response = process_text_request(&engine,
                                    "50 bench move 1 position=90 speed=30\n",
                                    1000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response, "ok 50 bench move accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED);
    assert(command.motor_mask == 0x01U);
    assert(command.values[0] == 90.0F);
    assert(command.speeds[0] == 30.0F);

    response = process_text_request(
        &engine,
        "51 bench move 1,3 position=90,-45 speed=30,20\n",
        2000U,
        PROTOCOL_ENGINE_STATUS_OK,
        &output_batch);
    assert(strcmp(response, "ok 51 bench move accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED);
    assert(command.motor_mask == 0x05U);
    assert(command.values[0] == 90.0F);
    assert(command.values[2] == -45.0F);
    assert(command.speeds[0] == 30.0F);
    assert(command.speeds[2] == 20.0F);

    response = process_text_request(
        &engine,
        "58 bench move 3,1 position=-45,90 speed=20,30\n",
        3000U,
        PROTOCOL_ENGINE_STATUS_OK,
        &output_batch);
    assert(strcmp(response, "ok 58 bench move accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED);
    assert(command.motor_mask == 0x05U);
    assert(command.values[0] == 90.0F);
    assert(command.values[2] == -45.0F);
    assert(command.speeds[0] == 30.0F);
    assert(command.speeds[2] == 20.0F);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if ((command.motor_mask & (uint8_t)(1U << joint_index)) == 0U)
        {
            assert(command.values[joint_index] == 0.0F);
            assert(command.speeds[joint_index] == 0.0F);
        }
    }
}

/** @brief Verifies one-shot bench moves reject malformed one-to-one lists. */
static void test_text_bench_move_rejections(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    const char *response;

    protocol_engine_init(&engine, 9189U);
    response = process_text_request(
        &engine,
        "52 bench move 1,3 position=90 speed=30,20\n",
        1000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 52 bench move code=count_mismatch field=position\n") == 0);

    response = process_text_request(
        &engine,
        "53 bench move 1,3 position=90,-45 speed=30\n",
        2000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 53 bench move code=count_mismatch field=speed\n") == 0);

    response = process_text_request(
        &engine,
        "54 bench move 1,1 position=10,20 speed=1,1\n",
        3000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 54 bench move code=bad_argument field=motors\n") == 0);

    response = process_text_request(&engine,
                                    "55 bench move 1 position=nan speed=1\n",
                                    4000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 55 bench move code=bad_argument field=position\n") == 0);

    response = process_text_request(&engine,
                                    "56 bench move 1 position=1 speed=0\n",
                                    5000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 56 bench move code=out_of_range field=speed\n") == 0);

    response = process_text_request(&engine,
                                    "59 bench move 1 position=1 speed=-1\n",
                                    6000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 59 bench move code=out_of_range field=speed\n") == 0);

    response = process_text_request(
        &engine,
        "60 bench move 1,3 position=90, speed=30,20\n",
        7000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 60 bench move code=count_mismatch field=position\n") == 0);

    response = process_text_request(&engine,
                                    "61 bench move 1 position=inf speed=1\n",
                                    8000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 61 bench move code=bad_argument field=position\n") == 0);

    response = process_text_request(&engine,
                                    "63 bench move 1 position=90, speed=30\n",
                                    9000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 63 bench move code=count_mismatch field=position\n") == 0);

    response = process_text_request(&engine,
                                    "64 bench move 1 position=1\n",
                                    10000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 64 bench move code=bad_argument field=speed\n") == 0);

    response = process_text_request(
        &engine,
        "65 bench move 1,,3 position=10,20 speed=1,1\n",
        11000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 65 bench move code=bad_argument field=motors\n") == 0);

    response = process_text_request(
        &engine,
        "66 bench move 1,3 position=,90 speed=30,20\n",
        12000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 66 bench move code=count_mismatch field=position\n") == 0);

    response = process_text_request(
        &engine,
        "67 bench move 1,3 position=90,,45 speed=30,20\n",
        13000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 67 bench move code=count_mismatch field=position\n") == 0);

    response = process_text_request(
        &engine,
        "68 bench move 1,3 position=90,45 speed=30,\n",
        14000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 68 bench move code=count_mismatch field=speed\n") == 0);

    response = process_text_request(
        &engine,
        "69 bench move 1,3 position=90,45 speed=,30\n",
        15000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 69 bench move code=count_mismatch field=speed\n") == 0);

    response = process_text_request(
        &engine,
        "70 bench move 1,3 position=90,45 speed=30,,20\n",
        16000U,
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        &output_batch);
    assert(strcmp(response,
                  "error 70 bench move code=count_mismatch field=speed\n") == 0);

    response = process_text_request(&engine,
                                    "71 bench move 1 position=abc speed=1\n",
                                    17000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 71 bench move code=bad_argument field=position\n") == 0);

    response = process_text_request(&engine,
                                    "72 bench move 1 position=1,2 speed=1\n",
                                    18000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 72 bench move code=count_mismatch field=position\n") == 0);
}

/** @brief Verifies accepted bench moves replay without a second queue entry. */
static void test_text_bench_move_replay(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolCommand command;
    const char *response;

    protocol_engine_init(&engine, 9190U);
    response = process_text_request(
        &engine,
        "62 bench move 3,1 position=-45,90 speed=20,30\n",
        1000U,
        PROTOCOL_ENGINE_STATUS_OK,
        &output_batch);
    assert(strcmp(response, "ok 62 bench move accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    assert(protocol_engine_pop_command(&engine, &command) == 0U);

    response = process_text_request(
        &engine,
        "62 bench move 3,1 position=-45,90 speed=20,30\n",
        2000U,
        PROTOCOL_ENGINE_STATUS_REPLAYED,
        &output_batch);
    assert(strcmp(response, "ok 62 bench move accepted=1\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) == 0U);

    response = process_text_request(
        &engine,
        "62 bench move 3,1 position=-44,90 speed=20,30\n",
        3000U,
        PROTOCOL_ENGINE_STATUS_REQUEST_ID_CONFLICT,
        &output_batch);
    assert(strcmp(response,
                  "error 62 bench move code=request_conflict\n") == 0);
    assert(protocol_engine_pop_command(&engine, &command) == 0U);
}

/** @brief Verifies bench structural rules and compile-time profile gate. */
static void test_text_bench_rejections(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    const char *response;

    protocol_engine_init(&engine, 9234U);
    response = process_text_request(&engine,
                                    "57 bench jog 1 delta=90 speed=30\n",
                                    1000U,
                                    PROTOCOL_ENGINE_STATUS_OK,
                                    &output_batch);
    assert(strcmp(response, "ok 57 bench jog accepted=1\n") == 0);

    response = process_text_request(&engine,
                                    "31 bench enable 3,1\n",
                                    2000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 31 bench enable code=bad_argument field=motors\n") == 0);

    response = process_text_request(&engine,
                                    "32 arm enable\n",
                                    3000U,
                                    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                                    &output_batch);
    assert(strcmp(response,
                  "error 32 arm enable code=profile current=bench required=arm\n") == 0);
}

/** @brief Verifies asynchronous bench completion uses the public done grammar. */
static void test_text_bench_done_output(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolCommand command;
    ProtocolCommandResult result;

    protocol_engine_init(&engine, 9345U);
    (void)process_text_request(&engine,
                               "40 bench jog 1,3 delta=0.2 speed=1\n",
                               1000U,
                               PROTOCOL_ENGINE_STATUS_OK,
                               &output_batch);
    assert(protocol_engine_pop_command(&engine, &command) != 0U);
    memset(&result, 0, sizeof(result));
    result.request_id = command.request_id;
    result.session_id = command.session_id;
    result.type = command.type;
    result.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
    result.accepted_at_us = command.accepted_at_us;
    result.completed_at_us = 3051000U;
    result.motor_mask = command.motor_mask;
    result.bench_relative_scope = command.bench_relative_scope;
    assert(protocol_engine_submit_command_result(&engine, &result) != 0U);
    assert(protocol_engine_pop_result_output(&engine, &output_batch) != 0U);
    assert(strcmp(output_batch.messages[0].data,
                  "done 40 bench jog result=completed elapsed_ms=3050 arrived=05\n") == 0);
}

/** @brief Verifies the communication watchdog is active only while energized. */
static void test_text_watchdog_scope(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolQueryContext query_context;

    protocol_engine_init(&engine, 9456U);
    memset(&query_context, 0, sizeof(query_context));
    query_context.arm.state = ARM_STATE_DISABLED;
    protocol_engine_update_query_context(&engine, &query_context);
    (void)process_text_request(&engine,
                               "1 hello\n",
                               1000U,
                               PROTOCOL_ENGINE_STATUS_OK,
                               &output_batch);
    assert(protocol_engine_watchdog_expired(&engine, 2000000U) == 0U);

    query_context.arm.state = ARM_STATE_READY;
    query_context.arm.enabled = 1U;
    protocol_engine_update_query_context(&engine, &query_context);
    (void)process_text_request(&engine,
                               "ping\n",
                               2001000U,
                               PROTOCOL_ENGINE_STATUS_OK,
                               &output_batch);
    assert(protocol_engine_watchdog_expired(&engine, 3000999U) == 0U);
    assert(protocol_engine_watchdog_expired(&engine, 3001000U) == 1U);
    assert(protocol_engine_watchdog_expired(&engine, 3002000U) == 0U);
}

/** @brief Verifies an unknown parsed command cannot refresh the live watchdog. */
static void test_unknown_text_command_does_not_keep_motors_energized(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    ProtocolQueryContext query_context;

    protocol_engine_init(&engine, 9467U);
    memset(&query_context, 0, sizeof(query_context));
    query_context.arm.state = ARM_STATE_READY;
    query_context.arm.enabled = 1U;
    protocol_engine_update_query_context(&engine, &query_context);
    (void)process_text_request(&engine,
                               "1 hello\n",
                               1000U,
                               PROTOCOL_ENGINE_STATUS_OK,
                               &output_batch);
    (void)process_text_request(&engine,
                               "2 nonsense\n",
                               900000U,
                               PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
                               &output_batch);
    assert(protocol_engine_watchdog_expired(&engine, 1001000U) == 1U);
}

/** @brief Verifies terminal-friendly bounded help topics. */
static void test_text_help(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;

    protocol_engine_init(&engine, 9567U);
    (void)process_text_request(&engine,
                               "help\n",
                               1000U,
                               PROTOCOL_ENGINE_STATUS_OK,
                               &output_batch);
    assert(strcmp(output_batch.messages[0].data,
                  "ok 0 help topics=show,stream,arm,bench "
                  "examples=show_state,arm_move,bench_jog\n") == 0);

    (void)process_text_request(&engine,
                               "help bench\n",
                               2000U,
                               PROTOCOL_ENGINE_STATUS_OK,
                               &output_batch);
    assert(strcmp(output_batch.messages[0].data,
                  "ok 0 help bench commands=init,enable,jog,stop,disable,clear\n") == 0);
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
    test_text_bench_commands();
    test_text_bench_rejections();
    test_text_bench_move_ordered_values();
    test_text_bench_move_rejections();
    test_text_bench_move_replay();
    test_text_bench_done_output();
    test_text_watchdog_scope();
    test_unknown_text_command_does_not_keep_motors_energized();
    test_text_help();
    puts("TEXT_PROTOCOL_ENGINE_TESTS_PASSED");
    return 0;
}
