/** @file persistence_wire_server.c
 * @brief Offline real-App text server with the existing synthetic B1 CAN fixture; no hardware ports are linked. */
#include "persistence_b1_fixture.inc"
#include <stdlib.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

/** @brief Advances only synthetic fixture transfers, with the same immutable typed/TX/RX callbacks used by B1 tests. */
static uint64_t wire_server_pump(uint64_t now_us, uint8_t early_ack)
{
    unsigned step;
    for (step = 0U; step < 4U && application_persistence.locked && !application_persistence.fault; ++step) {
        MotorPersistenceState state;
        aethor_app_persistence_service(now_us);
        if (!application_persistence.pmax_known || !application_persistence.mode_known) { break; }
        if (!application_persistence.health_proven || now_us - application_persistence.health.observed_us >= 50000ULL) {
            now_us = b1_fd(now_us); continue;
        }
        state = application_persistence_transaction.state;
        if (state == MOTOR_PERSISTENCE_BASELINE || state == MOTOR_PERSISTENCE_PREPARE_VERIFY ||
            state == MOTOR_PERSISTENCE_SAVE_VERIFY || state == MOTOR_PERSISTENCE_READBACK) {
            now_us = b1_pair(now_us, b1_register(application_persistence_transaction.read_index,
                (uint8_t)(state != MOTOR_PERSISTENCE_BASELINE)), early_ack, NULL);
        } else if (state == MOTOR_PERSISTENCE_PREPARE_WRITE) {
            now_us = b1_pair(now_us, application_persistence_transaction.write_step ? 2U : 0x44000000U, early_ack, NULL);
        } else if (state == MOTOR_PERSISTENCE_SAVE_WAIT) { now_us = b1_pair(now_us, 0U, early_ack, NULL); }
        else { break; }
    }
    return now_us;
}

/** @brief Serves bounded stdin requests and exact CRLF wire responses, reporting fixture time separately to the Python clock. */
int main(int argc, char **argv)
{
    char line[256];
    uint32_t mode = argc > 1 ? (uint32_t)strtoul(argv[1], NULL, 10) : 1U;
    float pmax = argc > 2 ? strtof(argv[2], NULL) : 12.5F;
    uint8_t early_ack = (uint8_t)(argc > 3 && argv[3][0] == '1');
    uint64_t now_us;
#ifdef _WIN32
    (void)_setmode(_fileno(stdin), _O_BINARY);
    (void)_setmode(_fileno(stdout), _O_BINARY);
#endif
    (void)setvbuf(stdout, NULL, _IONBF, 0);
    now_us = readonly_fixture(mode, pmax);
    b1_source_mode = mode; b1_source_pmax = pmax; b1_native_position = 0.25F;
    b1_fd_offset = b1_pm_offset = 0.0F;
    application_motor_runtime.bank.motors[6].feedback.position_rad = b1_native_position;
    printf("BOOT %lu CLOCK %llu\r\n", (unsigned long)application_protocol_engine.boot_id, (unsigned long long)now_us);
    while (fgets(line, sizeof(line), stdin)) {
        ProtocolOutputBatch output;
        now_us += 4000ULL;
        now_us = wire_server_pump(now_us, early_ack);
        aethor_app_integrated_set_can_idle(1U, now_us);
        (void)request(line, now_us, &output);
        if (output.count != 1U) { fprintf(stderr, "unexpected output count\n"); return 2; }
        if (fwrite(output.messages[0].data, 1U, output.messages[0].length, stdout) != output.messages[0].length) { return 3; }
        printf("CLOCK %llu\r\n", (unsigned long long)now_us);
    }
    return 0;
}
