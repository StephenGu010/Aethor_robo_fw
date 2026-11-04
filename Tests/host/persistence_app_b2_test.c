/** @file persistence_app_b2_test.c
 * @brief Fixed maintenance text contract exercised through the real App parser, with synthetic CAN only. */
#include "persistence_b1_fixture.inc"

/** @brief Begins the fixed typed lifetime through the real public text entry, without direct private admission. */
static uint64_t b2_begin_wire(uint32_t mode, float pmax)
{
    ProtocolOutputBatch output;
    uint64_t now_us = readonly_fixture(mode, pmax);
    memset(&application_persistence_wire, 0, sizeof(application_persistence_wire));
    b1_source_mode = mode; b1_source_pmax = pmax; b1_native_position = 0.25F;
    b1_fd_offset = b1_pm_offset = 0.0F;
    application_motor_runtime.bank.motors[6].feedback.position_rad = b1_native_position;
    assert(request("701 adrc persist_begin motor=7", now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "ok 701 adrc persist_begin schema=1 id=701 accepted=1\r\n"));
    return b1_fd(now_us);
}

/** @brief A locked maintenance owner keeps its evidence queries and original session ping available. */
static void b2_readonly_wire_contract(void)
{
    ProtocolOutputBatch output;
    uint64_t now_us = b2_begin_wire(1U, 12.5F);
    uint64_t total = application_persistence_transaction.total_deadline_us;
    assert(request("702 ping", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "ok 702 ping "));
    now_us = b1_read45(now_us, 0U, 0U);
    assert(request("703 adrc persist_status", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "persist_status schema=1 id=701 "));
    assert(strstr(output.messages[0].data, "locked=1"));
    assert(strstr(output.messages[0].data, "ready=1"));
    assert(request("704 adrc persist_item id=701 phase=1 index=43", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "rid=80 kind=float32 raw=1048576000 valid=1"));
    assert(request("705 adrc persist_health id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "health_proven=1"));
    assert(strstr(output.messages[0].data, "continuity_valid=1"));
    assert(request("706 adrc persist_save_receipt id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "id=701 valid=0\r\n"));
    assert(application_persistence_transaction.total_deadline_us == total);
    assert(request("707 adrc persist_commit id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "persist_commit schema=1 code="));
    assert(!application_persistence_transaction.save_used);
    puts("PERSISTENCE_APP_B2_READONLY_WIRE_PASS");
}

/** @brief Malformed and wrong-scope text cannot reach an alternate parser or arbitrary write interface. */
static void b2_strict_fields(void)
{
    static const char *invalid[] = {
        "801 adrc persist_prepare id=+701", "802 adrc persist_prepare id=701.0",
        "803 adrc persist_prepare id=7.01e2", "804 adrc persist_prepare id=4294967296",
        "805 adrc persist_prepare", "806 adrc persist_prepare id=701 id=701",
        "807 adrc persist_prepare id=701 raw=1140850688", "808 adrc persist_prepare 701",
        "809 adrc persist_item id=701 phase=1 index=45", "810 adrc persist_item id=701 phase=257 index=0",
        "811 adrc persist_status id=701", "812 adrc persist_release id=702",
        "813 adrc persist_begin motor=6", "814 adrc persist_unknown id=701",
        "815 adrc wide_position motor=7", "garbage =="
    };
    ProtocolOutputBatch output;
    uint64_t now_us = b2_begin_wire(1U, 12.5F);
    unsigned index;
    now_us = b1_read45(now_us, 0U, 0U);
    for (index = 0U; index < sizeof(invalid) / sizeof(invalid[0]); ++index) {
        assert(request(invalid[index], ++now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(!application_protocol_engine.command_write_sequence);
        assert(!application_persistence_transaction.save_used);
        assert(application_persistence.locked && !application_persistence.fault);
    }
    puts("PERSISTENCE_APP_B2_STRICT_FIELDS_PASS");
}

/** @brief Exercises the fixed writes and one Save through text, then publishes onlyID7 after an explicit quiet release. */
static void b2_saved_release(uint32_t mode, float pmax, uint8_t early_ack)
{
    ProtocolOutputBatch output;
    MotorDiscoveryResult other_discovery[6];
    MotorObject other_motors[6];
    uint64_t now_us = b2_begin_wire(mode, pmax);
    uint32_t previous_admission = aethor_app_persistence_admission_generation();
    uint8_t skipped = (uint8_t)(mode == 2U && pmax == 512.0F);
    memcpy(other_discovery, application_motor_runtime.discovery.results, sizeof(other_discovery));
    memcpy(other_motors, application_motor_runtime.bank.motors, sizeof(other_motors));
    now_us = b1_read45(now_us, 0U, early_ack);
    if (!skipped) {
        assert(request("901 adrc persist_prepare id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
        while (application_persistence_transaction.state == MOTOR_PERSISTENCE_PREPARE_WRITE) {
            if (!application_persistence.health_proven) { now_us = b1_fd(now_us); }
            now_us = b1_pair(now_us, application_persistence_transaction.write_step ? 2U : 0x44000000U, early_ack, NULL);
        }
        now_us = b1_read45(now_us, 1U, early_ack);
        assert(request("902 adrc persist_verify id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
        now_us = b1_read45(now_us, 1U, early_ack);
        assert(request("903 adrc persist_commit id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(request("904 adrc persist_commit id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        now_us = b1_pair(now_us, 0U, early_ack, NULL);
        now_us = application_persistence_transaction.settle_until_us;
        aethor_app_persistence_service(now_us);
        now_us = b1_read45(now_us, 1U, early_ack);
        assert(request("905 adrc persist_save_receipt id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "valid=1"));
        assert(strstr(output.messages[0].data, "tx_id=2047 tx_dlc=4 tx_raw=27918343 ack_id=23 ack_dlc=4 ack_raw=27918343"));
    }
    assert(application_motor_runtime.discovery.results[6].ranges.position_max_rad == pmax);
    assert(application_motor_runtime.discovery.results[6].observed_control_mode == mode);
    assert(request("906 adrc persist_release id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "id=701 accepted=1"));
    assert(application_persistence.locked);
    assert(request("907 adrc persist_status", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "locked=1"));
    now_us += 4000ULL;
    /* Only the trusted Platform idle callback can renew this observation. */
    aethor_app_integrated_set_can_idle(1U, now_us);
    assert(request("908 adrc persist_status", now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "locked=0 ready=1"));
    assert(strstr(output.messages[0].data, "released=1"));
    assert(!application_persistence.locked);
    assert(application_motor_runtime.discovery.results[6].ranges.position_max_rad == 512.0F);
    assert(application_motor_runtime.discovery.results[6].observed_control_mode == 2U);
    assert(application_motor_runtime.bank.motors[6].feedback_valid);
    assert(application_motor_runtime.bank.motors[6].feedback.timestamp_us == application_persistence.health.observed_us);
    assert(application_motor_runtime.bank.motors[6].configuration_consistent);
    assert(application_motor_runtime.bank.motors[6].parameter_valid_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK);
    assert(!memcmp(other_discovery, application_motor_runtime.discovery.results, sizeof(other_discovery)));
    assert(!memcmp(other_motors, application_motor_runtime.bank.motors, sizeof(other_motors)));
    assert(!aethor_app_persistence_allow_ordinary(previous_admission));
    assert(aethor_app_persistence_allow_ordinary(aethor_app_persistence_admission_generation()));
    assert(request("909 adrc persist_begin motor=7", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    printf("PERSISTENCE_APP_B2_SAVED_RELEASE mode=%lu pmax=%.1f early=%u skip=%u PASS\n",
        (unsigned long)mode, (double)pmax, early_ack, skipped);
}

/** @brief Release cannot borrow an old idle cache, hide a context fault or import invalid ordinary discovery values. */
static void b2_release_rejects(void)
{
    ProtocolOutputBatch output;
    unsigned variant;
    for (variant = 0U; variant < 5U; ++variant) {
        uint64_t now_us = b2_begin_wire(2U, 512.0F);
        now_us = b1_read45(now_us, 0U, 0U);
        assert(request("921 adrc persist_release id=701", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
        now_us += 4000ULL;
        if (variant == 0U) { /* Deliberately do not renew the Platform idle observation. */ }
        else {
            aethor_app_integrated_set_can_idle(1U, now_us);
            if (variant == 1U) { ++application_debug_ui.epoch; }
            if (variant == 2U) { application_persistence_transaction.current.items[4].raw = readonly_float_bits(-1.0F); }
            if (variant == 3U) { aethor_app_persistence_preempt(now_us); }
            if (variant == 4U) { aethor_app_integrated_set_can_idle(0U, now_us); }
        }
        assert(request("922 adrc persist_status", now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(application_persistence.locked && !application_persistence_wire.released);
        assert(!application_motor_runtime.bank.motors[6].target_valid);
    }
    puts("PERSISTENCE_APP_B2_RELEASE_REJECTS_PASS");
}

/** @brief Checks the real C serializers at full integer widths, separately from semantic evidence validation. */
static void b2_full_width_serialization(void)
{
    static const char *queries[] = {"4294967295 adrc persist_status", "4294967295 adrc persist_health id=701",
        "4294967295 adrc persist_item id=701 phase=1 index=0", "4294967295 adrc persist_save_receipt id=701"};
    ProtocolOutputBatch output;
    TextProtocolRequest parsed;
    ProtocolEngineStatus status;
    MotorPersistenceItem *item;
    AethorPersistenceSaveJournal *journal;
    unsigned index;
    uint64_t now_us = b2_begin_wire(2U, 512.0F);
    now_us = b1_read45(now_us, 0U, 0U);
    application_persistence.boot_id = application_persistence.local_epoch = UINT32_MAX;
    application_persistence.protocol_session = application_persistence.bound_link_word = UINT32_MAX;
    application_persistence.last_us = application_persistence_transaction.verified_us = UINT64_MAX;
    application_persistence_transaction.identity.boot_epoch = application_persistence_transaction.identity.owner_generation = UINT32_MAX;
    application_persistence_transaction.identity.request_id = UINT32_MAX;
    application_persistence.health.observed_us = UINT64_MAX;
    application_persistence.health.velocity = application_persistence.health.torque = -FLT_MIN;
    application_persistence.health.motor_temperature = application_persistence.health.driver_temperature = FLT_MIN;
    application_persistence.reference_native_position = application_persistence.native_position = -FLT_MIN;
    item = &application_persistence_transaction.current.items[0];
    item->identity = application_persistence_transaction.identity; item->identity.phase = 1U;
    item->identity.sequence = item->raw = UINT32_MAX;
    item->created_us = item->submitted_us = item->boundary_us = item->send_completed_observed_us =
        item->rx_lower_us = item->received_us = UINT64_MAX;
    journal = &application_persistence_save_journal;
    memset(journal, 0, sizeof(*journal)); journal->receipt = *item; journal->receipt.identity.phase = 8U;
    journal->tx.identifier = 2047U; journal->ack.identifier = 23U;
    journal->tx.length = journal->ack.length = 4U;
    journal->tx.data[0] = journal->ack.data[0] = 7U;
    journal->tx.data[2] = journal->ack.data[2] = 0xaaU; journal->tx.data[3] = journal->ack.data[3] = 1U;
    journal->settle_until_us = UINT64_MAX; journal->valid = 1U;
    for (index = 0U; index < sizeof(queries) / sizeof(queries[0]); ++index) {
        assert(text_protocol_parse_request(queries[index], strlen(queries[index]), &parsed) == TEXT_PROTOCOL_STATUS_OK);
        assert(aethor_app_persistence_wire_route(&parsed, now_us, &output, &status));
        assert(status == PROTOCOL_ENGINE_STATUS_OK && output.count == 1U);
        assert(output.messages[0].length < sizeof(output.messages[0].data));
        assert(output.messages[0].length == strlen(output.messages[0].data));
        assert(!strcmp(output.messages[0].data + output.messages[0].length - 2U, "\r\n"));
        printf("SERIALIZATION_ONLY width=%u bytes=%u %s", index, output.messages[0].length, output.messages[0].data);
    }
    puts("PERSISTENCE_APP_B2_FULL_WIDTH_SERIALIZATION_PASS");
}

/** @brief Runs the bounded wire tests; no Core, Platform, HAL, USB device or motor is accessed. */
int main(void)
{
    b2_readonly_wire_contract();
    b2_strict_fields();
    b2_saved_release(1U, 12.5F, 0U);
    b2_saved_release(1U, 12.5F, 1U);
    b2_saved_release(2U, 12.5F, 0U);
    b2_saved_release(1U, 512.0F, 0U);
    b2_saved_release(2U, 512.0F, 0U);
    b2_release_rejects();
    b2_full_width_serialization();
    return 0;
}
