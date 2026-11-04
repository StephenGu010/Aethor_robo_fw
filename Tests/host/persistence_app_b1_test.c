/** @file persistence_app_b1_test.c
 * @brief Bounded typed persistence facade tests with synthetic CAN; Core/Platform/HAL are not linked. */
#define main existing_integrated_fixture_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Starts a new synthetic firmware process; production has no reset/retry entry. */
static uint64_t readonly_fixture(uint32_t mode, float position_range)
{
    ProtocolOutputBatch output;
    MotorDiscoveryResult *discovery;
    memset(&application_persistence, 0, sizeof(application_persistence));
    memset(&application_persistence_transaction, 0, sizeof(application_persistence_transaction));
    aethor_app_init(1000U, 1234U);
    assert(request("0 hello", 2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    fixture_disabled_motor7(109000U);
    discovery = &application_motor_runtime.discovery.results[6];
    discovery->observed_control_mode = mode;
    discovery->ranges.position_max_rad = position_range;
    discovery->ranges.torque_max_nm = 10.0F;
    discovery->software_version = 808597046U; discovery->sub_version = 49U;
    discovery->hardware_version = 1U;
    aethor_app_integrated_set_can_idle(1U, 104000U);
    aethor_app_integrated_set_can_idle(1U, 110000U);
    aethor_app_persistence_observe_link(0x80000001UL, 110000U);
    return 110010U;
}

/** @brief Builds a real eight-byte feedback wire sample at the narrow VMAX200 zero quantization bin. */
static CanFrame readonly_feedback(uint8_t state)
{
    CanFrame frame = {23U, 8U, {7U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 25U}};
    frame.data[0] |= (uint8_t)(state << 4U);
    return frame;
}

/** @brief Converts a native synthetic float without aliasing or numeric conversion. */
static uint32_t readonly_float_bits(float value)
{ uint32_t raw; memcpy(&raw, &value, sizeof(raw)); return raw; }

/** @brief Supplies one value from the fixed all45 synthetic read-only register baseline. */
static uint32_t readonly_register(unsigned index)
{
    if (index == 7U) return 23U;
    if (index == 8U) return 7U;
    if (index == 10U) return application_persistence.confirmed_mode;
    if (index == 14U) return 808597046U;
    if (index == 36U) return 49U;
    if (index == 21U) return readonly_float_bits(application_persistence.ranges.position_max_rad);
    if (index == 22U) return readonly_float_bits(200.0F);
    if (index == 23U) return readonly_float_bits(10.0F);
    return motor_persistence_is_unsigned(index) ? 1U : readonly_float_bits(1.0F);
}

/** @brief Tracks the source wire values independently from the adapter's mutable confirmed mapping. */
static float b1_source_pmax, b1_native_position, b1_fd_offset, b1_pm_offset, b1_u_off;
static uint32_t b1_source_mode;

/** @brief Packs the synthetic native position using the currently confirmed 16bit range. */
static CanFrame b1_feedback(float native_position, uint8_t state)
{
    CanFrame frame = readonly_feedback(state);
    float pmax = application_persistence.ranges.position_max_rad;
    uint32_t encoded = (uint32_t)((native_position + pmax) * 65535.0F / (2.0F * pmax));
    frame.data[1] = (uint8_t)(encoded >> 8U); frame.data[2] = (uint8_t)encoded;
    return frame;
}

/** @brief Completes exactly one strict FD pair and a subsequent independently ordered native feedback. */
static uint64_t b1_fd_unchecked(uint64_t now_us)
{
    AethorPersistenceTransfer transfer;
    CanFrame feedback = b1_feedback(b1_native_position + b1_fd_offset, S3519_DRIVER_STATE_DISABLED);
    uint64_t total = application_persistence_transaction.total_deadline_us;
    uint64_t phase = application_persistence_transaction.phase_deadline_us;
    assert(aethor_app_persistence_next_transfer(++now_us, &transfer));
    assert(transfer.kind == AETHOR_PERSISTENCE_TRANSFER_DISABLE);
    assert(aethor_app_persistence_disable_submitted(&transfer.disable_identity, ++now_us));
    assert(aethor_app_persistence_disable_transmitted(&transfer.disable_identity, 1U, 100U, ++now_us));
    assert(aethor_app_persistence_receive(&feedback, now_us + 1U, now_us + 2U,
        &transfer.disable_identity, 101U, 1U));
    assert(application_persistence_transaction.total_deadline_us == total);
    assert(application_persistence_transaction.phase_deadline_us == phase);
    return now_us + 3U;
}

/** @brief Requires a completed safe strict FD for successful paths. */
static uint64_t b1_fd(uint64_t now_us)
{
    now_us = b1_fd_unchecked(now_us);
    assert(application_persistence.health_proven && !application_persistence.disable_wait_rx);
    return now_us;
}

/** @brief Returns fixed all45 source or authorized target values, keeping the other24 RW raw words unchanged. */
static uint32_t b1_register(unsigned index, uint8_t target)
{
    if (index == 10U) return target ? 2U : b1_source_mode;
    if (index == 21U) return readonly_float_bits(target ? 512.0F : b1_source_pmax);
    if (motor_persistence_rid(index) == 0x50U) return readonly_float_bits(b1_native_position + b1_pm_offset);
    if (motor_persistence_rid(index) == 0x32U) return readonly_float_bits(b1_u_off);
    return readonly_register(index);
}

/** @brief Drives one exact immutable parameter pair, covering ACK before and after actual-TX observation. */
static uint64_t b1_pair(uint64_t now_us, uint32_t raw, uint8_t early_ack, AethorPersistenceTransfer *saved)
{
    AethorPersistenceTransfer transfer;
    CanFrame reply = {23U, 8U, {7U, 0U, 0x33U, 0U}};
    MotorPersistenceState phase = application_persistence_transaction.state;
    unsigned byte;
    now_us += 100U;
    assert(aethor_app_persistence_next_transfer(now_us, &transfer));
    assert(transfer.kind == AETHOR_PERSISTENCE_TRANSFER_PARAMETER);
    assert(transfer.parameter_identity.phase == phase && transfer.parameter_identity.request_id == 701U);
    assert(transfer.parameter_identity.boot_epoch == application_persistence.boot_id);
    assert(transfer.parameter_identity.owner_generation == application_persistence.owner_generation);
    assert(transfer.frame.identifier == 0x7ffU && transfer.frame.data[0] == 7U && !transfer.frame.data[1]);
    if (phase == MOTOR_PERSISTENCE_PREPARE_WRITE) {
        unsigned rid = application_persistence_transaction.write_step == 0U ? 21U : 10U;
        assert(transfer.frame.length == 8U && transfer.frame.data[2] == 0x55U && transfer.frame.data[3] == rid);
        reply.data[2] = 0x55U;
        assert(raw == (rid == 21U ? 0x44000000U : 2U));
        for (byte = 0U; byte < 4U; ++byte) assert(transfer.frame.data[byte + 4U] == (uint8_t)(raw >> (byte * 8U)));
    } else if (phase == MOTOR_PERSISTENCE_SAVE_WAIT) {
        assert(transfer.frame.length == 4U && transfer.frame.data[2] == 0xaaU && transfer.frame.data[3] == 1U);
        reply.length = 4U; reply.data[2] = 0xaaU;
    } else {
        assert(transfer.frame.length == 4U && transfer.frame.data[2] == 0x33U);
        assert(transfer.frame.data[3] == motor_persistence_rid(application_persistence_transaction.read_index));
    }
    if (saved) *saved = transfer;
    assert(!aethor_app_persistence_next_transfer(now_us + 1U, &transfer));
    assert(aethor_app_persistence_parameter_submitted(&transfer.parameter_identity, now_us + 2U));
    reply.data[3] = transfer.frame.data[3];
    for (byte = 0U; byte < 4U; ++byte) reply.data[byte + 4U] = (uint8_t)(raw >> (8U * byte));
    if (early_ack) {
        assert(aethor_app_persistence_receive(&reply, now_us + 3U, now_us + 4U, NULL, 0U, 0U));
        assert(application_persistence_transaction.pending);
        aethor_app_persistence_parameter_transmitted(&transfer.parameter_identity, 1U, now_us + 3U, now_us + 5U);
    } else {
        aethor_app_persistence_parameter_transmitted(&transfer.parameter_identity, 1U, now_us + 3U, now_us + 4U);
        assert(aethor_app_persistence_receive(&reply, now_us + 5U, now_us + 6U, NULL, 0U, 0U));
    }
    assert(!application_persistence_transaction.pending && !application_persistence.parameter_outstanding);
    return now_us + 7U;
}

/** @brief Reads a full actual phase without borrowing old health or inventing readback evidence. */
static uint64_t b1_read45_unready(uint64_t now_us, uint8_t target, uint8_t early_ack)
{
    unsigned index;
    for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index) {
        if (!application_persistence.health_proven || now_us - application_persistence.health.observed_us >= 50000ULL)
            now_us = b1_fd(now_us);
        now_us = b1_pair(now_us, b1_register(index, target), early_ack, NULL);
        assert(!application_persistence.fault);
    }
    assert(!application_persistence.snapshot_ready);
    return now_us;
}

/** @brief Publishes a complete snapshot only after its independent new native-position proof. */
static uint64_t b1_read45(uint64_t now_us, uint8_t target, uint8_t early_ack)
{
    now_us = b1_read45_unready(now_us, target, early_ack);
    now_us = b1_fd(now_us);
    assert(application_persistence.snapshot_ready);
    return now_us;
}

/** @brief Opens one synthetic boot with a true nonzero native reference and independent source registers. */
static uint64_t b1_begin(uint32_t mode, float pmax)
{
    uint64_t now_us = readonly_fixture(mode, pmax);
    b1_source_mode = mode; b1_source_pmax = pmax; b1_native_position = 0.25F; b1_fd_offset = b1_pm_offset = 0.0F; b1_u_off = 1.0F;
    application_motor_runtime.bank.motors[6].feedback.position_rad = b1_native_position;
    assert(aethor_app_persistence_begin_readonly(701U, now_us));
    now_us = b1_fd(now_us);
    return now_us;
}

/** @brief Completes and publishes the initial source baseline for successful operation. */
static uint64_t b1_baseline(uint32_t mode, float pmax, uint8_t early_ack)
{ return b1_read45(b1_begin(mode, pmax), 0U, early_ack); }

/** @brief Getter exports preserve exact identity, raw and six original observation times without any state mutation. */
static void b1_export(uint8_t phase)
{
    AethorPersistenceItemView view;
    AethorPersistenceStatus status;
    AethorPersistenceHealthView health;
    unsigned index;
    uint64_t total = application_persistence_transaction.total_deadline_us;
    uint64_t deadline = application_persistence_transaction.phase_deadline_us;
    uint64_t last = application_persistence.last_us;
    for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index) {
        const MotorPersistenceItem *item = &application_persistence_transaction.current.items[index];
        assert(aethor_app_persistence_get_item(701U, phase, index, &view));
        assert(motor_persistence_identity_equal(&view.item.identity, &item->identity));
        assert(view.rid == motor_persistence_rid(index) && view.is_unsigned == motor_persistence_is_unsigned(index));
        assert(view.item.raw == item->raw && view.item.valid);
        assert(view.item.created_us == item->created_us && view.item.submitted_us == item->submitted_us);
        assert(view.item.boundary_us == item->boundary_us && view.item.send_completed_observed_us == item->send_completed_observed_us);
        assert(view.item.rx_lower_us == item->rx_lower_us && view.item.received_us == item->received_us);
        assert(view.item.created_us < view.item.submitted_us && view.item.submitted_us <= view.item.rx_lower_us);
        assert(view.item.rx_lower_us <= view.item.received_us);
    }
    assert(!aethor_app_persistence_get_item(702U, phase, 0U, &view));
    assert(!aethor_app_persistence_get_item(701U, phase + 1U, 0U, &view));
    assert(!aethor_app_persistence_get_item(701U, phase, 45U, &view));
    assert(aethor_app_persistence_get_status(&status) && status.snapshot_ready && status.snapshot_phase == phase);
    assert(aethor_app_persistence_get_health(701U, &health) && health.health_proven && health.continuity_checked);
    assert(health.reference_native_position == 0.25F && fabsf(health.native_position - 0.25F) <= health.continuity_tolerance);
    assert(application_persistence.last_us == last && application_persistence_transaction.total_deadline_us == total);
    assert(application_persistence_transaction.phase_deadline_us == deadline);
}

/** @brief Performs all fixed writes, acquiring a new strict FD after each confirmed mapping change. */
static uint64_t b1_prepare(uint64_t now_us, uint8_t early_ack)
{
    uint64_t total = application_persistence_transaction.total_deadline_us;
    AethorPersistenceTransfer unused;
    assert(!aethor_app_persistence_prepare(702U, now_us));
    assert(aethor_app_persistence_prepare(701U, ++now_us));
    assert(!aethor_app_persistence_prepare(701U, now_us));
    assert(application_persistence_transaction.total_deadline_us == total);
    while (application_persistence_transaction.state == MOTOR_PERSISTENCE_PREPARE_WRITE) {
        if (!application_persistence.health_proven) now_us = b1_fd(now_us);
        now_us = b1_pair(now_us, application_persistence_transaction.write_step == 0U ? 0x44000000U : 2U, early_ack, NULL);
        assert(!application_persistence.fault);
        assert(!application_persistence.health_proven);
        assert(application_motor_runtime.discovery.results[6].observed_control_mode == b1_source_mode);
        assert(application_motor_runtime.discovery.results[6].ranges.position_max_rad == b1_source_pmax);
    }
    assert(!aethor_app_persistence_prepare(701U, now_us));
    now_us = b1_read45(now_us, 1U, early_ack);
    assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_PREPARED);
    assert(!aethor_app_persistence_next_transfer(++now_us, &unused));
    b1_export(MOTOR_PERSISTENCE_PREPARE_VERIFY);
    return now_us;
}

/** @brief Exercises all four legitimate sources, one Save, exact journal retention and bounded terminal lifetime. */
static void b1_success(uint32_t mode, float pmax, uint8_t early_ack)
{
    uint64_t now_us = b1_baseline(mode, pmax, early_ack);
    AethorPersistenceTransfer save;
    AethorPersistenceSaveJournal journal, later;
    uint32_t compact;
    assert(aethor_app_persistence_get_baseline_raw(701U, 10U, &compact) && compact == mode);
    b1_export(MOTOR_PERSISTENCE_BASELINE);
    if (mode == 2U && pmax == 512.0F) {
        assert(application_persistence_transaction.skip_save && !aethor_app_persistence_prepare(701U, ++now_us));
        assert(!aethor_app_persistence_get_save_journal(701U, &journal));
    } else {
        now_us = b1_prepare(now_us, early_ack);
        assert(aethor_app_persistence_request_save(701U, ++now_us));
        assert(!aethor_app_persistence_request_save(701U, now_us));
        assert(!aethor_app_persistence_commit_save(701U, now_us));
        now_us = b1_read45(now_us, 1U, early_ack);
        b1_export(MOTOR_PERSISTENCE_SAVE_VERIFY);
        assert(application_persistence.health.observed_us > application_persistence_transaction.verified_us);
        assert(aethor_app_persistence_commit_save(701U, ++now_us));
        assert(!aethor_app_persistence_commit_save(701U, now_us));
        now_us = b1_pair(now_us, 0U, early_ack, &save);
        assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_SETTLE);
        assert(aethor_app_persistence_get_save_journal(701U, &journal));
        assert(journal.valid && journal.tx.identifier == 0x7ffU && journal.tx.length == 4U);
        assert(!memcmp(journal.tx.data, save.frame.data, 4U));
        assert(journal.ack.identifier == 23U && journal.ack.length == 4U && journal.ack.data[2] == 0xaaU);
        assert(motor_persistence_identity_equal(&journal.receipt.identity, &save.parameter_identity));
        assert(journal.receipt.created_us < journal.receipt.submitted_us);
        assert(journal.receipt.boundary_us == journal.receipt.submitted_us);
        assert(journal.receipt.send_completed_observed_us >= journal.receipt.submitted_us);
        assert(journal.receipt.rx_lower_us >= journal.receipt.submitted_us);
        assert(journal.settle_until_us >= journal.receipt.received_us + 30000ULL);
        aethor_app_persistence_service(journal.settle_until_us - 1U);
        assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_SETTLE);
        now_us = journal.settle_until_us;
        aethor_app_persistence_service(now_us);
        assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_READBACK);
        now_us = b1_read45(now_us, 1U, early_ack);
        assert(application_persistence_transaction.current.items[0].created_us >= journal.settle_until_us);
        b1_export(MOTOR_PERSISTENCE_READBACK);
        assert(aethor_app_persistence_get_save_journal(701U, &later));
        assert(!memcmp(&journal, &later, sizeof(journal)));
        assert(!application_persistence_transaction.save_uncertain && application_persistence_transaction.save_used);
    }
    assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_COMPLETE);
    assert(aethor_app_persistence_locked() && application_persistence.snapshot_ready);
    assert(!aethor_app_persistence_request_save(701U, ++now_us) && !aethor_app_persistence_commit_save(701U, now_us));
    /* Keep host export pauses alive with new FD, never renewing the fixed terminal total budget. */
    now_us += 50001ULL;
    now_us = b1_fd(now_us);
    aethor_app_persistence_service(application_persistence_transaction.total_deadline_us);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_TIMEOUT);
    assert(aethor_app_persistence_locked() && !application_persistence.snapshot_ready);
    printf("PERSISTENCE_APP_B1_SUCCESS mode=%lu pmax=%.1f early_ack=%u PASS\n", (unsigned long)mode, (double)pmax, early_ack);
}

/** @brief Reaches each real stage through the same bounded submitted/TX/RX path used for success. */
static uint64_t b1_reach(MotorPersistenceState state)
{
    uint64_t now_us = b1_begin(1U, 12.5F);
    if (state == MOTOR_PERSISTENCE_BASELINE) return now_us;
    now_us = b1_read45(now_us, 0U, 0U);
    if (state == MOTOR_PERSISTENCE_BASELINE_READY) return now_us;
    if (state == MOTOR_PERSISTENCE_PREPARE_WRITE || state == MOTOR_PERSISTENCE_PREPARE_VERIFY) {
        assert(aethor_app_persistence_prepare(701U, ++now_us));
        if (state == MOTOR_PERSISTENCE_PREPARE_WRITE) return now_us;
        while (application_persistence_transaction.state == MOTOR_PERSISTENCE_PREPARE_WRITE) {
            if (!application_persistence.health_proven) now_us = b1_fd(now_us);
            now_us = b1_pair(now_us, application_persistence_transaction.write_step ? 2U : 0x44000000U, 0U, NULL);
        }
        return b1_fd(now_us);
    }
    now_us = b1_prepare(now_us, 0U);
    if (state == MOTOR_PERSISTENCE_PREPARED) return now_us;
    assert(aethor_app_persistence_request_save(701U, ++now_us));
    if (state == MOTOR_PERSISTENCE_SAVE_VERIFY) return now_us;
    now_us = b1_read45(now_us, 1U, 0U);
    if (state == MOTOR_PERSISTENCE_SAVE_HEALTH) return now_us;
    assert(aethor_app_persistence_commit_save(701U, ++now_us));
    if (state == MOTOR_PERSISTENCE_SAVE_WAIT) return now_us;
    now_us = b1_pair(now_us, 0U, 0U, NULL);
    if (state == MOTOR_PERSISTENCE_SETTLE) return now_us;
    now_us = application_persistence_save_journal.settle_until_us;
    aethor_app_persistence_service(now_us);
    if (state == MOTOR_PERSISTENCE_READBACK) return now_us;
    return b1_read45(now_us, 1U, 0U);
}

/** @brief Every real stage latches bad-before-good, STOP, full CDC/link and source discovery changes, including COMPLETE. */
static void b1_fault_matrix(void)
{
    static const MotorPersistenceState stages[] = {MOTOR_PERSISTENCE_BASELINE, MOTOR_PERSISTENCE_BASELINE_READY,
        MOTOR_PERSISTENCE_PREPARE_WRITE, MOTOR_PERSISTENCE_PREPARE_VERIFY, MOTOR_PERSISTENCE_PREPARED,
        MOTOR_PERSISTENCE_SAVE_VERIFY, MOTOR_PERSISTENCE_SAVE_HEALTH, MOTOR_PERSISTENCE_SAVE_WAIT,
        MOTOR_PERSISTENCE_SETTLE, MOTOR_PERSISTENCE_READBACK, MOTOR_PERSISTENCE_COMPLETE};
    unsigned stage, failure;
    for (stage = 0U; stage < sizeof(stages) / sizeof(stages[0]); ++stage) {
        for (failure = 0U; failure < 5U; ++failure) {
            uint64_t now_us = b1_reach(stages[stage]);
            CanFrame good = b1_feedback(0.25F, S3519_DRIVER_STATE_DISABLED);
            CanFrame bad = b1_feedback(0.25F, S3519_DRIVER_STATE_ENABLED);
            AethorPersistenceTransfer transfer;
            MotorPersistenceError expected;
            if (failure == 0U) {
                assert(aethor_app_persistence_receive(&bad, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
                assert(aethor_app_persistence_receive(&good, now_us + 3U, now_us + 4U, NULL, 0U, 0U));
                expected = MOTOR_PERSISTENCE_ERROR_HEALTH;
                if (stages[stage] == MOTOR_PERSISTENCE_PREPARE_WRITE)
                    assert(application_persistence_transaction.error == MOTOR_PERSISTENCE_ERROR_HEALTH);
            } else if (failure == 1U) {
                application_debug_ui.mailbox.stop_intent_mask = 0x40U;
                aethor_app_persistence_service(now_us + 1U); expected = MOTOR_PERSISTENCE_ERROR_STOP;
                application_debug_ui.mailbox.stop_intent_mask = 0U;
            } else if (failure == 2U) {
                uint32_t original = application_persistence.link_word;
                aethor_app_persistence_observe_link(original + 1U, now_us + 1U);
                aethor_app_persistence_observe_link(original, now_us + 2U); expected = MOTOR_PERSISTENCE_ERROR_CONTEXT;
            } else if (failure == 3U) {
                application_motor_runtime.discovery.results[6].communication_timeout_raw ^= 1U;
                aethor_app_persistence_service(now_us + 1U); expected = MOTOR_PERSISTENCE_ERROR_CONTEXT;
            } else {
                aethor_app_persistence_service(application_persistence.last_us - 1U); expected = MOTOR_PERSISTENCE_ERROR_CLOCK;
            }
            printf("B1_FAULT stage=%u failure=%u expected=%u actual=%u ready=%u\n", (unsigned)stages[stage], failure, (unsigned)expected, (unsigned)application_persistence.fault, application_persistence.snapshot_ready);
            assert(application_persistence.fault == expected && !application_persistence.snapshot_ready);
            assert(!application_persistence.health_proven && aethor_app_persistence_locked());
            assert(!aethor_app_persistence_next_transfer(now_us + 5U, &transfer));
            assert(!aethor_app_persistence_prepare(701U, now_us + 5U));
            assert(!aethor_app_persistence_request_save(701U, now_us + 5U));
            assert(!aethor_app_persistence_commit_save(701U, now_us + 5U));
            assert(!aethor_app_persistence_begin_readonly(702U, now_us + 5U));
        }
    }
    puts("PERSISTENCE_APP_B1_STAGE_FAULT_MATRIX_PASS cases=55");
}

/** @brief Range/mode unknown windows isolate feedback, preserve source mapping and require a new FD after either callback order. */
static void b1_unknown_mapping(void)
{
    unsigned write, early;
    for (early = 0U; early < 2U; ++early) {
        uint64_t now_us = b1_baseline(1U, 12.5F, 0U);
        assert(aethor_app_persistence_prepare(701U, ++now_us));
        for (write = 0U; write < 2U; ++write) {
            AethorPersistenceTransfer transfer;
            CanFrame good = b1_feedback(0.25F, S3519_DRIVER_STATE_DISABLED);
            CanFrame reply = {23U, 8U, {7U, 0U, 0x55U, 0U}};
            uint32_t raw = write ? 2U : 0x44000000U;
            unsigned byte;
            if (!application_persistence.health_proven) now_us = b1_fd(now_us);
            assert(aethor_app_persistence_next_transfer(++now_us, &transfer));
            assert(aethor_app_persistence_parameter_submitted(&transfer.parameter_identity, ++now_us));
            assert(!application_persistence.health_proven);
            assert(write ? !application_persistence.mode_known : !application_persistence.pmax_known);
            assert(aethor_app_persistence_receive(&good, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
            now_us += 3U;
            assert(!application_persistence.health_proven && !application_persistence.fault);
            reply.data[3] = transfer.frame.data[3];
            for (byte = 0U; byte < 4U; ++byte) reply.data[byte + 4U] = (uint8_t)(raw >> (8U * byte));
            if (early) {
                assert(aethor_app_persistence_receive(&reply, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
                now_us += 3U;
                assert(write ? !application_persistence.mode_known : !application_persistence.pmax_known);
            }
            aethor_app_persistence_parameter_transmitted(&transfer.parameter_identity, 1U, now_us + 1U, now_us + 2U);
            now_us += 3U;
            if (!early) {
                assert(write ? !application_persistence.mode_known : !application_persistence.pmax_known);
                assert(aethor_app_persistence_receive(&reply, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
                now_us += 3U;
            }
            assert(application_persistence.pmax_known && application_persistence.mode_known);
            assert(!application_persistence.health_proven && !application_persistence.fault);
            assert(application_persistence.ranges.position_max_rad == 512.0F);
            assert(application_persistence.confirmed_mode == (write ? 2U : 1U));
            assert(aethor_app_persistence_receive(&good, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
            now_us += 3U;
            assert(!application_persistence.health_proven);
        }
    }
    puts("PERSISTENCE_APP_B1_UNKNOWN_MAPPING_ISOLATION_PASS cases=4");
}

/** @brief Both submitted unknown-mapping windows retain raw state/temperature safety without decoding opaque position/velocity/torque. */
static void b1_unknown_mapping_bad_then_good(void)
{
    unsigned write, variant, order;
    for (write = 0U; write < 2U; ++write) {
        for (variant = 0U; variant < 7U; ++variant) {
            for (order = 0U; order < 3U; ++order) {
                uint64_t now_us = b1_baseline(1U, 12.5F, 0U);
                AethorPersistenceTransfer transfer, unused;
                MotorPersistenceIdentity cancellation;
                CanFrame good = b1_feedback(0.25F, S3519_DRIVER_STATE_DISABLED);
                CanFrame bad, reply = {23U, 8U, {7U, 0U, 0x55U, 0U}};
                uint32_t raw = write ? 2U : 0x44000000U;
                uint64_t total, deadline;
                float proven_native;
                unsigned byte;
                assert(aethor_app_persistence_prepare(701U, ++now_us));
                if (write) {
                    now_us = b1_pair(now_us, 0x44000000U, 0U, NULL);
                    now_us = b1_fd(now_us);
                }
                assert(aethor_app_persistence_next_transfer(++now_us, &transfer));
                assert(aethor_app_persistence_parameter_submitted(&transfer.parameter_identity, ++now_us));
                assert(write ? !application_persistence.mode_known : !application_persistence.pmax_known);
                total = application_persistence_transaction.total_deadline_us;
                deadline = application_persistence_transaction.phase_deadline_us;
                proven_native = application_persistence.native_position;
                reply.data[3] = transfer.frame.data[3];
                for (byte = 0U; byte < 4U; ++byte) reply.data[byte + 4U] = (uint8_t)(raw >> (8U * byte));
                if (order == 1U) {
                    assert(aethor_app_persistence_receive(&reply, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
                    now_us += 3U;
                } else if (order == 2U) {
                    aethor_app_persistence_parameter_transmitted(&transfer.parameter_identity, 1U, now_us + 1U, now_us + 2U);
                    now_us += 3U;
                }
                /* Opaque raw motion bytes are neither trusted nor decoded with the old mapping. */
                good.data[1] = good.data[3] = good.data[4] = good.data[5] = 0xffU; good.data[2] = 0x34U;
                assert(aethor_app_persistence_receive(&good, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
                now_us += 3U;
                assert(!application_persistence.fault && !application_persistence.health_proven);
                assert(application_persistence.native_position == proven_native);
                bad = good;
                if (variant < 3U) {
                    uint8_t state = variant == 0U ? S3519_DRIVER_STATE_ENABLED : variant == 1U ? S3519_DRIVER_STATE_FAULT_MINIMUM : 15U;
                    bad.data[0] = (uint8_t)(7U | (state << 4U));
                    /* A feedback position byte matching a parameter opcode cannot hide an unsafe raw state. */
                    bad.data[2] = 0x33U;
                } else if (variant < 5U) bad.data[6] = variant == 3U ? 50U : 60U;
                else bad.data[7] = variant == 5U ? 50U : 60U;
                assert(aethor_app_persistence_receive(&bad, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
                now_us += 3U;
                printf("UNKNOWN_BAD_STICKY write=%u variant=%u order=%u fault=%u pure_error=%u debt=%u\n", write, variant,
                    order, (unsigned)application_persistence.fault, (unsigned)application_persistence_transaction.error,
                    application_persistence.parameter_outstanding);
                assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_HEALTH);
                assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_ABORTED);
                assert(application_persistence_transaction.error == MOTOR_PERSISTENCE_ERROR_HEALTH);
                assert(application_debug_ui.mailbox.stop_intent_mask & 0x40U);
                assert(!application_persistence.health_proven && !application_persistence.snapshot_ready);
                assert(aethor_app_persistence_receive(&good, now_us + 1U, now_us + 2U,
                    &application_persistence.health_identity, 101U, 1U));
                now_us += 3U;
                if (order != 2U) {
                    assert(aethor_app_persistence_cancel_identity(&cancellation));
                    assert(motor_persistence_identity_equal(&cancellation, &transfer.parameter_identity));
                    aethor_app_persistence_parameter_transmitted(&transfer.parameter_identity, 1U, now_us + 1U, now_us + 2U);
                    now_us += 3U;
                }
                assert(aethor_app_persistence_receive(&reply, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
                now_us += 3U;
                assert(!application_persistence.parameter_outstanding);
                assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_HEALTH && !application_persistence.health_proven);
                assert(write ? !application_persistence.mode_known : !application_persistence.pmax_known);
                assert(application_persistence.native_position == proven_native);
                assert(!aethor_app_persistence_next_transfer(++now_us, &unused));
                assert(!aethor_app_persistence_prepare(701U, now_us) && !aethor_app_persistence_commit_save(701U, now_us));
                assert(aethor_app_persistence_locked() && !application_persistence_transaction.save_used);
                assert(application_persistence_transaction.total_deadline_us == total && application_persistence_transaction.phase_deadline_us == deadline);
            }
        }
    }
    puts("PERSISTENCE_APP_B1_UNKNOWN_MAPPING_BAD_GOOD_STICKY_PASS cases=42");
}

/** @brief All stable phases reject each independent native FD or vendor RID0x50 p_m discontinuity with the specified unchanged threshold. */
static void b1_continuity_matrix(void)
{
    unsigned stage, source;
    for (stage = 0U; stage < 4U; ++stage) {
        for (source = 0U; source < 2U; ++source) {
            uint64_t now_us;
            if (stage == 0U) now_us = b1_begin(1U, 12.5F);
            else if (stage == 1U) now_us = b1_reach(MOTOR_PERSISTENCE_PREPARE_VERIFY);
            else if (stage == 2U) now_us = b1_reach(MOTOR_PERSISTENCE_SAVE_VERIFY);
            else now_us = b1_reach(MOTOR_PERSISTENCE_READBACK);
            if (source) b1_pm_offset = 0.2F;
            now_us = b1_read45_unready(now_us, (uint8_t)(stage != 0U), 0U);
            if (!source) b1_fd_offset = 0.2F;
            now_us = b1_fd_unchecked(now_us);
            assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_HEALTH);
            assert(!application_persistence.snapshot_ready && !application_persistence.continuity_checked);
            assert(!aethor_app_persistence_commit_save(701U, ++now_us));
        }
    }
    puts("PERSISTENCE_APP_B1_NATIVE_PM_CONTINUITY_PASS cases=8");
}

/** @brief Save cannot borrow a pre-verify FD, bypass stable continuity, or publish journal before exact actual TX. */
static void b1_save_gates_and_journal(void)
{
    uint64_t now_us = b1_reach(MOTOR_PERSISTENCE_SAVE_VERIFY);
    AethorPersistenceTransfer save, unused;
    AethorPersistenceSaveJournal journal;
    CanFrame ack = {23U, 4U, {7U, 0U, 0xaaU, 1U}};
    uint64_t total = application_persistence_transaction.total_deadline_us;
    uint64_t deadline = application_persistence_transaction.phase_deadline_us;
    now_us = b1_read45_unready(now_us, 1U, 0U);
    assert(!aethor_app_persistence_commit_save(701U, ++now_us));
    assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_SAVE_HEALTH);
    now_us = b1_fd(now_us);
    assert(aethor_app_persistence_commit_save(701U, ++now_us));
    assert(!aethor_app_persistence_commit_save(701U, now_us));
    assert(aethor_app_persistence_next_transfer(++now_us, &save));
    assert(aethor_app_persistence_parameter_submitted(&save.parameter_identity, ++now_us));
    assert(application_persistence_transaction.save_used && application_persistence_transaction.save_uncertain);
    assert(!aethor_app_persistence_get_save_journal(701U, &journal));
    assert(aethor_app_persistence_receive(&ack, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
    now_us += 3U;
    assert(!aethor_app_persistence_get_save_journal(701U, &journal));
    assert(!aethor_app_persistence_next_transfer(++now_us, &unused));
    aethor_app_persistence_preempt(++now_us);
    aethor_app_persistence_parameter_transmitted(&save.parameter_identity, 1U, now_us + 1U, now_us + 2U);
    now_us += 3U;
    assert(!application_persistence.parameter_outstanding);
    assert(!aethor_app_persistence_get_save_journal(701U, &journal));
    assert(application_persistence_transaction.save_used && application_persistence_transaction.save_uncertain);
    assert(!aethor_app_persistence_next_transfer(++now_us, &unused));
    assert(!aethor_app_persistence_commit_save(701U, now_us));
    assert(application_persistence_transaction.total_deadline_us == total && application_persistence_transaction.phase_deadline_us == deadline);
    puts("PERSISTENCE_APP_B1_POSTVERIFY_FD_ONCE_JOURNAL_GATES_PASS");
}

/** @brief New target verification rejects a critical old source value at the actual item, including deferred ACK. */
static void b1_target_rollback(void)
{
    unsigned stage, changed, early, index;
    for (stage = 0U; stage < 3U; ++stage) {
        for (changed = 0U; changed < 2U; ++changed) {
            for (early = 0U; early < 2U; ++early) {
                uint64_t now_us = b1_reach(stage == 0U ? MOTOR_PERSISTENCE_PREPARE_VERIFY :
                    stage == 1U ? MOTOR_PERSISTENCE_SAVE_VERIFY : MOTOR_PERSISTENCE_READBACK);
                unsigned bad_index = changed ? 21U : 10U;
                for (index = 0U; index <= bad_index; ++index) {
                    uint32_t raw = index == bad_index ? b1_register(index, 0U) : b1_register(index, 1U);
                    now_us = b1_pair(now_us, raw, (uint8_t)early, NULL);
                }
                assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CONTEXT);
                assert(!application_persistence.health_proven && aethor_app_persistence_locked());
                assert(application_motor_runtime.discovery.results[6].observed_control_mode == 1U);
            }
        }
    }
    puts("PERSISTENCE_APP_B1_TARGET_CRITICAL_ROLLBACK_PASS cases=12");
}

/** @brief Every transmitting phase retains exact original debt after revocation, rejects bad receipt time, and never retries missing ACK. */
static void b1_receipt_matrix(void)
{
    static const MotorPersistenceState stages[] = {MOTOR_PERSISTENCE_BASELINE, MOTOR_PERSISTENCE_PREPARE_WRITE,
        MOTOR_PERSISTENCE_PREPARE_VERIFY, MOTOR_PERSISTENCE_SAVE_VERIFY, MOTOR_PERSISTENCE_SAVE_WAIT,
        MOTOR_PERSISTENCE_READBACK};
    unsigned stage, failure;
    for (stage = 0U; stage < sizeof(stages) / sizeof(stages[0]); ++stage) {
        for (failure = 0U; failure < 5U; ++failure) {
            uint64_t now_us = b1_reach(stages[stage]);
            AethorPersistenceTransfer transfer, unused;
            MotorPersistenceIdentity original, wrong;
            AethorPersistenceSaveJournal journal;
            uint8_t already_saved = application_persistence_transaction.save_used;
            assert(aethor_app_persistence_next_transfer(++now_us, &transfer));
            assert(transfer.kind == AETHOR_PERSISTENCE_TRANSFER_PARAMETER);
            assert(aethor_app_persistence_parameter_submitted(&transfer.parameter_identity, ++now_us));
            original = transfer.parameter_identity; wrong = original; ++wrong.sequence;
            if (failure < 2U) {
                if (!failure) aethor_app_persistence_preempt(++now_us);
                else aethor_app_persistence_observe_link(application_persistence.link_word + 1U, ++now_us);
                assert(aethor_app_persistence_cancel_identity(&wrong));
                assert(motor_persistence_identity_equal(&wrong, &original));
                wrong = original; ++wrong.sequence;
                aethor_app_persistence_parameter_transmitted(&wrong, 1U, now_us + 1U, now_us + 2U);
                assert(application_persistence.parameter_outstanding);
                aethor_app_persistence_parameter_transmitted(&original, 1U, now_us + 3U, now_us + 4U);
                now_us += 5U;
                assert(!application_persistence.parameter_outstanding);
            } else if (failure == 2U) {
                aethor_app_persistence_parameter_transmitted(&original, 1U, now_us - 1U, now_us + 1U);
                now_us += 2U;
                assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_TX);
            } else if (failure == 3U) {
                aethor_app_persistence_parameter_transmitted(&original, 0U, now_us + 1U, now_us + 2U);
                now_us += 3U;
                assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_TX);
            } else {
                aethor_app_persistence_parameter_transmitted(&original, 1U, now_us + 1U, now_us + 2U);
                now_us = application_persistence_transaction.frame_deadline_us;
                aethor_app_persistence_service(now_us);
                assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_TIMEOUT);
            }
            assert(application_persistence.fault && aethor_app_persistence_locked());
            assert(!aethor_app_persistence_next_transfer(++now_us, &unused));
            assert(!aethor_app_persistence_commit_save(701U, now_us));
            if (stages[stage] == MOTOR_PERSISTENCE_SAVE_WAIT) {
                assert(application_persistence_transaction.save_used && application_persistence_transaction.save_uncertain);
                assert(!aethor_app_persistence_get_save_journal(701U, &journal));
            } else assert(application_persistence_transaction.save_used == already_saved);
        }
    }
    puts("PERSISTENCE_APP_B1_RECEIPT_DEBT_TIME_NO_ACK_PASS cases=30");
}

/** @brief In-flight FD exports retain the last completed proof identity, and 900 refreshes cannot overrun total lifetime. */
static void b1_bounded_refresh_and_export(void)
{
    uint64_t now_us = b1_baseline(2U, 512.0F, 0U);
    AethorPersistenceHealthView old, inflight;
    AethorPersistenceTransfer transfer;
    unsigned refreshes = 0U;
    assert(aethor_app_persistence_get_health(701U, &old));
    now_us += 50001ULL;
    assert(aethor_app_persistence_next_transfer(++now_us, &transfer));
    assert(aethor_app_persistence_get_health(701U, &inflight));
    assert(debug_ui_static_tx_identity_equal(&old.proof_identity, &inflight.proof_identity));
    assert(!debug_ui_static_tx_identity_equal(&transfer.disable_identity, &inflight.proof_identity));
    assert(aethor_app_persistence_disable_submitted(&transfer.disable_identity, ++now_us));
    assert(aethor_app_persistence_disable_transmitted(&transfer.disable_identity, 1U, 100U, ++now_us));
    {
        CanFrame feedback = b1_feedback(0.25F, S3519_DRIVER_STATE_DISABLED);
        assert(aethor_app_persistence_receive(&feedback, now_us + 1U, now_us + 2U, &transfer.disable_identity, 101U, 1U));
        now_us += 3U;
    }
    while (now_us + 50010ULL < application_persistence_transaction.total_deadline_us) {
        now_us = b1_fd(now_us + 50001ULL); ++refreshes;
    }
    assert(refreshes > 255U && application_persistence.disable_count > 255U);
    aethor_app_persistence_service(application_persistence_transaction.total_deadline_us);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_TIMEOUT);
    assert(!aethor_app_persistence_next_transfer(application_persistence_transaction.total_deadline_us + 1U, &transfer));
    printf("PERSISTENCE_APP_B1_BOUNDED_FD_EXPORT_PASS refreshes=%u limit=%u\n", refreshes, AETHOR_PERSISTENCE_DISABLE_LIMIT);
}

/** @brief Proves wire RID0x50 (p_m) and RID0x32 (u_off) cannot substitute for each other in continuity. */
static void b1_position_register_address(void)
{
    unsigned scenario, index;
    assert(motor_persistence_rid(37U) == 0x32U && motor_persistence_rid(43U) == 0x50U);
    assert(!motor_persistence_is_configuration(37U) && !motor_persistence_is_configuration(43U));
    for (scenario = 0U; scenario < 2U; ++scenario) {
        uint64_t now_us = b1_begin(1U, 12.5F);
        AethorPersistenceTransfer unused;
        float actual_position = scenario ? 0.25F : 1.0F;
        float current_offset = scenario ? 5.75F : 0.25F;
        for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index) {
            uint8_t rid = motor_persistence_rid(index);
            uint32_t raw = rid == 0x50U ? readonly_float_bits(actual_position) :
                rid == 0x32U ? readonly_float_bits(current_offset) : b1_register(index, 0U);
            now_us = b1_pair(now_us, raw, 0U, NULL);
            assert(!application_persistence.fault);
        }
        now_us = b1_fd_unchecked(now_us);
        printf("POSITION_RID_CHECK scenario=%u p_m_rid=80 p_m=%.2f u_off_rid=50 u_off=%.2f native=%.2f fault=%u ready=%u\n",
            scenario, (double)actual_position, (double)current_offset, (double)b1_native_position,
            (unsigned)application_persistence.fault, application_persistence.snapshot_ready);
        if (!scenario) {
            assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_HEALTH);
            assert(!application_persistence.snapshot_ready && !application_persistence.continuity_checked);
            assert(!aethor_app_persistence_next_transfer(++now_us, &unused));
        } else {
            assert(!application_persistence.fault && application_persistence.snapshot_ready);
            assert(application_persistence.continuity_checked && application_persistence.health_proven);
            assert(application_persistence_transaction.current.items[37].raw == readonly_float_bits(5.75F));
            assert(application_persistence_transaction.current.items[43].raw == readonly_float_bits(0.25F));
        }
    }
    puts("PERSISTENCE_APP_B1_POSITION_RID0X50_NOT_UOFF0X32_PASS cases=2");
}

/** @brief Independent u-phase current offset changes are finite RO data, not native-position discontinuities in any stable phase. */
static void b1_u_off_is_not_position(void)
{
    unsigned stage;
    for (stage = 0U; stage < 4U; ++stage) {
        uint64_t now_us;
        if (stage == 0U) now_us = b1_begin(1U, 12.5F);
        else if (stage == 1U) now_us = b1_reach(MOTOR_PERSISTENCE_PREPARE_VERIFY);
        else if (stage == 2U) now_us = b1_reach(MOTOR_PERSISTENCE_SAVE_VERIFY);
        else now_us = b1_reach(MOTOR_PERSISTENCE_READBACK);
        b1_u_off = 5.75F + (float)stage;
        now_us = b1_read45(now_us, (uint8_t)(stage != 0U), 0U);
        assert(!application_persistence.fault && application_persistence.snapshot_ready);
        assert(application_persistence.continuity_checked && application_persistence.health_proven);
        assert(application_persistence_transaction.current.items[37].raw == readonly_float_bits(b1_u_off));
        assert(application_persistence_transaction.current.items[43].raw == readonly_float_bits(0.25F));
        if (stage != 0U) assert(application_persistence_transaction.baseline_raw[37] != readonly_float_bits(b1_u_off));
        b1_export(application_persistence_transaction.current.items[43].identity.phase);
        printf("U_OFF_NOT_POSITION stage=%u u_off=%.2f p_m=0.25 native=0.25 ready=%u now=%llu\n", stage,
            (double)b1_u_off, application_persistence.snapshot_ready, (unsigned long long)now_us);
    }
    puts("PERSISTENCE_APP_B1_U_OFF_RO_CHANGE_NOT_POSITION_PASS cases=4");
}

/** @brief Every known-mapping phase checks genuine feedback despite an opcode-valued position byte, including adjacent control. */
static void b1_known_feedback_opcode_collision(void)
{
    static const MotorPersistenceState stages[] = {MOTOR_PERSISTENCE_BASELINE, MOTOR_PERSISTENCE_BASELINE_READY,
        MOTOR_PERSISTENCE_PREPARE_WRITE, MOTOR_PERSISTENCE_PREPARE_VERIFY, MOTOR_PERSISTENCE_PREPARED,
        MOTOR_PERSISTENCE_SAVE_VERIFY, MOTOR_PERSISTENCE_SAVE_HEALTH, MOTOR_PERSISTENCE_SAVE_WAIT,
        MOTOR_PERSISTENCE_SETTLE, MOTOR_PERSISTENCE_READBACK, MOTOR_PERSISTENCE_COMPLETE};
    static const uint8_t position_bytes[] = {0x33U, 0x55U, 0xaaU, 0x34U};
    unsigned stage, position, hazard;
    for (stage = 0U; stage < sizeof(stages) / sizeof(stages[0]); ++stage) {
        for (position = 0U; position < sizeof(position_bytes) / sizeof(position_bytes[0]); ++position) {
            for (hazard = 0U; hazard < 3U; ++hazard) {
                uint64_t now_us = b1_reach(stages[stage]);
                uint64_t total = application_persistence_transaction.total_deadline_us;
                uint64_t deadline = application_persistence_transaction.phase_deadline_us;
                uint8_t saved_before = application_persistence_transaction.save_used;
                CanFrame bad = b1_feedback(0.25F, hazard ? S3519_DRIVER_STATE_DISABLED : S3519_DRIVER_STATE_ENABLED);
                CanFrame good = b1_feedback(0.25F, S3519_DRIVER_STATE_DISABLED);
                AethorPersistenceTransfer unused;
                assert(application_persistence.pmax_known && application_persistence.mode_known);
                /* data1=0x80 is unambiguously feedback, not a [ESC,00,opcode,RID] parameter header. */
                bad.data[1] = 0x80U; bad.data[2] = position_bytes[position];
                if (hazard == 1U) bad.data[6] = 60U;
                if (hazard == 2U) bad.data[7] = 60U;
                assert(aethor_app_persistence_receive(&bad, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
                assert(aethor_app_persistence_receive(&good, now_us + 3U, now_us + 4U, NULL, 0U, 0U));
                now_us += 5U;
                printf("KNOWN_COLLISION_STICKY stage=%u opcode=0x%02x hazard=%u fault=%u ready=%u health=%u\n",
                    (unsigned)stages[stage], position_bytes[position], hazard, (unsigned)application_persistence.fault,
                    application_persistence.snapshot_ready, application_persistence.health_proven);
                assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_HEALTH);
                assert(!application_persistence.snapshot_ready && !application_persistence.health_proven);
                assert(application_debug_ui.mailbox.stop_intent_mask & 0x40U);
                assert(!aethor_app_persistence_commit_save(701U, now_us));
                assert(!aethor_app_persistence_next_transfer(++now_us, &unused));
                assert(aethor_app_persistence_locked() && application_persistence_transaction.save_used == saved_before);
                assert(application_persistence_transaction.total_deadline_us == total);
                assert(application_persistence_transaction.phase_deadline_us == deadline);
            }
        }
    }
    puts("PERSISTENCE_APP_B1_KNOWN_OPCODE_COLLISION_STICKY_PASS collision=99 adjacent_control=33");
}

/** @brief Exact parameter-shaped target, wrong-master and wrong-ESC replies stay quarantined instead of becoming feedback. */
static void b1_parameter_shape_quarantine(void)
{
    static const MotorPersistenceState stages[] = {MOTOR_PERSISTENCE_BASELINE, MOTOR_PERSISTENCE_BASELINE_READY,
        MOTOR_PERSISTENCE_PREPARE_WRITE, MOTOR_PERSISTENCE_PREPARE_VERIFY, MOTOR_PERSISTENCE_PREPARED,
        MOTOR_PERSISTENCE_SAVE_VERIFY, MOTOR_PERSISTENCE_SAVE_HEALTH, MOTOR_PERSISTENCE_SAVE_WAIT,
        MOTOR_PERSISTENCE_SETTLE, MOTOR_PERSISTENCE_READBACK, MOTOR_PERSISTENCE_COMPLETE};
    unsigned stage, variant, address;
    for (stage = 0U; stage < sizeof(stages) / sizeof(stages[0]); ++stage) {
        for (variant = 0U; variant < 4U; ++variant) {
            for (address = 0U; address < 3U; ++address) {
                uint64_t now_us = b1_reach(stages[stage]);
                uint64_t health_time = application_persistence.health.observed_us;
                uint64_t deadline = application_persistence_transaction.phase_deadline_us;
                uint64_t total = application_persistence_transaction.total_deadline_us;
                uint32_t accepted = application_motor_runtime.accepted_feedback_count;
                float native = application_persistence.native_position;
                CanFrame parameter = {23U, 8U, {7U, 0U, 0x33U, 0xffU, 0U, 0U, 60U, 60U}};
                if (variant == 1U) parameter.data[2] = 0x55U;
                else if (variant >= 2U) {
                    parameter.data[2] = 0xaaU; parameter.data[3] = variant == 2U ? 1U : 2U;
                    if (variant == 2U) parameter.length = 4U;
                }
                if (address == 1U) parameter.identifier = 24U;
                else if (address == 2U) parameter.data[0] = 8U;
                assert(aethor_app_persistence_receive(&parameter, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
                assert(!application_persistence.fault);
                assert(application_persistence.native_position == native && application_persistence.health.observed_us == health_time);
                assert(application_motor_runtime.accepted_feedback_count == accepted);
                assert(application_persistence_transaction.total_deadline_us == total && application_persistence_transaction.phase_deadline_us == deadline);
            }
        }
    }
    puts("PERSISTENCE_APP_B1_EXACT_PARAMETER_AA_SHAPE_QUARANTINE_PASS cases=132");
}

/** @brief Wrong RID/opcode/master and a late exact reply cannot impersonate feedback or complete the outstanding parameter read. */
static void b1_pending_wrong_and_late_ack(void)
{
    uint64_t now_us = b1_begin(1U, 12.5F);
    AethorPersistenceTransfer transfer;
    CanFrame reply = {23U, 8U, {7U, 0U, 0x33U, 0U, 0U, 0U, 0x80U, 0x3fU}};
    CanFrame rejected;
    uint64_t observed = application_persistence.health.observed_us;
    unsigned variant;
    assert(aethor_app_persistence_next_transfer(++now_us, &transfer));
    assert(aethor_app_persistence_parameter_submitted(&transfer.parameter_identity, ++now_us));
    aethor_app_persistence_parameter_transmitted(&transfer.parameter_identity, 1U, now_us + 1U, now_us + 2U);
    now_us += 3U;
    for (variant = 0U; variant < 4U; ++variant) {
        rejected = reply;
        if (variant == 0U) rejected.data[3] = 1U;
        else if (variant == 1U) rejected.data[2] = 0x55U;
        else if (variant == 2U) rejected.identifier = 24U;
        else { rejected.length = 4U; rejected.data[2] = 0xaaU; rejected.data[3] = 1U; }
        assert(aethor_app_persistence_receive(&rejected, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
        now_us += 3U;
        assert(!application_persistence.fault && application_persistence_transaction.pending);
        assert(application_persistence_transaction.read_index == 0U && application_persistence.health.observed_us == observed);
    }
    assert(aethor_app_persistence_receive(&reply, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
    now_us += 3U;
    assert(!application_persistence_transaction.pending && application_persistence_transaction.read_index == 1U);
    assert(aethor_app_persistence_receive(&reply, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
    assert(!application_persistence.fault && application_persistence_transaction.read_index == 1U);
    assert(application_persistence.health.observed_us == observed);
    puts("PERSISTENCE_APP_B1_WRONG_LATE_ACK_QUARANTINE_PASS");
}

/** @brief Runs real facade and pure-module behavior with explicitly synthetic observations. */
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0U);
    b1_known_feedback_opcode_collision();
    b1_parameter_shape_quarantine();
    b1_pending_wrong_and_late_ack();
    b1_position_register_address();
    b1_success(1U, 12.5F, 0U);
    b1_success(1U, 12.5F, 1U);
    b1_success(2U, 12.5F, 0U);
    b1_success(1U, 512.0F, 1U);
    b1_success(2U, 512.0F, 0U);
    b1_fault_matrix();
    b1_unknown_mapping();
    b1_unknown_mapping_bad_then_good();
    b1_continuity_matrix();
    b1_u_off_is_not_position();
    b1_save_gates_and_journal();
    b1_target_rollback();
    b1_receipt_matrix();
    b1_bounded_refresh_and_export();
    assert(sizeof(application_persistence) <= 256U);
    printf("PERSISTENCE_APP_B1_STATIC transaction=%u wrapper=%u journal=%u\n", (unsigned)sizeof(application_persistence_transaction),
        (unsigned)sizeof(application_persistence), (unsigned)sizeof(application_persistence_save_journal));
    puts("PERSISTENCE_APP_B1_FACADE_SYNTHETIC_PASS");
    return 0;
}
