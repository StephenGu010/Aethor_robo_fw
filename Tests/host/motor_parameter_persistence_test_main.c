/** @file motor_parameter_persistence_test_main.c
 * @brief Offline fixed-register surface tests; never invokes transport or hardware. */
#include "motor_parameter_persistence.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/** @brief Produces IEEE754 bits without aliasing or numeric conversion. */
static uint32_t float_bits(float value)
{ uint32_t bits; memcpy(&bits, &value, sizeof(bits)); return bits; }
/** @brief Builds explicitly synthetic complete vendor observations for offline tests. */
static void baseline(MotorPersistenceSnapshot *snapshot)
{
    unsigned index;
    memset(snapshot, 0, sizeof(*snapshot));
    for (index = 0; index < 45; ++index) {
        snapshot->items[index].valid = 1U;
        snapshot->items[index].raw = motor_persistence_is_unsigned(index) ? 1U : float_bits(1.0F);
    }
    snapshot->items[7].raw = 23U; snapshot->items[8].raw = 7U;
    snapshot->items[10].raw = 1U;
    snapshot->items[14].raw = 808597046U; snapshot->items[36].raw = 49U;
    snapshot->items[21].raw = float_bits(12.5F);
    snapshot->items[22].raw = float_bits(200.0F);
    snapshot->items[23].raw = float_bits(10.0F);
}
/** @brief Rejects incomplete, nonfinite and wrong identity snapshots and unauthorized changes. */
static void snapshot_tests(void)
{
    MotorPersistenceSnapshot original, prepared;
    baseline(&original);
    assert(motor_persistence_validate(&original, 0U));
    assert(!motor_persistence_validate(&original, 1U));
    prepared = original; prepared.items[21].raw = float_bits(512.0F); prepared.items[10].raw = 2U;
    assert(motor_persistence_validate(&prepared, 1U));
    assert(motor_persistence_configuration_matches(&original, &prepared, 1U));
    assert(!motor_persistence_configuration_matches(&original, &prepared, 0U));
    prepared.items[29].raw = float_bits(2.0F);
    assert(!motor_persistence_configuration_matches(&original, &prepared, 1U));
    prepared = original; prepared.items[7].raw = 8U; assert(!motor_persistence_validate(&prepared, 0U));
    prepared = original; prepared.items[14].raw = 0U; assert(!motor_persistence_validate(&prepared, 0U));
    prepared = original; prepared.items[10].raw = 3U; assert(!motor_persistence_validate(&prepared, 0U));
    prepared = original; prepared.items[29].raw = 0x7fc00000U; assert(!motor_persistence_validate(&prepared, 0U));
    prepared = original; prepared.items[44].valid = 0U; assert(!motor_persistence_validate(&prepared, 0U));
    prepared = original; prepared.items[43].raw = float_bits(-123.0F);
    assert(motor_persistence_configuration_matches(&original, &prepared, 0U));
}
/** @brief Requires observed pending debt before either successful or failed release. */
static void tracker_tests(void)
{
    MotorPersistenceTracker tracker = {0};
    MotorPersistenceIdentity identity = {1U, 2U, 3U, 4U, 1U}, receipt;
    uint8_t success = 99U;
    assert(motor_persistence_tracker_arm(&tracker, &identity, 1U, 1U, 0U));
    assert(!motor_persistence_tracker_collect(&tracker, 1U, 0U, 0U, &receipt, &success));
    assert(!motor_persistence_tracker_arm(&tracker, &identity, 2U, 2U, 0U));
    assert(motor_persistence_tracker_collect(&tracker, 0U, 1U, 0U, &receipt, &success));
    assert(success && motor_persistence_identity_equal(&identity, &receipt));
    receipt.sequence++; assert(!motor_persistence_identity_equal(&identity, &receipt));
    assert(motor_persistence_tracker_arm(&tracker, &identity, 1U, 1U, 1U));
    assert(motor_persistence_tracker_collect(&tracker, 0U, 1U, 0U, &receipt, &success));
    assert(!success);
    assert(motor_persistence_tracker_arm(&tracker, &identity, 1U, 1U, 0U));
    motor_persistence_tracker_cancel(&tracker);
    assert(!motor_persistence_tracker_collect(&tracker, 1U, 1U, 0U, &receipt, &success));
    assert(tracker.active);
    assert(motor_persistence_tracker_collect(&tracker, 0U, 1U, 0U, &receipt, &success));
    assert(!success);
    assert(!motor_persistence_tracker_arm(&tracker, &identity, 3U, 3U, 0U));
    /* HAL may have accepted TXBAR before the asynchronous Tx scan updates TXBRP.
     * No status at this instant proves neither cancellation nor absence of future debt. */
    assert(motor_persistence_tracker_arm(&tracker, &identity, 1U, 0U, 0U));
    assert(!motor_persistence_tracker_collect(&tracker, 0U, 0U, 0U, &receipt, &success));
    assert(tracker.active);
    assert(!motor_persistence_tracker_arm(&tracker, &identity, 2U, 2U, 0U));
    assert(!motor_persistence_tracker_collect(&tracker, 1U, 0U, 0U, &receipt, &success));
    assert(motor_persistence_tracker_collect(&tracker, 0U, 1U, 0U, &receipt, &success));
    assert(success);
    /* Cancellation before the first pending observation cannot release early or
     * become successful when that original request later reaches the bus. */
    assert(motor_persistence_tracker_arm(&tracker, &identity, 1U, 0U, 0U));
    motor_persistence_tracker_cancel(&tracker);
    assert(!motor_persistence_tracker_collect(&tracker, 0U, 0U, 0U, &receipt, &success));
    assert(!motor_persistence_tracker_collect(&tracker, 1U, 0U, 0U, &receipt, &success));
    assert(motor_persistence_tracker_collect(&tracker, 0U, 1U, 0U, &receipt, &success));
    assert(!success && !tracker.active);
    /* An old completion bit without any pending observation is also ambiguous. */
    assert(motor_persistence_tracker_arm(&tracker, &identity, 1U, 0U, 1U));
    assert(!motor_persistence_tracker_collect(&tracker, 0U, 1U, 0U, &receipt, &success));
    motor_persistence_tracker_cancel(&tracker);
    assert(!motor_persistence_tracker_collect(&tracker, 0U, 1U, 1U, &receipt, &success));
    assert(!motor_persistence_tracker_collect(&tracker, 1U, 1U, 1U, &receipt, &success));
    assert(motor_persistence_tracker_collect(&tracker, 0U, 1U, 1U, &receipt, &success));
    assert(!success);
}
#ifdef MOTOR_PERSISTENCE_TRANSACTION_VERSION
static MotorPersistenceTransaction transaction;
static MotorPersistenceSnapshot synthetic;
static uint64_t clock_us;
static unsigned save_count, write_count;
/** @brief Initializes an explicitly synthetic bus and fixed transaction binding. */
static void start_transaction(uint8_t already_target)
{
    baseline(&synthetic);
    if (already_target) { synthetic.items[21].raw = float_bits(512.0F); synthetic.items[10].raw = 2U; }
    clock_us = 1000U; save_count = write_count = 0U;
    motor_persistence_init(&transaction);
    assert(motor_persistence_begin(&transaction, 1U, 2U, 3U, clock_us));
}
/** @brief Supplies healthy disabled data without claiming real feedback evidence. */
static MotorPersistenceHealth healthy(void)
{
    MotorPersistenceHealth health = {0};
    health.boot_epoch = 1U; health.observed_us = clock_us;
    health.disabled = health.valid = health.idle = 1U;
    health.motor_temperature = health.driver_temperature = 30.0F;
    return health;
}
/** @brief Admits synthetic prepare with explicit newly observed disabled health. */
static uint8_t prepare_current(void)
{
    MotorPersistenceHealth health = healthy();
    return motor_persistence_prepare(&transaction, &health, clock_us);
}
/** @brief Emulates one bus exchange; all parameter data and TX receipts are synthetic. */
static void exchange(uint8_t early_ack)
{
    CanFrame command, reply;
    MotorPersistenceIdentity identity;
    unsigned index;
    uint32_t raw = 0U;
    if (transaction.pending && !transaction.submitted) {
        command = transaction.frame; identity = transaction.identity;
    } else { assert(motor_persistence_next(&transaction, clock_us, &command, &identity)); }
    assert(command.identifier == 0x7ffU && command.data[0] == 7U && command.data[1] == 0U);
    assert(motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
    assert(!motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
    ++clock_us;
    reply = command; reply.identifier = 23U;
    if (command.data[2] == 0xaaU) {
        static const uint8_t expected_save[4] = {7U, 0U, 0xaaU, 1U};
        assert(command.length == sizeof(expected_save));
        assert(memcmp(command.data, expected_save, sizeof(expected_save)) == 0);
        ++save_count;
    } else {
        for (index = 0U; index < 45U && motor_persistence_rid(index) != command.data[3]; ++index) {}
        assert(index < 45U);
        if (command.data[2] == 0x55U) {
            assert(command.length == 8U && (index == 21U || index == 10U));
            ++write_count;
            raw = (uint32_t)command.data[4] | (uint32_t)command.data[5] << 8 |
                (uint32_t)command.data[6] << 16 | (uint32_t)command.data[7] << 24;
            synthetic.items[index].raw = raw;
        } else { assert(command.data[2] == 0x33U && command.length == 4U); }
        raw = synthetic.items[index].raw;
        reply.length = 8U; reply.data[2] = command.data[2];
        for (index = 0U; index < 4U; ++index) { reply.data[4U + index] = (uint8_t)(raw >> (index * 8U)); }
    }
    ++clock_us;
    if (early_ack) {
        assert(motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
        assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
        assert(transaction.pending);
    }
    ++clock_us;
    assert(motor_persistence_report_tx(&transaction, &identity, clock_us, clock_us, 1U));
    if (!early_ack) { assert(motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us)); }
    assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    clock_us += 10U;
}
/** @brief Reads exactly the fixed whitelist, checking dense ordered RID requests. */
static void read_all(void)
{
    unsigned index;
    for (index = 0U; index < 45U; ++index) {
        CanFrame frame; MotorPersistenceIdentity identity;
        assert(motor_persistence_next(&transaction, clock_us, &frame, &identity));
        assert(frame.data[2] == 0x33U && frame.data[3] == motor_persistence_rid(index));
        exchange((uint8_t)(index & 1U));
    }
}
/** @brief Replays the observed 0x55 PMAX ACK and both fixed writes before/after actual TX; never substitutes it for readback. */
static void write_ack_opcode_tests(void)
{
    unsigned early_ack, write_step, invalid_case;
    for (early_ack = 0U; early_ack < 2U; ++early_ack) {
        start_transaction(0U); read_all(); assert(prepare_current());
        for (write_step = 0U; write_step < 2U; ++write_step) {
            CanFrame command, reply;
            MotorPersistenceIdentity identity;
            assert(motor_persistence_next(&transaction, clock_us, &command, &identity));
            assert(command.data[2] == 0x55U && command.data[3] == (write_step ? 10U : 21U));
            assert(motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
            assert(write_step ? !transaction.mode_known : !transaction.pmax_known);
            reply = command; reply.identifier = 23U;
            ++clock_us;
            if (early_ack) {
                assert(motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
                assert(transaction.pending && !transaction.actual_tx);
                assert(write_step ? !transaction.mode_known : !transaction.pmax_known);
            }
            ++clock_us;
            assert(motor_persistence_report_tx(&transaction, &identity, clock_us, clock_us, 1U));
            if (!early_ack) {
                assert(transaction.pending && transaction.actual_tx);
                assert(motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
            }
            assert(!transaction.pending && transaction.pmax_known && transaction.mode_known);
            assert(transaction.write_step == write_step + 1U && !transaction.save_used);
            assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
            synthetic.items[write_step ? 10U : 21U].raw = write_step ? 2U : float_bits(512.0F);
        }
        assert(transaction.state == MOTOR_PERSISTENCE_PREPARE_VERIFY);
        read_all(); assert(transaction.state == MOTOR_PERSISTENCE_PREPARED && !transaction.save_used);
    }
    /* A read-shaped reply cannot confirm a write even if all returned value bytes match. */
    for (invalid_case = 0U; invalid_case < 11U; ++invalid_case) {
        CanFrame command, reply;
        MotorPersistenceIdentity identity;
        uint64_t lower_us;
        uint8_t standard_data = 1U;
        start_transaction(0U); read_all(); assert(prepare_current());
        assert(motor_persistence_next(&transaction, clock_us, &command, &identity));
        assert(motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
        ++clock_us;
        assert(motor_persistence_report_tx(&transaction, &identity, clock_us, clock_us, 1U));
        reply = command; reply.identifier = 23U; lower_us = clock_us;
        switch (invalid_case) {
        case 0U: reply.data[2] = 0x33U; break;
        case 1U: reply.data[2] = 0xaaU; break;
        case 2U: reply.data[3] = 10U; break;
        case 3U: reply.identifier = 24U; break;
        case 4U: reply.data[0] = 8U; break;
        case 5U: reply.data[1] = 1U; break;
        case 6U: reply.length = 7U; break;
        case 7U: reply.length = 9U; break;
        case 8U: standard_data = 0U; break;
        case 9U: lower_us = transaction.boundary_us - 1U; break;
        default: lower_us = clock_us + 1U; break;
        }
        assert(!motor_persistence_accept(&transaction, &reply, standard_data, lower_us, clock_us));
        assert(transaction.pending && !transaction.deferred && !transaction.pmax_known && !transaction.save_used);
        reply = command; reply.identifier = 23U;
        assert(motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
        assert(transaction.pmax_known && !transaction.pending && !transaction.save_used);
    }
    /* An exact opcode with the wrong returned value still faults before restoring mapping. */
    start_transaction(0U); read_all(); assert(prepare_current());
    {
        CanFrame command, reply;
        MotorPersistenceIdentity identity;
        assert(motor_persistence_next(&transaction, clock_us, &command, &identity));
        assert(motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
        ++clock_us;
        assert(motor_persistence_report_tx(&transaction, &identity, clock_us, clock_us, 1U));
        reply = command; reply.identifier = 23U; reply.data[4] ^= 1U;
        assert(motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
        assert(transaction.state == MOTOR_PERSISTENCE_ABORTED && transaction.error == MOTOR_PERSISTENCE_ERROR_SNAPSHOT);
        assert(!transaction.pmax_known && !transaction.save_used);
    }
}
/** @brief Drives fixed volatile writes then verifies every register before Save admission. */
static void prepare_transaction(void)
{
    read_all(); assert(transaction.state == MOTOR_PERSISTENCE_BASELINE_READY);
    assert(prepare_current());
    while (transaction.state == MOTOR_PERSISTENCE_PREPARE_WRITE) { exchange(1U); }
    read_all(); assert(transaction.state == MOTOR_PERSISTENCE_PREPARED);
    assert(transaction.pmax_known && transaction.mode_known);
    assert(transaction.confirmed_pmax_raw == float_bits(512.0F) && transaction.confirmed_mode == 2U);
}
/** @brief Exercises complete synthetic save and target-already-present skip paths. */
static void happy_tests(void)
{
    MotorPersistenceHealth health;
    unsigned target_case;
    for (target_case = 0U; target_case < 3U; ++target_case) {
        start_transaction(0U);
        if (target_case == 1U) { synthetic.items[21].raw = float_bits(512.0F); }
        if (target_case == 2U) { synthetic.items[10].raw = 2U; }
        prepare_transaction();
        assert(write_count == (target_case ? 1U : 2U));
        health = healthy(); assert(motor_persistence_request_save(&transaction, &health, clock_us));
        read_all(); assert(transaction.state == MOTOR_PERSISTENCE_SAVE_HEALTH);
        health = healthy(); assert(motor_persistence_authorize_save(&transaction, &health, clock_us));
        exchange(1U);
        assert(transaction.state == MOTOR_PERSISTENCE_SETTLE && transaction.save_used && transaction.save_uncertain);
        assert(!motor_persistence_request_save(&transaction, &health, clock_us));
        clock_us = transaction.settle_until_us - 1U;
        motor_persistence_tick(&transaction, clock_us, 1U, 3U, 0U);
        assert(transaction.state == MOTOR_PERSISTENCE_SETTLE);
        ++clock_us; motor_persistence_tick(&transaction, clock_us, 1U, 3U, 0U);
        read_all(); assert(transaction.state == MOTOR_PERSISTENCE_COMPLETE);
        assert(save_count == 1U && !transaction.save_uncertain);
        assert(!motor_persistence_begin(&transaction, 1U, 99U, 3U, clock_us));
    }
    start_transaction(1U); read_all();
    assert(transaction.state == MOTOR_PERSISTENCE_COMPLETE && transaction.skip_save && !transaction.save_used);
    assert(!prepare_current());
}
/** @brief Invalidates every configuration word and fixed identity without silent preparation. */
static void rejection_tests(void)
{
    unsigned index;
    for (index = 0U; index < 45U; ++index) {
        if (!motor_persistence_is_configuration(index)) { continue; }
        start_transaction(0U); read_all();
        assert(prepare_current());
        while (transaction.state == MOTOR_PERSISTENCE_PREPARE_WRITE) { exchange(0U); }
        synthetic.items[index].raw ^= 1U;
        read_all(); assert(transaction.state == MOTOR_PERSISTENCE_ABORTED);
    }
    { const unsigned bad_indices[] = {7U, 8U, 14U, 36U, 10U, 21U, 22U, 23U, 29U};
      for (index = 0U; index < sizeof(bad_indices)/sizeof(bad_indices[0]); ++index) {
        start_transaction(0U); synthetic.items[bad_indices[index]].raw = 0x7fc00000U;
        read_all(); assert(transaction.state == MOTOR_PERSISTENCE_ABORTED);
      }
    }
}
/** @brief Rejects stale receipts, wrong wire frames, missing RX boundary and missing actual TX. */
static void transport_tests(void)
{
    CanFrame command, reply; MotorPersistenceIdentity identity, wrong;
    uint64_t deadline;
    start_transaction(0U);
    assert(motor_persistence_next(&transaction, clock_us, &command, &identity));
    deadline = transaction.frame_deadline_us;
    wrong = identity; ++wrong.sequence;
    assert(!motor_persistence_submitted(&transaction, &wrong, clock_us, clock_us, 1U));
    assert(!motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 0U));
    assert(!motor_persistence_submitted(&transaction, &identity, clock_us, clock_us + 1U, 1U));
    reply = command; reply.identifier = 23U; reply.length = 8U;
    assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    assert(motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
    assert(!motor_persistence_report_tx(&transaction, &wrong, clock_us, clock_us, 1U));
    reply.identifier = 24U; assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    reply.identifier = 23U; assert(!motor_persistence_accept(&transaction, &reply, 0U, clock_us, clock_us));
    reply.length = 4U; assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    reply.length = 8U; reply.data[3] = 1U;
    assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    reply.data[3] = 0U;
    assert(motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    assert(transaction.state == MOTOR_PERSISTENCE_BASELINE && transaction.read_index == 0U);
    clock_us = deadline; motor_persistence_tick(&transaction, clock_us, 1U, 3U, 0U);
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED && transaction.external_disable_required);
    start_transaction(0U); motor_persistence_tick(&transaction, clock_us, 2U, 3U, 0U);
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED);
    start_transaction(0U); motor_persistence_tick(&transaction, clock_us, 1U, 4U, 0U);
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED);
    start_transaction(0U); motor_persistence_tick(&transaction, clock_us, 1U, 3U, 1U);
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED);
    start_transaction(0U); clock_us = transaction.phase_deadline_us;
    motor_persistence_tick(&transaction, clock_us, 1U, 3U, 0U);
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED);
}
/** @brief Requires fresh disabled bounded health and a second complete raw verification. */
static void health_tests(void)
{
    MotorPersistenceHealth health;
    unsigned invalid_case;
    for (invalid_case = 0U; invalid_case < 11U; ++invalid_case) {
        start_transaction(0U); prepare_transaction(); health = healthy();
        switch (invalid_case) {
        case 0: health.disabled = 0U; break;
        case 1: health.fault = 1U; break;
        case 2: health.velocity = 0.051F; break;
        case 3: health.torque = -0.151F; break;
        case 4: health.motor_temperature = 50.0F; break;
        case 5: health.driver_temperature = 50.0F; break;
        case 6: health.boot_epoch = 2U; break;
        case 7: health.idle = 0U; break;
        case 8: clock_us += 100001U; break;
        case 9: health.observed_us++; break;
        default: { uint32_t nan_bits = 0x7fc00000U; memcpy(&health.velocity, &nan_bits, sizeof(nan_bits)); break; }
        }
        assert(!motor_persistence_request_save(&transaction, &health, clock_us));
        assert(!transaction.save_used);
    }
    start_transaction(0U); prepare_transaction(); health = healthy();
    assert(motor_persistence_request_save(&transaction, &health, clock_us));
    synthetic.items[29].raw ^= 1U; read_all();
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED && !transaction.save_used);
}
/** @brief Advances synthetic transaction to the final fresh-health admission window. */
static void save_ready(void)
{
    MotorPersistenceHealth health;
    start_transaction(0U); prepare_transaction(); health = healthy();
    assert(motor_persistence_request_save(&transaction, &health, clock_us));
    read_all(); health = healthy();
    assert(motor_persistence_authorize_save(&transaction, &health, clock_us));
}
/** @brief Exercises stale Save ACKs, queue-only uncertainty and irreversible failure latches. */
static void irreversible_tests(void)
{
    CanFrame frame, reply; MotorPersistenceIdentity identity;
    MotorPersistenceHealth health;
    uint64_t original_deadline;
    unsigned scenario;
    for (scenario = 0U; scenario < 4U; ++scenario) {
        save_ready();
        assert(motor_persistence_next(&transaction, clock_us, &frame, &identity));
        original_deadline = transaction.frame_deadline_us;
        reply = frame; reply.identifier = 23U; reply.data[3] = 1U;
        assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
        ++clock_us;
        assert(!motor_persistence_next(&transaction, clock_us, &frame, &identity));
        assert(transaction.frame_deadline_us == original_deadline);
        assert(motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
        assert(transaction.save_used && transaction.save_uncertain);
        assert(!motor_persistence_next(&transaction, clock_us, &frame, &identity));
        if (scenario == 0U) {
            clock_us = original_deadline; motor_persistence_tick(&transaction, clock_us, 1U, 3U, 0U);
        } else if (scenario == 1U) {
            motor_persistence_tick(&transaction, clock_us, 1U, 3U, 1U);
        } else if (scenario == 2U) {
            motor_persistence_tick(&transaction, clock_us, 2U, 3U, 0U);
        } else {
            assert(!motor_persistence_report_tx(&transaction, &identity, clock_us, clock_us, 0U));
        }
        assert(transaction.state == MOTOR_PERSISTENCE_ABORTED && transaction.save_used && transaction.save_uncertain);
        health = healthy(); assert(!motor_persistence_request_save(&transaction, &health, clock_us));
        assert(!motor_persistence_begin(&transaction, 1U, 4U, 3U, clock_us));
    }
    save_ready(); exchange(0U);
    clock_us = transaction.settle_until_us; motor_persistence_tick(&transaction, clock_us, 1U, 3U, 0U);
    synthetic.items[29].raw ^= 1U; read_all();
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED && transaction.save_used && transaction.save_uncertain);
    start_transaction(0U); prepare_transaction(); health = healthy();
    assert(motor_persistence_request_save(&transaction, &health, clock_us));
    read_all(); assert(!motor_persistence_authorize_save(&transaction, &health, clock_us));
    health = healthy(); health.observed_us = transaction.verified_us;
    assert(!motor_persistence_authorize_save(&transaction, &health, clock_us));
    health = healthy(); assert(motor_persistence_authorize_save(&transaction, &health, clock_us));
    clock_us += 100001U;
    assert(motor_persistence_next(&transaction, clock_us, &frame, &identity));
    assert(!motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED && !transaction.save_used);
}
/** @brief Rejects pre-boundary stale frames but accepts RX preceding later TXBTO observation. */
static void deferred_old_tests(void)
{
    CanFrame frame, reply; MotorPersistenceIdentity identity, wrong;
    unsigned field;
    start_transaction(0U);
    assert(motor_persistence_next(&transaction, clock_us, &frame, &identity));
    assert(motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
    reply = frame; reply.identifier = 23U; reply.length = 8U;
    assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us - 1U, clock_us));
    assert(motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    ++clock_us;
    for (field = 0U; field < 5U; ++field) {
        wrong = identity;
        switch (field) { case 0: wrong.boot_epoch++; break; case 1: wrong.request_id++; break;
            case 2: wrong.owner_generation++; break; case 3: wrong.phase++; break; default: wrong.sequence++; break; }
        assert(!motor_persistence_report_tx(&transaction, &wrong, clock_us, clock_us, 1U));
    }
    assert(motor_persistence_report_tx(&transaction, &identity, clock_us, clock_us, 1U));
    assert(transaction.read_index == 1U && !transaction.pending);
    assert(transaction.current.items[0].received_us < transaction.current.items[0].send_completed_observed_us);
    assert(motor_persistence_next(&transaction, clock_us, &frame, &identity));
    assert(motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
    assert(motor_persistence_report_tx(&transaction, &identity, clock_us, clock_us, 1U));
    assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    reply.data[3] = 1U;
    reply.data[2] = 0x55U;
    assert(!motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    reply.data[2] = 0x33U;
    assert(motor_persistence_accept(&transaction, &reply, 1U, clock_us, clock_us));
    assert(transaction.read_index == 2U);
}
/** @brief Fails closed if a previously admitted prepare becomes unhealthy through age. */
static void stale_prepare_tests(void)
{
    CanFrame frame; MotorPersistenceIdentity identity;
    MotorPersistenceHealth health;
    start_transaction(0U); read_all();
    assert(prepare_current());
    clock_us += 100001U;
    assert(motor_persistence_next(&transaction, clock_us, &frame, &identity));
    assert(!motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED);
    start_transaction(0U); read_all();
    health = healthy(); health.disabled = 0U;
    assert(!motor_persistence_prepare(&transaction, &health, clock_us));
    assert(prepare_current());
    clock_us += 100001U; health = healthy();
    assert(motor_persistence_refresh_health(&transaction, &health, clock_us));
    assert(motor_persistence_next(&transaction, clock_us, &frame, &identity));
    assert(motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
    assert(!transaction.pmax_known && transaction.mode_known);
    clock_us = transaction.frame_deadline_us;
    motor_persistence_tick(&transaction, clock_us, 1U, 3U, 0U);
    assert(transaction.state == MOTOR_PERSISTENCE_ABORTED && !transaction.pmax_known);
    assert(!motor_persistence_refresh_health(&transaction, &health, clock_us));
}
/** @brief Invalid refreshed health revokes prepare even if an older healthy sample was admitted. */
static void bad_health_refresh_tests(void)
{
    unsigned frame_prepared, invalid_case;
    for (frame_prepared = 0U; frame_prepared < 2U; ++frame_prepared) {
        for (invalid_case = 0U; invalid_case < 15U; ++invalid_case) {
            CanFrame frame;
            MotorPersistenceIdentity identity = {0};
            MotorPersistenceHealth health;
            uint32_t nan_bits = 0x7fc00000U;
            start_transaction(0U); read_all(); assert(prepare_current());
            if (frame_prepared) {
                assert(motor_persistence_next(&transaction, clock_us, &frame, &identity));
            }
            health = healthy();
            switch (invalid_case) {
            case 0U: health.disabled = 0U; break;
            case 1U: health.fault = 1U; break;
            case 2U: health.velocity = 0.051F; break;
            case 3U: health.torque = -0.151F; break;
            case 4U: health.motor_temperature = 50.0F; break;
            case 5U: health.driver_temperature = 50.0F; break;
            case 6U: health.valid = 0U; break;
            case 7U: health.idle = 0U; break;
            case 8U: health.boot_epoch++; break;
            case 9U: health.observed_us++; break;
            case 10U: clock_us += 100001U; break;
            case 11U: memcpy(&health.velocity, &nan_bits, sizeof(nan_bits)); break;
            case 12U: memcpy(&health.torque, &nan_bits, sizeof(nan_bits)); break;
            case 13U: memcpy(&health.driver_temperature, &nan_bits, sizeof(nan_bits)); break;
            default: break;
            }
            assert(!motor_persistence_refresh_health(&transaction,
                invalid_case == 14U ? NULL : &health, clock_us));
            assert(transaction.state == MOTOR_PERSISTENCE_ABORTED);
            assert(transaction.external_disable_required && !transaction.save_used);
            assert(!motor_persistence_next(&transaction, clock_us, &frame, &identity));
            assert(!motor_persistence_submitted(&transaction, &identity, clock_us, clock_us, 1U));
            health = healthy();
            assert(!motor_persistence_refresh_health(&transaction, &health, clock_us));
        }
    }
    /* Both fixed writes still allow valid feedback refresh without changing stage budget. */
    start_transaction(0U); read_all(); assert(prepare_current());
    while (transaction.state == MOTOR_PERSISTENCE_PREPARE_WRITE) {
        MotorPersistenceHealth health;
        uint64_t phase_deadline = transaction.phase_deadline_us;
        clock_us += 50000U; health = healthy();
        assert(motor_persistence_refresh_health(&transaction, &health, clock_us));
        assert(transaction.phase_deadline_us == phase_deadline);
        exchange(1U);
    }
    read_all(); assert(transaction.state == MOTOR_PERSISTENCE_PREPARED && write_count == 2U);
}
#endif
/** @brief Executes standalone whitelist, snapshot and cancellation tests. */
int main(void)
{
    unsigned index, writable_count = 0U;
    for (index = 0U; index < 45U; ++index) {
        unsigned expected = index < 37U ? index : index < 43U ? index + 13U : index + 37U;
        unsigned rid = motor_persistence_rid(index);
        unsigned expected_unsigned = rid == 7U || rid == 8U || rid == 9U || rid == 10U ||
            rid == 13U || rid == 14U || rid == 15U || rid == 16U || rid == 35U || rid == 36U;
        assert(rid == expected);
        assert(motor_persistence_is_unsigned(index) == expected_unsigned);
        writable_count += motor_persistence_is_configuration(index);
    }
    assert(writable_count == 26U);
    assert(motor_persistence_rid(45U) == 255U);
    assert(!motor_persistence_is_configuration(45U));
    assert(!motor_persistence_is_unsigned(45U));
    snapshot_tests();
    tracker_tests();
#ifdef MOTOR_PERSISTENCE_TRANSACTION_VERSION
    write_ack_opcode_tests();
    bad_health_refresh_tests();
    happy_tests(); rejection_tests(); transport_tests(); health_tests();
    irreversible_tests(); deferred_old_tests(); stale_prepare_tests();
    puts("SYNTHETIC ONLY: full persistence transaction and adversarial tests PASS; no hardware evidence");
#else
    assert(!"Missing fixed ID7 persistence transaction state machine");
#endif
    puts("fixed persistence register surface and snapshot PASS");
    return 0;
}
