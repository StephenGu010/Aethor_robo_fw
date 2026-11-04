/** @file persistence_core_clock_test.c
 * @brief Executes verbatim Core timing boundaries against the production App and persistence module.
 * Synthetic RTOS/Platform observations are deterministic and do not qualify a board, CAN bus or HAL.
 */
#define main existing_integrated_fixture_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Controls only the external clock, scheduler and immutable Platform receipt ports. */
typedef struct
{
    DebugUiStaticTxIdentity receipt_identity, rx_identity;
    uint64_t now_us, next_us, receipt_us, rx_boundary_us;
    uint16_t receipt_sof, rx_sof;
    unsigned critical_depth, notification_count, preemptions, receipt_count;
    uint8_t receipt_ready, ordered_rx, sof_valid, preempt_on_sample, pending_preemption;
} CoreClockFixture;
static CoreClockFixture clock_fixture;
static void *ProtocolTaskHandle;

static void RunExtractedControlBoundary(void);

/** @brief Simulates the higher-priority Control task only when the existing task boundary permits it. */
static void PreemptWithControl(void)
{
    assert(clock_fixture.critical_depth == 0U);
    clock_fixture.pending_preemption = 0U;
    clock_fixture.now_us = clock_fixture.next_us;
    ++clock_fixture.preemptions;
    RunExtractedControlBoundary();
}

/** @brief Tracks nested Core/App task critical sections without imposing a lock outside production code. */
static void EnterClockCritical(void) { ++clock_fixture.critical_depth; }

/** @brief Runs a pending higher-priority task at the exact outer critical exit. */
static void ExitClockCritical(void)
{
    assert(clock_fixture.critical_depth != 0U);
    --clock_fixture.critical_depth;
    if (!clock_fixture.critical_depth && clock_fixture.pending_preemption) { PreemptWithControl(); }
}
#define taskENTER_CRITICAL() EnterClockCritical()
#define taskEXIT_CRITICAL() ExitClockCritical()
#define TaskHandle_t void *

/** @brief Models the real clock's inner critical section and a task switch immediately after sampling. */
static uint64_t AethorMonotonicTimestampUs(void)
{
    uint64_t sampled_us;
    EnterClockCritical();
    sampled_us = clock_fixture.now_us;
    ExitClockCritical();
    if (clock_fixture.preempt_on_sample) {
        clock_fixture.preempt_on_sample = 0U;
        if (clock_fixture.critical_depth) { clock_fixture.pending_preemption = 1U; }
        else { PreemptWithControl(); }
    }
    return sampled_us;
}

/** @brief Retains the exact CDC binding used to admit the synthetic production App owner. */
static uint32_t CDC_PersistenceLinkState_HS(void) { return 0x80000001UL; }

/** @brief No parameter debt is cleared by a fabricated completion in this strict-receipt test. */
static uint8_t stm32_platform_can_cancel_persistence(const MotorPersistenceIdentity *identity)
{ (void)identity; return 0U; }

/** @brief No parameter receipt is available; actual production App cancellation remains executable. */
static uint8_t stm32_platform_can_collect_persistence(MotorPersistenceIdentity *identity,
    uint64_t observed_us, uint64_t *completed_us, uint8_t *succeeded)
{ (void)identity; (void)observed_us; (void)completed_us; (void)succeeded; return 0U; }

/** @brief Supplies one exact submitted DISABLE receipt after advancing the coarse board clock by one millisecond. */
static uint8_t stm32_platform_can_collect_static(DebugUiStaticTxIdentity *identity,
    uint8_t *succeeded, uint16_t *sof)
{
    assert(clock_fixture.critical_depth != 0U);
    if (!clock_fixture.receipt_ready) { return 0U; }
    clock_fixture.receipt_ready = 0U;
    clock_fixture.now_us = clock_fixture.receipt_us;
    ++clock_fixture.receipt_count;
    *identity = clock_fixture.receipt_identity;
    *succeeded = 1U;
    *sof = clock_fixture.receipt_sof;
    return 1U;
}

/** @brief Returns original immutable strict identity/SOF metadata without substituting decode time for arrival. */
static uint8_t stm32_platform_can_static_rx_identity(const CanFrame *frame,
    DebugUiStaticTxIdentity *identity, uint16_t *sof, uint8_t *sof_valid)
{
    assert(frame && clock_fixture.critical_depth);
    *identity = clock_fixture.rx_identity;
    *sof = clock_fixture.rx_sof;
    *sof_valid = clock_fixture.sof_valid;
    return clock_fixture.ordered_rx;
}

/** @brief Supplies no unrelated capability receipt while retaining the real Core collector call. */
static uint8_t stm32_platform_can_collect_capability_receipt(CanFrame *frame, uint8_t *succeeded)
{ (void)frame; (void)succeeded; return 0U; }

/** @brief Retains the real Core's original pre-inspection FIFO lower-bound publication. */
static void stm32_platform_can_observe_rx_boundary(uint64_t observed_us)
{ clock_fixture.rx_boundary_us = observed_us; }

/** @brief Returns the exact previously published boundary without adding clock resolution. */
static uint64_t stm32_platform_can_capability_rx_boundary(void) { return clock_fixture.rx_boundary_us; }

/** @brief Counts the bounded notification produced by the actual service fragment. */
static unsigned xTaskNotifyGive(void *handle)
{ assert(handle == ProtocolTaskHandle); return ++clock_fixture.notification_count; }

/** @brief Protocol output is observed only after processing; this test never sends USB data. */
static void QueueProtocolOutputBatch(const ProtocolOutputBatch *output) { assert(output != NULL); }

#include "persistence_core_clock_extracted.inc"

/** @brief Starts an independent synthetic firmware lifetime using real facade discovery and disabled feedback. */
static AethorPersistenceTransfer SeedMaintenance(uint8_t prepare_receipt)
{
    ProtocolOutputBatch output;
    AethorPersistenceTransfer transfer;
    MotorDiscoveryResult *discovery;
    memset(&clock_fixture, 0, sizeof(clock_fixture));
    memset(&application_persistence, 0, sizeof(application_persistence));
    memset(&application_persistence_transaction, 0, sizeof(application_persistence_transaction));
    aethor_app_init(1000U, 1234U);
    aethor_app_set_task_critical_hooks(EnterClockCritical, ExitClockCritical);
    assert(request("0 hello", 2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    fixture_disabled_motor7(109000U);
    discovery = &application_motor_runtime.discovery.results[6];
    discovery->ranges.torque_max_nm = 10.0F;
    discovery->software_version = 808597046U;
    discovery->sub_version = 49U;
    aethor_app_integrated_set_can_idle(1U, 104000U);
    aethor_app_integrated_set_can_idle(1U, 110000U);
    aethor_app_persistence_observe_link(0x80000001UL, 110000U);
    assert(aethor_app_persistence_begin_readonly(91U, 110010U));
    memset(&transfer, 0, sizeof(transfer));
    if (prepare_receipt) {
        assert(aethor_app_persistence_next_transfer(110011U, &transfer));
        assert(transfer.kind == AETHOR_PERSISTENCE_TRANSFER_DISABLE);
        assert(aethor_app_persistence_disable_submitted(&transfer.disable_identity, 110012U));
        clock_fixture.receipt_identity = clock_fixture.rx_identity = transfer.disable_identity;
        clock_fixture.receipt_ready = clock_fixture.ordered_rx = clock_fixture.sof_valid = 1U;
    }
    clock_fixture.now_us = 111000U;
    clock_fixture.next_us = clock_fixture.receipt_us = 112000U;
    clock_fixture.receipt_sof = 100U;
    clock_fixture.rx_sof = 101U;
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_NONE);
    assert(clock_fixture.critical_depth == 0U);
    return transfer;
}

/** @brief Reports the real wrapper result before asserting so RED contains the exact classified failure. */
static void AssertHealthy(const char *case_name, uint64_t original_deadline_us)
{
    printf("CLOCK_RESULT case=%s fault=%u last_us=%llu phase_deadline_us=%llu original_deadline_us=%llu\n",
        case_name, (unsigned)application_persistence.fault,
        (unsigned long long)application_persistence.last_us,
        (unsigned long long)application_persistence_transaction.phase_deadline_us,
        (unsigned long long)original_deadline_us);
    fflush(stdout);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_NONE);
    assert(application_persistence_transaction.phase_deadline_us == original_deadline_us);
    assert(clock_fixture.critical_depth == 0U);
}

/** @brief A newer collected receipt must precede a freshly sampled service time in the actual Control fragment. */
static void TestControlReceiptClock(void)
{
    uint64_t deadline_us;
    (void)SeedMaintenance(1U);
    deadline_us = application_persistence_transaction.phase_deadline_us;
    RunExtractedControlBoundary();
    AssertHealthy("control", deadline_us);
    assert(clock_fixture.receipt_count == 1U);
    assert(application_persistence.disable_actual_tx && application_persistence.disable_observed_us == 112000U);
    assert(application_persistence.last_us == 112000U);
    assert(clock_fixture.rx_boundary_us == 111000U);
}

/** @brief RX decode samples after its collector while preserving original arrival, strict identity, SOF and deadlines. */
static void TestReceiveReceiptClock(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 5U; ++scenario) {
        AethorPersistenceTransfer transfer = SeedMaintenance(1U);
        uint64_t arrival_lower_us = 111000U;
        uint64_t deadline_us = application_persistence_transaction.phase_deadline_us;
        CanFrame feedback = {23U, 8U, {7U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 25U}};
        if (scenario == 1U) { ++clock_fixture.rx_identity.request_id; }
        if (scenario == 2U) { clock_fixture.rx_sof = clock_fixture.receipt_sof; }
        if (scenario == 3U) { arrival_lower_us = 110012U; }
        if (scenario == 4U) { clock_fixture.sof_valid = 0U; }
        EnterClockCritical();
        ReceiveSingleStaticFrame(&feedback, arrival_lower_us, 111000U);
        ExitClockCritical();
        AssertHealthy("rx", deadline_us);
        assert(application_persistence.last_us == 112000U && application_persistence.disable_sof == 100U);
        assert(debug_ui_static_tx_identity_equal(&application_persistence.disable_identity, &transfer.disable_identity));
        assert(application_persistence.disable_created_us == 110011U && application_persistence.disable_submitted_us == 110012U);
        assert(application_persistence.health_proven == (scenario == 0U));
        assert(application_persistence.health.observed_us == (scenario == 0U ? arrival_lower_us : 109000U));
    }
}

/** @brief A higher-priority Control task cannot advance maintenance between protocol timestamp sampling and processing. */
static void TestProtocolPreemptionClock(void)
{
    uint64_t deadline_us;
    (void)SeedMaintenance(0U);
    deadline_us = application_persistence_transaction.phase_deadline_us;
    clock_fixture.preempt_on_sample = 1U;
    RunExtractedProtocolLine();
    AssertHealthy("protocol", deadline_us);
    assert(clock_fixture.preemptions == 1U && application_persistence.last_us == 112000U);
}

/** @brief Public UI admission prevents queued ordinary requests during maintenance, so empty UI calls cannot feed stale time back. */
static void TestUiPathDoesNotAdvanceMaintenanceClock(void)
{
    DebugUiRequest local_request;
    uint64_t deadline_us;
    (void)SeedMaintenance(0U);
    memset(&local_request, 0, sizeof(local_request));
    local_request.operation = DEBUG_UI_OPERATION_ACQUIRE;
    local_request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    local_request.identity.epoch = application_debug_ui.epoch;
    local_request.identity.request_id = 10U;
    local_request.created_at_us = 110500U;
    assert(aethor_app_debug_ui_submit(&local_request) == DEBUG_UI_REASON_DISABLED);
    assert(!debug_ui_mailbox_busy(&application_debug_ui.mailbox, false));
    deadline_us = application_persistence_transaction.phase_deadline_us;
    clock_fixture.preempt_on_sample = 1U;
    RunExtractedUiHead();
    clock_fixture.now_us = 111000U;
    clock_fixture.preempt_on_sample = 1U;
    RunExtractedUiTail();
    AssertHealthy("ui_unreachable", deadline_us);
    assert(clock_fixture.preemptions == 2U && application_persistence.last_us == 112000U);
}

/** @brief A genuine older public service observation remains a sticky CLOCK abort; no clamping masks it. */
static void TestRealRollbackStillAborts(void)
{
    uint64_t deadline_us;
    (void)SeedMaintenance(0U);
    deadline_us = application_persistence_transaction.phase_deadline_us;
    (void)aethor_app_service(112000U);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_NONE);
    (void)aethor_app_service(111000U);
    printf("CLOCK_ROLLBACK fault=%u expected=%u\n", (unsigned)application_persistence.fault,
        (unsigned)MOTOR_PERSISTENCE_ERROR_CLOCK);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CLOCK);
    assert(application_persistence_transaction.phase_deadline_us == deadline_us);
    (void)aethor_app_service(113000U);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CLOCK);
}

/** @brief Runs independent behavior cases so every pre-fix stale-clock failure can be retained separately. */
int main(int argument_count, char **arguments)
{
    const char *selected = argument_count > 1 ? arguments[1] : "all";
#define RUN_CLOCK_CASE(name, function) do { if (!strcmp(selected, "all") || !strcmp(selected, name)) { function(); puts("PERSISTENCE_CORE_CLOCK_PASS " name); } } while (0)
    RUN_CLOCK_CASE("control", TestControlReceiptClock);
    RUN_CLOCK_CASE("rx", TestReceiveReceiptClock);
    RUN_CLOCK_CASE("protocol", TestProtocolPreemptionClock);
    RUN_CLOCK_CASE("ui", TestUiPathDoesNotAdvanceMaintenanceClock);
    RUN_CLOCK_CASE("rollback", TestRealRollbackStillAborts);
#undef RUN_CLOCK_CASE
    return 0;
}
