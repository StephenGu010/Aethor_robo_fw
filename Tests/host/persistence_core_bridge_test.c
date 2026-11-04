/** @file persistence_core_bridge_test.c
 * @brief Executes verbatim Core control/RX/receipt code against controllable App and Platform ports.
 * These deterministic port tests do not compile the production App, Platform HAL, USB stack or firmware image.
 */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "can_tx_scheduler.h"
#include "motor_runtime.h"
#include "motor_persistence_app_port.h"
#include "stm32_adrc_channel.h"

/** @brief Synthetic RTOS tick type; only the extracted task's bounded delay is modeled. */
typedef uint32_t TickType_t;
/** @brief Minimal DWT register shape used only by the unchanged Core timing metrics. */
typedef struct { uint32_t CYCCNT; } CoreTestDwt;
/** @brief All controllable boundary state; no fixture field is production transaction evidence. */
typedef struct {
    AethorPersistenceTransfer transfer;
    MotorPersistenceIdentity parameter_receipt, cancelled, received_parameter;
    DebugUiStaticTxIdentity strict_receipt, received_strict, rx_identity;
    uint64_t completed_us, received_completed_us, received_now_us, rx_lower_us, rx_observed_us;
    uint32_t generation, link_word, observed_link_word, bound_link_word, link_change_mask;
    unsigned critical_depth, irq_depth, capture_count, allow_count, observe_count, final_observe_count;
    unsigned parameter_calls, parameter_commits, disable_calls, disable_commits, rejected_calls;
    unsigned parameter_receipts, cancel_calls, strict_receipts, legacy_receipts, rx_calls, legacy_rx_calls;
    unsigned generic_calls, group_calls, adrc_calls, probe_pops, adrc_pops, ordinary_pops, group_pops;
    unsigned queued_calls, capability_failures, emergency_calls, next_calls;
    uint8_t transfer_ready, parameter_receipt_ready, strict_receipt_ready, cancel_ready;
    uint8_t locked, parameter_accepts, strict_accepts, owner_valid, link_changes_at_final;
    uint8_t ordinary_ready, group_ready, emergency_ready, probe_ready, adrc_ready;
    uint8_t mutate_after_pop, expected_critical_pop, rx_consumed, ordered_rx, receipt_owned, ordinary_emergency;
    uint8_t last_succeeded, rx_sof_valid;
    uint16_t receipt_sof, rx_sof;
} CorePortFixture;
static CorePortFixture fixture;
static CoreTestDwt dwt_registers;
static CoreTestDwt *DWT = &dwt_registers;
static const uint32_t SystemCoreClock = 400000000U;
static uint32_t controlExecutionMaxUs, controlPeriodMaxUs, controlPreviousCycle;
static uint8_t controlCycleValid;
static void *ProtocolTaskHandle;
static jmp_buf task_return;
/** @brief Actual strict Platform defers this callback from reservation until service_tx immediately before HAL. */
static uint8_t (*deferred_strict_submitted)(const DebugUiStaticTxIdentity *, uint64_t);

/** @brief Track the real Core critical boundaries without manufacturing a lock in a stub. */
static void EnterTaskCritical(void) { ++fixture.critical_depth; }
/** @brief Reject unpaired critical exits. */
static void ExitTaskCritical(void) { assert(fixture.critical_depth); --fixture.critical_depth; }
#define taskENTER_CRITICAL() EnterTaskCritical()
#define taskEXIT_CRITICAL() ExitTaskCritical()
#define pdMS_TO_TICKS(value) (value)

/** @brief Compare immutable parameter identity fields without depending on struct padding. */
static uint8_t ParameterEqual(const MotorPersistenceIdentity *left, const MotorPersistenceIdentity *right)
{
    return (uint8_t)(left->boot_epoch == right->boot_epoch && left->request_id == right->request_id &&
        left->owner_generation == right->owner_generation && left->sequence == right->sequence && left->phase == right->phase);
}
/** @brief Supply a fixed monotonic observation time distinct from completion receipt time. */
static uint64_t AethorMonotonicTimestampUs(void) { return 8000ULL; }
/** @brief Supply the complete changing callback word, including its configured bit. */
uint32_t CDC_PersistenceLinkState_HS(void) { return fixture.link_word; }
/** @brief Record current admission before a potentially state-changing pop. */
uint32_t aethor_app_persistence_admission_generation(void)
{ assert(fixture.critical_depth); ++fixture.capture_count; return fixture.generation; }
/** @brief Model the App's final generation and maintenance exclusion decision. */
uint8_t aethor_app_persistence_allow_ordinary(uint32_t generation)
{
    assert(fixture.critical_depth); ++fixture.allow_count;
    return (uint8_t)(generation && generation == fixture.generation && !fixture.locked);
}
/** @brief Expose active/faulted maintenance ownership without altering it. */
uint8_t aethor_app_persistence_locked(void) { assert(fixture.critical_depth); return fixture.locked; }
/** @brief Invalidate the bound owner on any whole-word change, even configured-to-configured reconnect. */
void aethor_app_persistence_observe_link(uint32_t link_state, uint64_t now_us)
{
    assert(fixture.critical_depth && now_us); ++fixture.observe_count;
    if (fixture.irq_depth) ++fixture.final_observe_count;
    fixture.observed_link_word = link_state;
    if (link_state != fixture.bound_link_word) fixture.owner_valid = 0U;
}
/** @brief Yield one exact caller-owned transfer; rejected pairs cannot reappear. */
uint8_t aethor_app_persistence_next_transfer(uint64_t now_us, AethorPersistenceTransfer *transfer)
{
    assert(fixture.critical_depth && now_us); ++fixture.next_calls;
    if (!fixture.transfer_ready) return 0U;
    *transfer = fixture.transfer; fixture.transfer_ready = 0U; return 1U;
}
/** @brief Record rejection of the original immutable pair, never an ordinary queue retry. */
void aethor_app_persistence_transfer_rejected(const AethorPersistenceTransfer *transfer, uint64_t now_us)
{
    assert(fixture.critical_depth && now_us && transfer->kind == fixture.transfer.kind);
    assert(memcmp(&transfer->frame, &fixture.transfer.frame, sizeof(CanFrame)) == 0);
    ++fixture.rejected_calls;
}
/** @brief Return the original cancellation identity without synthesizing it from bytes. */
uint8_t aethor_app_persistence_cancel_identity(MotorPersistenceIdentity *identity)
{ assert(fixture.critical_depth); if (!fixture.cancel_ready) return 0U; *identity = fixture.cancelled; return 1U; }
/** @brief Check the final link observation occurred while the Platform interrupt mask was held. */
uint8_t aethor_app_persistence_parameter_submitted(const MotorPersistenceIdentity *identity, uint64_t boundary_us)
{
    assert(fixture.critical_depth && fixture.irq_depth && boundary_us == 8001ULL);
    assert(ParameterEqual(identity, &fixture.transfer.parameter_identity));
    assert(fixture.observed_link_word == fixture.link_word && fixture.final_observe_count);
    if (!fixture.owner_valid) return 0U;
    ++fixture.parameter_commits; return 1U;
}
/** @brief Check DISABLE retains its dedicated callback and the original strict identity. */
uint8_t aethor_app_persistence_disable_submitted(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us)
{
    assert(fixture.critical_depth && fixture.irq_depth && boundary_us == 8001ULL);
    assert(debug_ui_static_tx_identity_equal(identity, &fixture.transfer.disable_identity));
    assert(fixture.observed_link_word == fixture.link_word && fixture.final_observe_count);
    if (!fixture.owner_valid) return 0U;
    ++fixture.disable_commits; return 1U;
}
/** @brief Retain every receipt field and both observation times exactly as delivered. */
void aethor_app_persistence_parameter_transmitted(const MotorPersistenceIdentity *identity,
    uint8_t succeeded, uint64_t completed_observed_us, uint64_t now_us)
{
    assert(fixture.critical_depth); ++fixture.parameter_receipts;
    fixture.received_parameter = *identity; fixture.last_succeeded = succeeded;
    fixture.received_completed_us = completed_observed_us; fixture.received_now_us = now_us;
}
/** @brief Claim only this fixture's owned maintenance receipt; legacy strict receipts must continue. */
uint8_t aethor_app_persistence_disable_transmitted(const DebugUiStaticTxIdentity *identity,
    uint8_t succeeded, uint16_t sof, uint64_t observed_us)
{
    assert(fixture.critical_depth && observed_us == 8000ULL && sof == fixture.receipt_sof);
    assert(debug_ui_static_tx_identity_equal(identity, &fixture.strict_receipt));
    fixture.received_strict = *identity; fixture.last_succeeded = succeeded;
    ++fixture.strict_receipts; return fixture.receipt_owned;
}
/** @brief Preserve RX time bounds and strict SOF ordering before a possible legacy decode. */
uint8_t aethor_app_persistence_receive(const CanFrame *frame, uint64_t lower_us,
    uint64_t observed_us, const DebugUiStaticTxIdentity *identity, uint16_t sof, uint8_t sof_valid)
{
    assert(fixture.critical_depth && frame && sof == fixture.rx_sof && sof_valid == fixture.rx_sof_valid);
    assert((identity != NULL) == fixture.ordered_rx);
    if (identity) assert(debug_ui_static_tx_identity_equal(identity, &fixture.rx_identity));
    ++fixture.rx_calls; fixture.rx_lower_us = lower_us; fixture.rx_observed_us = observed_us;
    return fixture.rx_consumed;
}
/** @brief Model direct parameter reservation and its final IRQ-masked permission hook only. */
uint8_t stm32_platform_can_submit_persistence(const CanFrame *frame, const MotorPersistenceIdentity *identity,
    uint64_t now_us, uint64_t timeout_us, uint8_t (*submitted)(const MotorPersistenceIdentity *, uint64_t))
{
    assert(fixture.critical_depth && now_us && timeout_us == MOTOR_PERSISTENCE_FRAME_BUDGET_US);
    assert(ParameterEqual(identity, &fixture.transfer.parameter_identity));
    assert(memcmp(frame, &fixture.transfer.frame, sizeof(*frame)) == 0); ++fixture.parameter_calls;
    if (!fixture.parameter_accepts) return 0U;
    ++fixture.irq_depth;
    if (fixture.link_changes_at_final) fixture.link_word ^= fixture.link_change_mask;
    (void)submitted(identity, 8001ULL); --fixture.irq_depth; return 1U;
}
/** @brief Return a unique parameter receipt at a time different from the current task timestamp. */
uint8_t stm32_platform_can_collect_persistence(MotorPersistenceIdentity *identity, uint64_t now_us,
    uint64_t *completed_us, uint8_t *succeeded)
{
    assert(fixture.critical_depth && now_us == 8000ULL);
    if (!fixture.parameter_receipt_ready) return 0U;
    fixture.parameter_receipt_ready = 0U; *identity = fixture.parameter_receipt;
    *completed_us = fixture.completed_us; *succeeded = fixture.last_succeeded; return 1U;
}
/** @brief Verify cancellation uses the exact outstanding identity. */
uint8_t stm32_platform_can_cancel_persistence(const MotorPersistenceIdentity *identity)
{ assert(fixture.critical_depth && ParameterEqual(identity, &fixture.cancelled)); ++fixture.cancel_calls; return 1U; }
/** @brief Model only strict-slot reservation; the final hook is deferred until service_tx as in the real Platform. */
static uint8_t stm32_platform_can_submit_static(const CanFrame *frame, const DebugUiStaticTxIdentity *identity,
    uint64_t timeout_us, uint8_t (*submitted)(const DebugUiStaticTxIdentity *, uint64_t))
{
    assert(fixture.critical_depth && frame && identity && timeout_us); ++fixture.disable_calls;
    if (!fixture.strict_accepts) return 0U;
    assert(!deferred_strict_submitted); deferred_strict_submitted = submitted; return 1U;
}
/** @brief Expose one strict receipt, retaining its identity and hardware SOF. */
static uint8_t stm32_platform_can_collect_static(DebugUiStaticTxIdentity *identity, uint8_t *succeeded, uint16_t *sof)
{
    assert(fixture.critical_depth); if (!fixture.strict_receipt_ready) return 0U;
    fixture.strict_receipt_ready = 0U; *identity = fixture.strict_receipt;
    *succeeded = fixture.last_succeeded; *sof = fixture.receipt_sof; return 1U;
}
/** @brief Preserve the exact popped frame's ordering metadata. */
static uint8_t stm32_platform_can_static_rx_identity(const CanFrame *frame,
    DebugUiStaticTxIdentity *identity, uint16_t *sof, uint8_t *sof_valid)
{ assert(frame); *identity = fixture.rx_identity; *sof = fixture.rx_sof; *sof_valid = fixture.rx_sof_valid; return fixture.ordered_rx; }
/** @brief Record only unowned strict receipts in the legacy facade. */
static void aethor_app_static_transmitted(const DebugUiStaticTxIdentity *identity,
    uint8_t succeeded, uint16_t sof, uint64_t now_us)
{ assert(identity && now_us); (void)succeeded; (void)sof; ++fixture.legacy_receipts; }
/** @brief Count the legacy receive route after maintenance has declined to consume a frame. */
static uint8_t aethor_app_receive_static_frame(const CanFrame *frame, uint64_t lower_us,
    uint64_t observed_us, const DebugUiStaticTxIdentity *identity, uint16_t sof, uint8_t sof_valid)
{ assert(frame && lower_us && observed_us); (void)identity; (void)sof; (void)sof_valid; ++fixture.legacy_rx_calls; return 1U; }
/** @brief Retain the unchanged legacy strict callback signature. */
static uint8_t aethor_app_static_submitted(const DebugUiStaticTxIdentity *identity, uint64_t now_us)
{ assert(identity && now_us); return 1U; }
/** @brief Ordinary fixture frames are deliberately unrelated to static-byte classification. */
static uint8_t aethor_app_static_frame_identity(const CanFrame *frame, DebugUiStaticTxIdentity *identity)
{ (void)frame; (void)identity; return 0U; }
/** @brief Emergency fixture uses the original generic physical stop path. */
static uint8_t aethor_app_pos_load_terminal_frame_identity(const CanFrame *frame, DebugUiStaticTxIdentity *identity)
{ (void)frame; (void)identity; return 0U; }
/** @brief Supply the existing strict health deadline without altering it. */
static uint64_t aethor_app_debug_ui_preflight_timeout_us(void) { return 500000ULL; }
/** @brief Count notifications without treating queue acceptance as actual transmission. */
static void aethor_app_static_can_queued(const CanFrame *frame, uint8_t accepted, uint8_t strict, uint64_t now_us)
{ assert(frame && now_us); (void)accepted; (void)strict; ++fixture.queued_calls; }
/** @brief Execute generation races at the real next-frame pop boundary. */
static MotorRuntimeStatus aethor_app_next_can_frame(uint64_t now_us, CanFrame *frame, CanTxPriority *priority)
{
    assert(now_us); ++fixture.ordinary_pops;
    if (!fixture.ordinary_ready) return MOTOR_RUNTIME_STATUS_WAITING;
    if (fixture.expected_critical_pop) assert(fixture.critical_depth && fixture.capture_count);
    fixture.ordinary_ready = 0U; memset(frame, 0, sizeof(*frame)); frame->identifier = 0x107U;
    *priority = fixture.ordinary_emergency ? CAN_TX_PRIORITY_EMERGENCY : CAN_TX_PRIORITY_JOINT_CONTROL;
    if (fixture.mutate_after_pop) ++fixture.generation;
    return MOTOR_RUNTIME_STATUS_FRAME_READY;
}
/** @brief Execute the same admission race at the atomic control-group pop boundary. */
static uint8_t aethor_app_pop_control_group(CanFrame *frames)
{
    ++fixture.group_pops; if (!fixture.group_ready) return 0U;
    if (fixture.expected_critical_pop) assert(fixture.critical_depth && fixture.capture_count);
    fixture.group_ready = 0U; memset(frames, 0, sizeof(*frames) * ARM_JOINT_COUNT);
    if (fixture.mutate_after_pop) ++fixture.generation;
    return 1U;
}
/** @brief Yield one independent emergency stop, which maintenance cannot suppress. */
static uint8_t aethor_app_pop_emergency_can_frame(CanFrame *frame)
{
    if (!fixture.emergency_ready) return 0U;
    fixture.emergency_ready = 0U; memset(frame, 0, sizeof(*frame)); frame->identifier = 0x207U; return 1U;
}
/** @brief Count actual ordinary/emergency Platform admissions at the maintained call site. */
static CanTxSchedulerStatus stm32_platform_can_submit(CanTxPriority priority, const CanFrame *frame)
{
    assert(frame); ++fixture.generic_calls;
    if (priority == CAN_TX_PRIORITY_EMERGENCY) ++fixture.emergency_calls;
    else assert(fixture.critical_depth);
    return CAN_TX_SCHEDULER_STATUS_OK;
}
/** @brief Require group admission and final submission to share a task critical boundary. */
static CanTxSchedulerStatus stm32_platform_can_submit_control_group(const CanFrame *frames, uint8_t count)
{ assert(fixture.critical_depth && frames && count == ARM_JOINT_COUNT); ++fixture.group_calls; return CAN_TX_SCHEDULER_STATUS_OK; }
/** @brief Preserve the original failure callback's non-notifying result. */
static uint8_t aethor_app_report_control_group_failure(CanTxSchedulerStatus status, uint64_t now_us)
{ assert(status != CAN_TX_SCHEDULER_STATUS_OK && now_us); return 0U; }
/** @brief No capability receipt is fabricated by the fixture. */
static uint8_t stm32_platform_can_collect_capability_receipt(CanFrame *frame, uint8_t *succeeded)
{ (void)frame; (void)succeeded; return 0U; }
/** @brief Retain the existing capability-failure callback separately from maintenance receipts. */
static void aethor_app_adrc_report_capability_transmit(const CanFrame *frame, uint8_t succeeded, uint64_t now_us)
{ assert(frame && !succeeded && now_us); ++fixture.capability_failures; }
/** @brief Provide no independent ADRC receipt. */
AdrcCanReceipt stm32_adrc_channel_collect(void) { AdrcCanReceipt receipt = {0}; return receipt; }
/** @brief Count dedicated-channel submission without claiming hardware transmission. */
uint8_t stm32_adrc_channel_submit(const CanFrame *frame, AdrcCanKind kind, float torque)
{ assert(fixture.critical_depth && frame); (void)kind; (void)torque; ++fixture.adrc_calls; return 1U; }
/** @brief Yield a pending probe only if Core chooses to call the producer. */
static uint8_t aethor_app_integrated_pop_probe_frame(CanFrame *frame, uint64_t now_us)
{
    assert(frame && now_us); ++fixture.probe_pops;
    if (fixture.probe_ready && fixture.mutate_after_pop) ++fixture.generation;
    return fixture.probe_ready;
}
/** @brief Yield pending ADRC only if Core chooses to call the producer. */
static uint8_t aethor_app_adrc_pop_frame(CanFrame *frame, uint8_t *kind, float *torque)
{
    assert(frame); ++fixture.adrc_pops; *kind = ADRC_CAN_TORQUE; *torque = 0.0F;
    if (fixture.adrc_ready && fixture.mutate_after_pop) ++fixture.generation;
    return fixture.adrc_ready;
}
/** @brief Preserve unrelated capability RX observation hooks. */
static void aethor_app_adrc_observe_capability_rx_boundary(uint64_t now_us) { (void)now_us; }
/** @brief Model the published empty boundary without supplying hardware proof. */
static uint64_t stm32_platform_can_capability_rx_boundary(void) { return 7999ULL; }
/** @brief Keep FIFO-observation call sites executable. */
static void stm32_platform_can_observe_rx_boundary(uint64_t now_us) { assert(now_us); }
/** @brief Permit the unchanged integrated idle callback to execute. */
static uint8_t stm32_platform_can_is_idle(void) { return 1U; }
/** @brief Keep integrated activity accounting visible in the extracted code. */
static void aethor_app_integrated_note_legacy_can_activity(void) { }
/** @brief Keep integrated idle accounting visible in the extracted code. */
static void aethor_app_integrated_set_can_idle(uint8_t idle, uint64_t now_us) { assert(idle && now_us); }
/** @brief No unrelated application work is scheduled by this fixture. */
static uint8_t aethor_app_service(uint64_t now_us) { assert(now_us); return 0U; }
/** @brief Preserve the unchanged trial-pause branch without invoking it. */
static uint8_t aethor_app_adrc_take_guard_pause(void) { return 0U; }
/** @brief Preserve probe reporting without creating completion evidence. */
static void aethor_app_integrated_report_probe_transmit(uint8_t succeeded, uint64_t now_us)
{ (void)succeeded; assert(now_us); }
/** @brief Preserve dedicated-channel reporting without creating completion evidence. */
static void aethor_app_adrc_report_transmit_at(uint8_t kind, float torque, uint8_t succeeded, uint64_t now_us)
{ (void)kind; (void)torque; (void)succeeded; assert(now_us); }
/** @brief Preserve the dedicated disable submission callback. */
static void aethor_app_adrc_report_disable_submit_at(uint8_t kind, uint64_t now_us) { (void)kind; assert(now_us); }
/** @brief Retain the queue service boundary; no real HAL or bus is accessed. */
static uint8_t stm32_platform_can_service_tx(uint8_t maximum)
{
    assert(maximum == ARM_JOINT_COUNT);
    if (deferred_strict_submitted) {
        ++fixture.irq_depth;
        if (fixture.link_changes_at_final) fixture.link_word ^= fixture.link_change_mask;
        (void)deferred_strict_submitted(&fixture.transfer.disable_identity, 8001ULL);
        deferred_strict_submitted = NULL; --fixture.irq_depth;
    }
    return 1U;
}
/** @brief Preserve synthetic notification return values. */
static unsigned xTaskNotifyGive(void *handle) { (void)handle; return 1U; }
#define TaskHandle_t void *
/** @brief Keep the unused pause branch valid when compiling the complete control loop. */
static void vTaskDelay(unsigned ticks) { (void)ticks; }
/** @brief Supply a synthetic tick for the unchanged guard-pause branch. */
static TickType_t xTaskGetTickCount(void) { return 8U; }
/** @brief Exit after exactly one actual Core loop iteration. */
static void vTaskDelayUntil(TickType_t *last_wake, unsigned ticks)
{ assert(last_wake && ticks == 4U && !fixture.critical_depth); longjmp(task_return, 1); }

#include "persistence_core_extracted.inc"

/** @brief Reset independently controllable interfaces with nontrivial identity values. */
static void ResetFixture(void)
{
    memset(&fixture, 0, sizeof(fixture)); fixture.generation = 41U;
    deferred_strict_submitted = NULL;
    fixture.link_word = fixture.bound_link_word = 0x80000019U;
    fixture.link_change_mask = 1U;
    fixture.owner_valid = fixture.parameter_accepts = fixture.strict_accepts = 1U;
    fixture.transfer.kind = AETHOR_PERSISTENCE_TRANSFER_PARAMETER;
    fixture.transfer.frame.identifier = 0x7FFU; fixture.transfer.frame.length = 4U;
    fixture.transfer.frame.data[0] = 7U; fixture.transfer.frame.data[2] = 0x33U;
    fixture.transfer.frame.data[3] = 11U;
    fixture.transfer.parameter_identity = (MotorPersistenceIdentity){11U, 22U, 41U, 57U, 3U};
    fixture.transfer.disable_identity = (DebugUiStaticTxIdentity){11U, 23U, 58U, 7U, DEBUG_UI_STATIC_TX_PREFLIGHT};
    fixture.parameter_receipt = fixture.cancelled = fixture.transfer.parameter_identity;
    fixture.strict_receipt = fixture.rx_identity = fixture.transfer.disable_identity;
    fixture.receipt_sof = 123U; fixture.rx_sof = 129U; fixture.rx_sof_valid = 1U;
    fixture.completed_us = 7987ULL;
}
/** @brief Use the exact selected classic DISABLE bytes, independently of parameter-request encoding. */
static void SetDisableTransfer(void)
{
    fixture.transfer.kind = AETHOR_PERSISTENCE_TRANSFER_DISABLE;
    fixture.transfer.frame.identifier = 7U; fixture.transfer.frame.length = 8U;
    memset(fixture.transfer.frame.data, 0xFF, 7U); fixture.transfer.frame.data[7] = 0xFDU;
}
/** @brief Execute one real extracted control iteration, then assert it released every task lock. */
static void Tick(void)
{ if (!setjmp(task_return)) RunExtractedControlTask(); assert(!fixture.critical_depth && !fixture.irq_depth); }
/** @brief Check unique parameter receipt/cancellation identity and maintenance-first strict receipt ownership. */
static void TestReceipts(void)
{
    unsigned succeeded, owned;
    for (succeeded = 0U; succeeded < 2U; ++succeeded) {
        ResetFixture(); fixture.parameter_receipt_ready = fixture.cancel_ready = 1U;
        fixture.last_succeeded = (uint8_t)succeeded; Tick(); Tick();
        assert(fixture.parameter_receipts == 1U && fixture.cancel_calls >= 1U);
        assert(ParameterEqual(&fixture.received_parameter, &fixture.parameter_receipt));
        assert(fixture.last_succeeded == succeeded && fixture.received_completed_us == 7987ULL && fixture.received_now_us == 8000ULL);
    }
    for (owned = 0U; owned < 2U; ++owned) {
        ResetFixture(); fixture.strict_receipt_ready = 1U; fixture.receipt_owned = (uint8_t)owned; Tick();
        assert(fixture.strict_receipts == 1U && fixture.legacy_receipts == 1U - owned);
    }
}
/** @brief Check parameter bytes bypass the old collector, refusal cannot fall through, and FD uses the dedicated strict callback. */
static void TestRouting(void)
{
    static const uint8_t overlapping_rids[] = {1U, 2U, 3U, 11U, 12U, 20U};
    unsigned accepts, index;
    for (accepts = 0U; accepts < 2U; ++accepts) for (index = 0U; index < sizeof(overlapping_rids); ++index) {
        ResetFixture(); fixture.locked = fixture.transfer_ready = 1U; fixture.parameter_accepts = (uint8_t)accepts;
        fixture.transfer.frame.data[3] = overlapping_rids[index];
        Tick(); Tick();
        assert(fixture.parameter_calls == 1U && fixture.rejected_calls == 1U - accepts);
        assert(fixture.parameter_commits == accepts && !fixture.generic_calls && !fixture.disable_calls && !fixture.capability_failures);
        ResetFixture(); fixture.locked = fixture.transfer_ready = 1U;
        SetDisableTransfer();
        fixture.strict_accepts = (uint8_t)accepts; Tick(); Tick();
        assert(fixture.disable_calls == 1U && fixture.disable_commits == accepts);
        assert(fixture.rejected_calls == 1U - accepts && !fixture.parameter_calls && !fixture.generic_calls);
    }
}
/** @brief Check final callback observes a reconnect occurring after the tick's original observation under IRQ mask. */
static void TestFinalLink(void)
{
    unsigned disable, configured;
    for (disable = 0U; disable < 2U; ++disable) for (configured = 0U; configured < 2U; ++configured) {
        ResetFixture(); fixture.locked = fixture.transfer_ready = fixture.link_changes_at_final = 1U;
        if (disable) SetDisableTransfer();
        fixture.link_change_mask = configured ? 0x80000000U : 1U;
        Tick();
        assert(fixture.observe_count >= 2U && fixture.final_observe_count == 1U);
        assert(!fixture.owner_valid && !fixture.parameter_commits && !fixture.disable_commits);
        assert(!fixture.rejected_calls && fixture.observed_link_word == fixture.link_word);
    }
}
/** @brief Check pre-pop generation capture and the final post-pop permit on frames and groups. */
static void TestGeneration(void)
{
    unsigned producer, mutate;
    for (producer = 0U; producer < 4U; ++producer) for (mutate = 0U; mutate < 2U; ++mutate) {
        ResetFixture(); fixture.expected_critical_pop = 1U; fixture.mutate_after_pop = (uint8_t)mutate;
        if (producer == 0U) fixture.ordinary_ready = 1U;
        else if (producer == 1U) fixture.group_ready = 1U;
        else if (producer == 2U) fixture.probe_ready = 1U;
        else fixture.adrc_ready = 1U;
        Tick(); assert(fixture.capture_count && fixture.allow_count);
        assert(fixture.generic_calls + fixture.group_calls + fixture.adrc_calls == 1U - mutate);
    }
}
/** @brief Check queued legacy frame/group and dedicated ADRC/probe producers cannot enter Platform while maintenance owns the bus. */
static void TestMutex(void)
{
    ResetFixture(); fixture.locked = fixture.ordinary_ready = fixture.group_ready = fixture.probe_ready = fixture.adrc_ready = 1U;
    Tick(); assert(!fixture.generic_calls && !fixture.group_calls && !fixture.adrc_calls);
    assert(!fixture.probe_pops && !fixture.adrc_pops);
}
/** @brief Check maintenance/quarantine consumption precedes legacy feedback decoding and preserves RX ordering data. */
static void TestReceive(void)
{
    unsigned consumed, ordered;
    CanFrame frame = {0}; frame.identifier = 0x23U; frame.length = 4U; frame.data[2] = 0xAAU;
    for (consumed = 0U; consumed < 2U; ++consumed) for (ordered = 0U; ordered < 2U; ++ordered) {
        ResetFixture(); fixture.rx_consumed = (uint8_t)consumed; fixture.ordered_rx = (uint8_t)ordered;
        taskENTER_CRITICAL(); ReceiveSingleStaticFrame(&frame, 7990ULL, 8000ULL); taskEXIT_CRITICAL();
        assert(fixture.rx_calls == 1U && fixture.legacy_rx_calls == 1U - consumed);
        assert(fixture.rx_lower_us == 7990ULL && fixture.rx_observed_us == 8000ULL);
    }
}
/** @brief Check an independent queued emergency still reaches its original physical stop route under maintenance. */
static void TestEmergency(void)
{
    ResetFixture(); fixture.locked = fixture.emergency_ready = 1U; Tick();
    assert(fixture.emergency_calls == 1U && fixture.generic_calls == 1U);
    ResetFixture(); fixture.locked = fixture.ordinary_ready = fixture.ordinary_emergency = 1U; Tick();
    assert(fixture.emergency_calls == 1U && fixture.generic_calls == 1U);
}
/** @brief Select deterministic behavior cases; a failing assert is behavior RED, never a compile error. */
int main(int argc, char **argv)
{
    const char *selected = argc > 1 ? argv[1] : "all";
#define RUN_CASE(name, function) do { if (!strcmp(selected, "all") || !strcmp(selected, name)) { function(); puts("PERSISTENCE_CORE_CASE_PASS " name); } } while (0)
    RUN_CASE("receipt", TestReceipts); RUN_CASE("route", TestRouting); RUN_CASE("link", TestFinalLink);
    RUN_CASE("generation", TestGeneration); RUN_CASE("mutex", TestMutex); RUN_CASE("rx", TestReceive);
    RUN_CASE("emergency", TestEmergency);
    puts("PERSISTENCE_CORE_PORT_CONTRACT_PASS extracted_core=1 integrated=1 single=1 pos=1 persistence=1 production_app_hal_closed_loop=0");
    return 0;
}
