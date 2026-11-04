/** @file test_shaft_pilot.c
 * @brief Synthetic-only host fixtures; no CAN send, motor evidence, or physical qualification.
 */
#include "adrc_shaft_pilot.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Synthetic caller state and integer-grid stand-in, never a Motor implementation proof. */
typedef struct { ShaftPilot pilot; ShaftPilotConfig config; ShaftPilotPlan plan;
 ShaftPilotInput input; ShaftPilotOutput output; unsigned checks; } Fixture;
static Fixture fixture;
static unsigned completed;
static volatile unsigned default_completed;
static volatile unsigned low_completed;
static volatile unsigned visible_completed;
static volatile unsigned observe_completed;
static volatile unsigned platform_completed;
static volatile unsigned wide_completed;
static volatile unsigned rated70_completed;
/** @brief New fixed API expected by failing tests before production implementation. */
extern int shaft_pilot_init_visible(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief Fixed lower-energy VMAX=5 diagnostic, never an arbitrary cap override. */
extern int shaft_pilot_init_fine(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief Approved fixed observation API is absent until the production implementation passes RED. */
extern int shaft_pilot_init_observe(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief Fixed range-bounded platform API expected by RED before production implementation. */
extern int shaft_pilot_init_platform(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief 固定宽位置量程API的RED入口；不存在时必须链接失败。 */
extern int shaft_pilot_init_platform_wide(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);

/** @brief 固定70%平台API的RED入口；不存在时必须链接失败。 */
extern int shaft_pilot_init_platform70(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);
/** @brief 新固定100%速度且1Nm档必须先通过缺失API的RED链接检查。 */
extern int shaft_pilot_init_platform100(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction);

/** @brief Build deliberately synthetic range/baseline data rather than current readback defaults. */
static void prepare(int direction)
{
 memset(&fixture,0,sizeof(fixture));
 fixture.config.motor_id=7U; fixture.config.binding_id=123U;
 fixture.config.session_id=456U; fixture.config.gear_ratio=20.0F;
 fixture.config.position_max_native_rad=100.0F; fixture.config.velocity_max_rad_s=200.0F;
 fixture.config.torque_max_nm=10.0F; fixture.config.baseline.binding_id=123U;
 fixture.config.baseline.session_id=456U; fixture.config.baseline.point_count=64U;
 fixture.config.baseline.span64_shaft_rad=0.00001F;
 fixture.config.baseline.velocity32_abs_peak_rad_s=0.0003F;
 fixture.config.baseline.raw_zero_center_rad_s=200.0F/4095.0F;
 fixture.config.baseline.torque_abs_peak_nm=0.01F;
 fixture.plan.direction=direction; fixture.plan.cap_nm=1.75F;
 fixture.plan.slew_nm_s=4.0F; fixture.plan.step_nm=0.016F;
 fixture.input.now_us=100000U; fixture.input.motor_id=7U;
 fixture.input.binding_id=123U; fixture.input.session_id=456U;
 fixture.input.guard_armed=1U; fixture.input.guard_live=1U;
 fixture.input.feedback.valid=1U; fixture.input.feedback.rx_us=100000U;
 fixture.input.feedback.observed_us=100000U;
 fixture.input.feedback.raw_velocity_rad_s=fixture.config.baseline.raw_zero_center_rad_s;
 fixture.input.feedback.mos_temperature_c=25.0F;
 fixture.input.feedback.rotor_temperature_c=26.0F;
 fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
}

/** @brief Quantize a requested torque on a synthetic grid while retaining every output bound. */
static float synthetic_grid(const ShaftPilotOutput *output)
{
 float best=0.0F, error=1.0e9F; unsigned code; int found=0;
 for(code=0U;code<=4095U;++code) {
  float decoded=(float)code*20.0F/4095.0F-10.0F;
  float difference=fabsf(decoded-output->torque_nm);
  if(fabsf(decoded)<=output->cap_nm &&
     fabsf(decoded-output->previous_sent_nm)<=output->maximum_step_nm && difference<error) {
   found=1; error=difference; best=decoded;
  }
 }
 assert(found); return best;
}

/** @brief Advance synthetic observation and return the previous matching action receipt. */
static void advance(unsigned delta_us)
{
 ShaftPilotOutput previous=fixture.output;
 fixture.input.now_us+=delta_us;
 fixture.input.feedback.rx_us=fixture.input.now_us;
 fixture.input.feedback.observed_us=fixture.input.now_us;
 memset(&fixture.input.receipt,0,sizeof(fixture.input.receipt));
 if(previous.action!=SHAFT_ACTION_NONE) {
  fixture.input.receipt.valid=1U; fixture.input.receipt.success=1U;
  fixture.input.receipt.kind=previous.action; fixture.input.receipt.command_id=previous.command_id;
  fixture.input.receipt.completed_observed_us=previous.issued_us;
  if(previous.action==SHAFT_ACTION_ENABLE) fixture.input.feedback.driver_state=SHAFT_DRIVER_ENABLED;
  if(previous.action==SHAFT_ACTION_DISABLE) fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
  if(previous.action==SHAFT_ACTION_TORQUE) fixture.input.receipt.decoded_torque_nm=synthetic_grid(&previous);
 }
}

/** @brief Perform one synthetic scheduler tick with a successfully matched synthetic receipt. */
static void tick(unsigned delta_us)
{ advance(delta_us); shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output); }

/** @brief Start once from synthetic disabled feedback. */
static void begin(int direction)
{
 prepare(direction); assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 assert(fixture.output.action==SHAFT_ACTION_ENABLE);
}

/** @brief Reach ramp through a finite 64ms neutral phase without injecting a controller. */
static void reach_ramp(int direction)
{
 unsigned count; begin(direction);
 for(count=0U;count<20U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) tick(4000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
}

/** @brief Verify all positive finite ranges, frozen baseline identity, and plan limits are required. */
static void test_config(void)
{
 unsigned index; float *ranges[4];
 prepare(1); assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 assert(fabsf(fixture.pilot.status.quantum_shaft_rad-200.0F/65535.0F/20.0F)<1.0e-8F);
 assert(fabsf(fixture.pilot.status.quantum_velocity_rad_s-400.0F/4095.0F)<1.0e-6F);
 ranges[0]=&fixture.config.gear_ratio; ranges[1]=&fixture.config.position_max_native_rad;
 ranges[2]=&fixture.config.velocity_max_rad_s; ranges[3]=&fixture.config.torque_max_nm;
 for(index=0U;index<4U;++index) {
  float original=*ranges[index]; *ranges[index]=NAN;
  assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
  *ranges[index]=INFINITY; assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
  *ranges[index]=0.0F; assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
  *ranges[index]=original;
 }
 fixture.config.motor_id=6U; assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(1); fixture.config.baseline.session_id=999U;
 assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(1); fixture.config.baseline.point_count=63U;
 assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(1); fixture.config.baseline.torque_abs_peak_nm=0.151F;
 assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(1); fixture.plan.cap_nm=1.751F; assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(1); fixture.plan.slew_nm_s=4.01F; assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(1); fixture.plan.step_nm=0.01601F; assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(0); assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(1); fixture.config.baseline.raw_zero_center_rad_s=0.049F;
 assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 ++completed;
}

/** @brief Reject stale/unhealthy/foreign entry and insufficient absolute native PMAX headroom. */
static void test_entry(void)
{
 prepare(1); assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 fixture.input.feedback.native_position_rad=92.0F;
 assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 assert(fixture.output.action==SHAFT_ACTION_NONE);
 fixture.input.feedback.native_position_rad=0.0F; fixture.input.guard_live=0U;
 assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 fixture.input.guard_live=1U; fixture.input.feedback.raw_velocity_rad_s=0.049F;
 assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 fixture.input.feedback.raw_velocity_rad_s=fixture.config.baseline.raw_zero_center_rad_s;
 fixture.input.motor_id=6U; assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 fixture.input.motor_id=7U; fixture.input.now_us+=8001U;
 assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 ++completed;
}

/** @brief Validate receipt match, failure, finite decode, step, amplitude and clock gates. */
static void test_receipts(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<9U;++scenario) {
  reach_ramp(1); advance(4000U);
  if(scenario==0U) fixture.input.receipt.valid=0U;
  if(scenario==1U) fixture.input.receipt.command_id++;
  if(scenario==2U) fixture.input.receipt.success=0U;
  if(scenario==3U) fixture.input.receipt.decoded_torque_nm=NAN;
  if(scenario==4U) fixture.input.receipt.decoded_torque_nm=2.0F;
  if(scenario==5U) fixture.input.receipt.decoded_torque_nm=0.2F;
  if(scenario==6U) fixture.input.receipt.completed_observed_us=fixture.input.now_us+1U;
  if(scenario==7U) fixture.input.receipt.completed_observed_us=fixture.output.issued_us-1U;
  if(scenario==8U) fixture.input.receipt.kind=SHAFT_ACTION_ENABLE;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_RECEIPT);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 }
 ++completed;
}

/** @brief Verify requested ramp follows decoded grid receipts and both 3/5ms step bounds. */
static void test_ramp_grid(void)
{
 unsigned count, period;
 for(period=3000U;period<=5000U;period+=2000U) {
  reach_ramp(-1);
  for(count=0U;count<100U;++count) {
   tick(period);
   assert(fixture.output.action==SHAFT_ACTION_TORQUE);
   assert(fabsf(fixture.output.torque_nm-fixture.input.receipt.decoded_torque_nm)<=
     fminf(4.0F*(float)period*1.0e-6F,0.016F)+1.0e-7F);
   assert(fabsf(fixture.output.previous_sent_nm-fixture.input.receipt.decoded_torque_nm)<1.0e-7F);
   assert(fixture.output.previous_sent_valid);
   assert(fabsf(fixture.output.torque_nm)<=1.75F);
  }
 }
 ++completed;
}

/** @brief Verify first directed receive-window target ends early in both independent directions. */
static void test_target(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  reach_ramp(direction);
  for(count=0U;count<20U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
   fixture.input.feedback.native_position_rad+=(float)direction*0.12F*0.004F*20.0F;
   tick(4000U);
  }
  assert(fixture.pilot.status.outcome==SHAFT_END_TARGET);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 }
 ++completed;
}

/** @brief Verify exact 800ms natural end without response, cap extension, or automatic retry. */
static void test_time_end(void)
{
 unsigned count; reach_ramp(1);
 for(count=0U;count<199U;++count) tick(4000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
 tick(4000U); assert(fixture.pilot.status.outcome==SHAFT_END_TIME);
 assert(fixture.pilot.status.peak_successful_decoded_torque_nm<=fixture.plan.cap_nm);
 assert(fixture.pilot.status.peak_successful_decoded_torque_nm>=fixture.plan.cap_nm-
  fixture.pilot.status.quantum_torque_nm);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 ++completed;
}

/** @brief Verify independent live guard, schedule, freshness, state, fault and temperature gates. */
static void test_active_gates(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<10U;++scenario) {
  reach_ramp(1); advance(scenario==0U?2999U:(scenario==1U?5001U:4000U));
  if(scenario==2U) fixture.input.guard_live=0U;
  if(scenario==3U) fixture.input.feedback.rx_us=fixture.input.now_us-8001U;
  if(scenario==4U) fixture.input.feedback.mos_temperature_c=50.0F;
  if(scenario==5U) fixture.input.feedback.rotor_temperature_c=NAN;
  if(scenario==6U) fixture.input.feedback.driver_fault=1U;
  if(scenario==7U) fixture.input.feedback.driver_state=SHAFT_DRIVER_UNKNOWN;
  if(scenario==8U) fixture.input.feedback.raw_velocity_rad_s=0.301F;
  if(scenario==9U) fixture.input.binding_id++;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.pilot.status.outcome==SHAFT_END_FAULT);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 }
 ++completed;
}

/** @brief Test 32ms arrival interval gate independently of nominal position difference. */
static void test_arrival_window(void)
{
 unsigned count;
 /* Dedicated constant .295 window exceeds .3 after 2ms uncertainty although nominal is .295. */
 reach_ramp(-1);
 for(count=0U;count<9U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
  fixture.input.feedback.native_position_rad+=0.295F*0.004F*20.0F; tick(4000U);
 }
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_SPEED);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 ++completed;
}

/** @brief Adjacent Tmin zero permits displacement completeness only, never a finite velocity proof. */
static void test_adjacent_zero_tmin(void)
{
 begin(1); fixture.input.feedback.observed_us=fixture.input.now_us;
 advance(4000U); fixture.input.feedback.rx_us=fixture.input.now_us-2000U;
 fixture.input.feedback.native_position_rad=0.001F*20.0F;
 shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 assert(fixture.pilot.status.internal_sampling_unknown);
 assert(fixture.pilot.status.arrival_interval_conditional);
 ++completed;
}

/** @brief Stop recovery preserves fault, rejects anchor/receipt mismatch, and requires baseline dwell. */
static void test_stop_recovery(void)
{
 unsigned count; reach_ramp(1); advance(4000U);
 fixture.input.feedback.raw_velocity_rad_s=0.4F;
 shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_SPEED);
 fixture.input.feedback.raw_velocity_rad_s=fixture.config.baseline.raw_zero_center_rad_s;
 for(count=0U;count<30U && !fixture.pilot.status.safe_release;++count) tick(4000U);
 assert(fixture.pilot.status.safe_release);
 assert(fixture.pilot.status.outcome==SHAFT_END_FAULT);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_SPEED);
 assert(!fixture.pilot.status.caller_next_transaction_ready);
 fixture.input.guard_fired=1U; fixture.input.guard_acked=1U; tick(4000U);
 assert(!fixture.pilot.status.caller_next_transaction_ready);
 ++completed;
}

/** @brief Next-transaction readiness requires a normal endpoint, no fault/abort/lock, and real guard ack. */
static void test_next_transaction_gate(void)
{
 unsigned count, scenario;
 reach_ramp(1);
 for(count=0U;count<200U;++count) tick(4000U);
 fixture.input.guard_fired=1U; fixture.input.guard_acked=1U;
 for(count=0U;count<30U && !fixture.pilot.status.safe_release;++count) tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_TIME);
 assert(fixture.pilot.status.caller_next_transaction_ready);
 for(scenario=0U;scenario<3U;++scenario) {
  begin(1); fixture.input.guard_fired=1U; fixture.input.guard_acked=1U;
  if(scenario==0U) fixture.input.stop_requested=1U;
  if(scenario==1U) fixture.pilot.status.trace_count=SHAFT_PILOT_TRACE_CAPACITY;
  if(scenario==2U) fixture.pilot.origin_shaft_rad=-0.5F; /* Fault injection tests the travel gate directly. */
  tick(4000U); fixture.input.stop_requested=0U;
  for(count=0U;count<30U && !fixture.pilot.status.safe_release;++count) tick(4000U);
  assert(fixture.pilot.status.safe_release);
  assert(!fixture.pilot.status.caller_next_transaction_ready);
 }
 ++completed;
}

/** @brief Feedback gap rebuilds stop history and restarts dwell without clearing the first fault. */
static void test_stop_gap(void)
{
 unsigned count; reach_ramp(1); fixture.input.stop_requested=1U; tick(4000U);
 fixture.input.stop_requested=0U;
 for(count=0U;count<15U;++count) tick(4000U);
 tick(12000U); assert(!fixture.pilot.status.safe_release);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_FEEDBACK);
 for(count=0U;count<30U && !fixture.pilot.status.safe_release;++count) tick(4000U);
 assert(fixture.pilot.status.safe_release);
 assert(fixture.pilot.status.outcome==SHAFT_END_FAULT);
 ++completed;
}

/** @brief Lock deadline and trace overflow require continuing DISABLE and freeze bounded evidence. */
static void test_lock_and_trace(void)
{
 unsigned count; ShaftPilotTrace first;
 begin(1); fixture.input.stop_requested=1U; tick(3000U);
 fixture.input.stop_requested=0U;
 for(count=0U;count<500U;++count) {
  advance(3000U); fixture.input.feedback.driver_state=SHAFT_DRIVER_ENABLED;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(fixture.pilot.status.phase==SHAFT_PHASE_LOCKED);
 assert(fixture.pilot.status.trace_count<=502U);
 count=fixture.pilot.status.trace_count; tick(3000U);
 assert(fixture.pilot.status.trace_count==count);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 assert(shaft_pilot_trace_get(&fixture.pilot,0U,&first));
 assert(!shaft_pilot_trace_get(&fixture.pilot,SHAFT_PILOT_TRACE_CAPACITY,&first));
 begin(1); fixture.pilot.status.trace_count=SHAFT_PILOT_TRACE_CAPACITY;
 tick(4000U); assert(fixture.pilot.status.phase==SHAFT_PHASE_LOCKED);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_TRACE);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 assert(fixture.pilot.status.trace_count==SHAFT_PILOT_TRACE_CAPACITY);
 ++completed;
}

/** @brief Stop receipt must match a post-anchor disable and RX must be strictly after STOP. */
static void test_stop_anchor_receipt(void)
{
 unsigned count; reach_ramp(1); fixture.input.stop_requested=1U; tick(4000U);
 fixture.input.stop_requested=0U;
 for(count=0U;count<30U;++count) {
  advance(4000U); fixture.input.receipt.command_id++;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(!fixture.pilot.status.safe_release);
 for(count=0U;count<30U && !fixture.pilot.status.safe_release;++count) tick(4000U);
 assert(fixture.pilot.status.safe_release);
 assert(fixture.pilot.status.outcome==SHAFT_END_FAULT);
 ++completed;
}

/** @brief Stop raw zero, baseline velocity and dwell span reject neighboring/still-moving samples. */
static void test_stop_baseline(void)
{
 unsigned count; reach_ramp(1); fixture.input.stop_requested=1U; tick(4000U);
 fixture.input.stop_requested=0U;
 fixture.input.feedback.raw_velocity_rad_s=0.049F;
 for(count=0U;count<30U;++count) tick(4000U);
 assert(!fixture.pilot.status.safe_release);
 fixture.input.feedback.raw_velocity_rad_s=fixture.config.baseline.raw_zero_center_rad_s;
 for(count=0U;count<40U;++count) {
  fixture.input.feedback.native_position_rad+=0.000024F*20.0F; tick(4000U);
 }
 assert(!fixture.pilot.status.safe_release);
 for(count=0U;count<40U && !fixture.pilot.status.safe_release;++count) tick(4000U);
 assert(fixture.pilot.status.safe_release);
 ++completed;
}

/** @brief No invalid scalar or duplicate RX mutation can enter the history or produce torque. */
static void test_feedback_integrity(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<6U;++scenario) {
  reach_ramp(1); advance(4000U);
  if(scenario==0U) fixture.input.feedback.native_position_rad=INFINITY;
  if(scenario==1U) fixture.input.feedback.torque_nm=NAN;
  if(scenario==2U) fixture.input.feedback.observed_us=fixture.input.now_us+1U;
  if(scenario==3U) fixture.input.feedback.rx_us=fixture.input.now_us+1U;
  if(scenario==4U) { fixture.input.feedback.rx_us-=4000U; fixture.input.feedback.native_position_rad=0.001F; }
  if(scenario==5U) fixture.input.feedback.native_position_rad=0.1F;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  assert(fixture.pilot.status.first_fault!=SHAFT_FAULT_NONE);
 }
 ++completed;
}

/** @brief Recorded readback values are used only as offline compatibility data, with synthetic identity/time.
 * @note Source current_output_axis_readback_20261014.json says current_owner_epoch_valid_now=false.
 */
static void test_recorded_config_compatibility(void)
{
 prepare(1); fixture.config.gear_ratio=19.20319938659668F;
 fixture.config.position_max_native_rad=12.5F;
 fixture.config.baseline.span64_shaft_rad=1.9864904921325534e-5F;
 fixture.config.baseline.velocity32_abs_peak_rad_s=0.0006207782787914229F;
 fixture.config.baseline.torque_abs_peak_nm=0.0122098923F;
 fixture.config.baseline.raw_zero_center_rad_s=-0.0488433838F;
 fixture.input.feedback.raw_velocity_rad_s=-0.0488433838F;
 assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 assert(fabsf(fixture.pilot.status.quantum_shaft_rad-1.986520786133535e-5F)<1.0e-10F);
 assert(fabsf(fixture.pilot.status.initial_stop_torque_gate_nm-0.021977902068F)<1.0e-7F);
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 ++completed;
}

/** @brief Neutral 64ms minimum permits one 5ms sampling margin, rejecting age >=69ms. */
static void test_neutral_deadline(void)
{
 unsigned count; begin(1); tick(4000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_NEUTRAL);
 for(count=0U;count<12U;++count) tick(5000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_NEUTRAL);
 tick(5000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
 begin(1); tick(3000U);
 for(count=0U;count<22U;++count) tick(3000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
 begin(1); tick(4000U);
 for(count=0U;count<12U;++count) tick(5000U);
 tick(3000U); advance(3000U);
 fixture.input.feedback=fixture.pilot.previous_feedback;
 shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_NEUTRAL);
 tick(3000U);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NEUTRAL_TIMEOUT);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 ++completed;
}

/** @brief Final successful decoded torque remains recorded before same-tick STOP or guard abort. */
static void test_final_receipt_before_stop(void)
{
 unsigned scenario; ShaftPilotTrace record;
 for(scenario=0U;scenario<2U;++scenario) {
  reach_ramp(1); tick(4000U); advance(4000U);
  if(scenario==0U) fixture.input.stop_requested=1U;
  else fixture.input.guard_live=0U;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  assert(fixture.pilot.status.last_successful_decoded_torque_nm==fixture.input.receipt.decoded_torque_nm);
  assert(fixture.pilot.status.peak_successful_decoded_torque_nm>=fabsf(fixture.input.receipt.decoded_torque_nm));
  assert(shaft_pilot_trace_get(&fixture.pilot,fixture.pilot.status.trace_count-1U,&record));
  assert(record.input.receipt.kind==SHAFT_ACTION_TORQUE);
  assert(record.input.receipt.decoded_torque_nm==fixture.input.receipt.decoded_torque_nm);
  assert(record.output.action==SHAFT_ACTION_DISABLE);
 }
 ++completed;
}

/** @brief Arrival after the absolute 1.5s lock deadline cannot promote a natural DONE outcome. */
static void test_late_stop_completion(void)
{
 unsigned count; begin(1); fixture.input.stop_requested=1U; tick(3000U);
 fixture.input.stop_requested=0U;
 for(count=0U;count<477U;++count) {
  advance(3000U); fixture.input.feedback.driver_state=SHAFT_DRIVER_ENABLED;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 tick(3000U);
 for(count=0U;count<20U;++count) tick(3000U);
 assert(!fixture.pilot.status.safe_release);
 tick(8000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_LOCKED);
 assert(fixture.pilot.status.safe_release);
 assert(fixture.pilot.status.outcome==SHAFT_END_FAULT);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 ++completed;
}

/** @brief Derived stop gates must also be finite, even when every supplied range is positive finite. */
static void test_derived_finite(void)
{
 prepare(1); fixture.config.gear_ratio=1.0e-40F;
 assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(1); fixture.config.position_max_native_rad=1.0e10F;
 fixture.config.gear_ratio=1.0e-29F;
 assert(!shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 prepare(1); assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 /* Corrupted frozen-state injection separately exercises start's finite conversion defense. */
 fixture.pilot.config.position_max_native_rad=1.0e10F;
 fixture.pilot.config.gear_ratio=1.0e-29F;
 fixture.input.feedback.native_position_rad=9.0e9F;
 assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 assert(fixture.output.action==SHAFT_ACTION_NONE);
 ++completed;
}

/** @brief Matched successful finite out-of-bound actual sends remain diagnostic evidence, never ramp bases. */
static void test_violating_send_diagnostic(void)
{
 unsigned scenario; float previous, peak;
 for(scenario=0U;scenario<4U;++scenario) {
  reach_ramp(1); previous=fixture.pilot.last_sent_nm;
  peak=fixture.pilot.status.peak_successful_decoded_torque_nm;
  advance(4000U); fixture.input.receipt.decoded_torque_nm=1.9F;
  if(scenario==1U) fixture.input.receipt.command_id++;
  if(scenario==2U) fixture.input.receipt.success=0U;
  if(scenario==3U) fixture.input.receipt.decoded_torque_nm=NAN;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  assert(fixture.pilot.last_sent_nm==previous);
  if(scenario==0U) {
   assert(fixture.pilot.status.last_successful_decoded_torque_nm==1.9F);
   assert(fixture.pilot.status.peak_successful_decoded_torque_nm==1.9F);
  } else assert(fixture.pilot.status.peak_successful_decoded_torque_nm==peak);
 }
 ++completed;
}

/** @brief First neutral zero is only a nominal grid reference until an actual successful receipt exists. */
static void test_neutral_bootstrap_reference(void)
{
 begin(1); tick(4000U);
 assert(fixture.output.action==SHAFT_ACTION_TORQUE);
 assert(!fixture.output.previous_sent_valid);
 assert(fixture.output.previous_sent_nm==0.0F);
 tick(4000U); assert(fixture.output.previous_sent_valid);
 assert(fixture.output.previous_sent_nm==fixture.input.receipt.decoded_torque_nm);
 ++completed;
}

/** @brief Initialize fixed low diagnostic semantics using only synthetic binding and baseline. */
static int initialize_low(int direction)
{
 return shaft_pilot_init_low(&fixture.pilot,&fixture.config,direction);
}

/** @brief Synthetic readback-scale grid makes the short displacement gate observable; no plant prediction. */
static void low_begin(int direction)
{
 prepare(direction); fixture.config.gear_ratio=19.2031994F;
 /* Keep absolute-deadline injection above zero; unsigned underflow tests clock order instead. */
 fixture.input.now_us=2000000U;
 fixture.input.feedback.rx_us=fixture.input.now_us;
 fixture.input.feedback.observed_us=fixture.input.now_us;
 fixture.config.position_max_native_rad=12.5F;
 fixture.config.baseline.span64_shaft_rad=2.0F*(25.0F/65535.0F/fixture.config.gear_ratio);
 assert(initialize_low(direction));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
}

/** @brief Observe matching synthetic completion one millisecond before the consuming tick. */
static void low_tick(unsigned period_us)
{
 advance(period_us);
 if(fixture.input.receipt.kind==SHAFT_ACTION_TORQUE)
  fixture.input.receipt.completed_observed_us=fixture.input.now_us-1000U;
 shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
}

/** @brief Reach low ramp at each allowed synthetic scheduling interval. */
static void low_ramp(int direction, unsigned period_us)
{
 unsigned count; low_begin(direction);
 for(count=0U;count<24U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(period_us);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
}

/** @brief Fixed low cap is frozen, cannot exceed readback, and invalid direction is rejected. */
static void test_low_config(void)
{
 prepare(1);
 fixture.config.velocity_max_rad_s=5.0F;
 fixture.config.baseline.raw_zero_center_rad_s=5.0F/4095.0F;
 assert(shaft_pilot_init_fine(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.plan.cap_nm==0.20F && fixture.pilot.plan.slew_nm_s==4.0F &&
  fixture.pilot.plan.step_nm==0.016F && fixture.pilot.low_profile && !fixture.pilot.visible_profile);
 fixture.config.velocity_max_rad_s=200.0F;
 assert(!shaft_pilot_init_fine(&fixture.pilot,&fixture.config,1));
 prepare(1); assert(initialize_low(1));
 assert(fixture.pilot.plan.cap_nm==0.35F && fixture.pilot.plan.slew_nm_s==4.0F);
 assert(fixture.pilot.plan.step_nm==0.016F);
 assert(fixture.pilot.status.running_torque_gate_nm==0.35F+fixture.pilot.status.initial_stop_torque_gate_nm);
 assert(!initialize_low(0)); assert(!initialize_low(2));
 fixture.config.torque_max_nm=0.3F; assert(!initialize_low(1));
 assert(!shaft_pilot_init_low(NULL,&fixture.config,1));
 assert(!shaft_pilot_init_low(&fixture.pilot,NULL,1));
 ++completed;
}

/** @brief Constant .008 rad/s is a synthetic slow response, never a calibrated physical model.
 * @note Both directions and 3/4/5ms use actual decoded fixture codes and first receipt upper bound.
 */
static void test_low_plateau_time(void)
{
 unsigned period_us, count; int direction;
 for(direction=-1;direction<=1;direction+=2) for(period_us=3000U;period_us<=5000U;period_us+=1000U) {
  uint64_t plateau_observed_us=0U; low_ramp(direction,period_us);
  for(count=0U;count<140U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
   uint64_t previous_now=fixture.input.now_us;
   fixture.input.feedback.native_position_rad+=(float)direction*0.008F*(float)period_us*1.0e-6F*fixture.config.gear_ratio;
   low_tick(period_us);
   if(!plateau_observed_us && (float)direction*fixture.input.receipt.decoded_torque_nm>0.0F &&
      0.35F-(float)direction*fixture.input.receipt.decoded_torque_nm<fixture.pilot.status.quantum_torque_nm)
    plateau_observed_us=fixture.input.receipt.completed_observed_us;
   assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
   assert(fixture.output.cap_nm==0.35F);
   assert(fabsf(fixture.input.receipt.decoded_torque_nm)<=0.35F);
   if(fixture.pilot.status.phase==SHAFT_PHASE_RAMP) {
    assert(!plateau_observed_us || fixture.input.now_us-plateau_observed_us<80000U);
    assert(fabsf(fixture.output.torque_nm-fixture.input.receipt.decoded_torque_nm)<=
     fminf(4.0F*(float)period_us*1.0e-6F,0.016F)+1.0e-7F);
   } else {
    assert(plateau_observed_us && fixture.input.now_us-plateau_observed_us>=80000U);
    assert(previous_now<plateau_observed_us || previous_now-plateau_observed_us<80000U);
   }
  }
  assert(fixture.pilot.status.outcome==SHAFT_END_TIME);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  assert(fixture.input.now_us-fixture.pilot.phase_us<800000U);
 }
 for(direction=-1;direction<=1;direction+=2) {
  uint64_t plateau=0U;
  prepare(direction);
  fixture.config.velocity_max_rad_s=5.0F;
  fixture.config.baseline.raw_zero_center_rad_s=5.0F/4095.0F;
  fixture.input.feedback.raw_velocity_rad_s=fixture.config.baseline.raw_zero_center_rad_s;
  assert(shaft_pilot_init_fine(&fixture.pilot,&fixture.config,direction));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  for(count=0U;count<200U && fixture.pilot.status.phase!=SHAFT_PHASE_STOPPING;++count) {
   low_tick(4000U);
   if(!plateau && fixture.pilot.status.phase==SHAFT_PHASE_RAMP &&
      0.20F-(float)direction*fixture.input.receipt.decoded_torque_nm<fixture.pilot.status.quantum_torque_nm)
    plateau=fixture.input.receipt.completed_observed_us;
   assert(fabsf(fixture.input.receipt.decoded_torque_nm)<=0.20F);
  }
  assert(fixture.pilot.status.outcome==SHAFT_END_TIME && plateau && fixture.input.now_us-plateau>=80000U);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE && fixture.output.action==SHAFT_ACTION_DISABLE);
 }
 ++completed;
}

/** @brief Legal stalled decoded receipts never fabricate plateau TIME; absolute 800ms revokes low trial. */
static void test_low_unreached_plateau(void)
{
 unsigned count; low_ramp(1,4000U);
 for(count=0U;count<200U;++count) {
  advance(4000U); fixture.input.receipt.decoded_torque_nm=fixture.output.previous_sent_nm;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  if(count<199U) assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
 }
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
 assert(!fixture.pilot.status.caller_next_transaction_ready);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 ++completed;
}

/** @brief Synthetic .04 rad/s displacements cross the baseline+6q causal 8..12ms gate in both signs. */
static void test_low_displacement_abort(void)
{
 unsigned period_us, count; int direction;
 /* Reproduce the recorded seven-code boundary with synthetic timing/receipts.
  * Baseline came from one native code; the eighth code must still abort. */
 low_ramp(1,4000U);
 fixture.pilot.config.baseline.span64_shaft_rad=1.98646449e-5F;
 fixture.pilot.origin_shaft_rad=0.153924942F/fixture.config.gear_ratio;
 for(count=0U;count<SHAFT_PILOT_HISTORY_CAPACITY;++count)
  fixture.pilot.history[count].shaft_position_rad=fixture.pilot.origin_shaft_rad;
 fixture.pilot.previous_feedback.native_position_rad=0.153924942F;
 fixture.input.feedback.native_position_rad=0.156596184F;
 low_tick(4000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
 fixture.input.feedback.native_position_rad=0.156596184F+fixture.pilot.status.quantum_native_rad;
 low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
 for(direction=-1;direction<=1;direction+=2) for(period_us=3000U;period_us<=5000U;period_us+=1000U) {
  uint64_t first_changed_rx; low_ramp(direction,period_us); first_changed_rx=fixture.input.now_us;
  for(count=0U;count<5U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
   fixture.input.feedback.native_position_rad+=(float)direction*0.04F*(float)period_us*1.0e-6F*fixture.config.gear_ratio;
   low_tick(period_us);
  }
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert(fixture.input.now_us-first_changed_rx<=12000U);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 }
 ++completed;
}

/** @brief Position alternation within the captured synthetic span cannot falsely revoke the baseline. */
static void test_low_baseline_no_abort(void)
{
 unsigned count; float native_quantum; low_ramp(1,4000U);
 native_quantum=fixture.pilot.status.quantum_native_rad;
 for(count=0U;count<12U;++count) {
  fixture.input.feedback.native_position_rad=count%2U?native_quantum:-native_quantum;
  low_tick(4000U); assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 }
 ++completed;
}

/** @brief Early raw threshold is strict and bidirectional; the original .3 speed protection is unchanged. */
static void test_low_raw_abort(void)
{
 int direction; low_ramp(1,4000U);
 fixture.input.feedback.raw_velocity_rad_s=0.15F; low_tick(4000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
 for(direction=-1;direction<=1;direction+=2) {
  low_ramp(direction,4000U); fixture.input.feedback.raw_velocity_rad_s=(float)direction*0.15001F;
  low_tick(4000U); assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 }
 ++completed;
}

/** @brief Fault, stale data, period, binding and guard outrank ABORT and preserve the final send evidence. */
static void test_low_protection_priority(void)
{
 unsigned scenario;
 const ShaftPilotFault expected_faults[9]={SHAFT_FAULT_SPEED,SHAFT_FAULT_POSITION,
  SHAFT_FAULT_DRIVER,SHAFT_FAULT_TEMPERATURE,SHAFT_FAULT_FEEDBACK,SHAFT_FAULT_BINDING,
  SHAFT_FAULT_GUARD,SHAFT_FAULT_PERIOD,SHAFT_FAULT_RECEIPT};
 for(scenario=0U;scenario<9U;++scenario) {
  low_ramp(1,4000U); low_tick(4000U); advance(scenario==7U?5001U:4000U);
  fixture.input.feedback.raw_velocity_rad_s=0.151F;
  if(scenario==0U) fixture.input.feedback.raw_velocity_rad_s=0.301F;
  if(scenario==1U) fixture.input.feedback.native_position_rad=0.005F*fixture.config.gear_ratio;
  if(scenario==2U) fixture.input.feedback.driver_fault=1U;
  if(scenario==3U) fixture.input.feedback.mos_temperature_c=50.0F;
  if(scenario==4U) fixture.input.feedback.rx_us=fixture.input.now_us-8001U;
  if(scenario==5U) fixture.input.binding_id++;
  if(scenario==6U) fixture.input.guard_live=0U;
  if(scenario==8U) fixture.input.receipt.command_id++;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.pilot.status.outcome==SHAFT_END_FAULT);
  assert(fixture.pilot.status.first_fault==expected_faults[scenario]);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  if(scenario!=8U) assert(fixture.pilot.status.last_successful_decoded_torque_nm==fixture.input.receipt.decoded_torque_nm);
 }
 ++completed;
}

/** @brief Malformed/late/mismatched receipts cannot start or finish a purported low plateau. */
static void test_low_receipt_integrity(void)
{
 unsigned scenario, count;
 for(scenario=0U;scenario<9U;++scenario) {
  low_ramp(1,4000U);
  for(count=0U;count<30U && fabsf(fixture.pilot.last_sent_nm)<0.3F;++count) low_tick(4000U);
  advance(4000U);
  if(scenario==0U) fixture.input.receipt.valid=0U;
  if(scenario==1U) fixture.input.receipt.command_id++;
  if(scenario==2U) fixture.input.receipt.success=0U;
  if(scenario==3U) fixture.input.receipt.decoded_torque_nm=NAN;
  if(scenario==4U) fixture.input.receipt.decoded_torque_nm=0.351F;
  if(scenario==5U) fixture.input.receipt.decoded_torque_nm=0.1F;
  if(scenario==6U) fixture.input.receipt.completed_observed_us=fixture.input.now_us+1U;
  if(scenario==7U) fixture.input.receipt.completed_observed_us=fixture.output.issued_us-1U;
  if(scenario==8U) fixture.input.receipt.kind=SHAFT_ACTION_ENABLE;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_RECEIPT);
  assert(fixture.pilot.status.outcome==SHAFT_END_FAULT);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 }
 ++completed;
}

/** @brief Exercise healthy-platform deadline with original guards and feedback faults on that exact tick. */
static void test_low_plateau_deadline_priority(void)
{
 unsigned scenario, count;
 const ShaftPilotFault expected_faults[8]={SHAFT_FAULT_DRIVER,SHAFT_FAULT_GUARD,
  SHAFT_FAULT_BINDING,SHAFT_FAULT_FEEDBACK,SHAFT_FAULT_RECEIPT,SHAFT_FAULT_SPEED,
  SHAFT_FAULT_PERIOD,SHAFT_FAULT_ACTIVE_TIMEOUT};
 for(scenario=0U;scenario<10U;++scenario) {
  uint64_t plateau_observed_us=0U; low_ramp(1,4000U);
  for(count=0U;count<100U;++count) {
   low_tick(4000U);
   if(!plateau_observed_us && 0.35F-fixture.input.receipt.decoded_torque_nm<fixture.pilot.status.quantum_torque_nm)
    plateau_observed_us=fixture.input.receipt.completed_observed_us;
   if(plateau_observed_us && fixture.input.now_us-plateau_observed_us>=77000U) break;
  }
  assert(plateau_observed_us && fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
  advance(scenario==6U?5001U:4000U);
  assert(fixture.input.now_us-plateau_observed_us>=80000U);
  if(scenario==0U) fixture.input.feedback.torque_nm=fixture.pilot.status.running_torque_gate_nm+0.001F;
  if(scenario==1U) fixture.input.guard_armed=0U;
  if(scenario==2U) fixture.input.session_id++;
  if(scenario==3U) fixture.input.feedback.rx_us=fixture.input.now_us-8001U;
  if(scenario==4U) fixture.input.receipt.command_id++;
  if(scenario==5U) fixture.input.feedback.raw_velocity_rad_s=0.301F;
  if(scenario==7U) fixture.pilot.started_us=fixture.input.now_us-1400000U; /* Absolute-clock injection only. */
  if(scenario==8U) fixture.input.stop_requested=1U;
  if(scenario==9U) fixture.input.feedback.raw_velocity_rad_s=0.151F;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  if(scenario<8U) {
   assert(fixture.pilot.status.outcome==SHAFT_END_FAULT);
   assert(fixture.pilot.status.first_fault==expected_faults[scenario]);
  } else {
   assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
   assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  }
  assert(!fixture.pilot.status.caller_next_transaction_ready);
 }
 ++completed;
}

/** @brief Platform near the absolute pulse end cannot borrow an additional 80ms observation lease. */
static void test_low_late_plateau_and_ready_guard(void)
{
 unsigned count; uint64_t plateau_observed_us=0U; low_ramp(1,4000U);
 for(count=0U;count<178U;++count) {
  advance(4000U); fixture.input.receipt.decoded_torque_nm=fixture.output.previous_sent_nm;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 for(count=0U;count<22U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
  low_tick(4000U);
  if(!plateau_observed_us && 0.35F-fixture.input.receipt.decoded_torque_nm<fixture.pilot.status.quantum_torque_nm)
   plateau_observed_us=fixture.input.receipt.completed_observed_us;
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
 assert(fixture.input.now_us-fixture.pilot.phase_us==800000U);
 assert(!fixture.pilot.status.caller_next_transaction_ready);
 /* Isolate low ready gating under deliberately corrupted endpoint state; never a real transaction. */
 fixture.pilot.status.outcome=SHAFT_END_TIME;
 fixture.input.guard_fired=1U; fixture.input.guard_acked=1U;
 for(count=0U;count<30U && !fixture.pilot.status.safe_release;++count) low_tick(4000U);
 assert(fixture.pilot.status.safe_release && !fixture.pilot.status.caller_next_transaction_ready);
 ++completed;
}

/** @brief Healthy TIME retains the old actual-guard-ack ready gate; ABORT never opens it. */
static void test_low_stop_readiness(void)
{
 unsigned count, scenario;
 for(scenario=0U;scenario<2U;++scenario) {
  low_ramp(1,4000U);
  if(scenario==1U) fixture.input.feedback.raw_velocity_rad_s=0.151F;
  for(count=0U;count<100U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) low_tick(4000U);
  assert(fixture.pilot.status.outcome==(scenario?SHAFT_END_ABORT:SHAFT_END_TIME));
  fixture.input.feedback.raw_velocity_rad_s=fixture.config.baseline.raw_zero_center_rad_s;
  for(count=0U;count<30U && !fixture.pilot.status.safe_release;++count) low_tick(4000U);
  assert(fixture.pilot.status.safe_release && !fixture.pilot.status.caller_next_transaction_ready);
  fixture.input.guard_fired=1U; fixture.input.guard_acked=1U; low_tick(4000U);
  assert(fixture.pilot.status.caller_next_transaction_ready==(scenario?0U:1U));
 }
 ++completed;
}

/** @brief Stop and guard paths consume the last matching real-shape receipt in either low direction. */
static void test_low_stop_last_receipt(void)
{
 unsigned scenario; int direction;
 for(direction=-1;direction<=1;direction+=2) for(scenario=0U;scenario<2U;++scenario) {
  ShaftPilotTrace record; low_ramp(direction,4000U); low_tick(4000U); advance(4000U);
  if(scenario==0U) fixture.input.stop_requested=1U; else fixture.input.guard_live=0U;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.pilot.status.last_successful_decoded_torque_nm==fixture.input.receipt.decoded_torque_nm);
  assert(fixture.pilot.status.peak_successful_decoded_torque_nm>=fabsf(fixture.input.receipt.decoded_torque_nm));
  assert(shaft_pilot_trace_get(&fixture.pilot,fixture.pilot.status.trace_count-1U,&record));
  assert(record.input.receipt.kind==SHAFT_ACTION_TORQUE && record.output.action==SHAFT_ACTION_DISABLE);
  assert(!fixture.pilot.status.caller_next_transaction_ready);
 }
 ++completed;
}

/** @brief Low mode never labels .1rad/s as TARGET; exaggerated synthetic baseline isolates that endpoint. */
static void test_low_no_target_and_default_reinit(void)
{
 unsigned count; prepare(1); fixture.config.baseline.span64_shaft_rad=0.01F;
 assert(initialize_low(1)); assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 for(count=0U;count<20U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(4000U);
 for(count=0U;count<100U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
  fixture.input.feedback.native_position_rad+=0.12F*0.004F*fixture.config.gear_ratio; low_tick(4000U);
  assert(fixture.pilot.status.outcome!=SHAFT_END_TARGET);
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_TIME);
 /* Reinitialize previously low storage with the original trusted API: default 800ms and cap restored. */
 assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 fixture.input.feedback.native_position_rad=0.0F;
 fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 for(count=0U;count<20U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) tick(4000U);
 for(count=0U;count<199U;++count) tick(4000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP && fixture.output.cap_nm==1.75F);
 tick(4000U); assert(fixture.pilot.status.outcome==SHAFT_END_TIME);
 ++completed;
}

/** @brief Low stale stop data rebuilds dwell; absolute 1.5s lock and ongoing disable remain unmodified. */
static void test_low_stop_gap_and_lock(void)
{
 unsigned count; low_ramp(1,4000U); fixture.input.stop_requested=1U; low_tick(4000U);
 fixture.input.stop_requested=0U;
 for(count=0U;count<15U;++count) low_tick(4000U);
 low_tick(12000U); assert(!fixture.pilot.status.safe_release);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_FEEDBACK);
 for(count=0U;count<30U && !fixture.pilot.status.safe_release;++count) low_tick(4000U);
 assert(fixture.pilot.status.safe_release && !fixture.pilot.status.caller_next_transaction_ready);
 low_ramp(1,4000U); fixture.input.stop_requested=1U; low_tick(4000U); fixture.input.stop_requested=0U;
 for(count=0U;count<400U;++count) {
  advance(4000U); fixture.input.feedback.driver_state=SHAFT_DRIVER_ENABLED;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(fixture.pilot.status.phase==SHAFT_PHASE_LOCKED);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_STOP_TIMEOUT);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE && !fixture.pilot.status.caller_next_transaction_ready);
 ++completed;
}

/** @brief Start the visible profile with synthetic readback-scale data and no physical actuator. */
static void visible_ramp(int direction)
{
 unsigned count; prepare(direction);
 fixture.config.gear_ratio=19.2031994F; fixture.config.position_max_native_rad=12.5F;
 fixture.config.baseline.span64_shaft_rad=2.0F*(25.0F/65535.0F/fixture.config.gear_ratio);
 fixture.input.now_us=2000000U; fixture.input.feedback.rx_us=2000000U;
 fixture.input.feedback.observed_us=2000000U;
 assert(shaft_pilot_init_visible(&fixture.pilot,&fixture.config,direction));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 for(count=0U;count<24U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(4000U);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
}
/** @brief Visible entry cannot override frozen bounds, directions or readback health. */
static void test_visible_config(void)
{
 prepare(1); assert(shaft_pilot_init_visible(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.plan.cap_nm==0.5F && fixture.pilot.plan.slew_nm_s==2.0F && fixture.pilot.plan.step_nm==0.008F);
 assert(!shaft_pilot_init_visible(&fixture.pilot,&fixture.config,0));
 fixture.config.torque_max_nm=0.49F; assert(!shaft_pilot_init_visible(&fixture.pilot,&fixture.config,1));
 assert(!shaft_pilot_init_visible(NULL,&fixture.config,1));
 assert(!shaft_pilot_init_visible(&fixture.pilot,NULL,1)); ++completed;
}
/** @brief The actual synthetic inward-grid platform lasts at most 200ms, never the requested-cap instant. */
static void test_visible_plateau(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  visible_ramp(direction);
  for(count=0U;count<210U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
   low_tick(4000U);
   assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
   assert(fabsf(fixture.pilot.status.last_successful_decoded_torque_nm)<=0.5F);
   if(fixture.pilot.status.phase==SHAFT_PHASE_RAMP)
    assert(!fixture.pilot.plateau_seen || fixture.input.now_us-fixture.pilot.plateau_observed_us<200000U);
  }
  assert(fixture.pilot.status.outcome==SHAFT_END_TIME && fixture.pilot.plateau_seen);
  assert(fixture.input.now_us-fixture.pilot.plateau_observed_us>=200000U);
  assert(fixture.input.now_us-fixture.pilot.plateau_observed_us<204000U);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief Native signed angle target stops both directions without waiting to attain a platform. */
static void test_visible_angle(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  visible_ramp(direction);
  for(count=0U;count<45U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
   fixture.input.feedback.native_position_rad+=(float)direction*0.004F; low_tick(4000U);
  }
  assert(fixture.pilot.status.outcome==SHAFT_END_TARGET && fixture.output.action==SHAFT_ACTION_DISABLE);
  assert((float)direction*fixture.input.feedback.native_position_rad>=0.15F);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 }
 visible_ramp(1);
 for(count=0U;count<45U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
  fixture.input.feedback.native_position_rad-=0.004F; low_tick(4000U);
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.output.action==SHAFT_ACTION_DISABLE);
 ++completed;
}
/** @brief The new short-window envelope admits observed .04rad/s that remains too large for micro-motion low. */
static void test_visible_observation_envelope(void)
{
 unsigned count; visible_ramp(1);
 for(count=0U;count<6U;++count) {
  fixture.input.feedback.native_position_rad+=0.04F*0.004F*fixture.config.gear_ratio; low_tick(4000U);
  assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
 } ++completed;
}
/** @brief Fast short-window displacement aborts before 32ms and raw magnitude has its own stricter early exit. */
static void test_visible_early_abort(void)
{
 unsigned count; visible_ramp(1);
 for(count=0U;count<5U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
  fixture.input.feedback.native_position_rad+=0.2F*0.004F*fixture.config.gear_ratio; low_tick(4000U);
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 visible_ramp(1); fixture.input.feedback.raw_velocity_rad_s=0.151F; low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT); ++completed;
}
/** @brief Absolute ramp deadline and malformed actual receipt cannot fabricate TIME or eligibility. */
static void test_visible_deadline_receipt(void)
{
 unsigned count; visible_ramp(1);
 for(count=0U;count<200U;++count) {
  advance(4000U); fixture.input.receipt.decoded_torque_nm=fixture.output.previous_sent_nm;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && !fixture.pilot.status.caller_next_transaction_ready);
 visible_ramp(1); advance(4000U); fixture.input.receipt.decoded_torque_nm=0.51F;
 shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_RECEIPT);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE); ++completed;
}
/** @brief Original speed, driver, period, position and stop-request priorities remain unchanged. */
static void test_visible_original_protection(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<5U;++scenario) {
  visible_ramp(1); advance(scenario==2U?6000U:4000U);
  if(scenario==0U) fixture.input.feedback.raw_velocity_rad_s=0.301F;
  if(scenario==1U) fixture.input.feedback.driver_fault=1U;
  if(scenario==3U) fixture.input.feedback.native_position_rad+=0.1F;
  if(scenario==4U) fixture.input.stop_requested=1U;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  if(scenario==0U) assert(fixture.pilot.status.first_fault==SHAFT_FAULT_SPEED);
  if(scenario==1U) assert(fixture.pilot.status.first_fault==SHAFT_FAULT_DRIVER);
  if(scenario==2U) assert(fixture.pilot.status.first_fault==SHAFT_FAULT_PERIOD);
  if(scenario==3U) assert(fixture.pilot.status.first_fault==SHAFT_FAULT_POSITION);
  if(scenario==4U) assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
 } ++completed;
}
/** @brief Angle-ended release needs genuine disabled dwell/guard ACK; reinit default does not inherit visible semantics. */
static void test_visible_stop_and_reinit(void)
{
 unsigned count; visible_ramp(1);
 for(count=0U;count<45U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
  fixture.input.feedback.native_position_rad+=0.004F; low_tick(4000U);
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_TARGET);
 fixture.input.guard_fired=1U; fixture.input.guard_acked=1U; fixture.input.guard_live=0U;
 for(count=0U;count<60U && fixture.pilot.status.phase!=SHAFT_PHASE_DONE;++count) low_tick(4000U);
 assert(fixture.pilot.status.safe_release && fixture.pilot.status.caller_next_transaction_ready);
 assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 prepare(1); assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 assert(fixture.pilot.plan.cap_nm==1.75F); ++completed;
}
/** @brief Build an observe-compatible synthetic discovery without changing historical fixture defaults. */
static void observe_prepare(int direction)
{
 prepare(direction); fixture.config.gear_ratio=19.2031994F;
 fixture.config.position_max_native_rad=12.5F;
 fixture.config.baseline.span64_shaft_rad=2.0F*(25.0F/65535.0F/fixture.config.gear_ratio);
 fixture.input.now_us=2000000U; fixture.input.feedback.rx_us=2000000U;
 fixture.input.feedback.observed_us=2000000U;
}
/** @brief Enter the new fixed observation at a known native origin and allowed scheduler interval. */
static void observe_ramp(int direction, unsigned period_us, float origin_native_rad)
{
 unsigned count; observe_prepare(direction);
 fixture.input.feedback.native_position_rad=origin_native_rad;
 assert(shaft_pilot_init_observe(&fixture.pilot,&fixture.config,direction));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 for(count=0U;count<26U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(period_us);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
}
/** @brief Only the exact VMAX200/PMAX12.5/TMAX10 profile can select the frozen rated-half plan. */
static void test_observe_config(void)
{
 unsigned scenario; observe_prepare(1);
 assert(shaft_pilot_init_observe(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.plan.cap_nm==1.75F && fixture.pilot.plan.slew_nm_s==4.0F &&
  fixture.pilot.plan.step_nm==0.016F);
 for(scenario=0U;scenario<5U;++scenario) {
  observe_prepare(1);
  if(scenario==0U) fixture.config.velocity_max_rad_s=5.0F;
  if(scenario==1U) fixture.config.position_max_native_rad=12.6F;
  if(scenario==2U) fixture.config.torque_max_nm=11.0F;
  if(scenario==3U) fixture.config.gear_ratio=NAN;
  assert(!shaft_pilot_init_observe(&fixture.pilot,&fixture.config,scenario==4U?0:1));
 }
 assert(!shaft_pilot_init_observe(NULL,&fixture.config,1));
 assert(!shaft_pilot_init_observe(&fixture.pilot,NULL,1)); ++completed;
}
/** @brief A real-time-scale synthetic response may cross former .1/.3 gates without ending observe. */
static void test_observe_no_old_speed_endpoint(void)
{
 unsigned count; observe_ramp(1,4000U,0.0F);
 fixture.input.feedback.raw_velocity_rad_s=0.338217258F;
 for(count=0U;count<10U;++count) {
  fixture.input.feedback.native_position_rad+=0.5F*fixture.config.gear_ratio*0.004F;
  low_tick(4000U);
  assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 }
 assert(fixture.pilot.status.velocity_ready && fixture.pilot.status.velocity_rx32_rad_s>0.3F);
 ++completed;
}
/** @brief The requested rated-half raw-speed boundary uses strict comparison in both directions. */
static void test_observe_raw_speed_boundary(void)
{
 int direction; const float limit=20.6821516F;
 for(direction=-1;direction<=1;direction+=2) {
  observe_ramp(direction,4000U,0.0F);
  fixture.input.feedback.raw_velocity_rad_s=(float)direction*limit; low_tick(4000U);
  assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
  fixture.input.feedback.raw_velocity_rad_s=(float)direction*nextafterf(limit,INFINITY); low_tick(4000U);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_SPEED && fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief Both signed native-angle targets are relative to the captured origin, never output degrees. */
static void test_observe_angle_endpoint(void)
{
 int direction; unsigned count; const float origin=4.0F;
 for(direction=-1;direction<=1;direction+=2) {
  observe_ramp(direction,4000U,origin);
  for(count=0U;count<140U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
   fixture.input.feedback.native_position_rad+=(float)direction*0.004F; low_tick(4000U);
  }
  assert(fixture.pilot.status.outcome==SHAFT_END_TARGET && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert((float)direction*(fixture.input.feedback.native_position_rad-origin)>=0.523598776F);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief Wrong-way motion exits at five native degrees without a compensating torque reversal. */
static void test_observe_reverse_endpoint(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  observe_ramp(direction,4000U,0.0F);
  for(count=0U;count<30U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
   fixture.input.feedback.native_position_rad-=(float)direction*0.004F; low_tick(4000U);
  }
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief Sampling may overshoot 30 degrees; invalid single-sample displacement remains a fault. */
static void test_observe_sampled_angle_and_integrity(void)
{
 observe_ramp(1,4000U,0.0F);
 fixture.input.feedback.native_position_rad=1.55F; low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_TARGET && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 assert(fixture.input.feedback.native_position_rad>0.523598776F);
 observe_ramp(1,4000U,0.0F);
 /* Exceed even the arrival interval's two-millisecond clock uncertainty, not just the travel budget. */
 fixture.input.feedback.native_position_rad=3.0F; low_tick(4000U);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_POSITION && fixture.output.action==SHAFT_ACTION_DISABLE);
 ++completed;
}
/** @brief A legal eight-millisecond jump across ninety degrees faults before an angle completion. */
static void test_observe_native_travel(void)
{
 observe_ramp(1,4000U,0.0F); advance(4000U);
 /* A valid arrival interval may span eight milliseconds while service still occurs each four. */
 fixture.input.feedback.rx_us=fixture.pilot.previous_feedback.rx_us;
 fixture.input.feedback.observed_us=fixture.pilot.previous_feedback.observed_us;
 shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 fixture.input.feedback.native_position_rad=1.6F; low_tick(4000U);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_TRAVEL && fixture.output.action==SHAFT_ACTION_DISABLE);
 ++completed;
}
/** @brief Actual inward-grid plateau plus 200ms fits the new 900ms bound at every permitted interval. */
static void test_observe_plateau_scheduling(void)
{
 int direction; unsigned period_us, count;
 for(direction=-1;direction<=1;direction+=2) for(period_us=3000U;period_us<=5000U;period_us+=1000U) {
  uint64_t ramp_start; observe_ramp(direction,period_us,0.0F); ramp_start=fixture.pilot.phase_us;
  for(count=0U;count<310U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) {
   low_tick(period_us);
   assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
   assert(fabsf(fixture.pilot.status.last_successful_decoded_torque_nm)<=1.75F);
  }
  assert(fixture.pilot.status.outcome==SHAFT_END_TIME && fixture.pilot.plateau_seen);
  assert(fixture.input.now_us-fixture.pilot.plateau_observed_us>=200000U);
  assert(fixture.input.now_us-fixture.pilot.plateau_observed_us<200000U+period_us);
  assert(fixture.input.now_us-ramp_start<900000U && fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief An unachieved platform aborts at 900ms instead of becoming a successful timed observation. */
static void test_observe_unreached_platform_deadline(void)
{
 unsigned count; uint64_t start; observe_ramp(1,4000U,0.0F); start=fixture.pilot.phase_us;
 for(count=0U;count<225U;++count) {
  advance(4000U); fixture.input.receipt.decoded_torque_nm=fixture.output.previous_sent_nm;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(fixture.input.now_us-start==900000U && fixture.pilot.status.outcome==SHAFT_END_ABORT);
 assert(!fixture.pilot.plateau_seen && !fixture.pilot.status.caller_next_transaction_ready);
 assert(fixture.output.action==SHAFT_ACTION_DISABLE); ++completed;
}
/** @brief Binding, temperature, receipt, guard, period and external STOP protections remain effective. */
static void test_observe_other_protection(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<7U;++scenario) {
  observe_ramp(1,4000U,0.0F); advance(scenario==0U?6000U:4000U);
  if(scenario==1U) fixture.input.feedback.driver_fault=1U;
  if(scenario==2U) fixture.input.feedback.mos_temperature_c=50.0F;
  if(scenario==3U) fixture.input.receipt.success=0U;
  if(scenario==4U) fixture.input.guard_live=0U;
  if(scenario==5U) fixture.input.stop_requested=1U;
  if(scenario==6U) ++fixture.input.binding_id;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE && !fixture.pilot.status.caller_next_transaction_ready);
  if(scenario==5U) assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
  else assert(fixture.pilot.status.first_fault!=SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief STOP continues recording, uses the 180-degree budget and cannot leak observe into default. */
static void test_observe_stop_and_reinit(void)
{
 unsigned count; observe_ramp(1,4000U,0.0F);
 fixture.input.feedback.native_position_rad=0.524F; low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_TARGET);
 fixture.input.guard_fired=1U; fixture.input.guard_acked=1U; fixture.input.guard_live=0U;
 fixture.input.feedback.native_position_rad=2.0F; low_tick(4000U);
 assert(!fixture.pilot.status.safe_release && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 for(count=0U;count<80U && fixture.pilot.status.phase!=SHAFT_PHASE_DONE;++count) low_tick(4000U);
 assert(fixture.pilot.status.safe_release && fixture.pilot.status.caller_next_transaction_ready);
 assert(fixture.pilot.status.trace_count>40U);
 reach_ramp(1); fixture.input.feedback.raw_velocity_rad_s=0.338F; tick(4000U);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_SPEED);
 ++completed;
}
/** @brief The entry reserves the full native stopped budget, rejecting position wrap headroom loss. */
static void test_observe_entry_headroom(void)
{
 observe_prepare(1); fixture.input.feedback.native_position_rad=8.0F;
 assert(shaft_pilot_init_observe(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 observe_prepare(1); fixture.input.feedback.native_position_rad=10.0F;
 assert(shaft_pilot_init_observe(&fixture.pilot,&fixture.config,1));
 assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)); ++completed;
}
/** @brief Enter the platform variant with the same synthetic discovery and true receipt grid. */
static void platform_ramp(int direction, unsigned period_us, float origin)
{
 unsigned count; observe_prepare(direction); fixture.input.feedback.native_position_rad=origin;
 assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,direction));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 for(count=0U;count<26U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(period_us);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
}
/** @brief Platform accepts only the frozen ranges and cannot mutate old initialization. */
static void test_platform_config(void)
{
 observe_prepare(1); assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.plan.cap_nm==1.75F && fixture.pilot.plan.slew_nm_s==4.0F);
 fixture.config.velocity_max_rad_s=5.0F;
 assert(!shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 observe_prepare(1); fixture.config.position_max_native_rad=25.0F;
 assert(!shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 observe_prepare(1); fixture.config.torque_max_nm=20.0F;
 assert(!shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 observe_prepare(1); assert(!shaft_pilot_init_platform(&fixture.pilot,&fixture.config,0));
 ++completed;
}
/** @brief Both directions may cross the old 30/90-degree relative endpoints without completion. */
static void test_platform_cross_old_angles(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  platform_ramp(direction,4000U,0.0F);
  for(count=0U;count<100U;++count) {
   fixture.input.feedback.native_position_rad+=(float)direction*0.02F; low_tick(4000U);
   assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  }
  assert(fabsf(fixture.input.feedback.native_position_rad)>1.57079633F);
 } ++completed;
}
/** @brief Actual inward platform receipts and 200ms complete at each legal scheduler interval. */
static void test_platform_true_plateau(void)
{
 int direction; unsigned period, count;
 for(direction=-1;direction<=1;direction+=2) for(period=3000U;period<=5000U;period+=1000U) {
  platform_ramp(direction,period,0.0F);
  for(count=0U;count<310U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) low_tick(period);
  assert(fixture.pilot.plateau_seen && fixture.pilot.status.outcome==SHAFT_END_TIME);
  assert(fixture.input.now_us-fixture.pilot.plateau_observed_us>=200000U);
  assert(fixture.input.now_us-fixture.pilot.phase_us<900000U);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief Current velocity enlarges the minimum reserve and can terminate before the static edge. */
static void test_platform_dynamic_reserve(void)
{
 int direction;
 for(direction=-1;direction<=1;direction+=2) {
  platform_ramp(direction,4000U,(float)direction*9.0F); low_tick(4000U);
  assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
  fixture.input.feedback.raw_velocity_rad_s=(float)direction*1.0F; low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE && !fixture.pilot.status.caller_next_transaction_ready);
 } ++completed;
}
/** @brief The static range reserve and sample quantization are also effective at near-zero speed. */
static void test_platform_static_reserve(void)
{
 int direction;
 for(direction=-1;direction<=1;direction+=2) {
  platform_ramp(direction,4000U,(float)direction*9.0F);
  fixture.input.feedback.native_position_rad=(float)direction*9.4F; low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief Entry rejects a position allowed by the static reserve but lacking velocity lookahead. */
static void test_platform_entry_reserve(void)
{
 int direction;
 for(direction=-1;direction<=1;direction+=2) {
  observe_prepare(direction); fixture.input.feedback.native_position_rad=(float)direction*9.3F;
  assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,direction));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  observe_prepare(direction); fixture.input.feedback.native_position_rad=(float)direction*9.35F;
  assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,direction));
  assert(!shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 } ++completed;
}
/** @brief Range reserve applies to ENABLE and NEUTRAL as well as the nonzero ramp. */
static void test_platform_early_phase_reserve(void)
{
 unsigned neutral; int direction;
 for(direction=-1;direction<=1;direction+=2) for(neutral=0U;neutral<2U;++neutral) {
  observe_prepare(direction); fixture.input.feedback.native_position_rad=(float)direction*9.3F;
  assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,direction));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  if(neutral) { low_tick(4000U); assert(fixture.pilot.status.phase==SHAFT_PHASE_NEUTRAL); }
  fixture.input.feedback.native_position_rad=(float)direction*9.4F; low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.output.action==SHAFT_ACTION_DISABLE);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief Simultaneous range exhaustion must not hide driver loss or an early-phase deadline fault. */
static void test_platform_phase_fault_priority(void)
{
 unsigned count; observe_prepare(1); fixture.input.feedback.native_position_rad=9.3F;
 assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)); low_tick(4000U);
 advance(4000U); fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
 fixture.input.feedback.native_position_rad=9.4F;
 shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_DRIVER);
 observe_prepare(1); fixture.input.feedback.native_position_rad=9.3F;
 assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 for(count=0U;count<16U;++count) {
  advance(4000U); fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
  if(count==15U) fixture.input.feedback.native_position_rad=9.4F;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_ENABLE_TIMEOUT);
 observe_prepare(1); fixture.input.feedback.native_position_rad=9.3F;
 assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)); low_tick(3000U);
 fixture.input.feedback.raw_velocity_rad_s=0.049F;
 for(count=0U;count<23U;++count) {
  if(count==22U) fixture.input.feedback.native_position_rad=9.4F;
  low_tick(3000U);
 }
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NEUTRAL_TIMEOUT);
 ++completed;
}
/** @brief The absolute protocol edge remains checked after STOP rather than assuming disabled motion ceases. */
static void test_platform_stop_range(void)
{
 platform_ramp(1,4000U,9.0F); fixture.input.stop_requested=1U;
 fixture.input.feedback.native_position_rad=10.5F; low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
 fixture.input.feedback.native_position_rad=12.4999F; low_tick(4000U);
 assert(fixture.pilot.status.first_fault==SHAFT_FAULT_TRAVEL && fixture.output.action==SHAFT_ACTION_DISABLE);
 ++completed;
}
/** @brief Reverse direction and unreached plateau cannot become a successful observation. */
static void test_platform_reverse_and_deadline(void)
{
 unsigned count; platform_ramp(1,4000U,0.0F);
 fixture.input.feedback.native_position_rad=-0.09F; low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
 platform_ramp(1,4000U,0.0F);
 for(count=0U;count<225U;++count) {
  advance(4000U); fixture.input.receipt.decoded_torque_nm=fixture.output.previous_sent_nm;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && !fixture.pilot.plateau_seen);
 ++completed;
}
/** @brief Guard, feedback, speed, temperature, receipt and explicit STOP still outrank platform completion. */
static void test_platform_protections(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<7U;++scenario) {
  platform_ramp(1,4000U,0.0F); advance(scenario==0U?6000U:4000U);
  if(scenario==1U) fixture.input.feedback.driver_fault=1U;
  if(scenario==2U) fixture.input.feedback.mos_temperature_c=50.0F;
  if(scenario==3U) fixture.input.receipt.success=0U;
  if(scenario==4U) fixture.input.guard_live=0U;
  if(scenario==5U) fixture.input.stop_requested=1U;
  if(scenario==6U) fixture.input.feedback.raw_velocity_rad_s=20.7F;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  if(scenario==5U) assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
  else assert(fixture.pilot.status.first_fault!=SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief Only a completed plateau plus independent healthy stopped evidence grants next-transaction readiness. */
static void test_platform_stop_readiness_and_reinit(void)
{
 unsigned count; platform_ramp(1,4000U,0.0F);
 for(count=0U;count<240U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_TIME);
 fixture.input.guard_fired=1U; fixture.input.guard_acked=1U; fixture.input.guard_live=0U;
 for(count=0U;count<80U && fixture.pilot.status.phase!=SHAFT_PHASE_DONE;++count) low_tick(4000U);
 assert(fixture.pilot.status.safe_release && fixture.pilot.status.caller_next_transaction_ready);
 assert(shaft_pilot_init_observe(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.observe_profile && !fixture.pilot.platform_profile);
 assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.platform_profile);
 assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 assert(!fixture.pilot.observe_profile && !fixture.pilot.platform_profile);
 observe_ramp(1,4000U,0.0F); fixture.input.feedback.native_position_rad=0.524F; low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_TARGET);
 ++completed;
}
/** @brief 构造512原生rad冻结量程；基线位置一码随真实新量程同步。 */
static void platform_wide_prepare(int direction)
{
 observe_prepare(direction); fixture.config.position_max_native_rad=512.0F;
 fixture.config.baseline.span64_shaft_rad=2.0F*(1024.0F/65535.0F/fixture.config.gear_ratio);
}
/** @brief 在指定合法周期进入宽量程RAMP，全部回执仍是测试网格。 */
static void platform_wide_ramp(int direction, unsigned period_us)
{
 unsigned count; platform_wide_prepare(direction);
 assert(shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,direction));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 for(count=0U;count<26U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(period_us);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
}
/** @brief 仅测试独立边界保护：替换历史，绝不把合成位置跳变当作连续运动证明。 */
static void platform_wide_boundary(float native_position)
{
 fixture.input.feedback.native_position_rad=native_position;
 fixture.pilot.history_count=0U; fixture.pilot.history_next=0U; fixture.pilot.feedback_seen=0U;
}
/** @brief 新API仅接受512/200/10和合法方向，旧observe/platform仍拒绝512。 */
static void test_platform_wide_config(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<8U;++scenario) {
  platform_wide_prepare(1);
  if(scenario==0U) fixture.config.position_max_native_rad=12.5F;
  if(scenario==1U) fixture.config.position_max_native_rad=511.0F;
  if(scenario==2U) fixture.config.velocity_max_rad_s=5.0F;
  if(scenario==3U) fixture.config.torque_max_nm=20.0F;
  if(scenario==4U) fixture.config.position_max_native_rad=NAN;
  if(scenario==5U) fixture.config.velocity_max_rad_s=INFINITY;
  if(scenario==6U) fixture.config.baseline.point_count=63U;
  assert(!shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,scenario==7U?0:1));
  assert(!fixture.pilot.initialized && !fixture.pilot.observe_profile && !fixture.pilot.platform_profile &&
   !fixture.pilot.platform_wide_profile);
 }
 platform_wide_prepare(1); assert(!shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(!shaft_pilot_init_observe(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.observe_profile && fixture.pilot.platform_profile && fixture.pilot.platform_wide_profile);
 assert(fixture.pilot.plan.cap_nm==1.75F && fixture.pilot.plan.slew_nm_s==4.0F && fixture.pilot.plan.step_nm==0.016F);
 ++completed;
}
/** @brief 宽量程量化来自1024/65535；速度、力矩和停止门仍使用原冻结范围。 */
static void test_platform_wide_quantum(void)
{
 platform_wide_prepare(1); assert(shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,1));
 assert(fabsf(fixture.pilot.status.quantum_native_rad-1024.0F/65535.0F)<1.0e-8F);
 assert(fabsf(fixture.pilot.status.quantum_shaft_rad-1024.0F/65535.0F/fixture.config.gear_ratio)<1.0e-9F);
 assert(fixture.pilot.status.quantum_velocity_rad_s==400.0F/4095.0F);
 assert(fixture.pilot.status.quantum_torque_nm==20.0F/4095.0F); ++completed;
}
/** @brief 两方向入口精确使用1058ms预算；该预算已含一次25ms且不重复计入。 */
static void test_platform_wide_entry_budget(void)
{
 int direction; unsigned allowed;
 for(direction=-1;direction<=1;direction+=2) for(allowed=0U;allowed<2U;++allowed) {
  float required, boundary; platform_wide_prepare(direction);
  assert(shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,direction));
  required=12.5663706F+2.0F*fixture.pilot.status.quantum_native_rad+
   20.6821516F*fixture.config.gear_ratio*1.058F;
  boundary=512.0F-required;
  fixture.input.feedback.native_position_rad=(float)direction*(boundary+(allowed?-0.001F:0.001F));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)==(int)allowed);
 }
 ++completed;
}
/** @brief 本方向预算充足也不能绕过反方向最低4pi加两码余量。 */
static void test_platform_wide_opposite_reserve(void)
{
 int direction; unsigned allowed;
 for(direction=-1;direction<=1;direction+=2) for(allowed=0U;allowed<2U;++allowed) {
  float minimum; platform_wide_prepare(direction);
  assert(shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,direction));
  minimum=12.5663706F+2.0F*fixture.pilot.status.quantum_native_rad;
  fixture.input.feedback.native_position_rad=-(float)direction*(512.0F-minimum+(allowed?-0.01F:0.01F));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)==(int)allowed);
 } ++completed;
}
/** @brief 两方向3/4/5ms真回执平台必须持续至少200ms并早于900ms退出。 */
static void test_platform_wide_plateau(void)
{
 int direction; unsigned period, count;
 for(direction=-1;direction<=1;direction+=2) for(period=3000U;period<=5000U;period+=1000U) {
  platform_wide_ramp(direction,period);
  for(count=0U;count<310U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) low_tick(period);
  assert(fixture.pilot.plateau_seen && fixture.pilot.status.outcome==SHAFT_END_TIME);
  assert(fixture.input.now_us-fixture.pilot.plateau_observed_us>=200000U);
  assert(fixture.input.now_us-fixture.pilot.phase_us<900000U && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE && fixture.pilot.status.peak_successful_decoded_torque_nm<=1.75F);
 } ++completed;
}
/** @brief 宽量程跨旧PMAX和相对角度仍保持RAMP，不能误套旧停止角。 */
static void test_platform_wide_cross_old_angles(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  platform_wide_ramp(direction,4000U);
  for(count=0U;count<100U;++count) {
   fixture.input.feedback.native_position_rad+=(float)direction*0.2F; low_tick(4000U);
   assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  }
  assert(fabsf(fixture.input.feedback.native_position_rad)>12.5F);
 } ++completed;
}
/** @brief 动态reserve同时覆盖raw及可用32ms速度，取绝对值较大者。 */
static void test_platform_wide_dynamic_reserve(void)
{
 int direction; unsigned source;
 for(direction=-1;direction<=1;direction+=2) for(source=0U;source<3U;++source) {
  platform_wide_ramp(direction,4000U); platform_wide_boundary((float)direction*499.0F);
  fixture.input.feedback.raw_velocity_rad_s=source==1U?-(float)direction:0.0F;
  if(source==2U) {
   fixture.pilot.history_count=1U; fixture.pilot.history_next=1U;
   fixture.pilot.history[0].rx_us=fixture.input.now_us+4000U-32000U;
   fixture.pilot.history[0].observed_us=fixture.pilot.history[0].rx_us;
   fixture.pilot.history[0].shaft_position_rad=(float)direction*(499.0F/fixture.config.gear_ratio-0.032F);
  }
  low_tick(4000U);
  if(source) assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.output.action==SHAFT_ACTION_DISABLE);
  else assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief 零速最低4pi加两码仍触发，双向一致。 */
static void test_platform_wide_static_reserve(void)
{
 int direction;
 for(direction=-1;direction<=1;direction+=2) {
  platform_wide_ramp(direction,4000U); platform_wide_boundary((float)direction*499.41F);
  fixture.input.feedback.raw_velocity_rad_s=0.0F; low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief ENABLE和NEUTRAL也执行宽停止余量，非零力矩之前不得跳过。 */
static void test_platform_wide_early_phase_reserve(void)
{
 int direction; unsigned neutral;
 for(direction=-1;direction<=1;direction+=2) for(neutral=0U;neutral<2U;++neutral) {
  platform_wide_prepare(direction); assert(shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,direction));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  if(neutral) low_tick(4000U);
  platform_wide_boundary((float)direction*499.5F); low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.output.action==SHAFT_ACTION_DISABLE);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief 同周期范围不足不能掩盖驱动失能和ENABLE/NEUTRAL期限故障。 */
static void test_platform_wide_phase_priority(void)
{
 unsigned scenario, count;
 for(scenario=0U;scenario<3U;++scenario) {
  platform_wide_prepare(1); assert(shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,1));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  if(scenario!=1U) low_tick(scenario==2U?3000U:4000U);
  if(scenario==1U) for(count=0U;count<15U;++count) {
   advance(4000U); fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
   shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  }
  if(scenario==2U) {
   fixture.input.feedback.raw_velocity_rad_s=0.049F;
   for(count=0U;count<22U;++count) low_tick(3000U);
  }
  platform_wide_boundary(499.5F); advance(scenario==2U?3000U:4000U);
  if(scenario!=2U) fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.pilot.status.first_fault==(scenario==0U?SHAFT_FAULT_DRIVER:
   (scenario==1U?SHAFT_FAULT_ENABLE_TIMEOUT:SHAFT_FAULT_NEUTRAL_TIMEOUT)));
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief STOP之后仍检查新PMAX绝对两码边界以及超过映射范围的POSITION故障。 */
static void test_platform_wide_stop_range(void)
{
 int direction; unsigned outside;
 for(direction=-1;direction<=1;direction+=2) for(outside=0U;outside<2U;++outside) {
  platform_wide_ramp(direction,4000U); fixture.input.stop_requested=1U; low_tick(4000U);
  fixture.input.stop_requested=0U; platform_wide_boundary((float)direction*(outside?512.01F:511.98F));
  low_tick(4000U);
  assert(fixture.pilot.status.first_fault==(outside?SHAFT_FAULT_POSITION:SHAFT_FAULT_TRAVEL));
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief 反向5度和未达到真实平台的900ms结束仍只能ABORT。 */
static void test_platform_wide_reverse_and_deadline(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  platform_wide_ramp(direction,4000U);
  fixture.input.feedback.native_position_rad=-(float)direction*0.09F; low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
 }
 platform_wide_ramp(1,4000U);
 for(count=0U;count<225U;++count) {
  advance(4000U); fixture.input.receipt.decoded_torque_nm=fixture.output.previous_sent_nm;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && !fixture.pilot.plateau_seen); ++completed;
}
/** @brief 原周期、故障、温度、回执、独立租约、显式STOP和速度保护保持优先。 */
static void test_platform_wide_protections(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<8U;++scenario) {
  platform_wide_ramp(1,4000U);
  /* 独立注入即将满足平台期限的内部状态，检查故障/STOP优先级，不作为平台回执证明。 */
  fixture.pilot.plateau_seen=1U; fixture.pilot.plateau_observed_us=fixture.input.now_us-196000U;
  advance(scenario==0U?6000U:4000U);
  if(scenario==1U) fixture.input.feedback.driver_fault=1U;
  if(scenario==2U) fixture.input.feedback.mos_temperature_c=50.0F;
  if(scenario==3U) fixture.input.receipt.success=0U;
  if(scenario==4U) fixture.input.guard_live=0U;
  if(scenario==5U) fixture.input.stop_requested=1U;
  if(scenario==6U) fixture.input.feedback.raw_velocity_rad_s=20.7F;
  if(scenario==7U) fixture.input.feedback.rx_us=fixture.input.now_us-8001U;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  if(scenario==5U) assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
  else assert(fixture.pilot.status.first_fault!=SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief 真平台及新失能证据才允许ready，直接初始化与失败初始化均清除宽档行为。 */
static void test_platform_wide_stop_readiness_and_reinit(void)
{
 unsigned count; platform_wide_ramp(1,4000U);
 for(count=0U;count<310U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_TIME);
 fixture.input.guard_fired=1U; fixture.input.guard_acked=1U; fixture.input.guard_live=0U;
 for(count=0U;count<80U && fixture.pilot.status.phase!=SHAFT_PHASE_DONE;++count) low_tick(4000U);
 assert(fixture.pilot.status.safe_release && fixture.pilot.status.caller_next_transaction_ready);
 assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 assert(!fixture.pilot.observe_profile && !fixture.pilot.platform_profile && !fixture.pilot.platform_wide_profile);
 assert(shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,1));
 assert(!shaft_pilot_init_platform_wide(&fixture.pilot,NULL,1));
 assert(!fixture.pilot.initialized && !fixture.pilot.observe_profile && !fixture.pilot.platform_profile &&
  !fixture.pilot.platform_wide_profile);
 assert(shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_init_visible(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.visible_profile && !fixture.pilot.observe_profile && !fixture.pilot.platform_profile &&
  !fixture.pilot.platform_wide_profile);
 observe_prepare(1); assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(!fixture.pilot.platform_wide_profile);
 fixture.input.feedback.native_position_rad=9.3F;
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)); ++completed;
}
/** @brief 构造1024原生rad冻结量程；基线位置一码随真实新量程同步。 */
static void platform70_prepare(int direction)
{
 observe_prepare(direction); fixture.config.position_max_native_rad=1024.0F;
 fixture.config.baseline.span64_shaft_rad=2.0F*(2048.0F/65535.0F/fixture.config.gear_ratio);
}
/** @brief 在指定合法周期进入宽量程RAMP，全部回执仍是测试网格。 */
static void platform70_ramp(int direction, unsigned period_us)
{
 unsigned count; platform70_prepare(direction);
 assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,direction));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 for(count=0U;count<26U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(period_us);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
}
/** @brief 仅测试独立边界保护：替换历史，绝不把合成位置跳变当作连续运动证明。 */
static void platform70_boundary(float native_position)
{
 fixture.input.feedback.native_position_rad=native_position;
 fixture.pilot.history_count=0U; fixture.pilot.history_next=0U; fixture.pilot.feedback_seen=0U;
}
/** @brief 新API仅接受1024/200/10和合法方向，旧observe/platform仍拒绝1024。 */
static void test_platform70_config(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<8U;++scenario) {
  platform70_prepare(1);
  if(scenario==0U) fixture.config.position_max_native_rad=12.5F;
  if(scenario==1U) fixture.config.position_max_native_rad=1023.0F;
  if(scenario==2U) fixture.config.velocity_max_rad_s=5.0F;
  if(scenario==3U) fixture.config.torque_max_nm=20.0F;
  if(scenario==4U) fixture.config.position_max_native_rad=NAN;
  if(scenario==5U) fixture.config.velocity_max_rad_s=INFINITY;
  if(scenario==6U) fixture.config.baseline.point_count=63U;
  assert(!shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,scenario==7U?0:1));
  assert(!fixture.pilot.initialized && !fixture.pilot.observe_profile && !fixture.pilot.platform_profile &&
   !fixture.pilot.platform70_profile);
 }
 platform70_prepare(1); assert(!shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(!shaft_pilot_init_observe(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.observe_profile && fixture.pilot.platform_profile && fixture.pilot.platform70_profile);
 assert(fixture.pilot.plan.cap_nm==1.75F && fixture.pilot.plan.slew_nm_s==4.0F && fixture.pilot.plan.step_nm==0.016F);
 ++completed;
}
/** @brief 宽量程量化来自2048/65535；速度、力矩和停止门仍使用原冻结范围。 */
static void test_platform70_quantum(void)
{
 platform70_prepare(1); assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,1));
 assert(fabsf(fixture.pilot.status.quantum_native_rad-2048.0F/65535.0F)<1.0e-8F);
 assert(fabsf(fixture.pilot.status.quantum_shaft_rad-2048.0F/65535.0F/fixture.config.gear_ratio)<1.0e-9F);
 assert(fixture.pilot.status.quantum_velocity_rad_s==400.0F/4095.0F);
 assert(fixture.pilot.status.quantum_torque_nm==20.0F/4095.0F); ++completed;
}
/** @brief 两方向入口精确使用1058ms预算；该预算已含一次25ms且不重复计入。 */
static void test_platform70_entry_budget(void)
{
 int direction; unsigned allowed;
 for(direction=-1;direction<=1;direction+=2) for(allowed=0U;allowed<2U;++allowed) {
  float required, boundary; platform70_prepare(direction);
  assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,direction));
  required=25.1327412F+2.0F*fixture.pilot.status.quantum_native_rad+
   28.9550123F*fixture.config.gear_ratio*1.058F;
  boundary=1024.0F-required;
  fixture.input.feedback.native_position_rad=(float)direction*(boundary+(allowed?-0.001F:0.001F));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)==(int)allowed);
 }
 ++completed;
}
/** @brief 本方向预算充足也不能绕过反方向最低8pi加两码余量。 */
static void test_platform70_opposite_reserve(void)
{
 int direction; unsigned allowed;
 for(direction=-1;direction<=1;direction+=2) for(allowed=0U;allowed<2U;++allowed) {
  float minimum; platform70_prepare(direction);
  assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,direction));
  minimum=25.1327412F+2.0F*fixture.pilot.status.quantum_native_rad;
  fixture.input.feedback.native_position_rad=-(float)direction*(1024.0F-minimum+(allowed?-0.01F:0.01F));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)==(int)allowed);
 } ++completed;
}
/** @brief 两方向3/4/5ms真回执平台必须持续至少200ms并早于900ms退出。 */
static void test_platform70_plateau(void)
{
 int direction; unsigned period, count;
 for(direction=-1;direction<=1;direction+=2) for(period=3000U;period<=5000U;period+=1000U) {
  platform70_ramp(direction,period);
  for(count=0U;count<310U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) low_tick(period);
  assert(fixture.pilot.plateau_seen && fixture.pilot.status.outcome==SHAFT_END_TIME);
  assert(fixture.input.now_us-fixture.pilot.plateau_observed_us>=200000U);
  assert(fixture.input.now_us-fixture.pilot.phase_us<900000U && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE && fixture.pilot.status.peak_successful_decoded_torque_nm<=1.75F);
 } ++completed;
}
/** @brief 宽量程跨旧PMAX和相对角度仍保持RAMP，不能误套旧停止角。 */
static void test_platform70_cross_old_angles(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  platform70_ramp(direction,4000U);
  for(count=0U;count<100U;++count) {
   fixture.input.feedback.native_position_rad+=(float)direction*0.2F; low_tick(4000U);
   assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  }
  assert(fabsf(fixture.input.feedback.native_position_rad)>12.5F);
 } ++completed;
}
/** @brief 动态reserve同时覆盖raw及可用32ms速度，取绝对值较大者。 */
static void test_platform70_dynamic_reserve(void)
{
 int direction; unsigned source;
 for(direction=-1;direction<=1;direction+=2) for(source=0U;source<3U;++source) {
  platform70_ramp(direction,4000U); platform70_boundary((float)direction*998.6F);
  fixture.input.feedback.raw_velocity_rad_s=source==1U?-(float)direction:0.0F;
  if(source==2U) {
   fixture.pilot.history_count=1U; fixture.pilot.history_next=1U;
   fixture.pilot.history[0].rx_us=fixture.input.now_us+4000U-32000U;
   fixture.pilot.history[0].observed_us=fixture.pilot.history[0].rx_us;
   fixture.pilot.history[0].shaft_position_rad=(float)direction*(998.6F/fixture.config.gear_ratio-0.032F);
  }
  low_tick(4000U);
  if(source) assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.output.action==SHAFT_ACTION_DISABLE);
  else assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief 零速最低8pi加两码仍触发，双向一致。 */
static void test_platform70_static_reserve(void)
{
 int direction;
 for(direction=-1;direction<=1;direction+=2) {
  platform70_ramp(direction,4000U); platform70_boundary((float)direction*998.85F);
  fixture.input.feedback.raw_velocity_rad_s=0.0F; low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief ENABLE和NEUTRAL也执行宽停止余量，非零力矩之前不得跳过。 */
static void test_platform70_early_phase_reserve(void)
{
 int direction; unsigned neutral;
 for(direction=-1;direction<=1;direction+=2) for(neutral=0U;neutral<2U;++neutral) {
  platform70_prepare(direction); assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,direction));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  if(neutral) low_tick(4000U);
  platform70_boundary((float)direction*999.0F); low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.output.action==SHAFT_ACTION_DISABLE);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief 同周期范围不足不能掩盖驱动失能和ENABLE/NEUTRAL期限故障。 */
static void test_platform70_phase_priority(void)
{
 unsigned scenario, count;
 for(scenario=0U;scenario<3U;++scenario) {
  platform70_prepare(1); assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,1));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  if(scenario!=1U) low_tick(scenario==2U?3000U:4000U);
  if(scenario==1U) for(count=0U;count<15U;++count) {
   advance(4000U); fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
   shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  }
  if(scenario==2U) {
   fixture.input.feedback.raw_velocity_rad_s=0.049F;
   for(count=0U;count<22U;++count) low_tick(3000U);
  }
  platform70_boundary(999.0F); advance(scenario==2U?3000U:4000U);
  if(scenario!=2U) fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.pilot.status.first_fault==(scenario==0U?SHAFT_FAULT_DRIVER:
   (scenario==1U?SHAFT_FAULT_ENABLE_TIMEOUT:SHAFT_FAULT_NEUTRAL_TIMEOUT)));
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief STOP之后仍检查新PMAX绝对两码边界以及超过映射范围的POSITION故障。 */
static void test_platform70_stop_range(void)
{
 int direction; unsigned outside;
 for(direction=-1;direction<=1;direction+=2) for(outside=0U;outside<2U;++outside) {
  platform70_ramp(direction,4000U); fixture.input.stop_requested=1U; low_tick(4000U);
  fixture.input.stop_requested=0U; platform70_boundary((float)direction*(outside?1024.01F:1023.96F));
  low_tick(4000U);
  assert(fixture.pilot.status.first_fault==(outside?SHAFT_FAULT_POSITION:SHAFT_FAULT_TRAVEL));
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief 反向5度和未达到真实平台的900ms结束仍只能ABORT。 */
static void test_platform70_reverse_and_deadline(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  platform70_ramp(direction,4000U);
  fixture.input.feedback.native_position_rad=-(float)direction*0.09F; low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
 }
 platform70_ramp(1,4000U);
 for(count=0U;count<225U;++count) {
  advance(4000U); fixture.input.receipt.decoded_torque_nm=fixture.output.previous_sent_nm;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && !fixture.pilot.plateau_seen); ++completed;
}
/** @brief 原周期、故障、温度、回执、独立租约、显式STOP和速度保护保持优先。 */
static void test_platform70_protections(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<8U;++scenario) {
  platform70_ramp(1,4000U);
  /* 独立注入即将满足平台期限的内部状态，检查故障/STOP优先级，不作为平台回执证明。 */
  fixture.pilot.plateau_seen=1U; fixture.pilot.plateau_observed_us=fixture.input.now_us-196000U;
  advance(scenario==0U?6000U:4000U);
  if(scenario==1U) fixture.input.feedback.driver_fault=1U;
  if(scenario==2U) fixture.input.feedback.mos_temperature_c=50.0F;
  if(scenario==3U) fixture.input.receipt.success=0U;
  if(scenario==4U) fixture.input.guard_live=0U;
  if(scenario==5U) fixture.input.stop_requested=1U;
  if(scenario==6U) fixture.input.feedback.raw_velocity_rad_s=29.0F;
  if(scenario==7U) fixture.input.feedback.rx_us=fixture.input.now_us-8001U;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  if(scenario==5U) assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
  else assert(fixture.pilot.status.first_fault!=SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief 真平台及新失能证据才允许ready，直接初始化与失败初始化均清除宽档行为。 */
static void test_platform70_stop_readiness_and_reinit(void)
{
 unsigned count; platform70_ramp(1,4000U);
 for(count=0U;count<310U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_TIME);
 fixture.input.guard_fired=1U; fixture.input.guard_acked=1U; fixture.input.guard_live=0U;
 for(count=0U;count<80U && fixture.pilot.status.phase!=SHAFT_PHASE_DONE;++count) low_tick(4000U);
 assert(fixture.pilot.status.safe_release && fixture.pilot.status.caller_next_transaction_ready);
 assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 assert(!fixture.pilot.observe_profile && !fixture.pilot.platform_profile && !fixture.pilot.platform70_profile);
 assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,1));
 assert(!shaft_pilot_init_platform70(&fixture.pilot,NULL,1));
 assert(!fixture.pilot.initialized && !fixture.pilot.observe_profile && !fixture.pilot.platform_profile &&
  !fixture.pilot.platform70_profile);
 assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_init_visible(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.visible_profile && !fixture.pilot.observe_profile && !fixture.pilot.platform_profile &&
  !fixture.pilot.platform70_profile);
 observe_prepare(1); assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(!fixture.pilot.platform70_profile);
 fixture.input.feedback.native_position_rad=9.3F;
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)); ++completed;
}
/** @brief 新速度门按绝对值严格比较，旧半额定速度以上仍可观察；邻近超门值立即SPEED。 */
static void test_platform70_raw_speed_boundary(void)
{
 int direction; unsigned above;
 for(direction=-1;direction<=1;direction+=2) for(above=0U;above<2U;++above) {
  platform70_ramp(direction,4000U);
  fixture.input.feedback.raw_velocity_rad_s=(float)direction*(above?28.95502F:28.9550123F);
  low_tick(4000U);
  assert(fixture.pilot.status.first_fault==(above?SHAFT_FAULT_SPEED:SHAFT_FAULT_NONE));
  assert(fixture.output.action==(above?SHAFT_ACTION_DISABLE:SHAFT_ACTION_TORQUE));
 } ++completed;
}
/** @brief 32ms位移估速仍有独立70%门，raw零速不能掩盖它；27rad/s留出到达区间余量。 */
static void test_platform70_sampled_speed_boundary(void)
{
 /* 明确案例表保留两侧期望，避开ARMCC O3跨相邻assert误复用三目条件标志。 */
 static const struct { float speed_rad_s; ShaftPilotFault expected_fault; } cases[2]={
  {27.0F,SHAFT_FAULT_NONE},{30.0F,SHAFT_FAULT_SPEED}};
 int direction; unsigned above;
 for(direction=-1;direction<=1;direction+=2) for(above=0U;above<2U;++above) {
  float speed=cases[above].speed_rad_s;
  platform70_ramp(direction,4000U); platform70_boundary(0.0F);
  fixture.pilot.history_count=1U; fixture.pilot.history_next=1U;
  fixture.pilot.history[0].rx_us=fixture.input.now_us+4000U-32000U;
  fixture.pilot.history[0].observed_us=fixture.pilot.history[0].rx_us;
  fixture.pilot.history[0].shaft_position_rad=-(float)direction*speed*0.032F;
  low_tick(4000U);
  assert(fixture.pilot.status.velocity_ready);
  assert(fixture.pilot.status.first_fault==cases[above].expected_fault);
 } ++completed;
}
/** @brief ENABLE/NEUTRAL/RAMP各阶段均按raw与32ms绝对值最大者保留动态停止余量。 */
static void test_platform70_all_active_dynamic_reserve(void)
{
 int direction; unsigned phase, source, exhausted;
 for(direction=-1;direction<=1;direction+=2) for(phase=0U;phase<3U;++phase)
 for(source=0U;source<3U;++source) for(exhausted=0U;exhausted<2U;++exhausted) {
  float reserve, native_position;
  platform70_prepare(direction); assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,direction));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  if(phase==1U) low_tick(4000U);
  if(phase==2U) {
   unsigned count;
   for(count=0U;count<26U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(4000U);
   assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
  }
  reserve=25.1327412F+2.0F*fixture.pilot.status.quantum_native_rad+
   (source?1.0F:0.0F)*fixture.config.gear_ratio*0.025F;
  native_position=(float)direction*(1024.0F-reserve+(exhausted?0.01F:-0.01F));
  platform70_boundary(native_position);
  fixture.input.feedback.raw_velocity_rad_s=source==1U?-(float)direction:0.0F;
  if(source==2U) {
   fixture.pilot.history_count=1U; fixture.pilot.history_next=1U;
   fixture.pilot.history[0].rx_us=fixture.input.now_us+4000U-32000U;
   fixture.pilot.history[0].observed_us=fixture.pilot.history[0].rx_us;
   fixture.pilot.history[0].shaft_position_rad=native_position/fixture.config.gear_ratio+(float)direction*0.032F;
  }
  low_tick(4000U);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert((fixture.pilot.status.outcome==SHAFT_END_ABORT)==(int)exhausted);
  assert(fixture.output.action==(exhausted?SHAFT_ACTION_DISABLE:SHAFT_ACTION_TORQUE));
 } ++completed;
}
/** @brief 所有旧初始化与失败初始化逐字节清除新旗标，防止复用实例继承70%速度。 */
static void test_platform70_all_reinitializers(void)
{
 static ShaftPilot pristine;
 unsigned profile;
 for(profile=0U;profile<7U;++profile) {
  int actual, expected;
  platform70_prepare(1); assert(shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,1));
  if(profile==3U) {
   fixture.config.velocity_max_rad_s=5.0F;
   fixture.config.baseline.raw_zero_center_rad_s=5.0F/4095.0F;
  }
  if(profile==4U || profile==5U) fixture.config.position_max_native_rad=12.5F;
  if(profile==6U) fixture.config.position_max_native_rad=512.0F;
  if(profile==0U) {
   actual=shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan);
   expected=shaft_pilot_init(&pristine,&fixture.config,&fixture.plan);
  } else if(profile==1U) {
   actual=shaft_pilot_init_low(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_low(&pristine,&fixture.config,1);
  } else if(profile==2U) {
   actual=shaft_pilot_init_visible(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_visible(&pristine,&fixture.config,1);
  } else if(profile==3U) {
   actual=shaft_pilot_init_fine(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_fine(&pristine,&fixture.config,1);
  } else if(profile==4U) {
   actual=shaft_pilot_init_observe(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_observe(&pristine,&fixture.config,1);
  } else if(profile==5U) {
   actual=shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_platform(&pristine,&fixture.config,1);
  } else {
   actual=shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_platform_wide(&pristine,&fixture.config,1);
  }
  assert(actual && expected && !memcmp(&fixture.pilot,&pristine,sizeof(pristine)));
 }
 assert(shaft_pilot_init_platform70(NULL,&fixture.config,1)==0);
 ++completed;
}
/** @brief 构造1024原生rad冻结量程；基线位置一码随真实新量程同步。 */
static void platform100_prepare(int direction)
{
 observe_prepare(direction); fixture.config.position_max_native_rad=1024.0F;
 fixture.config.baseline.span64_shaft_rad=2.0F*(2048.0F/65535.0F/fixture.config.gear_ratio);
}
/** @brief 在指定合法周期进入宽量程RAMP，全部回执仍是测试网格。 */
static void platform100_ramp(int direction, unsigned period_us)
{
 unsigned count; platform100_prepare(direction);
 assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,direction));
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
 for(count=0U;count<26U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(period_us);
 assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
}
/** @brief 仅测试独立边界保护：替换历史，绝不把合成位置跳变当作连续运动证明。 */
static void platform100_boundary(float native_position)
{
 fixture.input.feedback.native_position_rad=native_position;
 fixture.pilot.history_count=0U; fixture.pilot.history_next=0U; fixture.pilot.feedback_seen=0U;
}
/** @brief 新API仅接受1024/200/10和合法方向，旧observe/platform仍拒绝1024。 */
static void test_platform100_config(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<8U;++scenario) {
  platform100_prepare(1);
  if(scenario==0U) fixture.config.position_max_native_rad=12.5F;
  if(scenario==1U) fixture.config.position_max_native_rad=1023.0F;
  if(scenario==2U) fixture.config.velocity_max_rad_s=5.0F;
  if(scenario==3U) fixture.config.torque_max_nm=20.0F;
  if(scenario==4U) fixture.config.position_max_native_rad=NAN;
  if(scenario==5U) fixture.config.velocity_max_rad_s=INFINITY;
  if(scenario==6U) fixture.config.baseline.point_count=63U;
  assert(!shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,scenario==7U?0:1));
  assert(!fixture.pilot.initialized && !fixture.pilot.observe_profile && !fixture.pilot.platform_profile &&
   !fixture.pilot.platform100_profile);
 }
 platform100_prepare(1); assert(!shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(!shaft_pilot_init_observe(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.observe_profile && fixture.pilot.platform_profile && fixture.pilot.platform100_profile);
 assert(fixture.pilot.plan.cap_nm==1.0F && fixture.pilot.plan.slew_nm_s==4.0F && fixture.pilot.plan.step_nm==0.016F);
 ++completed;
}
/** @brief 宽量程量化来自2048/65535；速度、力矩和停止门仍使用原冻结范围。 */
static void test_platform100_quantum(void)
{
 platform100_prepare(1); assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,1));
 assert(fabsf(fixture.pilot.status.quantum_native_rad-2048.0F/65535.0F)<1.0e-8F);
 assert(fabsf(fixture.pilot.status.quantum_shaft_rad-2048.0F/65535.0F/fixture.config.gear_ratio)<1.0e-9F);
 assert(fixture.pilot.status.quantum_velocity_rad_s==400.0F/4095.0F);
 assert(fixture.pilot.status.quantum_torque_nm==20.0F/4095.0F); ++completed;
}
/** @brief 两方向入口精确使用1058ms预算；该预算已含一次25ms且不重复计入。 */
static void test_platform100_entry_budget(void)
{
 int direction; unsigned allowed;
 for(direction=-1;direction<=1;direction+=2) for(allowed=0U;allowed<2U;++allowed) {
  float required, boundary; platform100_prepare(direction);
  assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,direction));
  required=56.5486679F+2.0F*fixture.pilot.status.quantum_native_rad+
   41.3643036F*fixture.config.gear_ratio*1.058F;
  boundary=1024.0F-required;
  fixture.input.feedback.native_position_rad=(float)direction*(boundary+(allowed?-0.001F:0.001F));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)==(int)allowed);
 }
 ++completed;
}
/** @brief 本方向预算充足也不能绕过反方向最低18pi加两码余量。 */
static void test_platform100_opposite_reserve(void)
{
 int direction; unsigned allowed;
 for(direction=-1;direction<=1;direction+=2) for(allowed=0U;allowed<2U;++allowed) {
  float minimum; platform100_prepare(direction);
  assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,direction));
  minimum=56.5486679F+2.0F*fixture.pilot.status.quantum_native_rad;
  fixture.input.feedback.native_position_rad=-(float)direction*(1024.0F-minimum+(allowed?-0.01F:0.01F));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)==(int)allowed);
 } ++completed;
}
/** @brief 两方向3/4/5ms真回执平台必须持续至少200ms并早于900ms退出。 */
static void test_platform100_plateau(void)
{
 int direction; unsigned period, count;
 for(direction=-1;direction<=1;direction+=2) for(period=3000U;period<=5000U;period+=1000U) {
  platform100_ramp(direction,period);
  for(count=0U;count<310U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) low_tick(period);
  assert(fixture.pilot.plateau_seen && fixture.pilot.status.outcome==SHAFT_END_TIME);
  assert(fixture.input.now_us-fixture.pilot.plateau_observed_us>=200000U);
  assert(fixture.input.now_us-fixture.pilot.phase_us<900000U && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE && fixture.pilot.status.peak_successful_decoded_torque_nm<=1.0F);
 } ++completed;
}
/** @brief 宽量程跨旧PMAX和相对角度仍保持RAMP，不能误套旧停止角。 */
static void test_platform100_cross_old_angles(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  platform100_ramp(direction,4000U);
  for(count=0U;count<100U;++count) {
   fixture.input.feedback.native_position_rad+=(float)direction*0.2F; low_tick(4000U);
   assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  }
  assert(fabsf(fixture.input.feedback.native_position_rad)>12.5F);
 } ++completed;
}
/** @brief 动态reserve同时覆盖raw及可用32ms速度，取绝对值较大者。 */
static void test_platform100_dynamic_reserve(void)
{
 int direction; unsigned source;
 for(direction=-1;direction<=1;direction+=2) for(source=0U;source<3U;++source) {
  platform100_ramp(direction,4000U); platform100_boundary((float)direction*967.2F);
  fixture.input.feedback.raw_velocity_rad_s=source==1U?-(float)direction:0.0F;
  if(source==2U) {
   fixture.pilot.history_count=1U; fixture.pilot.history_next=1U;
   fixture.pilot.history[0].rx_us=fixture.input.now_us+4000U-32000U;
   fixture.pilot.history[0].observed_us=fixture.pilot.history[0].rx_us;
   fixture.pilot.history[0].shaft_position_rad=(float)direction*(967.2F/fixture.config.gear_ratio-0.032F);
  }
  low_tick(4000U);
  if(source) assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.output.action==SHAFT_ACTION_DISABLE);
  else assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief 零速最低18pi加两码仍触发，双向一致。 */
static void test_platform100_static_reserve(void)
{
 int direction;
 for(direction=-1;direction<=1;direction+=2) {
  platform100_ramp(direction,4000U); platform100_boundary((float)direction*967.4F);
  fixture.input.feedback.raw_velocity_rad_s=0.0F; low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief ENABLE和NEUTRAL也执行宽停止余量，非零力矩之前不得跳过。 */
static void test_platform100_early_phase_reserve(void)
{
 int direction; unsigned neutral;
 for(direction=-1;direction<=1;direction+=2) for(neutral=0U;neutral<2U;++neutral) {
  platform100_prepare(direction); assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,direction));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  if(neutral) low_tick(4000U);
  platform100_boundary((float)direction*967.5F); low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && fixture.output.action==SHAFT_ACTION_DISABLE);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief 同周期范围不足不能掩盖驱动失能和ENABLE/NEUTRAL期限故障。 */
static void test_platform100_phase_priority(void)
{
 unsigned scenario, count;
 for(scenario=0U;scenario<3U;++scenario) {
  platform100_prepare(1); assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,1));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  if(scenario!=1U) low_tick(scenario==2U?3000U:4000U);
  if(scenario==1U) for(count=0U;count<15U;++count) {
   advance(4000U); fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
   shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  }
  if(scenario==2U) {
   fixture.input.feedback.raw_velocity_rad_s=0.049F;
   for(count=0U;count<22U;++count) low_tick(3000U);
  }
  platform100_boundary(967.5F); advance(scenario==2U?3000U:4000U);
  if(scenario!=2U) fixture.input.feedback.driver_state=SHAFT_DRIVER_DISABLED;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.pilot.status.first_fault==(scenario==0U?SHAFT_FAULT_DRIVER:
   (scenario==1U?SHAFT_FAULT_ENABLE_TIMEOUT:SHAFT_FAULT_NEUTRAL_TIMEOUT)));
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief STOP之后仍检查新PMAX绝对两码边界以及超过映射范围的POSITION故障。 */
static void test_platform100_stop_range(void)
{
 int direction; unsigned outside;
 for(direction=-1;direction<=1;direction+=2) for(outside=0U;outside<2U;++outside) {
  platform100_ramp(direction,4000U); fixture.input.stop_requested=1U; low_tick(4000U);
  fixture.input.stop_requested=0U; platform100_boundary((float)direction*(outside?1024.01F:1023.96F));
  low_tick(4000U);
  assert(fixture.pilot.status.first_fault==(outside?SHAFT_FAULT_POSITION:SHAFT_FAULT_TRAVEL));
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
 } ++completed;
}
/** @brief 反向5度和未达到真实平台的900ms结束仍只能ABORT。 */
static void test_platform100_reverse_and_deadline(void)
{
 int direction; unsigned count;
 for(direction=-1;direction<=1;direction+=2) {
  platform100_ramp(direction,4000U);
  fixture.input.feedback.native_position_rad=-(float)direction*0.09F; low_tick(4000U);
  assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
 }
 platform100_ramp(1,4000U);
 for(count=0U;count<225U;++count) {
  advance(4000U); fixture.input.receipt.decoded_torque_nm=fixture.output.previous_sent_nm;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
 }
 assert(fixture.pilot.status.outcome==SHAFT_END_ABORT && !fixture.pilot.plateau_seen); ++completed;
}
/** @brief 原周期、故障、温度、回执、独立租约、显式STOP和速度保护保持优先。 */
static void test_platform100_protections(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<8U;++scenario) {
  platform100_ramp(1,4000U);
  /* 独立注入即将满足平台期限的内部状态，检查故障/STOP优先级，不作为平台回执证明。 */
  fixture.pilot.plateau_seen=1U; fixture.pilot.plateau_observed_us=fixture.input.now_us-196000U;
  advance(scenario==0U?6000U:4000U);
  if(scenario==1U) fixture.input.feedback.driver_fault=1U;
  if(scenario==2U) fixture.input.feedback.mos_temperature_c=50.0F;
  if(scenario==3U) fixture.input.receipt.success=0U;
  if(scenario==4U) fixture.input.guard_live=0U;
  if(scenario==5U) fixture.input.stop_requested=1U;
  if(scenario==6U) fixture.input.feedback.raw_velocity_rad_s=42.0F;
  if(scenario==7U) fixture.input.feedback.rx_us=fixture.input.now_us-8001U;
  shaft_pilot_tick(&fixture.pilot,&fixture.input,&fixture.output);
  assert(fixture.output.action==SHAFT_ACTION_DISABLE);
  if(scenario==5U) assert(fixture.pilot.status.outcome==SHAFT_END_ABORT);
  else assert(fixture.pilot.status.first_fault!=SHAFT_FAULT_NONE);
 } ++completed;
}
/** @brief 真平台及新失能证据才允许ready，直接初始化与失败初始化均清除宽档行为。 */
static void test_platform100_stop_readiness_and_reinit(void)
{
 unsigned count; platform100_ramp(1,4000U);
 for(count=0U;count<310U && fixture.pilot.status.phase==SHAFT_PHASE_RAMP;++count) low_tick(4000U);
 assert(fixture.pilot.status.outcome==SHAFT_END_TIME);
 fixture.input.guard_fired=1U; fixture.input.guard_acked=1U; fixture.input.guard_live=0U;
 for(count=0U;count<80U && fixture.pilot.status.phase!=SHAFT_PHASE_DONE;++count) low_tick(4000U);
 assert(fixture.pilot.status.safe_release && fixture.pilot.status.caller_next_transaction_ready);
 assert(shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan));
 assert(!fixture.pilot.observe_profile && !fixture.pilot.platform_profile && !fixture.pilot.platform100_profile);
 assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,1));
 assert(!shaft_pilot_init_platform100(&fixture.pilot,NULL,1));
 assert(!fixture.pilot.initialized && !fixture.pilot.observe_profile && !fixture.pilot.platform_profile &&
  !fixture.pilot.platform100_profile);
 assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,1));
 assert(shaft_pilot_init_visible(&fixture.pilot,&fixture.config,1));
 assert(fixture.pilot.visible_profile && !fixture.pilot.observe_profile && !fixture.pilot.platform_profile &&
  !fixture.pilot.platform100_profile);
 observe_prepare(1); assert(shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1));
 assert(!fixture.pilot.platform100_profile);
 fixture.input.feedback.native_position_rad=9.3F;
 assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output)); ++completed;
}
/** @brief 新速度门按绝对值严格比较，旧半额定速度以上仍可观察；邻近超门值立即SPEED。 */
static void test_platform100_raw_speed_boundary(void)
{
 int direction; unsigned above;
 for(direction=-1;direction<=1;direction+=2) for(above=0U;above<2U;++above) {
  platform100_ramp(direction,4000U);
  fixture.input.feedback.raw_velocity_rad_s=(float)direction*(above?41.36432F:41.3643036F);
  low_tick(4000U);
  assert(fixture.pilot.status.first_fault==(above?SHAFT_FAULT_SPEED:SHAFT_FAULT_NONE));
  assert(fixture.output.action==(above?SHAFT_ACTION_DISABLE:SHAFT_ACTION_TORQUE));
 } ++completed;
}
/** @brief 32ms位移估速仍有独立100%门，raw零速不能掩盖它；38rad/s留出到达区间余量。 */
static void test_platform100_sampled_speed_boundary(void)
{
 /* 明确案例表保留两侧期望，避开ARMCC O3跨相邻assert误复用三目条件标志。 */
 static const struct { float speed_rad_s; ShaftPilotFault expected_fault; } cases[2]={
  {38.0F,SHAFT_FAULT_NONE},{42.0F,SHAFT_FAULT_SPEED}};
 int direction; unsigned above;
 for(direction=-1;direction<=1;direction+=2) for(above=0U;above<2U;++above) {
  float speed=cases[above].speed_rad_s;
  platform100_ramp(direction,4000U); platform100_boundary(0.0F);
  fixture.pilot.history_count=1U; fixture.pilot.history_next=1U;
  fixture.pilot.history[0].rx_us=fixture.input.now_us+4000U-32000U;
  fixture.pilot.history[0].observed_us=fixture.pilot.history[0].rx_us;
  fixture.pilot.history[0].shaft_position_rad=-(float)direction*speed*0.032F;
  low_tick(4000U);
  assert(fixture.pilot.status.velocity_ready);
  assert(fixture.pilot.status.first_fault==cases[above].expected_fault);
 } ++completed;
}
/** @brief ENABLE/NEUTRAL/RAMP各阶段均按raw与32ms绝对值最大者保留动态停止余量。 */
static void test_platform100_all_active_dynamic_reserve(void)
{
 int direction; unsigned phase, source, exhausted;
 for(direction=-1;direction<=1;direction+=2) for(phase=0U;phase<3U;++phase)
 for(source=0U;source<3U;++source) for(exhausted=0U;exhausted<2U;++exhausted) {
  float reserve, native_position;
  platform100_prepare(direction); assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,direction));
  assert(shaft_pilot_start(&fixture.pilot,&fixture.input,&fixture.output));
  if(phase==1U) low_tick(4000U);
  if(phase==2U) {
   unsigned count;
   for(count=0U;count<26U && fixture.pilot.status.phase!=SHAFT_PHASE_RAMP;++count) low_tick(4000U);
   assert(fixture.pilot.status.phase==SHAFT_PHASE_RAMP);
  }
  reserve=56.5486679F+2.0F*fixture.pilot.status.quantum_native_rad+
   (source?1.0F:0.0F)*fixture.config.gear_ratio*0.025F;
  native_position=(float)direction*(1024.0F-reserve+(exhausted?0.01F:-0.01F));
  platform100_boundary(native_position);
  fixture.input.feedback.raw_velocity_rad_s=source==1U?-(float)direction:0.0F;
  if(source==2U) {
   fixture.pilot.history_count=1U; fixture.pilot.history_next=1U;
   fixture.pilot.history[0].rx_us=fixture.input.now_us+4000U-32000U;
   fixture.pilot.history[0].observed_us=fixture.pilot.history[0].rx_us;
   fixture.pilot.history[0].shaft_position_rad=native_position/fixture.config.gear_ratio+(float)direction*0.032F;
  }
  low_tick(4000U);
  assert(fixture.pilot.status.first_fault==SHAFT_FAULT_NONE);
  assert((fixture.pilot.status.outcome==SHAFT_END_ABORT)==(int)exhausted);
  assert(fixture.output.action==(exhausted?SHAFT_ACTION_DISABLE:SHAFT_ACTION_TORQUE));
 } ++completed;
}
/** @brief 所有旧初始化与失败初始化逐字节清除新旗标，防止复用实例继承100%速度。 */
static void test_platform100_all_reinitializers(void)
{
 static ShaftPilot pristine;
 unsigned profile;
 for(profile=0U;profile<8U;++profile) {
  int actual, expected;
  platform100_prepare(1); assert(shaft_pilot_init_platform100(&fixture.pilot,&fixture.config,1));
  if(profile==3U) {
   fixture.config.velocity_max_rad_s=5.0F;
   fixture.config.baseline.raw_zero_center_rad_s=5.0F/4095.0F;
  }
  if(profile==4U || profile==5U) fixture.config.position_max_native_rad=12.5F;
  if(profile==6U) fixture.config.position_max_native_rad=512.0F;
  if(profile==0U) {
   actual=shaft_pilot_init(&fixture.pilot,&fixture.config,&fixture.plan);
   expected=shaft_pilot_init(&pristine,&fixture.config,&fixture.plan);
  } else if(profile==1U) {
   actual=shaft_pilot_init_low(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_low(&pristine,&fixture.config,1);
  } else if(profile==2U) {
   actual=shaft_pilot_init_visible(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_visible(&pristine,&fixture.config,1);
  } else if(profile==3U) {
   actual=shaft_pilot_init_fine(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_fine(&pristine,&fixture.config,1);
  } else if(profile==4U) {
   actual=shaft_pilot_init_observe(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_observe(&pristine,&fixture.config,1);
  } else if(profile==5U) {
   actual=shaft_pilot_init_platform(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_platform(&pristine,&fixture.config,1);
  } else if(profile==6U) {
   actual=shaft_pilot_init_platform_wide(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_platform_wide(&pristine,&fixture.config,1);
  } else {
   actual=shaft_pilot_init_platform70(&fixture.pilot,&fixture.config,1);
   expected=shaft_pilot_init_platform70(&pristine,&fixture.config,1);
  }
  assert(actual && expected && !memcmp(&fixture.pilot,&pristine,sizeof(pristine)));
 }
 assert(shaft_pilot_init_platform100(NULL,&fixture.config,1)==0);
 ++completed;
}
/** @brief Run all deterministic synthetic groups with assert failures retained as a nonzero exit. */
int main(void)
{
 test_config(); test_entry(); test_receipts(); test_ramp_grid(); test_target();
 test_time_end(); test_active_gates(); test_arrival_window(); test_adjacent_zero_tmin();
 test_stop_recovery(); test_stop_gap(); test_lock_and_trace(); test_stop_anchor_receipt();
 test_stop_baseline(); test_feedback_integrity();
 test_recorded_config_compatibility(); test_neutral_deadline();
 test_final_receipt_before_stop(); test_late_stop_completion(); test_derived_finite();
 test_next_transaction_gate();
 test_violating_send_diagnostic();
 test_neutral_bootstrap_reference();
 default_completed=completed; assert(default_completed==23U);
 test_low_config(); test_low_plateau_time(); test_low_unreached_plateau();
 test_low_displacement_abort(); test_low_baseline_no_abort(); test_low_raw_abort();
 test_low_protection_priority(); test_low_receipt_integrity(); test_low_stop_readiness();
 test_low_plateau_deadline_priority(); test_low_late_plateau_and_ready_guard();
 test_low_stop_last_receipt(); test_low_no_target_and_default_reinit(); test_low_stop_gap_and_lock();
 low_completed=completed; assert(low_completed==37U);
 test_visible_config(); test_visible_plateau(); test_visible_angle(); test_visible_observation_envelope();
 test_visible_early_abort(); test_visible_deadline_receipt(); test_visible_original_protection(); test_visible_stop_and_reinit();
 visible_completed=completed; assert(visible_completed==45U);
 test_observe_config(); test_observe_no_old_speed_endpoint(); test_observe_raw_speed_boundary();
 test_observe_angle_endpoint(); test_observe_reverse_endpoint(); test_observe_sampled_angle_and_integrity();
 test_observe_native_travel(); test_observe_plateau_scheduling(); test_observe_unreached_platform_deadline();
 test_observe_other_protection(); test_observe_stop_and_reinit(); test_observe_entry_headroom();
 observe_completed=completed; assert(observe_completed==57U);
 test_platform_config(); test_platform_cross_old_angles(); test_platform_true_plateau();
 test_platform_dynamic_reserve(); test_platform_static_reserve(); test_platform_entry_reserve();
 test_platform_early_phase_reserve();
 test_platform_phase_fault_priority();
 test_platform_stop_range(); test_platform_reverse_and_deadline(); test_platform_protections();
 test_platform_stop_readiness_and_reinit(); assert(completed==69U);
 platform_completed=completed;
 test_platform_wide_config(); test_platform_wide_quantum(); test_platform_wide_entry_budget();
 test_platform_wide_opposite_reserve(); test_platform_wide_plateau(); test_platform_wide_cross_old_angles();
 test_platform_wide_dynamic_reserve(); test_platform_wide_static_reserve(); test_platform_wide_early_phase_reserve();
 test_platform_wide_phase_priority(); test_platform_wide_stop_range(); test_platform_wide_reverse_and_deadline();
 test_platform_wide_protections(); test_platform_wide_stop_readiness_and_reinit(); assert(completed==83U);
 wide_completed=completed;
 test_platform70_config(); test_platform70_quantum(); test_platform70_entry_budget();
 test_platform70_opposite_reserve(); test_platform70_plateau(); test_platform70_cross_old_angles();
 test_platform70_dynamic_reserve(); test_platform70_static_reserve(); test_platform70_early_phase_reserve();
 test_platform70_phase_priority(); test_platform70_stop_range(); test_platform70_reverse_and_deadline();
 test_platform70_protections(); test_platform70_stop_readiness_and_reinit();
 test_platform70_raw_speed_boundary(); test_platform70_sampled_speed_boundary();
 test_platform70_all_active_dynamic_reserve(); test_platform70_all_reinitializers(); assert(completed==101U);
 rated70_completed=completed;
 test_platform100_config(); test_platform100_quantum(); test_platform100_entry_budget();
 test_platform100_opposite_reserve(); test_platform100_plateau(); test_platform100_cross_old_angles();
 test_platform100_dynamic_reserve(); test_platform100_static_reserve(); test_platform100_early_phase_reserve();
 test_platform100_phase_priority(); test_platform100_stop_range(); test_platform100_reverse_and_deadline();
 test_platform100_protections(); test_platform100_stop_readiness_and_reinit();
 test_platform100_raw_speed_boundary(); test_platform100_sampled_speed_boundary();
 test_platform100_all_active_dynamic_reserve(); test_platform100_all_reinitializers(); assert(completed==119U);
 printf("platform70_groups=%u legacy_groups=%u; fixed_pmax=1024 speed=28.9550123 reserve=25.1327412\n",rated70_completed-wide_completed,wide_completed);
 printf("platform100_groups=%u legacy_groups=%u; fixed_pmax=1024 speed=41.3643036 reserve=56.5486679 cap=1\n",completed-rated70_completed,rated70_completed);
 printf("default_groups=%u low_groups=%u visible_groups=%u observe_groups=%u platform_groups=%u wide_groups=%u; synthetic_assumptions_only=true\n",default_completed,low_completed-default_completed,visible_completed-low_completed,observe_completed-visible_completed,platform_completed-observe_completed,wide_completed-platform_completed);
 printf("PASS %u synthetic groups; physical_send_proof=false qualification=false\n",completed);
 printf("host_state_bytes=%lu trace_capacity=%u; MCU_memory_acceptance=false\n",
  (unsigned long)sizeof(ShaftPilot),(unsigned)SHAFT_PILOT_TRACE_CAPACITY);
 return 0;
}
