/** @file shaft_pilot.c
 * @brief Pure finite pilot with receipt-derived ramp and independent stopped-evidence recovery.
 * @note Arrival intervals are conditional observations; internal physical sampling remains unknown.
 */
#include "adrc_shaft_pilot.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/** @brief Reject NaN/infinities without relying on platform-specific classification macros. */
static int finite_scalar(float value)
{ return value==value && value<=FLT_MAX && value>=-FLT_MAX; }
/** @brief Require a finite strictly positive range or plan parameter. */
static int positive_scalar(float value)
{ return finite_scalar(value) && value>0.0F; }
/** @brief 独立100%/70%档选择冻结速度；历史档维持原预算。 */
static float velocity_budget(const ShaftPilot *pilot)
{ return pilot->platform100_profile?SHAFT_PLATFORM100_SPEED_RAD_S:
  (pilot->platform70_profile?SHAFT_PLATFORM70_SPEED_RAD_S:
  (pilot->observe_profile?SHAFT_OBSERVE_SPEED_RAD_S:0.3F)); }
/** @brief Reserve native range using observed speed and frozen latency arithmetic, not physical braking proof. */
static float platform_range_reserve(const ShaftPilot *pilot, const ShaftPilotInput *input)
{
 float velocity=fabsf(input->feedback.raw_velocity_rad_s);
 if(pilot->status.velocity_ready) velocity=fmaxf(velocity,fabsf(pilot->status.velocity_rx32_rad_s));
 return (pilot->platform100_profile?SHAFT_PLATFORM100_RESERVE_NATIVE_RAD:
  (pilot->platform70_profile?SHAFT_PLATFORM70_RESERVE_NATIVE_RAD:
  (pilot->platform_wide_profile?SHAFT_PLATFORM_WIDE_RESERVE_NATIVE_RAD:
  SHAFT_PLATFORM_RESERVE_NATIVE_RAD)))+2.0F*pilot->status.quantum_native_rad+
  velocity*pilot->config.gear_ratio*((float)SHAFT_PLATFORM_LOOKAHEAD_US*1.0e-6F);
}
/** @brief Keep a sampled forward range reserve; reaching it aborts rather than qualifying a plateau. */
static int platform_range_exhausted(const ShaftPilot *pilot, const ShaftPilotInput *input)
{
 float remaining=pilot->config.position_max_native_rad-
  (float)pilot->plan.direction*input->feedback.native_position_rad;
 return remaining<=platform_range_reserve(pilot,input);
}
/** @brief Use only magnitude-relative float rounding, never a neighboring velocity-code allowance. */
static float rounding_allowance(float value)
{ return 4.0F*FLT_EPSILON*fabsf(value); }
/** @brief Check the captured zero center magnitude against the true midtread protocol grid. */
static int raw_still(const ShaftPilot *pilot, float velocity)
{
 float center=fabsf(pilot->config.baseline.raw_zero_center_rad_s);
 return finite_scalar(velocity) && fabsf(velocity)<=0.05F &&
  fabsf(velocity)<=center+rounding_allowance(center);
}
/** @brief Identify phases that can request ENABLE/TORQUE. */
static int active_phase(ShaftPilotPhase phase)
{ return phase>=SHAFT_PHASE_ENABLE && phase<=SHAFT_PHASE_RAMP; }
/** @brief Preserve the first fault while revoking the trial's natural/abort result. */
static void latch_fault(ShaftPilot *pilot, ShaftPilotFault fault)
{
 if(fault==SHAFT_FAULT_NONE) return;
 if(pilot->status.first_fault==SHAFT_FAULT_NONE) pilot->status.first_fault=fault;
 pilot->status.outcome=SHAFT_END_FAULT;
}
/** @brief Clear history and current stop dwell after a gap or untrustworthy sample. */
static void reset_history(ShaftPilot *pilot)
{
 pilot->history_count=0U; pilot->history_next=0U;
 pilot->status.velocity_ready=0U; pilot->dwell_tracking=0U;
 pilot->status.safe_release=0U; pilot->status.caller_next_transaction_ready=0U;
}
/** @brief Begin immediate disable stopping exactly once; subsequent faults cannot move the anchor. */
static void begin_stop(ShaftPilot *pilot, ShaftPilotOutcome outcome, uint64_t now_us)
{
 if(!active_phase(pilot->status.phase)) return;
 if(pilot->status.first_fault==SHAFT_FAULT_NONE) pilot->status.outcome=outcome;
 pilot->status.phase=SHAFT_PHASE_STOPPING; pilot->stop_us=now_us;
 pilot->disable_confirmed=0U; pilot->dwell_tracking=0U;
 pilot->status.safe_release=0U;
}
/** @brief Verify frozen motor7/session identity; no external total ratio is ever accepted. */
static ShaftPilotFault binding_health(const ShaftPilot *pilot, const ShaftPilotInput *input)
{
 if(input->motor_id!=7U || input->binding_id!=pilot->config.binding_id ||
    input->session_id!=pilot->config.session_id) return SHAFT_FAULT_BINDING;
 return SHAFT_FAULT_NONE;
}
/** @brief Check timestamp ordering, fresh finite feedback, driver state and conservative temperatures. */
static ShaftPilotFault feedback_health(const ShaftPilot *pilot, const ShaftPilotInput *input,
 float torque_gate_nm)
{
 const ShaftPilotFeedback *feedback=&input->feedback;
 if(!feedback->valid || !feedback->rx_us || feedback->rx_us>feedback->observed_us ||
    feedback->observed_us>input->now_us || input->now_us-feedback->rx_us>8000U ||
    !finite_scalar(feedback->native_position_rad) || !finite_scalar(feedback->raw_velocity_rad_s) ||
    !finite_scalar(feedback->torque_nm)) return SHAFT_FAULT_FEEDBACK;
 if(fabsf(feedback->native_position_rad)>pilot->config.position_max_native_rad)
  return SHAFT_FAULT_POSITION;
 if(feedback->driver_fault || (feedback->driver_state!=SHAFT_DRIVER_DISABLED &&
    feedback->driver_state!=SHAFT_DRIVER_ENABLED) || fabsf(feedback->torque_nm)>torque_gate_nm)
  return SHAFT_FAULT_DRIVER;
 if(!finite_scalar(feedback->mos_temperature_c) || !finite_scalar(feedback->rotor_temperature_c) ||
    feedback->mos_temperature_c>=50.0F || feedback->rotor_temperature_c>=50.0F)
  return SHAFT_FAULT_TEMPERATURE;
 return SHAFT_FAULT_NONE;
}
/** @brief Reject duplicate receive-time mutations while allowing repeated unchanged snapshots. */
static int unchanged_sample(const ShaftPilotFeedback *current, const ShaftPilotFeedback *previous)
{
 return current->native_position_rad==previous->native_position_rad &&
  current->raw_velocity_rad_s==previous->raw_velocity_rad_s && current->torque_nm==previous->torque_nm &&
  current->mos_temperature_c==previous->mos_temperature_c &&
  current->rotor_temperature_c==previous->rotor_temperature_c &&
  current->driver_state==previous->driver_state && current->driver_fault==previous->driver_fault &&
  current->observed_us==previous->observed_us;
}
/** @brief Compute conditional arrival-time interval with 2ms endpoint allowance and no underflow. */
static int arrival_interval(const ShaftPilotSample *old_sample, const ShaftPilotSample *new_sample,
 uint64_t *minimum_us, uint64_t *maximum_us)
{
 uint64_t minimum_delta;
 if(new_sample->rx_us<=old_sample->rx_us || new_sample->observed_us<old_sample->observed_us ||
    new_sample->observed_us<old_sample->rx_us ||
    new_sample->observed_us-old_sample->rx_us>UINT64_MAX-2000U) return 0;
 minimum_delta=new_sample->rx_us>old_sample->observed_us?
  new_sample->rx_us-old_sample->observed_us:0U;
 *minimum_us=minimum_delta>2000U?minimum_delta-2000U:0U;
 *maximum_us=new_sample->observed_us-old_sample->rx_us+2000U;
 return 1;
}
/** @brief Append coherent feedback and form a nearest 32..40ms window, retaining both arrival endpoints.
 * @note Stop-time speed diagnostics do not block later independent baseline stopped evidence.
 */
static ShaftPilotFault update_history(ShaftPilot *pilot, const ShaftPilotInput *input,
 int *new_sample, int *continuous)
{
 ShaftPilotSample sample; ShaftPilotFault fault=SHAFT_FAULT_NONE;
 uint64_t closest_age=UINT64_MAX; unsigned offset, selected=0U;
 *new_sample=0; *continuous=1;
 sample.rx_us=input->feedback.rx_us; sample.observed_us=input->feedback.observed_us;
 sample.shaft_position_rad=input->feedback.native_position_rad/pilot->config.gear_ratio;
 if(!finite_scalar(sample.shaft_position_rad)) return SHAFT_FAULT_POSITION;
 if(pilot->feedback_seen) {
  if(sample.rx_us<pilot->previous_feedback.rx_us ||
     sample.observed_us<pilot->previous_feedback.observed_us) {
   reset_history(pilot); *continuous=0; return SHAFT_FAULT_FEEDBACK;
  }
  if(sample.rx_us==pilot->previous_feedback.rx_us) {
   if(!unchanged_sample(&input->feedback,&pilot->previous_feedback)) {
    reset_history(pilot); *continuous=0; return SHAFT_FAULT_FEEDBACK;
   }
   return SHAFT_FAULT_NONE;
  }
  if(sample.rx_us-pilot->previous_feedback.rx_us>8000U) {
   reset_history(pilot); *continuous=0; fault=SHAFT_FAULT_FEEDBACK;
  }
 }
 *new_sample=1;
 if(pilot->history_count) {
  unsigned previous=(pilot->history_next+SHAFT_PILOT_HISTORY_CAPACITY-1U)%SHAFT_PILOT_HISTORY_CAPACITY;
  uint64_t minimum_us, maximum_us;
  float displacement=fabsf(sample.shaft_position_rad-pilot->history[previous].shaft_position_rad);
  if(!arrival_interval(&pilot->history[previous],&sample,&minimum_us,&maximum_us)) {
   reset_history(pilot); *continuous=0; fault=SHAFT_FAULT_FEEDBACK;
   } else if(displacement>velocity_budget(pilot)*((float)maximum_us*1.0e-6F)+2.0F*pilot->status.quantum_shaft_rad)
   fault=SHAFT_FAULT_POSITION;
  /* Adjacent minimum may be zero; do not divide by it or claim a physical velocity bound. */
 }
 pilot->status.velocity_ready=0U;
 for(offset=0U;offset<pilot->history_count;++offset) {
  unsigned historical=(pilot->history_next+SHAFT_PILOT_HISTORY_CAPACITY-1U-offset)%SHAFT_PILOT_HISTORY_CAPACITY;
  uint64_t age=sample.rx_us-pilot->history[historical].rx_us;
  if(age>=32000U && age<=40000U && age<closest_age) { closest_age=age; selected=historical; }
 }
 if(closest_age!=UINT64_MAX) {
  const ShaftPilotSample *old_sample=&pilot->history[selected];
  float displacement=sample.shaft_position_rad-old_sample->shaft_position_rad;
  pilot->status.velocity_rx32_rad_s=displacement/((float)closest_age*1.0e-6F);
  pilot->status.window_rx_old_us=old_sample->rx_us;
  pilot->status.window_observed_old_us=old_sample->observed_us;
  pilot->status.window_rx_new_us=sample.rx_us;
  pilot->status.window_observed_new_us=sample.observed_us;
  if(!arrival_interval(old_sample,&sample,&pilot->status.window_tmin_us,&pilot->status.window_tmax_us) ||
     !pilot->status.window_tmin_us || !finite_scalar(pilot->status.velocity_rx32_rad_s)) {
   fault=SHAFT_FAULT_FEEDBACK;
  } else {
   pilot->status.velocity_ready=1U;
    if(fabsf(pilot->status.velocity_rx32_rad_s)>velocity_budget(pilot) ||
      fabsf(displacement)+2.0F*pilot->status.quantum_shaft_rad>
        velocity_budget(pilot)*((float)pilot->status.window_tmin_us*1.0e-6F))
    fault=SHAFT_FAULT_SPEED;
  }
 }
 pilot->history[pilot->history_next]=sample;
 pilot->history_next=(uint8_t)((pilot->history_next+1U)%SHAFT_PILOT_HISTORY_CAPACITY);
 if(pilot->history_count<SHAFT_PILOT_HISTORY_CAPACITY) ++pilot->history_count;
 pilot->previous_feedback=input->feedback; pilot->feedback_seen=1U;
 return fault;
}
/** @brief Require a successful receipt matching action id, kind, issue clock and decoded torque bounds. */
static ShaftPilotFault consume_receipt(ShaftPilot *pilot, const ShaftPilotInput *input)
{
 const ShaftPilotReceipt *receipt=&input->receipt;
 const ShaftPilotOutput *pending=&pilot->pending;
 if(pending->action==SHAFT_ACTION_NONE) return SHAFT_FAULT_NONE;
 if(!receipt->valid || !receipt->success || receipt->kind!=pending->action ||
    receipt->command_id!=pending->command_id || receipt->completed_observed_us<pending->issued_us ||
    receipt->completed_observed_us>input->now_us || !finite_scalar(receipt->decoded_torque_nm))
  return SHAFT_FAULT_RECEIPT;
 if(pending->action==SHAFT_ACTION_TORQUE) {
  /* A matched actual violating send remains evidence; it cannot become a legal next-ramp base. */
  pilot->status.last_successful_decoded_torque_nm=receipt->decoded_torque_nm;
  pilot->status.peak_successful_decoded_torque_nm=fmaxf(pilot->status.peak_successful_decoded_torque_nm,
   fabsf(receipt->decoded_torque_nm));
  if(fabsf(receipt->decoded_torque_nm)>pilot->plan.cap_nm ||
     fabsf(receipt->decoded_torque_nm-pending->previous_sent_nm)>pending->maximum_step_nm)
   return SHAFT_FAULT_RECEIPT;
  pilot->last_sent_nm=receipt->decoded_torque_nm; pilot->torque_seen=1U;
  if((pilot->low_profile || pilot->visible_profile || pilot->observe_profile) && !pilot->plateau_seen && pilot->status.phase==SHAFT_PHASE_RAMP &&
     (float)pilot->plan.direction*receipt->decoded_torque_nm>0.0F &&
     pilot->plan.cap_nm-(float)pilot->plan.direction*receipt->decoded_torque_nm<
      pilot->status.quantum_torque_nm) {
   /* Only the first matched legal completion can anchor the finite inward-grid platform. */
   pilot->plateau_seen=1U; pilot->plateau_observed_us=receipt->completed_observed_us;
  }
 }
 if(pending->action==SHAFT_ACTION_DISABLE && receipt->completed_observed_us>=pilot->stop_us)
  pilot->disable_confirmed=1U;
 return SHAFT_FAULT_NONE;
}
/** @brief Fill one bounded action; the receipt id remains unique within this caller-owned instance. */
static void issue_action(ShaftPilot *pilot, const ShaftPilotInput *input, ShaftPilotOutput *output,
 ShaftPilotAction action, float requested_nm, float maximum_step_nm)
{
 memset(output,0,sizeof(*output));
 output->action=action; output->motor_id=7U; output->binding_id=pilot->config.binding_id;
 output->session_id=pilot->config.session_id; output->issued_us=input->now_us;
 output->cap_nm=pilot->plan.cap_nm; output->previous_sent_nm=pilot->last_sent_nm;
 output->maximum_step_nm=maximum_step_nm; output->torque_nm=requested_nm;
 output->previous_sent_valid=pilot->torque_seen;
 if(action!=SHAFT_ACTION_NONE) output->command_id=++pilot->command_sequence;
 pilot->pending=*output;
}
/** @brief Append raw evidence once, then freeze the first terminal record; overflow locks and disables. */
static void append_trace(ShaftPilot *pilot, const ShaftPilotInput *input, ShaftPilotOutput *output)
{
 ShaftPilotTrace *record;
 if(pilot->trace_frozen) return;
 if(pilot->status.trace_count>=SHAFT_PILOT_TRACE_CAPACITY) {
  latch_fault(pilot,SHAFT_FAULT_TRACE); pilot->status.phase=SHAFT_PHASE_LOCKED;
  pilot->trace_frozen=1U;
  issue_action(pilot,input,output,SHAFT_ACTION_DISABLE,0.0F,0.0F); return;
 }
 record=&pilot->trace[pilot->status.trace_count++];
 record->input=*input; record->output=*output; record->phase=pilot->status.phase;
 record->outcome=pilot->status.outcome; record->first_fault=pilot->status.first_fault;
 record->shaft_position_rad=input->feedback.native_position_rad/pilot->config.gear_ratio;
 record->velocity_rx32_rad_s=pilot->status.velocity_rx32_rad_s;
 record->velocity_ready=pilot->status.velocity_ready;
 record->arrival_interval_conditional=1U; record->internal_sampling_unknown=1U;
 record->window_rx_old_us=pilot->status.window_rx_old_us;
 record->window_observed_old_us=pilot->status.window_observed_old_us;
 record->window_rx_new_us=pilot->status.window_rx_new_us;
 record->window_observed_new_us=pilot->status.window_observed_new_us;
 record->window_tmin_us=pilot->status.window_tmin_us;
 record->window_tmax_us=pilot->status.window_tmax_us;
 if(pilot->status.phase==SHAFT_PHASE_DONE || pilot->status.phase==SHAFT_PHASE_LOCKED)
  pilot->trace_frozen=1U;
}
/** @brief Validate frozen ranges and measured baseline before initializing any transaction state. */
int shaft_pilot_init(ShaftPilot *pilot, const ShaftPilotConfig *config, const ShaftPilotPlan *plan)
{
 float native_quantum, shaft_quantum, velocity_quantum, torque_quantum, zero_center;
 if(!pilot || !config || !plan) return 0;
 memset(pilot,0,sizeof(*pilot));
 if(config->motor_id!=7U || !config->binding_id || !config->session_id ||
    config->baseline.binding_id!=config->binding_id || config->baseline.session_id!=config->session_id ||
    config->baseline.point_count!=64U || !positive_scalar(config->gear_ratio) ||
    !positive_scalar(config->position_max_native_rad) || !positive_scalar(config->velocity_max_rad_s) ||
    !positive_scalar(config->torque_max_nm) ||
    !positive_scalar((float)((double)config->position_max_native_rad/(double)config->gear_ratio)) ||
    !finite_scalar(config->baseline.span64_shaft_rad) || config->baseline.span64_shaft_rad<0.0F ||
    !finite_scalar(config->baseline.velocity32_abs_peak_rad_s) ||
    config->baseline.velocity32_abs_peak_rad_s<0.0F || config->baseline.velocity32_abs_peak_rad_s>0.3F ||
    !finite_scalar(config->baseline.torque_abs_peak_nm) || config->baseline.torque_abs_peak_nm<0.0F ||
    config->baseline.torque_abs_peak_nm>0.15F || !finite_scalar(config->baseline.raw_zero_center_rad_s) ||
    !positive_scalar(plan->cap_nm) || plan->cap_nm>1.75F || plan->cap_nm>config->torque_max_nm ||
    !positive_scalar(plan->slew_nm_s) || plan->slew_nm_s>4.0F ||
    !positive_scalar(plan->step_nm) || plan->step_nm>0.016F ||
    (plan->direction!=1 && plan->direction!=-1)) return 0;
 native_quantum=(float)(2.0*(double)config->position_max_native_rad/65535.0);
 shaft_quantum=(float)((double)native_quantum/(double)config->gear_ratio);
 velocity_quantum=(float)(2.0*(double)config->velocity_max_rad_s/4095.0);
 torque_quantum=(float)(2.0*(double)config->torque_max_nm/4095.0);
 zero_center=fabsf(config->baseline.raw_zero_center_rad_s);
 if(!positive_scalar(native_quantum) || !positive_scalar(shaft_quantum) ||
    !positive_scalar(velocity_quantum) || !positive_scalar(torque_quantum) || zero_center>0.05F ||
    /* Mid-code decoding subtracts VMAX: its float cancellation uses the range magnitude.
     * The actual captured center still defines raw_still(), with no neighboring-code tolerance. */
    fabsf(zero_center-velocity_quantum*0.5F)>rounding_allowance(config->velocity_max_rad_s)) return 0;
 pilot->config=*config; pilot->plan=*plan;
 pilot->status.quantum_native_rad=native_quantum; pilot->status.quantum_shaft_rad=shaft_quantum;
 pilot->status.quantum_velocity_rad_s=velocity_quantum; pilot->status.quantum_torque_nm=torque_quantum;
 pilot->status.initial_stop_torque_gate_nm=config->baseline.torque_abs_peak_nm+2.0F*torque_quantum;
 pilot->status.running_torque_gate_nm=plan->cap_nm+pilot->status.initial_stop_torque_gate_nm;
 if(!finite_scalar(pilot->status.running_torque_gate_nm) ||
    !finite_scalar(config->baseline.span64_shaft_rad+2.0F*shaft_quantum) ||
    !finite_scalar(config->baseline.velocity32_abs_peak_rad_s+2.0F*shaft_quantum/0.032F)) return 0;
 pilot->status.arrival_interval_conditional=1U; pilot->status.internal_sampling_unknown=1U;
 pilot->initialized=1U; return 1;
}
/** @brief Freeze the sole low diagnostic plan without accepting caller amplitude/slew/step claims. */
int shaft_pilot_init_low(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction)
{
 ShaftPilotPlan plan;
 plan.direction=direction; plan.cap_nm=0.35F; plan.slew_nm_s=4.0F; plan.step_nm=0.016F;
 if(!shaft_pilot_init(pilot,config,&plan)) return 0;
 if(!finite_scalar(config->baseline.span64_shaft_rad+6.0F*pilot->status.quantum_shaft_rad)) {
  pilot->initialized=0U; return 0;
 }
 pilot->low_profile=1U; return 1;
}
/** @brief Freeze only the lower-energy low-speed plan; never reinterpret the existing low profile. */
int shaft_pilot_init_fine(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction)
{
 ShaftPilotPlan plan;
 if(!config || config->velocity_max_rad_s!=5.0F) {
  if(pilot) memset(pilot,0,sizeof(*pilot));
  return 0;
 }
 plan.direction=direction; plan.cap_nm=0.20F; plan.slew_nm_s=4.0F; plan.step_nm=0.016F;
 if(!shaft_pilot_init(pilot,config,&plan)) return 0;
 if(!finite_scalar(config->baseline.span64_shaft_rad+6.0F*pilot->status.quantum_shaft_rad)) {
  pilot->initialized=0U; return 0;
 }
 pilot->low_profile=1U; return 1;
}
/** @brief Freeze an exclusive visible plan; USB never supplies cap, slew, angle, hold or deadline values. */
int shaft_pilot_init_visible(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction)
{
 ShaftPilotPlan plan;
 plan.direction=direction; plan.cap_nm=0.5F; plan.slew_nm_s=2.0F; plan.step_nm=0.008F;
 if(!shaft_pilot_init(pilot,config,&plan)) return 0;
 if(!finite_scalar(config->baseline.span64_shaft_rad+6.0F*pilot->status.quantum_shaft_rad+0.0012F) ||
    !positive_scalar(0.15F/config->gear_ratio)) { pilot->initialized=0U; return 0; }
 pilot->visible_profile=1U; return 1;
}
/** @brief Freeze the approved rated-half observation; narrow feedback ranges cannot represent its speed gate. */
int shaft_pilot_init_observe(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction)
{
 ShaftPilotPlan plan;
 if(!config || config->position_max_native_rad!=12.5F || config->velocity_max_rad_s!=200.0F ||
    config->torque_max_nm!=10.0F) {
  if(pilot) memset(pilot,0,sizeof(*pilot));
  return 0;
 }
 plan.direction=direction; plan.cap_nm=1.75F; plan.slew_nm_s=4.0F; plan.step_nm=0.016F;
 if(!shaft_pilot_init(pilot,config,&plan)) return 0;
 pilot->observe_profile=1U; return 1;
}
/** @brief Select a separate fixed range-bounded plateau while preserving the historical observe identity. */
int shaft_pilot_init_platform(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction)
{
 if(!shaft_pilot_init_observe(pilot,config,direction)) return 0;
 pilot->platform_profile=1U; return 1;
}
/** @brief 新宽档只改变固定PMAX及停止/入口余量，沿用已有observe平台约束。 */
int shaft_pilot_init_platform_wide(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction)
{
 ShaftPilotPlan plan;
 if(!config || config->position_max_native_rad!=512.0F || config->velocity_max_rad_s!=200.0F ||
    config->torque_max_nm!=10.0F) {
  if(pilot) memset(pilot,0,sizeof(*pilot));
  return 0;
 }
 plan.direction=direction; plan.cap_nm=1.75F; plan.slew_nm_s=4.0F; plan.step_nm=0.016F;
 if(!shaft_pilot_init(pilot,config,&plan)) return 0;
 pilot->observe_profile=1U; pilot->platform_profile=1U; pilot->platform_wide_profile=1U; return 1;
}
/** @brief 独立70%档只接受固定1024/200/10；基础初始化统一清除所有旧状态和档位旗标。 */
int shaft_pilot_init_platform70(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction)
{
 ShaftPilotPlan plan;
 if(!config || config->position_max_native_rad!=1024.0F || config->velocity_max_rad_s!=200.0F ||
    config->torque_max_nm!=10.0F) {
  if(pilot) memset(pilot,0,sizeof(*pilot));
  return 0;
 }
 plan.direction=direction; plan.cap_nm=1.75F; plan.slew_nm_s=4.0F; plan.step_nm=0.016F;
 if(!shaft_pilot_init(pilot,config,&plan)) return 0;
 pilot->observe_profile=1U; pilot->platform_profile=1U; pilot->platform70_profile=1U; return 1;
}
/** @brief 独立100%档只接受固定1024/200/10；基础初始化统一清除所有旧状态和档位旗标。 */
int shaft_pilot_init_platform100(ShaftPilot *pilot, const ShaftPilotConfig *config, int direction)
{
 ShaftPilotPlan plan;
 if(!config || config->position_max_native_rad!=1024.0F || config->velocity_max_rad_s!=200.0F ||
    config->torque_max_nm!=10.0F) {
  if(pilot) memset(pilot,0,sizeof(*pilot));
  return 0;
 }
 plan.direction=direction; plan.cap_nm=1.0F; plan.slew_nm_s=4.0F; plan.step_nm=0.016F;
 if(!shaft_pilot_init(pilot,config,&plan)) return 0;
 pilot->observe_profile=1U; pilot->platform_profile=1U; pilot->platform100_profile=1U; return 1;
}
/** @brief Accept fresh bound healthy disabled entry with native PMAX headroom and a live independent guard. */
int shaft_pilot_start(ShaftPilot *pilot, const ShaftPilotInput *input, ShaftPilotOutput *output)
{
 double headroom; int new_sample, continuous; float shaft_position;
 if(!output) return 0;
 memset(output,0,sizeof(*output));
 if(!pilot || !input || !pilot->initialized || pilot->status.phase!=SHAFT_PHASE_IDLE ||
    binding_health(pilot,input)!=SHAFT_FAULT_NONE || !input->guard_armed || !input->guard_live ||
    input->stop_requested ||
    feedback_health(pilot,input,pilot->status.initial_stop_torque_gate_nm)!=SHAFT_FAULT_NONE ||
    input->feedback.driver_state!=SHAFT_DRIVER_DISABLED || !raw_still(pilot,input->feedback.raw_velocity_rad_s))
  return 0;
 headroom=(pilot->platform100_profile?(double)SHAFT_PLATFORM100_RESERVE_NATIVE_RAD:
  (pilot->platform70_profile?(double)SHAFT_PLATFORM70_RESERVE_NATIVE_RAD:
  (pilot->platform_wide_profile?(double)SHAFT_PLATFORM_WIDE_RESERVE_NATIVE_RAD:
  (pilot->observe_profile?(double)SHAFT_OBSERVE_STOP_NATIVE_RAD:
  0.45*(double)pilot->config.gear_ratio))))+2.0*(double)pilot->status.quantum_native_rad;
 if(headroom>(double)pilot->config.position_max_native_rad ||
    fabs((double)input->feedback.native_position_rad)+headroom>
     (double)pilot->config.position_max_native_rad) return 0;
 /* 入口本方向预算已包含25ms；不能在当前raw速度项之外重复加第二次25ms。 */
 if((pilot->platform_wide_profile || pilot->platform70_profile || pilot->platform100_profile) && (double)pilot->config.position_max_native_rad-
    (double)pilot->plan.direction*(double)input->feedback.native_position_rad<
    headroom+(double)velocity_budget(pilot)*(double)pilot->config.gear_ratio*
     ((double)(pilot->platform100_profile?SHAFT_PLATFORM100_ENTRY_US:
       (pilot->platform70_profile?SHAFT_PLATFORM70_ENTRY_US:SHAFT_PLATFORM_WIDE_ENTRY_US))*1.0e-6)) return 0;
 if(pilot->platform_profile && platform_range_exhausted(pilot,input)) return 0;
 shaft_position=input->feedback.native_position_rad/pilot->config.gear_ratio;
 if(!finite_scalar(shaft_position)) return 0;
 if(update_history(pilot,input,&new_sample,&continuous)!=SHAFT_FAULT_NONE) {
  reset_history(pilot); pilot->feedback_seen=0U; return 0;
 }
 pilot->origin_shaft_rad=shaft_position;
 pilot->origin_native_rad=input->feedback.native_position_rad;
 pilot->started_us=input->now_us; pilot->last_tick_us=input->now_us; pilot->phase_us=input->now_us;
 pilot->status.phase=SHAFT_PHASE_ENABLE;
 issue_action(pilot,input,output,SHAFT_ACTION_ENABLE,0.0F,0.0F);
 append_trace(pilot,input,output); return 1;
}
/** @brief Track baseline-derived stable disabled dwell independently from historical trial faults. */
static void stopped_evidence(ShaftPilot *pilot, const ShaftPilotInput *input, int new_sample,
 int continuous, ShaftPilotFault basic_fault, ShaftPilotFault receipt_fault)
{
 float position=input->feedback.native_position_rad/pilot->config.gear_ratio;
 float velocity_gate=pilot->config.baseline.velocity32_abs_peak_rad_s+
  2.0F*pilot->status.quantum_shaft_rad/0.032F;
 float span_gate=pilot->config.baseline.span64_shaft_rad+2.0F*pilot->status.quantum_shaft_rad;
 int healthy=basic_fault==SHAFT_FAULT_NONE && receipt_fault==SHAFT_FAULT_NONE && continuous &&
  input->feedback.driver_state==SHAFT_DRIVER_DISABLED && pilot->disable_confirmed &&
  input->feedback.rx_us>pilot->stop_us && raw_still(pilot,input->feedback.raw_velocity_rad_s) &&
  fabsf(input->feedback.torque_nm)<=pilot->status.initial_stop_torque_gate_nm &&
  pilot->status.velocity_ready && fabsf(pilot->status.velocity_rx32_rad_s)<=velocity_gate;
 if(!healthy) {
  pilot->dwell_tracking=0U; pilot->status.safe_release=0U;
  pilot->status.caller_next_transaction_ready=0U; return;
 }
 if(new_sample) {
  if(!pilot->dwell_tracking) {
   pilot->dwell_tracking=1U; pilot->dwell_us=input->feedback.rx_us;
   pilot->dwell_min_rad=position; pilot->dwell_max_rad=position;
  } else {
   if(position<pilot->dwell_min_rad) pilot->dwell_min_rad=position;
   if(position>pilot->dwell_max_rad) pilot->dwell_max_rad=position;
   if(pilot->dwell_max_rad-pilot->dwell_min_rad>span_gate) {
    pilot->dwell_us=input->feedback.rx_us; pilot->dwell_min_rad=position; pilot->dwell_max_rad=position;
    pilot->status.safe_release=0U;
   }
  }
  if(input->feedback.rx_us-pilot->dwell_us>=64000U) pilot->status.safe_release=1U;
 }
 if(pilot->status.safe_release && pilot->status.phase==SHAFT_PHASE_STOPPING)
  pilot->status.phase=SHAFT_PHASE_DONE;
 pilot->status.caller_next_transaction_ready=(uint8_t)(pilot->status.safe_release &&
  pilot->status.phase==SHAFT_PHASE_DONE && pilot->status.first_fault==SHAFT_FAULT_NONE &&
  (pilot->status.outcome==SHAFT_END_TARGET || pilot->status.outcome==SHAFT_END_TIME) &&
   ((!pilot->low_profile && !pilot->visible_profile && !pilot->observe_profile) ||
   (pilot->visible_profile && pilot->status.outcome==SHAFT_END_TARGET) ||
    (pilot->observe_profile && !pilot->platform_profile && pilot->status.outcome==SHAFT_END_TARGET) ||
   (pilot->plateau_seen && input->now_us>=pilot->plateau_observed_us &&
     input->now_us-pilot->plateau_observed_us>=
      (pilot->observe_profile?SHAFT_OBSERVE_PLATEAU_US:(pilot->visible_profile?200000U:80000U)))) &&
  input->guard_fired && input->guard_acked);
}
/** @brief Recover the original 16-bit code; float decoding error is far below half a code. */
static int32_t native_position_code(const ShaftPilot *pilot, double native_position)
{
 double maximum=(double)pilot->config.position_max_native_rad;
 double code=floor((native_position+maximum)*65535.0/(2.0*maximum)+0.5);
 if(code<0.0) return 0;
 if(code>65535.0) return 65535;
 return (int32_t)code;
}
/** @brief Revoke low diagnostics on strict raw magnitude or nearest causal 8..12ms displacement.
 * @note This baseline-derived early exit is an observation, not a physical velocity/safety guarantee.
 *       Existing integrity/health/protection gates run first and preserve their own first fault.
 */
static int low_early_abort(const ShaftPilot *pilot, const ShaftPilotInput *input, int new_sample)
{
 uint64_t closest_age=UINT64_MAX; unsigned offset, selected=0U;
 double baseline_codes=(double)pilot->config.baseline.span64_shaft_rad/(double)pilot->status.quantum_shaft_rad;
 double rounded_baseline=floor(baseline_codes+0.5);
 double displacement_gate_codes;
 if(!new_sample) return 0;
 if(fabsf(input->feedback.raw_velocity_rad_s)>0.15F) return 1;
 for(offset=0U;offset<pilot->history_count;++offset) {
  unsigned historical=(pilot->history_next+SHAFT_PILOT_HISTORY_CAPACITY-1U-offset)%SHAFT_PILOT_HISTORY_CAPACITY;
  uint64_t age=input->feedback.rx_us-pilot->history[historical].rx_us;
  if(age>=8000U && age<=12000U && age<closest_age) { closest_age=age; selected=historical; }
 }
 /* The trusted baseline is a span of quantized native positions. Two float
  * decodes/divisions and span subtraction can move an integer boundary slightly;
  * 8*epsilon*65535 bounds that arithmetic, and is less than 1/16 of one code.
  * Only snap such near-integer spans. Other caller spans round down, never up.
  * Recovering original codes avoids subtracting cancellation-affected SI floats.
  * No neighboring position code is added to either physical observation gate. */
 if(fabs(baseline_codes-rounded_baseline)>8.0*(double)FLT_EPSILON*65535.0)
  rounded_baseline=floor(baseline_codes);
 displacement_gate_codes=rounded_baseline+6.0;
 /* Visible uses an explicit observation-speed envelope instead of the micro-motion-only gate.
  * This is arrival-time arithmetic, never an asserted physical sample-speed upper bound. */
 if(pilot->visible_profile && closest_age!=UINT64_MAX)
  displacement_gate_codes+=0.10*(double)closest_age*1.0e-6/(double)pilot->status.quantum_shaft_rad;
 if(closest_age!=UINT64_MAX) {
  int32_t newest=native_position_code(pilot,(double)input->feedback.native_position_rad);
  int32_t oldest=native_position_code(pilot,(double)pilot->history[selected].shaft_position_rad*
   (double)pilot->config.gear_ratio);
  int32_t difference=newest-oldest;
  if(difference<0) difference=-difference;
  return (double)difference>displacement_gate_codes;
 }
 return 0;
}
/** @brief Advance one transaction with protection first and at most one ENABLE/TORQUE/DISABLE action. */
void shaft_pilot_tick(ShaftPilot *pilot, const ShaftPilotInput *input, ShaftPilotOutput *output)
{
 ShaftPilotFault basic_fault, history_fault=SHAFT_FAULT_NONE, receipt_fault, fault;
 uint64_t period_us=0U, age_us; int new_sample=0, continuous=1, was_active;
 float position, maximum_step, target;
 if(!output) return;
 memset(output,0,sizeof(*output));
 if(!pilot || !input || !pilot->initialized || pilot->status.phase==SHAFT_PHASE_IDLE) return;
 was_active=active_phase(pilot->status.phase);
 receipt_fault=consume_receipt(pilot,input);
 basic_fault=binding_health(pilot,input);
 if(basic_fault==SHAFT_FAULT_NONE)
  basic_fault=feedback_health(pilot,input,pilot->status.running_torque_gate_nm);
 fault=basic_fault;
 if(input->now_us<=pilot->last_tick_us || input->now_us<pilot->started_us)
  fault=SHAFT_FAULT_PERIOD;
 else period_us=input->now_us-pilot->last_tick_us;
 if(basic_fault==SHAFT_FAULT_NONE) {
  history_fault=update_history(pilot,input,&new_sample,&continuous);
  if(fault==SHAFT_FAULT_NONE) fault=history_fault;
 } else { reset_history(pilot); continuous=0; }
 if(fault==SHAFT_FAULT_NONE && fabsf(input->feedback.raw_velocity_rad_s)>velocity_budget(pilot))
  fault=SHAFT_FAULT_SPEED;
 position=input->feedback.native_position_rad/pilot->config.gear_ratio;
 if(fault==SHAFT_FAULT_NONE) {
  if(pilot->platform_profile) {
   if(fabsf(input->feedback.native_position_rad)>=pilot->config.position_max_native_rad-
      2.0F*pilot->status.quantum_native_rad) fault=SHAFT_FAULT_TRAVEL;
  } else if(pilot->observe_profile) {
   if(fabsf(input->feedback.native_position_rad-pilot->origin_native_rad)>
      (was_active?SHAFT_OBSERVE_ACTIVE_NATIVE_RAD:SHAFT_OBSERVE_STOP_NATIVE_RAD)) fault=SHAFT_FAULT_TRAVEL;
  } else if(fabsf(position-pilot->origin_shaft_rad)>(was_active?0.2799F:0.45F)) fault=SHAFT_FAULT_TRAVEL;
 }
 if(fault==SHAFT_FAULT_NONE) fault=receipt_fault;
 if(was_active && fault==SHAFT_FAULT_NONE && (!input->guard_armed || !input->guard_live))
  fault=SHAFT_FAULT_GUARD;
 if(was_active && fault==SHAFT_FAULT_NONE && (period_us<3000U || period_us>5000U))
  fault=SHAFT_FAULT_PERIOD;
 if(was_active && fault==SHAFT_FAULT_NONE && input->now_us-pilot->started_us>=1400000U)
  fault=SHAFT_FAULT_ACTIVE_TIMEOUT;
 if(!pilot->trace_frozen && pilot->status.trace_count>=SHAFT_PILOT_TRACE_CAPACITY) {
  if(fault==SHAFT_FAULT_NONE) fault=SHAFT_FAULT_TRACE;
  begin_stop(pilot,SHAFT_END_FAULT,input->now_us);
  pilot->status.phase=SHAFT_PHASE_LOCKED; pilot->trace_frozen=1U;
 }
 latch_fault(pilot,fault);
 if(was_active && fault!=SHAFT_FAULT_NONE) begin_stop(pilot,SHAFT_END_FAULT,input->now_us);
 if(was_active && input->stop_requested) begin_stop(pilot,SHAFT_END_ABORT,input->now_us);
 /* Phase health/deadlines outrank the platform's normal range ABORT in the same observation. */
 if(pilot->status.phase==SHAFT_PHASE_ENABLE && input->now_us-pilot->phase_us>=64000U) {
  latch_fault(pilot,SHAFT_FAULT_ENABLE_TIMEOUT); begin_stop(pilot,SHAFT_END_FAULT,input->now_us);
 }
 if(active_phase(pilot->status.phase) && pilot->status.phase!=SHAFT_PHASE_ENABLE &&
    input->feedback.driver_state!=SHAFT_DRIVER_ENABLED) {
  latch_fault(pilot,SHAFT_FAULT_DRIVER); begin_stop(pilot,SHAFT_END_FAULT,input->now_us);
 }
 if(pilot->status.phase==SHAFT_PHASE_NEUTRAL && input->now_us-pilot->phase_us>=69000U) {
  latch_fault(pilot,SHAFT_FAULT_NEUTRAL_TIMEOUT); begin_stop(pilot,SHAFT_END_FAULT,input->now_us);
 }
 if(active_phase(pilot->status.phase) && pilot->platform_profile && new_sample &&
    platform_range_exhausted(pilot,input)) begin_stop(pilot,SHAFT_END_ABORT,input->now_us);
 if(input->now_us>pilot->last_tick_us) pilot->last_tick_us=input->now_us;
 if(pilot->status.phase==SHAFT_PHASE_ENABLE) {
  if(new_sample && input->feedback.driver_state==SHAFT_DRIVER_ENABLED) {
   pilot->status.phase=SHAFT_PHASE_NEUTRAL; pilot->phase_us=input->now_us;
  } else {
   issue_action(pilot,input,output,SHAFT_ACTION_ENABLE,0.0F,0.0F); append_trace(pilot,input,output); return;
  }
 }
 if(pilot->status.phase==SHAFT_PHASE_NEUTRAL) {
  age_us=input->now_us-pilot->phase_us;
  /* 64ms dwell plus at most one 5ms scheduling observation margin, never a pulse extension. */
  if(age_us>=64000U && new_sample && pilot->status.velocity_ready && pilot->torque_seen &&
     raw_still(pilot,input->feedback.raw_velocity_rad_s) &&
     fabsf(input->feedback.torque_nm)<=pilot->status.initial_stop_torque_gate_nm) {
   pilot->status.phase=SHAFT_PHASE_RAMP; pilot->phase_us=input->now_us;
  }
 }
 if(pilot->status.phase==SHAFT_PHASE_RAMP) {
  if(pilot->observe_profile) {
   float directed_native=(float)pilot->plan.direction*(input->feedback.native_position_rad-pilot->origin_native_rad);
   if(input->now_us-pilot->phase_us>=SHAFT_OBSERVE_RAMP_US ||
      (new_sample && directed_native<=-SHAFT_OBSERVE_REVERSE_NATIVE_RAD))
    begin_stop(pilot,SHAFT_END_ABORT,input->now_us);
   else if(!pilot->platform_profile && new_sample && directed_native>=SHAFT_OBSERVE_ANGLE_NATIVE_RAD)
    begin_stop(pilot,SHAFT_END_TARGET,input->now_us);
   else if(pilot->plateau_seen && input->now_us>=pilot->plateau_observed_us &&
      input->now_us-pilot->plateau_observed_us>=SHAFT_OBSERVE_PLATEAU_US)
    begin_stop(pilot,SHAFT_END_TIME,input->now_us);
  } else if(pilot->visible_profile) {
   if(low_early_abort(pilot,input,new_sample) || input->now_us-pilot->phase_us>=800000U ||
      (new_sample && (float)pilot->plan.direction*(position-pilot->origin_shaft_rad)<=-0.15F/pilot->config.gear_ratio))
    begin_stop(pilot,SHAFT_END_ABORT,input->now_us);
   else if(new_sample && (float)pilot->plan.direction*(position-pilot->origin_shaft_rad)>=0.15F/pilot->config.gear_ratio)
    begin_stop(pilot,SHAFT_END_TARGET,input->now_us);
   else if(pilot->plateau_seen && input->now_us>=pilot->plateau_observed_us &&
      input->now_us-pilot->plateau_observed_us>=200000U)
    begin_stop(pilot,SHAFT_END_TIME,input->now_us);
  } else if(pilot->low_profile) {
   if(low_early_abort(pilot,input,new_sample))
    begin_stop(pilot,SHAFT_END_ABORT,input->now_us);
   else if(pilot->plateau_seen && input->now_us>=pilot->plateau_observed_us &&
      input->now_us-pilot->plateau_observed_us>=80000U)
    begin_stop(pilot,SHAFT_END_TIME,input->now_us);
   else if(input->now_us-pilot->phase_us>=800000U)
    begin_stop(pilot,SHAFT_END_ABORT,input->now_us);
  } else if(new_sample && pilot->status.velocity_ready &&
     (float)pilot->plan.direction*pilot->status.velocity_rx32_rad_s>=0.1F)
   begin_stop(pilot,SHAFT_END_TARGET,input->now_us);
  else if(input->now_us-pilot->phase_us>=800000U)
   begin_stop(pilot,SHAFT_END_TIME,input->now_us);
 }
 if(!active_phase(pilot->status.phase)) {
  /* History speed/travel diagnostics remain latched; healthy baseline stopping may recover independently. */
  if(pilot->status.phase==SHAFT_PHASE_STOPPING && !pilot->status.safe_release &&
     input->now_us>=pilot->started_us && input->now_us-pilot->started_us>=1500000U) {
   /* A late tick may recover safe_release, but cannot retrospectively beat the absolute deadline. */
   latch_fault(pilot,SHAFT_FAULT_STOP_TIMEOUT); pilot->status.phase=SHAFT_PHASE_LOCKED;
  }
  stopped_evidence(pilot,input,new_sample,continuous,basic_fault,receipt_fault);
  if(pilot->status.phase==SHAFT_PHASE_DONE && !pilot->status.safe_release)
   pilot->status.phase=SHAFT_PHASE_STOPPING;
  if(!pilot->status.safe_release && input->now_us>=pilot->started_us &&
     input->now_us-pilot->started_us>=1500000U) {
   latch_fault(pilot,SHAFT_FAULT_STOP_TIMEOUT); pilot->status.phase=SHAFT_PHASE_LOCKED;
  }
  issue_action(pilot,input,output,SHAFT_ACTION_DISABLE,0.0F,0.0F);
 } else {
  maximum_step=fminf(pilot->plan.slew_nm_s*((float)period_us*1.0e-6F),pilot->plan.step_nm);
  target=pilot->status.phase==SHAFT_PHASE_RAMP?(float)pilot->plan.direction*pilot->plan.cap_nm:0.0F;
  target=fmaxf(pilot->last_sent_nm-maximum_step,fminf(pilot->last_sent_nm+maximum_step,target));
  target=fmaxf(-pilot->plan.cap_nm,fminf(pilot->plan.cap_nm,target));
  issue_action(pilot,input,output,SHAFT_ACTION_TORQUE,target,maximum_step);
 }
 append_trace(pilot,input,output);
}
/** @brief Read an immutable bounded raw record without changing transaction state. */
int shaft_pilot_trace_get(const ShaftPilot *pilot, uint32_t index, ShaftPilotTrace *record)
{
 if(!pilot || !record || !pilot->initialized || index>=pilot->status.trace_count ||
    index>=SHAFT_PILOT_TRACE_CAPACITY) return 0;
 *record=pilot->trace[index]; return 1;
}
/** @brief Copy current diagnostics and disabled-evidence status. */
int shaft_pilot_status_get(const ShaftPilot *pilot, ShaftPilotStatus *status)
{
 if(!pilot || !status || !pilot->initialized) return 0;
 *status=pilot->status; return 1;
}
