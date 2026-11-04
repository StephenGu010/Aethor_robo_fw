/** @file adrc_simple_trial.c
 * @brief Bounded nominal-command gain trial; receive-time velocity is not a physical sample timestamp.
 */
#include "adrc_simple_trial.h"
#include <math.h>
#include <string.h>

/** @brief Unsigned subtraction supports a single uint64 clock rollover. */
static uint64_t elapsed(uint64_t newer, uint64_t older) { return newer - older; }
/** @brief Validate fresh native feedback, including unknown nonzero driver errors. */
static AdrcExperimentFault health(const AdrcSimpleInput *input, float torque_trip_nm)
{
 const AdrcExperimentInput *feedback=&input->feedback;
 if (!feedback->feedback_valid || feedback->axis_index!=6U ||
     elapsed(feedback->now_us,feedback->feedback_us)>8000U ||
     !isfinite(feedback->position_rad) || !isfinite(feedback->raw_velocity_rad_s) ||
     !isfinite(input->feedback_torque_nm)) return ADRC_FAULT_FEEDBACK;
 if (feedback->driver_fault || fabsf(input->feedback_torque_nm)>torque_trip_nm) return ADRC_FAULT_DRIVER;
 if (!isfinite(feedback->mos_temperature_c) || !isfinite(feedback->rotor_temperature_c) ||
     feedback->mos_temperature_c>=50.0f || feedback->rotor_temperature_c>=50.0f) return ADRC_FAULT_TEMPERATURE;
 if (fabsf(feedback->raw_velocity_rad_s)>0.5f) return ADRC_FAULT_SPEED;
 return ADRC_FAULT_NONE;
}
/** @brief Preserve first failure and initiate immediate DISABLE, never a torque ramp stop. */
static void stop_trial(AdrcSimpleTrial *trial, AdrcSimpleResult result, AdrcExperimentFault fault, uint64_t now)
{
 if (trial->status.phase==ADRC_SIMPLE_STOPPING || trial->status.phase==ADRC_SIMPLE_LOCKED) return;
 trial->status.result=result; trial->status.fault=fault;
 trial->status.phase=ADRC_SIMPLE_STOPPING; trial->stop_us=now;
 trial->dwell_tracking=0U; trial->disable_sent=0U;
}
/** @brief Maintain a 32 ms receive-time position difference using only new feedback. */
static void estimate(AdrcSimpleTrial *trial, const AdrcExperimentInput *feedback)
{
 unsigned index; uint64_t best_age=UINT64_MAX; float old_position=0.0f;
 for(index=0U;index<trial->history_count;++index) {
  uint64_t age=elapsed(feedback->feedback_us,trial->position_times[index]);
  if(age>=32000U && age<=40000U && age<best_age) { best_age=age; old_position=trial->positions[index]; }
 }
 if(best_age!=UINT64_MAX) { trial->velocity_rad_s=(feedback->position_rad-old_position)/(best_age*1.0e-6f);
  trial->estimate_midpoint_us=feedback->feedback_us-best_age/2U; trial->estimate_ready=1U; }
 trial->positions[trial->history_next]=feedback->position_rad;
 trial->position_times[trial->history_next]=feedback->feedback_us;
 trial->history_next=(uint8_t)((trial->history_next+1U)%16U);
 if(trial->history_count<16U) ++trial->history_count;
}
/** @brief Enter a finite phase and reset pulse fitting accumulators. */
static void phase_enter(AdrcSimpleTrial *trial, AdrcSimplePhase phase, uint64_t now)
{
 trial->status.phase=phase; trial->phase_us=now;
 trial->pulse_start_seen=0U; trial->pulse_end_seen=0U; trial->pulse_impulse=0.0f;
 trial->pulse_integral_us=0U;
 trial->pulse_platform_seen=0U; trial->pulse_platform_valid=1U; trial->pulse_platform_reference_nm=0.0f;
}
/** @brief Initialize caller-owned bounded storage without allocating or qualifying anything. */
int adrc_simple_init(AdrcSimpleTrial *trial, AdrcControllerCallback controller, void *context, float quantum)
{
 if(!trial || !controller || !isfinite(quantum) || quantum<=0.0f) return 0;
 memset(trial,0,sizeof(*trial)); trial->controller=controller;
 trial->controller_context=context; trial->position_quantum_rad=quantum;
 return adrc_simple_set_profile(trial,0U);
}
/** @brief Apply fixed running envelopes while preserving strict initial and disabled health. */
int adrc_simple_set_profile(AdrcSimpleTrial *trial, uint8_t profile)
{
 static const float torque_limits_nm[]={0.03f,0.05f,0.08f,0.10f,0.14f,0.20f,0.25f,0.35f,1.75f};
 static const uint32_t pulse_durations_us[]={80000U,160000U,200000U,240000U,360000U,360000U,360000U,360000U,640000U};
 if(!trial || trial->status.phase!=ADRC_SIMPLE_IDLE || profile>=9U) return 0;
 trial->control_reference_rad_s=0.1f;
 trial->profile=profile; trial->torque_limit_nm=torque_limits_nm[profile];
 trial->pulse_duration_us=pulse_durations_us[profile];
 trial->torque_trip_nm=profile==8U?1.85f:(profile==7U?0.40f:(profile==6U?0.28f:(profile==5U?0.23f:0.15f)));
 trial->torque_slew_nm_s=profile>=7U?4.0f:(profile>=5U?3.0f:2.0f);
 trial->torque_step_nm=profile>=7U?0.016f:(profile>=5U?0.012f:0.008f); return 1;
}
/** @brief Start once from fresh disabled native feedback after platform guard arming. */
int adrc_simple_start(AdrcSimpleTrial *trial, const AdrcSimpleInput *input, AdrcExperimentOutput *output)
{
 if(!trial || !input || !output) return 0;
 memset(output,0,sizeof(*output)); output->axis_index=6U;
 /* Native Vmax=200/12-bit feedback represents zero as approximately +/-0.04884 rad/s. */
 if(trial->status.phase!=ADRC_SIMPLE_IDLE || !trial->controller || health(input,0.15f)!=ADRC_FAULT_NONE ||
    input->feedback.enabled || fabsf(input->feedback.raw_velocity_rad_s)>0.05f) return 0;
 trial->started_us=input->feedback.now_us; trial->last_tick_us=input->feedback.now_us;
 trial->last_feedback_us=input->feedback.feedback_us;
 trial->initial_position_rad=input->feedback.position_rad;
 estimate(trial,&input->feedback); phase_enter(trial,ADRC_SIMPLE_ENABLE,input->feedback.now_us);
 output->enable_requested=1U; trial->expected_receipt=1U; trial->controller_reset=1U; return 1;
}
/** @brief Record only valid active torque receipts; bridge calls before any later abort can hide the last send. */
void adrc_simple_observe_receipt(AdrcSimpleTrial *trial, const AdrcSimpleInput *input)
{
 if(!trial || !input || trial->profile!=8U || trial->status.phase<ADRC_SIMPLE_NEUTRAL_1 ||
    trial->status.phase>=ADRC_SIMPLE_STOPPING || trial->expected_receipt!=2U ||
    !input->tx_receipt_valid || input->tx_kind!=1U || input->feedback.prev_send_failed ||
    !input->feedback.prev_sent_valid || !isfinite(input->feedback.prev_sent_nm) ||
    fabsf(input->feedback.prev_sent_nm)>trial->torque_limit_nm) return;
 trial->probe_peak_nm=fmaxf(trial->probe_peak_nm,fabsf(input->feedback.prev_sent_nm));
 trial->probe_reached=(uint8_t)(trial->probe_peak_nm>=0.99f*trial->torque_limit_nm);
}
/** @brief User abort preserves an already latched fault and never grants release. */
void adrc_simple_abort(AdrcSimpleTrial *trial)
{
 if(trial && trial->status.phase!=ADRC_SIMPLE_IDLE && trial->status.phase!=ADRC_SIMPLE_DONE)
  stop_trial(trial,ADRC_SIMPLE_ABORTED,ADRC_FAULT_NONE,trial->last_tick_us);
}
/** @brief Append raw evidence; bounded storage never wraps or overwrites earlier records. */
static void record(AdrcSimpleTrial *trial, const AdrcSimpleInput *input, const AdrcExperimentOutput *output)
{
 AdrcSimpleTrace *entry;
 if(trial->status.trace_count>=ADRC_SIMPLE_TRACE_CAPACITY) return;
 /* Freeze evidence at the first locked terminal sample; DISABLE service continues. */
 if(trial->status.trace_count &&
    trial->trace[trial->status.trace_count-1U].sample.state==ADRC_SIMPLE_LOCKED) return;
 entry=&trial->trace[trial->status.trace_count++];
 entry->sample.timestamp_us=input->feedback.now_us; entry->sample.feedback_us=input->feedback.feedback_us;
 entry->sample.position_rad=input->feedback.position_rad; entry->sample.velocity_rad_s=trial->velocity_rad_s;
 entry->sample.raw_velocity_rad_s=input->feedback.raw_velocity_rad_s;
 entry->sample.torque_requested_nm=output->torque_nm; entry->sample.torque_sent_previous_nm=input->feedback.prev_sent_nm;
 entry->sample.torque_raw_nm=trial->controller_raw_nm;
 entry->sample.z1=trial->controller_z1; entry->sample.z2=trial->controller_z2;
 entry->sample.state=(uint8_t)trial->status.phase; entry->sample.fault=(uint8_t)trial->status.fault;
 entry->sample.flags=(uint8_t)((input->feedback.prev_sent_valid?1U:0U)|(input->tx_receipt_valid?2U:0U));
 entry->sample.axis_index=6U; entry->sample.sequence=trial->status.trace_count-1U;
 entry->sample.reference_rad_s=trial->status.phase==ADRC_SIMPLE_CONTROL?trial->control_reference_rad_s:0.0f;
 entry->feedback_torque_nm=input->feedback_torque_nm; entry->enabled=input->feedback.enabled;
}
/** @brief Fit only an interior constant-torque interval, accounting for estimator midpoint lag. */
static int pulse_gain(AdrcSimpleTrial *trial, float *gain)
{
 float delta=trial->pulse_velocity_end-trial->pulse_velocity_start;
 float threshold=fmaxf(0.004f,4.0f*trial->position_quantum_rad/0.032f);
 if(!trial->pulse_start_seen || !trial->pulse_end_seen || fabsf(delta)<threshold || fabsf(trial->pulse_impulse)<0.0001f) return 0;
 if(!trial->pulse_integral_us) return 0;
 if(trial->profile>=4U && (!trial->pulse_platform_seen || !trial->pulse_platform_valid)) return 0;
 *gain=delta/(trial->pulse_impulse*elapsed(trial->pulse_midpoint_end_us,
  trial->pulse_midpoint_start_us)/(float)trial->pulse_integral_us);
 return isfinite(*gain) && *gain>0.0f?1:-1;
}
/** @brief Tick safety gates before control; stopping remains live until physical disable is proved. */
#if defined(__CC_ARM) && (__ARMCC_VERSION < 6000000)
/* ARMCC 5.06u4 O3 drops pulse target assignment and changes its integration
 * branch. Offline execution of the deployed ELF reproduces both errors.
 * Keep this supervisor function unoptimized; other code retains its settings. */
#pragma push
#pragma O0
#endif
void adrc_simple_tick(AdrcSimpleTrial *trial, const AdrcSimpleInput *input, AdrcExperimentOutput *output)
{
 const AdrcExperimentInput *feedback; uint64_t delta_us, phase_age; int new_sample;
 AdrcExperimentFault fault; float target=0.0f;
 if(!trial || !input || !output) return;
 feedback=&input->feedback; memset(output,0,sizeof(*output)); output->axis_index=6U;
 if(trial->status.phase==ADRC_SIMPLE_IDLE || trial->status.phase==ADRC_SIMPLE_DONE) return;
 adrc_simple_observe_receipt(trial,input);
 delta_us=elapsed(feedback->now_us,trial->last_tick_us);
 new_sample=feedback->feedback_us!=trial->last_feedback_us;
 fault=health(input,trial->torque_trip_nm);
 if(new_sample && elapsed(feedback->feedback_us,trial->last_feedback_us)>8000U) {
  /* Active probe late samples never refresh history; STOP may rebuild health without erasing failure. */
  if((trial->profile!=8U || trial->status.phase>=ADRC_SIMPLE_STOPPING) &&
     fault==ADRC_FAULT_NONE && elapsed(feedback->feedback_us,trial->last_feedback_us)<1500000U) {
   trial->history_count=0U; trial->history_next=0U; trial->estimate_ready=0U;
   estimate(trial,feedback); trial->last_feedback_us=feedback->feedback_us;
  }
  fault=ADRC_FAULT_FEEDBACK;
 }
 if(delta_us<3000U || delta_us>8000U ||
    ((trial->status.phase==ADRC_SIMPLE_CONTROL ||
      (trial->profile==8U && trial->status.phase<ADRC_SIMPLE_STOPPING)) && delta_us>5000U)) fault=ADRC_FAULT_PERIOD;
 /* Adjacent new feedback gives a receive-time difference, not physical instantaneous speed.
  * Evaluate before estimate() updates history; late/duplicate feedback cannot refresh it. */
 if(trial->profile==8U && trial->status.phase<ADRC_SIMPLE_STOPPING &&
    fault==ADRC_FAULT_NONE && new_sample && trial->history_count) {
  unsigned previous_index=(trial->history_next+15U)%16U;
  uint64_t receive_delta_us=elapsed(feedback->feedback_us,trial->position_times[previous_index]);
  if(receive_delta_us && receive_delta_us<=8000U &&
     fabsf((feedback->position_rad-trial->positions[previous_index])/(receive_delta_us*1.0e-6f))>0.5f)
   fault=ADRC_FAULT_SPEED;
 }
 /* Profile 8 has only ENABLE, NEUTRAL_1 and POSITIVE; corruption cannot reach a controller. */
 if(trial->profile==8U && trial->status.phase>ADRC_SIMPLE_POSITIVE && trial->status.phase<ADRC_SIMPLE_STOPPING)
  fault=ADRC_FAULT_CONTROLLER;
 if(fault==ADRC_FAULT_NONE && new_sample) { estimate(trial,feedback); trial->last_feedback_us=feedback->feedback_us; }
 if(fault==ADRC_FAULT_NONE && trial->estimate_ready && fabsf(trial->velocity_rad_s)>0.5f) fault=ADRC_FAULT_SPEED;
 if(fault==ADRC_FAULT_NONE && trial->status.phase<ADRC_SIMPLE_STOPPING &&
    fabsf(feedback->position_rad-trial->initial_position_rad)>0.05f) fault=ADRC_FAULT_TRAVEL;
 if(trial->status.phase<ADRC_SIMPLE_STOPPING) {
  if(feedback->prev_send_failed || !input->tx_receipt_valid ||
    input->tx_kind!=(trial->expected_receipt==1U?0U:1U) ||
    (trial->expected_receipt==2U && (!feedback->prev_sent_valid || !isfinite(feedback->prev_sent_nm) || fabsf(feedback->prev_sent_nm)>trial->torque_limit_nm))) fault=ADRC_FAULT_SEND;
  if(fault!=ADRC_FAULT_NONE) stop_trial(trial,ADRC_SIMPLE_FAILED,fault,feedback->now_us);
  else if(elapsed(feedback->now_us,trial->started_us)>=1400000U) stop_trial(trial,ADRC_SIMPLE_FAILED,ADRC_FAULT_TIME,feedback->now_us);
 }
 trial->last_tick_us=feedback->now_us; phase_age=elapsed(feedback->now_us,trial->phase_us);
 if(trial->status.phase==ADRC_SIMPLE_ENABLE) {
  if(new_sample && feedback->enabled) phase_enter(trial,ADRC_SIMPLE_NEUTRAL_1,feedback->now_us);
  else if(phase_age>=64000U) stop_trial(trial,ADRC_SIMPLE_FAILED,ADRC_FAULT_ENABLE_TIMEOUT,feedback->now_us);
  else { output->enable_requested=1U; trial->expected_receipt=1U; goto finish; }
 }
 if(trial->status.phase>ADRC_SIMPLE_ENABLE && trial->status.phase<ADRC_SIMPLE_STOPPING && !feedback->enabled)
  stop_trial(trial,ADRC_SIMPLE_FAILED,ADRC_FAULT_DRIVER,feedback->now_us);
 phase_age=elapsed(feedback->now_us,trial->phase_us);
 if(trial->status.phase==ADRC_SIMPLE_NEUTRAL_1 || trial->status.phase==ADRC_SIMPLE_NEUTRAL_2 || trial->status.phase==ADRC_SIMPLE_NEUTRAL_3) {
  if(phase_age>=64000U && new_sample) {
   if(trial->status.phase==ADRC_SIMPLE_NEUTRAL_3) {
    /* Select once, pointing toward the initial position without increasing the speed envelope. */
    trial->control_reference_rad_s=(trial->profile>=4U && feedback->position_rad>=trial->initial_position_rad)?-0.1f:0.1f;
    phase_enter(trial,ADRC_SIMPLE_CONTROL,feedback->now_us);
   }
   else phase_enter(trial,(AdrcSimplePhase)(trial->status.phase+1),feedback->now_us);
  }
 }
 phase_age=elapsed(feedback->now_us,trial->phase_us);
 if(trial->profile==8U && trial->status.phase==ADRC_SIMPLE_POSITIVE) {
  target=trial->torque_limit_nm;
  if(phase_age>=trial->pulse_duration_us) {
   stop_trial(trial,ADRC_SIMPLE_PENDING,ADRC_FAULT_NONE,feedback->now_us); target=0.0f;
  }
 }
 else if(trial->status.phase==ADRC_SIMPLE_POSITIVE || trial->status.phase==ADRC_SIMPLE_NEGATIVE) {
  uint32_t pulse_duration_us=(trial->profile>=4U && trial->status.phase==ADRC_SIMPLE_POSITIVE)?312000U:trial->pulse_duration_us;
  uint32_t fit_start_us=trial->profile==0U?48000U:pulse_duration_us-(trial->profile>=4U?144000U:80000U);
  uint32_t fit_end_us=trial->profile==0U?72000U:pulse_duration_us-16000U;
  target=trial->status.phase==ADRC_SIMPLE_POSITIVE?trial->torque_limit_nm:-trial->torque_limit_nm;
  /* Cover the maximum 40 ms difference history plus 8 ms feedback age.
   * Each previous receipt covers the preceding tick interval, including the boundary. */
  if(trial->profile>=4U && phase_age>=fit_start_us-48000U) {
   if(!trial->pulse_platform_seen) {
    trial->pulse_platform_seen=1U;
    trial->pulse_platform_reference_nm=feedback->prev_sent_nm;
    if(phase_age-delta_us>fit_start_us-48000U) trial->pulse_platform_valid=0U;
   }
   if(!feedback->prev_sent_valid || !isfinite(feedback->prev_sent_nm) ||
      feedback->prev_sent_nm*target<=0.0f || fabsf(feedback->prev_sent_nm)<0.9f*trial->torque_limit_nm ||
      fabsf(feedback->prev_sent_nm-trial->pulse_platform_reference_nm)>0.000001f)
    trial->pulse_platform_valid=0U;
  }
  if(phase_age>fit_start_us-16000U && phase_age<=fit_end_us-16000U) { trial->pulse_impulse+=feedback->prev_sent_nm*(delta_us*1.0e-6f); trial->pulse_integral_us+=delta_us; }
  if(new_sample && trial->estimate_ready && !trial->pulse_start_seen && phase_age>=fit_start_us && phase_age<fit_start_us+16000U) {
   trial->pulse_velocity_start=trial->velocity_rad_s; trial->pulse_midpoint_start_us=trial->estimate_midpoint_us; trial->pulse_start_seen=1U; }
  if(new_sample && trial->estimate_ready && !trial->pulse_end_seen && phase_age>=fit_end_us && phase_age<=pulse_duration_us) {
   trial->pulse_velocity_end=trial->velocity_rad_s; trial->pulse_midpoint_end_us=trial->estimate_midpoint_us; trial->pulse_end_seen=1U; }
  if(phase_age>=pulse_duration_us) {
   float gain=0.0f;
   int gain_result=pulse_gain(trial,&gain);
   if(gain_result<=0) stop_trial(trial,gain_result==0?ADRC_SIMPLE_COMPLETED_NO_RESPONSE:ADRC_SIMPLE_FAILED,ADRC_FAULT_CONTROLLER,feedback->now_us);
   else if(trial->status.phase==ADRC_SIMPLE_POSITIVE) { trial->positive_gain=gain; phase_enter(trial,ADRC_SIMPLE_NEUTRAL_2,feedback->now_us); }
   else if(gain<trial->positive_gain*0.5f || gain>trial->positive_gain*2.0f) stop_trial(trial,ADRC_SIMPLE_FAILED,ADRC_FAULT_CONTROLLER,feedback->now_us);
   else { trial->status.b0=0.5f*(gain+trial->positive_gain); phase_enter(trial,ADRC_SIMPLE_NEUTRAL_3,feedback->now_us); }
   target=0.0f;
  }
 }
 if(trial->status.phase==ADRC_SIMPLE_CONTROL) {
  AdrcControllerInput controller_input; AdrcControllerOutput controller_output;
  memset(&controller_input,0,sizeof(controller_input)); memset(&controller_output,0,sizeof(controller_output));
  controller_input.mode=ADRC_MODE_LADRC; controller_input.reference_rad_s=trial->control_reference_rad_s;
  controller_input.velocity_rad_s=trial->velocity_rad_s; controller_input.prev_sent_nm=feedback->prev_sent_nm;
  controller_input.b0=trial->status.b0; controller_input.wc_rad_s=5.0f; controller_input.wo_rad_s=15.0f;
  controller_input.reset=trial->controller_reset; controller_input.new_sample=(uint8_t)new_sample;
  if(!trial->controller(trial->controller_context,&controller_input,&controller_output) ||
    !isfinite(controller_output.torque_raw_nm) || !isfinite(controller_output.z1) || !isfinite(controller_output.z2))
   stop_trial(trial,ADRC_SIMPLE_FAILED,ADRC_FAULT_CONTROLLER,feedback->now_us);
  else { target=controller_output.torque_raw_nm; trial->controller_reset=0U;
   trial->controller_raw_nm=controller_output.torque_raw_nm;
   trial->controller_z1=controller_output.z1; trial->controller_z2=controller_output.z2; }
  if(new_sample) {
   float directed_velocity=trial->control_reference_rad_s>=0.0f?trial->velocity_rad_s:-trial->velocity_rad_s;
   if(directed_velocity>=0.05f && directed_velocity<=0.15f) {
    if(!trial->dwell_tracking) { trial->dwell_tracking=1U; trial->dwell_us=feedback->feedback_us; }
   } else trial->dwell_tracking=0U;
  }
  if(elapsed(feedback->now_us,trial->phase_us)>=400000U && trial->status.phase==ADRC_SIMPLE_CONTROL) {
   int tracking=trial->dwell_tracking && elapsed(feedback->feedback_us,trial->dwell_us)>=64000U;
   stop_trial(trial,tracking?ADRC_SIMPLE_SUCCESS:ADRC_SIMPLE_COMPLETED_NO_RESPONSE,
    tracking?ADRC_FAULT_NONE:ADRC_FAULT_CONTROLLER,feedback->now_us);
  }
 }
 if(trial->status.phase==ADRC_SIMPLE_STOPPING || trial->status.phase==ADRC_SIMPLE_LOCKED) {
  AdrcExperimentFault stop_fault=fault;
  /* Historical trial failure is distinct from present physical release conditions. */
  if(stop_fault==ADRC_FAULT_NONE && fabsf(feedback->position_rad-trial->initial_position_rad)>0.05f)
   stop_fault=ADRC_FAULT_TRAVEL;
  /* The first stop tick may still acknowledge torque/ENABLE; later ticks expect FD. */
  if(stop_fault==ADRC_FAULT_NONE && trial->expected_receipt==3U &&
     (feedback->prev_send_failed || !input->tx_receipt_valid || input->tx_kind!=2U))
   stop_fault=ADRC_FAULT_SEND;
  if(stop_fault!=ADRC_FAULT_NONE &&
     (trial->status.result==ADRC_SIMPLE_SUCCESS || trial->status.result==ADRC_SIMPLE_PENDING ||
      trial->status.result==ADRC_SIMPLE_PROBE_COMPLETED)) {
   trial->status.result=ADRC_SIMPLE_FAILED;
   if(trial->status.fault==ADRC_FAULT_NONE) trial->status.fault=stop_fault;
  }
  output->disable_requested=1U;
  trial->expected_receipt=3U;
  if(input->tx_receipt_valid && input->tx_kind==2U && !feedback->prev_send_failed) trial->disable_sent=1U;
  if(fault==ADRC_FAULT_NONE && fabsf(input->feedback_torque_nm)<=0.15f && new_sample && !feedback->enabled && trial->estimate_ready &&
    fabsf(trial->velocity_rad_s)<=0.03f && fabsf(feedback->raw_velocity_rad_s)<=0.05f &&
    elapsed(feedback->feedback_us,trial->stop_us)<1500000U && feedback->feedback_us!=trial->stop_us && trial->disable_sent) {
   if(!trial->dwell_tracking) { trial->dwell_us=feedback->feedback_us; trial->dwell_tracking=1U; }
   if(elapsed(feedback->feedback_us,trial->dwell_us)>=64000U) {
    trial->status.safe_release=1U; trial->status.phase=ADRC_SIMPLE_DONE;
    if(trial->profile==8U && trial->status.result==ADRC_SIMPLE_PENDING && trial->status.fault==ADRC_FAULT_NONE)
     trial->status.result=ADRC_SIMPLE_PROBE_COMPLETED;
   }
  } else if(fault!=ADRC_FAULT_NONE || fabsf(input->feedback_torque_nm)>0.15f || feedback->enabled || fabsf(trial->velocity_rad_s)>0.03f ||
    fabsf(feedback->raw_velocity_rad_s)>0.05f) trial->dwell_tracking=0U;
  if(trial->status.phase!=ADRC_SIMPLE_DONE && elapsed(feedback->now_us,trial->started_us)>=1500000U) {
   trial->status.phase=ADRC_SIMPLE_LOCKED;
   if(trial->status.fault==ADRC_FAULT_NONE) trial->status.fault=ADRC_FAULT_STOP_TIMEOUT;
   if(trial->status.result==ADRC_SIMPLE_SUCCESS ||
      (trial->profile==8U && (trial->status.result==ADRC_SIMPLE_PENDING || trial->status.result==ADRC_SIMPLE_PROBE_COMPLETED)))
    trial->status.result=ADRC_SIMPLE_FAILED;
  }
 } else {
  float maximum_step=trial->torque_slew_nm_s*(delta_us*1.0e-6f);
  target=fmaxf(-trial->torque_limit_nm,fminf(trial->torque_limit_nm,target));
  target=fmaxf(feedback->prev_sent_nm-maximum_step,fminf(feedback->prev_sent_nm+maximum_step,target));
  target=fmaxf(-trial->torque_limit_nm,fminf(trial->torque_limit_nm,target));
  trial->requested_nm=target; output->torque_nm=target; output->send_torque=1U; trial->expected_receipt=2U;
 }
finish:
 record(trial,input,output);
}
#if defined(__CC_ARM) && (__ARMCC_VERSION < 6000000)
#pragma pop
#endif
/** @brief Copy a bounded raw record for a serialized read-only query. */
int adrc_simple_trace_get(const AdrcSimpleTrial *trial, uint32_t index, AdrcSimpleTrace *record_out)
{
 if(!trial || !record_out || index>=trial->status.trace_count) return 0;
 *record_out=trial->trace[index]; return 1;
}
