/** @file adrc_simple_trial_test_main.c
 * @brief Synthetic plants exercise supervisor safety only, not generated-algorithm qualification.
 */
#include "adrc_simple_trial.h"
#if defined(ADRC_SIMPLE_USE_GENERATED)
#include "adrc_generated_adapter.h"
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static AdrcSimpleTrial trial;
static unsigned controller_calls;
/** @brief Select active generated adapter or proportional fixture; faults remain explicit injections. */
static int fixture_controller(void *context, const AdrcControllerInput *input, AdrcControllerOutput *output)
{
 int *fail=(int *)context;
 ++controller_calls;
 assert(input->b0>0.0f); assert(isfinite(input->prev_sent_nm));
#if defined(ADRC_SIMPLE_USE_GENERATED)
 if(*fail==1) return 0;
 if(!adrc_generated_controller_step(NULL,input,output)) return 0;
#else
 output->torque_raw_nm=5.0f*(input->reference_rad_s-input->velocity_rad_s)/input->b0;
 output->z1=input->velocity_rad_s; output->z2=0.0f;
#endif
 if(*fail==2) output->torque_raw_nm=0.0f;
 return *fail!=1;
}
/** @brief Run fixed 4 ms control/8 ms feedback with realistic command and position quantization. */
static void run_case(int scenario)
{
 AdrcSimpleInput input; AdrcExperimentOutput output; unsigned step;
 unsigned stop_ticks=0U; int stop_fault_injected=0;
 float position=0.0f, velocity=0.0f, actual=0.0f; int controller_fail=scenario==10?1:(scenario==17?2:0);
 uint64_t now=scenario==8?UINT64_MAX-200000U:100000U;
 memset(&input,0,sizeof(input)); input.feedback.now_us=now; input.feedback.feedback_us=now;
 input.feedback.axis_index=6U; input.feedback.feedback_valid=1U;
 input.feedback.mos_temperature_c=25.0f; input.feedback.rotor_temperature_c=25.0f;
 if(scenario==18) input.feedback.raw_velocity_rad_s=-200.0f/4095.0f;
 assert(adrc_simple_init(&trial,fixture_controller,&controller_fail,scenario==18?25.0f/65535.0f:0.00001f));
 assert(adrc_simple_start(&trial,&input,&output));
 for(step=1U;step<430U && trial.status.phase!=ADRC_SIMPLE_DONE;++step) {
  now+=scenario==19?(step%2U?3980U:4020U):4000U; input.feedback.now_us=now;
  input.tx_receipt_valid=1U; input.feedback.prev_send_failed=0U;
  input.tx_kind=output.enable_requested?0U:(output.disable_requested?2U:1U);
  if(output.enable_requested) input.feedback.enabled=1U;
  if(output.disable_requested && scenario!=11) { input.feedback.enabled=0U; actual=0.0f; velocity*=0.5f; }
  if(output.send_torque) actual=fmaxf(-0.03f,fminf(0.03f,roundf(output.torque_nm/0.0005f)*0.0005f));
  if(scenario==18 && output.send_torque) {
   actual=roundf((output.torque_nm+10.0f)*4095.0f/20.0f)*20.0f/4095.0f-10.0f;
   if(actual>0.03f) actual-=20.0f/4095.0f;
   if(actual< -0.03f) actual+=20.0f/4095.0f;
  }
  if(scenario==12) actual=0.0f;
  input.feedback.prev_sent_nm=actual; input.feedback.prev_sent_valid=output.send_torque;
  velocity+=(scenario==1?0.0f:(scenario==2?-40.0f:(scenario==18?100.0f:40.0f)))*actual*0.004f;
  if(scenario==18) velocity*=0.992f;
  position+=velocity*0.004f;
  if(step%2U==0U) {
   input.feedback.feedback_us=now;
   input.feedback.position_rad=roundf(position/0.00001f)*0.00001f;
   input.feedback.raw_velocity_rad_s=roundf(velocity/0.01f)*0.01f;
   if(scenario==18) {
    input.feedback.position_rad=roundf(position*65535.0f/25.0f)*25.0f/65535.0f;
    input.feedback.raw_velocity_rad_s=roundf((velocity+200.0f)*4095.0f/400.0f)*400.0f/4095.0f-200.0f;
   }
  }
  if(step==20U) {
   if(scenario==3) input.feedback.feedback_us=now-12000U;
   if(scenario==4) { input.tx_receipt_valid=0U; input.feedback.prev_send_failed=1U; }
   if(scenario==5 || scenario==11) adrc_simple_abort(&trial);
   if(scenario==6) input.feedback.raw_velocity_rad_s=0.6f;
   if(scenario==7) input.feedback.driver_fault=0xFFU;
   if(scenario==9) input.feedback.feedback_us=trial.last_feedback_us-1U;
   if(scenario==13) input.tx_kind=0U;
   if(scenario==14) input.feedback.now_us+=9000U;
   if(scenario==15) input.feedback.mos_temperature_c=50.0f;
   if(scenario==16) input.feedback.position_rad=0.051f;
  }
  if(step>20U && scenario==7) input.feedback.driver_fault=0U;
  if(step>20U && scenario==15) input.feedback.mos_temperature_c=25.0f;
  if(scenario>=20 && trial.status.phase==ADRC_SIMPLE_STOPPING) {
   ++stop_ticks;
   if(stop_ticks==2U) {
    assert(trial.status.result==ADRC_SIMPLE_SUCCESS);
    assert(trial.status.b0>0.0f); stop_fault_injected=1;
    if(scenario==20) input.feedback.driver_fault=0xFFU;
    if(scenario==21) input.feedback.raw_velocity_rad_s=0.6f;
    if(scenario==22) input.feedback.mos_temperature_c=50.0f;
    if(scenario==23) { position=0.051f; input.feedback.position_rad=position; }
    if(scenario==24) input.feedback.now_us+=9000U;
    if(scenario==25) input.feedback.prev_send_failed=1U;
    if(scenario==26) input.tx_receipt_valid=0U;
    if(scenario==27) input.tx_kind=1U;
   }
   if(stop_ticks>2U) {
    input.feedback.driver_fault=0U; input.feedback.mos_temperature_c=25.0f;
   }
  }
  adrc_simple_tick(&trial,&input,&output);
  assert(!(output.disable_requested && output.send_torque));
  assert(fabsf(output.torque_nm)<=0.030001f);
  if(output.send_torque) assert(fabsf(output.torque_nm-input.feedback.prev_sent_nm)<=0.00805f);
  if(trial.status.phase==ADRC_SIMPLE_STOPPING || trial.status.phase==ADRC_SIMPLE_LOCKED) assert(output.disable_requested);
 }
 if(scenario==0 || scenario==8 || scenario==18 || scenario==19) { assert(trial.status.result==ADRC_SIMPLE_SUCCESS); assert(trial.status.b0>20.0f && trial.status.b0<120.0f); }
 else assert(trial.status.result!=ADRC_SIMPLE_SUCCESS);
 if(scenario==11) { assert(trial.status.phase==ADRC_SIMPLE_LOCKED); assert(!trial.status.safe_release);
  /* Terminal evidence must freeze while the stop path continues servicing DISABLE. */
  assert(trial.status.trace_count<400U);
  assert(trial.trace[trial.status.trace_count-1U].sample.state==ADRC_SIMPLE_LOCKED); }
 else { assert(trial.status.phase==ADRC_SIMPLE_DONE); assert(trial.status.safe_release); }
 assert(trial.status.trace_count<=400U);
 if(scenario>=20) {
  static const AdrcExperimentFault expected_faults[]={ADRC_FAULT_DRIVER,ADRC_FAULT_SPEED,
   ADRC_FAULT_TEMPERATURE,ADRC_FAULT_TRAVEL,ADRC_FAULT_PERIOD,ADRC_FAULT_SEND,ADRC_FAULT_SEND,ADRC_FAULT_SEND};
  assert(stop_fault_injected); assert(trial.status.result==ADRC_SIMPLE_FAILED);
  assert(trial.status.fault==expected_faults[scenario-20]);
 }
 if(trial.status.result==ADRC_SIMPLE_SUCCESS) {
  unsigned index, phases_seen=0U; uint64_t tracking_started=0U, tracking_duration=0U;
  int tracking=0, observer_seen=0; float maximum_travel=0.0f, maximum_speed=0.0f;
  for(index=0U;index<trial.status.trace_count;++index) {
   const AdrcExperimentTrace *sample=&trial.trace[index].sample;
   phases_seen|=1U<<sample->state;
   maximum_travel=fmaxf(maximum_travel,fabsf(sample->position_rad));
   maximum_speed=fmaxf(maximum_speed,fabsf(sample->velocity_rad_s));
   /* The final controller tick records STOPPING after evaluating its last fresh feedback. */
   if(sample->state==ADRC_SIMPLE_CONTROL || (sample->state==ADRC_SIMPLE_STOPPING &&
      index>0U && trial.trace[index-1U].sample.state==ADRC_SIMPLE_CONTROL)) {
    assert(isfinite(sample->z1) && isfinite(sample->z2));
    if(fabsf(sample->z1)>0.0001f && fabsf(sample->z2)>0.0001f) observer_seen=1;
    if(sample->velocity_rad_s*trial.control_reference_rad_s>=0.005f && sample->velocity_rad_s*trial.control_reference_rad_s<=0.015f) {
     if(!tracking) { tracking=1; tracking_started=sample->timestamp_us; }
     tracking_duration=sample->timestamp_us-tracking_started;
    } else { tracking=0; tracking_duration=0U; }
   }
  }
  assert(phases_seen&(1U<<ADRC_SIMPLE_POSITIVE));
  assert(phases_seen&(1U<<ADRC_SIMPLE_NEGATIVE));
  assert(phases_seen&(1U<<ADRC_SIMPLE_CONTROL));
  assert(phases_seen&(1U<<ADRC_SIMPLE_DONE));
  assert(tracking_duration>=64000U); assert(maximum_travel<=0.05f); assert(maximum_speed<=0.5f);
#if defined(ADRC_SIMPLE_USE_GENERATED)
  assert(observer_seen);
#else
  (void)observer_seen;
#endif
  printf("  tracking_dwell_us=%llu max_travel=%.6f max_estimated_speed=%.6f\n",
   (unsigned long long)tracking_duration,maximum_travel,maximum_speed);
 }
 printf("case %d result=%d fault=%d b0=%.3f phase=%d PASS\n",scenario,trial.status.result,trial.status.fault,trial.status.b0,trial.status.phase);
}
/** @brief Quantize native MIT torque inward so the decoded command respects the envelope. */
static float native_torque(float requested_nm, float limit_nm)
{
 float actual_nm=roundf((requested_nm+10.0f)*4095.0f/20.0f)*20.0f/4095.0f-10.0f;
 if(actual_nm>limit_nm) actual_nm-=20.0f/4095.0f;
 if(actual_nm< -limit_nm) actual_nm+=20.0f/4095.0f;
 return actual_nm;
}
/** @brief Exercise each bounded profile with native quantization and synthetic faults or friction. */
static void run_profile_case(unsigned profile, unsigned scenario)
{
 static const float plant_gains[]={100.0f,30.0f,14.0f,9.0f,7.0f,3.6f,2.8f,2.0f};
 AdrcSimpleInput input; AdrcExperimentOutput output; unsigned step, positive_seen=0U, negative_seen=0U;
 float position=0.0f, velocity=0.0f, actual=0.0f, maximum_torque=0.0f;
 float maximum_travel=0.0f, maximum_speed=0.0f;
 float bridge_step_nm=profile==7U?0.016f:(profile>=5U?0.012f:0.008f);
 uint64_t now=100000U; int controller_fail=0, injected=0;
 memset(&input,0,sizeof(input)); input.feedback.now_us=now; input.feedback.feedback_us=now;
 input.feedback.axis_index=6U; input.feedback.feedback_valid=1U;
 input.feedback.mos_temperature_c=25.0f; input.feedback.rotor_temperature_c=25.0f;
 input.feedback.raw_velocity_rad_s=-200.0f/4095.0f;
 assert(adrc_simple_init(&trial,fixture_controller,&controller_fail,25.0f/65535.0f));
 assert(adrc_simple_set_profile(&trial,(uint8_t)profile));
 assert(adrc_simple_start(&trial,&input,&output));
 assert(!adrc_simple_set_profile(&trial,(uint8_t)((profile+1U)%4U)));
 for(step=1U;step<430U && trial.status.phase!=ADRC_SIMPLE_DONE;++step) {
  AdrcSimplePhase previous_phase=trial.status.phase;
  uint64_t previous_phase_us=trial.phase_us;
  uint32_t pulse_duration_us=(profile>=4U && previous_phase==ADRC_SIMPLE_POSITIVE)?312000U:trial.pulse_duration_us;
  uint32_t fit_start_us=profile==0U?48000U:pulse_duration_us-(profile>=4U?144000U:80000U);
  uint32_t fit_end_us=profile==0U?72000U:pulse_duration_us-16000U;
  now+=(profile>=4U && scenario==11U)?8000U:4000U; input.feedback.now_us=now;
  input.tx_receipt_valid=1U; input.feedback.prev_send_failed=0U;
  input.tx_kind=output.enable_requested?0U:(output.disable_requested?2U:1U);
  if(output.enable_requested) input.feedback.enabled=1U;
  if(output.disable_requested) { input.feedback.enabled=0U; actual=0.0f; velocity*=0.5f; }
  if(output.send_torque) {
   float decoded=native_torque(output.torque_nm,trial.torque_limit_nm);
   /* The real integer codec permits only one 0.004884 Nm code per 0.008 Nm receipt step. */
   if(profile>=4U) {
    float quantum=20.0f/4095.0f;
    int requested_code=(int)roundf((decoded+10.0f)/quantum);
    int minimum_code=(int)ceilf((fmaxf(-trial.torque_limit_nm,actual-bridge_step_nm)+10.0f)/quantum);
    int maximum_code=(int)floorf((fminf(trial.torque_limit_nm,actual+bridge_step_nm)+10.0f)/quantum);
    if(requested_code<minimum_code) requested_code=minimum_code;
    if(requested_code>maximum_code) requested_code=maximum_code;
    decoded=requested_code*quantum-10.0f;
    assert(fabsf(decoded-actual)<=bridge_step_nm+0.000001f);
   }
   actual=decoded;
  }
  /* Mutate actual receipts to exercise absent and changing plateau rejection. */
  if(profile>=4U && previous_phase==ADRC_SIMPLE_POSITIVE) {
   if(scenario==9U && now-previous_phase_us<220000U) actual=fminf(actual,0.10f);
   if(scenario==10U && now-previous_phase_us==240000U) actual-=20.0f/4095.0f;
  }
  maximum_torque=fmaxf(maximum_torque,fabsf(actual));
  input.feedback.prev_sent_nm=actual; input.feedback.prev_sent_valid=output.send_torque;
  if(scenario==1U) velocity=0.0f;
  else {
   float effective_torque=actual;
   /* A 0.14 Nm static dead zone cannot be overcome by the inward coded envelope. */
   if(scenario==8U && fabsf(actual)<=trial.torque_limit_nm) effective_torque=0.0f;
   /* Scenario 2 models static/Coulomb friction, not an altered measured command. */
   if(scenario==2U) {
    float friction_nm=profile==7U?0.34f:(profile>=5U?0.14f:0.006f);
    if(fabsf(velocity)<0.0001f && fabsf(actual)<=friction_nm) effective_torque=0.0f;
    else effective_torque-=copysignf(friction_nm,fabsf(velocity)>0.0001f?velocity:actual);
   }
   if(profile>=5U) {
    float gain=scenario==7U?100.0f:(scenario==13U?3.0f:(scenario==16U?(profile==5U?4.0f:3.0f):plant_gains[profile]));
    float damping=scenario==13U?2.0f:3.0f;
    /* Fixed near-threshold Coulomb friction and a local elastic presliding plant are counterexamples. */
    if(scenario==14U) {
     float friction_nm=0.14f;
     if(fabsf(velocity)<0.0001f && fabsf(actual)<=friction_nm) effective_torque=0.0f;
     else effective_torque-=copysignf(friction_nm,fabsf(velocity)>0.0001f?velocity:actual);
    }
    if(scenario==15U) effective_torque-=40.0f*position;
    velocity+=(gain*effective_torque-damping*velocity)*((scenario==11U)?0.008f:0.004f);
   } else if(profile>=4U && scenario>=13U) {
    static const float fixed_gains[]={4.0f,5.0f,6.5f};
    static const float fixed_damping[]={2.0f,3.0f,5.0f};
    velocity+=(fixed_gains[scenario-13U]*effective_torque-fixed_damping[scenario-13U]*velocity)*0.004f;
   } else {
    velocity+=(scenario==7U?100.0f:plant_gains[profile])*effective_torque*((profile>=4U && scenario==11U)?0.008f:0.004f);
    velocity*=0.992f;
   }
  }
  /* A brief start followed by constant speed must not fabricate late-window acceleration. */
  if(profile==7U && scenario==17U && previous_phase==ADRC_SIMPLE_POSITIVE && now-previous_phase_us>=100000U)
   velocity=0.08f;
  position+=velocity*((profile>=4U && scenario==11U)?0.008f:0.004f);
  if(step%2U==0U || (profile>=4U && scenario==11U)) {
   input.feedback.feedback_us=now;
   input.feedback.position_rad=roundf(position*65535.0f/25.0f)*25.0f/65535.0f;
   input.feedback.raw_velocity_rad_s=roundf((velocity+200.0f)*4095.0f/400.0f)*400.0f/4095.0f-200.0f;
  }
  if(!injected && previous_phase==ADRC_SIMPLE_POSITIVE && now-previous_phase_us>=60000U && scenario>=3U && scenario<=6U) {
   injected=1;
   if(scenario==3U) adrc_simple_abort(&trial);
   if(scenario==4U) input.feedback.prev_sent_nm=trial.torque_limit_nm+0.001f;
   if(scenario==5U) input.feedback.feedback_us=now-12000U;
   if(scenario==6U) input.feedback.driver_fault=0xFFU;
  } else input.feedback.driver_fault=0U;
  input.feedback_torque_nm=0.0f;
  if(profile>=4U && scenario==12U && previous_phase==ADRC_SIMPLE_POSITIVE && now-previous_phase_us==60000U)
   input.feedback_torque_nm=trial.torque_trip_nm+0.001f;
  if(profile>=5U && (previous_phase==ADRC_SIMPLE_POSITIVE || previous_phase==ADRC_SIMPLE_NEGATIVE) && now-previous_phase_us>=pulse_duration_us)
   printf("  fit profile=%u scenario=%u phase=%u delta=%.9f threshold=%.9f platform=%u/%u integral=%llu\n",profile,scenario,(unsigned)previous_phase,trial.pulse_velocity_end-trial.pulse_velocity_start,4.0f*trial.position_quantum_rad/0.032f,trial.pulse_platform_seen,trial.pulse_platform_valid,(unsigned long long)trial.pulse_integral_us);
  adrc_simple_tick(&trial,&input,&output);
  assert(!(output.disable_requested && output.send_torque));
  assert(fabsf(output.torque_nm)<=trial.torque_limit_nm+0.000001f);
  if(output.send_torque) assert(fabsf(output.torque_nm-input.feedback.prev_sent_nm)<=trial.torque_slew_nm_s*((profile>=4U && scenario==11U)?0.008f:0.004f)+0.000001f);
  if(trial.status.phase==ADRC_SIMPLE_STOPPING || trial.status.phase==ADRC_SIMPLE_LOCKED) assert(output.disable_requested);
  maximum_travel=fmaxf(maximum_travel,fabsf(input.feedback.position_rad));
  maximum_speed=fmaxf(maximum_speed,fabsf(trial.velocity_rad_s));
  if(trial.status.phase==ADRC_SIMPLE_POSITIVE || trial.status.phase==ADRC_SIMPLE_NEGATIVE) {
   uint64_t age=now-trial.phase_us;
   if(trial.pulse_start_seen) assert(trial.pulse_midpoint_start_us-trial.phase_us>=fit_start_us-16000U &&
    trial.pulse_midpoint_start_us-trial.phase_us<fit_start_us);
   if(trial.pulse_end_seen) assert(trial.pulse_midpoint_end_us-trial.phase_us>=fit_end_us-16000U &&
    trial.pulse_midpoint_end_us-trial.phase_us<=pulse_duration_us-16000U);
   if(age<=fit_start_us-16000U) assert(trial.pulse_integral_us==0U);
   if(age>=fit_end_us-16000U) assert(trial.pulse_integral_us==fit_end_us-fit_start_us);
  }
  if(previous_phase==ADRC_SIMPLE_POSITIVE && trial.status.phase==ADRC_SIMPLE_NEUTRAL_2) {
   assert(now-previous_phase_us==pulse_duration_us); positive_seen=1U;
  }
  if(previous_phase==ADRC_SIMPLE_NEGATIVE && trial.status.phase==ADRC_SIMPLE_NEUTRAL_3) {
   assert(now-previous_phase_us==pulse_duration_us); negative_seen=1U;
  }
 }
 printf("profile %u scenario %u result=%d fault=%d b0=%.3f positive=%.3f torque=%.6f travel=%.6f speed=%.6f\n",
  profile,scenario,trial.status.result,trial.status.fault,trial.status.b0,trial.positive_gain,
  maximum_torque,maximum_travel,maximum_speed);
 fflush(stdout);
 assert(trial.status.phase==ADRC_SIMPLE_DONE && trial.status.safe_release);
 assert(maximum_torque<=trial.torque_limit_nm);
 if(profile>=5U) {
  if(scenario==13U && profile==6U) { assert(trial.status.result==ADRC_SIMPLE_FAILED && trial.status.fault==ADRC_FAULT_TRAVEL); }
  else if(scenario==0U && profile==5U) assert(trial.status.result==ADRC_SIMPLE_COMPLETED_NO_RESPONSE && trial.status.b0==0.0f);
  else if(scenario==0U || scenario==13U || scenario==16U) {
   assert(positive_seen && negative_seen && trial.status.b0>0.0f);
   assert(trial.control_reference_rad_s==-0.1f);
   assert(trial.status.result==ADRC_SIMPLE_SUCCESS);
  } else if(scenario==3U) assert(injected && trial.status.result==ADRC_SIMPLE_ABORTED);
  else if((scenario>=4U && scenario<=7U) || scenario==12U) assert(trial.status.result==ADRC_SIMPLE_FAILED);
  else {
   assert(trial.status.b0==0.0f);
   if(scenario==15U) assert(trial.status.result==(profile==5U?ADRC_SIMPLE_FAILED:ADRC_SIMPLE_COMPLETED_NO_RESPONSE));
   else assert(trial.status.result==ADRC_SIMPLE_COMPLETED_NO_RESPONSE);
   if(scenario==9U || scenario==10U || scenario==11U) assert(!trial.pulse_platform_valid);
  }
 } else if(profile>=4U) {
  if(scenario==0U) { assert(positive_seen && trial.positive_gain>0.0f); assert(trial.status.result==ADRC_SIMPLE_FAILED && trial.status.fault==ADRC_FAULT_TRAVEL); }
  else if(scenario==15U) assert(trial.status.result==ADRC_SIMPLE_COMPLETED_NO_RESPONSE && trial.status.b0==0.0f);
  else if(scenario>=13U) {
   assert(positive_seen && negative_seen && trial.status.b0>0.0f);
   assert(trial.control_reference_rad_s==-0.1f);
   assert(trial.status.result==ADRC_SIMPLE_SUCCESS || trial.status.result==ADRC_SIMPLE_COMPLETED_NO_RESPONSE);
  }
  else if(scenario==1U || scenario==8U || scenario==9U || scenario==10U || scenario==11U) {
   if(scenario>=9U) assert(trial.pulse_platform_seen && !trial.pulse_platform_valid);
   assert(trial.status.result==ADRC_SIMPLE_COMPLETED_NO_RESPONSE && trial.status.b0==0.0f);
  } else if(scenario==12U) assert(trial.status.result==ADRC_SIMPLE_FAILED && trial.status.fault==ADRC_FAULT_DRIVER);
  else if(scenario==3U) assert(trial.status.result==ADRC_SIMPLE_ABORTED);
  else if(scenario>=4U && scenario<=6U) assert(trial.status.result==ADRC_SIMPLE_FAILED);
  else if(scenario==7U) assert(trial.status.result==ADRC_SIMPLE_FAILED &&
   (trial.status.fault==ADRC_FAULT_SPEED || trial.status.fault==ADRC_FAULT_TRAVEL));
 } else if(scenario==0U) {
  assert(positive_seen && negative_seen);
#if defined(ADRC_SIMPLE_USE_GENERATED)
  /* Profile 3 fits both pulses but this quantized plant misses the final tracking dwell. */
  assert(trial.status.result==(profile==3U?ADRC_SIMPLE_COMPLETED_NO_RESPONSE:ADRC_SIMPLE_SUCCESS));
#else
  assert(trial.status.result==ADRC_SIMPLE_SUCCESS);
#endif
  assert(maximum_travel<=0.05f && maximum_speed<=0.5f);
 } else if(scenario==1U) assert(trial.status.result==ADRC_SIMPLE_COMPLETED_NO_RESPONSE);
 else if(scenario==2U) {
#if defined(ADRC_SIMPLE_USE_GENERATED)
  assert(trial.status.result==ADRC_SIMPLE_COMPLETED_NO_RESPONSE);
#else
  assert(trial.status.result==(profile==2U?ADRC_SIMPLE_SUCCESS:ADRC_SIMPLE_COMPLETED_NO_RESPONSE));
#endif
 } else if(scenario==3U) assert(injected && trial.status.result==ADRC_SIMPLE_ABORTED);
 else if(scenario<=6U) {
  assert(injected && trial.status.result==ADRC_SIMPLE_FAILED);
  assert(trial.status.fault==(scenario==4U?ADRC_FAULT_SEND:(scenario==5U?ADRC_FAULT_FEEDBACK:ADRC_FAULT_DRIVER)));
 } else if(profile>0U) assert(trial.status.result==ADRC_SIMPLE_FAILED &&
  (trial.status.fault==ADRC_FAULT_SPEED || trial.status.fault==ADRC_FAULT_TRAVEL));
 if(trial.status.result==ADRC_SIMPLE_SUCCESS) {
  unsigned index; uint64_t tracking_start_us=0U, tracking_duration_us=0U; int tracking=0;
  for(index=0U;index<trial.status.trace_count;++index) {
   const AdrcExperimentTrace *sample=&trial.trace[index].sample;
   if(sample->state==ADRC_SIMPLE_CONTROL || (sample->state==ADRC_SIMPLE_STOPPING &&
      index>0U && trial.trace[index-1U].sample.state==ADRC_SIMPLE_CONTROL)) {
    if(sample->velocity_rad_s*trial.control_reference_rad_s>=0.005f && sample->velocity_rad_s*trial.control_reference_rad_s<=0.015f) {
     if(!tracking) { tracking=1; tracking_start_us=sample->timestamp_us; }
     tracking_duration_us=sample->timestamp_us-tracking_start_us;
    } else { tracking=0; tracking_duration_us=0U; }
   }
  }
  assert(tracking_duration_us>=64000U);
  assert(trial.status.b0*2.0f-trial.positive_gain>=trial.positive_gain*0.5f);
  assert(trial.status.b0*2.0f-trial.positive_gain<=trial.positive_gain*2.0f);
  printf("  profile tracking_dwell_us=%llu negative_gain=%.3f PASS\n",
   (unsigned long long)tracking_duration_us,trial.status.b0*2.0f-trial.positive_gain);
 }
}
/** @brief Verify bounded profile selection is idle-only and rejects invalid indices. */
static void profile_api_case(void)
{
 static const float limits[]={0.03f,0.05f,0.08f,0.10f,0.14f,0.20f,0.25f,0.35f};
 static const uint32_t durations[]={80000U,160000U,200000U,240000U,360000U,360000U,360000U,360000U};
 unsigned profile; int controller_fail=0;
 assert(adrc_simple_init(&trial,fixture_controller,&controller_fail,25.0f/65535.0f));
 assert(trial.profile==0U && trial.torque_limit_nm==limits[0]);
 assert(!adrc_simple_set_profile(NULL,0U));
 for(profile=0U;profile<8U;++profile) {
  assert(adrc_simple_set_profile(&trial,(uint8_t)profile));
  assert(trial.profile==profile && trial.torque_limit_nm==limits[profile]);
  assert(trial.pulse_duration_us==durations[profile]);
 }
 assert(!adrc_simple_set_profile(&trial,9U));
 assert(!adrc_simple_set_profile(&trial,255U));
 assert(trial.profile==7U && trial.torque_limit_nm==0.35f);
 trial.status.phase=ADRC_SIMPLE_POSITIVE;
 assert(!adrc_simple_set_profile(&trial,0U));
 assert(trial.profile==7U);
 puts("profile API boundaries PASS");
}
/** @brief Arithmetic boundary fixture: 8 ms receipt age plus 40 ms history needs 48 ms support. */
static void profile_platform_boundary_case(void)
{
 AdrcSimpleInput input; AdrcExperimentOutput output; int fail=0;
 memset(&input,0,sizeof(input));
 assert(adrc_simple_init(&trial,fixture_controller,&fail,25.0f/65535.0f));
 assert(adrc_simple_set_profile(&trial,4U));
 trial.status.phase=ADRC_SIMPLE_POSITIVE; trial.phase_us=100000U; trial.started_us=100000U;
 trial.last_tick_us=264000U; trial.last_feedback_us=252000U; trial.expected_receipt=2U;
 trial.history_count=1U; trial.position_times[0]=220000U; trial.positions[0]=0.0f;
 input.feedback.now_us=268000U; input.feedback.feedback_us=260000U;
 input.feedback.axis_index=6U; input.feedback.feedback_valid=1U; input.feedback.enabled=1U;
 input.feedback.prev_sent_valid=1U; input.feedback.prev_sent_nm=0.139194f;
 input.feedback.position_rad=0.004f; input.tx_receipt_valid=1U; input.tx_kind=1U;
 adrc_simple_tick(&trial,&input,&output);
 assert(trial.estimate_ready && trial.estimate_midpoint_us==240000U);
 assert(trial.pulse_start_seen && trial.pulse_platform_seen && !trial.pulse_platform_valid);
 /* A late first plateau receipt cannot fabricate the missing 120..164 ms coverage. */
 assert(trial.status.b0==0.0f);
 /* Feed an explicitly synthetic detectable delta to isolate platform gating from quantization. */
 trial.pulse_start_seen=1U; trial.pulse_end_seen=1U;
 trial.pulse_velocity_start=0.0f; trial.pulse_velocity_end=0.08f;
 trial.pulse_impulse=0.139194f*0.128f; trial.pulse_integral_us=128000U;
 trial.pulse_midpoint_start_us=252000U; trial.pulse_midpoint_end_us=380000U;
 trial.last_tick_us=408000U; trial.last_feedback_us=412000U;
 input.feedback.now_us=412000U; input.feedback.feedback_us=412000U;
 adrc_simple_tick(&trial,&input,&output);
 assert(trial.status.result==ADRC_SIMPLE_COMPLETED_NO_RESPONSE && trial.positive_gain==0.0f);
 assert(output.disable_requested && trial.status.b0==0.0f);
 /* Identical fit data with prior full platform evidence passes the positive gain arithmetic. */
 trial.status.phase=ADRC_SIMPLE_POSITIVE; trial.last_tick_us=408000U;
 trial.expected_receipt=2U; trial.pulse_platform_valid=1U;
 adrc_simple_tick(&trial,&input,&output);
 assert(trial.status.phase==ADRC_SIMPLE_NEUTRAL_2 && trial.positive_gain>4.0f);
 puts("profile 4 boundary: age8000/history40000 detectable-delta plateau gate PASS (arithmetic only)");
}
/** @brief Select return direction once and reject a same-magnitude velocity of opposite sign. */
static void profile_return_direction_case(void)
{
 unsigned direction, opposed, boundary;
 for(direction=0U;direction<2U;++direction) for(opposed=0U;opposed<2U;++opposed) for(boundary=0U;boundary<2U;++boundary) {
  AdrcSimpleInput input; AdrcExperimentOutput output; int fail=2;
  float reference=direction?-0.1f:0.1f;
  memset(&input,0,sizeof(input));
  assert(adrc_simple_init(&trial,fixture_controller,&fail,25.0f/65535.0f));
  assert(adrc_simple_set_profile(&trial,4U));
  trial.status.phase=ADRC_SIMPLE_NEUTRAL_3; trial.status.b0=5.0f;
  trial.started_us=100000U; trial.phase_us=100000U; trial.last_tick_us=160000U;
  trial.last_feedback_us=160000U; trial.expected_receipt=2U;
  trial.velocity_rad_s=(direction?-1.0f:1.0f)*(opposed?-1.0f:1.0f)*(boundary?0.15f:0.05f);
  input.feedback.now_us=164000U; input.feedback.feedback_us=164000U;
  input.feedback.position_rad=direction?0.01f:-0.01f;
  input.feedback.axis_index=6U; input.feedback.feedback_valid=1U; input.feedback.enabled=1U;
  input.feedback.prev_sent_valid=1U; input.tx_receipt_valid=1U; input.tx_kind=1U;
  adrc_simple_tick(&trial,&input,&output);
  assert(trial.status.phase==ADRC_SIMPLE_CONTROL && trial.control_reference_rad_s==reference);
  assert(trial.trace[0].sample.reference_rad_s==reference);
  assert(trial.dwell_tracking==(opposed?0U:1U));
  trial.history_count=0U; trial.history_next=0U;
  trial.last_tick_us=560000U; trial.last_feedback_us=560000U;
  input.feedback.now_us=564000U; input.feedback.feedback_us=564000U;
  adrc_simple_tick(&trial,&input,&output);
  assert(trial.status.result==(opposed?ADRC_SIMPLE_COMPLETED_NO_RESPONSE:ADRC_SIMPLE_SUCCESS));
  assert(output.disable_requested);
 }
 puts("profile 4 return direction and signed dwell: 8 arithmetic cases PASS");
}
/** @brief Separate running torque trips from strict start and disabled release health. */
static void output_profile_health_case(void)
{
 unsigned profile, boundary;
 for(profile=5U;profile<=7U;++profile) for(boundary=0U;boundary<2U;++boundary) {
  AdrcSimpleInput input; AdrcExperimentOutput output; int fail=0; unsigned tick;
  float trip=profile==7U?0.40f:(profile==5U?0.23f:0.28f);
  memset(&input,0,sizeof(input));
  assert(adrc_simple_init(&trial,fixture_controller,&fail,25.0f/65535.0f));
  assert(adrc_simple_set_profile(&trial,(uint8_t)profile));
  assert(trial.torque_trip_nm==trip && trial.torque_slew_nm_s==(profile==7U?4.0f:3.0f) && trial.torque_step_nm==(profile==7U?0.016f:0.012f));
  input.feedback.axis_index=6U; input.feedback.feedback_valid=1U;
  input.feedback.now_us=100000U; input.feedback.feedback_us=100000U;
  input.feedback_torque_nm=0.151f;
  assert(!adrc_simple_start(&trial,&input,&output));
  input.feedback_torque_nm=0.15f; assert(adrc_simple_start(&trial,&input,&output));
  input.feedback.now_us+=4000U; input.feedback.feedback_us+=4000U;
  input.feedback.enabled=1U; input.tx_receipt_valid=1U; input.tx_kind=0U;
  input.feedback_torque_nm=boundary?trip+0.00001f:trip;
  adrc_simple_tick(&trial,&input,&output);
  assert((trial.status.phase==ADRC_SIMPLE_STOPPING)==(boundary!=0U));
  if(boundary) assert(trial.status.fault==ADRC_FAULT_DRIVER);
  /* Residual running torque delays disabled dwell without corrupting a successful result. */
  trial.status.phase=ADRC_SIMPLE_STOPPING; trial.status.result=ADRC_SIMPLE_SUCCESS;
  trial.status.fault=ADRC_FAULT_NONE; trial.stop_us=input.feedback.now_us;
  trial.expected_receipt=3U; trial.dwell_tracking=0U;
  input.feedback.enabled=0U; input.tx_kind=2U; input.feedback.prev_sent_valid=0U;
  for(tick=0U;tick<30U;++tick) {
   input.feedback.now_us+=4000U; input.feedback.feedback_us+=4000U;
   input.feedback_torque_nm=tick<2U?trip:0.15f;
   adrc_simple_tick(&trial,&input,&output);
   if(tick<2U) assert(!trial.dwell_tracking && !trial.status.safe_release && output.disable_requested);
   if(trial.status.phase==ADRC_SIMPLE_DONE) break;
   assert(output.disable_requested);
  }
  assert(trial.status.safe_release && trial.status.result==ADRC_SIMPLE_SUCCESS);
 }
 puts("output profiles strict start, torque trip boundary and residual stop dwell: 6 cases PASS");
}
/** @brief New envelopes retain active time, period, travel, speed and locked recovery gates. */
static void output_profile_guard_case(void)
{
 unsigned profile, scenario, tick;
 for(profile=5U;profile<=7U;++profile) for(scenario=0U;scenario<(profile==7U?7U:5U);++scenario) {
  AdrcSimpleInput input; AdrcExperimentOutput output; int fail=0;
  memset(&input,0,sizeof(input));
  assert(adrc_simple_init(&trial,fixture_controller,&fail,25.0f/65535.0f));
  assert(adrc_simple_set_profile(&trial,(uint8_t)profile));
  trial.status.phase=ADRC_SIMPLE_CONTROL; trial.status.b0=3.0f;
  trial.started_us=100000U; trial.phase_us=1496000U; trial.last_tick_us=1496000U;
  trial.last_feedback_us=1496000U; trial.expected_receipt=2U;
  input.feedback.now_us=1500000U; input.feedback.feedback_us=1500000U;
  input.feedback.axis_index=6U; input.feedback.feedback_valid=1U; input.feedback.enabled=1U;
  input.feedback.prev_sent_valid=1U; input.tx_receipt_valid=1U; input.tx_kind=1U;
  if(scenario!=0U) trial.started_us=200000U;
  if(scenario==1U) trial.last_tick_us=1494000U;
  if(scenario==2U) input.feedback.position_rad=0.05001f;
  if(scenario==3U) input.feedback.raw_velocity_rad_s=0.50001f;
  if(scenario==4U) {
   trial.started_us=0U; trial.status.phase=ADRC_SIMPLE_STOPPING;
   trial.status.result=ADRC_SIMPLE_ABORTED; trial.stop_us=1400000U;
   trial.expected_receipt=3U; input.tx_kind=2U; input.feedback.enabled=0U;
   input.feedback_torque_nm=0.151f;
  }
  if(scenario>=5U) {
   trial.status.phase=ADRC_SIMPLE_STOPPING; trial.status.result=ADRC_SIMPLE_SUCCESS;
   trial.stop_us=input.feedback.now_us-4000U; trial.expected_receipt=3U;
   input.tx_kind=2U; input.feedback.enabled=0U;
   if(scenario==5U) input.feedback.raw_velocity_rad_s=0.50001f;
   else input.feedback.position_rad=0.05001f;
  }
  adrc_simple_tick(&trial,&input,&output);
  assert(output.disable_requested && !trial.status.safe_release);
  if(scenario<4U) {
   static const AdrcExperimentFault faults[]={ADRC_FAULT_TIME,ADRC_FAULT_PERIOD,ADRC_FAULT_TRAVEL,ADRC_FAULT_SPEED};
   assert(trial.status.result==ADRC_SIMPLE_FAILED && trial.status.fault==faults[scenario]);
  } else if(scenario>=5U) {
   assert(trial.status.result==ADRC_SIMPLE_FAILED);
   assert(trial.status.fault==(scenario==5U?ADRC_FAULT_SPEED:ADRC_FAULT_TRAVEL));
  } else {
   assert(trial.status.phase==ADRC_SIMPLE_LOCKED && !trial.dwell_tracking);
   for(tick=0U;tick<30U;++tick) {
    input.feedback.now_us+=4000U; input.feedback.feedback_us+=4000U;
    input.feedback_torque_nm=tick<3U?0.151f:0.15f;
    adrc_simple_tick(&trial,&input,&output);
    if(tick<3U) assert(trial.status.phase==ADRC_SIMPLE_LOCKED && !trial.dwell_tracking);
    if(trial.status.phase==ADRC_SIMPLE_DONE) break;
    assert(output.disable_requested);
   }
   assert(trial.status.safe_release && trial.status.result==ADRC_SIMPLE_ABORTED);
  }
 }
 puts("output profiles unchanged time/period/travel/speed and strict LOCKED recovery: 17 cases PASS");
}
/** @brief New fixed probe must start once without entering identification or control. */
static void startup_probe_acceptance_case(void)
{
 int fail=0;
 assert(adrc_simple_init(&trial,fixture_controller,&fail,25.0f/65535.0f));
 assert(adrc_simple_set_profile(&trial,8U));
 assert(trial.torque_limit_nm==1.75f && trial.pulse_duration_us==640000U);
}
/** @brief Start the fixed probe using a stationary synthetic input with healthy actual receipts. */
static void probe_fixture_start(AdrcSimpleInput *input, AdrcExperimentOutput *output)
{
 static int fail=0;
 memset(input,0,sizeof(*input));
 input->feedback.now_us=100000U; input->feedback.feedback_us=100000U;
 input->feedback.axis_index=6U; input->feedback.feedback_valid=1U;
 input->feedback.mos_temperature_c=25.0f; input->feedback.rotor_temperature_c=25.0f;
 controller_calls=0U;
 assert(adrc_simple_init(&trial,fixture_controller,&fail,25.0f/65535.0f));
 assert(adrc_simple_set_profile(&trial,8U));
 assert(adrc_simple_start(&trial,input,output));
}
/** @brief Feed one 4 ms receipt/sample while honoring the preceding enable, torque or disable request. */
static void probe_fixture_next(AdrcSimpleInput *input, const AdrcExperimentOutput *output)
{
 input->feedback.now_us+=4000U; input->feedback.feedback_us=input->feedback.now_us;
 input->tx_receipt_valid=1U; input->feedback.prev_send_failed=0U;
 input->tx_kind=output->enable_requested?0U:(output->disable_requested?2U:1U);
 if(output->enable_requested) input->feedback.enabled=1U;
 if(output->disable_requested) input->feedback.enabled=0U;
 input->feedback.prev_sent_valid=output->send_torque;
 input->feedback.prev_sent_nm=output->send_torque?output->torque_nm:0.0f;
}
/** @brief Stationary finite probes never identify b0 or run controller; STOP evidence determines completion. */
static void startup_probe_lifecycle_cases(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<12U;++scenario) {
  AdrcSimpleInput input; AdrcExperimentOutput output; unsigned tick, stop_ticks=0U;
  uint64_t pulse_started_us=0U; int injected=0;
  probe_fixture_start(&input,&output);
  for(tick=0U;tick<440U && trial.status.phase!=ADRC_SIMPLE_DONE;++tick) {
   probe_fixture_next(&input,&output);
   if(scenario==1U && input.tx_kind==1U) input.feedback.prev_sent_nm=fminf(input.feedback.prev_sent_nm,0.8f);
   if(scenario==5U && trial.status.phase==ADRC_SIMPLE_POSITIVE && !injected) {
    adrc_simple_abort(&trial); injected=1;
   }
   if(trial.status.phase>=ADRC_SIMPLE_STOPPING) {
    ++stop_ticks;
    if(scenario==2U) input.feedback_torque_nm=stop_ticks<=4U?0.151f:0.15f;
    if(scenario==4U) input.feedback.enabled=1U;
    if(stop_ticks==3U) {
     if(scenario==3U) input.feedback.raw_velocity_rad_s=0.50001f;
     if(scenario==6U) input.feedback.position_rad=0.05001f;
     if(scenario==7U) input.feedback.prev_send_failed=1U;
     if(scenario==8U) input.feedback_torque_nm=1.85001f;
     if(scenario==9U) input.feedback.feedback_valid=0U;
     if(scenario==10U) input.feedback.now_us+=5000U;
     if(scenario==11U) input.feedback.feedback_us-=12000U;
    } else if(scenario>=3U && scenario!=4U) {
     input.feedback.raw_velocity_rad_s=0.0f; input.feedback.position_rad=0.0f;
     input.feedback_torque_nm=0.0f; input.feedback.feedback_valid=1U;
    }
   }
   adrc_simple_tick(&trial,&input,&output);
   assert(trial.status.phase!=ADRC_SIMPLE_NEUTRAL_2 && trial.status.phase!=ADRC_SIMPLE_NEGATIVE &&
          trial.status.phase!=ADRC_SIMPLE_NEUTRAL_3 && trial.status.phase!=ADRC_SIMPLE_CONTROL);
   assert(!controller_calls && trial.status.b0==0.0f && trial.pulse_integral_us==0U);
   if(trial.status.phase==ADRC_SIMPLE_POSITIVE && !pulse_started_us) pulse_started_us=trial.phase_us;
   if(trial.status.phase>=ADRC_SIMPLE_STOPPING) {
    assert(output.disable_requested && !output.send_torque);
    if(!stop_ticks && scenario!=5U) {
     assert(input.feedback.now_us-pulse_started_us==640000U);
     assert(trial.status.result==ADRC_SIMPLE_PENDING);
    }
    if(scenario==2U && stop_ticks<=4U) assert(!trial.status.safe_release && !trial.dwell_tracking);
   } else if(output.send_torque) {
    assert(output.torque_nm>=0.0f && output.torque_nm<=1.75f);
    assert(output.torque_nm-input.feedback.prev_sent_nm<=0.016001f);
   }
   if(trial.status.phase==ADRC_SIMPLE_LOCKED) break;
  }
  if(scenario<=2U) {
   assert(trial.status.phase==ADRC_SIMPLE_DONE && trial.status.safe_release);
   assert(trial.status.result==ADRC_SIMPLE_PROBE_COMPLETED && trial.status.fault==ADRC_FAULT_NONE);
   assert(trial.probe_reached==(scenario==1U?0U:1U));
   assert(fabsf(trial.probe_peak_nm-(scenario==1U?0.8f:1.75f))<0.000001f);
  } else if(scenario==5U) assert(trial.status.result==ADRC_SIMPLE_ABORTED);
  else assert(trial.status.result==ADRC_SIMPLE_FAILED && trial.status.fault!=ADRC_FAULT_NONE);
  if(scenario==4U) {
   assert(trial.status.phase==ADRC_SIMPLE_LOCKED && !trial.status.safe_release);
   for(tick=0U;tick<40U && trial.status.phase!=ADRC_SIMPLE_DONE;++tick) {
    probe_fixture_next(&input,&output); adrc_simple_tick(&trial,&input,&output);
    assert(output.disable_requested && trial.status.result==ADRC_SIMPLE_FAILED);
   }
   assert(trial.status.phase==ADRC_SIMPLE_DONE && trial.status.safe_release);
   assert(trial.status.result==ADRC_SIMPLE_FAILED && trial.status.fault==ADRC_FAULT_STOP_TIMEOUT);
  }
 }
 puts("startup probe lifecycle: 12 cases PASS; closed_loop=0 qualification=0 controller_calls=0");
}
/** @brief Active probe rejects every unsafe input and every forbidden later phase immediately. */
static void startup_probe_guard_cases(void)
{
 unsigned scenario;
 static const AdrcExperimentFault expected_faults[]={ADRC_FAULT_SPEED,ADRC_FAULT_TRAVEL,
  ADRC_FAULT_SPEED,ADRC_FAULT_PERIOD,ADRC_FAULT_FEEDBACK,ADRC_FAULT_DRIVER,
  ADRC_FAULT_TEMPERATURE,ADRC_FAULT_SEND,ADRC_FAULT_SEND,ADRC_FAULT_DRIVER,
  ADRC_FAULT_SEND,ADRC_FAULT_SEND,ADRC_FAULT_CONTROLLER,ADRC_FAULT_CONTROLLER,
  ADRC_FAULT_CONTROLLER,ADRC_FAULT_CONTROLLER,ADRC_FAULT_SPEED,ADRC_FAULT_TIME,
  ADRC_FAULT_PERIOD,ADRC_FAULT_FEEDBACK};
 for(scenario=0U;scenario<20U;++scenario) {
  AdrcSimpleInput input; AdrcExperimentOutput output; uint8_t history_before;
  probe_fixture_start(&input,&output);
  trial.status.phase=ADRC_SIMPLE_POSITIVE; trial.phase_us=100000U; trial.expected_receipt=2U;
  probe_fixture_next(&input,&output); input.tx_kind=1U; input.feedback.prev_sent_valid=1U;
  input.feedback.enabled=1U; input.feedback.prev_sent_nm=0.1f;
  if(scenario==0U) { input.feedback.raw_velocity_rad_s=0.50001f; input.feedback.prev_sent_nm=1.74f; }
  if(scenario==1U) { input.feedback.position_rad=0.05001f; trial.history_count=0U; }
  if(scenario==2U) input.feedback.position_rad=0.00201f;
  if(scenario==3U) { input.feedback.now_us+=2000U; input.feedback.feedback_us=input.feedback.now_us; }
  if(scenario==4U) input.feedback.feedback_valid=0U;
  if(scenario==5U) input.feedback.driver_fault=1U;
  if(scenario==6U) input.feedback.mos_temperature_c=50.0f;
  if(scenario==7U) input.tx_receipt_valid=0U;
  if(scenario==8U) input.feedback.prev_send_failed=1U;
  if(scenario==9U) input.feedback_torque_nm=1.85001f;
  if(scenario==10U) input.feedback.prev_sent_valid=0U;
  if(scenario==11U) input.tx_kind=2U;
  if(scenario>=12U && scenario<=15U) trial.status.phase=(AdrcSimplePhase)(ADRC_SIMPLE_NEUTRAL_2+scenario-12U);
  if(scenario==16U) input.feedback.position_rad=-0.00201f;
  if(scenario==17U) { trial.started_us=input.feedback.now_us-1400000U; trial.history_count=0U; }
  if(scenario==18U) { input.feedback.now_us-=1001U; input.feedback.feedback_us=input.feedback.now_us; }
  if(scenario==19U) input.feedback.feedback_us-=12000U;
  history_before=trial.history_count;
  adrc_simple_tick(&trial,&input,&output);
  assert(output.disable_requested && !output.send_torque && !controller_calls);
  assert(trial.status.result==ADRC_SIMPLE_FAILED && trial.status.fault==expected_faults[scenario]);
  assert(trial.status.b0==0.0f && !trial.status.safe_release);
  if(scenario==2U || scenario==16U || scenario==19U) assert(trial.history_count==history_before);
  if(scenario==7U || scenario==8U || scenario==10U || scenario==11U) assert(trial.probe_peak_nm==0.0f);
  if(scenario==0U) assert(trial.probe_peak_nm==1.74f && trial.probe_reached);
 }
 puts("startup probe guards: 20 cases PASS");
}
/** @brief Receive history updates only with healthy new feedback; exact speed/period boundaries remain valid. */
static void startup_probe_feedback_cases(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<6U;++scenario) {
  AdrcSimpleInput input; AdrcExperimentOutput output;
  probe_fixture_start(&input,&output);
  trial.status.phase=ADRC_SIMPLE_POSITIVE; trial.expected_receipt=2U;
  probe_fixture_next(&input,&output); input.tx_kind=1U; input.feedback.prev_sent_valid=1U;
  input.feedback.enabled=1U;
  if(scenario==0U) input.feedback.feedback_us=trial.last_feedback_us;
  if(scenario==1U) input.feedback.position_rad=0.002f;
  if(scenario==2U) input.feedback.position_rad=-0.002f;
  if(scenario==3U) { input.feedback.now_us+=1000U; input.feedback.feedback_us=input.feedback.now_us; }
  if(scenario==4U) { input.feedback.now_us-=1000U; input.feedback.feedback_us=input.feedback.now_us; }
  if(scenario==5U) { input.feedback.feedback_us=trial.last_feedback_us-1U; }
  adrc_simple_tick(&trial,&input,&output);
  if(scenario==5U) {
   assert(output.disable_requested && trial.status.fault==ADRC_FAULT_FEEDBACK);
   assert(trial.history_count==1U && trial.last_feedback_us==100000U);
  } else {
   assert(output.send_torque && trial.status.phase==ADRC_SIMPLE_POSITIVE);
   assert(trial.history_count==(scenario==0U?1U:2U));
  }
  assert(!controller_calls);
 }
 puts("startup probe feedback history and speed/period boundaries: 6 cases PASS");
}
/** @brief Active late feedback stops without refreshing history; STOP alone may rebuild disabled evidence. */
static void startup_probe_late_stop_recovery_case(void)
{
 AdrcSimpleInput input; AdrcExperimentOutput output; unsigned tick;
 probe_fixture_start(&input,&output);
 trial.status.phase=ADRC_SIMPLE_POSITIVE; trial.expected_receipt=2U;
 input.feedback.now_us=112000U; input.feedback.feedback_us=112000U; trial.last_tick_us=108000U;
 input.feedback.enabled=1U; input.tx_receipt_valid=1U; input.tx_kind=1U; input.feedback.prev_sent_valid=1U;
 adrc_simple_tick(&trial,&input,&output);
 assert(output.disable_requested && trial.status.fault==ADRC_FAULT_FEEDBACK);
 assert(trial.history_count==1U && trial.last_feedback_us==100000U);
 for(tick=0U;tick<40U && trial.status.phase!=ADRC_SIMPLE_DONE;++tick) {
  probe_fixture_next(&input,&output); adrc_simple_tick(&trial,&input,&output);
  assert(output.disable_requested && trial.status.result==ADRC_SIMPLE_FAILED);
 }
 assert(trial.status.phase==ADRC_SIMPLE_DONE && trial.status.safe_release);
 assert(trial.status.result==ADRC_SIMPLE_FAILED && trial.status.fault==ADRC_FAULT_FEEDBACK);
 assert(!controller_calls);
 puts("startup probe active late sample and healthy disabled recovery: 1 case PASS");
}
/** @brief Receipt-only observation requires an active matching torque obligation and never advances the supervisor. */
static void startup_probe_receipt_observation_cases(void)
{
 unsigned scenario;
 for(scenario=0U;scenario<14U;++scenario) {
  AdrcSimpleInput input; AdrcExperimentOutput output; AdrcSimplePhase previous_phase;
  probe_fixture_start(&input,&output);
  trial.status.phase=ADRC_SIMPLE_POSITIVE; trial.expected_receipt=2U;
  input.tx_receipt_valid=1U; input.tx_kind=1U;
  input.feedback.prev_sent_valid=1U; input.feedback.prev_sent_nm=1.74f;
  if(scenario==0U) input.tx_receipt_valid=0U;
  if(scenario==1U) input.feedback.prev_sent_valid=0U;
  if(scenario==2U) input.tx_kind=2U;
  if(scenario==3U) trial.expected_receipt=1U;
  if(scenario==4U) input.feedback.prev_send_failed=1U;
  if(scenario==5U) input.feedback.prev_sent_nm=NAN;
  if(scenario==6U) input.feedback.prev_sent_nm=1.75001f;
  if(scenario==7U) trial.status.phase=ADRC_SIMPLE_STOPPING;
  if(scenario==8U) trial.status.phase=ADRC_SIMPLE_LOCKED;
  if(scenario==9U) trial.status.phase=ADRC_SIMPLE_DONE;
  if(scenario==10U) trial.status.phase=ADRC_SIMPLE_IDLE;
  if(scenario==11U) trial.profile=7U;
  if(scenario==13U) input.feedback.prev_sent_nm=-1.74f;
  previous_phase=trial.status.phase;
  adrc_simple_observe_receipt(&trial,&input);
  assert(trial.status.phase==previous_phase && trial.status.result==ADRC_SIMPLE_PENDING && !controller_calls);
  assert(trial.status.b0==0.0f && !trial.status.safe_release);
  if(scenario<12U) assert(trial.probe_peak_nm==0.0f && !trial.probe_reached);
  else {
   assert(trial.probe_peak_nm==1.74f && trial.probe_reached);
   adrc_simple_observe_receipt(&trial,&input); /* Identical valid observation is idempotent. */
   assert(trial.probe_peak_nm==1.74f);
   trial.status.phase=ADRC_SIMPLE_STOPPING; input.feedback.prev_sent_nm=1.75f;
   adrc_simple_observe_receipt(&trial,&input);
   assert(trial.probe_peak_nm==1.74f);
  }
 }
 adrc_simple_observe_receipt(NULL,NULL);
 puts("startup probe receipt-only observation boundaries: 14 cases PASS");
}
/** @brief Execute success, no-response, reversed gain, feedback, send, STOP, and guard cases. */
int main(void)
{
 int scenario; unsigned profile;
 startup_probe_acceptance_case(); startup_probe_lifecycle_cases(); startup_probe_guard_cases(); startup_probe_feedback_cases(); startup_probe_late_stop_recovery_case(); startup_probe_receipt_observation_cases(); profile_api_case(); output_profile_health_case(); output_profile_guard_case(); profile_platform_boundary_case(); profile_return_direction_case(); for(scenario=0;scenario<28;++scenario) run_case(scenario);
 for(profile=0U;profile<4U;++profile) for(scenario=0;scenario<8;++scenario) run_profile_case(profile,(unsigned)scenario);
 for(scenario=0;scenario<16;++scenario) if(scenario!=2) run_profile_case(4U,(unsigned)scenario);
 for(profile=5U;profile<=6U;++profile) for(scenario=0;scenario<17;++scenario) run_profile_case(profile,(unsigned)scenario);
 for(scenario=0;scenario<13;++scenario) run_profile_case(7U,(unsigned)scenario);
 run_profile_case(7U,17U);
#if defined(ADRC_SIMPLE_USE_GENERATED)
 puts("54 startup probe + 17 guards + 6 health + 48 output profile + 28 regression + 32 legacy profile + 15 startup profile + 1 boundary + 8 direction + 1 API active-generated-controller tests passed; profile 3 tracking remains unqualified; no hardware qualification claimed.");
#else
 puts("54 startup probe + 17 guards + 6 health + 48 output profile + 28 regression + 32 legacy profile + 15 startup profile + 1 boundary + 8 direction + 1 API proportional-fixture tests passed; no hardware qualification claimed.");
#endif
 return 0;
}

