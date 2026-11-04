/** @file adrc_wide_position_codec_test_main.c
 * @brief Exhaustive actual-codec mapping checks; no hardware or physical motion qualification.
 */
#include "s3519_codec.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief Form one synthetic packed feedback with independently selected position and velocity codes. */
static CanFrame feedback_frame(unsigned position_code, unsigned velocity_code)
{
 CanFrame frame;
 uint8_t payload[8]={7U,0U,0U,0U,0U,0U,25U,26U};
 payload[1]=(uint8_t)(position_code>>8U); payload[2]=(uint8_t)position_code;
 payload[3]=(uint8_t)(velocity_code>>4U);
 payload[4]=(uint8_t)(((velocity_code&15U)<<4U)|8U);
 assert(can_frame_init(&frame,23U,payload,8U)==CAN_FRAME_STATUS_OK);
 return frame;
}

/** @brief Check every 16-bit position code and both mid-zero/endpoint velocity codes at each mapping. */
static unsigned test_all_feedback_codes(void)
{
 const float position_ranges[3]={12.5F,512.0F,1024.0F};
 const unsigned velocity_codes[4]={0U,2047U,2048U,4095U};
 unsigned range_index, position_code, velocity_index, checks=0U;
 for(range_index=0U;range_index<3U;++range_index) {
  S3519Ranges ranges={position_ranges[range_index],200.0F,10.0F};
  float previous=-INFINITY;
  for(position_code=0U;position_code<=65535U;++position_code) {
   for(velocity_index=0U;velocity_index<4U;++velocity_index) {
    S3519Feedback feedback;
    CanFrame frame=feedback_frame(position_code,velocity_codes[velocity_index]);
    double expected_position=2.0*(double)ranges.position_max_rad*(double)position_code/65535.0-
      (double)ranges.position_max_rad;
    double expected_velocity=400.0*(double)velocity_codes[velocity_index]/4095.0-200.0;
    assert(s3519_decode_feedback(&frame,&ranges,&feedback)==S3519_CODEC_STATUS_OK);
    assert(isfinite(feedback.position_rad) && fabsf(feedback.position_rad)<=ranges.position_max_rad);
    assert(fabs((double)feedback.position_rad-expected_position)<=4.0*FLT_EPSILON*(double)ranges.position_max_rad);
    assert(fabs((double)feedback.velocity_rad_s-expected_velocity)<=4.0*FLT_EPSILON*200.0);
    assert(fabs((double)feedback.torque_nm-(20.0*2048.0/4095.0-10.0))<=4.0*FLT_EPSILON*10.0);
    assert(feedback.esc_id==7U && feedback.state==S3519_DRIVER_STATE_DISABLED);
    assert(feedback.mos_temperature_c==25.0F && feedback.rotor_temperature_c==26.0F);
    if(velocity_index==0U) {
     CanFrame command;
     unsigned encoded_position;
     assert(feedback.position_rad>previous); previous=feedback.position_rad;
     assert(s3519_pack_mit(7U,&ranges,feedback.position_rad,0.0F,0.0F,0.0F,0.0F,&command)==S3519_CODEC_STATUS_OK);
     encoded_position=((unsigned)command.data[0]<<8U)|command.data[1];
     assert(encoded_position<=position_code+1U && position_code<=encoded_position+1U);
    }
    ++checks;
   }
  }
 }
 return checks;
}

/** @brief Same bytes require their matching PMAX; pure-torque zero-gain requests retain their layout. */
static void test_mapping_mismatch_and_neutral(void)
{
 const S3519Ranges old_ranges={12.5F,200.0F,10.0F};
 const S3519Ranges wide_ranges={512.0F,200.0F,10.0F};
 CanFrame frame=feedback_frame(45000U,2048U), old_command, wide_command;
 S3519Feedback old_feedback, wide_feedback;
 assert(s3519_decode_feedback(&frame,&old_ranges,&old_feedback)==S3519_CODEC_STATUS_OK);
 assert(s3519_decode_feedback(&frame,&wide_ranges,&wide_feedback)==S3519_CODEC_STATUS_OK);
 assert(fabsf(wide_feedback.position_rad-old_feedback.position_rad)>100.0F);
 assert(wide_feedback.velocity_rad_s==old_feedback.velocity_rad_s && wide_feedback.torque_nm==old_feedback.torque_nm);
 assert(s3519_pack_mit(7U,&old_ranges,0.0F,0.0F,0.0F,0.0F,1.75F,&old_command)==S3519_CODEC_STATUS_OK);
 assert(s3519_pack_mit(7U,&wide_ranges,0.0F,0.0F,0.0F,0.0F,1.75F,&wide_command)==S3519_CODEC_STATUS_OK);
 assert(old_command.length==8U && !memcmp(old_command.data,wide_command.data,8U));
}

/** @brief The new fixed range doubles position scale while preserving velocity, torque and zero-gain bytes. */
static void test_rated70_mapping_and_endpoints(void)
{
 const S3519Ranges before={512.0F,200.0F,10.0F};
 const S3519Ranges after={1024.0F,200.0F,10.0F};
 CanFrame frame=feedback_frame(45000U,2048U), before_command, after_command;
 S3519Feedback before_feedback, after_feedback;
 assert(s3519_decode_feedback(&frame,&before,&before_feedback)==S3519_CODEC_STATUS_OK);
 assert(s3519_decode_feedback(&frame,&after,&after_feedback)==S3519_CODEC_STATUS_OK);
 assert(after_feedback.position_rad==2.0F*before_feedback.position_rad);
 assert(after_feedback.velocity_rad_s==before_feedback.velocity_rad_s && after_feedback.torque_nm==before_feedback.torque_nm);
 assert(s3519_pack_mit(7U,&before,0.0F,0.0F,0.0F,0.0F,1.75F,&before_command)==S3519_CODEC_STATUS_OK);
 assert(s3519_pack_mit(7U,&after,0.0F,0.0F,0.0F,0.0F,1.75F,&after_command)==S3519_CODEC_STATUS_OK);
 assert(!memcmp(before_command.data,after_command.data,8U));
 assert(s3519_pack_mit(7U,&after,nextafterf(1024.0F,INFINITY),0.0F,0.0F,0.0F,0.0F,&after_command)==S3519_CODEC_STATUS_INVALID_RANGE);
 assert(s3519_pack_mit(7U,&after,nextafterf(-1024.0F,-INFINITY),0.0F,0.0F,0.0F,0.0F,&after_command)==S3519_CODEC_STATUS_INVALID_RANGE);
}

/** @brief Mapping does not accept invalid limits or positions beyond its representable endpoints. */
static void test_invalid_ranges(void)
{
 S3519Ranges ranges={512.0F,200.0F,10.0F};
 CanFrame frame=feedback_frame(32768U,2048U), command;
 S3519Feedback feedback;
 assert(s3519_pack_mit(7U,&ranges,nextafterf(512.0F,INFINITY),0.0F,0.0F,0.0F,0.0F,&command)==S3519_CODEC_STATUS_INVALID_RANGE);
 ranges.position_max_rad=NAN;
 assert(s3519_decode_feedback(&frame,&ranges,&feedback)==S3519_CODEC_STATUS_INVALID_RANGE);
 ranges.position_max_rad=INFINITY;
 assert(s3519_decode_feedback(&frame,&ranges,&feedback)==S3519_CODEC_STATUS_INVALID_RANGE);
 ranges.position_max_rad=0.0F;
 assert(s3519_decode_feedback(&frame,&ranges,&feedback)==S3519_CODEC_STATUS_INVALID_RANGE);
}

/** @brief Run actual codec arithmetic and report only software coverage and representation precision. */
int main(void)
{
 unsigned checks=test_all_feedback_codes();
 test_mapping_mismatch_and_neutral(); test_rated70_mapping_and_endpoints(); test_invalid_ranges();
 assert(checks==786432U);
 printf("WIDE_POSITION_CODEC_PASSED feedback_cases=%u position_codes_per_range=65536 ranges=12.5,512,1024\n",checks);
 printf("qnative_rad=%.12g qnative_degrees=%.12g physical_accuracy_verified=false qualification=false\n",
  2048.0/65535.0,2048.0/65535.0*180.0/3.14159265358979323846);
 return 0;
}
