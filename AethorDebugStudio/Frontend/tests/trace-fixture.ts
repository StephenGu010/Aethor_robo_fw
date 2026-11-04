/** @file 最小合成trace工件，用于验证旧记录适配，不含真实CAN采集。 */
export const traceFixture={schema:'static-startup-capture-v1',captured_at_utc:'2026-10-26T00:00:00Z',generation:1,records:[
  {decoded:{timestamp_us:1000,serial:0,event_name:'begin',phase_name:'preflight',motor_id:7,can_raw_hex:''}},
  {decoded:{timestamp_us:2000,serial:1,event_name:'raw_rx',phase_name:'enable',motor_id:7,can_raw_hex:'0102',position_rad:999,velocity_rad_s:999,torque_nm:999}},
  {decoded:{timestamp_us:3000,serial:2,event_name:'accepted_rx',phase_name:'first_hold',motor_id:7,can_raw_hex:'0102',position_rad:1,target_rad:1,velocity_rad_s:0,torque_nm:0}},
  {decoded:{timestamp_us:5000,serial:5,event_name:'accepted_rx',phase_name:'first_hold',motor_id:7,can_raw_hex:'0102',position_rad:1,target_rad:1,velocity_rad_s:0,torque_nm:0}}
],decoded_header:{first_fault_record:{timestamp_us:4000,serial:3,can_raw_hex:'0a'}}};
