/** @file 配额准备流程纯模拟回归；不访问串口或网络。 */
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {prepareTorqueAllowance} from '../src/torquePreparation';
import {parseTorqueEnvelope} from '../src/torqueEnvelope';
import type {RemoteReceipt} from '../src/remoteControl';

/** 构造严格v2策略，模拟真实板端维护门禁。 */
function policy(blockers=32,boot='3',authorized=false,peak=5460):RemoteReceipt {
  return {requestId:1,operation:'torque',result:'policy',detail:'模拟',fields:{experimental:'1',motor:'7',axis:'output',
    continuous:'3.5',peak:String(peak/1000),window_ms:'10000',budget:'175.616',used:authorized?'0':'175.616',
    authorized:authorized?'1':'0',can_reset:blockers?'0':'1',token:blockers?'0':'19',boot,coord:'11',
    policy_v:'2',min_mnm:'3500',max_mnm:'6240',peak_mnm:String(peak),blockers:String(blockers),
    reset_temp:'35',evidence_ms:'5000',experiment_fresh_ms:'20'}};
}
/** 按调用次序模拟身份、失能回执与策略，不用缓存位置证明失能。 */
function fixture(options:{before?:number;after?:number;owner?:string;mode?:string;disableResult?:string;wrongMotor?:boolean;cancelAt?:string;bootChange?:boolean;resetUnknown?:boolean;mismatch?:boolean}={}) {
  const calls:string[]=[];let current=true,reads=0;
  const reviewed=parseTorqueEnvelope(policy())!;
  const run=()=>prepareTorqueAllowance(reviewed,6240,{
    isCurrent:()=>current,
    observe:()=>{},
    request:async(body)=>{
      const operation=String(body.operation);calls.push(operation);
      if(options.cancelAt===operation)current=false;
      if(operation==='caps')return {requestId:2,operation,result:'capability',detail:'模拟',fields:{boot:'3',coord:'11',owner:options.owner??'0',authority:'0',active_id:'0',mode:options.mode??'2',load:'1'}};
      if(operation==='torque'){
        reads++;return policy(reads===1?(options.before??32):reads===2?(options.after??0):0,
          options.bootChange&&reads>1?'4':'3',reads>=3,reads>=3?(options.mismatch?5460:6240):5460);
      }
      assert.equal(operation,'torque_reset');assert.equal(body.peakMilliNm,6240);assert.equal(body.token,19);
      return {requestId:3,operation,result:options.resetUnknown?'unknown':'reset',detail:'模拟',fields:{accepted:'1',authorized:'1'}};
    },
    disable:async()=>{calls.push('torque-disable');if(options.cancelAt==='disable')current=false;
      return {requestId:7,operation:'torque-disable',motorId:options.wrongMotor?6:7,accepted:true,result:options.disableResult??'completed',detail:'模拟',lines:[],feedbackConfirmed:false};},
  });
  return {calls,run};
}
test('模式2缺失标准反馈：一次失能后新门禁通过才恢复，并核对所选边界',async()=>{
  const f=fixture();const result=await f.run();assert.equal(result.result,'confirmed');assert.equal(result.policy?.peakMilliNm,6240);
  assert.deepEqual(f.calls,['caps','torque','torque-disable','torque','torque_reset','torque']);
});
test('执行前反馈已新鲜时直接维护，不追加失能',async()=>{
  const f=fixture({before:0});await f.run();assert.ok(!f.calls.includes('torque-disable'));assert.equal(f.calls.filter(x=>x==='torque_reset').length,1);
});
test('LCD占用、活动之外的未知模式与其他门禁阻止发送失能',async()=>{
  for(const options of [{owner:'1'},{mode:'0'},...[1,2,4,8,16,64,128,256,512,1024,544].map(before=>({before}))]){
    const f=fixture(options);assert.notEqual((await f.run()).result,'confirmed');assert.ok(!f.calls.includes('torque-disable'));assert.ok(!f.calls.includes('torque_reset'));
  }
});
test('失能失败、未知或错轴不恢复，不重试',async()=>{
  for(const options of [{disableResult:'failed'},{disableResult:'unknown'},{wrongMotor:true}]){
    const f=fixture(options);assert.equal((await f.run()).uncertainDisable,true);assert.equal(f.calls.filter(x=>x==='torque-disable').length,1);assert.ok(!f.calls.includes('torque_reset'));
  }
});
test('失能后仍过期、过热或未失能不能恢复',async()=>{
  for(const after of [32,512,64,128,256]){const f=fixture({after});assert.notEqual((await f.run()).result,'confirmed');assert.ok(!f.calls.includes('torque_reset'));}
});
test('重启、STOP、断连或导航使后续维护失效',async()=>{
  for(const options of [{bootChange:true},...['caps','torque','disable'].map(cancelAt=>({cancelAt}))]){
    const f=fixture(options);assert.notEqual((await f.run()).result,'confirmed');assert.ok(!f.calls.includes('torque_reset'));
  }
});
test('恢复未知或读回不符不宣称成功、不重发',async()=>{
  for(const options of [{resetUnknown:true},{mismatch:true},{cancelAt:'torque_reset'}]){const f=fixture(options);const result=await f.run();assert.notEqual(result.result,'confirmed');assert.equal(result.uncertainMaintenance,true);assert.equal(f.calls.filter(x=>x==='torque_reset').length,1);}
});
