/** @file 会话结束的身份、失能及单次提交回归；传输全部注入，不连接设备。 */
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {endRemoteSession,describeRemoteOwner,confirmSessionReleased} from '../src/remoteSession';
import type {RemoteReceipt} from '../src/remoteControl';

/** 建立可枚举的板端回复，任何未计划请求立即失败。 */
function fixture(overrides:Record<string,string>={},completion:Record<string,string>={}){
  const calls:Record<string,unknown>[]=[];let capabilityCount=0,submissionCount=0,current=true;
  const caps:Record<string,string>={token:'7',boot:'12',epoch:'9',owner:'2',active_id:'0',terminal_id:'44',...overrides};
  const request=async(body:Record<string,unknown>):Promise<RemoteReceipt>=>{
    calls.push(body);
    if(body.operation==='caps'){
      capabilityCount++;
      return {requestId:2,operation:'caps',result:'capability',detail:'模拟',fields:capabilityCount<3?caps:{...caps,owner:'0',epoch:'10',terminal_id:'91',...(caps.authority!==undefined?{authority:'0',remote_fault_release:'0'}:{})}};
    }
    if(body.operation==='result')return {requestId:3,operation:'result',result:body.transactionId===44?'failed':'completed',detail:'模拟',fields:{id:String(body.transactionId),accepted:'1',disabled:'1',...completion}};
    assert.equal(body.operation,'release');submissionCount++;
    return {requestId:91,operation:'release',result:'queued',detail:'模拟',fields:{}};
  };
  return {calls,caps,request,isCurrent:()=>current,wait:async()=>{},observe:()=>{},cancel:()=>{current=false;},submitted:()=>submissionCount};
}

test('失败且确认失能后只释放一次，复核实际控制权，不使能或运动',async()=>{
  const transport=fixture();const result=await endRemoteSession(transport);
  assert.equal(result.result,'completed');assert.equal(transport.submitted(),1);
  assert.deepEqual(transport.calls.map(body=>body.operation),['caps','result','caps','release','result','caps']);
  assert.match(describeRemoteOwner('1'),/本地保护/);
});
test('活动、本地占用、无会话或错误能力不提交释放',async()=>{
  const candidates:Record<string,string>[]=[{owner:'1'},{owner:'0'},{active_id:'45'},{boot:'0'},{epoch:'?'},{terminal_id:'0'}];
  for(const fields of candidates){
    const transport=fixture(fields);await endRemoteSession(transport);assert.equal(transport.submitted(),0);
  }
});
test('缺失失能证明、错误原编号及未知结果都保留只读恢复',async()=>{
  const candidates:Record<string,string>[]=[{disabled:'0'},{accepted:'0'},{id:'45'}];
  for(const completion of candidates){
    const transport=fixture({},completion);const result=await endRemoteSession(transport);
    assert.notEqual(result.result,'completed');assert.equal(transport.submitted(),0);
  }
});
test('读取终态后启动号、会话、终态编号、控制权或活动状态变化，不能提交',async()=>{
  for(const field of ['boot','epoch','terminal_id','owner','active_id']){
    const transport=fixture();let reads=0;
    const request=async(body:Record<string,unknown>)=>{
      const receipt=await transport.request(body);
      if(body.operation==='caps'&&++reads===2)return {...receipt,fields:{...receipt.fields,[field]:'123'}};
      return receipt;
    };
    await endRemoteSession({...transport,request});assert.equal(transport.submitted(),0);
  }
});
test('页面或STOP意图变化后，不使用迟到回复提交释放',async()=>{
  const transport=fixture();const request=async(body:Record<string,unknown>)=>{
    const receipt=await transport.request(body);if(body.operation==='result')transport.cancel();return receipt;
  };
  await endRemoteSession({...transport,request});assert.equal(transport.submitted(),0);
});
test('释放入队后结果未知，保留释放编号，不自动重发',async()=>{
  const transport=fixture();const request=async(body:Record<string,unknown>)=>{
    const receipt=await transport.request(body);
    return body.operation==='result'&&body.transactionId===91?{...receipt,result:'unknown'}:receipt;
  };
  const result=await endRemoteSession({...transport,request});
  assert.equal(result.transactionId,91);assert.equal(result.result,'unknown');assert.equal(transport.submitted(),1);
});

test('释放原终态完成仍须复核当前控制权和启动号，未知提交也不重发',async()=>{
  const receipt:RemoteReceipt={requestId:92,operation:'result',result:'completed',detail:'模拟',fields:{id:'91',accepted:'1',disabled:'1'}};
  const context={boot:'12',transactionId:91};
  for(const changes of [{owner:'1'},{boot:'13'},{active_id:'55'},{terminal_id:'99'}] as Record<string,string>[]){
    const calls:Record<string,unknown>[]=[];
    const options={request:async(body:Record<string,unknown>):Promise<RemoteReceipt>=>{calls.push(body);return {requestId:93,operation:'caps',result:'capability',detail:'模拟',fields:{boot:'12',epoch:'10',owner:'0',active_id:'0',terminal_id:'91',...changes}};},isCurrent:()=>true,wait:async()=>{},observe:()=>{}};
    assert.equal(await confirmSessionReleased(context,receipt,options),false);
    assert.deepEqual(calls,[{operation:'caps'}]);
  }
  const transport=fixture();transport.caps.owner='0';transport.caps.terminal_id='91';
  assert.equal(await confirmSessionReleased({...context,transactionId:0},receipt,transport),true);
});

test('仅新固件审核通过的原远程故障可结束，真实LCD和旧固件不能接管',async()=>{
  const accepted=fixture({owner:'1',authority:'3',remote_fault_release:'1'});
  assert.equal((await endRemoteSession(accepted)).result,'completed');assert.equal(accepted.submitted(),1);
  for(const fields of [{owner:'1',authority:'1',remote_fault_release:'1'},
    {owner:'1',authority:'3',remote_fault_release:'0'}, {owner:'1'}] as Record<string,string>[]){
    const blocked=fixture(fields);await endRemoteSession(blocked);assert.equal(blocked.submitted(),0);
  }
});
