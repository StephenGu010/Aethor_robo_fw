/** @file 单按钮执行编排测试：全部使用内存传输，禁止真实设备访问。 */
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {runRemoteWorkflow} from '../src/remoteWorkflow';
import type {RemoteDraft,RemoteReceipt} from '../src/remoteControl';
const draft:RemoteDraft={operation:'hold',delta:.01,speed:.02,kp:120,kd:.3,torque:0,duration:1000};
/** 构造顺序能力、动作和终态；可注入某一步失败或设备重启。 */
function fixture(failOperation='', reboot=false, initialMode='1'){
  const calls:string[]=[];let owner='0',operation='',serial=10,capsCount=0,mode=initialMode;
  return {calls,request:async(body:Record<string,unknown>):Promise<RemoteReceipt>=>{
    const next=String(body.operation);calls.push(next);
    if(next==='caps')return {requestId:++serial,operation:next,result:'capability',detail:'caps',fields:{boot:reboot&&++capsCount>1?'2':'1',token:String(serial),owner,fresh:'1',mode,static:'1',preflight:'1',switch:'1',active_id:'0'}};
    if(next==='result'){const failed=operation===failOperation;if(!failed){if(operation==='acquire')owner='2';if(operation==='release')owner='0';if(operation==='mode_mit')mode='1';if(operation==='mode_pos')mode='2';}return {requestId:++serial,operation:next,result:failed?'failed':'completed',detail:'result',fields:{id:String(body.transactionId),accepted:'1',disabled:'1'}};}
    operation=next;return {requestId:++serial,operation:next,result:'queued',detail:'queued',fields:{}};
  }};
}
test('静载一次启动依次预检、配置、申请、保持，每步只有一次提交',async()=>{
  const fake=fixture();
  const result=await runRemoteWorkflow(draft,{...fake,isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'completed');
  assert.deepEqual(fake.calls,['caps','preflight','result','caps','configure','result','caps','acquire','result','caps','hold','result','caps','release','result']);
});
test('预检失败停止，不下发参数或启动保持',async()=>{
  const fake=fixture('preflight');
  const result=await runRemoteWorkflow(draft,{...fake,isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'failed');assert.deepEqual(fake.calls,['caps','preflight','result']);
});
test('前置完成后设备重启中止，不沿用旧启动意图',async()=>{
  const fake=fixture('',true);
  const result=await runRemoteWorkflow(draft,{...fake,isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'rejected');assert.deepEqual(fake.calls,['caps','preflight','result','caps']);
});
test('调用期间断连，不发送后续步骤',async()=>{
  const fake=fixture();let current=true;
  const result=await runRemoteWorkflow(draft,{request:async body=>{const receipt=await fake.request(body);current=false;return receipt;},isCurrent:()=>current,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'cancelled');assert.deepEqual(fake.calls,['caps']);
});
test('POS带载一次申请、专用动作及释放，辅助操作不借用零增益',async()=>{
  const fake=fixture();const bodies:Record<string,unknown>[]=[];
  const result=await runRemoteWorkflow({...draft,operation:'pos_load',kp:0,kd:0},{
    request:async body=>{bodies.push(body);const receipt=await fake.request(body);if(body.operation==='caps')Object.assign(receipt.fields,{mode:'2',load:'1',load_delta:'1',load_speed:'1'});return receipt;},
    isCurrent:()=>true,wait:async()=>{},observe:()=>{},
  });
  assert.equal(result.result,'completed');
  assert.deepEqual(fake.calls,['caps','acquire','result','caps','pos_load','result','caps','release','result']);
  assert.equal(bodies.find(body=>body.operation==='acquire')?.kp,120);
  assert.equal(bodies.find(body=>body.operation==='pos_load')?.kp,0);
});
test('预检未确认失能即使completed也不能配置或保持',async()=>{
  const fake=fixture();
  const result=await runRemoteWorkflow(draft,{request:async body=>{const receipt=await fake.request(body);if(body.operation==='result')receipt.fields.disabled='0';return receipt;},isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'unknown');assert.deepEqual(fake.calls,['caps','preflight','result']);
});
test('提交HTTP异常只报告未知，绝不重复动作',async()=>{
  const fake=fixture();
  const result=await runRemoteWorkflow(draft,{request:async body=>{if(body.operation==='preflight'){fake.calls.push('preflight');throw new Error('timeout');}return fake.request(body);},isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'unknown');assert.deepEqual(fake.calls,['caps','preflight']);
});
test('模式2启动静载，先受保护切MIT并确认，然后进入静载流程',async()=>{
  const fake=fixture('',false,'2');
  const result=await runRemoteWorkflow(draft,{...fake,isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'completed');
  assert.deepEqual(fake.calls.slice(0,9),['caps','acquire','result','caps','mode_mit','result','caps','release','result']);
  assert.equal(fake.calls.filter(operation=>operation==='hold').length,1);
});
test('模式写入失败不继续静载，旧固件缺少切换能力也不写入',async()=>{
  const fake=fixture('mode_mit',false,'2');
  const result=await runRemoteWorkflow(draft,{...fake,isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'failed');assert.ok(!fake.calls.includes('preflight'));
  const old=fixture('',false,'2');
  const rejected=await runRemoteWorkflow(draft,{request:async body=>{const receipt=await old.request(body);delete receipt.fields.switch;return receipt;},isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(rejected.result,'rejected');assert.deepEqual(old.calls,['caps']);
});
test('旧固件缺少专用POS能力时，在申请控制权之前拒绝',async()=>{
  const fake=fixture('',false,'2');
  const result=await runRemoteWorkflow({...draft,operation:'pos_load',kp:0,kd:0},{...fake,isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'rejected');assert.deepEqual(fake.calls,['caps']);
});
test('模式2待机只进入板端预检，预检失败不释放或重发运动',async()=>{
  const fake=fixture('pos_load',false,'2');
  const result=await runRemoteWorkflow({...draft,operation:'pos_load',kp:0,kd:0},{
    request:async body=>{const receipt=await fake.request(body);if(body.operation==='caps')Object.assign(receipt.fields,{fresh:'0',defer:'1',load:'1',load_delta:'1',load_speed:'1'});return receipt;},
    isCurrent:()=>true,wait:async()=>{},observe:()=>{},
  });
  assert.equal(result.result,'failed');
  assert.deepEqual(fake.calls,['caps','acquire','result','caps','pos_load','result']);
});
test('预检回执不刷新标准反馈；配置仍拒绝缺失反馈',async()=>{
  const fake=fixture();
  const result=await runRemoteWorkflow(draft,{request:async body=>{const receipt=await fake.request(body);if(body.operation==='caps')Object.assign(receipt.fields,{fresh:'0',defer:'1'});return receipt;},isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(result.result,'rejected');assert.deepEqual(fake.calls,['caps','preflight','result','caps']);
});

test('home固定绝对目标与整数身份；申请准备不改变目标，只提交一次',async()=>{
  const fake=fixture('',false,'2');const bodies:Record<string,unknown>[]=[];
  const result=await runRemoteWorkflow({...draft,operation:'home',delta:0,kp:4294967295,kd:4294967294},{
    request:async body=>{bodies.push(body);const receipt=await fake.request(body);if(body.operation==='caps')Object.assign(receipt.fields,{boot:'4294967295',coord:'4294967294',home:'1',load_delta:'1',load_speed:'1'});return receipt;},
    isCurrent:()=>true,wait:async()=>{},observe:()=>{},
  });
  assert.equal(result.result,'completed');
  assert.deepEqual(fake.calls,['caps','acquire','result','caps','home','result','caps','release','result']);
  const submitted=bodies.filter(body=>body.operation==='home');assert.equal(submitted.length,1);
  assert.equal(submitted[0].delta,0);assert.equal(submitted[0].kp,4294967295);assert.equal(submitted[0].kd,4294967294);
  assert.equal(bodies.find(body=>body.operation==='acquire')?.kp,120);
});

test('home旧固件及准备后参考变化均拒绝，不能沿用旧目标',async()=>{
  const home:RemoteDraft={...draft,operation:'home',delta:100,kp:1,kd:3};
  const old=fixture('',false,'2');
  const rejected=await runRemoteWorkflow(home,{...old,isCurrent:()=>true,wait:async()=>{},observe:()=>{}});
  assert.equal(rejected.result,'rejected');assert.deepEqual(old.calls,['caps']);
  const fake=fixture('',false,'2');let count=0;
  const changed=await runRemoteWorkflow(home,{
    request:async body=>{const receipt=await fake.request(body);if(body.operation==='caps')Object.assign(receipt.fields,{coord:++count===1?'3':'4',home:'1',load_delta:'1',load_speed:'1'});return receipt;},
    isCurrent:()=>true,wait:async()=>{},observe:()=>{},
  });
  assert.equal(changed.result,'rejected');assert.deepEqual(fake.calls,['caps','acquire','result','caps']);
});

test('home超时与缺失失能证明锁定，绝不释放或重发',async()=>{
  const fake=fixture('',false,'2');
  const result=await runRemoteWorkflow({...draft,operation:'home',delta:100,kp:1,kd:3},{
    request:async body=>{const receipt=await fake.request(body);if(body.operation==='caps')Object.assign(receipt.fields,{coord:'3',home:'1',load_delta:'1',load_speed:'1'});if(body.operation==='result'&&fake.calls.includes('home'))receipt.fields.disabled='0';return receipt;},
    isCurrent:()=>true,wait:async()=>{},observe:()=>{},
  });
  assert.equal(result.result,'unknown');assert.deepEqual(fake.calls,['caps','acquire','result','caps','home','result']);
});

test('POS确定失败与失能确认分开：保留失败原因、原事务和运动锁，不释放或重发',async()=>{
  const fake=fixture('pos_load',false,'2');
  const result=await runRemoteWorkflow({...draft,operation:'pos_load',kp:0,kd:0},{
    request:async body=>{
      const receipt=await fake.request(body);
      if(body.operation==='caps')Object.assign(receipt.fields,{load:'1',load_delta:'1',load_speed:'1'});
      if(body.operation==='result'&&receipt.result==='failed')Object.assign(receipt.fields,{reason:'9',disabled:'0',stage:'1',error:'15',motor:'7'});
      return receipt;
    },
    isCurrent:()=>true,wait:async()=>{},observe:()=>{},
  });
  assert.equal(result.result,'failed');
  assert.ok(result.transactionId>0,'原事务仍供只读查询，页面必须保持运动锁');
  assert.equal(result.operation,'pos_load');
  assert.match(result.detail,/失能未确认/);
  assert.match(result.detail,/无故障/);
  assert.match(result.detail,/9/);
  assert.deepEqual(fake.calls,['caps','acquire','result','caps','pos_load','result']);
});

test('旧固件失败回执没有阶段字段时，仍保留确定失败与未确认失能',async()=>{
  const fake=fixture('preflight');
  const result=await runRemoteWorkflow(draft,{
    request:async body=>{const receipt=await fake.request(body);if(body.operation==='result')Object.assign(receipt.fields,{reason:'9',disabled:'0'});return receipt;},
    isCurrent:()=>true,wait:async()=>{},observe:()=>{},
  });
  assert.equal(result.result,'failed');assert.ok(result.transactionId>0);
  assert.match(result.detail,/失能未确认/);
  assert.deepEqual(fake.calls,['caps','preflight','result']);
});

