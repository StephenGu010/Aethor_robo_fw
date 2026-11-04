/** @file 离线草稿边界测试，禁止触发fetch或连接硬件。 */
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {remoteDraftError, remoteTerminalConfirmed,describeRemoteTerminal, type RemoteDraft, type RemoteReceipt} from '../src/remoteControl';
const draft:RemoteDraft={operation:'pos',delta:.01,speed:.02,kp:120,kd:.3,torque:0,duration:1000};
test('新首故障使用事发退出线，不能按旧5.46解释80%试验',()=>{
  const receipt:RemoteReceipt={requestId:9,operation:'result',result:'failed',fields:{accepted:'1',disabled:'1',reason:'16',motor:'7',tq_reason:'2',tq_auth:'1',tq_used:'0',tq_nm:'6.25',tq_limit:'6.240'},detail:'fixture'};
  assert.match(describeRemoteTerminal(receipt,'pos_load'),/6.24 N·m峰值/);
  assert.match(describeRemoteTerminal({...receipt,fields:{...receipt.fields,tq_nm:'6.2400'}},'pos_load'),/首次转矩保护/);
  assert.match(describeRemoteTerminal({...receipt,fields:{...receipt.fields,tq_nm:'3.5011',tq_limit:'3.501'}},'pos_load'),/3.501 N·m峰值/);
  for(const fields of [{tq_nm:'6.23'},{tq_limit:'NaN'},{tq_limit:'7.8'},{tq_limit:'3.499'}] as Record<string,string>[])
    assert.doesNotMatch(describeRemoteTerminal({...receipt,fields:{...receipt.fields,...fields}},'pos_load'),/首次转矩保护/);
});
test('首个转矩失败快照区分未授权基线、实验峰值、窗口和累计预算',()=>{
  const receipt:RemoteReceipt={requestId:9,operation:'result',result:'failed',fields:{id:'7',accepted:'1',disabled:'1',reason:'16',motor:'7',tq_reason:'1',tq_auth:'0',tq_used:'175.616000',tq_nm:'3.5238'},detail:'fixture'};
  const baseline=describeRemoteTerminal(receipt,'pos_load');
  assert.match(baseline,/首次转矩保护：3.50 N·m基线/);
  assert.match(baseline,/故障前实验配额未授权/);
  assert.match(baseline,/3.524 N·m/);
  for(const [reason,label,torque] of [['2','5.46 N·m峰值','5.4700'],['3','10秒超额窗口','3.6000'],['4','累计超额预算','4.0000']])
    assert.ok(describeRemoteTerminal({...receipt,fields:{...receipt.fields,tq_reason:reason,tq_auth:'1',tq_used:'0.000000',tq_nm:torque}},'pos_load').includes(label));
  const invalidFields:Record<string,string>[]=[{tq_used:'NaN'},{tq_used:'-1'},{tq_used:'176'},{tq_auth:'1'},{tq_reason:'5'},{tq_nm:''},{motor:'6'},{accepted:'0'}];
  for(const fields of invalidFields)
    assert.doesNotMatch(describeRemoteTerminal({...receipt,fields:{...receipt.fields,...fields}},'pos_load'),/首次转矩保护/);
});
test('远程草稿Kp/Kd冻结范围和非有限值',()=>{
  assert.equal(remoteDraftError(draft),null);
  for(const change of [{kp:0},{kp:201},{kd:.19},{kd:.41},{speed:0},{delta:NaN},{torque:Infinity},{duration:0},{duration:1000.5}]) assert.ok(remoteDraftError({...draft,...change}));
});
test('POS带载采用独立零增益参数和整秒保持',()=>{
  const loaded:RemoteDraft={...draft,operation:'pos_load',kp:0,kd:0};
  assert.equal(remoteDraftError(loaded),null);
  for(const change of [{kp:120},{kd:.3},{torque:1},{delta:0},{duration:1500}]) assert.ok(remoteDraftError({...loaded,...change}));
});

test('home启动号和坐标参考必须为非零uint32，保持网关和固件约束一致',()=>{
  const home:RemoteDraft={...draft,operation:'home',kp:7,kd:4,delta:0};
  assert.equal(remoteDraftError(home),null);
  for(const key of ['kp','kd'] as const)
    for(const invalid of [0,-1,1.1,4294967296])assert.ok(remoteDraftError({...home,[key]:invalid}));
  assert.equal(remoteDraftError({...home,kp:4294967295,kd:4294967295}),null);
});
test('终态与恢复共用准入/失能判据，失败清理未确认不得解锁',()=>{
  const terminal:RemoteReceipt={requestId:9,operation:'result',result:'completed',fields:{id:'7',accepted:'1',disabled:'0'},detail:'fixture'};
  assert.equal(remoteTerminalConfirmed(terminal,'preflight'),false);
  assert.equal(remoteTerminalConfirmed(terminal,'hold'),false);
  assert.equal(remoteTerminalConfirmed(terminal,null),false);
  assert.equal(remoteTerminalConfirmed(terminal,'configure'),true);
  assert.equal(remoteTerminalConfirmed({...terminal,result:'failed'},'hold'),false);
  assert.equal(remoteTerminalConfirmed({...terminal,fields:{...terminal.fields,disabled:'1'}},'hold'),true);
  assert.equal(remoteTerminalConfirmed({...terminal,result:'failed',fields:{accepted:'0'}},'hold'),true);
  assert.equal(remoteTerminalConfirmed({...terminal,fields:{}},'hold'),false);
});

test('正常终态的NONE原因与阶段不显示成未知错误',()=>{
  const receipt:RemoteReceipt={requestId:9,operation:'result',result:'completed',fields:{accepted:'1',disabled:'1',reason:'0',stage:'0'},detail:'fixture'};
  assert.doesNotMatch(describeRemoteTerminal(receipt,'pos_load'),/未识别|原因0|阶段码0|运动保持锁定/);
});

test('界面原因9只描述失能门，不凭相同数字编造驱动欠压',()=>{
  const receipt:RemoteReceipt={requestId:9,operation:'result',result:'failed',fields:{accepted:'1',disabled:'0',reason:'9',stage:'1'},detail:'fixture'};
  const detail=describeRemoteTerminal(receipt,'pos_load');
  assert.match(detail,/无故障/);assert.match(detail,/失能未确认/);assert.doesNotMatch(detail,/欠压/);
  assert.equal(remoteTerminalConfirmed(receipt,'pos_load'),false);
});
