/** @file Pure experimental torque-policy parsing tests; no network or hardware. */
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {parseTorqueEnvelope,describeTorqueEnvelopeReason,formatTorqueNm,torqueAllowanceSeconds,torqueEnvelopeBlockers} from '../src/torqueEnvelope';
import type {RemoteReceipt} from '../src/remoteControl';
const receipt:RemoteReceipt={requestId:1,operation:'torque',result:'policy',detail:'synthetic',fields:{experimental:'1',motor:'7',axis:'output',continuous:'3.5',peak:'5.46',window_ms:'10000',budget:'175.616',used:'0',authorized:'1',can_reset:'1',token:'17',boot:'3',coord:'11',reset_temp:'35',evidence_ms:'5000',motion_fresh_ms:'100'}};
const adjustable:RemoteReceipt={...receipt,fields:{...receipt.fields,policy_v:'2',min_mnm:'3500',max_mnm:'6240',peak_mnm:'6240',peak:'6.24',blockers:'0',experiment_fresh_ms:'20'}};
test('毫牛米边界显示无损且恒定80%配额计算为6.58秒',()=>{
  assert.equal(formatTorqueNm(3.501),'3.501');assert.equal(formatTorqueNm(3.579),'3.579');
  assert.equal(formatTorqueNm(6.24),'6.24');assert.equal(torqueAllowanceSeconds(6240).toFixed(2),'6.58');
  assert.ok(torqueAllowanceSeconds(6240,100)<torqueAllowanceSeconds(6240));
  assert.equal(torqueAllowanceSeconds(3500),0);assert.ok(Number.isNaN(torqueAllowanceSeconds(6241)));
  const policy=parseTorqueEnvelope({...adjustable,fields:{...adjustable.fields,blockers:'544',can_reset:'0',token:'0'}})!;
  assert.equal(torqueEnvelopeBlockers(policy).length,2);assert.match(torqueEnvelopeBlockers(policy).join(' '),/35℃/);
});

test('新版策略接受80%边界并保留整数单位和阻塞证据',()=>{
  const policy=parseTorqueEnvelope(adjustable);
  assert.ok(policy);assert.equal(policy.peakNm,6.24);assert.equal(policy.configurable,true);
  assert.equal(policy.peakMilliNm,6240);assert.equal(policy.blockers,0);
  const blocked=parseTorqueEnvelope({...adjustable,fields:{...adjustable.fields,blockers:'544',can_reset:'0',token:'0'}});
  assert.ok(blocked);assert.equal(blocked.blockers,544);
});

test('新版协议拒绝错范围、跨字段矛盾、未知版本和未知阻塞位',()=>{
  for(const fields of [{policy_v:'3'},{min_mnm:'3000'},{max_mnm:'7800'},{peak_mnm:'6241'},
    {peak_mnm:'6240.0'},{peak_mnm:'5460'},{blockers:'2048'},{blockers:'32'},
    {can_reset:'0',blockers:'0'},{reset_temp:'100'},{experiment_fresh_ms:'100'}] as Record<string,string>[])
    assert.equal(parseTorqueEnvelope({...adjustable,fields:{...adjustable.fields,...fields}}),null,JSON.stringify(fields));
  const missing={...adjustable.fields};delete missing.blockers;
  assert.equal(parseTorqueEnvelope({...adjustable,fields:missing}),null);
});

test('旧固定策略可读但不假装支持可调，原因采用当次边界',()=>{
  assert.equal(parseTorqueEnvelope(receipt)?.configurable,false);
  const revoked=parseTorqueEnvelope({...adjustable,fields:{...adjustable.fields,authorized:'0',reason:'2'}});
  assert.ok(revoked);assert.match(describeTorqueEnvelopeReason(revoked),/6.24/);
});
test('board policy values retain output axis, budget and explicit maintenance identity',()=>{
  const policy=parseTorqueEnvelope(receipt);
  assert.ok(policy);assert.equal(policy.peakNm,5.46);assert.equal(policy.windowSeconds,10);
  assert.equal(policy.canReset,true);assert.equal(policy.token,17);
  assert.equal(parseTorqueEnvelope({...receipt,fields:{...receipt.fields,authorized:'0',used:'175.616',token:'0',can_reset:'0'}})?.authorized,false);
});
test('old firmware, wrong axis, missing identity and malformed exposure cannot enable maintenance',()=>{
  const invalidFields:Record<string,string>[]=[{axis:'flange'},{peak:'10'},{used:'NaN'},{used:'-1'},{used:'176'},{token:'1.5'},{boot:'0'},{experimental:'0'},{can_reset:'1',token:'0'}];
  for(const candidate of [
    {...receipt,result:'accepted'}, {...receipt,operation:'caps'},
    ...invalidFields.map(fields=>({...receipt,fields:{...receipt.fields,...fields}})),
  ])assert.equal(parseTorqueEnvelope(candidate),null);
});

test('撤销原因5不冒充首故障或十秒超额记录，未知原因如实显示',()=>{
  const revoked=parseTorqueEnvelope({...receipt,fields:{...receipt.fields,reason:'5',authorized:'0',used:'175.616'}});
  assert.ok(revoked);assert.equal(revoked.reason,5);
  assert.match(describeTorqueEnvelopeReason(revoked),/其他故障/);
  assert.match(describeTorqueEnvelopeReason(revoked),/首故障/);
  const unknown=parseTorqueEnvelope({...receipt,fields:{...receipt.fields,reason:'77',authorized:'0'}});
  assert.ok(unknown);assert.match(describeTorqueEnvelopeReason(unknown),/77/);
  assert.match(describeTorqueEnvelopeReason(parseTorqueEnvelope({...receipt,fields:{...receipt.fields,reason:'0',authorized:'0'}})!),/重启/);
  assert.equal(parseTorqueEnvelope({...receipt,fields:{...receipt.fields,reason:'NaN'}}),null);
});
