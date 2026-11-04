/** @file 新版静载窗口与预算负例；所有采样均为合成测试。 */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { STATIC_RELEASE, nominalBudget, observeStatic } from '../src/staticPolicy.ts';
import { createDemoRecording } from '../src/demo.ts';
import { DEFAULT_SETTINGS } from '../src/catalog.ts';
import { parseRecording } from '../src/domain.ts';
/** 生成已绑定新窗口策略的单轴记录；不代表实机采集。 */
function fixture(errorDeg:number,timeMs=1200) {
  const record=createDemoRecording(DEFAULT_SETTINGS,'static');
  return {...record,staticContext:{policyId:STATIC_RELEASE.policyId,holdStartMs:1000,kpOutput:80,kdOutput:.2,torqueFfNm:0},samples:[{timeMs,positionDeg:errorDeg,targetDeg:0,speedDegS:0,torqueNm:.681,mosC:25,rotorC:25}]};
}
test('默认值来自已部署等效80/0.2，高增益不成为默认',()=>{assert.equal(DEFAULT_SETTINGS.kp,80);assert.equal(DEFAULT_SETTINGS.kd,.2);});
test('0.75°保持误差记录精度超差，1°内不判保护越界',()=>{
  const result=observeStatic(fixture(.75));assert.equal(result.precision,'exceeded');assert.equal(result.protection,'within');
  assert.equal(observeStatic(fixture(1)).protection,'within');assert.equal(observeStatic(fixture(-1.001)).protection,'exceeded');
});
test('启动接管仍是0.5°，保持0.5°边界不超差',()=>{
  const startup=observeStatic(fixture(.501,500));assert.equal(startup.protection,'exceeded');assert.equal(startup.precision,'unknown');
  assert.equal(observeStatic(fixture(.5)).precision,'within');
});
test('原生、法兰、其他轴和未绑定策略的旧记录不套1°规则',()=>{
  const record=fixture(.75);
  assert.equal(observeStatic({...record,coordinate:'native'}).bound,false);
  assert.equal(observeStatic({...record,coordinate:'flange'}).bound,false);
  assert.equal(observeStatic({...record,motorId:1}).bound,false);
  assert.equal(observeStatic({...record,staticContext:undefined}).bound,false);
});
test('理想预算区分位置、阻尼和前馈，不声称稳定性',()=>{
  const result=nominalBudget(120,.4,1,1,1);
  assert.ok(Math.abs(result.position-120*Math.PI/180)<1e-9);
  assert.ok(result.remaining>0&&result.remaining<.01);
  assert.ok(nominalBudget(120,.4,1.5,1,1).remaining<0);
  assert.throws(()=>nominalBudget(NaN,.2,1,1,1));
});
test('导出导入保留策略上下文，拒绝坏绑定字段',()=>{
  const record=fixture(.75);assert.equal(parseRecording(JSON.stringify(record)).staticContext?.holdStartMs,1000);
  assert.throws(()=>parseRecording(JSON.stringify({...record,staticContext:{...record.staticContext,holdStartMs:-1}})));
});
test('空样本/缺失目标保持未知，峰值使用已显示窗口',()=>{
  const record=fixture(.75);
  assert.equal(observeStatic({...record,samples:[]}).protection,'unknown');
  assert.equal(observeStatic({...record,samples:[{...record.samples[0],targetDeg:null}]}).precision,'unknown');
  assert.equal(observeStatic(record).peakTorqueNm,.681);
});
