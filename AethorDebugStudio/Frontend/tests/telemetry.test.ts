/** @file 工程遥测换算与来源隔离回归；只使用离线样本。 */
import assert from 'node:assert/strict';
import test from 'node:test';
import * as telemetry from '../src/feedbackStatus';
import { parseRecording, sampleSegments, toCsv } from '../src/domain';
import type { MotorSnapshot, Sample } from '../src/types';
const empty: Sample = {timeMs:0,positionDeg:null,targetDeg:null,speedDegS:null,torqueNm:null,mosC:null,rotorC:null};
const now = 100000;
const motor: MotorSnapshot = {id:7,state:'absent',positionDeg:null,speedDegS:null,torqueNm:null,mosC:null,rotorC:null,fault:null,ageMs:null,pmaxDeg:null,raw:'',receivedAt:new Date(now).toISOString(),register50Raw:44.8*Math.PI/180,register50AgeMs:10,register51Raw:0,register51AgeMs:10,pmContract:1,pmVelocityRadS:44.8*Math.PI/180,pmBoot:41,pmEpoch:7};
test('ID7 0x50位置和板端估算速度按转子比例换算法兰，不采用0x51零值',()=>{
  const sample=telemetry.captureTelemetry(motor,true,now,0);
  const flange=telemetry.projectTelemetry(sample,'flange','native',7);
  assert.ok(Math.abs(flange.positionDeg!-1)<1e-10);
  assert.ok(Math.abs(flange.speedDegS!-1)<1e-10);
  assert.equal(flange.positionSource,'register50'); assert.equal(flange.speedSource,'estimated');
  assert.equal(flange.torqueNm,null); assert.equal(flange.mosC,null);
  assert.equal(sample.positionDeg,null); assert.equal(sample.speedDegS,null);
});
test('标准位置为转子角度而标准速度为输出轴速度，转换不叠乘比例',()=>{
  const sample={...empty,positionDeg:44.8,speedDegS:44.8/19.2032};
  const flange=telemetry.projectTelemetry(sample,'flange','native',7);
  const output=telemetry.projectTelemetry(sample,'output','native',7);
  assert.ok(Math.abs(flange.positionDeg!-1)<1e-10); assert.ok(Math.abs(flange.speedDegS!-1)<1e-10);
  assert.equal(output.positionDeg,44.8/19.2032); assert.equal(output.speedDegS,sample.speedDegS);
  assert.equal(flange.speedSource,'standard'); assert.equal(flange.standardPositionDeg,44.8);
});
test('旧固件、错误轴、断连和过期寄存器不升级工程位置或估算速度',()=>{
  for(const changed of [{...motor,pmContract:null},{...motor,id:6},{...motor,register50AgeMs:500}]) {
    const result=telemetry.projectTelemetry(telemetry.captureTelemetry(changed,true,now,0),'output','native',changed.id);
    assert.equal(result.positionDeg,null); assert.equal(result.speedDegS,null);
  }
  for(const captured of [telemetry.captureTelemetry(motor,false,now,0),telemetry.captureTelemetry(motor,true,now+500,0),telemetry.captureTelemetry(motor,true,now-1,0)]) {
    assert.equal(telemetry.projectTelemetry(captured,'flange','native',7).positionDeg,null);
  }
});
test('不按主机时间差计算速度，缺失板端估算值保持未知',()=>{
  const sample=telemetry.captureTelemetry({...motor,pmVelocityRadS:null},true,now,0);
  assert.equal(telemetry.projectTelemetry(sample,'output','native',7).speedDegS,null);
  assert.equal(telemetry.projectTelemetry({...sample,register50Raw:999},'output','native',7).speedDegS,null);
});
test('坐标切换纯投影整段历史且来源切换断线',()=>{
  const first=telemetry.captureTelemetry(motor,true,now,0);
  const second={...empty,timeMs:20,positionDeg:44.8,speedDegS:1};
  const projected=[first,second].map(sample=>telemetry.projectTelemetry(sample,'flange','native',7));
  assert.equal(sampleSegments(projected,'positionDeg',250).length,2);
  assert.equal(first.positionDeg,null);
  assert.equal(telemetry.projectTelemetry(first,'output','native',7).positionDeg,44.8/19.2032);
});
test('工程记录和CSV保留原始坐标、契约、估算来源及独立年龄用于重现',()=>{
  const sample=telemetry.projectTelemetry(telemetry.captureTelemetry(motor,true,now,0),'flange','native',7);
  const record={schema:'aethor-studio-recording/v1',name:'telemetry',source:'device',coordinate:'flange',motorId:7,createdAt:new Date(now).toISOString(),samples:[sample],events:[]};
  const parsed=parseRecording(JSON.stringify(record));
  assert.equal(parsed.samples[0].pmContract,1); assert.equal(parsed.samples[0].speedSource,'estimated');
  assert.equal(parsed.samples[0].register50AgeMs,10);
  assert.match(toCsv(parsed),/pm_velocity_rotor_rad_s/); assert.match(toCsv(parsed),/estimated/);
  assert.throws(()=>parseRecording(JSON.stringify({...record,samples:[{...sample,pmContract:2}]})));
});
test('已有输出轴记录不冒充转子原始字段，超大原始值不生成无穷工程值',()=>{
  const existing=telemetry.projectTelemetry({...empty,positionDeg:10,speedDegS:2},'flange','output',7);
  assert.equal(existing.standardPositionDeg,null); assert.equal(existing.standardSpeedDegS,null);
  const huge=telemetry.projectTelemetry({...empty,register50Raw:1e308,register50AgeMs:0,pmContract:1,pmVelocityRadS:1e308},'flange','native',7);
  assert.equal(huge.positionDeg,null); assert.equal(huge.speedDegS,null);
});
test('同一COM的启动号或采样代次变化断线，新契约缺身份只显示单点',()=>{
  const first=telemetry.projectTelemetry(telemetry.captureTelemetry(motor,true,now,0),'output','native',7);
  assert.equal(first.pmBoot,41); assert.equal(first.pmEpoch,7);
  assert.equal(sampleSegments([first,{...first,timeMs:100}],'positionDeg',2500).length,1);
  for (const changed of [{...first,pmBoot:42},{...first,pmEpoch:8},{...first,pmBoot:null},{...first,pmEpoch:null}]) {
    for (const signal of ['positionDeg','speedDegS'] as const)
      assert.equal(sampleSegments([first,{...changed,timeMs:100}],signal,2500).length,2);
  }
  const unidentified={...first,pmBoot:null,pmEpoch:null};
  assert.equal(sampleSegments([unidentified,{...unidentified,timeMs:100}],'positionDeg',2500).length,2);
  assert.notEqual(unidentified.positionDeg,null);
  assert.equal(sampleSegments([{...empty,positionDeg:1},{...empty,timeMs:100,positionDeg:2}],'positionDeg',2500).length,1);
});
test('寄存器曲线即使上层门限为2500ms也不会跨过500ms过期窗口',()=>{
  const first=telemetry.projectTelemetry(telemetry.captureTelemetry(motor,true,now,0),'flange','native',7);
  for(const timeMs of [500,1000,2000]) {
    assert.equal(sampleSegments([first,{...first,timeMs}],'positionDeg',2500).length,2);
    assert.equal(sampleSegments([first,{...first,timeMs}],'speedDegS',2500).length,2);
  }
});
test('启动号和代次CSV/JSON可往返，拒绝负数小数及uint溢出',()=>{
  const sample=telemetry.captureTelemetry(motor,true,now,0);
  const record={schema:'aethor-studio-recording/v1',name:'identity',source:'device',coordinate:'native',motorId:7,createdAt:new Date(now).toISOString(),samples:[sample],events:[]};
  const parsed=parseRecording(JSON.stringify(record));
  assert.equal(parsed.samples[0].pmBoot,41); assert.equal(parsed.samples[0].pmEpoch,7);
  assert.match(toCsv(parsed),/pm_boot,pm_epoch/);
  for(const invalid of [-1,1.1,4294967296]) assert.throws(()=>parseRecording(JSON.stringify({...record,samples:[{...sample,pmBoot:invalid}]})));
});

test('标准反馈保留坐标身份及原始数值，投影和JSON/CSV不丢参考',()=>{
  const captured=telemetry.captureTelemetry({...motor,state:'disabled',ageMs:0,positionDeg:448,speedDegS:2,pmContract:null,coordinateBoot:4294967295,coordinateReference:4294967294},true,now,0);
  assert.equal(captured.positionDeg,448);assert.equal(captured.speedDegS,2);
  assert.equal(captured.coordinateBoot,4294967295);assert.equal(captured.coordinateReference,4294967294);
  const projected=telemetry.projectTelemetry(captured,'flange','native',7);
  assert.equal(projected.coordinateBoot,4294967295);assert.equal(projected.coordinateReference,4294967294);
  const record={schema:'aethor-studio-recording/v1',name:'coordinate identity',source:'device',coordinate:'native',motorId:7,createdAt:new Date(now).toISOString(),samples:[captured],events:[]};
  const parsed=parseRecording(JSON.stringify(record));
  assert.equal(parsed.samples[0].coordinateBoot,4294967295);assert.equal(parsed.samples[0].coordinateReference,4294967294);
  assert.equal(parsed.samples[0].positionDeg,448);assert.equal(parsed.samples[0].speedDegS,2);
  const csv=toCsv(parsed);assert.match(csv,/coordinate_boot,coordinate_reference/);assert.match(csv,/4294967295,4294967294/);assert.match(csv,/position_reference=raw_unshifted/);
  for(const key of ['coordinateBoot','coordinateReference'] as const)
    for(const invalid of [0,-1,1.1,4294967296,'7'])assert.throws(()=>parseRecording(JSON.stringify({...record,samples:[{...captured,[key]:invalid}]})));
  const unknown=parseRecording(JSON.stringify({...record,samples:[{...captured,coordinateReference:null}]}));
  assert.equal(unknown.samples[0].coordinateReference,null);
});

test('坐标身份变化或现代身份缺失时位置目标速度断线，旧无身份记录仍连续',()=>{
  const first:Sample={...empty,positionDeg:1,targetDeg:2,speedDegS:3,coordinateBoot:7,coordinateReference:4};
  for(const signal of ['positionDeg','targetDeg','speedDegS'] as const){
    assert.equal(sampleSegments([first,{...first,timeMs:100}],signal,2500).length,1);
    for(const changed of [{...first,coordinateBoot:8},{...first,coordinateReference:5},{...first,coordinateBoot:null},{...first,coordinateReference:undefined},{...first,coordinateBoot:null,coordinateReference:null}])
      assert.equal(sampleSegments([first,{...changed,timeMs:100}],signal,2500).length,2);
    const missing={...first,coordinateBoot:null,coordinateReference:null};
    assert.equal(sampleSegments([missing,{...missing,timeMs:100}],signal,2500).length,2);
    const legacy={...empty,positionDeg:1,targetDeg:2,speedDegS:3};
    assert.equal(sampleSegments([legacy,{...legacy,timeMs:100}],signal,2500).length,1);
    assert.equal(sampleSegments([legacy,{...first,timeMs:100}],signal,2500).length,2);
  }
});

test('采集无效坐标身份保持未知，旧快照不伪造现代身份',()=>{
  const legacy=telemetry.captureTelemetry(motor,true,now,0);
  assert.equal(Object.hasOwn(legacy,'coordinateBoot'),false);assert.equal(Object.hasOwn(legacy,'coordinateReference'),false);
  for(const invalid of [0,-1,1.1,4294967296,NaN]){
    const captured=telemetry.captureTelemetry({...motor,coordinateBoot:invalid,coordinateReference:invalid},true,now,0);
    assert.equal(captured.coordinateBoot,null);assert.equal(captured.coordinateReference,null);
  }
});
