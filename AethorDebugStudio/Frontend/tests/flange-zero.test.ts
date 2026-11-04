/** @file 软件零点纯函数回归；不访问网关或改变原始记录。 */
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {captureFlangeZero,flangeZeroCurrent,relativeFlangeSample,zeroIdentity,type FlangeZeroIdentity} from '../src/flangeZero';
import type {MotorSnapshot,Sample} from '../src/types';

const identity:FlangeZeroIdentity={connection:'COM7:1',boot:4294967295,reference:4294967294,contract:'coord'};
const sample:Sample={timeMs:0,positionDeg:-725,targetDeg:-720,speedDegS:5,torqueNm:0,mosC:null,rotorC:null};

test('软件零点保留多圈和负角度，速度和原样本不偏移',()=>{
  const zero=captureFlangeZero(-720,identity,1234)!;
  const relative=relativeFlangeSample(sample,zero);
  assert.equal(relative.positionDeg,-5);assert.equal(relative.targetDeg,0);assert.equal(relative.speedDegS,5);
  assert.equal(sample.positionDeg,-725);assert.equal(zero.createdAt,1234);
});

test('有限数、uint32及缺失身份严格拒绝',()=>{
  assert.equal(captureFlangeZero(NaN,identity,1234),null);
  assert.equal(captureFlangeZero(Infinity,identity,1234),null);
  assert.equal(captureFlangeZero(1,{...identity,boot:4294967296},1234),null);
  assert.equal(captureFlangeZero(1,{...identity,reference:-1},1234),null);
  assert.equal(captureFlangeZero(1,{...identity,boot:0},1234),null);
  assert.equal(captureFlangeZero(1,{...identity,reference:0},1234),null);
  assert.equal(zeroIdentity({id:7,coordinateBoot:0,coordinateReference:1} as MotorSnapshot,'COM7:1'),null);
  assert.equal(zeroIdentity({id:7,coordinateBoot:1,coordinateReference:0} as MotorSnapshot,'COM7:1'),null);
});

test('重连、重启、参考身份不连续失效；暂时反馈过期不丢失身份',()=>{
  const zero=captureFlangeZero(12,identity,1234)!;
  assert.equal(flangeZeroCurrent(zero,identity),true);
  for(const next of [{...identity,connection:'COM7:2'},{...identity,boot:1},{...identity,reference:2},null])assert.equal(flangeZeroCurrent(zero,next),false);
  const motor={id:7,coordinateBoot:identity.boot,coordinateReference:identity.reference,pmContract:1,pmBoot:identity.boot,pmEpoch:3} as MotorSnapshot;
  assert.deepEqual(zeroIdentity(motor,identity.connection),identity);
  assert.deepEqual(zeroIdentity({...motor,ageMs:99999},identity.connection),identity);
  assert.equal(zeroIdentity({...motor,coordinateReference:undefined,pmEpoch:undefined},identity.connection),null);
});

test('旧位置合同不冒充连续坐标身份',()=>{
  const motor={id:7,pmContract:1,pmBoot:5,pmEpoch:6} as MotorSnapshot;
  assert.equal(zeroIdentity(motor,'COM7:1'),null);
  assert.equal(zeroIdentity({...motor,id:6},'COM7:1'),null);
});
