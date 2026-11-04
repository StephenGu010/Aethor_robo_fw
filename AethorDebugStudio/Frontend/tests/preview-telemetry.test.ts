/** @file 总览及流程预览使用一致工程单位和新鲜度；仅服务端渲染离线夹具。 */
import assert from 'node:assert/strict';
import test from 'node:test';
import React from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { Overview, MotorTable } from '../src/pages/Overview';
import { Workflow } from '../src/pages/Workflow';
import type { Studio } from '../src/useStudio';
/** 构造ID7转子19.2032度等于输出轴1度的实时快照。 */
function fixture(): Studio {
  const now=Date.now();
  const sample={timeMs:0,positionDeg:19.2032,targetDeg:null,speedDegS:1,torqueNm:null,mosC:null,rotorC:null};
  return {mode:'live',now,running:false,settings:{motorId:7,angleDeg:1,speedDegS:1,durationS:1,torqueFfNm:0,massG:0,leverMm:0,kp:20,kd:1},
    samples:[sample,{...sample,timeMs:1000}],workflow:{stage:'idle',startedAt:0,frozen:null},
    recording:{schema:'aethor-studio-recording/v1',source:'device',coordinate:'native',motorId:7,createdAt:new Date(now).toISOString(),name:'preview',samples:[sample],events:[]},
    gateway:{connected:true,status:'test',motors:[{id:7,state:'disabled',positionDeg:19.2032,speedDegS:1,torqueNm:null,mosC:null,rotorC:null,fault:null,ageMs:0,receivedAt:new Date(now).toISOString(),pmaxDeg:null,raw:''}],lines:[],info:{}}} as unknown as Studio;
}
test('总览使用输出轴、POS流程使用法兰工程角度且500ms缺口断线',()=>{
  const studio=fixture();
  const overview=renderToStaticMarkup(React.createElement(Overview,{studio,navigate:()=>{}}));
  const workflow=renderToStaticMarkup(React.createElement(Workflow,{studio,flow:'pos'}));
  for(const markup of [overview]) {
    assert.match(markup,/电机输出轴/); assert.doesNotMatch(markup,/<polyline/);
    assert.doesNotMatch(markup,/>19\.20</); assert.match(markup,/>1\.0[0-9]</);
  }
  assert.match(workflow,/法兰端/);assert.doesNotMatch(workflow,/<polyline|>19\.20</);
  assert.doesNotMatch(workflow,/远程未接入|真机运动与参数下发尚未接入/);
  assert.match(workflow,/真实操作见手动调试/);
});
test('离线POS旧输出轴记录转换法兰预览，不改写记录原坐标',()=>{
  const studio=fixture();studio.mode='demo';studio.recording.coordinate='output';
  const markup=renderToStaticMarkup(React.createElement(Workflow,{studio,flow:'pos'}));
  assert.match(markup,/法兰端/);assert.doesNotMatch(markup,/>19\.20</);
  assert.equal(studio.recording.coordinate,'output');
});
test('表格ID7工程值带来源，旧反馈与未知标定轴不当实时角度',()=>{
  const studio=fixture();
  let markup=renderToStaticMarkup(React.createElement(MotorTable,{studio}));
  assert.match(markup,/>1\.00</); assert.match(markup,/驱动位置/);
  studio.gateway.motors[0].positionDeg=731.25;
  studio.gateway.motors[0].ageMs=2000;
  markup=renderToStaticMarkup(React.createElement(MotorTable,{studio}));
  assert.doesNotMatch(markup,/731\.25/); assert.match(markup,/过期/);
  studio.gateway.motors[0]={...studio.gateway.motors[0],id:6,positionDeg:987.65,ageMs:0};
  markup=renderToStaticMarkup(React.createElement(MotorTable,{studio}));
  assert.doesNotMatch(markup,/987\.65/); assert.match(markup,/标定未确认/);
});
test('表格0x50新鲜位置可显示，断连或过期不能沿用缓存',()=>{
  const studio=fixture();
  const motor=studio.gateway.motors[0];
  Object.assign(motor,{state:'absent',positionDeg:null,ageMs:null,pmContract:1,register50Raw:19.2032*Math.PI/180,register50AgeMs:0});
  let markup=renderToStaticMarkup(React.createElement(MotorTable,{studio}));
  assert.match(markup,/>1\.00</); assert.match(markup,/0x50位置/);
  motor.register50AgeMs=500;
  markup=renderToStaticMarkup(React.createElement(MotorTable,{studio}));
  assert.doesNotMatch(markup,/>1\.00</);
  motor.register50AgeMs=0; studio.gateway.connected=false;
  markup=renderToStaticMarkup(React.createElement(MotorTable,{studio}));
  assert.doesNotMatch(markup,/>1\.00</);
});
