/** @file 示波器空采样与设备诊断状态回归；不连接硬件。 */
import assert from 'node:assert/strict';
import test from 'node:test';
import React from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { Chart } from '../src/components/Chart';
import { explainFeedback } from '../src/feedbackStatus';
/** 验证空采样不会退化为空白网格。 */
test('所有信号未知时解释等待状态，真实零值仍绘图', () => {
  const unknown = { timeMs: 1000, positionDeg: null, targetDeg: null, speedDegS: null, torqueNm: null, mosC: null, rotorC: null };
  assert.match(renderToStaticMarkup(React.createElement(Chart, { samples: [unknown] })), /等待有效采样/);
  assert.match(renderToStaticMarkup(React.createElement(Chart, { samples: [{ ...unknown, positionDeg: 0 }] })), /位置反馈曲线/);
});
/** 分开说明USB查询成功、板端缓存年龄与电机反馈资格。 */
test('过期和从未收到的反馈给出不同原因', () => {
  const base = { id: 7, state: 'absent', ageMs: null, receivedAt: new Date().toISOString(), raw: 'ok 9 show motor joint=7 age_ms=2754802', positionDeg: null, speedDegS: null, torqueNm: null, mosC: null, rotorC: null, fault: null, pmaxDeg: null };
  assert.match(explainFeedback(base, true, Date.now()).detail, /2754/);
  assert.match(explainFeedback({ ...base, raw: 'age_ms=4294967295' }, true, Date.now()).title, /未收到/);
  assert.match(explainFeedback(base, false, Date.now()).title, /连接/);
  assert.match(explainFeedback({ ...base, state: 'disabled', ageMs: 0, raw: '' }, true, Date.now()).title, /有效/);
  assert.match(explainFeedback({ ...base, state: 'disabled', ageMs: 0, receivedAt: 'invalid', raw: '' }, true, Date.now()).title, /未知/);
});

/** 旧寄存器没有契约时仅诊断，0x51不再进入主曲线。 */
test('mode2 unqualified raw registers keep main feedback unknown', () => {
  const now=Date.now();
  const motor={id:7,state:'absent',positionDeg:null,speedDegS:null,torqueNm:null,mosC:null,rotorC:null,fault:null,ageMs:null,pmaxDeg:null,receivedAt:new Date(now).toISOString(),raw:'age_ms=4294967295',register50Raw:0.25,register50AgeMs:5,register51Raw:0};
  const health=explainFeedback(motor,true,now);
  assert.match(health.title,/诊断/);
  assert.equal(health.fresh,false);
  assert.match(explainFeedback({...motor,pmContract:1 as const},true,now+600).title,/过期/);
  const markup=renderToStaticMarkup(React.createElement(Chart,{samples:[{timeMs:0,positionDeg:null,targetDeg:null,speedDegS:null,torqueNm:null,mosC:null,rotorC:null,register51Raw:0.25}]}));
  assert.match(markup,/等待有效采样/);
  assert.doesNotMatch(markup,/<circle/);
});
