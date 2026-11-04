/** @file 领域行为负例；只使用合成输入，不连接设备。 */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { validateSettings, feedbackAge, canExecute, parseRecording, toCsv, sampleSegments, convertAngle } from '../src/domain.ts';
import { createDemoRecording } from '../src/demo.ts';
import { traceFixture } from './trace-fixture.ts';
const settings = { motorId: 7, angleDeg: 1, speedDegS: 1, durationS: 1, torqueFfNm: 0, massG: 0, leverMm: 0, kp: 20, kd: 1 };
const recording = { schema: 'aethor-studio-recording/v1', name: '样本', source: 'synthetic', coordinate: 'output', motorId: 7, createdAt: '2026-10-26T00:00:00Z', samples: [{ timeMs: 0, positionDeg: 0, targetDeg: 1, speedDegS: 0, torqueNm: 0, mosC: 25, rotorC: 25 }], events: [] };
test('拒绝非有限数、空数字、超界值及未验证轴的大角度', () => {
  assert.equal(validateSettings('static', settings).length, 0);
  assert.ok(validateSettings('static', { ...settings, torqueFfNm: NaN }).length);
  assert.ok(validateSettings('static', { ...settings, torqueFfNm: 3.51 }).length);
  assert.ok(validateSettings('pos', { ...settings, durationS: 0 }).length);
  assert.ok(validateSettings('pos', { ...settings, motorId: 1, angleDeg: 180 }).length);
});
test('真实与回放模式不能执行演示动作', () => {
  assert.equal(canExecute('demo', false, []).allowed, true);
  assert.equal(canExecute('live', false, []).allowed, false);
  assert.equal(canExecute('replay', false, []).allowed, false);
  assert.equal(canExecute('demo', true, []).allowed, false);
});
test('反馈年龄包含主机等待时间，不将未知变成新鲜', () => {
  assert.equal(feedbackAge(30, 1000, 1080), 110);
  assert.equal(feedbackAge(null, 1000, 1080), null);
  assert.equal(feedbackAge(30, 1100, 1080), null);
});
test('未确认映射不换算法兰量，已确认映射可逆', () => {
  assert.equal(convertAngle(10, null), null);
  assert.equal(convertAngle(20, 2), 10);
  assert.equal(convertAngle(20, 0), null);
});
test('记录导入保留零与未知，限制大小和无效样本', () => {
  assert.equal(parseRecording(JSON.stringify(recording)).samples[0].torqueNm, 0);
  assert.equal(parseRecording(JSON.stringify({ ...recording, samples: [{ ...recording.samples[0], torqueNm: null }] })).samples[0].torqueNm, null);
  assert.throws(() => parseRecording(JSON.stringify({ ...recording, motorId: 8 })));
  assert.throws(() => parseRecording(JSON.stringify({ ...recording, samples: [{ ...recording.samples[0], timeMs: -1 }] })));
  assert.throws(() => parseRecording(JSON.stringify({ ...recording, samples: [{ ...recording.samples[0], torqueNm: '1' }] })));
  assert.throws(() => parseRecording(' '.repeat(2_000_001)));
  assert.throws(() => parseRecording(JSON.stringify({ ...recording, source: 'hardware_qualified' })));
});
test('时间倒退被拒绝，空洞和空值使曲线断开', () => {
  assert.throws(() => parseRecording(JSON.stringify({ ...recording, samples: [{ ...recording.samples[0], timeMs: 5 }, recording.samples[0]] })));
  const samples = [0, 20, 40, 500].map((timeMs) => ({ ...recording.samples[0], timeMs }));
  assert.equal(sampleSegments(samples, 'positionDeg', 100).length, 2);
});
test('CSV含坐标和来源，字符串防公式注入', () => {
  const csv = toCsv({ ...recording, name: '=SUM(1,2)' } as ReturnType<typeof parseRecording>);
  assert.ok(csv.includes('coordinate=output'));
  assert.ok(csv.includes('source=synthetic'));
  assert.ok(csv.includes('time_ms'));
});

/** 寄存器字段独立导出，旧记录缺省与明确未知均保留。 */
test('原始位置记录保留零和缺失，拒绝伪数字', () => {
  const rawSample = {...recording.samples[0],positionDeg:null,register50Raw:null,register51Raw:0};
  const parsed = parseRecording(JSON.stringify({...recording,samples:[rawSample]}));
  assert.equal(parsed.samples[0].register51Raw,0);
  assert.equal(parsed.samples[0].register50Raw,null);
  assert.equal(parsed.samples[0].positionDeg,null);
  assert.match(toCsv(parsed),/register50_raw,register51_raw/);
  assert.ok(toCsv(parsed).endsWith(',0'));
  assert.equal(parseRecording(JSON.stringify(recording)).samples[0].register51Raw,undefined);
  assert.throws(() => parseRecording(JSON.stringify({...recording,samples:[{...rawSample,register51Raw:'0.25'}]})));
});
test('未知JSON与缺少schema的伪造记录被拒绝', () => {
  assert.throws(() => parseRecording('{}'));
  assert.throws(() => parseRecording('null'));
  assert.throws(() => parseRecording(JSON.stringify({ ...recording, coordinate: 'joint-assumed' })));
});
test('ID7的POS演示保留法兰口径，静载不生成移动目标', () => {
  assert.equal(createDemoRecording(settings,'pos').coordinate,'flange');
  const hold=createDemoRecording(settings,'static');
  assert.equal(hold.coordinate,'output');
  assert.equal(new Set(hold.samples.map(sample=>sample.targetDeg)).size,1);
});
test('显式记录缺口即使小于时间门槛也会断线', () => {
  const samples=[0,20,40].map(timeMs=>({...recording.samples[0],timeMs}));
  assert.equal(sampleSegments(samples,'positionDeg',250,[30]).length,2);
});
test('原始RX不当作采纳反馈，永久首故障槽和序号缺口被保留', () => {
  const trace=parseRecording(JSON.stringify(traceFixture));
  assert.equal(trace.samples.length,2);
  assert.equal(trace.samples[0].torqueNm,0);
  assert.ok(Math.abs(trace.samples[0].positionDeg!-180/Math.PI)<1e-9);
  assert.equal(trace.coordinate,'native');
  assert.ok(trace.events.some(event=>event.gap));
  assert.ok(trace.events.some(event=>event.title.includes('永久保留槽')&&event.level==='error'));
});
