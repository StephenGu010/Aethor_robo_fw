/** @file 可重复的离线数据生成器；不包含真实设备身份或通信。 */
import type { Flow, Recording, Sample, Settings, StudioEvent } from './types';
import { STATIC_RELEASE } from './staticPolicy';
/** 创建明确标识的合成测试记录，保留零位移与力矩的真实数值语义。 */
export function createDemoRecording(settings: Settings, flow: Flow = 'pos'): Recording {
  const start = -26.47;
  const moving = flow === 'pos' || flow === 'manual';
  const moveMs = moving ? Math.min(10000, Math.abs(settings.angleDeg) / settings.speedDegS * 1000) : 0;
  const holdBegin = 1000 + moveMs;
  const endMs = holdBegin + settings.durationS * 1000;
  const samples: Sample[] = [];
  for (let timeMs = 0; timeMs <= endMs; timeMs += 40) {
    const progress = moving ? Math.max(0, Math.min(1, (timeMs - 1000) / Math.max(moveMs, 1))) : 0;
    const target = start + (moving ? settings.angleDeg * progress : 0);
    const envelope = Math.sin(Math.PI * Math.min(1, timeMs / 1000));
    samples.push({ timeMs, positionDeg: target + Math.sin(timeMs / 130) * 0.018, targetDeg: target, speedDegS: progress > 0 && progress < 1 ? Math.sign(settings.angleDeg) * settings.speedDegS : 0.006 * Math.sin(timeMs / 100), torqueNm: (flow==='static'?settings.torqueFfNm:0) * Math.min(1, timeMs / 1000) + 0.12 + 0.022 * Math.sin(timeMs / 190) + envelope * 0.03, mosC: 29 + timeMs / 50000, rotorC: 27 + timeMs / 60000 });
  }
  const events: StudioEvent[] = [
    { timeMs: 0, level: 'info', title: '演示预检完成', detail: '合成反馈 · 所有动作仅在浏览器内执行' },
    { timeMs: 120, level: 'info', title: '冻结本次参数', detail: 'ID' + settings.motorId + ' · 当前目标与配置已冻结' },
    { timeMs: 1000, level: 'info', title: moving ? '模拟移动开始' : '模拟保持开始', detail: moving ? '按草稿方向生成合成位移；长动作预览压缩至10秒' : '保持本次捕获的当前位置' },
    { timeMs: holdBegin, level: 'info', title: '进入模拟保持', detail: '保持计时与准备阶段分别显示' },
    { timeMs: endMs, level: 'info', title: '演示流程完成', detail: '未发送电机命令，不产生硬件资格证明' },
  ];
  const staticContext=flow==='static'&&settings.motorId===7?{policyId:STATIC_RELEASE.policyId,holdStartMs:holdBegin,kpOutput:settings.kp,kdOutput:settings.kd,torqueFfNm:settings.torqueFfNm}:undefined;
  return { schema: 'aethor-studio-recording/v1', name: '离线演示 · ' + (flow === 'static' ? '单轴静载' : flow === 'seven' ? '七轴静载（选中轴）' : 'POS移动保持'), source: 'synthetic', coordinate: moving&&settings.motorId===7?'flange':'output', motorId: settings.motorId, createdAt: new Date().toISOString(), samples, events, staticContext };
}
