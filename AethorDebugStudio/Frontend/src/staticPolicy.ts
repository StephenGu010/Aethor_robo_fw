/** @file F聊天及2026-10-26发布工件的显示基线；不发送配置、不授予硬件资格。 */
import type { Recording } from './types';

export const STATIC_RELEASE = Object.freeze({
  policyId: 'id7-static-window-20261026',
  firmwareHash: '3281d1a80777d95a4b155ff1b212623cdac72668d7b54973832e76bc2dfb0fdb',
  date: '2026-10-26',
  precisionDeg: .5,
  takeoverGuardDeg: .5,
  holdGuardDeg: 1,
  torqueTripNm: 3.5,
  kpOutput: 80,
  kdOutput: .2,
  nativePositionRatio: 19.2032,
  totalFlangeRatio: 44.8,
});
export const GAIN_CANDIDATES = [
  { label:'80 / 0.2', kp:80, kd:.2, released:true },
  { label:'100 / 0.3', kp:100, kd:.3, released:false },
  { label:'120 / 0.3', kp:120, kd:.3, released:false },
  { label:'120 / 0.4', kp:120, kd:.4, released:false },
  { label:'160 / 0.4', kp:160, kd:.4, released:false },
];
/** 同向叠加的连续公式名义预算；不含编码量化、动态和停止余量。 */
export function nominalBudget(kp:number,kd:number,errorDeg:number,speedErrorRadS:number,feedforwardNm:number) {
  if (![kp,kd,errorDeg,speedErrorRadS,feedforwardNm].every(Number.isFinite) || kp<0 || kd<0) throw new Error('预算输入必须为有限数，增益不能为负');
  const position=kp*Math.abs(errorDeg)*Math.PI/180;
  const damping=kd*Math.abs(speedErrorRadS);
  const feedforward=Math.abs(feedforwardNm);
  const total=position+damping+feedforward;
  return {position,damping,feedforward,total,remaining:STATIC_RELEASE.torqueTripNm-total};
}
/** 已显示窗口的误差观察；不覆盖旧策略、未知坐标或其他轴。 */
export function observeStatic(recording:Recording) {
  const context=recording.staticContext;
  const bound=recording.motorId===7 && recording.coordinate==='output' && context?.policyId===STATIC_RELEASE.policyId;
  const peaks=recording.samples.map(sample=>sample.torqueNm).filter((value):value is number=>value!==null&&Number.isFinite(value));
  const peakTorqueNm=peaks.length?Math.max(...peaks.map(Math.abs)):null;
  let precision:'within'|'exceeded'|'unknown'='unknown';
  let protection:'within'|'exceeded'|'unknown'='unknown';
  let maxHoldErrorDeg:number|null=null;
  let firstCrossingMs:number|null=null;
  if(bound && context)for(const sample of recording.samples){
    if(sample.positionDeg===null||sample.targetDeg===null||!Number.isFinite(sample.positionDeg)||!Number.isFinite(sample.targetDeg))continue;
    const error=Math.abs(sample.positionDeg-sample.targetDeg);
    const holding=sample.timeMs>=context.holdStartMs;
    if(holding){maxHoldErrorDeg=Math.max(maxHoldErrorDeg??0,error);if(precision!=='exceeded')precision=error>STATIC_RELEASE.precisionDeg?'exceeded':'within';}
    const guard=holding?STATIC_RELEASE.holdGuardDeg:STATIC_RELEASE.takeoverGuardDeg;
    if(protection!=='exceeded')protection=error>guard?'exceeded':'within';
    if(error>guard&&firstCrossingMs===null)firstCrossingMs=sample.timeMs;
  }
  return {bound,precision,protection,maxHoldErrorDeg,firstCrossingMs,peakTorqueNm};
}
