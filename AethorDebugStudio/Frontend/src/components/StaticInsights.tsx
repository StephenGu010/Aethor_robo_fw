/** @file 静载窗口、误差观察与名义预算组件；不预测实机稳定性。 */
import { useState } from 'react';
import { ArrowRight, FlaskConical, Info, ShieldCheck } from 'lucide-react';
import { GAIN_CANDIDATES, nominalBudget, observeStatic, STATIC_RELEASE } from '../staticPolicy';
import { sampleSegments } from '../domain';
import { Badge, Note, PanelHeader, Value } from './Common';
import type { Recording, Sample } from '../types';
import {OUTPUT_PER_FLANGE,outputDegreesToDisplayDegrees,outputRadiansToDisplayDegrees,displayDegreesToOutputRadians,type ControlCoordinate} from '../axisUnits';
/** 分开显示精度窗口、保持保护和绝对行程，数值不可在此修改。 */
export function StaticWindows({ motorId,displayCoordinate='output' }: { motorId:number;displayCoordinate?:ControlCoordinate }) {
  const id7=motorId===7;
  const coordinate=id7?displayCoordinate:'output';
  const axisLabel=coordinate==='flange'?'法兰端':'输出轴';
  /** 保持既有窗口判断，只转换用户看到的角度。 */
  const angle=(value:number)=>outputDegreesToDisplayDegrees(value,coordinate).toFixed(coordinate==='flange'?3:1);
  return <section className="panel static-windows"><PanelHeader title="三个窗口，各管一件事" eyebrow="PRECISION · FOLLOWING ERROR · TRAVEL"><Badge>发布策略参考</Badge></PanelHeader>
    <div className="window-grid"><div className="window-item precision"><span>01 / 保持精度指标</span><strong>±{angle(.5)}<small>° {axisLabel}</small></strong><p>超出后记录精度超差，继续观察保持。</p></div><div className="window-item guard"><span>02 / 保持跟随误差保护</span><strong>±{angle(id7?1:.5)}<small>° {axisLabel}</small></strong><p>{id7?'仅ID7进入已证明首批HOLD后的保持阶段。':'其他轴保持原窗口，不套用ID7比例。'}</p></div><div className="window-item travel"><span>03 / 法兰姿态准入</span><strong>{id7?'±180':'未标定'}<small>{id7?'° 法兰':''}</small></strong><p>{id7?'静载为当前绝对姿态准入；POS为相对位移。':'没有确认的法兰映射，不从ID7推导。'}</p></div></div>
    <div className="window-footer"><Info size={14}/><span>{coordinate==='flange'?`启动与前馈接管检查法兰${angle(.5)}°；保持保护法兰${angle(1)}°。显示换算不改变原保护。`:'启动与前馈接管仍检查0.5°；绝对行程、精度和退出边界分别核对。'}</span></div>
  </section>;
}
/** 仅对显式绑定策略和输出轴坐标的记录绘制误差观察。 */
export function StaticObservation({ recording,samples,displayCoordinate='output' }: { recording:Recording; samples:Sample[];displayCoordinate?:ControlCoordinate }) {
  const coordinate=recording.motorId===7?displayCoordinate:'output';
  const axisLabel=coordinate==='flange'?'法兰端':'输出轴';
  /** 误差/刻度仅作仿射显示投影，原始记录和阈值判断不改写。 */
  const angle=(value:number)=>outputDegreesToDisplayDegrees(value,coordinate);
  const visible={...recording,samples};
  const result=observeStatic(visible);
  const holdStart=recording.staticContext?.holdStartMs??0;
  const errors=samples.map(sample=>({...sample,positionDeg:sample.positionDeg===null||sample.targetDeg===null?null:sample.positionDeg-sample.targetDeg}));
  const end=Math.max(1000,samples.at(-1)?.timeMs??1000);
  const x=(time:number)=>54+time/end*800;
  const amplitude=Math.max(1.2,...errors.map(sample=>Math.abs(sample.positionDeg??0)));
  /** 将实际误差范围投影到图幅，不截断峰值。 */
  const y=(value:number)=>112-value/amplitude*80;
  const segments=sampleSegments(errors,'positionDeg',250,recording.events.filter(event=>event.gap).map(event=>event.timeMs));
  return <section className="panel static-observation"><PanelHeader title="精度与保护，分别观察" eyebrow={coordinate==='flange'?'VISIBLE SAMPLES / FLANGE':'VISIBLE SAMPLES / OUTPUT SHAFT'}><Badge tone={result.bound?'blue':'neutral'}>{result.bound?'已绑定发布策略':'坐标 / 策略未绑定'}</Badge></PanelHeader>
    <div className="observation-metrics"><div><span>保持精度指标</span><strong className={result.precision==='exceeded'?'text-amber':''}>{result.precision==='unknown'?'待评定':result.precision==='exceeded'?'可见样本超差':'可见样本未超差'}</strong><small>保持阶段 |误差| &gt; {angle(.5).toFixed(3)}° {axisLabel}</small></div><div><span>跟随误差保护对照</span><strong className={result.protection==='exceeded'?'text-red':''}>{result.protection==='unknown'?'待评定':result.protection==='exceeded'?'样本越过保护线':'可见样本未越界'}</strong><small>仅做记录对照，不重写真实退出原因</small></div><div><span>最大保持误差</span><strong><Value value={result.maxHoldErrorDeg===null?null:angle(result.maxHoldErrorDeg)} digits={3} unit="°"/></strong><small>{axisLabel} · 已显示采样窗口</small></div><div><span>反馈转矩峰值</span><strong><Value value={result.peakTorqueNm} digits={3} unit="N·m"/></strong><small>非恒定目标 · 非全程峰值证明</small></div></div>
    {result.bound&&samples.length?<svg className="error-window-svg" viewBox="0 0 900 235" role="img" aria-label={axisLabel+'跟随误差窗口图'}><rect x="54" y={y(.5)} width="800" height={y(-.5)-y(.5)} fill="#eaf5ef"/>{[-1,-.5,0,.5,1].map(value=><g key={value}><line x1="54" x2="854" y1={y(value)} y2={y(value)} stroke={Math.abs(value)===1?'#d69283':value===0?'#b1c7bf':'#d2b97c'} strokeDasharray={value===0?undefined:'4 4'}/><text x="43" y={y(value)+4} textAnchor="end">{angle(value).toFixed(3)}°</text></g>)}<path d={`M54 ${y(.5)}H${x(Math.min(end,holdStart))}V${y(1)}H854M54 ${y(-.5)}H${x(Math.min(end,holdStart))}V${y(-1)}H854`} fill="none" stroke="#b75c46" strokeWidth="1.5"/>{segments.map((segment,index)=><polyline key={index} points={segment.map(sample=>x(sample.timeMs)+','+y(sample.positionDeg!)).join(' ')} fill="none" stroke="#007f74" strokeWidth="2.3"/>)}<text x="54" y="223">0s</text><text x="854" y="223" textAnchor="end">{(end/1000).toFixed(1)}s</text></svg>:<div className="static-empty"><Info size={20}/><strong>本记录暂不能按新窗口判读</strong><p>需要ID7、输出轴坐标及明确的静载策略绑定。原生/法兰数据与旧记录不会自动套用1°规则。</p></div>}
    <div className="observation-legend"><span><i className="precision-mark"/>{angle(.5).toFixed(3)}°精度带</span><span><i className="guard-mark"/>启动{angle(.5).toFixed(3)}° → 保持{angle(1).toFixed(3)}°保护线</span><span>缺口断线 · 记录对照不等于实机验收</span></div>
  </section>;
}
/** 独立假设工况计算器，选择候选不会改变演示草稿或写入设备。 */
export function GainBudget({displayCoordinate='output'}:{displayCoordinate?:ControlCoordinate}) {
  const [selected,setSelected]=useState(0);
  const [assumptions,setAssumptions]=useState({errorDeg:outputDegreesToDisplayDegrees(1,displayCoordinate),speedError:displayCoordinate==='flange'?outputRadiansToDisplayDegrees(1,'flange'):1,ff:1});
  const candidate=GAIN_CANDIDATES[selected];
  const valid=Object.values(assumptions).every(value=>Number.isFinite(value)&&Math.abs(value)<=1000);
  const budget=valid?nominalBudget(candidate.kp,candidate.kd,assumptions.errorDeg*(displayCoordinate==='flange'?OUTPUT_PER_FLANGE:1),displayCoordinate==='flange'?displayDegreesToOutputRadians(assumptions.speedError,'flange'):assumptions.speedError,assumptions.ff):null;
  return <section className="panel gain-budget"><PanelHeader title="先看三项叠加，再比较增益" eyebrow="OFFLINE GAIN BUDGET"><Badge tone="amber"><FlaskConical size={11}/>纯离线比较</Badge></PanelHeader>
    <div className="gain-budget-body"><p className="gain-intro">Kp决定偏差后的拉回力度，Kd影响运动阻尼；前馈提前承担已知负载。反馈力矩随实际需要变化，不追求恒定额定值。</p><div className="gain-presets" aria-label="离线增益候选">{GAIN_CANDIDATES.map((entry,index)=><button key={entry.label} className={index===selected?'selected':''} aria-pressed={index===selected} onClick={()=>setSelected(index)}><strong>{entry.label}</strong><span>{entry.released?'已发布基线':'离线候选 · 未部署'}</span></button>)}</div>
      <div className="budget-assumptions"><span>假设工况</span>{([{key:'errorDeg',label:'位置误差',unit:displayCoordinate==='flange'?'° 法兰端':'° 输出轴'},{key:'speedError',label:'速度误差',unit:displayCoordinate==='flange'?'°/s 法兰端':'rad/s 输出轴'},{key:'ff',label:'同向前馈',unit:'N·m'}] as const).map(field=><label key={field.key}>{field.label}<div><input aria-label={'预算'+field.label} type="number" step=".1" value={Number.isFinite(assumptions[field.key])?assumptions[field.key]:''} onChange={event=>setAssumptions(old=>({...old,[field.key]:event.target.value===''?NaN:Number(event.target.value)}))}/><small>{field.unit}</small></div></label>)}</div>
      {budget?<div className="budget-equation"><div><span>位置项 |Kp·e|</span><strong>{budget.position.toFixed(3)}</strong></div><b>+</b><div><span>阻尼项 |Kd·Δv|</span><strong>{budget.damping.toFixed(3)}</strong></div><b>+</b><div><span>前馈幅值</span><strong>{budget.feedforward.toFixed(3)}</strong></div><ArrowRight size={17}/><div className={'budget-total '+(budget.remaining<=0?'exhausted':'')}><span>对{STATIC_RELEASE.torqueTripNm} N·m的名义预算差</span><strong>{budget.remaining.toFixed(3)}<small> N·m</small></strong><em>{budget.remaining<=0?'名义预算已用尽':'仍须覆盖动态与不确定性'}</em></div></div>:<div className="validation-error" role="alert">预算输入须为有限数，幅值不超过1000；此范围仅为计算器输入上限。</div>}
      <Note><ShieldCheck size={14}/>连续公式按三项同向幅值相加，不含CAN编码量化、效率、动态和停止后的运动。3.5 N·m是反馈越限退出门槛，不能当作硬限流。此处不改变上方草稿、不下发参数。</Note>
      <details className="gain-semantics"><summary>等效参数与实际发包值有什么区别？</summary><p>这里Kp单位为N·m/rad，Kd为N·m·s/rad，均按输出轴解释。发布基线Kp80在现有位置兼容口径下，原生发包Kp约80 / 19.2032 ≈4.166，之后还要编码量化；不能直接把80写进CAN增益字段。</p></details>
    </div>
  </section>;
}

