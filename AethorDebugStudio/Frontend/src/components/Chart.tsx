/** @file 同单位SVG曲线；未知值和采样缺口不会被连成连续数据。 */
import { useId, useState } from 'react';
import { sampleSegments } from '../domain';
import type { Sample, Signal, StudioEvent } from '../types';
const GROUPS: { title: string; unit: string; signals: { key: Signal; label: string; color: string }[] }[] = [
  { title: '位置', unit: '°', signals: [{ key: 'targetDeg', label: '目标位置', color: '#98a7b3' }, { key: 'positionDeg', label: '实际位置', color: '#008e83' }] },
  { title: '速度', unit: '°/s', signals: [{ key: 'speedDegS', label: '反馈速度', color: '#397ac4' }] },
  { title: '转矩', unit: 'N·m', signals: [{ key: 'torqueNm', label: '反馈转矩', color: '#b57b2c' }] },
  { title: '温度', unit: '℃', signals: [{ key: 'mosC', label: 'MOS温度', color: '#a06c4d' }, { key: 'rotorC', label: '转子温度', color: '#397ac4' }] },
];
/** 用最后样本的会话时间与接收时刻衔接实时时钟，断流也继续滚动。 */
export function liveChartEndTime(latestSampleTimeMs:number,now:number,receivedAt:string|null|undefined):number {
  const received=receivedAt?Date.parse(receivedAt):NaN;
  return latestSampleTimeMs+(Number.isFinite(received)&&Number.isFinite(now)?Math.max(0,now-received):0);
}
/** 绘制位置/速度/力矩/温度之一，避免不同单位共用纵轴。 */
export function Chart({ samples: recordedSamples, events: recordedEvents = [], large = false, coordinateLabel = '电机输出轴', gapMs = 250, windowMs, windowEndMs, onGroupChange }: { samples: Sample[]; events?: StudioEvent[]; large?: boolean; coordinateLabel?: string; gapMs?: number; windowMs?: number; windowEndMs?: number; onGroupChange?: (unit: string) => void }) {
  const [groupIndex, setGroupIndex] = useState(0);
  const [hidden, setHidden] = useState<string[]>([]);
  const [hoveredSample, setHover] = useState<Sample | null>(null);
  const rolling=typeof windowMs==='number'&&Number.isFinite(windowMs)&&windowMs>0;
  const latest=Math.max(recordedSamples.at(-1)?.timeMs??0,rolling&&typeof windowEndMs==='number'&&Number.isFinite(windowEndMs)?windowEndMs:0);
  const start=rolling?Math.max(0,latest-windowMs):recordedSamples[0]?.timeMs??0;
  const end=Math.max(start+1000,latest);
  const samples=rolling?recordedSamples.filter(sample=>sample.timeMs>=start&&sample.timeMs<=end):recordedSamples;
  const events=rolling?recordedEvents.filter(event=>event.timeMs>=start&&event.timeMs<=end):recordedEvents;
  const hover=hoveredSample&&samples.includes(hoveredSample)?hoveredSample:null;
  // 工程投影在页面一次性处理整段历史；0x51只能在原始诊断中查看。
  const group = groupIndex === 1 && samples.some(sample=>sample.speedSource === 'estimated') ?
    {...GROUPS[1], signals:[{key:'speedDegS' as Signal,label:'速度 · 驱动反馈 / 板端估算',color:'#397ac4'}]} : GROUPS[groupIndex];
  const id = useId().replaceAll(':', '');
  const signals = group.signals.filter((signal) => !hidden.includes(signal.key));
  const values = samples.flatMap((sample) => signals.map((signal) => sample[signal.key]).filter((value): value is number => value !== null && Number.isFinite(value)));
  const minimum = values.length ? Math.min(...values) : 0;
  const maximum = values.length ? Math.max(...values) : 1;
  const margin = Math.max((maximum - minimum) * 0.25, groupIndex === 0 ? 0.05 : 0.02);
  const bottom = minimum - margin;
  const top = maximum + margin;
  /** 投影到固定viewBox中的安全数据区域。 */
  function x(time: number) { return 64 + (time - start) / (end - start) * 846; }
  /** 按相同单位的动态范围投影纵坐标。 */
  function y(value: number) { return 20 + (top - value) / (top - bottom) * 188; }
  return <div className={'chart ' + (large ? 'chart-large' : '')}>
    <div className="chart-toolbar"><div className="segmented" aria-label="曲线类型">{GROUPS.map((item, index) => <button key={item.title} className={groupIndex === index ? 'selected' : ''} onClick={() => { setGroupIndex(index); setHover(null); onGroupChange?.(item.unit); }}>{item.title}</button>)}</div><span className="micro">{coordinateLabel} · {group.unit}{rolling?` · 最近${windowMs/1000}秒`:''}</span></div>
    {!values.length ? <div className="chart-empty"><span className="empty-wave">∿</span><strong>{signals.length ? '等待有效采样' : '当前信号已全部隐藏'}</strong><span>{signals.length ? '未知数据不会以零值绘制' : '点击下方图例恢复显示'}</span></div> : <svg viewBox="0 0 940 248" role="img" aria-label={group.title + '反馈曲线'} onMouseLeave={() => setHover(null)} onMouseMove={(event) => {
      const fraction = (event.clientX - event.currentTarget.getBoundingClientRect().left) / event.currentTarget.getBoundingClientRect().width;
      const time = start + Math.max(0, Math.min(1, (fraction * 940 - 64) / 846)) * (end - start);
      setHover(samples.reduce((nearest, sample) => Math.abs(sample.timeMs - time) < Math.abs(nearest.timeMs - time) ? sample : nearest, samples[0]));
    }}>
      <defs><clipPath id={id}><rect x="64" y="20" width="846" height="188"/></clipPath></defs>
      {[0, 1, 2, 3, 4].map((tick) => <g key={tick}><line x1="64" x2="910" y1={20 + tick * 47} y2={20 + tick * 47} stroke="#e9eef0" strokeDasharray="3 4"/><text x="54" y={24 + tick * 47} textAnchor="end" className="axis-text">{(top - tick * (top - bottom) / 4).toFixed(2)}</text><text x={64 + tick * 211.5} y="232" textAnchor="middle" className="axis-text">{((start + tick * (end - start) / 4) / 1000).toFixed(1)}s</text></g>)}
      <g clipPath={'url(#' + id + ')'}>{events.filter((event) => event.level === 'error' || event.gap).map((event, index) => <line key={index} x1={x(event.timeMs)} x2={x(event.timeMs)} y1="20" y2="208" stroke="#c05a4a" strokeDasharray="4 4"/>)}
        {signals.map((signal) => sampleSegments(samples, signal.key, gapMs,events.filter((event)=>event.gap).map((event)=>event.timeMs)).map((segment, index) => segment.length===1?<circle key={signal.key+index} cx={x(segment[0].timeMs)} cy={y(segment[0][signal.key]!)} r="2.5" fill={signal.color}/>:<polyline key={signal.key + index} fill="none" stroke={signal.color} strokeWidth={signal.key === 'targetDeg' ? 1.5 : 2.4} strokeDasharray={signal.key === 'targetDeg' ? '6 4' : undefined} strokeLinejoin="round" points={segment.map((sample) => x(sample.timeMs) + ',' + y(sample[signal.key]!)).join(' ')}/>))}
        {hover && <line x1={x(hover.timeMs)} x2={x(hover.timeMs)} y1="20" y2="208" stroke="#566c77" strokeDasharray="2 3"/>}
      </g>
    </svg>}
    <div className="chart-legend">{group.signals.map((signal) => <button key={signal.key} className={hidden.includes(signal.key) ? 'muted' : ''} onClick={() => setHidden((old) => old.includes(signal.key) ? old.filter((key) => key !== signal.key) : [...old, signal.key])}><span style={{ backgroundColor: signal.color }} />{signal.label}{hover && ' ' + (hover[signal.key]?.toFixed(3) ?? '未知')}</button>)}<span className="micro">{hover ? (hover.timeMs / 1000).toFixed(3) + ' s' : '缺口断线 · 悬停查看数值'}</span></div>
  </div>;
}
