/** @file 工作台总览：状态与任务入口并列，不从示意图推断硬件状态。 */
import { Activity, ArrowRight, Clock3, ShieldCheck, Thermometer, Zap, SlidersHorizontal, ArrowUpRight, LockKeyhole } from 'lucide-react';
import { ArmDiagram } from '../components/ArmDiagram';
import { Badge, PanelHeader, TextLink, Value } from '../components/Common';
import { Chart, liveChartEndTime } from '../components/Chart';
import { ReleaseBaseline } from '../components/ReleaseBaseline';
import { COORDINATE_LABEL, feedbackAge } from '../domain';
import { captureTelemetry, projectTelemetry } from '../feedbackStatus';
import type { Studio } from '../useStudio';
import type { PageId } from '../types';
const MOTOR_STATE_LABEL: Record<string,string> = { unknown:'未知', absent:'无有效反馈', disabled:'驱动失能', moving:'运动反馈', holding:'保持反馈', fault:'故障' };
/** 显示七轴只读状态，真实缺失轴不使用演示值补齐。 */
export function MotorTable({ studio, compact = false }: { studio: Studio; compact?: boolean }) {
  return <div className="table-scroll"><table className="motor-table"><thead><tr><th>关节 / 电机</th><th>状态</th><th>反馈位置 <small>°</small></th>{!compact && <th>转矩 <small>N·m</small></th>}<th>反馈时效</th><th>坐标</th></tr></thead><tbody>{Array.from({length:7},(_,index) => {
    const id = index + 1;
    const rawMotor = studio.gateway.motors.find((item) => item.id === id);
    const demo = studio.mode === 'demo';
    const replay = studio.mode === 'replay';
    const selected = id === studio.settings.motorId;
    const sample = studio.samples.at(-1);
    const timestamp = Date.now();
    const current = rawMotor ? captureTelemetry(rawMotor,studio.gateway.connected,timestamp,0) : undefined;
    const engineering = current && id===7 ? projectTelemetry(current,'output','native',7) : undefined;
    const position = demo ? (selected ? sample?.positionDeg : [0, 12.5, -8.2, 0, 6.3, 0][index]) : replay ? (id === studio.recording.motorId ? sample?.positionDeg : null) : engineering?.positionDeg;
    const age = rawMotor?.receivedAt ? feedbackAge(rawMotor.ageMs, Date.parse(rawMotor.receivedAt), timestamp) : null;
    const positionSource = engineering?.positionSource === 'register50' ? '0x50位置' : engineering?.positionSource === 'standard' ? '驱动位置' : '位置缺失或过期';
    const displayedAge = engineering?.positionSource === 'register50' ? current?.register50AgeMs ?? null : age;
    const state = !studio.gateway.connected ? '未连接' : age !== null && age>1000 ? '反馈过期' : rawMotor ? MOTOR_STATE_LABEL[rawMotor.state]??rawMotor.state : '未知';
    return <tr key={id} className={selected ? 'active-row' : ''} onClick={() => !studio.running && studio.setSettings((old) => ({...old,motorId:id}))}><td><button className="axis-button" aria-label={'选择J'+id} disabled={studio.running}><span className="joint-token">{String(id).padStart(2,'0')}</span><strong>J{id}</strong><span className="micro">ID {id}</span>{selected && <span className="axis-selected">当前轴</span>}</button></td><td><Badge tone={demo ? 'green' : 'neutral'}>{demo ? '合成反馈' : replay ? '记录回放' : state}</Badge></td><td><Value value={position}/>{!demo&&!replay&&<div className="micro">{id===7?positionSource:'标定未确认'}</div>}</td>{!compact && <td><Value value={demo ? selected ? sample?.torqueNm : 0 : replay ? id === studio.recording.motorId ? sample?.torqueNm : null : current?.torqueNm} digits={3}/></td>}<td><span className={demo ? 'micro' : displayedAge === null || displayedAge > 1000 ? 'text-amber' : 'micro'}>{demo ? '离线样本' : replay ? '历史数据' : displayedAge === null ? '未知' : Math.round(displayedAge) + ' ms' + (displayedAge > 1000 ? ' · 过期' : '')}</span></td><td className="micro">{demo ? (id===studio.recording.motorId?COORDINATE_LABEL[studio.recording.coordinate]:'输出轴')+'（演示）' : replay ? COORDINATE_LABEL[studio.recording.coordinate] : id===7?'电机输出轴':'标定未确认'}</td></tr>;
  })}</tbody></table></div>;
}
/** 聚合当前任务、实时/记录曲线与选中轴摘要。 */
export function Overview({ studio, navigate }: { studio: Studio; navigate: (page: PageId) => void }) {
  const sample = studio.samples.at(-1);
  const motor = studio.gateway.motors.find((entry) => entry.id === studio.settings.motorId);
  const live = studio.mode === 'live';
  const current = live && motor ? captureTelemetry(motor,studio.gateway.connected,Date.now(),0) : undefined;
  const torque = live ? current?.torqueNm : sample?.torqueNm;
  const temp = live ? current?.mosC : sample?.mosC;
  const preview = live ? studio.samples.map(entry=>projectTelemetry(entry,'output','native',studio.settings.motorId)) : studio.samples;
  const previewCoordinate = live ? studio.settings.motorId===7?'电机输出轴':'标定未确认 · 位置/速度未知' : COORDINATE_LABEL[studio.recording.coordinate];
  return <>
    <div className="overview-intro"><div><span className="eyebrow">AETHOR ROBOTICS / DEBUG STUDIO</span><p>每一次调整，都有清晰的反馈。</p></div><span className="intro-index">01 — 09</span></div>
    <ReleaseBaseline compact onOpen={()=>navigate('static')}/>
    <div className="metric-grid">
      <div className="metric"><span className="metric-icon"><Activity size={19}/></span><div><span>数据来源</span><strong className="metric-text">{studio.mode === 'demo' ? '离线演示' : live ? '真实只读' : '记录回放'}</strong><small>{live ? studio.gateway.status : '未向电机发送命令'}</small></div></div>
      <div className="metric"><span className="metric-icon"><SlidersHorizontal size={19}/></span><div><span>当前选择</span><strong>J{studio.settings.motorId}<em> / ID {studio.settings.motorId}</em></strong><small>{live ? '实际模式未由接口提供' : '独立单轴调试'}</small></div></div>
      <div className="metric"><span className="metric-icon amber"><Zap size={19}/></span><div><span>反馈转矩</span><strong><Value value={torque} digits={3} unit="N·m"/></strong><small>{live ? '固件反馈口径' : '当前显示样本 · 非承载评级'}</small></div></div>
      <div className="metric"><span className="metric-icon blue"><Thermometer size={19}/></span><div><span>MOS 温度</span><strong><Value value={temp} digits={1} unit="℃"/></strong><small>{live ? '保留反馈时效信息' : '合成记录值'}</small></div></div>
    </div>
    <div className="overview-main">
      <section className="panel"><PanelHeader title="机械臂状态" eyebrow="ARM OVERVIEW"><Badge tone={live && studio.gateway.connected ? 'green' : 'neutral'}>{live ? studio.gateway.connected ? '串口已连接' : '未连接设备' : '离线环境'}</Badge></PanelHeader><ArmDiagram selected={studio.settings.motorId}/><div className="arm-foot"><div><span className="micro">当前坐标</span><strong>{previewCoordinate}</strong></div><TextLink onClick={() => navigate('seven')}>查看七轴</TextLink></div></section>
      <section className="panel task-panel"><PanelHeader title="选择调试任务" eyebrow="QUICK START"/><div className="task-list">
        {[{id:'static',title:'单轴静载',sub:'捕获位置 · 前馈 · 限时保持',icon:<LockKeyhole size={20}/>},{id:'pos',title:'POS 带载测试',sub:'相对移动 · 到位 · 保持',icon:<ArrowUpRight size={20}/>},{id:'manual',title:'手动调试',sub:'POS / MIT 参数与流程',icon:<SlidersHorizontal size={20}/>}].map((item,index)=><button className="task-entry" key={item.id} onClick={() => navigate(item.id as PageId)}><span className="task-number">0{index+1}</span><span className="task-entry-icon">{item.icon}</span><span><strong>{item.title}</strong><small>{item.sub}</small></span><ArrowRight size={17}/></button>)}
      </div><div className="task-bottom"><ShieldCheck size={21}/><div><strong>先检查，再执行</strong><p>控制权、模式、反馈与参数在开始前统一审核。</p></div></div></section>
    </div>
    <section className="panel chart-panel"><PanelHeader title={live ? '反馈示波' : '记录预览'} eyebrow="SIGNAL MONITOR"><TextLink onClick={()=>navigate('scope')}>打开示波器</TextLink></PanelHeader><Chart windowEndMs={liveChartEndTime(studio.duration,studio.now,studio.gateway.motors.find(motor=>motor.id===studio.settings.motorId)?.receivedAt)} windowMs={live?60000:undefined} gapMs={live?500:250} samples={preview} events={studio.recording.events} coordinateLabel={previewCoordinate}/></section>
    <section className="panel"><PanelHeader title="七轴状态" eyebrow="JOINT STATUS"><span className="micro"><Clock3 size={13}/> {live ? '轮询只读快照' : '当前为合成 / 历史数据'}</span></PanelHeader><MotorTable studio={studio}/></section>
  </>;
}
