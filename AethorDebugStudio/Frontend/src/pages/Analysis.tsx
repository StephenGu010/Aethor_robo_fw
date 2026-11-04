/** @file 示波与记录页面；支持有界JSON导入、CSV/JSON导出和历史回放。 */
import { useRef, useState } from 'react';
import { Download, FileJson2, FolderOpen, Pause, Play, Search, Upload, FileClock, AlertTriangle } from 'lucide-react';
import { Chart, liveChartEndTime } from '../components/Chart';
import { FeedbackHealth } from '../components/DeviceDebug';
import { captureTelemetry, projectTelemetry } from '../feedbackStatus';
import { StaticObservation } from '../components/StaticInsights';
import { Badge, Note, PanelHeader, Value } from '../components/Common';
import { COORDINATE_LABEL, MAX_RECORD_BYTES, parseRecording, toCsv } from '../domain';
import type { Studio } from '../useStudio';
import type { Coordinate, Recording } from '../types';
/** 创建本地下载，不向外部服务上传记录。 */
export function download(content: string, name: string, mime: string) {
  const url = URL.createObjectURL(new Blob([content], { type: mime }));
  const anchor = document.createElement('a'); anchor.href = url; anchor.download = name; anchor.click();
  window.setTimeout(() => URL.revokeObjectURL(url), 1000);
}
/** 查看同单位曲线、首故障与回放时间，不拼接数据缺口。 */
export function Scope({ studio }: { studio: Studio }) {
  const [selectedCoordinate, setSelectedCoordinate] = useState<Coordinate>('output');
  const motorId = studio.mode === 'live' ? studio.settings.motorId : studio.recording.motorId;
  const coordinate = motorId === 7 ? selectedCoordinate : studio.recording.coordinate;
  const samples = studio.samples.map(sample=>projectTelemetry(sample,coordinate,studio.recording.coordinate,motorId));
  const liveMotor = studio.gateway.motors.find(motor => motor.id === studio.settings.motorId);
  // 当前读值实时重新检查年龄，历史曲线只代表采样时刻；两者不延长标准反馈资格。
  const currentRaw = studio.mode === 'live' ? (liveMotor ? captureTelemetry(liveMotor,studio.gateway.connected,Date.now(),0) : undefined) : studio.samples.at(-1);
  const sample = currentRaw ? projectTelemetry(currentRaw,coordinate,studio.recording.coordinate,motorId) : undefined;
  const positionLabel = sample?.positionSource === 'register50' ? '0x50位置换算' : sample?.positionSource === 'standard' ? '驱动位置反馈' : '位置缺失或过期';
  const speedLabel = sample?.speedSource === 'estimated' ? '板端差分估算速度' : sample?.speedSource === 'standard' ? '驱动速度反馈' : '速度缺失或过期';
  const exportRecording: Recording = {...studio.recording,coordinate,samples:studio.recording.samples.map(entry=>projectTelemetry(entry,coordinate,studio.recording.coordinate,motorId))};
  const fault = studio.recording.events.find((event)=>event.level==='error');
  return <>
    {studio.mode==='live'&&<FeedbackHealth studio={studio}/>}
    <div className="scope-strip"><Badge tone={studio.mode==='demo'?'blue':studio.mode==='live'?'green':'amber'}>{studio.mode==='demo'?'合成记录':studio.mode==='live'?'真实设备遥测':'历史记录回放'}</Badge><span>ID {motorId}</span><label>显示坐标 <select aria-label="示波器显示坐标" value={coordinate} disabled={motorId!==7} onChange={event=>setSelectedCoordinate(event.target.value as Coordinate)}><option value="output">电机输出轴</option><option value="flange">法兰</option>{motorId!==7&&<option value="native">固件原始反馈</option>}</select></label><span>采集记录 {samples.length} · 有效位置 {samples.filter(entry=>typeof entry.positionDeg==='number'&&Number.isFinite(entry.positionDeg)).length}</span><button className="button secondary small" onClick={()=>download(toCsv(exportRecording),'aethor-signals.csv','text/csv;charset=utf-8')} disabled={!studio.recording.samples.length}><Download size={14}/>导出 CSV</button></div>
    <section className="panel"><PanelHeader title="信号观察" eyebrow="OSCILLOSCOPE"><span className="micro">{positionLabel} · {speedLabel} · 来源/启动/代次变化断线</span></PanelHeader><Chart key={coordinate} large windowEndMs={liveChartEndTime(studio.duration,studio.now,liveMotor?.receivedAt)} windowMs={studio.mode==='live'?60000:undefined} gapMs={studio.mode==='live'?(motorId===7?500:2500):250} samples={samples} events={studio.recording.events} coordinateLabel={COORDINATE_LABEL[coordinate]}/>
    {studio.mode==='replay'&&<div className="replay-controls"><button className="icon-button" aria-label={studio.playing?'暂停回放':'播放回放'} onClick={()=>{if(studio.visibleTime>=studio.duration)studio.setReplayTime(0);studio.setPlaying(!studio.playing);}}>{studio.playing?<Pause size={17}/>:<Play size={17}/>}</button><input aria-label="回放进度" type="range" min="0" max={Math.max(1,studio.duration)} value={studio.visibleTime} onChange={(event)=>{studio.setPlaying(false);studio.setReplayTime(Number(event.target.value));}}/><span className="number">{(studio.visibleTime/1000).toFixed(2)} / {(studio.duration/1000).toFixed(2)} s</span></div>}</section>
    {studio.recording.staticContext&&studio.mode!=='live'&&<StaticObservation recording={studio.recording} samples={studio.samples}/>}
    <div className="analysis-grid"><section className="panel"><PanelHeader title="当前采样" eyebrow="CURSOR READOUT"/><div className="readout-grid">{[{label:positionLabel,value:sample?.positionDeg,unit:'°'},{label:speedLabel,value:sample?.speedDegS,unit:'°/s'},{label:'反馈转矩',value:sample?.torqueNm,unit:'N·m'},{label:'MOS温度',value:sample?.mosC,unit:'℃'}].map((item)=><div key={item.label}><span>{item.label}</span><Value value={item.value} digits={3} unit={item.unit}/></div>)}</div><Note>估算速度仅用于观察；转矩、温度及失能确认需要标准反馈。未知量保持缺失，寄存器不授予运动资格。</Note></section><section className="panel"><PanelHeader title="首个异常" eyebrow="FIRST FAULT"/><div className="fault-summary">{fault?<><AlertTriangle size={24}/><div><strong>{fault.title}</strong><p>{fault.detail}</p><span>{fault.timeMs} ms</span></div></>:<><span className="empty-check">✓</span><div><strong>当前记录未包含首故障</strong><p>仅说明记录内容，不代表整段测试无异常。</p></div></>}</div></section></div>
    <section className="panel"><PanelHeader title="原始寄存器诊断" eyebrow="RAW REGISTERS"/><div className="readout-grid"><div><span>0x50 p_m · 契约 {currentRaw?.pmContract===1?'1 已确认':'未确认'}</span><Value value={currentRaw?.register50Raw} digits={6} unit={currentRaw?.pmContract===1?'转子 rad':'原始值'}/><p className="micro">样本年龄 {currentRaw?.register50AgeMs??'未知'} ms · 启动 {currentRaw?.pmBoot??'未知'} · 代次 {currentRaw?.pmEpoch??'未知'}</p></div><div><span>0x51 xout · 未获测量资格</span><Value value={currentRaw?.register51Raw} digits={6} unit="原始值"/><p className="micro">仅诊断，恒零不代表输出轴零位</p></div></div></section>
    <section className="panel"><PanelHeader title="阶段事件" eyebrow="EVENT TIMELINE"/><Events studio={studio}/></section>
  </>;
}
/** 渲染采集事件或真实串口行，保留原始字节与缺口标记。 */
export function Events({ studio }: { studio: Studio }) {
  if(studio.mode==='live')return <div className="raw-lines">{studio.gateway.lines.length?studio.gateway.lines.map((line,index)=><code key={index}>{line}</code>):<div className="empty-state"><FileClock size={25}/><strong>等待串口只读响应</strong><p>设备连接后显示本次会话原始响应。</p></div>}</div>;
  const events=studio.recording.events.filter((event)=>event.timeMs<=studio.visibleTime);
  return <div className="event-list">{events.length?events.map((event,index)=><div className={'event-row '+event.level} key={index}><span className="event-dot"/><span className="event-time">{(event.timeMs/1000).toFixed(3)}s</span><div><strong>{event.title}{event.gap&&<Badge tone="amber">记录缺口</Badge>}</strong><p>{event.detail}</p>{event.rawHex&&<code>CAN {event.rawHex}</code>}</div><Badge tone={event.level==='error'?'red':event.level==='warning'?'amber':'neutral'}>{event.level==='error'?'异常':event.level==='warning'?'注意':'事件'}</Badge></div>):<div className="empty-state"><FileClock size={25}/><strong>当前时间点没有事件</strong></div>}</div>;
}
/** 导入记录、查看本次会话历史并保留来源标记。 */
export function Records({ studio }: { studio: Studio }) {
  const input=useRef<HTMLInputElement>(null);
  const [query,setQuery]=useState('');
  const all=[studio.recording,...studio.history.filter((entry)=>entry!==studio.recording)];
  const recordings=all.filter((entry)=>entry.name.includes(query));
  /** 限制文件体积，再交给严格格式解析器。 */
  async function importFile(file?:File){
    if(!file)return;
    try{if(file.size>MAX_RECORD_BYTES)throw new Error('记录不能超过2MB');studio.loadRecording(parseRecording(await file.text()));}
    catch(error){studio.setNotice(error instanceof Error?error.message:'记录无法读取');}
    finally{if(input.current)input.current.value='';}
  }
  /** 输出原始结构而不是图表截图，保留来源与未知值。 */
  function exportJson(recording:Recording){download(JSON.stringify(recording,null,2),'aethor-recording.json','application/json');}
  return <>
    <div className="records-actions"><div className="search-box"><Search size={17}/><input aria-label="搜索记录" placeholder="搜索本次会话记录…" value={query} onChange={(event)=>setQuery(event.target.value)}/></div><input ref={input} className="visually-hidden" aria-label="导入记录文件" type="file" accept=".json,application/json" disabled={studio.running} onChange={(event)=>void importFile(event.target.files?.[0])}/><button className="button primary" disabled={studio.running} onClick={()=>input.current?.click()}><Upload size={16}/>导入 JSON 记录</button></div>
    <Note>记录保存在当前浏览器会话内，请导出需要保留的文件。支持工作台记录及现有冻结启动追踪格式，文件上限2MB；不会上传到云端。</Note>
    <div className="records-layout"><section className="panel"><PanelHeader title="会话记录" eyebrow="SESSION RECORDINGS"><span className="micro">{recordings.length} 条</span></PanelHeader><div className="record-list">{recordings.map((entry,index)=><div className={'record-entry '+(index===0?'active':'')} key={entry.createdAt+index}><div className="record-icon"><FileJson2 size={23}/></div><div className="record-copy"><strong>{entry.name}</strong><p>ID {entry.motorId} · {entry.samples.length}样本 · {COORDINATE_LABEL[entry.coordinate]}</p><small>{new Date(entry.createdAt).toLocaleString('zh-CN',{hour12:false})}</small><Badge tone={entry.source==='synthetic'?'blue':'neutral'}>{entry.source==='synthetic'?'合成数据':'设备记录 · 未评级'}</Badge></div><div className="record-actions"><button className="button secondary small" disabled={studio.running} onClick={()=>studio.loadRecording(entry)}><Play size={14}/>回放</button><button className="icon-button" aria-label={'导出JSON '+entry.name} onClick={()=>exportJson(entry)}><Download size={16}/></button></div></div>)}{!recordings.length&&<div className="empty-state"><FolderOpen size={26}/><strong>没有匹配记录</strong><p>换一个名称试试。</p></div>}</div></section>
    <aside className="panel format-guide"><span className="eyebrow">SUPPORTED FORMATS</span><h2>已有记录，也能直接查看。</h2><div><strong>工作台记录</strong><code>aethor-studio-recording/v1</code><p>样本、阶段事件、数据来源与坐标信息。</p></div><div><strong>板端启动追踪</strong><code>static-startup-capture-v1</code><p>导入冻结追踪，保留序号缺口与原始CAN。仅将accepted_rx作为已采纳反馈。</p></div><p className="micro">CAN发送顺序不能单独证明驱动内部已接受目标。导入操作不增加原始记录的证据等级。</p></aside></div>
    <section className="panel"><PanelHeader title="当前记录详情" eyebrow="RECORD DETAIL"><button className="button secondary small" disabled={studio.mode==='live'} onClick={()=>download(toCsv(studio.recording),'aethor-signals.csv','text/csv;charset=utf-8')}><Download size={14}/>导出 CSV</button></PanelHeader>{studio.mode==='live'?<Note>当前是真实只读模式。原始查询响应可在“实时示波”的事件区查看；本版不把演示记录冒充真实会话导出。</Note>:<Events studio={studio}/>}</section>
  </>;
}
