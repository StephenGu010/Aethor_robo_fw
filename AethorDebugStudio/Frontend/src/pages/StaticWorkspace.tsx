/** @file 按F最新发布重做的单轴静载工作区；只做草稿、观察与离线演示。 */
import { ArrowRight, Check, Circle, LockKeyhole, Play, RotateCcw, Square, Timer, Waves } from 'lucide-react';
import { Badge, Note, PanelHeader } from '../components/Common';
import { ReleaseBaseline } from '../components/ReleaseBaseline';
import { GainBudget, StaticObservation, StaticWindows } from '../components/StaticInsights';
import { DEFAULT_SETTINGS } from '../catalog';
import { canExecute, validateSettings } from '../domain';
import { STATIC_RELEASE } from '../staticPolicy';
import type { Settings } from '../types';
import type { Studio } from '../useStudio';
import '../static-workspace.css';
import {outputDegreesToDisplayDegrees,type ControlCoordinate} from '../axisUnits';

const INPUTS:{name:keyof Settings;label:string;unit:string;help:string;minimum:number;maximum:number;step:number}[]=[
  {name:'durationS',label:'保持时间',unit:'s',help:'正式保持计时，接管阶段单独显示',minimum:1,maximum:60,step:1},
  {name:'torqueFfNm',label:'前馈力矩',unit:'N·m 输出轴',help:'有符号的主动补偿量；不追求恒定额定力矩',minimum:-3.5,maximum:3.5,step:.01},
  {name:'kp',label:'位置刚度 Kp',unit:'N·m/rad',help:'输出轴等效量，偏差后的拉回力度',minimum:0,maximum:500,step:1},
  {name:'kd',label:'速度阻尼 Kd',unit:'N·m·s/rad',help:'输出轴等效量，影响运动阻尼与振荡',minimum:0,maximum:5,step:.01},
];
/** 显示本地草稿字段；保持现有输入ID及审核冻结语义。 */
function DraftField({field,studio}:{field:typeof INPUTS[number];studio:Studio}){
  const value=studio.settings[field.name];
  return <div className="static-field"><label htmlFor={field.name}>{field.label}<span>{field.unit}</span></label><input id={field.name} aria-describedby={field.name+'-description'} type="number" min={field.minimum} max={field.maximum} step={field.step} value={Number.isFinite(value)?value:''} disabled={studio.running||studio.workflow.stage==='review'} onChange={event=>studio.setSettings(old=>({...old,[field.name]:event.target.value===''?NaN:Number(event.target.value)}))}/><p id={field.name+'-description'}>{field.help}</p></div>;
}
/** 将发布基线、编辑、窗口观察、预算和结果放在独立但相邻的区域。 */
export function StaticWorkspace({studio}:{studio:Studio}){
  const displayCoordinate:ControlCoordinate=studio.settings.motorId===7?'flange':'output';
  const axisLabel=displayCoordinate==='flange'?'法兰端':'输出轴';
  /** 旧发布策略和历史记录只作显示换算，不重写其判据。 */
  const angle=(value:number)=>outputDegreesToDisplayDegrees(value,displayCoordinate).toFixed(3);
  const errors=validateSettings('static',studio.settings);
  const gate=canExecute(studio.mode,studio.running,errors);
  const editingLocked=studio.running||studio.workflow.stage==='review';
  const baselineDraft=studio.settings.motorId===7&&studio.settings.kp===STATIC_RELEASE.kpOutput&&studio.settings.kd===STATIC_RELEASE.kdOutput;
  const frozen=studio.workflow.frozen;
  const firstFault=studio.recording.events.find(event=>event.level==='error');
  return <div className="static-workspace">
    <ReleaseBaseline compact displayCoordinate={displayCoordinate}/>
    <div className="static-workspace-intro"><div><span className="eyebrow">STATIC HOLD / DISTURBANCE OBSERVATION</span><h2>保持角度，看清每一次偏差。</h2><p>反馈力矩按实际负载变化。精度超差与保护退出，分别记录。</p></div><Badge tone={studio.mode==='demo'?'blue':'neutral'}>{studio.mode==='demo'?'本地演示 · 不预测真实抗扰性能':studio.mode==='live'?'真实只读 · 不下发草稿':'历史回放 · 不重写退出原因'}</Badge></div>
    <div className="static-main-grid">
      <section className="panel static-draft"><PanelHeader title="本次保持草稿" eyebrow="LOCAL DRAFT"><Badge tone={baselineDraft?'green':'amber'}>{baselineDraft?'增益与发布基线一致':'离线候选 · 未部署'}</Badge></PanelHeader>
        <div className="static-draft-body"><div className="static-target-row"><div><label htmlFor="motor-select">目标电机</label><select id="motor-select" value={studio.settings.motorId} disabled={editingLocked} onChange={event=>studio.setSettings(old=>({...old,motorId:Number(event.target.value)}))}>{Array.from({length:7},(_,index)=><option key={index} value={index+1}>J{index+1} · 电机 ID {index+1}</option>)}</select></div><div className="capture-position"><LockKeyhole size={18}/><div><strong>锁定当前位置</strong><p>目标速度为0，不自动定位到某角度</p></div></div></div>
        <div className="static-form-grid">{INPUTS.map(field=><DraftField field={field} studio={studio} key={field.name}/>)}</div>
        <details className="load-details"><summary>装载记录 <span>仅记录，不生成负载许可</span></summary><div className="static-form-grid">{[{name:'massG',label:'额外挂重',unit:'g',help:'不含臂杆、夹具和承托分担',minimum:0,maximum:10000,step:1},{name:'leverMm',label:'有效重力力臂',unit:'mm',help:'转轴到重力作用线的垂直距离，不是必然等于杆长',minimum:0,maximum:1000,step:1}].map(field=><DraftField key={field.name} field={field as typeof INPUTS[number]} studio={studio}/>)}</div></details>
        <div className="static-parameter-hint">增益按输出轴等效量解释；以上范围只用于离线草稿校验。当前设备的模式、增益、控制权没有实时读回时，不自动补齐。</div>
        {errors.length>0&&<div className="validation-error" role="alert">{errors.map(error=><div key={error}>{error}</div>)}</div>}
        <div className="form-actions"><button className="button secondary" disabled={editingLocked} onClick={()=>studio.setSettings({...DEFAULT_SETTINGS,motorId:studio.settings.motorId})}><RotateCcw size={14}/>恢复演示基线</button><button className="button primary" disabled={!gate.allowed||studio.workflow.stage==='review'} title={gate.reason} onClick={()=>studio.review('static')}>预检与审核<ArrowRight size={15}/></button></div>{gate.reason&&<p className="disabled-reason">{gate.reason}</p>}
        <div className="static-draft-foot"><span><Circle size={9}/>编辑草稿</span><ArrowRight size={12}/><span>冻结参数</span><ArrowRight size={12}/><span>只在浏览器演示</span></div>
      </div></section>
      <div className="static-monitor-column"><StaticWindows motorId={studio.settings.motorId} displayCoordinate={displayCoordinate}/><StaticObservation recording={studio.recording} samples={studio.samples} displayCoordinate={displayCoordinate}/></div>
    </div>
    {studio.running&&<section className="panel running-panel"><div><Badge tone="green">模拟执行中</Badge><h2>{studio.elapsed<(studio.recording.staticContext?.holdStartMs??1000)?'前馈接管演示':'限时保持演示'}</h2><p>{(studio.elapsed/1000).toFixed(1)} / {(studio.duration/1000).toFixed(1)} s · 未发送设备命令</p></div><progress max={studio.duration} value={studio.elapsed}/><button className="button danger" onClick={studio.stop}><Square size={15}/>立即停止演示</button></section>}
    {['completed','stopped'].includes(studio.workflow.stage)&&<section className="panel static-result"><span className="static-result-icon"><Check size={23}/></span><div><span className="eyebrow">PROCESS RESULT ≠ PRECISION RESULT</span><h2>{studio.workflow.stage==='completed'?'演示流程完成':'演示已停止'}</h2><p>流程终态、{angle(.5)}°{axisLabel}精度观察与保护对照分别查看；无硬件失能或抗扰资格证明。</p></div><Badge>仅合成记录</Badge></section>}
    <div className="static-review-grid"><GainBudget key={displayCoordinate} displayCoordinate={displayCoordinate}/><section className="panel static-exit-card"><PanelHeader title="先保留首个退出原因" eyebrow="FIRST EXIT / EVIDENCE"/><div className="static-exit-body"><span className="exit-symbol"><Waves size={26}/></span><h3>{firstFault?.title??'当前记录未包含首故障'}</h3><p>{firstFault?.detail??'这表示当前记录没有该事件，不代表整段测试都无异常。'}</p><div className="historical-exit"><span>历史案例 · 旧{angle(.5)}°{axisLabel}保护版本</span><strong>{angle(.513)}°<small>{axisLabel}偏差</small></strong><p>当时转矩幅值约0.681 N·m，退出原因为位置偏差，并非达到3.5 N·m。</p><span>不能按新{angle(1)}°{axisLabel}门槛改写旧记录的退出原因。</span></div><Note>驱动“已失能”不等于机械已停稳。发布记录、当前反馈与实机停止行为分别核对。</Note></div></section></div>
    {studio.workflow.stage==='review'&&frozen&&<div className="modal-backdrop"><section className="modal review-modal" role="dialog" aria-modal="true" aria-labelledby="static-review-title"><div className="modal-icon"><Timer size={25}/></div><span className="eyebrow">FROZEN DRAFT / OFFLINE ONLY</span><h2 id="static-review-title">核对本次演示</h2><p>以下草稿已冻结。当前不会打开串口，不会写入设备参数。</p><div className="review-checks">{['数据来源为离线演示','不会打开串口或发送控制命令','精度与保护按绑定策略分别观察'].map(text=><div key={text}><Check size={15}/>{text}</div>)}</div><div className="review-values"><div><span>电机 / 保持时间</span><strong>ID {frozen.motorId} / {frozen.durationS} s</strong></div><div><span>输出轴等效 Kp / Kd</span><strong>{frozen.kp} / {frozen.kd}</strong></div><div><span>前馈</span><strong>{frozen.torqueFfNm} N·m</strong></div><div><span>已发布保护参考</span><strong>{angle(frozen.motorId===7?STATIC_RELEASE.holdGuardDeg:.5)}° {axisLabel}</strong></div></div><div className="modal-actions"><button autoFocus className="button secondary" onClick={()=>studio.setWorkflow({stage:'idle',startedAt:0,frozen:null})}>返回修改</button><button className="button primary" onClick={studio.start}><Play size={14}/>开始离线演示</button></div></section></div>}
  </div>;
}

