/** @file 手动/静载/POS/七轴离线演示页面；实时预览不授予本页运动权限。 */
import { useState } from 'react';
import { ArrowRight, Check, CheckCircle2, Circle, FileCheck2, LockKeyhole, Play, RotateCcw, Square, HelpCircle } from 'lucide-react';
import { Badge, Note, PanelHeader, Value } from '../components/Common';
import { Chart, liveChartEndTime } from '../components/Chart';
import { MotorTable } from './Overview';
import { canExecute, COORDINATE_LABEL, validateSettings } from '../domain';
import { projectTelemetry } from '../feedbackStatus';
import type { Studio } from '../useStudio';
import type { Flow, Settings } from '../types';
/** 单个带单位参数，说明与输入直接关联，空输入保留无效值。 */
function Parameter({ name, label, unit, description, studio, minimum, maximum, step = .1 }: { name: keyof Settings; label: string; unit: string; description: string; studio: Studio; minimum: number; maximum: number; step?: number }) {
  const value = studio.settings[name];
  return <div className="parameter"><label htmlFor={name}>{label}<span>{unit}</span></label><div className="input-unit"><input id={name} type="number" min={minimum} max={maximum} step={step} value={Number.isFinite(value) ? value : ''} disabled={studio.running || studio.workflow.stage === 'review'} onChange={(event) => studio.setSettings((old) => ({...old,[name]:event.target.value === '' ? NaN : Number(event.target.value)}))}/><span>{unit}</span></div><small id={name+'-help'}>{description}</small></div>;
}
/** 通过统一审核页冻结参数，再执行纯离线演示。 */
export function Workflow({ studio, flow }: { studio: Studio; flow: Flow }) {
  const [manualMode, setManualMode] = useState<'pos'|'mit'>('pos');
  const effectiveFlow = flow === 'manual' && manualMode === 'mit' ? 'static' : flow;
  const isStatic = effectiveFlow === 'static' || flow === 'seven';
  const errors = validateSettings(effectiveFlow, studio.settings);
  const gate = canExecute(studio.mode, studio.running, errors);
  const gateReason = studio.mode==='live' ? '本页参数仅用于离线演示；真实操作见手动调试。' : gate.reason;
  const flangePreview=flow==='pos'&&studio.recording.motorId===7;
  const preview = flangePreview?studio.samples.map(entry=>projectTelemetry(entry,'flange',studio.mode==='live'?'native':studio.recording.coordinate,7)):
    studio.mode==='live' ? studio.samples.map(entry=>projectTelemetry(entry,'output','native',studio.settings.motorId)) : studio.samples;
  const previewCoordinate = flangePreview?'法兰端':studio.mode==='live' ? studio.settings.motorId===7?'电机输出轴':'标定未确认 · 位置/速度未知' : COORDINATE_LABEL[studio.recording.coordinate];
  const steps = ['配置参数','预检与审核','执行过程','本次结果'];
  const stepIndex = studio.running ? 2 : ['completed','stopped'].includes(studio.workflow.stage) ? 3 : studio.workflow.stage === 'review' ? 1 : 0;
  const frozen = studio.workflow.frozen;
  return <>
    <div className="steps">{steps.map((step,index)=><div className={(index === stepIndex ? 'current' : index < stepIndex ? 'done' : '')} key={step}><span>{index < stepIndex ? <Check size={14}/> : '0'+(index+1)}</span><strong>{step}</strong>{index<3&&<div/>}</div>)}</div>
    {flow==='seven' && <section className="panel"><PanelHeader title="逐轴检查" eyebrow="SEVEN AXES"><Badge tone="amber">逐轴遥测 / 离线演示</Badge></PanelHeader><MotorTable studio={studio}/></section>}
    <div className="workflow-grid"><section className="panel parameter-panel"><PanelHeader title="测试参数" eyebrow="CONFIGURATION"><Badge tone={studio.mode==='demo'?'blue':'amber'}>{studio.mode==='demo'?'仅用于离线演示':'只读 / 回放'}</Badge></PanelHeader>
      <div className="parameters-body"><div className="parameter axis-field"><label htmlFor="motor-select">目标电机<span>ID</span></label><select id="motor-select" value={studio.settings.motorId} disabled={studio.running || studio.workflow.stage==='review'} onChange={(event)=>studio.setSettings((old)=>({...old,motorId:Number(event.target.value)}))}>{Array.from({length:7},(_,index)=><option key={index} value={index+1}>J{index+1} · 电机 ID {index+1}</option>)}</select><small>{flow==='seven'?'演示展示选中轴；真实七轴须逐轴准入':'切换轴不会自动执行动作或换算未知标定'}</small></div>
      {flow==='manual'&&<div className="segmented mode-switch" aria-label="手动控制模式"><button disabled={studio.running} className={manualMode==='pos'?'selected':''} onClick={()=>setManualMode('pos')}>POS 位移演示</button><button disabled={studio.running} className={manualMode==='mit'?'selected':''} onClick={()=>setManualMode('mit')}>MIT 保持演示</button></div>}
      <div className="form-section-title"><span>{isStatic?'保持配置':'移动与保持'}</span><HelpCircle size={14}/></div>
      <div className="parameter-grid">
      {!isStatic&&<><Parameter name="angleDeg" label="相对转角" unit={flow==='pos'&&studio.settings.motorId===7?'° 法兰端':'°'} description={studio.settings.motorId===7?'法兰相对起点 · 演示范围±180°':'输出轴演示小角度 · 范围±3°'} studio={studio} minimum={studio.settings.motorId===7?-180:-3} maximum={studio.settings.motorId===7?180:3}/><Parameter name="speedDegS" label="移动速度" unit={flow==='pos'&&studio.settings.motorId===7?'°/s 法兰端':'°/s'} description="与相对转角使用同一坐标；演示不证明真机速度" studio={studio} minimum={.01} maximum={studio.settings.motorId===7?10:3}/></>}
      <Parameter name="durationS" label="保持时间" unit="s" description="仅计正式保持阶段，不包含准备与移动" studio={studio} minimum={1} maximum={60} step={1}/>
      {effectiveFlow==='static'&&<Parameter name="torqueFfNm" label="前馈力矩" unit="N·m" description="输出轴有符号补偿量 · 不是反馈峰值或目标负载" studio={studio} minimum={-3.5} maximum={3.5} step={.01}/>}
      {effectiveFlow==='static'&&<><Parameter name="kp" label="位置刚度 Kp" unit="N·m/rad" description="输出轴等效草稿量，不是CAN原生发包值" studio={studio} minimum={0} maximum={500}/><Parameter name="kd" label="速度阻尼 Kd" unit="N·m·s/rad" description="输出轴等效阻尼；实际模式与资格分别核对" studio={studio} minimum={0} maximum={5}/></>}
      </div><div className="form-section-title">装载记录 <span className="micro">不参与自动承载评级</span></div><div className="parameter-grid"><Parameter name="massG" label="额外挂重" unit="g" description="仅记录额外挂重，不包含臂杆和夹具自重" studio={studio} minimum={0} maximum={10000} step={1}/><Parameter name="leverMm" label="有效重力力臂" unit="mm" description="转轴到重力作用线的垂直距离，不一定等于杆长" studio={studio} minimum={0} maximum={1000} step={1}/></div>
      <Note>{isStatic?'保持本次捕获的当前位置；此流程不会自动移动到某个角度。':'相对转角与当前起点共同决定终点；真实执行仍须核对驱动量程和机械空间。'}</Note>
      {errors.length>0&&<div className="validation-error" role="alert">{errors.map((error)=><div key={error}>{error}</div>)}</div>}
      <div className="form-actions"><button className="button secondary" disabled={studio.running || studio.workflow.stage==='review'} onClick={()=>studio.setSettings((old)=>({...old,angleDeg:1,speedDegS:1,durationS:1,torqueFfNm:0,massG:0,leverMm:0}))}><RotateCcw size={15}/>恢复演示默认</button><button className="button primary" disabled={!gate.allowed||studio.workflow.stage==='review'} title={gateReason} onClick={()=>studio.review(effectiveFlow)}>预检与审核<ArrowRight size={16}/></button></div>{gateReason&&<p className="disabled-reason">{gateReason}</p>}
      </div></section>
      <aside className="workflow-aside"><section className="panel"><PanelHeader title="本次执行" eyebrow="SESSION"/><div className="session-summary"><div className="session-state"><span className={'state-emblem '+(studio.running?'active':'')}><FileCheck2 size={26}/></span><strong>{({idle:'等待配置',preflight:'演示预检',review:'等待审核',running:'模拟执行中',completed:'演示完成',stopped:'演示已停止'})[studio.workflow.stage]}</strong><Badge tone={studio.running?'green':'neutral'}>{studio.mode==='demo'?'合成流程':'本页为离线演示'}</Badge></div><div className="kv"><span>控制对象</span><strong>J{frozen?.motorId??studio.settings.motorId}</strong></div><div className="kv"><span>控制来源</span><strong>{studio.mode==='demo'?'浏览器离线演示':'真实操作见手动调试'}</strong></div><div className="kv"><span>实际模式读回</span><strong>本页未提供</strong></div><div className="kv"><span>硬件失能确认</span><strong>本页不授予动作资格</strong></div></div></section>
      <section className="panel explanation"><span className="eyebrow">PARAMETER NOTES</span><h3>参数易懂，状态可查。</h3><p><strong>编辑值</strong>还在本地草稿里，审核后冻结。</p><p><strong>模拟执行值</strong>仅用于生成演示数据。</p><p><strong>实际读回值</strong>仅来自真实设备响应；未连接时不填入演示值。</p><p>软件保护阈值与字段上限，都不是机械臂承载额定值。</p></section></aside></div>
    {studio.running&&<section className="panel running-panel"><div><Badge tone="green">模拟执行中</Badge><h2>{studio.elapsed < 1000 ? '准备与目标交接' : '移动 / 保持观察'}</h2><p><Value value={studio.elapsed/1000} digits={1} unit="s"/> / {(studio.duration/1000).toFixed(1)}s · 未发送设备命令</p></div><progress max={studio.duration} value={studio.elapsed}/><button className="button danger" onClick={studio.stop}><Square size={16}/>立即停止演示</button></section>}
    {['completed','stopped'].includes(studio.workflow.stage)&&<section className="panel result-panel"><CheckCircle2 size={28}/><div><h2>{studio.workflow.stage==='completed'?'演示流程完成':'演示已停止'}</h2><p>合成记录已保存在本次会话。硬件状态、失能和带载资格均未验证。</p></div><Badge>非硬件验收</Badge></section>}
    <section className="panel"><PanelHeader title="目标与反馈预览" eyebrow="SIGNAL MONITOR"/><Chart windowEndMs={liveChartEndTime(studio.duration,studio.now,studio.gateway.motors.find(motor=>motor.id===studio.settings.motorId)?.receivedAt)} windowMs={studio.mode==='live'?60000:undefined} gapMs={studio.mode==='live'?500:250} samples={preview} events={studio.recording.events} coordinateLabel={previewCoordinate}/></section>
    {studio.workflow.stage==='review'&&frozen&&<div className="modal-backdrop"><section className="modal review-modal" role="dialog" aria-modal="true" aria-labelledby="review-title"><div className="modal-icon"><LockKeyhole size={25}/></div><span className="eyebrow">REVIEW BEFORE RUN</span><h2 id="review-title">核对本次演示</h2><p>以下参数已冻结。确认后只在浏览器生成合成反馈。</p><div className="review-checks">{['数据来源为离线演示','不会打开串口或发送控制命令','本次结果不作为实机验收'].map((text)=><div key={text}><Check size={16}/>{text}</div>)}</div><div className="review-values"><div><span>电机</span><strong>ID {frozen.motorId}</strong></div><div><span>保持时间</span><strong>{frozen.durationS} s</strong></div>{!isStatic&&<div><span>相对转角</span><strong>{frozen.angleDeg} °{flow==='pos'&&frozen.motorId===7?' 法兰端':''}</strong></div>}<div><span>前馈</span><strong>{effectiveFlow==='static'?frozen.torqueFfNm:0} N·m</strong></div></div><div className="modal-actions"><button autoFocus className="button secondary" onClick={()=>studio.setWorkflow({stage:'idle',startedAt:0,frozen:null})}><Circle size={14}/>返回修改</button><button className="button primary" onClick={studio.start}><Play size={15}/>开始离线演示</button></div></section></div>}
  </>;
}

