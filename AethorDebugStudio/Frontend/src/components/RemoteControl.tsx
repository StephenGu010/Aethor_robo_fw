/** @file 真实设备独立审核入口：读取能力、冻结草稿、显式提交一次、读取原事务终态。 */
import { useEffect, useRef, useState } from 'react';
import type { Studio } from '../useStudio';
import type {ControlReadback} from '../types';
import { remoteDraftError, remoteDefersFeedback, remoteTerminalConfirmed,remoteHomeError,describeRemoteTerminal, remoteRequest, remoteLatest, type RemoteDraft, type RemoteReceipt } from '../remoteControl';
import { monitorRemoteTransaction } from '../remoteMonitor';
import { runRemoteWorkflow } from '../remoteWorkflow';
import {endRemoteSession,describeRemoteOwner,confirmSessionReleased,remoteFaultReleaseAllowed} from '../remoteSession';
import { displayDegreesToOutputRadians,outputRadiansToDisplayDegrees,type ControlCoordinate } from '../axisUnits';
import { captureTelemetry,projectTelemetry } from '../feedbackStatus';
import {relativeFlangeSample} from '../flangeZero';
import {TorqueEnvelope} from './TorqueEnvelope';
const LABELS: Record<RemoteDraft['operation'], string> = { acquire:'申请远程控制权', release:'释放远程控制权', pos:'单次POS相对运动', pos_load:'POS带载移动并保持',home:'回到软件零点', configure:'下发静载RAM参数', hold:'单次静载保持', lock:'撤回静载RAM配置', preflight:'单轴失能预检', mode_mit:'切换MIT模式', mode_pos:'切换POS模式' };
const PARAMETER_HELP = {
  delta:'POS动作相对当前姿态的转角；正负号表示方向。',
  speed:'POS移动的输出轴速度，必须在设备确认范围内。',
  kp:'位置偏差后的拉回力度，允许80～200；默认120。',
  kd:'抑制速度变化的阻尼，允许0.20～0.40；默认0.30。',
  torque:'有符号前馈补偿，不是实测转矩，也不是追求恒定力矩。',
  duration:'正式保持时间，范围1～60秒；准备和移动阶段单独计算。',
} as const;
const VISIBLE_FIELDS:Record<RemoteDraft['operation'],readonly (keyof typeof PARAMETER_HELP)[]>={
  acquire:[],release:[],pos:['delta','speed'],pos_load:['delta','speed','duration'],home:[],
  configure:['kp','kd','torque','duration'],hold:['kp','kd','torque','duration'],
  lock:[],preflight:[],mode_mit:[],mode_pos:[],
};
/** 独立远程面板；任何断连/换轴/超时均作废审核，不产生后台动作。 */
export function RemoteControl({studio,initialOperation='configure',displayCoordinate='output',onControlReadback}:{studio:Studio;initialOperation?:RemoteDraft['operation'];displayCoordinate?:ControlCoordinate;onControlReadback?:(value:ControlReadback|null)=>void}) {
  const [draft,setDraft] = useState<RemoteDraft>({operation:initialOperation,delta:0,speed:.02,kp:initialOperation==='pos_load'?0:120,kd:initialOperation==='pos_load'?0:.3,torque:0,duration:1000});
  const [caps,setCaps] = useState<RemoteReceipt|null>(null);
  const [review,setReview] = useState<{draft:RemoteDraft;token:number;expires:number}|null>(null);
  const [receipt,setReceipt] = useState<RemoteReceipt|null>(null);
  const [transactionId,setTransactionId] = useState(0);
  const [message,setMessage] = useState('尚未读取设备能力；旧固件不支持时保持关闭');
  const [requestBusy,setBusy] = useState(false);
  const [torqueBusy,setTorqueBusy] = useState(false);
  const busy=requestBusy||torqueBusy||studio.torqueMaintenanceBusy;
  const [monitoring,setMonitoring] = useState(false);
  const [workflowUncertain,setWorkflowUncertain] = useState(false);
  const [homeSpeed,setHomeSpeed]=useState(1);
  const [homeReview,setHomeReview]=useState<{draft:RemoteDraft;expires:number;current:number;target:number;createdAt:number}|null>(null);
  const latestZero=useRef(studio.flangeZero);latestZero.current=studio.flangeZero;
  const inFlight = useRef(false);
  const pendingOperation = useRef<RemoteDraft['operation']|null>(null);
  const connectionVersion = useRef(0);
  const controlIntentVersion=studio.controlIntentVersion.current;
  const available = studio.mode === 'live' && studio.gateway.connected && studio.settings.motorId === 7;
  const coordinateLabel=displayCoordinate==='flange'?'法兰':'输出轴';
  const motor=studio.gateway.motors.find(entry=>entry.id===studio.settings.motorId);
  const current=displayCoordinate==='flange'&&motor?.id===7?projectTelemetry(captureTelemetry(motor,studio.gateway.connected,Date.now(),0),'flange','native',7):null;
  const zero=studio.flangeZero;
  const relativeCurrent=zero&&current?relativeFlangeSample(current,zero):null;
  const zeroChangesLocked=busy||monitoring||transactionId!==0||workflowUncertain||studio.remoteMotionLocked||!!review||!!homeReview;
  const zeroCapsMatch=!!caps&&caps.fields.active_id==='0'&&caps.fields.owner==='0'&&!!studio.flangeIdentity&&
    Number(caps.fields.boot)===studio.flangeIdentity.boot&&Number(caps.fields.coord)===studio.flangeIdentity.reference;
  // 仅回传本次能力查询的显示快照；清空能力或离开页面时同步作废，不额外发包。
  useEffect(()=>{
    onControlReadback?.(caps?{label:describeRemoteOwner(caps.fields.owner,caps.fields.authority),readAt:Date.now(),connectionKey:`${studio.gateway.port}:${studio.settings.motorId}`}:null);
    return()=>onControlReadback?.(null);
  },[caps,onControlReadback]);
  useEffect(() => { connectionVersion.current++;pendingOperation.current=null;setCaps(null);setReview(null);setTransactionId(0); return () => { connectionVersion.current++; }; },[studio.gateway.port,studio.gateway.connected,studio.settings.motorId,studio.mode]);
  useEffect(() => {setReview(null);setHomeReview(null);setCaps(null);},[controlIntentVersion]);
  useEffect(()=>{setHomeReview(null);},[zero]);
  /** 释放后能力核实尚未完成时，任何自动/手动恢复均必须保留该核实阶段。 */
  async function terminalCanUnlock(result:RemoteReceipt,operation:RemoteDraft['operation']|null,isCurrent:()=>boolean):Promise<boolean>{
    const recovery=studio.remoteSessionRecovery.current;
    if(!recovery)return isCurrent()&&remoteTerminalConfirmed(result,operation);
    const confirmed=await confirmSessionReleased(recovery,result,{request:remoteRequest,isCurrent,wait:async()=>{},
      observe:(_step,proof)=>{if(isCurrent())setCaps(proof.result==='capability'?proof:null);},
    });
    if(!confirmed||!isCurrent()||studio.remoteSessionRecovery.current!==recovery)return false;
    studio.remoteSessionRecovery.current=null;
    return true;
  }
  /** 一次显式点击冻结参数并顺序执行；离开页面、断连或换轴后不启动下一步。 */
  async function startWorkflow(input:RemoteDraft=draft) {
    if (!available || inFlight.current || monitoring || transactionId || workflowUncertain||studio.remoteMotionLocked||studio.torqueMaintenanceBusy) return;
    const error=remoteDraftError(input);
    if(error){setMessage(error);return;}
    const frozen={...input},version=connectionVersion.current,intentVersion=studio.controlIntentVersion.current;
    const homeZero=input.operation==='home'?latestZero.current:null;
    inFlight.current=true;setBusy(true);setReview(null);setHomeReview(null);setCaps(null);studio.lockRemoteMotion(true);
    try {
      const result=await runRemoteWorkflow(frozen,{
        request:remoteRequest,isCurrent:()=>version===connectionVersion.current&&intentVersion===studio.controlIntentVersion.current&&(!homeZero||latestZero.current===homeZero),
        wait:()=>new Promise(resolve=>setTimeout(resolve,500)),
        observe:(step,value)=>{if(version===connectionVersion.current){setReceipt(value);setMessage(`${LABELS[step as RemoteDraft['operation']]} · ${value.result}`);}},
      });
      const uncertain=['unknown','timeout'].includes(result.result)||(result.result==='cancelled'&&result.transactionId>0);
      studio.lockRemoteMotion(uncertain||result.transactionId>0);
      if(version===connectionVersion.current){pendingOperation.current=result.operation;setMessage(result.detail);setTransactionId(result.transactionId);setWorkflowUncertain(uncertain);}
    }finally{inFlight.current=false;setBusy(false);}
  }
  /** 仅跟踪当前连接上的原事务；退出页面或断连后忽略迟到结果，动作绝不补发。 */
  useEffect(() => {
    if (!available || !transactionId || busy) return;
    let current = true;
    let terminal:RemoteReceipt|null=null;
    const version = connectionVersion.current;
    const intentVersion=studio.controlIntentVersion.current;
    setMonitoring(true);
    void monitorRemoteTransaction(transactionId, {
      request: remoteRequest,
      wait: () => new Promise(resolve => setTimeout(resolve, 500)),
      isCurrent: () => current && version === connectionVersion.current,
      observe: result => { terminal=result;setReceipt(result); setMessage(result.result === 'pending' ? `事务 #${transactionId} 执行中，正在自动读取结果` : describeRemoteTerminal(result,pendingOperation.current)); },
    }).then(async outcome => {
      if (!current || version !== connectionVersion.current) return;
      setMonitoring(false);
      if (['completed','failed','rejected'].includes(outcome)) {
        const confirmed=terminal&&await terminalCanUnlock(terminal,pendingOperation.current,()=>current&&version===connectionVersion.current&&intentVersion===studio.controlIntentVersion.current);
        if(!current||version!==connectionVersion.current||intentVersion!==studio.controlIntentVersion.current)return;
        if(confirmed){setTransactionId(0);setWorkflowUncertain(false);studio.lockRemoteMotion(false);}
        else {setWorkflowUncertain(true);studio.lockRemoteMotion(true);setMessage(studio.remoteSessionRecovery.current?'释放事务已有回执，但当前控制权或启动号尚未核实；保持锁定，只能只读恢复。':terminal?describeRemoteTerminal(terminal,pendingOperation.current):`事务 #${transactionId} 缺少准入或失能确认；保持锁定，只能继续查询原事务`);}
      }
      else if (outcome !== 'cancelled') {studio.lockRemoteMotion(true);setMessage(`事务 #${transactionId} ${outcome === 'timeout' ? '等待超过查询上限' : '结果未知'}；动作未重发，可继续读取原事务`);}
    });
    return () => { current = false; setMonitoring(false); };
  }, [available, transactionId, busy, studio.gateway.port]);
  /** 锁定双击并保持最终结果；不执行补发。 */
  async function execute(body: object, submitted=false) {
    if (inFlight.current||studio.torqueMaintenanceBusy) return;
    inFlight.current=true;setBusy(true);if(submitted)studio.lockRemoteMotion(true);
    const version=connectionVersion.current;
    const intentVersion=studio.controlIntentVersion.current;
    try {
      const result = await remoteRequest(body);
      if(version!==connectionVersion.current)return;
      setReceipt(result);setMessage(['completed','failed','rejected'].includes(result.result)?describeRemoteTerminal(result,pendingOperation.current):result.detail);
      if (result.operation === 'caps') {
        setCaps(result.result==='capability'?result:null);setReview(null);
        const activeId=Number(result.fields.active_id??0);
        if(Number.isInteger(activeId)&&activeId>0){studio.lockRemoteMotion(true);pendingOperation.current=null;setTransactionId(activeId);setMessage(`设备保留原事务 #${activeId}（启动 ${result.fields.boot}）；仅查询结果，不补发动作`);}
      }
      if (submitted) {setTransactionId(result.result==='rejected'?0:result.requestId);setCaps(null);if(result.result==='rejected')studio.lockRemoteMotion(false);}
      if (result.operation==='result' && ['completed','failed','rejected'].includes(result.result)) {
        const confirmed=await terminalCanUnlock(result,pendingOperation.current,()=>version===connectionVersion.current&&intentVersion===studio.controlIntentVersion.current);
        if(version!==connectionVersion.current||intentVersion!==studio.controlIntentVersion.current)return;
        if(confirmed){setTransactionId(0);setWorkflowUncertain(false);studio.lockRemoteMotion(false);}
        else {setWorkflowUncertain(true);studio.lockRemoteMotion(true);setMessage(studio.remoteSessionRecovery.current?'释放事务已有回执，但当前控制权或启动号尚未核实；保持锁定，只能只读恢复。':describeRemoteTerminal(result,pendingOperation.current));}
      }
    } catch(error) {setMessage(String(error)+'；结果未知，未重发');setCaps(null);if(submitted)setWorkflowUncertain(true);}
    finally {inFlight.current=false;setBusy(false);}
  }
  /** 使用30秒内设备能力及输出轴范围冻结参数，RID50从不参与资格。 */
  function freeze() {
    const error=remoteDraftError(draft);
    if(error){setMessage(error);return;}
    if(!caps || (caps.fields.fresh!=='1'&&!remoteDefersFeedback(draft.operation,caps.fields)&&!(draft.operation==='preflight'&&caps.fields.preflight==='1'))){setMessage('缺少新鲜标准反馈，寄存器位置不能授予运动资格');return;}
    if (draft.operation==='pos' && (caps.fields.pos!=='1'|| Math.abs(draft.delta)>Number(caps.fields.max_delta)||draft.speed>Number(caps.fields.max_speed))) {setMessage('POS参数超出设备确认范围或能力未开放');return;}
    if (draft.operation==='pos_load' && (caps.fields.load!=='1' || caps.fields.mode!=='2' ||
      !Number.isFinite(Number(caps.fields.load_delta)) || !Number.isFinite(Number(caps.fields.load_speed)) ||
      Math.abs(draft.delta)>Number(caps.fields.load_delta) || draft.speed>Number(caps.fields.load_speed))) {setMessage('设备未开放专用POS带载能力，或模式/范围不匹配；普通POS不能替代');return;}
    if (['configure','hold','lock'].includes(draft.operation) && caps.fields.static!=='1'){setMessage('该固件未开放单轴静载能力');return;}
    const token=Number(caps.fields.token);
    if(!Number.isInteger(token)||token<=0){setMessage('设备审核令牌无效');return;}
    setReview({draft:{...draft},token,expires:Date.now()+30000});
  }
  /** 消费审核后只提交一次，过期不会刷新能力或自动继续。 */
  function submit() {
    if(!review||busy)return;
    const frozen=review;setReview(null);
    if(Date.now()>frozen.expires){setMessage('审核已过期，请重新读取能力并审核');return;}
    pendingOperation.current=frozen.draft.operation;
    void execute({...frozen.draft,token:frozen.token,motorId:7,coordinate:'output-rad'},true);
  }
  /** 恢复HTTP超时前已发送动作的编号，只读缓存，不重新发动作。 */
  async function recover() {
    if(inFlight.current)return;
    inFlight.current=true;setBusy(true);
    const version=connectionVersion.current;
    try {
      const result=await remoteLatest();
      if(version!==connectionVersion.current)return;
      setReceipt(result);setReview(null);setCaps(null);
      if(result){
        pendingOperation.current=Object.hasOwn(LABELS,result.operation)?result.operation as RemoteDraft['operation']:null;
        setMessage(result.detail);setTransactionId(result.operation==='result'?Number(result.fields.id??0):result.operation==='caps'||result.result==='rejected'?0:result.requestId);
      }
      else setMessage('网关没有保留远程回执');
    }catch(error){setMessage(String(error));}
    finally{inFlight.current=false;setBusy(false);}
  }
  /** 从本次设备能力中恢复最近终态编号，跨网关重启也只读原结果。 */
  function recoverTerminal() {
    const id=Number(caps?.fields.terminal_id??0);
    if(Number.isInteger(id)&&id>0){pendingOperation.current=null;setTransactionId(id);setReview(null);setMessage(`恢复设备启动 ${caps?.fields.boot} 的原事务 #${id}；只读查询终态`);}
  }
  /** 独立结束按钮只核对原事务并释放控制；不会沿用运动草稿或自动恢复配额。 */
  async function endSession() {
    if(!available||inFlight.current||monitoring||transactionId||workflowUncertain||studio.remoteMotionLocked||review||homeReview)return;
    const version=connectionVersion.current,intentVersion=studio.controlIntentVersion.current;
    const isCurrent=()=>version===connectionVersion.current&&intentVersion===studio.controlIntentVersion.current;
    inFlight.current=true;setBusy(true);studio.lockRemoteMotion(true);
    try{
      const result=await endRemoteSession({request:remoteRequest,isCurrent,
        releaseIntent:boot=>{studio.remoteSessionRecovery.current={boot,transactionId:0};},
        wait:()=>new Promise(resolve=>setTimeout(resolve,500)),
        observe:(step,value)=>{if(isCurrent()){setReceipt(value);if(step==='caps')setCaps(value.result==='capability'?value:null);}},
      });
      if(result.operation==='release'&&studio.remoteSessionRecovery.current)studio.remoteSessionRecovery.current.transactionId=result.transactionId;
      if(!isCurrent())return;
      if(result.result==='completed'||result.result==='rejected')studio.remoteSessionRecovery.current=null;
      const uncertain=['unknown','timeout'].includes(result.result)||result.transactionId>0;
      pendingOperation.current=result.operation;setMessage(result.detail);setTransactionId(result.transactionId);
      setWorkflowUncertain(uncertain);studio.lockRemoteMotion(uncertain);
    }finally{inFlight.current=false;setBusy(false);}
  }
  /** 先核对板端空闲和坐标身份，设零只改变共享软件参考。 */
  function captureZero() {
    if(zeroChangesLocked||!zeroCapsMatch)return;
    setMessage(studio.setCurrentFlangeZero()?'软件零点已设置；未发送运动命令':'缺少新鲜位置或当前状态不允许设零');
  }
  /** 新取独立home能力并冻结绝对目标；审核过程不申请控制权。 */
  async function freezeHome() {
    if(!available||zeroChangesLocked||!zero||current?.positionDeg==null||caps?.fields.home!=='1')return;
    const frozenZero=zero,version=connectionVersion.current,intentVersion=studio.controlIntentVersion.current;
    const frozen:RemoteDraft={operation:'home',delta:displayDegreesToOutputRadians(zero.positionDeg,'flange'),speed:displayDegreesToOutputRadians(homeSpeed,'flange'),kp:zero.identity.boot,kd:zero.identity.reference,torque:0,duration:1000};
    const invalid=remoteDraftError(frozen);
    if(invalid){setMessage(invalid);return;}
    inFlight.current=true;setBusy(true);
    try {
      const capability=await remoteRequest({operation:'caps'});
      if(version!==connectionVersion.current||intentVersion!==studio.controlIntentVersion.current||latestZero.current!==frozenZero)return;
      setCaps(capability.result==='capability'?capability:null);
      const error=remoteHomeError(frozen,capability.fields);
      if(capability.result!=='capability'||error||capability.fields.active_id!=='0'||capability.fields.owner!=='0'||(capability.fields.fresh!=='1'&&!remoteDefersFeedback('home',capability.fields))){setMessage(error??'设备已有控制事务或缺少独立回零预检能力');return;}
      const displacement=displayDegreesToOutputRadians(frozenZero.positionDeg-current.positionDeg,'flange');
      if(Math.abs(displacement)>Number(capability.fields.load_delta)){setMessage('预计回零位移超出设备POS范围；未申请控制权');return;}
      setHomeReview({draft:frozen,expires:Date.now()+30000,current:current.positionDeg,target:frozenZero.positionDeg,createdAt:frozenZero.createdAt});
    }catch(error){setMessage(String(error)+'；未提交回零动作');}
    finally{inFlight.current=false;setBusy(false);}
  }
  /** 一次性消费回零审核；身份或30秒期限变化后不执行。 */
  function submitHome() {
    if(!homeReview||busy||current?.positionDeg==null)return;
    const frozen=homeReview;setHomeReview(null);
    if(!latestZero.current||latestZero.current.createdAt!==frozen.createdAt||latestZero.current.identity.boot!==frozen.draft.kp||latestZero.current.identity.reference!==frozen.draft.kd||Date.now()>frozen.expires){setMessage('回零审核已失效，请重新读取并审核');return;}
    void startWorkflow(frozen.draft);
  }
  return <section className="panel device-debug remote-control" aria-label="远程参数与单轴控制"><div className="device-debug-body">
    <div className="remote-heading"><span className="eyebrow">PARAMETERS & CONTROL</span><h3>远程参数与单轴控制 · ID7</h3><p>位移与速度使用{displayCoordinate==='flange'?'法兰端':'电机输出轴'} ° / °/s。配置仅存RAM；读取能力和编辑草稿不会启动动作。远程通信失联5秒触发原STOP保护。</p></div>
    <div className="remote-capability-check" aria-label="检查设备与控制权限">
    <button className="button secondary" disabled={!available||busy||transactionId!==0} onClick={()=>void execute({operation:'caps'})}>读取远程能力</button>
    <p className="micro">检查设备与控制权限 · 只读取，不启动。先确认供电、ID7及反馈，再设置参数。</p>
    {!available&&<p className="validation-error">请先连接真实设备并选择ID7；其他轴只开放诊断与停止/失能。</p>}
    {caps&&<p>标准反馈：{caps.fields.fresh==='1'?'新鲜':'缺失/过期'} · 控制权：{describeRemoteOwner(caps.fields.owner,caps.fields.authority)} · 当前模式 {caps.fields.mode==='1'?'MIT':caps.fields.mode==='2'?'POS':'未知'} · 固件支持POS：{caps.fields.pos==='1'?'是':'未确认'} / 静载：{caps.fields.static==='1'?'是':'未确认'}</p>}
    {caps?.fields.fresh!=='1'&&caps&&<p>当前缺少新鲜标准反馈；待机寄存器位置不能代替测试准入反馈。POS / 静载为1仅表示固件开放该功能，不表示现在可以启动。</p>}
    {caps?.fields.fresh!=='1'&&caps?.fields.defer==='1'&&<p>板端允许先提交预检意图；真正运动仍等待DISABLE之后的新标准反馈，缺失时由板端拒绝。</p>}
    </div>
    <section className="workflow-step" data-workflow-step="parameters" aria-labelledby="parameter-step-title"><div className="workflow-step-title"><span>02</span><div><h2 id="parameter-step-title">设置本次参数</h2><p>{draft.operation==='hold'||draft.operation==='configure'?'静载使用当前位置保持；下方增益与转矩均为电机输出轴口径。':draft.operation==='pos_load'?'相对当前姿态移动后保持；位移正负决定方向，速度必须为正。':'单次操作按所选能力执行；参数不会自动下发。'}</p></div></div>
    {studio.torqueMaintenanceUncertain&&<div role="alert"><p>本次转矩维护结果尚未核实，禁止再次启动或维护。旧测试终态不能解除此锁；请只读核验当前身份、失能与保护策略。若仍缺标准反馈，可在独立承托后使用上方明确的失能入口，再只读核验。</p><button type="button" className="button secondary" disabled={!available||busy} onClick={()=>void studio.recoverTorquePreparation()}>只读核验维护状态</button></div>}
    <fieldset className="remote-fields" disabled={busy||!!review||!!homeReview||transactionId!==0||studio.remoteMotionLocked}><legend>本次参数 · 启动时冻结</legend>
      <label className="remote-operation">本次操作<select value={draft.operation} onChange={event=>{const operation=event.target.value as RemoteDraft['operation'];setDraft({...draft,operation,delta:VISIBLE_FIELDS[operation].includes('delta')?draft.delta:0,speed:VISIBLE_FIELDS[operation].includes('speed')?draft.speed:.02,duration:VISIBLE_FIELDS[operation].includes('duration')?draft.duration:1000,kp:operation==='pos_load'?0:120,kd:operation==='pos_load'?0:.3,torque:0});}}>{Object.entries(LABELS).filter(([value])=>value!=='home').map(([value,label])=><option value={value} key={value}>{label}</option>)}</select><small>常用测试可直接编辑下方参数；控制权、模式及预检为高级操作，执行影响见启动摘要。</small></label>
      {([['delta',coordinateLabel+'相对位移 °'],['speed',coordinateLabel+'速度 °/s'],['kp','输出轴Kp N·m/rad'],['kd','输出轴Kd N·m·s/rad'],['torque','输出轴前馈 N·m'],['duration','保持时间 s']] as const).filter(([key])=>VISIBLE_FIELDS[draft.operation].includes(key)).map(([key,label])=><label key={key}>{label}<input aria-label={label} aria-describedby={'remote-'+key+'-help'} type="number" step="any" value={Number.isFinite(draft[key])?(key==='delta'||key==='speed'?Number(outputRadiansToDisplayDegrees(draft[key],displayCoordinate).toFixed(6)):key==='duration'?draft[key]/1000:draft[key]):''} onChange={event=>{const value=event.target.value===''?NaN:Number(event.target.value);setDraft({...draft,[key]:key==='delta'||key==='speed'?displayDegreesToOutputRadians(value,displayCoordinate):key==='duration'?value*1000:value});}}/><small id={'remote-'+key+'-help'}>{key==='speed'?`POS移动的${coordinateLabel}速度，必须在设备确认范围内。`:draft.operation==='pos_load'&&key==='duration'?'到位后保持1至60整秒。':PARAMETER_HELP[key]}</small></label>)}
    </fieldset>
    {caps&&VISIBLE_FIELDS[draft.operation].includes('delta')&&<p className="parameter-range">设备本次范围：{Number.isFinite(Number(caps.fields[draft.operation==='pos_load'?'load_delta':'max_delta']))?`相对位移绝对值 ≤ ${outputRadiansToDisplayDegrees(Number(caps.fields[draft.operation==='pos_load'?'load_delta':'max_delta']),displayCoordinate).toFixed(3)}°`:'位移上限尚未取得'}；{Number.isFinite(Number(caps.fields[draft.operation==='pos_load'?'load_speed':'max_speed']))?`速度 ≤ ${outputRadiansToDisplayDegrees(Number(caps.fields[draft.operation==='pos_load'?'load_speed':'max_speed']),displayCoordinate).toFixed(3)}°/s`:'速度上限尚未取得'}。以启动时重新读取的能力为准。</p>}
    </section>
    <section className="workflow-step" data-workflow-step="protection" aria-labelledby="protection-step-title"><div className="workflow-step-title"><span>03</span><div><h2 id="protection-step-title">转矩保护</h2><p>草稿、板端读回与授权分别核对；调整退出线不是启动指令。</p></div></div>
    <TorqueEnvelope available={available} locked={requestBusy||monitoring||transactionId!==0||workflowUncertain||studio.remoteMotionLocked||(studio.torqueMaintenanceBusy&&!torqueBusy)||!!review||!!homeReview}
      preparationAvailable={studio.gateway.torquePreparationAvailable===true} prepareAllowance={studio.prepareTorque}
      identity={`${studio.gateway.port}:${controlIntentVersion}:${studio.flangeIdentity?.boot??0}:${studio.flangeIdentity?.reference??0}`} onBusyChange={setTorqueBusy}/>
    </section>
    <section className="workflow-step launch-step" data-workflow-step="start" aria-labelledby="launch-step-title"><div className="workflow-step-title"><span>04</span><div><h2 id="launch-step-title">审核与启动</h2><p>阅读下方摘要后再启动。主按钮会直接执行所选流程，不是仅打开审核。</p></div></div>
      <div className="launch-summary" aria-label="本次启动摘要"><strong>ID7 · {LABELS[draft.operation]}</strong>
        <p>{draft.operation==='hold'?`准备MIT模式和静载RAM参数，在当前位置保持 ${draft.duration/1000} 秒，然后停止、确认失能并释放控制权。保持可能纠偏运动。`:draft.operation==='pos_load'?`准备POS模式，从当前姿态向${draft.delta<0?'负':'正'}方向移动 ${outputRadiansToDisplayDegrees(draft.delta,displayCoordinate).toFixed(3)} ${coordinateLabel}°，速度 ${outputRadiansToDisplayDegrees(draft.speed,displayCoordinate).toFixed(3)} °/s，到位保持 ${draft.duration/1000} 秒，然后停止并确认失能、释放控制权。`:draft.operation==='configure'?'核对设备能力与新鲜反馈后下发当前静载RAM参数；需要时先准备MIT模式。此操作不自动执行失能预检、不启动保持，也不进行掉电保存。':draft.operation==='pos'?`执行一次相对POS运动：${coordinateLabel} ${outputRadiansToDisplayDegrees(draft.delta,displayCoordinate).toFixed(3)}°，速度 ${outputRadiansToDisplayDegrees(draft.speed,displayCoordinate).toFixed(3)}°/s；随后核对原事务及失能。`:'按所选操作执行控制权、模式或失能预检事务；步骤可能撤去保持力矩，请核对本次操作与承托条件。'}</p>
        {(draft.operation==='hold'||draft.operation==='configure')&&<p>输出轴 Kp {draft.kp} · Kd {draft.kd} · 前馈 {draft.torque} N·m。保护以板端本次实际读回为准；任一步失败即停止推进，不自动重发。</p>}
      </div>
    <button className="button danger" disabled={!available||busy||monitoring||transactionId!==0||!!review||!!homeReview||workflowUncertain||studio.remoteMotionLocked} onClick={()=>void startWorkflow()}>按参数启动一次 · {LABELS[draft.operation]}</button>
    {(workflowUncertain||studio.remoteMotionLocked)&&!busy&&<p role="alert">上次启动结果尚未确认；请恢复原事务并读取终态，不能重新启动。</p>}
      <details className="advanced-controls"><summary>高级分步操作</summary><p>独立冻结和提交所选底层操作。审核不会启动；“显式提交一次”会执行当前操作。保留给需要逐步检查的调试流程。</p>
    <div className="device-debug-actions"><button className="button secondary" disabled={!available||busy||!caps||!!review||!!homeReview||transactionId!==0||studio.remoteMotionLocked} onClick={freeze}>审核并冻结</button>
    <button className="button danger" disabled={!available||busy||!review||workflowUncertain||studio.remoteMotionLocked} onClick={submit}>显式提交一次</button>
    <button className="button secondary" disabled={!review||busy} onClick={()=>setReview(null)}>返回草稿</button>
</div>
    {review&&<p>已冻结：{LABELS[review.draft.operation]}，{coordinateLabel}δ={outputRadiansToDisplayDegrees(review.draft.delta,displayCoordinate).toFixed(3)}°，v={outputRadiansToDisplayDegrees(review.draft.speed,displayCoordinate).toFixed(3)}°/s，Kp={review.draft.kp}，Kd={review.draft.kd}，τ={review.draft.torque} N·m，{review.draft.duration} ms。</p>}
      </details>
    </section>
    <section className="workflow-step result-step" data-workflow-step="results" aria-labelledby="result-step-title"><div className="workflow-step-title"><span>05</span><div><h2 id="result-step-title">结果与恢复</h2><p>终态、失能和控制权分别核对。未知结果只能查询原事务，不能重复启动。</p></div></div>
      <p className="remote-status" role="status">{message}</p>
      {receipt&&<dl className="result-summary"><div><dt>测试 / 请求编号</dt><dd>#{receipt.fields.id??receipt.requestId}</dd></div><div><dt>板端结果</dt><dd>{receipt.result==='completed'?'已完成回执':receipt.result==='failed'?'失败':receipt.result==='rejected'?'已拒绝':receipt.result==='capability'?'只读能力':receipt.result==='queued'?'已排队':receipt.result==='pending'?'进行中':'尚未确认'}</dd></div><div><dt>失能证明</dt><dd>{receipt.fields.disabled==='1'?'板端已确认':'未提供或未确认'}</dd></div></dl>}
<div className="device-debug-actions">
    <button className="button secondary" disabled={!available||busy||monitoring||transactionId===0} onClick={()=>void execute({operation:'result',transactionId})}>{monitoring?'正在自动跟踪原事务':'读取原事务结果'}</button>
    <button className="button secondary" disabled={busy||monitoring} onClick={()=>void recover()}>恢复最近事务编号</button></div>
    {Number(caps?.fields.terminal_id??0)>0&&<button className="button secondary" disabled={busy} onClick={recoverTerminal}>恢复设备最近终态 #{caps?.fields.terminal_id}</button>}
    <div className="remote-session" role="region" aria-label="控制会话">
      <div><strong>控制会话 · {caps?describeRemoteOwner(caps.fields.owner,caps.fields.authority):'待读取'}</strong><p>测试失败且确认失能后，可单独结束原会话。按钮会核对原事务，只提交一次释放，不执行运动或恢复配额。</p></div>
      <button type="button" className="button secondary session-end-button" disabled={!available||busy||monitoring||transactionId!==0||workflowUncertain||studio.remoteMotionLocked||!!review||!!homeReview} onClick={()=>void endSession()}>结束会话／释放控制权</button>
      {(workflowUncertain||studio.remoteMotionLocked)&&<p>原事务尚未确认：请先恢复最近终态并读取原事务结果；失能未确认时不能结束会话。</p>}
      {caps&&remoteFaultReleaseAllowed(caps.fields)&&<p className="session-guidance">板端允许审核结束原上位机故障会话。仍会核对原事务失能证明、连接身份和运输清理；不会清除转矩配额记录或启动下一次测试。</p>}
      {caps?.fields.authority==='3'&&!remoteFaultReleaseAllowed(caps.fields)&&<p className="session-guidance">本地故障锁定尚未通过板端安全结束审核。请核对原事务失能证明、运输清理和连接身份，不能强制解除。</p>}
      {caps?.fields.owner==='1'&&caps.fields.authority===undefined&&<p className="session-guidance">当前协议未区分LCD实际控制与本地故障锁定；旧固件也未提供安全结束审核入口，仅更新网页无法解除。需升级对应固件，或在LCD控制准备页按原提示完成恢复并释放；上位机不会强行接管。</p>}
      {caps?.fields.owner==='1'&&caps.fields.authority!==undefined&&caps.fields.authority!=='3'&&<p className="session-guidance">LCD本地控制仍占用。请在LCD控制准备页结束并释放，再读取远程能力；上位机不会接管真实LCD会话。</p>}
    </div>
      {receipt&&<details className="protocol-details"><summary>原始协议回执</summary><p className="remote-receipt">#{receipt.requestId} · {receipt.result} · {Object.entries(receipt.fields).map(([key,value])=>`${key}=${value}`).join(' ')}</p></details>}
    </section>
    {displayCoordinate==='flange'&&<section className="workflow-step flange-zero-card" aria-labelledby="zero-card-title"><h2 id="zero-card-title">法兰零点与回零</h2><p>设零只设置软件参考；回零是真实运动。两者独立于本次相对位移参数。</p>
    {displayCoordinate==='flange'&&<><div className="remote-live-values" aria-label="法兰实时反馈"><div><small>原始法兰角度（绝对原始坐标）{current?.positionSource==='register50'?' · 0x50换算':''}</small><output aria-label="当前法兰角度">{current?.positionDeg==null?'—':current.positionDeg.toFixed(3)} °</output></div><div><small>相对零点角度</small><output aria-label="相对零点角度">{relativeCurrent?.positionDeg==null?'—':relativeCurrent.positionDeg.toFixed(3)} °</output></div><div><small>当前法兰速度{current?.speedSource==='estimated'?' · 估算':''}</small><output aria-label="当前法兰速度">{current?.speedDegS==null?'—':current.speedDegS.toFixed(3)} °/s</output></div></div>
      <p aria-label="零点状态">{zero?`已设置 · ${new Date(zero.createdAt).toLocaleTimeString()} · 原始法兰 ${zero.positionDeg.toFixed(3)}° · boot ${zero.identity.boot} / coord ${zero.identity.reference}`:'未设置软件零点'}。刷新、重连、重启或坐标参考变化后失效；图表及记录保留原始坐标。</p>
      {!studio.flangeIdentity&&<p>设备缺少连续坐标身份；需要支持coord_boot/coord_ref的固件才能设置零点。</p>}
      <div className="device-debug-actions"><button className="button secondary" disabled={!available||zeroChangesLocked||!zeroCapsMatch||current?.positionDeg==null||!!motor&&['moving','holding','fault'].includes(motor.state)} onClick={captureZero}>当前位置设为零点</button>
      <button className="button secondary" disabled={!zero||zeroChangesLocked} onClick={()=>studio.clearFlangeZero()}>清除零点</button></div>
      <p>设零前请读取远程能力。设零/清除只改变软件参考；相对位移参数仍以当前姿态为起点。</p>
      <label className="remote-home-speed">回零法兰速度 °/s<input aria-label="回零法兰速度 °/s" type="number" step="any" disabled={zeroChangesLocked} value={Number.isFinite(homeSpeed)?homeSpeed:''} onChange={event=>setHomeSpeed(event.target.value===''?NaN:Number(event.target.value))}/></label>
      <button className="button danger" disabled={!available||zeroChangesLocked||!zero||current?.positionDeg==null||caps?.fields.home!=='1'} onClick={()=>void freezeHome()}>审核回到零点（运动）</button>
      {caps&&caps.fields.home!=='1'&&<p>回零当前不可用，请核对电机供电、反馈和固件能力；回零运动已禁用。</p>}
      {homeReview&&<><p>回零审核：当前原始法兰 {homeReview.current.toFixed(3)}°，零点目标 {homeReview.target.toFixed(3)}°，{homeReview.target<homeReview.current?'负方向':homeReview.target>homeReview.current?'正方向':'原位'}，预计位移 {Math.abs(homeReview.target-homeReview.current).toFixed(3)}°，速度 {outputRadiansToDisplayDegrees(homeReview.draft.speed,'flange').toFixed(3)}°/s。目标在准备期间保持固定；到位后走原POS停止/失能流程。</p><button className="button danger" disabled={!available||busy||studio.remoteMotionLocked||current?.positionDeg==null} onClick={submitHome}>显式执行回零一次</button><button className="button secondary" disabled={busy} onClick={()=>setHomeReview(null)}>取消回零审核</button></>}
    </>}
    </section>}
  </div></section>;
}
