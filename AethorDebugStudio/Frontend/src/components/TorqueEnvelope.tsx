/** @file ID7试验保护草稿、板端读回和冷却维护；此组件不会使能或启动运动。 */
import {useEffect,useRef,useState} from 'react';
import {remoteRequest} from '../remoteControl';
import {parseTorqueEnvelope,describeTorqueEnvelopeReason,torqueEnvelopeBlockers,torqueAllowanceSeconds,formatTorqueNm,type TorqueEnvelopePolicy} from '../torqueEnvelope';
import './torque-envelope.css';
import type {Studio} from '../useStudio';

/** Show board-confirmed limits separately from motion controls and require a new physical declaration per restore. */
export function TorqueEnvelope({available,locked,identity,onBusyChange,preparationAvailable,prepareAllowance}:{available:boolean;locked:boolean;identity:string;onBusyChange:(busy:boolean)=>void;preparationAvailable:boolean;prepareAllowance:Studio['prepareTorque']}) {
  const [policy,setPolicy]=useState<TorqueEnvelopePolicy|null>(null);
  const [cooled,setCooled]=useState(false);
  const [busy,setBusy]=useState(false);
  const [draftNm,setDraftNm]=useState('5.46');
  const [readAt,setReadAt]=useState('');
  const [message,setMessage]=useState('点击读取板端策略；旧固件不支持时保持关闭。');
  const version=useRef(0),inFlight=useRef(false),blocked=useRef(locked);
  blocked.current=locked;
  useEffect(()=>{version.current++;setPolicy(null);setReadAt('');setCooled(false);setMessage('请读取当前板端策略；未读取的数值仅为草稿。');return()=>{version.current++;};},[available,identity]);
  useEffect(()=>{if(locked)setCooled(false);},[locked]);
  const draftMilliNm=Math.round(Number(draftNm)*1000);
  const validDraft=/^\d+(?:\.\d{1,3})?$/.test(draftNm)&&Number.isFinite(draftMilliNm)&&draftMilliNm>=3500&&draftMilliNm<=6240;
  const allowance=validDraft?torqueAllowanceSeconds(draftMilliNm):NaN;
  /** 改草稿不发送指令，并撤销针对旧数值的冷却声明。 */
  function editDraft(value:string){setDraftNm(value);setCooled(false);}
  /** 用户明确授权一次失能采样和维护；不重试、不启动测试，取消后不续跑。 */
  async function prepareAndApply(){
    if(!available||locked||inFlight.current||!preparationAvailable||!cooled||!validDraft||!policy?.configurable||policy.blockers!==32)return;
    const currentVersion=version.current,reviewed={...policy},selected=draftMilliNm;
    inFlight.current=true;setBusy(true);onBusyChange(true);setCooled(false);
    try {
      const result=await prepareAllowance(reviewed,selected,()=>version.current===currentVersion&&!blocked.current,
        (detail,observed)=>{if(version.current===currentVersion){setMessage(detail);setPolicy(observed);}});
      if(version.current===currentVersion){setMessage(result.detail);setPolicy(result.policy);setReadAt(new Date().toLocaleTimeString());}
    }finally{inFlight.current=false;setBusy(false);onBusyChange(false);}
  }
  /** Read once or restore once after a fresh policy token; uncertain results are never retried. */
  async function execute(restore:boolean) {
    if(!available||locked||inFlight.current||restore&&(!cooled||!policy?.canReset||policy.configurable&&!validDraft))return;
    const currentVersion=version.current;
    const reviewedPolicy=policy;
    const selectedMilliNm=policy?.configurable?draftMilliNm:5460;
    inFlight.current=true;setBusy(true);onBusyChange(true);
    if(restore)setCooled(false);
    try {
      const receipt=await remoteRequest({operation:'torque'});
      if(version.current!==currentVersion||blocked.current)return;
      const fresh=parseTorqueEnvelope(receipt);
      setPolicy(fresh);setReadAt(new Date().toLocaleTimeString());
      if(!fresh){setMessage('固件未确认实验策略，预算恢复保持关闭。');return;}
      if(!restore){setCooled(false);setMessage(fresh.authorized?'试验配额可用；下方显示板端读回，草稿尚未下发。':'实验配额未恢复，当前仍按3.5 N·m退出。');return;}
      if(!reviewedPolicy||fresh.boot!==reviewedPolicy.boot||fresh.reference!==reviewedPolicy.reference||fresh.configurable!==reviewedPolicy.configurable){
        setMessage('设备身份或策略版本已变化，本次声明失效；未发送恢复。请重新读取并核对。');return;
      }
      if(!fresh.canReset){setMessage('板端暂不允许恢复：请按下方恢复步骤核对控制权、冷却和新鲜失能反馈；不会自动切模式或预检。');return;}
      const result=await remoteRequest({operation:'torque_reset',token:fresh.token,cooledConfirmed:true,
        ...(fresh.configurable?{peakMilliNm:selectedMilliNm}:{})});
      if(version.current!==currentVersion||blocked.current)return;
      setPolicy(null);
      if(result.result!=='reset'||result.fields.accepted!=='1'||result.fields.authorized!=='1'){
        setMessage('恢复未确认或被拒绝；未重发。请只读查询当前策略。');return;
      }
      setMessage('板端已接受维护，正在只读核对生效策略。');
      const verified=parseTorqueEnvelope(await remoteRequest({operation:'torque'}));
      if(version.current!==currentVersion||blocked.current)return;
      if(!verified||verified.boot!==fresh.boot||verified.reference!==fresh.reference||verified.configurable!==fresh.configurable||
        !verified.authorized||verified.peakMilliNm!==selectedMilliNm){
        setMessage('恢复已接受，但读回未确认或与所选值不符；未重发，请重新读取策略。');return;
      }
      setPolicy(verified);setReadAt(new Date().toLocaleTimeString());
      setMessage(`实验配额已恢复；未使能、未启动运动。读回确认 ${formatTorqueNm(verified.peakNm)} N·m；开始测试仍须通过板端预检。`);
    }catch(error){if(version.current===currentVersion){setPolicy(null);setMessage(String(error)+'；未重发预算恢复。');}}
    finally{inFlight.current=false;setBusy(false);onBusyChange(false);}
  }
  return <fieldset className="torque-envelope" aria-label="ID7实验转矩保护" disabled={!available||locked||busy}>
    <legend>试验转矩与温度保护 · ID7电机输出轴</legend>
    <p>只影响ID7单轴静载和POS带载的保护退出线，不是目标转矩。软件在反馈越线后退出，不能代替驱动限流。</p>
    <div className="torque-draft-grid"><div>
      <label className="torque-draft-label">试验转矩退出线 N·m<input aria-label="试验转矩退出线 N·m" inputMode="decimal" type="number" min="3.5" max="6.24" step="0.001" value={draftNm} disabled={!policy?.configurable} onChange={event=>editDraft(event.target.value)}/></label>
      <p>可调范围3.50～6.24 N·m（峰值7.8 N·m的80%）；草稿不会自动下发。</p>
      <div className="device-debug-actions torque-presets">{[[3500,'额定 · 3.50 N·m'],[5460,'70%峰值 · 5.46 N·m'],[6240,'80%峰值 · 6.24 N·m']].map(([value,label])=><button key={value} type="button" className="button secondary small" disabled={!policy?.configurable} onClick={()=>editDraft((Number(value)/1000).toFixed(2))}>{label}</button>)}</div>
      {policy?.configurable&&!validDraft&&<p role="alert">请输入3.50～6.24之间的转矩，最多三位小数。</p>}
      {policy&&!policy.configurable&&<p>当前固件仅支持固定5.46 N·m，不能下发可调边界；升级配套固件后开放编辑。</p>}
      {!policy&&<p>先读取策略，确认固件是否支持调节。</p>}
    </div><div className="torque-constraints" aria-label="联合保护条件">
      <div><span>持续转矩基准</span><strong>3.50 N·m</strong></div><div><span>任一路温度退出</span><strong>超过55℃</strong></div>
      <div><span>连续超额上限</span><strong>10秒</strong></div><div><span>冷却维护温度门槛</span><strong>两路均≤35℃</strong></div>
      <p>{validDraft&&draftMilliNm>3500?`按草稿恒定转矩计算，满配额约${allowance.toFixed(2)}秒耗尽，其他保护可能更早退出。`:'选择3.50 N·m时不放宽基线。'}这些是软件规则，原厂未提供该超额工况的持续时间保证。</p>
    </div></div>
    {policy&&<>
      <div className="torque-policy-state" aria-live="polite"><strong>上次读取生效边界：{policy.authorized?`${formatTorqueNm(policy.peakNm)} N·m 实验退出线`:'3.50 N·m 基线退出线'}</strong>
        <p>{policy.authorized?'实验配额可用':'实验配额未启用 / 已失效'} · 读取时间 {readAt}</p><p>{describeTorqueEnvelopeReason(policy)}</p></div>
      <p>上次读取：连续基准 {policy.continuousNm.toFixed(2)} N·m · 瞬时退出 {formatTorqueNm(policy.peakNm)} N·m · 超额窗口 {policy.windowSeconds} 秒 · {policy.authorized?`累计超额配额已用 ${(100*policy.used/policy.capacity).toFixed(1)}%`:'实验放宽关闭，保持原3.5 N·m退出线'}</p>
      {!policy.authorized&&<p>保护退出会撤销剩余配额并将账本记满；账本100%不等于实际运行满10秒，也不能单独证明热量已耗尽。</p>}
      <p>{policy.canReset?'板端上次读取允许恢复；点击恢复时仍会重新核对。':'板端当前不允许恢复：'}</p>
      {!policy.canReset&&<ul className="torque-blockers">{torqueEnvelopeBlockers(policy).map(reason=><li key={reason}>{reason}</li>)}</ul>}
    </>}
    <p>累计超额预算不会因刷新、重连或重配而补回；主控重启后关闭放宽。软件退出不等同于驱动限流。</p>
    <p>超出3.5 N·m越多，预算消耗越快；连续超额约10秒即退出。恢复配额只做维护，不启动电机。</p>
    <details className="torque-recovery-guide"><summary>怎样恢复配额，以及为什么不是每轮都有10秒</summary><ol>
      <li>先结束原会话，读取远程能力，确认控制权“未申请”、没有活动事务。原上位机故障通过板端审核时可用上方结束按钮；真实LCD占用或旧固件未提供审核时，按LCD原提示恢复并释放。</li>
      <li>机械臂和负载独立承托，电机充分冷却。板端需确认无故障、低速失能、两路温度均不高于35℃，并有5秒内标准反馈。</li>
      <li>模式2待机可能只更新位置。若唯一阻塞为标准反馈过期，选好退出线并确认承托与冷却后，点击“失能检查并应用试验配额”：只发一次ID7失能，读取新标准反馈并通过全部原维护门禁后立即应用。无需切换MIT模式，不使能、不启动测试；失败不自动重试。</li>
      <li>重新读取转矩保护策略；新固件可先设置退出线。板端允许后，勾选本次冷却与承托确认，再点击“冷却后恢复实验配额”。恢复后仍需单独启动并通过测试预检。</li>
    </ol><p>5秒是反馈有效期，不是冷却所需时间。刷新、重连USB、结束会话均不会补回配额。累计上限175.616 (N·m)²·s，按超过3.5²的转矩平方与时间累计；降到基准以下也不返还已用配额。</p></details>
    <label><input type="checkbox" checked={cooled} onChange={event=>setCooled(event.target.checked)}/>本次确认电机已充分冷却，机械臂与负载独立承托</label>
    {policy?.configurable&&policy.blockers===32&&<p role="note">待机标准反馈过期：下方“失能检查并应用”会撤去ID7保持力矩。通过后，单轴静载和POS带载均可使用所选试验退出线。STOP会取消尚未发送的维护失能及后续恢复；已发送的失能不会撤销，原回执等待最长4秒。{!preparationAvailable&&'当前网关版本不支持，请启动配套新版网关。'}</p>}
    <div className="device-debug-actions">
      <button type="button" className="button secondary" onClick={()=>void execute(false)}>读取转矩保护策略</button>
      <button type="button" className="button secondary" disabled={!cooled||!policy?.canReset||policy.configurable&&!validDraft} onClick={()=>void execute(true)}>冷却后恢复实验配额</button>
      {policy?.configurable&&policy.blockers===32&&<button type="button" className="button secondary" disabled={!cooled||!validDraft||!preparationAvailable} onClick={()=>void prepareAndApply()}>失能检查并应用试验配额</button>}
    </div>
    <p role="status">{message}</p>
  </fieldset>;
}
