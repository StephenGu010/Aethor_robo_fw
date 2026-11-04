/** @file 真机采集诊断、停止/失能及独立审核的远程单轴控制入口。 */
import { useEffect, useState } from 'react';
import { Radio, Square, Unplug } from 'lucide-react';
import { Badge, Note, PanelHeader } from './Common';
import { explainFeedback } from '../feedbackStatus';
import type { ControlReadback,DebugOperation } from '../types';
import type { Studio } from '../useStudio';
import '../device-debug.css';
import { RemoteControl } from './RemoteControl';
import type { RemoteOperation } from '../remoteControl';
import type { ControlCoordinate } from '../axisUnits';
const OPERATION_LABEL: Record<DebugOperation, string> = { diagnostics: '采集诊断', stop: '停止选中轴', 'stop-all': '停止全部轴', disable: '失能选中轴', 'torque-disable':'配额准备 · 单次失能' };
/** 在示波器与调试区解释采集阻塞；连接正常不等于反馈新鲜。 */
export function FeedbackHealth({ studio }: { studio: Studio }) {
  const motor = studio.gateway.motors.find((entry) => entry.id === studio.settings.motorId);
  // 定时器只触发刷新；新快照可能晚于上一次tick，年龄必须按本次实际评估时刻计算。
  const feedback = explainFeedback(motor, studio.gateway.connected, Date.now());
  return <div className={'feedback-health ' + (feedback.fresh ? 'fresh' : '')}>
    <Radio size={20}/><div><strong>{feedback.title}</strong><p>{feedback.detail}</p><small>USB {studio.gateway.connected ? '已连接' : '未连接'} · ID {studio.settings.motorId} · 板端状态 {motor?.state ?? '未知'} · {motor?.id===7&&motor.pmContract===1?'0x50 转子位置契约已确认':'寄存器单位契约未确认'}</small></div>
  </div>;
}
/** 显式执行真实设备事务，失能前需要承托声明，页面加载不发送动作。 */
export function DeviceDebug({ studio, initialOperation,displayCoordinate,onControlReadback }: { studio: Studio; initialOperation?:RemoteOperation;displayCoordinate?:ControlCoordinate;onControlReadback?:(value:ControlReadback|null)=>void }) {
  const [loadSupported, setLoadSupported] = useState(false);
  useEffect(() => setLoadSupported(false), [studio.settings.motorId, studio.gateway.port, studio.gateway.connected]);
  const available = studio.mode === 'live' && studio.gateway.connected && studio.gateway.debugAvailable === true;
  const latest = studio.debugReceipts[0];
  return <><section className="panel device-debug" data-workflow-step="check">
    <PanelHeader title="01 · 连接与检查" eyebrow="DEVICE CHECK"><Badge tone={available ? 'green' : 'amber'}>{available ? '停止 / 失能通道已接入' : studio.gateway.debugAvailable ? '设备未连接' : '等待支持调试的网关'}</Badge></PanelHeader>
    <div className="device-debug-body">
      <div className="device-debug-toolbar"><label>操作对象<select aria-label="调试电机" value={studio.settings.motorId} disabled={studio.debugPending.length > 0} onChange={(event) => studio.setSettings((old) => ({ ...old, motorId: Number(event.target.value) }))}>{[1,2,3,4,5,6,7].map((id) => <option key={id} value={id}>J{id} / ID {id}</option>)}</select></label><span>选中操作只作用于该轴；“停止全部轴”作用于ID 1–7。</span></div>
      <FeedbackHealth studio={studio}/>
      <div className="device-debug-actions">
        <button className="button secondary" disabled={!available || studio.debugPending.includes('diagnostics')} onClick={() => void studio.debug('diagnostics')}><Radio size={16}/>{studio.debugPending.includes('diagnostics') ? '正在读取诊断…' : '读取配置与CAN诊断'}</button>
        <button className="button danger" disabled={!available || studio.debugPending.includes('stop')} onClick={() => void studio.debug('stop')}><Square size={15}/>停止 ID {studio.settings.motorId}</button>
        <button className="button danger" disabled={!available || studio.debugPending.includes('stop-all')} onClick={() => void studio.debug('stop-all')}><Square size={15}/>停止全部轴</button>
      </div>
      <div className="disable-control"><label><input type="checkbox" checked={loadSupported} onChange={(event) => setLoadSupported(event.target.checked)} disabled={!available}/>已独立承托负载，允许撤去选中轴的保持力矩</label><button className="button danger" disabled={!available || !loadSupported || studio.debugPending.includes('disable')} onClick={() => { void studio.debug('disable', loadSupported); setLoadSupported(false); }}><Unplug size={15}/>失能 ID {studio.settings.motorId}</button></div>
      <Note warning>STOP按固件停止策略执行，反馈不足时可能转为失能；负载需要独立承托。软件STOP不能代替硬件急停，失能会撤去保持力矩。设备回执与新鲜反馈分别核对，超时不自动重试。下方远程适配必须先由固件确认能力；未部署适配固件或缺少新鲜标准反馈时保持关闭。</Note>
      {latest && <div className="debug-receipt" role="region" aria-label="最近设备回执"><div><strong>{OPERATION_LABEL[latest.operation]} · {latest.operation === 'stop-all' ? 'ID 1–7' : 'ID ' + latest.motorId}</strong><Badge tone={latest.result === 'completed' ? 'neutral' : 'amber'}>#{latest.requestId} · {latest.result}</Badge></div><p>{latest.detail}</p><small>接收回执：{latest.accepted ? '已收到' : '未确认'} · 新鲜失能反馈：{latest.feedbackConfirmed ? '已确认' : '未确认'}</small><details className="protocol-details"><summary>原始设备回执</summary><div className="debug-raw">{latest.lines.map((line, index) => <code key={index}>{line}</code>)}</div></details></div>}
    </div>
  </section><RemoteControl studio={studio} initialOperation={initialOperation} displayCoordinate={displayCoordinate} onControlReadback={onControlReadback}/></>;
}
