/** @file 工作台会话控制；演示运行与真实只读会话严格分离。 */
import { useEffect, useRef, useState } from 'react';
import { DEFAULT_SETTINGS } from './catalog';
import { createDemoRecording } from './demo';
import { gatewayApi } from './api';
import { captureTelemetry,projectTelemetry } from './feedbackStatus';
import {captureFlangeZero,flangeZeroCurrent,zeroIdentity,type FlangeZero} from './flangeZero';
import type {RemoteSessionRecovery} from './remoteSession';
import {remoteRequest} from './remoteControl';
import {prepareTorqueAllowance,type TorquePreparationOptions,type TorquePreparationResult} from './torquePreparation';
import {parseTorqueEnvelope,type TorqueEnvelopePolicy} from './torqueEnvelope';
import { canExecute, validateSettings } from './domain';
import type { DataMode, DebugOperation, DebugReceipt, Flow, GatewaySnapshot, Recording, Sample, Settings, WorkflowState } from './types';
const EMPTY_GATEWAY: GatewaySnapshot = { connected: false, port: null, status: '未连接', error: null, receivedAt: null, info: {}, motors: [], lines: [], readOnly: true };
/** 管理单一会话，所有模拟动作只更新内存，无网络发送。 */
export function useStudio() {
  const [mode, setMode] = useState<DataMode>('demo');
  const [settings, setSettings] = useState<Settings>(DEFAULT_SETTINGS);
  const [savedRecording, setRecording] = useState<Recording>(() => createDemoRecording({ ...DEFAULT_SETTINGS, durationS: 8 }, 'static'));
  const [liveRecording, setLiveRecording] = useState<Recording>({ schema:'aethor-studio-recording/v1', name:'只读会话', source:'device', coordinate:'native', motorId:7, createdAt:new Date().toISOString(), samples:[], events:[] });
  const recording = mode === 'live' ? liveRecording : savedRecording;
  const [history, setHistory] = useState<Recording[]>([]);
  const [gateway, setGateway] = useState<GatewaySnapshot>(EMPTY_GATEWAY);
  const [workflow, setWorkflow] = useState<WorkflowState>({ stage: 'idle', startedAt: 0, frozen: null });
  const [flow, setFlow] = useState<Flow>('static');
  const [now, setNow] = useState(Date.now());
  const [notice, setNotice] = useState('');
  const [storedFlangeZero,setStoredFlangeZero]=useState<FlangeZero|null>(null);
  const [remoteControlLocked,setRemoteMotionLocked]=useState(false);
  const [torqueMaintenanceUncertain,setTorqueMaintenanceUncertain]=useState(false);
  const torqueUncertainIdentity=useRef<{boot:number;reference:number}|null>(null);
  const remoteMotionLocked=remoteControlLocked||torqueMaintenanceUncertain;
  const remoteMotionLock=useRef(false);
  const torqueMaintenanceOwner=useRef<symbol|null>(null);
  const [torqueMaintenanceBusy,setTorqueMaintenanceBusy]=useState(false);
  const remoteSessionRecovery=useRef<RemoteSessionRecovery|null>(null);
  const connectionGeneration=useRef(0);
  const connectionKey=`${mode}:${gateway.connected}:${gateway.port}:${settings.motorId}`;
  const lastConnectionKey=useRef(connectionKey);
  if(lastConnectionKey.current!==connectionKey){lastConnectionKey.current=connectionKey;connectionGeneration.current++;}
  const selectedMotor=gateway.motors.find(entry=>entry.id===settings.motorId);
  const flangeIdentity=mode==='live'&&gateway.connected?zeroIdentity(selectedMotor,`${gateway.port}:${connectionGeneration.current}`):null;
  const flangeZero=flangeZeroCurrent(storedFlangeZero,flangeIdentity)?storedFlangeZero:null;
  useEffect(()=>{if(!flangeZero)setStoredFlangeZero(null);},[flangeZero]);
  const activeDebug = useRef(new Set<DebugOperation>());
  const controlIntentVersion = useRef(0);
  const [debugPending, setDebugPending] = useState<DebugOperation[]>([]);
  const [debugReceipts, setDebugReceipts] = useState<DebugReceipt[]>([]);
  const [replayTime, setReplayTime] = useState<number | null>(null);
  const [playing, setPlaying] = useState(false);
  const running = workflow.stage === 'running';
  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), running ? 100 : 250);
    return () => window.clearInterval(timer);
  }, [running]);
  useEffect(() => {
    if (mode !== 'live') return;
    let active = true;
    let pending = false;
    let firstTimestamp: number | null = null;
    let lastReceivedAt = '';
    let previousPort: string | null = null;
    let connectionGap = false;
    setLiveRecording({ schema:'aethor-studio-recording/v1', name:'只读会话 · ID'+settings.motorId, source:'device', coordinate:'native', motorId:settings.motorId, createdAt:new Date().toISOString(), samples:[], events:[] });
    /** 轮询网关快照；故障时保留旧值并标记通信中断。 */
    async function poll() {
      if (pending) return;
      pending = true;
      try {
        const state = await gatewayApi.state();
        if (active) {
          setGateway(state);
          if (state.debugReceipts?.length) setDebugReceipts(state.debugReceipts);
          const motor = state.motors.find((entry) => entry.id === settings.motorId);
          if (!state.connected || (previousPort !== null && state.port !== previousPort)) connectionGap = true;
          if (state.connected) previousPort = state.port;
          if (motor?.receivedAt && motor.receivedAt !== lastReceivedAt && Number.isFinite(Date.parse(motor.receivedAt))) {
            lastReceivedAt = motor.receivedAt;
            const timestamp = Date.parse(motor.receivedAt);
            firstTimestamp ??= timestamp;
            const sample: Sample = captureTelemetry(motor,state.connected,Date.now(),Math.max(0,timestamp-firstTimestamp));
            const gap = connectionGap;
            connectionGap = !state.connected;
            setLiveRecording((old) => sample.timeMs < (old.samples.at(-1)?.timeMs ?? 0) ? old : { ...old, samples:[...old.samples,sample].slice(-3000),
              events:gap ? [...old.events,{timeMs:sample.timeMs,level:'warning',title:'连接中断或重连',detail:'跨会话不连接遥测曲线，不沿用估算状态。',gap:true}].slice(-3000) as Recording['events'] : old.events });
          }
        }
      }
      catch { connectionGap = true; if (active) setGateway((old) => ({ ...old, connected: false, status: '本地网关不可用', error: '请运行项目根目录的启动脚本' })); }
      finally { pending = false; }
    }
    void poll();
    const timer = window.setInterval(() => void poll(), 250);
    return () => { active = false; window.clearInterval(timer); };
  }, [mode, settings.motorId]);
  const duration = recording.samples.at(-1)?.timeMs ?? 0;
  const elapsed = running ? Math.min(duration, Math.max(0, now - workflow.startedAt)) : duration;
  useEffect(() => {
    if (!running || elapsed < duration) return;
    setWorkflow((old) => ({ ...old, stage: 'completed' }));
    setHistory((old) => [recording, ...old].slice(0, 20));
    setNotice('演示完成，已保存本次合成记录；未发送电机命令。');
  }, [elapsed, duration, running, recording]);
  useEffect(() => {
    if (!playing || mode !== 'replay') return;
    const timer = window.setInterval(() => setReplayTime((old) => {
      const next = (old ?? 0) + 100;
      if (next >= duration) { setPlaying(false); return duration; }
      return next;
    }), 100);
    return () => window.clearInterval(timer);
  }, [playing, mode, duration]);
  /** 更换数据源不能继承活动演示；真实连接不因页面切换而自动关闭。 */
  function changeMode(next: DataMode) {
    if (running) { setNotice('请先停止当前演示。'); return; }
    if (gateway.connected && next !== 'live') { setNotice('请先断开只读连接，再切换数据来源。'); return; }
    if (next === 'replay' && !recording.samples.length) { setNotice('请先导入记录。'); return; }
    setMode(next); setPlaying(false); setReplayTime(null);
    setWorkflow({ stage: 'idle', startedAt: 0, frozen: null });
  }
  useEffect(() => {
    if(mode==='demo'&&!running&&workflow.stage!=='review'&&savedRecording.motorId!==settings.motorId)setRecording(createDemoRecording(settings));
  },[mode,running,workflow.stage,settings,savedRecording.motorId]);
  /** 创建参数审核；只允许有效的离线演示草稿。 */
  function review(nextFlow: Flow) {
    const gate = canExecute(mode, running, validateSettings(nextFlow, settings));
    if (!gate.allowed) { setNotice(gate.reason); return; }
    setFlow(nextFlow);
    setWorkflow({ stage: 'review', startedAt: 0, frozen: structuredClone(settings) });
  }
  /** 启动冻结参数对应的模拟记录，不调用任何网关写方法。 */
  function start() {
    if (mode !== 'demo' || workflow.stage !== 'review' || !workflow.frozen) return;
    const next = createDemoRecording(workflow.frozen, flow);
    setRecording(next); setReplayTime(null);
    setWorkflow((old) => ({ ...old, stage: 'running', startedAt: Date.now() }));
  }
  /** 即时结束演示并截断其证据，不能冒充真实失能确认。 */
  function stop() {
    if (!running || mode !== 'demo') return;
    const stopped: Recording = { ...recording, name: recording.name + ' · 已停止', samples: recording.samples.filter((sample) => sample.timeMs <= elapsed), events: [...recording.events.filter((event) => event.timeMs < elapsed), { timeMs: elapsed, level: 'warning', title: '操作员停止演示', detail: '仅停止合成流程；未发送硬件STOP或失能命令' }] };
    setRecording(stopped); setHistory((old) => [stopped, ...old].slice(0, 20));
    setWorkflow((old) => ({ ...old, stage: 'stopped' })); setNotice('演示已停止。');
  }
  /** 导入或选择记录后进入回放，不连接或更改设备。 */
  function loadRecording(next: Recording) {
    if (running) { setNotice('请先停止演示再导入记录。'); return; }
    if (gateway.connected) { setNotice('请先断开只读连接，再导入历史记录。'); return; }
    setSettings((old)=>({...old,motorId:next.motorId}));
    setRecording(next); setMode('replay'); setReplayTime(0); setPlaying(false);
    setWorkflow({ stage: 'idle', startedAt: 0, frozen: null }); setNotice('已载入记录，进入回放模式。');
  }
  /** 显式连接只读网关，清除演示数据的可见性。 */
  async function connect(port: string) {
    if (running) throw new Error('请先停止演示');
    const state = await gatewayApi.connect(port);
    setGateway(state); changeMode('live');
  }
  /** 释放只读串口并保留最后反馈；不请求电机停止。 */
  async function disconnect() {
    controlIntentVersion.current++;
    try { setGateway(await gatewayApi.disconnect()); }
    catch (error) { setNotice(error instanceof Error ? error.message : '断开失败'); }
  }
  /** 在两个远程页面共享执行/未知事务锁，切页不能解除未确认动作。 */
  function lockRemoteMotion(locked:boolean) {
    remoteMotionLock.current=locked;setRemoteMotionLocked(locked);
  }
  /** 捕获新鲜法兰姿态，仅写当前会话软件参考，不发出请求。 */
  function setCurrentFlangeZero() {
    if(remoteMotionLock.current||torqueUncertainIdentity.current||torqueMaintenanceOwner.current||!flangeIdentity||!selectedMotor||['moving','holding','fault'].includes(selectedMotor.state))return false;
    const current=projectTelemetry(captureTelemetry(selectedMotor,gateway.connected,Date.now(),0),'flange','native',7);
    if(current.positionDeg===null)return false;
    const zero=captureFlangeZero(current.positionDeg,flangeIdentity,Date.now());
    if(!zero)return false;
    setStoredFlangeZero(zero);return true;
  }
  /** 只清除软件参考；活动或未知事务期间保留其冻结目标。 */
  function clearFlangeZero() {
    if(remoteMotionLock.current||torqueUncertainIdentity.current||torqueMaintenanceOwner.current)return;
    setStoredFlangeZero(null);
  }
  /** 固定操作显式发送一次；STOP可在诊断等待时排队，未知结果不重试。 */
  async function debug(operation: DebugOperation, loadSupported = false) {
    if(operation==='torque-disable'){setNotice('维护失能只能由转矩准备流程发出');return;}
    if (operation === 'stop' || operation === 'stop-all' || operation === 'disable') controlIntentVersion.current++;
    if(torqueMaintenanceOwner.current&&operation!=='stop'&&operation!=='stop-all'){setNotice('配额维护正在进行，本次未发送；可用STOP取消后续步骤');return;}
    if (mode !== 'live' || !gateway.connected || !gateway.debugAvailable) { setNotice('请连接支持调试事务的本地网关'); return; }
    if (activeDebug.current.has(operation)) return;
    activeDebug.current.add(operation); setDebugPending([...activeDebug.current]);
    try {
      const receipt = await gatewayApi.debug(operation, settings.motorId, loadSupported);
      setDebugReceipts((old) => [receipt, ...old.filter((entry) => entry.requestId !== receipt.requestId)].slice(0, 10));
      setNotice(receipt.detail);
    } catch (error) {
      setNotice((error instanceof Error ? error.message : '设备事务响应中断') + '；未自动重试。若请求已发送，设备结果可能未知，请查看原始响应。');
    } finally {
      activeDebug.current.delete(operation); setDebugPending([...activeDebug.current]);
    }
  }
  /** 持有跨页面维护锁；STOP/断连撤销后续步骤，未知失能保留动作锁。 */
  async function prepareTorque(reviewed:TorqueEnvelopePolicy,peakMilliNm:number,isPageCurrent:()=>boolean,
    observe:TorquePreparationOptions['observe']):Promise<TorquePreparationResult> {
    if(mode!=='live'||!gateway.connected||settings.motorId!==7||gateway.torquePreparationAvailable!==true||
      torqueMaintenanceOwner.current||torqueUncertainIdentity.current||remoteMotionLock.current||activeDebug.current.size)
      return {result:'blocked',detail:'当前网关未开放维护准备，或设备事务尚未结束。',policy:null,uncertainDisable:false,uncertainMaintenance:false};
    const owner=Symbol('torque-maintenance'),connection=connectionGeneration.current,intent=controlIntentVersion.current;
    torqueMaintenanceOwner.current=owner;setTorqueMaintenanceBusy(true);
    try {
      const result=await prepareTorqueAllowance(reviewed,peakMilliNm,{
        request:remoteRequest,observe,
        isCurrent:()=>torqueMaintenanceOwner.current===owner&&connection===connectionGeneration.current&&
          intent===controlIntentVersion.current&&isPageCurrent(),
        disable:async()=>{
          const receipt=await gatewayApi.debug('torque-disable',7,true);
          setDebugReceipts(old=>[receipt,...old.filter(entry=>entry.requestId!==receipt.requestId)].slice(0,10));
          return receipt;
        },
      });
      if(result.uncertainMaintenance){torqueUncertainIdentity.current={boot:reviewed.boot,reference:reviewed.reference};setTorqueMaintenanceUncertain(true);}
      setNotice(result.detail);return result;
    }finally{
      if(torqueMaintenanceOwner.current===owner){torqueMaintenanceOwner.current=null;setTorqueMaintenanceBusy(false);}
    }
  }
  /** 只读恢复独立维护锁；旧运动终态不能解除它，必须重新取得同身份的完整维护门禁。 */
  async function recoverTorquePreparation(){
    const expected=torqueUncertainIdentity.current;
    if(!expected||torqueMaintenanceOwner.current||!gateway.connected||mode!=='live'||settings.motorId!==7||activeDebug.current.size)return;
    const owner=Symbol('torque-recovery'),connection=connectionGeneration.current,intent=controlIntentVersion.current;
    torqueMaintenanceOwner.current=owner;setTorqueMaintenanceBusy(true);
    const current=()=>torqueUncertainIdentity.current===expected&&connection===connectionGeneration.current&&intent===controlIntentVersion.current;
    try{
      const caps=await remoteRequest({operation:'caps'});
      if(!current()||caps.result!=='capability'||caps.fields.boot!==String(expected.boot)||caps.fields.coord!==String(expected.reference)||
        caps.fields.owner!=='0'||caps.fields.authority!=='0'||caps.fields.active_id!=='0'){
        setNotice('维护身份或空闲控制权未确认，维护锁保留。');return;
      }
      const policy=parseTorqueEnvelope(await remoteRequest({operation:'torque'}));
      if(!current()||!policy?.configurable||policy.boot!==expected.boot||policy.reference!==expected.reference||!policy.canReset||policy.blockers!==0){
        setNotice('当前仍缺少新鲜失能、冷却或空闲证明；维护锁保留，未发送任何动作。');return;
      }
      torqueUncertainIdentity.current=null;setTorqueMaintenanceUncertain(false);
      setNotice(`只读核验完成：当前生效退出线 ${(policy.authorized?policy.peakNm:3.5).toFixed(3)} N·m；维护锁已解除，未恢复配额或启动测试。`);
    }catch(error){setNotice(String(error)+'；维护锁保留。');}
    finally{if(torqueMaintenanceOwner.current===owner){torqueMaintenanceOwner.current=null;setTorqueMaintenanceBusy(false);}}
  }
  const visibleTime = mode === 'replay' ? replayTime ?? duration : elapsed;
  const visibleSamples = recording.samples.filter((sample) => sample.timeMs <= visibleTime);
  const samples: Sample[] = visibleSamples;
  return { mode, changeMode, settings, setSettings, recording, samples, history, gateway, workflow, setWorkflow, running, now, notice, setNotice, review, start, stop, loadRecording, connect, disconnect, debug, debugPending, debugReceipts, controlIntentVersion, flangeZero,flangeIdentity,setCurrentFlangeZero,clearFlangeZero,remoteMotionLocked,lockRemoteMotion,remoteSessionRecovery,torqueMaintenanceBusy,torqueMaintenanceUncertain,recoverTorquePreparation,prepareTorque,elapsed, duration, visibleTime, replayTime, setReplayTime, playing, setPlaying };
}
export type Studio = ReturnType<typeof useStudio>;
