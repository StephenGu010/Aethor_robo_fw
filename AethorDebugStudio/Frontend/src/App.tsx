/** @file 工作台外壳与导航；顶部始终标识数据来源与硬件操作边界。 */
import { useCallback, useEffect, useState } from 'react';
import { Activity, BookOpen, ArrowDownToLine, ChevronDown, CircleHelp, FileClock, FlaskConical, LayoutDashboard, LockKeyhole, Menu, Monitor, PlugZap, Radio, Settings2, ShieldCheck, SlidersHorizontal, Square, Unplug, X, Layers3 } from 'lucide-react';
import type { LucideIcon } from 'lucide-react';
import { PAGES } from './catalog';
import { Badge } from './components/Common';
import { Overview } from './pages/Overview';
import { Workflow } from './pages/Workflow';
import { StaticWorkspace } from './pages/StaticWorkspace';
import { Scope, Records } from './pages/Analysis';
import { Maintenance, Lab } from './pages/Maintenance';
import { gatewayApi } from './api';
import { useStudio } from './useStudio';
import { DeviceDebug } from './components/DeviceDebug';
import type { ControlReadback, DataMode, PageId } from './types';
import {GettingStarted,UserManual} from './pages/UserManual';
import {explainFeedback} from './feedbackStatus';
import './styles.css';
import './usability.css';
const ICONS: Record<PageId,LucideIcon> = {overview:LayoutDashboard,guide:BookOpen,manual:SlidersHorizontal,static:LockKeyhole,pos:ArrowDownToLine,seven:Layers3,scope:Activity,maintenance:Settings2,records:FileClock,lab:FlaskConical};
/** 提供分组导航、全局来源选择和显式只读连接对话框。 */
export default function App() {
  const studio=useStudio();
  const [page,setPage]=useState<PageId>('overview');
  const [sidebarOpen,setSidebarOpen]=useState(false);
  const [connecting,setConnecting]=useState(false);
  const [connectionBusy,setConnectionBusy]=useState(false);
  const [ports,setPorts]=useState<string[]>([]);
  const [port,setPort]=useState('');
  const [connectionError,setConnectionError]=useState('');
  const [help,setHelp]=useState(false);
  const [controlReadback,setControlReadback]=useState<ControlReadback|null>(null);
  /** 稳定接收显式能力查询的快照；回传不改变控制权或发起设备请求。 */
  const reportControlReadback=useCallback((value:ControlReadback|null)=>setControlReadback(value),[]);
  useEffect(()=>setControlReadback(null),[studio.gateway.port,studio.gateway.connected,studio.settings.motorId,studio.mode]);
  const active=PAGES.find((entry)=>entry.id===page)!;
  const selectedMotor=studio.gateway.motors.find(entry=>entry.id===studio.settings.motorId);
  const feedback=explainFeedback(selectedMotor,studio.gateway.connected,Date.now());
  const matchingControl=controlReadback?.connectionKey===`${studio.gateway.port}:${studio.settings.motorId}`?controlReadback:null;
  const controlExpired=!!matchingControl&&Date.now()-matchingControl.readAt>=30000;
  /** 导航仅切换视图，绝不启动动作或抢夺控制权。 */
  function navigate(next:PageId){setPage(next);setSidebarOpen(false);window.scrollTo({top:0,behavior:'instant'});}
  useEffect(()=>{
    /** Escape只关闭非执行类对话框，不向硬件发送STOP。 */
    function onKey(event:KeyboardEvent){if(event.key==='Escape'){setConnecting(false);setHelp(false);setSidebarOpen(false);if(studio.workflow.stage==='review')studio.setWorkflow({stage:'idle',startedAt:0,frozen:null});}}
    window.addEventListener('keydown',onKey);return()=>window.removeEventListener('keydown',onKey);
  },[studio.workflow.stage,studio.setWorkflow]);
  useEffect(()=>{
    if(!connecting&&!help&&studio.workflow.stage!=='review')return;
    const previous=document.activeElement as HTMLElement|null;
    const modal=document.querySelector<HTMLElement>('.modal');
    /** 每次按键重新枚举可用控件，兼容连接状态及端口列表更新。 */
    const focusableControls=()=>Array.from(modal?.querySelectorAll<HTMLElement>('button:not(:disabled),select:not(:disabled),input:not(:disabled),a[href]')??[]);
    if(modal&&!modal.contains(document.activeElement))focusableControls()[0]?.focus();
    /** 在当前模态窗口内循环键盘焦点，关闭后恢复原入口。 */
    function trapFocus(event:KeyboardEvent){
      if(event.key!=='Tab')return;
      const focusable=focusableControls();
      const first=focusable[0],last=focusable.at(-1);
      if(!first||!last)return;
      if(!modal?.contains(document.activeElement)){event.preventDefault();(event.shiftKey?last:first).focus();}
      else if(event.shiftKey&&document.activeElement===first){event.preventDefault();last.focus();}
      else if(!event.shiftKey&&document.activeElement===last){event.preventDefault();first.focus();}
    }
    document.addEventListener('keydown',trapFocus);
    return()=>{document.removeEventListener('keydown',trapFocus);previous?.focus();};
  },[connecting,help,studio.workflow.stage]);
  /** 枚举可选端口，不打开设备。 */
  async function refreshPorts(){setConnectionError('');try{const found=await gatewayApi.ports();setPorts(found);setPort((old)=>found.includes(old)?old:found[0]??'');}catch{setConnectionError('本地网关未启动。请运行 Start-Studio.ps1 后再枚举端口。');}}
  /** 用户确认后才打开真实只读串口。 */
  async function connect(){if(!port)return;setConnectionBusy(true);try{await studio.connect(port);setConnecting(false);}catch(error){setConnectionError(error instanceof Error?error.message:'连接失败');}finally{setConnectionBusy(false);}}
  return <div className="app-shell">
    {sidebarOpen&&<button className="sidebar-scrim" aria-label="关闭导航" onClick={()=>setSidebarOpen(false)}/>}
    <aside className={'sidebar '+(sidebarOpen?'open':'')}><a href="#" className="brand" onClick={(event)=>{event.preventDefault();navigate('overview');}}><span className="brand-symbol">A<span/></span><div><strong>AETHOR<span>®</span></strong><small>DEBUG STUDIO</small></div></a><div className="workspace-label"><span className="workspace-dot"/>机械臂调试工作台<span>01</span></div>
      <nav aria-label="主导航">{['开始使用','测试与参数','观察与维护'].map(group=><div className="nav-group" key={group}><span className="nav-category">{group}</span>{PAGES.filter(entry=>entry.group===group).map(entry=>{const Icon=ICONS[entry.id];return <button key={entry.id} aria-label={entry.label} className={'nav-item '+(page===entry.id?'active':'')} aria-current={page===entry.id?'page':undefined} onClick={()=>navigate(entry.id)}><Icon size={18}/><span>{entry.label}</span>{page===entry.id&&<span className="nav-indicator"/>}</button>;})}</div>)}</nav>
      <div className="sidebar-bottom"><div className="local-environment"><Monitor size={18}/><div><strong>本地工作空间</strong><span>Windows · USB CDC</span></div><span className="local-dot"/></div><button className="sidebar-help" onClick={()=>setHelp(true)}><CircleHelp size={16}/>使用说明与版本<span>↗</span></button><div className="sidebar-version"><span>DEBUG STUDIO</span><span>v0.2.0</span></div></div>
    </aside>
    <div className="main-shell"><header className="topbar"><button className="icon-button mobile-menu" aria-label="打开导航" onClick={()=>setSidebarOpen(true)}><Menu size={21}/></button><div className="breadcrumbs"><span>工作空间</span><span>/</span><strong>{active.label}</strong></div><div className="topbar-actions"><div className="source-selector"><Radio size={14}/><select aria-label="数据来源" value={studio.mode} disabled={studio.running} onChange={(event)=>studio.changeMode(event.target.value as DataMode)}><option value="demo">离线演示</option><option value="live">真实设备 · 诊断调试</option><option value="replay">记录回放</option></select><ChevronDown size={13}/></div><span className="topbar-divider"/><button className="button secondary connection-button" disabled={studio.running} onClick={()=>{setConnecting(true);void refreshPorts();}}><PlugZap size={16}/><span>{studio.gateway.connected?studio.gateway.port:'连接设备'}</span></button><button className="stop-button" disabled={studio.mode==='live'?(!studio.gateway.connected||!studio.gateway.debugAvailable||studio.debugPending.includes('stop-all')):!studio.running} title={studio.mode==='live'?'向ID 1–7发送一次软件STOP，等待设备回执；不能代替硬件急停':'仅停止正在运行的离线演示'} onClick={()=>studio.mode==='live'?void studio.debug('stop-all'):studio.stop()}><Square size={13}/><span>{studio.mode==='live'?'STOP 全部轴':'停止演示'}</span></button></div></header><div className="global-device-status" role="region" aria-label="当前连接与控制状态"><span>选中轴 <strong>ID {studio.settings.motorId}</strong></span><span>{studio.mode==='live'?(studio.gateway.connected?'USB已连接':'USB未连接'):'当前为'+(studio.mode==='demo'?'离线演示':'记录回放')}</span>{studio.mode==='live'&&<><span>标准反馈：<strong>{feedback.fresh?'新鲜':'缺失或过期'}</strong></span><span>{matchingControl?(controlExpired?'控制权缓存已过期':'控制权：'+matchingControl.label)+' · 读取于 '+new Date(matchingControl.readAt).toLocaleTimeString():'控制权：待读取设备能力'}</span>{studio.remoteMotionLocked&&<span>事务锁定，请查原结果</span>}<span>反馈时间：{selectedMotor?.receivedAt?new Date(selectedMotor.receivedAt).toLocaleTimeString():'尚未取得'}</span></>}</div>
      <main><div className="page-heading"><div><div className="page-title-line"><h1>{active.title}</h1><span className="page-count">{String(PAGES.findIndex((entry)=>entry.id===page)+1).padStart(2,'0')}</span></div><p>{active.subtitle}</p></div><div className="page-context"><span className={'status-light '+(studio.mode==='live'&&studio.gateway.connected?'connected':'')}/><span>{studio.mode==='demo'?'演示数据 · 未连接设备':studio.mode==='replay'?'历史数据 · 不执行动作':studio.gateway.connected?'采集连接 · '+studio.gateway.port:'真实模式 · 未连接设备'}</span></div></div>
      {studio.mode==='live'&&<div className="live-banner"><ShieldCheck size={17}/><span>单轴静载、POS带载与手动调试均可设置参数后启动。设备须确认固件能力，并通过板端预检与反馈校验。</span><button className="button secondary small" onClick={()=>navigate('manual')}>打开设备控制</button>{studio.gateway.error&&<strong>{studio.gateway.error}</strong>}</div>}
      {page==='overview'&&<GettingStarted navigate={navigate} connect={()=>{setConnecting(true);void refreshPorts();}}/>}
      {page==='guide'?<UserManual navigate={navigate}/>:studio.mode==='live'&&(page==='manual'||page==='static'||page==='pos')?<DeviceDebug key={page} studio={studio} onControlReadback={reportControlReadback} initialOperation={page==='static'?'hold':page==='pos'?'pos_load':'configure'} displayCoordinate={page==='static'||page==='pos'?'flange':'output'}/>:page==='overview'?<Overview studio={studio} navigate={navigate}/>:page==='static'?<StaticWorkspace studio={studio}/>:page==='manual'||page==='pos'||page==='seven'?<Workflow key={page} studio={studio} flow={page}/>:page==='scope'?<Scope studio={studio}/>:page==='records'?<Records studio={studio}/>:page==='maintenance'?<Maintenance studio={studio}/>:<Lab navigate={navigate}/>}
      <footer className="page-footer"><span><ShieldCheck size={13}/>界面状态、设备反馈与实机资格分别记录</span><span>AETHOR / ENGINEERING WORKSPACE</span></footer></main>
    </div>
    {studio.notice&&<div className="toast" role="status"><span>{studio.notice}</span><button className="icon-button" aria-label="关闭提示" onClick={()=>studio.setNotice('')}><X size={16}/></button></div>}
    {connecting&&<div className="modal-backdrop"><section className="modal" role="dialog" aria-modal="true" aria-labelledby="connection-title"><button className="modal-close icon-button" aria-label="关闭连接窗口" onClick={()=>setConnecting(false)}><X size={18}/></button><div className="modal-icon"><PlugZap size={27}/></div><span className="eyebrow">LOCAL CONNECTION</span><h2 id="connection-title">连接真实设备</h2><p>Windows通信服务独占USB串口，连接后默认只发送固定查询，不使能、不运动、不写参数。停止/失能须单独点击调试入口。</p>{studio.gateway.connected?<><Badge tone="green">已连接 {studio.gateway.port}</Badge><div className="modal-actions"><button className="button secondary" onClick={()=>setConnecting(false)}>返回工作台</button><button className="button danger" onClick={()=>void studio.disconnect().then(()=>setConnecting(false))}><Unplug size={15}/>断开设备连接</button></div></>:<><label htmlFor="port-select">USB CDC 串口</label><div className="port-picker"><select autoFocus id="port-select" value={port} onChange={(event)=>setPort(event.target.value)}>{!ports.length&&<option value="">没有可用端口</option>}{ports.map((item)=><option key={item} value={item}>{item}</option>)}</select><button className="button secondary" onClick={()=>void refreshPorts()}>重新枚举</button></div><small className="micro">115200 · DTR/RTS关闭 · 不自动重连</small>{connectionError&&<div className="validation-error" role="alert">{connectionError}</div>}<div className="modal-actions"><button className="button secondary" onClick={()=>setConnecting(false)}>取消</button><button className="button primary" disabled={!port||connectionBusy} onClick={()=>void connect()}>{connectionBusy?'正在连接…':'连接并只读查询'}<PlugZap size={15}/></button></div></>}</section></div>}
    {help&&<div className="modal-backdrop"><section className="modal" role="dialog" aria-modal="true" aria-labelledby="help-title"><button className="modal-close icon-button" aria-label="关闭说明" onClick={()=>setHelp(false)}><X size={18}/></button><span className="eyebrow">ABOUT THIS WORKSPACE</span><h2 id="help-title">调试工作台 v0.2.0</h2><p>离线演示：调整参数、审核并模拟执行；STOP只停止演示。</p><p>记录回放：在“记录与诊断”导入JSON，再到示波器播放、暂停或拖动时间轴。</p><p>真实设备：连接后默认只读。已接入ID7静载、POS带载、RAM参数和独立回零；主启动按钮会执行真实流程，须由当前设备确认能力并通过保护检查。七轴远程动作、PMAX写入和掉电保存尚未接入。</p><p>所有记录保存在当前会话中，需要保留时请导出。关节示意图不是实际几何模型。</p><button autoFocus className="button primary full" onClick={()=>{setHelp(false);navigate('guide');}}>打开完整新手手册</button><button className="button secondary full" onClick={()=>setHelp(false)}>了解，返回工作台</button></section></div>}
  </div>;
}


