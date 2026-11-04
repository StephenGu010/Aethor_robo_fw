/** @file 驱动维护与能力登记；未接入的写操作始终禁用。 */
import { useState } from 'react';
import { Check, ChevronRight, FlaskConical, LockKeyhole, Search, ShieldCheck, SlidersHorizontal } from 'lucide-react';
import { Badge, Note, PanelHeader, Value } from '../components/Common';
import { FEATURES, STATUS_LABEL } from '../catalog';
import { ReleaseBaseline } from '../components/ReleaseBaseline';
import type { PageId } from '../types';
import type { Studio } from '../useStudio';
/** 只展示主控公开量程；不将缓存误称为独立驱动寄存器读回。 */
export function Maintenance({ studio }: { studio: Studio }) {
  const motor=studio.gateway.motors.find((entry)=>entry.id===studio.settings.motorId);
  const live=studio.mode==='live';
  return <>
    <ReleaseBaseline/>
    <Note warning>驱动维护会改变设备配置。本页已接入只读快照，驱动RAM量程写入、保存和重新发现尚未开放；静载RAM参数在“手动调试”中下发。以下维护步骤展示接入位置。</Note>
    <div className="maintenance-grid"><section className="panel"><PanelHeader title="量程与读回" eyebrow="DRIVER CONFIGURATION"><Badge tone="amber">写入未接入</Badge></PanelHeader><div className="maintenance-body"><div className="kv"><span>选中电机</span><strong>J{studio.settings.motorId} / ID {studio.settings.motorId}</strong></div><div className="config-readback"><span>主控公开的位置上限</span><strong><Value value={live?motor?.pmaxDeg:null} unit="°"/></strong><small>固件原始坐标；不是本轮直接读取驱动寄存器的凭据</small></div><div className="value-stages"><div><span>编辑 / 目标值</span><strong>未设置</strong></div><ChevronRight size={16}/><div><span>已下发值</span><strong>未下发</strong></div><ChevronRight size={16}/><div><span>实际读回</span><strong>{live&&motor?.pmaxDeg!=null?'见上方公开值':'尚未取得'}</strong></div></div><div className="kv"><span>实际控制模式</span><strong>当前接口未提供</strong></div><div className="kv"><span>本上位机保存事务</span><Badge>未执行</Badge></div><div className="kv"><span>本次会话断电验证</span><Badge>未取得</Badge></div><button className="button secondary full" disabled><LockKeyhole size={15}/>RAM量程准备 · 待接入</button></div></section>
    <section className="panel"><PanelHeader title="维护步骤" eyebrow="MAINTENANCE FLOW"/><div className="maintenance-steps">{['核对身份、控制权与新鲜失能反馈','保存当前参数快照与位置参考','仅写入本次授权RAM参数','驱动读回、主控重新发现','位置连续性检查','如需持久化：独立保存与断电验证'].map((text,index)=><div key={text}><span>{String(index+1).padStart(2,'0')}</span><div><strong>{text}</strong><small>{index===0?'需要真实设备证据，演示不能替代':index===5?'RAM成功不代表掉电保存成功':'尚未接入本版上位机事务'}</small></div></div>)}</div></section></div>
    <section className="panel"><PanelHeader title="设备与通信信息" eyebrow="DEVICE IDENTITY"/><div className="identity-grid">{[{name:'主控',value:studio.gateway.info.mcu},{name:'固件构建',value:studio.gateway.info.build},{name:'传输',value:studio.gateway.info.transport},{name:'电机 / 驱动',value:studio.gateway.info.motor?studio.gateway.info.motor+' / '+studio.gateway.info.driver:undefined},{name:'端口',value:studio.gateway.port},{name:'标定映射',value:undefined}].map((entry)=><div key={entry.name}><span>{entry.name}</span><strong>{live?entry.value??'未知':'未连接 · 不使用历史值代填'}</strong></div>)}</div></section>
    <div className="maintenance-note"><ShieldCheck size={23}/><div><strong>量程、机械限位与承载能力是三个不同的条件。</strong><p>PMAX表示位置编码范围。扩大编码量程，不等于扩大机械安全空间，也不构成负载许可。</p></div></div>
  </>;
}
/** 从功能登记表查询扩展状态，不将规划页面当作已实现控制。 */
export function Lab({ navigate }: { navigate:(page:PageId)=>void }) {
  const [filter,setFilter]=useState('all');
  const [search,setSearch]=useState('');
  const features=FEATURES.filter((feature)=>(filter==='all'||feature.status===filter)&&(feature.title+feature.description).toLowerCase().includes(search.toLowerCase()));
  return <>
    <section className="lab-intro"><span className="lab-emblem"><FlaskConical size={29}/></span><div><span className="eyebrow">CAPABILITIES & ROADMAP</span><h2>一个工作台，逐步接入每一种能力。</h2><p>界面接入、板端实现和实机资格分别验收。功能不会因出现按钮而自动解锁。</p></div><Badge>能力登记表</Badge></section>
    <div className="feature-tools"><div className="segmented">{[{id:'all',label:'全部能力'},{id:'available',label:'本版已接入'},{id:'pending',label:'远程待接入'},{id:'experimental',label:'实验能力'},{id:'planned',label:'规划中'}].map((item)=><button key={item.id} className={filter===item.id?'selected':''} onClick={()=>setFilter(item.id)}>{item.label}</button>)}</div><div className="search-box"><Search size={16}/><input aria-label="搜索功能" placeholder="搜索功能…" value={search} onChange={(event)=>setSearch(event.target.value)}/></div></div>
    <div className="feature-grid">{features.map((feature)=><article className="feature-card" key={feature.id}><div className="feature-top"><span className="feature-icon">{feature.status==='available'?<Check size={21}/>:feature.category==='实验'?<FlaskConical size={21}/>:<SlidersHorizontal size={21}/>}</span><Badge tone={feature.status==='available'?'green':feature.status==='experimental'?'amber':'neutral'}>{STATUS_LABEL[feature.status]}</Badge></div><span className="eyebrow">{feature.category}</span><h3>{feature.title}</h3><p>{feature.description}</p><button className="text-link" onClick={()=>navigate(feature.page)}>查看对应页面<ChevronRight size={15}/></button></article>)}</div>
    {!features.length&&<div className="empty-state"><Search size={26}/><strong>没有匹配能力</strong></div>}
  </>;
}
