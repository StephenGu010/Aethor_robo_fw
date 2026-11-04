/** @file 历史发布基线卡；与当前串口身份/实机资格分别呈现。 */
import { ArrowUpRight, Check, Cpu, FlaskConical, Radio, ShieldCheck } from 'lucide-react';
import { STATIC_RELEASE } from '../staticPolicy';
import { Badge } from './Common';
import {outputDegreesToDisplayDegrees,type ControlCoordinate} from '../axisUnits';
/** 总览、静载和维护页共享同一份已核对发布证据。 */
export function ReleaseBaseline({ compact=false, onOpen,displayCoordinate='output' }: { compact?:boolean; onOpen?:()=>void;displayCoordinate?:ControlCoordinate }) {
  /** 发布阈值仅换算显示，原策略口径保持输出轴。 */
  const angle=(value:number)=>outputDegreesToDisplayDegrees(value,displayCoordinate).toFixed(displayCoordinate==='flange'?3:1);
  const axisLabel=displayCoordinate==='flange'?'法兰端':'输出轴';
  return <section className={'release-baseline '+(compact?'compact':'')} aria-label="最新发布基线">
    <div className="release-heading"><span className="release-chip"><Cpu size={19}/></span><div><span className="eyebrow">LATEST RELEASE / {STATIC_RELEASE.date}</span><h2>ID7 静载窗口已分离</h2></div><Badge tone="green">主控部署有回执</Badge>{onOpen&&<button className="text-link" onClick={onOpen}>打开静载工作区<ArrowUpRight size={15}/></button>}</div>
    <div className="release-facts">
      <div><span>精度观察</span><strong>{angle(.5)}<small>° {axisLabel}</small></strong><p>超出记为精度超差</p></div>
      <div><span>保持保护</span><strong>{angle(1)}<small>° {axisLabel}</small></strong><p>超过后进入原失能清理</p></div>
      <div><span>启动接管</span><strong>{angle(.5)}<small>° {axisLabel}</small></strong><p>原检查继续保留</p></div>
      <div><span>已发布等效增益</span><strong>80<small> / Kd 0.2</small></strong><p>更高增益尚未部署</p></div>
    </div>
    {!compact&&<div className="release-progress"><span><Check size={13}/>软件离线验证</span><span><Check size={13}/>主控烧录与读回</span><span><FlaskConical size={13}/>抗扰物理资格待验证</span><span><Radio size={13}/>PC远程控制待接入</span></div>}
    <div className="release-foot"><span><ShieldCheck size={14}/>发布记录 ≠ 当前设备识别 · 转矩3.5 N·m及其他保护保留</span><details><summary>发布凭据</summary><p>LCD-POS-Load-Test · 505,748字节读回一致</p><code>{STATIC_RELEASE.firmwareHash}</code><p>来源：F聊天与对应部署回执。本轮未连接硬件核对当前镜像。</p></details></div>
  </section>;
}
