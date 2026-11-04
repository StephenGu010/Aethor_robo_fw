/** @file 可搜索、可打印的新手手册；直接读取独立Markdown手册，避免两份说明漂移。 */
import {useState} from 'react';
import {BookOpen,Printer,Search} from 'lucide-react';
import manualSource from '../../../docs/user-manual.md?raw';
import type {PageId} from '../types';

const chapters=manualSource.split(/^## /m).slice(1).map((chapter,index)=>{
  const [title,...lines]=chapter.trim().split('\n');
  return {id:`manual-chapter-${index+1}`,title,body:lines.join('\n').trim()};
});

/** 将本地受信手册的段落、列表和参数表渲染为语义HTML，不执行HTML或脚本。 */
function ManualContent({body}:{body:string}){
  return <>{body.split(/\n\s*\n/).map((block,index)=>{
    const lines=block.split('\n');
    if(lines.every(line=>/^\d+\. /.test(line)))return <ol key={index}>{lines.map((line,lineIndex)=><li key={lineIndex}>{line.replace(/^\d+\. /,'')}</li>)}</ol>;
    if(lines[0].startsWith('|')){
      const rows=lines.filter(line=>!/^\|[\s|:-]+$/.test(line)).map(line=>line.split('|').slice(1,-1).map(cell=>cell.trim()));
      return <div className="manual-table-wrap" key={index}><table><caption>参数的单位、含义与允许范围</caption><thead><tr>{rows[0].map(cell=><th scope="col" key={cell}>{cell}</th>)}</tr></thead><tbody>{rows.slice(1).map((row,rowIndex)=><tr key={rowIndex}>{row.map((cell,cellIndex)=>cellIndex===0?<th scope="row" key={cellIndex}>{cell}</th>:<td key={cellIndex}>{cell}</td>)}</tr>)}</tbody></table></div>;
    }
    return <p key={index}>{block}</p>;
  })}</>;
}

/** 搜索整章标题与正文；目录链接与打印保留同一份可独立阅读的手册。 */
export function UserManual({navigate}:{navigate:(page:PageId)=>void}){
  const [query,setQuery]=useState('');
  const normalized=query.trim().toLocaleLowerCase();
  const visible=chapters.filter(chapter=>(chapter.title+' '+chapter.body).toLocaleLowerCase().includes(normalized));
  return <div className="user-manual">
    <div className="manual-toolbar"><label className="manual-search"><Search size={18}/><span className="visually-hidden">搜索操作手册</span><input type="search" aria-label="搜索操作手册" placeholder="搜索：连接、Kp、零点、失能…" value={query} onChange={event=>setQuery(event.target.value)}/></label><button className="button secondary" disabled={!query} onClick={()=>setQuery('')}>清除搜索</button><button className="button secondary" onClick={()=>window.print()}><Printer size={16}/>打印手册</button></div>
    <p className="manual-search-count" role="status">{normalized?`找到 ${visible.length} 个相关章节`:`共 ${chapters.length} 章 · 当前设备的能力与反馈是执行依据`}</p>
    <div className="manual-layout"><nav className="manual-toc" aria-label="手册目录"><strong><BookOpen size={17}/>手册目录</strong>{visible.map(chapter=><a key={chapter.id} href={'#'+chapter.id}>{chapter.title}</a>)}<button className="button secondary" onClick={()=>navigate('overview')}>返回总览与连接</button></nav>
      <article className="manual-content" aria-label="操作手册正文">{visible.length?visible.map(chapter=><section id={chapter.id} key={chapter.id} tabIndex={-1}><h2>{chapter.title}</h2><ManualContent body={chapter.body}/></section>):<div className="empty-state"><Search size={24}/><h2>没有匹配章节，请换一个关键词。</h2></div>}</article>
    </div>
  </div>;
}

/** 总览新手路线只切换视图，连接和真实启动仍由独立按钮显式执行。 */
export function GettingStarted({navigate,connect}:{navigate:(page:PageId)=>void;connect:()=>void}){
  return <section className="getting-started" aria-labelledby="getting-started-title"><div className="getting-started-copy"><span className="eyebrow">GET STARTED</span><h2 id="getting-started-title">第一次使用，从这里开始</h2><p>先确认来源和反馈，再设置一次测试。主启动按钮会执行真实流程；阅读手册、连接和编辑参数均不会启动。</p><button className="button primary" onClick={()=>navigate('guide')}><BookOpen size={17}/>阅读新手手册</button></div><ol className="getting-started-steps"><li><strong>连接与检查</strong><span>USB连通与电机反馈分开核对。</span><button className="text-link" onClick={connect}>连接与只读查询 →</button></li><li><strong>选择测试</strong><span>静载保持当前位置；POS相对移动后保持。</span><div><button className="text-link" onClick={()=>navigate('static')}>单轴静载 →</button><button className="text-link" onClick={()=>navigate('pos')}>POS带载 →</button></div></li><li><strong>结束并保留记录</strong><span>核对失能和控制权，再导出本次记录。</span><button className="text-link" onClick={()=>navigate('records')}>记录与回放 →</button></li></ol></section>;
}
