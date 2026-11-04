/** @file 配额失能准备的隔离浏览器验收；所有API拦截，禁止连接真实串口。 */
import {test,expect,type Page} from '@playwright/test';

/** 模拟模式2缺反馈、失能后更新及维护读回，并留存所有命令。 */
async function prepare(page:Page,options:{holdDisable?:boolean;after?:number;unknown?:boolean;resetUnknown?:boolean;oldGateway?:boolean}={}){
  const operations:string[]=[];let sampled=false,authorized=false,peak=5460,connected=true;
  let reply:(()=>void)|undefined;
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected,port:'MOCK',status:'隔离模拟',info:{},motors:[],lines:[],debugAvailable:true,torquePreparationAvailable:!options.oldGateway}});
    if(path==='/api/ports')return route.fulfill({json:[]});
    const body=route.request().postDataJSON();operations.push(body.operation);
    if(path==='/api/debug'){
      expect(['torque-disable','stop-all']).toContain(body.operation);
      if(body.operation==='torque-disable'){
        expect(body.motorId).toBe(7);expect(body.loadSupported).toBe(true);
        if(options.holdDisable)await new Promise<void>(resolve=>{reply=resolve;});
        sampled=true;
      }
      return route.fulfill({json:{requestId:11,operation:body.operation,motorId:7,accepted:true,result:options.unknown?'unknown':'completed',detail:'隔离失能回执',lines:[],feedbackConfirmed:false}});
    }
    if(path!=='/api/remote')throw new Error('Unexpected API '+path);
    if(body.operation==='caps')return route.fulfill({json:{requestId:2,operation:'caps',result:'capability',detail:'模拟',fields:{boot:'3',coord:'11',owner:'0',authority:'0',mode:'2',active_id:'0',terminal_id:'77',load:'1'}}});
    if(body.operation==='result')return route.fulfill({json:{requestId:5,operation:'result',result:'completed',detail:'旧运动终态',fields:{id:'77',accepted:'1',disabled:'1'}}});
    if(body.operation==='torque'){
      const blockers=sampled?(options.after??0):32;
      return route.fulfill({json:{requestId:3,operation:'torque',result:'policy',detail:'模拟',fields:{experimental:'1',motor:'7',axis:'output',continuous:'3.5',peak:String(peak/1000),window_ms:'10000',budget:'175.616',used:authorized?'0':'175.616',authorized:authorized?'1':'0',can_reset:blockers?'0':'1',token:blockers?'0':'19',boot:'3',coord:'11',policy_v:'2',min_mnm:'3500',max_mnm:'6240',peak_mnm:String(peak),blockers:String(blockers),reset_temp:'35',evidence_ms:'5000',experiment_fresh_ms:'20'}}});
    }
    expect(body.operation).toBe('torque_reset');expect(sampled).toBe(true);expect(body.peakMilliNm).toBe(6240);
    authorized=true;peak=body.peakMilliNm;
    return route.fulfill({json:{requestId:4,operation:'torque_reset',result:options.resetUnknown?'unknown':'reset',fields:{accepted:'1',authorized:'1'}}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'POS 带载测试',exact:true}).click();
  const panel=page.getByRole('group',{name:'ID7实验转矩保护'});
  await panel.getByRole('button',{name:'读取转矩保护策略'}).click();
  await panel.getByRole('button',{name:'80%峰值 · 6.24 N·m'}).click();
  await panel.getByRole('checkbox').check();
  return {panel,operations,reply:()=>reply?.(),pending:()=>!!reply,disconnect:()=>{connected=false;}};
}
test('POS页模式2显式一次失能后启用所选配额，双击不重发且不运动',async({page})=>{
  const f=await prepare(page);await f.panel.getByRole('button',{name:'失能检查并应用试验配额'}).dblclick();
  await expect(f.panel.getByText(/读回确认 6.240/)).toBeVisible();
  expect(f.operations).toEqual(['torque','caps','torque','torque-disable','torque','torque_reset','torque']);
  await expect(f.panel.getByText('上次读取生效边界：6.24 N·m 实验退出线')).toBeVisible();
  await expect(f.panel.getByRole('checkbox')).not.toBeChecked();
  await page.screenshot({path:'../artifacts/torque-preparation-mock-20261103.png',fullPage:true});
});
test('新反馈温度过高拒绝恢复且不重复失能',async({page})=>{
  const f=await prepare(page,{after:512});await f.panel.getByRole('button',{name:'失能检查并应用试验配额'}).click();
  await expect(f.panel.getByText(/新反馈未通过冷却/)).toBeVisible();
  expect(f.operations.filter(x=>x==='torque-disable')).toHaveLength(1);expect(f.operations).not.toContain('torque_reset');
});
test('STOP取消维护后续步骤，迟到失能完成不会启用配额',async({page})=>{
  const f=await prepare(page,{holdDisable:true});await f.panel.getByRole('button',{name:'失能检查并应用试验配额'}).click();
  await expect.poll(f.pending).toBe(true);await page.getByRole('button',{name:'STOP 全部轴',exact:true}).click();
  f.reply();await expect.poll(()=>f.operations.includes('stop-all')).toBe(true);
  await expect(page.getByText('失能提交后操作意图已变化；未继续恢复配额，请读取原回执。')).toBeVisible();
  await expect(f.panel.getByRole('button',{name:'读取转矩保护策略'})).toBeDisabled();expect(f.operations).not.toContain('torque_reset');
});
test('导航期间保留跨页维护锁，旧页面迟到结果不能继续维护',async({page})=>{
  const f=await prepare(page,{holdDisable:true});await f.panel.getByRole('button',{name:'失能检查并应用试验配额'}).click();
  await expect.poll(f.pending).toBe(true);await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  await expect(page.getByRole('button',{name:'按参数启动一次 · 单次静载保持',exact:true})).toBeDisabled();
  f.reply();await expect(page.getByText('失能提交后操作意图已变化；未继续恢复配额，请读取原回执。')).toBeVisible();
  await expect(page.getByRole('group',{name:'ID7实验转矩保护'}).getByRole('button',{name:'读取转矩保护策略'})).toBeDisabled();expect(f.operations).not.toContain('torque_reset');
});
test('断连和未知失能都不继续恢复',async({page})=>{
  const f=await prepare(page,{holdDisable:true});await f.panel.getByRole('button',{name:'失能检查并应用试验配额'}).click();
  await expect.poll(f.pending).toBe(true);f.disconnect();await expect(page.getByRole('region',{name:'当前连接与控制状态'})).toContainText('USB未连接');
  f.reply();expect(f.operations).not.toContain('torque_reset');
});
test('旧网关不开放失能准备按钮',async({page})=>{
  const f=await prepare(page,{oldGateway:true});await expect(f.panel.getByRole('button',{name:'失能检查并应用试验配额'})).toBeDisabled();expect(f.operations).toEqual(['torque']);
});
test('旧运动失能终态不能解除本次维护未知锁，只读维护核验才解除',async({page})=>{
  const f=await prepare(page,{unknown:true});await f.panel.getByRole('button',{name:'失能检查并应用试验配额'}).click();
  await expect(page.getByRole('button',{name:'只读核验维护状态'})).toBeVisible();
  await page.getByRole('button',{name:'读取远程能力',exact:true}).click();
  await page.getByRole('button',{name:'恢复设备最近终态 #77',exact:true}).click();
  await expect.poll(()=>f.operations.includes('result')).toBe(true);
  await expect(page.getByRole('button',{name:'按参数启动一次 · POS带载移动并保持',exact:true})).toBeDisabled();
  await expect(page.getByRole('button',{name:'只读核验维护状态'})).toBeEnabled();
  await page.getByRole('button',{name:'只读核验维护状态'}).click();
  await expect(page.getByText(/只读核验完成：当前生效退出线 3.500/)).toBeVisible();
  expect(f.operations).not.toContain('torque_reset');
});
test('配额恢复响应未知也保留独立维护锁，禁止重新启动',async({page})=>{
  const f=await prepare(page,{resetUnknown:true});await f.panel.getByRole('button',{name:'失能检查并应用试验配额'}).click();
  await expect(page.getByRole('button',{name:'只读核验维护状态'})).toBeVisible();
  await expect(page.getByRole('button',{name:'按参数启动一次 · POS带载移动并保持',exact:true})).toBeDisabled();
  expect(f.operations.filter(x=>x==='torque_reset')).toHaveLength(1);
  await page.getByRole('button',{name:'只读核验维护状态'}).click();
  await expect(page.getByText(/只读核验完成：当前生效退出线 6.240/)).toBeVisible();
});
