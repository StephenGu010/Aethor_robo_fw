/** @file 可调转矩真实浏览器流程；全部API在测试内拦截，不访问串口。 */
import {test,expect,type Page} from '@playwright/test';

/** 模拟板端读回与一次维护事务，保留所有请求用于断言没有运动。 */
async function prepare(page:Page,options:{blockers?:number;unknown?:boolean;readbackMismatch?:boolean;bootChanges?:boolean;holdPolicy?:boolean}={}) {
  const requests:Record<string,unknown>[]=[];
  let peak=5460,authorized=false;
  let connected=true,releasePolicy:(()=>void)|undefined;
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected,port:'MOCK',status:'隔离模拟',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/ports')return route.fulfill({json:[]});
    if(path!=='/api/remote')throw new Error('Unexpected API '+path);
    const body=route.request().postDataJSON();requests.push(body);
    if(options.holdPolicy&&body.operation==='torque'&&requests.length===2)await new Promise<void>(resolve=>{releasePolicy=resolve;});
    if(body.operation==='torque')return route.fulfill({json:{requestId:requests.length,operation:'torque',result:'policy',detail:'隔离模拟',fields:{
      experimental:'1',motor:'7',axis:'output',policy_v:'2',min_mnm:'3500',max_mnm:'6240',peak_mnm:String(peak),
      continuous:'3.5',peak:String(peak/1000),window_ms:'10000',budget:'175.616',used:authorized?'0':'175.616',
      authorized:authorized?'1':'0',reason:'0',can_reset:options.blockers?'0':'1',blockers:String(options.blockers??0),
      token:options.blockers?'0':String(requests.length+10),boot:options.bootChanges&&requests.length>1?'4':'3',coord:'11',reset_temp:'35',evidence_ms:'5000',experiment_fresh_ms:'20',
    }}});
    if(body.operation==='torque_reset'){
      expect(body.cooledConfirmed).toBe(true);expect(body.token).toBeGreaterThan(0);
      if(!options.unknown){peak=options.readbackMismatch?5460:body.peakMilliNm;authorized=true;}
      return route.fulfill({json:{requestId:requests.length,operation:'torque_reset',result:options.unknown?'unknown':'reset',detail:'隔离模拟',fields:{accepted:'1',authorized:'1'}}});
    }
    throw new Error('Unexpected action '+body.operation);
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  const panel=page.getByRole('group',{name:'ID7实验转矩保护'});
  await panel.getByRole('button',{name:'读取转矩保护策略'}).click();
  return {panel,requests,disconnect:()=>{connected=false;},reply:()=>releasePolicy?.()};
}

test('80%草稿先不生效，冷却维护后读回确认且不产生电机命令',async({page})=>{
  const {panel,requests}=await prepare(page);
  await panel.getByRole('button',{name:'80%峰值 · 6.24 N·m'}).click();
  await expect(panel.getByLabel('试验转矩退出线 N·m')).toHaveValue('6.24');
  await expect(panel.getByText(/约6.58秒/)).toBeVisible();
  expect(requests.map(request=>request.operation)).toEqual(['torque']);
  await panel.getByRole('checkbox').check();
  await panel.getByRole('button',{name:'冷却后恢复实验配额'}).dblclick();
  await expect(panel.getByText(/读回确认.*6.24/)).toBeVisible();
  expect(requests.map(request=>request.operation)).toEqual(['torque','torque','torque_reset','torque']);
  expect(requests[2].peakMilliNm).toBe(6240);
  await expect(panel.getByRole('checkbox')).not.toBeChecked();
});

test('越界草稿和板端温度阻塞不能申请配额',async({page})=>{
  const {panel,requests}=await prepare(page,{blockers:512});
  await expect(panel.getByText(/温度无效或任一路超过35℃/)).toBeVisible();
  await panel.getByLabel('试验转矩退出线 N·m').fill('6.25');
  await panel.getByRole('checkbox').check();
  await expect(panel.getByRole('button',{name:'冷却后恢复实验配额'})).toBeDisabled();
  expect(requests.map(request=>request.operation)).toEqual(['torque']);
});

test('维护回执未知不重发，不宣称生效',async({page})=>{
  const {panel,requests}=await prepare(page,{unknown:true});
  await panel.getByRole('checkbox').check();
  await panel.getByRole('button',{name:'冷却后恢复实验配额'}).click();
  await expect(panel.getByText(/恢复未确认或被拒绝；未重发/)).toBeVisible();
  expect(requests.map(request=>request.operation)).toEqual(['torque','torque','torque_reset']);
});

test('读回与所选值不符不会显示确认成功',async({page})=>{
  const {panel,requests}=await prepare(page,{readbackMismatch:true});
  await panel.getByRole('button',{name:'80%峰值 · 6.24 N·m'}).click();
  await panel.getByRole('checkbox').check();
  await panel.getByRole('button',{name:'冷却后恢复实验配额'}).click();
  await expect(panel.getByText(/读回未确认/)).toBeVisible();
  expect(requests.filter(request=>request.operation==='torque_reset')).toHaveLength(1);
});

test('维护前新查询发现启动号变化，冷却声明作废且不写',async({page})=>{
  const {panel,requests}=await prepare(page,{bootChanges:true});
  await panel.getByRole('checkbox').check();
  await panel.getByRole('button',{name:'冷却后恢复实验配额'}).click();
  await expect(panel.getByText(/设备身份或策略版本已变化/)).toBeVisible();
  await expect(panel.getByRole('checkbox')).not.toBeChecked();
  expect(requests.map(request=>request.operation)).toEqual(['torque','torque']);
});

test('查询期间断连，迟到策略不能继续恢复配额',async({page})=>{
  const transport=await prepare(page,{holdPolicy:true});
  await transport.panel.getByRole('checkbox').check();
  await transport.panel.getByRole('button',{name:'冷却后恢复实验配额'}).click();
  await expect.poll(()=>transport.requests.length).toBe(2);
  transport.disconnect();
  await expect(page.getByRole('region',{name:'当前连接与控制状态'})).toContainText('USB未连接');
  transport.reply();
  await expect(transport.panel.getByRole('button',{name:'冷却后恢复实验配额'})).toBeDisabled();
  await expect(transport.panel.getByRole('checkbox')).not.toBeChecked();
  expect(transport.requests.map(request=>request.operation)).toEqual(['torque','torque']);
});
