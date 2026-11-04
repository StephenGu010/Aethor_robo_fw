/** @file Fully intercepted browser regression for exposure maintenance; never contacts a device. */
import {test,expect,type Page} from '@playwright/test';

/** Provide a mock gateway and record every request without any serial or motor action. */
async function fixture(page:Page,{supported=true,unknown=false}={}) {
  const operations:string[]=[];
  let authorized=false;
  await page.route('**/api/**',async route=>{
    const request=route.request(),path=new URL(request.url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'隔离模拟',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/ports')return route.fulfill({json:[]});
    if(path==='/api/remote'){
      const body=request.postDataJSON();operations.push(body.operation);
      if(body.operation==='torque')return route.fulfill({json:{requestId:operations.length,operation:'torque',result:supported?'policy':'rejected',detail:'隔离模拟',fields:supported?{experimental:'1',motor:'7',axis:'output',continuous:'3.5',peak:'5.46',window_ms:'10000',budget:'175.616',used:authorized?'0':'175.616',authorized:authorized?'1':'0',can_reset:'1',token:String(operations.length+10),boot:'3',coord:'11'}:{}}});
      if(body.operation==='torque_reset'){
        expect(body.cooledConfirmed).toBe(true);expect(body.token).toBeGreaterThan(0);
        expect(body.peakMilliNm).toBeUndefined();authorized=!unknown;
        return route.fulfill({json:{requestId:operations.length,operation:'torque_reset',result:unknown?'unknown':'reset',detail:'隔离模拟',fields:{accepted:'1',authorized:'1'}}});
      }
      throw new Error('Unexpected action: '+body.operation);
    }
    throw new Error('Unexpected API: '+path);
  });
  await page.goto('/');
  await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  return operations;
}

test('策略不自动读取，冷却声明与新令牌只恢复一次配额，不使能或运动',async({page})=>{
  const operations=await fixture(page);
  const panel=page.getByRole('group',{name:'ID7实验转矩保护'});
  const restore=panel.getByRole('button',{name:'冷却后恢复实验配额'});
  await expect(restore).toBeDisabled();expect(operations).toEqual([]);
  await panel.getByRole('button',{name:'读取转矩保护策略'}).click();
  await expect(panel.getByText(/瞬时退出 5.46 N·m/)).toBeVisible();
  await panel.scrollIntoViewIfNeeded();
  await page.screenshot({path:'../artifacts/torque-envelope-preview-20261029.png',fullPage:true});
  await expect(restore).toBeDisabled();
  await panel.getByRole('checkbox').check();
  await restore.click();
  await expect(panel.getByText(/实验配额已恢复；未使能、未启动运动/)).toBeVisible();
  await expect(restore).toBeDisabled();
  expect(operations).toEqual(['torque','torque','torque_reset','torque']);
  await page.reload();expect(operations).toEqual(['torque','torque','torque_reset','torque']);
});

test('旧固件不授予恢复入口，未知结果不重发',async({page})=>{
  let operations=await fixture(page,{supported:false});
  let panel=page.getByRole('group',{name:'ID7实验转矩保护'});
  await panel.getByRole('button',{name:'读取转矩保护策略'}).click();
  await expect(panel.getByText('固件未确认实验策略，预算恢复保持关闭。')).toBeVisible();
  await panel.getByRole('checkbox').check();
  await expect(panel.getByRole('button',{name:'冷却后恢复实验配额'})).toBeDisabled();
  expect(operations).toEqual(['torque']);
  await page.unrouteAll({behavior:'wait'});
  operations=await fixture(page,{unknown:true});panel=page.getByRole('group',{name:'ID7实验转矩保护'});
  await panel.getByRole('button',{name:'读取转矩保护策略'}).click();
  await panel.getByRole('checkbox').check();
  await panel.getByRole('button',{name:'冷却后恢复实验配额'}).click();
  await expect(panel.getByText(/恢复未确认或被拒绝；未重发/)).toBeVisible();
  await expect(panel.getByRole('button',{name:'冷却后恢复实验配额'})).toBeDisabled();
  expect(operations).toEqual(['torque','torque','torque_reset']);
});
