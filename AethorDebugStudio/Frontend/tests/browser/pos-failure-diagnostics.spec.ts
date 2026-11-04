/** @file POS欠压形态失败的隔离浏览器回归：所有API均拦截，不连接真实设备。 */
import {test,expect} from '@playwright/test';

for(const [reason,authorization,torque,label] of [['1','0','3.5238','3.50 N·m基线'],['2','1','5.4700','5.46 N·m峰值'],['3','1','3.6000','10秒超额窗口'],['4','1','4.0000','累计超额预算']]) {
  test(`隔离POS首故障显示${label}且不自动释放或恢复配额`,async({page})=>{
    let serial=700,transactionId=0,operation='',owner='0';const submitted:string[]=[];
    await page.route('**/api/**',async route=>{
      const path=new URL(route.request().url()).pathname;
      if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'隔离模拟 · 非实机',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true}});
      if(path==='/api/ports')return route.fulfill({json:[]});
      if(path==='/api/remote') {
        const body=route.request().postDataJSON();serial++;
        if(body.operation==='caps')return route.fulfill({json:{requestId:serial,operation:'caps',result:'capability',detail:'隔离模拟',fields:{token:String(serial),boot:'7',owner,mode:'2',fresh:'1',load:'1',load_delta:'10',load_speed:'10',active_id:'0'}}});
        if(body.operation==='result') {
          if(operation==='acquire')owner='2';
          return route.fulfill({json:{requestId:serial,operation:'result',result:operation==='pos_load'?'failed':'completed',detail:'隔离模拟',fields:{id:String(transactionId),accepted:'1',disabled:'1',reason:operation==='pos_load'?'16':'0',stage:'6',motor:'7',...(operation==='pos_load'?{tq_reason:reason,tq_auth:authorization,tq_used:reason==='1'?'175.616000':'0.000000',tq_nm:torque}:{})}}});
        }
        operation=String(body.operation);transactionId=serial;submitted.push(operation);
        return route.fulfill({json:{requestId:serial,operation,result:'queued',detail:'隔离模拟',fields:{}}});
      }
      throw new Error('未拦截API：'+path);
    });
    await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
    await page.getByRole('navigation').getByRole('button',{name:'POS 带载测试',exact:true}).click();
    await page.getByLabel('法兰相对位移 °',{exact:true}).fill('1');
    await page.getByRole('button',{name:'按参数启动一次 · POS带载移动并保持',exact:true}).click();
    await expect(page.locator('p.remote-status')).toContainText(`首次转矩保护：${label}`);
    await expect(page.locator('p.remote-status')).toContainText('板端回执已确认失能');
    expect(submitted).toEqual(['acquire','pos_load']);
    await page.screenshot({path:`../artifacts/pos-first-fault-${reason}-20261029.png`,fullPage:true});
  });
}

test('POS已失败但失能未确认时显示原因，跨页面仍锁定且不释放或重发',async({page})=>{
  const submitted:string[]=[];
  let owner='0',operation='',transactionId=0,serial=500;
  await page.route('**/api/**',async route=>{
    const request=route.request(),path=new URL(request.url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'隔离模拟 · 非实机',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/ports')return route.fulfill({json:[]});
    if(path==='/api/remote'){
      const body=request.postDataJSON();serial++;
      if(body.operation==='caps')return route.fulfill({json:{requestId:serial,operation:'caps',result:'capability',detail:'隔离模拟',fields:{token:String(serial),boot:'7',owner,mode:'2',fresh:'1',load:'1',load_delta:'10',load_speed:'10',active_id:'0'}}});
      if(body.operation==='result'){
        if(operation==='acquire')owner='2';
        return route.fulfill({json:{requestId:serial,operation:'result',result:operation==='pos_load'?'failed':'completed',detail:'板端回执',fields:{id:String(transactionId),accepted:'1',disabled:operation==='pos_load'?'0':'1',reason:operation==='pos_load'?'9':'0',stage:'1',error:'15',motor:'7'}}});
      }
      operation=String(body.operation);transactionId=serial;submitted.push(operation);
      return route.fulfill({json:{requestId:serial,operation,result:'queued',detail:'隔离模拟',fields:{}}});
    }
    throw new Error('未拦截的测试API：'+path);
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'POS 带载测试',exact:true}).click();
  await page.getByLabel('法兰相对位移 °',{exact:true}).fill('1');
  const start=page.getByRole('button',{name:'按参数启动一次 · POS带载移动并保持',exact:true});
  await start.click();
  await expect(page.getByRole('button',{name:'读取原事务结果',exact:true})).toBeEnabled();
  const status=page.locator('p.remote-status');
  await expect(status).toContainText('板端已报告失败');
  await expect(status).toContainText('失能未确认');
  await expect(status).toContainText('原因9');
  await expect(start).toBeDisabled();
  expect(submitted).toEqual(['acquire','pos_load']);
  await page.screenshot({path:'../artifacts/pos-failure-diagnostics-fixture-20261029.png',fullPage:true});
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  await expect(page.getByRole('button',{name:'按参数启动一次 · 单次静载保持',exact:true})).toBeDisabled();
  expect(submitted).toEqual(['acquire','pos_load']);
});
