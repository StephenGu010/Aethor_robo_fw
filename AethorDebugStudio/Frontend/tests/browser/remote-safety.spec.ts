/** @file 单按钮流程的STOP竞态与失能证据回归；所有接口拦截，无真实串口。 */
import { test, expect } from '@playwright/test';

test('步骤间STOP立即取消启动意图，迟到能力回复不能启动下一步',async({page})=>{
  const operations:string[]=[];let capabilityCount=0,originalId=0,serial=0;
  let resumeCapability:(()=>void)|undefined;
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/debug')return route.fulfill({json:{requestId:700,operation:'stop-all',motorId:7,result:'completed',accepted:true,feedbackConfirmed:true,detail:'模拟STOP完成',lines:[]}});
    if(path==='/api/remote'){
      const body=route.request().postDataJSON();operations.push(body.operation);serial++;
      if(body.operation==='caps'){
        capabilityCount++;
        if(capabilityCount===3)await new Promise<void>(resolve=>{resumeCapability=resolve;});
        return route.fulfill({json:{requestId:serial,operation:'caps',result:'capability',detail:'模拟',fields:{token:String(serial),boot:'7',owner:'0',mode:'1',fresh:'1',static:'1',preflight:'1',active_id:'0'}}});
      }
      if(body.operation==='result')return route.fulfill({json:{requestId:serial,operation:'result',result:'completed',detail:'模拟',fields:{id:String(originalId),accepted:'1',disabled:'1'}}});
      originalId=serial;
      return route.fulfill({json:{requestId:serial,operation:body.operation,result:'queued',detail:'模拟',fields:{}}});
    }
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  await page.getByRole('button',{name:'按参数启动一次 · 单次静载保持',exact:true}).click();
  await expect.poll(()=>!!resumeCapability).toBe(true);
  await page.getByRole('button',{name:'STOP 全部轴',exact:true}).click();
  await expect(page.getByRole('region',{name:'最近设备回执'}).getByText('模拟STOP完成',{exact:true})).toBeVisible();
  resumeCapability!();
  await expect(page.getByRole('button',{name:'按参数启动一次 · 单次静载保持',exact:true})).toBeEnabled();
  expect(operations.filter(operation=>!['caps','result'].includes(operation))).toEqual(['preflight','configure']);
});

test('自动恢复遇到completed但失能未确认，持续锁定且不启动保持',async({page})=>{
  const operations:string[]=[];let originalId=0,serial=0,resultReads=0;
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/remote'){
      const body=route.request().postDataJSON();operations.push(body.operation);serial++;
      if(body.operation==='caps')return route.fulfill({json:{requestId:serial,operation:'caps',result:'capability',detail:'模拟',fields:{token:String(serial),boot:'7',owner:'0',mode:'1',fresh:'1',static:'1',preflight:'1',active_id:'0'}}});
      if(body.operation==='result'){resultReads++;return route.fulfill({json:{requestId:serial,operation:'result',result:'completed',detail:'模拟缺失能确认',fields:{id:String(originalId),accepted:'1',disabled:'0'}}});}
      originalId=serial;return route.fulfill({json:{requestId:serial,operation:body.operation,result:'queued',detail:'模拟',fields:{}}});
    }
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  await page.getByRole('button',{name:'按参数启动一次 · 单次静载保持',exact:true}).click();
  await expect.poll(()=>resultReads).toBeGreaterThanOrEqual(2);
  await expect(page.getByRole('button',{name:'按参数启动一次 · 单次静载保持',exact:true})).toBeDisabled();
  await expect(page.getByRole('alert')).toContainText('上次启动结果尚未确认');
  expect(operations.filter(operation=>!['caps','result'].includes(operation))).toEqual(['preflight']);
});

test('恢复另一事务不能沿用此前RAM配置的免失能规则',async({page})=>{
  let originalId=0,serial=0;const queriedIds:number[]=[];
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/remote'){
      if(route.request().method()==='GET')return route.fulfill({json:{requestId:81,operation:'result',result:'completed',detail:'另一事务',fields:{id:'77',accepted:'1',disabled:'0'}}});
      const body=route.request().postDataJSON();serial++;
      if(body.operation==='caps')return route.fulfill({json:{requestId:serial,operation:'caps',result:'capability',detail:'模拟',fields:{token:'3',fresh:'1',static:'1'}}});
      if(body.operation==='result'){queriedIds.push(body.transactionId);return route.fulfill({json:{requestId:serial,operation:'result',result:'completed',detail:'模拟',fields:{id:String(body.transactionId),accepted:'1',disabled:'0'}}});}
      originalId=serial;return route.fulfill({json:{requestId:serial,operation:body.operation,result:'queued',detail:'模拟',fields:{}}});
    }
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByRole('button',{name:'读取远程能力'}).click();
  await page.getByText('高级分步操作',{exact:true}).click();
  await page.getByRole('button',{name:'审核并冻结'}).click();
  await page.getByRole('button',{name:'显式提交一次'}).click();
  await expect.poll(()=>queriedIds.includes(originalId)).toBe(true);
  await expect(page.getByRole('button',{name:'按参数启动一次 · 下发静载RAM参数',exact:true})).toBeEnabled();
  await page.getByRole('button',{name:'恢复最近事务编号'}).click();
  await expect.poll(()=>queriedIds.includes(77)).toBe(true);
  await expect(page.getByRole('button',{name:'按参数启动一次 · 下发静载RAM参数',exact:true})).toBeDisabled();
  await expect(page.getByRole('alert')).toContainText('上次启动结果尚未确认');
});

test('恢复查询期间断连，迟到回执不会进入重连后的会话',async({page})=>{
  let connected=true;let finishRecovery:(()=>void)|undefined;const queriedIds:number[]=[];
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected,port:'MOCK',status:connected?'模拟连接':'模拟断开',info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/remote'){
      if(route.request().method()==='GET'){
        await new Promise<void>(resolve=>{finishRecovery=resolve;});
        return route.fulfill({json:{requestId:88,operation:'result',result:'completed',detail:'旧连接回执',fields:{id:'77',accepted:'1',disabled:'1'}}});
      }
      const body=route.request().postDataJSON();queriedIds.push(body.transactionId);
      return route.fulfill({json:{requestId:89,operation:'result',result:'completed',detail:'旧事务',fields:{id:'77',accepted:'1',disabled:'1'}}});
    }
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByRole('button',{name:'恢复最近事务编号'}).click();
  await expect.poll(()=>!!finishRecovery).toBe(true);
  connected=false;
  await expect(page.getByRole('button',{name:'STOP 全部轴',exact:true})).toBeDisabled();
  connected=true;
  await expect(page.getByRole('button',{name:'STOP 全部轴',exact:true})).toBeEnabled();
  finishRecovery!();
  await expect(page.getByRole('button',{name:'恢复最近事务编号'})).toBeEnabled();
  await expect(page.getByRole('button',{name:'读取原事务结果'})).toBeDisabled();
  expect(queriedIds).toEqual([]);
});
