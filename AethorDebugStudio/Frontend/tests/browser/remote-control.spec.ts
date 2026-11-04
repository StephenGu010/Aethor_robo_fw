/** @file 浏览器全API拦截验证审核与单次提交，不访问任何串口服务。 */
import {test,expect} from '@playwright/test';
test('设置参数后一键完成静载前置步骤和保持，未重复发送',async({page})=>{
  const operations:string[]=[];let owner='0',original=0,serial=0,current='',mode='2',holdDuration=0;
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/remote'){
      const body=route.request().postDataJSON();operations.push(body.operation);serial++;
      if(body.operation==='caps')return route.fulfill({json:{requestId:serial,operation:'caps',result:'capability',detail:'模拟',fields:{token:String(serial),boot:'7',owner,mode,fresh:'1',static:'1',preflight:'1',switch:'1',active_id:'0'}}});
      if(body.operation==='result'){
        if(current==='acquire')owner='2';
        if(current==='release')owner='0';
        if(current==='mode_mit')mode='1';
        return route.fulfill({json:{requestId:serial,operation:'result',result:'completed',detail:'模拟',fields:{id:String(original),accepted:'1',disabled:'1'}}});
      }
      current=body.operation;original=serial;
      if(current==='hold')holdDuration=body.duration;
      return route.fulfill({json:{requestId:serial,operation:current,result:'queued',detail:'模拟',fields:{}}});
    }
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByLabel('本次操作').selectOption('hold');
  await expect(page.getByLabel('输出轴相对位移 °',{exact:true})).toHaveCount(0);
  await page.getByLabel('保持时间 s',{exact:true}).fill('2');
  await page.getByRole('button',{name:'按参数启动一次 · 单次静载保持',exact:true}).click();
  await expect(page.getByText('本次流程已收到板端完成回执')).toBeVisible();
  expect(operations).toEqual(['caps','acquire','result','caps','mode_mit','result','caps','release','result','caps','preflight','result','caps','configure','result','caps','acquire','result','caps','hold','result','caps','release','result']);
  expect(holdDuration).toBe(2000);
});
test('专用POS带载入口隐藏MIT参数，旧固件缺少能力时拒绝提交',async({page})=>{
  const operations:string[]=[];
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/remote'){const body=route.request().postDataJSON();operations.push(body.operation);return route.fulfill({json:{requestId:1,operation:'caps',result:'capability',detail:'旧能力',fields:{schema:'1',token:'7',fresh:'1',mode:'2',pos:'1'}}});}
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByLabel('本次操作').selectOption('pos_load');
  await expect(page.getByLabel('输出轴Kp N·m/rad',{exact:true})).toHaveCount(0);
  await page.getByLabel('输出轴相对位移 °',{exact:true}).fill('1');
  await page.getByRole('button',{name:'读取远程能力'}).click();
  await page.getByText('高级分步操作',{exact:true}).click();
  await page.getByRole('button',{name:'审核并冻结'}).click();
  await expect(page.getByText('设备未开放专用POS带载能力，或模式/范围不匹配；普通POS不能替代')).toBeVisible();
  await expect(page.getByRole('button',{name:'显式提交一次'})).toBeDisabled();
  expect(operations).toEqual(['caps']);
});
test('远程草稿必须能力确认和冻结后才单次提交，终态只读查询',async({page})=>{
  const commands:Record<string,unknown>[]=[];
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true,readOnly:true}});
    if(path==='/api/remote'){
      const body=route.request().postDataJSON();commands.push(body);
      return route.fulfill({json:{requestId:commands.length,operation:body.operation,result:body.operation==='caps'?'capability':body.operation==='result'?'completed':'queued',detail:'模拟回执',fields:body.operation==='caps'?{schema:'1',token:'17',boot:'3',epoch:'2',fresh:'1',mode:'2',owner:'0',pos:'1',static:'1',max_delta:'1',max_speed:'1'}:{schema:'1',id:'2',reason:'0',accepted:'1',disabled:'1'}}});
    }
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByText('高级分步操作',{exact:true}).click();
  await expect(page.getByRole('button',{name:'显式提交一次'})).toBeDisabled();
  expect(commands).toHaveLength(0);
  await page.getByLabel('本次操作').selectOption('pos');
  await page.getByRole('button',{name:'读取远程能力'}).click();
  await page.getByLabel('输出轴相对位移 °',{exact:true}).fill('1');
  await page.getByLabel('输出轴速度 °/s',{exact:true}).fill('5');
  await page.getByRole('button',{name:'审核并冻结'}).click();
  await expect(page.getByRole('button',{name:'显式提交一次'})).toBeEnabled();
  await page.getByRole('button',{name:'显式提交一次'}).click();
  await expect(page.getByRole('button',{name:'显式提交一次'})).toBeDisabled();
  await expect.poll(()=>commands.map(entry=>entry.operation)).toEqual(['caps','pos','result']);
  expect(commands[1]).toMatchObject({token:17,kp:120,kd:.3,motorId:7,coordinate:'output-rad'});
  expect(Number(commands[1].delta)).toBeCloseTo(Math.PI/180,10);
  expect(Number(commands[1].speed)).toBeCloseTo(5*Math.PI/180,10);
  expect(commands.map(entry=>entry.operation)).toEqual(['caps','pos','result']);
  await expect(page.getByRole('button',{name:'读取原事务结果'})).toBeDisabled();
});
test('缺失标准反馈不会被寄存器显示资格替代',async({page})=>{
  const operations:string[]=[];
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/remote'){const body=route.request().postDataJSON();operations.push(body.operation);return route.fulfill({json:{requestId:1,operation:'caps',result:'capability',detail:'模拟',fields:{schema:'1',token:'7',fresh:'0',static:'1'}}});}
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByRole('button',{name:'读取远程能力'}).click();
  await page.getByText('高级分步操作',{exact:true}).click();
  await page.getByRole('button',{name:'审核并冻结'}).click();
  await expect(page.getByText('缺少新鲜标准反馈，寄存器位置不能授予运动资格')).toBeVisible();
  await expect(page.getByRole('button',{name:'显式提交一次'})).toBeDisabled();
  expect(operations).toEqual(['caps']);
});
test('网关重连后通过设备能力恢复原事务，仅查询不重发',async({page})=>{
  const commands:Record<string,unknown>[]=[];
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK_RECONNECTED',status:'模拟',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/remote'){
      const body=route.request().postDataJSON();commands.push(body);
      return route.fulfill({json:{requestId:45,operation:body.operation,result:body.operation==='caps'?'capability':'completed',detail:'模拟恢复',fields:body.operation==='caps'?{schema:'1',boot:'9',token:'3',fresh:'0',active_id:'41',terminal_id:'39'}:{schema:'1',id:'41',accepted:'1',disabled:'1'}}});
    }
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByRole('button',{name:'读取远程能力'}).click();
  await expect.poll(()=>commands.length).toBe(2);
  expect(commands).toEqual([{operation:'caps'},{operation:'result',transactionId:41}]);
});

