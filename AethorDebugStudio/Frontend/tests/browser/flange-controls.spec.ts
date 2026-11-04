/** @file 法兰端参数、审核与反馈一致性；全接口拦截，禁止访问电机。 */
import {test,expect} from '@playwright/test';
const ratio=44.8/19.2032;

test('POS带载输入法兰度和速度，协议只反向换算一次',async({page})=>{
  const commands:Record<string,unknown>[]=[];
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/remote'){
      const body=route.request().postDataJSON();commands.push(body);
      return route.fulfill({json:{requestId:commands.length,operation:body.operation,result:body.operation==='caps'?'capability':body.operation==='result'?'completed':'queued',detail:'模拟',fields:body.operation==='caps'?{token:'7',fresh:'1',mode:'2',owner:'2',load:'1',load_delta:'1',load_speed:'1'}:{id:'2',accepted:'1',disabled:'1'}}});
    }
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'POS 带载测试',exact:true}).click();
  await page.getByLabel('法兰相对位移 °',{exact:true}).fill('10');
  await page.getByLabel('法兰速度 °/s',{exact:true}).fill('5');
  await page.getByLabel('保持时间 s',{exact:true}).fill('2');
  await page.getByRole('button',{name:'读取远程能力'}).click();
  await page.getByText('高级分步操作',{exact:true}).click();
  await page.getByRole('button',{name:'审核并冻结'}).click();
  await expect(page.getByText(/已冻结：.*法兰δ=10.000°，v=5.000°\/s/)).toBeVisible();
  await page.getByRole('button',{name:'显式提交一次'}).click();
  await expect.poll(()=>commands.map(command=>command.operation)).toEqual(['caps','pos_load','result']);
  expect(commands[1].coordinate).toBe('output-rad');
  expect(Number(commands[1].delta)).toBeCloseTo(10*Math.PI/180*ratio,10);
  expect(Number(commands[1].speed)).toBeCloseTo(5*Math.PI/180*ratio,10);
  expect(commands[1].duration).toBe(2000);
});

test('静载反馈统一到法兰，增益维持输出轴口径且不会自动发动作',async({page})=>{
  const writes:string[]=[];
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(route.request().method()==='POST')writes.push(path);
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',info:{},lines:[],debugAvailable:true,motors:[{id:7,state:'disabled',positionDeg:44.8*180/Math.PI,speedDegS:ratio*5,torqueNm:.2,mosC:30,rotorC:25,fault:0,ageMs:0,receivedAt:new Date().toISOString(),raw:'age_ms=0'}]}});
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  await expect(page.getByLabel('当前法兰角度',{exact:true})).toContainText('57.296');
  await expect(page.getByLabel('当前法兰速度',{exact:true})).toContainText('5.000');
  await expect(page.getByLabel('输出轴Kp N·m/rad',{exact:true})).toHaveValue('120');
  expect(writes).toEqual([]);
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByLabel('本次操作').selectOption('pos');
  await expect(page.getByLabel('输出轴相对位移 °',{exact:true})).toBeVisible();
});
