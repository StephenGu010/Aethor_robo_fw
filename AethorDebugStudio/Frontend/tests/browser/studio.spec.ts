/** @file 关键用户流程、来源隔离、导入导出与响应式浏览器测试。 */
import { test, expect } from '@playwright/test';
import AxeBuilder from '@axe-core/playwright';
import { mkdir } from 'node:fs/promises';
import { execFileSync } from 'node:child_process';
test.beforeEach(async({page})=>{await page.goto('/');});
test('九个页面均可打开且无运行时异常',async({page})=>{
  const errors:string[]=[];page.on('pageerror',(error)=>errors.push(error.message));
  for(const title of ['手动调试','单轴静载','POS 带载测试','七轴调试','实时示波','驱动与维护','记录与诊断','实验与扩展','工作台总览']){
    await page.getByRole('navigation').getByRole('button',{name:title,exact:true}).click();
    await expect(page.getByRole('heading',{level:1})).toBeVisible();
  }
  expect(errors).toEqual([]);
  await mkdir('../artifacts',{recursive:true});
  await page.evaluate(()=>window.scrollTo(0,0));
  await page.screenshot({path:'../artifacts/overview-desktop.png',fullPage:true});
});
test('无效参数拦截，审核冻结，单次完成与即时停止',async({page})=>{
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  await page.locator('#torqueFfNm').fill('4');
  await expect(page.getByRole('button',{name:'预检与审核'})).toBeDisabled();
  await expect(page.getByRole('alert')).toContainText('前馈力矩');
  await page.locator('#torqueFfNm').fill('0');
  await page.getByRole('button',{name:'预检与审核'}).click();
  await expect(page.getByRole('dialog')).toContainText('不会打开串口');
  await expect(page.locator('#torqueFfNm')).toBeDisabled();
  await page.getByRole('button',{name:'开始离线演示'}).click();
  await expect(page.getByRole('button',{name:'立即停止演示'})).toBeVisible();
  await page.getByRole('button',{name:'立即停止演示'}).click();
  await expect(page.getByRole('heading',{name:'演示已停止'})).toBeVisible();
  await expect(page.getByRole('button',{name:'停止演示',exact:true})).toBeDisabled();
  await page.getByRole('button',{name:'预检与审核'}).click();
  await page.getByRole('button',{name:'开始离线演示'}).click();
  await expect(page.getByRole('heading',{name:'演示流程完成'})).toBeVisible({timeout:6000});
  await page.evaluate(()=>window.scrollTo(0,0));
  await page.screenshot({path:'../artifacts/static-workflow.png',fullPage:true});
});
test('导入记录可回放与导出，坏记录明确提示',async({page})=>{
  await page.getByRole('navigation').getByRole('button',{name:'记录与诊断',exact:true}).click();
  const recording={schema:'aethor-studio-recording/v1',name:'浏览器验证样本',source:'device',coordinate:'native',motorId:1,createdAt:'2026-10-26T00:00:00Z',samples:[0,100,200].map(timeMs=>({timeMs,positionDeg:0,targetDeg:null,speedDegS:0,torqueNm:0,mosC:null,rotorC:null})),events:[]};
  await page.getByLabel('导入记录文件').setInputFiles({name:'record.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify(recording))});
  await expect(page.getByLabel('数据来源')).toHaveValue('replay');
  await expect(page.getByText('浏览器验证样本',{exact:true})).toBeVisible();
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  await expect(page.getByLabel('目标电机')).toHaveValue('1');
  await page.getByRole('navigation').getByRole('button',{name:'实时示波',exact:true}).click();
  await page.getByRole('button',{name:'播放回放'}).click();
  await expect(page.getByLabel('回放进度')).toHaveValue('200');
  const download=page.waitForEvent('download');await page.getByRole('button',{name:'导出 CSV'}).click();expect((await download).suggestedFilename()).toBe('aethor-signals.csv');
  await page.getByRole('navigation').getByRole('button',{name:'记录与诊断',exact:true}).click();
  await page.getByLabel('导入记录文件').setInputFiles({name:'bad.json',mimeType:'application/json',buffer:Buffer.from('{}')});
  await expect(page.getByRole('status')).toContainText('不支持的记录格式');
});
test('真实模式不使用演示数值且不能启动动作',async({page})=>{
  await page.route('**/api/state',route=>route.fulfill({json:{connected:false,port:null,status:'未连接',error:null,receivedAt:null,info:{},motors:[],lines:[],readOnly:true}}));
  await page.getByLabel('数据来源').selectOption('live');
  await expect(page.getByText('真实模式 · 未连接设备')).toBeVisible();
  await expect(page.getByRole('button',{name:'STOP 全部轴'})).toBeDisabled();
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  await expect(page.getByLabel('本次操作')).toHaveValue('hold');
  await expect(page.getByRole('button',{name:'按参数启动一次 · 单次静载保持',exact:true})).toBeDisabled();
  await expect(page.getByRole('button',{name:'开始离线演示'})).toHaveCount(0);
});
test('只读连接仅枚举，未点击连接不请求设备连接',async({page})=>{
  const mutations:string[]=[];
  page.on('request',request=>{if(request.method()==='POST')mutations.push(request.url());});
  await page.route('**/api/ports',route=>route.fulfill({json:['COM99']}));
  await page.getByRole('button',{name:'连接设备'}).click();
  await expect(page.getByRole('dialog')).toContainText('不使能、不运动、不写参数');
  await expect(page.getByLabel('USB CDC 串口')).toHaveValue('COM99');
  await page.getByRole('button',{name:'取消',exact:true}).click();
  expect(mutations).toEqual([]);
});
test('320、768、1440宽度无整页横向溢出',async({page})=>{
  for(const width of [320,768,1440]){
    await page.setViewportSize({width,height:960});
    if(width===320){await page.getByRole('button',{name:'打开导航'}).click();await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();}
    const overflow=await page.evaluate(()=>document.documentElement.scrollWidth>window.innerWidth);
    expect(overflow,'width='+width).toBe(false);
    await page.screenshot({path:'../artifacts/layout-'+width+'.png',fullPage:true,animations:'disabled'});
  }
});
test('总览和参数页满足基础可访问性规则',async({page})=>{
  const violations=(await new AxeBuilder({page}).withTags(['wcag2a','wcag2aa']).analyze()).violations;
  expect(violations.map(violation=>({id:violation.id,nodes:violation.nodes.map(node=>node.target)}))).toEqual([]);
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  const form=(await new AxeBuilder({page}).withTags(['wcag2a','wcag2aa']).analyze()).violations;
  expect(form.map(violation=>({id:violation.id,nodes:violation.nodes.map(node=>node.target)}))).toEqual([]);
});
test('只读快照可绘图和导出，已连接会话不能静默切换来源',async({page})=>{
  await page.route('**/api/state',route=>route.fulfill({json:{connected:true,port:'COM_TEST',status:'只读连接',error:null,receivedAt:new Date().toISOString(),info:{},motors:[{id:7,state:'disabled',positionDeg:0,speedDegS:0,torqueNm:0,mosC:25,rotorC:24,fault:0,ageMs:0,receivedAt:new Date().toISOString(),pmaxDeg:null,raw:'合成测试响应'}],lines:[],readOnly:true}}));
  await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'实时示波',exact:true}).click();
  await expect(page.getByRole('img',{name:'位置反馈曲线'})).toBeVisible();
  await expect(page.getByText('等待有效采样')).toHaveCount(0);
  await expect(page.getByRole('button',{name:'导出 CSV'})).toBeEnabled();
  await page.getByLabel('数据来源').selectOption('demo');
  await expect(page.getByLabel('数据来源')).toHaveValue('live');
  await expect(page.getByRole('status')).toContainText('先断开只读连接');
});
test('生产构建经.NET网关提供页面和本地资源',async({page})=>{
  const errors:string[]=[];
  page.on('pageerror',error=>errors.push(error.message));
  page.on('console',message=>{if(message.type()==='error')errors.push(message.text());});
  const response=await page.goto('http://127.0.0.1:5179/');
  expect(response?.status()).toBe(200);
  await expect(page.getByRole('heading',{level:1,name:'调试工作台'})).toBeVisible();
  await expect(page.getByRole('heading',{name:'ID7 静载窗口已分离'})).toBeVisible();
  await expect(page.getByRole('img',{name:'位置反馈曲线'})).toBeVisible();
  expect(errors).toEqual([]);
  const httpVerification=execFileSync(process.execPath,['../scripts/verify-http.mjs'],{encoding:'utf8'});
  expect(httpVerification).toContain('HTTP_TESTS passed=16 failed=0');
  await page.screenshot({path:'../artifacts/production-overview.png',fullPage:true});
});
test('新版静载分离窗口、默认增益与研究候选',async({page})=>{
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  await expect(page.locator('#kp')).toHaveValue('80');
  await expect(page.locator('#kd')).toHaveValue('0.2');
  await expect(page.getByRole('heading',{name:'三个窗口，各管一件事'})).toBeVisible();
  await expect(page.getByText('启动与前馈接管检查法兰0.214°；保持保护法兰0.429°。显示换算不改变原保护。')).toBeVisible();
  await page.getByRole('button',{name:'120 / 0.4 离线候选 · 未部署',exact:true}).click();
  await expect(page.locator('#kp')).toHaveValue('80');
  await expect(page.locator('#kd')).toHaveValue('0.2');
  await expect(page.getByText('0.006',{exact:false}).first()).toBeVisible();
  await page.getByLabel('预算位置误差').fill('1.5');
  await expect(page.getByText('名义预算已用尽',{exact:true})).toBeVisible();
  await page.evaluate(()=>window.scrollTo(0,0));
  await page.screenshot({path:'../artifacts/static-redesign-desktop.png',fullPage:true,animations:'disabled'});
});
test('新版静载在320/768/1440/1920下无整页溢出',async({page})=>{
  for(const width of [320,768,1440,1920]){
    await page.setViewportSize({width,height:1080});
    if(width<=768)await page.getByRole('button',{name:'打开导航'}).click();
    await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
    expect(await page.evaluate(()=>document.documentElement.scrollWidth>window.innerWidth),'width='+width).toBe(false);
    await page.screenshot({path:'../artifacts/static-redesign-'+width+'.png',fullPage:true,animations:'disabled'});
  }
});
