/** @file 健康卡在新快照晚于UI定时器时不误报；全部API拦截，无真实串口。 */
import {test,expect} from '@playwright/test';
test('使用评估时钟接纳新快照，500ms过期及断连仍保持关闭',async({page})=>{
  const base=Date.parse('2026-10-27T08:00:00Z');
  let received=base,age=119,connected=true,holdNext=false,waiting=false;
  let releaseReply:(()=>void)|undefined;
  let posts=0;
  // 固定页面版本，避免开发服务热更新在红绿切换时重载测试会话。
  await page.routeWebSocket('**/*',()=>{});
  await page.clock.install({time:base});await page.clock.pauseAt(base);
  await page.route('**/api/**',async route=>{
    if(route.request().method()!=='GET')posts++;
    if(!route.request().url().endsWith('/api/state'))return route.fulfill({json:{}});
    if(holdNext){holdNext=false;waiting=true;await new Promise<void>(resolve=>{releaseReply=resolve;});}
    return route.fulfill({json:{connected,port:'MOCK',status:'fixture',info:{},lines:[],motors:[{id:7,state:'absent',positionDeg:null,speedDegS:null,torqueNm:null,mosC:null,rotorC:null,fault:null,ageMs:null,pmaxDeg:null,raw:'',receivedAt:new Date(received).toISOString(),register50Raw:4.927679,register50AgeMs:age,pmContract:1,pmBoot:7,pmEpoch:1,coordinateBoot:7,coordinateReference:1}]}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'实时示波',exact:true}).click();
  await expect(page.getByText('位置寄存器正在更新',{exact:true})).toBeVisible();
  holdNext=true;await page.clock.runFor(250);await expect.poll(()=>waiting).toBe(true);
  received=base+260;await page.clock.setSystemTime(received);releaseReply!();
  await expect(page.getByRole('heading',{name:'最近60秒示波',exact:true})).toBeVisible();
  await expect(page.getByText('位置寄存器正在更新',{exact:true})).toBeVisible();
  await expect(page.getByText('位置寄存器样本已过期',{exact:true})).toHaveCount(0);
  await page.screenshot({path:'../artifacts/health-clock-fixture-fixed-20261027.png'});
  age=500;await page.clock.runFor(250);
  await expect(page.getByText('位置寄存器样本已过期',{exact:true})).toBeVisible();
  connected=false;await page.clock.runFor(250);
  await expect(page.getByText('设备未连接',{exact:true})).toBeVisible();
  expect(posts).toBe(0);
});
