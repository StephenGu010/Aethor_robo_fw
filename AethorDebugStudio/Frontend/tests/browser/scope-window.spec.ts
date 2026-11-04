/** @file 最近60秒实时时钟浏览器回归；全部网关响应拦截，不访问串口。 */
import {test,expect} from '@playwright/test';
test('实时示波滚动最近60秒，断流超过窗口后清空旧曲线',async({page})=>{
  const base=Date.parse('2026-10-27T06:00:00Z');
  let received=base;
  await page.clock.install({time:base});
  await page.route('**/api/**',async route=>{
    if(!route.request().url().endsWith('/api/state'))return route.fulfill({json:{}});
    return route.fulfill({json:{connected:true,port:'COM4',status:'test',error:null,receivedAt:new Date(received).toISOString(),info:{},motors:[{id:7,state:'disabled',positionDeg:44.8,speedDegS:0,torqueNm:0,mosC:25,rotorC:25,fault:0,ageMs:0,receivedAt:new Date(received).toISOString(),pmaxDeg:null,raw:''}],lines:[],readOnly:true}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'实时示波',exact:true}).click();
  await expect(page.getByRole('img',{name:'位置反馈曲线'})).toBeVisible();
  received=base+70000;await page.clock.fastForward(70000);
  await expect(page.getByRole('heading',{name:'最近60秒示波',exact:true})).toBeVisible();
  await expect.poll(async()=>{
    const axis=(await page.getByRole('img',{name:'位置反馈曲线'}).locator('text').allTextContents()).filter(text=>text.endsWith('s')).map(parseFloat);
    return axis.length===5&&axis[0]>=10&&Math.abs(axis[4]-axis[0]-60)<.11;
  }).toBe(true);
  await page.clock.fastForward(61000);
  await expect(page.getByText('等待有效采样',{exact:true})).toBeVisible();
  await expect(page.getByRole('img',{name:'位置反馈曲线'})).toHaveCount(0);
});
