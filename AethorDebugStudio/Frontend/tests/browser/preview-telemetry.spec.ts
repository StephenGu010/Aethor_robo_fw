/** @file 总览/流程实时预览工程单位回归；全部网关快照均拦截，不接入硬件。 */
import { test, expect } from '@playwright/test';
test('总览工程单位与过期隐藏保持正确，实时POS导航进入专用控制',async({page})=>{
  let expired=false;
  await page.route('**/api/state',route=>route.fulfill({json:{connected:true,port:'COM_TEST',status:'只读测试',error:null,info:{},readOnly:true,lines:[],
    motors:[{id:7,state:'disabled',positionDeg:19.2032,speedDegS:1,torqueNm:null,mosC:null,rotorC:null,fault:null,ageMs:expired?2000:0,receivedAt:new Date().toISOString(),pmaxDeg:null,raw:'test'}]}}));
  await page.goto('/');
  await page.getByLabel('数据来源').selectOption('live');
  await expect(page.locator('.motor-table tr.active-row')).toContainText('驱动位置');
  await expect(page.locator('.motor-table tr.active-row')).toContainText('1.00');
  await expect(page.locator('.motor-table tr.active-row')).not.toContainText('19.20');
  await expect(page.getByText('电机输出轴 · ° · 最近60秒',{exact:true})).toBeVisible();
  expired=true;
  await expect(page.locator('.motor-table tr.active-row')).toContainText('反馈过期');
  await expect(page.locator('.motor-table tr.active-row')).not.toContainText('1.00');
  await page.getByRole('navigation').getByRole('button',{name:'POS 带载测试',exact:true}).click();
  await expect(page.getByLabel('本次操作')).toHaveValue('pos_load');
  await expect(page.getByLabel('法兰相对位移 °',{exact:true})).toBeVisible();
  await expect(page.getByRole('button',{name:'按参数启动一次 · POS带载移动并保持',exact:true})).toBeVisible();
  await expect(page.getByText('远程未接入',{exact:true})).toHaveCount(0);
});
