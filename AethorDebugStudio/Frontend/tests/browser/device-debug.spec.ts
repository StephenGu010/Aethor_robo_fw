/** @file 模拟网关响应验证调试交互；所有设备请求均拦截，不接入COM。 */
import { test, expect } from '@playwright/test';
import AxeBuilder from '@axe-core/playwright';
/** 固定为过期电机反馈，时间每次查询更新，缓存年龄保持真实语义。 */
function staleSnapshot() {
  return { connected:true, port:'COM_TEST', status:'只读连接', error:null, receivedAt:new Date().toISOString(), info:{}, readOnly:true, debugAvailable:true,
    motors:[{ id:7, state:'absent', positionDeg:null, speedDegS:null, torqueNm:null, mosC:null, rotorC:null, fault:null, ageMs:null, receivedAt:new Date().toISOString(), pmaxDeg:null, raw:'ok 8 show motor joint=7 age_ms=2754802' }], lines:[] };
}
test.beforeEach(async ({page}) => {
  await page.route('**/api/state', route => route.fulfill({json:staleSnapshot()}));
  await page.goto('/');
  await page.getByLabel('数据来源').selectOption('live');
});
test('过期空采样显示原因，不绘制伪造零线', async ({page}) => {
  await page.getByRole('navigation').getByRole('button',{name:'实时示波',exact:true}).click();
  await expect(page.getByText('电机反馈已过期',{exact:true})).toBeVisible();
  await expect(page.getByText(/距板端最后一次电机反馈约 2754 秒/)).toBeVisible();
  await expect(page.getByText('等待有效采样',{exact:true})).toBeVisible();
  await expect(page.getByRole('img',{name:'位置反馈曲线'})).toHaveCount(0);
});

test('0x50工程位置和板端估算速度切换坐标，过期后隐藏当前读值', async ({page}) => {
  let expired = false;
  let lastReceivedAt = '';
  const operations: unknown[] = [];
  await page.route('**/api/debug', async route => {
    operations.push(route.request().postDataJSON());
    await route.fulfill({status:409,json:{error:'本测试禁止设备操作'}});
  });
  await page.route('**/api/state', route => {
    const snapshot = staleSnapshot();
    if (!expired) lastReceivedAt = snapshot.motors[0].receivedAt;
    const motor = {...snapshot.motors[0], ageMs:null,
      receivedAt:lastReceivedAt,
      register50Raw:44.8*Math.PI/180, register50AgeMs:5, pmContract:1, pmVelocityRadS:44.8*Math.PI/180,pmBoot:41,pmEpoch:7,
      register51Raw:0, register51AgeMs:5,
      raw:'ok 8 show motor joint=7 age_ms=4294967295 pm_contract=1 reg50_raw=0.7819 reg51_raw=0'};
    return route.fulfill({json:{...snapshot,motors:[motor]}});
  });
  await page.getByRole('navigation').getByRole('button',{name:'实时示波',exact:true}).click();
  await expect(page.getByText('位置寄存器正在更新',{exact:true})).toBeVisible();
  await expect(page.getByRole('img',{name:'位置反馈曲线'})).toBeVisible();
  await expect(page.getByLabel('示波器显示坐标')).toHaveValue('output');
  await expect(page.locator('.readout-grid > div').first()).toContainText('2.333');
  await page.getByLabel('示波器显示坐标').selectOption('flange');
  await expect(page.locator('.readout-grid > div').first()).toContainText('1.000');
  await expect(page.locator('.readout-grid > div').nth(1)).toContainText('板端差分估算速度');
  await expect(page.locator('.readout-grid > div').nth(1)).toContainText('1.000');
  await page.getByRole('button',{name:'速度',exact:true}).click();
  await expect(page.getByRole('img',{name:'速度反馈曲线'})).toBeVisible();
  await expect(page.getByText('速度 · 驱动反馈 / 板端估算',{exact:true})).toBeVisible();
  await page.screenshot({path:'../artifacts/mode2-position-mock.png',fullPage:true});
  expired = true;
  await expect(page.getByText('位置寄存器样本已过期',{exact:true})).toBeVisible();
  await expect(page.locator('.readout-grid > div').first()).not.toContainText('1.000');
  expect(operations).toEqual([]);
});
test('同COM启动号和代次改变时主曲线分段，缺身份只画单点', async ({page}) => {
  let pmBoot: number | null = 41;
  let pmEpoch: number | null = 7;
  await page.route('**/api/state', route => {
    const snapshot=staleSnapshot();
    return route.fulfill({json:{...snapshot,motors:[{...snapshot.motors[0],pmContract:1,pmBoot,pmEpoch,
      register50Raw:1,register50AgeMs:0,pmVelocityRadS:1,raw:'pm_contract=1 reg50_raw=1'}]}});
  });
  await page.getByRole('navigation').getByRole('button',{name:'实时示波',exact:true}).click();
  const curve=page.getByRole('img',{name:'位置反馈曲线'});
  await expect(curve.locator('polyline')).toHaveCount(1);
  pmBoot=42;
  await expect(curve.locator('polyline')).toHaveCount(2);
  pmEpoch=8;
  await expect(curve.locator('polyline')).toHaveCount(3);
  pmBoot=null; pmEpoch=null;
  await expect.poll(()=>curve.locator('circle').count()).toBeGreaterThanOrEqual(2);
  await expect(curve.locator('polyline')).toHaveCount(3);
});

test('调试仅点击发送一次，失能有承托门槛，回执不冒充反馈确认', async ({page}) => {
  const operations: object[] = [];
  await page.route('**/api/debug', async route => {
    const request = route.request().postDataJSON(); operations.push(request);
    await route.fulfill({json:{requestId:91,operation:request.operation,motorId:request.motorId,accepted:true,result:'completed',detail:'设备报告命令完成；仍需新鲜反馈确认状态',lines:['ok 91 bench stop accepted=1','done 91 bench stop result=completed stopped=40'],feedbackConfirmed:false}});
  });
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await expect(page.getByRole('heading',{name:'01 · 连接与检查',exact:true})).toBeVisible();
  await expect(page.getByRole('button',{name:'失能 ID 7',exact:true})).toBeDisabled();
  expect(operations).toEqual([]);
  await page.getByRole('button',{name:'停止 ID 7',exact:true}).click();
  await expect(page.getByRole('region',{name:'最近设备回执'})).toContainText('新鲜失能反馈：未确认');
  expect(operations).toEqual([{operation:'stop',motorId:7,loadSupported:false}]);
  await page.getByLabel('已独立承托负载，允许撤去选中轴的保持力矩').check();
  await expect(page.getByRole('button',{name:'失能 ID 7',exact:true})).toBeEnabled();
  await page.getByLabel('调试电机').selectOption('6');
  await expect(page.getByRole('button',{name:'失能 ID 6',exact:true})).toBeDisabled();
  expect(operations).toHaveLength(1);
});
test('诊断和全轴STOP分开回执，拒绝不自动重试', async ({page}) => {
  const operations: string[] = [];
  await page.route('**/api/debug', async route => {
    const request = route.request().postDataJSON(); operations.push(request.operation);
    await route.fulfill({json:{requestId:92,operation:request.operation,motorId:request.motorId,accepted:false,result:'rejected',detail:'设备拒绝命令：profile',lines:['error 92 bench stop code=profile'],feedbackConfirmed:false}});
  });
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByRole('button',{name:'读取配置与CAN诊断',exact:true}).click();
  await expect(page.getByRole('region',{name:'最近设备回执'})).toContainText('rejected');
  await page.getByRole('button',{name:'STOP 全部轴',exact:true}).click();
  await expect(page.getByRole('region',{name:'最近设备回执'})).toContainText('停止全部轴');
  expect(operations).toEqual(['diagnostics','stop-all']);
});
test('调试区可访问并适应窄屏', async ({page}) => {
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  expect((await new AxeBuilder({page}).withTags(['wcag2a','wcag2aa']).analyze()).violations).toEqual([]);
  for (const width of [320,768,1440]) {
    await page.setViewportSize({width,height:1000});
    expect(await page.evaluate(() => document.documentElement.scrollWidth > window.innerWidth),'width='+width).toBe(false);
  }
  await page.setViewportSize({width:1440,height:1080});
  await page.screenshot({path:'../artifacts/device-debug-mock.png',fullPage:true});
});
