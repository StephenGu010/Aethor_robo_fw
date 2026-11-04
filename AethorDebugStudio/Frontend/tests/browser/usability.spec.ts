/** @file 新手手册与真实控制信息顺序验收；所有API均拦截，绝不连接设备。 */
import {test,expect,type Page} from '@playwright/test';
import AxeBuilder from '@axe-core/playwright';

/** 提供只读模拟连接并拒绝测试未声明的动作。 */
async function mockDevice(page:Page){
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'模拟',info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/remote')return route.fulfill({json:{requestId:1,operation:'caps',result:'capability',detail:'模拟只读能力',fields:{token:'1',boot:'7',owner:'0',mode:'1',fresh:'0',static:'1',load:'1',active_id:'0'}}});
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
}
test.beforeEach(async({page})=>{await mockDevice(page);await page.goto('/');});

test('已连接弹窗初始焦点与正反Tab均留在窗口内',async({page})=>{
  await page.getByLabel('数据来源').selectOption('live');
  const connection=page.getByRole('button',{name:'MOCK',exact:true});
  await connection.focus();await page.keyboard.press('Enter');
  const dialog=page.getByRole('dialog');await expect(dialog).toBeVisible();
  await expect.poll(()=>dialog.evaluate(element=>element.contains(document.activeElement))).toBe(true);
  for(const key of ['Tab','Tab','Shift+Tab','Shift+Tab']){
    await page.keyboard.press(key);
    expect(await dialog.evaluate(element=>element.contains(document.activeElement))).toBe(true);
  }
  await page.keyboard.press('Escape');await expect(connection).toBeFocused();
});

test('总览提供手册入口，目录与搜索可解释零点和超额余量',async({page})=>{
  await expect(page.getByText('开始使用',{exact:true})).toBeVisible();
  await page.getByRole('button',{name:'阅读新手手册',exact:true}).click();
  await expect(page.getByRole('navigation',{name:'手册目录'})).toBeVisible();
  await page.getByRole('searchbox',{name:'搜索操作手册'}).fill('零点');
  await expect(page.getByRole('heading',{name:'设零与回零'})).toBeVisible();
  await expect(page.getByText('刷新、重连、主控重启或坐标参考变化后，软件零点失效。',{exact:true})).toBeVisible();
  await page.getByRole('searchbox',{name:'搜索操作手册'}).fill('不存在的关键词');
  await expect(page.getByText('没有匹配章节，请换一个关键词。')).toBeVisible();
  await page.getByRole('button',{name:'清除搜索'}).click();
  await expect(page.getByRole('heading',{name:'联合保护与短时超额余量'})).toBeVisible();
});

test('真实静载按参数保护启动结果排序，高级操作默认折叠且可键盘展开',async({page})=>{
  await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation',{name:'主导航'}).getByRole('button',{name:'单轴静载',exact:true}).click();
  const sections=page.locator('[data-workflow-step]');
  await expect(sections).toHaveCount(5);
  expect(await sections.evaluateAll(elements=>elements.map(element=>element.getAttribute('data-workflow-step')))).toEqual(['check','parameters','protection','start','results']);
  await expect(page.getByRole('button',{name:'按参数启动一次 · 单次静载保持',exact:true})).toBeVisible();
  await expect(page.getByLabel('法兰相对位移 °',{exact:true})).toHaveCount(0);
  await expect(page.getByRole('button',{name:'审核并冻结'})).toBeHidden();
  const advanced=page.getByText('高级分步操作',{exact:true});await advanced.focus();await page.keyboard.press('Enter');
  await expect(page.getByRole('button',{name:'审核并冻结'})).toBeVisible();
  await expect(page.getByRole('button',{name:'结束会话／释放控制权'})).toBeVisible();
});

test('顶部显示本次读取的控制方并将过期缓存标明',async({page})=>{
  await page.clock.install();
  await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation',{name:'主导航'}).getByRole('button',{name:'单轴静载',exact:true}).click();
  await page.getByRole('button',{name:'读取远程能力',exact:true}).click();
  const status=page.getByRole('region',{name:'当前连接与控制状态'});
  await expect(status).toContainText('控制权：未申请');
  await page.clock.fastForward(31000);
  await expect(status).toContainText('控制权缓存已过期');
  await page.getByRole('navigation',{name:'主导航'}).getByRole('button',{name:'新手手册',exact:true}).click();
  await expect(status).toContainText('控制权：待读取设备能力');
});

for(const width of [320,768,1024,1440,1920])test(`手册和静载在${width}px不产生横向溢出`,async({page})=>{
  await page.setViewportSize({width,height:1000});
  if(width<=768)await page.getByRole('button',{name:'打开导航'}).click();
  await page.getByRole('navigation',{name:'主导航'}).getByRole('button',{name:'新手手册',exact:true}).click();
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  await page.screenshot({path:`../artifacts/usability-manual-${width}.png`,fullPage:true});
  await page.getByLabel('数据来源').selectOption('live');
  if(width<=768)await page.getByRole('button',{name:'打开导航'}).click();
  await page.getByRole('navigation',{name:'主导航'}).getByRole('button',{name:'单轴静载',exact:true}).click();
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  await page.screenshot({path:`../artifacts/usability-static-${width}.png`,fullPage:true});
});

test('新手手册及真实静载没有axe严重可访问性问题',async({page})=>{
  await page.getByRole('button',{name:'阅读新手手册',exact:true}).click();
  expect((await new AxeBuilder({page}).analyze()).violations).toEqual([]);
  await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation',{name:'主导航'}).getByRole('button',{name:'单轴静载',exact:true}).click();
  expect((await new AxeBuilder({page}).analyze()).violations).toEqual([]);
});
