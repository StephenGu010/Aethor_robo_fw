/** @file 会话结束和配额恢复说明的隔离浏览器回归；所有API拦截，禁止真实硬件请求。 */
import {test,expect,type Page} from '@playwright/test';

/** 模拟失败事务及释放状态，记录完整操作序列以排除运动、模式切换或预算恢复。 */
async function fixture(page:Page,{owner='2',disabled='1',uncertainOwner=false,faultRecovery=false}={}){
  const operations:string[]=[];let released=false,allowFinalOwner=!uncertainOwner;
  await page.route('**/api/**',async route=>{
    const request=route.request(),path=new URL(request.url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:true,port:'MOCK',status:'隔离会话测试',receivedAt:new Date().toISOString(),info:{},motors:[],lines:[],debugAvailable:true}});
    if(path==='/api/ports')return route.fulfill({json:[]});
    if(path!=='/api/remote')throw new Error('Unexpected hardware API: '+path);
    if(request.method()==='GET')return route.fulfill({json:null});
    const body=request.postDataJSON();operations.push(body.operation);
    if(body.operation==='caps')return route.fulfill({json:{requestId:2,operation:'caps',result:'capability',detail:'隔离模拟',fields:{token:'7',boot:'12',epoch:released?'10':'9',owner:released?(allowFinalOwner?'0':'1'):owner,active_id:'0',terminal_id:released?'91':'44',fresh:'0',mode:'2',defer:'1',pos:'1',load:'1',static:'1',...(faultRecovery?{authority:released?'0':'3',remote_fault_release:released?'0':'1'}:{})}}});
    if(body.operation==='result')return route.fulfill({json:{requestId:3,operation:'result',result:body.transactionId===44?'failed':'completed',detail:'隔离模拟',fields:{id:String(body.transactionId),accepted:'1',disabled,reason:body.transactionId===44?'16':'0',stage:body.transactionId===44?'6':'0'}}});
    if(body.operation==='release'){released=true;return route.fulfill({json:{requestId:91,operation:'release',result:'queued',detail:'隔离模拟',fields:{}}});}
    if(body.operation==='torque')return route.fulfill({json:{requestId:4,operation:'torque',result:'policy',detail:'隔离模拟',fields:{experimental:'1',motor:'7',axis:'output',continuous:'3.5',peak:'5.46',window_ms:'10000',budget:'175.616',used:'175.616',reason:'5',authorized:'0',can_reset:'0',token:'0',boot:'12',coord:'33'}}});
    throw new Error('Forbidden motor or maintenance operation: '+body.operation);
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'POS 带载测试',exact:true}).click();
  return {operations,allowOwner:()=>{allowFinalOwner=true;}};
}

test('明显结束按钮核对失败失能后只释放一次，不沿用POS参数或开始运动',async({page})=>{
  const {operations}=await fixture(page);
  await page.getByRole('button',{name:'结束会话／释放控制权',exact:true}).click();
  await expect(page.getByText('会话已结束，控制权已释放；未使能、未运动，也未恢复实验配额',{exact:true})).toBeVisible();
  expect(operations).toEqual(['caps','result','caps','release','result','caps']);
  await page.getByRole('button',{name:'结束会话／释放控制权',exact:true}).click();
  await expect(page.getByText('当前控制权未申请，无需重复释放',{exact:true})).toBeVisible();
  expect(operations.filter(operation=>operation==='release')).toHaveLength(1);
});

test('失能未确认只读原结果，结束和运动入口保持锁定',async({page})=>{
  const {operations}=await fixture(page,{disabled:'0'});
  await page.getByRole('button',{name:'结束会话／释放控制权',exact:true}).click();
  await expect(page.getByRole('button',{name:'按参数启动一次 · POS带载移动并保持',exact:true})).toBeDisabled();
  await expect(page.getByRole('button',{name:'结束会话／释放控制权',exact:true})).toBeDisabled();
  expect(operations.every(operation=>['caps','result'].includes(operation))).toBe(true);
});

test('本地占用及原因5如实说明，配额不自动恢复，窄屏无溢出',async({page})=>{
  const {operations}=await fixture(page,{owner:'1'});
  await page.getByRole('button',{name:'结束会话／释放控制权',exact:true}).click();
  await expect(page.getByRole('region',{name:'控制会话'})).toContainText('LCD / 本地保护占用');
  await expect(page.getByText(/当前协议未区分LCD实际控制与本地故障锁定/)).toBeVisible();
  await expect(page.getByText(/POS \/ 静载为1仅表示固件开放该功能/)).toBeVisible();
  await page.getByRole('button',{name:'读取转矩保护策略'}).click();
  await expect(page.getByText(/3.50 N·m 基线退出线/)).toBeVisible();
  await expect(page.getByText(/此码不能确定首故障/)).toBeVisible();
  await expect(page.getByText(/账本100%不等于实际运行满10秒/)).toBeVisible();
  await page.getByRole('checkbox',{name:/本次确认电机已充分冷却/}).check();
  await expect(page.getByRole('button',{name:'冷却后恢复实验配额'})).toBeDisabled();
  expect(operations).toEqual(['caps','torque']);
  await page.screenshot({path:'../artifacts/session-recovery-desktop-20261029.png',fullPage:true});
  await page.setViewportSize({width:390,height:844});
  await expect(page.getByRole('button',{name:'结束会话／释放控制权',exact:true})).toBeVisible();
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1)).toBe(true);
  await page.screenshot({path:'../artifacts/session-recovery-mobile-20261029.png',fullPage:true});
});

test('释放completed后当前控制权未核实，自动和跨页面恢复都不解锁',async({page})=>{
  const transport=await fixture(page,{uncertainOwner:true});
  const start=page.getByRole('button',{name:'按参数启动一次 · POS带载移动并保持',exact:true});
  await page.getByRole('button',{name:'结束会话／释放控制权',exact:true}).click();
  await expect.poll(()=>transport.operations.filter(operation=>operation==='result').length).toBeGreaterThanOrEqual(3);
  await expect(start).toBeDisabled();
  await page.getByRole('navigation').getByRole('button',{name:'手动调试',exact:true}).click();
  await page.getByRole('button',{name:'读取远程能力'}).click();
  await page.getByRole('button',{name:'恢复设备最近终态 #91'}).click();
  await expect(page.getByRole('button',{name:'按参数启动一次 · 下发静载RAM参数',exact:true})).toBeDisabled();
  transport.allowOwner();
  await page.getByRole('button',{name:'读取原事务结果',exact:true}).click();
  await expect(page.getByRole('button',{name:'按参数启动一次 · 下发静载RAM参数',exact:true})).toBeEnabled();
  expect(transport.operations.filter(operation=>operation==='release')).toHaveLength(1);
  expect(transport.operations.every(operation=>['caps','result','release'].includes(operation))).toBe(true);
});

test('新固件明确允许的本地故障只结束原远程会话，不接管LCD或启动POS',async({page})=>{
  const {operations}=await fixture(page,{owner:'1',faultRecovery:true});
  await page.getByRole('button',{name:'结束会话／释放控制权',exact:true}).click();
  await expect(page.getByText('会话已结束，控制权已释放；未使能、未运动，也未恢复实验配额',{exact:true})).toBeVisible();
  expect(operations).toEqual(['caps','result','caps','release','result','caps']);
});
