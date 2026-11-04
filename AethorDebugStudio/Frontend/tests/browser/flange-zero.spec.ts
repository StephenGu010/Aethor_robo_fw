/** @file 零点和home隔离浏览器回归；全部API被拦截，不访问真实设备。 */
import {test,expect,type Page} from '@playwright/test';

/** 创建可控被动反馈与单次事务，API仅返回内存数据。 */
async function fixture(page:Page,home=true){
  const commands:Record<string,unknown>[]=[];
  const state={connected:true,position:448,reference:4,boot:7,age:0,owner:'0',operation:'',disabled:'1'};
  await page.route('**/api/**',async route=>{
    const path=new URL(route.request().url()).pathname;
    if(path==='/api/state')return route.fulfill({json:{connected:state.connected,port:'MOCK',status:'模拟',info:{},lines:[],debugAvailable:true,motors:[{id:7,state:'disabled',positionDeg:state.position,speedDegS:44.8/19.2032*5,ageMs:state.age,receivedAt:new Date().toISOString(),raw:'',coordinateBoot:state.boot,coordinateReference:state.reference}]}});
    if(path==='/api/remote'){
      const body=route.request().postDataJSON();commands.push(body);
      if(body.operation==='caps')return route.fulfill({json:{requestId:commands.length,operation:'caps',result:'capability',detail:'模拟能力',fields:{boot:String(state.boot),coord:String(state.reference),token:String(commands.length),active_id:'0',owner:state.owner,fresh:'1',mode:'2',home:home?'1':'0',load:'1',load_delta:'1',load_speed:'1',switch:'1'}}});
      if(body.operation==='result'){
        if(state.operation==='acquire')state.owner='2';
        if(state.operation==='release')state.owner='0';
        return route.fulfill({json:{requestId:commands.length,operation:'result',result:'completed',detail:'模拟终态',fields:{id:String(body.transactionId),accepted:'1',disabled:state.disabled}}});
      }
      state.operation=String(body.operation);
      return route.fulfill({json:{requestId:commands.length,operation:body.operation,result:'queued',detail:'模拟入队',fields:{}}});
    }
    return route.fulfill({json:path==='/api/ports'?[]:{}});
  });
  await page.goto('/');await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'单轴静载',exact:true}).click();
  return {state,commands};
}

test('设置/清除不发动作，跨页共享相对角度且速度不偏移，重启失效',async({page})=>{
  const {state,commands}=await fixture(page);
  await page.getByRole('button',{name:'读取远程能力',exact:true}).click();
  await page.getByRole('button',{name:'当前位置设为零点',exact:true}).click();
  await expect(page.getByLabel('相对零点角度',{exact:true})).toContainText('0.000');
  state.position=492.8;
  await expect(page.getByLabel('相对零点角度',{exact:true})).toContainText('1.000');
  await expect(page.getByLabel('当前法兰速度',{exact:true})).toContainText('5.000');
  await page.getByRole('navigation').getByRole('button',{name:'POS 带载测试',exact:true}).click();
  await expect(page.getByLabel('相对零点角度',{exact:true})).toContainText('1.000');
  state.boot=8;
  await expect(page.getByLabel('零点状态',{exact:true})).toContainText('未设置');
  expect(commands.map(command=>command.operation)).toEqual(['caps']);
});

test('home独立审核绑定固定绝对目标，准备时位置变化不改目标且只提交一次',async({page})=>{
  const {state,commands}=await fixture(page);
  await page.getByRole('button',{name:'读取远程能力',exact:true}).click();
  await page.getByRole('button',{name:'当前位置设为零点',exact:true}).click();
  state.position=492.8;
  await expect(page.getByLabel('相对零点角度',{exact:true})).toContainText('1.000');
  await page.getByRole('button',{name:'审核回到零点（运动）',exact:true}).click();
  await expect(page.getByText(/回零审核：.*负方向.*1.000/)).toBeVisible();
  expect(commands.filter(command=>command.operation==='home')).toHaveLength(0);
  state.position=537.6;
  await page.getByRole('button',{name:'显式执行回零一次',exact:true}).click();
  await expect.poll(()=>commands.filter(command=>command.operation==='home').length).toBe(1);
  await expect.poll(()=>commands.filter(command=>command.operation==='release').length).toBe(1);
  const submitted=commands.find(command=>command.operation==='home')!;
  expect(Number(submitted.delta)).toBeCloseTo(448/19.2032*Math.PI/180,10);
  expect(submitted.kp).toBe(7);expect(submitted.kd).toBe(4);expect(submitted.duration).toBe(1000);
});

test('旧固件回零关闭，反馈过期保留零点并禁回零，参考变更作废审核',async({page})=>{
  const {state}=await fixture(page,false);
  await page.getByRole('button',{name:'读取远程能力',exact:true}).click();
  await page.getByRole('button',{name:'当前位置设为零点',exact:true}).click();
  await expect(page.getByRole('button',{name:'审核回到零点（运动）',exact:true})).toBeDisabled();
  state.age=5000;
  await expect(page.getByLabel('相对零点角度',{exact:true})).toContainText('—');
  await expect(page.getByLabel('零点状态',{exact:true})).toContainText('已设置');
  state.reference=5;
  await expect(page.getByLabel('零点状态',{exact:true})).toContainText('未设置');
});

test('回零审核遇到过期反馈禁提交，恢复后参考变化即作废；清零与刷新均不发动作',async({page})=>{
  const {state,commands}=await fixture(page);
  await page.getByRole('button',{name:'读取远程能力',exact:true}).click();
  await page.getByRole('button',{name:'当前位置设为零点',exact:true}).click();
  await page.getByRole('button',{name:'审核回到零点（运动）',exact:true}).click();
  state.age=5000;
  await expect(page.getByRole('button',{name:'显式执行回零一次',exact:true})).toBeDisabled();
  await expect(page.getByLabel('零点状态',{exact:true})).toContainText('已设置');
  state.age=0;
  await expect(page.getByRole('button',{name:'显式执行回零一次',exact:true})).toBeEnabled();
  state.reference++;
  await expect(page.getByRole('button',{name:'显式执行回零一次',exact:true})).toHaveCount(0);
  await page.getByRole('button',{name:'读取远程能力',exact:true}).click();
  await page.getByRole('button',{name:'当前位置设为零点',exact:true}).click();
  await page.getByRole('button',{name:'清除零点',exact:true}).click();
  await expect(page.getByLabel('零点状态',{exact:true})).toContainText('未设置');
  await page.getByRole('button',{name:'当前位置设为零点',exact:true}).click();
  await expect(page.getByLabel('零点状态',{exact:true})).toContainText('已设置');
  await page.reload();
  await page.getByLabel('数据来源').selectOption('live');
  await page.getByRole('navigation').getByRole('button',{name:'POS 带载测试',exact:true}).click();
  await expect(page.getByLabel('零点状态',{exact:true})).toContainText('未设置');
  expect(commands.every(command=>command.operation==='caps')).toBe(true);
});

test('回零失能未确认跨页保持锁定，零点和新动作均不能重设或重发',async({page})=>{
  const {state,commands}=await fixture(page);
  await page.getByRole('button',{name:'读取远程能力',exact:true}).click();
  await page.getByRole('button',{name:'当前位置设为零点',exact:true}).click();
  await page.getByRole('button',{name:'审核回到零点（运动）',exact:true}).click();
  state.disabled='0';
  await page.getByRole('button',{name:'显式执行回零一次',exact:true}).click();
  await expect(page.getByRole('alert')).toContainText('上次启动结果尚未确认');
  await page.getByRole('navigation').getByRole('button',{name:'POS 带载测试',exact:true}).click();
  await expect(page.getByRole('button',{name:'当前位置设为零点',exact:true})).toBeDisabled();
  await expect(page.getByRole('button',{name:'清除零点',exact:true})).toBeDisabled();
  await expect(page.getByRole('button',{name:'按参数启动一次 · POS带载移动并保持',exact:true})).toBeDisabled();
  expect(commands.filter(command=>command.operation==='home')).toHaveLength(1);
  expect(commands.filter(command=>command.operation==='acquire')).toHaveLength(1);
});
