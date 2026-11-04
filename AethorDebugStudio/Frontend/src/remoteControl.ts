/** @file 固定ID7远程控制客户端，输出轴单位；不自动查询、提交或重试。 */
import {formatTorqueNm} from './torqueEnvelope';
export type RemoteOperation = 'acquire' | 'release' | 'pos' | 'pos_load' | 'home' | 'configure' | 'hold' | 'lock' | 'preflight' | 'mode_mit' | 'mode_pos';
/** home的delta为绝对输出轴rad，kp/kd分别为uint32启动号/坐标参考；不作为增益。 */
export interface RemoteDraft { operation: RemoteOperation; delta: number; speed: number; kp: number; kd: number; torque: number; duration: number }
/** 单次板端回执；字段仅为证据，不能单独证明实际停止。 */
export interface RemoteReceipt { requestId: number; operation: string; result: string; fields: Record<string,string>; detail: string }
/** 统一工作流和恢复查询的终态判据；未知原操作采用需要失能证明的保守规则。 */
export function remoteTerminalConfirmed(receipt:RemoteReceipt,operation:RemoteOperation|null):boolean {
  if(!['completed','failed','rejected'].includes(receipt.result))return false;
  if(receipt.result!=='completed'&&receipt.fields.accepted==='0')return true;
  if(receipt.fields.accepted!=='1')return false;
  const requiresDisable=operation===null||receipt.result!=='completed'||
    ['preflight','hold','pos','pos_load','home','mode_mit','mode_pos'].includes(operation);
  return !requiresDisable||receipt.fields.disabled==='1';
}
/** 只解释原事务附带的完整首故障快照，不用故障后的配额账本倒推当时状态。 */
function describeFirstTorqueFault(receipt:RemoteReceipt):string|null {
  const fields=receipt.fields;
  if(receipt.result!=='failed'||fields.accepted!=='1'||fields.reason!=='16'||fields.motor!=='7'||
    !['0','1'].includes(fields.tq_auth)||!['1','2','3','4'].includes(fields.tq_reason))return null;
  if(![fields.tq_used,fields.tq_nm].every(value=>typeof value==='string'&&/^\d+(?:\.\d+)?$/.test(value)))return null;
  const used=Number(fields.tq_used),torque=Number(fields.tq_nm),authorized=fields.tq_auth==='1';
  const limit=fields.tq_limit===undefined?(authorized?5.46:3.5):Number(fields.tq_limit);
  // 原固件以四位小数发送采样；首故障枚举已由板端判定，允许半个末位的舍入误差。
  const decimals=fields.tq_nm.split('.')[1]?.length??0;
  const roundingTolerance=Math.min(0.00005,0.5*10**(-decimals));
  if(fields.tq_limit!==undefined&&!/^\d+(?:\.\d+)?$/.test(fields.tq_limit)||!Number.isFinite(limit)||limit<3.5||limit>6.24||!authorized&&limit!==3.5)return null;
  if(!Number.isFinite(used)||used<0||used>175.616||!Number.isFinite(torque)||torque<0||
    (fields.tq_reason==='1'?authorized||torque+roundingTolerance<=3.5:!authorized)||fields.tq_reason==='2'&&torque+roundingTolerance<=limit)return null;
  const branches:Record<string,string>={'1':'3.50 N·m基线','2':`${formatTorqueNm(limit)} N·m峰值`,'3':'10秒超额窗口','4':'累计超额预算'};
  return `首次转矩保护：${branches[fields.tq_reason]}；故障前实验配额${authorized?'已授权':'未授权'}，账本读数 ${(100*used/175.616).toFixed(1)}%；触发反馈 ${torque.toFixed(3)} N·m（电机输出轴）`;
}
/** 解释板端终态与独立失能证明；失败原因不能被“安全状态未知”覆盖，亦不因此解锁。 */
export function describeRemoteTerminal(receipt:RemoteReceipt,operation:RemoteOperation|null):string {
  const reasons:Record<string,string>={
    '8':'标准反馈缺失或过期','9':'未满足失能/无故障/低速条件','11':'参数或位置超出范围',
    '14':'设备尚未就绪','16':'转矩超限','17':'速度超限','18':'温度超限','19':'反馈数值无效',
  };
  const stages:Record<string,string>={'1':'参数校验','2':'身份识别','3':'模式切换','4':'清故障','5':'使能','6':'运动','7':'保持','8':'失能'};
  const parts=[receipt.result==='failed'?'板端已报告失败':receipt.result==='rejected'?'板端已拒绝请求':'板端已报告终态'];
  const reason=receipt.fields.reason;
  if(reason&&reason!=='0'&&/^\d{1,3}$/.test(reason))parts.push(`${reasons[reason]??'未识别的退出原因'}（原因${reason}）`);
  const torqueFault=describeFirstTorqueFault(receipt);
  if(torqueFault)parts.push(torqueFault);
  const stage=receipt.fields.stage;
  if(stage&&stage!=='0'&&/^\d{1,3}$/.test(stage))parts.push(`${stages[stage]??'阶段'}（阶段码${stage}）`);
  for(const [key,label] of [['error','错误码'],['detail','详细码'],['motor','故障轴ID']] as const){
    const value=receipt.fields[key];
    if(value&&/^\d{1,5}$/.test(value)&&value!=='0')parts.push(`${label}${value}`);
  }
  if(!remoteTerminalConfirmed(receipt,operation)){
    parts.push(receipt.fields.accepted==='1'?'失能未确认，运动保持锁定，仅可读取原事务结果':'准入或失能证明不完整，运动保持锁定');
  }else if(receipt.fields.disabled==='1')parts.push('板端回执已确认失能');
  return parts.join('；');
}
/** 仅接收固件声明的预检意图能力；不把它解释为新鲜反馈或实机运动资格。 */
export function remoteDefersFeedback(operation:RemoteOperation,fields:Record<string,string>):boolean {
  return fields.defer==='1'&&['acquire','release','pos','pos_load','home','mode_mit','mode_pos'].includes(operation);
}
/** 校验草稿数值，设备能力范围由审核时再次约束。 */
export function remoteDraftError(draft: RemoteDraft): string | null {
  if (![draft.delta,draft.speed,draft.kp,draft.kd,draft.torque,draft.duration].every(Number.isFinite)) return '所有参数必须是有限数值';
  if(draft.operation==='home') {
    if(!Number.isInteger(draft.kp)||draft.kp<=0||draft.kp>4294967295||!Number.isInteger(draft.kd)||draft.kd<=0||draft.kd>4294967295||draft.torque!==0||draft.duration!==1000)return '回零需要有效uint32启动号/坐标参考、零前馈及固定1000毫秒终止流程';
  } else if (draft.operation === 'pos_load') {
    if (draft.kp !== 0 || draft.kd !== 0 || draft.torque !== 0 || draft.delta === 0 || draft.duration % 1000 !== 0) return 'POS带载需要非零位移、整秒保持，Kp/Kd/前馈必须为零';
  } else if (draft.kp < 80 || draft.kp > 200 || draft.kd < .2 || draft.kd > .4) return 'Kp范围80–200；Kd范围0.20–0.40';
  if (draft.speed <= 0 || !Number.isInteger(draft.duration) || draft.duration < 1000 || draft.duration > 60000) return '速度必须大于0，时长1000–60000毫秒';
  return null;
}
/** 独立home能力与冻结身份必须逐步一致；绝对目标不能按位移上限直接裁剪。 */
export function remoteHomeError(draft:RemoteDraft,fields:Record<string,string>):string|null {
  if(fields.home!=='1'||!/^\d+$/.test(fields.boot??'')||!/^\d+$/.test(fields.coord??'')||Number(fields.boot)!==draft.kp||Number(fields.coord)!==draft.kd)return '设备未开放回零或启动/坐标参考已变化，请重新设零';
  const limit=Number(fields.load_delta),speed=Number(fields.load_speed);
  if(!Number.isFinite(limit)||limit<=0||!Number.isFinite(speed)||speed<=0||draft.speed>speed)return '回零速度或设备POS范围无效';
  return null;
}
/** 固定同源POST，保留来源标记；HTTP失败不会补发。 */
export async function remoteRequest(body: object): Promise<RemoteReceipt> {
  const response = await fetch('/api/remote', { method: 'POST', headers: {'Content-Type':'application/json','X-Aethor-Client':'studio-v1'}, body: JSON.stringify(body), signal: AbortSignal.timeout(10000) });
  const receipt = await response.json();
  if (!response.ok) throw new Error(receipt.error ?? '远程接口不支持');
  return receipt as RemoteReceipt;
}
/** HTTP失败或刷新后只读恢复网关已保留回执；不触发设备请求。 */
export async function remoteLatest(): Promise<RemoteReceipt|null> {
  const response=await fetch('/api/remote',{signal:AbortSignal.timeout(5000)});
  if(!response.ok)throw new Error('网关不支持远程回执恢复');
  return response.json();
}
