/** @file Parse only the explicitly identified ID7 experimental output-shaft policy; never infers hardware qualification. */
import type {RemoteReceipt} from './remoteControl';
/** Board-reported software exposure allowance and its separate one-use maintenance identity. */
export interface TorqueEnvelopePolicy {
  continuousNm:number;peakNm:number;windowSeconds:number;capacity:number;used:number;
  authorized:boolean;canReset:boolean;token:number;boot:number;reference:number;
  reason:number|null;
  configurable:boolean;peakMilliNm:number;blockers:number|null;
}
/** Reject incomplete, wrong-axis or incompatible firmware fields instead of inventing protection values. */
export function parseTorqueEnvelope(receipt:RemoteReceipt):TorqueEnvelopePolicy|null {
  const fields=receipt.fields;
  if(receipt.operation!=='torque'||receipt.result!=='policy'||fields.experimental!=='1'||fields.motor!=='7'||fields.axis!=='output')return null;
  const continuousNm=Number(fields.continuous),peakNm=Number(fields.peak),windowMs=Number(fields.window_ms);
  const capacity=Number(fields.budget),used=Number(fields.used),token=Number(fields.token),boot=Number(fields.boot),reference=Number(fields.coord);
  const reason=fields.reason===undefined?null:Number(fields.reason);
  const configurable=fields.policy_v==='2';
  const peakMilliNm=configurable?Number(fields.peak_mnm):5460;
  const blockers=configurable?Number(fields.blockers):null;
  if(fields.policy_v!==undefined&&!configurable)return null;
  if(configurable&&(fields.min_mnm!=='3500'||fields.max_mnm!=='6240'||
    !/^\d+$/.test(fields.peak_mnm??'')||peakMilliNm<3500||peakMilliNm>6240||peakNm!==peakMilliNm/1000||
    !/^\d+$/.test(fields.blockers??'')||blockers===null||!Number.isInteger(blockers)||blockers<0||blockers>2047||
    (fields.can_reset==='1')!==(blockers===0)||fields.reset_temp!=='35'||fields.evidence_ms!=='5000'||fields.experiment_fresh_ms!=='20'))return null;
  if(continuousNm!==3.5||(!configurable&&peakNm!==5.46)||windowMs!==10000||capacity!==175.616||!Number.isFinite(used)||used<0||used>capacity||
    ![fields.continuous,fields.peak,fields.window_ms,fields.budget,fields.used].every(value=>typeof value==='string'&&/^\d+(?:\.\d+)?$/.test(value))||
    ![fields.token,fields.boot,fields.coord].every(value=>typeof value==='string'&&/^\d+$/.test(value))||
    ![token,boot,reference].every(value=>Number.isInteger(value)&&value>=0&&value<=0xffffffff)||boot===0||reference===0||
    !['0','1'].includes(fields.authorized)||!['0','1'].includes(fields.can_reset)||fields.can_reset==='1'&&token===0||
    reason!==null&&(!/^\d+$/.test(fields.reason)||!Number.isInteger(reason)||reason<0||reason>255))return null;
  return {continuousNm,peakNm,windowSeconds:windowMs/1000,capacity,used,authorized:fields.authorized==='1',canReset:fields.can_reset==='1',token,boot,reference,reason,configurable,peakMilliNm,blockers};
}

/** 翻译板端配额账本原因；旧固件的统一撤销码不能用来认定测试首故障。 */
export function describeTorqueEnvelopeReason(policy:TorqueEnvelopePolicy):string {
  if(policy.authorized)return '实验配额可用；仍须通过本次测试的板端预检。';
  const reasons:Record<number,string>={
    0:'主控重启后默认关闭，或尚未执行本次冷却恢复。',
    1:'实验配额未授权时触发3.5 N·m基线保护。',
    2:`发生瞬时转矩保护，剩余试验配额已撤销。当前所选退出线为${formatTorqueNm(policy.peakNm)} N·m；原测试的实际退出线请查看首故障记录。`,
    3:'连续超额窗口达到约10秒，实验配额已撤销。',
    4:'累计超额预算达到上限，实验配额已撤销。',
    5:'反馈或服务时序异常，或其他故障退出后统一撤销配额。此码不能确定首故障，请核对原事务退出原因。',
    6:'控制时钟或反馈时间回退，实验配额已撤销。',
    7:'反馈数值或累计预算无效，实验配额已撤销。',
  };
  return policy.reason===null?'板端未提供配额细分原因；请查看原事务终态。':
    reasons[policy.reason]??`未识别的板端原因码 ${policy.reason}；保持实验配额关闭。`;
}

/** 按毫牛米配置精度显示，避免把3.501的实际保护线显示成3.50。 */
export function formatTorqueNm(value:number):string {
  return value.toFixed(Math.round(value*1000)%10===0?2:3);
}

/** 解释固件逐项门禁；仅提示下一步，不替用户执行模式切换或失能。 */
export function torqueEnvelopeBlockers(policy:TorqueEnvelopePolicy):string[] {
  if(policy.blockers===null)return policy.canReset?[]:['旧固件未提供逐项原因；请核对控制权、冷却和新鲜失能反馈。'];
  const conditions:readonly [number,string][]=[
    [1,'控制权未释放：先结束原会话；LCD占用时须从LCD释放。'],
    [2,'设备仍有执行或待处理请求：等待原事务结束并读取结果。'],
    [4,'上次失能尚未确认：读取原测试结果，不能重复启动。'],
    [8,'电机身份或当前连接身份未确认：检查供电、识别状态和USB连接。'],
    [16,'主控未处于无故障的失能空闲状态：先处理原测试结果。'],
    [32,'缺少5秒内标准反馈：位置寄存器不能代替失能和温度反馈。'],
    [64,'驱动未确认失能或仍有故障：先核查驱动故障和失能反馈。'],
    [128,'速度未满足静止条件：等待机械臂稳定。'],
    [256,'转矩无效或超过3.50 N·m：核对独立承托和反馈。'],
    [512,'温度无效或任一路超过35℃：充分冷却后重新读取。'],
    [1024,'当前固件配置不支持此试验维护。'],
  ];
  return conditions.filter(([bit])=>(policy.blockers!&bit)!==0).map(([,description])=>description);
}

/** 计算给定恒定转矩下的软件剩余配额时间，不作为电机允许持续时间。 */
export function torqueAllowanceSeconds(peakMilliNm:number,used=0):number {
  const peakNm=peakMilliNm/1000;
  if(!Number.isInteger(peakMilliNm)||peakMilliNm<3500||peakMilliNm>6240||!Number.isFinite(used)||used<0||used>175.616)return NaN;
  return peakMilliNm===3500?0:Math.min(10,(175.616-used)/(peakNm*peakNm-3.5*3.5));
}
