/** @file 模式2配额准备：明确单次失能、重新核验原维护门禁、一次应用；无使能和运动。 */
import type {DebugReceipt} from './types';
import type {RemoteReceipt} from './remoteControl';
import {parseTorqueEnvelope,type TorqueEnvelopePolicy} from './torqueEnvelope';

/** 可注入传输和取消代际；生产调用者负责持有全局维护锁。 */
export interface TorquePreparationOptions {
  request:(body:Record<string,unknown>)=>Promise<RemoteReceipt>;
  disable:()=>Promise<DebugReceipt>;
  isCurrent:()=>boolean;
  observe:(message:string,policy:TorqueEnvelopePolicy|null)=>void;
}
/** 未确认失能必须保留全局动作锁，不能用维护失败解除原保护。 */
export interface TorquePreparationResult {
  result:'confirmed'|'blocked'|'cancelled'|'unknown';detail:string;
  policy:TorqueEnvelopePolicy|null;uncertainDisable:boolean;uncertainMaintenance:boolean;
}
/** 只在唯一阻塞为过期标准反馈时发一次失能，旧缓存不能满足失能后的原门禁。 */
export async function prepareTorqueAllowance(reviewed:TorqueEnvelopePolicy,peakMilliNm:number,options:TorquePreparationOptions):Promise<TorquePreparationResult> {
  let policy:TorqueEnvelopePolicy|null=null,uncertainDisable=false,resetUnconfirmed=false;
  const done=(result:TorquePreparationResult['result'],detail:string):TorquePreparationResult=>({result,detail,policy,uncertainDisable,uncertainMaintenance:uncertainDisable||resetUnconfirmed});
  const current=()=>options.isCurrent();
  const matches=(value:TorqueEnvelopePolicy|null)=>value?.configurable&&value.boot===reviewed.boot&&value.reference===reviewed.reference;
  if(!reviewed.configurable||reviewed.blockers!==32||!Number.isInteger(peakMilliNm)||peakMilliNm<3500||peakMilliNm>6240)
    return done('blocked','仅支持当前v2策略且唯一阻塞为标准反馈过期；其他故障须先处理。');
  try {
    if(!current())return done('cancelled','本次维护已取消，未发送动作。');
    const caps=await options.request({operation:'caps'});
    if(!current())return done('cancelled','连接或操作意图已变化，未发送失能。');
    if(caps.result!=='capability'||caps.fields.boot!==String(reviewed.boot)||caps.fields.coord!==String(reviewed.reference)||
      caps.fields.owner!=='0'||caps.fields.authority!=='0'||caps.fields.active_id!=='0'||!['1','2'].includes(caps.fields.mode))
      return done('blocked','身份、模式或空闲控制权未确认；请结束原会话后重新读取。');
    policy=parseTorqueEnvelope(await options.request({operation:'torque'}));
    if(!current())return done('cancelled','维护已取消，未发送下一步。');
    if(!matches(policy)||!policy||![0,32].includes(policy.blockers!))return done('blocked','设备身份已变化或出现其他门禁，未发送失能。');
    if(policy.blockers===32){
      options.observe('正在单次失能采样；会撤去ID7保持力矩，不使能、不切换模式。',policy);
      if(!current())return done('cancelled','维护已取消，未发送失能。');
      uncertainDisable=true;
      const disabled=await options.disable();
      if(!current())return done('cancelled','失能提交后操作意图已变化；未继续恢复配额，请读取原回执。');
      if(disabled.operation!=='torque-disable'||disabled.motorId!==7||!disabled.accepted||disabled.result!=='completed')
        return done('unknown',`单次失能未确认完成（${disabled.result}）；未重试、未恢复配额。`);
      // completed不单独证明新反馈；前读缺失且后读通过，才确认新可用的标准反馈。
      policy=parseTorqueEnvelope(await options.request({operation:'torque'}));
      if(!current())return done('cancelled','操作意图已变化，未恢复配额。');
      if(!matches(policy)||!policy)return done('unknown','失能后身份或策略无法确认；未恢复配额。');
      if(policy.blockers!==null&&(policy.blockers&(4|8|16|32|64|128))===0)uncertainDisable=false;
      if(!policy.canReset||policy.blockers!==0){options.observe('已读取失能后的策略，原维护门禁未通过。',policy);return done('blocked','新反馈未通过冷却、失能或空闲门禁；未恢复配额、未重复失能。');}
      uncertainDisable=false;
    }
    if(!policy.canReset||!current())return done('cancelled','维护意图已失效，未恢复配额。');
    resetUnconfirmed=true;
    const result=await options.request({operation:'torque_reset',token:policy.token,cooledConfirmed:true,peakMilliNm});
    if(!current())return done('cancelled','维护提交后连接或操作意图已变化，请只读查询策略；未重发。');
    policy=null;
    if(result.result!=='reset'||result.fields.accepted!=='1'||result.fields.authorized!=='1')return done('unknown','配额恢复未确认；未重发，请只读查询策略。');
    policy=parseTorqueEnvelope(await options.request({operation:'torque'}));
    if(!current())return done('cancelled','连接或操作意图已变化，读回不再作为当前确认。');
    if(!matches(policy)||!policy?.authorized||policy.peakMilliNm!==peakMilliNm)return done('unknown','维护已接受但读回不符或未知；未重发。');
    resetUnconfirmed=false;
    return done('confirmed',`读回确认 ${(peakMilliNm/1000).toFixed(3)} N·m；单轴静载和POS带载共用此配额。未使能、未启动运动。`);
  }catch(error){policy=null;return done('unknown',String(error)+'；未重试失能或配额恢复。');}
}
