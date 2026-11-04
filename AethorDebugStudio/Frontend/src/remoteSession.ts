/** @file 仅结束已有上位机会话：身份复核、原事务失能证据、单次释放；不产生运动或接管。 */
import {remoteTerminalConfirmed,type RemoteReceipt} from './remoteControl';
import {monitorRemoteTransaction} from './remoteMonitor';
import type {RemoteWorkflowOptions,RemoteWorkflowResult} from './remoteWorkflow';

/** 跨调试页面保留尚未核实的释放意图；编号0表示HTTP回复未知，不能重发。 */
export interface RemoteSessionRecovery {boot:string;transactionId:number}
/** 原事务已失能且即将发送唯一释放请求时登记共享恢复上下文。 */
export interface SessionEndOptions extends RemoteWorkflowOptions {releaseIntent?:(boot:string)=>void}

/** 协议owner=1未区分LCD实际持有和本地故障锁定，显示时保留此不确定性。 */
export function describeRemoteOwner(owner:string|undefined,authority?:string):string {
  if(authority==='3')return '本地故障锁定';
  return owner==='2'?'上位机持有':owner==='1'?'LCD / 本地保护占用':owner==='0'?'未申请':'控制权未知';
}

/** 仅采用新固件明确给出的故障结束审核标志，不由owner或过期反馈推断资格。 */
export function remoteFaultReleaseAllowed(fields:Record<string,string>):boolean {
  return fields.authority==='3'&&fields.remote_fault_release==='1'&&['1','2'].includes(fields.owner);
}

/** 只接受明确的uint32身份，避免空值或负值被数值转换后当成有效编号。 */
function positiveIdentity(value:string|undefined):boolean {
  return /^\d+$/.test(value??'')&&Number.isSafeInteger(Number(value))&&Number(value)>0&&Number(value)<=0xffffffff;
}

/** 一次用户结束意图只提交一次release；未知/活动/失能未确认时只能恢复原编号。 */
export async function endRemoteSession(options:SessionEndOptions):Promise<RemoteWorkflowResult>{
  let pendingId=0,releaseAttempted=false;
  const done=(result:string,detail:string,id=0,operation:RemoteWorkflowResult['operation']=null):RemoteWorkflowResult=>({result,detail,transactionId:id,operation});
  try{
    if(!options.isCurrent())return done('cancelled','结束意图已取消');
    const initial=await options.request({operation:'caps'});
    if(!options.isCurrent())return done('cancelled','连接或STOP意图已变化，未释放控制权');
    options.observe('caps',initial);
    const fields=initial.fields;
    const faultRelease=remoteFaultReleaseAllowed(fields);
    if(initial.result!=='capability'||!positiveIdentity(fields.boot)||!positiveIdentity(fields.epoch)||fields.active_id!=='0')
      return done('rejected','设备能力不完整或仍有活动事务；请先读取原事务终态，不会提交释放');
    if(fields.authority==='3'&&!faultRelease)return done('rejected','本地故障锁定尚不满足板端安全结束审核；请保留保护，核对失能、运输清理和会话身份');
    if(fields.owner==='0'&&(fields.authority===undefined||fields.authority==='0'))return done('completed','当前控制权未申请，无需重复释放');
    if(fields.owner==='1'&&!faultRelease)return done('rejected','当前固件未提供合格的本地故障结束审核；仅网页无法解除。请升级对应固件，或在LCD控制准备页恢复并释放');
    if((fields.owner!=='2'&&!faultRelease)||!positiveIdentity(fields.terminal_id))return done('rejected','缺少可核对的上位机终态和失能证明，未释放控制权');
    const originalId=Number(fields.terminal_id);
    const original=await options.request({operation:'result',transactionId:originalId});
    if(!options.isCurrent())return done('cancelled','连接或STOP意图已变化，未释放控制权');
    options.observe('result',original);
    if(original.operation!=='result'||original.fields.id!==String(originalId)||
      !remoteTerminalConfirmed(original,null)||original.fields.accepted!=='1'||original.fields.disabled!=='1'||faultRelease&&original.result!=='failed')
      return done('unknown','原事务终态或失能尚未确认，仅可继续读取原事务结果；未提交释放',originalId);
    const review=await options.request({operation:'caps'});
    if(!options.isCurrent())return done('cancelled','连接或STOP意图已变化，未释放控制权');
    options.observe('caps',review);
    if(review.result!=='capability'||review.fields.boot!==fields.boot||review.fields.epoch!==fields.epoch||
      review.fields.terminal_id!==fields.terminal_id||review.fields.owner!==fields.owner||
      review.fields.authority!==fields.authority||review.fields.remote_fault_release!==fields.remote_fault_release||
      review.fields.active_id!=='0'||!positiveIdentity(review.fields.token))
      return done('rejected','启动号、会话或控制权已变化，结束意图失效；未提交释放');
    if(!options.isCurrent())return done('cancelled','结束意图已取消');
    options.releaseIntent?.(fields.boot);releaseAttempted=true;
    const submitted=await options.request({operation:'release',token:Number(review.fields.token),motorId:7,
      coordinate:'output-rad',delta:0,speed:.02,kp:120,kd:.3,torque:0,duration:1000});
    pendingId=submitted.requestId;
    if(!options.isCurrent())return done('cancelled','提交后的连接已变化，仅可恢复释放事务',pendingId,'release');
    options.observe('release',submitted);
    if(submitted.result!=='queued')return done(submitted.result==='rejected'?'rejected':'unknown',
      '板端未确认释放入队；未重发，请重新读取能力',submitted.result==='rejected'?0:pendingId,'release');
    let terminal:RemoteReceipt|null=null;
    const outcome=await monitorRemoteTransaction(pendingId,{...options,observe:receipt=>{terminal=receipt;options.observe('result',receipt);}});
    const completion=terminal as RemoteReceipt|null;
    if(outcome!=='completed'||!completion||!remoteTerminalConfirmed(completion,'release'))
      return done('unknown','释放终态尚未确认，保留释放事务编号，只读恢复；未重发',pendingId,'release');
    if(!options.isCurrent())return done('cancelled','连接已变化，仅可恢复释放事务',pendingId,'release');
    const after=await options.request({operation:'caps'});
    if(!options.isCurrent())return done('cancelled','连接已变化，仅可恢复释放事务',pendingId,'release');
    options.observe('caps',after);
    if(after.result!=='capability'||after.fields.boot!==fields.boot||after.fields.owner!=='0'||
      after.fields.authority!==undefined&&after.fields.authority!=='0'||
      after.fields.active_id!=='0'||after.fields.terminal_id!==String(pendingId))
      return done('unknown','收到释放终态，但当前控制权尚未核实；请只读恢复，不重新释放',pendingId,'release');
    return done('completed','会话已结束，控制权已释放；未使能、未运动，也未恢复实验配额');
  }catch(error){return done('unknown',String(error)+'；结束会话未确认，未重发',pendingId,releaseAttempted?'release':null);}
}

/** 只读恢复必须重新证明释放后的当前会话状态；原来的completed回执本身不能解锁。 */
export async function confirmSessionReleased(context:RemoteSessionRecovery,receipt:RemoteReceipt,options:RemoteWorkflowOptions):Promise<boolean>{
  if(!options.isCurrent()||!positiveIdentity(context.boot)||receipt.operation!=='result'||
    receipt.result!=='completed'||receipt.fields.accepted!=='1'||!positiveIdentity(receipt.fields.id)||
    context.transactionId!==0&&String(context.transactionId)!==receipt.fields.id)return false;
  try{
    const proof=await options.request({operation:'caps'});
    if(!options.isCurrent())return false;
    options.observe('caps',proof);
    return proof.result==='capability'&&proof.fields.boot===context.boot&&positiveIdentity(proof.fields.epoch)&&
      proof.fields.owner==='0'&&(proof.fields.authority===undefined||proof.fields.authority==='0')&&
      proof.fields.active_id==='0'&&proof.fields.terminal_id===receipt.fields.id;
  }catch{return false;}
}
