/** @file ID7会话内法兰软件参考；只计算投影，不改原始反馈或发送动作。 */
import type {MotorSnapshot,Sample} from './types';

/** 板端明确坐标合同和当前连接共同标识参考，不用采样epoch替代。 */
export interface FlangeZeroIdentity {connection:string;boot:number;reference:number;contract:'coord'}
/** 软件零点仅在内存保存，刷新后自然失效。 */
export interface FlangeZero {positionDeg:number;createdAt:number;identity:FlangeZeroIdentity}

/** 严格验证非零无损uint32身份；缺失数值不能隐式变成零。 */
function validIdentity(value:number|undefined|null):value is number {
  return typeof value==='number'&&Number.isInteger(value)&&value>=1&&value<=4294967295;
}
/** 从被动快照读取坐标身份，反馈年龄不改变身份连续性。 */
export function zeroIdentity(motor:MotorSnapshot|undefined,connection:string):FlangeZeroIdentity|null {
  if(motor?.id!==7||!connection||!validIdentity(motor.coordinateBoot)||!validIdentity(motor.coordinateReference))return null;
  return {connection,boot:motor.coordinateBoot,reference:motor.coordinateReference,contract:'coord'};
}
/** 捕获有限法兰原始角度及身份，不取模多圈坐标。 */
export function captureFlangeZero(positionDeg:number,identity:FlangeZeroIdentity,createdAt:number):FlangeZero|null {
  if(!Number.isFinite(positionDeg)||!Number.isFinite(createdAt)||!identity.connection||!validIdentity(identity.boot)||!validIdentity(identity.reference))return null;
  return {positionDeg,createdAt,identity:{...identity}};
}
/** 逐项核对连接、启动号与坐标基准；身份缺失即失效。 */
export function flangeZeroCurrent(zero:FlangeZero|null,identity:FlangeZeroIdentity|null):boolean {
  return !!zero&&!!identity&&zero.identity.connection===identity.connection&&zero.identity.boot===identity.boot&&
    zero.identity.reference===identity.reference&&zero.identity.contract===identity.contract;
}
/** 仅偏移角度与绝对目标，速度及原样本保持原值。 */
export function relativeFlangeSample(sample:Sample,zero:FlangeZero):Sample {
  return {...sample,positionDeg:sample.positionDeg===null?null:sample.positionDeg-zero.positionDeg,
    targetDeg:sample.targetDeg===null?null:sample.targetDeg-zero.positionDeg};
}
