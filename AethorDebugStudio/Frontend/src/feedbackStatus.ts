/** @file 区分USB连接、主控缓存年龄与有效电机反馈。 */
import { feedbackAge } from './domain';
import type { Coordinate, MotorSnapshot, Sample } from './types';
import { INTERNAL_RATIO, ROTOR_PER_FLANGE } from './axisUnits';
/** 只保留有限数；未知数据不能经隐式数字转换成为零。 */
function finiteValue(value: number | null | undefined): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null;
}
/** 身份字段必须是无损uint32，不猜测缺失启动号或采样代次。 */
function identityValue(value: number | null | undefined): number | null {
  return typeof value === 'number' && Number.isInteger(value) && value >= 0 && value <= 4294967295 ? value : null;
}
/** 连续坐标合同使用非零uint32；未知身份不能借用采样epoch。 */
function coordinateIdentityValue(value:number|null|undefined):number|null {
  const identity=identityValue(value);
  return identity!==null&&identity>0?identity:null;
}
/** 只按寄存器自己的年龄显示有限原始值，不借用USB接收时刻延长有效期。 */
export function registerValue(motor: MotorSnapshot, register: 50 | 51, now: number): number | null {
  const value = register === 50 ? motor.register50Raw : motor.register51Raw;
  const age = register === 50 ? motor.register50AgeMs : motor.register51AgeMs;
  const received = motor.receivedAt ? Date.parse(motor.receivedAt) : NaN;
  const effectiveAge = feedbackAge(age ?? null, received, now);
  return typeof value === 'number' && Number.isFinite(value) && Number.isFinite(received) &&
    effectiveAge !== null && effectiveAge < 500 ? value : null;
}
/** 固定原始记录口径：标准位置为转子度、标准速度为输出轴度每秒；估算仅来自板端。 */
export function captureTelemetry(motor: MotorSnapshot, connected: boolean, now: number, timeMs: number): Sample {
  const received = motor.receivedAt ? Date.parse(motor.receivedAt) : NaN;
  const standardAge = feedbackAge(motor.ageMs, received, now);
  const standardFresh = connected && standardAge !== null && standardAge <= 1000 && ['disabled','moving','holding','fault'].includes(motor.state);
  const register50Raw = connected ? registerValue(motor,50,now) : null;
  const register51Raw = connected ? registerValue(motor,51,now) : null;
  const contract = motor.id === 7 && motor.pmContract === 1 ? 1 : null;
  const coordinateIdentity:Partial<Sample>=motor.coordinateBoot!==undefined||motor.coordinateReference!==undefined?
    {coordinateBoot:motor.id===7?coordinateIdentityValue(motor.coordinateBoot):null,coordinateReference:motor.id===7?coordinateIdentityValue(motor.coordinateReference):null}:{};
  return {timeMs, positionDeg:standardFresh?finiteValue(motor.positionDeg):null, targetDeg:null,
    speedDegS:standardFresh?finiteValue(motor.speedDegS):null, torqueNm:standardFresh?finiteValue(motor.torqueNm):null,
    mosC:standardFresh?finiteValue(motor.mosC):null, rotorC:standardFresh?finiteValue(motor.rotorC):null,
    register50Raw, register51Raw,
    register50AgeMs:feedbackAge(motor.register50AgeMs??null,received,now), register51AgeMs:feedbackAge(motor.register51AgeMs??null,received,now),
    pmContract:contract, pmVelocityRadS:contract === 1 && register50Raw !== null ? finiteValue(motor.pmVelocityRadS) : null,
    pmBoot:contract === 1 ? identityValue(motor.pmBoot) : null, pmEpoch:contract === 1 ? identityValue(motor.pmEpoch) : null,...coordinateIdentity};
}
/** 整段记录按同一坐标纯投影；ID7确认比例才可跨轴换算，0x51没有工程测量资格。 */
export function projectTelemetry(sample: Sample, coordinate: Coordinate, recordedCoordinate: Coordinate, motorId: number): Sample {
  const confirmed = motorId === 7;
  const sameCoordinate = coordinate === recordedCoordinate;
  const ratio = coordinate === 'flange' ? ROTOR_PER_FLANGE : coordinate === 'output' ? INTERNAL_RATIO : 1;
  const recordedRatio = recordedCoordinate === 'flange' ? ROTOR_PER_FLANGE : recordedCoordinate === 'output' ? INTERNAL_RATIO : 1;
  const positionFactor = sameCoordinate ? 1 : confirmed ? recordedRatio / ratio : null;
  const speedFactor = sameCoordinate ? 1 : confirmed ?
    (recordedCoordinate === 'native' ? INTERNAL_RATIO : recordedRatio) / (coordinate === 'native' ? INTERNAL_RATIO : ratio) : null;
  const standardPosition = finiteValue(sample.positionDeg);
  const standardSpeed = finiteValue(sample.speedDegS);
  const registerQualified = confirmed && sample.pmContract === 1 && finiteValue(sample.register50Raw) !== null &&
    typeof sample.register50AgeMs === 'number' && sample.register50AgeMs >= 0 && sample.register50AgeMs < 500;
  const position = finiteValue(standardPosition !== null && positionFactor !== null ? standardPosition * positionFactor :
    registerQualified ? sample.register50Raw! * 180 / Math.PI / ratio : null);
  const estimatedSpeed = registerQualified ? finiteValue(sample.pmVelocityRadS) : null;
  const speed = finiteValue(standardSpeed !== null && speedFactor !== null ? standardSpeed * speedFactor :
    estimatedSpeed !== null ? estimatedSpeed * 180 / Math.PI / (coordinate === 'native' ? INTERNAL_RATIO : ratio) : null);
  return {...sample, positionDeg:position, speedDegS:speed,
    targetDeg:sample.targetDeg !== null && positionFactor !== null ? sample.targetDeg * positionFactor : null,
    standardPositionDeg:sample.standardPositionDeg !== undefined ? sample.standardPositionDeg : recordedCoordinate === 'native' ? standardPosition : null,
    standardSpeedDegS:sample.standardSpeedDegS !== undefined ? sample.standardSpeedDegS : recordedCoordinate === 'native' ? standardSpeed : null,
    positionSource:position === null ? 'missing' : standardPosition !== null ? (sample.positionSource ?? 'standard') : 'register50',
    speedSource:speed === null ? 'missing' : standardSpeed !== null ? (sample.speedSource ?? 'standard') : 'estimated'};
}
/** 解释真实采集状态；不把串口回复时间当成CAN反馈时间。 */
export function explainFeedback(motor: MotorSnapshot | undefined, connected: boolean, now: number) {
  if (!connected) return { title: '设备未连接', detail: '连接USB串口后才能读取控制板反馈。', fresh: false };
  if (!motor?.receivedAt) return { title: '等待控制板回复', detail: '尚未取得选中轴的查询响应。', fresh: false };
  const received = Date.parse(motor.receivedAt);
  if (!Number.isFinite(received)) return { title: '反馈时间未知', detail: '接收时间无效，本次数据不参与绘图。', fresh: false };
  if (!['disabled', 'moving', 'holding', 'fault'].includes(motor.state) && motor.register50Raw !== undefined && motor.register50Raw !== null) {
    if (motor.id !== 7 || motor.pmContract !== 1) return {title:'寄存器仅供诊断',detail:'固件未确认ID7的p_m单位契约；不换算工程位置或速度。0x51不具备测量资格。',fresh:false};
    if (registerValue(motor, 50, now) !== null) return { title: '位置寄存器正在更新', detail: '0x50转子位置已确认；速度仅在板端估算有效时显示。标准状态、转矩、温度及失能确认仍未知。', fresh: false };
    return { title: '位置寄存器样本已过期', detail: '超过500ms没有新位置样本；暂停绘制，标准状态反馈仍未知。', fresh: false };
  }
  const rawAge = /(?:^|\s)age_ms=(\d+)(?:\s|$)/.exec(motor.raw);
  const reportedAge = rawAge ? Number(rawAge[1]) : motor.ageMs;
  if (reportedAge === 4294967295) return { title: '尚未收到电机反馈', detail: '控制板未取得该轴CAN反馈；USB查询成功不代表电机在线。', fresh: false };
  const age = feedbackAge(motor.ageMs, received, now);
  const elapsed = Math.max(0, now - received);
  if (age !== null && age <= 1000 && ['disabled', 'moving', 'holding', 'fault'].includes(motor.state)) {
    return { title: '正在接收有效反馈', detail: `反馈年龄 ${Math.round(age)} ms；主控缓存查询约4 Hz，适合慢变化观察。`, fresh: true };
  }
  if (reportedAge !== null && Number.isFinite(reportedAge) && reportedAge >= 0 && reportedAge < 4294967295 && reportedAge + elapsed > 1000) {
    return { title: '电机反馈已过期', detail: `距板端最后一次电机反馈约 ${Math.floor((reportedAge + elapsed) / 1000)} 秒；show motor只读缓存，不主动产生CAN反馈。`, fresh: false };
  }
  return { title: '电机反馈无效或未知', detail: `板端状态 ${motor.state}；未知数据不画成零值，也不代表已失能。`, fresh: false };
}
