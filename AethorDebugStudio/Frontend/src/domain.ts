/** @file 参数、记录和曲线的纯逻辑；不包含串口或运动发送。 */
import type { Coordinate, DataMode, Flow, Recording, Sample, Settings, Signal, StaticContext, StudioEvent } from './types';
export const COORDINATE_LABEL: Record<Coordinate, string> = { native: '固件原始反馈', output: '电机输出轴', flange: '法兰' };
export const MAX_RECORD_BYTES = 2_000_000;
/** 验证演示流程草稿；这些编辑边界不授予真实设备动作资格。 */
export function validateSettings(flow: Flow, settings: Settings): string[] {
  const errors: string[] = [];
  if (!Number.isInteger(settings.motorId) || settings.motorId < 1 || settings.motorId > 7) errors.push('电机编号必须为1–7');
  const bounds: [keyof Settings, number, number, string][] = [
    ['durationS', 1, 60, '保持时间'], ['massG', 0, 10000, '额外挂重'], ['leverMm', 0, 1000, '有效力臂'],
  ];
  if (flow === 'manual' || flow === 'pos') bounds.push(['angleDeg', settings.motorId === 7 ? -180 : -3, settings.motorId === 7 ? 180 : 3, '相对转角'], ['speedDegS', 0.01, settings.motorId === 7 ? 10 : 3, '移动速度']);
  if (flow === 'static' || flow === 'manual') bounds.push(['torqueFfNm', -3.5, 3.5, '前馈力矩'], ['kp', 0, 500, '位置刚度Kp'], ['kd', 0, 5, '阻尼Kd']);
  for (const [key, minimum, maximum, label] of bounds) {
    if (!Number.isFinite(settings[key]) || settings[key] < minimum || settings[key] > maximum) errors.push(label + '须在' + minimum + '至' + maximum + '范围内');
  }
  return errors;
}
/** 只允许离线演示；真实与回放来源始终拒绝模拟控制入口。 */
export function canExecute(mode: DataMode, running: boolean, errors: string[]): { allowed: boolean; reason: string } {
  if (mode !== 'demo') return { allowed: false, reason: mode === 'live' ? '本表单用于离线演示；真实参数下发、单轴动作及停止请进入手动调试区' : '回放模式不执行动作' };
  if (running) return { allowed: false, reason: '已有演示正在执行' };
  return { allowed: errors.length === 0, reason: errors[0] ?? '' };
}
/** 将设备报告年龄与主机等待时间相加；时间异常或未知时不输出新鲜值。 */
export function feedbackAge(deviceAgeMs: number | null, receivedAtMs: number, nowMs: number): number | null {
  if (deviceAgeMs === null || !Number.isFinite(deviceAgeMs) || deviceAgeMs < 0 || !Number.isFinite(receivedAtMs) || nowMs < receivedAtMs) return null;
  return deviceAgeMs + nowMs - receivedAtMs;
}
/** 仅当用户有明确已确认比例时换算显示值；null表示未标定。 */
export function convertAngle(outputAngle: number, outputPerFlange: number | null): number | null {
  return outputPerFlange !== null && Number.isFinite(outputPerFlange) && outputPerFlange > 0 && Number.isFinite(outputAngle) ? outputAngle / outputPerFlange : null;
}
/** 判断字典而不是数组或null，避免导入时的隐式类型转换。 */
function objectValue(value: unknown): Record<string, unknown> {
  if (value === null || typeof value !== 'object' || Array.isArray(value)) throw new Error('记录结构错误');
  return value as Record<string, unknown>;
}
/** 读取有限数字或保留null；拒绝字符串、布尔和无限值。 */
function numeric(value: unknown, nullable = true): number | null {
  if (nullable && value === null) return null;
  if (typeof value !== 'number' || !Number.isFinite(value)) throw new Error('记录包含非有限数值或错误数字类型');
  return value;
}
/** 限长读取不可信文本；界面仅以文本渲染它。 */
function textValue(value: unknown, maximum = 500): string {
  if (typeof value !== 'string' || value.length > maximum) throw new Error('记录文本缺失或过长');
  return value;
}
/** 导入现有冻结启动trace；应用快照不作为原始RX的已解码反馈。 */
function parseNativeTrace(value: Record<string, unknown>): Recording {
  if (!Array.isArray(value.records) || value.records.length < 1 || value.records.length > 256) throw new Error('启动追踪记录数无效');
  const decoded = value.records.map((entry) => objectValue(objectValue(entry).decoded));
  const firstTime = numeric(decoded[0].timestamp_us, false)!;
  const accepted = decoded.filter((entry) => entry.event_name === 'accepted_rx' && typeof entry.motor_id === 'number' && entry.motor_id >= 1 && entry.motor_id <= 7);
  const identified = decoded.find((entry) => typeof entry.motor_id === 'number' && Number.isInteger(entry.motor_id) && entry.motor_id >= 1 && entry.motor_id <= 7);
  if (!identified) throw new Error('追踪中缺少可确认的电机编号');
  const motorId = accepted.length ? Number(accepted[0].motor_id) : Number(identified.motor_id);
  const samples: Sample[] = accepted.filter((entry) => entry.motor_id === motorId).map((entry) => ({
    timeMs: (numeric(entry.timestamp_us, false)! - firstTime) / 1000,
    positionDeg: traceNumber(entry.position_rad, 180 / Math.PI), targetDeg: traceNumber(entry.target_rad, 180 / Math.PI),
    speedDegS: traceNumber(entry.velocity_rad_s, 180 / Math.PI), torqueNm: traceNumber(entry.torque_nm), mosC: null, rotorC: null,
  }));
  const events: StudioEvent[] = decoded.map((entry, index) => ({
    timeMs: (numeric(entry.timestamp_us, false)! - firstTime) / 1000,
    level: entry.event_name === 'first_fault' ? 'error' : 'info',
    title: textValue(entry.event_name, 80),
    detail: '阶段 ' + String(entry.phase_name ?? '?') + ' · 序号 ' + String(entry.serial) + ' · CAN仅证明总线观察顺序',
    rawHex: textValue(entry.can_raw_hex ?? '', 32),
    gap: index > 0 && Number(entry.serial) !== Number(decoded[index - 1].serial) + 1,
  }));
  const header = value.decoded_header && typeof value.decoded_header === 'object' ? objectValue(value.decoded_header) : null;
  if (header?.first_fault_record) {
    const firstFault = objectValue(header.first_fault_record);
    if (!decoded.some((entry) => entry.serial === firstFault.serial)) events.push({ timeMs:(numeric(firstFault.timestamp_us,false)!-firstTime)/1000, level:'error', title:'首故障 · 永久保留槽', detail:'原始环形窗口未保留此事件，从首故障独立槽恢复', rawHex:textValue(firstFault.can_raw_hex??'',32) });
  }
  events.sort((left,right)=>left.timeMs-right.timeMs);
  return { schema: 'aethor-studio-recording/v1', name: '启动追踪 · 第' + String(value.generation ?? '?') + '代', source: 'device', coordinate: 'native', motorId, createdAt: textValue(value.captured_at_utc, 100), samples, events };
}
/** 原始trace中明确的NaN标记映射为未知，不按0处理。 */
function traceNumber(value: unknown, multiplier = 1): number | null {
  if (typeof value === 'string' && ['NaN', '+Inf', '-Inf'].includes(value)) return null;
  const numberValue = numeric(value);
  return numberValue === null ? null : numberValue * multiplier;
}
/** 对两个支持的JSON格式做边界校验；限制2MB及20,000个样本。 */
export function parseRecording(text: string): Recording {
  if (new TextEncoder().encode(text).length > MAX_RECORD_BYTES) throw new Error('记录不能超过2MB');
  let value = objectValue(JSON.parse(text));
  if (value.schema === 'static-startup-capture-v1') value = parseNativeTrace(value) as unknown as Record<string, unknown>;
  if (value.schema !== 'aethor-studio-recording/v1') throw new Error('不支持的记录格式');
  if (!['synthetic', 'device'].includes(String(value.source)) || !['native', 'output', 'flange'].includes(String(value.coordinate))) throw new Error('数据来源或坐标无效');
  if (!Number.isInteger(value.motorId) || Number(value.motorId) < 1 || Number(value.motorId) > 7) throw new Error('电机编号无效');
  const createdAt = textValue(value.createdAt, 100);
  if (!Number.isFinite(Date.parse(createdAt))) throw new Error('记录日期无效');
  if (!Array.isArray(value.samples) || value.samples.length > 20000 || !Array.isArray(value.events) || value.events.length > 5000) throw new Error('样本或事件数量超限');
  let lastTime = -1;
  const samples = value.samples.map((entry) => {
    const sample = objectValue(entry);
    const timeMs = numeric(sample.timeMs, false)!;
    if (timeMs < 0 || timeMs < lastTime) throw new Error('样本时间不能倒退');
    lastTime = timeMs;
    const telemetry: Partial<Sample> = {};
    for (const key of ['register50Raw','register51Raw','register50AgeMs','register51AgeMs','pmVelocityRadS','standardPositionDeg','standardSpeedDegS'] as const)
      if (sample[key] !== undefined) telemetry[key] = numeric(sample[key]);
    if (sample.pmContract !== undefined) {
      if (sample.pmContract !== null && sample.pmContract !== 1) throw new Error('寄存器单位契约无效');
      telemetry.pmContract = sample.pmContract;
    }
    for (const key of ['pmBoot','pmEpoch','coordinateBoot','coordinateReference'] as const) {
      if (sample[key] === undefined) continue;
      const identity = numeric(sample[key]);
      const minimum=key==='coordinateBoot'||key==='coordinateReference'?1:0;
      if (identity !== null && (!Number.isInteger(identity) || identity < minimum || identity > 4294967295)) throw new Error('遥测启动号、采样代次或坐标参考无效');
      telemetry[key] = identity;
    }
    if (sample.positionSource !== undefined) {
      if (!['standard','register50','missing'].includes(String(sample.positionSource))) throw new Error('位置来源无效');
      telemetry.positionSource = sample.positionSource as Sample['positionSource'];
    }
    if (sample.speedSource !== undefined) {
      if (!['standard','estimated','missing'].includes(String(sample.speedSource))) throw new Error('速度来源无效');
      telemetry.speedSource = sample.speedSource as Sample['speedSource'];
    }
    return { timeMs, positionDeg: numeric(sample.positionDeg), targetDeg: numeric(sample.targetDeg), speedDegS: numeric(sample.speedDegS), torqueNm: numeric(sample.torqueNm), mosC: numeric(sample.mosC), rotorC: numeric(sample.rotorC), ...telemetry };
  });
  const events: StudioEvent[] = value.events.map((entry) => {
    const event = objectValue(entry);
    const timeMs = numeric(event.timeMs, false)!;
    if (timeMs < 0 || !['info', 'warning', 'error'].includes(String(event.level))) throw new Error('事件时间或等级错误');
    const rawHex = event.rawHex === undefined ? undefined : textValue(event.rawHex, 512);
    if (rawHex !== undefined && !/^[0-9a-f]*$/i.test(rawHex)) throw new Error('CAN字节不是十六进制');
    return { timeMs, level: event.level as StudioEvent['level'], title: textValue(event.title, 160), detail: textValue(event.detail, 1000), rawHex, gap: event.gap === true };
  });
  let staticContext:StaticContext|undefined;
  if(value.staticContext!==undefined){
    const context=objectValue(value.staticContext);
    const holdStartMs=numeric(context.holdStartMs,false)!;
    const kpOutput=numeric(context.kpOutput,false)!;
    const kdOutput=numeric(context.kdOutput,false)!;
    const torqueFfNm=numeric(context.torqueFfNm,false)!;
    if(holdStartMs<0||kpOutput<0||kdOutput<0)throw new Error('静载策略上下文无效');
    staticContext={policyId:textValue(context.policyId,100),holdStartMs,kpOutput,kdOutput,torqueFfNm};
  }
  return { schema: 'aethor-studio-recording/v1', name: textValue(value.name, 160), source: value.source as Recording['source'], coordinate: value.coordinate as Coordinate, motorId: Number(value.motorId), createdAt, samples, events, staticContext };
}
/** 旧记录维持原兼容；一旦带现代坐标字段，只有两端完整相同身份才允许连续。 */
function sameCoordinateReference(previous:Sample,current:Sample):boolean {
  const fields=['coordinateBoot','coordinateReference'] as const;
  if(fields.every(key=>previous[key]===undefined&&current[key]===undefined))return true;
  return fields.every(key=>{
    const previousIdentity=previous[key],currentIdentity=current[key];
    return typeof previousIdentity==='number'&&Number.isInteger(previousIdentity)&&previousIdentity>=1&&previousIdentity<=4294967295&&previousIdentity===currentIdentity;
  });
}
/** 将缺失值与超时段拆成独立折线，防止视觉上填补数据缺口。 */
export function sampleSegments(samples: Sample[], signal: Signal, gapMs = 250, explicitGaps: number[] = []): Sample[][] {
  const segments: Sample[][] = [];
  let segment: Sample[] = [];
  for (const sample of samples) {
    const previousTime = segment.at(-1)?.timeMs;
    const previous = segment.at(-1);
    const sourceChanged = previous !== undefined && ((signal === 'positionDeg' && previous.positionSource !== sample.positionSource) || (signal === 'speedDegS' && previous.speedSource !== sample.speedSource));
    // 新契约缺任一身份只显示单点；同COM上的重启或采样代次变化不能延续旧折线。
    const identityRequired = previous?.pmContract === 1 || sample.pmContract === 1;
    const identityChanged = identityRequired && (previous?.pmBoot == null || previous.pmEpoch == null || sample.pmBoot == null || sample.pmEpoch == null || previous.pmBoot !== sample.pmBoot || previous.pmEpoch !== sample.pmEpoch);
    const coordinateChanged=previous!==undefined&&['positionDeg','targetDeg','speedDegS'].includes(signal)&&!sameCoordinateReference(previous,sample);
    const registerSignal = (signal === 'positionDeg' && sample.positionSource === 'register50') || (signal === 'speedDegS' && sample.speedSource === 'estimated') || signal === 'register50Raw' || signal === 'register51Raw';
    const interval = previousTime === undefined ? 0 : sample.timeMs - previousTime;
    const expiredGap = registerSignal ? interval >= Math.min(gapMs,500) : interval > gapMs;
    const hasGap = previousTime !== undefined && (sourceChanged || identityChanged || coordinateChanged || expiredGap || explicitGaps.some((time) => time > previousTime && time <= sample.timeMs));
    const valid = typeof sample[signal] === 'number' && Number.isFinite(sample[signal]);
    if (!valid || hasGap) { if (segment.length) segments.push(segment); segment = []; }
    if (valid) segment.push(sample);
  }
  if (segment.length) segments.push(segment);
  return segments;
}
/** CSV仅输出固定列与数字，用户文本不进入公式单元格。 */
export function toCsv(recording: Recording): string {
  const header = '# source=' + recording.source + ';coordinate=' + recording.coordinate + ';motor=' + recording.motorId + ';position_reference=raw_unshifted;id7_rotor_per_output=19.2032;id7_rotor_per_flange=44.8;pm_velocity=board_estimate_only\n';
  const columns: (keyof Sample)[] = ['timeMs', 'positionDeg', 'targetDeg', 'speedDegS', 'torqueNm', 'mosC', 'rotorC', 'standardPositionDeg', 'standardSpeedDegS', 'pmContract', 'pmBoot', 'pmEpoch', 'pmVelocityRadS', 'register50AgeMs', 'register51AgeMs', 'positionSource', 'speedSource','coordinateBoot','coordinateReference', 'register50Raw', 'register51Raw'];
  return '\uFEFF' + header + 'time_ms,position_deg,target_deg,speed_deg_s,torque_nm,mos_c,rotor_c,standard_position_rotor_deg,standard_speed_output_deg_s,pm_contract,pm_boot,pm_epoch,pm_velocity_rotor_rad_s,register50_age_ms,register51_age_ms,position_source,speed_source,coordinate_boot,coordinate_reference,register50_raw,register51_raw\n' + recording.samples.map((sample) => columns.map((key) => sample[key] ?? '').join(',')).join('\n');
}
