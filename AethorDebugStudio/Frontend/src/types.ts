/** @file 工作台值类型；真实、合成与回放来源不能互相冒充。 */
export type DataMode = 'demo' | 'live' | 'replay';
export type Flow = 'manual' | 'static' | 'pos' | 'seven';
export type PageId = 'overview' | Flow | 'guide' | 'scope' | 'maintenance' | 'records' | 'lab';
export type Coordinate = 'native' | 'output' | 'flange';
/** 顶部只读控制方快照；只表示读取时的设备声明，不授予操作权限。 */
export interface ControlReadback { label:string; readAt:number; connectionKey:string }
/** 带单位的编辑草稿；kp/kd采用输出轴等效量，不能直接作为CAN编码。 */
export interface Settings { motorId: number; angleDeg: number; speedDegS: number; durationS: number; torqueFfNm: number; massG: number; leverMm: number; kp: number; kd: number }
/** 单个原始坐标采样；坐标身份用于断线和导出，未知反馈始终为null，软件零点不覆盖记录。 */
export interface Sample { coordinateBoot?: number | null; coordinateReference?: number | null; timeMs: number; positionDeg: number | null; targetDeg: number | null; speedDegS: number | null; torqueNm: number | null; mosC: number | null; rotorC: number | null; register50Raw?: number | null; register51Raw?: number | null; register50AgeMs?: number | null; register51AgeMs?: number | null; pmContract?: 1 | null; pmVelocityRadS?: number | null; pmBoot?: number | null; pmEpoch?: number | null; standardPositionDeg?: number | null; standardSpeedDegS?: number | null; positionSource?: 'standard' | 'register50' | 'missing'; speedSource?: 'standard' | 'estimated' | 'missing' }
export type Signal = 'positionDeg' | 'targetDeg' | 'speedDegS' | 'torqueNm' | 'mosC' | 'rotorC' | 'register50Raw' | 'register51Raw';
/** 事件日志保留人可读描述与原始CAN，缺口以gap显式呈现。 */
export interface StudioEvent { timeMs: number; level: 'info' | 'warning' | 'error'; title: string; detail: string; rawHex?: string; gap?: boolean }
/** 单轴静载的显式策略/坐标绑定；旧文件未绑定时不能套用新门槛。 */
export interface StaticContext { policyId:string; holdStartMs:number; kpOutput:number; kdOutput:number; torqueFfNm:number }
/** 可移植记录；source表示数据来源而非物理资格。 */
export interface Recording { schema: 'aethor-studio-recording/v1'; name: string; source: 'synthetic' | 'device'; coordinate: Coordinate; motorId: number; createdAt: string; samples: Sample[]; events: StudioEvent[]; staticContext?:StaticContext }
/** 真实网关公开快照，反馈坐标保留固件原始口径。 */
export interface MotorSnapshot { coordinateBoot?: number | null; coordinateReference?: number | null; id: number; state: string; positionDeg: number | null; speedDegS: number | null; torqueNm: number | null; mosC: number | null; rotorC: number | null; fault: number | null; ageMs: number | null; receivedAt: string | null; pmaxDeg: number | null; raw: string; register50Raw?: number | null; register50AgeMs?: number | null; register51Raw?: number | null; register51AgeMs?: number | null; pmContract?: 1 | null; pmVelocityRadS?: number | null; pmBoot?: number | null; pmEpoch?: number | null }
/** 会话信息不从历史部署日志推断当前固件。 */
export interface GatewaySnapshot { connected: boolean; port: string | null; status: string; error: string | null; receivedAt: string | null; info: Record<string, string>; motors: MotorSnapshot[]; lines: string[]; readOnly: boolean; debugAvailable?: boolean; torquePreparationAvailable?: boolean; debugReceipts?: DebugReceipt[] }
export type DebugOperation = 'diagnostics' | 'stop' | 'stop-all' | 'disable' | 'torque-disable';
/** 有编号的软件回执；completed不能单独证明物理停止或已失能。 */
export interface DebugReceipt { requestId: number; operation: DebugOperation; motorId: number; accepted: boolean; result: string; detail: string; lines: string[]; feedbackConfirmed: boolean }
export interface Feature { id: string; title: string; category: string; status: 'available' | 'pending' | 'experimental' | 'planned'; description: string; page: PageId }
export interface WorkflowState { stage: 'idle' | 'preflight' | 'review' | 'running' | 'completed' | 'stopped'; startedAt: number; frozen: Settings | null }
