/** @file 页面与功能登记表；状态指本版上位机接入情况，不代表硬件资格。 */
import type { Feature, PageId, Settings } from './types';
import { STATIC_RELEASE } from './staticPolicy';
export const DEFAULT_SETTINGS: Settings = { motorId: 7, angleDeg: 1, speedDegS: 1, durationS: 1, torqueFfNm: 0, massG: 0, leverMm: 0, kp: STATIC_RELEASE.kpOutput, kd: STATIC_RELEASE.kdOutput };
export const PAGES: { id: PageId; title: string; subtitle: string; label: string; group: string }[] = [
  { id: 'overview', title: '调试工作台', subtitle: '先连接和检查反馈，再设置一次有边界的测试。', label: '工作台总览', group:'开始使用' },
  { id: 'guide', title: '新手手册', subtitle: '从连接到结束会话，按实际按钮查找操作与限制。', label: '新手手册', group:'开始使用' },
  { id: 'static', title: '单轴静载', subtitle: '检查 → 参数 → 转矩保护 → 启动一次 → 结果与恢复。', label: '单轴静载', group:'测试与参数' },
  { id: 'pos', title: 'POS 带载测试', subtitle: '相对当前姿态移动，到位后限时保持，再停止并确认失能。', label: 'POS 带载测试', group:'测试与参数' },
  { id: 'manual', title: '手动操作与参数', subtitle: '单次动作与RAM参数写入；先检查设备能力，再明确提交。', label: '手动调试', group:'测试与参数' },
  { id: 'seven', title: '七轴调试 · 离线流程', subtitle: '本页为离线流程演示；七轴远程运动尚未接入。', label: '七轴调试', group:'测试与参数' },
  { id: 'scope', title: '最近60秒示波', subtitle: '真实模式持续观察最近60秒；回放使用记录时间轴。', label: '实时示波', group:'观察与维护' },
  { id: 'records', title: '记录与回放', subtitle: '导入、导出与回放，保留来源和首个异常。', label: '记录与诊断', group:'观察与维护' },
  { id: 'maintenance', title: '驱动信息与维护', subtitle: '区分临时配置、实际读回与掉电保存。', label: '驱动与维护', group:'观察与维护' },
  { id: 'lab', title: '功能扩展与接入状态', subtitle: '界面已接入、实验能力与规划功能分别列出。', label: '实验与扩展', group:'观察与维护' },
];
export const FEATURES: Feature[] = [
  { id: 'read', title: '七轴只读状态', category: '设备', status: 'available', description: '选择本机USB串口后只读查询；电机CAN反馈缺失时显示未知，不据此授予运行资格。', page: 'overview' },
  { id: 'device-debug', title: '采集诊断与停止/失能', category: '诊断', status: 'available', description: '固定白名单查询、软件STOP和显式失能；回执留痕，超时不自动重试，实机动作尚未执行验证。', page: 'manual' },
  { id: 'manual', title: 'POS / MIT 手动调试', category: '控制', status: 'available', description: '真实模式已接入ID7单次操作、模式准备与静载RAM参数；设备能力、失能反馈和实机资格仍须分别确认。', page: 'manual' },
  { id: 'static', title: '单轴静载与抗扰观察', category: '控制', status: 'available', description: '真实页面设置参数后启动单轴流程；设备须确认固件能力与反馈，界面接入不代表实机验收。', page: 'static' },
  { id: 'gain-budget', title: 'Kp / Kd 离线预算', category: '实验', status: 'available', description: '80/0.2是发布基线；较高增益独立比较，不自动下发，不构成稳定性或承载证明。', page: 'static' },
  { id: 'pos', title: 'POS 移动与限时保持', category: '控制', status: 'available', description: '真实页面使用专用POS带载流程，按参数移动并限时保持；设备能力和实机资格单独验证。', page: 'pos' },
  { id: 'seven', title: '七轴静载保持', category: '控制', status: 'pending', description: '已有LCD入口；七轴标定与逐轴准入不能由界面替代。', page: 'seven' },
  { id: 'trace', title: '启动追踪与记录回放', category: '诊断', status: 'available', description: '导入现有static-startup-capture-v1，区分原始RX和采纳反馈。', page: 'records' },
  { id: 'pmax', title: 'PMAX 与位置连续性', category: '维护', status: 'pending', description: '已有独立维护流程；本版查看数据，参数写入尚未接入。', page: 'maintenance' },
  { id: 'persist', title: '驱动掉电保存', category: '维护', status: 'experimental', description: '需完整参数快照、保存回执及真正断电后读回。', page: 'maintenance' },
  { id: 'adrc', title: 'ADRC 诊断与辨识', category: '实验', status: 'experimental', description: '存在独立板端后端，闭环资格与远程调试需单独验收。', page: 'lab' },
  { id: 'trajectory', title: '轨迹编排与多轴联动', category: '规划', status: 'planned', description: '预留扩展位置，尚未接入控制接口。', page: 'lab' },
];
export const STATUS_LABEL = { available: '本版已接入', pending: '远程待接入', experimental: '实验能力', planned: '规划中' };
