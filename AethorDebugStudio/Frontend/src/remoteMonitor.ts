/** @file 有界只读事务跟踪；不提交动作、不重试失败请求、不跨连接接收结果。 */
import type { RemoteReceipt } from './remoteControl';

/** 可注入的只读传输与连接生命周期，用于实际页面及无硬件测试。 */
export interface RemoteMonitorOptions {
  request: (body: { operation: 'result'; transactionId: number }) => Promise<RemoteReceipt>;
  wait: () => Promise<void>;
  isCurrent: () => boolean;
  observe: (receipt: RemoteReceipt) => void;
  maxPolls?: number;
}

/** 等待原事务终态；未知或有界等待耗尽时由用户恢复原事务，绝不重复动作。 */
export async function monitorRemoteTransaction(transactionId: number, options: RemoteMonitorOptions): Promise<string> {
  const maxPolls = options.maxPolls ?? 180;
  if (!Number.isSafeInteger(transactionId) || transactionId <= 0 || !Number.isInteger(maxPolls) || maxPolls < 1 || maxPolls > 180) return 'unknown';
  for (let index = 0; index < maxPolls; index++) {
    if (!options.isCurrent()) return 'cancelled';
    try {
      const result = await options.request({ operation: 'result', transactionId });
      if (!options.isCurrent()) return 'cancelled';
      if (result.operation !== 'result' || result.fields.id !== String(transactionId) ||
          !['pending', 'completed', 'failed', 'rejected'].includes(result.result)) return 'unknown';
      options.observe(result);
      if (result.result !== 'pending') return result.result;
      if (index + 1 < maxPolls) await options.wait();
    } catch { return options.isCurrent() ? 'unknown' : 'cancelled'; }
  }
  return 'timeout';
}
