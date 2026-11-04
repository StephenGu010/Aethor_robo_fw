/** @file 固定同源客户端；只读采集与显式停止/失能事务分开，无任意命令接口。 */
import type { DebugOperation, DebugReceipt, GatewaySnapshot } from './types';
/** 仅向固定相对路径请求JSON并检查HTTP错误。 */
async function request<T>(path: string, body?: object, timeoutMs = 5000): Promise<T> {
  const response = await fetch('/api/' + path, { method: body ? 'POST' : 'GET', headers: body ? { 'Content-Type': 'application/json', 'X-Aethor-Client': 'studio-v1' } : {}, body: body ? JSON.stringify(body) : undefined, signal: AbortSignal.timeout(timeoutMs) });
  if (!response.ok) { const error = await response.json().catch(() => ({})); throw new Error(error.error ?? '本地网关未响应'); }
  return response.json() as Promise<T>;
}
export const gatewayApi = {
  /** 枚举不打开串口。 */
  ports: () => request<string[]>('ports'),
  /** 读取主机缓存，不发送浏览器指定的设备命令。 */
  state: () => request<GatewaySnapshot>('state'),
  /** 用户显式连接；服务端仅发送固定只读查询。 */
  connect: (port: string) => request<GatewaySnapshot>('connect', { port }),
  /** 释放串口，不发送电机动作。 */
  disconnect: () => request<GatewaySnapshot>('disconnect', {}),
  /** 显式提交一次固定事务，等待回执；客户端不做重试。 */
  debug: (operation: DebugOperation, motorId: number, loadSupported = false) => request<DebugReceipt>('debug', { operation, motorId, loadSupported }, 20000),
};
