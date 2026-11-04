/** @file 自动跟踪只读事务的离线测试；不连接串口或真实设备。 */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { monitorRemoteTransaction } from '../src/remoteMonitor';
import type { RemoteReceipt } from '../src/remoteControl';

/** 构造固定原事务的设备回复。 */
function receipt(result: string, id = '42'): RemoteReceipt {
  return { requestId: 99, operation: 'result', result, fields: { id }, detail: result };
}
test('自动查询原事务到终态，永不再次提交动作', async () => {
  const requests: object[] = [];
  const results = [receipt('pending'), receipt('completed')];
  const observed: string[] = [];
  const outcome = await monitorRemoteTransaction(42, {
    request: async body => { requests.push(body); return results.shift()!; },
    wait: async () => {}, isCurrent: () => true,
    observe: value => observed.push(value.result), maxPolls: 3,
  });
  assert.equal(outcome, 'completed');
  assert.deepEqual(requests, [{ operation: 'result', transactionId: 42 }, { operation: 'result', transactionId: 42 }]);
  assert.deepEqual(observed, ['pending', 'completed']);
});
test('错号、未知、异常立即停止，不补发', async () => {
  for (const value of [receipt('completed', '41'), receipt('unknown'), new Error('HTTP timeout')]) {
    let calls = 0;
    const outcome = await monitorRemoteTransaction(42, {
      request: async () => { calls++; if(value instanceof Error) throw value; return value; },
      wait: async () => {}, isCurrent: () => true, observe: () => {}, maxPolls: 3,
    });
    assert.equal(outcome, 'unknown'); assert.equal(calls, 1);
  }
});
test('断连或换轴后不再查询，不接收旧连接终态', async () => {
  let current = true; let observed = 0;
  const outcome = await monitorRemoteTransaction(42, {
    request: async () => { current = false; return receipt('completed'); },
    wait: async () => {}, isCurrent: () => current, observe: () => observed++, maxPolls: 3,
  });
  assert.equal(outcome, 'cancelled'); assert.equal(observed, 0);
});
test('持续pending有界退出，保留原事务供恢复', async () => {
  let calls = 0;
  const outcome = await monitorRemoteTransaction(42, {
    request: async () => { calls++; return receipt('pending'); },
    wait: async () => {}, isCurrent: () => true, observe: () => {}, maxPolls: 2,
  });
  assert.equal(outcome, 'timeout'); assert.equal(calls, 2);
});
