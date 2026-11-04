/** @file 本机HTTP边界验收：只提交无效端口和拒绝型调试请求，不打开设备或发送动作。 */
import assert from 'node:assert/strict';
import http from 'node:http';
const base='http://127.0.0.1:5179';
let count=0;
/** 请求固定地址并验证状态码，失败立即终止。 */
async function check(path,options,status){const response=await fetch(base+path,options);assert.equal(response.status,status,path);count++;return response;}
const health=await (await check('/api/health',{},200)).json();
assert.equal(health.readOnly,true);assert.equal(health.hardwareTested,false);
const state=await (await check('/api/state',{},200)).json();assert.equal(typeof state.connected,'boolean');assert.equal(state.readOnly,true);
await check('/api/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({port:'INVALID'})},403);
await check('/api/connect',{method:'POST',headers:{'Content-Type':'application/json','X-Aethor-Client':'studio-v1',Origin:'https://untrusted.example'},body:JSON.stringify({port:'INVALID'})},403);
await check('/api/connect',{method:'POST',headers:{'Content-Type':'application/json','X-Aethor-Client':'studio-v1'},body:JSON.stringify({port:'INVALID'})},400);
await check('/api/connect',{method:'POST',headers:{'Content-Type':'text/plain','X-Aethor-Client':'studio-v1'},body:'INVALID'},400);
await check('/api/command',{method:'POST',headers:{'Content-Type':'application/json','X-Aethor-Client':'studio-v1'},body:JSON.stringify({command:'enable'})},404);
await check('/api/missing',{},404);
const debugHeaders={'Content-Type':'application/json','X-Aethor-Client':'studio-v1'};
assert.equal(health.debugAvailable,true);
await check('/api/debug',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({operation:'stop',motorId:7})},403);
await check('/api/debug',{method:'POST',headers:debugHeaders,body:JSON.stringify({operation:'enable',motorId:7})},400);
await check('/api/debug',{method:'POST',headers:debugHeaders,body:JSON.stringify({operation:'stop',motorId:0})},400);
await check('/api/debug',{method:'POST',headers:debugHeaders,body:JSON.stringify({operation:'disable',motorId:7,loadSupported:false})},400);
await check('/api/debug',{method:'POST',headers:debugHeaders,body:JSON.stringify({operation:'stop',motorId:7,command:'enable'})},400);
await check('/api/debug',{method:'POST',headers:debugHeaders,body:'{"operation":'},400);
const hostileHostStatus=await new Promise((resolve,reject)=>{const request=http.request(base+'/api/health',{headers:{Host:'untrusted.example'}},response=>{response.resume();resolve(response.statusCode);});request.on('error',reject);request.end();});
assert.equal(hostileHostStatus,403);count++;
const page=await check('/',{},200);assert.ok((page.headers.get('content-security-policy')??'').includes("frame-ancestors 'none'"));assert.ok((await page.text()).includes('Aethor'));
console.log('HTTP_TESTS passed='+count+' failed=0 real_port_opened_by_test=false hardware_command_sent=false');
