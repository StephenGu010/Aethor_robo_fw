/** @file 固定浏览器缓存位于项目目录，避免Windows沙箱每进程LocalAppData变化。 */
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const install=process.argv.includes('--install');
const args=install?['install','--only-shell','chromium']:['test',...process.argv.slice(2)];
const result=spawnSync(process.execPath,[path.join(root,'Frontend/node_modules/@playwright/test/cli.js'),...args],{cwd:path.join(root,'Frontend'),env:{...process.env,PLAYWRIGHT_BROWSERS_PATH:path.join(root,'.playwright'),STUDIO_TEST_RUN:String(Date.now())},stdio:'inherit'});
if(result.error)console.error(result.error.message);
process.exit(result.status??1);
