/** @file 配额恢复隔离验收：独立前端端口，测试API全部拦截，不启动真实网关。 */
import {defineConfig} from '@playwright/test';
export default defineConfig({testDir:'./tests/browser',testMatch:['torque-preparation.spec.ts','adjustable-torque.spec.ts','torque-envelope.spec.ts','remote-safety.spec.ts','session-recovery.spec.ts'],workers:1,reporter:'list',outputDir:'../artifacts/quota-browser-'+Date.now(),use:{baseURL:'http://127.0.0.1:5180',viewport:{width:1440,height:1000},trace:'retain-on-failure'},webServer:{command:'npm run dev -- --port 5180',url:'http://127.0.0.1:5180',reuseExistingServer:true}});
