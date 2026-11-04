/** @file 新手流程独立浏览器验收；只启动前端，全部API由测试拦截。 */
import {defineConfig} from '@playwright/test';
export default defineConfig({testDir:'./tests/browser',testMatch:'usability.spec.ts',workers:1,reporter:'list',outputDir:'../artifacts/usability-browser-'+(process.env.STUDIO_TEST_RUN??Date.now()),use:{baseURL:'http://127.0.0.1:5178',viewport:{width:1440,height:1000},trace:'retain-on-failure'},webServer:{command:'npm run dev',url:'http://127.0.0.1:5178',reuseExistingServer:true}});
