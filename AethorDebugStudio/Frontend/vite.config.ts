/** @file 本地开发与构建配置；网关仅通过本机同源代理访问。 */
import { defineConfig } from 'vite';
export default defineConfig({ server: { strictPort: true, proxy: { '/api': 'http://127.0.0.1:5179' } }, build: { outDir: 'dist', emptyOutDir: false } });
