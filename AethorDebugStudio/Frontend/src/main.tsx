/** @file React入口及错误边界；渲染故障不触发设备操作。 */
import { Component, type ErrorInfo, type ReactNode, StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import App from './App';
/** 将页面异常转换为可恢复状态，不声称硬件已停止。 */
class StudioErrorBoundary extends Component<{children:ReactNode},{failed:boolean}> {
  state={failed:false};
  /** 在后代渲染异常时切换到错误说明。 */
  static getDerivedStateFromError(){return {failed:true};}
  /** 记录本地渲染异常，不收集设备或凭证。 */
  componentDidCatch(error:Error,info:ErrorInfo){console.error('工作台渲染异常',error.message,info.componentStack);}
  /** 提供刷新入口和真实硬件状态边界。 */
  render(){return this.state.failed?<div className="fatal-error"><h1>工作台遇到显示错误</h1><p>刷新页面可重新载入。此提示不能代表机械臂已经停止；真实停止请使用既有LCD入口。</p><button onClick={()=>window.location.reload()}>重新载入</button></div>:this.props.children;}
}
createRoot(document.getElementById('root')!).render(<StrictMode><StudioErrorBoundary><App/></StudioErrorBoundary></StrictMode>);
