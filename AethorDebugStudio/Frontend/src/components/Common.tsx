/** @file 共用状态、数字和说明组件。 */
import { ArrowUpRight, Info } from 'lucide-react';
import type { ReactNode } from 'react';
/** 限定语义色和文本，避免只通过颜色表达状态。 */
export function Badge({ children, tone = 'neutral' }: { children: ReactNode; tone?: 'neutral' | 'green' | 'amber' | 'red' | 'blue' }) { return <span className={'badge ' + tone}><span className="badge-dot"/>{children}</span>; }
/** 显示未知而非0，采用等宽数字。 */
export function Value({ value, digits = 2, unit }: { value: number | null | undefined; digits?: number; unit?: string }) { return <span className="number">{value === null || value === undefined || !Number.isFinite(value) ? '—' : value.toFixed(digits)}{unit && <small>{unit}</small>}</span>; }
/** 提供轻量语义化面板标题。 */
export function PanelHeader({ title, eyebrow, children }: { title: string; eyebrow?: string; children?: ReactNode }) { return <div className="panel-header"><div>{eyebrow && <span className="eyebrow">{eyebrow}</span>}<h2>{title}</h2></div>{children}</div>; }
/** 内联说明，不将工程解释堆入主操作按钮。 */
export function Note({ children, warning = false }: { children: ReactNode; warning?: boolean }) { return <div className={'note ' + (warning ? 'warning' : '')}><Info size={16}/><div>{children}</div></div>; }
/** 用于有清晰下一步的页面跳转。 */
export function TextLink({ children, onClick }: { children: ReactNode; onClick: () => void }) { return <button className="text-link" onClick={onClick}>{children}<ArrowUpRight size={15}/></button>; }
