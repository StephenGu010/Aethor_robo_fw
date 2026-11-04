/** @file 已核实ID7的角度/速度显示换算；控制草稿和协议始终保留输出轴rad/rad/s。 */
export type ControlCoordinate='output'|'flange';
export const INTERNAL_RATIO=19.2032;
export const ROTOR_PER_FLANGE=44.8;
export const OUTPUT_PER_FLANGE=ROTOR_PER_FLANGE/INTERNAL_RATIO;

/** 将显示的度或度/秒转回既有输出轴rad或rad/s；保留无效输入供校验。 */
export function displayDegreesToOutputRadians(value:number,coordinate:ControlCoordinate):number {
  return value*Math.PI/180*(coordinate==='flange'?OUTPUT_PER_FLANGE:1);
}
/** 将输出轴rad或rad/s投影为当前坐标的度或度/秒，不能改变协议值。 */
export function outputRadiansToDisplayDegrees(value:number,coordinate:ControlCoordinate):number {
  return value*180/Math.PI/(coordinate==='flange'?OUTPUT_PER_FLANGE:1);
}
/** 将输出轴角度、角速度或保护线只作显示换算；原策略判断保持原坐标。 */
export function outputDegreesToDisplayDegrees(value:number,coordinate:ControlCoordinate):number {
  return value/(coordinate==='flange'?OUTPUT_PER_FLANGE:1);
}
