/** @file ID7法兰显示与输出轴协议的双向换算测试，不访问真实设备。 */
import {test} from 'node:test';
import assert from 'node:assert/strict';
import {OUTPUT_PER_FLANGE,displayDegreesToOutputRadians,outputRadiansToDisplayDegrees,outputDegreesToDisplayDegrees} from '../src/axisUnits';
import {createElement} from 'react';
import {renderToStaticMarkup} from 'react-dom/server';
import {StaticWindows,StaticObservation,GainBudget} from '../src/components/StaticInsights';
import {createDemoRecording} from '../src/demo';
import {DEFAULT_SETTINGS} from '../src/catalog';

test('法兰角度和速度采用44.8/19.2032，不叠乘总减速比',()=>{
  assert.ok(Math.abs(OUTPUT_PER_FLANGE-44.8/19.2032)<1e-12);
  const outputRadians=displayDegreesToOutputRadians(10,'flange');
  assert.ok(Math.abs(outputRadians-Math.PI/18*44.8/19.2032)<1e-12);
  assert.ok(Math.abs(outputRadiansToDisplayDegrees(outputRadians,'flange')-10)<1e-12);
  assert.ok(Math.abs(outputDegreesToDisplayDegrees(15,'flange')-15*19.2032/44.8)<1e-12);
});
test('输出轴入口保留原换算，负方向、零及未知值不伪造',()=>{
  assert.ok(Math.abs(displayDegreesToOutputRadians(-10,'output')+Math.PI/18)<1e-12);
  assert.equal(outputRadiansToDisplayDegrees(0,'flange'),0);
  assert.ok(Number.isNaN(displayDegreesToOutputRadians(NaN,'flange')));
});
test('静载窗口与误差以法兰显示，原输出轴策略判据和力矩预算不变',()=>{
  const windows=renderToStaticMarkup(createElement(StaticWindows,{motorId:7,displayCoordinate:'flange'}));
  assert.match(windows,/±0\.214/);assert.match(windows,/±0\.429/);assert.match(windows,/法兰端/);
  const recording=createDemoRecording(DEFAULT_SETTINGS,'static');
  recording.staticContext!.holdStartMs=1000;
  const samples=[{...recording.samples[0],timeMs:1200,positionDeg:.75,targetDeg:0}];
  const observation=renderToStaticMarkup(createElement(StaticObservation,{recording,samples,displayCoordinate:'flange'}));
  assert.match(observation,/可见样本超差/);assert.match(observation,/0\.321/);assert.match(observation,/法兰端跟随误差窗口图/);
  const budget=renderToStaticMarkup(createElement(GainBudget,{displayCoordinate:'flange'}));
  assert.match(budget,/°\/s 法兰端/);assert.match(budget,/1\.396/);assert.match(budget,/0\.200/);
});
