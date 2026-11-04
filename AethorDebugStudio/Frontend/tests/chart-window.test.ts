/** @file 实时滚动窗口只控制显示，不截断原始采样或历史回放。 */
import assert from 'node:assert/strict';
import test from 'node:test';
import React from 'react';
import {renderToStaticMarkup} from 'react-dom/server';
import {Chart} from '../src/components/Chart';
import type {Sample} from '../src/types';
/** 创建带有早期极值和最近有效样本的工程记录。 */
function samples():Sample[]{return [0,10000,40000,70000].map(timeMs=>({timeMs,positionDeg:timeMs===0?999:1,targetDeg:null,speedDegS:0,torqueNm:null,mosC:null,rotorC:null}));}
test('实时图只显示最近60秒，旧点不影响纵轴且原始样本保留',()=>{
  const source=samples();
  const markup=renderToStaticMarkup(React.createElement(Chart,{samples:source,windowMs:60000} as React.ComponentProps<typeof Chart>));
  assert.match(markup,/>10\.0s</);assert.match(markup,/>70\.0s</);assert.doesNotMatch(markup,/>0\.0s</);
  assert.match(markup,/最近60秒/);assert.doesNotMatch(markup,/1248\.50/);
  assert.equal(source.length,4);assert.equal(source[0].positionDeg,999);
});
test('历史图默认展示整段记录，实时窗口内未知值不补零',()=>{
  const all=renderToStaticMarkup(React.createElement(Chart,{samples:samples()}));assert.match(all,/>0\.0s</);
  const missing=samples().map(sample=>({...sample,positionDeg:sample.timeMs===0?999:null}));
  const markup=renderToStaticMarkup(React.createElement(Chart,{samples:missing,windowMs:60000} as React.ComponentProps<typeof Chart>));
  assert.match(markup,/等待有效采样/);assert.doesNotMatch(markup,/<polyline|<circle/);
});
test('停止采集后窗口继续随时钟推进，超过60秒的旧样本离开视窗',()=>{
  const markup=renderToStaticMarkup(React.createElement(Chart,{samples:samples(),windowMs:60000,windowEndMs:131000} as React.ComponentProps<typeof Chart>));
  assert.match(markup,/等待有效采样/);assert.doesNotMatch(markup,/<polyline|<circle/);
});
