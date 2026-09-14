# 数值字体来源与复现

`ui_font_numeric_28.c` 导出 `ui_font_numeric_28`，供 LVGL 8.3.11 使用。它从仓库现有 `Middlewares/Third_Party/LVGL/src/font/lv_font_montserrat_28.c` 提取 23 个字符：空格、`% + - . /`、`0–9`、`a f i m n s`、`°`。额外字母允许 `nan` / `inf` 与 `ms` 文本正常显示。中文及普通文本仍使用既有字体。

上游源文件 SHA-256：`c963a5d3f55ed415b2d991f8ea4766797f98692f9253db3d079caf34bc1a661e`。原字体文件完整保留。LVGL 来源：https://github.com/lvgl/lvgl/tree/v8.3.11 ，许可证保存在 `Middlewares/Third_Party/LVGL/LICENCE.txt`。原字体头记录 Montserrat-Medium.ttf、28 px、4 bpp、未压缩转换参数。本子集不包含 Font Awesome 图标。

在固件根目录运行：

```powershell
python Ui/fonts/generate_numeric_28.py
python Ui/fonts/generate_numeric_28.py --check
```

生成脚本先校验原文件哈希，再逐字形提取位图并核对原索引跨度、字形尺寸和原始字节；保留 advance、偏移、行高 30、基线 5。生成后连续 glyph ID 通过 sparse cmap 对应 Unicode，保留 ID 0。位图总计 2763 字节；这不是目标 map 的最终 RO 占用。字距调整明确关闭，单字形视觉与原字体一致，多字组合间距可能与完整字体的 kerning 有差异。`--check` 还检查生成文件逐字节可复现。

## MIT notice

Copyright (c) 2021 LVGL Kft

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
