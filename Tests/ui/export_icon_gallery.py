"""导出真实 LVGL 模拟快照画廊；这些图片不是烧录后实机屏幕证据。

读取已验证渲染清单及 PNG，复制原图并生成每页最多 12 张的联系图。
所有缩放使用 2 倍最近邻；输入文件始终只读。
"""

import argparse
import hashlib
import html
import json
import shutil
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def read_images(source: Path) -> list[tuple[str, Image.Image]]:
    """校验清单尺寸与 RGB 哈希，拒绝缺图、额外图片或目录穿越路径。"""
    manifest = json.loads((source / "render_manifest.json").read_text(encoding="utf-8-sig"))
    images = []
    seen_names = set()
    for entry in manifest["images"]:
        name = entry["image"]
        if Path(name).name != name or not name.lower().endswith(".png") or name in seen_names:
            raise ValueError(f"清单图片名无效或重复: {name}")
        seen_names.add(name)
        with Image.open(source / name) as original:
            pixels = original.convert("RGB")
        if pixels.size != (entry["width"], entry["height"]):
            raise ValueError(f"图片尺寸与清单不一致: {name}")
        if hashlib.sha256(pixels.tobytes()).hexdigest() != entry["rgb_sha256"]:
            raise ValueError(f"图片 RGB 哈希与清单不一致: {name}")
        images.append((name, pixels))
    actual_names = {path.name for path in source.glob("*.png")}
    if not images or actual_names != seen_names:
        raise ValueError(f"清单与 PNG 集合不一致: {actual_names ^ seen_names}")
    return sorted(images, key=lambda entry: entry[0])


def draw_contact_sheet(images: list[tuple[str, Image.Image]], output: Path) -> None:
    """以三列四行上限导出带文件名及 SIM 标识的 2 倍最近邻联系图。"""
    font = ImageFont.load_default()
    cell_width = max(max(pixels.width * 2, int(font.getlength(name)) + 20)
                     for name, pixels in images) + 24
    cell_height = max(pixels.height for _, pixels in images) * 2 + 64
    columns = min(3, len(images))
    rows = (len(images) + columns - 1) // columns
    sheet = Image.new("RGB", (columns * cell_width, rows * cell_height), "#101820")
    drawing = ImageDraw.Draw(sheet)
    for index, (name, pixels) in enumerate(images):
        left = (index % columns) * cell_width + 12
        top = (index // columns) * cell_height + 10
        drawing.text((left, top), name, font=font, fill="white")
        drawing.text((left, top + 18), "SIMULATED / REAL LVGL / NOT HARDWARE", font=font, fill="#56d7e6")
        doubled = pixels.resize((pixels.width * 2, pixels.height * 2), Image.Resampling.NEAREST)
        sheet.paste(doubled, (left, top + 40))
    sheet.save(output)


def export_gallery(source: Path, output: Path) -> None:
    """复制原始 PNG 并输出离线 HTML 和联系图，保持模拟证据说明可见。"""
    source = source.resolve()
    output = output.resolve()
    if output == source or source in output.parents or output in source.parents:
        raise ValueError("输出目录必须与输入目录分离，且不能相互包含")
    images = read_images(source)
    screens = output / "screens"
    screens.mkdir(parents=True, exist_ok=True)
    cards = []
    for name, _ in images:
        shutil.copy2(source / name, screens / name)
        safe_name = html.escape(name, quote=True)
        cards.append(f'<figure><a href="screens/{safe_name}"><img src="screens/{safe_name}" '
                     f'alt="{safe_name} 模拟快照" loading="lazy"></a>'
                     f'<figcaption>{safe_name}<br><strong>SIM · 模拟数据</strong></figcaption></figure>')
    contact_links = []
    for start in range(0, len(images), 12):
        contact_name = f"contact_{start // 12 + 1:02d}.png"
        draw_contact_sheet(images[start:start + 12], output / contact_name)
        contact_links.append(f'<a href="{contact_name}">{contact_name}</a>')
    document = '''<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LCD 图标 UI · LVGL 模拟快照</title>
<style>
body{margin:24px;background:#101820;color:#edf4f7;font:16px/1.6 system-ui,sans-serif}
h1{font-size:26px}a{color:#56d7e6}.gallery{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,584px),1fr));gap:20px}
figure{margin:0;padding:12px;background:#192832;border-radius:8px;overflow:auto}
img{width:560px;height:auto;max-width:100%;image-rendering:pixelated}figcaption{overflow-wrap:anywhere}strong{color:#56d7e6}
</style><h1>LCD 图标 UI · 全部页面画廊</h1>
<p>这些图片由真实 LVGL 8.3.11 软件栅格器生成，使用模拟数据；不是烧录后的实机屏幕，也不代表硬件验证通过。
原始 PNG 保存在 screens，页面显示和联系图采用 2 倍最近邻放大。</p>
'''
    document += f'<p>共 {len(images)} 张。联系图：{" · ".join(contact_links)}</p>'
    document += '<main class="gallery">' + "\n".join(cards) + '</main></html>\n'
    (output / "index.html").write_text(document, encoding="utf-8")
    print(f"已导出 {len(images)} 张模拟快照: {output / 'index.html'}")


def main() -> None:
    """解析显式输出目录；默认输入为此测试目录下的 artifacts。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parent / "artifacts")
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    export_gallery(arguments.source, arguments.output)


if __name__ == "__main__":
    main()
