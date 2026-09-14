"""Generate sparse Noto CJK popup fonts matching the approved design typography.

SIL OFL 1.1; the original font hash is pinned in SOURCE.json. Only popup text,
digits and units are included. No complete additional CJK font is embedded.
"""
from pathlib import Path
import argparse
import hashlib
import json
from PIL import Image, ImageDraw, ImageFont

FONT_SETS = {
    11: ' SIM',
    12: ' 每步0123456789.+-/°ms仅修改草稿上下调值中保存左取消',
    14: ' 0123456789.+-/°ms>naif',
    17: ' 0123456789/POSMITHKpd相对角度输出轴速度保持时间移动',
    30: ' 0123456789.+-/°msnaif',
}


def font_source(path: Path, size: int, characters: str) -> tuple[str, dict]:
    """Emit one LVGL font preserving pinned Noto glyph metrics and sparse bitmaps."""
    face = ImageFont.truetype(str(path), size)
    prefix = f'popup_{size}'
    points = sorted(set(map(ord, characters)))
    bitmap, descriptors = [], ['    {0, 0, 0, 0, 0, 0},']
    for point in points:
        glyph = chr(point)
        left, top, right, bottom = face.getbbox(glyph, anchor='ls')
        width, height, start = right-left, bottom-top, len(bitmap)
        if width and height:
            image = Image.new('L', (width, height), 0)
            ImageDraw.Draw(image).text((-left, -top), glyph, font=face, fill=255, anchor='ls')
            pixels = [value*15//255 for value in image.get_flattened_data()]
            if len(pixels) % 2:
                pixels.append(0)
            bitmap.extend(pixels[index]*16+pixels[index+1] for index in range(0, len(pixels), 2))
        descriptors.append(f'    {{{start}, {round(face.getlength(glyph)*16)}, {width}, {height}, {left}, {-bottom}}}, /* U+{point:04X} */')
    lines = [f'static const uint8_t {prefix}_bitmap[] = {{']
    lines.extend('    '+', '.join(f'0x{value:02x}' for value in bitmap[index:index+20])+',' for index in range(0, len(bitmap), 20))
    lines += ['};', f'static const lv_font_fmt_txt_glyph_dsc_t {prefix}_glyphs[] = {{', *descriptors, '};',
              f'static const uint16_t {prefix}_unicode[] = {{'+', '.join(str(point-32) for point in points)+'};',
              f'static const lv_font_fmt_txt_cmap_t {prefix}_cmap[] = {{{{',
              f'    .range_start=32, .range_length={points[-1]-31}, .glyph_id_start=1,',
              f'    .unicode_list={prefix}_unicode, .glyph_id_ofs_list=NULL, .list_length={len(points)},',
              '    .type=LV_FONT_FMT_TXT_CMAP_SPARSE_TINY', '}};',
              f'static lv_font_fmt_txt_glyph_cache_t {prefix}_cache;',
              f'static const lv_font_fmt_txt_dsc_t {prefix}_description = {{',
              f'    .glyph_bitmap={prefix}_bitmap, .glyph_dsc={prefix}_glyphs, .cmaps={prefix}_cmap,',
              '    .kern_dsc=NULL, .kern_scale=0, .cmap_num=1, .bpp=4,',
              f'    .kern_classes=0, .bitmap_format=0, .cache=&{prefix}_cache', '};',
              f'/** @brief Sparse {size}px Noto popup font, SIL OFL 1.1. */',
              f'const lv_font_t ui_font_popup_{size} = {{',
              '    .get_glyph_dsc=lv_font_get_glyph_dsc_fmt_txt, .get_glyph_bitmap=lv_font_get_bitmap_fmt_txt,',
              f'    .line_height={size+4}, .base_line=4, .subpx=LV_FONT_SUBPX_NONE,',
              f'    .underline_position=-2, .underline_thickness=1, .dsc=&{prefix}_description,',
              '    .fallback=NULL, .user_data=NULL', '};']
    return '\n'.join(lines), {'size_px': size, 'glyphs': len(points), 'bitmap_bytes': len(bitmap), 'characters': ''.join(map(chr, points))}


def generate(path: Path, check: bool) -> None:
    """Pin the source and verify reproducibility before accepting generated fonts."""
    directory = Path(__file__).resolve().parent
    source = json.loads((directory/'SOURCE.json').read_text())
    assert hashlib.sha256(path.read_bytes()).hexdigest() == source['source_sha256']
    fragments, metadata = [], []
    for size, characters in FONT_SETS.items():
        fragment, record = font_source(path, size, characters)
        fragments.append(fragment)
        metadata.append(record)
    contents = '/** @file ui_font_popup.c\n * @brief Generated sparse Noto popup fonts; SIL OFL 1.1, see OFL.txt.\n * Reproduce with Ui/fonts/generate_popup_fonts.py; no full CJK font payload.\n */\n#include "lvgl.h"\n' + '\n'.join(fragments)+'\n'
    destination = directory/'ui_font_popup.c'
    manifest = {'source_sha256': source['source_sha256'], 'source': source['source'],
                'copyright': source['copyright'], 'license': 'SIL OFL 1.1', 'fonts': metadata}
    if check:
        assert destination.read_text(encoding='utf-8') == contents
        assert json.loads((directory/'POPUP_SOURCE.json').read_text(encoding='utf-8')) == manifest
    else:
        destination.write_text(contents, encoding='utf-8')
        (directory/'POPUP_SOURCE.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('POPUP_FONTS_VERIFIED', sum(record['bitmap_bytes'] for record in metadata), 'bitmap bytes')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('font', type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    generate(args.font, args.check)
