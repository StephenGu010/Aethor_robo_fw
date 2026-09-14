#!/usr/bin/env python3
"""Reproduce and verify the numeric LVGL 8.3 Montserrat 28 subset without font tools."""

import argparse
import hashlib
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "Middlewares/Third_Party/LVGL/src/font/lv_font_montserrat_28.c"
OUTPUT = Path(__file__).with_name("ui_font_numeric_28.c")
SOURCE_SHA256 = "c963a5d3f55ed415b2d991f8ea4766797f98692f9253db3d079caf34bc1a661e"
CHARACTERS = " %+ -./0123456789afimns\u00b0"


def array_body(source_text, array_name):
    """Read one simple upstream C array, failing if its declaration changes."""
    match = re.search(r"\b" + array_name + r"\[\]\s*=\s*\{(.*?)\n\};", source_text, re.S)
    if match is None:
        raise ValueError(f"Missing upstream array: {array_name}")
    return match.group(1)


def generate():
    """Copy selected glyph bytes and metrics and generate a sparse sequential cmap."""
    source_bytes = SOURCE.read_bytes()
    if hashlib.sha256(source_bytes).hexdigest() != SOURCE_SHA256:
        raise ValueError("Upstream font hash changed; review glyph mapping before regenerating")
    source_text = source_bytes.decode("utf-8")
    bitmap_text = re.sub(r"/\*.*?\*/", "", array_body(source_text, "glyph_bitmap"), flags=re.S)
    source_bitmap = bytes(int(value, 16) for value in re.findall(r"0x[0-9a-fA-F]+", bitmap_text))
    descriptors = [dict((name, int(value)) for name, value in re.findall(r"\.(\w+)\s*=\s*(-?\d+)", entry))
                   for entry in re.findall(r"\{([^{}]+)\}", array_body(source_text, "glyph_dsc"))]
    codepoints = sorted(set(map(ord, CHARACTERS)))
    glyph_bitmap = bytearray()
    glyph_entries = ["    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0},"]
    bitmap_lines = []
    for codepoint in codepoints:
        source_id = codepoint - 31 if 32 <= codepoint <= 126 else 96
        assert codepoint <= 126 or codepoint == 176
        descriptor = descriptors[source_id].copy()
        bitmap_start = descriptor["bitmap_index"]
        bitmap_length = (descriptor["box_w"] * descriptor["box_h"] + 1) // 2
        original_bitmap = source_bitmap[bitmap_start:bitmap_start + bitmap_length]
        assert len(original_bitmap) == bitmap_length
        assert descriptors[source_id + 1]["bitmap_index"] - bitmap_start == bitmap_length
        descriptor["bitmap_index"] = len(glyph_bitmap)
        glyph_bitmap.extend(original_bitmap)
        assert bytes(glyph_bitmap[descriptor["bitmap_index"]:]) == original_bitmap
        assert all(descriptor[key] == descriptors[source_id][key] for key in descriptor if key != "bitmap_index")
        bitmap_lines.append(f"    /* U+{codepoint:04X} */")
        for offset in range(0, bitmap_length, 12):
            bitmap_lines.append("    " + ", ".join(f"0x{value:02x}" for value in original_bitmap[offset:offset + 12]) + ",")
        glyph_entries.append("    {" + ", ".join(f".{key} = {value}" for key, value in descriptor.items()) + f"}}, /* U+{codepoint:04X} */")
    output_text = '''/**
 * @file ui_font_numeric_28.c
 * @brief Generated Montserrat 28 numeric subset; keep original 4-bpp glyphs/metrics.
 * Regenerate: python Ui/fonts/generate_numeric_28.py
 * Source and license: NUMERIC_28_SOURCE.md; kerning intentionally disabled.
 */
#include "lvgl.h"

/* Original glyph pixels, copied without resampling or compression. */
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
''' + "\n".join(bitmap_lines) + '''
};

/* Glyph zero is reserved; subsequent IDs follow sorted Unicode codepoints. */
static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
''' + "\n".join(glyph_entries) + '''
};

static const uint16_t unicode_list[] = {
    ''' + ", ".join(str(codepoint - codepoints[0]) for codepoint in codepoints) + '''
};

static const lv_font_fmt_txt_cmap_t cmaps[] = {
    {.range_start = 32, .range_length = 145, .glyph_id_start = 1,
     .unicode_list = unicode_list, .glyph_id_ofs_list = NULL,
     .list_length = ''' + str(len(codepoints)) + ''', .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY}
};

static lv_font_fmt_txt_glyph_cache_t cache;
static const lv_font_fmt_txt_dsc_t font_dsc = {
    .glyph_bitmap = glyph_bitmap, .glyph_dsc = glyph_dsc, .cmaps = cmaps,
    .kern_dsc = NULL, .kern_scale = 0, .cmap_num = 1, .bpp = 4,
    .kern_classes = 0, .bitmap_format = 0, .cache = &cache
};

/* Public font descriptor keeps the upstream baseline and line height. */
const lv_font_t ui_font_numeric_28 = {
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,
    .line_height = 30, .base_line = 5, .subpx = LV_FONT_SUBPX_NONE,
    .underline_position = -2, .underline_thickness = 1, .dsc = &font_dsc
};
'''
    return output_text, len(codepoints), len(glyph_bitmap)


def main():
    """Write the deterministic subset or check the committed artifact byte-for-byte."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Verify existing generated artifact without writing")
    arguments = parser.parse_args()
    output_text, glyph_count, bitmap_size = generate()
    expected_bytes = output_text.encode("utf-8")
    if arguments.check:
        if OUTPUT.read_bytes() != expected_bytes:
            raise ValueError("Generated font differs; regenerate it")
    else:
        OUTPUT.write_bytes(expected_bytes)
    print(f"Verified {glyph_count} glyphs: original pixels/metrics match; bitmap={bitmap_size} bytes; source SHA256 verified")


if __name__ == "__main__":
    main()
