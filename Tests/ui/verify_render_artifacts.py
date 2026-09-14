"""Verify real LVGL PPM output and convert it to reviewable PNG files.

This is deterministic file-format conversion of actual raster output. It also
checks the versioned font inventory against every current UI/model C literal.
"""
from pathlib import Path
import hashlib
import json
import re
import sys
from PIL import Image


def verify(directory: Path) -> None:
    """Check image size/content/uniqueness and every source Unicode codepoint."""
    root = Path(__file__).resolve().parents[2]
    inventory = set((root / "Ui/fonts/charset.txt").read_text(encoding="utf-8"))
    for source in (root / "Ui/debug_ui_view.c", root / "Ui/debug_ui_astra_layout.inc",
                   root / "App/DebugUi/debug_ui_model.c"):
        for literal in re.findall(r'"(?:\\.|[^"\\])*"', source.read_text(encoding="utf-8")):
            missing = {character for character in literal if ord(character) > 127 and character not in inventory}
            assert not missing, f"{source}: missing font inventory {missing}"
    manifest = []
    hashes = set()
    for path in sorted(directory.glob("*.ppm")):
        with Image.open(path) as raster:
            assert raster.size == (280, 240)
            assert len(raster.getcolors(280 * 240) or []) > 15, f"Blank/limited raster: {path}"
            digest = hashlib.sha256(raster.tobytes()).hexdigest()
            assert digest not in hashes, f"Duplicate rendered page: {path}"
            hashes.add(digest)
            png = path.with_suffix(".png")
            raster.save(png)
            manifest.append({"image": png.name, "rgb_sha256": digest, "width": 280, "height": 240})
    required = {"17_review_set_pos.png", "18_review_set_mit.png", "19_remote_id3.png",
                "20_stop_latched.png", "21_stop_latch_confirmed.png", "22_stop_latch_unconfirmed.png",
                "23_remote_multiple_targets.png"}
    assert required <= {item["image"] for item in manifest}, "Missing review-regression renders"
    assert len(manifest) >= 23
    (directory / "render_manifest.json").write_text(json.dumps({
        "renderer": "actual LVGL 8.3.11 software raster + production double-buffer flush port",
        "data": "SIMULATED ONLY; no hardware validation", "images": manifest,
        "all_current_literal_codepoints_in_font_inventory": True,
    }, indent=2) + "\n", encoding="utf-8")
    print(f"RENDER_ARTIFACTS_OK: {len(manifest)} distinct 280x240 pages; current Unicode inventory complete")


if __name__ == "__main__":
    verify(Path(sys.argv[1]))
