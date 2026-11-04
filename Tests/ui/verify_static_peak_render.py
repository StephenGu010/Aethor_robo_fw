"""Verify and convert actual LVGL static-observation raster evidence, without redrawing it."""
from pathlib import Path
import hashlib
import json
import sys
from PIL import Image


def verify_render(directory: Path) -> None:
    """Require all six unique 280x240 monochrome rasters and preserve their exact PNG pixels."""
    names = (
        "310_takeover_current_and_peak", "311_takeover_feedback_expired", "312_hold_current_and_peak",
        "313_hold_feedback_expired", "314_takeover_first_cause_after_stop", "315_hold_first_cause_after_stop",
    )
    images = []
    hashes = set()
    for name in names:
        source = directory / f"{name}.ppm"
        with Image.open(source) as raster:
            assert raster.size == (280, 240), source
            assert len(raster.getcolors(280 * 240) or []) > 15, source
            assert max(max(pixel) - min(pixel) for pixel in raster.get_flattened_data()) <= 8, source
            digest = hashlib.sha256(raster.tobytes()).hexdigest()
            assert digest not in hashes, source
            hashes.add(digest)
            target = source.with_suffix(".png")
            raster.save(target, format="PNG")
            with Image.open(target) as persisted:
                assert persisted.tobytes() == raster.tobytes(), target
            images.append({"image": target.name, "rgb_sha256": digest, "width": 280, "height": 240})
    (directory / "render_manifest.json").write_text(json.dumps({
        "renderer": "actual LVGL 8.3.11 software raster + production double-buffer flush port",
        "synthetic_feedback_only": True,
        "ui_refresh_period_ms": 100,
        "physical_feedback_period_qualified": False,
        "images": images,
    }, indent=2) + "\n", encoding="utf-8")
    print(f"STATIC_PEAK_RASTER_PASS images={len(images)} exact_pixels=1 monochrome=1")


if __name__ == "__main__":
    verify_render(Path(sys.argv[1]))
