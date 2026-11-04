"""Verify PNG persistence preserves exact pixels and original render freshness.

All fixtures are retained under the render build directory; no cleanup or deletion occurs.
"""
from pathlib import Path
import errno
import os
import time
from unittest.mock import patch
from PIL import Image
from verify_render_artifacts import save_png_if_changed


def verify_persistence() -> None:
    """Exercise equal/changed pixels, stale sources and bounded transient/permanent errors."""
    fixture = Path(__file__).resolve().parent / "build/render" / f"persistence_fixture_{time.time_ns()}"
    fixture.mkdir(parents=True)
    source = fixture / "render.ppm"
    target = fixture / "render.png"
    first_timestamp = 1_600_000_000_000_000_000
    fresh_timestamp = first_timestamp + 10_000_000_000
    raster = Image.new("RGB", (280, 240), "black")
    raster.save(source)
    os.utime(source, ns=(first_timestamp, first_timestamp))
    save_png_if_changed(raster, target, source.stat().st_mtime_ns)
    initial_payload = target.read_bytes()
    assert target.stat().st_mtime_ns == first_timestamp
    os.utime(source, ns=(fresh_timestamp, fresh_timestamp))
    with patch.object(Path, "write_bytes", side_effect=AssertionError("Equal pixels must not be rewritten")):
        save_png_if_changed(raster, target, source.stat().st_mtime_ns)
    assert target.read_bytes() == initial_payload and target.stat().st_mtime_ns == fresh_timestamp
    os.utime(source, ns=(first_timestamp, first_timestamp))
    save_png_if_changed(raster, target, source.stat().st_mtime_ns)
    assert target.stat().st_mtime_ns == first_timestamp, "Stale PPM cannot become fresh PNG evidence"
    raster.putpixel((1, 1), (255, 255, 255))
    original_write = Path.write_bytes
    calls = []

    def transient_write(path: Path, payload: bytes) -> int:
        """Reproduce two Windows preview locks before a successful third write."""
        calls.append(path)
        if len(calls) < 3:
            raise OSError(errno.EINVAL if len(calls) == 1 else errno.EACCES, "Synthetic preview lock")
        return original_write(path, payload)

    with patch.object(Path, "write_bytes", transient_write), patch("verify_render_artifacts.time.sleep") as sleep:
        save_png_if_changed(raster, target, fresh_timestamp)
        assert len(calls) == 3 and sleep.call_count == 2
    assert target.read_bytes() != initial_payload and target.stat().st_mtime_ns == fresh_timestamp
    raster.putpixel((2, 2), (255, 255, 255))
    with patch.object(Path, "write_bytes", side_effect=OSError(errno.EIO, "Synthetic permanent error")) as write:
        try:
            save_png_if_changed(raster, target, fresh_timestamp)
        except OSError as error:
            assert error.errno == errno.EIO and write.call_count == 1
        else:
            raise AssertionError("Permanent errors must be visible")
    with patch.object(Path, "write_bytes", side_effect=OSError(errno.EINVAL, "Persistent preview lock")) as write, \
         patch("verify_render_artifacts.time.sleep") as sleep:
        try:
            save_png_if_changed(raster, target, fresh_timestamp)
        except OSError as error:
            assert error.errno == errno.EINVAL and write.call_count == 3 and sleep.call_count == 2
        else:
            raise AssertionError("Persistent locks must fail after three attempts")
    print(f"PNG_PERSISTENCE_OK identical_bytes=1 source_mtime=1 stale_source_rejected_by_freshness=1 transient_retry=3 permanent_error_visible=1 fixture={fixture}")


if __name__ == "__main__":
    verify_persistence()
