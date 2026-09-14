"""Export a reviewable LCD delta against the pre-existing dirty-worktree backup.

Reads Git inventory only. It never stages, resets, switches, removes or restores
files. The original ZIP remains unchanged; new source files are listed explicitly.
"""
from pathlib import Path
import argparse
import difflib
import hashlib
import json
import shutil
import subprocess
import zipfile


def sha256(data: bytes) -> str:
    """Return the content digest used in both the baseline and exported manifest."""
    return hashlib.sha256(data).hexdigest()


def export_snapshot(root: Path, baseline: Path, destination: Path, git: str) -> None:
    """Capture current source and before/after hashes without touching the worktree."""
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(baseline) as archive:
        original = json.loads(archive.read("_baseline/manifest.json"))
        before = {entry["path"]: entry for entry in original["files"]}
        for name, entry in before.items():
            assert sha256(archive.read(name)) == entry["sha256"], f"Baseline mismatch: {name}"
        inventory = subprocess.check_output([git, "ls-files", "-z", "--cached", "--others", "--exclude-standard"], cwd=root)
        names = sorted(set(inventory.decode("utf-8").rstrip("\0").split("\0")))
        head = subprocess.check_output([git, "rev-parse", "HEAD"], cwd=root, text=True).strip()
        branch = subprocess.check_output([git, "branch", "--show-current"], cwd=root, text=True).strip()
        assert head == original["head"] and branch == original["branch"], "Unexpected branch or HEAD change"
        assert all((root / name).is_file() for name in before), "A baseline file is missing; do not silently export a deletion"
        changes, files, patch_parts = [], [], []
        package = destination / "lcd_software_snapshot.zip"
        with zipfile.ZipFile(package, "w", compression=zipfile.ZIP_DEFLATED) as snapshot:
            for name in names:
                current = (root / name).read_bytes()
                digest = sha256(current)
                files.append({"path": name, "bytes": len(current), "sha256": digest})
                snapshot.writestr(name, current)
                prior = before.get(name)
                if prior is not None and digest == prior["sha256"]:
                    continue
                change = {"path": name, "status": "modified" if prior else "added",
                          "before_sha256": prior["sha256"] if prior else None,
                          "after_sha256": digest, "bytes": len(current)}
                changes.append(change)
                previous = archive.read(name) if prior else b""
                try:
                    old_text, new_text = previous.decode("utf-8-sig"), current.decode("utf-8-sig")
                    assert "\0" not in old_text + new_text
                    if not name.lower().endswith(".hex"):
                        patch_parts.extend(difflib.unified_diff(old_text.splitlines(True), new_text.splitlines(True),
                            fromfile="a/" + name if prior else "/dev/null", tofile="b/" + name))
                except (UnicodeDecodeError, AssertionError):
                    change["text_diff_omitted"] = True
            manifest = {"root": str(root), "head": head, "branch": branch,
                        "baseline": str(baseline), "baseline_sha256": sha256(baseline.read_bytes()),
                        "software_only": True, "hardware_tested": False,
                        "original_files_preserved": True, "files": files, "changes": changes}
            snapshot.writestr("_snapshot/manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        (destination / "source-delta.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        (destination / "lcd-only-review.diff").write_text("".join(patch_parts), encoding="utf-8")
        with zipfile.ZipFile(package) as snapshot:
            assert snapshot.testzip() is None
            for entry in files:
                assert sha256(snapshot.read(entry["path"])) == entry["sha256"]
        (destination / "snapshot.sha256").write_text(sha256(package.read_bytes()) + "  " + package.name + "\n", encoding="ascii")
        print(f"LCD_SNAPSHOT_VERIFIED files={len(files)} changed={len(changes)} original_files={len(before)}")
        print(package)


def main() -> None:
    """Accept explicit baseline and output paths for the current original branch."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--git", default=shutil.which("git"), required=shutil.which("git") is None)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    export_snapshot(root, args.baseline.resolve(), args.output.resolve(), args.git)


if __name__ == "__main__":
    main()
