"""Verify checkout newline compatibility using in-memory font reads and blocked writes.

The real source and committed subset are read only as fixtures. Tests neither rewrite
font files nor invoke font tools; the generator module is loaded without bytecode files.
"""
from __future__ import annotations

from contextlib import redirect_stdout
import io
from pathlib import Path
import types
import unittest
from unittest.mock import Mock, patch


PROJECT_ROOT = Path(__file__).resolve().parents[2]
GENERATOR_PATH = PROJECT_ROOT / "Ui/fonts/generate_numeric_28.py"
FONT_GENERATOR = types.ModuleType("numeric_font_generator_under_test")
FONT_GENERATOR.__file__ = str(GENERATOR_PATH)
exec(compile(GENERATOR_PATH.read_text(encoding="utf-8"), str(GENERATOR_PATH), "exec"),
     FONT_GENERATOR.__dict__)


class NumericFontLineEndingTests(unittest.TestCase):
    """Accept only CRLF-to-LF checkout differences while retaining byte integrity."""

    @classmethod
    def setUpClass(cls) -> None:
        """Read existing fixtures once; all generator reads below are memory mocks."""
        cls.source_lf = FONT_GENERATOR.SOURCE.read_bytes().replace(b"\r\n", b"\n")
        cls.artifact_lf = FONT_GENERATOR.OUTPUT.read_bytes().replace(b"\r\n", b"\n")

    def run_check(self, source_bytes: bytes, artifact_bytes: bytes) -> None:
        """Execute --check with mocked reads and reject any attempted file write."""
        source = Mock(spec=Path)
        source.read_bytes.return_value = source_bytes
        artifact = Mock(spec=Path)
        artifact.read_bytes.return_value = artifact_bytes
        with patch.object(FONT_GENERATOR, "SOURCE", source), \
                patch.object(FONT_GENERATOR, "OUTPUT", artifact), \
                patch("sys.argv", [str(GENERATOR_PATH), "--check"]), \
                patch.object(Path, "write_bytes", side_effect=AssertionError("Font writes are forbidden")), \
                redirect_stdout(io.StringIO()):
            try:
                FONT_GENERATOR.main()
            finally:
                source.write_bytes.assert_not_called()
                artifact.write_bytes.assert_not_called()

    def test_original_source_hash_remains_frozen(self) -> None:
        """Keep the reviewed upstream LF fingerprint rather than accepting new content."""
        self.assertEqual(FONT_GENERATOR.SOURCE_SHA256,
                         "c963a5d3f55ed415b2d991f8ea4766797f98692f9253db3d079caf34bc1a661e")

    def test_lf_and_crlf_generate_identical_subset(self) -> None:
        """Both source checkout forms must reproduce the committed LF subset in memory."""
        for newline in (b"\n", b"\r\n"):
            with self.subTest(newline=newline):
                source = Mock(spec=Path)
                source.read_bytes.return_value = self.source_lf.replace(b"\n", newline)
                with patch.object(FONT_GENERATOR, "SOURCE", source), \
                        patch.object(Path, "write_bytes", side_effect=AssertionError("Font writes are forbidden")):
                    output_text, glyph_count, bitmap_size = FONT_GENERATOR.generate()
                self.assertEqual(output_text.encode("utf-8"), self.artifact_lf)
                self.assertGreater(glyph_count, 0)
                self.assertGreater(bitmap_size, 0)
                source.write_bytes.assert_not_called()

    def test_check_accepts_all_lf_and_crlf_combinations(self) -> None:
        """Source and artifact may independently use either Git checkout newline form."""
        for source_newline in (b"\n", b"\r\n"):
            for artifact_newline in (b"\n", b"\r\n"):
                with self.subTest(source=source_newline, artifact=artifact_newline):
                    self.run_check(self.source_lf.replace(b"\n", source_newline),
                                   self.artifact_lf.replace(b"\n", artifact_newline))

    def test_changed_source_content_is_rejected(self) -> None:
        """A glyph literal edit must still fail the frozen upstream SHA-256 gate."""
        changed_source = self.source_lf.replace(b"0x", b"1x", 1)
        self.assertNotEqual(changed_source, self.source_lf)
        for newline in (b"\n", b"\r\n"):
            with self.subTest(newline=newline), self.assertRaisesRegex(ValueError, "Upstream font hash changed"):
                self.run_check(changed_source.replace(b"\n", newline), self.artifact_lf)

    def test_changed_artifact_content_is_rejected(self) -> None:
        """A subset glyph edit must fail even when the upstream source hash is valid."""
        changed_artifact = self.artifact_lf.replace(b"0x", b"1x", 1)
        self.assertNotEqual(changed_artifact, self.artifact_lf)
        for newline in (b"\n", b"\r\n"):
            with self.subTest(newline=newline), self.assertRaisesRegex(ValueError, "Generated font differs"):
                self.run_check(self.source_lf, changed_artifact.replace(b"\n", newline))

    def test_other_byte_changes_are_not_normalized(self) -> None:
        """Bare CR, BOM and extra spaces remain errors instead of being stripped."""
        for changed_source in (self.source_lf.replace(b"\n", b"\r", 1),
                               b"\xef\xbb\xbf" + self.source_lf, self.source_lf + b" "):
            with self.subTest(target="source"), self.assertRaisesRegex(ValueError, "Upstream font hash changed"):
                self.run_check(changed_source, self.artifact_lf)
        for changed_artifact in (self.artifact_lf.replace(b"\n", b"\r", 1),
                                 b"\xef\xbb\xbf" + self.artifact_lf, self.artifact_lf + b" "):
            with self.subTest(target="artifact"), self.assertRaisesRegex(ValueError, "Generated font differs"):
                self.run_check(self.source_lf, changed_artifact)


if __name__ == "__main__":
    unittest.main()
