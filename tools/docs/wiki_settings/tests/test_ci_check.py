import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SPEC_ROOT = Path(__file__).resolve().parents[1]
CI_CHECK_PATH = SPEC_ROOT / "ci_check.py"

sys.path.insert(0, str(SPEC_ROOT))
try:
  SPEC = importlib.util.spec_from_file_location("wiki_settings_ci_check", CI_CHECK_PATH)
  assert SPEC is not None and SPEC.loader is not None
  CI_CHECK = importlib.util.module_from_spec(SPEC)
  sys.modules[SPEC.name] = CI_CHECK
  SPEC.loader.exec_module(CI_CHECK)
finally:
  sys.path.pop(0)


class WikiSettingsCiCheckTest(unittest.TestCase):
  def test_read_only_check_writes_reports_without_touching_wiki(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      wiki = root / "wiki"
      wiki.mkdir()
      guide = wiki / "Guide-Stable.md"
      original = "# Existing guide\n\nDo not change.\n"
      guide.write_text(original, encoding="utf-8", newline="\n")
      output = root / "report"

      status, report = CI_CHECK.run_check(
        catalog=CI_CHECK.DEFAULT_CATALOG,
        wiki_dir=wiki,
        output_dir=output,
        locales=("ko", "en", "zh"),
        catalog_commit="a" * 40,
        wiki_commit=None,
      )

      self.assertEqual(status, 0)
      self.assertTrue(report["ok"])
      self.assertEqual(guide.read_text(encoding="utf-8"), original)
      self.assertEqual([path.name for path in wiki.iterdir()], ["Guide-Stable.md"])
      self.assertTrue((output / "summary.md").is_file())
      self.assertTrue((output / "pages.diff").is_file())
      stored = json.loads((output / "summary.json").read_text(encoding="utf-8"))
      self.assertEqual(stored["result"]["settings"], 171)
      self.assertEqual(stored["validationIssues"], [])
      self.assertIn("Settings-Catalog.md", (output / "pages.diff").read_text(encoding="utf-8"))

  def test_report_directory_cannot_be_the_wiki_checkout(self):
    with tempfile.TemporaryDirectory() as temp:
      wiki = Path(temp) / "wiki"
      wiki.mkdir()
      with self.assertRaisesRegex(CI_CHECK.generate.GenerationError, "must differ"):
        CI_CHECK.run_check(
          catalog=CI_CHECK.DEFAULT_CATALOG,
          wiki_dir=wiki,
          output_dir=wiki,
          locales=("ko", "en"),
          catalog_commit="a" * 40,
          wiki_commit=None,
        )


if __name__ == "__main__":
  unittest.main()
