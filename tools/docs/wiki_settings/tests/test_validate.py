import importlib.util
import io
import json
from pathlib import Path
from contextlib import redirect_stderr
import sys
import unittest


SPEC_ROOT = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = SPEC_ROOT / "validate.py"
SCHEMA_PATH = SPEC_ROOT / "content.schema.json"
VALID_EXAMPLE = SPEC_ROOT / "examples" / "valid" / "AutoNaviSpeedCtrlEnd.ko.json"
INVALID_EXAMPLE = SPEC_ROOT / "examples" / "invalid" / "unknown-section.json"

SPEC = importlib.util.spec_from_file_location("wiki_settings_validate", VALIDATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
VALIDATOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = VALIDATOR
SPEC.loader.exec_module(VALIDATOR)


def load_json(path):
  return json.loads(path.read_text(encoding="utf-8"))


def messages(issues):
  return [issue.message for issue in issues]


class WikiSettingsValidatorTest(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.schema = VALIDATOR.load_schema(SCHEMA_PATH)

  def test_valid_published_example_passes(self):
    document = load_json(VALID_EXAMPLE)
    self.assertEqual(VALIDATOR.validate_published_document(document, self.schema), [])

  def test_invalid_section_example_is_rejected(self):
    document = load_json(INVALID_EXAMPLE)
    errors = VALIDATOR.validate_published_document(document, self.schema)
    self.assertTrue(any("must be one of" in message for message in messages(errors)))
    self.assertTrue(any("exactly one overview" in message for message in messages(errors)))

  def test_content_hash_is_verified(self):
    document = load_json(VALID_EXAMPLE)
    document["identity"]["summary"] = "변조된 설명"
    errors = VALIDATOR.validate_published_document(document, self.schema)
    self.assertTrue(any("canonical content hash" in message for message in messages(errors)))

  def test_duplicate_and_out_of_order_sections_are_rejected(self):
    document = load_json(VALID_EXAMPLE)
    document["sections"] = [
      document["sections"][1],
      document["sections"][0],
      document["sections"][1],
    ]
    document["source"]["contentHash"] = VALIDATOR.canonical_content_hash(document)
    errors = VALIDATOR.validate_published_document(document, self.schema)
    self.assertIn("section kinds must be unique", messages(errors))
    self.assertIn("sections must follow overview, behavior, usage, reference order", messages(errors))

  def test_valid_wiki_markdown_passes(self):
    markdown = """# 속도·감속

<!-- CARROT:SETTING:BEGIN AutoNaviSpeedCtrlEnd -->
<a id="AutoNaviSpeedCtrlEnd"></a>
## 과속카메라 감속 완료 시간

<!-- CARROT:AUTO:BEGIN AutoNaviSpeedCtrlEnd -->
<!-- CARROT:SECTION:BEGIN OVERVIEW -->
- **기본값:** `6초`
<!-- CARROT:SECTION:END OVERVIEW -->
<!-- CARROT:AUTO:END AutoNaviSpeedCtrlEnd -->

<!-- CARROT:MANUAL:BEGIN AutoNaviSpeedCtrlEnd -->
<!-- CARROT:SECTION:BEGIN REFERENCE -->
### 참고
<!-- CARROT:NOTICE level="NOTE" -->
> [!NOTE]
> 감속 완료 지점을 계산하는 시간입니다.
<!-- CARROT:NOTICE:END -->
<!-- CARROT:SECTION:END REFERENCE -->
<!-- CARROT:MANUAL:END AutoNaviSpeedCtrlEnd -->
<!-- CARROT:SETTING:END AutoNaviSpeedCtrlEnd -->
"""
    self.assertEqual(VALIDATOR.validate_wiki_markdown(markdown), [])

  def test_markdown_rejects_unsafe_html_links_and_missing_alt(self):
    markdown = """# 설정

<!-- CARROT:SETTING:BEGIN AlwaysLateral -->
<a id="AlwaysLateral"></a>
## 상시 조향
<!-- CARROT:AUTO:BEGIN AlwaysLateral -->
<!-- CARROT:SECTION:BEGIN OVERVIEW -->
![ ](javascript:alert)
[위험](javascript:alert)
<script>alert(1)</script>
<!-- CARROT:SECTION:END OVERVIEW -->
<!-- CARROT:AUTO:END AlwaysLateral -->
<!-- CARROT:SETTING:END AlwaysLateral -->
"""
    errors = VALIDATOR.validate_wiki_markdown(markdown)
    combined = "\n".join(messages(errors))
    self.assertIn("image alt text is required", combined)
    self.assertIn("image URL must be an anchor or HTTP(S)", combined)
    self.assertIn("link URL must be an anchor or HTTP(S)", combined)
    self.assertIn("dangerous raw HTML tag", combined)

  def test_markdown_rejects_unbalanced_markers_and_wide_tables(self):
    markdown = """# 설정

<!-- CARROT:SETTING:BEGIN AlwaysLateral -->
## 상시 조향
<!-- CARROT:AUTO:BEGIN AlwaysLateral -->
<!-- CARROT:SECTION:BEGIN OVERVIEW -->
| A | B | C | D |
|---|---|---|---|
"""
    errors = VALIDATOR.validate_wiki_markdown(markdown)
    combined = "\n".join(messages(errors))
    self.assertIn("at most three columns", combined)
    self.assertIn("unclosed setting marker", combined)

  def test_cli_returns_nonzero_for_invalid_fixture(self):
    error_output = io.StringIO()
    with redirect_stderr(error_output):
      result = VALIDATOR.main([str(INVALID_EXAMPLE)])
    self.assertEqual(result, 1)
    self.assertIn("validation failed", error_output.getvalue())


if __name__ == "__main__":
  unittest.main()
