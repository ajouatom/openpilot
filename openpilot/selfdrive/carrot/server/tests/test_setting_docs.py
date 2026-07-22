from openpilot.selfdrive.carrot.server.services import setting_docs
from openpilot.selfdrive.carrot.server.services.settings import get_settings_cached


def test_extract_markdown_section_stops_at_same_or_higher_heading():
  markdown = """# Page

<a id="target"></a>
## Target

first

### Child

second

## Next

third
"""
  section = setting_docs.extract_markdown_section(markdown, "target")
  assert section is not None
  assert section.startswith("## Target")
  assert "### Child" in section
  assert "## Next" not in section
  assert setting_docs.extract_markdown_section(markdown, "missing") is None


def test_every_current_setting_maps_to_a_document_section_in_korean_and_english():
  _data, _groups, by_name, _groups_list = get_settings_cached()
  missing_references = [name for name in by_name if setting_docs.get_setting_doc_reference(name) is None]
  assert missing_references == []

  missing_sections = []
  for name in by_name:
    for language in ("ko", "en"):
      if setting_docs.load_setting_doc(name, language) is None:
        missing_sections.append(f"{language}:{name}")
  assert missing_sections == []


def test_speed_setting_returns_only_its_mapped_section():
  payload = setting_docs.load_setting_doc("AutoNaviSpeedCtrlEnd", "ko")
  assert payload is not None
  assert payload["source"] == "speed-deceleration"
  assert payload["anchor"] == "speed-camera"
  assert "`AutoNaviSpeedCtrlEnd`" in payload["markdown"]
  assert "## 2. 도로 제한속도" not in payload["markdown"]


def test_chinese_uses_english_until_a_matching_guide_exists():
  payload = setting_docs.load_setting_doc("AutoEngage", "zh")
  assert payload is not None
  assert payload["language_requested"] == "zh"
  assert payload["language_resolved"] == "en"
  assert payload["fallback"] is True
  assert "Startup and auto" in payload["markdown"]


def test_unknown_setting_and_unsafe_language_are_bounded():
  assert setting_docs.load_setting_doc("../../secret", "ko") is None
  assert setting_docs.normalize_doc_language("../../ko") == "en"
