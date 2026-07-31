import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from urllib.parse import quote


SPEC_ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = SPEC_ROOT / "generate.py"
VALIDATOR_PATH = SPEC_ROOT / "validate.py"


def load_module(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = module
  spec.loader.exec_module(module)
  return module


GENERATOR = load_module("wiki_settings_generate", GENERATOR_PATH)
VALIDATOR = load_module("wiki_settings_validate_for_generator", VALIDATOR_PATH)

COMMIT = "a" * 40
STAMP = "2026-07-23T00:00:00Z"


def param(name, *, default=1, title=None):
  title = title or name
  return {
    "group": "테스트",
    "name": name,
    "title": title,
    "descr": f"{title} 설명",
    "egroup": "Test",
    "etitle": f"{title} EN",
    "edescr": f"{title} description",
    "cgroup": "测试",
    "ctitle": f"{title} ZH",
    "cdescr": f"{title} 说明",
    "min": 0,
    "max": 10,
    "default": default,
    "unit": 1,
  }


def leaf(leaf_id, params):
  return {
    "id": leaf_id,
    "ko": f"{leaf_id} 한글",
    "en": f"{leaf_id} English",
    "zh": f"{leaf_id} 中文",
    "params": list(params),
  }


def catalog(params, leaves):
  return {
    "apilot": 1,
    "menu": [{
      "id": "ROOT",
      "ko": "루트",
      "en": "Root",
      "zh": "根",
      "groups": leaves,
    }],
    "params": params,
  }


def write_catalog(root, payload):
  path = root / "carrot_settings.json"
  path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8", newline="\n")
  return path


class WikiSettingsGeneratorTest(unittest.TestCase):
  def test_current_catalog_generates_all_settings_and_valid_markdown(self):
    result = GENERATOR.generate(
      GENERATOR.DEFAULT_CATALOG,
      catalog_commit=COMMIT,
      generated_at=STAMP,
    )
    self.assertEqual(len(result.generated_settings), 171)
    self.assertEqual(result.index["review"], {"current": 0, "needs_review": 171})
    self.assertEqual(result.index["locales"], ["ko", "en", "zh"])
    self.assertEqual(len(result.pages), (171 * 3) + 2)
    setting_pages = {
      name: text
      for name, text in result.pages.items()
      if GENERATOR.GENERATED_PAGE_RE.fullmatch(name)
    }
    self.assertEqual(len(setting_pages), 171 * 3)
    self.assertTrue(all(text.count("<!-- CARROT:SETTING:BEGIN ") == 1 for text in setting_pages.values()))
    for name, text in result.pages.items():
      if name.endswith(".md"):
        self.assertEqual(VALIDATOR.validate_wiki_markdown(text), [], name)
      if GENERATOR.GENERATED_PAGE_RE.fullmatch(name):
        self.assertIn(
          f'<!-- CARROT:AUTHORING guide="{GENERATOR.AUTHORING_GUIDE_URL}" '
          'editable="CARROT:MANUAL only" -->',
          text,
        )
    catalog_settings, _leaves = GENERATOR.load_catalog(
      GENERATOR.DEFAULT_CATALOG,
      ("ko", "en", "zh"),
    )
    settings_by_param = {setting.param: setting for setting in catalog_settings}
    for param_name, item in result.index["settings"].items():
      for locale in ("ko", "en", "zh"):
        self.assertEqual(
          item["locales"][locale]["page"],
          settings_by_param[param_name].page_name(locale),
        )
    catalog_page = result.pages[GENERATOR.CATALOG_PAGE_NAME]
    self.assertIn(
      "https://github.com/ajouatom/openpilot/wiki/"
      + quote("KO-상시-조향", safe="-._~"),
      catalog_page,
    )
    self.assertIn("### 주행 제어\n\n#### 시작·오토\n\n##### 시작 동작", catalog_page)

  def test_manual_region_is_byte_preserved_and_semantic_change_requires_review(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      first_catalog = write_catalog(root, catalog(
        [param("ParamA")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      first = GENERATOR.generate(
        first_catalog,
        locales=("ko", "en"),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      wiki = root / "wiki"
      GENERATOR.write_result(first, wiki)

      semantic = first.generated_settings["ParamA"].semantic_hash
      manual = (
        "<!-- CARROT:MANUAL:BEGIN ParamA -->\n"
        f'<!-- CARROT:REVIEW reviewed-for="{semantic}" reviewed-at="2026-07-23T00:00:00Z" -->\n'
        "<!-- CARROT:SECTION:BEGIN BEHAVIOR -->\n"
        "### 동작\n\n"
        "사람이 작성한 설명  \n"
        "둘째 줄\n"
        "<!-- CARROT:SECTION:END BEHAVIOR -->\n"
        "<!-- CARROT:MANUAL:END ParamA -->"
      )
      page = wiki / first.index["settings"]["ParamA"]["locales"]["ko"]["page"]
      original = page.read_bytes().decode("utf-8")
      old_manual = GENERATOR.MANUAL_BLOCK_RE.search(original)
      self.assertIsNotNone(old_manual)
      page.write_bytes(original.replace(old_manual.group(0), manual).encode("utf-8"))

      changed_payload = catalog(
        [param("ParamA", default=2)],
        [leaf("GROUP_A", ["ParamA"])],
      )
      changed_catalog = write_catalog(root, changed_payload)
      changed = GENERATOR.generate(
        changed_catalog,
        wiki_dir=wiki,
        locales=("ko", "en"),
        catalog_commit="b" * 40,
        generated_at="2026-07-23T00:01:00Z",
      )
      changed_page = changed.pages[page.name]
      preserved = GENERATOR.MANUAL_BLOCK_RE.search(changed_page)
      self.assertIsNotNone(preserved)
      self.assertEqual(preserved.group(0).encode("utf-8"), manual.encode("utf-8"))
      self.assertEqual(changed.generated_settings["ParamA"].review_status, "needs_review")
      self.assertIn("ParamA", changed.setting_changes["changed"])

  def test_same_input_is_idempotent(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      catalog_path = write_catalog(root, catalog(
        [param("ParamA"), param("ParamB", default=2)],
        [leaf("GROUP_A", ["ParamA", "ParamB"])],
      ))
      first = GENERATOR.generate(
        catalog_path,
        locales=("ko", "en"),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      wiki = root / "wiki"
      GENERATOR.write_result(first, wiki)
      second = GENERATOR.generate(
        catalog_path,
        wiki_dir=wiki,
        locales=("ko", "en"),
        catalog_commit=COMMIT,
      )
      self.assertEqual(first.pages, second.pages)
      self.assertEqual(second.page_changes, {"added": [], "changed": [], "removed": []})
      self.assertEqual(
        second.setting_changes,
        {"added": [], "changed": [], "moved": [], "removed": [], "reviewChanged": []},
      )

  def test_group_page_migrates_to_one_setting_page_without_losing_manual_content(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      catalog_path = write_catalog(root, catalog(
        [param("ParamA")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      initial = GENERATOR.generate(
        catalog_path,
        locales=("ko",),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      setting_page = initial.pages["KO-ParamA.md"]
      block = GENERATOR.SETTING_BLOCK_RE.search(setting_page)
      self.assertIsNotNone(block)
      custom_manual = GENERATOR.MANUAL_BLOCK_RE.search(block.group(0))
      self.assertIsNotNone(custom_manual)
      custom_manual_text = custom_manual.group(0).replace(
        '<!-- CARROT:REVIEW reviewed-for="" -->',
        '<!-- CARROT:REVIEW reviewed-for="" -->\n기존 그룹 페이지의 수동 설명',
      )
      legacy_block = block.group(0).replace(custom_manual.group(0), custom_manual_text)

      wiki = root / "wiki"
      wiki.mkdir()
      legacy_name = "KO-Settings-Group-A.md"
      (wiki / legacy_name).write_text(
        '<!-- CARROT:GENERATED schema="1" locale="ko" group="GROUP_A" -->\n'
        "# 기존 그룹 페이지\n\n"
        f"{legacy_block}",
        encoding="utf-8",
        newline="\n",
      )

      migrated = GENERATOR.generate(
        catalog_path,
        wiki_dir=wiki,
        locales=("ko",),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      self.assertIn(custom_manual_text, migrated.pages["KO-ParamA.md"])
      self.assertIn(legacy_name, migrated.page_changes["removed"])

  def test_windows_crlf_checkout_keeps_managed_pages_recognizable(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      catalog_path = write_catalog(root, catalog(
        [param("ParamA")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      wiki = root / "wiki"
      initial = GENERATOR.generate(
        catalog_path,
        locales=("ko", "en", "zh"),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      GENERATOR.write_result(initial, wiki)
      page = wiki / "KO-ParamA.md"
      page.write_bytes(page.read_bytes().replace(b"\n", b"\r\n"))

      regenerated = GENERATOR.generate(
        catalog_path,
        wiki_dir=wiki,
        locales=("ko", "en", "zh"),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      self.assertIn("KO-ParamA.md", regenerated.pages)
      self.assertEqual(regenerated.setting_changes["added"], [])

  def test_add_change_move_remove_diff_and_manual_move(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      initial_catalog = write_catalog(root, catalog(
        [param("ParamA"), param("ParamB")],
        [leaf("GROUP_A", ["ParamA"]), leaf("GROUP_B", ["ParamB"])],
      ))
      initial = GENERATOR.generate(
        initial_catalog,
        locales=("ko", "en"),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      wiki = root / "wiki"
      GENERATOR.write_result(initial, wiki)

      page_name = initial.index["settings"]["ParamB"]["locales"]["ko"]["page"]
      page = wiki / page_name
      text = page.read_text(encoding="utf-8")
      existing_manual = GENERATOR.MANUAL_BLOCK_RE.search(text)
      self.assertIsNotNone(existing_manual)
      custom_manual = existing_manual.group(0).replace(
        '<!-- CARROT:REVIEW reviewed-for="" -->',
        '<!-- CARROT:REVIEW reviewed-for="" -->\n수동 설명은 이동해도 유지',
      )
      page.write_text(text.replace(existing_manual.group(0), custom_manual), encoding="utf-8", newline="\n")

      next_catalog = write_catalog(root, catalog(
        [param("ParamB", default=2), param("ParamC")],
        [leaf("GROUP_C", ["ParamB", "ParamC"])],
      ))
      result = GENERATOR.generate(
        next_catalog,
        wiki_dir=wiki,
        locales=("ko", "en"),
        catalog_commit="b" * 40,
        generated_at="2026-07-23T00:01:00Z",
      )
      self.assertEqual(result.setting_changes["added"], ["ParamC"])
      self.assertEqual(result.setting_changes["removed"], ["ParamA"])
      self.assertIn("ParamB", result.setting_changes["changed"])
      self.assertNotIn("ParamB", result.setting_changes["moved"])
      new_page = result.index["settings"]["ParamB"]["locales"]["ko"]["page"]
      self.assertIn(custom_manual, result.pages[new_page])
      self.assertIn("KO-ParamA.md", result.page_changes["removed"])
      self.assertNotIn("KO-ParamB.md", result.page_changes["removed"])

  def test_catalog_rejects_duplicate_or_unplaced_parameters(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      duplicate_path = write_catalog(root, catalog(
        [param("ParamA"), param("ParamA")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      with self.assertRaisesRegex(GENERATOR.GenerationError, "duplicate parameter"):
        GENERATOR.load_catalog(duplicate_path, ("ko", "en"))

      unplaced_path = write_catalog(root, catalog(
        [param("ParamA"), param("ParamB")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      with self.assertRaisesRegex(GENERATOR.GenerationError, "missing from menu"):
        GENERATOR.load_catalog(unplaced_path, ("ko", "en"))

  def test_unmanaged_wiki_pages_are_ignored_or_blocked_by_namespace(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      catalog_path = write_catalog(root, catalog(
        [param("ParamA")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      wiki = root / "wiki"
      wiki.mkdir()
      guide = wiki / "Guide-Settings.md"
      guide.write_text("# Existing guide\n", encoding="utf-8", newline="\n")
      localized_guide = wiki / "KO-직접작성.md"
      localized_guide.write_text("# 직접 작성한 문서\n", encoding="utf-8", newline="\n")
      GENERATOR.generate(
        catalog_path,
        wiki_dir=wiki,
        locales=("ko", "en"),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      self.assertEqual(guide.read_text(encoding="utf-8"), "# Existing guide\n")
      self.assertEqual(
        localized_guide.read_text(encoding="utf-8"),
        "# 직접 작성한 문서\n",
      )

      collision = wiki / "KO-ParamA.md"
      collision.write_text("# Human page without markers\n", encoding="utf-8", newline="\n")
      with self.assertRaisesRegex(GENERATOR.GenerationError, "refusing to overwrite"):
        GENERATOR.generate(
          catalog_path,
          wiki_dir=wiki,
          locales=("ko", "en"),
          catalog_commit=COMMIT,
          generated_at=STAMP,
        )

  def test_localized_title_rename_moves_page_and_preserves_manual_content(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      catalog_path = write_catalog(root, catalog(
        [param("ParamA", title="기존 이름")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      initial = GENERATOR.generate(
        catalog_path,
        locales=("ko", "en", "zh"),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      wiki = root / "wiki"
      GENERATOR.write_result(initial, wiki)
      old_page_name = initial.index["settings"]["ParamA"]["locales"]["ko"]["page"]
      old_page = wiki / old_page_name
      old_text = old_page.read_text(encoding="utf-8")
      manual = GENERATOR.MANUAL_BLOCK_RE.search(old_text)
      self.assertIsNotNone(manual)
      custom_manual = manual.group(0).replace(
        '<!-- CARROT:REVIEW reviewed-for="" -->',
        '<!-- CARROT:REVIEW reviewed-for="" -->\n이 설명은 이름이 바뀌어도 유지',
      )
      old_page.write_text(
        old_text.replace(manual.group(0), custom_manual),
        encoding="utf-8",
        newline="\n",
      )

      renamed_catalog = write_catalog(root, catalog(
        [param("ParamA", title="새 이름")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      renamed = GENERATOR.generate(
        renamed_catalog,
        wiki_dir=wiki,
        locales=("ko", "en", "zh"),
        catalog_commit="b" * 40,
        generated_at="2026-07-23T00:01:00Z",
      )
      new_page_name = renamed.index["settings"]["ParamA"]["locales"]["ko"]["page"]
      self.assertEqual(new_page_name, "KO-새-이름.md")
      self.assertIn(custom_manual, renamed.pages[new_page_name])
      self.assertIn(old_page_name, renamed.page_changes["removed"])
      self.assertIn("ParamA", renamed.setting_changes["moved"])

  def test_localized_page_names_reject_sanitized_collisions(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      catalog_path = write_catalog(root, catalog(
        [
          param("ParamA", title="같은/이름"),
          param("ParamB", title="같은:이름"),
        ],
        [leaf("GROUP_A", ["ParamA", "ParamB"])],
      ))
      with self.assertRaisesRegex(GENERATOR.GenerationError, "titles collide"):
        GENERATOR.generate(
          catalog_path,
          locales=("ko", "en", "zh"),
          catalog_commit=COMMIT,
          generated_at=STAMP,
        )

  def test_existing_sidebar_keeps_its_structure_and_adds_the_korean_web_menu_tree(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      catalog_path = write_catalog(root, catalog(
        [param("ParamA"), param("ParamB")],
        [
          leaf("GROUP_A", ["ParamA"]),
          leaf("GROUP_B", ["ParamB"]),
        ],
      ))
      wiki = root / "wiki"
      wiki.mkdir()
      original_sidebar = (
        "## carrotpilot Wiki\n\n"
        "- [[소개|Home]]\n"
        "- 사용 설명서\n"
        "  - [설정 이해하기](https://example.com/settings.md)\n"
        "    - [버튼·프리셋 상세](https://example.com/buttons.md)\n"
        "  - [[Carrot Web|Guide-Carrot-Web]]\n"
      )
      (wiki / GENERATOR.SIDEBAR_NAME).write_text(
        original_sidebar,
        encoding="utf-8",
        newline="\n",
      )

      first = GENERATOR.generate(
        catalog_path,
        wiki_dir=wiki,
        locales=("ko", "en", "zh"),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      sidebar = first.pages[GENERATOR.SIDEBAR_NAME]
      catalog_begin = (
        "    - [[전체 설정|Settings-Catalog]] "
        "<!-- CARROT:SETTINGS-CATALOG:BEGIN -->"
      )
      catalog_end = "<!-- CARROT:SETTINGS-CATALOG:END -->"
      self.assertEqual(sidebar.count(catalog_begin), 1)
      self.assertEqual(sidebar.count(catalog_end), 1)
      self.assertIn("      - 루트", sidebar)
      self.assertIn(r"        - GROUP\_A 한글", sidebar)
      self.assertIn("          - [[ParamA|KO-ParamA]]", sidebar)
      self.assertIn(r"        - GROUP\_B 한글", sidebar)
      self.assertIn("          - [[ParamB|KO-ParamB]]", sidebar)
      self.assertLess(sidebar.index("[[ParamA|KO-ParamA]]"), sidebar.index("[[ParamB|KO-ParamB]]"))
      self.assertIn("    - [버튼·프리셋 상세]", sidebar)
      self.assertNotIn(GENERATOR.SIDEBAR_NAME, first.index["pages"])

      GENERATOR.write_result(first, wiki)
      second = GENERATOR.generate(
        catalog_path,
        wiki_dir=wiki,
        locales=("ko", "en", "zh"),
        catalog_commit=COMMIT,
      )
      second_sidebar = second.pages[GENERATOR.SIDEBAR_NAME]
      self.assertEqual(second_sidebar.count(catalog_begin), 1)
      self.assertEqual(second_sidebar.count(catalog_end), 1)
      self.assertEqual(second_sidebar.count("[[ParamA|KO-ParamA]]"), 1)

  def test_existing_sidebar_migrates_the_legacy_single_catalog_link(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      catalog_path = write_catalog(root, catalog(
        [param("ParamA")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      wiki = root / "wiki"
      wiki.mkdir()
      (wiki / GENERATOR.SIDEBAR_NAME).write_text(
        "## carrotpilot Wiki\n\n"
        "- 사용 설명서\n"
        "  - [설정 이해하기](https://example.com/settings.md)\n"
        "    - [[전체 설정|Settings-Catalog]] <!-- CARROT:SETTINGS-CATALOG -->\n"
        "    - [버튼·프리셋 상세](https://example.com/buttons.md)\n",
        encoding="utf-8",
        newline="\n",
      )

      result = GENERATOR.generate(
        catalog_path,
        wiki_dir=wiki,
        locales=("ko", "en", "zh"),
        catalog_commit=COMMIT,
        generated_at=STAMP,
      )
      sidebar = result.pages[GENERATOR.SIDEBAR_NAME]
      self.assertNotIn(GENERATOR.SIDEBAR_CATALOG_MARKER, sidebar)
      self.assertEqual(sidebar.count(GENERATOR.SIDEBAR_CATALOG_BEGIN_MARKER), 1)
      self.assertEqual(sidebar.count(GENERATOR.SIDEBAR_CATALOG_END_MARKER), 1)
      self.assertIn("[[ParamA|KO-ParamA]]", sidebar)
      self.assertIn("    - [버튼·프리셋 상세]", sidebar)

  def test_unmanaged_catalog_files_are_not_overwritten(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      catalog_path = write_catalog(root, catalog(
        [param("ParamA")],
        [leaf("GROUP_A", ["ParamA"])],
      ))
      wiki = root / "wiki"
      wiki.mkdir()
      (wiki / GENERATOR.CATALOG_PAGE_NAME).write_text(
        "# Human catalog page\n", encoding="utf-8", newline="\n",
      )
      with self.assertRaisesRegex(GENERATOR.GenerationError, "not generator-managed"):
        GENERATOR.generate(
          catalog_path,
          wiki_dir=wiki,
          locales=("ko", "en"),
          catalog_commit=COMMIT,
          generated_at=STAMP,
        )


if __name__ == "__main__":
  unittest.main()
