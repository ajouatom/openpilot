#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import difflib
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Any, Iterable
import unicodedata
from urllib.parse import quote


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CATALOG = REPO_ROOT / "openpilot" / "selfdrive" / "carrot_settings.json"
INDEX_NAME = "Settings-Catalog.json"
CATALOG_PAGE_NAME = "Settings-Catalog.md"
SIDEBAR_NAME = "_Sidebar.md"
SCHEMA_VERSION = 1
DEFAULT_LOCALES = ("ko", "en", "zh")
AUTHORING_GUIDE_URL = (
  "https://github.com/ajouatom/openpilot/blob/carrot-wip/"
  "tools/docs/wiki_settings/AUTHORING_GUIDE.md"
)

PARAM_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
COMMIT_RE = re.compile(r"^[a-f0-9]{7,40}$")
GENERATED_PAGE_RE = re.compile(r'^(?:KO|EN|ZH)-[^\x00-\x1f<>:"/\\|?*#%[\]{}]+\.md$')
LEGACY_LOCALE_GENERATED_PAGE_RE = re.compile(
  r"^[A-Z]{2,3}-Settings-[A-Za-z0-9_-]+\.md$"
)
LEGACY_GENERATED_PAGE_RE = re.compile(r"^Settings-(?!Catalog(?:\.|$))[A-Za-z0-9-]+\.md$")
GENERATED_HEADER_RE = re.compile(
  r'^<!-- CARROT:GENERATED schema="\d+" locale="(?P<locale>[a-z]{2,3})" '
  r'group="[A-Za-z][A-Za-z0-9_]*"(?: param="[A-Za-z][A-Za-z0-9_]*")? -->$',
  re.MULTILINE,
)
WIKI_PAGE_UNSAFE_RE = re.compile(r'[\x00-\x1f<>:"/\\|?*#%[\]{}]+')
WIKI_PAGE_WHITESPACE_RE = re.compile(r"\s+")
WIKI_PAGE_DASH_RE = re.compile(r"-+")
SIDEBAR_CATALOG_MARKER = "<!-- CARROT:SETTINGS-CATALOG -->"
SIDEBAR_CATALOG_BEGIN_MARKER = "<!-- CARROT:SETTINGS-CATALOG:BEGIN -->"
SIDEBAR_CATALOG_END_MARKER = "<!-- CARROT:SETTINGS-CATALOG:END -->"
SIDEBAR_GUIDE_LABEL = "설정 이해하기"
SIDEBAR_CATALOG_LABEL = "전체 설정"
SETTING_BLOCK_RE = re.compile(
  r"(?ms)^<!-- CARROT:SETTING:BEGIN (?P<param>[A-Za-z][A-Za-z0-9_]*) -->\n"
  r".*?"
  r"^<!-- CARROT:SETTING:END (?P=param) -->(?:\n|$)"
)
MANUAL_BLOCK_RE = re.compile(
  r"(?ms)^<!-- CARROT:MANUAL:BEGIN (?P<param>[A-Za-z][A-Za-z0-9_]*) -->\n"
  r".*?"
  r"^<!-- CARROT:MANUAL:END (?P=param) -->"
)
AUTO_SOURCE_RE = re.compile(
  r'^<!-- CARROT:AUTO:SOURCE facts="(?P<facts>[a-f0-9]{64})" '
  r'semantic="(?P<semantic>[a-f0-9]{64})" review="(?P<review>current|needs_review)" -->$',
  re.MULTILINE,
)
REVIEW_RE = re.compile(
  r'^<!-- CARROT:REVIEW reviewed-for="(?P<semantic>[a-f0-9]{64})?"'
  r'(?: reviewed-at="(?P<reviewed_at>[^"]+)")? -->$',
  re.MULTILINE,
)

CONTROL_KINDS = frozenset(("toggle", "segmented", "select", "slider"))
DISPLAY_UNITS = {
  "raw": "",
  "speedKph": "km/h",
  "distanceCm": "cm",
  "timeSec": "s",
  "timeMin": "min",
  "percent": "%",
  "degree": "deg",
}
LOCALE_FIELDS = {
  "ko": ("title", "descr", "ko"),
  "en": ("etitle", "edescr", "en"),
  "zh": ("ctitle", "cdescr", "zh"),
}
LABELS = {
  "ko": {
    "catalog_title": "전체 설정 사전",
    "catalog_intro": "현재 설정 카탈로그에서 자동 생성된 설정별 페이지입니다.",
    "page_suffix": "설정",
    "menu": "메뉴",
    "shortcuts": "바로가기",
    "type": "종류",
    "default": "기본값",
    "range": "범위",
    "step": "변경 단위",
    "unit": "표시 단위",
    "options": "선택지",
    "types": {
      "toggle": "토글",
      "segmented": "구간 선택",
      "select": "선택",
      "slider": "숫자",
      "text": "텍스트",
    },
  },
  "en": {
    "catalog_title": "Settings catalog",
    "catalog_intro": "Per-setting pages generated from the current settings catalog.",
    "page_suffix": "settings",
    "menu": "Menu",
    "shortcuts": "Quick links",
    "type": "Type",
    "default": "Default",
    "range": "Range",
    "step": "Step",
    "unit": "Display unit",
    "options": "Options",
    "types": {
      "toggle": "Toggle",
      "segmented": "Segmented choice",
      "select": "Select",
      "slider": "Number",
      "text": "Text",
    },
  },
  "zh": {
    "catalog_title": "设置目录",
    "catalog_intro": "根据当前设置目录自动生成的各项设置页面。",
    "page_suffix": "设置",
    "menu": "菜单",
    "shortcuts": "快速链接",
    "type": "类型",
    "default": "默认值",
    "range": "范围",
    "step": "步长",
    "unit": "显示单位",
    "options": "选项",
    "types": {
      "toggle": "开关",
      "segmented": "分段选择",
      "select": "选择",
      "slider": "数值",
      "text": "文本",
    },
  },
}


class GenerationError(ValueError):
  pass


@dataclass(frozen=True)
class MenuLeaf:
  ids: tuple[str, ...]
  labels: dict[str, tuple[str, ...]]
  params: tuple[str, ...]

  @property
  def id(self) -> str:
    return self.ids[-1]

  def path_label(self, locale: str) -> str:
    return " > ".join(self.labels[locale])


@dataclass(frozen=True)
class CatalogSetting:
  order: int
  raw: dict[str, Any]
  leaf: MenuLeaf

  @property
  def param(self) -> str:
    return self.raw["name"]

  def page_name(self, locale: str) -> str:
    title_field, _description_field, _menu_field = LOCALE_FIELDS[locale]
    return f"{locale.upper()}-{_wiki_page_slug(self.raw[title_field])}.md"


@dataclass(frozen=True)
class ExistingSetting:
  param: str
  page: str
  manual_block: str
  facts_hash: str | None
  semantic_hash: str | None
  review_status: str | None
  reviewed_for: str | None


@dataclass(frozen=True)
class GeneratedSetting:
  param: str
  page: str
  anchor: str
  facts_hash: str
  semantic_hash: str
  review_status: str


@dataclass
class GenerationResult:
  pages: dict[str, str]
  index: dict[str, Any]
  generated_settings: dict[str, GeneratedSetting]
  previous_generated_pages: set[str]
  page_changes: dict[str, list[str]]
  setting_changes: dict[str, list[str]]
  warnings: list[str]
  wiki_dir: Path | None

  def report(self) -> dict[str, Any]:
    return {
      "schemaVersion": SCHEMA_VERSION,
      "mode": "dry-run",
      "catalogCommit": self.index["catalogCommit"],
      "locales": self.index["locales"],
      "settings": len(self.generated_settings),
      "pages": len(self.pages),
      "pageChanges": self.page_changes,
      "settingChanges": self.setting_changes,
      "review": self.index["review"],
      "warnings": self.warnings,
    }

  def unified_diff(self) -> str:
    chunks: list[str] = []
    old_pages = _read_candidate_pages(self.wiki_dir)
    if self.wiki_dir is not None and (self.wiki_dir / INDEX_NAME).is_file():
      old_pages[INDEX_NAME] = (self.wiki_dir / INDEX_NAME).read_text(encoding="utf-8")
    all_names = sorted(set(self.pages) | self.previous_generated_pages)
    for name in all_names:
      old = old_pages.get(name, "")
      new = self.pages.get(name, "")
      if old == new:
        continue
      chunks.extend(difflib.unified_diff(
        old.splitlines(keepends=True),
        new.splitlines(keepends=True),
        fromfile=f"a/{name}",
        tofile=f"b/{name}",
      ))
    return "".join(chunks)


def _canonical_hash(value: Any) -> str:
  encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
  return hashlib.sha256(encoded).hexdigest()


def _text_hash(value: str) -> str:
  return hashlib.sha256(value.encode("utf-8")).hexdigest()


def _normalize_text(value: Any) -> str:
  return " ".join(str(value or "").replace("\r\n", "\n").replace("\r", "\n").split())


def _wiki_page_slug(value: Any) -> str:
  title = unicodedata.normalize("NFKC", _normalize_text(value))
  slug = WIKI_PAGE_UNSAFE_RE.sub("-", title)
  slug = WIKI_PAGE_WHITESPACE_RE.sub("-", slug)
  slug = WIKI_PAGE_DASH_RE.sub("-", slug).strip("-. ")
  if not slug or slug in {".", ".."}:
    raise GenerationError(f"setting title cannot form a safe Wiki page name: {title!r}")
  return slug


def _markdown_text(value: Any) -> str:
  text = _normalize_text(value)
  for char in ("\\", "`", "*", "_", "[", "]", "<", ">"):
    text = text.replace(char, f"\\{char}")
  return text


def _json_text(value: Any) -> str:
  return json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n"


def _git_head(repo_root: Path = REPO_ROOT) -> str:
  completed = subprocess.run(
    ["git", "rev-parse", "HEAD"],
    cwd=repo_root,
    check=False,
    capture_output=True,
    text=True,
  )
  commit = completed.stdout.strip().lower()
  if completed.returncode or not COMMIT_RE.fullmatch(commit):
    raise GenerationError("catalog commit was not supplied and git HEAD could not be resolved")
  return commit


def _validate_locale(locale: str) -> None:
  if locale not in LOCALE_FIELDS:
    raise GenerationError(f"unsupported locale {locale!r}; choose from {', '.join(LOCALE_FIELDS)}")


def _walk_menu(
  node: dict[str, Any],
  *,
  parent_ids: tuple[str, ...],
  parent_labels: dict[str, tuple[str, ...]],
  leaves: list[MenuLeaf],
) -> None:
  node_id = node.get("id")
  if not isinstance(node_id, str) or not node_id:
    raise GenerationError("every menu node must have a non-empty id")
  ids = parent_ids + (node_id,)
  labels: dict[str, tuple[str, ...]] = {}
  for locale, (_, _, node_field) in LOCALE_FIELDS.items():
    label = node.get(node_field)
    if not isinstance(label, str) or not label.strip():
      raise GenerationError(f"menu node {node_id} is missing its {locale} label")
    labels[locale] = parent_labels.get(locale, ()) + (_normalize_text(label),)

  children = node.get("groups")
  params = node.get("params")
  if children is not None and params is not None:
    raise GenerationError(f"menu node {node_id} cannot contain both groups and params")
  if children is not None:
    if not isinstance(children, list) or not children:
      raise GenerationError(f"menu node {node_id}.groups must be a non-empty array")
    for child in children:
      if not isinstance(child, dict):
        raise GenerationError(f"menu node {node_id} contains a non-object child")
      _walk_menu(child, parent_ids=ids, parent_labels=labels, leaves=leaves)
    return
  if not isinstance(params, list) or not params:
    raise GenerationError(f"menu leaf {node_id} must contain a non-empty params array")
  if any(not isinstance(param, str) or not PARAM_RE.fullmatch(param) for param in params):
    raise GenerationError(f"menu leaf {node_id} contains an invalid parameter name")
  if len(params) != len(set(params)):
    raise GenerationError(f"menu leaf {node_id} contains duplicate parameters")
  leaves.append(MenuLeaf(ids=ids, labels=labels, params=tuple(params)))


def _control_kind(raw: dict[str, Any]) -> str:
  override = raw.get("control")
  if override is not None:
    if override not in CONTROL_KINDS:
      raise GenerationError(f"{raw.get('name')}: unsupported control {override!r}")
    return override
  minimum = raw.get("min")
  maximum = raw.get("max")
  if not isinstance(minimum, (int, float)) or not isinstance(maximum, (int, float)):
    return "text"
  if minimum == 0 and maximum == 1:
    return "toggle"
  if float(minimum).is_integer() and float(maximum).is_integer():
    count = int(maximum - minimum + 1)
    if 2 <= count <= 4:
      return "segmented"
    if 4 < count <= 8:
      return "select"
  return "slider"


def _validate_param(raw: dict[str, Any], requested_locales: tuple[str, ...]) -> None:
  param = raw.get("name")
  if not isinstance(param, str) or not PARAM_RE.fullmatch(param):
    raise GenerationError(f"invalid parameter name {param!r}")
  required = {"group", "name", "title", "descr", "egroup", "etitle", "edescr", "default"}
  missing = sorted(required - raw.keys())
  if missing:
    raise GenerationError(f"{param}: missing required fields: {', '.join(missing)}")
  for locale in requested_locales:
    title_field, descr_field, _ = LOCALE_FIELDS[locale]
    if not isinstance(raw.get(title_field), str) or not raw[title_field].strip():
      raise GenerationError(f"{param}: missing {locale} title")
    if not isinstance(raw.get(descr_field), str):
      raise GenerationError(f"{param}: {locale} description must be a string")

  minimum = raw.get("min")
  maximum = raw.get("max")
  unit = raw.get("unit")
  numeric_fields = (minimum, maximum, unit)
  if any(value is not None for value in numeric_fields):
    if not all(isinstance(value, (int, float)) for value in numeric_fields):
      raise GenerationError(f"{param}: min, max, and unit must be supplied together as numbers")
    if minimum > maximum:
      raise GenerationError(f"{param}: min must not exceed max")
    if unit <= 0:
      raise GenerationError(f"{param}: unit must be positive")
    default = raw.get("default")
    if not isinstance(default, (int, float)):
      raise GenerationError(f"{param}: numeric settings must have a numeric default")

  display_unit = raw.get("display_unit")
  if display_unit is not None and display_unit not in DISPLAY_UNITS:
    raise GenerationError(f"{param}: unsupported display_unit {display_unit!r}")
  _control_kind(raw)

  options = raw.get("options")
  if options is not None:
    if not isinstance(options, dict):
      raise GenerationError(f"{param}: options must be an object")
    if not isinstance(minimum, (int, float)) or not isinstance(maximum, (int, float)):
      raise GenerationError(f"{param}: options require numeric min/max")
    expected = int(maximum - minimum + 1)
    for locale in requested_locales:
      values = options.get(locale)
      if not isinstance(values, list) or len(values) != expected or any(not isinstance(item, str) for item in values):
        raise GenerationError(f"{param}: {locale} options must contain {expected} strings")


def load_catalog(path: Path, locales: Iterable[str] = DEFAULT_LOCALES) -> tuple[list[CatalogSetting], list[MenuLeaf]]:
  requested_locales = tuple(dict.fromkeys(locales))
  for locale in requested_locales:
    _validate_locale(locale)
  try:
    payload = json.loads(path.read_text(encoding="utf-8"))
  except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
    raise GenerationError(f"cannot read catalog {path}: {error}") from error
  if not isinstance(payload, dict) or not isinstance(payload.get("menu"), list) or not isinstance(payload.get("params"), list):
    raise GenerationError("catalog must contain menu and params arrays")

  leaves: list[MenuLeaf] = []
  for node in payload["menu"]:
    if not isinstance(node, dict):
      raise GenerationError("catalog menu entries must be objects")
    _walk_menu(node, parent_ids=(), parent_labels={}, leaves=leaves)
  leaf_ids = [leaf.id for leaf in leaves]
  if len(leaf_ids) != len(set(leaf_ids)):
    raise GenerationError("leaf menu ids must be globally unique for stable Wiki page names")

  by_name: dict[str, tuple[int, dict[str, Any]]] = {}
  for index, raw in enumerate(payload["params"]):
    if not isinstance(raw, dict):
      raise GenerationError(f"params[{index}] must be an object")
    _validate_param(raw, requested_locales)
    name = raw["name"]
    if name in by_name:
      raise GenerationError(f"duplicate parameter definition {name}")
    by_name[name] = (index, raw)

  placement: dict[str, MenuLeaf] = {}
  for leaf in leaves:
    for param in leaf.params:
      if param in placement:
        raise GenerationError(f"parameter {param} appears in more than one menu leaf")
      placement[param] = leaf
  missing_definitions = sorted(set(placement) - set(by_name))
  missing_placement = sorted(set(by_name) - set(placement))
  if missing_definitions:
    raise GenerationError(f"menu contains undefined parameters: {', '.join(missing_definitions)}")
  if missing_placement:
    raise GenerationError(f"parameters missing from menu: {', '.join(missing_placement)}")

  settings = [
    CatalogSetting(order=index, raw=raw, leaf=placement[name])
    for name, (index, raw) in by_name.items()
  ]
  settings.sort(key=lambda item: item.order)
  return settings, leaves


def _semantic_facts(raw: dict[str, Any]) -> dict[str, Any]:
  return {
    "name": raw["name"],
    "descriptions": {
      "ko": _normalize_text(raw.get("descr")),
      "en": _normalize_text(raw.get("edescr")),
      "zh": _normalize_text(raw.get("cdescr")),
    },
    "control": _control_kind(raw),
    "default": raw.get("default"),
    "min": raw.get("min"),
    "max": raw.get("max"),
    "unit": raw.get("unit"),
    "displayUnit": raw.get("display_unit", "raw"),
    "options": raw.get("options"),
    "risk": raw.get("risk"),
  }


def _localized_facts(setting: CatalogSetting, locale: str) -> dict[str, Any]:
  raw = setting.raw
  title_field, descr_field, _ = LOCALE_FIELDS[locale]
  return {
    "semantic": _semantic_facts(raw),
    "locale": locale,
    "title": _normalize_text(raw[title_field]),
    "description": _normalize_text(raw[descr_field]),
    "menuIds": setting.leaf.ids,
    "menuLabels": setting.leaf.labels[locale],
  }


def _format_value(raw: dict[str, Any], value: Any, locale: str) -> str:
  options = raw.get("options")
  minimum = raw.get("min")
  if isinstance(options, dict) and isinstance(options.get(locale), list) and isinstance(minimum, (int, float)):
    index = value - minimum if isinstance(value, (int, float)) else None
    if isinstance(index, (int, float)) and float(index).is_integer():
      labels = options[locale]
      int_index = int(index)
      if 0 <= int_index < len(labels):
        return f"{labels[int_index]} (`{value}`)"
  unit = DISPLAY_UNITS.get(raw.get("display_unit", "raw"), "")
  return f"`{value}{unit}`"


def _manual_block(param: str, existing: ExistingSetting | None) -> str:
  if existing is not None:
    return existing.manual_block
  return (
    f"<!-- CARROT:MANUAL:BEGIN {param} -->\n"
    '<!-- CARROT:REVIEW reviewed-for="" -->\n'
    f"<!-- CARROT:MANUAL:END {param} -->"
  )


def _reviewed_for(manual_block: str) -> str | None:
  match = REVIEW_RE.search(manual_block)
  return match.group("semantic") or None if match else None


def _render_setting(
  setting: CatalogSetting,
  locale: str,
  existing: ExistingSetting | None,
) -> tuple[str, GeneratedSetting]:
  labels = LABELS[locale]
  raw = setting.raw
  title_field, descr_field, _ = LOCALE_FIELDS[locale]
  title = _markdown_text(raw[title_field])
  description = _markdown_text(raw[descr_field] or raw[title_field])
  facts_hash = _canonical_hash(_localized_facts(setting, locale))
  semantic_hash = _canonical_hash(_semantic_facts(raw))
  manual = _manual_block(setting.param, existing)
  review_status = "current" if _reviewed_for(manual) == semantic_hash else "needs_review"
  kind = _control_kind(raw)

  overview = [
    f"- **{labels['type']}:** {labels['types'][kind]}",
    f"- **{labels['default']}:** {_format_value(raw, raw.get('default'), locale)}",
  ]
  if isinstance(raw.get("min"), (int, float)) and isinstance(raw.get("max"), (int, float)):
    overview.append(
      f"- **{labels['range']}:** {_format_value(raw, raw['min'], locale)} ~ "
      f"{_format_value(raw, raw['max'], locale)}"
    )
  if isinstance(raw.get("unit"), (int, float)) and kind not in {"toggle", "text"}:
    overview.append(f"- **{labels['step']}:** `{raw['unit']}`")
  display_unit = DISPLAY_UNITS.get(raw.get("display_unit", "raw"), "")
  if display_unit:
    overview.append(f"- **{labels['unit']}:** `{display_unit}`")
  options = raw.get("options", {}).get(locale) if isinstance(raw.get("options"), dict) else None
  if isinstance(options, list):
    overview.append(
      f"- **{labels['options']}:** "
      + ", ".join(f"`{index + raw.get('min', 0)}` {_markdown_text(option)}" for index, option in enumerate(options))
    )
  overview.append(f"- **{labels['menu']}:** {_markdown_text(setting.leaf.path_label(locale))}")

  block = "\n".join([
    f"<!-- CARROT:SETTING:BEGIN {setting.param} -->",
    f'<a id="{setting.param}"></a>',
    "",
    f"## {title}",
    "",
    f"`{setting.param}`",
    "",
    description,
    "",
    f"<!-- CARROT:AUTO:BEGIN {setting.param} -->",
    f'<!-- CARROT:AUTO:SOURCE facts="{facts_hash}" semantic="{semantic_hash}" review="{review_status}" -->',
    "<!-- CARROT:SECTION:BEGIN OVERVIEW -->",
    *overview,
    "<!-- CARROT:SECTION:END OVERVIEW -->",
    f"<!-- CARROT:AUTO:END {setting.param} -->",
    "",
    manual,
    "",
    f"<!-- CARROT:SETTING:END {setting.param} -->",
    "",
  ])
  generated = GeneratedSetting(
    param=setting.param,
    page=setting.page_name(locale),
    anchor=setting.param,
    facts_hash=facts_hash,
    semantic_hash=semantic_hash,
    review_status=review_status,
  )
  return block, generated


def _render_setting_page(
  setting: CatalogSetting,
  locale: str,
  existing: dict[tuple[str, str], ExistingSetting],
) -> tuple[str, GeneratedSetting]:
  labels = LABELS[locale]
  title = _markdown_text(setting.raw[LOCALE_FIELDS[locale][0]])
  menu = _markdown_text(setting.leaf.path_label(locale))
  prior = existing.get((locale, setting.param)) or existing.get(("*", setting.param))
  block, metadata = _render_setting(setting, locale, prior)
  page = "\n".join([
    f'<!-- CARROT:GENERATED schema="{SCHEMA_VERSION}" locale="{locale}" '
    f'group="{setting.leaf.id}" param="{setting.param}" -->',
    f'<!-- CARROT:AUTHORING guide="{AUTHORING_GUIDE_URL}" editable="CARROT:MANUAL only" -->',
    f"# {title}",
    "",
    f"**{labels['menu']}:** {menu}",
    "",
    "---",
    "",
    block,
  ])
  if not page.endswith("\n"):
    page += "\n"
  if metadata.page != setting.page_name(locale):
    raise AssertionError("generated setting page mismatch")
  return page, metadata


def _render_catalog_page(
  settings: list[CatalogSetting],
  leaves: list[MenuLeaf],
  locales: tuple[str, ...],
) -> str:
  lines = [
    "<!-- CARROT:GENERATED-CATALOG -->",
    "# 전체 설정 · Settings catalog · 设置目录",
    "",
    "Generated from `openpilot/selfdrive/carrot_settings.json`.",
    "",
  ]
  for locale in locales:
    labels = LABELS[locale]
    lines.extend((f"## {labels['catalog_title']} · `{locale}`", "", labels["catalog_intro"], ""))
    previous_labels: tuple[str, ...] = ()
    for leaf in leaves:
      leaf_settings = [setting for setting in settings if setting.leaf.id == leaf.id]
      if not leaf_settings:
        continue
      current_labels = leaf.labels[locale]
      common = 0
      while (
        common < len(previous_labels)
        and common < len(current_labels)
        and previous_labels[common] == current_labels[common]
      ):
        common += 1
      for depth, label in enumerate(current_labels[common:], start=common):
        heading_level = 3 + depth
        if heading_level > 6:
          raise GenerationError(
            f"{leaf.id}: menu depth exceeds the GitHub Wiki heading limit for locale {locale}"
          )
        lines.extend((f"{'#' * heading_level} {_markdown_text(label)}", ""))
      for setting in leaf_settings:
        title = _markdown_text(setting.raw[LOCALE_FIELDS[locale][0]])
        page = setting.page_name(locale)
        target = (
          "https://github.com/ajouatom/openpilot/wiki/"
          f"{quote(page.removesuffix('.md'), safe='-._~')}"
        )
        lines.append(f"- [{title}]({target}) — `{setting.param}`")
      lines.append("")
      previous_labels = current_labels
  return "\n".join(lines)


def _render_sidebar_catalog(
  settings: list[CatalogSetting],
  leaves: list[MenuLeaf],
  indent: str,
) -> list[str]:
  lines = [
    f"{indent}- [[{SIDEBAR_CATALOG_LABEL}|{CATALOG_PAGE_NAME.removesuffix('.md')}]] "
    f"{SIDEBAR_CATALOG_BEGIN_MARKER}"
  ]
  previous_labels: tuple[str, ...] = ()
  rendered_settings = 0
  for leaf in leaves:
    leaf_settings = [setting for setting in settings if setting.leaf.id == leaf.id]
    if not leaf_settings:
      continue
    current_labels = leaf.labels["ko"]
    common = 0
    while (
      common < len(previous_labels)
      and common < len(current_labels)
      and previous_labels[common] == current_labels[common]
    ):
      common += 1
    for depth, label in enumerate(current_labels[common:], start=common):
      item_indent = f"{indent}{'  ' * (depth + 1)}"
      lines.append(f"{item_indent}- {_markdown_text(label)}")
    setting_indent = f"{indent}{'  ' * (len(current_labels) + 1)}"
    for setting in leaf_settings:
      title = _markdown_text(setting.raw[LOCALE_FIELDS["ko"][0]]).replace("|", r"\|")
      target = setting.page_name("ko").removesuffix(".md")
      lines.append(f"{setting_indent}- [[{title}|{target}]]")
      rendered_settings += 1
    previous_labels = current_labels

  if not rendered_settings:
    raise GenerationError(f"{SIDEBAR_NAME}: cannot render an empty settings tree")
  lines[-1] = f"{lines[-1]} {SIDEBAR_CATALOG_END_MARKER}"
  return lines


def _render_sidebar(
  existing: str,
  settings: list[CatalogSetting],
  leaves: list[MenuLeaf],
) -> str:
  normalized = existing.replace("\r\n", "\n").replace("\r", "\n")
  lines = normalized.splitlines()
  begin_lines = [
    index for index, line in enumerate(lines) if SIDEBAR_CATALOG_BEGIN_MARKER in line
  ]
  end_lines = [
    index for index, line in enumerate(lines) if SIDEBAR_CATALOG_END_MARKER in line
  ]
  legacy_lines = [
    index for index, line in enumerate(lines) if SIDEBAR_CATALOG_MARKER in line
  ]
  if len(begin_lines) > 1 or len(end_lines) > 1 or len(legacy_lines) > 1:
    raise GenerationError(f"{SIDEBAR_NAME}: contains duplicate settings catalog markers")
  if bool(begin_lines) != bool(end_lines):
    raise GenerationError(f"{SIDEBAR_NAME}: contains an incomplete settings catalog block")
  if begin_lines and legacy_lines:
    raise GenerationError(f"{SIDEBAR_NAME}: mixes legacy and current settings catalog markers")

  if begin_lines:
    begin_index = begin_lines[0]
    end_index = end_lines[0]
    if begin_index > end_index:
      raise GenerationError(f"{SIDEBAR_NAME}: settings catalog markers are out of order")
    indent = re.match(r"^\s*", lines[begin_index]).group(0)
    lines[begin_index:end_index + 1] = _render_sidebar_catalog(settings, leaves, indent)
  elif legacy_lines:
    marker_index = legacy_lines[0]
    indent = re.match(r"^\s*", lines[marker_index]).group(0)
    lines[marker_index:marker_index + 1] = _render_sidebar_catalog(settings, leaves, indent)
  else:
    guide_lines = [
      index for index, line in enumerate(lines)
      if SIDEBAR_GUIDE_LABEL in line and re.match(r"^\s*-\s+", line)
    ]
    if len(guide_lines) != 1:
      raise GenerationError(
        f"{SIDEBAR_NAME}: expected exactly one {SIDEBAR_GUIDE_LABEL!r} navigation item"
      )
    guide_index = guide_lines[0]
    parent_indent = re.match(r"^\s*", lines[guide_index]).group(0)
    child_indent = f"{parent_indent}  "
    lines[guide_index + 1:guide_index + 1] = _render_sidebar_catalog(
      settings,
      leaves,
      child_indent,
    )
  return "\n".join(lines) + "\n"


def _read_candidate_pages(wiki_dir: Path | None) -> dict[str, str]:
  if wiki_dir is None or not wiki_dir.is_dir():
    return {}
  pages: dict[str, str] = {}
  for path in wiki_dir.glob("*.md"):
    try:
      text = path.read_bytes().decode("utf-8")
      pages[path.name] = text.replace("\r\n", "\n").replace("\r", "\n")
    except UnicodeDecodeError as error:
      raise GenerationError(f"Wiki page must be UTF-8: {path}: {error}") from error
  return pages


def _load_existing_index(wiki_dir: Path | None) -> dict[str, Any] | None:
  if wiki_dir is None:
    return None
  path = wiki_dir / INDEX_NAME
  if not path.is_file():
    return None
  try:
    payload = json.loads(path.read_text(encoding="utf-8"))
  except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
    raise GenerationError(f"cannot read existing {INDEX_NAME}: {error}") from error
  if not isinstance(payload, dict):
    raise GenerationError(f"existing {INDEX_NAME} must contain a JSON object")
  if (
    payload.get("schemaVersion") != SCHEMA_VERSION
    or not isinstance(payload.get("pages"), dict)
    or not isinstance(payload.get("settings"), dict)
  ):
    raise GenerationError(
      f"existing {INDEX_NAME} is not a managed Carrot settings index; refusing to overwrite it"
    )
  return payload


def scan_existing_settings(wiki_dir: Path | None) -> tuple[dict[tuple[str, str], ExistingSetting], list[str]]:
  existing: dict[tuple[str, str], ExistingSetting] = {}
  warnings: list[str] = []
  for page_name, text in _read_candidate_pages(wiki_dir).items():
    legacy_reserved_name = bool(
      LEGACY_LOCALE_GENERATED_PAGE_RE.fullmatch(page_name)
      or LEGACY_GENERATED_PAGE_RE.fullmatch(page_name)
    )
    header_match = GENERATED_HEADER_RE.search(text)
    if not legacy_reserved_name and header_match is None:
      continue
    setting_matches = list(SETTING_BLOCK_RE.finditer(text))
    if not setting_matches:
      raise GenerationError(
        f"{page_name}: managed page name exists without CARROT setting markers; refusing to overwrite it"
      )
    locale_match = re.match(r"^([A-Z]{2,3})-", page_name)
    locale = (
      header_match.group("locale").lower()
      if header_match is not None
      else locale_match.group(1).lower() if locale_match else "*"
    )
    for match in setting_matches:
      param = match.group("param")
      block = match.group(0)
      manual_match = MANUAL_BLOCK_RE.search(block)
      if manual_match is None or manual_match.group("param") != param:
        raise GenerationError(f"{page_name}: {param} has no matching MANUAL region")
      source_match = AUTO_SOURCE_RE.search(block)
      manual = manual_match.group(0)
      if source_match is None:
        warnings.append(f"{page_name}: {param} has no CARROT:AUTO:SOURCE metadata; review required")
      if REVIEW_RE.search(manual) is None:
        warnings.append(f"{page_name}: {param} has no CARROT:REVIEW metadata; review required")
      key = (locale, param)
      if key in existing:
        raise GenerationError(f"{param} appears more than once for locale {locale}")
      existing[key] = ExistingSetting(
        param=param,
        page=page_name,
        manual_block=manual,
        facts_hash=source_match.group("facts") if source_match else None,
        semantic_hash=source_match.group("semantic") if source_match else None,
        review_status=source_match.group("review") if source_match else None,
        reviewed_for=_reviewed_for(manual),
      )
  return existing, warnings


def _setting_diff(
  generated: dict[tuple[str, str], GeneratedSetting],
  existing: dict[tuple[str, str], ExistingSetting],
  locales: tuple[str, ...],
) -> dict[str, list[str]]:
  current_params = {param for _locale, param in generated}
  old_params = {
    param
    for (locale, param), _setting in existing.items()
    if locale == "*" or locale in locales
  }
  added = sorted(current_params - old_params)
  removed = sorted(old_params - current_params)
  changed: set[str] = set()
  moved: set[str] = set()
  review_changed: set[str] = set()
  for param in sorted(current_params & old_params):
    candidates = [
      (locale, setting) for (locale, candidate), setting in existing.items()
      if candidate == param and (locale == "*" or locale in locales)
    ]
    for locale, item in candidates:
      current_locale = locales[0] if locale == "*" else locale
      current = generated.get((current_locale, param))
      if current is None:
        continue
      if item.facts_hash != current.facts_hash:
        changed.add(param)
      if item.page and item.page != current.page:
        moved.add(param)
      if item.review_status is not None and current.review_status != item.review_status:
        review_changed.add(param)
  return {
    "added": added,
    "changed": sorted(changed),
    "moved": sorted(moved),
    "removed": removed,
    "reviewChanged": sorted(review_changed),
  }


def _generated_page_names(wiki_dir: Path | None, existing_index: dict[str, Any] | None) -> set[str]:
  names = {CATALOG_PAGE_NAME, INDEX_NAME}
  if isinstance(existing_index, dict):
    pages = existing_index.get("pages")
    if isinstance(pages, dict):
      names.update(name for name in pages if isinstance(name, str))
  if wiki_dir and wiki_dir.is_dir():
    for path in wiki_dir.glob("*.md"):
      try:
        text = path.read_text(encoding="utf-8")
      except UnicodeDecodeError as error:
        raise GenerationError(f"Wiki page must be UTF-8: {path}: {error}") from error
      if (
        GENERATED_HEADER_RE.search(text)
        or (
          LEGACY_GENERATED_PAGE_RE.fullmatch(path.name)
          and SETTING_BLOCK_RE.search(text)
        )
      ):
        names.add(path.name)
  return names


def _page_diff(
  pages: dict[str, str],
  wiki_dir: Path | None,
  previous_generated_pages: set[str],
) -> dict[str, list[str]]:
  old = _read_candidate_pages(wiki_dir)
  old_index = ""
  if wiki_dir is not None and (wiki_dir / INDEX_NAME).is_file():
    old_index = (wiki_dir / INDEX_NAME).read_text(encoding="utf-8")
  added = sorted(name for name in pages if name not in old and name != INDEX_NAME)
  changed = sorted(name for name, text in pages.items() if name in old and old[name] != text)
  if INDEX_NAME in pages:
    if not old_index:
      added.append(INDEX_NAME)
    elif old_index != pages[INDEX_NAME]:
      changed.append(INDEX_NAME)
  removed = sorted(name for name in previous_generated_pages if name not in pages and name not in {INDEX_NAME})
  return {
    "added": sorted(set(added)),
    "changed": sorted(set(changed)),
    "removed": removed,
  }


def _stable_generated_at(
  existing_index: dict[str, Any] | None,
  *,
  candidate: dict[str, Any],
  requested: str | None,
) -> str:
  if requested:
    try:
      parsed = datetime.fromisoformat(requested.replace("Z", "+00:00"))
    except ValueError as error:
      raise GenerationError("--generated-at must be an ISO 8601 date-time") from error
    if parsed.tzinfo is None:
      raise GenerationError("--generated-at must include a timezone")
    return requested
  if isinstance(existing_index, dict):
    old = {key: value for key, value in existing_index.items() if key not in {"generatedAt", "contentHash"}}
    if old == candidate and isinstance(existing_index.get("generatedAt"), str):
      return existing_index["generatedAt"]
  return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def generate(
  catalog_path: Path = DEFAULT_CATALOG,
  *,
  wiki_dir: Path | None = None,
  index_dir: Path | None = None,
  locales: Iterable[str] = DEFAULT_LOCALES,
  catalog_commit: str | None = None,
  wiki_commit: str | None = None,
  generated_at: str | None = None,
) -> GenerationResult:
  requested_locales = tuple(dict.fromkeys(locale.lower() for locale in locales))
  if not requested_locales:
    raise GenerationError("at least one locale is required")
  settings, leaves = load_catalog(catalog_path, requested_locales)
  commit = (catalog_commit or _git_head()).lower()
  if not COMMIT_RE.fullmatch(commit):
    raise GenerationError("catalog commit must contain 7 to 40 lowercase hexadecimal characters")
  if wiki_commit is not None and not COMMIT_RE.fullmatch(wiki_commit.lower()):
    raise GenerationError("wiki commit must contain 7 to 40 lowercase hexadecimal characters")

  existing, warnings = scan_existing_settings(wiki_dir)
  existing_pages = _read_candidate_pages(wiki_dir)
  existing_catalog_page = existing_pages.get(CATALOG_PAGE_NAME)
  if (
    existing_catalog_page is not None
    and "<!-- CARROT:GENERATED-CATALOG -->" not in existing_catalog_page
  ):
    raise GenerationError(
      f"{CATALOG_PAGE_NAME}: existing page is not generator-managed; refusing to overwrite it"
    )
  for setting in settings:
    for locale in requested_locales:
      _title_field, descr_field, _node_field = LOCALE_FIELDS[locale]
      if not setting.raw[descr_field].strip():
        warnings.append(f"{setting.param}: empty {locale} description; title used as fallback")
  existing_index = _load_existing_index(index_dir or wiki_dir)

  pages: dict[str, str] = {}
  generated_by_locale: dict[tuple[str, str], GeneratedSetting] = {}
  generated_name_owners: dict[str, tuple[str, str]] = {}
  existing_managed_pages = {item.page for item in existing.values()}
  for locale in requested_locales:
    for setting in settings:
      page, metadata = _render_setting_page(setting, locale, existing)
      page_name = setting.page_name(locale)
      normalized_name = unicodedata.normalize("NFKC", page_name).casefold()
      prior_owner = generated_name_owners.get(normalized_name)
      if prior_owner is not None:
        raise GenerationError(
          f"localized setting titles collide on Wiki page {page_name}: "
          f"{prior_owner[1]} and {setting.param}"
        )
      generated_name_owners[normalized_name] = (locale, setting.param)
      if page_name in existing_pages and page_name not in existing_managed_pages:
        raise GenerationError(
          f"{page_name}: localized setting page already exists without Carrot markers; "
          "refusing to overwrite it"
        )
      pages[page_name] = page
      generated_by_locale[(locale, metadata.param)] = metadata
  pages[CATALOG_PAGE_NAME] = _render_catalog_page(settings, leaves, requested_locales)
  existing_sidebar = existing_pages.get(SIDEBAR_NAME)
  if existing_sidebar is not None and "ko" in requested_locales:
    pages[SIDEBAR_NAME] = _render_sidebar(existing_sidebar, settings, leaves)

  canonical_locale = "en" if "en" in requested_locales else requested_locales[0]
  canonical_settings = {
    param: metadata
    for (locale, param), metadata in generated_by_locale.items()
    if locale == canonical_locale
  }
  index_settings: dict[str, Any] = {}
  for setting in settings:
    localized = {
      locale: {
        "page": generated_by_locale[(locale, setting.param)].page,
        "anchor": setting.param,
        "factsHash": generated_by_locale[(locale, setting.param)].facts_hash,
        "reviewStatus": generated_by_locale[(locale, setting.param)].review_status,
      }
      for locale in requested_locales
    }
    canonical = canonical_settings[setting.param]
    aggregate_review = (
      "current"
      if all(item["reviewStatus"] == "current" for item in localized.values())
      else "needs_review"
    )
    index_settings[setting.param] = {
      "group": setting.leaf.id,
      "semanticHash": canonical.semantic_hash,
      "reviewStatus": aggregate_review,
      "locales": localized,
    }

  page_index = {
    name: {"hash": _text_hash(text), "bytes": len(text.encode("utf-8"))}
    for name, text in sorted(pages.items())
    if name != SIDEBAR_NAME
  }
  review_counts = {
    "current": sum(item["reviewStatus"] == "current" for item in index_settings.values()),
    "needs_review": sum(item["reviewStatus"] == "needs_review" for item in index_settings.values()),
  }
  index_candidate: dict[str, Any] = {
    "schemaVersion": SCHEMA_VERSION,
    "catalogCommit": commit,
    "locales": list(requested_locales),
    "pages": page_index,
    "settings": index_settings,
    "review": review_counts,
  }
  if wiki_commit is not None:
    index_candidate["wikiCommit"] = wiki_commit.lower()
  stable_time = _stable_generated_at(existing_index, candidate=index_candidate, requested=generated_at)
  index = {**index_candidate, "generatedAt": stable_time}
  index["contentHash"] = _canonical_hash(index)
  pages[INDEX_NAME] = _json_text(index)

  previous_generated_pages = _generated_page_names(wiki_dir, existing_index)
  page_changes = _page_diff(pages, wiki_dir, previous_generated_pages)
  setting_changes = _setting_diff(generated_by_locale, existing, requested_locales)
  return GenerationResult(
    pages=pages,
    index=index,
    generated_settings=canonical_settings,
    previous_generated_pages=previous_generated_pages,
    page_changes=page_changes,
    setting_changes=setting_changes,
    warnings=warnings,
    wiki_dir=wiki_dir,
  )


def write_result(result: GenerationResult, output_dir: Path) -> None:
  source = result.wiki_dir
  output_dir = output_dir.resolve()
  if source is not None:
    source = source.resolve()
  if source is not None and source != output_dir:
    output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, output_dir, dirs_exist_ok=True)
  else:
    output_dir.mkdir(parents=True, exist_ok=True)

  for name in sorted(result.previous_generated_pages - result.pages.keys()):
    target = output_dir / name
    if target.parent != output_dir:
      raise GenerationError(f"refusing to delete a generated path outside output directory: {name}")
    if target.is_file():
      target.unlink()
  for name, text in result.pages.items():
    target = output_dir / name
    if target.parent != output_dir:
      raise GenerationError(f"refusing to write a generated path outside output directory: {name}")
    target.write_bytes(text.encode("utf-8"))


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
    description="Generate group and locale Wiki pages from carrot_settings.json without touching the Wiki by default."
  )
  parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG, help="carrot_settings.json path")
  parser.add_argument("--wiki-dir", type=Path, help="existing Wiki checkout used to preserve MANUAL regions")
  parser.add_argument("--index-dir", type=Path, help="optional existing Published index used for stable timestamps")
  parser.add_argument("--locales", nargs="+", default=list(DEFAULT_LOCALES), help="locales to generate")
  parser.add_argument("--catalog-commit", help="source carrot-wip commit (defaults to local HEAD)")
  parser.add_argument("--wiki-commit", help="optional source Wiki commit recorded in the index")
  parser.add_argument("--generated-at", help="fixed ISO 8601 timestamp for reproducible runs")
  parser.add_argument("--write", action="store_true", help="write staged output; without this flag the command is dry-run")
  parser.add_argument("--output-dir", type=Path, help="staging directory used only with --write")
  parser.add_argument("--show-diff", action="store_true", help="print unified page diffs after the JSON report")
  return parser


def main(argv: list[str] | None = None) -> int:
  args = build_parser().parse_args(argv)
  if args.write and args.output_dir is None:
    print("--write requires --output-dir", file=sys.stderr)
    return 2
  if not args.write and args.output_dir is not None:
    print("--output-dir requires --write", file=sys.stderr)
    return 2
  try:
    result = generate(
      args.catalog,
      wiki_dir=args.wiki_dir,
      index_dir=args.index_dir,
      locales=args.locales,
      catalog_commit=args.catalog_commit,
      wiki_commit=args.wiki_commit,
      generated_at=args.generated_at,
    )
    if args.write:
      write_result(result, args.output_dir)
    report = result.report()
    report["mode"] = "write" if args.write else "dry-run"
    print(json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2))
    if args.show_diff:
      diff = result.unified_diff()
      if diff:
        print(diff, end="" if diff.endswith("\n") else "\n")
    return 0
  except (GenerationError, OSError) as error:
    print(f"Wiki settings generation failed: {error}", file=sys.stderr)
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
