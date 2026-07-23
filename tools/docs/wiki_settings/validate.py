#!/usr/bin/env python3
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any, Iterable
from urllib.parse import urlsplit


SPEC_ROOT = Path(__file__).resolve().parent
DEFAULT_SCHEMA = SPEC_ROOT / "content.schema.json"
DEFAULT_INPUT = SPEC_ROOT / "examples" / "valid"

SECTION_ORDER = ("overview", "behavior", "usage", "reference")
SAFE_HTML_TAGS = frozenset(("a", "details", "summary"))
COMMIT_RE = re.compile(r"^[a-f0-9]{7,40}$")
HASH_RE = re.compile(r"^[a-f0-9]{64}$")
LOCALE_RE = re.compile(r"^[A-Za-z]{2,3}(?:-[A-Za-z0-9]{2,8})*$")
PARAM_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")

SETTING_BEGIN_RE = re.compile(r"^<!-- CARROT:SETTING:BEGIN ([A-Za-z][A-Za-z0-9_]*) -->$")
SETTING_END_RE = re.compile(r"^<!-- CARROT:SETTING:END ([A-Za-z][A-Za-z0-9_]*) -->$")
AUTO_BEGIN_RE = re.compile(r"^<!-- CARROT:AUTO:BEGIN ([A-Za-z][A-Za-z0-9_]*) -->$")
AUTO_END_RE = re.compile(r"^<!-- CARROT:AUTO:END ([A-Za-z][A-Za-z0-9_]*) -->$")
MANUAL_BEGIN_RE = re.compile(r"^<!-- CARROT:MANUAL:BEGIN ([A-Za-z][A-Za-z0-9_]*) -->$")
MANUAL_END_RE = re.compile(r"^<!-- CARROT:MANUAL:END ([A-Za-z][A-Za-z0-9_]*) -->$")
SECTION_BEGIN_RE = re.compile(r"^<!-- CARROT:SECTION:BEGIN ([A-Z]+) -->$")
SECTION_END_RE = re.compile(r"^<!-- CARROT:SECTION:END ([A-Z]+) -->$")
NOTICE_BEGIN_RE = re.compile(r'^<!-- CARROT:NOTICE level="([A-Z]+)" -->$')
NOTICE_END_RE = re.compile(r"^<!-- CARROT:NOTICE:END -->$")
ALERT_RE = re.compile(r"^>\s*\[!([A-Z]+)\]\s*$")
RAW_TAG_RE = re.compile(r"<\s*/?\s*([A-Za-z][A-Za-z0-9-]*)\b[^>]*>")
EVENT_ATTRIBUTE_RE = re.compile(r"<[^>]+\son[a-z0-9_-]+\s*=", re.IGNORECASE)
IMAGE_RE = re.compile(r"!\[([^\]]*)\]\(([^)]+)\)")
LINK_RE = re.compile(r"(?<!!)\[([^\]]+)\]\(([^)]+)\)")
TABLE_DELIMITER_RE = re.compile(r"^\s*\|?\s*:?-{3,}:?\s*(?:\|\s*:?-{3,}:?\s*)+\|?\s*$")


@dataclass(frozen=True)
class ValidationIssue:
  location: str
  message: str
  line: int | None = None

  def format(self, path: Path) -> str:
    line = f":{self.line}" if self.line is not None else ""
    location = f" [{self.location}]" if self.location else ""
    return f"{path}{line}{location}: {self.message}"


@dataclass
class MarkdownSettingState:
  param: str
  line: int
  auto_param: str | None = None
  manual_param: str | None = None
  section: str | None = None
  notice_level: str | None = None
  notice_has_alert: bool = False
  sections: list[str] | None = None
  notice_count: int = 0

  def __post_init__(self) -> None:
    if self.sections is None:
      self.sections = []


def _issue(issues: list[ValidationIssue], location: str, message: str, line: int | None = None) -> None:
  issues.append(ValidationIssue(location, message, line))


def _load_json(path: Path) -> tuple[Any | None, list[ValidationIssue]]:
  try:
    return json.loads(path.read_text(encoding="utf-8")), []
  except UnicodeDecodeError as error:
    return None, [ValidationIssue("", f"file must be UTF-8: {error}")]
  except json.JSONDecodeError as error:
    return None, [ValidationIssue("", f"invalid JSON: {error.msg}", error.lineno)]


def load_schema(path: Path = DEFAULT_SCHEMA) -> dict[str, Any]:
  payload, issues = _load_json(path)
  if issues:
    raise ValueError(issues[0].message)
  if not isinstance(payload, dict):
    raise ValueError(f"schema root must be an object: {path}")
  if payload.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
    raise ValueError(f"unsupported schema dialect: {payload.get('$schema')!r}")
  if payload.get("properties", {}).get("schemaVersion", {}).get("const") != 1:
    raise ValueError("validator supports content schema version 1 only")
  return payload


def canonical_content_hash(document: dict[str, Any]) -> str:
  normalized = deepcopy(document)
  source = normalized.get("source")
  if isinstance(source, dict):
    source.pop("contentHash", None)
  encoded = json.dumps(normalized, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
  return hashlib.sha256(encoded).hexdigest()


def _check_object(
  value: Any,
  location: str,
  issues: list[ValidationIssue],
  *,
  allowed: set[str],
  required: set[str] = frozenset(),
) -> dict[str, Any] | None:
  if not isinstance(value, dict):
    _issue(issues, location, "must be an object")
    return None
  for key in sorted(required - value.keys()):
    _issue(issues, f"{location}.{key}", "required property is missing")
  for key in sorted(value.keys() - allowed):
    _issue(issues, f"{location}.{key}", "unknown property")
  return value


def _check_string(
  value: Any,
  location: str,
  issues: list[ValidationIssue],
  *,
  minimum: int = 1,
  maximum: int | None = None,
  pattern: re.Pattern[str] | None = None,
) -> str | None:
  if not isinstance(value, str):
    _issue(issues, location, "must be a string")
    return None
  if len(value) < minimum:
    _issue(issues, location, f"must contain at least {minimum} character(s)")
  if maximum is not None and len(value) > maximum:
    _issue(issues, location, f"must contain at most {maximum} characters")
  if pattern is not None and not pattern.fullmatch(value):
    _issue(issues, location, "has an invalid format")
  return value


def _check_datetime(value: Any, location: str, issues: list[ValidationIssue]) -> None:
  text = _check_string(value, location, issues)
  if text is None:
    return
  try:
    parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
      raise ValueError("timezone is required")
  except ValueError:
    _issue(issues, location, "must be an ISO 8601 date-time with a timezone")


def _schema_contract(schema: dict[str, Any]) -> tuple[dict[str, set[str]], dict[str, set[str]]]:
  section_tokens: dict[str, set[str]] = {}
  for definition_name in ("overviewSection", "behaviorSection", "usageSection", "referenceSection"):
    definition = schema["$defs"][definition_name]
    kind = definition["properties"]["kind"]["const"]
    token_enum = definition["properties"]["blocks"]["items"]["allOf"][1]["properties"]["token"]["enum"]
    section_tokens[kind] = set(token_enum)

  enums = {
    "notice_levels": set(schema["$defs"]["notice"]["properties"]["level"]["enum"]),
    "relation_types": set(schema["$defs"]["relation"]["properties"]["type"]["enum"]),
    "media_types": set(schema["$defs"]["media"]["properties"]["type"]["enum"]),
    "lifecycle_statuses": set(schema["$defs"]["lifecycle"]["properties"]["status"]["enum"]),
    "review_statuses": set(schema["$defs"]["review"]["properties"]["status"]["enum"]),
  }
  return section_tokens, enums


def validate_published_document(document: Any, schema: dict[str, Any]) -> list[ValidationIssue]:
  issues: list[ValidationIssue] = []
  root_properties = set(schema["properties"])
  root_required = set(schema["required"])
  root = _check_object(document, "$", issues, allowed=root_properties, required=root_required)
  if root is None:
    return issues

  expected_version = schema["properties"]["schemaVersion"]["const"]
  if root.get("schemaVersion") != expected_version:
    _issue(issues, "$.schemaVersion", f"must equal {expected_version}")

  source = _check_object(
    root.get("source"), "$.source", issues,
    allowed={"catalogCommit", "wikiCommit", "generatedAt", "contentHash"},
    required={"catalogCommit", "wikiCommit", "generatedAt", "contentHash"},
  )
  if source is not None:
    _check_string(source.get("catalogCommit"), "$.source.catalogCommit", issues, pattern=COMMIT_RE)
    _check_string(source.get("wikiCommit"), "$.source.wikiCommit", issues, pattern=COMMIT_RE)
    _check_datetime(source.get("generatedAt"), "$.source.generatedAt", issues)
    content_hash = _check_string(source.get("contentHash"), "$.source.contentHash", issues, pattern=HASH_RE)
    if content_hash is not None and HASH_RE.fullmatch(content_hash):
      expected_hash = canonical_content_hash(root)
      if content_hash != expected_hash:
        _issue(issues, "$.source.contentHash", f"does not match canonical content hash {expected_hash}")

  _check_string(root.get("locale"), "$.locale", issues, pattern=LOCALE_RE)

  identity = _check_object(
    root.get("identity"), "$.identity", issues,
    allowed={"param", "title", "summary"},
    required={"param", "title", "summary"},
  )
  if identity is not None:
    _check_string(identity.get("param"), "$.identity.param", issues, pattern=PARAM_RE)
    _check_string(identity.get("title"), "$.identity.title", issues, maximum=120)
    _check_string(identity.get("summary"), "$.identity.summary", issues, maximum=300)

  section_tokens, enums = _schema_contract(schema)
  sections = root.get("sections")
  section_kinds: list[str] = []
  if not isinstance(sections, list):
    _issue(issues, "$.sections", "must be an array")
  else:
    if not 1 <= len(sections) <= 4:
      _issue(issues, "$.sections", "must contain between 1 and 4 sections")
    for index, raw_section in enumerate(sections):
      location = f"$.sections[{index}]"
      section = _check_object(
        raw_section, location, issues,
        allowed={"kind", "blocks"}, required={"kind", "blocks"},
      )
      if section is None:
        continue
      kind = section.get("kind")
      if not isinstance(kind, str) or kind not in section_tokens:
        _issue(issues, f"{location}.kind", f"must be one of {', '.join(SECTION_ORDER)}")
        continue
      section_kinds.append(kind)
      blocks = section.get("blocks")
      if not isinstance(blocks, list) or not blocks:
        _issue(issues, f"{location}.blocks", "must be a non-empty array")
        continue
      seen_tokens: set[str] = set()
      for block_index, raw_block in enumerate(blocks):
        block_location = f"{location}.blocks[{block_index}]"
        block = _check_object(
          raw_block, block_location, issues,
          allowed={"token", "markdown"}, required={"token", "markdown"},
        )
        if block is None:
          continue
        token = block.get("token")
        if not isinstance(token, str) or token not in section_tokens[kind]:
          _issue(issues, f"{block_location}.token", f"token is not allowed in {kind}")
        elif token in seen_tokens:
          _issue(issues, f"{block_location}.token", "duplicate token in section")
        else:
          seen_tokens.add(token)
        _check_string(block.get("markdown"), f"{block_location}.markdown", issues, maximum=10000)

  if section_kinds.count("overview") != 1:
    _issue(issues, "$.sections", "must contain exactly one overview section")
  if len(section_kinds) != len(set(section_kinds)):
    _issue(issues, "$.sections", "section kinds must be unique")
  order = [SECTION_ORDER.index(kind) for kind in section_kinds]
  if order != sorted(order):
    _issue(issues, "$.sections", "sections must follow overview, behavior, usage, reference order")

  _validate_scope(root.get("scope"), issues)
  _validate_notices(root.get("notices"), issues, enums["notice_levels"], schema)
  _validate_relations(root.get("relations"), issues, enums["relation_types"])
  _validate_media(root.get("media"), issues, enums["media_types"])
  _validate_lifecycle(root.get("lifecycle"), issues, enums["lifecycle_statuses"])
  _validate_review(root.get("review"), issues, enums["review_statuses"])
  return issues


def _validate_scope(value: Any, issues: list[ValidationIssue]) -> None:
  if value is None:
    return
  allowed = {"vehicles", "hardware", "software", "regions", "modes", "connections"}
  scope = _check_object(value, "$.scope", issues, allowed=allowed)
  if scope is None:
    return
  for key, items in scope.items():
    location = f"$.scope.{key}"
    if not isinstance(items, list):
      _issue(issues, location, "must be an array")
      continue
    if len(items) != len(set(item for item in items if isinstance(item, str))):
      _issue(issues, location, "must not contain duplicate values")
    for index, item in enumerate(items):
      _check_string(item, f"{location}[{index}]", issues, maximum=120)


def _validate_notices(
  value: Any,
  issues: list[ValidationIssue],
  allowed_levels: set[str],
  schema: dict[str, Any],
) -> None:
  if not isinstance(value, list):
    _issue(issues, "$.notices", "must be an array")
    return
  maximum = schema["properties"]["notices"]["maxItems"]
  if len(value) > maximum:
    _issue(issues, "$.notices", f"must contain at most {maximum} notices")
  for index, raw_notice in enumerate(value):
    location = f"$.notices[{index}]"
    notice = _check_object(
      raw_notice, location, issues,
      allowed={"level", "section", "markdown"}, required={"level", "section", "markdown"},
    )
    if notice is None:
      continue
    if notice.get("level") not in allowed_levels:
      _issue(issues, f"{location}.level", "unsupported notice level")
    if notice.get("section") not in SECTION_ORDER:
      _issue(issues, f"{location}.section", "unsupported target section")
    _check_string(notice.get("markdown"), f"{location}.markdown", issues, maximum=1000)


def _validate_relations(value: Any, issues: list[ValidationIssue], allowed_types: set[str]) -> None:
  if not isinstance(value, list):
    _issue(issues, "$.relations", "must be an array")
    return
  seen: set[tuple[Any, Any]] = set()
  for index, raw_relation in enumerate(value):
    location = f"$.relations[{index}]"
    relation = _check_object(
      raw_relation, location, issues,
      allowed={"type", "target"}, required={"type", "target"},
    )
    if relation is None:
      continue
    if relation.get("type") not in allowed_types:
      _issue(issues, f"{location}.type", "unsupported relation type")
    _check_string(relation.get("target"), f"{location}.target", issues, pattern=PARAM_RE)
    pair = (relation.get("type"), relation.get("target"))
    if pair in seen:
      _issue(issues, location, "duplicate relation")
    seen.add(pair)


def _validate_media(value: Any, issues: list[ValidationIssue], allowed_types: set[str]) -> None:
  if value is None:
    return
  if not isinstance(value, list):
    _issue(issues, "$.media", "must be an array")
    return
  for index, raw_media in enumerate(value):
    location = f"$.media[{index}]"
    media = _check_object(
      raw_media, location, issues,
      allowed={"type", "url", "alt", "caption", "durationSeconds"},
      required={"type", "url", "alt"},
    )
    if media is None:
      continue
    if media.get("type") not in allowed_types:
      _issue(issues, f"{location}.type", "unsupported media type")
    url = _check_string(media.get("url"), f"{location}.url", issues)
    if url is not None and urlsplit(url).scheme.lower() not in {"http", "https"}:
      _issue(issues, f"{location}.url", "media URL must use HTTP or HTTPS")
    _check_string(media.get("alt"), f"{location}.alt", issues, maximum=300)
    if "caption" in media:
      _check_string(media["caption"], f"{location}.caption", issues, minimum=0, maximum=500)
    if "durationSeconds" in media:
      duration = media["durationSeconds"]
      if not isinstance(duration, (int, float)) or isinstance(duration, bool) or duration <= 0:
        _issue(issues, f"{location}.durationSeconds", "must be a positive number")


def _validate_lifecycle(value: Any, issues: list[ValidationIssue], allowed_statuses: set[str]) -> None:
  lifecycle = _check_object(
    value, "$.lifecycle", issues,
    allowed={"status", "introduced", "changed"}, required={"status"},
  )
  if lifecycle is None:
    return
  if lifecycle.get("status") not in allowed_statuses:
    _issue(issues, "$.lifecycle.status", "unsupported lifecycle status")
  for key in ("introduced", "changed"):
    if key in lifecycle:
      _check_string(lifecycle[key], f"$.lifecycle.{key}", issues)


def _validate_review(value: Any, issues: list[ValidationIssue], allowed_statuses: set[str]) -> None:
  review = _check_object(
    value, "$.review", issues,
    allowed={"status", "reviewedForCommit", "reviewedAt"},
    required={"status", "reviewedForCommit"},
  )
  if review is None:
    return
  status = review.get("status")
  if status not in allowed_statuses:
    _issue(issues, "$.review.status", "unsupported review status")
  _check_string(review.get("reviewedForCommit"), "$.review.reviewedForCommit", issues, pattern=COMMIT_RE)
  if status == "current" and "reviewedAt" not in review:
    _issue(issues, "$.review.reviewedAt", "is required when review status is current")
  if "reviewedAt" in review:
    _check_datetime(review["reviewedAt"], "$.review.reviewedAt", issues)


def _link_is_safe(target: str) -> bool:
  clean = target.strip().split(maxsplit=1)[0]
  if clean.startswith("#"):
    return bool(re.fullmatch(r"#[A-Za-z0-9_-]+", clean))
  return urlsplit(clean).scheme.lower() in {"http", "https"}


def _table_width(line: str) -> int:
  clean = line.strip().strip("|")
  return len([cell for cell in clean.split("|") if cell.strip()])


def validate_wiki_markdown(text: str) -> list[ValidationIssue]:
  issues: list[ValidationIssue] = []
  state: MarkdownSettingState | None = None
  fenced = False
  last_heading_level = 0

  for line_number, line in enumerate(text.replace("\r\n", "\n").split("\n"), start=1):
    stripped = line.strip()
    if stripped.startswith("```"):
      fenced = not fenced
      continue
    if fenced:
      continue

    match = SETTING_BEGIN_RE.fullmatch(stripped)
    if match:
      if state is not None:
        _issue(issues, "markdown", "nested setting marker", line_number)
      else:
        state = MarkdownSettingState(match.group(1), line_number)
      continue

    match = SETTING_END_RE.fullmatch(stripped)
    if match:
      if state is None:
        _issue(issues, "markdown", "setting end without a matching begin", line_number)
      else:
        if match.group(1) != state.param:
          _issue(issues, "markdown", "setting end parameter does not match begin", line_number)
        if state.auto_param or state.manual_param or state.section or state.notice_level:
          _issue(issues, state.param, "setting ended with an open nested marker", line_number)
        assert state.sections is not None
        lowered = [section.lower() for section in state.sections]
        if lowered.count("overview") != 1:
          _issue(issues, state.param, "must contain exactly one OVERVIEW section", line_number)
        if len(lowered) != len(set(lowered)):
          _issue(issues, state.param, "section kinds must be unique", line_number)
        order = [SECTION_ORDER.index(section) for section in lowered if section in SECTION_ORDER]
        if order != sorted(order):
          _issue(issues, state.param, "sections are out of order", line_number)
        if state.notice_count > 2:
          _issue(issues, state.param, "must contain at most two notices", line_number)
      state = None
      continue

    for pattern, field, label in (
      (AUTO_BEGIN_RE, "auto_param", "AUTO"),
      (MANUAL_BEGIN_RE, "manual_param", "MANUAL"),
    ):
      match = pattern.fullmatch(stripped)
      if match:
        if state is None:
          _issue(issues, "markdown", f"{label} begin must be inside a setting", line_number)
        elif state.auto_param or state.manual_param:
          _issue(issues, state.param, f"{label} region overlaps another region", line_number)
        elif match.group(1) != state.param:
          _issue(issues, state.param, f"{label} parameter does not match setting", line_number)
        else:
          setattr(state, field, match.group(1))
        break
    else:
      match = None
    if match:
      continue

    for pattern, field, label in (
      (AUTO_END_RE, "auto_param", "AUTO"),
      (MANUAL_END_RE, "manual_param", "MANUAL"),
    ):
      match = pattern.fullmatch(stripped)
      if match:
        if state is None or getattr(state, field) != match.group(1):
          _issue(issues, "markdown", f"{label} end has no matching begin", line_number)
        elif state.section or state.notice_level:
          _issue(issues, state.param, f"{label} ended with an open nested marker", line_number)
        else:
          setattr(state, field, None)
        break
    else:
      match = None
    if match:
      continue

    match = SECTION_BEGIN_RE.fullmatch(stripped)
    if match:
      section = match.group(1)
      if state is None or not (state.auto_param or state.manual_param):
        _issue(issues, "markdown", "section must be inside AUTO or MANUAL", line_number)
      elif state.section is not None:
        _issue(issues, state.param, "nested section marker", line_number)
      elif section.lower() not in SECTION_ORDER:
        _issue(issues, state.param, f"unsupported section {section}", line_number)
      else:
        state.section = section
        assert state.sections is not None
        state.sections.append(section)
      continue

    match = SECTION_END_RE.fullmatch(stripped)
    if match:
      if state is None or state.section != match.group(1):
        _issue(issues, "markdown", "section end has no matching begin", line_number)
      elif state.notice_level:
        _issue(issues, state.param, "section ended with an open notice", line_number)
      else:
        state.section = None
      continue

    match = NOTICE_BEGIN_RE.fullmatch(stripped)
    if match:
      if state is None or state.section is None:
        _issue(issues, "markdown", "notice must be inside a section", line_number)
      elif state.notice_level is not None:
        _issue(issues, state.param, "nested notice marker", line_number)
      else:
        state.notice_level = match.group(1)
        state.notice_has_alert = False
        state.notice_count += 1
      continue

    if NOTICE_END_RE.fullmatch(stripped):
      if state is None or state.notice_level is None:
        _issue(issues, "markdown", "notice end has no matching begin", line_number)
      else:
        if not state.notice_has_alert:
          _issue(issues, state.param, "notice must contain a matching GitHub Alert", line_number)
        state.notice_level = None
        state.notice_has_alert = False
      continue

    alert = ALERT_RE.fullmatch(stripped)
    if alert and state is not None and state.notice_level is not None:
      if alert.group(1) != state.notice_level:
        _issue(issues, state.param, "notice level and GitHub Alert level differ", line_number)
      state.notice_has_alert = True

    heading = re.match(r"^(#{1,6})\s+\S", stripped)
    if heading:
      level = len(heading.group(1))
      if last_heading_level and level > last_heading_level + 1:
        _issue(issues, "markdown", "heading level must not skip a level", line_number)
      last_heading_level = level

    lower = stripped.casefold()
    if any(f"<{tag}" in lower or f"</{tag}" in lower for tag in ("script", "style", "iframe")):
      _issue(issues, "markdown", "dangerous raw HTML tag", line_number)
    if EVENT_ATTRIBUTE_RE.search(line):
      _issue(issues, "markdown", "HTML event attributes are not allowed", line_number)
    for raw_tag in RAW_TAG_RE.findall(line):
      if raw_tag.casefold() not in SAFE_HTML_TAGS:
        _issue(issues, "markdown", f"raw HTML tag <{raw_tag}> is not allowed", line_number)
    if "<a " in lower and not re.fullmatch(r'\s*<a id="[A-Za-z][A-Za-z0-9_-]*"></a>\s*', line):
      _issue(issues, "markdown", "anchor HTML must contain only a safe id", line_number)

    for alt, target in IMAGE_RE.findall(line):
      if not alt.strip():
        _issue(issues, "markdown", "image alt text is required", line_number)
      if not _link_is_safe(target):
        _issue(issues, "markdown", "image URL must be an anchor or HTTP(S)", line_number)
    for _label, target in LINK_RE.findall(line):
      if not _link_is_safe(target):
        _issue(issues, "markdown", "link URL must be an anchor or HTTP(S)", line_number)

    if TABLE_DELIMITER_RE.fullmatch(line) and _table_width(line) > 3:
      _issue(issues, "markdown", "tables may contain at most three columns", line_number)

  if fenced:
    _issue(issues, "markdown", "unclosed fenced code block")
  if state is not None:
    _issue(issues, state.param, f"unclosed setting marker opened at line {state.line}")
  return issues


def validate_path(path: Path, schema: dict[str, Any]) -> list[ValidationIssue]:
  if path.suffix.casefold() == ".json":
    payload, issues = _load_json(path)
    return issues if issues else validate_published_document(payload, schema)
  if path.suffix.casefold() in {".md", ".markdown"}:
    try:
      text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError as error:
      return [ValidationIssue("", f"file must be UTF-8: {error}")]
    return validate_wiki_markdown(text)
  return [ValidationIssue("", "unsupported file type; expected .json, .md, or .markdown")]


def iter_input_files(paths: Iterable[Path]) -> Iterable[Path]:
  for path in paths:
    if path.is_dir():
      yield from sorted(
        candidate for candidate in path.rglob("*")
        if candidate.is_file() and candidate.suffix.casefold() in {".json", ".md", ".markdown"}
      )
    else:
      yield path


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(description="Validate Carrot Wiki setting content.")
  parser.add_argument("paths", nargs="*", type=Path, default=[DEFAULT_INPUT], help="files or directories to validate")
  parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA, help="content schema path")
  return parser


def main(argv: list[str] | None = None) -> int:
  args = build_parser().parse_args(argv)
  try:
    schema = load_schema(args.schema)
  except (OSError, ValueError) as error:
    print(f"{args.schema}: {error}", file=sys.stderr)
    return 2

  files = list(iter_input_files(args.paths))
  if not files:
    print("No supported input files found.", file=sys.stderr)
    return 2

  issue_count = 0
  for path in files:
    try:
      issues = validate_path(path, schema)
    except OSError as error:
      issues = [ValidationIssue("", str(error))]
    for issue in issues:
      print(issue.format(path), file=sys.stderr)
    issue_count += len(issues)

  if issue_count:
    print(f"Wiki settings validation failed: {issue_count} issue(s) in {len(files)} file(s).", file=sys.stderr)
    return 1
  print(f"Wiki settings validation passed: {len(files)} file(s).")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
