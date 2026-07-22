"""Read the code-owned user guide section for a Carrot Web setting.

The public guides live at repository root under ``docs/user`` instead of the
web static tree.  This module exposes only a fixed set of documents and maps a
setting through the existing settings menu, so request input can never become
an arbitrary filesystem path.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Any

from openpilot.common.basedir import BASEDIR

from .settings import get_settings_cached, settings_cache


USER_DOCS_ROOT = Path(BASEDIR) / "docs" / "user"
SUPPORTED_DOC_LANGUAGES = frozenset(("ko", "en", "zh"))

_DOC_REFERENCE_RE = re.compile(r"^(?P<source>[a-z0-9-]+)#(?P<anchor>[a-z0-9-]+)$")
_HEADING_RE = re.compile(r"^(?P<marks>#{1,6})[ \t]+\S.*$", re.MULTILINE)
_SECTION_NAV_RE = re.compile(
  r"^\[(?:\d+개 세부 구역으로 돌아가기|Back to (?:the )?.*?)\]\([^\n]+\)\s*$",
  re.IGNORECASE | re.MULTILINE,
)

# Values are repository-relative document names plus stable explicit anchors.
# Detailed guides win at section level; broader groups fall back to the settings
# overview so every current parameter can show useful context.
_DOC_BY_SECTION_ID = {
  "BUTTON_MODE": "buttons-presets#button-modes",
  "BUTTON_UNIT": "buttons-presets#speed-units",
  "BUTTON_TEST": "buttons-presets#button-spam",
  "BASIC_SPEEDTABLE": "buttons-presets#speed-presets",
  "SPEED_CAMERA": "speed-deceleration#speed-camera",
  "SPEED_LIMIT": "speed-deceleration#road-speed-limit",
  "SPEED_BUMP": "speed-deceleration#speed-bump",
  "SPEED_CURVE": "speed-deceleration#curve-turn",
  "SPEED_TRAFFIC": "speed-deceleration#traffic-light",
  "CRUISE_ACCEL__CRUISE_DRIVE_MODE": "cruise-gap#driving-mode",
  "CRUISE_ACCEL__CRUISE_ACCEL_TABLE": "cruise-gap#acceleration-table",
  "CRUISE_STOPGO": "cruise-gap#stop-resume",
  "CRUISE_LONGTUNE": "cruise-gap#longitudinal-tuning",
  "CRUISE_GAP": "cruise-gap#following-gap",
  "CRUISE_LEAD": "cruise-gap#lead-response",
  "CRUISE_CARROT": "cruise-gap#carrot-cruise",
}

_DOC_BY_GROUP_ID = {
  "START_AUTO": "settings#start-auto",
  "STEER": "settings#vehicle-steering",
  "VEH_HYUNDAI": "settings#vehicle-hardware",
  "VEH_CANFD": "settings#vehicle-hardware",
  "VEH_RADAR": "radar#front-radar",
  "VEH_DM": "settings#vehicle-hardware",
  "VEH_AUX": "settings#vehicle-hardware",
  "DISP_SCREEN": "settings#display",
  "DISP_PATH": "settings#display",
  "DISP_BRIGHT": "settings#display",
  "DISP_HUD": "settings#display",
  "SYS_RECORD": "settings#system",
  "SYS_NET": "settings#system",
  "SYS_SOUND": "settings#system",
  "SYS_SW": "settings#system",
}

_DOC_BY_PARAMETER = {
  "EnableRadarTracks": "radar#front-radar",
  "EnableCornerRadar": "radar#corner-radar",
  "RadarLeadModelMode": "radar#lead-fusion",
}


def normalize_doc_language(value: str) -> str:
  language = str(value or "").strip().lower()
  return language if language in SUPPORTED_DOC_LANGUAGES else "en"


def _split_doc_reference(reference: str) -> tuple[str, str] | None:
  match = _DOC_REFERENCE_RE.fullmatch(str(reference or "").strip())
  if match is None:
    return None
  return match.group("source"), match.group("anchor")


def get_setting_doc_reference(name: str) -> str | None:
  target = str(name or "").strip()
  if not target:
    return None
  override = _DOC_BY_PARAMETER.get(target)
  if override:
    return override

  # Refreshes the mtime-backed menu cache when carrot_settings.json changed.
  get_settings_cached()
  categories = settings_cache.get("categories") or []
  for category in categories:
    for group in category.get("groups", []):
      group_reference = _DOC_BY_GROUP_ID.get(str(group.get("id") or ""))
      for section in group.get("sections", []):
        if target not in section.get("items", []):
          continue
        section_reference = _DOC_BY_SECTION_ID.get(str(section.get("id") or ""))
        return section_reference or group_reference
  return None


def extract_markdown_section(markdown: str, anchor: str) -> str | None:
  marker = f'<a id="{anchor}"></a>'
  marker_start = markdown.find(marker)
  if marker_start < 0:
    return None

  content_start = marker_start + len(marker)
  heading = _HEADING_RE.search(markdown, content_start)
  if heading is None or markdown[content_start:heading.start()].strip():
    return None

  level = len(heading.group("marks"))
  content_end = len(markdown)
  for candidate in _HEADING_RE.finditer(markdown, heading.end()):
    if len(candidate.group("marks")) <= level:
      content_end = candidate.start()
      break

  section = markdown[heading.start():content_end].strip()
  section = _SECTION_NAV_RE.sub("", section).strip()
  return section or None


def _resolved_doc_path(source: str, requested_language: str) -> tuple[Path, str] | None:
  language = normalize_doc_language(requested_language)
  requested_path = USER_DOCS_ROOT / language / f"{source}.md"
  if requested_path.is_file():
    return requested_path, language

  # Chinese detailed guides are optional for now.  The existing ctitle/cdescr
  # remain Chinese in the UI while the longer guide explicitly falls back to
  # English.  Any future docs/user/zh file is picked up automatically.
  if language == "zh":
    english_path = USER_DOCS_ROOT / "en" / f"{source}.md"
    if english_path.is_file():
      return english_path, "en"
  return None


def load_setting_doc(name: str, language: str) -> dict[str, Any] | None:
  reference = get_setting_doc_reference(name)
  split_reference = _split_doc_reference(reference or "")
  if split_reference is None:
    return None
  source, anchor = split_reference

  requested_language = normalize_doc_language(language)
  resolved = _resolved_doc_path(source, requested_language)
  if resolved is None:
    return None
  path, resolved_language = resolved

  markdown = path.read_text(encoding="utf-8")
  section = extract_markdown_section(markdown, anchor)
  if section is None:
    return None

  return {
    "name": str(name),
    "source": source,
    "anchor": anchor,
    "language_requested": requested_language,
    "language_resolved": resolved_language,
    "fallback": requested_language != resolved_language,
    "markdown": section,
  }
