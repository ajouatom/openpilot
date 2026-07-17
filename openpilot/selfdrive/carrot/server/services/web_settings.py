import json
import os
from typing import Any, Callable, Dict, List, Optional

from ..config import CARROT_WEB_SETTINGS_PATH


WEB_PRIMARY_PAGES = {"last", "carrot", "setting", "tools", "logs", "terminal"}
WEB_LANGUAGES = {"", "en", "ko", "zh"}
LOG_UPLOAD_TARGETS = {"carrot", "toss"}
WEB_REPLAY_INSIGHTS_TABS = {"events", "graphs", "sensors", "advanced"}
WEB_DRIVE_LAYOUT_MODES = {"split", "area_1", "area_2"}
WEB_DRIVE_LAYOUT_CONTENTS = {"vision", "navigation"}


def _to_bool(value: Any) -> bool:
  if isinstance(value, str):
    return value.strip().lower() in {"1", "true", "yes", "on"}
  return bool(value)


def _normalize_language(value: Any) -> str:
  lang = str(value or "").strip().lower()
  aliases = {
    "main_ko": "ko",
    "main_en": "en",
    "main_zh-chs": "zh",
    "main_zh-cht": "zh",
  }
  lang = aliases.get(lang, lang)
  if lang.startswith("ko"):
    return "ko"
  if lang.startswith("zh"):
    return "zh"
  if lang.startswith("en"):
    return "en"
  return lang if lang in WEB_LANGUAGES else ""


def _normalize_kmap_url(value: Any) -> str:
  url = str(value or "").strip()
  return url or "https://jominki354.github.io/kmap/"


def _normalize_toss_url(value: Any) -> str:
  url = str(value or "").strip().rstrip("/")
  if url and not url.lower().startswith(("http://", "https://")):
    url = f"https://{url}"
  return url


def _normalize_stripped(value: Any) -> str:
  return str(value or "").strip()
def _normalize_drive_split_ratio(value: Any, fallback: float) -> str:
  try:
    ratio = float(value)
  except (TypeError, ValueError):
    ratio = fallback
  ratio = min(0.7, max(0.3, ratio))
  ratio = round(ratio / 0.05) * 0.05
  return f"{ratio:.2f}"


def _normalize_carrot_navi_split_ratio(value: Any) -> str:
  return _normalize_drive_split_ratio(value, 0.7)


def _normalize_carrot_navi_vertical_split_ratio(value: Any) -> str:
  return _normalize_drive_split_ratio(value, 0.5)


class _Field:
  """One web setting. The spec is the single source of truth for the key's
  type, default and validation — defaults, sanitize and the client-facing
  schema are all derived from it, so adding a setting means one entry here."""

  __slots__ = ("key", "type", "default", "choices", "normalize")

  def __init__(
    self,
    key: str,
    type: str,
    default: Any,
    choices: Optional[set] = None,
    normalize: Optional[Callable[[Any], Any]] = None,
  ) -> None:
    self.key = key
    self.type = type
    self.default = default
    self.choices = choices
    self.normalize = normalize

  def coerce(self, value: Any) -> Any:
    if self.type == "bool":
      return _to_bool(value)
    if self.type == "enum":
      candidate = str(value if value is not None else "").strip().lower()
      return candidate if candidate in (self.choices or set()) else self.default
    # "str" — a custom normalize callable owns validation/fallback.
    if self.normalize is not None:
      return self.normalize(value)
    return str(value if value is not None else "")


# Declarative schema. Order here is the canonical key order.
WEB_SETTINGS_SPEC: List[_Field] = [
  _Field("auto_update_git_pull", "bool", False),
  _Field("start_page", "enum", "last", choices=WEB_PRIMARY_PAGES),
  _Field("mini_hud_enabled", "bool", False),
  _Field("web_language", "str", "", normalize=_normalize_language),
  _Field("vision_fullscreen_default", "bool", False),
  _Field("replay_insights_tab", "enum", "events", choices=WEB_REPLAY_INSIGHTS_TABS),
  _Field("carrot_navi_enabled", "bool", True),
  _Field("carrot_navi_horizontal_mode", "enum", "split", choices=WEB_DRIVE_LAYOUT_MODES),
  _Field("carrot_navi_horizontal_area_1", "enum", "vision", choices=WEB_DRIVE_LAYOUT_CONTENTS),
  _Field("carrot_navi_horizontal_area_2", "enum", "navigation", choices=WEB_DRIVE_LAYOUT_CONTENTS),
  _Field("carrot_navi_split_ratio", "str", "0.70", normalize=_normalize_carrot_navi_split_ratio),
  _Field("carrot_navi_vertical_mode", "enum", "split", choices=WEB_DRIVE_LAYOUT_MODES),
  _Field("carrot_navi_vertical_area_1", "enum", "vision", choices=WEB_DRIVE_LAYOUT_CONTENTS),
  _Field("carrot_navi_vertical_area_2", "enum", "navigation", choices=WEB_DRIVE_LAYOUT_CONTENTS),
  _Field("carrot_navi_vertical_split_ratio", "str", "0.50", normalize=_normalize_carrot_navi_vertical_split_ratio),
  _Field("kmap_enabled", "bool", False),
  _Field("kmap_url", "str", "https://jominki354.github.io/kmap/", normalize=_normalize_kmap_url),
  _Field("kmap_overlay_heading_up", "bool", False),
  _Field("kmap_overlay_curvature_color", "bool", False),
  _Field("kmap_map_type", "enum", "roadmap", choices={"roadmap", "satellite", "hybrid"}),
  _Field("nav_hud_enabled", "bool", True),
  _Field("log_upload_target", "enum", "carrot", choices=LOG_UPLOAD_TARGETS),
  _Field("toss_upload_url", "str", "https://op.wjcloud.kr", normalize=_normalize_toss_url),
  _Field("toss_upload_token", "str", "", normalize=_normalize_stripped),
  # Remote support last-used settings, persisted so the owner's choices survive a
  # reload. Stored as strings via the enum type (numeric values are parsed back
  # to ints client-side). Command permission defaults to "approve_each" so a
  # fresh device always starts in the safest, confirm-each-command mode.
  _Field("support_permission_mode", "enum", "approve_each", choices={"approve_each", "allow_all"}),
  _Field("support_ttl_seconds", "enum", "1800", choices={"900", "1800", "3600"}),
  _Field("support_command_timeout_seconds", "enum", "30", choices={"15", "30", "60", "120"}),
]

_SPEC_BY_KEY: Dict[str, _Field] = {f.key: f for f in WEB_SETTINGS_SPEC}

# Derived from the spec so there is no second list of defaults to keep in sync.
DEFAULT_WEB_SETTINGS: Dict[str, Any] = {f.key: f.default for f in WEB_SETTINGS_SPEC}



def sanitize_web_settings(raw: Optional[Dict[str, Any]]) -> Dict[str, Any]:
  raw = raw or {}
  settings: Dict[str, Any] = {}
  for field in WEB_SETTINGS_SPEC:
    value = raw.get(field.key, field.default) if isinstance(raw, dict) else field.default
    settings[field.key] = field.coerce(value)
  for orientation in ("horizontal", "vertical"):
    area_1_key = f"carrot_navi_{orientation}_area_1"
    area_2_key = f"carrot_navi_{orientation}_area_2"
    if settings[area_1_key] == settings[area_2_key]:
      settings[area_2_key] = "navigation" if settings[area_1_key] == "vision" else "vision"
  return settings


def web_settings_client_spec() -> List[Dict[str, Any]]:
  """JSON-safe schema for the browser: type + default + (enum) choices per key.
  The frontend derives its defaults/normalization from this so it never has to
  re-declare them. `normalize` callables are intentionally omitted."""
  spec: List[Dict[str, Any]] = []
  for field in WEB_SETTINGS_SPEC:
    entry: Dict[str, Any] = {"key": field.key, "type": field.type, "default": field.default}
    if field.type == "enum" and field.choices:
      entry["choices"] = sorted(field.choices)
    spec.append(entry)
  return spec


def read_web_settings() -> Dict[str, Any]:
  try:
    with open(CARROT_WEB_SETTINGS_PATH, "r", encoding="utf-8") as f:
      raw = json.load(f)
  except Exception:
    return dict(DEFAULT_WEB_SETTINGS)
  return sanitize_web_settings(raw if isinstance(raw, dict) else {})


def write_web_settings(settings: Dict[str, Any]) -> Dict[str, Any]:
  clean = sanitize_web_settings(settings)
  os.makedirs(os.path.dirname(CARROT_WEB_SETTINGS_PATH), exist_ok=True)
  tmp_path = CARROT_WEB_SETTINGS_PATH + ".tmp"
  with open(tmp_path, "w", encoding="utf-8") as f:
    json.dump(clean, f, ensure_ascii=False, indent=2, sort_keys=True)
    f.write("\n")
  os.replace(tmp_path, CARROT_WEB_SETTINGS_PATH)
  return clean


def update_web_settings(updates: Dict[str, Any]) -> Dict[str, Any]:
  current = read_web_settings()
  allowed = {key: value for key, value in (updates or {}).items() if key in _SPEC_BY_KEY}
  current.update(allowed)
  return write_web_settings(current)
