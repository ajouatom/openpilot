import copy
import json
import logging
import os
import re
import threading
from typing import Any, Callable, Dict, List, Optional, Tuple

from openpilot.selfdrive.carrot.web_upload import DEFAULT_WEB_UPLOAD_URL, normalize_base_url

from ..config import CARROT_WEB_SETTINGS_PATH, WEB_DIR
from .web_capabilities import is_known_web_capability


WEB_PRIMARY_PAGES = {"last", "carrot", "setting", "tools", "logs", "terminal"}
WEB_LANGUAGES = {"", "en", "ko", "zh"}
WEB_REPLAY_INSIGHTS_TABS = {"events", "graphs", "sensors", "advanced"}
WEB_DRIVE_LAYOUT_MODES = {"split", "area_1", "area_2"}

# Existing settings files created before the drive-layout keys were persisted
# should retain the former split layout. Fresh installs use WEB_SETTINGS_SPEC.
LEGACY_DRIVE_LAYOUT_DEFAULTS = {
  "carrot_navi_horizontal_mode": "split",
  "carrot_navi_horizontal_area_1": "vision",
  "carrot_navi_horizontal_area_2": "navigation",
  "carrot_navi_vertical_mode": "split",
  "carrot_navi_vertical_area_1": "vision",
  "carrot_navi_vertical_area_2": "navigation",
}

DRIVE_CONTENT_CATALOG_PATH = os.path.join(
  WEB_DIR,
  "src",
  "features",
  "drive",
  "core",
  "content_catalog.json",
)
DRIVE_CONTENT_CATALOG_SCHEMA_VERSION = 1
DRIVE_CONTENT_CATALOG_SLOTS = {"primary", "secondary"}
DRIVE_CONTENT_CATALOG_SOURCES = {"live", "replay"}
NO_SUPPORTED_CONTENT = "NO_SUPPORTED_CONTENT"

_DRIVE_CONTENT_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
_DRIVE_LAYOUT_CONTENT_KEYS = {
  "carrot_navi_horizontal_area_1",
  "carrot_navi_horizontal_area_2",
  "carrot_navi_vertical_area_1",
  "carrot_navi_vertical_area_2",
}
_SAFE_DRIVE_CONTENT_CATALOG: Dict[str, Any] = {
  "schemaVersion": DRIVE_CONTENT_CATALOG_SCHEMA_VERSION,
  "defaults": {"primary": "vision", "secondary": "navigation"},
  "contents": [
    {
      "id": "vision",
      "labelKey": "web_settings_carrot_vision",
      "supportedSlots": ["primary", "secondary"],
      "supportedSources": ["live", "replay"],
      "singleton": True,
    },
    {
      "id": "navigation",
      "labelKey": "web_settings_navigation",
      "supportedSlots": ["primary", "secondary"],
      "supportedSources": ["live", "replay"],
      "singleton": True,
    },
  ],
}

_catalog_cache: Dict[str, Tuple[Optional[Tuple[int, int]], Dict[str, Any]]] = {}
_catalog_cache_lock = threading.RLock()
_logger = logging.getLogger(__name__)


class NoSupportedContentError(ValueError):
  code = NO_SUPPORTED_CONTENT

  def __init__(self, slot: str, source: str) -> None:
    self.slot = slot
    self.source = source
    super().__init__(f"{NO_SUPPORTED_CONTENT}: slot={slot} source={source}")


def _require_exact_keys(value: Dict[str, Any], expected: set, label: str) -> None:
  actual = set(value)
  if actual != expected:
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    raise ValueError(f"{label} keys mismatch: missing={missing} unknown={unknown}")


def _validate_enum_list(value: Any, allowed: set, label: str) -> List[str]:
  if not isinstance(value, list) or not value:
    raise ValueError(f"{label} must be a non-empty list")
  if any(not isinstance(item, str) or item not in allowed for item in value):
    raise ValueError(f"{label} contains an unsupported value")
  if len(value) != len(set(value)):
    raise ValueError(f"{label} contains duplicate values")
  return list(value)


def validate_drive_content_catalog(raw: Any) -> Dict[str, Any]:
  if not isinstance(raw, dict):
    raise ValueError("catalog must be an object")
  _require_exact_keys(raw, {"schemaVersion", "defaults", "contents"}, "catalog")
  if type(raw["schemaVersion"]) is not int or raw["schemaVersion"] != DRIVE_CONTENT_CATALOG_SCHEMA_VERSION:
    raise ValueError("catalog schemaVersion must be 1")

  defaults = raw["defaults"]
  if not isinstance(defaults, dict):
    raise ValueError("catalog defaults must be an object")
  _require_exact_keys(defaults, {"primary", "secondary"}, "catalog defaults")

  contents = raw["contents"]
  if not isinstance(contents, list) or not contents:
    raise ValueError("catalog contents must be a non-empty list")

  normalized_contents: List[Dict[str, Any]] = []
  seen_ids = set()
  descriptor_keys = {"id", "labelKey", "supportedSlots", "supportedSources", "singleton"}
  for index, descriptor in enumerate(contents):
    label = f"catalog contents[{index}]"
    if not isinstance(descriptor, dict):
      raise ValueError(f"{label} must be an object")
    _require_exact_keys(descriptor, descriptor_keys, label)

    content_id = descriptor["id"]
    if not isinstance(content_id, str) or not _DRIVE_CONTENT_ID_PATTERN.fullmatch(content_id):
      raise ValueError(f"{label}.id is invalid")
    if content_id in seen_ids:
      raise ValueError(f"duplicate catalog content id: {content_id}")
    seen_ids.add(content_id)

    label_key = descriptor["labelKey"]
    if not isinstance(label_key, str) or not label_key.strip():
      raise ValueError(f"{label}.labelKey must be a non-empty string")
    if type(descriptor["singleton"]) is not bool:
      raise ValueError(f"{label}.singleton must be a boolean")

    normalized_contents.append({
      "id": content_id,
      "labelKey": label_key,
      "supportedSlots": _validate_enum_list(
        descriptor["supportedSlots"],
        DRIVE_CONTENT_CATALOG_SLOTS,
        f"{label}.supportedSlots",
      ),
      "supportedSources": _validate_enum_list(
        descriptor["supportedSources"],
        DRIVE_CONTENT_CATALOG_SOURCES,
        f"{label}.supportedSources",
      ),
      "singleton": descriptor["singleton"],
    })

  normalized_defaults: Dict[str, str] = {}
  for slot in ("primary", "secondary"):
    default_id = defaults[slot]
    if not isinstance(default_id, str) or default_id not in seen_ids:
      raise ValueError(f"catalog defaults.{slot} must reference a content id")
    normalized_defaults[slot] = default_id

  return {
    "schemaVersion": DRIVE_CONTENT_CATALOG_SCHEMA_VERSION,
    "defaults": normalized_defaults,
    "contents": normalized_contents,
  }


def _catalog_signature(path: str) -> Tuple[int, int]:
  stat = os.stat(path)
  return stat.st_mtime_ns, stat.st_size


def _clear_drive_content_catalog_cache(path: Optional[os.PathLike] = None) -> None:
  with _catalog_cache_lock:
    if path is None:
      _catalog_cache.clear()
    else:
      _catalog_cache.pop(os.path.abspath(os.fspath(path)), None)


def load_drive_content_catalog(path: Optional[os.PathLike] = None) -> Dict[str, Any]:
  catalog_path = os.path.abspath(os.fspath(path or DRIVE_CONTENT_CATALOG_PATH))
  for attempt in range(2):
    signature: Optional[Tuple[int, int]]
    stat_error: Optional[Exception] = None
    try:
      signature = _catalog_signature(catalog_path)
    except OSError as exc:
      signature = None
      stat_error = exc

    with _catalog_cache_lock:
      cached = _catalog_cache.get(catalog_path)
      if cached is not None and cached[0] == signature:
        return copy.deepcopy(cached[1])

    raw: Any = None
    read_error: Optional[Exception] = stat_error
    if read_error is None:
      try:
        with open(catalog_path, "r", encoding="utf-8") as f:
          raw = json.load(f)
      except Exception as exc:
        read_error = exc

      try:
        after_signature: Optional[Tuple[int, int]] = _catalog_signature(catalog_path)
      except OSError as exc:
        after_signature = None
        if read_error is None:
          read_error = exc
      if after_signature != signature:
        if attempt == 0:
          continue
        error = RuntimeError(
          f"catalog changed while reading: before={signature} after={after_signature}"
        )
        _logger.warning("Using safe drive content catalog for %s: %s", catalog_path, error)
        return copy.deepcopy(_SAFE_DRIVE_CONTENT_CATALOG)

    if read_error is None:
      try:
        catalog = validate_drive_content_catalog(raw)
      except Exception as exc:
        read_error = exc
    if read_error is not None:
      _logger.warning("Using safe drive content catalog for %s: %s", catalog_path, read_error)
      catalog = copy.deepcopy(_SAFE_DRIVE_CONTENT_CATALOG)

    with _catalog_cache_lock:
      _catalog_cache[catalog_path] = (signature, catalog)
    return copy.deepcopy(catalog)

  return copy.deepcopy(_SAFE_DRIVE_CONTENT_CATALOG)


def _normalize_content_id(value: Any) -> str:
  return str(value if value is not None else "").strip().lower()


def _select_drive_content(
  requested_id: str,
  fallback_id: str,
  slot: str,
  source: str,
  contents: List[Dict[str, Any]],
  selected_ids: List[str],
) -> str:
  by_id = {descriptor["id"]: descriptor for descriptor in contents}

  def supported(descriptor: Optional[Dict[str, Any]]) -> bool:
    if descriptor is None:
      return False
    if slot not in descriptor["supportedSlots"] or source not in descriptor["supportedSources"]:
      return False
    return not (descriptor["singleton"] and descriptor["id"] in selected_ids)

  for content_id in (requested_id, fallback_id):
    descriptor = by_id.get(content_id)
    if supported(descriptor):
      return descriptor["id"]
  for descriptor in contents:
    if supported(descriptor):
      return descriptor["id"]
  raise NoSupportedContentError(slot, source)


def normalize_drive_layout_contents(
  area_1: Any,
  area_2: Any,
  catalog: Optional[Dict[str, Any]] = None,
  source: str = "live",
) -> Tuple[str, str]:
  resolved_catalog = validate_drive_content_catalog(catalog) if catalog is not None else load_drive_content_catalog()
  contents = resolved_catalog["contents"]
  selected_ids: List[str] = []
  normalized_area_1 = _select_drive_content(
    _normalize_content_id(area_1),
    "vision",
    "primary",
    source,
    contents,
    selected_ids,
  )
  selected_ids.append(normalized_area_1)
  normalized_area_2 = _select_drive_content(
    _normalize_content_id(area_2),
    "navigation",
    "secondary",
    source,
    contents,
    selected_ids,
  )
  return normalized_area_1, normalized_area_2


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


def _normalize_web_upload_url(value: Any) -> str:
  try:
    url = normalize_base_url(value, DEFAULT_WEB_UPLOAD_URL)
    # Migrate defaults used before the upload receiver moved to its dedicated
    # HTTPS virtual host. The main shind0 host serves Carrot Web and returns an
    # HTML 404 for upload API paths.
    if url.casefold() in {"https://op.wjcloud.kr", "https://shind0.synology.me"}:
      return DEFAULT_WEB_UPLOAD_URL
    return url
  except ValueError:
    return DEFAULT_WEB_UPLOAD_URL


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
  """One web setting. Type/default and ordinary validation live here. Drive
  content choices and pair normalization are derived from the source catalog."""

  __slots__ = ("key", "type", "default", "choices", "normalize", "requires_capability")

  def __init__(
    self,
    key: str,
    type: str,
    default: Any,
    choices: Optional[set] = None,
    normalize: Optional[Callable[[Any], Any]] = None,
    requires_capability: Optional[str] = None,
  ) -> None:
    if requires_capability is not None and not is_known_web_capability(requires_capability):
      raise ValueError(f"unknown web capability for {key}: {requires_capability}")
    self.key = key
    self.type = type
    self.default = default
    self.choices = choices
    self.normalize = normalize
    self.requires_capability = requires_capability

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
  _Field("web_lab_enabled", "bool", False),
  _Field("vision_fullscreen_default", "bool", False),
  _Field("carrot_navi_fullscreen_on_tap", "bool", False),
  _Field("vision_ar_enabled", "bool", False, requires_capability="web_lab"),
  _Field("vision_ar_debug", "bool", False, requires_capability="web_lab"),
  _Field("vision_display_mode", "enum", "normal", choices={"fit", "normal", "crop"}),
  _Field("replay_hud_visible", "bool", False),
  _Field("replay_insights_tab", "enum", "events", choices=WEB_REPLAY_INSIGHTS_TABS),
  _Field("carrot_navi_horizontal_mode", "enum", "area_1", choices=WEB_DRIVE_LAYOUT_MODES),
  _Field("carrot_navi_horizontal_area_1", "enum", "navigation"),
  _Field("carrot_navi_horizontal_area_2", "enum", "vision"),
  _Field("carrot_navi_split_ratio", "str", "0.70", normalize=_normalize_carrot_navi_split_ratio),
  _Field("carrot_navi_vertical_mode", "enum", "area_1", choices=WEB_DRIVE_LAYOUT_MODES),
  _Field("carrot_navi_vertical_area_1", "enum", "navigation"),
  _Field("carrot_navi_vertical_area_2", "enum", "vision"),
  _Field("carrot_navi_vertical_split_ratio", "str", "0.50", normalize=_normalize_carrot_navi_vertical_split_ratio),
  _Field("kmap_enabled", "bool", False),
  _Field("kmap_url", "str", "https://jominki354.github.io/kmap/", normalize=_normalize_kmap_url),
  _Field("kmap_overlay_heading_up", "bool", False),
  _Field("kmap_overlay_curvature_color", "bool", False),
  _Field("kmap_map_type", "enum", "roadmap", choices={"roadmap", "satellite", "hybrid"}),
  _Field("web_upload_url", "str", DEFAULT_WEB_UPLOAD_URL, normalize=_normalize_web_upload_url),
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
  legacy_aliases = {
    "web_upload_url": "toss_upload_url",
  }
  settings: Dict[str, Any] = {}
  for field in WEB_SETTINGS_SPEC:
    if isinstance(raw, dict) and field.key in raw:
      value = raw[field.key]
    elif isinstance(raw, dict) and legacy_aliases.get(field.key) in raw:
      value = raw[legacy_aliases[field.key]]
    else:
      value = field.default
    settings[field.key] = _normalize_content_id(value) if field.key in _DRIVE_LAYOUT_CONTENT_KEYS else field.coerce(value)
  catalog = load_drive_content_catalog()
  for orientation in ("horizontal", "vertical"):
    area_1_key = f"carrot_navi_{orientation}_area_1"
    area_2_key = f"carrot_navi_{orientation}_area_2"
    settings[area_1_key], settings[area_2_key] = normalize_drive_layout_contents(
      settings[area_1_key],
      settings[area_2_key],
      catalog=catalog,
    )
  return settings


def web_settings_client_spec() -> List[Dict[str, Any]]:
  """JSON-safe schema for the browser: type + default + (enum) choices per key.
  The frontend derives its defaults/normalization from this so it never has to
  re-declare them. `normalize` callables are intentionally omitted."""
  spec: List[Dict[str, Any]] = []
  catalog_choices = [descriptor["id"] for descriptor in load_drive_content_catalog()["contents"]]
  for field in WEB_SETTINGS_SPEC:
    entry: Dict[str, Any] = {"key": field.key, "type": field.type, "default": field.default}
    if field.requires_capability:
      entry["requiresCapability"] = field.requires_capability
    if field.key in _DRIVE_LAYOUT_CONTENT_KEYS:
      entry["choices"] = list(catalog_choices)
    elif field.type == "enum" and field.choices:
      entry["choices"] = sorted(field.choices)
    spec.append(entry)
  return spec


def web_setting_defaults_for_capability(capability_id: str) -> Dict[str, Any]:
  return {
    field.key: copy.deepcopy(field.default)
    for field in WEB_SETTINGS_SPEC
    if field.requires_capability == capability_id
  }


def read_web_settings() -> Dict[str, Any]:
  try:
    with open(CARROT_WEB_SETTINGS_PATH, "r", encoding="utf-8") as f:
      raw = json.load(f)
  except Exception:
    return dict(DEFAULT_WEB_SETTINGS)
  if not isinstance(raw, dict):
    return dict(DEFAULT_WEB_SETTINGS)
  for key, value in LEGACY_DRIVE_LAYOUT_DEFAULTS.items():
    raw.setdefault(key, value)
  return sanitize_web_settings(raw)


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
