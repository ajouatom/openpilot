import json
import os
from typing import Any, Dict, Optional

from ..config import CARROT_WEB_SETTINGS_PATH


WEB_PRIMARY_PAGES = {"last", "carrot", "setting", "tools", "logs", "terminal"}
WEB_LANGUAGES = {"", "en", "ko", "zh", "ja", "fr"}

DEFAULT_WEB_SETTINGS: Dict[str, Any] = {
  "auto_update_git_pull": False,
  "start_page": "last",
  "web_language": "",
}


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
    "main_ja": "ja",
    "main_fr": "fr",
  }
  lang = aliases.get(lang, lang)
  if lang.startswith("ko"):
    return "ko"
  if lang.startswith("zh"):
    return "zh"
  if lang.startswith("ja"):
    return "ja"
  if lang.startswith("fr"):
    return "fr"
  if lang.startswith("en"):
    return "en"
  return lang if lang in WEB_LANGUAGES else ""


def sanitize_web_settings(raw: Optional[Dict[str, Any]]) -> Dict[str, Any]:
  raw = raw or {}
  settings = dict(DEFAULT_WEB_SETTINGS)

  if "auto_update_git_pull" in raw:
    settings["auto_update_git_pull"] = _to_bool(raw.get("auto_update_git_pull"))

  start_page = str(raw.get("start_page", settings["start_page"]) or "").strip().lower()
  settings["start_page"] = start_page if start_page in WEB_PRIMARY_PAGES else "last"

  settings["web_language"] = _normalize_language(raw.get("web_language", settings["web_language"]))
  return settings


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
  allowed = {key: value for key, value in (updates or {}).items() if key in DEFAULT_WEB_SETTINGS}
  current.update(allowed)
  return write_web_settings(current)
