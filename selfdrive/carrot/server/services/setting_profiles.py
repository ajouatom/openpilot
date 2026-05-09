from __future__ import annotations

import json
import os
import re
import subprocess
import uuid
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from ..config import CARROT_SETTING_PROFILES_PATH
from .params import (
  get_param_values,
  preview_param_restore_values,
  restore_param_values_validated,
)
from .settings import get_settings_cached


REPO_DIR = "/data/openpilot"
MAX_SETTING_PROFILES = 40
MAX_PROFILE_NAME_LEN = 40


def _now_iso() -> str:
  return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _git(args: List[str], timeout: float = 3.0) -> str:
  try:
    out = subprocess.check_output(
      ["git", *args],
      cwd=REPO_DIR,
      stderr=subprocess.DEVNULL,
      timeout=timeout,
    )
    return out.decode("utf-8", "replace").strip()
  except Exception:
    return ""


def _commit_url(remote: str, commit: str) -> str:
  remote = str(remote or "").strip()
  commit = str(commit or "").strip()
  if not remote or not commit:
    return ""

  match = re.match(r"^https?://github\.com/([^/]+)/([^/#?]+?)(?:\.git)?/?$", remote)
  if not match:
    match = re.match(r"^git@github\.com:([^/]+)/(.+?)(?:\.git)?$", remote)
  if not match:
    return ""

  owner, repo = match.groups()
  return f"https://github.com/{owner}/{repo}/commit/{commit}"


def read_git_profile_meta() -> Dict[str, Any]:
  branch = _git(["branch", "--show-current"])
  commit = _git(["rev-parse", "HEAD"])
  commit_date = _git(["show", "-s", "--format=%cI", "HEAD"])
  remote = _git(["config", "--get", "remote.origin.url"])
  return {
    "branch": branch,
    "commit": commit,
    "commit_short": commit[:7] if commit else "",
    "commit_date": commit_date,
    "remote": remote,
    "commit_url": _commit_url(remote, commit),
  }


def _clean_name(value: Any) -> str:
  name = str(value or "").strip()
  name = re.sub(r"\s+", " ", name)
  return name[:MAX_PROFILE_NAME_LEN]


def _clean_values(values: Any) -> Dict[str, Any]:
  if not isinstance(values, dict):
    return {}
  _, _, by_name, _ = get_settings_cached()
  allowed = set(by_name.keys())
  return {
    str(key): value
    for key, value in values.items()
    if str(key) in allowed
  }


def _setting_defaults() -> Dict[str, Any]:
  _, _, by_name, _ = get_settings_cached()
  return {
    name: meta.get("default", 0)
    for name, meta in by_name.items()
  }


def snapshot_current_setting_values() -> Dict[str, Any]:
  defaults = _setting_defaults()
  return get_param_values(list(defaults.keys()), defaults)


def _sanitize_profile(raw: Any) -> Optional[Dict[str, Any]]:
  if not isinstance(raw, dict):
    return None
  profile_id = str(raw.get("id") or "").strip()
  name = _clean_name(raw.get("name"))
  values = _clean_values(raw.get("values"))
  if not profile_id or not name or not values:
    return None

  created_at = str(raw.get("created_at") or "").strip()
  updated_at = str(raw.get("updated_at") or created_at).strip()
  meta = raw.get("meta") if isinstance(raw.get("meta"), dict) else {}
  return {
    "id": profile_id,
    "name": name,
    "created_at": created_at,
    "updated_at": updated_at,
    "meta": {
      "branch": str(meta.get("branch") or ""),
      "commit": str(meta.get("commit") or ""),
      "commit_short": str(meta.get("commit_short") or ""),
      "commit_date": str(meta.get("commit_date") or ""),
      "remote": str(meta.get("remote") or ""),
      "commit_url": str(meta.get("commit_url") or ""),
    },
    "values": values,
  }


def read_setting_profiles() -> Dict[str, Any]:
  try:
    with open(CARROT_SETTING_PROFILES_PATH, "r", encoding="utf-8") as f:
      raw = json.load(f)
  except Exception:
    raw = {}

  profiles = raw.get("profiles") if isinstance(raw, dict) else []
  clean = []
  seen = set()
  for item in profiles if isinstance(profiles, list) else []:
    profile = _sanitize_profile(item)
    if not profile or profile["id"] in seen:
      continue
    seen.add(profile["id"])
    clean.append(profile)
    if len(clean) >= MAX_SETTING_PROFILES:
      break
  return {"profiles": clean}


def write_setting_profiles(data: Dict[str, Any]) -> Dict[str, Any]:
  clean = {"profiles": []}
  seen = set()
  for item in data.get("profiles", []) if isinstance(data, dict) else []:
    profile = _sanitize_profile(item)
    if not profile or profile["id"] in seen:
      continue
    seen.add(profile["id"])
    clean["profiles"].append(profile)
    if len(clean["profiles"]) >= MAX_SETTING_PROFILES:
      break

  os.makedirs(os.path.dirname(CARROT_SETTING_PROFILES_PATH), exist_ok=True)
  tmp_path = CARROT_SETTING_PROFILES_PATH + ".tmp"
  with open(tmp_path, "w", encoding="utf-8") as f:
    json.dump(clean, f, ensure_ascii=False, indent=2, sort_keys=True)
    f.write("\n")
  os.replace(tmp_path, CARROT_SETTING_PROFILES_PATH)
  return clean


def get_setting_profile(profile_id: str) -> Optional[Dict[str, Any]]:
  profile_id = str(profile_id or "").strip()
  for profile in read_setting_profiles()["profiles"]:
    if profile["id"] == profile_id:
      return profile
  return None


def create_setting_profile(name: str) -> Dict[str, Any]:
  clean_name = _clean_name(name)
  if not clean_name:
    raise ValueError("missing profile name")

  data = read_setting_profiles()
  now = _now_iso()
  profile = {
    "id": uuid.uuid4().hex,
    "name": clean_name,
    "created_at": now,
    "updated_at": now,
    "meta": read_git_profile_meta(),
    "values": snapshot_current_setting_values(),
  }
  data["profiles"].append(profile)
  write_setting_profiles(data)
  return profile


def update_setting_profile(profile_id: str, updates: Dict[str, Any]) -> Dict[str, Any]:
  data = read_setting_profiles()
  for profile in data["profiles"]:
    if profile["id"] != profile_id:
      continue
    if "name" in updates:
      name = _clean_name(updates.get("name"))
      if not name:
        raise ValueError("missing profile name")
      profile["name"] = name
    if "values" in updates:
      profile["values"] = _clean_values(updates.get("values"))
    profile["updated_at"] = _now_iso()
    write_setting_profiles(data)
    return profile
  raise KeyError("profile not found")


def delete_setting_profile(profile_id: str) -> None:
  data = read_setting_profiles()
  next_profiles = [profile for profile in data["profiles"] if profile["id"] != profile_id]
  if len(next_profiles) == len(data["profiles"]):
    raise KeyError("profile not found")
  data["profiles"] = next_profiles
  write_setting_profiles(data)


def preview_setting_profile(profile_id: str, values: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
  profile = get_setting_profile(profile_id)
  if profile is None:
    raise KeyError("profile not found")
  restore_values = _clean_values(values) if values is not None else profile["values"]
  return preview_param_restore_values(restore_values)


def apply_setting_profile(profile_id: str, values: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
  profile = get_setting_profile(profile_id)
  if profile is None:
    raise KeyError("profile not found")
  restore_values = _clean_values(values) if values is not None else profile["values"]
  return restore_param_values_validated(restore_values)
