"""
Device info service — gathers device metadata for the web Device tab.

Collects: device type, dongle ID, serial, calibration status,
supported languages, current language, software version, update state.
Adapts to TICI / MICI / TIZI via HARDWARE.get_device_type().
"""
from __future__ import annotations

import math
import subprocess
from typing import Any, Dict, List

from .params import HAS_PARAMS, Params, get_param_value


# ── device type ──────────────────────────────────────────────
def get_device_type() -> str:
  try:
    from openpilot.system.hardware import HARDWARE
    return HARDWARE.get_device_type()
  except Exception:
    return "unknown"


# ── calibration ──────────────────────────────────────────────
def get_calibration_status() -> Dict[str, Any]:
  """Parse CalibrationParams and return human-readable status."""
  if not HAS_PARAMS:
    return {"calibrated": False, "pitch": None, "yaw": None}

  params = Params()
  try:
    calib_bytes = params.get("CalibrationParams")
    if not calib_bytes:
      return {"calibrated": False, "pitch": None, "yaw": None}

    from cereal import messaging, log
    calib = messaging.log_from_bytes(calib_bytes, log.Event).liveCalibration

    uncal = 0  # cereal::LiveCalibrationData::Status::UNCALIBRATED
    try:
      uncal = log.LiveCalibrationData.Status.uncalibrated
    except Exception:
      pass

    if calib.calStatus == uncal:
      return {"calibrated": False, "pitch": None, "yaw": None}

    pitch = round(math.degrees(calib.rpyCalib[1]), 1)
    yaw = round(math.degrees(calib.rpyCalib[2]), 1)
    return {"calibrated": True, "pitch": pitch, "yaw": yaw}
  except Exception:
    return {"calibrated": False, "pitch": None, "yaw": None}


# ── supported languages ─────────────────────────────────────
# Matches the list from Qt getSupportedLanguages() / multilang
SUPPORTED_LANGUAGES: List[Dict[str, str]] = [
  {"code": "main_en", "name": "English"},
  {"code": "main_ko", "name": "한국어"},
  {"code": "main_zh-CHS", "name": "简体中文"},
  {"code": "main_zh-CHT", "name": "繁體中文"},
  {"code": "main_ja", "name": "日本語"},
  {"code": "main_fr", "name": "Français"},
  {"code": "main_pt-BR", "name": "Português"},
  {"code": "main_de", "name": "Deutsch"},
  {"code": "main_es", "name": "Español"},
  {"code": "main_tr", "name": "Türkçe"},
  {"code": "main_th", "name": "ไทย"},
  {"code": "main_ar", "name": "العربية"},
  {"code": "main_pl", "name": "Polski"},
  {"code": "main_nl", "name": "Nederlands"},
]


# ── software / update state ─────────────────────────────────
def get_update_status() -> Dict[str, Any]:
  if not HAS_PARAMS:
    return {}
  return {
    "version": get_param_value("UpdaterCurrentDescription", ""),
    "state": get_param_value("UpdaterState", "idle"),
    "available": get_param_value("UpdateAvailable", False),
    "fetch_available": get_param_value("UpdaterFetchAvailable", False),
    "failed_count": get_param_value("UpdateFailedCount", 0),
    "target_branch": get_param_value("UpdaterTargetBranch", ""),
    "git_branch": get_param_value("GitBranch", ""),
    "available_branches": get_param_value("UpdaterAvailableBranches", ""),
    "last_update_time": get_param_value("LastUpdateTime", ""),
    "new_description": get_param_value("UpdaterNewDescription", ""),
  }


# ── network viewer data ────────────────────────────────────
def _split_nmcli_line(line: str) -> List[str]:
  parts: List[str] = []
  buf: List[str] = []
  escaped = False
  for ch in line:
    if escaped:
      buf.append(ch)
      escaped = False
    elif ch == "\\":
      escaped = True
    elif ch == ":":
      parts.append("".join(buf))
      buf = []
    else:
      buf.append(ch)
  parts.append("".join(buf))
  return parts


def get_wifi_networks() -> List[Dict[str, Any]]:
  """Read visible Wi-Fi networks without connecting, forgetting, or editing."""
  try:
    proc = subprocess.run(
      ["nmcli", "-t", "-f", "ACTIVE,SSID,SECURITY,SIGNAL", "dev", "wifi", "list", "--rescan", "no"],
      check=False,
      capture_output=True,
      encoding="utf-8",
      errors="replace",
      timeout=3,
    )
  except Exception:
    return []

  if proc.returncode != 0:
    return []

  seen: Dict[str, Dict[str, Any]] = {}
  for raw in proc.stdout.splitlines():
    parts = _split_nmcli_line(raw)
    if len(parts) < 4:
      continue
    active, ssid, security, signal = parts[:4]
    ssid = ssid.strip()
    if not ssid:
      continue
    try:
      signal_value = int(signal)
    except Exception:
      signal_value = None
    entry = {
      "ssid": ssid,
      "connected": active.strip().lower() == "yes",
      "security": security.strip(),
      "signal": signal_value,
      "secure": bool(security.strip() and security.strip() != "--"),
    }
    prev = seen.get(ssid)
    if prev is None or (entry["connected"] and not prev.get("connected")) or ((entry["signal"] or 0) > (prev.get("signal") or 0)):
      seen[ssid] = entry

  return sorted(seen.values(), key=lambda n: (not n.get("connected"), -(n.get("signal") or -1), n.get("ssid") or ""))


def get_wifi_ip_address() -> str:
  try:
    proc = subprocess.run(
      ["nmcli", "-t", "-f", "IP4.ADDRESS", "dev", "show", "wlan0"],
      check=False,
      capture_output=True,
      encoding="utf-8",
      errors="replace",
      timeout=2,
    )
  except Exception:
    return ""

  if proc.returncode != 0:
    return ""

  for raw in proc.stdout.splitlines():
    if ":" not in raw:
      continue
    value = raw.split(":", 1)[1].strip()
    if value:
      return value.split("/", 1)[0]
  return ""


# ── aggregate ────────────────────────────────────────────────
def get_device_info() -> Dict[str, Any]:
  """Single call to gather all device info for the web Device tab."""
  device_type = get_device_type()
  is_mici = device_type == "mici"

  return {
    "device_type": device_type,
    "dongle_id": get_param_value("DongleId", "N/A"),
    "serial": get_param_value("HardwareSerial", "N/A"),
    "calibration": get_calibration_status(),
    "language": get_param_value("LanguageSetting", "main_en"),
    "languages": SUPPORTED_LANGUAGES,
    "is_metric": get_param_value("IsMetric", False),
    "software_menu": get_param_value("SoftwareMenu", 0),
    "update": get_update_status(),
    "network": {
      "wifi": get_wifi_networks(),
      "ip_address": get_wifi_ip_address(),
      "tethering_enabled": get_param_value("HotspotOnBoot", False),
      "roaming_enabled": get_param_value("GsmRoaming", False),
      "gsm_metered": get_param_value("GsmMetered", False),
      "apn": get_param_value("GsmApn", ""),
    },
    # feature flags for conditional rendering
    "has_cellular": not is_mici,
    "has_amplifier": not is_mici,
  }
