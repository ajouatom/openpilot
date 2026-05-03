"""Device support services that are not plain Params reads."""
from __future__ import annotations

import math
import subprocess
import time
from typing import Any, Dict, List

from .params import HAS_PARAMS, Params, get_param_value


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

NETWORK_CACHE_TTL_SEC = 5.0
_network_cache: Dict[str, Any] = {
  "monotonic": 0.0,
  "data": None,
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
      ["nmcli", "-t", "-f", "ACTIVE,SSID,SECURITY,SIGNAL", "dev", "wifi", "list", "--rescan", "auto"],
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


def get_device_network(force: bool = False) -> Dict[str, Any]:
  now = time.monotonic()
  cached = _network_cache.get("data")
  if not force and isinstance(cached, dict) and now - float(_network_cache.get("monotonic") or 0.0) < NETWORK_CACHE_TTL_SEC:
    return dict(cached)

  data = {
    "wifi": get_wifi_networks(),
    "ip_address": get_wifi_ip_address(),
    "tethering_enabled": get_param_value("HotspotOnBoot", False),
    "roaming_enabled": get_param_value("GsmRoaming", False),
    "gsm_metered": get_param_value("GsmMetered", False),
    "apn": get_param_value("GsmApn", ""),
  }
  _network_cache["monotonic"] = now
  _network_cache["data"] = dict(data)
  return data
