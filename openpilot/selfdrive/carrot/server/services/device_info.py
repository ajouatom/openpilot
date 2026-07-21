"""Device support services that are not plain Params reads."""
from __future__ import annotations

import math
import subprocess
import threading
import time
from copy import deepcopy
from typing import Any, Dict, List

from .params import HAS_PARAMS, Params, get_param_value, get_param_values
from .ssh_keys import get_ssh_key_status


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

    import openpilot.cereal.messaging as messaging
    from openpilot.cereal import log
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

# Single source for the Device tab's parameters.
#
# The group -> parameter mapping used to exist twice: here as a flat default
# map, and again in web/js/pages/setting_device_config.js as four arrays. The
# two never disagreed, but nothing stopped them from doing so silently. The
# groups below are shipped to the browser in the settings snapshot so the web
# reads its parameter names from this list instead of restating them.
#
# Order matters: it is the order the rows are rendered in.
DEVICE_SETTING_GROUPS: tuple[tuple[str, tuple[tuple[str, Any], ...]], ...] = (
  ("Device", (
    ("DeviceType", "unknown"),
    ("DongleId", ""),
    ("HardwareSerial", ""),
    ("LanguageSetting", "main_en"),
    ("SoftwareMenu", 1),
  )),
  ("Software", (
    ("UpdaterCurrentDescription", ""),
    ("UpdaterState", ""),
    ("UpdateAvailable", False),
    ("UpdaterFetchAvailable", False),
    ("UpdateFailedCount", 0),
    ("UpdaterTargetBranch", ""),
    ("GitBranch", ""),
    ("UpdaterAvailableBranches", ""),
    ("LastUpdateTime", ""),
    ("UpdaterNewDescription", ""),
  )),
  ("Toggles", (
    ("OpenpilotEnabledToggle", False),
    ("ExperimentalMode", False),
    ("ExperimentalModeConfirmed", False),
    ("DisengageOnAccelerator", False),
    ("IsLdwEnabled", False),
    ("AlwaysOnDM", False),
    ("RecordFront", False),
    ("RecordAudio", False),
    ("IsMetric", False),
    ("LongitudinalPersonality", 1),
  )),
  ("Developer", (
    ("AdbEnabled", False),
    ("SshEnabled", False),
    ("JoystickDebugMode", False),
    ("LongitudinalManeuverMode", False),
    ("AlphaLongitudinalEnabled", False),
    ("GithubUsername", ""),
    ("GithubSshKeys", ""),
  )),
)

DEVICE_SETTING_DEFAULTS: Dict[str, Any] = {
  name: default
  for _group, entries in DEVICE_SETTING_GROUPS
  for name, default in entries
}


def get_device_setting_group_names() -> Dict[str, list]:
  """Group -> parameter names, for the browser to read its groups from."""
  return {group: [name for name, _default in entries] for group, entries in DEVICE_SETTING_GROUPS}

DEVICE_NETWORK_REFRESH_INTERVAL_SEC = 15.0
_network_cache_lock = threading.Lock()
_network_cache: Dict[str, Any] = {
  "monotonic": 0.0,
  "data": None,
}


def get_device_setting_values(ssh_status: Dict[str, Any] | None = None) -> Dict[str, Any]:
  """Read every value needed to render the Device settings groups."""
  names = [
    name for name in DEVICE_SETTING_DEFAULTS
    if name not in ("DeviceType", "GithubUsername", "GithubSshKeys")
  ]
  values = get_param_values(names, DEVICE_SETTING_DEFAULTS)
  try:
    from openpilot.system.hardware import HARDWARE
    values["DeviceType"] = HARDWARE.get_device_type()
  except Exception:
    values["DeviceType"] = DEVICE_SETTING_DEFAULTS["DeviceType"]

  status = ssh_status if isinstance(ssh_status, dict) else get_ssh_key_status()
  values["GithubUsername"] = status.get("username", values.get("GithubUsername", ""))
  # The UI only needs presence, so the public snapshot never carries key bodies.
  values["GithubSshKeys"] = "1" if status.get("has_keys") else ""
  return values


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


def _network_param_values() -> Dict[str, Any]:
  return {
    "tethering_enabled": get_param_value("HotspotOnBoot", False),
    "roaming_enabled": get_param_value("GsmRoaming", False),
    "gsm_metered": get_param_value("GsmMetered", False),
    "apn": get_param_value("GsmApn", ""),
  }


def get_device_network_snapshot() -> Dict[str, Any]:
  """Return immediately without running NetworkManager commands."""
  with _network_cache_lock:
    cached = _network_cache.get("data")
    snapshot = deepcopy(cached) if isinstance(cached, dict) else None
  if snapshot is not None:
    snapshot.update(_network_param_values())
    return snapshot
  return {
    "wifi": [],
    "ip_address": "",
    **_network_param_values(),
  }


def refresh_device_network() -> Dict[str, Any]:
  """Refresh the slow NetworkManager data for future request snapshots."""
  data = {
    "wifi": get_wifi_networks(),
    "ip_address": get_wifi_ip_address(),
    **_network_param_values(),
  }
  with _network_cache_lock:
    _network_cache["monotonic"] = time.monotonic()
    _network_cache["data"] = deepcopy(data)
  return deepcopy(data)


def get_device_network(force: bool = False) -> Dict[str, Any]:
  """Compatibility entry point: only an explicit force may perform I/O."""
  return refresh_device_network() if force else get_device_network_snapshot()
