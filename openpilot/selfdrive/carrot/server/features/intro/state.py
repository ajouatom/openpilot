"""Carrot intro state — decides whether the intro should be shown.

The decision is made HERE, on the device. There is no comma/carrot server API
that can answer "is this a fresh install" (the only outbound call is the
fire-and-forget heartbeat in services/heartbeat.py). Deciding locally also
works offline, which is the normal state of a freshly flashed device.

Flag file: CARROT_STATE_DIR/intro.json  (/data/carrot/state/intro.json)
That directory is OUTSIDE the git working tree on purpose — the reset/sync
tools run `git clean -xfd`, which would wipe an in-repo flag and make the
intro reappear after every reset. See config.CARROT_DATA_DIR.

Scope: per DEVICE, not per browser. Swapping phones must not re-trigger it.
A factory reset / reflash wipes /data and the intro returns — that is the
correct meaning of "first install".


── Why the "already onboarded" guard is the load-bearing part ──

"No intro.json" is also true for every existing device that pulls this
update. Without a guard they would all see the intro, and the wizard writes
vehicle control params — it would overwrite a tuned setup. So the risk is
asymmetric:

  hiding it from a new user   -> they configure in settings instead. Minor.
  showing it to an existing user -> their tuning is overwritten. Serious.

When in doubt, hide.


── How params actually behave (this drives the whole design) ──

system/manager/manager.py:78 writes every unset key's default to disk on
EVERY manager start:

    for k in params.all_keys():
      default_value = params.get_default_value(k)
      if default_value is not None and params.get(k) is None:
        params.put(k, default_value)

Consequences that broke the obvious approaches:

  · "param file exists" proves nothing — after one boot they all exist.
  · "params.get(k) is None" is only true for keys WITHOUT a default
    (CarName, CalibrationParams, DongleId, LiveTorqueParameters).
  · CompletedTrainingVersion defaults to "0.2.0", so it is ALWAYS set.
    Testing it for truthiness matches every device and would hide the intro
    from everyone, forever. (This module had exactly that bug.)
  · Comparing against carrot_settings.json defaults is wrong too — they
    disagree with params_keys.h in at least one place (SpeedFromPCM is 2 in
    params_keys.h, 0 in carrot_settings.json).

What DOES work: compare against `params.get_default_value(k)`, the same value
manager wrote. 159 of the 163 carrot settings have one. Any difference means
a human changed it.
"""
from __future__ import annotations

import json
import os
import time
from typing import Any, Dict

from ...config import (
  CARROT_SETTING_FAVORITES_PATH,
  CARROT_SETTING_PROFILES_PATH,
  CARROT_STATE_DIR,
  CARROT_WEB_SETTINGS_PATH,
  CARROT_YOUTUBE_LIVE_STATE_PATH,
)
from ...services.params import HAS_PARAMS, Params
from ...services.settings import get_settings_cached

CARROT_INTRO_STATE_PATH = os.path.join(CARROT_STATE_DIR, "intro.json")

STATE_VERSION = 1

REASON_FINISHED = "user_finished"
REASON_RESTORED = "restored_file"
REASON_SKIPPED = "skipped"
REASON_EXISTING = "existing_install"   # written by the upgrade guard, not the user

# params_keys.h default for CarSelected3. A device that has booted always has
# this value until the user picks a real car.
CAR_SELECTED_UNSET = "MOCK"


def _read_raw() -> Dict[str, Any]:
  try:
    with open(CARROT_INTRO_STATE_PATH, "r", encoding="utf-8") as f:
      raw = json.load(f)
    return raw if isinstance(raw, dict) else {}
  except Exception:
    return {}


def _write_raw(data: Dict[str, Any]) -> None:
  """Atomic write, same pattern as services/web_settings.py."""
  os.makedirs(CARROT_STATE_DIR, exist_ok=True)
  tmp_path = CARROT_INTRO_STATE_PATH + ".tmp"
  with open(tmp_path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2, sort_keys=True)
    f.write("\n")
  os.replace(tmp_path, CARROT_INTRO_STATE_PATH)


def _text(value: Any) -> str:
  if isinstance(value, (bytes, bytearray, memoryview)):
    value = bytes(value).decode("utf-8", errors="replace")
  return str(value if value is not None else "").strip()


def _car_chosen(params: "Params") -> bool:
  """Has a real car been picked in CarrotPilot?

  CarSelected3 is CarrotPilot's own param and defaults to "MOCK", so anything
  else means the user went through car selection. The web already treats
  blank / "-" / anything containing "mock" as unset
  (web/js/pages/setting.js isMissingCarSelectionLabel) — same rule here."""
  text = _text(params.get("CarSelected3"))
  if not text or text == "-":
    return False
  return "mock" not in text.lower()


def _car_fingerprinted(params: "Params") -> bool:
  """CarName has no default, so it stays unset until the car is fingerprinted
  on a real drive. Present = this device has driven a real car."""
  text = _text(params.get("CarName"))
  return bool(text) and "mock" not in text.lower()


def _settings_touched(params: "Params") -> str:
  """Any CarrotPilot setting moved off the value manager wrote at boot.

  This is the signal that catches an existing user who tuned from the device
  UI and never opened the web — exactly the person whose setup the wizard
  would trash. Returns the first differing param name, or "".

  159/163 settings carry a params_keys.h default; the rest are skipped
  because there is nothing trustworthy to compare against."""
  try:
    _, _, by_name, _ = get_settings_cached()
  except Exception:
    return ""

  for name in by_name.keys():
    if not name:
      continue
    try:
      default = params.get_default_value(name)
      if default is None:
        continue                      # no default -> nothing to compare
      if _text(params.get(name)) != _text(default):
        return name
    except Exception:
      continue                        # unknown key on this build — skip it
  return ""


def _web_state_files() -> str:
  """Files that only exist once the CarrotPilot web app has been used."""
  for path, label in (
    (CARROT_WEB_SETTINGS_PATH, "web_settings_exists"),
    (CARROT_SETTING_PROFILES_PATH, "setting_profiles_exists"),
    (CARROT_SETTING_FAVORITES_PATH, "setting_favorites_exists"),
    (CARROT_YOUTUBE_LIVE_STATE_PATH, "youtube_live_exists"),
  ):
    if os.path.isfile(path):
      return label
  return ""


def _looks_already_onboarded() -> str:
  """Evidence this device was in use before the intro existed.
  Returns a short reason, or "" when it looks like a fresh install.

  Every signal here is CarrotPilot-specific on purpose. Generic openpilot
  state (CalibrationParams, CompletedTrainingVersion, DongleId) is NOT used:
  it survives a switch from another fork, and it is also set by a brand new
  CarrotPilot user who simply drove before opening the web — both would be
  wrongly treated as existing users."""
  hit = _web_state_files()
  if hit:
    return hit

  if not HAS_PARAMS or Params is None:
    return ""

  try:
    params = Params()
  except Exception:
    return ""

  if _car_chosen(params):
    return "car_already_selected"

  touched = _settings_touched(params)
  if touched:
    return f"setting_changed:{touched}"

  if _car_fingerprinted(params):
    return "car_fingerprinted"

  return ""


def read_intro_state() -> Dict[str, Any]:
  raw = _read_raw()
  return {
    "version": int(raw.get("version") or STATE_VERSION),
    "completed": bool(raw.get("completed")),
    "completedAt": raw.get("completedAt") or 0,
    "reason": str(raw.get("reason") or ""),
  }


def mark_completed(reason: str = REASON_FINISHED) -> Dict[str, Any]:
  data = {
    "version": STATE_VERSION,
    "completed": True,
    "completedAt": int(time.time()),
    "reason": str(reason or REASON_FINISHED)[:64],
  }
  try:
    _write_raw(data)
  except Exception:
    # A failed write means the intro shows again next time. Annoying, but the
    # caller must not be blocked from entering the app over it.
    pass
  return data


def intro_bootstrap() -> Dict[str, Any]:
  """What static.py injects into window.__CARROT_BOOTSTRAP__.intro.

  Shipping this in the bootstrap (instead of a separate fetch) lets the client
  gate on the very first render, so the home page never flashes before the
  intro takes over.

  Side effect: the first time this runs on a device that was already in use,
  it records completion so the scan never runs again."""
  state = read_intro_state()
  if state["completed"]:
    return {"shouldShow": False, "reason": state["reason"] or "already_completed"}

  existing = _looks_already_onboarded()
  if existing:
    mark_completed(REASON_EXISTING)
    return {"shouldShow": False, "reason": existing}

  return {"shouldShow": True, "reason": "fresh_install"}
