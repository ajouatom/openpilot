from __future__ import annotations

import asyncio
import time

from aiohttp import WSMsgType, web

from openpilot.cereal import car, messaging
from openpilot.common.params import Params
from openpilot.selfdrive.car.openpilot_toggle import CruiseMainOpenpilotToggle


AudibleAlert = car.CarControl.HUDControl.AudibleAlert
ButtonType = car.CarState.ButtonEvent.Type
SELFDRIVE_STATE_TIMEOUT = 5.0
# Poll cadence for the sound state loop. selfdriveState publishes at 100Hz and the
# device's own soundd only samples it at 20Hz (50ms), so polling here at ~5ms lets
# the browser detect an alert change *before* the device and absorb the WS + audio
# output latency, landing near-simultaneous instead of ~50ms late. We only send on
# change, so a faster poll shortens detection latency without adding traffic.
SOUND_POLL_INTERVAL = 0.005


def _enum_raw(value, default: int = 0) -> int:
  try:
    return int(getattr(value, "raw", value))
  except (TypeError, ValueError):
    return default


def _message_valid(sm, service: str) -> bool:
  try:
    return bool(sm.valid[service])
  except Exception:
    return False


def _param_percent(params: Params, key: str) -> float:
  try:
    return max(0.0, min(2.0, float(params.get_int(key)) / 100.0))
  except Exception:
    return 1.0


def _param_string(value) -> str:
  if isinstance(value, (bytes, bytearray, memoryview)):
    return bytes(value).decode("utf-8", errors="replace")
  return str(value or "")


def _sound_directory(params: Params) -> str:
  try:
    sound_language = _param_string(params.get("SoundLanguageSetting", return_default=True)).strip()
    if not sound_language or sound_language.lower() == "auto":
      sound_language = _param_string(params.get("LanguageSetting", return_default=True)).strip() or "en"
  except Exception:
    sound_language = "en"
  normalized = sound_language.replace("_", "-").lower()
  if normalized.startswith("main-"):
    normalized = normalized[5:]
  if normalized == "ko" or normalized.startswith("ko-"):
    return "sounds"
  if normalized in ("zh-chs", "zh-hans") or normalized.startswith("zh"):
    return "sounds_chs"
  return "sounds_eng"


def _is_tizi() -> bool:
  try:
    from openpilot.system.hardware import HARDWARE
    return HARDWARE.get_device_type() == "tizi"
  except Exception:
    return False


async def _send_sound_states(ws: web.WebSocketResponse) -> None:
  sm = messaging.SubMaster(["selfdriveState", "carrotMan", "carState"])
  params = Params()
  cruise_main_toggle = CruiseMainOpenpilotToggle(ButtonType.mainCruise)
  last_signature = None
  prompt_sequence = 0
  sequence = 0
  next_param_read = 0.0
  volume = 1.0
  engage_volume = 1.0
  sound_directory = "sounds_eng"
  emitted_countdown = 100
  tizi = _is_tizi()

  while not ws.closed:
    sm.update(0)
    now = time.monotonic()

    enabled = bool(sm["selfdriveState"].enabled) if _message_valid(sm, "selfdriveState") else False
    alert = _enum_raw(sm["selfdriveState"].alertSound) if _message_valid(sm, "selfdriveState") else 0

    try:
      missing_for = now - float(sm.recv_time["selfdriveState"])
    except Exception:
      missing_for = 0.0
    if missing_for > SELFDRIVE_STATE_TIMEOUT:
      if enabled and missing_for < SELFDRIVE_STATE_TIMEOUT + 10.0:
        alert = _enum_raw(AudibleAlert.warningImmediate)
      else:
        alert = _enum_raw(AudibleAlert.none)

    countdown = 100
    if _message_valid(sm, "carrotMan"):
      try:
        countdown = int(sm["carrotMan"].leftSec)
      except (TypeError, ValueError):
        countdown = 100
    if last_signature is None or sm.updated["selfdriveState"] or sm.updated["carrotMan"]:
      emitted_countdown = countdown

    button_events = sm["carState"].buttonEvents if _message_valid(sm, "carState") else ()
    if cruise_main_toggle.update(button_events, enabled, now=now):
      prompt_sequence += 1
      alert = _enum_raw(AudibleAlert.prompt)

    if now >= next_param_read:
      volume = _param_percent(params, "SoundVolumeAdjust")
      engage_volume = _param_percent(params, "SoundVolumeAdjustEngage")
      sound_directory = _sound_directory(params)
      next_param_read = now + 1.0

    signature = (alert, emitted_countdown, prompt_sequence, volume, engage_volume, sound_directory, tizi)
    if signature != last_signature:
      sequence += 1
      await ws.send_json({
        "type": "soundState",
        "sequence": sequence,
        "alert": alert,
        "countdown": emitted_countdown,
        "promptSequence": prompt_sequence,
        "volume": volume,
        "engageVolume": engage_volume,
        "soundDirectory": sound_directory,
        "tizi": tizi,
      })
      last_signature = signature

    await asyncio.sleep(SOUND_POLL_INTERVAL)


async def ws_web_sound(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=20, max_msg_size=64 * 1024, compress=False)
  await ws.prepare(request)
  sender = asyncio.create_task(_send_sound_states(ws))
  try:
    async for msg in ws:
      if msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.ERROR):
        break
  finally:
    sender.cancel()
    try:
      await sender
    except asyncio.CancelledError:
      pass
    except Exception:
      pass
  return ws


def register(app: web.Application) -> None:
  app.router.add_get("/ws/web_sound", ws_web_sound)
