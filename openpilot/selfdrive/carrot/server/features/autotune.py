"""AutoTuner server feature: Extracts driving telemetry and parameter domain knowledge for AI-driven autotuning."""
from __future__ import annotations

import asyncio
import json
import os
import time
from typing import Any

from aiohttp import web

from openpilot.cereal import log as capnp_log
from ..config import DASHCAM_ROOT
from ..services.param_changes import append_param_change
from ..services.params import HAS_PARAMS, Params, clamp_numeric, get_param_values, set_param_value
from ..services.settings import get_settings_cached
from .dashcam.catalog import build_routes
from .dashcam.paths import route_name, segment_index
from .dashcam.report import _decompress, _pick_log_file

DOWN_SAMPLE_HZ = 5.0  # 5Hz downsampled telemetry for prompt payload
MAX_TELEMETRY_POINTS = 300  # Max points across sampled segments for fast LLM response
AI_CONFIG_PARAM_KEY = "CarrotAiConfig"


def _extract_telemetry_from_log(log_path: str) -> list[dict[str, Any]]:
  """Parses a log file and extracts downsampled driving state frames."""
  dat = _decompress(log_path)
  events = list(capnp_log.Event.read_multiple_bytes(dat))

  frames: list[dict[str, Any]] = []
  curr_cs = None
  curr_rs = None
  curr_lp = None
  curr_cc = None
  last_sample_t = -1.0
  sample_interval = 1.0 / DOWN_SAMPLE_HZ

  for evt in events:
    try:
      which = evt.which()
      if which == "carState":
        curr_cs = evt.carState
      elif which == "radarState":
        curr_rs = evt.radarState
      elif which == "longitudinalPlan":
        curr_lp = evt.longitudinalPlan
      elif which == "carControl":
        curr_cc = evt.carControl

      t_now = evt.logMonoTime * 1e-9
      if last_sample_t < 0 or (t_now - last_sample_t >= sample_interval):
        if curr_cs is not None:
          v_ego = float(getattr(curr_cs, "vEgo", 0.0)) * 3.6  # km/h
          a_ego = float(getattr(curr_cs, "aEgo", 0.0))  # m/s^2
          gas_pressed = bool(getattr(curr_cs, "gasPressed", False))
          brake_pressed = bool(getattr(curr_cs, "brakePressed", False))
          steering_pressed = bool(getattr(curr_cs, "steeringPressed", False))
          standstill = bool(getattr(curr_cs, "standstill", False))
          steer_angle = float(getattr(curr_cs, "steeringAngleDeg", 0.0))

          # Target plan
          a_target = 0.0
          if curr_lp is not None:
            if hasattr(curr_lp, "aTarget"):
              a_target = float(curr_lp.aTarget)
            elif hasattr(curr_lp, "accels") and len(curr_lp.accels) > 0:
              a_target = float(curr_lp.accels[0])

          # Actuator command
          a_cmd = 0.0
          steer_torque = 0.0
          desired_steer_angle = 0.0
          if curr_cc is not None:
            actuators = getattr(curr_cc, "actuators", None)
            if actuators is not None:
              a_cmd = float(getattr(actuators, "accel", 0.0))
              steer_torque = float(getattr(actuators, "steer", getattr(actuators, "torque", 0.0)))
              desired_steer_angle = float(getattr(actuators, "steeringAngleDeg", getattr(actuators, "steerAngleDeg", 0.0)))

          # Lead radar information
          has_lead = False
          d_rel = 0.0
          v_lead = 0.0
          a_lead = 0.0
          j_lead = 0.0
          if curr_rs is not None:
            lead_one = getattr(curr_rs, "leadOne", None)
            if lead_one and getattr(lead_one, "status", False):
              has_lead = True
              d_rel = float(getattr(lead_one, "dRel", 0.0))
              v_lead = float(getattr(lead_one, "vLead", 0.0)) * 3.6  # km/h
              a_lead = float(getattr(lead_one, "aLeadK", getattr(lead_one, "aLead", 0.0)))  # m/s^2
              j_lead = float(getattr(lead_one, "jLead", 0.0))  # m/s^3

          frames.append({
            "t": round(t_now, 2),
            "vEgo": round(v_ego, 1),
            "aEgo": round(a_ego, 2),
            "aTarget": round(a_target, 2),
            "aCmd": round(a_cmd, 2),
            "gas": gas_pressed,
            "brake": brake_pressed,
            "steer": steering_pressed,
            "standstill": standstill,
            "hasLead": has_lead,
            "dRel": round(d_rel, 1),
            "vLead": round(v_lead, 1),
            "aLead": round(a_lead, 2),
            "jLead": round(j_lead, 2),
            "steerAngle": round(steer_angle, 1),
            "desiredSteerAngle": round(desired_steer_angle, 1),
            "steerTorque": round(steer_torque, 2),
          })
          last_sample_t = t_now
    except Exception:
      pass

  return frames


def _detect_key_episodes(frames: list[dict[str, Any]], include_driver_override: bool = False) -> list[dict[str, Any]]:
  """Detects notable events such as harsh catch-up acceleration, harsh braking, steering tracking errors, and driver overrides."""
  episodes: list[dict[str, Any]] = []

  for i in range(1, len(frames)):
    f_prev = frames[i - 1]
    f = frames[i]

    # Episode 1: Departure start (standstill -> moving with lead)
    if f_prev["standstill"] and not f["standstill"] and f["hasLead"]:
      episodes.append({
        "type": "departure_start",
        "t": f["t"],
        "vEgo": f["vEgo"],
        "dRel": f["dRel"],
        "vLead": f["vLead"],
        "aTarget": f["aTarget"],
        "aCmd": f["aCmd"],
        "isOverride": False,
        "description": f"정차 후 출발: vEgo={f['vEgo']}km/h, dRel={f['dRel']}m, aTarget={f['aTarget']}m/s^2",
      })

    # Episode 2: Harsh catch-up acceleration spike (aTarget > 1.4 m/s^2 when following lead)
    if f["hasLead"] and f["aTarget"] >= 1.4 and not f["gas"]:
      episodes.append({
        "type": "harsh_catchup_accel",
        "t": f["t"],
        "vEgo": f["vEgo"],
        "dRel": f["dRel"],
        "aTarget": f["aTarget"],
        "aCmd": f["aCmd"],
        "jLead": f["jLead"],
        "isOverride": False,
        "description": f"선행차 추종 급가속 피크: aTarget={f['aTarget']}m/s^2, aCmd={f['aCmd']}m/s^2, jLead={f['jLead']}",
      })

    # Episode 3: Harsh deceleration (aTarget < -1.8 m/s^2 or aEgo < -2.0 m/s^2)
    if (f["aTarget"] <= -1.8 or f["aEgo"] <= -2.0) and not f["brake"]:
      episodes.append({
        "type": "harsh_decel",
        "t": f["t"],
        "vEgo": f["vEgo"],
        "dRel": f["dRel"] if f["hasLead"] else 0,
        "aEgo": f["aEgo"],
        "aTarget": f["aTarget"],
        "isOverride": False,
        "description": f"급감속/급제동 발생: aTarget={f['aTarget']}m/s^2, aEgo={f['aEgo']}m/s^2",
      })

    # Episode 4: Steering tracking error (|desired - actual| > 3.0 deg at speed > 30 km/h)
    steer_err = abs(f["desiredSteerAngle"] - f["steerAngle"])
    if f["vEgo"] >= 30.0 and steer_err >= 3.0 and not f["steer"]:
      episodes.append({
        "type": "steering_tracking_error",
        "t": f["t"],
        "vEgo": f["vEgo"],
        "steerAngle": f["steerAngle"],
        "desiredSteerAngle": f["desiredSteerAngle"],
        "steerTorque": f["steerTorque"],
        "isOverride": False,
        "description": f"조향 추종 오차({steer_err:.1f}deg): 목표={f['desiredSteerAngle']}deg, 실제={f['steerAngle']}deg, 토크={f['steerTorque']}",
      })

    # Optional: Driver Manual Override Episodes (Driver Discomfort Feedback)
    if include_driver_override:
      # Driver Brake Override (e.g. driver intervened because automated decel was too late or accel was aggressive)
      if f["brake"] and not f_prev["brake"] and f["vEgo"] >= 5.0:
        episodes.append({
          "type": "driver_brake_override",
          "t": f["t"],
          "vEgo": f["vEgo"],
          "dRel": f["dRel"] if f["hasLead"] else 0,
          "aTarget": f_prev["aTarget"],
          "aEgo": f["aEgo"],
          "isOverride": True,
          "overrideType": "brake",
          "description": f"운전자 브레이크 수동개입 (자동 가속/감속 불만): vEgo={f['vEgo']}km/h, 직전 aTarget={f_prev['aTarget']}m/s^2, dRel={f['dRel']}m",
        })

      # Driver Gas Override (e.g. driver intervened because departure/follow acceleration was sluggish)
      if f["gas"] and not f_prev["gas"] and f["vEgo"] <= 80.0:
        episodes.append({
          "type": "driver_gas_override",
          "t": f["t"],
          "vEgo": f["vEgo"],
          "dRel": f["dRel"] if f["hasLead"] else 0,
          "vLead": f["vLead"] if f["hasLead"] else 0,
          "aTarget": f_prev["aTarget"],
          "isOverride": True,
          "overrideType": "gas",
          "description": f"운전자 가속페달 수동개입 (출발 굼뜸/거리 벌어짐 불만): vEgo={f['vEgo']}km/h, 직전 aTarget={f_prev['aTarget']}m/s^2, dRel={f['dRel']}m",
        })

      # Driver Steer Override (e.g. driver manually corrected steering due to lane drift or curve deviation)
      if f["steer"] and not f_prev["steer"] and f["vEgo"] >= 20.0:
        episodes.append({
          "type": "driver_steer_override",
          "t": f["t"],
          "vEgo": f["vEgo"],
          "steerAngle": f["steerAngle"],
          "desiredSteerAngle": f["desiredSteerAngle"],
          "isOverride": True,
          "overrideType": "steer",
          "description": f"운전자 조향 핸들 수동개입 (차선 쏠림/곡률 추종 보정): vEgo={f['vEgo']}km/h, 오차={abs(f['desiredSteerAngle'] - f['steerAngle']):.1f}deg",
        })

  # Limit max episodes to avoid blowing up prompt token size
  if len(episodes) > 30:
    step = len(episodes) // 30
    episodes = episodes[::step]

  return episodes


def _extract_all_frames_sync(folders: list[str]) -> list[dict[str, Any]]:
  """Helper to extract frames across folder paths synchronously in a thread."""
  all_frames: list[dict[str, Any]] = []
  for seg_folder in folders:
    full_seg_path = os.path.join(DASHCAM_ROOT, seg_folder) if not os.path.isabs(seg_folder) else seg_folder
    if not os.path.isdir(full_seg_path):
      continue

    picked = _pick_log_file(full_seg_path, prefer_rlog=False)
    if not picked:
      continue
    path, _ = picked
    try:
      frames = _extract_telemetry_from_log(path)
      all_frames.extend(frames)
    except Exception:
      pass
  return all_frames


async def api_autotune_extract_telemetry(request: web.Request) -> web.Response:
  """Extracts compact telemetry frames and episodes from selected segments of a route."""
  route_filter = ""
  segments_filter: list[str] = []
  include_driver_override = False

  if request.method == "POST":
    try:
      body = await request.json()
      route_filter = body.get("route", "")
      segments_filter = body.get("segments", [])
      include_driver_override = bool(body.get("includeDriverOverride", body.get("include_driver_override", False)))
    except Exception:
      pass
  else:
    route_filter = request.query.get("route", "")
    seg_param = request.query.get("segments", "")
    if seg_param:
      segments_filter = [s.strip() for s in seg_param.split(",") if s.strip()]
    include_param = request.query.get("includeDriverOverride", request.query.get("include_driver_override", "0"))
    include_driver_override = include_param in ("1", "true", "True")

  if not route_filter:
    return web.json_response({"ok": False, "error": "route parameter is required"}, status=400)

  routes = build_routes()
  target_route = None
  for r in routes:
    r_name = r.get("route", "")
    r_title = r.get("title", "")
    if r_name == route_filter or r_title == route_filter or route_filter.endswith(r_title) or route_filter.endswith(r_name):
      target_route = r
      break

  if not target_route:
    return web.json_response({"ok": False, "error": f"route {route_filter} not found"}, status=404)

  segment_folders = target_route.get("segmentFolders", [])
  if segments_filter:
    selected_folders = [s for s in segment_folders if s in segments_filter or os.path.basename(s) in segments_filter]
  else:
    selected_folders = segment_folders

  if not selected_folders:
    selected_folders = segment_folders

  # Sample at most 4 representative segments to keep extraction lightning-fast (< 1s)
  if len(selected_folders) > 4:
    step = len(selected_folders) / 4.0
    sampled_folders = [selected_folders[int(i * step)] for i in range(4)]
  else:
    sampled_folders = selected_folders

  # Run extraction in worker thread so event loop is never blocked
  all_frames = await asyncio.to_thread(_extract_all_frames_sync, sampled_folders)

  # Subsample all_frames if too large for LLM context
  total_frames = len(all_frames)
  if total_frames > MAX_TELEMETRY_POINTS:
    step = total_frames // MAX_TELEMETRY_POINTS
    sample_frames = all_frames[::step]
  else:
    sample_frames = all_frames

  episodes = _detect_key_episodes(all_frames, include_driver_override=include_driver_override)

  # Count driver overrides in raw frames
  gas_overrides = sum(1 for i in range(1, len(all_frames)) if all_frames[i]["gas"] and not all_frames[i-1]["gas"])
  brake_overrides = sum(1 for i in range(1, len(all_frames)) if all_frames[i]["brake"] and not all_frames[i-1]["brake"] and all_frames[i]["vEgo"] >= 5.0)
  steer_overrides = sum(1 for i in range(1, len(all_frames)) if all_frames[i]["steer"] and not all_frames[i-1]["steer"] and all_frames[i]["vEgo"] >= 20.0)

  # Fetch current parameters
  current_params: dict[str, Any] = {}
  try:
    _, _, by_name, _ = get_settings_cached()
    param_keys = list(by_name.keys())
  except Exception:
    param_keys = [
      "CruiseMaxVals0", "CruiseMaxVals1", "CruiseMaxVals2", "CruiseMaxVals3",
      "CruiseMaxVals4", "CruiseMaxVals5", "CruiseMaxVals6",
      "DynamicTFollow", "DynamicTFollowLC", "EnableSpeedTF", "TFollowGap1",
      "TFollowGap2", "TFollowGap3", "TFollowGap4", "TFollowDecelBoost",
      "AChangeCostStarting", "RadarReactionFactor", "LongitudinalPersonality",
      "LongTuningKpV", "LongTuningKiV", "LongTuningKf", "StoppingAccel",
      "StopDistanceCarrot", "CarSelected3", "LateralTorqueCustom",
      "LateralTorqueKpV", "LateralTorqueKiV", "LateralTorqueKf", "LateralTorqueFriction",
      "LateralTorqueAccelFactor", "SteerRatioRate", "SteerActuatorDelay"
    ]
  current_params = get_param_values(param_keys, {})

  # Compute overall stats
  speeds = [f["vEgo"] for f in all_frames]
  accels = [f["aEgo"] for f in all_frames]
  targets = [f["aTarget"] for f in all_frames]

  stats = {
    "totalFrames": total_frames,
    "durationSec": round(total_frames / DOWN_SAMPLE_HZ, 1),
    "maxSpeedKph": max(speeds) if speeds else 0.0,
    "maxAccel": max(accels) if accels else 0.0,
    "minAccel": min(accels) if accels else 0.0,
    "maxTargetAccel": max(targets) if targets else 0.0,
    "episodeCount": len(episodes),
    "driverOverrides": {
      "gas": gas_overrides,
      "brake": brake_overrides,
      "steer": steer_overrides,
      "total": gas_overrides + brake_overrides + steer_overrides,
      "included": include_driver_override,
    },
  }

  return web.json_response({
    "ok": True,
    "route": route_filter,
    "segmentCount": len(selected_folders),
    "stats": stats,
    "episodes": episodes,
    "telemetrySample": sample_frames,
    "currentParams": current_params,
  })


async def api_autotune_apply_params(request: web.Request) -> web.Response:
  """Applies a batch of recommended parameters with rollback backup creation."""
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  params_to_set = body.get("params")
  if not isinstance(params_to_set, dict) or not params_to_set:
    return web.json_response({"ok": False, "error": "missing params dict"}, status=400)

  applied: dict[str, Any] = {}
  errors: dict[str, str] = {}
  previous_snapshot: dict[str, Any] = {}

  try:
    _, _, by_name, _ = get_settings_cached()
  except Exception:
    by_name = {}

  # Read previous values for rollback snapshot
  previous_snapshot = get_param_values(list(params_to_set.keys()), {})

  # Apply new values
  for k, raw_v in params_to_set.items():
    try:
      p = by_name.get(k)
      val: Any = str(raw_v)
      if p is not None:
        try:
          if isinstance(p.get("min"), (int, float)) and isinstance(p.get("max"), (int, float)):
            fv = float(val)
            fv = clamp_numeric(fv, p)
            if isinstance(p.get("min"), int) and isinstance(p.get("max"), int) and isinstance(p.get("default"), int):
              val = int(round(fv))
            else:
              val = fv
        except Exception:
          pass

      set_param_value(k, val, p)
      applied[k] = val

      # Record param change in history
      prev_v = previous_snapshot.get(k)
      if str(prev_v) != str(val):
        try:
          append_param_change(
            k,
            prev_v,
            val,
            source="autotuner",
            engaged=False,
          )
        except Exception:
          pass
    except Exception as exc:
      errors[k] = str(exc)

  return web.json_response({
    "ok": len(errors) == 0,
    "applied": applied,
    "errors": errors,
    "error": "; ".join(f"{k}: {err}" for k, err in errors.items()) if errors else "",
    "rollbackSnapshot": previous_snapshot,
  })


async def api_autotune_get_config(request: web.Request) -> web.Response:
  """Retrieves persisted AI configuration from device params with file fallback."""
  cfg = {}
  try:
    if HAS_PARAMS:
      params = Params()
      val = params.get(AI_CONFIG_PARAM_KEY)
      if val:
        if isinstance(val, (bytes, bytearray)):
          val = val.decode("utf-8")
        cfg = json.loads(val)
  except Exception:
    pass

  if not cfg:
    try:
      file_path = f"/data/params/d/{AI_CONFIG_PARAM_KEY}"
      if os.path.isfile(file_path):
        with open(file_path, "r", encoding="utf-8") as f:
          cfg = json.loads(f.read().strip())
    except Exception:
      pass

  return web.json_response({"ok": True, "config": cfg})


async def api_autotune_set_config(request: web.Request) -> web.Response:
  """Persists AI configuration to device params and filesystem so it survives reboots and syncs across all devices."""
  try:
    body = await request.json()
    cfg = body.get("config", {})
    if isinstance(cfg, dict):
      serialized = json.dumps(cfg)
      if HAS_PARAMS:
        try:
          params = Params()
          params.put(AI_CONFIG_PARAM_KEY, serialized)
        except Exception:
          pass
      try:
        os.makedirs("/data/params/d", exist_ok=True)
        with open(f"/data/params/d/{AI_CONFIG_PARAM_KEY}", "w", encoding="utf-8") as f:
          f.write(serialized)
      except Exception:
        pass
    return web.json_response({"ok": True, "config": cfg})
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=400)


def register(app: web.Application) -> None:
  app.router.add_post("/api/autotune/extract-telemetry", api_autotune_extract_telemetry)
  app.router.add_get("/api/autotune/extract-telemetry", api_autotune_extract_telemetry)
  app.router.add_post("/api/autotune/apply-params", api_autotune_apply_params)
  app.router.add_get("/api/autotune/config", api_autotune_get_config)
  app.router.add_post("/api/autotune/config", api_autotune_set_config)
