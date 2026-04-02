from __future__ import annotations

import time
from typing import Any

from .contract import DEFAULT_CAMERA_KIND, PAYLOAD_KIND_SERVICE_RAW, SCHEMA_VERSION
from .normalize import safe_bool, safe_float, safe_get, safe_int, safe_text, to_list
from .services import common_services, optional_services


# LIVE PAYLOAD SAFETY RULES:
# - Missing/non-alive services are normal during startup/stale windows.
# - Broker must not raise when a service is absent.
# - /ws/live is now a thin "service-raw semantic relay":
#   meta/runtime stay stable, service payloads stay close to the original
#   service field names so clients can assemble/render locally.
# - Missing services are represented as None under services[name].
#
def empty_live_payload(*, repo_flavor: str, selected_camera: str = DEFAULT_CAMERA_KIND) -> dict:
  now_ms = int(time.time() * 1000)
  return {
    "meta": {
      "schemaVersion": SCHEMA_VERSION,
      "repoFlavor": repo_flavor,
      "selectedCamera": selected_camera,
      "generatedAtMs": now_ms,
      "payloadKind": PAYLOAD_KIND_SERVICE_RAW,
    },
    "runtime": {
      "generatedAtMs": now_ms,
      "snapshotFresh": False,
      "serviceAlive": {},
      "coreServicesAlive": {},
      "optionalServicesAlive": {},
      "activeCoreServices": 0,
      "activeOptionalServices": 0,
      "missingServices": [],
      "missingCoreServices": [],
      "missingOptionalServices": [],
      "params": {},
    },
    "services": {},
  }


def build_live_payload(
  *,
  sm: Any,
  repo_flavor: str,
  selected_camera: str = DEFAULT_CAMERA_KIND,
  params_snapshot: dict | None = None,
  capability_flags: list[str] | None = None,
  previous_payload: dict | None = None,
) -> dict:
  now_ms = int(time.time() * 1000)
  payload = previous_payload if isinstance(previous_payload, dict) else empty_live_payload(repo_flavor=repo_flavor, selected_camera=selected_camera)
  meta = payload.get("meta")
  if not isinstance(meta, dict):
    meta = {}
    payload["meta"] = meta
  runtime = payload.get("runtime")
  if not isinstance(runtime, dict):
    runtime = {}
    payload["runtime"] = runtime
  services_payload = payload.get("services")
  if not isinstance(services_payload, dict):
    services_payload = {}
    payload["services"] = services_payload

  meta["schemaVersion"] = SCHEMA_VERSION
  meta["repoFlavor"] = repo_flavor
  meta["selectedCamera"] = selected_camera
  meta["generatedAtMs"] = now_ms
  meta["payloadKind"] = PAYLOAD_KIND_SERVICE_RAW
  caps = _ensure_list(meta, "capabilities")
  caps.clear()
  caps.extend(capability_flags or ())

  runtime["generatedAtMs"] = now_ms
  runtime["params"] = params_snapshot or {}

  service_alive = _ensure_dict(runtime, "serviceAlive")
  _update_alive_map(sm, service_alive)
  core_alive = _ensure_dict(runtime, "coreServicesAlive")
  _update_alive_subset(service_alive, common_services(), core_alive)
  optional_alive = _ensure_dict(runtime, "optionalServicesAlive")
  _update_alive_subset(service_alive, optional_services(), optional_alive)
  runtime["activeCoreServices"] = sum(1 for alive in core_alive.values() if alive)
  runtime["activeOptionalServices"] = sum(1 for alive in optional_alive.values() if alive)
  _refill_missing(service_alive, _ensure_list(runtime, "missingServices"))
  _refill_missing(core_alive, _ensure_list(runtime, "missingCoreServices"))
  _refill_missing(optional_alive, _ensure_list(runtime, "missingOptionalServices"))
  runtime["snapshotFresh"] = len(runtime["missingCoreServices"]) == 0

  service_names = []
  for name in list(common_services()) + list(optional_services()):
    if name in service_names:
      continue
    service_names.append(name)
    services_payload[name] = _build_service_payload(sm, name, previous=services_payload.get(name))
  for stale_name in list(services_payload.keys()):
    if stale_name not in service_names:
      del services_payload[stale_name]
  return payload


def empty_snapshot(*, repo_flavor: str, selected_camera: str = DEFAULT_CAMERA_KIND) -> dict:
  return empty_live_payload(repo_flavor=repo_flavor, selected_camera=selected_camera)


def build_snapshot(
  *,
  sm: Any,
  repo_flavor: str,
  selected_camera: str = DEFAULT_CAMERA_KIND,
  params_snapshot: dict | None = None,
  capability_flags: list[str] | None = None,
) -> dict:
  return build_live_payload(
    sm=sm,
    repo_flavor=repo_flavor,
    selected_camera=selected_camera,
    params_snapshot=params_snapshot,
    capability_flags=capability_flags,
  )


def _service_alive_map(sm: Any) -> dict[str, bool]:
  try:
    names = list(sm.data.keys())
  except Exception:
    names = []
  return {name: _service_alive(sm, name) for name in names}


def _alive_subset(service_alive: dict[str, bool], names: list[str] | tuple[str, ...]) -> dict[str, bool]:
  return {name: bool(service_alive.get(name, False)) for name in names}


def _service_alive(sm: Any, name: str) -> bool:
  try:
    return bool(sm.alive[name])
  except Exception:
    return False


def _service(sm: Any, name: str) -> Any:
  if not _service_alive(sm, name):
    return None
  try:
    return sm[name]
  except Exception:
    return None


def _build_service_payload(sm: Any, name: str, previous: dict[str, Any] | None = None) -> dict[str, Any] | None:
  builder = _SERVICE_BUILDERS.get(name)
  service = _service(sm, name)
  if service is None:
    return None
  if builder is None:
    return {}
  try:
    return builder(service, previous)
  except Exception:
    return None


def _update_alive_map(sm: Any, target: dict[str, bool]) -> None:
  target.clear()
  try:
    names = list(sm.data.keys())
  except Exception:
    return
  for name in names:
    target[name] = _service_alive(sm, name)


def _update_alive_subset(source: dict[str, bool], names: list[str] | tuple[str, ...], target: dict[str, bool]) -> None:
  target.clear()
  for name in names:
    target[name] = bool(source.get(name, False))


def _refill_missing(alive_map: dict[str, bool], target: list) -> None:
  target.clear()
  target.extend(name for name, alive in alive_map.items() if not alive)


def _ensure_dict(container: dict[str, Any], key: str) -> dict[str, Any]:
  value = container.get(key)
  if isinstance(value, dict):
    return value
  value = {}
  container[key] = value
  return value


def _ensure_list(container: dict[str, Any], key: str) -> list[Any]:
  value = container.get(key)
  if isinstance(value, list):
    return value
  value = []
  container[key] = value
  return value


def _fill_list(target: list[Any], value: Any, limit: int | None = None) -> list[Any]:
  del target[:]
  if value is None:
    return target
  try:
    for idx, item in enumerate(value):
      if limit is not None and idx >= limit:
        break
      target.append(item)
  except Exception:
    del target[:]
  return target


def _limited_items(value: Any, limit: int | None = None) -> list[Any]:
  items: list[Any] = []
  if value is None:
    return items
  try:
    for idx, item in enumerate(value):
      if limit is not None and idx >= limit:
        break
      items.append(item)
  except Exception:
    return []
  return items


def _update_xyz_payload(payload: dict[str, Any], source: Any) -> dict[str, Any]:
  _fill_list(_ensure_list(payload, "x"), safe_get(source, "x"), limit=33)
  _fill_list(_ensure_list(payload, "y"), safe_get(source, "y"), limit=33)
  _fill_list(_ensure_list(payload, "z"), safe_get(source, "z"), limit=33)
  return payload


def _sync_xyz_payloads(target: list[Any], source: Any, limit: int) -> list[Any]:
  items = _limited_items(source, limit=limit)
  if len(target) > len(items):
    del target[len(items):]
  for idx, item in enumerate(items):
    payload = target[idx] if idx < len(target) and isinstance(target[idx], dict) else {}
    payload = _update_xyz_payload(payload, item)
    if idx < len(target):
      target[idx] = payload
    else:
      target.append(payload)
  return target


def _sync_leads_payloads(target: list[Any], source: Any, limit: int = 4) -> list[Any]:
  items = _limited_items(source, limit=limit)
  if len(target) > len(items):
    del target[len(items):]
  for idx, lead in enumerate(items):
    payload = target[idx] if idx < len(target) and isinstance(target[idx], dict) else {}
    payload["prob"] = safe_float(safe_get(lead, "prob"))
    payload["x"] = safe_float(safe_get(lead, "x"))
    payload["y"] = safe_float(safe_get(lead, "y"))
    payload["v"] = safe_float(safe_get(lead, "v"))
    payload["a"] = safe_float(safe_get(lead, "a"))
    if idx < len(target):
      target[idx] = payload
    else:
      target.append(payload)
  return target


def _sync_radar_payloads(target: list[Any], source: Any, limit: int = 8) -> list[Any]:
  items = _limited_items(source, limit=limit)
  if len(target) > len(items):
    del target[len(items):]
  for idx, lead in enumerate(items):
    payload = target[idx] if idx < len(target) and isinstance(target[idx], dict) else {}
    payload.clear()
    payload.update(_radar_lead(lead) or {})
    if idx < len(target):
      target[idx] = payload
    else:
      target.append(payload)
  return target


def _build_selfdrive_state(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["enabled"] = safe_bool(safe_get(service, "enabled"))
  p["experimentalMode"] = safe_bool(safe_get(service, "experimentalMode"))
  p["alertType"] = safe_text(safe_get(service, "alertType"))
  p["alertText1"] = safe_text(safe_get(service, "alertText1"))
  p["alertText2"] = safe_text(safe_get(service, "alertText2"))
  return p


def _build_car_state(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["vEgo"] = safe_float(safe_get(service, "vEgo"))
  p["aEgo"] = safe_float(safe_get(service, "aEgo"))
  p["vEgoCluster"] = safe_float(safe_get(service, "vEgoCluster"))
  p["vCruiseCluster"] = safe_float(safe_get(service, "vCruiseCluster"))
  p["steeringAngleDeg"] = safe_float(safe_get(service, "steeringAngleDeg"))
  p["useLaneLineSpeed"] = safe_int(safe_get(service, "useLaneLineSpeed"))
  p["leftLaneLine"] = safe_int(safe_get(service, "leftLaneLine"))
  p["rightLaneLine"] = safe_int(safe_get(service, "rightLaneLine"))
  p["gearShifter"] = safe_text(safe_get(service, "gearShifter"))
  p["brakeLights"] = safe_bool(safe_get(service, "brakeLights"))
  p["leftBlindspot"] = safe_bool(safe_get(service, "leftBlindspot"))
  p["rightBlindspot"] = safe_bool(safe_get(service, "rightBlindspot"))
  p["standstill"] = safe_bool(safe_get(service, "standstill"))
  return p


def _build_controls_state(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["enabled"] = safe_bool(safe_get(service, "enabled"))
  p["experimentalMode"] = safe_bool(safe_get(service, "experimentalMode"))
  p["vCruise"] = safe_float(safe_get(service, "vCruise"))
  p["vCruiseCluster"] = safe_float(safe_get(service, "vCruiseCluster"))
  p["activeLaneLine"] = safe_bool(safe_get(service, "activeLaneLine"))
  p["curvature"] = safe_float(safe_get(service, "curvature"))
  p["desiredCurvature"] = safe_float(safe_get(service, "desiredCurvature"))
  return p


def _build_longitudinal_plan(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["personality"] = safe_text(safe_get(service, "personality"))
  p["myDrivingMode"] = safe_int(safe_get(service, "myDrivingMode"))
  _fill_list(_ensure_list(p, "accels"), safe_get(service, "accels"), limit=33)
  _fill_list(_ensure_list(p, "speeds"), safe_get(service, "speeds"), limit=33)
  _fill_list(_ensure_list(p, "jerks"), safe_get(service, "jerks"), limit=33)
  p["tFollow"] = safe_float(safe_get(service, "tFollow"))
  p["desiredDistance"] = safe_float(safe_get(service, "desiredDistance"))
  p["xState"] = safe_int(safe_get(service, "xState"))
  p["trafficState"] = safe_int(safe_get(service, "trafficState"))
  return p


def _build_live_calibration(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  payload = previous if isinstance(previous, dict) else {}
  payload["calStatus"] = safe_int(safe_get(service, "calStatus"))
  _fill_list(_ensure_list(payload, "rpyCalib"), safe_get(service, "rpyCalib"), limit=3)
  _fill_list(_ensure_list(payload, "wideFromDeviceEuler"), safe_get(service, "wideFromDeviceEuler"), limit=3)
  return payload


def _build_model_v2(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  position = safe_get(service, "position")
  lane_lines = safe_get(service, "laneLines")
  road_edges = safe_get(service, "roadEdges")
  leads = safe_get(service, "leadsV3")
  payload = previous if isinstance(previous, dict) else {}
  payload["frameId"] = safe_int(safe_get(service, "frameId"))
  _update_xyz_payload(_ensure_dict(payload, "position"), position)
  _fill_list(_ensure_list(payload, "laneLineProbs"), safe_get(service, "laneLineProbs"), limit=4)
  _fill_list(_ensure_list(payload, "laneLineStds"), safe_get(service, "laneLineStds"), limit=4)
  _fill_list(_ensure_list(payload, "roadEdgeStds"), safe_get(service, "roadEdgeStds"), limit=2)
  _sync_xyz_payloads(_ensure_list(payload, "laneLines"), lane_lines, limit=4)
  _sync_xyz_payloads(_ensure_list(payload, "roadEdges"), road_edges, limit=2)
  _sync_leads_payloads(_ensure_list(payload, "leadsV3"), leads, limit=4)
  return payload


def _build_camera_state(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  payload = previous if isinstance(previous, dict) else {}
  payload["frameId"] = safe_int(safe_get(service, "frameId"))
  payload["timestampSof"] = safe_int(safe_get(service, "timestampSof"))
  payload["timestampEof"] = safe_int(safe_get(service, "timestampEof"))
  payload["width"] = safe_int(safe_get(service, "width"))
  payload["height"] = safe_int(safe_get(service, "height"))
  return payload


def _build_device_state(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["started"] = safe_bool(safe_get(service, "started"))
  p["freeSpacePercent"] = safe_float(safe_get(service, "freeSpacePercent"))
  p["memoryUsagePercent"] = safe_float(safe_get(service, "memoryUsagePercent"))
  p["cpuTempC"] = to_list(safe_get(service, "cpuTempC"))
  return p


def _build_peripheral_state(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["voltage"] = safe_int(safe_get(service, "voltage"))
  return p


def _build_radar_state(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["leadOne"] = _radar_lead(safe_get(service, "leadOne"))
  p["leadTwo"] = _radar_lead(safe_get(service, "leadTwo"))
  p["leadLeft"] = _radar_lead(safe_get(service, "leadLeft"))
  p["leadRight"] = _radar_lead(safe_get(service, "leadRight"))
  _sync_radar_payloads(_ensure_list(p, "leadsLeft"), safe_get(service, "leadsLeft"), limit=6)
  _sync_radar_payloads(_ensure_list(p, "leadsCenter"), safe_get(service, "leadsCenter"), limit=6)
  _sync_radar_payloads(_ensure_list(p, "leadsRight"), safe_get(service, "leadsRight"), limit=6)
  return p


def _radar_lead(lead: Any) -> dict[str, Any] | None:
  if lead is None:
    return None
  return {
    "status": safe_bool(safe_get(lead, "status")),
    "dRel": safe_float(safe_get(lead, "dRel")),
    "yRel": safe_float(safe_get(lead, "yRel")),
    "vRel": safe_float(safe_get(lead, "vRel")),
    "aRel": safe_float(safe_get(lead, "aRel")),
    "vLeadK": safe_float(safe_get(lead, "vLeadK")),
    "vLat": safe_float(safe_get(lead, "vLat")),
    "aLead": safe_float(safe_get(lead, "aLead")),
    "aLeadK": safe_float(safe_get(lead, "aLeadK")),
    "modelProb": safe_float(safe_get(lead, "modelProb")),
    "score": safe_float(safe_get(lead, "score")),
    "radar": safe_bool(safe_get(lead, "radar")),
    "radarTrackId": safe_int(safe_get(lead, "radarTrackId")),
    "fcw": safe_bool(safe_get(lead, "fcw")),
    "jLead": safe_float(safe_get(lead, "jLead")),
  }


def _build_carrot(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["desiredSpeed"] = safe_float(safe_get(service, "desiredSpeed"))
  p["desiredSource"] = safe_text(safe_get(service, "desiredSource"))
  p["activeCarrot"] = safe_int(safe_get(service, "activeCarrot"))
  p["xState"] = safe_text(safe_get(service, "xState"))
  p["trafficState"] = safe_text(safe_get(service, "trafficState"))
  p["tFollow"] = safe_float(safe_get(service, "tFollow"))
  p["desiredDistance"] = safe_float(safe_get(service, "desiredDistance"))
  p["laneChangeState"] = safe_text(safe_get(service, "laneChangeState"))
  p["laneChangeDirection"] = safe_text(safe_get(service, "laneChangeDirection"))
  p["leftBlindspot"] = safe_bool(safe_get(service, "leftBlindspot"))
  p["rightBlindspot"] = safe_bool(safe_get(service, "rightBlindspot"))
  p["stockDebugTopRightText"] = safe_text(safe_get(service, "stockDebugTopRightText"))
  return p


def _build_gps(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["latitude"] = safe_float(safe_get(service, "latitude"))
  p["longitude"] = safe_float(safe_get(service, "longitude"))
  return p


def _build_lateral_plan(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["pathMode"] = safe_text(safe_get(service, "pathMode"))
  p["useLaneLineSpeed"] = safe_float(safe_get(service, "useLaneLineSpeed"))
  p["laneChangeState"] = safe_int(safe_get(service, "laneChangeState"))
  p["laneChangeDirection"] = safe_int(safe_get(service, "laneChangeDirection"))
  p["latDebugText"] = safe_text(safe_get(service, "latDebugText"))
  p["useLaneLines"] = safe_bool(safe_get(service, "useLaneLines"))
  _update_xyz_payload(_ensure_dict(p, "position"), safe_get(service, "position"))
  _fill_list(_ensure_list(p, "distances"), safe_get(service, "distances"), limit=33)
  return p


def _build_nav_instruction(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["mainText"] = safe_text(safe_get(service, "mainText"))
  p["distanceText"] = safe_text(safe_get(service, "distanceText"))
  p["turnType"] = safe_text(safe_get(service, "turnType"))
  return p


def _build_car_control(service: Any, previous: dict[str, Any] | None = None) -> dict[str, Any]:
  p = previous if isinstance(previous, dict) else {}
  p["latActive"] = safe_bool(safe_get(service, "latActive"))
  p["longActive"] = safe_bool(safe_get(service, "longActive"))
  return p


_SERVICE_BUILDERS: dict[str, Any] = {
  "selfdriveState": _build_selfdrive_state,
  "carState": _build_car_state,
  "controlsState": _build_controls_state,
  "longitudinalPlan": _build_longitudinal_plan,
  "liveCalibration": _build_live_calibration,
  "modelV2": _build_model_v2,
  "roadCameraState": _build_camera_state,
  "wideRoadCameraState": _build_camera_state,
  "deviceState": _build_device_state,
  "peripheralState": _build_peripheral_state,
  "radarState": _build_radar_state,
  "carrotMan": _build_carrot,
  "gpsLocationExternal": _build_gps,
  "lateralPlan": _build_lateral_plan,
  "navInstructionCarrot": _build_nav_instruction,
  "carControl": _build_car_control,
}
