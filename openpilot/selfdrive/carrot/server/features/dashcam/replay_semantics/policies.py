from __future__ import annotations

import re

from .models import StatisticsPolicy


GROUPS = (
  ("control", "replay_group_control"),
  ("vehicle", "replay_group_vehicle"),
  ("navigation", "replay_group_navigation"),
  ("planning", "replay_group_planning"),
  ("perception", "replay_group_perception"),
  ("location", "replay_group_location"),
  ("sensors", "replay_group_sensors"),
  ("camera", "replay_group_camera"),
  ("device", "replay_group_device"),
  ("connectivity", "replay_group_connectivity"),
  ("diagnostics", "replay_group_diagnostics"),
  ("raw", "replay_group_raw"),
)

GROUP_LABEL_KEYS = dict(GROUPS)

DOMAIN_GROUPS = {
  "audio": "raw",
  "feedback_audio": "raw",
  "control": "control",
  "control_command": "control",
  "control_event": "control",
  "control_output": "control",
  "control_state": "control",
  "debug_control": "control",
  "driver_assistance": "control",
  "feedback_marker": "control",
  "planning": "planning",
  "planning_tool": "planning",
  "model": "perception",
  "perception": "perception",
  "perception_raw": "perception",
  "driver_monitoring": "perception",
  "driver_monitoring_model": "perception",
  "vehicle_state": "vehicle",
  "vehicle_interface": "vehicle",
  "vehicle_metadata": "vehicle",
  "carrot_navigation": "navigation",
  "carrot_navigation_media": "navigation",
  "navigation": "navigation",
  "navigation_legacy": "navigation",
  "navigation_media": "navigation",
  "location": "location",
  "location_raw": "location",
  "localization": "location",
  "gnss_diagnostic": "location",
  "device_sensor": "sensors",
  "calibration": "sensors",
  "camera_metadata": "camera",
  "media": "camera",
  "media_index": "camera",
  "video_transport": "camera",
  "device_system": "device",
  "process_system": "diagnostics",
  "diagnostic_log": "diagnostics",
  "log_metadata": "diagnostics",
  "time_system": "diagnostics",
  "ui_debug": "diagnostics",
  "stream_transport": "connectivity",
  "stream_index": "connectivity",
  "upload_system": "connectivity",
  "vehicle_bus": "raw",
  "vehicle_bus_command": "raw",
  "custom_reserved": "raw",
  "user_input": "raw",
  "legacy/deprecated": "raw",
}

STATISTICS_POLICIES = {
  "continuous-numeric": StatisticsPolicy(
    "continuous-numeric",
    ("count", "invalid", "missing", "min", "max", "mean", "time-weighted-mean", "rms", "stddev", "percentiles", "histogram", "delta", "rate"),
    "minmax-envelope",
  ),
  "numeric-list": StatisticsPolicy(
    "numeric-list",
    ("count", "invalid", "missing", "empty", "length", "min", "max", "mean", "time-weighted-mean", "rms", "stddev", "percentiles", "histogram"),
    "minmax-envelope",
  ),
  "sequence-numeric": StatisticsPolicy(
    "sequence-numeric",
    ("count", "invalid", "missing", "min", "max", "duplicates", "backwards", "delta"),
    "state-transition-cap",
  ),
  "unresolved-numeric": StatisticsPolicy(
    "unresolved-numeric",
    ("count", "invalid", "missing", "min", "max", "mean", "stddev", "percentiles", "histogram"),
    "minmax-envelope",
  ),
  "exact-integer": StatisticsPolicy(
    "exact-integer",
    ("count", "invalid", "missing", "min", "max", "mean", "percentiles", "duplicates", "backwards", "max-step"),
    "state-transition-cap",
  ),
  "categorical-state": StatisticsPolicy(
    "categorical-state",
    ("count", "invalid", "missing", "distinct", "transitions", "state-duration"),
    "state-transition-cap",
  ),
  "text-transition": StatisticsPolicy(
    "text-transition",
    ("count", "invalid", "missing", "distinct-safe", "transitions", "length"),
    "state-transition-cap",
  ),
  "list-summary": StatisticsPolicy(
    "list-summary",
    ("count", "invalid", "missing", "empty", "length", "element-kind"),
    "snapshot-only",
  ),
  "binary-metadata": StatisticsPolicy(
    "binary-metadata",
    ("count", "invalid", "missing", "length", "distinct-safe"),
    "snapshot-only",
  ),
  "event-marker": StatisticsPolicy(
    "event-marker",
    ("count", "invalid", "occurrence-time"),
    "event-marker",
  ),
  "raw-metadata": StatisticsPolicy(
    "raw-metadata",
    ("count", "invalid", "missing"),
    "snapshot-only",
  ),
}


def group_for_domain(domain: str, service: str) -> str:
  normalized_domain = str(domain or "").strip().lower()
  if normalized_domain in DOMAIN_GROUPS:
    return DOMAIN_GROUPS[normalized_domain]
  name = str(service or "").lower()
  patterns = (
    (r"control|selfdrive|carcontrol", "control"),
    (r"plan|desire|trajectory", "planning"),
    (r"radar|model|lead|driverstate|monitoring", "perception"),
    (r"nav|map|route|carrot", "navigation"),
    (r"gps|gnss|ublox|location|position", "location"),
    (r"camera|encodeidx|thumbnail", "camera"),
    (r"sensor|gyro|accel|magnet|calibration", "sensors"),
    (r"wifi|network|upload|connect|ethernet", "connectivity"),
    (r"log|proc|manager|sentinel|clock|tombstone", "diagnostics"),
    (r"car|panda|vehicle|torque", "vehicle"),
    (r"device|thermal|power|peripheral", "device"),
  )
  for pattern, group in patterns:
    if re.search(pattern, name):
      return group
  return "raw"


def sequence_like(path: str) -> bool:
  leaf = str(path or "").lower().replace("[]", "").rsplit(".", 1)[-1]
  return bool(re.search(r"(id|index|idx|sequence|counter|timestamp.*)$", leaf))


def policy_key(capnp_type: str, path: str, unit_status: str) -> str:
  normalized = str(capnp_type or "").lower()
  if normalized in ("int64", "uint64", "list<int64>", "list<uint64>"):
    return "exact-integer"
  if normalized.startswith(("int", "uint", "float")):
    if sequence_like(path):
      return "sequence-numeric"
    if unit_status == "unresolved-numeric":
      return "unresolved-numeric"
    return "numeric-list" if "[]" in path else "continuous-numeric"
  if normalized in ("bool", "enum"):
    return "categorical-state"
  if normalized == "text":
    return "text-transition"
  if normalized == "data":
    return "binary-metadata"
  if normalized.startswith("list<") or "[]" in path:
    return "list-summary"
  if normalized == "marker":
    return "event-marker"
  return "raw-metadata"


def recommended_view(policy: str, unit_status: str) -> str:
  if policy == "continuous-numeric":
    return "graph" if unit_status != "unresolved-numeric" else "numeric-audit"
  if policy == "numeric-list":
    return "graph"
  if policy == "sequence-numeric":
    return "exact-values"
  if policy == "unresolved-numeric":
    return "numeric-audit"
  if policy == "exact-integer":
    return "exact-values"
  if policy == "categorical-state":
    return "state-band"
  if policy == "text-transition":
    return "text-transitions"
  if policy == "binary-metadata":
    return "binary-metadata"
  if policy == "event-marker":
    return "event-marker"
  if policy == "list-summary":
    return "list-snapshot"
  return "raw"


def display_precision(unit: str, policy: str) -> int | None:
  if policy not in ("continuous-numeric", "numeric-list", "unresolved-numeric"):
    return None
  if unit in ("%", "count", "count/code", "boolean", "categorical"):
    return 0
  if unit in ("deg", "km/h", "m", "s", "ms"):
    return 1
  return 2
