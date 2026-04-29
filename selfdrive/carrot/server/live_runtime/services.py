from __future__ import annotations

from typing import Iterable, Sequence


# NOTE FOR FUTURE EDITS:
# - Every service listed here must be safe to miss temporarily.
# - If a service is not alive or a field is unavailable, snapshot.py must emit
#   a safe empty value (None / [] / False) instead of raising.
# - Adding a service here is not enough; keep empty_snapshot() defaults and
#   _fill_*() logic aligned so broker output stays stable during startup/stale periods.
#
# COMMON_SERVICES:
# - Required for the baseline /ws/live contract.
# - Removing one of these usually breaks core HUD or stock graphics output.
COMMON_SERVICES: tuple[str, ...] = (
  "selfdriveState",    # Engage state / HUD state.
  "carState",          # Vehicle speed, gear, brake, steering.
  "controlsState",     # Enabled state, alerts, actuator-related HUD state.
  "longitudinalPlan",  # Cruise and longitudinal helper state.
  "liveCalibration",   # Projection / camera calibration basis.
  "modelV2",           # Lane, path, road edge, lead graphics source.
  "roadCameraState",   # Road frameId / timestamp / sync basis.
  "deviceState",       # Device health and metric state.
)

# OPTIONAL_SERVICES:
# - Feature-expansion services.
# - Removing these should only disable specific features, not the base contract.
OPTIONAL_SERVICES: tuple[str, ...] = (
  "radarState",           # Radar badge / vector / lead state.
  "carrotMan",            # Carrot-specific HUD and state.
  "gpsLocationExternal",  # Time / location helpers.
  "lateralPlan",          # Lane-less and lateral helper state.
  "navInstructionCarrot", # Nav instruction text / turn state.
  "peripheralState",      # Voltage and peripheral info.
  "wideRoadCameraState",  # Metadata kept for sync/fallback compatibility.
  "carControl",           # Extra actuator / debug state when needed.
)

DEFAULT_OPTIONAL_SERVICES: tuple[str, ...] = OPTIONAL_SERVICES


def build_service_list(
  include_optional: Iterable[str] | None = None,
  exclude: Iterable[str] | None = None,
) -> list[str]:
  exclude_set = set(exclude or ())
  optional_names = tuple(include_optional or DEFAULT_OPTIONAL_SERVICES)
  ordered: list[str] = []

  for name in COMMON_SERVICES:
    if name not in exclude_set:
      ordered.append(name)

  for name in optional_names:
    if name not in exclude_set and name not in ordered:
      ordered.append(name)

  return ordered


def common_services() -> Sequence[str]:
  return COMMON_SERVICES


def optional_services() -> Sequence[str]:
  return OPTIONAL_SERVICES
