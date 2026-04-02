from __future__ import annotations

RAW_CORE_SERVICES: tuple[str, ...] = (
  "selfdriveState",
  "carState",
  "controlsState",
  "longitudinalPlan",
  "liveCalibration",
  "modelV2",
  "roadCameraState",
  "deviceState",
)

RAW_OPTIONAL_SERVICES: tuple[str, ...] = (
  "radarState",
  "carrotMan",
  "gpsLocationExternal",
  "lateralPlan",
  "liveDelay",
  "liveTorqueParameters",
  "liveParameters",
  "navInstructionCarrot",
  "peripheralState",
  "wideRoadCameraState",
  "carControl",
)

DEFAULT_RAW_SERVICES: tuple[str, ...] = RAW_CORE_SERVICES + RAW_OPTIONAL_SERVICES


def raw_services() -> tuple[str, ...]:
  return DEFAULT_RAW_SERVICES


def is_supported_raw_service(service: str) -> bool:
  return service in DEFAULT_RAW_SERVICES
