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
  # Per-target radar tracks. Requested only while the Drive Insights forward
  # view is open, so the device opens this socket on demand rather than always.
  "liveTracks",
  # AR anchor inputs. 20Hz each, so they are requested only while the AR
  # overlay holds an "ar" activity lease - never on the always-on overlay path.
  "cameraOdometry",
  "livePose",
  # TMap 안내 의미. cereal 멀티캐스트라 지도 뷰의 단일 viewer ownership 과 무관하다.
  "carrotNavi",
)

DEFAULT_RAW_SERVICES: tuple[str, ...] = RAW_CORE_SERVICES + RAW_OPTIONAL_SERVICES


def raw_services() -> tuple[str, ...]:
  return DEFAULT_RAW_SERVICES


def is_supported_raw_service(service: str) -> bool:
  return service in DEFAULT_RAW_SERVICES
