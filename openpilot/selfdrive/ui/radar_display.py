"""제어 트랙과 분리된 최신 ARS 표시점을 선택한다."""

from dataclasses import dataclass
from math import isfinite
import time


DISPLAY_MAX_AGE_S = 0.2
DISPLAY_MAX_POINTS = 20
# 원형 슬롯의 12비트 거리/횡거리 출력 범위다. 실물 센서 성능 한계가 아니다.
DISPLAY_MAX_DISTANCE_M = (4095 - 160) / 16
# upstream 좌우 반전 후의 두 범위를 합쳐 정상 경계점을 보존한다.
DISPLAY_MIN_LATERAL_M = -2048 * .015
DISPLAY_MAX_LATERAL_M = 2048 * .015
DISPLAY_TRACK_ID_LIMIT = 2**31


@dataclass(frozen=True)
class RadarDisplayPoint:
  dRel: float
  yRel: float
  vLeadK: float
  vLat: float
  radar: bool = True
  modelProb: float = 0.03


def _finite_float(value):
  try:
    result = float(value)
    return result if isfinite(result) else None
  except (TypeError, ValueError, OverflowError):
    return None


def _fresh(now_s, timestamp_s):
  value = _finite_float(timestamp_s)
  return value is not None and value <= now_s <= value + DISPLAY_MAX_AGE_S


def select_radar_display_points(sm, show_radar_info, *, now=None, started_frame=0):
  """None은 기존 표시, 빈 tuple은 지원 수신기의 표시 소실을 뜻한다."""
  if show_radar_info < 3:
    return None
  try:
    data = sm["liveTracks"]
  except (KeyError, TypeError):
    return None
  if not bool(getattr(data, "hasDisplayData", False)):
    return None
  # builder를 받은 오프라인 호출도 읽기 전용으로 바꿔 기본 errors 할당을 피한다.
  as_reader = getattr(data, "as_reader", None)
  if callable(as_reader):
    data = as_reader()

  # 지원 표지가 있으면 오류/빈 상태에서도 예전 제어 트랙으로 되돌리지 않는다.
  now_s = _finite_float(time.monotonic() if now is None else now)
  if now_s is None:
    return ()
  for name in ("seen", "alive", "valid"):
    if not getattr(sm, name, {}).get("liveTracks", False):
      return ()
  if getattr(sm, "recv_frame", {}).get("liveTracks", -1) < started_frame:
    return ()
  received_s = getattr(sm, "recv_time", {}).get("liveTracks")
  publication_ns = _finite_float(getattr(sm, "logMonoTime", {}).get("liveTracks"))
  if publication_ns is None or publication_ns <= 0 or not _fresh(now_s, received_s) or not _fresh(now_s, publication_ns * 1e-9):
    return ()
  try:
    if any(data.errors.to_dict().values()) or len(data.displayPoints) > DISPLAY_MAX_POINTS:
      return ()
  except (AttributeError, TypeError):
    return ()

  result = []
  identities = set()
  for point in data.displayPoints:
    # 예측점·미해결 값으로 주변 차량을 만들어 표시하지 않는다.
    if not bool(getattr(point, "measured", False)) or str(getattr(point, "radarSource", "")) != "frontRadar":
      continue
    fields = [_finite_float(getattr(point, field, None)) for field in ("dRel", "yRel", "vLead", "yvRel")]
    if any(value is None for value in fields):
      continue
    d_rel, y_rel, v_lead, v_lat = fields
    if not 0 < d_rel <= DISPLAY_MAX_DISTANCE_M or not DISPLAY_MIN_LATERAL_M <= y_rel <= DISPLAY_MAX_LATERAL_M:
      continue
    identity = getattr(point, "trackId", None)
    if isinstance(identity, bool) or not isinstance(identity, int) or not 0 <= identity < DISPLAY_TRACK_ID_LIMIT or identity in identities:
      return ()
    identities.add(identity)
    # 부모가 이미 적용한 좌우 반전 결과를 그대로 사용한다.
    result.append(RadarDisplayPoint(d_rel, y_rel, v_lead, v_lat))
  return tuple(result)
