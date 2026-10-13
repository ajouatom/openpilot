"""RAV4 2022의 직접 가감속 모드에서 관측한 순정 설정속도를 연결한다."""

from math import isfinite

from openpilot.common.constants import CV


class ToyotaPcmSetSpeedSync:
  """버튼 메시지를 생성하지 않고 유효한 PCM 속도 변경과 재개만 관측한다."""

  def __init__(self, max_speed_kph: float = 145.0):
    self.max_speed_kph = max_speed_kph
    self._observed_speed_kph: float | None = None
    self._was_enabled: bool | None = None

  def reset(self):
    self._observed_speed_kph = None
    self._was_enabled = None

  def update(self, CP, mode, CS, *, local_changed=False) -> tuple[float, float] | None:
    """확인한 순정 기본속도·계기판 속도를 반환하고 나머지 상황은 적용하지 않는다."""
    if not (
      getattr(CP, "carFingerprint", "") == "TOYOTA_RAV4_TSS2_2022"
      and getattr(CP, "brand", "") == "toyota"
      and getattr(CP, "pcmCruise", False)
      and getattr(CP, "openpilotLongitudinalControl", False)
      and mode == 0
      and getattr(CS, "canValid", False)
    ):
      self.reset()
      return None

    cruise_state = getattr(CS, "cruiseState", None)
    if cruise_state is None or not getattr(cruise_state, "available", False):
      self.reset()
      return None
    try:
      speed_kph = float(cruise_state.speed) * CV.MS_TO_KPH
    except (AttributeError, TypeError, ValueError, OverflowError):
      self.reset()
      return None
    if not isfinite(speed_kph):
      self.reset()
      return None
    # 원형 PCM 설정속도는 1 km/h 단위다. Float32 m/s 왕복 변환 오차만 정규화한다.
    unset_speed = speed_kph == 0.0
    speed_kph = round(speed_kph, 3)
    enabled = bool(getattr(cruise_state, "enabled", False))
    if unset_speed and not enabled:
      # 최초 SET 전의 0은 미설정이다. 속도로 쓰지 않고 비활성 관측만 보존한다.
      self._observed_speed_kph = None
      self._was_enabled = False
      return None
    if not 0 < speed_kph <= self.max_speed_kph:
      self.reset()
      return None

    previous_speed = self._observed_speed_kph
    resumed = enabled and self._was_enabled is False
    changed = previous_speed is not None and abs(speed_kph - previous_speed) >= 0.5
    # 로컬 버튼·블루투스가 우선인 프레임에서도 기준을 갱신해 다음 프레임의 덮어쓰기를 막는다.
    self._observed_speed_kph = speed_kph
    self._was_enabled = enabled
    if not enabled or local_changed or not (resumed or changed):
      return None

    try:
      cluster_kph = float(cruise_state.speedCluster) * CV.MS_TO_KPH
    except (AttributeError, TypeError, ValueError, OverflowError):
      cluster_kph = speed_kph
    if not isfinite(cluster_kph) or cluster_kph <= 0:
      cluster_kph = speed_kph
    else:
      cluster_kph = round(cluster_kph, 3)
      if not 0 < cluster_kph <= self.max_speed_kph:
        cluster_kph = speed_kph
    return speed_kph, cluster_kph
