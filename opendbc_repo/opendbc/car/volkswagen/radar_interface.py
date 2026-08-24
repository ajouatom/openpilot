import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.volkswagen.values import DBC, VolkswagenFlags

# Ported from infiniteCable2/opendbc (VW MEB radar), adapted to carrot RadarInterfaceBase API.
# Strukturen_01 (0x24F) carries up to 6 tracked objects (Same/Left/Right lane x 2) on the radar bus.

RADAR_ADDR = 0x24F
NO_OBJECT = 0
LANE_TYPES = ("Same_Lane", "Left_Lane", "Right_Lane")
RADAR_DT = 0.04           # Strukturen_01 25Hz
YV_FILTER_ALPHA = 0.25    # 횡속도 저역필터 (프레임 노이즈 억제)
YV_MAX = 6.0              # m/s, 비현실적 횡속도 클램프
SIGNAL_SETS = tuple(
  (
    f"{prefix}_ObjectID",
    f"{prefix}_Long_Distance",
    f"{prefix}_Lat_Distance",
    f"{prefix}_Rel_Velo",
  )
  for lane in LANE_TYPES
  for idx in (1, 2)
  for prefix in (f"{lane}_0{idx}",)
)


def get_radar_can_parser(CP):
  if not (CP.flags & VolkswagenFlags.MEB):
    return None
  if CP.radarUnavailable or Bus.radar not in DBC[CP.carFingerprint]:
    return None
  messages = [("Strukturen_01", 25)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 2)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages: set[int] = set()
    self.trigger_msg = RADAR_ADDR
    self._track_id_counter = 0
    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)
    # 횡속도(yvRel) 산출: 레이더가 횡속도를 직접 안 주므로 yRel 미분+저역필터로 생성.
    # radard의 끼어들기 선승격 판정(yvRel 기반 미래 횡위치 예측)에 필요.
    self._yv_state: dict[int, tuple[float, float]] = {}  # obj_id -> (prev_yRel, yv_filtered)

  def update(self, can_strings):
    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    radar_data = self._process_radar_frame()
    self.updated_messages.clear()
    return radar_data

  def _process_radar_frame(self):
    ret = structs.RadarData()

    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True
      return ret

    msg = self.rcp.vl["Strukturen_01"]
    get = msg.__getitem__

    # 레이더 가림/이용불가 (commaai/opendbc 동일): Distance_Status != 0 이면 센서가 가려진 상태
    # (3 = obstructed 실측). 눈/진흙으로 가려져도 "앞차 없음"으로 오인하지 않게 오류로 올린다.
    if msg["Distance_Status"] != 0:
      ret.errors.radarUnavailableTemporary = True

    active_objects: dict[int, tuple[float, float, float]] = {}
    for obj_id_sig, long_sig, lat_sig, vel_sig in SIGNAL_SETS:
      obj_id = get(obj_id_sig)
      if obj_id == NO_OBJECT:
        continue

      d_rel = get(long_sig)
      y_rel = get(lat_sig)
      v_rel = get(vel_sig)

      # 유효성 게이트 (safety): 전방 리드만 통과시킨다.
      # Long_Distance는 offset -3.75m라 raw가 낮으면 0 근처/음수로 디코딩됨 -> 노이즈/유령 객체가
      # "코앞에 차"로 잡혀 인게이지 순간 오탐 FCW/급제동을 유발. 비현실적 근접(<=1m)·범위 밖은 제외.
      if not (1.0 < d_rel < 250.0):
        continue
      if abs(y_rel) > 10.0:
        continue


      if obj_id in active_objects:
        ret.errors.canError = True
        return ret

      active_objects[obj_id] = (d_rel, y_rel, v_rel)

    for obj_id, (d_rel, y_rel, v_rel) in active_objects.items():
      if obj_id not in self.pts:
        pt = structs.RadarData.RadarPoint()
        pt.trackId = self._track_id_counter
        self._track_id_counter += 1
        self.pts[obj_id] = pt
      else:
        pt = self.pts[obj_id]

      pt.measured = True
      pt.dRel = d_rel
      pt.yRel = y_rel
      pt.vRel = v_rel
      # ★ 핵심 수정: 절대 리드속도 = ego + 상대속도. 이걸 안 넣어서 vLead가 0(정지)으로 읽혀
      # 모든 레이더 객체가 "정지차"로 잡혀 급제동·출발막힘이 났음. carrot radard/현대RI와 동일하게 설정.
      pt.vLead = self.v_ego + v_rel
      pt.aRel = math.nan
      # 횡속도: yRel 미분 + 저역필터 (첫 프레임 0). ID 재사용 점프는 클램프+필터로 완충.
      prev = self._yv_state.get(obj_id)
      if prev is None:
        yv = 0.0
      else:
        raw = (y_rel - prev[0]) / RADAR_DT
        raw = max(-YV_MAX, min(YV_MAX, raw))
        yv = prev[1] + YV_FILTER_ALPHA * (raw - prev[1])
      self._yv_state[obj_id] = (y_rel, yv)
      pt.yvRel = yv

    inactive_ids = self.pts.keys() - active_objects.keys()
    for obj_id in inactive_ids:
      self.pts.pop(obj_id, None)
      self._yv_state.pop(obj_id, None)


    ret.points = list(self.pts.values())
    return ret
