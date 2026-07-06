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

    active_objects: dict[int, tuple[float, float, float]] = {}
    for obj_id_sig, long_sig, lat_sig, vel_sig in SIGNAL_SETS:
      obj_id = get(obj_id_sig)
      if obj_id == NO_OBJECT:
        continue

      d_rel = get(long_sig)
      y_rel = get(lat_sig)
      v_rel = get(vel_sig)

      # 유효성 게이트 (safety): 전방 리드만 통과시킨다.
      # Long_Distance는 offset -6m라 raw가 낮으면 0 근처/음수로 디코딩됨 -> 노이즈/유령 객체가
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
      pt.yvRel = math.nan

    inactive_ids = self.pts.keys() - active_objects.keys()
    for obj_id in inactive_ids:
      self.pts.pop(obj_id, None)


    ret.points = list(self.pts.values())
    return ret
