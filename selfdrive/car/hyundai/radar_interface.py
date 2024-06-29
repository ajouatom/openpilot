import math

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.selfdrive.car.hyundai.values import DBC, CANFD_CAR
from openpilot.common.params import Params
from openpilot.common.filter_simple import StreamingMovingAverage

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP):
  enable_radar_tracks = Params().get_int("EnableRadarTracks")
  #if DBC[CP.carFingerprint]['radar'] is None and not enable_radar_tracks:
  if enable_radar_tracks == 0:
    return None

  print("RadarInterface: RadarTracks...")
  messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser('hyundai_kia_mando_front_radar_generated', messages, 1)

def get_radar_can_parser_scc(CP):
  scc2 = Params().get_int("SccConnectedBus2") > 0

  enable_radar_tracks = Params().get_int("EnableRadarTracks")

  # 레이더트랙만 이용하고 싶은경우, => mix 시험...
  if enable_radar_tracks in [1]:
    return None

  if CP.carFingerprint in CANFD_CAR and not scc2:
    return None

  # 롱컨은 켜졌지만, 배선개조가 안된경우 비젼롱컨
  if CP.openpilotLongitudinalControl and not scc2:
    return None

  ## 레이더트랙과 SCC레이더를 같이 사용하고 싶은경우.. (서로 주파수가 달라 사용하기엔 radard를 수정해야함)
  #if enable_radar_tracks and not scc2: #레이더트랙은 켜져있으나, SCC2가 아닌경우  : SCC기능정지후 사용하는 롱컨..
  #  return None

  print("RadarInterface: SCC Radar (Bus{})".format( 2 if scc2 else 0))
  if CP.carFingerprint in CANFD_CAR:
    messages = [("SCC_CONTROL", 50)]
  else:
    messages = [("SCC11", 50)]    
  return CANParser(DBC[CP.carFingerprint]['pt'], messages, 2 if scc2 else 0)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.enable_radar_tracks = Params().get_int("EnableRadarTracks")

    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 1

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)
    
    self.rcp_scc = get_radar_can_parser_scc(CP)
    self.updated_messages_scc = set()
    self.trigger_msg_scc = 0x420
    self.dRelFilter = StreamingMovingAverage(2)
    self.vRelFilter = StreamingMovingAverage(4)

    self.canfd = True if CP.carFingerprint in CANFD_CAR else False
    if self.canfd:
      self.trigger_msg_scc = 416

  def update(self, can_strings):
    #if not self.enable_radar_tracks and (self.radar_off_can or (self.rcp is None)):
    #  return super().update(None)

    if self.rcp is None and self.rcp_scc is None:
      return super().update(None)

    if self.rcp is not None:
      vls = self.rcp.update_strings(can_strings)
      self.updated_messages.update(vls)

    if self.rcp_scc is not None:
      vls_scc = self.rcp_scc.update_strings(can_strings)
      self.updated_messages_scc.update(vls_scc)

    trigger_msg_radar = True if self.trigger_msg in self.updated_messages else False
    trigger_msg_scc = True if self.trigger_msg_scc in self.updated_messages_scc else False

    if trigger_msg_radar or trigger_msg_scc:
      rr = self._update(self.updated_messages, trigger_msg_radar, trigger_msg_scc)
    else:
      return None

    self.updated_messages.clear()
    self.updated_messages_scc.clear()

    return rr

  def _update(self, updated_messages, trigger_msg_radar, trigger_msg_scc):
    ret = car.RadarData.new_message()
    if self.rcp is None and self.rcp_scc is None:
      return ret

    errors = []

    if self.rcp is not None and trigger_msg_radar:
      if not self.rcp.can_valid:
        errors.append("canError")

      for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
        msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

        if addr not in self.pts:
          self.pts[addr] = car.RadarData.RadarPoint.new_message()
          self.pts[addr].trackId = self.track_id
          self.track_id += 1

        valid = msg['STATE'] in (3, 4)
        if valid:
          azimuth = math.radians(msg['AZIMUTH'])
          self.pts[addr].measured = True
          self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
          self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
          self.pts[addr].vRel = msg['REL_SPEED']
          self.pts[addr].aRel = msg['REL_ACCEL']
          self.pts[addr].yvRel = float('nan')

        else:
          del self.pts[addr]

    if self.rcp_scc is not None and trigger_msg_scc:
      if not self.rcp_scc.can_valid:
        errors.append("canError")

      cpt = self.rcp_scc.vl
      if self.canfd:
        dRel = cpt["SCC_CONTROL"]['ACC_ObjDist']
        vRel = cpt["SCC_CONTROL"]['ACC_ObjRelSpd']
        valid = 0 < dRel < 150 #cpt["SCC_CONTROL"]['OBJ_STATUS'] and dRel < 150
        for ii in range(1):
          if valid:
            if ii not in self.pts:
              self.pts[ii] = car.RadarData.RadarPoint.new_message()
              self.pts[ii].trackId = 0 #self.track_id
              #self.track_id += 1
              dRel = self.dRelFilter.set(dRel)
              vRel = self.vRelFilter.set(vRel)
            else:
              dRel = self.dRelFilter.process(dRel)
              vRel = self.vRelFilter.process(vRel)
            self.pts[ii].dRel = dRel #cpt["SCC11"]['ACC_ObjDist']  # from front of car
            self.pts[ii].yRel = 0 #-cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
            self.pts[ii].vRel = vRel #cpt["SCC11"]['ACC_ObjRelSpd']
            self.pts[ii].aRel = 0 #float('nan')
            self.pts[ii].yvRel = float('nan')
            self.pts[ii].measured = True
          else:
            if ii in self.pts:
              del self.pts[ii]
      else:
        dRel = cpt["SCC11"]['ACC_ObjDist']
        vRel = cpt["SCC11"]['ACC_ObjRelSpd']
        valid = cpt["SCC11"]['ACC_ObjStatus'] and dRel < 150
        for ii in range(1):
          if valid:
            if ii not in self.pts:
              self.pts[ii] = car.RadarData.RadarPoint.new_message()
              self.pts[ii].trackId = 0 #self.track_id
              #self.track_id += 1
              dRel = self.dRelFilter.set(dRel)
              vRel = self.vRelFilter.set(vRel)
            else:
              dRel = self.dRelFilter.process(dRel)
              vRel = self.vRelFilter.process(vRel)
            self.pts[ii].dRel = dRel #cpt["SCC11"]['ACC_ObjDist']  # from front of car
            self.pts[ii].yRel = -cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
            self.pts[ii].vRel = vRel #cpt["SCC11"]['ACC_ObjRelSpd']
            self.pts[ii].aRel = 0 #float('nan')
            self.pts[ii].yvRel = float('nan')
            self.pts[ii].measured = True
          else:
            if ii in self.pts:
              del self.pts[ii]

    ret.points = list(self.pts.values())
    ret.errors = errors
    return ret
