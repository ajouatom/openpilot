import math
import os

from opendbc import DBC_PATH
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags, HyundaiExtFlags
from openpilot.common.params import Params
from opendbc.car.hyundai.hyundaicanfd import CanBus
from openpilot.common.filter_simple import MyMovingAverage

SCC_TID = 0
RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
RADAR_START_ADDR_CANFD1 = 0x210
RADAR_MSG_COUNT1 = 16
RADAR_START_ADDR_CANFD2 = 0x3A5 # Group 2, Group 1: 0x210 2媛쒖뵫?덉뼱???쇰떒 蹂대쪟.
RADAR_MSG_COUNT2 = 32
RADAR_START_ADDR_CANFD3 = 0x400
RADAR_MSG_COUNT3 = 30
CORNER_OBJECT_235_START_ADDR = 0x235
CORNER_OBJECT_235_MSG_COUNT = 20
CORNER_OBJECT_235_TRACK_ID_OFFSET = 200
CORNER_OBJECT_235_DBC = 'hyundai_canfd_corner_radar_235_generated'
CORNER_OBJECT_180_START_ADDR = 0x180
CORNER_OBJECT_180_MSG_COUNT = 5
CORNER_OBJECT_180_SLOTS_PER_MSG = 2
CORNER_OBJECT_180_TRACK_ID_OFFSET = 240
CORNER_OBJECT_180_DBC = 'hyundai_canfd_corner_radar_180_generated'

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP, radar_tracks, msg_start_addr, msg_count):
  if not radar_tracks:
    return None
  #if Bus.radar not in DBC[CP.carFingerprint]:
  #  return None
  print("RadarInterface: RadarTracks...")

  if CP.flags & HyundaiFlags.CANFD:
    CAN = CanBus(CP)
    messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(msg_start_addr, msg_start_addr + msg_count)]
    return CANParser('hyundai_canfd_radar_generated', messages, CAN.ACAN)
  else:
    messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(msg_start_addr, msg_start_addr + msg_count)]
  #return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)
    return CANParser('hyundai_kia_mando_front_radar_generated', messages, 1)

def get_corner_object_can_parser(CP, enabled):
  if not enabled or not (CP.flags & HyundaiFlags.CANFD):
    return None

  dbc_path = os.path.join(DBC_PATH, f"{CORNER_OBJECT_235_DBC}.dbc")
  if not os.path.exists(dbc_path):
    print(f"RadarInterface: missing {CORNER_OBJECT_235_DBC}.dbc, 0x235 corner radar disabled")
    return None

  CAN = CanBus(CP)
  messages = [(f"CORNER_RADAR_235_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_235_START_ADDR, CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT)]
  return CANParser(CORNER_OBJECT_235_DBC, messages, CAN.ACAN)

def get_corner_object_180_can_parser(CP, enabled):
  if not enabled or not (CP.flags & HyundaiFlags.CANFD):
    return None

  dbc_path = os.path.join(DBC_PATH, f"{CORNER_OBJECT_180_DBC}.dbc")
  if not os.path.exists(dbc_path):
    print(f"RadarInterface: missing {CORNER_OBJECT_180_DBC}.dbc, 0x180 corner radar disabled")
    return None

  CAN = CanBus(CP)
  messages = [(f"CORNER_RADAR_180_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_180_START_ADDR, CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT)]
  return CANParser(CORNER_OBJECT_180_DBC, messages, CAN.ACAN)

def get_radar_can_parser_scc(CP):
  CAN = CanBus(CP)
  if CP.flags & HyundaiFlags.CANFD:
    messages = [("SCC_CONTROL", 50)]
    bus = CAN.ECAN
  else:
    messages = [("SCC11", 50)]
    bus = CAN.ECAN

  print("$$$$$$$$ ECAN = ", CAN.ECAN)    
  bus = CAN.CAM if CP.flags & HyundaiFlags.CAMERA_SCC else bus
  return CANParser(DBC[CP.carFingerprint][Bus.pt], messages, bus)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    
    self.canfd = True if CP.flags & HyundaiFlags.CANFD else False
    self.radar_group1 = False
    self.radar_group3 = False
    if self.canfd:
      if CP.extFlags & HyundaiExtFlags.RADAR_GROUP1.value:
        self.radar_start_addr = RADAR_START_ADDR_CANFD1
        self.radar_msg_count = RADAR_MSG_COUNT1
        self.radar_group1 = True
      elif CP.extFlags & HyundaiExtFlags.RADAR_GROUP3.value:
        self.radar_start_addr = RADAR_START_ADDR_CANFD3
        self.radar_msg_count = RADAR_MSG_COUNT3
        self.radar_group3 = True
      else:
        self.radar_start_addr = RADAR_START_ADDR_CANFD2
        self.radar_msg_count = RADAR_MSG_COUNT2
    else:
      self.radar_start_addr = RADAR_START_ADDR
      self.radar_msg_count = RADAR_MSG_COUNT
      
    self.params = Params()
    self.radar_tracks = self.params.get_int("EnableRadarTracks") >= 1
    self.corner_object_tracks = bool(CP.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_235.value) and self.params.get_int("EnableCornerRadar") > 0
    self.corner_object_180_tracks = bool(CP.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_180.value) and self.params.get_int("EnableCornerRadar") > 0
    self.updated_tracks = set()
    self.updated_scc = set()
    self.updated_corner_objects = set()
    self.updated_corner_objects_180 = set()
    self.corner_object_missed_updates = 0
    self.corner_object_180_missed_updates = 0
    self.rcp_tracks = get_radar_can_parser(CP, self.radar_tracks, self.radar_start_addr, self.radar_msg_count)
    self.rcp_corner_objects = get_corner_object_can_parser(CP, self.corner_object_tracks)
    self.rcp_corner_objects_180 = get_corner_object_180_can_parser(CP, self.corner_object_180_tracks)
    # Enabling raw radar tracks on legacy CAN disables the stock SCC11 stream on
    # some Hyundai/Kia platforms. Camera-SCC cars may still use SCC11.
    use_scc_parser = not (self.radar_tracks and not self.canfd and not (CP.flags & HyundaiFlags.CAMERA_SCC))
    self.rcp_scc = get_radar_can_parser_scc(CP) if use_scc_parser else None
    self.trigger_msg_scc = 416 if self.canfd else 0x420

    self.trigger_msg_tracks = self.radar_start_addr + self.radar_msg_count - 1
    self.trigger_msg_corner_objects = CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT - 1
    self.trigger_msg_corner_objects_180 = CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT - 1
    self.track_id = 0

    self.corner_objects_available = self.rcp_corner_objects is not None or self.rcp_corner_objects_180 is not None
    self.radar_off_can = CP.radarUnavailable and not self.corner_objects_available
    print(
      "RadarInterface: "
      f"radarUnavailable={CP.radarUnavailable} radarTracks={self.radar_tracks} "
      f"corner235={self.rcp_corner_objects is not None} corner180={self.rcp_corner_objects_180 is not None} "
      f"radarOffCan={self.radar_off_can}"
    )

    self.vRel_last = 0
    self.dRel_last = 0

    # Initialize pts
    total_tracks = self.radar_msg_count * ( 2 if self.radar_group1 else 1)
    for track_id in range(total_tracks):
      t_id = track_id + 32
      self.pts[t_id] = structs.RadarData.RadarPoint()
      self.pts[t_id].measured = False
      self.pts[t_id].trackId = t_id

    self.pts[SCC_TID] = structs.RadarData.RadarPoint()
    self.pts[SCC_TID].trackId = SCC_TID
    for slot in range(CORNER_OBJECT_235_MSG_COUNT):
      t_id = CORNER_OBJECT_235_TRACK_ID_OFFSET + slot
      self.pts[t_id] = structs.RadarData.RadarPoint()
      self.pts[t_id].measured = False
      self.pts[t_id].trackId = t_id
    for slot in range(CORNER_OBJECT_180_MSG_COUNT * CORNER_OBJECT_180_SLOTS_PER_MSG):
      t_id = CORNER_OBJECT_180_TRACK_ID_OFFSET + slot
      self.pts[t_id] = structs.RadarData.RadarPoint()
      self.pts[t_id].measured = False
      self.pts[t_id].trackId = t_id

    self.frame = 0


  def update(self, can_strings):
    self.frame += 1
    if self.radar_off_can or (self.rcp_tracks is None and self.rcp_scc is None and self.rcp_corner_objects is None and self.rcp_corner_objects_180 is None):
      return super().update(None)

    if self.rcp_scc is not None:
      vls_s = self.rcp_scc.update(can_strings)
      self.updated_scc.update(vls_s)

    track_ready = False
    if self.radar_tracks and self.rcp_tracks is not None:
      vls_t = self.rcp_tracks.update(can_strings)
      self.updated_tracks.update(vls_t)
      track_ready = self.trigger_msg_tracks in self.updated_tracks

    corner_ready = False
    if self.rcp_corner_objects is not None:
      vls_c = self.rcp_corner_objects.update(can_strings)
      self.updated_corner_objects.update(vls_c)
      corner_ready = self.trigger_msg_corner_objects in self.updated_corner_objects

    corner_180_ready = False
    if self.rcp_corner_objects_180 is not None:
      vls_180 = self.rcp_corner_objects_180.update(can_strings)
      self.updated_corner_objects_180.update(vls_180)
      corner_180_ready = self.trigger_msg_corner_objects_180 in self.updated_corner_objects_180

    scc_ready = not self.radar_tracks and self.frame % 5 == 0 and self.rcp_scc is not None

    if track_ready:
      self._update(self.updated_tracks)
      self.updated_tracks.clear()

    if corner_ready:
      self._update_corner_objects(self.updated_corner_objects)
      self.corner_object_missed_updates = 0
      self.updated_corner_objects.clear()

    if corner_180_ready:
      self._update_corner_objects_180(self.updated_corner_objects_180)
      self.corner_object_180_missed_updates = 0
      self.updated_corner_objects_180.clear()

    # Corner radar runs at its own cadence. Do not let corner-only frames publish
    # RadarData, since liveTracks uses a fixed radarTimeStep for aLead/jLead.
    publish_ready = track_ready or scc_ready
    if not publish_ready:
      return None

    if self.rcp_scc is not None:
      self._update_scc(self.updated_scc)
    if self.rcp_corner_objects is not None:
      if self.updated_corner_objects:
        self._update_corner_objects(self.updated_corner_objects)
        self.corner_object_missed_updates = 0
      else:
        self.corner_object_missed_updates += 1
        if self.corner_object_missed_updates > 10:
          self._clear_corner_objects()
    if self.rcp_corner_objects_180 is not None:
      if self.updated_corner_objects_180:
        self._update_corner_objects_180(self.updated_corner_objects_180)
        self.corner_object_180_missed_updates = 0
      else:
        self.corner_object_180_missed_updates += 1
        if self.corner_object_180_missed_updates > 10:
          self._clear_corner_objects_180()
    self.updated_scc.clear()
    self.updated_corner_objects.clear()
    self.updated_corner_objects_180.clear()

    ret = structs.RadarData()
    if ((self.rcp_tracks is not None and self.radar_tracks and not self.rcp_tracks.can_valid) or
        (self.rcp_scc is not None and not self.corner_objects_available and not self.rcp_scc.can_valid) or
        (self.rcp_corner_objects is not None and not self.rcp_corner_objects.can_valid) or
        (self.rcp_corner_objects_180 is not None and not self.rcp_corner_objects_180.can_valid)):
      ret.errors.canError = True
    ret.points = list(self.pts.values())
    return ret

  def _update(self, updated_messages):

    t_id = 32
    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):

      msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]

      if self.radar_group1:
        valid = msg['VALID_CNT1'] > 10
      elif self.radar_group3:
        # Group 3 marks an empty object slot with LONG_DIST raw 0x7ff (204.7 m).
        valid = msg['LONG_DIST'] < 204.7
      elif self.canfd:
        valid = msg['VALID_CNT'] > 10
      else:
        valid = msg['STATE'] in (3, 4)

      self.pts[t_id].measured = bool(valid)
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
      elif self.radar_group1:
        self.pts[t_id].dRel = msg['LONG_DIST1']
        self.pts[t_id].yRel = msg['LAT_DIST1']
        self.pts[t_id].vRel = msg['REL_SPEED1']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = msg['REL_ACCEL1']
        self.pts[t_id].yvRel = msg['LAT_SPEED1']
      elif self.canfd:
        if self.radar_group3:
          # Group 3 reports the object's center. Convert it to the rear surface to match SCC/vision dRel.
          self.pts[t_id].dRel = max(0.0, msg['LONG_DIST'] - msg['OBJECT_LENGTH'] * 0.5 - 0.1)
        else:
          self.pts[t_id].dRel = msg['LONG_DIST']
        self.pts[t_id].yRel = msg['LAT_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan') if self.radar_group3 else msg['REL_ACCEL']
        self.pts[t_id].yvRel = 0.0 if self.radar_group3 else msg['LAT_SPEED']
      else:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[t_id].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[t_id].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = msg['REL_ACCEL']
        self.pts[t_id].yvRel = 0.0

      t_id += 1
    # radar group1? ?섎굹??msg??2媛쒖쓽 ?덉씠?붽? ?ㅼ뼱?덉쓬.
    if self.radar_group1:
      for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):
        msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]

        valid = msg['VALID_CNT2'] > 10
        self.pts[t_id].measured = bool(valid)
        if not valid:
          self.pts[t_id].dRel = 0
          self.pts[t_id].yRel = 0
          self.pts[t_id].vRel = 0
          self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
          self.pts[t_id].aRel = float('nan')
          self.pts[t_id].yvRel = 0
        else:
          self.pts[t_id].dRel = msg['LONG_DIST2']
          self.pts[t_id].yRel = msg['LAT_DIST2']
          self.pts[t_id].vRel = msg['REL_SPEED2']
          self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
          self.pts[t_id].aRel = msg['REL_ACCEL2']
          self.pts[t_id].yvRel = msg['LAT_SPEED2']

        t_id += 1

  def _update_corner_objects(self, updated_messages):
    if self.rcp_corner_objects is None:
      return

    if not updated_messages:
      self._clear_corner_objects()
      return

    for slot, addr in enumerate(range(CORNER_OBJECT_235_START_ADDR, CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT)):
      t_id = CORNER_OBJECT_235_TRACK_ID_OFFSET + slot
      msg = self.rcp_corner_objects.vl[f"CORNER_RADAR_235_OBJECTS_{addr:x}"]

      d_rel = msg["OBJ_REL_POS_X"]
      y_rel = msg["OBJ_REL_POS_Y"]
      v_rel = msg["OBJ_REL_VEL_X"]
      yv_rel = msg["OBJ_REL_VEL_Y"]
      a_rel = msg["OBJ_REL_ACCEL_X"]
      valid = msg["OBJ_QUAL_LEVEL"] > 0 and 0.2 < d_rel < 180.0 and abs(y_rel) < 40.0 and v_rel > -99.0

      self.pts[t_id].measured = bool(valid)
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
        continue

      self.pts[t_id].dRel = d_rel
      self.pts[t_id].yRel = y_rel
      self.pts[t_id].vRel = v_rel
      self.pts[t_id].vLead = v_rel + self.v_ego
      self.pts[t_id].aRel = a_rel
      self.pts[t_id].yvRel = yv_rel

  def _update_corner_objects_180(self, updated_messages):
    if self.rcp_corner_objects_180 is None:
      return

    if not updated_messages:
      self._clear_corner_objects_180()
      return

    for msg_index, addr in enumerate(range(CORNER_OBJECT_180_START_ADDR, CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT)):
      msg = self.rcp_corner_objects_180.vl[f"CORNER_RADAR_180_OBJECTS_{addr:x}"]
      for slot_index in range(CORNER_OBJECT_180_SLOTS_PER_MSG):
        t_id = CORNER_OBJECT_180_TRACK_ID_OFFSET + msg_index * CORNER_OBJECT_180_SLOTS_PER_MSG + slot_index
        prefix = f"SLOT{slot_index + 1}_"
        d_rel = msg[f"{prefix}REL_POS_X"]
        y_rel = msg[f"{prefix}REL_POS_Y"]
        v_rel = msg[f"{prefix}REL_VEL_X"]
        yv_rel = msg[f"{prefix}REL_VEL_Y"]
        a_rel = msg[f"{prefix}REL_ACCEL_X"]
        valid = msg[f"{prefix}QUAL_LEVEL"] > 0 and 0.2 < d_rel < 180.0 and abs(y_rel) < 40.0 and v_rel > -99.0

        self.pts[t_id].measured = bool(valid)
        if not valid:
          self._clear_point(t_id)
          continue

        self.pts[t_id].dRel = d_rel
        self.pts[t_id].yRel = y_rel
        self.pts[t_id].vRel = v_rel
        self.pts[t_id].vLead = v_rel + self.v_ego
        self.pts[t_id].aRel = a_rel
        self.pts[t_id].yvRel = yv_rel

  def _clear_point(self, t_id):
    self.pts[t_id].measured = False
    self.pts[t_id].dRel = 0
    self.pts[t_id].yRel = 0
    self.pts[t_id].vRel = 0
    self.pts[t_id].vLead = self.v_ego
    self.pts[t_id].aRel = float('nan')
    self.pts[t_id].yvRel = 0

  def _clear_corner_objects(self):
    for slot in range(CORNER_OBJECT_235_MSG_COUNT):
      self._clear_point(CORNER_OBJECT_235_TRACK_ID_OFFSET + slot)

  def _clear_corner_objects_180(self):
    for slot in range(CORNER_OBJECT_180_MSG_COUNT * CORNER_OBJECT_180_SLOTS_PER_MSG):
      self._clear_point(CORNER_OBJECT_180_TRACK_ID_OFFSET + slot)

  def _update_scc(self, updated_messages):
    cpt = self.rcp_scc.vl
    t_id = SCC_TID
    if self.canfd:
      dRel = cpt["SCC_CONTROL"]['ACC_ObjDist']
      vRel = cpt["SCC_CONTROL"]['ACC_ObjRelSpd']
      new_pts = abs(dRel - self.dRel_last) > 3 or abs(vRel - self.vRel_last) > 1
      vLead = vRel + self.v_ego
      valid = 0 < dRel < 150 and not new_pts #cpt["SCC_CONTROL"]['OBJ_STATUS'] and dRel < 150
      self.pts[t_id].measured = bool(valid)
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
      else:
        self.pts[t_id].dRel = dRel
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = vRel
        self.pts[t_id].vLead = vLead
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0 #float('nan')
    else:
      dRel = cpt["SCC11"]['ACC_ObjDist']
      vRel = cpt["SCC11"]['ACC_ObjRelSpd']
      new_pts = abs(dRel - self.dRel_last) > 3 or abs(vRel - self.vRel_last) > 1
      vLead = vRel + self.v_ego
      valid = cpt["SCC11"]['ACC_ObjStatus'] and dRel < 150 and not new_pts
      self.pts[t_id].measured = bool(valid)
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
      else:
        self.pts[t_id].dRel = dRel
        self.pts[t_id].yRel = -cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
        self.pts[t_id].vRel = vRel
        self.pts[t_id].vLead = vLead
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0 #float('nan')

    self.dRel_last = dRel
    self.vRel_last = vRel
