import math
import os
from collections import deque

from opendbc import DBC_PATH
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags, HyundaiExtFlags
from openpilot.common.params import Params
from opendbc.car.hyundai.hyundaicanfd import CanBus

SCC_TID = 0
RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 64
RADAR_REQUIRED_MSG_COUNT = 32
RADAR_MSG_COUNT4 = 8
RADAR_GROUP4_MAX_LONG_DIST = 325.0
RADAR_GROUP4_MAX_YREL = 6.0
RADAR_START_ADDR_CANFD1 = 0x210
RADAR_MSG_COUNT1 = 16
RADAR_START_ADDR_CANFD2 = 0x3A5  # Group 2; Group 1 uses two 0x210 messages. Pending validation.
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
CORNER_OBJECT_430_LEFT_START_ADDR = 0x430
CORNER_OBJECT_430_RIGHT_START_ADDR = 0x440
CORNER_OBJECT_430_MSG_COUNT_PER_SIDE = 8
CORNER_OBJECT_430_SLOTS_PER_MSG = 7
CORNER_OBJECT_430_TRACK_ID_OFFSET = 300
CORNER_OBJECT_430_DBC = 'hyundai_canfd_corner_radar_430_generated'


def canfd_group2_track_status(msg):
  """Return the existing age gate and the radar-native object state."""
  return msg['VALID_CNT'] > 10, int(msg['VALID'])


CORNER_OBJECT_430_EMPTY_RAW_VALUES = (0x010d1f40, 0x00010d1f)
CORNER_OBJECT_430_DEFAULT_DISTANCE_RAW_MIN = 2520  # 126.0 m
CORNER_OBJECT_430_DEFAULT_DISTANCE_RAW_MAX = 2600  # 130.0 m
CORNER_OBJECT_430_MAX_DREL = 120.0
CORNER_OBJECT_430_MAX_TRACKS_PER_SIDE = 4
CORNER_OBJECT_430_DT = 0.05
CORNER_OBJECT_430_MAX_DREL_DELTA = 1.5
CORNER_OBJECT_430_CANDIDATE_META_BYTE_3 = (2,)
CORNER_OBJECT_430_CANDIDATE_EXCLUDED_SLOTS = (1,)
CORNER_OBJECT_430_CANDIDATE_RAW_DELTA = 200
CORNER_OBJECT_430_STRONG_META_BYTE_2 = (10,)
CORNER_OBJECT_430_WEAK_META_BYTE_2 = (5, 6, 7, 8, 9)
CORNER_OBJECT_430_STRONG_MIN_SUPPORT = 2
CORNER_OBJECT_430_WEAK_MIN_SUPPORT = 3
CORNER_OBJECT_430_CLUSTER_RAW_GAP = 200
CORNER_OBJECT_430_TRACK_MATCH_MAX_DREL_DELTA = 3.0
CORNER_OBJECT_430_MAX_ABS_VREL = 20.0
CORNER_OBJECT_430_MAX_ABS_YVREL = 3.0
CORNER_OBJECT_430_VREL_ALPHA = 0.35
CORNER_OBJECT_430_YVREL_ALPHA = 0.35
CORNER_OBJECT_430_LATERAL_CELL_MSG_WEIGHT = 0.35
CORNER_OBJECT_430_LATERAL_CELL_SLOT_WEIGHT = 0.65
CORNER_OBJECT_430_YREL_OFFSET = 5.8
CORNER_OBJECT_430_YREL_SCALE = 1.1
CORNER_OBJECT_430_RIGHT_CELL_MIRROR = 7.0
CORNER_OBJECT_430_MIN_ABS_YREL = 0.8
CORNER_OBJECT_430_MAX_ABS_YREL = 4.2
CORNER_OBJECT_430_HISTORY_SIZE = 8
CORNER_OBJECT_430_MIN_HISTORY = 5
CORNER_OBJECT_430_MIN_INWARD_YREL_DELTA = 0.35
CORNER_OBJECT_430_MIN_RECENT_INWARD_YREL_DELTA = 0.05
CORNER_OBJECT_430_MIN_INWARD_RATIO = 0.65
CORNER_OBJECT_430_INWARD_CENTER_ABS_YREL = 1.55
CORNER_OBJECT_430_INWARD_KEEP_YVREL_ABS_YREL = 2.2
CORNER_OBJECT_430_EARLY_INWARD_NONCENTER_FRAMES = 2
CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL = 2.0
CORNER_OBJECT_STABLE_TRACK_ID_START = 1000
CORNER_OBJECT_IDENTITY_STALE_CYCLES = 3
CORNER_OBJECT_IDENTITY_MAX_DREL_DELTA = 7.0
CORNER_OBJECT_IDENTITY_MAX_YREL_DELTA = 3.2
CORNER_OBJECT_HANDOFF_MAX_DREL_DELTA = 2.0
CORNER_OBJECT_HANDOFF_MAX_YREL_DELTA = 1.0
CORNER_OBJECT_HANDOFF_MAX_VREL_DELTA = 3.0
CORNER_SIDE_OBJECT_MAX_DREL = 0.2
CORNER_SIDE_OBJECT_MIN_ABS_YREL = 1.4
CORNER_SIDE_OBJECT_MAX_ABS_YREL = 4.5

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


class CornerObjectTrackIdManager:
  def __init__(self):
    self.next_track_id = CORNER_OBJECT_STABLE_TRACK_ID_START
    self.source_cycles: dict[str, int] = {}
    self.track_states: dict[tuple[str, int], tuple[int, int, int, float, float, int]] = {}

  def clear_source(self, source: str):
    self.track_states = {key: value for key, value in self.track_states.items() if key[0] != source}
    self.source_cycles.pop(source, None)

  def get_track_ids(self, source: str, candidates) -> dict[int, int]:
    cycle = self.source_cycles.get(source, 0) + 1
    self.source_cycles[source] = cycle
    previous = {
      track_id: state for (state_source, track_id), state in self.track_states.items()
      if state_source == source and cycle - state[5] <= CORNER_OBJECT_IDENTITY_STALE_CYCLES
    }
    used_track_ids = set()
    assignments = {}

    # Prefer the previous CAN slot, then permit a physically continuous slot
    # handoff. Object IDs are not globally unique: two distant objects can use
    # the same ID at the same time.
    for candidate in candidates:
      slot_id, object_id, age, _, d_rel, y_rel, *_ = candidate
      matches = []
      for track_id, state in previous.items():
        previous_slot, previous_object_id, previous_age, previous_d_rel, previous_y_rel, _ = state
        if track_id in used_track_ids or object_id != previous_object_id or age < previous_age:
          continue
        d_delta = abs(d_rel - previous_d_rel)
        y_delta = abs(y_rel - previous_y_rel)
        if d_delta > CORNER_OBJECT_IDENTITY_MAX_DREL_DELTA or y_delta > CORNER_OBJECT_IDENTITY_MAX_YREL_DELTA:
          continue
        matches.append((previous_slot != slot_id, d_delta + y_delta * 1.5, track_id))

      if matches:
        track_id = min(matches)[2]
      else:
        track_id = self.next_track_id
        self.next_track_id += 1
      assignments[slot_id] = track_id
      used_track_ids.add(track_id)
      self.track_states[(source, track_id)] = (slot_id, object_id, age, d_rel, y_rel, cycle)

    self.track_states = {
      key: state for key, state in self.track_states.items()
      if key[0] != source or cycle - state[5] <= CORNER_OBJECT_IDENTITY_STALE_CYCLES
    }
    return assignments


def deduplicate_corner_candidates(candidates):
  objects = []
  for candidate in candidates:
    _, object_id, age, quality, d_rel, y_rel, v_rel, *_ = candidate
    duplicate_index = None
    for index, previous in enumerate(objects):
      if object_id != previous[1]:
        continue
      if (abs(d_rel - previous[4]) <= CORNER_OBJECT_HANDOFF_MAX_DREL_DELTA and
          abs(y_rel - previous[5]) <= CORNER_OBJECT_HANDOFF_MAX_YREL_DELTA and
          abs(v_rel - previous[6]) <= CORNER_OBJECT_HANDOFF_MAX_VREL_DELTA):
        duplicate_index = index
        break
    if duplicate_index is None:
      objects.append(candidate)
    elif (age, quality) > (objects[duplicate_index][2], objects[duplicate_index][3]):
      objects[duplicate_index] = candidate
  return objects


def corner_object_position_valid(d_rel: float, y_rel: float) -> bool:
  normal_object = 0.2 < d_rel < 180.0
  clipped_side_object = (
    0.0 <= d_rel <= CORNER_SIDE_OBJECT_MAX_DREL and
    CORNER_SIDE_OBJECT_MIN_ABS_YREL <= abs(y_rel) <= CORNER_SIDE_OBJECT_MAX_ABS_YREL
  )
  return (normal_object or clipped_side_object) and abs(y_rel) < 40.0


def get_radar_can_parser(CP, radar_tracks, msg_start_addr, msg_count, required_msg_count, radar_group4=False):
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
    # Legacy Mando radars expose either 32 or 64 consecutive slots. Keep the
    # first 32 mandatory for timing/CAN validity and accept the upper bank when
    # present, so a 32-slot radar remains fully compatible.
    messages = [(f"RADAR_TRACK_{addr:x}", 20 if index < required_msg_count else math.nan)
                for index, addr in enumerate(range(msg_start_addr, msg_start_addr + msg_count))]
  #return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)
    dbc_name = 'hyundai_kia_denso_front_radar_generated' if radar_group4 else 'hyundai_kia_mando_front_radar_generated'
    return CANParser(dbc_name, messages, 1)

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

def get_corner_object_430_can_parser(CP, enabled):
  if not enabled or not (CP.flags & HyundaiFlags.CANFD):
    return None

  dbc_path = os.path.join(DBC_PATH, f"{CORNER_OBJECT_430_DBC}.dbc")
  if not os.path.exists(dbc_path):
    print(f"RadarInterface: missing {CORNER_OBJECT_430_DBC}.dbc, 0x430/0x440 corner radar disabled")
    return None

  CAN = CanBus(CP)
  messages = [(f"CORNER_RADAR_430_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_430_LEFT_START_ADDR, CORNER_OBJECT_430_LEFT_START_ADDR + CORNER_OBJECT_430_MSG_COUNT_PER_SIDE)]
  messages += [(f"CORNER_RADAR_430_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_430_RIGHT_START_ADDR, CORNER_OBJECT_430_RIGHT_START_ADDR + CORNER_OBJECT_430_MSG_COUNT_PER_SIDE)]
  return CANParser(CORNER_OBJECT_430_DBC, messages, CAN.ACAN)

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
    self.radar_group4 = not self.canfd and bool(CP.extFlags & HyundaiExtFlags.RADAR_GROUP4.value)
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
      self.radar_msg_count = RADAR_MSG_COUNT4 if self.radar_group4 else RADAR_MSG_COUNT
    self.radar_required_msg_count = self.radar_msg_count
    if not self.canfd and not self.radar_group4:
      self.radar_required_msg_count = RADAR_REQUIRED_MSG_COUNT

    self.params = Params()
    self.radar_tracks = self.params.get_int("EnableRadarTracks") >= 1
    self.corner_object_tracks = bool(CP.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_235.value) and self.params.get_int("EnableCornerRadar") > 0
    self.corner_object_180_tracks = bool(CP.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_180.value) and self.params.get_int("EnableCornerRadar") > 0
    # The 0x430/0x440 DBC exposes unvalidated range-bin candidates rather than
    # confirmed objects. Promoting them can create false side tracks and unsafe
    # lead selection, so retain the decoder for offline analysis only.
    self.corner_object_430_tracks = False
    self.updated_tracks = set()
    self.updated_scc = set()
    self.updated_corner_objects = set()
    self.updated_corner_objects_180 = set()
    self.updated_corner_objects_430 = set()
    self.corner_object_missed_updates = 0
    self.corner_object_180_missed_updates = 0
    self.corner_object_430_missed_updates = 0
    self.corner_object_track_ids = CornerObjectTrackIdManager()
    self.rcp_tracks = get_radar_can_parser(
      CP, self.radar_tracks, self.radar_start_addr, self.radar_msg_count,
      self.radar_required_msg_count, self.radar_group4,
    )
    self.rcp_corner_objects = get_corner_object_can_parser(CP, self.corner_object_tracks)
    self.rcp_corner_objects_180 = get_corner_object_180_can_parser(CP, self.corner_object_180_tracks)
    self.rcp_corner_objects_430 = get_corner_object_430_can_parser(CP, self.corner_object_430_tracks)
    # Enabling raw radar tracks on legacy CAN disables the stock SCC11 stream on
    # some Hyundai/Kia platforms. Camera-SCC cars may still use SCC11.
    use_scc_parser = not (self.radar_tracks and not self.canfd and not (CP.flags & HyundaiFlags.CAMERA_SCC))
    self.rcp_scc = get_radar_can_parser_scc(CP) if use_scc_parser else None
    self.trigger_msg_scc = 416 if self.canfd else 0x420

    self.trigger_msg_tracks = self.radar_start_addr + self.radar_required_msg_count - 1
    self.trigger_msg_corner_objects = CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT - 1
    self.trigger_msg_corner_objects_180 = CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT - 1
    self.trigger_msg_corner_objects_430 = CORNER_OBJECT_430_RIGHT_START_ADDR + CORNER_OBJECT_430_MSG_COUNT_PER_SIDE - 1
    self.track_id = 0

    self.corner_objects_available = self.rcp_corner_objects is not None or self.rcp_corner_objects_180 is not None or self.rcp_corner_objects_430 is not None
    self.radar_off_can = CP.radarUnavailable and not self.corner_objects_available
    print(
      "RadarInterface: "
      f"radarUnavailable={CP.radarUnavailable} radarTracks={self.radar_tracks} "
      f"group4={self.radar_group4} "
      f"corner235={self.rcp_corner_objects is not None} corner180={self.rcp_corner_objects_180 is not None} "
      f"corner430={self.rcp_corner_objects_430 is not None} "
      f"radarOffCan={self.radar_off_can}"
    )

    self.vRel_last = 0
    self.dRel_last = 0
    self.corner_object_430_prev_d_rel = {}
    self.corner_object_430_prev_v_rel = {}
    self.corner_object_430_prev_y_rel = {}
    self.corner_object_430_prev_yv_rel = {}
    self.corner_object_430_prev_code = {}
    self.corner_object_430_history = {}
    self.corner_object_430_noncenter_inward_frames = {}

    # Initialize pts
    if self.rcp_tracks is not None:
      total_tracks = self.radar_msg_count * (2 if self.radar_group1 else 1)
      for track_id in range(total_tracks):
        t_id = track_id + 32
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id

    if self.rcp_scc is not None:
      self.pts[SCC_TID] = structs.RadarData.RadarPoint()
      self.pts[SCC_TID].trackId = SCC_TID
      self.pts[SCC_TID].radarSource = "scc"
    if self.rcp_corner_objects is not None:
      for slot in range(CORNER_OBJECT_235_MSG_COUNT):
        t_id = CORNER_OBJECT_235_TRACK_ID_OFFSET + slot
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id
        self.pts[t_id].radarSource = "corner235"
    if self.rcp_corner_objects_180 is not None:
      for slot in range(CORNER_OBJECT_180_MSG_COUNT * CORNER_OBJECT_180_SLOTS_PER_MSG):
        t_id = CORNER_OBJECT_180_TRACK_ID_OFFSET + slot
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id
        self.pts[t_id].radarSource = "corner180"
    if self.rcp_corner_objects_430 is not None:
      for slot in range(CORNER_OBJECT_430_MSG_COUNT_PER_SIDE * 2 * CORNER_OBJECT_430_SLOTS_PER_MSG):
        t_id = CORNER_OBJECT_430_TRACK_ID_OFFSET + slot
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id
        self.pts[t_id].radarSource = "corner430"

    self.frame = 0


  def update(self, can_strings):
    self.frame += 1
    if self.radar_off_can or (self.rcp_tracks is None and self.rcp_scc is None and self.rcp_corner_objects is None and self.rcp_corner_objects_180 is None and self.rcp_corner_objects_430 is None):
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

    corner_430_ready = False
    if self.rcp_corner_objects_430 is not None:
      vls_430 = self.rcp_corner_objects_430.update(can_strings)
      self.updated_corner_objects_430.update(vls_430)
      corner_430_ready = self.trigger_msg_corner_objects_430 in self.updated_corner_objects_430

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

    if corner_430_ready:
      self._update_corner_objects_430(self.updated_corner_objects_430)
      self.corner_object_430_missed_updates = 0
      self.updated_corner_objects_430.clear()

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
    if self.rcp_corner_objects_430 is not None:
      if self.updated_corner_objects_430:
        self._update_corner_objects_430(self.updated_corner_objects_430)
        self.corner_object_430_missed_updates = 0
      else:
        self.corner_object_430_missed_updates += 1
        if self.corner_object_430_missed_updates > 10:
          self._clear_corner_objects_430()
    self.updated_scc.clear()
    self.updated_corner_objects.clear()
    self.updated_corner_objects_180.clear()
    self.updated_corner_objects_430.clear()

    ret = structs.RadarData()
    if ((self.rcp_tracks is not None and self.radar_tracks and not self.rcp_tracks.can_valid) or
        (self.rcp_scc is not None and not self.corner_objects_available and not self.rcp_scc.can_valid) or
        (self.rcp_corner_objects is not None and not self.rcp_corner_objects.can_valid) or
        (self.rcp_corner_objects_180 is not None and not self.rcp_corner_objects_180.can_valid) or
        (self.rcp_corner_objects_430 is not None and not self.rcp_corner_objects_430.can_valid)):
      ret.errors.canError = True
    ret.points = [point for point in self.pts.values() if point.measured]
    return ret

  def _update(self, updated_messages):

    t_id = 32
    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):

      msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]
      track_state = 0
      optional_track_stale = (addr >= self.radar_start_addr + self.radar_required_msg_count and
                              addr not in updated_messages)

      if self.radar_group1:
        valid = msg['VALID_CNT1'] > 10
      elif self.radar_group3:
        # Group 3 marks an empty object slot with LONG_DIST raw 0x7ff (204.7 m).
        valid = msg['LONG_DIST'] < 204.7
      elif self.canfd:
        valid, track_state = canfd_group2_track_status(msg)
      elif self.radar_group4:
        # EN: DNMWR006 exposes eight stable tracked-object slots at 0x500-0x507.
        #     Messages from 0x508 onward are distance-sorted raw detections without
        #     stable IDs, so they are excluded. OBJECT_STATE 3 is a confirmed track;
        #     empty slots use LONG_DIST raw 0xfff8 (409.55 m). Driving logs reached
        #     317.80 m, so 325 m preserves every observed confirmed track while
        #     retaining margin from the empty-slot sentinel. Keep the +/-6 m
        #     ego/adjacent-lane envelope to suppress farther roadside reflections.
        # KO: DNMWR006의 안정적인 추적 객체 슬롯은 0x500~0x507의 8개임.
        #     0x508 이후 메시지는 고정 ID가 없는 거리순 raw detection이므로 제외함.
        #     OBJECT_STATE 3은 확정 추적 객체이며, 빈 슬롯은 LONG_DIST raw
        #     0xfff8(409.55m)을 사용함. 주행 로그의 최대값은 317.80m였으므로
        #     325m 상한으로 관측된 확정 트랙을 모두 보존하면서 빈 슬롯 값과 충분한
        #     여유를 확보함. 원거리 도로변 반사를 줄이기 위해 좌우 6m 범위를 유지함.
        valid = (msg['OBJECT_STATE'] == 3 and 0.2 < msg['LONG_DIST'] < RADAR_GROUP4_MAX_LONG_DIST and
                 abs(msg['LAT_DIST']) <= RADAR_GROUP4_MAX_YREL)
      else:
        valid = msg['STATE'] in (3, 4)

      # Optional slots do not participate in CAN validity. Therefore explicitly
      # require a fresh frame in each radar cycle instead of retaining a valid
      # object from the last cycle if the upper bank stops transmitting.
      valid = valid and not optional_track_stale

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
        self.pts[t_id].trackState = track_state
      elif self.radar_group4:
        self.pts[t_id].dRel = msg['LONG_DIST']
        self.pts[t_id].yRel = -msg['LAT_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0.0
      else:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[t_id].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[t_id].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = msg['REL_ACCEL']
        self.pts[t_id].yvRel = 0.0

      t_id += 1
    # Radar group 1 carries two messages per object.
    if self.radar_group1:
      for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):
        msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]

        optional_track_stale = (addr >= self.radar_start_addr + self.radar_required_msg_count and
                                addr not in updated_messages)
        valid = msg['VALID_CNT2'] > 10 and not optional_track_stale
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

    candidates = []
    for slot, addr in enumerate(range(CORNER_OBJECT_235_START_ADDR, CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT)):
      t_id = CORNER_OBJECT_235_TRACK_ID_OFFSET + slot
      msg = self.rcp_corner_objects.vl[f"CORNER_RADAR_235_OBJECTS_{addr:x}"]

      d_rel = msg["OBJ_REL_POS_X"]
      y_rel = msg["OBJ_REL_POS_Y"]
      v_rel = msg["OBJ_REL_VEL_X"]
      yv_rel = msg["OBJ_REL_VEL_Y"]
      a_rel = msg["OBJ_REL_ACCEL_X"]
      # Side objects are clipped to x=0 by the corner radar. Quality, identity,
      # and lateral motion still describe a real object, so keep them for
      # corner-confirmed front-radar association in radard.
      valid = msg["OBJ_QUAL_LEVEL"] > 0 and corner_object_position_valid(d_rel, y_rel) and v_rel > -99.0

      if not valid:
        continue
      candidates.append((t_id, int(msg["OBJ_OBJECT_ID"]), int(msg["OBJ_AGE"]), int(msg["OBJ_QUAL_LEVEL"]),
                         d_rel, y_rel, v_rel, yv_rel, a_rel))

    self._apply_corner_objects("corner235", candidates,
                               range(CORNER_OBJECT_235_TRACK_ID_OFFSET,
                                     CORNER_OBJECT_235_TRACK_ID_OFFSET + CORNER_OBJECT_235_MSG_COUNT))

  def _update_corner_objects_180(self, updated_messages):
    if self.rcp_corner_objects_180 is None:
      return

    if not updated_messages:
      self._clear_corner_objects_180()
      return

    candidates = []
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
        valid = msg[f"{prefix}QUAL_LEVEL"] > 0 and corner_object_position_valid(d_rel, y_rel) and v_rel > -99.0

        if not valid:
          continue
        candidates.append((t_id, int(msg[f"{prefix}OBJECT_ID"]), int(msg[f"{prefix}AGE"]), int(msg[f"{prefix}QUAL_LEVEL"]),
                           d_rel, y_rel, v_rel, yv_rel, a_rel))

    self._apply_corner_objects("corner180", candidates,
                               range(CORNER_OBJECT_180_TRACK_ID_OFFSET,
                                     CORNER_OBJECT_180_TRACK_ID_OFFSET + CORNER_OBJECT_180_MSG_COUNT * CORNER_OBJECT_180_SLOTS_PER_MSG))

  def _apply_corner_objects(self, source, candidates, slot_ids):
    for t_id in slot_ids:
      self._clear_point(t_id)

    # The same object can occupy two CAN slots for one cycle during a slot handoff.
    # Only merge physically close copies. The radar can assign one object ID to
    # two distant objects concurrently, so object ID alone is not an identity.
    objects = deduplicate_corner_candidates(candidates)
    track_ids = self.corner_object_track_ids.get_track_ids(source, objects)

    for t_id, _, _, _, d_rel, y_rel, v_rel, yv_rel, a_rel in objects:
      point = self.pts[t_id]
      point.measured = True
      point.trackId = track_ids[t_id]
      point.radarSource = source
      point.dRel = d_rel
      point.yRel = y_rel
      point.vRel = v_rel
      point.vLead = v_rel + self.v_ego
      point.aRel = a_rel
      point.yvRel = yv_rel

  def _update_corner_objects_430(self, updated_messages):
    if self.rcp_corner_objects_430 is None:
      return

    if not updated_messages:
      self._clear_corner_objects_430()
      return

    bank_defs = (
      (CORNER_OBJECT_430_LEFT_START_ADDR, 1.0, 0),
      (CORNER_OBJECT_430_RIGHT_START_ADDR, -1.0, CORNER_OBJECT_430_MSG_COUNT_PER_SIDE * CORNER_OBJECT_430_SLOTS_PER_MSG),
    )
    for start_addr, side_sign, track_base in bank_defs:
      bins = []
      for msg_index, addr in enumerate(range(start_addr, start_addr + CORNER_OBJECT_430_MSG_COUNT_PER_SIDE)):
        msg = self.rcp_corner_objects_430.vl[f"CORNER_RADAR_430_OBJECTS_{addr:x}"]
        for slot_index in range(CORNER_OBJECT_430_SLOTS_PER_MSG):
          prefix = f"SLOT{slot_index + 1}_"
          distance_raw = int(msg[f"{prefix}DISTANCE_RAW"])
          raw = (
            distance_raw |
            (int(msg[f"{prefix}META_13_15"]) << 13) |
            (int(msg[f"{prefix}META_BYTE_2"]) << 16) |
            (int(msg[f"{prefix}META_BYTE_3"]) << 24)
          )
          code = (
            int(msg[f"{prefix}META_13_15"]),
            int(msg[f"{prefix}META_BYTE_2"]),
            int(msg[f"{prefix}META_BYTE_3"]),
          )
          d_rel = distance_raw * 0.05
          default_distance = CORNER_OBJECT_430_DEFAULT_DISTANCE_RAW_MIN <= distance_raw <= CORNER_OBJECT_430_DEFAULT_DISTANCE_RAW_MAX
          base_valid = (
            raw not in CORNER_OBJECT_430_EMPTY_RAW_VALUES and
            distance_raw not in (0, 8000, 8191) and
            not default_distance and
            0.2 < d_rel < CORNER_OBJECT_430_MAX_DREL
          )
          candidate_valid = (
            base_valid and
            slot_index + 1 not in CORNER_OBJECT_430_CANDIDATE_EXCLUDED_SLOTS and
            code[2] in CORNER_OBJECT_430_CANDIDATE_META_BYTE_3 and
            code[1] in CORNER_OBJECT_430_STRONG_META_BYTE_2 + CORNER_OBJECT_430_WEAK_META_BYTE_2
          )
          bins.append({
            "msg_index": msg_index,
            "slot_index": slot_index,
            "distance_raw": distance_raw,
            "d_rel": d_rel,
            "code": code,
            "candidate_valid": candidate_valid,
          })

      supported_bins = []
      candidates = [b for b in bins if b["candidate_valid"]]
      for b in candidates:
        support = 1
        for other in candidates:
          if other is b:
            continue
          if abs(other["msg_index"] - b["msg_index"]) > 1:
            continue
          if abs(other["slot_index"] - b["slot_index"]) > 2:
            continue
          if abs(other["distance_raw"] - b["distance_raw"]) > CORNER_OBJECT_430_CANDIDATE_RAW_DELTA:
            continue
          support += 1
        min_support = (CORNER_OBJECT_430_STRONG_MIN_SUPPORT if b["code"][1] in CORNER_OBJECT_430_STRONG_META_BYTE_2
                       else CORNER_OBJECT_430_WEAK_MIN_SUPPORT)
        if support >= min_support:
          supported_bins.append({**b, "support": support})

      clusters = []
      for b in sorted(supported_bins, key=lambda item: item["distance_raw"]):
        if not clusters or b["distance_raw"] - clusters[-1][-1]["distance_raw"] > CORNER_OBJECT_430_CLUSTER_RAW_GAP:
          clusters.append([b])
        else:
          clusters[-1].append(b)
      clusters = sorted(clusters, key=lambda cluster: sum(b["distance_raw"] for b in cluster) / len(cluster))[:CORNER_OBJECT_430_MAX_TRACKS_PER_SIDE]

      cluster_objects = []
      for cluster in clusters:
        msg_index = sum(b["msg_index"] for b in cluster) / len(cluster)
        slot = sum(b["slot_index"] + 1 for b in cluster) / len(cluster)
        lateral_cell = (CORNER_OBJECT_430_LATERAL_CELL_MSG_WEIGHT * msg_index +
                        CORNER_OBJECT_430_LATERAL_CELL_SLOT_WEIGHT * slot)
        mapped_cell = lateral_cell if side_sign > 0.0 else CORNER_OBJECT_430_RIGHT_CELL_MIRROR - lateral_cell
        y_abs = max(CORNER_OBJECT_430_MIN_ABS_YREL,
                    min(CORNER_OBJECT_430_MAX_ABS_YREL,
                        CORNER_OBJECT_430_YREL_OFFSET - CORNER_OBJECT_430_YREL_SCALE * mapped_cell))
        cluster_objects.append({
          "d_rel": sum(b["d_rel"] for b in cluster) / len(cluster),
          "y_rel": side_sign * y_abs,
          "code": max((b["code"] for b in cluster), key=lambda code: sum(1 for item in cluster if item["code"] == code)),
        })

      active_t_ids = set()
      side_track_ids = [
        CORNER_OBJECT_430_TRACK_ID_OFFSET + track_base + slot
        for slot in range(CORNER_OBJECT_430_MAX_TRACKS_PER_SIDE)
      ]
      unmatched_track_ids = {t_id for t_id in side_track_ids if t_id in self.corner_object_430_prev_d_rel}
      unused_track_ids = [t_id for t_id in side_track_ids if t_id not in unmatched_track_ids]

      for cluster in cluster_objects:
        d_rel = cluster["d_rel"]
        code = cluster["code"]
        matched_t_id = None
        if unmatched_track_ids:
          nearest_t_id = min(unmatched_track_ids, key=lambda t_id: abs(d_rel - self.corner_object_430_prev_d_rel[t_id]))
          if abs(d_rel - self.corner_object_430_prev_d_rel[nearest_t_id]) <= CORNER_OBJECT_430_TRACK_MATCH_MAX_DREL_DELTA:
            matched_t_id = nearest_t_id
            unmatched_track_ids.remove(matched_t_id)
        if matched_t_id is None and unused_track_ids:
          matched_t_id = unused_track_ids.pop(0)
        if matched_t_id is None:
          continue

        t_id = matched_t_id
        active_t_ids.add(t_id)
        prev_d_rel = self.corner_object_430_prev_d_rel.get(t_id)
        prev_code = self.corner_object_430_prev_code.get(t_id)
        self.corner_object_430_prev_d_rel[t_id] = d_rel
        self.corner_object_430_prev_y_rel[t_id] = cluster["y_rel"]
        self.corner_object_430_prev_code[t_id] = code
        reset_track = prev_d_rel is None or code != prev_code or abs(d_rel - prev_d_rel) > CORNER_OBJECT_430_MAX_DREL_DELTA
        if reset_track:
          self.corner_object_430_prev_v_rel.pop(t_id, None)
          self.corner_object_430_prev_yv_rel.pop(t_id, None)
          self.corner_object_430_history.pop(t_id, None)
          self.corner_object_430_noncenter_inward_frames.pop(t_id, None)

        history = self.corner_object_430_history.setdefault(t_id, deque(maxlen=CORNER_OBJECT_430_HISTORY_SIZE))
        history.append((d_rel, cluster["y_rel"]))
        if len(history) < CORNER_OBJECT_430_MIN_HISTORY:
          self._clear_point(t_id)
          continue

        window_dt = CORNER_OBJECT_430_DT * (len(history) - 1)
        first_d_rel, first_y_rel = history[0]
        hist_v_rel = (d_rel - first_d_rel) / window_dt
        if abs(hist_v_rel) > CORNER_OBJECT_430_MAX_ABS_VREL:
          self.corner_object_430_prev_v_rel.pop(t_id, None)
          self.corner_object_430_prev_yv_rel.pop(t_id, None)
          self.corner_object_430_history.pop(t_id, None)
          self.corner_object_430_noncenter_inward_frames.pop(t_id, None)
          self._clear_point(t_id)
          continue
        prev_v_rel = self.corner_object_430_prev_v_rel.get(t_id, hist_v_rel)
        v_rel = (1.0 - CORNER_OBJECT_430_VREL_ALPHA) * prev_v_rel + CORNER_OBJECT_430_VREL_ALPHA * hist_v_rel
        self.corner_object_430_prev_v_rel[t_id] = v_rel

        inward_steps = 0
        usable_steps = 0
        prev_abs_y = abs(history[0][1])
        for _, y_rel in list(history)[1:]:
          abs_y = abs(y_rel)
          delta = prev_abs_y - abs_y
          if abs(delta) > 1e-3:
            usable_steps += 1
            if delta > 0.0:
              inward_steps += 1
          prev_abs_y = abs_y
        net_inward_y = abs(first_y_rel) - abs(cluster["y_rel"])
        inward_ratio = inward_steps / usable_steps if usable_steps > 0 else 0.0
        hist_yv_rel = (cluster["y_rel"] - first_y_rel) / window_dt
        recent_inward_y = abs(history[-3][1]) - abs(cluster["y_rel"]) if len(history) >= 3 else net_inward_y
        if (net_inward_y < CORNER_OBJECT_430_MIN_INWARD_YREL_DELTA or
            recent_inward_y < CORNER_OBJECT_430_MIN_RECENT_INWARD_YREL_DELTA or
            inward_ratio < CORNER_OBJECT_430_MIN_INWARD_RATIO or
            abs(hist_yv_rel) > CORNER_OBJECT_430_MAX_ABS_YVREL):
          hist_yv_rel = 0.0
        inward_motion_candidate = hist_yv_rel != 0.0 and abs(cluster["y_rel"]) <= CORNER_OBJECT_430_INWARD_KEEP_YVREL_ABS_YREL
        inward_center_candidate = inward_motion_candidate and abs(cluster["y_rel"]) <= CORNER_OBJECT_430_INWARD_CENTER_ABS_YREL
        y_rel = cluster["y_rel"]
        if inward_motion_candidate:
          if inward_center_candidate:
            self.corner_object_430_noncenter_inward_frames[t_id] = 0
            prev_yv_rel = self.corner_object_430_prev_yv_rel.get(t_id, hist_yv_rel)
            yv_rel = (1.0 - CORNER_OBJECT_430_YVREL_ALPHA) * prev_yv_rel + CORNER_OBJECT_430_YVREL_ALPHA * hist_yv_rel
          else:
            noncenter_frames = self.corner_object_430_noncenter_inward_frames.get(t_id, 0) + 1
            self.corner_object_430_noncenter_inward_frames[t_id] = noncenter_frames
            if noncenter_frames <= CORNER_OBJECT_430_EARLY_INWARD_NONCENTER_FRAMES:
              prev_yv_rel = self.corner_object_430_prev_yv_rel.get(t_id, hist_yv_rel)
              yv_rel = (1.0 - CORNER_OBJECT_430_YVREL_ALPHA) * prev_yv_rel + CORNER_OBJECT_430_YVREL_ALPHA * hist_yv_rel
            else:
              yv_rel = 0.0
          if not inward_center_candidate and abs(y_rel) < CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL:
            y_rel = math.copysign(CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL, y_rel)
        else:
          hist_yv_rel = 0.0
          yv_rel = 0.0
          self.corner_object_430_noncenter_inward_frames[t_id] = 0
          if abs(y_rel) < CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL:
            y_rel = math.copysign(CORNER_OBJECT_430_SIDE_KEEP_ABS_YREL, y_rel)
        self.corner_object_430_prev_yv_rel[t_id] = yv_rel

        self.pts[t_id].measured = True
        self.pts[t_id].trackId = t_id
        self.pts[t_id].dRel = d_rel
        self.pts[t_id].yRel = y_rel
        self.pts[t_id].vRel = v_rel
        self.pts[t_id].vLead = v_rel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = yv_rel

      side_track_count = CORNER_OBJECT_430_MSG_COUNT_PER_SIDE * CORNER_OBJECT_430_SLOTS_PER_MSG
      for slot in range(side_track_count):
        t_id = CORNER_OBJECT_430_TRACK_ID_OFFSET + track_base + slot
        if t_id in active_t_ids:
          continue
        self.corner_object_430_prev_d_rel.pop(t_id, None)
        self.corner_object_430_prev_v_rel.pop(t_id, None)
        self.corner_object_430_prev_y_rel.pop(t_id, None)
        self.corner_object_430_prev_yv_rel.pop(t_id, None)
        self.corner_object_430_prev_code.pop(t_id, None)
        self.corner_object_430_history.pop(t_id, None)
        self.corner_object_430_noncenter_inward_frames.pop(t_id, None)
        self._clear_point(t_id)


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
    self.corner_object_track_ids.clear_source("corner235")

  def _clear_corner_objects_180(self):
    for slot in range(CORNER_OBJECT_180_MSG_COUNT * CORNER_OBJECT_180_SLOTS_PER_MSG):
      self._clear_point(CORNER_OBJECT_180_TRACK_ID_OFFSET + slot)
    self.corner_object_track_ids.clear_source("corner180")

  def _clear_corner_objects_430(self):
    self.corner_object_430_prev_d_rel.clear()
    self.corner_object_430_prev_v_rel.clear()
    self.corner_object_430_prev_y_rel.clear()
    self.corner_object_430_prev_yv_rel.clear()
    self.corner_object_430_prev_code.clear()
    self.corner_object_430_history.clear()
    self.corner_object_430_noncenter_inward_frames.clear()
    for slot in range(CORNER_OBJECT_430_MSG_COUNT_PER_SIDE * 2 * CORNER_OBJECT_430_SLOTS_PER_MSG):
      self._clear_point(CORNER_OBJECT_430_TRACK_ID_OFFSET + slot)

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
