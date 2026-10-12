import bisect
import copy
import math
import os
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from numbers import Integral, Real
from typing import Sequence

import numpy as np

from opendbc import DBC_PATH
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.radar_lead_filter import RadarLeadFilter
from opendbc.car.hyundai.values import DBC, HyundaiFlags, HyundaiExtFlags
from openpilot.common.params import Params
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_group3 import Group3Object, Group3TrackIds

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

from opendbc.car.hyundai.radar_bosch import (  # noqa: E402
  BOSCH_INACTIVE_WORD,
  BOSCH_MAX_RAW_TRACK_ID,
  BOSCH_TRACK_ADDRESSES,
  BOSCH_WINDOW_NS,
  BOSCH_STALE_NS,
  BOSCH_OUTPUT_INTERVAL_NS,
  BOSCH_SAMPLE_HOLD_NS,
  BOSCH_ALEAD_SAMPLE_PERIOD_S,
  BOSCH_ALEAD_STATE_MAX,
  BOSCH_B5_OFF,
  BOSCH_B5_ACTIVE,
  BOSCH_B5_MODE,
  BOSCH_B5_PARENT_STABILITY_SCANS,
  BOSCH_B5_COUNTER_MAX,
  BOSCH_B5_CONTEXT_MAX_AGE_NS,
  BOSCH_B5_HISTORY_NS,
  BOSCH_B5_HISTORY_MIN_SPAN_NS,
  BOSCH_B5_MOTION_HISTORY_NS,
  BOSCH_B5_RADAR_TO_DEVICE_X_M,
  BOSCH_B5_OUTSIDE_MARGIN_M,
  BOSCH_B5_PARENT_MIN_AGE_SCANS,
  BOSCH_B5_MIN_WORLD_SPEED_MPS,
  BOSCH_B5_MAX_D_M,
  BOSCH_B5_MAX_Y_M,
  BOSCH_B5_MAX_DV_MPS,
  BOSCH_B5_MAX_DWORLD_MPS,
  BOSCH_B5_MIN_VEGO_MPS,
  BOSCH_B5_MAX_CURVATURE_1PM,
  BOSCH_B5_EDGE_STD_MAX_M,
  BOSCH_B5_EDGE_WIDTH_MIN_M,
  BOSCH_B5_EDGE_WIDTH_MAX_M,
  BOSCH_B5_LANE_PROB_MIN,
  BOSCH_B5_LANE_STD_MAX_M,
  BOSCH_B5_LANE_STABILITY_MAX_M,
  BOSCH_B5_CORRIDOR_WIDTH_MIN_M,
  BOSCH_B5_CORRIDOR_WIDTH_MAX_M,
  BOSCH_B5_EDGE_LANE_BAND_MAX_M,
  BOSCH_B5_DIVERGENCE_MIN_M,
  BOSCH_P91_OFF,
  BOSCH_P91_SHADOW,
  BOSCH_P91_ACTIVE,
  BOSCH_P91_ACTIVE_TEST,
  BOSCH_P91_MODE,
  BOSCH_P91_SUPPORT_HOLD_NS,
  BOSCH_FAMILY_COMPANION_OFF,
  BOSCH_FAMILY_COMPANION_SHADOW,
  BOSCH_FAMILY_COMPANION_ACTIVE,
  BOSCH_FAMILY_COMPANION_MODE,
  BOSCH_FAMILY_MIN_ANCHOR_AGE_SCANS,
  BOSCH_FAMILY_MIN_D_M,
  BOSCH_FAMILY_MAX_D_M,
  BOSCH_FAMILY_MAX_Y_M,
  BOSCH_FAMILY_MAX_DV_MPS,
  BOSCH_FAMILY_MAX_WORLD_SPEED_MPS,
  BOSCH_FAMILY_MAX_D_STEP_M,
  BOSCH_FAMILY_MAX_Y_STEP_M,
  BOSCH_FAMILY_MIN_WORLD_SPEED_MPS,
  BOSCH_FAMILY_MAX_GAP_NS,
  BOSCH_FAMILY_INWARD_SCANS,
  BOSCH_FAMILY_INWARD_MPS,
  BOSCH_BURST_MULTIRETURN_OFF,
  BOSCH_BURST_MULTIRETURN_SHADOW,
  BOSCH_BURST_MULTIRETURN_ACTIVE,
  BOSCH_BURST_MULTIRETURN_MODE,
  BOSCH_BURST_MIN_MEMBERS,
  BOSCH_BURST_MAX_WORLD_SPAN_MPS,
  BOSCH_BURST_MAX_D_SPAN_M,
  BOSCH_BURST_MIN_Y_SPAN_M,
  BOSCH_BURST_MIN_WORLD_SPEED_MPS,
  BOSCH_BURST_MIN_VEGO_MPS,
  BOSCH_BURST_NEIGHBOUR_AGE_SCANS,
  BOSCH_BURST_NEIGHBOUR_D_M,
  BOSCH_BURST_HOLD_MAX_SCANS,
  BOSCH_BURST_MAX_MISSING_SCANS,
  BOSCH_BURST_MAX_GAP_NS,
  BOSCH_BURST_STATE_MAX,
  BOSCH_SIDEPASS_LATERAL_OFF,
  BOSCH_SIDEPASS_LATERAL_SHADOW,
  BOSCH_SIDEPASS_LATERAL_ACTIVE,
  BOSCH_SIDEPASS_LATERAL_MODE,
  BOSCH_SIDEPASS_WINDOW_NS,
  BOSCH_SIDEPASS_MIN_SCANS,
  BOSCH_SIDEPASS_MIN_SPAN_NS,
  BOSCH_SIDEPASS_SLIDE_ARM_MPS,
  BOSCH_SIDEPASS_SLIDE_RELEASE_MPS,
  BOSCH_SIDEPASS_RELEASE_SCANS,
  BOSCH_SIDEPASS_CLOSING_MPS,
  BOSCH_SIDEPASS_MIN_LEAD_SPEED_MPS,
  BOSCH_SIDEPASS_CORRIDOR_M,
  BOSCH_SIDEPASS_ZONE_M,
  BOSCH_SIDEPASS_MAX_D_M,
  BOSCH_SIDEPASS_MAX_OFFSET_M,
  BOSCH_SIDEPASS_REALIGN_MPS,
  BOSCH_SIDEPASS_MAX_GAP_NS,
  BOSCH_SIDEPASS_STATE_MAX,
  BOSCH_OEM_STATE_NONE,
  BOSCH_OEM_STATE_TENTATIVE,
  BOSCH_OEM_STATE_SELECTED,
  BOSCH_OEM_STATE_VALIDATED,
  BOSCH_OEM_GATE_OFF,
  BOSCH_OEM_GATE_SHADOW,
  BOSCH_OEM_GATE_ACTIVE,
  BOSCH_OEM_GATE_MODE,
  BOSCH_OEM_EVIDENCE_HOLD_NS,
  BOSCH_OEM_INTENT_MIN_MPS2,
  BOSCH_OEM_INTENT_MAX_MPS2,
  BOSCH_OEM_INTENT_FRESH_NS,
  BOSCH_OEM_INTENT_WINDOW_NS,
  BOSCH_OEM_INTENT_MIN_SAMPLES,
  BOSCH_OEM_GATE_NEUTRAL_MPS2,
  BOSCH_OEM_GATE_PERSIST_SCANS,
  BOSCH_OEM_GATE_MIN_SPEED_MPS,
  BOSCH_OEM_GATE_MIN_RANGE_M,
  BOSCH_OEM_GATE_MIN_TTC_S,
  BOSCH_OEM_GATE_CUTIN_MPS,
  BOSCH_OEM_GATE_CUTIN_SCANS,
  BOSCH_OEM_GATE_LATERAL_WINDOW_NS,
  BOSCH_OEM_GATE_IN_PATH_M,
  BOSCH_OEM_GATE_SETTLED_SCANS,
  BOSCH_OEM_GATE_SIBLING_D_M,
  BOSCH_OEM_GATE_SIBLING_Y_M,
  BOSCH_OEM_GATE_SIBLING_V_MPS,
  BOSCH_SCC_BUS,
  BOSCH_SCC11_ADDR,
  BOSCH_SCC12_ADDR,
  BOSCH_SCC11_OBJ_VALID_BIT,
  BOSCH_SCC12_AREQ_RAW_BIT,
  BOSCH_SCC_ADDRESSES,
  BOSCH_SCC_STALE_NS,
  BOSCH_CAMERA_EXTENDED_OFF,
  BOSCH_CAMERA_EXTENDED_SHADOW,
  BOSCH_CAMERA_EXTENDED_ACTIVE,
  BOSCH_CAMERA_EXTENDED_PUBLICATION,
  BOSCH_CAMERA_EXTENDED_ACTIVE_TEST,
  BOSCH_CAMERA_EXTENDED_TEST_INTERVALS,
  BOSCH_CAMERA_MATURITY_HOLD_NS,
  BOSCH_CAMERA_MATURITY_WORD1_INTERVALS,
  BOSCH_CAMERA_EXTENDED_MODE,
  BOSCH_CAMERA_HEADER,
  BOSCH_CAMERA_FIRST_OBJECT,
  BOSCH_CAMERA_LAST_OBJECT,
  BOSCH_CAMERA_LAST_FAMILY,
  BOSCH_CAMERA_SLOTS,
  BOSCH_CAMERA_PERIOD_NS,
  BOSCH_CAMERA_FRAME_TOLERANCE_NS,
  BOSCH_CAMERA_OBSERVATION_GAP_NS,
  BOSCH_CAMERA_E2_HOLD_NS,
  BOSCH_CAMERA_SNAPSHOT_COUNT,
  BOSCH_CAMERA_WIDTH_REF_M,
  BOSCH_CAMERA_RANGE_LSB,
  BOSCH_CAMERA_LONG_BASE_M,
  BOSCH_CAMERA_LONG_RANGE_K,
  BOSCH_CAMERA_LONG_POS_WIDTH_K,
  BOSCH_CAMERA_LONG_NEG_WIDTH_K,
  BOSCH_CAMERA_LONG_MAX_M,
  BOSCH_CAMERA_LAT_MAX_M,
  BOSCH_CAMERA_ANGLE_LSB,
  BOSCH_CAMERA_ASSOC_UNRESOLVED,
  BOSCH_CAMERA_ASSOC_ASSIGNED,
  BOSCH_CAMERA_ASSOC_AMBIGUOUS,
  BOSCH_CAMERA_CURVE_REACQUIRE_OFF,
  BOSCH_CAMERA_CURVE_REACQUIRE_SHADOW,
  BOSCH_CAMERA_CURVE_REACQUIRE_ACTIVE,
  BOSCH_CAMERA_CURVE_REACQUIRE_MODE,
  BOSCH_CAMERA_CURVE_REACQUIRE_HOLD_NS,
  BOSCH_CAMERA_CURVE_REACQUIRE_MIN_D_M,
  BOSCH_CAMERA_CURVE_REACQUIRE_MAX_D_M,
  BOSCH_CAMERA_CURVE_REACQUIRE_MIN_YAW_RATE,
  BOSCH_CAMERA_CURVE_REACQUIRE_LAT_MAX_M,
  BOSCH_CAMERA_CURVE_REACQUIRE_BEAR_MAX_RAD,
  BOSCH_CAMERA_CURVE_REACQUIRE_STATE_MAX,
  BOSCH_TRUCK_P2_CLASS,
  BOSCH_TRUCK_P2_CONFIRMATIONS,
  BOSCH_TRUCK_P2_STATE_MAX,
  BOSCH_TRUCK_P2_WIDTH_MIN_M,
  BOSCH_TRUCK_P2_DD_MIN_M,
  BOSCH_TRUCK_P2_DD_MAX_M,
  BOSCH_TRUCK_P2_DY_MAX_M,
  BOSCH_TRUCK_P2_DV_MAX_MPS,
  BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS,
  BOSCH_TRUCK_A0_RECOVERY_BEARING_EXCESS_RAD,
  BOSCH_TRUCK_A0_RECOVERY_COST_MARGIN,
  BOSCH_COMPANION_DEFER_OFF,
  BOSCH_COMPANION_DEFER_ACTIVE,
  BOSCH_COMPANION_DEFER_MODE,
  BOSCH_COMPANION_DEFER_CONFIRMATIONS,
  BOSCH_COMPANION_DEFER_HOLD_NS,
  BOSCH_COMPANION_DEFER_STATE_MAX,
  BOSCH_COMPANION_DEFER_DD_MIN_M,
  BOSCH_COMPANION_DEFER_DD_MAX_M,
  BOSCH_COMPANION_DEFER_DY_MAX_M,
  BOSCH_COMPANION_DEFER_DV_MAX_MPS,
  BOSCH_COMPANION_DEFER_Y_MAX_M,
  BOSCH_COMMON_ANCESTRY_MAX_AGE_NS,
  BOSCH_COMMON_ANCESTRY_RAW_PAIR_MAX,
  BOSCH_COMPANION_DEFER_WIDTH_DD_M,
  BOSCH_OEM_NEARER_PUBLICATION_OFF,
  BOSCH_OEM_NEARER_PUBLICATION_ACTIVE,
  BOSCH_OEM_NEARER_PUBLICATION_MODE,
  _bosch_camera_signed,
  _bosch_camera_range_span,
  _bosch_camera_long_window,
  BoschCameraObject,
  BoschCameraCycleCache,
  _BoschExtendedHistory,
  _BoschCurveReacquireHistory,
  _BoschTruckPairHistory,
  _BoschExtendedRepresentative,
  BoschCameraExtendedGrouping,
  bosch_numpy_linear_sum_assignment,
  _bosch_integer,
  _bosch_finite,
  BoschRawDetection,
  BoschRawTrackingConfig,
  BoschRawTrack,
  BoschRawAssociationDecision,
  _BoschRawState,
  _bosch_advance,
  _bosch_admissible_cycle,
  _bosch_rectangular_unique,
  _bosch_component_matching,
  _bosch_raw_unique_component,
  _bosch_raw_assignment,
  BoschRawTrackManager,
  BoschGroupingConfig,
  BoschModelSnapshot,
  BoschVisionCue,
  BoschPublishedSurface,
  BoschPhysicalObject,
  BoschAssociationDecision,
  _BoschPairEvidence,
  _BoschCommonGroupAncestry,
  _BoschPhysicalState,
  _BoschProvisionalBundleState,
  BoschProvisionalBundleDecision,
  _bosch_group_representative,
  _bosch_physical_component,
  _bosch_physical_assignment,
  BoschObjectGroupManager,
  BoschPhysicalTracker,
  _BoschPublicationPassThrough,
  _BoschFamilyCompanionState,
  BoschFamilyCompanionDecision,
  _BoschFamilyCompanionFilter,
  _BoschBurstMultiReturnState,
  BoschBurstMultiReturnDecision,
  _BoschBurstMultiReturnDefer,
  _BoschSidePassLateralState,
  BoschSidePassLateralDecision,
  _BoschSidePassLateralEstimator,
  _BoschP91PairState,
  _BoschPersistentSpatialCloneFilter,
  _BoschOemGateState,
  _BoschOemIntentTracker,
  _BoschOemValidationGate,
  BOSCH_PUBLICATION_ALIAS_START,
  BOSCH_PUBLICATION_ALIAS_COUNT,
  BOSCH_PUBLICATION_ALIAS_GRACE_S,
  BoschPublicationAliasAllocator,
  BoschB5ModelContext,
  _BoschB5ParentState,
  BoschB5Decision,
  BoschBirthB5Defer,
  bosch_published_surface,
  BoschLeadAccelerationState,
  BoschLeadAccelerationEstimator,
  bosch_fill_point,
  bosch_append_points,
  bosch_to_native_radar_data,
  _BoschCanFrame,
  bosch_decode_frame,
  _bosch_match_processed_target_word,
  bosch_make_points,
  BoschRadarProvider,
)




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
    self.radar_track_mode = self.params.get_int("EnableRadarTracks")
    self.radar_tracks = self.radar_track_mode >= 1
    self.bosch = None
    self._bosch_objects = ()
    self._bosch_now_ns = 0
    self._bosch_context = None
    self._bosch_path_ns = None
    self._bosch_path_source_ns = 0
    self._bosch_path = ()
    self._bosch_last_pose = (None, 0)
    self._bosch_last_model = (None, 0)
    if self.radar_tracks and CP.extFlags & HyundaiExtFlags.BOSCH_RADAR:
      CAN = CanBus(CP)
      bus = CAN.ACAN if CP.extFlags & HyundaiExtFlags.BOSCH_RADAR_BUS1 else CAN.CAM
      # Same bus the stock SCC parser uses, so the gate reads the factory
      # module rather than openpilot's own SCC11/SCC12 transmissions.
      scc_bus = CAN.CAM if CP.flags & HyundaiFlags.CAMERA_SCC else CAN.ECAN
      self.bosch = BoschRadarProvider(bus, camera_bus=CAN.ACAN, scc_bus=scc_bus,
                                      camera_extended_mode=BOSCH_CAMERA_EXTENDED_MODE)
      self._bosch_lead_acceleration = BoschLeadAccelerationEstimator()
      self._bosch_make_points = bosch_make_points
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
      CP, self.radar_tracks and self.bosch is None, self.radar_start_addr, self.radar_msg_count,
      self.radar_required_msg_count, self.radar_group4,
    )
    self.rcp_corner_objects = get_corner_object_can_parser(CP, self.corner_object_tracks)
    self.rcp_corner_objects_180 = get_corner_object_180_can_parser(CP, self.corner_object_180_tracks)
    self.rcp_corner_objects_430 = get_corner_object_430_can_parser(CP, self.corner_object_430_tracks)
    # Enabling raw radar tracks on legacy CAN disables the stock SCC11 stream on
    # some Hyundai/Kia platforms. Camera-SCC cars may still use SCC11.
    use_scc_parser = self.bosch is not None or not (self.radar_tracks and not self.canfd and not (CP.flags & HyundaiFlags.CAMERA_SCC))
    self.rcp_scc = get_radar_can_parser_scc(CP) if use_scc_parser else None
    self.trigger_msg_scc = 416 if self.canfd else 0x420

    self.trigger_msg_tracks = self.radar_start_addr + self.radar_required_msg_count - 1
    self.trigger_msg_corner_objects = CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT - 1
    self.trigger_msg_corner_objects_180 = CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT - 1
    self.trigger_msg_corner_objects_430 = CORNER_OBJECT_430_RIGHT_START_ADDR + CORNER_OBJECT_430_MSG_COUNT_PER_SIDE - 1
    self.track_id = 0

    self.corner_objects_available = self.rcp_corner_objects is not None or self.rcp_corner_objects_180 is not None or self.rcp_corner_objects_430 is not None
    self.radar_off_can = CP.radarUnavailable and not self.corner_objects_available and self.bosch is None
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

    self.group3_track_ids = Group3TrackIds()

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

  def set_bosch_context(self, now_ns, pose=None, pose_ns=0, model=None, model_ns=0):
    """Optional receive context; Device yaw is right-positive, radar y is left."""
    if self.bosch is None:
      return
    # A message stamped after now_ns (IPC/scheduling order) is not current for
    # this batch yet. Keep the previously accepted one instead of dropping the
    # context; the newer message is taken on the first call that reaches its
    # stamp. Otherwise a few ms of delivery latency decides whether road-edge,
    # path and yaw context exist for a scan.
    last_pose, last_model = getattr(self, '_bosch_last_pose', (None, 0)), getattr(self, '_bosch_last_model', (None, 0))
    if pose is not None and pose_ns > now_ns:
      pose, pose_ns = last_pose
    else:
      self._bosch_last_pose = (pose, pose_ns)
    if model is not None and model_ns > now_ns:
      model, model_ns = last_model
    else:
      self._bosch_last_model = (model, model_ns)
    yaw = None
    if pose is not None and 0 <= now_ns - pose_ns <= 200_000_000:
      # pose is an immutable message reused at 100 Hz; read its fields once.
      cached = getattr(self, '_bosch_pose_yaw', None)
      if cached is None or cached[0] is not pose:
        angular = getattr(pose, 'angularVelocityDevice', None)
        pose_yaw = None
        if pose.inputsOK and pose.sensorsOK and angular is not None and angular.valid and math.isfinite(angular.z):
          pose_yaw = -float(angular.z)
        cached = self._bosch_pose_yaw = (pose, pose_yaw)
      yaw = cached[1]
    self.bosch.b5_birth_defer.ingest_pose(int(pose_ns), yaw)
    cues = ()
    path = ()
    source_ns = 0
    if model is not None and 0 <= now_ns - model_ns <= 200_000_000:
      # One plain snapshot per message, keyed like the path cache below by its
      # stamp (and object). Keeping two covers a held-back newer message
      # (model_ns > now_ns above) alternating with the accepted one.
      snapshots = getattr(self, '_bosch_model_snapshots', ())
      snapshot = next((snap for source, ns, snap in snapshots if source is model and ns == model_ns), None)
      if snapshot is None:
        snapshot = BoschModelSnapshot(model)
        self._bosch_model_snapshots = snapshots[-1:] + ((model, model_ns, snapshot),)
      self.bosch.road_edge_filter.ingest_model(snapshot, int(model_ns))
      self.bosch.b5_birth_defer.ingest_model(snapshot, int(model_ns))
      self.bosch.mirror_m3_shadow.ingest_model(snapshot, int(model_ns))
      cues = snapshot.cues
      model = snapshot
      if self._bosch_path_ns != model_ns:
        position = getattr(model, 'position', None)
        self._bosch_path = tuple(zip(position.x, position.y)) if position is not None else ()
        # The camera frame the model consumed, not the time it finished.
        self._bosch_path_source_ns = int(getattr(model, 'timestampEof', 0) or 0)
        self._bosch_path_ns = model_ns
      path = self._bosch_path
      source_ns = self._bosch_path_source_ns
    else:
      self.bosch.road_edge_filter.invalidate_model()
    self._bosch_context = (int(now_ns), yaw, cues, path, model_ns, source_ns)

  def update_carrot(self, v_ego, a_ego, rcv_time, can_packets):
    # Keep the legacy SCC/corner MyTrack processing intact. Bosch objects are
    # deliberately outside self.pts: MyTrack synthesizes lateral velocity and
    # acceleration, which have not been decoded for this sensor.
    ret = super().update_carrot(v_ego, a_ego, rcv_time, can_packets)
    if ret is not None and self.bosch is not None:
      scan_ns = self.bosch.last_scan_timestamp_ns
      objects = (self._bosch_objects if scan_ns is not None and 0 <= self._bosch_now_ns - scan_ns <= BOSCH_SAMPLE_HOLD_NS
                 else ())
      alias = self.bosch.publication_aliases.update(
        self._bosch_now_ns, (obj.physical_track_id for obj in objects), self.bosch.tracker.group_manager.states)
      # Some replay/test harnesses construct RadarInterface without __init__.
      # Keep the estimator Bosch-local and initialize it on first use there too.
      if not hasattr(self, '_bosch_lead_acceleration'):
        self._bosch_lead_acceleration = BoschLeadAccelerationEstimator()
      if self.bosch._debug_timeout:
        self._bosch_lead_acceleration.reset('PROVIDER_TIMEOUT')
        a_lead_by_pid = {}
      else:
        a_lead_by_pid = self._bosch_lead_acceleration.update(
          objects, scan_ns, self.v_ego, self.bosch.tracker.group_manager.states)
      objects = self.bosch.publication_view(objects, self._bosch_now_ns)
      bosch_append_points(ret, objects, self.v_ego, self._bosch_now_ns, alias, a_lead_by_pid)
    return ret

  def update(self, can_strings):
    self.frame += 1
    if self.radar_off_can or (self.bosch is None and self.rcp_tracks is None and self.rcp_scc is None and self.rcp_corner_objects is None and self.rcp_corner_objects_180 is None and self.rcp_corner_objects_430 is None):
      return super().update(None)

    if self.rcp_scc is not None:
      vls_s = self.rcp_scc.update(can_strings)
      self.updated_scc.update(vls_s)

    track_ready = False
    if self.bosch is not None:
      now_ns, yaw, cues, path, path_ns, path_source_ns = (
        self._bosch_context or (time.monotonic_ns(), None, (), (), None, 0))
      self._bosch_now_ns = now_ns
      self._bosch_context = None
      self.bosch.b5_birth_defer.ingest_speed(now_ns, self.v_ego)
      objects = self.bosch.update(can_strings, now_ns=now_ns, v_ego=self.v_ego,
                                  yaw_rate_left=yaw, vision=cues, path=path, path_ns=path_ns,
                                  path_source_ns=path_source_ns)
      if objects is not None:
        self._bosch_objects = objects
        track_ready = True
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

    scc_ready = (not self.radar_tracks or self.bosch is not None) and self.frame % 5 == 0 and self.rcp_scc is not None

    if track_ready and self.rcp_tracks is not None:
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
    bosch_front_only = self.bosch is not None and self.radar_track_mode == 1
    publish_ready = scc_ready if self.bosch is not None else track_ready or scc_ready
    if bosch_front_only:
      # Mode 1 consumes frontRadar; optional SCC reception cannot gate its clock.
      publish_ready = self.frame % 5 == 0
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
    if self.bosch is not None:
      ret.errors.canError = self.bosch.can_error
      ret.errors.wrongConfig = self.bosch.wrong_config
    if ((self.rcp_tracks is not None and self.radar_tracks and not self.rcp_tracks.can_valid) or
        (self.rcp_scc is not None and not bosch_front_only and not self.corner_objects_available and not self.rcp_scc.can_valid) or
        (self.rcp_corner_objects is not None and not self.rcp_corner_objects.can_valid) or
        (self.rcp_corner_objects_180 is not None and not self.rcp_corner_objects_180.can_valid) or
        (self.rcp_corner_objects_430 is not None and not self.rcp_corner_objects_430.can_valid)):
      ret.errors.canError = True
    ret.points = [point for point in self.pts.values() if point.measured]
    return ret

  def _update(self, updated_messages):
    if self.radar_group3:
      objects = {
        addr: Group3Object.from_signals(self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"])
        for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count)
        if addr in updated_messages
      }
      assignments = self.group3_track_ids.update(objects)
      # Remove old slot entries, including migrated IDs. An unmeasured copy in
      # the previous slot would otherwise reset the surviving object's filter.
      for slot in range(32, 32 + self.radar_msg_count):
        self.pts.pop(slot, None)
      for addr, track_id in assignments.items():
        obj = objects[addr]
        point = structs.RadarData.RadarPoint()
        point.trackId, point.radarSource, point.measured = track_id, "frontRadar", True
        point.dRel, point.yRel, point.vRel = obj.d_rel, obj.y, obj.v
        point.vLead, point.aRel, point.yvRel = self.v_ego + obj.v, float("nan"), 0.0
        self.pts[32 + addr - self.radar_start_addr] = point
      return

    t_id = 32
    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):

      msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]
      track_state = 0
      optional_track_stale = (addr >= self.radar_start_addr + self.radar_required_msg_count and
                              addr not in updated_messages)

      if self.radar_group1:
        valid = msg['VALID_CNT1'] > 10
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
        self.pts[t_id].dRel = msg['LONG_DIST']
        self.pts[t_id].yRel = msg['LAT_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = msg['REL_ACCEL']
        self.pts[t_id].yvRel = msg['LAT_SPEED']
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
