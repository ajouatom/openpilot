from dataclasses import dataclass, field
from collections import deque

import numpy as np
from openpilot.common.realtime import DT_MDL

from openpilot.selfdrive.controls.lib.cutin_helpers import is_corner_track_id, is_stable_corner_track_id

from .lane_math import calculate_lane_width
from .hysteresis import ExistCounter


SIDE_LEAD_CLOSE_DREL = 5.0
SIDE_LEAD_MIN_GAP_BASE = 6.0
SIDE_LEAD_MIN_GAP_TIME = 0.30
SIDE_LEAD_MIN_GAP_MAX = 12.0
SIDE_LEAD_PROJECT_SEC = 1.5
SIDE_LEAD_TTC_MIN_CLOSING = 0.5
SIDE_LEAD_TTC_NEAR_MARGIN = 6.0

# A vehicle that just left the blind spot can release the conservative BSD hold
# only when the same close side-radar track is consistently pulling away.
SIDE_BSD_RECEDING_MIN_VREL = 3.0
SIDE_BSD_RECEDING_START_MAX_DREL = 6.0
SIDE_BSD_RECEDING_CLEAR_MIN_DREL = 5.0
SIDE_BSD_RECEDING_MIN_DREL_GAIN = 1.0
SIDE_BSD_RECEDING_CONFIRM_SEC = 0.25
SIDE_BSD_RECEDING_MAX_DREL_STEP_BACK = 0.25


@dataclass
class SideState:
  name: str  # "left" / "right"

  # lane/edge distances
  lane_width: float = 0.0
  lane_width_diff: float = 0.0
  dist_to_edge: float = 0.0
  dist_to_edge_far: float = 0.0

  # current lane prob (ego lane line prob on that side)
  cur_prob: float = 1.0
  current_lane_missing: bool = False

  # counters
  lane_exist_count: ExistCounter = field(default_factory=lambda: ExistCounter(0.2))
  lane_width_count: ExistCounter = field(default_factory=lambda: ExistCounter(0.2))
  edge_count: ExistCounter = field(default_factory=lambda: ExistCounter(0.2))

  # availability
  lane_available: bool = False
  edge_available: bool = False

  # smoothing
  lane_width_queue: deque = field(default_factory=lambda: deque(maxlen=int(1.0 / DT_MDL)))

  # lane line info
  lane_line_info_raw: int = 0
  lane_line_info_mod: int = 0
  last_lane_line_mod: int = 0
  lane_line_info_edge_detect: bool = False

  # transitions
  lane_available_last: bool = False
  edge_available_last: bool = False
  lane_available_trigger: bool = False
  lane_appeared: bool = False

  # obstacles
  object_detected_count: int = 0
  side_object_detected: bool = False

  # BSD hold (after detection)
  bsd_hold_counter: int = 0
  bsd_detected_now: bool = False
  bsd_receding_track_id: int = -1
  bsd_receding_frames: int = 0
  bsd_receding_start_d_rel: float = 0.0
  bsd_receding_last_d_rel: float = 0.0

  # computed lane change availability (includes BSD+object)
  lane_change_available_geom: bool = False
  lane_change_available: bool = False
  lane_change_available_last: bool = False
  lane_change_available_released: bool = False
  lane_width_sum: float = 0.0

  def update_lane_geometry(self,
                           lane_outer, lane_outer_prob,
                           lane_current,
                           road_edge,
                           cur_prob: float):
    lane_w, dist_edge, dist_edge_far, lane_valid = calculate_lane_width(
      lane_outer, lane_outer_prob, lane_current, road_edge
    )

    self.lane_exist_count.update(bool(lane_valid))

    # running mean (O(1))
    if len(self.lane_width_queue) == self.lane_width_queue.maxlen:
      self.lane_width_sum -= self.lane_width_queue.popleft()
    self.lane_width_queue.append(lane_w)
    self.lane_width_sum += lane_w
    self.lane_width = self.lane_width_sum / len(self.lane_width_queue)

    self.lane_width_diff = (self.lane_width_queue[-1] - self.lane_width_queue[0]) if len(self.lane_width_queue) >= 2 else 0.0

    self.dist_to_edge = float(dist_edge)
    self.dist_to_edge_far = float(dist_edge_far)

    min_lane_width = 2.5
    self.lane_width_count.update(self.lane_width > min_lane_width)
    self.edge_count.update(self.dist_to_edge > min_lane_width)

    available_count = int(0.2 / DT_MDL)
    self.lane_available = self.lane_width_count.counter > available_count
    self.edge_available = (self.edge_count.counter > available_count) and (self.dist_to_edge_far > min_lane_width)

    self.cur_prob = float(cur_prob)
    self.current_lane_missing = self.cur_prob < 0.3

  def update_lane_line_info(self, lane_line_info_raw: int):
    self.lane_line_info_raw = int(lane_line_info_raw)
    mod = self.lane_line_info_raw % 10
    # edge_detect: true on the transition into an allowed 0/5 lane-line type.
    self.lane_line_info_edge_detect = (mod in (0, 5)) and (self.last_lane_line_mod not in (0, 5))
    self.last_lane_line_mod = mod
    self.lane_line_info_mod = mod

  def update_obstacles(self,
                       v_ego: float,
                       radar_obj,           # radarState.leadLeft / leadRight
                       blindspot: bool,      # carstate.leftBlindspot/rightBlindspot
                       ignore_bsd: bool,
                       bsd_hold_sec: float = 2.0,
                       radar_objects=()):
    radar_objects = tuple(radar_objects)
    corner_objects = tuple(obj for obj in radar_objects if self._is_corner_radar_object(obj))
    primary_object_detected = self._side_lead_is_unsafe(v_ego, radar_obj)
    object_detected = primary_object_detected or any(
      self._side_lead_is_unsafe(v_ego, obj) for obj in corner_objects
    )
    if object_detected:
      self.object_detected_count = max(1, self.object_detected_count + 1)
    else:
      self.object_detected_count = min(-1, self.object_detected_count - 1)

    self.side_object_detected = self.object_detected_count > int(-0.3 / DT_MDL)

    # Hold BSD detection for a short period after it is observed.
    self.bsd_detected_now = bool(blindspot)
    if self.bsd_detected_now and not ignore_bsd:
      self.bsd_hold_counter = int(bsd_hold_sec / DT_MDL)
      self._reset_bsd_receding_track()
    elif not ignore_bsd:
      self.bsd_hold_counter = max(0, self.bsd_hold_counter - 1)
      if (
        self.bsd_hold_counter > 0
        and self._bsd_receding_release_ready(v_ego, corner_objects)
      ):
        self.bsd_hold_counter = 0
        # The receding-track confirmation is stronger than the ordinary
        # side-object decay. Allow the release immediately when the primary
        # object is also clear.
        if not primary_object_detected:
          self.object_detected_count = int(-0.3 / DT_MDL)
          self.side_object_detected = False
    else:
      self.bsd_hold_counter = 0
      self._reset_bsd_receding_track()

  def _reset_bsd_receding_track(self):
    self.bsd_receding_track_id = -1
    self.bsd_receding_frames = 0
    self.bsd_receding_start_d_rel = 0.0
    self.bsd_receding_last_d_rel = 0.0

  @classmethod
  def _is_corner_radar_object(cls, radar_obj) -> bool:
    if not bool(getattr(radar_obj, "status", False)):
      return False
    track_id = int(cls._lead_float(radar_obj, "radarTrackId", -1.0))
    return is_corner_track_id(track_id) or is_stable_corner_track_id(track_id)

  def _bsd_receding_release_ready(self, v_ego: float, radar_objects) -> bool:
    candidates = []
    for radar_obj in radar_objects:
      if not bool(getattr(radar_obj, "status", False)):
        continue
      track_id = int(self._lead_float(radar_obj, "radarTrackId", -1.0))
      d_rel = self._lead_float(radar_obj, "dRel", 255.0)
      v_rel = self._lead_float(
        radar_obj,
        "vRel",
        self._lead_float(radar_obj, "vLead", v_ego) - v_ego,
      )
      continuing = track_id == self.bsd_receding_track_id
      if (
        track_id >= 0
        and v_rel >= SIDE_BSD_RECEDING_MIN_VREL
        and 0.1 < d_rel < 160.0
        and (continuing or d_rel <= SIDE_BSD_RECEDING_START_MAX_DREL)
      ):
        candidates.append((not continuing, d_rel, track_id, radar_obj))

    if not candidates:
      self._reset_bsd_receding_track()
      return False

    _, d_rel, track_id, selected = min(candidates, key=lambda candidate: candidate[:3])
    # Never use one receding vehicle to clear another unsafe side object.
    unsafe_other = any(
      int(self._lead_float(radar_obj, "radarTrackId", -1.0)) != track_id
      and self._side_lead_is_unsafe(v_ego, radar_obj)
      for radar_obj in radar_objects
    )
    distance_continuous = (
      track_id == self.bsd_receding_track_id
      and d_rel
      >= self.bsd_receding_last_d_rel
      - SIDE_BSD_RECEDING_MAX_DREL_STEP_BACK
    )
    if unsafe_other or not distance_continuous:
      self.bsd_receding_track_id = track_id
      self.bsd_receding_frames = 1
      self.bsd_receding_start_d_rel = d_rel
    else:
      self.bsd_receding_frames += 1
    self.bsd_receding_last_d_rel = d_rel

    required_frames = 1 + int(SIDE_BSD_RECEDING_CONFIRM_SEC / DT_MDL)
    return (
      not unsafe_other
      and not self._side_lead_is_unsafe(v_ego, selected)
      and self.bsd_receding_frames >= required_frames
      and d_rel >= SIDE_BSD_RECEDING_CLEAR_MIN_DREL
      and d_rel - self.bsd_receding_start_d_rel
      >= SIDE_BSD_RECEDING_MIN_DREL_GAIN
    )

  @staticmethod
  def _lead_float(radar_obj, name: str, default: float = 0.0) -> float:
    try:
      return float(getattr(radar_obj, name))
    except (AttributeError, TypeError, ValueError):
      return default

  def _side_lead_is_unsafe(self, v_ego: float, radar_obj) -> bool:
    if radar_obj is None or not radar_obj.status:
      return False

    d_rel = self._lead_float(radar_obj, "dRel", 255.0)
    if not 0.1 < d_rel < 160.0:
      return False

    v_lead = self._lead_float(radar_obj, "vLead", v_ego)
    v_rel = self._lead_float(radar_obj, "vRel", v_lead - v_ego)
    v_ego = max(0.0, float(v_ego))

    if d_rel <= SIDE_LEAD_CLOSE_DREL:
      return True

    min_gap = float(np.clip(v_ego * SIDE_LEAD_MIN_GAP_TIME,
                            SIDE_LEAD_MIN_GAP_BASE,
                            SIDE_LEAD_MIN_GAP_MAX))

    # Slower side-lane vehicles matter most when ego is closing on them.
    closing_speed = max(0.0, -v_rel)
    if closing_speed > SIDE_LEAD_TTC_MIN_CLOSING:
      ttc = d_rel / closing_speed
      ttc_limit = float(np.interp(v_ego, [0.0, 15.0, 30.0], [2.0, 3.0, 3.5]))
      near_enough = d_rel < max(min_gap + SIDE_LEAD_TTC_NEAR_MARGIN, v_ego * 1.2)
      if near_enough and ttc < ttc_limit:
        return True

      projected_gap = d_rel + v_rel * SIDE_LEAD_PROJECT_SEC
      if projected_gap < min_gap:
        return True

    # Same-speed adjacent vehicles still need a basic speed-scaled gap.
    return d_rel < min_gap and v_rel < 1.0

  def compute_lane_change_available(self, lane_line_info_lt_20: bool, ignore_bsd: bool):
    # geometric availability
    self.lane_change_available_geom = (self.lane_available or self.edge_available) and lane_line_info_lt_20

    # Include BSD/object checks in final lane-change availability.
    bsd_active = (self.bsd_hold_counter > 0) and (not ignore_bsd)
    self.lane_change_available = self.lane_change_available_geom and (not self.side_object_detected) and (not bsd_active)
    self.lane_change_available_released = self.lane_change_available and not self.lane_change_available_last

  def update_triggers(self):
    # Preserve the existing lane_available_trigger behavior.
    self.lane_available_trigger = False
    if self.lane_width_diff > 0.8 and (self.lane_width < self.dist_to_edge):
      self.lane_available_trigger = True

    # Use >= so the appeared state is not missed after the threshold.
    # Limit distant edges so intersections do not over-trigger lane_appeared.
    appeared_now = self.lane_exist_count.counter >= int(0.2 / DT_MDL)
    self.lane_appeared = (self.lane_appeared or appeared_now) and (self.dist_to_edge < 4.0)

  def commit_last(self):
    self.lane_available_last = self.lane_available
    self.edge_available_last = self.edge_available
    self.lane_change_available_last = self.lane_change_available
