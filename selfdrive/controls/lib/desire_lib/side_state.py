from dataclasses import dataclass, field
from collections import deque
from typing import Optional

import numpy as np
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV

from .lane_math import calculate_lane_width
from .hysteresis import ExistCounter


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

  # computed “lane change available” (includes BSD+object)
  lane_change_available_geom: bool = False
  lane_change_available: bool = False
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
    # edge_detect: 0/5로 바뀌는 순간 (기존은 좌/우가 같은 self.lane_line_info 공유라 버그성)
    self.lane_line_info_edge_detect = (mod in (0, 5)) and (self.last_lane_line_mod not in (0, 5))
    self.last_lane_line_mod = mod
    self.lane_line_info_mod = mod

  def update_obstacles(self,
                       v_ego: float,
                       radar_obj,           # radarState.leadLeft / leadRight
                       blindspot: bool,      # carstate.leftBlindspot/rightBlindspot
                       ignore_bsd: bool,
                       bsd_hold_sec: float = 2.0):
    # object_detected (radar 기반)
    if radar_obj is not None and radar_obj.status:
      d = radar_obj.dRel
      v = radar_obj.vLead
      side_object_dist = d + v * 4.0
    else:
      side_object_dist = 255.0

    object_detected = side_object_dist < (v_ego * 3.0)
    if object_detected:
      self.object_detected_count = max(1, self.object_detected_count + 1)
    else:
      self.object_detected_count = min(-1, self.object_detected_count - 1)

    self.side_object_detected = self.object_detected_count > int(-0.3 / DT_MDL)

    # BSD hold (요구사항: 검출 후 2초 유지)
    self.bsd_detected_now = bool(blindspot)
    if self.bsd_detected_now and not ignore_bsd:
      self.bsd_hold_counter = int(bsd_hold_sec / DT_MDL)
    else:
      self.bsd_hold_counter = max(0, self.bsd_hold_counter - 1)

  def compute_lane_change_available(self, lane_line_info_lt_20: bool, ignore_bsd: bool):
    # geometric availability
    self.lane_change_available_geom = (self.lane_available or self.edge_available) and lane_line_info_lt_20

    # include bsd/object into lane_change_available (요구사항)
    bsd_active = (self.bsd_hold_counter > 0) and (not ignore_bsd)
    self.lane_change_available = self.lane_change_available_geom and (not self.side_object_detected) and (not bsd_active)

  def update_triggers(self):
    # lane_available_trigger (기존 로직 유지)
    self.lane_available_trigger = False
    if self.lane_width_diff > 0.8 and (self.lane_width < self.dist_to_edge):
      self.lane_available_trigger = True

    # lane_appeared (bugfix: == 말고 >=가 자연스러움)
    # + edge가 너무 멀면(교차로) lane_appeared를 과도하게 true로 만들지 않게 제한
    appeared_now = self.lane_exist_count.counter >= int(0.2 / DT_MDL)
    self.lane_appeared = (self.lane_appeared or appeared_now) and (self.dist_to_edge < 4.0)

  def commit_last(self):
    self.lane_available_last = self.lane_available
    self.edge_available_last = self.edge_available
