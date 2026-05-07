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
  outer_lane_prob: float = 0.0
  
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
    self.outer_lane_prob = float(lane_outer_prob)

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
                     radar_obj,
                     blindspot: bool,
                     ignore_bsd: bool,
                     bsd_hold_sec: float = 2.0):
    # 옆 차선 앞 차량 간격 판단 (개선)
    if radar_obj is not None and radar_obj.status:
        d_rel = radar_obj.dRel          # 현재 거리 (m)
        v_lead = radar_obj.vLead        # 앞 차 절대속도 (m/s)
        v_rel = v_lead - v_ego          # 상대속도 (음수 = 접근 중)

        # 충돌 예상 시간 (TTC): 접근 중일 때만 계산
        if v_rel < -0.5:
            ttc = d_rel / (-v_rel)      # 양수 값 (초)
        else:
            ttc = 99.0                  # 멀어지거나 동속이면 위험 없음

        # 간격 기준: 아래 두 조건 중 하나라도 해당하면 위험
        # 1) 절대 거리 기준: 현재 거리가 자차 속도 * 2.5초 이내 (최소 15m)
        safe_dist = max(v_ego * 2.5, 15.0)
        too_close = d_rel < safe_dist

        # 2) TTC 기준: 충돌 예상 시간이 5초 이내
        ttc_danger = ttc < 5.0

        object_detected = too_close or ttc_danger
    else:
        object_detected = False

    if object_detected:
        self.object_detected_count = max(1, self.object_detected_count + 1)
    else:
        self.object_detected_count = min(-1, self.object_detected_count - 1)

    self.side_object_detected = self.object_detected_count > int(-0.3 / DT_MDL)

    # BSD hold (기존 유지)
    self.bsd_detected_now = bool(blindspot)
    if self.bsd_detected_now and not ignore_bsd:
        self.bsd_hold_counter = int(bsd_hold_sec / DT_MDL)
    else:
        self.bsd_hold_counter = max(0, self.bsd_hold_counter - 1)


  def compute_lane_change_available(self, lane_line_info_lt_20: bool, ignore_bsd: bool):
    # lane_available은 거리 기반이므로 중앙선 너머 반대차선을 잘못 감지할 수 있음
    # outer_lane_prob >= 0.5 조건을 추가해 모델이 실제로 차선을 인식할 때만 허용
    lane_avail_confirmed = self.lane_available and (self.outer_lane_prob >= 0.5)

    self.lane_change_available_geom = (
        (lane_avail_confirmed or self.edge_available) and lane_line_info_lt_20
    )

    bsd_active = (self.bsd_hold_counter > 0) and (not ignore_bsd)
    self.lane_change_available = (
        self.lane_change_available_geom
        and (not self.side_object_detected)
        and (not bsd_active)
    )
    
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
