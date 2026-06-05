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

  # ── 차선 변경 가능 여부 (용도별 3종) ──────────────────────────
  # lane_change_available_geom  : 기하학적 조건만 (차선 폭·도로 경계)
  # lane_change_available       : 기하학 + 장애물 없음 + BSD 없음 (완전 가능)
  # lane_change_available_no_bsd: 기하학 + 장애물 없음 (BSD 무시, 토크 override용)
  # lane_change_available_hold  : laneChangeStarting 취소 판단용
  #                               True→즉시, False→0.3초 후 전환 (비대칭 히스테리시스)
  lane_change_available_geom:   bool = False
  lane_change_available:        bool = False
  lane_change_available_no_bsd: bool = False
  lane_change_available_hold:   bool = False
  avail_false_count:            int  = 0

  # ── 코너 레이더 활성 여부 ──────────────────────────────────────
  corner_radar_active: bool = False

  # ── 측전방 코너 레이더 상태 ────────────────────────────────────
  front_prev:     float = 0.0   # 이전 프레임 거리 (m)
  front_approach: float = 0.0   # EMA 접근 속도 (m/s), 양수 = 가까워지는 중
  front_miss:     int   = 0     # 연속 미감지 프레임 수

  # ── 측후방 코너 레이더 상태 ────────────────────────────────────
  rear_prev:     float = 0.0
  rear_approach: float = 0.0
  rear_miss:     int   = 0

  # ── 상수 (인스턴스마다 바뀌지 않음) ───────────────────────────
  _MISS_RESET_FRAMES: int   = field(default=5,    init=False, repr=False, compare=False)
  _APPR_RAW_LIMIT:   float  = field(default=25.0, init=False, repr=False, compare=False)
  _APPR_EMA_ALPHA:   float  = field(default=0.6,  init=False, repr=False, compare=False)
  _APPR_MIN_REF:     float  = field(default=0.10, init=False, repr=False, compare=False)
  _APPR_MAX_REF:     float  = field(default=8.0, init=False, repr=False, compare=False)


  # ════════════════════════════════════════════════════════════════
  #  차선 / 도로 경계 업데이트
  # ════════════════════════════════════════════════════════════════

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
    # edge_detect: 0/5�� �ٲ�� ���� (������ ��/�찡 ���� self.lane_line_info ������ ���׼�)
    self.lane_line_info_edge_detect = (mod in (0, 5)) and (self.last_lane_line_mod not in (0, 5))
    self.last_lane_line_mod = mod
    self.lane_line_info_mod = mod

  def update_obstacles(self,
                       v_ego: float,
                       radar_obj,           # radarState.leadLeft / leadRight
                       blindspot: bool,      # carstate.leftBlindspot/rightBlindspot
                       ignore_bsd: bool,
                       bsd_hold_sec: float = 2.0):
    # object_detected (radar ���)
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

    # BSD hold (�䱸����: ���� �� 2�� ����)
    self.bsd_detected_now = bool(blindspot)
    if self.bsd_detected_now and not ignore_bsd:
      self.bsd_hold_counter = int(bsd_hold_sec / DT_MDL)
    else:
      self.bsd_hold_counter = max(0, self.bsd_hold_counter - 1)

  def compute_lane_change_available(self, lane_line_info_lt_20: bool, ignore_bsd: bool):
    # geometric availability
    self.lane_change_available_geom = (self.lane_available or self.edge_available) and lane_line_info_lt_20

    # include bsd/object into lane_change_available (�䱸����)
    bsd_active = (self.bsd_hold_counter > 0) and (not ignore_bsd)
    self.lane_change_available = self.lane_change_available_geom and (not self.side_object_detected) and (not bsd_active)

  def update_triggers(self):
    # lane_available_trigger (���� ���� ����)
    self.lane_available_trigger = False
    if self.lane_width_diff > 0.8 and (self.lane_width < self.dist_to_edge):
      self.lane_available_trigger = True

    # lane_appeared (bugfix: == ���� >=�� �ڿ�������)
    # + edge�� �ʹ� �ָ�(������) lane_appeared�� �����ϰ� true�� ������ �ʰ� ����
    appeared_now = self.lane_exist_count.counter >= int(0.2 / DT_MDL)
    self.lane_appeared = (self.lane_appeared or appeared_now) and (self.dist_to_edge < 4.0)

  def commit_last(self):
    self.lane_available_last        = self.lane_available
    self.edge_available_last        = self.edge_available
    self.lane_change_available_last = self.lane_change_available


  # ════════════════════════════════════════════════════════════════
  #  내부 감지 로직
  # ════════════════════════════════════════════════════════════════

  def _radar_block(self, radar_obj, v_ego: float, gap: float) -> bool:
    """leadLeft / leadRight 레이더 트랙 기반 차단 판단."""
    if radar_obj is None or not radar_obj.status:
      return False

    d_rel  = float(radar_obj.dRel)
    v_lead = float(radar_obj.vLead)
    v_rel  = v_lead - v_ego
    v_ref  = max(v_ego, 1.0)
    hw_now = d_rel / v_ref

    approach_hw = float(np.interp(gap, [1.0, 6.0], [0.55, 1.80]))
    recede_hw   = float(np.interp(gap, [1.0, 6.0], [0.40, 1.30]))
    rear_hw     = float(np.interp(gap, [1.0, 6.0], [0.35, 1.10]))

    if d_rel > 0 and v_rel < 0:
      hw_block = hw_now < approach_hw
    elif d_rel > 0 and v_rel >= 0:
      hw_block = hw_now < recede_hw
    else:
      hw_block = hw_now < rear_hw

    # TTC
    approach_speed = -v_rel
    if d_rel > 0 and approach_speed > 0.5:
      ttc_th    = float(np.interp(gap, [1.0, 6.0], [2.0, 6.0]))
      ttc_block = (d_rel / approach_speed) < ttc_th
    else:
      ttc_block = False

    # 미래 예측 headway
    T_LOOK   = float(np.interp(gap, [1.0, 6.0], [1.5, 3.5]))
    d_future = d_rel + v_rel * T_LOOK
    if d_future > 0:
      hwf_th       = float(np.interp(gap, [1.0, 6.0], [0.45, 1.40]))
      future_block = (d_future / v_ref) < hwf_th
    else:
      future_block = True

    return hw_block or ttc_block or future_block


  def _corner_block_front(self, d_cur: float, d_lat: float,
                          v_ego: float, gap: float) -> bool:
    if d_cur <= 0:
      self.front_miss += 1
      if self.front_miss >= self._MISS_RESET_FRAMES:
        self.front_prev     = 0.0
        self.front_approach = 0.0
        self.front_miss     = 0
      return False

    self.front_miss = 0

    # ── 접근 속도 EMA
    if self.front_prev > 0:
      raw = (self.front_prev - d_cur) / DT_MDL
      if abs(raw) < self._APPR_RAW_LIMIT:
        self.front_approach = (
          (1.0 - self._APPR_EMA_ALPHA) * self.front_approach +
          self._APPR_EMA_ALPHA * raw
        )
    else:
      first_detect_th = float(np.interp(gap, [1.0, 6.0], [15.0, 30.0]))
      self.front_approach = self._APPR_MIN_REF * 2.0 if d_cur < first_detect_th else 0.0

    self.front_prev = d_cur

    appr_norm = float(np.clip(self.front_approach, 0.0, self._APPR_MAX_REF))

    # ── 거리 임계 보간
    dist_th_min = float(np.interp(gap, [1.0, 6.0], [6.0,  12.0]))
    dist_th_max = float(np.interp(gap, [1.0, 6.0], [15.0, 30.0]))
    dist_th     = float(np.interp(appr_norm,
                                  [self._APPR_MIN_REF, self._APPR_MAX_REF],
                                  [dist_th_min, dist_th_max]))

    # ── 미래 위치 예측 차단 (접근 중일 때 더 멀리서 감지)
    # 예측 시간(T_predict)만큼 후의 거리를 계산해서 dist_th 이내면 차단
    # 가까워질수록 T_predict를 늘려 더 보수적으로 동작
    future_block = False
    if self.front_approach > self._APPR_MIN_REF:
      T_predict   = float(np.interp(gap, [1.0, 6.0], [2.0, 4.0]))
      d_future    = d_cur - self.front_approach * T_predict
      future_block = d_future < dist_th  # 미래에 동적 임계 이내로 들어오면 차단

    # ── TTC 임계 보간
    ttc_th_min = float(np.interp(gap, [1.0, 6.0], [1.5, 3.5]))
    ttc_th     = float(np.interp(appr_norm,
                                [self._APPR_MIN_REF, self._APPR_MAX_REF],
                                [ttc_th_min, ttc_th_min * 1.5]))

    dist_block   = d_cur < dist_th
    ttc_block    = (self.front_approach > self._APPR_MIN_REF) and \
                  (d_cur / max(self.front_approach, 0.1)) < ttc_th

    safety_th    = float(np.interp(gap, [1.0, 6.0], [5.0, 8.0]))
    safety_block = d_cur < safety_th

    # ── 횡거리 보강
    lat_block = False
    if d_lat > 0:
      lat_th      = float(np.interp(gap, [1.0, 6.0], [3.0,  4.5]))
      long_lat_th = float(np.interp(gap, [1.0, 6.0], [12.0, 24.0]))
      lat_block   = (d_lat < lat_th) and (d_cur < long_lat_th)

    return dist_block or ttc_block or safety_block or future_block or lat_block


  def _corner_block_rear(self, d_cur: float,
                        v_ego: float, gap: float) -> bool:
    if d_cur <= 0:
      self.rear_miss += 1
      if self.rear_miss >= self._MISS_RESET_FRAMES:
        self.rear_prev     = 0.0
        self.rear_approach = 0.0
        self.rear_miss     = 0
      return False

    self.rear_miss = 0

    # ── 접근 속도 EMA
    if self.rear_prev > 0:
      raw = (self.rear_prev - d_cur) / DT_MDL
      if abs(raw) < self._APPR_RAW_LIMIT:
        self.rear_approach = (
          (1.0 - self._APPR_EMA_ALPHA) * self.rear_approach +
          self._APPR_EMA_ALPHA * raw
        )
    else:
      first_detect_th = float(np.interp(gap, [1.0, 6.0], [25.0, 50.0]))
      self.rear_approach = self._APPR_MIN_REF * 2.0 if d_cur < first_detect_th else 0.0

    self.rear_prev = d_cur

    appr_norm = float(np.clip(self.rear_approach, 0.0, self._APPR_MAX_REF))

    # ── 거리 임계 보간
    dist_th_min = float(np.interp(gap, [1.0, 6.0], [9.0,  17.0]))
    dist_th_max = float(np.interp(gap, [1.0, 6.0], [22.0, 48.0]))
    dist_th     = float(np.interp(appr_norm,
                                  [self._APPR_MIN_REF, self._APPR_MAX_REF],
                                  [dist_th_min, dist_th_max]))

    # ── 미래 위치 예측 차단 (후방은 더 긴 예측 시간 적용)
    future_block = False
    if self.rear_approach > self._APPR_MIN_REF:
      T_predict    = float(np.interp(gap, [1.0, 6.0], [3.0, 6.0]))
      d_future     = d_cur - self.rear_approach * T_predict
      future_block = d_future < dist_th  # 미래에 동적 임계 이내로 들어오면 차단

    # ── TTC 임계 보간
    ttc_th_min = float(np.interp(gap, [1.0, 6.0], [3.0, 6.0]))
    ttc_th     = float(np.interp(appr_norm,
                                [self._APPR_MIN_REF, self._APPR_MAX_REF],
                                [ttc_th_min, ttc_th_min * 1.5]))

    dist_block   = d_cur < dist_th
    ttc_block    = (self.rear_approach > self._APPR_MIN_REF) and \
                  (d_cur / max(self.rear_approach, 0.1)) < ttc_th

    safety_th    = float(np.interp(gap, [1.0, 6.0], [4.0, 7.0]))
    safety_block = d_cur < safety_th

    return dist_block or ttc_block or safety_block or future_block

