from dataclasses import dataclass, field
from collections import deque
import numpy as np
from openpilot.common.realtime import DT_MDL

from .lane_math import calculate_lane_width
from .hysteresis import ExistCounter


@dataclass
class SideState:
  name: str  # "left" / "right"

  # ── 차선/도로 경계 거리 ────────────────────────────────────────
  lane_width: float = 0.0
  lane_width_diff: float = 0.0
  dist_to_edge: float = 0.0
  dist_to_edge_far: float = 0.0

  # ── 자차 차선 확률 ─────────────────────────────────────────────
  cur_prob: float = 1.0
  current_lane_missing: bool = False

  # ── 히스테리시스 카운터 ────────────────────────────────────────
  lane_exist_count: ExistCounter = field(default_factory=lambda: ExistCounter(0.2))
  lane_width_count: ExistCounter = field(default_factory=lambda: ExistCounter(0.2))
  edge_count:       ExistCounter = field(default_factory=lambda: ExistCounter(0.2))

  # ── 차선/경계 가용 여부 ────────────────────────────────────────
  lane_available:            bool = False
  edge_available:            bool = False
  lane_change_available_last: bool = False

  # ── 차선 폭 스무딩 ─────────────────────────────────────────────
  lane_width_queue: deque = field(default_factory=lambda: deque(maxlen=int(1.0 / DT_MDL)))
  lane_width_sum:   float = 0.0

  # ── 차선선 정보 ────────────────────────────────────────────────
  lane_line_info_raw:        int   = 0
  lane_line_info_mod:        int   = 0
  last_lane_line_mod:        int   = 0
  lane_line_info_edge_detect: bool = False

  # ── 상태 전환 ──────────────────────────────────────────────────
  lane_available_last:    bool = False
  edge_available_last:    bool = False
  lane_available_trigger: bool = False
  lane_appeared:          bool = False

  # ── 측방 장애물 감지 ───────────────────────────────────────────
  object_detected_count: int  = 0
  side_object_detected:  bool = False
  object_clear_count:    int  = 0

  # ── BSD hold ───────────────────────────────────────────────────
  bsd_hold_counter: int  = 0
  bsd_detected_now: bool = False
  bsd_clear_count: int = field(default_factory=lambda: int(10.0 / DT_MDL))

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

    # running mean O(1)
    if len(self.lane_width_queue) == self.lane_width_queue.maxlen:
      self.lane_width_sum -= self.lane_width_queue.popleft()
    self.lane_width_queue.append(lane_w)
    self.lane_width_sum += lane_w
    self.lane_width = self.lane_width_sum / len(self.lane_width_queue)

    self.lane_width_diff = (
      (self.lane_width_queue[-1] - self.lane_width_queue[0])
      if len(self.lane_width_queue) >= 2 else 0.0
    )
    self.dist_to_edge     = float(dist_edge)
    self.dist_to_edge_far = float(dist_edge_far)

    min_lane_width = 2.5
    self.lane_width_count.update(self.lane_width > min_lane_width)
    self.edge_count.update(self.dist_to_edge > min_lane_width)

    available_count      = int(0.2 / DT_MDL)
    self.lane_available  = self.lane_width_count.counter > available_count
    self.edge_available  = (
      (self.edge_count.counter > available_count) and
      (self.dist_to_edge_far > min_lane_width)
    )

    self.cur_prob             = float(cur_prob)
    self.current_lane_missing = self.cur_prob < 0.3


  def update_lane_line_info(self, lane_line_info_raw: int):
    self.lane_line_info_raw = int(lane_line_info_raw)
    mod = self.lane_line_info_raw % 10
    # 0/5 로 바뀌는 순간만 edge_detect = True
    self.lane_line_info_edge_detect = (
      (mod in (0, 5)) and (self.last_lane_line_mod not in (0, 5))
    )
    self.last_lane_line_mod = mod
    self.lane_line_info_mod = mod


  # ════════════════════════════════════════════════════════════════
  #  측방 장애물 / BSD 업데이트
  # ════════════════════════════════════════════════════════════════
  def update_obstacles(self,
                      v_ego: float,
                      radar_obj,
                      blindspot: bool,
                      ignore_bsd: bool,
                      bsd_hold_sec: float = 2.5,
                      side_gap_margin: float = 3.0,
                      corner_long_dist_f: float = 0.0,
                      corner_long_dist_r: float = 0.0,
                      corner_lat_dist: float = 0.0,
                      object_clear_sec: float = 0.5):
    gap = float(np.clip(side_gap_margin, 1.0, 6.0))

    # ── 1) 사이드 레이더 (leadLeft / leadRight)
    radar_detected = self._radar_block(radar_obj, v_ego, gap)

    # ── 2) 측전방 코너 레이더
    front_detected = self._corner_block_front(corner_long_dist_f, corner_lat_dist, v_ego, gap)

    # ── 3) 측후방 코너 레이더
    rear_detected = self._corner_block_rear(corner_long_dist_r, v_ego, gap)

    # ── 4) 코너 레이더 활성 여부 (거리값 유효 여부로 판단)
    corner_radar_active = (corner_long_dist_f > 0) or (corner_long_dist_r > 0)
    self.corner_radar_active = corner_radar_active

    # ── 5) BSD → object 연동
    bsd_now = bool(blindspot) and (not ignore_bsd)

    bsd_as_object = bsd_now

    object_detected = radar_detected or front_detected or rear_detected or bsd_as_object

    # ── 6) 디바운싱
    CLEAR_FRAMES = max(1, int(object_clear_sec / DT_MDL))

    if object_detected:
      self.object_detected_count = 1
      self.object_clear_count    = 0
      self.side_object_detected  = True
    else:
      self.object_clear_count += 1
      if self.object_clear_count >= CLEAR_FRAMES:
        self.object_detected_count = 0
        self.side_object_detected  = False

    # ── 7) BSD hold
    self.bsd_detected_now = bsd_now

    # 코너 레이더 없는 차량은 hold 시간을 늘려 보수적으로 동작
    effective_hold_sec = bsd_hold_sec if corner_radar_active else max(bsd_hold_sec, 3.5)

    if self.bsd_detected_now:
      self.bsd_hold_counter = int(effective_hold_sec / DT_MDL)
      self.bsd_clear_count  = 0
    else:
      if self.bsd_hold_counter > 0:
        self.bsd_hold_counter -= 1
      else:
        self.bsd_clear_count += 1


  # ════════════════════════════════════════════════════════════════
  #  차선 변경 가능 여부 판단
  # ════════════════════════════════════════════════════════════════

  def compute_lane_change_available(self,
                                    lane_line_info_lt_20: bool,
                                    bsd_level: int,
                                    bsd_clear_sec: float = 1.0):
    BSD_CLEAR_FRAMES = max(1, int(bsd_clear_sec / DT_MDL))

    self.lane_change_available_geom = (
      (self.lane_available or self.edge_available) and lane_line_info_lt_20
    )

    ignore_bsd    = (bsd_level < 0)
    bsd_stabilized = (
      (self.bsd_hold_counter == 0) and
      (self.bsd_clear_count >= BSD_CLEAR_FRAMES)
    )
    bsd_active = (not bsd_stabilized) and (not ignore_bsd)

    # 완전 가능: geom + 장애물 없음 + BSD 없음
    self.lane_change_available = (
      self.lane_change_available_geom
      and (not self.side_object_detected)
      and (not bsd_active)
    )

    # BSD 무시 버전: geom + 장애물 없음
    self.lane_change_available_no_bsd = (
      self.lane_change_available_geom
      and (not self.side_object_detected)
    )

    # hold: geometry flicker 완화용, 장애물/BSD는 즉시 차단
    instant_block = self.side_object_detected or ((bsd_level >= 1) and bsd_active)
    hold_base     = (
      self.lane_change_available if (bsd_level >= 1)
      else self.lane_change_available_no_bsd
    )
    avail_false_thresh = int(0.3 / DT_MDL)

    if instant_block:
      self.avail_false_count          = avail_false_thresh
      self.lane_change_available_hold = False
    elif hold_base:
      self.avail_false_count          = 0
      self.lane_change_available_hold = True
    else:
      self.avail_false_count += 1
      if self.avail_false_count >= avail_false_thresh:
        self.lane_change_available_hold = False


  # ════════════════════════════════════════════════════════════════
  #  트리거 / 커밋
  # ════════════════════════════════════════════════════════════════

  def update_triggers(self):
    self.lane_available_trigger = (
      self.lane_width_diff > 0.8 and
      self.lane_width < self.dist_to_edge
    )
    appeared_now    = self.lane_exist_count.counter >= int(0.2 / DT_MDL)
    self.lane_appeared = (
      (self.lane_appeared or appeared_now) and
      (self.dist_to_edge < 4.0)
    )

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

