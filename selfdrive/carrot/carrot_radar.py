import numpy as np
import math

RADAR_TO_CAMERA = 1.52  # keep in sync with radard.py

class LinearVisionRadarMatcher:
  """
  Φ(feature) @ w 로 점수를 계산해서 argmax로 선택.

  이번 수정의 핵심:
  1) "정지차 sticky"는 비전(v_vis) 기준이 아니라 '레이더 last_track 기준'으로 판단
     - 주행 중 전방 정지차(lead v≈0)에서 비전 v가 튀어도 매칭이 안 깨지게
  2) 정지차 sticky 상태에서는
     - vel gate(속도 차 gate)를 크게 완화
     - dv(속도 차) 항의 영향(dv2)을 스케일 다운(=비전 속도 노이즈 무시)
  3) 기존의 vision_stopped * dv2 같은 헷갈리는 상호작용(weight 8/9)은 제거(0으로 고정)
     - 대신 dv_scale로 직관적으로 처리
  """

  def __init__(self,
               w=None,
               switch_margin: float = 0.45,   # 스위치 억제(조금 더 sticky)
               max_missed: int = 8,
               use_softmax: bool = False,
               softmax_temp: float = 1.0,

               # ---- 정지차 sticky 모드(=앞차 정지) 판단/강화 파라미터 ----
               stationary_vlead_thr: float = 2.0,   # last_track.vLead < 이 값이면 "정지차 붙잡는 중"
               stationary_alead_thr: float = 1.0,   # last_track.aLead 절대값이 너무 크면 제외(옵션)
               stationary_cnt_thr: int = 3,         # last_track.cnt가 이 이상일 때만(안정된 트랙)
               stationary_drel_max: float = 150.0,  # 너무 멀면 정지차 판단하지 않음(옵션)

               dv_scale_stationary: float = 0.25,   # 정지차 모드에서 dv 영향 축소(0.0이면 완전 무시)
               vel_guard_stationary: float = 120.0, # 정지차 모드에서 vel gate 완화
               y_lim_stationary: float = 3.5        # 정지차 모드에서 y gate 완화(비전 y 튐 대비)
               ):
    # ---- 상태 ----
    self.last_id = None
    self.last_score = None
    self.missed = 0

    # ---- 선택 정책 ----
    self.switch_margin = float(switch_margin)
    self.max_missed = int(max_missed)

    # ---- 디버그/확률 필요하면 ----
    self.use_softmax = bool(use_softmax)
    self.softmax_temp = float(softmax_temp)

    # ---- 정지차 sticky 파라미터 ----
    self.stationary_vlead_thr = float(stationary_vlead_thr)
    self.stationary_alead_thr = float(stationary_alead_thr)
    self.stationary_cnt_thr = int(stationary_cnt_thr)
    self.stationary_drel_max = float(stationary_drel_max)

    self.dv_scale_stationary = float(dv_scale_stationary)
    self.vel_guard_stationary = float(vel_guard_stationary)
    self.y_lim_stationary = float(y_lim_stationary)

    # ---- 기본 weight (튜닝 포인트) ----
    # feature 순서는 _build_features() 주석과 반드시 일치해야 함
    # (8,9 항목은 더 이상 사용하지 않음: 0으로 고정)
    if w is None:
      self.w = np.array([
        -1.00,  # 0: dx2
        -1.30,  # 1: dy2
        -0.35,  # 2: dv2  (정지차에서는 dv_scale로 추가 축소)
        -0.80,  # 3: lane_pen (1-in_lane)
        +1.60,  # 4: same_as_last  (sticky 강화)
        +0.40,  # 5: measured
        -0.25,  # 6: cnt_small_pen (cnt<2 penalty)
        -0.60,  # 7: v_abs (고속 물체 약간 불리)
         0.00,  # 8: (unused)
         0.00,  # 9: (unused)
      ], dtype=np.float32)
    else:
      w = np.array(w, dtype=np.float32)
      # 길이 부족/초과 방어(최소 10개 맞추기)
      if w.size < 10:
        w2 = np.zeros(10, dtype=np.float32)
        w2[:w.size] = w
        w = w2
      elif w.size > 10:
        w = w[:10]
      # 8,9는 사용 안 하므로 0으로 고정(혼동 방지)
      w[8] = 0.0
      w[9] = 0.0
      self.w = w

  def reset(self):
    self.last_id = None
    self.last_score = None
    self.missed = 0

  def _softmax(self, x):
    x = x / max(self.softmax_temp, 1e-6)
    x = x - np.max(x)
    e = np.exp(x)
    return e / max(np.sum(e), 1e-9)

  def _vectorize_tracks(self, tracks: dict):
    # NOTE: keys() 순서가 중요하지 않으면 이렇게 가도 OK
    ids = np.fromiter(tracks.keys(), dtype=np.int32)
    ts  = [tracks[i] for i in ids]

    d = np.array([t.dRel for t in ts], dtype=np.float32)
    y = np.array([t.yRel for t in ts], dtype=np.float32)
    v = np.array([t.vlead_for_matching() for t in ts], dtype=np.float32)  # 스파이크 억제 버전

    in_lane = np.array([getattr(t, "in_lane_prob", 0.0) for t in ts], dtype=np.float32)
    measured = np.array([1.0 if getattr(t, "measured", True) else 0.0 for t in ts], dtype=np.float32)
    cnt = np.array([float(getattr(t, "cnt", 0)) for t in ts], dtype=np.float32)

    return ids, ts, d, y, v, in_lane, measured, cnt

  def _gating_mask(self,
                   d, y, v,
                   offset_d, lead_y, v_vis,
                   min_d, max_d,
                   y_lim,
                   vel_guard):
    m = (d > min_d) & (d < max_d)
    m &= (np.abs(y + lead_y) < y_lim)
    m &= (np.abs(v - v_vis) < vel_guard)
    return m

  def _build_features(self,
                      ids, d, y, v, in_lane, measured, cnt,
                      offset_d, lead_y, v_vis,
                      xStd, yStd, vStd,
                      dv_scale: float):
    """
    Φ: (N, F)
    0 dx2 = ((d-offset_d)/xStd)^2
    1 dy2 = ((y+lead_y)/yStd)^2
    2 dv2 = (((v-v_vis)*dv_scale)/vStd)^2   <-- 정지차 모드에서는 dv_scale < 1
    3 lane_pen = (1 - in_lane)
    4 same_as_last
    5 measured
    6 cnt_small_pen
    7 v_abs
    8 unused (0)
    9 unused (0)
    """
    xStd = max(float(xStd), 1e-3)
    yStd = max(float(yStd), 1e-3)
    vStd = max(float(vStd), 1e-3)

    dx = (d - float(offset_d)) / xStd
    dy = (y + float(lead_y))  / yStd

    # 핵심: dv는 dv_scale로 스케일링 (정지차 모드면 0.25 등으로 축소)
    dv = ((v - float(v_vis)) * float(dv_scale)) / vStd

    dx2 = dx * dx
    dy2 = dy * dy
    dv2 = dv * dv

    lane_pen = 1.0 - np.clip(in_lane, 0.0, 1.0)

    if self.last_id is None:
      same_last = np.zeros_like(dx2, dtype=np.float32)
    else:
      same_last = (ids == int(self.last_id)).astype(np.float32)

    meas = np.clip(measured, 0.0, 1.0)
    cnt_small_pen = (cnt < 2.0).astype(np.float32)

    v_abs = np.abs(v)
    v_abs = np.clip(v_abs / 40.0, 0.0, 3.0).astype(np.float32)

    zeros = np.zeros_like(dx2, dtype=np.float32)

    Phi = np.stack([
      dx2,
      dy2,
      dv2,
      lane_pen,
      same_last,
      meas,
      cnt_small_pen,
      v_abs,
      zeros,   # 8 unused
      zeros,   # 9 unused
    ], axis=1).astype(np.float32)

    return Phi

  def _stationary_sticky_mode(self, tracks: dict) -> bool:
    """
    '주행 중 전방 정지차' 상황을 잡기 위한 모드 판정.
    비전 속도는 믿지 않으므로, 마지막으로 붙잡은 레이더 트랙(last_track)을 기준으로 판단.
    """
    if self.last_id is None:
      return False
    t = tracks.get(int(self.last_id))
    if t is None:
      return False

    # 트랙이 충분히 안정적일 때만
    if getattr(t, "cnt", 0) < self.stationary_cnt_thr:
      return False

    # 너무 멀면 정지차 판단 안 함(옵션)
    if getattr(t, "dRel", 9999.0) > self.stationary_drel_max:
      return False

    vlead = abs(float(getattr(t, "vLead", 9999.0)))
    alead = abs(float(getattr(t, "aLead", 0.0)))

    if vlead < self.stationary_vlead_thr and alead < self.stationary_alead_thr:
      return True

    return False

  def match_vision_to_track(self, v_ego: float, lead, tracks: dict):
    """
    리턴: best_track (또는 None)
    """
    if not tracks:
      return self._on_miss_and_fallback(tracks)

    offset_d = float(lead.x[0] - RADAR_TO_CAMERA)
    lead_y   = float(lead.y[0])
    v_vis    = float(lead.v[0])
    prob     = float(lead.prob)

    xStd = float(lead.xStd[0])
    yStd = float(lead.yStd[0])
    vStd = float(lead.vStd[0])

    # --- 기본 distance gate ---
    max_d = max(offset_d * 1.25, 5.0)
    min_d = max(offset_d * 0.80, 1.0)

    # --- 기본 y gate ---
    y_lim = 2.0 if prob > 0.5 else 2.6

    # --- 기본 vel guard (말도 안되는 것만 제거) ---
    vel_tol = float(max(v_vis * np.interp(prob, [0.8, 0.98], [0.3, 0.5]), 5.0))
    vel_guard = max(vel_tol * 3.0, 20.0)

    # --- 핵심: 정지차 sticky 모드 판정(레이더 last_track 기준) ---
    stationary_mode = self._stationary_sticky_mode(tracks)

    # 정지차 모드면 속도 gate 풀고, y도 완화 (비전 튐 대비)
    if stationary_mode:
      vel_guard = max(vel_guard, self.vel_guard_stationary)
      y_lim = max(y_lim, self.y_lim_stationary)

    # dv 스케일(정지차 모드면 dv 영향 축소)
    dv_scale = self.dv_scale_stationary if stationary_mode else 1.0

    # --- vectorize ---
    ids, ts, d, y, v, in_lane, measured, cnt = self._vectorize_tracks(tracks)

    # --- gating ---
    m = self._gating_mask(d, y, v,
                          offset_d, lead_y, v_vis,
                          min_d, max_d,
                          y_lim,
                          vel_guard)

    if not np.any(m):
      return self._on_miss_and_fallback(tracks)

    ids_m = ids[m]
    d_m, y_m, v_m = d[m], y[m], v[m]
    in_lane_m = in_lane[m]
    measured_m = measured[m]
    cnt_m = cnt[m]

    # --- build features ---
    Phi = self._build_features(ids_m, d_m, y_m, v_m, in_lane_m, measured_m, cnt_m,
                               offset_d, lead_y, v_vis,
                               xStd, yStd, vStd,
                               dv_scale=dv_scale)

    # --- linear score (클수록 좋음) ---
    scores = Phi @ self.w

    # (선택) 확률이 필요하면
    if self.use_softmax:
      _ = self._softmax(scores)

    # best
    best_idx = int(np.argmax(scores))
    best_id = int(ids_m[best_idx])
    best_score = float(scores[best_idx])

    # --- hysteresis switch ---
    chosen_id = best_id
    chosen_score = best_score

    if self.last_id is not None and int(self.last_id) in tracks:
      last_pos = np.where(ids_m == int(self.last_id))[0]
      if last_pos.size == 1:
        last_score_now = float(scores[int(last_pos[0])])

        if best_id != int(self.last_id):
          # 정지차 모드에서는 스위치를 더 억제(조금 더 sticky)
          margin = self.switch_margin * (1.4 if stationary_mode else 1.0)

          if best_score < last_score_now + margin:
            chosen_id = int(self.last_id)
            chosen_score = last_score_now

    # commit
    self.last_id = chosen_id
    self.last_score = chosen_score
    self.missed = 0

    return tracks.get(chosen_id, None)

  def _on_miss_and_fallback(self, tracks):
    self.missed += 1
    if self.missed > self.max_missed:
      self.reset()
      return None

    # vision이 튀는 프레임에서는 last를 잠깐 유지
    if self.last_id is not None and int(self.last_id) in tracks:
      return tracks[int(self.last_id)]
    return None
