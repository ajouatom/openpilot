#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any
import heapq
import copy

import capnp
from openpilot.cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D
from openpilot.selfdrive.controls.lib.drive_helpers import is_volkswagen_meb
from openpilot.selfdrive.controls.lib.cutin_helpers import (
  CUTIN_DEFAULT_ENTER_MIN_INWARD_SPEED,
  associate_cutin_tracks,
  combine_cutin_future_projection,
  cutin_confirmation_frames,
  cutin_min_track_age_frames,
  cutin_entry_rejection_reason,
  cutin_tuning_from_sensitivity,
  effective_cutin_inward_speed,
  is_corner_radar_source,
  is_corner_track_id,
  is_fast_cutin_entry,
  new_cutin_position_history,
  update_cutin_confirmation,
  update_lane_relative_motion,
)


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

STICKY_SELECTED_COUNT_MAX = int(2.0 / DT_MDL)
STICKY_MAX_DPATH = 0.8
STICKY_FAR_DREL = 60.0
STICKY_MAX_DPATH_FAR = 1.2
STICKY_PATH_Y_STD_GAIN = 0.5

# VW MEB(ID.4/ID.5) 전용: 레이더 vLead 미분(interfaces.py MyTrack)이 만드는 aLeadK 노이즈 스파이크를
# 모델(비전) 가속도 기준으로 제한할 밴드. 양자화된 Rel_Velo 탓에 정상 추종 중에도 aLeadK가
# -1.4~-2.8까지 튀어 오탐 FCW/급제동을 유발했음(모델 a는 -0.6~+0.6로 매끈).
MEB_ALEAD_CLAMP_BAND = 1.0

CUTIN_STICKY_FRAMES = int(0.7 / DT_MDL)
CUTIN_OUTPUT_HOLD_FRAMES = max(1, int(round(0.5 / DT_MDL)))
CUTIN_OUTPUT_HOLD_DREL_M = 3.0
CUTIN_OUTPUT_HOLD_YREL_M = 1.0
CUTIN_OUTPUT_HOLD_VREL_MPS = 2.0
CUTIN_KEEP_FUTURE_IN_LANE_PROB = 0.12
CUTIN_KEEP_MAX_DPATH_FUTURE = 1.6
CUTIN_KEEP_MAX_MOVING_AWAY = 0.3
CORNER_ACCEL_MIN_TRACK_AGE = 6
CORNER_ACCEL_MAX_ABS_DPATH = 1.5
CORNER_ACCEL_MAX_ABS_ALEAD = 3.0
CUTIN_PROMOTE_DREL_MARGIN = 1.0
CORNER_FRONT_MATCH_PROMOTE_DREL_MARGIN = 8.0
CUTIN_DEFAULT_CONFIRM_S = 0.20
CUTIN_DEFAULT_MIN_TRACK_AGE_S = 0.25
CUTIN_DEFAULT_ENTER_MIN_X = 1.0
CUTIN_DEFAULT_ENTER_MAX_X = 55.0
CUTIN_DEFAULT_ENTER_MIN_ABS_DPATH = 1.5
CUTIN_DEFAULT_ENTER_FUTURE_IN_LANE_PROB = 0.20
CUTIN_DEFAULT_ENTER_CENTERING_GAIN = 0.20
CUTIN_FIXED_SENSITIVITY = 50.0
CUTIN_YAW_COMP_GAIN = 0.6
CUTIN_YAW_COMP_MAX_DREL = 50.0
CUTIN_YAW_COMP_MAX_YAW_RATE = 0.35
CUTIN_YAW_COMP_MAX_YVREL_CORRECTION = 1.5
CUTIN_YAW_COMP_MAX_VREL_CORRECTION = 0.6
RADAR_ONLY_FALLBACK_VISION_PROB = 0.55

VISION_ONLY_RADAR_TRACK_MODE = -2

CENTER_LEAD_NEAR_DPATH_LIMIT = 1.2
CENTER_LEAD_FAR_DPATH_LIMIT = 0.9
CENTER_LEAD_FAR_DREL = 60.0
CENTER_LEAD_NEAR_IN_LANE_PROB = 0.3
CENTER_LEAD_FAR_IN_LANE_PROB = 0.45
RADAR_ONLY_CENTER_DPATH_NEAR_LIMIT = 1.1
RADAR_ONLY_CENTER_DPATH_MID_LIMIT = 0.9
RADAR_ONLY_CENTER_DPATH_FAR_LIMIT = 0.75
RADAR_ONLY_CENTER_MID_DREL = 60.0
RADAR_ONLY_CENTER_FAR_DREL = 80.0
RADAR_ONLY_CENTER_MAX_DREL = 100.0

RADAR_CENTER_PROMOTION_MAX_LANE_CENTER_OFFSET = 1.5
RADAR_CENTER_PROMOTION_RECEDING_MAX_DREL = 45.0
RADAR_CENTER_PROMOTION_RECEDING_VREL = 0.5
CORNER_FRONT_MATCH_DREL = 3.0
CORNER_FRONT_MATCH_VREL = 2.0
CORNER_CENTER_MIN_AGE = int(0.25 / DT_MDL)
CORNER_STOPPED_MIN_AGE = int(0.35 / DT_MDL)
CORNER_STOPPED_MIN_DREL = 5.0
CORNER_STOPPED_MAX_DREL = 120.0
CORNER_STOPPED_MAX_VLEAD = 1.8
CORNER_STOPPED_MAX_YVREL = 0.8
CORNER_STOPPED_NEAR_DPATH_LIMIT = 1.0
CORNER_STOPPED_FAR_DPATH_LIMIT = 0.75
CORNER_STOPPED_NEAR_IN_LANE_PROB = 0.35
CORNER_STOPPED_FAR_IN_LANE_PROB = 0.5
CORNER_STOPPED_FAR_DREL = 60.0
CORNER_VISION_KEEP_PROB = 0.75
FRONT_RADAR_VISION_MATCH_MIN_PROB = 0.4

def laplacian_pdf(x: float, mu: float, b: float):
  diff = abs(x - mu) / max(b, 1e-4)
  return 0.0 if diff > 50.0 else math.exp(-diff)

def clamp(x: float, lo: float, hi: float) -> float:
  return float(np.clip(x, lo, hi))

def is_radar_center_promotion_safe(lead: dict[str, Any]) -> bool:
  d_rel = float(lead.get("dRel", 999.0))
  y_rel = float(lead.get("yRel", 999.0))
  d_path = float(lead.get("dPath", 999.0))
  v_rel = float(lead.get("vRel", 999.0))

  # Unmatched center candidates rely on predicted lane geometry. Avoid promotion
  # when curvature can project an adjacent-lane radar return onto the ego path.
  if abs(d_path - y_rel) >= RADAR_CENTER_PROMOTION_MAX_LANE_CENTER_OFFSET:
    return False

  # A far lead pulling away cannot constrain longitudinal control yet.
  return d_rel <= RADAR_CENTER_PROMOTION_RECEDING_MAX_DREL or v_rel <= RADAR_CENTER_PROMOTION_RECEDING_VREL

EMPTY_LEAD = {
  "dRel": 0.0,
  "yRel": 0.0,
  "vRel": 0.0,
  "aRel": 0.0,
  "vLead": 0.0,
  "aLead": 0.0,
  "dPath": 0.0,
  "vLat": 0.0,
  "vLeadK": 0.0,
  "aLeadK": 0.0,
  "fcw": False,
  "status": False,
  "aLeadTau": 0.0,
  "modelProb": 0.0,
  "radar": False,
  "radarTrackId": -1,
  "jLead": 0.0,
  "score": 0.0,
}

def empty_lead():
  return EMPTY_LEAD.copy()

def select_side_leads(front_leads: list[dict[str, Any]], corner_leads: list[dict[str, Any]],
                      corner_tracks_available: bool) -> list[dict[str, Any]]:
  return corner_leads if corner_tracks_available else front_leads

def pick_side_lead(leads: list[dict[str, Any]]) -> dict[str, Any]:
  return min(
    (ld for ld in leads if ld['dRel'] > 5 and abs(ld['dPath']) < 3.5),
    key=lambda d: d['dRel'],
    default=empty_lead()
  )

class Track:
  def __init__(self, identifier: int):
    self.identifier = identifier
    self.radar_source = "frontRadar"
    self.is_corner_radar = False
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)

    self.is_stopped_car_count = 0
    self.selected_count = 0
    self.cut_in_count = 0
    self.cut_in_start_abs_dpath = 0.0
    self.measured = False
    self.score = 0.0
    self.in_lane_prob = 0.0
    self.in_lane_prob_future = 0.0

    self.dRel = 0.0
    self.yRel = 0.0
    self.vRel = 0.0
    self.vLead = 0.0
    self.vLeadK = 0.0
    self.aLead = 0.0
    self.aLeadK = 0.0
    self.jLead = 0.0
    self.yvLead = 0.0
    self.dRel_future = 0.0
    self.yRel_future = 0.0
    self.dPath_future = 0.0
    self.dPath = 0.0
    self.lane_half_width = 1.8
    self.dPath_rate = 0.0
    self.dPath_inward_speed = 0.0
    self._cutin_position_history = new_cutin_position_history(DT_MDL)
    self.sticky_dPath = 0.0
    self.sticky_path_y_std = 0.0

    # ---- noise filter state (new) ----
    self._vLead_last = 0.0
    self._vLead_filt = 0.0
    self._vLead_filt_init = False

  def inherit_cutin_state(self, source: 'Track') -> None:
    self.measured = source.measured
    self.radar_source = source.radar_source
    self.is_corner_radar = source.is_corner_radar
    self.dRel = source.dRel
    self.yRel = source.yRel
    self.vRel = source.vRel
    self.vLead = source.vLead
    self.cnt = source.cnt
    self.cut_in_count = source.cut_in_count
    self.cut_in_start_abs_dpath = source.cut_in_start_abs_dpath
    self._cutin_position_history.clear()
    self._cutin_position_history.extend(source._cutin_position_history)

  def update(self, md, pt, ready, radar_reaction_factor, radar_lat_factor, yaw_rate, is_corner_radar=False, v_ego=0.0):
    prev_measured = self.measured
    prev_dRel = self.dRel
    prev_yRel = self.yRel
    prev_vLead = self.vLead

    self.dRel = pt.dRel
    self.yRel = pt.yRel
    self.vRel = pt.vRel

    self.vLead = self.vLeadK = pt.vLead
    self.aLead = self.aLeadK = pt.aLead
    self.jLead = pt.jLead
    self.yvLead = pt.yvRel

    self.measured = pt.measured
    self.radar_source = str(pt.radarSource)
    self.is_corner_radar = is_corner_radar
    track_discontinuous = prev_measured and (
      abs(self.dRel - prev_dRel) > 5.0 or
      abs(self.yRel - prev_yRel) > 2.0 or
      abs(self.vLead - prev_vLead) > 7.0
    )
    if not self.measured:
      self.cnt = 0
      self.selected_count = 0
      self.is_stopped_car_count = 0
      self.cut_in_count = 0
      self.cut_in_start_abs_dpath = 0.0
      self._cutin_position_history.clear()
      # optional: also reset filter init when track is not measured
      self._vLead_filt_init = False
    elif track_discontinuous:
      self.selected_count = 0
      self.is_stopped_car_count = 0
      self.cut_in_count = 0
      self.cut_in_start_abs_dpath = 0.0
      self._cutin_position_history.clear()

    v_rel_future, yv_rel_future = self.yaw_compensated_velocities(yaw_rate)
    self.dRel_future = self.dRel + v_rel_future * radar_lat_factor
    self.yRel_future = self.yRel + yv_rel_future * radar_lat_factor
    if ready:
      self.d_path(md)
      self.dPath_rate, self.dPath_inward_speed = update_lane_relative_motion(
        self._cutin_position_history,
        self.dRel,
        self.yRel,
        md.laneLines[1].x,
        md.laneLines[1].y,
        md.laneLines[2].y,
        self.measured,
        track_discontinuous,
        DT_MDL,
      )
      self.dPath_future, self.in_lane_prob_future = combine_cutin_future_projection(
        self.dPath,
        self.dPath_rate,
        radar_lat_factor,
        self.lane_half_width,
        self.dPath_future,
        self.in_lane_prob_future,
      )
      self.dPath_inward_speed = effective_cutin_inward_speed(
        self.dRel,
        v_ego=v_ego,
        temporal_inward_speed=self.dPath_inward_speed,
        d_path=self.dPath,
        projected_d_path=self.dPath_future,
        horizon_s=radar_lat_factor,
      )
      if self.selected_count > 0:
        self.sticky_dPath, self.sticky_path_y_std = self.path_d_path(md)

      if self.selected_count > 0 and abs(self.sticky_dPath) > self.sticky_dpath_limit():
        self.selected_count = 0
        self.is_stopped_car_count = 0

    a_lead_threshold = 0.5 * radar_reaction_factor
    if abs(self.aLead) < a_lead_threshold and abs(self.jLead) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU * radar_reaction_factor
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def d_path(self, md):
    lane_xs = md.laneLines[1].x
    left_ys = md.laneLines[1].y
    right_ys = md.laneLines[2].y

    def d_path_interp(dRel, yRel):
      left_lane_y = np.interp(dRel, lane_xs, left_ys)
      right_lane_y = np.interp(dRel, lane_xs, right_ys)
      center_y = (left_lane_y + right_lane_y) / 2.0
      lane_half_width = max(0.1, abs(right_lane_y - left_lane_y) / 2.0)
      dist_from_center = yRel + center_y
      in_lane_prob = max(0.0, 1.0 - (abs(dist_from_center) / lane_half_width))
      return dist_from_center, in_lane_prob, lane_half_width

    self.dPath, self.in_lane_prob, self.lane_half_width = d_path_interp(self.dRel, self.yRel)
    self.dPath_future, self.in_lane_prob_future, _ = d_path_interp(self.dRel_future, self.yRel_future)

  def yaw_compensated_velocities(self, yaw_rate: float) -> tuple[float, float]:
    # A curved ego path creates apparent lateral velocity in the ego frame
    # (yaw_rate * dRel). Remove it before cut-in projection so adjacent-lane
    # objects on curves are not classified as moving into our lane. Keep this
    # projection-local so the published lead speed remains the raw radar value.
    yaw_rate = clamp(float(yaw_rate), -CUTIN_YAW_COMP_MAX_YAW_RATE, CUTIN_YAW_COMP_MAX_YAW_RATE)
    d_rel_for_comp = clamp(self.dRel, 0.0, CUTIN_YAW_COMP_MAX_DREL)
    yv_rel_corr = clamp(
      -yaw_rate * d_rel_for_comp * CUTIN_YAW_COMP_GAIN,
      -CUTIN_YAW_COMP_MAX_YVREL_CORRECTION,
      CUTIN_YAW_COMP_MAX_YVREL_CORRECTION,
    )
    v_rel_corr = clamp(
      yaw_rate * self.yRel * CUTIN_YAW_COMP_GAIN,
      -CUTIN_YAW_COMP_MAX_VREL_CORRECTION,
      CUTIN_YAW_COMP_MAX_VREL_CORRECTION,
    )
    return float(self.vRel + v_rel_corr), float(self.yvLead + yv_rel_corr)

  def path_d_path(self, md) -> tuple[float, float]:
    path_y = float(np.interp(self.dRel, md.position.x, md.position.y))
    path_y_std = float(np.interp(self.dRel, md.position.x, md.position.yStd)) if len(md.position.yStd) else 0.0
    return float(self.yRel + path_y), path_y_std

  def sticky_dpath_limit(self) -> float:
    if self.dRel < STICKY_FAR_DREL:
      return STICKY_MAX_DPATH
    return float(np.clip(STICKY_MAX_DPATH + STICKY_PATH_Y_STD_GAIN * self.sticky_path_y_std,
                         STICKY_MAX_DPATH, STICKY_MAX_DPATH_FAR))

  # ---- noise suppression only when cnt>=2 ----
  def vlead_for_matching(self, dv_max: float = 4.0, alpha: float = 0.35) -> float:
    """
    Returns vLead to be used in matching score.
    - If cnt < 2: raw vLead (no filtering)
    - If cnt >= 2: clamp spike + IIR smooth
    """
    v = float(self.vLead)

    if self.cnt < 2:
      return v

    if not self._vLead_filt_init:
      self._vLead_last = v
      self._vLead_filt = v
      self._vLead_filt_init = True
      return v

    v_last = self._vLead_last
    self._vLead_last = v

    v_clamped = clamp(v, v_last - dv_max, v_last + dv_max)
    self._vLead_filt = alpha * v_clamped + (1.0 - alpha) * self._vLead_filt
    return float(self._vLead_filt)

  def get_RadarState(self, model_prob: float = 0.0, vision_y_rel=0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel) if self.yRel != 0.0 else vision_y_rel,
      "dPath": float(self.dPath),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLead": float(self.aLead),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "jLead": float(self.jLead),
      "vLat": float(self.yvLead),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
      "score": self.score,
    }

  def potential_low_speed_lead(self, v_ego: float):
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    return f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, lead_prob: float,
                          tracks: dict[int, Track], update_counters: bool = True):
  if not tracks:
    return None

  offset_vision_dist = float(lead.x[0] - RADAR_TO_CAMERA)

  # distance gates
  max_vision_dist  = max(offset_vision_dist * 1.25, 5.0)
  min_vision_dist  = max(offset_vision_dist * 0.80, 1.0)
  max_vision_dist2 = max(offset_vision_dist * 1.45, 5.0)
  min_vision_dist2 = 1.5

  # velocity tolerance (same intent)
  vel_tol = float(max(lead.v[0] * np.interp(lead_prob, [0.8, 0.98], [0.3, 0.5]), 5.0))
  # hard guardrail for moving-bias (prevents absurd match)
  vel_guard = max(vel_tol * 3.0, 20.0)

  def dist_sane(t: Track, wide: bool = False) -> bool:
    if wide:
      return (min_vision_dist2 < t.dRel < max_vision_dist2)
    return (min_vision_dist < t.dRel < max_vision_dist)

  def y_sane(t: Track, wide: bool = False) -> bool:
    lim = 4.0 if wide else 2.0
    return abs(t.yRel + float(lead.y[0])) < lim

  def vel_sane(t: Track) -> bool:
    """
    Keep your philosophy:
      - if it's moving, likely "the car we should read"
    but add guardrail and (optionally) in-lane preference.
    """
    v_vis = float(lead.v[0])
    v_trk = float(t.vLead)
    dv = abs(v_trk - v_vis)

    # normal strict check
    if dv < vel_tol:
      return True

    # moving-bias: allow more mismatch for moving objects,
    # but only within a reasonable guardrail.
    moving = (v_trk > 3.0)
    if not moving:
      return False

    if dv > vel_guard:
      return False

    # If in-lane probability exists (it does in your Track), use it as safety.
    # When it's clearly not in our lane, don't use moving-bias.
    # (This line is intentionally mild; you can tune 0.2~0.5)
    if hasattr(t, "dPath") and (t.in_lane_prob < 0.25):
      return False

    return True

  def score_pair(t: Track):
    """
    score1: normal yStd
    score2: wide yStd for cut-in
    NOTE: uses t.vlead_for_matching() only for scoring (cnt>=2 only).
    """
    pd = laplacian_pdf(float(t.dRel), offset_vision_dist, float(lead.xStd[0]))
    py = laplacian_pdf(float(t.yRel), -float(lead.y[0]), float(lead.yStd[0]))
    py2 = laplacian_pdf(float(t.yRel), -float(lead.y[0]), float(lead.yStd[0]) * 2.0)

    v_use = float(t.vlead_for_matching())  # noise suppression only if cnt>=2
    pv = laplacian_pdf(v_use, float(lead.v[0]), float(lead.vStd[0]))

    s1 = pd * py * pv
    s2 = pd * py2 * pv
    return s1, s2

  # ---- pick best candidates (FIX: true 1st/2nd) ----
  first_track, second_track, extra_track = None, None, None
  first_score, second_score, extra_score = -1e18, -1e18, -1e18

  for t in tracks.values():
    s1, s2 = score_pair(t)
    t.score = s1

    if s1 > first_score:
      second_track, second_score = first_track, first_score
      first_track, first_score = t, s1
    elif s1 > second_score:
      second_track, second_score = t, s1

    if s2 > extra_score:
      extra_track, extra_score = t, s2

  # score floor
  if first_track is None or first_score < 1e-4:
    return None

  # ---- selection policy (same logic, cleaner & safer) ----
  best_track = None

  # A) normal match
  if dist_sane(first_track) and vel_sane(first_track):
    select_second_track = False
    if second_track is not None and vel_sane(second_track) and second_track.in_lane_prob > 0.3:
      if second_track.cnt > 5 and offset_vision_dist * 0.5 < second_track.dRel < first_track.dRel:
        select_second_track = True

    if select_second_track:
      best_track = second_track
    elif y_sane(first_track):
      if lead_prob > 0.5:
        best_track = first_track
      elif lead_prob > 0.4 and first_track.selected_count > 0:
        best_track = first_track
    elif lead_prob > 0.6:
      best_track = first_track

  # B) stopped-car-like (only if not chosen yet)
  if best_track is None and dist_sane(first_track) and y_sane(first_track, wide=True):
    if (second_track is not None and second_score > 1e-5 and
        dist_sane(second_track) and y_sane(second_track) and vel_sane(second_track)):
      best_track = second_track
    elif first_track.selected_count > 0:
      best_track = first_track
    else:
      first_track.is_stopped_car_count += 2
      if first_track.is_stopped_car_count > int(1.0 / DT_MDL):
        best_track = first_track

  # C) cut-in wide matching (only if not chosen yet)
  if best_track is None and offset_vision_dist < 90.0 and lead_prob > 0.65:
    # wide-y winner first (cut-in)
    if (extra_track is not None and extra_score > first_score and
        dist_sane(extra_track, wide=True) and vel_sane(extra_track) and y_sane(extra_track, wide=True)):
      best_track = extra_track

    # then allow first/second with wide gates
    elif dist_sane(first_track, wide=True) and vel_sane(first_track) and y_sane(first_track, wide=True):
      best_track = first_track

    elif (second_track is not None and second_score > 1e-4 and
          dist_sane(second_track, wide=True) and vel_sane(second_track) and y_sane(second_track, wide=True)):
      best_track = second_track

  # ---- update counters ----
  if update_counters:
    for t in tracks.values():
      if t is best_track and best_track is not None:
        t.selected_count = min(t.selected_count + 1, STICKY_SELECTED_COUNT_MAX)
      elif best_track is not None:
        t.selected_count = 0
        t.is_stopped_car_count = max(0, t.is_stopped_car_count - 1)

  return best_track


def get_RadarState_from_vision(md, lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  dRel = float(lead_msg.x[0] - RADAR_TO_CAMERA)
  yRel = float(-lead_msg.y[0])
  dPath = yRel + np.interp(dRel, md.position.x, md.position.y)
  return {
    "dRel": float(dRel),
    "yRel": yRel,
    "dPath" : float(dPath),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLead": float(lead_msg.a[0]),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "jLead": 0.0,
    "vLat" : 0.0,
    "fcw": False,
    "modelProb": float(lead_prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }

class RadarD:
  def __init__(self, delay: float = 0.0, is_vw_meb: bool = False, car_brand: str = ""):
    self.current_time = 0.0

    # VW MEB(ID.4/ID.5)에서만 True -> get_lead가 infiniteCable2(=comma) 리드선택을 따름(sticky/track_scc 미사용).
    self.is_vw_meb = is_vw_meb
    self.car_brand = car_brand.lower()

    self.tracks: dict[int, Track] = {}

    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]

    self.v_ego = 0.0
    print("###RadarD.. : delay = ", delay, int(round(delay / DT_MDL))+1)
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

    self.params = Params()
    self.enable_radar_tracks = self.params.get_int("EnableRadarTracks")
    self.enable_corner_radar = self.params.get_int("EnableCornerRadar")
    self.radar_lat_factor = 0.0
    self.cutin_yaw_rate = 0.0
    self.cutin_yaw_rate_filter = FirstOrderFilter(0.0, 0.20, DT_MDL)
    self.cutin_confirm_frames = max(1, int(round(CUTIN_DEFAULT_CONFIRM_S / DT_MDL)))
    self.cutin_min_track_age = max(1, int(round(CUTIN_DEFAULT_MIN_TRACK_AGE_S / DT_MDL)))
    self.cutin_enter_min_x = CUTIN_DEFAULT_ENTER_MIN_X
    self.cutin_enter_max_x = CUTIN_DEFAULT_ENTER_MAX_X
    self.cutin_enter_min_abs_dpath = CUTIN_DEFAULT_ENTER_MIN_ABS_DPATH
    self.cutin_enter_future_in_lane_prob = CUTIN_DEFAULT_ENTER_FUTURE_IN_LANE_PROB
    self.cutin_enter_centering_gain = CUTIN_DEFAULT_ENTER_CENTERING_GAIN
    self.cutin_enter_min_inward_speed = CUTIN_DEFAULT_ENTER_MIN_INWARD_SPEED
    self.cutin_tuning = cutin_tuning_from_sensitivity(CUTIN_FIXED_SENSITIVITY)

    self.radar_detected = False
    self.lead_one_front_radar_vision_match = False
    self.leadCenter = None
    self.leadTwo = None
    self.leadCutIn = empty_lead()
    self.cutin_output_hold_count = 0
    self.cutin_output_hold_reference: tuple[float, float, float] | None = None
    self.cornerLeadStopped = empty_lead()
    self.corner_tracks_available = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    self.enable_radar_tracks = self.params.get_int("EnableRadarTracks")
    self.enable_corner_radar = self.params.get_int("EnableCornerRadar")
    cutin_enabled = self.enable_corner_radar > 1
    cutin_tuning = cutin_tuning_from_sensitivity(CUTIN_FIXED_SENSITIVITY)
    self.cutin_tuning = cutin_tuning
    self.radar_lat_factor = cutin_tuning["horizon_s"] if cutin_enabled else 0.0
    self.cutin_confirm_frames = max(1, int(round(cutin_tuning["confirm_s"] / DT_MDL)))
    self.cutin_min_track_age = max(1, int(round(cutin_tuning["min_track_age_s"] / DT_MDL)))
    self.cutin_enter_min_x = cutin_tuning["enter_min_x"]
    self.cutin_enter_max_x = cutin_tuning["enter_max_x"]
    self.cutin_enter_min_abs_dpath = cutin_tuning["enter_min_abs_dpath"]
    self.cutin_enter_future_in_lane_prob = cutin_tuning["enter_future_in_lane_prob"]
    self.cutin_enter_centering_gain = cutin_tuning["enter_centering_gain"]
    self.cutin_enter_min_inward_speed = cutin_tuning["enter_min_inward_speed"]
    self.radar_reaction_factor = self.params.get_float("RadarReactionFactor") * 0.01
    self.detect_cut_in = cutin_enabled
    vision_only_mode = self.enable_radar_tracks <= VISION_ONLY_RADAR_TRACK_MODE
    self.cutin_yaw_rate = self._cutin_yaw_rate_from_state(sm)

    leads_v3 = sm['modelV2'].leadsV3
    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    if vision_only_mode:
      self.tracks.clear()
    else:
      previous_corner_sources = {
        tid: trk for tid, trk in self.tracks.items() if trk.measured and self._is_corner_track(trk)
      }
      previous_corner_tracks: dict[int, Track] = {}
      for tid, source in previous_corner_sources.items():
        snapshot = Track(tid)
        snapshot.inherit_cutin_state(source)
        previous_corner_tracks[tid] = snapshot
      current_corner_points = {
        pt.trackId: (float(pt.dRel), float(pt.yRel), float(pt.vRel))
        for pt in rr.points
        if pt.measured and self._radar_point_is_corner(pt)
      }
      previous_corner_positions = {
        tid: (trk.dRel, trk.yRel, trk.vRel) for tid, trk in previous_corner_tracks.items()
      }
      cutin_associations = associate_cutin_tracks(previous_corner_positions, current_corner_points)
      valid_ids = set()
      for pt in rr.points:
        track_id = pt.trackId
        valid_ids.add(track_id)

        if track_id not in self.tracks:
          self.tracks[track_id] = Track(track_id)

        source_id = cutin_associations.get(track_id)
        if source_id is not None and source_id != track_id:
          self.tracks[track_id].inherit_cutin_state(previous_corner_tracks[source_id])

        point_is_corner = self._radar_point_is_corner(pt)
        track_yaw_rate = self.cutin_yaw_rate if point_is_corner else 0.0
        self.tracks[track_id].update(sm['modelV2'], pt, self.ready, self.radar_reaction_factor,
                                     self.radar_lat_factor, track_yaw_rate, point_is_corner, self.v_ego)

      for tid in list(self.tracks.keys()):
        if tid not in valid_ids:
          self.tracks.pop(tid)

    # *** publish radarState ***
    radar_state_valid = sm.all_checks()
    if not radar_state_valid and self.radar_state_valid:
      print("radarState invalid: sm.all_checks() failed")
      for name in sm.data.keys():
        alive = sm.alive.get(name, None)
        valid = sm.valid.get(name, None)
        freq_ok = sm.freq_ok.get(name, None)
        updated = sm.updated.get(name, None)

        if not alive or not valid or not freq_ok:
          print(
            f"  {name}: "
            f"alive={alive}, "
            f"valid={valid}, "
            f"freq_ok={freq_ok}, "
            f"updated={updated}"
          )

    self.radar_state_valid = radar_state_valid
    if not self.radar_state_valid:
      self.radar_state = log.RadarState.new_message()

    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego

    if len(leads_v3) > 1:
      for i in range(2):
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

      md = sm['modelV2']

      corner_radar_enabled = self.enable_corner_radar > 0
      alive_tracks = {tid: trk for tid, trk in self.tracks.items() if trk.measured and trk.cnt > 2 }
      front_tracks = {tid: trk for tid, trk in alive_tracks.items() if not self._is_corner_track(trk)}
      corner_tracks = {tid: trk for tid, trk in alive_tracks.items() if corner_radar_enabled and self._is_corner_track(trk)}
      self.corner_tracks_available = len(corner_tracks) > 0

      self.radar_state.leadOne, self.radar_detected = self.get_lead(sm['carState'], md, front_tracks, 0, leads_v3[0], model_v_ego, self.lead_prob_filters[0].x, low_speed_override=False)
      self.radar_state.leadTwo, _ = self.get_lead(sm['carState'], md, front_tracks, 1, leads_v3[1], model_v_ego, self.lead_prob_filters[1].x, low_speed_override=False)

      self.lane_line_available = md.laneLineProbs[1] > 0.5 and md.laneLineProbs[2] > 0.5
      compute_tracks = dict(front_tracks)
      compute_tracks.update(corner_tracks)
      self.compute_leads(self.v_ego, compute_tracks, md, self.lead_prob_filters[0].x, front_tracks)
      if self.leadTwo is not None:
        self.radar_state.leadTwo = self.leadTwo
      if self.enable_radar_tracks >= 3 or (self.cornerLeadStopped and self.cornerLeadStopped.get("status")):
        self._pick_lead_one_from_state()

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)

  def _is_corner_track(self, t: Track) -> bool:
    return t.is_corner_radar

  def _radar_point_is_corner(self, point: Any) -> bool:
    source = str(point.radarSource)
    return is_corner_radar_source(source) or (
      source == "frontRadar" and self.car_brand == "hyundai" and is_corner_track_id(int(point.trackId))
    )

  def _track_id_is_corner(self, track_id: int) -> bool:
    track = self.tracks.get(track_id)
    return track is not None and self._is_corner_track(track)

  def _cutin_yaw_rate_from_state(self, sm: messaging.SubMaster) -> float:
    yaw_rate = 0.0
    live_pose = sm['livePose'] if 'livePose' in sm.data else None
    if live_pose is not None and live_pose.angularVelocityDevice.valid and live_pose.inputsOK and live_pose.sensorsOK:
      yaw_rate = float(live_pose.angularVelocityDevice.z)
    elif len(sm['modelV2'].orientationRate.z):
      yaw_rate = float(sm['modelV2'].orientationRate.z[0])

    yaw_rate = clamp(yaw_rate, -CUTIN_YAW_COMP_MAX_YAW_RATE, CUTIN_YAW_COMP_MAX_YAW_RATE)
    return float(self.cutin_yaw_rate_filter.update(yaw_rate))

  def _matching_front_track(self, corner: Track, front_tracks: dict[int, Track]) -> Track | None:
    matches = []
    for t in front_tracks.values():
      if not t.measured or t.cnt <= 2:
        continue
      if abs(t.dRel - corner.dRel) > CORNER_FRONT_MATCH_DREL:
        continue
      if abs(t.vRel - corner.vRel) > CORNER_FRONT_MATCH_VREL:
        continue
      matches.append(t)

    if not matches:
      return None

    return min(matches, key=lambda t: abs(t.dRel - corner.dRel) + abs(t.vRel - corner.vRel))

  def _corner_in_lane_ok(self, t: Track, stopped: bool = False, matched_front: bool = False) -> bool:
    if not self.lane_line_available:
      return False

    if stopped:
      dpath_limit = CORNER_STOPPED_NEAR_DPATH_LIMIT
      in_lane_min = CORNER_STOPPED_NEAR_IN_LANE_PROB
      if t.dRel > CORNER_STOPPED_FAR_DREL:
        dpath_limit = CORNER_STOPPED_FAR_DPATH_LIMIT
        in_lane_min = CORNER_STOPPED_FAR_IN_LANE_PROB
      if matched_front:
        in_lane_min = max(0.2, in_lane_min - 0.15)
        dpath_limit += 0.15
      return abs(t.dPath) < dpath_limit and t.in_lane_prob > in_lane_min

    return self._is_center_lead_candidate(t)

  def _is_corner_center_candidate(self, t: Track) -> bool:
    return (
      self._is_corner_track(t) and
      t.cnt >= CORNER_CENTER_MIN_AGE and
      3.0 < t.dRel < RADAR_ONLY_CENTER_MAX_DREL and
      t.vLead > 2.0 and
      self._corner_in_lane_ok(t)
    )

  def _is_corner_stopped_candidate(self, t: Track, matched_front: bool = False) -> bool:
    return (
      self._is_corner_track(t) and
      t.cnt >= CORNER_STOPPED_MIN_AGE and
      CORNER_STOPPED_MIN_DREL < t.dRel < CORNER_STOPPED_MAX_DREL and
      abs(t.vLead) < CORNER_STOPPED_MAX_VLEAD and
      abs(t.yvLead) < CORNER_STOPPED_MAX_YVREL and
      self._corner_in_lane_ok(t, stopped=True, matched_front=matched_front)
    )

  def _corner_track_accel_allowed(self, t: Track) -> bool:
    return (
      t.cnt >= CORNER_ACCEL_MIN_TRACK_AGE and
      self._track_is_closer_than_lead_one(t) and
      abs(t.dPath) < CORNER_ACCEL_MAX_ABS_DPATH and
      math.isfinite(t.aLeadK) and
      abs(t.aLeadK) < CORNER_ACCEL_MAX_ABS_ALEAD
    )

  def _corner_lead_from_track(self, t: Track, model_prob: float = 0.0, vision_y_rel: float = 0.0, use_accel: bool = True) -> dict[str, Any]:
    ld = t.get_RadarState(model_prob, vision_y_rel)
    if use_accel and self._corner_track_accel_allowed(t):
      a_lead = float(np.clip(t.aLeadK, -CORNER_ACCEL_MAX_ABS_ALEAD, CORNER_ACCEL_MAX_ABS_ALEAD))
      ld["aLead"] = a_lead
      ld["aLeadK"] = a_lead
    else:
      ld["aLead"] = 0.0
      ld["aLeadK"] = 0.0
    ld["aLeadTau"] = _LEAD_ACCEL_TAU
    ld["jLead"] = 0.0
    return ld

  def _corner_stopped_lead_from_track(self, t: Track, lead_prob: float) -> dict[str, Any]:
    ld = self._corner_lead_from_track(t, min(0.04, lead_prob), 0.0, use_accel=False)
    ld["modelProb"] = 0.04
    ld["vLead"] = 0.0
    ld["vLeadK"] = 0.0
    ld["vRel"] = -float(self.v_ego)
    return ld

  def get_sticky_track(self, tracks: dict[int, Track]) -> Track | None:
    sticky_tracks = []
    for t in tracks.values():
      if t.selected_count > 0 and abs(t.sticky_dPath) > t.sticky_dpath_limit():
        t.selected_count = 0
        t.is_stopped_car_count = 0
        continue

      if t.measured and t.cnt > 2 and t.selected_count > 0 and 1.0 < t.dRel < 150.0:
        sticky_tracks.append(t)

    if not sticky_tracks:
      return None

    return max(sticky_tracks, key=lambda t: (t.selected_count, -t.dRel))

  def get_lead(self, CS, md, tracks: dict[int, Track], index: int, lead_msg: capnp._DynamicStructReader,
               model_v_ego: float, lead_prob: float, low_speed_override: bool = True) -> dict[str, Any]:

    v_ego = self.v_ego
    ready = self.ready
    if index == 0:
      self.lead_one_front_radar_vision_match = False
    front_radar_vision_match = False

    # VW MEB(ID.4/ID.5): infiniteCable2(=comma) get_lead 정확 복제.
    # carrot의 sticky_track/track_scc 우회 승격을 쓰지 않고, "비전과 sane하게 매칭된 레이더 +
    # 비전단독 + (leadOne만)저속override" 만 사용. -> 정지물체/먼객체가 비전 확인 없이 리드로
    # 승격돼 급제동하던 문제 제거. is_vw_meb 게이트라 타 차종은 아래 carrot 원본 그대로.
    if self.is_vw_meb:
      if len(tracks) > 0 and ready and lead_prob > .5:
        track = match_vision_to_track(v_ego, lead_msg, lead_prob, tracks, update_counters=(index == 0))
      else:
        track = None
      front_radar_vision_match = track is not None
      lead_dict = empty_lead()
      radar = False
      if track is not None:
        vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
        lead_dict = track.get_RadarState(lead_prob, vision_y_rel)
        radar = True
      elif ready and lead_prob > .5:
        lead_dict = get_RadarState_from_vision(md, lead_msg, v_ego, model_v_ego, lead_prob)
      if index == 0:  # infiniteCable2: leadOne만 low_speed_override (저속<4 & 0.75~25m 근접 정지물)
        low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
        if len(low_speed_tracks) > 0:
          closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
          if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
            vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
            lead_dict = closest_track.get_RadarState(lead_prob, vision_y_rel)
            radar = True
            front_radar_vision_match = False
      # 레이더 aLeadK를 모델 가속도 ±MEB_ALEAD_CLAMP_BAND로 제한. vLead 미분 노이즈 스파이크만
      # 깎아 오탐 FCW/급제동을 막고, 실제 앞차 제동(모델도 감지)은 그대로 보존한다.
      if radar and lead_dict.get('status') and lead_prob > .5:
        model_a = float(lead_msg.a[0])
        a_clamped = float(np.clip(lead_dict['aLeadK'], model_a - MEB_ALEAD_CLAMP_BAND, model_a + MEB_ALEAD_CLAMP_BAND))
        lead_dict['aLead'] = a_clamped
        lead_dict['aLeadK'] = a_clamped
      if index == 0 and front_radar_vision_match:
        self.lead_one_front_radar_vision_match = True
      return lead_dict, radar

    ## backup SCC radar(0, 1 trackid)
    if self.enable_radar_tracks <= 0:
      track_scc = tracks.get(0)
    else:
      track_scc = tracks.pop(0, None)

    # Determine leads, this is where the essential logic happens
    if len(tracks) > 0 and ready and lead_prob > .4:
      track = match_vision_to_track(v_ego, lead_msg, lead_prob, tracks, update_counters=(index == 0))
    else:
      track = None
    front_radar_vision_match = track is not None

    sticky_track = False
    if track is None and index == 0 and not self.corner_tracks_available:
      track = self.get_sticky_track(tracks)
      if track is not None:
        sticky_track = True
        front_radar_vision_match = False
        track.selected_count = min(track.selected_count + 1, STICKY_SELECTED_COUNT_MAX)

    if (track is None or (lead_prob < .6 and not sticky_track)) and track_scc is not None and track_scc.cnt > 2:
      #if self.enable_radar_tracks in [-1, 2] or model_v_ego < 5 or track_scc.vLead < 5.0:
      if self.enable_radar_tracks == -1 or (self.enable_radar_tracks >= 2 and track_scc.vLead < 5.0):
        track = track_scc
        front_radar_vision_match = False

    lead_dict = empty_lead()
    radar = False
    if track is not None:
      vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
      lead_dict = track.get_RadarState(lead_prob, vision_y_rel)
      radar = True
    elif (track is None) and ready and (lead_prob > .5):
      lead_dict = get_RadarState_from_vision(md, lead_msg, v_ego, model_v_ego, lead_prob)

    if low_speed_override:
      low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
      if len(low_speed_tracks) > 0:
        closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

        # Only choose new track if it is actually closer than the previous one
        if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
          vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
          lead_dict = closest_track.get_RadarState(lead_prob, vision_y_rel)
          front_radar_vision_match = False

    if index == 0 and front_radar_vision_match:
      self.lead_one_front_radar_vision_match = True
    return lead_dict, radar

  def _cutin_is_closer_or_matches_lead_one(self, t: Track, matched_front: bool = False) -> bool:
    if self._track_is_closer_than_lead_one(t):
      return True
    if not matched_front:
      return False

    lead_one = self.radar_state.leadOne
    if not lead_one.status or not lead_one.radar:
      return False
    if self._track_id_is_corner(int(lead_one.radarTrackId)):
      return False

    return (
      abs(t.dRel - float(lead_one.dRel)) < CORNER_FRONT_MATCH_DREL and
      abs(t.vRel - float(lead_one.vRel)) < CORNER_FRONT_MATCH_VREL
    )

  def _is_cutin_enter_candidate(self, t: Track, matched_front: bool = False) -> bool:
    min_track_age = cutin_min_track_age_frames(
      self.cutin_min_track_age, t.dRel, t.dPath_inward_speed, self.v_ego
    )
    reason = cutin_entry_rejection_reason(
      enabled=self.detect_cut_in,
      lane_line_available=self.lane_line_available,
      corner_track=self._is_corner_track(t),
      closer_or_matching=self._cutin_is_closer_or_matches_lead_one(t, matched_front),
      track_count=t.cnt,
      min_track_age=min_track_age,
      d_rel=t.dRel,
      v_lead=t.vLead,
      d_path=t.dPath,
      d_path_future=t.dPath_future,
      in_lane_prob=t.in_lane_prob,
      in_lane_prob_future=t.in_lane_prob_future,
      inward_speed=t.dPath_inward_speed,
      tuning=self.cutin_tuning,
      fast_lane_entry=is_fast_cutin_entry(
        t.dRel,
        self.v_ego,
        t.dPath,
        t.lane_half_width,
        t.dPath_inward_speed,
        max(0.0, -math.copysign(1.0, t.yRel) * t.yvLead),
      ),
    )
    return reason is None

  def _is_cutin_keep_candidate(self, t: Track, matched_front: bool = False) -> bool:
    if not self.detect_cut_in or not self.lane_line_available or not self._is_corner_track(t):
      return False
    if not self._cutin_is_closer_or_matches_lead_one(t, matched_front):
      return False
    if not (0.8 < t.dRel < 55.0 and t.vLead > 2.0):
      return False

    moving_away = abs(t.dPath_future) - abs(t.dPath)
    if moving_away > CUTIN_KEEP_MAX_MOVING_AWAY:
      return False

    return (
      t.in_lane_prob_future > CUTIN_KEEP_FUTURE_IN_LANE_PROB or
      abs(t.dPath_future) < CUTIN_KEEP_MAX_DPATH_FUTURE
    )

  def _update_cutin_sticky(self, t: Track, matched_front: bool = False) -> bool:
    entering = self._is_cutin_enter_candidate(t, matched_front)
    keeping = t.cut_in_count > 0 and self._is_cutin_keep_candidate(t, matched_front)
    confirm_frames = cutin_confirmation_frames(self.cutin_confirm_frames, t.dRel, t.dPath_inward_speed, self.v_ego)
    t.cut_in_count, t.cut_in_start_abs_dpath = update_cutin_confirmation(
      t.cut_in_count,
      t.cut_in_start_abs_dpath,
      t.dPath,
      t.dRel,
      entering,
      keeping,
      confirm_frames,
      CUTIN_STICKY_FRAMES,
      self.cutin_tuning["enter_min_progress"],
      self.v_ego,
    )

    return t.cut_in_count >= confirm_frames

  def _apply_cutin_output_hold(self, cutin_list: list[dict[str, Any]], tracks: dict[int, Track]) -> list[dict[str, Any]]:
    if cutin_list:
      nearest = min(cutin_list, key=lambda lead: float(lead['dRel']))
      self.cutin_output_hold_reference = (
        float(nearest['dRel']), float(nearest['yRel']), float(nearest['vRel'])
      )
      self.cutin_output_hold_count = CUTIN_OUTPUT_HOLD_FRAMES
      return cutin_list

    reference = self.cutin_output_hold_reference
    if self.cutin_output_hold_count <= 0 or reference is None:
      self.cutin_output_hold_reference = None
      return cutin_list

    d_rel, y_rel, v_rel = reference
    matches = [
      track for track in tracks.values()
      if self._is_corner_track(track) and track.measured
      and abs(track.dRel - d_rel) <= CUTIN_OUTPUT_HOLD_DREL_M
      and abs(track.yRel - y_rel) <= CUTIN_OUTPUT_HOLD_YREL_M
      and abs(track.vRel - v_rel) <= CUTIN_OUTPUT_HOLD_VREL_MPS
    ]
    if not matches:
      self.cutin_output_hold_count = 0
      self.cutin_output_hold_reference = None
      return cutin_list

    track = min(
      matches,
      key=lambda candidate: (
        abs(candidate.dRel - d_rel)
        + abs(candidate.yRel - y_rel)
        + 0.5 * abs(candidate.vRel - v_rel)
      ),
    )
    lead = self._corner_lead_from_track(track, 0, 0)
    lead['modelProb'] = 0.03
    self.cutin_output_hold_reference = (track.dRel, track.yRel, track.vRel)
    self.cutin_output_hold_count -= 1
    if self.cutin_output_hold_count == 0:
      self.cutin_output_hold_reference = None
    return [lead]

  def _cutin_can_replace_lead_one(self, cutin: dict[str, Any]) -> bool:
    lead_one = self.radar_state.leadOne
    if not lead_one.status:
      return True
    if self._lead_one_has_front_radar_vision_match():
      return False

    return cutin["dRel"] + CUTIN_PROMOTE_DREL_MARGIN < lead_one.dRel

  def _track_is_closer_than_lead_one(self, t: Track) -> bool:
    lead_one = self.radar_state.leadOne
    if not lead_one.status:
      return True
    return t.dRel + CUTIN_PROMOTE_DREL_MARGIN < lead_one.dRel

  def _lead_is_closer_than_lead_one(self, lead: dict[str, Any]) -> bool:
    lead_one = self.radar_state.leadOne
    if not lead_one.status:
      return True
    return lead["dRel"] + CUTIN_PROMOTE_DREL_MARGIN < lead_one.dRel

  def _corner_stopped_can_replace_lead_one(self, stopped: dict[str, Any]) -> bool:
    lead_one = self.radar_state.leadOne
    if not lead_one.status:
      return True

    if stopped["dRel"] + self._corner_promote_drel_margin() < lead_one.dRel:
      return True

    if lead_one.radar:
      return False

    vision_prob = lead_one.modelProb if lead_one.status else 0.0
    same_object = abs(stopped["dRel"] - lead_one.dRel) < CORNER_FRONT_MATCH_DREL
    return same_object and vision_prob < CORNER_VISION_KEEP_PROB

  def _corner_promote_drel_margin(self) -> float:
    return CORNER_FRONT_MATCH_PROMOTE_DREL_MARGIN if self._lead_one_has_front_radar_vision_match() else CUTIN_PROMOTE_DREL_MARGIN

  def _corner_lead_clearly_closer_than_lead_one(self, lead: dict[str, Any]) -> bool:
    lead_one = self.radar_state.leadOne
    if not lead_one.status:
      return True
    return lead["dRel"] + CORNER_FRONT_MATCH_PROMOTE_DREL_MARGIN < lead_one.dRel

  def _lead_one_has_front_radar_vision_match(self) -> bool:
    lead_one = self.radar_state.leadOne
    if not self.lead_one_front_radar_vision_match or not lead_one.status or not lead_one.radar:
      return False
    if self._track_id_is_corner(int(lead_one.radarTrackId)):
      return False
    return float(lead_one.modelProb) >= FRONT_RADAR_VISION_MATCH_MIN_PROB

  def _lead_is_corner_track(self, lead: dict[str, Any]) -> bool:
    track_id = int(lead.get("radarTrackId", -1))
    return self._track_id_is_corner(track_id)

  def _is_center_lead_candidate(self, t: Track) -> bool:
    in_lane_min = CENTER_LEAD_NEAR_IN_LANE_PROB
    dpath_limit = CENTER_LEAD_NEAR_DPATH_LIMIT
    if t.dRel > CENTER_LEAD_FAR_DREL:
      in_lane_min = CENTER_LEAD_FAR_IN_LANE_PROB
      dpath_limit = CENTER_LEAD_FAR_DPATH_LIMIT

    return t.in_lane_prob > in_lane_min and abs(t.dPath) < dpath_limit

  def _radar_only_center_ok(self, lead: dict[str, Any]) -> bool:
    d_rel = float(lead.get("dRel", 999.0))
    d_path = abs(float(lead.get("dPath", 999.0)))

    if d_rel > RADAR_ONLY_CENTER_MAX_DREL:
      return False
    if d_rel > RADAR_ONLY_CENTER_FAR_DREL:
      return d_path < RADAR_ONLY_CENTER_DPATH_FAR_LIMIT
    if d_rel > RADAR_ONLY_CENTER_MID_DREL:
      return d_path < RADAR_ONLY_CENTER_DPATH_MID_LIMIT
    return d_path < RADAR_ONLY_CENTER_DPATH_NEAR_LIMIT

  def compute_leads(self, v_ego, tracks, md, lead_prob, front_tracks: dict[int, Track] | None = None):
    self.leadCenter = None
    self.leadTwo = None
    self.leadCutIn = empty_lead()
    self.cornerLeadStopped = empty_lead()
    front_tracks = front_tracks or {}

    lead_msg = md.leadsV3[0] if (md is not None and len(md.position.x) == 33) else None
    if lead_msg is None:
      # reset
      self.cutin_output_hold_count = 0
      self.cutin_output_hold_reference = None
      self.radar_state.leadsLeft = []
      self.radar_state.leadsCenter = []
      self.radar_state.leadsRight = []
      self.radar_state.leadsCutIn = []
      self.radar_state.leadsLeft2 = []
      self.radar_state.leadsRight2 = []
      self.radar_state.leadLeft = empty_lead()
      self.radar_state.leadRight = empty_lead()
      return

    front_left_list, front_right_list = [], []
    corner_left_list, corner_right_list = [], []
    center_list, cutin_list = [], []
    corner_center_list, corner_stopped_list = [], []
    for c in tracks.values():
      y_rel_neg = - c.yRel
      is_corner = self._is_corner_track(c)
      matching_front = self._matching_front_track(c, front_tracks) if is_corner else None
      # center
      if self._is_center_lead_candidate(c):
        c.cut_in_count = max(c.cut_in_count - 1, 0)
        if c.cnt > 3:
          ld = self._corner_lead_from_track(c, lead_prob, float(-lead_msg.y[0])) if is_corner else c.get_RadarState(lead_prob, float(-lead_msg.y[0]))
          ld['modelProb'] = 0.01
          center_list.append(ld)
          if self._is_corner_center_candidate(c):
            corner_center_list.append(ld)

      if self._is_corner_stopped_candidate(c, matched_front=matching_front is not None):
        corner_stopped_list.append(self._corner_stopped_lead_from_track(c, lead_prob))

      # left/right
      if self._is_center_lead_candidate(c):
        continue
      elif y_rel_neg < 0: #left_lane_y:
        ld = self._corner_lead_from_track(c, 0, 0) if is_corner else c.get_RadarState(0, 0)
        if self._update_cutin_sticky(c, matching_front is not None):
          ld['modelProb'] = 0.03
          cutin_list.append(ld)
        if is_corner:
          corner_left_list.append(ld)
        else:
          front_left_list.append(ld)
      else:
        ld = self._corner_lead_from_track(c, 0, 0) if is_corner else c.get_RadarState(0, 0)
        if self._update_cutin_sticky(c, matching_front is not None):
          ld['modelProb'] = 0.03
          cutin_list.append(ld)
        if is_corner:
          corner_right_list.append(ld)
        else:
          front_right_list.append(ld)

    left_list = select_side_leads(front_left_list, corner_left_list, self.corner_tracks_available)
    right_list = select_side_leads(front_right_list, corner_right_list, self.corner_tracks_available)
    cutin_list = self._apply_cutin_output_hold(cutin_list, tracks)

    self.radar_state.leadsLeft   = left_list
    self.radar_state.leadsRight  = right_list
    self.radar_state.leadsCenter = center_list
    self.radar_state.leadsCutIn = cutin_list
    self.leadCutIn = min(
      (ld for ld in cutin_list if self.cutin_enter_min_x < ld['dRel'] < self.cutin_enter_max_x and ld['vLead'] > 4),
      key=lambda d: d['dRel'],
      default=empty_lead()
    )
    self.cornerLeadStopped = min(
      corner_stopped_list,
      key=lambda d: d['dRel'],
      default=empty_lead()
    )

    self.radar_state.leadLeft = pick_side_lead(left_list)
    self.radar_state.leadRight = pick_side_lead(right_list)

    self.leadTwo = None
    if self.lane_line_available:
      self.leadCenter = min(
          (ld for ld in center_list if ld['vLead'] > 5 and ld['radar'] and ld['dRel'] > 3.5),
          key=lambda d: d['dRel'],
          default=None
      )
      if self.radar_state.leadOne.status and self.radar_state.leadOne.radar:
        self.leadTwo = min(
            (ld for ld in center_list if ld['vLead'] > 5 and ld['radar'] and not self._lead_is_corner_track(ld) and self.radar_state.leadOne.dRel < ld['dRel'] < 80),
            key=lambda d: d['dRel'],
            default=None
        )
        if self.leadTwo is not None:
          self.leadTwo = copy.deepcopy(self.leadTwo)
          #gap = self.leadTwo['dRel'] - self.radar_state.leadOne.dRel
          #offset = 3.0 + min(gap * 0.2, 10)
          #self.leadTwo['dRel'] = self.radar_state.leadOne.dRel + offset
          self.leadTwo['dRel'] = max(self.radar_state.leadOne.dRel + 3.0, self.leadTwo['dRel'] - 8.0) # lead+1 차를 뒤로 8M후퇴하여, mpc에서  감자하도록함.. 최소 lead보다 3M앞에 위치하도록
    else:
      self.leadCenter = None

    if self.leadCutIn and self.leadCutIn.get("status") and self.detect_cut_in:
      self.leadTwo = copy.deepcopy(self.leadCutIn)
      self.leadTwo["modelProb"] = 0.03

    def _ok(ld):
        return (ld.get('vLead', 0) > 2 and
                abs(ld.get('dPath', 0)) < 4.2 and
                ld.get('dRel', 0) > 2)

    def _pick_two_with_gap(cands, min_gap=5.0):
        xs = sorted((ld for ld in cands if _ok(ld)), key=lambda d: d['dRel'])
        if not xs:
            return []
        first = xs[0]
        second = None
        for ld in xs[1:]:
            # 5m 이상 떨어진 후보만 허용 (>= 5.0)
            if (ld['dRel'] - first['dRel']) >= min_gap:
                second = ld
                break
        return [first] if second is None else [first, second]

    self.radar_state.leadsLeft2  = _pick_two_with_gap(left_list,  min_gap=5.0)
    self.radar_state.leadsRight2 = _pick_two_with_gap(right_list, min_gap=5.0)

  def _pick_lead_one_from_state(self):
    chosen = None
    detected = self.radar_detected

    if (self.leadCenter and self.leadCenter["status"] and
        not self._lead_is_corner_track(self.leadCenter) and
        is_radar_center_promotion_safe(self.leadCenter)):
      lead_one = self.radar_state.leadOne
      vision_prob = lead_one.modelProb if lead_one.status else 0.0

      if self.radar_detected:
        if lead_one.status and self.leadCenter["dRel"] + self._corner_promote_drel_margin() < lead_one.dRel:
          chosen = self.leadCenter
          chosen["modelProb"] = 0.01
      else:
        radar_clearly_closer = lead_one.status and self.leadCenter["dRel"] + self._corner_promote_drel_margin() < lead_one.dRel
        vision_weak_or_missing = (not lead_one.status) or vision_prob < RADAR_ONLY_FALLBACK_VISION_PROB

        if vision_weak_or_missing and (not lead_one.status or radar_clearly_closer) and self._radar_only_center_ok(self.leadCenter):
          chosen = self.leadCenter
          chosen["modelProb"] = 0.02
          detected = True

    if chosen is not None:
        self.radar_state.leadOne = chosen
        self.radar_detected = detected


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks', 'livePose'], poll='modelV2',
                           ignore_alive=['livePose'], ignore_valid=['livePose'])
  #sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'])
  pm = messaging.PubMaster(['radarState'])

  # VW MEB(ID.4/ID.5)만 infiniteCable2식 리드선택. 타 차종은 carrot 원본.
  RD = RadarD(CP.radarDelay, is_vw_meb=is_volkswagen_meb(CP), car_brand=CP.brand)

  while 1:
    sm.update()

    if sm.updated['modelV2']:
      RD.update(sm, sm['liveTracks'])
      RD.publish(pm)


if __name__ == "__main__":
  main()
