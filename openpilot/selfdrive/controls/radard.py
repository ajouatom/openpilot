#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any
import copy

import capnp
from openpilot.cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import is_volkswagen_meb
from openpilot.selfdrive.controls.lib.cutin_helpers import (
  associate_cutin_tracks,
  combine_cutin_future_projection,
  CORNER_CUTIN_MAX_DREL_M,
  cutin_confirmation_frames,
  cutin_min_track_age_frames,
  cutin_entry_rejection_reason,
  cutin_tuning_from_sensitivity,
  effective_cutin_inward_speed,
  hold_side_corner_front_matches,
  is_corner_confirmed_near_cutin,
  is_cutin_track_discontinuous,
  FRONT_CUTIN_MIN_CONFIRM_S,
  is_corner_radar_source,
  is_corner_track_id,
  is_fast_cutin_entry,
  is_front_radar_cutin_candidate,
  is_front_radar_cutin_enabled,
  match_side_corner_to_front_tracks,
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
CUTIN_FIXED_SENSITIVITY = 50.0
CUTIN_YAW_COMP_GAIN = 0.6
CUTIN_YAW_COMP_MAX_DREL = 50.0
CUTIN_YAW_COMP_MAX_YAW_RATE = 0.35
CUTIN_YAW_COMP_MAX_YVREL_CORRECTION = 1.5
CUTIN_YAW_COMP_MAX_VREL_CORRECTION = 0.6

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
# A newly appearing close, stopped corner-radar return can be a roadside
# reflection sweeping through the model path. Require it to have first been
# qualified at long range, unless front radar independently corroborates it.
CORNER_STOPPED_UNMATCHED_MIN_ACQUISITION_DREL = 70.0

def laplacian_pdf(x: float, mu: float, b: float):
  diff = abs(x - mu) / max(b, 1e-4)
  return 0.0 if diff > 50.0 else math.exp(-diff)


def is_vision_radar_lateral_match_sane(radar_y_rel: float, vision_y_rel: float, d_path: float) -> bool:
  return abs(radar_y_rel - vision_y_rel) < 2.0 or abs(d_path) < 2.4


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
    self.cutin_cnt = 0
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
    self.path_dPath = 0.0
    self.path_dPath_future = 0.0
    self.path_in_lane_prob = 0.0
    self.path_in_lane_prob_future = 0.0
    self.path_dPath_rate = 0.0
    self.path_inward_speed = 0.0
    self.path_y_std = float('inf')
    self._cutin_path_position_history = new_cutin_position_history(DT_MDL)
    self.side_corner_confirmed_count = 0
    self.cutin_radar_inward_speed = 0.0
    self.sticky_dPath = 0.0
    self.sticky_path_y_std = 0.0
    self.corner_stopped_acquired = False

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
    self.cutin_cnt = source.cutin_cnt
    self.cut_in_start_abs_dpath = source.cut_in_start_abs_dpath
    self._cutin_position_history.clear()
    self._cutin_position_history.extend(source._cutin_position_history)
    self.path_dPath = source.path_dPath
    self.path_dPath_future = source.path_dPath_future
    self.path_in_lane_prob = source.path_in_lane_prob
    self.path_in_lane_prob_future = source.path_in_lane_prob_future
    self.path_dPath_rate = source.path_dPath_rate
    self.path_inward_speed = source.path_inward_speed
    self.path_y_std = source.path_y_std
    self._cutin_path_position_history.clear()
    self._cutin_path_position_history.extend(source._cutin_path_position_history)
    self.side_corner_confirmed_count = source.side_corner_confirmed_count
    self.cutin_radar_inward_speed = source.cutin_radar_inward_speed

  def update(self, md, pt, ready, radar_reaction_factor, radar_lat_factor, yaw_rate, is_corner_radar=False,
             is_cutin_track=False, v_ego=0.0, side_corner_confirmed=False):
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
    track_discontinuous = (
      is_cutin_track_discontinuous(prev_measured, prev_dRel, prev_yRel, prev_vLead,
                                   self.dRel, self.yRel, self.vLead)
      if is_cutin_track
      else prev_measured and (
        abs(self.dRel - prev_dRel) > 5.0 or
        abs(self.yRel - prev_yRel) > 2.0 or
        abs(self.vLead - prev_vLead) > 7.0
      )
    )
    if not self.measured:
      self.cnt = 0
      self.selected_count = 0
      self.is_stopped_car_count = 0
      self.cut_in_count = 0
      self.cutin_cnt = 0
      self.cut_in_start_abs_dpath = 0.0
      self._cutin_position_history.clear()
      self._cutin_path_position_history.clear()
      self.side_corner_confirmed_count = 0
      self.corner_stopped_acquired = False
      # optional: also reset filter init when track is not measured
      self._vLead_filt_init = False
    elif track_discontinuous:
      self.cnt = 0
      self.selected_count = 0
      self.is_stopped_car_count = 0
      self.cut_in_count = 0
      self.cutin_cnt = 0
      self.cut_in_start_abs_dpath = 0.0
      self._cutin_position_history.clear()
      self._cutin_path_position_history.clear()
      self.corner_stopped_acquired = False

    if not is_corner_radar:
      self.corner_stopped_acquired = False

    if self.measured and side_corner_confirmed:
      self.side_corner_confirmed_count += 1
    elif not side_corner_confirmed:
      self.side_corner_confirmed_count = 0

    if self.measured and is_cutin_track:
      self.cutin_cnt += 1
    elif not is_cutin_track:
      self.cut_in_count = 0
      self.cutin_cnt = 0
      self.cut_in_start_abs_dpath = 0.0
      self._cutin_path_position_history.clear()

    v_rel_future, yv_rel_future = self.yaw_compensated_velocities(yaw_rate)
    self.dRel_future = self.dRel + v_rel_future * radar_lat_factor
    self.yRel_future = self.yRel + yv_rel_future * radar_lat_factor
    if ready:
      self.d_path(md)
      if side_corner_confirmed:
        self.cutin_path_d_path(md)
      if is_cutin_track and radar_lat_factor > 0.0:
        self.cutin_radar_inward_speed = max(
          0.0, -math.copysign(1.0, self.dPath) * yv_rel_future
        )
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
          self.cutin_radar_inward_speed,
        )
        self.dPath_inward_speed = effective_cutin_inward_speed(
          self.dRel,
          v_ego=v_ego,
          temporal_inward_speed=self.dPath_inward_speed,
          d_path=self.dPath,
          projected_d_path=self.dPath_future,
          horizon_s=radar_lat_factor,
        )
        if side_corner_confirmed and len(md.position.x) >= 2 and len(md.position.y) == len(md.position.x):
          self.path_dPath_rate, self.path_inward_speed = update_lane_relative_motion(
            self._cutin_path_position_history,
            self.dRel,
            self.yRel,
            md.position.x,
            md.position.y,
            md.position.y,
            self.measured,
            track_discontinuous,
            DT_MDL,
          )
          self.path_dPath_future, self.path_in_lane_prob_future = combine_cutin_future_projection(
            self.path_dPath,
            self.path_dPath_rate,
            radar_lat_factor,
            self.lane_half_width,
            self.path_dPath_future,
            self.path_in_lane_prob_future,
            max(0.0, -math.copysign(1.0, self.path_dPath) * yv_rel_future),
          )
          self.path_inward_speed = effective_cutin_inward_speed(
            self.dRel,
            v_ego=v_ego,
            temporal_inward_speed=self.path_inward_speed,
            d_path=self.path_dPath,
            projected_d_path=self.path_dPath_future,
            horizon_s=radar_lat_factor,
          )
          self.cutin_radar_inward_speed = max(
            0.0, -math.copysign(1.0, self.path_dPath) * yv_rel_future
          )
        else:
          self._cutin_path_position_history.clear()
          self.path_dPath_rate = 0.0
          self.path_inward_speed = 0.0
      else:
        self._cutin_position_history.clear()
        self._cutin_path_position_history.clear()
        self.dPath_rate = 0.0
        self.dPath_inward_speed = 0.0
        self.path_dPath_rate = 0.0
        self.path_inward_speed = 0.0
        self.cutin_radar_inward_speed = 0.0
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

  def cutin_path_d_path(self, md) -> None:
    if len(md.position.x) < 2 or len(md.position.y) != len(md.position.x):
      self.path_dPath = self.dPath
      self.path_dPath_future = self.dPath_future
      self.path_in_lane_prob = self.in_lane_prob
      self.path_in_lane_prob_future = self.in_lane_prob_future
      self.path_y_std = float('inf')
      return

    path_y = float(np.interp(self.dRel, md.position.x, md.position.y))
    path_y_future = float(np.interp(self.dRel_future, md.position.x, md.position.y))
    self.path_dPath = float(self.yRel + path_y)
    self.path_dPath_future = float(self.yRel_future + path_y_future)
    self.path_in_lane_prob = max(0.0, 1.0 - abs(self.path_dPath) / self.lane_half_width)
    self.path_in_lane_prob_future = max(0.0, 1.0 - abs(self.path_dPath_future) / self.lane_half_width)
    self.path_y_std = (
      float(np.interp(self.dRel, md.position.x, md.position.yStd))
      if len(md.position.yStd) == len(md.position.x)
      else float('inf')
    )

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
    if not is_vision_radar_lateral_match_sane(t.yRel, -float(lead.y[0]), t.dPath):
      continue

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
    elif lead_prob > 0.6 and abs(first_track.dPath) < 2.4:
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
    self.front_cutin_enabled = False
    self.corner_cutin_enabled = False
    self.radar_lat_factor = 0.0
    self.cutin_yaw_rate = 0.0
    self.cutin_yaw_rate_filter = FirstOrderFilter(0.0, 0.20, DT_MDL)
    self.cutin_tuning = cutin_tuning_from_sensitivity(CUTIN_FIXED_SENSITIVITY)
    self.cutin_confirm_frames = max(1, int(round(self.cutin_tuning["confirm_s"] / DT_MDL)))
    self.front_cutin_confirm_frames = max(
      self.cutin_confirm_frames,
      int(round(FRONT_CUTIN_MIN_CONFIRM_S / DT_MDL)),
    )
    self.cutin_min_track_age = max(1, int(round(self.cutin_tuning["min_track_age_s"] / DT_MDL)))
    self.cutin_enter_min_x = self.cutin_tuning["enter_min_x"]
    self.cutin_enter_max_x = self.cutin_tuning["enter_max_x"]
    self.cutin_enter_min_abs_dpath = self.cutin_tuning["enter_min_abs_dpath"]
    self.cutin_enter_future_in_lane_prob = self.cutin_tuning["enter_future_in_lane_prob"]
    self.cutin_enter_centering_gain = self.cutin_tuning["enter_centering_gain"]
    self.cutin_enter_min_inward_speed = self.cutin_tuning["enter_min_inward_speed"]

    self.radar_detected = False
    self.leadCenter = None
    self.leadTwo = None
    self.leadCutIn = empty_lead()
    self.cutin_output_hold_count = 0
    self.cutin_output_hold_reference: tuple[float, float, float] | None = None
    self.cornerLeadStopped = empty_lead()
    self.corner_tracks_available = False
    self.side_corner_front_matches: dict[int, int] = {}
    self.side_corner_front_match_misses: dict[int, int] = {}

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    self.enable_radar_tracks = self.params.get_int("EnableRadarTracks")
    self.enable_corner_radar = self.params.get_int("EnableCornerRadar")
    self.corner_cutin_enabled = self.enable_corner_radar > 1 and self.car_brand == "hyundai"
    self.front_cutin_enabled = is_front_radar_cutin_enabled(
      self.enable_radar_tracks, self.enable_corner_radar, self.car_brand
    )
    cutin_enabled = self.corner_cutin_enabled or self.front_cutin_enabled
    if self.is_vw_meb:
      # VW MEB(ID.4): 코너레이더가 없어 전방 레이더 트랙으로 끼어들기 판정 (구 carrot 검증 로직의 MEB 이식).
      # RadarLatFactor(기본 0)로 옵트인. 타 차종은 코너레이더+현대 조건 그대로.
      cutin_enabled = self.params.get_float("RadarLatFactor") > 0.0
      self.corner_cutin_enabled = False
      self.front_cutin_enabled = False
    self.radar_lat_factor = self.cutin_tuning["horizon_s"] if cutin_enabled else 0.0
    self.radar_reaction_factor = self.params.get_float("RadarReactionFactor") * 0.01
    self.detect_cut_in = cutin_enabled
    if self.corner_cutin_enabled:
      current_side_matches = self._side_corner_front_matches(rr.points)
      available_front_ids = {
        int(point.trackId) for point in rr.points
        if point.measured and not self._radar_point_is_corner(point)
      }
      self.side_corner_front_matches, self.side_corner_front_match_misses = hold_side_corner_front_matches(
        current_side_matches,
        self.side_corner_front_matches,
        self.side_corner_front_match_misses,
        available_front_ids,
      )
    else:
      self.side_corner_front_matches = {}
      self.side_corner_front_match_misses = {}
    vision_only_mode = self.enable_radar_tracks <= VISION_ONLY_RADAR_TRACK_MODE
    self.cutin_yaw_rate = self._cutin_yaw_rate_from_state(sm) if cutin_enabled else 0.0

    leads_v3 = sm['modelV2'].leadsV3
    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    if vision_only_mode:
      self.tracks.clear()
    else:
      previous_cutin_tracks: dict[int, Track] = {}
      cutin_associations: dict[int, int] = {}
      if cutin_enabled:
        previous_cutin_sources = {
          tid: trk for tid, trk in self.tracks.items() if trk.measured and self._is_active_cutin_track(trk)
        }
        for tid, source in previous_cutin_sources.items():
          snapshot = Track(tid)
          snapshot.inherit_cutin_state(source)
          previous_cutin_tracks[tid] = snapshot
        current_cutin_points = {
          pt.trackId: (float(pt.dRel), float(pt.yRel), float(pt.vRel))
          for pt in rr.points
          if pt.measured and self._radar_point_is_active_cutin(pt)
        }
        previous_cutin_positions = {
          tid: (trk.dRel, trk.yRel, trk.vRel) for tid, trk in previous_cutin_tracks.items()
        }
        cutin_associations = associate_cutin_tracks(previous_cutin_positions, current_cutin_points)
      valid_ids = set()
      for pt in rr.points:
        track_id = pt.trackId
        valid_ids.add(track_id)

        if track_id not in self.tracks:
          self.tracks[track_id] = Track(track_id)

        source_id = cutin_associations.get(track_id)
        if source_id is not None and source_id != track_id:
          self.tracks[track_id].inherit_cutin_state(previous_cutin_tracks[source_id])

        point_is_corner = self._radar_point_is_corner(pt)
        point_is_cutin = self._radar_point_is_active_cutin(pt)
        track_yaw_rate = self.cutin_yaw_rate if point_is_cutin else 0.0
        self.tracks[track_id].update(sm['modelV2'], pt, self.ready, self.radar_reaction_factor,
                                     self.radar_lat_factor, track_yaw_rate, point_is_corner, point_is_cutin, self.v_ego,
                                     track_id in self.side_corner_front_matches)

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

      self.radar_state.leadOne, self.radar_detected = self.get_lead(
        sm['carState'], md, front_tracks, 0, leads_v3[0], model_v_ego,
        self.lead_prob_filters[0].x, low_speed_override=False, vision_match_only=True,
      )
      self.radar_state.leadTwo = empty_lead()

      self.lane_line_available = md.laneLineProbs[1] > 0.5 and md.laneLineProbs[2] > 0.5
      compute_tracks = dict(front_tracks)
      compute_tracks.update(corner_tracks)
      self.compute_leads(self.v_ego, compute_tracks, md, self.lead_prob_filters[0].x, front_tracks)
      if self.leadTwo is not None:
        self.radar_state.leadTwo = self.leadTwo

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)

  def _is_corner_track(self, t: Track) -> bool:
    return t.is_corner_radar

  def _is_front_cutin_track(self, t: Track) -> bool:
    return is_front_radar_cutin_candidate(
      t.identifier, t.radar_source, t.dRel, t.yRel, self._is_corner_track(t)
    )

  def _is_active_cutin_track(self, t: Track) -> bool:
    if self.front_cutin_enabled:
      return self._is_front_cutin_track(t)
    return self.corner_cutin_enabled and (
      self._is_corner_track(t) or t.identifier in self.side_corner_front_matches
    )

  def _side_corner_front_matches(self, points: Any) -> dict[int, int]:
    corner_tracks = {
      int(point.trackId): (float(point.dRel), float(point.yRel), float(point.vRel))
      for point in points
      if point.measured and self._radar_point_is_corner(point)
    }
    front_tracks = {
      int(point.trackId): (float(point.dRel), float(point.yRel), float(point.vRel))
      for point in points
      if point.measured and not self._radar_point_is_corner(point)
      and str(point.radarSource) != "scc" and int(point.trackId) != 0
    }
    return match_side_corner_to_front_tracks(corner_tracks, front_tracks)

  def _radar_point_is_corner(self, point: Any) -> bool:
    source = str(point.radarSource)
    return is_corner_radar_source(source) or (
      source == "frontRadar" and self.car_brand == "hyundai" and is_corner_track_id(int(point.trackId))
    )

  def _radar_point_is_front_cutin(self, point: Any) -> bool:
    source = str(point.radarSource)
    return is_front_radar_cutin_candidate(
      int(point.trackId), source, float(point.dRel), float(point.yRel), self._radar_point_is_corner(point)
    )

  def _radar_point_is_active_cutin(self, point: Any) -> bool:
    if self.front_cutin_enabled:
      return self._radar_point_is_front_cutin(point)
    return self.corner_cutin_enabled and (
      self._radar_point_is_corner(point) or int(point.trackId) in self.side_corner_front_matches
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

  def _is_corner_center_candidate(self, t: Track, matched_front: bool = False) -> bool:
    return (
      self._is_corner_track(t) and
      t.cnt >= CORNER_CENTER_MIN_AGE and
      3.0 < t.dRel < RADAR_ONLY_CENTER_MAX_DREL and
      (matched_front or t.cut_in_count > 0) and
      t.vLead > 2.0 and
      self._corner_in_lane_ok(t)
    )

  def _is_corner_stopped_candidate(self, t: Track, matched_front: bool = False) -> bool:
    qualifies = (
      self._is_corner_track(t) and
      t.cnt >= CORNER_STOPPED_MIN_AGE and
      CORNER_STOPPED_MIN_DREL < t.dRel < CORNER_STOPPED_MAX_DREL and
      abs(t.vLead) < CORNER_STOPPED_MAX_VLEAD and
      abs(t.yvLead) < CORNER_STOPPED_MAX_YVREL and
      self._corner_in_lane_ok(t, stopped=True, matched_front=matched_front)
    )
    if not qualifies:
      return False

    if matched_front or t.dRel >= CORNER_STOPPED_UNMATCHED_MIN_ACQUISITION_DREL:
      t.corner_stopped_acquired = True

    return t.corner_stopped_acquired

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
               model_v_ego: float, lead_prob: float, low_speed_override: bool = True,
               vision_match_only: bool = False) -> dict[str, Any]:

    v_ego = self.v_ego
    ready = self.ready

    # VW MEB(ID.4/ID.5): infiniteCable2(=comma) get_lead 정확 복제.
    # carrot의 sticky_track/track_scc 우회 승격을 쓰지 않고, "비전과 sane하게 매칭된 레이더 +
    # 비전단독 + (leadOne만)저속override" 만 사용. -> 정지물체/먼객체가 비전 확인 없이 리드로
    # 승격돼 급제동하던 문제 제거. is_vw_meb 게이트라 타 차종은 아래 carrot 원본 그대로.
    if self.is_vw_meb:
      if len(tracks) > 0 and ready and lead_prob > .5:
        track = match_vision_to_track(v_ego, lead_msg, lead_prob, tracks, update_counters=(index == 0))
      else:
        track = None
      lead_dict = empty_lead()
      radar = False
      if track is not None:
        vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
        lead_dict = track.get_RadarState(lead_prob, vision_y_rel)
        radar = True
      elif ready and lead_prob > .5:
        lead_dict = get_RadarState_from_vision(md, lead_msg, v_ego, model_v_ego, lead_prob)
      if index == 0 and not vision_match_only:  # infiniteCable2: leadOne만 low_speed_override (저속<4 & 0.75~25m 근접 정지물)
        low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
        if len(low_speed_tracks) > 0:
          closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
          if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
            vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
            lead_dict = closest_track.get_RadarState(lead_prob, vision_y_rel)
            radar = True
      # 레이더 aLeadK를 모델 가속도 ±MEB_ALEAD_CLAMP_BAND로 제한. vLead 미분 노이즈 스파이크만
      # 깎아 오탐 FCW/급제동을 막고, 실제 앞차 제동(모델도 감지)은 그대로 보존한다.
      if radar and lead_dict.get('status') and lead_prob > .5:
        model_a = float(lead_msg.a[0])
        a_clamped = float(np.clip(lead_dict['aLeadK'], model_a - MEB_ALEAD_CLAMP_BAND, model_a + MEB_ALEAD_CLAMP_BAND))
        lead_dict['aLead'] = a_clamped
        lead_dict['aLeadK'] = a_clamped
      return lead_dict, radar

    ## backup SCC radar(0, 1 trackid)
    track_scc = tracks.get(0)
    match_tracks = tracks if self.enable_radar_tracks <= 0 else {
      track_id: track for track_id, track in tracks.items() if track_id != 0
    }

    # Determine leads, this is where the essential logic happens
    if len(match_tracks) > 0 and ready and lead_prob > .4:
      track = match_vision_to_track(v_ego, lead_msg, lead_prob, match_tracks, update_counters=(index == 0))
    else:
      track = None
    sticky_track = False
    if track is None and index == 0 and not vision_match_only and not self.corner_tracks_available:
      track = self.get_sticky_track(match_tracks)
      if track is not None:
        sticky_track = True
        track.selected_count = min(track.selected_count + 1, STICKY_SELECTED_COUNT_MAX)

    if (not vision_match_only and (track is None or (lead_prob < .6 and not sticky_track))
        and track_scc is not None and track_scc.cnt > 2):
      #if self.enable_radar_tracks in [-1, 2] or model_v_ego < 5 or track_scc.vLead < 5.0:
      if self.enable_radar_tracks == -1 or (self.enable_radar_tracks >= 2 and track_scc.vLead < 5.0):
        track = track_scc

    lead_dict = empty_lead()
    radar = False
    if track is not None:
      vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
      lead_dict = track.get_RadarState(lead_prob, vision_y_rel)
      radar = True
    elif (track is None) and ready and (lead_prob > .5):
      lead_dict = get_RadarState_from_vision(md, lead_msg, v_ego, model_v_ego, lead_prob)

    if low_speed_override and not vision_match_only:
      low_speed_tracks = [c for c in match_tracks.values() if c.potential_low_speed_lead(v_ego)]
      if len(low_speed_tracks) > 0:
        closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

        # Only choose new track if it is actually closer than the previous one
        if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
          vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
          lead_dict = closest_track.get_RadarState(lead_prob, vision_y_rel)
    return lead_dict, radar

  def _cutin_is_closer_or_matches_lead_one(self, t: Track, matched_front: bool = False) -> bool:
    if self._track_is_closer_than_lead_one(t):
      return True
    if self.front_cutin_enabled:
      lead_one = self.radar_state.leadOne
      return bool(
        lead_one.status and lead_one.radar and int(lead_one.radarTrackId) == t.identifier
      )
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
    if t.identifier in self.side_corner_front_matches:
      return is_corner_confirmed_near_cutin(
        confirmed_frames=t.side_corner_confirmed_count,
        d_rel=t.dRel,
        v_lead=t.vLead,
        d_path=t.path_dPath,
        d_path_future=t.path_dPath_future,
        inward_speed=t.path_inward_speed,
        radar_inward_speed=t.cutin_radar_inward_speed,
        path_y_std=t.path_y_std,
      )

    min_track_age = cutin_min_track_age_frames(
      self.cutin_min_track_age, t.dRel, t.dPath_inward_speed, self.v_ego
    )
    track_count = t.cutin_cnt if self.front_cutin_enabled else t.cnt
    reason = cutin_entry_rejection_reason(
      enabled=self.detect_cut_in,
      lane_line_available=self.lane_line_available,
      # MEB는 전방 레이더 트랙도 후보 허용 (코너레이더 부재), 타 차종은 코너 트랙만
      corner_track=self._is_active_cutin_track(t) or self.is_vw_meb,
      closer_or_matching=self._cutin_is_closer_or_matches_lead_one(t, matched_front),
      track_count=track_count,
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
        t.cutin_radar_inward_speed,
        v_rel=t.vRel,
      ),
      radar_inward_speed=t.cutin_radar_inward_speed,
      max_d_rel=CORNER_CUTIN_MAX_DREL_M if self._is_corner_track(t) else None,
    )
    return reason is None

  def _is_cutin_keep_candidate(self, t: Track, matched_front: bool = False) -> bool:
    if not self.detect_cut_in or not (self._is_active_cutin_track(t) or self.is_vw_meb):
      return False
    if t.identifier in self.side_corner_front_matches:
      moving_away = abs(t.path_dPath_future) - abs(t.path_dPath)
      return (
        t.side_corner_confirmed_count > 0 and
        0.8 < t.dRel < 8.0 and
        t.vLead > 0.0 and
        t.path_y_std <= 0.8 and
        moving_away <= CUTIN_KEEP_MAX_MOVING_AWAY and
        (t.path_in_lane_prob_future > CUTIN_KEEP_FUTURE_IN_LANE_PROB or abs(t.path_dPath_future) < CUTIN_KEEP_MAX_DPATH_FUTURE)
      )
    if not self.front_cutin_enabled and not self.lane_line_available:
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
    side_corner_confirmed = t.identifier in self.side_corner_front_matches
    if (self.front_cutin_enabled or side_corner_confirmed) and keeping:
      entering = True
    confirmation_d_path = t.path_dPath if side_corner_confirmed else t.dPath
    confirmation_inward_speed = t.path_inward_speed if side_corner_confirmed else t.dPath_inward_speed
    base_confirm_frames = self.front_cutin_confirm_frames if self.front_cutin_enabled else self.cutin_confirm_frames
    confirm_frames = cutin_confirmation_frames(base_confirm_frames, t.dRel, confirmation_inward_speed, self.v_ego)
    t.cut_in_count, t.cut_in_start_abs_dpath = update_cutin_confirmation(
      t.cut_in_count,
      t.cut_in_start_abs_dpath,
      confirmation_d_path,
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
      if self._is_active_cutin_track(track) and track.measured
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
    lead = self._corner_lead_from_track(track, 0, 0) if self._is_corner_track(track) else track.get_RadarState(0, 0)
    lead['modelProb'] = 0.03
    self.cutin_output_hold_reference = (track.dRel, track.yRel, track.vRel)
    self.cutin_output_hold_count -= 1
    if self.cutin_output_hold_count == 0:
      self.cutin_output_hold_reference = None
    return [lead]

  def _track_is_closer_than_lead_one(self, t: Track) -> bool:
    lead_one = self.radar_state.leadOne
    if not lead_one.status:
      return True
    return t.dRel + CUTIN_PROMOTE_DREL_MARGIN < lead_one.dRel

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
    corner_stopped_list = []
    for c in tracks.values():
      y_rel_neg = - c.yRel
      is_corner = self._is_corner_track(c)
      matching_front = self._matching_front_track(c, front_tracks) if is_corner else None
      # center
      if self._is_center_lead_candidate(c):
        c.cut_in_count = max(c.cut_in_count - 1, 0)
        center_usable = not is_corner or self._is_corner_center_candidate(c, matching_front is not None)
        if c.cnt > 3 and center_usable:
          ld = self._corner_lead_from_track(c, lead_prob, float(-lead_msg.y[0])) if is_corner else c.get_RadarState(lead_prob, float(-lead_msg.y[0]))
          ld['modelProb'] = 0.01
          center_list.append(ld)

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

    def cutin_lead_eligible(lead: dict[str, Any]) -> bool:
      return (
        int(lead.get('radarTrackId', -1)) in self.side_corner_front_matches
        and 0.8 < lead['dRel'] < 8.0 and lead['vLead'] > 0.0
      ) or (
        self.cutin_enter_min_x < lead['dRel'] < self.cutin_enter_max_x and lead['vLead'] > 4.0
      )

    self.leadCutIn = min(
      (lead for lead in cutin_list if cutin_lead_eligible(lead)),
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
          (ld for ld in center_list if ld['vLead'] > 2.0 and ld['radar'] and ld['dRel'] > 0.8),
          key=lambda d: d['dRel'],
          default=None
      )
    else:
      self.leadCenter = None

    lead_one_track_id = (
      int(self.radar_state.leadOne.radarTrackId)
      if self.radar_state.leadOne.status and self.radar_state.leadOne.radar
      else None
    )

    def external_candidate(lead: dict[str, Any] | None) -> bool:
      if not lead or not lead.get('status') or not lead.get('radar'):
        return False
      return lead_one_track_id is None or int(lead.get('radarTrackId', -1)) != lead_one_track_id

    # leadOne is reserved for the primary vision-matched front track. Every
    # radar-only aid competes for leadTwo without changing its measured range.
    external_groups = (
      [
        lead for lead in cutin_list
        if self.detect_cut_in and cutin_lead_eligible(lead) and external_candidate(lead)
      ],
      [self.cornerLeadStopped] if external_candidate(self.cornerLeadStopped) else [],
      [
        ld for ld in center_list
        if external_candidate(ld) and ld['dRel'] > 0.8 and ld['vLead'] > 2.0 and self._radar_only_center_ok(ld)
      ],
    )
    for candidates in external_groups:
      if candidates:
        self.leadTwo = copy.deepcopy(min(candidates, key=lambda lead: lead['dRel']))
        break

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
