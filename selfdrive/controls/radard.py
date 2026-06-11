#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any
import heapq
import copy

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


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

CUTIN_CONFIRM_FRAMES = int(0.25 / DT_MDL)
CUTIN_STICKY_FRAMES = int(0.7 / DT_MDL)
CUTIN_MIN_TRACK_AGE = int(1.0 / DT_MDL)
CUTIN_ENTER_FUTURE_IN_LANE_PROB = 0.25
CUTIN_ENTER_PROB_GAIN = 0.12
CUTIN_ENTER_CENTERING_GAIN = 0.25
CUTIN_KEEP_FUTURE_IN_LANE_PROB = 0.12
CUTIN_KEEP_MAX_DPATH_FUTURE = 1.6
CUTIN_KEEP_MAX_MOVING_AWAY = 0.3
CUTIN_PROMOTE_DREL_MARGIN = 1.0
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


def laplacian_pdf(x: float, mu: float, b: float):
  diff = abs(x - mu) / max(b, 1e-4)
  return 0.0 if diff > 50.0 else math.exp(-diff)

def clamp(x: float, lo: float, hi: float) -> float:
  return float(np.clip(x, lo, hi))

class Track:
  def __init__(self, identifier: int):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)

    self.is_stopped_car_count = 0
    self.selected_count = 0
    self.cut_in_count = 0
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
    self.sticky_dPath = 0.0
    self.sticky_path_y_std = 0.0

    # ---- noise filter state (new) ----
    self._vLead_last = 0.0
    self._vLead_filt = 0.0
    self._vLead_filt_init = False

  def update(self, md, pt, ready, radar_reaction_factor, radar_lat_factor):
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
    if not self.measured:
      self.cnt = 0
      self.selected_count = 0
      self.is_stopped_car_count = 0
      # optional: also reset filter init when track is not measured
      self._vLead_filt_init = False
    elif prev_measured and self.selected_count > 0:
      if (abs(self.dRel - prev_dRel) > 5.0 or
          abs(self.yRel - prev_yRel) > 2.0 or
          abs(self.vLead - prev_vLead) > 7.0):
        self.selected_count = 0
        self.is_stopped_car_count = 0

    self.yRel_future = self.yRel + self.yvLead * radar_lat_factor
    self.dRel_future = self.dRel + self.vLead * radar_lat_factor
    if ready:
      self.d_path(md)
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
      return dist_from_center, in_lane_prob

    self.dPath, self.in_lane_prob = d_path_interp(self.dRel, self.yRel)
    self.dPath_future, self.in_lane_prob_future = d_path_interp(self.dRel_future, self.yRel_future)

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
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

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

    self.radar_detected = False
    self.leadCenter = None
    self.leadTwo = None
    self.leadCutIn = {'status': False}

    self._corner_lat_hist = {
      "L": deque(maxlen=10),
      "R": deque(maxlen=10),
    }
    self._corner_state = {"L": 0, "R": 0}  # -1,0,+1


  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    self.enable_radar_tracks = self.params.get_int("EnableRadarTracks")
    self.enable_corner_radar = self.params.get_int("EnableCornerRadar")
    self.radar_lat_factor = self.params.get_float("RadarLatFactor") * 0.01
    self.radar_reaction_factor = self.params.get_float("RadarReactionFactor") * 0.01
    self.detect_cut_in = self.radar_lat_factor > 0
    vision_only_mode = self.enable_radar_tracks <= VISION_ONLY_RADAR_TRACK_MODE

    leads_v3 = sm['modelV2'].leadsV3
    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    if vision_only_mode:
      self.tracks.clear()
    else:
      valid_ids = set()
      for pt in rr.points:
        track_id = pt.trackId
        valid_ids.add(track_id)

        if track_id not in self.tracks:
          self.tracks[track_id] = Track(track_id)

        self.tracks[track_id].update(sm['modelV2'], pt, self.ready, self.radar_reaction_factor, self.radar_lat_factor)

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

      alive_tracks = {tid: trk for tid, trk in self.tracks.items() if trk.cnt > 2 }
      self.radar_state.leadOne, self.radar_detected = self.get_lead(sm['carState'], md, alive_tracks, 0, leads_v3[0], model_v_ego, self.lead_prob_filters[0].x, low_speed_override=False)
      self.radar_state.leadTwo, _ = self.get_lead(sm['carState'], md, alive_tracks, 1, leads_v3[1], model_v_ego, self.lead_prob_filters[1].x, low_speed_override=False)

      self.lane_line_available = md.laneLineProbs[1] > 0.5 and md.laneLineProbs[2] > 0.5
      self.compute_leads(self.v_ego, alive_tracks, md, self.lead_prob_filters[0].x)
      if self.leadTwo is not None:
        self.radar_state.leadTwo = self.leadTwo
      if self.enable_radar_tracks >= 3:
        self._pick_lead_one_from_state()

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)

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

    sticky_track = False
    if track is None and index == 0:
      track = self.get_sticky_track(tracks)
      if track is not None:
        sticky_track = True
        track.selected_count = min(track.selected_count + 1, STICKY_SELECTED_COUNT_MAX)

    if (track is None or (lead_prob < .6 and not sticky_track)) and track_scc is not None and track_scc.cnt > 2:
      #if self.enable_radar_tracks in [-1, 2] or model_v_ego < 5 or track_scc.vLead < 5.0:
      if self.enable_radar_tracks == -1 or (self.enable_radar_tracks >= 2 and track_scc.vLead < 5.0):
        track = track_scc

    lead_dict = {'status': False}
    radar = False
    if track is not None:
      vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
      lead_dict = track.get_RadarState(lead_prob, vision_y_rel)
      radar = True
    elif (track is None) and ready and (lead_prob > .5):
      lead_dict = get_RadarState_from_vision(md, lead_msg, v_ego, model_v_ego, lead_prob)

    if self.enable_corner_radar > 1:
      lead_dict = self.corner_radar(CS, lead_dict)

    if low_speed_override:
      low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
      if len(low_speed_tracks) > 0:
        closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

        # Only choose new track if it is actually closer than the previous one
        if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
          vision_y_rel = float(-lead_msg.y[0]) if ready else 0.0
          lead_dict = closest_track.get_RadarState(lead_prob, vision_y_rel)

    return lead_dict, radar

  def _is_cutin_enter_candidate(self, t: Track) -> bool:
    if not self.detect_cut_in or not self.lane_line_available:
      return False
    if t.cnt < CUTIN_MIN_TRACK_AGE:
      return False
    if not (3.0 < t.dRel < 50.0 and t.vLead > 4.0):
      return False
    if t.in_lane_prob_future < CUTIN_ENTER_FUTURE_IN_LANE_PROB:
      return False
    if (t.in_lane_prob_future - t.in_lane_prob) < CUTIN_ENTER_PROB_GAIN:
      return False
    if (abs(t.dPath) - abs(t.dPath_future)) < CUTIN_ENTER_CENTERING_GAIN:
      return False
    return True

  def _is_cutin_keep_candidate(self, t: Track) -> bool:
    if not self.detect_cut_in or not self.lane_line_available:
      return False
    if not (2.5 < t.dRel < 55.0 and t.vLead > 2.0):
      return False

    moving_away = abs(t.dPath_future) - abs(t.dPath)
    if moving_away > CUTIN_KEEP_MAX_MOVING_AWAY:
      return False

    return (
      t.in_lane_prob_future > CUTIN_KEEP_FUTURE_IN_LANE_PROB or
      abs(t.dPath_future) < CUTIN_KEEP_MAX_DPATH_FUTURE
    )

  def _update_cutin_sticky(self, t: Track) -> bool:
    if self._is_cutin_enter_candidate(t):
      t.cut_in_count = min(t.cut_in_count + 1, CUTIN_STICKY_FRAMES)
    elif t.cut_in_count > 0 and self._is_cutin_keep_candidate(t):
      t.cut_in_count = max(t.cut_in_count - 1, 0)
    else:
      t.cut_in_count = 0

    return t.cut_in_count >= CUTIN_CONFIRM_FRAMES

  def _cutin_can_replace_lead_one(self, cutin: dict[str, Any]) -> bool:
    lead_one = self.radar_state.leadOne
    if not lead_one.status:
      return True

    return cutin["dRel"] + CUTIN_PROMOTE_DREL_MARGIN < lead_one.dRel

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

    if d_rel > RADAR_ONLY_CENTER_FAR_DREL:
      return d_path < RADAR_ONLY_CENTER_DPATH_FAR_LIMIT
    if d_rel > RADAR_ONLY_CENTER_MID_DREL:
      return d_path < RADAR_ONLY_CENTER_DPATH_MID_LIMIT
    return d_path < RADAR_ONLY_CENTER_DPATH_NEAR_LIMIT

  def compute_leads(self, v_ego, tracks, md, lead_prob):
    self.leadCenter = None
    self.leadTwo = None
    self.leadCutIn = {'status': False}

    lead_msg = md.leadsV3[0] if (md is not None and len(md.position.x) == 33) else None
    if lead_msg is None:
      # reset
      self.radar_state.leadsLeft = []
      self.radar_state.leadsCenter = []
      self.radar_state.leadsRight = []
      self.radar_state.leadsCutIn = []
      self.radar_state.leadsLeft2 = []
      self.radar_state.leadsRight2 = []
      self.radar_state.leadLeft = {'status': False}
      self.radar_state.leadRight = {'status': False}
      return

    left_list, right_list, center_list, cutin_list = [], [], [], []
    for c in tracks.values():
      y_rel_neg = - c.yRel
      # center
      if self._is_center_lead_candidate(c):
        c.cut_in_count = max(c.cut_in_count - 1, 0)
        if c.cnt > 3:
          ld = c.get_RadarState(lead_prob, float(-lead_msg.y[0]))
          ld['modelProb'] = 0.01
          center_list.append(ld)

      # left/right
      elif y_rel_neg < 0: #left_lane_y:
        ld = c.get_RadarState(0, 0)
        if self._update_cutin_sticky(c):
          ld['modelProb'] = 0.03
          cutin_list.append(ld)
        left_list.append(ld)
      else:
        ld = c.get_RadarState(0, 0)
        if self._update_cutin_sticky(c):
          ld['modelProb'] = 0.03
          cutin_list.append(ld)
        right_list.append(ld)

    self.radar_state.leadsLeft   = left_list
    self.radar_state.leadsRight  = right_list
    self.radar_state.leadsCenter = center_list
    self.radar_state.leadsCutIn = cutin_list
    self.leadCutIn = min(
      (ld for ld in cutin_list if 3 < ld['dRel'] < 50 and ld['vLead'] > 4),
      key=lambda d: d['dRel'],
      default={'status': False}
    )

    self.radar_state.leadLeft  = min(
        (ld for ld in left_list if ld['dRel'] > 5 and abs(ld['dPath']) < 3.5),
        key=lambda d: d['dRel'],
        default={'status': False}
    )
    self.radar_state.leadRight = min(
        (ld for ld in right_list if ld['dRel'] > 5 and abs(ld['dPath']) < 3.5),
        key=lambda d: d['dRel'],
        default={'status': False}
    )

    self.leadTwo = None
    if self.lane_line_available:
      self.leadCenter = min(
          (ld for ld in center_list if ld['vLead'] > 5 and ld['radar'] and ld['dRel'] > 3.5),
          key=lambda d: d['dRel'],
          default=None
      )
      if self.radar_state.leadOne.status and self.radar_state.leadOne.radar:
        self.leadTwo = min(
            (ld for ld in center_list if ld['vLead'] > 5 and ld['radar'] and self.radar_state.leadOne.dRel < ld['dRel'] < 80),
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

    if self.leadCutIn and self.leadCutIn.get("status") and self.detect_cut_in:
      if self._cutin_can_replace_lead_one(self.leadCutIn):
        chosen = self.leadCutIn
        chosen["modelProb"] = 0.03
        detected = True

    elif self.leadCenter and self.leadCenter["status"]:
      lead_one = self.radar_state.leadOne
      vision_prob = lead_one.modelProb if lead_one.status else 0.0

      if self.radar_detected:
        if lead_one.status and self.leadCenter["dRel"] + CUTIN_PROMOTE_DREL_MARGIN < lead_one.dRel:
          chosen = self.leadCenter
          chosen["modelProb"] = 0.01
      else:
        radar_clearly_closer = lead_one.status and self.leadCenter["dRel"] + CUTIN_PROMOTE_DREL_MARGIN < lead_one.dRel
        vision_weak_or_missing = (not lead_one.status) or vision_prob < RADAR_ONLY_FALLBACK_VISION_PROB

        if vision_weak_or_missing and (not lead_one.status or radar_clearly_closer) and self._radar_only_center_ok(self.leadCenter):
          chosen = self.leadCenter
          chosen["modelProb"] = 0.02
          detected = True

    if chosen is not None:
        self.radar_state.leadOne = chosen
        self.radar_detected = detected

  def _corner_update_state(self, side: str, cur_lat: float, enter_lat: float = 2.8) -> int:
    # 유효 범위 밖이면 리셋
    if not (0.0 < cur_lat < enter_lat):
      self._corner_lat_hist[side].clear()
      self._corner_state[side] = 0
      return 0

    h = self._corner_lat_hist[side]
    h.append(cur_lat)

    n = len(h)
    if n < 3:
      # 데이터 너무 적으면 이전 상태 유지
      return self._corner_state[side]

    delta = h[-1] - h[0]
    th = 0.02 # 3 * (20 / n)

    if delta < -th:
      self._corner_state[side] = +1   # approaching
    elif delta > th:
      self._corner_state[side] = -1   # leaving
    else:
      self._corner_state[side] = 0    # maintain

    return self._corner_state[side]

  def corner_radar(self, CS, lead_dict):
    ENTER_LAT = 2.2
    KEEP_LAT  = 2.0
    EXIT_LAT  = 1.2

    left_lat, right_lat = abs(CS.leftLatDist), abs(CS.rightLatDist)
    left_state  = self._corner_update_state("L", left_lat)
    right_state = self._corner_update_state("R", right_lat)

    # 1) left usable?
    left_ok = False
    if left_state > 0:
      left_ok = left_lat < ENTER_LAT
    elif left_state == 0:
      left_ok = 0 < left_lat < KEEP_LAT
    else:  # leaving
      left_ok = left_lat <= EXIT_LAT

    # 2) right usable?
    right_ok = False
    if right_state > 0:
      right_ok = right_lat < ENTER_LAT
    elif right_state == 0:
      right_ok = 0 < right_lat < KEEP_LAT
    else:
      right_ok = right_lat <= EXIT_LAT

    # 3) 아무도 못 쓰면 skip
    if not left_ok and not right_ok:
      return lead_dict

    # 4) 둘 다 되면 longDist로 선택
    if left_ok and right_ok:
      if CS.leftLongDist <= CS.rightLongDist:
        lat_dist, long_dist = +left_lat, CS.leftLongDist
      else:
        lat_dist, long_dist = -right_lat, CS.rightLongDist
    elif left_ok:
      lat_dist, long_dist = +left_lat, CS.leftLongDist
    else:
      lat_dist, long_dist = -right_lat, CS.rightLongDist

    if lead_dict['status']:
      if lead_dict['dRel'] > long_dist:
        lead_dict['dRel'] = long_dist
        lead_dict['yRel'] = lat_dist
        lead_dict['vRel'] = 0.0
        lead_dict['vLead'] = CS.vEgo if CS.vEgo < lead_dict['vLead'] else lead_dict['vLead']
        lead_dict['vLeadK'] = lead_dict['vLead']
        lead_dict['aLead'] = CS.aEgo if CS.aEgo < lead_dict['aLead'] else lead_dict['aLead']
        lead_dict['aLeadK'] = lead_dict['aLead']
        lead_dict['aLeadTau'] = _LEAD_ACCEL_TAU
        lead_dict['jLead'] = 0.0
        lead_dict['vLat'] = 0.0
        lead_dict['modelProb'] = 1.0
        lead_dict['radarTrackId'] = -1
        lead_dict['radar'] = True
    else:
      lead_dict['status'] = True
      lead_dict['dRel'] = long_dist
      lead_dict['yRel'] = lat_dist
      lead_dict['vRel'] = 0.0
      lead_dict['vLead'] = CS.vEgo
      lead_dict['vLeadK'] = CS.vEgo
      lead_dict['aLead'] = CS.aEgo
      lead_dict['aLeadK'] = CS.aEgo
      lead_dict['aLeadTau'] = _LEAD_ACCEL_TAU
      lead_dict['jLead'] = 0.0
      lead_dict['vLat'] = 0.0
      lead_dict['modelProb'] = 1.0
      lead_dict['radarTrackId'] = -1
      lead_dict['radar'] = True

    return lead_dict

# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  #sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'])
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    if sm.updated['modelV2']:
      RD.update(sm, sm['liveTracks'])
      RD.publish(pm)


if __name__ == "__main__":
  main()
