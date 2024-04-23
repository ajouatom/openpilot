#!/usr/bin/env python3
import importlib
import math
from collections import deque
from typing import Any, Optional

import capnp
from cereal import messaging, log, car
from openpilot.common.numpy_fast import interp, clip
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, Ratekeeper, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog

from openpilot.common.simple_kalman import KF1D

from openpilot.common.params import Params
from selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
import numpy as np

from openpilot.common.filter_simple import StreamingMovingAverage

# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [dt * 0.01 for dt in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[interp(dt, dts, K0)], [interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, y_rel: float, kalman_params: KalmanParams, radar_ts: float):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = _LEAD_ACCEL_TAU
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
    self.kf_y = KF1D([[y_rel], [0.0]], self.K_A, self.K_C, self.K_K)
    self.dRel = 0.0
    self.vRel = 0.0
    self.vLat = 0.0
    self.vision_prob = 0.0
    self.radar_ts = radar_ts

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float, measured: float, a_rel: float, aLeadTauPos: float, aLeadTauNeg: float, aLeadTauThreshold: float, a_ego: float):

    #apilot: changed radar target
    if abs(self.dRel - d_rel) > 3.0 or abs(self.vRel - v_rel) > 20.0 * self.radar_ts: # 거리3M이상, 20m/s^2이상 상대속도 차이날때 초기화
      self.cnt = 0
      self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)
      self.kf_y = KF1D([[y_rel], [0.0]], self.K_A, self.K_C, self.K_K) 
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.aRel = a_rel   # REL_ACCEL: radar track만 나옴.
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)
      self.kf_y.update(self.yRel)

    self.vLat = float(self.kf_y.x[1][0])

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])
    #self.aLeadK = a_rel + a_ego if a_rel != 0 else float(self.kf.x[ACCEL][0]) ## radar track의 A_REL을 사용하도록 함. 값이 약간 더 큼.

    # Learn if constant acceleration
    #if abs(self.aLeadK) < 0.5:
    #  self.aLeadTau = _LEAD_ACCEL_TAU

    # 감속일때는 0.2로 시작..  (시험중... 삭제)
    #aLeadTau_apply = 0.2 if self.aLeadK < 0.0 else aLeadTau
    #if -0.2 < self.aLeadK < 0.5:
    #  self.aLeadTau = aLeadTauInit
    #else:
    #  self.aLeadTau = min(self.aLeadTau * 0.9, aLeadTau_apply)
    if abs(self.aLeadK) < aLeadTauThreshold:
      self.aLeadTau = aLeadTauPos if self.aLeadK >= 0 else aLeadTauNeg
    else:
      self.aLeadTau *= 0.9

    self.cnt += 1

  def get_key_for_cluster(self):
    # Weigh y higher since radar is inaccurate in this dimension
    return [self.dRel, self.yRel*2, self.vRel]

  def reset_a_lead(self, aLeadK: float, aLeadTauInit: float):
    self.kf = KF1D([[self.vLead], [aLeadK]], self.K_A, self.K_C, self.K_K)
    self.aLeadK = aLeadK
    self.aLeadTau = aLeadTauInit

  def get_RadarState(self, model_prob: float = 0.0, vision_y_rel = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel) if self.yRel != 0 else vision_y_rel,
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
      "aRel": float(self.aRel),
      "vLat": float(self.vLat),
    }

  def get_lane_position(self) -> str:
    if self.yRel > 0 and self.vRel > 5.0:
      return 'left'
    elif self.yRel < 0 and self.vRel > 5.0:
      return 'right'
    else:
      return 'front'

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track]):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA
  tracks_len = len(tracks)

  def prob(c, c_key):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel + c.vLat * 2.0, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    #속도가 빠른것에 weight를 더줌. apilot, 
    #231120: 감속정지중, 전방차량정지상태인데, 주변의 노이즈성 레이더포인트가 검출되어 버림. 
    #       이 포인트는 약간 먼데, 주행하고 있는것처럼.. 인식됨. 아주 잠깐 인식됨.
    #        속도관련 weight를 없애야하나?  TG를 지나고 있는 차를 우선시하도록 만든건데...
    weight_v = interp(c.vRel + v_ego, [0, 10], [0.3, 1])
    # This is isn't exactly right, but good heuristic
    prob = prob_d * prob_y * prob_v * weight_v
    c.vision_prob = prob
    
    return prob # if c_key != 0 else 0

  track_key, track = max(tracks.items(), key=lambda item: prob(item[1], item[0]))
  #track = max(tracks.values(), key=prob)

  #prob_values = {track_id: prob(track) for track_id, track in tracks.items()}
  #max_prob_track_id = max(prob_values, key=prob_values.get)
  #max_prob_value = prob_values[max_prob_track_id]
  #track = tracks.get(max_prob_track_id)
  # yRel값은 왼쪽이 +값, lead.y[0]값은 왼쪽이 -값

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.35, 5.0])
  vel_tolerance = 20.0 if lead.prob > 0.98 else 15.0 if lead.prob > 0.95 else 10 # high vision track prob, increase tolerance (for stopped car)
  #vel_tolerance = interp(lead.prob, [0.80, 0.85, 0.98, 1.0], [10.0, 20.0, 25.0, 30.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < vel_tolerance) or (v_ego + track.vRel > 3)
  ##간혹 어수선한경우, 전방에 차가 없지만, 좌우에 차가많은경우 억지로 레이더를 가져오는 경우가 있음..(레이더트랙의 경우)
  y_sane = (abs(-lead.y[0]-track.yRel) < 3.2 / 2.)  #lane_width assumed 3.2M, laplacian_pdf 의 prob값을 검증하려했지만, y값으로 처리해도 될듯함.
  if dist_sane and vel_sane and y_sane:
    return track
  else:
    return None

def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  
  aLeadK = float(lead_msg.a[0])
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": aLeadK, 
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead_org(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, low_speed_override: bool = True) -> dict[str, Any]:
  ## SCC레이더는 일단 보관하고 리스트에서 삭제...
  track_scc = tracks.get(0)
  #if track_scc is not None:  
  #  del tracks[0]            ## tracks에서 삭제하면안됨... ㅠㅠ

  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_msg.prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks)
  else:
    track = None

  ## carrot : 삭제함.. 오히려 SCC레이더의 값이 엉뚱한곳을 가르키는 차량이 있음.. kona_ev.. 엉뚱한 차량을 보고 쓸데없는 감속을 함.
  ## vision match후 발견된 track이 없으면
  ##  track_scc 가 있는 지 확인하고
  ##    비전과의 차이가 35%(5M)이상 차이나면 scc가 발견못한것이기 때문에 비전것으로 처리함.
  #if track_scc is not None and track is None:
  #  track = track_scc
  #  if lead_msg.prob > .5:
  #    offset_vision_dist = lead_msg.x[0] - RADAR_TO_CAMERA
  #    if offset_vision_dist < track.dRel - 5.0: #끼어드는 차량이 있는 경우 처리..
  #      track = None

  lead_dict = {'status': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob, float(-lead_msg.y[0]))
  elif (track is None) and ready and (lead_msg.prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState(lead_msg.prob, float(-lead_msg.y[0]))

  return lead_dict

def get_lead_side(v_ego, tracks, md, lane_width, model_v_ego):
  lead_msg = md.leadsV3[0]
  leadCenter = {'status': False}
  leadLeft = {'status': False}
  leadRight = {'status': False}

  ## SCC레이더는 일단 보관하고 리스트에서 삭제...
  track_scc = tracks.get(0)
  #if track_scc is not None:
  #  del tracks[0]

  #if len(tracks) == 0:
  #  return [[],[],[],leadLeft,leadRight]
  if md is not None and len(md.position.x) == TRAJECTORY_SIZE:
    md_y = md.position.y
    md_x = md.position.x
  else:
    return [[],[],[],leadCenter,leadLeft,leadRight]

  leads_center = {}
  leads_left = {}
  leads_right = {}
  next_lane_y = lane_width / 2 + lane_width * 0.8
  for c in tracks.values():
    # d_y :  path_y - traks_y 의 diff값
    # yRel값은 왼쪽이 +값, lead.y[0]값은 왼쪽이 -값
    d_y = -c.yRel - interp(c.dRel, md_x, md_y)
    if abs(d_y) < lane_width/2:
      ld = c.get_RadarState(lead_msg.prob, float(-lead_msg.y[0]))
      leads_center[c.dRel] = ld
    elif -next_lane_y < d_y < 0:
      ld = c.get_RadarState(0.0)
      leads_left[c.dRel] = ld
    elif 0 < d_y < next_lane_y:
      ld = c.get_RadarState(0.0)
      leads_right[c.dRel] = ld

  if lead_msg.prob > 0.5:
    ld = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)
    leads_center[ld['dRel']] = ld
  #ll,lr = [[l[k] for k in sorted(list(l.keys()))] for l in [leads_left,leads_right]]
  #lc = sorted(leads_center.values(), key=lambda c:c["dRel"])
  ll = list(leads_left.values())
  lr = list(leads_right.values())

  if leads_center:
    dRel_min = min(leads_center.keys())
    lc = [leads_center[dRel_min]]
  else:
    lc = {}

  leadLeft = min((lead for dRel, lead in leads_left.items() if lead['dRel'] > 5.0), key=lambda x: x['dRel'], default=leadLeft)
  leadRight = min((lead for dRel, lead in leads_right.items() if lead['dRel'] > 5.0), key=lambda x: x['dRel'], default=leadRight)
  leadCenter = min((lead for dRel, lead in leads_center.items() if lead['vLead'] > 0.5 and lead['radar']), key=lambda x: x['dRel'], default=leadCenter)

  #filtered_leads_left = {dRel: lead for dRel, lead in leads_left.items() if lead['dRel'] > 5.0}
  #if filtered_leads_left:
  #  dRel_min = min(filtered_leads_left.keys())
  #  leadLeft = filtered_leads_left[dRel_min]

  #filtered_leads_right = {dRel: lead for dRel, lead in leads_right.items() if lead['dRel'] > 5.0}
  #if filtered_leads_right:
  #  dRel_min = min(filtered_leads_right.keys())
  #  leadRight = filtered_leads_right[dRel_min]

  return [ll, lc, lr, leadCenter, leadLeft, leadRight]

LEAD_KALMAN_SPEED, LEAD_KALMAN_ACCEL = 0, 1
def lead_kf(v_lead: float, a_lead: float, dt: float = 0.05):
  # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
  # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
  assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
  A = [[1.0, dt], [0.0, 1.0]]
  C = [1.0, 0.0]
  #Q = np.matrix([[10., 0.0], [0.0, 100.]])
  #R = 1e3
  #K = np.matrix([[ 0.05705578], [ 0.03073241]])
  dts = [dt * 0.01 for dt in range(1, 21)]
  K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
        0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
        0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
        0.35353899, 0.36200124]
  K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
        0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
        0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
        0.26393339, 0.26278425]
  K = [[interp(dt, dts, K0)], [interp(dt, dts, K1)]]

  kf = KF1D([[v_lead], [a_lead]], A, C, K)
  return kf


class VisionTrack:
  def __init__(self, radar_ts):
    self.radar_ts = radar_ts
    self.dRel = 0.0
    self.vRel = 0.0
    self.yRel = 0.0
    self.vLead = 0.0
    self.aLead = 0.0
    self.vLeadK = 0.0
    self.aLeadK = 0.0
    self.aLeadTau = _LEAD_ACCEL_TAU
    self.prob = 0.0
    self.status = False
    self.aLeadTauPos = float(Params().get_int("ALeadTauPos")) / 100. 
    self.aLeadTauNeg = float(Params().get_int("ALeadTauNeg")) / 100. 
    self.aLeadTauThreshold = float(Params().get_int("ALeadTauThreshold")) / 100.

    self.kf: KF1D | None = None
    self.kf_v: KF1D | None = None

  def get_lead(self):
    return {
      "dRel": self.dRel,
      "yRel": self.yRel,
      "vRel": self.vRel,
      "vLead": self.vLead,
      "vLeadK": self.vLeadK,    ## TODO: 아직 vLeadK는 엉망인듯...
      "aLeadK": 0.0 if self.mixRadarInfo in [3] else clip(self.aLeadK, self.aLead - 1.0, self.aLead + 1.0),
      "aLeadTau": 0.3 if self.mixRadarInfo in [3] else self.aLeadTau,
      "fcw": False,
      "modelProb": self.prob,
      "status": self.status,
      "radar": False,
      "radarTrackId": -1,
    }

  def reset(self):
    self.status = False
    self.kf = None
    self.kf_v = None
    self.aLeadTau = _LEAD_ACCEL_TAU

  def update(self, lead_msg, model_v_ego, v_ego):
    self.aLeadTauPos = float(Params().get_int("ALeadTauPos")) / 100. 
    self.aLeadTauNeg = float(Params().get_int("ALeadTauNeg")) / 100. 
    self.aLeadTauThreshold = float(Params().get_int("ALeadTauThreshold")) / 100.
    self.mixRadarInfo = int(Params().get_int("MixRadarInfo"))

    lead_v_rel_pred = lead_msg.v[0] - model_v_ego
    self.prob = lead_msg.prob
    self.v_ego = v_ego
    if self.prob > .5:
      self.dRel = float(lead_msg.x[0]) - RADAR_TO_CAMERA
      self.yRel = float(-lead_msg.y[0])
      self.vRel = lead_v_rel_pred
      self.vLead = float(v_ego + lead_v_rel_pred)
      self.aLead = lead_msg.a[0]
      if self.prob < 0.99:
        self.kf = None
        self.kf_v = None
      self.status = True
    else:
      self.reset()

    if self.kf is None:
      self.kf = lead_kf(self.vLead, self.aLead, self.radar_ts)
      self.kf_v = lead_kf(self.dRel, self.vRel, self.radar_ts)
    else:
      self.kf.update(self.vLead)
      self.kf_v.update(self.dRel)

    #self.vLeadK = float(self.kf.x[LEAD_KALMAN_SPEED][0])
    self.vLeadK = float(self.kf_v.x[1][0]) + model_v_ego
    self.aLeadK = float(self.kf.x[LEAD_KALMAN_ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLead) < self.aLeadTauThreshold:
      self.aLeadTau = self.aLeadTauPos if self.aLeadK >= 0 else self.aLeadTauNeg
    else:
      self.aLeadTau = min(self.aLeadTau * 0.9, _LEAD_ACCEL_TAU)

class RadarD:
  def __init__(self, radar_ts: float, delay: int = 0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.tracks_empty: dict[int, Track] = {}
    self.kalman_params = KalmanParams(radar_ts)

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=delay+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False
    self.showRadarInfo = True
    self.mixRadarInfo = 0
    self.aLeadTauPos = 1.5
    self.aLeadTauNeg = 1.5
    self.aLeadTauThreshold = 0.5
    self.vision_tracks = [VisionTrack(radar_ts), VisionTrack(radar_ts)]
    self.a_ego = 0.0

    self.radar_ts = radar_ts

  def update(self, sm: messaging.SubMaster, rr: Optional[car.RadarData]):
    #self.showRadarInfo = int(Params().get("ShowRadarInfo"))
    self.mixRadarInfo = int(Params().get_int("MixRadarInfo"))
    self.aLeadTauPos = float(Params().get_int("ALeadTauPos")) / 100. 
    self.aLeadTauNeg = float(Params().get_int("ALeadTauNeg")) / 100. 
    self.aLeadTauThreshold = float(Params().get_int("ALeadTauThreshold")) / 100.

    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    leads_v3 = sm['modelV2'].leadsV3

    radar_points = []
    radar_errors = []
    if rr is not None:
      radar_points = rr.points
      radar_errors = rr.errors

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.a_ego = sm['carState'].aEgo
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {}
    for pt in radar_points:
      if pt.trackId == 0 and pt.yRel == 0: # SCC radar
        if self.ready and leads_v3[0].prob > 0.5:
          pt.yRel = -leads_v3[0].y[0]
      ar_pts[pt.trackId] = [pt.dRel, pt.yRel, pt.vRel, pt.measured, pt.aRel]

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, rpt[1], self.kalman_params, self.radar_ts)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], rpt[4], self.aLeadTauPos, self.aLeadTauNeg, self.aLeadTauThreshold, self.a_ego)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks() and len(radar_errors) == 0
    self.radar_state = log.RadarState.new_message()
    model_updated = False if self.radar_state.mdMonoTime == sm.logMonoTime['modelV2'] else True
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = list(radar_errors)
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    if len(sm['modelV2'].temporalPose.trans):
      model_v_ego = sm['modelV2'].temporalPose.trans[0]
    else:
      model_v_ego = self.v_ego
    #leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      if model_updated:
        self.vision_tracks[0].update(leads_v3[0], model_v_ego, self.v_ego)
        self.vision_tracks[1].update(leads_v3[1], model_v_ego, self.v_ego)

      ll, lc, lr, leadCenter, self.radar_state.leadLeft, self.radar_state.leadRight = get_lead_side(self.v_ego, self.tracks, sm['modelV2'], sm['lateralPlan'].laneWidth, model_v_ego)
      self.radar_state.leadsLeft = list(ll)
      self.radar_state.leadsCenter = list(lc)
      self.radar_state.leadsRight = list(lr)

      if self.mixRadarInfo in [1]: ## leadOne: radar or vision, leadTwo: vision 
        self.radar_state.leadOne = self.get_lead(self.tracks, 0, leads_v3[0], model_v_ego, low_speed_override=False)
        self.radar_state.leadTwo = self.get_lead(self.tracks_empty, 0, leads_v3[0], model_v_ego, low_speed_override=False)
      elif self.mixRadarInfo in [2,3]: ## vision only mode
        self.radar_state.leadOne = self.get_lead(self.tracks_empty, 0, leads_v3[0], model_v_ego, low_speed_override=False)
        self.radar_state.leadTwo = self.get_lead(self.tracks_empty, 1, leads_v3[1], model_v_ego, low_speed_override=False)
      elif self.mixRadarInfo in [4]: ## additional radar detector
        self.radar_state.leadOne = self.get_lead(self.tracks_empty, 0, leads_v3[0], model_v_ego, low_speed_override=False)
        self.radar_state.leadTwo = leadCenter
      else: ## comma stock.
        self.radar_state.leadOne = self.get_lead(self.tracks, 0, leads_v3[0], model_v_ego, low_speed_override=False)
        self.radar_state.leadTwo = self.get_lead(self.tracks, 1, leads_v3[1], model_v_ego, low_speed_override=False)

  def publish(self, pm: messaging.PubMaster, lag_ms: float):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    radar_msg.radarState.cumLagMs = lag_ms
    pm.send("radarState", radar_msg)

    # publish tracks for UI debugging (keep last)
    tracks_msg = messaging.new_message('liveTracks', len(self.tracks))
    tracks_msg.valid = self.radar_state_valid
    for index, tid in enumerate(sorted(self.tracks.keys())):
      tracks_msg.liveTracks[index] = {
        "trackId": tid,
        "dRel": float(self.tracks[tid].dRel),
        "yRel": float(self.tracks[tid].yRel),
        "vRel": float(self.tracks[tid].vRel),
        "aRel": float(self.tracks[tid].aRel),
        "vLat": float(self.tracks[tid].vLat),
      }
    pm.send('liveTracks', tracks_msg)

  def get_lead(self, tracks: dict[int, Track], index: int, lead_msg: capnp._DynamicStructReader,
               model_v_ego: float, low_speed_override: bool = True) -> dict[str, Any]:

    v_ego = self.v_ego
    ready = self.ready
    ## SCC레이더는 일단 보관하고 리스트에서 삭제...
    track_scc = tracks.get(0)
    #if track_scc is not None:  
    #  del tracks[0]            ## tracks에서 삭제하면안됨... ㅠㅠ

    # Determine leads, this is where the essential logic happens
    if len(tracks) > 0 and ready and lead_msg.prob > .5:
      track = match_vision_to_track(v_ego, lead_msg, tracks)
    else:
      track = None

    # vision match후 발견된 track이 없으면
    #  track_scc 가 있는 지 확인하고
    #    비전과의 차이가 35%(5M)이상 차이나면 scc가 발견못한것이기 때문에 비전것으로 처리함.
    if track_scc is not None and track is None: 
      track = track_scc
      if self.vision_tracks[index].prob > .5:
        if self.vision_tracks[index].dRel < track.dRel - 5.0: #끼어드는 차량이 있는 경우 처리..
          track = None

    lead_dict = {'status': False}
    if track is not None:
      lead_dict = track.get_RadarState(lead_msg.prob, float(-lead_msg.y[0]))
    elif (track is None) and ready and (lead_msg.prob > .5):
      lead_dict = self.vision_tracks[index].get_lead()

    if low_speed_override:
      low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
      if len(low_speed_tracks) > 0:
        closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

        # Only choose new track if it is actually closer than the previous one
        if (not lead_dict['status']) or (closest_track.dRel < lead_dict['dRel']):
          lead_dict = closest_track.get_RadarState(lead_msg.prob, float(-lead_msg.y[0]))

    return lead_dict

# fuses camera and radar data for best lead detection
def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  with car.CarParams.from_bytes(Params().get("CarParams", block=True)) as msg:
    CP = msg
  cloudlog.info("radard got CarParams")

  # import the radar from the fingerprint
  cloudlog.info("radard is importing %s", CP.carName)
  RadarInterface = importlib.import_module(f'selfdrive.car.{CP.carName}.radar_interface').RadarInterface

  # *** setup messaging
  can_sock = messaging.sub_sock('can')
  sm = messaging.SubMaster(['modelV2', 'carState', 'lateralPlan'], frequency=int(1./DT_CTRL))
  pm = messaging.PubMaster(['radarState', 'liveTracks'])

  RI = RadarInterface(CP)

  rk = Ratekeeper(1.0 / CP.radarTimeStep, print_delay_threshold=None)
  RD = RadarD(CP.radarTimeStep, RI.delay)

  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    rr = RI.update(can_strings)
    sm.update(0)
    if rr is None:
      continue

    RD.update(sm, rr)
    RD.publish(pm, -rk.remaining*1000.0)

    rk.monitor_time()


if __name__ == "__main__":
  main()
