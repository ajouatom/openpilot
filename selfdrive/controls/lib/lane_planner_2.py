import math
import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp, clip
from common.realtime import DT_MDL
from common.swaglog import cloudlog
#from common.logger import sLogger
from common.params import Params

TRAJECTORY_SIZE = 33
# positive numbers go right
CAMERA_OFFSET = 0 #0.08
MIN_LANE_DISTANCE = 2.6
MAX_LANE_DISTANCE = 3.7
MAX_LANE_CENTERING_AWAY = 1.85
KEEP_MIN_DISTANCE_FROM_LANE = 1.35
KEEP_MIN_DISTANCE_FROM_EDGELANE = 1.15

def clamp(num, min_value, max_value):
  # weird broken case, do something reasonable
  if min_value > num > max_value:
    return (min_value + max_value) * 0.5
  # ok, basic min/max below
  if num < min_value:
    return min_value
  if num > max_value:
    return max_value
  return num

def sigmoid(x, scale=1, offset=0):
  return (1 / (1 + math.exp(x*scale))) + offset

def lerp(start, end, t):
  t = clamp(t, 0.0, 1.0)
  return (start * (1.0 - t)) + (end * t)

def max_abs(a, b):
  return a if abs(a) > abs(b) else b

class LanePlanner:
  def __init__(self):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.2, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = 3.2
    self.lane_change_multiplier = 1
    self.Options = Params()
    self.updateOptions = 100

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.debugText = ""
    self.lane_width_left = 0.0
    self.lane_width_right = 0.0
    self.lane_width_left_filtered = FirstOrderFilter(1.0, 0.99, DT_MDL)
    self.lane_width_right_filtered = FirstOrderFilter(1.0, 0.99, DT_MDL)
    self.lane_offset_filtered = FirstOrderFilter(0.0, 0.99, DT_MDL)

    self.lanefull_mode = False

  def parse_model(self, md):

    self.updateOptions -= 1
    if self.updateOptions <= 0:
      self.updateOptions = 100

    lane_lines = md.laneLines
    edges = md.roadEdges

    if len(lane_lines) >= 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y)
      self.rll_y = np.array(lane_lines[2].y)
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    if len(edges[0].t) == TRAJECTORY_SIZE:
      self.le_y = np.array(edges[0].y) + md.roadEdgeStds[0] * 0.4
      self.re_y = np.array(edges[1].y) - md.roadEdgeStds[1] * 0.4
    else:
      self.le_y = self.lll_y
      self.re_y = self.rll_y

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, CS, v_ego, path_t, path_xyz, curve_speed):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    #speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
    speed_lane_width = interp(v_ego, [0., 60.], [2.8, 3.5])
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    # 가장 차선이 진한쪽으로 골라서..
    #self.d_prob = l_prob + r_prob - l_prob * r_prob
    self.d_prob = max(l_prob, r_prob)

    ## self.d_prob: lane_prob
    # lanefull: offset + mix (laneline path, laneless)
    # laneless: offset + laneless

    # 좌/우의 차선폭을 필터링.
    if self.lane_width_left > 0:
      self.lane_width_left_filtered.update(self.lane_width_left)
      self.lane_width_left_filtered.x = self.lane_width_left
    if self.lane_width_right > 0:
      self.lane_width_right_filtered.update(self.lane_width_right)
      self.lane_width_right_filtered.x = self.lane_width_right

    self.adjustLaneOffset = float(Params().get_int("AdjustLaneOffset")) * 0.01
    self.adjustCurveOffset = float(Params().get_int("AdjustCurveOffset")) * 0.01
    ADJUST_OFFSET_LIMIT = 0.4 #max(self.adjustLaneOffset, self.adjustCurveOffset)
    offset_curve = 0.0
    ## curve offset
    offset_curve = -interp(abs(curve_speed), [50, 200], [self.adjustCurveOffset, 0.0]) * np.sign(curve_speed)

    # roadedge offset
    # 여유가 없는곳(도로경계쪽)으로 붙임. 
    offset_lane = 0.0
    #if self.lane_width_left_filtered.x < 1.8 and self.lane_width_right_filtered.x < 1.8:
    #  offset_lane = 0.0
    if self.lane_width_left_filtered.x > 2.5 and self.lane_width_right_filtered.x > 2.5: #양쪽에 차선이 있는경우
      offset_lane = 0.0
    elif self.lane_width_left_filtered.x > 2.5:
      offset_lane = interp(self.lane_width, [2.5, 2.9], [0.0, self.adjustLaneOffset]) # 차선이 좁으면 안함..
    elif self.lane_width_right_filtered.x > 2.5:
      offset_lane = interp(self.lane_width, [2.5, 2.9], [0.0, -self.adjustLaneOffset]) # 차선이 좁으면 안함..

    #select lane path
    if self.lane_width <= 2.2:  ## 240122 가급적 도로경계쪽차선을 따라가도록 함. 원상복구: 도로경계는 지저분함... ㅋㅋ
      # 차선이 좁아지면, 도로경계쪽에 있는 차선 위주로 따라가도록함. 다시생각해봐야...
      if l_prob > 0.5 and self.lane_width_left_filtered.x < 2.0:
        lane_path_y = path_from_left_lane
      elif r_prob > 0.5 and self.lane_width_right_filtered.x < 2.0:
        lane_path_y = path_from_right_lane
      else:
        lane_path_y = path_from_left_lane if l_prob > 0.5 or l_prob > r_prob else path_from_right_lane
    else:
      lane_path_y = path_from_left_lane if l_prob > 0.5 or l_prob > r_prob else path_from_right_lane
    #lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)

    # offset_center = lane_line_center - laneless_center

    ## 0.5초 앞의 중심을 보도록함.
    #lane_path_y_center = interp(0.5, path_t, lane_path_y)
    #path_xyz_y_center = interp(0.5, path_t, path_xyz[:,1])
    lane_path_y_center = lane_path_y[0]
    path_xyz_y_center = path_xyz[:,1][0]

    ## laneless일때는 center보정 TODO: 이걸해야되나? 
    ##일단 지워야겠다. 저속에서 차를 피해가는데 왜 보정을 해?
    #diff_center = (lane_path_y_center - path_xyz_y_center) if not self.lanefull_mode else 0.0
    diff_center = 0.0
    #print("center = {:.2f}={:.2f}-{:.2f}, lanefull={}".format(diff_center, lane_path_y_center, path_xyz_y_center, self.lanefull_mode))
    #diff_center = lane_path_y[5] - path_xyz[:,1][5] if not self.lanefull_mode else 0.0
    if offset_curve * offset_lane < 0:
      offset_total = clip(offset_curve + offset_lane + diff_center, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)
    else:
      offset_total = clip(max(offset_curve, offset_lane, key=abs) + diff_center, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)

    ## self.d_prob = 0 if lane_changing
    self.d_prob *= self.lane_change_multiplier  ## 차선변경중에는 꺼버림.
    if self.lane_change_multiplier < 0.5:
      self.lane_offset_filtered.x = 0.0
    else:
      self.lane_offset_filtered.update(interp(self.d_prob, [0, 0.3], [0, offset_total]))

    ## laneless at lowspeed
    self.d_prob *= interp(v_ego*3.6, [5., 10.], [0.0, 1.0])

    self.debugText = "OFFSET({:.2f}={:.2f}+{:.2f}+{:.2f}),Vc:{:.2f},dp:{:.1f},lf:{},lrw={:.1f}|{:.1f}|{:.1f}".format(
      self.lane_offset_filtered.x,
      diff_center, offset_lane, offset_curve,
      curve_speed,
      self.d_prob, self.lanefull_mode,
      self.lane_width_left_filtered.x, self.lane_width, self.lane_width_right_filtered.x)

    useLaneLineDebug = Params().get_int("UseLaneLineDebug")
    if self.lanefull_mode:        
      if useLaneLineDebug == 0:
        safe_idxs = np.isfinite(self.ll_t)
        if safe_idxs[0]:
          lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
          path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
      else:
        #safe_idxs = np.isfinite(self.ll_x)
        #if safe_idxs[0]:
        #TODO: 여기에 왜? v_ego*0.1을 했을까? 까먹음...
        lane_path_y_interp = np.interp(path_xyz[:,0] + v_ego * useLaneLineDebug*0.01, self.ll_x, lane_path_y)
        path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]

    path_xyz[:, 1] += (CAMERA_OFFSET + self.lane_offset_filtered.x)

    self.offset_total = offset_total #self.lane_offset_filtered.x

    return path_xyz
