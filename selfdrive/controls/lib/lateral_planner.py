import time
import numpy as np
from openpilot.common.realtime import DT_MDL
from openpilot.common.numpy_fast import interp
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

from openpilot.common.params import Params
#from openpilot.selfdrive.controls.lib.lane_planner import LanePlanner
from openpilot.selfdrive.controls.lib.lane_planner_2 import LanePlanner


TRAJECTORY_SIZE = 33
CAMERA_OFFSET = 0.04


PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.04
# Extreme steering rate is unpleasant, even
# when it does not cause bad jerk.
# TODO this cost should be lowered when low
# speed lateral control is stable on all cars
STEERING_RATE_COST = 700.0


class LateralPlanner:
  def __init__(self, CP, debug=False):
    self.DH = DesireHelper()

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.x_sol = np.zeros((TRAJECTORY_SIZE, 4), dtype=np.float32)
    self.v_ego = MIN_SPEED
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0

    self.debug_mode = debug

    self.latDebugText = ""
    # lane_mode
    self.LP = LanePlanner()
    self.readParams = 0
    self.lanelines_active = False
    self.lanelines_active_tmp = False

    self.pathOffset = float(int(Params().get("PathOffset", encoding="utf8")))*0.01
    self.useLaneLineSpeed = float(int(Params().get("UseLaneLineSpeed", encoding="utf8")))
    self.useLaneLineMode = False
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros((TRAJECTORY_SIZE,))
    self.d_path_w_lines_xyz = np.zeros((TRAJECTORY_SIZE, 3))

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))
    self.vcurv = []

  def reset_mpc(self, x0=None):
    if x0 is None:
      x0 = np.zeros(4)
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    self.readParams -= 1
    if self.readParams <= 0:
      self.readParams = 100
      self.useLaneLineSpeed = float(int(Params().get("UseLaneLineSpeed", encoding="utf8")))
      self.pathOffset = float(int(Params().get("PathOffset", encoding="utf8")))*0.01
    if self.useLaneLineSpeed > 0:
      self.update_lane_mode(sm)
      return
    v_ego_car = sm['carState'].vEgo

    # Parse model predictions
    md = sm['modelV2']
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.velocity.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolution.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
      car_speed = np.linalg.norm(self.velocity_xyz, axis=1) - get_speed_error(md, v_ego_car)
      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
      self.v_ego = self.v_plan[0]
      self.x_sol = np.column_stack([md.lateralPlannerSolution.x, md.lateralPlannerSolution.y, md.lateralPlannerSolution.yaw, md.lateralPlannerSolution.yawRate])

    # Lane change logic
    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]
    lane_change_prob = self.l_lane_change_prob + self.r_lane_change_prob
    self.DH.update(sm['carState'], md, sm['carControl'].latActive, lane_change_prob, sm)

  def publish(self, sm, pm):
    if self.useLaneLineSpeed > 0:
      self.publish_lane_mode(sm, pm)
      return
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.dPathPoints = self.path_xyz[:,1].tolist()
    lateralPlan.psis = self.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlan.curvatures = (self.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(0) for _ in range(CONTROL_N-1)] # TODO: unused

    lateralPlan.mpcSolutionValid = bool(1)
    lateralPlan.solverExecutionTime = 0.0
    if self.debug_mode:
      lateralPlan.solverState = log.LateralPlan.SolverState.new_message()
      lateralPlan.solverState.x = self.x_sol.tolist()

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = False
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    # ajouatom
    lateralPlan.laneWidth = 3.0 #float(self.LP.lane_width)
    lateralPlan.latDebugText = self.latDebugText
    lateralPlan.laneWidthLeft = float(self.DH.lane_width_left)
    lateralPlan.laneWidthRight = float(self.DH.lane_width_right)
    lateralPlan.distanceToRoadEdgeLeft = float(self.DH.distance_to_road_edge_left)
    lateralPlan.distanceToRoadEdgeRight = float(self.DH.distance_to_road_edge_right)

    pm.send('lateralPlan', plan_send)
    
  def update_lane_mode(self, sm):
    # clip speed , lateral planning is not possible at 0 speed
    measured_curvature = sm['controlsState'].curvature
    v_ego_car = sm['carState'].vEgo

    # Parse model predictions
    md = sm['modelV2']
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = np.array(md.orientation.z)
      self.plan_yaw_rate = np.array(md.orientationRate.z)
      self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
      car_speed = np.linalg.norm(self.velocity_xyz, axis=1) - get_speed_error(md, v_ego_car)
      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
      self.v_ego = self.v_plan[0]

    # Parse model predictions
    self.LP.parse_model(md)
    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    self.DH.update(sm['carState'], md, sm['carControl'].latActive, lane_change_prob, sm)

    if self.v_ego*3.6 >= self.useLaneLineSpeed + 2:
      self.useLaneLineMode = True
    elif self.v_ego*3.6 < self.useLaneLineSpeed - 2:
      self.useLaneLineMode = False

    #if self.useLaneLineMode and self.useLaneLineSpeed > 0:
    #  # Turn off lanes during lane change
    #  if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
    #    self.LP.lll_prob *= self.DH.lane_change_ll_prob
    #    self.LP.rll_prob *= self.DH.lane_change_ll_prob
  
    #  # dynamic laneline/laneless logic
    #  if self.LP.lll_prob < 0.3 and self.LP.rll_prob < 0.3:
    #    self.lanelines_active_tmp = False
    #  elif self.LP.lll_prob > 0.5 and self.LP.rll_prob > 0.5:
    #    self.lanelines_active_tmp = True
    #  self.lanelines_active = self.lanelines_active_tmp
      
    #else:
    #  self.lanelines_active = False

    #self.latDebugText = "laneMode({}), active({}/{}), prob={:.2f}".format(self.useLaneLineMode, self.lanelines_active_tmp, self.lanelines_active, self.LP.lll_prob)
    # Calculate final driving path and set MPC costs
    #self.path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz, self.lanelines_active)

    # Turn off lanes during lane change
    #if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
    if self.DH.desire != log.LateralPlan.Desire.none:
      self.LP.lane_change_multiplier = self.DH.lane_change_ll_prob
    else:
      self.LP.lane_change_multiplier = 1.0

    # lanelines calculation?
    self.LP.lanefull_mode = self.useLaneLineMode
    self.LP.lane_width_left = self.DH.lane_width_left
    self.LP.lane_width_right = self.DH.lane_width_right
    self.LP.curvature = measured_curvature
    self.path_xyz = self.LP.get_d_path(sm['carState'], self.v_ego, self.t_idxs, self.path_xyz, self.vcurv)
    self.latDebugText = self.LP.debugText
    self.lanelines_active = True if self.LP.d_prob > 0.3 and self.LP.lanefull_mode else False

    self.path_xyz[:, 1] += self.pathOffset

    self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                             LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                             STEERING_RATE_COST)

    y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
    heading_pts = self.plan_yaw[:LAT_MPC_N+1]
    yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N+1]
    self.y_pts = y_pts

    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    assert len(yaw_rate_pts) == LAT_MPC_N + 1
    lateral_factor = np.clip(self.factor1 - (self.factor2 * self.v_plan**2), 0.0, np.inf)
    p = np.column_stack([self.v_plan, lateral_factor])
    self.lat_mpc.run(self.x0,
                     p,
                     y_pts,
                     heading_pts,
                     yaw_rate_pts)
    # init state for next iteration
    # mpc.u_sol is the desired second derivative of psi given x0 curv state.
    # with x0[3] = measured_yaw_rate, this would be the actual desired yaw rate.
    # instead, interpolate x_sol so that x0[3] is the desired yaw rate for lat_control.
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    #  Check for infeasible MPC solution
    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = time.monotonic()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature * self.v_ego
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.lat_mpc.cost > 1e6 or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish_lane_mode(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(x.item() / self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
    self.vcurv = (100 * self.lat_mpc.x_sol[:, 3]/self.v_ego).tolist()

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time
    if self.debug_mode:
      lateralPlan.solverCost = self.lat_mpc.cost
      lateralPlan.solverState = log.LateralPlan.SolverState.new_message()
      lateralPlan.solverState.x = self.lat_mpc.x_sol.tolist()
      lateralPlan.solverState.u = self.lat_mpc.u_sol.flatten().tolist()

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self.lanelines_active
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction
    lateralPlan.laneWidth = float(self.LP.lane_width)

    #plan_send.lateralPlan.dPathWLinesX = [float(x) for x in self.d_path_w_lines_xyz[:, 0]]
    #plan_send.lateralPlan.dPathWLinesY = [float(y) for y in self.d_path_w_lines_xyz[:, 1]]
    #lateralPlan.laneWidthLeft = float(self.DH.lane_width_left)
    #lateralPlan.laneWidthRight = float(self.DH.lane_width_right)
    
    self.x_sol = self.lat_mpc.x_sol


    lateralPlan.latDebugText = self.latDebugText
    lateralPlan.laneWidthLeft = float(self.DH.lane_width_left)
    lateralPlan.laneWidthRight = float(self.DH.lane_width_right)
    lateralPlan.distanceToRoadEdgeLeft = float(self.DH.distance_to_road_edge_left)
    lateralPlan.distanceToRoadEdgeRight = float(self.DH.distance_to_road_edge_right)

    pm.send('lateralPlan', plan_send)

    
