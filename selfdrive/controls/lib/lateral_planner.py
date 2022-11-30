import numpy as np
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import interp
from selfdrive.controls.lib.lane_planner import LanePlanner
from common.params import Params
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error
from selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

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
  def __init__(self, CP):
    self.DH = DesireHelper()
    self.LP = LanePlanner()
    self.readParams = 0
    self.lanelines_active = False
    self.lanelines_active_tmp = False

    self.pathOffset = float(int(Params().get("PathOffset", encoding="utf8")))*0.01
    self.pathCostApply = float(int(Params().get("PathCostApply", encoding="utf8")))*0.01
    self.pathCostApplyLow = float(int(Params().get("PathCostApplyLow", encoding="utf8")))*0.01
    self.lateralMotionCost = float(int(Params().get("LateralMotionCost", encoding="utf8")))*0.01
    self.lateralAccelCost = float(int(Params().get("LateralAccelCost", encoding="utf8")))*0.01
    self.lateralJerkCost = float(int(Params().get("LateralJerkCost", encoding="utf8")))*0.01
    self.useLaneLineSpeed = float(int(Params().get("UseLaneLineSpeed", encoding="utf8")))

    self.useLaneLineMode = False

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros((TRAJECTORY_SIZE,))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.v_ego = 0.0
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0
    self.d_path_w_lines_xyz = np.zeros((TRAJECTORY_SIZE, 3))

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

    self.steeringRateCost = 700.

  def reset_mpc(self, x0=np.zeros(4)):
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    self.readParams -= 1
    if self.readParams <= 0:
      self.readParams = 100
      self.useLaneLineSpeed = float(int(Params().get("UseLaneLineSpeed", encoding="utf8")))
      self.pathOffset = float(int(Params().get("PathOffset", encoding="utf8")))*0.01
      self.pathCostApply = float(int(Params().get("PathCostApply", encoding="utf8")))*0.01
      self.pathCostApplyLow = float(int(Params().get("PathCostApplyLow", encoding="utf8")))*0.01
      self.steeringRateCost = float(int(Params().get("SteeringRateCost", encoding="utf8")))
    elif self.readParams == 50:
      self.lateralMotionCost = float(int(Params().get("LateralMotionCost", encoding="utf8")))*0.01
      self.lateralAccelCost = float(int(Params().get("LateralAccelCost", encoding="utf8")))*0.01
      self.lateralJerkCost = float(int(Params().get("LateralJerkCost", encoding="utf8")))*0.01

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
    turn_prob = self.LP.l_turn_prob + self.LP.r_turn_prob
    # Lane change logic
    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, md, turn_prob)

    if self.v_ego*3.6 >= self.useLaneLineSpeed + 2:
      self.useLaneLineMode = True
    elif self.v_ego*3.6 < self.useLaneLineSpeed - 2:
      self.useLaneLineMode = False

    if self.useLaneLineMode and self.useLaneLineSpeed > 0:
      # Turn off lanes during lane change
      if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
        self.LP.lll_prob *= self.DH.lane_change_ll_prob
        self.LP.rll_prob *= self.DH.lane_change_ll_prob
  
      # dynamic laneline/laneless logic
      if self.LP.lll_prob < 0.3 and self.LP.rll_prob < 0.3:
        self.lanelines_active_tmp = False
      elif self.LP.lll_prob > 0.5 and self.LP.rll_prob > 0.5:
        self.lanelines_active_tmp = True
      self.lanelines_active = self.lanelines_active_tmp
  
      # Calculate final driving path and set MPC costs
      if self.lanelines_active:
        self.path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
      
    else:
      self.lanelines_active = False

    self.path_xyz[:, 1] += self.pathOffset

    pathCost = interp(self.v_ego, [30./3.6, 100/3.6], [self.pathCostApplyLow, self.pathCostApply])
    #steeringRateCost = interp(self.v_ego, [2., 10.], [self.steeringRateCost, self.steeringRateCost/3.])
    #self.lat_mpc.set_weights(pathCost, LATERAL_MOTION_COST,
    #                         LATERAL_ACCEL_COST, LATERAL_JERK_COST,
    #                         steeringRateCost)
    self.lat_mpc.set_weights(pathCost, self.lateralMotionCost,
                             self.lateralAccelCost, self.lateralJerkCost,
                             self.steeringRateCost)

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
    t = sec_since_boot()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature * self.v_ego
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.lat_mpc.cost > 20000. or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    #C2#lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(x/self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    #C2#lateralPlan.solverExecutionTime = self.lat_mpc.solve_time

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self.lanelines_active
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction
    lateralPlan.desireEvent = self.DH.desireEvent
    lateralPlan.laneWidth = 3.7 # float(self.LP.lane_width)

    #C2#plan_send.lateralPlan.dPathWLinesX = [float(x) for x in self.d_path_w_lines_xyz[:, 0]]
    #C2#plan_send.lateralPlan.dPathWLinesY = [float(y) for y in self.d_path_w_lines_xyz[:, 1]]

    pm.send('lateralPlan', plan_send)
