#!/usr/bin/env python3
import math
import numpy as np
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params
from cereal import log, car

import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, N
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.gm.values import CAR, EV_CAR
TransmissionType = car.CarParams.TransmissionType

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
A_CRUISE_MAX_BP_APILOT = [0., 40 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 80 * CV.KPH_TO_MS, 110 * CV.KPH_TO_MS, 140 * CV.KPH_TO_MS]


# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False
    self.dt = dt

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.v_model_error = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.params = Params()
    self.param_read_counter = 0
    self.read_param()

  #ajouatom
    self.cruiseMaxVals1 = 1.6
    self.cruiseMaxVals2 = 1.2
    self.cruiseMaxVals3 = 1.0
    self.cruiseMaxVals4 = 0.8
    self.cruiseMaxVals5 = 0.7
    self.cruiseMaxVals6 = 0.6
    self.cruiseMInVals = -1.2

    self.v_cruise_last = 0.0
    self.vCluRatio = 1.0
  
  def read_param(self):
    # ajouatom
    self.cruiseMaxVals1 = float(self.params.get_int("CruiseMaxVals1")) / 100.
    self.cruiseMaxVals2 = float(self.params.get_int("CruiseMaxVals2")) / 100.
    self.cruiseMaxVals3 = float(self.params.get_int("CruiseMaxVals3")) / 100.
    self.cruiseMaxVals4 = float(self.params.get_int("CruiseMaxVals4")) / 100.
    self.cruiseMaxVals5 = float(self.params.get_int("CruiseMaxVals5")) / 100.
    self.cruiseMaxVals6 = float(self.params.get_int("CruiseMaxVals6")) / 100.
    self.cruiseMinVals = -float(self.params.get_int("CruiseMinVals")) / 100.
    self.myHighModeFactor = float(self.params.get_int("MyHighModeFactor")) / 100.
    
  def get_carrot_accel(self, v_ego, curveSpeed, angle_steers):
    cruiseMaxVals = [self.cruiseMaxVals1, self.cruiseMaxVals2, self.cruiseMaxVals3, self.cruiseMaxVals4, self.cruiseMaxVals5, self.cruiseMaxVals6]
    #apply_curve_speed = interp(v_ego, [0, 10 * CV.KPH_TO_MS], [300, abs(curveSpeed)])
    apply_angle_steers = interp(angle_steers, [0, 10, 50], [1.0, 0.8, 0.1])
    #return interp(v_ego, A_CRUISE_MAX_BP_APILOT, cruiseMaxVals) * interp(apply_curve_speed, [0, 120], [0.1, 1.0]) * apply_angle_steers
    return interp(v_ego, A_CRUISE_MAX_BP_APILOT, cruiseMaxVals) * apply_angle_steers
    
  @staticmethod
  def parse_model(model_msg, model_error):
    if (len(model_msg.position.x) == 33 and
       len(model_msg.velocity.x) == 33 and
       len(model_msg.acceleration.x) == 33):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    return x, v, a, j

  def update(self, sm, carrot_planner):
    if self.param_read_counter % 50 == 0:
      self.read_param()
    self.param_read_counter += 1
    self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'
    self.mpc.experimentalMode = sm['controlsState'].experimentalMode

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['controlsState'].vCruise, V_CRUISE_MAX)

    v_cruise_kph = carrot_planner.update(sm, v_cruise_kph)

    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    vCluRatio = sm['carState'].vCluRatio
    if vCluRatio > 0.5:
      self.vCluRatio = vCluRatio
      v_cruise *= vCluRatio
      #v_cruise = int(v_cruise * CV.MS_TO_KPH + 0.25) * CV.KPH_TO_MS

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel
    soft_hold = sm['carControl'].hudControl.softHold > 0 #ajouatom

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off or soft_hold if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      #accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      carrot_accel = self.get_carrot_accel(v_ego, carrot_planner.curveSpeed, sm['carState'].steeringAngleDeg)*self.mpc.mySafeFactor
      if self.mpc.myDrivingMode == 4: # high mode
        carrot_accel *= self.myHighModeFactor
      myMaxAccel = clip(carrot_accel, 0.05, ACCEL_MAX)
      accel_limits = [self.cruiseMinVals, myMaxAccel]      
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])
      
      #ajouatom
      self.mpc.prev_a = np.full(N+1, self.a_desired) ## mpc에서는 prev_a를 참고하여 constraint작동함.... pid off -> on시에는 현재 constraint가 작동하지 않아서 집어넣어봄...
      accel_limits_turns[0] = accel_limits_turns[0] = 0.0

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    # Compute model v_ego error
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    if force_slow_decel:
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    #self.mpc.set_weights(prev_accel_constraint, personality=sm['controlsState'].personality)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    x, v, a, j = self.parse_model(sm['modelV2'], self.v_model_error)
    self.v_cruise_last = v_cruise

    self.mpc.update(sm, reset_state, prev_accel_constraint, sm['radarState'],  v_cruise, x, v, a, j, carrot_planner, personality=sm['controlsState'].personality)

    self.v_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill and not reset_state
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    if self.CP.transmissionType == TransmissionType.direct and EV_CAR:
      self.a_desired = float(interp(self.CP.radarTimeStep, ModelConstants.T_IDXS[:CONTROL_N], self.a_desired_trajectory))
      self.v_desired_filter.x = self.v_desired_filter.x + self.CP.radarTimeStep * (self.a_desired + a_prev) / 2.0
    else:
      self.a_desired = float(interp(self.dt, ModelConstants.T_IDXS[:CONTROL_N], self.a_desired_trajectory))
      self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm, carrot_planner):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.debugLongText = self.mpc.debugLongText
    #longitudinalPlan.debugLongText2 = "VC:{:.1f}".format(self.v_cruise_last*3.6)
    longitudinalPlan.trafficState = self.mpc.trafficState.value
    longitudinalPlan.xState = self.mpc.xState.value
    longitudinalPlan.tFollow = float(self.mpc.t_follow)

    longitudinalPlan.curveSpeed = float(carrot_planner.curveSpeed)
    longitudinalPlan.activeAPM = carrot_planner.activeAPM
    longitudinalPlan.leftBlinkerExt = carrot_planner.leftBlinkerExt
    longitudinalPlan.rightBlinkerExt = carrot_planner.rightBlinkerExt
    longitudinalPlan.limitSpeed = carrot_planner.limitSpeed
    longitudinalPlan.carrotEvent = carrot_planner.event
    longitudinalPlan.vCruiseTarget = float(carrot_planner.v_cruise_kph)
    longitudinalPlan.vCruiseTargetSource = carrot_planner.source

    longitudinalPlan.debugLongText2 = carrot_planner.log

    pm.send('longitudinalPlan', plan_send)
