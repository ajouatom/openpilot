#!/usr/bin/env python3
import os
import time
import numpy as np
from cereal import log
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.car.interfaces import ACCEL_MIN
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params

if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

from openpilot.common.realtime import DT_MDL
from openpilot.common.filter_simple import StreamingMovingAverage
from enum import Enum

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']

X_DIM = 3
U_DIM = 1
PARAM_DIM = 8
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 6. #3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.8 #0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'


# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
T_FOLLOW = 1.45
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.5

tFollowGap1 = 1.1
tFollowGap2 = 1.2
tFollowGap3 = 1.4
tFollowGap4 = 1.6

class XState(Enum):
  lead = 0
  cruise = 1
  e2eCruise = 2
  e2eStop = 3
  e2ePrepare = 4
  e2eStopped = 5

  def __str__(self):
    return self.name

class TrafficState(Enum):
  off = 0
  red = 1
  green = 2

  def __str__(self):
    return self.name

def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.moreRelaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.standard:
    return 1.0
  elif personality==log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  print("get_T_FOLLOW no!!!")
  if personality==log.LongitudinalPersonality.moreRelaxed:
    return 2.00
  elif personality==log.LongitudinalPersonality.relaxed:
    return 1.75
  elif personality==log.LongitudinalPersonality.standard:
    return 1.45
  elif personality==log.LongitudinalPersonality.aggressive:
    return 1.25
  else:
    raise NotImplementedError("Longitudinal personality not supported")

def get_stopped_equivalence_factor(v_lead, v_ego, t_follow=get_T_FOLLOW(), stop_distance=STOP_DISTANCE):
  return (v_lead**2) / (2 * COMFORT_BRAKE)

def get_safe_obstacle_distance(v_ego, t_follow=get_T_FOLLOW(), comfort_brake=COMFORT_BRAKE, stop_distance=STOP_DISTANCE):
  return (v_ego**2) / (2 * comfort_brake) + t_follow * v_ego + stop_distance

def desired_follow_distance(v_ego, v_lead, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead, v_ego)


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # set up states & controls
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  comfort_brake = SX.sym('comfort_brake')
  stop_distance = SX.sym('stop_distance')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor, comfort_brake, stop_distance)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]
  comfort_brake = ocp.model.p[6]
  stop_distance = ocp.model.p[7]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow, comfort_brake, stop_distance)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  #ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, get_T_FOLLOW(), LEAD_DANGER_FACTOR, COMFORT_BRAKE, STOP_DISTANCE])
  #ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, T_FOLLOW, LEAD_DANGER_FACTOR, COMFORT_BRAKE, STOP_DISTANCE])
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, lead_t_follow, LEAD_DANGER_FACTOR, comfort_brake, stop_distance])

  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, mode='acc'):
    self.trafficStopDistanceAdjust = 1.5
    self.aChangeCost = 200
    self.aChangeCostStart = 40
    self.tFollowSpeedAdd = 0.0
    self.tFollowSpeedAddM = 0.0
    self.tFollowLeadCarSpeed = 0.0
    self.tFollowLeadCarAccel = 0.0
    self.tFollowGap1 = 1.1
    self.tFollowGap2 = 1.2
    self.tFollowGap3 = 1.4
    self.tFollowGap4 = 1.6
    self.lo_timer = 0 
    self.v_ego_prev = 0.0
    self.trafficState = TrafficState.off
    self.xStopFilter = StreamingMovingAverage(3)
    self.xStopFilter2 = StreamingMovingAverage(15)
    self.vFilter = StreamingMovingAverage(10)
    self.t_follow_prev = self.get_T_FOLLOW()
    self.stop_distance = STOP_DISTANCE
    self.fakeCruiseDistance = 0.0
    self.comfortBrake = COMFORT_BRAKE
    self.comfort_brake = self.comfortBrake
    self.xState = XState.cruise
    self.xStop = 0.0
    self.actual_stop_distance = 0.0
    self.debugLongText = ""
    self.myDrivingMode = 3 # general mode
    self.mySafeModeFactor = 0.8
    self.myEcoModeFactor = 0.8
    self.mySafeFactor = 1.0
    self.stopping_count = 0
    self.traffic_starting_count = 0
    self.user_stop_distance = -1
    
    self.t_follow = 0

    self.mode = mode
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2]
    self.experimentalMode = False

  def reset(self):
    # self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.solver.reset()
    # self.solver.options_set('print_level', 2)
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N,1))
    self.params = np.zeros((N+1, PARAM_DIM))
    self.stop_distance = STOP_DISTANCE
    self.comfort_brake = self.comfortBrake
    self.xState = XState.cruise
    self.startSignCount = 0
    self.stopSignCount = 0
    self.trafficState = TrafficState.off
    
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # TODO don't hardcode A_CHANGE_COST idx
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):
    jerk_factor = get_jerk_factor(personality)
    if self.mode == 'acc':
      a_change_cost = self.aChangeCost if prev_accel_constraint else self.aChangeCostStart
      cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, jerk_factor * a_change_cost, jerk_factor * J_EGO_COST]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else self.aChangeCostStart
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:  # probably only helps if v < v_prev
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      #a_lead = lead.aLeadK
      a_lead = lead.aLead
      a_lead_tau = lead.aLeadTau
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def update_tf(self, v_ego, t_follow):
    t_follow = interp(v_ego * CV.MS_TO_KPH, [0, 40, 100], [t_follow, t_follow + self.tFollowSpeedAddM, t_follow + self.tFollowSpeedAdd]) 
    #t_follow = interp(v_ego * CV.MS_TO_KPH, [0, 100], [t_follow, t_follow + self.tFollowSpeedAdd])
    t_follow = max(0.6, t_follow * (2.0 - self.mySafeFactor)) 
    if v_ego < self.v_ego_prev:
      t_follow = max(t_follow, self.t_follow_prev)
    self.t_follow_prev = t_follow
    self.v_ego_prev = v_ego
    #return np.full(N+1, t_follow)
    return t_follow

  def update_dynamic_tf(self, t_follow, lead_data, a_ego, v_ego):
    if lead_data.status:
      t_follow *= clip(1.0 - lead_data.vRel * self.tFollowLeadCarSpeed, -1.2, 1.2)
      t_follow *= clip(1.0 - min(lead_data.aLeadK, 0.0) * self.tFollowLeadCarAccel, -1.2, 1.2)

    #t_follow *= clip(1.0 - min(a_ego, 0.0) * self.tFollowMyCarAccel, -1.2, 1.2)
    return t_follow

  def set_accel_limits(self, min_a, max_a):
    # TODO this sets a max accel limit, but the minimum limit is only for cruise decel
    # needs refactor
    self.cruise_min_a = min_a
    self.max_a = max_a

  def update(self, sm, reset_state, prev_accel_constraint, radarstate, v_cruise, x, v, a, j, carrot_planner, personality=log.LongitudinalPersonality.standard):
    t_follow = self.get_T_FOLLOW(personality)
    #self.debugLongText = "v_cruise ={:.1f}".format(v_cruise)
    carstate = sm['carState']
    controlsState = sm['controlsState']
    model = sm['modelV2']

    self.update_params()
    v_ego = self.x0[1]
    a_ego = self.x0[2]
    self.trafficState = TrafficState.off
    # carrot
    self.comfort_brake = self.comfortBrake
    applyStopDistance = self.stop_distance  * (2.0 - self.mySafeFactor)
    t_follow = self.update_tf(v_ego, t_follow)
    t_follow = self.update_dynamic_tf(t_follow, radarstate.leadOne, a_ego, v_ego)
    carrotTest3 = Params().get_int("CarrotTest3")
    if carrotTest3 in [1,2]:
      check_cut_out = radarstate.leadOne.dPath * radarstate.leadOne.vLat
      #print("{:.1f}, {:.1f}".format(check_cut_out, radarstate.leadOne.dPath + radarstate.leadOne.vLat))
      if check_cut_out > 0:
        t_follow *= interp(abs(radarstate.leadOne.dPath + radarstate.leadOne.vLat), [0.5, 1.0, 2.0], [1.0, 0.5, 0.2])
      elif carrotTest3 == 2:
        t_follow *= interp(abs(radarstate.leadOne.vLat), [0.5, 1.0, 2.0], [1.0, 1.1, 1.3])
    self.t_follow = t_follow
    
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1], self.x_sol[:,1], self.t_follow, applyStopDistance)
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1], self.x_sol[:,1], self.t_follow, applyStopDistance)
    self.params[:,0] = ACCEL_MIN if not reset_state else a_ego
    self.params[:,1] = self.max_a if not reset_state else a_ego

    if self.experimentalMode:
      self.mode == 'blended'
      stop_x = 1000.0
    else:
      v_cruise, stop_x, self.mode = self.update_apilot(carstate, controlsState, radarstate, model, v_cruise, carrot_planner)
    #self.debugLongText = "{},{},{:.1f},tf={:.2f},{:.1f},stop={:.1f},{:.1f},xv={:.0f},{:.0f}".format(
    #  str(self.xState), str(self.trafficState), v_cruise*3.6, t_follow, t_follow*v_ego+6.0, stop_x, self.actual_stop_distance,x[-1],v[-1])
    xe, ve = x[-1], v[-1]
    # TODO: e2eStop시 속도증가된는 문제발생, 일단 속도제한해보자.. 왜그러지? cruise_obstacle이 더 작을텐데...
    if self.xState == XState.e2eStop:
      self.max_a = 0.0

    self.set_weights(prev_accel_constraint=prev_accel_constraint, personality=personality)

    # Update in ACC mode or ACC/e2e blend
    if self.mode == 'acc':
      self.params[:,5] = LEAD_DANGER_FACTOR

      adjustDist = self.trafficStopDistanceAdjust if v_ego > 0.1 else -2.0
      x2 = stop_x * np.ones(N+1) + adjustDist


      # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
      # when the leads are no factor.
      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow, self.comfort_brake, applyStopDistance + self.fakeCruiseDistance)
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle, x2])
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      # These are not used in ACC mode
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0
      #print("{:.1f}, {:.1f}".format(lead_0_obstacle[0], cruise_obstacle[0]))

    elif self.mode == 'blended':
      self.params[:,5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle,
                                     lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1,0] < x_and_cruise[1,1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = t_follow
    self.params[:,6] = self.comfort_brake
    self.params[:,7] = applyStopDistance

    self.debugLongText = "{},tf={:.2f},{:.1f},stop={:.1f},{:.1f},xv={:.0f},{:.0f},xt={:.0f},{:.0f}".format(
      str(self.xState), t_follow, t_follow*v_ego+6.0, stop_x, self.actual_stop_distance,xe,ve, self.params[:,2][0], lead_0_obstacle[0])

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    # Check if it got within lead comfort range
    # TODO This should be done cleaner
    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow, self.comfort_brake, applyStopDistance)) - self.x_sol[:,0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow, self.comfort_brake, applyStopDistance)) - self.x_sol[:,0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

  def run(self):
    # t0 = time.monotonic()
    # reset = 0
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    # qp_iter = self.solver.get_stats('statistics')[-1][-1] # SQP_RTI specific
    # print(f"long_mpc timings: tot {self.solve_time:.2e}, qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e}, \
    # integrator {self.time_integrator:.2e}, qp_iter {qp_iter}")
    # res = self.solver.get_residuals()
    # print(f"long_mpc residuals: {res[0]:.2e}, {res[1]:.2e}, {res[2]:.2e}, {res[3]:.2e}")
    # self.solver.print_statistics()

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.prev_a = np.interp(T_IDXS + 0.05, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()
      # reset = 1
    # print(f"long_mpc timings: total internal {self.solve_time:.2e}, external: {(time.monotonic() - t0):.2e} qp {self.time_qp_solution:.2e}, \
    # lin {self.time_linearization:.2e} qp_iter {qp_iter}, reset {reset}")

  def get_T_FOLLOW(self, personality=log.LongitudinalPersonality.standard):
    if personality==log.LongitudinalPersonality.moreRelaxed:
      return self.tFollowGap4 #1.75
    elif personality==log.LongitudinalPersonality.relaxed:
      return self.tFollowGap3 #1.75
    elif personality==log.LongitudinalPersonality.standard:
      return self.tFollowGap2 #1.45
    elif personality==log.LongitudinalPersonality.aggressive:
      return self.tFollowGap1 #1.25
    else:
      raise NotImplementedError("Longitudinal personality not supported")

  def update_params(self):
    self.lo_timer += 1

    params = Params()
    if self.lo_timer % 2 == 0:
      self.myDrivingMode = params.get_int("MyDrivingMode")
      self.myEcoModeFactor = params.get_float("MyEcoModeFactor") / 100.
      self.mySafeModeFactor = params.get_float("MySafeModeFactor") / 100.
      self.mySafeFactor = 1.0
      if self.myDrivingMode == 1: # eco
        self.mySafeFactor = self.myEcoModeFactor
      elif self.myDrivingMode == 2: #safe
        self.mySafeFactor = self.mySafeModeFactor

    if self.lo_timer > 200:
      self.lo_timer = 0
    elif self.lo_timer == 20:
      pass
    elif self.lo_timer == 80:
      self.tFollowSpeedAdd = params.get_float("TFollowSpeedAdd") / 100.
      self.tFollowSpeedAddM = params.get_float("TFollowSpeedAddM") / 100.
      self.tFollowLeadCarSpeed = params.get_float("TFollowLeadCarSpeed") / 1000.
      self.tFollowLeadCarAccel = params.get_float("TFollowLeadCarAccel") / 100.
    elif self.lo_timer == 100:
      self.tFollowGap1 = params.get_float("TFollowGap1") / 100.
      self.tFollowGap2 = params.get_float("TFollowGap2") / 100.
      self.tFollowGap3 = params.get_float("TFollowGap3") / 100.
      self.tFollowGap4 = params.get_float("TFollowGap4") / 100.
    elif self.lo_timer == 120:
      self.stop_distance = params.get_float("StopDistanceCarrot") / 100.
      self.trafficStopDistanceAdjust = params.get_float("TrafficStopDistanceAdjust") / 100.
      self.comfortBrake = params.get_float("ComfortBrake") / 100.

  def update_stop_dist(self, stop_x):
    stop_x = self.xStopFilter.process(stop_x, median = True)
    stop_x = self.xStopFilter2.process(stop_x)
    return stop_x


  def check_model_stopping(self, v, v_ego, model_x, y):
    v_ego_kph = v_ego * CV.MS_TO_KPH
    model_v = self.vFilter.process(v[-1])
    startSign = model_v > 5.0 or model_v > (v[0]+2)

    if v_ego_kph < 1.0:
      stopSign = model_x < 20.0 and model_v < 10.0
    elif v_ego_kph < 82.0:
      stopSign = model_x < interp(v[0], [60/3.6, 80/3.6], [120.0, 150]) and ((model_v < 3.0) or (model_v < v[0]*0.7))  and abs(y[-1]) < 5.0
    else:
      stopSign = False

    #self.stopSignCount = self.stopSignCount + 1 if (stopSign and (model_x > get_safe_obstacle_distance(v_ego, t_follow=0, comfort_brake=COMFORT_BRAKE, stop_distance=-1.0))) else 0
    self.stopSignCount = self.stopSignCount + 1 if stopSign else 0
    self.startSignCount = self.startSignCount + 1 if startSign and not stopSign else 0

    if self.stopSignCount * DT_MDL > 0.0:
      self.trafficState = TrafficState.red
    elif self.startSignCount * DT_MDL > 0.2:
      self.trafficState = TrafficState.green
    else:
      self.trafficState = TrafficState.off


  def update_apilot(self, carstate, controlsState, radarstate, model, v_cruise, carrot_planner):
    v_ego = carstate.vEgo
    v_ego_kph = v_ego * CV.MS_TO_KPH
    x = model.position.x
    y = model.position.y
    v = model.velocity.x

    self.fakeCruiseDistance = 0.0
    radar_detected = radarstate.leadOne.status & radarstate.leadOne.radar

    self.xStop = self.update_stop_dist(x[31])
    stop_model_x = self.xStop

    #self.check_model_stopping(v, v_ego, self.xStop, y)
    self.check_model_stopping(v, v_ego, x[-1], y)

    if (carstate.rightBlinker and not carstate.leftBlinker) or self.myDrivingMode == 4 or (carrot_planner.rightBlinkerExt % 10000) > 0:
      self.trafficState = TrafficState.off
    elif controlsState.trafficLight in [22, 2]:
      self.trafficState = TrafficState.green
      self.xState = XState.e2eCruise
      self.traffic_starting_count = 10.0 / DT_MDL  ##신호출발시 10초가 될때까지 신호감지를 정지함.
    elif controlsState.trafficLight in [11, 1] and self.user_stop_distance < 0:
      user_stop_decel = 1.0
      self.user_stop_distance = v_ego ** 2 / (user_stop_decel * 2)

    if carstate.gasPressed or carstate.brakePressed:
      self.user_stop_distance = -1

    if self.xState == XState.e2eStopped:
      if carstate.gasPressed:
        self.xState = XState.e2ePrepare
      elif radar_detected and (radarstate.leadOne.dRel - stop_model_x) < 2.0:
        self.xState = XState.lead
      elif self.stopping_count == 0:
        if self.trafficState == TrafficState.green:
          self.xState = XState.e2ePrepare
      self.stopping_count = max(0, self.stopping_count - 1)
      v_cruise = 0
    elif self.xState == XState.e2eStop:
      self.stopping_count = 0
      if carstate.gasPressed:
        self.xState = XState.e2ePrepare
      elif radar_detected and (radarstate.leadOne.dRel - stop_model_x) < 2.0:
        self.xState = XState.lead
      else:
        if self.trafficState == TrafficState.green:
          self.xState = XState.e2ePrepare
        else:
          self.comfort_brake = COMFORT_BRAKE * 0.9
          #self.comfort_brake = COMFORT_BRAKE
          self.trafficStopAdjustRatio = interp(v_ego_kph, [0, 100], [1.0, 0.7])
          stop_dist = self.xStop * interp(self.xStop, [0, 100], [1.0, self.trafficStopAdjustRatio])  ##남은거리에 따라 정지거리 비율조정
          if stop_dist > 10.0: ### 10M이상일때만, self.actual_stop_distance를 업데이트함.
            self.actual_stop_distance = stop_dist
          stop_model_x = 0
          self.fakeCruiseDistance = 0 if self.actual_stop_distance > 10.0 else 10.0
          if v_ego < 0.3:
            self.stopping_count = 0.5 / DT_MDL
            self.xState = XState.e2eStopped
    elif self.xState == XState.e2ePrepare:
      if self.status:
        self.xState = XState.lead
      elif v_ego_kph < 5.0 and self.trafficState != TrafficState.green:
        self.xState = XState.e2eStop
        self.actual_stop_distance = 2.0
      elif v_ego_kph > 5.0: # and stop_model_x > 30.0:
        self.xState = XState.e2eCruise
    else: #XState.lead, XState.cruise, XState.e2eCruise
      self.traffic_starting_count = max(0, self.traffic_starting_count - 1)
      if self.status:
        self.xState = XState.lead
      elif self.trafficState == TrafficState.red and abs(carstate.steeringAngleDeg) < 30 and self.traffic_starting_count == 0:
        self.xState = XState.e2eStop
        self.actual_stop_distance = self.xStop
      else:
        self.xState = XState.e2eCruise

    if self.trafficState in [TrafficState.off, TrafficState.green] or self.xState not in [XState.e2eStop, XState.e2eStopped]:
      stop_model_x = 1000.0

    if self.user_stop_distance >= 0:
      self.user_stop_distance = max(0, self.user_stop_distance - v_ego * DT_MDL)
      self.actual_stop_distance = self.user_stop_distance
      self.xState = XState.e2eStop if self.user_stop_distance > 0 else XState.e2eStopped
      
    mode = 'blended' if self.xState in [XState.e2ePrepare] else 'acc'

    self.comfort_brake *= self.mySafeFactor
    self.actual_stop_distance = max(0, self.actual_stop_distance - (v_ego * DT_MDL))
    
    if stop_model_x == 1000.0: ##  e2eCruise, lead인경우
      self.actual_stop_distance = 0.0
    elif self.actual_stop_distance > 0: ## e2eStop, e2eStopped인경우..
      stop_model_x = 0.0
      
    #self.debugLongText = "XState({}),stop_x={:.1f},stopDist={:.1f},Traffic={}".format(str(self.xState), stop_x, self.actual_stop_distance, str(self.trafficState))
    #번호를 읽을때는 self.xState.value
      
    stop_dist =  stop_model_x + self.actual_stop_distance
    stop_dist = max(stop_dist, v_ego ** 2 / (self.comfort_brake * 2))
    return v_cruise, stop_dist, mode


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
