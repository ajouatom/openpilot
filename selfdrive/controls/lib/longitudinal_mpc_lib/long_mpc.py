#!/usr/bin/env python3
import os
import numpy as np

from common.realtime import sec_since_boot
from common.numpy_fast import clip, interp
from selfdrive.swaglog import cloudlog
from selfdrive.modeld.constants import index_function
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from common.conversions import Conversions as CV
from common.params import Params
from common.realtime import DT_MDL
from common.filter_simple import StreamingMovingAverage
from cereal import log, car
EventName = car.CarEvent.EventName

XState = log.LongitudinalPlan.XState

if __name__ == '__main__':  # generating code
  from third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython  # pylint: disable=no-name-in-module, import-error

from casadi import SX, vertcat

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

X_EGO_OBSTACLE_COST = 6.#3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'


DIFF_RADAR_VISION = 2.0
# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
MIN_ACCEL = -4.0 #-3.5
MAX_ACCEL = 2.5
T_FOLLOW = 1.45
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.5

def get_stopped_equivalence_factor(v_lead, v_ego, t_follow=T_FOLLOW, stop_distance=STOP_DISTANCE, krkeegan=False):
  if not krkeegan:
    return (v_lead**2) / (2 * COMFORT_BRAKE)

  # KRKeegan방법은 lead거리값을 고의로 늘려주어. solver가 더빠른 가속을 유발하도록 하는것 같음...
  # KRKeegan this offset rapidly decreases the following distance when the lead pulls
  # away, resulting in an early demand for acceleration.
  v_diff_offset = 0
  if np.all(v_lead - v_ego > 0):
    v_diff_offset = ((v_lead - v_ego) * 1.)
    v_diff_offset = np.clip(v_diff_offset, 0, stop_distance / 2)
    v_diff_offset = np.maximum(v_diff_offset * ((10 - v_ego)/10), 0)
  distance = (v_lead**2) / (2 * COMFORT_BRAKE) + v_diff_offset
  return distance

def get_safe_obstacle_distance(v_ego, t_follow=T_FOLLOW, comfort_brake=COMFORT_BRAKE, stop_distance=STOP_DISTANCE):
  return (v_ego**2) / (2 * comfort_brake) + t_follow * v_ego + stop_distance

def desired_follow_distance(v_ego, v_lead):
  return get_safe_obstacle_distance(v_ego) - get_stopped_equivalence_factor(v_lead, v_ego)


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
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, T_FOLLOW, LEAD_DANGER_FACTOR, COMFORT_BRAKE, STOP_DISTANCE])

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
    self.mode = mode
    self.debugLongText1 = ""
    self.debugLongText2 = ""
    self.trafficState = 0
    self.XEgoObstacleCost = 3.
    self.JEgoCost = 5.
    self.AChangeCost = 200.
    self.DangerZoneCost = 100.
    self.leadDangerFactor = LEAD_DANGER_FACTOR
    self.trafficStopDistanceAdjust = 0.
    self.applyLongDynamicCost = False
    self.trafficStopAccel = 1.
    self.trafficStopModelSpeed = True
    self.trafficStopMode = 1
    self.applyDynamicTFollow = 1.0
    self.applyDynamicTFollowApart = 1.0
    self.applyDynamicTFollowDecel = 1.0
    self.softHoldMode = 1
    self.tFollowRatio = 1.0
    self.stopDistance = STOP_DISTANCE
    self.softHoldTimer = 0
    self.lo_timer = 0 
    self.v_cruise = 0.
    self.xStopFilter = StreamingMovingAverage(3)  #11
    self.xStopFilter2 = StreamingMovingAverage(15) #3
    self.vFilter = StreamingMovingAverage(10)
    self.applyCruiseGap = 1.
    self.applyModelDistOrder = 32
    self.trafficStopUpdateDist = 10.0
    self.trafficDetectBrightness = 300
    self.fakeCruiseDistance = 0.0
    self.stopDist = 0.0
    self.e2eCruiseCount = 0
    self.mpcEvent = 0
    self.lightSensor = 0
    self.prev_x = 0

    self.t_follow = T_FOLLOW
    self.comfort_brake = COMFORT_BRAKE
    self.xState = XState.cruise
    self.xStop = 0.0
    self.e2ePaused = False
    self.trafficError = False
    self.longActiveUser = 0
    self.cruiseButtonCounter = 0
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2]
    self.x_obstacle_min = 0.0
    self.openpilotLongitudinalControl = False

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
    self.t_follow = T_FOLLOW
    self.comfort_brake = COMFORT_BRAKE
    self.xState = XState.cruise
    self.startSignCount = 0
    self.stopSignCount = 0

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

  def get_cost_multipliers(self, v_lead0, v_lead1):
    v_ego = self.x0[1]
    v_ego_bps = [0, 10]
    TFs = [1.2, 1.45, 1.8]
    # KRKeegan adjustments to costs for different TFs
    # these were calculated using the test_longitudial.py deceleration tests
  #TF에 의한 a,j,d cost변경
    a_change_tf = interp(self.t_follow, TFs, [.8, 1., 1.1]) # 가까울수록 작게
    j_ego_tf = interp(self.t_follow, TFs, [.8, 1., 1.1]) #가까울수록 작게
    d_zone_tf = interp(self.t_follow, TFs, [1.3, 1., 1.]) # 가까울수록 크게

    # KRKeegan adjustments to improve sluggish acceleration
    # do not apply to deceleration
    j_ego_v_ego = 1
    a_change_v_ego = 1
    if (v_lead0 - v_ego >= 0) and (v_lead1 - v_ego >= 0):  #상대차량이 현재속도보다 빠르다면...
      j_ego_v_ego = interp(v_ego, v_ego_bps, [.05, 1.])
      a_change_v_ego = interp(v_ego, v_ego_bps, [.05, 1.])
    # Select the appropriate min/max of the options
    j_ego = min(j_ego_tf, j_ego_v_ego)
    a_change = min(a_change_tf, a_change_v_ego)
    return (a_change, j_ego, d_zone_tf)

  def set_weights(self, prev_accel_constraint=True, v_lead0=0, v_lead1=0):
    if self.mode == 'acc':
      a_change_cost = self.AChangeCost if prev_accel_constraint else 0
      if self.applyLongDynamicCost:
        cost_mulitpliers = self.get_cost_multipliers(v_lead0, v_lead1)
        cost_weights = [self.XEgoObstacleCost, X_EGO_COST, V_EGO_COST, A_EGO_COST, a_change_cost * cost_mulitpliers[0], self.JEgoCost * cost_mulitpliers[1]]
        constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, self.DangerZoneCost * cost_mulitpliers[2]]
      else:
        cost_weights = [self.XEgoObstacleCost, X_EGO_COST, V_EGO_COST, A_EGO_COST, a_change_cost, self.JEgoCost]
        constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, self.DangerZoneCost]
    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0
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
      for i in range(0, N+1):
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
      #x_lead = lead.dRel if lead.radar else max(lead.dRel-DIFF_RADAR_VISION, 0.)
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-MIN_ACCEL * 2)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def set_accel_limits(self, min_a, max_a):
    # TODO this sets a max accel limit, but the minimum limit is only for cruise decel
    # needs refactor
    self.cruise_min_a = min_a
    self.max_a = max_a

  def update(self, carstate, radarstate, model, controls, v_cruise, x, v, a, j, y, prev_accel_constraint, light_sensor):

    self.update_params()
    if light_sensor >= 0:
      self.lightSensor = light_sensor
    v_ego = self.x0[1]
    a_ego = carstate.aEgo

    #self.trafficState = 0
    self.debugLongText1 = ""
    self.mySafeModeFactor = clip(controls.mySafeModeFactor, 0.5, 1.0)

    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    self.update_gap_tf(controls, radarstate, v_ego, a_ego)

    self.comfort_brake = COMFORT_BRAKE
    #self.set_weights(prev_accel_constraint=prev_accel_constraint, v_lead0=lead_xv_0[0,1], v_lead1=lead_xv_1[0,1])

    applyStopDistance = self.stopDistance * (2.0 - self.mySafeModeFactor)

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1], self.x_sol[:,1], self.t_follow, self.stopDistance, krkeegan=self.applyLongDynamicCost)
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1], self.x_sol[:,1], self.t_follow, self.stopDistance, krkeegan=self.applyLongDynamicCost)
    self.params[:,0] = MIN_ACCEL
    self.params[:,1] = self.max_a

    v_cruise, stop_x, self.mode = self.update_apilot(controls, carstate, radarstate, model, v_cruise, self.mode)
    self.set_weights(prev_accel_constraint=prev_accel_constraint, v_lead0=lead_xv_0[0,1], v_lead1=lead_xv_1[0,1])

    # Update in ACC mode or ACC/e2e blend
    if self.mode == 'acc':
      self.params[:,5] = self.leadDangerFactor #LEAD_DANGER_FACTOR

      x2 = stop_x * np.ones(N+1) + self.trafficStopDistanceAdjust

      # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
      # when the leads are no factor.
      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)

      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, self.t_follow, self.comfort_brake, applyStopDistance + self.fakeCruiseDistance)
      
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle, x2])

      #self.debugLongText1 = 'A{:.2f},Y{:.1f},TR={:.2f},state={} {},L{:3.1f} C{:3.1f},{:3.1f},{:3.1f} X{:3.1f} S{:3.1f},V={:.1f}:{:.1f}:{:.1f}'.format(
      #  self.prev_a[0], y[-1], self.t_follow, self.xState, self.e2ePaused, lead_0_obstacle[0], cruise_obstacle[0], cruise_obstacle[1], cruise_obstacle[-1],model.position.x[-1], model_x, v_ego*3.6, v[0]*3.6, v[-1]*3.6)
      self.debugLongText1 = "L{},A{:3.2f},L0{:5.1f},C{:5.1f},X{:5.1f},S{:5.1f}".format(self.lightSensor, self.max_a, lead_0_obstacle[0], cruise_obstacle[0], x2[0], self.stopDist)

      self.source = SOURCES[np.argmin(x_obstacles[0])]

      # These are not used in ACC mode
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      self.params[:,5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle,
                                     lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))
      self.debugLongText1 = "A{:3.2f},L0{:5.1f},C{:5.1f},X{:5.1f},F{:5.1f}".format(self.max_a, lead_0_obstacle[0], cruise_target[0], x[0], xforward[0])

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
    self.params[:,4] = self.t_follow
    self.params[:,6] = self.comfort_brake
    self.params[:,7] = applyStopDistance
    

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    # Check if it got within lead comfort range
    # TODO This should be done cleaner
    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], self.t_follow, self.comfort_brake, self.stopDistance)) - self.x_sol[:,0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], self.t_follow, self.comfort_brake, self.stopDistance)) - self.x_sol[:,0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

    self.v_cruise = v_cruise
    self.x_obstacle_min = self.params[:,2]

  def run(self):
    # t0 = sec_since_boot()
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
    # print(f"long_mpc timings: tot {self.solve_time:.2e}, qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e}, integrator {self.time_integrator:.2e}, qp_iter {qp_iter}")
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

    t = sec_since_boot()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()
      # reset = 1
    # print(f"long_mpc timings: total internal {self.solve_time:.2e}, external: {(sec_since_boot() - t0):.2e} qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e} qp_iter {qp_iter}, reset {reset}")


  def update_params(self):
    self.lo_timer += 1
    if self.lo_timer > 200:
      self.lo_timer = 0
      self.XEgoObstacleCost = float(int(Params().get("XEgoObstacleCost", encoding="utf8")))
      self.JEgoCost = float(int(Params().get("JEgoCost", encoding="utf8")))
    elif self.lo_timer == 20:
      self.AChangeCost = float(int(Params().get("AChangeCost", encoding="utf8")))
      self.DangerZoneCost = float(int(Params().get("DangerZoneCost", encoding="utf8")))
    elif self.lo_timer == 40:
      self.leadDangerFactor = float(int(Params().get("LeadDangerFactor", encoding="utf8"))) * 0.01
      self.trafficStopDistanceAdjust = float(int(Params().get("TrafficStopDistanceAdjust", encoding="utf8"))) / 100.
    elif self.lo_timer == 60:
      self.applyLongDynamicCost = Params().get_bool("ApplyLongDynamicCost")
      self.trafficStopAccel = float(int(Params().get("TrafficStopAccel", encoding="utf8"))) / 100.
    elif self.lo_timer == 80:
      self.trafficStopModelSpeed = Params().get_bool("TrafficStopModelSpeed")
      self.trafficStopMode = int(Params().get("TrafficStopMode", encoding="utf8"))
      self.stopDistance = float(int(Params().get("StopDistance", encoding="utf8"))) / 100.
    elif self.lo_timer == 100:
      self.applyDynamicTFollow = float(int(Params().get("ApplyDynamicTFollow", encoding="utf8"))) / 100.
      self.trafficDetectBrightness = int(Params().get("TrafficDetectBrightness", encoding="utf8"))
    elif self.lo_timer == 120:
      self.applyDynamicTFollowApart = float(int(Params().get("ApplyDynamicTFollowApart", encoding="utf8"))) / 100.
      self.applyDynamicTFollowDecel = float(int(Params().get("ApplyDynamicTFollowDecel", encoding="utf8"))) / 100.
    elif self.lo_timer == 140:
      self.tFollowRatio = float(int(Params().get("TFollowRatio", encoding="utf8"))) / 100.     
      self.softHoldMode = int(Params().get("SoftHoldMode", encoding="utf8"))
    elif self.lo_timer == 160:
      self.applyModelDistOrder = int(Params().get("ApplyModelDistOrder", encoding="utf8"))
      self.trafficStopUpdateDist = int(Params().get("TrafficStopUpdateDist", encoding="utf8"))

  def update_gap_tf(self, controls, radarstate, v_ego, a_ego):
    v_ego_kph = v_ego * CV.MS_TO_KPH
    if controls.longCruiseGap >= 4:
      self.applyCruiseGap = interp(v_ego_kph, [0, 45, 60, 100, 120, 140], [1,1,2,2,3,4])
      cruiseGapRatio = interp(self.applyCruiseGap, [1,2,3,4], [1.1, 1.2, 1.3, 1.45])
      self.applyCruiseGap = clip(self.applyCruiseGap, 1, 4)
    else:
      self.applyCruiseGap = float(controls.longCruiseGap)
      cruiseGapRatio = interp(controls.longCruiseGap, [1,2,3], [1.1, 1.3, 1.6])

    self.t_follow = max(0.9, cruiseGapRatio * self.tFollowRatio * (2.0 - self.mySafeModeFactor)) # 0.9아래는 위험하니 적용안함.


    # lead값을 고의로 줄여주면, 빨리 감속, lead값을 늘려주면 빨리가속,
    if radarstate.leadOne.status:
      self.t_follow *= interp(radarstate.leadOne.vRel*3.6, [-100., 0, 100.], [self.applyDynamicTFollow, 1.0, self.applyDynamicTFollowApart])
      #self.t_follow *= interp(self.prev_a[0], [-4, 0], [self.applyDynamicTFollowDecel, 1.0])
      # 선행차감속도* 내차감속도 : 둘다감속이 심하면 더 t_follow를 크게..
      self.t_follow *= interp(radarstate.leadOne.aLeadK, [-4, 0], [self.applyDynamicTFollowDecel, 1.0]) # 선행차의 accel텀은 이미 사용하고 있지만(aLeadK).... 그러나, t_follow에 추가로 적용시험
      self.t_follow *= interp(a_ego, [-4, 0], [self.applyDynamicTFollowDecel, 1.0]) # 내차의 감속도에 추가 적용

      if not self.openpilotLongitudinalControl:
        if v_ego_kph < 0.1:
          self.applyCruiseGap = 1
        else:
          self.applyCruiseGap = int(interp(a_ego, [-1.5, -0.5], [4, self.applyCruiseGap]))
        #elif v_ego_kph < 60:
        #  self.applyCruiseGap = int(interp(radarstate.leadOne.vRel*3.6, [-5.0, -2.0, -1.0], [4, self.applyCruiseGap, 1]))
        #elif v_ego_kph < 120:
        #  self.applyCruiseGap = int(interp(radarstate.leadOne.vRel*3.6, [-10.0, -5.0, -1.0], [4, self.applyCruiseGap, 2]))
        #else:
        #  self.applyCruiseGap = int(interp(radarstate.leadOne.vRel*3.6, [-20.0, -10.0, -1.0], [4, self.applyCruiseGap, 3]))
      
        #elif a_ego < 0.1:
        #  self.applyCruiseGap = int(interp(a_ego, [-2.0, 0.0], [4, self.applyCruiseGap]))
        #else:
        #  self.applyCruiseGap = int(interp(radarstate.leadOne.vRel*3.6, [0, 10.0], [self.applyCruiseGap, 1]))

  def update_stop_dist(self, stop_x):
    stop_x = self.xStopFilter.process(stop_x, median = True)
    stop_x = self.xStopFilter2.process(stop_x)
    return stop_x

  def check_model_stopping(self, carstate, v, v_ego, model_x, y):
    v_ego_kph = v_ego * CV.MS_TO_KPH
    model_v = self.vFilter.process(v[-1])
    startSign = (model_v > 5.0 or model_v > (v[0]+2)) # and model_x > 50.0

    ## 시그널이 10M이상 떨리면... 신호가 잘못된걸로...
    if (self.prev_x - model_x) > 10:
      startSign = False
    self.prev_x = model_x
    if v_ego_kph < 1.0: 
      stopSign = model_x < 20.0 and model_v < 10.0
    elif v_ego_kph < 80.0:
      if self.trafficDetectBrightness < self.lightSensor:
        stopSign = model_x < 110.0 and ((model_v < 3.0) or (model_v < v[0]*0.6)) and abs(y[-1]) < 10.0
      else:
        stopSign = model_x < 130.0 and ((model_v < 3.0) or (model_v < v[0]*0.7)) and abs(y[-1]) < 10.0
    else:
      stopSign = False

    self.stopSignCount = self.stopSignCount + 1 if (stopSign and (model_x > get_safe_obstacle_distance(v_ego, t_follow=0, comfort_brake=COMFORT_BRAKE, stop_distance=-1.0))) else 0
    self.startSignCount = self.startSignCount + 1 if startSign and not stopSign else 0

    if self.stopSignCount * DT_MDL > 0.0 and carstate.rightBlinker == False:
      self.trafficState = 1
    elif self.startSignCount * DT_MDL > 0.5:
      self.trafficState = 2  
    else:
      self.trafficState = 0

# Stopping Dist....
# v[-1] : stop signal 감지... 3m/s이하가 나오면 정지로 감지하자...
# stop신호를 감지하면... 이때 거리를 저장... 목표 정지위치
#   그러나 정지거리보다 가까이에 전방에 차가 있으면 그냥 따라간다.
#   stop signal(v[-1]) 이 또들어오면.... stopdist를 업데이트하자...
# stopdist에 따라서 계속 주행하여 정지한다.
# 이때 (+) 키가 눌리면... 정지 제어를 하지 않고 그대로 주행한다. prepare로 전환...
## 이때 다시 (-) 가눌리면... 정지로 갈까?
##
# 정지완료후
## 신호가 바뀌면 출발한다.
### 단, 낮시간이나 설정에 따라 자동출발여부를 확인한다.
####     혹시나, 감속시에 한번이라도 정지가 출발신호가 수신되었으면, 수동출발하도록 한다.

  def update_apilot(self, controls, carstate, radarstate, model, v_cruise, mode):

    v_ego = carstate.vEgo
    v_ego_kph = v_ego * CV.MS_TO_KPH
    x = model.position.x
    y = model.position.y
    v = model.velocity.x

    self.fakeCruiseDistance = 0.0
    radar_detected = radarstate.leadOne.status & radarstate.leadOne.radar

    ## 모델의 정지거리 필터링
    stop_x = x[self.applyModelDistOrder]
    self.xStop = self.update_stop_dist(stop_x)
    stop_x = self.xStop
    ## 모델의 신호정지 검사
    #self.check_model_stopping(carstate, v, v_ego, self.xStop, y)
    self.check_model_stopping(carstate, v, v_ego, x[-1], y)

    cruiseButtonCounterDiff = controls.cruiseButtonCounter - self.cruiseButtonCounter
    if cruiseButtonCounterDiff != 0:
      self.trafficError = 0

    if self.e2eCruiseCount > 0:
      self.e2eCruiseCount -= 1

    if carstate.gasPressed or carstate.brakePressed or self.longActiveUser <= 0:
      self.mpcEvent = 0

    # cruise_helper에서 깜박이 켜고 신호감지, 브레이크 크루즈ON을 기동하면... 신호오류와 같이, 크루즈버튼으로 출발해야함.
    if self.longActiveUser != controls.longActiveUser:
      if controls.longActiveUser > 10:
        self.trafficError = True

    # SOFT_HOLD: 검사
    if carstate.brakePressed and v_ego < 0.1 and self.softHoldMode > 0:  
      self.softHoldTimer += 1
      if self.softHoldTimer*DT_MDL >= 0.7: 
        self.xState = XState.softHold
        self.mpcEvent = EventName.autoHold
    else:
      self.softHoldTimer = 0

    ## 소프트홀드중
    if self.xState == XState.softHold:
      stop_x = 0.0
      self.trafficError = 0
      if carstate.gasPressed:
        self.xState = XState.e2eCruisePrepare
      elif self.trafficState == 2:
        self.mpcEvent = EventName.trafficSignChanged

      if cruiseButtonCounterDiff > 0:
        if self.trafficState == 1:
          self.xState = XState.e2eStop
        else:
          self.xState = XState.e2eCruise
          self.mpcEvent = EventName.trafficSignGreen
    #고속모드 또는 신호감지 일시정지: 신호정지 사용안함.
    elif controls.myDrivingMode == 4 or self.trafficStopMode==0: 
      if self.status:
        self.xState = XState.lead
      else:
        self.xState = XState.cruise
      self.trafficState = 0
      self.trafficError = False
      stop_x = 1000.0
    ## 신호감속정지중
    elif self.xState == XState.e2eStop:
      if carstate.gasPressed:
        self.xState = XState.e2eCruisePrepare
        stop_x = 1000.0
      elif self.trafficStopMode==2:
        if self.trafficState == 2:
          if v_ego_kph > 30:
             self.xState = XState.e2eCruise
          elif carstate.aEgo > 1.0:
            self.mpcEvent = EventName.trafficSignGreen
      else:
        if v_ego < 0.1:
          if self.trafficDetectBrightness < self.lightSensor:
            self.trafficError = True
            self.mpcEvent = EventName.trafficError
            ## 조도가 높고, 정지중, +키를 누르면 출발!
            if cruiseButtonCounterDiff > 0:
                self.xState = XState.e2eCruisePrepare
                self.e2eCruiseCount = 3 * DT_MDL
                self.mpcEvent = EventName.trafficSignGreen
          if self.trafficState == 2  and (not self.trafficError or (self.trafficError and cruiseButtonCounterDiff > 0)):
              self.xState = XState.e2eCruisePrepare
              self.e2eCruiseCount = 3 * DT_MDL
              self.mpcEvent = EventName.trafficSignGreen
          else:
            if self.trafficState == 2 and self.trafficError:
              self.mpcEvent = EventName.trafficSignChanged
            if self.trafficError and cruiseButtonCounterDiff > 0:
              self.trafficError = False
            elif not self.trafficError and cruiseButtonCounterDiff < 0:
              self.trafficError = True          
            self.stopDist = 0.0
            v_cruise = 0.0
            stop_x = 0.0
        elif radar_detected and (radarstate.leadOne.dRel - stop_x) < 4.0: # 레이더감지, 정지라인보다 선행차가 가까이있다면..  2->4M : 즉 정지선이 앞차보다 4M뒤에 있다면 앞차안봄..
          self.xState = XState.lead
          stop_x = 1000.0
        elif cruiseButtonCounterDiff > 0:
          self.xState = XState.e2eCruisePrepare
          stop_x = 1000.0
        else:
          self.comfort_brake = COMFORT_BRAKE * self.trafficStopAccel
          if controls.longActiveUser > 0 and self.longActiveUser <= 0:  # longActive된경우
            self.stopDist = 2 if self.xStop < 2 else self.xStop
          else:
            if not self.trafficError and self.trafficState == 1 and self.xStop > self.trafficStopUpdateDist:  # 정지조건에만 update함. 20M이상에서만 Update하자. 이후에는 너무 급격히 정지함. 시험..
              self.stopDist = self.xStop
            elif self.trafficState == 2: ## 감속도중 파란불이면 그냥출발
              #self.trafficError = True
              self.xState = XState.e2eCruisePrepare
              stop_x = 1000.0
          self.fakeCruiseDistance = 0 if self.stopDist > 10.0 else 10.0
    ## e2eCruisePrepare 일시정지중
    elif self.xState == XState.e2eCruisePrepare:
      self.mpcEvent = 0
      ## 신호정지 일시정지 해제 검사
      if controls.longActiveUser <= 0:
        self.xState = XState.e2eCruise
      elif (carstate.brakePressed or cruiseButtonCounterDiff < 0) and self.e2eCruiseCount > 0:  ##잘못된 신호출발이 발생했을때....
        self.xState = XState.e2eStop
        self.stopDist = 2.0 # 너무짧아도 리턴전에 계산할것임..
        self.trafficError = True
      elif cruiseButtonCounterDiff != 0: #신호감지무시중 버튼이 눌리면 다시 재개함.
        self.xState = XState.e2eCruise
      elif v_ego_kph < 1.0 and self.trafficState != 2:  ## 출발신호이지만.... 정지신호로 바뀐경우(모델신호 변심) 다시 정지하는걸로..
        self.xState = XState.e2eStop
      elif (v_ego_kph > 30.0 and (stop_x > 60.0 and abs(y[-1])<2.0)):
        self.xState = XState.e2eCruise
      else:
        #self.trafficState = 0
        self.trafficError = False
        stop_x = 1000.0
    ## 신호감지주행중
    else: # e2eCruise, lead, cruise 상태
      self.trafficError = False
      if self.status:
        self.xState = XState.lead
        stop_x = 1000.0
      elif self.trafficState == 1 and not carstate.gasPressed:
        self.xState = XState.e2eStop
        self.mpcEvent = EventName.trafficStopping
      else:
        self.xState = XState.e2eCruise
        if carstate.brakePressed and v_ego_kph < 1.0  and self.softHoldMode > 0:
          self.xState = XState.softHold
      if self.trafficState in [0, 2]: #stop_x > 100.0:
        stop_x = 1000.0

    if self.trafficStopMode == 2:
      mode = 'blended' if self.xState in [XState.e2eStop, XState.e2eCruisePrepare] else 'acc'

    self.comfort_brake *= self.mySafeModeFactor
    self.longActiveUser = controls.longActiveUser
    self.cruiseButtonCounter = controls.cruiseButtonCounter

    self.stopDist -= (v_ego * DT_MDL)
    if self.stopDist < 0:
      self.stopDist = 0.
    elif stop_x == 1000.0:
      self.stopDist = 0.0
    elif self.stopDist > 0:
      stop_dist = v_ego ** 2 / (2.8 * 2) # 2.8m/s^2 으로 감속할경우 필요한 거리.
      self.stopDist = self.stopDist if self.stopDist > stop_dist else stop_dist
      stop_x = 0.0
#    else:
#      self.fakeCruiseDistance += self.stopDist
    return v_cruise, stop_x+self.stopDist, mode

if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
