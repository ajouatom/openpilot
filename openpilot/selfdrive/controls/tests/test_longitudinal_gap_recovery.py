import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_gap_recovery import LeadGapState, gap_reference
from openpilot.selfdrive.controls.lib.longitudinal_safe_follow import SafeFollowState


def load_mpc_update(path):
  """Run production wiring with a recording solver; this is not a solver test."""
  from openpilot.selfdrive.controls.lib import longitudinal_preview as preview
  from openpilot.selfdrive.carrot.traffic_stop import get_traffic_stop_distance_adjust, get_traffic_stop_obstacle_distance
  from openpilot.selfdrive.carrot.radar_motion.lane_change_gap import LaneChangeGapPlan
  from openpilot.selfdrive.controls.lib.longitudinal_cutout import cutout_obstacle_relief

  class RecordingSolver:
    def __init__(self, *args): self.weights = {}
    def reset(self): pass
    def set(self, *args): pass
    def cost_set(self, i, key, value): self.weights[i, key] = np.copy(value)
    def constraints_set(self, *args): pass

  ns = {'np': np, 'log': SimpleNamespace(LongitudinalPersonality=SimpleNamespace(standard=1, aggressive=0)),
            'DT_MDL': .05, 'N': 12, 'COMFORT_BRAKE': 2.5, 'STOP_DISTANCE': 6., 'ACCEL_MIN': -3.5, 'LEAD_ACCEL_TAU': 1.5,
            'MODEL_NAME': 'long', 'ACADOS_SOLVER_TYPE': 'SQP_RTI', 'COST_DIM': 6, 'COST_E_DIM': 5, 'X_DIM': 3, 'PARAM_DIM': 8,
            'A_CHANGE_COST': 200., 'A_CHANGE_COST_STARTING': 10., 'LEAD_DANGER_FACTOR': .8, 'LIMIT_COST': 1e6,
            'DANGER_ZONE_COST': 100., 'CRASH_DISTANCE': .25, 'X_EGO_OBSTACLE_COST': 5., 'X_EGO_COST': 0.,
            'V_EGO_COST': 0., 'A_EGO_COST': 0., 'J_EGO_COST': 5., 'SOURCES': ['lead0','lead1','cruise','e2e'],
            'AcadosOcpSolverCython': RecordingSolver, 'LEAD_ACCEL_MIN_TRACK_FRAMES': 3,
            'LeadAccelResponseState': preview.LeadAccelResponseState, 'get_lead_accel_mpc_request': preview.get_lead_accel_mpc_request,
            'LeadGapState': LeadGapState, 'gap_reference': gap_reference,
            'SafeFollowState': SafeFollowState,
            'get_traffic_stop_distance_adjust': get_traffic_stop_distance_adjust,
            'get_traffic_stop_obstacle_distance': get_traffic_stop_obstacle_distance,
            'LaneChangeGapPlan': LaneChangeGapPlan, 'cutout_obstacle_relief': cutout_obstacle_relief}
  ns['T_IDXS'] = 10*(np.arange(13)/12)**2
  ns['T_DIFFS'] = np.diff(ns['T_IDXS'], prepend=0.)
  ns['FCW_IDXS'] = ns['T_IDXS'] < 5
  ns['PRED_DANGER_IDXS'] = (ns['T_IDXS'] > .2) & (ns['T_IDXS'] < 3.)
  tree = ast.parse(path.read_text(encoding='utf-8'))
  nodes = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and not node.name.startswith('gen_')]
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), ns)
  cls = ns['LongitudinalMpc']
  cls.run = lambda self: None
  return cls


def run_update(cls, level=1, *, distance=45.15, speed=79.04/3.6, lead_speed=None,
               frames=10, status=True, radar=True, enabled=True, reset=False, lane_change=False, mode='acc', lead_index=0,
               lead_accel=0., driving_mode=3, ego_accel=0., gap_enabled=True, relative_speed=None):
  if lead_speed is None:
    lead_speed = speed-1.96
  lead = SimpleNamespace(status=status, radar=radar, radarTrackId=49, dRel=distance, vLead=lead_speed,
                         vRel=lead_speed-speed if relative_speed is None else relative_speed,
                         aLeadK=lead_accel, aLeadTau=1.5, modelProb=1.)
  absent = SimpleNamespace(status=False, radar=False, radarTrackId=-1, vRel=0., aLeadK=0., modelProb=0.)
  rs = SimpleNamespace(leadOne=lead if lead_index == 0 else absent, leadTwo=lead if lead_index == 1 else absent)
  carrot = SimpleNamespace(leadAccelResponse=level, myDrivingMode=driving_mode, jerk_factor=1., comfort_brake=2.4, stop_distance=6.,
                           mode=mode, v_cruise=30., stop_dist=1000., trafficStopDistanceAdjust=0., lane_change_active=lane_change,
                           get_T_FOLLOW=lambda *args, **kwargs: .65)
  mpc = cls(mode=mode)
  mpc.set_cur_state(speed, ego_accel)
  mpc.set_accel_limits(-1.2, 2.)
  for _ in range(20):
    mpc.update(carrot, reset, rs, 30., *(np.zeros(13) for _ in range(4)), lead_accel_response_enabled=enabled, lead_gap_enabled=gap_enabled and enabled,
               lead_track_frames=(frames, frames))
  return mpc


@pytest.fixture
def mpc_class():
  return load_mpc_update(Path(__file__).resolve().parents[1]/'lib/longitudinal_mpc_lib/long_mpc.py')


def step(state, **kwargs):
  args = {'level': 0, 'track_id': 49, 'enabled': True, 'dt': .05, 'ego_speed': 10., 'lead_speed': 10.,
          'distance': 40., 'desired_distance': 20., 'base_tf': .6}
  args.update(kwargs)
  args.setdefault('relative_speed', args['lead_speed'] - args['ego_speed'])
  return state.update(**args)


def test_half_of_excess_is_captured_once_not_replenished_by_a_large_gap():
  state = LeadGapState()
  step(state)
  assert state.extra_tf == pytest.approx(1.)
  for _ in range(100):
    step(state, distance=60.)
  assert .36 < state.extra_tf < .38
  for _ in range(1000):
    step(state, distance=60. + .1*np.sin(_))
  assert state.extra_tf < .0001


def test_matched_gap_adds_nothing_and_existing_long_tf_is_not_shortened():
  state = LeadGapState()
  assert step(state, distance=20.) == 0.
  assert step(state, track_id=50, base_tf=3.) == 0.


@pytest.mark.parametrize('speed', [0., .01, .2, 1., 30.])
def test_low_speed_capture_is_finite_and_capped(speed):
  state = LeadGapState()
  step(state, ego_speed=speed, lead_speed=0., distance=1000.)
  assert np.isfinite(state.extra_tf)
  assert state.extra_tf == pytest.approx(1.9)


def test_stopped_lead_holds_tf_but_stopping_ego_loses_distance_margin():
  state = LeadGapState()
  step(state, lead_speed=0.)
  initial = state.extra_tf
  for _ in range(500):
    step(state, ego_speed=max(0., 10.-_*0.05), lead_speed=0.)
  assert state.extra_tf == initial
  times, speeds = np.array([0., 1., 2., 3.]), np.array([10., 5., 1., 0.])
  margin = state.margins(level=0, times=times, ego_speeds=speeds, lead_speeds=np.zeros(4), base_tf=.6)
  np.testing.assert_allclose(margin, speeds*initial)
  assert margin[-1] == 0.


def test_slow_lead_recovers_more_slowly_and_tiers_are_ordered():
  values = []
  for level in range(6):
    state = LeadGapState()
    for _ in range(101):
      step(state, level=level)
    values.append(state.extra_tf)
  assert values[0] > values[1] > values[2] > values[3] > values[4] > values[5] == 0.
  state = LeadGapState()
  for _ in range(101):
    step(state, lead_speed=1.)
  assert values[0] < state.extra_tf < 1.


def test_filter_respects_elapsed_time_at_different_planner_rates():
  values = []
  for dt in (.02, .05, .1):
    state = LeadGapState()
    step(state, dt=dt)
    for _ in range(round(5./dt)):
      step(state, dt=dt)
    values.append(state.extra_tf)
  assert max(values)-min(values) < .004


def test_opening_recaptures_after_departure_and_later_reacceleration():
  state = LeadGapState()
  step(state, ego_speed=0., lead_speed=0., distance=6., desired_distance=6.)
  for _ in range(40):
    step(state, ego_speed=1., lead_speed=2., distance=8., desired_distance=6.)
  captured = state.extra_tf
  assert captured > .8
  for _ in range(100):
    step(state, ego_speed=10., lead_speed=10., distance=100.)
  assert state.extra_tf < captured
  before = state.extra_tf
  for _ in range(40):
    step(state, ego_speed=10., lead_speed=12., distance=100.)
  assert state.extra_tf > before + .8


def test_opening_does_not_hold_a_large_old_tf_when_candidate_has_fallen():
  state = LeadGapState()
  step(state)
  initial = state.extra_tf
  for _ in range(100):
    step(state, relative_speed=1., lead_speed=11., distance=20.)
  assert state.extra_tf == pytest.approx(initial / 1.01**100)
  times = np.array([0., 1., 2., 3.])
  margins = state.margins(level=0, times=times, ego_speeds=np.full(4, 10.), lead_speeds=np.full(4, 11.), base_tf=.6)
  assert np.all(np.diff(margins) < 0.)


def test_relative_speed_noise_and_single_spike_do_not_replenish():
  state = LeadGapState()
  step(state, distance=20.)
  for i in range(200):
    step(state, distance=60., relative_speed=.06*np.sin(i))
  assert state.extra_tf == 0.
  step(state, distance=60., relative_speed=.4)
  assert state.extra_tf == 0.
  for _ in range(20):
    step(state, distance=60., relative_speed=1.)
  assert 0.3 < state.extra_tf <= .5


def test_capture_rise_is_limited_and_a_track_change_clears_opening_filter():
  state = LeadGapState()
  step(state, distance=20.)
  for _ in range(80):
    before = state.extra_tf
    step(state, distance=1000., relative_speed=2.)
    assert state.extra_tf - before <= .025 + 1e-10
  assert state.extra_tf > 1.5
  step(state, track_id=50, distance=20., relative_speed=-2.)
  assert state.extra_tf == 0.
  assert state.filtered_relative_speed == -2.
  for _ in range(30):
    step(state, track_id=50, distance=100., relative_speed=-2.)
  assert state.extra_tf == 0.


@pytest.mark.parametrize('kwargs', [{'enabled': False}, {'track_id': -1}, {'distance': np.nan},
                                   {'relative_speed': np.inf}, {'dt': 0.}, {'level': 5}])
def test_invalidation_clears_headroom(kwargs):
  state = LeadGapState()
  step(state)
  assert state.extra_tf > 0.
  assert step(state, **kwargs) == 0.
  assert state.key is None and state.extra_tf == 0.


def test_new_track_and_level_do_not_inherit_previous_extra_tf():
  state = LeadGapState()
  step(state, distance=100.)
  assert state.extra_tf > 1.
  assert step(state, track_id=50, distance=20.) == 0.
  assert step(state, level=1, distance=20.) == 0.


def test_reference_preserves_physical_obstacles_and_obeys_closer_cruise_target():
  obstacles = np.array([[100., 200., 110., 1000.], [100., 200., 80., 1000.]])
  saved = obstacles.copy()
  reference = gap_reference(obstacles, np.array([[6., 0.], [6., 0.]]), np.array([20., 20.]))
  np.testing.assert_array_equal(obstacles, saved)
  np.testing.assert_allclose(reference, [.2, 0.])


@pytest.mark.parametrize('lead_index', [0, 1])
def test_mpc_headroom_only_changes_comfort_reference(mpc_class, lead_index):
  args = {'level': 0, 'lead_index': lead_index}
  base = run_update(mpc_class, gap_enabled=False, **args)
  changed = run_update(mpc_class, **args)
  np.testing.assert_array_equal(base.params, changed.params)
  np.testing.assert_array_equal(base.yref[:,1:], changed.yref[:,1:])
  assert base.source == changed.source
  assert np.any(changed.yref[:,0] > 0.)
  assert changed.lead_gap_margins[0,lead_index] > 0.


@pytest.mark.parametrize('lead_index', [0, 1])
def test_mpc_capture_uses_measured_relative_speed_not_planned_ego_speed(mpc_class, lead_index):
  # Planned ego speed can exceed measured ego speed during departure. The
  # supplied radar vRel still says that the physical gap is opening.
  mpc = run_update(mpc_class, 0, speed=12., lead_speed=11., relative_speed=.8, lead_index=lead_index)
  assert mpc.lead_gap_states[lead_index].filtered_relative_speed == pytest.approx(.8)


@pytest.mark.parametrize('kwargs', [{'level': 5}, {'frames': 2}, {'status': False}, {'radar': False},
                                   {'enabled': False}, {'reset': True}, {'lane_change': True}, {'mode': 'blended'},
                                   {'speed': 50/3.6, 'lead_speed': 0., 'distance': 50.}])
def test_mpc_bypasses_or_has_no_excess_for_close_stationary_acquisition(mpc_class, kwargs):
  mpc = run_update(mpc_class, **kwargs)
  assert np.all(mpc.yref[:,0] == 0.)


@pytest.mark.parametrize('mode', [1, 2, 3, 4])
@pytest.mark.parametrize('lead_accel', [-4., 0., 2.])
def test_level_five_exact_wiring_parity(mpc_class, mode, lead_accel):
  args = {'level': 5, 'driving_mode': mode, 'lead_accel': lead_accel, 'ego_accel': 1.5}
  before = run_update(mpc_class, gap_enabled=False, **args)
  after = run_update(mpc_class, gap_enabled=True, **args)
  np.testing.assert_array_equal(before.params, after.params)
  np.testing.assert_array_equal(before.yref, after.yref)
  assert before.source == after.source
  for key in before.solver.weights:
    np.testing.assert_array_equal(before.solver.weights[key], after.solver.weights[key])


def test_mpc_reset_clears_gap_state(mpc_class):
  mpc = run_update(mpc_class, 0)
  assert np.any(mpc.lead_gap_margins > 0.)
  mpc.reset()
  assert np.all(mpc.lead_gap_margins == 0.)
  assert all(state.key is None for state in mpc.lead_gap_states)


@pytest.mark.parametrize('gas,reset,force,lane,expected', [(False, False, False, False, True),
  (True, False, False, False, False), (False, True, False, False, False),
  (False, False, True, False, False), (False, False, False, True, False)])
def test_planner_gap_gate_allows_zero_level_and_stopping_but_not_override(gas, reset, force, lane, expected):
  path = Path(__file__).resolve().parents[1]/'lib/longitudinal_planner.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  expression = next(node.value for node in ast.walk(tree) if isinstance(node, ast.keyword) and node.arg == 'lead_gap_enabled')
  assert eval(compile(ast.Expression(expression), str(path), 'eval'), {
    'reset_state': reset, 'sm': {'carState': SimpleNamespace(gasPressed=gas)}, 'force_slow_decel': force,
    'carrot': SimpleNamespace(leadAccelResponse=0, lane_change_active=lane),
    'self': SimpleNamespace(output_should_stop=True)}) == expected
