import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_approach import APPROACH_TUNING, LeadApproachState, approach_margin, approach_reference
from openpilot.selfdrive.controls.lib.longitudinal_safe_follow import SafeFollowState


@pytest.mark.parametrize('level', range(6))
def test_stationary_car_first_seen_at_50m_has_no_extra_gap(level):
  # Current urgency also suppresses later nodes where the old plan slows down.
  assert np.all(approach_margin(level, [50/3.6, 5.0], [0., 0.], [50., 30.], 6.) == 0.)


def test_earlier_stationary_acquisition_and_tier_order():
  margins = [float(approach_margin(level, 50/3.6, 0., 100., 6.)) for level in range(1, 6)]
  assert margins[0] > margins[1] > margins[2] > margins[3] > margins[4] == 0.
  assert margins[0] <= 12.


@pytest.mark.parametrize('level', range(1, 5))
def test_no_preference_when_matching_or_opening(level):
  assert np.all(approach_margin(level, [20.]*3, [20., 20.2, 25.], [40.]*3, 6.) == 0.)


def test_ioniq_approach_at_32_seconds():
  extra = approach_margin(1, 79.04/3.6, 79.04/3.6-1.96, 45.15, 6.)
  assert 3.59 < extra < 5.9


@pytest.mark.parametrize('level', range(1, 5))
def test_bounds_and_non_accumulation(level):
  rng = np.random.default_rng(8)
  for _ in range(100):
    ego, lead, distance = rng.uniform(0, 45, 13), rng.uniform(0, 45, 13), rng.uniform(0, 160, 13)
    actual = approach_margin(level, ego, lead, distance, 6.)
    assert np.all(np.isfinite(actual)) and np.all(actual >= 0.)
    assert np.all(actual <= APPROACH_TUNING[level][1])
    assert np.all(actual <= .25*np.maximum(distance-6, 0))
    np.testing.assert_array_equal(actual, approach_margin(level, ego, lead, distance, 6.))


def test_bad_inputs_do_not_create_preferences():
  assert np.all(approach_margin(1, [np.nan, 20, 20, -1], [0, np.inf, 0, 0], [50, 50, np.nan, 50], 6) == 0)
  assert approach_margin(1, 20, 0, 50, np.nan) == 0


def test_acquisition_loss_level_and_track_changes():
  state = LeadApproachState()
  assert state.update(1, 49, True, .05) == pytest.approx(.0625)
  for _ in range(20):
    state.update(1, 49, True, .05)
  assert state.strength == 1.
  assert state.update(1, 50, True, .05) == pytest.approx(.0625)
  assert state.update(2, 50, True, .05) == pytest.approx(.0625)
  assert state.update(2, 50, False, .05) == 0
  assert state.update(5, 50, True, .05) == 0


def test_reference_sign_physical_obstacles_and_cruise_masking():
  obstacles = np.array([[100., 200., 110., 1000.], [100., 200., 80., 1000.]])
  saved = obstacles.copy()
  reference = approach_reference(obstacles, np.array([[6., 0.], [6., 0.]]), np.array([20., 20.]))
  np.testing.assert_array_equal(obstacles, saved)
  np.testing.assert_allclose(reference, [.2, 0.])
  assert (100 - 0 - 94) / 30 - reference[0] == pytest.approx(0.)


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
            'LeadApproachState': LeadApproachState, 'approach_margin': approach_margin, 'approach_reference': approach_reference,
            'SafeFollowState': SafeFollowState,
            'CLOSING_DEADBAND': .2, 'get_traffic_stop_distance_adjust': get_traffic_stop_distance_adjust,
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
               lead_accel=0., driving_mode=3, ego_accel=0.):
  if lead_speed is None:
    lead_speed = speed-1.96
  lead = SimpleNamespace(status=status, radar=radar, radarTrackId=49, dRel=distance, vLead=lead_speed,
                         vRel=lead_speed-speed, aLeadK=lead_accel, aLeadTau=1.5, modelProb=1.)
  absent = SimpleNamespace(status=False, radar=False, radarTrackId=-1, vRel=0., aLeadK=0., modelProb=0.)
  rs = SimpleNamespace(leadOne=lead if lead_index == 0 else absent, leadTwo=lead if lead_index == 1 else absent)
  carrot = SimpleNamespace(leadAccelResponse=level, myDrivingMode=driving_mode, jerk_factor=1., comfort_brake=2.4, stop_distance=6.,
                           mode=mode, v_cruise=30., stop_dist=1000., trafficStopDistanceAdjust=0., lane_change_active=lane_change,
                           get_T_FOLLOW=lambda *args, **kwargs: .65)
  mpc = cls(mode=mode)
  mpc.set_cur_state(speed, ego_accel)
  mpc.set_accel_limits(-1.2, 2.)
  for _ in range(20):
    mpc.update(carrot, reset, rs, 30., *(np.zeros(13) for _ in range(4)), lead_accel_response_enabled=enabled,
               lead_track_frames=(frames, frames))
  return mpc


@pytest.fixture
def mpc_class():
  return load_mpc_update(Path(__file__).resolve().parents[1]/'lib/longitudinal_mpc_lib/long_mpc.py')


@pytest.mark.parametrize('lead_index', [0, 1])
def test_mpc_changes_only_distance_reference_for_closing_lead(mpc_class, lead_index):
  base = run_update(mpc_class, 0, lead_index=lead_index)
  mild = run_update(mpc_class, 1, lead_index=lead_index)
  np.testing.assert_array_equal(base.params, mild.params)
  np.testing.assert_array_equal(base.yref[:,1:], mild.yref[:,1:])
  assert base.source == mild.source
  assert np.any(mild.yref[:,0] > 0)
  assert mild.lead_approach_margins[0,lead_index] > 0


@pytest.mark.parametrize('kwargs', [{'level': 0}, {'level': 5}, {'frames': 2}, {'status': False}, {'radar': False},
                                   {'enabled': False}, {'reset': True}, {'lane_change': True}, {'mode': 'blended'},
                                   {'speed': 50/3.6, 'lead_speed': 0, 'distance': 50}, {'lead_speed': 25.}])
def test_mpc_inhibits_and_emergency_keep_original_reference(mpc_class, kwargs):
  mpc = run_update(mpc_class, **kwargs)
  assert np.all(mpc.yref[:,0] == 0.)


def test_mpc_reset_clears_approach_state(mpc_class):
  mpc = run_update(mpc_class)
  assert np.any(mpc.lead_approach_margins > 0)
  mpc.reset()
  assert np.all(mpc.lead_approach_margins == 0)
  assert all(state.strength == 0 for state in mpc.lead_approach_states)


@pytest.mark.parametrize('level', range(1, 6))
def test_hard_lead_braking_retains_original_obstacles_and_limits(mpc_class, level):
  base = run_update(mpc_class, 0, lead_accel=-4.)
  changed = run_update(mpc_class, level, lead_accel=-4.)
  np.testing.assert_array_equal(base.params, changed.params)
  assert np.all(changed.yref[:,0] >= base.yref[:,0])
  assert changed.predicted_danger_margin == base.predicted_danger_margin
