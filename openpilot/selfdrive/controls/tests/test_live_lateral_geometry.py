"""Real consumer methods/schema; only IPC and the native solver/filter are replaced."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import log
from opendbc.car.lateral_geometry import LateralGeometry, requested_center_to_front_ratio
from openpilot.selfdrive.controls.tests.test_lane_model_speed_planner import make_planner, inputs, update
from openpilot.selfdrive.controls.tests.test_lateral_startup_control import control_fixture


def production_learner(CP, ratio=None):
  class Filter:
    def __init__(self):
      self.rewinds = 0

    def reset_rewind(self):
      self.rewinds += 1

  class Kalman:
    initial_x = np.array([1., 15., 0., 0., 10., 0., 0., 0., 0.])
    P_initial = np.eye(9) * 1e-6

    def __init__(self, *args):
      self.filter = Filter()
      self.init_count = 0
      self.t = 0.0

    def set_globals(self, **values):
      self.globals = values

    def init_state(self, x, covs, filter_time):
      self.init_count += 1
      self.x, self.P = x.copy(), covs.copy()

  def new_message(service):
    event = log.Event.new_message()
    event.init(service)
    return event

  source = Path(__file__).parents[2] / 'locationd/paramsd.py'
  tree = ast.parse(source.read_text(encoding='utf-8'))
  nodes = [ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)]
  nodes += [node for node in tree.body if isinstance(node, ast.Assign) or
            isinstance(node, ast.ClassDef) and node.name == 'VehicleParamsLearner' or
            isinstance(node, ast.FunctionDef) and node.name == 'check_valid_with_hysteresis']
  states = NS(STIFFNESS=slice(0, 1), STEER_RATIO=slice(1, 2), ANGLE_OFFSET=slice(2, 3),
              ANGLE_OFFSET_FAST=slice(3, 4), YAW_RATE=slice(6, 7), ROAD_ROLL=slice(8, 9))
  namespace = {'np': np, 'math': math, 'DT_MDL': 0.05, 'CarKalman': Kalman, 'States': states,
               'GENERATED_DIR': '', 'LateralGeometry': LateralGeometry, 'PoseCalibrator': lambda: NS(),
               'messaging': NS(new_message=new_message), 'log': log,
               'cloudlog': NS(error=lambda *args: None)}
  exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(source), 'exec'), namespace)
  return namespace['VehicleParamsLearner'](CP, 17.0, 0.8, 0.01, geometry_ratio=ratio)


def test_live_edit_and_zero_restore_reach_filter_controller_and_planner_without_restart():
  control, step_control = control_fixture()
  cp = control.CP
  learner = production_learner(cp)
  planner = make_planner(cp)
  original_state, original_cov = learner.kf.x.copy(), learner.kf.P.copy()
  original_vm = control.VM
  original_mpc = planner.lat_mpc
  values = {'CenterToFrontRatio': 60}
  params = NS(get_int=lambda key: values[key])
  old_ratio = learner.geometry.ratio
  for value in (60, 0):
    values['CenterToFrontRatio'] = value
    target = requested_center_to_front_ratio(params, cp)
    for _ in range(45):
      learner.update_geometry(target, 0.05)
      assert abs(learner.geometry.ratio - old_ratio) <= 0.0050001
      old_ratio = learner.geometry.ratio
      # Use the serialized producer message, including the new additive field.
      event = learner.get_msg(True)
      with log.Event.from_bytes(event.to_bytes()) as received:
        lp = received.liveParameters
        assert lp.valid
        control.sm['liveParameters'] = lp
        step_control()
        sm = inputs(speed=10, start=10, end=10)
        sm['liveParameters'] = lp
        update(planner, sm)
        assert control.VM.geometry.ratio == pytest.approx(lp.centerToFrontRatio)
        assert planner.geometry.ratio == pytest.approx(lp.centerToFrontRatio)
        expected = max(cp.wheelbase * (1 - lp.centerToFrontRatio) - planner.factor2 * 100, 0)
        assert planner.lat_mpc.input_geometry[0] == pytest.approx(expected)
      assert learner.kf.globals['center_to_front'] == pytest.approx(learner.geometry.center_to_front)
      assert learner.kf.globals['stiffness_front'] == pytest.approx(control.VM.cF_orig)
    assert learner.geometry.ratio == pytest.approx(target)
  assert learner.kf.init_count == 1
  np.testing.assert_array_equal(learner.kf.x, original_state)
  np.testing.assert_array_equal(learner.kf.P, original_cov)
  assert learner.kf.filter.rewinds > 0
  assert control.VM is original_vm and planner.lat_mpc is original_mpc
  assert control.VM.aF == pytest.approx(cp.centerToFront)
  assert control.VM.cF_orig == pytest.approx(cp.tireStiffnessFront)


def test_startup_override_is_published_without_a_startup_ramp():
  control, _ = control_fixture()
  learner = production_learner(control.CP, 0.6)
  assert learner.get_msg(True).liveParameters.centerToFrontRatio == pytest.approx(0.6)
  assert learner.kf.globals['center_to_front'] == pytest.approx(control.CP.wheelbase * 0.6)


@pytest.mark.parametrize('dt', [float('nan'), float('inf'), -1, 0])
def test_invalid_timing_cannot_change_applied_geometry(dt):
  control, _ = control_fixture()
  learner = production_learner(control.CP)
  ratio = learner.geometry.ratio
  learner.update_geometry(0.6, dt)
  assert learner.geometry.ratio == ratio
  assert learner.kf.filter.rewinds == 0


def test_delayed_cycle_does_not_catch_up_with_a_large_geometry_jump():
  control, _ = control_fixture()
  learner = production_learner(control.CP)
  ratio = learner.geometry.ratio
  learner.update_geometry(0.6, 10)
  assert learner.geometry.ratio == pytest.approx(ratio + 0.005)
