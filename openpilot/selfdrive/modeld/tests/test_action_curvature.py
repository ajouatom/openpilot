import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants, Plan


@pytest.fixture(scope='module')
def action_functions():
  # Execute the production numerical functions without initializing GPU or IPC.
  modeld = Path(__file__).parents[1] / 'modeld.py'
  helpers = modeld.parents[1] / 'controls/lib/drive_helpers.py'
  namespace = {
    'np': np, 'ModelConstants': ModelConstants, 'Plan': Plan,
    'log': SimpleNamespace(ModelDataV2=SimpleNamespace(Action=SimpleNamespace)),
    'DT_MDL': .05,
  }
  names = {'get_action_from_model', 'get_accel_from_plan', 'get_curvature_from_plan',
           'curv_from_psis', 'smooth_value', 'MIN_SPEED', 'LONG_SMOOTH_SECONDS', 'MIN_LAT_CONTROL_SPEED'}
  for source in (helpers, modeld):
    tree = ast.parse(source.read_text(encoding='utf8'))
    body = [node for node in tree.body if
            (isinstance(node, ast.FunctionDef) and node.name in names) or
            (isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in node.targets))]
    exec(compile(ast.Module(body=body, type_ignores=[]), str(source), 'exec'), namespace)
  return SimpleNamespace(**namespace)


def run_action(functions, speed, lateral_accel, smoothing=0.):
  plan = np.zeros((1, ModelConstants.IDX_N, ModelConstants.PLAN_WIDTH))
  plan[0, :, Plan.VELOCITY][:, 0] = speed
  plan[0, :, Plan.T_FROM_CURRENT_EULER][:, 2] = np.asarray(ModelConstants.T_IDXS) * .02
  plan[0, :, Plan.ORIENTATION_RATE][:, 2] = .02
  output = {'plan': plan}
  if lateral_accel is not None:
    output['action'] = np.array([[lateral_accel, -.2]])
  previous = SimpleNamespace(desiredCurvature=.007, desiredAcceleration=.1, desiredVelocity=2.)
  return functions.get_action_from_model(output, previous, .3, .4, speed, smoothing, .05)


@pytest.mark.parametrize('speed', [.301, .5, 1., 1.5, 2., 2.499, 2.5, 3., 10., 30.])
@pytest.mark.parametrize('lateral_accel', [-.1, 0., .1])
@pytest.mark.parametrize('smoothing', [0., .3])
def test_action_curvature_speed_floor(action_functions, speed, lateral_accel, smoothing):
  result = run_action(action_functions, speed, lateral_accel, smoothing)
  target = lateral_accel / max(2.5, speed)**2
  expected = action_functions.smooth_value(target, .007, smoothing)
  assert result.desiredCurvature == pytest.approx(expected)
  assert result.desiredAcceleration == pytest.approx(action_functions.smooth_value(-.2, .1, .3))
  assert result.desiredVelocity == pytest.approx(action_functions.smooth_value(speed, 2., .3))
  assert result.shouldStop is False


@pytest.mark.parametrize('speed', [0., .1, .3])
@pytest.mark.parametrize('lateral_accel', [None, -.1, .1])
def test_standstill_retains_previous_curvature(action_functions, speed, lateral_accel):
  result = run_action(action_functions, speed, lateral_accel)
  assert result.desiredCurvature == .007
  assert result.shouldStop == (speed < .05)


@pytest.mark.parametrize('speed', [.301, .5, 1., 2., 2.5, 10.])
def test_plan_only_model_keeps_existing_conversion(action_functions, speed):
  result = run_action(action_functions, speed, None)
  assert result.desiredCurvature == pytest.approx(.02 / max(1., speed))
  assert result.desiredAcceleration == pytest.approx(action_functions.smooth_value(0., .1, .3))


def test_curvature_is_continuous_at_floor(action_functions):
  before = run_action(action_functions, 2.5 - 1e-8, .1).desiredCurvature
  after = run_action(action_functions, 2.5 + 1e-8, .1).desiredCurvature
  assert abs(after - before) < 1e-9
