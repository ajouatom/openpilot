"""Baseline stop-intent timing and same-cycle publication."""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan


@pytest.mark.parametrize('threshold', [.07, .5])
def test_stop_waits_for_normal_action_horizon(threshold):
  # The removed preview used to request a stop on this approaching trajectory.
  times = np.array([0., .25, .5, 1., 1.25, 1.5, 2.25, 2.5])
  speeds = np.array([.6, .55, .34, .08, 0., 0., 0., 0.])
  accels = np.array([-.5, -.5, -.5, -.2, 0., 0., 0., 0.])
  _, should_stop, _, _ = get_accel_from_plan(speeds, accels, times, action_t=.25, vEgoStopping=threshold)
  assert not should_stop
  _, should_stop, _, _ = get_accel_from_plan(speeds, accels, times, action_t=1.25, vEgoStopping=threshold)
  assert should_stop


@pytest.mark.parametrize('mode', ['acc', 'blended'])
@pytest.mark.parametrize('mpc_stop', [False, True])
@pytest.mark.parametrize('model_stop', [False, True])
def test_planner_uses_current_mpc_or_model_stop_without_early_override(mode, mpc_stop, model_stop):
  path = Path(__file__).resolve().parents[1] / 'lib/longitudinal_planner.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  update = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'update')
  start = next(i for i, n in enumerate(update.body) if isinstance(n, ast.If)
               and any(isinstance(x, ast.Assign) and any(ast.unparse(t) == 'self.output_should_stop' for t in x.targets)
                       for x in n.body))
  state = SimpleNamespace(mpc=SimpleNamespace(mode=mode))
  ns = {'self': state, 'output_should_stop_mpc': mpc_stop, 'output_should_stop_e2e': model_stop,
        'output_a_target_mpc': -.3, 'output_a_target_e2e': -.5, 'output_a_target_base': -.3,
        'output_v_target_mpc': .6, 'output_v_target_now_e2e': .4}
  state.j_desired_trajectory = [0.]
  exec(compile(ast.Module(body=update.body[start:], type_ignores=[]), str(path), 'exec'), ns)
  assert state.output_should_stop == (mpc_stop if mode == 'acc' else mpc_stop or model_stop)


def test_controlsd_publishes_new_longitudinal_state_in_same_cycle():
  path = Path(__file__).resolve().parents[1] / 'controlsd.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                and any(isinstance(x, ast.Assign) and isinstance(x.value, ast.Call)
                        and ast.unparse(x.value.func) == 'self.LoC.update' for x in n.body))
  start = next(i for i, n in enumerate(method.body) if isinstance(n, ast.Assign)
               and isinstance(n.value, ast.Call) and ast.unparse(n.value.func) == 'self.LoC.update')
  end = next(i for i in range(start, len(method.body)) if isinstance(method.body[i], ast.Assign)
             and any(ast.unparse(t) == 'actuators.accel' for t in method.body[i].targets))
  control = SimpleNamespace(long_control_state='pid')
  def update(*args):
    control.long_control_state = 'stopping'
    return -.5, -.5, 0.
  control.update = update
  actuators = SimpleNamespace(longControlState='pid')
  ns = {'self': SimpleNamespace(LoC=control, sm={'radarState': None}), 'CC': SimpleNamespace(longActive=True),
        'CS': None, 'long_plan': None, 'pid_accel_limits': None, 't_since_plan': 0., 'actuators': actuators}
  exec(compile(ast.Module(body=method.body[start:end+1], type_ignores=[]), str(path), 'exec'), ns)
  assert actuators.longControlState == 'stopping'
  assert actuators.accel == -.5
