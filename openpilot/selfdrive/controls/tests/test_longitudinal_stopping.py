"""Stop-intent timing and publication, without a native MPC/vehicle simulation."""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.selfdrive.controls.lib.longitudinal_stopping import should_prepare_stop


TIMES = np.array([0., .25, .5, 1., 1.25, 1.5, 2.25, 2.5])
SPEEDS = np.array([.6, .48, .34, .08, 0., 0., 0., 0.])
ACCELS = np.array([-.5, -.5, -.5, -.2, 0., 0., 0., 0.])


def prepare(speeds=SPEEDS, accels=ACCELS, times=TIMES, **kwargs):
  return should_prepare_stop(speeds, accels, times, **dict(v_ego=.6, action_t=.25, v_stop=.07, **kwargs))


def test_stop_is_prepared_while_near_term_speed_still_exceeds_stop_threshold():
  assert SPEEDS[1] > .07
  assert prepare()


@pytest.mark.parametrize('speeds,accels', [
  (np.full(8, .3), np.zeros(8)),  # ordinary crawl
  (np.linspace(0., 1., 8), np.full(8, .4)),  # launch
  (np.array([.6, .5, .4, .2, .1, .1, .1, .1]), ACCELS),  # will keep creeping
  (np.array([.6, .48, .34, .08, 0., .2, 0., 0.]), ACCELS),  # intervening restart
])
def test_moving_or_restarting_plan_does_not_prepare(speeds, accels):
  assert not prepare(speeds, accels)


@pytest.mark.parametrize('speed', [.7001, 3., -0.1, float('nan'), float('inf')])
def test_early_stop_is_limited_to_valid_low_speed(speed):
  assert not should_prepare_stop(SPEEDS, ACCELS, TIMES, v_ego=speed, action_t=.25, v_stop=.07)


def test_early_stop_needs_valid_sufficient_prediction_horizon():
  assert not prepare(SPEEDS[:-1])
  assert not prepare(accels=[float('nan')]*8)
  assert not prepare(times=np.zeros(8))
  assert not prepare(times=TIMES / 3.)
  assert not should_prepare_stop(SPEEDS, ACCELS, TIMES, v_ego=.6, action_t=.25, v_stop=0.)


@pytest.fixture
def planner_early_stop():
  path = Path(__file__).resolve().parents[1] / 'lib/longitudinal_planner.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalPlanner')
  update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update')
  start = next(i for i, n in enumerate(update.body) if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == 'early_stop_enabled' for t in n.targets))
  code = compile(ast.Module(body=update.body[start:start+2], type_ignores=[]), str(path), 'exec')

  def run(*, setting=True, brand='hyundai', flags=HyundaiFlags.CANFD, longitudinal=True, mode='acc',
          reset=False, original=False, **overrides):
    cs = SimpleNamespace(**({'gasPressed': False, 'brakePressed': False, 'canValid': True, 'gearShifter': 'drive',
                             'brakeHoldActive': False, 'parkingBrake': False} | overrides))
    state = SimpleNamespace(CP=SimpleNamespace(brand=brand, flags=flags, openpilotLongitudinalControl=longitudinal),
                            params=SimpleNamespace(get_bool=lambda key: setting), mpc=SimpleNamespace(mode=mode),
                            v_desired_trajectory=SPEEDS.copy(), a_desired_trajectory=ACCELS.copy(), output_should_stop=original)
    exec(code, {'self': state, 'sm': {'carState': cs}, 'reset_state': reset, 'v_ego': .6, 'action_t': .25, 'vEgoStopping': .07,
                'CONTROL_N_T_IDX': TIMES, 'HyundaiFlags': HyundaiFlags, 'should_prepare_stop': should_prepare_stop})
    # The experiment changes the stop indication, not the acceleration plan.
    np.testing.assert_array_equal(state.a_desired_trajectory, ACCELS)
    return state.output_should_stop
  return run


def test_planner_early_stop_is_opt_in_and_does_not_suppress_existing_stop(planner_early_stop):
  assert planner_early_stop()
  assert not planner_early_stop(setting=False)
  assert planner_early_stop(setting=False, original=True)


@pytest.mark.parametrize('blocked', [
  {'brand': 'toyota'}, {'flags': 0}, {'longitudinal': False}, {'mode': 'blended'}, {'reset': True},
  {'gasPressed': True}, {'brakePressed': True}, {'canValid': False}, {'gearShifter': 'reverse'},
  {'brakeHoldActive': True}, {'parkingBrake': True},
])
def test_planner_retains_other_modes_and_interlocks(planner_early_stop, blocked):
  assert not planner_early_stop(**blocked)


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
