"""Exercise the production planner's mode routing without the native MPC/IPC backend."""

import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.lane_model_speed import LaneModelSpeedGuard


class StubMpc:
  def reset(self, x0):
    self.x_sol = np.zeros((33, 4))
    self.u_sol = np.zeros((32, 1))
    self.solution_status = 0
    self.cost = 0.0

  def set_weights(self, *args):
    pass

  def run(self, x0, p, y, heading, yaw_rate):
    self.input_speed = p[:, 0].copy()


class StubLanePlanner:
  def __init__(self):
    self.debugText = ''

  def parse_model(self, md):
    pass

  def get_d_path(self, cs, speed, t, path, curve_speed):
    return path, self.lanefull_mode


def make_planner():
  # Compile the complete, unchanged production class. Only the numerical MPC,
  # lane geometry, Params and IPC imports are substituted for routing tests.
  source = Path(__file__).parents[1] / 'lib/lateral_planner.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  params = SimpleNamespace(get_int=lambda _: 1, get_float=lambda _: 1.0)
  namespace = {
    'np': np,
    'DT_MDL': 0.05,
    'TRAJECTORY_SIZE': 33,
    'MIN_SPEED': 1.0,
    'LAT_MPC_N': 32,
    'Params': lambda: params,
    'LateralMpc': StubMpc,
    'LanePlanner': StubLanePlanner,
    'LaneModelSpeedGuard': LaneModelSpeedGuard,
    'time': SimpleNamespace(monotonic=lambda: 0),
    'log': SimpleNamespace(Desire=SimpleNamespace(none=0)),
    'yaw_from_path_no_scipy': lambda *a, **k: (np.zeros(33), np.zeros(33)),
  }
  exec(
    compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LateralPlanner'], type_ignores=[]), str(source), 'exec'),
    namespace,
  )
  cp = SimpleNamespace(wheelbase=2.7, centerToFront=1.1, mass=1800, tireStiffnessRear=100000)
  return namespace['LateralPlanner'](cp)


def inputs(speed=30.0, start=30.0, end=30.0, lane_speed=1):
  model = SimpleNamespace(
    position=SimpleNamespace(x=np.linspace(0, 300, 33), y=np.zeros(33), z=np.zeros(33), t=np.linspace(0, 10, 33)),
    orientation=SimpleNamespace(x=np.zeros(33), z=np.zeros(33)),
    orientationRate=SimpleNamespace(z=np.zeros(33)),
    velocity=SimpleNamespace(x=np.linspace(start, end, 33), y=np.zeros(33), z=np.zeros(33)),
    acceleration=SimpleNamespace(x=np.zeros(33)),
    meta=SimpleNamespace(desire=0, laneWidthLeft=3.5, laneWidthRight=3.5),
  )
  return {
    'modelV2': model,
    'carState': SimpleNamespace(vEgo=speed, useLaneLineSpeed=lane_speed),
    'controlsState': SimpleNamespace(curvature=0.0),
    'carrotMan': SimpleNamespace(vTurnSpeed=200),
  }


def update(planner, sm):
  planner.update(sm, SimpleNamespace(atc_active=False))
  return planner.lanelines_active


def test_actual_planner_immediately_routes_mismatched_speed_to_laneless():
  planner = make_planner()
  for _ in range(21):
    update(planner, inputs())
  assert planner.lanelines_active
  assert not update(planner, inputs(29.04, 9.93, 12.09))
  # The fix changes eligibility, not model geometry, MPC speed or actuator limits.
  assert planner.v_plan[0] == 9.93
  for _ in range(20):
    assert not update(planner, inputs())
  assert update(planner, inputs())


def test_original_deceleration_and_explicit_laneless_remain_effective():
  planner = make_planner()
  for _ in range(30):
    assert not update(planner, inputs(lane_speed=0))
  for _ in range(30):
    update(planner, inputs())
  assert planner.lanelines_active
  assert not update(planner, inputs(30, 30, 15))


def test_missing_model_speed_cannot_reuse_lane_mode_eligibility():
  planner = make_planner()
  for _ in range(21):
    update(planner, inputs())
  sm = inputs()
  sm['modelV2'].velocity.x = []
  assert not update(planner, sm)
  for _ in range(20):
    assert not update(planner, inputs())
  assert update(planner, inputs())
