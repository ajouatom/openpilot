import ast
import math
from functools import partial
from numbers import Number
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import car, log
from openpilot.common.realtime import DT_CTRL
from openpilot.common.constants import CV
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, get_lag_adjusted_curvature
from openpilot.selfdrive.controls.lib.latcontrol import MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.lateral_readiness import LateralStartupGate, lateral_vehicle_parameters
from openpilot.selfdrive.controls.lib.steer_ratio import resolve_vehicle_model_steer_ratio
from openpilot.selfdrive.controls.tests.test_lateral_readiness import Messages


def control_fixture():
  # Execute the complete production state_control and actual torque controller,
  # replacing only IPC/settings, the longitudinal loop and hardware construction.
  path = Path(__file__).parents[1] / 'controlsd.py'
  tree = ast.parse(path.read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controls')
  names = {'lateral_control_allowed', 'state_control'}
  nodes = [n for n in tree.body + cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
  ns = {'math': math, 'np': np, 'Number': Number, 'car': car, 'DT_CTRL': DT_CTRL, 'CV': CV,
        'MIN_LATERAL_CONTROL_SPEED': MIN_LATERAL_CONTROL_SPEED, 'LaneChangeState': log.LaneChangeState,
        'LaneChangeDirection': log.LaneChangeDirection, 'ACTUATOR_FIELDS': tuple(car.CarControl.Actuators.schema.fields),
        'clip_curvature': clip_curvature, 'get_lag_adjusted_curvature': get_lag_adjusted_curvature,
        'resolve_vehicle_model_steer_ratio': resolve_vehicle_model_steer_ratio,
        'lateral_vehicle_parameters': lateral_vehicle_parameters,
        'cloudlog': NS(error=lambda msg: pytest.fail(msg))}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), ns)
  cp = car.CarParams.new_message(brand='hyundai', steerRatio=12.81, wheelbase=2.84, mass=1600,
                                 rotationalInertia=2700, centerToFront=1.136,
                                 tireStiffnessFront=100000, tireStiffnessRear=100000)
  torque = cp.lateralTuning.init('torque')
  torque.kp = 1.0
  torque.ki = 0.1
  torque.kf = 1.0
  torque.latAccelFactor = 2.963873863
  torque.friction = 0.07813665
  torque.useSteeringAngle = True
  cp = cp.as_reader()
  ci = NS(use_nnff=False, use_nnff_lite=False,
          torque_from_lateral_accel=lambda: partial(CarInterfaceBase.torque_from_lateral_accel_linear, None),
          get_pid_accel_limits=lambda *args: (-3.0, 2.0))
  sm = Messages()
  sm.update(longitudinalPlan=NS(), radarState=NS(), carrotMan=NS(vTurnSpeed=100), liveDelay=NS(lateralDelay=0.1))
  sm.frame = 1
  sm.recv_frame = {'longitudinalPlan': 1}
  sm['carState'].gearShifter = 'drive'
  sm['carState'].latEnabled = True
  settings = {'AlwaysLateral': True, 'SteerRatioRate': 88, 'CustomSR': 0, 'LatSmoothSec': 13}
  params = NS(get_float=lambda k: float(settings.get(k, 0)), get_int=lambda k: int(settings.get(k, 0)),
              get_bool=lambda k: bool(settings.get(k, False)))
  control = NS(CP=cp, CI=ci, sm=sm, params=params, VM=VehicleModel(cp), LaC=LatControlTorque(cp, ci),
               LoC=NS(reset=lambda: None, update=lambda *args: (0.0, 0.0, 0.0), long_control_state='off'),
               carrot_controls=NS(lat_suspend_control=lambda cs, active: active), is_vw_meb=False,
               desired_curvature=0.0, lateral_startup=LateralStartupGate(), lateral_started=False,
               steer_limited_by_safety=False)
  return control, lambda: ns['state_control'](control)


@pytest.mark.parametrize('enabled', [False, True])
def test_full_control_loop_waits_only_for_first_readiness(enabled):
  control, step = control_fixture()
  sm = control.sm
  sm['selfdriveState'].enabled = sm['selfdriveState'].active = enabled
  live = sm['liveParameters']
  sm['liveParameters'] = log.LiveParametersData.new_message()
  sm.seen['liveParameters'] = sm.seen['modelV2'] = False
  for _ in range(1500):  # longer than either initialization timeout
    cc, state = step()
    assert not cc.latActive and cc.actuators.torque == 0
    assert control.VM.sR == pytest.approx(12.81 * 0.88, rel=1e-6)
    assert abs(control.curvature) < 0.01
  sm['liveParameters'] = live
  sm.seen['liveParameters'] = sm.seen['modelV2'] = True
  control.desired_curvature = -0.15  # previous geometry/activation must not leak through
  cc, state = step()
  assert cc.latActive
  assert math.isfinite(cc.actuators.torque)
  assert abs(cc.actuators.curvature - control.curvature) < 0.001
  control.LaC.pid.i = 0.8
  sm.alive['modelV2'] = False
  cc, state = step()
  assert cc.latActive and control.LaC.pid.i > 0.7
  # Neither disengagement nor later invalidity rearms startup or nominal fallback.
  sm['carState'].latEnabled = False
  sm.valid['liveParameters'] = False
  live.steerRatio = 15.5
  cc, _ = step()
  assert not cc.latActive
  assert control.VM.sR == pytest.approx(15.5 * 0.88, rel=1e-6)
  assert control.LaC.pid.i > 0.7
  sm['carState'].latEnabled = True
  control.desired_curvature = 0.01
  cc, _ = step()
  assert cc.latActive and control.lateral_startup.ready
  assert cc.actuators.curvature > 0.005  # no new measured-curvature reset on reengagement


def test_normal_torque_reset_preserves_existing_integral_and_nn_history():
  control, _ = control_fixture()
  from collections import deque
  lac = control.LaC
  lac.use_nnff = True
  for name in ('lateral_accel_desired_deque', 'roll_deque', 'error_deque'):
    setattr(lac, name, deque([100.0] * 30, maxlen=30))
  lac.pid.i = 0.8
  lac.reset()
  assert lac.pid.i == 0.8
  assert len(lac.roll_deque) == len(lac.error_deque) == len(lac.lateral_accel_desired_deque) == 30
