import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.lib.lateral_readiness import (
  LATERAL_SERVICES, LateralStartupGate, lateral_inputs_ready, lateral_vehicle_parameters,
)


class Messages(dict):
  def __init__(self):
    super().__init__(
      modelV2=log.ModelDataV2.new_message(),
      liveParameters=log.LiveParametersData.new_message(valid=True, sensorValid=True, posenetValid=True,
                                                       steerRatio=14.35, stiffnessFactor=1.0),
      livePose=log.LivePose.new_message(inputsOK=True, sensorsOK=True, posenetOK=True),
      liveTorqueParameters=log.LiveTorqueParametersData.new_message(),
      selfdriveState=log.SelfdriveState.new_message(), onroadEvents=[],
      lateralPlan=log.LateralPlan.new_message(), carControl=car.CarControl.new_message(),
      carState=car.CarState.new_message(canValid=True, vEgo=16.0, steeringAngleDeg=-2.2))
    self.seen = dict.fromkeys(self, True)
    self.valid = dict.fromkeys(self, True)
    self.alive = dict.fromkeys(self, True)
    self.freq_ok = dict.fromkeys(self, True)

  def all_checks(self, services):
    return all(self.valid[s] and self.alive[s] and self.freq_ok[s] for s in services)


@pytest.mark.parametrize('service,failure', [(s, f) for s in LATERAL_SERVICES
                                           for f in ('seen', 'valid', 'alive', 'freq_ok')
                                           if (s, f) != ('onroadEvents', 'freq_ok')])
def test_missing_invalid_stale_or_slow_input_blocks_lateral(service, failure):
  sm = Messages()
  assert lateral_inputs_ready(sm, sm['carState'])
  getattr(sm, failure)[service] = False
  assert not lateral_inputs_ready(sm, sm['carState'])
  getattr(sm, failure)[service] = True
  assert lateral_inputs_ready(sm, sm['carState'])


@pytest.mark.parametrize('name,value', [('steerRatio', 0), ('steerRatio', 0.1), ('steerRatio', math.nan),
                                      ('stiffnessFactor', 0), ('stiffnessFactor', math.inf),
                                      ('angleOffsetDeg', math.nan), ('roll', math.inf),
                                      ('valid', False), ('sensorValid', False), ('posenetValid', False)])
def test_invalid_live_geometry_never_replaces_nominal_or_permits_steering(name, value):
  sm = Messages()
  setattr(sm['liveParameters'], name, value)
  assert not lateral_inputs_ready(sm, sm['carState'])
  lp = lateral_vehicle_parameters(sm, NS(steerRatio=12.81))
  assert (lp.steerRatio, lp.stiffnessFactor, lp.angleOffsetDeg, lp.roll) == (12.81, 1.0, 0.0, 0.0)


def test_initialization_event_blocks_even_when_inputs_look_healthy():
  sm = Messages()
  sm['onroadEvents'] = [log.OnroadEvent.new_message(name='selfdriveInitializing')]
  assert not lateral_inputs_ready(sm, sm['carState'])
  sm['onroadEvents'] = []  # timeout alone cannot replace the unseen model
  sm.seen['modelV2'] = False
  assert not lateral_inputs_ready(sm, sm['carState'])


@pytest.mark.parametrize('field', ['inputsOK', 'sensorsOK', 'posenetOK'])
def test_unhealthy_pose_is_rejected(field):
  sm = Messages()
  setattr(sm['livePose'], field, False)
  assert not lateral_inputs_ready(sm, sm['carState'])


@pytest.mark.parametrize('field,value', [('canValid', False), ('steerFaultTemporary', True),
                                      ('steerFaultPermanent', True), ('vEgo', math.nan),
                                      ('steeringAngleDeg', math.inf), ('steeringTorque', math.nan)])
def test_invalid_car_state_is_rejected(field, value):
  sm = Messages()
  setattr(sm['carState'], field, value)
  assert not lateral_inputs_ready(sm, sm['carState'])


def test_invalid_model_action_and_selected_lane_plan_are_rejected():
  sm = Messages()
  sm['modelV2'].action.desiredCurvature = math.nan
  assert not lateral_inputs_ready(sm, sm['carState'])
  sm['modelV2'].action.desiredCurvature = 0.001
  sm['lateralPlan'].useLaneLines = True
  assert not lateral_inputs_ready(sm, sm['carState'])
  sm['lateralPlan'].mpcSolutionValid = True
  for name in ('psis', 'curvatures', 'distances'):
    setattr(sm['lateralPlan'], name, [0.0] * 17)
  assert lateral_inputs_ready(sm, sm['carState'])
  sm['lateralPlan'].curvatures[5] = math.nan
  assert not lateral_inputs_ready(sm, sm['carState'])
  sm['lateralPlan'].curvatures[5] = 0.0
  sm.alive['lateralPlan'] = False
  assert not lateral_inputs_ready(sm, sm['carState'])


def production_guard(path, predicate):
  tree = ast.parse(path.read_text(encoding='utf8'))
  node = next(n for n in ast.walk(tree) if predicate(n))
  return compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(path), 'exec')


@pytest.mark.parametrize('boundary', ['carState', 'carControl'])
@pytest.mark.parametrize('service,failure', [(s, f) for s in (*LATERAL_SERVICES, 'boundary')
                                           for f in ('seen', 'valid', 'alive', 'freq_ok')
                                           if (s, f) != ('onroadEvents', 'freq_ok')])
def test_startup_latch_waits_once_and_never_rechecks(boundary, service, failure):
  sm = Messages()
  gate = LateralStartupGate()
  service = boundary if service == 'boundary' else service
  getattr(sm, failure)[service] = False
  assert not gate.update(sm, sm['carState'], boundary)
  getattr(sm, failure)[service] = True
  assert gate.update(sm, sm['carState'], boundary)
  getattr(sm, failure)[service] = False
  assert gate.update(sm, sm['carState'], boundary)
  assert not LateralStartupGate().update(sm, sm['carState'], boundary)
  # Completed startup does not even read/check any inputs again.
  assert gate.update(None, None, boundary)


@pytest.mark.parametrize('failure', ['modelV2', 'liveParameters', 'livePose', 'carControl'])
@pytest.mark.parametrize('already_ready', [False, True])
def test_final_can_boundary_rejects_queued_active_command_only_during_startup(failure, already_ready):
  path = Path(__file__).parents[2] / 'car/card.py'
  latch = production_guard(path, lambda n: isinstance(n, ast.Assign) and
                           ast.unparse(n.targets[0]) == 'lateral_ready')
  guard = production_guard(path, lambda n: isinstance(n, ast.If) and
                           ast.unparse(n.test) == 'CC.latActive and (not lateral_ready)')
  sm = Messages()
  cc = car.CarControl.new_message(enabled=True, latActive=True, longActive=True)
  cc.actuators.torque = 1.0
  cc.actuators.accel = 0.7
  sm.valid[failure] = False
  ns = {'self': NS(sm=sm, lateral_startup=LateralStartupGate(ready=already_ready)),
        'CC': cc.as_reader(), 'CS': sm['carState']}
  exec(latch, ns)
  exec(guard, ns)
  result = ns['CC']
  if already_ready:
    assert result.latActive and result.actuators.torque == 1.0
  else:
    assert not result.latActive and result.actuators.torque == result.actuators.curvature == 0
    assert result.actuators.steeringAngleDeg == sm['carState'].steeringAngleDeg
  assert result.enabled and result.longActive and result.actuators.accel == pytest.approx(0.7)
  assert cc.latActive and cc.actuators.torque == 1  # original subscription is not mutated
