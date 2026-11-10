from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import car
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.selfdrive.controls.lib.drive_helpers import (
  MAX_LATERAL_JERK,
  model_action_curvature,
  supports_low_speed_torque_response,
)
from openpilot.selfdrive.controls.tests.test_lateral_startup_control import control_fixture


def profile(name='HYUNDAI_PORTER_II_EV', *, angle=False, flags=None):
  return NS(
    brand='hyundai',
    carFingerprint=name,
    flags=int(HyundaiFlags.CANFD_HDA2 | HyundaiFlags.CAMERA_SCC) if flags is None else flags,
    steerControlType=car.CarParams.SteerControlType.angle if angle else car.CarParams.SteerControlType.torque,
  )


@pytest.mark.parametrize('cp', [profile(), profile('HYUNDAI_IONIQ_5')])
@pytest.mark.parametrize('speed', [0.31, 5.0, 30 / 3.6])
def test_already_smoothed_model_action_is_not_delayed_again_at_low_speed(cp, speed):
  assert supports_low_speed_torque_response(cp)
  assert model_action_curvature(cp, speed, 0.01, 0.005) == 0.01


@pytest.mark.parametrize(
  'cp',
  [
    profile(),
    profile('HYUNDAI_IONIQ_5'),
    profile('HYUNDAI_IONIQ_5', angle=True),
    profile('HYUNDAI_IONIQ_5', flags=0),
    profile('HYUNDAI_IONIQ_5_PE'),
    profile('TOYOTA_RAV4'),
  ],
)
@pytest.mark.parametrize('speed', [40 / 3.6, 20.0, 35.0])
def test_high_speed_and_unrelated_profiles_retain_exact_old_filter(cp, speed):
  alpha = 1 - np.exp(-0.01 / 0.1)
  assert model_action_curvature(cp, speed, -0.02, 0.01) == alpha * -0.02 + (1 - alpha) * 0.01


@pytest.mark.parametrize(
  'cp', [profile('HYUNDAI_IONIQ_5', angle=True), profile('HYUNDAI_IONIQ_5', flags=0), profile('HYUNDAI_IONIQ_5_PE'), profile('HYUNDAI_SONATA')]
)
def test_low_speed_exception_does_not_spread_to_other_steering_profiles(cp):
  assert not supports_low_speed_torque_response(cp)
  assert model_action_curvature(cp, 5.0, 0.01, 0.0) == pytest.approx(0.01 * (1 - np.exp(-0.1)))


def test_speed_blend_has_no_curvature_jump_at_either_boundary():
  cp = profile()
  for speed in (30 / 3.6, 40 / 3.6):
    assert abs(model_action_curvature(cp, speed - 1e-8, 0.02, 0.01) - model_action_curvature(cp, speed + 1e-8, 0.02, 0.01)) < 1e-9


@pytest.mark.parametrize('name', ['HYUNDAI_PORTER_II_EV', 'HYUNDAI_IONIQ_5'])
@pytest.mark.parametrize('sign', [-1, 1])
def test_real_control_path_preserves_jerk_limits_and_disengagement(name, sign):
  with pytest.MonkeyPatch.context() as patcher:
    patcher.setattr('openpilot.selfdrive.controls.lib.latcontrol_torque.Params', lambda: NS(get_int=lambda _k: 0))
    control, step = control_fixture()
  cp = control.CP.as_builder()
  cp.carFingerprint, cp.flags = name, profile(name).flags
  control.CP = cp.as_reader()
  control.lateral_startup.ready = control.lateral_started = True
  control.sm['carState'].vEgo = 5.0
  control.sm['modelV2'].action.desiredCurvature = sign * 0.02
  previous = 0.0
  for _ in range(50):
    cc, _ = step()
    assert cc.latActive and not cc.longActive
    current = cc.actuators.curvature
    assert abs(current - previous) <= MAX_LATERAL_JERK / 25 * 0.01 + 1e-8
    assert 0 <= sign * current <= 0.02 + 1e-8
    assert abs(cc.actuators.torque) <= 1.0
    previous = current
  assert previous == pytest.approx(sign * 0.02)
  control.sm['carState'].latEnabled = False
  cc, _ = step()
  assert not cc.latActive and cc.actuators.torque == 0
