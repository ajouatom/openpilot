from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.angle_curvature_trim import AngleCurvatureTrim


def test_trim_and_driver_handoff():
  trim = AngleCurvatureTrim(1 << 24)
  cs = SimpleNamespace(steeringAngleDeg=0.0, steeringPressed=False, steeringRateDeg=10.0, vEgo=5.0)
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  vm = SimpleNamespace(calc_curvature=lambda angle, speed, roll: 0.0,
                       get_steer_from_curvature=lambda curvature, speed, roll: curvature * 1000.0)

  for _ in range(20):
    trim.update(True, cs, vm, params, 0.01)
  assert 0.0 < trim.curvature_trim <= 4.0e-4

  cs.steeringPressed = True
  assert trim.update(True, cs, vm, params, 0.01) == cs.steeringAngleDeg
  assert trim.curvature_trim == 0.0


def test_inactive_and_non_hyundai_do_not_trim():
  cs = SimpleNamespace(steeringAngleDeg=5.0, steeringPressed=False, steeringRateDeg=10.0, vEgo=5.0)
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  vm = SimpleNamespace(calc_curvature=lambda angle, speed, roll: 0.0,
                       get_steer_from_curvature=lambda curvature, speed, roll: curvature * 1000.0)
  trim = AngleCurvatureTrim(0)
  trim.update(True, cs, vm, params, 0.01)
  assert trim.curvature_trim == 0.0
  assert trim.update(False, cs, vm, params, 0.01) == cs.steeringAngleDeg
