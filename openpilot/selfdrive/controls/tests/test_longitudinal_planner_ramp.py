import pytest
from types import SimpleNamespace
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.cereal import car


def test_continuous_accel_ramp_smoothness():
  dt = 0.05
  ramp_filter = FirstOrderFilter(0.0, 1.5, dt)

  # Starting from standstill (0 km/h) -> target 1.6 m/s^2
  target_a = 1.6
  accels = []
  for t in range(80):  # 4 seconds
    eff_target = max(0.6 if t == 0 else 0.0, target_a)
    ramp_filter.update(eff_target)
    accels.append(ramp_filter.x)

  # Verify initial step is gentle and continuous (no 1.6 spike at t=0)
  assert accels[0] < 0.1
  assert accels[10] < 0.8  # at 0.5s, progressive buildup
  assert accels[-1] == pytest.approx(1.6, abs=0.2)  # smoothly converges to target


def test_longcontrol_pos_limit_capping():
  CP = car.CarParams.new_message()
  CP.longitudinalTuning.kpBP = [0.0]
  CP.longitudinalTuning.kpV = [1.0]
  CP.longitudinalTuning.kiBP = [0.0]
  CP.longitudinalTuning.kiV = [0.1]
  lc = LongControl(CP)

  # Plan calls for accel of 1.2 m/s^2
  long_plan = SimpleNamespace(
    aTarget=1.2,
    vTargetNow=10.0,
    jTargetNow=0.0,
    shouldStop=False,
    hasLead=True,
  )
  accel_limits = [-3.5, 2.5]
  radarState = SimpleNamespace(leadOne=SimpleNamespace(status=True, dRel=20.0))
  CS = SimpleNamespace(
    vEgo=10.0,
    aEgo=1.2,
    brakePressed=False,
    softHoldActive=0,
    cruiseState=SimpleNamespace(standstill=False),
  )

  lc.update(True, CS, long_plan, accel_limits, 0.0, radarState)
  # PID pos_limit must be clamped to feedforward + 0.35 = 1.55 rather than full 2.5
  assert lc.pid.pos_limit == pytest.approx(1.55, abs=0.05)
