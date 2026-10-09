from types import SimpleNamespace

import pytest

# Also provides the Windows-only Params/hardware import shims.
from openpilot.selfdrive.controls.tests.test_longcontrol_hyundai_tuning import DictParams, RejectingParams, make_cp, stop_inputs
import openpilot.selfdrive.controls.lib.longcontrol as lc


@pytest.mark.parametrize('target', [-4., -.5, -.1, 0., .2, 1., 3.])
def test_toyota_target_is_not_corrected_twice(monkeypatch, target):
  monkeypatch.setattr(lc, 'Params', RejectingParams)
  control = lc.LongControl(make_cp('toyota'))
  cs = SimpleNamespace(softHoldActive=0, vEgo=20., aEgo=0., brakePressed=False,
                       cruiseState=SimpleNamespace(standstill=False))
  plan = SimpleNamespace(aTarget=target, vTargetNow=25., jTargetNow=0., shouldStop=False)
  radar = SimpleNamespace()
  # Cross several live-refresh boundaries with acceleration noise and an
  # unrelated speed error. Only the PCM controller should correct tracking.
  for frame in range(220):
    cs.aEgo = (-1 if frame % 2 else 1) * .4
    accel, _, _ = control.update(True, cs, plan, (-3.5, 2.), 0., radar)
    assert accel == pytest.approx(min(2., max(-3.5, target)))
    assert control.pid.p == control.pid.i == 0.


def test_toyota_ignores_saved_and_live_outer_gains(monkeypatch):
  params = DictParams({'StoppingAccel': -50, 'LongTuningKpV': 200, 'LongTuningKiV': 2000, 'LongTuningKf': 0})
  monkeypatch.setattr(lc, 'Params', lambda: params)
  control = lc.LongControl(make_cp('toyota'))
  for gains in ((100, 0, 100), (0, 100, 200)):
    params.values.update(zip(('LongTuningKpV', 'LongTuningKiV', 'LongTuningKf'), gains, strict=True))
    control._refresh_longitudinal_tuning()
    assert control.pid._k_p == ([0.], [0.])
    assert control.pid._k_i == ([0.], [0.])
    assert control.pid.k_f == 1.
  assert params.writes == []


@pytest.mark.parametrize('soft_hold', [0, 1])
def test_toyota_stopping_and_disengagement_remain_active(monkeypatch, soft_hold):
  monkeypatch.setattr(lc, 'Params', RejectingParams)
  cp = make_cp('toyota')
  cp.stopAccel = -2.
  control = lc.LongControl(cp)
  control.last_output_accel = -.2
  cs, plan, radar = stop_inputs(soft_hold=soft_hold)
  accel, _, _ = control.update(True, cs, plan, (-3.5, 2.), 0., radar)
  assert control.long_control_state == lc.LongCtrlState.stopping
  assert accel == pytest.approx(-2. if soft_hold else -.2 - cp.stoppingDecelRate * lc.DT_CTRL)
  accel, _, _ = control.update(False, cs, plan, (-3.5, 2.), 0., radar)
  assert accel == 0.
  assert control.long_control_state == lc.LongCtrlState.off
