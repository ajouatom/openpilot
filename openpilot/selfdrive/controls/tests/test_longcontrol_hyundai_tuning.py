import sys
from types import ModuleType, SimpleNamespace

import pytest


# The production modules are available on the Linux device. Windows unit-test
# hosts need small import shims for platform-only hardware and Params bindings;
# the tests below exercise the tuning refresh directly without constructing
# either dependency.
if sys.platform == "win32":
  hardware_module = ModuleType("openpilot.system.hardware")
  hardware_module.PC = True
  sys.modules.setdefault("openpilot.system.hardware", hardware_module)

  params_module = ModuleType("openpilot.common.params")
  params_module.Params = object
  sys.modules.setdefault("openpilot.common.params", params_module)

import openpilot.selfdrive.controls.lib.longcontrol as longcontrol_module
from openpilot.selfdrive.controls.lib.longcontrol import (
  HYUNDAI_LONGITUDINAL_KF,
  HYUNDAI_LONGITUDINAL_KI,
  HYUNDAI_LONGITUDINAL_KP,
  LongControl,
)


class RejectingParams:
  def get_float(self, name):
    raise AssertionError(f"Fixed tuning must not read adjustable params: {name}")


class DictParams:
  def __init__(self, values):
    self.values = values
    self.writes = []

  def get_float(self, name):
    assert name != "StoppingAccel", "Removed stopping acceleration setting must never be read"
    return self.values[name]

  def put_int(self, name, value):
    self.values[name] = value
    self.writes.append((name, value))


def make_cp(brand="hyundai"):
  return SimpleNamespace(
    brand=brand,
    longitudinalTuning=SimpleNamespace(
      kpBP=[0.0], kpV=[9.0], kiBP=[0.0], kiV=[9.0], kf=9.0,
    ),
    startingState=False,
    vEgoStarting=0.5,
    stopAccel=0.0,
    stoppingDecelRate=0.8,
  )


def make_control(*, hyundai, params):
  control = LongControl.__new__(LongControl)
  control.hyundai_fixed_longitudinal_tuning = hyundai
  control.pid = SimpleNamespace(_k_p=([0.0], [9.0]), _k_i=([0.0], [9.0]), k_f=9.0)
  control.CP = SimpleNamespace(
    longitudinalTuning=SimpleNamespace(kpBP=[0.0], kiBP=[0.0]),
  )
  control.params = params
  return control


def test_hyundai_constructor_overrides_car_tune_immediately(monkeypatch):
  cp = make_cp()
  monkeypatch.setattr(longcontrol_module, "Params", RejectingParams)

  control = LongControl(cp)

  assert control.hyundai_fixed_longitudinal_tuning is True
  assert control.pid._k_p == ([0.0], [HYUNDAI_LONGITUDINAL_KP])
  assert control.pid._k_i == ([0.0], [HYUNDAI_LONGITUDINAL_KI])
  assert control.pid.k_f == HYUNDAI_LONGITUDINAL_KF


@pytest.mark.parametrize("brand", ["hyundai", "toyota", "gm", "mock"])
@pytest.mark.parametrize("stored_value", [None, 0, -10, -50, -100])
def test_stopping_accel_is_fixed_for_all_brands(monkeypatch, brand, stored_value):
  params = DictParams({"StoppingAccel": stored_value})
  monkeypatch.setattr(longcontrol_module, "Params", lambda: params)

  control = LongControl(make_cp(brand))

  assert params.values["StoppingAccel"] == stored_value
  assert control.stopping_accel == -0.5
  assert params.writes == []


@pytest.mark.parametrize("brand", ["hyundai", "toyota", "gm", "mock"])
@pytest.mark.parametrize("stored_value", [None, 0, -10, -50, -100])
def test_fixed_stop_target_from_first_frame_and_after_param_refresh(monkeypatch, brand, stored_value):
  params = DictParams({"StoppingAccel": stored_value, "LongTuningKpV": 100, "LongTuningKiV": 0, "LongTuningKf": 100})
  monkeypatch.setattr(longcontrol_module, "Params", lambda: params)
  control = LongControl(make_cp(brand))
  cs = SimpleNamespace(
    softHoldActive=0, vEgo=0.1, aEgo=0.0, brakePressed=False,
    cruiseState=SimpleNamespace(standstill=False),
  )
  plan = SimpleNamespace(aTarget=0.0, vTargetNow=0.0, jTargetNow=0.0, shouldStop=True)
  radar = SimpleNamespace(leadOne=SimpleNamespace(status=False, dRel=0.0))

  for _ in range(150):
    accel, _, _ = control.update(True, cs, plan, (-3.5, 2.0), 0.0, radar)
    assert accel < 0.0

  assert control.stopping_accel == -0.5
  assert -0.5 - control.CP.stoppingDecelRate * longcontrol_module.DT_CTRL <= accel <= -0.5
  assert params.writes == []


@pytest.mark.parametrize("a_ego, expected", [(-0.6, "pid"), (-0.5, "pid"), (-0.4, "stopping")])
def test_fixed_stop_entry_threshold(monkeypatch, a_ego, expected):
  monkeypatch.setattr(longcontrol_module, "Params", RejectingParams)
  control = LongControl(make_cp())
  control.long_control_state = longcontrol_module.LongCtrlState.pid
  cs = SimpleNamespace(
    softHoldActive=0, vEgo=0.1, aEgo=a_ego, brakePressed=False,
    cruiseState=SimpleNamespace(standstill=False),
  )
  plan = SimpleNamespace(aTarget=-0.5, vTargetNow=0.0, jTargetNow=0.0, shouldStop=True)
  radar = SimpleNamespace(leadOne=SimpleNamespace(status=False, dRel=0.0))

  control.update(True, cs, plan, (-3.5, 2.0), 0.0, radar)

  assert control.long_control_state == getattr(longcontrol_module.LongCtrlState, expected)


@pytest.mark.parametrize("soft_hold, previous_accel, expected", [(0, -1.0, -1.0), (1, 0.0, -2.0)])
def test_stopping_preserves_stronger_braking_and_vehicle_soft_hold(monkeypatch, soft_hold, previous_accel, expected):
  monkeypatch.setattr(longcontrol_module, "Params", RejectingParams)
  cp = make_cp()
  cp.stopAccel = -2.0
  control = LongControl(cp)
  control.last_output_accel = previous_accel
  cs = SimpleNamespace(
    softHoldActive=soft_hold, vEgo=0.0, aEgo=0.0, brakePressed=False,
    cruiseState=SimpleNamespace(standstill=True),
  )
  plan = SimpleNamespace(aTarget=0.0, vTargetNow=0.0, jTargetNow=0.0, shouldStop=True)
  radar = SimpleNamespace(leadOne=SimpleNamespace(status=False, dRel=0.0))

  accel, _, _ = control.update(True, cs, plan, (-3.5, 2.0), 0.0, radar)

  assert accel == expected


def test_hyundai_tuning_is_fixed_without_reading_params():
  control = make_control(hyundai=True, params=RejectingParams())

  control._refresh_longitudinal_tuning()

  assert control.pid._k_p == ([0.0], [HYUNDAI_LONGITUDINAL_KP])
  assert control.pid._k_i == ([0.0], [HYUNDAI_LONGITUDINAL_KI])
  assert control.pid.k_f == HYUNDAI_LONGITUDINAL_KF
  assert (HYUNDAI_LONGITUDINAL_KP, HYUNDAI_LONGITUDINAL_KI, HYUNDAI_LONGITUDINAL_KF) == (1.0, 0.0, 1.0)


def test_other_brands_keep_adjustable_single_point_tuning():
  control = make_control(
    hyundai=False,
    params=DictParams({"LongTuningKpV": 85, "LongTuningKiV": 25, "LongTuningKf": 120}),
  )

  control._refresh_longitudinal_tuning()

  assert control.pid._k_p == ([0.0], [0.85])
  assert control.pid._k_i == ([0.0], [0.025])
  assert control.pid.k_f == 1.2
