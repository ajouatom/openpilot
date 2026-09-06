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
    if name == "StoppingAccel":
      return -50
    raise AssertionError("Hyundai fixed tuning must not read adjustable PID params")


class DictParams:
  def __init__(self, values):
    self.values = values
    self.writes = []

  def get_float(self, name):
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


@pytest.mark.parametrize("stored_value", [0, -10, -50, -100])
def test_hyundai_startup_restores_only_zero_stopping_accel(monkeypatch, stored_value):
  params = DictParams({"StoppingAccel": stored_value})
  monkeypatch.setattr(longcontrol_module, "Params", lambda: params)

  control = LongControl(make_cp())

  expected = -50 if stored_value == 0 else stored_value
  assert params.values["StoppingAccel"] == expected
  assert control.stopping_accel == pytest.approx(expected * 0.01)
  assert params.writes == ([("StoppingAccel", -50)] if stored_value == 0 else [])


@pytest.mark.parametrize("brand", ["toyota", "gm", "mock"])
@pytest.mark.parametrize("stored_value", [0, -30])
def test_other_brands_keep_stopping_accel_at_startup(monkeypatch, brand, stored_value):
  params = DictParams({"StoppingAccel": stored_value})
  monkeypatch.setattr(longcontrol_module, "Params", lambda: params)

  control = LongControl(make_cp(brand))

  assert params.values["StoppingAccel"] == stored_value
  assert control.stopping_accel == pytest.approx(stored_value * 0.01)
  assert params.writes == []


def test_hyundai_stopping_accel_restored_again_after_user_saves_zero(monkeypatch):
  params = DictParams({"StoppingAccel": 0})
  monkeypatch.setattr(longcontrol_module, "Params", lambda: params)

  LongControl(make_cp())
  LongControl(make_cp())
  assert params.writes == [("StoppingAccel", -50)]

  params.values["StoppingAccel"] = 0
  control = LongControl(make_cp())
  assert control.stopping_accel == -0.5
  assert params.writes == [("StoppingAccel", -50), ("StoppingAccel", -50)]


def test_hyundai_zero_setting_brakes_from_first_frame_and_after_param_refresh(monkeypatch):
  params = DictParams({"StoppingAccel": 0})
  monkeypatch.setattr(longcontrol_module, "Params", lambda: params)
  control = LongControl(make_cp())
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
  assert params.writes == [("StoppingAccel", -50)]


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
