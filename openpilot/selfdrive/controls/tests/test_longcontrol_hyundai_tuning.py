import sys
from types import ModuleType, SimpleNamespace


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
  def get_float(self, _name):
    raise AssertionError("Hyundai fixed tuning must not read adjustable PID params")


class DictParams:
  def __init__(self, values):
    self.values = values

  def get_float(self, name):
    return self.values[name]


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
  cp = SimpleNamespace(
    brand="hyundai",
    longitudinalTuning=SimpleNamespace(
      kpBP=[0.0], kpV=[9.0], kiBP=[0.0], kiV=[9.0], kf=9.0,
    ),
  )
  monkeypatch.setattr(longcontrol_module, "Params", RejectingParams)

  control = LongControl(cp)

  assert control.hyundai_fixed_longitudinal_tuning is True
  assert control.pid._k_p == ([0.0], [HYUNDAI_LONGITUDINAL_KP])
  assert control.pid._k_i == ([0.0], [HYUNDAI_LONGITUDINAL_KI])
  assert control.pid.k_f == HYUNDAI_LONGITUDINAL_KF


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
