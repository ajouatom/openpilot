import json
import sys
from pathlib import Path
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

from opendbc.car.hyundai.values import HyundaiFlags
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
      return -50.0
    raise AssertionError(f"Fixed tuning must not read adjustable params: {name}")


class DictParams:
  def __init__(self, values):
    self.values = values
    self.writes = []

  def get_float(self, name):
    return float(self.values.get(name, 0))

  def get_bool(self, name):
    assert name != "CanfdStopRetry", "Removed stop retry setting must never be read"
    return bool(self.values.get(name, False))

  def put_int(self, name, value):
    self.values[name] = value
    self.writes.append((name, value))


def make_cp(brand="hyundai"):
  return SimpleNamespace(
    brand=brand,
    flags=0, openpilotLongitudinalControl=True,
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
@pytest.mark.parametrize("stored, expected", [
  (None, -.5), (0, -.5), (10, -.5), (-10, -.5), (-50, -.5), (-70, -.7),
  (-100, -1.), (-150, -1.), (float('nan'), -.5), (float('inf'), -.5), ('invalid', -.5),
])
def test_stopping_accel_bounds_apply_from_first_frame(monkeypatch, brand, stored, expected):
  params = DictParams({"StoppingAccel": stored})
  monkeypatch.setattr(longcontrol_module, "Params", lambda: params)
  control = LongControl(make_cp(brand))
  assert control.stopping_accel == pytest.approx(expected)
  assert params.writes == []


def stop_inputs(*, soft_hold=0, a_ego=0., nearby=False):
  cs = SimpleNamespace(softHoldActive=soft_hold, vEgo=.1, aEgo=a_ego, brakePressed=False,
                       cruiseState=SimpleNamespace(standstill=False))
  plan = SimpleNamespace(aTarget=-.5, vTargetNow=.1, jTargetNow=0., shouldStop=True)
  radar = SimpleNamespace(leadOne=SimpleNamespace(status=nearby, dRel=3.))
  return cs, plan, radar


@pytest.mark.parametrize("flags", [0, HyundaiFlags.CANFD])
@pytest.mark.parametrize("stored", [-50, -80, -100])
@pytest.mark.parametrize("offset, expected", [(-.1, "pid"), (0., "pid"), (.1, "stopping")])
def test_restored_entry_threshold(monkeypatch, flags, stored, offset, expected):
  monkeypatch.setattr(longcontrol_module, "Params", lambda: DictParams({"StoppingAccel": stored}))
  cp = make_cp()
  cp.flags = flags
  control = LongControl(cp)
  control.long_control_state = longcontrol_module.LongCtrlState.pid
  cs, plan, radar = stop_inputs(a_ego=stored * .01 + offset)
  control.update(True, cs, plan, (-3.5, 2.), 0., radar)
  assert control.long_control_state == getattr(longcontrol_module.LongCtrlState, expected)


@pytest.mark.parametrize("flags", [0, HyundaiFlags.CANFD])
def test_original_close_lead_handover_and_launch(monkeypatch, flags):
  monkeypatch.setattr(longcontrol_module, "Params", RejectingParams)
  cp = make_cp()
  cp.flags = flags
  control = LongControl(cp)
  control.long_control_state = longcontrol_module.LongCtrlState.pid
  control.last_output_accel = -.8
  cs, plan, radar = stop_inputs(a_ego=-.8, nearby=True)
  accel, _, _ = control.update(True, cs, plan, (-3.5, 2.), 0., radar)
  assert control.long_control_state == longcontrol_module.LongCtrlState.stopping
  assert accel == -.8
  plan.shouldStop = False
  control.update(True, cs, plan, (-3.5, 2.), 0., radar)
  assert control.long_control_state == longcontrol_module.LongCtrlState.pid


@pytest.mark.parametrize("flags", [0, HyundaiFlags.CANFD])
@pytest.mark.parametrize("soft_hold", [0, 1])
@pytest.mark.parametrize("initial", [-1.2, -.3, 0.])
@pytest.mark.parametrize("stored", [-50, -80, -100])
def test_original_one_way_ramp_and_vehicle_soft_hold(monkeypatch, flags, soft_hold, initial, stored):
  monkeypatch.setattr(longcontrol_module, "Params", lambda: DictParams({"StoppingAccel": stored}))
  cp = make_cp()
  cp.flags = flags
  cp.stopAccel = -2.
  control = LongControl(cp)
  control.last_output_accel = initial
  cs, plan, radar = stop_inputs(soft_hold=soft_hold)
  previous = initial
  for _ in range(150):
    accel, _, _ = control.update(True, cs, plan, (-3.5, 2.), 0., radar)
    if soft_hold:
      assert accel == cp.stopAccel
    else:
      assert previous - cp.stoppingDecelRate * longcontrol_module.DT_CTRL - 1e-9 <= accel <= previous
    previous = accel
  if not soft_hold:
    target = min(initial, stored * .01)
    assert target - cp.stoppingDecelRate * longcontrol_module.DT_CTRL - 1e-9 <= accel <= target


@pytest.mark.parametrize("stored, expected", [(-80, -.8), (0, -.5), (-200, -1.), ('bad', -.5)])
def test_live_setting_refresh_keeps_retry_independent(monkeypatch, stored, expected):
  params = DictParams({"StoppingAccel": -50, "CanfdStopRetry": False})
  monkeypatch.setattr(longcontrol_module, "Params", lambda: params)
  control = LongControl(make_cp())
  cs, plan, radar = stop_inputs()
  control.update(True, cs, plan, (-3.5, 2.), 0., radar)
  params.values["StoppingAccel"] = stored
  for _ in range(99):
    control.update(True, cs, plan, (-3.5, 2.), 0., radar)
  assert control.stopping_accel == pytest.approx(expected)


def test_stopping_setting_catalog_bounds_and_menu():
  root = Path(__file__).resolve().parents[4]
  catalog = json.loads((root / 'openpilot/selfdrive/carrot_settings.json').read_text(encoding='utf-8'))
  def objects(value):
    if isinstance(value, dict):
      yield value
      for child in value.values():
        yield from objects(child)
    elif isinstance(value, list):
      for child in value:
        yield from objects(child)
  entries = list(objects(catalog))
  setting = next(x for x in entries if x.get('name') == 'StoppingAccel')
  assert (setting['min'], setting['max'], setting['default'], setting['unit']) == (-100, -50, -50, 10)
  menu = next(x for x in entries if x.get('id') == 'CRUISE_STOPGO')
  assert 'StoppingAccel' in menu['params']
  assert not any(x.get('name') == 'CanfdStopRetry' for x in entries)
  assert '{"StoppingAccel", {PERSISTENT, INT, "-50"}}' in (root / 'openpilot/common/params_keys.h').read_text(encoding='utf-8')


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
