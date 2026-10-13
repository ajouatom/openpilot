"""RAV4 mode0의 PCM 관측·입력 우선순위·공통 감속 상한 회귀.

Params와 블루투스 입출력은 메모리에 격리한다. 순항 클래스는 전체 메서드를,
당근 상한은 원형 AST 본문과 실제 get_carrot_man을 실행한다.
MPC·호스트 권한·IPC·CAN 송신·실차 검증은 아니다.
"""

import ast
from enum import Enum
from functools import cache
import math
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
import openpilot.selfdrive.car.cruise as cruise
from openpilot.selfdrive.car.toyota_pcm_speed import ToyotaPcmSetSpeedSync
from openpilot.selfdrive.carrot.carrot_man_input import get_carrot_man


class MemoryParams:
  def __init__(self, *args, **kwargs):
    self.values = {"CruiseSpeedUnit": 10, "CruiseSpeedUnitBasic": 1, "CruiseButtonLongDelay": 50,
                   "CruiseButtonMode": 0, "AutoNaviSpeedSafetyFactor": 100, "AutoGasTokSpeed": 30,
                   "AutoGasCancelSpeed": 30, "SpeedFromPCM": 0}

  def get_int(self, key):
    return int(self.values.get(key, 0))

  def get_float(self, key):
    return float(self.values.get(key, 0))

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_int_nonblocking(self, key, value):
    self.values[key] = int(value)

  def put_bool_nonblocking(self, key, value):
    self.values[key] = bool(value)


class MemoryReader:
  def __init__(self, *args, **kwargs):
    self.action = None
    self.is_repeat = False

  def read(self, *, allowed):
    value, self.action = self.action, None
    return value if allowed else None


class SubMaster(dict):
  def __init__(self, cs, *, speed=250, source="road"):
    event = log.Event.new_message()
    event.init("carrotMan")
    event.carrotMan.desiredSpeed = speed
    event.carrotMan.desiredSource = source
    event.carrotMan.nRoadLimitSpeed = 80
    super().__init__(carState=cs, carControl=car.CarControl(enabled=True, longActive=True), carrotMan=event.carrotMan)
    self.alive = {"carrotMan": True, "longitudinalPlan": False, "radarState": False, "drivingModelData": False}
    self.seen = {"carrotMan": True}
    self.valid = {"carrotMan": True}
    self.recv_time = {"carrotMan": time.monotonic()}


def make_cp(**overrides):
  values = {"brand": "toyota", "carFingerprint": "TOYOTA_RAV4_TSS2_2022", "pcmCruise": True,
            "openpilotLongitudinalControl": True}
  values.update(overrides)
  return car.CarParams(**values)


def make_cs(speed=75., *, enabled=False, cluster=None):
  return car.CarState(canValid=True, gearShifter="drive", vEgo=21., vEgoCluster=21.,
                      cruiseState={"available": True, "enabled": enabled, "speed": speed / 3.6,
                                   "speedCluster": (speed + 1. if cluster is None else cluster) / 3.6})


@pytest.fixture
def full_helper(monkeypatch):
  monkeypatch.setattr(cruise, "Params", MemoryParams)
  monkeypatch.setattr(cruise, "CommandReader", MemoryReader)
  value = cruise.VCruiseCarrot(make_cp())
  value.v_cruise_kph = value.v_cruise_cluster_kph = 83.
  value.cruise_state_available_last = True
  # 페달 초기 경계가 아닌, 두 페달을 계속 놓고 있는 관측 상태다.
  value._gas_pressed_count = value._brake_pressed_count = -2
  return value


def step(helper, cs, sm=None):
  sm = SubMaster(cs) if sm is None else sm
  sm["carState"] = cs
  helper.update_v_cruise(cs, sm, True)
  return helper.v_cruise_kph


@cache
def cap_function():
  root = Path(__file__).resolve().parents[4]
  tree = ast.parse((root / "openpilot/selfdrive/carrot/carrot_functions.py").read_text(encoding="utf-8"))
  enum_node = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "XState")
  planner = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "CarrotPlanner")
  method = next(node for node in planner.body if isinstance(node, ast.FunctionDef) and node.name == "_update_carrot_man")
  realtime = ast.parse((root / "openpilot/common/realtime.py").read_text(encoding="utf-8"))
  dt_node = next(node for node in realtime.body if isinstance(node, ast.Assign)
                 and any(isinstance(target, ast.Name) and target.id == "DT_MDL" for target in node.targets))
  namespace = {"Enum": Enum, "get_carrot_man": get_carrot_man, "EventName": log.OnroadEvent.EventName,
               "DT_MDL": ast.literal_eval(dt_node.value)}
  # 상한 판단 본문은 그대로 실행하고, 무관한 Events/IPC 모듈을 불러오지 않는다.
  exec(compile(ast.Module(body=[enum_node, method], type_ignores=[]), "<원형 당근 상한 메서드>", "exec"), namespace)
  return namespace


def cap(sm, base):
  namespace = cap_function()
  value = SimpleNamespace()
  value.signal_detection_suspended = False
  value.trafficState_carrot = 0
  value.soft_hold_active = 0
  value.xState = namespace["XState"].e2eCruise
  return namespace["_update_carrot_man"](value, sm, 75.6, base)[0]


def test_first_enabled_and_stable_stock_only_establish_a_baseline(full_helper):
  cs = make_cs(enabled=True)
  assert step(full_helper, cs) == 83.
  assert step(full_helper, cs) == 83.


def test_set_adjust_cancel_resume_without_events_and_without_cp_change(full_helper):
  cs = make_cs()
  before = full_helper.CP.to_dict()
  assert step(full_helper, cs) == 83.
  cs.cruiseState.enabled = True
  assert step(full_helper, cs) == pytest.approx(75.)
  cs.cruiseState.speed = 74 / 3.6
  assert step(full_helper, cs) == pytest.approx(74.)
  cs.cruiseState.enabled = False
  cs.cruiseState.speed = 80 / 3.6
  assert step(full_helper, cs) == pytest.approx(74.)
  cs.cruiseState.enabled = True
  assert step(full_helper, cs) == pytest.approx(80.)
  assert not cs.buttonEvents and full_helper.CP.to_dict() == before


def test_first_set_after_disabled_unset_zero_is_observed(full_helper):
  cs = make_cs(0, cluster=0)
  step(full_helper, cs)
  cs.cruiseState.speed = 60 / 3.6
  cs.cruiseState.speedCluster = 61 / 3.6
  cs.cruiseState.enabled = True
  assert step(full_helper, cs) == pytest.approx(60.)
  assert full_helper.v_cruise_cluster_kph == pytest.approx(61.)


def test_real_bluetooth_button_wins_over_same_frame_pcm_and_stable_followup(full_helper):
  cs = make_cs(enabled=True)
  step(full_helper, cs)
  cs.cruiseState.speed = 74 / 3.6
  full_helper.bluetooth_commands.action = "decelCruise"
  assert step(full_helper, cs) == 82.
  assert step(full_helper, cs) == 82.
  cs.cruiseState.speed = 73 / 3.6
  assert step(full_helper, cs) == pytest.approx(73.)
  assert not cs.buttonEvents


def test_direct_carrot_speed_command_wins_and_refreshes_pcm_baseline(full_helper):
  cs = make_cs(enabled=True)
  sm = SubMaster(cs)
  step(full_helper, cs, sm)
  cs.cruiseState.speed = 74 / 3.6
  sm["carrotMan"].carrotCmdIndex = 1
  sm["carrotMan"].carrotCmd = "SPEED"
  sm["carrotMan"].carrotArg = "60"
  assert step(full_helper, cs, sm) == 60.
  assert step(full_helper, cs, sm) == 60.


@pytest.mark.parametrize("source, desired", [("vturn", 55), ("route", 45), ("cam", 30)])
def test_existing_curve_map_camera_cap_remains_below_pcm_base(full_helper, source, desired):
  cs = make_cs()
  sm = SubMaster(cs, speed=desired, source=source)
  step(full_helper, cs, sm)
  cs.cruiseState.enabled = True
  assert step(full_helper, cs, sm) == pytest.approx(75.)
  assert cap(sm, full_helper.v_cruise_kph) == desired
  cs.cruiseState.speed = 90 / 3.6
  assert step(full_helper, cs, sm) == pytest.approx(90.)
  assert cap(sm, full_helper.v_cruise_kph) == desired


@pytest.mark.parametrize("fault", ["unseen", "stale", "invalid", "zero", "future"])
def test_unusable_navigation_cap_is_not_an_imaginary_stop(fault):
  sm = SubMaster(make_cs(), speed=45)
  if fault == "unseen":
    sm.seen["carrotMan"] = False
  elif fault == "invalid":
    sm.valid["carrotMan"] = False
  elif fault == "zero":
    sm["carrotMan"].desiredSpeed = 0
  else:
    sm.recv_time["carrotMan"] = time.monotonic() + (0.1 if fault == "future" else -1.1)
  assert cap(sm, 83.) == 83.


@pytest.mark.parametrize("overrides, mode, expected", [
  ({"brand": "honda"}, 0, 83), ({"brand": "hyundai"}, 0, 83),
  ({"carFingerprint": "TOYOTA_RAV4_TSS2"}, 0, 83),
  ({"carFingerprint": "TOYOTA_RAV4_TSS2_2023"}, 0, 83),
  ({"pcmCruise": False}, 0, 83), ({"openpilotLongitudinalControl": False}, 0, 83),
  ({}, 1, 74), ({}, 2, 83), ({}, 3, 74),
])
def test_other_vehicle_stock_control_and_existing_modes_keep_original_policy(full_helper, overrides, mode, expected):
  full_helper.CP = make_cp(**overrides)
  full_helper.params.values["SpeedFromPCM"] = mode
  cs = make_cs(enabled=True)
  step(full_helper, cs)
  cs.cruiseState.speed = 74 / 3.6
  assert step(full_helper, cs) == pytest.approx(expected)


@pytest.mark.parametrize("invalid", [0, -1, 145.001, 146, 255, math.nan, math.inf])
def test_bad_speed_resets_a_known_disabled_transition(invalid):
  sync, cp, cs = ToyotaPcmSetSpeedSync(), make_cp(), make_cs()
  assert sync.update(cp, 0, cs) is None
  cs.cruiseState.speed = invalid / 3.6
  cs.cruiseState.enabled = True
  assert sync.update(cp, 0, cs) is None
  cs.cruiseState.speed = 75 / 3.6
  assert sync.update(cp, 0, cs) is None


@pytest.mark.parametrize("fault", ["can", "available", "mode"])
def test_lost_input_or_mode_switch_does_not_create_a_resume_edge(fault):
  sync, cp, cs = ToyotaPcmSetSpeedSync(), make_cp(), make_cs()
  assert sync.update(cp, 0, cs) is None
  mode = 0
  if fault == "can":
    cs.canValid = False
  elif fault == "available":
    cs.cruiseState.available = False
  else:
    mode = 3
  assert sync.update(cp, mode, cs) is None
  cs.canValid = cs.cruiseState.available = cs.cruiseState.enabled = True
  assert sync.update(cp, 0, cs) is None


@pytest.mark.parametrize("cluster", [0, -1, 255, math.nan, math.inf])
def test_cluster_sentinel_falls_back_to_valid_base(cluster):
  sync, cp, cs = ToyotaPcmSetSpeedSync(), make_cp(), make_cs(cluster=cluster)
  assert sync.update(cp, 0, cs) is None
  cs.cruiseState.enabled = True
  assert sync.update(cp, 0, cs) == pytest.approx((75., 75.))


def test_float32_145_kph_wire_boundary_is_valid():
  sync, cp, cs = ToyotaPcmSetSpeedSync(), make_cp(), make_cs(145, cluster=145)
  with car.CarState.from_bytes(cs.as_reader().as_builder().to_bytes()) as decoded:
    assert sync.update(cp, 0, decoded) is None
  cs.cruiseState.enabled = True
  with car.CarState.from_bytes(cs.as_reader().as_builder().to_bytes()) as decoded:
    assert sync.update(cp, 0, decoded) == pytest.approx((145., 145.))


def test_missing_can_valid_is_not_an_observed_enable_edge():
  sync, cp = ToyotaPcmSetSpeedSync(), make_cp()
  cs = SimpleNamespace(cruiseState=SimpleNamespace(available=True, enabled=False, speed=75 / 3.6))
  assert sync.update(cp, 0, cs) is None
  cs.cruiseState.enabled = True
  assert sync.update(cp, 0, cs) is None


def test_mode0_keeps_the_original_30_kph_pcm_clip(full_helper):
  cs = make_cs(20)
  step(full_helper, cs)
  cs.cruiseState.enabled = True
  assert step(full_helper, cs) == 30.


@pytest.mark.parametrize("change, apply", [(0.49, False), (0.5, True)])
def test_pcm_changes_require_half_kph(change, apply):
  sync, cp, cs = ToyotaPcmSetSpeedSync(), make_cp(), make_cs(enabled=True)
  assert sync.update(cp, 0, cs) is None
  cs.cruiseState.speed = (75 + change) / 3.6
  result = sync.update(cp, 0, cs)
  assert (result is not None) is apply
