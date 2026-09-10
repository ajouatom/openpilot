from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.car.cruise import VCruiseCarrot
from openpilot.selfdrive.carrot import carrot_man_input
from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner, XState


class SubMaster(dict):
  def __init__(self, *, seen=True, valid=True, alive=True, age=0.05, speed=50):
    event = log.Event.new_message()
    event.init('carrotMan')
    event.carrotMan.desiredSpeed = speed
    super().__init__(carrotMan=event.carrotMan, carState=SimpleNamespace(leftBlinker=False))
    self.seen = {'carrotMan': seen}
    self.valid = {'carrotMan': valid}
    self.alive = {'carrotMan': alive}
    self.recv_time = {'carrotMan': 100.0 - age}


@pytest.fixture(autouse=True)
def clock(monkeypatch):
  monkeypatch.setattr(carrot_man_input.time, 'monotonic', lambda: 100.0)


def planner():
  value = CarrotPlanner.__new__(CarrotPlanner)
  value.trafficState_carrot = 0
  value.soft_hold_active = 0
  value.xState = XState.e2eCruise
  return value


@pytest.mark.parametrize('overrides', [
  {'seen': False, 'speed': 0},  # Uploaded IONIQ 5: alive/valid defaults were true.
  {'seen': False, 'speed': 50},
  {'valid': False},
  {'alive': False},
  {'age': 1.001},
  {'age': -0.01},
  {'speed': 0},
  {'speed': -1},
  {'speed': 251},
])
def test_unusable_navigation_cannot_reduce_set_speed(overrides):
  sm = SubMaster(**overrides)
  value = planner()
  for cruise_speed in (21.0, 32.0, 50.0):
    assert value._update_carrot_man(sm, 20.0, cruise_speed) == (cruise_speed, False)


@pytest.mark.parametrize('speed, expected', [(1, 1), (30, 30), (50, 50), (250, 80)])
def test_received_navigation_preserves_existing_speed_cap(speed, expected):
  assert planner()._update_carrot_man(SubMaster(speed=speed), 60, 80) == (expected, False)


def test_navigation_loss_clears_turn_and_red_signal_state_and_recovers():
  sm = SubMaster(speed=30)
  sm['carState'].leftBlinker = True
  sm['carrotMan'].activeCarrot = 3
  sm['carrotMan'].xDistToTurn = 50
  sm['carrotMan'].atcType = 'turn left'
  sm['carrotMan'].trafficState = 1
  value = planner()
  assert value._update_carrot_man(sm, 60, 80) == (30, True)
  assert value.carrot_stay_stop

  sm.recv_time['carrotMan'] = 98.9
  assert value._update_carrot_man(sm, 60, 80) == (80, False)
  assert not value.carrot_stay_stop
  assert value.trafficState_carrot == value.activeCarrot == value.xDistToTurn == 0
  assert value.atcType == ''
  assert value.xState == XState.e2eCruise

  sm.recv_time['carrotMan'] = 99.95
  assert value._update_carrot_man(sm, 60, 80) == (30, True)


def test_navigation_loss_does_not_cancel_model_stop():
  value = planner()
  value.xState = XState.e2eStop
  assert value._update_carrot_man(SubMaster(seen=False), 20, 50) == (50, False)
  assert value.xState == XState.e2eStop


def test_cruise_helper_drops_stale_speed_and_commands_then_recovers():
  value = VCruiseCarrot.__new__(VCruiseCarrot)
  sm = SubMaster(speed=30)
  sm['carrotMan'].nRoadLimitSpeed = 40
  sm['carrotMan'].carrotCmdIndex = 7
  sm['carrotMan'].carrotCmd = 'SPEED'
  sm['carrotMan'].carrotArg = '30'
  value._update_carrot_man(sm)
  assert (value.desiredSpeed, value.nRoadLimitSpeed, value.carrot_cmd) == (30, 40, 'SPEED')

  sm.recv_time['carrotMan'] = 98.9
  value._update_carrot_man(sm)
  assert (value.desiredSpeed, value.nRoadLimitSpeed, value.carrot_cmd, value.carrot_arg) == (250, 0, '', '')
  assert value.carrot_cmd_index == 7  # Do not replay a previously consumed command.

  sm.recv_time['carrotMan'] = 99.95
  value._update_carrot_man(sm)
  assert value.desiredSpeed == 30
