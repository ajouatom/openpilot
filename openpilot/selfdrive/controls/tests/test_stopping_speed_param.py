import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.common.stopping_params import get_stopping_speed
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan


class Params:
  def __init__(self, value):
    self.value = value
    self.pending = []

  def get_float(self, key):
    assert key == "VEgoStopping"
    return float(self.value)

  def put_int(self, key, value):
    assert key == "VEgoStopping"
    self.value = value

  def put_int_nonblocking(self, key, value):
    self.pending.append((key, value))


@pytest.mark.parametrize("old", [-1, 0, 1, 2, 5, 9])
def test_startup_repairs_old_value_and_keeps_it_on_next_read(old):
  params = Params(old)
  assert get_stopping_speed(params, blocking=True) == .1
  assert params.value == 10
  assert get_stopping_speed(params) == .1
  assert params.pending == []


@pytest.mark.parametrize("value", [10, 15, 20, 50, 100])
def test_supported_values_remain_adjustable_without_writes(value):
  params = Params(value)
  assert get_stopping_speed(params) == pytest.approx(value * .01)
  assert params.value == value
  assert params.pending == []


def test_live_low_write_cannot_delay_stop_while_persistence_is_pending():
  params = Params(20)
  assert get_stopping_speed(params) == .2
  params.value = 2  # e.g. an old settings client or restored snapshot
  threshold = get_stopping_speed(params)
  assert params.value == 2  # queued writes have not reached storage yet
  assert params.pending == [("VEgoStopping", 10)]
  # Recorded stop-approach targets at the delay and delay+1 s horizons.
  times = np.array([0., .3, 1.3])
  speeds = np.array([.1, .066716544, .030907153])
  accels = np.array([-.13, -.08, 0.])
  assert not get_accel_from_plan(speeds, accels, times, action_t=.3, vEgoStopping=.02)[1]
  assert get_accel_from_plan(speeds, accels, times, action_t=.3, vEgoStopping=threshold)[1]
  # A plan to accelerate again must still prevent an ordinary stop decision.
  speeds[-1] = .2
  assert not get_accel_from_plan(speeds, accels, times, action_t=.3, vEgoStopping=threshold)[1]


@pytest.mark.parametrize("value", [None, "invalid", "nan", "inf", "-inf"])
def test_invalid_value_restores_default(value):
  params = Params(value)
  assert get_stopping_speed(params, blocking=True) == .5
  assert params.value == 50


def test_catalog_minimum_matches_control_and_keeps_default():
  path = Path(__file__).resolve().parents[2] / 'carrot_settings.json'
  data = json.loads(path.read_text(encoding='utf-8'))
  def settings(value):
    if isinstance(value, dict):
      if value.get('name') == 'VEgoStopping':
        yield value
      for child in value.values():
        yield from settings(child)
    elif isinstance(value, list):
      for child in value:
        yield from settings(child)
  setting, = settings(data)
  assert (setting['min'], setting['max'], setting['default'], setting['unit']) == (10, 100, 50, 5)
  assert get_stopping_speed(Params(setting['min'])) == .1
