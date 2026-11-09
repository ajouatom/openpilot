import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import car
from openpilot.selfdrive.ui.stop_object import stop_object_visible, draw_stop_object


class Messages(dict):
  def __init__(self):
    cc = car.CarControl.new_message(enabled=True, longActive=True)
    cc.hudControl.signalStopObject = True
    out = car.CarControl.Actuators.new_message(stopObjectActive=True)
    super().__init__(
      carControl=cc,
      carOutput=NS(actuatorsOutput=out),
      carState=car.CarState.new_message(canValid=True),
      radarState=NS(leadOne=NS(status=False), leadTwo=NS(status=False)),
    )
    self.seen = self.alive = self.valid = dict.fromkeys(self, True)
    self.recv_frame = dict.fromkeys(self, 10)
    self.recv_time = dict.fromkeys(self, 99.95)
    self.logMonoTime = {k: int(99.95e9) for k in self}


def test_cone_requires_applied_packet_not_just_request():
  sm = Messages()
  assert stop_object_visible(sm, now=100.0)
  sm['carOutput'].actuatorsOutput.stopObjectActive = False
  assert not stop_object_visible(sm, now=100.0)


@pytest.mark.parametrize('service', ['carOutput', 'carControl', 'carState', 'radarState'])
@pytest.mark.parametrize('reason', ['old', 'future', 'invalid', 'session'])
def test_cone_expires_on_unhealthy_or_old_session(service, reason):
  sm = Messages()
  if reason == 'old':
    sm.logMonoTime[service] = int(99.7e9)
  elif reason == 'future':
    sm.logMonoTime[service] = int(100.1e9)
  elif reason == 'invalid':
    sm.valid[service] = False
  elif reason == 'session':
    sm.recv_frame[service] = 0
  assert not stop_object_visible(sm, now=100.0, started_frame=1)


@pytest.mark.parametrize('reason', ['lead', 'pedal', 'disabled', 'request_released'])
def test_cone_never_masks_actual_lead_or_survives_departure(reason):
  sm = Messages()
  if reason == 'lead':
    sm['radarState'].leadTwo.status = True
  elif reason == 'pedal':
    sm['carState'].gasPressed = True
  elif reason == 'disabled':
    sm['carControl'].longActive = False
  elif reason == 'request_released':
    sm['carControl'].hudControl.signalStopObject = False
  assert not stop_object_visible(sm, now=100.0)


def test_actual_draw_uses_three_metre_projection_and_cone_not_car(monkeypatch):
  import openpilot.selfdrive.ui.stop_object as module

  monkeypatch.setattr(module.time, 'monotonic', lambda: 100.0)
  calls = []
  rl = NS(
    Color=lambda *v: v, Vector2=lambda x, y: (x, y), draw_triangle=lambda *v: calls.append(('triangle', v)), draw_rectangle=lambda *v: calls.append(('base', v))
  )
  monkeypatch.setitem(sys.modules, 'pyray', rl)
  projected = []
  renderer = NS(
    _path=NS(raw_points=np.array([[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [10.0, 0.0, 0.0]])),
    _path_offset_z=1.2,
    _get_path_length_idx=lambda x, d: 1,
    _rect=NS(x=0, y=0, width=1920, height=1080),
    _map_to_screen=lambda *v: projected.append(v) or (960, 1200),
  )
  draw_stop_object(renderer, Messages())
  assert projected == [(3.0, 0.0, 1.2)]
  assert [name for name, _ in calls] == ['base'] + ['triangle'] * 5
  # Marker remains within a cropped camera view even when its ground point is below it.
  for name, values in calls:
    if name == 'triangle':
      assert all(0 <= x <= 1920 and 0 <= y <= 1080 for x, y in values[:3])

  # Actual renderer clipping must not erase a valid close ground projection.
  calls.clear()
  renderer._map_to_screen = lambda *v: None
  renderer._car_space_transform = np.array([[960., 0., 0.], [2000., 0., 0.], [1., 0., 0.]])
  draw_stop_object(renderer, Messages())
  assert len(calls) == 6
  calls.clear()
  renderer._car_space_transform[2, 0] = -1.
  draw_stop_object(renderer, Messages())
  assert not calls
