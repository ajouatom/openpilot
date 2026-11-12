"""Sharp-turn steerSaturated trigger in selfdrived.update_events, run without device drivers."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
from openpilot.common.realtime import DT_CTRL

ROOT = Path(__file__).resolve().parents[1]
EventName = log.OnroadEvent.EventName


def load_trigger():
  tree = ast.parse((ROOT / 'selfdrived.py').read_text(encoding='utf8'))
  consts = [n for n in tree.body if isinstance(n, ast.Assign) and
            any(isinstance(t, ast.Name) and t.id.startswith('SHARP_TURN_') for t in n.targets)]
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update_events')
  start = next(i for i, n in enumerate(method.body) if isinstance(n, ast.Assign) and
               any(isinstance(t, ast.Name) and t.id == 'sharp_turn' for t in n.targets))
  wrapper = ast.parse('def check(self, lac): pass')
  wrapper.body[0].body = method.body[start:start + 3]
  module = ast.Module(body=consts + wrapper.body, type_ignores=[])
  ns = {'car': car, 'EventName': EventName, 'DT_CTRL': DT_CTRL}
  exec(compile(ast.fix_missing_locations(module), '<sharp turn trigger>', 'exec'), ns)
  return ns['check']


class Events(list):
  @property
  def names(self):
    return self

  def add(self, event):
    self.append(event)


def run(angles, control='angle', active=True, preset=(), brand='hyundai'):
  check = load_trigger()
  sd = SimpleNamespace(CP=car.CarParams(steerControlType=control, brand=brand), sharp_turn_frames=0)
  fired = []
  for angle in angles:
    sd.events = Events(preset)
    sd.sm = {'carControl': SimpleNamespace(actuators=SimpleNamespace(steeringAngleDeg=angle))}
    check(sd, SimpleNamespace(active=active))
    fired.append(sd.events.count(EventName.steerSaturated))
  return fired


@pytest.mark.parametrize("angle", (175., -175., 300.))
def test_request_at_clip_for_a_tenth_of_a_second_fires(angle):
  assert run([angle] * 12) == [0] * 9 + [1] * 3


def test_below_clip_never_fires():
  assert not any(run([174.] * 100))


def test_short_spike_does_not_fire():
  assert not any(run(([180.] * 5 + [0.]) * 10))


@pytest.mark.parametrize("control,active", (("torque", True), ("angle", False)))
def test_torque_cars_and_inactive_lateral_never_fire(control, active):
  assert not any(run([200.] * 50, control=control, active=active))


def test_no_duplicate_when_the_undershoot_check_fired():
  assert run([200.] * 12, preset=[EventName.steerSaturated])[-1] == 1


def test_other_brands_with_other_clips_never_fire():
  assert not any(run([200.] * 50, brand='tesla'))  # Tesla clips at 360: 175 is not its limit
