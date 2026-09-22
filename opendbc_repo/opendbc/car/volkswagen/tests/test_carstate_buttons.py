from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.values import CAR

ButtonType = structs.CarState.ButtonEvent.Type


@pytest.fixture(params=[
  (CAR.VOLKSWAGEN_PASSAT_NMS, False),
  (CAR.VOLKSWAGEN_GOLF_MK7, False),
  (CAR.VOLKSWAGEN_ID4_MK1, False),
  (CAR.VOLKSWAGEN_ID4_MK1, True),
])
def vw_buttons(request):
  candidate, alternate = request.param
  CP = structs.CarParams(carFingerprint=candidate, flags=int(candidate.config.flags), transmissionType='automatic')
  CS = CarState(CP)
  buttons = CS.CCP.BUTTONS_ALT if alternate else CS.CCP.BUTTONS
  cp = SimpleNamespace(vl={})
  for button in buttons:
    cp.vl.setdefault(button.can_addr, {})[button.can_msg] = 0

  def update(**pressed):
    for name, state in pressed.items():
      button = next(button for button in buttons if button.event_type == getattr(ButtonType, name))
      cp.vl[button.can_addr][button.can_msg] = button.values[0] if state else 0
    events = CS.create_button_events(cp, buttons)
    return [(str(event.type), event.pressed) for event in events], CS.update_button_enable(events)

  return CS, update


@pytest.mark.parametrize('pcm', [False, True])
@pytest.mark.parametrize('physical,logical,can_enable', [
  ('setCruise', 'setCruise', True),
  ('resumeCruise', 'resumeCruise', True),
  ('accelCruise', 'accelCruise', False),
  ('decelCruise', 'decelCruise', False),
])
def test_speed_button_edges_and_physical_engagement(vw_buttons, pcm, physical, logical, can_enable):
  CS, update = vw_buttons
  CS.CP.pcmCruise = pcm
  assert update() == ([], False)
  assert update(**{physical: True}) == ([(logical, True)], False)
  for _ in range(60):
    assert update() == ([], False)
  assert update(**{physical: False}) == ([(logical, False)], can_enable and not pcm)
  assert update() == ([], False)


@pytest.mark.parametrize('physical,logical', [('setCruise', 'decelCruise'), ('resumeCruise', 'accelCruise')])
@pytest.mark.parametrize('release_physical_first', [False, True])
def test_separate_buttons_keep_independent_edges(vw_buttons, physical, logical, release_physical_first):
  _, update = vw_buttons
  assert update(**{physical: True}) == ([(physical, True)], False)
  assert update(**{logical: True}) == ([(logical, True)], False)
  first, last = (physical, logical) if release_physical_first else (logical, physical)
  assert update(**{first: False}) == ([(first, False)], release_physical_first)
  assert update() == ([], False)
  assert update(**{last: False}) == ([(last, False)], not release_physical_first)
  assert update() == ([], False)


@pytest.mark.parametrize('physical,logical', [('setCruise', 'decelCruise'), ('resumeCruise', 'accelCruise')])
def test_button_handoff_preserves_each_physical_event(vw_buttons, physical, logical):
  _, update = vw_buttons
  assert update(**{physical: True}) == ([(physical, True)], False)
  assert update(**{physical: False, logical: True}) == ([(physical, False), (logical, True)], True)
  assert update(**{logical: False}) == ([(logical, False)], False)


@pytest.mark.parametrize('button', ['cancel', 'gapAdjustCruise'])
def test_other_buttons_keep_their_events(vw_buttons, button):
  _, update = vw_buttons
  assert update(**{button: True}) == ([(button, True)], False)
  assert update() == ([], False)
  assert update(**{button: False}) == ([(button, False)], False)
