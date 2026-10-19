from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.navd.navd import RouteEngine
from openpilot.selfdrive.navd.helpers import Coordinate


@pytest.mark.parametrize('step_index,expected', [(0, [100., 300., 600.]), (1, [0., 200., 500.]),
                                               (2, [-200., 0., 300.])])
@pytest.mark.parametrize('arrival_banner', [False, True])
def test_real_cereal_serializes_mapbox_maneuver_distances(step_index, expected, arrival_banner):
  engine = RouteEngine.__new__(RouteEngine)
  engine.route = []
  engine.route_geometry = []
  for index, (distance, modifier) in enumerate(zip([100., 200., 300.], ['right', 'left', 'straight'], strict=True)):
    banners = [{'distanceAlongGeometry': distance, 'primary': {'text': 'Test road', 'type': 'turn', 'modifier': modifier}}]
    engine.route.append({'distance': distance, 'duration': distance / 10., 'duration_typical': None,
                         'bannerInstructions': [] if index == 2 and not arrival_banner else banners})
    engine.route_geometry.append([Coordinate(37. + index * .01, 127.), Coordinate(37.01 + index * .01, 127.)])
  engine.step_idx = step_index
  engine.last_position = engine.route_geometry[step_index][0]
  engine.localizer_valid = False
  published = []
  engine.pm = SimpleNamespace(send=lambda service, message: published.append((service, message.to_bytes())))

  # Exercise send_instruction, not a substitute serializer. np.cumsum in the
  # nav branch produces numpy.float64, rejected by the device's cereal binding.
  engine.send_instruction()
  assert len(published) == 1 and published[0][0] == 'navInstruction'
  with log.Event.from_bytes(published[0][1]) as event:
    assert event.valid
    maneuvers = event.navInstruction.allManeuvers
    assert [m.distance for m in maneuvers] == pytest.approx(expected if arrival_banner else expected[:2])
    assert [m.modifier for m in maneuvers] == (['right', 'left', 'straight'] if arrival_banner else ['right', 'left'])
    assert event.navInstruction.maneuverDistance == pytest.approx(engine.route[step_index]['distance'])
