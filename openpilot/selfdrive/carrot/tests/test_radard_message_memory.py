"""Exercise the production publisher with real Cap'n Proto arenas, without IPC."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log


LIST_FIELDS = {
  'leadsLeft': 'leads_left', 'leadsCenter': 'leads_center',
  'leadsRight': 'leads_right', 'leadsCutIn': 'leads_cutin',
  'leadsLeft2': 'leads_left2', 'leadsRight2': 'leads_right2',
}
LEAD_FIELDS = {
  'leadOne': 'lead_one', 'leadTwo': 'lead_two', 'leadLeft': 'lead_left',
  'leadRight': 'lead_right', 'leadCutInRisk': 'lead_cutin_risk',
}


def load_publisher(source=None):
  # Load the actual methods while excluding device-only Params/IPC imports.
  # The controller output is supplied below; serialization is never mocked.
  path = Path(__file__).parents[1] / 'radar/radard_dpath.py'
  tree = ast.parse(path.read_text(encoding='utf8') if source is None else source)
  names = {'DPathRadarD', 'empty_lead', '_yaw_rate', '_model_measurement_time_s'}
  body = [ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)]
  body += [node for node in tree.body if (
    isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in names
  ) or (isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'EMPTY_LEAD' for t in node.targets))]

  def new_message(service):
    event = log.Event.new_message()
    event.init(service)
    return event

  namespace = {'log': log, 'messaging': SimpleNamespace(new_message=new_message)}
  exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(path), 'exec'), namespace)
  return namespace['DPathRadarD'], namespace['empty_lead']


class Inputs(dict):
  def __init__(self):
    super().__init__(modelV2=SimpleNamespace(timestampEof=1_000_000_000),
                     carState=SimpleNamespace(vEgo=10.0), livePose=SimpleNamespace())
    self.logMonoTime = {'modelV2': 1_050_000_000, 'liveTracks': 1_000_000_000, 'carState': 1_000_000_000}
    self.valid = True

  def all_checks(self):
    return self.valid


def publisher_fixture(source=None):
  cls, empty_lead = load_publisher(source)
  radar = cls.__new__(cls)
  output = SimpleNamespace(**{field: [] for field in LIST_FIELDS.values()},
                           **{field: None for field in LEAD_FIELDS.values()})
  radar.controller = SimpleNamespace(update=lambda **kwargs: output)
  radar.radar_state = log.RadarState.new_message()
  radar.radar_state_valid = False
  return radar, output, Inputs(), car.RadarData.new_message(), empty_lead


@pytest.mark.parametrize('count', (0, 1, 16))
def test_repeated_publication_has_bounded_arena(count):
  radar, output, sm, rr, empty_lead = publisher_fixture()
  lead = dict(empty_lead(), status=True, dRel=23.0, radarTrackId=17)
  for field in LIST_FIELDS.values():
    setattr(output, field, [lead] * count)
  for field in LEAD_FIELDS.values():
    setattr(output, field, lead)
  # Check total allocated segments, not total_size: the latter visits only
  # reachable objects and misses allocations orphaned by list replacement.
  radar.update(sm, rr)
  baseline = sum(map(len, radar.radar_state.to_segments()))
  pm = SimpleNamespace(send=lambda service, message: message.to_bytes())
  for _ in range(2000):
    radar.update(sm, rr)
    radar.publish(pm)
  assert sum(map(len, radar.radar_state.to_segments())) == baseline


def test_publication_replaces_lists_leads_errors_and_validity():
  radar, output, sm, rr, empty_lead = publisher_fixture()
  published = []
  pm = SimpleNamespace(send=lambda service, message: published.append(message.to_bytes()))
  for frame, count in enumerate((3, 0, 1, 5, 0)):
    sm.valid = frame % 2 == 0
    sm.logMonoTime['modelV2'] += 50_000_000
    rr.errors.canError = bool(count)
    lead = dict(empty_lead(), status=True, dRel=float(frame + 10), radarTrackId=frame)
    for field in LIST_FIELDS.values():
      setattr(output, field, [lead] * count)
    for field in LEAD_FIELDS.values():
      setattr(output, field, lead if count else None)
    radar.update(sm, rr)
    radar.publish(pm)
    with log.Event.from_bytes(published[-1]) as event:
      assert event.valid == sm.valid
      assert event.radarState.mdMonoTime == sm.logMonoTime['modelV2']
      assert event.radarState.carStateMonoTime == sm.logMonoTime['carState']
      assert event.radarState.radarErrors.to_dict() == rr.errors.to_dict()
      for field in LIST_FIELDS:
        assert len(getattr(event.radarState, field)) == count
        assert all(item.dRel == lead['dRel'] for item in getattr(event.radarState, field))
      for field in LEAD_FIELDS:
        assert getattr(event.radarState, field).to_dict() == (lead if count else empty_lead())
  with log.Event.from_bytes(published[0]) as first:
    assert first.radarState.leadOne.dRel == 10.0
    assert len(first.radarState.leadsLeft) == 3
