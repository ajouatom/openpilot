import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.radar import effective_radar_track_mode


class EndOfInputs(Exception):
  pass


def run_planner_events(mocker, brand, configured_mode, radar_period_ms):
  model_times = list(range(0, 1000, 50))
  radar_times = list(range(20, 1000, radar_period_ms))
  events = iter(sorted(set(model_times + radar_times)))
  services = ('modelV2', 'radarState', 'liveTracks')
  radar_state = object()

  class SubMaster:
    def __init__(self):
      self.now = 10.0
      self.seen = dict.fromkeys(services, False)
      self.updated = dict.fromkeys(services, False)
      self.valid = dict.fromkeys(services, True)
      self.alive = dict.fromkeys(services, True)
      self.recv_time = dict.fromkeys(services, 0.0)
      self.logMonoTime = dict.fromkeys(services, 0)

    def update(self):
      try:
        offset_ms = next(events)
      except StopIteration:
        raise EndOfInputs from None
      self.now = 10.0 + offset_ms / 1000.0
      for service in services:
        updated = offset_ms in (radar_times if service == 'liveTracks' else model_times)
        self.updated[service] = updated
        if updated:
          self.seen[service] = True
          self.recv_time[service] = self.now
          self.logMonoTime[service] = 10_000_000_000 + offset_ms * 1_000_000

    def __getitem__(self, service):
      return {
        'selfdriveState': SimpleNamespace(experimentalMode=False),
        'carState': SimpleNamespace(vEgo=10.0),
        'radarState': radar_state,
      }.get(service, SimpleNamespace())

    def all_checks(self, _services):
      return True

  sm = SubMaster()
  sub_master_factory = mocker.Mock(return_value=sm)
  cp = SimpleNamespace(brand=brand, radarUnavailable=False, radarDelay=0.8)
  params = mocker.Mock()
  params.get_int.return_value = configured_mode
  fast_radar = mocker.Mock()
  fast_radar.lead_one_ready.return_value = True
  fast_radar.build.return_value = SimpleNamespace(
    radar_state=radar_state, lead_mask=1, lead_one_track_id=35, lead_one_reason='active',
  )
  planner = mocker.Mock()
  calls = []
  planner.publish.side_effect = lambda *args, **kwargs: calls.append((sm.now, kwargs))
  namespace = {
    'time': SimpleNamespace(monotonic=lambda: sm.now),
    'car': SimpleNamespace(CarParams=object),
    'Params': lambda: params,
    'Priority': SimpleNamespace(CTRL_LOW=0),
    'config_realtime_process': mocker.Mock(),
    'cloudlog': mocker.Mock(),
    'LongitudinalPlanner': mocker.Mock(return_value=planner),
    'LateralPlanner': mocker.Mock(),
    'FastRadarOverlay': mocker.Mock(return_value=fast_radar),
    'RadarStateOverride': lambda original, replacement: original,
    'LaneDepartureWarning': mocker.Mock(),
    'CarrotPlanner': lambda: SimpleNamespace(mode='acc'),
    'effective_radar_track_mode': effective_radar_track_mode,
    'messaging': SimpleNamespace(
      log_from_bytes=lambda value, schema: cp,
      PubMaster=mocker.Mock(),
      SubMaster=sub_master_factory,
      new_message=lambda service: SimpleNamespace(driverAssistance=SimpleNamespace()),
    ),
  }
  # Exercise the production event loop without loading hardware, messaging or
  # the native MPC solver. Only imports and the __main__ invocation are omitted.
  path = Path(__file__).parents[1] / 'plannerd.py'
  tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
  tree.body = [node for node in tree.body if isinstance(node, (ast.Assign, ast.FunctionDef))]
  exec(compile(tree, str(path), 'exec'), namespace)
  with pytest.raises(EndOfInputs):
    namespace['main']()
  assert planner.update.call_count == len(calls)
  return calls, sub_master_factory.call_args.kwargs['poll'], fast_radar


@pytest.mark.parametrize('brand', ('volkswagen', 'toyota', 'honda', 'ford', 'subaru'))
@pytest.mark.parametrize('radar_period_ms', (20, 40, 50))
@pytest.mark.parametrize('configured_mode', (0, 1, 3))
def test_other_brands_plan_at_model_cadence_with_a_stable_radar_lead(mocker, brand, radar_period_ms, configured_mode):
  calls, poll, fast_radar = run_planner_events(mocker, brand, configured_mode, radar_period_ms)

  assert [timestamp for timestamp, _ in calls] == pytest.approx([10.0 + n * 0.05 for n in range(20)])
  assert poll == 'modelV2'
  assert all(values['planning_trigger'] == 'modelV2' for _, values in calls)
  assert all(values['fast_lead_mask'] == 0 for _, values in calls)
  fast_radar.build.assert_not_called()


@pytest.mark.parametrize('configured_mode', (1, 2, 3))
def test_hyundai_keeps_current_radar_cadence_and_overlay(mocker, configured_mode):
  calls, poll, fast_radar = run_planner_events(mocker, 'hyundai', configured_mode, 50)

  # The first model arrives before radar; subsequent plans use the radar clock.
  expected = [10.0, *(10.0 + ms / 1000.0 for ms in range(70, 1000, 50))]
  assert [timestamp for timestamp, _ in calls] == pytest.approx(expected)
  assert poll == ['modelV2', 'liveTracks']
  assert calls[0][1]['planning_trigger'] == 'modelV2'
  assert all(values['planning_trigger'] == 'liveTracks' for _, values in calls[1:])
  assert fast_radar.build.call_count == len(calls) - 1


@pytest.mark.parametrize('configured_mode', (-2, -1, 0))
def test_hyundai_without_radar_tracks_keeps_model_cadence(mocker, configured_mode):
  calls, poll, fast_radar = run_planner_events(mocker, 'hyundai', configured_mode, 50)

  assert [timestamp for timestamp, _ in calls] == pytest.approx([10.0 + n * 0.05 for n in range(20)])
  assert poll == 'modelV2'
  fast_radar.build.assert_not_called()
