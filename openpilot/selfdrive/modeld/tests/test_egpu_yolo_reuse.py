import pytest

from openpilot.selfdrive.modeld.egpu_yolo_reuse import frame_permitted, leased, read_session, stationary_permitted


@pytest.mark.parametrize('expires', [0, 100, 99, 106, float('inf'), float('nan'), '103', None])
def test_expired_unbounded_or_invalid_lease_never_enables(expires):
  assert not leased({'expires': expires, 'prepared': True, 'enabled': True}, 100)


def test_prepared_session_can_load_without_enabling_execution():
  session = {'expires': 103, 'prepared': True, 'enabled': False}
  assert leased(session, 100, enabled=False)
  assert not leased(session, 100)
  session['enabled'] = True
  assert leased(session, 100)
  session['prepared'] = False
  assert not leased(session, 100, enabled=False)


@pytest.mark.parametrize('content', ['{', 'null', '[]', '{}', '{"expires": "bad"}'])
def test_incomplete_or_corrupt_session_fails_closed(tmp_path, monkeypatch, content):
  from openpilot.selfdrive.modeld import egpu_yolo_reuse
  target = tmp_path/'session.json'
  monkeypatch.setattr(egpu_yolo_reuse, 'session_path', lambda: target)
  assert not read_session()
  target.write_text(content)
  assert not read_session()


def test_session_mode_is_explicit_validated_and_cannot_promote_stationary_owner(tmp_path, monkeypatch):
  import json
  from openpilot.selfdrive.modeld import egpu_yolo_reuse
  path = tmp_path/'session.json'
  monkeypatch.setattr(egpu_yolo_reuse, 'session_path', lambda: path)
  monkeypatch.setattr(egpu_yolo_reuse, 'camera_time', lambda: 100)
  session = {'expires': 103, 'prepared': True, 'enabled': True}
  path.write_text(json.dumps(session))
  assert read_session(mode='stationary') and not read_session(mode='road_observation')
  session['mode'] = 'road_observation'
  path.write_text(json.dumps(session))
  assert read_session(mode='road_observation') and not read_session(mode='stationary')
  session['mode'] = 'unexpected'
  path.write_text(json.dumps(session))
  assert not read_session()


@pytest.mark.parametrize('gear', ['drive', 'reverse', 'neutral', 'sport', 'low', 'brake', 'eco', 'manumatic'])
@pytest.mark.parametrize('enabled', [False, True])
def test_only_explicit_observation_mode_allows_motion_and_engaged_controls(gear, enabled):
  state = {'fresh': True, 'started': True, 'gear': gear, 'standstill': False, 'speed': 15., 'enabled': enabled,
           'lat_active': enabled, 'long_active': enabled, 'primary_seconds': .035, 'dropped': False}
  assert frame_permitted(mode='road_observation', **state)
  assert not frame_permitted(mode='stationary', **state)
  assert not frame_permitted(mode='invalid', **state)


@pytest.mark.parametrize('override', [{'fresh': False}, {'started': False}, {'gear': 'unknown'}, {'speed': float('nan')},
                                    {'primary_seconds': .06}, {'primary_seconds': float('nan')}, {'dropped': True}])
def test_observation_mode_preserves_freshness_primary_timing_and_drop_guards(override):
  state = {'fresh': True, 'started': True, 'gear': 'drive', 'standstill': False, 'speed': 15., 'enabled': True,
           'lat_active': True, 'long_active': True, 'primary_seconds': .035, 'dropped': False}
  assert not frame_permitted(mode='road_observation', **{**state, **override})


@pytest.mark.parametrize('override', [
  {}, {'fresh': False}, {'started': False}, {'parked': False}, {'standstill': False},
  {'speed': .01}, {'speed': -.01}, {'speed': float('nan')}, {'enabled': True},
  {'lat_active': True}, {'long_active': True}, {'primary_seconds': .06},
  {'primary_seconds': float('nan')}, {'primary_seconds': 0}, {'dropped': True},
])
def test_only_fresh_disabled_parked_frame_with_timely_primary_can_run(override):
  state = {'fresh': True, 'started': True, 'parked': True, 'standstill': True, 'speed': 0., 'enabled': False,
           'lat_active': False, 'long_active': False, 'primary_seconds': .035, 'dropped': False}
  assert stationary_permitted(**{**state, **override}) is (not override)


@pytest.mark.parametrize('permitted,prepared,enabled,expected', [
  (False, True, True, None), (True, False, True, None),
  (True, True, False, 'paused'), (True, True, True, 'run'),
])
def test_session_gates_actual_submission_and_publication_cost_latches_overrun(monkeypatch, permitted, prepared, enabled, expected):
  from types import SimpleNamespace
  import numpy as np
  from openpilot.selfdrive.modeld.egpu_yolo import IdleBudget
  from openpilot.selfdrive.modeld import egpu_yolo_reuse
  clock = [101.27]
  monkeypatch.setattr(egpu_yolo_reuse, 'camera_time', lambda: clock[0])
  monkeypatch.setattr(egpu_yolo_reuse, 'read_session', lambda **kwargs: prepared and (enabled or kwargs.get('enabled') is False))
  runtime = egpu_yolo_reuse.ReuseRuntime.__new__(egpu_yolo_reuse.ReuseRuntime)
  runtime.budget = IdleBudget(.003)
  for frame in range(25):
    runtime.budget.observe(frame, 100+frame*.05, 100+frame*.05+.01)
  runtime.width, runtime.height = 512, 256
  runtime.model_id, runtime.phases = 'test', (.0003, .0017, .0004)
  runtime.names = ['person', 'bicycle']
  runtime.last_publish = runtime.last_execution = 0.
  submissions, publications = [], []

  def infer():
    submissions.append(True)
    clock[0] += .003
    return []

  def send(packet):
    publications.append(packet[0])
    # Account for publication time, including a stall after inference returned.
    clock[0] = 101.32

  runtime.infer = infer
  runtime.output = SimpleNamespace(send=send)
  runtime.after_publish(None, 25, 101250000000, 101255000000, 101.26, 101.27, False,
                        'road', np.eye(3), (1344, 760), lambda: False, 101.26, 101.27, permitted=permitted)
  assert bool(submissions) is (expected == 'run')
  assert [m['state'] for m in publications] == ([] if expected is None else [expected])
  assert runtime.budget.overruns == int(expected == 'run')
  if expected == 'run':
    assert runtime.budget.admit(102) == 'overrun'

@pytest.mark.parametrize('model_id,expected_runs', [('signal-v33-observe-s260911', 0), ('regular-yolo', 1)])
def test_signal_only_limits_actual_submissions_to_five_hz(monkeypatch, model_id, expected_runs):
  from types import SimpleNamespace
  import numpy as np
  from openpilot.selfdrive.modeld.egpu_yolo import IdleBudget
  from openpilot.selfdrive.modeld import egpu_yolo_reuse
  monkeypatch.setattr(egpu_yolo_reuse, 'camera_time', lambda: 101.27)
  monkeypatch.setattr(egpu_yolo_reuse, 'read_session', lambda **kwargs: True)
  runtime = egpu_yolo_reuse.ReuseRuntime.__new__(egpu_yolo_reuse.ReuseRuntime)
  runtime.budget = IdleBudget(.003)
  for frame in range(25):
    runtime.budget.observe(frame, 100+frame*.05, 100+frame*.05+.01)
  runtime.budget.last_run = 101.1
  runtime.model_id, runtime.phases = model_id, (0., 0., 0.)
  runtime.names = ['red_visible', 'green_visible']
  runtime.last_publish = runtime.last_execution = 0.
  submissions = []
  runtime.infer = lambda: submissions.append(True) or []
  runtime.output = SimpleNamespace(send=lambda packet: None)
  runtime.after_publish(None, 25, 101250000000, 101255000000, 101.26, 101.27, False,
                        'road', np.eye(3), (1344, 760), lambda: False, 101.26, 101.27, permitted=True)
  assert len(submissions) == expected_runs
