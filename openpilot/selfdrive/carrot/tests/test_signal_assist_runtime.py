import json
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.signal_assist_runtime import SignalAssistRuntime, publish_observation


class SM(dict):
  def __init__(self):
    super().__init__(carState=NS(canValid=True, gearShifter='drive'), selfdriveState=NS(enabled=True), carControl=NS(longActive=True))
    names = ['carState', 'modelV2', 'radarState', 'selfdriveState', 'carControl']
    self.seen = dict.fromkeys(names, True)
    self.valid = dict.fromkeys(names, True)
    self.alive = dict.fromkeys(names, True)
    self.recv_time = dict.fromkeys(names, 10.)


def test_default_does_not_enable_control(tmp_path):
  runtime = SignalAssistRuntime(tmp_path / 'enable', tmp_path / 'obs')
  assert runtime.assist is None


def test_atomic_round_trip_and_disable(tmp_path):
  flag, path = tmp_path / 'enable', tmp_path / 'obs'
  flag.write_text('1')
  runtime = SignalAssistRuntime(flag, path)
  publish_observation({'tracks': []}, 1, 10., 'worker', path)
  obs, context = runtime.read(SM(), 10.1)
  assert obs['timestamp'] == 10. and context['enabled'] and context['valid']
  flag.write_text('0')
  obs, context = runtime.read(SM(), 10.7)
  assert obs is None and not context['enabled']


@pytest.mark.parametrize('payload', ['invalid', '[]', '{}', 'x' * 40000,
                                    json.dumps(dict(version=2, stream='road', size=[1344, 760]))],
                         ids=['bad_json', 'list', 'missing', 'oversized', 'wrong_version'])
def test_bad_transport_is_unknown(tmp_path, payload):
  flag, path = tmp_path / 'enable', tmp_path / 'obs'
  flag.write_text('1'); path.write_text(payload)
  obs, _ = SignalAssistRuntime(flag, path).read(SM(), 10.1)
  assert obs is None


@pytest.mark.parametrize('condition', ['stale', 'unseen', 'invalid', 'dead', 'park', 'inactive'])
def test_runtime_gates_vehicle_context(tmp_path, condition):
  flag = tmp_path / 'enable'; flag.write_text('1')
  sm = SM()
  if condition == 'stale': sm.recv_time['carState'] = 9.
  elif condition == 'unseen': sm.seen['modelV2'] = False
  elif condition == 'invalid': sm.valid['carState'] = False
  elif condition == 'dead': sm.alive['radarState'] = False
  elif condition == 'park': sm['carState'].gearShifter = 'park'
  elif condition == 'inactive': sm['carControl'].longActive = False
  _, context = SignalAssistRuntime(flag, tmp_path / 'absent').read(sm, 10.1)
  assert not all(context[x] for x in ('valid', 'enabled', 'drive'))


def test_producer_history_survives_transport_and_fresh_consumer_release(tmp_path):
  from tools.signal_analysis.signal_tracker import SignalTracker
  from openpilot.selfdrive.carrot.tests.test_signal_assist import step
  flag, path = tmp_path / 'enable', tmp_path / 'obs'
  flag.write_text('1')
  runtime = SignalAssistRuntime(flag, path)
  tracker = SignalTracker()
  def consume(now):
    sm = SM()
    sm.recv_time = dict.fromkeys(sm.recv_time, now)
    obs, context = runtime.read(sm, now)
    context.pop('now')
    return step(runtime.assist, now, obs, **context)
  for i in range(11):
    t = 9. + i * .1
    result = tracker.update(t, [dict(box=[640, 250, 680, 270], raw='red', quality=.8)])
    publish_observation(result, i, t, 'worker', path)
    dec = consume(t+.1)
  assert dec.hold
  for i in range(3):
    t = 10.2 + i * .2
    result = tracker.update(t, [dict(box=[640, 250, 680, 270], raw='green', quality=.8)])
    publish_observation(result, 11+i, t, 'worker', path)
  dec = consume(10.79)
  assert dec.released and not dec.hold
