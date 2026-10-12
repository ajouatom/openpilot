import json

from openpilot.selfdrive.carrot.signal_assist_runtime import SignalAssistRuntime, publish_observation
from openpilot.selfdrive.carrot.signal_display import SignalDisplayReader
from openpilot.selfdrive.carrot.tests.test_signal_assist_runtime import SM


def test_identical_packet_still_rechecks_context_and_mode(tmp_path):
  flag, path = tmp_path/'enabled', tmp_path/'observation'
  flag.write_text('1')
  runtime = SignalAssistRuntime(flag, path)
  sm = SM()
  publish_observation({'tracks': []}, 1, 10., 'session', path)
  first, ctx = runtime.read(sm, 10.1)
  assert ctx['valid'] and ctx['enabled']
  repeated, ctx = runtime.read(sm, 10.3)
  assert repeated == first and not ctx['valid']
  sm.recv_time = dict.fromkeys(sm.recv_time, 10.4)
  assert runtime.read(sm, 10.4)[1]['valid']
  flag.write_text('0')
  assert runtime.read(sm, 10.7)[0] is None
  flag.write_text('1')
  # The same file is older than the new activation, even if previously cached.
  assert runtime.read(sm, 11.3)[0] is None


def test_changed_malformed_deleted_and_restored_packets(tmp_path):
  flag, path = tmp_path/'enabled', tmp_path/'observation'
  flag.write_text('1')
  runtime = SignalAssistRuntime(flag, path)
  sm = SM()
  publish_observation({'tracks': []}, 1, 10., 'one', path)
  first = runtime.read(sm, 10.01)[0]
  for bad in ('bad', '[]', 'x'*40000):
    path.write_text(bad)
    assert runtime.read(sm, 10.02)[0] is None
    publish_observation({'tracks': []}, 1, 10., 'one', path)
    assert runtime.read(sm, 10.03)[0] == first
  path.unlink()
  assert runtime.read(sm, 10.04)[0] is None
  publish_observation({'tracks': []}, 2, 10.05, 'two', path)
  assert runtime.read(sm, 10.06)[0]['session'] == 'two'


def test_display_duplicate_bytes_never_renew_previous_color(tmp_path):
  flag, path = tmp_path/'enabled', tmp_path/'display'
  flag.write_text('1')
  reader = SignalDisplayReader(flag, path)
  record = {'version': 1, 'stream': 'road', 'size': [1344,760], 'timestamp': 10., 'frame_id': 100,
            'session': 'one', 'tracks': [{'id': 1, 'box': [600,200,640,210], 'age': 0., 'raw': 'red', 'state': 'red'}]}
  path.write_text(json.dumps(record))
  def camera(t):
    return (100+round((t-10)*20), t, 1344,760,False)
  assert reader.read(camera(10.1), 10.1)[0]['state'] == 'red'
  assert reader.read(camera(10.5), 10.5)[0]['held']
  assert not reader.read(camera(12.1), 12.1)
  # A fresh camera frame cannot resurrect the expired JSON observation.
  assert not reader.read(camera(12.2), 12.2)
  record.update(timestamp=12.3, frame_id=146)
  record['tracks'][0].update(raw='green', state='green')
  path.write_text(json.dumps(record))
  assert reader.read(camera(12.3), 12.3)[0]['state'] == 'green'
