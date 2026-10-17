from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld.signal_tracking_shadow import (
  comparison_requested, copy_nv12_rgb, new_trackers, next_delay, publish_control_observation, result_fields, tracking_requested,
)


def test_tracking_is_explicit_opt_in(tmp_path):
  assert not tracking_requested(tmp_path)
  (tmp_path / 'tracking_enabled').write_text('1')
  assert tracking_requested(tmp_path)
  (tmp_path / 'tracking_enabled').write_text('0')
  assert not tracking_requested(tmp_path)


def test_comparison_requires_its_own_explicit_opt_in(tmp_path):
  (tmp_path / 'tracking_enabled').write_text('1')
  assert not comparison_requested(tmp_path)
  (tmp_path / 'daytime_comparison_enabled').write_text('1')
  assert comparison_requested(tmp_path)
  (tmp_path / 'daytime_comparison_enabled').write_text('0')
  assert not comparison_requested(tmp_path)


def test_comparison_has_independent_trackers_and_reset_history():
  calls = []
  def factory(**kwargs):
    item = dict(kwargs)
    calls.append(item)
    return item
  legacy, trial = new_trackers(factory, True)
  assert legacy == {} and trial == {'daytime_cores': True}
  assert legacy is not trial
  replacement, replacement_trial = new_trackers(factory, True)
  assert replacement is not legacy and replacement_trial is not trial
  assert new_trackers(factory, False)[1] is None


def test_production_uses_only_one_validated_tracker_even_with_comparison_flag():
  calls = []
  def factory(**kwargs):
    calls.append(kwargs)
    return object()
  main, extra = new_trackers(factory, True, False, single=True)
  assert main is not None and extra is None
  assert calls == [{'daytime_cores': True, 'robust_tracking': True, 'verified_lamps': True}]


@pytest.mark.parametrize('prediction', ['red', 'green', 'unknown'])
def test_comparison_never_writes_control_transport(prediction):
  calls = []
  result = dict(state=prediction, tracks=[])
  publish_control_observation(result, 4, 2., 'test', comparison=True, publisher=lambda *a: calls.append(a))
  assert calls == []
  publish_control_observation(result, 4, 2., 'test', comparison=False, publisher=lambda *a: calls.append(a))
  assert calls == [(result, 4, 2., 'test')]


@pytest.mark.parametrize('age', [-1, 201, float('nan')])
def test_stale_output_cannot_be_green(age):
  result = result_fields({'state': 'green', 'reason': 'agreeing_visible_tracks'}, age)
  assert result['prediction'] == 'unknown' and not result['fresh'] and not result['control_permission']


@pytest.mark.parametrize('wall,cpu', [(.01, .01), (.08, .04), (.3, .2)])
def test_cpu_and_rate_budget(wall, cpu):
  delay = next_delay(wall, cpu)
  assert wall + delay >= .05
  assert cpu / (wall + delay) <= .5


def test_preemption_counts_towards_cpu_rest_without_catchup():
  assert next_delay(.08, .03) == 0
  assert next_delay(.04, .03) == pytest.approx(.02)
  assert next_delay(.01, .01) == pytest.approx(.04)
  assert next_delay(.5, .1) == 0


def test_two_plane_matches_packed_conversion_with_random_padding():
  import cv2
  rng = np.random.default_rng(16)
  w, h, stride = 1344, 760, 1408
  offset = stride*h+128
  raw = rng.integers(0, 256, offset+stride*h//2, dtype=np.uint8)
  packed = np.concatenate((raw[:stride*h].reshape(h, stride)[:, :w],
                           raw[offset:].reshape(h//2, stride)[:, :w]), axis=0)
  expected = cv2.cvtColor(packed, cv2.COLOR_YUV2RGB_NV12)
  frame = SimpleNamespace(width=w, height=h, stride=stride, uv_offset=offset, data=raw)
  result = copy_nv12_rgb(frame)
  np.testing.assert_array_equal(result, expected)
  raw[:] = 0
  np.testing.assert_array_equal(result, expected)


def test_nv12_padding_and_no_input_mutation():
  width, height, stride = 1344, 760, 1408
  offset = stride * height + 128
  raw = np.full(offset + stride * height // 2, 99, np.uint8)
  y = raw[:stride * height].reshape(height, stride)
  uv = raw[offset:].reshape(height // 2, stride)
  y[:, :width] = 16
  y[:, width // 2:width] = 235
  uv[:, :width] = 128
  before = raw.copy()
  frame = SimpleNamespace(width=width, height=height, stride=stride, uv_offset=offset, data=raw)
  rgb = copy_nv12_rgb(frame)
  assert rgb.shape == (height, width, 3)
  assert (rgb[:, :width // 2] == 0).all() and (rgb[:, width // 2:] == 255).all()
  np.testing.assert_array_equal(raw, before)
  frame.data = raw[:-1]
  with pytest.raises(ValueError, match='truncated'):
    copy_nv12_rgb(frame)
  frame.width = 1008
  with pytest.raises(ValueError, match='geometry'):
    copy_nv12_rgb(frame)


def test_existing_worker_dispatches_only_with_opt_in(monkeypatch, tmp_path):
  from openpilot.selfdrive.modeld import signal_color_shadow, signal_tracking_shadow
  calls = []
  (tmp_path / 'tracking_enabled').write_text('1')
  monkeypatch.setattr(signal_tracking_shadow, 'run', lambda directory, duration: calls.append((directory, duration)))
  signal_color_shadow.run(tmp_path, 2)
  assert calls == [(tmp_path, 2)]


@pytest.mark.parametrize('single', [False, True])
@pytest.mark.parametrize('start_enabled', [False, True])
@pytest.mark.parametrize('comparison,revalidated,override', [
  (True,False,None),(True,True,None),(False,False,None),(False,True,None),(False,False,True),(False,True,True),
])
def test_worker_modes_select_engines_and_bound_transport(monkeypatch, tmp_path, comparison, revalidated, override, start_enabled, single):
  import sys
  from openpilot.selfdrive.modeld import signal_tracking_shadow as worker
  from openpilot.selfdrive.carrot import signal_assist_runtime
  from openpilot.selfdrive.carrot import signal_display
  from tools.signal_analysis import signal_tracker
  from openpilot.selfdrive.carrot import signal_mode
  if single:
    monkeypatch.setattr(worker, 'DIRECTORY', tmp_path)
    monkeypatch.setattr(signal_mode, 'signal_mode', lambda: 2)

  for name in ['enabled', 'tracking_enabled']:
    (tmp_path / name).write_text('1')
  (tmp_path / 'daytime_comparison_enabled').write_text(str(int(comparison)))
  (tmp_path / 'revalidated_enabled').write_text(str(int(revalidated)))
  (tmp_path / 'display_enabled').write_text('1')
  (tmp_path / 'start_guard_enabled').write_text(str(int(start_enabled)))
  from openpilot.selfdrive.carrot import signal_start_guard
  monkeypatch.setattr(signal_start_guard, 'publish_start_observation', lambda *a: None)
  rgb = object()
  calls, events, published = [], [], []
  displays = []
  class Tracker:
    def __init__(self, **kwargs):
      self.trial = kwargs.get('daytime_cores', False)
      self.robust = kwargs.get('robust_tracking', False)
    def process(self, image, timestamp):
      assert image is rgb
      calls.append((self.trial, timestamp, self.robust))
      return dict(state='green' if self.trial else 'red', reason='test', tracks=[])
  class Camera:
    frame_id = 7
    def __init__(self, *args):
      pass
    def is_connected(self):
      return True
    def recv(self, **kwargs):
      self.timestamp_eof = worker.time.monotonic_ns()
      (tmp_path / 'tracking_enabled').write_text('0')
      return SimpleNamespace(frame_id=self.frame_id)
  monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(LOCK_EX=1, LOCK_NB=2, flock=lambda *a: None))
  monkeypatch.setitem(sys.modules, 'msgq.visionipc', SimpleNamespace(
    VisionIpcClient=Camera, VisionStreamType=SimpleNamespace(VISION_STREAM_ROAD=0)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=SimpleNamespace(
    event=lambda name, **record: events.append((name, record)), exception=lambda *a: pytest.fail('publish failed'))))
  monkeypatch.setattr(worker, 'configure_worker_scheduling', lambda: None)
  monkeypatch.setattr(worker, 'copy_nv12_rgb', lambda frame: rgb)
  monkeypatch.setattr(worker.time, 'sleep', lambda seconds: None)
  monkeypatch.setattr(signal_tracker, 'SignalTracker', Tracker)
  monkeypatch.setattr(signal_assist_runtime, 'publish_observation', lambda *a: published.append(a))
  monkeypatch.setattr(signal_display, 'publish_display', lambda record: displays.append(record))
  worker.run(tmp_path, comparison_override=override)
  comparison = comparison if override is None else override
  if single:
    comparison, revalidated = True, True
  assert [c[0] for c in calls] == ([True] if single else [False, True] if comparison else [revalidated])
  assert [c[2] for c in calls] == ([True] if single else [False, revalidated] if comparison else [revalidated])
  assert len(published) == (0 if comparison else 1)
  assert len(displays) == 1
  assert displays[0]['prediction'] == ('green' if comparison or revalidated else 'red')
  records = {name: record for name, record in events}
  assert records['signalTrackingShadow']['prediction'] == ('green' if single or revalidated and not comparison else 'red')
  if comparison and not single:
    assert calls[0][1] == calls[1][1]
    assert records['signalTrackingDaytimeShadow']['prediction'] == 'green'
  assert all(r['assist_transport'] == (not comparison) and not r['control_permission'] for r in records.values())
  assert all(r['revalidated'] == revalidated for r in records.values())
  assert displays[0]['start_guard_transport'] == start_enabled
  assert (tmp_path / 'daytime_comparison_latest.json').exists() == (comparison and not single)
