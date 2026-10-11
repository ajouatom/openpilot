from types import SimpleNamespace
import sys

import numpy as np
import pytest

from openpilot.selfdrive.modeld import signal_neural_shadow as worker


def test_neural_dispatch_requires_explicit_opt_in(monkeypatch, tmp_path):
  from openpilot.selfdrive.modeld import signal_color_shadow, signal_tracking_shadow
  calls = []
  monkeypatch.setattr(worker, 'run', lambda *a: calls.append('neural'))
  monkeypatch.setattr(signal_tracking_shadow, 'run', lambda *a: calls.append('old'))
  (tmp_path / 'tracking_enabled').write_text('1')
  signal_color_shadow.run(tmp_path, 1)
  (tmp_path / 'neural_enabled').write_text('1')
  signal_color_shadow.run(tmp_path, 1)
  assert calls == ['old', 'neural']
  assert worker.backend_requested(tmp_path) == 'cpu'
  (tmp_path / 'neural_backend').write_text('automatic')
  with pytest.raises(ValueError): worker.backend_requested(tmp_path)


@pytest.mark.parametrize('label,score,expected', [('green', .9, 'green'), ('left,red', .8, 'red'),
                                               ('green', .4, 'unknown'), ('yellow', .9, 'unknown')])
def test_current_color_is_display_only(label, score, expected):
  result = worker.display_tracks([dict(id=1, box=[1,2,3,4], label=label, score=score,
                                       source='tracked', neural_age=.1)])[0]
  assert result['evidence']['raw'] == expected
  assert result['state'] == 'unknown'


@pytest.mark.parametrize('reused', [False, True])
def test_live_camera_worker_publishes_display_and_never_control(monkeypatch, tmp_path, reused):
  from tools.signal_analysis import neural_signal
  from openpilot.selfdrive.modeld import signal_tracking_shadow
  from openpilot.selfdrive.carrot import signal_display, signal_assist_runtime
  for flag in ['enabled', 'neural_enabled', 'display_enabled', 'assist_enabled']:
    (tmp_path / flag).write_text('1')
  events, displays = [], []
  class Network:
    DETECTOR_SHA = CLASSIFIER_SHA = 'test'
    def __init__(self, *a, **kw): self.detector = self.classifier = self
    def run(self, *a): return []
    def detect(self, rgb, rect): return [dict(box=[40,30,60,40],score=.9)]
    def classify(self, rgb, box): return dict(label='green', score=.9)
  class Camera:
    frame_id = 4
    def __init__(self, *a): pass
    def is_connected(self): return True
    def recv(self, **kw):
      self.timestamp_eof = worker.time.monotonic_ns()
      (tmp_path / 'neural_enabled').write_text('0')
      return SimpleNamespace(frame_id=5 if reused else 4)
  monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(LOCK_EX=1, LOCK_NB=2, flock=lambda *a: None))
  monkeypatch.setitem(sys.modules, 'msgq.visionipc', SimpleNamespace(
    VisionIpcClient=Camera, VisionStreamType=SimpleNamespace(VISION_STREAM_ROAD=0)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=SimpleNamespace(
    event=lambda n, **r: events.append((n,r)))))
  monkeypatch.setattr(worker, 'configure_worker_scheduling', lambda: None)
  monkeypatch.setattr(worker.time, 'sleep', lambda _: None)
  monkeypatch.setattr(neural_signal, 'NeuralSignal', Network)
  monkeypatch.setattr(signal_tracking_shadow, 'copy_nv12_rgb', lambda _: np.zeros((100,160,3),np.uint8))
  monkeypatch.setattr(signal_display, 'publish_display', displays.append)
  monkeypatch.setattr(signal_assist_runtime, 'publish_observation', lambda *a: pytest.fail('control transport used'))
  worker.run(tmp_path)
  assert len(displays) == (0 if reused else 1)
  if not reused:
    assert displays[0]['tracks'][0]['evidence']['raw'] == 'green'
    assert displays[0]['neural'] and displays[0]['backend'] == 'cpu'
  assert all(not r['control_permission'] and not r['assist_transport'] for _,r in events)
