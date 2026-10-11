from types import SimpleNamespace
import numpy as np
import pytest

from tools.signal_analysis.neural_signal import NeuralSignalSequence, copy_nv12_crop_rgb


def test_padded_nv12_crop_exactly_matches_full_conversion():
  rng = np.random.default_rng(41)
  frame = SimpleNamespace(width=96, height=64, stride=112, uv_offset=112*68,
                          data=rng.integers(0, 256, 112*100, dtype=np.uint8))
  full = copy_nv12_crop_rgb(frame, (0, 0, 96, 64))
  crop = copy_nv12_crop_rgb(frame, (20, 12, 76, 46))
  np.testing.assert_array_equal(crop, full[12:46, 20:76])


@pytest.mark.parametrize('rect', [(1, 0, 20, 20), (0, 3, 20, 20), (-2, 0, 20, 20), (0, 0, 98, 20), (0, 0, 0, 20)])
def test_invalid_crop_rejected(rect):
  frame = SimpleNamespace(width=96, height=64, stride=96, uv_offset=96*64, data=bytes(96*96))
  with pytest.raises(ValueError): copy_nv12_crop_rgb(frame, rect)


def test_tracking_reads_current_color_and_expires_without_redetection():
  class Network:
    def __init__(self): self.calls = 0
    def detect(self, rgb, rect):
      self.calls += 1
      return [dict(box=[40, 30, 60, 40], score=.9)] if self.calls == 1 else []
    def classify(self, rgb, box): return dict(label='green' if rgb[35, 50, 1] else 'red', score=.9)
  network = Network(); observer = NeuralSignalSequence(network)
  rgb = np.random.default_rng(10).integers(30, 180, (100, 160, 3), dtype=np.uint8)
  rgb[30:40, 40:60] = [255, 0, 0]
  assert observer.process(rgb, 1.)['tracks'][0]['label'] == 'red'
  rgb[30:40, 40:60] = [0, 255, 0]
  r = observer.process(rgb, 1.05)
  assert not r['detected'] and r['tracks'][0]['label'] == 'green'
  for t in [1.2, 1.4]: observer.process(rgb, t)
  assert observer.process(rgb, 1.55)['tracks'] == []


def test_camera_gap_clears_previous_object():
  class Network:
    def detect(self, rgb, rect): return []
  observer = NeuralSignalSequence(Network())
  observer.previous_time = 1.; observer.tracks = [{'stale': True}]
  assert observer.process(np.zeros((100, 160, 3), np.uint8), 2.)['tracks'] == []


def test_lost_object_search_uses_ten_hz_without_float_schedule_drift():
  class Network:
    def detect(self, rgb, rect): return []
  observer = NeuralSignalSequence(Network())
  rgb = np.zeros((100, 160, 3), np.uint8)
  frames = [observer.process(rgb, 30.+i*.05) for i in range(9)]
  assert [i for i, r in enumerate(frames) if r['detected']] == [0, 2, 4, 6, 8]
