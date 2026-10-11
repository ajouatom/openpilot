import gzip
import json

import numpy as np
import pytest

from tools.signal_analysis import replay_color_experiment as replay


def test_delayed_result_joins_previous_segments_exact_encoded_frame(tmp_path, monkeypatch):
  routes = tmp_path/'routes'
  for number in [0, 1]:
    path = routes/f'route--id--{number}'
    path.mkdir(parents=True)
    (path/'fcamera.hevc').touch()
    ids = [0, 1] if number == 0 else [2, 3]
    frames = [0] if number == 0 else [1, 2]
    data = dict(roadEncodeIdx=[[0, dict(frameId=f, segmentId=i)] for i, f in enumerate(ids)],
                daytime=[[0, dict(frame_id=f, timestamp_eof=int((1+f*.2)*1e9), session='camera', result_age_ms=100)] for f in frames])
    (path/'decoded.json.gz').write_bytes(gzip.compress(json.dumps(data).encode()))

  class Capture:
    def __init__(self, path): self.index = 0
    def grab(self):
      self.index += 1
      return self.index <= 2
    def retrieve(self): return True, np.zeros((8, 8, 3), np.uint8)
    def release(self): pass

  class Tracker:
    def __init__(self, **kwargs): self.times = []
    def process(self, image, timestamp):
      self.times.append(timestamp)
      return dict(tracks=[], times=self.times[:])

  monkeypatch.setattr(replay.cv2, 'VideoCapture', Capture)
  monkeypatch.setattr(replay, 'SignalTracker', Tracker)
  output = tmp_path/'output'
  summary = replay.run(routes, output)
  assert [s['samples'] for s in summary] == [2, 1]
  assert json.loads((output/'unmapped.json').read_text()) == []
  rows = [json.loads(s) for s in gzip.open(output/'route--id--1.jsonl.gz', 'rt')]
  assert rows[0]['index'] == 0 and rows[0]['frame_id'] == 2
  assert rows[0]['baseline']['times'] == pytest.approx([1., 1.2, 1.4])
