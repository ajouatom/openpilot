import gzip
import json

import numpy as np
import pytest

from tools.signal_analysis import replay_color_experiment as replay


@pytest.mark.parametrize('motion', [False, True])
@pytest.mark.parametrize('pose', [False, True])
def test_delayed_result_joins_previous_segments_exact_encoded_frame(tmp_path, monkeypatch, motion, pose):
  routes = tmp_path/'routes'
  motion_dir = tmp_path/'motion'
  motion_dir.mkdir()
  pose_dir = tmp_path/'pose'
  pose_dir.mkdir()
  for number in [0, 1]:
    path = routes/f'route--id--{number}'
    path.mkdir(parents=True)
    (path/'fcamera.hevc').touch()
    ids = [0, 1] if number == 0 else [2, 3]
    frames = [0] if number == 0 else [1, 2]
    data = dict(roadEncodeIdx=[[0, dict(frameId=f, segmentId=i)] for i, f in enumerate(ids)],
                daytime=[[0, dict(frame_id=f, timestamp_eof=int((1+f*.2)*1e9), session='camera', result_age_ms=100)] for f in frames])
    (path/'decoded.json.gz').write_bytes(gzip.compress(json.dumps(data).encode()))
    samples = [dict(timestamp=.4+i*.1, angle=0., rate=0., speed=5., valid=True) for i in range(10)] if number == 0 else [dict(timestamp=1.4, angle=30., rate=0., speed=5., valid=True)]
    (motion_dir/(path.name+'.json.gz')).write_bytes(gzip.compress(json.dumps(samples).encode()))
    poses = [dict(timestamp=s['timestamp'], sample_time=s['timestamp']-.02, yaw=0. if s['angle']==0 else 3.2, yaw_std=.8, valid=True) for s in samples]
    (pose_dir/(path.name+'.json.gz')).write_bytes(gzip.compress(json.dumps(poses).encode()))

  class Capture:
    def __init__(self, path): self.index = 0
    def grab(self):
      self.index += 1
      return self.index <= 2
    def retrieve(self): return True, np.zeros((8, 8, 3), np.uint8)
    def release(self): pass

  class Tracker:
    def __init__(self, **kwargs): self.times = []
    def process(self, image, timestamp, **kwargs):
      self.times.append(timestamp)
      return dict(tracks=[], times=self.times[:], **kwargs)

  monkeypatch.setattr(replay.cv2, 'VideoCapture', Capture)
  monkeypatch.setattr(replay, 'SignalTracker', Tracker)
  output = tmp_path/'output'
  summary = replay.run(routes, output, motion_dir if motion else None, pose_dir if pose else None)
  assert [s['samples'] for s in summary] == [2, 1]
  assert json.loads((output/'unmapped.json').read_text()) == []
  rows = [json.loads(s) for s in gzip.open(output/'route--id--1.jsonl.gz', 'rt')]
  assert rows[0]['index'] == 0 and rows[0]['frame_id'] == 2
  assert rows[0]['baseline']['times'] == pytest.approx([1., 1.2, 1.4])
  if motion:
    earlier = [json.loads(s) for s in gzip.open(output/'route--id--0.jsonl.gz', 'rt')]
    assert all(r['motion_allows_green'] for r in earlier)
    assert not rows[0]['motion_allows_green']
    assert not rows[0]['motion_candidate']['allow_green']
  if pose:
    earlier = [json.loads(s) for s in gzip.open(output/'route--id--0.jsonl.gz', 'rt')]
    assert all(r['pose_allows_green'] for r in earlier)
    assert not rows[0]['pose_allows_green']
    assert not rows[0]['pose_candidate']['allow_green']
