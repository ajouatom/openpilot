from openpilot.selfdrive.modeld.camera_gap import CameraGapTrace
from openpilot.selfdrive.modeld.camera_sync import receive_camera_pair
from openpilot.selfdrive.modeld.tests.test_camera_sync import Camera


def test_recorded_camera_pair_is_complete_if_both_frames_reach_the_reader():
  # Owner N16 drive: both camera states contain frame 18031. Its 11 us skew
  # is within the existing 20 ms rule; do not relax that rule to hide the gap.
  main = [(18030, 13129977409028), (18031, 13130030963101), (18032, 13130077419154)]
  wide = [(18030, 13129977419861), (18031, 13130030974403), (18032, 13130077430352)]
  for traced in (False, True):
    a, b = Camera(main), Camera(wide)
    ids = [receive_camera_pair(a, b, trace=[] if traced else None)[1].frame_id for _ in range(3)]
    assert ids == [18030, 18031, 18032]


def test_trace_distinguishes_queue_gap_from_pair_resync(monkeypatch):
  calls = []
  clock = [10.]
  monkeypatch.setattr('openpilot.selfdrive.modeld.camera_gap.time.monotonic', lambda: clock[0])
  recorder = CameraGapTrace(lambda *a, **kw: calls.append(kw))
  main = Camera([(1, 50_000_000), (3, 150_000_000)])
  wide = Camera([(1, 50_000_000), (2, 100_000_000), (3, 150_000_000)])
  assert recorder.receive(main, wide)[1].frame_id == 1
  recorder.record(1, 0, 'JoiningModel', loop=50.)
  assert not calls
  assert recorder.receive(main, wide)[1].frame_id == 3
  recorder.record(3, 1, 'JoiningModel', loop=80.)
  assert len(calls) == 1
  row = calls[0]
  assert row['previous']['frame_id'] == 1 and row['previous']['timings_ms']['loop'] == 50.
  assert [(v['camera'], v['frame_id']) for v in row['current']['received']] == [('main', 3), ('extra', 2), ('extra', 3)]
  for frame in range(4, 100):
    recorder.record(frame, 1, 'JoiningModel', loop=80.)
  assert len(calls) == 1 and recorder.previous['frame_id'] == 99
  clock[0] += 1
  recorder.record(100, 1, 'JoiningModel', loop=80.)
  assert len(calls) == 2 and calls[-1]['previous']['frame_id'] == 99


def test_trace_is_bounded_and_logging_failure_cannot_stop_inference():
  def fail(*args, **kwargs):
    raise OSError('logger unavailable')
  recorder = CameraGapTrace(fail)
  main = Camera((i, i * 100_000_000) for i in range(100))
  wide = Camera((i, i * 100_000_000 + 50_000_000) for i in range(100))
  assert recorder.receive(main, wide) is None
  assert len(recorder.received) <= 12
  recorder.record(20, 1, 'JoiningModel', loop=100.)
  assert recorder.previous['frame_id'] == 20
