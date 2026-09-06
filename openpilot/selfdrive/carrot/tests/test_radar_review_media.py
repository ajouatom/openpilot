import pytest

from openpilot.selfdrive.carrot.radar.tools.radar_review_media import camera_frame_times, plan_review_clip


def indexes(count=1200):
  return [dict(segmentNum=1, segmentId=i, timestampEof=63_142_542_557_381 + i * 50_000_000) for i in range(count)]


def test_review_clip_uses_exposure_index_instead_of_120_fps_container_guess():
  times = camera_frame_times(indexes())
  plan = plan_review_clip(times, 12.85, 22.0)
  assert plan.fps == 20.0
  assert plan.first_frame == 257
  assert plan.end_frame == 441
  assert len(plan.source_times_s) == 184
  assert plan.duration_s == pytest.approx(9.2)
  assert plan.source_times_s[80] == pytest.approx(16.85)


def test_review_clip_keeps_exact_timestamps_when_camera_frames_are_dropped():
  values = indexes(10)
  for value in values[5:]:
    value['timestampEof'] += 50_000_000
  times = camera_frame_times(values)
  plan = plan_review_clip(times, 0.0, 0.5)
  assert plan.fps == 20.0
  assert plan.source_times_s[5] == pytest.approx(0.3)
  assert plan.source_times_s[-1] == pytest.approx(0.5)
  assert plan.duration_s == pytest.approx(0.5)


@pytest.mark.parametrize('kind', ('missing', 'duplicate', 'other_segment', 'nonmonotonic'))
def test_review_rejects_unreliable_camera_index(kind):
  values = indexes(10)
  if kind == 'missing':
    values.pop(3)
  elif kind == 'duplicate':
    values.append(values[3])
  elif kind == 'other_segment':
    values[3]['segmentNum'] = 2
  else:
    values[3]['timestampEof'] = values[2]['timestampEof']
  with pytest.raises(ValueError):
    camera_frame_times(values)


@pytest.mark.parametrize('bounds', ((-2, -1), (60, 61), (5, 4), (1, float('nan'))))
def test_review_rejects_clip_outside_camera_or_invalid_bounds(bounds):
  with pytest.raises(ValueError):
    plan_review_clip(camera_frame_times(indexes()), *bounds)
