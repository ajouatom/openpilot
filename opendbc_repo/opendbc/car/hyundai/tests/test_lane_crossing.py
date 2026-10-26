from types import SimpleNamespace

from opendbc.car.hyundai.lane_crossing import LaneCrossing


def md(frame, left, right, prob=1.0):
  line = lambda y: SimpleNamespace(y=[y])
  return SimpleNamespace(frameId=frame, laneLines=[line(-5.), line(-left), line(right), line(5.)],
                         laneLineProbs=[0., prob, prob, 0.])


def run(track, allowed=True):
  """track: (left, right, prob) per 20 Hz model frame -> side reported on each frame."""
  lc = LaneCrossing()
  return [lc.update(md(i, *f), i * 0.05, allowed) for i, f in enumerate(track)]


def lane_change_left(prob_drop_at=1.4):
  # 1.6/1.8 m in lane, then 0.8 m/s to the left; probs collapse while crossing (10-26 logs)
  track = [(1.6, 1.8, 1.0)] * 40
  for k in range(40):
    left = max(1.6 - 0.04 * k, 0.05)
    track.append((left, 3.4 - left, 1.0 if left > prob_drop_at else 0.02))
  return track


def test_left_crossing_reported_once_before_the_centre_crosses():
  out = run(lane_change_left())
  assert out.count(3) == 1 and out.count(4) == 0
  hit = out.index(3) - 40
  assert 1.6 - 0.04 * hit < 0.9  # tyre on the line, centre still in lane


def test_right_crossing():
  track = [(r, l, p) for l, r, p in lane_change_left()]
  assert run(track).count(4) == 1


def test_lane_keeping_wobble_and_not_allowed_stay_quiet():
  wobble = [(1.7 + 0.15 * ((i // 10) % 2), 1.7 - 0.15 * ((i // 10) % 2), 1.0) for i in range(200)]
  assert not any(run(wobble))
  assert not any(run(lane_change_left(), allowed=False))


def test_relabel_after_crossing_is_not_a_new_crossing():
  # right after a left crossing the crossed line becomes the right line at 0.1 m and the new left line appears at 3.3 m
  track = [(1.7, 1.7, 1.0)] * 40 + [(3.3, 0.1, 0.02)] + [(3.3 - 0.04 * k, 0.1 + 0.04 * k, 0.02) for k in range(30)]
  assert not any(run(track))


def test_road_narrowing_on_one_side_only():
  # left line closes in but the right line does not move away (lane narrowing, merge): no lateral motion of the car
  track = [(1.7, 1.7, 1.0)] * 40 + [(1.7 - 0.04 * k, 1.7, 1.0) for k in range(40)]
  assert not any(run(track))
