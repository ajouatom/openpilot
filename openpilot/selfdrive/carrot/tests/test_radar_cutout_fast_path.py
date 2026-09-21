import math
from dataclasses import dataclass

import pytest

from openpilot.selfdrive.carrot.radar_motion.predictor import RadarMotionPredictor


@dataclass(frozen=True)
class Point:
  track_id: int
  d_rel: float
  y_rel: float
  source: str
  v_rel: float = 0.0
  a_rel: float = -0.4
  v_lead: float = 12.0
  a_lead: float = -0.4
  yv_rel: float = 0.0
  measured: bool = True


@pytest.mark.parametrize('source', ['frontRadar', 'corner235', 'corner180'])
@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('curve', [0.0, 0.0015])
def test_cutout_only_preserves_probabilities_and_all_tracking_state(source, side, curve):
  reference = RadarMotionPredictor()
  compact = RadarMotionPredictor(cut_out_only=True)
  path = tuple((float(x), curve * x * x) for x in range(0, 101, 5))
  nonzero = 0
  now = 0.0
  for frame in range(240):
    now += 0.05 if frame % 37 else 0.10
    # Enter, leave, slot swap, dropout, and acquisition of a different lead.
    y = side * (0.03 * max(0, frame - 35) if frame < 105 else 1.2 * math.sin(frame * 0.05))
    ids = (10, 11) if frame < 125 else (11, 10)
    points = [Point(ids[0], 30.0, y, source, yv_rel=side * 0.6),
              Point(ids[1], 48.0, side * 0.3, source)]
    if 145 <= frame < 151:
      points = points[1:]
    requested = {(source, ids[0] if frame < 180 else ids[1])}
    kwargs = {'time_s': now, 'points': points, 'path': path, 'v_ego': 12.0,
              'yaw_rate_rad_s': 0.01 if curve else 0.0, 'prediction_identities': requested}
    full = reference.update(**kwargs)
    fast = compact.update(**kwargs)
    assert full.keys() == fast.keys()
    for key in full:
      assert full[key].track_id == fast[key].track_id
      assert full[key].cut_out_probability == fast[key].cut_out_probability
      nonzero += full[key].cut_out_probability > 0.0
    assert reference._states == compact._states
    assert reference._retired_states == compact._retired_states
    assert reference._next_continuity_id == compact._next_continuity_id
  if source == 'frontRadar' and not curve:
    assert nonzero > 0
