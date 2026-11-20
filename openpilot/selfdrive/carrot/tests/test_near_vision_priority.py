import pytest

from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController
from openpilot.selfdrive.carrot.tests.test_radar_motion_predictor import Point, model_with_lead


def _run(near_vision_priority: bool, vision_d: float, radar_d: float, frames: int = 20, v_ego: float = 2.0):
  controller = DPathRadarController(enable_radar_tracks=1, near_vision_priority=near_vision_priority)
  output = None
  for index in range(frames):
    model = model_with_lead(vision_d, 0.0, v_ego, probability=0.98)
    point = Point(7, radar_d, 0.0, v_rel=0.0, v_lead=v_ego, trackState=2)
    output = controller.update(index * 0.05, v_ego, (point,), model)
  return output.lead_one


def test_k7_near_camera_lead_wins_over_farther_radar_object():
  # Pull-away seen on 2026-09-30: camera 3.0 m, radar object 5.3 m.
  stock = _run(False, 3.0, 5.3)
  assert stock is not None and stock["radar"] and stock["dRel"] == pytest.approx(5.3)
  near = _run(True, 3.0, 5.3)
  assert near is not None and not near["radar"]
  assert near["dRel"] == pytest.approx(3.0, abs=0.05)


@pytest.mark.parametrize("vision_d,radar_d", (
  (3.0, 4.2),    # within 1.5 m + 10 %: normal radar/camera disagreement
  (8.0, 9.5),
  (15.0, 19.0),  # beyond 10 m the normal matcher rules apply
))
def test_k7_radar_kept_when_close_to_camera_or_far(vision_d, radar_d):
  lead = _run(True, vision_d, radar_d, v_ego=8.0)
  assert lead is not None and lead["radar"]
  assert lead["dRel"] == pytest.approx(radar_d)


def test_radar_closer_than_camera_is_never_replaced():
  lead = _run(True, 6.0, 4.0)
  assert lead is not None and lead["radar"]
  assert lead["dRel"] == pytest.approx(4.0)


def test_low_confidence_camera_does_not_override_radar():
  controller = DPathRadarController(enable_radar_tracks=1, near_vision_priority=True)
  lead = None
  for index in range(20):
    model = model_with_lead(3.0, 0.0, 2.0, probability=0.6)
    lead = controller.update(index * 0.05, 2.0, (Point(7, 5.3, 0.0, v_lead=2.0, trackState=2),), model).lead_one
  assert lead is not None and lead["radar"]
