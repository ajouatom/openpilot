from dataclasses import replace

import pytest

from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController
from openpilot.selfdrive.carrot.radar_motion.primary import VisionRadarMatcher, snapshot_radar_points
from openpilot.selfdrive.carrot.tests.test_radar_motion_predictor import Point, model_with_lead


@pytest.mark.parametrize("mode", (-1, 0))
@pytest.mark.parametrize("side", (-1, 1))
def test_scc_only_curve_release_and_reacquisition(mode, side):
  controller = DPathRadarController(enable_radar_tracks=mode)
  model = model_with_lead(26.0, side * 4.0, 12.0, probability=0.98)
  model.position.y = (0.0, -side * 20.0)
  scc = Point(0, 28.0, 0.0, v_rel=-3.0, source="scc")
  for i, points in enumerate(((scc,), (), (), (replace(scc, y_rel=side * 50.0),))):
    lead = controller.update(i * 0.05, 17.0, points, model).lead_one
    assert lead is not None and lead["status"]
    assert lead["radar"] == bool(points)
    assert lead["dRel"] == pytest.approx(28.0 if points else 26.0)
    assert lead["yRel"] == (0.0 if points else side * 4.0)
    if points:
      assert lead["dPath"] == lead["vLat"] == 0.0


@pytest.mark.parametrize("mode", (-1, 0))
def test_scc_dropout_vision_probability_hold_is_bounded(mode):
  controller = DPathRadarController(enable_radar_tracks=mode)
  model = model_with_lead(25.0, 3.0, 8.0, probability=0.39)
  assert controller.update(0.0, 10.0, (), model).lead_one is None
  model.leadsV3[0].prob = 0.4
  assert controller.update(0.05, 10.0, (), model).lead_one is not None
  model.leadsV3[0].prob = 0.36
  for i in range(10):
    assert controller.update(0.1 + i * 0.05, 10.0, (), model).lead_one is not None
  assert controller.update(0.6, 10.0, (), model).lead_one is None
  model.leadsV3[0].prob = 0.0
  assert controller.update(0.65, 10.0, (), model).lead_one is None


@pytest.mark.parametrize("mode", (-1, 0, 2, 3))
@pytest.mark.parametrize("scc_y", (-50.0, 0.0, 50.0))
def test_scc_association_never_uses_reported_lateral_position(mode, scc_y):
  model = model_with_lead(30.0, 4.0, 2.0, probability=0.99)
  model.position.y = (0.0, -12.0)
  controller = DPathRadarController(enable_radar_tracks=mode)
  point = Point(0, 30.0, scc_y, v_rel=-8.0, source="scc")
  lead = controller.update(0.0, 10.0, (point,), model).lead_one
  assert lead is not None and lead["radar"] and lead["radarTrackId"] == 0
  assert lead["yRel"] == lead["dPath"] == lead["vLat"] == 0.0


@pytest.mark.parametrize("mode", (-2, 1))
def test_disabled_scc_is_not_selected_even_without_front_points(mode):
  controller = DPathRadarController(enable_radar_tracks=mode)
  model = model_with_lead(30.0, 0.0, 12.0, probability=0.0)
  point = Point(0, 30.0, 0.0, source="scc")
  assert controller.update(0.0, 10.0, (point,), model).lead_one is None


@pytest.mark.parametrize("mode", (-1, 0, 2, 3))
def test_unmeasured_or_stale_scc_is_not_retained(mode):
  controller = DPathRadarController(enable_radar_tracks=mode)
  model = model_with_lead(30.0, 0.0, 2.0, probability=0.0)
  point = Point(0, 30.0, 0.0, v_rel=-8.0, source="scc", measured=False)
  assert controller.update(0.0, 10.0, (point,), model).lead_one is None
  point = replace(point, measured=True)
  assert controller.update(0.1, 10.0, (point,), model, radar_to_model_time_s=0.3).lead_one is None


def test_scc_does_not_provide_geometric_stationary_evidence():
  model = model_with_lead(30.0, 0.0, 0.0, probability=0.0)
  matcher = VisionRadarMatcher()
  points = snapshot_radar_points((Point(0, 30.0, 0.0, v_rel=-10.0, source="scc"),), 10.0)
  for i in range(30):
    assert matcher.match(model, points, ((0.0, 0.0), (100.0, 0.0)), time_s=i * 0.05) is None


@pytest.mark.parametrize("distance,speed", ((80.0, 2.0), (30.0, -20.0)))
def test_mode_two_scc_still_requires_longitudinal_vision_agreement(distance, speed):
  controller = DPathRadarController(enable_radar_tracks=2)
  point = Point(0, distance, 0.0, v_rel=speed - 10.0, source="scc")
  model = model_with_lead(30.0, 0.0, 2.0, probability=0.99)
  for i in range(20):
    lead = controller.update(i * 0.05, 10.0, (point,), model).lead_one
    assert lead is not None and not lead["radar"]
