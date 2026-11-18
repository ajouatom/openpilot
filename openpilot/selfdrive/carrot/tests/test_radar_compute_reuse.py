"""Sharing frame-local calculations must preserve association and track history."""
import math
import random
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController
from openpilot.selfdrive.carrot.radar_motion.trajectory_cutin import (
  RADAR_TO_CAMERA_M, TrajectoryCutInDetector, _finite, _vision_supports,
)
from openpilot.selfdrive.carrot.tests.test_radar_motion_predictor import Point, model_with_lead


def original_vision_supports(point, model, precise):
  """Pre-reuse association, including the per-point missing-velocity fallback."""
  for lead in getattr(model, "leadsV3", ()):
    if _finite(getattr(lead, "prob", 0.0)) < (0.90 if precise else 0.35):
      continue
    d_rel = _finite(getattr(lead, "x", (0.0,))[0]) - RADAR_TO_CAMERA_M
    y_rel = -_finite(getattr(lead, "y", (0.0,))[0])
    v_lead = _finite(getattr(lead, "v", (point.v_lead,))[0], point.v_lead)
    x_std = max(1.0, _finite(getattr(lead, "xStd", (1.0,))[0], 1.0))
    y_std = max(0.5, _finite(getattr(lead, "yStd", (0.5,))[0], 0.5))
    v_std = max(1.0, _finite(getattr(lead, "vStd", (1.0,))[0], 1.0))
    if (abs(point.d_rel - d_rel) <= max(4.0, 0.20 * point.d_rel, 0.0 if precise else 2.0 * x_std)
        and abs(point.y_rel - y_rel) <= (1.5 if precise else max(1.5, 2.0 * y_std))
        and abs(point.v_lead - v_lead) <= max(5.0, 2.0 * v_std)):
      return True
  return False


@pytest.mark.parametrize("seed", range(12))
def test_shared_vision_bounds_match_original(seed):
  rng = random.Random(seed)
  for _ in range(25):
    leads = []
    for _ in range(3):
      values = {"prob": rng.choice((0.0, 0.349999, 0.35, 0.899999, 0.9, 1.0, math.nan))}
      for key, low, high in (("x", 0., 40.), ("y", -5., 5.), ("v", -5., 30.),
                             ("xStd", 0., 10.), ("yStd", 0., 3.), ("vStd", 0., 10.)):
        if rng.randrange(4):
          values[key] = [rng.choice((rng.uniform(low, high), math.nan, math.inf, -math.inf))]
      leads.append(SimpleNamespace(**values))
    model = SimpleNamespace(leadsV3=leads)
    prepared = {}
    for _ in range(12):
      point = Point(1, rng.uniform(0., 40.), rng.uniform(-5., 5.), 0., v_lead=rng.uniform(-5., 30.))
      for precise in (False, True):
        assert _vision_supports(point, model, prepared, precise=precise) == original_vision_supports(point, model, precise)


def test_real_model_snapshot_is_frame_local_and_keeps_probability_thresholds():
  model = log.ModelDataV2.new_message()
  model.init("leadsV3", 1)
  lead = model.leadsV3[0]
  lead.prob = 0.9
  lead.x, lead.y, lead.v = [11.52], [0.], [4.]
  lead.xStd, lead.yStd, lead.vStd = [1.], [0.5], [1.]
  point = Point(1, 10., 0., 0., v_lead=4.)
  first = {}
  for precise in (False, True):
    assert _vision_supports(point, model, first, precise=precise) == original_vision_supports(point, model, precise)
  lead.x = [80.]
  assert _vision_supports(point, model, first)
  assert not _vision_supports(point, model, {})
  lead.prob = 0.0
  assert not _vision_supports(point, model, {})


@pytest.mark.parametrize("velocity", (None, math.nan, math.inf, -math.inf))
def test_missing_vision_velocity_falls_back_separately_for_each_point(velocity):
  lead = SimpleNamespace(prob=1., x=[11.52], y=[0.])
  if velocity is not None:
    lead.v = [velocity]
  model = SimpleNamespace(leadsV3=[lead])
  prepared = {}
  for speed in (0., 3., 30.):
    assert _vision_supports(Point(1, 10., 0., 0., v_lead=speed), model, prepared, precise=True)


def test_unused_model_fields_keep_original_lazy_reads():
  unused = SimpleNamespace(prob=1., x=[], y=[], v=[])
  model = SimpleNamespace(leadsV3=[unused])
  assert TrajectoryCutInDetector().update(0., 3., (), ((0., 0.), (100., 0.)), model) == ()
  matching = SimpleNamespace(prob=1., x=[11.52], y=[0.], v=[3.])
  model.leadsV3 = [matching, unused]
  point = Point(1, 10., 0., 0., v_lead=3.)
  prepared = {}
  assert _vision_supports(point, model, prepared)
  assert _vision_supports(point, model, prepared, precise=True)
  unused.prob = .5
  model.leadsV3 = [unused]
  assert not _vision_supports(point, model, {}, precise=True)


@pytest.mark.parametrize("initial_speed,first_slow_frame", ((0.8, 32), (1.5, 49)))
def test_slowing_adjacent_target_keeps_confirmed_risk_and_observations(initial_speed, first_slow_frame):
  controller = DPathRadarController(prefer_corner_radar=True)
  distance = 6.
  outputs = []
  for index in range(81):
    t = index * .05
    speed = max(.4, initial_speed - .8 * max(0., t - 1.2))
    rel = speed - 3.
    distance += rel * .05
    lateral = 2.7 - .6 * t
    points = [Point(1200, distance, lateral, rel, source="corner235", v_lead=speed, yv_rel=-.6),
              Point(40, distance+.2, lateral, rel, source="frontRadar", v_lead=speed, yv_rel=-.6),
              Point(1, 25., 0., 0., source="frontRadar", v_lead=3.)]
    outputs.append(controller.update(t, 3., points, model_with_lead(25., 0., 3.)))
    if index == first_slow_frame:
      assert speed <= .5
      assert outputs[-1].lead_cutin_risk
      assert outputs[-1].leads_cutin
      estimates = controller.trajectory_cutin.last_estimates
      target = next(e for e in estimates if e.point.track_id == 1200)
      assert target.history_s >= 1.4
      assert target.confirmed_cutin and target.predecel_risk
  # Ordinary release still applies; reusing a calculation never extends a latch.
  assert not outputs[-1].lead_cutin_risk
