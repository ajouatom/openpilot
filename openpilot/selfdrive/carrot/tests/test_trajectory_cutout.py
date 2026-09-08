from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.carrot.radar_motion.primary import RadarPointSnapshot, VisionLead
from openpilot.selfdrive.carrot.radar_motion.trajectory_cutout import TrajectoryCutOutTracker
from openpilot.selfdrive.controls.lib.longitudinal_cutout import cutout_obstacle_relief


PATH = ((0.0, 0.0), (100.0, 0.0))
POINT = RadarPointSnapshot(50, "frontRadar", 25.0, 0.0, -1.0, 0.0, 0.0, 14.0, -0.5, 0.0, True, 2)
VISION = VisionLead(0.99, 25.0, 0.0, 14.0, 1.0, 0.2, 1.0)


def step(tracker, t, y, *, side=1, vision_far=True, point_changes=None, **kwargs):
  p = replace(POINT, d_rel=25.0-t, y_rel=side*y, **(point_changes or {}))
  v = replace(VISION, d_rel=60.0 if vision_far else p.d_rel, y_rel=side*y)
  return tracker.update(t, p, p, kwargs.pop("vision", v), kwargs.pop("path", PATH),
                        kwargs.pop("v_ego", 15.0), kwargs.pop("yaw_rate", 0.0))


def seed(tracker, side=1):
  for i in range(11):
    assert step(tracker, i*.05, 0.0, side=side, vision_far=False).confidence == 0.0


def depart(tracker, side=1, **kwargs):
  result = None
  for i in range(1, 17):
    result = step(tracker, .5+i*.05, i*.075, side=side, **kwargs)
  return result


@pytest.mark.parametrize("side", (-1, 1))
def test_confirmed_departure_ramps_but_preserves_measured_lead(side):
  tracker = TrajectoryCutOutTracker()
  seed(tracker, side)
  prediction = depart(tracker, side)
  assert prediction.confidence == pytest.approx(1.0)
  assert prediction.time_s == pytest.approx((2.15-1.2)/1.5)


@pytest.mark.parametrize("change", ("unseeded", "vision_stays", "weak_vision", "nearer_vision", "uncertain_range",
                                    "turn", "path_shift", "unmeasured", "tentative", "stopped", "low_ego", "scc"))
def test_departure_requires_independent_evidence(change):
  tracker = TrajectoryCutOutTracker()
  if change != "unseeded":
    seed(tracker)
  kwargs = {
    "vision_stays": {"vision_far": False},
    "weak_vision": {"vision": replace(VISION, probability=.1, d_rel=60)},
    "nearer_vision": {"vision": replace(VISION, d_rel=10)},
    "uncertain_range": {"vision": replace(VISION, d_rel=30, x_std=20)},
    "turn": {"yaw_rate": .03},
    "path_shift": {"path": ((0.0, 1.0), (100.0, 1.0))},
    "unmeasured": {"point_changes": {"measured": False}},
    "tentative": {"point_changes": {"radar_track_state": 1}},
    "stopped": {"point_changes": {"v_lead": 0.0}},
    "low_ego": {"v_ego": 3.0},
    "scc": {"point_changes": {"source": "scc"}},
  }.get(change, {})
  assert depart(tracker, **kwargs).confidence == 0.0


@pytest.mark.parametrize("change", ("new_id", "gap", "reverse_time", "jump", "reentry", "stopped_lateral", "vision_returns", "invalid_path"))
def test_active_departure_revokes_on_changed_evidence(change):
  tracker = TrajectoryCutOutTracker()
  seed(tracker)
  assert depart(tracker).confidence > 0.0
  kwargs = {}
  t, y = 1.35, 1.275
  if change == "new_id":
    kwargs = {"point_changes": {"track_id": 51}}
  if change == "gap":
    t = 1.6
  if change == "reverse_time":
    t = 1.3
  if change == "jump":
    y = 2.0
  if change == "reentry":
    y = 1.1
  if change == "vision_returns":
    kwargs = {"vision_far": False}
  if change == "invalid_path":
    kwargs = {"path": ()}
  if change == "stopped_lateral":
    for i in range(1, 5):
      result = step(tracker, 1.3+i*.05, 1.2)
  else:
    result = step(tracker, t, y, **kwargs)
  assert result.confidence == 0.0


def test_jitter_and_path_motion_alone_do_not_trigger():
  tracker = TrajectoryCutOutTracker()
  seed(tracker)
  for i in range(1, 21):
    assert step(tracker, .5+i*.05, .5 + .12*(-1)**i).confidence == 0.0
  tracker.reset()
  seed(tracker)
  for i in range(1, 21):
    offset = -i*.05
    assert step(tracker, .5+i*.05, 0, path=((0., offset), (100., offset))).confidence == 0.0


def lead(**changes):
  values = {"status": True, "radar": True, "radarTrackId": 50, "dRel": 25.0, "vRel": -1.0,
                "vLead": 14.0, "aLeadK": -.5, "cutOutTime": 1.0, "cutOutConfidence": 1.0}
  return SimpleNamespace(**(values | changes))


def test_relief_keeps_pre_clearance_obstacle_and_caps_future_headway():
  times = np.array([0., .5, 1., 1.3, 1.5, 1.8, 3., 10.])
  target = lead()
  before = vars(target).copy()
  relief = cutout_obstacle_relief(target, 15., times, 1.45, 6.)
  np.testing.assert_array_equal(relief[:4], np.zeros(4))
  assert relief[4] == pytest.approx(3.0)
  assert relief[-1] == pytest.approx(7.5)
  assert vars(target) == before
  assert cutout_obstacle_relief(target, 15., times, .4, 6.)[-1] == pytest.approx(3.)
  np.testing.assert_allclose(cutout_obstacle_relief(lead(cutOutConfidence=.25), 15., times, 1.45, 6.), relief*.25)


@pytest.mark.parametrize("changes", ({"status": False}, {"radar": False}, {"radarTrackId": -1},
                                    {"cutOutTime": 0.}, {"cutOutTime": 3.}, {"cutOutTime": float("nan")},
                                    {"cutOutConfidence": float("inf")}, {"cutOutConfidence": -1.},
                                    {"dRel": 8., "vRel": -4.}, {"vLead": 0.}, {"aLeadK": -8.}))
def test_relief_rejects_invalid_or_closing_risk(changes):
  np.testing.assert_array_equal(cutout_obstacle_relief(lead(**changes), 15., np.array([0., 2., 5.]), 1.45, 6.), 0.)


def test_old_messages_and_long_stopping_distance_keep_original_control():
  old = lead()
  del old.cutOutTime
  del old.cutOutConfidence
  for target, stopping in ((old, 6.), (lead(), 25.)):
    assert not cutout_obstacle_relief(target, 15., np.array([0., 3., 5.]), 1.45, stopping).any()


def test_radar_schema_roundtrip_and_builder_copy_preserve_cutout_metadata():
  from openpilot.cereal import log
  message = log.RadarState.new_message()
  message.leadOne = vars(lead())
  with log.RadarState.from_bytes(message.to_bytes()) as decoded:
    copied = decoded.as_builder()
    copied.leadOne.dRel = 24.5  # same-track longitudinal refresh
    assert copied.leadOne.cutOutTime == 1.0
    assert copied.leadOne.cutOutConfidence == 1.0


def test_corner_lateral_history_can_confirm_quantized_front_departure():
  tracker = TrajectoryCutOutTracker()
  results = []
  for i in range(31):
    t = i*.05
    y = max(0., t-.5)*1.2
    p = replace(POINT, d_rel=25-t, y_rel=round(y/.3)*.3)
    corner = replace(p, source="corner235", track_id=1356, y_rel=y)
    v = replace(VISION, d_rel=p.d_rel if t <= .5 else 60.)
    results.append(tracker.update(t, p, corner, v, PATH, 15., 0.))
  assert results[-1].confidence == pytest.approx(1.)
  # Losing the independently associated lateral identity requires new evidence.
  assert tracker.update(1.55, p, p, v, PATH, 15., 0.).confidence == 0.


def test_measured_departure_fixture_reaches_relief_only_after_evidence():
  import json
  from pathlib import Path
  fixture = Path(__file__).with_name("fixtures") / "cutout_departure.json"
  tracker = TrajectoryCutOutTracker()
  active = []
  for sample in json.loads(fixture.read_text(encoding="utf-8"))["samples"]:
    sample["point"] = RadarPointSnapshot(**sample["point"]) if sample["point"] else None
    sample["lateral"] = RadarPointSnapshot(**sample["lateral"]) if sample["lateral"] else None
    sample["vision"] = VisionLead(**sample["vision"]) if sample["vision"] else None
    output = tracker.update(**sample)
    if output.confidence:
      active.append((sample["time_s"], output))
  assert active
  assert active[0][0] == pytest.approx(11.40344, abs=.001)
  assert active[0][1].time_s == pytest.approx(.688, abs=.01)
  assert 0.1 < active[0][1].confidence < .2
