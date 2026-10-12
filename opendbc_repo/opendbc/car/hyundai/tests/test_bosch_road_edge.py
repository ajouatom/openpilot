from dataclasses import replace
import math
from types import SimpleNamespace as NS

import pytest

from opendbc.car.hyundai.radar_bosch import (BoschModelSnapshot, BoschPhysicalObject, BoschRoadEdgePublicationFilter,
                                             BoschVisionCue)
from opendbc.car.hyundai.radar_interface import RadarInterface


T = 1_000_000_000


def model(stds=(0.5, 0.5), left=(-2.0, -2.0), right=(2.0, 2.0), xs=(0.0, 150.0)):
  return NS(roadEdges=[NS(x=xs, y=left), NS(x=xs, y=right)], roadEdgeStds=stds)


def point(pid=1, d=50.0, y=5.0, **kwargs):
  return BoschPhysicalObject(pid, T, (), 0, d, y, 0.0, False, False, 1, 'test', **kwargs)


def apply(objects, sample=None, **kwargs):
  f = BoschRoadEdgePublicationFilter()
  f.ingest_model(sample or model(), T)
  f.update(objects, T, **kwargs)
  return f, f.publication_view(objects, T)


def test_both_sides_margin_and_no_input_mutation():
  objects = (point(y=5), point(2, y=-5), point(3, y=4), point(4, y=-4), point(5, y=0))
  f, result = apply(objects)
  assert f.would_suppress == {1, 2}
  assert result == objects[2:]
  assert result[0] is objects[2]
  assert objects[0].y_rel == 5 and len(objects) == 5


@pytest.mark.parametrize('sample', [
  model(stds=(1.01, .1)), model(stds=(float('nan'), .1)), model(stds=(-.1, .1)),
  model(xs=(0., 0.)), model(left=(float('nan'), -2.)), model(left=(3., 3.)),
  model(left=(-2.,)), NS(roadEdges=[], roadEdgeStds=[]),
])
def test_bad_geometry_preserves_output(sample):
  objects = (point(),)
  _, result = apply(objects, sample)
  assert result is objects


def test_no_range_extrapolation():
  objects = (point(d=149), point(2, d=-2))
  _, result = apply(objects)
  assert result is objects


def test_camera_origin_offset_on_curved_edge():
  objects = (point(y=9.1), point(2, y=9.3))
  f, result = apply(objects, model(xs=(0., 100.), left=(-2., -12.), right=(2., 12.)))
  assert result == objects[:1]
  assert f.would_suppress == {2}


def test_independent_evidence_protected():
  objects = (replace(point(), oem_selected=True), replace(point(2), vision_supported=True), point(3), point(4), point(5))
  f, result = apply(objects, word0_pids={3}, camera_associations={4: (1, 12), 5: (0, -1)})
  assert f.would_suppress == {5}
  assert result == objects[:4]


def test_stale_future_missing_and_invalid_scan_fail_open():
  objects = (point(),)
  for model_ns, valid in [(T - 200_000_001, True), (T + 1, True), (T, False)]:
    f = BoschRoadEdgePublicationFilter()
    f.ingest_model(model(), model_ns)
    f.update(objects, T, valid=valid)
    assert f.publication_view(objects, T) is objects
  f = BoschRoadEdgePublicationFilter()
  f.update(objects, T)
  assert f.publication_view(objects, T) is objects


def test_latest_uncertain_model_does_not_use_older_good_model():
  f = BoschRoadEdgePublicationFilter()
  f.ingest_model(model(), T - 50_000_000)
  f.ingest_model(model(stds=(2., 2.)), T)
  objects = (point(),)
  f.update(objects, T)
  assert f.publication_view(objects, T) is objects


def test_future_model_cannot_influence_past_scan():
  f = BoschRoadEdgePublicationFilter()
  f.ingest_model(model(), T)
  f.ingest_model(model(left=(-20., -20.), right=(20., 20.)), T + 1)
  f.update((point(),), T)
  assert f.would_suppress == {1}


def test_no_stale_publication_or_reused_pid_suppression():
  objects = (point(),)
  f, _ = apply(objects)
  assert f.publication_view(objects, T + 200_000_001) is objects
  assert f.publication_view(objects, T - 1) is objects
  assert f.publication_view(objects, None) is objects
  newer = (replace(objects[0], timestamp_ns=T + 1),)
  assert f.publication_view(newer, T + 1) == newer
  f.invalidate_model()
  assert f.publication_view(objects, T) is objects


def test_new_scan_releases_old_decision_and_clock_reset_clears_context():
  f, _ = apply((point(),))
  inside = (point(y=0),)
  f.update(inside, T)
  assert f.publication_view(inside, T) is inside
  f.update((replace(point(), timestamp_ns=T-1),), T-1)
  assert not f.would_suppress and not f.model_edges


def test_disabled_keeps_all_objects():
  f = BoschRoadEdgePublicationFilter(enabled=False)
  f.ingest_model(model(), T)
  objects = (point(),)
  f.update(objects, T)
  assert f.publication_view(objects, T) is objects


def test_context_does_not_touch_non_bosch_interface():
  interface = NS(bosch=None)
  RadarInterface.set_bosch_context(interface, T, model=model(), model_ns=T)
  assert vars(interface) == {'bosch': None}


def test_same_timestamp_changed_coordinates_are_revalidated():
  f = BoschRoadEdgePublicationFilter()
  sample = model()
  objects = (point(),)
  f.ingest_model(sample, T)
  f.update(objects, T)
  assert f.would_suppress == {1}
  sample.roadEdges[0].y = (-20., -20.)
  sample.roadEdges[1].y = (20., 20.)
  f.ingest_model(sample, T)
  f.update(objects, T)
  assert f.publication_view(objects, T) is objects


def test_repeated_coordinates_still_read_new_uncertainty_and_invalid_data():
  f = BoschRoadEdgePublicationFilter()
  objects = (point(),)
  f.ingest_model(model(), T)
  f.update(objects, T)
  assert f.would_suppress == {1}
  f.ingest_model(model(stds=(1., 1.)), T)
  f.update(objects, T)
  assert f.publication_view(objects, T) is objects
  f.ingest_model(model(), T)
  f.update(objects, T)
  assert f.would_suppress == {1}
  f.ingest_model(model(left=(float('nan'), -2.)), T)
  assert f.publication_view(objects, T) is objects
  f.update(objects, T)
  assert not f.would_suppress


def test_cached_coordinates_preserve_signed_zero_and_own_input_values():
  f = BoschRoadEdgePublicationFilter()
  sample = model(xs=[-0., 150.])
  f.ingest_model(sample, T)
  before = f.model_edges[-1][1][0][0][0]
  sample.roadEdges[0].x[0] = 0.
  f.ingest_model(sample, T)
  after = f.model_edges[-1][1][0][0][0]
  assert math.copysign(1., before[0]) == -1.
  assert math.copysign(1., after[0]) == 1.


def _context_interface():
  recorder = NS(models=[], poses=[])
  b5 = NS(ingest_pose=lambda ns, yaw: recorder.poses.append((ns, yaw)),
          ingest_model=lambda m, ns: recorder.models.append(('b5', ns)))
  mirror = NS(ingest_model=lambda m, ns: recorder.models.append(('mirror', ns)))
  bosch = NS(road_edge_filter=BoschRoadEdgePublicationFilter(), b5_birth_defer=b5, mirror_m3_shadow=mirror)
  interface = NS(bosch=bosch, _bosch_path_ns=None, _bosch_path=(), _bosch_path_source_ns=0)
  return interface, recorder


def _context_model(x0=0.0):
  sample = model()
  sample.leadsV3 = [NS(x=[30.0], y=[0.5], prob=.9)]
  sample.position = NS(x=[x0, 50.0], y=[0.0, 0.0])
  sample.timestampEof = T - 50_000_000
  return sample


def _pose(z=.1):
  return NS(inputsOK=True, sensorsOK=True, angularVelocityDevice=NS(valid=True, z=z))


def test_future_model_keeps_previous_context_instead_of_invalidating():
  # A modelV2 stamped a few ms after the radar batch receive time is not current
  # yet: the road-edge model, path and cue of the previous model stay in use.
  interface, recorder = _context_interface()
  first = _context_model()
  RadarInterface.set_bosch_context(interface, T, model=first, model_ns=T - 10_000_000)
  now, _, cues, path, path_ns, _ = interface._bosch_context
  assert interface.bosch.road_edge_filter.model_edges and cues and path and path_ns == T - 10_000_000
  RadarInterface.set_bosch_context(interface, T + 10_000_000, model=_context_model(1.0), model_ns=T + 13_000_000)
  _, _, cues, path, path_ns, _ = interface._bosch_context
  assert interface.bosch.road_edge_filter.model_edges          # not invalidated
  assert cues and path and path_ns == T - 10_000_000            # previous model still current
  # The first call that reaches the newer stamp takes it.
  RadarInterface.set_bosch_context(interface, T + 20_000_000, model=_context_model(1.0), model_ns=T + 13_000_000)
  assert interface._bosch_context[4] == T + 13_000_000
  assert recorder.models[-1] == ('mirror', T + 13_000_000)


def test_stale_or_missing_model_still_invalidates():
  interface, _ = _context_interface()
  RadarInterface.set_bosch_context(interface, T, model=_context_model(), model_ns=T - 10_000_000)
  RadarInterface.set_bosch_context(interface, T + 300_000_000, model=_context_model(), model_ns=T - 10_000_000)
  assert not interface.bosch.road_edge_filter.model_edges
  assert interface._bosch_context[2] == () and interface._bosch_context[3] == ()
  interface, _ = _context_interface()
  RadarInterface.set_bosch_context(interface, T, model=None, model_ns=0)
  assert not interface.bosch.road_edge_filter.model_edges


def test_future_pose_keeps_previous_yaw():
  interface, recorder = _context_interface()
  RadarInterface.set_bosch_context(interface, T, pose=_pose(.1), pose_ns=T - 5_000_000)
  assert interface._bosch_context[1] == -.1
  RadarInterface.set_bosch_context(interface, T + 10_000_000, pose=_pose(.3), pose_ns=T + 12_000_000)
  assert interface._bosch_context[1] == -.1                     # newer pose not current yet
  assert all(ns <= T + 10_000_000 for ns, _ in recorder.poses)
  RadarInterface.set_bosch_context(interface, T + 20_000_000, pose=_pose(.3), pose_ns=T + 12_000_000)
  assert interface._bosch_context[1] == -.3


def test_context_reads_each_model_message_once():
  # The context runs at 100 Hz with the same 20 Hz modelV2. One snapshot serves
  # every consumer; repeating it leaves the road-edge lists untouched.
  interface, _ = _context_interface()
  sample = _context_model()
  RadarInterface.set_bosch_context(interface, T, model=sample, model_ns=T - 10_000_000)
  first = interface._bosch_context
  edges = interface.bosch.road_edge_filter.model_edges
  RadarInterface.set_bosch_context(interface, T + 10_000_000, model=sample, model_ns=T - 10_000_000)
  assert interface.bosch.road_edge_filter.model_edges is edges
  assert interface._bosch_context[1:] == first[1:]
  assert first[2] == (BoschVisionCue(30.0 - 1.52, -0.5, .9),)
  assert first[3] == ((0.0, 0.0), (50.0, 0.0))
  assert len(interface._bosch_model_snapshots) == 1
  RadarInterface.set_bosch_context(interface, T + 20_000_000, model=_context_model(1.0), model_ns=T + 15_000_000)
  assert interface.bosch.road_edge_filter.model_edges is not edges
  assert interface._bosch_context[3][0] == (1.0, 0.0)


def test_repeated_snapshot_still_clears_for_unusable_edges():
  f = BoschRoadEdgePublicationFilter()
  bad = _context_model()
  bad.roadEdgeStds = (2., 2.)
  bad = BoschModelSnapshot(bad)
  f.ingest_model(bad, T)
  f.would_suppress = frozenset({1})
  f.ingest_model(bad, T)
  assert not f.would_suppress
  f.invalidate_model()
  f.ingest_model(bad, T)
  assert f.model_edges == [(T, None)]
