from copy import deepcopy
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.radar_bosch import (
  BOSCH_CAMERA_ASSOC_ASSIGNED,
  BOSCH_MIRROR_BIRTH_ACTIVE,
  BOSCH_MIRROR_M3_ACTIVE,
  BOSCH_MIRROR_M3_OFF,
  BOSCH_MIRROR_M3_SHADOW,
  BoschMirrorBirthHold,
  BoschMirrorM3Shadow,
)

_DEFAULT_EDGE = object()


def obj(d=30.0, y=5.0, v=-8.0, age=1, *, oem=False, vision=False, members=()):
  return (float(d), float(y), float(v), int(age), tuple(members), None, bool(oem), bool(vision))


def edge_sample(ns=1_000_000_000, *, xs=(0.0, 100.0), left=(2.0, 2.0), right=(-2.0, -2.0)):
  return ns, tuple(xs), tuple(left), tuple(right)


def edge_birth(shadow=None, *, ns=1_000_000_000, candidate=None, parent=None,
               edge=_DEFAULT_EDGE, v_ego=12.0, decisions=None, camera=None, word0=()):
  shadow = shadow or BoschMirrorM3Shadow(BOSCH_MIRROR_M3_SHADOW)
  candidate = candidate or obj()
  parent = parent or obj(d=30.0, y=0.0, v=-8.0, age=10)
  sample = edge_sample() if edge is _DEFAULT_EDGE else edge
  shadow.model_edges = [sample] if sample is not None else []
  mirror = {1001: candidate, 2001: parent}
  shadow.update(ns, v_ego, mirror, decisions or {}, camera_associations=camera, word0_pids=word0)
  return shadow


class TestBoschMirrorM3Shadow:
  def test_retired_birth_guards_are_pruned_but_coasting_ids_remain(self):
    shadow = BoschMirrorM3Shadow()
    shadow.update(1_000_000_000, 12, {1001: obj(y=0)}, {}, live_pids={1001})
    shadow.update(1_100_000_000, 12, {}, {}, live_pids={1001})
    assert shadow.seen_births == {1001}
    shadow.update(1_200_000_000, 12, {}, {}, live_pids=set())
    assert not shadow.seen_births

  def test_birth_guard_memory_is_bounded_under_continuous_pid_churn(self):
    shadow = BoschMirrorM3Shadow()
    for index in range(2000):
      pid, ns = 1001 + index, 1_000_000_000 + index * 100_000_000
      shadow.update(ns, 12, {pid: obj(y=0)}, {}, live_pids={pid})
      assert shadow.seen_births == {pid}

  def test_default_is_active_and_other_mode_is_rejected(self):
    assert BoschMirrorM3Shadow().mode == BOSCH_MIRROR_M3_ACTIVE
    with pytest.raises(ValueError):
      BoschMirrorM3Shadow(3)

  def test_active_and_shadow_decisions_and_events_are_identical(self):
    shadow = edge_birth(BoschMirrorM3Shadow(BOSCH_MIRROR_M3_SHADOW))
    active = edge_birth(BoschMirrorM3Shadow(BOSCH_MIRROR_M3_ACTIVE))
    assert shadow.holds == active.holds
    assert shadow.last_events == active.last_events

  def test_left_edge_birth_starts_hold_with_reference_fields(self):
    shadow = edge_birth()
    state = shadow.holds[1001]
    assert (state.path, state.parent_pid, state.wall_y_m, state.residual_m) == ('EDGE', 2001, 2.0, 1.0)
    assert shadow.last_events[0]['action'] == 'HOLD'
    assert shadow.last_events[0]['v_ego'] == 12.0

  def test_v1_qualify_uses_m1_parent_wall_and_residual_without_edge(self):
    m1_decision = SimpleNamespace(action='QUALIFY', parent_pid=2999, wall_y_m=3.0, residual_m=0.25)
    shadow = edge_birth(edge=None, decisions={1001: m1_decision})
    state = shadow.holds[1001]
    assert (state.path, state.parent_pid, state.wall_y_m, state.residual_m) == ('V1', 2999, 3.0, 0.25)

  def test_hold_coupling_uses_reference_three_decimal_wall_and_residual(self):
    shadow = edge_birth(candidate=obj(y=7.0), parent=obj(y=0.0, age=10),
                        edge=edge_sample(left=(3.8282, 3.8282)))
    state = shadow.holds[1001]
    assert (state.wall_y_m, state.residual_m) == (3.828, 0.656)
    # Full precision would exceed the 3 m extension residual on this scan;
    # the reference simulator stores wall/residual to 3 decimals at birth.
    shadow.update(1_100_000_000, 12.0,
                  {1001: obj(y=9.125, age=2), 2001: obj(y=-4.46875, age=11)}, {})
    assert 1001 in shadow.holds

  def test_edge_birth_fails_closed_for_nonfinite_ego_speed(self):
    shadow = edge_birth(v_ego=float('nan'))
    assert not shadow.holds

  @pytest.mark.parametrize(('candidate', 'v_ego', 'edge', 'parent'), [
    (obj(y=-5.0), 12.0, edge_sample(right=(2.0, 2.0)), obj(y=0.0)),  # right side is disabled
    (obj(), 9.99, edge_sample(), obj(y=0.0)),                       # ego speed
    (obj(d=80.001), 12.0, edge_sample(), obj(d=80.0, y=0.0)),      # range
    (obj(), 12.0, None, obj(y=0.0)),                               # no edge
    (obj(), 12.0, edge_sample(left=(0.5, 0.5)), obj(y=0.0)),       # wall side minimum
    (obj(y=4.5), 12.0, edge_sample(left=(4.0, 4.0)), obj(y=0.0)),  # beyond minimum
    (obj(), 12.0, edge_sample(), obj(y=0.0, age=9)),               # parent age
    (obj(), 12.0, edge_sample(), obj(y=0.0, v=-12.0)),            # parent world speed
    (obj(), 12.0, edge_sample(), obj(y=1.1)),                      # parent inside wall
    (obj(), 12.0, edge_sample(), obj(d=31.501, y=0.0)),            # parent range lock
    (obj(), 12.0, edge_sample(), obj(y=0.0, v=-7.49)),             # parent velocity lock
    (obj(y=7.1), 12.0, edge_sample(), obj(y=0.0)),                 # mirror residual
  ])
  def test_edge_birth_fail_closed_inputs(self, candidate, v_ego, edge, parent):
    shadow = edge_birth(candidate=candidate, v_ego=v_ego, edge=edge, parent=parent)
    assert not shadow.holds

  def test_stale_future_and_duplicate_model_samples(self):
    shadow = BoschMirrorM3Shadow(BOSCH_MIRROR_M3_SHADOW)
    model = SimpleNamespace(roadEdges=[SimpleNamespace(x=[0, 10], y=[-2, -2]),
                                       SimpleNamespace(x=[0, 10], y=[2, 2])])
    shadow.ingest_model(model, 1_000_000_000)
    shadow.ingest_model(model, 1_000_000_000)
    assert len(shadow.model_edges) == 1
    assert shadow._edge_at(1_200_000_000) == shadow.model_edges[0]
    assert shadow._edge_at(1_200_000_001) is None
    assert shadow._edge_at(999_999_999) is None

  def test_ingest_requires_two_road_edges_and_keeps_only_four_unique_times(self):
    shadow = BoschMirrorM3Shadow(BOSCH_MIRROR_M3_SHADOW)
    one_edge = SimpleNamespace(roadEdges=[SimpleNamespace(x=[0, 1], y=[0, 0])])
    shadow.ingest_model(one_edge, 1)
    assert not shadow.model_edges
    model = SimpleNamespace(roadEdges=[SimpleNamespace(x=[0, 1], y=[-1, -1]),
                                       SimpleNamespace(x=[0, 1], y=[2, 2])])
    for ns in range(1, 7):
      shadow.ingest_model(model, ns)
    assert [sample[0] for sample in shadow.model_edges] == [3, 4, 5, 6]
    assert shadow.model_edges[-1][2] == (1.0, 1.0)  # road-frame y_left = -model_y

  def test_numpy_interpolation_clamps_outside_edge_extent(self):
    sample = edge_sample(xs=(0.0, 10.0), left=(1.5, 2.0))
    shadow = edge_birth(candidate=obj(d=30.0, y=5.0), parent=obj(d=30.0, y=0.0, age=10), edge=sample)
    assert shadow.holds[1001].wall_y_m == 2.0

  def test_equal_residual_keeps_first_qualified_parent(self):
    shadow = BoschMirrorM3Shadow(BOSCH_MIRROR_M3_SHADOW)
    mirror = {
      1001: obj(),
      2001: obj(y=0.0, age=10),
      2002: obj(y=0.0, age=10),
    }
    shadow.model_edges = [edge_sample()]
    shadow.update(1_000_000_000, 12.0, mirror, {})
    assert shadow.holds[1001].parent_pid == 2001

  @pytest.mark.parametrize('identity', ['oem', 'vision', 'camera', 'word0'])
  def test_independent_identity_prevents_edge_birth(self, identity):
    candidate = obj(oem=identity == 'oem', vision=identity == 'vision')
    camera = {1001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7)} if identity == 'camera' else None
    word0 = {1001} if identity == 'word0' else ()
    shadow = edge_birth(candidate=candidate, camera=camera, word0=word0)
    assert not shadow.holds

  def test_extension_continues_until_cap_then_releases(self):
    shadow = edge_birth()
    for index in range(1, 50):
      mirror = {1001: obj(age=index + 1), 2001: obj(y=0.0, age=10 + index)}
      shadow.update(1_000_000_000 + index * 100_000_000, 12.0, mirror, {})
      assert 1001 in shadow.holds
    mirror = {1001: obj(age=51), 2001: obj(y=0.0, age=60)}
    shadow.update(6_000_000_000, 12.0, mirror, {})
    event = shadow.last_events[-1]
    assert event['reason'] == 'EXT_CAP'
    assert event['held_scans'] == 51
    assert event['ext_scans'] == 40

  def test_expired_after_ten_scans_when_coupling_is_lost(self):
    shadow = edge_birth()
    for index in range(1, 11):
      parent = obj(y=0.0, age=10 + index) if index < 10 else None
      mirror = {1001: obj(age=index + 1)}
      if parent:
        mirror[2001] = parent
      shadow.update(1_000_000_000 + index * 100_000_000, 12.0, mirror, {})
      if index < 10:
        assert 1001 in shadow.holds
    assert shadow.last_events[-1]['reason'] == 'EXPIRED'

  @pytest.mark.parametrize('reason', [
    'INDEPENDENT_IDENTITY', 'MOVED_INSIDE', 'PARENT_LOST', 'DIVERGED', 'PID_DEAD', 'GAP_RESET',
  ])
  def test_hold_release_reasons(self, reason):
    shadow = edge_birth()
    if reason == 'INDEPENDENT_IDENTITY':
      shadow.update(1_100_000_000, 12.0, {1001: obj(age=2), 2001: obj(y=0.0, age=11)}, {},
                    word0_pids={1001})
    elif reason == 'MOVED_INSIDE':
      shadow.update(1_100_000_000, 12.0, {1001: obj(y=4.4, age=2), 2001: obj(y=0.0, age=11)}, {})
    elif reason == 'PARENT_LOST':
      shadow.update(1_100_000_000, 12.0, {1001: obj(age=2)}, {})
      shadow.update(1_200_000_000, 12.0, {1001: obj(age=3)}, {})
    elif reason == 'DIVERGED':
      for index in range(1, 4):
        shadow.update(1_000_000_000 + index * 100_000_000, 12.0,
                      {1001: obj(v=-6.0, age=index + 1), 2001: obj(y=0.0, age=10 + index)}, {})
    elif reason == 'PID_DEAD':
      shadow.update(1_100_000_000, 12.0, {2001: obj(y=0.0, age=11)}, {})
    else:
      shadow.update(1_300_000_000, 12.0, {1001: obj(age=2), 2001: obj(y=0.0, age=11)}, {})
    assert shadow.last_events[-1]['reason'] == reason
    assert not shadow.holds

  def test_m1_objects_and_decisions_are_read_only_inputs(self):
    shadow = BoschMirrorM3Shadow(BOSCH_MIRROR_M3_SHADOW)
    mirror = {1001: obj(), 2001: obj(y=0.0, age=10)}
    decisions = {1001: SimpleNamespace(action='NO_DECISION', parent_pid=None, wall_y_m=None, residual_m=None)}
    before = (deepcopy(mirror), deepcopy(decisions))
    shadow.model_edges = [edge_sample()]
    shadow.update(1_000_000_000, 12.0, mirror, decisions)
    assert (mirror, decisions) == before

  def test_m1_off_and_m3_off_make_no_holds(self):
    candidate, parent = obj(), obj(y=0.0, age=10)
    for mode, enabled in ((BOSCH_MIRROR_M3_OFF, True), (BOSCH_MIRROR_M3_SHADOW, False)):
      shadow = BoschMirrorM3Shadow(mode)
      shadow.model_edges = [edge_sample()]
      shadow.update(1_000_000_000, 12.0, {1001: candidate, 2001: parent}, {}, mirror_enabled=enabled)
      assert not shadow.holds

  def test_active_publication_view_suppresses_only_held_physical_pids(self):
    shadow = edge_birth(BoschMirrorM3Shadow(BOSCH_MIRROR_M3_ACTIVE))
    points = (SimpleNamespace(physical_track_id=1001), SimpleNamespace(physical_track_id=2001))
    result = shadow.publication_view(points)
    assert tuple(point.physical_track_id for point in result) == (2001,)
    assert shadow.publication_suppressed == 1

  def test_shadow_and_off_publication_views_are_noops_even_with_holds(self):
    points = (SimpleNamespace(physical_track_id=1001), SimpleNamespace(physical_track_id=2001))
    for mode in (BOSCH_MIRROR_M3_SHADOW, BOSCH_MIRROR_M3_OFF):
      shadow = edge_birth(BoschMirrorM3Shadow(BOSCH_MIRROR_M3_SHADOW))
      shadow.mode = mode
      assert shadow.publication_view(points) is points
      assert shadow.publication_suppressed == 0

  def test_m1_and_m3_overlap_is_suppressed_once_in_nested_publication_views(self):
    m1 = BoschMirrorBirthHold(BOSCH_MIRROR_BIRTH_ACTIVE)
    m1.would_suppress = frozenset({1001})
    m3 = edge_birth(BoschMirrorM3Shadow(BOSCH_MIRROR_M3_ACTIVE))
    points = (SimpleNamespace(physical_track_id=1001), SimpleNamespace(physical_track_id=2001))
    after_m1 = m1.publication_view(points)
    after_m3 = m3.publication_view(after_m1)
    assert tuple(point.physical_track_id for point in after_m3) == (2001,)
    assert m1.publication_suppressed == 1
    assert m3.publication_suppressed == 0
