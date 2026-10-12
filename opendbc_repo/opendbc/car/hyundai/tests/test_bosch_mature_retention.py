from dataclasses import replace
import math
import pickle

import pytest

from opendbc.car.hyundai.radar_bosch import (
  BoschGroupingConfig, BoschObjectGroupManager, BoschRadarProvider, BoschRawDetection, BoschRawTrack,
  _BoschMatureGroupCertificate,
)


STEP = 100_000_000
STRAIGHT = tuple((float(x), 0.) for x in range(0, 120, 5))


def raw(rid, scan, d=50., y=0., v=-5., age=None):
  return BoschRawTrack(rid, BoschRawDetection(scan*STEP, rid-1, float(d), float(y), float(v)), scan if age is None else age)


def update(manager, scan, dd=2.8, dy=0., dv=0., age=None, outsider=False, missing=False):
  tracks = [raw(1, scan, age=age)]
  if not missing:
    tracks.append(raw(2, scan, 50.+dd, dy, -5.+dv, age))
  if outsider:
    tracks.append(raw(3, scan, 54., 0., -5.))
  if manager.mature_retention is not None:
    manager.mature_retention.ingest_path(STRAIGHT, scan*STEP)
  return manager.update(scan*STEP, tracks, v_ego=10.)


def mature(dd=2.8, dy=0.):
  manager = BoschObjectGroupManager(BoschGroupingConfig(mature_retention_enabled=True))
  for scan in range(1, 31):
    objects = update(manager, scan, dd, dy)
  assert len(objects) == 1
  assert manager.mature_retention.families[(1, 2)].ready
  return manager


@pytest.mark.parametrize('value', [0, 1, None, 'true', math.nan])
def test_switch_requires_boolean(value):
  with pytest.raises(ValueError, match='mature_retention_enabled must be boolean'):
    BoschGroupingConfig(mature_retention_enabled=value)
  with pytest.raises(ValueError):
    BoschRadarProvider(1, mature_group_retention=value)


def test_default_allocates_no_retention_and_keeps_strict_split():
  default = BoschObjectGroupManager()
  explicit = BoschObjectGroupManager(BoschGroupingConfig(mature_retention_enabled=False))
  assert default.mature_retention is explicit.mature_retention is None
  for scan in range(1, 41):
    dd = 2.8 if scan < 31 else 3.125
    assert update(default, scan, dd) == update(explicit, scan, dd)
  assert len(default.states) == 2
  # The provider enables guarded retention by default; the bare manager and an explicit opt-out do not.
  assert BoschRadarProvider(1).tracker.group_manager.mature_retention is not None
  assert BoschRadarProvider(1, mature_group_retention=False).tracker.group_manager.mature_retention is None


def test_bounded_retention_preserves_pid_but_never_writes_strict_pair_evidence():
  manager = mature()
  pid = next(iter(manager.states))
  result = update(manager, 31, dd=3.125)
  assert len(result) == 1 and result[0].physical_track_id == pid
  assert (1, 2) not in manager.pairs
  assert manager.mature_retention.events[0]['added'] == [(1, 2)]
  assert max(len(c.history) for c in manager.mature_retention.families.values()) == 21
  assert len(manager.mature_retention.last_raw_ages) == 2
  for scan in range(32, 51):
    assert len(update(manager, scan, dd=3.125)) == 1
  assert len(update(manager, 51, dd=3.125)) == 2
  assert manager.mature_retention.stats['lease_expiry_veto'] == 1


def test_unmatured_or_never_formed_families_cannot_be_retained():
  manager = BoschObjectGroupManager(BoschGroupingConfig(mature_retention_enabled=True))
  for scan in range(1, 11):
    update(manager, scan)
  assert len(update(manager, 11, dd=3.125)) == 2
  never = BoschObjectGroupManager(BoschGroupingConfig(mature_retention_enabled=True))
  for scan in range(1, 41):
    assert len(update(never, scan, dd=3.125)) == 2
  assert not never.mature_retention.events


@pytest.mark.parametrize('kwargs,reason', [
  ({'dd': 3.5}, 'extent_veto'), ({'dy': 1.5625}, 'extent_veto'),
  ({'dd': 3.125, 'dv': .75}, 'motion_veto'), ({'dd': 3.125, 'outsider': True}, 'near_competitor_veto'),
  ({'dd': 3.125, 'age': 1}, 'age_rollback_veto'), ({'missing': True}, 'missing_member_veto'),
])
def test_adversarial_lifecycle_and_geometry_guards(kwargs, reason):
  manager = mature()
  result = update(manager, 31, **kwargs)
  assert not manager.mature_retention.events
  assert manager.mature_retention.stats[reason] == 1
  if not kwargs.get('missing'):
    assert len(result) >= 2


def test_input_gap_resets_archive_and_maturity():
  manager = mature()
  assert len(update(manager, 33, dd=3.125)) == 2
  retention = manager.mature_retention
  assert retention.stats['gap_resets'] == 1
  assert not retention.suspended and not retention.families


def test_shape_growth_veto_and_bounded_rotation_allowance():
  manager = mature(dd=.25, dy=1.4)
  assert len(update(manager, 31, dd=1.625, dy=1.5)) == 2
  assert manager.mature_retention.stats['shape_growth_veto'] == 1
  rotation = mature(dd=.25, dy=1.4)
  assert len(update(rotation, 31, dd=1.625, dy=0.)) == 1
  assert rotation.mature_retention.stats['retained_edges'] == 1


def test_natural_strict_rejoin_reuses_maturity_without_forcing_children_or_copying_pid_history():
  manager = mature()
  assert len(update(manager, 31, dd=3.5)) == 2
  # Split children cannot receive archived edges during renewed pair evidence.
  assert len(update(manager, 32)) == len(update(manager, 33)) == 2
  assert not manager.mature_retention.events
  assert len(update(manager, 34)) == 1
  assert len(update(manager, 35, dd=3.125)) == 1
  assert manager.mature_retention.stats['natural_rejoin_certificate_reused'] == 1
  memberships = [rid for state in manager.states.values() for rid in state.member_last_seen]
  assert len(memberships) == len(set(memberships))
  certificate = manager.mature_retention.families[(1, 2)]
  assert set(vars(certificate)) == {'start_ns', 'last_strict_ns', 'ages', 'history', 'ready'}


def test_archive_pruned_after_expiry_or_mixed_outsider_group():
  manager = mature()
  retention = manager.mature_retention
  assert len(update(manager, 31, dd=3.5)) == 2
  for scan in range(32, 62):
    update(manager, scan, dd=3.5)
  assert (1, 2) not in retention.suspended
  manager = mature()
  retention = manager.mature_retention
  prior = next(iter(manager.states.values()))
  prior.observation = replace(prior.observation, members=prior.observation.members+(raw(3, 30, 54.),))
  update(manager, 31, dd=3.5, outsider=True)
  assert (1, 2) not in retention.suspended


def test_strict_outsider_competition_does_not_add_family_edges():
  manager = mature()
  retention = manager.mature_retention
  raws = (raw(1, 31), raw(2, 31, 53.125), raw(3, 31, 49.))
  compatibility = [4, 0, 1]
  retention.before(manager, 31*STEP, raws, compatibility, [0.]*9, {1: 0, 2: 1, 3: 2}, 10.)
  assert compatibility == [4, 0, 1]
  assert retention.stats['strict_competitor_veto'] == 1


def test_archive_and_raw_age_bounds_are_deterministic():
  manager = BoschObjectGroupManager(BoschGroupingConfig(mature_retention_enabled=True))
  retention = manager.mature_retention
  raws = tuple(raw(rid, 1, float(10+rid)) for rid in range(1, 33))
  retention.suspended = {
    (a, b): _BoschMatureGroupCertificate(0, STEP, (0, 0), ready=True)
    for a in range(1, 33) for b in range(a+1, 33)
  }
  retention.before(manager, STEP, raws, [0]*32, [0.]*1024, {r.raw_track_id: i for i, r in enumerate(raws)}, 10.)
  assert len(retention.suspended) == 64
  assert tuple(retention.suspended) == tuple(sorted(retention.suspended))
  assert len(retention.last_raw_ages) == 32


def test_missing_recent_shape_reference_fails_open():
  manager = mature()
  cert = manager.mature_retention.families[(1, 2)]
  cert.history = [(0, {(1, 2): (-2.8, 0.)})]
  assert len(update(manager, 31, dd=3.125)) == 2
  assert manager.mature_retention.stats['separation_reference_missing_veto'] == 1


def test_rejected_timestamp_is_transactional_for_certificates_and_ownership():
  manager = mature()
  before = pickle.dumps(manager.__dict__)
  with pytest.raises(ValueError, match='strictly increasing'):
    manager.update(30*STEP, ())
  assert pickle.dumps(manager.__dict__) == before


def test_thirty_two_raws_produce_at_most_sixteen_live_family_certificates():
  manager = BoschObjectGroupManager(BoschGroupingConfig(mature_retention_enabled=True))
  for scan in range(1, 31):
    tracks = tuple(raw(rid, scan, 10.+((rid-1)//2)*10.+((rid-1) % 2)*.25) for rid in range(1, 33))
    manager.update(scan*STEP, tracks, v_ego=10.)
  retention = manager.mature_retention
  assert len(retention.families) == len(retention.suspended) == 16
  assert len(retention.last_raw_ages) == 32
  assert all(len(cert.history) == 21 for cert in retention.families.values())


def test_certificate_clone_isolates_mutable_containers_and_field_assignments():
  certificate = mature().mature_retention.families[(1, 2)]
  before = pickle.dumps(certificate)
  cloned = certificate.clone()
  assert cloned == certificate and cloned is not certificate
  assert cloned.ages is certificate.ages
  assert cloned.history is not certificate.history
  for original_entry, cloned_entry in zip(certificate.history, cloned.history, strict=True):
    assert cloned_entry is not original_entry and cloned_entry[1] is not original_entry[1]
    for pair, offset in original_entry[1].items():
      assert next(key for key in cloned_entry[1] if key == pair) is pair
      assert cloned_entry[1][pair] is offset
  cloned.history[0][1][(1, 2)] = (999., -0.)
  cloned.history.append((0, {}))
  cloned.start_ns = cloned.last_strict_ns = 0
  cloned.ages = (0, 0)
  cloned.ready = False
  assert pickle.dumps(certificate) == before
  cloned_before = pickle.dumps(cloned)
  certificate.history[1][1].clear()
  certificate.history.pop()
  certificate.ages = (1, 1)
  certificate.ready = False
  assert pickle.dumps(cloned) == cloned_before


def test_certificate_clone_preserves_repeated_internal_history_aliases():
  offsets = {(1, 2): (-0., 1.5)}
  entry = (STEP, offsets)
  certificate = _BoschMatureGroupCertificate(0, STEP, (1, 1), [entry, entry, (2*STEP, offsets)], True)
  cloned = certificate.clone()
  assert cloned == certificate
  assert cloned.history[0] is cloned.history[1]
  assert cloned.history[0][1] is cloned.history[2][1]
  assert cloned.history[0] is not entry and cloned.history[0][1] is not offsets
  cloned.history[0][1][(1, 2)] = (2., 3.)
  assert cloned.history[2][1][(1, 2)] == (2., 3.)
  assert offsets == {(1, 2): (-0., 1.5)}


def test_manager_deepcopy_and_pickle_keep_certificate_graph_isolated():
  import copy
  manager = mature()
  before = pickle.dumps(manager.__dict__)
  copied = copy.deepcopy(manager)
  assert pickle.dumps(copied.__dict__) == before
  copied.mature_retention.families[(1, 2)].history[0][1].clear()
  copied.mature_retention.families[(1, 2)].ages = (0, 0)
  copied.mature_retention.stats['probe'] += 1
  assert pickle.dumps(manager.__dict__) == before
  restored = pickle.loads(pickle.dumps(manager))
  assert pickle.dumps(restored.__dict__) == before


def test_id_overflow_staging_preserves_certificate_transactions():
  manager = mature()
  pid = next(iter(manager.states))
  manager.next_id = 2**31-1
  # The overflow guard must stage this carry update, without needing a new ID.
  assert update(manager, 31)[0].physical_track_id == pid
  before = pickle.dumps(manager.__dict__)
  with pytest.raises(OverflowError, match='physical Int32 ID space exhausted'):
    manager.update(32*STEP, (raw(3, 32, 100.), raw(4, 32, 150.)), v_ego=10.)
  assert pickle.dumps(manager.__dict__) == before


def test_archive_and_natural_rejoin_preserve_mutable_snapshot_isolation():
  manager = mature()
  retention = manager.mature_retention
  for scan, dd in ((31, 3.5), (32, 2.8), (33, 2.8), (34, 2.8), (35, 3.125)):
    update(manager, scan, dd)
  assert retention.stats['natural_rejoin_certificate_reused'] == 1
  live = retention.families[(1, 2)]
  archived = retention.suspended[(1, 2)]
  assert live.history is not archived.history
  assert all(a[1] is not b[1] for a, b in zip(live.history, archived.history, strict=True))
  before = pickle.dumps(archived)
  live.history[0][1].clear()
  live.history.pop()
  assert pickle.dumps(archived) == before
  live_before = pickle.dumps(live)
  archived.history[1][1].clear()
  assert pickle.dumps(live) == live_before


# ---- world-robust surface guard -------------------------------------------------------------------------------


class _Pair:
  """Near return 1 at 50 m, far return 2 at 50 m + dd. With far_first the far return is tracked alone first,
  so its PID and continuity anchor predate the near one."""
  def __init__(self, y_near=0., y_far=0., v_far=-5., far_first=False, path=STRAIGHT):
    self.m = BoschObjectGroupManager(BoschGroupingConfig(mature_retention_enabled=True))
    self.y_near, self.y_far, self.v_far, self.far_first, self.path = y_near, y_far, v_far, far_first, path

  def step(self, scan, dd, oem_slot=None, feed_path=True):
    if feed_path:
      self.m.mature_retention.ingest_path(self.path, scan*STEP)
    far = raw(2, scan, 50.+dd, self.y_far, self.v_far, scan)
    if self.far_first and scan <= 5:
      tracks = [far]
    else:
      tracks = [raw(1, scan, 50., self.y_near, -5., scan-(5 if self.far_first else 0)), far]
    return self.m.update(scan*STEP, tracks, v_ego=10., oem_slot=oem_slot)

  def boundary(self, oem_slot=None, feed_path=True):
    for scan in range(1, 31):
      objects = self.step(scan, 2.8, oem_slot)
    assert len(objects) == 1 and self.m.mature_retention.families[(1, 2)].ready
    return self.step(31, 3.125, oem_slot, feed_path)


@pytest.mark.parametrize('y_near,y_far,groups', [(.1, -.1, 1), (.45, 0., 1), (.8, 0., 2), (0., 1.2, 1)])
def test_guard_requires_surface_lateral_coverage(y_near, y_far, groups):
  pair = _Pair(y_near, y_far)
  assert len(pair.boundary()) == groups
  assert pair.m.mature_retention.stats['guard_cover_lateral_veto'] == (groups == 2)


def test_guard_rejects_hidden_nearer_member_unless_oem_publishes_it():
  pair = _Pair(far_first=True)
  assert len(pair.boundary()) == 2
  assert pair.m.mature_retention.stats['guard_surface_not_nearest_veto'] == 1
  assert len(_Pair(far_first=True).boundary(oem_slot=0)) == 1


def test_guard_rejects_surface_faster_than_hidden_member():
  pair = _Pair(v_far=-5.5)
  assert len(pair.boundary()) == 2
  assert pair.m.mature_retention.stats['guard_cover_speed_veto'] == 1


def test_guard_fails_closed_without_path():
  pair = _Pair(path=())
  assert len(pair.boundary()) == 2
  assert pair.m.mature_retention.stats['guard_path_missing_veto'] == 1


@pytest.mark.parametrize('age,retained', [(5, True), (6, False)])
def test_guard_path_hold_is_bounded(age, retained):
  pair = _Pair()
  for scan in range(1, 31):
    pair.step(scan, 2.8)
  retention = pair.m.mature_retention
  retention.path, retention.path_ns = STRAIGHT, (31-age)*STEP
  assert (len(pair.step(31, 3.125, feed_path=False)) == 1) is retained
