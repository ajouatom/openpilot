import pytest

from opendbc.car.hyundai.radar_bosch import (
  BOSCH_CAMERA_ASSOC_ASSIGNED,
  BOSCH_MIRROR_BIRTH_ACTIVE,
  BOSCH_MIRROR_BIRTH_OFF,
  BOSCH_MIRROR_BIRTH_SHADOW,
  BoschMirrorBirthHold,
  BoschPhysicalObject,
  BoschRawDetection,
  BoschRawTrack,
)


def raw_track(raw_id, ns, d_rel, y_rel, v_rel, age=1):
  detection = BoschRawDetection(ns, raw_id % 32, float(d_rel), float(y_rel), float(v_rel), 0)
  return BoschRawTrack(raw_id, detection, age, False)


def physical(pid, raw_id, ns, d_rel, y_rel, v_rel, age, *, oem=False, vision=False):
  member = raw_track(raw_id, ns, d_rel, y_rel, v_rel, age)
  return BoschPhysicalObject(pid, ns, (member,), raw_id, float(d_rel), float(y_rel), float(v_rel),
                             oem, vision, age, 'single_return')


def mirror_scan(candidate_y=8.0, *, ns=1_000_000_000, candidate_age=1,
                candidate_v=-15.0, parent_present=True, oem=False, vision=False):
  parent_y = .5
  wall_y = (candidate_y + parent_y) / 2
  candidate = physical(1001, 1001, ns, 30.5, candidate_y, candidate_v,
                       candidate_age, oem=oem, vision=vision)
  objects = [candidate]
  raws = [candidate.members[0]]
  if parent_present:
    parent = physical(2001, 2001, ns, 30.0, parent_y, -15.0, 10)
    objects.append(parent)
    raws.append(parent.members[0])
  for index, d_rel in enumerate((10., 12., 14., 16., 18., 20., 22.)):
    raws.append(raw_track(3000 + index, ns, d_rel, wall_y, -20.0, 5))
  return tuple(objects), tuple(raws)


def update_birth_hold(hold, *, candidate_y=8.0, ns=1_000_000_000):
  objects, raws = mirror_scan(candidate_y=candidate_y, ns=ns)
  hold.update(objects, raws, ns, 20.0, 0.0)
  return objects


def start_hold():
  hold = BoschMirrorBirthHold(BOSCH_MIRROR_BIRTH_SHADOW)
  update_birth_hold(hold)
  assert 1001 in hold.holds
  return hold


class TestBoschMirrorBirthHold:
  def test_retired_birth_guards_are_pruned_but_coasting_ids_remain(self):
    hold = BoschMirrorBirthHold()
    objects = (physical(1001, 1001, 1_000_000_000, 30, 0, -10, 1),)
    hold.update(objects, (), 1_000_000_000, 20, 0, live_pids={1001})
    # A live tracker state can coast without appearing in the current scan.
    hold.update((), (), 1_100_000_000, 20, 0, live_pids={1001})
    assert hold.seen_births == {1001}
    hold.update((), (), 1_200_000_000, 20, 0, live_pids=set())
    assert not hold.seen_births

  def test_birth_guard_memory_is_bounded_under_continuous_pid_churn(self):
    hold = BoschMirrorBirthHold()
    for index in range(2000):
      pid, ns = 1001 + index, 1_000_000_000 + index * 100_000_000
      objects = (physical(pid, pid, ns, 30, 0, -10, 1),)
      hold.update(objects, (), ns, 20, 0, live_pids={pid})
      assert hold.seen_births == {pid}

  def test_default_mode_is_active(self):
    assert BoschMirrorBirthHold().mode == BOSCH_MIRROR_BIRTH_ACTIVE

  def test_nonbirth_objects_skip_wall_search(self, monkeypatch):
    hold = BoschMirrorBirthHold(BOSCH_MIRROR_BIRTH_SHADOW)
    objects, raws = mirror_scan(candidate_age=2)
    calls = 0
    wall = hold._wall

    def count_wall_calls(*args, **kwargs):
      nonlocal calls
      calls += 1
      return wall(*args, **kwargs)

    monkeypatch.setattr(hold, '_wall', count_wall_calls)
    hold.update(objects, raws, 1_000_000_000, 20.0, 0.0)
    assert calls == 0

  def test_mirror_birth_holds_but_nonbirth_does_not(self):
    hold = BoschMirrorBirthHold()
    objects, raws = mirror_scan(candidate_age=1)
    hold.update(objects, raws, 1_000_000_000, 20.0, 0.0)
    assert hold.holds[1001].parent_pid == 2001
    assert hold.holds[1001].wall_y_m == pytest.approx(4.25)

    nonbirth = BoschMirrorBirthHold()
    objects, raws = mirror_scan(candidate_age=2)
    nonbirth.update(objects, raws, 1_000_000_000, 20.0, 0.0)
    assert not nonbirth.holds

  def test_y_min_boundary_is_inclusive(self):
    at_boundary = BoschMirrorBirthHold()
    objects, raws = mirror_scan(candidate_y=4.5)
    at_boundary.update(objects, raws, 1_000_000_000, 20.0, 0.0)
    assert 1001 in at_boundary.holds

    below_boundary = BoschMirrorBirthHold()
    objects, raws = mirror_scan(candidate_y=4.49)
    below_boundary.update(objects, raws, 1_000_000_000, 20.0, 0.0)
    assert not below_boundary.holds

  @pytest.mark.parametrize('evidence', ['oem', 'vision', 'camera', 'word0'])
  def test_independent_identity_releases(self, evidence):
    hold = start_hold()
    ns = 1_100_000_000
    objects, raws = mirror_scan(ns=ns, candidate_age=2,
                                oem=evidence == 'oem', vision=evidence == 'vision')
    camera = {1001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7)} if evidence == 'camera' else {}
    word0 = {1001} if evidence == 'word0' else set()
    hold.update(objects, raws, ns, 20.0, 0.0,
                camera_associations=camera, word0_pids=word0)
    assert 1001 not in hold.holds
    assert hold.last_events[-1]['reason'] == 'INDEPENDENT_IDENTITY'

  def test_moved_inside_releases(self):
    hold = start_hold()
    ns = 1_100_000_000
    objects, raws = mirror_scan(candidate_y=4.4, ns=ns, candidate_age=2)
    hold.update(objects, raws, ns, 20.0, 0.0)
    assert hold.last_events[-1]['reason'] == 'MOVED_INSIDE'

  def test_parent_lost_releases_after_two_scans(self):
    hold = start_hold()
    for index in (1, 2):
      ns = 1_000_000_000 + index * 100_000_000
      objects, raws = mirror_scan(ns=ns, candidate_age=index + 1, parent_present=False)
      hold.update(objects, raws, ns, 20.0, 0.0)
      if index == 1:
        assert 1001 in hold.holds
    assert hold.last_events[-1]['reason'] == 'PARENT_LOST'

  def test_divergence_releases_after_three_scans(self):
    hold = start_hold()
    for index in (1, 2, 3):
      ns = 1_000_000_000 + index * 100_000_000
      objects, raws = mirror_scan(ns=ns, candidate_age=index + 1, candidate_v=-12.0)
      hold.update(objects, raws, ns, 20.0, 0.0)
      if index < 3:
        assert 1001 in hold.holds
    assert hold.last_events[-1]['reason'] == 'DIVERGED'

  def test_expiry_is_after_ten_held_scans(self):
    hold = start_hold()
    for index in range(1, 11):
      ns = 1_000_000_000 + index * 100_000_000
      objects, raws = mirror_scan(ns=ns, candidate_age=index + 1)
      hold.update(objects, raws, ns, 20.0, 0.0)
      if index < 10:
        assert 1001 in hold.holds
    assert hold.last_events[-1]['reason'] == 'EXPIRED'
    assert hold.last_events[-1]['held_scans'] == 11

  def test_gap_resets_holds_and_birth_memory(self):
    hold = start_hold()
    ns = 1_300_000_000
    objects, raws = mirror_scan(ns=ns, candidate_age=2)
    hold.update(objects, raws, ns, 20.0, 0.0)
    assert hold.last_events[0]['reason'] == 'GAP_RESET'
    assert not hold.holds

    ns += 100_000_000
    objects, raws = mirror_scan(ns=ns, candidate_age=1)
    hold.update(objects, raws, ns, 20.0, 0.0)
    assert 1001 in hold.holds

  def test_clock_regression_resets_holds_and_birth_memory(self):
    hold = start_hold()
    ns = 900_000_000
    objects, raws = mirror_scan(ns=ns, candidate_age=2)
    hold.update(objects, raws, ns, 20.0, 0.0)
    assert hold.last_events[0]['reason'] == 'GAP_RESET'
    assert not hold.holds

    ns = 1_000_000_000
    objects, raws = mirror_scan(ns=ns, candidate_age=1)
    hold.update(objects, raws, ns, 20.0, 0.0)
    assert 1001 in hold.holds

  def test_provider_timeout_reset_releases_all_state(self):
    hold = start_hold()
    hold.reset(1_150_000_000, 'PROVIDER_TIMEOUT')
    assert hold.last_events[0]['reason'] == 'PROVIDER_TIMEOUT'
    assert not hold.holds
    assert not hold.seen_births
    assert not hold.hist

  def test_pid_death_releases_and_same_pid_is_not_reheld(self):
    hold = start_hold()
    ns = 1_100_000_000
    objects, raws = mirror_scan(ns=ns, candidate_age=2, parent_present=True)
    objects = tuple(obj for obj in objects if obj.physical_track_id != 1001)
    raws = tuple(raw for raw in raws if raw.raw_track_id != 1001)
    hold.update(objects, raws, ns, 20.0, 0.0)
    assert hold.last_events[-1]['reason'] == 'PID_DEAD'

    ns += 100_000_000
    objects, raws = mirror_scan(ns=ns, candidate_age=1)
    hold.update(objects, raws, ns, 20.0, 0.0)
    assert 1001 not in hold.holds

  @pytest.mark.parametrize('mode', [BOSCH_MIRROR_BIRTH_OFF, BOSCH_MIRROR_BIRTH_SHADOW])
  def test_off_and_shadow_leave_publication_tuple_unchanged(self, mode):
    hold = BoschMirrorBirthHold(mode)
    objects, raws = mirror_scan()
    hold.update(objects, raws, 1_000_000_000, 20.0, 0.0)
    assert hold.publication_view(objects) is objects

  def test_active_only_removes_held_pid(self):
    hold = BoschMirrorBirthHold(BOSCH_MIRROR_BIRTH_ACTIVE)
    objects, raws = mirror_scan()
    hold.update(objects, raws, 1_000_000_000, 20.0, 0.0)
    published = hold.publication_view(objects)
    assert tuple(obj.physical_track_id for obj in published) == (2001,)
    assert hold.publication_suppressed == 1
