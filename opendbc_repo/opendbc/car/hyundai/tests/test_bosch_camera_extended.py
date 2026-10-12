import gc
import math
import sys
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from opendbc.car.hyundai.radar_interface import (
  BOSCH_CAMERA_ASSOC_AMBIGUOUS,
  BOSCH_CAMERA_ASSOC_ASSIGNED,
  BOSCH_CAMERA_ASSOC_UNRESOLVED,
  BOSCH_CAMERA_CURVE_REACQUIRE_ACTIVE,
  BOSCH_CAMERA_CURVE_REACQUIRE_HOLD_NS,
  BOSCH_CAMERA_CURVE_REACQUIRE_OFF,
  BOSCH_CAMERA_CURVE_REACQUIRE_SHADOW,
  BOSCH_CAMERA_E2_HOLD_NS,
  BOSCH_CAMERA_EXTENDED_ACTIVE,
  BOSCH_CAMERA_EXTENDED_PUBLICATION,
  BOSCH_CAMERA_EXTENDED_OFF,
  BOSCH_CAMERA_EXTENDED_SHADOW,
  BOSCH_CAMERA_HEADER,
  BOSCH_CAMERA_LAT_MAX_M,
  BOSCH_CAMERA_LONG_MAX_M,
  BOSCH_CAMERA_RANGE_LSB,
  BOSCH_CAMERA_WIDTH_REF_M,
  BOSCH_OEM_STATE_NONE,
  BOSCH_OEM_STATE_SELECTED,
  BOSCH_OEM_STATE_TENTATIVE,
  BOSCH_OEM_STATE_VALIDATED,
  BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS,
  BOSCH_TRUCK_P2_CONFIRMATIONS,
  BoschCameraCycleCache,
  BoschCameraExtendedGrouping,
  BoschCameraObject,
  BoschPhysicalObject,
  BoschRawDetection,
  BoschRawTrack,
  BoschObjectGroupManager,
  BoschPhysicalTracker,
  BoschPublishedSurface,
  BoschRadarProvider,
  RadarInterface,
  bosch_append_points,
  bosch_fill_point,
  bosch_published_surface,
  _bosch_camera_long_window,
  _bosch_camera_range_span,
  _BoschCurveReacquireHistory,
)
from opendbc.car import structs
from opendbc.car.hyundai.radar_bosch import BoschRawTrackManager, birth_d_path


def signed(value, bits):
  return value & ((1 << bits) - 1)


def camera_frames(counter, objects):
  frames = [(BOSCH_CAMERA_HEADER, len(objects) | (counter << 52))]
  for slot, obj in enumerate(objects):
    obj_id, long_m, lat_m, vrel_mps, width_m, cls, ext, right, left = obj
    a = (obj_id | (round(long_m / BOSCH_CAMERA_RANGE_LSB) << 8) |
         (signed(round(lat_m / BOSCH_CAMERA_RANGE_LSB), 12) << 20) |
         (signed(round(vrel_mps / BOSCH_CAMERA_RANGE_LSB), 12) << 40) | (counter << 52))
    b = round(width_m / .05) | (cls << 48) | (ext << 50) | (counter << 52)
    c = (signed(round(right * 4496.3), 13) << 18) | (signed(round(left * 4496.3), 13) << 31) | (counter << 52)
    frames.extend(((0x739 + 3 * slot, a), (0x73a + 3 * slot, b), (0x73b + 3 * slot, c)))
  return frames


def feed(cache, ns, counter, objects, omit=(), counter_override=None):
  for address, word in camera_frames(counter, objects):
    if address in omit:
      continue
    if counter_override and address in counter_override:
      word &= ~(0xf << 52)
      word |= counter_override[address] << 52
    cache.ingest(ns, address, word.to_bytes(8, 'little'))


def physical(pid, d_rel, y_rel=0., v_rel=0., ns=1_000_000_000, age=4):
  raw = pid - 999_000
  detection = BoschRawDetection(ns, raw % 32, float(d_rel), float(y_rel), float(v_rel), 1)
  member = BoschRawTrack(raw, detection, age, False)
  return BoschPhysicalObject(pid, ns, (member,), raw, float(d_rel), float(y_rel), float(v_rel),
                             False, False, age, 'single_return')


class TestBoschCameraCycleCache:
  OBJ = (7, 20., -1.25, -2.5, 2.65, 1, 0, -.08, .06)

  def test_scalar_decode_matches_frozen_bit_layout(self):
    cache = BoschCameraCycleCache()
    feed(cache, 1_000_000_000, 3, [self.OBJ])
    snapshot = cache.snapshot(1_001_000_000)
    assert snapshot is not None
    objects, count, cycle, complete_ns = snapshot
    obj = objects[0]
    assert (count, cycle, complete_ns) == (1, 0, 1_000_000_000)
    assert (obj.obj_id, obj.episode, obj.class_code) == (7, 1, 1)
    assert (obj.long_m, obj.lat_m, obj.vrel_mps) == (20., -1.25, -2.5)
    assert obj.width_m == pytest.approx(2.65)
    assert obj.angle_right == pytest.approx(round(-.08 * 4496.3) / 4496.3)
    assert obj.angle_left == pytest.approx(round(.06 * 4496.3) / 4496.3)

  def test_counter_mixing_and_incomplete_cycles_are_unavailable(self):
    for omit, override in [((0x73a,), None), ((), {0x73b: 4})]:
      cache = BoschCameraCycleCache()
      feed(cache, 1_000_000_000, 3, [self.OBJ], omit=omit, counter_override=override)
      assert cache.snapshot(1_010_000_000) is None

  def test_dense_count_and_occupied_slot_must_agree(self):
    cache = BoschCameraCycleCache()
    bad = list(self.OBJ)
    bad[0] = bad[1] = bad[2] = bad[3] = 0
    feed(cache, 1_000_000_000, 0, [tuple(bad)])
    assert cache.snapshot(1_010_000_000) is None

  def test_id_episode_survives_slot_handoff_but_not_two_missing_cycles(self):
    cache = BoschCameraCycleCache()
    feed(cache, 1_000_000_000, 0, [self.OBJ])
    first = cache.snapshot(1_001_000_000)[0][0].episode
    other = (9, 18., 0., 0., 1.7, 2, 0, -.1, .1)
    feed(cache, 1_040_000_000, 1, [other, self.OBJ])
    moved = cache.snapshot(1_041_000_000)
    assert moved[0][1].episode == first
    feed(cache, 1_080_000_000, 2, [other])
    feed(cache, 1_120_000_000, 3, [other])
    feed(cache, 1_160_000_000, 4, [self.OBJ])
    assert cache.snapshot(1_161_000_000)[0][0].episode != first

  def test_future_and_stale_snapshots_are_never_selected(self):
    cache = BoschCameraCycleCache()
    feed(cache, 1_000_000_000, 0, [self.OBJ])
    assert cache.snapshot(999_999_999) is None
    assert cache.snapshot(1_160_000_000) is not None
    assert cache.snapshot(1_160_000_001) is None


class TestBoschCameraAssociationAndGeometry:
  def test_retained_scan_camera_scalars_survive_ring_reuse_and_expire(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_PUBLICATION)
    start = 1_000_000_000
    initial = (7, 14., 0., 0., 2.5, 1, 0, -.06, .06)
    for cycle in range(2):
      feed(grouping.camera, start + cycle * 40_000_000, cycle, [initial])
    scan = start + 50_000_000
    objects = (physical(1_000_100, 12.5, ns=scan), physical(1_000_101, 18.5, ns=scan))
    grouping.update(scan, objects, 10.)
    episode, retained = next(iter(grouping.last_camera_by_episode.items()))
    scan_values = replace(retained)
    changed = (7, 15., .25, 1., 1.9, 6, 0, -.05, .05)
    for cycle in range(2, 7):
      feed(grouping.camera, start + cycle * 40_000_000, cycle, [changed])
    # Reusing the producer's ring must not change the association's scan values.
    assert grouping.last_camera_by_episode[episode] == scan_values
    assert retained == scan_values
    grouping.update(start + 500_000_000, (), 10.)
    assert grouping.last_camera_by_episode == {}

  def test_a0_many_to_one_and_ambiguous(self):
    # cam 20 m, 폭 1.70 m. 창은 양쪽 모두 1.75 + 0.12*d_rel 이고, 더 이상
    # "카메라가 더 멀 때만" 허용하지 않는다.
    cam = BoschCameraObject(7, 11, 20., 0., 0., 1.7, 1, .1, -.1)
    nearer = physical(1_000_001, 17.5)    # d_long +2.5, 창 3.85
    farther = physical(1_000_002, 23.5)   # d_long -3.5, 창 4.57  (구 gate는 기각했다)
    assert BoschCameraExtendedGrouping._associate(nearer, [cam], 1) == (BOSCH_CAMERA_ASSOC_ASSIGNED, 11, 1)
    assert BoschCameraExtendedGrouping._associate(farther, [cam], 1) == (BOSCH_CAMERA_ASSOC_ASSIGNED, 11, 1)
    twin = BoschCameraObject(8, 12, 20., 0., 0., 1.7, 1, .1, -.1)
    assert BoschCameraExtendedGrouping._associate(nearer, [cam, twin], 2)[0] == BOSCH_CAMERA_ASSOC_AMBIGUOUS
    # 양쪽 바깥은 여전히 기각된다
    assert BoschCameraExtendedGrouping._associate(physical(1_000_003, 15.),
                                                  [cam], 1)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED
    assert BoschCameraExtendedGrouping._associate(physical(1_000_004, 26.),
                                                  [cam], 1)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_range_proportional_window_and_width_asymmetry(self):
    # 단안 거리 오차는 비례하므로 창도 비례한다
    assert _bosch_camera_range_span(0.) == pytest.approx(1.75)
    assert _bosch_camera_range_span(20.) == pytest.approx(4.15)
    assert _bosch_camera_range_span(100.) == pytest.approx(13.75)
    # 승용차 폭에서는 대칭이다
    assert _bosch_camera_long_window(_bosch_camera_range_span(20.), 1.70) == \
        pytest.approx((4.15, 4.15))
    assert _bosch_camera_long_window(_bosch_camera_range_span(20.), 1.55) == \
        pytest.approx((4.15, 4.15))
    # 대형차는 가까운 쪽이 훨씬 넓다: 레이더가 차체 깊은 곳의 return을 유지한다
    l_pos, l_neg = _bosch_camera_long_window(_bosch_camera_range_span(20.), 2.45)
    assert (l_pos, l_neg) == pytest.approx((7.15, 16.15))
    assert l_neg > l_pos
    # 창에는 상한이 있다
    assert _bosch_camera_long_window(_bosch_camera_range_span(200.), 3.0) == \
        pytest.approx((BOSCH_CAMERA_LONG_MAX_M, BOSCH_CAMERA_LONG_MAX_M))

  def test_large_vehicle_return_depth_stays_one_object(self):
    # 폭 2.45 m 대형차: 후면 20 m, 차체 깊은 return 34 m 까지 같은 객체다
    cam = BoschCameraObject(7, 11, 20., 0., 0., 2.45, 1, .1, -.1)
    for d_rel in (14.0, 20.0, 27.0, 34.0):
      assert BoschCameraExtendedGrouping._associate(physical(1_000_001, d_rel),
                                                    [cam], 1)[0] == BOSCH_CAMERA_ASSOC_ASSIGNED
    # 그 바깥은 기각된다. 창은 각 member의 d_rel에서 계산되므로 12.5 m 에서는
    # l_pos 6.25 < 7.5, 40 m 에서는 l_neg 18.55 < 20.0 이다.
    assert BoschCameraExtendedGrouping._associate(physical(1_000_002, 12.5),
                                                  [cam], 1)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED
    assert BoschCameraExtendedGrouping._associate(physical(1_000_003, 40.0),
                                                  [cam], 1)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  @pytest.mark.parametrize(('d_rel', 'assigned'), ((2.0, True), (12.0, True), (80.0, True)))
  def test_camera_may_read_nearer_than_the_radar_at_every_range(self, d_rel, assigned):
    # 교정된 스케일에서 참 쌍의 약 1/3은 카메라가 레이더보다 가깝다.
    # 구 gate(l_neg = 0)는 그것을 전부 기각했다.
    span = _bosch_camera_range_span(d_rel)
    cam = BoschCameraObject(7, 11, d_rel - span * 0.5, 0., 0., 1.7, 1, .1, -.1)
    verdict = BoschCameraExtendedGrouping._associate(physical(1_000_001, d_rel), [cam], 1)
    assert (verdict[0] == BOSCH_CAMERA_ASSOC_ASSIGNED) is assigned

  def test_lateral_gate_is_one_and_three_quarter_metres(self):
    cam = BoschCameraObject(7, 11, 20., 0., 0., 1.7, 1, .5, -.5)
    inside = physical(1_000_001, 20., y_rel=-(BOSCH_CAMERA_LAT_MAX_M - 0.05))
    outside = physical(1_000_002, 20., y_rel=-(BOSCH_CAMERA_LAT_MAX_M + 0.05))
    assert BoschCameraExtendedGrouping._associate(inside, [cam], 1)[0] == BOSCH_CAMERA_ASSOC_ASSIGNED
    assert BoschCameraExtendedGrouping._associate(outside, [cam], 1)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_bearing_gate_and_velocity_are_unchanged(self):
    # bearing 창(0.020 rad)은 스케일과 무관하며 이번 변경에서 손대지 않았다
    cam = BoschCameraObject(7, 11, 20., 0., 0., 1.7, 1, .02, .01)
    inside = physical(1_000_001, 20., y_rel=-20. * 0.0199)
    outside = physical(1_000_002, 20., y_rel=-20. * 0.0401)
    assert BoschCameraExtendedGrouping._associate(inside, [cam], 1)[0] == BOSCH_CAMERA_ASSOC_ASSIGNED
    assert BoschCameraExtendedGrouping._associate(outside, [cam], 1)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED
    # 속도에는 hard gate가 없다: 큰 dv도 기각되지 않는다(비용에만 들어간다)
    fast = BoschCameraObject(9, 13, 20., 0., 12., 1.7, 1, .1, -.1)
    assert BoschCameraExtendedGrouping._associate(physical(1_000_003, 20.),
                                                  [fast], 1)[0] == BOSCH_CAMERA_ASSOC_ASSIGNED

  @pytest.mark.parametrize(('dd', 'expected'), ((3.0, False), (3.25, True), (12.0, True), (12.01, False)))
  def test_distance_boundaries(self, dd, expected):
    assert BoschCameraExtendedGrouping._geometry(physical(1_000_001, 20.), physical(1_000_002, 20. + dd), 10., 0.)[3] is expected

  @pytest.mark.parametrize(('dy', 'dv', 'expected'), ((1.5, 1.5, True), (1.5001, 0., False), (0., 1.5001, False)))
  def test_lateral_velocity_boundaries(self, dy, dv, expected):
    assert BoschCameraExtendedGrouping._geometry(physical(1_000_001, 20., 0., 0.), physical(1_000_002, 24., dy, dv), 10., 0.)[3] is expected

  def test_stationary_moving_conflict(self):
    a = physical(1_000_001, 20., v_rel=-10.)
    b = physical(1_000_002, 24., v_rel=-8.5)
    assert not BoschCameraExtendedGrouping._geometry(a, b, 10., 0.)[3]

  def test_complete_link_rejects_single_link_chain(self):
    objects = (physical(1_000_001, 10.), physical(1_000_002, 14.), physical(1_000_003, 18.))
    edges = {(1_000_001, 1_000_002): 1., (1_000_002, 1_000_003): 1.}
    groups = BoschCameraExtendedGrouping._complete_link(objects, edges, ())
    assert max(map(len, groups)) == 2

  def test_complete_link_without_edges_returns_ordered_singletons(self):
    objects = (physical(1_000_003, 18.), physical(1_000_001, 10.), physical(1_000_002, 14.))
    previous = ({1_000_001, 1_000_002},)
    assert BoschCameraExtendedGrouping._complete_link(objects, {}, previous) == [
      {1_000_001}, {1_000_002}, {1_000_003},
    ]


class TestBoschCameraCurveReacquire:
  NS = 1_000_000_000
  PID = 1_000_001
  CAMERA_ID = 238
  EPISODE = 293

  @staticmethod
  def grouping(mode=BOSCH_CAMERA_CURVE_REACQUIRE_ACTIVE):
    return BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE, mode)

  def target(self, *, pid=PID, d_rel=85., y_rel=5.7, oem=True, ns=None):
    return replace(physical(pid, d_rel, y_rel=y_rel, ns=self.NS if ns is None else ns),
                   oem_selected=oem)

  def camera(self, *, obj_id=CAMERA_ID, episode=EPISODE, long_m=85., lat_m=-7.7,
             left=-.09, right=-.04, width=1.65, cls=1):
    return BoschCameraObject(obj_id, episode, long_m, lat_m, 0., width, cls, left, right)

  def seed(self, grouping, *, pid=PID, obj_id=CAMERA_ID, episode=EPISODE, ns=NS):
    grouping.curve_reacquire_histories[pid] = _BoschCurveReacquireHistory(obj_id, episode, ns)

  def verdict(self, *, target=None, camera=None, yaw=.045, age_ns=100_000_000,
              grouping=None, cameras=None):
    grouping = grouping or self.grouping()
    self.seed(grouping)
    camera = camera or self.camera()
    cameras = cameras or [camera]
    return grouping._curve_reacquire(target or self.target(), cameras, len(cameras),
                                     self.NS + age_ns, yaw)

  def test_reacquires_the_frozen_long_range_curve_near_miss(self):
    assert self.verdict() == (BOSCH_CAMERA_ASSOC_ASSIGNED, self.EPISODE, 1)

  def test_no_recent_assigned_history_does_not_reacquire(self):
    grouping = self.grouping()
    assert grouping._curve_reacquire(self.target(), [self.camera()], 1,
                                     self.NS + 100_000_000, .045)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_word1_on_a_different_pid_does_not_reacquire(self):
    assert self.verdict(target=self.target(oem=False))[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_inactive_word1_does_not_reacquire(self):
    assert self.verdict(target=self.target(oem=False))[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  @pytest.mark.parametrize(('age_ns', 'assigned'), (
    (BOSCH_CAMERA_CURVE_REACQUIRE_HOLD_NS, True),
    (BOSCH_CAMERA_CURVE_REACQUIRE_HOLD_NS + 1, False),
  ))
  def test_history_timeout_boundary(self, age_ns, assigned):
    verdict = self.verdict(age_ns=age_ns)
    assert (verdict[0] == BOSCH_CAMERA_ASSOC_ASSIGNED) is assigned

  def test_camera_episode_change_does_not_reacquire(self):
    assert self.verdict(camera=self.camera(episode=self.EPISODE + 1))[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_stable_camera_id_change_does_not_reacquire(self):
    assert self.verdict(camera=self.camera(obj_id=self.CAMERA_ID + 1))[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_bosch_pid_change_does_not_reacquire(self):
    assert self.verdict(target=self.target(pid=self.PID + 1))[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  @pytest.mark.parametrize('d_rel', (69.999, 100.0))
  def test_range_boundaries_fail_open(self, d_rel):
    target = self.target(d_rel=d_rel)
    camera = self.camera(long_m=d_rel, lat_m=-(target.y_rel + 2.0))
    assert self.verdict(target=target, camera=camera)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_weak_curve_does_not_reacquire(self):
    assert self.verdict(yaw=.014999)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_longitudinal_gate_failure_does_not_reacquire(self):
    assert self.verdict(camera=self.camera(long_m=110.))[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_lateral_envelope_failure_does_not_reacquire(self):
    assert self.verdict(camera=self.camera(lat_m=-(5.7 + 2.351)))[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_bearing_envelope_failure_does_not_reacquire(self):
    camera = self.camera(left=-.03, right=-.02)
    assert self.verdict(camera=camera)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_competing_exact_camera_records_fail_open(self):
    camera = self.camera()
    assert self.verdict(cameras=[camera, replace(camera)])[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_t33_clone_like_distinct_pid_has_no_inherited_history(self):
    t33 = self.target(pid=1_000_294, y_rel=-12., oem=False)
    assert self.verdict(target=t33)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_timestamp_reverse_clears_history(self):
    grouping = self.grouping()
    self.seed(grouping)
    grouping.last_ns = self.NS + 100_000_000
    grouping.camera.snapshot = lambda ns: ([self.camera()], 1, 0, ns)
    grouping.update(self.NS, (self.target(ns=self.NS),), 20., .045)
    assert not grouping.curve_reacquire_histories

  def test_camera_stale_gap_clears_history(self):
    grouping = self.grouping()
    self.seed(grouping)
    grouping.last_ns = self.NS
    grouping.update(self.NS + 200_000_000, (self.target(ns=self.NS + 200_000_000),), 20., .045)
    assert not grouping.curve_reacquire_histories

  def test_new_provider_segment_or_replay_reset_starts_empty(self):
    old = self.grouping()
    self.seed(old)
    assert old.curve_reacquire_histories
    assert not self.grouping().curve_reacquire_histories

  def test_pid_removal_clears_history(self):
    grouping = self.grouping()
    self.seed(grouping)
    grouping._curve_history_prepare(self.NS + 100_000_000, {}, [self.camera()], 1)
    assert not grouping.curve_reacquire_histories

  def test_word1_switch_releases_immediately(self):
    assert self.verdict(target=self.target(oem=False))[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_baseline_cut_in_association_is_unchanged(self):
    for y_rel in (2.4, 1.8, 1.2, .6, 0.):
      camera = self.camera(long_m=24.6, lat_m=-y_rel, left=-.15, right=.15, cls=2)
      target = self.target(d_rel=25., y_rel=y_rel)
      assert BoschCameraExtendedGrouping._associate(target, [camera], 1) == \
        (BOSCH_CAMERA_ASSOC_ASSIGNED, self.EPISODE, 2)

  def test_large_vehicle_baseline_association_is_unchanged(self):
    camera = self.camera(long_m=20., lat_m=0., left=-.1, right=.1, width=2.45)
    for d_rel in (14., 20., 27., 34.):
      assert BoschCameraExtendedGrouping._associate(self.target(d_rel=d_rel, y_rel=0.),
                                                    [camera], 1)[0] == BOSCH_CAMERA_ASSOC_ASSIGNED

  def test_only_baseline_assigned_seeds_and_active_does_not_refresh(self):
    grouping = self.grouping()
    baseline_camera = self.camera(lat_m=-5.7)
    grouping.camera.snapshot = lambda ns: ([baseline_camera], 1, 0, ns)
    target = self.target(ns=self.NS)
    partner = self.target(pid=self.PID + 1, d_rel=90., y_rel=5.9, oem=False, ns=self.NS)
    grouping.update(self.NS, (target, partner), 20., .045)
    assert grouping.curve_reacquire_histories[self.PID].last_assigned_ns == self.NS

    near_miss = self.camera()
    grouping.camera.snapshot = lambda ns: ([near_miss], 1, 1, ns)
    target = self.target(ns=self.NS + 100_000_000)
    partner = self.target(pid=self.PID + 1, d_rel=90., y_rel=5.9, oem=False,
                          ns=self.NS + 100_000_000)
    grouping.update(self.NS + 100_000_000, (target, partner), 20., .045)
    assert grouping.last_baseline_associations[self.PID][0] == BOSCH_CAMERA_ASSOC_UNRESOLVED
    assert grouping.last_associations[self.PID][0] == BOSCH_CAMERA_ASSOC_ASSIGNED
    assert grouping.curve_reacquire_histories[self.PID].last_assigned_ns == self.NS

  @pytest.mark.parametrize('mode', (BOSCH_CAMERA_CURVE_REACQUIRE_OFF,
                                    BOSCH_CAMERA_CURVE_REACQUIRE_SHADOW))
  def test_off_and_shadow_do_not_change_the_baseline_verdict(self, mode):
    grouping = self.grouping(mode)
    self.seed(grouping)
    camera = self.camera()
    grouping.camera.snapshot = lambda ns: ([camera], 1, 0, ns)
    target = self.target(ns=self.NS + 100_000_000)
    partner = self.target(pid=self.PID + 1, d_rel=90., y_rel=5.9, oem=False,
                          ns=self.NS + 100_000_000)
    grouping.update(self.NS + 100_000_000, (target, partner), 20., .045)
    assert grouping.last_associations[self.PID][0] == BOSCH_CAMERA_ASSOC_UNRESOLVED
    if mode == BOSCH_CAMERA_CURVE_REACQUIRE_SHADOW:
      assert grouping.last_curve_reacquire_would == ((self.PID, self.CAMERA_ID, self.EPISODE),)


class TestBoschCameraE2Overlay:
  @staticmethod
  def statuses(grouping, mapping):
    grouping._associate = lambda obj, *_: mapping.get(obj.physical_track_id, (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
    grouping.camera.snapshot = lambda ns: ([BoschCameraObject()], 1, 0, ns)

  def test_off_is_exact_identity_and_has_no_camera_state(self):
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_OFF)
    assert grouping.camera is None
    assert grouping.update(1_000_000_000, objects, 10.) is objects

  def test_shadow_and_active_preserve_observations_while_grouping(self):
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    mapping = {pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for pid in (1_000_001, 1_000_002)}
    shadow = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_SHADOW)
    self.statuses(shadow, mapping)
    assert shadow.update(1_000_000_000, objects, 10.) is objects
    assert shadow.last_groups == ((1_000_001, 1_000_002),)
    active = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    self.statuses(active, mapping)
    output = active.update(1_000_000_000, objects, 10.)
    assert output is objects
    assert active.last_groups == shadow.last_groups
    self.statuses(active, {})
    split = active.update(1_100_000_000, objects, 10.)
    assert len(split) == 2

  @pytest.mark.parametrize(('elapsed', 'held'), ((249_000_000, True), (250_000_000, True), (250_000_001, False)))
  def test_e2_timeout_is_exact_nanoseconds(self, elapsed, held):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    confirmed = {pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for pid in (1_000_001, 1_000_002)}
    self.statuses(grouping, confirmed)
    grouping.update(1_000_000_000, objects, 10.)
    now = 1_000_000_000
    while now + 100_000_000 < 1_000_000_000 + elapsed:
      now += 100_000_000
      self.statuses(grouping, {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)})
      grouping.update(now, objects, 10.)
    self.statuses(grouping, {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)})
    output = grouping.update(1_000_000_000 + elapsed, objects, 10.)
    assert output is objects
    assert bool(grouping.last_groups) is held

  @pytest.mark.parametrize(('gap', 'held'), ((159_000_000, True), (160_000_000, True), (160_000_001, False)))
  def test_observation_gap_boundary(self, gap, held):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    self.statuses(grouping, {pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for pid in (1_000_001, 1_000_002)})
    grouping.update(1_000_000_000, objects, 10.)
    self.statuses(grouping, {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)})
    assert grouping.update(1_000_000_000 + gap, objects, 10.) is objects
    assert bool(grouping.last_groups) is held

  @pytest.mark.parametrize('veto', ['different', 'ambiguous', 'class', 'geometry', 'no_anchor'])
  def test_e2_current_hard_vetoes(self, veto):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.))
    self.statuses(grouping, {pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for pid in (1_000_001, 1_000_002)})
    grouping.update(1_000_000_000, objects, 10.)
    mapping = {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)}
    current = objects
    if veto == 'different':
      mapping[1_000_002] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 8, 1)
    elif veto == 'ambiguous':
      mapping[1_000_002] = (BOSCH_CAMERA_ASSOC_AMBIGUOUS, -1, -1)
    elif veto == 'class':
      mapping[1_000_001] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 2)
    elif veto == 'geometry':
      current = (objects[0], physical(1_000_002, 27.1))
    elif veto == 'no_anchor':
      mapping = {}
    self.statuses(grouping, mapping)
    assert grouping.update(1_100_000_000, current, 10.) is current
    assert grouping.last_groups == ()

  def test_coast_never_recruits_a_new_member(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 15.), physical(1_000_002, 19.), physical(1_000_003, 23.))
    self.statuses(grouping, {
      1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
      1_000_002: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
    })
    grouping.update(1_000_000_000, objects, 10.)
    self.statuses(grouping, {
      1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
      1_000_003: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
    })
    output = grouping.update(1_100_000_000, objects, 10.)
    assert grouping.last_groups == ((1_000_001, 1_000_002),)
    assert output is objects


class TestBoschCameraPublicationSafety:
  @staticmethod
  def group(overlay, objects, ns):
    TestBoschCameraE2Overlay.statuses(overlay, {
      obj.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for obj in objects
    })
    return overlay.update(ns, objects, 10.)

  def test_original_representative_preserves_s12_oem_tie_policy(self):
    ns = 1_000_000_000
    near = replace(physical(1_000_002, 13., .5, 2., ns, 1), vision_supported=True)
    far = replace(physical(1_000_001, 16.25, .46875, 2., ns, 43), vision_supported=True, oem_selected=True)
    overlay = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    output = self.group(overlay, (far, near), ns)
    assert output == (far, near)
    assert overlay.representatives[0].representative_pid == far.physical_track_id
    from opendbc.car import structs
    data = structs.RadarData.new_message()
    bosch_append_points(data, output, 10., ns + 10_000_000)
    assert data.points[1].dRel == pytest.approx(13.02, abs=1e-6)
    assert (data.points[1].yRel, data.points[1].vRel) == (near.y_rel, near.v_rel)
    assert data.points[0].dRel == pytest.approx(16.27, abs=1e-6)

  def test_a_nearer_member_takes_the_representative_from_a_farther_one(self):
    # publication_view hides every other member of a mature group, so the
    # nearest surface of the vehicle is the one downstream has to keep. A
    # nearer member joining the group takes the representative even though
    # continuity would have held the previous farther one.
    overlay = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    first = (physical(1_000_001, 16.25), physical(1_000_003, 20.))
    assert self.group(overlay, first, 1_000_000_000) is first
    assert overlay.representatives[0].representative_pid == first[0].physical_track_id
    current = (physical(1_000_001, 16.25, ns=1_100_000_000), physical(1_000_002, 13., ns=1_100_000_000))
    assert self.group(overlay, current, 1_100_000_000) is current
    assert overlay.representatives[0].representative_pid == current[1].physical_track_id
    assert overlay.representative_switches == 1

  def test_range_order_inside_a_group_cannot_chatter(self):
    # Every pair in a group passed 3.0 < dd <= 12.0, so the members are at
    # least 3 m apart and quantisation cannot reorder them. Walk a rigid pair
    # through a closing approach and confirm the representative never moves.
    overlay = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(30):
      ns = 1_000_000_000 + i * 100_000_000
      near = physical(1_000_002, 30. - .25 * i, .0625 * (i % 3), -2.5, ns)
      far = physical(1_000_001, 36.5 - .25 * i, -.0625 * (i % 2), -2.5, ns)
      assert self.group(overlay, (far, near), ns) == (far, near)
      assert overlay.representatives[0].representative_pid == near.physical_track_id
    assert overlay.representative_switches == 0

  def test_reordered_members_do_not_chatter(self):
    overlay = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      members = (physical(1_000_001, 13. + i * .25, ns=ns), physical(1_000_002, 16.25 + i * .25, ns=ns))
      ordered = members if i % 2 else members[::-1]
      assert self.group(overlay, ordered, ns) is ordered
      assert overlay.representatives[0].representative_pid == members[0].physical_track_id
    assert overlay.representative_switches == 0

  @pytest.mark.parametrize('mode', (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_SHADOW, BOSCH_CAMERA_EXTENDED_ACTIVE))
  def test_provider_qualifier_receives_complete_baseline_tuple(self, monkeypatch, mode):
    provider = BoschRadarProvider(1, camera_extended_mode=mode)
    objects = (physical(1_000_001, 13.), physical(1_000_002, 16.25))
    provider.tracker.update(900_000_000, (), v_ego=10.)
    monkeypatch.setattr(provider.tracker, 'update', lambda *a, **kw: objects)
    if mode != BOSCH_CAMERA_EXTENDED_OFF:
      TestBoschCameraE2Overlay.statuses(provider.camera_extended, {
        obj.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for obj in objects})
    seen = []
    original = provider.qualifier.update
    def qualify(got, *a, **kw):
      seen.append(got)
      return original(got, *a, **kw)
    monkeypatch.setattr(provider.qualifier, 'update', qualify)
    output = provider._finish_scan(1_000_000_000, 0, (), 10., 0., ())
    assert seen == [objects] and seen[0] is objects
    assert output is objects
    view = provider.publication_view(output)
    assert view is output

  def test_missing_qualified_representative_preserves_other_points(self):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 13.), physical(1_000_002, 16.25))
    self.group(provider.camera_extended, objects, 1_000_000_000)
    qualified = (objects[1],)
    assert provider.publication_view(qualified) is qualified

  @pytest.mark.parametrize('disappear_reappear', (False, True))
  def test_native_boundary_preserves_fifo_and_every_member_history_identity(self, monkeypatch, disappear_reappear):
    from opendbc.car import structs
    import opendbc.car.hyundai.radar_interface as module
    def base_update(*args):
      data = structs.RadarData.new_message()
      data.points = [dict(trackId=0, dRel=80., vRel=0., radarSource='scc', measured=True)]
      return data
    monkeypatch.setattr(module.RadarInterfaceBase, 'update_carrot', base_update)
    interfaces = []
    for mode in (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_ACTIVE):
      interface = RadarInterface.__new__(RadarInterface)
      interface.bosch = BoschRadarProvider(1, qualification=False, camera_extended_mode=mode)
      interface.v_ego = 10.
      interfaces.append(interface)
    unrelated_alias = None
    preserved_samples = 0
    for i in range(24):
      ns = 1_000_000_000 + i * 100_000_000
      objects = [physical(1_000_001, 13., ns=ns), physical(1_010_690, 30., 3., ns=ns)]
      if not disappear_reappear or i not in (8, 9, 10):
        objects.append(physical(1_000_002, 16.25, ns=ns))
      if i >= 5:
        objects.append(physical(1_001_000 + i // 3, 60., -4., ns=ns))
      objects = tuple(objects)
      outputs = []
      for interface in interfaces:
        p = interface.bosch
        p.tracker.group_manager.states = {o.physical_track_id: None for o in objects}
        p.last_scan_timestamp_ns = interface._bosch_now_ns = ns
        interface._bosch_objects = objects
        if p.camera_extended.mode == BOSCH_CAMERA_EXTENDED_ACTIVE:
          self.group(p.camera_extended, objects, ns)
        outputs.append(interface.update_carrot(10., 0., ns * 1e-9, []))
      a, b = (x.bosch.publication_aliases for x in interfaces)
      assert a.physical_to_alias == b.physical_to_alias
      assert a.last_published_ns == b.last_published_ns
      assert list(a.free_aliases) == list(b.free_aliases)
      alias = a.physical_to_alias[1_010_690]
      unrelated_alias = alias if unrelated_alias is None else unrelated_alias
      assert alias == unrelated_alias
      for output in outputs:
        assert output.points[0].to_dict() == base_update().points[0].to_dict()
        assert any(p.trackId == alias and p.dRel == 30. for p in output.points)
      if 1_000_002 in a.physical_to_alias:
        duplicate_alias = a.physical_to_alias[1_000_002]
        assert any(p.trackId == duplicate_alias for p in outputs[1].points)
        preserved_samples += 1
      assert len(a.physical_to_alias) <= 64
    assert preserved_samples > 10


class TestBoschCameraConservativePublication:
  @pytest.mark.parametrize('scans', (1, 2, 3, 4, 5, 6, 50))
  def test_group_age_never_authorizes_history_loss(self, scans):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(scans):
      ns = 1_000_000_000 + i * 99_123_456
      objects = (physical(1_000_001, 15., ns=ns, age=i+1), physical(1_000_002, 19., ns=ns, age=i+1))
      assert TestBoschCameraPublicationSafety.group(provider.camera_extended, objects, ns) is objects
      assert provider.publication_view(objects) is objects
      assert provider.camera_extended.last_groups == ((1_000_001, 1_000_002),)
    assert provider.camera_extended.representatives[0].age == scans

  @pytest.mark.parametrize('split_after', (1, 2, 5, 50))
  def test_split_reappearance_has_no_camera_induced_observation_gap(self, split_after):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    seen = {1_000_001: [], 1_000_002: []}
    for i in range(split_after+3):
      ns = 1_000_000_000 + i * 100_000_000
      objects = (physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns, age=i+1))
      mapping = {o.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for o in objects} if i < split_after else {}
      TestBoschCameraE2Overlay.statuses(provider.camera_extended, mapping)
      assert provider.camera_extended.update(ns, objects, 10.) is objects
      for obj in provider.publication_view(objects):
        seen[obj.physical_track_id].append(ns)
    assert seen[1_000_001] == seen[1_000_002]
    assert len(seen[1_000_002]) == split_after+3
    assert provider.camera_extended.histories == {}

  @pytest.mark.parametrize(('near_y', 'far_y', 'near_v', 'far_v'), (
    (-2.5625, -1.96875, -6., -5.75), (1.09375, -.09375, 2., 2.), (0., 0., 0., 0.)))
  def test_geometry_and_identical_lateral_values_do_not_authorize_pid_collapse(self, near_y, far_y, near_v, far_v):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    objects = (physical(1_000_001, 19.5, near_y, near_v), physical(1_000_002, 24., far_y, far_v))
    TestBoschCameraPublicationSafety.group(provider.camera_extended, objects, 1_000_000_000)
    assert provider.camera_extended.last_groups
    assert provider.publication_view(objects) is objects

  def test_new_member_replaces_group_without_hiding_its_first_observation(self):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(8):
      ns = 1_000_000_000 + i * 100_000_000
      other = 1_000_002 if i < 6 else 1_000_003
      objects = (physical(1_000_001, 15., ns=ns), physical(other, 19., ns=ns, age=1 if i==6 else 4))
      TestBoschCameraPublicationSafety.group(provider.camera_extended, objects, ns)
      assert provider.publication_view(objects) is objects
      assert tuple(provider.camera_extended.histories) == ((1_000_001, other),)
    assert provider.camera_extended.max_state_count == 1


class TestBoschActiveTestPublication:
  @staticmethod
  def scan(provider, ns, objects=None, mapping=None):
    objects = objects or (physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns))
    ext = provider.camera_extended
    if ext.camera is not None:
      TestBoschCameraE2Overlay.statuses(ext, mapping if mapping is not None else {
        o.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for o in objects})
    assert ext.update(ns, objects, 10.) is objects
    return objects, provider.publication_view(objects)

  @pytest.mark.parametrize('mode', (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_SHADOW,
                                   BOSCH_CAMERA_EXTENDED_ACTIVE, BOSCH_CAMERA_EXTENDED_PUBLICATION))
  def test_class1_one_completed_interval_only_test_mode_suppresses(self, mode):
    # Both members carry a camera class-1 verdict, which already ties them to one
    # body: one stable interval suffices (class-6 truck-P2 sets still need two).
    p = BoschRadarProvider(1, camera_extended_mode=mode)
    for i, ns in enumerate((1_000_000_000, 1_099_123_456, 1_198_765_432, 1_299_000_000)):
      objects, view = self.scan(p, ns)
      assert len(view) == (1 if mode == BOSCH_CAMERA_EXTENDED_PUBLICATION and i >= 1 else 2)
      if i < 1 or mode != BOSCH_CAMERA_EXTENDED_PUBLICATION:
        assert view is objects
    if mode == BOSCH_CAMERA_EXTENDED_PUBLICATION:
      assert p.camera_extended.histories[(1_000_001, 1_000_002)].stable_intervals == 2

  COAST = {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
           1_000_002: (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1)}

  def test_coast_keeps_an_already_mature_group_collapsed_onto_its_nearest_member(self):
    # A scan that keeps the group through the E2 coast has no strict camera
    # edge. A group that was already mature stays collapsed while the motion
    # still tracks, so one missing camera confirmation no longer re-splits a
    # large vehicle. The accumulated maturity also survives the coast.
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    members = (1_000_001, 1_000_002)
    for i in range(3):
      _, view = self.scan(p, 1_000_000_000 + i * 100_000_000)
    assert len(view) == 1 and p.camera_extended.histories[members].stable_intervals == 2
    ns = 1_300_000_000
    objects = (physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns))
    _, view = self.scan(p, ns, objects, self.COAST)
    assert [o.physical_track_id for o in view] == [1_000_001]   # nearest surface stays published
    assert p.camera_extended.histories[members].stable_intervals == 2
    _, view = self.scan(p, 1_400_000_000)
    assert len(view) == 1                                    # confirmed again, still mature

  def test_coast_never_makes_a_group_mature(self):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    for i in range(1):                                       # no completed interval: not mature yet
      _, view = self.scan(p, 1_000_000_000 + i * 100_000_000)
    assert len(view) == 2
    ns = 1_200_000_000
    objects = (physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns))
    _, view = self.scan(p, ns, objects, self.COAST)
    assert view is objects
    assert not p.camera_extended.mature_groups

  @pytest.mark.parametrize('change', ('stale_member', 'far_anchor'))
  def test_coast_opens_without_fresh_members_or_with_a_farther_representative(self, change):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    for i in range(3):
      self.scan(p, 1_000_000_000 + i * 100_000_000)
    ns = 1_300_000_000
    near, far = physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns)
    if change == 'stale_member':
      far = replace(far, timestamp_ns=ns - 1)
    else:
      # The OEM now selects the far member: collapsing onto it would hide the
      # nearer surface, which a coast must never do.
      far = replace(far, oem_selected=True)
    objects = (near, far)
    _, view = self.scan(p, ns, objects, self.COAST)
    assert view is objects
    assert not p.camera_extended.mature_groups

  def test_coast_beyond_the_hold_window_discards_maturity(self):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    members = (1_000_001, 1_000_002)
    for i in range(3):
      self.scan(p, 1_000_000_000 + i * 100_000_000)
    coast = {1_000_001: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
             1_000_002: (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1)}
    ns = 1_200_000_000
    views = []
    for _ in range(3):                                       # 300 ms of coasting
      ns += 100_000_000
      objects = (physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns))
      _, view = self.scan(p, ns, objects, coast)
      views.append((objects, view))
    # Inside BOSCH_CAMERA_MATURITY_HOLD_NS the mature group stays collapsed onto
    # its nearest member; past the hold it opens again.
    assert all(view is objects or [o.physical_track_id for o in view] == [1_000_001] for objects, view in views)
    assert views[-1][1] is views[-1][0]
    # past BOSCH_CAMERA_E2_HOLD_NS the coast drops the set, so the maturity is
    # gone either way: absent, or present with nothing accumulated.
    held = p.camera_extended.histories.get(members)
    assert held is None or held.stable_intervals == 0

  def test_a_stable_word1_anchor_matures_in_one_interval(self):
    # The OEM selected the same member of this exact set on the previous scan,
    # so one stable interval is enough. Without that the requirement stays two.
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    anchored = lambda ns: (replace(physical(1_000_001, 15., ns=ns), oem_selected=True),
                           physical(1_000_002, 19., ns=ns))
    _, view = self.scan(p, 1_000_000_000, anchored(1_000_000_000))
    assert view is not None and len(view) == 2
    _, view = self.scan(p, 1_100_000_000, anchored(1_100_000_000))
    assert len(view) == 1
    assert p.camera_extended.histories[(1_000_001, 1_000_002)].oem_anchor == 1_000_001

  def test_a_moving_word1_anchor_still_needs_two_intervals(self):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    first = (replace(physical(1_000_001, 15.), oem_selected=True), physical(1_000_002, 19.))
    _, view = self.scan(p, 1_000_000_000, first)
    assert len(view) == 2
    ns = 1_100_000_000
    moved = (physical(1_000_001, 15., ns=ns),
             replace(physical(1_000_002, 19., ns=ns), oem_selected=True))
    _, view = self.scan(p, ns, moved)
    assert len(view) == 2                                    # anchor changed: no acceleration
    _, view = self.scan(p, 1_200_000_000, (
      physical(1_000_001, 15., ns=1_200_000_000),
      replace(physical(1_000_002, 19., ns=1_200_000_000), oem_selected=True)))
    assert len(view) == 1

  @pytest.mark.parametrize('failure', ('tuple', 'missing', 'p2', 'ambiguous', 'class', 'episode',
                                      'g0', 'conflict', 'stale_member', 'stale_camera', 'future_camera',
                                      'motion_jump', 'gap', 'duplicate_ns'))
  def test_current_failure_resets_maturity_and_immediately_opens_publication(self, failure):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    for i in range(3):
      _, view = self.scan(p, 1_000_000_000 + i * 100_000_000)
    assert len(view) == 1
    ns = 1_300_000_000
    if failure == 'gap':
      ns = 1_360_000_001
    elif failure == 'duplicate_ns':
      ns = 1_200_000_000
    objects = (physical(1_000_001, 15., ns=ns), physical(1_000_002, 19., ns=ns))
    mapping = {o.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1) for o in objects}
    if failure == 'tuple':
      objects = (objects[0], physical(1_000_003, 19., ns=ns))
      mapping[1_000_003] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1)
    elif failure == 'missing':
      objects = (objects[0],)
    elif failure == 'p2':
      del mapping[1_000_002]
    elif failure == 'ambiguous':
      mapping[1_000_002] = (BOSCH_CAMERA_ASSOC_AMBIGUOUS, -1, -1)
    elif failure == 'class':
      mapping[1_000_002] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 2)
    elif failure == 'episode':
      mapping = {p: (BOSCH_CAMERA_ASSOC_ASSIGNED, 8, 1) for p in mapping}
    elif failure == 'g0':
      objects = (objects[0], physical(1_000_002, 19., 2., ns=ns))
    elif failure == 'conflict':
      objects = (replace(objects[0], v_rel=-10.), replace(objects[1], v_rel=-8.5))
    elif failure == 'stale_member':
      objects = (objects[0], replace(objects[1], timestamp_ns=ns - 1))
    elif failure == 'motion_jump':
      objects = tuple(replace(o, d_rel=o.d_rel + 1.5) for o in objects)
    ext = p.camera_extended
    TestBoschCameraE2Overlay.statuses(ext, mapping)
    if failure in ('stale_camera', 'future_camera'):
      cam_ns = ns - 160_000_001 if failure == 'stale_camera' else ns + 1
      ext.camera.snapshot = lambda _: ([BoschCameraObject()], 1, 0, cam_ns)
    ext.update(ns, objects, 10.)
    if failure == 'p2':
      # A missing strict edge with the set otherwise intact is a coast, not a
      # failure: inside BOSCH_CAMERA_MATURITY_HOLD_NS the accumulated maturity
      # is held, and the already-mature group stays collapsed onto its nearest
      # member (see test_coast_keeps_an_already_mature_group_collapsed...).
      assert [o.physical_track_id for o in p.publication_view(objects)] == [1_000_001]
      assert ext.last_maturity_resets == 0
      assert ext.histories[(1_000_001, 1_000_002)].stable_intervals == 2
    else:
      assert p.publication_view(objects) is objects
      assert not ext.mature_groups
      assert ext.last_maturity_resets == 1
      assert all(s.stable_intervals == 0 for s in ext.histories.values())

  @pytest.mark.parametrize('jitter, mature', ((.33, True), (.8, False)))
  def test_far_lateral_noise_scales_the_maturity_bound_with_range(self, jitter, mature):
    # A distant large body's bearing noise grows with range: 0.33 m of lateral
    # scatter at 65-72 m used to reset maturity every interval (fixed 0.26 m
    # bound). The range term admits it; scatter well beyond it still resets.
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    for i in range(4):
      ns = 1_000_000_000 + i * 100_000_000
      y = jitter if i % 2 else 0.
      objects = (physical(1_000_001, 65., y, ns=ns), physical(1_000_002, 72., y, ns=ns))
      _, view = self.scan(p, ns, objects)
    assert (len(view) == 1) is mature

  def test_missing_qualified_representative_or_member_is_fail_open(self):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    for i in range(3):
      objects, _ = self.scan(p, 1_000_000_000 + i * 100_000_000)
    for qualified in ((objects[0],), (objects[1],)):
      assert p.publication_view(qualified) is qualified
    stale = tuple(replace(o, timestamp_ns=1_100_000_000) for o in objects)
    assert p.publication_view(stale) is stale

  def test_real_decoder_m2_and_native_boundary_keep_scc_and_alias_reservation(self, monkeypatch):
    import opendbc.car.hyundai.radar_interface as module
    from opendbc.car import structs
    def baseline(*args):
      result = structs.RadarData.new_message()
      result.points = [dict(trackId=0, dRel=80., vRel=0., radarSource='scc', measured=True)]
      return result
    monkeypatch.setattr(module.RadarInterfaceBase, 'update_carrot', baseline)
    interfaces = []
    for mode in (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_PUBLICATION):
      ri = RadarInterface.__new__(RadarInterface)
      ri.bosch = BoschRadarProvider(1, qualification=False, camera_extended_mode=mode)
      ri.v_ego = 10.
      interfaces.append(ri)
    visible = []
    for i in range(9):
      ns = 1_000_000_000 + i * 100_000_000
      objects = (physical(1_000_001, 15., ns=ns, age=i+1), physical(1_010_690, 19., ns=ns, age=i+1))
      outputs = []
      for ri in interfaces:
        p = ri.bosch
        if p.camera_extended.camera is not None:
          # camera 공백으로 split/reappearance를 유발한 뒤 maturity를 다시 시작한다.
          # 카메라 거리는 교정된 0.05 스케일에서 member 15/19 m 와 정합하는 값이다
          # (구 0.0625 스케일에서는 같은 장면이 20 m 로 읽혔다).
          feed(p.camera_extended.camera, ns, i, [(7, 16.5, 0., 0., 1.7, 1, 0, -.1, .1)] if i != 5 else [])
        p.camera_extended.update(ns, objects, 10.)
        p.tracker.group_manager.states = {o.physical_track_id: None for o in objects}
        p.last_scan_timestamp_ns = ri._bosch_now_ns = ns
        ri._bosch_objects = objects
        outputs.append(ri.update_carrot(10., 0., ns * 1e-9, []))
      a, b = (ri.bosch.publication_aliases for ri in interfaces)
      assert a.physical_to_alias == b.physical_to_alias
      assert a.last_published_ns == b.last_published_ns
      assert list(a.free_aliases) == list(b.free_aliases)
      target = a.physical_to_alias[1_010_690]
      visible.append(any(point.trackId == target for point in outputs[1].points))
      for result in outputs:
        assert result.points[0].to_dict() == baseline().points[0].to_dict()
      assert target not in b.free_aliases
    # Class-1 sets collapse after one interval; the camera gap at i=5 splits the
    # set and the reappearance restarts with one interval.
    assert visible == [True, False, False, False, False, True, True, False, False]

  def test_route254_original_policy_keeps_24m_lateral_member(self):
    p = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    for i in range(3):
      ns = 1_000_000_000 + i * 100_000_000
      far = replace(physical(1_001_303, 24. - .575 * i, -1.96875, -5.75, ns), oem_selected=True)
      near = physical(1_001_321, 19.5 - .6 * i, -2.5625, -6., ns)
      _, view = self.scan(p, ns, (far, near))
    assert view == (far,)
    assert p.camera_extended.representatives[0].representative_pid == far.physical_track_id


class TestBoschTruckAwareP2:
  IDS = (1_004_581, 1_004_624)

  @staticmethod
  def statuses(grouping, ns, mapping, *, camera_id=203, episode=186, width=2.45,
               camera_d=24., camera_y=0., camera_v=0., camera_ns=None):
    grouping._associate = lambda obj, *_: mapping.get(obj.physical_track_id, (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
    camera = BoschCameraObject(camera_id, episode, camera_d, camera_y, camera_v, width, 6, .2, -.2)
    grouping.camera.snapshot = lambda _: ([camera], 1, 0, ns if camera_ns is None else camera_ns)

  @classmethod
  def objects(cls, ns, *, far_d=28., far_y=0., far_v=0., far_pid=None):
    return (physical(cls.IDS[0], 20., 0., 0., ns),
            physical(cls.IDS[1] if far_pid is None else far_pid, far_d, far_y, far_v, ns))

  @classmethod
  def assigned(cls, episode=186, class_code=6, far_pid=None):
    return {cls.IDS[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code),
            cls.IDS[1] if far_pid is None else far_pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code)}

  def test_class1_baseline_p2_is_still_immediate(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    objects = self.objects(ns)
    self.statuses(grouping, ns, self.assigned(class_code=1))
    assert grouping.update(ns, objects, 10.) is objects
    assert grouping.last_groups == (self.IDS,)
    assert grouping.truck_pair_histories == {}

  def test_class6_does_not_open_p2_immediately(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    self.statuses(grouping, ns, self.assigned())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == ()
    assert grouping.truck_pair_histories[self.IDS].confirmations == 1

  def test_same_episode_class6_and_current_g0_still_require_history(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS - 1):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
      assert grouping.last_groups == ()

  def test_stable_large_vehicle_opens_only_on_the_confirmation_threshold(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
      assert bool(grouping.last_groups) is (i == BOSCH_TRUCK_P2_CONFIRMATIONS - 1)
    assert grouping.last_groups == (self.IDS,)
    assert grouping.truck_pair_histories[self.IDS].confirmations == BOSCH_TRUCK_P2_CONFIRMATIONS

  @pytest.mark.parametrize(('reset', 'kwargs'), (
    ('episode', {'episode': 187}),
    ('camera_id', {'camera_id': 204}),
    ('camera_d', {'camera_d': 25.}),
    ('camera_y', {'camera_y': .25}),
    ('camera_v', {'camera_v': 1.}),
    ('camera_width', {'width': 2.60}),
  ))
  def test_camera_identity_or_motion_discontinuity_resets(self, reset, kwargs):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS - 1):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
    ns += 100_000_000
    mapping = self.assigned(episode=kwargs.get('episode', 186))
    self.statuses(grouping, ns, mapping, **kwargs)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == (), reset
    assert grouping.truck_pair_histories[self.IDS].confirmations == 1

  @pytest.mark.parametrize(('name', 'objects'), (
    ('dd', lambda self, ns: self.objects(ns, far_d=29.)),
    ('dy', lambda self, ns: self.objects(ns, far_y=.75)),
    ('dv', lambda self, ns: self.objects(ns, far_v=.5)),
  ))
  def test_radar_geometry_discontinuity_resets(self, name, objects):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS - 1):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns, far_d=28.), 10.)
    ns += 100_000_000
    self.statuses(grouping, ns, self.assigned())
    grouping.update(ns, objects(self, ns), 10.)
    assert grouping.last_groups == (), name
    assert grouping.truck_pair_histories[self.IDS].confirmations == 1

  def test_member_pid_change_starts_a_new_pair_at_one(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS - 1):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
    ns += 100_000_000
    new_pid = 1_004_625
    self.statuses(grouping, ns, self.assigned(far_pid=new_pid))
    grouping.update(ns, self.objects(ns, far_pid=new_pid), 10.)
    assert tuple(grouping.truck_pair_histories) == ((self.IDS[0], new_pid),)
    assert next(iter(grouping.truck_pair_histories.values())).confirmations == 1

  @pytest.mark.parametrize('failure', ('stale', 'ambiguous', 'class_change'))
  def test_stale_ambiguity_or_class_change_clears_pair_state(self, failure):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS - 1):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
    ns += 100_000_000
    mapping = self.assigned()
    camera_ns = None
    if failure == 'stale':
      camera_ns = ns - 160_000_001
    elif failure == 'ambiguous':
      mapping[self.IDS[1]] = (BOSCH_CAMERA_ASSOC_AMBIGUOUS, -1, -1)
    else:
      mapping[self.IDS[1]] = (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 2)
    self.statuses(grouping, ns, mapping, camera_ns=camera_ns)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == ()
    assert grouping.truck_pair_histories == {}

  def test_two_distinct_convoy_members_with_different_episodes_never_group(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    mapping = {self.IDS[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 6),
               self.IDS[1]: (BOSCH_CAMERA_ASSOC_ASSIGNED, 187, 6)}
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, mapping)
      grouping.update(ns, self.objects(ns), 10.)
      assert grouping.last_groups == ()

  def test_adjacent_vehicle_pair_fails_absolute_lateral_gate(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns, far_y=1.), 10.)
    assert grouping.last_groups == ()

  def test_cut_in_motion_change_resets_before_authorization(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns, far_y=.75 if i % 2 else 0.), 10.)
      assert grouping.last_groups == ()

  def test_narrow_class6_phantom_never_enters_pair_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned(), width=2.20)
      grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == ()
    assert grouping.truck_pair_histories == {}

  def test_class1_box_truck_m2_publication_sequence_is_exact(self):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    visible = []
    for i in range(6):
      ns = 1_000_000_000 + i * 100_000_000
      objects = self.objects(ns)
      self.statuses(provider.camera_extended, ns, self.assigned(class_code=1))
      provider.camera_extended.update(ns, objects, 10.)
      visible.append(len(provider.publication_view(objects)))
    assert visible == [2, 1, 1, 1, 1, 1]

  def test_class6_truck_p2_set_still_needs_two_intervals(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_PUBLICATION)
    mature = []
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS + 3):
      ns = 1_000_000_000 + i * 100_000_000
      self.statuses(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
      mature.append(bool(grouping.mature_groups))
    strict_at = BOSCH_TRUCK_P2_CONFIRMATIONS - 1
    assert mature.index(True) == strict_at + 2

  @pytest.mark.parametrize(('field', 'value', 'accepted'), (
    ('width', 2.25, True), ('width', 2.20, False),
    ('dd', 5.50, True), ('dd', 5.49, False), ('dd', 10.75, True), ('dd', 12.00, True), ('dd', 12.01, False),
    ('dy', .875, True), ('dy', .876, False), ('dv', .50, True), ('dv', .501, False),
  ))
  def test_absolute_truck_gate_boundaries(self, field, value, accepted):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS):
      ns = 1_000_000_000 + i * 100_000_000
      width = value if field == 'width' else 2.45
      far_d = 20. + value if field == 'dd' else 28.
      far_y = value if field == 'dy' else 0.
      far_v = value if field == 'dv' else 0.
      self.statuses(grouping, ns, self.assigned(), width=width)
      grouping.update(ns, self.objects(ns, far_d=far_d, far_y=far_y, far_v=far_v), 10.)
    assert bool(grouping.last_groups) is accepted

  def test_pair_state_is_bounded_and_retires_on_next_scan(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    objects = tuple(physical(1_000_000 + i, 10. + i, ns=ns) for i in range(24))
    mapping = {obj.physical_track_id: (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 6) for obj in objects}
    self.statuses(grouping, ns, mapping)
    grouping.update(ns, objects, 10.)
    assert len(grouping.truck_pair_histories) == 16
    ns += 100_000_000
    self.statuses(grouping, ns, {})
    grouping.update(ns, (), 10.)
    assert grouping.truck_pair_histories == {}


class TestBoschLargeFootprintAssociation:
  """The far member of a camera-confirmed large vehicle may miss only the A0
  bearing gate (front of a long body in an adjacent lane) and still form the
  strict pair, when nothing else explains it and it lies in the body footprint."""
  IDS = TestBoschTruckAwareP2.IDS

  def run(self, *, near_class=1, far_status=None, width=2.45, camera_d=24., camera_y=0., far_d=28., far_y=0.,
          far_v=0.):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    mapping = {self.IDS[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, near_class),
               self.IDS[1]: far_status or (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1)}
    TestBoschTruckAwareP2.statuses(grouping, ns, mapping, width=width, camera_d=camera_d, camera_y=camera_y)
    grouping.update(ns, TestBoschTruckAwareP2.objects(ns, far_d=far_d, far_y=far_y, far_v=far_v), 10.)
    return grouping

  def test_far_member_inside_the_footprint_forms_the_class1_pair(self):
    grouping = self.run()
    assert grouping.last_footprint_associations == (self.IDS[1],)
    assert grouping.last_groups == (self.IDS,)
    # The exported A0 verdicts are unchanged for every other consumer.
    assert grouping.last_associations[self.IDS[1]][0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  @pytest.mark.parametrize('case, kwargs', (
    ('beyond_body_length', {'camera_d': 15.}),             # 13 m behind the rear face
    ('lateral_outside_body', {'camera_y': .5, 'far_y': 1.4}),
    ('speed_differs', {'far_v': .75}),
    ('small_camera_object', {'near_class': 2}),
    ('narrow_class6', {'near_class': 6, 'width': 2.20}),
    ('explained_by_other_camera', {'far_status': (BOSCH_CAMERA_ASSOC_ASSIGNED, 187, 2)}),
    ('ambiguous_far', {'far_status': (BOSCH_CAMERA_ASSOC_AMBIGUOUS, -1, -1)}),
  ))
  def test_no_footprint_outside_its_conditions(self, case, kwargs):
    grouping = self.run(**kwargs)
    assert grouping.last_footprint_associations == (), case
    assert grouping.last_groups == (), case

  def test_footprint_precedes_a0_recovery_for_a_far_bearing_miss(self):
    # The A0-recovery near-miss geometry: far member 3 m behind the rear face of
    # a wide class-6 camera object. The footprint associates it at once, so the
    # truck-P2 pair starts counting confirmations without a recovery seed.
    recovery = TestBoschLargeVehicleA0Recovery
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    recovery.configure(grouping, ns, recovery.one_sided())
    grouping.update(ns, recovery.objects(ns), 10.)
    assert grouping.last_footprint_associations == (recovery.IDS[1],)
    assert grouping.truck_pair_histories[recovery.IDS].confirmations == 1
    assert grouping.last_truck_recovery_count == 0


class TestBoschLargeVehicleA0Recovery:
  IDS = (1_004_581, 1_004_624)

  @pytest.fixture(autouse=True)
  def _isolate_a0_recovery(self, monkeypatch):
    # The far member of these fixtures also lies inside the large-vehicle
    # footprint, which now associates it before A0 recovery is consulted (see
    # TestBoschLargeFootprintAssociation). Recovery still covers what the
    # footprint does not (e.g. a missing near member), so test it in isolation.
    monkeypatch.setattr(BoschCameraExtendedGrouping, '_large_footprint_associations',
                        staticmethod(lambda *_a, **_k: {}))

  @classmethod
  def objects(cls, ns, *, far_pid=None, far_d=26., far_y=-4.5, far_v=0.):
    return (physical(cls.IDS[0], 20., -4., 0., ns),
            physical(cls.IDS[1] if far_pid is None else far_pid, far_d, far_y, far_v, ns))

  @staticmethod
  def camera(*, camera_id=203, episode=186, camera_d=23., camera_y=4.25,
             camera_v=0., width=2.45, class_code=6, left=.205, right=.197):
    return BoschCameraObject(camera_id, episode, camera_d, camera_y, camera_v,
                             width, class_code, left, right)

  @classmethod
  def configure(cls, grouping, ns, mapping, *, camera_ns=None, cameras=None, **camera_kwargs):
    grouping._associate = lambda obj, *_: mapping.get(obj.physical_track_id, (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
    values = [cls.camera(**camera_kwargs)] if cameras is None else cameras
    grouping.camera.snapshot = lambda _: (values, len(values), 0, ns if camera_ns is None else camera_ns)

  @classmethod
  def assigned(cls, *, episode=186, class_code=6, far_pid=None):
    return {cls.IDS[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code),
            cls.IDS[1] if far_pid is None else far_pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code)}

  @classmethod
  def one_sided(cls, *, episode=186, class_code=6, missing=1, far_pid=None, missing_status=BOSCH_CAMERA_ASSOC_UNRESOLVED):
    ids = (cls.IDS[0], cls.IDS[1] if far_pid is None else far_pid)
    mapping = {ids[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code),
               ids[1]: (BOSCH_CAMERA_ASSOC_ASSIGNED, episode, class_code)}
    mapping[ids[missing]] = (missing_status, -1, -1)
    return mapping

  @classmethod
  def seed(cls, grouping):
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS):
      ns = 1_000_000_000 + i * 100_000_000
      cls.configure(grouping, ns, cls.assigned())
      grouping.update(ns, cls.objects(ns), 10.)
    return ns

  def test_frozen_a0_verdict_is_exact_and_near_miss_remains_unresolved(self):
    camera = self.camera()
    near, far = self.objects(1_000_000_000)
    assert BoschCameraExtendedGrouping._associate(near, [camera], 1) == (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 6)
    assert BoschCameraExtendedGrouping._associate(far, [camera], 1) == (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1)

  def test_unseeded_one_side_a0_never_recovers(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  def test_seeded_large_pair_recovers_one_side_transient_unresolved(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_associations[self.IDS[1]][0] == BOSCH_CAMERA_ASSOC_UNRESOLVED
    assert grouping.last_groups == (self.IDS,)
    assert grouping.last_truck_recovery_count == 1

  def test_both_unresolved_never_recovers(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, {})
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  def test_camera_id_change_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(), camera_id=204)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_episode_change_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(episode=187), episode=187)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_groups == ()

  def test_class_change_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(class_code=2), class_code=2)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_groups == ()

  def test_stale_camera_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(), camera_ns=ns - 160_000_001)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_ambiguous_missing_side_never_recovers(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(missing_status=BOSCH_CAMERA_ASSOC_AMBIGUOUS))
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_pair_g0_failure_resets_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns, far_d=32.1), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  @pytest.mark.parametrize(('field', 'kwargs'), (
    ('dd', {'far_d': 27.}), ('dy', {'far_y': -4.}), ('dv', {'far_v': .5}),
  ))
  def test_radar_dd_dy_dv_discontinuity_resets_recovery(self, field, kwargs):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns, **kwargs), 10.)
    assert grouping.truck_pair_histories == {}, field

  @pytest.mark.parametrize(('field', 'kwargs'), (
    ('d', {'camera_d': 24.}), ('y', {'camera_y': 4.5}),
    ('v', {'camera_v': 1.}), ('width', {'width': 2.60}),
  ))
  def test_camera_motion_or_width_jump_resets_recovery(self, field, kwargs):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided(), **kwargs)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {}, field

  def test_member_pid_change_cannot_inherit_seed(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    new_pid = self.IDS[1] + 1
    self.configure(grouping, ns, self.one_sided(far_pid=new_pid), cameras=[self.camera()])
    grouping.update(ns, self.objects(ns, far_pid=new_pid), 10.)
    assert grouping.truck_pair_histories == {}

  def test_missing_representative_identity_cannot_recover(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    grouping.representatives.clear()
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_recovery_hold_timeout_retires_pair_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping)
    for age in range(1, BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS + 1):
      ns += 100_000_000
      self.configure(grouping, ns, self.one_sided())
      grouping.update(ns, self.objects(ns), 10.)
      assert grouping.truck_pair_histories[self.IDS].recovery_age == age
    ns += 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0

  def test_different_vehicle_pair_without_seed_is_rejected(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(20):
      ns = 1_000_000_000 + i * 100_000_000
      mapping = {self.IDS[0]: (BOSCH_CAMERA_ASSOC_ASSIGNED, 186, 6),
                 self.IDS[1]: (BOSCH_CAMERA_ASSOC_ASSIGNED, 187, 6)}
      self.configure(grouping, ns, mapping)
      grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  def test_adjacent_or_cut_in_geometry_fails_open(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_PUBLICATION)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns, far_y=-2.5), 10.)
    # 기존 E2 coast는 허용하지만 새 strict recovery/M2 suppression은 열지 않는다.
    assert grouping.truck_pair_histories == {} and grouping.last_truck_recovery_count == 0
    assert grouping.mature_groups == ()

  def test_narrow_phantom_pair_cannot_form_recovery_seed(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(12):
      ns = 1_000_000_000 + i * 100_000_000
      self.configure(grouping, ns, self.assigned(), width=2.20)
      grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == () and grouping.truck_pair_histories == {}

  def test_class1_path_is_immediate_and_has_no_recovery_state(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = 1_000_000_000
    self.configure(grouping, ns, self.assigned(class_code=1), class_code=1)
    grouping.update(ns, self.objects(ns), 10.)
    assert grouping.last_groups == (self.IDS,) and grouping.truck_pair_histories == {}

  def test_existing_truck_seed_is_exact_without_recovery(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    for i in range(BOSCH_TRUCK_P2_CONFIRMATIONS):
      ns = 1_000_000_000 + i * 100_000_000
      self.configure(grouping, ns, self.assigned())
      grouping.update(ns, self.objects(ns), 10.)
      assert bool(grouping.last_groups) is (i == BOSCH_TRUCK_P2_CONFIRMATIONS - 1)
      assert grouping.last_truck_recovery_count == 0

  def test_recovery_state_is_bounded_and_cleans_up(self):
    grouping = BoschCameraExtendedGrouping(BOSCH_CAMERA_EXTENDED_ACTIVE)
    ns = self.seed(grouping) + 100_000_000
    self.configure(grouping, ns, self.one_sided())
    grouping.update(ns, self.objects(ns), 10.)
    assert len(grouping.truck_pair_histories) == 1
    ns += 100_000_000
    self.configure(grouping, ns, {})
    grouping.update(ns, (), 10.)
    assert grouping.truck_pair_histories == {}


class TestBoschCompanionDeferral:
  """Baseline geometry/hold behaviour after common ancestry is confirmed.

  Common-ancestry construction and fail-open cases live in test_radar.py. This
  class keeps the pre-existing geometry, OEM and hold boundaries isolated.
  """
  EPISODE = 7
  NEAR, FAR = 1_000_001, 1_000_002

  @staticmethod
  def camera(width_m=2.5, episode=EPISODE):
    return BoschCameraObject(obj_id=3, episode=episode, long_m=20., width_m=width_m, class_code=6)

  @classmethod
  def configure(cls, provider, mapping=None, width_m=2.5, episode=EPISODE):
    ext = provider.camera_extended
    mapping = mapping if mapping is not None else {
      pid: (BOSCH_CAMERA_ASSOC_ASSIGNED, cls.EPISODE, 6) for pid in (cls.NEAR, cls.FAR)}
    ext._associate = lambda obj, *_: mapping.get(obj.physical_track_id,
                                                 (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
    camera = cls.camera(width_m, episode)
    ext.camera.snapshot = lambda ns: ([camera], 1, 0, ns)

  @classmethod
  def objects(cls, ns, near_d=12.5, far_d=18.75, oem='near', y_rel=0.):
    near = physical(cls.NEAR, near_d, y_rel=y_rel, ns=ns)
    far = physical(cls.FAR, far_d, y_rel=y_rel, ns=ns)
    if oem == 'near':
      near = replace(near, oem_selected=True)
    elif oem == 'far':
      far = replace(far, oem_selected=True)
    return (near, far)

  @classmethod
  def scan(cls, provider, ns, objects=None, state=BOSCH_OEM_STATE_VALIDATED, **kwargs):
    objects = cls.objects(ns, **kwargs) if objects is None else objects
    provider.last_oem_state = state
    provider.camera_extended.update(ns, objects, 25.)
    return objects, provider.publication_view(objects)

  @classmethod
  def provider(cls, width_m=2.5):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_PUBLICATION)
    provider.tracker.group_manager.common_ancestry_evidence = lambda *_args: (True, True)
    cls.configure(provider, width_m=width_m)
    return provider

  def test_defers_the_farther_surface_and_keeps_the_oem_anchor(self):
    provider = self.provider()
    objects, view = self.scan(provider, 1_000_000_000)
    assert [obj.physical_track_id for obj in view] == [self.NEAR]
    assert provider.last_companion_deferred == (self.FAR,)
    assert provider.companion_deferred_points == 1

  def test_repeated_publication_view_for_one_scan_is_stable(self):
    provider = self.provider()
    objects, view = self.scan(provider, 1_000_000_000)
    assert [o.physical_track_id for o in provider.publication_view(objects)] == [self.NEAR]

  @pytest.mark.parametrize(('kwargs', 'reason'), (
    ({'oem': 'far'}, 'word1 on the farther surface'),
    ({'oem': 'none'}, 'no OEM anchor at all'),
    ({'near_d': 12.5, 'far_d': 15.0}, 'separation below the 3 m window'),
    ({'near_d': 12.5, 'far_d': 25.0}, 'separation beyond the 12 m window'),
    ({'y_rel': 3.0}, 'pair outside the lead corridor'),
  ))
  def test_publishes_both_when_the_evidence_is_incomplete(self, kwargs, reason):
    provider = self.provider()
    objects, view = self.scan(provider, 1_000_000_000, **kwargs)
    assert view is objects, reason

  def test_requires_oem_validation(self):
    provider = self.provider()
    for state in (BOSCH_OEM_STATE_NONE, BOSCH_OEM_STATE_TENTATIVE, BOSCH_OEM_STATE_SELECTED):
      objects, view = self.scan(provider, 1_000_000_000, state=state)
      assert view is objects

  def test_narrow_camera_object_caps_the_separation(self):
    # A 1.9 m wide object cannot own two returns 6 m apart; a wide one can.
    for width, deferred in ((1.9, False), (2.5, True)):
      provider = self.provider(width)
      _, view = self.scan(provider, 1_000_000_000)
      assert (len(view) == 1) is deferred, width

  def test_a_lost_anchor_republishes_within_the_hold(self):
    provider = self.provider()
    ns = 1_000_000_000
    _, view = self.scan(provider, ns)
    assert len(view) == 1
    # word1 moves away: the pair coasts on the hold, then reopens.
    ns += 100_000_000
    _, view = self.scan(provider, ns, oem='none')
    assert len(view) == 1
    ns += 300_000_000
    provider.camera_extended.histories = {}
    _, view = self.scan(provider, ns, oem='none')
    assert len(view) == 2

  def test_a_stale_camera_reopens_publication(self):
    provider = self.provider()
    _, view = self.scan(provider, 1_000_000_000)
    assert len(view) == 1
    ext = provider.camera_extended
    camera = self.camera()
    ext.camera.snapshot = lambda ns: ([camera], 1, 0, ns - 300_000_000)
    objects, view = self.scan(provider, 1_100_000_000)
    assert view is objects
    assert provider.companion_pairs == {}

  def test_never_removes_the_last_contact_of_a_vehicle(self):
    provider = self.provider()
    ns = 1_000_000_000
    objects, view = self.scan(provider, ns)
    assert len(view) == 1
    # the nearer surface is gone from the published set: the farther one stays
    ns += 100_000_000
    objects = self.objects(ns)
    provider.last_oem_state = BOSCH_OEM_STATE_VALIDATED
    provider.camera_extended.update(ns, objects, 25.)
    only_far = (objects[1],)
    assert provider.publication_view(only_far) is only_far

  def test_off_mode_is_exact_identity(self):
    provider = self.provider()
    import opendbc.car.hyundai.radar_bosch as module
    original = module.BOSCH_COMPANION_DEFER_MODE
    module.BOSCH_COMPANION_DEFER_MODE = module.BOSCH_COMPANION_DEFER_OFF
    try:
      objects, view = self.scan(provider, 1_000_000_000)
      assert view is objects
    finally:
      module.BOSCH_COMPANION_DEFER_MODE = original


class TestBoschOemNearerPublication:
  """Publication-stage range correction.

  A physical object publishes its representative member's range, and the
  representative is held by continuity, so a farther member can keep the role
  for the life of the object. When the OEM's own word1 target is a member of
  that object and is nearer, the published coordinates move to it. The
  correction is the last step of publication_view and its result goes straight
  to RadarData, so nothing inside the provider can be affected by it.
  """
  SCAN_NS = 1_000_000_000

  @staticmethod
  def member(raw_track_id, slot, d_rel, y_rel=0., v_rel=0., ns=SCAN_NS):
    detection = BoschRawDetection(ns, slot, float(d_rel), float(y_rel), float(v_rel), 1)
    return BoschRawTrack(raw_track_id, detection, 4)

  @classmethod
  def grouped(cls, pid, members, *, representative, ns=SCAN_NS, oem_selected=True):
    rep = next(m for m in members if m.raw_track_id == representative)
    return BoschPhysicalObject(pid, ns, tuple(members), rep.raw_track_id, rep.d_rel, rep.y_rel,
                               rep.v_rel, oem_selected, False, 40, 'temporal_complete_link')

  @classmethod
  def provider(cls, oem_slot=4, scan_ns=SCAN_NS):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_OFF)
    provider.last_oem_slot = oem_slot
    provider.last_scan_timestamp_ns = scan_ns
    return provider

  @classmethod
  def standstill_object(cls, pid=1_000_001, ns=SCAN_NS):
    # the Route259 shape: OEM word1 on slot 4 at 3.0 m, representative at 6.0 m
    return cls.grouped(pid, (cls.member(877, 4, 3.0, .25, ns=ns),
                             cls.member(1024, 17, 4.75, .3, ns=ns),
                             cls.member(943, 2, 6.0, .31, ns=ns)), representative=943, ns=ns)

  def test_publishes_the_oem_member_when_it_is_nearer(self):
    provider = self.provider()
    obj = self.standstill_object()
    view = provider.publication_view((obj,), self.SCAN_NS)
    assert len(view) == 1
    # the published surface moves; the continuity anchor stays where it was
    assert bosch_published_surface(view[0]) == (877, 3.0, .25, 0.)
    assert (view[0].d_rel, view[0].representative_raw_track_id) == (obj.d_rel, obj.representative_raw_track_id)
    assert view[0].physical_track_id == obj.physical_track_id
    assert provider.oem_nearer_corrections == 1
    assert provider.last_oem_nearer == (obj.physical_track_id,)

  def test_never_moves_a_contact_farther(self):
    provider = self.provider(oem_slot=2)          # word1 now on the far member
    obj = self.standstill_object()
    view = provider.publication_view((obj,), self.SCAN_NS)
    assert bosch_published_surface(view[0])[1] == obj.d_rel
    assert provider.oem_nearer_corrections == 0

  def test_leaves_the_physical_object_and_its_members_untouched(self):
    provider = self.provider()
    obj = self.standstill_object()
    view = provider.publication_view((obj,), self.SCAN_NS)
    assert obj.d_rel == 6.0                        # the tracker's object is unchanged
    assert view[0].members is obj.members          # membership is never rewritten
    assert {m.raw_track_id for m in view[0].members} == {877, 1024, 943}

  @pytest.mark.parametrize(('kwargs', 'reason'), (
    ({'oem_slot': None}, 'no OEM target this scan'),
    ({'oem_slot': 9}, 'OEM target is not a member of this object'),
  ))
  def test_untouched_without_an_owned_oem_member(self, kwargs, reason):
    provider = self.provider(**kwargs)
    objects = (self.standstill_object(),)
    assert provider.publication_view(objects, self.SCAN_NS) is objects, reason

  def test_single_member_object_is_untouched(self):
    provider = self.provider()
    objects = (self.grouped(1_000_002, (self.member(877, 4, 3.0),), representative=877),)
    assert provider.publication_view(objects, self.SCAN_NS) is objects

  def test_stale_object_is_untouched(self):
    provider = self.provider(scan_ns=self.SCAN_NS + 100_000_000)
    objects = (self.standstill_object(),)
    assert provider.publication_view(objects, self.SCAN_NS) is objects

  def test_correction_is_bounded_by_the_objects_own_members(self):
    provider = self.provider()
    obj = self.standstill_object()
    view = provider.publication_view((obj,), self.SCAN_NS)
    published = bosch_published_surface(view[0])[1]
    assert published >= min(m.d_rel for m in obj.members)
    assert published <= obj.d_rel

  def test_off_mode_is_exact_identity(self):
    import opendbc.car.hyundai.radar_bosch as module
    provider = self.provider()
    original = module.BOSCH_OEM_NEARER_PUBLICATION_MODE
    module.BOSCH_OEM_NEARER_PUBLICATION_MODE = module.BOSCH_OEM_NEARER_PUBLICATION_OFF
    try:
      objects = (self.standstill_object(),)
      assert provider.publication_view(objects, self.SCAN_NS) is objects
    finally:
      module.BOSCH_OEM_NEARER_PUBLICATION_MODE = original

  def test_two_vehicles_are_never_mixed(self):
    # A separate object at the OEM slot's range must not pull the other one in:
    # the correction only ever reads members of the object it is correcting.
    provider = self.provider()
    lead = self.standstill_object(1_000_001)
    other = self.grouped(1_000_002, (self.member(500, 11, 12.0), self.member(501, 12, 14.0)),
                         representative=501, oem_selected=False)
    view = provider.publication_view((lead, other), self.SCAN_NS)
    corrected = {obj.physical_track_id: obj for obj in view}
    assert bosch_published_surface(corrected[1_000_001])[1] == 3.0
    # untouched: slot 4 is not its member, so it still publishes its anchor
    assert bosch_published_surface(corrected[1_000_002]) == (501, 14.0, 0., 0.)


class TestBoschContinuityAnchorPublicationSplit:
  """A physical object carries two roles, and they are separate fields.

  CONTINUITY ANCHOR -- `d_rel`/`y_rel`/`v_rel` and `representative_raw_track_id`.
  Internal state: the next scan projects it forward, the representative
  continuity term scores against that projection, the physical-ID assignment
  score rewards a cluster that still holds it, and the qualifier, P91, the OEM
  gate and the camera grouping all keep state derived from it.

  PUBLISHED SURFACE -- `published_surface`, the one member whose geometry
  reaches RadarData. Only the last step of `publication_view` may set one, and
  nothing inside the provider reads it, so moving it cannot change an identity.
  """
  SCAN_NS = 1_000_000_000
  STEP_NS = 100_000_000

  @staticmethod
  def track(raw_track_id, slot, d_rel, y_rel=0., v_rel=0., ns=SCAN_NS):
    return BoschRawTrack(raw_track_id, BoschRawDetection(ns, slot, float(d_rel), float(y_rel),
                                                         float(v_rel), 1), 4)

  @classmethod
  def grouped(cls, pid, members, representative, ns=SCAN_NS, surface=None):
    rep = next(m for m in members if m.raw_track_id == representative)
    return BoschPhysicalObject(pid, ns, tuple(members), rep.raw_track_id, rep.d_rel, rep.y_rel,
                               rep.v_rel, True, False, 40, 'temporal_complete_link', surface)

  @classmethod
  def suv(cls, ns=SCAN_NS):
    # Route259 shape: OEM word1 on slot 4 at 3.0 m, continuity anchor at 6.0 m
    return (cls.track(877, 4, 3.0, .25, -.5, ns),
            cls.track(1024, 17, 4.75, .3, -.5, ns),
            cls.track(943, 2, 6.0, .31, -.5, ns))

  def test_without_a_surface_the_anchor_is_published(self):
    obj = self.grouped(1_000_001, self.suv(), 943)
    assert obj.published_surface is None
    assert bosch_published_surface(obj) == (943, 6.0, .31, -.5)
    point = structs.RadarData.RadarPoint()
    bosch_fill_point(point, obj, 10.)
    assert (point.dRel, point.vRel) == (6.0, -.5)
    assert point.yRel == pytest.approx(.31)
    assert point.vLead == pytest.approx(9.5)

  def test_a_surface_moves_only_the_published_coordinates(self):
    anchor = self.grouped(1_000_001, self.suv(), 943)
    moved = replace(anchor, published_surface=BoschPublishedSurface(877, 3.0, .25, -.5))
    assert (moved.d_rel, moved.y_rel, moved.v_rel) == (anchor.d_rel, anchor.y_rel, anchor.v_rel)
    assert moved.representative_raw_track_id == anchor.representative_raw_track_id
    assert moved.members is anchor.members
    point = structs.RadarData.RadarPoint()
    bosch_fill_point(point, moved, 10.)
    assert point.dRel == 3.0
    assert point.yRel == pytest.approx(.25)

  def test_the_surface_is_one_whole_member(self):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_OFF)
    provider.last_oem_slot, provider.last_scan_timestamp_ns = 4, self.SCAN_NS
    members = self.suv()
    view = provider.publication_view((self.grouped(1_000_001, members, 943),), self.SCAN_NS)
    raw, d_rel, y_rel, v_rel = bosch_published_surface(view[0])
    member = next(m for m in members if m.raw_track_id == raw)
    assert (d_rel, y_rel, v_rel) == (member.d_rel, member.y_rel, member.v_rel)

  def test_the_published_range_ages_on_the_surface_member(self):
    # the two members were measured 40 ms apart; the extrapolation must use the
    # age of the member actually being published, not the anchor's
    members = (self.track(877, 4, 3.0, .25, -2., self.SCAN_NS),
               self.track(943, 2, 6.0, .31, -2., self.SCAN_NS - 40_000_000))
    obj = self.grouped(1_000_001, members, 943)
    moved = replace(obj, published_surface=BoschPublishedSurface(877, 3.0, .25, -2.))
    now_ns = self.SCAN_NS + 20_000_000
    for candidate, expected in ((obj, 6.0 - 2. * .06), (moved, 3.0 - 2. * .02)):
      data = structs.RadarData.new_message()
      bosch_append_points(data, (candidate,), 10., now_ns)
      assert data.points[0].dRel == pytest.approx(expected, abs=1e-4)

  def test_the_tracker_never_emits_a_surface(self):
    objects = BoschPhysicalTracker().update(self.SCAN_NS, [m.detection for m in self.suv()])
    assert objects and all(obj.published_surface is None for obj in objects)

  def test_publication_view_leaves_the_tracker_state_untouched(self):
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_OFF,
                                  qualification=False)
    manager = provider.tracker.group_manager
    objects = provider.tracker.update(self.SCAN_NS, [m.detection for m in self.suv()], oem_slot=4)
    provider.last_oem_slot, provider.last_scan_timestamp_ns = 4, self.SCAN_NS
    before = {pid: state.observation for pid, state in manager.states.items()}
    provider.publication_view(objects, self.SCAN_NS)
    assert {pid: state.observation for pid, state in manager.states.items()} == before
    assert all(state.observation.published_surface is None for state in manager.states.values())

  def test_publishing_a_nearer_surface_does_not_move_the_anchor_next_scan(self):
    """The anchor the next scan projects is the one the tracker chose, always."""
    provider = BoschRadarProvider(1, camera_extended_mode=BOSCH_CAMERA_EXTENDED_OFF,
                                  qualification=False)
    published, anchors = [], []
    for step in range(4):
      ns = self.SCAN_NS + step * self.STEP_NS
      objects = provider.tracker.update(ns, [m.detection for m in self.suv(ns)], oem_slot=4)
      provider.last_oem_slot, provider.last_scan_timestamp_ns = 4, ns
      view = provider.publication_view(objects, ns)
      published.append([bosch_published_surface(obj)[:2] for obj in view])
      anchors.append([(obj.representative_raw_track_id, obj.d_rel) for obj in objects])
    assert anchors[-1] == anchors[-2]
    assert published[-1] == published[-2]

  def test_a_surface_cannot_reach_the_physical_assignment(self):
    """Same raw input, with and without a surface attached to the published
    tuple: the manager state, IDs and statistics are identical."""
    def run(attach):
      manager = BoschObjectGroupManager()
      signature = []
      for step in range(5):
        ns = self.SCAN_NS + step * self.STEP_NS
        objects = manager.update(ns, self.suv(ns), oem_slot=4)
        if attach:
          # exactly what publication does: a copy, never the stored observation
          [replace(obj, published_surface=BoschPublishedSurface(877, 3.0, .25, -.5))
           for obj in objects]
        signature.append([(obj.physical_track_id, obj.representative_raw_track_id, obj.d_rel,
                           obj.age_scans, obj.member_slots) for obj in objects])
      return manager, signature

    plain, plain_signature = run(False)
    surfaced, surfaced_signature = run(True)
    assert plain_signature == surfaced_signature
    assert dict(plain.stats) == dict(surfaced.stats)
    assert plain.next_id == surfaced.next_id
    assert sorted(plain.states) == sorted(surfaced.states)

  def test_anchor_survives_a_nearer_member_joining(self):
    manager = BoschObjectGroupManager()
    anchors = []
    for step in range(6):
      ns = self.SCAN_NS + step * self.STEP_NS
      members = self.suv(ns)[1:] if step < 3 else self.suv(ns)
      objects = manager.update(ns, members, oem_slot=4)
      anchors.append([(obj.physical_track_id, obj.representative_raw_track_id) for obj in objects])
    # a nearer member joining the cluster does not take the anchor away from the
    # member the continuity term already holds
    assert all(raw != 877 for _pid, raw in anchors[-1])

  def test_stale_object_state_is_released(self):
    manager = BoschObjectGroupManager()
    objects = manager.update(self.SCAN_NS, self.suv(), oem_slot=4)
    pid = objects[0].physical_track_id
    assert pid in manager.states
    late = self.SCAN_NS + 10 * self.STEP_NS
    manager.update(late, (self.track(4242, 30, 80.0, 0., 0., late),))
    assert pid not in manager.states


class TestBoschCameraScaleCorrection:
  """0.0625 -> 0.05 스케일 교정과 그에 맞춘 association 창의 회귀 고정."""

  @staticmethod
  def word(long_raw, lat_raw, vrel_raw, obj_id=7, counter=0):
    return (obj_id | (long_raw << 8) | (signed(lat_raw, 12) << 20) |
            (signed(vrel_raw, 12) << 40) | (counter << 52))

  def test_longitudinal_raw_count_is_read_at_five_centimetres(self):
    cache = BoschCameraCycleCache()
    cache.ingest(1_000_000_000, BOSCH_CAMERA_HEADER, (1).to_bytes(8, 'little'))
    cache.ingest(1_000_000_000, 0x739, self.word(400, 0, 0).to_bytes(8, 'little'))
    cache.ingest(1_000_000_000, 0x73a, (round(1.70 / .05) | (1 << 48)).to_bytes(8, 'little'))
    cache.ingest(1_000_000_000, 0x73b, (0).to_bytes(8, 'little'))
    obj = cache.snapshot(1_001_000_000)[0][0]
    # 400 counts: 0.05 이면 20.00 m, 0.0625 이면 25.00 m
    assert obj.long_m == pytest.approx(20.0)
    assert BOSCH_CAMERA_RANGE_LSB == 0.05

  def test_lateral_is_signed_at_five_centimetres_with_inverted_sign(self):
    cache = BoschCameraCycleCache()
    cache.ingest(1_000_000_000, BOSCH_CAMERA_HEADER, (1).to_bytes(8, 'little'))
    cache.ingest(1_000_000_000, 0x739, self.word(400, -70, 0).to_bytes(8, 'little'))
    cache.ingest(1_000_000_000, 0x73a, (round(1.70 / .05) | (1 << 48)).to_bytes(8, 'little'))
    cache.ingest(1_000_000_000, 0x73b, (0).to_bytes(8, 'little'))
    obj = cache.snapshot(1_001_000_000)[0][0]
    assert obj.lat_m == pytest.approx(-3.50)
    # 카메라 부호는 Bosch/SCC와 반대다: d_lat 은 더하기로 만든다
    target = physical(1_000_001, 20., y_rel=3.50)
    cam = BoschCameraObject(7, 11, 20., obj.lat_m, 0., 1.7, 1, .2, -.2)
    assert BoschCameraExtendedGrouping._associate(target, [cam], 1)[0] == BOSCH_CAMERA_ASSOC_ASSIGNED

  def test_relative_speed_is_signed_at_five_centimetres(self):
    cache = BoschCameraCycleCache()
    cache.ingest(1_000_000_000, BOSCH_CAMERA_HEADER, (1).to_bytes(8, 'little'))
    cache.ingest(1_000_000_000, 0x739, self.word(400, 0, -50).to_bytes(8, 'little'))
    cache.ingest(1_000_000_000, 0x73a, (round(1.70 / .05) | (1 << 48)).to_bytes(8, 'little'))
    cache.ingest(1_000_000_000, 0x73b, (0).to_bytes(8, 'little'))
    obj = cache.snapshot(1_001_000_000)[0][0]
    assert obj.vrel_mps == pytest.approx(-2.50)

  @pytest.mark.parametrize('d_rel', (4.0, 25.0, 90.0))
  def test_same_object_at_every_range_band(self, d_rel):
    # 실측 중앙값은 +0.15 m 근처이고 p05/p95 는 창 안에 있다
    for residual in (-0.9, 0.0, +0.9):
      cam = BoschCameraObject(7, 11, d_rel + residual, 0., 0., 1.7, 1, .1, -.1)
      verdict = BoschCameraExtendedGrouping._associate(physical(1_000_001, d_rel), [cam], 1)
      assert verdict[0] == BOSCH_CAMERA_ASSOC_ASSIGNED

  @pytest.mark.parametrize('side', (+1, -1))
  def test_adjacent_lane_vehicle_is_not_associated(self, side):
    # 옆차선 차량: 카메라 lat 은 Bosch 부호의 반대이므로 ego-lane 레이더 객체와의
    # d_lat 은 한 차선 폭이 된다
    cam = BoschCameraObject(7, 11, 20., -side * 3.5, 0., 1.7, 2, .2, -.2)
    ego_lane = physical(1_000_001, 20., y_rel=0.)
    assert BoschCameraExtendedGrouping._associate(ego_lane, [cam], 1)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED
    same_lane = physical(1_000_002, 20., y_rel=side * 3.5)
    assert BoschCameraExtendedGrouping._associate(same_lane, [cam], 1)[0] == BOSCH_CAMERA_ASSOC_ASSIGNED

  def test_oncoming_object_is_separated_by_geometry_not_by_a_new_field(self):
    # 대향 차량은 옆차선 기하로 분리된다. opposite bit(B bit29)는 이 커밋에서
    # production gate로 쓰지 않는다: 같은 기하면 여전히 연관된다.
    oncoming = BoschCameraObject(7, 11, 30., -3.6, -25., 1.7, 2, .2, -.2)
    ours = physical(1_000_001, 30., y_rel=0., v_rel=-1.)
    assert BoschCameraExtendedGrouping._associate(ours, [oncoming], 1)[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED

  def test_two_vehicles_in_one_bearing_are_ambiguous_not_arbitrary(self):
    near = BoschCameraObject(7, 11, 20., 0., 0., 1.7, 2, .1, -.1)
    far = BoschCameraObject(8, 12, 21., 0., 0., 1.7, 2, .1, -.1)
    verdict = BoschCameraExtendedGrouping._associate(physical(1_000_001, 20.5), [near, far], 2)
    assert verdict[0] == BOSCH_CAMERA_ASSOC_AMBIGUOUS

  def test_cut_in_keeps_its_association_across_the_lane_transition(self):
    # 옆차선에서 자기 차선으로 들어오는 차량. 레이더 y 와 카메라 lat 이 함께
    # 움직이는 한 연관은 유지된다.
    for y_rel in (2.4, 1.8, 1.2, 0.6, 0.0):
      cam = BoschCameraObject(7, 11, 24.6, -y_rel, -1.5, 1.7, 2, .15, -.15)
      target = physical(1_000_001, 25., y_rel=y_rel, v_rel=-1.5)
      assert BoschCameraExtendedGrouping._associate(target, [cam], 1) == \
          (BOSCH_CAMERA_ASSOC_ASSIGNED, 11, 2)

  def test_frozen_policies_this_change_does_not_touch(self):
    import opendbc.car.hyundai.radar_interface as module
    # bearing 창 / 비용 가중치 / 모호성 마진 / 폭 기준 / 각도 LSB
    assert module.BOSCH_CAMERA_WIDTH_REF_M == 1.70
    assert module.BOSCH_CAMERA_ANGLE_LSB == 1.0 / 4496.3
    # 이번 변경의 상수들
    assert (module.BOSCH_CAMERA_LONG_BASE_M, module.BOSCH_CAMERA_LONG_RANGE_K) == (1.75, 0.12)
    assert (module.BOSCH_CAMERA_LONG_POS_WIDTH_K, module.BOSCH_CAMERA_LONG_NEG_WIDTH_K) == (4.0, 16.0)
    assert (module.BOSCH_CAMERA_LONG_MAX_M, module.BOSCH_CAMERA_LAT_MAX_M) == (20.0, 1.75)
    # 기존 ACTIVE 정책 상수는 그대로다
    assert module.BOSCH_P91_MODE == module.BOSCH_P91_ACTIVE
    assert module.BOSCH_OEM_GATE_MODE == module.BOSCH_OEM_GATE_ACTIVE
    assert module.BOSCH_COMPANION_DEFER_MODE == module.BOSCH_COMPANION_DEFER_ACTIVE
    assert module.BOSCH_CAMERA_EXTENDED_MODE == module.BOSCH_CAMERA_EXTENDED_PUBLICATION
    # Confirmations 5 -> 3 by the 2026-10-08 truck-split stage 5 (frozen criteria in
    # docs/bosch_radar.md section 3.2); the width gate is unchanged.
    assert (module.BOSCH_TRUCK_P2_WIDTH_MIN_M, module.BOSCH_TRUCK_P2_CONFIRMATIONS) == (2.25, 3)

  def test_new_constants_are_read_only_by_the_bosch_camera_path(self):
    from pathlib import Path
    import re
    interface_path = Path(module_path())
    interface_source = interface_path.read_text(encoding='utf-8')
    bosch_source = interface_path.with_name('radar_bosch.py').read_text(encoding='utf-8')
    for name in ('BOSCH_CAMERA_RANGE_LSB', 'BOSCH_CAMERA_LONG_BASE_M',
                 'BOSCH_CAMERA_LONG_RANGE_K', 'BOSCH_CAMERA_LONG_POS_WIDTH_K',
                 'BOSCH_CAMERA_LONG_NEG_WIDTH_K', 'BOSCH_CAMERA_LONG_MAX_M',
                 'BOSCH_CAMERA_LAT_MAX_M'):
      uses = [m.start() for m in re.finditer(name, bosch_source)]
      assert uses, name
      # generic RadarInterface 경로에서는 이 상수들을 사용하지 않는다
      generic = interface_source.index('class RadarInterface(')
      assert not re.search(name, interface_source[generic:]), name


def module_path():
  import opendbc.car.hyundai.radar_interface as module
  return module.__file__


# First-observation publication with current camera support and fail-open inputs.
_UNSUPPORTED_BIRTH_NS = 1_000_000_000
_UNSUPPORTED_BIRTH_PATH = ((0., 0.), (160., 0.))


def _unsupported_birth_fixture():
  p = BoschRadarProvider(0)
  raw = BoschRawTrack(7, BoschRawDetection(_UNSUPPORTED_BIRTH_NS, 1, 50., 4., -5., 0), 1)
  obj = BoschPhysicalObject(1000001, _UNSUPPORTED_BIRTH_NS, (raw,), 7, 50., 4., -5., False, False, 1, 'single_return')
  p.last_scan_timestamp_ns = _UNSUPPORTED_BIRTH_NS
  p._last_now_ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  p._unsupported_birth_context = (p._last_now_ns, 20., 0., _UNSUPPORTED_BIRTH_PATH, _UNSUPPORTED_BIRTH_NS)
  p.tracker.raw_manager.last_identity_evidence = {7: (_UNSUPPORTED_BIRTH_NS, 0, False)}
  p.camera_extended.last_ns = _UNSUPPORTED_BIRTH_NS
  p.camera_extended.camera = NS(snapshot=lambda ns: ((), 0, 0, _UNSUPPORTED_BIRTH_NS))
  p.publication_aliases.physical_to_alias = {1000001: 32}
  return p, obj


@pytest.fixture
def birth_and_corner_gates(monkeypatch):
  """Exercise the retained birth/corner mechanisms at fixture range.

  Production keeps the birth guard off and limits the corner gate to far points."""
  module = sys.modules[BoschRadarProvider.__module__]
  monkeypatch.setattr(module, 'BOSCH_UNSUPPORTED_BIRTH_PUBLICATION', True)
  monkeypatch.setattr(module, 'BOSCH_SURFACE_CORNER_GATE', True)
  monkeypatch.setattr(module, 'BOSCH_SURFACE_CORNER_MIN_RANGE_M', 0.)


def test_default_publication_keeps_unsupported_first_singleton(monkeypatch):
  p, obj = _unsupported_birth_fixture()
  monkeypatch.setattr(p.unsupported_birth, 'verdict', lambda *args, **kwargs: pytest.fail('Birth is off; no corner edge'))
  objects = (obj,)
  assert p.publication_view(objects, _UNSUPPORTED_BIRTH_NS + 50_000_000) == objects
  assert not p.unsupported_birth.last_decisions


def _unsupported_birth_view(p, objects, ns=_UNSUPPORTED_BIRTH_NS + 50_000_000):
  return p.unsupported_birth.publication_view(p, objects, ns)


@pytest.mark.usefixtures('birth_and_corner_gates')
def test_actual_provider_final_publication_and_native_array_exclude_first_scan():
  p, obj = _unsupported_birth_fixture()
  assert p.publication_view((obj,), _UNSUPPORTED_BIRTH_NS + 50_000_000) == ()
  msg = structs.RadarData.new_message()
  bosch_append_points(msg, p.publication_view((obj,), _UNSUPPORTED_BIRTH_NS + 90_000_000), 20., _UNSUPPORTED_BIRTH_NS + 90_000_000,
                      alias=p.publication_aliases.physical_to_alias)
  assert not msg.points
  assert obj.members[0].raw_track_id == 7 and obj.age_scans == 1
  assert p.publication_aliases.physical_to_alias == {1000001: 32}


def test_normal_first_observation_delay_is_an_explicit_policy_limit():
  p, obj = _unsupported_birth_fixture()
  assert not _unsupported_birth_view(p, (obj,))  # A normal off-path actor can also be deferred.
  grown = replace(obj, age_scans=2)
  assert _unsupported_birth_view(p, (grown,)) == (grown,)


@pytest.mark.parametrize('support', ['vision', 'oem', 'word0', 'scan_camera', 'current_camera', 'ambiguous_camera'])
def test_independent_current_support_preserves_actor(support):
  p, obj = _unsupported_birth_fixture()
  if support == 'vision':obj = replace(obj, vision_supported=True)
  if support == 'oem':obj = replace(obj, oem_selected=True)
  if support == 'word0':p._debug_processed_pids = {obj.physical_track_id}
  if support == 'scan_camera':p.camera_extended.last_associations = {obj.physical_track_id: (1, 0, 0)}
  if support in ('current_camera', 'ambiguous_camera'):
    p.camera_extended._associate = lambda *args: (1 if support == 'current_camera' else 2, 0, 0)
  assert _unsupported_birth_view(p, (obj,)) == (obj,)


def test_later_camera_support_only_releases_after_its_availability():
  p, obj = _unsupported_birth_fixture()
  snapshot = p.camera_extended.camera.snapshot
  p.camera_extended.camera.snapshot = lambda ns: snapshot(ns) if ns < _UNSUPPORTED_BIRTH_NS + 70_000_000 else ((object(),), 1, 0, _UNSUPPORTED_BIRTH_NS + 70_000_000)
  p.camera_extended._associate = lambda obj, candidates, count: (1, 0, 0) if count else (0, -1, -1)
  assert _unsupported_birth_view(p, (obj,), _UNSUPPORTED_BIRTH_NS + 50_000_000) == ()
  assert _unsupported_birth_view(p, (obj,), _UNSUPPORTED_BIRTH_NS + 90_000_000) == (obj,)


@pytest.mark.parametrize('change', ['stationary', 'corridor', 'path_extent', 'no_pose', 'stale_ego', 'future_ego',
                                   'no_path', 'stale_path', 'future_path', 'missing_camera', 'stale_camera',
                                   'ambiguous_raw', 'multiple_raw_candidates', 'missing_raw', 'future_raw', 'can_error', 'disabled'])
def test_uncertain_input_and_protected_geometry_fail_open(change):
  p, obj = _unsupported_birth_fixture();context=list(p._unsupported_birth_context)
  if change == 'stationary':obj=replace(obj, v_rel=-20.)
  if change == 'corridor':obj=replace(obj, y_rel=1.8)
  if change == 'path_extent':obj=replace(obj, d_rel=161.)
  if change == 'no_pose':context[2]=None
  if change == 'stale_ego':context[0]=_UNSUPPORTED_BIRTH_NS-200_000_001
  if change == 'future_ego':context[0]=_UNSUPPORTED_BIRTH_NS+50_000_001
  if change == 'no_path':context[3]=()
  if change == 'stale_path':context[4]=_UNSUPPORTED_BIRTH_NS-200_000_001
  if change == 'future_path':context[4]=_UNSUPPORTED_BIRTH_NS+50_000_001
  if change == 'missing_camera':p.camera_extended.camera=None
  if change == 'stale_camera':p.camera_extended.camera.snapshot=lambda ns: ((), 0, 0, _UNSUPPORTED_BIRTH_NS-200_000_001)
  if change == 'ambiguous_raw':p.tracker.raw_manager.last_identity_evidence={7:(_UNSUPPORTED_BIRTH_NS,1,True)}
  if change == 'multiple_raw_candidates':p.tracker.raw_manager.last_identity_evidence={7:(_UNSUPPORTED_BIRTH_NS,2,False)}
  if change == 'missing_raw':p.tracker.raw_manager.last_identity_evidence={}
  if change == 'future_raw':p.tracker.raw_manager.last_identity_evidence={7:(_UNSUPPORTED_BIRTH_NS+1,0,False)}
  if change == 'can_error':p.can_error=True
  if change == 'disabled':p.unsupported_birth.enabled=False
  p._unsupported_birth_context=tuple(context)
  assert _unsupported_birth_view(p,(obj,)) == (obj,)


def test_minimal_raw_identity_evidence_matches_optional_trace_and_expires():
  plain=BoschRawTrackManager();traced=BoschRawTrackManager();traced.trace_decisions=True
  for index in range(4):
    ns=_UNSUPPORTED_BIRTH_NS+index*100_000_000
    raw=(BoschRawDetection(ns,0,50.,0.,-5.,0),BoschRawDetection(ns,1,50.,0.,-5.,0))
    assert plain.update(ns,raw)==traced.update(ns,raw)
    assert not plain.last_decisions
    assert plain.last_identity_evidence=={d.raw_track_id:(ns,d.candidate_count,d.identity_ambiguous) for d in traced.last_decisions}
  plain.update(_UNSUPPORTED_BIRTH_NS+500_000_000,())
  assert not plain.last_identity_evidence


def test_geometry_uses_nearest_normal_and_model_y_inversion():
  path=((0.,0.),(100.,-10.))
  assert birth_d_path(path,50.,5.) == pytest.approx(0.)
  assert birth_d_path(path,50.,9.) == pytest.approx(4./(1.01**.5))


def test_off_switch_retains_identity_alias_and_nan_fields():
  p,obj=_unsupported_birth_fixture();p.unsupported_birth.enabled=False
  objects=p.publication_view((obj,),_UNSUPPORTED_BIRTH_NS+50_000_000)
  assert objects==(obj,) and objects[0] is obj
  msg=structs.RadarData.new_message();bosch_append_points(msg,objects,20.,_UNSUPPORTED_BIRTH_NS+50_000_000,alias=p.publication_aliases.physical_to_alias)
  assert msg.points[0].trackId==32 and msg.points[0].dRel==49.75
  import math
  assert math.isnan(msg.points[0].aLead) and math.isnan(msg.points[0].yvRel)


@pytest.mark.usefixtures('birth_and_corner_gates')
def test_real_radar_interface_control_input_filters_bosch_and_keeps_scc(monkeypatch):
  from opendbc.car.hyundai.radar_interface import RadarInterface
  from opendbc.car.interfaces import RadarInterfaceBase
  def legacy_result(*args):
    ret=structs.RadarData.new_message();point=ret.init('points',1)[0]
    point.trackId=0;point.dRel=10.;point.vRel=-1.;point.measured=True;point.radarSource='scc'
    return ret
  monkeypatch.setattr(RadarInterfaceBase,'update_carrot',legacy_result)
  interface=RadarInterface.__new__(RadarInterface)
  p,obj=_unsupported_birth_fixture();p.publication_aliases.physical_to_alias={}
  p.tracker.group_manager.states={obj.physical_track_id:object()}
  interface.bosch=p;interface._bosch_objects=(obj,);interface._bosch_now_ns=_UNSUPPORTED_BIRTH_NS+50_000_000
  interface.v_ego=20.
  filtered=interface.update_carrot(20.,0.,1.05,[])
  assert len(filtered.points)==1 and filtered.points[0].radarSource=='scc'
  p.unsupported_birth.enabled=False
  unfiltered=interface.update_carrot(20.,0.,1.05,[])
  assert len(unfiltered.points)==2
  assert unfiltered.points[0].to_dict()==filtered.points[0].to_dict()
  assert unfiltered.points[1].trackId==32 and unfiltered.points[1].radarSource=='frontRadar'


def test_unsupported_birth_skips_verdict_for_existing_objects(monkeypatch):
  p, obj = _unsupported_birth_fixture()
  established = replace(obj, age_scans=2)
  monkeypatch.setattr(p.unsupported_birth, 'verdict', lambda *args, **kwargs: pytest.fail('No birth verdict needed'))
  objects = (established,)
  assert _unsupported_birth_view(p, objects) is objects


def test_unsupported_birth_reuses_snapshot_only_within_publication():
  p, obj = _unsupported_birth_fixture()
  other = replace(obj, physical_track_id=1000002)
  calls = []
  p.camera_extended.camera.snapshot = lambda ns: calls.append(ns) or ((), 0, 0, _UNSUPPORTED_BIRTH_NS)
  assert not _unsupported_birth_view(p, (obj, other))
  assert len(calls) == 1
  assert not _unsupported_birth_view(p, (obj, other))
  assert len(calls) == 2


def test_unsupported_birth_same_time_changed_camera_is_read_again():
  p, obj = _unsupported_birth_fixture()
  assert not _unsupported_birth_view(p, (obj,))
  p.camera_extended.camera.snapshot = lambda ns: ((object(),), 1, 0, _UNSUPPORTED_BIRTH_NS)
  p.camera_extended._associate = lambda *args: (1, 0, 0)
  assert _unsupported_birth_view(p, (obj,)) == (obj,)
  assert not p.unsupported_birth.last_decisions


def test_unsupported_birth_same_time_changed_path_is_read_again():
  p, obj = _unsupported_birth_fixture()
  assert not _unsupported_birth_view(p, (obj,))
  context = list(p._unsupported_birth_context)
  context[3] = ((0., -4.), (160., -4.))
  p._unsupported_birth_context = tuple(context)
  assert _unsupported_birth_view(p, (obj,)) == (obj,)


def test_unsupported_birth_duplicate_pid_keeps_unexcluded_object():
  p, obj = _unsupported_birth_fixture()
  established = replace(obj, age_scans=2)
  assert _unsupported_birth_view(p, (obj, established)) == (established,)


def _publication_edge_model(width=2., std=.1):
  return NS(roadEdges=[NS(x=(0., 160.), y=(-width, -width)),
                       NS(x=(0., 160.), y=(width, width))], roadEdgeStds=(std, std))


@pytest.mark.usefixtures('birth_and_corner_gates')
def test_current_native_edge_excludes_established_unsupported_point_without_track_or_alias_change():
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, age_scans=20)
  ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  p.road_edge_filter.ingest_model(_publication_edge_model(), ns)
  assert _unsupported_birth_view(p, (obj,), ns) == (obj,)
  assert p.publication_view((obj,), ns) == ()
  msg = structs.RadarData.new_message()
  bosch_append_points(msg, p.publication_view((obj,), ns), 20., ns,
                      alias=p.publication_aliases.physical_to_alias)
  assert not msg.points
  assert obj.members[0].raw_track_id == 7 and obj.age_scans == 20
  assert p.publication_aliases.physical_to_alias == {1000001: 32}


@pytest.mark.usefixtures('birth_and_corner_gates')
@pytest.mark.parametrize('protection', ['vision', 'oem', 'word0', 'camera', 'ambiguous_camera',
                                      'corridor', 'stationary', 'ambiguous_raw', 'competing_raw',
                                      'coasted_raw', 'missing_pose', 'disabled'])
def test_current_native_edge_preserves_supported_or_uncertain_points(protection):
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, age_scans=20)
  ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  p.road_edge_filter.ingest_model(_publication_edge_model(), ns)
  if protection == 'vision': obj = replace(obj, vision_supported=True)
  if protection == 'oem': obj = replace(obj, oem_selected=True)
  if protection == 'word0': p._debug_processed_pids = {obj.physical_track_id}
  if protection in ('camera', 'ambiguous_camera'):
    p.camera_extended._associate = lambda *args: (1 if protection == 'camera' else 2, 0, 0)
  if protection == 'corridor': obj = replace(obj, y_rel=1.8)
  if protection == 'stationary': obj = replace(obj, v_rel=-20.)
  if protection == 'ambiguous_raw': p.tracker.raw_manager.last_identity_evidence[7] = (_UNSUPPORTED_BIRTH_NS, 1, True)
  if protection == 'competing_raw': p.tracker.raw_manager.last_identity_evidence[7] = (_UNSUPPORTED_BIRTH_NS, 2, False)
  if protection == 'coasted_raw': p.last_scan_timestamp_ns += 1
  if protection == 'missing_pose':
    context = list(p._unsupported_birth_context)
    context[2] = None
    p._unsupported_birth_context = tuple(context)
  if protection == 'disabled': p.unsupported_birth.enabled = False
  objects = (obj,)
  assert p.publication_view(objects, ns) == objects


@pytest.mark.usefixtures('birth_and_corner_gates')
def test_current_native_edge_rereads_support_and_model_even_at_same_publication_time():
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, age_scans=20)
  ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  p.road_edge_filter.ingest_model(_publication_edge_model(), ns)
  assert not p.publication_view((obj,), ns)
  p.camera_extended._associate = lambda *args: (1, 0, 0)
  assert p.publication_view((obj,), ns) == (obj,)
  p.camera_extended._associate = lambda *args: (0, -1, -1)
  assert not p.publication_view((obj,), ns)
  p.road_edge_filter.ingest_model(_publication_edge_model(width=20.), ns)
  assert p.publication_view((obj,), ns) == (obj,)
  p.road_edge_filter.ingest_model(_publication_edge_model(), ns)
  assert not p.publication_view((obj,), ns)
  p.road_edge_filter.ingest_model(_publication_edge_model(std=2.), ns)
  assert p.publication_view((obj,), ns) == (obj,)


@pytest.mark.usefixtures('birth_and_corner_gates')
@pytest.mark.parametrize('model_ns', [_UNSUPPORTED_BIRTH_NS - 200_000_001,
                                   _UNSUPPORTED_BIRTH_NS + 50_000_001])
def test_current_native_edge_missing_stale_or_future_model_keeps_point(model_ns):
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, age_scans=20)
  ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  assert p.publication_view((obj,), ns) == (obj,)
  p.road_edge_filter.ingest_model(_publication_edge_model(), model_ns)
  assert p.publication_view((obj,), ns) == (obj,)


@pytest.mark.usefixtures('birth_and_corner_gates')
def test_current_native_edge_uses_projected_float32_point_not_unprojected_anchor():
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, age_scans=20, y_rel=4.025)
  ns = _UNSUPPORTED_BIRTH_NS + 100_000_000
  # left(d) = -2 - .02*d: the anchor is inside the margin, but the
  # current native range has moved 0.5m toward the narrower boundary.
  p.road_edge_filter.ingest_model(NS(
    roadEdges=[NS(x=(0., 160.), y=(-2., -5.2)), NS(x=(0., 160.), y=(2., 5.2))],
    roadEdgeStds=(0., 0.)), ns)
  assert obj.y_rel < 3. + .02 * (obj.d_rel + 1.52)
  assert not p.publication_view((obj,), ns)


def test_native_surface_helper_exactly_matches_capnp_rounding_and_signed_zero():
  import struct
  from opendbc.car.hyundai.radar_bosch import BoschPublishedSurface, bosch_native_surface_coordinates
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, published_surface=BoschPublishedSurface(7, 51.1234567, -0., -3.1234567))
  ns = _UNSUPPORTED_BIRTH_NS + 73_000_001
  msg = structs.RadarData.new_message()
  bosch_append_points(msg, (obj,), 20., ns, alias=p.publication_aliases.physical_to_alias)
  actual = msg.points[0]
  calculated = bosch_native_surface_coordinates(obj, ns)
  assert [struct.pack('<f', value) for value in calculated] == [
    struct.pack('<f', getattr(actual, field)) for field in ('dRel', 'yRel', 'vRel')]


@pytest.mark.usefixtures('birth_and_corner_gates')
def test_current_native_edge_real_interface_preserves_scc_control_point(monkeypatch):
  from opendbc.car.hyundai.radar_interface import RadarInterface
  from opendbc.car.interfaces import RadarInterfaceBase
  def legacy_result(*args):
    ret = structs.RadarData.new_message()
    point = ret.init('points', 1)[0]
    point.trackId = 0
    point.dRel = 10.
    point.vRel = -1.
    point.measured = True
    point.radarSource = 'scc'
    return ret
  monkeypatch.setattr(RadarInterfaceBase, 'update_carrot', legacy_result)
  interface = RadarInterface.__new__(RadarInterface)
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, age_scans=20)
  ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  p.road_edge_filter.ingest_model(_publication_edge_model(), ns)
  p.publication_aliases.physical_to_alias = {}
  p.tracker.group_manager.states = {obj.physical_track_id: object()}
  interface.bosch = p
  interface._bosch_objects = (obj,)
  interface._bosch_now_ns = ns
  interface.v_ego = 20.
  result = interface.update_carrot(20., 0., 1.05, [])
  assert len(result.points) == 1 and result.points[0].radarSource == 'scc'
  assert result.points[0].trackId == 0 and result.points[0].dRel == 10.


def _far_corner_fixture(side=1):
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, age_scans=20, y_rel=4. * side)
  ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  context = list(p._unsupported_birth_context)
  context[2] = -.05 * side
  context[3] = ((0., 0.), (1., 0.), (50., 20. * side), (160., 50. * side))
  p._unsupported_birth_context = tuple(context)
  edges = ((12., 22.) if side == 1 else (-22., -12.))
  model = NS(roadEdges=[NS(x=(0., 160.), y=(edge, edge)) for edge in edges],
             roadEdgeStds=(.2, 2.) if side == 1 else (2., .2))
  p.road_edge_filter.ingest_model(model, ns)
  return p, obj, ns, model


@pytest.mark.usefixtures('birth_and_corner_gates')
@pytest.mark.parametrize('side', (1, -1))
def test_far_corner_uses_confident_exterior_despite_uncertain_opposite_edge(side):
  p, obj, ns, model = _far_corner_fixture(side)
  assert p.road_edge_filter.model_edges[-1][1] is None  # scan gate unchanged
  assert not p.publication_view((obj,), ns)
  # Current camera support releases at the same timestamp; no history hold.
  p.camera_extended._associate = lambda *args: (1, 0, 0)
  assert p.publication_view((obj,), ns) == (obj,)


@pytest.mark.usefixtures('birth_and_corner_gates')
@pytest.mark.parametrize('change', ('near_curve', 'opposite_yaw', 'opposite_path',
    'uncertain_exterior', 'invalid_opposite', 'crossed_edges', 'future', 'stale', 'tied_raw'))
def test_far_corner_keeps_without_independent_current_geometry(change):
  p, obj, ns, model = _far_corner_fixture()
  context = list(p._unsupported_birth_context)
  if change == 'near_curve': context[3] = ((0., 0.), (1., 0.), (50., 2.), (160., 4.))
  elif change == 'opposite_yaw': context[2] = .05
  elif change == 'opposite_path': context[3] = ((0., 0.), (1., 0.), (50., -20.), (160., -50.))
  elif change == 'uncertain_exterior': model.roadEdgeStds = (1.01, 2.)
  elif change == 'invalid_opposite': model.roadEdgeStds = (.2, float('nan'))
  elif change == 'crossed_edges': model.roadEdges.reverse()
  elif change == 'future': p.road_edge_filter.invalidate_model()
  elif change == 'stale': p.road_edge_filter.invalidate_model()
  elif change == 'tied_raw': p.tracker.raw_manager.last_identity_evidence[7] = (_UNSUPPORTED_BIRTH_NS, 2, True)
  p._unsupported_birth_context = tuple(context)
  stamp = ns + 1 if change == 'future' else ns - 200_000_001 if change == 'stale' else ns
  p.road_edge_filter.ingest_model(model, stamp)
  assert p.publication_view((obj,), ns) == (obj,)


@pytest.mark.usefixtures('birth_and_corner_gates')
def test_far_corner_invalid_current_sample_cannot_reuse_previous_good_edge():
  p, obj, ns, model = _far_corner_fixture()
  assert not p.publication_view((obj,), ns)
  model.roadEdgeStds = (2., 2.)
  p.road_edge_filter.ingest_model(model, ns)
  assert p.publication_view((obj,), ns) == (obj,)


@pytest.mark.usefixtures('birth_and_corner_gates')
def test_far_corner_distinguishes_competing_edges_from_a_tied_assignment():
  p, obj, ns, _ = _far_corner_fixture()
  p.tracker.raw_manager.last_identity_evidence[7] = (_UNSUPPORTED_BIRTH_NS, 2, False)
  assert not p.publication_view((obj,), ns)
  p.tracker.raw_manager.last_identity_evidence[7] = (_UNSUPPORTED_BIRTH_NS, 2, True)
  assert p.publication_view((obj,), ns) == (obj,)


def test_default_corner_gate_is_far_only_and_birth_guard_off():
  module = sys.modules[BoschRadarProvider.__module__]
  assert module.BOSCH_SURFACE_CORNER_GATE and module.BOSCH_SURFACE_CORNER_MIN_RANGE_M == 100.
  assert not module.BOSCH_UNSUPPORTED_BIRTH_PUBLICATION


def test_default_corner_gate_keeps_near_unsupported_point_outside_edge(monkeypatch):
  # Exit lanes, gores, shoulders and parallel roads put real cars outside the model road edge.
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, age_scans=20)
  ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  p.road_edge_filter.ingest_model(_publication_edge_model(), ns)
  assert p.publication_view((obj,), ns) == (obj,)
  # The same point is removed by the mechanism itself; only the range floor keeps it.
  monkeypatch.setattr(sys.modules[BoschRadarProvider.__module__], 'BOSCH_SURFACE_CORNER_MIN_RANGE_M', 0.)
  assert p.publication_view((obj,), ns) == ()


@pytest.mark.parametrize('d_rel, kept', [(100.2, True), (100.25, False), (150., False)])
def test_default_corner_gate_floor_uses_published_native_range(d_rel, kept):
  from opendbc.car.hyundai.radar_bosch import bosch_native_surface_coordinates
  p, obj = _unsupported_birth_fixture()
  raw = replace(obj.members[0], detection=replace(obj.members[0].detection, d_rel=d_rel))
  obj = replace(obj, age_scans=20, d_rel=d_rel, members=(raw,))
  ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  p.road_edge_filter.ingest_model(_publication_edge_model(), ns)
  # 50 ms of v_rel -5 m/s moves the published range 0.25 m nearer than the anchor.
  assert (bosch_native_surface_coordinates(obj, ns)[0] >= 100.) == (not kept)
  assert p.publication_view((obj,), ns) == ((obj,) if kept else ())


def test_default_corner_gate_far_point_keeps_current_support():
  p, obj = _unsupported_birth_fixture()
  obj = replace(obj, age_scans=20, d_rel=120.)
  ns = _UNSUPPORTED_BIRTH_NS + 50_000_000
  p.road_edge_filter.ingest_model(_publication_edge_model(), ns)
  assert p.publication_view((obj,), ns) == ()
  p.camera_extended._associate = lambda *args: (1, 0, 0)
  assert p.publication_view((obj,), ns) == (obj,)


def test_native_surface_helper_uses_published_member_measurement_time():
  import struct
  from opendbc.car.hyundai.radar_bosch import BoschPublishedSurface, bosch_native_surface_coordinates
  _, obj = _unsupported_birth_fixture()
  other = replace(obj.members[0], raw_track_id=8,
                  detection=replace(obj.members[0].detection, timestamp_ns=_UNSUPPORTED_BIRTH_NS + 20_000_000))
  obj = replace(obj, members=(obj.members[0], other),
                published_surface=BoschPublishedSurface(8, 51.1234567, -.1, -3.1234567))
  ns = _UNSUPPORTED_BIRTH_NS + 73_000_001
  msg = structs.RadarData.new_message()
  bosch_append_points(msg, (obj,), 20., ns)
  assert [struct.pack('<f', v) for v in bosch_native_surface_coordinates(obj, ns)] == [
    struct.pack('<f', getattr(msg.points[0], field)) for field in ('dRel', 'yRel', 'vRel')]


def test_append_keeps_earlier_points_and_writes_what_fill_point_writes():
  # bosch_append_points copies earlier (SCC) points before re-initializing the
  # list and writes Bosch points through cached schema fields.
  ns = 1_000_000_000
  obj = physical(1_000_001, 16.25, .5, 2., ns)
  msg = structs.RadarData.new_message()
  scc = msg.init('points', 1)[0]
  scc.trackId, scc.dRel, scc.vRel, scc.aRel, scc.measured, scc.radarSource = 0, 10., -1., math.nan, True, 'scc'
  bosch_append_points(msg, (obj,), 10., ns + 10_000_000, alias={obj.physical_track_id: 7},
                      a_lead_by_pid={obj.physical_track_id: .5})
  assert len(msg.points) == 2
  kept = msg.points[0]
  assert (kept.trackId, kept.dRel, kept.vRel, kept.radarSource, kept.measured) == (0, 10., -1., 'scc', True)
  assert math.isnan(kept.aRel)
  expected = structs.RadarData.new_message().init('points', 1)[0]
  bosch_fill_point(expected, obj, 10., {obj.physical_track_id: 7}, .5)
  expected.dRel += expected.vRel * (10_000_000 * 1e-9)
  assert msg.points[1].copy().to_bytes() == expected.copy().to_bytes()


def test_append_points_leaves_no_reference_cycle():
  # radarcan runs with gc disabled, so a reference cycle per publication is never
  # freed; reading schema.fields per publication leaked 185 MB/h on device.
  ns = 1_000_000_000
  obj = physical(1_000_001, 16.25, .5, 2., ns)
  bosch_append_points(structs.RadarData.new_message(), (obj,), 10., ns)
  gc.collect()
  gc.disable()
  try:
    for _ in range(20):
      bosch_append_points(structs.RadarData.new_message(), (obj,), 10., ns)
    assert gc.collect() == 0
  finally:
    gc.enable()
