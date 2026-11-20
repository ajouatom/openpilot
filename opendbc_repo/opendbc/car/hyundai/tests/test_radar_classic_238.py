# classic Hyundai 0x238 객체 디코더의 주소 구조와 운동학 값을 검증하는 테스트
import pytest

from opendbc.car.hyundai.radar_classic_238 import (
  CLASSIC_238_END_ADDR,
  CLASSIC_238_START_ADDR,
  Classic238AssociationTracker,
  Classic238DisplayTrack,
  Classic238Assembler,
  Classic238DisplayTracker,
  Classic238Object,
  Classic238RadarKinematics,
  Classic238TrackObservation,
  Classic238Triplet,
  classic_238_address_role,
  classic_238_display_ids,
  classic_238_longitudinal_tracks,
  classic_238_radar_available,
)


class TestClassic238ObjectDecoder:
  def test_k7_active_object_kinematics(self):
    obj = Classic238Object.from_first_frame(bytes.fromhex("0c797fdb8fe1f204"))

    assert obj.status == 1
    assert obj.d_rel == pytest.approx(19.95)
    assert obj.y_rel == pytest.approx(0.15)
    assert obj.v_lead == pytest.approx(15.10)

  @pytest.mark.parametrize(
    "address, expected",
    (
      (0x238, (0, 0)),
      (0x239, (0, 1)),
      (0x23A, (0, 2)),
      (0x23B, (1, 0)),
      (0x253, (9, 0)),
      (0x255, (9, 2)),
    ),
  )
  def test_three_frames_map_to_ten_slots(self, address, expected):
    assert classic_238_address_role(address) == expected

  @pytest.mark.parametrize("address", (CLASSIC_238_START_ADDR - 1, CLASSIC_238_END_ADDR + 1))
  def test_address_outside_group_is_rejected(self, address):
    with pytest.raises(ValueError):
      classic_238_address_role(address)

  def test_non_eight_byte_frame_is_rejected(self):
    with pytest.raises(ValueError):
      Classic238Object.from_first_frame(bytes(7))

  def test_triplet_counters_and_object_sequence(self):
    triplet = Classic238Triplet.from_frames(
      bytes.fromhex("0c797fdb8fe1f204"),
      bytes.fromhex("0f9d774945c80002"),
      bytes.fromhex("085c40e8ffe084ec"),
    )

    assert triplet.counter_consistent
    assert triplet.rolling_counters == (0, 0, 0)
    assert triplet.object_sequence == 0xE8
    assert triplet.obj.d_rel == pytest.approx(19.95)

  def test_triplet_counter_mismatch_is_visible(self):
    triplet = Classic238Triplet.from_frames(
      bytes.fromhex("0c797fdb8fe1f204"),
      bytes.fromhex("4f9d774945c80002"),
      bytes.fromhex("085c40e8ffe084ec"),
    )

    assert not triplet.counter_consistent
    assert triplet.rolling_counters == (0, 1, 0)

  def test_live_assembler_emits_one_complete_consistent_triplet(self):
    assembler = Classic238Assembler()
    frames = (
      bytes.fromhex("0c797fdb8fe1f204"),
      bytes.fromhex("0f9d774945c80002"),
      bytes.fromhex("085c40e8ffe084ec"),
    )
    assert assembler.update(1_000_000_000, 0x238, frames[0]) is None
    assert assembler.update(1_010_000_000, 0x239, frames[1]) is None
    slot, triplet = assembler.update(1_020_000_000, 0x23A, frames[2])
    assert slot == 0
    assert triplet.object_sequence == 0xE8
    assert assembler.update(1_020_000_000, 0x23A, frames[2]) is None

  def test_live_assembler_rejects_counter_mismatch_and_stale_triplet(self):
    mismatch = Classic238Assembler()
    assert mismatch.update(1_000_000_000, 0x238, bytes.fromhex("0c797fdb8fe1f204")) is None
    assert mismatch.update(1_010_000_000, 0x239, bytes.fromhex("4f9d774945c80002")) is None
    assert mismatch.update(1_020_000_000, 0x23A, bytes.fromhex("085c40e8ffe084ec")) is None

    stale = Classic238Assembler(max_age_ns=10)
    assert stale.update(100, 0x238, bytes.fromhex("0c797fdb8fe1f204")) is None
    assert stale.update(105, 0x239, bytes.fromhex("0f9d774945c80002")) is None
    assert stale.update(120, 0x23A, bytes.fromhex("085c40e8ffe084ec")) is None

  @pytest.mark.parametrize("spread_ns, emitted", ((13_200_000, True), (30_000_000, True), (40_000_000, False)))
  def test_live_assembler_limits_triplet_spread(self, spread_ns, emitted):
    assembler = Classic238Assembler()
    assert assembler.update(1_000_000_000, 0x238, bytes.fromhex("0c797fdb8fe1f204")) is None
    assert assembler.update(1_000_000_000 + spread_ns // 2, 0x239, bytes.fromhex("0f9d774945c80002")) is None
    result = assembler.update(1_000_000_000 + spread_ns, 0x23A, bytes.fromhex("085c40e8ffe084ec"))
    assert (result is not None) == emitted

  def test_live_assembler_rejects_same_counter_from_four_scans_earlier(self):
    # The 2-bit counter repeats every 4 scans (~120 ms): a stale first frame
    # with a matching counter must not be combined with the current scan.
    assembler = Classic238Assembler()
    assert assembler.update(1_000_000_000, 0x238, bytes.fromhex("0c797fdb8fe1f204")) is None
    assert assembler.update(1_120_000_000, 0x239, bytes.fromhex("0f9d774945c80002")) is None
    assert assembler.update(1_121_000_000, 0x23A, bytes.fromhex("085c40e8ffe084ec")) is None

  def test_radar_kinematics_are_ready_for_radar_point_mapping(self):
    triplet = Classic238Triplet.from_frames(
      bytes.fromhex("0c797fdb8fe1f204"),
      bytes.fromhex("0f9d774945c80002"),
      bytes.fromhex("085c40e8ffe084ec"),
    )
    point = Classic238RadarKinematics.from_triplet(triplet, v_ego=20.0)
    assert point.d_rel == pytest.approx(19.95)
    assert point.y_rel == pytest.approx(0.15)
    assert point.v_lead == pytest.approx(15.10)
    assert point.v_rel == pytest.approx(-4.90)
    assert point.yv_rel == 0.0

  def test_display_tracker_keeps_active_track_and_expires_it(self):
    tracker = Classic238DisplayTracker(max_age_ns=30_000_000)
    frames = (
      bytes.fromhex("0c797fdb8fe1f204"),
      bytes.fromhex("0f9d774945c80002"),
      bytes.fromhex("085c40e8ffe084ec"),
    )
    for offset, payload in enumerate(frames):
      tracker.update(1_000_000_000 + offset * 10_000_000, 0x238 + offset, payload, v_ego=20.0)
    tracker.finish_scan(1_020_000_000)

    tracks = tracker.current(1_020_000_000)
    assert len(tracks) == 1
    assert tracks[0].slot == 0
    assert tracks[0].track_id == 0
    assert tracks[0].status == 1
    assert tracks[0].object_sequence == 0xE8
    assert tracks[0].kinematics.v_rel == pytest.approx(-4.90)
    assert tracker.current(1_050_000_001) == []

  def test_display_tracker_removes_empty_slot(self):
    tracker = Classic238DisplayTracker()
    active = (
      bytes.fromhex("0c797fdb8fe1f204"),
      bytes.fromhex("0f9d774945c80002"),
      bytes.fromhex("085c40e8ffe084ec"),
    )
    for offset, payload in enumerate(active):
      tracker.update(1_000_000_000 + offset, 0x238 + offset, payload, v_ego=20.0)
    tracker.finish_scan(1_000_000_002)
    assert len(tracker.current(1_000_000_002)) == 1

    empty = (
      bytes.fromhex("0000000000000000"),
      bytes.fromhex("0000000000000000"),
      bytes.fromhex("0000000100000000"),
    )
    for offset, payload in enumerate(empty):
      tracker.update(1_010_000_000 + offset, 0x238 + offset, payload, v_ego=20.0)
    tracker.finish_scan(1_010_000_002)
    assert len(tracker.current(1_010_000_002)) == 1
    assert tracker.current(1_200_000_003) == []

  def test_association_keeps_id_across_slot_change_and_short_loss(self):
    tracker = Classic238AssociationTracker(max_age_ns=150_000_000)

    def observation(slot, d_rel, y_rel, v_lead):
      return Classic238TrackObservation(
        slot=slot, status=2, object_sequence=0,
        kinematics=Classic238RadarKinematics(
          d_rel=d_rel, y_rel=y_rel, v_rel=-1.0, v_lead=v_lead,
          a_rel=float("nan"), yv_rel=0.0,
        ),
      )

    first = tracker.update_scan(1_000_000_000, [observation(3, 30.0, -1.0, 20.0)])
    assert first[0].track_id == 0
    assert tracker.update_scan(1_050_000_000, [])[0].track_id == 0
    moved = tracker.update_scan(1_100_000_000, [observation(2, 29.9, -1.1, 20.2)])
    assert len(moved) == 1
    assert moved[0].track_id == 0
    assert moved[0].slot == 2

  def test_association_does_not_merge_distant_objects(self):
    tracker = Classic238AssociationTracker()
    first = Classic238TrackObservation(
      slot=0, status=2, object_sequence=0,
      kinematics=Classic238RadarKinematics(10.0, 0.0, 0.0, 10.0, float("nan"), 0.0),
    )
    distant = Classic238TrackObservation(
      slot=0, status=2, object_sequence=1,
      kinematics=Classic238RadarKinematics(30.0, 0.0, 0.0, 10.0, float("nan"), 0.0),
    )
    tracker.update_scan(1_000_000_000, [first])
    tracks = tracker.update_scan(1_050_000_000, [distant])
    assert {track.track_id for track in tracks} == {0, 1}


class TestClassic238LongitudinalSelection:
  @staticmethod
  def track(status=2, distance=12.0, lateral=0.5, age_ns=0, v_lead=8.0):
    kinematics = Classic238RadarKinematics(d_rel=distance, y_rel=lateral, v_rel=-2.0, v_lead=v_lead,
                                           a_rel=float("nan"), yv_rel=0.0)
    return Classic238DisplayTrack(track_id=3, slot=0, status=status, object_sequence=0,
                                  kinematics=kinematics, last_seen_ns=1_000_000_000 - age_ns)

  @pytest.mark.parametrize("v_lead", (8.0, 0.0, -1.0))
  @pytest.mark.parametrize("status", (2, 6))
  def test_fresh_confirmed_track_near_ego_lane_is_selected(self, v_lead, status):
    track = self.track(status=status, v_lead=v_lead)
    assert classic_238_longitudinal_tracks([track], 1_000_000_000) == [track]

  @pytest.mark.parametrize("kwargs", (
    {"status": 1, "distance": 20.1}, {"status": 1, "lateral": 1.3}, {"status": 3}, {"status": 4}, {"age_ns": 150_000_001},
    {"status": 5}, {"distance": 2.9}, {"distance": 200.1}, {"lateral": 4.6}, {"lateral": -4.6},
    {"v_lead": -1.1},
  ))
  def test_unverified_stale_or_implausible_tracks_are_rejected(self, kwargs):
    assert classic_238_longitudinal_tracks([self.track(**kwargs)], 1_000_000_000) == []

  @pytest.mark.parametrize("lateral", (2.1, -3.5, 4.5))
  def test_confirmed_adjacent_lane_track_is_passed_for_cut_in_tracking(self, lateral):
    track = self.track(lateral=lateral)
    assert classic_238_longitudinal_tracks([track], 1_000_000_000) == [track]

  @pytest.mark.parametrize(("distance", "lateral"), [(3.0, 0.0), (8.5, 0.5), (20.0, -1.2)])
  def test_near_in_lane_status_one_is_a_tentative_candidate(self, distance, lateral):
    track = self.track(status=1, distance=distance, lateral=lateral)
    assert classic_238_longitudinal_tracks([track], 1_000_000_000) == [track]

  def test_stream_detection_requires_all_triplets_without_legacy_tracks(self):
    fingerprint = {addr: 8 for addr in range(CLASSIC_238_START_ADDR, CLASSIC_238_END_ADDR + 1)}
    assert classic_238_radar_available(fingerprint)
    assert not classic_238_radar_available({**fingerprint, 0x500: 8})
    assert not classic_238_radar_available({**fingerprint, CLASSIC_238_END_ADDR: 7})
    del fingerprint[0x247]
    assert not classic_238_radar_available(fingerprint)


class TestClassic238ObjectIdAssociation:
  @staticmethod
  def observation(object_id, distance, lateral=0.0, slot=0, v_lead=8.0):
    kinematics = Classic238RadarKinematics(d_rel=distance, y_rel=lateral, v_rel=0.0, v_lead=v_lead,
                                           a_rel=float("nan"), yv_rel=0.0)
    return Classic238TrackObservation(slot=slot, status=2, object_sequence=0, kinematics=kinematics,
                                      object_id=object_id)

  def test_triplet_object_id_requires_matching_mirror(self):
    first, third = bytes.fromhex("0c797fdb8fe1f204"), bytes.fromhex("085c40e8ffe084ec")
    assert Classic238Triplet.from_frames(first, bytes.fromhex("0f9d774945c80002"), third).object_id == 2
    assert Classic238Triplet.from_frames(first, bytes.fromhex("0f9d774945c80003"), third).object_id is None

  def test_object_id_keeps_identity_when_positions_cross(self):
    tracker = Classic238AssociationTracker()
    tracker.update_scan(0, [self.observation(10, 20.0, slot=0), self.observation(11, 21.5, 0.3, slot=1)])
    ids = {track.object_id: track.track_id for track in tracker.current(0)}
    # Position alone would swap the two tracks; the matching IDs keep them.
    tracks = tracker.update_scan(30_000_000, [self.observation(10, 21.2, 0.2, slot=1), self.observation(11, 20.2, 0.1, slot=0)])
    assert {track.object_id: track.track_id for track in tracks} == ids

  def test_same_object_id_with_physical_jump_starts_new_track(self):
    tracker = Classic238AssociationTracker()
    first = tracker.update_scan(0, [self.observation(10, 20.0)])[0].track_id
    tracks = tracker.update_scan(30_000_000, [self.observation(10, 45.0)])
    assert [track.track_id for track in tracks if track.last_seen_ns == 30_000_000] != [first]

  def test_unknown_object_id_falls_back_to_position(self):
    tracker = Classic238AssociationTracker()
    first = tracker.update_scan(0, [self.observation(None, 20.0)])[0].track_id
    tracks = tracker.update_scan(30_000_000, [self.observation(None, 20.3)])
    assert [track.track_id for track in tracks] == [first]

  def test_same_object_id_resumes_track_after_short_gap(self):
    tracker = Classic238AssociationTracker()
    first = tracker.update_scan(0, [self.observation(10, 20.0)])[0].track_id
    assert tracker.current(200_000_000) == []
    tracks = tracker.update_scan(250_000_000, [self.observation(10, 20.4)])
    assert [track.track_id for track in tracks] == [first]

  def test_gap_resume_rejects_other_id_long_gap_and_missing_id(self):
    for object_id, resume_ns, first_id in ((11, 250_000_000, 10), (10, 400_000_000, 10), (None, 250_000_000, None)):
      tracker = Classic238AssociationTracker()
      first = tracker.update_scan(0, [self.observation(first_id, 20.0)])[0].track_id
      tracker.current(200_000_000)
      tracks = tracker.update_scan(resume_ns, [self.observation(object_id, 20.4)])
      assert [track.track_id for track in tracks] != [first]

  def test_display_ids_use_radar_object_id_once(self):
    def track(track_id, object_id, seen_ns):
      kinematics = Classic238RadarKinematics(d_rel=20.0, y_rel=0.0, v_rel=0.0, v_lead=8.0,
                                             a_rel=float("nan"), yv_rel=0.0)
      return Classic238DisplayTrack(track_id=track_id, slot=0, status=2, object_sequence=0,
                                    kinematics=kinematics, last_seen_ns=seen_ns, object_id=object_id)

    tracks = [track(0, 17, 0), track(1, 17, 30_000_000), track(2, None, 30_000_000), track(3, 63, 0)]
    # The fresher holder of a reused ID keeps it; unknown and older duplicates fall back.
    assert classic_238_display_ids(tracks) == [1000, 17, 1002, 63]