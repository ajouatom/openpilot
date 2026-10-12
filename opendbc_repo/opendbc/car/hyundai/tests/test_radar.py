import gc
import math
import sys
import weakref
from dataclasses import replace
from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
import opendbc.car.hyundai.hyundaicanfd as hyundaicanfd
import opendbc.car.hyundai.radar_interface as radar_interface_module
import opendbc.car.hyundai.radar_bosch as radar_bosch_module
from opendbc.car.hyundai.radar_interface import (
  CORNER_OBJECT_STABLE_TRACK_ID_START,
  RADAR_MSG_COUNT,
  RADAR_MSG_COUNT3,
  RADAR_MSG_COUNT4,
  RADAR_REQUIRED_MSG_COUNT,
  RADAR_START_ADDR_CANFD3,
  BOSCH_B5_ACTIVE,
  BOSCH_B5_PARENT_STABILITY_SCANS,
  BOSCH_ALEAD_SAMPLE_PERIOD_S,
  BOSCH_CAMERA_ASSOC_ASSIGNED,
  BOSCH_CAMERA_ASSOC_UNRESOLVED,
  BOSCH_OEM_STATE_NONE,
  BoschObjectGroupManager,
  BoschB5ModelContext,
  BoschBirthB5Defer,
  BoschGroupingConfig,
  BoschLeadAccelerationEstimator,
  BoschPhysicalObject,
  BoschPhysicalTracker,
  BoschPublicationAliasAllocator,
  BoschRadarProvider,
  BoschRawDetection,
  BoschRawTrack,
  BoschRawTrackManager,
  BoschRawTrackingConfig,
  CornerObjectTrackIdManager,
  RadarInterface,
  bosch_append_points,
  bosch_fill_point,
  bosch_to_native_radar_data,
  canfd_group2_track_status,
  corner_object_position_valid,
  deduplicate_corner_candidates,
)
from opendbc.car.radar_tracks import MyTrack
from opendbc.car.hyundai.values import BOSCH_MRREVO14F_CAR, CAR, HyundaiExtFlags, HyundaiFlags, bosch_mrrevo14f_bus


class TestBoschRectangularCertificate:
  @staticmethod
  def margin(rows, columns, unmatched=.65):
    return 1e-12 * max(1., 2 * unmatched) * (rows + columns + 1)

  def test_large_unique_component_skips_square_dummy_rows(self, monkeypatch):
    # One connected 59-node chain, with a strictly best real matching and one
    # missed track. No detection or gate is removed by the fast certificate.
    edges = [[(i, .01)] + ([(i + 1, .2)] if i < 28 else []) for i in range(29)]
    edges.append([(28, .3)])
    shapes = []
    original = radar_interface_module.bosch_numpy_linear_sum_assignment

    def solve(costs, **kwargs):
      shapes.append(costs.shape)
      return original(costs, **kwargs)

    monkeypatch.setattr(radar_bosch_module, 'bosch_numpy_linear_sum_assignment', solve)
    result = radar_interface_module._bosch_component_matching(
      list(range(30)), list(range(29)), edges, .65, self.margin(30, 29))
    assert result == ({i: i for i in range(29)}, True)
    assert shapes == [(30, 59)]

  @pytest.mark.parametrize('edges,columns', [
    ([[(0, .1), (1, .1)]], 2),  # alternating path moves the unmatched detection
    ([[(0, .1)], [(0, .1)]], 1),  # alternating path moves the missed track
    ([[(0, 1.3)]], 1),  # an edge ties the birth plus miss cost
    ([[(0, .1), (1, .1)], [(0, .1), (1, .1)]], 2),  # alternating cycle
  ])
  def test_birth_miss_and_cycle_ties_keep_square_choice(self, monkeypatch, edges, columns):
    rows = list(range(len(edges)))
    columns = list(range(columns))
    margin = self.margin(len(rows), len(columns))
    fast = radar_interface_module._bosch_rectangular_unique
    assert fast(rows, columns, edges, .65, margin) is None
    actual = radar_interface_module._bosch_component_matching(rows, columns, edges, .65, margin)
    monkeypatch.setattr(radar_bosch_module, '_bosch_rectangular_unique', lambda *args: None)
    expected = radar_interface_module._bosch_component_matching(rows, columns, edges, .65, margin)
    assert actual == expected
    assert not actual[1]

  def test_uncertified_tie_solve_never_calls_fast_path(self, monkeypatch):
    def unexpected(*args):
      raise AssertionError('tie-break must retain the original square solver')
    monkeypatch.setattr(radar_bosch_module, '_bosch_rectangular_unique', unexpected)
    assert radar_interface_module._bosch_component_matching([0], [0], [[(0, .1)]], .65) == ({0: 0}, False)

  def test_numerically_close_alternative_is_left_to_square(self):
    margin = self.margin(1, 2)
    assert radar_interface_module._bosch_rectangular_unique(
      [0], [0, 1], [[(0, .1), (1, .1 + margin)]], .65, margin) is None

  def test_misses_and_births_are_optional_and_order_is_preserved(self):
    edges = [[(0, .3)], [(1, .01)], [(2, .4)]]
    assert radar_interface_module._bosch_component_matching(
      [2, 1, 0], [2, 0, 1], edges, .05, self.margin(3, 3, .05)) == ({1: 1}, True)

  def test_open_component_and_invalid_cost_keep_refusal(self):
    margin = self.margin(1, 1)
    for edges in ([[(2, .1)]], [[(0, math.nan)]], [[(0, -math.inf)]]):
      assert radar_interface_module._bosch_component_matching([0], [0], edges, .65, margin) is None

  def test_invalid_duals_do_not_certify(self, monkeypatch):
    import numpy as np
    monkeypatch.setattr(radar_bosch_module, 'bosch_numpy_linear_sum_assignment',
                        lambda *a, **k: (np.array([0]), np.array([0]), np.array([0.]), np.array([1., 0.])))
    assert radar_interface_module._bosch_rectangular_unique([0], [0], [[(0, .1)]], .65, self.margin(1, 1)) is None


class TestMandoRadar:
  @staticmethod
  def make_interface(monkeypatch):
    class FakeParams:
      def get_int(self, key):
        return 1 if key == "EnableRadarTracks" else 0

    monkeypatch.setattr(radar_interface_module, "Params", FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = CAR.HYUNDAI_GRANDEUR_IG
    cp.flags = 0
    cp.extFlags = 0
    cp.radarUnavailable = False
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
    return RadarInterface(cp)

  def test_optional_upper_track_bank_and_32_slot_compatibility(self, monkeypatch):
    radar_interface = self.make_interface(monkeypatch)

    assert RADAR_MSG_COUNT == 64
    assert RADAR_REQUIRED_MSG_COUNT == 32
    assert radar_interface.radar_msg_count == 64
    assert radar_interface.radar_required_msg_count == 32
    assert radar_interface.trigger_msg_tracks == 0x51F
    assert not radar_interface.rcp_tracks.message_states[0x51F].ignore_alive
    assert radar_interface.rcp_tracks.message_states[0x520].ignore_alive
    assert radar_interface.rcp_tracks.message_states[0x53F].ignore_alive

    # This confirmed 0x52d target is the missing lead observed in a real
    # 64-slot Grandeur IG log. The lower bank still provides the cycle trigger.
    active_dat = bytes.fromhex("0060589b03fec0b2")
    lower_bank = [(addr, bytes(8), 1) for addr in range(0x500, 0x520)]
    radar_data = radar_interface.update([0, lower_bank + [(0x52D, active_dat, 1)]])
    point = next(point for point in radar_data.points if point.trackId == 77)

    assert not radar_data.errors.canError
    assert point.measured
    assert point.dRel == pytest.approx(math.cos(math.radians(2.2)) * 15.5)
    assert point.yRel == pytest.approx(-0.5 * math.sin(math.radians(2.2)) * 15.5)
    assert point.vRel == pytest.approx(1.78)
    assert point.aRel == pytest.approx(-0.04)

    # A 32-slot radar sends only the required lower bank. It must keep
    # publishing without a CAN error, and the optional point must not go stale.
    radar_data = radar_interface.update([0, lower_bank])
    assert not radar_data.errors.canError
    assert all(point.trackId != 77 for point in radar_data.points)


class TestCanfdGroup2Radar:
  @staticmethod
  def parse(dat):
    name = "RADAR_TRACK_3ac"
    parser = CANParser("hyundai_canfd_radar_generated", [(name, 20)], 1)
    parser.update([0, [(0x3AC, bytes.fromhex(dat), 1)]])
    return parser.vl[name]

  def test_tentative_overpass_reflection_is_not_confirmed(self):
    track = self.parse("cb6706810c0f2069acf0ff5cbc0680330200003dd0020000")

    assert track["VALID_CNT"] == 15
    assert track["VALID"] == 1
    assert track["LONG_DIST"] == pytest.approx(17.2)
    assert canfd_group2_track_status(track) == (True, 1)

  def test_confirmed_vehicle_remains_eligible(self):
    track = self.parse("dbc1db5261ff308db8230030be0b400100000000d0020000")

    assert track["VALID_CNT"] == 255
    assert track["VALID"] == 2
    assert track["LONG_DIST"] == pytest.approx(95.25)
    assert canfd_group2_track_status(track) == (True, 2)


class TestDensoRadar:
  @staticmethod
  def parse(addr, dat):
    name = f"RADAR_TRACK_{addr:x}"
    parser = CANParser("hyundai_kia_denso_front_radar_generated", [(name, 20)], 1)
    parser.update([0, [(addr, bytes.fromhex(dat), 1)]])
    return parser.vl[name]

  def test_active_track_signals(self):
    # Person walking toward the parked car, left of the camera center.
    track = self.parse(0x503, "bc047efcc1fe8b00")

    assert track["LONG_DIST"] == pytest.approx(7.1875)
    assert track["LAT_DIST"] == pytest.approx(-1.625)
    assert track["REL_SPEED"] == pytest.approx(-0.734375)
    assert track["OBJECT_STATE"] == 3

  def test_empty_track(self):
    track = self.parse(0x507, "53fff80000000081")

    assert track["LONG_DIST"] == pytest.approx(409.55)
    assert track["LAT_DIST"] == 0
    assert track["REL_SPEED"] == 0
    assert track["OBJECT_STATE"] == 0

  def test_long_range_lateral_distance(self):
    # Real driving sample: treating the signed field as -12 degrees would put
    # this target about 34 m sideways at 161 m. It is instead -3.0 m lateral.
    track = self.parse(0x506, "b664eafa00cd230b")

    assert track["LONG_DIST"] == pytest.approx(161.4625)
    assert track["LAT_DIST"] == pytest.approx(-3.0)
    assert track["OBJECT_STATE"] == 3

  def test_parser_selection_and_point_conversion(self, monkeypatch):
    class FakeParams:
      def get_int(self, key):
        return 1 if key == "EnableRadarTracks" else 0

    monkeypatch.setattr(radar_interface_module, "Params", FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = CAR.KIA_SORENTO
    cp.flags = 0
    cp.extFlags = HyundaiExtFlags.RADAR_GROUP4.value
    cp.radarUnavailable = False
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]

    radar_interface = RadarInterface(cp)

    assert radar_interface.radar_group4
    assert RADAR_MSG_COUNT4 == 8
    assert radar_interface.radar_msg_count == RADAR_MSG_COUNT4
    assert radar_interface.trigger_msg_tracks == 0x507

    active_dat = bytes.fromhex("bc047efcc1fe8b00")
    empty_dat = bytes.fromhex("bcfff80000000081")
    packets = [(addr, active_dat if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 35)

    assert point.measured
    assert point.dRel == pytest.approx(7.1875)
    assert point.yRel == pytest.approx(1.625)
    assert point.vRel == pytest.approx(-0.734375)
    assert math.isnan(point.aRel)

    # EN: Confirm that the long-range sample survives the filter and converts
    #     radar-left-negative to openpilot-left-positive coordinates.
    # KO: 장거리 샘플의 필터 통과와 레이더 좌측 음수 좌표가 openpilot 좌측
    #     양수 좌표로 변환되는지 확인함.
    long_range_dat = bytes.fromhex("b664eafa00cd230b")
    packets = [(addr, long_range_dat if addr == 0x506 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 38)

    assert point.dRel == pytest.approx(161.4625)
    assert point.yRel == pytest.approx(3.0)

    # EN: A state-0 raw detection must not enter a stable tracked-object slot.
    # KO: 상태 0인 raw detection이 안정적인 추적 객체 슬롯에 들어오지 않음을 확인함.
    raw_detection = bytes.fromhex("d702f4fc200000e4")
    packets = [(addr, raw_detection if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    assert not radar_data.points

    # EN: A real confirmed track beyond the former 205 m limit remains valid.
    # KO: 기존 205m 상한을 넘는 실제 확정 트랙도 유효하게 유지됨.
    confirmed_213m_track = bytes.fromhex("35854c0780f163e0")
    packets = [(addr, confirmed_213m_track if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 35)
    assert point.dRel == pytest.approx(213.275)
    assert point.yRel == pytest.approx(-3.75)

    # EN: The 325 m boundary is rejected, leaving ample separation from the
    #     409.55 m empty-slot sentinel.
    # KO: 325m 경계값을 제외해 409.55m 빈 슬롯 값과 충분한 간격을 확보함.
    boundary_track = bytes.fromhex("bccb200000000300")
    packets = [(addr, boundary_track if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    assert not radar_data.points

    # EN: The wider profile keeps a real stable track at 4.875 m, covering more
    #     of the outer adjacent lane than the conservative 4.5 m profile.
    # KO: 넓어진 필터에서 4.875m의 실제 안정 트랙을 유지해 보수적인 4.5m
    #     설정보다 바깥쪽 인접 차선을 더 넓게 포함함.
    outer_lane_track = bytes.fromhex("d80b66f640000300")
    packets = [(addr, outer_lane_track if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 35)
    assert point.yRel == pytest.approx(4.875)

    # EN: Tracks beyond the widened envelope are rejected as roadside clutter;
    #     this payload differs only in lateral distance (-7.0 m).
    # KO: 넓어진 범위를 벗어난 트랙은 도로변 잡음으로 제외함. 이 payload는
    #     횡방향 거리(-7.0m)만 다름.
    far_side_reflection = bytes.fromhex("d80b66f200000300")
    packets = [(addr, far_side_reflection if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    assert not radar_data.points


class TestRadarGroup3:
  @staticmethod
  def parse(addr, dat):
    name = f"RADAR_TRACK_{addr:x}"
    parser = CANParser("hyundai_canfd_radar_generated", [(name, 20)], 1)
    parser.update([0, [(addr, bytes.fromhex(dat), 1)]])
    return parser.vl[name]

  def test_group3_active_track(self):
    track = self.parse(0x406, "e1043b0f02590e692a227e16f80fe00f28fcc753a20a0000")

    assert track["OBJECT_ID"] == 15
    assert track["OBJECT_LENGTH"] == pytest.approx(4.4)
    assert track["LONG_DIST"] == pytest.approx(55.4)
    assert track["LAT_DIST"] == pytest.approx(-3.0)
    assert track["REL_SPEED"] == pytest.approx(4.4)

  def test_group3_empty_track(self):
    track = self.parse(0x407, "c03d3b0000000000ff0700000000000000d0020000000000")

    assert track["OBJECT_LENGTH"] == 0
    assert track["LONG_DIST"] == pytest.approx(204.7)
    assert track["LAT_DIST"] == 0
    assert track["REL_SPEED"] == 0

  def test_group3_parser_selection(self, monkeypatch):
    class FakeParams:
      def get_int(self, key):
        return 1 if key == "EnableRadarTracks" else 0

    monkeypatch.setattr(radar_interface_module, "Params", FakeParams)
    monkeypatch.setattr(hyundaicanfd, "Params", FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = next(car for car, dbc in radar_interface_module.DBC.items() if "hyundai_canfd" in dbc[Bus.pt])
    cp.flags = HyundaiFlags.CANFD.value
    cp.extFlags = HyundaiExtFlags.RADAR_GROUP3.value
    cp.radarUnavailable = False
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]

    radar_interface = RadarInterface(cp)

    assert radar_interface.radar_group3
    assert radar_interface.radar_start_addr == RADAR_START_ADDR_CANFD3
    assert radar_interface.radar_msg_count == RADAR_MSG_COUNT3
    assert radar_interface.trigger_msg_tracks == 0x41D

    active_dat = bytes.fromhex("e1043b0f02590e692a227e16f80fe00f28fcc753a20a0000")
    empty_dat = bytes.fromhex("c03d3b0000000000ff0700000000000000d0020000000000")
    packets = [(addr, active_dat if addr == 0x406 else empty_dat, 1) for addr in range(0x400, 0x41E)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId >= 1_000_000)

    assert point.measured
    assert point.dRel == pytest.approx(53.1)
    assert point.yRel == pytest.approx(-3.0)
    assert point.vRel == pytest.approx(4.4)
    first_id = point.trackId
    packets = [(addr, active_dat if addr == 0x410 else empty_dat, 1) for addr in range(0x400, 0x41E)]
    moved = radar_interface.update([0, packets])
    assert [p.trackId for p in moved.points] == [first_id]
    assert len([p for p in radar_interface.pts.values() if p.trackId == first_id]) == 1
    # Missing the occupied slot must not republish CANParser's cached value.
    missing = radar_interface.update([0, [(addr, empty_dat, 1) for addr in range(0x400, 0x41E) if addr != 0x410]])
    assert not missing.points
    returned = radar_interface.update([0, packets])
    assert returned.points[0].trackId != first_id



class TestCornerRadarObjectIdentity:
  @staticmethod
  def set_bits(data, start, size, value):
    for offset in range(size):
      bit = start + offset
      data[bit // 8] |= ((value >> offset) & 1) << (bit % 8)

  @pytest.mark.parametrize(
    "dbc,msg_name,addr,age_signal,id_signal,age_start,id_start",
    (
      ("hyundai_canfd_corner_radar_180_generated", "CORNER_RADAR_180_OBJECTS_180", 0x180,
       "SLOT1_AGE", "SLOT1_OBJECT_ID", 32, 44),
      ("hyundai_canfd_corner_radar_235_generated", "CORNER_RADAR_235_OBJECTS_235", 0x235,
       "OBJ_AGE", "OBJ_OBJECT_ID", 32, 44),
    ),
  )
  def test_object_identity_signals(self, dbc, msg_name, addr, age_signal, id_signal, age_start, id_start):
    data = bytearray(32)
    self.set_bits(data, age_start, 8, 23)
    self.set_bits(data, id_start, 7, 46)
    parser = CANParser(dbc, [(msg_name, 33)], 1)
    parser.update([0, [(addr, bytes(data), 1)]])

    assert parser.vl[msg_name][age_signal] == 23
    assert parser.vl[msg_name][id_signal] == 46

  def test_track_id_survives_slot_move_and_resets_with_age(self):
    manager = CornerObjectTrackIdManager()
    first = [(240, 108, 240, 40, 25.0, 2.8, 1.0, -0.2, 0.0)]
    first_id = manager.get_track_ids("corner180", first)[240]
    next_age = [(240, 108, 241, 40, 25.1, 2.8, 1.0, -0.2, 0.0)]
    other_source = [(201, 108, 241, 40, 25.1, 2.8, 1.0, -0.2, 0.0)]
    reset_age = [(240, 108, 2, 40, 25.2, 2.8, 1.0, -0.2, 0.0)]

    assert first_id == CORNER_OBJECT_STABLE_TRACK_ID_START
    assert manager.get_track_ids("corner180", next_age)[240] == first_id
    assert manager.get_track_ids("corner235", other_source)[201] != first_id
    assert manager.get_track_ids("corner180", reset_age)[240] != first_id

  def test_same_object_id_at_different_positions_remains_distinct(self):
    manager = CornerObjectTrackIdManager()
    candidates = [
      (201, 32, 111, 35, 13.75, 2.90, 5.00, -0.65, 0.0),
      (202, 32, 191, 35, 149.35, -2.90, 3.05, 0.00, 0.0),
    ]

    objects = deduplicate_corner_candidates(candidates)
    track_ids = manager.get_track_ids("corner235", objects)

    assert len(objects) == 2
    assert track_ids[201] != track_ids[202]

  def test_interface_publishes_distant_same_id_objects(self):
    radar_interface = RadarInterface.__new__(RadarInterface)
    radar_interface.v_ego = 20.0
    radar_interface.corner_object_track_ids = CornerObjectTrackIdManager()
    radar_interface.pts = {}
    for slot_id in (201, 202):
      radar_interface.pts[slot_id] = structs.RadarData.RadarPoint()

    candidates = [
      (201, 32, 111, 35, 13.10, 2.90, 4.90, -0.65, 0.0),
      (202, 32, 191, 35, 149.70, -2.90, 3.05, 0.00, 0.0),
    ]
    radar_interface._apply_corner_objects(
      "corner235", candidates, (201, 202),
    )

    near = radar_interface.pts[201]
    far = radar_interface.pts[202]
    assert near.measured and far.measured
    assert near.trackId != far.trackId
    assert near.dRel == pytest.approx(13.10)
    assert far.dRel == pytest.approx(149.70)

  def test_physical_slot_handoff_is_deduplicated_and_keeps_track_id(self):
    manager = CornerObjectTrackIdManager()
    first = [(201, 46, 23, 40, 25.0, 2.8, 1.0, -0.2, 0.0)]
    first_id = manager.get_track_ids("corner235", first)[201]
    handoff = [
      (201, 46, 24, 38, 25.1, 2.75, 1.0, -0.2, 0.0),
      (207, 46, 25, 42, 25.2, 2.70, 1.0, -0.2, 0.0),
    ]

    objects = deduplicate_corner_candidates(handoff)
    track_ids = manager.get_track_ids("corner235", objects)

    assert len(objects) == 1
    assert track_ids[207] == first_id

  def test_clipped_side_object_position_is_valid(self):
    assert corner_object_position_valid(0.0, 2.8)
    assert corner_object_position_valid(25.0, 0.2)
    assert not corner_object_position_valid(0.0, 0.0)
    assert not corner_object_position_valid(0.0, 5.0)


class TestCornerRadar430CandidateFilter:
  @staticmethod
  def slot_word(distance_raw, meta13=0, b2=10, b3=2):
    return distance_raw | (meta13 << 13) | (b2 << 16) | (b3 << 24)

  @classmethod
  def message(cls, slots):
    words = [0x010d1f40] * 7
    for slot, word in slots.items():
      words[slot - 1] = word

    dat = bytearray(32)
    for idx, word in enumerate(words):
      dat[4 + idx * 4:8 + idx * 4] = int(word).to_bytes(4, "little")
    return bytes(dat)

  @staticmethod
  def build_interface(monkeypatch):
    class FakeParams:
      def get_int(self, key):
        return 1 if key == "EnableCornerRadar" else 0

    monkeypatch.setattr(radar_interface_module, "Params", FakeParams)
    monkeypatch.setattr(hyundaicanfd, "Params", FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = next(car for car, dbc in radar_interface_module.DBC.items() if "hyundai_canfd" in dbc[Bus.pt])
    cp.flags = HyundaiFlags.CANFD.value
    cp.extFlags = HyundaiExtFlags.CORNER_RADAR_OBJECTS_430.value
    cp.radarUnavailable = True
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
    return RadarInterface(cp)

  @staticmethod
  def update_frames(radar_interface, packets, frames=5):
    radar_data = None
    for _ in range(frames):
      radar_data = radar_interface.update([0, packets])
    return radar_data

  def test_430_bins_are_not_promoted_to_live_tracks(self, monkeypatch):
    radar_interface = self.build_interface(monkeypatch)
    assert radar_interface.rcp_corner_objects_430 is None

    empty = self.message({})
    supported_bins = self.message({
      6: self.slot_word(1000),
      7: self.slot_word(1004),
    })
    packets = [(addr, supported_bins if addr == 0x436 else empty, 1) for addr in range(0x430, 0x438)]
    packets += [(addr, empty, 1) for addr in range(0x440, 0x448)]

    radar_data = self.update_frames(radar_interface, packets)

    assert all(str(point.radarSource) != "corner430" for point in radar_data.points)

  def test_430_expires_noncenter_inward_yvrel(self, monkeypatch):
    radar_interface = self.build_interface(monkeypatch)
    empty = self.message({})
    frame_defs = (
      (0x431, 4, 5),
      (0x433, 3, 4),
      (0x435, 2, 3),
      (0x430, 5, 6),
      (0x432, 4, 5),
      (0x434, 3, 4),
      (0x436, 2, 3),
    )

    radar_data = None
    for addr, first_slot, second_slot in frame_defs:
      msg = self.message({
        first_slot: self.slot_word(1000),
        second_slot: self.slot_word(1004),
      })
      packets = [(a, msg if a == addr else empty, 1) for a in range(0x430, 0x438)]
      packets += [(a, empty, 1) for a in range(0x440, 0x448)]
      radar_data = radar_interface.update([0, packets])
    radar_data = self.update_frames(radar_interface, packets, frames=3)
    points = {point.trackId: point for point in radar_data.points}

    assert 300 not in points
    assert all(str(point.radarSource) != "corner430" for point in points.values())


class TestBoschOffPathPublication:
  PID = 1_000_900
  START_NS = 1_000_000_000

  @classmethod
  def obj(cls, index=0, *, pid=None, raw_id=900, d=80., y=-7., v=-9.35,
          oem=False, vision=False, age=None):
    ns = cls.START_NS + index * 100_000_000
    detection = BoschRawDetection(ns, raw_id % 32, float(d), float(y), float(v), 1)
    member = BoschRawTrack(raw_id, detection, age or index + 1, False)
    return BoschPhysicalObject(pid or cls.PID, ns, (member,), raw_id, float(d), float(y), float(v),
                               oem, vision, age or index + 1, 'single_return')

  @staticmethod
  def filter():
    return radar_interface_module._BoschPublicationPassThrough()

  @classmethod
  def wall(cls, index, **kwargs):
    kwargs.setdefault('d', 80. - 9.35 * index * .1)
    return cls.obj(index, **kwargs)

  @pytest.mark.parametrize(('y_rel', 'v_rel'), (
    (.25, -10.),    # in-path stationary target
    (3.01, -10.),   # immediately outside the removed corridor boundary
    (-7., -10.),    # stationary roadside structure
    (4., -8.),      # adjacent-lane moving vehicle
    (8., 10.),      # opposite-direction vehicle
  ))
  def test_path_relevance_and_stationary_status_do_not_change_publication(self, y_rel, v_rel):
    qualifier = self.filter()
    obj = self.obj(y=y_rel, v=v_rel)
    objects = (obj,)
    assert qualifier.update(objects, obj.timestamp_ns, 10.) is objects

  def test_removed_boundaries_cannot_cause_publish_unpublish_chatter(self):
    qualifier = self.filter()
    for index, y_rel in enumerate((2.99, 3.01, 5.49, 5.51, 4.99, 3.0, -3.01, -5.51)):
      obj = self.wall(index, y=y_rel, v=-10.)
      objects = (obj,)
      assert qualifier.update(objects, obj.timestamp_ns, 10.) is objects

  @pytest.mark.parametrize('path', (
    (), ((0., 0.),), ((0., 0.), (50., math.nan)), ((50., 0.), (0., 0.)),
    ((0., 0.), (50., 4.), (100., 8.)),
  ))
  @pytest.mark.parametrize(('v_ego', 'yaw_rate'), ((0., 0.), (10., None), (math.nan, math.nan)))
  def test_path_availability_speed_and_yaw_do_not_gate_publication(self, path, v_ego, yaw_rate):
    qualifier = self.filter()
    obj = self.wall(0, y=-9., v=-10.)
    objects = (obj,)
    assert qualifier.update(objects, obj.timestamp_ns, v_ego, path, yaw_rate=yaw_rate) is objects

  def test_publication_hook_has_no_pid_member_or_temporal_state(self):
    qualifier = self.filter()
    first = self.wall(0)
    changed = self.wall(20, raw_id=901, y=9., v=-10.)
    for objects in ((first,), (), (changed,)):
      assert qualifier.update(objects, self.START_NS, 10.) is objects
    assert vars(qualifier) == {}

  def test_long_lived_far_offpath_static_is_always_published(self):
    qualifier = self.filter()
    for index in range(20):
      obj = self.wall(index, v=-9.5)
      objects = (obj,)
      assert qualifier.update(objects, obj.timestamp_ns, 10.) is objects

  @pytest.mark.parametrize(('field', 'value'), (('vision', True), ('oem', True)))
  def test_support_status_does_not_change_pass_through(self, field, value):
    qualifier = self.filter()
    obj = self.wall(0, **{field: value})
    objects = (obj,)
    assert qualifier.update(objects, obj.timestamp_ns, 10.) is objects

  def test_in_path_real_stationary_target_is_published(self):
    qualifier = self.filter()
    for index in range(20):
      obj = self.obj(index, d=30. - index, y=.25, v=-10.)
      objects = (obj,)
      assert qualifier.update(objects, obj.timestamp_ns, 10.) is objects

  def test_adjacent_slow_moving_vehicle_is_published(self):
    qualifier = self.filter()
    for index in range(20):
      obj = self.obj(index, d=50. - .88 * index, y=-7., v=-8.8)
      objects = (obj,)
      assert qualifier.update(objects, obj.timestamp_ns, 10.) is objects

  @pytest.mark.parametrize('side', (-1., 1.))
  def test_crossing_and_turning_lateral_motion_is_published(self, side):
    qualifier = self.filter()
    for index in range(16):
      obj = self.wall(index, y=side * (10. + .3 * index))
      objects = (obj,)
      assert qualifier.update(objects, obj.timestamp_ns, 10.) is objects

  def test_member_change_gap_and_segment_reset_need_no_qualifier_history(self):
    qualifier = self.filter()
    sequence = (
      self.wall(0),
      self.wall(8, raw_id=901),
      self.obj(20, d=70.65, y=-7., v=-9.35),
    )
    for obj in sequence:
      objects = (obj,)
      assert qualifier.update(objects, obj.timestamp_ns, 10.) is objects
    assert vars(qualifier) == {}

  def test_curve_path_and_yaw_do_not_gate_publication(self):
    qualifier = self.filter()
    path = ((0., 0.), (50., 4.), (100., 8.))
    for index in range(12):
      d_rel = 80. - .855 * index
      obj = self.obj(index, d=d_rel, y=d_rel * .08 + .4, v=-8.55)
      objects = (obj,)
      assert qualifier.update(objects, obj.timestamp_ns, 10., path, yaw_rate=.1) is objects

  def test_low_speed_parked_roadside_and_structure_are_published(self):
    qualifier = self.filter()
    for index, v_rel in enumerate((-1.35, -1.5)):
      obj = self.obj(index, d=40. - index, y=-9., v=v_rel)
      objects = (obj,)
      assert qualifier.update(objects, obj.timestamp_ns, 2.) is objects

  def test_multi_member_object_is_published_without_temporal_extension(self):
    qualifier = self.filter()
    obj = self.wall(0)
    detection = BoschRawDetection(obj.timestamp_ns, 6, obj.d_rel + .5, obj.y_rel, obj.v_rel, 1)
    member = BoschRawTrack(901, detection, 1, False)
    multi = BoschPhysicalObject(obj.physical_track_id, obj.timestamp_ns, obj.members + (member,),
                                obj.representative_raw_track_id, obj.d_rel, obj.y_rel, obj.v_rel,
                                obj.oem_selected, obj.vision_supported, obj.age_scans,
                                'temporal_complete_link')
    objects = (multi,)
    assert qualifier.update(objects, multi.timestamp_ns, 10.) is objects
    assert vars(qualifier) == {}

  def test_camera_mode_contract_does_not_change_publication(self):
    for mode in (0, 1, 3):  # OFF, SHADOW, ACTIVE_TEST
      provider = BoschRadarProvider(1, camera_extended_mode=mode)
      obj = self.wall(0, y=-9., v=-10.)
      objects = (obj,)
      assert provider.camera_extended.update(obj.timestamp_ns, objects, 10., 0.) is objects
      assert provider.qualifier.update(objects, obj.timestamp_ns, 10.) is objects


class TestBoschP91PersistentSpatialClone:
  START_NS = 8_000_000_000
  PARENT_PID = 1_001_000
  CANDIDATE_PID = 1_001_001

  @classmethod
  def obj(cls, pid, index, *, d, y, v=0., vision=False, oem=False, raw_id=None, members=1):
    ns = cls.START_NS + index * 100_000_000
    raw_id = raw_id if raw_id is not None else pid % 100_000
    tracks = []
    for member_index in range(members):
      detection = BoschRawDetection(ns, (raw_id + member_index) % 32, d + .25 * member_index,
                                    y, v, raw_word=raw_id + member_index)
      tracks.append(BoschRawTrack(raw_id + member_index, detection, index + 1, False))
    return BoschPhysicalObject(pid, ns, tuple(tracks), raw_id, d, y, v, oem, vision,
                               index + 1, 'single_return' if members == 1 else 'temporal_complete_link')

  @classmethod
  def pair(cls, index, *, candidate_y=5., candidate_d=41.3, candidate_v=0.,
           candidate_vision=False, candidate_oem=False, candidate_members=1):
    parent = cls.obj(cls.PARENT_PID, index, d=30., y=0., v=0., vision=True)
    candidate = cls.obj(cls.CANDIDATE_PID, index, d=candidate_d, y=candidate_y,
                        v=candidate_v, vision=candidate_vision, oem=candidate_oem,
                        members=candidate_members)
    return parent, candidate

  @staticmethod
  def detector(mode=radar_interface_module.BOSCH_P91_ACTIVE_TEST):
    return radar_interface_module._BoschPersistentSpatialCloneFilter(mode)

  def mature(self, detector, count=31):
    result = frozenset()
    for index in range(count):
      pair = self.pair(index)
      result = detector.update(pair, pair[0].timestamp_ns, 20.)
    return result

  def test_supported_parent_persistent_spatial_copy_suppresses_after_warmup(self):
    detector = self.detector()
    for index in range(30):
      pair = self.pair(index)
      assert detector.update(pair, pair[0].timestamp_ns, 20.) == frozenset()
    pair = self.pair(30)
    assert detector.update(pair, pair[0].timestamp_ns, 20.) == frozenset({self.CANDIDATE_PID})
    decision = detector.decisions[self.CANDIDATE_PID]
    assert decision['parent_pid'] == self.PARENT_PID
    assert decision['evidence_age_s'] >= 3.
    assert decision['v_rmse_mps'] == 0.

  def test_candidate_camera_support_is_immediate_keep_and_reset(self):
    detector = self.detector()
    assert self.mature(detector) == frozenset({self.CANDIDATE_PID})
    pair = self.pair(31, candidate_vision=True)
    assert detector.update(pair, pair[0].timestamp_ns, 20.) == frozenset()
    assert self.CANDIDATE_PID not in detector._states

  def test_candidate_word1_selection_is_immediate_keep_and_reset(self):
    detector = self.detector()
    assert self.mature(detector) == frozenset({self.CANDIDATE_PID})
    pair = self.pair(31, candidate_oem=True)
    assert detector.update(pair, pair[0].timestamp_ns, 20.) == frozenset()
    assert self.CANDIDATE_PID not in detector._states

  def test_candidate_word0_selection_is_immediate_keep_and_reset(self):
    detector = self.detector()
    assert self.mature(detector) == frozenset({self.CANDIDATE_PID})
    pair = self.pair(31)
    assert detector.update(pair, pair[0].timestamp_ns, 20.,
                           word0_pids=(self.CANDIDATE_PID,)) == frozenset()
    assert self.CANDIDATE_PID not in detector._states

  def test_lateral_cutin_excursion_never_matures(self):
    detector = self.detector()
    for index in range(40):
      pair = self.pair(index, candidate_y=7. - .25 * index)
      assert detector.update(pair, pair[0].timestamp_ns, 20.) == frozenset()

  def test_legitimate_large_vehicle_companion_is_kept(self):
    detector = self.detector()
    for index in range(50):
      pair = self.pair(index, candidate_y=.8, candidate_d=40., candidate_members=2)
      assert detector.update(pair, pair[0].timestamp_ns, 20.) == frozenset()
    assert not detector._states

  def test_short_lived_similar_speed_adjacent_object_is_kept(self):
    detector = self.detector()
    assert self.mature(detector, 20) == frozenset()

  def test_parent_disappearance_releases_and_resets(self):
    detector = self.detector()
    assert self.mature(detector) == frozenset({self.CANDIDATE_PID})
    candidate = self.pair(31)[1]
    assert detector.update((candidate,), candidate.timestamp_ns, 20.) == frozenset()
    assert not detector._states

  def test_state_cleanup_is_bounded_by_live_candidates(self):
    detector = self.detector()
    parent = self.pair(0)[0]
    candidates = tuple(self.obj(self.CANDIDATE_PID + index, 0, d=41. + index * .1,
                                y=5., raw_id=2_000 + index) for index in range(20))
    detector.update((parent,) + candidates, parent.timestamp_ns, 20.)
    assert len(detector._states) <= len(candidates)
    detector.update((), parent.timestamp_ns + 100_000_000, 20.)
    assert detector._states == {} and detector._support_until == {}

  def test_shadow_mode_leaves_publication_and_aliases_unchanged(self):
    provider = BoschRadarProvider(1, qualification=False, camera_extended_mode=0,
                                  p91_mode=radar_interface_module.BOSCH_P91_SHADOW)
    for index in range(31):
      pair = self.pair(index)
      provider.p91.update(pair, pair[0].timestamp_ns, 20.)
    pair = self.pair(30)
    aliases = provider.publication_aliases.update(pair[0].timestamp_ns,
                                                   [obj.physical_track_id for obj in pair],
                                                   {obj.physical_track_id for obj in pair})
    assert provider.p91.would_suppress == frozenset({self.CANDIDATE_PID})
    assert provider.publication_view(pair, pair[0].timestamp_ns) == pair
    assert set(aliases) == {self.PARENT_PID, self.CANDIDATE_PID}

  def test_active_test_veto_preserves_internal_pid_and_alias_binding(self):
    provider = BoschRadarProvider(1, qualification=False, camera_extended_mode=0,
                                  p91_mode=radar_interface_module.BOSCH_P91_ACTIVE_TEST)
    for index in range(31):
      pair = self.pair(index)
      provider.p91.update(pair, pair[0].timestamp_ns, 20.)
    pair = self.pair(30)
    provider.publication_aliases.update(pair[0].timestamp_ns,
                                        [obj.physical_track_id for obj in pair],
                                        {obj.physical_track_id for obj in pair})
    assert provider.publication_view(pair, pair[0].timestamp_ns) == (pair[0],)
    assert self.CANDIDATE_PID in provider.publication_aliases.physical_to_alias
    assert self.CANDIDATE_PID in provider.p91._states


class TestBoschPublicationAlias:
  PID = 1_000_475

  @classmethod
  def objects(cls):
    return BoschPhysicalTracker().update(1_000_000_000, (
      BoschRawDetection(1_000_000_000, 9, 37.125, -1.25, -3.125, raw_word=1),))

  def test_slot_member_and_representative_changes_keep_alias(self):
    allocator = BoschPublicationAliasAllocator()
    obj = replace(self.objects()[0], physical_track_id=self.PID)
    aliases = []
    for index, slot in enumerate((9, 14, 21)):
      member = replace(obj.members[0], raw_track_id=101 + index,
                       detection=replace(obj.members[0].detection, slot=slot))
      changed = replace(obj, members=(member,), representative_raw_track_id=member.raw_track_id)
      alias = allocator.update(index * 100_000_000, [changed.physical_track_id], {self.PID})
      point = structs.RadarData.RadarPoint()
      bosch_fill_point(point, changed, 10., alias)
      aliases.append(point.trackId)
      assert changed.physical_track_id == self.PID
    assert aliases == [32, 32, 32]

  def test_short_publication_gap_preserves_alias(self):
    allocator = BoschPublicationAliasAllocator()
    first = allocator.update(0, [self.PID], {self.PID})
    assert allocator.update(200_000_000, [], {self.PID}) == {}
    assert allocator.current_usage == 1
    assert allocator.update(360_000_000, [self.PID], {self.PID}) == first

  def test_grace_boundary_and_expiry(self):
    allocator = BoschPublicationAliasAllocator()
    allocator.update(0, [self.PID], {self.PID})
    allocator.update(500_000_000, [], {self.PID})
    assert allocator.current_usage == 1
    allocator.update(600_000_000, [], {self.PID})
    assert allocator.current_usage == 0
    assert allocator.alias_to_physical == allocator.last_published_ns == {}
    assert allocator.update(610_000_000, [self.PID], {self.PID})[self.PID] == 33

  def test_return_after_grace_without_intermediate_update(self):
    allocator = BoschPublicationAliasAllocator()
    allocator.update(0, [self.PID], {self.PID})
    assert allocator.update(600_000_000, [self.PID], {self.PID})[self.PID] == 33

  def test_dead_physical_state_releases_immediately(self):
    allocator = BoschPublicationAliasAllocator()
    allocator.update(0, [self.PID], {self.PID})
    allocator.update(1, [], {})
    assert allocator.current_usage == 0
    assert allocator.alias_to_physical == allocator.last_published_ns == {}
    assert allocator.free_aliases[-1] == 32

  def test_fifo_reuses_release_order(self):
    allocator = BoschPublicationAliasAllocator()
    ids = set(range(self.PID, self.PID + 64))
    first = allocator.update(0, ids, ids)
    # Release alias 95 first, then 32: reuse must follow release time, not ID.
    ids.remove(self.PID + 63)
    allocator.update(1, ids, ids)
    ids.remove(self.PID)
    allocator.update(2, ids, ids)
    ids.update((2_000_000, 2_000_001))
    alias = allocator.update(3, ids, ids)
    assert alias[2_000_000] == first[self.PID + 63] == 95
    assert alias[2_000_001] == first[self.PID] == 32

  def test_full_pool_is_unique_in_range_without_denial(self):
    allocator = BoschPublicationAliasAllocator()
    ids = set(range(self.PID, self.PID + 64))
    alias = allocator.update(0, ids, ids)
    assert set(alias.values()) == set(range(32, 96))
    assert allocator.current_usage == allocator.peak_usage == 64
    assert allocator.denial_count == 0
    assert allocator.alias_to_physical == {a: pid for pid, a in alias.items()}
    for pid in ids:
      point = structs.RadarData.RadarPoint()
      bosch_fill_point(point, replace(self.objects()[0], physical_track_id=pid), 10., alias)
      assert 32 <= point.trackId <= 95
      assert str(point.radarSource) == 'frontRadar'

  def test_pool_pressure_reclaims_only_unpublished_grace_binding(self):
    allocator = BoschPublicationAliasAllocator()
    ids = set(range(self.PID, self.PID + 64))
    first = allocator.update(0, ids, ids)
    active = ids - {self.PID}
    allocator.update(1, active, ids)
    active.add(2_000_000)
    alias = allocator.update(2, active, ids | active)
    assert len(alias) == 64
    assert alias[2_000_000] == first[self.PID]
    assert all(alias[pid] == first[pid] for pid in active if pid in first)
    assert allocator.denial_count == 0
    assert allocator.grace_eviction_count == 1

  def test_impossible_oversized_scan_is_not_silently_truncated(self):
    allocator = BoschPublicationAliasAllocator()
    first = allocator.update(0, [self.PID], {self.PID})
    ids = set(range(self.PID, self.PID + 65))
    with pytest.raises(ValueError, match='alias pool'):
      allocator.update(1, ids, ids)
    assert allocator.denial_count == 1
    assert allocator.physical_to_alias == first

  def test_optional_alias_preserves_native_fields_and_scc(self):
    objects = self.objects()
    physical_id = objects[0].physical_track_id
    native = bosch_to_native_radar_data(objects, 1_000_000_000, v_ego=10.)
    assert native.points[0].trackId == physical_id
    before, after = structs.RadarData.new_message(), structs.RadarData.new_message()
    scc = dict(trackId=0, dRel=80., vRel=-.5, radarSource='scc', measured=True)
    before.points, after.points = [scc], [scc]
    bosch_append_points(before, objects, 10., 1_013_219_371, alias=None)
    bosch_append_points(after, objects, 10., 1_013_219_371, alias={physical_id: 37})
    assert after.points[1].trackId == 37
    assert before.points[1].trackId == physical_id
    after.points[1].trackId = physical_id
    assert before.to_bytes() == after.to_bytes()

  def test_offpath_publication_and_stale_cleanup_preserve_scc(self, monkeypatch):
    provider = BoschRadarProvider(1)
    interface = RadarInterface.__new__(RadarInterface)
    interface.bosch, interface.v_ego = provider, 0.
    def base_update(*args):
      data = structs.RadarData.new_message()
      data.points = [dict(trackId=0, dRel=80., vRel=0., radarSource='scc', measured=True)]
      return data
    monkeypatch.setattr(radar_interface_module.RadarInterfaceBase, 'update_carrot', base_update)
    for index, lateral in enumerate((-4.5, 1.5)):
      now = 1_000_000_000 + index * 100_000_000
      objects = provider.tracker.update(now, [BoschRawDetection(now, 9, 30., lateral, 0., raw_word=1)])
      interface._bosch_objects = provider.qualifier.update(objects, now, 0.)
      provider.last_scan_timestamp_ns = interface._bosch_now_ns = now
      data = interface.update_carrot(0., 0., now * 1e-9, [])
      assert data.points[0].to_dict() == base_update().points[0].to_dict()
      assert len(data.points) == 2
      alias = 32 + index
      assert data.points[1].trackId == alias
      assert provider.publication_aliases.alias_to_physical[alias] == objects[0].physical_track_id
    # Empty/stale publications still age bindings, including a stalled provider
    # whose group state has not been advanced by another CAN scan.
    interface._bosch_now_ns += 600_000_000
    data = interface.update_carrot(0., 0., interface._bosch_now_ns * 1e-9, [])
    assert data.to_bytes() == base_update().to_bytes()
    assert provider.publication_aliases.current_usage == 0

  @pytest.mark.parametrize(('mode', 'missing_scc'), ((1, False), (2, False), (3, False), (1, True)))
  def test_production_boundary_uses_alias_and_preserves_tracking(self, monkeypatch, mode, missing_scc):
    class FakeParams:
      def get_int(self, key):
        return mode if key == 'EnableRadarTracks' else 0

    monkeypatch.setattr(radar_interface_module, 'Params', FakeParams)
    monkeypatch.setattr(hyundaicanfd, 'Params', FakeParams)
    cp = structs.CarParams()
    cp.carFingerprint = CAR.HYUNDAI_GRANDEUR_IG
    cp.extFlags = int(HyundaiExtFlags.BOSCH_RADAR | HyundaiExtFlags.BOSCH_RADAR_BUS1)
    cp.radarTimeStep = .05
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
    interface = RadarInterface(cp)
    assert interface.rcp_tracks is None
    if missing_scc and mode == 1:
      interface.rcp_scc = None
    publications = []
    inactive = radar_interface_module.BOSCH_INACTIVE_WORD.to_bytes(4, 'little')
    active = (120 | (1024 << 10) | (512 << 21)).to_bytes(4, 'little')
    for frame in range(30):
      now = 1_000_000_000 + frame * 10_000_000
      packets = []
      if frame in (0, 10, 20):
        slot = (9, 14, 21)[frame // 10]
        messages = []
        for address in range(0x602, 0x612):
          words = [active if (address - 0x602) * 2 + half == slot else inactive for half in (0, 1)]
          messages.append((address, b''.join(words), interface.bosch.bus))
        messages.append((0x612, bytes([0]) + frame.to_bytes(3, 'little') + bytes(4), interface.bosch.bus))
        packets = [(now, messages)]
      interface.set_bosch_context(now)
      result = interface.update_carrot(10., 0., now * 1e-9, packets)
      if result is not None:
        front = [p for p in result.points if str(p.radarSource) == 'frontRadar']
        assert len(front) == 1
        point = front[0]
        assert point.trackId == 32 and point.measured
        assert (point.dRel, point.yRel, point.vRel, point.vLead) == (30., 0., 0., 10.)
        assert point.trackState == 0
        if mode == 1:
          assert not result.errors.canError
        physical_id = interface.bosch._debug_objects[0].physical_track_id
        assert physical_id == 1_000_000
        assert interface.bosch.publication_aliases.alias_to_physical[point.trackId] == physical_id
        assert interface.bosch.slot_to_ids[slot][1] == physical_id
        publications.append(frame)
    assert publications == [9, 14, 19, 24, 29]
    assert interface.bosch.publication_aliases.denial_count == 0


class TestBoschLeadAccelerationEstimator:
  PID = 1_000_700

  @classmethod
  def obj(cls, ns, v_rel, *, pid=None, raw_id=70):
    pid = cls.PID if pid is None else pid
    detection = BoschRawDetection(ns, raw_id % 32, 40., 0., float(v_rel), raw_word=raw_id)
    track = BoschRawTrack(raw_id, detection, 1, False)
    return BoschPhysicalObject(pid, ns, (track,), raw_id, 40., 0., float(v_rel),
                               False, False, 1, 'single_return')

  @classmethod
  def run(cls, velocities, *, estimator=None, pid=None, start_ns=1_000_000_000):
    pid = cls.PID if pid is None else pid
    estimator = BoschLeadAccelerationEstimator() if estimator is None else estimator
    values = []
    for index, velocity in enumerate(velocities):
      ns = start_ns + index * 100_000_000
      values.append(estimator.update((cls.obj(ns, velocity, pid=pid),), ns, 20., {pid})[pid])
    return estimator, values

  def test_constant_speed_braking_and_reacceleration(self):
    _, steady = self.run([0.] * 20)
    assert steady == [0.] * 20
    estimator, braking = self.run([0.] * 6 + [-.25 * i for i in range(1, 9)])
    assert min(braking) < -.2
    _, release = self.run([-2. + .25 * i for i in range(1, 13)], estimator=estimator,
                          start_ns=1_000_000_000 + len(braking) * 100_000_000)
    assert max(release) > 0.

  def test_held_publications_do_not_update(self):
    estimator = BoschLeadAccelerationEstimator()
    first = self.obj(1_000_000_000, 0.)
    estimator.update((first,), first.timestamp_ns, 20., {self.PID})
    estimator.update((first,), first.timestamp_ns, 21., {self.PID})
    assert estimator.update_count == 1
    assert estimator.last_publication_kind == 'HELD'
    estimator.update((self.obj(1_100_000_000, -.25),), 1_100_000_000, 20., {self.PID})
    assert estimator.update_count == 2
    assert estimator.last_publication_kind == 'FRESH'

  def test_same_pid_representative_switch_and_new_pid_independence(self):
    estimator, _ = self.run([0.] * 6)
    state = estimator.states[self.PID]
    estimator.update((self.obj(1_600_000_000, -.25, raw_id=71),), 1_600_000_000, 20., {self.PID})
    assert estimator.states[self.PID] is state
    new_pid = self.PID + 1
    result = estimator.update((self.obj(3_000_000_000, -2., pid=new_pid, raw_id=72),),
                              3_000_000_000, 20., {new_pid})
    assert result[new_pid] == 0.
    assert self.PID not in estimator.states

  def test_expiry_scan_gap_and_explicit_timeout_reset(self):
    estimator, _ = self.run([0.] * 6)
    scan_ns = estimator.last_scan_ns
    estimator.update((), scan_ns + 100_000_000, 20., set())
    assert not estimator.states
    assert estimator.last_expiry_reason == 'PID_DEATH'
    estimator, _ = self.run([0.] * 6)
    timeout = estimator.last_scan_ns + radar_interface_module.BOSCH_STALE_NS
    result = estimator.update((self.obj(timeout, -2.),), timeout, 20., {self.PID})
    assert result[self.PID] == 0.
    assert estimator.last_reset_reason == 'SCAN_GAP'
    estimator.reset('PROVIDER_TIMEOUT')
    assert not estimator.states
    assert estimator.last_reset_reason == 'PROVIDER_TIMEOUT'

  def test_prefix_and_exact_mytrack_parity(self):
    velocities = [0.] * 6 + [-.25 * i for i in range(1, 9)] + [-2. + .25 * i for i in range(1, 9)]
    _, full = self.run(velocities)
    for cutoff in (1, 6, 10, len(velocities)):
      _, prefix = self.run(velocities[:cutoff])
      assert prefix == full[:cutoff]
    expected, track = [], None
    for velocity in velocities:
      point = SimpleNamespace(radarSource='frontRadar', dRel=40., yRel=0., vRel=velocity,
                              yvRel=0., vLead=20. + velocity, measured=True)
      track = MyTrack(self.PID, point, BOSCH_ALEAD_SAMPLE_PERIOD_S) if track is None else track
      track.update(point, 0.)
      expected.append(float(track.aLead) if track.cnt >= 6 else 0.)
    assert full == expected

  def test_only_alead_becomes_finite(self):
    estimator, _ = self.run([0.] * 6 + [-.25] * 4)
    target = self.obj(estimator.last_scan_ns, -.25)
    values = estimator.update((target,), estimator.last_scan_ns, 20., {self.PID})
    data = structs.RadarData.new_message()
    bosch_append_points(data, (target,), 20., target.timestamp_ns, a_lead_by_pid=values)
    point = data.points[0]
    assert math.isfinite(point.aLead)
    assert math.isnan(point.aRel)
    assert math.isnan(point.yvRel)
    assert math.isnan(point.jLead)

  def test_current_fresh_observation_is_written_to_same_frame(self):
    estimator, values = self.run([0.] * 6)
    previous = values[-1]
    ns = estimator.last_scan_ns + 100_000_000
    target = self.obj(ns, -2.)
    current = estimator.update((target,), ns, 20., {self.PID})
    assert current[self.PID] == estimator.states[self.PID].a_lead
    assert current[self.PID] != previous
    data = structs.RadarData.new_message()
    bosch_append_points(data, (target,), 20., ns, a_lead_by_pid=current)
    assert data.points[0].aLead == pytest.approx(current[self.PID], abs=1e-6)

  def test_nonfinite_input_fails_unknown_and_restarts_state(self):
    estimator, _ = self.run([0.] * 6)
    ns = estimator.last_scan_ns + 100_000_000
    result = estimator.update((self.obj(ns, 0.),), ns, math.nan, {self.PID})
    assert math.isnan(result[self.PID])
    assert self.PID not in estimator.states
    assert estimator.invalid_input_count == 1
    ns += 100_000_000
    recovered = estimator.update((self.obj(ns, 0.),), ns, 20., {self.PID})
    assert recovered[self.PID] == 0.
    assert estimator.states[self.PID].update_count == 1

  def test_invalid_scan_timestamp_fails_unknown(self):
    estimator, _ = self.run([0.] * 6)
    assert estimator.update((self.obj(estimator.last_scan_ns, 0.),), math.nan, 20., {self.PID}) == {}
    assert not estimator.states
    assert estimator.last_reset_reason == 'INVALID_SCAN_TIMESTAMP'


class TestBoschB5BirthDefer:
  STEP_NS = 100_000_000
  START_NS = 1_000_000_000
  PARENT_PID = 1_000_100
  TARGET_PID = 1_000_101

  @staticmethod
  def track(raw_id, slot, timestamp_ns, age, *, d_rel=40., y_rel=0., vcode=512):
    word = (vcode << 21) | 1
    detection = BoschRawDetection(timestamp_ns, slot, d_rel, y_rel, vcode * .25 - 128., raw_word=word)
    return BoschRawTrack(raw_id, detection, age)

  @staticmethod
  def obj(pid, timestamp_ns, members, age, *, representative=None):
    representative = members[0].raw_track_id if representative is None else representative
    surface = next(member for member in members if member.raw_track_id == representative)
    return BoschPhysicalObject(
      pid, timestamp_ns, tuple(members), representative,
      surface.d_rel, surface.y_rel, surface.v_rel, False, False, age, 'single_return')

  @staticmethod
  def model(source_ns, publication_ns, *, edge_std=.1, lane_shift=0.):
    xs = tuple(float(value) for value in range(0, 201, 10))

    def line(radar_y):
      return tuple((x, -(radar_y + lane_shift)) for x in xs)

    return BoschB5ModelContext(
      publication_ns, source_ns,
      (line(3.), line(-3.)), (edge_std, edge_std),
      (line(2.), line(.7), line(-.7), line(-2.)),
      (1., 1., 1., 1.), (.1, .1, .1, .1), line(0.))

  @classmethod
  def prime(cls, scans=249, *, parent_raws=(10,), representative=10):
    b5 = BoschBirthB5Defer(BOSCH_B5_ACTIVE)
    for index in range(1, scans + 1):
      ns = cls.START_NS + (index - 1) * cls.STEP_NS
      members = tuple(cls.track(raw_id, raw_id, ns, index) for raw_id in parent_raws)
      parent = cls.obj(cls.PARENT_PID, ns, members, index, representative=representative)
      b5.update((parent,), members, ns, ns + 5_000_000, 10., 0.,
                live_pids=(cls.PARENT_PID,), live_raw_ids=parent_raws)
    return b5

  @classmethod
  def add_context(cls, b5, scan_ns, *, edge_std=.1):
    for offset in (120_000_000, 80_000_000, 40_000_000, 0):
      b5.ingest_speed(scan_ns - offset, 0.)
      b5.ingest_pose(scan_ns - offset, 0.)
    b5.model_contexts = [
      cls.model(scan_ns - 110_000_000, scan_ns - 100_000_000, edge_std=edge_std),
      cls.model(scan_ns - 70_000_000, scan_ns - 60_000_000, edge_std=edge_std),
      cls.model(scan_ns - 30_000_000, scan_ns - 20_000_000, edge_std=edge_std),
    ]

  @classmethod
  def birth(cls, b5, scan_index=250, *, parent_raws=(10,), parent_rep=10,
            target_raws=(20,), target_rep=20, parent_vcode=512, target_vcode=512,
            v_ego=10., yaw=0., edge_std=.1, context=True, extra_objects=(),
            extra_raw=(), camera=(), word0=()):
    ns = cls.START_NS + (scan_index - 1) * cls.STEP_NS
    if context:
      cls.add_context(b5, ns, edge_std=edge_std)
    parent_members = tuple(cls.track(raw_id, raw_id, ns, scan_index, d_rel=40., y_rel=0., vcode=parent_vcode)
                           for raw_id in parent_raws)
    target_members = tuple(cls.track(raw_id, raw_id, ns, 1, d_rel=45., y_rel=4.5, vcode=target_vcode)
                           for raw_id in target_raws)
    parent = cls.obj(cls.PARENT_PID, ns, parent_members, scan_index, representative=parent_rep)
    target = cls.obj(cls.TARGET_PID, ns, target_members, 1, representative=target_rep)
    objects = (parent, target) + tuple(extra_objects)
    raw = parent_members + target_members + tuple(extra_raw)
    live_pids = tuple(obj.physical_track_id for obj in objects)
    live_raw = tuple(member.raw_track_id for obj in objects for member in obj.members)
    live_raw += tuple(track.raw_track_id for track in extra_raw)
    b5.update(objects, raw, ns, ns + 5_000_000, v_ego, yaw,
              camera_associations=camera, oem_state=BOSCH_OEM_STATE_NONE,
              word0_pids=word0, live_pids=live_pids, live_raw_ids=live_raw)
    return parent, target, b5.last_decisions[0]

  def test_exact_b5_defers_birth_once_and_restores_next_scan(self):
    b5 = self.prime()
    parent, target, decision = self.birth(b5)
    assert decision.decision and decision.parent_stable_scans == BOSCH_B5_PARENT_STABILITY_SCANS
    assert b5.publication_view((parent, target)) == (parent,)
    # RadarInterface may build more than one consumer view of the same scan;
    # the action and accounting remain one birth publication tick.
    assert b5.publication_view((parent, target)) == (parent,)
    assert b5.suppressed_scans == 1 and b5.suppressed_points == 1
    assert self.TARGET_PID in b5.states

    ns = target.timestamp_ns + self.STEP_NS
    parent_track = self.track(10, 10, ns, 251, d_rel=40., y_rel=0.)
    target_track = self.track(20, 20, ns, 2, d_rel=45., y_rel=4.5)
    parent2 = self.obj(self.PARENT_PID, ns, (parent_track,), 251)
    target2 = self.obj(self.TARGET_PID, ns, (target_track,), 2)
    b5.update((parent2, target2), (parent_track, target_track), ns, ns + 5_000_000, 10., 0.,
              live_pids=(self.PARENT_PID, self.TARGET_PID), live_raw_ids=(10, 20))
    assert b5.would_suppress == frozenset()
    assert b5.publication_view((parent2, target2)) == (parent2, target2)

  def test_radar_interface_ingests_bosch_pose_and_frozen_model_context(self):
    interface = RadarInterface.__new__(RadarInterface)
    interface.bosch = BoschRadarProvider(1, b5_mode=BOSCH_B5_ACTIVE)
    interface._bosch_path_ns = None
    interface._bosch_path = ()
    interface._bosch_path_source_ns = 0
    interface._bosch_context = None
    now_ns = self.START_NS

    def line(y):
      return SimpleNamespace(x=(0., 10., 20.), y=(y, y, y))

    pose = SimpleNamespace(
      inputsOK=True, sensorsOK=True,
      angularVelocityDevice=SimpleNamespace(valid=True, z=.2))
    model = SimpleNamespace(
      timestampEof=now_ns - 30_000_000, roadEdges=(line(-3.), line(3.)),
      roadEdgeStds=(.1, .1), laneLines=(line(-2.), line(-.7), line(.7), line(2.)),
      laneLineProbs=(1., 1., 1., 1.), laneLineStds=(.1, .1, .1, .1),
      position=line(0.), leadsV3=())
    interface.set_bosch_context(
      now_ns, pose, now_ns - 10_000_000, model, now_ns - 5_000_000)

    b5 = interface.bosch.b5_birth_defer
    assert b5.pose_samples[-1] == (now_ns - 10_000_000, -.2, True)
    assert b5.model_contexts[-1].source_ns == model.timestampEof
    assert interface._bosch_context[0] == now_ns

    pose_sample = b5.pose_samples[-1]
    model_context = b5.model_contexts[-1]
    interface.set_bosch_context(
      now_ns + 1_000_000, pose, now_ns - 10_000_000, model, now_ns - 5_000_000)
    assert b5.pose_samples[-1] is pose_sample
    assert b5.model_contexts[-1] is model_context

  def test_model_context_drops_only_non_finite_points(self):
    b5 = BoschBirthB5Defer(BOSCH_B5_ACTIVE)

    def line(y, x=(0., 10., 20.)):
      return SimpleNamespace(x=x, y=(y, y, y))

    model = SimpleNamespace(
      timestampEof=self.START_NS - 30_000_000, roadEdges=(line(-3.), line(3.)),
      roadEdgeStds=(.1, .1), laneLines=(line(-2.), line(-.7, (0., math.nan, 20.)), line(.7), line(2.)),
      laneLineProbs=(1., 1., 1., 1.), laneLineStds=(.1, .1, .1, .1), position=line(0.))
    b5.ingest_model(model, self.START_NS)
    context = b5.model_contexts[-1]
    assert context.lane_lines[0] == ((0., -2.), (10., -2.), (20., -2.))
    assert context.lane_lines[1] == ((0., -.7), (20., -.7))

  def test_249_is_fail_open_and_250_is_eligible(self):
    _, _, before = self.birth(self.prime(248), scan_index=249)
    _, _, at = self.birth(self.prime(249), scan_index=250)
    assert before.parent_stable_scans == 249 and not before.decision
    assert at.parent_stable_scans == 250 and at.decision

  def test_member_or_representative_change_resets_counter(self):
    _, _, member = self.birth(self.prime(), parent_raws=(10, 11), parent_rep=10)
    _, _, representative = self.birth(
      self.prime(parent_raws=(10, 11), representative=10),
      parent_raws=(10, 11), parent_rep=11)
    assert member.parent_stable_scans == 1 and not member.decision
    assert representative.parent_stable_scans == 1 and not representative.decision

  def test_pid_death_rebirth_and_gap_do_not_inherit_stability(self):
    b5 = self.prime()
    death_ns = self.START_NS + 249 * self.STEP_NS
    b5.update((), (), death_ns, death_ns, 10., 0., live_pids=(), live_raw_ids=())
    assert self.PARENT_PID not in b5.states
    raw = self.track(30, 30, death_ns + self.STEP_NS, 1)
    parent = self.obj(self.PARENT_PID, death_ns + self.STEP_NS, (raw,), 1)
    b5.update((parent,), (raw,), death_ns + self.STEP_NS, death_ns + self.STEP_NS, 10., 0.,
              live_pids=(self.PARENT_PID,), live_raw_ids=(30,))
    assert b5.states[self.PARENT_PID].stable_scans == 1

    b5 = self.prime()
    gap_ns = b5.last_scan_ns + 300_000_000
    raw = self.track(10, 10, gap_ns, 250)
    parent = self.obj(self.PARENT_PID, gap_ns, (raw,), 250)
    b5.update((parent,), (raw,), gap_ns, gap_ns, 10., 0.,
              live_pids=(self.PARENT_PID,), live_raw_ids=(10,))
    assert b5.last_reset_reason == 'RADAR_INPUT_GAP'
    assert b5.states[self.PARENT_PID].stable_scans == 1

  def test_exact_vrel_is_required_and_one_lsb_fails_open(self):
    _, _, exact = self.birth(self.prime(), parent_vcode=512, target_vcode=512)
    _, _, one_lsb = self.birth(self.prime(), parent_vcode=512, target_vcode=513)
    assert exact.decision
    assert not one_lsb.decision and one_lsb.reason == 'VREL_NOT_EXACT_FAIL_OPEN'

  def test_birth_counts_and_singleton_are_exact(self):
    ns = self.START_NS + 249 * self.STEP_NS
    extra_track = self.track(30, 30, ns, 1, d_rel=150., y_rel=0.)
    extra_obj = self.obj(1_000_200, ns, (extra_track,), 100)
    _, _, raw_two = self.birth(self.prime(), extra_objects=(extra_obj,), extra_raw=(extra_track,))
    assert not raw_two.decision and raw_two.reason == 'NEW_RAW_COUNT_FAIL_OPEN'

    second_track = self.track(30, 30, ns, 1, d_rel=150., y_rel=0.)
    second_birth = self.obj(1_000_200, ns, (second_track,), 1)
    _, _, pid_two = self.birth(self.prime(), extra_objects=(second_birth,), extra_raw=(second_track,))
    assert pid_two.new_pid_count == 2 and not pid_two.decision

    _, _, grouped = self.birth(self.prime(), target_raws=(20, 21), target_rep=20)
    assert not grouped.singleton and not grouped.decision and grouped.reason == 'NON_SINGLETON_FAIL_OPEN'

  @pytest.mark.parametrize(('v_ego', 'yaw', 'context', 'edge_std', 'reason'), (
    (2.9, 0., True, .1, 'LOW_SPEED_FAIL_OPEN'),
    (10., .21, True, .1, 'HIGH_CURVATURE_FAIL_OPEN'),
    (10., 0., False, .1, 'MODEL_STALE_FAIL_OPEN'),
    (10., 0., True, .6, 'ROAD_EDGE_UNSTABLE_FAIL_OPEN'),
  ))
  def test_context_fail_open_conditions(self, v_ego, yaw, context, edge_std, reason):
    _, _, decision = self.birth(self.prime(), v_ego=v_ego, yaw=yaw, context=context, edge_std=edge_std)
    assert not decision.decision and decision.reason == reason

  def test_multiple_births_share_geometry_without_sharing_decisions(self, monkeypatch):
    b5 = self.prime()
    ns = self.START_NS + 249 * self.STEP_NS
    extra_track = self.track(30, 30, ns, 1, d_rel=46., y_rel=-4.5)
    extra = self.obj(1_000_200, ns, (extra_track,), 1)
    geometries, alignments = [], []
    geometry, align = b5._geometry, b5._align

    def counted_geometry(context, scan_ns):
      geometries.append(id(context))
      return geometry(context, scan_ns)

    def counted_align(obj, shape):
      alignments.append(obj.physical_track_id)
      return align(obj, shape)

    monkeypatch.setattr(b5, '_geometry', counted_geometry)
    monkeypatch.setattr(b5, '_align', counted_align)
    self.birth(b5, extra_objects=(extra,), extra_raw=(extra_track,))
    assert len(geometries) == len(set(geometries)) == 3
    assert sorted(alignments) == [self.PARENT_PID, self.TARGET_PID, 1_000_200]
    assert len(b5.last_decisions) == 2
    assert all(d.n4_eligible and not d.decision and d.reason == 'NEW_RAW_COUNT_FAIL_OPEN'
               for d in b5.last_decisions)

  def test_geometry_reuse_does_not_survive_context_replacement(self):
    b5 = self.prime()
    _, _, first = self.birth(b5)
    assert first.decision
    _, _, next_scan = self.birth(b5, scan_index=251, edge_std=.6)
    assert not next_scan.decision and next_scan.reason == 'ROAD_EDGE_UNSTABLE_FAIL_OPEN'

  def test_direct_parent_query_reads_replaced_same_timestamp_context(self):
    b5 = self.prime()
    prior = {rid: frozenset(pids) for rid, pids in b5.raw_prior_owners.items()}
    parent, target, _ = self.birth(b5)
    ns = target.timestamp_ns
    args = (target, (parent, target), ns, ns + 5_000_000, 10., 0., {}, BOSCH_OEM_STATE_NONE, (), prior)
    assert b5._n4_parent(*args)[0] is parent
    self.add_context(b5, ns, edge_std=.6)
    assert b5._n4_parent(*args) == (None, 'ROAD_EDGE_UNSTABLE_FAIL_OPEN')

  @pytest.mark.parametrize(('x', 'expected'), ((0., 10.), (.5, 15.), (1., 20.), (1.5, 35.), (2., 40.)))
  def test_geometry_interpolation_preserves_duplicate_and_endpoint_values(self, x, expected):
    points = ((0., 10.), (1., 20.), (1., 30.), (2., 40.))
    assert BoschBirthB5Defer._interp(points, x) == expected
    assert math.isnan(BoschBirthB5Defer._interp(points, -.01))
    assert math.isnan(BoschBirthB5Defer._interp(points, 2.01))

  @pytest.mark.parametrize(('query', 'expected'), ((-1, 1.), (0, 1.), (5, 2.), (10, 3.), (15, 6.), (20, 7.), (21, 7.)))
  def test_motion_interpolation_prepared_times_preserve_boundaries(self, query, expected):
    samples = [(0, 1.), (10, 3.), (10, 5.), (20, 7.)]
    assert BoschBirthB5Defer._interpolate(samples, query) == expected
    assert BoschBirthB5Defer._interpolate(samples, query, times=[0, 10, 10, 20]) == expected

  def test_motion_reuses_times_only_within_call_and_keeps_pose_validity(self):
    b5 = BoschBirthB5Defer()
    start, end = 1_000_000_000, 1_030_000_000
    b5.speed_samples = [(start, 10.), (end, 10.)]
    b5.pose_samples = [(start, 0., True), (end, 0., True)]
    x, y, heading = b5._integrate_motion(start, end)
    assert x == pytest.approx(.3) and y == 0. and heading == 0.
    b5.speed_samples = [(start, 20.), (end, 20.)]
    assert b5._integrate_motion(start, end)[0] == pytest.approx(.6)
    b5.pose_samples[0] = (start, 0., False)
    assert b5._integrate_motion(start, end) is None
    b5.pose_samples = [(start, 0., True)]
    assert b5._integrate_motion(start, start) == (0., 0., 0.)
    assert b5._integrate_motion(start, start-1) is None

  def test_independent_camera_and_ambiguous_parent_fail_open(self):
    camera = {
      self.TARGET_PID: (BOSCH_CAMERA_ASSOC_ASSIGNED, 7, 1),
      self.PARENT_PID: (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, 0),
    }
    _, _, independent = self.birth(self.prime(), camera=camera)
    assert not independent.decision and independent.reason == 'INDEPENDENT_CAMERA_FAIL_OPEN'

    ns = self.START_NS + 249 * self.STEP_NS
    other_track = self.track(30, 30, ns, 300, d_rel=42., y_rel=1.)
    other = self.obj(1_000_200, ns, (other_track,), 300)
    _, _, ambiguous = self.birth(self.prime(), extra_objects=(other,), extra_raw=(other_track,))
    assert not ambiguous.decision and ambiguous.reason == 'AMBIGUOUS_PARENT_FAIL_OPEN'

  def test_alias_is_allocated_before_defer_and_reused_on_release(self):
    b5 = self.prime()
    parent, target, decision = self.birth(b5)
    assert decision.decision
    allocator = BoschPublicationAliasAllocator()
    first = allocator.update(target.timestamp_ns, (self.PARENT_PID, self.TARGET_PID),
                             (self.PARENT_PID, self.TARGET_PID))
    assert self.TARGET_PID not in {obj.physical_track_id for obj in b5.publication_view((parent, target))}
    target_alias = first[self.TARGET_PID]

    ns = target.timestamp_ns + self.STEP_NS
    ptrack = self.track(10, 10, ns, 251, d_rel=40., y_rel=0.)
    ttrack = self.track(20, 20, ns, 2, d_rel=45., y_rel=4.5)
    parent2 = self.obj(self.PARENT_PID, ns, (ptrack,), 251)
    target2 = self.obj(self.TARGET_PID, ns, (ttrack,), 2)
    b5.update((parent2, target2), (ptrack, ttrack), ns, ns, 10., 0.,
              live_pids=(self.PARENT_PID, self.TARGET_PID), live_raw_ids=(10, 20))
    second = allocator.update(ns, (self.PARENT_PID, self.TARGET_PID),
                              (self.PARENT_PID, self.TARGET_PID))
    assert second[self.TARGET_PID] == target_alias
    assert b5.publication_view((parent2, target2)) == (parent2, target2)

  def test_negative_view_is_tuple_identical_and_state_is_bounded(self):
    b5 = self.prime()
    parent, target, decision = self.birth(b5, context=False)
    baseline = (parent, target)
    assert not decision.decision
    assert b5.publication_view(baseline) is baseline
    assert len(b5.states) <= 2
    assert len(b5.raw_prior_owners) <= 2


class TestBoschPhysicalIdContinuityTrace:
  """The physical layer carries an ID on raw-member overlap alone.

  These cover the shadow trace that records how far the inheriting state is
  from the state that ID last published. The trace is diagnostic only: it must
  never change an assignment, and it must not silently start rejecting one.
  """
  SCAN_NS = 100_000_000

  @staticmethod
  def track(raw_track_id, slot, d_rel, y_rel, v_rel, timestamp_ns, age_scans, recovered=False):
    detection = BoschRawDetection(timestamp_ns, slot, d_rel, y_rel, v_rel, raw_word=1)
    return BoschRawTrack(raw_track_id, detection, age_scans, recovered)

  @classmethod
  def coasted_carry_scans(cls):
    """Two returns group, one drops out, then the other reappears far away.

    This is the recorded route-252 shape: the physical ID follows a member kept
    only for coasting, to a position the previous object never occupied.
    """
    scans = []
    for index in range(3):
      ns = (index + 1) * cls.SCAN_NS
      scans.append((ns, (cls.track(55, 8, 11.50, -0.625, 0.50, ns, index + 1),
                         cls.track(56, 20, 9.25, -1.71875, 0.25, ns, index + 1))))
    ns = 4 * cls.SCAN_NS
    scans.append((ns, (cls.track(56, 20, 9.25, -1.84375, 0.25, ns, 4),)))
    ns = 5 * cls.SCAN_NS
    scans.append((ns, (cls.track(55, 7, 14.75, 1.78125, 0.75, ns, 4, recovered=True),)))
    return scans

  @classmethod
  def run(cls, scans, trace):
    manager = BoschObjectGroupManager()
    manager.trace_decisions = trace
    outputs, decisions = [], []
    for timestamp_ns, tracks in scans:
      outputs.append(manager.update(timestamp_ns, tracks))
      decisions.append(manager.last_decisions)
    return manager, outputs, decisions

  @staticmethod
  def signature(outputs):
    return [[(obj.physical_track_id, obj.timestamp_ns, obj.d_rel, obj.y_rel, obj.v_rel,
              obj.representative_raw_track_id, obj.age_scans, obj.member_slots,
              tuple(m.raw_track_id for m in obj.members), obj.grouping_evidence)
             for obj in scan] for scan in outputs]

  def test_trace_does_not_change_any_assignment(self):
    scans = self.coasted_carry_scans()
    off_manager, off_outputs, off_decisions = self.run(scans, False)
    on_manager, on_outputs, _ = self.run(scans, True)

    assert self.signature(off_outputs) == self.signature(on_outputs)
    assert dict(off_manager.stats) == dict(on_manager.stats)
    assert off_manager.next_id == on_manager.next_id
    # Off leaves nothing behind to read.
    assert all(scan == () for scan in off_decisions)

  def test_coasted_member_carries_the_id_without_a_continuity_check(self):
    scans = self.coasted_carry_scans()
    _, outputs, decisions = self.run(scans, True)

    carrier = next(obj for obj in outputs[3] if 56 in {m.raw_track_id for m in obj.members})
    inheritor = next(obj for obj in outputs[4] if 55 in {m.raw_track_id for m in obj.members})
    # Same physical ID, although raw 55 was never part of the published object.
    assert inheritor.physical_track_id == carrier.physical_track_id
    assert inheritor.d_rel == 14.75

    decision = next(d for d in decisions[4] if d.physical_track_id == inheritor.physical_track_id)
    assert decision.previous_member_raw_track_ids == (56,)
    assert decision.member_raw_track_ids == (55,)
    assert decision.member_overlap == 1          # scored against the coasting set
    assert decision.observed_member_overlap == 0  # but absent from the last published object
    assert decision.grouping_evidence == "single_return"
    # A single-member cluster skips representative selection, so production never
    # evaluates this residual; only the trace does.
    assert decision.predicted_d_rel == pytest.approx(9.25 + 0.25 * 0.1)
    assert decision.residual_d_m == pytest.approx(14.75 - (9.25 + 0.25 * 0.1))
    assert decision.residual_y_m > 3.0

  def test_trace_records_a_residual_for_every_inherited_id(self):
    scans = self.coasted_carry_scans()
    _, outputs, decisions = self.run(scans, True)

    for scan_outputs, scan_decisions in zip(outputs, decisions):
      assert len(scan_decisions) == len(scan_outputs)
      for decision in scan_decisions:
        if decision.assignment_mode == "created":
          assert decision.previous_physical_track_id is None
          assert decision.residual_d_m is None
        else:
          assert decision.residual_d_m is not None
          assert decision.residual_y_m is not None
          assert decision.residual_v_mps is not None
          assert decision.dt_s > 0.0


class TestBoschProvisionalBirthBundle:
  SCAN_NS = 100_000_000

  @staticmethod
  def track(raw_id, slot, d_rel, y_rel, v_rel, ns, age):
    detection = BoschRawDetection(ns, slot, d_rel, y_rel, v_rel, raw_word=raw_id)
    return BoschRawTrack(raw_id, detection, age, False)

  @classmethod
  def pair(cls, index, *, d=(79.75, 78.25), y=(-1.4375, -1.21875),
           v=(-13., -13.), ages=None):
    ns = (index + 1) * cls.SCAN_NS
    ages = ages or (index + 1, index + 1)
    return ns, (cls.track(499, 17, d[0], y[0], v[0], ns, ages[0]),
                cls.track(500, 18, d[1], y[1], v[1], ns, ages[1]))

  @staticmethod
  def public(manager, objects, **kwargs):
    return manager.provisional_publication_view(objects, **kwargs)

  def test_bosch_provisional_bundle_same_scan_duplicate(self):
    manager = BoschObjectGroupManager()
    ns, tracks = self.pair(0)
    objects = manager.update(ns, tracks, v_ego=30., yaw_rate=.008169143)
    public = self.public(manager, objects)

    assert len(objects) == 2 and len(manager.states) == 2
    assert {m.raw_track_id for obj in objects for m in obj.members} == {499, 500}
    assert len(public) == 1
    decision = manager.last_provisional_decisions[-1]
    assert decision.action == 'CREATE'
    assert decision.raw_track_ids == (499, 500)
    assert decision.delta_d_m == 1.5
    assert decision.delta_y_m == .21875
    assert decision.delta_bearing_deg == pytest.approx(.140335, abs=1e-6)
    assert decision.delta_vrel_mps == 0.
    assert decision.delta_world_speed_mps == pytest.approx(.001787, abs=1e-6)

  def test_bosch_provisional_bundle_second_scan_coherent(self):
    manager = BoschObjectGroupManager()
    first_ns, first = self.pair(0)
    self.public(manager, manager.update(first_ns, first, v_ego=30.))
    second_ns, second = self.pair(1, d=(78.75, 77.75), y=(-1.40625, -1.25))
    objects = manager.update(second_ns, second, v_ego=30.)

    assert len(self.public(manager, objects)) == 1
    assert manager.last_provisional_decisions[-1].action == 'COHERENT'
    assert manager.last_provisional_decisions[-1].representative_raw_track_id == 499
    assert {obj.age_scans for obj in objects} == {2}

  def test_bosch_provisional_bundle_split_fail_open(self):
    manager = BoschObjectGroupManager()
    ns, tracks = self.pair(0)
    self.public(manager, manager.update(ns, tracks, v_ego=30.))
    ns, tracks = self.pair(1, y=(-1.5, -.75))
    objects = manager.update(ns, tracks, v_ego=30.)

    assert self.public(manager, objects) == objects
    assert not manager.provisional_bundles
    assert manager.last_provisional_decisions[-1].release_reason == 'criteria_diverged'

  def test_bosch_provisional_bundle_camera_supported_fail_open(self):
    manager = BoschObjectGroupManager()
    ns, tracks = self.pair(0)
    objects = manager.update(ns, tracks, v_ego=30.)
    pids = tuple(obj.physical_track_id for obj in objects)

    assert self.public(manager, objects, strict_associations={pids[0]: 7}) == objects
    assert not manager.provisional_bundles
    assert manager.last_provisional_decisions[-1].release_reason == 'strict_camera_identity'

  def test_bosch_provisional_bundle_oem_supported_fail_open(self):
    manager = BoschObjectGroupManager()
    ns, tracks = self.pair(0)
    objects = manager.update(ns, tracks, v_ego=30., oem_slot=17)
    assert self.public(manager, objects) == objects
    assert not manager.provisional_bundles

    manager = BoschObjectGroupManager()
    objects = manager.update(ns, tracks, v_ego=30.)
    assert self.public(manager, objects, oem_pids=(objects[1].physical_track_id,)) == objects
    assert manager.last_provisional_decisions[-1].release_reason == 'oem_identity'

  def test_bosch_provisional_bundle_two_real_objects_not_merged(self):
    manager = BoschObjectGroupManager()
    ns, tracks = self.pair(0, y=(-1.5, -1.2))
    objects = manager.update(ns, tracks, v_ego=30.)
    assert self.public(manager, objects) == objects
    assert not manager.provisional_bundles

  def test_bosch_provisional_bundle_normal_group_handoff(self):
    manager = BoschObjectGroupManager()
    for index in range(3):
      ns, tracks = self.pair(index, d=(79.75-index, 78.25-index))
      objects = manager.update(ns, tracks, v_ego=30.)
      public = self.public(manager, objects)
    assert len(objects) == len(public) == 1
    assert tuple(m.raw_track_id for m in objects[0].members) == (499, 500)
    assert objects[0].representative_raw_track_id == 499
    assert objects[0].grouping_evidence == 'temporal_complete_link'
    assert not manager.provisional_bundles
    assert manager.last_provisional_decisions[-1].release_reason == 'normal_group_handoff'

  def test_bosch_provisional_bundle_state_reset(self):
    manager = BoschObjectGroupManager()
    ns, tracks = self.pair(0)
    self.public(manager, manager.update(ns, tracks, v_ego=30.))
    objects = manager.update(2 * self.SCAN_NS, (), v_ego=30.)
    assert objects == () and not manager.provisional_bundles
    assert manager.last_provisional_decisions[-1].release_reason == 'member_missing'

    manager = BoschObjectGroupManager()
    ns, tracks = self.pair(0)
    self.public(manager, manager.update(ns, tracks, v_ego=30.))
    ns, tracks = self.pair(1, ages=(1, 1))
    objects = manager.update(ns, tracks, v_ego=30.)
    assert self.public(manager, objects) == objects
    assert manager.last_provisional_decisions[-1].release_reason == 'age_rollback'

  def test_bosch_s37_not_matched_by_s32_bundle_rule(self):
    manager = BoschObjectGroupManager()
    ns = self.SCAN_NS
    tracks = (
      self.track(129, 4, 59.75, -3.34375, -5., ns, 209),
      self.track(338, 18, 68.5, -1.40625, -5., ns, 1),
    )
    objects = manager.update(ns, tracks, v_ego=29.5)
    assert self.public(manager, objects) == objects
    assert not manager.provisional_bundles

  def test_bosch_provisional_bundle_alias_and_representative_continuity(self):
    manager = BoschObjectGroupManager()
    allocator = BoschPublicationAliasAllocator()
    representative_alias = None
    for index in range(3):
      ns, tracks = self.pair(index, d=(79.75-index, 78.25-index))
      objects = manager.update(ns, tracks, v_ego=30.)
      aliases = allocator.update(ns, (obj.physical_track_id for obj in objects), manager.states)
      public = self.public(manager, objects)
      if index == 0:
        rep_pid = public[0].physical_track_id
        representative_alias = aliases[rep_pid]
        assert manager.last_provisional_decisions[0].representative_raw_track_id == 499
      assert aliases[public[0].physical_track_id] == representative_alias
    assert allocator.denial_count == 0

  def test_bosch_provisional_bundle_can_be_disabled_for_baseline_replay(self):
    manager = BoschObjectGroupManager(BoschGroupingConfig(provisional_enabled=False))
    ns, tracks = self.pair(0)
    objects = manager.update(ns, tracks, v_ego=30.)
    assert self.public(manager, objects) == objects
    assert manager.last_provisional_decisions == ()


class TestBoschFamilyCompanionB1:
  SCAN_NS = 100_000_000
  ANCHOR_PID = 1_000_145
  NEWBORN_PID = 1_000_369
  ANCHOR_RAW = 129
  NEWBORN_RAW = 338

  @staticmethod
  def obj(pid, raw_id, slot, ns, age, d_rel, y_rel, v_rel, *, vision=False, oem=False,
          extra_members=()):
    tracks = [BoschRawTrack(raw_id, BoschRawDetection(ns, slot, d_rel, y_rel, v_rel, raw_word=raw_id), age, False)]
    for extra_raw, extra_slot in extra_members:
      tracks.append(BoschRawTrack(extra_raw, BoschRawDetection(ns, extra_slot, d_rel+.25, y_rel, v_rel,
                                                               raw_word=extra_raw), age, False))
    return BoschPhysicalObject(pid, ns, tuple(tracks), raw_id, d_rel, y_rel, v_rel, oem, vision,
                               age, 'single_return' if len(tracks) == 1 else 'temporal_complete_link')

  @classmethod
  def scan(cls, index, *, anchor=True, newborn=True, newborn_v=-5., newborn_y=-1.40625,
           newborn_d=None, anchor_members=(), newborn_members=()):
    ns = (index + 1) * cls.SCAN_NS
    result = []
    if anchor:
      result.append(cls.obj(cls.ANCHOR_PID, cls.ANCHOR_RAW, 4, ns, 30 + index,
                            59.75-.5*index, -3.34375, -5., extra_members=anchor_members))
    if newborn:
      result.append(cls.obj(cls.NEWBORN_PID, cls.NEWBORN_RAW, 18, ns, 1 + index,
                            (68.5-.5*index) if newborn_d is None else newborn_d,
                            newborn_y, newborn_v, extra_members=newborn_members))
    return ns, tuple(result)

  @staticmethod
  def filter(mode=None):
    mode = radar_interface_module.BOSCH_FAMILY_COMPANION_ACTIVE if mode is None else mode
    return radar_interface_module._BoschFamilyCompanionFilter(mode)

  @classmethod
  def activate(cls, gate=None):
    gate = gate or cls.filter()
    ns, objects = cls.scan(0)
    assert gate.update(objects, ns, 29.5) == frozenset()
    ns, objects = cls.scan(1)
    assert gate.update(objects, ns, 29.5) == frozenset({cls.NEWBORN_PID})
    return gate, ns, objects

  def test_bosch_family_companion_first_scan_not_held(self):
    gate = self.filter()
    ns, objects = self.scan(0)
    assert gate.update(objects, ns, 29.5) == frozenset()
    assert gate.publication_view(objects) == objects
    assert gate._states[self.NEWBORN_PID].active is False

  def test_bosch_family_companion_two_scan_hold(self):
    gate, _, objects = self.activate()
    assert [obj.physical_track_id for obj in gate.publication_view(objects)] == [self.ANCHOR_PID]
    decision = gate.last_decisions[-1]
    assert decision.action == 'HOLD'
    assert decision.delta_d_scan1_m == pytest.approx(8.75)
    assert decision.delta_d_scan2_m == pytest.approx(8.75)
    assert decision.public_suppressed

  def test_bosch_family_companion_strict_camera_release(self):
    gate, _, _ = self.activate()
    ns, objects = self.scan(2)
    assert gate.update(objects, ns, 29.5, strict_associations={self.NEWBORN_PID: 7}) == frozenset()
    assert gate.last_decisions[-1].release_reason == 'STRICT_CAMERA'
    assert gate.publication_view(objects) == objects

  def test_bosch_family_companion_shared_strict_camera_is_not_independent(self):
    gate = self.filter()
    ns, objects = self.scan(0)
    strict = {self.NEWBORN_PID: 7, self.ANCHOR_PID: 7}
    gate.update(objects, ns, 29.5, strict_associations=strict)
    ns, objects = self.scan(1)
    assert gate.update(objects, ns, 29.5, strict_associations=strict) == frozenset({self.NEWBORN_PID})

  def test_bosch_family_companion_oem_release(self):
    gate, _, _ = self.activate()
    ns, objects = self.scan(2)
    assert gate.update(objects, ns, 29.5, oem_pids=(self.NEWBORN_PID,)) == frozenset()
    assert gate.last_decisions[-1].release_reason == 'OEM_IDENTITY'

  def test_bosch_family_companion_speed_divergence_release(self):
    gate, _, _ = self.activate()
    ns, objects = self.scan(2, newborn_v=-4.)
    assert gate.update(objects, ns, 29.5) == frozenset()
    assert gate.last_decisions[-1].release_reason == 'SPEED_DIVERGENCE'

  def test_bosch_family_companion_geometry_divergence_release(self):
    gate, _, _ = self.activate()
    ns, objects = self.scan(2, newborn_y=.25)
    assert gate.update(objects, ns, 29.5) == frozenset()
    assert gate.last_decisions[-1].release_reason == 'GEOMETRY_DIVERGENCE'

  def test_bosch_family_companion_anchor_loss_release(self):
    gate, _, _ = self.activate()
    ns, objects = self.scan(2, anchor=False)
    assert gate.update(objects, ns, 29.5) == frozenset()
    assert gate.last_decisions[-1].release_reason == 'ANCHOR_LOST'

  def test_bosch_family_companion_scan_gap_reset(self):
    gate, _, _ = self.activate()
    ns, objects = self.scan(3)
    assert gate.update(objects, ns, 29.5) == frozenset()
    assert gate.last_decisions[-1].release_reason == 'SCAN_GAP'
    assert gate._states == {}

  def test_bosch_family_companion_internal_history_preserved(self):
    gate, _, objects = self.activate()
    before = tuple((obj.physical_track_id, obj.age_scans,
                    tuple(member.raw_track_id for member in obj.members)) for obj in objects)
    gate.publication_view(objects)
    after = tuple((obj.physical_track_id, obj.age_scans,
                   tuple(member.raw_track_id for member in obj.members)) for obj in objects)
    assert before == after
    assert before[-1] == (self.NEWBORN_PID, 2, (self.NEWBORN_RAW,))

  def test_bosch_family_companion_anchor_public_preserved(self):
    gate, _, objects = self.activate()
    public = gate.publication_view(objects)
    assert len(public) == 1 and public[0] is objects[0]
    assert public[0].physical_track_id == self.ANCHOR_PID

  def test_bosch_family_companion_normal_group_handoff(self):
    gate, _, _ = self.activate()
    ns, objects = self.scan(2, newborn=False, anchor_members=((self.NEWBORN_RAW, 18),))
    assert gate.update(objects, ns, 29.5) == frozenset()
    assert gate.last_decisions[-1].action == 'HANDOFF'
    assert gate.last_decisions[-1].release_reason == 'NORMAL_HANDOFF'

  def test_bosch_family_companion_release_keeps_alias_binding(self):
    gate, ns, objects = self.activate()
    allocator = BoschPublicationAliasAllocator()
    aliases = allocator.update(ns, (obj.physical_track_id for obj in objects), {
      obj.physical_track_id: radar_interface_module._BoschPhysicalState(obj, {
        member.raw_track_id: ns for member in obj.members}) for obj in objects})
    newborn_alias = aliases[self.NEWBORN_PID]
    ns, objects = self.scan(2)
    gate.update(objects, ns, 29.5, strict_associations={self.NEWBORN_PID: 9})
    aliases = allocator.update(ns, (obj.physical_track_id for obj in objects), {
      obj.physical_track_id: radar_interface_module._BoschPhysicalState(obj, {
        member.raw_track_id: ns for member in obj.members}) for obj in objects})
    assert aliases[self.NEWBORN_PID] == newborn_alias

  def test_bosch_family_companion_inward_motion_release(self):
    gate, _, _ = self.activate()
    # Three consecutive inward steps at more than the established 0.90 m/s.
    for index, y in enumerate((-1.30, -1.10, -.90), start=2):
      ns, objects = self.scan(index, newborn_y=y)
      gate.update(objects, ns, 29.5)
    assert gate.last_decisions[-1].release_reason == 'INWARD_MOTION'

  def test_bosch_s32_bundle_unchanged_by_family_filter(self):
    manager = BoschObjectGroupManager()
    family = self.filter()
    helper = TestBoschProvisionalBirthBundle
    actions = []
    for index in range(3):
      ns, tracks = helper.pair(index, d=(79.75-index, 78.25-index))
      objects = manager.update(ns, tracks, v_ego=30.)
      actions.append(manager.last_provisional_decisions[-1].action)
      s32_pids = {pid for pair in manager.provisional_pid_pairs.values() for pid in pair}
      family.update(objects, ns, 30., excluded_pids=s32_pids)
      assert family.would_suppress == frozenset()
    assert actions == ['CREATE', 'COHERENT', 'HANDOFF']


class TestBoschBurstMultiReturnDefer:
  """S33 burst multi-return: three surfaces of one body born in one scan.

  The reference scene is route28b S33, where a truck at 24 m/s world speed
  produced five singleton births across two scans; one of them landed in the
  ego lane and became a false lead.  The geometry reproduced here -- a thin
  range slab, a wide lateral fan, one world speed, a mature neighbour at that
  same speed -- is the one the user confirmed on video for three cargo trucks
  and one distant passenger car.
  """
  SCAN_NS = 100_000_000
  ANCHOR_PID = 1_000_012
  ANCHOR_RAW = 61
  V_EGO = 29.5
  # Route28b S33 geometry: the burst sits in a 3 m slab about 60 m out, fanned
  # from -3.1 m to +10.75 m, all of it at the anchor's world speed.
  BURST = ((1_000_116, 201, 57.25, -3.125), (1_000_117, 202, 58.75, 3.5),
           (1_000_118, 203, 58.75, 7.5), (1_000_119, 204, 60.25, 10.75))

  @staticmethod
  def obj(pid, raw_id, slot, ns, age, d_rel, y_rel, v_rel, *, vision=False, oem=False,
          extra_members=()):
    tracks = [BoschRawTrack(raw_id, BoschRawDetection(ns, slot, d_rel, y_rel, v_rel,
                                                      raw_word=raw_id), age, False)]
    for extra_raw, extra_slot in extra_members:
      tracks.append(BoschRawTrack(extra_raw, BoschRawDetection(ns, extra_slot, d_rel+.25, y_rel,
                                                               v_rel, raw_word=extra_raw),
                                  age, False))
    return BoschPhysicalObject(pid, ns, tuple(tracks), raw_id, d_rel, y_rel, v_rel, oem, vision,
                               age, 'single_return' if len(tracks) == 1 else 'temporal_complete_link')

  @classmethod
  def scan(cls, index, *, members=3, anchor=True, anchor_v=-5.5, births=True, child_age=None,
           d_span=None, y_span=None, world_offsets=(), drop=(), supported={}, grew=()):
    """One scan: a mature anchor plus `members` singleton births beside it.

    `index` 0 is the burst scan; later indices keep the same objects alive so a
    hold can be followed.  Every keyword exists to break exactly one clause of
    the burst proof without disturbing the others.
    """
    ns = (index+1)*cls.SCAN_NS
    result = []
    if anchor:
      result.append(cls.obj(cls.ANCHOR_PID, cls.ANCHOR_RAW, 4, ns, 30+index, 64.5, -3.375,
                            anchor_v))
    if births:
      selected = cls.BURST[:members]
      base_d = selected[0][2]
      base_y = selected[0][3]
      for position, (pid, raw, d_rel, y_rel) in enumerate(selected):
        if pid in drop:
          continue
        if d_span is not None:
          d_rel = base_d + d_span*position/max(1, len(selected)-1)
        if y_span is not None:
          y_rel = base_y + y_span*position/max(1, len(selected)-1)
        v_rel = -5.5 + (world_offsets[position] if position < len(world_offsets) else 0.)
        result.append(cls.obj(pid, raw, 20+position, ns, index+1 if child_age is None else child_age,
                              d_rel, y_rel, v_rel,
                              vision=supported.get(pid) == 'camera',
                              oem=supported.get(pid) == 'word1',
                              extra_members=((raw+64, 26+position),) if pid in grew else ()))
    return ns, tuple(result)

  @staticmethod
  def filter(mode=None):
    mode = radar_interface_module.BOSCH_BURST_MULTIRETURN_ACTIVE if mode is None else mode
    return radar_interface_module._BoschBurstMultiReturnDefer(mode)

  @classmethod
  def arm(cls, defer=None, **kwargs):
    defer = defer or cls.filter()
    ns, objects = cls.scan(0, **kwargs)
    held = defer.update(objects, ns, cls.V_EGO)
    return defer, ns, objects, held

  def test_bosch_burst_s33_style_burst_is_held_on_its_birth_scan(self):
    defer, _ns, objects, held = self.arm()
    assert held == frozenset({1_000_116, 1_000_117, 1_000_118})
    assert [obj.physical_track_id for obj in defer.publication_view(objects)] == [self.ANCHOR_PID]
    decision = defer.last_decisions[-1]
    assert (decision.action, decision.release_reason) == ('HOLD', '')
    assert decision.burst_members == 3
    assert decision.anchor_pid == self.ANCHOR_PID
    assert decision.burst_d_span_m == pytest.approx(1.5)
    assert decision.burst_y_span_m == pytest.approx(10.625)

  def test_bosch_burst_two_births_are_never_held(self):
    # Two surfaces are an ordinary pair of objects, not a proven burst.
    _defer, _ns, objects, held = self.arm(members=2, y_span=10.625)
    assert held == frozenset()

  def test_bosch_burst_four_births_are_held(self):
    _defer, _ns, objects, held = self.arm(members=4)
    assert held == frozenset({1_000_116, 1_000_117, 1_000_118, 1_000_119})

  def test_bosch_burst_range_spread_beyond_the_slab_is_not_a_burst(self):
    _defer, _ns, _objects, held = self.arm(d_span=4.0)
    assert held == frozenset()

  def test_bosch_burst_narrow_lateral_fan_is_not_a_burst(self):
    _defer, _ns, _objects, held = self.arm(y_span=6.0)
    assert held == frozenset()

  def test_bosch_burst_mixed_world_speeds_do_not_band_together(self):
    _defer, _ns, _objects, held = self.arm(world_offsets=(0., .5, 1.0))
    assert held == frozenset()

  def test_bosch_burst_without_a_mature_neighbour_is_not_held(self):
    _defer, _ns, _objects, held = self.arm(anchor=False)
    assert held == frozenset()

  def test_bosch_burst_neighbour_at_a_different_speed_is_not_an_anchor(self):
    _defer, _ns, _objects, held = self.arm(anchor_v=-2.)
    assert held == frozenset()

  def test_bosch_burst_member_with_word1_is_published(self):
    _defer, _ns, _objects, held = self.arm(supported={1_000_118: 'word1'})
    assert held == frozenset({1_000_116, 1_000_117})

  def test_bosch_burst_member_with_camera_is_published(self):
    _defer, _ns, _objects, held = self.arm(supported={1_000_118: 'camera'})
    assert held == frozenset({1_000_116, 1_000_117})

  def test_bosch_burst_member_with_validated_word0_is_published(self):
    defer = self.filter()
    ns, objects = self.scan(0)
    held = defer.update(objects, ns, self.V_EGO, word0_validated_pids=(1_000_118,))
    assert held == frozenset({1_000_116, 1_000_117})

  def test_bosch_burst_unvalidated_word0_is_not_identity_evidence(self):
    # word0 alone has been observed pointing at a clone, so the provider only
    # passes it here once the OEM validated it in the same scan.
    _defer, _ns, _objects, held = self.arm()
    assert 1_000_118 in held

  def test_bosch_burst_evidence_during_hold_releases_immediately(self):
    defer, _ns, _objects, _held = self.arm()
    ns, objects = self.scan(1, supported={1_000_118: 'camera'})
    assert defer.update(objects, ns, self.V_EGO) == frozenset({1_000_116, 1_000_117})
    release = next(d for d in defer.last_decisions if d.child_pid == 1_000_118)
    assert (release.action, release.release_reason) == ('RELEASE', 'CHILD_EVIDENCE')
    assert 1_000_118 in {obj.physical_track_id for obj in defer.publication_view(objects)}

  def test_bosch_burst_child_gaining_a_member_releases_immediately(self):
    defer, _ns, _objects, _held = self.arm()
    ns, objects = self.scan(1, grew=(1_000_118,))
    assert defer.update(objects, ns, self.V_EGO) == frozenset({1_000_116, 1_000_117})
    release = next(d for d in defer.last_decisions if d.child_pid == 1_000_118)
    assert release.release_reason == 'CHILD_GREW'

  def test_bosch_burst_two_missing_scans_expire_the_state(self):
    defer, _ns, _objects, _held = self.arm()
    ns, objects = self.scan(1, drop=(1_000_118,))
    assert 1_000_118 in defer.update(objects, ns, self.V_EGO)
    ns, objects = self.scan(2, drop=(1_000_118,))
    assert defer.update(objects, ns, self.V_EGO) == frozenset({1_000_116, 1_000_117})
    release = next(d for d in defer.last_decisions if d.child_pid == 1_000_118)
    assert release.release_reason == 'CHILD_LOST'
    assert 1_000_118 not in defer._states

  def test_bosch_burst_scan_gap_releases_and_cannot_start_a_burst(self):
    defer, ns, _objects, held = self.arm()
    assert held
    late_ns = ns + radar_interface_module.BOSCH_BURST_MAX_GAP_NS + 1
    _ns, objects = self.scan(0)
    assert defer.update(objects, late_ns, self.V_EGO) == frozenset()
    assert defer._states == {}
    assert {d.release_reason for d in defer.last_decisions} == {'SCAN_GAP'}

  def test_bosch_burst_hold_budget_bounds_the_defer_at_five_scans(self):
    defer, _ns, _objects, _held = self.arm()
    suppressed = 1
    for index in range(1, 12):
      ns, objects = self.scan(index)
      if 1_000_118 in defer.update(objects, ns, self.V_EGO):
        suppressed += 1
      else:
        break
    assert suppressed == radar_interface_module.BOSCH_BURST_HOLD_MAX_SCANS
    release = next(d for d in defer.last_decisions if d.child_pid == 1_000_118)
    assert release.release_reason == 'HOLD_BUDGET'
    assert defer._states == {}

  def test_bosch_burst_reset_clears_every_hold(self):
    defer, ns, _objects, held = self.arm()
    assert held and defer._states
    defer.reset(ns + self.SCAN_NS, 'STATE_RESET')
    assert defer._states == {} and defer.would_suppress == frozenset()
    assert {d.release_reason for d in defer.last_decisions} == {'STATE_RESET'}

  def test_bosch_burst_below_the_speed_floor_never_arms(self):
    defer = self.filter()
    ns, objects = self.scan(0)
    assert defer.update(objects, ns, 4.) == frozenset()

  def test_bosch_burst_off_and_shadow_modes_do_not_change_publication(self):
    for mode in (radar_interface_module.BOSCH_BURST_MULTIRETURN_OFF,
                 radar_interface_module.BOSCH_BURST_MULTIRETURN_SHADOW):
      defer = self.filter(mode)
      ns, objects = self.scan(0)
      held = defer.update(objects, ns, self.V_EGO)
      assert defer.publication_view(objects) == objects
      assert held == (frozenset() if mode == radar_interface_module.BOSCH_BURST_MULTIRETURN_OFF
                      else frozenset({1_000_116, 1_000_117, 1_000_118}))

  def test_bosch_burst_two_adjacent_vehicles_are_never_held(self):
    # Two real cars in neighbouring lanes, born together beside a mature third:
    # the pair count alone keeps them out, and so does the lateral span once a
    # plausible third object joins them one lane over.
    ns = self.SCAN_NS
    anchor = self.obj(self.ANCHOR_PID, self.ANCHOR_RAW, 4, ns, 30, 64.5, -3.375, -5.5)
    left = self.obj(1_000_301, 211, 21, ns, 1, 58.75, 3.5, -5.5)
    right = self.obj(1_000_302, 212, 22, ns, 1, 59.0, -0.125, -5.5)
    third = self.obj(1_000_303, 213, 23, ns, 1, 58.5, -3.5, -5.5)
    defer = self.filter()
    assert defer.update((anchor, left, right), ns, self.V_EGO) == frozenset()
    defer = self.filter()
    assert defer.update((anchor, left, right, third), ns, self.V_EGO) == frozenset()

  def test_bosch_burst_member_order_does_not_change_the_result(self):
    _ns, objects = self.scan(0)
    reference = self.filter().update(objects, self.SCAN_NS, self.V_EGO)
    for rotation in range(1, len(objects)):
      shuffled = objects[rotation:] + objects[:rotation]
      assert self.filter().update(shuffled, self.SCAN_NS, self.V_EGO) == reference

  def test_bosch_burst_does_not_change_family_companion_behaviour(self):
    # B1's own window is untouched: the same scans produce the same B1 states
    # and the same B1 hold whether or not the burst filter runs beside it.
    helper = TestBoschFamilyCompanionB1
    expected = []
    family = helper.filter()
    for index in range(3):
      ns, objects = helper.scan(index)
      expected.append(family.update(objects, ns, 29.5))
    family = helper.filter()
    defer = self.filter()
    actual = []
    for index in range(3):
      ns, objects = helper.scan(index)
      actual.append(family.update(objects, ns, 29.5))
      defer.update(objects, ns, 29.5)
    assert actual == expected
    assert expected[1] == frozenset({helper.NEWBORN_PID})
    assert defer.holds == 0

  def test_bosch_burst_hold_preserves_object_identity_and_state(self):
    # A deferred surface keeps its physical id, members, representative and age:
    # only the public tuple loses it, and only for this scan.
    defer, _ns, objects, held = self.arm()
    assert held
    view = defer.publication_view(objects)
    assert all(obj in objects for obj in view)
    ns, objects = self.scan(1, supported={1_000_118: 'word1'})
    defer.update(objects, ns, self.V_EGO)
    released = next(obj for obj in defer.publication_view(objects)
                    if obj.physical_track_id == 1_000_118)
    assert released.age_scans == 2
    assert released.representative_raw_track_id == 203
    assert [member.raw_track_id for member in released.members] == [203]

  def test_bosch_burst_state_is_bounded(self):
    defer, _ns, _objects, _held = self.arm(members=4)
    assert len(defer._states) <= radar_interface_module.BOSCH_BURST_STATE_MAX
    assert defer.state_peak == 4
    for index in range(1, 8):
      ns, objects = self.scan(index)
      defer.update(objects, ns, self.V_EGO)
      assert len(defer._states) <= radar_interface_module.BOSCH_BURST_STATE_MAX
    assert defer._states == {}


class TestBoschResearchShadowRemoval:
  def test_mirror_family_collector_is_not_wired(self):
    assert not hasattr(radar_interface_module, '_BoschMirrorFamilyResearchShadow')
    provider = BoschRadarProvider(1, qualification=False)
    assert not hasattr(provider, 'mirror_research_shadow')
    with pytest.raises(TypeError):
      BoschRadarProvider(1, qualification=False, mirror_research_shadow=True)


class TestBoschCameraCompanionCommonAncestry:
  START_NS = 20_000_000_000
  STEP_NS = 100_000_000
  EPISODE = 7
  NEAR = 1_000_101
  FAR = 1_000_102

  @staticmethod
  def raw(raw_id, slot, ns, d_rel, y_rel=0., v_rel=0., age=10):
    detection = BoschRawDetection(ns, slot, d_rel, y_rel, v_rel, raw_word=raw_id)
    return BoschRawTrack(raw_id, detection, age, False)

  @classmethod
  def obj(cls, pid, ns, raw_specs, representative, d_rel, *, oem=False, age=10):
    members = tuple(cls.raw(raw_id, slot, ns, d_rel, age=age) for raw_id, slot in raw_specs)
    return BoschPhysicalObject(pid, ns, members, representative, d_rel, 0., 0., oem, True,
                               age, 'single_return' if len(members) == 1 else 'temporal_complete_link')

  @classmethod
  def current_pair(cls, ns, *, near_pid=None, far_pid=None, near_raw=2, far_raw=1,
                   near_slot=2, far_slot=1):
    near_pid = cls.NEAR if near_pid is None else near_pid
    far_pid = cls.FAR if far_pid is None else far_pid
    near = cls.obj(near_pid, ns, ((near_raw, near_slot),), near_raw, 2.5, oem=True)
    far = cls.obj(far_pid, ns, ((far_raw, far_slot),), far_raw, 6.0)
    return near, far

  @staticmethod
  def set_live(manager, objects):
    manager.states = {
      obj.physical_track_id: radar_interface_module._BoschPhysicalState(
        obj, {member.raw_track_id: obj.timestamp_ns for member in obj.members})
      for obj in objects
    }
    manager._update_common_ancestry(objects[0].timestamp_ns, objects)

  @classmethod
  def provider(cls):
    provider = BoschRadarProvider(
      1, qualification=False,
      camera_extended_mode=radar_interface_module.BOSCH_CAMERA_EXTENDED_ACTIVE,
      p91_mode=radar_interface_module.BOSCH_P91_OFF,
      oem_gate_mode=radar_interface_module.BOSCH_OEM_GATE_OFF,
      family_companion_mode=radar_interface_module.BOSCH_FAMILY_COMPANION_OFF,
      burst_multireturn_mode=radar_interface_module.BOSCH_BURST_MULTIRETURN_OFF)
    ext = provider.camera_extended
    ext._associate = lambda _obj, *_args: (
      radar_interface_module.BOSCH_CAMERA_ASSOC_ASSIGNED, cls.EPISODE, 6)
    camera = radar_interface_module.BoschCameraObject(
      obj_id=3, episode=cls.EPISODE, long_m=4., width_m=2.5, class_code=6)
    ext.camera.snapshot = lambda ns: ([camera], 1, 0, ns)
    return provider

  @classmethod
  def publication(cls, provider, objects):
    ns = objects[0].timestamp_ns
    provider.last_oem_state = radar_interface_module.BOSCH_OEM_STATE_VALIDATED
    provider.camera_extended.update(ns, objects, 10.)
    return provider.publication_view(objects)

  @classmethod
  def seed(cls, provider, *, split=True, handoff=True):
    manager = provider.tracker.group_manager
    t0 = cls.START_NS
    if split:
      ancestor = cls.obj(cls.NEAR, t0, ((1, 1), (2, 2)), 1, 4.)
    else:
      ancestor = cls.obj(cls.NEAR, t0, ((1, 1),), 1, 4.)
    cls.set_live(manager, (ancestor,))
    t1 = t0 + cls.STEP_NS
    pair = (cls.current_pair(t1) if handoff else
            cls.current_pair(t1, near_raw=1, far_raw=2, near_slot=1, far_slot=2))
    cls.set_live(manager, pair)
    return pair

  def test_baseline_geometry_without_common_ancestry_keeps_far_object(self):
    provider = self.provider()
    pair = self.current_pair(self.START_NS)
    self.set_live(provider.tracker.group_manager, pair)
    assert [obj.physical_track_id for obj in self.publication(provider, pair)] == [self.NEAR, self.FAR]

  def test_group_split_without_representative_handoff_keeps_far_object(self):
    provider = self.provider()
    pair = self.seed(provider, split=True, handoff=False)
    evidence = provider.tracker.group_manager.common_ancestry_evidence(
      pair[0], pair[1], pair[0].timestamp_ns, pair[0].timestamp_ns)
    assert evidence == (True, False)
    assert [obj.physical_track_id for obj in self.publication(provider, pair)] == [self.NEAR, self.FAR]

  def test_representative_handoff_without_prior_group_split_keeps_far_object(self):
    provider = self.provider()
    pair = self.seed(provider, split=False, handoff=True)
    evidence = provider.tracker.group_manager.common_ancestry_evidence(
      pair[0], pair[1], pair[0].timestamp_ns, pair[0].timestamp_ns)
    assert evidence == (False, True)
    assert [obj.physical_track_id for obj in self.publication(provider, pair)] == [self.NEAR, self.FAR]

  def test_split_and_handoff_with_baseline_conditions_defers_far_object(self):
    provider = self.provider()
    pair = self.seed(provider)
    assert [obj.physical_track_id for obj in self.publication(provider, pair)] == [self.NEAR]
    assert provider.last_companion_deferred == (self.FAR,)

  def test_independent_long_lived_child_like_geometry_keeps_far_object(self):
    provider = self.provider()
    for index in range(100):
      pair = self.current_pair(self.START_NS + index * self.STEP_NS, near_raw=11, far_raw=12)
      self.set_live(provider.tracker.group_manager, pair)
      assert [obj.physical_track_id for obj in self.publication(provider, pair)] == [self.NEAR, self.FAR]

  def test_expired_ancestry_keeps_far_object(self):
    provider = self.provider()
    pair = self.seed(provider)
    assert len(self.publication(provider, pair)) == 1
    ns = pair[0].timestamp_ns + radar_interface_module.BOSCH_COMMON_ANCESTRY_MAX_AGE_NS + 1
    expired_pair = self.current_pair(ns)
    self.set_live(provider.tracker.group_manager, expired_pair)
    assert [obj.physical_track_id for obj in self.publication(provider, expired_pair)] == [self.NEAR, self.FAR]

  def test_pid_loss_and_provider_reset_do_not_reuse_stale_ancestry(self):
    provider = self.provider()
    pair = self.seed(provider)
    assert len(self.publication(provider, pair)) == 1
    manager = provider.tracker.group_manager
    far_only = (self.current_pair(pair[0].timestamp_ns + self.STEP_NS)[1],)
    self.set_live(manager, far_only)
    assert manager.common_group_ancestry == {}

    reset_provider = self.provider()
    reset_pair = self.seed(reset_provider)
    assert len(self.publication(reset_provider, reset_pair)) == 1
    assert reset_provider.update([], now_ns=reset_pair[0].timestamp_ns, v_ego=0.) is None
    timeout_ns = reset_pair[0].timestamp_ns + radar_interface_module.BOSCH_STALE_NS
    assert reset_provider.update([], now_ns=timeout_ns, v_ego=0.) == ()
    assert reset_provider.tracker.group_manager.common_group_ancestry == {}
    assert reset_provider.tracker.group_manager.common_representative_owners == {}
    reused_pair = self.current_pair(timeout_ns + self.STEP_NS)
    self.set_live(reset_provider.tracker.group_manager, reused_pair)
    assert [obj.physical_track_id for obj in self.publication(reset_provider, reused_pair)] == [self.NEAR, self.FAR]

  def test_raw_slot_reuse_without_lineage_keeps_far_object(self):
    provider = self.provider()
    ancestor = self.obj(self.NEAR, self.START_NS, ((1, 1), (2, 2)), 1, 4.)
    self.set_live(provider.tracker.group_manager, (ancestor,))
    pair = self.current_pair(self.START_NS + self.STEP_NS, near_raw=101, far_raw=102,
                             near_slot=1, far_slot=2)
    self.set_live(provider.tracker.group_manager, pair)
    assert [obj.physical_track_id for obj in self.publication(provider, pair)] == [self.NEAR, self.FAR]

  def test_pair_disappearance_and_reacquisition_do_not_reuse_ancestry(self):
    provider = self.provider()
    pair = self.seed(provider)
    assert len(self.publication(provider, pair)) == 1
    near_only = (self.current_pair(pair[0].timestamp_ns + self.STEP_NS)[0],)
    self.set_live(provider.tracker.group_manager, near_only)
    assert [obj.physical_track_id for obj in self.publication(provider, near_only)] == [self.NEAR]
    reacquired = self.current_pair(near_only[0].timestamp_ns + self.STEP_NS)
    self.set_live(provider.tracker.group_manager, reacquired)
    assert [obj.physical_track_id for obj in self.publication(provider, reacquired)] == [self.NEAR, self.FAR]

  def test_candidate_b_burst_decision_is_unchanged(self):
    helper = TestBoschBurstMultiReturnDefer
    first, second = helper.filter(), helper.filter()
    ns, objects = helper.scan(0)
    expected = first.update(objects, ns, helper.V_EGO)

    provider = self.provider()
    self.seed(provider)
    actual = second.update(objects, ns, helper.V_EGO)
    assert actual == expected == frozenset({1_000_116, 1_000_117, 1_000_118})

  def test_non_bosch_hyundai_does_not_construct_the_provider(self, monkeypatch):
    class FakeParams:
      def get_int(self, key):
        return 1 if key == 'EnableRadarTracks' else 0

    monkeypatch.setattr(radar_interface_module, 'Params', FakeParams)
    monkeypatch.setattr(radar_interface_module, 'get_radar_can_parser', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(radar_interface_module, 'get_radar_can_parser_scc', lambda *_args, **_kwargs: None)
    cp = structs.CarParams()
    cp.carFingerprint = CAR.HYUNDAI_GRANDEUR_IG
    cp.extFlags = 0
    cp.radarUnavailable = True
    interface = RadarInterface(cp)
    assert interface.bosch is None


class TestBoschRawAssociationTrace:
  """Raw returns are matched globally, and a coasted track outbids a new one.

  unmatched_cost is charged twice when a track dies and a return is born, so a
  single expensive match can win. The trace records what that match cost and
  what the return could have had instead.
  """
  SCAN_NS = 100_000_000

  @staticmethod
  def detection(slot, d_rel, y_rel, v_rel, timestamp_ns):
    return BoschRawDetection(timestamp_ns, slot, d_rel, y_rel, v_rel, raw_word=1)

  @classmethod
  def coasted_recovery_scans(cls):
    """A return drops out for one scan and comes back 3.25 m further away."""
    first = cls.SCAN_NS
    second = 2 * cls.SCAN_NS
    third = 3 * cls.SCAN_NS
    return [
      (first, (cls.detection(7, 11.50, 0.0, 0.0, first),
               cls.detection(20, 9.25, -1.84375, 0.25, first))),
      (second, (cls.detection(20, 9.25, -1.84375, 0.25, second),)),
      (third, (cls.detection(7, 14.75, 0.0, 0.0, third),)),
    ]

  @classmethod
  def run(cls, scans, trace):
    manager = BoschRawTrackManager()
    manager.trace_decisions = trace
    outputs, decisions = [], []
    for timestamp_ns, detections in scans:
      outputs.append(manager.update(timestamp_ns, detections))
      decisions.append(manager.last_decisions)
    return manager, outputs, decisions

  @staticmethod
  def signature(outputs):
    return [[(track.raw_track_id, track.slot, track.previous_slot, track.age_scans,
              track.recovered, track.d_rel, track.y_rel, track.v_rel)
             for track in scan] for scan in outputs]

  def test_trace_does_not_change_any_match(self):
    scans = self.coasted_recovery_scans()
    off_manager, off_outputs, off_decisions = self.run(scans, False)
    on_manager, on_outputs, _ = self.run(scans, True)

    assert self.signature(off_outputs) == self.signature(on_outputs)
    assert off_manager.stats == on_manager.stats
    assert off_manager.next_id == on_manager.next_id
    assert all(scan == () for scan in off_decisions)

  def test_coasted_track_keeps_its_id_across_a_near_gate_jump(self):
    scans = self.coasted_recovery_scans()
    _, outputs, decisions = self.run(scans, True)

    first_id = next(track.raw_track_id for track in outputs[0] if track.slot == 7)
    recovered = outputs[2][0]
    assert recovered.raw_track_id == first_id
    assert recovered.recovered

    decision = decisions[2][0]
    assert decision.raw_track_id == first_id
    assert decision.created is False
    assert decision.predicted_d_rel == pytest.approx(11.50)
    assert decision.residual_d_m == pytest.approx(3.25)
    # Inside the 3.5 m distance gate, so nothing rejects it; the trace is the
    # only place the size of the step is written down.
    assert decision.residual_d_m < BoschRawTrackingConfig().distance_gate_m
    assert decision.chosen_cost == pytest.approx((3.25 / 3.5) ** 2 / 3 - 0.03)
    assert decision.candidate_count == 1
    assert decision.best_alternative_cost is None

  def test_trace_reports_the_candidate_a_new_track_declined(self):
    """A closer return takes the coasted track, so the far one starts a new ID."""
    scans = self.coasted_recovery_scans()[:2]
    third = 3 * self.SCAN_NS
    scans.append((third, (self.detection(3, 11.50, 0.0, 0.0, third),
                          self.detection(7, 14.75, 0.0, 0.0, third),
                          self.detection(20, 9.25, -1.84375, 0.25, third))))
    _, outputs, decisions = self.run(scans, True)

    born = next(d for d in decisions[2] if d.created)
    assert born.slot == 7
    assert born.chosen_cost is None
    # The coasted track was a gated candidate for this return and lost it to the
    # closer one; without the trace that alternative leaves no record at all.
    assert born.candidate_count == 1
    assert born.best_alternative_cost == pytest.approx((3.25 / 3.5) ** 2 / 3 - 0.03)


class TestBoschOemValidationGate:
  """0x601 word0/word1 read as a state, with the stock SCC as veto evidence."""
  START_NS = 12_000_000_000
  PID = 2_000_001
  OTHER_PID = 2_000_002

  @classmethod
  def obj(cls, index, *, pid=None, d=70., y=-0.5, v=-9., vision=False, oem=False, members=1):
    pid = cls.PID if pid is None else pid
    ns = cls.START_NS + index * 100_000_000
    raw_id = pid % 100_000
    tracks = []
    for member in range(members):
      detection = BoschRawDetection(ns, (raw_id + member) % 32, d + .25 * member, y, v,
                                    raw_word=raw_id + member)
      tracks.append(BoschRawTrack(raw_id + member, detection, index + 1, False))
    return BoschPhysicalObject(pid, ns, tuple(tracks), raw_id, d, y, v, oem, vision,
                               index + 1, 'single_return' if members == 1 else 'temporal_complete_link')

  @staticmethod
  def gate(mode=None):
    mode = radar_interface_module.BOSCH_OEM_GATE_ACTIVE if mode is None else mode
    return radar_interface_module._BoschOemValidationGate(mode)

  @classmethod
  def feed_intent(cls, gate, index, value=0.):
    """Three fresh SCC12 samples inside the scan window, as the car sends them."""
    ns = cls.START_NS + index * 100_000_000
    for offset in (-40_000_000, -20_000_000, 0):
      gate.intent.ingest(ns + offset, value)

  @classmethod
  def run(cls, gate, count, *, start=0, intent=0., **kwargs):
    result = frozenset()
    for index in range(start, start + count):
      obj = cls.obj(index, **kwargs)
      if intent is not None:
        cls.feed_intent(gate, index, intent)
      result = gate.update((obj,), obj.timestamp_ns, 30.,
                           state=radar_interface_module.BOSCH_OEM_STATE_TENTATIVE,
                           word0_pids=(obj.physical_track_id,), oem_valid=False)
    return result

  def test_state_classification_covers_all_four_combinations(self):
    classify = radar_interface_module._BoschOemValidationGate.classify
    assert classify(True, True) == radar_interface_module.BOSCH_OEM_STATE_VALIDATED
    assert classify(False, True) == radar_interface_module.BOSCH_OEM_STATE_SELECTED
    assert classify(True, False) == radar_interface_module.BOSCH_OEM_STATE_TENTATIVE
    assert classify(False, False) == radar_interface_module.BOSCH_OEM_STATE_NONE

  def test_sustained_tentative_in_path_object_is_withheld(self):
    persist = radar_interface_module.BOSCH_OEM_GATE_PERSIST_SCANS
    gate = self.gate()
    assert self.run(gate, persist - 1) == frozenset()
    assert self.run(gate, 1, start=persist - 1) == frozenset({self.PID})
    assert gate.reasons[self.PID]["reason"] == "tentative_oem_disagreement"

  def test_validated_state_never_withholds(self):
    gate = self.gate()
    for index in range(8):
      obj = self.obj(index, oem=True)
      self.feed_intent(gate, index)
      assert gate.update((obj,), obj.timestamp_ns, 30.,
                         state=radar_interface_module.BOSCH_OEM_STATE_VALIDATED,
                         word0_pids=(self.PID,), oem_valid=True) == frozenset()

  def test_selected_only_side_target_is_published(self):
    gate = self.gate()
    for index in range(8):
      obj = self.obj(index, y=-4.5, oem=True)
      self.feed_intent(gate, index)
      assert gate.update((obj,), obj.timestamp_ns, 30.,
                         state=radar_interface_module.BOSCH_OEM_STATE_SELECTED,
                         word0_pids=(), oem_valid=True) == frozenset()

  @pytest.mark.parametrize("kwargs,reason", [
    ({"vision": True}, "recent_evidence"),
    ({"oem": True}, "was_validated"),
    ({"members": 2}, "recent_evidence"),
    ({"d": 25.}, "close_range"),
    ({"v": -25.}, "short_ttc"),
  ])
  def test_independent_evidence_and_safety_windows_fail_open(self, kwargs, reason):
    gate = self.gate()
    assert self.run(gate, 8, **kwargs) == frozenset()
    assert gate.reasons[self.PID]["reason"] == reason

  def test_low_speed_is_fail_open(self):
    gate = self.gate()
    held = frozenset()
    for index in range(8):
      obj = self.obj(index)
      self.feed_intent(gate, index)
      held = gate.update((obj,), obj.timestamp_ns, 3.,
                         state=radar_interface_module.BOSCH_OEM_STATE_TENTATIVE,
                         word0_pids=(self.PID,), oem_valid=False)
    assert held == frozenset()
    assert gate.reasons[self.PID]["reason"] == "low_speed"

  def test_scc_validity_acquisition_is_immediate_keep(self):
    gate = self.gate()
    assert self.run(gate, 8) == frozenset({self.PID})
    obj = self.obj(8)
    self.feed_intent(gate, 8)
    assert gate.update((obj,), obj.timestamp_ns, 30.,
                       state=radar_interface_module.BOSCH_OEM_STATE_TENTATIVE,
                       word0_pids=(self.PID,), oem_valid=True) == frozenset()
    assert gate.reasons[self.PID]["reason"] == "scc_valid"

  def test_word1_acquisition_recovers_without_identity_churn(self):
    gate = self.gate()
    assert self.run(gate, 8) == frozenset({self.PID})
    obj = self.obj(8, oem=True)
    self.feed_intent(gate, 8)
    assert gate.update((obj,), obj.timestamp_ns, 30.,
                       state=radar_interface_module.BOSCH_OEM_STATE_VALIDATED,
                       word0_pids=(self.PID,), oem_valid=True) == frozenset()
    # The gate owns no identity, so recovery cannot renumber a PID or an alias.
    assert gate.publication_withheld == 0

  def test_short_validation_dropout_keeps_grace(self):
    """11 -> 10 -> 11 must not demote a lead the OEM has ever validated."""
    gate = self.gate()
    for index in range(6):
      obj = self.obj(index, oem=True)
      self.feed_intent(gate, index)
      gate.update((obj,), obj.timestamp_ns, 30.,
                  state=radar_interface_module.BOSCH_OEM_STATE_VALIDATED,
                  word0_pids=(self.PID,), oem_valid=True)
    for index in range(6, 10):
      obj = self.obj(index)
      self.feed_intent(gate, index)
      assert gate.update((obj,), obj.timestamp_ns, 30.,
                         state=radar_interface_module.BOSCH_OEM_STATE_TENTATIVE,
                         word0_pids=(self.PID,), oem_valid=False) == frozenset()
    assert gate.reasons[self.PID]["reason"] == "was_validated"

  def test_camera_support_grace_survives_a_short_vision_dropout(self):
    gate = self.gate()
    for index in range(4):
      obj = self.obj(index, vision=True)
      self.feed_intent(gate, index)
      gate.update((obj,), obj.timestamp_ns, 30.,
                  state=radar_interface_module.BOSCH_OEM_STATE_TENTATIVE,
                  word0_pids=(self.PID,), oem_valid=False)
    assert self.run(gate, 4, start=4) == frozenset()
    assert gate.reasons[self.PID]["reason"] == "recent_evidence"

  def test_out_of_path_candidate_is_never_withheld(self):
    gate = self.gate()
    assert self.run(gate, 8, y=-2.4) == frozenset()
    assert gate.reasons[self.PID]["reason"] == "not_in_path"

  def test_object_settled_in_the_path_is_never_withheld(self):
    settled = radar_interface_module.BOSCH_OEM_GATE_SETTLED_SCANS
    gate = self.gate()
    assert self.run(gate, settled + 4) == frozenset()
    assert gate.reasons[self.PID]["reason"] == "settled_in_path"

  def test_companion_of_a_validated_large_vehicle_is_kept(self):
    """One return of a box truck is the OEM target; the other is not a ghost."""
    gate = self.gate()
    held = frozenset()
    for index in range(6):
      near = self.obj(index, d=64., y=-0.2, oem=True)
      far = self.obj(index, pid=self.OTHER_PID, d=72., y=-0.4)
      self.feed_intent(gate, index)
      held = gate.update((near, far), near.timestamp_ns, 30.,
                         state=radar_interface_module.BOSCH_OEM_STATE_TENTATIVE,
                         word0_pids=(self.OTHER_PID,), oem_valid=False)
    assert held == frozenset()
    assert gate.reasons[self.OTHER_PID]["reason"] == "validated_sibling"

  def test_monotone_lateral_cutin_is_fail_open(self):
    gate = self.gate()
    held = frozenset()
    for index in range(8):
      obj = self.obj(index, y=-2.2 + .2 * index)
      self.feed_intent(gate, index)
      held = gate.update((obj,), obj.timestamp_ns, 30.,
                         state=radar_interface_module.BOSCH_OEM_STATE_TENTATIVE,
                         word0_pids=(self.PID,), oem_valid=False)
    assert held == frozenset()
    assert gate.reasons[self.PID]["reason"] == "lateral_cutin"

  def test_oscillating_lateral_excursion_does_not_earn_the_cutin_exemption(self):
    """A large-vehicle ghost wanders in and back out; only unbroken runs count.

    The offsets are the measured route274 segment 19 car-carrier return, whose
    lateral excursion looked enough like a cut-in to promote it to leadOne.
    """
    gate = self.gate()
    held = frozenset()
    for index, y in enumerate((-1.50, -1.41, -1.12, -1.03, -0.94, -0.41, -0.91,
                               -0.94, -0.84, -0.47, -0.50, -0.44)):
      obj = self.obj(index, y=y)
      self.feed_intent(gate, index)
      held = gate.update((obj,), obj.timestamp_ns, 30.,
                         state=radar_interface_module.BOSCH_OEM_STATE_TENTATIVE,
                         word0_pids=(self.PID,), oem_valid=False)
    assert held == frozenset({self.PID})

  def test_oem_deceleration_request_is_fail_open(self):
    gate = self.gate()
    assert self.run(gate, 8, intent=-0.8) == frozenset()
    assert gate.reasons[self.PID]["reason"] == "oem_decelerating"

  def test_absent_oem_intent_is_fail_open(self):
    gate = self.gate()
    assert self.run(gate, 8, intent=None) == frozenset()
    assert gate.reasons[self.PID]["reason"] == "oem_intent_unavailable"

  def test_stale_oem_intent_is_fail_open(self):
    gate = self.gate()
    gate.intent.ingest(self.START_NS - 1_000_000_000, 0.)
    assert self.run(gate, 8, intent=None) == frozenset()
    assert gate.reasons[self.PID]["reason"] == "oem_intent_unavailable"

  def test_out_of_band_intent_samples_are_ignored(self):
    gate = self.gate()
    assert self.run(gate, 8, intent=-10.23) == frozenset()
    assert gate.reasons[self.PID]["reason"] == "oem_intent_unavailable"

  def test_ambiguous_word0_match_is_fail_open(self):
    gate = self.gate()
    held = frozenset()
    for index in range(8):
      first = self.obj(index)
      second = self.obj(index, pid=self.OTHER_PID, d=70.5)
      self.feed_intent(gate, index)
      held = gate.update((first, second), first.timestamp_ns, 30.,
                         state=radar_interface_module.BOSCH_OEM_STATE_TENTATIVE,
                         word0_pids=(self.PID, self.OTHER_PID), oem_valid=False)
    assert held == frozenset()

  def test_shadow_mode_decides_without_changing_publication(self):
    gate = self.gate(radar_interface_module.BOSCH_OEM_GATE_SHADOW)
    assert self.run(gate, 8) == frozenset({self.PID})
    assert gate.publication_withheld == 0

  def test_off_mode_holds_no_state(self):
    gate = self.gate(radar_interface_module.BOSCH_OEM_GATE_OFF)
    assert self.run(gate, 8) == frozenset()
    assert gate._states == {}

  def test_departed_objects_leave_no_state_behind(self):
    gate = self.gate()
    self.run(gate, 8)
    assert self.PID in gate._states
    gate.update((), self.START_NS + 900_000_000, 30.,
                state=radar_interface_module.BOSCH_OEM_STATE_NONE)
    assert gate._states == {}


class TestBoschRawAssociationLowSpeedInvariants:
  """What the low-speed raw-association audit measured, locked as behaviour.

  These are not aspirations: every threshold asserted here is the value the
  audit read off the production source, and every scenario is one the audit
  found in the nine-route corpus. They exist so a later association change has
  to state, in a diff, which of these it is choosing to move.
  """
  SCAN_NS = 100_000_000

  @staticmethod
  def detection(slot, d_rel, y_rel, v_rel, timestamp_ns):
    return BoschRawDetection(timestamp_ns, slot, d_rel, y_rel, v_rel, raw_word=1)

  @classmethod
  def run(cls, scans, config=None):
    manager = BoschRawTrackManager(config)
    outputs = [manager.update(ns, detections) for ns, detections in scans]
    return manager, outputs

  @staticmethod
  def ids(scan):
    return {track.slot: track.raw_track_id for track in scan}

  # --- near-range quantisation --------------------------------------------
  def test_one_lsb_jitter_at_three_metres_keeps_one_identity(self):
    """d steps by one 0.25 m LSB and y by one 0.03125 m LSB; still one track."""
    scans = []
    for index in range(8):
      ns = (index + 1) * self.SCAN_NS
      d = 3.00 + .25 * (index % 2)
      y = .25 + .03125 * (index % 3)
      scans.append((ns, (self.detection(4, d, y, 0., ns),)))
    _, outputs = self.run(scans)
    assert len({scan[0].raw_track_id for scan in outputs}) == 1
    assert outputs[-1][0].age_scans == 8

  # --- coast retries -------------------------------------------------------
  def test_one_scan_miss_recovers_the_same_identity(self):
    scans = [(self.SCAN_NS, (self.detection(4, 5., 0., 0., self.SCAN_NS),)),
             (2 * self.SCAN_NS, ()),
             (3 * self.SCAN_NS, (self.detection(4, 5., 0., 0., 3 * self.SCAN_NS),))]
    _, outputs = self.run(scans)
    assert outputs[2][0].raw_track_id == outputs[0][0].raw_track_id
    assert outputs[2][0].recovered

  def test_the_coast_window_is_exactly_three_nominal_scan_periods(self):
    """Recovery at a 300 ms gap keeps the identity; 400 ms starts a new one."""
    for gap_scans, same in ((3, True), (4, False)):
      first = self.SCAN_NS
      last = (1 + gap_scans) * self.SCAN_NS
      scans = [(first, (self.detection(4, 5., 0., 0., first),))]
      scans += [(index * self.SCAN_NS, ()) for index in range(2, 1 + gap_scans)]
      scans.append((last, (self.detection(4, 5., 0., 0., last),)))
      _, outputs = self.run(scans)
      assert (outputs[-1][0].raw_track_id == outputs[0][0].raw_track_id) is same

  def test_a_cadence_above_100ms_costs_the_third_retry(self):
    """coast_s is 0.3 s and the scan period is nominally 0.1 s, so the third
    retry survives or dies on clock drift alone. The audit measured a median
    cadence of 99.995 ms with p95 101.455 ms, and 80,568 of 156,515 raw-track
    expiries landing within 20 ms of the limit. Both sides are asserted here.
    """
    for period_ns, same in ((99_970_000, True), (100_030_000, False)):
      scans = [(period_ns, (self.detection(4, 5., 0., 0., period_ns),))]
      scans += [(index * period_ns, ()) for index in range(2, 4)]
      last = 4 * period_ns
      scans.append((last, (self.detection(4, 5., 0., 0., last),)))
      _, outputs = self.run(scans)
      assert (outputs[-1][0].raw_track_id == outputs[0][0].raw_track_id) is same

  # --- neighbours and competition -----------------------------------------
  def test_an_adjacent_near_range_return_is_not_absorbed(self):
    """Two returns 0.75 m apart laterally at 3 m are about 14 deg apart, outside
    the 12 deg near gate, so losing one never re-labels the other."""
    first, second = self.SCAN_NS, 2 * self.SCAN_NS
    scans = [(first, (self.detection(4, 3., 0., 0., first),
                      self.detection(5, 3., .75, 0., first))),
             (second, (self.detection(5, 3., .75, 0., second),))]
    _, outputs = self.run(scans)
    assert outputs[1][0].raw_track_id == self.ids(outputs[0])[5]

  def test_a_standstill_queue_never_swaps_identities(self):
    scans = []
    for index in range(6):
      ns = (index + 1) * self.SCAN_NS
      scans.append((ns, (self.detection(4, 5., 0., 0., ns),
                         self.detection(9, 8., 0., 0., ns))))
    _, outputs = self.run(scans)
    assert all(self.ids(scan) == self.ids(outputs[0]) for scan in outputs)

  def test_one_return_for_two_tracks_goes_to_the_cheaper_one(self):
    first, second = self.SCAN_NS, 2 * self.SCAN_NS
    scans = [(first, (self.detection(4, 5., 0., 0., first),
                      self.detection(9, 7., 0., 0., first))),
             (second, (self.detection(4, 5.25, 0., 0., second),))]
    _, outputs = self.run(scans)
    assert outputs[1][0].raw_track_id == self.ids(outputs[0])[4]

  def test_exact_identity_tie_is_traced_without_changing_generation(self):
    """A tied result is observable, but remains behaviour-neutral pending GT."""
    first, second = self.SCAN_NS, 2 * self.SCAN_NS
    manager = BoschRawTrackManager()
    manager.trace_decisions = True
    initial = manager.update(first, (
      self.detection(4, 5., 0., 0., first),
      self.detection(9, 7., 0., 0., first),
    ))
    observed = manager.update(second, (self.detection(12, 6., 0., 0., second),))

    assert observed[0].raw_track_id in {track.raw_track_id for track in initial}
    assert manager.last_tied_components == 1
    assert manager.last_ambiguous_detections == 1
    assert manager.stats["ambiguous_identity_components"] == 1
    assert manager.stats["ambiguous_identity_detections"] == 1
    assert not manager.last_decisions[0].created
    assert manager.last_decisions[0].identity_ambiguous

  def test_slot_evidence_resolves_an_otherwise_equal_identity_choice(self):
    first, second = self.SCAN_NS, 2 * self.SCAN_NS
    manager = BoschRawTrackManager()
    initial = manager.update(first, (
      self.detection(4, 5., 0., 0., first),
      self.detection(9, 7., 0., 0., first),
    ))
    observed = manager.update(second, (self.detection(4, 6., 0., 0., second),))

    assert observed[0].raw_track_id == self.ids(initial)[4]
    assert manager.last_tied_components == 0
    assert manager.last_ambiguous_detections == 0

  # --- slot is not identity ------------------------------------------------
  def test_slot_reuse_by_a_distant_target_starts_a_new_identity(self):
    first, second = self.SCAN_NS, 2 * self.SCAN_NS
    scans = [(first, (self.detection(4, 5., 0., 0., first),)),
             (second, (self.detection(4, 20., 0., 0., second),))]
    _, outputs = self.run(scans)
    assert outputs[1][0].raw_track_id != outputs[0][0].raw_track_id

  def test_the_same_slot_bonus_cannot_reach_past_a_hard_gate(self):
    config = BoschRawTrackingConfig(same_slot_bonus=.1)
    first, second = self.SCAN_NS, 2 * self.SCAN_NS
    scans = [(first, (self.detection(4, 5., 0., 0., first),)),
             (second, (self.detection(4, 9., 0., 0., second),))]
    _, outputs = self.run(scans, config)
    assert outputs[1][0].raw_track_id != outputs[0][0].raw_track_id

  # --- opposing motion -----------------------------------------------------
  def test_an_opposing_return_fails_the_velocity_gate(self):
    first, second = self.SCAN_NS, 2 * self.SCAN_NS
    scans = [(first, (self.detection(4, 20., 0., 0., first),)),
             (second, (self.detection(4, 20., 0., -2.25, second),))]
    _, outputs = self.run(scans)
    assert outputs[1][0].raw_track_id != outputs[0][0].raw_track_id

  # --- large vehicles ------------------------------------------------------
  def test_two_multi_return_vehicles_keep_four_identities(self):
    scans = []
    for index in range(6):
      ns = (index + 1) * self.SCAN_NS
      scans.append((ns, (self.detection(2, 10., 0., 0., ns),
                         self.detection(3, 11., .25, 0., ns),
                         self.detection(8, 20., 0., 0., ns),
                         self.detection(9, 21., .25, 0., ns))))
    _, outputs = self.run(scans)
    assert all(self.ids(scan) == self.ids(outputs[0]) for scan in outputs)
    assert len(set(self.ids(outputs[-1]).values())) == 4

  # --- the raw layer sees neither camera nor OEM --------------------------
  def test_raw_association_takes_no_camera_or_oem_input(self):
    """Camera staleness and OEM word conflicts cannot reach this layer, so no
    association verdict in the audit can be attributed to them."""
    import inspect
    parameters = set(inspect.signature(BoschRawTrackManager.update).parameters)
    assert parameters == {"self", "timestamp_ns", "detections", "yaw_rate"}
    assert set(BoschRawTrackingConfig().__dataclass_fields__) == {
      "distance_gate_m", "speed_gate_mps", "bearing_near_deg", "bearing_mid_deg",
      "bearing_far_deg", "bearing_distant_deg", "coast_s", "same_slot_bonus",
      "unmatched_cost"}

  # --- clock discipline ----------------------------------------------------
  def test_a_timestamp_reset_is_refused_and_leaves_state_untouched(self):
    manager = BoschRawTrackManager()
    manager.update(2 * self.SCAN_NS, (self.detection(4, 5., 0., 0., 2 * self.SCAN_NS),))
    before = dict(manager.stats)
    with pytest.raises(ValueError):
      manager.update(self.SCAN_NS, (self.detection(4, 5., 0., 0., self.SCAN_NS),))
    assert manager.stats == before
    assert manager.last_timestamp_ns == 2 * self.SCAN_NS

  def test_a_segment_boundary_gap_starts_new_identities(self):
    first = self.SCAN_NS
    later = first + 10_000_000_000
    scans = [(first, (self.detection(4, 5., 0., 0., first),)),
             (later, (self.detection(4, 5., 0., 0., later),))]
    _, outputs = self.run(scans)
    assert outputs[1][0].raw_track_id != outputs[0][0].raw_track_id

  # --- the bearing gate is the only lateral discriminator ------------------
  def test_the_bearing_gate_is_the_only_lateral_limit(self):
    config = BoschRawTrackingConfig()
    assert config.distance_gate_m == 3.5
    assert config.speed_gate_mps == 2.0
    assert config.coast_s == .3
    assert config.same_slot_bonus == .03
    for distance, degrees in ((14.9, 12.), (15., 5.), (29.9, 5.), (30., 2.5),
                              (59.9, 2.5), (60., 2.)):
      assert config.bearing_gate_rad(distance) == pytest.approx(math.radians(degrees))
    # implied lateral tolerance, which is what actually separates neighbours
    assert 3. * math.tan(config.bearing_gate_rad(3.)) == pytest.approx(.6376, abs=1e-3)
    assert 15. * math.tan(config.bearing_gate_rad(15.)) == pytest.approx(1.3123, abs=1e-3)


class TestBoschSidePassLateralEstimator:
  """Route2bc S21/S23: a real truck's side-body return slides along the body.

  The raw return is real and stays published; only its sliding lateral is not
  read as vehicle motion.  Actual lane entry must never wait for the gate.
  """
  SCAN_NS = 100_000_000
  PID = 1_000_845
  RAW = 822
  V_EGO = 28.

  @staticmethod
  def obj(pid, raw_id, ns, age, d_rel, y_rel, v_rel, *, oem=False, extra_members=()):
    tracks = [BoschRawTrack(raw_id, BoschRawDetection(ns, 24, float(d_rel), float(y_rel), float(v_rel),
                                                      raw_word=raw_id), age, False)]
    for extra_raw, extra_slot in extra_members:
      tracks.append(BoschRawTrack(extra_raw, BoschRawDetection(ns, extra_slot, float(d_rel) + .25, float(y_rel),
                                                               float(v_rel), raw_word=extra_raw), age, False))
    return BoschPhysicalObject(pid, ns, tuple(tracks), raw_id, float(d_rel), float(y_rel), float(v_rel), oem,
                               False, age, 'single_return' if len(tracks) == 1 else 'temporal_complete_link')

  @staticmethod
  def estimator(mode=None):
    mode = radar_interface_module.BOSCH_SIDEPASS_LATERAL_ACTIVE if mode is None else mode
    return radar_interface_module._BoschSidePassLateralEstimator(mode)

  @classmethod
  def side_pass(cls, index, *, pid=None, raw=None, y0=-2.9375, y_rate=.75, range_rate=-5., v_rel=-9., d0=18.75):
    """Truck side return: range closes 4 m/s slower than vRel, lateral drifts inward."""
    ns = (index + 1) * cls.SCAN_NS
    t = index * .1
    return ns, cls.obj(cls.PID if pid is None else pid, cls.RAW if raw is None else raw, ns, index + 1,
                       d0 + range_rate * t, y0 + y_rate * t, v_rel)

  @classmethod
  def run(cls, estimator, scans, decisions=None, **kwargs):
    published = []
    for ns, objects in scans:
      estimator.update(objects, ns, cls.V_EGO, **kwargs)
      if decisions is not None:
        decisions.extend(estimator.last_decisions)
      view = estimator.publication_view(objects)
      published.append({obj.physical_track_id: radar_interface_module.bosch_published_surface(obj)[2]
                        for obj in view})
    return published

  def settle_rigid(self, estimator, ns, d, y, age, *, scans=30):
    """Feed rigid (range follows vRel) scans at a fixed lateral until disarmed."""
    for _ in range(scans):
      ns += self.SCAN_NS
      d -= .9
      age += 1
      obj = self.obj(self.PID, self.RAW, ns, age, d, y, -9.)
      estimator.update((obj,), ns, self.V_EGO)
      if not estimator._states[self.PID].armed:
        return ns, d, age, obj
    raise AssertionError('slide evidence never ended')

  def armed_estimator(self, scans=6, d0=18.75, y0=-2.9375):
    estimator = self.estimator()
    history = [self.side_pass(index, d0=d0, y0=y0) for index in range(scans)]
    published = self.run(estimator, [(ns, (obj,)) for ns, obj in history])
    assert any(decision.action == 'ARM' for decision in estimator.last_decisions) or \
      estimator._states[self.PID].armed
    return estimator, history, published

  def test_side_pass_sliding_return_keeps_its_physical_anchor(self):
    estimator = self.estimator()
    history = [self.side_pass(index) for index in range(11)]
    published = self.run(estimator, [(ns, (obj,)) for ns, obj in history])
    state = estimator._states[self.PID]
    assert state.armed and estimator.arms == 1
    ys = [row[self.PID] for row in published]
    arm_index = next(index for index, row in enumerate(published) if row[self.PID] != history[index][1].y_rel)
    # Before arming the raw lateral is published unchanged; afterwards the
    # anchor holds while the raw return keeps sliding inward.
    assert ys[:arm_index] == [obj.y_rel for _, obj in history[:arm_index]]
    assert len(set(ys[arm_index - 1:])) == 1
    assert history[-1][1].y_rel > ys[-1] + .3

  def test_raw_member_identity_and_publication_are_preserved(self):
    estimator = self.estimator()
    history = [self.side_pass(index) for index in range(11)]
    for ns, obj in history:
      estimator.update((obj,), ns, self.V_EGO)
      view = estimator.publication_view((obj,))
    ns, obj = history[-1]
    assert len(view) == 1
    public = view[0]
    assert public.physical_track_id == obj.physical_track_id
    assert public.members == obj.members
    assert (public.d_rel, public.y_rel, public.v_rel) == (obj.d_rel, obj.y_rel, obj.v_rel)
    assert public.representative_raw_track_id == obj.representative_raw_track_id
    surface = public.published_surface
    assert surface.lateral_estimate and surface.raw_track_id == self.RAW
    assert (surface.d_rel, surface.v_rel) == (obj.d_rel, obj.v_rel)
    assert obj.members[0].y_rel == obj.y_rel  # the raw observation itself is untouched

  def test_rigid_inward_lane_entry_is_never_armed(self):
    estimator = self.estimator()
    scans = [self.side_pass(index, range_rate=-9., v_rel=-9., y_rate=.9) for index in range(12)]
    published = self.run(estimator, [(ns, (obj,)) for ns, obj in scans])
    assert estimator.arms == 0
    assert [row[self.PID] for row in published] == [obj.y_rel for _, obj in scans]

  def test_corridor_entry_releases_on_the_same_scan(self):
    estimator, history, _ = self.armed_estimator()
    ns = history[-1][0] + self.SCAN_NS
    entering = self.obj(self.PID, self.RAW, ns, 7, 15., -1.5, -9.)
    estimator.update((entering,), ns, self.V_EGO)
    view = estimator.publication_view((entering,))
    assert view[0] is entering
    assert estimator.last_decisions[-1].reason == 'CORRIDOR_ENTRY'
    assert not estimator._states[self.PID].armed

  def test_oem_word1_selection_releases_on_the_same_scan(self):
    estimator, history, _ = self.armed_estimator()
    ns, obj = self.side_pass(6)
    selected = self.obj(self.PID, self.RAW, ns, 7, obj.d_rel, obj.y_rel, obj.v_rel, oem=True)
    estimator.update((selected,), ns, self.V_EGO)
    assert estimator.publication_view((selected,))[0] is selected
    assert estimator.last_decisions[-1].reason == 'OEM_SELECTED'

  def test_validated_word0_selection_releases_on_the_same_scan(self):
    estimator, _, _ = self.armed_estimator()
    ns, obj = self.side_pass(6)
    estimator.update((obj,), ns, self.V_EGO, oem_pids=frozenset({self.PID}))
    assert estimator.publication_view((obj,))[0] is obj
    assert estimator.last_decisions[-1].reason == 'OEM_SELECTED'

  def test_inward_travel_beyond_one_body_releases(self):
    estimator = self.estimator()
    decisions = []
    scans = [self.side_pass(index, y0=-4.9375, y_rate=1.5) for index in range(13)]
    published = self.run(estimator, [(ns, (obj,)) for ns, obj in scans], decisions)
    assert 'BEYOND_BODY' in [decision.reason for decision in decisions]
    gaps = [abs(row[self.PID] - obj.y_rel) for row, (_, obj) in zip(published, scans)]
    assert max(gaps) <= radar_interface_module.BOSCH_SIDEPASS_MAX_OFFSET_M + 1e-6
    release = next(index for index, decision in enumerate(decisions) if decision.reason == 'BEYOND_BODY')
    assert decisions[release].published_y_m == decisions[release].raw_y_m

  def test_evidence_end_realigns_below_the_downstream_motion_floor(self):
    estimator, history, _ = self.armed_estimator(8, d0=44.)
    last_ns, last = history[-1]
    y = last.y_rel
    ns, d, age, obj = self.settle_rigid(estimator, last_ns, last.d_rel, y, 8)
    offsets = [radar_interface_module.bosch_published_surface(estimator.publication_view((obj,))[0])[2] - y]
    for _ in range(12):
      ns += self.SCAN_NS
      d -= .9
      age += 1
      obj = self.obj(self.PID, self.RAW, ns, age, d, y, -9.)
      estimator.update((obj,), ns, self.V_EGO)
      offsets.append(radar_interface_module.bosch_published_surface(estimator.publication_view((obj,))[0])[2] - y)
    step_limit = radar_interface_module.BOSCH_SIDEPASS_REALIGN_MPS * .1 + 1e-9
    assert offsets[0] != 0.
    for before, after in zip(offsets, offsets[1:]):
      assert abs(after - before) <= step_limit
      assert abs(after) <= abs(before)
    assert abs(offsets[-1]) < abs(offsets[0])

  def test_rigid_motion_after_the_slide_passes_through_without_a_jump(self):
    estimator, history, _ = self.armed_estimator(8, d0=44., y0=-3.9375)
    last_ns, last = history[-1]
    y = last.y_rel
    ns, d, age, obj = self.settle_rigid(estimator, last_ns, last.d_rel, y, 8)
    published = radar_interface_module.bosch_published_surface(estimator.publication_view((obj,))[0])[2]
    step_limit = radar_interface_module.BOSCH_SIDEPASS_REALIGN_MPS * .1 + 1e-9
    for _ in range(6):
      ns += self.SCAN_NS
      d -= .9
      y += .1  # rigid inward travel of the raw return after the slide ended
      age += 1
      obj = self.obj(self.PID, self.RAW, ns, age, d, y, -9.)
      estimator.update((obj,), ns, self.V_EGO)
      now = radar_interface_module.bosch_published_surface(estimator.publication_view((obj,))[0])[2]
      # the new raw movement reaches the published lateral one-to-one; only the
      # residual offset re-aligns, never as a step
      assert abs((now - published) - .1) <= step_limit
      published = now

  def test_anchor_survives_a_one_scan_absence_of_the_same_track(self):
    estimator, _, published = self.armed_estimator()
    anchor = published[-1][self.PID]
    ns, _ = self.side_pass(6)
    estimator.update((), ns, self.V_EGO)
    assert estimator._states[self.PID].armed and estimator.last_decisions == ()
    decisions = []
    ns, obj = self.side_pass(7)
    after = self.run(estimator, [(ns, (obj,))], decisions)
    # reacquisition continues the anchor instead of stepping to the raw lateral
    assert after[0][self.PID] == anchor and obj.y_rel != anchor
    assert decisions == [] and estimator._states[self.PID].armed

  def test_pid_death_and_rebirth_do_not_inherit_the_anchor(self):
    estimator, history, _ = self.armed_estimator()
    ns = history[-1][0]
    decisions = []
    while ns - history[-1][0] <= radar_interface_module.BOSCH_SIDEPASS_MAX_GAP_NS:
      ns += self.SCAN_NS
      estimator.update((), ns, self.V_EGO)
      decisions.extend(estimator.last_decisions)
    assert self.PID not in estimator._states
    assert [(decision.action, decision.reason) for decision in decisions] == [('RESET', 'ABSENT')]
    reborn_pid = self.PID + 1
    ns += self.SCAN_NS
    returning = self.obj(self.PID, self.RAW, ns, 12, 14., -2.2, -9.)
    reborn = self.obj(reborn_pid, self.RAW + 1, ns, 1, 15., -2.5, -9.)
    estimator.update((returning, reborn), ns, self.V_EGO)
    view = estimator.publication_view((returning, reborn))
    assert view[0] is returning and view[1] is reborn
    for pid in (self.PID, reborn_pid):
      assert not estimator._states[pid].armed and len(estimator._states[pid].samples) == 1

  def test_representative_change_resets_the_state(self):
    estimator, history, _ = self.armed_estimator()
    ns, obj = self.side_pass(6, raw=self.RAW + 5)
    estimator.update((obj,), ns, self.V_EGO)
    assert estimator.publication_view((obj,))[0] is obj
    assert any(decision.reason == 'REPRESENTATIVE_CHANGE' for decision in estimator.last_decisions)
    assert estimator._states[self.PID].raw_track_id == self.RAW + 5
    assert not estimator._states[self.PID].armed

  def test_multi_member_object_never_starts_or_arms(self):
    estimator = self.estimator()
    scans = []
    for index in range(10):
      ns, obj = self.side_pass(index)
      scans.append((ns, (self.obj(obj.physical_track_id, self.RAW, ns, index + 1, obj.d_rel, obj.y_rel, obj.v_rel,
                                  extra_members=((self.RAW + 1, 25),)),)))
    published = self.run(estimator, scans)
    assert estimator.arms == 0 and self.PID not in estimator._states
    assert [row[self.PID] for row in published] == [objects[0].y_rel for _, objects in scans]

  def test_scan_gap_resets_every_state(self):
    estimator, history, _ = self.armed_estimator()
    ns, obj = self.side_pass(6)
    ns += radar_interface_module.BOSCH_SIDEPASS_MAX_GAP_NS + self.SCAN_NS
    late = self.obj(self.PID, self.RAW, ns, 7, obj.d_rel, obj.y_rel, obj.v_rel)
    estimator.update((late,), ns, self.V_EGO)
    assert estimator.publication_view((late,))[0] is late
    assert estimator.last_decisions[0].reason == 'SCAN_GAP'
    assert not estimator._states[self.PID].armed

  def test_provider_timeout_drops_every_anchor(self):
    provider = BoschRadarProvider(1)
    estimator = provider.sidepass_lateral
    history = [self.side_pass(index) for index in range(8)]
    for ns, obj in history:
      estimator.update((obj,), ns, self.V_EGO)
    assert estimator._states[self.PID].armed
    start_ns = history[-1][0]
    assert provider.update([], now_ns=start_ns, v_ego=self.V_EGO) is None
    timeout_ns = start_ns + radar_interface_module.BOSCH_STALE_NS
    assert provider.update([], now_ns=timeout_ns, v_ego=self.V_EGO) == ()
    assert estimator._states == {} and estimator.published_y == {}
    assert [(decision.action, decision.reason) for decision in estimator.last_decisions] == \
      [('RESET', 'PROVIDER_TIMEOUT')]
    ns = timeout_ns + self.SCAN_NS
    returning = self.obj(self.PID, self.RAW, ns, 20, 14., -2.4, -9.)
    estimator.update((returning,), ns, self.V_EGO)
    assert estimator.publication_view((returning,))[0] is returning
    assert not estimator._states[self.PID].armed and len(estimator._states[self.PID].samples) == 1

  def test_shadow_mode_decides_without_changing_publication(self):
    estimator = self.estimator(radar_interface_module.BOSCH_SIDEPASS_LATERAL_SHADOW)
    history = [self.side_pass(index) for index in range(11)]
    for ns, obj in history:
      estimator.update((obj,), ns, self.V_EGO)
      assert estimator.publication_view((obj,))[0] is obj
    assert estimator.arms == 1 and estimator.published_y

  def test_off_mode_holds_no_state(self):
    estimator = self.estimator(radar_interface_module.BOSCH_SIDEPASS_LATERAL_OFF)
    for ns, obj in (self.side_pass(index) for index in range(11)):
      estimator.update((obj,), ns, self.V_EGO)
      assert estimator.publication_view((obj,))[0] is obj
    assert estimator._states == {} and estimator.arms == 0

  def test_slow_or_opposite_direction_objects_are_never_armed(self):
    for v_rel, v_ego in ((-1., 28.), (-9., 5.), (-30., 20.)):
      estimator = self.estimator()
      for index in range(11):
        ns, obj = self.side_pass(index, v_rel=v_rel, range_rate=v_rel + 4., d0=44.)
        estimator.update((obj,), ns, v_ego)
      assert estimator.arms == 0, (v_rel, v_ego)

  def test_two_adjacent_real_vehicles_stay_independent(self):
    estimator = self.estimator()
    other_pid, other_raw = self.PID + 7, self.RAW + 7
    scans = []
    for index in range(11):
      ns, sliding = self.side_pass(index)
      rigid = self.obj(other_pid, other_raw, ns, index + 1, 25. - .9 * index, 2.9 - .06 * index, -9.)
      scans.append((ns, (sliding, rigid)))
    published = self.run(estimator, scans)
    assert estimator._states[self.PID].armed
    assert other_pid not in estimator.published_y
    assert [row[other_pid] for row in published] == [objects[1].y_rel for _, objects in scans]

  def test_decisions_are_prefix_invariant(self):
    history = [self.side_pass(index) for index in range(14)]
    full = self.run(self.estimator(), [(ns, (obj,)) for ns, obj in history])
    for cut in (3, 5, 8, 13):
      assert self.run(self.estimator(), [(ns, (obj,)) for ns, obj in history[:cut]]) == full[:cut]

  def test_estimate_never_mutates_tracker_objects(self):
    estimator = self.estimator()
    history = [self.side_pass(index) for index in range(11)]
    for ns, obj in history:
      before = (obj.physical_track_id, obj.members, obj.d_rel, obj.y_rel, obj.v_rel, obj.published_surface)
      estimator.update((obj,), ns, self.V_EGO)
      estimator.publication_view((obj,))
      assert (obj.physical_track_id, obj.members, obj.d_rel, obj.y_rel, obj.v_rel, obj.published_surface) == before

  def test_provider_publishes_the_estimate_through_the_native_point(self):
    provider = BoschRadarProvider(1)
    history = [self.side_pass(index) for index in range(11)]
    view = ()
    for ns, obj in history:
      provider.sidepass_lateral.update((obj,), ns, self.V_EGO)
      view = provider.sidepass_lateral.publication_view((obj,))
    radar = structs.RadarData.new_message()
    bosch_append_points(radar, view, self.V_EGO, history[-1][0])
    point = radar.points[0]
    assert point.trackId == self.PID
    assert point.yRel == pytest.approx(provider.sidepass_lateral.published_y[self.PID], abs=1e-5)
    assert point.yRel < history[-1][1].y_rel - .3
    assert point.vRel == pytest.approx(history[-1][1].v_rel)
    assert provider.debug_snapshot == {} or 'sidepass_lateral_mode' in provider.debug_snapshot


# Generated by scripts/split_bosch.py: preserve the public compatibility surface.
def test_bosch_names_are_identity_reexported():
  names = (
    'BOSCH_INACTIVE_WORD',
    'BOSCH_MAX_RAW_TRACK_ID',
    'BOSCH_TRACK_ADDRESSES',
    'BOSCH_WINDOW_NS',
    'BOSCH_STALE_NS',
    'BOSCH_OUTPUT_INTERVAL_NS',
    'BOSCH_SAMPLE_HOLD_NS',
    'BOSCH_ALEAD_SAMPLE_PERIOD_S',
    'BOSCH_ALEAD_STATE_MAX',
    'BOSCH_B5_OFF',
    'BOSCH_B5_ACTIVE',
    'BOSCH_B5_MODE',
    'BOSCH_B5_PARENT_STABILITY_SCANS',
    'BOSCH_B5_COUNTER_MAX',
    'BOSCH_B5_CONTEXT_MAX_AGE_NS',
    'BOSCH_B5_HISTORY_NS',
    'BOSCH_B5_HISTORY_MIN_SPAN_NS',
    'BOSCH_B5_MOTION_HISTORY_NS',
    'BOSCH_B5_RADAR_TO_DEVICE_X_M',
    'BOSCH_B5_OUTSIDE_MARGIN_M',
    'BOSCH_B5_PARENT_MIN_AGE_SCANS',
    'BOSCH_B5_MIN_WORLD_SPEED_MPS',
    'BOSCH_B5_MAX_D_M',
    'BOSCH_B5_MAX_Y_M',
    'BOSCH_B5_MAX_DV_MPS',
    'BOSCH_B5_MAX_DWORLD_MPS',
    'BOSCH_B5_MIN_VEGO_MPS',
    'BOSCH_B5_MAX_CURVATURE_1PM',
    'BOSCH_B5_EDGE_STD_MAX_M',
    'BOSCH_B5_EDGE_WIDTH_MIN_M',
    'BOSCH_B5_EDGE_WIDTH_MAX_M',
    'BOSCH_B5_LANE_PROB_MIN',
    'BOSCH_B5_LANE_STD_MAX_M',
    'BOSCH_B5_LANE_STABILITY_MAX_M',
    'BOSCH_B5_CORRIDOR_WIDTH_MIN_M',
    'BOSCH_B5_CORRIDOR_WIDTH_MAX_M',
    'BOSCH_B5_EDGE_LANE_BAND_MAX_M',
    'BOSCH_B5_DIVERGENCE_MIN_M',
    'BOSCH_P91_OFF',
    'BOSCH_P91_SHADOW',
    'BOSCH_P91_ACTIVE',
    'BOSCH_P91_ACTIVE_TEST',
    'BOSCH_P91_MODE',
    'BOSCH_P91_SUPPORT_HOLD_NS',
    'BOSCH_FAMILY_COMPANION_OFF',
    'BOSCH_FAMILY_COMPANION_SHADOW',
    'BOSCH_FAMILY_COMPANION_ACTIVE',
    'BOSCH_FAMILY_COMPANION_MODE',
    'BOSCH_FAMILY_MIN_ANCHOR_AGE_SCANS',
    'BOSCH_FAMILY_MIN_D_M',
    'BOSCH_FAMILY_MAX_D_M',
    'BOSCH_FAMILY_MAX_Y_M',
    'BOSCH_FAMILY_MAX_DV_MPS',
    'BOSCH_FAMILY_MAX_WORLD_SPEED_MPS',
    'BOSCH_FAMILY_MAX_D_STEP_M',
    'BOSCH_FAMILY_MAX_Y_STEP_M',
    'BOSCH_FAMILY_MIN_WORLD_SPEED_MPS',
    'BOSCH_FAMILY_MAX_GAP_NS',
    'BOSCH_FAMILY_INWARD_SCANS',
    'BOSCH_FAMILY_INWARD_MPS',
    'BOSCH_BURST_MULTIRETURN_OFF',
    'BOSCH_BURST_MULTIRETURN_SHADOW',
    'BOSCH_BURST_MULTIRETURN_ACTIVE',
    'BOSCH_BURST_MULTIRETURN_MODE',
    'BOSCH_BURST_MIN_MEMBERS',
    'BOSCH_BURST_MAX_WORLD_SPAN_MPS',
    'BOSCH_BURST_MAX_D_SPAN_M',
    'BOSCH_BURST_MIN_Y_SPAN_M',
    'BOSCH_BURST_MIN_WORLD_SPEED_MPS',
    'BOSCH_BURST_MIN_VEGO_MPS',
    'BOSCH_BURST_NEIGHBOUR_AGE_SCANS',
    'BOSCH_BURST_NEIGHBOUR_D_M',
    'BOSCH_BURST_HOLD_MAX_SCANS',
    'BOSCH_BURST_MAX_MISSING_SCANS',
    'BOSCH_BURST_MAX_GAP_NS',
    'BOSCH_BURST_STATE_MAX',
    'BOSCH_SIDEPASS_LATERAL_OFF',
    'BOSCH_SIDEPASS_LATERAL_SHADOW',
    'BOSCH_SIDEPASS_LATERAL_ACTIVE',
    'BOSCH_SIDEPASS_LATERAL_MODE',
    'BOSCH_SIDEPASS_WINDOW_NS',
    'BOSCH_SIDEPASS_MIN_SCANS',
    'BOSCH_SIDEPASS_MIN_SPAN_NS',
    'BOSCH_SIDEPASS_SLIDE_ARM_MPS',
    'BOSCH_SIDEPASS_SLIDE_RELEASE_MPS',
    'BOSCH_SIDEPASS_RELEASE_SCANS',
    'BOSCH_SIDEPASS_CLOSING_MPS',
    'BOSCH_SIDEPASS_MIN_LEAD_SPEED_MPS',
    'BOSCH_SIDEPASS_CORRIDOR_M',
    'BOSCH_SIDEPASS_ZONE_M',
    'BOSCH_SIDEPASS_MAX_D_M',
    'BOSCH_SIDEPASS_MAX_OFFSET_M',
    'BOSCH_SIDEPASS_REALIGN_MPS',
    'BOSCH_SIDEPASS_MAX_GAP_NS',
    'BOSCH_SIDEPASS_STATE_MAX',
    'BOSCH_OEM_STATE_NONE',
    'BOSCH_OEM_STATE_TENTATIVE',
    'BOSCH_OEM_STATE_SELECTED',
    'BOSCH_OEM_STATE_VALIDATED',
    'BOSCH_OEM_GATE_OFF',
    'BOSCH_OEM_GATE_SHADOW',
    'BOSCH_OEM_GATE_ACTIVE',
    'BOSCH_OEM_GATE_MODE',
    'BOSCH_OEM_EVIDENCE_HOLD_NS',
    'BOSCH_OEM_INTENT_MIN_MPS2',
    'BOSCH_OEM_INTENT_MAX_MPS2',
    'BOSCH_OEM_INTENT_FRESH_NS',
    'BOSCH_OEM_INTENT_WINDOW_NS',
    'BOSCH_OEM_INTENT_MIN_SAMPLES',
    'BOSCH_OEM_GATE_NEUTRAL_MPS2',
    'BOSCH_OEM_GATE_PERSIST_SCANS',
    'BOSCH_OEM_GATE_MIN_SPEED_MPS',
    'BOSCH_OEM_GATE_MIN_RANGE_M',
    'BOSCH_OEM_GATE_MIN_TTC_S',
    'BOSCH_OEM_GATE_CUTIN_MPS',
    'BOSCH_OEM_GATE_CUTIN_SCANS',
    'BOSCH_OEM_GATE_LATERAL_WINDOW_NS',
    'BOSCH_OEM_GATE_IN_PATH_M',
    'BOSCH_OEM_GATE_SETTLED_SCANS',
    'BOSCH_OEM_GATE_SIBLING_D_M',
    'BOSCH_OEM_GATE_SIBLING_Y_M',
    'BOSCH_OEM_GATE_SIBLING_V_MPS',
    'BOSCH_SCC_BUS',
    'BOSCH_SCC11_ADDR',
    'BOSCH_SCC12_ADDR',
    'BOSCH_SCC11_OBJ_VALID_BIT',
    'BOSCH_SCC12_AREQ_RAW_BIT',
    'BOSCH_SCC_ADDRESSES',
    'BOSCH_SCC_STALE_NS',
    'BOSCH_CAMERA_EXTENDED_OFF',
    'BOSCH_CAMERA_EXTENDED_SHADOW',
    'BOSCH_CAMERA_EXTENDED_ACTIVE',
    'BOSCH_CAMERA_EXTENDED_ACTIVE_TEST',
    'BOSCH_CAMERA_EXTENDED_TEST_INTERVALS',
    'BOSCH_CAMERA_MATURITY_HOLD_NS',
    'BOSCH_CAMERA_MATURITY_WORD1_INTERVALS',
    'BOSCH_CAMERA_EXTENDED_MODE',
    'BOSCH_CAMERA_HEADER',
    'BOSCH_CAMERA_FIRST_OBJECT',
    'BOSCH_CAMERA_LAST_OBJECT',
    'BOSCH_CAMERA_LAST_FAMILY',
    'BOSCH_CAMERA_SLOTS',
    'BOSCH_CAMERA_PERIOD_NS',
    'BOSCH_CAMERA_FRAME_TOLERANCE_NS',
    'BOSCH_CAMERA_OBSERVATION_GAP_NS',
    'BOSCH_CAMERA_E2_HOLD_NS',
    'BOSCH_CAMERA_SNAPSHOT_COUNT',
    'BOSCH_CAMERA_WIDTH_REF_M',
    'BOSCH_CAMERA_RANGE_LSB',
    'BOSCH_CAMERA_LONG_BASE_M',
    'BOSCH_CAMERA_LONG_RANGE_K',
    'BOSCH_CAMERA_LONG_POS_WIDTH_K',
    'BOSCH_CAMERA_LONG_NEG_WIDTH_K',
    'BOSCH_CAMERA_LONG_MAX_M',
    'BOSCH_CAMERA_LAT_MAX_M',
    'BOSCH_CAMERA_ANGLE_LSB',
    'BOSCH_CAMERA_ASSOC_UNRESOLVED',
    'BOSCH_CAMERA_ASSOC_ASSIGNED',
    'BOSCH_CAMERA_ASSOC_AMBIGUOUS',
    'BOSCH_CAMERA_CURVE_REACQUIRE_OFF',
    'BOSCH_CAMERA_CURVE_REACQUIRE_SHADOW',
    'BOSCH_CAMERA_CURVE_REACQUIRE_ACTIVE',
    'BOSCH_CAMERA_CURVE_REACQUIRE_MODE',
    'BOSCH_CAMERA_CURVE_REACQUIRE_HOLD_NS',
    'BOSCH_CAMERA_CURVE_REACQUIRE_MIN_D_M',
    'BOSCH_CAMERA_CURVE_REACQUIRE_MAX_D_M',
    'BOSCH_CAMERA_CURVE_REACQUIRE_MIN_YAW_RATE',
    'BOSCH_CAMERA_CURVE_REACQUIRE_LAT_MAX_M',
    'BOSCH_CAMERA_CURVE_REACQUIRE_BEAR_MAX_RAD',
    'BOSCH_CAMERA_CURVE_REACQUIRE_STATE_MAX',
    'BOSCH_TRUCK_P2_CLASS',
    'BOSCH_TRUCK_P2_CONFIRMATIONS',
    'BOSCH_TRUCK_P2_STATE_MAX',
    'BOSCH_TRUCK_P2_WIDTH_MIN_M',
    'BOSCH_TRUCK_P2_DD_MIN_M',
    'BOSCH_TRUCK_P2_DD_MAX_M',
    'BOSCH_TRUCK_P2_DY_MAX_M',
    'BOSCH_TRUCK_P2_DV_MAX_MPS',
    'BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS',
    'BOSCH_TRUCK_A0_RECOVERY_BEARING_EXCESS_RAD',
    'BOSCH_TRUCK_A0_RECOVERY_COST_MARGIN',
    'BOSCH_COMPANION_DEFER_OFF',
    'BOSCH_COMPANION_DEFER_ACTIVE',
    'BOSCH_COMPANION_DEFER_MODE',
    'BOSCH_COMPANION_DEFER_CONFIRMATIONS',
    'BOSCH_COMPANION_DEFER_HOLD_NS',
    'BOSCH_COMPANION_DEFER_STATE_MAX',
    'BOSCH_COMPANION_DEFER_DD_MIN_M',
    'BOSCH_COMPANION_DEFER_DD_MAX_M',
    'BOSCH_COMPANION_DEFER_DY_MAX_M',
    'BOSCH_COMPANION_DEFER_DV_MAX_MPS',
    'BOSCH_COMPANION_DEFER_Y_MAX_M',
    'BOSCH_COMMON_ANCESTRY_MAX_AGE_NS',
    'BOSCH_COMMON_ANCESTRY_RAW_PAIR_MAX',
    'BOSCH_COMPANION_DEFER_WIDTH_DD_M',
    'BOSCH_OEM_NEARER_PUBLICATION_OFF',
    'BOSCH_OEM_NEARER_PUBLICATION_ACTIVE',
    'BOSCH_OEM_NEARER_PUBLICATION_MODE',
    '_bosch_camera_signed',
    '_bosch_camera_range_span',
    '_bosch_camera_long_window',
    'BoschCameraObject',
    'BoschCameraCycleCache',
    '_BoschExtendedHistory',
    '_BoschCurveReacquireHistory',
    '_BoschTruckPairHistory',
    '_BoschExtendedRepresentative',
    'BoschCameraExtendedGrouping',
    'bosch_numpy_linear_sum_assignment',
    '_bosch_integer',
    '_bosch_finite',
    'BoschRawDetection',
    'BoschRawTrackingConfig',
    'BoschRawTrack',
    'BoschRawAssociationDecision',
    '_BoschRawState',
    '_bosch_advance',
    '_bosch_admissible_cycle',
    '_bosch_rectangular_unique',
    '_bosch_component_matching',
    '_bosch_raw_unique_component',
    '_bosch_raw_assignment',
    'BoschRawTrackManager',
    'BoschGroupingConfig',
    'BoschVisionCue',
    'BoschPublishedSurface',
    'BoschPhysicalObject',
    'BoschAssociationDecision',
    '_BoschPairEvidence',
    '_BoschCommonGroupAncestry',
    '_BoschPhysicalState',
    '_BoschProvisionalBundleState',
    'BoschProvisionalBundleDecision',
    '_bosch_group_representative',
    '_bosch_physical_component',
    '_bosch_physical_assignment',
    'BoschObjectGroupManager',
    'BoschPhysicalTracker',
    '_BoschPublicationPassThrough',
    '_BoschFamilyCompanionState',
    'BoschFamilyCompanionDecision',
    '_BoschFamilyCompanionFilter',
    '_BoschBurstMultiReturnState',
    'BoschBurstMultiReturnDecision',
    '_BoschBurstMultiReturnDefer',
    '_BoschSidePassLateralState',
    'BoschSidePassLateralDecision',
    '_BoschSidePassLateralEstimator',
    '_BoschP91PairState',
    '_BoschPersistentSpatialCloneFilter',
    '_BoschOemGateState',
    '_BoschOemIntentTracker',
    '_BoschOemValidationGate',
    'BOSCH_PUBLICATION_ALIAS_START',
    'BOSCH_PUBLICATION_ALIAS_COUNT',
    'BOSCH_PUBLICATION_ALIAS_GRACE_S',
    'BoschPublicationAliasAllocator',
    'BoschB5ModelContext',
    '_BoschB5ParentState',
    'BoschB5Decision',
    'BoschBirthB5Defer',
    'bosch_published_surface',
    'BoschLeadAccelerationState',
    'BoschLeadAccelerationEstimator',
    'bosch_fill_point',
    'bosch_append_points',
    'bosch_to_native_radar_data',
    '_BoschCanFrame',
    'bosch_decode_frame',
    '_bosch_match_processed_target_word',
    'bosch_make_points',
    'BoschRadarProvider',
  )
  assert all(getattr(radar_interface_module, name) is getattr(radar_bosch_module, name) for name in names)


@pytest.mark.parametrize('solver', [radar_bosch_module._bosch_raw_unique_component, radar_bosch_module._bosch_physical_component])
@pytest.mark.parametrize('outcome', ['unique', 'tie', 'exception'])
def test_recursive_matching_releases_closure_with_gc_disabled(solver, outcome):
  refs = []
  previous_profile = sys.getprofile()
  previously_enabled = gc.isenabled()

  def observe(frame, event, arg):
    if event == 'call' and frame.f_code.co_name == 'solve' and frame.f_back.f_code is solver.__code__:
      refs.append(weakref.ref(frame.f_back.f_locals['solve']))
      if outcome == 'exception':
        raise RuntimeError('injected solver failure')

  def run():
    edges = [[(0, .1), (1, .1)], [(0, .1), (1, .05 if outcome == 'unique' else .1)]]
    columns = [[(0, .1), (1, .1)], [(0, .1), (1, edges[1][1][1])]]
    if solver is radar_bosch_module._bosch_raw_unique_component:
      return solver([0, 1], [0, 1], edges, columns, 1.)
    # A unique physical optimum uses the larger score; the same fixture has one.
    return solver([0, 1], [0, 1], edges, columns, 1e-12)

  try:
    gc.disable()
    for _ in range(100):
      sys.setprofile(observe)
      if outcome == 'exception':
        with pytest.raises(RuntimeError, match='injected solver failure'):
          run()
      else:
        result = run()
        assert (result is None) == (outcome == 'tie')
      sys.setprofile(previous_profile)
    assert len(refs) == 100
    assert all(ref() is None for ref in refs)
  finally:
    sys.setprofile(previous_profile)
    if previously_enabled:
      gc.enable()


class TestBoschMrrevo14fDetection:
  CAM, ACAN = 2, 1

  @staticmethod
  def _objects(dlc=8, skip=None):
    return {addr: dlc for addr in range(0x601, 0x613) if addr != skip}

  @pytest.mark.parametrize("candidate", sorted(BOSCH_MRREVO14F_CAR))
  def test_candidates_select_the_camera_bus(self, candidate):
    fingerprint = {0: {0x100: 8}, self.CAM: self._objects()}
    assert bosch_mrrevo14f_bus(candidate, 0, fingerprint, (self.CAM, self.ACAN)) == self.CAM

  def test_acan_wiring_and_camera_preference(self):
    only_acan = {self.ACAN: self._objects()}
    both = {self.CAM: self._objects(), self.ACAN: self._objects()}
    assert bosch_mrrevo14f_bus(CAR.KIA_FORTE, 0, only_acan, (self.CAM, self.ACAN)) == self.ACAN
    assert bosch_mrrevo14f_bus(CAR.KIA_FORTE, 0, both, (self.CAM, self.ACAN)) == self.CAM

  @pytest.mark.parametrize("objects", [dict(skip=0x612), dict(skip=0x601), dict(dlc=7)])
  def test_incomplete_object_set_is_not_bosch(self, objects):
    fingerprint = {self.CAM: self._objects(**objects)}
    assert bosch_mrrevo14f_bus(CAR.HYUNDAI_ELANTRA, 0, fingerprint, (self.CAM, self.ACAN)) is None

  def test_objects_split_across_buses_are_not_bosch(self):
    fingerprint = {self.CAM: {a: 8 for a in range(0x601, 0x60a)}, self.ACAN: {a: 8 for a in range(0x60a, 0x613)}}
    assert bosch_mrrevo14f_bus(CAR.HYUNDAI_ELANTRA, 0, fingerprint, (self.CAM, self.ACAN)) is None

  @pytest.mark.parametrize("candidate", [CAR.HYUNDAI_KONA_HEV, CAR.KIA_NIRO_EV, CAR.HYUNDAI_SONATA])
  def test_other_platforms_stay_off(self, candidate):
    assert candidate not in BOSCH_MRREVO14F_CAR
    assert bosch_mrrevo14f_bus(candidate, 0, {self.CAM: self._objects()}, (self.CAM, self.ACAN)) is None

  def test_canfd_flag_stays_off(self):
    fingerprint = {self.CAM: self._objects()}
    assert bosch_mrrevo14f_bus(CAR.HYUNDAI_ELANTRA, int(HyundaiFlags.CANFD), fingerprint, (self.CAM, self.ACAN)) is None

  def test_candidates_are_not_canfd_platforms(self):
    assert all(not car.config.flags & HyundaiFlags.CANFD for car in BOSCH_MRREVO14F_CAR)
