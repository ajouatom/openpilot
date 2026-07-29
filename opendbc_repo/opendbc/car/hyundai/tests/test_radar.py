import math

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
import opendbc.car.hyundai.hyundaicanfd as hyundaicanfd
import opendbc.car.hyundai.radar_interface as radar_interface_module
from opendbc.car.hyundai.radar_interface import (
  CORNER_OBJECT_STABLE_TRACK_ID_START,
  RADAR_MSG_COUNT3,
  RADAR_MSG_COUNT4,
  RADAR_START_ADDR_CANFD3,
  CornerObjectTrackIdManager,
  RadarInterface,
  corner_object_position_valid,
  deduplicate_corner_candidates,
)
from opendbc.car.hyundai.values import CAR, HyundaiExtFlags, HyundaiFlags


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
    point = next(point for point in radar_data.points if point.trackId == 38)

    assert point.measured
    assert point.dRel == pytest.approx(53.1)
    assert point.yRel == pytest.approx(-3.0)
    assert point.vRel == pytest.approx(4.4)


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

  def test_430_promotes_supported_neighbor_bins(self, monkeypatch):
    radar_interface = self.build_interface(monkeypatch)
    empty = self.message({})
    supported_bins = self.message({
      6: self.slot_word(1000),
      7: self.slot_word(1004),
    })
    packets = [(addr, supported_bins if addr == 0x436 else empty, 1) for addr in range(0x430, 0x438)]
    packets += [(addr, empty, 1) for addr in range(0x440, 0x448)]

    radar_data = self.update_frames(radar_interface, packets)
    points = {point.trackId: point for point in radar_data.points}

    assert points[300].measured
    assert str(points[300].radarSource) == "corner430"
    assert points[300].dRel == pytest.approx(50.1)
    assert points[300].yRel == pytest.approx(2.0)
    assert points[300].yvRel == 0.0

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

    assert points[300].measured
    assert points[300].yRel == pytest.approx(2.0)
    assert points[300].yvRel == 0.0
