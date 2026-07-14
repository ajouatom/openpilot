import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
import opendbc.car.hyundai.hyundaicanfd as hyundaicanfd
import opendbc.car.hyundai.radar_interface as radar_interface_module
from opendbc.car.hyundai.radar_interface import (
  CORNER_OBJECT_STABLE_TRACK_ID_START,
  RADAR_MSG_COUNT3,
  RADAR_START_ADDR_CANFD3,
  CornerObjectTrackIdManager,
  RadarInterface,
)
from opendbc.car.hyundai.values import HyundaiExtFlags, HyundaiFlags


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
    first_id = manager.get_track_id("corner180", object_id=108, age=240)

    assert first_id == CORNER_OBJECT_STABLE_TRACK_ID_START
    assert manager.get_track_id("corner180", object_id=108, age=241) == first_id
    assert manager.get_track_id("corner235", object_id=108, age=241) != first_id
    assert manager.get_track_id("corner180", object_id=108, age=2) != first_id
