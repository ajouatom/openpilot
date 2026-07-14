import math

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
import opendbc.car.hyundai.hyundaicanfd as hyundaicanfd
import opendbc.car.hyundai.radar_interface as radar_interface_module
from opendbc.car.hyundai.radar_interface import RADAR_MSG_COUNT3, RADAR_MSG_COUNT4, RADAR_START_ADDR_CANFD3, RadarInterface
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
    assert track["AZIMUTH"] == pytest.approx(-6.5)
    assert track["REL_SPEED"] == pytest.approx(-0.734375)
    assert track["OBJECT_STATE"] == 3

  def test_empty_track(self):
    track = self.parse(0x507, "53fff80000000081")

    assert track["LONG_DIST"] == pytest.approx(409.55)
    assert track["AZIMUTH"] == 0
    assert track["REL_SPEED"] == 0
    assert track["OBJECT_STATE"] == 0

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
    assert radar_interface.radar_msg_count == RADAR_MSG_COUNT4
    assert radar_interface.trigger_msg_tracks == 0x507

    active_dat = bytes.fromhex("bc047efcc1fe8b00")
    empty_dat = bytes.fromhex("bcfff80000000081")
    packets = [(addr, active_dat if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    point = next(point for point in radar_data.points if point.trackId == 35)

    assert point.measured
    assert point.dRel == pytest.approx(math.cos(math.radians(-6.5)) * 7.1875)
    assert point.yRel == pytest.approx(-math.sin(math.radians(-6.5)) * 7.1875)
    assert point.vRel == pytest.approx(-0.734375)
    assert math.isnan(point.aRel)

    # 0x508 and above carry state 0 distance-sorted detections. Even if such a
    # payload appears in a track slot, it must not become a RadarPoint.
    raw_detection = bytes.fromhex("d702f4fc200000e4")
    packets = [(addr, raw_detection if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
    radar_data = radar_interface.update([0, packets])
    assert not radar_data.points

    # Stable tracks well outside the ego/adjacent-lane envelope are roadside
    # reflections and must not create clutter in liveTracks.
    side_reflection = bytes.fromhex("d80b66f640000300")
    packets = [(addr, side_reflection if addr == 0x503 else empty_dat, 1) for addr in range(0x500, 0x508)]
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
