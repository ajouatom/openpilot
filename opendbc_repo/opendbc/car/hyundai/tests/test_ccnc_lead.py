from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.tests.test_scc_lead import make_cs, make_radar, send as send_scc
from opendbc.car.hyundai.values import HyundaiFlags


def send_ccnc(monkeypatch, radar, *, enabled=True, stock=None, present=True):
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: SimpleNamespace(get_int=lambda key: 0))
  packer = CANPacker("hyundai_canfd_generated")
  source = {key: 0 for key in packer.dbc.name_to_msg["CCNC_0x162"].sigs}
  source.update(FF_DISTANCE=204.6, FF_DETECT_ALT=2, FF_DISTANCE_ALT=31.2, FF_LATERAL_ALT=0.7,
                RF_DETECT=12, RF_DETECT_DISTANCE=14.3, RF_DETECT_LATERAL=2.7)
  source.update(stock or {})
  original = source.copy()
  cs = make_cs(radar)
  cs.out.steeringAngleDeg = 0.0
  cs.out.leftBlinker = cs.out.rightBlinker = False
  cs.modelV2 = cs.lfahda_cluster = cs.cruise_buttons_msg = None
  cs.adrv_0x161 = cs.adrv_0x200 = cs.adrv_0x1ea = None
  cs.ccnc_0x162 = source if present else None
  messages = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5,
    SimpleNamespace(enabled=enabled, latActive=True), cs, SimpleNamespace(), 0, False, False, 0, False, 0, 0,
  )
  assert source == original
  if not present:
    assert messages == []
    return None
  assert len(messages) == 1 and messages[0][0] == 0x162 and messages[0][2] == 0
  parser = CANParser("hyundai_canfd_generated", [("CCNC_0x162", 20)], 0)
  assert 0x162 in parser.update([1_000_000_000, messages])
  values = dict(parser.vl["CCNC_0x162"])
  assert values["CHECKSUM"] == hyundaicanfd.hkg_can_fd_checksum(0x162, None, bytearray(messages[0][1]))
  for key in ("FF_DETECT_ALT", "FF_DISTANCE_ALT", "FF_LATERAL_ALT", "RF_DETECT", "RF_DETECT_DISTANCE", "RF_DETECT_LATERAL"):
    assert values[key] == pytest.approx(original[key])
  return values


@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("y_rel", [-1.2, 0.0, 1.2])
def test_ccnc_and_scc_show_same_lead_when_stock_object_is_missing(monkeypatch, enabled, y_rel):
  radar = make_radar(yRel=y_rel)
  values = send_ccnc(monkeypatch, radar, enabled=enabled)
  scc = send_scc(make_cs(radar), enabled)
  assert values["FF_DISTANCE"] == pytest.approx(scc["ACC_ObjDist"])
  # Decode the physical signed lateral position from the unsigned DBC field.
  lateral = values["FF_LATERAL"] - (12.8 if values["FF_LATERAL"] >= 6.4 else 0)
  assert lateral == pytest.approx(scc["ACC_ObjLatPos"])
  assert values["FF_DETECT"] == (4 if enabled else 3)


@pytest.mark.parametrize("radar", [None, make_radar(status=False), make_radar(dRel=-1),
                                  make_radar(dRel=float("nan")), make_radar(yRel=float("inf")), make_radar(vRel=float("nan"))])
def test_missing_or_invalid_lead_hides_stale_ccnc_object(monkeypatch, radar):
  values = send_ccnc(monkeypatch, radar, stock=dict(FF_DETECT=12, FF_DISTANCE=42.7, FF_LATERAL=1.2))
  assert values["FF_DETECT"] == 0
  assert values["FF_DISTANCE"] == pytest.approx(204.6)
  assert values["FF_LATERAL"] == 0


@pytest.mark.parametrize(("distance", "lateral", "expected_type"), [(42.7, 11.6, 12), (60.0, 11.6, 4), (42.7, 2.0, 4)])
def test_motorcycle_class_is_preserved_only_for_matching_stock_object(monkeypatch, distance, lateral, expected_type):
  values = send_ccnc(monkeypatch, make_radar(yRel=1.2), stock=dict(FF_DETECT=12, FF_DISTANCE=distance, FF_LATERAL=lateral))
  assert values["FF_DETECT"] == expected_type
  assert values["FF_DISTANCE"] == pytest.approx(42.7)
  assert values["FF_LATERAL"] == pytest.approx(11.6)


@pytest.mark.parametrize(("y_rel", "raw_lateral"), [(1000, 6.4), (-1000, 6.3)])
def test_ccnc_geometry_bounds_do_not_wrap(monkeypatch, y_rel, raw_lateral):
  values = send_ccnc(monkeypatch, make_radar(dRel=1000, yRel=y_rel))
  assert values["FF_DISTANCE"] == pytest.approx(204.5)
  assert values["FF_LATERAL"] == pytest.approx(raw_lateral)


def test_absent_ccnc_message_is_not_synthesized(monkeypatch):
  send_ccnc(monkeypatch, make_radar(), present=False)
