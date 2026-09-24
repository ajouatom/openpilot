from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.tests.test_scc_lead import make_cs, make_radar, send as send_scc
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.cereal import log


def send_ccnc(monkeypatch, radar, *, enabled=True, stock=None, present=True, with_target=False, model=None, hud_lateral=None):
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: "0"))
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
  cs.modelV2 = model
  if model is not None and isinstance(model, SimpleNamespace):
    model.meta = SimpleNamespace(desire=SimpleNamespace(raw=0), desireState=[])
  cs.adrv_0x161 = cs.adrv_0x200 = cs.adrv_0x1ea = None
  cs.ccnc_0x162 = source if present else None
  if with_target:
    cs.adrv_0x161 = {key: 0 for key in packer.dbc.name_to_msg["ADRV_0x161"].sigs}
    cs.out.latEnabled = True
    cs.out.vehicleNaviAvailable = cs.out.leftBlindspot = cs.out.rightBlindspot = False
    cs.out.leftLaneLine = cs.out.rightLaneLine = 0
    cs.is_metric = True
    cs.trailer_connected = False
  messages = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5,
    SimpleNamespace(enabled=enabled, latActive=True), cs, structs.CarControl().hudControl, 0, False, False, 0, False, 0, 0,
    hud_lateral=hud_lateral,
  )
  assert source == original
  if not present:
    assert messages == []
    return None
  assert len(messages) == (2 if with_target else 1) and messages[-1][0] == 0x162 and messages[-1][2] == 0
  parser = CANParser("hyundai_canfd_generated", [("CCNC_0x162", 20), ("ADRV_0x161", 20)], 0)
  assert 0x162 in parser.update([1_000_000_000, messages])
  values = dict(parser.vl["CCNC_0x162"])
  assert values["CHECKSUM"] == hyundaicanfd.hkg_can_fd_checksum(0x162, None, bytearray(messages[-1][1]))
  assert values["RF_DETECT"] == 3
  for key in ("FF_DETECT_ALT", "FF_DISTANCE_ALT", "FF_LATERAL_ALT", "RF_DETECT_DISTANCE", "RF_DETECT_LATERAL"):
    assert values[key] == pytest.approx(original[key])
  if with_target:
    values["target_values"] = dict(parser.vl["ADRV_0x161"])
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


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("second", [False, True])
def test_curve_display_uses_path_at_selected_lead_distance(monkeypatch, direction, second):
  radar = make_radar(dRel=30, yRel=-2.7 * direction)
  if second:
    radar.leadTwo = radar.leadOne
    radar.leadOne = make_radar(dRel=60).leadOne
  model = log.ModelDataV2.new_message()
  model.position.x = [0, 20, 40, 80]
  model.position.y = [0, direction, 3 * direction, 8 * direction]
  model = model.as_reader()
  cs = make_cs(radar)
  cs.modelV2 = model
  scc = send_scc(cs)
  cc = send_ccnc(monkeypatch, radar, model=model, stock=dict(FF_DETECT=12, FF_DISTANCE=30, FF_LATERAL=0))
  assert scc["ACC_ObjLatPos"] == pytest.approx(0.7 * direction)
  assert cc["FF_LATERAL"] == pytest.approx((0.7 * direction) % 12.8)
  assert cc["FF_DETECT"] == 12
  assert cc["FF_DISTANCE"] == scc["ACC_ObjDist"] == 30


@pytest.mark.parametrize(("x", "y"), [([], []), ([0], [0]), ([0, 20], [0]),
                                       ([0, 20], [0, float("nan")]), ([0, float("inf")], [0, 1]),
                                       ([20, 0], [0, 1]), ([0, 0], [0, 1])])
def test_invalid_path_retains_uncompensated_display(monkeypatch, x, y):
  radar = make_radar(yRel=-1.2)
  model = SimpleNamespace(position=SimpleNamespace(x=x, y=y))
  cs = make_cs(radar)
  cs.modelV2 = model
  assert send_scc(cs)["ACC_ObjLatPos"] == pytest.approx(1.2)
  assert send_ccnc(monkeypatch, radar, model=model)["FF_LATERAL"] == pytest.approx(1.2)


@pytest.mark.parametrize(("first", "second", "expected"), [
  ({"dRel": 20.0}, {"dRel": 9.6, "yRel": 0.3, "vRel": -2.1}, 2),
  ({"dRel": 9.6}, {"dRel": 20.0}, 1),
  ({"dRel": 9.6}, {"dRel": 9.6, "yRel": 0.3}, 1),
  ({"status": False}, {"dRel": 9.6}, 2),
  ({"dRel": float("nan")}, {"dRel": 9.6}, 2),
  ({"dRel": 20.0}, {"dRel": 0.0}, 1),
  ({"dRel": 20.0}, {"dRel": 9.6, "yRel": float("nan")}, 1),
  ({"dRel": 20.0}, {"dRel": 9.6, "status": False}, 1),
  ({"status": False}, {"status": False}, 0),
])
def test_all_vehicle_displays_follow_nearest_valid_lead(monkeypatch, first, second, expected):
  radar = make_radar(**first)
  radar.leadTwo = make_radar(**second).leadOne
  cc = send_ccnc(monkeypatch, radar, with_target=True)
  scc = send_scc(make_cs(radar))
  target = cc["target_values"]
  if expected:
    lead = radar.leadOne if expected == 1 else radar.leadTwo
    assert scc["ACC_ObjDist"] == pytest.approx(lead.dRel)
    assert scc["ACC_ObjLatPos"] == pytest.approx(-lead.yRel)
    assert scc["ACC_ObjRelSpd"] == pytest.approx(lead.vRel)
    assert cc["FF_DISTANCE"] == target["TARGET_DISTANCE"] == pytest.approx(lead.dRel)
    assert cc["FF_LATERAL"] == pytest.approx((-lead.yRel) % 12.8)
    assert target["TARGET"] == 1 and target["DISTANCE_LEAD"] == 2
  else:
    assert scc["HUD_LEAD_INFO"] == cc["FF_DETECT"] == target["TARGET"] == target["DISTANCE_LEAD"] == 0
