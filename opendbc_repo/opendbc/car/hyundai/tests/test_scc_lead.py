from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai.hyundaicanfd import create_acc_control_scc2
from opendbc.car.interfaces import CarInterfaceBase


def make_radar(**overrides):
  lead = dict(status=True, dRel=42.7, yRel=-1.2, vRel=-3.4)
  lead.update(overrides)
  return SimpleNamespace(leadOne=SimpleNamespace(**lead))


def make_cs(radar):
  return SimpleNamespace(
    radarState=radar,
    scc_control={"COUNTER": 47, "ACC_ObjDist": 204.6, "ACC_ObjLatPos": 0.0,
                 "ACC_ObjRelSpd": 239.4, "InfoDisplay": 5},
    softHoldActive=0, paddle_button_prev=0,
    out=SimpleNamespace(aEgo=-0.5, vEgo=15.0, brakeHoldActive=False, parkingBrake=False,
                        cruiseState=SimpleNamespace(available=True)),
  )


def send(cs, enabled=True, hud_lateral=None):
  msg, _ = create_acc_control_scc2(
    CANPacker("hyundai_canfd_generated"), SimpleNamespace(ECAN=0), enabled, -0.5, -0.8, False, False, 80.0,
    SimpleNamespace(leadDistanceBars=2, leadVisible=False),
    SimpleNamespace(carrot_cruise=0, jerk_u=1.0, jerk_l=2.0), cs, hud_lateral=hud_lateral,
  )
  assert msg[0] == 0x1a0 and msg[2] == 0
  parser = CANParser("hyundai_canfd_generated", [("SCC_CONTROL", 50)], 0)
  assert 0x1a0 in parser.update([1_000_000_000, [msg]])
  return dict(parser.vl["SCC_CONTROL"])


@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize(("y_rel", "v_rel", "hud_info"), [(-1.2, -3.4, 2), (1.2, 3.4, 1), (0.0, 0.0, 2)])
def test_bus0_object_uses_lead_one_when_stock_has_no_object(enabled, y_rel, v_rel, hud_info):
  cs = make_cs(make_radar(yRel=y_rel, vRel=v_rel))
  original = cs.scc_control.copy()
  values = send(cs, enabled)
  assert values["ACC_ObjDist"] == pytest.approx(42.7)
  assert values["ACC_ObjLatPos"] == pytest.approx(-y_rel)
  assert values["ACC_ObjRelSpd"] == pytest.approx(v_rel)
  assert values["HUD_LEAD_INFO"] == hud_info
  assert cs.scc_control == original


@pytest.mark.parametrize("radar", [None, make_radar(status=False), make_radar(dRel=0), make_radar(dRel=-1),
                                  make_radar(dRel=float("inf")), make_radar(yRel=float("nan")), make_radar(vRel=float("nan"))])
def test_missing_or_invalid_lead_clears_previous_object(radar):
  cs = make_cs(make_radar())
  previous = send(cs)
  cs.scc_control.update(previous)
  cs.radarState = radar
  values = send(cs)
  assert values["ACC_ObjDist"] == pytest.approx(204.6)
  assert values["ACC_ObjLatPos"] == pytest.approx(0)
  assert values["ACC_ObjRelSpd"] == pytest.approx(239.4)
  assert values["HUD_LEAD_INFO"] == 0
  object_fields = {"ACC_ObjDist", "ACC_ObjLatPos", "ACC_ObjRelSpd", "HUD_LEAD_INFO", "CHECKSUM"}
  assert {k: v for k, v in values.items() if k not in object_fields} == {
    k: v for k, v in previous.items() if k not in object_fields
  }


@pytest.mark.parametrize(("y_rel", "v_rel", "lat", "speed"), [(1000, -1000, -45.6, -170), (-1000, 1000, 5.5, 239.3)])
def test_out_of_range_values_do_not_wrap_into_other_objects(y_rel, v_rel, lat, speed):
  values = send(make_cs(make_radar(dRel=1000, yRel=y_rel, vRel=v_rel)))
  assert values["ACC_ObjDist"] == pytest.approx(204.5)
  assert values["ACC_ObjLatPos"] == pytest.approx(lat)
  assert values["ACC_ObjRelSpd"] == pytest.approx(speed)


def test_apply_passes_radar_to_controller_and_clears_it_when_unavailable():
  cs = make_cs(None)
  interface = SimpleNamespace(CS=cs, CC=SimpleNamespace(update=lambda c, state, now: send(state)))
  radar = make_radar()
  values = CarInterfaceBase.apply(interface, structs.CarControl(), 1_000_000_000, None, radar)
  assert cs.radarState is radar
  assert values["ACC_ObjDist"] == pytest.approx(42.7)
  values = CarInterfaceBase.apply(interface, structs.CarControl(), 1_010_000_000)
  assert cs.radarState is None
  assert values["HUD_LEAD_INFO"] == 0
