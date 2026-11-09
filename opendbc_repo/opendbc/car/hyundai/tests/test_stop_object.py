from types import SimpleNamespace as NS

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai.hyundaicanfd import create_acc_control_scc2
from opendbc.car.hyundai.stop_object import signal_stop_object_requested
from opendbc.car.hyundai.stopping import CanfdStopping
from opendbc.car.hyundai.tests.test_stopping import make_cs
from opendbc.car.hyundai.values import HyundaiFlags


def send(cs, controller, *, requested=True, stopping=True, enabled=True, override=False):
  hud = NS(leadDistanceBars=2, signalStopObject=requested)
  msg, _ = create_acc_control_scc2(
    CANPacker("hyundai_canfd_generated"),
    NS(ECAN=0),
    enabled,
    -0.5,
    -0.5,
    stopping,
    override,
    30.0,
    hud,
    NS(carrot_cruise=0, jerk_u=2.0, jerk_l=1.0),
    cs,
    controller,
  )
  parser = CANParser("hyundai_canfd_generated", [("SCC_CONTROL", 50)], 0)
  parser.update([1_000_000_000, [msg]])
  return dict(parser.vl["SCC_CONTROL"]), msg[1]


def test_final_stop_object_adds_only_object_fields_and_crc():
  cs = make_cs()
  before = dict(cs.scc_control)
  old, old_bytes = send(cs, CanfdStopping(), requested=False)
  controller = CanfdStopping()
  new, new_bytes = send(cs, controller)
  assert controller.stop_object_active
  assert (new["ACCMode"], new["StopReq"], new["HUD_LEAD_INFO"]) == (1, 1, 2)
  assert new["ACC_ObjDist"] == 3.0
  assert new["ACC_ObjRelSpd"] == pytest.approx(-cs.out.vEgo)
  assert new["ACC_ObjLatPos"] == 0.0
  changed = {k for k in new if new[k] != old[k]}
  assert changed <= {"ACC_ObjDist", "ACC_ObjRelSpd", "CHECKSUM"}
  assert {"ACC_ObjDist", "ACC_ObjRelSpd"} <= changed
  assert old_bytes != new_bytes and cs.scc_control == before
  assert new["aReqRaw"] == new["aReqValue"] == 0.0


@pytest.mark.parametrize(
  "field,value",
  [
    ("gasPressed", True),
    ("brakePressed", True),
    ("parkingBrake", True),
    ("brakeHoldActive", True),
    ("canValid", False),
    ("gearShifter", "reverse"),
    ("vEgo", 1.0),
    ("vEgo", float("nan")),
  ],
)
def test_interlocks_do_not_emit_synthetic_object(field, value):
  cs = make_cs()
  setattr(cs.out, field, value)
  controller = CanfdStopping()
  values, _ = send(cs, controller)
  assert not controller.stop_object_active
  assert values["ACC_ObjDist"] != 3.0


@pytest.mark.parametrize("kwargs", [{"requested": False}, {"stopping": False}, {"enabled": False}, {"override": True}])
def test_object_releases_without_latching(kwargs):
  cs = make_cs()
  controller = CanfdStopping()
  send(cs, controller)
  assert controller.stop_object_active
  values, _ = send(cs, controller, **kwargs)
  assert not controller.stop_object_active and values["ACC_ObjDist"] == pytest.approx(204.6)


@pytest.mark.parametrize("which", ["leadOne", "leadTwo"])
def test_real_lead_wins_and_input_is_never_modified(which):
  cs = make_cs()
  lead = NS(status=True, dRel=8.0, vRel=-0.4, yRel=0.2)
  cs.radarState = NS(leadOne=None, leadTwo=None)
  setattr(cs.radarState, which, lead)
  controller = CanfdStopping()
  values, _ = send(cs, controller)
  assert not controller.stop_object_active
  assert values["ACC_ObjDist"] == 8.0 and values["ACC_ObjRelSpd"] == pytest.approx(-0.4)
  assert lead.dRel == 8.0 and lead.vRel == -0.4


def setup_request():
  cp = structs.CarParams(brand="hyundai", flags=int(HyundaiFlags.CANFD | HyundaiFlags.CAMERA_SCC), openpilotLongitudinalControl=True)
  cs = structs.CarState(canValid=True, gearShifter="drive")
  cc = structs.CarControl(enabled=True, longActive=True)
  plan = NS(shouldStop=True, xState=3, hasLead=False)
  radar = NS(leadOne=NS(status=False), leadTwo=NS(status=False))
  return cp, cs, cc, plan, radar


@pytest.mark.parametrize("xstate,expected", [(0, False), (1, False), (2, False), (3, True), (4, False), (5, True), (6, False)])
def test_request_only_for_signal_stopping_states(xstate, expected):
  cp, cs, cc, plan, radar = setup_request()
  plan.xState = xstate
  assert signal_stop_object_requested(cp, cs, cc, plan, radar, inputs_ready=True) == expected


@pytest.mark.parametrize("block", ["classic", "no_camera", "other_brand", "invalid", "lead", "off", "pedal", "no_stop"])
def test_request_admission_scope(block):
  cp, cs, cc, plan, radar = setup_request()
  ready = True
  if block == "classic":
    cp.flags = int(HyundaiFlags.CAMERA_SCC)
  elif block == "no_camera":
    cp.flags = int(HyundaiFlags.CANFD)
  elif block == "other_brand":
    cp.brand = "toyota"
  elif block == "invalid":
    ready = False
  elif block == "lead":
    radar.leadTwo.status = True
  elif block == "off":
    cc.longActive = False
  elif block == "pedal":
    cs.gasPressed = True
  elif block == "no_stop":
    plan.shouldStop = False
  assert not signal_stop_object_requested(cp, cs, cc, plan, radar, inputs_ready=ready)
