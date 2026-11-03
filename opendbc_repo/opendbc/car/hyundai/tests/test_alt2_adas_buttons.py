from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags


def state():
  return SimpleNamespace(
    cruise_buttons_alt2={"LFA_BTN": 0, "CRUISE_BUTTONS": 0},
    lfahda_cluster={"HDA_LFA_SymSta": 0, "HDA_CntrlModSta": 0},
    MainMode_ACC=False, ACCMode=0, scc_control={"InfoDisplay": 0},
    out=SimpleNamespace(vEgo=20., brakeHoldActive=False, parkingBrake=False),
  )


def request(cs, frame, enabled=True, stopping=False):
  address, data, bus = hyundaicanfd.create_alt2_adas_button_request(
    CANPacker("hyundai_canfd_generated"), SimpleNamespace(CAM=2), frame,
    SimpleNamespace(enabled=enabled), cs, stopping,
  )
  assert (address, bus, len(data)) == (0x10B, 2, 16)
  assert int.from_bytes(data[:2], "little") == hyundaicanfd.hkg_can_fd_checksum(address, None, bytearray(data))
  assert not any(data[2:10] + data[11:])  # intent only; Panda supplies the actual frame
  return data[10]


def test_retry_sequence_has_release_between_lfa_and_scc():
  cs = state()
  values = [request(cs, frame) for frame in range(0, 400, 2)]
  for offset in (0, 100):
    assert values[offset:offset + 15] == [0, 128, 128, 128, 128, 128, 0, 0, 0, 0, 0, 8, 8, 8, 0]


@pytest.mark.parametrize("stock_lfa", (1, 2, 3))
def test_lfa_retry_stops_when_stock_lfa_is_not_off(stock_lfa):
  cs = state()
  cs.lfahda_cluster["HDA_LFA_SymSta"] = stock_lfa
  assert request(cs, 2) == 0


def test_unknown_stock_lfa_is_not_treated_as_off():
  cs = state()
  cs.lfahda_cluster = None
  assert request(cs, 2) == 0


@pytest.mark.parametrize("key,value", (("LFA_BTN", 1), ("CRUISE_BUTTONS", 1), ("CRUISE_BUTTONS", 2),
                                      ("CRUISE_BUTTONS", 4), ("CRUISE_BUTTONS", 8), ("CRUISE_BUTTONS", 15)))
def test_driver_buttons_suppress_automatic_requests(key, value):
  cs = state()
  cs.cruise_buttons_alt2[key] = value
  assert request(cs, 2) == 0
  assert request(cs, 22) == 0


@pytest.mark.parametrize("mode", (1, 2))
def test_active_scc_is_not_toggled_off_when_hda_is_off(mode):
  cs = state()
  cs.MainMode_ACC, cs.ACCMode = True, mode
  assert not any(request(cs, f) & 15 for f in range(0, 1000, 2))


@pytest.mark.parametrize("mode", (0, 4))
def test_scc_main_on_but_cancelled_requests_set(mode):
  cs = state()
  cs.MainMode_ACC, cs.ACCMode = True, mode
  assert request(cs, 22) == 2


@pytest.mark.parametrize("interlock", ("brakeHoldActive", "parkingBrake"))
def test_interlock_blocks_scc_activation(interlock):
  cs = state()
  setattr(cs.out, interlock, True)
  assert request(cs, 22) == 0


def test_scc_requires_enabled_and_speed():
  cs = state()
  assert request(cs, 22, enabled=False) == 0
  cs.out.vEgo = 3.
  assert request(cs, 22) == 0


def test_standstill_resume_retains_existing_stopping_guard():
  cs = state()
  cs.MainMode_ACC, cs.ACCMode = True, 1
  cs.scc_control["InfoDisplay"] = 4
  assert request(cs, 12) == 2
  assert request(cs, 12, stopping=True) == 0


@pytest.mark.parametrize("alt2", (False, True))
def test_ccnc_routes_button_request_to_observed_format(monkeypatch, alt2):
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: SimpleNamespace(get_int=lambda key: 0))
  cs = state()
  if not alt2:
    cs.cruise_buttons_alt2 = None
  cs.cruise_buttons_msg = {"NORMAL_CRUISE_MAIN_BTN": 0, "LFA_BTN": 0, "CRUISE_BUTTONS": 0}
  cs.cruise_btns_msg_canfd = "CRUISE_BUTTONS_ALT"
  cs.modelV2 = cs.adrv_0x161 = cs.adrv_0x200 = cs.adrv_0x1ea = cs.ccnc_0x162 = None
  msgs = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC.value), CANPacker("hyundai_canfd_generated"),
    SimpleNamespace(CAM=2, ECAN=0), 2, SimpleNamespace(enabled=True, latActive=True), cs,
    SimpleNamespace(), 0, False, False, 0, False, 0, 0,
  )
  assert len(msgs) == 1
  assert msgs[0][0] == (0x10B if alt2 else 0x1AA)
  assert msgs[0][2] == 2


def cluster_main_request(monkeypatch, *, alt2=True, longitudinal=True, corner=2,
                         camera=True, main=False, mode=0, driver=None, **out_values):
  from opendbc.car import structs
  from opendbc.can.parser import get_raw_value

  # AutoEngage and AutoCruiseControl remain zero in this normal entry-point test.
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: SimpleNamespace(get_int=lambda key: 0))
  cs = state()
  cs.out = structs.CarState(canValid=True, gearShifter="drive", vEgo=20.)
  for key, value in out_values.items():
    setattr(cs.out, key, value)
  cs.MainMode_ACC, cs.ACCMode = main, mode
  cs.scc_control["InfoDisplay"] = 4
  cs.modelV2 = cs.adrv_0x161 = cs.adrv_0x200 = cs.adrv_0x1ea = cs.ccnc_0x162 = None
  cs.cruise_buttons_msg = {"NORMAL_CRUISE_MAIN_BTN": 0, "ADAPTIVE_CRUISE_MAIN_BTN": 0,
                           "LFA_BTN": 0, "CRUISE_BUTTONS": 0}
  cs.cruise_btns_msg_canfd = "CRUISE_BUTTONS_ALT"
  if driver:
    cs.cruise_buttons_msg.update(driver)
    cs.cruise_buttons_alt2.update(driver)
  if not alt2:
    cs.cruise_buttons_alt2 = None
  packer = CANPacker("hyundai_canfd_generated")
  cp = structs.CarParams(flags=int(HyundaiFlags.CAMERA_SCC) if camera else 0,
                        openpilotLongitudinalControl=longitudinal)
  frames = []
  for frame in range(0, 400, 2):
    messages = hyundaicanfd.create_ccnc_messages(
      cp, packer, SimpleNamespace(CAM=2, ECAN=0), frame,
      SimpleNamespace(enabled=False, latActive=False), cs, SimpleNamespace(),
      0, False, False, corner, False, 0, 0,
    )
    for address, data, bus in messages:
      if address not in (0x10B, 0x1AA):
        continue
      assert bus == 2
      assert int.from_bytes(data[:2], "little") == hyundaicanfd.hkg_can_fd_checksum(address, None, bytearray(data))
      name = "CRUISE_BUTTONS_ALT2" if alt2 else "CRUISE_BUTTONS_ALT"
      values = {key: get_raw_value(data, sig) for key, sig in packer.dbc.name_to_msg[name].sigs.items()}
      frames.append((frame, values))
  return frames


@pytest.mark.parametrize("alt2", (False, True))
def test_cluster_main_activates_with_carrot_disabled(monkeypatch, alt2):
  frames = cluster_main_request(monkeypatch, alt2=alt2)
  if alt2:
    asserted = [frame for frame, values in frames if values["CRUISE_BUTTONS"] == 8]
    assert asserted == [22, 24, 26, 222, 224, 226]
  else:
    asserted = [frame for frame, values in frames if values["ADAPTIVE_CRUISE_MAIN_BTN"]]
    assert asserted == [12, 14, 16, 212, 214, 216]


@pytest.mark.parametrize("alt2", (False, True))
@pytest.mark.parametrize("blocked", (
  {"longitudinal": False}, {"camera": False}, {"corner": 0},
  {"canValid": False}, {"gearShifter": "park"}, {"gearShifter": "reverse"},
  {"vEgo": 3.}, {"vEgo": float("nan")}, {"vEgo": float("inf")}, {"brakePressed": True},
  {"brakeHoldActive": True}, {"parkingBrake": True},
))
def test_cluster_main_requires_replaced_scc_and_valid_driving_state(monkeypatch, alt2, blocked):
  frames = cluster_main_request(monkeypatch, alt2=alt2, **blocked)
  assert all(values["CRUISE_BUTTONS"] == 0 and not values.get("ADAPTIVE_CRUISE_MAIN_BTN", 0)
             for _, values in frames)


@pytest.mark.parametrize("alt2", (False, True))
@pytest.mark.parametrize("mode", (0, 1, 2, 4))
def test_cluster_main_does_not_toggle_or_resume_existing_main(monkeypatch, alt2, mode):
  frames = cluster_main_request(monkeypatch, alt2=alt2, main=True, mode=mode)
  assert all(values["CRUISE_BUTTONS"] == 0 and not values.get("ADAPTIVE_CRUISE_MAIN_BTN", 0)
             for _, values in frames)


@pytest.mark.parametrize("alt2,button", [(False, b) for b in (1, 2, 4)] + [(True, b) for b in (1, 2, 4, 8, 15)])
def test_cluster_main_defers_to_driver_buttons(monkeypatch, alt2, button):
  frames = cluster_main_request(monkeypatch, alt2=alt2, driver={"CRUISE_BUTTONS": button})
  assert all(values["CRUISE_BUTTONS"] == (0 if alt2 else button)
             and not values.get("ADAPTIVE_CRUISE_MAIN_BTN", 0) for _, values in frames)
