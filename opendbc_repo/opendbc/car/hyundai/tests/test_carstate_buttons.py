from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.carstate import CarState, _get_legacy_button_capabilities
from opendbc.car.hyundai.values import Buttons, HyundaiFlags


def make_carstate(*, lfa_button=False, cruise_button_alt=False, lfa_button_alt=False):
  carstate = CarState.__new__(CarState)
  carstate.HAS_LFA_BUTTON = lfa_button
  carstate.CRUISE_BUTTON_ALT = cruise_button_alt
  carstate.CRUISE_BUTTON_LFA = lfa_button_alt
  return carstate


def make_canfd_main_carstate(*, main_enabled=True, manual_main_off_latched=False, main_mode_acc=True, camera_scc=False):
  carstate = CarState.__new__(CarState)
  carstate.CP = SimpleNamespace(openpilotLongitudinalControl=True, flags=HyundaiFlags.CAMERA_SCC if camera_scc else 0)
  carstate.main_enabled = main_enabled
  carstate.manual_main_off_latched = manual_main_off_latched
  carstate.MainMode_ACC = main_mode_acc
  return carstate


@pytest.mark.parametrize(("fingerprint", "expected"), (
  ({0x4F1: 4}, (False, False, False)),
  ({0x391: 8, 0x4F1: 4}, (True, False, False)),
  ({0x3EF: 8, 0x416: 8}, (False, True, True)),
  ({0x391: 8, 0x3EF: 8}, (True, True, False)),
))
def test_legacy_button_capabilities_follow_fingerprint(fingerprint, expected):
  assert _get_legacy_button_capabilities(fingerprint) == expected


def test_standard_button_messages_do_not_access_missing_alt_messages():
  carstate = make_carstate(lfa_button=True)
  cp = SimpleNamespace(
    vl={"BCM_PO_11": {"LFA_Pressed": 0}},
    vl_all={"CLU11": {
      "CF_Clu_CruiseSwState": [Buttons.SET_DECEL],
      "CF_Clu_CruiseSwMain": [1],
    }},
  )

  assert carstate._get_legacy_cruise_buttons(cp) == [Buttons.SET_DECEL]
  assert carstate._get_legacy_main_buttons(cp) == [1]

  cp.vl["BCM_PO_11"]["LFA_Pressed"] = 1
  assert carstate._get_legacy_cruise_buttons(cp) == [Buttons.LFA_BUTTON]


def test_alt_button_messages_are_used_when_observed():
  carstate = make_carstate(cruise_button_alt=True, lfa_button_alt=True)
  cp = SimpleNamespace(
    vl={
      "CRUISE_BUTTON_ALT": {"CruiseSwState": Buttons.RES_ACCEL},
      "CRUISE_BUTTON_LFA": {"CruiseSwLfa": 0},
    },
    vl_all={"CRUISE_BUTTON_ALT": {"CruiseSwMain": [1]}},
  )

  assert carstate._get_legacy_cruise_buttons(cp) == [Buttons.RES_ACCEL]
  assert carstate._get_legacy_main_buttons(cp) == [1]

  cp.vl["CRUISE_BUTTON_LFA"]["CruiseSwLfa"] = 1
  assert carstate._get_legacy_cruise_buttons(cp) == [Buttons.LFA_BUTTON]


def test_alt_cruise_and_lfa_sources_are_independent():
  carstate = make_carstate(lfa_button=True, cruise_button_alt=True)
  cp = SimpleNamespace(
    vl={
      "BCM_PO_11": {"LFA_Pressed": 0},
      "CRUISE_BUTTON_ALT": {"CruiseSwState": Buttons.CANCEL},
    },
    vl_all={"CRUISE_BUTTON_ALT": {"CruiseSwMain": [0]}},
  )

  assert carstate._get_legacy_cruise_buttons(cp) == [Buttons.CANCEL]
  assert carstate._get_legacy_main_buttons(cp) == [0]

  cp.vl["BCM_PO_11"]["LFA_Pressed"] = 1
  assert carstate._get_legacy_cruise_buttons(cp) == [Buttons.LFA_BUTTON]


def test_canfd_manual_main_off_ignores_delayed_camera_main_state():
  carstate = make_canfd_main_carstate()

  carstate._update_canfd_main_enabled(Buttons.NONE, main_button_released=True)
  assert not carstate.main_enabled
  assert carstate.manual_main_off_latched

  # The report log keeps MainMode_ACC high for about 80 ms after release.
  for _ in range(10):
    carstate._update_canfd_main_enabled(Buttons.NONE, main_button_released=False)
    assert not carstate.main_enabled

  carstate.MainMode_ACC = False
  carstate._update_canfd_main_enabled(Buttons.NONE, main_button_released=False)
  assert not carstate.main_enabled


def test_canfd_main_button_turns_cruise_back_on_and_clears_manual_off():
  carstate = make_canfd_main_carstate(main_enabled=False, manual_main_off_latched=True, main_mode_acc=False)

  carstate._update_canfd_main_enabled(Buttons.NONE, main_button_released=True)

  assert carstate.main_enabled
  assert not carstate.manual_main_off_latched


@pytest.mark.parametrize("button", (Buttons.RES_ACCEL, Buttons.SET_DECEL))
def test_canfd_cruise_enable_button_clears_manual_main_off(button):
  carstate = make_canfd_main_carstate(main_enabled=False, manual_main_off_latched=True, main_mode_acc=False)

  carstate._update_canfd_main_enabled(button, main_button_released=False)

  assert carstate.main_enabled
  assert not carstate.manual_main_off_latched


def test_non_camera_scc_main_still_enables_cruise_without_manual_off():
  carstate = make_canfd_main_carstate(main_enabled=False)

  carstate._update_canfd_main_enabled(Buttons.NONE, main_button_released=False)

  assert carstate.main_enabled


@pytest.mark.parametrize("auto_engage", (0, 1, 2))
def test_camera_scc_automatic_main_preserves_driver_initialization(auto_engage):
  state = make_canfd_main_carstate(main_enabled=auto_engage == 2, camera_scc=True)
  for stock_main in (False, True, True, False, True):
    state.MainMode_ACC = stock_main
    state._update_canfd_main_enabled(Buttons.NONE, main_button_released=False)
    assert state.main_enabled == (auto_engage == 2)


@pytest.mark.parametrize("button", (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.NONE))
def test_camera_scc_driver_can_enable_then_disable_with_oem_main_still_on(button):
  state = make_canfd_main_carstate(main_enabled=False, camera_scc=True)
  state._update_canfd_main_enabled(button, main_button_released=button == Buttons.NONE)
  assert state.main_enabled
  state._update_canfd_main_enabled(Buttons.NONE, main_button_released=True)
  assert not state.main_enabled and state.manual_main_off_latched
  for _ in range(20):
    state._update_canfd_main_enabled(Buttons.NONE, main_button_released=False)
    assert not state.main_enabled
