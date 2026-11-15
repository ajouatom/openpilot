from unittest.mock import Mock

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carstate, hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags


MESSAGES = [
  ("LFA", "lfa", 121), ("LFA_ALT", "lfa_alt", 121),
  ("LFAHDA_CLUSTER", "lfahda_cluster", 121),
  ("ADRV_0x161", "adrv_0x161", 122), ("ADRV_0x200", "adrv_0x200", 122),
  ("ADRV_0x1ea", "adrv_0x1ea", 122), ("ADRV_0x160", "adrv_0x160", 122),
  ("CCNC_0x162", "ccnc_0x162", 122),
]


@pytest.fixture
def setup(monkeypatch):
  params = Mock()
  params.get_int.return_value = 0
  params.get_bool.side_effect = lambda key: key == "ControlsReady"
  params.get.return_value = '{0: {}, 1: {}, 2: {}}'
  monkeypatch.setattr(carstate, "Params", lambda: params)
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: params)
  cp = structs.CarParams(carFingerprint=CAR.KIA_EV6,
                        flags=int(HyundaiFlags.CANFD | HyundaiFlags.EV | HyundaiFlags.CANFD_CAMERA_SCC),
                        safetyConfigs=[{}])
  state = carstate.CarState(cp)
  parsers = state.get_can_parsers(cp)
  for parser in parsers.values():
    parser.controls_ready = True
  return state, parsers, CANPacker("hyundai_canfd_generated")


def tick(state, parsers, frame, messages=()):
  for parser in parsers.values():
    parser.update([(frame * 10_000_000, list(messages))])
  state.monitor_fingerprint(parsers, True)


@pytest.mark.parametrize("name,attr,ready", MESSAGES)
def test_message_reappearing_after_startup_is_not_permanently_lost(setup, name, attr, ready):
  state, parsers, packer = setup
  state.controls_ready_count = ready - 1
  tick(state, parsers, ready)
  assert getattr(state, attr) is None
  state.controls_ready_count = carstate.READY_COUNT_OK + 1
  message = packer.make_can_msg(name, parsers[Bus.cam].bus, {"COUNTER": 1})
  tick(state, parsers, 300, [message])
  assert getattr(state, attr) is None  # Discovery alone is not a decoded frame.
  message = packer.make_can_msg(name, parsers[Bus.cam].bus, {"COUNTER": 2})
  tick(state, parsers, 301, [message])
  assert getattr(state, attr) is parsers[Bus.cam].vl[name]
  assert getattr(state, attr)["COUNTER"] == 2
  # Keep the parser's live dictionary and counter/CRC checks, not a frozen copy.
  message = packer.make_can_msg(name, parsers[Bus.cam].bus, {"COUNTER": 3})
  tick(state, parsers, 302, [message])
  assert getattr(state, attr)["COUNTER"] == 3
  assert not parsers[Bus.cam].message_states[message[0]].ignore_counter
  assert not parsers[Bus.cam].message_states[message[0]].ignore_checksum


@pytest.mark.parametrize("name,attr,ready", MESSAGES)
def test_initial_registration_never_exposes_zero_initialized_payload(setup, name, attr, ready):
  state, parsers, packer = setup
  state.controls_ready_count = ready - 1
  message = packer.make_can_msg(name, parsers[Bus.cam].bus, {"COUNTER": 12})
  tick(state, parsers, ready, [message])
  assert getattr(state, attr) is None
  message = packer.make_can_msg(name, parsers[Bus.cam].bus, {"COUNTER": 13})
  tick(state, parsers, ready + 1, [message])
  assert getattr(state, attr)["COUNTER"] == 13


@pytest.mark.parametrize("source", [0, 1, 128, 130])
def test_other_bus_and_tx_echo_do_not_discover_messages(setup, source):
  state, parsers, packer = setup
  state.controls_ready_count = carstate.READY_COUNT_OK + 1
  tick(state, parsers, 300, [packer.make_can_msg("LFA", source, {})])
  assert state.lfa is None
  assert not parsers[Bus.cam].addresses


def test_absent_optional_messages_do_not_add_validity_checks(setup):
  state, parsers, _ = setup
  state.controls_ready_count = carstate.READY_COUNT_OK + 1
  for frame in range(300, 600):
    tick(state, parsers, frame)
  assert not parsers[Bus.cam].addresses
  assert parsers[Bus.cam].can_valid


def test_bad_checksum_then_valid_receive_and_expired_first_frame(setup):
  state, parsers, packer = setup
  state.controls_ready_count = carstate.READY_COUNT_OK + 1
  parser = parsers[Bus.cam]
  message = packer.make_can_msg("LFA", parser.bus, {"COUNTER": 1})
  tick(state, parsers, 300, [message])
  address, data, bus = packer.make_can_msg("LFA", parser.bus, {"COUNTER": 2})
  corrupt = bytes([data[0] ^ 1]) + data[1:]
  tick(state, parsers, 301, [(address, corrupt, bus)])
  assert state.lfa is None
  assert address not in parser.dat
  # A valid but now old frame also cannot initialize a transmit template.
  parser.update([(3_020_000_000, [(address, data, bus)])])
  tick(state, parsers, 400)
  assert state.lfa is None
  tick(state, parsers, 401, [packer.make_can_msg("LFA", bus, {"COUNTER": 3})])
  assert state.lfa["COUNTER"] == 3


def test_bad_counter_cannot_initialize_template(setup):
  state, parsers, packer = setup
  state.controls_ready_count = carstate.READY_COUNT_OK + 1
  parser = parsers[Bus.cam]
  message = packer.make_can_msg("LFA", parser.bus, {"COUNTER": 7})
  tick(state, parsers, 300, [message])
  # Accumulate invalid repeats in one parser update before the cache can bind.
  parser.update([(3_010_000_000, [message] * 8)])
  # There were initially tolerated frames; expire them before attempting bind.
  tick(state, parsers, 400, [message])
  assert state.lfa is None
  for counter in range(8, 14):
    tick(state, parsers, 401 + counter, [packer.make_can_msg("LFA", parser.bus, {"COUNTER": counter})])
  assert state.lfa["COUNTER"] == 13


def test_wrong_dlc_cannot_initialize_template(setup):
  state, parsers, packer = setup
  state.controls_ready_count = carstate.READY_COUNT_OK + 1
  parser = parsers[Bus.cam]
  address, data, bus = packer.make_can_msg("LFA", parser.bus, {"COUNTER": 1})
  tick(state, parsers, 300, [(address, data, bus)])
  tick(state, parsers, 301, [(address, data[:-1], bus)])
  assert state.lfa is None


@pytest.mark.parametrize("name,attr,ready", MESSAGES)
def test_registration_still_waits_for_startup_gate(setup, name, attr, ready):
  state, parsers, packer = setup
  state.controls_ready_count = ready - 2
  tick(state, parsers, ready - 1, [packer.make_can_msg(name, parsers[Bus.cam].bus, {})])
  assert getattr(state, attr) is None
  assert not parsers[Bus.cam].addresses
