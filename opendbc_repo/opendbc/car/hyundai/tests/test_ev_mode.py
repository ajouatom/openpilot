import math

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai.carstate import CarState, EV_MODE_STATUS_TIMEOUT_NS, _get_ev_mode_state
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CANFD_HYBRID_STATUS_ADDR, CANFD_HYBRID_STATUS_DLC, CAR, DBC, EV_MODE_ACTIVE_VALUES, \
                                       EV_MODE_STATUS_ADDR, EV_MODE_STATUS_DLC, EV_MODE_STATUS_MSG, EV_MODE_STATUS_SIGNAL, \
                                       HyundaiExtFlags, HyundaiFlags
from openpilot.common.params import Params


def get_params(candidate, *, hybrid=True, hybrid_bus=0, hybrid_status=None, hybrid_status_bus=0,
               hybrid_status_dlc=CANFD_HYBRID_STATUS_DLC, status_bus=0, status_dlc=EV_MODE_STATUS_DLC):
  Params().put_int("HyundaiCameraSCC", 1)
  Params().put_int("CanfdHDA2", 1)

  fingerprint = gen_empty_fingerprint()
  if hybrid:
    fingerprint[hybrid_bus][0x105] = 32
  if hybrid_status is None:
    hybrid_status = hybrid
  if hybrid_status:
    fingerprint[hybrid_status_bus][CANFD_HYBRID_STATUS_ADDR] = hybrid_status_dlc
  if status_bus is not None:
    fingerprint[status_bus][EV_MODE_STATUS_ADDR] = status_dlc
  return CarInterface.get_params(candidate, fingerprint, [], False, False, False)


@pytest.mark.parametrize(("candidate", "hybrid", "status_bus", "status_dlc", "expected"), (
  (CAR.HYUNDAI_SANTAFE_MX5_HEV, True, 0, 32, True),
  (CAR.HYUNDAI_SANTAFE_MX5_HEV, False, 0, 32, False),
  (CAR.HYUNDAI_SANTAFE_MX5_HEV, True, None, 32, False),
  (CAR.HYUNDAI_SANTAFE_MX5_HEV, True, 1, 32, False),
  (CAR.HYUNDAI_SANTAFE_MX5_HEV, True, 0, 16, False),
  (CAR.KIA_SORENTO_4TH_GEN, False, None, 32, False),
  # Shared ICE/HEV/PHEV candidates rely on the observed ECAN capability frames, not their model name.
  (CAR.HYUNDAI_TUCSON_4TH_GEN, True, 0, 32, True),
  (CAR.HYUNDAI_TUCSON_4TH_GEN, False, 0, 32, False),
  (CAR.KIA_SORENTO_HEV_4TH_GEN, True, 0, 32, True),
  (CAR.HYUNDAI_KONA_HEV_2ND_GEN, True, 0, 32, True),
  (CAR.HYUNDAI_KONA_HEV_2ND_GEN, False, 0, 32, False),
  (CAR.HYUNDAI_ELANTRA_HEV_2021, True, 0, 32, False),
))
def test_ev_mode_capability(candidate, hybrid, status_bus, status_dlc, expected):
  CP = get_params(candidate, hybrid=hybrid, status_bus=status_bus, status_dlc=status_dlc)
  assert bool(CP.extFlags & HyundaiExtFlags.EV_MODE_STATUS_230) == expected


@pytest.mark.parametrize(("hybrid_status_bus", "hybrid_status_dlc"), (
  (1, CANFD_HYBRID_STATUS_DLC),
  (0, 16),
))
def test_ev_mode_capability_requires_ecan_hybrid_status_dlc32(hybrid_status_bus, hybrid_status_dlc):
  CP = get_params(CAR.HYUNDAI_SANTAFE_MX5_HEV, hybrid_status_bus=hybrid_status_bus,
                  hybrid_status_dlc=hybrid_status_dlc)

  assert not bool(CP.extFlags & HyundaiExtFlags.EV_MODE_STATUS_230)


def test_0x105_and_ev_status_without_0xfa_do_not_enable_ev_mode():
  CP = get_params(CAR.HYUNDAI_TUCSON_4TH_GEN, hybrid=True, hybrid_status=False)

  assert bool(CP.flags & HyundaiFlags.HYBRID)
  assert not bool(CP.extFlags & HyundaiExtFlags.EV_MODE_STATUS_230)


def test_ev_mode_display_capability_does_not_change_hybrid_safety_classification():
  CP = get_params(CAR.HYUNDAI_TUCSON_4TH_GEN, hybrid=False, hybrid_status=True)

  assert not bool(CP.flags & HyundaiFlags.HYBRID)
  assert bool(CP.extFlags & HyundaiExtFlags.EV_MODE_STATUS_230)


def test_ev_mode_capability_uses_the_detected_ecan_offset():
  CP = get_params(CAR.KIA_SORENTO_HEV_4TH_GEN, hybrid_bus=4, hybrid_status_bus=4, status_bus=4)

  assert bool(CP.extFlags & HyundaiExtFlags.EV_MODE_STATUS_230)


def test_ev_mode_parser_registration_is_capability_gated():
  supported = get_params(CAR.HYUNDAI_SANTAFE_MX5_HEV)
  unsupported = get_params(CAR.KIA_SORENTO_4TH_GEN, hybrid=False, status_bus=1, status_dlc=16)

  supported_parser = CarState.get_can_parsers_canfd(None, supported)[Bus.pt]
  unsupported_parser = CarState.get_can_parsers_canfd(None, unsupported)[Bus.pt]

  assert EV_MODE_STATUS_ADDR in supported_parser.addresses
  assert supported_parser.message_states[EV_MODE_STATUS_ADDR].ignore_alive
  assert supported_parser.message_states[EV_MODE_STATUS_ADDR].ignore_counter
  assert EV_MODE_STATUS_ADDR not in unsupported_parser.addresses


def test_sorento_ice_corner_radar_status_does_not_enable_ev_mode():
  Params().put_int("HyundaiCameraSCC", 1)
  Params().put_int("CanfdHDA2", 1)

  fingerprint = gen_empty_fingerprint()
  fingerprint[1][0x230] = 16  # Actual Sorento ICE ACAN corner-radar status frame.
  CP = CarInterface.get_params(CAR.KIA_SORENTO_4TH_GEN, fingerprint, [], False, False, False)
  parser = CarState.get_can_parsers_canfd(None, CP)[Bus.pt]

  assert not bool(CP.extFlags & HyundaiExtFlags.EV_MODE_STATUS_230)
  assert EV_MODE_STATUS_ADDR not in parser.addresses


def test_ev_mode_state_requires_a_fresh_dlc32_frame():
  CP = get_params(CAR.HYUNDAI_SANTAFE_MX5_HEV)
  parser = CarState.get_can_parsers_canfd(None, CP)[Bus.pt]
  packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

  assert _get_ev_mode_state(parser) == (False, False)

  timestamp = 1_000_000_000
  wrong_bus_short_status = (EV_MODE_STATUS_ADDR, b"\x00" * 16, 1)
  parser.update([timestamp, [wrong_bus_short_status]])
  assert _get_ev_mode_state(parser) == (False, False)

  ev_active = packer.make_can_msg(EV_MODE_STATUS_MSG, parser.bus, {"COUNTER": 1, EV_MODE_STATUS_SIGNAL: 6})
  parser.update([timestamp + 100_000_000, [ev_active]])
  assert _get_ev_mode_state(parser) == (True, True)

  ev_inactive = packer.make_can_msg(EV_MODE_STATUS_MSG, parser.bus, {"COUNTER": 2, EV_MODE_STATUS_SIGNAL: 3})
  parser.update([timestamp + 200_000_000, [ev_inactive]])
  assert _get_ev_mode_state(parser) == (False, True)

  parser.dat[EV_MODE_STATUS_ADDR] = b"\x00" * 16
  assert _get_ev_mode_state(parser) == (False, False)

  parser.dat[EV_MODE_STATUS_ADDR] = ev_inactive[1]
  parser.update([timestamp + 200_000_000 + EV_MODE_STATUS_TIMEOUT_NS + 1, []])
  assert not parser.bus_timeout
  assert _get_ev_mode_state(parser) == (False, False)


@pytest.mark.parametrize("mode", range(16))
def test_ev_mode_enum_mapping(mode):
  CP = get_params(CAR.HYUNDAI_SANTAFE_MX5_HEV)
  parser = CarState.get_can_parsers_canfd(None, CP)[Bus.pt]
  packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
  msg = packer.make_can_msg(EV_MODE_STATUS_MSG, parser.bus, {"COUNTER": mode, EV_MODE_STATUS_SIGNAL: mode})

  parser.update([1_000_000_000, [msg]])

  assert int(parser.vl[EV_MODE_STATUS_MSG][EV_MODE_STATUS_SIGNAL]) == mode
  assert _get_ev_mode_state(parser) == (mode in EV_MODE_ACTIVE_VALUES, True)


@pytest.mark.parametrize(("payload", "expected_mode", "expected_active"), (
  ("758139048402000000000000000000000000d3009805001c24000000c05dc80f", 1, True),
  ("4d466f048402000000000000000000000000bf007408001c24000000c05dc80f", 2, True),
  ("32206e048402000000000000000000000000c9007418001c24000000c05dc80f", 6, True),
  # Route 455 mode 3 was a false positive with the old single-bit 0x230 interpretation.
  ("4f935e0484020000000000000000000000004400640d001c10000000c05dc40f", 3, False),
  ("066558048402000000000000000000000000c4003020001c24000000c05dc80f", 8, False),
  ("059683048402000000000000000000000000a1008425001c24000000c05dc80f", 9, False),
  ("011305048402000000000000000000000000d600a829001c24000000c05dc80f", 10, False),
))
def test_ev_mode_dbc_decodes_real_mx5_frames(payload, expected_mode, expected_active):
  parser = CANParser("hyundai_canfd_generated", [(EV_MODE_STATUS_MSG, math.nan)], 0)
  updated = parser.update([1_000_000_000, [(EV_MODE_STATUS_ADDR, bytes.fromhex(payload), 0)]])

  assert updated == {EV_MODE_STATUS_ADDR}
  assert int(parser.vl[EV_MODE_STATUS_MSG][EV_MODE_STATUS_SIGNAL]) == expected_mode
  assert _get_ev_mode_state(parser) == (expected_active, True)


def test_ev_mode_rejects_checksum_corruption():
  CP = get_params(CAR.HYUNDAI_SANTAFE_MX5_HEV)
  parser = CarState.get_can_parsers_canfd(None, CP)[Bus.pt]
  payload = bytearray.fromhex("32206e048402000000000000000000000000c9007418001c24000000c05dc80f")
  payload[-1] ^= 1

  parser.update([1_000_000_000, [(EV_MODE_STATUS_ADDR, bytes(payload), parser.bus)]])

  assert EV_MODE_STATUS_ADDR not in parser.dat
  assert _get_ev_mode_state(parser) == (False, False)


def test_ev_mode_fields_default_invalid():
  state = structs.CarState()
  assert not state.evModeActive
  assert not state.evModeValid


def test_ev_mode_parser_is_optional_for_can_validity():
  CP = get_params(CAR.HYUNDAI_SANTAFE_MX5_HEV)
  parser = CarState.get_can_parsers_canfd(None, CP)[Bus.pt]
  state = parser.message_states[EV_MODE_STATUS_ADDR]

  assert state.ignore_alive
  assert state.ignore_counter
