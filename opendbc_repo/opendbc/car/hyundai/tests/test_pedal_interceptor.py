from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
import time

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car import structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.pedal_interceptor import RAY_PEDAL_INTERCEPTOR_PARAMS, get_ray_pedal_target
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags, get_pedal_interceptor_params
from opendbc.car.hyundai.values import PEDAL_INTERCEPTOR_PARAMS
from opendbc.car.pedal_interceptor import (PEDAL_INTERCEPTOR_DBC, PedalInterceptor,
                                         PedalInterceptorState, create_pedal_interceptor_command)


def car_params(candidate, enabled, fingerprint=None):
  class Settings:
    def get_bool(self, key):
      return enabled if key == "CommaPedalEnabled" else False

    def get_int(self, key):
      return 0

  with patch("opendbc.car.hyundai.interface.Params", Settings):
    return CarInterface.get_params(candidate, fingerprint or gen_empty_fingerprint(), [], False, False, False)


def pedal_fingerprint():
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x201] = 6
  return fingerprint


@pytest.mark.parametrize("candidate", list(CAR))
def test_toggle_preserves_all_unconfigured_vehicle_car_params(candidate):
  fingerprint = pedal_fingerprint()
  try:
    off = car_params(candidate, False, fingerprint)
  except KeyError as error:
    # Some existing platform entries have no torque metadata. Preserve that
    # existing result too; this port must not repair unrelated vehicle support.
    assert error.args == (candidate,)
    with pytest.raises(KeyError) as enabled_error:
      car_params(candidate, True, fingerprint)
    assert enabled_error.value.args == error.args
    return
  on = car_params(candidate, True, fingerprint)
  if candidate == CAR.KIA_RAY_EV:
    assert not off.flags & HyundaiFlags.PEDAL_INTERCEPTOR
    assert on.flags & HyundaiFlags.PEDAL_INTERCEPTOR
  else:
    assert on.to_dict() == off.to_dict()
    assert get_pedal_interceptor_params(on) is None


def test_profile_requires_the_device_but_not_ray_lfa_or_lda_markers():
  CP = car_params(CAR.KIA_RAY_EV, True, pedal_fingerprint())
  assert CP.enableGasInterceptorDEPRECATED
  assert CP.openpilotLongitudinalControl and not CP.pcmCruise
  assert CP.safetyConfigs[-1].safetyParam == (
    HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.LONG |
    HyundaiSafetyFlags.PEDAL_INTERCEPTOR | HyundaiSafetyFlags.PEDAL_INTERCEPTOR_RAY
  )
  for fingerprint in (gen_empty_fingerprint(), {**pedal_fingerprint(), 0: {0x201: 8}}):
    assert not car_params(CAR.KIA_RAY_EV, True, fingerprint).flags & HyundaiFlags.PEDAL_INTERCEPTOR


def test_non_cc_only_vehicle_cannot_activate_even_with_an_explicit_profile(monkeypatch):
  candidate = CAR.HYUNDAI_SONATA
  before = car_params(candidate, False, pedal_fingerprint())
  assert not before.flags & HyundaiFlags.CC_ONLY_CAR
  assert not before.flags & (HyundaiFlags.CANFD | HyundaiFlags.LEGACY | HyundaiFlags.CAMERA_SCC)
  monkeypatch.setitem(PEDAL_INTERCEPTOR_PARAMS, candidate, RAY_PEDAL_INTERCEPTOR_PARAMS)
  enabled = car_params(candidate, True, pedal_fingerprint())
  assert enabled.to_dict() == before.to_dict()
  assert not enabled.enableGasInterceptorDEPRECATED
  assert get_pedal_interceptor_params(enabled) is None
  assert Bus.party not in CarState.__new__(CarState).get_can_parsers(enabled)
  # Reject a stale/inconsistent CP flag too, before parser/controller creation.
  enabled.flags |= HyundaiFlags.PEDAL_INTERCEPTOR.value
  assert get_pedal_interceptor_params(enabled) is None
  assert Bus.party not in CarState.__new__(CarState).get_can_parsers(enabled)


def test_dbc_retains_installed_pedal_scaling_and_disabled_raw_zero():
  packer = CANPacker(PEDAL_INTERCEPTOR_DBC)
  for message, names in (("GAS_COMMAND", ("GAS_COMMAND", "GAS_COMMAND2")),
                         ("GAS_SENSOR", ("INTERCEPTOR_GAS", "INTERCEPTOR_GAS2"))):
    signals = packer.dbc.name_to_msg[message].sigs
    assert [(signals[name].factor, signals[name].offset) for name in names] == [(0.672, -177.408), (0.332, -165.004)]
  command = create_pedal_interceptor_command(packer, 0.1, 3)
  assert command[1][:5] == bytes.fromhex("012e023e83")
  assert create_pedal_interceptor_command(packer, 0.0, 4)[1][:5] == bytes.fromhex("0000000004")


@pytest.mark.parametrize("bus,address", [(0, 0x420), (0, 0x421), (2, 0x420), (2, 0x421)])
def test_existing_scc_cannot_select_the_new_protocol(bus, address):
  fingerprint = pedal_fingerprint()
  fingerprint[bus][address] = 8
  on = car_params(CAR.KIA_RAY_EV, True, fingerprint)
  off = car_params(CAR.KIA_RAY_EV, False, fingerprint)
  assert on.to_dict() == off.to_dict()


def test_parsers_follow_selected_cp_without_global_dbc_map_changes():
  # Calling the parser factory does not require constructing the vehicle state.
  state = CarState.__new__(CarState)
  off = state.get_can_parsers(car_params(CAR.KIA_RAY_EV, False, pedal_fingerprint()))
  on = state.get_can_parsers(car_params(CAR.KIA_RAY_EV, True, pedal_fingerprint()))
  assert set(off) == {Bus.pt, Bus.cam}
  assert set(on) == {Bus.pt, Bus.cam, Bus.party}
  assert on[Bus.party].dbc_name == PEDAL_INTERCEPTOR_DBC and on[Bus.party].bus == 0
  assert on[Bus.pt].dbc_name == off[Bus.pt].dbc_name == "hyundai_kia_generic"


@pytest.mark.parametrize("state", range(6))
def test_real_parser_raw_state_and_driver_override(state):
  parser = CANParser(PEDAL_INTERCEPTOR_DBC, [("GAS_SENSOR", 50)], 0)
  packer = CANPacker(PEDAL_INTERCEPTOR_DBC)
  status = PedalInterceptorState()
  status.update(parser, RAY_PEDAL_INTERCEPTOR_PARAMS)
  assert status.faulted
  now = time.monotonic_ns()
  frame = packer.make_can_msg("GAS_SENSOR", 0, {
    "INTERCEPTOR_GAS": 264 * 0.672 - 177.408, "INTERCEPTOR_GAS2": 497 * 0.332 - 165.004,
    "STATE": state, "COUNTER_PEDAL": 1,
  })
  parser.update([(now, [frame])])
  status.update(parser, RAY_PEDAL_INTERCEPTOR_PARAMS)
  assert status.valid and status.state == state
  assert status.faulted == (state != 0)
  assert not status.gas_pressed
  press = packer.make_can_msg("GAS_SENSOR", 0, {
    "INTERCEPTOR_GAS": 310 * 0.672 - 177.408, "INTERCEPTOR_GAS2": 593 * 0.332 - 165.004,
    "STATE": state, "COUNTER_PEDAL": 2,
  })
  parser.update([(now + 20_000_000, [press])])
  status.update(parser, RAY_PEDAL_INTERCEPTOR_PARAMS)
  assert status.gas_pressed
  parser.update([(now + 220_000_001, [])])
  status.update(parser, RAY_PEDAL_INTERCEPTOR_PARAMS)
  assert status.faulted and not status.valid


def test_protocol_rejects_bad_crc_and_stalled_counter():
  parser = CANParser(PEDAL_INTERCEPTOR_DBC, [("GAS_SENSOR", 50)], 0)
  packer = CANPacker(PEDAL_INTERCEPTOR_DBC)
  frame = packer.make_can_msg("GAS_SENSOR", 0, {"STATE": 0, "COUNTER_PEDAL": 1})
  now = time.monotonic_ns()
  addr, data, bus = frame
  assert parser.update([(now, [frame])]) == {addr}
  bad = data[:-1] + bytes([data[-1] ^ 1])
  assert parser.update([(now + 20_000_000, [(addr, bad, bus)])]) == set()
  assert parser.dat[addr] == data
  for i in range(1, 7):
    parser.update([(now + i * 20_000_000, [frame])])
  assert not parser.can_valid


def test_shared_protocol_accepts_explicit_non_ray_calibration(tmp_path):
  source = Path(__file__).resolve().parents[3] / "dbc/pedal_interceptor.dbc"
  dbc = tmp_path / "pedal_interceptor.dbc"
  dbc.write_text(source.read_text().replace("(0.672,-177.408)", "(1,0)").replace("(0.332,-165.004)", "(2,-10)"))
  profile = replace(RAY_PEDAL_INTERCEPTOR_PARAMS, target=lambda *args: 0.2,
                    dbc_name=str(dbc),
                    driver_thresholds=(20, 30), rate_up=0.2, rate_down=0.1)
  command = PedalInterceptor(CANPacker(profile.dbc_name), profile).update(60, True, 0.0, 0.0, 10.0)
  assert int.from_bytes(command[1][:2], "big") == 51
  assert int.from_bytes(command[1][2:4], "big") == 31
  assert command[1][4] == 0x8F


def test_scheduler_heartbeat_rollover_and_immediate_target_zero():
  pedal = PedalInterceptor(CANPacker(PEDAL_INTERCEPTOR_DBC), RAY_PEDAL_INTERCEPTOR_PARAMS)
  messages = [(frame, pedal.update(frame, True, 1.0, 5.0, 20.0)) for frame in range(68)]
  sent = [(frame, msg) for frame, msg in messages if msg is not None]
  assert [frame for frame, _ in sent] == list(range(0, 68, 4))
  assert [msg[1][4] & 0xF for _, msg in sent] == list(range(16)) + [0]
  # The reference resets directly to zero for overspeed/disable, without a ramp.
  zero = pedal.update(68, True, 1.0, 25.0, 20.0)
  assert zero[1][:4] == bytes(4) and zero[1][4] & 0x80 == 0
  assert pedal.gas_last == 0.0
  assert pedal.update(72, False, 1.0, 5.0, 20.0)[1][:4] == bytes(4)


@pytest.mark.parametrize("accel,speed,target", [(float("nan"), 0.0, 10.0), (1.0, 5.0, 0.0), (1.0, 12.0, 10.0)])
def test_reference_invalid_and_overspeed_targets_release(accel, speed, target):
  assert get_ray_pedal_target(accel, speed, target) == 0.0


@pytest.mark.parametrize("stale_pedal_flag", [False, True])
def test_pedal_init_skips_scc_disable_but_keeps_existing_vehicle_init(stale_pedal_flag):
  class Settings:
    def get_int(self, key):
      return 0

    def put_int(self, key, value):
      pass

  pedal_cp = car_params(CAR.KIA_RAY_EV, True, pedal_fingerprint())
  standard_cp = car_params(CAR.HYUNDAI_ELANTRA, True, pedal_fingerprint())
  standard_cp.openpilotLongitudinalControl = True
  if stale_pedal_flag:
    standard_cp.flags |= HyundaiFlags.PEDAL_INTERCEPTOR.value
  with patch("opendbc.car.hyundai.interface.Params", Settings), patch("opendbc.car.hyundai.interface.disable_ecu") as disable:
    CarInterface.init(pedal_cp, None, None)
    disable.assert_not_called()
    CarInterface.init(standard_cp, None, None)
    disable.assert_called_once_with(None, None, bus=0, addr=0x7D0, com_cont_req=b"\x28\x83\x01")


def test_selected_interface_runs_real_state_controller_and_releases_on_override(monkeypatch):
  fingerprint = pedal_fingerprint()
  fingerprint[2][0x485] = 8

  class Settings:
    def get(self, key, *args, **kwargs):
      return str(fingerprint) if key == "FingerPrints" else ""

    def get_bool(self, key):
      return key in ("CommaPedalEnabled", "ControlsReady")

    def get_int(self, key):
      return {"AutoEngage": 2, "MaxAngleFrames": 89, "SpeedFromPCM": 2,
              "CruiseButtonTest1": 8, "CruiseButtonTest2": 30, "CruiseButtonTest3": 1}.get(key, 0)

    def get_float(self, key):
      return 0.0

    def put_int(self, *args):
      pass

    def put_bool(self, *args):
      pass

  for module in ("interface", "carstate", "carcontroller"):
    monkeypatch.setattr(f"opendbc.car.hyundai.{module}.Params", Settings)
  monkeypatch.setattr("opendbc.car.interfaces.Params", Settings)
  CP = CarInterface.get_params(CAR.KIA_RAY_EV, fingerprint, [], False, False, False)
  interface = CarInterface(CP)
  packer = CANPacker("hyundai_kia_generic")
  pedal_packer = CANPacker(PEDAL_INTERCEPTOR_DBC)
  control = structs.CarControl()
  control.enabled = control.longActive = True
  control.hudControl.setSpeed = 20.0
  control.actuators.accel = 1.0
  now = time.monotonic_ns()

  def tick(frame, *, brake=False, driver=False, fault=0, oem_cc=False, cruise_ready=False, missing_cruise=False):
    frames = []
    for role, parser in interface.can_parsers.items():
      if role == Bus.party:
        frames.append(pedal_packer.make_can_msg("GAS_SENSOR", 0, {
          "INTERCEPTOR_GAS": (310 if driver else 264) * 0.672 - 177.408,
          "INTERCEPTOR_GAS2": (593 if driver else 497) * 0.332 - 165.004, "STATE": fault, "COUNTER_PEDAL": frame & 0xF,
        }))
        continue
      for address in list(parser.addresses):
        values = {}
        if address == 0x394:
          values = {"DriverOverride": 2 if brake else 0}
        elif address == 0x371:
          values = {"Accel_Pedal_Pos": 50}
        frames.append(packer.make_can_msg(address, parser.bus, values))
    # LABEL11 is registered by the selected adapter. CC_Engaged is deliberately
    # observed as a raw bit so the shared DBC and other vehicles stay unchanged.
    frames = [msg for msg in frames if not (msg[0] == 0x592 and msg[2] == 0)]
    if not missing_cruise:
      frames.append((0x592, bytes([0, 0, 0, 0, (0x04 if cruise_ready else 0) | (0x08 if oem_cc else 0), 0, 0, 0]), 0))
    frames.append(packer.make_can_msg("LKAS11", 2, {}))
    state = interface.update([(now + frame * 10_000_000, frames)])
    _, sends = interface.apply(control.as_reader(), now + frame * 10_000_000)
    return state, sends

  for frame in range(208):
    state, sends = tick(frame)
  assert state.cruiseState.available
  assert state.canValid and not state.canTimeout
  assert 0x201 not in interface.can_parsers[Bus.pt].addresses
  assert not state.accFaulted and not state.gasPressed
  assert state.gas > 0.0  # Commanded native gas must not masquerade as driver input.
  state, sends = tick(208)
  assert any(address == 0x200 and data[4] & 0x80 for address, data, _ in sends)
  assert not any(address in (0x420, 0x421, 0x389, 0x50A, 0x7D0) for address, _, _ in sends)

  for first, inputs in ((209, {"cruise_ready": True}), (213, {}), (217, {"oem_cc": True}), (221, {})):
    for frame in range(first, first + 4):
      state, sends = tick(frame, **inputs)
    blocked = bool(inputs)
    assert state.cruiseState.available == (not blocked)
    assert state.cruiseState.enabled == inputs.get("oem_cc", False)
    pedal = next(data for address, data, _ in sends if address == 0x200)
    assert bool(pedal[4] & 0x80) == (not blocked)
    if blocked:
      assert pedal[:4] == bytes(4)
    assert not any(address == 0x4F1 for address, _, _ in sends)

  for first, inputs in ((225, {"brake": True}), (229, {"driver": True}), (233, {"fault": 5})):
    for frame in range(first, first + 4):
      state, sends = tick(frame, **inputs)
    pedal = next(data for address, data, _ in sends if address == 0x200)
    assert pedal[:4] == bytes(4) and pedal[4] & 0x80 == 0
  control.longActive = False
  for frame in range(237, 241):
    _, sends = tick(frame)
  assert next(data for address, data, _ in sends if address == 0x200)[:4] == bytes(4)
  control.longActive = True
  for frame in range(241, 345):
    state, sends = tick(frame, missing_cruise=True)
  assert not state.cruiseState.available
  assert next(data for address, data, _ in sends if address == 0x200)[:4] == bytes(4)
