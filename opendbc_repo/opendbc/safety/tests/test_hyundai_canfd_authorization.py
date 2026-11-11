"""Exercise real CAN-FD RX, TX and buffered delivery without a vehicle."""
import os
from pathlib import Path
import shlex
import subprocess

import pytest

from opendbc.can import CANPacker
from opendbc.safety import ALTERNATIVE_EXPERIENCE, DLC_TO_LEN, LEN_TO_DLC
from opendbc.safety.tests.libsafety.libsafety_py import ffi

ffi.cdef('''
int canfd_test_forward(CANPacket_t *pkt);
void canfd_test_set_tick(uint32_t value);
int canfd_test_queue_count(int addr, int bus);
bool canfd_test_inputs_valid(void);
''')


@pytest.fixture(scope="session")
def native(tmp_path_factory):
  safety = Path(__file__).resolve().parents[1]
  output = tmp_path_factory.mktemp('canfd-authorization') / 'safety.so'
  subprocess.run([*shlex.split(os.environ.get('CC', 'cc')), '-shared', '-fPIC', '-O0', '-std=gnu11',
                  f'-I{safety / "board"}', f'-I{safety}', f'-I{safety / "tests"}',
                  f'-I{safety / "tests/libsafety"}', str(Path(__file__).with_suffix('.c').with_name('hyundai_canfd_authorization.c')),
                  '-o', str(output)], check=True)
  return ffi.dlopen(str(output))


class Hooks:
  def __init__(self, lib, param=29, *, always_lateral=False):
    self.lib = lib
    param |= 8192
    self.param = param
    self.pt_bus = 0 if param & 8 or not param & 16 else 1
    self.steer_bus = 0
    self.steer_msg = 'LFA' if param & 4 or not param & 16 else ('LKAS_ALT' if param & 128 else 'LKAS')
    self.packer = CANPacker('hyundai_canfd_generated')
    self.counter = 0
    self.now = 1_000_000
    assert lib.set_safety_hooks(28, param) == 0
    lib.init_tests()
    lib.set_timer(self.now)
    lib.set_alternative_experience(1 | (ALTERNATIVE_EXPERIENCE.HYUNDAI_CANFD_ALWAYS_LATERAL if always_lateral else 0))

  def packet(self, addr, data, bus=0):
    pkt = ffi.new('CANPacket_t *')
    pkt.addr, pkt.bus, pkt.data_len_code = addr, bus, LEN_TO_DLC[len(data)]
    pkt.data = data
    self.lib.can_set_checksum(pkt)
    return pkt

  def packed(self, name, values, bus=0):
    addr, data, bus = self.packer.make_can_msg(name, bus, values)
    return self.packet(addr, data, bus)

  def seed(self, *, button=0, brake=False, gas=False, driver=0):
    self.now += 10_000
    self.lib.set_timer(self.now)
    self.counter += 1
    buttons = 'CRUISE_BUTTONS_ALT' if self.param & 32 else 'CRUISE_BUTTONS'
    frames = [
      ('ACCELERATOR', {'ACCELERATOR_PEDAL': int(gas)}),
      ('TCS', {'DriverBraking': int(brake)}),
      ('WHEEL_SPEEDS', {f'WHEEL_SPEED_{p}': 40 for p in range(1, 5)}),
      ('MDPS', {'STEERING_COL_TORQUE': driver}),
      (buttons, {'CRUISE_BUTTONS': button}),
    ]
    for name, values in frames:
      values['COUNTER'] = self.counter % (16 if name == 'CRUISE_BUTTONS' else 256)
      assert self.lib.safety_rx_hook(self.packed(name, values, self.pt_bus)), name
    if not self.param & 4:
      assert self.lib.safety_rx_hook(self.packed('SCC_CONTROL', {'COUNTER': self.counter, 'ACCMode': 0},
                                               2 if self.param & 8 else self.pt_bus))
    assert self.lib.canfd_test_inputs_valid()

  def engage(self):
    self.seed(button=2)
    self.seed()
    assert self.lib.get_controls_allowed()

  def steer(self, torque=2, request=1, *, bus=None):
    return self.packed(self.steer_msg, {'TORQUE_REQUEST': torque, 'STEER_REQ': request},
                       self.steer_bus if bus is None else bus)

  def accel(self, value, *, raw=None, mode=1):
    return self.packed('SCC_CONTROL', {'aReqRaw': value if raw is None else raw, 'aReqValue': value, 'ACCMode': mode},
                       0 if self.param & 8 or not self.param & 16 else 1)

  def tx(self, pkt):
    return bool(self.lib.safety_tx_hook(pkt))

  def forwarded(self, pkt):
    bus = self.lib.canfd_test_forward(pkt)
    return bus, DLC_TO_LEN[pkt.data_len_code], bytes(pkt.data[0:DLC_TO_LEN[pkt.data_len_code]])


@pytest.fixture
def hooks(native):
  h = Hooks(native)
  h.seed()
  return h


@pytest.mark.parametrize('torque,last,rt,driver,steer_req', [
  (513, 513, 513, 0, 1), (-513, -513, -513, 0, 1),
  (11, 0, 0, 0, 1), (41, 50, 50, -1000, 1),
  (113, 113, 0, 0, 1), (512, 512, 512, -1000, 1), (10, 10, 10, 0, 0),
])
def test_steering_violations_cannot_be_transmitted_or_cached(hooks, torque, last, rt, driver, steer_req):
  hooks.engage()
  hooks.lib.set_desired_torque_last(last)
  hooks.lib.set_rt_torque_last(rt)
  hooks.lib.set_torque_driver(driver, driver)
  assert not hooks.tx(hooks.steer(torque, steer_req))
  assert hooks.lib.canfd_test_queue_count(0x12A, 0) == 0


@pytest.mark.parametrize('torque', [-512, -10, -2, 0, 2, 10, 512])
def test_authorized_steering_boundaries(hooks, torque):
  hooks.engage()
  hooks.lib.set_desired_torque_last(torque)
  hooks.lib.set_rt_torque_last(torque)
  assert hooks.tx(hooks.steer(torque))


@pytest.mark.parametrize('value,raw', [(1, None), (-1, None), (0, 1), (1, 0), (2.51, None), (-4.01, None)])
@pytest.mark.parametrize('mode', [0, 1, 2, 4])
def test_tx_accel_never_creates_permission(hooks, value, raw, mode):
  assert not hooks.lib.get_controls_allowed()
  assert not hooks.tx(hooks.accel(value, raw=raw, mode=mode))
  assert not hooks.lib.get_controls_allowed()
  assert hooks.lib.canfd_test_queue_count(0x1A0, 0) == 0


@pytest.mark.parametrize('mode', [0, 1, 2, 4])
def test_inactive_and_cancel_do_not_create_permission(hooks, mode):
  assert hooks.tx(hooks.accel(0, mode=mode))
  assert not hooks.lib.get_controls_allowed()


@pytest.mark.parametrize('value,allowed', [(-4.01, False), (-4., True), (0, True), (2.5, True), (2.51, False)])
def test_authorized_accel_range(hooks, value, allowed):
  hooks.engage()
  assert hooks.tx(hooks.accel(value)) == allowed


def test_gas_override_blocks_accel_without_disabling_lateral(hooks):
  hooks.engage()
  hooks.seed(gas=True)
  assert hooks.lib.get_controls_allowed()
  assert not hooks.tx(hooks.accel(1))
  assert hooks.tx(hooks.accel(0, mode=2))
  assert hooks.tx(hooks.steer())


@pytest.mark.parametrize('param', [29, 29 | 2048, 29 | 32])
def test_always_lateral_is_explicit_and_requires_fresh_valid_rx(native, param):
  h = Hooks(native, param)
  h.seed()
  assert not h.tx(h.steer())
  h.lib.set_alternative_experience(33)
  assert h.tx(h.steer())
  assert not h.lib.get_controls_allowed()
  assert not h.tx(h.accel(1))
  h.lib.set_timer(h.now + 1_000_001)
  assert not h.tx(h.steer())
  h.seed()
  assert h.tx(h.steer())
  h.lib.canfd_test_set_tick(4314)  # fine clock repeats after a full wrap
  assert not h.tx(h.steer())


def test_always_lateral_cannot_actuate_before_initial_rx_or_after_bad_checksum(native):
  h = Hooks(native, always_lateral=True)
  assert not h.tx(h.steer())
  h.seed()
  assert h.tx(h.steer())
  invalid = h.packed('TCS', {'DriverBraking': 0}, 0)
  invalid.data[0] ^= 1
  assert not h.lib.safety_rx_hook(invalid)
  assert not h.tx(h.steer())


@pytest.mark.parametrize('size', [0, 8, 12, 24, 32, 64])
def test_wrong_length_does_not_poison_queue_or_forward(hooks, size):
  hooks.engage()
  for _ in range(2):
    assert not hooks.tx(hooks.packet(0x12A, bytes(size)))
  assert hooks.lib.canfd_test_queue_count(0x12A, 0) == 0
  stock = hooks.steer(0, 0, bus=2)
  original = bytes(stock.data[0:16])
  assert hooks.forwarded(stock) == (0, 16, original)


@pytest.mark.parametrize('relay,extended,bus', [(True, False, 0), (False, True, 0), (False, False, 3)])
def test_other_rejected_headers_have_no_buffer_side_effect(hooks, relay, extended, bus):
  hooks.engage()
  hooks.lib.set_relay_malfunction(relay)
  pkt = hooks.steer(bus=bus)
  pkt.extended = extended
  assert not hooks.tx(pkt)
  assert hooks.lib.canfd_test_queue_count(0x12A, 0) == 0


def test_valid_replacement_reaches_forwarding(hooks):
  hooks.engage()
  for _ in range(2):
    assert hooks.tx(hooks.steer(2))
  forwarded = hooks.steer(0, 0, bus=2)
  assert hooks.lib.canfd_test_forward(forwarded) == 0
  assert (((forwarded.data[6] & 15) << 7) | (forwarded.data[5] >> 1)) - 1024 == 2
  assert DLC_TO_LEN[forwarded.data_len_code] == 16


@pytest.mark.parametrize('cause', ['cancel', 'brake', 'gas', 'stale'])
def test_queued_accel_revocation_and_reengagement_do_not_replay_old_command(hooks, cause):
  hooks.engage()
  for _ in range(2):
    assert hooks.tx(hooks.accel(1))
  if cause == 'stale':
    hooks.lib.set_timer(hooks.now + 1_000_001)
  else:
    hooks.seed(button=4 if cause == 'cancel' else 0, brake=cause == 'brake', gas=cause == 'gas')
  original = hooks.accel(0)
  original.bus = 2
  expected = bytes(original.data[0:32])
  assert hooks.forwarded(original) == (0, 32, expected)
  assert hooks.lib.canfd_test_queue_count(0x1A0, 0) == 0
  hooks.engage()
  original = hooks.accel(0)
  original.bus = 2
  expected = bytes(original.data[0:32])
  assert hooks.forwarded(original) == (0, 32, expected)


def test_cancel_purges_steering_but_independent_lateral_remains_available(hooks):
  hooks.engage()
  for _ in range(2):
    assert hooks.tx(hooks.steer(2))
  hooks.seed(button=4)
  assert hooks.lib.canfd_test_queue_count(0x12A, 0) == 0
  assert not hooks.tx(hooks.steer())
  hooks.lib.set_alternative_experience(33)
  assert hooks.tx(hooks.steer())
  assert not hooks.tx(hooks.accel(1))


def test_main_press_revokes_longitudinal_permission_until_new_driver_set(hooks):
  hooks.engage()
  assert hooks.tx(hooks.accel(1))
  for counter, main in ((4, 1), (5, 0)):
    assert hooks.lib.safety_rx_hook(hooks.packed('CRUISE_BUTTONS', {
      'COUNTER': counter, 'ADAPTIVE_CRUISE_MAIN_BTN': main, 'CRUISE_BUTTONS': 0}, 0))
  assert not hooks.lib.get_controls_allowed()
  assert not hooks.tx(hooks.accel(1))
  hooks.lib.set_alternative_experience(33)
  assert hooks.tx(hooks.steer())
  hooks.counter = 5
  hooks.engage()
  assert hooks.tx(hooks.accel(1))


def test_valid_torque_cut_sequence_preserves_eps_fault_avoidance(hooks):
  hooks.engage()
  for _ in range(89):
    hooks.now += 10_000
    hooks.lib.set_timer(hooks.now)
    if _ % 20 == 0:
      hooks.seed()
    assert hooks.tx(hooks.steer(2, 1))
  assert hooks.tx(hooks.steer(2, 0))
  assert hooks.tx(hooks.steer(2, 0))
  assert not hooks.tx(hooks.steer(2, 0))


def test_received_wrong_bus_or_length_cannot_enable_controls(hooks):
  for bus, size in ((3, 8), (0, 16), (2, 8)):
    pressed = hooks.packed('CRUISE_BUTTONS', {'CRUISE_BUTTONS': 2, 'COUNTER': 2}, bus)
    pressed.data_len_code = LEN_TO_DLC[size]
    hooks.lib.safety_rx_hook(pressed)
    released = hooks.packed('CRUISE_BUTTONS', {'CRUISE_BUTTONS': 0, 'COUNTER': 3}, bus)
    released.data_len_code = LEN_TO_DLC[size]
    hooks.lib.safety_rx_hook(released)
    assert not hooks.lib.get_controls_allowed()
    assert not hooks.tx(hooks.accel(1))


@pytest.mark.parametrize('message,size', [('LKAS', 16), ('LKAS_ALT', 32), ('LFA_ALT', 24)])
def test_other_steering_encodings_cannot_bypass_strict_torque_checks(hooks, message, size):
  hooks.engage()
  pkt = hooks.packed(message, {}, 0)
  assert DLC_TO_LEN[pkt.data_len_code] == size
  assert not hooks.tx(pkt)


@pytest.mark.parametrize('button', [0, 1, 2, 3, 4, 5, 6, 7])
@pytest.mark.parametrize('engaged', [False, True])
def test_button_permissions_do_not_grant_or_bypass_longitudinal_control(hooks, button, engaged):
  if engaged:
    hooks.engage()
  expected = button in (0, 4) or (engaged and button in (1, 2, 3))
  assert hooks.tx(hooks.packed('CRUISE_BUTTONS', {'CRUISE_BUTTONS': button}, 2)) == expected
  assert bool(hooks.lib.get_controls_allowed()) == engaged
