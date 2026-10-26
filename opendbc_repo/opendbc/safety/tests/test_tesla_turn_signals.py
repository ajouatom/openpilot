import unittest

from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import TeslaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py


def checksum(data):
  data[7] = (0xEC + sum(data[:7])) & 0xFF
  return data


def idle(counter=4):
  return checksum(bytearray([0xA5, 0x8C, 0x61, 0xB4, 0x5A, 0xC3, (counter << 4) | 7, 0]))


def request(direction=1, original=None):
  data = bytearray(idle() if original is None else original)
  data[1] = (data[1] & 0xFC) | direction
  data[2] = (data[2] & 0xE1) | ((4 if direction == 3 else 8) << 1)
  data[6] = (data[6] & 15) | ((((data[6] >> 4) + 1) & 15) << 4)
  return checksum(data)


class TestTeslaTurnSignals(unittest.TestCase):
  TX_MSGS = None

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.reset(TeslaSafetyFlags.AUTO_TURN_SIGNALS)

  def reset(self, flags):
    assert self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, flags) == 0
    self.safety.init_tests()
    self.safety.set_timer(0)

  def rx(self, data=None, bus=1):
    return self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x3E9, bus, idle() if data is None else data))

  def tx(self, data=None, bus=1):
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x3E9, bus, request() if data is None else data))

  def test_independent_permission_for_stock_and_openpilot_longitudinal(self):
    for flags in range(16):
      self.reset(flags)
      self.rx()
      self.assertEqual(bool(self.tx()), bool(flags & TeslaSafetyFlags.AUTO_TURN_SIGNALS))

  def test_requires_real_fresh_template_and_consumes_it(self):
    self.assertFalse(self.tx())
    self.rx(bus=0)
    self.assertFalse(self.tx())
    self.rx()
    self.assertTrue(self.tx())
    self.assertFalse(self.tx())
    self.rx()
    self.safety.set_timer(1_500_001)
    self.assertFalse(self.tx())

  def test_bad_checksum_or_non_idle_rx_never_arms(self):
    bad = idle()
    bad[7] ^= 1
    self.rx(bad)
    self.assertFalse(self.tx())
    self.rx(request())
    self.assertFalse(self.tx())

  def test_reverse_requires_cancel_and_cancel_survives_controls_disallowed(self):
    self.rx()
    self.assertTrue(self.tx())
    self.rx()
    self.assertFalse(self.tx(request(2)))
    self.safety.set_controls_allowed(False)
    self.assertTrue(self.tx(request(3)))
    self.rx()
    self.assertTrue(self.tx(request(2)))

  def test_rejects_unowned_bits_counter_checksum_reason_bus_and_length(self):
    masks = [0xFF, 0xFC, 0xE1, 0xFF, 0xFF, 0xFF, 0x0F]
    for index, mask in enumerate(masks):
      for bit in range(8):
        if mask & (1 << bit):
          self.rx()
          data = request()
          data[index] ^= 1 << bit
          self.assertFalse(self.tx(checksum(data)))
    for index in (6, 7, 2):
      self.rx()
      data = request()
      data[index] ^= 0x10 if index == 6 else 1 if index == 7 else 2
      if index != 7:
        checksum(data)
      self.assertFalse(self.tx(data))
    self.rx()
    self.assertFalse(self.tx(bus=0))
    self.assertFalse(self.tx(request()[:4]))

  def test_session_has_time_and_frame_limits_but_can_still_cancel(self):
    self.rx()
    self.assertTrue(self.tx())
    self.safety.set_timer(60_000_001)
    self.rx()
    self.assertFalse(self.tx())
    self.assertTrue(self.tx(request(3)))
    self.reset(TeslaSafetyFlags.AUTO_TURN_SIGNALS)
    for _ in range(1200):
      self.rx()
      self.assertTrue(self.tx())
    self.rx()
    self.assertFalse(self.tx())
    self.assertTrue(self.tx(request(3)))

  def test_reset_clears_session_and_template(self):
    self.rx()
    self.assertTrue(self.tx())
    self.reset(TeslaSafetyFlags.AUTO_TURN_SIGNALS)
    self.assertFalse(self.tx())
    self.rx()
    self.assertFalse(self.tx(request(3)))

  def test_cancel_retries_are_bounded_when_the_host_loses_tx_echo(self):
    self.rx()
    self.assertTrue(self.tx())
    for _ in range(3):
      self.rx()
      self.assertTrue(self.tx(request(3)))
    self.rx()
    self.assertFalse(self.tx(request(3)))
    self.assertTrue(self.tx())
    self.rx()
    self.assertTrue(self.tx(request(3)))
    self.safety.set_timer(5_000_001)
    self.rx()
    self.assertFalse(self.tx(request(3)))

  def test_extended_returned_and_rejected_packets_cannot_supply_a_template(self):
    for flag in ("extended", "returned", "rejected"):
      self.reset(TeslaSafetyFlags.AUTO_TURN_SIGNALS)
      packet = libsafety_py.make_CANPacket(0x3E9, 1, idle())
      setattr(packet[0], flag, 1)
      self.safety.can_set_checksum(packet)
      self.safety.safety_rx_hook(packet)
      self.assertFalse(self.tx())

  def test_relay_malfunction_and_summon_still_block_lamp_output(self):
    self.rx()
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self.tx())
    self.safety.set_relay_malfunction(False)
    data = bytearray(8)
    data[3] = 3 << 1
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x286, 0, data))
    self.assertFalse(self.tx())
