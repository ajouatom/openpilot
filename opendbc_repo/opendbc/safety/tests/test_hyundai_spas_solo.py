#!/usr/bin/env python3
"""Parked SPAS trial for a car with no stock SPAS sender (HDA1, no APRK).

Uses libsafety directly with hand-built frames, so it does not need the
CAN packer (which imports openpilot here).
"""
import unittest

from opendbc.safety.tests.libsafety import libsafety_py

SAFETY_HYUNDAI_CANFD = 28
SPAS_FLAG = 4096
SPAS1, SPAS2 = 0x165, 0x16A
LEFT, RIGHT = 0x60, 0x80


def crc16(address: int, data: bytes) -> int:
  crc = 0
  for b in list(data[2:]) + [address & 0xFF, (address >> 8) & 0xFF]:
    crc ^= b << 8
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
      crc &= 0xFFFF
  return crc ^ {8: 0x5F29, 16: 0x041D, 24: 0x819D, 32: 0x9F5B}[len(data)]


def canfd(address: int, length: int, counter: int, fill: dict | None = None) -> bytes:
  data = bytearray(length)
  data[2] = counter & 0xFF
  for i, value in (fill or {}).items():
    data[i] = value
  data[0:2] = crc16(address, data).to_bytes(2, 'little')
  return bytes(data)


def stalk(counter: int = 0, left: bool = False) -> bytes:
  """0x3C1 lever frame (idle unless `left`) with the CRC8 (poly 0x1D, xor 0x65) on byte 0."""
  body = bytearray(8)
  body[1] = (counter & 15) << 4
  if left:
    body[3] |= 0x40
  crc = 0
  for b in body[1:]:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  body[0] = crc ^ 0x65
  return bytes(body)


class TestHyundaiSpasSolo(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.assertEqual(self.safety.set_safety_hooks(SAFETY_HYUNDAI_CANFD, SPAS_FLAG), 0)
    self.safety.init_tests()
    self.now = 1_000_000
    self.counter = 0
    self.tx_counter = [0, 0]
    self.wheel = 0
    self.lever_left = False

  def rx(self, address, data, bus=0):
    return self.safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))

  def tx(self, address, data, bus=0):
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))

  def tick(self, ms: int):
    """Parked vehicle inputs at 100 Hz (lever at 5 Hz) for `ms` milliseconds."""
    for _ in range(ms // 10):
      self.now += 10_000
      self.safety.set_timer(self.now)
      self.counter = (self.counter + 1) & 0xFF
      speeds = {8: self.wheel, 10: self.wheel, 12: self.wheel, 14: self.wheel}
      self.rx(0xA0, canfd(0xA0, 24, self.counter, speeds))
      self.rx(0x35, canfd(0x35, 32, self.counter))
      if self.counter % 20 == 0:
        self.rx(0x3C1, stalk(left=self.lever_left))

  def spas(self, direction=LEFT, step_ms=20, counter=None, extra=None):
    self.tick(step_ms)
    sent = []
    for i, (address, length) in enumerate(((SPAS1, 24), (SPAS2, 32))):
      fill = {16: direction} if address == SPAS2 and direction else {}
      fill.update(extra or {})
      c = self.tx_counter[i] if counter is None else counter
      sent.append(bool(self.tx(address, canfd(address, length, c, fill))))
      self.tx_counter[i] = (c + 1) & 0xFF
    return sent

  def test_parked_trial_without_stock_sender(self):
    self.tick(1000)
    self.assertEqual(self.spas(), [True, True])
    for _ in range(20):
      self.assertEqual(self.spas(), [True, True])

  def test_payload_counter_rate_and_direction(self):
    self.tick(1000)
    self.assertEqual(self.spas(), [True, True])
    self.assertEqual(self.spas(extra={20: 1}), [False, False], "only the direction byte may be set")
    self.assertEqual(self.spas(step_ms=10)[1], False, "faster than 15 ms")
    self.setUp()
    self.tick(1000)
    self.assertEqual(self.spas(), [True, True])
    self.assertEqual(self.spas(counter=7), [False, False], "counter must advance by one")
    self.setUp()
    self.tick(1000)
    self.assertEqual(self.spas(LEFT), [True, True])
    self.assertEqual(self.spas(RIGHT)[1], False, "direction is fixed for the trial")

  def test_three_seconds_then_thirty_second_cooldown(self):
    self.tick(1000)
    for _ in range(150):  # arming packet plus 149 more: 2.98 s after arming
      self.assertEqual(self.spas(), [True, True])
    self.assertEqual(self.spas()[1], False, "three-second limit")
    self.tick(10_000)
    self.tx_counter = [0, 0]
    self.assertEqual(self.spas()[1], False, "cooldown")
    self.tick(30_000)
    self.tx_counter = [0, 0]
    self.assertEqual(self.spas(), [True, True], "re-armed after the cooldown")

  def test_stock_sender_appearing_revokes(self):
    self.tick(1000)
    self.assertEqual(self.spas(), [True, True])
    self.rx(SPAS2, canfd(SPAS2, 32, 5))
    self.assertEqual(self.spas()[1], False)

  def test_stock_sender_present_does_not_arm(self):
    self.tick(1000)
    self.rx(SPAS1, canfd(SPAS1, 24, 1))
    self.rx(SPAS2, canfd(SPAS2, 32, 1))
    self.assertEqual(self.spas()[0], False)

  def test_moving_revokes_the_parked_trial(self):
    self.tick(1000)
    self.assertEqual(self.spas(), [True, True])
    self.wheel = 50
    self.assertEqual(self.spas()[1], False, "the parked trial ends when the car moves")

  def test_no_uds_during_trial(self):
    self.tick(1000)
    self.assertEqual(self.spas(), [True, True])
    self.assertFalse(self.tx(0x7B1, bytes([0x02, 0x10, 0x03, 0, 0, 0, 0, 0])))

  def test_disabled_without_flag(self):
    self.assertEqual(self.safety.set_safety_hooks(SAFETY_HYUNDAI_CANFD, 0), 0)
    self.safety.init_tests()
    self.tick(1000)
    self.assertEqual(self.spas()[0], False)


  # --- driving lease (phase 11) and zero-direction tail (phase 12) ---

  def drive(self, ms=500):
    self.wheel = 50  # moving: not the parked trial
    self.tick(ms)

  def test_driving_lease_holds_for_a_lane_change(self):
    self.drive(1000)
    self.assertEqual(self.spas(), [True, True])
    for _ in range(400):  # 8 s, a long lane change
      self.assertEqual(self.spas(), [True, True])

  def test_driving_lease_ends_at_thirty_seconds_with_a_zero_tail(self):
    self.drive(1000)
    for _ in range(1500):  # 30 s
      self.assertEqual(self.spas(), [True, True])
    self.assertEqual(self.spas()[1], False, "direction refused after 30 s")
    self.assertEqual(self.spas(None), [True, True], "zero-direction tail passes")
    self.tick(600)
    self.assertEqual(self.spas(None)[0], False, "tail is 0.5 s")
    self.tick(1000)
    self.tx_counter = [0, 0]
    self.assertEqual(self.spas(), [True, True], "a new lease after the 1 s cooldown")

  def test_driver_lever_revokes_the_direction_but_not_the_tail(self):
    self.drive(1000)
    self.assertEqual(self.spas(), [True, True])
    self.lever_left = True
    # Keep sending at 50 Hz, as the card does, until the next lever frame.
    results = [self.spas()[1] for _ in range(15)]
    self.assertIn(False, results, "direction refused once the lever is used")
    self.assertEqual(self.spas(None), [True, True], "zero-direction tail passes")

  def test_driving_lease_needs_an_idle_lever(self):
    self.lever_left = True
    self.drive(1000)
    self.assertEqual(self.spas()[0], False)

  def test_stock_sender_appearing_while_driving_revokes(self):
    self.drive(1000)
    self.assertEqual(self.spas(), [True, True])
    self.rx(SPAS2, canfd(SPAS2, 32, 5))
    self.assertEqual(self.spas()[1], False)

  def test_tail_refuses_a_direction(self):
    self.drive(1000)
    for _ in range(1500):
      self.spas()
    self.assertEqual(self.spas(None), [True, True])
    self.assertEqual(self.spas(RIGHT)[1], False)

  def test_parked_trial_ends_with_a_zero_tail(self):
    self.tick(1000)
    for _ in range(150):
      self.assertEqual(self.spas(), [True, True])
    self.assertEqual(self.spas()[1], False)
    self.assertEqual(self.spas(None), [True, True], "zero tail after the parked trial")


class TestHyundaiCornerRadar4B9(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.assertEqual(self.safety.set_safety_hooks(SAFETY_HYUNDAI_CANFD, 0), 0)
    self.safety.init_tests()

  def tx(self, data, bus=2):
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x4B9, bus, bytes(data)))

  def test_only_the_card_frames_on_the_camera_bus(self):
    self.assertTrue(self.tx([0, 0, 128, 138, 50, 48, 1, 0]))
    self.assertTrue(self.tx([255] * 8))
    self.assertFalse(self.tx([0, 0, 128, 138, 50, 48, 1, 1]))
    self.assertFalse(self.tx([0] * 8))
    self.assertFalse(self.tx([255] * 8, bus=0))


if __name__ == "__main__":
  unittest.main()
