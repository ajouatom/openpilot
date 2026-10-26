import unittest

from opendbc.car.hyundai.palisade_cgw_test import PalisadeCgwTest


class TestPalisadeCgwTest(unittest.TestCase):
  def setUp(self):
    self.sender = PalisadeCgwTest()
    self.now = 1_000_000_000
    self.raw = bytes.fromhex('a5b5e781f9f9a53f')
    self.call(0)

  def call(self, selection=1, **changes):
    args = dict(selection=selection, now=self.now, raw=self.raw,
                timestamps=(self.now,) * 3, parked=True, inactive=True)
    args.update(changes)
    return self.sender.update(**args)

  def test_only_selected_bit_changes(self):
    for direction, index, mask, lever in ((1, 2, 8, 4), (2, 7, 64, 2)):
      self.call(0)
      addr, data, bus = self.call(direction)[0]
      self.assertEqual((addr, bus), (0x541, 0))
      expected = bytearray(self.raw)
      expected[index] |= mask
      expected[5] |= lever
      self.assertEqual(data, bytes(expected))

  def test_lamp_flashes_like_stock(self):
    # stock 10-26 00000e40: 0.35 s on / 0.35 s off, lever bits held throughout
    self.call(0)
    frames = []
    for tick in range(28):
      self.now = 1_000_000_000 + tick * 50_000_000
      frames += self.call(1)
    lamps = [(data[2] >> 3) & 3 for _, data, _ in frames]
    self.assertEqual(lamps, ([1] * 7 + [0] * 7) * 2)
    self.assertTrue(all((data[5] >> 1) & 3 == 2 for _, data, _ in frames))

  def test_rate_duration_and_off_rearm(self):
    sent = 0
    for tick in range(401):
      self.now = 1_000_000_000 + tick * 10_000_000
      sent += len(self.call())
    self.assertEqual(sent, 60)
    self.assertEqual(self.call(2), [])
    self.call(0)
    self.assertEqual(len(self.call(2)), 1)

  def test_gates_latch_until_off(self):
    for change in (dict(parked=False), dict(inactive=False), dict(raw=b''),
                   dict(timestamps=(1, 1, 1)), dict(timestamps=(self.now + 1,) * 3),
                   dict(timestamps=(0,) * 3)):
      self.call(0)
      self.assertEqual(self.call(**change), [])
      self.assertEqual(self.call(), [])
    for index, mask in ((2, 8), (2, 16), (7, 64), (7, 128), (4, 2), (4, 4), (5, 2), (5, 4)):
      self.call(0)
      raw = bytearray(self.raw)
      raw[index] |= mask
      self.assertEqual(self.call(raw=raw), [])
      self.assertEqual(self.call(), [])

  def test_no_startup_request_or_direction_switch(self):
    self.sender = PalisadeCgwTest()
    self.assertEqual(self.call(), [])
    self.call(0)
    self.assertTrue(self.call())
    self.now += 50_000_000
    self.assertEqual(self.call(2), [])
    self.assertEqual(self.call(), [])


if __name__ == '__main__':
  unittest.main()
