from panda.tests.libpanda.libpanda_py import load_libpanda_stock, make_CANPacket


libpanda_stock = load_libpanda_stock()


def tesla_checksum(addr, data, checksum_byte):
  checksum = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  return (checksum + sum(value for i, value in enumerate(data) if i != checksum_byte)) & 0xFF


def send_power(state, counters):
  for counter in counters:
    data = bytearray(8)
    data[0] = state << 5
    data[6] = counter << 4
    data[7] = tesla_checksum(0x221, data, 7)
    libpanda_stock.ignition_can_hook(make_CANPacket(0x221, 0, data))


def send_gear(value, counters):
  for counter in counters:
    data = bytearray(8)
    data[1] = counter
    data[2] = value << 5
    data[0] = tesla_checksum(0x118, data, 0)
    libpanda_stock.ignition_can_hook(make_CANPacket(0x118, 0, data))


def test_stock_firmware_preserves_tesla_onroad_without_wake():
  libpanda_stock.ignition_can = False
  libpanda_stock.wake_on_can = False
  libpanda_stock.tesla_power_on_can = False

  send_power(3, (14, 15))
  assert libpanda_stock.tesla_power_on_can
  assert not libpanda_stock.wake_on_can

  send_gear(4, (14, 15))
  assert libpanda_stock.ignition_can


def test_stock_firmware_does_not_wake_in_accessory_mode():
  libpanda_stock.ignition_can = False
  libpanda_stock.wake_on_can = False
  libpanda_stock.tesla_power_on_can = False

  send_power(2, (0, 1))
  assert libpanda_stock.tesla_power_on_can
  assert not libpanda_stock.wake_on_can
  assert not libpanda_stock.ignition_can
