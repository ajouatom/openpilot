from opendbc.can import CANPacker
from opendbc.car import create_gas_interceptor_command
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


def setup_ray_safety():
  safety = libsafety_py.libsafety
  safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0x9405)
  safety.init_tests()
  return safety


def safety_tx(safety, msg):
  addr, dat, bus = msg
  return safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat))


def test_ray_pedal_tx_requires_controls_allowed_and_valid_crc():
  safety = setup_ray_safety()
  packer = CANPacker("hyundai_kia_ray_pedal")
  assert safety_tx(safety, create_gas_interceptor_command(packer, 0.0, 0))
  assert not safety_tx(safety, create_gas_interceptor_command(packer, 0.1, 1))

  safety.set_controls_allowed(True)
  command = create_gas_interceptor_command(packer, 0.1, 2)
  assert safety_tx(safety, command)
  addr, dat, bus = command
  bad_crc = bytearray(dat)
  bad_crc[-1] ^= 1
  assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, bytes(bad_crc)))


def test_ray_pedal_driver_override_blocks_positive_command():
  safety = setup_ray_safety()
  safety.set_controls_allowed(True)
  packer = CANPacker("hyundai_kia_ray_pedal")
  physical_press = packer.make_can_msg("GAS_SENSOR", 0, {
    "INTERCEPTOR_GAS": (310 - 264) * 0.672,
    "INTERCEPTOR_GAS2": (593 - 497) * 0.332,
    "STATE": 0,
    "COUNTER_PEDAL": 1,
  })
  addr, dat, bus = physical_press
  assert safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
  assert not safety_tx(safety, create_gas_interceptor_command(packer, 0.1, 2))


def test_ray_pedal_safety_mode_and_sensor_crc_are_signature_scoped():
  packer = CANPacker("hyundai_kia_ray_pedal")
  for param in (0, 1, 4, 0x9401):
    safety = libsafety_py.libsafety
    safety.set_safety_hooks(CarParams.SafetyModel.hyundai, param)
    safety.init_tests()
    safety.set_controls_allowed(True)
    assert not safety_tx(safety, create_gas_interceptor_command(packer, 0.1, 1))

  safety = setup_ray_safety()
  physical_rest = bytes.fromhex("01f403d55de8")
  assert safety.safety_rx_hook(libsafety_py.make_CANPacket(0x201, 0, physical_rest))
  bad_crc = bytearray(physical_rest)
  bad_crc[-1] ^= 1
  assert not safety.safety_rx_hook(libsafety_py.make_CANPacket(0x201, 0, bytes(bad_crc)))
