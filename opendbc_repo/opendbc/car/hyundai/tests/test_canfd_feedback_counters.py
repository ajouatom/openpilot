import binascii
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car.hyundai import hyundaicanfd


def feedback(packer, name, snapshot):
  bus = SimpleNamespace(CAM=2, ECAN=0)
  if name == "TCS":
    return hyundaicanfd.create_tcs_messages(packer, bus, SimpleNamespace(tcs=snapshot))
  state = SimpleNamespace(mdps=snapshot, adrv_0x161=None, lfa=None, steer_touch_2af=None)
  return hyundaicanfd.create_steering_messages_camera_scc(
    41, packer, None, bus, None, False, 0, state, 0, 0, False,
  )


@pytest.mark.parametrize("name", ["MDPS", "TCS"])
def test_feedback_counter_tracks_transmissions(name):
  packer = CANPacker("hyundai_canfd_generated")
  # Repeated, skipped and wrapped RX values must not reset the independent TX sequence.
  rx_counters = [253, 253, 255, 0, None, 0, 7, 7]
  sent = 0
  for rx in rx_counters:
    snapshot = None if rx is None else {"COUNTER": rx, "CHECKSUM": 1234}
    if snapshot is not None:
      snapshot.update({"STEERING_COL_TORQUE": 12} if name == "MDPS" else
                      {"DriverBraking": 1, "ACC_REQ": 0, "NEW_SIGNAL_1": 0})
    original = None if snapshot is None else snapshot.copy()
    messages = feedback(packer, name, snapshot)
    assert snapshot == original
    if snapshot is None:
      assert not messages
      continue
    assert len(messages) == 1
    address, data, bus = messages[0]
    assert bus == 2
    assert data[2] == (254 + sent) % 256
    # Independently calculate wire CRC, including the new counter.
    crc = binascii.crc_hqx(data[2:] + address.to_bytes(2, "little"), 0)
    crc ^= {24: 0x819D, 32: 0x9F5B}[len(data)]
    assert int.from_bytes(data[:2], "little") == crc
    expected = snapshot.copy()
    expected["COUNTER"] = (254 + sent) % 256
    if name == "TCS":
      expected.update(DriverBraking=0, DriverBrakingLowSens=0, NEW_SIGNAL_20=0,
                      NEW_SIGNAL_11=0, NEW_SIGNAL_1=1)
    assert data == CANPacker("hyundai_canfd_generated").make_can_msg(name, 2, expected)[1]
    sent += 1


def test_feedback_counters_are_independent():
  packer = CANPacker("hyundai_canfd_generated")
  for i in range(6):
    mdps = feedback(packer, "MDPS", {"COUNTER": 100, "STEERING_COL_TORQUE": 0})
    assert mdps[0][1][2] == 101 + i
    if i % 2 == 0:
      tcs = feedback(packer, "TCS", {"COUNTER": 200, "ACC_REQ": 1})
      assert tcs[0][1][2] == 201 + i // 2
