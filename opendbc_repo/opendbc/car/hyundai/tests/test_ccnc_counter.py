import binascii
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, native
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags


@pytest.mark.parametrize("direct", [False, True])
@pytest.mark.parametrize("backend", ["python", "native"])
def test_ccnc_counter_seed_once_and_advance_only_when_message_generated(monkeypatch, direct, backend):
  if backend == "native" and native.pack is None:
    pytest.skip("native CAN extension is not built")
  if backend == "python":
    monkeypatch.setattr(native, "pack", None)
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: SimpleNamespace(get_int=lambda key: 0))
  packer = CANPacker("hyundai_canfd_generated")
  source = {key: 0 for key in packer.dbc.name_to_msg["CCNC_0x162"].sigs}
  source.update(COUNTER=254, COUNTRY=7)
  state = SimpleNamespace(
    out=SimpleNamespace(brakeHoldActive=False, parkingBrake=False, leftBlinker=False, rightBlinker=False),
    modelV2=None, radarState=None, lfahda_cluster=None, cruise_buttons_msg=None,
    adrv_0x161=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
  )
  cp = SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC | (HyundaiFlags.CANFD_CLUSTER_DIRECT_TX if direct else 0))
  bus = SimpleNamespace(ECAN=0, CAM=2)
  control = SimpleNamespace(latActive=False, enabled=False)

  def send(frame):
    return hyundaicanfd.create_ccnc_messages(cp, packer, bus, frame, control, state,
                                           SimpleNamespace(), 0, False, False, 0, False, 0, 0)

  assert send(0) == []
  assert 0x162 not in packer.counters
  state.ccnc_0x162 = source
  assert send(1) == []  # Not a scheduled 20 Hz transmission; must not seed yet.
  assert 0x162 not in packer.counters

  # Repeated, skipped, backwards and wrapped RX values cannot reseed TX.
  # Run past a full 8-bit wrap, with absent-template and unscheduled calls.
  for sent in range(300):
    source["COUNTER"] = [254, 254, 0, 9, 3, 255, 0][sent % 7]
    original = source.copy()
    [(address, data, target)] = send(sent * 5)
    assert (address, target) == (0x162, 0)
    assert data[2] == (255 + sent) % 256
    assert packer.counters[0x162] == (256 + sent) % 256
    crc = binascii.crc_hqx(data[2:] + address.to_bytes(2, "little"), 0) ^ 0x9F5B
    assert int.from_bytes(data[:2], "little") == crc
    assert source == original
    if sent == 0:
      body = data[3:]
    assert data[3:] == body  # Counter/CRC changes cannot alter display fields.
    assert send(sent * 5 + 1) == []
    state.ccnc_0x162 = None
    assert send(sent * 5 + 5) == []
    state.ccnc_0x162 = source
    assert packer.counters[0x162] == (256 + sent) % 256

  # State is per packer/controller lifetime; another instance seeds afresh.
  packer = CANPacker("hyundai_canfd_generated")
  source["COUNTER"] = 40
  assert send(1500)[0][1][2] == 41
