import binascii
import pytest
import random
from unittest.mock import patch

from panda import Panda, PandaDFU
import panda.python.spi as spi_module
from panda.python.spi import (
  SPI_V3_VERSION, PandaSpiException, PandaProtocolMismatch, PandaSpiNackResponse,
  PandaSpiV3RemoteError, SpiDevice, SpiV3Status,
)

pytestmark = [
  pytest.mark.test_panda_types((Panda.HW_TYPE_TRES, ))
]

@pytest.mark.skip("doesn't work, bootloader seems to ignore commands once it sees junk")
def test_dfu_with_spam(p):
  dfu_serial = p.get_dfu_serial()

  # enter DFU
  p.reset(enter_bootstub=True)
  p.reset(enter_bootloader=True)
  assert Panda.wait_for_dfu(dfu_serial, timeout=19), "failed to enter DFU"

  # send junk
  d = SpiDevice()
  for _ in range(9):
    with d.acquire() as spi:
      dat = [random.randint(-1, 255) for _ in range(random.randint(1, 100))]
      spi.xfer(dat)

    # should still show up
    assert dfu_serial in PandaDFU.list()

class TestSpi:
  def _ping(self, mocker, panda):
    # A successful logical request is encoded once even if v3 needs multiple
    # response polls. V2 still has exactly a header and data ACK.
    v3 = panda._handle.protocol_version == SPI_V3_VERSION
    spy = mocker.spy(spi_module, 'spi_v3_encode_frame') if v3 else mocker.spy(panda._handle, '_wait_for_ack')
    panda.health()
    assert spy.call_count == (1 if v3 else 2)
    mocker.stop(spy)

  def test_protocol_version_check(self, p):
    for bootstub in (False, True):
      p.reset(enter_bootstub=bootstub)
      with patch('panda.python.spi.PandaSpiHandle.SUPPORTED_PROTOCOL_VERSIONS', (999, )):
        # list should still work with wrong version
        assert p._serial in Panda.list()

        # connect but raise protocol error
        with pytest.raises(PandaProtocolMismatch):
          Panda(p._serial)

  def test_protocol_version_data(self, p):
    for bootstub in (False, True):
      p.reset(enter_bootstub=bootstub)
      v = p._handle.get_protocol_version()

      uid = binascii.hexlify(v[:12]).decode()
      assert uid == p.get_uid()

      hwtype = v[12]
      assert hwtype == ord(p.get_type())

      bstub = v[13]
      assert bstub == (0xEE if bootstub else 0xCC)
      assert v[14] == p._handle.protocol_version

  def test_all_comm_types(self, mocker, p):
    v3 = p._handle.protocol_version == SPI_V3_VERSION
    spy = mocker.spy(spi_module, 'spi_v3_encode_frame') if v3 else mocker.spy(p._handle, '_wait_for_ack')

    # controlRead + controlWrite
    p.health()
    p.can_clear(0)
    assert spy.call_count == (2 if v3 else 2*2)

    # bulkRead + bulkWrite
    p.can_recv()
    p.can_send(0x123, b"somedata", 0)
    assert spy.call_count == (4 if v3 else 2*4)

  def test_bad_header(self, mocker, p):
    if p._handle.protocol_version == SPI_V3_VERSION:
      encode_frame = spi_module.spi_v3_encode_frame

      def corrupt_magic(frame):
        encoded = encode_frame(frame)
        return bytes([encoded[0] ^ 1]) + encoded[1:]

      with patch('panda.python.spi.spi_v3_encode_frame', side_effect=corrupt_magic):
        with pytest.raises(PandaSpiException):
          p._handle.controlRead(Panda.REQUEST_IN, 0xd2, 0, 0, p.HEALTH_STRUCT.size, timeout=50)
    else:
      with patch('panda.python.spi.SYNC', 0), pytest.raises(PandaSpiNackResponse):
        p._handle.controlRead(Panda.REQUEST_IN, 0xd2, 0, 0, p.HEALTH_STRUCT.size, timeout=50)
    self._ping(mocker, p)

  def test_bad_checksum(self, mocker, p):
    cnt = p.health()['spi_checksum_error_count']
    if p._handle.protocol_version == SPI_V3_VERSION:
      encode_frame = spi_module.spi_v3_encode_frame

      def corrupt_checksum(frame):
        encoded = encode_frame(frame)
        return encoded[:-1] + bytes([encoded[-1] ^ 1])

      with patch('panda.python.spi.spi_v3_encode_frame', side_effect=corrupt_checksum):
        with pytest.raises(PandaSpiException):
          p._handle.controlRead(Panda.REQUEST_IN, 0xd2, 0, 0, p.HEALTH_STRUCT.size, timeout=50)
    else:
      with patch('panda.python.spi.PandaSpiHandle._calc_checksum', return_value=0), pytest.raises(PandaSpiNackResponse):
        p._handle.controlRead(Panda.REQUEST_IN, 0xd2, 0, 0, p.HEALTH_STRUCT.size, timeout=50)
    self._ping(mocker, p)
    assert (p.health()['spi_checksum_error_count'] - cnt) > 0

  def test_non_existent_endpoint(self, mocker, p):
    for _ in range(10):
      ep = random.randint(4, 20)
      expected_exception = PandaSpiV3RemoteError if p._handle.protocol_version == SPI_V3_VERSION else PandaSpiNackResponse
      with pytest.raises(expected_exception) as exc:
        p._handle.bulkRead(ep, random.randint(1, 1000), timeout=50)
      if p._handle.protocol_version == SPI_V3_VERSION:
        assert exc.value.status == SpiV3Status.BAD_ENDPOINT

      self._ping(mocker, p)

      with pytest.raises(expected_exception) as exc:
        p._handle.bulkWrite(ep, b"abc", timeout=50)
      if p._handle.protocol_version == SPI_V3_VERSION:
        assert exc.value.status == SpiV3Status.BAD_ENDPOINT

      self._ping(mocker, p)
