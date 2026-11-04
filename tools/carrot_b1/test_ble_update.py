import asyncio
import hashlib
import hmac
import struct
import pytest

from tools.carrot_b1.ble_update import META, install  # noqa: TID251


class Board:
  def __init__(self):
    self.offset = 0
    self.ops = []

  async def read_gatt_char(self, uuid):
    if uuid == META:
      return struct.pack('<I', 71)
    return struct.pack('<BI', 2 if self.ops[-1] == 3 else 1, self.offset)

  async def write_gatt_char(self, uuid, data, response):
    self.ops.append(data[0])
    if data[0] in (1, 4):
      assert hmac.compare_digest(data[41:], hmac.digest(bytes(32), data[:41], 'sha256'))
      assert data[9:41] == hashlib.sha256(IMAGE).digest()
    elif data[0] == 2:
      assert struct.unpack_from('<I', data, 1)[0] == self.offset
      self.offset += len(data)-5


IMAGE = bytes([0xe9]) + bytes(1000)


@pytest.mark.parametrize('guard,quiet,expected', [(None,False,1), (lambda:None,True,1), (lambda:None,False,4)])
def test_maintenance_manifest(guard, quiet, expected):
  board = Board()
  asyncio.run(install(board, IMAGE, bytes(32), transfer_check=guard, quiet_bus=quiet))
  assert board.ops == [expected, 2, 2, 2, 3]
  assert board.offset == len(IMAGE)


def test_moving_aborts_before_commit():
  board = Board()
  def parked():
    if board.offset:
      raise RuntimeError('left park')
  with pytest.raises(RuntimeError, match='left park'):
    asyncio.run(install(board, IMAGE, bytes(32), parked_check=parked))
  assert board.ops == [4, 2]
