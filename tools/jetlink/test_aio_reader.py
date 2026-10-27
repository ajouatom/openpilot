import errno
import os
import sys

import pytest

from openpilot.selfdrive.modeld.jetlink.aio import Reader, load_library


@pytest.fixture
def library():
  if sys.platform != 'linux':
    pytest.skip('Linux AIO ABI')
  result = load_library()
  assert result is not None
  return result


def test_native_pipeline_preserves_order_and_buffer_ownership(tmp_path, library):
  # Regular files exercise real io_setup/submit/getevents/destroy, including
  # completions already available before the next read call. Endpoint offset=0
  # stays unchanged; test-only offsets make every submitted chunk distinct.
  chunks = [bytes([i % 251]) * 16384 for i in range(80)]
  path = tmp_path / 'chunks'
  path.write_bytes(b''.join(chunks))
  fd = os.open(path, os.O_RDONLY)
  reader = Reader(fd, library, offset_step=16384)
  try:
    output = bytearray(16384)
    retained = []
    for expected in chunks:
      count = reader.read_into(output)
      assert count == len(expected) and output == expected
      retained.append(bytes(output))
    assert retained == chunks
    with pytest.raises(OSError) as exc:
      reader.read_into(output)
    assert exc.value.errno == errno.ENODATA
  finally:
    reader.close()
    reader.close()
    os.close(fd)


def test_native_partial_last_read_and_pending_cleanup(tmp_path, library):
  path = tmp_path / 'short'
  path.write_bytes(b'a' * 16384 + b'b' * 512)
  fd = os.open(path, os.O_RDONLY)
  reader = Reader(fd, library, offset_step=16384)
  try:
    output = bytearray(16384)
    assert reader.read_into(output) == 16384
    assert reader.read_into(output) == 512 and output[:512] == b'b' * 512
  finally:
    reader.close()
    os.close(fd)


def test_native_bad_fd_and_small_buffer_fail_closed(library):
  reader = Reader(-1, library)
  try:
    with pytest.raises(OSError) as exc:
      reader.read_into(bytearray(10))
    assert exc.value.errno == errno.EINVAL
    with pytest.raises(OSError) as exc:
      reader.read_into(bytearray(16384))
    assert exc.value.errno == errno.EBADF
  finally:
    reader.close()


def test_cleanup_failure_keeps_owned_context():
  class Library:
    def jetlink_aio_create(self, *args): return 123
    def jetlink_aio_close(self, *args): return -errno.EBUSY
  reader = Reader(1, Library())
  with pytest.raises(OSError):
    reader.close()
  assert reader.context == 123
