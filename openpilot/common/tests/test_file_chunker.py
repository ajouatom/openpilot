import builtins
from pathlib import Path

import pytest

from openpilot.common import file_chunker


def test_interrupted_rechunk_invalidates_manifest_and_can_retry(tmp_path: Path, monkeypatch):
  monkeypatch.setattr(file_chunker, 'CHUNK_SIZE', 4)
  path = tmp_path / 'model.pkl'
  data = b'0123456789'
  path.write_bytes(data)
  targets = file_chunker.get_chunk_targets(str(path), len(data))
  file_chunker.chunk_file(path, targets)
  assert file_chunker.read_file_chunked(path) == data

  rebuilt = b'abcdefghij'
  path.write_bytes(rebuilt)
  real_open = builtins.open

  def fail_second_chunk(filename, mode='r', *args, **kwargs):
    if str(filename) == targets[2] and mode == 'wb':
      raise OSError('simulated write failure')
    return real_open(filename, mode, *args, **kwargs)

  with monkeypatch.context() as patch:
    patch.setattr(builtins, 'open', fail_second_chunk)
    with pytest.raises(OSError, match='simulated write failure'):
      file_chunker.chunk_file(path, targets)
  assert not Path(targets[0]).exists()
  assert path.read_bytes() == rebuilt
  assert file_chunker.read_file_chunked(path) == rebuilt

  file_chunker.chunk_file(path, targets)
  assert not path.exists()
  assert file_chunker.read_file_chunked(path) == rebuilt


def test_rechunk_with_smaller_model_keeps_valid_chunk_count(tmp_path: Path, monkeypatch):
  monkeypatch.setattr(file_chunker, 'CHUNK_SIZE', 4)
  path = tmp_path / 'model.pkl'
  targets = file_chunker.get_chunk_targets(str(path), 12)
  path.write_bytes(b'abcdefghij')
  file_chunker.chunk_file(path, targets)
  path.write_bytes(b'new')
  file_chunker.chunk_file(path, targets)
  assert file_chunker.read_file_chunked(path) == b'new'
