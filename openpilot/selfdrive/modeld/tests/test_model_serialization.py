import io
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.modeld.helpers import dump_oob, load_oob, validate_model_file


def model_bytes():
  stream = io.BytesIO()
  dump_oob({'weights': np.arange(8, dtype=np.uint8), 'bias': np.arange(4, dtype=np.float32)}, stream)
  return stream.getvalue()


def test_model_buffers_round_trip():
  result = load_oob(io.BytesIO(model_bytes()))
  np.testing.assert_array_equal(result['weights'], np.arange(8, dtype=np.uint8))
  np.testing.assert_array_equal(result['bias'], np.arange(4, dtype=np.float32))


@pytest.mark.parametrize('missing', [1, 3, 16])
def test_truncated_model_buffer_is_rejected(missing):
  with pytest.raises(EOFError, match='incomplete model buffer'):
    load_oob(io.BytesIO(model_bytes()[:-missing]))


@pytest.mark.parametrize('suffix', [b'\x00', b'extra', b'\x00' * 8])
def test_compiled_model_validation_rejects_trailing_data(tmp_path: Path, suffix):
  path = tmp_path / 'model.pkl'
  path.write_bytes(model_bytes() + suffix)
  with pytest.raises(ValueError, match='unexpected model buffer data'):
    validate_model_file(path)


def test_compiled_model_validation(tmp_path: Path):
  path = tmp_path / 'model.pkl'
  path.write_bytes(model_bytes())
  validate_model_file(path)
  path.write_bytes(model_bytes()[:-1])
  with pytest.raises(EOFError, match='incomplete model buffer'):
    validate_model_file(path)
