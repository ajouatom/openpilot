import hashlib
from pathlib import Path


def test_internal_default_is_cd210():
  path = Path(__file__).parents[1] / 'models/driving_supercombo.onnx'
  with path.open('rb') as stream:
    assert hashlib.file_digest(stream, 'sha256').hexdigest() == 'f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5'
