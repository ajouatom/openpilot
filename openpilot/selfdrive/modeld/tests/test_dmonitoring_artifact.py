import hashlib
from pathlib import Path

from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict


def test_super_leicht_artifact_and_runtime_contract():
  path = Path(__file__).parents[1] / 'models/dmonitoring_model.onnx'
  # Official commaai/openpilot#38942, not an LFS pointer or an older cached model.
  assert hashlib.sha256(path.read_bytes()).hexdigest() == 'dee5a294e8afaacc9295ac5d100e00733ecac278e79264b96331e40a3ede1b04'
  metadata = make_metadata_dict(path)
  assert metadata['model_checkpoint'] == 'a9462a65-1886-462a-8847-ad4624d9abfc/200'
  assert metadata['input_shapes'] == {'input_img': (1, 1382400), 'calib': (1, 3)}
  assert metadata['output_shapes'] == {'outputs': (1, 553)}
  slices = metadata['output_slices']
  assert slices['sleep_prob_lhd'] == slice(19, 20)
  assert slices['sleep_prob_rhd'] == slice(39, 40)
  assert slices['wheel_on_right'] == slice(40, 41)
