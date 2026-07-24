import ast
from pathlib import Path

from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict


MODELD_DIR = Path(__file__).resolve().parents[1]
MODEL_PATH = MODELD_DIR / "models" / "driving_supercombo.onnx"


def test_rebel_legion_model_interface():
  metadata = make_metadata_dict(MODEL_PATH)

  assert metadata["model_checkpoint"] == "1c8e05fa-bb24-42ad-af22-c0e6d59a5df5/100/6d9d6f8a-5c82-41f6-92aa-4c1a11eb5645/400"
  assert metadata["input_shapes"] == {
    "img": (1, 12, 128, 256),
    "big_img": (1, 12, 128, 256),
    "desire_pulse": (1, 25, 8),
    "traffic_convention": (1, 2),
    "action_t": (1, 2),
    "features_buffer": (1, 24, 512),
  }
  assert metadata["output_shapes"] == {"outputs": (1, 2580)}
  assert metadata["output_slices"]["action"] == slice(2062, 2066)
  assert metadata["output_slices"]["hidden_state"] == slice(2066, 2578)


def test_rebel_legion_does_not_change_carrot_smoothing():
  tree = ast.parse((MODELD_DIR / "modeld.py").read_text(encoding="utf-8"))
  constants = {}
  for node in tree.body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      if node.targets[0].id in ("LAT_SMOOTH_SECONDS", "LONG_SMOOTH_SECONDS"):
        constants[node.targets[0].id] = ast.literal_eval(node.value)

  assert constants == {
    "LAT_SMOOTH_SECONDS": 0.0,
    "LONG_SMOOTH_SECONDS": 0.3,
  }
