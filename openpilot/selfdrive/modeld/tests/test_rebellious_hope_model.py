import ast
from pathlib import Path

import numpy as np

from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
from openpilot.selfdrive.modeld.parse_model_outputs import Parser


MODELD_DIR = Path(__file__).resolve().parents[1]
MODEL_PATH = MODELD_DIR / "models" / "driving_supercombo.onnx"


def test_rebellious_hope_model_interface():
  metadata = make_metadata_dict(MODEL_PATH)

  assert metadata["model_checkpoint"] == "5c5d862b-d4dd-4087-aa8b-232ff9ac943f/100/d0c60276-564d-4b97-a59c-f1ab853d8c26/400"
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


def test_rebellious_hope_does_not_override_carrot_smoothing():
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


def test_modelstate_always_evaluates_current_frame():
  tree = ast.parse((MODELD_DIR / "modeld.py").read_text(encoding="utf-8"))
  model_state = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "ModelState")
  run = next(node for node in model_state.body if isinstance(node, ast.FunctionDef) and node.name == "run")

  assert [arg.arg for arg in run.args.args] == ["self", "bufs", "transforms", "inputs"]
  assert not any(isinstance(node, ast.Name) and node.id == "prepare_only" for node in ast.walk(tree))


def test_parser_mdn_default_shape():
  outputs = {"value": np.array([[1.5, 0.0]], dtype=np.float32)}
  Parser().parse_mdn("value", outputs, out_N=0)

  np.testing.assert_array_equal(outputs["value"], np.array([1.5], dtype=np.float32))
  np.testing.assert_array_equal(outputs["value_stds"], np.array([1.0], dtype=np.float32))
