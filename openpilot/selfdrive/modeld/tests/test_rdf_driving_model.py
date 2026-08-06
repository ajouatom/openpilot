import ast
import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
from openpilot.selfdrive.modeld.parse_model_outputs import Parser


MODELD_DIR = Path(__file__).resolve().parents[1]
MODEL_PATH = MODELD_DIR / "models" / "driving_supercombo.onnx"
MANIFEST_PATH = MODEL_PATH.with_suffix(".onnx.json")


def test_rdf_driving_model_interface():
  if not MODEL_PATH.is_file():
    pytest.skip("RDF model is downloaded before the device build")
  metadata = make_metadata_dict(MODEL_PATH)

  assert metadata["model_checkpoint"] == "5c5d862b-d4dd-4087-aa8b-232ff9ac943f/100/1c13f8c7-a35d-4796-880a-902f600130db/400"
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


def test_rdf_manifest_records_model_interface():
  manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))

  assert manifest["model_checkpoint"] == "5c5d862b-d4dd-4087-aa8b-232ff9ac943f/100/1c13f8c7-a35d-4796-880a-902f600130db/400"
  assert manifest["input_shapes"] == {
    "img": [1, 12, 128, 256],
    "big_img": [1, 12, 128, 256],
    "desire_pulse": [1, 25, 8],
    "traffic_convention": [1, 2],
    "action_t": [1, 2],
    "features_buffer": [1, 24, 512],
  }
  assert manifest["output_shapes"] == {"outputs": [1, 2580]}
  assert manifest["output_slices"] == {
    "action": [2062, 2066],
    "hidden_state": [2066, 2578],
  }


def test_rdf_driving_does_not_override_carrot_smoothing():
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


def test_modelstate_skips_eval_after_vipc_drop():
  tree = ast.parse((MODELD_DIR / "modeld.py").read_text(encoding="utf-8"))
  model_state = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "ModelState")
  run = next(node for node in model_state.body if isinstance(node, ast.FunctionDef) and node.name == "run")
  main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")

  assert [arg.arg for arg in run.args.args] == ["self", "bufs", "transforms", "inputs", "prepare_only"]
  prepare_guard = next(
    node for node in run.body
    if isinstance(node, ast.If) and isinstance(node.test, ast.Name) and node.test.id == "prepare_only"
  )
  assert len(prepare_guard.body) == 1
  assert isinstance(prepare_guard.body[0], ast.Return)
  assert isinstance(prepare_guard.body[0].value, ast.Constant) and prepare_guard.body[0].value.value is None

  prepare_assignment = next(
    node for node in ast.walk(main)
    if isinstance(node, ast.Assign)
    and any(isinstance(target, ast.Name) and target.id == "prepare_only" for target in node.targets)
  )
  assert isinstance(prepare_assignment.value, ast.Compare)
  assert isinstance(prepare_assignment.value.left, ast.Name) and prepare_assignment.value.left.id == "vipc_dropped_frames"
  assert len(prepare_assignment.value.ops) == 1 and isinstance(prepare_assignment.value.ops[0], ast.Gt)
  assert len(prepare_assignment.value.comparators) == 1
  assert isinstance(prepare_assignment.value.comparators[0], ast.Constant) and prepare_assignment.value.comparators[0].value == 0

  model_run = next(
    node for node in ast.walk(main)
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "run"
    and isinstance(node.func.value, ast.Name) and node.func.value.id == "model"
  )
  assert isinstance(model_run.args[-1], ast.Name) and model_run.args[-1].id == "prepare_only"


def test_parser_mdn_default_shape():
  outputs = {"value": np.array([[1.5, 0.0]], dtype=np.float32)}
  Parser().parse_mdn("value", outputs, out_N=0)

  np.testing.assert_array_equal(outputs["value"], np.array([1.5], dtype=np.float32))
  np.testing.assert_array_equal(outputs["value_stds"], np.array([1.0], dtype=np.float32))
