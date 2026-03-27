import json

import numpy as np
import pytest

import openpilot.selfdrive.controls.lib.neurodob as neurodob


def write_model(path, payload):
  path.write_text(json.dumps(payload), encoding="utf-8")


def test_mlp_predict_and_clamp(tmp_path):
  model_path = tmp_path / "torque_delta_TEST.json"
  write_model(model_path, {
    "version": "1.0",
    "model_type": "mlp_delta",
    "control_type": "torque",
    "feature_order": list(neurodob.TORQUE_FEATURE_ORDER),
    "input_mean": [0.0] * 8,
    "input_std": [1.0] * 8,
    "max_delta": 0.25,
    "layers": [
      {
        "W": [[2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        "b": [0.0],
        "activation": "identity",
      },
    ],
  })

  model = neurodob.NeuroDOBModel.from_file(str(model_path))
  assert model.predict([0.1] + [0.0] * 7) == pytest.approx(0.2)
  assert model.predict([1.0] + [0.0] * 7) == pytest.approx(0.25)


def test_gru_supports_split_bias_export(tmp_path):
  model_path = tmp_path / "angle_delta_TEST.json"
  write_model(model_path, {
    "version": "1.0",
    "model_type": "gru_delta",
    "control_type": "angle",
    "feature_order": list(neurodob.ANGLE_FEATURE_ORDER),
    "input_mean": [0.0] * 8,
    "input_std": [1.0] * 8,
    "hidden_size": 2,
    "max_delta": 1.0,
    "gru": {
      "Wz": np.zeros((2, 8), dtype=np.float32).tolist(),
      "Wr": np.zeros((2, 8), dtype=np.float32).tolist(),
      "Wh": np.zeros((2, 8), dtype=np.float32).tolist(),
      "Uz": np.zeros((2, 2), dtype=np.float32).tolist(),
      "Ur": np.zeros((2, 2), dtype=np.float32).tolist(),
      "Uh": np.zeros((2, 2), dtype=np.float32).tolist(),
      "bz": [0.0, 0.0],
      "br": [0.0, 0.0],
      "bh_ih": [0.0, 0.0],
      "bh_hh": [0.0, 0.0],
    },
    "fc": {
      "W": [[0.0, 0.0]],
      "b": [0.0],
    },
  })

  model = neurodob.NeuroDOBModel.from_file(str(model_path))
  assert model.predict([0.0] * 8) == pytest.approx(0.0)
  model.reset()
  assert np.allclose(model.h, np.zeros(2, dtype=np.float32))


def test_get_neurodob_model_prefers_custom(tmp_path, monkeypatch):
  base = {
    "version": "1.0",
    "model_type": "mlp_delta",
    "control_type": "torque",
    "feature_order": list(neurodob.TORQUE_FEATURE_ORDER),
    "input_mean": [0.0] * 8,
    "input_std": [1.0] * 8,
    "max_delta": 1.0,
    "layers": [{"W": [[1.0] + [0.0] * 7], "b": [0.0], "activation": "identity"}],
  }
  write_model(tmp_path / "torque_delta_TEST.json", base)
  custom = dict(base)
  custom["layers"] = [{"W": [[3.0] + [0.0] * 7], "b": [0.0], "activation": "identity"}]
  write_model(tmp_path / "torque_delta_TEST_custom.json", custom)

  monkeypatch.setattr(neurodob, "NEURODOB_MODELS_DIR", str(tmp_path))

  model = neurodob.get_neurodob_model("TEST", "torque")
  assert model is not None
  assert model.predict([0.1] + [0.0] * 7) == pytest.approx(0.3)
