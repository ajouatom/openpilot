import json
import os
from typing import Any

import numpy as np


NEURODOB_MODELS_DIR = os.path.join(os.path.dirname(__file__), "neurodob_models")

TORQUE_FEATURE_ORDER = (
  "lateral_error",
  "lateral_error_rate",
  "yaw_rate_error",
  "steering_rate_deg",
  "control_output",
  "v_ego",
  "roll",
  "desired_lateral_accel",
)

ANGLE_FEATURE_ORDER = (
  "angle_error",
  "angle_error_rate",
  "yaw_rate_error",
  "steering_rate_deg",
  "control_output",
  "v_ego",
  "roll",
  "desired_lateral_accel",
)


def sigmoid(x: np.ndarray) -> np.ndarray:
  return 1.0 / (1.0 + np.exp(-np.clip(x, -500.0, 500.0)))


ACTIVATIONS = {
  "identity": lambda x: x,
  "sigmoid": sigmoid,
  "tanh": np.tanh,
}


def expected_feature_order(control_type: str) -> tuple[str, ...]:
  return ANGLE_FEATURE_ORDER if control_type == "angle" else TORQUE_FEATURE_ORDER


class NeuroDOBModel:
  def __init__(self, params: dict[str, Any], source_path: str = ""):
    self.source_path = source_path
    self.version = str(params.get("version", "1.0"))
    self.model_type = str(params.get("model_type", "mlp_delta"))
    self.control_type = str(params.get("control_type", "torque"))
    self.max_delta = float(params.get("max_delta", 0.15))

    default_feature_order = list(expected_feature_order(self.control_type))
    self.feature_order = tuple(params.get("feature_order", default_feature_order))
    if self.feature_order != expected_feature_order(self.control_type):
      raise ValueError(f"unexpected feature order for {self.control_type}: {self.feature_order}")

    self.input_mean = np.asarray(params["input_mean"], dtype=np.float32).reshape(-1)
    self.input_std = np.asarray(params["input_std"], dtype=np.float32).reshape(-1)
    if len(self.input_mean) != len(self.feature_order) or len(self.input_std) != len(self.feature_order):
      raise ValueError(f"input stats length mismatch for {source_path or 'NeuroDOB model'}")

    if self.model_type == "mlp_delta":
      self._load_mlp(params)
    elif self.model_type == "gru_delta":
      self._load_gru(params)
    else:
      raise ValueError(f"unsupported NeuroDOB model_type: {self.model_type}")

  @classmethod
  def from_file(cls, params_file: str) -> "NeuroDOBModel":
    if not params_file.endswith(".json"):
      raise ValueError(f"unsupported NeuroDOB model format: {params_file}")

    with open(params_file, "r", encoding="utf-8") as f:
      params = json.load(f)
    return cls(params, params_file)

  def _load_mlp(self, params: dict[str, Any]) -> None:
    raw_layers = params.get("layers")
    if not isinstance(raw_layers, list) or len(raw_layers) == 0:
      raise ValueError("mlp_delta requires non-empty layers")

    self.layers = []
    for layer in raw_layers:
      W = np.asarray(layer["W"], dtype=np.float32)
      b = np.asarray(layer["b"], dtype=np.float32).reshape(-1)
      activation_name = str(layer.get("activation", "tanh"))
      if activation_name not in ACTIVATIONS:
        raise ValueError(f"unsupported activation: {activation_name}")

      use_bn = bool(layer.get("batch_norm", False))
      bn_gamma = np.asarray(layer["bn_gamma"], dtype=np.float32).reshape(-1) if use_bn else None
      bn_beta = np.asarray(layer["bn_beta"], dtype=np.float32).reshape(-1) if use_bn else None
      bn_mean = np.asarray(layer["bn_mean"], dtype=np.float32).reshape(-1) if use_bn else None
      bn_var = np.asarray(layer["bn_var"], dtype=np.float32).reshape(-1) if use_bn else None
      self.layers.append((W, b, ACTIVATIONS[activation_name], use_bn, bn_gamma, bn_beta, bn_mean, bn_var))

  def _load_gru(self, params: dict[str, Any]) -> None:
    gru = params["gru"]
    self.Wz = np.asarray(gru["Wz"], dtype=np.float32)
    self.Wr = np.asarray(gru["Wr"], dtype=np.float32)
    self.Wh = np.asarray(gru["Wh"], dtype=np.float32)
    self.Uz = np.asarray(gru["Uz"], dtype=np.float32)
    self.Ur = np.asarray(gru["Ur"], dtype=np.float32)
    self.Uh = np.asarray(gru["Uh"], dtype=np.float32)
    self.bz = np.asarray(gru["bz"], dtype=np.float32).reshape(-1)
    self.br = np.asarray(gru["br"], dtype=np.float32).reshape(-1)

    if "bh" in gru:
      self.bh = np.asarray(gru["bh"], dtype=np.float32).reshape(-1)
    elif "bh_ih" in gru and "bh_hh" in gru:
      self.bh = np.asarray(gru["bh_ih"], dtype=np.float32).reshape(-1) + np.asarray(gru["bh_hh"], dtype=np.float32).reshape(-1)
    else:
      raise ValueError("gru_delta requires bh or bh_ih/bh_hh")

    fc = params["fc"]
    self.fc_W = np.asarray(fc["W"], dtype=np.float32)
    self.fc_b = np.asarray(fc["b"], dtype=np.float32).reshape(-1)

    hidden_size = int(params.get("hidden_size", self.bz.shape[0]))
    if hidden_size <= 0:
      raise ValueError("gru_delta hidden_size must be positive")
    self.h = np.zeros(hidden_size, dtype=np.float32)

  def _matvec(self, W: np.ndarray, x: np.ndarray) -> np.ndarray:
    if W.ndim != 2:
      raise ValueError(f"expected matrix, got shape {W.shape}")
    if W.shape[1] == x.shape[0]:
      return W @ x
    if W.shape[0] == x.shape[0]:
      return x @ W
    raise ValueError(f"incompatible shapes {W.shape} and {x.shape}")

  def _affine(self, W: np.ndarray, b: np.ndarray, x: np.ndarray) -> np.ndarray:
    y = self._matvec(W, x) + b
    return np.asarray(y, dtype=np.float32).reshape(-1)

  def predict(self, raw_input: list[float] | tuple[float, ...]) -> float:
    x = np.asarray(raw_input, dtype=np.float32).reshape(-1)
    if x.shape[0] != self.input_mean.shape[0]:
      raise ValueError(f"expected {self.input_mean.shape[0]} features, got {x.shape[0]}")

    x = (x - self.input_mean) / (self.input_std + 1e-6)

    if self.model_type == "mlp_delta":
      return self._forward_mlp(x)
    if self.model_type == "gru_delta":
      return self._forward_gru(x)
    raise ValueError(f"unsupported NeuroDOB model_type: {self.model_type}")

  def _forward_mlp(self, x: np.ndarray) -> float:
    for W, b, act, use_bn, bn_gamma, bn_beta, bn_mean, bn_var in self.layers:
      x = self._affine(W, b, x)
      if use_bn and bn_mean is not None and bn_var is not None and bn_gamma is not None and bn_beta is not None:
        x = (x - bn_mean) / np.sqrt(bn_var + 1e-5)
        x = bn_gamma * x + bn_beta
      x = np.asarray(act(x), dtype=np.float32).reshape(-1)

    delta = float(x[0]) if x.size else 0.0
    return float(np.clip(delta, -self.max_delta, self.max_delta))

  def _forward_gru(self, x: np.ndarray) -> float:
    z = sigmoid(self._matvec(self.Wz, x) + self._matvec(self.Uz, self.h) + self.bz)
    r = sigmoid(self._matvec(self.Wr, x) + self._matvec(self.Ur, self.h) + self.br)
    h_candidate = np.tanh(self._matvec(self.Wh, x) + self._matvec(self.Uh, r * self.h) + self.bh)
    self.h = (1.0 - z) * self.h + z * h_candidate
    out = self._affine(self.fc_W, self.fc_b, self.h)
    delta = float(out[0]) if out.size else 0.0
    return float(np.clip(delta, -self.max_delta, self.max_delta))

  def reset(self) -> None:
    if self.model_type == "gru_delta":
      self.h = np.zeros_like(self.h)


def get_neurodob_model(car_fingerprint: str, control_type: str) -> NeuroDOBModel | None:
  prefix = "angle_delta" if control_type == "angle" else "torque_delta"
  base_name = f"{prefix}_{car_fingerprint}"

  for suffix in ("_custom.json", ".json"):
    path = os.path.join(NEURODOB_MODELS_DIR, f"{base_name}{suffix}")
    if not os.path.isfile(path):
      continue

    try:
      model = NeuroDOBModel.from_file(path)
    except Exception as e:
      print(f"NeuroDOB load failed: {path} ({e})")
      return None

    if model.control_type != control_type:
      print(f"NeuroDOB control_type mismatch: expected {control_type}, got {model.control_type} ({path})")
      return None
    return model

  return None
