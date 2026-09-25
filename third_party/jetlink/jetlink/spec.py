"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Every size on the wire, derived from the model's ONNX metadata.

Mirrors get_policy_npy_shapes / make_input_queues in openpilot master's
selfdrive/modeld/compile_modeld.py. feat_dim is prod(fb[2:]); older forks use
fb[2], which gives 32 rather than 16384 for the big model's (1,32,32,512)
features_buffer. tests/test_queues.py catches the drift.
"""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass

from jetlink.onnx_meta import OnnxMeta, parse_file
from jetlink.protocol import INFER_REQ_SIZE, INFER_RESP_SIZE

# openpilot ModelConstants; duplicated so the server needs no openpilot import
MODEL_RUN_FREQ = 20
MODEL_CONTEXT_FREQ = 5
DEFAULT_FRAME_SKIP = MODEL_RUN_FREQ // MODEL_CONTEXT_FREQ  # 4

CHUNK = 4 << 20  # model upload chunk


@dataclass(frozen=True)
class ModelSpec:
  """Everything both ends need to agree on, derived from the ONNX."""
  sha256: str
  nbytes: int
  frame_skip: int
  input_shapes: dict[str, tuple[int, ...]]
  output_shapes: dict[str, tuple[int, ...]]
  output_slices: dict[str, slice]
  checkpoint: str | None

  # --- vision ---
  @property
  def img_shape(self) -> tuple[int, ...]:
    return self.input_shapes['img']  # (1, 12, H, W)

  @property
  def n_frames(self) -> int:
    return self.img_shape[1] // 6

  @property
  def model_hw(self) -> tuple[int, int]:
    return self.img_shape[2], self.img_shape[3]

  @property
  def img_buf_shape(self) -> tuple[int, int, int, int]:
    h, w = self.model_hw
    return (self.frame_skip * (self.n_frames - 1) + 1, 6, h, w)

  @property
  def warped_shape(self) -> tuple[int, int, int, int]:
    """What `warp` on the comma produces: narrow and wide stacked."""
    h, w = self.model_hw
    return (2, 6, h, w)

  @property
  def warped_nbytes(self) -> int:
    return math.prod(self.warped_shape)  # uint8

  # --- recurrent / scalar inputs ---
  @property
  def feat_dim(self) -> int:
    """Flattened per-frame feature size. (1,32,32,512) -> 16384."""
    fb = self.input_shapes['features_buffer']
    return math.prod(fb[2:])

  @property
  def packed_shapes(self) -> dict[str, tuple[int, ...]]:
    dp = self.input_shapes['desire_pulse']
    tc = self.input_shapes['traffic_convention']
    at = self.input_shapes['action_t']
    fb = self.input_shapes['features_buffer']
    return {
      'desire': (dp[2],),
      'traffic_convention': tuple(tc),
      'action_t': tuple(at),
      'prev_feat': (fb[0], self.feat_dim),
    }

  @property
  def packed_sizes(self) -> list[int]:
    return [math.prod(s) for s in self.packed_shapes.values()]

  @property
  def packed_nelem(self) -> int:
    return sum(self.packed_sizes)

  @property
  def packed_nbytes(self) -> int:
    return self.packed_nelem * 4  # float32

  @property
  def feat_q_shape(self) -> tuple[int, int, int]:
    fb = self.input_shapes['features_buffer']
    return (self.frame_skip * fb[1], fb[0], self.feat_dim)

  @property
  def desire_q_shape(self) -> tuple[int, int, int]:
    dp = self.input_shapes['desire_pulse']
    return (self.frame_skip * dp[1], dp[0], dp[2])

  # --- output ---
  @property
  def output_nelem(self) -> int:
    return math.prod(self.output_shapes['outputs'])

  @property
  def output_nbytes(self) -> int:
    return self.output_nelem * 4  # we return float32, as openpilot's JIT does

  # --- wire sizes ---
  @property
  def infer_req_nbytes(self) -> int:
    return INFER_REQ_SIZE + self.warped_nbytes + self.packed_nbytes

  @property
  def infer_resp_nbytes(self) -> int:
    return INFER_RESP_SIZE + self.output_nbytes

  # -- the wire form of a spec ---------------------------------------------
  # One encoder and one decoder: both ends and the bench need this, and copies
  # would drift the moment a field is added.

  def to_dict(self) -> dict:
    return {
      'sha256': self.sha256,
      'nbytes': self.nbytes,
      'frame_skip': self.frame_skip,
      'checkpoint': self.checkpoint,
      'input_shapes': {k: list(v) for k, v in self.input_shapes.items()},
      'output_shapes': {k: list(v) for k, v in self.output_shapes.items()},
      'output_slices': {k: [v.start, v.stop] for k, v in self.output_slices.items()},
    }

  @classmethod
  def from_dict(cls, d: dict) -> ModelSpec:
    return cls(
      sha256=d['sha256'], nbytes=d['nbytes'],
      frame_skip=d.get('frame_skip', DEFAULT_FRAME_SKIP),
      input_shapes={k: tuple(v) for k, v in d['input_shapes'].items()},
      output_shapes={k: tuple(v) for k, v in d['output_shapes'].items()},
      output_slices={k: slice(*v) for k, v in d['output_slices'].items()},
      checkpoint=d.get('checkpoint'))


def sha256_file(path: str, bufsize: int = 1 << 20) -> tuple[str, int]:
  h = hashlib.sha256()
  n = 0
  with open(path, 'rb') as f:
    while chunk := f.read(bufsize):
      h.update(chunk)
      n += len(chunk)
  return h.hexdigest(), n


def spec_from_onnx(path: str, frame_skip: int = DEFAULT_FRAME_SKIP,
                   sha256: str | None = None, nbytes: int | None = None) -> ModelSpec:
  meta: OnnxMeta = parse_file(path)
  if sha256 is None or nbytes is None:
    sha256, nbytes = sha256_file(path)
  return spec_from_meta(meta, sha256, nbytes, frame_skip)


def spec_from_meta(meta: OnnxMeta, sha256: str, nbytes: int,
                   frame_skip: int = DEFAULT_FRAME_SKIP) -> ModelSpec:
  return ModelSpec(
    sha256=sha256,
    nbytes=nbytes,
    frame_skip=frame_skip,
    input_shapes=dict(meta.inputs),
    output_shapes=dict(meta.outputs),
    output_slices=meta.output_slices,
    checkpoint=meta.model_checkpoint,
  )
