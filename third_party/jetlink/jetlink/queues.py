"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

The model's history buffers, reimplemented in numpy.

openpilot folds these into the tinygrad JIT on the GPU. Shipping the history
across the link would cost ~10 MB a frame, so the queues live on the server and
the comma sends only the newest warped frame and the packed scalars.
tests/test_queues.py checks this against the tinygrad original.

Two value-preserving differences: ring buffers rather than openpilot's rolling
`cat(buf[1:], new)`, which copies 8.4 MB a frame; and float16 storage rather
than float32 cast at the model boundary, so only the new row is cast. Pass
dtype=np.float32 for openpilot's exact intermediates.
"""
from __future__ import annotations

import numpy as np

from jetlink.spec import ModelSpec


# numpy has no vectorised float16 store on aarch64: uint8 -> float16 costs
# 5.2 ns/element, 68% of the frame. Only 256 inputs exist, so a lookup gives the
# same bits at memcpy speed. Viewed as uint16 so np.take can share the dtype.
_U8_TO_F16_BITS = np.arange(256, dtype=np.uint8).astype(np.float16).view(np.uint16)


def _strided_runs(head: int, n: int, step: int) -> list[tuple[slice, slice]]:
  """Rows head, head+step, ... (mod n) as at most two strided slices.

  Strided slices hit numpy's memcpy path; fancy indexing costs more than the
  copy it performs. A wrapped sequence is always two runs of constant stride.
  """
  m = -(-n // step)                        # number of sampled rows
  m1 = min(m, -(-(n - head) // step))      # rows before the wrap
  runs = [(slice(head, head + m1 * step, step), slice(0, m1))]
  if m1 < m:
    start = head + m1 * step - n
    runs.append((slice(start, start + (m - m1) * step, step), slice(m1, m)))
  return runs


class RingQueue:
  """Fixed-length FIFO over a preallocated array.

  Logical index i (0 = oldest) lives at physical (head + i) % n.
  """

  def __init__(self, shape: tuple[int, ...], dtype):
    self.buf = np.zeros(shape, dtype=dtype)
    self.n = shape[0]
    self.head = 0
    # only fp16 rings can use the lookup table; float32 falls back to assignment
    self._lut = self.buf.dtype == np.float16

  def reset(self) -> None:
    self.buf[:] = 0
    self.head = 0

  def push(self, value) -> None:
    # The slot the oldest element occupies becomes the newest once head moves.
    dest = self.buf[self.head]
    if self._lut and getattr(value, 'dtype', None) == np.uint8:
      # take-with-out, not `dest[:] = lut[src]`: the latter builds a 393 KB
      # temporary, 2.4 ms against 1.4 ms on the Orin (the other way on x86, so
      # measure there). clip skips a bounds check a uint8 index cannot fail.
      np.take(_U8_TO_F16_BITS, value.reshape(-1),
              out=dest.reshape(-1).view(np.uint16), mode='clip')
    else:
      dest[...] = value
    self.head = (self.head + 1) % self.n

  def gather(self, step: int, out: np.ndarray) -> np.ndarray:
    """Write logical rows 0, step, 2*step, ... into `out`, oldest first."""
    for src, dst in _strided_runs(self.head, self.n, step):
      out[dst] = self.buf[src]
    return out

  def logical(self, step: int = 1) -> np.ndarray:
    """Same as gather(), allocating the destination."""
    m = -(-self.n // step)
    return self.gather(step, np.empty((m, *self.buf.shape[1:]), self.buf.dtype))


def sample_skip(q: RingQueue, frame_skip: int, out: np.ndarray | None = None) -> np.ndarray:
  """openpilot: buf[::frame_skip].contiguous().flatten(0, 1).unsqueeze(0)

  With `out`, shaped (k, *buf.shape[1:]), the gather lands straight in
  TensorRT's pinned input buffers.
  """
  if out is not None:
    return q.gather(frame_skip, out)
  s = q.logical(frame_skip)
  return s.reshape(1, s.shape[0] * s.shape[1], *s.shape[2:])


def sample_desire(q: RingQueue, frame_skip: int, out: np.ndarray | None = None) -> np.ndarray:
  """openpilot: buf.reshape(-1, frame_skip, *buf.shape[1:]).max(1).flatten(0, 1).unsqueeze(0)"""
  s = q.logical(1)
  m = s.reshape(-1, frame_skip, *s.shape[1:]).max(axis=1)
  if out is not None:
    out[...] = m.reshape(out.shape)
    return out
  return m.reshape(1, m.shape[0] * m.shape[1], *m.shape[2:])


class PolicyQueues:
  """Server-side state for one model. Everything `run_policy` owned in the JIT."""

  def __init__(self, spec: ModelSpec, dtype=np.float16):
    self.spec = spec
    self.dtype = dtype
    self.frame_skip = spec.frame_skip

    offset = 0
    self._packed_layout = []
    for size, shape in zip(spec.packed_sizes, spec.packed_shapes.values(), strict=True):
      self._packed_layout.append((offset, offset + size, shape))
      offset += size

    self.img_q = RingQueue(spec.img_buf_shape, dtype)
    self.big_img_q = RingQueue(spec.img_buf_shape, dtype)
    self.feat_q = RingQueue(spec.feat_q_shape, dtype)
    self.desire_q = RingQueue(spec.desire_q_shape, dtype)

    # features_buffer is declared 4-D (1,32,32,512); the queue produces the flat
    # (1,32,16384) over the same memory, so a reshape covers it.
    self.model_shapes = dict(spec.input_shapes)

  def reset(self) -> None:
    for q in (self.img_q, self.big_img_q, self.feat_q, self.desire_q):
      q.reset()

  def _unpack(self, packed: np.ndarray):
    # slice views, not np.split: the offsets never change and split allocates
    return tuple(packed[a:b].reshape(shape) for a, b, shape in self._packed_layout)

  def _push(self, warped: np.ndarray, packed: np.ndarray):
    spec = self.spec
    if warped.shape != spec.warped_shape:
      raise ValueError(f"warped {warped.shape} != {spec.warped_shape}")
    if packed.size != spec.packed_nelem:
      raise ValueError(f"packed {packed.size} != {spec.packed_nelem}")
    desire, traffic_convention, action_t, prev_feat = self._unpack(packed)
    # push() casts into a typed buffer, so uint8 -> float16 costs one row here
    # rather than the whole sampled window later
    self.img_q.push(warped[0])
    self.big_img_q.push(warped[1])
    self.desire_q.push(desire.reshape(1, -1))
    self.feat_q.push(prev_feat.reshape(1, -1))
    return traffic_convention, action_t

  def step(self, warped: np.ndarray, packed: np.ndarray) -> dict[str, np.ndarray]:
    """Advance the queues one frame and return the model's inputs.

    warped: (2, 6, H, W) uint8 from openpilot's warp. packed: flat float32,
    laid out per ModelSpec.packed_shapes. Allocates; the server uses
    step_into(), so there is one implementation to keep correct.
    """
    dest = {n: np.empty(s, self.dtype) for n, s in self.model_shapes.items()}
    self.step_into(warped, packed, dest)
    return dest

  def step_into(self, warped: np.ndarray, packed: np.ndarray,
                dest: dict[str, np.ndarray]) -> None:
    """Same as step(), writing into caller-owned buffers.

    `dest` maps input name to an array of the declared shape. In the server
    those are TensorRT's pinned buffers, so the gather is the only copy.
    """
    traffic_convention, action_t = self._push(warped, packed)
    fs = self.frame_skip

    for name, q in (('img', self.img_q), ('big_img', self.big_img_q)):
      d = dest[name]
      sample_skip(q, fs, out=d.reshape(-1, *q.buf.shape[1:]))

    d = dest['features_buffer']
    sample_skip(self.feat_q, fs, out=d.reshape(-1, *self.feat_q.buf.shape[1:]))

    sample_desire(self.desire_q, fs, out=dest['desire_pulse'])
    dest['traffic_convention'][...] = traffic_convention.reshape(
      dest['traffic_convention'].shape)
    dest['action_t'][...] = action_t.reshape(dest['action_t'].shape)
