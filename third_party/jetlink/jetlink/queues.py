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

  def feedback(self, outputs: dict[str, np.ndarray]) -> None:
    """Queued graphs receive their previous features from the client."""

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


class StatefulQueues:
  """Keep graph-owned history on the host; only images/scalars cross USB.

  State uses the engine's actual dtype (the TRT image queue is promoted to
  fp16). All outputs are checked before another inference can use the state;
  an invalid result clears every state buffer, including shared TRT staging.
  """
  def __init__(self, spec: ModelSpec, inputs: dict[str, np.ndarray]):
    from jetlink.finite import native_checker
    self._native_finite = native_checker()
    self.spec = spec
    self.direct_map = None
    self.input_cast = None
    self.state = {name: inputs[name] for name in spec.state_pairs}
    # NumPy's aarch64 float16 isfinite path is scalar. IEEE binary16 is
    # non-finite exactly when all five exponent bits are set, for BOTH signs
    # and every NaN payload. Reuse bounded scratch space across all outputs.
    largest = max(int(np.prod(shape)) for shape in spec.output_shapes.values())
    self._finite_bits = np.empty(largest, dtype=np.uint16)
    self._finite_bad = np.empty(largest, dtype=np.bool_)
    if not self.state or set(spec.input_shapes) != set(self.state) | {'new_img', *spec.packed_shapes}:
      raise ValueError('Unsupported stateful graph inputs')
    if set(spec.output_shapes) != {'outputs', *spec.state_pairs.values()}:
      raise ValueError('Unsupported stateful graph outputs')
    for name, output in spec.state_pairs.items():
      if inputs[name].shape != spec.input_shapes[name] or spec.output_shapes[output] != spec.input_shapes[name]:
        raise ValueError(f'State shape mismatch: {name}')
    self.reset()

  def reset(self) -> None:
    for value in self.state.values():
      value.fill(0)

  def step_into(self, warped, packed, dest):
    if warped.shape != self.spec.warped_shape or warped.dtype != np.uint8 or packed.size != self.spec.packed_nelem:
      raise ValueError('Invalid stateful frame shape/type')
    if not np.isfinite(packed).all():
      self.reset()
      raise ValueError('Non-finite stateful scalar input')
    self._copy_image(warped, dest)
    offset = 0
    for name, size in zip(self.spec.packed_shapes, self.spec.packed_sizes, strict=True):
      dest[name][...] = packed[offset:offset + size].reshape(dest[name].shape)
      offset += size

  def _copy_image(self, warped, dest):
    image = dest['new_img']
    if image.dtype == np.float16:
      if self.input_cast is not None:
        self.input_cast(warped, image)
      else:
        np.take(_U8_TO_F16_BITS, warped.reshape(-1), out=image.reshape(-1).view(np.uint16), mode='clip')
    else:
      image[...] = warped

  def step_compact_into(self, warped, compact, dest):
    if self.direct_map is None:
      raise RuntimeError('Direct-map input staging is disabled')
    if warped.shape != self.spec.warped_shape or warped.dtype != np.uint8:
      raise ValueError('Invalid stateful frame shape/type')
    try:
      fields = self.direct_map.fields(compact, dest)
    except ValueError:
      self.reset()
      raise
    self._copy_image(warped, dest)
    self.direct_map.copy(fields)

  def _all_finite(self, value):
    if self._native_finite is not None:
      return self._native_finite(value)
    if value.dtype != np.float16:
      return np.isfinite(value).all()
    bits = self._finite_bits[:value.size].reshape(value.shape)
    bad = self._finite_bad[:value.size].reshape(value.shape)
    np.bitwise_and(value.view(np.uint16), 0x7c00, out=bits)
    np.equal(bits, 0x7c00, out=bad)
    return not bad.any()

  def feedback(self, outputs):
    try:
      for name, shape in self.spec.output_shapes.items():
        if name not in outputs or outputs[name].shape != shape:
          raise ValueError(f'Invalid stateful output shape: {name}')
        if not self._all_finite(outputs[name]):
          raise FloatingPointError(f'Non-finite stateful output: {name}')
      for name, output in self.spec.state_pairs.items():
        if outputs[output].dtype != self.state[name].dtype:
          raise ValueError(f'State feedback dtype mismatch: {name}')
      for name, output in self.spec.state_pairs.items():
        np.copyto(self.state[name], outputs[output], casting='no')
    except (ValueError, FloatingPointError):
      self.reset()
      raise


class DeviceStateQueues(StatefulQueues):
  """Same scalar/image preparation; validated recurrence belongs to the GPU.

  Bindings/shapes/dtypes are validated at load. The engine checks every state
  element on device before its reply event; driving outputs remain CPU checked.
  Host state buffers are retained for the existing binding lifetime contract,
  but are neither uploaded nor downloaded per frame.
  """
  def __init__(self, spec, host_inputs, device_state):
    self.device_state = device_state
    if set(spec.state_pairs) != device_state.input_names or set(spec.state_pairs.values()) != device_state.output_names:
      raise ValueError('Device state pairs differ from model contract')
    super().__init__(spec, host_inputs)

  def reset(self):
    super().reset()
    self.device_state.reset()

  def feedback(self, outputs):
    try:
      if set(outputs) != {'outputs'} or outputs['outputs'].shape != self.spec.output_shapes['outputs']:
        raise ValueError('Invalid device-state driving output shape')
      if not self._all_finite(outputs['outputs']):
        raise FloatingPointError('Non-finite driving output')
      self.device_state.validate()
    except (ValueError, FloatingPointError):
      self.reset()
      raise



def make_queues(spec, host_inputs, device_state=None, *, direct_map=False, native_image=False):
  if device_state is not None:
    if not spec.stateful:
      raise ValueError('Device recurrence requires a stateful model')
    queues = DeviceStateQueues(spec, host_inputs, device_state)
  else:
    queues = StatefulQueues(spec, host_inputs) if spec.stateful else PolicyQueues(spec)
  if direct_map:
    from jetlink.direct_map import DirectMap
    queues.direct_map = DirectMap(spec, host_inputs)
  if native_image and spec.stateful:
    from jetlink.input_cast import native_u8_to_f16
    queues.input_cast = native_u8_to_f16()
  return queues
