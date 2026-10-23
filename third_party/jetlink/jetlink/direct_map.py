"""Experimental compact-map scatter; no image conversion or wire-format change."""
import numpy as np

from jetlink.packed_codec import MAP_PIXELS, _layout

U8_F16_BITS = np.arange(256, dtype=np.uint8).astype(np.float16).view(np.uint16)


class DirectMap:
  def __init__(self, spec, dest):
    _layout(spec)
    self.spec, self.dest = spec, dest
    self.bindings = []
    for name in ('new_img', *spec.packed_shapes):
      value = dest[name]
      if (value.shape != spec.input_shapes[name] or value.dtype not in (np.float16, np.float32)
          or not value.flags.c_contiguous or not value.flags.aligned or not value.flags.writeable):
        raise ValueError(f'Unsupported direct-map binding: {name}')
      self.bindings.append((name, value, value.ctypes.data, value.dtype))
    self.layout = []
    offset = 0
    for name, count in zip(spec.packed_shapes, spec.packed_sizes, strict=True):
      is_map = name == 'nav_map'
      self.layout.append((name, offset, count, is_map))
      offset += count if is_map else count * 4
    self.nbytes = spec.packed_nbytes - MAP_PIXELS * 3
    if offset != self.nbytes:
      raise ValueError('Invalid direct-map layout')

  def validate_size(self, payload):
    raw = memoryview(payload).cast('B')
    if raw.nbytes != self.nbytes:
      raise ValueError('Invalid compact navigation context size')
    return raw

  def fields(self, payload, dest):
    raw = self.validate_size(payload)
    if dest is not self.dest or any(dest.get(name) is not value or value.ctypes.data != pointer
        or value.dtype != dtype or value.shape != self.spec.input_shapes[name]
        or not value.flags.writeable or not value.flags.c_contiguous or not value.flags.aligned
        for name, value, pointer, dtype in self.bindings):
      raise ValueError('Direct-map bindings changed after load')
    fields = [(name, np.frombuffer(raw, np.uint8 if is_map else np.float32, count, offset), is_map)
              for name, offset, count, is_map in self.layout]
    # Validate every scalar before any destination write. uint8 proves map
    # finiteness/range; fractional or negative-zero maps use the old raw path.
    if any(not np.isfinite(value).all() for _, value, is_map in fields if not is_map):
      raise ValueError('Non-finite stateful scalar input')
    return fields

  def copy(self, fields):
    for name, value, is_map in fields:
      target = self.dest[name]
      if is_map and target.dtype == np.float16:
        np.take(U8_F16_BITS, value, out=target.reshape(-1).view(np.uint16), mode='clip')
      else:
        target[...] = value.reshape(target.shape)
