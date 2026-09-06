"""Read visible NV12 planes from padded VisionIPC camera allocations."""

import numpy as np


def nv12_y_plane(data: bytes | memoryview, width: int, height: int, stride: int) -> np.ndarray:
  if width <= 0 or height <= 0 or stride < width:
    raise ValueError("invalid NV12 dimensions or stride")
  raw = np.frombuffer(data, dtype=np.uint8)
  y_size = stride * height
  if raw.size < y_size:
    raise ValueError(f"short NV12 Y plane: {raw.size} < {y_size}")
  return raw[:y_size].reshape(height, stride)[:, :width]


def pack_nv12(data: bytes | memoryview, width: int, height: int, stride: int, uv_offset: int) -> np.ndarray:
  """Pack visible Y and UV rows, excluding scanline padding and extra allocation bytes."""
  y = nv12_y_plane(data, width, height, stride)
  if width % 2 or height % 2:
    raise ValueError("NV12 width and height must be even")
  if uv_offset < stride * height:
    raise ValueError("NV12 UV plane overlaps the visible Y plane")
  raw = np.frombuffer(data, dtype=np.uint8)
  uv_end = uv_offset + stride * (height // 2)
  if raw.size < uv_end:
    raise ValueError(f"short NV12 UV plane: {raw.size} < {uv_end}")
  uv = raw[uv_offset:uv_end].reshape(height // 2, stride)[:, :width]
  return np.vstack((y, uv))
