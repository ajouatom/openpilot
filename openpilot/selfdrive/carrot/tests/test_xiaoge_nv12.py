import numpy as np
import pytest

from openpilot.selfdrive.carrot.xiaoge.nv12 import nv12_y_plane, pack_nv12


def test_camera_allocation_with_reported_size_and_stride():
  # The camera allocator reserves aligned scanlines and extra bytes beyond NV12.
  width, height, stride = 1344, 760, 1408
  uv_offset = stride * 768
  raw = np.full(2_428_928, 255, dtype=np.uint8)
  y = raw[:stride * height].reshape(height, stride)
  y[:, :width] = 80
  uv = raw[uv_offset:uv_offset + stride * (height // 2)].reshape(height // 2, stride)
  uv[:, :width:2] = 90
  uv[:, 1:width:2] = 240

  # Reproduce the exact old failure from the vehicle's diagnostics page.
  with pytest.raises(ValueError, match="cannot reshape array of size 2428928"):
    raw.reshape(-1, stride)

  packed = pack_nv12(memoryview(raw), width, height, stride, uv_offset)

  assert packed.shape == (1140, 1344)
  assert packed.flags.c_contiguous
  np.testing.assert_array_equal(packed[:height], np.full((height, width), 80, dtype=np.uint8))
  np.testing.assert_array_equal(packed[height:, ::2], np.full((height // 2, width // 2), 90, dtype=np.uint8))
  np.testing.assert_array_equal(packed[height:, 1::2], np.full((height // 2, width // 2), 240, dtype=np.uint8))


@pytest.mark.parametrize("padding,gap,trailer", [(0, 0, 0), (2, 0, 1), (2, 7, 13), (4, 24, 4097)])
def test_nv12_planes_ignore_row_plane_and_trailing_padding(padding, gap, trailer):
  width, height = 8, 4
  stride = width + padding
  uv_offset = stride * height + gap
  raw = np.full(uv_offset + stride * (height // 2) + trailer, 255, dtype=np.uint8)
  expected_y = np.arange(width * height, dtype=np.uint8).reshape(height, width)
  expected_uv = np.arange(width * height // 2, dtype=np.uint8).reshape(height // 2, width) + 90
  raw[:stride * height].reshape(height, stride)[:, :width] = expected_y
  raw[uv_offset:uv_offset + stride * (height // 2)].reshape(height // 2, stride)[:, :width] = expected_uv

  np.testing.assert_array_equal(nv12_y_plane(memoryview(raw), width, height, stride), expected_y)
  np.testing.assert_array_equal(pack_nv12(memoryview(raw), width, height, stride, uv_offset), np.vstack((expected_y, expected_uv)))


@pytest.mark.parametrize("size,width,height,stride,uv_offset,error", [
  (24, 0, 4, 4, 16, "dimensions"),
  (24, 4, 4, 3, 16, "dimensions"),
  (15, 4, 4, 4, 16, "short NV12 Y plane"),
  (24, 3, 4, 4, 16, "must be even"),
  (24, 4, 3, 4, 16, "must be even"),
  (24, 4, 4, 4, 15, "overlaps"),
  (23, 4, 4, 4, 16, "short NV12 UV plane"),
])
def test_invalid_camera_layouts_are_rejected(size, width, height, stride, uv_offset, error):
  with pytest.raises(ValueError, match=error):
    pack_nv12(bytes(size), width, height, stride, uv_offset)
