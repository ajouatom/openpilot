"""C4 Jetlink warp with direct NV12 sampling in the model's six-channel layout."""


def make_warp(nv12, model_w, model_h, frame_skip):
  from tinygrad import Tensor
  from tinygrad.helpers import Context
  from openpilot.selfdrive.modeld.compile_modeld import WARP_DEV

  cam_w, cam_h, stride, y_height, _, _ = nv12

  def prepare(frame, matrix):
    channel = Tensor.arange(6).reshape(6, 1, 1)
    chroma = channel >= 4
    col = Tensor.arange(model_w // 2).reshape(1, 1, model_w // 2)
    row = Tensor.arange(model_h // 2).reshape(1, model_h // 2, 1)
    # Same packing as frames_to_tensor: Y00, Y10, Y01, Y11, U, V.
    x = chroma.where(col, col * 2 + channel // 2)
    y = chroma.where(row, row * 2 + channel % 2)
    # Preserve UV_SCALE @ matrix @ UV_SCALE_INV and the original operation
    # order, rounding and border clamping; only the gather/layout are fused.
    translation = chroma.where(.5, 1.)
    projective = chroma.where(2., 1.)
    src_x = matrix[0, 0] * x + matrix[0, 1] * y + matrix[0, 2] * translation
    src_y = matrix[1, 0] * x + matrix[1, 1] * y + matrix[1, 2] * translation
    src_w = (matrix[2, 0] * projective) * x + (matrix[2, 1] * projective) * y + matrix[2, 2]
    sx = Tensor.round(src_x / src_w).maximum(0).minimum(chroma.where(cam_w // 2 - 1, cam_w - 1)).cast('int')
    sy = Tensor.round(src_y / src_w).maximum(0).minimum(chroma.where(cam_h // 2 - 1, cam_h - 1)).cast('int')
    index = chroma.where(stride * y_height + sy * stride + sx * 2 + channel - 4, sy * stride + sx)
    return frame[index].realize()

  def warp(tfm, big_tfm, frame, big_frame):
    tfm, big_tfm = tfm.to(WARP_DEV), big_tfm.to(WARP_DEV)
    Tensor.realize(tfm, big_tfm)
    with Context(SPLIT_REDUCEOP=0):
      return Tensor.cat(prepare(frame, tfm).unsqueeze(0), prepare(big_frame, big_tfm).unsqueeze(0))

  return warp


def validation_matrices():
  import numpy as np
  rng = np.random.default_rng(7527)
  probes = [np.eye(3), [[2.3, .01, 20.2], [-.02, 2.1, 40.3], [.0001, -.0002, 1]],
            [[1, 0, -200], [0, 1, -100], [0, 0, 1]], [[1, 0, 3000], [0, 1, 2000], [0, 0, 1]]]
  probes += [[[1, 0, x], [0, 1, x], [0, 0, 1]] for x in (.49999, .5, .50001, 1., 1.5, -.5, -1.5)]
  probes += [np.array([[2., 0, 20], [0, 2., 40], [0, 0, 1]]) + rng.normal(size=(3, 3)) *
             np.array([[.1, .01, 10], [.01, .1, 10], [.0001, .0001, 0]]) for _ in range(16)]
  return probes


def validated_warp(reference, nv12, transforms, inputs, frame_skip, device_type):
  """Keep C3 unchanged; on C4 accept the candidate only after exact GPU parity."""
  if device_type != 'mici':
    return reference
  import numpy as np
  from tinygrad import Tensor, TinyJit
  from openpilot.common.swaglog import cloudlog

  try:
    rng = np.random.default_rng(7527)
    frames = {key: Tensor(rng.integers(0, 256, nv12.size, dtype=np.uint8), device='QCOM').realize()
              for key in ('frame', 'big_frame')}
    candidate = TinyJit(make_warp(nv12, 512, 256, frame_skip))
    for _ in range(3):
      candidate(**inputs, **frames).numpy()
    probes = validation_matrices()
    for index, matrix in enumerate(probes):
      transforms['tfm'][:] = matrix
      transforms['big_tfm'][:] = np.asarray(matrix) * np.array([[1.01, 1, 1], [1, .99, 1], [1, 1, 1]])
      expected = reference(**inputs, **frames).numpy()
      actual = candidate(**inputs, **frames).numpy()
      if actual.shape != expected.shape or actual.dtype != expected.dtype or not np.array_equal(actual, expected):
        raise ValueError(f'fused camera warp pixel validation failed at probe {index}')
    cloudlog.warning('Jetlink fused C4 warp verified: %d probes, %d pixels each', len(probes), actual.size)
    return candidate
  except Exception:
    cloudlog.exception('Jetlink fused warp unavailable; retaining original camera warp')
    return reference
  finally:
    for matrix in transforms.values():
      matrix[:] = np.eye(3, dtype=np.float32)
