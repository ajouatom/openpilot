#!/usr/bin/env python3
"""Build standalone modeld warp pkls for the Carrot model selector.

Recent openpilot builds compile modeld warp JITs into the unified modeld pkl via
`selfdrive/modeld/compile_modeld.py` and no longer ship standalone
`selfdrive/modeld/compile_warp.py` or `warp_*_tinygrad.pkl` artifacts. The
Carrot split-model runner still expects the legacy standalone warp callable, so
this compatibility compiler recreates that artifact with the current warp math.
"""
from __future__ import annotations

import argparse
import os
import pickle
import time
from collections import namedtuple

import numpy as np

from tinygrad.device import Device
from tinygrad.engine.jit import TinyJit
from tinygrad.helpers import Context
from tinygrad.tensor import Tensor

from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

NV12Frame = namedtuple("NV12Frame", ['width', 'height', 'stride', 'y_height', 'uv_height', 'size'])
WARP_DEV = os.getenv('WARP_DEV')


def _parse_size(s):
  w, h = s.lower().split('x')
  return int(w), int(h)


def warp_perspective_tinygrad(src_flat, M_inv, dst_shape, src_shape, stride_pad, border_fill_val=None):
  w_dst, h_dst = dst_shape
  h_src, w_src = src_shape

  x = Tensor.arange(w_dst).reshape(1, w_dst).expand(h_dst, w_dst).reshape(-1)
  y = Tensor.arange(h_dst).reshape(h_dst, 1).expand(h_dst, w_dst).reshape(-1)

  # inline 3x3 matmul as elementwise to avoid reduce op (enables fusion with gather)
  src_x = M_inv[0, 0] * x + M_inv[0, 1] * y + M_inv[0, 2]
  src_y = M_inv[1, 0] * x + M_inv[1, 1] * y + M_inv[1, 2]
  src_w = M_inv[2, 0] * x + M_inv[2, 1] * y + M_inv[2, 2]

  src_x = src_x / src_w
  src_y = src_y / src_w

  x_round = Tensor.round(src_x)
  y_round = Tensor.round(src_y)
  x_nn_clipped = x_round.clip(0, w_src - 1).cast('int')
  y_nn_clipped = y_round.clip(0, h_src - 1).cast('int')
  idx = y_nn_clipped * (w_src + stride_pad) + x_nn_clipped
  sampled = src_flat[idx]

  if border_fill_val is None:
    return sampled

  in_bounds = ((x_round >= 0) & (x_round <= w_src - 1) &
               (y_round >= 0) & (y_round <= h_src - 1)).cast(sampled.dtype)
  return sampled * in_bounds + Tensor(border_fill_val, dtype=sampled.dtype) * (1 - in_bounds)


def frames_to_tensor(frames):
  H = (frames.shape[0] * 2) // 3
  W = frames.shape[1]
  in_img1 = Tensor.cat(frames[0:H:2, 0::2],
                       frames[1:H:2, 0::2],
                       frames[0:H:2, 1::2],
                       frames[1:H:2, 1::2],
                       frames[H:H+H//4].reshape((H//2, W//2)),
                       frames[H+H//4:H+H//2].reshape((H//2, W//2)), dim=0).reshape((6, H//2, W//2))
  return in_img1


def make_frame_prepare(nv12: NV12Frame, model_w, model_h):
  cam_w, cam_h, stride, y_height, uv_height, _ = nv12
  uv_offset = stride * y_height
  stride_pad = stride - cam_w

  def frame_prepare_tinygrad(input_frame, M_inv):
    # UV_SCALE @ M_inv @ UV_SCALE_INV simplifies to elementwise scaling
    M_inv_uv = M_inv * Tensor([[1.0, 1.0, 0.5], [1.0, 1.0, 0.5], [2.0, 2.0, 1.0]], device=WARP_DEV)
    # deinterleave NV12 UV plane (UVUV... -> separate U, V)
    uv = input_frame[uv_offset:uv_offset + uv_height * stride].reshape(uv_height, stride)
    with Context(SPLIT_REDUCEOP=0):
      y = warp_perspective_tinygrad(input_frame[:cam_h*stride],
                                    M_inv, (model_w, model_h),
                                    (cam_h, cam_w), stride_pad).realize()
      u = warp_perspective_tinygrad(uv[:cam_h//2, :cam_w:2].flatten(),
                                    M_inv_uv, (model_w//2, model_h//2),
                                    (cam_h//2, cam_w//2), 0).realize()
      v = warp_perspective_tinygrad(uv[:cam_h//2, 1:cam_w:2].flatten(),
                                    M_inv_uv, (model_w//2, model_h//2),
                                    (cam_h//2, cam_w//2), 0).realize()
    yuv = y.cat(u).cat(v).reshape((model_h * 3 // 2, model_w))
    return frames_to_tensor(yuv)
  return frame_prepare_tinygrad


def make_update_img_input(frame_prepare):
  def update_img_input_tinygrad(frame_buffer, frame, M_inv):
    M_inv = M_inv.to(Device.DEFAULT).realize()
    new_img = frame_prepare(frame, M_inv)
    frame_buffer.assign(frame_buffer[6:].cat(new_img, dim=0).contiguous())
    return Tensor.cat(frame_buffer[:6], frame_buffer[-6:], dim=0).contiguous().reshape(1, 12, new_img.shape[1], new_img.shape[2])
  return update_img_input_tinygrad


def make_update_both_imgs(frame_prepare):
  update_img = make_update_img_input(frame_prepare)

  def update_both_imgs_tinygrad(calib_img_buffer, new_img, M_inv,
                                calib_big_img_buffer, new_big_img, M_inv_big):
    calib_img_pair = update_img(calib_img_buffer, new_img, M_inv)
    calib_big_img_pair = update_img(calib_big_img_buffer, new_big_img, M_inv_big)
    return calib_img_pair, calib_big_img_pair
  return update_both_imgs_tinygrad


def compile_modeld_legacy_warp(nv12: NV12Frame, model_w: int, model_h: int, output: str) -> None:
  print(f"Compiling legacy modeld warp for {nv12.width}x{nv12.height} -> {model_w}x{model_h}...")

  frame_prepare = make_frame_prepare(nv12, model_w, model_h)
  update_img_jit = TinyJit(make_update_both_imgs(frame_prepare), prune=True)

  img_buffer_shape = (30, model_h // 2, model_w // 2)
  full_buffer = Tensor.zeros(img_buffer_shape, dtype='uint8').contiguous().realize()
  big_full_buffer = Tensor.zeros(img_buffer_shape, dtype='uint8').contiguous().realize()
  for i in range(3):
    img_inputs = [full_buffer,
                  Tensor(np.random.randint(0, 256, nv12.size, dtype=np.uint8)).realize(),
                  Tensor(Tensor.randn(3, 3).mul(8).realize().numpy(), device='NPY')]
    big_img_inputs = [big_full_buffer,
                      Tensor(np.random.randint(0, 256, nv12.size, dtype=np.uint8)).realize(),
                      Tensor(Tensor.randn(3, 3).mul(8).realize().numpy(), device='NPY')]
    inputs = img_inputs + big_img_inputs
    Device.default.synchronize()
    st = time.perf_counter()
    update_img_jit(*inputs)
    mt = time.perf_counter()
    Device.default.synchronize()
    et = time.perf_counter()
    print(f"  [{i+1}/3] enqueue {(mt-st)*1e3:6.2f} ms -- total {(et-st)*1e3:6.2f} ms")

  with open(output, "wb") as f:
    pickle.dump(update_img_jit, f)
  print(f"  Saved to {output} ({os.path.getsize(output) / 1e6:.2f} MB)")


def main() -> None:
  p = argparse.ArgumentParser()
  p.add_argument('--camera-resolution', type=_parse_size, required=True, help='camera resolution WxH')
  p.add_argument('--model-size', type=_parse_size, required=True, help='model input WxH')
  p.add_argument('--output', required=True)
  args = p.parse_args()

  cam_w, cam_h = args.camera_resolution
  model_w, model_h = args.model_size
  nv12 = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
  compile_modeld_legacy_warp(nv12, model_w, model_h, args.output)


if __name__ == "__main__":
  main()
