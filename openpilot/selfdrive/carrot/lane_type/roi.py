"""The same 6–46 metre lane strips for training and live NV12 inference."""
import numpy as np

ROI_WIDTH = 64
ROI_HEIGHT = 192
NEAR_M = 6.0
FAR_M = 46.0
HALF_WIDTH_M = 0.45
MIN_LANE_PROBABILITY = 0.5


def lane_maps(line, probability, rpy, intrinsics, width, height):
  """Return calibrated pixel maps, or None when a complete strip is unavailable.

  modelV2 lane z already includes the road's camera-relative height, as in UI.
  Rows run far to near; preserve gaps along forty metres instead of a small box.
  """
  from openpilot.common.transformations.orientation import rot_from_euler
  from openpilot.common.transformations.camera import view_frame_from_device_frame

  xyz = np.array([line.x, line.y, line.z], dtype=np.float64)
  rpy = np.asarray(rpy, dtype=np.float64)
  k = np.asarray(intrinsics, dtype=np.float64)
  if (xyz.ndim != 2 or xyz.shape[0] != 3 or xyz.shape[1] < 3 or not np.isfinite(xyz).all() or
      rpy.shape != (3,) or not np.isfinite(rpy).all() or k.shape != (3, 3) or not np.isfinite(k).all() or
      not np.isfinite(probability) or probability < MIN_LANE_PROBABILITY or
      np.any(np.diff(xyz[0]) <= 0) or xyz[0, 0] > NEAR_M or xyz[0, -1] < FAR_M):
    return None
  distances = np.linspace(FAR_M, NEAR_M, ROI_HEIGHT)
  lateral = np.interp(distances, xyz[0], xyz[1])
  vertical = np.interp(distances, xyz[0], xyz[2])
  points = np.stack((np.broadcast_to(distances[:, None], (ROI_HEIGHT, ROI_WIDTH)),
                     lateral[:, None] + np.linspace(-HALF_WIDTH_M, HALF_WIDTH_M, ROI_WIDTH),
                     np.broadcast_to(vertical[:, None], (ROI_HEIGHT, ROI_WIDTH))), axis=-1)
  projection = k @ view_frame_from_device_frame @ rot_from_euler(rpy)
  pixels = points @ projection.T
  if (pixels[..., 2] <= 0).any():
    return None
  mx = (pixels[..., 0] / pixels[..., 2]).astype(np.float32)
  my = (pixels[..., 1] / pixels[..., 2]).astype(np.float32)
  # A cropped stripe can turn a solid line into a false gap: reject it entirely.
  if not ((mx >= 0) & (mx < width-1) & (my >= 0) & (my < height-1)).all():
    return None
  return mx, my


def rgb_strip(image, maps):
  import cv2
  return cv2.remap(image, *maps, cv2.INTER_LINEAR)


def nv12_strip(data, width, height, stride, uv_offset, maps):
  """Sample shared NV12 planes directly; copy only a 64x192 RGB strip.

  Never convert, pack or copy a complete camera frame. Chroma is sampled from
  its interleaved two-channel view, avoiding a full-plane U/V strided copy.
  """
  import cv2
  if width <= 0 or height <= 0 or width % 2 or height % 2 or stride < width or stride % 2 or uv_offset < stride*height:
    raise ValueError('invalid NV12 layout')
  raw = np.frombuffer(data, dtype=np.uint8)
  end = uv_offset + stride*(height//2)
  if raw.size < end:
    raise ValueError('truncated NV12 allocation')
  y = raw[:stride*height].reshape(height, stride)[:, :width]
  uv = raw[uv_offset:end].reshape(height//2, stride//2, 2)[:, :width//2]
  mx, my = maps
  lum = cv2.remap(y, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).astype(np.float32)-16
  chroma = cv2.remap(uv, mx*.5, my*.5, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).astype(np.float32)-128
  u, v = chroma[..., 0], chroma[..., 1]
  return np.clip(np.stack((1.164383*lum+1.596027*v,
                           1.164383*lum-.391762*u-.812968*v,
                           1.164383*lum+2.017232*u), axis=-1), 0, 255).astype(np.uint8)


def input_blob(strips):
  """RGB/ImageNet normalization shared with training; batch only valid sides."""
  image = np.stack(strips).astype(np.float32)/255.0
  image = (image-np.array([.485, .456, .406], dtype=np.float32))/np.array([.229, .224, .225], dtype=np.float32)
  return np.ascontiguousarray(image.transpose(0, 3, 1, 2))
