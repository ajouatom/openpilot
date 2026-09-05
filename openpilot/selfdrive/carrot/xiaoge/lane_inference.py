#!/usr/bin/env python3
"""
Python/NumPy implementation of the ONNX lane segmentation inference engine
migrated from the drivingassist Android application (LaneInferenceEngine.kt & LaneResultSelector.kt).
"""

from pathlib import Path
import numpy as np

try:
  import cv2
except ImportError:
  cv2 = None

DEFAULT_LANE_MODEL_PATH = Path(__file__).resolve().parent / "assets" / "lane.onnx"

INPUT_SIZE = 416
PROTO_SIZE = 104
NUM_CLASSES = 6
MASK_COEFFICIENTS = 32
CONFIDENCE_THRESHOLD = 0.25
IOU_THRESHOLD = 0.5
CENTER_X_52 = PROTO_SIZE / 2.0  # 52.0

# Class mappings: 0, 2, 5 -> Solid (1), 1 -> Dashed (0), 3, 4 -> Ignored
SOLID_CLASSES = {0, 2, 5}
DASHED_CLASSES = {1}
IGNORED_CLASSES = {3, 4}


def class_to_type(class_id: int) -> int:
  if class_id in SOLID_CLASSES:
    return 1  # Solid
  if class_id in DASHED_CLASSES:
    return 0  # Dashed
  return -1  # Unavailable / Unknown


class LaneCandidate:
  def __init__(self, class_id: int, score: float, bottom: int, center_at_bottom: float, mask: np.ndarray | None = None):
    self.class_id = class_id
    self.score = score
    self.bottom = bottom
    self.center_at_bottom = center_at_bottom
    self.lane_type = class_to_type(class_id)
    self.mask = mask


def select_lane_results(candidates: list[LaneCandidate]) -> tuple[int, int, float, float]:
  """
  Selects the nearest left and right lane boundaries using distance from image bottom-center.
  Returns: (left_type, right_type, left_conf, right_conf)
  """
  left_candidates = [c for c in candidates if c.center_at_bottom < CENTER_X_52]
  right_candidates = [c for c in candidates if c.center_at_bottom >= CENTER_X_52]

  def nearest(side_candidates: list[LaneCandidate]) -> tuple[int, float]:
    if not side_candidates:
      return -1, 0.0
    best = min(
      side_candidates,
      key=lambda c: (c.center_at_bottom - CENTER_X_52) ** 2 + 2.0 * (c.bottom - (PROTO_SIZE - 1)) ** 2
    )
    return best.lane_type, best.score

  left_type, left_conf = nearest(left_candidates)
  right_type, right_conf = nearest(right_candidates)
  return left_type, right_type, left_conf, right_conf


def non_maximum_suppression(
  boxes_left: np.ndarray,
  boxes_top: np.ndarray,
  boxes_right: np.ndarray,
  boxes_bottom: np.ndarray,
  scores: np.ndarray,
  class_ids: np.ndarray,
  iou_threshold: float,
) -> np.ndarray:
  """Keep the highest-scoring non-overlapping boxes independently for each class."""
  areas = (boxes_right - boxes_left) * (boxes_bottom - boxes_top)
  kept_indices: list[int] = []
  for class_id in np.unique(class_ids):
    indices = np.flatnonzero(class_ids == class_id)
    order = indices[np.argsort(-scores[indices], kind="stable")]
    while len(order):
      current = order[0]
      kept_indices.append(int(current))
      remaining = order[1:]
      if not len(remaining):
        break
      intersection_width = np.maximum(0.0, np.minimum(boxes_right[current], boxes_right[remaining]) - np.maximum(boxes_left[current], boxes_left[remaining]))
      intersection_height = np.maximum(0.0, np.minimum(boxes_bottom[current], boxes_bottom[remaining]) - np.maximum(boxes_top[current], boxes_top[remaining]))
      intersection = intersection_width * intersection_height
      iou = intersection / (areas[current] + areas[remaining] - intersection + 1e-6)
      order = remaining[iou <= iou_threshold]
  return np.array(sorted(kept_indices, key=lambda index: scores[index], reverse=True), dtype=np.intp)


class LaneInference:
  def __init__(self, model_path: Path = DEFAULT_LANE_MODEL_PATH):
    self.model_path = model_path
    self.net = None
    self.valid = False
    self.error = ""

  def load(self) -> bool:
    if cv2 is None:
      self.error = "OpenCV (cv2) is missing"
      self.valid = False
      return False
    if not self.model_path.is_file():
      self.error = f"Model file not found: {self.model_path}"
      self.valid = False
      return False
    try:
      self.net = cv2.dnn.readNetFromONNX(str(self.model_path))
      self.valid = True
      self.error = ""
      return True
    except Exception as exc:
      self.error = str(exc)
      self.valid = False
      return False

  def preprocess_image(self, image: np.ndarray, width: int, height: int) -> np.ndarray:
    """
    Reproduces the image contract from xiaoge_data.py and the Android client:
    centered square crop of the NV12 Y plane, resized to 416x416 grayscale JPEG
    semantics, expanded into equal RGB channels and normalized to NCHW float32.
    """
    if image.shape[0] == height + height // 2 and image.shape[1] == width:
      y_plane = image[:height, :width]
      crop_size = min(width, height)
      start_x = (width - crop_size) // 2
      start_y = (height - crop_size) // 2
      gray = np.ascontiguousarray(y_plane[start_y:start_y + crop_size, start_x:start_x + crop_size])
    elif image.ndim == 2:
      gray = image[:height, :width]
    elif image.ndim == 3 and image.shape[2] == 3:
      gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
      raise ValueError(f"unsupported lane image shape: {image.shape}")

    if gray.shape != (INPUT_SIZE, INPUT_SIZE):
      gray = cv2.resize(gray, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_LINEAR)

    normalized = gray.astype(np.float32) / 255.0
    return np.broadcast_to(normalized, (3, INPUT_SIZE, INPUT_SIZE)).copy()[np.newaxis, ...]

  def infer(self, image: np.ndarray, width: int, height: int, conf_thresh: float = CONFIDENCE_THRESHOLD, iou_thresh: float = IOU_THRESHOLD) -> dict:
    if not self.valid or self.net is None:
      return {
        "leftLine": -1,
        "rightLine": -1,
        "leftConf": 0.0,
        "rightConf": 0.0,
        "valid": False,
        "error": self.error or "Model not loaded",
        "candidatesCount": 0,
      }

    try:
      blob = self.preprocess_image(image, width, height)
      self.net.setInput(blob)
      out_names = self.net.getUnconnectedOutLayersNames()
      outputs = dict(zip(out_names, self.net.forward(out_names), strict=True))
      predictions = outputs["output0"][0]  # shape (42, 3549)
      prototypes = outputs["output1"][0]   # shape (32, 104, 104)

      # Extract scores across 6 classes for 3549 anchors
      class_scores = predictions[4:10, :]  # shape (6, 3549)
      max_scores = np.max(class_scores, axis=0)  # shape (3549,)
      class_ids = np.argmax(class_scores, axis=0)  # shape (3549,)

      mask_valid = max_scores >= conf_thresh
      if not np.any(mask_valid):
        return {
          "leftLine": -1,
          "rightLine": -1,
          "leftConf": 0.0,
          "rightConf": 0.0,
          "valid": True,
          "error": "",
          "candidatesCount": 0,
        }

      valid_indices = np.where(mask_valid)[0]
      boxes_cx = predictions[0, valid_indices]
      boxes_cy = predictions[1, valid_indices]
      boxes_w = predictions[2, valid_indices]
      boxes_h = predictions[3, valid_indices]
      scores = max_scores[valid_indices]
      c_ids = class_ids[valid_indices]
      coeffs = predictions[10:42, valid_indices].T  # shape (N, 32)

      boxes_left = boxes_cx - boxes_w * 0.5
      boxes_top = boxes_cy - boxes_h * 0.5
      boxes_right = boxes_cx + boxes_w * 0.5
      boxes_bottom = boxes_cy + boxes_h * 0.5
      kept_indices = non_maximum_suppression(
        boxes_left, boxes_top, boxes_right, boxes_bottom, scores, c_ids, iou_thresh,
      )

      candidates = []
      for k_idx in kept_indices:
        cid = int(c_ids[k_idx])
        if cid in IGNORED_CLASSES:
          continue

        score = float(scores[k_idx])
        coeff = coeffs[k_idx]  # shape (32,)

        # Crop box coordinates in 104x104 space
        left_p = int(np.clip((boxes_left[k_idx] / INPUT_SIZE) * PROTO_SIZE, 0, PROTO_SIZE - 1))
        top_p = int(np.clip((boxes_top[k_idx] / INPUT_SIZE) * PROTO_SIZE, 0, PROTO_SIZE - 1))
        right_p = int(np.clip((boxes_right[k_idx] / INPUT_SIZE) * PROTO_SIZE, 0, PROTO_SIZE - 1))
        bottom_p = int(np.clip((boxes_bottom[k_idx] / INPUT_SIZE) * PROTO_SIZE, 0, PROTO_SIZE - 1))

        if right_p <= left_p or bottom_p <= top_p:
          continue

        # Compute mask within bounding box crop
        proto_crop = prototypes[:, top_p:bottom_p + 1, left_p:right_p + 1]  # shape (32, H, W)
        mask_crop = np.tensordot(coeff, proto_crop, axes=(0, 0)) > 0.0      # shape (H, W) boolean

        if not np.any(mask_crop):
          continue

        # Find lowest non-zero row in mask crop
        row_has_mask = np.any(mask_crop, axis=1)
        nonzero_rows = np.where(row_has_mask)[0]
        if len(nonzero_rows) == 0:
          continue

        lowest_row_in_crop = nonzero_rows[-1]
        bottom_y = top_p + lowest_row_in_crop

        if bottom_y < PROTO_SIZE // 2:  # bottom < 52
          continue

        lowest_row_cols = np.where(mask_crop[lowest_row_in_crop])[0]
        center_x_in_crop = np.mean(lowest_row_cols)
        center_at_bottom = left_p + center_x_in_crop

        candidates.append(LaneCandidate(cid, score, bottom_y, center_at_bottom))

      left_type, right_type, left_conf, right_conf = select_lane_results(candidates)

      return {
        "leftLine": left_type,
        "rightLine": right_type,
        "leftConf": round(left_conf, 3),
        "rightConf": round(right_conf, 3),
        "valid": True,
        "error": "",
        "candidatesCount": len(candidates),
      }

    except Exception as exc:
      return {
        "leftLine": -1,
        "rightLine": -1,
        "leftConf": 0.0,
        "rightConf": 0.0,
        "valid": False,
        "error": str(exc),
        "candidatesCount": 0,
      }
