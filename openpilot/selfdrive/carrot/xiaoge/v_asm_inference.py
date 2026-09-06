from pathlib import Path

import numpy as np

try:
  import cv2
except ModuleNotFoundError as error:
  if error.name == "cv2":
    raise SystemExit(
      "V-ASM requires OpenCV. Activate the openpilot virtual environment, then run " +
      "`python -m pip install --no-cache-dir -r openpilot/selfdrive/carrot/xiaoge/requirements.txt`."
    ) from None
  raise


MODEL_INPUT_HEIGHT = 256
MODEL_INPUT_WIDTH = 352
HYSTERESIS_ON = 0.65
HYSTERESIS_OFF = 0.25
ASSETS_DIR = Path(__file__).resolve().parent / "assets"
DEFAULT_MODEL_PATH = ASSETS_DIR / "v_asm_model.onnx"


class VASMInference:
  def __init__(self, model_path: Path = DEFAULT_MODEL_PATH):
    self.model_path = model_path
    self.net = None
    self.valid = False
    self.error = ""
    self.frame_size = (0, 0)
    self.config_size = (0, 0)
    self.raw_polygons = {"left": None, "right": None}
    self.bboxes = {"left": None, "right": None}
    self.masks = {"left": None, "right": None}
    self.reset()

  def load(self) -> bool:
    if not self.model_path.is_file():
      self.error = f"model missing: {self.model_path}"
      return False
    try:
      self.net = cv2.dnn.readNetFromONNX(str(self.model_path))
      self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
      self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    except cv2.error as error:
      self.error = f"could not load model: {error}"
      return False
    self.valid = True
    self.error = ""
    return True

  def reset(self) -> None:
    self.scores = {"left": 0.0, "right": 0.0}
    self.active = {"left": False, "right": False}
    self.confidence = {"left": 0.0, "right": 0.0}

  @property
  def configured_sides(self) -> tuple[str, ...]:
    return tuple(side for side in ("left", "right") if self.raw_polygons[side] is not None)

  def load_config(self, config: dict) -> None:
    self.reset()
    self.frame_size = (0, 0)
    self.config_size = (int(config["width"]), int(config["height"]))
    for side in ("left", "right"):
      polygon = config.get(f"poly_{side}", [])
      self.raw_polygons[side] = np.array(polygon, dtype=np.float32) if len(polygon) >= 3 else None
      self.bboxes[side] = None
      self.masks[side] = None

  def _prepare_geometry(self, height: int, width: int) -> None:
    if self.frame_size == (height, width):
      return
    self.frame_size = (height, width)
    config_width, config_height = self.config_size
    scale_x = width / config_width
    scale_y = height / config_height
    for side in ("left", "right"):
      raw_polygon = self.raw_polygons[side]
      if raw_polygon is None:
        continue
      polygon = raw_polygon.copy()
      polygon[:, 0] *= scale_x
      polygon[:, 1] *= scale_y
      x, y, crop_width, crop_height = cv2.boundingRect(polygon.astype(np.int32))
      x = max(0, min((x // 2) * 2, width - 2))
      y = max(0, min((y // 2) * 2, height - 2))
      crop_width = max(2, min(((crop_width + 1) // 2) * 2, width - x))
      crop_height = max(2, min(((crop_height + 1) // 2) * 2, height - y))
      crop_width = (crop_width // 2) * 2
      crop_height = (crop_height // 2) * 2
      self.bboxes[side] = (x, y, crop_width, crop_height)
      mask = np.zeros((crop_height, crop_width), dtype=np.uint8)
      cv2.fillPoly(mask, [polygon.astype(np.int32) - [x, y]], 255)
      self.masks[side] = mask

  def _confidence(self, nv12: np.ndarray, frame_height: int, side: str) -> float:
    bbox = self.bboxes[side]
    if bbox is None or self.net is None:
      return 0.0
    x, y, width, height = bbox
    y_crop = nv12[y:y + height, x:x + width]
    uv_crop = nv12[frame_height + y // 2:frame_height + (y + height) // 2, x:x + width]
    rgb = cv2.cvtColor(np.vstack((y_crop, uv_crop)), cv2.COLOR_YUV2RGB_NV12)
    rgb = cv2.bitwise_and(rgb, rgb, mask=self.masks[side])
    rgb = cv2.resize(rgb, (MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT), interpolation=cv2.INTER_LINEAR)
    blob = np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))[None, :]
    self.net.setInput(blob)
    predictions = np.squeeze(self.net.forward())
    if predictions.ndim == 2:
      if predictions.shape[0] < predictions.shape[1]:
        predictions = predictions.T
      if predictions.shape[1] >= 6:
        predictions = predictions[np.round(predictions[:, 5]).astype(int) == 0]
        return float(np.max(predictions[:, 4])) if len(predictions) else 0.0
      return float(np.max(predictions[:, 4 if predictions.shape[1] >= 5 else 0]))
    return float(np.max(predictions)) if predictions.size else 0.0

  def update(self, nv12: np.ndarray, width: int, height: int, side: str, threshold: float, smoothing_seconds: float, dt: float) -> None:
    self._prepare_geometry(height, width)
    confidence = self._confidence(nv12, height, side)
    score_delta = min(1.0, dt / max(smoothing_seconds, 0.001))
    self.scores[side] = min(1.0, self.scores[side] + score_delta) if confidence >= threshold else max(0.0, self.scores[side] - score_delta)
    self.confidence[side] = confidence
    if self.scores[side] >= HYSTERESIS_ON:
      self.active[side] = True
    elif self.scores[side] <= HYSTERESIS_OFF:
      self.active[side] = False
