import io
import json
import os
import pickle
import shutil
import struct
import tempfile
from collections.abc import Collection
from pathlib import Path
from typing import TypeVar

from openpilot.common.file_chunker import get_manifest_path
from openpilot.selfdrive.modeld.big_model import active_manifest, read_state

MODELS_DIR = Path(__file__).resolve().parent / 'models'
TG_INPUT_DEVICES_PATH = MODELS_DIR / 'tg_input_devices.json'
USBGPU_USB_IDS = ((0xADD1, 0x0001), (0x3801, 0x0001))
VisionStreamT = TypeVar("VisionStreamT")


def select_vision_streams(available_streams: Collection[VisionStreamT], road_stream: VisionStreamT,
                          wide_stream: VisionStreamT, use_wide_camera: bool) -> tuple[VisionStreamT | None, bool]:
  """Select modeld camera inputs without waiting on a disabled wide camera."""
  if road_stream in available_streams:
    return road_stream, use_wide_camera and wide_stream in available_streams
  if use_wide_camera and wide_stream in available_streams:
    return wide_stream, False
  return None, False


def _default_tg_input_devices(process_name: str, usbgpu: bool):
  backend = os.getenv("DEV", "QCOM")
  defaults = {
    'openpilot.selfdrive.modeld.modeld': {
      'default': {'WARP_DEV': backend, 'QUEUE_DEV': backend},
      'usbgpu': {'WARP_DEV': backend, 'QUEUE_DEV': 'AMD'},
    },
    'openpilot.selfdrive.modeld.dmonitoringmodeld': {
      'default': {'DEV': backend},
    },
  }
  return defaults[process_name]['default' if not usbgpu else 'usbgpu']


def get_tg_input_devices(process_name: str, usbgpu: bool):
  try:
    with open(TG_INPUT_DEVICES_PATH) as f:
      return json.load(f)[process_name]['default' if not usbgpu else 'usbgpu']
  except FileNotFoundError:
    return _default_tg_input_devices(process_name, usbgpu)

def modeld_pkl_path(usbgpu: bool, model_sha256: str | None = None):
  # carrot model selector: load a custom-compiled model dir instead of the built-in one
  models_dir = Path(override) if (override := os.getenv('MODELD_MODELS_DIR')) else MODELS_DIR
  if not usbgpu:
    return models_dir / 'driving_tinygrad.pkl'
  if model_sha256 is None:
    model = active_manifest()
    model_sha256 = model.sha256 if model is not None else 'unavailable'
  return models_dir / f'big_driving_{model_sha256[:16]}_tinygrad.pkl'

def dump_oob(obj, f):
  with tempfile.TemporaryFile(dir=".") as tmp:
    def buffer_callback(pb: pickle.PickleBuffer):
      m = pb.raw()
      tmp.write(struct.pack('<q', m.nbytes))
      tmp.write(m)
      pb.release() # keep peak ram at ~1 buffer
    stream = io.BytesIO()
    pickle.Pickler(stream, protocol=5, buffer_callback=buffer_callback).dump(obj)
    opcodes = stream.getvalue()
    f.write(struct.pack('<q', len(opcodes)))
    f.write(opcodes)
    tmp.seek(0)
    shutil.copyfileobj(tmp, f)

def load_oob(f):
  opcodes = f.read(struct.unpack('<q', f.read(8))[0])
  def buffers():
    while (h := f.read(8)):
      pb = pickle.PickleBuffer(bytearray(struct.unpack('<q', h)[0]))
      f.readinto(pb)
      yield pb
  return pickle.load(io.BytesIO(opcodes), buffers=buffers())

def usbgpu_present() -> bool:
  for d in Path("/sys/bus/usb/devices").glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      if usb_id in USBGPU_USB_IDS:
        return True
    except Exception:
      pass
  return False

def usbgpu_compiled_path() -> Path | None:
  state = read_state()
  for model in (state['active'], state['previous']):
    if model is None:
      continue
    path = modeld_pkl_path(usbgpu=True, model_sha256=model.sha256)
    if Path(get_manifest_path(path)).is_file():
      return path
  return None


def usbgpu_compiled() -> bool:
  return usbgpu_compiled_path() is not None
