"""Small display previews only; model camera buffers and transforms are untouched."""
import base64
import ctypes
import json
import os
from io import BytesIO
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

WIDTH, HEIGHT = 384, 240


def jpeg_preview(nv12):
  from PIL import Image
  raw = np.frombuffer(nv12, np.uint8)
  y = raw[:WIDTH * HEIGHT].reshape(HEIGHT, WIDTH)
  uv = raw[WIDTH * HEIGHT:].reshape(HEIGHT // 2, WIDTH)
  yuv = np.empty((HEIGHT, WIDTH, 3), np.uint8)
  yuv[:, :, 0] = y
  yuv[:, :, 1] = uv[:, 0::2].repeat(2, axis=0).repeat(2, axis=1)
  yuv[:, :, 2] = uv[:, 1::2].repeat(2, axis=0).repeat(2, axis=1)
  image = Image.fromarray(yuv, 'YCbCr')
  for quality in (50, 35, 20):
    output = BytesIO()
    image.save(output, format='JPEG', quality=quality)
    data = output.getvalue()
    if len(data) <= 20 * 1024:
      return data
  raise ValueError('display preview exceeds its USB budget')


def preview_nv12(frame):
  raw = np.frombuffer(frame.data, np.uint8)
  y = raw[:frame.uv_offset].reshape(-1, frame.stride)
  uv = raw[frame.uv_offset:frame.uv_offset + (frame.height // 2) * frame.stride].reshape(-1, frame.stride)
  # Preserve the entire camera field of view, independent of the model warp.
  yi = np.arange(HEIGHT) * frame.height // HEIGHT
  xi = np.arange(WIDTH) * frame.width // WIDTH
  uyi = np.arange(HEIGHT // 2) * (frame.height // 2) // (HEIGHT // 2)
  uxi = np.arange(WIDTH // 2) * (frame.width // 2) // (WIDTH // 2)
  small_y = y[np.ix_(yi, xi)]
  small_uv = np.empty((HEIGHT // 2, WIDTH), np.uint8)
  small_uv[:, 0::2] = uv[np.ix_(uyi, uxi * 2)]
  small_uv[:, 1::2] = uv[np.ix_(uyi, uxi * 2 + 1)]
  return small_y.tobytes() + small_uv.tobytes()


class CameraPublisher:
  def __init__(self):
    self.path = Path(f'/dev/shm/carrot-jetlink-cameras-{os.getpid()}.json')
    self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--publish', str(self.path)],
                                    stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

  @property
  def latest(self):
    try:
      data = json.loads(self.path.read_text())
      if 0 <= time.monotonic() - data['updated'] < .3:
        return data['cameras']
    except (OSError, ValueError, KeyError):
      pass
    return {}

  @staticmethod
  def run(path):
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    from openpilot.common.display_scheduling import DisplayScheduler
    from openpilot.common.params import Params
    # JPEG and VisionIPC run outside the inference owner's GIL. A low-priority
    # Python thread can otherwise hold that GIL while it is descheduled.
    ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0)
    if os.getppid() == 1:
      return
    scheduler = DisplayScheduler(7, enabled=True)
    params = Params()
    clients = {name: VisionIpcClient('camerad', stream, conflate=True) for name, stream in (
      ('road', VisionStreamType.VISION_STREAM_ROAD), ('wide', VisionStreamType.VISION_STREAM_WIDE_ROAD))}
    while True:
      start = time.monotonic()
      scheduler.update(params.get_bool('IsOnroad'))
      result = {}
      for name, client in clients.items():
        try:
          if not client.is_connected() and not client.connect(False):
            continue
          frame = client.recv(timeout_ms=0)
          if frame is not None:
            result[name] = {'width': WIDTH, 'height': HEIGHT, 'frame': client.frame_id,
                            'time': time.monotonic(), 'jpeg': base64.b64encode(jpeg_preview(preview_nv12(frame))).decode()}
        except Exception:
          continue  # Missing previews never affect inference or vehicle state.
      temporary = path.with_suffix('.tmp')
      temporary.write_text(json.dumps({'updated': time.monotonic(), 'cameras': result}))
      os.replace(temporary, path)
      time.sleep(max(0, start + .1 - time.monotonic()))

  def close(self):
    self.process.terminate()
    try:
      self.process.wait(timeout=1)
    except subprocess.TimeoutExpired:
      self.process.kill()
      self.process.wait(timeout=1)
    self.path.unlink(missing_ok=True)
    self.path.with_suffix('.tmp').unlink(missing_ok=True)


class RemoteRoadCamera:
  def __init__(self):
    self.is_wide = False
    self.texture = None
    self.last_frame = None

  def select_stream(self, prefer_wide):
    self.is_wide = prefer_wide
    return self.is_wide

  def draw(self, destination):
    from hud_protocol import read_snapshot
    import pyray as rl
    from PIL import Image
    snapshot = read_snapshot()
    if snapshot is None:
      return False
    _, packet = snapshot
    name = 'wide' if self.is_wide else 'road'
    frame = packet.get('cameras', {}).get(name)
    if frame is None or not 0 <= packet['sent'] - frame['time'] < .3:
      return False
    key = (name, frame['frame'])
    if key != self.last_frame:
      if (frame['width'], frame['height']) != (WIDTH, HEIGHT):
        return False
      data = base64.b64decode(frame['jpeg'])
      if len(data) > 20 * 1024:
        return False
      with Image.open(BytesIO(data)) as image:
        if image.size != (WIDTH, HEIGHT):
          return False
        rgb = image.convert('RGB').tobytes()
      pointer = rl.ffi.cast('void *', rl.ffi.from_buffer(rgb))
      if self.texture is None:
        self.texture = rl.load_texture_from_image(rl.Image(pointer, WIDTH, HEIGHT, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8))
        rl.set_texture_filter(self.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
      else:
        rl.update_texture(self.texture, pointer)
      self.last_frame = key
    rl.draw_texture_pro(self.texture, rl.Rectangle(0, 0, WIDTH, HEIGHT), destination, rl.Vector2(0, 0), 0, rl.WHITE)
    return True

  def close(self):
    import pyray as rl
    if self.texture is not None:
      rl.unload_texture(self.texture)
      self.texture = None


if __name__ == '__main__' and len(sys.argv) == 3 and sys.argv[1] == '--publish':
  CameraPublisher.run(Path(sys.argv[2]))
