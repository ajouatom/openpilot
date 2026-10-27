"""Bounded NAV model preview for the Wi-Fi display, never an inference input."""
import base64
import hashlib
import io
import math
import os
from pathlib import Path
import struct
import time

PREVIEW = Path('/dev/shm/carrot_nav_map.png')
MAX_BYTES = 32 * 1024
MAX_AGE = 2.


def png_shape(data):
  return (24 <= len(data) <= MAX_BYTES and data[:16] == b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR'
          and struct.unpack('>II', data[16:24]) == (256, 256))


class NavPreviewPublisher:
  def __init__(self, path=PREVIEW):
    self.path = path
    self.next_read = 0.
    self.cached = None

  def packet(self, enabled):
    now = time.monotonic()
    if not enabled:
      self.cached = None
      self.next_read = 0.
      return None
    if now >= self.next_read:
      self.next_read = now + .5
      self.cached = None
      try:
        with self.path.open('rb') as source:
          age = time.time() - os.fstat(source.fileno()).st_mtime
          data = source.read(MAX_BYTES + 1)
        if 0 <= age < MAX_AGE and png_shape(data):
          self.cached = (now - age, hashlib.sha256(data).hexdigest(), base64.b64encode(data).decode())
      except OSError:
        pass
    if self.cached is not None:
      captured, digest, encoded = self.cached
      age = now - captured
      if 0 <= age < MAX_AGE:
        return {'sha256': digest, 'png': encoded, 'age': age}
    return None


class RemoteNavPreview:
  def __init__(self, read_snapshot, path=PREVIEW):
    self.read_snapshot = read_snapshot
    self.path = path
    self.digest = None
    self.status = {'fresh': False}

  def fresh(self):
    self.status = {'fresh': False}
    snapshot = self.read_snapshot()
    if snapshot is None:
      return False
    try:
      received, record = snapshot
      if record.get('params', {}).get('ClusterHudScreenMode') != base64.b64encode(b'7').decode():
        return False
      value = record['nav_map_preview']
      elapsed = time.monotonic() - received
      source_age = float(value['age'])
      age = source_age + elapsed
      if not (math.isfinite(age) and 0 <= elapsed < .5 and 0 <= source_age and 0 <= age < MAX_AGE):
        return False
      encoded = value['png']
      if not isinstance(encoded, str) or len(encoded) > 4 * ((MAX_BYTES + 2) // 3):
        return False
      data = base64.b64decode(encoded, validate=True)
      digest = hashlib.sha256(data).hexdigest()
      if digest != value['sha256'] or not png_shape(data):
        return False
      if digest != self.digest or not self.path.is_file():
        from PIL import Image
        # Validate the full bounded image before the native renderer opens it.
        with Image.open(io.BytesIO(data)) as image:
          if image.format != 'PNG' or image.size != (256, 256):
            return False
          image.load()
        temporary = self.path.with_suffix('.remote.tmp')
        temporary.write_bytes(data)
        os.replace(temporary, self.path)
        self.digest = digest
      self.status = {'fresh': True, 'age_ms': round(age * 1000), 'sha256': digest}
      return True
    except (OSError, ValueError, KeyError, TypeError):
      return False
