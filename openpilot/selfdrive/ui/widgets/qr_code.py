import numpy as np
import pyray as rl
import qrcode

from openpilot.common.swaglog import cloudlog


class QRCodeTexture:
  def __init__(self):
    self._data: str | None = None
    self._texture: rl.Texture | None = None

  @property
  def available(self) -> bool:
    return self._texture is not None and self._texture.id != 0

  def set_data(self, data: str | None) -> bool:
    normalized = str(data or "")
    if normalized == self._data:
      return False

    self.destroy()
    self._data = normalized
    if not normalized:
      return True

    try:
      qr = qrcode.QRCode(
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=10,
        border=4,
      )
      qr.add_data(normalized)
      qr.make(fit=True)
      pil_image = qr.make_image(fill_color="black", back_color="white").convert("RGBA")
      pixels = np.asarray(pil_image, dtype=np.uint8)

      image = rl.Image()
      image.data = rl.ffi.cast("void *", pixels.ctypes.data)
      image.width = pil_image.width
      image.height = pil_image.height
      image.mipmaps = 1
      image.format = rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8

      self._texture = rl.load_texture_from_image(image)
      rl.set_texture_filter(self._texture, rl.TextureFilter.TEXTURE_FILTER_POINT)
    except Exception:
      cloudlog.exception("Carrot Web QR code generation failed")
      self._texture = None
    return True

  def render(self, rect: rl.Rectangle) -> bool:
    if not self.available:
      return False

    source = rl.Rectangle(0, 0, self._texture.width, self._texture.height)
    rl.draw_texture_pro(self._texture, source, rect, rl.Vector2(0, 0), 0.0, rl.WHITE)
    return True

  def destroy(self) -> None:
    if self.available and rl.is_window_ready():
      rl.unload_texture(self._texture)
    self._texture = None
    self._data = None

  def __del__(self):
    self.destroy()
