# cython: language_level=3, boundscheck=False, wraparound=False
# distutils: language=c++
"""Same Raylib primitives/order, with per-vertex work kept across one boundary."""
from libc.stdint cimport uintptr_t
from libc.limits cimport INT_MAX
from libcpp.vector cimport vector
from libc.math cimport fabs, INFINITY
from libc.string cimport memmove
from collections import OrderedDict
import numpy as np

ctypedef fused coordinate:
  float
  double


def project_ribbon_batch(const coordinate[:, :] line, double half_width, double z_offset, Py_ssize_t max_idx,
                         transform, clip, bint allow_invert=True, max_distance=None, double y_shift=0., Py_ssize_t start_idx=0):
  """Prepare both sides in one allocation; retain NumPy's exact matrix product."""
  cdef Py_ssize_t first, stop, step, i, j = 0, n = 0
  cdef float left_y = -half_width + y_shift, right_y = half_width + y_shift, dz = z_offset
  cdef coordinate x, y, z
  cdef bint endpoint = max_distance is not None and 0 < max_idx < line.shape[0] - 1
  if line.shape[1] != 3:
    raise ValueError('expected (N,3) points')
  first, stop, step = slice(start_idx, max_idx + 1).indices(line.shape[0])
  for i in range(first, stop):
    if line[i, 0] >= 0:
      n += 1
  if endpoint:
    # np.interp owns repeated-node/NaN/boundary behavior and float64 rounding.
    x = max_distance
    y = np.interp(max_distance, [line[max_idx, 0], line[max_idx+1, 0]], [line[max_idx, 1], line[max_idx+1, 1]])
    z = np.interp(max_distance, [line[max_idx, 0], line[max_idx+1, 0]], [line[max_idx, 2], line[max_idx+1, 2]])
    if x >= 0:
      n += 1
    else:
      endpoint = False
  if n == 0:
    return np.empty((0, 2), dtype=np.float32)
  sides = np.empty((2, n, 3), dtype=np.float32 if coordinate is float else np.float64)
  cdef coordinate[:, :, ::1] out = sides
  for i in range(first, stop):
    if not line[i, 0] >= 0:
      continue
    out[0, j, 0] = out[1, j, 0] = line[i, 0] + <coordinate>0.
    out[0, j, 1] = line[i, 1] + left_y
    out[1, j, 1] = line[i, 1] + right_y
    out[0, j, 2] = out[1, j, 2] = line[i, 2] + dz
    j += 1
  if endpoint:
    out[0, j, 0] = out[1, j, 0] = x + <coordinate>0.
    out[0, j, 1] = y + left_y
    out[1, j, 1] = y + right_y
    out[0, j, 2] = out[1, j, 2] = z + dz
  projected = (transform @ sides.reshape(2*n, 3).T).reshape(3, 2, n)
  # Transform and line can have different dtypes. Dispatch by product dtype.
  return clip_ribbon(projected, clip.x, clip.x + clip.width, clip.y, clip.y + clip.height, allow_invert)


def offset_sides(const coordinate[:, :] points, float left_y, float right_y, float left_z, float right_z):
  """Keep the original float32 offsets, including for float64 input points."""
  cdef Py_ssize_t i, n = points.shape[0]
  if points.shape[1] != 3:
    raise ValueError('expected (N,3) points')
  out = np.empty((2, n, 3), dtype=np.float32 if coordinate is float else np.float64)
  cdef coordinate[:, :, ::1] result = out
  for i in range(n):
    result[0, i, 0] = points[i, 0] + <coordinate>0.
    result[1, i, 0] = points[i, 0] + <coordinate>0.
    result[0, i, 1] = points[i, 1] + left_y
    result[1, i, 1] = points[i, 1] + right_y
    result[0, i, 2] = points[i, 2] + left_z
    result[1, i, 2] = points[i, 2] + right_z
  return out


def path_sides(const double[:, :] points, const double[:] y_offset, const double[:] z_offset):
  cdef Py_ssize_t i, n = points.shape[0]
  if points.shape[1] != 3 or y_offset.shape[0] != n or z_offset.shape[0] != n:
    raise ValueError('expected (N,3) points and N offsets')
  out = np.empty((2, n, 3), dtype=np.float64)
  cdef double[:, :, ::1] result = out
  for i in range(n):
    result[0, i, 0] = points[i, 0]
    result[1, i, 0] = points[i, 0]
    result[0, i, 1] = points[i, 1] - y_offset[i]
    result[1, i, 1] = points[i, 1] + y_offset[i]
    result[0, i, 2] = points[i, 2] + z_offset[i]
    result[1, i, 2] = points[i, 2] + z_offset[i]
  return out


def project_path_batch(line, double width, double z_start, double z_end, transform, clip, bint allow_invert=True):
  """Keep interpolation/BLAS rounding while preparing and clipping in one call."""
  points = np.asarray(line, dtype=np.float64)
  if points.ndim != 2 or points.shape[1] != 3:
    raise ValueError('expected (N,3) points')
  if len(points) == 0:
    return np.empty((0, 2), dtype=np.float32)
  z_off = np.interp(points[:, 0], [0., 100.], [z_start, z_end])
  y_off = np.interp(z_off, [-3., 0., 3.], [1.5, .5, 1.5]) * width
  sides = path_sides(points, y_off, z_off)
  projected = (sides @ transform.T).transpose(2, 0, 1)
  return _clip_ribbon[double](projected, clip.x, clip.x + clip.width, clip.y, clip.y + clip.height, allow_invert, None)


cdef _clip_ribbon(const coordinate[:, :, :] projected, coordinate x_min, coordinate x_max,
                  coordinate y_min, coordinate y_max, bint allow_invert, const coordinate[:] forward):
  """Divide, clip both sides and compact in the original vertex order.

  NumPy retains ownership of matrix products: BLAS rounding at clip/hill
  boundaries must not change when the optional native backend is enabled.
  Accept strided views so path, tire, lane and barrier projections share this.
  """
  cdef Py_ssize_t i, n = projected.shape[2], count = 0
  cdef coordinate lz, rz, lx, ly, rx, ry, min_y = INFINITY
  cdef coordinate epsilon = 1e-6
  if projected.shape[0] != 3 or projected.shape[1] != 2:
    raise ValueError('expected (3,2,N) projected coordinates')
  out = np.empty((2*n, 2), dtype=np.float32)
  cdef float[:, ::1] result = out
  for i in range(n):
    if forward is not None and not forward[i] >= 0:
      continue
    lz = projected[2, 0, i]
    rz = projected[2, 1, i]
    if not (fabs(lz) >= epsilon and fabs(rz) >= epsilon):
      continue
    lx = projected[0, 0, i] / lz
    ly = projected[1, 0, i] / lz
    rx = projected[0, 1, i] / rz
    ry = projected[1, 1, i] / rz
    if not (x_min <= lx <= x_max and x_min <= rx <= x_max and y_min <= ly <= y_max and y_min <= ry <= y_max):
      continue
    if not allow_invert and ly > min_y:
      continue
    min_y = ly
    result[count, 0] = lx
    result[count, 1] = ly
    result[2*n-1-count, 0] = rx
    result[2*n-1-count, 1] = ry
    count += 1
  if count:
    memmove(&result[count, 0], &result[2*n-count, 0], count * 2 * sizeof(float))
  return out[:2*count]


def clip_ribbon(const coordinate[:, :, :] projected, coordinate x_min, coordinate x_max,
                coordinate y_min, coordinate y_max, bint allow_invert=True):
  return _clip_ribbon[coordinate](projected, x_min, x_max, y_min, y_max, allow_invert, None)


def clip_dashes(const coordinate[:, :, :] projected, const coordinate[:] forward, const long long[:] lengths,
                coordinate x_min, coordinate x_max, coordinate y_min, coordinate y_max):
  cdef Py_ssize_t i, start = 0, end = 0, n = projected.shape[2]
  if projected.shape[0] != 3 or projected.shape[1] != 2 or forward.shape[0] != n:
    raise ValueError('expected (3,2,N) projections and N forward coordinates')
  for i in range(lengths.shape[0]):
    if lengths[i] < 0 or lengths[i] > n - end:
      raise ValueError('invalid dash lengths')
    end += lengths[i]
  if end != n:
    raise ValueError('dash lengths must cover the projected points')
  result = []
  for i in range(lengths.shape[0]):
    end = start + lengths[i]
    polygon = _clip_ribbon(projected[:, :, start:end], x_min, x_max, y_min, y_max, True, forward[start:end])
    if polygon.size:
      result.append(polygon)
    start = end
  return result

cdef extern from *:
  """
  struct CarrotVector2 { float x, y; };
  struct CarrotColor { unsigned char r, g, b, a; };
  typedef void (*CarrotLine)(CarrotVector2, CarrotVector2, float, CarrotColor);
  typedef void (*CarrotStrip)(const CarrotVector2*, int, CarrotColor);
  """
  cdef struct CarrotVector2:
    float x
    float y
  cdef struct CarrotColor:
    unsigned char r, g, b, a
  ctypedef void (*CarrotLine)(CarrotVector2, CarrotVector2, float, CarrotColor) noexcept
  ctypedef void (*CarrotStrip)(const CarrotVector2*, int, CarrotColor) noexcept


def outline(const float[:, ::1] points, uintptr_t address, float width,
            unsigned char r, unsigned char g, unsigned char b, unsigned char a):
  cdef Py_ssize_t i, n = points.shape[0]
  cdef CarrotVector2 p, q
  cdef CarrotColor color
  cdef CarrotLine draw = <CarrotLine>address
  if points.shape[1] != 2 or address == 0:
    raise ValueError('expected (N,2) points and a valid Raylib function')
  if n < 2:
    return
  color.r = r; color.g = g; color.b = b; color.a = a
  for i in range(n):
    p.x = points[i, 0]; p.y = points[i, 1]
    q.x = points[(i + 1) % n, 0]; q.y = points[(i + 1) % n, 1]
    draw(p, q, width, color)


def ribbon(const float[:, ::1] points, uintptr_t address,
           unsigned char r, unsigned char g, unsigned char b, unsigned char a):
  cdef Py_ssize_t i, n = (points.shape[0] // 2) * 2
  cdef vector[CarrotVector2] strip
  cdef CarrotColor color
  cdef CarrotStrip draw = <CarrotStrip>address
  if points.shape[1] != 2 or address == 0 or n > INT_MAX:
    raise ValueError('expected (N,2) points and a valid Raylib function')
  if n == 0:
    return
  strip.resize(n)
  for i in range(n // 2):
    strip[2*i].x = points[i, 0]; strip[2*i].y = points[i, 1]
    strip[2*i+1].x = points[n-i-1, 0]; strip[2*i+1].y = points[n-i-1, 1]
  color.r = r; color.g = g; color.b = b; color.a = a
  draw(&strip[0], <int>n, color)


# Match the loaded Raylib ABI before using these declarations (native_text.py).
# Call its own GetGlyphIndex/GetCodepointNext so fallback glyph selection and
# UTF-8 decoding stay with the installed library.
cdef extern from *:
  """
  struct CarrotRectangle { float x, y, width, height; };
  struct CarrotTexture { unsigned int id; int width, height, mipmaps, format; };
  struct CarrotImage { void *data; int width, height, mipmaps, format; };
  struct CarrotGlyph { int value, offsetX, offsetY, advanceX; CarrotImage image; };
  struct CarrotFont {
    int baseSize, glyphCount, glyphPadding;
    CarrotTexture texture; CarrotRectangle *recs; CarrotGlyph *glyphs;
  };
  typedef void (*CarrotText)(CarrotFont, const char*, CarrotVector2, float, float, CarrotColor);
  typedef void (*CarrotTexturePro)(CarrotTexture, CarrotRectangle, CarrotRectangle, CarrotVector2, float, CarrotColor);
  typedef int (*CarrotGlyphIndex)(CarrotFont, int);
  typedef int (*CarrotCodepoint)(const char*, int*);
  """
  cdef struct CarrotRectangle:
    float x, y, width, height
  cdef struct CarrotTexture:
    unsigned int id
    int width, height, mipmaps, format
  cdef struct CarrotImage:
    void *data
    int width, height, mipmaps, format
  cdef struct CarrotGlyph:
    int value, offsetX, offsetY, advanceX
    CarrotImage image
  cdef struct CarrotFont:
    int baseSize, glyphCount, glyphPadding
    CarrotTexture texture
    CarrotRectangle *recs
    CarrotGlyph *glyphs
  ctypedef void (*CarrotText)(CarrotFont, const char*, CarrotVector2, float, float, CarrotColor) noexcept
  ctypedef void (*CarrotTexturePro)(CarrotTexture, CarrotRectangle, CarrotRectangle, CarrotVector2, float, CarrotColor) noexcept
  ctypedef int (*CarrotGlyphIndex)(CarrotFont, int) noexcept
  ctypedef int (*CarrotCodepoint)(const char*, int*) noexcept


def text_abi():
  return {
    'Vector2': (sizeof(CarrotVector2), (<uintptr_t>&(<CarrotVector2*>0).x, <uintptr_t>&(<CarrotVector2*>0).y)),
    'Color': (sizeof(CarrotColor), tuple(range(4))),
    'Rectangle': (sizeof(CarrotRectangle), (<uintptr_t>&(<CarrotRectangle*>0).x, <uintptr_t>&(<CarrotRectangle*>0).y,
                                         <uintptr_t>&(<CarrotRectangle*>0).width, <uintptr_t>&(<CarrotRectangle*>0).height)),
    'Texture': (sizeof(CarrotTexture), (<uintptr_t>&(<CarrotTexture*>0).id, <uintptr_t>&(<CarrotTexture*>0).width,
                                     <uintptr_t>&(<CarrotTexture*>0).height, <uintptr_t>&(<CarrotTexture*>0).mipmaps,
                                     <uintptr_t>&(<CarrotTexture*>0).format)),
    'Image': (sizeof(CarrotImage), (<uintptr_t>&(<CarrotImage*>0).data, <uintptr_t>&(<CarrotImage*>0).width,
                                 <uintptr_t>&(<CarrotImage*>0).height, <uintptr_t>&(<CarrotImage*>0).mipmaps,
                                 <uintptr_t>&(<CarrotImage*>0).format)),
    'GlyphInfo': (sizeof(CarrotGlyph), (<uintptr_t>&(<CarrotGlyph*>0).value, <uintptr_t>&(<CarrotGlyph*>0).offsetX,
                                     <uintptr_t>&(<CarrotGlyph*>0).offsetY, <uintptr_t>&(<CarrotGlyph*>0).advanceX,
                                     <uintptr_t>&(<CarrotGlyph*>0).image)),
    'Font': (sizeof(CarrotFont), (<uintptr_t>&(<CarrotFont*>0).baseSize, <uintptr_t>&(<CarrotFont*>0).glyphCount,
                               <uintptr_t>&(<CarrotFont*>0).glyphPadding, <uintptr_t>&(<CarrotFont*>0).texture,
                               <uintptr_t>&(<CarrotFont*>0).recs, <uintptr_t>&(<CarrotFont*>0).glyphs)),
  }


cdef struct TextQuad:
  CarrotRectangle source
  float advance, offset_x, offset_y, padding, width, height


cdef class TextLayout:
  cdef vector[TextQuad] quads


cdef class TextRenderer:
  """Bounded glyph-layout reuse; no extra textures, GL state or frame delay.

  Retain the exact DrawTextEx -> DrawTextCodepoint float operation order and
  submit the same DrawTexturePro primitives, including all eight outlines.
  Multiline text uses DrawTextEx so global line spacing remains authoritative.
  """
  cdef CarrotText text_fn
  cdef CarrotTexturePro texture_fn
  cdef CarrotGlyphIndex glyph_fn
  cdef CarrotCodepoint codepoint_fn
  cdef object layouts
  cdef Py_ssize_t glyph_count, hits, misses

  def __init__(self, uintptr_t text_fn, uintptr_t texture_fn, uintptr_t glyph_fn, uintptr_t codepoint_fn):
    if not (text_fn and texture_fn and glyph_fn and codepoint_fn):
      raise ValueError('expected loaded Raylib functions')
    self.text_fn = <CarrotText>text_fn
    self.texture_fn = <CarrotTexturePro>texture_fn
    self.glyph_fn = <CarrotGlyphIndex>glyph_fn
    self.codepoint_fn = <CarrotCodepoint>codepoint_fn
    self.layouts = OrderedDict()
    self.glyph_count = self.hits = self.misses = 0

  def clear(self):
    self.layouts.clear()
    self.glyph_count = self.hits = self.misses = 0

  def stats(self):
    return {'entries': len(self.layouts), 'glyphs': self.glyph_count, 'hits': self.hits, 'misses': self.misses}

  cdef TextLayout layout(self, CarrotFont font, bytes text, float size, float spacing):
    cdef TextLayout result, removed
    cdef TextQuad quad
    cdef const char* encoded = text
    cdef Py_ssize_t i = 0, n = len(text)
    cdef int codepoint, byte_count, index
    cdef float advance = 0, scale = size / font.baseSize
    key = (font.texture.id, font.texture.width, font.texture.height, font.texture.mipmaps, font.texture.format,
           <uintptr_t>font.recs, <uintptr_t>font.glyphs, font.baseSize, font.glyphCount, font.glyphPadding, size, spacing, text)
    result = self.layouts.get(key)
    if result is not None:
      self.layouts.move_to_end(key)
      self.hits += 1
      return result
    self.misses += 1
    result = TextLayout()
    while i < n and encoded[i] != 0:
      byte_count = 0
      codepoint = self.codepoint_fn(encoded + i, &byte_count)
      if byte_count <= 0 or byte_count > n - i:
        return None
      index = self.glyph_fn(font, codepoint)
      if not 0 <= index < font.glyphCount:
        return None
      if codepoint != 32 and codepoint != 9:
        quad.source.x = font.recs[index].x - <float>font.glyphPadding
        quad.source.y = font.recs[index].y - <float>font.glyphPadding
        quad.source.width = font.recs[index].width + <float>2 * font.glyphPadding
        quad.source.height = font.recs[index].height + <float>2 * font.glyphPadding
        quad.width = quad.source.width * scale
        quad.height = quad.source.height * scale
        quad.advance = advance
        quad.offset_x = font.glyphs[index].offsetX * scale
        quad.offset_y = font.glyphs[index].offsetY * scale
        quad.padding = <float>font.glyphPadding * scale
        result.quads.push_back(quad)
      if font.glyphs[index].advanceX == 0:
        advance += font.recs[index].width * scale + spacing
      else:
        advance += <float>font.glyphs[index].advanceX * scale + spacing
      i += byte_count
    self.layouts[key] = result
    self.glyph_count += result.quads.size()
    while len(self.layouts) > 256 or self.glyph_count > 8192:
      _, removed = self.layouts.popitem(last=False)
      self.glyph_count -= removed.quads.size()
    return result

  cdef void layer(self, CarrotFont font, bytes text, float size, CarrotVector2 position,
                  CarrotColor color, TextLayout layout, float spacing=0):
    cdef size_t i
    cdef TextQuad quad
    cdef CarrotRectangle dest
    cdef CarrotVector2 origin
    cdef float glyph_x, glyph_y
    if layout is None:
      self.text_fn(font, text, position, size, spacing, color)
      return
    origin.x = origin.y = 0
    for i in range(layout.quads.size()):
      quad = layout.quads[i]
      # Keep each float rounding point used by the original library. Do not
      # combine advance/offset/padding or translate a pre-rounded rectangle.
      glyph_x = position.x + quad.advance
      glyph_y = position.y + <float>0
      dest.x = glyph_x + quad.offset_x
      dest.x = dest.x - quad.padding
      dest.y = glyph_y + quad.offset_y
      dest.y = dest.y - quad.padding
      dest.width = quad.width
      dest.height = quad.height
      self.texture_fn(font.texture, quad.source, dest, origin, 0, color)

  def draw(self, uintptr_t font_address, bytes text, double x, double y, float size,
           const double[:, ::1] offsets, double border, double shadow, color, border_color, shadow_color,
           bint cache=True):
    cdef CarrotFont font
    cdef CarrotColor tint
    cdef CarrotVector2 position
    cdef TextLayout run = None
    cdef Py_ssize_t i
    if not font_address or offsets.shape[0] != 8 or offsets.shape[1] != 2:
      raise ValueError('expected a Font and eight outline directions')
    font = (<CarrotFont*>font_address)[0]
    if cache and len(text) <= 512 and b'\n' not in text and font.texture.id and font.baseSize > 0 and font.glyphCount > 0:
      if font.recs != NULL and font.glyphs != NULL:
        run = self.layout(font, text, size, 0)
    if border > 0:
      tint.r, tint.g, tint.b, tint.a = border_color
      for i in range(8):
        position.x = <float>(x + border * offsets[i, 0])
        position.y = <float>(y + border * offsets[i, 1])
        self.layer(font, text, size, position, tint, run)
    if shadow != 0:
      tint.r, tint.g, tint.b, tint.a = shadow_color
      position.x = <float>(x + shadow)
      position.y = <float>(y + shadow)
      self.layer(font, text, size, position, tint, run)
    tint.r, tint.g, tint.b, tint.a = color
    position.x = <float>x
    position.y = <float>y
    self.layer(font, text, size, position, tint, run)

  def draw_plain(self, uintptr_t font_address, bytes text, float x, float y, float size, float spacing, color, bint cache=True):
    cdef CarrotFont font
    cdef CarrotColor tint
    cdef CarrotVector2 position
    cdef TextLayout run = None
    if not font_address:
      raise ValueError('expected a Font')
    font = (<CarrotFont*>font_address)[0]
    if cache and len(text) <= 512 and b'\n' not in text and font.texture.id and font.baseSize > 0 and font.glyphCount > 0:
      if font.recs != NULL and font.glyphs != NULL:
        run = self.layout(font, text, size, spacing)
    tint.r, tint.g, tint.b, tint.a = color
    position.x, position.y = x, y
    self.layer(font, text, size, position, tint, run, spacing)
