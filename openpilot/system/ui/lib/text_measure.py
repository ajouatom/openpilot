import pyray as rl
from collections import OrderedDict
from openpilot.system.ui.lib.application import FONT_SCALE, font_fallback
from openpilot.system.ui.lib.emoji import find_emoji

_MAX_ENTRIES = 2048
_MAX_TEXT_LENGTH = 512
_cache: OrderedDict[tuple, rl.Vector2] = OrderedDict()


def measure_text_cached(font: rl.Font, text: str, font_size: int, spacing: float = 0) -> rl.Vector2:
  """Caches text measurements to avoid redundant calculations."""
  font = font_fallback(font)
  spacing = round(spacing, 4)
  # Retain the full key: a hash collision must never return another label's size.
  key = (font.texture.id, font.baseSize, font.glyphCount, font.glyphPadding, text, font_size, spacing)
  if key in _cache:
    _cache.move_to_end(key)
    return _cache[key]

  # Measure normal characters without emojis, then add standard width for each found emoji
  emoji = find_emoji(text)
  if emoji:
    non_emoji_text = ""
    last_index = 0
    for start, end, _ in emoji:
      non_emoji_text += text[last_index:start]
      last_index = end
    non_emoji_text += text[last_index:]
  else:
    non_emoji_text = text

  result = rl.measure_text_ex(font, non_emoji_text, font_size * FONT_SCALE, spacing)  # noqa: TID251
  if emoji:
    result.x += len(emoji) * font_size * FONT_SCALE
    # If just emoji assume a single line height
    if result.y == 0:
      result.y = font_size * FONT_SCALE

  if len(text) <= _MAX_TEXT_LENGTH:
    _cache[key] = result
    if len(_cache) > _MAX_ENTRIES:
      _cache.popitem(last=False)
  return result
