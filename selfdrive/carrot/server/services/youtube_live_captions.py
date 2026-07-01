from __future__ import annotations

from datetime import datetime
from typing import Iterable


_ANNEXB_START_CODE = b"\x00\x00\x00\x01"
_CEA608_RCL = (0x14, 0x20)
_CEA608_EDM = (0x14, 0x2C)
_CEA608_ENM = (0x14, 0x2E)
_CEA608_EOC = (0x14, 0x2F)
# Row 1, white, regular, indent 4. This is the closest stable PAC position to
# center for a 19-character timestamp without cumulative tab-offset commands.
_CEA608_TOP_CENTER_PAC = (0x11, 0x52)


def _with_odd_parity(value: int) -> int:
  value &= 0x7F
  return value | (0x80 if value.bit_count() % 2 == 0 else 0)


def _rbsp_escape(payload: bytes) -> bytes:
  escaped = bytearray()
  zero_count = 0
  for value in payload:
    if zero_count >= 2 and value <= 0x03:
      escaped.append(0x03)
      zero_count = 0
    escaped.append(value)
    zero_count = zero_count + 1 if value == 0 else 0
  return bytes(escaped)


def build_cea608_sei(pairs: tuple[int, int] | Iterable[tuple[int, int]]) -> bytes:
  if isinstance(pairs, tuple) and len(pairs) == 2 and all(isinstance(value, int) for value in pairs):
    caption_pairs = [pairs]
  else:
    caption_pairs = list(pairs)
  if not caption_pairs or len(caption_pairs) > 31:
    raise ValueError("CEA-608 SEI requires 1 to 31 caption pairs")

  cc_data = b"".join(
    bytes((0xFC, _with_odd_parity(first), _with_odd_parity(second)))
    for first, second in caption_pairs
  )
  itu_t_t35 = (
    b"\xB5\x00\x31GA94\x03"
    + bytes((0x40 | len(caption_pairs), 0xFF))
    + cc_data
    + b"\xFF"
  )
  sei_rbsp = bytes((0x04, len(itu_t_t35))) + itu_t_t35 + b"\x80"
  return _ANNEXB_START_CODE + b"\x06" + _rbsp_escape(sei_rbsp)


def _timestamp_pairs(text: str) -> list[tuple[int, int]]:
  clean = "".join(ch if 0x20 <= ord(ch) <= 0x7E else "?" for ch in text)[:32]
  if len(clean) % 2:
    clean += " "
  text_pairs = [(ord(clean[index]), ord(clean[index + 1])) for index in range(0, len(clean), 2)]
  return [
    _CEA608_RCL, _CEA608_RCL,
    _CEA608_ENM, _CEA608_ENM,
    _CEA608_TOP_CENTER_PAC, _CEA608_TOP_CENTER_PAC,
    *text_pairs,
    _CEA608_EOC,
  ]


class Cea608TimestampInjector:
  def __init__(self) -> None:
    self._enabled = False
    self._last_text = ""
    self.packets_injected = 0

  def reset(self) -> None:
    self._enabled = False
    self._last_text = ""

  def inject(self, payload: bytes, *, enabled: bool, now: datetime | None = None) -> bytes:
    if not payload:
      return payload

    if enabled != self._enabled:
      self._enabled = enabled
      self._last_text = ""
      if not enabled:
        self.packets_injected += 1
        return build_cea608_sei((_CEA608_EDM, _CEA608_EDM)) + payload

    if enabled:
      local_now = now if now is not None else datetime.now().astimezone()
      text = local_now.strftime("%Y-%m-%d %H:%M:%S")
      if text != self._last_text:
        self._last_text = text
        self.packets_injected += 1
        return build_cea608_sei(_timestamp_pairs(text)) + payload

    return payload
