from datetime import datetime, timezone

from selfdrive.carrot.server.services.youtube_live_captions import Cea608TimestampInjector, build_cea608_sei
from selfdrive.carrot.server.services.youtube_live_muxer import _annexb_to_avcc


def _unescape_rbsp(payload: bytes) -> bytes:
  result = bytearray()
  index = 0
  while index < len(payload):
    if payload[index:index + 3] == b"\x00\x00\x03":
      result.extend(b"\x00\x00")
      index += 3
    else:
      result.append(payload[index])
      index += 1
  return bytes(result)


def _caption_pair(sei: bytes) -> tuple[int, int]:
  assert sei.startswith(b"\x00\x00\x00\x01\x06")
  rbsp = _unescape_rbsp(sei[5:])
  assert rbsp[:2] == b"\x04\x0e"
  assert rbsp[2:10] == b"\xB5\x00\x31GA94\x03"
  assert rbsp[10:12] == b"\x41\xFF"
  assert rbsp[12] == 0xFC
  return rbsp[13] & 0x7F, rbsp[14] & 0x7F


def test_build_cea608_sei_has_atsc_payload_and_odd_parity() -> None:
  sei = build_cea608_sei((ord("2"), ord("0")))
  rbsp = _unescape_rbsp(sei[5:])
  assert _caption_pair(sei) == (ord("2"), ord("0"))
  assert rbsp[13].bit_count() % 2 == 1
  assert rbsp[14].bit_count() % 2 == 1
  assert rbsp[-1] == 0x80


def test_flv_avcc_conversion_keeps_caption_sei_before_video() -> None:
  video = b"\x00\x00\x00\x01\x65\x88\x84"
  avcc = _annexb_to_avcc(build_cea608_sei((ord("2"), ord("0"))) + video)
  first_size = int.from_bytes(avcc[:4], "big")
  first_nalu = avcc[4:4 + first_size]
  second_offset = 4 + first_size
  second_size = int.from_bytes(avcc[second_offset:second_offset + 4], "big")
  second_nalu = avcc[second_offset + 4:second_offset + 4 + second_size]
  assert first_nalu[0] & 0x1F == 6
  assert second_nalu[0] & 0x1F == 5


def test_timestamp_sequence_is_injected_without_changing_video_payload() -> None:
  injector = Cea608TimestampInjector()
  video = b"\x00\x00\x00\x01\x65\x88\x84"
  now = datetime(2026, 7, 1, 8, 32, 9, tzinfo=timezone.utc)
  pairs = []
  for _ in range(18):
    injected = injector.inject(video, enabled=True, now=now)
    assert injected.endswith(video)
    pairs.append(_caption_pair(injected[:-len(video)]))

  text = "".join(chr(value) for pair in pairs[6:-2] for value in pair).rstrip()
  assert text == "2026-07-01 08:32:09"
  assert pairs[:2] == [(0x14, 0x20)] * 2
  assert pairs[2:4] == [(0x14, 0x2E)] * 2
  assert pairs[4:6] == [(0x11, 0x54)] * 2
  assert pairs[-2:] == [(0x14, 0x2F)] * 2


def test_disabling_captions_sends_erase_display_memory() -> None:
  injector = Cea608TimestampInjector()
  video = b"\x00\x00\x00\x01\x41\x01"
  injector.inject(video, enabled=True, now=datetime(2026, 7, 1, tzinfo=timezone.utc))
  first = injector.inject(video, enabled=False)
  second = injector.inject(video, enabled=False)
  assert _caption_pair(first[:-len(video)]) == (0x14, 0x2C)
  assert _caption_pair(second[:-len(video)]) == (0x14, 0x2C)
  assert injector.inject(video, enabled=False) == video
