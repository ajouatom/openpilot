from __future__ import annotations

import threading

from openpilot.selfdrive.carrot.server.services.youtube_h264 import (
  H264_NAL_IDR,
  H264_NAL_PPS,
  H264_NAL_SPS,
  access_unit_nalus,
  access_unit_to_avcc,
  avc_decoder_configuration,
  has_idr,
  nal_unit_types,
  normalize_access_unit,
  validate_stream_start,
)
from openpilot.selfdrive.carrot.server.services.youtube_live_muxer import H264FlvMuxer


START = b"\x00\x00\x00\x01"
SPS = b"\x67\x64\x00\x1f\xac\xd9\x40"
PPS = b"\x68\xee\x3c\x80"
IDR = b"\x65\x88\x84"
P_FRAME = b"\x41\x9a\x22"
HEADER = START + SPS + START + PPS
IDR_FRAME = START + IDR
P_FRAME_ACCESS_UNIT = START + P_FRAME


def _raises_value_error(callback) -> None:
  try:
    callback()
  except ValueError:
    return
  raise AssertionError("expected ValueError")


def _bare_muxer() -> tuple[H264FlvMuxer, list[tuple[int, int, bytes]]]:
  muxer = object.__new__(H264FlvMuxer)
  muxer._lock = threading.RLock()
  muxer._closed = False
  muxer._packet_index = 0
  muxer._fps = 20
  muxer._last_video_ms = 0
  muxer._mux_silence_until = lambda _target_pts: None
  tags: list[tuple[int, int, bytes]] = []
  muxer._write_tag = lambda tag_type, timestamp_ms, payload: tags.append((tag_type, timestamp_ms, payload))
  return muxer, tags


def test_annexb_header_builds_a_valid_avc_configuration():
  config = avc_decoder_configuration(HEADER)

  assert config[0] == 1
  assert config[4] & 0x03 == 3
  assert avc_decoder_configuration(config) == config


def test_access_units_are_validated_and_converted_to_avcc():
  assert access_unit_nalus(IDR_FRAME) == [IDR]
  assert nal_unit_types(HEADER) == (H264_NAL_SPS, H264_NAL_PPS)
  assert nal_unit_types(IDR_FRAME) == (H264_NAL_IDR,)
  assert has_idr(IDR_FRAME) is True
  assert has_idr(P_FRAME_ACCESS_UNIT) is False
  assert access_unit_to_avcc(IDR_FRAME) == len(IDR).to_bytes(4, "big") + IDR
  assert normalize_access_unit(IDR_FRAME).is_idr is True


def test_stream_start_requires_sps_pps_and_an_idr():
  validate_stream_start(HEADER, IDR_FRAME)

  _raises_value_error(lambda: validate_stream_start(START + SPS, IDR_FRAME))
  _raises_value_error(lambda: validate_stream_start(HEADER, P_FRAME_ACCESS_UNIT))
  _raises_value_error(lambda: access_unit_to_avcc(b"not-h264"))


def test_muxer_requires_and_marks_an_idr_as_the_first_video_packet():
  muxer, tags = _bare_muxer()
  muxer.mux(IDR_FRAME, keyframe=True)
  muxer.mux(P_FRAME_ACCESS_UNIT, timestamp_ms=0)

  assert muxer._packet_index == 2
  assert tags[0][0:2] == (9, 0)
  assert tags[0][2].startswith(b"\x17\x01\x00\x00\x00")
  assert tags[1][0:2] == (9, 1)
  assert tags[1][2].startswith(b"\x27\x01\x00\x00\x00")

  invalid_muxer, _ = _bare_muxer()
  _raises_value_error(lambda: invalid_muxer.mux(P_FRAME_ACCESS_UNIT))
