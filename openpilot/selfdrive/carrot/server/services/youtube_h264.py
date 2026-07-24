from __future__ import annotations

from dataclasses import dataclass


ANNEXB_START_CODE_3 = b"\x00\x00\x01"
ANNEXB_START_CODE_4 = b"\x00\x00\x00\x01"
H264_NAL_IDR = 5
H264_NAL_SPS = 7
H264_NAL_PPS = 8


@dataclass(frozen=True, slots=True)
class H264AccessUnit:
  avcc: bytes
  nal_types: tuple[int, ...]

  @property
  def is_idr(self) -> bool:
    return H264_NAL_IDR in self.nal_types


def _has_annexb_prefix(payload: bytes) -> bool:
  prefix = payload[:8]
  return ANNEXB_START_CODE_3 in prefix or ANNEXB_START_CODE_4 in prefix


def annexb_nalus(payload: bytes) -> list[bytes]:
  starts: list[tuple[int, int]] = []
  index = 0
  size = len(payload)
  while index + 3 <= size:
    if index + 4 <= size and payload[index:index + 4] == ANNEXB_START_CODE_4:
      starts.append((index, 4))
      index += 4
    elif payload[index:index + 3] == ANNEXB_START_CODE_3:
      starts.append((index, 3))
      index += 3
    else:
      index += 1

  nalus = []
  for position, (offset, start_size) in enumerate(starts):
    start = offset + start_size
    end = starts[position + 1][0] if position + 1 < len(starts) else size
    while end > start and payload[end - 1] == 0:
      end -= 1
    if end > start:
      nalus.append(payload[start:end])
  return nalus


def avcc_nalus(payload: bytes, *, length_size: int = 4) -> list[bytes]:
  if length_size not in (1, 2, 4):
    raise ValueError("unsupported H.264 AVCC length size")

  nalus = []
  offset = 0
  while offset < len(payload):
    if offset + length_size > len(payload):
      raise ValueError("truncated H.264 AVCC length")
    nalu_size = int.from_bytes(payload[offset:offset + length_size], "big")
    offset += length_size
    if nalu_size <= 0 or offset + nalu_size > len(payload):
      raise ValueError("invalid H.264 AVCC NAL size")
    nalus.append(payload[offset:offset + nalu_size])
    offset += nalu_size
  if not nalus:
    raise ValueError("H.264 access unit has no NAL units")
  return nalus


def access_unit_nalus(payload: bytes) -> list[bytes]:
  if not payload:
    raise ValueError("H.264 access unit is empty")
  nalus = annexb_nalus(payload) if _has_annexb_prefix(payload) else avcc_nalus(payload)
  if not nalus:
    raise ValueError("H.264 access unit has no NAL units")
  return nalus


def nal_unit_types(payload: bytes) -> tuple[int, ...]:
  try:
    return tuple(nalu[0] & 0x1F for nalu in access_unit_nalus(payload) if nalu)
  except ValueError:
    return ()


def has_idr(payload: bytes) -> bool:
  return H264_NAL_IDR in nal_unit_types(payload)


def _validate_avc_decoder_configuration(config: bytes) -> None:
  if len(config) < 7 or config[0] != 1:
    raise ValueError("invalid H.264 AVC decoder configuration")
  if (config[4] & 0x03) != 3:
    raise ValueError("H.264 AVC configuration must use four-byte NAL lengths")

  offset = 6
  sps_count = config[5] & 0x1F
  if sps_count <= 0:
    raise ValueError("H.264 codec header has no SPS")
  for _ in range(sps_count):
    if offset + 2 > len(config):
      raise ValueError("truncated H.264 SPS length")
    size = int.from_bytes(config[offset:offset + 2], "big")
    offset += 2
    if size < 4 or offset + size > len(config) or config[offset] & 0x1F != H264_NAL_SPS:
      raise ValueError("invalid H.264 SPS")
    offset += size

  if offset >= len(config):
    raise ValueError("H.264 codec header has no PPS")
  pps_count = config[offset]
  offset += 1
  if pps_count <= 0:
    raise ValueError("H.264 codec header has no PPS")
  for _ in range(pps_count):
    if offset + 2 > len(config):
      raise ValueError("truncated H.264 PPS length")
    size = int.from_bytes(config[offset:offset + 2], "big")
    offset += 2
    if size <= 0 or offset + size > len(config) or config[offset] & 0x1F != H264_NAL_PPS:
      raise ValueError("invalid H.264 PPS")
    offset += size


def avc_decoder_configuration(codec_header: bytes) -> bytes:
  if codec_header[:1] == b"\x01":
    config = bytes(codec_header)
    _validate_avc_decoder_configuration(config)
    return config

  nalus = annexb_nalus(codec_header)
  sps_units = [nalu for nalu in nalus if nalu and nalu[0] & 0x1F == H264_NAL_SPS]
  pps_units = [nalu for nalu in nalus if nalu and nalu[0] & 0x1F == H264_NAL_PPS]
  if not sps_units or not pps_units or len(sps_units[0]) < 4:
    raise ValueError("H.264 codec header has no SPS/PPS")
  if len(sps_units) > 31 or len(pps_units) > 255:
    raise ValueError("H.264 codec header has too many parameter sets")

  first_sps = sps_units[0]
  result = bytearray((1, first_sps[1], first_sps[2], first_sps[3], 0xFF, 0xE0 | len(sps_units)))
  for sps in sps_units:
    result.extend(len(sps).to_bytes(2, "big"))
    result.extend(sps)
  result.append(len(pps_units))
  for pps in pps_units:
    result.extend(len(pps).to_bytes(2, "big"))
    result.extend(pps)
  config = bytes(result)
  _validate_avc_decoder_configuration(config)
  return config


def normalize_access_unit(payload: bytes) -> H264AccessUnit:
  nalus = access_unit_nalus(payload)
  nal_types = tuple(nalu[0] & 0x1F for nalu in nalus if nalu)
  avcc = b"".join(len(nalu).to_bytes(4, "big") + nalu for nalu in nalus)
  return H264AccessUnit(avcc=avcc, nal_types=nal_types)


def access_unit_to_avcc(payload: bytes) -> bytes:
  return normalize_access_unit(payload).avcc


def validate_stream_start(codec_header: bytes, access_unit: bytes) -> None:
  avc_decoder_configuration(codec_header)
  if not has_idr(access_unit):
    raise ValueError("first H.264 access unit is not an IDR frame")
