"""Minimal ISO-BMFF (MP4) muxer for H.264 access units.

The ffmpeg bundled on the device is an encode-only build without the raw H.264
demuxer, so hardware screen recordings are muxed here instead of shelling out.
One chunk is written per sample, which keeps stsc/stco trivial; the mdat size
is patched when the file is finalized.
"""
from __future__ import annotations

import struct
from pathlib import Path

_MATRIX = struct.pack(">9i", 0x00010000, 0, 0, 0, 0x00010000, 0, 0, 0, 0x40000000)
_ZERO = b"\x00"


def _box(box_type: bytes, payload: bytes) -> bytes:
  return struct.pack(">I", len(payload) + 8) + box_type + payload


def _full_box(box_type: bytes, version: int, flags: int, payload: bytes) -> bytes:
  return _box(box_type, struct.pack(">BI", version, flags)[1:] + payload)


def split_annex_b(data: bytes) -> list[bytes]:
  """Split an Annex-B H.264 byte stream into NAL units (start codes removed)."""
  nals: list[bytes] = []
  index = 0
  length = len(data)
  while index < length:
    start = -1
    start_len = 0
    for i in range(index, max(index, length - 3)):
      if data[i] == 0 and data[i + 1] == 0:
        if data[i + 2] == 1:
          start, start_len = i, 3
          break
        if i + 3 < length and data[i + 2] == 0 and data[i + 3] == 1:
          start, start_len = i, 4
          break
    if start < 0:
      break
    payload_start = start + start_len
    next_start = length
    for j in range(payload_start, max(payload_start, length - 3)):
      if data[j] == 0 and data[j + 1] == 0:
        if data[j + 2] == 1:
          next_start = j
          break
        if j + 3 < length and data[j + 2] == 0 and data[j + 3] == 1:
          next_start = j
          break
    end = next_start
    while end > payload_start and data[end - 1] == 0:
      end -= 1
    if end > payload_start:
      nals.append(bytes(data[payload_start:end]))
    index = next_start if next_start < length else length
  return nals


class Mp4H264Writer:
  """Writes a single H.264 video track to an MP4 file."""

  def __init__(self, path, fps: int, width: int, height: int):
    self._path = Path(path)
    self._fps = max(1, int(fps))
    self._width = int(width)
    self._height = int(height)
    self._file = open(self._path, "wb")
    self._file.write(_box(b"ftyp", b"isom" + struct.pack(">I", 0x200) + b"isomiso2avc1mp41"))
    self._mdat_pos = self._file.tell()
    self._file.write(struct.pack(">I", 0) + b"mdat")
    self._sizes: list[int] = []
    self._offsets: list[int] = []
    self._sync: list[int] = []
    self._sps: bytes | None = None
    self._pps: bytes | None = None
    self._closed = False

  @property
  def sample_count(self) -> int:
    return len(self._sizes)

  def add_access_unit(self, data: bytes) -> bool:
    if self._closed:
      return False
    nals = split_annex_b(data)
    if not nals:
      return False
    payload = bytearray()
    has_vcl = False
    keyframe = False
    for nal in nals:
      nal_type = nal[0] & 0x1F
      if nal_type == 7 and self._sps is None:
        self._sps = nal
      elif nal_type == 8 and self._pps is None:
        self._pps = nal
      elif nal_type in (1, 5):
        has_vcl = True
        keyframe = keyframe or nal_type == 5
      payload += struct.pack(">I", len(nal)) + nal
    if not has_vcl:
      return False
    offset = self._file.tell()
    self._file.write(payload)
    self._sizes.append(len(payload))
    self._offsets.append(offset)
    if keyframe or not self._sync:
      self._sync.append(len(self._sizes))
    return True

  def close(self) -> None:
    if self._closed:
      return
    self._closed = True
    try:
      if self._sizes and self._sps is not None and self._pps is not None:
        moov_pos = self._file.tell()
        self._file.write(self._moov())
        mdat_size = moov_pos - self._mdat_pos
      else:
        # No decodable video: leave a header-only file behind.
        mdat_size = self._file.tell() - self._mdat_pos
      self._file.seek(self._mdat_pos)
      self._file.write(struct.pack(">I", mdat_size))
    finally:
      self._file.close()

  # -- box builders -------------------------------------------------------

  def _moov(self) -> bytes:
    timescale = self._fps * 1000
    delta = 1000
    duration = delta * len(self._sizes)
    return _box(b"moov", self._mvhd(timescale, duration) + self._trak(timescale, duration, delta))

  def _mvhd(self, timescale: int, duration: int) -> bytes:
    payload = (
      struct.pack(">II", 0, 0)
      + struct.pack(">II", timescale, duration)
      + struct.pack(">I", 0x00010000)
      + struct.pack(">H", 0x0100)
      + struct.pack(">H", 0)
      + struct.pack(">II", 0, 0)
      + _MATRIX
      + b"\x00" * 24
      + struct.pack(">I", 2)
    )
    return _full_box(b"mvhd", 0, 0, payload)

  def _trak(self, timescale: int, duration: int, delta: int) -> bytes:
    return _box(b"trak", self._tkhd(duration) + self._mdia(timescale, duration, delta))

  def _tkhd(self, duration: int) -> bytes:
    payload = (
      struct.pack(">II", 0, 0)
      + struct.pack(">I", 1)
      + struct.pack(">I", 0)
      + struct.pack(">I", duration)
      + struct.pack(">II", 0, 0)
      + struct.pack(">HHHH", 0, 0, 0, 0)
      + _MATRIX
      + struct.pack(">II", self._width << 16, self._height << 16)
    )
    return _full_box(b"tkhd", 0, 0x000007, payload)

  def _mdia(self, timescale: int, duration: int, delta: int) -> bytes:
    mdhd = _full_box(b"mdhd", 0, 0, struct.pack(">IIIIHH", 0, 0, timescale, duration, 0x55C4, 0))
    hdlr = _full_box(b"hdlr", 0, 0, struct.pack(">I", 0) + b"vide" + b"\x00" * 12 + b"VideoHandler\x00")
    minf = _box(b"minf", self._vmhd() + self._dinf() + self._stbl(delta))
    return _box(b"mdia", mdhd + hdlr + minf)

  @staticmethod
  def _vmhd() -> bytes:
    return _full_box(b"vmhd", 0, 1, struct.pack(">HHHH", 0, 0, 0, 0))

  @staticmethod
  def _dinf() -> bytes:
    url = _full_box(b"url ", 0, 1, b"")
    return _box(b"dinf", _full_box(b"dref", 0, 0, struct.pack(">I", 1) + url))

  def _stbl(self, delta: int) -> bytes:
    stsd = _full_box(b"stsd", 0, 0, struct.pack(">I", 1) + self._avc1())
    stts = _full_box(b"stts", 0, 0, struct.pack(">III", 1, len(self._sizes), delta))
    boxes = stsd + stts
    if len(self._sync) != len(self._sizes):
      stss = _full_box(b"stss", 0, 0, struct.pack(">I", len(self._sync)) + b"".join(struct.pack(">I", n) for n in self._sync))
      boxes += stss
    stsc = _full_box(b"stsc", 0, 0, struct.pack(">IIII", 1, 1, 1, 1))
    stsz = _full_box(b"stsz", 0, 0, struct.pack(">II", 0, len(self._sizes)) + b"".join(struct.pack(">I", s) for s in self._sizes))
    stco = _full_box(b"stco", 0, 0, struct.pack(">I", len(self._offsets)) + b"".join(struct.pack(">I", o) for o in self._offsets))
    return _box(b"stbl", boxes + stsc + stsz + stco)

  def _avc1(self) -> bytes:
    compressorname = b"\x00" * 32
    payload = (
      b"\x00" * 6
      + struct.pack(">H", 1)
      + struct.pack(">HH", 0, 0)
      + b"\x00" * 12
      + struct.pack(">HH", self._width, self._height)
      + struct.pack(">II", 0x00480000, 0x00480000)
      + struct.pack(">I", 0)
      + struct.pack(">H", 1)
      + compressorname
      + struct.pack(">H", 0x0018)
      + struct.pack(">H", 0xFFFF)
      + self._avcc()
    )
    return _box(b"avc1", payload)

  def _avcc(self) -> bytes:
    sps = self._sps or b""
    pps = self._pps or b""
    profile = sps[1] if len(sps) > 3 else 0x42
    compatibility = sps[2] if len(sps) > 3 else 0
    level = sps[3] if len(sps) > 3 else 0x1F
    payload = (
      bytes([1, profile, compatibility, level, 0xFF, 0xE1])
      + struct.pack(">H", len(sps))
      + sps
      + bytes([1])
      + struct.pack(">H", len(pps))
      + pps
    )
    return _box(b"avcC", payload)
