"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Wire protocol.

Every message is a 32-byte header and an opaque payload. 32 keeps float32
payloads 8-byte aligned, so both ends can view the receive buffer without
copying.

INFER is a fixed struct plus two raw arrays sized at handshake, not json: one
vectored write out, one read into a preallocated buffer back.
"""
from __future__ import annotations

import struct
from enum import IntEnum

MAGIC = 0x4B4E4C4A  # b'JLNK'
# Keep the vendored server pinned at v2; clients select v3 per stream.
VERSION = 2
SUPPORTED_VERSIONS = (2, 3)

# Legacy v2 packet size. A transfer that is an exact multiple does not terminate
# a bulk read, so the sender adds one byte; v3 uses 512 (see packet_multiple()).
PACKET_MULTIPLE = 1024
_PACKET_MULTIPLES = {2: 1024, 3: 512}

# The gadget pads every message to a burst so it never ends on a short packet:
# dwc3 flushed its TX FIFO past one about once in 400 frames, and the host read
# those bytes as the next header. Host to device keeps the PADDED byte instead.
GADGET_TX_ALIGN = 16 * PACKET_MULTIPLE

# magic, version, msg_type, seq, flags, length, reserved, 4 pad
HEADER_FMT = '<IHHIIIQ4x'
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 32

_header = struct.Struct(HEADER_FMT)


class Msg(IntEnum):
  HELLO_REQ = 1        # {} -> server describes itself
  HELLO_RESP = 2       # json: server caps, loaded engine, shapes
  ENGINE_REQ = 3       # json: {sha256, nbytes, frame_skip} -> make this model ready
  ENGINE_RESP = 4      # json: {state: ready|need_upload|building|failed, spec when ready, ...}
  UPLOAD_CHUNK = 5     # u64 offset + bytes
  UPLOAD_DONE = 6      # json: {sha256}
  PROGRESS = 7         # json: {stage, frac, msg} - unsolicited, server -> client
  INFER_REQ = 8        # InferHeader + warped(u8) + packed(f32)
  INFER_RESP = 9       # InferRespHeader + outputs(f32)
  # 10, 11 were RESET_REQ/RESP; queues are cleared with Flag.RESET_QUEUES
  STATE_REQ = 12       # telemetry
  STATE_RESP = 13      # json
  ERROR = 14           # json: {error, detail}
  PING = 15
  PONG = 16
  SHUTDOWN_REQ = 17    # json: {reason} -> power the Jetson off for good; see server/power.py
  SHUTDOWN_RESP = 18   # json: {ok, detail}


class Flag(IntEnum):
  RESET_QUEUES = 1 << 0   # on INFER_REQ: warm-start, clear history before this frame
  WANT_STATE = 1 << 1     # on INFER_REQ: append telemetry json to the response.
                          # Piggybacked because at 20 Hz a separate exchange
                          # would race a frame.
  WANT_HIDDEN = 1 << 2    # on protocol 3 INFER_REQ: include hidden state in the response
  PADDED = 1 << 7         # one pad byte follows the payload; see PACKET_MULTIPLE


# INFER_REQ: frame_id, flags. Sizes of the two arrays come from the handshake.
INFER_REQ_FMT = '<II'
INFER_REQ_SIZE = struct.calcsize(INFER_REQ_FMT)
_infer_req = struct.Struct(INFER_REQ_FMT)

# INFER_RESP: frame_id, status, server-side timings in microseconds
INFER_RESP_FMT = '<IIIII'
INFER_RESP_SIZE = struct.calcsize(INFER_RESP_FMT)
_infer_resp = struct.Struct(INFER_RESP_FMT)


class ProtocolError(RuntimeError):
  pass


def validate_version(version: int) -> int:
  if type(version) is not int or version not in SUPPORTED_VERSIONS:
    raise ValueError(f"unsupported protocol version {version!r}; supported versions are {SUPPORTED_VERSIONS}")
  return version


def packet_multiple(version: int = VERSION) -> int:
  return _PACKET_MULTIPLES[validate_version(version)]


def pack_header(msg_type: int, seq: int, length: int, flags: int = 0, reserved: int = 0,
                version: int = VERSION) -> bytes:
  return _header.pack(MAGIC, validate_version(version), int(msg_type), seq, flags, length, reserved)


def unpack_header(buf, version: int = VERSION) -> tuple[int, int, int, int, int, int, int]:
  magic, wire_version, msg_type, seq, flags, length, reserved = _header.unpack_from(buf)
  if magic != MAGIC:
    raise ProtocolError(f"bad magic 0x{magic:08x} (link desynced or not a jetlink peer)")
  try:
    expected = validate_version(version)
  except ValueError as e:
    raise ProtocolError(str(e)) from e
  if wire_version != expected:
    raise ProtocolError(f"peer speaks protocol v{wire_version}, we speak protocol v{expected}")
  return magic, wire_version, msg_type, seq, flags, length, reserved


def pack_infer_req(frame_id: int, flags: int = 0) -> bytes:
  return _infer_req.pack(frame_id, flags)


def unpack_infer_req(buf, offset: int = 0) -> tuple[int, int]:
  return _infer_req.unpack_from(buf, offset)


def pack_infer_resp(frame_id: int, status: int, gpu_us: int, queue_us: int, total_us: int) -> bytes:
  return _infer_resp.pack(frame_id, status, gpu_us, queue_us, total_us)


def unpack_infer_resp(buf, offset: int = 0) -> tuple[int, int, int, int, int]:
  return _infer_resp.unpack_from(buf, offset)


class Status(IntEnum):
  OK = 0
  NOT_READY = 1       # no engine loaded
  BAD_SHAPE = 2
  INFER_FAILED = 3
  NOT_FINITE = 4      # model produced NaN/Inf; caller must fall back
