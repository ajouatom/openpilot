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
# The comma package and the server are updated together, so each speaks exactly
# one version, and a header of any other is a broken stream. 3 keeps the hidden
# state on the server: INFER_REQ has no prev_feat and INFER_RESP no
# hidden_state slice, which takes the big models' reply from 72 KB (five 16 KB
# reads on the comma) to 8 KB (one). Tensors are unchanged, so existing plans
# still load.
VERSION = 3

# A bulk transfer ends on a short packet, so a message that is an exact multiple
# of the packet size never terminates the peer's read and arrives a frame late;
# the sender appends a pad byte and sets Flag.PADDED. 512 is a high-speed packet
# and divides a SuperSpeed one, so every message ends short on a USB 2 link too:
# an Android phone, or a cable that fell back to high speed. A reader takes the
# pad byte whenever the flag says so, at any length.
PACKET_MULTIPLE = 512

# The gadget pads every message to a burst so it never ends on a short packet:
# dwc3 flushed its TX FIFO past one about once in 400 frames, and the host read
# those bytes as the next header. Host to device keeps the PADDED byte instead.
GADGET_TX_ALIGN = 16 * 1024

# The comma's gadget as a USB host finds it: these IDs (pid.codes' test
# allocation; get a real PID before distributing this), which
# scripts/comma/jetlink-root.sh presents, then the link's interface by its class
# triple (transport/ffs.py), because the comma set to iOS presents a composite
# gadget and a reordered config could move the number. Endpoint addresses are
# read from the descriptors, never assumed: FunctionFS renumbers them at bind.
USB_VID = 0x1209
USB_PID = 0x0001
USB_VENDOR_CLASS = (0xFF, 0xFF, 0xFF)
USB_MAX_PACKET = 1024   # SuperSpeed bulk

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
  INFER_RESP = 9       # InferRespHeader + outputs(f32) but hidden_state, which stays on the server
  # 10, 11 were RESET_REQ/RESP; queues are cleared with Flag.RESET_QUEUES
  STATE_REQ = 12       # telemetry
  STATE_RESP = 13      # json
  ERROR = 14           # json: {error, detail}
  PING = 15            # on a connection with no hello and no ENGINE_REQ: ERROR no_hello
  PONG = 16
  SHUTDOWN_REQ = 17    # json: {reason} -> power the Jetson off for good; see JetlinkClient.shutdown
  SHUTDOWN_RESP = 18   # json: {ok, detail}
  LEAVE = 19           # json: {reason, ...what the client measured}; client -> server, no reply.
                       # The client stops using the link: it handed the model back or is exiting.
                       # The connection may stay for a later HELLO_REQ. An older server answers
                       # ERROR unknown_message, which the client discards (JetlinkClient.leave)


# Why a client stops using the link, LEAVE's json 'reason' (JetlinkClient.leave);
# the server logs each in words (Session.onLeave) and anything else verbatim
LEAVE_BEHIND = 'behind'            # modeld fell behind the large model
LEAVE_LOST = 'lost'                # the comma lost the link, said when it can still carry it
LEAVE_STOPPED = 'stopped'          # modeld stopped
LEAVE_PROVISIONED = 'provisioned'  # the provisioning run finished
LEAVE_REASONS = (LEAVE_BEHIND, LEAVE_LOST, LEAVE_STOPPED, LEAVE_PROVISIONED)


class Flag(IntEnum):
  RESET_QUEUES = 1 << 0   # on INFER_REQ: warm-start, clear history before this frame
  WANT_STATE = 1 << 1     # on INFER_REQ: append telemetry json to the response.
                          # Piggybacked because at 20 Hz a separate exchange
                          # would race a frame.
  WANT_HIDDEN = 1 << 2    # on INFER_REQ: keep hidden_state in the response, for
                          # a caller logging the whole output vector. 64 KB more
                          # on the big models; nothing else needs it.
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


def pack_header(msg_type: int, seq: int, length: int, flags: int = 0, reserved: int = 0) -> bytes:
  return _header.pack(MAGIC, VERSION, int(msg_type), seq, flags, length, reserved)


def unpack_header(buf) -> tuple[int, int, int, int, int, int, int]:
  magic, version, msg_type, seq, flags, length, reserved = _header.unpack_from(buf)
  if magic != MAGIC:
    raise ProtocolError(f"bad magic 0x{magic:08x} (link desynced or not a jetlink peer)")
  if version != VERSION:
    raise ProtocolError(f"peer speaks protocol v{version}, we speak v{VERSION}")
  return magic, version, msg_type, seq, flags, length, reserved


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
