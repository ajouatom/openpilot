import binascii
import ctypes
import os
import fcntl
import math
import time
import struct
import threading
from contextlib import contextmanager
from dataclasses import dataclass
from enum import IntEnum
from functools import reduce
from collections.abc import Callable

from .base import BaseHandle, BaseSTBootloaderHandle, TIMEOUT
from .constants import McuType, MCU_TYPE_BY_IDCODE, USBPACKET_MAX_SIZE
from .utils import logger

try:
  import spidev
except ImportError:
  spidev = None

# Constants
SYNC = 0x5A
HACK = 0x79
DACK = 0x85
NACK = 0x1F
CHECKSUM_START = 0xAB

MIN_ACK_TIMEOUT_MS = 100
MAX_XFER_RETRY_COUNT = 5
SPI_V2_TURNAROUND_NS = 400_000

XFER_SIZE = 0x40*31

DEV_PATH = "/dev/spidev0.0"

SPI_V3_MAGIC = b"\x5a\xc3\x69\x96"
SPI_V3_VERSION = 3
SPI_V3_HEADER_SIZE = 28
SPI_V3_TRAILER_SIZE = 4
SPI_V3_WIRE_BUFFER_SIZE = 2048
SPI_V3_MAX_PAYLOAD_SIZE = SPI_V3_WIRE_BUFFER_SIZE - SPI_V3_HEADER_SIZE - SPI_V3_TRAILER_SIZE
SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE = 512
SPI_V3_FILLER = 0xCD
SPI_V3_POLL_CHUNK_SIZE = 256
SPI_V3_MAX_FILLER_PREFIX_BYTES = 512
SPI_V3_POLL_INTERVAL_US = 250
SPI_V3_RESPONSE_ATTEMPT_TIMEOUT_MS = 8
SPI_V3_TRANSFER_TIMEOUT_MS = 100

SPI_VERSION_REQUEST = b"VERSION"
SPI_VERSION_MAX_DATA_SIZE = 1000
SPI_VERSION_METADATA_SIZE = 15
# VERSION is a legacy-shaped response even when it selects v3. Legacy Panda
# firmware re-arms its 7-byte header RX DMA as soon as the final response byte
# leaves TX DMA, so a fixed-size read may clock dummy bytes into that new
# header. A new process may also inherit one maximum-sized v3 response from a
# host that died mid-transfer. Poll one byte at a time, budget that full stale
# frame plus service margin, and stop exactly at the valid VERSION CRC byte.
SPI_VERSION_POLL_SERVICE_MARGIN_BYTES = 64
SPI_VERSION_MAX_POLL_BYTES = SPI_V3_WIRE_BUFFER_SIZE + SPI_VERSION_POLL_SERVICE_MARGIN_BYTES
SPI_VERSION_POLL_INTERVAL_US = 100


class SpiV3FrameType(IntEnum):
  REQUEST = 1
  RESPONSE = 2


class SpiV3Status(IntEnum):
  OK = 0
  BAD_FRAME = 1
  UNSUPPORTED_VERSION = 2
  BAD_LENGTH = 3
  BAD_ENDPOINT = 4
  BUSY = 5
  SEQUENCE_CONFLICT = 6
  INTERNAL_ERROR = 7


@dataclass(frozen=True)
class SpiV3Frame:
  frame_type: SpiV3FrameType
  status: SpiV3Status
  session_id: int
  sequence: int
  endpoint: int
  payload: bytes = b""
  max_response_length: int = 0
  flags: int = 0


def _make_crc32c_table() -> tuple[int, ...]:
  table = []
  for byte in range(256):
    crc = byte
    for _ in range(8):
      crc = (crc >> 1) ^ (0x82F63B78 if (crc & 1) else 0)
    table.append(crc)
  return tuple(table)


_CRC32C_TABLE = _make_crc32c_table()


def spi_v3_crc32c(data: bytes | bytearray | memoryview) -> int:
  crc = 0xFFFFFFFF
  for byte in data:
    crc = _CRC32C_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
  return crc ^ 0xFFFFFFFF


def spi_v3_encode_frame(frame: SpiV3Frame) -> bytes:
  payload = bytes(frame.payload)
  if frame.frame_type not in (SpiV3FrameType.REQUEST, SpiV3FrameType.RESPONSE):
    raise ValueError("invalid SPI v3 frame type")
  if frame.status not in SpiV3Status:
    raise ValueError("invalid SPI v3 status")
  if not 0 <= frame.session_id <= 0xFFFFFFFF or not 0 <= frame.sequence <= 0xFFFFFFFF:
    raise ValueError("invalid SPI v3 session or sequence")
  if not 0 <= frame.endpoint <= 0xFF or frame.flags != 0:
    raise ValueError("invalid SPI v3 frame fields")
  if len(payload) > SPI_V3_MAX_PAYLOAD_SIZE or not 0 <= frame.max_response_length <= SPI_V3_MAX_PAYLOAD_SIZE:
    raise ValueError("SPI v3 payload is too large")
  if frame.frame_type == SpiV3FrameType.REQUEST:
    if frame.status != SpiV3Status.OK:
      raise ValueError("SPI v3 request has non-OK status")
  elif frame.max_response_length != 0:
    raise ValueError("SPI v3 response has a request length limit")

  header = struct.pack(
    "<4sBBBBIIBBHHH", SPI_V3_MAGIC, SPI_V3_VERSION, frame.frame_type, frame.flags, frame.status,
    frame.session_id, frame.sequence, frame.endpoint, 0, len(payload), frame.max_response_length, 0,
  )
  header += struct.pack("<I", spi_v3_crc32c(header))
  encoded = header + payload
  return encoded + struct.pack("<I", spi_v3_crc32c(encoded))


class SpiV3StreamDecoder:
  def __init__(self, max_payload_size: int = SPI_V3_MAX_PAYLOAD_SIZE):
    if not 0 <= max_payload_size <= SPI_V3_MAX_PAYLOAD_SIZE:
      raise ValueError("invalid SPI v3 decoder payload limit")
    self.max_payload_size = max_payload_size
    self.buffer = bytearray()
    self.bad_header_crc = 0
    self.bad_frame_crc = 0
    self.bad_version = 0
    self.bad_length = 0
    self.bad_fields = 0

  def _discard_prefix(self, length: int) -> None:
    del self.buffer[:length]

  def feed(self, data: bytes | bytearray | memoryview) -> list[SpiV3Frame]:
    self.buffer.extend(data)
    frames = []
    while True:
      magic_offset = self.buffer.find(SPI_V3_MAGIC)
      if magic_offset < 0:
        keep = 0
        for candidate in range(min(len(self.buffer), len(SPI_V3_MAGIC) - 1), 0, -1):
          if self.buffer[-candidate:] == SPI_V3_MAGIC[:candidate]:
            keep = candidate
            break
        self._discard_prefix(len(self.buffer) - keep)
        break

      self._discard_prefix(magic_offset)
      if len(self.buffer) < SPI_V3_HEADER_SIZE:
        break

      received_header_crc = struct.unpack_from("<I", self.buffer, 24)[0]
      if spi_v3_crc32c(memoryview(self.buffer)[:24]) != received_header_crc:
        self.bad_header_crc += 1
        self._discard_prefix(1)
        continue
      if self.buffer[4] != SPI_V3_VERSION:
        self.bad_version += 1
        self._discard_prefix(1)
        continue

      payload_length, max_response_length = struct.unpack_from("<HH", self.buffer, 18)
      if payload_length > self.max_payload_size or max_response_length > SPI_V3_MAX_PAYLOAD_SIZE:
        self.bad_length += 1
        self._discard_prefix(1)
        continue

      raw_type, flags, raw_status = self.buffer[5:8]
      try:
        frame_type = SpiV3FrameType(raw_type)
        status = SpiV3Status(raw_status)
      except ValueError:
        self.bad_fields += 1
        self._discard_prefix(1)
        continue

      semantic_fields = (
        flags == 0 and self.buffer[17] == 0 and struct.unpack_from("<H", self.buffer, 22)[0] == 0 and
        ((frame_type == SpiV3FrameType.REQUEST and status == SpiV3Status.OK) or
         (frame_type == SpiV3FrameType.RESPONSE and max_response_length == 0))
      )
      if not semantic_fields:
        self.bad_fields += 1
        self._discard_prefix(1)
        continue

      frame_length = SPI_V3_HEADER_SIZE + payload_length + SPI_V3_TRAILER_SIZE
      if len(self.buffer) < frame_length:
        break
      received_frame_crc = struct.unpack_from("<I", self.buffer, frame_length - SPI_V3_TRAILER_SIZE)[0]
      if spi_v3_crc32c(memoryview(self.buffer)[:frame_length - SPI_V3_TRAILER_SIZE]) != received_frame_crc:
        self.bad_frame_crc += 1
        self._discard_prefix(1)
        continue

      session_id, sequence = struct.unpack_from("<II", self.buffer, 8)
      frames.append(SpiV3Frame(
        frame_type=frame_type,
        status=status,
        session_id=session_id,
        sequence=sequence,
        endpoint=self.buffer[16],
        payload=bytes(self.buffer[SPI_V3_HEADER_SIZE:SPI_V3_HEADER_SIZE + payload_length]),
        max_response_length=max_response_length,
        flags=flags,
      ))
      self._discard_prefix(frame_length)

    return frames


def crc8(data):
  crc = 0xFF    # standard init value
  poly = 0xD5   # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size - 1, -1, -1):
    crc ^= data[i]
    for _ in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc


class SpiVersionStreamDecoder:
  """Decoder for the stable, legacy-shaped VERSION discovery response."""

  def __init__(self):
    self.buffer = bytearray()
    self.bad_checksum = 0
    self.bad_length = 0

  def feed(self, data: bytes | bytearray | memoryview) -> bytes | None:
    self.buffer.extend(data)
    while True:
      version_offset = self.buffer.find(SPI_VERSION_REQUEST)
      if version_offset < 0:
        keep = 0
        for candidate in range(min(len(self.buffer), len(SPI_VERSION_REQUEST) - 1), 0, -1):
          if self.buffer[-candidate:] == SPI_VERSION_REQUEST[:candidate]:
            keep = candidate
            break
        del self.buffer[:len(self.buffer) - keep]
        return None

      del self.buffer[:version_offset]
      data_offset = len(SPI_VERSION_REQUEST) + 2
      if len(self.buffer) < data_offset:
        return None
      data_length = struct.unpack_from("<H", self.buffer, len(SPI_VERSION_REQUEST))[0]
      if not SPI_VERSION_METADATA_SIZE <= data_length <= SPI_VERSION_MAX_DATA_SIZE:
        self.bad_length += 1
        del self.buffer[0]
        continue

      packet_length = data_offset + data_length + 1
      if len(self.buffer) < packet_length:
        return None
      if crc8(memoryview(self.buffer)[:packet_length - 1]) != self.buffer[packet_length - 1]:
        self.bad_checksum += 1
        del self.buffer[0]
        continue

      response = bytes(self.buffer[data_offset:data_offset + data_length])
      del self.buffer[:packet_length]
      return response


class PandaSpiException(Exception):
  pass

class PandaProtocolMismatch(PandaSpiException):
  pass

class PandaSpiUnavailable(PandaSpiException):
  pass

class PandaSpiNackResponse(PandaSpiException):
  pass

class PandaSpiMissingAck(PandaSpiException):
  pass

class PandaSpiBadChecksum(PandaSpiException):
  pass

class PandaSpiTransferFailed(PandaSpiException):
  pass


class PandaSpiV3RemoteError(PandaSpiException):
  def __init__(self, status: SpiV3Status):
    self.status = status
    super().__init__(f"SPI v3 request failed with status {status.name}")


class PandaSpiTransfer(ctypes.Structure):
  _fields_ = [
    ('rx_buf', ctypes.c_uint64),
    ('tx_buf', ctypes.c_uint64),
    ('tx_length', ctypes.c_uint32),
    ('rx_length_max', ctypes.c_uint32),
    ('timeout', ctypes.c_uint32),
    ('endpoint', ctypes.c_uint8),
    ('expect_disconnect', ctypes.c_uint8),
  ]


SPI_LOCK = threading.Lock()
SPI_DEVICES = {}
class SpiDevice:
  """
  Provides locked, thread-safe access to a panda's SPI interface.
  """

  # 50MHz is the max of the 845. older rev comma three
  # may not support the full 50MHz
  MAX_SPEED = 50000000

  def __init__(self, speed=MAX_SPEED):
    assert speed <= self.MAX_SPEED

    if not os.path.exists(DEV_PATH):
      raise PandaSpiUnavailable(f"SPI device not found: {DEV_PATH}")
    if spidev is None:
      raise PandaSpiUnavailable("spidev is not installed")

    with SPI_LOCK:
      if speed not in SPI_DEVICES:
        SPI_DEVICES[speed] = spidev.SpiDev()  # pylint: disable=c-extension-no-member
        SPI_DEVICES[speed].open(0, 0)
        SPI_DEVICES[speed].max_speed_hz = speed
      self._spidev = SPI_DEVICES[speed]

  @contextmanager
  def acquire(self):
    try:
      SPI_LOCK.acquire()
      fcntl.flock(self._spidev, fcntl.LOCK_EX)
      yield self._spidev
    finally:
      fcntl.flock(self._spidev, fcntl.LOCK_UN)
      SPI_LOCK.release()

  def close(self):
    pass


class PandaSpiHandle(BaseHandle):
  """
  A class that mimics a libusb1 handle for panda SPI communications.
  """

  PROTOCOL_VERSION = SPI_V3_VERSION
  SUPPORTED_PROTOCOL_VERSIONS = (2, SPI_V3_VERSION)

  def __init__(self) -> None:
    self.dev = SpiDevice()
    self.protocol_version = 2
    self._v3_session_id = int.from_bytes(os.urandom(4), "little") or 1
    self._v3_next_sequence = 1

    self._transfer_v2_raw: Callable[[SpiDevice, int, bytes, int, int, bool], bytes] = self._transfer_spidev

    if "KERN" in os.environ:
      # The out-of-tree ioctl performs HACK->payload entirely in the kernel and
      # has no 400 us RX-DMA turnaround. Sleeping around the ioctl cannot fix
      # that internal race, so explicitly bypass KERN until its driver grows
      # the same guard. Direct spidev keeps v2 bootstub/F4 flashing reliable.
      logger.warning("KERN Panda SPI transport lacks the required v2 turnaround; using direct spidev")

  def set_protocol_version(self, version: int) -> None:
    if version not in self.SUPPORTED_PROTOCOL_VERSIONS:
      supported = ", ".join(str(v) for v in self.SUPPORTED_PROTOCOL_VERSIONS)
      raise PandaProtocolMismatch(f"unsupported panda SPI protocol {version}; supported versions: {supported}")
    self.protocol_version = version

  # helpers
  def _calc_checksum(self, data: bytes) -> int:
    cksum = CHECKSUM_START
    for b in data:
      cksum ^= b
    return cksum

  def _wait_for_ack(self, spi, ack_val: int, timeout: int, tx: int, length: int = 1) -> bytes:
    timeout_s = max(MIN_ACK_TIMEOUT_MS, timeout) * 1e-3

    start = time.monotonic()
    while (timeout == 0) or ((time.monotonic() - start) < timeout_s):
      dat = spi.xfer2([tx, ] * length)
      if dat[0] == NACK:
        raise PandaSpiNackResponse
      elif dat[0] == ack_val:
        return bytes(dat)

    raise PandaSpiMissingAck

  @staticmethod
  def _wait_for_v2_turnaround(start_ns: int) -> None:
    deadline_ns = start_ns + SPI_V2_TURNAROUND_NS
    while True:
      remaining_ns = deadline_ns - time.monotonic_ns()
      if remaining_ns <= 0:
        return
      time.sleep(remaining_ns * 1e-9)

  def _transfer_spidev(self, spi, endpoint: int, data, timeout: int, max_rx_len: int = 1000, expect_disconnect: bool = False) -> bytes:
    max_rx_len = max(USBPACKET_MAX_SIZE, max_rx_len)
    bus_active = False
    try:
      logger.debug("- send header")
      packet = struct.pack("<BBHH", SYNC, endpoint, len(data), max_rx_len)
      packet += bytes([self._calc_checksum(packet), ])
      bus_active = True
      spi.xfer2(packet)

      logger.debug("- waiting for header ACK")
      self._wait_for_ack(spi, HACK, MIN_ACK_TIMEOUT_MS, 0x11)

      # V2 Panda switches its RX DMA phase from the HACK TX-complete ISR.
      # Clocking the payload too soon races that re-arm, especially at 50 MHz.
      self._wait_for_v2_turnaround(time.monotonic_ns())

      logger.debug("- sending data")
      packet = bytes([*data, self._calc_checksum(data)])
      spi.xfer2(packet)

      if expect_disconnect:
        logger.debug("- expecting disconnect, returning")
        return b""

      logger.debug("- waiting for data ACK")
      preread_len = USBPACKET_MAX_SIZE + 1  # read enough for a controlRead
      dat = self._wait_for_ack(spi, DACK, timeout, 0x13, length=3 + preread_len)

      # get response length, then response
      response_len = struct.unpack("<H", dat[1:3])[0]
      if response_len > max_rx_len:
        raise PandaSpiException(f"response length greater than max ({max_rx_len} {response_len})")

      # read rest
      remaining = (response_len + 1) - preread_len
      if remaining > 0:
        dat += bytes(spi.readbytes(remaining))

      dat = dat[:3 + response_len + 1]
      if self._calc_checksum(dat) != 0:
        raise PandaSpiBadChecksum

      return dat[3:-1]
    finally:
      if bus_active:
        # Hold SpiDevice's thread lock and flock for the full terminal quiet
        # time, so another process cannot start the next v2 header early.
        self._wait_for_v2_turnaround(time.monotonic_ns())

  def _transfer_kernel_driver(self, spi, endpoint: int, data, timeout: int, max_rx_len: int = 1000, expect_disconnect: bool = False) -> bytes:
    raise PandaSpiUnavailable("KERN Panda SPI transport does not implement the required v2 400 us turnaround")

  @staticmethod
  def _inspect_v3_frames(frames: list[SpiV3Frame], request: SpiV3Frame) -> tuple[str, bytes | None]:
    for frame in frames:
      if frame.frame_type != SpiV3FrameType.RESPONSE or frame.session_id != request.session_id or frame.sequence != request.sequence:
        # A delayed response from an earlier request or connection is safe to
        # consume and ignore because v3 is self-framed.
        continue
      if frame.endpoint != request.endpoint:
        raise PandaSpiException(f"SPI v3 response endpoint mismatch ({request.endpoint} {frame.endpoint})")
      if frame.max_response_length != 0:
        raise PandaSpiException("SPI v3 response contains a request length limit")
      if len(frame.payload) > request.max_response_length:
        raise PandaSpiException(f"response length greater than max ({request.max_response_length} {len(frame.payload)})")
      if frame.status == SpiV3Status.BUSY:
        return "retry", None
      if frame.status != SpiV3Status.OK:
        raise PandaSpiV3RemoteError(frame.status)
      return "complete", frame.payload
    return "continue", None

  @staticmethod
  def _clock_v3(spi, tx: bytes, decoder: SpiV3StreamDecoder) -> list[SpiV3Frame]:
    try:
      rx = spi.xfer2(tx)
    except (OSError, ValueError) as e:
      raise PandaSpiException("SPI v3 transfer failed") from e
    if len(rx) != len(tx):
      raise PandaSpiException(f"SPI v3 transfer length mismatch ({len(tx)} {len(rx)})")
    return decoder.feed(bytes(rx))

  def _transfer_v3_spidev(self, spi, endpoint: int, data, timeout: int, max_rx_len: int, expect_disconnect: bool) -> bytes:
    payload = bytes(data)
    if not 0 <= endpoint <= 0xFF:
      raise PandaSpiException(f"invalid SPI v3 endpoint: {endpoint}")
    if len(payload) > SPI_V3_MAX_PAYLOAD_SIZE or not 0 <= max_rx_len <= SPI_V3_MAX_PAYLOAD_SIZE:
      raise PandaSpiException(
        f"SPI v3 transfer exceeds {SPI_V3_MAX_PAYLOAD_SIZE}-byte payload limit ({len(payload)} {max_rx_len})",
      )
    if endpoint == 3 and len(payload) > SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE:
      raise PandaSpiException(
        f"SPI v3 endpoint 3 write exceeds {SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE}-byte payload limit ({len(payload)})",
      )

    sequence = self._v3_next_sequence
    self._v3_next_sequence = (self._v3_next_sequence + 1) & 0xFFFFFFFF
    request = SpiV3Frame(
      frame_type=SpiV3FrameType.REQUEST,
      status=SpiV3Status.OK,
      session_id=self._v3_session_id,
      sequence=sequence,
      endpoint=endpoint,
      payload=payload,
      max_response_length=max_rx_len,
    )
    # Encode exactly once. A timeout or BUSY response must resend these exact
    # bytes so Panda's replay cache cannot execute endpoint 3 twice.
    encoded_request = spi_v3_encode_frame(request)
    decoder = SpiV3StreamDecoder()

    if expect_disconnect:
      self._clock_v3(spi, encoded_request, decoder)
      return b""

    overall_timeout_ms = max(SPI_V3_TRANSFER_TIMEOUT_MS, timeout)
    deadline = time.monotonic() + overall_timeout_ms * 1e-3
    maximum_attempts = max(1, math.ceil(overall_timeout_ms / SPI_V3_RESPONSE_ATTEMPT_TIMEOUT_MS))
    for attempt in range(maximum_attempts):
      if attempt != 0 and time.monotonic() >= deadline:
        break
      attempt_start = time.monotonic()
      frames = self._clock_v3(spi, encoded_request, decoder)
      action, response = self._inspect_v3_frames(frames, request)
      if action == "complete":
        assert response is not None
        return response

      # Never clock more than one maximum-sized response plus 512 bytes of
      # leading filler per attempt. This keeps a missing response from
      # overflowing Panda's 8 KiB circular RX ring before its main loop drains.
      scan_bytes_remaining = SPI_V3_HEADER_SIZE + SPI_V3_TRAILER_SIZE + max_rx_len + SPI_V3_MAX_FILLER_PREFIX_BYTES
      while action == "continue" and scan_bytes_remaining > 0 and time.monotonic() < deadline:
        poll_size = min(SPI_V3_POLL_CHUNK_SIZE, scan_bytes_remaining)
        filler = bytes([SPI_V3_FILLER]) * poll_size
        frames = self._clock_v3(spi, filler, decoder)
        scan_bytes_remaining -= poll_size
        action, response = self._inspect_v3_frames(frames, request)
        if action == "continue" and scan_bytes_remaining > 0 and SPI_V3_POLL_INTERVAL_US:
          time.sleep(SPI_V3_POLL_INTERVAL_US * 1e-6)
      if action == "complete":
        assert response is not None
        return response
      now = time.monotonic()
      if now >= deadline or attempt + 1 >= maximum_attempts:
        break

      # BUSY and a response-attempt timeout both retry encoded_request without
      # changing its session, sequence, header, payload, or CRCs. Keep request
      # starts at least 8 ms apart so firmware has time to drain received bytes.
      next_attempt = min(deadline, attempt_start + SPI_V3_RESPONSE_ATTEMPT_TIMEOUT_MS * 1e-3)
      if now < next_attempt:
        time.sleep(next_attempt - now)

    if decoder.bad_header_crc or decoder.bad_frame_crc:
      raise PandaSpiBadChecksum
    raise PandaSpiMissingAck

  def _transfer_v2(self, endpoint: int, data, timeout: int, max_rx_len: int, expect_disconnect: bool) -> bytes:
    logger.debug("starting transfer: endpoint=%d, max_rx_len=%d", endpoint, max_rx_len)
    logger.debug("==============================================")

    n = 0
    start_time = time.monotonic()
    exc = PandaSpiException()
    while (timeout == 0) or (time.monotonic() - start_time) < timeout*1e-3:
      n += 1
      logger.debug("\ntry #%d", n)
      with self.dev.acquire() as spi:
        try:
          return self._transfer_v2_raw(spi, endpoint, data, timeout, max_rx_len, expect_disconnect)
        except PandaSpiException as e:
          exc = e
          logger.debug("SPI transfer failed, retrying", exc_info=True)

    raise exc

  def _transfer(self, endpoint: int, data, timeout: int, max_rx_len: int = 1000, expect_disconnect: bool = False) -> bytes:
    logger.debug("starting transfer: protocol=%d, endpoint=%d, max_rx_len=%d", self.protocol_version, endpoint, max_rx_len)
    if self.protocol_version == SPI_V3_VERSION:
      # Keep the whole stop-and-wait exchange under both locks. This prevents a
      # second process from interleaving a request between this request and its
      # response polls or byte-identical retries.
      with self.dev.acquire() as spi:
        return self._transfer_v3_spidev(spi, endpoint, data, timeout, max_rx_len, expect_disconnect)
    return self._transfer_v2(endpoint, data, timeout, max_rx_len, expect_disconnect)

  def get_protocol_version(self) -> bytes:
    exc = PandaSpiException()
    with self.dev.acquire() as spi:
      decoder = SpiVersionStreamDecoder()
      bus_active = False
      try:
        # Send VERSION once. Re-sending a seven-byte request while an earlier
        # response is partly shifted could cross its final byte and become a
        # bogus v2 header. Include the request transaction's MISO in the same
        # stream decoder because a delayed response may already start there.
        bus_active = True
        request_rx = bytes(spi.xfer2(SPI_VERSION_REQUEST))
        response = decoder.feed(request_rx)
        if response is not None:
          return response

        # v2 arms VERSION TX from its RX-complete ISR; v3 does so from the main
        # service loop. This one-time guard covers both without imposing the
        # old delay on normal v3 transfers.
        self._wait_for_v2_turnaround(time.monotonic_ns())
        for poll in range(SPI_VERSION_MAX_POLL_BYTES):
          try:
            chunk = bytes(spi.readbytes(1))
          except (OSError, ValueError) as e:
            raise PandaSpiException("SPI VERSION transfer failed") from e
          response = decoder.feed(chunk)
          if response is not None:
            return response
          if decoder.bad_checksum:
            raise PandaSpiBadChecksum
          if decoder.bad_length:
            raise PandaSpiException("SPI VERSION response length invalid")
          if poll + 1 < SPI_VERSION_MAX_POLL_BYTES:
            time.sleep(SPI_VERSION_POLL_INTERVAL_US * 1e-6)

        raise PandaSpiMissingAck
      except (OSError, ValueError, PandaSpiException) as e:
        if isinstance(e, PandaSpiException):
          exc = e
        else:
          exc = PandaSpiException("SPI VERSION transfer failed")
        logger.debug("SPI get protocol version failed", exc_info=True)
      finally:
        if bus_active:
          # Hold flock through the terminal quiet period before a v2 fallback
          # header. On a valid response the preceding poll stopped at its CRC.
          self._wait_for_v2_turnaround(time.monotonic_ns())
    raise exc

  # libusb1 functions
  def close(self):
    self.dev.close()

  def controlWrite(self, request_type: int, request: int, value: int, index: int, data, timeout: int = TIMEOUT, expect_disconnect: bool = False):
    max_rx_len = 0 if self.protocol_version == SPI_V3_VERSION else 1000
    return self._transfer(0, struct.pack("<BHHH", request, value, index, 0), timeout, max_rx_len=max_rx_len, expect_disconnect=expect_disconnect)

  def controlRead(self, request_type: int, request: int, value: int, index: int, length: int, timeout: int = TIMEOUT):
    return self._transfer(0, struct.pack("<BHHH", request, value, index, length), timeout, max_rx_len=length)

  def bulkWrite(self, endpoint: int, data: bytes, timeout: int = TIMEOUT) -> int:
    max_rx_len = 0 if self.protocol_version == SPI_V3_VERSION else 1000
    xfer_size = SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE if self.protocol_version == SPI_V3_VERSION and endpoint == 3 else XFER_SIZE
    for x in range(math.ceil(len(data) / xfer_size)):
      self._transfer(endpoint, data[xfer_size*x:xfer_size*(x+1)], timeout, max_rx_len=max_rx_len)
    return len(data)

  def bulkRead(self, endpoint: int, length: int, timeout: int = TIMEOUT) -> bytes:
    ret = b""
    for _ in range(math.ceil(length / XFER_SIZE)):
      d = self._transfer(endpoint, [], timeout, max_rx_len=XFER_SIZE)
      ret += d
      if len(d) < XFER_SIZE:
        break
    return ret


class STBootloaderSPIHandle(BaseSTBootloaderHandle):
  """
    Implementation of the STM32 SPI bootloader protocol described in:
    https://www.st.com/resource/en/application_note/an4286-spi-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf
  """

  SYNC = 0x5A
  ACK = 0x79
  NACK = 0x1F

  def __init__(self):
    self.dev = SpiDevice(speed=1000000)

    # say hello
    try:
      with self.dev.acquire() as spi:
        spi.xfer([self.SYNC, ])
        try:
          self._get_ack(spi, 0.1)
        except (PandaSpiNackResponse, PandaSpiMissingAck):
          # NACK ok here, will only ACK the first time
          pass

      self._mcu_type = MCU_TYPE_BY_IDCODE[self.get_chip_id()]
    except PandaSpiException:
      raise PandaSpiException("failed to connect to panda") from None

  def _get_ack(self, spi, timeout=1.0):
    data = 0x00
    start_time = time.monotonic()
    while data not in (self.ACK, self.NACK) and (time.monotonic() - start_time < timeout):
      data = spi.xfer([0x00, ])[0]
      time.sleep(0)
    spi.xfer([self.ACK, ])

    if data == self.NACK:
      raise PandaSpiNackResponse
    elif data != self.ACK:
      raise PandaSpiMissingAck

  def _cmd_no_retry(self, cmd: int, data: list[bytes] | None = None, read_bytes: int = 0, predata=None) -> bytes:
    ret = b""
    with self.dev.acquire() as spi:
      # sync + command
      spi.xfer([self.SYNC, ])
      spi.xfer([cmd, cmd ^ 0xFF])
      self._get_ack(spi, timeout=0.01)

      # "predata" - for commands that send the first data without a checksum
      if predata is not None:
        spi.xfer(predata)
        self._get_ack(spi)

      # send data
      if data is not None:
        for d in data:
          if predata is not None:
            spi.xfer(d + self._checksum(predata + d))
          else:
            spi.xfer(d + self._checksum(d))
          self._get_ack(spi, timeout=20)

      # receive
      if read_bytes > 0:
        ret = spi.xfer([0x00, ]*(read_bytes + 1))[1:]
        if data is None or len(data) == 0:
          self._get_ack(spi)

    return bytes(ret)

  def _cmd(self, cmd: int, data: list[bytes] | None = None, read_bytes: int = 0, predata=None) -> bytes:
    exc = PandaSpiException()
    for n in range(MAX_XFER_RETRY_COUNT):
      try:
        return self._cmd_no_retry(cmd, data, read_bytes, predata)
      except PandaSpiException as e:
        exc = e
        logger.debug("SPI transfer failed, %d retries left", MAX_XFER_RETRY_COUNT - n - 1, exc_info=True)
    raise exc

  def _checksum(self, data: bytes) -> bytes:
    if len(data) == 1:
      ret = data[0] ^ 0xFF
    else:
      ret = reduce(lambda a, b: a ^ b, data)
    return bytes([ret, ])

  # *** Bootloader commands ***

  def read(self, address: int, length: int):
    data = [struct.pack('>I', address), struct.pack('B', length - 1)]
    return self._cmd(0x11, data=data, read_bytes=length)

  def get_bootloader_id(self):
    return self.read(0x1FF1E7FE, 1)

  def get_chip_id(self) -> int:
    r = self._cmd(0x02, read_bytes=3)
    if r[0] != 1: # response length - 1
      raise PandaSpiException("incorrect response length")
    return ((r[1] << 8) + r[2])

  def go_cmd(self, address: int) -> None:
    self._cmd(0x21, data=[struct.pack('>I', address), ])

  # *** helpers ***

  def get_uid(self):
    dat = self.read(McuType.H7.config.uid_address, 12)
    return binascii.hexlify(dat).decode()

  def erase_sector(self, sector: int):
    p = struct.pack('>H', 0)  # number of sectors to erase
    d = struct.pack('>H', sector)
    self._cmd(0x44, data=[d, ], predata=p)

  # *** PandaDFU API ***

  def get_mcu_type(self):
    return self._mcu_type

  def clear_status(self):
    pass

  def close(self):
    self.dev.close()

  def program(self, address, dat):
    bs = 256  # max block size for writing to flash over SPI
    dat += b"\xFF" * ((bs - len(dat)) % bs)
    for i in range(len(dat) // bs):
      block = dat[i * bs:(i + 1) * bs]
      self._cmd(0x31, data=[
        struct.pack('>I', address + i*bs),
        bytes([len(block) - 1]) + block,
      ])

  def jump(self, address):
    self.go_cmd(self._mcu_type.config.bootstub_address)
