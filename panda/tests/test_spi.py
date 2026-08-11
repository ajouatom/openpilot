import struct
import time
from contextlib import contextmanager

import pytest

import panda.python as panda_python
import panda.python.spi as spi_module
from panda import Panda
from panda.python.spi import (
  CHECKSUM_START,
  DACK,
  HACK,
  NACK,
  PandaProtocolMismatch,
  PandaSpiBadChecksum,
  PandaSpiException,
  PandaSpiHandle,
  PandaSpiMissingAck,
  PandaSpiNackResponse,
  PandaSpiV3RemoteError,
  SPI_V2_TURNAROUND_NS,
  SPI_VERSION_MAX_POLL_BYTES,
  SPI_VERSION_POLL_SERVICE_MARGIN_BYTES,
  SPI_VERSION_REQUEST,
  SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE,
  SPI_V3_FILLER,
  SPI_V3_HEADER_SIZE,
  SPI_V3_MAGIC,
  SPI_V3_MAX_PAYLOAD_SIZE,
  SPI_V3_TRAILER_SIZE,
  SPI_V3_WIRE_BUFFER_SIZE,
  SpiV3Frame,
  SpiV3FrameType,
  SpiV3Status,
  SpiV3StreamDecoder,
  SpiVersionStreamDecoder,
  XFER_SIZE,
  crc8,
  spi_v3_crc32c,
  spi_v3_encode_frame,
)


def response_for(request, status=SpiV3Status.OK, payload=b""):
  return SpiV3Frame(
    frame_type=SpiV3FrameType.RESPONSE,
    status=status,
    session_id=request.session_id,
    sequence=request.sequence,
    endpoint=request.endpoint,
    payload=payload,
  )


class FakeSpiV3:
  def __init__(self, response_plan=None, *, filler_prefix=7, drop_first_terminal=False, corrupt_first_terminal=False):
    self.response_plan = response_plan or (lambda request, call: response_for(request))
    self.filler_prefix = filler_prefix
    self.drop_first_terminal = drop_first_terminal
    self.corrupt_first_terminal = corrupt_first_terminal
    self.decoder = SpiV3StreamDecoder()
    self.pending = bytearray()
    self.request_wires = []
    self.response_calls = 0
    self.execution_count = 0
    self.cache = {}

  def _queue(self, encoded):
    self.pending.extend(bytes([SPI_V3_FILLER]) * self.filler_prefix)
    self.pending.extend(encoded)

  def xfer2(self, tx):
    tx = bytes(tx)

    # SPI is full duplex: only bytes queued before this CS assertion are
    # visible on MISO. A response produced by this MOSI request starts on a
    # later poll transaction.
    rx = bytes(self.pending[:len(tx)])
    del self.pending[:len(tx)]
    rx += bytes([SPI_V3_FILLER]) * (len(tx) - len(rx))

    for request in self.decoder.feed(tx):
      if request.frame_type != SpiV3FrameType.REQUEST:
        continue
      self.request_wires.append(tx)
      key = (request.session_id, request.sequence)
      encoded = self.cache.get(key)
      if encoded is not None:
        self._queue(encoded)
        continue

      self.response_calls += 1
      response = self.response_plan(request, self.response_calls)
      encoded = spi_v3_encode_frame(response)
      if response.status != SpiV3Status.BUSY:
        self.cache[key] = encoded
        if response.status == SpiV3Status.OK:
          self.execution_count += 1

      if self.drop_first_terminal and response.status != SpiV3Status.BUSY:
        self.drop_first_terminal = False
        continue
      if self.corrupt_first_terminal and response.status != SpiV3Status.BUSY:
        self.corrupt_first_terminal = False
        encoded = encoded[:-1] + bytes([encoded[-1] ^ 1])
      self._queue(encoded)

    return list(rx)


def make_v3_handle():
  handle = object.__new__(PandaSpiHandle)
  handle.protocol_version = 3
  handle._v3_session_id = 0x12345678
  handle._v3_next_sequence = 1
  return handle


def test_spi_v3_crc_frame_limits_and_stream_resync():
  assert spi_v3_crc32c(b"123456789") == 0xE3069283
  assert spi_v3_crc32c(b"") == 0

  wire_vector = SpiV3Frame(
    SpiV3FrameType.REQUEST, SpiV3Status.OK, 0x12345678, 0x89ABCDEF, 3,
    b"\x00\xcd\xff", 512,
  )
  assert spi_v3_encode_frame(wire_vector) == bytes.fromhex(
    "5ac369960301000078563412efcdab890300030000020000" +
    "831ad31800cdffd23fabd2",
  )

  payload = bytes((i & 0xFF) for i in range(SPI_V3_MAX_PAYLOAD_SIZE))
  maximum = SpiV3Frame(SpiV3FrameType.REQUEST, SpiV3Status.OK, 0x12345678, 0x89ABCDEF, 3, payload, SPI_V3_MAX_PAYLOAD_SIZE)
  encoded_maximum = spi_v3_encode_frame(maximum)
  assert len(encoded_maximum) == SPI_V3_WIRE_BUFFER_SIZE
  with pytest.raises(ValueError):
    spi_v3_encode_frame(SpiV3Frame(SpiV3FrameType.REQUEST, SpiV3Status.OK, 1, 1, 3, payload + b"x", 0))

  valid = spi_v3_encode_frame(response_for(maximum, payload=b"valid"))
  bad_header = bytearray(valid)
  bad_header[8] ^= 0x80
  bad_frame = bytearray(valid)
  bad_frame[SPI_V3_HEADER_SIZE] ^= 1
  stream = bytes([SPI_V3_FILLER]) * 31 + SPI_V3_MAGIC[:2] + bytes(bad_header) + bytes(bad_frame) + valid

  decoder = SpiV3StreamDecoder()
  decoded = []
  for offset in range(0, len(stream), 11):
    decoded.extend(decoder.feed(stream[offset:offset + 11]))
  assert decoded == [response_for(maximum, payload=b"valid")]
  assert decoder.bad_header_crc == 1
  assert decoder.bad_frame_crc == 1


def test_spi_v3_filler_scan_max_response_and_max_length_validation():
  payload = bytes((i & 0xFF) for i in range(SPI_V3_MAX_PAYLOAD_SIZE))
  fake = FakeSpiV3(lambda request, call: response_for(request, payload=payload), filler_prefix=509)
  handle = make_v3_handle()

  assert handle._transfer_v3_spidev(fake, 0xAB, b"", 100, SPI_V3_MAX_PAYLOAD_SIZE, False) == payload
  request = SpiV3StreamDecoder().feed(fake.request_wires[0])[0]
  assert request.max_response_length == SPI_V3_MAX_PAYLOAD_SIZE
  assert request.payload == b""
  with pytest.raises(PandaSpiException):
    handle._transfer_v3_spidev(fake, 1, b"", 100, SPI_V3_MAX_PAYLOAD_SIZE + 1, False)

  too_long = FakeSpiV3(lambda request, call: response_for(request, payload=b"12345"))
  with pytest.raises(PandaSpiException, match="response length greater than max"):
    handle._transfer_v3_spidev(too_long, 1, b"", 100, 4, False)


def test_spi_v3_endpoint3_limit_does_not_shrink_other_endpoints():
  payload = bytes(SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE + 1)
  handle = make_v3_handle()
  fake = FakeSpiV3()

  assert handle._transfer_v3_spidev(fake, 2, payload, 100, 0, False) == b""
  with pytest.raises(PandaSpiException, match="endpoint 3 write exceeds 512-byte"):
    handle._transfer_v3_spidev(fake, 3, payload, 100, 0, False)
  assert len(fake.request_wires) == 1


@pytest.mark.parametrize(("protocol_version", "endpoint", "length", "expected_chunks", "expected_max_rx_len"), [
  (3, 3, (SPI_V3_CAN_WRITE_MAX_PAYLOAD_SIZE * 2) + 1, [512, 512, 1], 0),
  (3, 2, XFER_SIZE + 1, [XFER_SIZE, 1], 0),
  (2, 3, XFER_SIZE + 1, [XFER_SIZE, 1], 1000),
])
def test_bulk_write_chunks_only_v3_endpoint3_at_512(monkeypatch, protocol_version, endpoint, length,
                                                     expected_chunks, expected_max_rx_len):
  handle = make_v3_handle()
  handle.protocol_version = protocol_version
  calls = []

  def transfer(call_endpoint, data, timeout, max_rx_len=1000, expect_disconnect=False):
    calls.append((call_endpoint, bytes(data), timeout, max_rx_len, expect_disconnect))
    return b""

  monkeypatch.setattr(handle, "_transfer", transfer)
  assert handle.bulkWrite(endpoint, bytes(length), timeout=17) == length
  assert [len(call[1]) for call in calls] == expected_chunks
  assert all(call[0] == endpoint and call[2] == 17 and call[3] == expected_max_rx_len for call in calls)


def test_spi_v3_busy_retries_byte_identical_request(monkeypatch):
  monkeypatch.setattr(spi_module, "SPI_V3_TRANSFER_TIMEOUT_MS", 20)

  def plan(request, call):
    return response_for(request, SpiV3Status.BUSY if call == 1 else SpiV3Status.OK)

  fake = FakeSpiV3(plan)
  handle = make_v3_handle()
  assert handle._transfer_v3_spidev(fake, 3, b"sendcan", 20, 0, False) == b""
  assert len(fake.request_wires) == 2
  assert fake.request_wires[0] == fake.request_wires[1]
  assert fake.execution_count == 1


def test_spi_v3_scan_budget_pacing_and_frozen_clock_are_finite(monkeypatch):
  class NoResponseSpi:
    def __init__(self):
      self.sizes = []

    def xfer2(self, tx):
      self.sizes.append(len(tx))
      return [SPI_V3_FILLER] * len(tx)

  sleeps = []
  monkeypatch.setattr(spi_module, "SPI_V3_TRANSFER_TIMEOUT_MS", 8)
  monkeypatch.setattr(spi_module, "SPI_V3_RESPONSE_ATTEMPT_TIMEOUT_MS", 8)
  monkeypatch.setattr(spi_module.time, "monotonic", lambda: 1.0)
  monkeypatch.setattr(spi_module.time, "sleep", sleeps.append)
  fake = NoResponseSpi()
  handle = make_v3_handle()

  with pytest.raises(PandaSpiMissingAck):
    handle._transfer_v3_spidev(fake, 1, b"", 8, 0, False)
  # One 32-byte request, then exactly 32 + max_rx(0) + 512 scan bytes.
  assert fake.sizes == [SPI_V3_HEADER_SIZE + SPI_V3_TRAILER_SIZE, 256, 256, 32]
  assert sleeps == pytest.approx([250e-6, 250e-6])


@pytest.mark.parametrize("corrupt_response", [False, True])
def test_spi_v3_lost_or_corrupt_response_replays_endpoint3_once(monkeypatch, corrupt_response):
  monkeypatch.setattr(spi_module, "SPI_V3_RESPONSE_ATTEMPT_TIMEOUT_MS", 1)
  monkeypatch.setattr(spi_module, "SPI_V3_TRANSFER_TIMEOUT_MS", 30)
  fake = FakeSpiV3(drop_first_terminal=not corrupt_response, corrupt_first_terminal=corrupt_response)
  handle = make_v3_handle()

  assert handle._transfer_v3_spidev(fake, 3, b"one CAN batch", 30, 0, False) == b""
  assert len(fake.request_wires) >= 2
  assert all(wire == fake.request_wires[0] for wire in fake.request_wires)
  assert fake.execution_count == 1
  assert handle._v3_next_sequence == 2


@pytest.mark.parametrize("status", [
  SpiV3Status.BAD_FRAME,
  SpiV3Status.UNSUPPORTED_VERSION,
  SpiV3Status.BAD_LENGTH,
  SpiV3Status.BAD_ENDPOINT,
  SpiV3Status.SEQUENCE_CONFLICT,
  SpiV3Status.INTERNAL_ERROR,
])
def test_spi_v3_terminal_status_is_not_retried(status):
  fake = FakeSpiV3(lambda request, call: response_for(request, status))
  handle = make_v3_handle()
  with pytest.raises(PandaSpiV3RemoteError) as exc:
    handle._transfer_v3_spidev(fake, 3, b"batch", 100, 0, False)
  assert exc.value.status == status
  assert len(fake.request_wires) == 1


class FakeSpiV2:
  def __init__(self, nack_first_data=False):
    self.nack_first_data = nack_first_data
    self.events = []
    self.raw_count = 0

  def xfer2(self, tx):
    now = time.monotonic_ns()
    if isinstance(tx, list) and tx[0] == 0x11:
      self.events.append(("hack", now))
      return [HACK] * len(tx)
    if isinstance(tx, list) and tx[0] == 0x13:
      if self.nack_first_data:
        self.nack_first_data = False
        self.events.append(("nack", now))
        return [NACK] * len(tx)
      self.events.append(("dack", now))
      checksum = CHECKSUM_START ^ DACK
      return [DACK, 0, 0, checksum] + ([0] * (len(tx) - 4))

    event = "header" if (self.raw_count % 2) == 0 else "payload"
    self.raw_count += 1
    self.events.append((event, now))
    return [0] * len(tx)


def event_times(fake, name):
  return [timestamp for event, timestamp in fake.events if event == name]


def test_spi_v2_hack_and_terminal_quiet_time():
  handle = object.__new__(PandaSpiHandle)
  fake = FakeSpiV2()

  assert handle._transfer_spidev(fake, 0, b"x", 100) == b""
  assert handle._transfer_spidev(fake, 0, b"y", 100) == b""
  hacks = event_times(fake, "hack")
  payloads = event_times(fake, "payload")
  dacks = event_times(fake, "dack")
  headers = event_times(fake, "header")
  assert payloads[0] - hacks[0] >= SPI_V2_TURNAROUND_NS
  assert payloads[1] - hacks[1] >= SPI_V2_TURNAROUND_NS
  assert headers[1] - dacks[0] >= SPI_V2_TURNAROUND_NS


def test_spi_v2_nack_to_next_header_quiet_time():
  handle = object.__new__(PandaSpiHandle)
  fake = FakeSpiV2(nack_first_data=True)

  with pytest.raises(PandaSpiNackResponse):
    handle._transfer_spidev(fake, 3, b"x", 100)
  assert handle._transfer_spidev(fake, 3, b"x", 100) == b""
  assert event_times(fake, "header")[1] - event_times(fake, "nack")[0] >= SPI_V2_TURNAROUND_NS


def encode_version_response(payload):
  packet = SPI_VERSION_REQUEST + struct.pack("<H", len(payload)) + payload
  return packet + bytes([crc8(packet)])


def test_version_stream_decoder_and_probe_stop_exactly_at_crc(monkeypatch):
  payload = version_packet(bytes(range(12)), False, 3)
  valid = encode_version_response(payload)
  bad = valid[:-1] + bytes([valid[-1] ^ 1])
  decoder = SpiVersionStreamDecoder()
  decoded = None
  stream = bytes([SPI_V3_FILLER]) * 19 + bad + bytes([SPI_V3_FILLER]) * 62 + valid
  for offset in range(0, len(stream), 13):
    decoded = decoder.feed(stream[offset:offset + 13]) or decoded
  assert decoded == payload
  assert decoder.bad_checksum == 1

  class VersionSpi:
    def __init__(self):
      self.stream = bytearray(bytes([SPI_V3_FILLER]) * 19 + valid + b"must-not-be-clocked")
      self.writes = 0
      self.reads = 0

    def xfer2(self, request):
      assert bytes(request) == SPI_VERSION_REQUEST
      self.writes += 1
      return [SPI_V3_FILLER] * len(request)

    def readbytes(self, length):
      self.reads += 1
      chunk = bytes(self.stream[:length])
      del self.stream[:length]
      return list(chunk + bytes([SPI_V3_FILLER]) * (length - len(chunk)))

  class Device:
    def __init__(self, spi):
      self.spi = spi

    @contextmanager
    def acquire(self):
      yield self.spi

  version_spi = VersionSpi()
  handle = object.__new__(PandaSpiHandle)
  handle.dev = Device(version_spi)
  quiet = []
  handle._wait_for_v2_turnaround = quiet.append
  monkeypatch.setattr(spi_module.time, "sleep", lambda delay: None)
  assert handle.get_protocol_version() == payload
  assert version_spi.writes == 1
  assert version_spi.reads == 19 + len(valid)
  assert bytes(version_spi.stream) == b"must-not-be-clocked"
  assert len(quiet) == 2


def test_version_probe_decodes_response_started_during_request(monkeypatch):
  payload = version_packet(bytes(range(12)), False, 3)
  valid = encode_version_response(payload)

  class VersionSpi:
    def __init__(self):
      self.stream = bytearray(valid[3:] + b"must-not-be-clocked")
      self.reads = 0

    def xfer2(self, request):
      assert bytes(request) == SPI_VERSION_REQUEST
      return [SPI_V3_FILLER] * 4 + list(valid[:3])

    def readbytes(self, length):
      assert length == 1
      self.reads += 1
      chunk = bytes(self.stream[:length])
      del self.stream[:length]
      return list(chunk)

  class Device:
    def __init__(self, spi):
      self.spi = spi

    @contextmanager
    def acquire(self):
      yield self.spi

  version_spi = VersionSpi()
  handle = object.__new__(PandaSpiHandle)
  handle.dev = Device(version_spi)
  quiet = []
  handle._wait_for_v2_turnaround = quiet.append
  monkeypatch.setattr(spi_module.time, "sleep", lambda delay: None)

  assert handle.get_protocol_version() == payload
  assert version_spi.reads == len(valid) - 3
  assert bytes(version_spi.stream) == b"must-not-be-clocked"
  assert len(quiet) == 2


def test_version_probe_drains_full_stale_v3_response_then_stops_at_crc(monkeypatch):
  payload = version_packet(bytes(range(12)), False, 3)
  valid = encode_version_response(payload)
  stale = spi_v3_encode_frame(SpiV3Frame(
    SpiV3FrameType.RESPONSE, SpiV3Status.OK, 0x12345678, 7, 0xAB,
    bytes([0xA5]) * SPI_V3_MAX_PAYLOAD_SIZE,
  ))
  assert len(stale) == SPI_V3_WIRE_BUFFER_SIZE
  assert SPI_VERSION_REQUEST not in stale
  assert len(valid) < SPI_VERSION_POLL_SERVICE_MARGIN_BYTES
  service_filler = bytes([SPI_V3_FILLER]) * (SPI_VERSION_POLL_SERVICE_MARGIN_BYTES - len(valid))

  class VersionSpi:
    def __init__(self):
      self.stream = bytearray(stale[len(SPI_VERSION_REQUEST):] + service_filler + valid + b"must-not-be-clocked")
      self.writes = 0
      self.reads = 0

    def xfer2(self, request):
      assert bytes(request) == SPI_VERSION_REQUEST
      self.writes += 1
      return list(stale[:len(request)])

    def readbytes(self, length):
      assert length == 1
      self.reads += 1
      chunk = bytes(self.stream[:length])
      del self.stream[:length]
      return list(chunk)

  class Device:
    def __init__(self, spi):
      self.spi = spi

    @contextmanager
    def acquire(self):
      yield self.spi

  version_spi = VersionSpi()
  handle = object.__new__(PandaSpiHandle)
  handle.dev = Device(version_spi)
  quiet = []
  handle._wait_for_v2_turnaround = quiet.append
  monkeypatch.setattr(spi_module.time, "sleep", lambda delay: None)

  assert handle.get_protocol_version() == payload
  assert version_spi.writes == 1
  assert version_spi.reads == len(stale) - len(SPI_VERSION_REQUEST) + SPI_VERSION_POLL_SERVICE_MARGIN_BYTES
  assert version_spi.reads <= SPI_VERSION_MAX_POLL_BYTES
  assert bytes(version_spi.stream) == b"must-not-be-clocked"
  assert len(quiet) == 2


def test_version_probe_stops_at_bad_crc_without_overclock(monkeypatch):
  payload = version_packet(bytes(range(12)), False, 2)
  valid = encode_version_response(payload)
  bad = valid[:-1] + bytes([valid[-1] ^ 1])

  class VersionSpi:
    def __init__(self):
      self.stream = bytearray(bad + b"must-not-be-clocked")
      self.reads = 0

    def xfer2(self, request):
      assert bytes(request) == SPI_VERSION_REQUEST
      return [SPI_V3_FILLER] * len(request)

    def readbytes(self, length):
      assert length == 1
      self.reads += 1
      chunk = bytes(self.stream[:length])
      del self.stream[:length]
      return list(chunk)

  class Device:
    @contextmanager
    def acquire(self):
      yield version_spi

  version_spi = VersionSpi()
  handle = object.__new__(PandaSpiHandle)
  handle.dev = Device()
  quiet = []
  handle._wait_for_v2_turnaround = quiet.append
  monkeypatch.setattr(spi_module.time, "sleep", lambda delay: None)

  with pytest.raises(PandaSpiBadChecksum):
    handle.get_protocol_version()
  assert version_spi.reads == len(bad)
  assert bytes(version_spi.stream) == b"must-not-be-clocked"
  assert len(quiet) == 2


def test_version_probe_has_bounded_filler_scan_and_terminal_quiet(monkeypatch):
  class MissingVersionSpi:
    def __init__(self):
      self.writes = 0
      self.read_bytes = 0

    def xfer2(self, request):
      self.writes += 1
      return [SPI_V3_FILLER] * len(request)

    def readbytes(self, length):
      self.read_bytes += length
      return [SPI_V3_FILLER] * length

  class Device:
    @contextmanager
    def acquire(self):
      yield missing

  missing = MissingVersionSpi()
  handle = object.__new__(PandaSpiHandle)
  handle.dev = Device()
  quiet = []
  handle._wait_for_v2_turnaround = quiet.append
  monkeypatch.setattr(spi_module.time, "sleep", lambda delay: None)
  with pytest.raises(PandaSpiMissingAck):
    handle.get_protocol_version()
  assert missing.writes == 1
  assert missing.read_bytes == SPI_VERSION_MAX_POLL_BYTES
  assert len(quiet) == 2


def version_packet(uid, bootstub, version, hw_type=9):
  return uid + bytes([hw_type, 0xEE if bootstub else 0xCC, version])


def test_spi_connect_switches_app_bootstub_flash_transports(monkeypatch):
  uid = bytes(range(12))

  class FakeHandle:
    SUPPORTED_PROTOCOL_VERSIONS = (2, 3)
    advertisements = iter([
      version_packet(uid, False, 2, hw_type=6),  # F4 application
      version_packet(uid, False, 3),  # H7 application
      version_packet(uid, True, 2),   # H7/F4 bootstub during flashing
      version_packet(uid, False, 3),  # H7 application after flash reset
    ])

    def __init__(self):
      self.protocol_version = 2
      self.advertisement = next(self.advertisements)

    def get_protocol_version(self):
      return self.advertisement

    def set_protocol_version(self, version):
      self.protocol_version = version

  monkeypatch.setattr(panda_python, "PandaSpiHandle", FakeHandle)
  expected = [(2, False), (3, False), (2, True), (3, False)]
  for protocol_version, expected_bootstub in expected:
    _, handle, serial, bootstub, _ = Panda.spi_connect(uid.hex())
    assert serial == uid.hex()
    assert bootstub is expected_bootstub
    assert handle.protocol_version == protocol_version


def test_spi_connect_unknown_version_lists_but_rejects_connection(monkeypatch):
  uid = bytes(range(12))

  class FakeHandle:
    SUPPORTED_PROTOCOL_VERSIONS = (2, 3)

    def __init__(self):
      self.configured = False

    def get_protocol_version(self):
      return version_packet(uid, False, 99)

    def set_protocol_version(self, version):
      self.configured = True

  monkeypatch.setattr(panda_python, "PandaSpiHandle", FakeHandle)
  with pytest.raises(PandaProtocolMismatch, match="got 99"):
    Panda.spi_connect(uid.hex())
  _, handle, serial, _, _ = Panda.spi_connect(uid.hex(), ignore_version=True)
  assert serial == uid.hex()
  assert not handle.configured


@pytest.mark.parametrize(("bootstub", "hw_type"), [(True, 9), (False, 6), (False, 0)])
def test_spi_connect_rejects_v3_for_bootstub_or_non_h7(monkeypatch, bootstub, hw_type):
  uid = bytes(range(12))

  class FakeHandle:
    SUPPORTED_PROTOCOL_VERSIONS = (2, 3)

    def __init__(self):
      self.configured = False

    def get_protocol_version(self):
      return version_packet(uid, bootstub, 3, hw_type=hw_type)

    def set_protocol_version(self, version):
      self.configured = True

  monkeypatch.setattr(panda_python, "PandaSpiHandle", FakeHandle)
  with pytest.raises(PandaProtocolMismatch, match="protocol mismatch"):
    Panda.spi_connect(uid.hex())
  _, handle, serial, _, _ = Panda.spi_connect(uid.hex(), ignore_version=True)
  assert serial == uid.hex()
  assert not handle.configured


def test_spi_version_probe_failure_keeps_v2_fallback_for_listing(monkeypatch):
  uid = bytes(range(12))

  class FakeHandle:
    SUPPORTED_PROTOCOL_VERSIONS = (2, 3)

    def __init__(self):
      self.protocol_version = 2
      self.requests = []

    def get_protocol_version(self):
      raise PandaSpiException

    def controlRead(self, request_type, request, value, index, length, timeout=0):
      self.requests.append((self.protocol_version, request))
      if request == 0xC3:
        return uid
      if request == 0xB0:
        return b"\0" * 4 + b"\xde\xad\xd0\x0d" + b"\0" * 4
      raise AssertionError(request)

    def set_protocol_version(self, version):
      raise AssertionError("unknown fallback protocol must not be selected")

  monkeypatch.setattr(panda_python, "PandaSpiHandle", FakeHandle)
  _, handle, serial, bootstub, _ = Panda.spi_connect(uid.hex(), ignore_version=True)
  assert serial == uid.hex()
  assert bootstub
  assert handle.protocol_version == 2
  assert handle.requests == [(2, 0xC3), (2, 0xB0)]


def test_v3_transport_does_not_apply_v2_quiet_time():
  handle = make_v3_handle()
  handle._wait_for_v2_turnaround = lambda start_ns: pytest.fail("v2 guard used by v3")
  assert handle._transfer_v3_spidev(FakeSpiV3(), 3, b"batch", 100, 0, False) == b""


def test_kern_v2_ioctl_is_explicitly_bypassed_for_guard(monkeypatch):
  class Device:
    pass

  monkeypatch.setenv("KERN", "1")
  monkeypatch.setattr(spi_module, "SpiDevice", Device)
  handle = PandaSpiHandle()
  assert handle._transfer_v2_raw.__func__ is PandaSpiHandle._transfer_spidev


def test_spi_v3_wire_size_constants_are_consistent():
  assert SPI_V3_HEADER_SIZE + SPI_V3_MAX_PAYLOAD_SIZE + SPI_V3_TRAILER_SIZE == SPI_V3_WIRE_BUFFER_SIZE
