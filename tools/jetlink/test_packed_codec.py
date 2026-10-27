"""Lossless navigation payloads, explicit negotiation and bounded decoding."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).resolve().parents[2] / 'third_party/jetlink')]
from jetlink import protocol as P
from jetlink.client import JetlinkClient
from jetlink.packed_codec import CAPABILITY, FLAG, encode, decode
from jetlink.server.session import Session, EngineHost
from test_resaction import SPEC, inputs, advanced
from test_v3 import MemoryStream, packet


@pytest.mark.parametrize('capability', [None, False, 1, True])
def test_client_requires_explicit_capability_and_keeps_float_bits(capability):
  hello = {'protocol': 2, 'stateful_v2': True}
  if capability is not None:
    hello[CAPABILITY] = capability
  stream = MemoryStream(incoming=packet(2, P.Msg.HELLO_RESP, 1, json.dumps(hello).encode()))
  client = JetlinkClient(stream)
  client.hello()
  client.spec = SPEC
  stream.out.clear()
  packed = np.zeros(SPEC.packed_nelem, np.float32)
  # Keep fractional values, negative zero and NaN payloads bit-for-bit. The
  # existing frame validity checks, not this codec, reject invalid model input.
  packed[:4].view(np.uint32)[:] = [0x3eaaaaab, 0x80000000, 0x7fc00042, 0xff800000]
  client.infer_begin(np.zeros(SPEC.warped_shape, np.uint8), packed, 42, reset=True)
  message = MemoryStream(incoming=stream.out).recv(1)
  fid, flags = P.unpack_infer_req(message.payload)
  assert fid == 42 and flags & P.Flag.RESET_QUEUES
  context = message.payload[P.INFER_REQ_SIZE + SPEC.warped_nbytes:]
  assert bool(flags & FLAG) == (capability is True)
  restored = decode(context, SPEC) if flags & FLAG else bytes(context)
  assert restored == packed.tobytes()
  if capability is True:
    assert len(stream.out) < 512 * 1024


@pytest.mark.parametrize('value', [.25, -0., -1., 256., np.nan, np.inf])
def test_non_integer_or_noncanonical_map_keeps_original_float_bits(value):
  raw = np.zeros(SPEC.packed_nelem, np.float32)
  raw[12] = value
  result, compressed = encode(raw, SPEC)
  assert not compressed and bytes(result) == raw.tobytes()


@pytest.mark.parametrize('difference', [-1, 1, 65536])
def test_decoder_rejects_wrong_size(difference):
  payload = bytes(SPEC.packed_nbytes - 3 * 65536 + difference)
  with pytest.raises(ValueError):
    decode(payload, SPEC)


def test_new_hello_does_not_retain_previous_host_capability():
  capable = {'protocol': 2, 'stateful_v2': True, CAPABILITY: True}
  legacy = {'protocol': 2, 'stateful_v2': True}
  stream = MemoryStream(incoming=packet(2, P.Msg.HELLO_RESP, 1, json.dumps(capable).encode()) +
                        packet(2, P.Msg.HELLO_RESP, 2, json.dumps(legacy).encode()))
  client = JetlinkClient(stream)
  client.hello()
  assert client._nav_u8
  client.hello()
  assert client._stateful_v2 and not client._nav_u8


def test_server_runs_same_inputs_and_state_for_compressed_frame():
  dest = inputs()
  engine = NS(inputs={n: NS(shape=s) for n,s in SPEC.input_shapes.items()},
              outputs={n: NS(shape=s) for n,s in SPEC.output_shapes.items()},
              host_input=dest.__getitem__, warm=lambda: 'warm', last_gpu_us=1)
  seen = []
  def run():
    seen.append({n: v.copy() for n,v in dest.items()})
    return advanced(dest, 2)
  engine.run = run
  loaded = EngineHost._warm(None, engine, SPEC)
  sent = []
  transport = NS(protocol_version=2, send=lambda kind, seq, parts, flags: sent.append(b''.join(memoryview(p).cast('B') for p in parts)))
  host = NS(telemetry=NS(read=dict), frame_stats=NS(record=lambda *a: None))
  from host_reader import ReadAheadTransport
  wrapper = object.__new__(ReadAheadTransport)
  wrapper.transport = transport
  session = Session(wrapper, host)
  packed = np.arange(SPEC.packed_nelem, dtype=np.float32) / 3
  packed[12:12 + 65536] = np.arange(65536, dtype=np.uint8)
  compressed, used = encode(packed, SPEC)
  assert used
  images = np.arange(SPEC.warped_nbytes, dtype=np.uint8).reshape(SPEC.warped_shape)
  for i, context in enumerate((packed.tobytes(), compressed)):
    flags = P.Flag.RESET_QUEUES | (FLAG if i else 0)
    payload = P.pack_infer_req(i, flags) + images.tobytes() + context
    session._infer(loaded, NS(seq=i, payload=memoryview(payload)))
    assert P.unpack_infer_resp(memoryview(sent[-1]))[:2] == (i, P.Status.OK)
  assert seen[0].keys() == seen[1].keys()
  for name in seen[0]:
    np.testing.assert_array_equal(seen[0][name], seen[1][name])
  payload = P.pack_infer_req(3, FLAG) + images.tobytes() + compressed[:-1]
  session._infer(loaded, NS(seq=3, payload=memoryview(payload)))
  assert P.unpack_infer_resp(memoryview(sent[-1]))[1] == P.Status.BAD_SHAPE
  assert len(seen) == 2
