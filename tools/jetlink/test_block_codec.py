"""Real system codec, strict negotiation, bounded malformed-frame rejection."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).resolve().parents[2] / 'third_party/jetlink')]
from jetlink import block_codec as B, packed_codec as M, protocol as P
from jetlink.client import JetlinkClient
from jetlink.server.session import Session, EngineHost
from test_resaction import SPEC, inputs, advanced
from test_v3 import MemoryStream, packet


@pytest.mark.skipif(B.library() is None, reason='System liblz4 unavailable')
def test_native_codec_exact_bytes_and_bounded_failures():
  raw = bytes(range(256)) * 1793
  block = B.encode((raw[:393216], memoryview(raw)[393216:]))
  assert block is not None and B.decode(block, len(raw)) == raw
  for bad, size in ((block[:-1], len(raw)), (block, len(raw) - 1), (block, len(raw) + 1),
                    (b'', len(raw)), (b'\xff', len(raw)), (block, B.MAX_BYTES + 1)):
    with pytest.raises(ValueError):
      B.decode(bad, size)
  noise = np.random.default_rng(30).integers(0, 256, SPEC.warped_nbytes, dtype=np.uint8)
  assert B.encode((noise,)) is None


def test_missing_library_keeps_raw_protocol(monkeypatch):
  monkeypatch.setattr(B, 'library', lambda: None)
  stream = MemoryStream(incoming=packet(2, P.Msg.HELLO_RESP, 1,
                        json.dumps({'protocol': 2, 'stateful_v2': True, B.CAPABILITY: True}).encode()))
  client = JetlinkClient(stream, allow_block_compression=True)
  client.hello()
  assert not client._lz4
  assert B.encode((b'hello',)) is None
  with pytest.raises(ValueError):
    B.decode(b'hello', 5)


@pytest.mark.parametrize('opt_in', [False, True])
@pytest.mark.parametrize('peer_support', [False, True])
def test_full_block_compression_requires_local_opt_in_and_peer_support(monkeypatch, opt_in, peer_support):
  monkeypatch.setattr(B, 'library', lambda: object())
  response = {'protocol': 2, 'stateful_v2': True, B.CAPABILITY: peer_support, M.CAPABILITY: True}
  wire = MemoryStream(incoming=packet(2, P.Msg.HELLO_RESP, 1, json.dumps(response).encode()))
  client = JetlinkClient(wire, **({'allow_block_compression': True} if opt_in else {}))
  client.hello()
  assert client._lz4 == (opt_in and peer_support)
  assert client._nav_u8


@pytest.mark.skipif(B.library() is None, reason='System liblz4 unavailable')
@pytest.mark.parametrize('compact_map', [False, True])
def test_real_client_server_roundtrip_and_corrupt_frame_stops_before_inference(compact_map):
  response = {'protocol': 2, 'stateful_v2': True, B.CAPABILITY: True, M.CAPABILITY: compact_map}
  wire = MemoryStream(incoming=packet(2, P.Msg.HELLO_RESP, 1, json.dumps(response).encode()))
  client = JetlinkClient(wire, allow_block_compression=True)
  client.hello()
  client.spec = SPEC
  wire.out.clear()
  images = np.arange(SPEC.warped_nbytes, dtype=np.uint8).reshape(SPEC.warped_shape)
  context = np.zeros(SPEC.packed_nelem, np.float32)
  context[12:12 + 65536] = np.arange(65536, dtype=np.uint8)
  context[-47:] = .125
  client.infer_begin(images, context, frame_id=23, reset=True)
  msg = MemoryStream(incoming=wire.out).recv(1)
  fid, flags = P.unpack_infer_req(msg.payload)
  assert fid == 23 and flags & B.FLAG and bool(flags & M.FLAG) == compact_map
  dest = inputs()
  engine = NS(inputs={n: NS(shape=s) for n, s in SPEC.input_shapes.items()},
              outputs={n: NS(shape=s) for n, s in SPEC.output_shapes.items()},
              host_input=dest.__getitem__, warm=lambda: 'warm', last_gpu_us=1)
  seen = []
  def run():
    seen.append({n: a.copy() for n, a in dest.items()})
    return advanced(dest, 2)
  engine.run = run
  loaded = EngineHost._warm(None, engine, SPEC)
  sent = []
  transport = NS(protocol_version=2, send=lambda kind, seq, parts, flags: sent.append(b''.join(parts)))
  session = Session(transport, NS(telemetry=NS(read=dict), frame_stats=NS(record=lambda *a: None)))
  session._infer(loaded, msg)
  assert P.unpack_infer_resp(memoryview(sent[-1]))[:2] == (23, P.Status.OK)
  np.testing.assert_array_equal(seen[0]['new_img'], images)
  np.testing.assert_array_equal(seen[0]['nav_map'].ravel(), context[12:12 + 65536])
  np.testing.assert_array_equal(seen[0]['navigation'].ravel(), context[-47:])
  assert all(not np.any(seen[0][n]) for n in SPEC.state_pairs)
  session._infer(loaded, NS(seq=99, payload=msg.payload[:-1]))
  assert len(seen) == 1
  assert P.unpack_infer_resp(memoryview(sent[-1]))[1] == P.Status.BAD_SHAPE
  # Corrupt reset-marked input cannot modify already accepted history.
  assert all(np.all(a == 2) for a in loaded.queues.state.values())
