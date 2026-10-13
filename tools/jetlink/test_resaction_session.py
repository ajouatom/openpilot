"""Exercise the actual pinned host session and comma client together."""
from pathlib import Path
import socket
import sys
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink')]
from jetlink.client import JetlinkClient
from jetlink.server.session import EngineHost, Session
from jetlink.transport.tcp import TcpTransport
from jetlink.transport.base import LinkError
from jetlink import protocol as P
from openpilot.selfdrive.modeld.jetlink.contracts import JETSON_SPEC as SPEC


def test_pinned_host_hello_to_stateful_inference_and_reset():
  buffers = {name: np.zeros(shape, np.float16 if name in ('new_img', 'state_img_q') else np.float32)
             for name, shape in SPEC.input_shapes.items()}
  def run():
    value = buffers['state_feat_q'].flat[0] + 1
    return {'outputs': np.full(SPEC.output_shapes['outputs'], value, np.float32),
            **{output: np.full_like(buffers[name], value) for name, output in SPEC.state_pairs.items()}}
  engine = NS(inputs={name: NS(shape=shape) for name, shape in SPEC.input_shapes.items()},
              outputs={name: NS(shape=shape) for name, shape in SPEC.output_shapes.items()},
              host_input=buffers.__getitem__, warm=lambda: 'warm', run=run, last_gpu_us=1, close=lambda: None)
  backend = NS(describe=lambda: {'backend': 'trt', 'runtime_version': '10.3.0', 'device': 'Orin-sm87'})
  host = EngineHost(NS(backend=backend, inventory=list))
  host.loaded = host._warm(engine, SPEC)
  ours, peer = socket.socketpair()
  server_transport = TcpTransport(peer)
  session = Session(server_transport, host)
  thread = threading.Thread(target=session.serve_forever, daemon=True)
  thread.start()
  client = JetlinkClient(TcpTransport(ours), deadline=3)
  try:
    peer_info = client.hello()
    assert peer_info['protocol'] == 2 and peer_info['stateful_v2'] is True
    assert client.ensure_engine(SPEC.sha256, SPEC.nbytes, frame_skip=4) == SPEC
    for frame, reset, expected in ((1, True, 1), (2, False, 2), (3, True, 1)):
      result = client.infer(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), frame, reset=reset)
      np.testing.assert_array_equal(result, np.full(SPEC.output_nelem, expected, np.float32))
  finally:
    client.close()
    thread.join(5)
    session.close()
    server_transport.close()
    host.close()
  assert not thread.is_alive()


@pytest.mark.parametrize('capability', [None, False, 1, 'true'])
def test_stateful_v2_permission_is_strict_and_revoked_on_new_hello(capability):
  from test_v3 import MemoryStream, packet
  import json
  def hello(value):
    return json.dumps({'protocol': 2, 'stateful_v2': value}).encode()
  stream = MemoryStream(2, incoming=packet(2, P.Msg.HELLO_RESP, 1, hello(True)) +
                                   packet(2, P.Msg.HELLO_RESP, 2, hello(capability)))
  client = JetlinkClient(stream)
  client.hello()
  assert client._stateful_v2
  client.hello()
  client.spec = SPEC
  before = len(stream.out)
  with pytest.raises(LinkError, match='stateful models require'):
    client.infer_begin(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32))
  assert len(stream.out) == before
