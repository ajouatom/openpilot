"""Session-local v2/v3 interoperability, retaining Carrot's full output contract.

v3 keeps recurrent features on the server. Request WANT_HIDDEN to retain real
hidden outputs for our existing parser/raw diagnostics, rather than fabricating
zeros or changing modeld's IPC. This intentionally does not take the compact
reply optimization. Reference: zoompilot/jetlink 24e8917673829cbe1d146735956e9038cc93f227.
"""
import json
import os
import time

import numpy as np

from openpilot.selfdrive.modeld.jetlink.link import SPEC, validate_spec
from jetlink import protocol as V2, protocol_v3 as V3
from jetlink.client import JetlinkClient, _as_bytes
from jetlink.transport.base import LinkError


class ProtocolAttempts:
  """Try v2 first; retry the other version only after a failed HELLO.

  There is no in-band negotiation upstream: mismatched headers close the link.
  The caller must close/reopen the transport before trying the next version.
  Model/upload/inference failures never change an established wire version.
  """
  def __init__(self):
    self.reset()

  def reset(self):
    self.version = 2

  def hello(self, transport):
    version = self.version
    transport.wire_protocol = V2 if version == 2 else V3
    transport.protocol_version = version
    client = (JetlinkClient if version == 2 else V3Client)(transport, name='carrot-jetlink')
    try:
      peer = client.hello()
      if not isinstance(peer, dict) or type(peer.get('protocol')) is not int or peer['protocol'] != version:
        raise LinkError('HELLO protocol does not match the session header')
    except Exception:
      # In particular, no receive timeout or invalid HELLO permits treating
      # the peer as ready. Reopen before attempting the other strict version.
      self.version = 5 - version
      client.close()
      raise
    return client, peer


class V3Client(JetlinkClient):
  def infer_begin(self, warped, packed, frame_id=0, reset=False, want_state=False, deadline=None):
    if self.spec is None:
      raise LinkError('ensure_engine() first')
    if self.dead:
      raise LinkError('link previously failed')
    # This adapter only bridges the existing, exact Cinque v2 model contract.
    # Protocol v3 does not imply a promotion to the Cinque v3 driving model.
    validate_spec(self.spec)
    images = _as_bytes(warped, SPEC.warped_nbytes, 'warped')
    context = _as_bytes(packed, SPEC.packed_nbytes, 'packed')
    # The pinned packed layout is desire[8], traffic[2], action_t[2], prev_feat.
    # The server owns prev_feat in v3, including resets and finite-only feedback.
    scalars = context[:48]
    if not np.isfinite(np.frombuffer(scalars, dtype=np.float32)).all():
      raise LinkError('non-finite v3 scalar input')
    flags = V3.Flag.WANT_HIDDEN | (V3.Flag.RESET_QUEUES if reset else 0) | (V3.Flag.WANT_STATE if want_state else 0)
    seq = self._next_seq()
    self._want_state = want_state
    self._infer_frame_id = frame_id
    self._infer_started = time.monotonic()
    try:
      self.t.send(V3.Msg.INFER_REQ, seq, (V3.pack_infer_req(frame_id, flags), images, scalars),
                  timeout=self.deadline if deadline is None else deadline)
    except LinkError:
      self.dead = True
      raise
    return seq

  def infer_end(self, seq, deadline=None):
    try:
      remaining = (self.deadline if deadline is None else deadline) - (time.monotonic() - self._infer_started)
      if remaining <= 0:
        raise LinkError('frame deadline elapsed during send')
      msg = self._expect(V3.Msg.INFER_RESP, seq, remaining)
      if msg.payload.nbytes < V3.INFER_RESP_SIZE:
        raise LinkError('inference response is missing its header')
      frame, status, gpu_us, queue_us, total_us = V3.unpack_infer_resp(msg.payload)
      if frame != self._infer_frame_id or status != V3.Status.OK:
        raise LinkError(f'invalid v3 inference response: frame={frame}, status={status}')
      end = V3.INFER_RESP_SIZE + SPEC.output_nbytes
      if msg.payload.nbytes < end:
        raise LinkError('v3 response is missing the requested full model outputs')
      telemetry = None
      if msg.payload.nbytes > end:
        if not self._want_state:
          raise LinkError('unexpected bytes after v3 model outputs')
        telemetry = json.loads(bytes(msg.payload[end:]))
        if not isinstance(telemetry, dict):
          raise LinkError('invalid v3 telemetry')
      output = np.frombuffer(msg.payload, np.float32, SPEC.output_nelem, V3.INFER_RESP_SIZE).copy()
      if not np.isfinite(output).all():
        raise LinkError('non-finite v3 model output')
      self.last_timings = gpu_us, queue_us, total_us
      if telemetry is not None:
        self.last_state = telemetry
      return output
    except (LinkError, ValueError) as exc:
      self.dead = True
      raise LinkError(f'v3 inference failed; link abandoned: {exc}') from exc


class ProtocolChoice:
  def __init__(self, setting=None):
    setting = os.environ.get('JETLINK_PROTOCOL', 'auto') if setting is None else setting
    if setting not in ('auto', '2', '3'):
      raise ValueError('JETLINK_PROTOCOL must be auto, 2 or 3')
    self.automatic = setting == 'auto'
    self.version = 3 if self.automatic else int(setting)

  def handshake_failed(self):
    if self.automatic:
      self.version = 2 if self.version == 3 else 3
