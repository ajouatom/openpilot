"""Verified, offroad provisioning for compatible upstream Mac and phone apps."""
import hashlib
import os
from pathlib import Path
import re
import shutil
import tempfile
import time
import urllib.request

from openpilot.common.jetlink_peer import may_provision
from openpilot.selfdrive.modeld.jetlink.link import SPEC, validate_spec
from openpilot.selfdrive.modeld.jetlink.contracts import validate_model_spec
from openpilot.selfdrive.modeld.jetlink.selection import ModelChanged, approved_spec, remember_spec
from jetlink.client import EngineMissing
from jetlink.transport.base import LinkTimeout

MODEL_URL = 'https://upload.shind0.synology.me/models/carrot-jetlink-cinque-v2/big_driving_supercombo.onnx'
CACHE = Path('/data/models/carrot-jetlink-mac')


class PreparationDeferred(RuntimeError):
  pass


def require_setup(offroad, connected):
  if not connected():
    raise PreparationDeferred('Jetlink host disconnected; waiting for reconnection')
  if not offroad():
    raise PreparationDeferred('Jetlink model setup requires ignition off')


def model_file(offroad, connected, progress, cache=CACHE):
  require_setup(offroad, connected)
  deadline = time.monotonic() + 1800
  cache.mkdir(parents=True, exist_ok=True)
  target = cache / (SPEC.sha256 + '.onnx')
  if target.is_file() and target.stat().st_size == SPEC.nbytes:
    value = hashlib.sha256()
    with target.open('rb') as stream:
      for block in iter(lambda: stream.read(4 << 20), b''):
        require_setup(offroad, connected)
        value.update(block)
        progress('verify', 0., 'Checking cached Jetlink model')
    if value.hexdigest() == SPEC.sha256:
      return target
  if shutil.disk_usage(cache).free < SPEC.nbytes + (64 << 20):
    raise OSError('Not enough free space for the Jetlink model')
  fd, name = tempfile.mkstemp(dir=cache, prefix='download-', suffix='.part')
  temporary = Path(name)
  try:
    progress('download', 0., 'Downloading Cinque v2 for Jetlink')
    with os.fdopen(fd, 'wb') as output:
      with urllib.request.urlopen(MODEL_URL, timeout=10) as response:
        if response.geturl() != MODEL_URL:
          raise ValueError('Unexpected Jetlink model download redirect')
        count, value = 0, hashlib.sha256()
        while True:
          require_setup(offroad, connected)
          if time.monotonic() >= deadline:
            raise TimeoutError('Jetlink model download exceeded 30 minutes')
          block = response.read(1 << 20)
          if not block:
            break
          count += len(block)
          if count > SPEC.nbytes:
            raise ValueError('Oversized Jetlink model download')
          output.write(block)
          value.update(block)
          progress('download', count / SPEC.nbytes, 'Downloading Cinque v2 for Jetlink')
      if count != SPEC.nbytes or value.hexdigest() != SPEC.sha256:
        raise ValueError('Jetlink model checksum/size mismatch')
      require_setup(offroad, connected)
      output.flush()
      os.fsync(output.fileno())
    os.replace(temporary, target)
    return target
  finally:
    temporary.unlink(missing_ok=True)


class PreparationTransport:
  """Poll setup cancellation while existing client waits for engine responses.

  Installed only during Mac preparation; inference keeps the original transport
  and deadlines. The underlying transport retains partially received messages.
  """
  def __init__(self, transport, check):
    self.transport, self.check = transport, check

  def __getattr__(self, name):
    return getattr(self.transport, name)

  def send(self, *args, **kwargs):
    self.check()
    return self.transport.send(*args, **kwargs)

  def send_json(self, *args, **kwargs):
    self.check()
    return self.transport.send_json(*args, **kwargs)

  def recv(self, timeout=None):
    end = None if timeout is None else time.monotonic() + timeout
    while True:
      self.check()
      remaining = 1. if end is None else min(1., end - time.monotonic())
      if remaining <= 0:
        raise LinkTimeout('Jetlink preparation response timeout')
      try:
        return self.transport.recv(timeout=remaining)
      except LinkTimeout:
        pass


def prepare(client, peer, offroad, connected, progress, cache=CACHE, mode='usb'):
  """Retain the legacy Jetson path; phones require their explicit cable mode."""
  protocol = peer.get('protocol') if isinstance(peer, dict) else None
  app_capable = protocol == 3 and may_provision(peer, mode)
  legacy_v3 = protocol == 3 and isinstance(peer, dict) and 'loaded' in peer and not app_capable
  if protocol == 3 and not app_capable and not legacy_v3:
    raise PreparationDeferred('Jetlink requires ignition off: select and prepare a driving model in the App')
  if protocol != 3 and not may_provision(peer, mode):
    if mode in ('ios', 'android'):
      raise ValueError(f'Unsupported {mode} Jetlink peer identity: {peer}')
    validate_spec(client.ensure_engine(SPEC.sha256, SPEC.nbytes, frame_skip=SPEC.frame_skip, build_timeout=30))
    return
  if protocol == 3 and mode in ('ios', 'android') and not may_provision(peer, mode):
    raise ValueError(f'Unsupported {mode} Jetlink peer identity: {peer}')

  app_selection = app_capable
  selected = peer.get('loaded') if app_selection else SPEC.sha256
  if app_selection and selected is None:
    raise PreparationDeferred('Jetlink requires ignition off: select and prepare a driving model in the App')
  if app_selection:
    if not isinstance(selected, str) or re.fullmatch('[0-9a-f]{64}', selected) is None:
      raise ValueError('Invalid Jetlink App loaded model identity')
    approved = approved_spec(cache)
    needs_approval = approved.sha256 != selected if approved is not None else selected != SPEC.sha256
    if needs_approval and not offroad():
      raise PreparationDeferred('A different Jetlink App model must be approved with ignition off')
  else:
    approved = None
    needs_approval = False

  # HELLO resets the upstream session's requested model, so engine_state is
  # normally "none" even when loaded names a resident engine. ENGINE_REQ below
  # still verifies its full spec; pending setup remains offroad-only.
  preloaded = peer.get('loaded') == selected
  if not preloaded:
    require_setup(offroad, connected)

  def check():
    if not connected():
      raise PreparationDeferred('Jetlink host disconnected; waiting for reconnection')
    if not preloaded or needs_approval:
      require_setup(offroad, connected)
    progress('prepare', None, 'Waiting for Jetlink model preparation')

  def stopped():
    # A ready response returns without building. Any pending build/upload must
    # still stop when ignition starts, including a stale preloaded HELLO.
    require_setup(offroad, connected)
    return False

  def report(stage, fraction, message):
    require_setup(offroad, connected)
    progress(stage, fraction, message)

  original = client.t
  client.t = PreparationTransport(original, check)
  try:
    if app_selection:
      current = client.state()
      if not isinstance(current, dict) or 'loaded' not in current:
        raise ValueError('Jetlink App did not report its loaded model')
      if current['loaded'] != selected:
        raise ModelChanged('Jetlink App engine changed before approval; reconnect to its current pick')
    args = {'frame_skip': SPEC.frame_skip, 'build_timeout': 900, 'should_stop': stopped, 'progress': report}
    try:
      result = client.ensure_engine(selected, SPEC.nbytes if selected == SPEC.sha256 else 0, **args)
    except EngineMissing:
      if app_selection:
        raise PreparationDeferred('Selected App engine is not ready; finish its preparation offroad') from None
      require_setup(offroad, connected)
      path = model_file(offroad, connected, progress, cache)
      require_setup(offroad, connected)
      result = client.ensure_engine(SPEC.sha256, SPEC.nbytes, onnx_path=path, **args)
    if selected == SPEC.sha256:
      validate_spec(result)
    else:
      validate_model_spec(result)
    if approved is not None and approved.sha256 == selected and approved.to_dict() != result.to_dict():
      raise ValueError('Previously approved Jetlink model contract changed')
    if needs_approval:
      require_setup(offroad, connected)
    if app_selection and offroad():
      remember_spec(cache, result)
    return result
  finally:
    client.t = original
    client.progress_cb = None
    client._should_stop = lambda: False
