"""USB gadget owner and Jetlink client, isolated from modeld's frame deadline."""
import fcntl
import json
import logging
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

import numpy as np

from openpilot.selfdrive.modeld.jetlink import VENDOR
from openpilot.selfdrive.modeld.jetlink.link import (SPEC, SOCKET, STATUS, REQUEST, REPLY, receive, send, validate_spec)
from jetlink.client import JetlinkClient
from jetlink.transport.ffs import FfsTransport

GADGET = '/sys/kernel/config/usb_gadget/jetlink'
ROLE = Path('/sys/class/power_supply/usb/typec_mode')
log = logging.getLogger('carrot.jetlink')
sys.path.insert(0, str(VENDOR.parents[1] / 'tools/jetlink'))
from hud_protocol import Publisher, HUD_CAPABILITY, HUD_MESSAGE


def update_affinity():
  from openpilot.common.params import Params
  cores = {7} if Params().get_bool('IsOnroad') and Path('/sys/devices/system/cpu/cpu7/online').read_text().strip() == '1' else set(range(4))
  for thread in Path('/proc/self/task').iterdir():
    try:
      os.sched_setaffinity(int(thread.name), cores)
    except ProcessLookupError:
      pass


def publish_hud(client, publisher):
  if publisher is not None:
    try:
      packet = publisher.packet()
    except Exception:
      log.exception('display snapshot unavailable')
      return
    if packet:
      client.t.send(HUD_MESSAGE, client._next_seq(), [packet])


def host_attached():
  try:
    # CC orientation alone also detects eGPU/hub cables. Require the peer to
    # be a source/host: never seize the existing eGPU's USB role.
    return ROLE.read_text().strip().startswith('Source attached')
  except OSError:
    return False


def publish(status, **extra):
  record = dict(state=status, updated=time.monotonic(), model='Cinque v2', sha256=SPEC.sha256, **extra)
  temporary = STATUS.with_suffix('.tmp')
  temporary.write_text(json.dumps(record))
  os.replace(temporary, STATUS)


class CarrotTransport(FfsTransport):
  def _widen_affinity(self):
    os.sched_setaffinity(0, set(range(min(4, os.cpu_count() or 1))))

  def _raise_reader_priority(self):
    # Preserve the Carrot camera/control/sensor scheduling contract. Upstream
    # raises this thread to FIFO51 on all cores; that is not our placement.
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))


def serve_local(listener, client, peer):
  publisher = Publisher() if peer.get(HUD_CAPABILITY) else None
  try:
    _serve_local(listener, client, peer, publisher)
  finally:
    if publisher is not None:
      publisher.close()


def _serve_local(listener, client, peer, publisher):
  from openpilot.common.params import Params
  params = Params()
  last_status = 0.
  last_ping = time.monotonic()
  while host_attached():
    if time.monotonic() - last_status >= 1:
      update_affinity()
      publish('ready', peer=peer)
      last_status = time.monotonic()
    try:
      connection, _ = listener.accept()
    except TimeoutError:
      publish_hud(client, publisher)
      if time.monotonic() - last_ping > 2:
        client.ping()
        last_ping = time.monotonic()
      continue
    with connection:
      connection.settimeout(.5)
      try:
        send(connection, json.dumps({'spec': SPEC.to_dict(), 'peer': peer}).encode())
        while host_attached():
          if time.monotonic() - last_status >= 1:
            update_affinity()
            if publisher is not None:
              params.put_bool_nonblocking('ClusterHudConnected', bool((client.last_state or {}).get('carrot_hud_connected')))
            publish('ready', peer=peer, timings=list(client.last_timings))
            last_status = time.monotonic()
          try:
            request = receive(connection)
          except TimeoutError:
            client.ping()
            continue
          if len(request) != REQUEST.size + SPEC.warped_nbytes + SPEC.packed_nbytes:
            raise ValueError('invalid local inference request')
          frame, reset = REQUEST.unpack_from(request)
          if reset not in (0, 1):
            raise ValueError('invalid reset flag')
          images = np.frombuffer(request, np.uint8, SPEC.warped_nbytes, REQUEST.size).reshape(SPEC.warped_shape)
          packed = np.frombuffer(request, np.float32, SPEC.packed_nelem, REQUEST.size + SPEC.warped_nbytes)
          if not np.all(np.isfinite(packed)):
            raise ValueError('invalid local model context')
          output = client.infer(images, packed, frame, reset=bool(reset), want_state=(frame % 20 == 0))
          send(connection, REPLY.pack(frame, *client.last_timings) + output.tobytes())
          publish_hud(client, publisher)
      except (ConnectionError, BrokenPipeError, ValueError) as exc:
        log.info('local client ended: %s', exc)
    last_ping = time.monotonic()


def main():
  logging.basicConfig(level=logging.INFO)
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  os.sched_setaffinity(0, set(range(min(4, os.cpu_count() or 1))))
  lock = open('/dev/shm/carrot-jetlink.lock', 'w')
  fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  Path(SOCKET).unlink(missing_ok=True)
  listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  listener.bind(SOCKET)
  os.chmod(SOCKET, 0o600)
  listener.listen(1)
  listener.settimeout(.05)
  setup = VENDOR.parents[1] / 'tools/jetlink/setup_gadget.sh'
  try:
    while True:
      if not host_attached():
        publish('waiting')
        time.sleep(1)
        continue
      client = None
      try:
        publish('connecting')
        subprocess.run(['sudo', '-n', 'bash', str(setup)], check=True, timeout=15, capture_output=True)
        udc = next(Path('/sys/class/udc').iterdir()).name
        client = JetlinkClient(CarrotTransport('/dev/ffs-jetlink', gadget=GADGET, udc=udc), name='carrot-jetlink')
        peer = client.hello()
        speed = (Path('/sys/class/udc') / udc / 'current_speed').read_text().strip()
        if speed not in ('super-speed', 'super-speed-plus'):
          raise RuntimeError(f'USB 5Gbps or faster required; negotiated {speed}')
        publish('loading', peer=peer)
        validate_spec(client.ensure_engine(SPEC.sha256, SPEC.nbytes, frame_skip=SPEC.frame_skip, build_timeout=30))
        # Warm independently of camera/modeld; every real session resets state.
        for frame in range(10):
          client.infer(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), frame, reset=True)
        log.info('Jetlink ready: %s', peer)
        serve_local(listener, client, peer)
      except Exception as exc:
        log.exception('Jetlink connection failed')
        publish('retrying', error=str(exc)[:300])
      finally:
        if client is not None and client.last_state is not None and 'carrot_hud_connected' in client.last_state:
          from openpilot.common.params import Params
          Params().put_bool_nonblocking('ClusterHudConnected', False)
        if client is not None:
          try:
            client.close()
          except Exception:
            log.exception('Jetlink close failed')
      time.sleep(2)
  finally:
    listener.close()
    Path(SOCKET).unlink(missing_ok=True)
    publish('stopped')


if __name__ == '__main__':
  main()
