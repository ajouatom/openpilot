"""USB gadget owner and Jetlink client, isolated from modeld's frame deadline."""
import fcntl
import gc
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
from openpilot.selfdrive.modeld.jetlink.link import (SPEC, SOCKET, STATUS, REQUEST, REPLY, PacketReader, send, send_parts)
from openpilot.selfdrive.modeld.jetlink.phase import Publisher as PhasePublisher
from openpilot.selfdrive.modeld.jetlink.mac import prepare, PreparationDeferred
from jetlink.client import JetlinkClient
from jetlink.transport.ffs import FfsTransport

GADGET = '/sys/kernel/config/usb_gadget/jetlink'
ROLE = Path('/sys/class/power_supply/usb/typec_mode')
log = logging.getLogger('carrot.jetlink')
sys.path.insert(0, str(VENDOR.parents[1] / 'tools/jetlink'))
from hud_protocol import Publisher, HUD_CAPABILITY, HUD_MESSAGE
from hud_navi import CAPABILITY as NAVI_CAPABILITY, MESSAGE as NAVI_MESSAGE
from hud_navi import send_ready_after_reply, PUMP_CAPABILITY
from wifi_protocol import CAPABILITY as WIFI_CAPABILITY, Publisher as WifiPublisher


def update_affinity():
  from openpilot.common.params import Params
  onroad = Params().get_bool('IsOnroad') and Path('/sys/devices/system/cpu/cpu7/online').read_text().strip() == '1'
  # Keep USB completion and IPC work off modeld/DM's core7. Pinning the
  # reader there delayed runnable reads behind DM and blocked the host's
  # response write. Include the normal-priority watchdog in this placement.
  # Model/DM policies and the display children's separate policy are unchanged.
  cores = set(range(4))
  os.sched_setscheduler(0, os.SCHED_FIFO if onroad else os.SCHED_OTHER, os.sched_param(1 if onroad else 0))
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
    # At most one small fragment per inference window. No video
    # decoding or unbounded stream draining occurs on the USB owner's thread.
    media = publisher.media_packet()
    if media:
      client.t.send(NAVI_MESSAGE, client._next_seq(), [media])


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
    # The short USB receive/copy worker is realtime, below modeld/DM.
    # Display workers remain normal priority. Existing camera/control/sensor
    # placements and priorities are untouched.
    os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(1))


def serve_local(listener, client, peer, wifi=None):
  publisher = Publisher(navi=bool(peer.get(NAVI_CAPABILITY))) if peer.get(HUD_CAPABILITY) else None
  phase = PhasePublisher()
  try:
    _serve_local(listener, client, peer, publisher, phase, wifi)
  finally:
    phase.close()
    if publisher is not None:
      publisher.close()


def _serve_local(listener, client, peer, publisher, phase, wifi=None):
  from openpilot.common.params import Params
  from openpilot.common.runtime_diagnostics import RuntimeDiagnostics
  from openpilot.common.swaglog import cloudlog
  params = Params()
  diagnostics = RuntimeDiagnostics('jetlinkd', cloudlog.event)
  last_status = 0.
  last_ping = time.monotonic()
  telemetry_updated = 0.
  while host_attached():
    if time.monotonic() - last_status >= 1:
      update_affinity()
      publish('ready', peer=peer, telemetry=client.last_state, telemetry_updated=telemetry_updated)
      last_status = time.monotonic()
    try:
      connection, _ = listener.accept()
    except TimeoutError:
      if wifi is not None:
        wifi.send(client)
      publish_hud(client, publisher)
      if time.monotonic() - last_ping > 2:
        client.last_state = client.state()
        telemetry_updated = time.monotonic()
        last_ping = time.monotonic()
      continue
    with connection:
      connection.settimeout(.5)
      reader = PacketReader(REQUEST.size + SPEC.warped_nbytes + SPEC.packed_nbytes)
      try:
        send(connection, json.dumps({'spec': SPEC.to_dict(), 'peer': peer}).encode())
        while host_attached():
          loop_started, cpu_started = time.monotonic(), time.thread_time()
          if time.monotonic() - last_status >= 1:
            update_affinity()
            if publisher is not None:
              params.put_bool_nonblocking('ClusterHudConnected', bool((client.last_state or {}).get('carrot_hud_connected')))
            publish('ready', peer=peer, timings=list(client.last_timings),
                    telemetry=client.last_state, telemetry_updated=telemetry_updated,
                    navigation_tail=getattr(publisher, 'tail_stats', {}))
            last_status = time.monotonic()
          receive_started = time.monotonic()
          try:
            request = reader.receive(connection)
          except TimeoutError:
            if wifi is not None:
              wifi.send(client)
            publish_hud(client, publisher)
            client.last_state = client.state()
            telemetry_updated = time.monotonic()
            continue
          received = time.monotonic()
          if len(request) != REQUEST.size + SPEC.warped_nbytes + SPEC.packed_nbytes:
            raise ValueError('invalid local inference request')
          frame, reset, source_sof = REQUEST.unpack_from(request)
          if reset not in (0, 1):
            raise ValueError('invalid reset flag')
          images = np.frombuffer(request, np.uint8, SPEC.warped_nbytes, REQUEST.size).reshape(SPEC.warped_shape)
          packed = np.frombuffer(request, np.float32, SPEC.packed_nelem, REQUEST.size + SPEC.warped_nbytes)
          if not np.all(np.isfinite(packed)):
            raise ValueError('invalid local model context')
          started = time.monotonic()
          seq = client.infer_begin(images, packed, frame, reset=bool(reset), want_state=(frame % 20 == 0))
          sent = time.monotonic()
          if peer.get('carrot_host') == 'jetson':
            phase.sent(source_sof)
          phase_done = time.monotonic()
          if peer.get(NAVI_CAPABILITY):
            publish_hud(client, publisher)
          hud_done = time.monotonic()
          previous_state = client.last_state
          output = client.infer_end(seq)
          if client.last_state is not previous_state:
            telemetry_updated = time.monotonic()
          completed = time.monotonic()
          try:
            send_parts(connection, REPLY.pack(frame, *client.last_timings), output)
          finally:
            # Preserve the send/response split even when modeld already timed
            # out and the local reply raises BrokenPipeError.
            if completed - started > .05:
              log.warning('USB frame %d: send %.1f response %.1f IPC %.1f ms', frame,
                          (sent-started)*1000, (completed-sent)*1000, (time.monotonic()-completed)*1000)
          replied = time.monotonic()
          if not peer.get(NAVI_CAPABILITY):
            publish_hud(client, publisher)
          else:
            # modeld already has its reply. Drain only a bounded ready tail;
            # never add these fragments before infer_end or delay its reply.
            send_ready_after_reply(client, publisher, fast_receiver=peer.get(PUMP_CAPABILITY) is True)
          tail_done = time.monotonic()
          if wifi is not None:
            wifi.send(client)
          finished = time.monotonic()
          # Record in rlog as well as stderr. Receive time includes normal idle
          # waiting; the reply tail may delay admission of the NEXT request.
          diagnostics.record(
            context={'frame_id': frame, 'usb_seq': seq},
            housekeeping_ms=(receive_started-loop_started)*1000,
            ipc_receive_ms=(received-receive_started)*1000,
            request_parse_ms=(started-received)*1000,
            usb_send_ms=(sent-started)*1000,
            phase_ms=(phase_done-sent)*1000,
            hud_ms=(hud_done-phase_done)*1000,
            usb_response_ms=(completed-hud_done)*1000,
            ipc_reply_ms=(replied-completed)*1000,
            display_tail_ms=(tail_done-replied)*1000,
            wifi_tail_ms=(finished-tail_done)*1000,
            server_gpu_ms=client.last_timings[0]/1000,
            server_queue_ms=client.last_timings[1]/1000,
            server_total_ms=client.last_timings[2]/1000,
            loop_ms=(finished-loop_started)*1000,
            thread_cpu_ms=(time.thread_time()-cpu_started)*1000,
          )
      except (ConnectionError, BrokenPipeError, ValueError) as exc:
        log.info('local client ended: %s', exc)
    last_ping = time.monotonic()


def main():
  logging.basicConfig(level=logging.INFO)
  # Like modeld, do not let cyclic GC scan the manager's inherited object graph
  # while an inference reply is due. Collect between USB sessions instead.
  gc.disable()
  # The manager forks us with its large, long-lived Python object graph.
  # Session cleanup must not dirty all those shared pages on the first plug-in.
  # Freeze only once, before creating any USB session; new session cycles still
  # participate in the existing between-session collections below.
  gc.freeze()
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
  peer = None
  try:
    while True:
      update_affinity()
      if not host_attached():
        publish('waiting', peer=peer)
        time.sleep(1)
        continue
      client = None
      wifi = None
      try:
        publish('connecting', peer=peer)
        gc.collect()
        subprocess.run(['sudo', '-n', 'bash', str(setup)], check=True, timeout=15, capture_output=True)
        udc = next(Path('/sys/class/udc').iterdir()).name
        client = JetlinkClient(CarrotTransport('/dev/ffs-jetlink', gadget=GADGET, udc=udc), name='carrot-jetlink')
        peer = client.hello()
        speed = (Path('/sys/class/udc') / udc / 'current_speed').read_text().strip()
        if speed not in ('super-speed', 'super-speed-plus'):
          raise RuntimeError(f'USB 5Gbps or faster required; negotiated {speed}')
        if peer.get(WIFI_CAPABILITY) is True:
          wifi = WifiPublisher()
          # Provision before ensure_engine: a new host may need Internet to
          # fetch its first model. This is outside every model frame deadline.
          wifi.send(client, initial=True)
        publish('loading', peer=peer)
        from openpilot.common.params import Params
        params = Params()
        last_progress = 0.

        def progress(stage, fraction, message):
          nonlocal last_progress
          if time.monotonic() - last_progress >= 1:
            publish('loading', peer=peer, preparation={'stage': stage, 'fraction': fraction, 'message': str(message)[:160]})
            last_progress = time.monotonic()

        prepare(client, peer, lambda: params.get_bool('IsOffroad') and not params.get_bool('IsOnroad'),
                host_attached, progress)
        # Warm independently of camera/modeld; every real session resets state.
        for frame in range(10):
          client.infer(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), frame, reset=True)
        log.info('Jetlink ready: %s', peer)
        serve_local(listener, client, peer, wifi)
      except PreparationDeferred as exc:
        log.info('%s', exc)
        publish('loading', peer=peer, preparation={'stage': 'waiting', 'message': str(exc)})
      except Exception as exc:
        log.exception('Jetlink connection failed')
        publish('retrying', peer=peer, error=str(exc)[:300])
      finally:
        if wifi is not None:
          wifi.close()
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
