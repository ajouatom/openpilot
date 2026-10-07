"""USB gadget owner and Jetlink client, isolated from modeld's frame deadline."""
import fcntl
import gc
import json
import logging
import os
from pathlib import Path
import socket
import sys
import time

import numpy as np

from openpilot.common.jetlink_peer import classify_auto_peer, may_provision
from openpilot.selfdrive.modeld.jetlink import VENDOR
from openpilot.selfdrive.modeld.jetlink.link import (SOCKET, STATUS, REQUEST, REPLY, PacketReader, send, send_parts)
from openpilot.selfdrive.modeld.jetlink.phase import Publisher as PhasePublisher
from openpilot.selfdrive.modeld.jetlink.mac import prepare, PreparationDeferred
from openpilot.selfdrive.modeld.jetlink.compat import ProtocolChoice
from openpilot.selfdrive.modeld.jetlink.mobile import transport_mode, setup_gadget, MobileCable, AutoTransport, MobileLinkError
from openpilot.selfdrive.modeld.jetlink.contracts import model_name
from openpilot.selfdrive.modeld.jetlink.selection import ModelChanged, check_loaded
from openpilot.selfdrive.modeld.jetlink.startup import CAPABILITY as BOOT_UPDATE_CAPABILITY, wait_for_boot_update
from jetlink.client import JetlinkClient
from jetlink.transport.base import LinkError, LinkTimeout
from jetlink.transport.ffs import FfsTransport

GADGET = '/sys/kernel/config/usb_gadget/jetlink'
ROLE = Path('/sys/class/power_supply/usb/typec_mode')
DATA_ROLE = Path('/sys/class/usbpd/usbpd0/current_dr')
TRANSPORT_MODE = 'usb'
SESSION_MODE = 'usb'
ACTIVE_UDC = None
SESSION_DETACHED = False
log = logging.getLogger('carrot.jetlink')
sys.path.insert(0, str(VENDOR.parents[1] / 'tools/jetlink'))
from hud_protocol import Publisher, HUD_CAPABILITY, HUD_MESSAGE
from hud_navi import CAPABILITY as NAVI_CAPABILITY, MESSAGE as NAVI_MESSAGE
from hud_navi import send_ready_after_reply, PUMP_CAPABILITY
from wifi_protocol import CAPABILITY as WIFI_CAPABILITY, Publisher as WifiPublisher


class UnsafeCleanupError(LinkError):
  pass


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
  global SESSION_DETACHED
  attached = _role_attached()
  if attached and ACTIVE_UDC is not None:
    try:
      attached = (Path('/sys/class/udc') / ACTIVE_UDC / 'state').read_text().strip() == 'configured'
    except OSError:
      # A selected active session must not treat unknown controller presence
      # as a working cable. Startup arbitration uses the transport's checks.
      attached = False
  if not attached and ACTIVE_UDC is not None:
    SESSION_DETACHED = True
  return attached


def _role_attached():
  if TRANSPORT_MODE == 'auto':
    try:
      return DATA_ROLE.read_text().strip() == 'ufp'
    except FileNotFoundError:
      # Older systems have no data-role attribute. Never substitute the power
      # role when a present attribute reports dfp or is unreadable.
      pass
    except OSError:
      return False
  try:
    if TRANSPORT_MODE in ('ios', 'android'):
      return DATA_ROLE.read_text().strip() == 'ufp'
    # CC orientation alone also detects eGPU/hub cables. Require the peer to
    # be a source/host: never seize the existing eGPU's USB role.
    return ROLE.read_text().strip().startswith('Source attached')
  except OSError:
    return False


def publish(status, spec=None, **extra):
  record = dict(state=status, updated=time.monotonic(), model=model_name(spec) if spec else 'App selection',
                sha256=spec.sha256 if spec else None,
                transport=SESSION_MODE if TRANSPORT_MODE == 'auto' else TRANSPORT_MODE,
                configured_transport=TRANSPORT_MODE, **extra)
  temporary = STATUS.with_suffix('.tmp')
  temporary.write_text(json.dumps(record))
  os.replace(temporary, STATUS)


class CarrotTransport(FfsTransport):
  def _configured(self):
    deadline = getattr(self, '_probe_deadline', None)
    if deadline is not None and time.monotonic() >= deadline:
      raise LinkTimeout('Auto USB discovery deadline expired')
    return super()._configured()

  def _write_timeout(self, default=None):
    timeout = super()._write_timeout(default)
    deadline = getattr(self, '_probe_deadline', None)
    if deadline is not None:
      remaining = deadline - time.monotonic()
      if remaining <= 0:
        raise LinkTimeout('Auto USB discovery deadline expired')
      return remaining if timeout is None else min(timeout, remaining)
    return timeout

  def recv(self, timeout=None):
    deadline = getattr(self, '_probe_deadline', None)
    if deadline is not None:
      remaining = deadline - time.monotonic()
      if remaining <= 0:
        raise LinkTimeout('Auto USB discovery deadline expired before HELLO receive')
      timeout = remaining if timeout is None else min(timeout, remaining)
    return super().recv(timeout)

  def _wait_for_host_ready(self):
    deadline = getattr(self, '_probe_deadline', None)
    if deadline is not None and time.monotonic() >= deadline:
      return False
    return super()._wait_for_host_ready()

  def close(self):
    reader = getattr(self, '_reader', None)
    watchdog = getattr(getattr(self, '_write_guard', None), 'thread', None)
    gadget = getattr(self, 'gadget', None)
    super().close()
    if any(thread is not None and thread.is_alive() for thread in (reader, watchdog)):
      raise UnsafeCleanupError('FunctionFS cleanup left an active reader/watchdog; refusing to rebind')
    if gadget is not None:
      try:
        bound = (Path(gadget) / 'UDC').read_text().strip()
      except OSError as exc:
        raise UnsafeCleanupError(f'Cannot verify FunctionFS controller cleanup: {exc}') from exc
      if bound:
        raise UnsafeCleanupError(f'FunctionFS controller remains bound to {bound}; refusing to rebind')

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
  spec = client.spec
  requested = True
  follows_app = peer.get('protocol') == 3 and may_provision(peer, session_mode())
  while host_attached():
    if follows_app and requested and params.get_bool('IsOffroad') and not params.get_bool('IsOnroad'):
      # The App's Use Model must not compete with our old ENGINE_REQ.
      # HELLO clears that request without dropping the physical cable.
      fresh_peer = client.hello()
      check_loaded(client, fresh_peer)
      requested = False
    if time.monotonic() - last_status >= 1:
      update_affinity()
      publish('ready', spec=spec, peer=peer, telemetry=client.last_state, telemetry_updated=telemetry_updated)
      last_status = time.monotonic()
    try:
      connection, _ = listener.accept()
    except TimeoutError:
      if wifi is not None:
        wifi.send(client)
      publish_hud(client, publisher)
      if time.monotonic() - last_ping > 2:
        client.last_state = client.state()
        check_loaded(client, client.last_state)
        telemetry_updated = time.monotonic()
        last_ping = time.monotonic()
      continue
    with connection:
      connection.settimeout(.5)
      reader = PacketReader(REQUEST.size + spec.warped_nbytes + spec.packed_nbytes)
      try:
        if not requested:
          check_loaded(client, client.state())
          result = client.ensure_engine(spec.sha256, spec.nbytes, frame_skip=spec.frame_skip, build_timeout=30)
          if result.to_dict() != spec.to_dict():
            raise ModelChanged('Jetlink model contract changed before local inference')
          requested = True
        send(connection, json.dumps({'spec': spec.to_dict(), 'peer': peer}).encode())
        while host_attached():
          loop_started, cpu_started = time.monotonic(), time.thread_time()
          if time.monotonic() - last_status >= 1:
            update_affinity()
            if publisher is not None:
              params.put_bool_nonblocking('ClusterHudConnected', bool((client.last_state or {}).get('carrot_hud_connected')))
            publish('ready', spec=spec, peer=peer, timings=list(client.last_timings),
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
            check_loaded(client, client.last_state)
            telemetry_updated = time.monotonic()
            continue
          received = time.monotonic()
          if len(request) != REQUEST.size + spec.warped_nbytes + spec.packed_nbytes:
            raise ValueError('invalid local inference request')
          frame, reset, source_sof = REQUEST.unpack_from(request)
          if reset not in (0, 1):
            raise ValueError('invalid reset flag')
          images = np.frombuffer(request, np.uint8, spec.warped_nbytes, REQUEST.size).reshape(spec.warped_shape)
          packed = np.frombuffer(request, np.float32, spec.packed_nelem, REQUEST.size + spec.warped_nbytes)
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
          check_loaded(client, client.last_state)
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
  global TRANSPORT_MODE, SESSION_MODE, ACTIVE_UDC, SESSION_DETACHED
  logging.basicConfig(level=logging.INFO)
  TRANSPORT_MODE = transport_mode()
  SESSION_MODE = TRANSPORT_MODE
  ACTIVE_UDC, SESSION_DETACHED = None, False
  protocol = ProtocolChoice()
  if TRANSPORT_MODE in ('ios', 'android'):
    if protocol.version != 3:
      raise ValueError('Phone Jetlink mode requires protocol 3')
    protocol.automatic = False
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
  peer = None
  owner = cable = None
  cleanup_error = None
  usb_probes = 0
  usb_mode = None
  tcp_seen = False
  mobile_unavailable = False
  try:
    while True:
      update_affinity()
      if cleanup_error:
        publish('blocked', peer=None, error=cleanup_error)
        time.sleep(1)
        continue
      if SESSION_DETACHED or not host_attached():
        if owner is not None or cable is not None:
          cleanup_error = close_resources(cable, owner)
          owner = cable = None
        peer = None
        SESSION_MODE = TRANSPORT_MODE
        usb_probes, usb_mode, tcp_seen = 0, None, False
        mobile_unavailable = False
        ACTIVE_UDC, SESSION_DETACHED = None, False
        # Forget a previous host's wire choice after a physical detach.
        protocol = ProtocolChoice()
        if TRANSPORT_MODE in ('ios', 'android'):
          protocol.automatic = False
        if cleanup_error:
          continue
        hint = {}
        if TRANSPORT_MODE in ('auto', 'ios', 'android'):
          hint['preparation'] = {'stage': 'usb-role', 'message': 'Phone must host USB; use a powered USB 3 hub or adapter'}
        publish('waiting', peer=peer, **hint)
        time.sleep(1)
        continue
      client = None
      wifi = None
      transport = None
      try:
        publish('connecting', peer=peer, protocol=protocol.version)
        gc.collect()
        if owner is None:
          # Optional iOS networking must not make existing bulk USB depend on
          # NCM, dnsmasq or a firewall backend. Cleanup is verified below first.
          setup_gadget('usb' if mobile_unavailable else TRANSPORT_MODE)
          udc = next(Path('/sys/class/udc').iterdir()).name
          owner = CarrotTransport('/dev/ffs-jetlink', gadget=GADGET, udc=udc)
          if TRANSPORT_MODE in ('auto', 'ios') and not mobile_unavailable:
            cable = MobileCable(owner)
            cable.start()
            if TRANSPORT_MODE == 'auto':
              log.warning('%s', AutoTransport.diagnostic)
        else:
          udc = owner.bound_udc
        transport = owner
        if TRANSPORT_MODE == 'auto' and usb_mode is not None:
          SESSION_MODE = usb_mode
        elif TRANSPORT_MODE in ('auto', 'ios') and not mobile_unavailable:
          limit = 2 if protocol.automatic else 1
          arbitration = (AutoTransport(owner, cable, allow_usb=not tcp_seen,
                                       grace=AutoTransport.RETRY_GRACE if usb_probes >= limit else None)
                         if TRANSPORT_MODE == 'auto' else None)
          while host_attached():
            try:
              if arbitration is not None:
                transport, SESSION_MODE = arbitration.accept(timeout=1.)
                if transport is owner and usb_probes >= limit:
                  usb_probes = 0
                if transport is not owner:
                  # A decisive dial locks this attachment to NCM even if its
                  # HELLO fails or the App takes a long time to redial.
                  tcp_seen = True
              else:
                transport = cable.accept(timeout=1.)
              break
            except (TimeoutError, LinkTimeout):
              message = ''
              if arbitration:
                message = ('NCM selected; waiting for the App to redial, without USB probing' if tcp_seen else
                           arbitration.fallback_diagnostic if usb_probes >= limit else arbitration.diagnostic)
              hint = {'preparation': {'stage': 'transport-selection', 'message': message}} if arbitration else {}
              publish('connecting', peer=None, protocol=protocol.version, **hint)
          else:
            raise LinkError('Cable disconnected before a transport was selected')
        wire_protocol = protocol
        if session_mode() == 'ios':
          if not protocol.automatic and protocol.version != 3:
            raise ValueError('Selected iOS cable requires protocol 3')
          wire_protocol = ProtocolChoice('3')
        transport.protocol_version = wire_protocol.version
        client = JetlinkClient(transport, name='carrot-jetlink')
        if TRANSPORT_MODE == 'auto' and transport is owner and usb_mode is None and not mobile_unavailable:
          usb_probes += 1
          publish('connecting', peer=None, preparation={'stage': 'usb-probe',
                  'message': f'Bounded USB HELLO probe {usb_probes}/{limit}, protocol {wire_protocol.version} (3 seconds); failure re-enumerates NCM'})
          peer = probe_usb(client, wire_protocol)
        else:
          peer = greet(client, wire_protocol)
        if TRANSPORT_MODE == 'auto':
          selected = classify_auto_peer(peer, 'tcp' if transport is not owner else 'usb')
          if selected is None and transport is owner and legacy_usb_peer(peer):
            selected = 'usb'
          if selected is None:
            raise ValueError('Unrecognized Jetlink App HELLO on the selected auto transport')
          SESSION_MODE = selected
          if transport is owner:
            usb_mode = selected
        speed = (Path('/sys/class/udc') / udc / 'current_speed').read_text().strip()
        validate_speed(speed, TRANSPORT_MODE, peer=peer, selected=session_mode())
        ACTIVE_UDC = udc
        if peer.get(WIFI_CAPABILITY) is True:
          wifi = WifiPublisher()
          # Provision before ensure_engine: a new host may need Internet to
          # fetch its first model. This is outside every model frame deadline.
          wifi.send(client, initial=True)
        wait_for_boot_update(client, peer, wifi, host_attached, publish)
        publish('loading', peer=peer)
        from openpilot.common.params import Params
        params = Params()
        last_progress = 0.

        def progress(stage, fraction, message, current_peer=peer):
          nonlocal last_progress
          if time.monotonic() - last_progress >= 1:
            publish('loading', peer=current_peer, preparation={'stage': stage, 'fraction': fraction, 'message': str(message)[:160]})
            last_progress = time.monotonic()

        peer = wait_for_selection(client, peer, progress)
        prepare(client, peer, lambda current=params: current.get_bool('IsOffroad') and not current.get_bool('IsOnroad'),
                host_attached, progress, mode=session_mode())
        spec = client.spec
        if spec is None:
          raise ValueError('Jetlink preparation completed without a model spec')
        # Warm independently of camera/modeld; every real session resets state.
        for frame in range(10):
          client.infer(np.zeros(spec.warped_shape, np.uint8), np.zeros(spec.packed_nelem, np.float32), frame, reset=True)
        log.info('Jetlink ready: %s', peer)
        serve_local(listener, client, peer, wifi)
      except PreparationDeferred as exc:
        log.info('%s', exc)
        publish('loading', peer=peer, preparation={'stage': 'waiting', 'message': str(exc)})
      except UnsafeCleanupError as exc:
        log.exception('Jetlink constructor cleanup was unsafe')
        cleanup_error = f'Unsafe Jetlink cleanup; restart/reboot offroad before retrying: {exc}'[:300]
        publish('blocked', peer=None, error=cleanup_error)
      except MobileLinkError as exc:
        log.exception('Jetlink cable setup failed')
        if TRANSPORT_MODE == 'auto' and not mobile_unavailable:
          mobile_unavailable = True
          publish('retrying', peer=None, error=f'iOS/NCM unavailable; retrying bulk USB after verified cleanup: {exc}'[:300])
        else:
          publish('retrying', peer=None, error=str(exc)[:300])
      except Exception as exc:
        log.exception('Jetlink connection failed')
        publish('retrying', peer=peer, error=str(exc)[:300])
      finally:
        ACTIVE_UDC = None
        if client is not None and client.last_state is not None and 'carrot_hud_connected' in client.last_state:
          from openpilot.common.params import Params
          try:
            Params().put_bool_nonblocking('ClusterHudConnected', False)
          except Exception:
            log.exception('Jetlink HUD cleanup failed')
        cleanup_error = close_resources(wifi, client) or cleanup_error
        if client is None and transport is not owner:
          cleanup_error = close_resources(transport) or cleanup_error
        if TRANSPORT_MODE not in ('auto', 'ios'):
          if client is None:
            cleanup_error = close_resources(owner) or cleanup_error
          owner = None
        elif transport is owner or cable is None or getattr(cable, 'listener', None) is None:
          # Failed startup is not a valid persistent listener. Do not retry a
          # gadget whose cleanup could not prove the previous owner is gone.
          cleanup_error = close_resources(cable, owner) or cleanup_error
          cable = owner = None
        peer = None
        SESSION_MODE = TRANSPORT_MODE
      time.sleep(2)
  finally:
    close_resources(cable, owner, listener)
    Path(SOCKET).unlink(missing_ok=True)
    publish('stopped')


def close_resources(*resources):
  """Log every cleanup failure; caller must block rather than reclaim unsafely."""
  failures = []
  for resource in resources:
    if resource is not None:
      try:
        resource.close()
      except Exception as exc:
        log.exception('Jetlink %s cleanup failed', type(resource).__name__)
        failures.append(f'{type(resource).__name__}: {exc}')
  return ('Unsafe Jetlink cleanup; restart/reboot offroad before retrying: ' + '; '.join(failures))[:300] if failures else None


def session_mode():
  return SESSION_MODE if TRANSPORT_MODE == 'auto' else TRANSPORT_MODE


def legacy_usb_peer(peer):
  if not (isinstance(peer, dict) and type(peer.get('protocol')) is int and peer['protocol'] in (2, 3)):
    return False
  # The signed-update bootstrap intentionally has no inference backend/device.
  # It must reach Wi-Fi provisioning and wait_for_boot_update before model setup.
  if peer.get('carrot_host') == 'jetson' and peer.get(BOOT_UPDATE_CAPABILITY) is True:
    return True
  return (peer.get('carrot_host') in (None, 'jetson') and peer.get('backend') == 'trt'
          and isinstance(peer.get('device'), str) and 'orin' in peer['device'].lower())


def probe_usb(client, protocol, timeout=3.):
  """Bound endpoint readiness, every write and HELLO receive; never parallel TCP."""
  client.t._probe_deadline = time.monotonic() + timeout
  try:
    return greet(client, protocol, timeout=timeout)
  finally:
    client.t._probe_deadline = None


def validate_speed(speed, mode, peer=None, selected=None):
  accepted = ('super-speed', 'super-speed-plus')
  phone_or_mac = mode in ('ios', 'android')
  if mode == 'auto':
    path = 'tcp' if selected == 'ios' else 'usb'
    phone_or_mac = classify_auto_peer(peer, path) is not None
  if phone_or_mac:
    accepted += ('high-speed',)
  if speed not in accepted:
    raise RuntimeError(f'Unsupported USB speed {speed!r} for {mode}; use a USB 3 data cable')
  if mode == 'auto' and speed == 'high-speed':
    log.warning('Recognized %s App is using USB 2; real-time throughput is not guaranteed', selected)


def wait_for_selection(client, peer, progress):
  if peer.get('protocol') != 3 or not may_provision(peer, session_mode()):
    return peer
  while peer.get('loaded') is None:
    if not host_attached():
      raise PreparationDeferred('Jetlink host disconnected before App model preparation')
    progress('app-selection', None, 'With ignition off, select Use Model in the Jetlink App')
    time.sleep(1)
    current = client.state()
    if not isinstance(current, dict) or 'loaded' not in current:
      raise ValueError('Jetlink App did not report its loaded model')
    peer = dict(peer, loaded=current['loaded'])
  return peer


def greet(client, protocol, timeout=None):
  try:
    return client.hello() if timeout is None else client.hello(timeout=timeout)
  except LinkError:
    attempted = protocol.version
    protocol.handshake_failed()
    log.warning('Jetlink HELLO failed using protocol %d; next fresh session will use %d',
                attempted, protocol.version)
    raise


if __name__ == '__main__':
  main()
