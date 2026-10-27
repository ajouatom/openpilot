"""Pinned upstream server with negotiated Carrot extensions; CAN output is opt-in."""
import os
import json
from pathlib import Path
import sys
import time

CHECK_IMPORTS = '--check-imports' in sys.argv

if __name__ == '__main__' and not CHECK_IMPORTS:
  if Path('/etc/nv_tegra_release').is_file():
    from install_updates import migrate_running_release
    migrate_running_release()
  from boot_update import wait_for_runtime
  wait_for_runtime()

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'third_party/jetlink'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from hud_protocol import HUD_CAPABILITY, HUD_MESSAGE, MAX_HUD_BYTES, HUD_PACKET, HEADER
from hud_navi import CAPABILITY as NAVI_CAPABILITY, MESSAGE as NAVI_MESSAGE, HostForwarder
from hud_navi import PUMP_CAPABILITY
from hud_wifi import CAPABILITY as HUD_WIFI_CAPABILITY, USB_CAPABILITY as HUD_USB_CAPABILITY
from hud_wifi import REQUEST as HUD_WIFI_REQUEST, RESPONSE as HUD_WIFI_RESPONSE, Worker
from host_reader import ReadAheadTransport
from host_health import health_worker
from host_lpm import SessionLease
from blink_protocol import Host as BlinkerHost, CAPABILITY as BLINKER_CAPABILITY, BEGIN as BLINKER_BEGIN, ACK as BLINKER_ACK, DATA as BLINKER_DATA
from wifi_protocol import CAPABILITY as WIFI_CAPABILITY, MESSAGE as WIFI_MESSAGE, receive as receive_wifi
from model_library import CAPABILITY as LIBRARY_CAPABILITY, REQUEST as LIBRARY_REQUEST, RESPONSE as LIBRARY_RESPONSE, host_request as library_request
from jetlink import protocol as P
from jetlink import request_chunks, deferred_map, packed_codec, output_projection
from jetlink.transport.base import Message, LinkError
from jetlink.server.session import Session
from jetlink.server import main as server


class DisplayTelemetry:
  def __init__(self, original):
    self.original = original
    self.blinker = None
    self.health = health_worker() if Path('/etc/nv_tegra_release').is_file() else None

  def read(self):
    connected = False
    try:
      status = json.loads(Path('/dev/shm/carrot-jetlink-hud-status.json').read_text())
      connected = 0 <= time.monotonic() - status['updated'] < 2
    except (OSError, ValueError, KeyError, TypeError):
      pass
    return {**self.original.read(), 'carrot_hud_connected': connected,
            **({'carrot_blinker': self.blinker.telemetry()} if self.blinker is not None and self.blinker.worker is not None else {}),
            **({'carrot_health': self.health.read()} if self.health else {})}


class CarrotSession(Session):
  def _send(self, msg_type, seq, parts=(), flags=0):
    if msg_type == P.Msg.INFER_RESP and self.telemetry.health is not None:
      status = P.unpack_infer_resp(parts[0])[1]
      if status != P.Status.OK:
        self.telemetry.health.record_error('Inference', f'result status {status}')
    return super()._send(msg_type, seq, parts, flags)

  def _error(self, seq, error, detail=''):
    if self.telemetry.health is not None:
      self.telemetry.health.record_error(error, detail)
    return super()._error(seq, error, detail)

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.request_chunks = request_chunks.Receiver()
    self.deferred_maps = deferred_map.Receiver()
    self.deferred_negotiated = False
    # Restore the bounded USB display receiver. The new capability is separate
    # from old HUD support so a Wi-Fi-only comma never starts USB accidentally.
    self.usb_display = True
    self.hud_wifi_supported = False
    if sys.platform == 'linux':
      self.t = ReadAheadTransport(self.t)
    self.telemetry = DisplayTelemetry(self.telemetry)
    self.blinker = BlinkerHost()
    self.telemetry.blinker = self.blinker
    self.navi = HostForwarder() if sys.platform == 'linux' and self.usb_display else None
    self.hud_wifi = None
    self.power_lease = SessionLease(self.t.transport if isinstance(self.t, ReadAheadTransport) else self.t)

  def _send_json(self, msg_type, seq, obj, flags=0):
    if msg_type == P.Msg.ENGINE_RESP and obj.get('state') == 'failed' and self.telemetry.health is not None:
      self.telemetry.health.record_error('Engine', obj.get('detail', 'failed'))
    if msg_type == P.Msg.HELLO_RESP:
      host = 'mac' if sys.platform == 'darwin' else (
        'jetson' if Path('/etc/nv_tegra_release').is_file() else 'unknown')
      legacy_display = sys.platform == 'linux' and self.usb_display
      obj = {**obj, HUD_CAPABILITY: legacy_display, NAVI_CAPABILITY: legacy_display,
             PUMP_CAPABILITY: legacy_display, WIFI_CAPABILITY: host == 'jetson', 'carrot_host': host,
             HUD_WIFI_CAPABILITY: self.hud_wifi_supported,
             HUD_USB_CAPABILITY: host == 'jetson' and legacy_display, LIBRARY_CAPABILITY: host == 'jetson'}
      if host == 'jetson' and self.blinker.supported():
        obj[BLINKER_CAPABILITY] = True
      if self.deferred_negotiated:
        obj[deferred_map.CAPABILITY] = deferred_map.descriptor()
      if host == 'jetson':
        obj[request_chunks.CAPABILITY] = request_chunks.descriptor()
        obj['carrot_source_commit'] = (ROOT / 'SOURCE_COMMIT').read_text().strip()
        obj['carrot_boot_update_installed'] = Path('/opt/carrot-jetlink/boot-update-required').is_file()
    return super()._send_json(msg_type, seq, obj, flags)

  def handle(self, msg):
    if msg.msg_type == LIBRARY_REQUEST:
      if msg.seq <= self.last_seq:
        return
      self.last_seq = msg.seq
      try:
        if len(msg.payload) > 1024:
          raise ValueError('oversized library request')
        result = library_request(self.host, json.loads(bytes(msg.payload)), ROOT)
      except (OSError, ValueError, TypeError) as exc:
        result = {'error': str(exc)[:160]}
      self._send_json(LIBRARY_RESPONSE, msg.seq, result)
      return
    if getattr(self, 'power_lease', None) is not None:
      self.power_lease.touch()
    if msg.msg_type == P.Msg.HELLO_REQ:
      if getattr(self, 'blinker', None) is not None:
        self.blinker.close()
        self.blinker = BlinkerHost()
        self.telemetry.blinker = self.blinker
      self.request_chunks.clear()
      self.deferred_maps.clear()
      self.deferred_negotiated = False
      try:
        offer = json.loads(bytes(msg.payload)).get('client') or {}
        self.deferred_negotiated = self.t.protocol_version == 2 and deferred_map.matches(offer.get(deferred_map.CAPABILITY))
      except (ValueError, AttributeError):
        pass
    elif msg.msg_type == request_chunks.MESSAGE:
      if self.t.protocol_version != 2:
        raise request_chunks.LinkError('chunked requests require protocol 2')
      msg = self.request_chunks.accept(msg, self.last_seq)
      if msg is None:
        return
    elif getattr(self, 'request_chunks', None) is not None and self.request_chunks.key is not None:
      if msg.seq <= self.last_seq:
        return
      self.request_chunks.clear()
      raise request_chunks.LinkError('interrupted chunked request')
    if msg.msg_type == P.Msg.ENGINE_REQ:
      self.deferred_maps.clear()
    if msg.msg_type == deferred_map.MESSAGE:
      if msg.seq <= self.last_seq:
        return
      try:
        with self.host.lock:
          if (not self.deferred_negotiated or self.t.protocol_version != 2 or
              self.host.loaded is None or not output_projection.eligible(self.host.loaded.spec)):
            raise ValueError('unnegotiated map staging')
          self.deferred_maps.accept(msg.seq, msg.payload)
        self.last_seq = msg.seq
      except ValueError as exc:
        self.deferred_maps.clear()
        raise LinkError(str(exc)) from exc
      return
    if msg.msg_type in (BLINKER_BEGIN, BLINKER_DATA):
      if msg.seq <= self.last_seq:
        return
      self.last_seq = msg.seq
      blinker = getattr(self, 'blinker', None)
      if msg.msg_type == BLINKER_BEGIN:
        result = {'ready': False}
        try:
          if blinker is not None and len(msg.payload) < 64 and json.loads(bytes(msg.payload)) == {'version': 2}:
            result = blinker.begin()
        except (OSError, ValueError, RuntimeError):
          pass
        self._send_json(BLINKER_ACK, msg.seq, result)
      elif blinker is not None:
        blinker.send(msg.payload)
      return
    if msg.msg_type == HUD_WIFI_REQUEST:
      if msg.seq <= self.last_seq:
        return
      self.last_seq = msg.seq
      ready = False
      try:
        if not self.hud_wifi_supported or len(msg.payload) > 128:
          raise ValueError('HUD Wi-Fi negotiation unavailable')
        request = json.loads(bytes(msg.payload))
        key = request['key']
        if not isinstance(key, str) or len(key) != 64 or len(bytes.fromhex(key)) != 32:
          raise ValueError('HUD Wi-Fi session key')
        if self.hud_wifi is not None:
          self.hud_wifi.close()
          self.hud_wifi = None
        self.hud_wifi = Worker('receive', key)
        ready = True
      except (OSError, ValueError, KeyError, TypeError, RuntimeError):
        pass  # Never log the session key or affect the model contract.
      self._send_json(HUD_WIFI_RESPONSE, msg.seq, {'ready': ready})
      return
    if msg.msg_type == WIFI_MESSAGE:
      if msg.seq > self.last_seq:
        self.last_seq = msg.seq
        receive_wifi(msg.payload)
      return
    if msg.msg_type == NAVI_MESSAGE:
      if msg.seq > self.last_seq:
        self.last_seq = msg.seq
        if self.usb_display and self.navi is not None:
          self.navi.send(msg.payload)
      return
    if msg.msg_type != HUD_MESSAGE:
      return super().handle(msg)
    if msg.seq <= self.last_seq:
      return
    self.last_seq = msg.seq
    if not self.usb_display:
      return  # Reject USB display even before negotiation or after WLAN failure.
    if not 0 < len(msg.payload) <= MAX_HUD_BYTES:
      return  # Display data must never change or invalidate a model result.
    temporary = HUD_PACKET.with_suffix('.tmp')
    try:
      with temporary.open('wb') as f:
        f.write(HEADER.pack(time.monotonic()))
        f.write(msg.payload)
      os.replace(temporary, HUD_PACKET)
    except OSError:
      pass  # An unavailable display does not stop inference.

  def _infer(self, loaded, msg):
    try:
      if msg.payload.nbytes < P.INFER_REQ_SIZE:
        raise ValueError('incomplete frame')
      frame, flags = P.unpack_infer_req(msg.payload)
      if flags & deferred_map.FLAG:
        from jetlink.block_codec import FLAG as LZ4_FLAG
        if (not self.deferred_negotiated or self.t.protocol_version != 2 or
            not flags & packed_codec.FLAG or flags & (LZ4_FLAG | P.Flag.RESET_QUEUES)):
          raise ValueError('unnegotiated map reference')
        offset = P.INFER_REQ_SIZE + loaded.spec.warped_nbytes
        packed = self.deferred_maps.expand(msg.payload[offset:], loaded.spec)
        payload = (P.pack_infer_req(frame, flags & ~deferred_map.FLAG) +
                   bytes(msg.payload[P.INFER_REQ_SIZE:offset]) + packed)
        msg = Message(msg.msg_type, msg.seq, msg.flags, memoryview(payload))
      else:
        # Every inline request supersedes the old image, including cancellation.
        self.deferred_maps.clear()
    except ValueError:
      self.deferred_maps.clear()
      self._send(P.Msg.INFER_RESP, msg.seq, (P.pack_infer_resp(0, P.Status.BAD_SHAPE, 0, 0, 0),))
      return
    return super()._infer(loaded, msg)

  def close(self):
    try:
      if getattr(self, 'blinker', None) is not None:
        self.blinker.close()
      if self.hud_wifi is not None:
        self.hud_wifi.close()
        self.hud_wifi = None
      if self.navi is not None:
        self.navi.close()
      if isinstance(self.t, ReadAheadTransport):
        self.t.close()
      super().close()
    finally:
      if getattr(self, 'power_lease', None) is not None:
        self.power_lease.close()


if __name__ == '__main__':
  if CHECK_IMPORTS:
    print('SERVER_IMPORTS_OK')
    raise SystemExit(0)
  if Path('/etc/nv_tegra_release').is_file():
    from blink_driver import prepare_optional
    prepare_optional()
    health_worker()
  server.Session = CarrotSession
  raise SystemExit(server.main())
