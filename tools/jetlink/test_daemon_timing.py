import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest


@pytest.mark.parametrize('map_staging', [None, 0, 65560])
@pytest.mark.parametrize('navigation', [False, True])
@pytest.mark.parametrize('fast_receiver', [False, True])
@pytest.mark.parametrize('usb', [False, True])
@pytest.mark.parametrize('projection_request', [False, True])
def test_timing_records_usb_and_display_tail_after_unchanged_reply(monkeypatch, navigation, fast_receiver, usb, projection_request, map_staging):
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  from openpilot.selfdrive.modeld.jetlink import daemon, link
  from openpilot.common import runtime_diagnostics

  now = [10.]
  calls, events, replies = [], [], []

  def step(name, duration):
    calls.append(name)
    now[0] += duration

  monkeypatch.setattr(daemon.time, 'monotonic', lambda: now[0])
  monkeypatch.setattr(daemon.time, 'thread_time', lambda: 0.)
  monkeypatch.setattr(runtime_diagnostics.RuntimeDiagnostics, '_schedstat', lambda _: ())
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(put_bool_nonblocking=lambda *a: None)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', NS(cloudlog=NS(event=lambda *a, **kw: events.append(kw))))
  attached = iter([True, True, False, False])
  monkeypatch.setattr(daemon, 'host_attached', lambda: next(attached))
  monkeypatch.setattr(daemon, 'update_affinity', lambda: None)
  monkeypatch.setattr(daemon, 'publish', lambda *a, **kw: None)
  monkeypatch.setattr(daemon, 'send', lambda *a: None)

  request = (daemon.REQUEST.pack(42, (3 if projection_request else 1) | (4 if map_staging is not None else 0), 123) + bytes(link.SPEC.warped_nbytes) + bytes(link.SPEC.packed_nbytes))

  if map_staging is not None:
    from jetlink import deferred_map
    request += deferred_map.META.pack(77, 10_000_000_000, 10_500_000_000)

  def receive(connection):
    step('receive', 1.)  # Normal idle time must stay separate from USB work.
    return request

  monkeypatch.setattr(daemon, 'PacketReader', lambda _: NS(receive=receive))
  output = np.zeros(link.SPEC.output_nelem, np.float32)

  class Connection:
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def settimeout(self, timeout): pass

  connection = Connection()

  def begin(images, packed, frame, reset, want_state, **options):
    assert options == ({"nav_metadata": (77, 10_000_000_000, 10_500_000_000)} if map_staging is not None else {})
    assert frame == 42 and reset and not want_state
    assert client.project_output == projection_request
    assert images.shape == link.SPEC.warped_shape and packed.size == link.SPEC.packed_nelem
    step('usb_send', .003)
    return 99

  def end(seq):
    assert seq == 99
    step('usb_response', .08)
    return output

  def reply(conn, header, result):
    assert conn is connection and result is output
    replies.append(daemon.REPLY.unpack(header))
    step('reply', .002)

  monkeypatch.setattr(daemon, 'send_parts', reply)
  monkeypatch.setattr(daemon, 'publish_hud', lambda *a: step('hud', .004))
  monkeypatch.setattr(daemon, 'send_ready_after_reply', lambda *a, **kw: step('tail', .006))
  monkeypatch.setattr(daemon, 'send_ready_during_inference', lambda *a, **kw: step('early_media', .006) or 123)
  client = NS(deferred_map_available=True, flush_map=lambda: step('map_stage', .001) or map_staging, last_map_used=True, spec=link.SPEC, output_projection_available=projection_request, last_state=None, last_timings=(20000, 1000, 22000), infer_begin=begin, infer_end=end,
              t=NS(last_receive={'prepare': .002, 'read_wait': .025, 'handoff': .009}))
  peer = {'carrot_host': 'jetson', daemon.NAVI_CAPABILITY: navigation, daemon.PUMP_CAPABILITY: fast_receiver,
          daemon.HUD_USB_CAPABILITY: usb}
  daemon._serve_local(NS(accept=lambda: (connection, None)), client, peer, NS(),
                      NS(sent=lambda sof: step('phase', .001)), NS(send=lambda _: step('wifi', .007)))

  assert replies == [(42, 20000, 1000, 22000)]
  early = ['hud'] + (['early_media'] if fast_receiver and not map_staging else []) if usb and navigation else []
  tail = (['hud'] if not navigation else ['tail'] if not fast_receiver else []) if usb else []
  assert calls == ['receive', 'usb_send', 'phase', *(['map_stage'] if map_staging is not None else []), *early, 'usb_response', 'reply', *tail, 'wifi']
  assert len(events) == 1 and events[0]['component'] == 'jetlinkd'
  assert events[0]['frame_id'] == 42 and events[0]['usb_seq'] == 99
  metrics = events[0]['metrics']
  assert metrics['map_stage_bytes']['max'] == (map_staging or 0)
  assert metrics['map_deferred_used']['max'] == int(map_staging is not None)
  peak = events[0]['peak_sample']
  assert peak['context'] == {'frame_id': 42, 'usb_seq': 99}
  assert peak['metrics']['usb_send_ms'] == pytest.approx(3.)
  assert peak['metrics']['server_total_ms'] == pytest.approx(22.)
  hud_ms = ((4. + (6. if fast_receiver and not map_staging else 0.)) if usb and navigation else 0.) + (1. if map_staging is not None else 0.)
  tail_ms = (4. if not navigation else 6. if not fast_receiver else 0.) if usb else 0.
  for key, value in {'ipc_receive_ms': 1000., 'usb_send_ms': 3., 'usb_response_ms': 80.,
                     'ipc_reply_ms': 2., 'hud_ms': hud_ms,
                     'usb_exchange_ms': 84. + hud_ms,
                     'submit_to_response_ms': 81. + hud_ms,
                     'display_early_media_ms': 6. if usb and navigation and fast_receiver and not map_staging else 0.,
                     'display_early_media_bytes': 123 if usb and navigation and fast_receiver and not map_staging else 0,
                     'response_reader_prepare_max_ms': 2., 'response_reader_read_wait_max_ms': 25.,
                     'response_reader_handoff_max_ms': 9.,
                     'display_tail_ms': tail_ms, 'wifi_tail_ms': 7.,
                     'server_total_ms': 22.}.items():
    assert metrics[key]['max'] == pytest.approx(value)


@pytest.mark.parametrize('have_hud,have_media', [(True, True), (True, False), (False, True), (False, False)])
def test_display_metrics_separate_snapshot_and_usb_writes(monkeypatch, have_hud, have_media):
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  from openpilot.selfdrive.modeld.jetlink import daemon
  now = [0.]
  calls = []
  def packet():
    calls.append('snapshot')
    now[0] += .002
    return b'hud' if have_hud else None
  def media():
    calls.append('media')
    now[0] += .001
    return b'video' if have_media else None
  def send(kind, seq, parts):
    calls.append((kind, seq, parts))
    now[0] += .003 if kind == daemon.HUD_MESSAGE else .007
  sequence = iter([10, 11])
  client = NS(t=NS(send=send), _next_seq=lambda: next(sequence))
  monkeypatch.setattr(daemon.time, 'monotonic', lambda: now[0])
  metrics = daemon.publish_hud(client, NS(packet=packet, media_packet=media))
  assert metrics == pytest.approx(dict(display_snapshot_read_ms=2., display_hud_send_ms=3. if have_hud else 0.,
                                      display_media_read_ms=1., display_media_send_ms=7. if have_media else 0.,
                                      display_hud_bytes=3 if have_hud else 0, display_media_bytes=5 if have_media else 0))
  expected = ['snapshot']
  if have_hud: expected.append((daemon.HUD_MESSAGE, 10, [b'hud']))
  expected.append('media')
  if have_media: expected.append((daemon.NAVI_MESSAGE, 11 if have_hud else 10, [b'video']))
  assert calls == expected
  assert not any(daemon.publish_hud(client, None).values())


def test_display_usb_failure_still_abandons_send_before_media(monkeypatch):
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  from openpilot.selfdrive.modeld.jetlink import daemon
  def fail(*args):
    raise ConnectionError('USB write failed')
  def unexpected():
    pytest.fail('must not consume the next media fragment after a failed USB write')
  with pytest.raises(ConnectionError, match='USB write failed'):
    daemon.publish_hud(NS(t=NS(send=fail), _next_seq=lambda: 1), NS(packet=lambda: b'hud', media_packet=unexpected))
