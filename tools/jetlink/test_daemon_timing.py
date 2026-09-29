import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest


@pytest.mark.parametrize('navigation', [False, True])
def test_timing_records_usb_and_display_tail_after_unchanged_reply(monkeypatch, navigation):
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  from openpilot.selfdrive.modeld.jetlink import daemon
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

  request = (daemon.REQUEST.pack(42, 1, 123) + bytes(daemon.SPEC.warped_nbytes) + bytes(daemon.SPEC.packed_nbytes))

  def receive(connection):
    step('receive', 1.)  # Normal idle time must stay separate from USB work.
    return request

  monkeypatch.setattr(daemon, 'PacketReader', lambda _: NS(receive=receive))
  output = np.zeros(daemon.SPEC.output_nelem, np.float32)

  class Connection:
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def settimeout(self, timeout): pass

  connection = Connection()

  def begin(images, packed, frame, reset, want_state):
    assert frame == 42 and reset and not want_state
    assert images.shape == daemon.SPEC.warped_shape and packed.size == daemon.SPEC.packed_nelem
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
  client = NS(last_state=None, last_timings=(20000, 1000, 22000), infer_begin=begin, infer_end=end)
  peer = {'carrot_host': 'jetson', daemon.NAVI_CAPABILITY: navigation}
  daemon._serve_local(NS(accept=lambda: (connection, None)), client, peer, NS(),
                      NS(sent=lambda sof: step('phase', .001)), NS(send=lambda _: step('wifi', .007)))

  assert replies == [(42, 20000, 1000, 22000)]
  assert calls == (['receive', 'usb_send', 'phase', 'hud', 'usb_response', 'reply', 'tail', 'wifi'] if navigation else
                   ['receive', 'usb_send', 'phase', 'usb_response', 'reply', 'hud', 'wifi'])
  assert len(events) == 1 and events[0]['component'] == 'jetlinkd'
  assert events[0]['frame_id'] == 42 and events[0]['usb_seq'] == 99
  metrics = events[0]['metrics']
  for key, value in {'ipc_receive_ms': 1000., 'usb_send_ms': 3., 'usb_response_ms': 80.,
                     'ipc_reply_ms': 2., 'hud_ms': 4. if navigation else 0.,
                     'display_tail_ms': 6. if navigation else 4., 'wifi_tail_ms': 7.,
                     'server_total_ms': 22.}.items():
    assert metrics[key]['max'] == pytest.approx(value)
