"""Remote signal freshness, frame alignment, actual renderer and packet budgets."""
import copy
import json
from types import SimpleNamespace as NS

import pytest

import hud_signal as h
import hud_protocol as protocol
import hud_camera
from openpilot.selfdrive.carrot import signal_display as signals


def observation(stamp=10., frame=20, raw='red', session='observer'):
  return dict(version=1, stream='road', size=[1344, 760], timestamp=stamp,
              frame_id=frame, session=session,
              tracks=[dict(id=3, box=[600, 200, 640, 215], age=0, raw=raw, state=raw)])


def link(value=None):
  now = [100.1]
  packet = {'sent': 10.1, 'signal_display': {'version': 1, 'enabled': True, 'record': value or observation()}}
  snapshot = [(100.1, packet)]
  remote = h.RemoteSignals(lambda: snapshot[0], lambda: now[0])
  return remote, now, packet, snapshot


def test_source_only_metadata_mode_expiry_and_allowlist(tmp_path, monkeypatch):
  path = tmp_path / 'signal'
  monkeypatch.setattr(signals, 'display_requested', lambda: True)
  value = observation()
  value['unused'] = 'do not forward'
  path.write_text(json.dumps(value))
  packet = h.signal_packet(10.1, path)
  assert packet['record'] == observation()
  assert h.signal_packet(12.01, path)['record'] is None
  path.write_bytes(b'x' * (signals.MAX_BYTES + 1))
  assert h.signal_packet(10.1, path)['record'] is None
  monkeypatch.setattr(signals, 'display_requested', lambda: False)
  assert h.signal_packet(10.1, path) == dict(version=1, enabled=False, record=None)


@pytest.mark.parametrize('fault', ['missing', 'nan', 'future', 'old', 'session', 'frame', 'wide', 'size', 'many', 'bad_box'])
def test_invalid_signal_metadata_rejected(fault):
  value = observation()
  if fault == 'missing': value = None
  if fault == 'nan': value['timestamp'] = float('nan')
  if fault == 'future': value['timestamp'] = 10.2
  if fault == 'old': value['timestamp'] = 7.
  if fault == 'session': value['session'] = 'x' * 129
  if fault == 'frame': value['frame_id'] = True
  if fault == 'wide': value['stream'] = 'wide'
  if fault == 'size': value['size'] = [384, 240]
  if fault == 'many': value['tracks'] *= 21
  if fault == 'bad_box': value['tracks'][0]['box'][0] = float('inf')
  result = h.clean_record(value, 10.1)
  assert result is None or result['tracks'] == []


def test_clock_domains_repeat_expiry_and_disconnect():
  remote, now, packet, snapshot = link()
  assert remote.status() == (1, False, True)
  assert remote.record()['timestamp'] == 100.
  # Later HUD packets with the same source observation cannot renew it.
  for source, host in [(10.4, 100.45), (11.9, 102.05)]:
    packet['sent'] = source
    now[0] = host
    snapshot[0] = (host, packet)
  assert remote.status() == (0, False, False)
  packet['sent'] = 12.
  packet['signal_display']['record'] = observation(11.9, 58, 'green')
  now[0] = 102.2
  snapshot[0] = (102.2, packet)
  assert remote.status() == (2, False, True)
  snapshot[0] = None
  assert remote.status() == (0, False, False)


def test_unknown_preserves_only_bounded_previous_color_and_restart_clears():
  remote, now, packet, snapshot = link()
  assert remote.status() == (1, False, True)
  packet['sent'] = 10.6
  packet['signal_display']['record'] = observation(10.5, 30, 'unknown')
  snapshot[0] = (100.6, packet); now[0] = 100.6
  assert remote.status() == (1, True, True)
  packet['sent'] = 12.2
  packet['signal_display']['record'] = observation(12.1, 62, 'unknown')
  snapshot[0] = (102.2, packet); now[0] = 102.2
  assert remote.status() == (0, False, True)
  packet['signal_display']['record'] = observation(12.1, 62, 'green', 'restart')
  assert remote.status() == (2, False, True)
  packet['signal_display']['enabled'] = False
  assert remote.status() == (0, False, False)


@pytest.mark.parametrize('fault', ['legacy', 'missing', 'disabled', 'stale_snapshot', 'future_receive', 'version'])
def test_optional_or_stale_payload_hides_all_remote_results(fault):
  remote, now, packet, snapshot = link()
  assert remote.status()[0] == 1
  if fault == 'legacy': packet.pop('signal_display')
  if fault == 'missing': packet['signal_display']['record'] = None
  if fault == 'disabled': packet['signal_display']['enabled'] = False
  if fault == 'stale_snapshot': now[0] += .51
  if fault == 'future_receive': now[0] -= 1
  if fault == 'version': packet['signal_display']['version'] = 2
  assert remote.status() == (0, False, False)
  assert remote.reader().read((20, 100., 1344, 760, False)) == ()


def test_camera_uses_real_capture_metadata_not_jpeg_publish_time(monkeypatch):
  now = [100.1]
  monkeypatch.setattr(hud_camera.time, 'monotonic', lambda: now[0])
  camera = hud_camera.RemoteRoadCamera()
  frame = dict(frame=20, time=10.09, capture_time=10., source_width=1344, source_height=760)
  camera._set_display_frame(frame, 10.1, 100.1)
  assert camera.display_frame == (20, 100., 1344, 760, False)
  now[0] = 100.36
  camera._set_display_frame(frame, 10.2, 100.36)
  assert camera.display_frame is None  # Repeated JPEG cannot renew capture.
  camera._set_display_frame({'frame': 20}, 10.2, 100.36)
  assert camera.display_frame is None
  camera.select_stream(True)
  assert camera.display_frame is None


def test_actual_renderer_boxes_and_status_match_remote_camera(monkeypatch):
  from cluster_renderer import ClusterUiRenderer
  import cluster_renderer as renderer_module
  remote, now, packet, snapshot = link()
  monkeypatch.setattr(signals, 'signal_display_status', remote.status)
  monkeypatch.setattr(signals, 'display_requested', remote.requested)
  reader = remote.reader()
  camera = NS(display_frame=(20, 100., 1344, 760, False), is_wide=False)
  renderer = object.__new__(ClusterUiRenderer)
  renderer._signal_display_reader = reader
  renderer._brake_display_reader = NS(read=lambda frame: None)
  renderer.height, renderer.language = 760, 'en'
  labels, rectangles = [], []
  renderer._draw_text_with_stroke = lambda text, *args, **kw: labels.append(text)
  monkeypatch.setattr(renderer_module.rl, 'draw_rectangle_lines_ex', lambda rect, *args: rectangles.append(rect))
  projection = NS(dest=NS(x=0, y=0, width=1344, height=760), video_dest=NS(x=0, y=0, width=1344, height=760))
  renderer._draw_signal_detections(projection, camera)
  assert 'R' in labels and any('LIVE' in text for text in labels)
  assert len(rectangles) == 2 and rectangles[0].x == 598 and rectangles[0].width == 44
  # Wide and mismatched frames must not acquire road-camera boxes.
  for frame in [(20, 100., 1344, 760, True), (100, 100., 1344, 760, False), None]:
    rectangles.clear(); labels.clear(); camera.display_frame = frame
    renderer._draw_signal_detections(projection, camera)
    assert rectangles == []
  # Existing camera-independent colored status ring is also actually called.
  rings = []
  monkeypatch.setattr(renderer_module.rl, 'draw_circle_lines', lambda *args: rings.append(args))
  renderer._draw_model_traffic_state(0)
  assert len(rings) == 3


def test_budget_never_evicts_existing_camera_for_signals():
  base = dict(events={}, cameras={'road': {'jpeg': 'x' * (96 * 1024 - 100)}})
  before = protocol.encode_snapshot(copy.deepcopy(base))
  candidate = copy.deepcopy(base)
  candidate['signal_display'] = dict(version=1, enabled=True, record=observation())
  assert protocol.encode_snapshot(candidate) == before


def test_maximum_metadata_is_bounded_and_no_additional_image():
  value = observation()
  value['tracks'] = [dict(value['tracks'][0], id=i + 1) for i in range(20)]
  record = h.clean_record(value, 10.1)
  assert record is not None
  assert len(json.dumps(record, separators=(',', ':')).encode()) <= h.MAX_WIRE_BYTES
  assert set(record) == {'version', 'stream', 'size', 'timestamp', 'frame_id', 'session', 'tracks'}


def test_install_connects_already_imported_renderer_without_vehicle_writes(monkeypatch):
  import cluster_renderer
  with monkeypatch.context() as restore:
    # Register original bindings with pytest before install mutates them.
    for module, name in [(signals, 'SignalDisplayReader'), (signals, 'signal_display_status'),
                         (signals, 'display_requested'), (cluster_renderer, 'SignalDisplayReader')]:
      restore.setattr(module, name, getattr(module, name))
    remote = h.install(lambda: None)
    assert isinstance(cluster_renderer.SignalDisplayReader(), h.RemoteSignalReader)
    assert signals.signal_display_status() == (0, False, False)
    assert not signals.display_requested()
    assert remote.reader().record_draw(None, 1) is None


def test_actual_remote_camera_draw_refreshes_reused_frame_id_and_binds_capture(monkeypatch):
  import base64
  import io
  import pyray as rl
  from PIL import Image
  buffer = io.BytesIO()
  Image.new('RGB', (384, 240), (90, 80, 70)).save(buffer, format='JPEG')
  frame = dict(width=384, height=240, frame=20, time=10.1, capture_time=10.,
               source_width=1344, source_height=760, jpeg=base64.b64encode(buffer.getvalue()).decode())
  packet = dict(sent=10.1, cameras={'road': frame})
  now = [100.1]
  snapshot = [(100.1, packet)]
  monkeypatch.setattr(protocol, 'read_snapshot', lambda: snapshot[0])
  monkeypatch.setattr(hud_camera.time, 'monotonic', lambda: now[0])
  updates = []
  monkeypatch.setattr(rl, 'load_texture_from_image', lambda image: NS(id=1))
  monkeypatch.setattr(rl, 'set_texture_filter', lambda *args: None)
  monkeypatch.setattr(rl, 'update_texture', lambda *args: updates.append(True))
  monkeypatch.setattr(rl, 'draw_texture_pro', lambda *args: None)
  camera = hud_camera.RemoteRoadCamera()
  assert camera.draw(NS()) and camera.display_frame == (20, 100., 1344, 760, False)
  # Camera restart may reuse a frame ID; decode new JPEG and bind new timestamp.
  frame.update(time=10.2, capture_time=10.15)
  packet['sent'] = 10.2; snapshot[0] = (100.2, packet); now[0] = 100.2
  assert camera.draw(NS()) and len(updates) == 1
  assert camera.display_frame[1] == pytest.approx(100.15)
  snapshot[0] = None
  assert not camera.draw(NS()) and camera.display_frame is None
