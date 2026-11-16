# Jetson USB CAN link file behind the HUD's yellow CAN badge
from openpilot.common import can_link


def test_publish_then_connected_until_stale(tmp_path, monkeypatch):
  path = tmp_path / 'can-link.json'
  monkeypatch.setattr(can_link, '_next_write', 0.)
  can_link.publish(100., True, path)
  assert can_link.connected(100., path) and can_link.connected(106.9, path)
  assert not can_link.connected(107.1, path)  # older than STALE_S: badge off
  assert not can_link.connected(99., path)  # stamp from the future (clock mismatch): off


def test_unavailable_missing_or_garbled_is_off(tmp_path, monkeypatch):
  path = tmp_path / 'can-link.json'
  assert not can_link.connected(0., path)
  monkeypatch.setattr(can_link, '_next_write', 0.)
  can_link.publish(10., False, path)
  assert not can_link.connected(10., path)
  path.write_text('{"available": true, "stamp": true}')
  assert not can_link.connected(1., path)
  path.write_text('not json')
  assert not can_link.connected(1., path)


def test_publish_is_throttled(tmp_path, monkeypatch):
  path = tmp_path / 'can-link.json'
  monkeypatch.setattr(can_link, '_next_write', 0.)
  can_link.publish(10., True, path)
  can_link.publish(10.2, False, path)  # within 0.5 s: not written
  assert can_link.connected(10.2, path)
  can_link.publish(10.6, False, path)
  assert not can_link.connected(10.6, path)
