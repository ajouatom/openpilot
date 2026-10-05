import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/carrot/cluster')]
import hud_health as hud
from host_health import thermal_health, storage_health


def sensor(root, index, temp, limit, cooling='cpu-balanced'):
  zone = root / f'thermal_zone{index}'
  zone.mkdir()
  (zone / 'type').write_text(f'cpu{index}-thermal')
  (zone / 'temp').write_text(str(temp * 1000))
  (zone / 'trip_point_0_type').write_text('passive')
  (zone / 'trip_point_0_temp').write_text(str(limit * 1000))
  (zone / 'cdev0_trip_point').write_text('0')
  (zone / 'cdev0').mkdir()
  (zone / 'cdev0/type').write_text(cooling)
  return zone


@pytest.mark.parametrize('temp,severity,color', [(93, 'ok', hud.NORMAL), (94, 'warning', hud.WARNING), (99, 'error', hud.ERROR)])
def test_real_trip_drives_temperature_and_bilingual_warning(tmp_path, temp, severity, color):
  sensor(tmp_path, 0, temp, 99)
  health = thermal_health(tmp_path)
  assert hud.temperature_label(health) == (f'{temp:.1f}°C', color)
  assert health['thermal_severity'] == severity
  for language, word in [('ko', '냉각'), ('en', 'cooling')]:
    message, _ = hud.health_banner(health, language)
    assert (word in message) == (severity != 'ok')
    if severity != 'ok':
      assert f'{temp:.1f}°C' in message and '99.0°C' in message and 'passive' not in message


def test_surface_notification_is_not_overheat_and_each_zone_uses_own_trip(tmp_path):
  sensor(tmp_path, 0, 75, 70, 'hot-surface-alert')
  assert thermal_health(tmp_path)['thermal_severity'] == 'ok'
  sensor(tmp_path, 1, 87, 92)
  health = thermal_health(tmp_path)
  assert health['thermal_severity'] == 'warning'
  assert 'cpu1-thermal' in health['thermal_reason'] and 'cpu0-thermal' not in health['thermal_reason']


def test_storage_fault_is_not_labelled_overheating(tmp_path):
  sensor(tmp_path, 0, 65, 99)
  health = thermal_health(tmp_path)
  marker = tmp_path / 'marker'
  marker.touch()
  storage_health(health, tmp_path / 'missing', marker)
  assert hud.temperature_label(health) == ('65.0°C', hud.NORMAL)
  message, color = hud.health_banner(health, 'en')
  assert 'Storage' in message and 'overheating' not in message and color == hud.ERROR


@pytest.mark.parametrize('value', [None, float('nan'), float('inf'), -256, 200, '99', True])
def test_missing_or_invalid_temperature_does_not_look_current(value):
  assert hud.temperature_label({'temp_c': value}) == ('--°C', hud.UNKNOWN)


def test_freshness_and_vehicle_alert_precedence(monkeypatch, tmp_path):
  import json
  path = tmp_path / 'health'
  path.write_text(json.dumps({'updated': 10., 'temp_c': 99., 'thermal_severity': 'error', 'thermal_reason': 'cpu 99C'}))
  monkeypatch.setattr(hud, 'STATUS', path)
  monkeypatch.setattr(hud.time, 'monotonic', lambda: 13.)
  monkeypatch.setattr(hud.ClusterUiRenderer, 'render', lambda *a: None)
  renderer = object.__new__(hud.HealthRenderer)
  renderer.render(NS())
  assert renderer.host_health == {}
  renderer.width, renderer.height, renderer.language = 1920, 480, 'ko'
  renderer.host_health = {'thermal_severity': 'error', 'thermal_reason': 'cpu 99C'}
  renderer.host_badge_drawn = True
  calls = []
  for name in ('rl_push_matrix', 'rl_pop_matrix', 'rl_scalef'):
    monkeypatch.setattr(hud.rl, name, lambda *a: None)
  monkeypatch.setattr(hud.rl, 'draw_rectangle', lambda *a: calls.append('strip'))
  monkeypatch.setattr(hud.ClusterUiRenderer, '_draw_alert_overlay', lambda *a: calls.append('vehicle'))
  renderer._draw_text = lambda *a, **kw: calls.append('text')
  renderer._ellipsize_text = lambda text, *a: text
  renderer._draw_alert_overlay(NS(size=3, text1='TAKE OVER', text2=''))
  assert calls == ['vehicle']
  calls.clear()
  renderer._draw_alert_overlay(None)
  assert calls == ['strip', 'text', 'vehicle']
