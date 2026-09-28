from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'third_party/jetlink'))
from jetlink.server import cache


def test_unchanged_preload_does_not_write(tmp_path, monkeypatch):
  engine = cache.EngineCache(tmp_path, SimpleNamespace(name='synthetic'))
  engine.remember_loaded('a' * 64, 2)
  def fail(*args, **kwargs):
    raise AssertionError('unchanged preload wrote storage')
  monkeypatch.setattr(cache.tempfile, 'NamedTemporaryFile', fail)
  engine.remember_loaded('a' * 64, 2)
  assert engine.last_loaded() == ('a' * 64, 2)


def test_interrupted_metadata_replacement_preserves_previous_preload(tmp_path, monkeypatch):
  engine = cache.EngineCache(tmp_path, SimpleNamespace(name='synthetic'))
  engine.remember_loaded('a' * 64, 2)
  def fail(*args):
    raise OSError('simulated power cut before rename')
  monkeypatch.setattr(cache.os, 'replace', fail)
  engine.remember_loaded('b' * 64, 3)
  assert engine.last_loaded() == ('a' * 64, 2)
  assert not list(tmp_path.glob('.last-loaded-*'))
