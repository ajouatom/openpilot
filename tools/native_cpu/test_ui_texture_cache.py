"""Admission, immediate fallback and frame-boundary ownership without a GPU."""
from types import SimpleNamespace

import pytest

from openpilot.system.ui.lib import text_texture, native_text


@pytest.fixture
def cache(monkeypatch):
  monkeypatch.setattr(text_texture, 'ENABLED', True)
  monkeypatch.setattr(native_text, '_ENABLED', True)
  monkeypatch.setattr(native_text.native_draw, '_ENABLED', True)
  instance = text_texture.TextTextureCache()
  calls = []
  instance.compositor = SimpleNamespace(identity=lambda: True, draw=lambda *args: calls.append(('draw', args)))
  rl = SimpleNamespace(ffi=SimpleNamespace(cast=lambda _, v: v, addressof=lambda v: v.id),
                       unload_render_texture=lambda v: calls.append(('unload', v)), unload_shader=lambda v: None)
  font = SimpleNamespace(texture=SimpleNamespace(id=1, width=512, height=512, mipmaps=1, format=7), baseSize=64,
                         glyphCount=100, glyphPadding=4, recs=123, glyphs=456)
  args = [rl, font, b'CRUISE', 15., 30., 32., 2., 4., (255, 255, 255, 255), (0, 0, 0, 255), (0, 0, 0, 100)]

  def build(rl, key, font, args):
    instance.entries[key] = (SimpleNamespace(texture=SimpleNamespace(id=99)), (10, 20, 80, 40), 3200)
    instance.pixels += 3200
    calls.append(('build', key))
    yield from ()
  monkeypatch.setattr(instance, '_build', build)
  return instance, args, calls


def warm(instance, args):
  for _ in range(text_texture.STABLE_FRAMES):
    instance.prepare(args[0])
    assert not instance.draw(*args)
  instance.prepare(args[0])
  assert instance.draw(*args)


def test_stable_distinct_frames_and_single_build_budget(cache):
  instance, args, calls = cache
  other = args.copy()
  other[2] = b'VISION'
  for _ in range(text_texture.STABLE_FRAMES):
    instance.prepare(args[0])
    for _ in range(3):
      assert not instance.draw(*args)
      assert not instance.draw(*other)
  assert not any(c[0] == 'build' for c in calls)
  instance.prepare(args[0])
  assert sum(c[0] == 'build' for c in calls) == 1
  assert instance.draw(*args) != instance.draw(*other)
  instance.prepare(args[0])
  assert sum(c[0] == 'build' for c in calls) == 2


@pytest.mark.parametrize('index,value', [(2, b'NEW'), (3, 15.01), (4, 30.01), (5, 31.), (6, 3.), (7, -4.),
                                        (8, (255, 0, 0, 255)), (9, (10, 20, 30, 40)), (10, (0, 0, 0, 200))])
def test_new_value_never_draws_old_image(cache, index, value):
  instance, args, calls = cache
  warm(instance, args)
  before = len(calls)
  args[index] = value
  assert not instance.draw(*args)
  assert len(calls) == before


@pytest.mark.parametrize('reason', ['recording', 'scale', 'transform', 'backend', 'multiline', 'font'])
def test_ineligible_context_keeps_original_text(cache, monkeypatch, reason):
  instance, args, calls = cache
  warm(instance, args)
  if reason == 'recording':
    instance.prepare(args[0], direct=False)
  elif reason == 'scale':
    instance.prepare(args[0], scale=.75)
  elif reason == 'transform':
    instance.compositor.identity = lambda: False
  elif reason == 'backend':
    monkeypatch.setattr(native_text, '_ENABLED', False)
  elif reason == 'multiline':
    args[2] = b'A\nB'
  else:
    args[1].texture.id += 1
  before = len(calls)
  assert not instance.draw(*args)
  assert len(calls) == before


def test_absence_resets_admission_and_pending_storage_bounded(cache):
  instance, args, _ = cache
  instance.prepare(args[0])
  for i in range(400):
    args[2] = str(i).encode()
    instance.draw(*args)
  assert len(instance.observed) <= text_texture.MAX_ENTRIES * 2
  instance.prepare(args[0])
  instance.prepare(args[0])
  assert not instance.observed


def test_clear_releases_gpu_resources_and_disables_until_next_frame(cache):
  instance, args, calls = cache
  warm(instance, args)
  instance.clear(args[0])
  assert not instance.entries and not instance.observed and instance.pixels == 0
  assert any(c[0] == 'unload' for c in calls)
  assert not instance.draw(*args)


def test_build_failure_falls_back_once(cache, monkeypatch):
  instance, args, _ = cache
  warm(instance, args)
  instance.clear(args[0])
  def fail(*args):
    raise RuntimeError('binding mismatch')
  monkeypatch.setattr(instance, '_build', fail)
  for _ in range(text_texture.STABLE_FRAMES):
    instance.prepare(args[0])
    instance.draw(*args)
  instance.prepare(args[0])
  assert instance.disabled and not instance.ready and not instance.draw(*args)


@pytest.mark.parametrize('cancel', [False, True])
def test_build_stages_are_distributed_and_cancelled_on_disappearance(cache, monkeypatch, cancel):
  instance, args, calls = cache
  def stages(*args):
    try:
      calls.append(('ink',))
      yield
      calls.append(('coverage',))
      yield
      calls.append(('convert',))
    finally:
      calls.append(('cleanup',))
  monkeypatch.setattr(instance, '_build', stages)
  for _ in range(text_texture.STABLE_FRAMES):
    instance.prepare(args[0])
    instance.draw(*args)
  instance.prepare(args[0])
  assert [c[0] for c in calls] == ['ink']
  if not cancel:
    instance.draw(*args)
  instance.prepare(args[0])
  if cancel:
    assert [c[0] for c in calls] == ['ink', 'cleanup']
  else:
    assert [c[0] for c in calls] == ['ink', 'coverage']
    instance.draw(*args)
    instance.prepare(args[0])
    assert [c[0] for c in calls] == ['ink', 'coverage', 'convert', 'cleanup']
  assert instance.pending is None
