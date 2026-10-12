"""Regression tests for daylight rim-green and delayed red acquisition."""
import cv2
import pytest

from tools.signal_analysis import signal_tracker as module  # noqa: TID251 - repository-root analysis tools
from tools.signal_analysis.test_signal_revalidation import picture  # noqa: TID251


BOX = [630, 230, 678, 242]


def tracker():
  return module.SignalTracker(daytime_cores=True, robust_tracking=True, verified_lamps=True)


def detection(color, box=BOX):
  return {'box': list(box), 'raw': color, 'quality': .9, 'source': 'detected'}


def test_red_acquires_with_300ms_frames_but_green_does_not_relax_its_confirmation():
  t = tracker()
  for stamp in [1., 1.3, 1.6]:
    r = t.update(stamp, [detection('red')])
  assert r['state'] == 'red' and r['tracks'][0]['observations'] == 3
  for stamp in [1.9, 2.2, 2.5]:
    r = t.update(stamp, [detection('green')])
    assert r['state'] != 'green'


@pytest.mark.parametrize('gap,box', [(.501, BOX), (.3, [650, 230, 698, 242])])
def test_red_acquisition_does_not_bridge_long_gap_or_other_housing(gap, box):
  t = tracker()
  t.update(1., [detection('red')])
  t.update(1.2, [detection('red')])
  r = t.update(1.2+gap, [detection('red', box)])
  assert r['state'] == 'unknown'
  assert next(v for v in r['tracks'] if v['age'] == 0)['observations'] == 1


@pytest.mark.parametrize('color', [(0, 240, 0), (0, 220, 230), (50, 160, 100)])
def test_bright_interior_green_and_cyan_are_supported(color):
  rgb = picture('unknown')
  cv2.circle(rgb, (672, 236), 3, color, -1)
  assert module.illuminated_green(rgb, BOX)


def test_colored_sky_on_housing_border_is_not_a_lit_green():
  rgb = picture('unknown')
  rgb[230:242, 677] = (70, 110, 130)
  rgb[241, 665:678] = (70, 110, 130)
  assert not module.illuminated_green(rgb, BOX)


def test_green_looking_unlit_patch_is_not_a_lamp():
  rgb = picture('unknown')
  rgb[231:241, 645:678] = (20, 80, 70)
  assert not module.illuminated_green(rgb, BOX)


def test_bright_weakly_saturated_blue_sky_patch_is_not_cyan_lamp():
  rgb = picture('unknown')
  cv2.circle(rgb, (672, 236), 3, (100, 140, 154), -1)
  assert not module.illuminated_green(rgb, BOX)


def test_dark_images_keep_verified_identity_but_never_invent_color_or_green_release(monkeypatch):
  t = tracker()
  monkeypatch.setattr(module, 'detect', lambda *args, **kwargs: [detection('red')])
  for stamp in [1., 1.15, 1.3]:
    r = t.process(picture('red'), stamp)
  ident = r['tracks'][0]['id']
  assert r['state'] == 'red'
  monkeypatch.setattr(module, 'detect', lambda *args, **kwargs: [detection('green')])
  for stamp in [1.5, 1.7, 1.9, 2.1, 2.3]:
    # A false-green proposal at the old red box must undergo the same current
    # image check as a template. Neither source may inherit green permission.
    r = t.process(picture('unknown'), stamp)
    assert r['state'] == 'unknown'
    assert r['tracks'][0]['id'] == ident and r['tracks'][0]['seen_red']
    assert r['tracks'][0]['evidence']['raw'] == 'unknown'
  for stamp in [2.45, 2.6, 2.75]:
    r = t.process(picture('green'), stamp)
  assert r['state'] == 'green'


def test_unknown_does_not_help_acquire_red_or_green():
  t = tracker()
  for stamp, color in [(1., 'red'), (1.2, 'unknown'), (1.4, 'red'), (1.6, 'green'), (1.8, 'green'), (2., 'green')]:
    r = t.update(stamp, [detection(color)])
    assert r['state'] == 'unknown'


def test_image_loss_expires_red_identity_and_green_cannot_reuse_it():
  t = tracker()
  for stamp in [1., 1.15, 1.3]:
    t.update(stamp, [detection('red')])
  assert t.update(1.81, [])['tracks'] == []
  for stamp in [2., 2.15, 2.3]:
    r = t.update(stamp, [detection('green')])
    assert r['state'] == 'unknown' and not r['tracks'][0]['seen_red']


def test_fresh_green_without_red_history_does_not_grant_transition(monkeypatch):
  t = tracker()
  monkeypatch.setattr(module, 'detect', lambda *args, **kwargs: [detection('green')])
  for stamp in [1., 1.15, 1.3, 1.45]:
    r = t.process(picture('green'), stamp)
    assert r['state'] == 'unknown'


def test_validation_is_not_enabled_in_legacy_offline_replays():
  assert not module.SignalTracker().verified_lamps
