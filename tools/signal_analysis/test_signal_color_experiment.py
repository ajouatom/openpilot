"""Color-only experiment: suppress blue evidence without granting stale green."""
from unittest.mock import patch

import cv2
import numpy as np
import pytest

from tools.signal_analysis import signal_tracker as module
from tools.signal_analysis import test_signal_tracker as fixtures


def lamp(hue):
  rgb = np.full((80, 160, 3), 120, np.uint8)
  rgb[30:50, 40:120] = 10
  color = cv2.cvtColor(np.array([[[hue, 200, 240]]], np.uint8), cv2.COLOR_HSV2RGB)[0, 0]
  cv2.circle(rgb, (110, 40), 5, tuple(map(int, color)), -1)
  return rgb, [40, 30, 120, 50]


@pytest.mark.parametrize('hue', [94, 100, 104])
def test_blue_dominant_pixels_cannot_be_green(hue):
  rgb, box = lamp(hue)
  assert module.lamp_evidence(rgb, box)[0] == 'green'
  assert module.lamp_evidence(rgb, box, conservative_green=True)[0] == 'unknown'


@pytest.mark.parametrize('hue', [45, 60, 80, 90])
def test_green_and_cyan_lamp_survive(hue):
  rgb, box = lamp(hue)
  assert module.lamp_evidence(rgb, box, conservative_green=True)[0] == 'green'


@pytest.mark.parametrize('color', [(255, 0, 0), (0, 255, 0), (0, 255, 255)])
def test_white_center_night_halo_survives(color):
  frame = fixtures.TestNightProposals.frame(color=color)
  old = module.night_proposals(frame)
  assert old
  assert old == module.night_proposals(frame, conservative_green=True)


def test_day_red_to_green_retains_confirmation():
  tracker = module.SignalTracker(daytime_cores=True, robust_tracking=True, conservative_green=True)
  for timestamp, color in zip([1., 1.2, 1.4, 1.6, 1.8, 2.], ['red']*3+['green']*3):
    result = tracker.process(fixtures.TestDayProposals.frame(color), timestamp)
    if timestamp == 1.6:
      assert result['state'] == 'unknown'
  assert result['state'] == 'green'


def test_template_path_uses_same_filter_and_unknown_never_confirms_green():
  rgb, box = lamp(100)
  tracker = module.SignalTracker(conservative_green=True)
  for t in [1., 1.1, 1.2]:
    tracker.update(t, [dict(box=box, raw='red', quality=.8, source='test')])
  tracker.previous_gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
  with patch.object(module, 'detect', return_value=[]):
    for t in [1.3, 1.4, 1.5, 1.6]:
      result = tracker.process(rgb, t)
      fresh = [v for v in result['tracks'] if v['age'] == 0]
      assert fresh and fresh[0]['evidence']['source'] == 'causal_template'
      assert fresh[0]['evidence']['raw'] == 'unknown'
      assert result['state'] != 'green'


def test_experiment_is_disabled_by_default():
  assert not module.SignalTracker().conservative_green
