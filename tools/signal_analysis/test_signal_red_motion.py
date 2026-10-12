import cv2
import numpy as np
import pytest

import signal_tracker as module


@pytest.mark.parametrize('verified,color,same_id', [(True, 'red', True), (False, 'red', False), (True, 'green', False)])
def test_pending_small_red_follows_unique_current_image_after_vertical_motion(monkeypatch, verified, color, same_id):
  tracker = module.SignalTracker(robust_tracking=True, verified_lamps=verified)
  box = [60, 40, 72, 46]
  detection = {'box': box, 'raw': 'red', 'quality': .8, 'source': 'night_lamp_core', 'core_diameter': 3}
  frame = np.zeros((100, 160, 3), dtype=np.uint8)
  rng = np.random.default_rng(41)
  frame[34:52, 54:78] = rng.integers(0, 150, (18, 24, 3), dtype=np.uint8)
  monkeypatch.setattr(module, 'detect', lambda *a, **kw: [dict(detection)])
  tracker.process(frame, 0.)
  first = tracker.process(frame, .1)['tracks'][0]
  assert not first['seen_red']  # Only two observations so far.
  monkeypatch.setattr(module, 'detect', lambda *a, **kw: [])
  monkeypatch.setattr(module, 'lamp_evidence', lambda *a, **kw: (color, .9, {}))
  moved = cv2.warpAffine(frame, np.float32([[1, 0, 0], [0, 1, 7]]), (160, 100))
  result = tracker.process(moved, .2)
  fresh = [t for t in result['tracks'] if t['age'] == 0]
  retained = [t for t in fresh if t['id'] == first['id']]
  assert bool(retained) == same_id
  if same_id:
    assert retained[0]['box'] == [60, 47, 72, 53]
    assert retained[0]['state'] == 'red'
  else:
    assert result['state'] != 'green'


@pytest.mark.parametrize('score,verified,raw,gap', [(.89, True, 'red', .1), (.99, False, 'red', .1),
                                                   (.99, True, 'green', .1), (.99, True, 'red', .3)])
def test_no_overlap_reassociation_rejects_weak_unverified_green_or_late_match(score, verified, raw, gap):
  tracker = module.SignalTracker(robust_tracking=True, verified_lamps=verified)
  for t in (0., .1, .2):
    tracker.update(t, [{'box': [60, 40, 72, 46], 'raw': 'red', 'quality': .8, 'source': 'night_lamp_core'}])
  result = tracker.update(.2+gap, [{'box': [60, 47, 72, 53], 'raw': raw, 'quality': .9, 'source': 'causal_template',
                                      'continuity_id': 1, 'red_motion_verified': True, 'match_score': score}])
  assert all(t['id'] != 1 for t in result['tracks'] if t['age'] == 0)


def test_ambiguous_image_match_cannot_bridge_nonoverlapping_small_lamp(monkeypatch):
  tracker = module.SignalTracker(robust_tracking=True, verified_lamps=True)
  for t in (0., .1):
    tracker.update(t, [{'box': [60, 40, 72, 46], 'raw': 'red', 'quality': .8, 'source': 'night_lamp_core'}])
  tracker.previous_gray = np.random.default_rng(3).integers(0, 150, (100, 160), dtype=np.uint8)
  monkeypatch.setattr(module, 'detect', lambda *a, **kw: [])
  monkeypatch.setattr(module, 'lamp_evidence', lambda *a, **kw: ('red', .9, {}))
  scores = np.full((21, 21), -.5, np.float32)
  scores[17, 10] = .99
  scores[7, 10] = .97  # Two near-equal matches to neighboring lamps.
  monkeypatch.setattr(cv2, 'matchTemplate', lambda *a, **kw: scores.copy())
  result = tracker.process(np.zeros((100, 160, 3), np.uint8), .2)
  assert all(t['id'] != 1 for t in result['tracks'] if t['age'] == 0)
