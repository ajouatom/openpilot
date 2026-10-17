import copy
import math

import pytest

from tools.signal_analysis.signal_central_selection import CentralSignalSelector, RoadSignalRegion
from tools.signal_analysis.test_signal_reference_geometry import model


def track(ident, box, color='red', seen_red=True):
  return dict(id=ident, box=box, age=0., observations=10, state=color if seen_red else 'unknown',
              seen_red=seen_red, evidence=dict(raw=color))


def observation(stamp, tracks, session='a'):
  return dict(timestamp_eof=round(stamp*1e9), frame_id=round(stamp*20), tracks=tracks, session=session)


def warm(selector, tracks, region=None):
  for stamp in (1., 1.15, 1.3, 1.45):
    result = selector.update(observation(stamp, tracks), region)
  return result


def scene():
  return [track(1, [608, 292, 644, 303]), track(2, [676, 294, 709, 305]),
          track(3, [853, 202, 893, 222], 'green', False)]


def test_default_has_no_image_fallback():
  assert not warm(CentralSignalSelector(), scene()).suppressed_ids


def test_stable_central_group_removes_only_separate_green_first_without_mutation():
  tracks = scene()
  original = copy.deepcopy(tracks)
  selector = CentralSignalSelector(image_fallback=True)
  assert not selector.update(observation(1., tracks), None).suppressed_ids
  result = warm(selector, tracks)
  assert result.suppressed_ids == (3,)
  assert result.selected_ids == (1, 2)
  assert result.anchor_ids == (1, 2)
  assert tracks == original


@pytest.mark.parametrize('color,seen', [('red', False), ('red', True), ('green', True), ('unknown', True)])
def test_red_or_red_history_never_removed_even_far_outside_group(color, seen):
  tracks = scene()
  tracks[-1] = track(3, [1100, 200, 1140, 220], color, seen)
  assert not warm(CentralSignalSelector(image_fallback=True), tracks).suppressed_ids


def test_multiple_central_green_heads_kept_and_green_first_cannot_anchor():
  tracks = [track(1, [600, 220, 640, 240], 'green', False),
            track(2, [700, 220, 740, 240], 'green', False), scene()[-1]]
  assert not warm(CentralSignalSelector(image_fallback=True), tracks).anchor_ids
  tracks[0].update(seen_red=True, state='green')
  result = warm(CentralSignalSelector(image_fallback=True), tracks)
  assert result.selected_ids == (1, 2)


@pytest.mark.parametrize('change', ['gap', 'session', 'reverse', 'missing', 'stale'])
def test_selection_does_not_survive_lost_or_discontinuous_anchor(change):
  selector = CentralSignalSelector(image_fallback=True)
  tracks = scene()
  warm(selector, tracks)
  stamp, session = 1.6, 'a'
  if change == 'gap': stamp = 1.9
  if change == 'session': session = 'b'
  if change == 'reverse': stamp = 1.2
  if change == 'missing': tracks = tracks[-1:]
  if change == 'stale':
    for t in tracks[:-1]: t['age'] = .1
  assert not selector.update(observation(stamp, tracks, session), None).suppressed_ids


def test_dark_current_boxes_have_bounded_memory_without_color_injection():
  selector = CentralSignalSelector(image_fallback=True)
  tracks = scene()
  warm(selector, tracks)
  for t in tracks[:-1]:
    t['state'], t['evidence']['raw'] = 'unknown', 'unknown'
  for stamp in (1.6, 1.75, 1.9):
    assert selector.update(observation(stamp, tracks), None).suppressed_ids == (3,)
  assert not selector.update(observation(2.05, tracks), None).suppressed_ids
  assert tracks[0]['evidence']['raw'] == 'unknown'


def test_duplicate_observation_does_not_renew_confirmation():
  selector = CentralSignalSelector(image_fallback=True)
  first = observation(1., scene())
  for _ in range(50):
    assert not selector.update(first, None).suppressed_ids
  assert not selector.update(observation(1.4, scene()), None).suppressed_ids


def test_unsafe_all_colour_comparator_is_explicit_and_separate():
  tracks = scene()
  tracks[-1] = track(3, [1100, 200, 1140, 220], 'red')
  assert warm(CentralSignalSelector(image_fallback=True, all_colours_trial=True), tracks).suppressed_ids == (3,)
  assert not warm(CentralSignalSelector(image_fallback=True), tracks).suppressed_ids


def test_projected_road_center_moves_with_curved_geometry():
  m = model()
  region = RoadSignalRegion(m, [0., 0., 0.])
  assert region.classify([662, 240, 682, 250]) == 'central'
  assert region.classify([1100, 240, 1120, 250]) == 'peripheral'
  assert region.classify([662, 700, 682, 710]) == 'unknown'
  moved = copy.deepcopy(m)
  for c in moved.laneLines:
    c.y = [y+8 for y in c.y]
  assert RoadSignalRegion(moved, [0., 0., 0.]).classify([662, 240, 682, 250]) != 'central'


@pytest.mark.parametrize('rpy', [[math.nan, 0, 0], [0, 0], [0, 0, math.inf]])
def test_invalid_projection_never_excludes(rpy):
  region = RoadSignalRegion(model(), rpy)
  assert region.classify([1000, 200, 1040, 220]) == 'unknown'


def test_uncertain_geometry_never_supplies_road_selection():
  m = model()
  m.laneLineProbs = [.1]*4
  m.roadEdgeStds = [2., 2.]
  region = RoadSignalRegion(m, [0., 0., 0.])
  assert not region.ready
  assert not warm(CentralSignalSelector(), scene(), region).suppressed_ids


@pytest.mark.parametrize('pair', [([481, 210, 521, 230], [348, 212, 380, 228]),
                                ([695, 210, 739, 232], [803, 210, 843, 230]),
                                ([747, 150, 803, 178], [891, 154, 943, 180])])
def test_near_pair_is_not_discarded_for_a_far_central_red(pair):
  tracks = [track(1, [626, 377, 642, 385]),
            track(2, pair[0], 'green', False), track(3, pair[1], 'green', False)]
  assert not warm(CentralSignalSelector(image_fallback=True), tracks).suppressed_ids


def test_overlapping_green_duplicates_do_not_protect_a_side_object():
  tracks = scene()+[track(4, [852, 202, 892, 222], 'green', False)]
  assert warm(CentralSignalSelector(image_fallback=True), tracks).suppressed_ids == (3, 4)


def test_replay_preserves_original_first_twenty_transport_limit():
  from tools.signal_analysis.replay_signal_central import start_observation
  tracks = [track(i, [600, 200, 640, 220]) for i in range(1, 23)]
  result = start_observation(observation(1., tracks), suppressed=(1, 2))
  assert [t['id'] for t in result['tracks']] == list(range(3, 21))


def test_replay_never_uses_future_context():
  from tools.signal_analysis.replay_signal_central import Timeline
  timeline = Timeline({'modelV2': [(1., 'past'), (1.2, 'future')]})
  assert timeline.latest('modelV2', 1.1, .25) == 'past'
  assert timeline.latest('modelV2', .9, .25) is None
  assert timeline.latest('modelV2', 1.5, .25) is None
