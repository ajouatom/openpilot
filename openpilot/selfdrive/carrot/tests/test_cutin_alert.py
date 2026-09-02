from openpilot.selfdrive.controls.lib.cutin_alert import (
  CutinAlertCandidate,
  CutinAlertTracker,
  promoted_cutin_candidates,
)


def candidate(track_id: int, d_rel: float, y_rel: float, v_rel: float = 0.0) -> CutinAlertCandidate:
  return CutinAlertCandidate(track_id, d_rel, y_rel, v_rel)


def test_cutin_alert_only_once_for_continuous_candidate() -> None:
  tracker = CutinAlertTracker()

  assert tracker.update((candidate(10, 20.0, 2.0),))
  assert not tracker.update((candidate(10, 19.7, 1.8),))


def test_cutin_alert_does_not_repeat_for_continuous_reid() -> None:
  tracker = CutinAlertTracker()

  assert tracker.update((candidate(10, 20.0, 2.0, -1.0),))
  assert not tracker.update((candidate(11, 19.5, 1.7, -1.1),))


def test_cutin_alert_repeats_for_new_back_to_back_vehicle() -> None:
  tracker = CutinAlertTracker()

  assert tracker.update((candidate(2816, 38.0, 0.0, 9.45),))
  assert tracker.update((candidate(2818, 9.0, -2.75, 8.85),))


def test_cutin_alert_detects_added_vehicle_while_first_remains() -> None:
  tracker = CutinAlertTracker()
  first = candidate(10, 20.0, 1.5)
  second = candidate(20, 8.0, -2.5)

  assert tracker.update((first,))
  assert tracker.update((first, second))
  assert not tracker.update((first, second))


def test_cutin_alert_resets_when_disabled_or_empty() -> None:
  tracker = CutinAlertTracker()
  lead = candidate(10, 20.0, 1.5)

  assert tracker.update((lead,))
  assert not tracker.update((lead,), enabled=False)
  assert tracker.update((lead,))
  assert not tracker.update(())
  assert tracker.update((lead,))


def test_cutin_alert_waits_until_detected_candidate_is_promoted_to_lead_two() -> None:
  tracker = CutinAlertTracker()
  detected = candidate(2015, 9.8, -0.13, -1.2)

  assert not tracker.update(promoted_cutin_candidates((detected,), None))
  assert tracker.update(promoted_cutin_candidates((detected,), detected))
  assert not tracker.update(promoted_cutin_candidates((detected,), detected))


def test_cutin_alert_ignores_lead_two_that_is_not_the_detected_cutin() -> None:
  detected = candidate(2015, 9.8, -0.13, -1.2)
  other_lead_two = candidate(49, 14.0, 0.0, 0.0)

  assert promoted_cutin_candidates((detected,), other_lead_two) == ()


def test_cutin_promotion_match_rejects_cross_sensor_track_id_collision() -> None:
  detected = candidate(42, 20.0, 2.0, -1.0)
  other_lead_two = candidate(42, 24.0, 0.0, -1.0)

  assert promoted_cutin_candidates((detected,), other_lead_two) == ()
