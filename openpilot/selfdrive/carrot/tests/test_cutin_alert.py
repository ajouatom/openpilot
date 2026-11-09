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


def test_cutin_alert_remembers_same_track_for_memory_window() -> None:
  tracker = CutinAlertTracker()

  assert tracker.update((candidate(3752, 45.7, -1.05),), now_s=0.0)
  assert not tracker.update((), now_s=0.05)
  assert not tracker.update((candidate(3752, 46.1, 0.69),), now_s=0.5)
  assert not tracker.update((), now_s=0.6)
  assert tracker.update((candidate(3752, 46.6, 1.44),), now_s=3.7)


def test_cutin_alert_memory_rejects_same_id_far_away() -> None:
  tracker = CutinAlertTracker()

  assert tracker.update((candidate(42, 9.0, -2.5, -1.0),), now_s=0.0)
  assert not tracker.update((), now_s=0.1)
  assert tracker.update((candidate(42, 24.0, 1.8, -6.0),), now_s=1.8)


def test_cutin_alert_memory_does_not_reid_new_vehicle_after_gap() -> None:
  tracker = CutinAlertTracker()

  assert tracker.update((candidate(100, 11.5, -0.9, -0.5),), now_s=1.0)
  assert not tracker.update((), now_s=1.1)
  assert tracker.update((candidate(101, 12.3, -1.5, -0.3),), now_s=2.6)


def test_cutin_alert_memory_survives_disable_until_reset() -> None:
  tracker = CutinAlertTracker()
  lead = candidate(10, 20.0, 1.5)

  assert tracker.update((lead,), now_s=0.0)
  assert not tracker.update((lead,), enabled=False, now_s=0.1)
  assert not tracker.update((lead,), now_s=0.2)
  tracker.reset()
  assert tracker.update((lead,), now_s=0.3)


def test_promoted_cutin_rejects_fast_lateral_crossing() -> None:
  crossing = CutinAlertCandidate(1985, 35.5, -0.39, -1.58, v_lat=-8.7)

  assert promoted_cutin_candidates((crossing,), crossing) == ()


def test_promoted_cutin_rejects_scc_track_zero() -> None:
  scc = candidate(0, 30.0, 0.5, -2.0)

  assert promoted_cutin_candidates((scc,), scc) == ()
