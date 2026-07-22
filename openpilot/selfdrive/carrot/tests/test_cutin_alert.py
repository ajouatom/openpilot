from openpilot.selfdrive.controls.lib.cutin_alert import (
  CutinAlertCandidate,
  CutinAlertTracker,
  StationaryLeadAlertTracker,
  is_stationary_lead_alert_candidate,
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


def test_stationary_alert_candidate_requires_moving_ego_and_radar() -> None:
  base = dict(status=True, radar=True, d_rel=80.0, v_lead=0.2, v_ego=20.0)

  assert is_stationary_lead_alert_candidate(**base)
  assert not is_stationary_lead_alert_candidate(**(base | {"radar": False}))
  assert not is_stationary_lead_alert_candidate(**(base | {"v_ego": 3.9}))
  assert not is_stationary_lead_alert_candidate(**(base | {"v_lead": 1.0}))
  assert not is_stationary_lead_alert_candidate(**(base | {"d_rel": 140.0}))


def test_stationary_alert_ignores_short_dropout_and_realerts_after_clear() -> None:
  tracker = StationaryLeadAlertTracker(dropout_frames=3)
  lead = candidate(40, 80.0, 0.2, 0.2)

  assert tracker.update((lead,))
  assert not tracker.update((candidate(40, 79.0, 0.2, 0.2),))
  assert not tracker.update(())
  assert not tracker.update((candidate(41, 78.5, 0.1, 0.1),))
  assert not tracker.update(())
  assert not tracker.update(())
  assert not tracker.update(())
  assert tracker.update((candidate(50, 60.0, 0.0, 0.0),))


def test_stationary_alert_resets_when_disabled() -> None:
  tracker = StationaryLeadAlertTracker()
  lead = candidate(40, 80.0, 0.2, 0.2)

  assert tracker.update((lead,))
  assert not tracker.update((lead,), enabled=False)
  assert tracker.update((lead,))
