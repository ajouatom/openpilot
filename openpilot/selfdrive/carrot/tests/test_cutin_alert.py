from pathlib import Path

from openpilot.selfdrive.controls.lib.cutin_alert import (
  CutinAlertCandidate,
  CutinAlertTracker,
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


def test_cutin_alert_waits_for_path_occupancy_without_repeating_on_flicker() -> None:
  tracker = CutinAlertTracker()
  lead = candidate(10, 20.0, 2.0, -1.0)

  assert not tracker.update((lead,), alert_candidates=())
  assert tracker.update((lead,), alert_candidates=(lead,))
  assert not tracker.update((lead,), alert_candidates=())
  assert not tracker.update((lead,), alert_candidates=(lead,))


def test_vehicle_audio_uses_only_path_occupied_cutins() -> None:
  source = (
    Path(__file__).resolve().parents[2]
    / "selfdrived"
    / "selfdrived.py"
  ).read_text(encoding="utf-8")

  assert "*self.sm['radarState'].leadsCutIn," in source
  assert "*self.sm['radarState'].leadsCutInPath," in source
  assert "for lead in self.sm['radarState'].leadsCutInPath\n" in source
  assert "alert_candidates=path_cutin_candidates" in source
