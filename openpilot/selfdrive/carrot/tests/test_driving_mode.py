from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.driving_mode import DrivingMode, DrivingModeDetector, get_mode_lead_response


def advance(detector, seconds, *, ego=0., lead_speed=0., distance=8., status=True,
            track=42, valid=True, dt=.05, relative_speed=None, lead_accel=0.):
  car = NS(vEgo=ego / 3.6)
  lead = NS(status=status, vLead=lead_speed / 3.6, dRel=distance,
            vRel=(lead_speed - ego) / 3.6 if relative_speed is None else relative_speed,
            radar=True, radarTrackId=track, aLeadK=lead_accel)
  for _ in range(round(seconds / dt)):
    detector.update_data(car, lead, valid=valid, dt=dt)
  return detector.congested


@pytest.mark.parametrize('mode,ceiling', [(DrivingMode.Eco, 2), (DrivingMode.Safe, 3),
                                         (DrivingMode.Normal, 5), (DrivingMode.High, 5)])
@pytest.mark.parametrize('requested', range(6))
def test_mode_never_increases_or_enables_user_response(mode, ceiling, requested):
  assert get_mode_lead_response(requested, mode) == min(requested, ceiling)


@pytest.mark.parametrize('dt', [.025, .05, .1])
def test_stopped_lead_enters_safe_promptly_at_different_update_rates(dt):
  detector = DrivingModeDetector()
  assert not advance(detector, .2, dt=dt)
  assert advance(detector, .2, dt=dt)


def test_stopped_lead_gets_safe_before_close_range_at_road_speed():
  assert advance(DrivingModeDetector(), .4, ego=80, distance=100)
  assert not advance(DrivingModeDetector(), 10., ego=5, distance=100)


def test_moving_queue_enters_without_a_full_stop():
  detector = DrivingModeDetector()
  assert not advance(detector, 7., ego=20, lead_speed=20, distance=25)
  assert advance(detector, 1.2, ego=20, lead_speed=20, distance=25)


def test_separate_slow_fragments_do_not_accumulate_into_a_queue():
  detector = DrivingModeDetector()
  for _ in range(4):
    assert not advance(detector, 4., ego=20, lead_speed=20, distance=25)
    assert not advance(detector, 1., ego=50, lead_speed=50, distance=35)


def test_repeated_short_launches_preserve_safe():
  detector = DrivingModeDetector()
  assert advance(detector, .4)
  for _ in range(8):
    assert advance(detector, 3., ego=20, lead_speed=30, distance=35)
    assert advance(detector, 2., ego=10, lead_speed=8, distance=16)
    assert advance(detector, .5)


@pytest.mark.parametrize('dt', [.025, .05, .1])
@pytest.mark.parametrize('automatic,base_mode', [(1, DrivingMode.Normal), (2, DrivingMode.Eco)])
def test_strong_lead_acceleration_releases_safe_before_flow_recovery(dt, automatic, base_mode):
  detector = DrivingModeDetector()
  advance(detector, .4, dt=dt)
  assert advance(detector, .4, ego=10, lead_speed=10, distance=16, lead_accel=1.6, dt=dt)
  assert not advance(detector, .2, ego=10, lead_speed=10, distance=16, lead_accel=1.6, dt=dt)
  assert detector.get_mode(automatic) == base_mode
  assert not advance(detector, 2., ego=10, lead_speed=10, distance=16, lead_accel=1.6, dt=dt)
  assert advance(detector, .4, dt=dt)


def test_acceleration_threshold_is_strict_and_spikes_do_not_accumulate():
  detector = DrivingModeDetector()
  advance(detector, .4)
  assert advance(detector, 2., ego=10, lead_speed=10, distance=16, lead_accel=1.5)
  for _ in range(4):
    assert advance(detector, .4, ego=10, lead_speed=10, distance=16, lead_accel=2.)
    assert advance(detector, .1, ego=10, lead_speed=10, distance=16, lead_accel=1.5)


@pytest.mark.parametrize('interruption', [{'valid': False}, {'status': False}, {'track': 43},
                                         {'lead_accel': float('nan')}, {'lead_speed': 0.}])
def test_acceleration_release_restarts_after_interrupted_evidence(interruption):
  detector = DrivingModeDetector()
  advance(detector, .4)
  sample = {'ego': 10, 'lead_speed': 10, 'distance': 8, 'lead_accel': 2.}
  assert advance(detector, .4, **sample)
  assert advance(detector, .1, **(sample | interruption))
  assert advance(detector, .4, **sample)
  assert not advance(detector, .2, **sample)


def test_stopping_approach_keeps_safe_even_with_high_lead_acceleration():
  detector = DrivingModeDetector()
  assert advance(detector, 2., ego=20, lead_speed=5, distance=8, lead_accel=2.)


@pytest.mark.parametrize('ego,lead_speed,distance', [(45, 45, 30), (20, 30, 35)])
def test_sustained_flow_or_opening_gap_releases_safe(ego, lead_speed, distance):
  detector = DrivingModeDetector()
  advance(detector, .4)
  assert advance(detector, 5., ego=ego, lead_speed=lead_speed, distance=distance)
  assert not advance(detector, 1.2, ego=ego, lead_speed=lead_speed, distance=distance)


def test_lead_change_resets_release_evidence_but_preserves_queue():
  detector = DrivingModeDetector()
  advance(detector, .4)
  for track in range(5):
    assert advance(detector, 4., ego=20, lead_speed=30, distance=35, track=track)
  assert not advance(detector, 2.2, ego=20, lead_speed=30, distance=35, track=4)


def test_brief_lead_loss_and_stopped_lead_loss_do_not_clear_queue():
  detector = DrivingModeDetector()
  advance(detector, .4)
  assert advance(detector, 20., status=False)
  assert advance(detector, 2., ego=25, status=False)
  assert advance(detector, .2, ego=25, lead_speed=25, distance=20)
  assert advance(detector, 3., ego=25, status=False)
  assert not advance(detector, 1.2, ego=25, status=False)


def test_stale_or_invalid_data_neither_enters_nor_releases_safe():
  detector = DrivingModeDetector()
  assert not advance(detector, 10., valid=False)
  advance(detector, .4)
  assert advance(detector, 20., ego=60, status=False, valid=False)
  assert advance(detector, 20., ego=60, lead_speed=60, distance=float('nan'))
  assert advance(detector, 5., ego=60, lead_speed=60, distance=35)
  assert not advance(detector, 1.2, ego=60, lead_speed=60, distance=35)


def test_invalid_ego_speed_and_large_time_gap_do_not_count_as_recovery():
  detector = DrivingModeDetector()
  advance(detector, .4)
  assert advance(detector, 10., ego=float('nan'), status=False)
  assert advance(detector, 10., ego=60, status=False, dt=1.)


def test_open_road_never_enters_safe():
  detector = DrivingModeDetector()
  assert not advance(detector, 60., ego=80, status=False)


def test_braking_wave_resets_recovery_even_above_35_kph():
  detector = DrivingModeDetector()
  advance(detector, .4)
  assert advance(detector, 5., ego=45, lead_speed=45, distance=35)
  assert advance(detector, 1., ego=45, lead_speed=45, distance=35, lead_accel=-1.)
  assert advance(detector, 5., ego=45, lead_speed=45, distance=35)
  assert not advance(detector, 1.2, ego=45, lead_speed=45, distance=35)
