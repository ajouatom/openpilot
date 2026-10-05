from opendbc.car.hyundai.navi_speedcam import (
  CLASS_BOX, CLASS_FIXED, CLASS_HARD, CLASS_MOBILE_ZONE, SpeedcamPolicy, decode_camera_profile,
)


def _update(policy, distance, active=True, speed=60, accel=False, mobile_decel=False, skip_box=False, skip_mobile=False,
            skip_fixed=False, skip_unknown=False):
  return policy.update(active, speed, distance, accel, mobile_decel, skip_box, skip_mobile, skip_fixed, skip_unknown)


def test_decode_uses_low_nine_bits_and_keeps_rear_flag():
  assert decode_camera_profile(0xD0, 176) == (0, 60, False)
  assert decode_camera_profile(0xD3, 346) == (3, 60, False)
  # Labelled rear signal-and-speed cameras: kind 1 with 0x321 above bit 9.
  assert decode_camera_profile(0x64291, 487) == (1, 40, True)
  assert decode_camera_profile(0x64271, 791) == (1, 30, True)
  # Labelled rear speed camera in a 30 km/h school zone: kind 0 with the same flag.
  assert decode_camera_profile(0x64270, 78) == (0, 30, True)
  assert decode_camera_profile(0x6, 118) is None           # speed bump
  assert decode_camera_profile(0xD7, 0) is None            # kind 7 zone entry
  assert decode_camera_profile(0xFFFFFFFF, 8191) is None


def test_mobile_zone_is_ignored_below_mode_three():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD3, 361, 0.0)                      # kind 3 at the 60 km/h warning lead
  assert _update(policy, 0.0) == (True, False)
  assert policy.warning_class == CLASS_MOBILE_ZONE
  assert _update(policy, 0.0, mobile_decel=True) == (False, False)


def test_saturated_preview_matches_only_at_warning_lead():
  # 17:55:19 drive: a mobile-zone preview at the lead and a fixed camera ~1.9 km ahead.
  policy = SpeedcamPolicy()
  policy.add_preview(0xD3, 1995, -1634.0)                 # saturated, lands 361 m ahead
  policy.add_preview(0xD0, 1996, -88.0)                   # saturated, 1908 m ahead
  _update(policy, 0.0)
  assert policy.warning_class == CLASS_MOBILE_ZONE


def test_fixed_camera_inside_mobile_zone_always_decelerates():
  # 18:54 drive: fixed 364 m and mobile zone 398 m ahead of the same 60 km/h warning.
  policy = SpeedcamPolicy()
  policy.add_preview(0xD0, 1682, -1318.0)
  policy.add_preview(0xD3, 1714, -1316.0)
  assert _update(policy, 0.0, accel=True, skip_mobile=True, skip_box=True) == (False, False)
  assert policy.warning_class == CLASS_FIXED


def test_precise_fixed_preview_upgrades_a_running_mobile_zone():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD3, 361, 0.0)
  _update(policy, 0.0)
  policy.add_preview(0xD0, 200, 100.0)
  assert _update(policy, 100.0) == (False, False)
  assert policy.warning_class == CLASS_FIXED


def test_unknown_warning_is_hard():
  policy = SpeedcamPolicy()
  assert _update(policy, 0.0, accel=True, skip_box=True, skip_mobile=True) == (False, False)


def test_rear_flag_is_hard_even_with_box_kind():
  policy = SpeedcamPolicy()
  policy.add_preview(0x642D2, 366, 0.0)                   # kind 2 @60 with the rear flag
  assert _update(policy, 0.0, accel=True, skip_box=True) == (False, False)
  assert policy.warning_class == CLASS_HARD


def test_box_skip_needs_its_toggle_and_consumes_one_press():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD2, 366, 0.0)
  assert _update(policy, 0.0, accel=True) == (False, False)
  assert policy.warning_class == CLASS_BOX
  assert _update(policy, 10.0, accel=True, skip_box=True) == (True, True)
  assert _update(policy, 20.0, accel=True, skip_box=True) == (True, False)
  # The skip ends with the warning.
  assert _update(policy, 30.0, active=False, skip_box=True) == (False, False)
  policy.add_preview(0xD2, 366, 400.0)
  assert _update(policy, 400.0, skip_box=True) == (False, False)


def test_mobile_zone_skip_applies_only_when_mode_three_decelerates():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD3, 361, 0.0)
  assert _update(policy, 0.0, mobile_decel=True) == (False, False)
  assert _update(policy, 5.0, accel=True, mobile_decel=True, skip_mobile=True) == (True, True)


def test_speed_change_starts_a_new_warning():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD3, 361, 0.0)
  _update(policy, 0.0, mobile_decel=True, skip_mobile=True, accel=True)
  assert policy.skipped
  policy.add_preview(0xB1, 305, 300.0)                   # 50 km/h signal camera
  assert _update(policy, 300.0, speed=50, mobile_decel=True, skip_mobile=True, accel=True) == (False, False)
  assert not policy.skipped and policy.warning_class == CLASS_HARD


def test_late_mobile_zone_preview_at_the_start_lead_still_counts():
  # 00000462 / 00000473 (2026-09): the zone preview arrived 7-9 m after the warning began,
  # its target still one lead (366 m @60) ahead of the warning start.
  policy = SpeedcamPolicy()
  assert _update(policy, 0.0) == (False, False)           # no match yet: hard
  policy.add_preview(0xD3, 347, 8.0)                      # target 355 m = start + lead - 11
  assert _update(policy, 8.0) == (True, False)
  assert policy.warning_class == CLASS_MOBILE_ZONE


def test_unrelated_mobile_zone_cannot_release_a_running_warning():
  # A warning with no preview of its own (e.g. a signal camera) must stay hard even when
  # a same-speed mobile-zone preview shows up nearby later in the warning.
  policy = SpeedcamPolicy()
  assert _update(policy, 0.0) == (False, False)
  policy.add_preview(0xD3, 150, 100.0)                    # target 250 m: 116 m off the start lead
  assert _update(policy, 100.0) == (False, False)
  assert policy.warning_class is None                     # still unknown, treated as hard
  policy.add_preview(0xD3, 150, 300.0)                    # target 450 m, also nowhere near the lead
  assert _update(policy, 300.0) == (False, False)


def test_mobile_zone_far_from_the_start_lead_is_ignored_even_at_warning_start():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD3, 200, 0.0)                      # precise, 200 m ahead: inside the old window
  assert _update(policy, 0.0) == (False, False)
  assert policy.warning_class is None


def test_repeated_previews_merge_and_do_not_pile_up_while_stopped():
  policy = SpeedcamPolicy()
  for _ in range(50):                                     # stationary: repeats at the same odometer
    policy.add_preview(0xD3, 361, 0.0)
  assert len(policy.previews) == 1
  policy.add_preview(0xD3, 351, 10.0)                     # same camera 10 m later: same target
  policy.add_preview(0xD0, 361, 10.0)                     # a different kind at that spot stays separate
  assert len(policy.previews) == 2
  assert policy.previews[0]["received"] == 10.0


def test_precise_repeat_replaces_a_horizon_bound_position():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD3, 1995, 0.0)                     # saturated: target only bounded
  policy.add_preview(0xD3, 1980, 10.0)                    # precise repeat, 5 m further
  assert len(policy.previews) == 1
  p = policy.previews[0]
  assert (p["target"], p["saturated"]) == (1990.0, False)
  policy.add_preview(0xD3, 1990, 12.0)                    # a later saturated repeat keeps the precise target
  assert (p["target"], p["saturated"]) == (1990.0, False)


def test_fixed_camera_skip_needs_its_own_toggle():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD0, 366, 0.0)                      # kind 0 fixed @60 at the warning lead
  assert _update(policy, 0.0, accel=True, skip_box=True, skip_mobile=True) == (False, False)
  assert policy.warning_class == CLASS_FIXED
  assert _update(policy, 10.0, accel=True, skip_fixed=True) == (True, True)
  assert _update(policy, 20.0, accel=True, skip_fixed=True) == (True, False)


def test_signal_rear_and_unknown_never_skip_with_the_fixed_toggle():
  signal = SpeedcamPolicy()
  signal.add_preview(0xD1, 366, 0.0)                      # kind 1 signal-and-speed
  signal.add_preview(0xD0, 366, 0.0)                      # with a fixed camera at the same spot
  assert _update(signal, 0.0, accel=True, skip_fixed=True) == (False, False)
  assert signal.warning_class == CLASS_HARD

  rear = SpeedcamPolicy()
  rear.add_preview(0x642D0, 366, 0.0)                     # kind 0 + rear flag
  assert _update(rear, 0.0, accel=True, skip_fixed=True) == (False, False)

  unknown = SpeedcamPolicy()                              # no preview at all
  assert _update(unknown, 0.0, accel=True, skip_fixed=True) == (False, False)


def test_unknown_warning_skip_is_experimental_and_spares_protected_zones():
  # 2026-10-04 field report: a 60 km/h warning with no camera preview stayed hard and uncancellable.
  policy = SpeedcamPolicy()
  assert _update(policy, 0.0, accel=True, skip_fixed=True) == (False, False)      # toggle off: hard
  assert _update(policy, 10.0, accel=True, skip_unknown=True) == (True, True)     # toggle on: one press skips
  assert _update(policy, 20.0, accel=True, skip_unknown=True) == (True, False)

  school = SpeedcamPolicy()                                                       # 30 km/h zones never skip
  assert _update(school, 0.0, speed=30, accel=True, skip_unknown=True) == (False, False)


def test_unknown_skip_ends_when_a_signal_camera_is_matched():
  policy = SpeedcamPolicy()
  assert _update(policy, 0.0, accel=True, skip_unknown=True) == (True, True)
  policy.add_preview(0xD1, 200, 100.0)                                            # signal camera turns up
  assert _update(policy, 100.0, skip_unknown=True) == (False, False)              # decelerates again
  assert policy.warning_class == CLASS_HARD


def test_unknown_skip_does_not_touch_classified_warnings():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD0, 366, 0.0)                                              # fixed camera at the lead
  assert _update(policy, 0.0, accel=True, skip_unknown=True) == (False, False)


def test_early_lead_window_matches_warnings_that_start_early():
  # 2026-10-05 field logs (Gangwon, 60 km/h): the warning started 462 m before a box camera
  # (lead 366 m). Off: unknown (hard). On: the saturated preview at 1.26x lead is the box.
  for early, expected in ((False, None), (True, CLASS_BOX)):
    policy = SpeedcamPolicy()
    policy.early_lead = early
    policy.add_preview(0xD2, 1999, -1537.0)              # saturated, lands 462 m ahead of the warning start
    _update(policy, 0.0)
    assert policy.warning_class == expected


def test_early_lead_also_covers_a_mobile_zone_but_never_a_late_start():
  zone = SpeedcamPolicy()
  zone.early_lead = True
  zone.add_preview(0xD3, 1463, -997.0)                   # kind 3 target 466 m ahead of the start
  _update(zone, 0.0)
  assert zone.warning_class == CLASS_MOBILE_ZONE

  late = SpeedcamPolicy()
  late.early_lead = True
  late.add_preview(0xD2, 1999, -1749.0)                  # 250 m ahead: below the unchanged near end
  _update(late, 0.0)
  assert late.warning_class is None
