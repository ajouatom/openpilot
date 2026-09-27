from opendbc.car.hyundai.navi_speedcam import (
  CLASS_BOX, CLASS_HARD, CLASS_MOBILE_ZONE, SpeedcamPolicy, decode_camera_profile,
)


def _update(policy, distance, active=True, speed=60, accel=False, mobile_decel=False, skip_box=False, skip_mobile=False):
  return policy.update(active, speed, distance, accel, mobile_decel, skip_box, skip_mobile)


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
  assert policy.warning_class == CLASS_HARD


def test_precise_fixed_preview_upgrades_a_running_mobile_zone():
  policy = SpeedcamPolicy()
  policy.add_preview(0xD3, 361, 0.0)
  _update(policy, 0.0)
  policy.add_preview(0xD0, 200, 100.0)
  assert _update(policy, 100.0) == (False, False)
  assert policy.warning_class == CLASS_HARD


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
