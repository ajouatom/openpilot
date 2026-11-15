"""CameraSetSpeedSync: the camera's stock set speed follows openpilot's so a device drop-out keeps the speed."""
from types import SimpleNamespace
from opendbc.car.hyundai.hyundaicanfd import CameraSetSpeedSync

PRESS = 12  # inside the 10 < frame % 30 <= 16 pulse window
IDLE = 20


def test_presses_toward_the_target_only_inside_the_pulse_window():
  s = CameraSetSpeedSync()
  assert s.button(87.4, 80, PRESS) == 1  # + (RES_ACCEL)
  assert s.button(87.4, 81, IDLE) == 0
  assert s.button(70, 81, PRESS) == 2  # - (SET_DECEL) for a new target


def test_equal_or_unknown_camera_speed_sends_nothing():
  s = CameraSetSpeedSync()
  assert s.button(87, 87, PRESS) == 0
  assert s.button(87, 0, PRESS) == 0


def test_overshoot_stops_for_that_target_but_not_the_next():
  s = CameraSetSpeedSync()
  assert s.button(87, 85, PRESS) == 1
  assert s.button(87, 90, PRESS) == 0  # stepped past (5/10 km/h steps): do not toggle
  assert s.button(87, 90, PRESS) == 0
  assert s.button(95, 90, PRESS) == 1


def test_reaching_the_target_resets_the_direction():
  s = CameraSetSpeedSync()
  assert s.button(87, 86, PRESS) == 1
  assert s.button(87, 87, PRESS) == 0
  assert s.button(87, 88, PRESS) == 2  # camera moved later (e.g. driver): approach again, not an overshoot


def test_target_is_clamped_to_the_stock_range():
  s = CameraSetSpeedSync()
  assert s.button(20, 30, PRESS) == 0
  assert s.button(250, 180, PRESS) == 0


GAP_PRESS = 112  # inside the 10 < frame % 100 <= 16 window (one press per second)


def test_gap_presses_until_the_camera_matches_openpilot():
  from opendbc.car.hyundai.hyundaicanfd import CameraGapSync
  s = CameraGapSync()
  assert s.button(1, 4, GAP_PRESS) == 3  # GAP_DIST: 11-08/09 logs had openpilot 1 with the camera at 4
  assert s.button(1, 4, GAP_PRESS + 50) == 0  # outside the window
  assert s.button(1, 1, GAP_PRESS + 100) == 0
  assert s.button(1, 0, GAP_PRESS) == 0  # camera gap unknown


def test_gap_sync_gives_up_when_the_camera_does_not_follow():
  from opendbc.car.hyundai.hyundaicanfd import CameraGapSync
  s = CameraGapSync()
  sent = sum(s.button(2, 4, GAP_PRESS + 100 * i) == 3 for i in range(20))
  assert sent == CameraGapSync.MAX_PRESSES
  assert s.button(3, 4, GAP_PRESS + 3000) == 3  # a new target tries again


def test_jump_guard_waits_while_faster_than_the_camera():
  # faster than VSetDis, a stock press SETS VSetDis to the current speed (route 89 seg20: 81 -> 127)
  s = CameraSetSpeedSync()
  assert s.button(100, 80, PRESS, v_ego=95) == 1    # + jump lands at 95, still under the target: fine
  s = CameraSetSpeedSync()
  assert s.button(100, 80, PRESS, v_ego=105) == 0 and s.busy  # + jump would pass 100: wait
  assert s.button(100, 80, PRESS, v_ego=100) == 1   # back under the target (+1 rounding): press
  s = CameraSetSpeedSync()
  assert s.button(70, 80, PRESS, v_ego=85) == 0 and s.busy    # - would jump UP to 85: wait
  assert s.button(70, 80, PRESS, v_ego=80) == 2


def test_busy_only_while_working_toward_a_valid_target():
  s = CameraSetSpeedSync()
  assert s.button(87, 80, IDLE) == 0 and s.busy       # outside the press window but not there yet
  assert s.button(87, 87, PRESS) == 0 and not s.busy
  assert s.button(87, 255, PRESS) == 0 and not s.busy  # 255 = MAIN off: no valid set speed
  assert s.button(87, 85, PRESS) == 1 and s.busy
  assert s.button(87, 90, PRESS) == 0 and not s.busy   # overshoot: done for this target


def _cs(acc_mode=1, vset=80, gap=2, kph=80.):
  return SimpleNamespace(scc_control={"VSetDis": vset, "DISTANCE_SETTING": gap}, ACCMode=acc_mode, is_metric=True,
                         out=SimpleNamespace(vEgo=kph / 3.6, vEgoCluster=kph / 3.6))


def _hud(kph=90., bars=3):
  return SimpleNamespace(setSpeed=kph / 3.6, leadDistanceBars=bars)


def _sync(cs, hud, frame, quiet=True):
  from opendbc.car.hyundai.hyundaicanfd import CameraGapSync, _camera_sync_button
  return _camera_sync_button(CameraSetSpeedSync(), CameraGapSync(), cs, hud, frame, quiet)


def test_sync_runs_only_while_the_stock_scc_plainly_cruises():
  assert _sync(_cs(), _hud(), PRESS) == 1
  assert _sync(_cs(acc_mode=2), _hud(), PRESS) == 0  # gas override: a press would set VSetDis to the current speed
  assert _sync(_cs(acc_mode=3), _hud(), PRESS) == 0
  assert _sync(_cs(vset=255), _hud(), PRESS) == 0    # MAIN off
  assert _sync(_cs(kph=2.), _hud(), PRESS) == 0      # standstill: + would resume from hold
  assert _sync(_cs(), _hud(), PRESS, quiet=False) == 0
  cs = _cs()
  cs.scc_control = None
  assert _sync(cs, _hud(), PRESS) == 0


def test_gap_waits_until_the_set_speed_is_done():
  from opendbc.car.hyundai.hyundaicanfd import CameraGapSync, _camera_sync_button
  speed, gap = CameraSetSpeedSync(), CameraGapSync()
  assert GAP_PRESS % 30 > 16  # a frame where the speed sync has no press of its own
  assert _camera_sync_button(speed, gap, _cs(vset=80, gap=2), _hud(90, 3), GAP_PRESS, True) == 0  # speed still busy
  assert _camera_sync_button(speed, gap, _cs(vset=90, gap=2), _hud(90, 3), GAP_PRESS, True) == 3  # now the gap
  speed, gap = CameraSetSpeedSync(), CameraGapSync()
  # waiting on the jump guard also holds the gap back
  assert _camera_sync_button(speed, gap, _cs(vset=80, gap=2, kph=100), _hud(90, 3), GAP_PRESS, True) == 0


def test_driver_press_holds_the_sync_for_a_second(monkeypatch):
  from opendbc.car import structs
  from opendbc.can import CANPacker
  from opendbc.car.hyundai import hyundaicanfd
  from opendbc.car.hyundai.values import HyundaiFlags
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 1, get=lambda key: '0'))
  for attr in ('_set_speed_sync', '_gap_sync', '_driver_press_frame', '_lane_line_check'):
    monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, attr, raising=False)
  seen = []
  monkeypatch.setattr(hyundaicanfd, '_camera_sync_button', lambda s, g, cs, hud, frame, quiet: seen.append((frame, quiet)) or 0)
  cs = SimpleNamespace(out=structs.CarState(vEgo=22.), modelV2=None, lfahda_cluster=None, cruise_buttons_alt2=None,
                       cruise_buttons_msg={"LFA_BTN": 0, "CRUISE_BUTTONS": 0}, cruise_btns_msg_canfd="CRUISE_BUTTONS",
                       scc_control={"InfoDisplay": 0, "VSetDis": 80, "DISTANCE_SETTING": 2}, MainMode_ACC=True,
                       ACCMode=1, adrv_0x1ea=None, is_metric=True)
  cc = structs.CarControl(enabled=True)

  def at(frame, pressed=0):
    cs.cruise_buttons_msg["CRUISE_BUTTONS"] = pressed
    hyundaicanfd.create_ccnc_messages(SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), CANPacker('hyundai_canfd_generated'),
                                      SimpleNamespace(ECAN=0, CAM=2), frame, cc, cs, cc.hudControl, 0, False, False, 0,
                                      False, 0, 0)

  at(2)
  assert seen[-1] == (2, True)                 # nobody pressed yet
  at(4, pressed=3)                            # driver GAP: not quiet, so no sync press on this frame
  assert seen[-1] == (4, False)
  at(102)
  assert seen[-1] == (102, False)              # 0.98 s after the press: still waiting
  at(106)
  assert seen[-1] == (106, True)               # over 1 s: sync again
