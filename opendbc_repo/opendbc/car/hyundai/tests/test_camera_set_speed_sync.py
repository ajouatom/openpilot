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


def test_target_outside_the_stock_range_is_left_alone():
  s = CameraSetSpeedSync()
  assert s.button(20, 30, PRESS) == 0 and not s.busy
  assert s.button(250, 180, PRESS) == 0 and not s.busy
  assert s.button(25, 25, PRESS) == 0 and s.button(25, 30, PRESS) == 0  # 25 mph: never pushed up to the 30 floor


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
  assert s.button(100, 80, PRESS, v_ego=95) == 1     # + jump lands at 95, still under the target
  s = CameraSetSpeedSync()
  assert s.button(100, 80, PRESS, v_ego=101) == 0 and s.busy  # + jump would land past 100: wait
  assert s.button(100, 80, PRESS, v_ego=100.4) == 1  # at the target (cluster speed rounds to 100): press
  s = CameraSetSpeedSync()
  assert s.button(70, 80, PRESS, v_ego=81) == 0 and s.busy    # - would jump UP to 81: wait
  assert s.button(70, 80, PRESS, v_ego=80) == 2


def test_busy_only_while_working_toward_a_valid_target():
  s = CameraSetSpeedSync()
  assert s.button(87, 80, IDLE) == 0 and s.busy       # outside the press window but not there yet
  assert s.button(87, 87, PRESS) == 0 and not s.busy
  assert s.button(87, 255, PRESS) == 0 and not s.busy  # 255 = MAIN off: no valid set speed
  assert s.button(87, 85, PRESS) == 1 and s.busy
  assert s.button(87, 90, PRESS) == 0 and not s.busy   # overshoot: done for this target


def _cs(acc_mode=1, vset=80, gap=2, kph=80., set_kph=90.):
  return SimpleNamespace(scc_control={"VSetDis": vset, "DISTANCE_SETTING": gap}, ACCMode=acc_mode, is_metric=True,
                         out=SimpleNamespace(vEgo=kph / 3.6, vEgoCluster=kph / 3.6, vCruiseCluster=set_kph))


def _hud(bars=3, shown_kph=90.):
  return SimpleNamespace(setSpeed=shown_kph / 3.6, leadDistanceBars=bars)


def test_sync_runs_only_while_the_stock_scc_plainly_cruises():
  from opendbc.car.hyundai.hyundaicanfd import CameraSync
  assert CameraSync().button(_cs(), _hud(), PRESS) == 1
  assert CameraSync().button(_cs(acc_mode=2), _hud(), PRESS) == 0  # gas override: a press sets VSetDis to the speed
  assert CameraSync().button(_cs(acc_mode=3), _hud(), PRESS) == 0
  assert CameraSync().button(_cs(vset=255), _hud(), PRESS) == 0    # MAIN off
  assert CameraSync().button(_cs(set_kph=255), _hud(), PRESS) == 0  # no cruise set speed
  assert CameraSync().button(_cs(kph=2.), _hud(), PRESS) == 0      # standstill: + would resume from hold
  cs = _cs()
  cs.scc_control = None
  assert CameraSync().button(cs, _hud(), PRESS) == 0


def test_target_is_the_cruise_set_speed_not_the_displayed_speed():
  from opendbc.car.hyundai.hyundaicanfd import CameraSync
  # at a red light hudControl.setSpeed follows the plan down; the camera keeps the driver's 80
  assert CameraSync().button(_cs(vset=80, gap=3, set_kph=80, kph=60), _hud(bars=3, shown_kph=30), PRESS) == 0


def test_driver_press_holds_the_sync_for_a_second():
  from opendbc.car.hyundai.hyundaicanfd import SYNC_DRIVER_HOLD_FRAMES, CameraSync
  s = CameraSync()
  s.note_driver(PRESS, True)
  assert s.button(_cs(), _hud(), PRESS + 2 * 30) == 0       # 0.6 s later: still waiting
  s.note_driver(PRESS + 2 * 30, False)
  assert s.button(_cs(), _hud(), PRESS + 4 * 30) == 1       # 1.2 s later: sync again
  assert PRESS + 4 * 30 - PRESS > SYNC_DRIVER_HOLD_FRAMES


def test_gap_waits_until_the_set_speed_is_done():
  from opendbc.car.hyundai.hyundaicanfd import CameraSync
  s = CameraSync()
  assert GAP_PRESS % 30 > 16  # a frame where the speed sync has no press of its own
  assert s.button(_cs(vset=80, gap=2), _hud(3), GAP_PRESS) == 0   # speed still busy
  assert s.button(_cs(vset=90, gap=2), _hud(3), GAP_PRESS) == 3   # now the gap
  # waiting on the jump guard also holds the gap back
  assert CameraSync().button(_cs(vset=80, gap=2, kph=100), _hud(3), GAP_PRESS) == 0


def test_a_different_button_waits_for_a_release():
  from opendbc.car.hyundai.hyundaicanfd import SYNC_SWITCH_FRAMES, CameraSync
  s = CameraSync()
  assert [s.button(_cs(vset=90, gap=2), _hud(3), f) for f in (312, 314, 316)] == [3, 3, 3]  # one GAP press (60 ms)
  assert s.button(_cs(vset=91, gap=2), _hud(3), 342) == 0          # - inside SYNC_SWITCH_FRAMES after it: not yet
  assert 372 - 316 > SYNC_SWITCH_FRAMES
  assert s.button(_cs(vset=91, gap=2), _hud(3), 372) == 2          # next speed window: -
  s = CameraSync()
  assert s.button(_cs(vset=89, gap=2), _hud(3), 312) == 1          # +
  assert s.button(_cs(vset=90, gap=2), _hud(3), 314) == 0          # GAP right after +: not yet
  assert s.button(_cs(vset=90, gap=2), _hud(3), 412) == 3          # next gap window


def test_create_ccnc_messages_notes_every_driver_press(monkeypatch):
  from opendbc.car import structs
  from opendbc.car.hyundai import hyundaicanfd
  from opendbc.car.hyundai.values import HyundaiFlags
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 1, get=lambda key: '0'))
  for attr in ('_camera_sync', '_lane_line_check'):
    monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, attr, raising=False)
  sent = []
  packer = SimpleNamespace(make_can_msg=lambda name, bus, values: sent.append((name, dict(values))) or (0, b'', bus))
  cs = SimpleNamespace(out=structs.CarState(vEgo=22., vEgoCluster=22., vCruiseCluster=80.), modelV2=None,
                       lfahda_cluster=None, cruise_buttons_alt2=None, cruise_buttons_msg={"LFA_BTN": 0, "CRUISE_BUTTONS": 0},
                       cruise_btns_msg_canfd="CRUISE_BUTTONS", MainMode_ACC=True, ACCMode=1, adrv_0x1ea=None, is_metric=True,
                       scc_control={"InfoDisplay": 0, "VSetDis": 80, "DISTANCE_SETTING": 2})
  cc = structs.CarControl(enabled=True)
  cc.hudControl.leadDistanceBars = 3

  def button_at(frame, pressed=0):
    cs.cruise_buttons_msg["CRUISE_BUTTONS"] = pressed
    sent.clear()
    hyundaicanfd.create_ccnc_messages(SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2),
                                      frame, cc, cs, cc.hudControl, 0, False, False, 0, False, 0, 0)
    return [v["CRUISE_BUTTONS"] for name, v in sent if name == "CRUISE_BUTTONS"][0]

  assert button_at(22, pressed=3) == 3  # the driver's own GAP goes through unchanged
  assert button_at(112) == 0            # 0.9 s later: the gap sync still waits
  assert button_at(212) == 3            # over 1 s: the sync presses GAP (camera 2 -> openpilot 3)
