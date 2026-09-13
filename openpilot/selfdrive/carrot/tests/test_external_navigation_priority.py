from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot import carrot_serv
from openpilot.selfdrive.carrot.tests.test_vehicle_speed_camera_control import _car_state


@pytest.fixture
def navigation_update(monkeypatch):
  values = {"AutoNaviSpeedBumpSpeed": 22, "AutoNaviSpeedBumpTime": 5,
            "AutoNaviSpeedBumpEndDistance": 600, "AutoNaviSpeedCtrlMode": 2,
            "AutoNaviSpeedCtrlEnd": 10, "AutoNaviSpeedDecelRate": 135,
            "AutoNaviRearCameraHoldDistance": 100,
            "AutoNaviSpeedSafetyFactor": 100, "AutoNaviCountDownMode": 2,
            "VehicleNaviCanControl": 1, "VehicleNaviSchoolZoneControl": 1,
            "VehicleSpeedCameraControlMode": 1, "IsMetric": 1, "AutoRoadSpeedLimitOffset": -1}

  class Params:
    def __init__(self, *_args):
      pass

    def get(self, key):
      return str(values.get(key, 0))

    def get_int(self, key):
      return int(self.get(key))

    def get_float(self, key):
      return float(self.get(key))

    def get_bool(self, key):
      return bool(self.get_int(key))

  monkeypatch.setattr(carrot_serv, "Params", Params)
  serv = carrot_serv.CarrotServ()
  monkeypatch.setattr(serv, "_update_carrot_navi", lambda _sm: False)
  monkeypatch.setattr(serv, "_update_gps", lambda *_args: 0.)
  monkeypatch.setattr(serv, "update_nav_instruction", lambda _sm: None)
  monkeypatch.setattr(serv, "update_auto_turn", lambda *_args: (250., "none", 250., 0.))
  CS = _car_state(speed_limit=0, distance=0, v_ego=22 / 3.6)
  CS.vehicleNaviActive = CS.vehicleNaviAvailable = True

  class SubMaster(dict):
    alive = {"carState": True, "selfdriveState": True, "navInstruction": False}

  sm = SubMaster(carState=CS, selfdriveState=SimpleNamespace(distanceTraveled=0.))
  sent = {}
  pm = SimpleNamespace(send=lambda name, message: sent.update({name: message}))

  def update(distance_traveled=None):
    if distance_traveled is not None:
      sm['selfdriveState'].distanceTraveled = distance_traveled
    serv.update_navi("", sm, pm, 200., [], [], 200., "gpsLocationExternal")
    return sent["carrotMan"].carrotMan

  return serv, CS, update


@pytest.mark.parametrize("sdi_type", (75, 76))
def test_rear_camera_holds_through_pass_and_releases_by_traveled_distance(navigation_update, sdi_type):
  serv, CS, update = navigation_update
  serv.active_count, serv.active_sdi_count = 80, 200
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = sdi_type, 50, 20
  assert update().desiredSpeed == 50
  result = update(20)
  assert result.xSpdDist == 0  # Countdown ends at the camera, not at hold release.
  assert (result.desiredSource, result.desiredSpeed) == ("cam", 50)
  assert "100m" in result.szSdiDescr
  for _ in range(10):
    assert update(20).desiredSpeed == 50  # Stopping does not consume the hold.
  assert update(119).desiredSpeed == 50
  result = update(120)
  assert (result.desiredSource, result.desiredSpeed) == ("road", 200)
  assert not serv.rear_camera_events


def test_rear_hold_survives_next_guidance_and_preserves_stronger_limit(navigation_update):
  serv, CS, update = navigation_update
  serv.active_count, serv.active_sdi_count = 80, 200
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = 76, 50, 20
  update()
  update(20)
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = 1, 80, 1000
  assert update(21).desiredSpeed == 50
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = 22, 22, 15
  result = update(22)
  assert (result.desiredSource, result.desiredSpeed) == ("bump", 22)
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = -1, 0, 0
  assert update(40).desiredSpeed == 50


def test_rear_hold_disconnect_resumes_stock_control(navigation_update):
  serv, CS, update = navigation_update
  serv.active_count, serv.active_sdi_count = 80, 200
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = 75, 50, 20
  update()
  CS.speedLimit, CS.speedLimitDistance = 30, 10
  serv.active_count = 1
  result = update(20)
  assert (result.desiredSource, result.desiredSpeed) == ("hda", 30)
  assert not serv.rear_camera_events


@pytest.mark.parametrize("reset", ("disabled", "off_route", "distance_reset", "session", "no_car_state"))
def test_rear_hold_clears_invalid_or_disabled_tracking(navigation_update, reset):
  serv, CS, update = navigation_update
  serv.active_count, serv.active_sdi_count = 80, 200
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = 75, 50, 20
  update()
  assert serv.rear_camera_events
  serv.xSpdType = -1
  if reset == "disabled":
    serv.autoNaviRearCameraHoldDistance = 0
  elif reset == "off_route":
    serv.carrot_navi_off_route = True
  elif reset == "session":
    serv._reset_carrot_navi_sequences("next-session")
  speed, remaining = serv._rear_camera_speed(None if reset == "no_car_state" else CS, -1 if reset == "distance_reset" else 0)
  assert (speed, remaining) == (250, 0)
  assert not serv.rear_camera_events


def test_rear_hold_does_not_arm_distant_or_front_camera(navigation_update):
  serv, CS, update = navigation_update
  serv.active_count, serv.active_sdi_count = 80, 200
  for kind, distance in ((1, 20), (76, 500), (75, 0)):
    serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = kind, 50, distance
    update()
    assert not serv.rear_camera_events


@pytest.mark.parametrize("stock_source", ("hda", "hda_bump", "hda_section", "school"))
def test_connected_external_without_hazard_never_falls_back_to_stock(navigation_update, stock_source):
  serv, CS, update = navigation_update
  if stock_source == "hda":
    CS.speedLimit, CS.speedLimitDistance = 50, 10
  elif stock_source == "hda_bump":
    CS.speedBumpDistance = 30
  elif stock_source == "hda_section":
    CS.vehicleNaviSectionActive, CS.vehicleNaviSpeed = True, 30
  else:
    CS.schoolZoneActive = True
  serv.active_count = 80
  result = update()
  assert (result.desiredSource, result.desiredSpeed) == ("road", 200)
  assert not result.vehicleNaviActive
  assert result.xSpdCountDown == 100
  assert result.nRoadLimitSpeed == 30

  serv.active_count = 1  # The existing connection timer expires on this update.
  result = update()
  assert result.desiredSource == stock_source
  assert result.desiredSpeed in (22, 30, 50)
  assert result.leftSec == 100  # Publish an idle beat when changing source.


def test_uploaded_bump_distance_mismatch_uses_external_release_point(navigation_update):
  serv, CS, update = navigation_update
  serv.active_count, serv.active_sdi_count = 80, 200
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = 22, 22, 13
  CS.speedBumpDistance = 30.62
  result = update()
  assert (result.desiredSource, result.desiredSpeed) == ("bump", 22)
  assert result.xSpdCountDown == 1  # External 13 m, not stock 30.62 m.
  assert update().leftSec == 1

  serv.xSpdDist = 5.5
  CS.speedBumpDistance = 22
  result = update()
  assert (result.desiredSource, result.desiredSpeed) == ("road", 200)
  assert result.leftSec == 0
  assert not result.vehicleNaviActive


def test_connected_external_camera_wins_over_closer_stock_bump(navigation_update):
  serv, CS, update = navigation_update
  serv.active_count, serv.active_sdi_count = 80, 200
  serv.xSpdType, serv.xSpdLimit, serv.xSpdDist = 1, 60, 20
  CS.speedBumpDistance = 8
  CS.speedLimit, CS.speedLimitDistance = 50, 10
  result = update()
  assert (result.desiredSource, result.desiredSpeed) == ("cam", 60)
  assert result.xSpdCountDown == 2
  assert result.nRoadLimitSpeed == 30
