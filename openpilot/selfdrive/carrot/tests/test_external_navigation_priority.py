from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot import carrot_serv
from openpilot.selfdrive.carrot.tests.test_vehicle_speed_camera_control import _car_state


@pytest.fixture
def navigation_update(monkeypatch):
  values = {"AutoNaviSpeedBumpSpeed": 22, "AutoNaviSpeedBumpTime": 5,
            "AutoNaviSpeedBumpEndDistance": 600, "AutoNaviSpeedCtrlMode": 2,
            "AutoNaviSpeedCtrlEnd": 10, "AutoNaviSpeedDecelRate": 135,
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

  def update():
    serv.update_navi("", sm, pm, 200., [], [], 200., "gpsLocationExternal")
    return sent["carrotMan"].carrotMan

  return serv, CS, update


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
