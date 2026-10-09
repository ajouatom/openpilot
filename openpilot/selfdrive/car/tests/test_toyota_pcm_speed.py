from types import SimpleNamespace

import pytest

from openpilot.cereal import car
import openpilot.selfdrive.car.cruise as cruise


class ZeroParams:
  def __init__(self, *args):
    pass

  def get_int(self, key):
    return 0

  def get_bool(self, key):
    return False


@pytest.mark.parametrize('brand, pcm, mode, uses_stock', [
  ('toyota', True, 3, True), ('toyota', True, 1, True),
  ('toyota', True, 0, False), ('toyota', True, 2, False),
  ('honda', True, 3, False), ('hyundai', True, 3, False),
  ('honda', True, 1, True), ('mock', False, 3, False),
])
@pytest.mark.parametrize('longitudinal', [True, False])
def test_pcm_set_resume_and_adjustments_replace_stale_speed(monkeypatch, brand, pcm, mode, uses_stock, longitudinal):
  monkeypatch.setattr(cruise, 'Params', ZeroParams)
  helper = cruise.VCruiseCarrot(SimpleNamespace(brand=brand, pcmCruise=pcm, openpilotLongitudinalControl=longitudinal))
  helper.speed_from_pcm = mode
  helper.v_cruise_kph = 83.
  helper.cruise_state_available_last = True
  helper.update_params = lambda is_metric: None
  helper._prepare_brake_gas = lambda CS, CC: None
  helper._update_cruise_buttons = lambda CS, CC, speed: speed
  helper._update_carrot_man = lambda sm: None
  sm = {'carControl': car.CarControl(enabled=True)}
  sm = type('SM', (dict,), {'alive': {'longitudinalPlan': False, 'radarState': False, 'drivingModelData': False}})(sm)
  cs = car.CarState(gearShifter='drive', vEgo=21., vEgoCluster=21.,
                    cruiseState={'available': True, 'enabled': True})
  # The incident's 83/75 mismatch, then physical SET/RES changes. No speed
  # buttonEvents are present on Toyota; only PCM set speed changes.
  for speed, enabled in ((75., True), (74., True), (74., False), (74., True), (80., True)):
    cs.cruiseState.speed = speed / 3.6
    cs.cruiseState.speedCluster = (speed + 1.) / 3.6
    cs.cruiseState.enabled = enabled
    helper.update_v_cruise(cs, sm, True)
    assert helper.v_cruise_kph == pytest.approx(speed if uses_stock else 83.)
    assert helper.v_cruise_cluster_kph == pytest.approx(speed + 1. if uses_stock else 83.)
