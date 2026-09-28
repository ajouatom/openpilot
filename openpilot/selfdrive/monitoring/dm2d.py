"""20 Hz DM dispatcher: stock camera criteria or automatic interaction fallback."""
import time
import math

from openpilot.cereal import car
import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_DMON, Ratekeeper, config_realtime_process
from openpilot.selfdrive.monitoring.config import experimental_mode
from openpilot.selfdrive.carrot.bluetooth.model import CommandReader
from openpilot.selfdrive.monitoring.dm2 import DriverMonitoring2
from openpilot.selfdrive.monitoring.dm2_context import CameraAvailability, InteractionEdges, ObjectObservation, TrafficContext


def traffic_observations(radar):
  leads = [radar.leadOne, radar.leadTwo, radar.leadLeft, radar.leadRight]
  for name in ("leadsCenter", "leadsLeft", "leadsRight", "leadsLeft2", "leadsRight2", "leadsCutIn"):
    leads.extend(getattr(radar, name))
  return [ObjectObservation(p.dRel, p.dPath, p.vLead, p.vRel) for p in leads if p.status]


def side_coverage(params, cp):
  # RadarInterface includes corner parser health in radarErrors on these platforms.
  from opendbc.car.hyundai.values import HyundaiExtFlags
  flags = int(HyundaiExtFlags.CORNER_RADAR_OBJECTS_235 | HyundaiExtFlags.CORNER_RADAR_OBJECTS_180 |
              HyundaiExtFlags.CORNER_RADAR_OBJECTS_430)
  return cp.brand == "hyundai" and params.get_int("EnableCornerRadar") > 0 and bool(int(cp.extFlags) & flags)


def camera_sample_usable(sm, rhd, demo=False):
  if not sm.all_checks(['driverStateV2'] if demo else ['driverStateV2', 'liveCalibration', 'modelV2']):
    return False
  driver = sm['driverStateV2'].rightDriverData if rhd else sm['driverStateV2'].leftDriverData
  vectors = (driver.faceOrientation, driver.facePosition, driver.faceOrientationStd, driver.facePositionStd)
  lengths = (2, 2, 2, 2)
  if not demo:
    vectors += (sm['liveCalibration'].rpyCalib, sm['modelV2'].meta.disengagePredictions.brakeDisengageProbs)
    lengths += (3, 1)
  return all(len(values) >= length and all(math.isfinite(v) for v in values)
             for values, length in zip(vectors, lengths, strict=True))


def run_dm2(params, experimental):
  services = ['carState', 'selfdriveState', 'modelV2', 'radarState', 'liveCalibration', 'carParams', 'driverStateV2']
  sm = messaging.SubMaster(services, poll='driverStateV2', frequency=int(1 / DT_DMON))
  pm = messaging.PubMaster(['driverMonitoringState'])
  # carState is 100 Hz; conflating it to 20 Hz can lose a complete button press.
  input_sock = messaging.sub_sock('carState', conflate=False)
  dm = DriverMonitoring2(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"))
  traffic, inputs = TrafficContext(), InteractionEdges()
  bluetooth = CommandReader('attention')
  camera_health = CameraAvailability()
  rk = Ratekeeper(int(1 / DT_DMON), print_delay_threshold=None)
  strict, clear = True, False
  covered = False
  allow_speed_buttons = False
  demo_mode = params.get_bool("IsDriverViewEnabled")
  while True:
    sm.update(int(DT_DMON * 1000))
    now = time.monotonic()
    if sm.updated['carParams']:
      covered = side_coverage(params, sm['carParams'])
      # Stock-ACC speed button injection can be indistinguishable from driver
      # input after a gateway echo. Do not grant credit on that configuration.
      allow_speed_buttons = sm['carParams'].openpilotLongitudinalControl or params.get_int("SpeedFromPCM") == 0
    cs = sm['carState']
    valid = sm.all_checks(['carState', 'selfdriveState']) and cs.canValid
    response = False
    for packet in messaging.drain_sock(input_sock, wait_for_one=False):
      sample = packet.carState
      if packet.valid and sample.canValid and 0 <= now - packet.logMonoTime / 1e9 < 0.25:
        buttons = [(str(be.type), be.pressed) for be in sample.buttonEvents
                   if allow_speed_buttons or str(be.type) not in inputs.SPEED_BUTTONS]
        response |= inputs.update(now, sample.gasPressed, sample.brakePressed, sample.steeringPressed, buttons)
    bt_action = bluetooth.read(allowed=valid, now=now)
    if bt_action is not None:
      response = True
      inputs.last_response = now
      if bt_action.removesuffix('Long') in inputs.SPEED_BUTTONS:
        inputs.last_pedal_or_speed = now
    road_ok = sm.all_checks(['modelV2', 'radarState']) and not any(sm['radarState'].radarErrors.to_dict().values())
    if not road_ok or sm.updated['radarState']:
      model = sm['modelV2']
      straight = (abs(cs.steeringAngleDeg) < 5 and abs(cs.aEgo) < 0.5 and
                  len(model.orientationRate.z) >= 10 and max(abs(v) for v in model.orientationRate.z[:10]) < 0.01 and
                  len(model.laneLineProbs) == 4 and min(model.laneLineProbs[1:3]) > 0.8 and
                  not cs.leftBlinker and not cs.rightBlinker)
      strict, clear = traffic.update(now, traffic_observations(sm['radarState']), road_ok, straight, covered)
    camera_ok = camera_health.update(now, camera_sample_usable(sm, dm.wheel_on_right, demo_mode) and
                                     0 <= now - sm.logMonoTime['driverStateV2'] / 1e9 < 0.5)
    factor = inputs.timeout_factor(now, experimental, strict, clear)
    dm.set_camera_available(camera_ok, factor)
    if camera_ok:
      if sm.updated['driverStateV2'] and (valid or demo_mode):
        dm.relax_pose = experimental and not strict
        dm.run_step(sm, demo=demo_mode)
        dm.credit_camera_interaction(now, inputs.last_response)
    elif valid:
      dm.input_credit_seconds = 0.0
      dm.run_without_camera(response and valid, sm['selfdriveState'].enabled,
                            cs.vEgo < dm.settings._ALERT_MIN_SPEED,
                            cs.gearShifter not in (car.CarState.GearShifter.drive, car.CarState.GearShifter.low), factor)
    packet = dm.get_state_packet(valid=valid or (demo_mode and camera_ok))
    packet.driverMonitoringState.cameraUnavailable = not camera_ok
    packet.driverMonitoringState.dm2Experimental = experimental
    packet.driverMonitoringState.dm2StrictTimeRemaining = max(0.0, traffic.strict_until - now)
    packet.driverMonitoringState.dm2WheelTimeoutFactor = dm.wheel_factor if not camera_ok else 1.0
    pm.send('driverMonitoringState', packet)
    if rk.frame % 40 == 0:
      dm.always_on = params.get_bool("AlwaysOnDM")
      demo_mode = params.get_bool("IsDriverViewEnabled")
    if camera_ok and rk.frame % 6000 == 0 and not demo_mode and dm.wheelpos_offsetter.filtered_stat.n > dm.settings._WHEELPOS_FILTER_MIN_COUNT:
      params.put_bool("IsRhdDetected", dm.wheel_on_right)
    rk.keep_time()


def main():
  config_realtime_process([0, 1, 2, 3], 5)
  params = Params()
  run_dm2(params, experimental_mode(params))


if __name__ == '__main__':
  main()
