from types import SimpleNamespace

from openpilot.selfdrive.test.process_replay.process_replay import DriverMonitoringRcvCallback, generate_params_config


def event(service, seconds):
  return SimpleNamespace(which=lambda: service, logMonoTime=int(seconds * 1e9))


def driver_monitoring_state(*, disabled, rhd=False, valid=True):
  return SimpleNamespace(
    which=lambda: "driverMonitoringState",
    valid=valid,
    driverMonitoringState=SimpleNamespace(dm2Disabled=disabled, isRHD=rhd),
  )


def test_driver_monitoring_replay_uses_driver_model_cadence_when_present():
  callback = DriverMonitoringRcvCallback(initial_driver_time=int(1.025e9))
  assert not callback(event("modelV2", 1.0), None, 0)
  assert callback(event("driverStateV2", 1.025), None, 0)
  assert not callback(event("modelV2", 1.05), None, 0)
  assert callback(event("driverStateV2", 1.075), None, 0)


def test_driver_monitoring_replay_ticks_through_driver_and_road_model_gaps():
  callback = DriverMonitoringRcvCallback()
  assert callback(event("modelV2", 1.0), None, 0)
  assert not callback(event("carState", 1.01), None, 0)
  assert callback(event("modelV2", 1.05), None, 0)

  assert callback(event("driverStateV2", 1.075), None, 0)
  assert not callback(event("modelV2", 1.10), None, 0)
  assert not callback(event("carState", 1.125), None, 0)
  assert callback(event("carState", 1.135), None, 0)
  assert not callback(event("carState", 1.15), None, 0)
  assert callback(event("modelV2", 1.185), None, 0)


def test_process_replay_derives_initial_driver_monitoring_setting():
  assert generate_params_config(lr=[])["DriverMonitoringEnabled"] is True
  assert generate_params_config(lr=[])["DriverMonitoringSessionDisabled"] is False
  assert generate_params_config(lr=[driver_monitoring_state(disabled=True)])["DriverMonitoringSessionDisabled"] is True
  assert generate_params_config(lr=[
    driver_monitoring_state(disabled=False, valid=False),
    driver_monitoring_state(disabled=True),
  ])["DriverMonitoringSessionDisabled"] is True
  assert generate_params_config(
    lr=[driver_monitoring_state(disabled=True)],
    custom_params={"DriverMonitoringSessionDisabled": False},
  )["DriverMonitoringSessionDisabled"] is False
  assert generate_params_config(
    lr=[driver_monitoring_state(disabled=False)],
    custom_params={"DriverMonitoringEnabled": False},
  )["DriverMonitoringEnabled"] is False
