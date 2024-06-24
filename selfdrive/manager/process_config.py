import os

from cereal import car, log
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE, PC, TICI
from openpilot.selfdrive.manager.process import PythonProcess, NativeProcess, DaemonProcess

WIFI = log.DeviceState.NetworkType.wifi

WEBCAM = os.getenv("USE_WEBCAM") is not None

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started or params.get_bool("IsDriverViewEnabled")

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and CP.notCar

def iscar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not CP.notCar

def logging(started, params, CP: car.CarParams) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

def ublox(started, params, CP: car.CarParams) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox)
  return started and use_ublox

def qcomgps(started, params, CP: car.CarParams) -> bool:
  return started and not ublox_available()

def always_run(started, params, CP: car.CarParams) -> bool:
  return True

def only_onroad(started: bool, params, CP: car.CarParams) -> bool:
  return started

def only_offroad(started, params, CP: car.CarParams) -> bool:
  return not started

def allow_uploads(started, params, CP: car.CarParams) -> bool:
  #enable_logging = not (params.get_bool("FireTheBabysitter") and params.get_bool("NoLogging"))
  #wifi_connected = HARDWARE.get_network_type() == WIFI and not started
  return False
  #return wifi_connected if params.get_bool("DisableOnroadUploads") else enable_logging

def enable_dm(started, params, CP: car.CarParams) -> bool:
  return (started or params.get_bool("IsDriverViewEnabled")) and params.get_int("ShowDmInfo") >= 0

def enable_logging(started, params, CP: car.CarParams) -> bool:
  #return True
  #print("$$$$$$$$$ Logging Disabled = {}".format(params.get_bool("NoLogging")))
  return not params.get_bool("NoLogging")

NoLogging = Params().get_bool("NoLogging")
def osm(started, params, CP: car.CarParams) -> bool:
  return True #params_memory.get_bool("RoadNameUI") or params_memory.get_bool("SpeedLimitController")

procs = [
  DaemonProcess("manage_athenad", "selfdrive.athena.manage_athenad", "AthenadPid"),

  NativeProcess("camerad", "system/camerad", ["./camerad"], driverview),
  NativeProcess("logcatd", "system/logcatd", ["./logcatd"], (enable_logging and only_onroad)),
  NativeProcess("proclogd", "system/proclogd", ["./proclogd"], only_onroad),
  PythonProcess("logmessaged", "system.logmessaged", enable_logging),
  PythonProcess("micd", "system.micd", iscar),
  PythonProcess("timed", "system.timed", always_run, enabled=not PC),

  PythonProcess("dmonitoringmodeld", "selfdrive.modeld.dmonitoringmodeld", enable_dm, enabled=(not PC or WEBCAM)),
  #PythonProcess("carrotmodeld", "selfdrive.modeld.carrot.carrotmodeld", only_onroad),
  NativeProcess("encoderd", "system/loggerd", ["./encoderd"], only_onroad),
  NativeProcess("stream_encoderd", "system/loggerd", ["./encoderd", "--stream"], notcar),
  NativeProcess("loggerd", "system/loggerd", ["./loggerd"], (enable_logging and logging), enabled=not NoLogging),
  NativeProcess("modeld", "selfdrive/modeld", ["./modeld"], only_onroad),
  NativeProcess("sensord", "system/sensord", ["./sensord"], only_onroad, enabled=not PC),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], always_run, watchdog_max_dt=(5 if not PC else None)),
  PythonProcess("soundd", "selfdrive.ui.soundd", only_onroad),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"], only_onroad),
  NativeProcess("boardd", "selfdrive/boardd", ["./boardd"], always_run, enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd", only_onroad),
  PythonProcess("torqued", "selfdrive.locationd.torqued", only_onroad),
  PythonProcess("controlsd", "selfdrive.controls.controlsd", only_onroad),
  PythonProcess("deleter", "system.loggerd.deleter", always_run),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", enable_dm, enabled=(not PC or WEBCAM)),
  PythonProcess("qcomgpsd", "system.qcomgpsd.qcomgpsd", qcomgps, enabled=TICI),
  #PythonProcess("ugpsd", "system.ugpsd", only_onroad, enabled=TICI),
  PythonProcess("navd", "selfdrive.navd.navd", only_onroad),
  PythonProcess("pandad", "selfdrive.boardd.pandad", always_run),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd", only_onroad),
  NativeProcess("ubloxd", "system/ubloxd", ["./ubloxd"], ublox, enabled=TICI),
  PythonProcess("pigeond", "system.ubloxd.pigeond", ublox, enabled=TICI),
  PythonProcess("plannerd", "selfdrive.controls.plannerd", only_onroad),
  PythonProcess("radard", "selfdrive.controls.radard", only_onroad),
  PythonProcess("thermald", "selfdrive.thermald.thermald", always_run),
  PythonProcess("tombstoned", "selfdrive.tombstoned", enable_logging, enabled=not PC),
  PythonProcess("updated", "selfdrive.updated.updated", only_offroad, enabled=not PC),
  PythonProcess("uploader", "system.loggerd.uploader", allow_uploads),
  PythonProcess("statsd", "selfdrive.statsd", always_run),

  # debug procs
  NativeProcess("bridge", "cereal/messaging", ["./bridge"], notcar),
  PythonProcess("webrtcd", "system.webrtc.webrtcd", notcar),
  PythonProcess("webjoystick", "tools.bodyteleop.web", notcar),

  PythonProcess("fleet_manager", "selfdrive.frogpilot.fleetmanager.fleet_manager", always_run),
  PythonProcess("mapd", "selfdrive.frogpilot.functions.mapd", osm),
  
  PythonProcess("road_speed_limiter", "selfdrive.carrot.road_speed_limiter", always_run),
  PythonProcess("carrot_man", "selfdrive.carrot.carrot_man", always_run),
  
  PythonProcess("navi_controller", "selfdrive.controls.neokii.navi_controller", always_run, enabled=not PC),
  #PythonProcess("navi_route", "selfdrive.controls.neokii.navi_route", only_onroad, enabled=Params().get_bool('UseExternalNaviRoutes')),
  PythonProcess("navi_route", "selfdrive.controls.neokii.navi_route", only_onroad, enabled=not PC),
  
]

managed_processes = {p.name: p for p in procs}
