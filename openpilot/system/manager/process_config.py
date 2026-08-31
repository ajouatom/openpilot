import os
import platform
import importlib.util

from openpilot.cereal import car
from openpilot.common.params import Params
from openpilot.system.hardware import PC, TICI
from openpilot.system.manager.process import PythonProcess, NativeProcess, DaemonProcess

try:
  BODYTELEOP_AVAILABLE = importlib.util.find_spec("openpilot.tools.bodyteleop.web") is not None
except ModuleNotFoundError:
  BODYTELEOP_AVAILABLE = False

WEBCAM = os.getenv("USE_WEBCAM") is not None
CARROT_WEB_EXTERNAL = os.getenv("CARROT_WEB_EXTERNAL") == "1"

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started or params.get_bool("IsDriverViewEnabled")

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and CP.notCar

def iscar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not CP.notCar

def logging(started: bool, params: Params, CP: car.CarParams) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

def ublox(started: bool, params: Params, CP: car.CarParams) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox)
  return started and use_ublox

def joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("JoystickDebugMode")

def not_joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("JoystickDebugMode")

def long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LongitudinalManeuverMode")

def lat_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LateralManeuverMode")

def not_long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("LongitudinalManeuverMode")

def qcomgps(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not ublox_available()

def always_run(started: bool, params: Params, CP: car.CarParams) -> bool:
  return True

def only_onroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started


def only_offroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started

def enable_updated(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started and params.get_bool("SoftwareMenu")

def or_(*fns):
  return lambda *args: any(fn(*args) for fn in fns)

def and_(*fns):
  return lambda *args: all(fn(*args) for fn in fns)

def enable_dm(started, params, CP: car.CarParams) -> bool:
  return (started or params.get_bool("IsDriverViewEnabled")) and params.get_int("DisableDM") == 0

#def enable_connect(started, params, CP: car.CarParams) -> bool:
#  return params.get_int("EnableConnect") > 0

def enable_xiaoge_data(started, params, CP: car.CarParams) -> bool:
  return params.get_bool("ShareData")

def cluster_hud_active(params: Params) -> bool:
  try:
    return params.get_int("ClusterHud") == 1
  except Exception:
    return False

def enable_webrtc(started, params, CP: car.CarParams) -> bool:
  # Cluster HUD consumes the road camera directly. Keep Carrot Vision's
  # WebRTC/encoder processes out of the same onroad session.
  return params.get_int("DisableDM") == 2 and not cluster_hud_active(params)

def c3x_lite(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("HardwareC3xLite")

def enable_youtube_low_encoder(started, params, CP: car.CarParams) -> bool:
  try:
    return params.get_int("CarrotYouTubeLive") > 0 and params.get_int("CarrotYouTubeQuality") not in (1, 2, 3)
  except Exception:
    return False

def enable_youtube_medium_encoder(started, params, CP: car.CarParams) -> bool:
  try:
    return params.get_int("CarrotYouTubeLive") > 0 and params.get_int("CarrotYouTubeQuality") == 1
  except Exception:
    return False

def enable_youtube_encoder(started, params, CP: car.CarParams) -> bool:
  try:
    return params.get_int("CarrotYouTubeLive") > 0 and params.get_int("CarrotYouTubeQuality") == 2
  except Exception:
    return False

def enable_youtube_wide_encoder(started, params, CP: car.CarParams) -> bool:
  try:
    use_wide_camera = bool(params.get("UseWideCamera", return_default=True))
    return use_wide_camera and params.get_int("CarrotYouTubeLive") > 0 and params.get_int("CarrotYouTubeQuality") == 3
  except Exception:
    return False

def enable_cluster_hud(started, params, CP: car.CarParams) -> bool:
  return cluster_hud_active(params)

procs = [
  DaemonProcess("manage_athenad", "openpilot.system.athena.manage_athenad", "AthenadPid"),

  NativeProcess("loggerd", "openpilot/system/loggerd", ["./loggerd"], logging),
  NativeProcess("encoderd", "openpilot/system/loggerd", ["./encoderd"], only_onroad),
  # Preserve generic multi-camera WebRTC for notCar users. Carrot Vision on a
  # real device is road-only and remains gated by DisableDM == 2.
  NativeProcess("stream_encoderd", "openpilot/system/loggerd", ["./encoderd", "--stream"], notcar),
  # Prewarm the hardware encoder with the rest of the onroad stack. The
  # encoder process stays idle until CarrotVisionActive is set by a session.
  NativeProcess("carrot_vision_encoderd", "openpilot/system/loggerd", ["./encoderd", "--carrot-vision-road"], and_(iscar, enable_webrtc)),
  NativeProcess("youtube_low_encoderd", "openpilot/system/loggerd", ["./encoderd", "--youtube-low"], and_(only_onroad, enable_youtube_low_encoder)),
  NativeProcess("youtube_medium_encoderd", "openpilot/system/loggerd", ["./encoderd", "--youtube-medium"], and_(only_onroad, enable_youtube_medium_encoder)),
  NativeProcess("youtube_encoderd", "openpilot/system/loggerd", ["./encoderd", "--youtube"], and_(only_onroad, enable_youtube_encoder)),
  NativeProcess("youtube_wide_encoderd", "openpilot/system/loggerd", ["./encoderd", "--youtube-wide"], and_(only_onroad, enable_youtube_wide_encoder)),
  PythonProcess("logmessaged", "openpilot.system.logmessaged", always_run),

  NativeProcess("camerad", "openpilot/system/camerad", ["./camerad"], driverview, enabled=not WEBCAM),
  PythonProcess("webcamerad", "openpilot.tools.webcam.camerad", driverview, enabled=WEBCAM),
  PythonProcess("proclogd", "openpilot.system.proclogd", only_onroad, enabled=platform.system() != "Darwin"),
  PythonProcess("journald", "openpilot.system.journald", only_onroad, platform.system() != "Darwin"),
  PythonProcess("micd", "openpilot.system.micd", iscar),
  PythonProcess("timed", "openpilot.system.timed", always_run, enabled=not PC),

  PythonProcess("modeld", "openpilot.selfdrive.modeld.modeld", only_onroad),
  PythonProcess("dmonitoringmodeld", "openpilot.selfdrive.modeld.dmonitoringmodeld", enable_dm, enabled=(WEBCAM or not PC)),
  PythonProcess("sensord", "openpilot.system.sensord.sensord", only_onroad, enabled=not PC),
  PythonProcess("ui", "openpilot.selfdrive.ui.ui", always_run, restart_if_crash=True),
  PythonProcess("soundd", "openpilot.selfdrive.ui.soundd", driverview),
  PythonProcess("locationd", "openpilot.selfdrive.locationd.locationd", only_onroad),
  NativeProcess("_pandad", "openpilot/selfdrive/pandad", ["./pandad"], always_run, enabled=False),
  PythonProcess("calibrationd", "openpilot.selfdrive.locationd.calibrationd", only_onroad),
  PythonProcess("torqued", "openpilot.selfdrive.locationd.torqued", only_onroad),
  PythonProcess("controlsd", "openpilot.selfdrive.controls.controlsd", and_(not_joystick, iscar)),
  PythonProcess("joystickd", "openpilot.tools.joystick.joystickd", or_(joystick, notcar)),
  PythonProcess("selfdrived", "openpilot.selfdrive.selfdrived.selfdrived", only_onroad),
  PythonProcess("card", "openpilot.selfdrive.car.card", only_onroad),
  PythonProcess("deleter", "openpilot.system.loggerd.deleter", always_run),
  PythonProcess("dmonitoringd", "openpilot.selfdrive.monitoring.dmonitoringd", enable_dm, enabled=(WEBCAM or not PC)),
  PythonProcess("qcomgpsd", "openpilot.system.qcomgpsd.qcomgpsd", qcomgps, enabled=TICI),
  PythonProcess("navd", "openpilot.selfdrive.navd.navd", only_onroad),
  PythonProcess("pandad", "openpilot.selfdrive.pandad.pandad", always_run),
  PythonProcess("paramsd", "openpilot.selfdrive.locationd.paramsd", only_onroad),
  PythonProcess("lagd", "openpilot.selfdrive.locationd.lagd", only_onroad),
  PythonProcess("ubloxd", "openpilot.system.ubloxd.ubloxd", ublox, enabled=TICI),
  PythonProcess("pigeond", "openpilot.system.ubloxd.pigeond", ublox, enabled=TICI),
  PythonProcess("plannerd", "openpilot.selfdrive.controls.plannerd", not_long_maneuver),
  PythonProcess("maneuversd", "openpilot.tools.longitudinal_maneuvers.maneuversd", long_maneuver),
  PythonProcess("lateral_maneuversd", "openpilot.tools.lateral_maneuvers.lateral_maneuversd", lat_maneuver),
  PythonProcess("radard", "openpilot.selfdrive.carrot.radar.radard_dpath", only_onroad),
  PythonProcess("hardwared", "openpilot.system.hardware.hardwared", always_run),
  PythonProcess("modem", "openpilot.system.hardware.tici.modem", always_run, enabled=TICI),
  PythonProcess("tombstoned", "openpilot.system.tombstoned", always_run, enabled=not PC),
  PythonProcess("updated", "openpilot.system.updated.updated", enable_updated, enabled=not PC),
  #PythonProcess("uploader", "openpilot.system.loggerd.uploader", enable_connect),
  PythonProcess("statsd", "openpilot.system.statsd", always_run),
  PythonProcess("feedbackd", "openpilot.selfdrive.ui.feedback.feedbackd", only_onroad),

  # debug procs
  NativeProcess("bridge", "openpilot/cereal/messaging", ["./bridge"], notcar),
  PythonProcess("webrtcd", "openpilot.system.webrtc.webrtcd", notcar),
  PythonProcess("carrot_webrtcd", "openpilot.system.webrtc.carrot_webrtcd", and_(iscar, enable_webrtc)),
  PythonProcess("webjoystick", "openpilot.tools.bodyteleop.web", notcar, enabled=BODYTELEOP_AVAILABLE),
  PythonProcess("joystick", "openpilot.tools.joystick.joystick_control", and_(joystick, iscar)),

  PythonProcess("carrot_man", "openpilot.selfdrive.carrot.carrot_man", always_run, restart_if_crash=True),#, enabled=not PC),
  # carrot_navi permanently owns TCP 7714 and publishes navigation data over cereal.
  PythonProcess("carrot_navi", "openpilot.selfdrive.carrot.carrot_navi", always_run, restart_if_crash=True),

  PythonProcess("carrot_server", "openpilot.selfdrive.carrot.carrot_server", always_run, enabled=not CARROT_WEB_EXTERNAL),
  PythonProcess("cweb_push", "openpilot.selfdrive.carrot.cweb_push", always_run, enabled=not PC),
  PythonProcess("carrot_cluster", "openpilot.selfdrive.carrot.cluster_autorun", enable_cluster_hud, restart_if_crash=True),

  #Xiaoge data broadcaster (conditional on ShareData param)
  PythonProcess("xiaoge_data", "openpilot.selfdrive.carrot.xiaoge_data", enable_xiaoge_data),

  # C3x lite has no speaker; mirror alerts to the GPIO buzzer instead.
  PythonProcess("beep", "openpilot.selfdrive.controls.beep", c3x_lite, enabled=TICI),
]

managed_processes = {p.name: p for p in procs}
