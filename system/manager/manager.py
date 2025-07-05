#!/usr/bin/env python3
import datetime
import os
import signal
import sys
import traceback

from cereal import log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.text_window import TextWindow
from openpilot.system.hardware import HARDWARE
from openpilot.system.manager.helpers import unblock_stdout, write_onroad_params, save_bootlog
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.version import get_build_metadata, terms_version, training_version
from openpilot.system.hardware.hw import Paths

def get_default_params():
  default_params : list[tuple[str, str | bytes]] = [
    # kans
    ("LongPitch", "1"),
    ("EVTable", "1"),
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("GsmMetered", "1"),
    ("HasAcceptedTerms", "0"),
    ("LanguageSetting", "main_en"),
    ("OpenpilotEnabledToggle", "1"),
    ("LongitudinalPersonality", str(log.LongitudinalPersonality.standard)),
    ("IsMetric", "1"),

    ("SearchInput", "0"),
    ("GMapKey", "0"),
    ("MapboxStyle", "0"),    


    ("LongitudinalPersonalityMax", "3"),
    ("ShowDebugUI", "0"),
    ("ShowTpms", "1"),
    ("ShowDateTime", "1"),
    ("ShowPathEnd", "1"),
    ("ShowCustomBrightness", "100"),
    ("ShowLaneInfo", "1"),
    ("ShowRadarInfo", "1"),
    ("ShowDeviceState", "1"),
    ("ShowRouteInfo", "1"),
    ("ShowPathMode", "9"),
    ("ShowPathColor", "13"),
    ("ShowPathModeCruiseOff", "0"),
    ("ShowPathColorCruiseOff", "19"),
    ("ShowPathModeLane", "14"),
    ("ShowPathColorLane", "13"),
    ("ShowPlotMode", "0"),
    ("AutoCruiseControl", "0"),
    ("CruiseEcoControl", "2"),
    ("CarrotCruiseDecel", "-1"),
    ("CarrotCruiseAtcDecel", "-1"),
    ("AutoGasTokSpeed", "0"),
    ("AutoGasSyncSpeed", "1"),
    ("AutoEngage", "0"),
    ("DisableMinSteerSpeed", "0"),
    ("SoftHoldMode", "0"),

    ("AutoSpeedUptoRoadSpeedLimit", "0"),
    ("AutoRoadSpeedAdjust", "50"),
    ("AutoCurveSpeedLowerLimit", "30"),
    ("AutoCurveSpeedFactor", "120"),
    ("AutoCurveSpeedAggressiveness", "100"),

    ("AutoTurnControl", "0"),
    ("AutoTurnControlSpeedTurn", "20"),
    ("AutoTurnControlTurnEnd", "6"),
    ("AutoTurnMapChange", "0"),

    ("AutoNaviSpeedCtrlEnd", "7"),
    ("AutoNaviSpeedCtrlMode", "2"),
    ("AutoNaviSpeedBumpTime", "1"),
    ("AutoNaviSpeedBumpSpeed", "35"),
    ("AutoNaviSpeedSafetyFactor", "105"),
    ("AutoNaviSpeedDecelRate", "120"),
    ("AutoRoadSpeedLimitOffset", "-1"),
    ("AutoNaviCountDownMode", "2"),
    ("TurnSpeedControlMode", "1"),
    ("MapTurnSpeedFactor", "90"),
    ("StoppingAccel", "0"),
    ("StopDistanceCarrot", "550"),
    ("JLeadFactor3", "0"),
    ("CruiseButtonMode", "0"),
    ("LfaButtonMode", "0"),
    ("CruiseButtonTest1", "8"),
    ("CruiseButtonTest2", "30"),
    ("CruiseButtonTest3", "1"),
    ("CruiseSpeedUnit", "10"),
    ("CruiseSpeed1", "30"),
    ("CruiseSpeed2", "50"),
    ("CruiseSpeed3", "80"),
    ("CruiseSpeed4", "110"),
    ("CruiseSpeed5", "130"),
    ("PaddleMode", "0"),
    ("MyDrivingMode", "3"),
    ("MyDrivingModeAuto", "0"),
    ("TrafficLightDetectMode", "2"),
    ("CruiseMaxVals0", "160"),
    ("CruiseMaxVals1", "200"),
    ("CruiseMaxVals2", "160"),
    ("CruiseMaxVals3", "130"),
    ("CruiseMaxVals4", "110"),
    ("CruiseMaxVals5", "95"),
    ("CruiseMaxVals6", "80"),
    ("LongTuningKpV", "100"),
    ("LongTuningKiV", "0"),
    ("LongTuningKf", "100"),
    ("LongActuatorDelay", "20"),
    ("VEgoStopping", "50"),
    ("RadarReactionFactor", "100"),
    ("EnableRadarTracks", "0"),
    ("EnableCornerRadar", "0"),
    ("HyundaiCameraSCC", "0"),
    ("IsLdwsCar", "0"),
    ("CanfdHDA2", "0"),
    ("CanfdDebug", "0"),
    ("SoundVolumeAdjust", "100"),
    ("SoundVolumeAdjustEngage", "10"),
    ("TFollowGap1", "110"),
    ("TFollowGap2", "120"),
    ("TFollowGap3", "140"),
    ("TFollowGap4", "160"),
    ("DynamicTFollow", "0"),
    ("DynamicTFollowLC", "100"),
    ("HapticFeedbackWhenSpeedCamera", "0"),
    ("UseLaneLineSpeed", "0"),
    ("PathOffset", "0"),
    ("UseLaneLineCurveSpeed", "0"),
    ("AdjustLaneOffset", "0"),
    ("LaneChangeNeedTorque", "0"),
    ("LaneChangeDelay", "0"),
    ("LaneChangeBsd", "0"),
    ("MaxAngleFrames", "89"),
    ("LateralTorqueCustom", "0"),
    ("LateralTorqueAccelFactor", "2500"),
    ("LateralTorqueFriction", "100"),
    ("LateralTorqueKpV", "100"),
    ("LateralTorqueKiV", "10"),
    ("LateralTorqueKf", "100"),
    ("LateralTorqueKd", "0"),
    ("LatMpcPathCost", "100"),
    ("LatMpcMotionCost", "11"),
    ("LatMpcAccelCost", "0"),
    ("LatMpcJerkCost", "4"),
    ("LatMpcSteeringRateCost", "700"),
    ("LatMpcInputOffset", "6"),
    ("LatMpcOutputOffset", "5"),
    ("CustomSteerMax", "0"),
    ("CustomSteerDeltaUp", "0"),
    ("CustomSteerDeltaDown", "0"),
    ("CustomSteerDeltaUpLC", "0"),
    ("CustomSteerDeltaDownLC", "0"),
    ("SpeedFromPCM", "2"),
    ("SteerActuatorDelay", "0"),
    ("MaxTimeOffroadMin", "60"),
    ("DisableDM", "0"),
    ("EnableConnect", "0"),
    ("MuteDoor", "0"),
    ("MuteSeatbelt", "0"),
    ("RecordRoadCam", "0"),
    ("HDPuse", "0"),
    ("CruiseOnDist", "400"),
    ("HotspotOnBoot", "0"),
    ("SoftwareMenu", "1"),
    ("CustomSR", "0"),
    ("SteerRatioRate", "100"),
  ]
  return default_params

def set_default_params():
  params = Params()
  default_params = get_default_params()
  try:
    default_params.remove(("GMapKey", "0"))
    default_params.remove(("CompletedTrainingVersion", "0"))
    default_params.remove(("LanguageSetting", "main_en"))
    default_params.remove(("GsmMetered", "1"))
  except ValueError:
    pass
  for k, v in default_params:
    params.put(k, v)
    print(f"SetToDefault[{k}]={v}")

def get_default_params_key():
  default_params = get_default_params()
  all_keys = [key for key, _ in default_params]
  return all_keys

def manager_init() -> None:
  save_bootlog()

  build_metadata = get_build_metadata()

  params = Params()
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)
  if build_metadata.release_channel:
    params.clear_all(ParamKeyType.DEVELOPMENT_ONLY)

  default_params = get_default_params()

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # Create folders needed for msgq
  try:
    os.mkdir(Paths.shm_path())
  except FileExistsError:
    pass
  except PermissionError:
    print(f"WARNING: failed to make {Paths.shm_path()}")

  # set params
  serial = HARDWARE.get_serial()
  params.put("Version", build_metadata.openpilot.version)
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", build_metadata.openpilot.git_commit)
  params.put("GitCommitDate", build_metadata.openpilot.git_commit_date)
  params.put("GitBranch", build_metadata.channel)
  params.put("GitRemote", build_metadata.openpilot.git_origin)
  params.put_bool("IsTestedBranch", build_metadata.tested_channel)
  params.put_bool("IsReleaseBranch", build_metadata.release_channel)
  params.put("HardwareSerial", serial)

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = build_metadata.openpilot.git_normalized_origin # Needed for swaglog
  os.environ['GIT_BRANCH'] = build_metadata.channel # Needed for swaglog
  os.environ['GIT_COMMIT'] = build_metadata.openpilot.git_commit # Needed for swaglog

  if not build_metadata.openpilot.is_dirty:
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

  # preimport all processes
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: list[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  print_timer = 0

  started_prev = False

  while True:
    sm.update(1000)

    started = sm['deviceState'].started

    if started and not started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)

    # update onroad params, which drives pandad's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print_timer = (print_timer + 1)%10
    if print_timer == 0:
      print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState', valid=True)
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break

def main() -> None:
  manager_init()
  print(f"python ../../opendbc/car/hyundai/values.py > {Params().get_param_path()}/SupportedCars")
  os.system(f"python ../../opendbc/car/hyundai/values.py > {Params().get_param_path()}/SupportedCars")
  os.system(f"python ../../opendbc/car/gm/values.py > {Params().get_param_path()}/SupportedCars_gm")
  os.system(f"python ../../opendbc/car/toyota/values.py > {Params().get_param_path()}/SupportedCars_toyota")
  os.system(f"python ../../opendbc/car/mazda/values.py > {Params().get_param_path()}/SupportedCars_mazda")

  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
