#!/usr/bin/env python3
import datetime
import os
import signal
import subprocess
import sys
import traceback
from multiprocessing import Process

from cereal import log
import cereal.messaging as messaging
import openpilot.selfdrive.sentry as sentry
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.text_window import TextWindow
from openpilot.system.hardware import HARDWARE, PC
from openpilot.selfdrive.manager.helpers import unblock_stdout, write_onroad_params, save_bootlog
from openpilot.selfdrive.manager.process import ensure_running, launcher
from openpilot.selfdrive.manager.process_config import managed_processes
from openpilot.selfdrive.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.version import get_build_metadata, terms_version, training_version

def get_default_params():
  default_params : list[tuple[str, str | bytes]] = [
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("GsmMetered", "1"),
    ("HasAcceptedTerms", "0"),
    ("LanguageSetting", "main_en"),
    ("OpenpilotEnabledToggle", "1"),
    ("LongitudinalPersonality", str(log.LongitudinalPersonality.standard)),
    ("LongitudinalPersonalityMax", "3"),
    ("AlwaysOnLateralEnabled", "1"),
    ("SearchInput", "0"),
    ("GMapKey", "0"),
    ("SteerRatio", "0"),
    ("MuteDoor", "0"),
    ("MuteSeatbelt", "0"),
    ("LongPitch", "0"),
    ("EVTable", "0"),
    ("TSS2Tune", "0"),
    ("ShowDebugUI", "0"),
    ("ShowDateTime", "1"),
    ("ShowHudMode", "4"),
    ("ShowSteerRotate", "1"),
    ("ShowPathEnd", "1"),
    ("ShowAccelRpm", "0"),
    ("ShowTpms", "1"),
    ("ShowSteerMode", "2"),
    ("ShowDeviceState", "1"),
    ("ShowCustomBrightness", "100"),
    ("ShowConnInfo", "1"),
    ("ShowLaneInfo", "1"),
    ("ShowBlindSpot", "1"),
    ("ShowGapInfo", "-1"),
    ("ShowDmInfo", "1"),
    ("ShowRadarInfo", "1"),
    ("MixRadarInfo", "0"),
    ("CarrotTest3", "0"),
    ("ShowPathMode", "9"),
    ("ShowPathColor", "13"),
    ("ShowPathModeCruiseOff", "0"),
    ("ShowPathColorCruiseOff", "19"),
    ("ShowPathModeLane", "14"),
    ("ShowPathColorLane", "13"),
    ("ShowPathWidth", "100"),
    ("ShowPlotMode", "0"),

    ("AutoCruiseControl", "2"),    
    ("AutoResumeFromGasSpeed", "30"),
    ("AutoCancelFromGasMode", "2"),    
    ("AutoResumeFromBrakeReleaseTrafficSign", "0"),
    ("SoftHoldMode", "0"),       

    ("AutoSpeedUptoRoadSpeedLimit", "100"),

    ("MapboxStyle", "0"),    

    ("AutoCurveSpeedLowerLimit", "30"),
    ("AutoCurveSpeedFactor", "120"),
    ("AutoCurveSpeedAggressiveness", "100"),

    ("AutoTurnControl", "0"),

    ("AutoLaneChangeSpeed", "20"),
    ("LaneChangeNeedTorque", "0"),
    ("ManualSteeringOverride", "0"),
    ("LaneChangeLaneCheck", "1"),

    ("AutoTurnControlSpeedLaneChange", "60"),
    ("AutoTurnControlSpeedTurn", "20"),
    ("AutoTurnControlTurnEnd", "6"),
    ("AutoTurnMapChange", "0"),

    ("AutoNaviSpeedCtrl", "1"),
    ("AutoNaviSpeedCtrlEnd", "7"),
    ("AutoNaviSpeedBumpTime", "1"),
    ("AutoNaviSpeedBumpSpeed", "35"),
    ("AutoNaviSpeedSafetyFactor", "105"),
    ("AutoNaviSpeedDecelRate", "200"),

    ("StartAccelApply", "0"),
    ("StopAccelApply", "0"),
    ("StoppingAccel", "-40"),

    ("StopDistanceCarrot", "550"), 
    ("ComfortBrake", "250"), 
    ("TrafficStopDistanceAdjust", "150"), 
    ("ALeadTauPos", "120"), 
    ("ALeadTauNeg", "60"), 
    ("ALeadTauThreshold", "40"), 
    ("CruiseButtonMode", "0"),      
    ("CruiseButtonTest1", "8"),      
    ("CruiseButtonTest2", "30"),      
    ("CruiseButtonTest3", "1"),      
    ("CruiseSpeedUnit", "10"),
    ("MyDrivingMode", "3"),      
    ("MySafeModeFactor", "80"),      
    ("MyEcoModeFactor", "90"),  
    ("CruiseMaxVals1", "200"),
    ("CruiseMaxVals2", "160"),
    ("CruiseMaxVals3", "130"),
    ("CruiseMaxVals4", "110"),
    ("CruiseMaxVals5", "95"),
    ("CruiseMaxVals6", "80"),
    ("CruiseMinVals", "140"),
    ("MyHighModeFactor", "100"),
    ("CruiseSpeedMin", "10"),
    ("LongitudinalTuningKpV", "100"),     
    ("LongitudinalTuningKiV", "0"),     
    ("LongitudinalTuningKf", "100"),     
    ("EnableRadarTracks", "0"),      
    ("EnableAVM", "0"),      
    ("SccConnectedBus2", "0"),
    ("CanfdHDA2", "0"),
    ("SoundVolumeAdjust", "100"),
    ("SoundVolumeAdjustEngage", "10"),
    ("StartRecord", "0"),
    ("StopRecord", "0"),
    ("TFollowSpeedAdd", "10"),
    ("TFollowSpeedAddM", "0"),
    ("TFollowLeadCarSpeed", "0"),
    ("TFollowLeadCarAccel", "0"),
    ("TFollowGap1", "110"),
    ("TFollowGap2", "120"),
    ("TFollowGap3", "140"),
    ("TFollowGap4", "160"),
    ("HapticFeedbackWhenSpeedCamera", "0"),       
    ("CruiseEcoControl", "2"),
    ("UseLaneLineSpeed", "0"),    
    ("UseLaneLineCurveSpeed", "0"),    
    ("UseLaneLineSpeedApply", "0"),    
    ("AdjustLaneOffset", "0"),    
    ("AdjustCurveOffset", "0"),    
    ("AdjustLaneTime", "13"),    
    ("PathOffset", "0"),  
    ("MaxAngleFrames", "89"),       
    ("LateralTorqueCustom", "0"),       
    ("LateralTorqueAccelFactor", "2500"),       
    ("LateralTorqueFriction", "100"),       
    ("CustomSteerMax", "0"),       
    ("CustomSteerDeltaUp", "0"),       
    ("CustomSteerDeltaDown", "0"),       
    ("SpeedFromPCM", "2"),       
    ("MaxTimeOffroadMin", "60"),       
    ("SteerActuatorDelay", "50"),       
    ("CruiseOnDist", "0"),
    ("MSLCEnabled", "0"),
    ("NoLogging", "0"),
    ("HotspotOnBoot", "0"),

    ("MTSCAggressiveness", "100"),
    ("MTSCCurvatureCheck", "1"),
    ("MTSCEnabled", "0"),
    ("NNFF", "0"),
    ("NNFFLite", "0"),
  ]
  return default_params

def set_default_params():
  params = Params()
  default_params = get_default_params()
  try:
    default_params.remove(("GMapKey", "0"))
    defulat_params.remove(("CompletedTrainingVersion", "0"))
    default_params.remove(("LanguageSetting", "main_en"))
    default_params.remove(("GsmMetered", "1"))
  except ValueError:
    pass
  for k, v in default_params:
    params.put(k, v)
    print(f"SetToDefault[{k}]={v}")

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

  if not PC:
    default_params.append(("LastUpdateTime", datetime.datetime.utcnow().isoformat().encode('utf8')))

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  # set version params
  params.put("Version", build_metadata.openpilot.version)
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", build_metadata.openpilot.git_commit)
  params.put("GitCommitDate", build_metadata.openpilot.git_commit_date)
  params.put("GitBranch", build_metadata.channel)
  params.put("GitRemote", build_metadata.openpilot.git_origin)
  params.put_bool("IsTestedBranch", build_metadata.tested_channel)
  params.put_bool("IsReleaseBranch", build_metadata.release_channel)

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    serial = params.get("HardwareSerial")
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

def is_running_on_wsl2():
  try:
    with open('/proc/version', 'r') as f:
      contents = f.read()
      return 'WSL2' in contents or 'Ubuntu' in contents
  except FileNotFoundError:
    return False

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

  #if params.get_bool("UseExternalNaviRoutes"):
  #  ignore += ["navd"]
  #elif not params.get_bool("UseExternalNaviRoutes"):
  #  ignore += ["navi_route"]
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

    # update onroad params, which drives boardd's safety setter thread
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
      if params.get_bool(param) and not is_running_on_wsl2():
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def main() -> None:
  manager_init()
  os.system("python /data/openpilot/selfdrive/car/hyundai/values.py > /data/params/d/SupportedCars")
  os.system("python /data/openpilot/selfdrive/car/gm/values.py > /data/params/d/SupportedCars_gm")
  os.system("python /data/openpilot/selfdrive/car/toyota/values.py > /data/params/d/SupportedCars_toyota")

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
