#include "selfdrive/common/params.h"

#include <dirent.h>
#include <sys/file.h>

#include <csignal>
#include <unordered_map>

#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"

namespace {

volatile sig_atomic_t params_do_exit = 0;
void params_sig_handler(int signal) {
  params_do_exit = 1;
}

int fsync_dir(const std::string &path) {
  int result = -1;
  int fd = HANDLE_EINTR(open(path.c_str(), O_RDONLY, 0755));
  if (fd >= 0) {
    result = fsync(fd);
    close(fd);
  }
  return result;
}

bool create_params_path(const std::string &param_path, const std::string &key_path) {
  // Make sure params path exists
  if (!util::file_exists(param_path) && !util::create_directories(param_path, 0775)) {
    return false;
  }

  // See if the symlink exists, otherwise create it
  if (!util::file_exists(key_path)) {
    // 1) Create temp folder
    // 2) Symlink it to temp link
    // 3) Move symlink to <params>/d

    std::string tmp_path = param_path + "/.tmp_XXXXXX";
    // this should be OK since mkdtemp just replaces characters in place
    char *tmp_dir = mkdtemp((char *)tmp_path.c_str());
    if (tmp_dir == NULL) {
      return false;
    }

    std::string link_path = std::string(tmp_dir) + ".link";
    if (symlink(tmp_dir, link_path.c_str()) != 0) {
      return false;
    }

    // don't return false if it has been created by other
    if (rename(link_path.c_str(), key_path.c_str()) != 0 && errno != EEXIST) {
      return false;
    }
  }

  return true;
}

std::string ensure_params_path(const std::string &path = {}) {
  std::string params_path = path.empty() ? Path::params() : path;
  if (!create_params_path(params_path, params_path + "/d")) {
    throw std::runtime_error(util::string_format("Failed to ensure params path, errno=%d", errno));
  }
  return params_path;
}

class FileLock {
public:
  FileLock(const std::string &fn) {
    fd_ = HANDLE_EINTR(open(fn.c_str(), O_CREAT, 0775));
    if (fd_ < 0 || HANDLE_EINTR(flock(fd_, LOCK_EX)) < 0) {
      LOGE("Failed to lock file %s, errno=%d", fn.c_str(), errno);
    }
  }
  ~FileLock() { close(fd_); }

private:
  int fd_ = -1;
};

std::unordered_map<std::string, uint32_t> keys = {
    {"AccessToken", CLEAR_ON_MANAGER_START | DONT_LOG},
    {"AssistNowToken", PERSISTENT},
    {"AthenadPid", PERSISTENT},
    {"AthenadUploadQueue", PERSISTENT},
    {"CalibrationParams", PERSISTENT},
    {"CameraDebugExpGain", CLEAR_ON_MANAGER_START},
    {"CameraDebugExpTime", CLEAR_ON_MANAGER_START},
    {"CarBatteryCapacity", PERSISTENT},
    {"CarParams", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"CarParamsCache", CLEAR_ON_MANAGER_START},
    {"CarParamsPersistent", PERSISTENT},
    {"CarVin", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"CellularUnmetered", PERSISTENT},
    {"CompletedTrainingVersion", PERSISTENT},
    {"ControlsReady", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"CurrentRoute", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"DashcamOverride", PERSISTENT},
    {"DisableLogging", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"DisablePowerDown", PERSISTENT},
    {"ExperimentalMode", PERSISTENT},
    {"ExperimentalModeConfirmed", PERSISTENT},
    {"ExperimentalLongitudinalEnabled", PERSISTENT}, // WARNING: THIS MAY DISABLE AEB
    {"DisableUpdates", PERSISTENT},
    {"DisengageOnAccelerator", PERSISTENT},
    {"DongleId", PERSISTENT},
    {"DoReboot", CLEAR_ON_MANAGER_START},
    {"DoShutdown", CLEAR_ON_MANAGER_START},
    {"DoUninstall", CLEAR_ON_MANAGER_START},
    {"EnableWideCamera", CLEAR_ON_MANAGER_START},
    {"EndToEndToggle", PERSISTENT},
    {"ForcePowerDown", CLEAR_ON_MANAGER_START},
    {"GitBranch", PERSISTENT},
    {"GitCommit", PERSISTENT},
    {"GitDiff", PERSISTENT},
    {"GithubSshKeys", PERSISTENT},
    {"GithubUsername", PERSISTENT},
    {"GitRemote", PERSISTENT},
    {"GsmApn", PERSISTENT},
    {"GsmMetered", PERSISTENT},
    {"GsmRoaming", PERSISTENT},
    {"HardwareSerial", PERSISTENT},
    {"HasAcceptedTerms", PERSISTENT},
    {"IMEI", PERSISTENT},
    {"InstallDate", PERSISTENT},
    {"IsDriverViewEnabled", CLEAR_ON_MANAGER_START},
    {"IsEngaged", PERSISTENT},
    {"IsLdwEnabled", PERSISTENT},
    {"IsMetric", PERSISTENT},
    {"IsOffroad", CLEAR_ON_MANAGER_START},
    {"IsOnroad", PERSISTENT},
    {"IsRHD", PERSISTENT},
    {"IsRhdDetected", PERSISTENT},
    {"IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"IsTestedBranch", CLEAR_ON_MANAGER_START},
    {"IsUpdateAvailable", CLEAR_ON_MANAGER_START},
    {"JoystickDebugMode", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_OFF},
    {"LaikadEphemeris", PERSISTENT | DONT_LOG},
    {"LanguageSetting", PERSISTENT},
    {"LastAthenaPingTime", CLEAR_ON_MANAGER_START},
    {"LastGPSPosition", PERSISTENT},
    {"LastManagerExitReason", CLEAR_ON_MANAGER_START},
    {"LastPeripheralPandaType", PERSISTENT},
    {"LastPowerDropDetected", CLEAR_ON_MANAGER_START},
    {"LastSystemShutdown", CLEAR_ON_MANAGER_START},
    {"LastUpdateException", PERSISTENT},
    {"LastUpdateTime", PERSISTENT},
    {"LiveParameters", PERSISTENT},
    {"LiveTorqueCarParams", PERSISTENT | DONT_LOG},
    {"LiveTorqueParameters", PERSISTENT | DONT_LOG},
    {"NavDestination", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_OFF},
    {"NavSettingTime24h", PERSISTENT},
    {"NavSettingLeftSide", PERSISTENT},
    {"NavdRender", PERSISTENT},
    {"OpenpilotEnabledToggle", PERSISTENT},
    {"PandaHeartbeatLost", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_OFF},
    {"PandaSignatures", CLEAR_ON_MANAGER_START},
    {"Passive", PERSISTENT},
    {"PrimeRedirected", PERSISTENT},
    {"PrimeType", PERSISTENT},
    {"RecordFront", PERSISTENT},
    {"RecordFrontLock", PERSISTENT},  // for the internal fleet
    {"ReleaseNotes", PERSISTENT},
    {"ReplayControlsState", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"ShouldDoUpdate", CLEAR_ON_MANAGER_START},
    {"SnoozeUpdate", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_OFF},
    {"SshEnabled", PERSISTENT},
    {"SubscriberInfo", PERSISTENT},
    {"TermsVersion", PERSISTENT},
    {"Timezone", PERSISTENT},
    {"TrainingVersion", PERSISTENT},
    {"UbloxAvailable", PERSISTENT},
    {"UpdateAvailable", CLEAR_ON_MANAGER_START},
    {"UpdateFailedCount", CLEAR_ON_MANAGER_START},
    {"UpdaterState", CLEAR_ON_MANAGER_START},
    {"UpdaterFetchAvailable", CLEAR_ON_MANAGER_START},
    {"UpdaterTargetBranch", CLEAR_ON_MANAGER_START},
    {"UpdaterAvailableBranches", CLEAR_ON_MANAGER_START},
    {"UpdaterCurrentDescription", CLEAR_ON_MANAGER_START},
    {"UpdaterCurrentReleaseNotes", CLEAR_ON_MANAGER_START},
    {"UpdaterNewDescription", CLEAR_ON_MANAGER_START},
    {"UpdaterNewReleaseNotes", CLEAR_ON_MANAGER_START},
    {"Version", PERSISTENT},
    {"VisionRadarToggle", PERSISTENT},
    {"ApiCache_Device", PERSISTENT},
    {"ApiCache_DriveStats", PERSISTENT},
    {"ApiCache_NavDestinations", PERSISTENT},
    {"ApiCache_Owner", PERSISTENT},
    {"Offroad_BadNvme", CLEAR_ON_MANAGER_START},
    {"Offroad_CarUnrecognized", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"Offroad_ChargeDisabled", CLEAR_ON_MANAGER_START },
    {"Offroad_ConnectivityNeeded", CLEAR_ON_MANAGER_START},
    {"Offroad_ConnectivityNeededPrompt", CLEAR_ON_MANAGER_START},
    {"Offroad_InvalidTime", CLEAR_ON_MANAGER_START},
    {"Offroad_IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"Offroad_NeosUpdate", CLEAR_ON_MANAGER_START},
    {"Offroad_NoFirmware", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"Offroad_StorageMissing", CLEAR_ON_MANAGER_START},
    {"Offroad_TemperatureTooHigh", CLEAR_ON_MANAGER_START},
    {"Offroad_UnofficialHardware", CLEAR_ON_MANAGER_START},
    {"Offroad_UpdateFailed", CLEAR_ON_MANAGER_START},
    {"SelectedCar", PERSISTENT},
    {"SupportedCars", PERSISTENT},
    {"SupportedCars_gm", PERSISTENT},
    {"SoftRestartTriggered", CLEAR_ON_MANAGER_START},
    {"OpkrPrebuiltOn", PERSISTENT},
    {"OPKRServer", PERSISTENT},
    {"OPKRServerAPI", PERSISTENT},
    { "ShowDebugUI", PERSISTENT },
    { "ShowDateTime", PERSISTENT },
    { "ShowHudMode", PERSISTENT },
    { "ShowSteerRotate", PERSISTENT },
    { "ShowPathEnd", PERSISTENT },
    { "ShowAccelRpm", PERSISTENT },
    { "ShowTpms", PERSISTENT },
    { "ShowSteerMode", PERSISTENT },
    { "ShowDeviceState", PERSISTENT },
    { "ShowConnInfo", PERSISTENT },
    { "ShowLaneInfo", PERSISTENT },
    { "ShowBlindSpot", PERSISTENT },
    { "ShowGapInfo", PERSISTENT },
    { "ShowDmInfo", PERSISTENT },
    { "ShowRadarInfo", PERSISTENT },
    { "MixRadarInfo", PERSISTENT },
    { "ShowZOffset", PERSISTENT },
    { "ShowPathMode", PERSISTENT },
    { "ShowPathColor", PERSISTENT },
    { "ShowPathModeCruiseOff", PERSISTENT },
    { "ShowPathColorCruiseOff", PERSISTENT },
    { "ShowPathModeLane", PERSISTENT },
    { "ShowPathColorLane", PERSISTENT },
    { "ShowPathWidth", PERSISTENT },
    { "ShowPlotMode", PERSISTENT },
    { "AutoResumeFromGas", PERSISTENT },
    { "AutoResumeFromGasSpeed", PERSISTENT },
    { "AutoResumeFromGasSpeedMode", PERSISTENT },
    { "AutoCancelFromGasMode", PERSISTENT },
    { "AutoCurveSpeedCtrlUse", PERSISTENT },
    { "AutoCurveSpeedFactor", PERSISTENT },
    { "AutoCurveSpeedFactorIn", PERSISTENT },
    { "AutoTurnControl", PERSISTENT },
    { "AutoTurnSpeed", PERSISTENT },
    { "AutoTurnTimeMax", PERSISTENT },
    { "AutoLaneChangeSpeed", PERSISTENT },
    { "AutoNaviSpeedCtrl", PERSISTENT },
    { "AutoNaviSpeedCtrlStart", PERSISTENT },
    { "AutoNaviSpeedCtrlEnd", PERSISTENT },
    { "AutoRoadLimitCtrl", PERSISTENT },
    { "AutoResumeFromBrakeRelease", PERSISTENT },
    { "AutoResumeFromBrakeReleaseDist", PERSISTENT },
    { "AutoResumeFromBrakeReleaseLeadCar", PERSISTENT },
    { "AutoResumeFromBrakeCarSpeed", PERSISTENT },
    { "AutoResumeFromBrakeReleaseTrafficSign", PERSISTENT },
    { "XEgoObstacleCost", PERSISTENT },
    { "JEgoCost", PERSISTENT },
    { "AChangeCost", PERSISTENT },
    { "DangerZoneCost", PERSISTENT },
    { "LeadDangerFactor", PERSISTENT },
    { "LongControlActiveSound", PERSISTENT },
    { "StopAccelApply", PERSISTENT },
    { "StartAccelApply", PERSISTENT },
    { "TrafficStopDistanceAdjust", PERSISTENT },
    { "AutoSpeedUptoRoadSpeedLimit", PERSISTENT },
    { "ApplyLongDynamicCost", PERSISTENT },
    { "AutoSpeedAdjustWithLeadCar", PERSISTENT },
    { "TrafficStopAccel", PERSISTENT },
    { "TrafficStopModelSpeed", PERSISTENT },
    { "TrafficStopMode", PERSISTENT },
    { "CruiseButtonMode", PERSISTENT },
    { "CruiseSpeedUnit", PERSISTENT },
    { "GapButtonMode", PERSISTENT },
    { "PrevCruiseGap", PERSISTENT },
    { "CruiseSpeedMin", PERSISTENT },
    { "InitMyDrivingMode", PERSISTENT },
    { "MyDrivingMode", PERSISTENT },
    { "MySafeModeFactor", PERSISTENT },
    { "LiveSteerRatioApply", PERSISTENT },
    { "SteerActuatorDelay", PERSISTENT },
    { "CruiseControlMode", PERSISTENT },
    { "CruiseOnDist", PERSISTENT },
    { "SteerRatioApply", PERSISTENT },
    { "MyEcoModeFactor", PERSISTENT },
    { "CruiseMaxVals1", PERSISTENT },
    { "CruiseMaxVals2", PERSISTENT },
    { "CruiseMaxVals3", PERSISTENT },
    { "CruiseMaxVals4", PERSISTENT },
    { "CruiseMaxVals5", PERSISTENT },
    { "CruiseMaxVals6", PERSISTENT },
    { "AutoSyncCruiseSpeedMax", PERSISTENT },
    { "StopDistance", PERSISTENT },
    { "CustomMapbox", PERSISTENT },
    { "CustomMapboxTokenPk", PERSISTENT },
    { "CustomMapboxTokenSk", PERSISTENT },    
    { "EnableGmap", PERSISTENT},
    { "GmapKey", PERSISTENT },
    { "LongitudinalTuningKpV", PERSISTENT },
    { "LongitudinalTuningKiV", PERSISTENT },
    { "LongitudinalActuatorDelayLowerBound", PERSISTENT },
    { "LongitudinalActuatorDelayUpperBound", PERSISTENT },        
    { "EnableRadarTracks", PERSISTENT },
    { "EnableAutoEngage", PERSISTENT },
    { "ApplyDynamicTFollow", PERSISTENT },
    { "ApplyDynamicTFollowApart", PERSISTENT },
    { "ApplyDynamicTFollowDecel", PERSISTENT },
    { "SccConnectedBus2", PERSISTENT },
    { "TFollowRatio", PERSISTENT },
    { "JerkUpperLowerLimit", PERSISTENT },
    { "OPKRTimeZone", PERSISTENT},
    { "KeepEngage", PERSISTENT },
    { "UseLaneLineSpeed", PERSISTENT },
    { "PathOffset", PERSISTENT },
    { "PathCostApply", PERSISTENT },
    { "PathCostApplyLow", PERSISTENT },
    { "HapticFeedbackWhenSpeedCamera", PERSISTENT },
    { "SoftHoldMode", PERSISTENT },
    { "ApplyModelDistOrder", PERSISTENT },
    { "TrafficStopUpdateDist", PERSISTENT },
    { "TrafficDetectBrightness", PERSISTENT },
    { "SteeringRateCost", PERSISTENT },
    { "LateralMotionCost", PERSISTENT },
    { "LateralAccelCost", PERSISTENT },
    { "LateralJerkCost", PERSISTENT },
    { "LateralTorqueKp", PERSISTENT },
    { "LateralTorqueKi", PERSISTENT },
    { "LateralTorqueKd", PERSISTENT },
    { "LateralTorqueKf", PERSISTENT },
    { "LateralTorqueCustom", PERSISTENT },
    { "LateralTorqueAccelFactor", PERSISTENT },
    { "LateralTorqueFriction", PERSISTENT },
    { "SteerDeltaUp", PERSISTENT },
    { "SteerDeltaDown", PERSISTENT },
};

} // namespace

Params::Params(const std::string &path) {
  static std::string default_param_path = ensure_params_path();
  params_path = path.empty() ? default_param_path : ensure_params_path(path);
}

bool Params::checkKey(const std::string &key) {
  return keys.find(key) != keys.end();
}

ParamKeyType Params::getKeyType(const std::string &key) {
  return static_cast<ParamKeyType>(keys[key]);
}

int Params::put(const char* key, const char* value, size_t value_size) {
  // Information about safely and atomically writing a file: https://lwn.net/Articles/457667/
  // 1) Create temp file
  // 2) Write data to temp file
  // 3) fsync() the temp file
  // 4) rename the temp file to the real name
  // 5) fsync() the containing directory
  std::string tmp_path = params_path + "/.tmp_value_XXXXXX";
  int tmp_fd = mkstemp((char*)tmp_path.c_str());
  if (tmp_fd < 0) return -1;

  int result = -1;
  do {
    // Write value to temp.
    ssize_t bytes_written = HANDLE_EINTR(write(tmp_fd, value, value_size));
    if (bytes_written < 0 || (size_t)bytes_written != value_size) {
      result = -20;
      break;
    }

    // fsync to force persist the changes.
    if ((result = fsync(tmp_fd)) < 0) break;

    FileLock file_lock(params_path + "/.lock");

    // Move temp into place.
    if ((result = rename(tmp_path.c_str(), getParamPath(key).c_str())) < 0) break;

    // fsync parent directory
    result = fsync_dir(getParamPath());
  } while (false);

  close(tmp_fd);
  ::unlink(tmp_path.c_str());
  return result;
}

int Params::remove(const std::string &key) {
  FileLock file_lock(params_path + "/.lock");
  int result = unlink(getParamPath(key).c_str());
  if (result != 0) {
    return result;
  }
  return fsync_dir(getParamPath());
}

std::string Params::get(const std::string &key, bool block) {
  if (!block) {
    return util::read_file(getParamPath(key));
  } else {
    // blocking read until successful
    params_do_exit = 0;
    void (*prev_handler_sigint)(int) = std::signal(SIGINT, params_sig_handler);
    void (*prev_handler_sigterm)(int) = std::signal(SIGTERM, params_sig_handler);

    std::string value;
    while (!params_do_exit) {
      if (value = util::read_file(getParamPath(key)); !value.empty()) {
        break;
      }
      util::sleep_for(100);  // 0.1 s
    }

    std::signal(SIGINT, prev_handler_sigint);
    std::signal(SIGTERM, prev_handler_sigterm);
    return value;
  }
}

std::map<std::string, std::string> Params::readAll() {
  FileLock file_lock(params_path + "/.lock");
  return util::read_files_in_dir(getParamPath());
}

void Params::clearAll(ParamKeyType key_type) {
  FileLock file_lock(params_path + "/.lock");

  std::string path;
  for (auto &[key, type] : keys) {
    if (type & key_type) {
      unlink(getParamPath(key).c_str());
    }
  }

  fsync_dir(getParamPath());
}
